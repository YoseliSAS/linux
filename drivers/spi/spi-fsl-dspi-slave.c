// SPDX-License-Identifier: GPL-2.0+
//
// Copyright 2014 Eurogiciel Ingenierie, Inc.
// Copyright 2020 NXP
// Copyright 2025 Yoseli - Jean-Michel Hautbois
//
// Freescale DSPI slave driver
// This file contains a driver for the Freescale DSPI peripheral in slave mode.

#include <linux/cdev.h>
#include <linux/circ_buf.h>
#include <linux/debugfs.h>
#include <linux/dmaengine.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <asm/mcfsim.h>
#include <asm/mcfqspi.h>
#include <asm/coldfire.h>
#include <linux/io.h>
#include <asm/uaccess.h>
#include <linux/dma-mapping.h>

#include <linux/time.h>

#if defined(CONFIG_M5441x)
#include <asm/mcf5441x_dspi.h>
#include <asm/mcf5441x_gpio.h>
#include <linux/spi/spi-fsl-dspi.h>
#include "spi-fsl-dspi-slave.h"
#endif

#define CLASS_NAME "dspi_slave"
#define DEVICE_NAME "rtspi-data"
#define DRIVER_NAME "Coldfire DSPI Slave"

/****************************************************************************/

/*
 * Driver static configurations
 */

/* S0TOS2 signal "polarity" : positive vs. negative logic. */
//#define DSPI_DEBUG_TRACE	1
//#define DSPI_DEBUG_TRACE_FULL	1

/* DSPI regmap access */
static const struct regmap_range dspi_volatile_ranges[] = {
	regmap_reg_range(SPI_MCR, SPI_TCR),
	regmap_reg_range(SPI_SR, SPI_SR),
	regmap_reg_range(SPI_PUSHR, SPI_RXFR3),
};

static const struct regmap_access_table dspi_volatile_table = {
	.yes_ranges	= dspi_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(dspi_volatile_ranges),
};

static const struct regmap_config dspi_regmap_config = {
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.max_register	= 0x88,
	.volatile_table	= &dspi_volatile_table,
};

/****************************************************************************/

static irqreturn_t dspi_interrupt(int, void *);

/* Default timeout to 5s */
#define DSPI_SLAVE_TIMEOUT_MS 5000
#define DSPI_SLAVE_RUNNER_TIMEOUT_MS 8
static ktime_t read_fifo_timeout;

/* Global reference of dynamically allocated "drv_data".
 *
 * TODO: We support only one instance as we manage one entry in /dev.
 * (This should be enhanced to manage multiple driver instance)
 */
static struct driver_data *chrdev_drvdata;

//static DEFINE_SEMAPHORE(rw_lock, 1);

/****************************************************************************/

static inline void sync_s0tos2_set(u8 val)
{
	u8 mask = BIT(3);
	u8 reg;
#ifdef DSPI_DEBUG_TRACE
	trace_printk("DSPI: sync_s0tos2 = %s\n", val ? "0" : "1");
#endif
	/* Set GPIOG3 as output */
	reg = __raw_readb(MCFGPIO_PDDR_G);
	reg |= mask;
	__raw_writeb(reg, MCFGPIO_PDDR_G);

	/* Update the GPIO output register with inverted logic */
	reg = __raw_readb(MCFGPIO_PODR_G);
	if (!val)
		reg |= mask;
	else
		reg &= ~mask;

	__raw_writeb(reg, MCFGPIO_PODR_G);
}

static inline void stat_reset_counters(struct driver_data *drv_data)
{
	drv_data->stat_spi_nbbytes_recv    = 0;
	drv_data->stat_spi_nbbytes_sent    = 0;
	drv_data->stat_rx_hwfifo_overflow  = 0;
	drv_data->stat_rx_kfifo_overflow   = 0;
	drv_data->stat_tx_hwfifo_underflow = 0;
	drv_data->frame_perf.avg_latency   = 0;
	drv_data->frame_perf.max_latency   = 0;
	drv_data->frame_perf.min_latency   = KTIME_MAX;
	drv_data->frame_perf.total_latency = 0;
}

/****************************************************************************/

/*
 * Char device related callback
 */

static u8 chrdev_is_open;

/* read() and write() exchanges between kernel and userspace are based
 * on a fixed buffer length equal to the following constant in bytes.
 */
static const int SPISLAVE_MSG_FIFO_SIZE = 32;

/* Exclusive access to /dev entry */
static DEFINE_MUTEX(chrdev_client_mutex);

static ssize_t memcpy_32to16(void *dest, const void *src, size_t n)
{
	unsigned int i = 0;
	u16 *d = (u16 *)dest;
	u32 *s = (u32 *)src;

	for (i = 0; i < n / 2; i++)
		d[i] = (u16)(s[i] & 0xFFFF);

	return n;
}

static ssize_t memcpy_16to32(void *dest, const void *src, size_t n)
{
	unsigned int i = 0;
	u16 *s = (u16 *)src;
	u32 *d = (u32 *)dest;

	for (i = n >> 1; i > 0; i--)
		*d++ = (u32)(*s++ & 0xFFFF);

	return n;
}

static void reinit_timeout(u32 timeout)
{
	read_fifo_timeout = ms_to_ktime(timeout);
}

/* Use a Wait queue for userspace synchronisation on read() syscall */
static DECLARE_WAIT_QUEUE_HEAD(read_buf_wq);

static void hwfifo_prepare(struct driver_data *drv_data)
{
	/* Set CLR_TXF & CLR_RXF bit : Clear TX & RX fifo */
	regmap_update_bits(drv_data->regmap, SPI_MCR,
			   SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF,
			   SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF);
}

static inline int kfifo_tx_kfifo_to_hw(struct driver_data *drv_data)
{
	unsigned int nb_elem;
	unsigned int total_sent = 0;

	//trace_printk("send %04x\n", drv_data->mmap_buffer[0]);
	for (nb_elem = 0; nb_elem < SPISLAVE_MSG_FIFO_SIZE / 2; nb_elem += 1) {
		regmap_write(drv_data->regmap,
			     SPI_PUSHR,
			     (drv_data->tx_buffer[nb_elem]) & 0xffff);

		total_sent += 2;
		drv_data->stat_spi_nbbytes_sent += 2;
	}

	return total_sent;
}

/****************************************************************************/

/*
 * SPI local functions
 */

static inline int wait_for_txctr(unsigned long timeout_ns)
{
	u32 value;
	u64 start_time, cur_time;

	start_time = ktime_get_raw_fast_ns();

	do {
		if (regmap_read(chrdev_drvdata->regmap, SPI_SR, &value))
			return -EIO;

		/* Success if TXCTR is 0 */
		if (((value >> 12) & 0xF) == 0)
			return 0;

		cur_time = ktime_get_raw_fast_ns();
		if ((cur_time - start_time) >= timeout_ns)
			return -ETIMEDOUT;

		cpu_relax();
	} while (1);
}

static inline void dspi_setup_chip(struct driver_data *drv_data)
{
	regmap_write(drv_data->regmap, SPI_MCR, SPI_MCR_PCSIS(0xff));
	regmap_update_bits(drv_data->regmap, SPI_MCR,
			   SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF,
			   SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF);
	regmap_update_bits(drv_data->regmap, SPI_MCR,
			   SPI_MCR_ROOE, SPI_MCR_ROOE);

	regmap_write(drv_data->regmap, SPI_SR, SPI_SR_CLEAR);
	regmap_write(drv_data->regmap, SPI_CTARE(0),
		     SPI_FRAME_EBITS(16));

	if (drv_data->mode != DSPI_DMA_MODE) {
		regmap_update_bits(drv_data->regmap, SPI_RSER,
				   SPI_RSER_RFDFE | SPI_RSER_TFUFE,
				   SPI_RSER_RFDFE | SPI_RSER_TFUFE);
	} else {
		regmap_write(chrdev_drvdata->regmap, SPI_RSER,
			     SPI_RSER_TFFFE | SPI_RSER_TFFFD |
			     SPI_RSER_RFDFE | SPI_RSER_RFDFD);
	}
}

static void dspi_stop_hw(void)
{
	regmap_write(chrdev_drvdata->regmap, SPI_MCR, 1);
	if (chrdev_drvdata->mode == DSPI_DMA_MODE) {
		if (atomic_read(&chrdev_drvdata->rx_dma_running) ||
		    atomic_read(&chrdev_drvdata->tx_dma_running)) {
			trace_printk("RX DMA running => false\n");
			atomic_set(&chrdev_drvdata->rx_dma_running, 0);
			trace_printk("TX DMA running => false\n");
			atomic_set(&chrdev_drvdata->tx_dma_running, 0);
			dmaengine_terminate_all(chrdev_drvdata->chan_rx);
			dmaengine_terminate_all(chrdev_drvdata->chan_tx);
		}
	}
}

static void dspi_start_hw(void)
{
	/* Reset HW TX & HW RX fifo */
	hwfifo_prepare(chrdev_drvdata);

	/* Enable DSPI */
	regmap_update_bits(chrdev_drvdata->regmap, SPI_MCR, SPI_MCR_HALT, 0);
	dspi_setup_chip(chrdev_drvdata);
}

/****************************************************************************/
/*
 * DMA related functions
 */

static int dspi_slave_dma_setup_channel(struct driver_data *drv_data)
{
	struct dma_chan *chan;
	int ret;

	/* Prepare for RX : */
	chan = dma_request_chan(&drv_data->pdev->dev, "rx");
	if (IS_ERR(chan)) {
		drv_data->chan_rx = NULL;
		ret = PTR_ERR(chan);
		dev_err(&drv_data->pdev->dev, "cannot get the DMA channel: %d\n", ret);
		goto err;
	}
	drv_data->chan_rx = chan;
	drv_data->rx_priority = 15;
	drv_data->chan_rx->private = &drv_data->rx_priority;

	/* Set the RX DMA channel to a high prio */
	__raw_writeb(6, MCFEDMA_IRQ_PRIO0 + drv_data->chan_rx->chan_id);

	/* Prepare for TX : */
	chan = dma_request_chan(&drv_data->pdev->dev, "tx");
	if (IS_ERR(chan)) {
		drv_data->chan_tx = NULL;
		ret = PTR_ERR(chan);
		dev_err(&drv_data->pdev->dev, "cannot get the DMA channel: %d\n", ret);
		goto err_tx;
	}
	drv_data->chan_tx = chan;
	drv_data->tx_priority = 14;
	drv_data->chan_tx->private = &drv_data->tx_priority;

	dev_info(&drv_data->pdev->dev, "DMA channels %d and %d acquired\n",
		 drv_data->chan_rx->chan_id, drv_data->chan_tx->chan_id);

	drv_data->rx_dma_buf = NULL;
	drv_data->tx_dma_buf = NULL;
	drv_data->rx_dma_buf_size = PAGE_SIZE;
	drv_data->tx_dma_buf_size = PAGE_SIZE;

	drv_data->rx_dma_buf = dma_alloc_coherent(drv_data->chan_rx->device->dev,
						  drv_data->rx_dma_buf_size, &drv_data->rx_dma_phys,
						  GFP_KERNEL);
	if (!drv_data->rx_dma_buf) {
		ret = -ENOMEM;
		goto err_rx_dma_buf;
	}

	drv_data->tx_dma_buf = dma_alloc_coherent(drv_data->chan_tx->device->dev,
						  drv_data->tx_dma_buf_size, &drv_data->tx_dma_phys,
						  GFP_KERNEL);
	if (!drv_data->tx_dma_buf) {
		ret = -ENOMEM;
		goto err_tx_dma_buf;
	}

	dev_info(&drv_data->pdev->dev, "RX buffer allocated at %pad, dma_buf: %pad, size: %d\n",
		 &drv_data->rx_dma_buf,
		 &drv_data->rx_dma_phys,
		  drv_data->rx_dma_buf_size);
	dev_info(&drv_data->pdev->dev, "TX buffer allocated at %pad, dma_buf: %pad, size: %d\n",
		 &drv_data->tx_dma_buf,
		 &drv_data->tx_dma_phys,
		  drv_data->tx_dma_buf_size);

	return 0;

err_tx_dma_buf:
	dma_free_coherent(drv_data->chan_rx->device->dev, drv_data->rx_dma_buf_size,
			  drv_data->rx_dma_buf, drv_data->rx_dma_phys);
err_rx_dma_buf:
	dma_release_channel(drv_data->chan_tx);
err_tx:
	dma_release_channel(drv_data->chan_rx);
err:
	return ret;
}

static int dspi_slave_dma_setup_rx(struct driver_data *drv_data)
{
	struct dma_slave_config slave_config = {};
	struct device *dev = &drv_data->pdev->dev;
	int ret;

	drv_data->rx_period_length = SPISLAVE_MSG_FIFO_SIZE / 2 * DMA_SLAVE_BUSWIDTH_4_BYTES;

	memset(&slave_config, 0, sizeof(slave_config));
	slave_config.direction = DMA_DEV_TO_MEM;
	slave_config.src_addr = drv_data->dspi_base + SPI_POPR;
	slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	slave_config.src_maxburst = 1;
	slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	slave_config.dst_maxburst = 1;

	ret = dmaengine_slave_config(drv_data->chan_rx, &slave_config);
	if (ret) {
		dev_err(dev, "error in RX dma configuration.\n");
		goto err;
	}
#ifdef DSPI_DEBUG_TRACE
	trace_printk("RX DMA setup done\n");
#endif

	return 0;
err:
	dma_release_channel(drv_data->chan_rx);
	return ret;
}

static int dspi_slave_dma_setup_tx(struct driver_data *drv_data)
{
	struct dma_slave_config slave_config = {};
	struct device *dev = &drv_data->pdev->dev;
	int ret;

	drv_data->tx_period_length = SPISLAVE_MSG_FIFO_SIZE / 2 * DMA_SLAVE_BUSWIDTH_4_BYTES;

	memset(&slave_config, 0, sizeof(slave_config));
	slave_config.direction = DMA_MEM_TO_DEV;
	slave_config.dst_addr = drv_data->dspi_base + SPI_PUSHR;
	slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	slave_config.dst_maxburst = 1;
	slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	slave_config.src_maxburst = 1;

	ret = dmaengine_slave_config(drv_data->chan_tx, &slave_config);
	if (ret) {
		dev_err(dev, "error in TX dma configuration.\n");
		goto err;
	}

#ifdef DSPI_DEBUG_TRACE
	trace_printk("TX DMA setup done\n");
#endif

	return 0;
err:
	dma_release_channel(drv_data->chan_rx);
	return ret;
}

static int dspi_slave_dma_setup(struct driver_data *drv_data)
{
	int ret;

	ret = dspi_slave_dma_setup_rx(drv_data);
	if (ret)
		return ret;

	ret = dspi_slave_dma_setup_tx(drv_data);
	if (ret)
		return ret;

	return 0;
}

int dspi_slave_next_xfer_rx_dma(struct driver_data *drv_data)
{
	struct dma_chan	*chan = drv_data->chan_rx;
	struct device *dev = &drv_data->pdev->dev;
	dma_addr_t current_phys;
	size_t transfer_size;
	int ret;

	if (drv_data->mode != DSPI_DMA_MODE)
		return -EINVAL;

	if (atomic_read(&drv_data->rx_dma_running) == 1) {
		// Already running, do nothing
		trace_printk("RX DMA already running\n");
		return 0;
	}

	current_phys = drv_data->rx_dma_phys + drv_data->rx_dma_buf_offset;
	transfer_size = drv_data->rx_period_length;

	//regmap_update_bits(drv_data->regmap, SPI_SR, SPI_SR_RFDF, 1);

#ifdef DSPI_DEBUG_TRACE
	trace_printk("RX DMA: current_phys = %pad, transfer_size = %d\n",
		     &current_phys, transfer_size);
#endif
	drv_data->rx_desc =  dmaengine_prep_slave_single(chan,
							 current_phys,
							 transfer_size,
							 DMA_DEV_TO_MEM,
							 DMA_PREP_INTERRUPT);

	if (!drv_data->rx_desc) {
		dev_err(dev, "RX DMA preparation error\n");
		ret = -EINVAL;
		goto err;
	}

	drv_data->rx_desc->callback = dspi_slave_dma_rx_callback;
	drv_data->rx_desc->callback_param = drv_data;

	drv_data->rx_cookie = dmaengine_submit(drv_data->rx_desc);
	if (dma_submit_error(drv_data->rx_cookie)) {
		dev_err(dev, "RX DMA submit error\n");
		ret = -EINVAL;
		goto err;
	}

#ifdef DSPI_DEBUG_TRACE
	trace_printk("Reinit RX completion\n");
#endif
	reinit_completion(&chrdev_drvdata->read_complete);
	dma_async_issue_pending(chan);

#ifdef DSPI_DEBUG_TRACE
		trace_printk("RX DMA running => true\n");
#endif
	atomic_set(&drv_data->rx_dma_running, 1);

#ifdef DSPI_DEBUG_TRACE
	trace_printk("Prepared RX DMA, rx_cookie = %d, desc @ %p\n",
		     drv_data->rx_cookie, drv_data->rx_desc);
#endif
	return 0;
err:
	return ret;
}

int dspi_slave_next_xfer_tx_dma(struct driver_data *drv_data)
{
	struct dma_chan	*chan = drv_data->chan_tx;
	struct device *dev = &drv_data->pdev->dev;
	dma_addr_t current_phys = drv_data->tx_dma_phys + drv_data->tx_dma_buf_offset;
	size_t transfer_size = drv_data->tx_period_length;
	u32 nb_bytes;
	unsigned long flags;

	if (drv_data->mode != DSPI_DMA_MODE)
		return -EINVAL;
#if 0
	if (atomic_read(&drv_data->tx_dma_running) == 1) {
		// Already running, do nothing
		trace_printk("TX DMA already running\n");
		return 0;
	}
#endif
	raw_spin_lock_irqsave(&chrdev_drvdata->lock, flags);
	nb_bytes = memcpy_16to32((void *)current_phys,
				 chrdev_drvdata->tx_buffer,
				 chrdev_drvdata->tx_period_length);
	raw_spin_unlock_irqrestore(&chrdev_drvdata->lock, flags);

#ifdef DSPI_DEBUG_TRACE_FULL
	// Dump the content into a buffer to be displayed with printk
	trace_print_hex_dump("tx: ", DUMP_PREFIX_ADDRESS, 32, 4,
			     phys_buf,
			     chrdev_drvdata->tx_period_length,
			     false);
#endif

#ifdef DSPI_DEBUG_TRACE
	trace_printk("TX DMA: current_phys = %pad, transfer_size = %d\n",
		     &current_phys, transfer_size);
#endif
	drv_data->tx_desc =  dmaengine_prep_slave_single(drv_data->chan_tx,
							 current_phys,
							 transfer_size,
							 DMA_MEM_TO_DEV,
							 DMA_PREP_INTERRUPT);
	if (!drv_data->tx_desc) {
		dev_err(dev, "TX DMA preparation error\n");
		return -EINVAL;
	}

	drv_data->tx_desc->callback = dspi_slave_dma_tx_callback;
	drv_data->tx_desc->callback_param = drv_data;

	drv_data->tx_cookie = dmaengine_submit(drv_data->tx_desc);
	if (dma_submit_error(drv_data->tx_cookie)) {
		dev_err(dev, "TX DMA submit error\n");
		devm_kfree(dev, drv_data->tx_dma_buf);
		return -EINVAL;
	}

	reinit_completion(&chrdev_drvdata->write_complete);
	dma_async_issue_pending(chan);

#ifdef DSPI_DEBUG_TRACE
	trace_printk("TX DMA running => true\n");
#endif
	atomic_set(&drv_data->tx_dma_running, 1);
#ifdef DSPI_DEBUG_TRACE
	trace_printk("Prepared TX DMA, tx_cookie = %d, desc @ %p\n",
		     drv_data->tx_cookie, drv_data->tx_desc);
#endif

	return 0;
}

void dspi_slave_dma_tx_callback(void *data)
{
	struct driver_data *drv_data = (struct driver_data *)data;
	struct dma_chan	*chan = drv_data->chan_tx;
	struct dma_tx_state state;
	enum dma_status status;

#ifdef DSPI_DEBUG_TRACE
	trace_printk("TX DMA running => false\n");
#endif
	atomic_set(&drv_data->tx_dma_running, 0);

	status = dmaengine_tx_status(chan, drv_data->tx_cookie, &state);

	if (status == DMA_ERROR) {
		trace_printk("error: %08x\n", __raw_readl(0xfc044004));
		goto done;
	}

	chrdev_drvdata->tx_dma_buf_offset += chrdev_drvdata->tx_period_length;
	if (chrdev_drvdata->tx_dma_buf_offset >= chrdev_drvdata->tx_dma_buf_size)
		chrdev_drvdata->tx_dma_buf_offset = 0;

done:
	complete(&drv_data->write_complete);

#ifdef DSPI_DEBUG_TRACE
	trace_printk("TX DMA callback done, cookie: %d\n", state.last);
#endif
}

void dspi_slave_dma_rx_callback(void *data)
{
	struct driver_data *drv_data = (struct driver_data *)data;
	struct dma_chan	*chan = drv_data->chan_rx;
	struct dma_tx_state state;
	enum dma_status status;
	unsigned int r_bytes = drv_data->rx_period_length;
	unsigned int w_bytes = 0;
	void *phys_buf = (void *)(drv_data->rx_dma_phys + drv_data->rx_dma_buf_offset);
	unsigned long flags;

#ifdef DSPI_DEBUG_TRACE
	trace_printk("RX DMA running => false\n");
#endif
	atomic_set(&drv_data->rx_dma_running, 0);

	drv_data->frame_perf.irq_received = ktime_get_raw_fast_ns();
	drv_data->frame_perf.frame_number++;

	reinit_timeout(DSPI_SLAVE_RUNNER_TIMEOUT_MS);

	status = dmaengine_tx_status(chan, drv_data->rx_cookie, &state);

	if (status == DMA_ERROR) {
		trace_printk("error: %08x, cookie=%d\n",
			     __raw_readl(0xfc044004), drv_data->rx_cookie);
		goto done;
	}

	raw_spin_lock_irqsave(&chrdev_drvdata->lock, flags);
	w_bytes = memcpy_32to16(drv_data->rx_buffer,
				phys_buf,
				r_bytes / 2);
	raw_spin_unlock_irqrestore(&chrdev_drvdata->lock, flags);
	drv_data->stat_spi_nbbytes_recv += w_bytes;

	drv_data->rx_dma_buf_offset += drv_data->rx_period_length;
	if (drv_data->rx_dma_buf_offset >= drv_data->rx_dma_buf_size)
		drv_data->rx_dma_buf_offset = 0;

done:
	/* Get ready for next transaction */
#ifdef DSPI_DEBUG_TRACE
	trace_printk("RX DMA callback done, residue=%d\n", state.residue);
#endif
	complete(&drv_data->read_complete);

	return;
}

static inline void update_stats(struct driver_data *drv_data)
{
	if (drv_data->frame_perf.frame_number > 1) {
		u64 latency = drv_data->frame_perf.wait_next_frame - drv_data->frame_perf.irq_received;

		drv_data->frame_perf.max_latency = max(drv_data->frame_perf.max_latency, latency);
		drv_data->frame_perf.min_latency = min(drv_data->frame_perf.min_latency, latency);
		drv_data->frame_perf.latency = drv_data->frame_perf.latency + latency;
		drv_data->frame_perf.total_latency += latency;

		/* Compute average latency */
		u64 tmp = drv_data->frame_perf.total_latency;
		do_div(tmp, drv_data->frame_perf.frame_number);
		drv_data->frame_perf.avg_latency = tmp;

		/* Update the latency output if:
		 * - we received 512 frames
		 * - we have a new max latency
		 */
		if ((drv_data->frame_perf.frame_number % 512 == 0) ||
		    (drv_data->frame_perf.max_latency == latency)) {
			/* Display as cyclictest C:    600 Min:    115 Avg:  128 Max:     251 */
			trace_printk("C: %8u Min: %8llu Avg: %8llu Max: %8llu\n",
				     drv_data->frame_perf.frame_number,
				     drv_data->frame_perf.min_latency / 1024,
				     drv_data->frame_perf.latency / 512 / 1024,
				     drv_data->frame_perf.max_latency / 1024);
			drv_data->frame_perf.latency = 0;
		}
	}
}

static ssize_t chrdev_device_read(struct file *filp,
				  char __user *buffer,
				  size_t length,
				  loff_t *offset)
{
	int status;
	unsigned int nb_bytes;

#ifdef DSPI_DEBUG_TRACE
	trace_printk("Wait for next frame\n");
#endif

	/* Buffer size must be at least equal to kfifo size */
	if (length < SPISLAVE_MSG_FIFO_SIZE) {
		return -EINVAL;
	}

	/* Check length asked not exceed internal kfifo.
	 * This way we can synchronize user apps treatment on kfifo
	 * flush as well */
	if (length > SPISLAVE_MSG_FIFO_SIZE) {
		length = SPISLAVE_MSG_FIFO_SIZE;
	}

	chrdev_drvdata->frame_perf.wait_next_frame = ktime_get_raw_fast_ns();
#ifdef DSPI_DEBUG_TRACE
	trace_printk("wait for finished TX\n");
	trace_printk("Reinit TX completion\n");
#endif
#if 0
	/* If TX is not finished, we will have an issue ! */
	status = wait_for_completion_interruptible(&chrdev_drvdata->write_complete);
	if (status == -ERESTARTSYS) {
		trace_printk("Signal received\n");
		return 0;
	}
#endif

#ifdef DSPI_DEBUG_TRACE
	trace_printk("wait for next RX\n");
#endif

	update_stats(chrdev_drvdata);
	/* If RX Fifo is not full, we block the caller */
	status = wait_for_completion_interruptible_timeout(&chrdev_drvdata->read_complete,
							   msecs_to_jiffies(ktime_to_ms(read_fifo_timeout)));

	if (status == 0) {
#ifndef CONFIG_TRACING
		printk_once(KERN_ERR "DSPI: timeout occurred on read() syscall\n");
#else
		trace_printk("Timeout on read() syscall, received %u frames\n",
			     chrdev_drvdata->frame_perf.frame_number);
#endif
		return -EAGAIN;
	}

	if (status < 0) {
		trace_printk("Signal received\n");
		if (chrdev_drvdata->mode == DSPI_DMA_MODE) {
			trace_printk("RX DMA running => false\n");
			atomic_set(&chrdev_drvdata->rx_dma_running, 0);
			dmaengine_terminate_all(chrdev_drvdata->chan_rx);
		}
		return 0;
	}

	/* Data are ready, now we send them to the caller */
	nb_bytes = chrdev_drvdata->tx_period_length / 2;

	status = __copy_to_user_inatomic(buffer, chrdev_drvdata->rx_buffer, nb_bytes);

#ifdef DSPI_DEBUG_TRACE
	if (status)
		trace_printk("Error while copying data to user: %d\n", status);
	trace_printk("Copied %d bytes to user\n", status ? status : nb_bytes);
#endif

	dspi_slave_next_xfer_rx_dma(chrdev_drvdata);
	return status ? status : nb_bytes;
}

static ssize_t chrdev_device_write(struct file *filp,
	const char __user *buffer, size_t length, loff_t *offset)
{
	int status;
	unsigned int nb_bytes;
	u8 txctr;

#ifdef DSPI_DEBUG_TRACE
	trace_printk("Write frame\n");
#endif
	if (chrdev_drvdata->mode != DSPI_DMA_MODE) {
		status = __copy_from_user_inatomic(chrdev_drvdata->tx_buffer, buffer, length);

		dspi_slave_next_xfer_tx_dma(chrdev_drvdata);
	} else {
		/* Check length is SPISLAVE_MSG_FIFO_SIZE */
		if (unlikely(length != SPISLAVE_MSG_FIFO_SIZE)) {
			return -EINVAL;
		}

		txctr = wait_for_txctr(1000000);

		if (txctr != 0) {
			trace_printk("HW TX fifo not empty, drain\n");
			/* Drain TX fifo */
			regmap_update_bits(chrdev_drvdata->regmap, SPI_MCR, SPI_MCR_CLR_TXF, SPI_MCR_CLR_TXF);
			return -EIO;
		}

		preempt_disable();
		reinit_completion(&chrdev_drvdata->write_complete);

		/* Push whole buffer in TX fifo */
		status = __copy_from_user_inatomic(chrdev_drvdata->tx_buffer, buffer, length);

		/* If HW TX FIFO is empty, we transfer datas from
		 * kfifo to HW FIFO.
		 */
		if (status == 0)
			nb_bytes = kfifo_tx_kfifo_to_hw(chrdev_drvdata);

		complete(&chrdev_drvdata->write_complete);
		preempt_enable();
	}
	chrdev_drvdata->frame_perf.frame_sent = ktime_get_raw_fast_ns();
#ifdef DSPI_DEBUG_TRACE
	trace_printk("Write %d bytes to HW FIFO\n", nb_bytes / 2);
#endif

	return status ? status : nb_bytes;
}

static void sync_spi_hw(void)
{
	sync_s0tos2_set(false);
	dspi_stop_hw();

	dspi_slave_dma_setup(chrdev_drvdata);
}

static int chrdev_device_fsync(struct file *filp, loff_t start, loff_t end, int datasync)
{
	reinit_timeout(DSPI_SLAVE_TIMEOUT_MS);
#ifdef DSPI_DEBUG_TRACE
		trace_printk("fsync\n");
#endif

	if (!regmap_test_bits(chrdev_drvdata->regmap, SPI_MCR, SPI_MCR_HALT)) {
		regmap_update_bits(chrdev_drvdata->regmap, SPI_MCR, SPI_MCR_HALT, 1);
		dmaengine_terminate_all(chrdev_drvdata->chan_rx);
		dmaengine_terminate_all(chrdev_drvdata->chan_tx);
		atomic_set(&chrdev_drvdata->rx_dma_running, 0);
		atomic_set(&chrdev_drvdata->tx_dma_running, 0);
		reinit_completion(&chrdev_drvdata->read_complete);
		reinit_completion(&chrdev_drvdata->write_complete);

		chrdev_drvdata->rx_dma_buf_offset = 0;
		chrdev_drvdata->tx_dma_buf_offset = 0;
	}

	chrdev_drvdata->frame_perf.latency = 0;
	chrdev_drvdata->frame_perf.max_latency = 0;
	chrdev_drvdata->frame_perf.min_latency = KTIME_MAX;
	chrdev_drvdata->frame_perf.frame_number = 0;

	dspi_setup_chip(chrdev_drvdata);

	sync_spi_hw();

	dspi_start_hw();

	dspi_slave_next_xfer_rx_dma(chrdev_drvdata);

	return 0;
}

/*
 * /dev callbacks
 */

static int dspi_error_task(void *data)
{
	struct driver_data *drv_data = (struct driver_data *)data;
	struct sched_param sp = { .sched_priority = 98 };

	sched_setscheduler(current, SCHED_FIFO, &sp);

	while (!kthread_should_stop()) {
		wait_for_completion_interruptible(&drv_data->read_error_complete);

#ifdef DSPI_DEBUG_TRACE
		trace_printk("DSPI: restart\n");
#endif

		sync_spi_hw();

		dspi_start_hw();
	}

	return 0;
}

static int chrdev_device_open(struct inode *inode, struct file *filp)
{
	/* Ensure that only one process has access to our device
	 * at any one time
	 */
	if (!mutex_trylock(&chrdev_client_mutex)) {
		printk(KERN_DEBUG "DSPI: Try to open char device more than once.\n");
		return -EBUSY;
	}

	chrdev_is_open = true;

	stream_open(inode, filp);

	filp->private_data = inode->i_private;

#ifdef DSPI_DEBUG_TRACE
	trace_printk("Init completions\n");
#endif

	init_completion(&chrdev_drvdata->read_complete);
	init_completion(&chrdev_drvdata->write_complete);

	if (chrdev_drvdata->mode != DSPI_DMA_MODE) {
		init_completion(&chrdev_drvdata->read_error_complete);

#ifdef DSPI_DEBUG_TRACE
		trace_printk("Create dspi error task thread\n");
#endif
		chrdev_drvdata->read_error_task = kthread_run(dspi_error_task, chrdev_drvdata, "dspi_error_task");
		if (IS_ERR(chrdev_drvdata->read_error_task)) {
			mutex_unlock(&chrdev_client_mutex);
			return PTR_ERR(chrdev_drvdata->read_error_task);
		}
	}

	trace_printk("open\n");
	return 0;
}

static int chrdev_device_close(struct inode *inode, struct file *filp)
{
	chrdev_is_open = false;

	sync_spi_hw();

	if (chrdev_drvdata->read_error_task) {
		kthread_stop(chrdev_drvdata->read_error_task);
		chrdev_drvdata->read_error_task = NULL;
	}

	mutex_unlock(&chrdev_client_mutex);
	return 0;
}

static const struct file_operations chrdev_fops = {
	.read = chrdev_device_read,
	.write = chrdev_device_write,
	.open = chrdev_device_open,
	.release = chrdev_device_close,
	.fsync = chrdev_device_fsync,
};

static ssize_t debugfs_write_sync_s0tos2(struct file *file,
					 const char *buffer,
					 size_t count,
					 loff_t *data)
{
	/* Allowed strings are "0\n" and "1\n" */
	if (count != 2) {
		return -EINVAL;
	}

	switch (buffer[0]) {
	case '0':
		sync_s0tos2_set(0);
		/* printk(KERN_DEBUG "DSPI: sync_s0tos2 = 0\n"); */
		break;
	case '1':
		sync_s0tos2_set(1);
		/* printk(KERN_DEBUG "DSPI: sync_s0tos2 = 1\n"); */
		break;
	default:
		return -EINVAL;
	}

	return count;
}

/****************************************************************************/

/*
 * IRQ handler
 */

static irqreturn_t dspi_interrupt(int irq, void *dev_id)
{
	u8 restart_dspi = false;
	struct driver_data *drv_data = (struct driver_data *)dev_id;
	unsigned long flags;

	regmap_read(drv_data->regmap, SPI_SR, &drv_data->irq_status);

	local_lock_irqsave(&drv_data->lock, flags);

	reinit_timeout(DSPI_SLAVE_RUNNER_TIMEOUT_MS);
#if 0
	if (chrdev_drvdata->irq_status & MCF_DSPI_DSR_RFOF) {
		/* RX HW FIFO overflow occurs */
		drv_data->stat_rx_hwfifo_overflow += 1;
#ifndef CONFIG_TRACING
		dev_dbg(&drv_data->pdev->dev,
			"DSPI: dspi-slave: RX FIFO overflow occurs (occur. #%llu) !\n",
			drv_data->stat_rx_hwfifo_overflow);
#else
		trace_printk("RX FIFO overflow\n");
#endif
		restart_dspi = true;
	}
 #endif
	if (drv_data->irq_status & MCF_DSPI_DSR_TFUF) {
		/* TX HW FIFO underflow occurs */
		drv_data->stat_tx_hwfifo_underflow += 1;

#ifndef CONFIG_TRACING
		dev_dbg(&drv_data->pdev->dev,
			"DSPI: dspi-slave: TX FIFO underflow occurs (occur. #%llu) !\n",
			drv_data->stat_tx_hwfifo_underflow);
#else
		trace_printk("TX FIFO underflow, last interrupt received at %lld, last frame sent at %lld\n",
			     ktime_to_ns(drv_data->frame_perf.irq_received),
			     ktime_to_ns(drv_data->frame_perf.frame_sent));
#endif
		restart_dspi = true;
	}

	if (!restart_dspi && !chrdev_is_open) {
		/* This should never happen (except if userspace app crash ?) */
		printk_once(KERN_ERR "DSPI: SPI slave data arrived while no process open the device\n");

		restart_dspi = true;
	}

	/* SPI input word arrived in RX FIFO */
	if (!restart_dspi && (drv_data->irq_status & MCF_DSPI_DSR_RFDF)) {
		/* When we enter here, we have data in hardware RX FIFO */
		u32 in_data;
		int i = 0;

		/* While datas are available in HW RX FIFO, we drain it. */
		while (i < SPISLAVE_MSG_FIFO_SIZE / 2) {
			/* We pop one element from the hardware RX FIFO */
			/* Copy to the mmap'ed buffer
			 */
			regmap_read(drv_data->regmap, SPI_POPR, &in_data);
			drv_data->rx_buffer[i] = in_data;

			/* RFDF must be cleared only after the DSPI_POPR
			 * register is read.
			 */
			regmap_update_bits(drv_data->regmap, SPI_SR, SPI_SR_RFDF, 1);

			/* Increment stat incoming bytes counter */
			drv_data->stat_spi_nbbytes_recv += 2;
			i++;
		}
		drv_data->frame_perf.irq_received = ktime_get_raw_fast_ns();
		drv_data->frame_perf.frame_number++;
	}

	/*
	 * FIFO overflow and underflow management
	 */
	if (restart_dspi) {
		/* Here, we missed some data.
		 * we need to reset the whole DSPI block to clear fifo
		 */
		complete(&drv_data->read_error_complete);
	}

	/* Let's awake any sleeping process waiting on read() syscall */
	complete(&drv_data->read_complete);

	local_unlock_irqrestore(&drv_data->lock, flags);

	regmap_update_bits(drv_data->regmap, SPI_SR, SPI_SR_TFUF, SPI_SR_TFUF);
	return IRQ_HANDLED;
}

/****************************************************************************/

/*
 * SPI block control
 */

static int setup(struct driver_data *drv_data)
{
	/* Set idle states for all chip select signals to high */
	regmap_write(drv_data->regmap, SPI_MCR, SPI_MCR_PCSIS(0xff));
	regmap_update_bits(drv_data->regmap, SPI_MCR,
			   SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF,
			   SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF);
	regmap_update_bits(drv_data->regmap, SPI_MCR,
			   SPI_MCR_ROOE, SPI_MCR_ROOE);

	regmap_write(drv_data->regmap, SPI_SR, SPI_SR_CLEAR);
	regmap_write(drv_data->regmap, SPI_CTARE(0),
		     SPI_FRAME_EBITS(16));

	return 0;
}

static ssize_t reset_stats_write(struct file *file,
				 const char __user *buf,
				 size_t count,
				 loff_t *ppos)
{
	struct driver_data *drv_data = file_inode(file)->i_private;

	stat_reset_counters(drv_data);

	pr_info("DSPI: Statistics cleared\n");
	return count; /* pretend we consumed all data */
}

static const struct file_operations reset_stats_fops = {
	.owner = THIS_MODULE,
	.write = reset_stats_write,
};

/****************************************************************************/

static const struct file_operations debugfs_sync_s0tos2_fops = {
	.write = debugfs_write_sync_s0tos2,
};

/*
 * Generic Device driver routines and interface implementation
 */

static int coldfire_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct coldfire_spi_slave *platform_info;
	struct driver_data *drv_data = 0;
	void __iomem *dspi_base;
	const struct regmap_config *regmap_config;
	struct resource *res;
	int irq;
	int status = 0;
	int ret;

	platform_info = (struct coldfire_spi_slave *)dev->platform_data;

	drv_data = kzalloc(sizeof(*drv_data), GFP_KERNEL);
	if (!drv_data)
		return -ENOMEM;

	mutex_init(&chrdev_client_mutex);

	/* Global variable init */
	chrdev_is_open = false;
	chrdev_drvdata = NULL;

	dev_info(&pdev->dev, "DSPI: Coldfire DSPI Slave driver\n");
	/* Setup register addresses */
	dspi_base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (!dspi_base) {
		dev_err(&pdev->dev, "Failed to remap SPI memory\n");
		goto out_error_after_drv_data_alloc;
	}

	regmap_config = &dspi_regmap_config;
	drv_data->regmap = devm_regmap_init_mmio(&pdev->dev, dspi_base, regmap_config);
	if (IS_ERR(drv_data->regmap)) {
		dev_err(&pdev->dev, "failed to init regmap: %ld\n",
			PTR_ERR(drv_data->regmap));
		ret = PTR_ERR(drv_data->regmap);
		goto out_error_after_drv_data_alloc;
	}

	drv_data->pdev = pdev;

	drv_data->dspi_base = res->start;
	dspi_slave_dma_setup_channel(drv_data);

	irq = platform_get_irq(pdev, 0);
	status = request_threaded_irq(irq, dspi_interrupt, NULL,
				      0, "dspi-slave", drv_data);
	if (status < 0) {
		dev_err(&pdev->dev, "Unable to attach ColdFire DSPI interrupt\n");
		goto out_error_after_drv_data_alloc;
	}

	local_lock_init(&drv_data->lock);

	stat_reset_counters(drv_data);

	platform_set_drvdata(pdev, drv_data);
	setup(drv_data);

	/* We allocate a major for our device */
	drv_data->chrdev_major = register_chrdev(0, DEVICE_NAME, &chrdev_fops);
	if (drv_data->chrdev_major < 0) {
		dev_err(&pdev->dev,
			"Unable to register device: error %d\n",
			drv_data->chrdev_major);
		status = drv_data->chrdev_major;
		goto out_error_irq_alloc;
	}

	/* We create a new "virtual" device class. */
	drv_data->chrdev_class = class_create(CLASS_NAME);
	if (IS_ERR(drv_data->chrdev_class)) {
		dev_err(&pdev->dev,
			"failed to register device class '%s'\n",
			CLASS_NAME);
		status = PTR_ERR(drv_data->chrdev_class);
		goto out_error_classreg;
	}

	/* We instantiate the device */
	drv_data->chrdev = device_create(drv_data->chrdev_class,
					 NULL,
					 MKDEV(drv_data->chrdev_major, 0),
					 NULL,
					 CLASS_NAME "_" DEVICE_NAME);
	if (IS_ERR(drv_data->chrdev)) {
		dev_err(&pdev->dev,
			"Unable to create device '%s_%s'\n",
			CLASS_NAME,
			DEVICE_NAME);
		status = PTR_ERR(drv_data->chrdev);
		goto out_error_devreg;
	}

	/* Right now, we only support one /dev entry, and one pointer to drv_data.
	 * if developer register more than one "dspi-slave" instance in platform,
	 * this will stop the runtime at this point.
	 */
	BUG_ON(chrdev_drvdata != NULL);
	chrdev_drvdata = drv_data;

	chrdev_drvdata->frame_perf.latency = 0;
	chrdev_drvdata->frame_perf.max_latency = 0;
	chrdev_drvdata->frame_perf.min_latency = KTIME_MAX;
	chrdev_drvdata->frame_perf.frame_number = 0;

	chrdev_drvdata->mode = DSPI_DMA_MODE;

	/* Create a debugfs to drive sync_s0tos2_set()
	 * from /sys/kernel/debug/dspi-slave/sync_s0tos2
	 */
	chrdev_drvdata->debugfs_direntry = debugfs_create_dir(CLASS_NAME, NULL);
	if (!debugfs_create_file("sync_s0tos2",
				 0600,
				 drv_data->debugfs_direntry,
				 chrdev_drvdata,
				 &debugfs_sync_s0tos2_fops))
		dev_warn(&pdev->dev, "Unable to create %s entry\n", "sync_s0tos2");

	debugfs_create_u64("stat_spi_nbbytes_recv", 0444,
			   drv_data->debugfs_direntry,
			   &drv_data->stat_spi_nbbytes_recv);

	debugfs_create_u64("stat_spi_nbbytes_sent", 0444,
			   drv_data->debugfs_direntry,
			   &drv_data->stat_spi_nbbytes_sent);

	debugfs_create_u64("stat_rx_hwfifo_overflow", 0444,
			   drv_data->debugfs_direntry,
			   &drv_data->stat_rx_hwfifo_overflow);

	debugfs_create_u64("stat_rx_kfifo_overflow", 0444,
			   drv_data->debugfs_direntry,
			   &drv_data->stat_rx_kfifo_overflow);

	debugfs_create_u64("stat_tx_hwfifo_underflow", 0444,
			   drv_data->debugfs_direntry,
			   &drv_data->stat_tx_hwfifo_underflow);

	debugfs_create_file("reset_stats", 0220,
			    drv_data->debugfs_direntry,
			    drv_data, &reset_stats_fops);

	/* Create debugfs files for latency */
	debugfs_create_u64("min_latency_ns", 0444,
			   drv_data->debugfs_direntry,
			   &drv_data->frame_perf.min_latency);

	debugfs_create_u64("max_latency_ns", 0444,
			   drv_data->debugfs_direntry,
			   &drv_data->frame_perf.max_latency);

	debugfs_create_u64("avg_latency_ns", 0444,
			   drv_data->debugfs_direntry,
			   &drv_data->frame_perf.avg_latency);

	dev_info(&pdev->dev, "DSPI: Coldfire slave initialized (DSPI%d)\n",
		 platform_info->bus_num);

	sync_spi_hw();

	/* Reset HW TX & HW RX fifo */
	hwfifo_prepare(drv_data);

	/* Apply registers settings to HW layer */
	dspi_setup_chip(drv_data);

	/* We inform S12X not to send a bitstream */
	sync_s0tos2_set(false);

	return status;

out_error_devreg:
	class_unregister(chrdev_drvdata->chrdev_class);
	class_destroy(chrdev_drvdata->chrdev_class);

out_error_classreg:
	unregister_chrdev(chrdev_drvdata->chrdev_major, DEVICE_NAME);

out_error_irq_alloc:
	free_irq(platform_info->irq_vector, chrdev_drvdata);

out_error_after_drv_data_alloc:
	kfree(chrdev_drvdata);

	return status;
}

static void coldfire_spi_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct driver_data *drv_data = platform_get_drvdata(pdev);
	struct coldfire_spi_slave *platform_info;

	if (!drv_data)
		return;

	complete(&drv_data->read_complete);

	if (drv_data->read_error_task) {
		kthread_stop(drv_data->read_error_task);
		drv_data->read_error_task = NULL;
	}

	platform_info = (struct coldfire_spi_slave *)dev->platform_data;

	/* In case bitstream is still coming...
	 * inform SIL2 it should stop it
	 */
	sync_s0tos2_set(false);

	/* Disable RX and TX */
	regmap_update_bits(drv_data->regmap, SPI_MCR,
			   SPI_MCR_DIS_TXF | SPI_MCR_DIS_RXF,
			   SPI_MCR_DIS_TXF | SPI_MCR_DIS_RXF);

	/* Stop Running */
	regmap_update_bits(drv_data->regmap, SPI_MCR, SPI_MCR_HALT, 1);

	device_destroy(drv_data->chrdev_class, MKDEV(drv_data->chrdev_major, 0));
	class_unregister(drv_data->chrdev_class);
	class_destroy(drv_data->chrdev_class);
	unregister_chrdev(drv_data->chrdev_major, DEVICE_NAME);

	/* Release IRQ */
	free_irq(platform_info->irq_vector, drv_data);

	debugfs_remove_recursive(drv_data->debugfs_direntry);

	/* Free allocated memory */
	kfree(drv_data);

	/* Prevent double remove */
	platform_set_drvdata(pdev, NULL);

	printk(KERN_INFO "DSPI: Coldfire slave unloaded (DSPI%d)\n", platform_info->bus_num);
}

static void coldfire_spi_shutdown(struct platform_device *pdev)
{
	coldfire_spi_remove(pdev);
}

/* CONFIG_PM is not handled whatever the power management policy */
#define coldfire_spi_suspend NULL
#define coldfire_spi_resume NULL

static struct platform_driver driver = {
	.driver = {
		.name = "fsl-dspi-slave",
		.bus = &platform_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = coldfire_spi_probe,
	.remove = coldfire_spi_remove,
	.shutdown = coldfire_spi_shutdown,
	.suspend = coldfire_spi_suspend,
	.resume = coldfire_spi_resume,
};

static int __init coldfire_spi_init(void)
{
	platform_driver_register(&driver);

	return 0;
}
module_init(coldfire_spi_init);

static void __exit coldfire_spi_exit(void)
{
	platform_driver_unregister(&driver);
}
module_exit(coldfire_spi_exit);
