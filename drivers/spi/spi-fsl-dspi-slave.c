/*
 * dspi_mcf_slave.c - DSPI controller for ColdFire M5441x processors
 *
 * Copyright (c) 2014 Eurogiciel Ingenierie, Inc.
 * Author:	Y.Gicquel <yannick.gicquel@open.eurogiciel.org>
 *
 * Based on dspi_mcf.c
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 ***************************************************************************
 * Changes:
 *   v0.004	Slave mode support.
 *   v0.003	M5301x support.
 *   v0.002	M547x/M548x support.
 *   v0.001	Initial version. Coldfire DSPI master driver.
 ****************************************************************************/

/*
 * Includes
 */

#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
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
#define S0TOS2_LOGIC_INVERTED

/* Uncomment following if you need to use hardirq for DSPI slave irq handler */
//#define DSPI_SLAVE_HARDIRQ_HANDLER

/****************************************************************************/

#define DSPI_CS(cs) ((1<<(cs))<<16)

static irqreturn_t dspi_interrupt(int, void *);

/* Default timeout to 5s */
static ktime_t read_fifo_timeout = 5ULL * NSEC_PER_SEC;

//static DEFINE_SEMAPHORE(rw_lock, 1);

/****************************************************************************/

static inline void sync_s0tos2_set(u8 val)
{
	trace_printk("DSPI: sync_s0tos2 = %s\n", val ? "0" : "1");
#if 1
	/* GPIOG3 output mode */
	__raw_writeb(__raw_readb(MCFGPIO_PDDR_G) | (1 << 3), MCFGPIO_PDDR_G);


#ifdef S0TOS2_LOGIC_INVERTED
	if (!val)
#else
	if (val)
#endif
		__raw_writeb(__raw_readb(MCFGPIO_PODR_G) | (1 << 3), MCFGPIO_PODR_G); /* GPIOG3 set to 1 */
	else
		__raw_writeb(__raw_readb(MCFGPIO_PODR_G) & ~(1 << 3), MCFGPIO_PODR_G); /* GPIOG3 set to 0 */
#else
#ifdef S0TOS2_LOGIC_INVERTED
	if (!val)
#else
	if (val)
#endif
	/* G3 => 7*8 + 3 */
	gpio_request(59, "sync_s0tos2");
	gpio_direction_output(59, !!val);
#endif
}

static inline u8 sync_s2tos0_get(void)
{
#if 0
	/* GPIOC4 input mode */
	__raw_writeb(__raw_readb(MCFGPIO_PDDR_C) & ~(1 << 4), MCFGPIO_PDDR_C);

	return __raw_readb(MCFGPIO_PPDSDR_C) & (1 << 4);
#else
	/* C4 => 3*8 + 4 */
	gpio_request(28, "sync_s2tos0");
	gpio_direction_input(28);
	return gpio_get_value(28);
#endif
}

static inline void stat_reset_counters(struct driver_data *drv_data)
{
	drv_data->stat_spi_nbbytes_recv    = 0;
	drv_data->stat_spi_nbbytes_sent    = 0;
	drv_data->stat_rx_hwfifo_overflow  = 0;
	drv_data->stat_rx_kfifo_overflow   = 0;
	drv_data->stat_tx_hwfifo_underflow = 0;
}

/****************************************************************************/

/*
 * Char device related callback
 */

/* Global reference of dynamically allocated "drv_data".
 *
 * TODO: We support only one instance as we manage one entry in /dev.
 * (This should be enhanced to manage multiple driver instance) */
static struct driver_data *chrdev_drvdata;
static uint8_t chrdev_is_open;

/* read() and write() exchanges between kernel and userspace are based
 * on a fixed buffer length equal to the following constant */
#define SPISLAVE_MSG_FIFO_SIZE 32 /* in bytes */

/* Exclusive access to /dev entry */
static DEFINE_MUTEX(chrdev_client_mutex);

/* Use a Wait queue for userspace synchronisation on read() syscall */
static DECLARE_WAIT_QUEUE_HEAD(read_buf_wq);

static inline void hwfifo_prepare(struct driver_data *drv_data)
{
	/* Set CLR_TXF & CLR_RXF bit : Clear TX & RX fifo */
	// *((volatile u32 *)drv_data->mcr) |=
	// 	(MCF_DSPI_DMCR_CLRTXF | MCF_DSPI_DMCR_CLRRXF);
	__raw_writel(__raw_readl(drv_data->mcr) |
		(MCF_DSPI_DMCR_CLRTXF | MCF_DSPI_DMCR_CLRRXF), drv_data->mcr);
}

static inline int kfifo_tx_kfifo_to_hw(struct driver_data *drv_data)
{
	unsigned int nb_elem;
	unsigned int total_sent = 0;

	//trace_printk("send %04x\n", drv_data->mmap_buffer[0]);
	for (nb_elem = 0; nb_elem < SPISLAVE_MSG_FIFO_SIZE / 2; nb_elem += 1) {

		*((volatile u32 *)drv_data->dspi_dtfr) = MCF_DSPI_DTFR_TXDATA(drv_data->mmap_buffer[nb_elem]);

		total_sent += 2;
		drv_data->stat_spi_nbbytes_sent += 2;
	}

	return total_sent;
}

/****************************************************************************/

/*
 * SPI local functions
 */

static inline void dspi_setup_chip(struct driver_data *drv_data)
{
	struct chip_data *chip = drv_data->cur_chip;

	__raw_writel(chip->mcr_val, drv_data->mcr);
	__raw_writel(chip->ctar_val, drv_data->ctar + drv_data->cs);

	__raw_writel(MCF_DSPI_DRSER_RFDFE | MCF_DSPI_DRSER_RFOFE | MCF_DSPI_DRSER_TFUFE, drv_data->dspi_rser);
}

/****************************************************************************/

/*
 * /dev EPORT4 related callbacks (sync_s2tos0 event handler)
 */

#define M5441X_EPORT4_IRQ_SOURCE	(4)
#define M5441X_EPORT4_IRQ_VECTOR	(64 + M5441X_EPORT4_IRQ_SOURCE)

#define S2TOS0_EPORT_DEVICE_NAME	"s2tos"


/* Device variables */
static struct device *s2tos0_eport_device;
static int s2tos0_eport_major;
static int s2tos0_eport_firstread;

static DECLARE_WAIT_QUEUE_HEAD(s2tos0_eport_waitq);

static volatile int s2tos0_eport_hasdata;

static irqreturn_t s2tos0_eport_handler(int irq, void *dev_id)
{
	s2tos0_eport_hasdata = 1;

	wake_up(&s2tos0_eport_waitq);

	return IRQ_HANDLED;
}

/* File operations */
static int s2tos0_eport_open(struct inode *inode, struct file *filp)
{
	s2tos0_eport_firstread = true;

	return 0;
}

static int s2tos0_eport_close(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t s2tos0_eport_read(
	struct file *filp,
	char __user *buffer,
	size_t length,
	loff_t *offset)
{
	char msg[2];
	int count;
	unsigned long flags;

	if (length < 1)
		return -EINVAL;

	if (unlikely(s2tos0_eport_firstread))
		s2tos0_eport_firstread = false;
	else {
		s2tos0_eport_hasdata = 0;
		wait_event_interruptible(s2tos0_eport_waitq, s2tos0_eport_hasdata);
	}

	local_irq_save(flags);

	/* Pinmux: PAR_IRQ04 -> GPIO (PC4) */
	// byte = MCFGPIO_PAR_IRQ0H;
	// byte = byte & (~0x0c);
	// MCFGPIO_PAR_IRQ0H = byte;
	__raw_writeb(__raw_readb(MCFGPIO_PAR_IRQ0H) & (~0x0c), MCFGPIO_PAR_IRQ0H);

	/* Read PC4/IRQ04 */
	msg[0] = (__raw_readb(MCFGPIO_PPDSDR_C) & 0x10) ? '1' : '0';

	/* Pinmux: PAR_IRQ04 -> Primary (IRQ) */
	// byte = MCFGPIO_PAR_IRQ0H;
	// byte = byte | 0x0c;
	// MCFGPIO_PAR_IRQ0H = byte;
	__raw_writeb(__raw_readb(MCFGPIO_PAR_IRQ0H) | 0x0c, MCFGPIO_PAR_IRQ0H);

	local_irq_restore(flags);

	if (length >= 2) {
		msg[1] = '\n';
		count = 2;
	} else
		count = 1;

	if (copy_to_user(buffer, msg, count) != 0) {
		dev_err(s2tos0_eport_device, "Failed to copy data to user space\n");
		return -EFAULT;
	}

	return count;
}

static const struct file_operations s2tos0_eport_fops = {
	.read = s2tos0_eport_read,
	.open = s2tos0_eport_open,
	.release = s2tos0_eport_close
};

static int s2tos0_eport_init(struct platform_device *pdev, struct class * klass)
{
	int retval;

	/* Register /dev/s2tos0 char device */
	s2tos0_eport_major = register_chrdev(0, S2TOS0_EPORT_DEVICE_NAME, &s2tos0_eport_fops);

	if (s2tos0_eport_major < 0) {
		retval = s2tos0_eport_major;
		printk(KERN_ERR "DSPI: Failed to register eport device (error: %d)\n", retval);
		goto register_chrdev_failed;
	}

	s2tos0_eport_device = device_create(klass,
					  NULL,
					  MKDEV(s2tos0_eport_major, 0),
					  NULL, "%s%d",
					  S2TOS0_EPORT_DEVICE_NAME,
					  0);

	if (IS_ERR(s2tos0_eport_device)) {
		printk(KERN_ERR "DSPI: Failed to create device '%s0'\n", S2TOS0_EPORT_DEVICE_NAME);
		retval = PTR_ERR(s2tos0_eport_device);
		goto device_create_failed;
	}

	retval = request_irq(M5441X_EPORT4_IRQ_VECTOR, s2tos0_eport_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "s2tos0-eportd", NULL);
	if (retval < 0) {
		printk(KERN_ERR "Unable to attach EPORT interrupt: %d\n", retval);
		goto register_irq_failed;
	}

	return 0;

device_create_failed:
	unregister_chrdev(s2tos0_eport_major, S2TOS0_EPORT_DEVICE_NAME);

register_chrdev_failed:
	free_irq(M5441X_EPORT4_IRQ_VECTOR, NULL);

register_irq_failed:
	return retval;
}

static void s2tos0_eport_exit(struct platform_device *pdev)
{
	struct driver_data *drv_data = platform_get_drvdata(pdev);

	/* Unregister IRQ4 handler */
	free_irq(M5441X_EPORT4_IRQ_VECTOR, NULL);

	device_destroy(drv_data->chrdev_class, MKDEV(s2tos0_eport_major, 0));
	unregister_chrdev(s2tos0_eport_major, S2TOS0_EPORT_DEVICE_NAME);
}

/****************************************************************************/

/*
 * /dev callbacks
 */

 static void measure_time(void)
 {
	static int count = 0;

	ktime_t diff = ktime_sub(chrdev_drvdata->frame_perf.wait_next_frame, chrdev_drvdata->frame_perf.irq_received);
	chrdev_drvdata->average_frame_time += diff;
	chrdev_drvdata->min_frame_time = diff < chrdev_drvdata->min_frame_time ? diff : chrdev_drvdata->min_frame_time;
	chrdev_drvdata->max_frame_time = diff > chrdev_drvdata->max_frame_time ? diff : chrdev_drvdata->max_frame_time;

	count++;
	if (count >= 512) {
		/* Display in the cyclictest form: Min:    364 Avg:  761 Max:    2629 */
		trace_printk("Min: %5lld Avg: %5lld Max: %5lld\n",
			ktime_to_us(chrdev_drvdata->min_frame_time),
			ktime_to_us(chrdev_drvdata->average_frame_time >> 9),
			ktime_to_us(chrdev_drvdata->max_frame_time));
		chrdev_drvdata->average_frame_time = 0;
		count = 0;
	}
}

static void sync_spi_hw(void)
{
	read_fifo_timeout = ktime_set(5, 0);

	/* Disable the DSPI IP */
	chrdev_drvdata->cur_chip->mcr.halt = 1;
	dspi_setup_chip(chrdev_drvdata);

	/* Reset HW TX & HW RX fifo */
	hwfifo_prepare(chrdev_drvdata);

	/* Enable the DSPI IP */
	chrdev_drvdata->cur_chip->mcr.halt = 0;
	dspi_setup_chip(chrdev_drvdata);

	chrdev_drvdata->state = DSPI_SLAVE_STATE_IDLE;
}

static int chrdev_device_open(struct inode *inode, struct file *filp)
{
	/* Ensure that only one process has access to our device
	 * at any one time */
	if (!mutex_trylock(&chrdev_client_mutex)) {
		printk (KERN_DEBUG "DSPI: Try to open char device more than once.\n");
		return -EBUSY;
	}

	/* We store the PID of userspace apps to send signal from
	 * IRQ handler. */
	chrdev_drvdata->sinfo.si_signo = SIGUSR1;
	chrdev_drvdata->sinfo.si_code = SI_KERNEL;
	chrdev_drvdata->sinfo.si_int = 0;

	chrdev_drvdata->task_user = current;

	chrdev_is_open = true;

	stream_open(inode, filp);

	filp->private_data = inode->i_private;

	return 0;
}

static int chrdev_device_close(struct inode *inode, struct file *filp)
{
	chrdev_is_open = false;
	chrdev_drvdata->task_user = NULL;

	complete(&chrdev_drvdata->read_complete);
	mutex_unlock(&chrdev_client_mutex);
	return 0;
}

static ssize_t chrdev_device_read(struct file *filp,
	char __user *buffer, size_t length, loff_t *offset)
{
	int status;
	unsigned int nb_bytes;

	reinit_completion(&chrdev_drvdata->read_complete);
	//trace_printk("Wait for next frame\n");
	if (chrdev_drvdata->frame_perf.frame_number != 0) {
		chrdev_drvdata->frame_perf.wait_next_frame = ktime_get();
		measure_time();
	}

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

	/* If RX Fifo is not full, we block the caller */
	status = wait_for_completion_interruptible_timeout(&chrdev_drvdata->read_complete, msecs_to_jiffies(ktime_to_ms(read_fifo_timeout)));
	switch(chrdev_drvdata->state) {
		case DSPI_SLAVE_STATE_IDLE:
		case DSPI_SLAVE_STATE_RX:
			break;
		case DSPI_SLAVE_STATE_RESTART:
			/* If we are in restart state, we return 0 */
			if (chrdev_drvdata->stream_restarted)
				chrdev_drvdata->stream_restarted = false;
			trace_printk("DSPI_SLAVE_STATE_RESTART\n");
			return 0;
		default:
			trace_printk("%s: Wrong state %d\n", __func__, chrdev_drvdata->state);
			/* Blocking read() return 0 after signal */
	}

	if (status == 0) {
#ifndef CONFIG_TRACING
		printk_once(KERN_ERR "DSPI: timeout occured on read() syscall\n");
#else
		trace_printk("Timeout on read() syscall\n");
#endif
		return -EAGAIN;
	}

	if (status < 0) {
		trace_printk("Signal received\n");
		return 0;
	}

	/* Data are ready, now we send them to the caller */
	//status = kfifo_to_user(&rx_kfifo, buffer, length, &nb_bytes);
	nb_bytes = SPISLAVE_MSG_FIFO_SIZE;
	status = __copy_to_user_inatomic(buffer, chrdev_drvdata->mmap_buffer, nb_bytes);

	chrdev_drvdata->state = DSPI_SLAVE_STATE_RX_DONE;

	if (status) {
		trace_printk("Error while copying data to user: %d\n", status);
	}
	//trace_printk("Copied %d bytes to user\n", status ? status : nb_bytes);
	return status ? status : nb_bytes;
}

static ssize_t chrdev_device_write(struct file *filp,
	const char __user *buffer, size_t length, loff_t *offset)
{
	int status;
	unsigned int nb_bytes;
	u8 txctr, timeout;

	/* Check length is SPISLAVE_MSG_FIFO_SIZE */
	if (unlikely(length != SPISLAVE_MSG_FIFO_SIZE)) {
		return -EINVAL;
	}

	/* Check TX FIFO is empty.
	 * If userspace app trig this issue, this mean read() and
	 * write() call sequence is not respected */
	timeout = 10;
	while ((txctr = MCF_DSPI_DSR_GET_TXCTR(*(volatile u32 *)chrdev_drvdata->dspi_sr) != 0) && (timeout > 0)) {
		trace_printk("HW TX fifo not empty (txctr = %u) !\n", txctr);
		udelay(100);
		timeout--;
	}

	chrdev_drvdata->state = DSPI_SLAVE_STATE_TX;

	preempt_disable();
	/* Push whole buffer in TX fifo */
	status = __copy_from_user_inatomic(chrdev_drvdata->mmap_buffer, buffer, length);
	//status = kfifo_from_user(&tx_kfifo, buffer, length, &nb_bytes);

	/* If HW TX FIFO is empty, we transfer datas from
	 * kfifo to HW FIFO. */
	if (status == 0)
		nb_bytes = kfifo_tx_kfifo_to_hw(chrdev_drvdata);

	chrdev_drvdata->frame_perf.frame_sent = ktime_get();

	preempt_enable();

	//trace_printk("Write %d bytes to HW FIFO\n", nb_bytes);
	chrdev_drvdata->state = DSPI_SLAVE_STATE_IDLE;
	return status ? status : nb_bytes;
}

static int chrdev_device_fsync (struct file *filp, loff_t start, loff_t end, int datasync)
{
	sync_spi_hw();

	return 0;
}

static int chrdev_device_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long pfn;

	pfn = virt_to_phys(chrdev_drvdata->mmap_buffer) >> PAGE_SHIFT;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vm_flags_set(vma, VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP);

	if (remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static struct file_operations chrdev_fops = {
	.read = chrdev_device_read,
	.write = chrdev_device_write,
	.open = chrdev_device_open,
	.release = chrdev_device_close,
	.fsync = chrdev_device_fsync,
	.mmap = chrdev_device_mmap,
};

/****************************************************************************/
#if 0
/*
 * procfs related callback
 */
static int procfs_read_stat(
	char *page_buffer,
	char **my_first_byte,
	off_t virtual_start,
	int length,
	int *eof,
	void *data)
{
	int my_buffer_offset = 0;
	char * const my_base = page_buffer;
	unsigned long long value = *((unsigned long long *)data);

	if (virtual_start == 0) {
		my_buffer_offset += sprintf(my_base + my_buffer_offset,
		   "%llu\n", value);

		*my_first_byte = page_buffer;
		return  my_buffer_offset;
	} else {
		*eof = 1;
		return 0;
	}
}
static int procfs_write_reset_stat(struct file *file,
					const char *buffer,
					unsigned long count,
					void *data)
{
	struct driver_data *drv_data = (struct driver_data *) data;

	stat_reset_counters(drv_data);
	printk(KERN_INFO "DSPI: Statistics cleared\n");

	return count;
}

static int procfs_read_sync_s2tos0(
	char *page_buffer,
	char **my_first_byte,
	off_t virtual_start,
	int length,
	int *eof,
	void *data)
{
	int my_buffer_offset = 0;
	char * const my_base = page_buffer;

	if (virtual_start == 0) {
		my_buffer_offset += sprintf(my_base + my_buffer_offset,
		   "%c\n", sync_s2tos0_get() ? '1' : '0');

		*my_first_byte = page_buffer;
		return  my_buffer_offset;
	} else {
		*eof = 1;
		return 0;
	}
}

static int procfs_write_sync_s0tos2(struct file *file,
					const char *buffer,
					unsigned long count,
					void *data)
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
#endif

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
	struct device * dev = &drv_data->pdev->dev;
	volatile u32 irq_status = __raw_readl(drv_data->dspi_sr);

	//trace_printk("Enter dspi_interrupt\n");

	read_fifo_timeout = ktime_set(0, 10 * NSEC_PER_MSEC);

	if (irq_status & MCF_DSPI_DSR_RFOF) {
		/* RX HW FIFO overflow occurs */
		drv_data->stat_rx_hwfifo_overflow += 1;
#ifndef CONFIG_TRACING
		dev_dbg(dev,
		 	"DSPI: dspi-slave: RX FIFO overflow occurs (occur. #%llu) !\n",
			drv_data->stat_rx_hwfifo_overflow);
#else
		trace_printk("RX FIFO overflow\n");
#endif
		restart_dspi = true;
	}

	if (irq_status & MCF_DSPI_DSR_TFUF) {
		/* TX HW FIFO underflow occurs */
		drv_data->stat_tx_hwfifo_underflow += 1;

#ifndef CONFIG_TRACING
		dev_dbg(dev,
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
	if (!restart_dspi && (irq_status & MCF_DSPI_DSR_RFDF)) {
		/* When we enter here, we have data in hardware RX FIFO */
		u16 in_data;
		int i = 0;

		/* While datas are available in HW RX FIFO, we drain it. */
		while (__raw_readl(drv_data->dspi_sr) & MCF_DSPI_DSR_RFDF) {

			/* We pop one element from the hardware RX FIFO */
			//in_data = MCF_DSPI_DRFR_RXDATA(*drv_data->dspi_drfr);
			//in_data = __raw_readw(drv_data->dspi_drfr);

			/* Copy to the mmap'ed buffer */
			in_data = MCF_DSPI_DRFR_RXDATA(__raw_readl(drv_data->dspi_drfr));
			drv_data->mmap_buffer[i] = in_data;

			/* RFDF must be cleared only after the DSPI_POPR
			 * register is read. */
			//*((volatile u32 *)drv_data->dspi_sr) |= MCF_DSPI_DSR_RFDF;
			__raw_writel(__raw_readl(drv_data->dspi_sr) | MCF_DSPI_DSR_RFDF, drv_data->dspi_sr);

			/* Increment stat incoming bytes counter */
			drv_data->stat_spi_nbbytes_recv += 2;
			i++;
		}
		drv_data->state = DSPI_SLAVE_STATE_RX;
		drv_data->frame_perf.irq_received = ktime_get();
		drv_data->frame_perf.frame_number++;
		//trace_printk("get %04x\n", drv_data->mmap_buffer[0]);
	}

	/*
	 * FIFO overflow and underflow management
	 */
	if (restart_dspi) {
		/* Here, we missed some data.
		 * we need to reset the whole DSPI block to clear fifo */
		trace_printk("Restart FIFO\n");

		drv_data->state = DSPI_SLAVE_STATE_RESTART;

		/* We inform S12X not to send a bitstream anymore */
		sync_s0tos2_set(false);

		sync_spi_hw();
		/* We will unblock application read() with a buffer of size 0 */
		drv_data->stream_restarted = true;
	}

	/* Let's awake any sleeping process waiting on read() syscall */
	complete(&drv_data->read_complete);

	__raw_writel(MCF_DSPI_DSR_RFOF | MCF_DSPI_DSR_TFUF, drv_data->dspi_sr);
	return IRQ_HANDLED;
}

/****************************************************************************/

/*
 * SPI block control
 */

static int setup(struct driver_data *drv_data)
{
	struct chip_data *chip = drv_data->cur_chip;

	chip->mcr.master = 0;     /* 0: slave mode */
	chip->mcr.cont_scke = 0;  /* 0: disable continuous SCK */
	chip->mcr.dconf = 0;      /* 0: reserved */
	chip->mcr.frz = 0;        /* 0: do not halt serial transfers */
	chip->mcr.mtfe = 0;       /* 0: disable modified SPI transfer */
	chip->mcr.pcsse = 0;      /* 0: reserved */
	chip->mcr.rooe = 0;       /* 0: drop data on RX fifo overflow */
	chip->mcr.pcsis = 0xFF;   /* /SS is inactive in high state */
	chip->mcr.reserved15 = 0; /* 0: reserved */
	chip->mcr.mdis = 0;       /* 0: module disable bit */
	chip->mcr.dis_tx = 0;     /* 0: TX FIFO is enable */
	chip->mcr.dis_rxf = 0;    /* 0: RX FIFO is enable */
	chip->mcr.clr_tx = 1;     /* 1: Flush TX FIFO */
	chip->mcr.clr_rxf = 1;    /* 1: Flush RX FIFO */
	chip->mcr.smpl_pt = 0;    /* X: SPI modified transfer setting */
	chip->mcr.reserved71 = 0; /* 0: reserved */
	chip->mcr.halt = 0;       /* 0: start transfers */

	chip->ctar.fmsz = 15; /* #BITS per frame - 1 */
	chip->ctar.cpha = 0;  /* 0: DAT sampled on rising edge of SCK */
	chip->ctar.cpol = 0;  /* 0: inactive SCK polarity is 0V */
	chip->ctar.lsbfe = 0; /* X: this bit is ignored in slave mode */

	/* TODO: Check values */
	chip->ctar.dbr = 0;
	chip->ctar.pbr = 0;
	chip->ctar.br = 0;
	chip->ctar.pcssck = 0;
	chip->ctar.pasc = 0;
	chip->ctar.pdt = 0;
	chip->ctar.cssck = 0;
	chip->ctar.asc = 0;
	chip->ctar.dt = 0;

	return 0;
}

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
	struct resource *memory_resource;
	int irq;
	int status = 0;

	platform_info = (struct coldfire_spi_slave *)dev->platform_data;

	drv_data = kzalloc(sizeof(struct driver_data), GFP_KERNEL);
	if (!drv_data)
		return -ENOMEM;

	mutex_init(&chrdev_client_mutex);

	/* Global variable init */
	chrdev_is_open = false;
	chrdev_drvdata = NULL;

	/* Setup register addresses */
	memory_resource = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, "spi-slave-module");
	if (!memory_resource) {
		dev_dbg(dev, "can not find platform module memory\n");
		goto out_error_after_drv_data_alloc;
	}

	drv_data->pdev = pdev;
	drv_data->mcr = MCF_DSPI1_DMCR;
	drv_data->ctar = MCF_DSPI1_DCTAR0;
	drv_data->dspi_sr = MCF_DSPI1_DSR;
	drv_data->dspi_rser = MCF_DSPI1_DRSER;
	drv_data->dspi_dtfr = MCF_DSPI1_DTFR;
	drv_data->dspi_drfr = MCF_DSPI1_DRFR;

	memory_resource = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						       "spi-slave-par");
	if (!memory_resource) {
		dev_dbg(dev, "No spi-slave-par memory\n");
		goto out_error_after_drv_data_alloc;
	}

	memory_resource = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						       "spi-slave-int-level");
	if (!memory_resource) {
		dev_dbg(dev, "No spi-slave-int-level memory\n");
		goto out_error_after_drv_data_alloc;
	}
	drv_data->int_icr = (void *)memory_resource->start;

	memory_resource = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						       "spi-slave-int-mask");
	if (!memory_resource) {
		dev_dbg(dev, "No spi-slave-int-mask memory\n");
		goto out_error_after_drv_data_alloc;
	}
	drv_data->int_mr = (void *)memory_resource->start;

	init_completion(&drv_data->read_complete);
	drv_data->state = DSPI_SLAVE_STATE_IDLE;

	/*
	 * Sync signal configuration
	 */
	sync_s2tos0_get(); /* Muxing in input for this signal */

	/* We inform S12X not to send a bitstream */
	sync_s0tos2_set(false);

	irq = platform_info->irq_vector;
	status = request_irq(irq, dspi_interrupt, 0, "dspi-slave", drv_data);
	if (status < 0) {
		dev_err(&pdev->dev, "Unable to attach ColdFire DSPI interrupt\n");
		goto out_error_after_drv_data_alloc;
	}

	/* Enhance the interrupt priority */
	__raw_writeb(6, MCFINTC1_ICR0 + MCFINT1_DSPI1);

	*drv_data->int_icr = platform_info->irq_lp;
	*drv_data->int_mr &= ~platform_info->irq_mask;
	drv_data->stream_restarted = false;

	/* Prepare a buffer to export with mmap */
	drv_data->mmap_buffer = devm_kmalloc(dev, 2*SPISLAVE_MSG_FIFO_SIZE, GFP_KERNEL);
	if (!drv_data->mmap_buffer) {
		dev_err(dev, "can not allocate mmap buffer\n");
		goto out_error_after_drv_data_alloc;
	}
	drv_data->mmap_buffer_size = 2*SPISLAVE_MSG_FIFO_SIZE;
	mutex_init(&drv_data->lock);

	stat_reset_counters(drv_data);

	/* Chip select is always PCS0 in slave mode */
	drv_data->cs = 0;
#if 0
	/*
	 * Statistics
	 */
	stat_reset_counters(drv_data);

	/* Create /proc/dspi-slave directory */
	drv_data->procfs_direntry = proc_mkdir("dspi-slave", NULL);
	if (!drv_data->procfs_direntry) {
		status = -ENOMEM;
		dev_err(&pdev->dev,
			"Unable to create /proc/dspi-slave directory\n");
		goto out_error_after_drv_data_alloc;
	}

	/* Create statistics entries */
	procfs_entry = create_proc_read_entry("stat_spi_nbbytes_recv",
				S_IRUGO, drv_data->procfs_direntry,
				procfs_read_stat, (void *) &drv_data->stat_spi_nbbytes_recv);

	procfs_entry = create_proc_read_entry("stat_spi_nbbytes_sent",
				S_IRUGO, drv_data->procfs_direntry,
				procfs_read_stat, (void *) &drv_data->stat_spi_nbbytes_sent);

	procfs_entry = create_proc_read_entry("stat_rx_hwfifo_overflow",
				S_IRUGO, drv_data->procfs_direntry,
				procfs_read_stat, (void *) &drv_data->stat_rx_hwfifo_overflow);

	procfs_entry = create_proc_read_entry("stat_rx_kfifo_overflow",
				S_IRUGO, drv_data->procfs_direntry,
				procfs_read_stat, (void *) &drv_data->stat_rx_kfifo_overflow);

	procfs_entry = create_proc_read_entry("stat_tx_hwfifo_underflow",
				S_IRUGO, drv_data->procfs_direntry,
				procfs_read_stat, (void *) &drv_data->stat_tx_hwfifo_underflow);

	/* Write only entry to reset statistics */
	procfs_entry = create_proc_entry("reset_stats",
				S_IWUGO, drv_data->procfs_direntry);

	if (procfs_entry == NULL) {
		status = -ENOMEM;
		dev_warn(&pdev->dev, "Unable to create /proc entry\n");
		goto out_error_after_drv_data_alloc;
	} else {
		procfs_entry->data = (void *) drv_data;
		procfs_entry->write_proc = procfs_write_reset_stat;
	}

	/* Entries for sync_s0tos2 and sync_s2tos0 signals */
	procfs_entry = create_proc_read_entry("sync_s2tos0",
				S_IRUGO, drv_data->procfs_direntry,
				procfs_read_sync_s2tos0, (void *) drv_data);

	procfs_entry = create_proc_entry("sync_s0tos2",
				S_IWUGO, drv_data->procfs_direntry);

	if (procfs_entry == NULL) {
		status = -ENOMEM;
		dev_warn(&pdev->dev, "Unable to create /proc entry\n");
		goto out_error_after_drv_data_alloc;
	} else {
		procfs_entry->data = (void *) drv_data;
		procfs_entry->write_proc = procfs_write_sync_s0tos2;
	}
#endif

	/* Create a debugfs to drive sync_s0tos2_set() from /sys/kernel/debug/dspi-slave/sync_s0tos2 */
	drv_data->debugfs_direntry = debugfs_create_dir(CLASS_NAME, NULL);
	if (!debugfs_create_file("sync_s0tos2", 0200, drv_data->debugfs_direntry, drv_data, &debugfs_sync_s0tos2_fops))
		dev_warn(&pdev->dev, "Unable to create %s entry\n", "sync_s0tos2");

	/* TODO: Pack this struct in platform_info */
	drv_data->cur_chip = kzalloc(sizeof(struct chip_data), GFP_KERNEL);
	if (drv_data->cur_chip == NULL) {
		status = -ENOMEM;
		goto out_error_irq_alloc;
	}

	platform_set_drvdata(pdev, drv_data);
	setup(drv_data);

	/* Reset HW TX & HW RX fifo */
	hwfifo_prepare(drv_data);

	/* Apply registers settings to HW layer */
	dspi_setup_chip(drv_data);

	/* We allocate a major for our device */
	drv_data->chrdev_major = register_chrdev(0, DEVICE_NAME, &chrdev_fops);
	if (drv_data->chrdev_major < 0) {
		dev_err(&pdev->dev, "Unable to register device: error %d\n", drv_data->chrdev_major);
		status = drv_data->chrdev_major;
		goto out_error_after_drv_data_alloc;
	}

	/* We create a new "virtual" device class. */
	drv_data->chrdev_class = class_create(CLASS_NAME);
	if (IS_ERR(drv_data->chrdev_class)) {
		dev_err(&pdev->dev, "failed to register device class '%s'\n", CLASS_NAME);
		status = PTR_ERR(drv_data->chrdev_class);
		goto out_error_classreg;
	}

	/* We instantiate the device */
	drv_data->chrdev = device_create(drv_data->chrdev_class, NULL, MKDEV(drv_data->chrdev_major, 0), NULL, CLASS_NAME "_" DEVICE_NAME);
	if (IS_ERR(drv_data->chrdev)) {
		dev_err(&pdev->dev, "Unable to create device '%s_%s'\n", CLASS_NAME, DEVICE_NAME);
		status = PTR_ERR(drv_data->chrdev);
		goto out_error_devreg;
	}

	/* sync_s2tos0 IRQ based handler init */
	if ((status = s2tos0_eport_init(pdev, drv_data->chrdev_class)))
		goto out_error_devreg;

	/* Right now, we only support one /dev entry, and one pointer to drv_data.
	 * if developper register more than one "dspi-slave" instance in platform,
	 * this will stop the runtime at this point. */
	BUG_ON(chrdev_drvdata != NULL);
	chrdev_drvdata = drv_data;

	chrdev_drvdata->average_frame_time = 0;
	chrdev_drvdata->min_frame_time = KTIME_MAX;
	chrdev_drvdata->max_frame_time = 0;
	chrdev_drvdata->frame_perf.frame_number = 0;

	printk(KERN_INFO "DSPI: Coldfire slave initialized (DSPI%d)\n", platform_info->bus_num);

	return status;

out_error_devreg:
	class_unregister(drv_data->chrdev_class);
	class_destroy(drv_data->chrdev_class);

out_error_classreg:
	unregister_chrdev(drv_data->chrdev_major, DEVICE_NAME);
	kfree(drv_data->cur_chip);

out_error_irq_alloc:
	//remove_irq(platform_info->irq_vector, &dspi_irqaction);
	free_irq(platform_info->irq_vector, drv_data);

out_error_after_drv_data_alloc:
	kfree(drv_data);

	return status;
}

static void coldfire_spi_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct driver_data *drv_data = platform_get_drvdata(pdev);
	struct coldfire_spi_slave *platform_info;

	if (!drv_data)
		return;

	platform_info = (struct coldfire_spi_slave *)dev->platform_data;

	/* In case bitstream is still coming... inform SIL2 it should
	 * stop it */
	sync_s0tos2_set(false);

	s2tos0_eport_exit(pdev);

	device_destroy(drv_data->chrdev_class, MKDEV(drv_data->chrdev_major, 0));
	class_unregister(drv_data->chrdev_class);
	class_destroy(drv_data->chrdev_class);
	unregister_chrdev(drv_data->chrdev_major, DEVICE_NAME);

	/* Release IRQ */
	//remove_irq(platform_info->irq_vector, &dspi_irqaction);
	free_irq(platform_info->irq_vector, drv_data);

#if 0
	/* Remove ProcFS entries */
	remove_proc_entry("sync_s2tos0", drv_data->procfs_direntry);
	remove_proc_entry("sync_s0tos2", drv_data->procfs_direntry);
	remove_proc_entry("reset_stats", drv_data->procfs_direntry);
	remove_proc_entry("stat_tx_hwfifo_underflow", drv_data->procfs_direntry);
	remove_proc_entry("stat_rx_kfifo_overflow", drv_data->procfs_direntry);
	remove_proc_entry("stat_rx_hwfifo_overflow", drv_data->procfs_direntry);
	remove_proc_entry("stat_spi_nbbytes_sent", drv_data->procfs_direntry);
	remove_proc_entry("stat_spi_nbbytes_recv", drv_data->procfs_direntry);
	remove_proc_entry("dspi-slave", NULL);
#endif

	debugfs_remove_recursive(drv_data->debugfs_direntry);

	/* Free allocated memory */
	kfree(drv_data->cur_chip);
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
		.name = "mcf-spi-slave",
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

MODULE_AUTHOR("Yannick Gicquel");
MODULE_DESCRIPTION("ColdFire DSPI Controller (slave mode)");
MODULE_LICENSE("GPL");
