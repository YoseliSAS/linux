#ifndef __FSL_DSPI_SLAVE_H__
#define __FSL_DSPI_SLAVE_H__

#define SPI_MCR				0x00
#define SPI_MCR_HOST			BIT(31)
#define SPI_MCR_ROOE			BIT(24)
#define SPI_MCR_PCSIS(x)		((x) << 16)
#define SPI_MCR_CLR_TXF			BIT(11)
#define SPI_MCR_CLR_RXF			BIT(10)
#define SPI_MCR_XSPI			BIT(3)
#define SPI_MCR_DIS_TXF			BIT(13)
#define SPI_MCR_DIS_RXF			BIT(12)
#define SPI_MCR_HALT			BIT(0)

#define SPI_TCR				0x08
#define SPI_TCR_GET_TCNT(x)		(((x) & GENMASK(31, 16)) >> 16)

#define SPI_CTAR(x)			(0x0c + (((x) & GENMASK(1, 0)) * 4))
#define SPI_CTAR_FMSZ(x)		(((x) << 27) & GENMASK(30, 27))
#define SPI_CTAR_CPOL			BIT(26)
#define SPI_CTAR_CPHA			BIT(25)
#define SPI_CTAR_LSBFE			BIT(24)
#define SPI_CTAR_PCSSCK(x)		(((x) << 22) & GENMASK(23, 22))
#define SPI_CTAR_PASC(x)		(((x) << 20) & GENMASK(21, 20))
#define SPI_CTAR_PDT(x)			(((x) << 18) & GENMASK(19, 18))
#define SPI_CTAR_PBR(x)			(((x) << 16) & GENMASK(17, 16))
#define SPI_CTAR_CSSCK(x)		(((x) << 12) & GENMASK(15, 12))
#define SPI_CTAR_ASC(x)			(((x) << 8) & GENMASK(11, 8))
#define SPI_CTAR_DT(x)			(((x) << 4) & GENMASK(7, 4))
#define SPI_CTAR_BR(x)			((x) & GENMASK(3, 0))
#define SPI_CTAR_SCALE_BITS		0xf

#define SPI_CTAR0_SLAVE			0x0c

#define SPI_SR				0x2c
#define SPI_SR_TCFQF			BIT(31)
#define SPI_SR_TFUF			BIT(27)
#define SPI_SR_TFFF			BIT(25)
#define SPI_SR_CMDTCF			BIT(23)
#define SPI_SR_SPEF			BIT(21)
#define SPI_SR_RFOF			BIT(19)
#define SPI_SR_TFIWF			BIT(18)
#define SPI_SR_RFDF			BIT(17)
#define SPI_SR_CMDFFF			BIT(16)
#define SPI_SR_CLEAR			(SPI_SR_TCFQF | \
					SPI_SR_TFUF | SPI_SR_TFFF | \
					SPI_SR_CMDTCF | SPI_SR_SPEF | \
					SPI_SR_RFOF | SPI_SR_TFIWF | \
					SPI_SR_RFDF | SPI_SR_CMDFFF)

#define SPI_RSER_TFUFE			BIT(27)
#define SPI_RSER_TFFFE			BIT(25)
#define SPI_RSER_TFFFD			BIT(24)
#define SPI_RSER_RFDFE			BIT(17)
#define SPI_RSER_RFDFD			BIT(16)

#define SPI_RSER			0x30
#define SPI_RSER_TCFQE			BIT(31)
#define SPI_RSER_CMDTCFE		BIT(23)

#define SPI_PUSHR			0x34
#define SPI_PUSHR_CMD_CONT		BIT(15)
#define SPI_PUSHR_CMD_CTAS(x)		(((x) << 12 & GENMASK(14, 12)))
#define SPI_PUSHR_CMD_EOQ		BIT(11)
#define SPI_PUSHR_CMD_CTCNT		BIT(10)
#define SPI_PUSHR_CMD_PCS(x)		(BIT(x) & GENMASK(5, 0))

#define SPI_PUSHR_SLAVE			0x34

#define SPI_POPR			0x38

#define SPI_TXFR0			0x3c
#define SPI_TXFR1			0x40
#define SPI_TXFR2			0x44
#define SPI_TXFR3			0x48
#define SPI_RXFR0			0x7c
#define SPI_RXFR1			0x80
#define SPI_RXFR2			0x84
#define SPI_RXFR3			0x88

#define SPI_CTARE(x)			(0x11c + (((x) & GENMASK(1, 0)) * 4))
#define SPI_CTARE_FMSZE(x)		(((x) & 0x1) << 16)
#define SPI_CTARE_DTCP(x)		((x) & 0x7ff)

#define SPI_SREX			0x13c

#define SPI_FRAME_BITS(bits)		SPI_CTAR_FMSZ((bits) - 1)
#define SPI_FRAME_EBITS(bits)		SPI_CTARE_FMSZE(((bits) - 1) >> 4)

/*
 * Local Data Structures
 */

typedef enum {
	DSPI_SLAVE_STATE_IDLE,
	DSPI_SLAVE_STATE_RX,
	DSPI_SLAVE_STATE_RX_DONE,
	DSPI_SLAVE_STATE_TX,
	DSPI_SLAVE_STATE_RESTART,
} dspi_slave_state_t;

struct dspi_slave_perf {
	ktime_t irq_received;
	ktime_t wait_next_frame;
	u64 frame_sent;
	u64 latency;
	u64 max_latency;
	u64 min_latency;
	u64 frame_number;
};

enum dspi_trans_mode {
	DSPI_POLLING_MODE,
	DSPI_DMA_MODE,
};

struct driver_data {
	/* Driver model hookup */
	struct platform_device *pdev;

	/* Statistics */
	unsigned long long stat_spi_nbbytes_recv;
	unsigned long long stat_spi_nbbytes_sent;
	unsigned long long stat_rx_hwfifo_overflow;
	unsigned long long stat_rx_kfifo_overflow;
	unsigned long long stat_tx_hwfifo_underflow;

	/* debugfs */
	struct dentry *debugfs_direntry;

	/* Device variables */
	int chrdev_major;
	struct class *chrdev_class;
	struct device *chrdev;

	phys_addr_t 				dspi_base;
	struct regmap				*regmap;
	struct regmap				*regmap_pushr;
	u32 irq_status;

	u8  *int_icr;	   /* Interrupt level and priority register */
	u32 *int_mr;       /* Interrupt mask register */

	/* mmap */
	//u16 *mmap_buffer;
	u16 rx_buffer[32];
	u16 tx_buffer[32];

	/* locking */
	local_lock_t 				lock;

	struct completion 			read_complete;
	struct completion 			write_complete;
	struct completion			read_error_complete;

	struct task_struct			*read_error_task;

	dspi_slave_state_t			state;

	/* Add time measurement variables */
	struct dspi_slave_perf 			frame_perf;
	ktime_t 				average_frame_time;
	ktime_t 				min_frame_time;
	ktime_t 				max_frame_time;

	/* DMA */
	struct dma_chan				*chan_rx;
	dma_addr_t				rx_dma_phys;
	u32					*rx_dma_buf;
	u32					rx_dma_buf_size;
	u32					rx_period_length;
	u32					rx_dma_buf_offset;
	dma_cookie_t				rx_cookie;
	struct dma_async_tx_descriptor 		*rx_desc;
	atomic_t				rx_dma_running;
	int 					rx_priority;

	struct dma_chan				*chan_tx;
	dma_addr_t				tx_dma_phys;
	u32					*tx_dma_buf;
	u32					tx_dma_buf_size;
	u32					tx_period_length;
	u32					tx_dma_buf_offset;
	dma_cookie_t				tx_cookie;
	struct dma_async_tx_descriptor 		*tx_desc;
	atomic_t				tx_dma_running;
	int 					tx_priority;

	u8					mode;

	wait_queue_head_t			wq_tx;
};

/* Define ioctl commands */
#define SPIRT_IOCTL_GET_FRAME _IOR('M', 1, int)
#define SPIRT_IOCTL_SET_FRAME _IO('M', 2)

/*********************************************************************
*
* Edge Port Module (EPORT)
*
*********************************************************************/

/* Bit definitions and macros for EPPAR */
#define MCFEPORT_EPPAR_EPPA1(x)        (((x)&0x0003)<<2)
#define MCFEPORT_EPPAR_EPPA2(x)        (((x)&0x0003)<<4)
#define MCFEPORT_EPPAR_EPPA3(x)        (((x)&0x0003)<<6)
#define MCFEPORT_EPPAR_EPPA4(x)        (((x)&0x0003)<<8)
#define MCFEPORT_EPPAR_EPPA5(x)        (((x)&0x0003)<<10)
#define MCFEPORT_EPPAR_EPPA6(x)        (((x)&0x0003)<<12)
#define MCFEPORT_EPPAR_EPPA7(x)        (((x)&0x0003)<<14)
#define MCFEPORT_EPPAR_LEVEL           (0)
#define MCFEPORT_EPPAR_RISING          (1)
#define MCFEPORT_EPPAR_FALLING         (2)
#define MCFEPORT_EPPAR_BOTH            (3)
#define MCFEPORT_EPPAR_EPPA7_LEVEL     (0x0000)
#define MCFEPORT_EPPAR_EPPA7_RISING    (0x4000)
#define MCFEPORT_EPPAR_EPPA7_FALLING   (0x8000)
#define MCFEPORT_EPPAR_EPPA7_BOTH      (0xC000)
#define MCFEPORT_EPPAR_EPPA6_LEVEL     (0x0000)
#define MCFEPORT_EPPAR_EPPA6_RISING    (0x1000)
#define MCFEPORT_EPPAR_EPPA6_FALLING   (0x2000)
#define MCFEPORT_EPPAR_EPPA6_BOTH      (0x3000)
#define MCFEPORT_EPPAR_EPPA5_LEVEL     (0x0000)
#define MCFEPORT_EPPAR_EPPA5_RISING    (0x0400)
#define MCFEPORT_EPPAR_EPPA5_FALLING   (0x0800)
#define MCFEPORT_EPPAR_EPPA5_BOTH      (0x0C00)
#define MCFEPORT_EPPAR_EPPA4_LEVEL     (0x0000)
#define MCFEPORT_EPPAR_EPPA4_RISING    (0x0100)
#define MCFEPORT_EPPAR_EPPA4_FALLING   (0x0200)
#define MCFEPORT_EPPAR_EPPA4_BOTH      (0x0300)
#define MCFEPORT_EPPAR_EPPA3_LEVEL     (0x0000)
#define MCFEPORT_EPPAR_EPPA3_RISING    (0x0040)
#define MCFEPORT_EPPAR_EPPA3_FALLING   (0x0080)
#define MCFEPORT_EPPAR_EPPA3_BOTH      (0x00C0)
#define MCFEPORT_EPPAR_EPPA2_LEVEL     (0x0000)
#define MCFEPORT_EPPAR_EPPA2_RISING    (0x0010)
#define MCFEPORT_EPPAR_EPPA2_FALLING   (0x0020)
#define MCFEPORT_EPPAR_EPPA2_BOTH      (0x0030)
#define MCFEPORT_EPPAR_EPPA1_LEVEL     (0x0000)
#define MCFEPORT_EPPAR_EPPA1_RISING    (0x0004)
#define MCFEPORT_EPPAR_EPPA1_FALLING   (0x0008)
#define MCFEPORT_EPPAR_EPPA1_BOTH      (0x000C)
#define MCFEPORT_EPPAR_EPPA0_LEVEL     (0x0000)
#define MCFEPORT_EPPAR_EPPA0_RISING    (0x0001)
#define MCFEPORT_EPPAR_EPPA0_FALLING   (0x0002)
#define MCFEPORT_EPPAR_EPPA0_BOTH      (0x0003)

/* Bit definitions and macros for EPIER */
#define MCFEPORT_EPIER_EPIE0   (0x01)
#define MCFEPORT_EPIER_EPIE1   (0x02)
#define MCFEPORT_EPIER_EPIE2   (0x04)
#define MCFEPORT_EPIER_EPIE3   (0x08)
#define MCFEPORT_EPIER_EPIE4   (0x10)
#define MCFEPORT_EPIER_EPIE5   (0x20)
#define MCFEPORT_EPIER_EPIE6   (0x40)
#define MCFEPORT_EPIER_EPIE7   (0x80)

/* Bit definitions and macros for EPFR */
#define MCFEPORT_EPFR_EPF0     (0x01)
#define MCFEPORT_EPFR_EPF1     (0x02)
#define MCFEPORT_EPFR_EPF2     (0x04)
#define MCFEPORT_EPFR_EPF3     (0x08)
#define MCFEPORT_EPFR_EPF4     (0x10)
#define MCFEPORT_EPFR_EPF5     (0x20)
#define MCFEPORT_EPFR_EPF6     (0x40)
#define MCFEPORT_EPFR_EPF7     (0x80)

#endif /* __FSL_DSPI_SLAVE_H__ */
