#ifndef __FSL_DSPI_SLAVE_H__
#define __FSL_DSPI_SLAVE_H__

/*
 * Local Data Structures
 */

struct DSPI_MCR {
	unsigned master:1;
	unsigned cont_scke:1;
	unsigned dconf:2;
	unsigned frz:1;
	unsigned mtfe:1;
	unsigned pcsse:1;
	unsigned rooe:1;
	unsigned pcsis:8;
	unsigned reserved15:1;
	unsigned mdis:1;
	unsigned dis_tx:1;
	unsigned dis_rxf:1;
	unsigned clr_tx:1;
	unsigned clr_rxf:1;
	unsigned smpl_pt:2;
	unsigned reserved71:7;
	unsigned halt:1;
};

struct DSPI_CTAR {
	unsigned dbr:1;
	unsigned fmsz:4;
	unsigned cpol:1;
	unsigned cpha:1;
	unsigned lsbfe:1;
	unsigned pcssck:2;
	unsigned pasc:2;
	unsigned pdt:2;
	unsigned pbr:2;
	unsigned cssck:4;
	unsigned asc:4;
	unsigned dt:4;
	unsigned br:4;
};

struct chip_data {
	/* dspi data */
	union {
		u32 mcr_val;
		struct DSPI_MCR mcr;
	};
	union {
		u32 ctar_val;
		struct DSPI_CTAR ctar;
	};
};

typedef enum {
	DSPI_SLAVE_STATE_IDLE,
	DSPI_SLAVE_STATE_RX,
	DSPI_SLAVE_STATE_RX_DONE,
	DSPI_SLAVE_STATE_TX,
	DSPI_SLAVE_STATE_RESTART,
} dspi_slave_state_t;

struct dspi_slave_perf {
	ktime_t irq_received;
	ktime_t frame_sent;
	ktime_t wait_next_frame;
	u64 frame_number;
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
	struct siginfo sinfo;
	struct task_struct *task_user;

	/* Current message transfer state info */
	struct chip_data *cur_chip;
	u8 cs;

	volatile u32 mcr;		/* DSPI MCR register */
	volatile u32 ctar;		/* DSPI CTAR register */
	volatile u32 dspi_dtfr;		/* DSPI DTFR register */
	volatile u32 dspi_drfr;		/* DSPI DRFR register */
	volatile u32 dspi_rser;		/* DSPI RSER register */
	volatile u32 dspi_sr;		/* DSPI status register */

	u8  *int_icr;	   /* Interrupt level and priority register */
	u32 *int_mr;       /* Interrupt mask register */

	u8 stream_restarted; /* Boolean to unblock read() during failover */

	/* mmap */
	u16 *mmap_buffer;
	size_t mmap_buffer_size;

	/* locking */
	struct mutex lock;

	struct completion read_complete;

	struct task_struct	*kthread;
	dspi_slave_state_t	state;

	/* Add time measurement variables */
	struct dspi_slave_perf frame_perf;
	ktime_t average_frame_time;
	ktime_t min_frame_time;
	ktime_t max_frame_time;
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
