// SPDX-License-Identifier: GPL-2.0
/*
 *	m5441x_dlc_next.c -- support for Coldfire m5441x DLC Next board
 *
 *	(C) Copyright Wabtec
 */

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-fsl-dspi.h>
#include <linux/spi/flash.h>
#include <linux/dma-mapping.h>
#include <asm/mcfsim.h>
#include <asm/mcfuart.h>

#define	MCFGPIO_PAR_FBCTL_TA_MASK	0xFC
#define MCFGPIO_PAR_FBCTL_TA_NFC_RB	0x01

/* ethernet mac addresses from uboot */
unsigned char uboot_enet0[6] = {0};
EXPORT_SYMBOL(uboot_enet0);
unsigned char uboot_enet1[6] = {0};
EXPORT_SYMBOL(uboot_enet1);

static u64 mcf_uart_mask = DMA_BIT_MASK(32);

static struct resource mcf_uart2_resource[] = {
	[0] = {
		.start = MCFUART_BASE2,
		.end   = MCFUART_BASE2 + 0x3fff,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = 6,
		.end   = 7,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.start = MCF_IRQ_UART2,
		.end   = MCF_IRQ_UART2,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device mcf_uart2 = {
	.name			= "mcfuart",
	.id			= 2,
	.num_resources = ARRAY_SIZE(mcf_uart2_resource),
	.resource = mcf_uart2_resource,
	.dev = {
		.dma_mask = &mcf_uart_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource mcf_uart6_resource[] = {
	[0] = {
		.start = MCFUART_BASE6,
		.end   = MCFUART_BASE6 + 0x3fff,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = 22,
		.end   = 23,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.start = MCF_IRQ_UART6,
		.end   = MCF_IRQ_UART6,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device mcf_uart6 = {
	.name			= "mcfuart",
	.id			= 6,
	.num_resources = ARRAY_SIZE(mcf_uart6_resource),
	.resource = mcf_uart6_resource,
	.dev = {
		.dma_mask = &mcf_uart_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource mcf_dmatmr2_resource[] = {
	[0] = {
		.start = MCFDMATIMER_BASE2,
		.end   = MCFDMATIMER_BASE2 + 0xf,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MCFDMATIMER_IRQ_DTIM2,
		.end   = MCFDMATIMER_IRQ_DTIM2,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = MCFDMATIMER_IRQ_PRIO2,
		.end   = MCFDMATIMER_IRQ_PRIO2,
		.flags = IORESOURCE_REG,
		.name = "prio_reg",
	}
};

static struct platform_device mcf_dmatmr2 = {
	.name			= "mcftmr",
	.id			= 2,
	.num_resources = ARRAY_SIZE(mcf_dmatmr2_resource),
	.resource = mcf_dmatmr2_resource,
};

static struct mtd_partition dlc_next_nor_partitions[] = {
	{
		.name = "w25q01",
		.size = (1024*64*2048),
		.offset = 0x00000000,
		.mask_flags = 0,
	}
};

static struct flash_platform_data dlc_next_spi_flash_data = {
	.name = "Micron mt25ql01G SPI Flash chip",
	.parts = dlc_next_nor_partitions,
	.nr_parts = ARRAY_SIZE(dlc_next_nor_partitions),
	.type = "w25q01jv",
};

static struct spi_board_info dlc_next_board_info[] __initdata = {
	{
		.modalias = "spi-nor",
		.max_speed_hz = 50000000,
		.bus_num = 0,
		.chip_select = 0,
		.platform_data = &dlc_next_spi_flash_data,
		.mode = SPI_MODE_0,
	}
};

static struct resource nfc_resources[] = {
	{
		.start = MCF_NFC_BASE,
		.end = MCF_NFC_BASE + MCF_NFC_SIZE + 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MCF_NFC_ISR,
		.end = MCF_NFC_ISR,
		.flags = IORESOURCE_IRQ,
	},

};

/* flash bbt descriptors */
static uint8_t dlcnext_nand_bbt_pattern[] = { 'B', 'b', 't', '0' };
static uint8_t dlcnext_nand_mirror_pattern[] = { '1', 't', 'b', 'B' };

static struct nand_bbt_descr dlcnext_nand_bbt_main_descr = {
	.options	= NAND_BBT_LASTBLOCK | /*NAND_BBT_CREATE |*/
			  NAND_BBT_WRITE | NAND_BBT_2BIT | NAND_BBT_ABSPAGE |
			  NAND_BBT_VERSION,
	.offs		= 11,
	.len		= 4,
	.veroffs	= 15,
	.maxblocks	= 4,
	.pattern	= dlcnext_nand_bbt_pattern,
	.pages[0] 	= 262080,
};

static struct nand_bbt_descr dlcnext_nand_bbt_mirror_descr = {
	.options	= NAND_BBT_LASTBLOCK | NAND_BBT_CREATE |
			  NAND_BBT_WRITE | NAND_BBT_2BIT | NAND_BBT_ABSPAGE |
			  NAND_BBT_VERSION,
	.offs		= 11,
	.len		= 4,
	.veroffs	= 15,
	.maxblocks	= 4,
	.pattern	= dlcnext_nand_mirror_pattern,
	.pages[0] 	= 262016,
};

static struct nand_chip dlcnext_nand_pdata = {
	.options 			= NAND_NEED_READRDY | NAND_CACHEPRG,
	.ecc.engine_type	= NAND_ECC_ENGINE_TYPE_ON_HOST,
	.ecc.placement		= NAND_ECC_PLACEMENT_OOB,
	.ecc.strength		= 24,
	.ecc.size			= 0x0800,
	.bbt_options		= NAND_BBT_USE_FLASH,
	.bbt_td				= &dlcnext_nand_bbt_main_descr,
	.bbt_md				= &dlcnext_nand_bbt_mirror_descr,
};

static struct platform_device nfc_device = {
	.name           = "mcf5441x-nfc",
	.id             = -1,
	.dev			= {
		.platform_data = &dlcnext_nand_pdata,
	},
	.resource       = nfc_resources,
	.num_resources  = ARRAY_SIZE(nfc_resources),
};

static struct i2c_board_info mcf_i2c2_devices[] = {
	{
		I2C_BOARD_INFO("isl1208", 0x6f),
	},
	{
		I2C_BOARD_INFO("24c64", 0x53),
	},
};

/* SPI controller data, SPI (0) */
static struct fsl_dspi_platform_data dspi_spi0_info = {
	.cs_num = 4,
	.bus_num = 0,
	.sck_cs_delay = 100,
	.cs_sck_delay = 100,
	.bits_per_word = 8,
};

static struct resource dspi_spi0_resource[] = {
	[0] = {
		.start = MCFDSPI_BASE0,
		.end   = MCFDSPI_BASE0 + 0xFF,
		.flags = IORESOURCE_MEM,
		},
	[1] = {
		.start = MCF_IRQ_DSPI0,
		.end   = MCF_IRQ_DSPI0,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 dlc_next_dspi_mask = DMA_BIT_MASK(32);

/* SPI controller, id = bus number */
static struct platform_device dspi_spi0_device = {
	.name = "fsl-dspi",
	.id = 0,
	.num_resources = ARRAY_SIZE(dspi_spi0_resource),
	.resource = dspi_spi0_resource,
	.dev = {
		.platform_data = &dspi_spi0_info,
		.dma_mask = &dlc_next_dspi_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct coldfire_spi_slave spi1_slave_info = {
	.bus_num = 1,
	.irq_source = MCFINT1_DSPI1,
	.irq_vector = MCFINT1_VECBASE + MCFINT1_DSPI1,
	.irq_mask = (1 << (MCFINT1_DSPI1 - 32)),
	.irq_lp = 0x2,		/* irq level */
};

#define MCF_INTC1_ICR54         0xfc04c076
#define MCF_INTC1_IMRH          0xfc04c008

static struct resource coldfire_spi1_resources[] = {
	[0] = {
		.name = "spi-slave-par",
		.start = MCFGPIO_PAR_SDHCH,	/* PAR_ESDHCH */
		.end = MCFGPIO_PAR_SDHCL,	/* PAR_ESDHCL */
		.flags = IORESOURCE_MEM
	},

	[1] = {
		.name = "spi-slave-module",
		.start = MCFDSPI_BASE1,	/* DSPI MCR Base */
		.end = MCFDSPI_BASE1 + 0xc0,	/* DSPI mem map end */
		.flags = IORESOURCE_MEM
	},

	[2] = {
		.name = "spi-slave-int-level",
		.start = MCF_INTC1_ICR54,	/* ICR start */
		.end = MCF_INTC1_ICR54,	/* ICR end */
		.flags = IORESOURCE_MEM
	},

	[3] = {
		.name = "spi-slave-int-mask",
		.start = MCF_INTC1_IMRH,	/* IMRL */
		.end = MCF_INTC1_IMRH,	/* IMRL */
		.flags = IORESOURCE_MEM
	}
};

static struct platform_device dspi_spi1_device = {
	.name = "mcf-spi-slave",
	.id = -1,
	.resource = coldfire_spi1_resources,
	.num_resources = ARRAY_SIZE(coldfire_spi1_resources),
	.dev = {
		.platform_data = &spi1_slave_info,
	}
};

static struct platform_device *dlc_next_devices[] __initdata = {
	&mcf_uart2,
	&mcf_uart6,
	&mcf_dmatmr2,
	&nfc_device,
	&dspi_spi0_device,
	&dspi_spi1_device,
};

#define MCFGPIO_PAR_DSPIO_SCK_MASK		(0xF3)
#define MCFGPIO_PAR_DSPIO_SCK_DSPI0SCK		(0x0C)
#define MCFGPIO_PAR_DSPIO_SOUT_MASK		(0xCF)
#define MCFGPIO_PAR_DSPIO_SOUT_DSPI0SOUT	(0x30)
#define MCFGPIO_PAR_DSPIO_SIN_MASK		(0x3F)
#define MCFGPIO_PAR_DSPIO_SIN_DSPI0SIN		(0xC0)
#define MCFGPIO_PAR_DSPIO_PCS0_MASK		(0xFC)
#define MCFGPIO_PAR_DSPIO_PCS0_DSPI0PCS0	(0x03)

static int __init init_m5441x_dlc_next(void)
{
	/* Configure DSPI0 */
	/*MCFGPIO_PAR_DSPI0WH =
		(MCFGPIO_PAR_DSPI0WH & MCFGPIO_PAR_DSPI0_SCK_MASK) |
		MCFGPIO_PAR_DSPI0_SCK_DSPI0SCK;*/
	u8 dspi0wh = __raw_readb(MCFGPIO_PAR_DSPIOWH);
	dspi0wh &= MCFGPIO_PAR_DSPIO_SCK_MASK;
	dspi0wh |= MCFGPIO_PAR_DSPIO_SCK_DSPI0SCK;
	__raw_writeb(dspi0wh, MCFGPIO_PAR_DSPIOWH);

	/* MCFGPIO_PAR_DSPI0WH =
		(MCFGPIO_PAR_DSPI0WH & MCFGPIO_PAR_DSPI0_SOUT_MASK) |
		MCFGPIO_PAR_DSPI0_SOUT_DSPI0SOUT;*/
	dspi0wh = __raw_readb(MCFGPIO_PAR_DSPIOWH);
	dspi0wh &= MCFGPIO_PAR_DSPIO_SOUT_MASK;
	dspi0wh |= MCFGPIO_PAR_DSPIO_SOUT_DSPI0SOUT;
	__raw_writeb(dspi0wh, MCFGPIO_PAR_DSPIOWH);

	/*MCFGPIO_PAR_DSPI0WH =
		(MCFGPIO_PAR_DSPI0WH & MCFGPIO_PAR_DSPI0_SIN_MASK) |
		MCFGPIO_PAR_DSPI0_SIN_DSPI0SIN;*/
	dspi0wh = __raw_readb(MCFGPIO_PAR_DSPIOWH);
	dspi0wh &= MCFGPIO_PAR_DSPIO_SIN_MASK;
	dspi0wh |= MCFGPIO_PAR_DSPIO_SIN_DSPI0SIN;
	__raw_writeb(dspi0wh, MCFGPIO_PAR_DSPIOWH);

	/*MCFGPIO_PAR_DSPI0WH =
		(MCFGPIO_PAR_DSPI0WH & MCFGPIO_PAR_DSPI0_PCS0_MASK) |
		MCFGPIO_PAR_DSPI0_PCS0_DSPI0PCS0;*/
    /* Set DSPI0_PCS1 (PC0 /FLASH_SPI_WP) as GPIO */
	dspi0wh = __raw_readb(MCFGPIO_PAR_DSPIOWH);
	dspi0wh &= MCFGPIO_PAR_DSPIO_PCS0_MASK;
	dspi0wh |= MCFGPIO_PAR_DSPIO_PCS0_DSPI0PCS0;
	__raw_writeb(dspi0wh, MCFGPIO_PAR_DSPIOWH);

	/* MCFGPIO_PAR_DSPI0WL = 0x00; */
	__raw_writeb(0x00, MCFGPIO_PAR_DSPIOWL);

	/* DSPI1 configuration */
	/* MCF_PM_PPMCR0 = 0xf; */
	/* MCF_GPIO_PAR_ESDHCH = 0x55; */
	/* MCF_GPIO_PAR_ESDHCL = 0x05; */
	/* MCF_GPIO_SRCR_IRQ0 = 3; */
	/* MCF_GPIO_SRCR_SDHC = 3; */
	__raw_writeb(0xf, MCFPM_PPMCR0);
	__raw_writeb(0x55, MCFGPIO_PAR_SDHCH);
	__raw_writeb(0x05, MCFGPIO_PAR_SDHCL);
	__raw_writeb(3, MCFGPIO_SRCR_IRQ0);
	__raw_writeb(3, MCFGPIO_SRCR_SDHC);

	/* Board gpio setup */
	platform_add_devices(dlc_next_devices, ARRAY_SIZE(dlc_next_devices));

	/*
	 * MCF_GPIO_PAR_FBCTL &= (MCF_GPIO_PAR_FBCTL_TA_MASK);
	 * MCF_GPIO_PAR_FBCTL |= MCF_GPIO_PAR_FBCTL_TA_NFC_RB;
	 */
	u8 fbctl = __raw_readb(MCFGPIO_PAR_FBCTL);
	fbctl &= MCFGPIO_PAR_FBCTL_TA_MASK;
	fbctl |= MCFGPIO_PAR_FBCTL_TA_NFC_RB;
	__raw_writeb(fbctl, MCFGPIO_PAR_FBCTL);

	/* Configure fec interfaces */
	__raw_writeb(0x03, MCFGPIO_PAR_FEC);
	__raw_writeb(0x0f, MCFGPIO_SRCR_FEC);

	printk("Register I2C bus 2 devices\n");
	i2c_register_board_info(2, mcf_i2c2_devices,
		ARRAY_SIZE(mcf_i2c2_devices));

	spi_register_board_info(dlc_next_board_info, ARRAY_SIZE(dlc_next_board_info));

	printk("M5441x DLC Next board init\n");

	return 0;
}

postcore_initcall(init_m5441x_dlc_next);
