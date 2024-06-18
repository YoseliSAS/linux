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
	.type = "mt25ql01G",
};

static struct spi_board_info dlc_next_board_info[] __initdata = {
	{
		.modalias = "m25p80",
		.max_speed_hz = 5000000,
		.bus_num = 0,
		.chip_select = 1,
		.platform_data = &dlc_next_spi_flash_data,
		.mode = SPI_MODE_3,
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

static struct platform_device *dlc_next_devices[] __initdata = {
	&nfc_device,
};

static int __init init_m5441x_dlc_next(void)
{
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
