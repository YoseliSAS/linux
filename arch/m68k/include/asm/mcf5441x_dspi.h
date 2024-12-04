/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Lanttor.Guo@freescale.com
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __MCF5441X_DSPI_H__
#define __MCF5441X_DSPI_H__

/*********************************************************************
*
* DMA Serial Peripheral Interface (DSPI)
*
*********************************************************************/

/* Register read/write macros */
/* DSPI Module Configuration Register */
#define MCF_DSPI_DMCR           0xFC05C000
#define MCF_DSPI1_DMCR          0xFC03C000
/* DSPI Transfer Count Register */
#define MCF_DSPI_DTCR           0xFC05C008
#define MCF_DSPI1_DTCR          0xFC03C008
/* DSPI Clock and Transfer Attributes Register */
#define MCF_DSPI_DCTAR0         0xFC05C00C
#define MCF_DSPI1_DCTAR0        0xFC03C00C
/* DSPI Clock and Transfer Attributes Register */
#define MCF_DSPI_DCTAR1         0xFC05C010
/* DSPI Clock and Transfer Attributes Register */
#define MCF_DSPI_DCTAR2         0xFC05C014
/* DSPI Clock and Transfer Attributes Register */
#define MCF_DSPI_DCTAR3         0xFC05C018
/* DSPI Clock and Transfer Attributes Register */
#define MCF_DSPI_DCTAR4         0xFC05C01C
/* DSPI Clock and Transfer Attributes Register */
#define MCF_DSPI_DCTAR5         0xFC05C020
/* DSPI Clock and Transfer Attributes Register */
#define MCF_DSPI_DCTAR6         0xFC05C024
/* DSPI Clock and Transfer Attributes Register */
#define MCF_DSPI_DCTAR7         0xFC05C028
/* DSPI Status Register */
#define MCF_DSPI_DSR            0xFC05C02C
#define MCF_DSPI1_DSR           0xFC03C02C
/* DSPI DMA/Interrupt Request Select and Enable Register */
#define MCF_DSPI_DRSER          0xFC05C030
#define MCF_DSPI1_DRSER         0xFC03C030
/* DSPI Transmit FIFO Register */
#define MCF_DSPI_DTFR           0xFC05C034
#define MCF_DSPI1_DTFR          0xFC03C034
/* DSPI Receive FIFO Register */
#define MCF_DSPI_DRFR           0xFC05C038
#define MCF_DSPI1_DRFR          0xFC03C038
/* DSPI Transmit FIFO Debugging Registers */
#define MCF_DSPI_DTFDR0         0xFC05C03C
/* DSPI Transmit FIFO Debugging Registers */
#define MCF_DSPI_DTFDR1         0xFC05C040
/* DSPI Transmit FIFO Debugging Registers */
#define MCF_DSPI_DTFDR2         0xFC05C044
/* DSPI Transmit FIFO Debugging Registers */
#define MCF_DSPI_DTFDR3         0xFC05C048
/* DSPI Receive FIFO Debugging Registers */
#define MCF_DSPI_DRFDR0         0xFC05C07C
/* DSPI Receive FIFO Debugging Registers */
#define MCF_DSPI_DRFDR1         0xFC05C080
/* DSPI Receive FIFO Debugging Registers */
#define MCF_DSPI_DRFDR2         0xFC05C084
/* DSPI Receive FIFO Debugging Registers */
#define MCF_DSPI_DRFDR3         0xFC05C088

/* Parameterized register read/write macros for multiple registers */
/* DSPI Clock and Transfer Attributes Register */
#define MCF_DSPI_DCTAR(x)       0xFC05C00C+((x)*0x004)
/* DSPI Transmit FIFO Debugging Registers */
#define MCF_DSPI_DTFDR(x)       0xFC05C03C+((x)*0x004)
/* DSPI Receive FIFO Debugging Registers */
#define MCF_DSPI_DRFDR(x)       0xFC05C07C+((x)*0x004)

/* Bit definitions and macros for DMCR */
/* Halt -- stops and starts DSPI transfers */
#define MCF_DSPI_DMCR_HALT          (0x00000001)
/* Sample point selection */
#define MCF_DSPI_DMCR_SMPLPT(x)     (((x)&0x00000003)<<8)
/* Clear receive FIFO */
#define MCF_DSPI_DMCR_CLRRXF        (0x00000400)
/* Clear transmit FIFO */
#define MCF_DSPI_DMCR_CLRTXF        (0x00000800)
/* Disable receive FIFO */
#define MCF_DSPI_DMCR_DISRXF        (0x00001000)
/* Disable transmit FIFO */
#define MCF_DSPI_DMCR_DISTXF        (0x00002000)
/* Module Disable */
#define MCF_DSPI_DMCR_MDIS          (0x00004000)
/* Peripheral chip-select 0 inactive state */
#define MCF_DSPI_DMCR_PCSIS0        (0x00010000)
/* Peripheral chip-select 1 inactive state */
#define MCF_DSPI_DMCR_PCSIS1        (0x00020000)
/* Peripheral chip-select 2 inactive state */
#define MCF_DSPI_DMCR_PCSIS2        (0x00040000)
/* Peripheral chip-select 3 inactive state */
#define MCF_DSPI_DMCR_PCSIS3        (0x00080000)
/* Peripheral chip-select 4 inactive state */
#define MCF_DSPI_DMCR_PCSIS4        (0x00100000)
/* Peripheral chip-select 5 inactive state */
#define MCF_DSPI_DMCR_PCSIS5        (0x00200000)
/* Peripheral chip-select 6 inactive state */
#define MCF_DSPI_DMCR_PCSIS6        (0x00400000)
/* Peripheral chip-select 7 inactive state */
#define MCF_DSPI_DMCR_PCSIS7        (0x00800000)
/* Receive FIFO overflow overwrite enable */
#define MCF_DSPI_DMCR_ROOE          (0x01000000)
/* Peripheral chip select strobe enable */
#define MCF_DSPI_DMCR_PCSSE         (0x02000000)
/* Modified timing format enable */
#define MCF_DSPI_DMCR_MTFE          (0x04000000)
/* Freeze */
#define MCF_DSPI_DMCR_FRZ           (0x08000000)
/* DSPI configuration */
#define MCF_DSPI_DMCR_DCONF(x)      (((x)&0x00000003)<<28)
/* Continuous SCK enable */
#define MCF_DSPI_DMCR_CONT_SCKE     (0x40000000)
/* Master/Slave mode select */
#define MCF_DSPI_DMCR_MSTR          (0x80000000)
#define MCF_DSPI_DMCR_DCONF_SPI     (0x00000000)
#define MCF_DSPI_DMCR_PCSIS7_LOW    (0x00000000)
#define MCF_DSPI_DMCR_PCSIS7_HIGH   (0x00800000)
#define MCF_DSPI_DMCR_PCSIS6_LOW    (0x00000000)
#define MCF_DSPI_DMCR_PCSIS6_HIGH   (0x00400000)
#define MCF_DSPI_DMCR_PCSIS5_LOW    (0x00000000)
#define MCF_DSPI_DMCR_PCSIS5_HIGH   (0x00200000)
#define MCF_DSPI_DMCR_PCSIS4_LOW    (0x00000000)
#define MCF_DSPI_DMCR_PCSIS4_HIGH   (0x00100000)
#define MCF_DSPI_DMCR_PCSIS3_LOW    (0x00000000)
#define MCF_DSPI_DMCR_PCSIS3_HIGH   (0x00080000)
#define MCF_DSPI_DMCR_PCSIS2_LOW    (0x00000000)
#define MCF_DSPI_DMCR_PCSIS2_HIGH   (0x00040000)
#define MCF_DSPI_DMCR_PCSIS1_LOW    (0x00000000)
#define MCF_DSPI_DMCR_PCSIS1_HIGH   (0x00020000)
#define MCF_DSPI_DMCR_PCSIS0_LOW    (0x00000000)
#define MCF_DSPI_DMCR_PCSIS0_HIGH   (0x00010000)

/* Bit definitions and macros for DTCR */
/* SPI transfer count */
#define MCF_DSPI_DTCR_SPI_TCNT(x)   (((x)&0x0000FFFF)<<16)

/* Bit definitions and macros for DCTAR group */
/* Baud rate scaler */
#define MCF_DSPI_DCTAR_BR(x)                (((x)&0x0000000F))
/* Delay after transfer scaler */
#define MCF_DSPI_DCTAR_DT(x)                (((x)&0x0000000F)<<4)
/* After SCK delay scaler */
#define MCF_DSPI_DCTAR_ASC(x)               (((x)&0x0000000F)<<8)
/* PCS to SCK delay scaler */
#define MCF_DSPI_DCTAR_CSSCK(x)             (((x)&0x0000000F)<<12)
/* Baud rate prescaler */
#define MCF_DSPI_DCTAR_PBR(x)               (((x)&0x00000003)<<16)
/* Delay after transfer prescaler */
#define MCF_DSPI_DCTAR_PDT(x)               (((x)&0x00000003)<<18)
/* After SCK delay prescaler */
#define MCF_DSPI_DCTAR_PASC(x)              (((x)&0x00000003)<<20)
/* PCS to SCK delay prescaler */
#define MCF_DSPI_DCTAR_PCSSCK(x)            (((x)&0x00000003)<<22)
/* LSB first enable */
#define MCF_DSPI_DCTAR_LSBFE                (0x01000000)
/* Clock phase */
#define MCF_DSPI_DCTAR_CPHA                 (0x02000000)
/* Clock polarity */
#define MCF_DSPI_DCTAR_CPOL                 (0x04000000)
/* Frame size */
#define MCF_DSPI_DCTAR_FMSZ(x)              (((x)&0x0000000F)<<27)
/* Double baud rate */
#define MCF_DSPI_DCTAR_DBR                  (0x80000000)
#define MCF_DSPI_DCTAR_CPOL_LOW             (0x00000000)
#define MCF_DSPI_DCTAR_CPOL_HIGH            (0x04000000)
#define MCF_DSPI_DCTAR_CPHA_LATCH_RISING    (0x00000000)
#define MCF_DSPI_DCTAR_CPHA_LATCH_FALLING   (0x02000000)
#define MCF_DSPI_DCTAR_PCSSCK_1CLK          (0x00000000)
#define MCF_DSPI_DCTAR_PCSSCK_3CLK          (0x00400000)
#define MCF_DSPI_DCTAR_PCSSCK_5CLK          (0x00800000)
#define MCF_DSPI_DCTAR_PCSSCK_7CLK          (0x00C00000)
#define MCF_DSPI_DCTAR_PASC_1CLK            (0x00000000)
#define MCF_DSPI_DCTAR_PASC_3CLK            (0x00100000)
#define MCF_DSPI_DCTAR_PASC_5CLK            (0x00200000)
#define MCF_DSPI_DCTAR_PASC_7CLK            (0x00300000)
#define MCF_DSPI_DCTAR_PDT_1CLK             (0x00000000)
#define MCF_DSPI_DCTAR_PDT_3CLK             (0x00040000)
#define MCF_DSPI_DCTAR_PDT_5CLK             (0x00080000)
#define MCF_DSPI_DCTAR_PDT_7CLK             (0x000C0000)
#define MCF_DSPI_DCTAR_PBR_2CLK             (0x00000000)
#define MCF_DSPI_DCTAR_PBR_3CLK             (0x00010000)
#define MCF_DSPI_DCTAR_PBR_5CLK             (0x00020000)
#define MCF_DSPI_DCTAR_PBR_7CLK             (0x00030000)

/* Bit definitions and macros for DCTAR0 */
/* Baud rate scaler */
#define MCF_DSPI_DCTAR0_BR(x)                   (((x)&0x0000000F))
/* Delay after transfer scaler */
#define MCF_DSPI_DCTAR0_DT(x)                   (((x)&0x0000000F)<<4)
/* After SCK delay scaler */
#define MCF_DSPI_DCTAR0_ASC(x)                  (((x)&0x0000000F)<<8)
/* PCS to SCK delay scaler */
#define MCF_DSPI_DCTAR0_CSSCK(x)                (((x)&0x0000000F)<<12)
/* Baud rate prescaler */
#define MCF_DSPI_DCTAR0_PBR(x)                  (((x)&0x00000003)<<16)
/* Delay after transfer prescaler */
#define MCF_DSPI_DCTAR0_PDT(x)                  (((x)&0x00000003)<<18)
/* After SCK delay prescaler */
#define MCF_DSPI_DCTAR0_PASC(x)                 (((x)&0x00000003)<<20)
/* PCS to SCK delay prescaler */
#define MCF_DSPI_DCTAR0_PCSSCK(x)               (((x)&0x00000003)<<22)
/* LSB first enable */
#define MCF_DSPI_DCTAR0_LSBFE                   (0x01000000)
/* Clock phase */
#define MCF_DSPI_DCTAR0_CPHA                    (0x02000000)
/* Clock polarity */
#define MCF_DSPI_DCTAR0_CPOL                    (0x04000000)
/* Frame size */
#define MCF_DSPI_DCTAR0_FMSZ(x)                 (((x)&0x0000000F)<<27)
/* Double baud rate */
#define MCF_DSPI_DCTAR0_DBR                     (0x80000000)
#define MCF_DSPI_DCTAR0_CPOL_LOW                (0x00000000)
#define MCF_DSPI_DCTAR0_CPOL_HIGH               (0x04000000)
#define MCF_DSPI_DCTAR0_CPHA_LATCH_RISING       (0x00000000)
#define MCF_DSPI_DCTAR0_CPHA_LATCH_FALLING      (0x02000000)
#define MCF_DSPI_DCTAR0_PCSSCK_1CLK             (0x00000000)
#define MCF_DSPI_DCTAR0_PCSSCK_3CLK             (0x00400000)
#define MCF_DSPI_DCTAR0_PCSSCK_5CLK             (0x00800000)
#define MCF_DSPI_DCTAR0_PCSSCK_7CLK             (0x00C00000)
#define MCF_DSPI_DCTAR0_PASC_1CLK               (0x00000000)
#define MCF_DSPI_DCTAR0_PASC_3CLK               (0x00100000)
#define MCF_DSPI_DCTAR0_PASC_5CLK               (0x00200000)
#define MCF_DSPI_DCTAR0_PASC_7CLK               (0x00300000)
#define MCF_DSPI_DCTAR0_PDT_1CLK                (0x00000000)
#define MCF_DSPI_DCTAR0_PDT_3CLK                (0x00040000)
#define MCF_DSPI_DCTAR0_PDT_5CLK                (0x00080000)
#define MCF_DSPI_DCTAR0_PDT_7CLK                (0x000C0000)
#define MCF_DSPI_DCTAR0_PBR_2CLK                (0x00000000)
#define MCF_DSPI_DCTAR0_PBR_3CLK                (0x00010000)
#define MCF_DSPI_DCTAR0_PBR_5CLK                (0x00020000)
#define MCF_DSPI_DCTAR0_PBR_7CLK                (0x00030000)

/* Bit definitions and macros for DCTAR1 */
/* Baud rate scaler */
#define MCF_DSPI_DCTAR1_BR(x)       (((x)&0x0000000F))
/* Delay after transfer scaler */
#define MCF_DSPI_DCTAR1_DT(x)       (((x)&0x0000000F)<<4)
/* After SCK delay scaler */
#define MCF_DSPI_DCTAR1_ASC(x)      (((x)&0x0000000F)<<8)
/* PCS to SCK delay scaler */
#define MCF_DSPI_DCTAR1_CSSCK(x)    (((x)&0x0000000F)<<12)
/* Baud rate prescaler */
#define MCF_DSPI_DCTAR1_PBR(x)      (((x)&0x00000003)<<16)
/* Delay after transfer prescaler */
#define MCF_DSPI_DCTAR1_PDT(x)      (((x)&0x00000003)<<18)
/* After SCK delay prescaler */
#define MCF_DSPI_DCTAR1_PASC(x)     (((x)&0x00000003)<<20)
/* PCS to SCK delay prescaler */
#define MCF_DSPI_DCTAR1_PCSSCK(x)   (((x)&0x00000003)<<22)
/* LSB first enable */
#define MCF_DSPI_DCTAR1_LSBFE       (0x01000000)
/* Clock phase */
#define MCF_DSPI_DCTAR1_CPHA        (0x02000000)
/* Clock polarity */
#define MCF_DSPI_DCTAR1_CPOL        (0x04000000)
/* Frame size */
#define MCF_DSPI_DCTAR1_FMSZ(x)     (((x)&0x0000000F)<<27)
/* Double baud rate */
#define MCF_DSPI_DCTAR1_DBR         (0x80000000)


/* Bit definitions and macros for DCTAR2 */
/* Baud rate scaler */
#define MCF_DSPI_DCTAR2_BR(x)       (((x)&0x0000000F))
/* Delay after transfer scaler */
#define MCF_DSPI_DCTAR2_DT(x)       (((x)&0x0000000F)<<4)
/* After SCK delay scaler */
#define MCF_DSPI_DCTAR2_ASC(x)      (((x)&0x0000000F)<<8)
/* PCS to SCK delay scaler */
#define MCF_DSPI_DCTAR2_CSSCK(x)    (((x)&0x0000000F)<<12)
/* Baud rate prescaler */
#define MCF_DSPI_DCTAR2_PBR(x)      (((x)&0x00000003)<<16)
/* Delay after transfer prescaler */
#define MCF_DSPI_DCTAR2_PDT(x)      (((x)&0x00000003)<<18)
/* After SCK delay prescaler */
#define MCF_DSPI_DCTAR2_PASC(x)     (((x)&0x00000003)<<20)
/* PCS to SCK delay prescaler */
#define MCF_DSPI_DCTAR2_PCSSCK(x)   (((x)&0x00000003)<<22)
/* LSB first enable */
#define MCF_DSPI_DCTAR2_LSBFE       (0x01000000)
/* Clock phase */
#define MCF_DSPI_DCTAR2_CPHA        (0x02000000)
/* Clock polarity */
#define MCF_DSPI_DCTAR2_CPOL        (0x04000000)
/* Frame size */
#define MCF_DSPI_DCTAR2_FMSZ(x)     (((x)&0x0000000F)<<27)
/* Double baud rate */
#define MCF_DSPI_DCTAR2_DBR         (0x80000000)

/* Bit definitions and macros for DCTAR3 */
/* Baud rate scaler */
#define MCF_DSPI_DCTAR3_BR(x)       (((x)&0x0000000F))
/* Delay after transfer scaler */
#define MCF_DSPI_DCTAR3_DT(x)       (((x)&0x0000000F)<<4)
/* After SCK delay scaler */
#define MCF_DSPI_DCTAR3_ASC(x)      (((x)&0x0000000F)<<8)
/* PCS to SCK delay scaler */
#define MCF_DSPI_DCTAR3_CSSCK(x)    (((x)&0x0000000F)<<12)
/* Baud rate prescaler */
#define MCF_DSPI_DCTAR3_PBR(x)      (((x)&0x00000003)<<16)
/* Delay after transfer prescaler */
#define MCF_DSPI_DCTAR3_PDT(x)      (((x)&0x00000003)<<18)
/* After SCK delay prescaler */
#define MCF_DSPI_DCTAR3_PASC(x)     (((x)&0x00000003)<<20)
/* PCS to SCK delay prescaler */
#define MCF_DSPI_DCTAR3_PCSSCK(x)   (((x)&0x00000003)<<22)
/* LSB first enable */
#define MCF_DSPI_DCTAR3_LSBFE       (0x01000000)
/* Clock phase */
#define MCF_DSPI_DCTAR3_CPHA        (0x02000000)
/* Clock polarity */
#define MCF_DSPI_DCTAR3_CPOL        (0x04000000)
/* Frame size */
#define MCF_DSPI_DCTAR3_FMSZ(x)     (((x)&0x0000000F)<<27)
/* Double baud rate */
#define MCF_DSPI_DCTAR3_DBR         (0x80000000)

/* Bit definitions and macros for DCTAR4 */
/* Baud rate scaler */
#define MCF_DSPI_DCTAR4_BR(x)       (((x)&0x0000000F))
/* Delay after transfer scaler */
#define MCF_DSPI_DCTAR4_DT(x)       (((x)&0x0000000F)<<4)
/* After SCK delay scaler */
#define MCF_DSPI_DCTAR4_ASC(x)      (((x)&0x0000000F)<<8)
/* PCS to SCK delay scaler */
#define MCF_DSPI_DCTAR4_CSSCK(x)    (((x)&0x0000000F)<<12)
/* Baud rate prescaler */
#define MCF_DSPI_DCTAR4_PBR(x)      (((x)&0x00000003)<<16)
/* Delay after transfer prescaler */
#define MCF_DSPI_DCTAR4_PDT(x)      (((x)&0x00000003)<<18)
/* After SCK delay prescaler */
#define MCF_DSPI_DCTAR4_PASC(x)     (((x)&0x00000003)<<20)
/* PCS to SCK delay prescaler */
#define MCF_DSPI_DCTAR4_PCSSCK(x)   (((x)&0x00000003)<<22)
/* LSB first enable */
#define MCF_DSPI_DCTAR4_LSBFE       (0x01000000)
/* Clock phase */
#define MCF_DSPI_DCTAR4_CPHA        (0x02000000)
/* Clock polarity */
#define MCF_DSPI_DCTAR4_CPOL        (0x04000000)
/* Frame size */
#define MCF_DSPI_DCTAR4_FMSZ(x)     (((x)&0x0000000F)<<27)
/* Double baud rate */
#define MCF_DSPI_DCTAR4_DBR         (0x80000000)

/* Bit definitions and macros for DCTAR5 */
/* Baud rate scaler */
#define MCF_DSPI_DCTAR5_BR(x)       (((x)&0x0000000F))
/* Delay after transfer scaler */
#define MCF_DSPI_DCTAR5_DT(x)       (((x)&0x0000000F)<<4)
/* After SCK delay scaler */
#define MCF_DSPI_DCTAR5_ASC(x)      (((x)&0x0000000F)<<8)
/* PCS to SCK delay scaler */
#define MCF_DSPI_DCTAR5_CSSCK(x)    (((x)&0x0000000F)<<12)
/* Baud rate prescaler */
#define MCF_DSPI_DCTAR5_PBR(x)      (((x)&0x00000003)<<16)
/* Delay after transfer prescaler */
#define MCF_DSPI_DCTAR5_PDT(x)      (((x)&0x00000003)<<18)
/* After SCK delay prescaler */
#define MCF_DSPI_DCTAR5_PASC(x)     (((x)&0x00000003)<<20)
/* PCS to SCK delay prescaler */
#define MCF_DSPI_DCTAR5_PCSSCK(x)   (((x)&0x00000003)<<22)
/* LSB first enable */
#define MCF_DSPI_DCTAR5_LSBFE       (0x01000000)
/* Clock phase */
#define MCF_DSPI_DCTAR5_CPHA        (0x02000000)
/* Clock polarity */
#define MCF_DSPI_DCTAR5_CPOL        (0x04000000)
/* Frame size */
#define MCF_DSPI_DCTAR5_FMSZ(x)     (((x)&0x0000000F)<<27)
/* Double baud rate */
#define MCF_DSPI_DCTAR5_DBR         (0x80000000)

/* Bit definitions and macros for DCTAR6 */
/* Baud rate scaler */
#define MCF_DSPI_DCTAR6_BR(x)       (((x)&0x0000000F))
/* Delay after transfer scaler */
#define MCF_DSPI_DCTAR6_DT(x)       (((x)&0x0000000F)<<4)
/* After SCK delay scaler */
#define MCF_DSPI_DCTAR6_ASC(x)      (((x)&0x0000000F)<<8)
/* PCS to SCK delay scaler */
#define MCF_DSPI_DCTAR6_CSSCK(x)    (((x)&0x0000000F)<<12)
/* Baud rate prescaler */
#define MCF_DSPI_DCTAR6_PBR(x)      (((x)&0x00000003)<<16)
/* Delay after transfer prescaler */
#define MCF_DSPI_DCTAR6_PDT(x)      (((x)&0x00000003)<<18)
/* After SCK delay prescaler */
#define MCF_DSPI_DCTAR6_PASC(x)     (((x)&0x00000003)<<20)
/* PCS to SCK delay prescaler */
#define MCF_DSPI_DCTAR6_PCSSCK(x)   (((x)&0x00000003)<<22)
/* LSB first enable */
#define MCF_DSPI_DCTAR6_LSBFE       (0x01000000)
/* Clock phase */
#define MCF_DSPI_DCTAR6_CPHA        (0x02000000)
/* Clock polarity */
#define MCF_DSPI_DCTAR6_CPOL        (0x04000000)
/* Frame size */
#define MCF_DSPI_DCTAR6_FMSZ(x)     (((x)&0x0000000F)<<27)
/* Double baud rate */
#define MCF_DSPI_DCTAR6_DBR         (0x80000000)

/* Bit definitions and macros for DCTAR7 */
/* Baud rate scaler */
#define MCF_DSPI_DCTAR7_BR(x)       (((x)&0x0000000F))
/* Delay after transfer scaler */
#define MCF_DSPI_DCTAR7_DT(x)       (((x)&0x0000000F)<<4)
/* After SCK delay scaler */
#define MCF_DSPI_DCTAR7_ASC(x)      (((x)&0x0000000F)<<8)
/* PCS to SCK delay scaler */
#define MCF_DSPI_DCTAR7_CSSCK(x)    (((x)&0x0000000F)<<12)
/* Baud rate prescaler */
#define MCF_DSPI_DCTAR7_PBR(x)      (((x)&0x00000003)<<16)
/* Delay after transfer prescaler */
#define MCF_DSPI_DCTAR7_PDT(x)      (((x)&0x00000003)<<18)
/* After SCK delay prescaler */
#define MCF_DSPI_DCTAR7_PASC(x)     (((x)&0x00000003)<<20)
/* PCS to SCK delay prescaler */
#define MCF_DSPI_DCTAR7_PCSSCK(x)   (((x)&0x00000003)<<22)
/* LSB first enable */
#define MCF_DSPI_DCTAR7_LSBFE       (0x01000000)
/* Clock phase */
#define MCF_DSPI_DCTAR7_CPHA        (0x02000000)
/* Clock polarity */
#define MCF_DSPI_DCTAR7_CPOL        (0x04000000)
/* Frame size */
#define MCF_DSPI_DCTAR7_FMSZ(x)     (((x)&0x0000000F)<<27)
/* Double baud rate */
#define MCF_DSPI_DCTAR7_DBR         (0x80000000)

/* Bit definitions and macros for DSR */
/* Receive next pointer */
#define MCF_DSPI_DSR_RXPTR(x)   (((x)&0x0000000F))
/* Receive FIFO counter */
#define MCF_DSPI_DSR_RXCTR(x)   (((x)&0x0000000F)<<4)
/* Transmit next pointer */
#define MCF_DSPI_DSR_TXPTR(x)   (((x)&0x0000000F)<<8)
/* Transmit FIFO counter */
#define MCF_DSPI_DSR_TXCTR(x)   (((x)&0x0000000F)<<12)
#define MCF_DSPI_DSR_GET_TXCTR(x) (((x)>>12)&0x0000000F)
/* Receive FIFO drain flag */
#define MCF_DSPI_DSR_RFDF       (0x00020000)
/* Receive FIFO overflow flag */
#define MCF_DSPI_DSR_RFOF       (0x00080000)
/* Transmit FIFO fill flag */
#define MCF_DSPI_DSR_TFFF       (0x02000000)
/* Transmit FIFO underflow flag */
#define MCF_DSPI_DSR_TFUF       (0x08000000)
/* End of queue flag */
#define MCF_DSPI_DSR_EOQF       (0x10000000)
/* Tx and Rx status (enabled | disabled) */
#define MCF_DSPI_DSR_TXRXS      (0x40000000)
/* Transfer complete flag */
#define MCF_DSPI_DSR_TCF        (0x80000000)

/* Bit definitions and macros for DRSER */
/* Receive FIFO drain DMA or interrupt select */
#define MCF_DSPI_DRSER_RFDFS    (0x00010000)
/* Receive FIFO drain request enable */
#define MCF_DSPI_DRSER_RFDFE    (0x00020000)
/* Receive FIFO overflow request enable */
#define MCF_DSPI_DRSER_RFOFE    (0x00080000)
/* Transmit FIFO fill DMA or interrupt select */
#define MCF_DSPI_DRSER_TFFFS    (0x01000000)
/* Transmit FIFO fill request enable */
#define MCF_DSPI_DRSER_TFFFE    (0x02000000)
/* Transmit FIFO underflow request enable */
#define MCF_DSPI_DRSER_TFUFE    (0x08000000)
/* DSPI finished request enable */
#define MCF_DSPI_DRSER_EOQFE    (0x10000000)
/* Transmission complete request enable */
#define MCF_DSPI_DRSER_TCFE     (0x80000000)

/* Bit definitions and macros for DTFR */
/* Transmit data */
#define MCF_DSPI_DTFR_TXDATA(x)     (((x)&0x0000FFFF))
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFR_PCS0          (0x00010000)
/* Peripheral chip select 1 */
#define MCF_DSPI_DTFR_PCS1          (0x00020000)
/* Peripheral chip select 2 */
#define MCF_DSPI_DTFR_PCS2          (0x00040000)
/* Peripheral chip select 3 */
#define MCF_DSPI_DTFR_PCS3          (0x00080000)
/* Peripheral chip select 4 */
#define MCF_DSPI_DTFR_PCS4          (0x00100000)
/* Peripheral chip select 5 */
#define MCF_DSPI_DTFR_PCS5          (0x00200000)
/* Peripheral chip select 6 */
#define MCF_DSPI_DTFR_PCS6          (0x00400000)
/* Peripheral chip select 7 */
#define MCF_DSPI_DTFR_PCS7          (0x00800000)
/* Clear SPI transfer counter */
#define MCF_DSPI_DTFR_CTCNT         (0x04000000)
/* End of queue */
#define MCF_DSPI_DTFR_EOQ           (0x08000000)
/* Clock and transfer attributes select */
#define MCF_DSPI_DTFR_CTAS(x)       (((x)&0x00000007)<<28)
/* Continuous peripheral chip-select enable */
#define MCF_DSPI_DTFR_CONT          (0x80000000)

/* Bit definitions and macros for DRFR */
/* Receive data */
#define MCF_DSPI_DRFR_RXDATA(x)     (((x)&0x0000FFFF))

/* Bit definitions and macros for DTFDR group */
/* Transmit data */
#define MCF_DSPI_DTFDR_TXDATA(x)    (((x)&0x0000FFFF))
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFDR_PCS0         (0x00010000)
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFDR_PCS1         (0x00020000)
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFDR_PCS2         (0x00040000)
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFDR_PCS3         (0x00080000)
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFDR_PCS4         (0x00100000)
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFDR_PCS5         (0x00200000)
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFDR_PCS6         (0x00400000)
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFDR_PCS7         (0x00800000)
/* Clear SPI transfer counter */
#define MCF_DSPI_DTFDR_CTCNT        (0x04000000)
/* End of queue */
#define MCF_DSPI_DTFDR_EOQ          (0x08000000)
/* Clock and transfer attributes select */
#define MCF_DSPI_DTFDR_CTAS(x)      (((x)&0x00000007)<<28)
/* Continuous peripheral chip-select enable */
#define MCF_DSPI_DTFDR_CONT         (0x80000000)

/* Bit definitions and macros for DTFDR0 */
/* Transmit data */
#define MCF_DSPI_DTFDR0_TXDATA(x)   (((x)&0x0000FFFF))
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFDR0_PCS0        (0x00010000)
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFDR0_PCS1        (0x00020000)
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFDR0_PCS2        (0x00040000)
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFDR0_PCS3        (0x00080000)
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFDR0_PCS4        (0x00100000)
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFDR0_PCS5        (0x00200000)
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFDR0_PCS6        (0x00400000)
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFDR0_PCS7        (0x00800000)
/* Clear SPI transfer counter */
#define MCF_DSPI_DTFDR0_CTCNT       (0x04000000)
/* End of queue */
#define MCF_DSPI_DTFDR0_EOQ         (0x08000000)
/* Clock and transfer attributes select */
#define MCF_DSPI_DTFDR0_CTAS(x)     (((x)&0x00000007)<<28)
/* Continuous peripheral chip-select enable */
#define MCF_DSPI_DTFDR0_CONT        (0x80000000)
/* Bit definitions and macros for DTFDR1 */
/* Transmit data */
#define MCF_DSPI_DTFDR1_TXDATA(x)   (((x)&0x0000FFFF))
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFDR1_PCS0        (0x00010000)
/* Peripheral chip select 1 */
#define MCF_DSPI_DTFDR1_PCS1        (0x00020000)
/* Peripheral chip select 2 */
#define MCF_DSPI_DTFDR1_PCS2        (0x00040000)
/* Peripheral chip select 3 */
#define MCF_DSPI_DTFDR1_PCS3        (0x00080000)
/* Peripheral chip select 4 */
#define MCF_DSPI_DTFDR1_PCS4        (0x00100000)
/* Peripheral chip select 5 */
#define MCF_DSPI_DTFDR1_PCS5        (0x00200000)
/* Peripheral chip select 6 */
#define MCF_DSPI_DTFDR1_PCS6        (0x00400000)
/* Peripheral chip select 7 */
#define MCF_DSPI_DTFDR1_PCS7        (0x00800000)
/* Clear SPI transfer counter */
#define MCF_DSPI_DTFDR1_CTCNT       (0x04000000)
/* End of queue */
#define MCF_DSPI_DTFDR1_EOQ         (0x08000000)
/* Clock and transfer attributes select */
#define MCF_DSPI_DTFDR1_CTAS(x)     (((x)&0x00000007)<<28)
/* Continuous peripheral chip-select enable */
#define MCF_DSPI_DTFDR1_CONT        (0x80000000)

/* Bit definitions and macros for DTFDR2 */
#define MCF_DSPI_DTFDR2_TXDATA(x)   (((x)&0x0000FFFF))
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFDR2_PCS0        (0x00010000)
/* Peripheral chip select 1 */
#define MCF_DSPI_DTFDR2_PCS1        (0x00020000)
/* Peripheral chip select 2 */
#define MCF_DSPI_DTFDR2_PCS2        (0x00040000)
/* Peripheral chip select 3 */
#define MCF_DSPI_DTFDR2_PCS3        (0x00080000)
/* Peripheral chip select 4 */
#define MCF_DSPI_DTFDR2_PCS4        (0x00100000)
/* Peripheral chip select 5 */
#define MCF_DSPI_DTFDR2_PCS5        (0x00200000)
/* Peripheral chip select 6 */
#define MCF_DSPI_DTFDR2_PCS6        (0x00400000)
/* Peripheral chip select 7 */
#define MCF_DSPI_DTFDR2_PCS7        (0x00800000)
/* Clear SPI transfer counter */
#define MCF_DSPI_DTFDR2_CTCNT       (0x04000000)
/* End of queue */
#define MCF_DSPI_DTFDR2_EOQ         (0x08000000)
/* Clock and transfer attributes select */
#define MCF_DSPI_DTFDR2_CTAS(x)     (((x)&0x00000007)<<28)
/* Continuous peripheral chip-select enable */
#define MCF_DSPI_DTFDR2_CONT        (0x80000000)

/* Bit definitions and macros for DTFDR3 */
/* Transmit data */
#define MCF_DSPI_DTFDR3_TXDATA(x)   (((x)&0x0000FFFF))
/* Peripheral chip select 0 */
#define MCF_DSPI_DTFDR3_PCS0        (0x00010000)
/* Peripheral chip select 1 */
#define MCF_DSPI_DTFDR3_PCS1        (0x00020000)
/* Peripheral chip select 2 */
#define MCF_DSPI_DTFDR3_PCS2        (0x00040000)
/* Peripheral chip select 3 */
#define MCF_DSPI_DTFDR3_PCS3        (0x00080000)
/* Peripheral chip select 4 */
#define MCF_DSPI_DTFDR3_PCS4        (0x00100000)
/* Peripheral chip select 5 */
#define MCF_DSPI_DTFDR3_PCS5        (0x00200000)
/* Peripheral chip select 6 */
#define MCF_DSPI_DTFDR3_PCS6        (0x00400000)
/* Peripheral chip select 7 */
#define MCF_DSPI_DTFDR3_PCS7        (0x00800000)
/* Clear SPI transfer counter */
#define MCF_DSPI_DTFDR3_CTCNT       (0x04000000)
/* End of queue */
#define MCF_DSPI_DTFDR3_EOQ         (0x08000000)
/* Clock and transfer attributes select */
#define MCF_DSPI_DTFDR3_CTAS(x)     (((x)&0x00000007)<<28)
/* Continuous peripheral chip-select enable */
#define MCF_DSPI_DTFDR3_CONT        (0x80000000)

/* Bit definitions and macros for DRFDR group */
#define MCF_DSPI_DRFDR_RXDATA(x)    (((x)&0x0000FFFF))

/* Bit definitions and macros for DRFDR0 */
#define MCF_DSPI_DRFDR0_RXDATA(x)   (((x)&0x0000FFFF))

/* Bit definitions and macros for DRFDR1 */
#define MCF_DSPI_DRFDR1_RXDATA(x)   (((x)&0x0000FFFF))

/* Bit definitions and macros for DRFDR2 */
#define MCF_DSPI_DRFDR2_RXDATA(x)   (((x)&0x0000FFFF))

/* Bit definitions and macros for DRFDR3 */
#define MCF_DSPI_DRFDR3_RXDATA(x)   (((x)&0x0000FFFF))

/********************************************************************/

#endif /* __MCF5441X_DSPI_H__ */
