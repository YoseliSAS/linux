/*
 * Copyright (C) 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 * Author: Andrey Butok
 *
 * This file is based on mcfqspi.h
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
 *   v0.001	25 March 2008		Andrey Butok
 *		Initial Release - developed on uClinux with 2.6.23 kernel.
 *
 */

#ifndef MCFDSPI_H_
#define MCFDSPI_H_

struct coldfire_dspi_chip {
	u8 mode;
	u8 bits_per_word;
	u16 void_write_data;
	/* Only used in master mode */
	u8 dbr;		/* Double baud rate */
	u8 pbr;		/* Baud rate prescaler */
	u8 br;		/* Baud rate scaler */
	u8 pcssck;	/* PCS to SCK delay prescaler */
	u8 pasc;	/* After SCK delay prescaler */
	u8 pdt;		/* Delay after transfer prescaler */
	u8 cssck;	/* PCS to SCK delay scaler */
	u8 asc;		/* After SCK delay scaler */
	u8 dt;		/* Delay after transfer scaler */
};

struct coldfire_spi_master {
	u16 bus_num;
	u16 num_chipselect;
	u8  irq_source;
	u32 irq_vector;
	u32 irq_mask;
	u8  irq_lp;
	u8  par_val;
	u16 par_val16;
	u32 *irq_list;
	void (*cs_control)(u8 cs, u8 command);
};

struct coldfire_spi_slave {
	u16 bus_num;
	u16 num_chipselect;
	u8  irq_source;
	u32 irq_vector;
	u32 irq_mask;
	u8  irq_lp;
};

struct coldfire_spi_chip {
	u8 mode;
	u8 bits_per_word;
	u8 del_cs_to_clk;
	u8 del_after_trans;
	u16 void_write_data;
};
#endif /*MCFDSPI_H_*/
