/***************************************************************************
 * mcfcau.h - Common header file for Freescale ColdFire
 *            Cryptographic Acceleration Unit (CAU) drivers.
 *
 * Copyright (C) 2007-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 * Author: Andrey Butok
 *         Shrek Wu B16972@freescale.com
 *
 * NOTE: You can find the ColdFire CAU module on MCF5445X and MCF52235.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 ***************************************************************************
 * Changes:
 * v0.01	14 August 2007	Andrey Butok
 */

#ifndef MCFCAU_H
#define MCFCAU_H

#include <linux/spinlock.h>

/* CAU Registers (CAx) */
#define MCFCAU_CASR		(0x0)
#define MCFCAU_CAA		(0x1)
#define MCFCAU_CA0		(0x2)
#define MCFCAU_CA1		(0x3)
#define MCFCAU_CA2		(0x4)
#define MCFCAU_CA3		(0x5)
#define MCFCAU_CA4		(0x6)
#define MCFCAU_CA5		(0x7)

 /* CAU Commands */
#define MCFCAU_CNOP		(0x000)
#define MCFCAU_LDR		(0x010)
#define MCFCAU_STR		(0x020)
#define MCFCAU_ADR		(0x030)
#define MCFCAU_RADR		(0x040)
#define MCFCAU_ADRA		(0x050)
#define MCFCAU_XOR		(0x060)
#define MCFCAU_ROTL		(0x070)
#define MCFCAU_MVRA		(0x080)
#define MCFCAU_MVAR		(0x090)
#define MCFCAU_AESS		(0x0A0)
#define MCFCAU_AESIS		(0x0B0)
#define MCFCAU_AESC		(0x0C0)
#define MCFCAU_AESIC		(0x0D0)
#define MCFCAU_AESR		(0x0E0)
#define MCFCAU_AESIR		(0x0F0)
#define MCFCAU_DESR		(0x100)
#define MCFCAU_DESK		(0x110)
#define MCFCAU_HASH		(0x120)
#define MCFCAU_SHS		(0x130)
#define MCFCAU_MDS		(0x140)
#define MCFCAU_ILL		(0x1F0)

/* DESR Fields */
#define MCFCAU_IP		(0x08)	/* initial permutation */
#define MCFCAU_FP		(0x04)	/* final permutation */
#define MCFCAU_KSL1		(0x00)	/* key schedule left 1 bit */
#define MCFCAU_KSL2		(0x01)	/* key schedule left 2 bits */
#define MCFCAU_KSR1		(0x02)	/* key schedule right 1 bit */
#define MCFCAU_KSR2		(0x03)	/* key schedule right 2 bits */

/* DESK Field */
#define MCFCAU_DC		(0x01)	/* decrypt key schedule */
#define MCFCAU_CP		(0x02)	/* check parity */

/* HASH Functions Codes */
#define MCFCAU_HFF		(0x0)	/* MD5 F() CA1&CA2 | ~CA1&CA3 */
#define MCFCAU_HFG		(0x1)	/* MD5 G() CA1&CA3 | CA2&~CA3 */
#define MCFCAU_HFH		(0x2)	/* MD5 H(), SHA Parity() CA1^CA2^CA3 */
#define MCFCAU_HFI		(0x3)	/* MD5 I() CA2^(CA1|~CA3) */
#define MCFCAU_HFC		(0x4)	/* SHA Ch() CA1&CA2 ^ ~CA1&CA3 */
#define MCFCAU_HFM		(0x5)
/* SHA Maj() CA1&CA2 ^ CA1&CA3 ^ CA2&CA3 */

#define MCFCAU_CRA_PRIORITY	(300)

extern spinlock_t mcfcau_lock;

#ifdef DEBUG
#define DBG(fmt, args...) printk(KERN_INFO "[%s]  " fmt ,\
			__func__, ## args)
#else
#define DBG(fmt, args...) do {} while (0)
#endif

#endif
