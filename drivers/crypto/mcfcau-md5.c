/***************************************************************************
 * mcfcau-md5.c - Implementation of MD5 Message Digest Algorithm (RFC1321)
 *                for Freescale ColdFire Cryptographic Acceleration Unit (CAU).
 *
 * Copyright (C) 2007-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 * Author: Andrey Butok
 *         Shrek Wu B16972@freescale.com
 *         Alison Wang b18965@freescale.com
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
 * v0.01	30 September 2007	Andrey Butok
 *		Initial Release - developed on 2.6.20 Linux kernel.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/crypto.h>
#include <linux/types.h>
#include <crypto/algapi.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <asm/byteorder.h>

#include "mcfcau.h"

#define MCFCAU_MD5_DIGEST_SIZE		(16)
#define MCFCAU_MD5_HMAC_BLOCK_SIZE	(64)
#define MCFCAU_MD5_BLOCK_WORDS		(16)
#define MCFCAU_MD5_HASH_WORDS		(4)

#define	MCFCAU_MD5_DRIVER_DESC		"MD5 ColdFire CAU driver"
#define	MCFCAU_MD5_DRIVER_VERSION	"v0.01"


struct mcfcau_md5_ctx {
	u32 hash[MCFCAU_MD5_HASH_WORDS];
	u32 block[MCFCAU_MD5_BLOCK_WORDS];
	u64 byte_count;
};

u32 mcfcau_md5_t[64] = {0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee,
			0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
			0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be,
			0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
			0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa,
			0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
			0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed,
			0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
			0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c,
			0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
			0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05,
			0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
			0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039,
			0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
			0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1,
			0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391};



static void mcfcau_md5_transform(u32 *hash, u32 const *in)
{
	int i;
	u32 *md5_t_p = &mcfcau_md5_t[0];
	unsigned long iflags;

	spin_lock_irqsave(&mcfcau_lock, iflags);
	asm("move.l	%0, %%a1" : : "m"(hash) : "a1");
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CAA) : "a1");/*a*/
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA1) : "a1");/*b*/
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA2) : "a1");/*c*/
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA3) : "a1");/*d*/
	asm("move.l	%0, %%a0" : : "m"(in) : "a0");	/* X[] */
	asm("move.l	%0, %%a3" : : "m"(md5_t_p) : "a3");	/* T[] */

	/*  Round 1 */
	asm("moveq.l	#7, %%d4" : : : "d4");	/* for rotating by 7 */
	asm("moveq.l	#12, %%d5" : : : "d5");	/* for rotating by 12 */
	asm("moveq.l	#17, %%d6" : : : "d6");	/* for rotating by 17 */
	asm("moveq.l	#22, %%d7" : : : "d7");	/* for rotating by 22 */

	for (i = 0; i < 4; i++) {
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_HASH+MCFCAU_HFF));
		/* a+F(b,c,d) */
		asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
			: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
		/* add byterev x[i] */
		asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
			: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
		/* add t[i] */
		asm("cp0ld.l	%%d4,%%d0,#1,%0"
			: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
		/* rotate by 7 */
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
		/* add b */
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_MDS));
		/* register to register shift */

		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_HASH+MCFCAU_HFF));
		asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
			: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
		asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
			: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
		asm("cp0ld.l	%%d5,%%d0,#1,%0"
			: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_MDS));

		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_HASH+MCFCAU_HFF));
		asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
			: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
		asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
			: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
		asm("cp0ld.l	%%d6,%%d0,#1,%0"
			: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_MDS));

		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_HASH+MCFCAU_HFF));
		asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
			: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
		asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
			: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
		asm("cp0ld.l	%%d7,%%d0,#1,%0"
			: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_MDS));
	};


	/* Round 2 */
	asm("moveq.l	#5, %%d4" : : : "d4");	/* for rotating by 5 */
	asm("moveq.l	#9, %%d5" : : : "d5");	/* for rotating by 9 */
	asm("moveq.l	#14, %%d6" : : : "d6");	/* for rotating by 14 */
	asm("moveq.l	#20, %%d7" : : : "d7");	/* for rotating by 20 */

	asm("lea -60(%%a0),%%a0" : : : "a0");

	for (i = 0; i < 2; i++) {
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_HASH+MCFCAU_HFG));
		asm("cp0ld.l	(%%a0),%%d0,#1,%0"
			: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
		asm("lea	20(%%a0),%%a0"		: : : "a0");
		asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
			: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
		asm("cp0ld.l	%%d4,%%d0,#1,%0"
			: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_MDS));

		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_HASH+MCFCAU_HFG));
		asm("cp0ld.l	(%%a0),%%d0,#1,%0"
			: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
		asm("lea	20(%%a0),%%a0"	: : : "a0");
		asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
			: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
		asm("cp0ld.l	%%d5,%%d0,#1,%0"
			: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"	: : "n"(MCFCAU_MDS));

		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_HASH+MCFCAU_HFG));
		asm("cp0ld.l	(%%a0),%%d0,#1,%0"
			: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
		asm("lea	-44(%%a0),%%a0"		: : : "a0");
		asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
			: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
		asm("cp0ld.l	%%d6,%%d0,#1,%0"
			: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_MDS));

		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_HASH+MCFCAU_HFG));
		asm("cp0ld.l	(%%a0),%%d0,#1,%0"
			: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
		asm("lea	20(%%a0),%%a0"	: : : "a0");
		asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
			: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
		asm("cp0ld.l	%%d7,%%d0,#1,%0"
			: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_MDS));
	};

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFG));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	20(%%a0),%%a0"
		: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d4,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFG));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	-44(%%a0),%%a0"
		: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d5,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFG));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	20(%%a0),%%a0"
		: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d6,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFG));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	20(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d7,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));


	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFG));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	-44(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d4,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFG));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	20(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d5,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFG));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	20(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d6,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFG));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	-28(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d7,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));


	/* Round 3 */
	asm("moveq.l	#4, %%d4" : : : "d4");	/* for rotating by 5 */
	asm("moveq.l	#11, %%d5" : : : "d5");	/* for rotating by 9 */
	asm("moveq.l	#16, %%d6" : : : "d6");	/* for rotating by 14 */
	asm("moveq.l	#23, %%d7" : : : "d7");	/* for rotating by 20 */


	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFH));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	12(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d4,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFH));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	12(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d5,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFH));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	12(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d6,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0" : : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFH));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	-52(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d7,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));


	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFH));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	12(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d4,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFH));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	12(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d5,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFH));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	12(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d6,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFH));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	12(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d7,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));


	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFH));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	-52(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d4,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFH));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	12(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d5,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFH));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	12(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d6,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFH));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	12(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d7,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));


	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFH));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	12(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d4,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFH));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	12(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d5,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFH));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	-52(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d6,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFH));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	-8(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d7,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	/* Round 4 */
	asm("moveq.l	#6, %%d4" : : : "d4");	/* for rotating by 6 */
	asm("moveq.l	#10, %%d5" : : : "d5");	/* for rotating by 10 */
	asm("moveq.l	#15, %%d6" : : : "d6");	/* for rotating by 15 */
	asm("moveq.l	#21, %%d7" : : : "d7");	/* for rotating by 21 */


	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFI));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	28(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d4,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFI));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	28(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d5,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFI));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	-36(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d6,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFI));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	28(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d7,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));


	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFI));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	-36(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d4,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFI));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	28(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d5,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFI));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	-36(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d6,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFI));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	28(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d7,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));


	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFI));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	28(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d4,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFI));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	-36(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d5,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFI));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	28(%%a0),%%a0"	: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d6,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFI));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	-36(%%a0),%%a0" : : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d7,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));


	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFI));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	28(%%a0),%%a0"
		: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d4,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFI));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	-36(%%a0),%%a0"
		: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d5,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFI));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	28(%%a0),%%a0"
		: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d6,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));

	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_HASH+MCFCAU_HFI));
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_RADR+MCFCAU_CAA) : "a0");
	asm("lea	28(%%a0),%%a0"		: : : "a0");
	asm("cp0ld.l	(%%a3)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a3");
	asm("cp0ld.l	%%d7,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ADRA+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MDS));


	asm("move.l	%0, %%a1" : : "m"(hash) : "a1");

	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a1");/*a*/
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CA1) : "a1");/*b*/
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CA2) : "a1");/*c*/
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CA3) : "a1");/*d*/

	asm("cp0st.l	%%d0,-(%%a1),#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA3) : "a1");/*d*/
	asm("cp0st.l	%%d0,-(%%a1),#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA2) : "a1");/*c*/
	asm("cp0st.l	%%d0,-(%%a1),#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA1) : "a1");/*b*/
	asm("cp0st.l	%%d0,-(%%a1),#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CAA) : "a1");/*a*/
	spin_unlock_irqrestore(&mcfcau_lock, iflags);
}

static int mcfcau_md5_initialization(struct shash_desc *desc)
{
	struct mcfcau_md5_ctx *mctx = shash_desc_ctx(desc);

	DBG("mcfcau_md5_initialization\n");
	mctx->hash[0] = 0x67452301;
	mctx->hash[1] = 0xefcdab89;
	mctx->hash[2] = 0x98badcfe;
	mctx->hash[3] = 0x10325476;
	mctx->byte_count = 0;

	return 0;
}

static int mcfcau_md5_update(struct shash_desc *desc,
		const u8 *data, unsigned int len)
{
	struct mcfcau_md5_ctx *mctx = shash_desc_ctx(desc);
	const u32 avail = sizeof(mctx->block) - (mctx->byte_count & 0x3f);

	DBG("mcfcau_md5_update\n");
	mctx->byte_count += len;

	if (avail > len) {
		memcpy((char *)mctx->block + (sizeof(mctx->block) - avail),
		       data, len);
	} else {
		memcpy((char *)mctx->block +
			(sizeof(mctx->block) - avail), data, avail);

		mcfcau_md5_transform(mctx->hash, mctx->block);
		data += avail;
		len -= avail;

		while (len >= sizeof(mctx->block)) {
			memcpy(mctx->block, data, sizeof(mctx->block));
			mcfcau_md5_transform(mctx->hash, mctx->block);
			data += sizeof(mctx->block);
			len -= sizeof(mctx->block);
		}

		memcpy(mctx->block, data, len);
	}

	return 0;
}

static int mcfcau_md5_final(struct shash_desc *desc, u8 *out)
{
	struct mcfcau_md5_ctx *mctx = shash_desc_ctx(desc);
	const unsigned int offset = mctx->byte_count & 0x3f;
	char *p = (char *)mctx->block + offset;
	int padding = 56 - (offset + 1);

	DBG("mcfcau_md5_final\n");

	*p++ = 0x80;
	if (padding < 0) {
		memset(p, 0x00, padding + sizeof(u64));
		mcfcau_md5_transform(mctx->hash, mctx->block);
		p = (char *)mctx->block;
		padding = 56;
	}

	memset(p, 0, padding);
	mctx->block[14] = mctx->byte_count << 3;
	mctx->block[15] = mctx->byte_count >> 29;
	le32_to_cpu_array(&mctx->block[14], 2);

	mcfcau_md5_transform(mctx->hash, mctx->block);

	cpu_to_le32_array(mctx->hash, sizeof(mctx->hash) / sizeof(u32));
	memcpy(out, mctx->hash, sizeof(mctx->hash));
	memset(mctx, 0, sizeof(*mctx));

	return 0;
}

static struct shash_alg mcfcau_md5_alg = {
	.init = mcfcau_md5_initialization,
	.update = mcfcau_md5_update,
	.final = mcfcau_md5_final,
	.digestsize = MCFCAU_MD5_DIGEST_SIZE,
	.descsize = sizeof(struct mcfcau_md5_ctx),
	.statesize = sizeof(struct mcfcau_md5_ctx),
	.base = {
		.cra_name = "md5",
		.cra_driver_name = "md5-mcfcau",
		.cra_blocksize = MCFCAU_MD5_HMAC_BLOCK_SIZE,
		.cra_flags = CRYPTO_ALG_TYPE_SHASH,
		.cra_priority =	MCFCAU_CRA_PRIORITY,
		.cra_module = THIS_MODULE
	}
};

static int __init mcfcau_md5_init(void)
{
	int ret = 0;

	ret = crypto_register_shash(&mcfcau_md5_alg);
	printk(KERN_INFO MCFCAU_MD5_DRIVER_DESC " "
		MCFCAU_MD5_DRIVER_VERSION " %s.\n",
		ret ? "failed" : "registered");
	return ret;
}

static void __exit mcfcau_md5_exit(void)
{
	crypto_unregister_shash(&mcfcau_md5_alg);
	printk(KERN_INFO MCFCAU_MD5_DRIVER_DESC " "
		MCFCAU_MD5_DRIVER_VERSION " unregistered.\n");
}

module_init(mcfcau_md5_init);
module_exit(mcfcau_md5_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(MCFCAU_MD5_DRIVER_DESC);
MODULE_AUTHOR("Andrey Butok");
