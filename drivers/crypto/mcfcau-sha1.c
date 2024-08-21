/***************************************************************************
 * mcfcau-sha1.c - Implementation of SHA1 Secure Hash Algorithm
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
 * v0.01	15 October 2007	Andrey Butok
 *		Initial Release - developed on 2.6.20 Linux kernel.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/crypto.h>
#include <linux/types.h>
#include <crypto/algapi.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>

#include "mcfcau.h"

#define MCFCAU_SHA1_DIGEST_WORDS	(5)
#define MCFCAU_SHA1_WORKSPACE_WORDS	(80)

#define MCFCAU_SHA1_DIGEST_SIZE		(20)
#define MCFCAU_SHA1_HMAC_BLOCK_SIZE	(64)

#define	MCFCAU_SHA1_DRIVER_DESC		"SHA1 ColdFire CAU driver"
#define	MCFCAU_SHA1_DRIVER_VERSION	"v0.01"

static const u32 K[4] = {0x5A827999L,	/* Rounds  0-19: sqrt(2) * 2^30 */
			0x6ED9EBA1L,	/* Rounds 20-39: sqrt(3) * 2^30 */
			0x8F1BBCDCL,	/* Rounds 40-59: sqrt(5) * 2^30 */
			0xCA62C1D6L};	/* Rounds 60-79: sqrt(10) * 2^30 */

struct mcfcau_sha1_ctx {
	u64 count;
	u32 state[5];
	u8 buffer[64];
};

static noinline void mcfcau_sha1_transform(__u32 *digest,
		const char *in, __u32 *W)
{
	int i;
	u32 *tmp_p;
	unsigned long iflags;

	/* (a) Devide M(i) into 16 words W */
	for (i = 0; i < 16; i++)
		W[i] = ((const u32 *)in)[i];

	/* (b) W[i+16] = S^1(W[i+13] ^ W[i+8] ^ W[i+2] ^ W[i]) */
	tmp_p = &W[16];

	spin_lock_irqsave(&mcfcau_lock, iflags);
	asm("move.l	%0, %%a0" : : "m"(tmp_p) : "a0");
	asm("moveq.l	#1, %%d3" : : : "d3");

	for (i = 0; i < 64; i++) {
		asm("cp0ld.l	-64(%%a0),%%d0,#1,%0"
			: : "n"(MCFCAU_LDR+MCFCAU_CA0));
		asm("cp0ld.l	-56(%%a0),%%d0,#1,%0"
			: : "n"(MCFCAU_XOR+MCFCAU_CA0));
		asm("cp0ld.l	-32(%%a0),%%d0,#1,%0"
			: : "n"(MCFCAU_XOR+MCFCAU_CA0));
		asm("cp0ld.l	-12(%%a0),%%d0,#1,%0"
			: : "n"(MCFCAU_XOR+MCFCAU_CA0));
		asm("cp0ld.l	%%d3,%%d0,#1,%0"
			: : "n"(MCFCAU_ROTL+MCFCAU_CA0) : "d3");
		asm("cp0st.l	%%d0,(%%a0)+,#1,%0"
			: : "n"(MCFCAU_STR+MCFCAU_CA0));
	}

	/* (c) */
	asm("move.l	%0, %%a0" : : "m"(digest) : "a0");
	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA0) : "a0"); /* a */
	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA1) : "a0"); /* b */
	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA2) : "a0"); /* c */
	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA3) : "a0"); /* d */
	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA4) : "a0"); /* e */

	/* (d) */
	asm("moveq.l	#5, %%d0" : : : "d0");
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_MVRA+MCFCAU_CA0));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_ROTL+MCFCAU_CAA)); /*S^5(A)*/

	tmp_p = (u32 *)K;
	asm("move.l	%0, %%a0" : : "m"(tmp_p) : "a0");
	asm("move.l	%0, %%a1" : : "m"(W) : "a1");

	for (i = 0; i < 20; i++) {
		/* t = f1(b, c, d) + K1 + rol32(a, 5) + e + W[i]; */
		/* e = d; d = c; c = rol32(b, 30); b = a; a = t; */
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_HASH+MCFCAU_HFC)); /*f(b,c,d)*/
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_ADRA+MCFCAU_CA4)); /*+e*/
		asm("cp0ld.l	(%%a0),%%d0,#1,%0"
			: : "n"(MCFCAU_ADR+MCFCAU_CAA)); /*+K*/
		asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
			: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a1"); /*+W*/
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_SHS));
	}

	asm("add.l #4,%%a0"	: : : "a0"); /* update K */

	for (; i < 40; i++) {
		/* t = f2(b, c, d) + K2 + rol32(a, 5) + e + W[i]; */
		/* e = d; d = c; c = rol32(b, 30); b = a; a = t; */
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_HASH+MCFCAU_HFH)); /*f(b,c,d)*/
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_ADRA+MCFCAU_CA4)); /*+e*/
		asm("cp0ld.l	(%%a0),%%d0,#1,%0"
			: : "n"(MCFCAU_ADR+MCFCAU_CAA)); /*+K*/
		asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
			: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a1"); /*+W*/
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_SHS));
	}

	asm("add.l #4,%%a0"	 : : : "a0"); /* update K */

	for (; i < 60; i++) {
		/* t = f3(b, c, d) + K3 + rol32(a, 5) + e + W[i]; */
		/* e = d; d = c; c = rol32(b, 30); b = a; a = t; */
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_HASH+MCFCAU_HFM)); /*f(b,c,d)*/
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_ADRA+MCFCAU_CA4)); /*+e*/
		asm("cp0ld.l	(%%a0),%%d0,#1,%0"
			: : "n"(MCFCAU_ADR+MCFCAU_CAA)); /*+K*/
		asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
			: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a1"); /*+W*/
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_SHS));
	}

	asm("add.l #4,%%a0"	: : : "a0"); /* update K */

	for (; i < 80; i++) {
		/* t = f2(b, c, d) + K4 + rol32(a, 5) + e + W[i]; */
		/* e = d; d = c; c = rol32(b, 30); b = a; a = t; */
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_HASH+MCFCAU_HFH)); /*f(b,c,d)*/
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_ADRA+MCFCAU_CA4)); /*+e*/
		asm("cp0ld.l	(%%a0),%%d0,#1,%0"
			: : "n"(MCFCAU_ADR+MCFCAU_CAA)); /*+K*/
		asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
			: : "n"(MCFCAU_ADR+MCFCAU_CAA) : "a1"); /*+W*/
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_SHS));
	}

	/* (e) */
	asm("move.l	%0, %%a0" : : "m"(digest) : "a0");
	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CA0) : "a0"); /* +a */
	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CA1) : "a0"); /* +b */
	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CA2) : "a0"); /* +c */
	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CA3) : "a0"); /* +d */
	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_ADR+MCFCAU_CA4) : "a0"); /* +e */

	asm("cp0st.l	%%d0,-(%%a0),#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA4) : "a0");
	asm("cp0st.l	%%d0,-(%%a0),#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA3) : "a0");
	asm("cp0st.l	%%d0,-(%%a0),#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA2) : "a0");
	asm("cp0st.l	%%d0,-(%%a0),#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA1) : "a0");
	asm("cp0st.l	%%d0,-(%%a0),#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA0) : "a0");
	spin_unlock_irqrestore(&mcfcau_lock, iflags);
}

static int mcfcau_sha1_init(struct shash_desc *desc)
{
	struct mcfcau_sha1_ctx *sctx = shash_desc_ctx(desc);
	static const struct mcfcau_sha1_ctx initstate = {
	  0,
	  { 0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0 },
	  { 0, }
	};

	DBG("mcfcau_sha1_init\n");
	*sctx = initstate;

	return 0;
}

static int mcfcau_sha1_update(struct shash_desc *desc, const u8 *data,
				unsigned int len)
{
	struct mcfcau_sha1_ctx *sctx = shash_desc_ctx(desc);
	unsigned int partial, done;
	const u8 *src;

	DBG("mcfcau_sha1_update\n");
	partial = sctx->count & 0x3f;
	sctx->count += len;
	done = 0;
	src = data;

	if ((partial + len) > 63) {
		u32 temp[MCFCAU_SHA1_WORKSPACE_WORDS];

		if (partial) {
			done = -partial;
			memcpy(sctx->buffer + partial, data, done + 64);
			src = sctx->buffer;
		}

		do {
			mcfcau_sha1_transform(sctx->state, src, temp);
			done += 64;
			src = data + done;
		} while (done + 63 < len);

		memset(temp, 0, sizeof(temp));
		partial = 0;
	}
	memcpy(sctx->buffer + partial, src, len - done);
	return 0;
}


/* Add padding and return the message digest. */
static int mcfcau_sha1_final(struct shash_desc *desc, u8 *out)
{
	struct mcfcau_sha1_ctx *sctx = shash_desc_ctx(desc);
	u32 *dst = (u32 *)out;
	u32 i, index, padlen;
	u64 bits;
	static const u8 padding[64] = { 0x80, };

	DBG("mcfcau_sha1_final\n");
	bits = sctx->count << 3;

	/* Pad out to 56 mod 64 */
	index = sctx->count & 0x3f;
	padlen = (index < 56) ? (56 - index) : ((64+56) - index);
	mcfcau_sha1_update(desc, padding, padlen);

	/* Append length */
	mcfcau_sha1_update(desc, (const u8 *)&bits, sizeof(bits));

	/* Store state in digest */
	for (i = 0; i < 5; i++)
		dst[i] = sctx->state[i];

	/* Wipe context */
	memset(sctx, 0, sizeof *sctx);

	return 0;
}

static struct shash_alg mcfcau_sha1_alg = {
	.init		=	mcfcau_sha1_init,
	.update		=	mcfcau_sha1_update,
	.final		=	mcfcau_sha1_final,
	.digestsize	=	MCFCAU_SHA1_DIGEST_SIZE,
	.descsize	=	sizeof(struct mcfcau_sha1_ctx),
	.statesize	=	sizeof(struct mcfcau_sha1_ctx),
	.base	=	{
		.cra_name	 =	"sha1",
		.cra_driver_name =	"sha1-mcfcau",
		.cra_priority	 =	MCFCAU_CRA_PRIORITY,
		.cra_flags	 =	CRYPTO_ALG_TYPE_SHASH,
		.cra_blocksize	 =	MCFCAU_SHA1_HMAC_BLOCK_SIZE,
		.cra_module	 =	THIS_MODULE,
	}
};

static int __init init(void)
{
	int ret = crypto_register_shash(&mcfcau_sha1_alg);
	printk(KERN_INFO MCFCAU_SHA1_DRIVER_DESC " "
		MCFCAU_SHA1_DRIVER_VERSION " %s.\n",
		ret ? "failed" : "registered");
	return ret;
}

static void __exit fini(void)
{
	crypto_unregister_shash(&mcfcau_sha1_alg);
	printk(KERN_INFO MCFCAU_SHA1_DRIVER_DESC " "
		MCFCAU_SHA1_DRIVER_VERSION " unregistered.\n");
}

module_init(init);
module_exit(fini);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(MCFCAU_SHA1_DRIVER_DESC);
MODULE_AUTHOR("Andrey Butok");
