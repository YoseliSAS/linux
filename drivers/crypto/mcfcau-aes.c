/***************************************************************************
 * mcfcau-aes.c - Implementation of AES Cipher Algorithm
 *                for Freescale ColdFire Cryptographic Acceleration Unit (CAU).
 *
 * Copyright (C) 2007-2011 Freescale Semiconductor Inc. All Rights Reserved.
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
 * v0.01	17 September 2007	Andrey Butok
 *		Initial Release - developed on 2.6.20 Linux kernel.
 */

#include <crypto/algapi.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/crypto.h>

/*
#undef DEBUG
#define DEBUG 1
*/

#include "mcfcau.h"

#define MCFCAU_AES_MIN_KEY_SIZE	(16)
#define MCFCAU_AES_MAX_KEY_SIZE	(32)
#define MCFCAU_AES_BLOCK_SIZE	(16)

#define	MCFCAU_AES_DRIVER_DESC		"AES ColdFire CAU driver"
#define	MCFCAU_AES_DRIVER_VERSION	"v0.01"

struct mcfcau_aes_ctx {
	int Nr_1;
	u32 buf[120];
	u32 buf_tmp[16];
};

static u32 mcfcau_rco_tab[10]
	= { 0x01000000, 0x02000000, 0x04000000, 0x08000000,
	0x10000000, 0x20000000, 0x40000000, 0x80000000,
	0x1b000000, 0x36000000};

int mcfcau_aes_setkey(struct crypto_tfm *tfm, const u8 *in_key,
		       unsigned int key_len)
{
	struct mcfcau_aes_ctx *ctx = crypto_tfm_ctx(tfm);
	const u32 *key = (const u32 *)in_key;
	u32 *flags = &tfm->crt_flags;
	u32 i;
	u32 *key_sch = (&ctx->buf[0]);
	u32 *temp_p, *rcon_p;
	u32 Nx;
	u32 Nk;
	unsigned long iflags;

	DBG("mcfcau_aes_setkey\n");

	if (key_len % 8) {
		*flags |= CRYPTO_TFM_RES_BAD_KEY_LEN;
		return -EINVAL;
	}

	Nk = key_len>>2;

	for (i = 0; i < Nk; i++)
		key_sch[i] = key[i];

	ctx->Nr_1 = Nk+5;

	/* Key Expansion */
	temp_p = &key_sch[Nk-1];
	rcon_p = &mcfcau_rco_tab[0];

	spin_lock_irqsave(&mcfcau_lock, iflags);

	asm volatile ("move.l	%0, %%a1"
		: : "m"(temp_p) : "a1");
	asm volatile ("move.l	%0, %%a3"
		: : "m"(rcon_p) : "a3");
	asm volatile ("move.l	%0, %%a4"
		: : "m"(key_sch) : "a4");

	Nx = (Nk+7)<<2; /* (Nr+1)*Nb */

	for (i = Nk; i < Nx; i++) {
		/* temp = key_sch[Nk-1] */
		asm volatile ("cp0ld.l	(%%a1)+,%%d0,#1,%0"
			: : "n"(MCFCAU_LDR+MCFCAU_CAA) : "a1");

		if (i % Nk == 0) {
			asm volatile ("moveq	#8, %%d0" : : : "d0");
			/* CAA=RotWord(temp) */
			asm volatile ("cp0ld.l	%%d0,%%d0,#1,%0"
				: : "n"(MCFCAU_ROTL+MCFCAU_CAA) : "d0");
			/* SubWord(CAA) */
			asm volatile ("cp0ld.l	%%d0,%%d0,#1,%0"
				: : "n"(MCFCAU_AESS+MCFCAU_CAA));
			/* ACC xor rcon[i/Nk] */
			asm volatile ("cp0ld.l	(%%a3)+,%%d0,#1,%0"
				: : "n"(MCFCAU_XOR+MCFCAU_CAA) : "a3");

		} else if ((Nk > 6) && (i % Nk == 4)) {
			/* SubWord(ACC) */
			asm volatile ("cp0ld.l	%%d0,%%d0,#1,%0"
				: : "n"(MCFCAU_AESS+MCFCAU_CAA));
		}

		/* key_sch[i]^=key_sch[i-Nk]; store ACC to key_sch[i] */
		asm volatile ("cp0ld.l	(%%a4)+,%%d0,#1,%0"
			: : "n"(MCFCAU_XOR+MCFCAU_CAA) : "a4");
		asm volatile ("cp0st.l	%%d0,(%%a1),#1,%0"
			: : "n"(MCFCAU_STR+MCFCAU_CAA));
	}
	spin_unlock_irqrestore(&mcfcau_lock, iflags);

	return 0;
}


/* encrypt a block of text */
static void mcfcau_aes_encrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
	struct mcfcau_aes_ctx *ctx = crypto_tfm_ctx(tfm);
	const int Nr_1 = ctx->Nr_1;

	u32 *key_sch = &(ctx->buf[0]);
	u32 i;
	unsigned long iflags;

	DBG("mcfcau_aes_encrypt\n");

	spin_lock_irqsave(&mcfcau_lock, iflags);
	asm("move.l	%0, %%a1" : : "m"(in) : "a1");
	asm("move.l	%0, %%a0" : : "m"(key_sch) : "a0");
	/* state=in */
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA0) : "a1");
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA1) : "a1");
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA2) : "a1");
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA3) : "a1");
	/* AddRoundKey() */
	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_XOR+MCFCAU_CA0) : "a0");
	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_XOR+MCFCAU_CA1) : "a0");
	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_XOR+MCFCAU_CA2) : "a0");
	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_XOR+MCFCAU_CA3) : "a0");

	for (i = Nr_1; i > 0; i--) {
		/* SubBytes(state) */
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_AESS+MCFCAU_CA0));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_AESS+MCFCAU_CA1));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_AESS+MCFCAU_CA2));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_AESS+MCFCAU_CA3));
		/* ShiftRows(state) */
		asm("cp0ld.l	%%d0,%%d0,#1,%0" : : "n"(MCFCAU_AESR));
		/* MixColumns(state); AddRoundKey() */
		asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
			: : "n"(MCFCAU_AESC+MCFCAU_CA0) : "a0");
		asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
			: : "n"(MCFCAU_AESC+MCFCAU_CA1) : "a0");
		asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
			: : "n"(MCFCAU_AESC+MCFCAU_CA2) : "a0");
		asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
			: : "n"(MCFCAU_AESC+MCFCAU_CA3) : "a0");
	}
	/* SubBytes(state)*/
	asm("cp0ld.l	%%d0,%%d0,#1,%0" : : "n"(MCFCAU_AESS+MCFCAU_CA0));
	asm("cp0ld.l	%%d0,%%d0,#1,%0" : : "n"(MCFCAU_AESS+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0" : : "n"(MCFCAU_AESS+MCFCAU_CA2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0" : : "n"(MCFCAU_AESS+MCFCAU_CA3));
	/* ShiftRows(state) */
	asm("cp0ld.l	%%d0,%%d0,#1,%0" : : "n"(MCFCAU_AESR));
	/* AddRoundKey() */
	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_XOR+MCFCAU_CA0) : "a0");
	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_XOR+MCFCAU_CA1) : "a0");
	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_XOR+MCFCAU_CA2) : "a0");
	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_XOR+MCFCAU_CA3) : "a0");
	/* out = state */
	asm("move.l	%0, %%a1" : : "m"(out) : "a1");
	asm("cp0st.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA0) : "d0");
	asm("cp0st.l	%%d0,%%d1,#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA1) : "d1");

	asm("move.l	%%d0,(%%a1)+" : : : "a1");
	asm("move.l	%%d1,(%%a1)+" : : : "a1");

	asm("cp0st.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA2) : "d0");
	asm("cp0st.l	%%d0,%%d1,#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA3) : "d1");

	asm("move.l	%%d0,(%%a1)+" : : : "a1");
	asm("move.l	%%d1,(%%a1)+" : : : "a1");
	spin_unlock_irqrestore(&mcfcau_lock, iflags);
}


/* decrypt a block of text */
static void mcfcau_aes_decrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
	struct mcfcau_aes_ctx *ctx = crypto_tfm_ctx(tfm);
	u32 *key_sch = &(ctx->buf[0]);
	u32 i;
	unsigned long iflags;
	const int Nr_1 = ctx->Nr_1;
	key_sch = &key_sch[(Nr_1+2)*4];

	DBG("mcfcau_aes_decrypt\n");

	spin_lock_irqsave(&mcfcau_lock, iflags);

	asm("move.l	%0, %%a1" : : "m"(in) : "a1");
	asm("move.l	%0, %%a0" : : "m"(key_sch) : "a0");
	/* state=in */
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA0) : "a1");
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA1) : "a1");
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA2) : "a1");
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA3) : "a1");
	/* AddRoundKey() */
	asm("cp0ld.l	-(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_XOR+MCFCAU_CA3) : "a0");
	asm("cp0ld.l	-(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_XOR+MCFCAU_CA2) : "a0");
	asm("cp0ld.l	-(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_XOR+MCFCAU_CA1) : "a0");
	asm("cp0ld.l	-(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_XOR+MCFCAU_CA0) : "a0");

	for (i = Nr_1; i > 0; i--) {
		/* InvShiftRows(state) */
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_AESIR));
		/* InvSubBytes(state) */
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_AESIS+MCFCAU_CA3));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_AESIS+MCFCAU_CA2));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_AESIS+MCFCAU_CA1));
		asm("cp0ld.l	%%d0,%%d0,#1,%0"
			: : "n"(MCFCAU_AESIS+MCFCAU_CA0));
		/* InvMixColumns(state); AddRoundKey() */
		asm("cp0ld.l	-(%%a0),%%d0,#1,%0"
			: : "n"(MCFCAU_AESIC+MCFCAU_CA3) : "a0");
		asm("cp0ld.l	-(%%a0),%%d0,#1,%0"
			: : "n"(MCFCAU_AESIC+MCFCAU_CA2) : "a0");
		asm("cp0ld.l	-(%%a0),%%d0,#1,%0"
			: : "n"(MCFCAU_AESIC+MCFCAU_CA1) : "a0");
		asm("cp0ld.l	-(%%a0),%%d0,#1,%0"
			: : "n"(MCFCAU_AESIC+MCFCAU_CA0) : "a0");
	}
	/* InvShiftRows(state) */
	asm("cp0ld.l	%%d0,%%d0,#1,%0" : : "n"(MCFCAU_AESIR));
	/* InvSubBytes(state)*/
	asm("cp0ld.l	%%d0,%%d0,#1,%0" : : "n"(MCFCAU_AESIS+MCFCAU_CA3));
	asm("cp0ld.l	%%d0,%%d0,#1,%0" : : "n"(MCFCAU_AESIS+MCFCAU_CA2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0" : : "n"(MCFCAU_AESIS+MCFCAU_CA1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0" : : "n"(MCFCAU_AESIS+MCFCAU_CA0));
	/* AddRoundKey() */
	asm("cp0ld.l	-(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_XOR+MCFCAU_CA3) : "a0");
	asm("cp0ld.l	-(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_XOR+MCFCAU_CA2) : "a0");
	asm("cp0ld.l	-(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_XOR+MCFCAU_CA1) : "a0");
	asm("cp0ld.l	-(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_XOR+MCFCAU_CA0) : "a0");
	/* out = state */
	asm("move.l	%0, %%a1"	 : : "m"(out) : "a1");
	asm("cp0st.l	%%d0,%%d0,#1,%0" : : "n"(MCFCAU_STR+MCFCAU_CA0) : "d0");
	asm("cp0st.l	%%d0,%%d1,#1,%0" : : "n"(MCFCAU_STR+MCFCAU_CA1) : "d1");

	asm("move.l	%%d0,(%%a1)+" : : : "a1");
	asm("move.l	%%d1,(%%a1)+" : : : "a1");

	asm("cp0st.l	%%d0,%%d0,#1,%0" : : "n"(MCFCAU_STR+MCFCAU_CA2) : "d0");
	asm("cp0st.l	%%d0,%%d1,#1,%0" : : "n"(MCFCAU_STR+MCFCAU_CA3) : "d1");

	asm("move.l	%%d0,(%%a1)+" : : : "a1");
	asm("move.l	%%d1,(%%a1)+" : : : "a1");
	spin_unlock_irqrestore(&mcfcau_lock, iflags);

}


static struct crypto_alg mcfcau_aes_alg = {
	.cra_name		=	"aes",
	.cra_driver_name	=	"aes-mcfcau",
	.cra_priority		=	MCFCAU_CRA_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_CIPHER,
	.cra_blocksize		=	MCFCAU_AES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct mcfcau_aes_ctx),
	.cra_alignmask		=	3,
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(mcfcau_aes_alg.cra_list),
	.cra_u			=	{
		.cipher = {
			.cia_min_keysize	=	MCFCAU_AES_MIN_KEY_SIZE,
			.cia_max_keysize	=	MCFCAU_AES_MAX_KEY_SIZE,
			.cia_setkey		=	mcfcau_aes_setkey,
			.cia_encrypt		=	mcfcau_aes_encrypt,
			.cia_decrypt		=	mcfcau_aes_decrypt
		}
	}
};

static int __init mcfcau_aes_init(void)
{
	int ret = crypto_register_alg(&mcfcau_aes_alg);

	printk(KERN_INFO MCFCAU_AES_DRIVER_DESC " "
		MCFCAU_AES_DRIVER_VERSION " %s.\n",
		ret ? "failed" : "registered");
	return ret;
}

static void __exit mcfcau_aes_fini(void)
{
	crypto_unregister_alg(&mcfcau_aes_alg);
	printk(KERN_INFO MCFCAU_AES_DRIVER_DESC " "
		MCFCAU_AES_DRIVER_VERSION " unregistered.\n");
}

module_init(mcfcau_aes_init);
module_exit(mcfcau_aes_fini);

MODULE_DESCRIPTION(MCFCAU_AES_DRIVER_DESC);
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Andrey Butok");
