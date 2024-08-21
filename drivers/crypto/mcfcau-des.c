/***************************************************************************
 * mcfcau-des.c - Implementation of DES & Triple DES EDE Cipher Algorithms
 *                for Freescale ColdFire Cryptographic Acceleration Unit (CAU).
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
 *		Initial Release - developed on 2.6.20 Linux kernel.
 */

#include <crypto/algapi.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/crypto.h>
#include <linux/types.h>

/*
#undef DEBUG
#define DEBUG 1
*/

#include "mcfcau.h"

#define MCFCAU_DES_KEY_SIZE		(8)
#define MCFCAU_DES_EXPKEY_WORDS		(32)
#define MCFCAU_DES_BLOCK_SIZE		(8)

#define MCFCAU_DES3_EDE_KEY_SIZE	(3 * MCFCAU_DES_KEY_SIZE)
#define MCFCAU_DES3_EDE_EXPKEY_WORDS	(3 * MCFCAU_DES_EXPKEY_WORDS)
#define MCFCAU_DES3_EDE_BLOCK_SIZE	(MCFCAU_DES_BLOCK_SIZE)

#define	MCFCAU_DES_DRIVER_DESC		"DES & 3DES ColdFire CAU driver"
#define	MCFCAU_DES_DRIVER_VERSION	"v0.01"

struct mcfcau_des_ctx {
	u32 expkey[MCFCAU_DES_EXPKEY_WORDS];
};

struct mcfcau_des3_ede_ctx {
	u32 expkey[MCFCAU_DES3_EDE_EXPKEY_WORDS];
};

/* DES round operations */
static inline void mcfcau_des_encipher(void)
{
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESK));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_IP+MCFCAU_KSL1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSL2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSL2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSL2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSL2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSL2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSL2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSL1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSL2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSL2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSL2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSL2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSL2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSL2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSL1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_FP+MCFCAU_KSL1));
}

static inline void mcfcau_des_decipher(void)
{
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESK+MCFCAU_DC));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_IP+MCFCAU_KSR1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSR2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSR2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSR2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSR2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSR2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSR2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSR1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSR2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSR2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSR2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSR2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSR2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSR2));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_KSR1));
	asm("cp0ld.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_DESR+MCFCAU_FP+MCFCAU_KSL1));
}


static int mcfcau_des_setkey(struct crypto_tfm *tfm, const u8 *key_p,
		      unsigned int keylen)
{
	struct mcfcau_des_ctx *dctx = crypto_tfm_ctx(tfm);
	u32 *flags = &tfm->crt_flags;
	u32 * key = (u32 *) key_p;

	DBG("mcfcau_des_setkey\n");

	/*
	* RFC2451: Weak key checks SHOULD be performed.
	*
	* FIPS 74:
	*   Keys having duals are keys which produce all zeros, all ones, or
	*   alternating zero-one patterns in the C and D registers
	*   after Permuted
	*   Choice 1 has operated on the key.
	*
	*/
	if (*flags & CRYPTO_TFM_REQ_FORBID_WEAK_KEYS) { /* FIPS 74 */
		if (key[0] < 0xE001E00l) {
			if (key[0] < 0x1FE01FE0) {
				if (key[0] < 0x01E001E0) {
					if (((key[0] == 0x01010101) &&
						(key[1] == 0x01010101)) ||
						((key[0] == 0x011F011F) &&
						(key[1] == 0x010E010E)))
						goto WEAK_KEY;
				} else {
					if (((key[0] == 0x01E001E0) &&
						(key[1] == 0x01F101F1)) ||
						((key[0] == 0x01FE01FE) &&
						(key[1] == 0x01FE01FE)))
						goto WEAK_KEY;
				}
			} else {
				if (key[0] < 0x1F1F1F1F) {
					if (((key[0] == 0x1FE01FE0) &&
						(key[1] == 0x0EF10EF1)) ||
						((key[0] == 0x1F011F0l) &&
						(key[1] == 0x0E010E01)))
						goto WEAK_KEY;
				} else{
					if (((key[0] == 0x1F1F1F1F) &&
						(key[1] == 0x0E0E0E0E)) ||
						((key[0] == 0x1FFE1FFE) &&
						(key[1] == 0x0EFE0EFE)))
						goto WEAK_KEY;
				}
			}
		} else {
			if (key[0] < 0xFE01FE01) {
				if (key[0] < 0xE0E0E0E0) {
					if (((key[0] == 0xE001E00l) &&
						(key[1] == 0xF101F101)) ||
						((key[0] == 0xE01FE01F) &&
						(key[1] == 0xF10EF10E)))
						goto WEAK_KEY;
				} else {
					if (((key[0] == 0xE0E0E0E0) &&
						(key[1] == 0xF1F1F1F1)) ||
						((key[0] == 0xE0FEE0FE) &&
						(key[1] == 0xF1FEF1FE)))
						goto WEAK_KEY;
				}
			} else {
				if (key[0] < 0xFEE0FEE0) {
					if (((key[0] == 0xFE01FE01) &&
						(key[1] == 0xFE01FE01)) ||
						((key[0] == 0xFE1FFE1F) &&
						(key[1] == 0xFE0EFE0E)))
						goto WEAK_KEY;
				} else {
					if (((key[0] == 0xFEE0FEE0) &&
						(key[1] == 0xFEF1FEF1)) ||
						((key[0] == 0xFEFEFEFE)
						&& (key[1] == 0xFEFEFEFE)))
						goto WEAK_KEY;
				}
			}
		}
	}
	memcpy(dctx->expkey, key_p, keylen);
	return 0;
WEAK_KEY:
	*flags |= CRYPTO_TFM_RES_WEAK_KEY;
	return -EINVAL;
}


void mcfcau_des_encrypt(struct crypto_tfm *tfm, u8 *dst, const u8 *src)
{
	struct mcfcau_des_ctx *ctx = crypto_tfm_ctx(tfm);
	u32 *des_key_tmp = ctx->expkey;
	unsigned long iflags;

	DBG("mcfcau_des_encrypt\n");

	spin_lock_irqsave(&mcfcau_lock, iflags);

	asm("move.l	%0, %%a0" : : "m"(src) : "a0");
	asm("move.l	%0, %%a1" : : "m"(des_key_tmp) : "a1");

	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA2) : "a0");
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA3));
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA0) : "a1");
	asm("cp0ld.l	(%%a1),%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA1));

	mcfcau_des_encipher();

	asm("cp0st.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA2) : "d0");
	asm("cp0st.l	%%d0,%%d1,#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA3) : "d1");
	asm("move.l	%0, %%a1" : : "m"(dst) : "a1");
	asm("move.l	%d0,(%a1)+");
	asm("move.l	%d1,(%a1)");

	spin_unlock_irqrestore(&mcfcau_lock, iflags);
}


void mcfcau_des_decrypt(struct crypto_tfm *tfm, u8 *dst, const u8 *src)
{
	struct mcfcau_des_ctx *ctx = crypto_tfm_ctx(tfm);
	u32 *des_key_tmp = ctx->expkey;
	unsigned long iflags;

	DBG("mcfcau_des_decrypt\n");

	spin_lock_irqsave(&mcfcau_lock, iflags);

	asm("move.l	%0, %%a0" : : "m"(src) : "a0");
	asm("move.l	%0, %%a1" : : "m"(des_key_tmp) : "a1");

	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA2) : "a0");
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA3));
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA0) : "a1");
	asm("cp0ld.l	(%%a1),%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA1));

	mcfcau_des_decipher();

	asm("move.l	%0, %%a1" : : "m"(dst) : "a1");
	asm("cp0st.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA2) : "d0");
	asm("cp0st.l	%%d0,%%d1,#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA3) : "d1");
	asm("move.l	%d0,(%a1)+");
	asm("move.l	%d1,(%a1)");

	spin_unlock_irqrestore(&mcfcau_lock, iflags);
}


/*
 * RFC2451:
 *
 *   For DES-EDE3, there is no known need to reject weak or
 *   complementation keys.  Any weakness is obviated by the use of
 *   multiple keys.
 *
 *   However, if the first two or last two independent 64-bit keys are
 *   equal (k1 == k2 or k2 == k3), then the DES3 operation is simply the
 *   same as DES.  Implementers MUST reject keys that exhibit this
 *   property.
 *
 */

static int mcfcau_des3_ede_setkey(
	struct crypto_tfm *tfm, const u8 *key_p, unsigned int keylen)
{
	const u32 *key = (const u32 *)key_p;
	struct mcfcau_des3_ede_ctx *dctx = crypto_tfm_ctx(tfm);
	u32 *flags = &tfm->crt_flags;

	DBG("mcfcau_des3_ede_setkey\n");

	if (unlikely(!((key[0] ^ key[2]) | (key[1] ^ key[3])) ||
		     !((key[2] ^ key[4]) | (key[3] ^ key[5])))) {
		*flags |= CRYPTO_TFM_RES_BAD_KEY_SCHED;
		return -EINVAL;
	}

	memcpy(dctx->expkey, key_p, keylen);

	return 0;
}

static void mcfcau_des3_ede_encrypt(
	struct crypto_tfm *tfm, u8 *dst, const u8 *src)
{
	struct mcfcau_des3_ede_ctx *dctx = crypto_tfm_ctx(tfm);
	const u32 *des_key_tmp = dctx->expkey;
	unsigned long iflags;

	DBG("mcfcau_des3_ede_encrypt\n");

	spin_lock_irqsave(&mcfcau_lock, iflags);

	/*EK1*/
	asm("move.l	%0, %%a0"
		: : "m"(src) : "a0");
	asm("move.l	%0, %%a1"
		: : "m"(des_key_tmp) : "a1");

	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA2) : "a0");
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA3));
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA0) : "a1");
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA1) : "a1");

	mcfcau_des_encipher();

	/*DK2*/
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA0) : "a1");
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA1) : "a1");

	mcfcau_des_decipher();

	/*EK3*/
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA0) : "a1");
	asm("cp0ld.l	(%%a1),%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA1));

	mcfcau_des_encipher();

	asm("move.l	%0, %%a1"
		: : "m"(dst) : "a1");
	asm("cp0st.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA2) : "d0");
	asm("cp0st.l	%%d0,%%d1,#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA3) : "d1");
	asm("move.l	%d0,(%a1)+");
	asm("move.l	%d1,(%a1)");

	spin_unlock_irqrestore(&mcfcau_lock, iflags);
}

static void mcfcau_des3_ede_decrypt(
	struct crypto_tfm *tfm, u8 *dst, const u8 *src)
{
	struct mcfcau_des3_ede_ctx *dctx = crypto_tfm_ctx(tfm);
	const u32 *des_key_tmp = dctx->expkey + 6 - 2;
	unsigned long iflags;

	DBG("mcfcau_des3_ede_decrypt\n");

	spin_lock_irqsave(&mcfcau_lock, iflags);

	/*DK3*/
	asm("move.l	%0, %%a0"
		: : "m"(src) : "a0");
	asm("move.l	%0, %%a1"
		: : "m"(des_key_tmp) : "a1");

	asm("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA2) : "a0");
	asm("cp0ld.l	(%%a0),%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA3));
	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA0) : "a1");
	asm("cp0ld.l	(%%a1),%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA1));

	mcfcau_des_decipher();

	/*EK2*/
	asm("suba.l	#12,%a1");	/*dec key pointer*/

	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA0) : "a1");
	asm("cp0ld.l	(%%a1),%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA1));

	mcfcau_des_encipher();

	/*DK1*/
	asm("suba.l	#12,%a1");	/*dec key pointer*/

	asm("cp0ld.l	(%%a1)+,%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA0) : "a1");
	asm("cp0ld.l	(%%a1),%%d0,#1,%0"
		: : "n"(MCFCAU_LDR+MCFCAU_CA1));

	mcfcau_des_decipher();

	asm("move.l	%0, %%a1"
		: : "m"(dst) : "a1");
	asm("cp0st.l	%%d0,%%d0,#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA2) : "d0");
	asm("cp0st.l	%%d0,%%d1,#1,%0"
		: : "n"(MCFCAU_STR+MCFCAU_CA3) : "d1");
	asm("move.l	%d0,(%a1)+");
	asm("move.l	%d1,(%a1)");

	spin_unlock_irqrestore(&mcfcau_lock, iflags);
}


static struct crypto_alg mcfcau_des_alg = {
	.cra_name		=	"des",
	.cra_driver_name	=	"des-mcfcau",
	.cra_priority		=	MCFCAU_CRA_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_CIPHER,
	.cra_blocksize		=	MCFCAU_DES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct mcfcau_des_ctx),
	.cra_module		=	THIS_MODULE,
	.cra_alignmask		=	3,
	.cra_list		=	LIST_HEAD_INIT(mcfcau_des_alg.cra_list),
	.cra_u			=	{ .cipher = {
	.cia_min_keysize	=	MCFCAU_DES_KEY_SIZE,
	.cia_max_keysize	=	MCFCAU_DES_KEY_SIZE,
	.cia_setkey		=	mcfcau_des_setkey,
	.cia_encrypt		=	mcfcau_des_encrypt,
	.cia_decrypt		=	mcfcau_des_decrypt } }
};

static struct crypto_alg mcfcau_des3_ede_alg = {
	.cra_name		=	"des3_ede",
	.cra_driver_name	=	"des3_ede-mcfcau",
	.cra_priority		=	MCFCAU_CRA_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_CIPHER,
	.cra_blocksize		=	MCFCAU_DES3_EDE_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct mcfcau_des3_ede_ctx),
	.cra_module		=	THIS_MODULE,
	.cra_alignmask		=	3,
	.cra_list		=
			LIST_HEAD_INIT(mcfcau_des3_ede_alg.cra_list),
	.cra_u			=	{ .cipher = {
	.cia_min_keysize	=	MCFCAU_DES3_EDE_KEY_SIZE,
	.cia_max_keysize	=	MCFCAU_DES3_EDE_KEY_SIZE,
	.cia_setkey		=	mcfcau_des3_ede_setkey,
	.cia_encrypt		=	mcfcau_des3_ede_encrypt,
	.cia_decrypt		=	mcfcau_des3_ede_decrypt } }
};

MODULE_ALIAS("mcfcau_des3_ede");

static int __init mcfcau_des_init(void)
{
	int ret;

	ret = crypto_register_alg(&mcfcau_des_alg);
	if (ret < 0)
		goto out;

	ret = crypto_register_alg(&mcfcau_des3_ede_alg);
	if (ret < 0)
		crypto_unregister_alg(&mcfcau_des_alg);
out:
	printk(KERN_INFO MCFCAU_DES_DRIVER_DESC " "
		MCFCAU_DES_DRIVER_VERSION " %s.\n",
		ret ? "failed" : "registered");
	return ret;
}

static void __exit mcfcau_des_exit(void)
{
	crypto_unregister_alg(&mcfcau_des3_ede_alg);
	crypto_unregister_alg(&mcfcau_des_alg);

	printk(KERN_INFO MCFCAU_DES_DRIVER_DESC " "
		MCFCAU_DES_DRIVER_VERSION " unregistered.\n");
}

module_init(mcfcau_des_init);
module_exit(mcfcau_des_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DES & Triple DES EDE Cipher Algorithms for ColdFire CAU");
MODULE_AUTHOR("Andrey Butok");
