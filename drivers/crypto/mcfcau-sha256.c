// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * mcfcau-sha256.c - SHA-256 Secure Hash Algorithm implementation
 *                   for Freescale ColdFire Cryptographic Acceleration Unit (CAU)
 *
 * Copyright (C) 2007-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2024 Jean-Michel Hautbois <jeanmichel.hautbois@yoseli.org>
 *
 * Based on mcfcau-sha1.c by Andrey Butok and Shrek Wu.
 *
 * The CAU version 2 (found on MCF5441x) adds SHA-256 support with dedicated
 * hardware acceleration for the compression function.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/crypto.h>
#include <linux/types.h>
#include <crypto/sha2.h>
#include <crypto/algapi.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <asm/byteorder.h>

#include "mcfcau.h"

#define MCFCAU_SHA256_DRIVER_DESC	"SHA256 ColdFire CAU driver"
#define MCFCAU_SHA256_DRIVER_VERSION	"v0.01"

/* SHA-256 round constants */
static const u32 K256[64] = {
	0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
	0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
	0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
	0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
	0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
	0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
	0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
	0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
	0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
	0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
	0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
	0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
	0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
	0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
	0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
	0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

struct mcfcau_sha256_ctx {
	u64 count;
	u32 state[8];
	u8 buffer[SHA256_BLOCK_SIZE];
};

/*
 * SHA-256 message schedule expansion (done in software)
 * W[i] = sigma1(W[i-2]) + W[i-7] + sigma0(W[i-15]) + W[i-16]
 */
static inline u32 sigma0(u32 x)
{
	return ror32(x, 7) ^ ror32(x, 18) ^ (x >> 3);
}

static inline u32 sigma1(u32 x)
{
	return ror32(x, 17) ^ ror32(x, 19) ^ (x >> 10);
}

/*
 * SHA-256 compression function using CAU hardware acceleration
 *
 * Register mapping:
 *   CA0=a, CA1=b, CA2=c, CA3=d, CA4=e, CA5=f, CA6=g, CA7=h
 *   CA8=temporary (T1), CAA=accumulator
 *
 * Round function:
 *   T1 = h + Sigma1(e) + Ch(e,f,g) + K[i] + W[i]
 *   T2 = Sigma0(a) + Maj(a,b,c)
 *   h=g, g=f, f=e, e=d+T1, d=c, c=b, b=a, a=T1+T2
 *
 * The SHS2 command performs the register shift:
 *   CA0=CAA, CA1=CA0, CA2=CA1, CA3=CA2,
 *   CA4=CA3+CA8, CA5=CA4, CA6=CA5, CA7=CA6
 */
static noinline void mcfcau_sha256_transform(u32 *state, const u8 *data,
					     u32 *W)
{
	int i;
	u32 *kp;
	u32 *wp;
	unsigned long iflags;

	/* Prepare message schedule W[0..63] */
	for (i = 0; i < 16; i++)
		W[i] = be32_to_cpu(((__be32 *)data)[i]);

	for (i = 16; i < 64; i++)
		W[i] = sigma1(W[i - 2]) + W[i - 7] +
		       sigma0(W[i - 15]) + W[i - 16];

	spin_lock_irqsave(&mcfcau_lock, iflags);

	/* Load initial hash values into CAU registers */
	asm volatile("move.l	%0, %%a0" : : "m"(state) : "a0");
	asm volatile("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		     : : "n"(MCFCAU_LDR + MCFCAU_CA0) : "a0"); /* a */
	asm volatile("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		     : : "n"(MCFCAU_LDR + MCFCAU_CA1) : "a0"); /* b */
	asm volatile("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		     : : "n"(MCFCAU_LDR + MCFCAU_CA2) : "a0"); /* c */
	asm volatile("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		     : : "n"(MCFCAU_LDR + MCFCAU_CA3) : "a0"); /* d */
	asm volatile("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		     : : "n"(MCFCAU_LDR + MCFCAU_CA4) : "a0"); /* e */
	asm volatile("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		     : : "n"(MCFCAU_LDR + MCFCAU_CA5) : "a0"); /* f */
	asm volatile("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		     : : "n"(MCFCAU_LDR + MCFCAU_CA6) : "a0"); /* g */
	asm volatile("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		     : : "n"(MCFCAU_LDR + MCFCAU_CA7) : "a0"); /* h */

	/* Setup pointers for round constants and message schedule */
	kp = (u32 *)K256;
	wp = W;
	asm volatile("move.l	%0, %%a0" : : "m"(kp) : "a0");	/* K pointer */
	asm volatile("move.l	%0, %%a1" : : "m"(wp) : "a1");	/* W pointer */

	/* 64 rounds of SHA-256 compression */
	for (i = 0; i < 64; i++) {
		/*
		 * Compute T1 = h + Sigma1(e) + Ch(e,f,g) + K[i] + W[i]
		 * CAA = h (CA7)
		 */
		asm volatile("cp0ld.l	%%d0,%%d0,#1,%0"
			     : : "n"(MCFCAU_MVRA + MCFCAU_CA7));
		/* CAA += Sigma1(e) using HF2T */
		asm volatile("cp0ld.l	%%d0,%%d0,#1,%0"
			     : : "n"(MCFCAU_HASH + MCFCAU_HF2T));
		/* CAA += Ch(e,f,g) using HF2C */
		asm volatile("cp0ld.l	%%d0,%%d0,#1,%0"
			     : : "n"(MCFCAU_HASH + MCFCAU_HF2C));
		/* CAA += K[i] */
		asm volatile("cp0ld.l	(%%a0)+,%%d0,#1,%0"
			     : : "n"(MCFCAU_ADR + MCFCAU_CAA) : "a0");
		/* CAA += W[i] -> CAA = T1 */
		asm volatile("cp0ld.l	(%%a1)+,%%d0,#1,%0"
			     : : "n"(MCFCAU_ADR + MCFCAU_CAA) : "a1");

		/* Save T1 to CA8 for SHS2 (e = d + T1) */
		asm volatile("cp0ld.l	%%d0,%%d0,#1,%0"
			     : : "n"(MCFCAU_MVAR + MCFCAU_CA8));

		/*
		 * Compute T2 = Sigma0(a) + Maj(a,b,c)
		 * CAA += Sigma0(a) using HF2S (CAA still = T1)
		 */
		asm volatile("cp0ld.l	%%d0,%%d0,#1,%0"
			     : : "n"(MCFCAU_HASH + MCFCAU_HF2S));
		/* CAA += Maj(a,b,c) using HF2M -> CAA = T1 + T2 = new_a */
		asm volatile("cp0ld.l	%%d0,%%d0,#1,%0"
			     : : "n"(MCFCAU_HASH + MCFCAU_HF2M));

		/*
		 * SHS2: Perform the register shift
		 * CA0=CAA(new_a), CA1=CA0, CA2=CA1, CA3=CA2,
		 * CA4=CA3+CA8(d+T1=new_e), CA5=CA4, CA6=CA5, CA7=CA6
		 */
		asm volatile("cp0ld.l	%%d0,%%d0,#1,%0"
			     : : "n"(MCFCAU_SHS2));
	}

	/* Add the compressed chunk to the current hash value */
	asm volatile("move.l	%0, %%a0" : : "m"(state) : "a0");
	asm volatile("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		     : : "n"(MCFCAU_ADR + MCFCAU_CA0) : "a0");
	asm volatile("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		     : : "n"(MCFCAU_ADR + MCFCAU_CA1) : "a0");
	asm volatile("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		     : : "n"(MCFCAU_ADR + MCFCAU_CA2) : "a0");
	asm volatile("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		     : : "n"(MCFCAU_ADR + MCFCAU_CA3) : "a0");
	asm volatile("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		     : : "n"(MCFCAU_ADR + MCFCAU_CA4) : "a0");
	asm volatile("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		     : : "n"(MCFCAU_ADR + MCFCAU_CA5) : "a0");
	asm volatile("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		     : : "n"(MCFCAU_ADR + MCFCAU_CA6) : "a0");
	asm volatile("cp0ld.l	(%%a0)+,%%d0,#1,%0"
		     : : "n"(MCFCAU_ADR + MCFCAU_CA7) : "a0");

	/* Store results back to state */
	asm volatile("cp0st.l	%%d0,-(%%a0),#1,%0"
		     : : "n"(MCFCAU_STR + MCFCAU_CA7) : "a0");
	asm volatile("cp0st.l	%%d0,-(%%a0),#1,%0"
		     : : "n"(MCFCAU_STR + MCFCAU_CA6) : "a0");
	asm volatile("cp0st.l	%%d0,-(%%a0),#1,%0"
		     : : "n"(MCFCAU_STR + MCFCAU_CA5) : "a0");
	asm volatile("cp0st.l	%%d0,-(%%a0),#1,%0"
		     : : "n"(MCFCAU_STR + MCFCAU_CA4) : "a0");
	asm volatile("cp0st.l	%%d0,-(%%a0),#1,%0"
		     : : "n"(MCFCAU_STR + MCFCAU_CA3) : "a0");
	asm volatile("cp0st.l	%%d0,-(%%a0),#1,%0"
		     : : "n"(MCFCAU_STR + MCFCAU_CA2) : "a0");
	asm volatile("cp0st.l	%%d0,-(%%a0),#1,%0"
		     : : "n"(MCFCAU_STR + MCFCAU_CA1) : "a0");
	asm volatile("cp0st.l	%%d0,-(%%a0),#1,%0"
		     : : "n"(MCFCAU_STR + MCFCAU_CA0) : "a0");

	spin_unlock_irqrestore(&mcfcau_lock, iflags);
}

static int mcfcau_sha256_init(struct shash_desc *desc)
{
	struct mcfcau_sha256_ctx *sctx = shash_desc_ctx(desc);

	sctx->state[0] = SHA256_H0;
	sctx->state[1] = SHA256_H1;
	sctx->state[2] = SHA256_H2;
	sctx->state[3] = SHA256_H3;
	sctx->state[4] = SHA256_H4;
	sctx->state[5] = SHA256_H5;
	sctx->state[6] = SHA256_H6;
	sctx->state[7] = SHA256_H7;
	sctx->count = 0;

	return 0;
}

static int mcfcau_sha256_update(struct shash_desc *desc, const u8 *data,
				unsigned int len)
{
	struct mcfcau_sha256_ctx *sctx = shash_desc_ctx(desc);
	unsigned int partial, done;
	const u8 *src;

	partial = sctx->count & (SHA256_BLOCK_SIZE - 1);
	sctx->count += len;
	done = 0;
	src = data;

	if ((partial + len) >= SHA256_BLOCK_SIZE) {
		u32 W[64];

		if (partial) {
			done = -partial;
			memcpy(sctx->buffer + partial, data,
			       done + SHA256_BLOCK_SIZE);
			src = sctx->buffer;
		}

		do {
			mcfcau_sha256_transform(sctx->state, src, W);
			done += SHA256_BLOCK_SIZE;
			src = data + done;
		} while (done + SHA256_BLOCK_SIZE - 1 < len);

		memzero_explicit(W, sizeof(W));
		partial = 0;
	}
	memcpy(sctx->buffer + partial, src, len - done);

	return 0;
}

static int mcfcau_sha256_final(struct shash_desc *desc, u8 *out)
{
	struct mcfcau_sha256_ctx *sctx = shash_desc_ctx(desc);
	__be32 *dst = (__be32 *)out;
	__be64 bits;
	unsigned int index, pad_len;
	int i;
	static const u8 padding[SHA256_BLOCK_SIZE] = { 0x80, };

	/* Save number of bits */
	bits = cpu_to_be64(sctx->count << 3);

	/* Pad out to 56 mod 64 */
	index = sctx->count & (SHA256_BLOCK_SIZE - 1);
	pad_len = (index < 56) ? (56 - index) : (SHA256_BLOCK_SIZE + 56 - index);
	mcfcau_sha256_update(desc, padding, pad_len);

	/* Append length (before padding) */
	mcfcau_sha256_update(desc, (const u8 *)&bits, sizeof(bits));

	/* Store state in digest (big-endian) */
	for (i = 0; i < 8; i++)
		dst[i] = cpu_to_be32(sctx->state[i]);

	/* Wipe context */
	memzero_explicit(sctx, sizeof(*sctx));

	return 0;
}

static struct shash_alg mcfcau_sha256_alg = {
	.init		= mcfcau_sha256_init,
	.update		= mcfcau_sha256_update,
	.final		= mcfcau_sha256_final,
	.digestsize	= SHA256_DIGEST_SIZE,
	.descsize	= sizeof(struct mcfcau_sha256_ctx),
	.statesize	= sizeof(struct mcfcau_sha256_ctx),
	.base		= {
		.cra_name	= "sha256",
		.cra_driver_name = "sha256-mcfcau",
		.cra_priority	= MCFCAU_CRA_PRIORITY,
		.cra_flags	= CRYPTO_ALG_TYPE_SHASH,
		.cra_blocksize	= SHA256_BLOCK_SIZE,
		.cra_module	= THIS_MODULE,
	}
};

static int __init mcfcau_sha256_init_module(void)
{
	int ret = crypto_register_shash(&mcfcau_sha256_alg);

	pr_info(MCFCAU_SHA256_DRIVER_DESC " " MCFCAU_SHA256_DRIVER_VERSION
		" %s.\n", ret ? "failed" : "registered");
	return ret;
}

static void __exit mcfcau_sha256_exit_module(void)
{
	crypto_unregister_shash(&mcfcau_sha256_alg);
	pr_info(MCFCAU_SHA256_DRIVER_DESC " " MCFCAU_SHA256_DRIVER_VERSION
		" unregistered.\n");
}

module_init(mcfcau_sha256_init_module);
module_exit(mcfcau_sha256_exit_module);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(MCFCAU_SHA256_DRIVER_DESC);
MODULE_AUTHOR("Jean-Michel Hautbois <jeanmichel.hautbois@yoseli.org>");
