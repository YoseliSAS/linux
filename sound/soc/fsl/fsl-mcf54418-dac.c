// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Freescale MCF54418 DAC Driver
 *
 * Copyright (C) 2025 Wabtec Corporation
 *
 * This driver provides ALSA/ASoC support for the MCF54418 dual 12-bit DACs
 * with DMA support and timer-synchronized updates.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_data/dma-mcf-edma.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <asm/m5441xsim.h>

/* Removed dma_mode parameter - using single period mode only for reliability */

/*
 * Board type selection - determines GPIO pin assignments
 *
 * The MCF54418 is used on multiple boards with different GPIO mappings:
 * - DLCNext:  Audio on PB0/PC7 (GPIO 8/23), requires muxing from CAN1
 * - DLCMI20:  Audio on PG2/PH4 (GPIO 50/60), no pin conflict
 *
 * Set via module parameter: board_type=dlcnext or board_type=dlcmi20
 * Note: On DLCNext, loading this module disables CAN1 (shared pins).
 *       To use CAN1, don't load this module.
 */
static char *board_type = "dlcnext";
module_param(board_type, charp, 0444);
MODULE_PARM_DESC(board_type, "Board type: dlcnext (default) or dlcmi20");

/* DAC Register offsets */
#define DAC_CR			0x00	/* Control Register */
#define DAC_DATA		0x02	/* Data Register */
#define DAC_SR			0x0A	/* Status Register */
#define DAC_FILTCNT		0x0C	/* Filter Counter */

/* ITERATION 70: DAC Control Register bits - CORRECTED from old driver!
 * Old driver struct reg_control bit order (LSB to MSB):
 * [0]pdn, [1]format, [2]sync_en, [3]auto, [4]down, [5]up, [6]hsls, [7]dmaen, [8-9]wmlvl, ...
 */
#define DAC_CR_PDN		BIT(0)	/* Power Down */
#define DAC_CR_FORMAT		BIT(1)	/* Data Format: 0=right-justified, 1=left-justified */
#define DAC_CR_SYNC_EN		BIT(2)	/* Timer Sync Enable - FIXED was BIT(14), should be BIT(2) */
#define DAC_CR_AUTO		BIT(3)	/* Auto mode */
#define DAC_CR_DOWN		BIT(4)	/* Down */
#define DAC_CR_UP		BIT(5)	/* Up */
#define DAC_CR_HSLS		BIT(6)	/* High speed / Low speed */
#define DAC_CR_DMAEN		BIT(7)	/* DMA Enable - FIXED was BIT(15), should be BIT(7) */
#define DAC_CR_WMLVL_SHIFT	8
#define DAC_CR_WMLVL_MASK	(0x03 << DAC_CR_WMLVL_SHIFT)	/* Watermark Level [9:8] */
#define DAC_CR_FILT_EN		BIT(12)	/* Filter Enable */
#define DAC_CR_WMLVL_0		(0x00 << DAC_CR_WMLVL_SHIFT)
#define DAC_CR_WMLVL_2		(0x01 << DAC_CR_WMLVL_SHIFT)
#define DAC_CR_WMLVL_4		(0x02 << DAC_CR_WMLVL_SHIFT)
#define DAC_CR_WMLVL_6		(0x03 << DAC_CR_WMLVL_SHIFT)

/* DAC Status Register bits */
#define DAC_SR_FULL		BIT(1)	/* FIFO Full */
#define DAC_SR_EMPTY		BIT(0)	/* FIFO Empty */

/* Timer Register offsets (DTIM) */
#define DTIM_DTMR		0x00	/* Timer Mode Register */
#define DTIM_DTXMR		0x02	/* Extended Mode Register */
#define DTIM_DTER		0x03	/* Event Register */
#define DTIM_DTRR		0x04	/* Reference Register */
#define DTIM_DTCR		0x08	/* Capture Register */
#define DTIM_DTCN		0x0C	/* Counter Register */

/* Timer Mode Register bits */
#define DTIM_DTMR_RST		BIT(0)	/* Reset */
/* ITERATION 69 FIX: DIV1 was WRONG! Should be 0x01, not 0x00
 * CLK[2:1] bits: 00=STOP, 01=DIV1, 10=DIV16
 */
#define DTIM_DTMR_CLK_STOP	(0x00 << 1)
#define DTIM_DTMR_CLK_DIV1	(0x01 << 1)	/* FIXED: was 0x00, should be 0x01 */
#define DTIM_DTMR_CLK_DIV16	(0x02 << 1)	/* FIXED: was 0x01, should be 0x02 */
#define DTIM_DTMR_FRR		BIT(3)	/* Free Run/Restart */
#define DTIM_DTMR_ORRI		BIT(4)	/* Output Reference Request/Interrupt Enable */
#define DTIM_DTMR_OM		BIT(5)	/* Output Mode */
#define DTIM_DTMR_CE_NONE	(0x00 << 6)	/* Capture Disabled */
#define DTIM_DTMR_CE_RISING	(0x01 << 6)
#define DTIM_DTMR_CE_FALLING	(0x02 << 6)
#define DTIM_DTMR_CE_ANY	(0x03 << 6)
#define DTIM_DTMR_PS_MASK	0xFF00	/* Prescaler */
#define DTIM_DTMR_PS_SHIFT	8
#define DTIM_DTXMR_DMAEN	BIT(7)	/* DMA request enable (vs interrupt) */

/* Timer Event Register bits (DTER) - WRITE-1-TO-CLEAR status bits!
 * Per manual page 39-6: "Writing a 1 to DTERn[REF] or DTERn[CAP] clears it"
 * These are STATUS bits set by hardware, NOT configuration bits!
 */
#define DTIM_DTER_REF		BIT(0)	/* Reference Event status (write 1 to clear!) */
#define DTIM_DTER_CAP		BIT(1)	/* Capture Event status (write 1 to clear!) */

/* ITERATION 71: CCM DAC Timer Sync Register
 * CRITICAL FIX: Old driver uses 0xEC09001E absolute address
 * CCM base = 0xEC090000, so offset = 0x1E (NOT 0x28!)
 */
#define CCM_DACTSR		0x1E
#define CCM_DACTSR_DAC0_DTIM3	(0x03 << 0)	/* DAC0 sync to DTIM3 */
#define CCM_DACTSR_DAC1_DTIM3	(0x03 << 4)	/* DAC1 sync to DTIM3 */

/* ITERATION 71: CCM Miscellaneous Control Register 2
 * Old driver: 0xEC09001A = offset 0x1A (NOT 0x34!)
 */
#define CCM_MISCCR2		0x1A
#define CCM_MISCCR2_DAC0SEL	BIT(5)	/* Enable DAC0 analog output - bit 5 per old driver */
#define CCM_MISCCR2_DAC1SEL	BIT(6)	/* Enable DAC1 analog output - bit 6 per old driver */

#define MCF_EDMA_CHAN_DAC0	62
#define MCF_EDMA_CHAN_DAC1	63

/* Volume range: 0-8, same as original driver (REQ 05100) */
#define DAC_VOLUME_MAX		8

/*
 * GPIO pins for audio control - board-dependent
 *
 * DLCNext:  PB0 (GPIO 8) = MUTE, PC7 (GPIO 23) = SHUTDOWN
 *           These pins are shared with CAN1, requires PAR_CANI2C muxing
 * DLCMI20:  PG2 (GPIO 50) = MUTE, PH4 (GPIO 60) = SHUTDOWN
 *           CAN1 uses separate pins, no muxing conflict
 */
#define MCF_GPIO_AUDIO_MUTE_DLCNEXT	8	/* PB0 */
#define MCF_GPIO_AUDIO_SHUTDOWN_DLCNEXT	23	/* PC7 */
#define MCF_GPIO_AUDIO_MUTE_DLCMI20	50	/* PG2 */
#define MCF_GPIO_AUDIO_SHUTDOWN_DLCMI20	60	/* PH4 */

/* PAR_CANI2C muxing - only needed on DLCNext where audio shares CAN1 pins */
#define MCF_GPIO_PAR_CANI2C_CAN1TX_MASK		(0xF3)	/* Clear bits[3:2] */
#define MCF_GPIO_PAR_CANI2C_CAN1TX_GPIO		(0x00)	/* GPIO mode */
#define MCF_GPIO_PAR_CANI2C_CAN1RX_MASK		(0xFC)	/* Clear bits[1:0] */
#define MCF_GPIO_PAR_CANI2C_CAN1RX_GPIO		(0x00)	/* GPIO mode */

struct mcf54418_dac {
	struct device *dev;
	void __iomem *dac0_base;
	void __iomem *dac1_base;
	void __iomem *dtim_base;
	void __iomem *ccm_base;
	struct clk *bus_clk;
	unsigned int bus_clk_rate;

	/* DMA - managed by dmaengine PCM via slave map */
	struct snd_dmaengine_dai_dma_data dma_params_tx;

	/* Audio parameters */
	unsigned int sample_rate;
	unsigned int channels;
	snd_pcm_format_t format;
	unsigned int volume_dac0;	/* DAC0 volume 0-8 (REQ 05100) */
	unsigned int volume_dac1;	/* DAC1 volume 0-8 (REQ 05100) */
	bool dual_mono;			/* Duplicate mono to both DACs (REQ 05050) */

	/* GPIO control */
	int gpio_mute;
	int gpio_shutdown;
	bool mute_inverted;		/* If true, LOW = unmuted */
	bool shutdown_inverted;		/* If true, LOW = powered on */

	/* State */
	bool enabled;

	/* Debug */
	struct dentry *debugfs_root;
	struct snd_pcm_substream *active_substream;
};


struct mcf54418_dac_pcm_runtime {
	struct dma_chan *dma_chan_left;		/* DMA channel for DAC0 (left) */
	struct dma_chan *dma_chan_right;	/* DMA channel for DAC1 (right) - stereo only */
	struct dma_slave_config slave_config_left;	/* DMA config for left */
	struct dma_slave_config slave_config_right;	/* DMA config for right */

	/* Period tracking */
	unsigned int current_period;		/* Period that just completed (for ALSA) */
	unsigned int next_source_period;	/* Next ALSA period to read */
	unsigned int total_periods;		/* Total number of periods */
	size_t period_bytes;			/* Size of each period in bytes */
	dma_addr_t dma_addr;			/* DMA buffer physical address */
	unsigned int channels;			/* 1=mono, 2=stereo */

	/* DMA descriptor tracking */
	struct dma_async_tx_descriptor *desc_left;
	struct dma_async_tx_descriptor *desc_right;
	dma_cookie_t cookie_left;
	dma_cookie_t cookie_right;

	/* State */
	bool running;				/* DMA is running */
	spinlock_t lock;			/* Protect state */
	atomic_t dma_complete_count;		/* Track DMA completions for stereo */

	/* Cyclic process buffers for volume application.
	 * One contiguous buffer per DAC of 2 * process_buf_size.
	 * DMA loops continuously: INT_HALF after first half,
	 * INT_MAJOR after second half. Software refills the
	 * half that just finished playing.
	 */
	void *process_buf_dac0;			/* Cyclic buffer for DAC0 */
	void *process_buf_dac1;			/* Cyclic buffer for DAC1 */
	dma_addr_t process_dma_dac0;		/* DMA address for DAC0 */
	dma_addr_t process_dma_dac1;		/* DMA address for DAC1 */
	size_t process_buf_size;		/* Size of ONE half (one period) */
	unsigned int process_buf_idx;		/* Which half to refill (0 or 1) */
};

static const struct snd_pcm_hardware mcf54418_dac_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_BLOCK_TRANSFER,
	/* Support S16_LE (wav native), S16_BE (CPU native), and U16_BE (DAC native).
	 * The driver converts all formats to unsigned 12-bit in process_period().
	 */
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S16_BE |
				  SNDRV_PCM_FMTBIT_U16_BE,
	.rates			= SNDRV_PCM_RATE_8000_48000,
	.rate_min		= 8000,
	.rate_max		= 48000,
	.channels_min		= 1,
	.channels_max		= 2,
	.buffer_bytes_max	= 64 * 1024,
	.period_bytes_min	= 512,
	.period_bytes_max	= 8192,
	.periods_min		= 2,
	.periods_max		= 128,
	.fifo_size		= 8,	/* 8-entry FIFO per DAC */
};

static inline void dac_writel(struct mcf54418_dac *dac, int channel,
			      unsigned int reg, u32 val)
{
	void __iomem *base = channel ? dac->dac1_base : dac->dac0_base;

	writew(val, base + reg);
}

static inline u32 dac_readl(struct mcf54418_dac *dac, int channel,
			    unsigned int reg)
{
	void __iomem *base = channel ? dac->dac1_base : dac->dac0_base;

	return readw(base + reg);
}

/*
 * Apply volume to a single sample (0-8 scale, like original driver).
 * Sample is in U16_BE offset-binary format where 0x800 = silence.
 */
static inline u16 mcf54418_apply_volume(u16 sample, unsigned int volume)
{
	int centered;

	if (volume >= DAC_VOLUME_MAX)
		return sample;
	if (volume == 0)
		return 0x800;  /* Silence = midpoint for offset-binary */

	/* Center around 0x800, apply attenuation, recenter */
	centered = (int)sample - 0x800;

	if (volume == 7)
		centered = (centered >> 1) + (centered >> 2);  /* 75% */
	else
		centered = centered >> (DAC_VOLUME_MAX - volume - 1);

	return (u16)clamp(centered + 0x800, 0, 0xFFF);
}

/*
 * Convert a raw 16-bit sample to 12-bit unsigned offset-binary for the DAC.
 * DAC expects 12-bit values (0x000-0xFFF) where 0x800 = silence.
 *
 * S16_LE: byte-swap to native, then signed→unsigned, then >>4 for 12-bit.
 * S16_BE: signed→unsigned, then >>4 for 12-bit.
 * U16_BE: >>4 for 12-bit (already unsigned).
 */
static inline u16 mcf54418_to_dac12(u16 raw, snd_pcm_format_t format)
{
	u16 unsigned16;

	switch (format) {
	case SNDRV_PCM_FORMAT_S16_LE:
		unsigned16 = (u16)((s16)swab16(raw) + 0x8000);
		break;
	case SNDRV_PCM_FORMAT_S16_BE:
		unsigned16 = (u16)((s16)raw + 0x8000);
		break;
	default: /* U16_BE */
		unsigned16 = raw;
		break;
	}
	return unsigned16 >> 4;
}

/*
 * Process a period of samples: apply per-channel volume and handle dual-mono.
 * Source buffer is from ALSA (S16_BE or U16_BE format).
 * Destination buffers are DMA-capable for submission to DAC0/DAC1.
 */
static void mcf54418_dac_process_period(struct mcf54418_dac *dac,
					struct mcf54418_dac_pcm_runtime *prtd,
					const void *src_buf,
					size_t period_bytes)
{
	const u16 *src = src_buf;
	snd_pcm_format_t format = dac->format;
	size_t offset = prtd->process_buf_idx * prtd->process_buf_size;
	u16 *dst0 = prtd->process_buf_dac0 + offset;
	u16 *dst1 = prtd->process_buf_dac1 + offset;
	size_t num_samples;
	unsigned int i;

	if (prtd->channels == 2 && !dac->dual_mono) {
		/* Stereo: L->DAC0, R->DAC1 with per-channel volume */
		num_samples = period_bytes / 4;  /* 2 bytes * 2 channels */
		for (i = 0; i < num_samples; i++) {
			u16 l = mcf54418_to_dac12(src[i * 2], format);
			u16 r = mcf54418_to_dac12(src[i * 2 + 1], format);

			dst0[i] = mcf54418_apply_volume(l, dac->volume_dac0);
			dst1[i] = mcf54418_apply_volume(r, dac->volume_dac1);
		}
	} else if (dac->dual_mono) {
		/* Dual-mono: same source to both DACs with different volumes */
		if (prtd->channels == 2) {
			/* Stereo input but dual-mono: use only left channel */
			num_samples = period_bytes / 4;
			for (i = 0; i < num_samples; i++) {
				u16 s = mcf54418_to_dac12(src[i * 2], format);

				dst0[i] = mcf54418_apply_volume(s, dac->volume_dac0);
				dst1[i] = mcf54418_apply_volume(s, dac->volume_dac1);
			}
		} else {
			/* Mono input: duplicate to both DACs */
			num_samples = period_bytes / 2;
			for (i = 0; i < num_samples; i++) {
				u16 s = mcf54418_to_dac12(src[i], format);

				dst0[i] = mcf54418_apply_volume(s, dac->volume_dac0);
				dst1[i] = mcf54418_apply_volume(s, dac->volume_dac1);
			}
		}
	} else {
		/* Mono without dual-mono: DAC0 only */
		num_samples = period_bytes / 2;
		for (i = 0; i < num_samples; i++) {
			u16 s = mcf54418_to_dac12(src[i], format);

			dst0[i] = mcf54418_apply_volume(s, dac->volume_dac0);
		}
	}
}

static void mcf54418_dac_setup_ccm(struct mcf54418_dac *dac);

static void mcf54418_dac_enable_analog_outputs(struct mcf54418_dac *dac)
{
	u16 misccr2;

	/* Enable DAC analog outputs via CCM_MISCCR2 register */
	misccr2 = readw(dac->ccm_base + CCM_MISCCR2);
	misccr2 |= CCM_MISCCR2_DAC0SEL | CCM_MISCCR2_DAC1SEL;
	writew(misccr2, dac->ccm_base + CCM_MISCCR2);

	dev_info(dac->dev, "Enabled DAC analog outputs (MISCCR2=0x%04x)\n", misccr2);
}

static void mcf54418_dac_enable(struct mcf54418_dac *dac, bool enable)
{
	int i;
	u16 cr;

	/* ITERATION 84: Check eDMA TCD62 registers to see if DREQ bit is set!
	 * showed DTRR preserved correctly but IRQ 192 count=0 (no DMA requests).
	 * Suspect: dmaengine framework may be setting DREQ=1 in TCD.CSR which would
	 * disable DMA requests after the first major loop completes.
	 *
	 * eDMA TCD layout (32 bytes per channel, channel 62 starts at offset 62*32 = 0x7C0):
	 * Base: 0xFC045000 (from MCF5441X reference manual)
	 * TCD62 base: 0xFC045000 + 0x7C0 = 0xFC0457C0
	 * TCD offsets:
	 *   +0x00: SADDR (32-bit) - Source address
	 *   +0x04: SOFF (16-bit) - Signed source address offset
	 *   +0x06: ATTR (16-bit) - Transfer attributes
	 *   +0x08: NBYTES (32-bit) - Minor byte count
	 *   +0x0C: SLAST (32-bit) - Last source address adjustment
	 *   +0x10: DADDR (32-bit) - Destination address
	 *   +0x14: DOFF (16-bit) - Signed destination address offset
	 *   +0x16: CITER (16-bit) - Current major iteration count
	 *   +0x18: DLAST_SGA (32-bit) - Last destination address adjustment
	 *   +0x1C: CSR (16-bit) - Control and Status
	 *   +0x1E: BITER (16-bit) - Beginning major iteration count
	 *
	 * CSR bits:
	 *   bit 3: DREQ - Disable Request: 1=disable DMA requests after major loop
	 *   bit 2: INTHALF - Enable interrupt when major count is half complete
	 *   bit 1: INTMAJOR - Enable interrupt when major loop completes
	 *   bit 0: START - Explicitly start channel (not used with hardware requests)
	 */
	/* CCM routing is configured once during probe via mcf54418_dac_setup_ccm() */

	/* Enable both DAC0 and DAC1 for stereo support */
	for (i = 0; i < 2; i++) {
		cr = dac_readl(dac, i, DAC_CR);
		if (enable) {
			cr &= ~(DAC_CR_PDN | DAC_CR_AUTO | DAC_CR_WMLVL_MASK);
			cr |= DAC_CR_DMAEN | DAC_CR_SYNC_EN | (0x02 << DAC_CR_WMLVL_SHIFT);  /* WMLVL=2 */
			/* DON'T prime FIFO - let DMA fill it via WMLVL=2 */
		} else {
			cr |= DAC_CR_PDN;
			cr &= ~(DAC_CR_DMAEN | DAC_CR_SYNC_EN);
		}
		dac_writel(dac, i, DAC_CR, cr);
	}

	if (enable) {
		u16 dtmr;

		/* Step 1: Configure CCM routing (DTIM3 → DAC0) */
		mcf54418_dac_setup_ccm(dac);
		/* Step 2: START timer (NOW CCM is ready to route SYNC pulses!) */
		dtmr = DTIM_DTMR_CLK_DIV1 | DTIM_DTMR_FRR | DTIM_DTMR_RST;  /* 0x000B */
		writew(dtmr, dac->dtim_base + DTIM_DTMR);

		/* Clear REF and CAP flags */
		writeb(0x03, dac->dtim_base + DTIM_DTER);
	}

	dac->enabled = enable;
}

static int mcf54418_dac_setup_timer(struct mcf54418_dac *dac, unsigned int rate)
{
	u32 divider;

	/* ITERATION 67: MATCH OLD DRIVER EXACTLY!
	 * Old driver uses DIV1 (no prescaler), OM=1 for output pulses (NO ORRI bit!).
	 * Timer output (TnOUT via OM) goes to CCM → DAC, then DAC generates DMA request.
	 * Sequence: DTIM3_OUT (OM) → CCM routing → DAC SYNC_IN → DAC (DMAEN) → eDMA channel 62
	 * NOT: DTIM3_ORRI → eDMA directly (that was the mistake!)
	 */
	divider = dac->bus_clk_rate / rate;  /* DIV1, no /16 prescaler */
	if (divider > 0xFFFFFFFF)
		divider = 0xFFFFFFFF;
	if (divider < 1)
		divider = 1;

	/* Stop timer */
	writew(0x0000, dac->dtim_base + DTIM_DTMR);

	/* Set reference count */
	writel(divider, dac->dtim_base + DTIM_DTRR);

	/* CCM routing is configured once during probe via mcf54418_dac_setup_ccm() */
	return 0;
}

static void mcf54418_dac_setup_ccm(struct mcf54418_dac *dac)
{
	u16 dactsr, readback;

	/* Configure DAC0 and DAC1 to sync with DTIM3
	 * Per MCF54418RM-CCM.pdf page 10-14, DACTSR register bits:
	 *   DAC0CH  [4:3] = 0x03 (channel 3 = DTIM3)
	 *   DAC0SRC [2:0] = 0x05 (TnOUT = timer output)
	 *   DAC1CH  [12:11] = 0x03 (channel 3 = DTIM3)
	 *   DAC1SRC [10:8] = 0x05 (TnOUT = timer output)
	 */
	dactsr = readw(dac->ccm_base + CCM_DACTSR);
	dactsr &= ~0x1F1F;  /* Clear DAC0 bits [4:0] and DAC1 bits [12:8] */
	dactsr |= (0x03 << 3) | (0x05 << 0);   /* DAC0: channel=3 (DTIM3), src=5 (TnOUT) */
	dactsr |= (0x03 << 11) | (0x05 << 8);  /* DAC1: channel=3 (DTIM3), src=5 (TnOUT) */
	writew(dactsr, dac->ccm_base + CCM_DACTSR);

	/* Read back to verify */
	readback = readw(dac->ccm_base + CCM_DACTSR);
	dev_info(dac->dev, "CCM routing: DACTSR=0x%04x (DAC0: ch=%d, src=%d | DAC1: ch=%d, src=%d)\n",
		 readback,
		 (readback >> 3) & 0x03, readback & 0x07,
		 (readback >> 11) & 0x03, (readback >> 8) & 0x07);
}

/* DMA filter function for non-DT platforms using mcf-edma
 * ITERATION 56 FIX 2: Match by channel ID directly, not by driver.
 * The dmaengine framework iterates through ALL DMA channels in the system,
 * including channels from other DMA controllers. The mcf_edma_filter_fn was
 * rejecting all channels because it checked driver pointer equality first.
 * Instead, we match by channel ID alone.
 */
static bool mcf54418_dac_dma_filter(struct dma_chan *chan, void *param)
{
	unsigned long requested_chan = (unsigned long)param;

	return chan->chan_id == requested_chan;
}

/* Custom compat_request_channel that doesn't depend on dma_data being set */
static struct dma_chan *mcf54418_dac_compat_request_channel(
	struct snd_soc_pcm_runtime *rtd,
	struct snd_pcm_substream *substream)
{
	struct dma_chan *chan;
	struct dma_slave_config slave_config;
	int ret;
	/* For playback (stream 0), request channel 62 */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		chan = snd_dmaengine_pcm_request_channel(mcf54418_dac_dma_filter,
							  (void *)(uintptr_t)62);
		if (!chan) {
			dev_err(rtd->dev, "failed to get DMA channel for playback\n");
			return NULL;
		}
		/* ITERATION 65 UPDATED: Configure DMA slave parameters for MCF54418 DAC
		 * CRITICAL: Each DTIM3 match triggers ONE 16-bit sample transfer.
		 * The old working driver uses nbytes=2, which means:
		 * - Each hardware trigger (DTIM3 match at 11025 Hz) transfers 2 bytes (one 16-bit sample)
		 * - The DMA runs continuously, retriggered by DTIM3 for each sample
		 *
		 * Settings:
		 * - addr_width = 2 bytes (16-bit per sample)
		 * - maxburst = 1 (ONE sample per hardware trigger)
		 * - nbytes = width * burst = 2 * 1 = 2 bytes per minor loop
		 * - direction = MEM_TO_DEV
		 * - dst_addr = 0xFC098002 (DAC VDACR register high 16 bits)
		 */
		memset(&slave_config, 0, sizeof(slave_config));
		slave_config.direction = DMA_MEM_TO_DEV;
		slave_config.dst_addr = 0xFC098002;  /* DAC VDACR high 16 bits */
		slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;  /* 16-bit transfers */
		slave_config.dst_maxburst = 1;  /* ONE sample per trigger, nbytes = 2 * 1 = 2 */

		ret = dmaengine_slave_config(chan, &slave_config);
		if (ret < 0) {
			dev_err(rtd->dev, "failed to configure DMA slave: %d\n", ret);
			dma_release_channel(chan);
			return NULL;
		}
		return chan;
	}

	/* No capture support */
	return NULL;
}

/* ITERATION 64: Custom prepare_slave_config for MCF54418 DAC */
static int mcf54418_dac_prepare_slave_config(struct snd_pcm_substream *substream,
					      struct snd_pcm_hw_params *params,
					      struct dma_slave_config *slave_config)
{
	int ret;

	/* Call the default helper to set up basic parameters */
	ret = snd_dmaengine_pcm_prepare_slave_config(substream, params, slave_config);
	if (ret < 0)
		return ret;

	/* ITERATION 65: Override with MCF54418 DAC-specific settings
	 * The DAC requires 16-bit transfers to 0xFC098002 (VDACR high 16 bits).
	 *
	 * CRITICAL: The old working driver uses nbytes=2 (one 16-bit sample per trigger).
	 * Each DTIM3 match at 11025 Hz triggers transfer of ONE 16-bit sample.
	 * Setting nbytes=2048  was WRONG - it tried to transfer 1024 samples
	 * on a single hardware trigger, which the DAC FIFO cannot handle.
	 *
	 * Correct settings:
	 * - width: 2 bytes (16-bit per sample)
	 * - burst: 1 (ONE sample per hardware trigger)
	 * - nbytes = width * burst = 2 * 1 = 2 bytes
	 *
	 * The DMA will be retriggered by DTIM3 at 11025 Hz for each successive sample.
	 */
	slave_config->dst_addr = 0xFC098002;
	slave_config->dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	slave_config->dst_maxburst = 1;
	return 0;
}

/* Custom PCM component - manual period-by-period DMA management
 * This replaces the default dmaengine PCM to avoid prep_dma_cyclic()
 * which doesn't work on MCF54418 due to READ-ONLY BITER register.
 */

static int mcf54418_dac_pcm_open(struct snd_soc_component *component,
				 struct snd_pcm_substream *substream)
{
	struct mcf54418_dac_pcm_runtime *prtd;
	struct dma_chan *chan;
	int ret;
	/* Only support playback */
	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -EINVAL;

	/* Allocate runtime data */
	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (!prtd)
		return -ENOMEM;

	spin_lock_init(&prtd->lock);
	substream->runtime->private_data = prtd;

	/* Request DMA channel 62 for DAC0 (left channel in stereo, or mono) */
	chan = snd_dmaengine_pcm_request_channel(mcf54418_dac_dma_filter,
						  (void *)(uintptr_t)MCF_EDMA_CHAN_DAC0);
	if (IS_ERR(chan)) {
		dev_err(component->dev, "Failed to request DMA channel %d (DAC0): %ld\n",
			MCF_EDMA_CHAN_DAC0, PTR_ERR(chan));
		ret = PTR_ERR(chan);
		goto err_free_prtd;
	}

	prtd->dma_chan_left = chan;
	prtd->dma_chan_right = NULL;  /* Allocated later if stereo */
	/* Set PCM hardware constraints */
	snd_soc_set_runtime_hwparams(substream, &mcf54418_dac_pcm_hardware);

	/* Store active substream for debugfs access */
	{
		struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
		struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
		struct mcf54418_dac *dac = snd_soc_dai_get_drvdata(cpu_dai);

		dac->active_substream = substream;
	}

	return 0;

err_free_prtd:
	kfree(prtd);
	return ret;
}

static int mcf54418_dac_pcm_close(struct snd_soc_component *component,
				  struct snd_pcm_substream *substream)
{
	struct mcf54418_dac_pcm_runtime *prtd = substream->runtime->private_data;
	struct mcf54418_dac *dac = snd_soc_component_get_drvdata(component);

	dac->active_substream = NULL;

	if (prtd) {
		/* Release left channel (always allocated) */
		if (prtd->dma_chan_left) {
			dmaengine_terminate_sync(prtd->dma_chan_left);
			dma_release_channel(prtd->dma_chan_left);
		}
		/* Release right channel (stereo or dual-mono) */
		if (prtd->dma_chan_right) {
			dmaengine_terminate_sync(prtd->dma_chan_right);
			dma_release_channel(prtd->dma_chan_right);
		}
		/* Free cyclic processing buffers (2 * process_buf_size each) */
		if (prtd->process_buf_dac0)
			dma_free_coherent(component->dev,
					  2 * prtd->process_buf_size,
					  prtd->process_buf_dac0,
					  prtd->process_dma_dac0);
		if (prtd->process_buf_dac1)
			dma_free_coherent(component->dev,
					  2 * prtd->process_buf_size,
					  prtd->process_buf_dac1,
					  prtd->process_dma_dac1);
		kfree(prtd);
	}

	return 0;
}

static int mcf54418_dac_pcm_hw_params(struct snd_soc_component *component,
				      struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct mcf54418_dac_pcm_runtime *prtd = substream->runtime->private_data;
	struct mcf54418_dac *dac = snd_soc_component_get_drvdata(component);
	size_t period_bytes = params_period_bytes(params);
	unsigned int periods = params_periods(params);
	unsigned int channels = params_channels(params);
	size_t process_buf_size;
	bool need_dac1;
	int ret;

	/* Save channel count */
	prtd->channels = channels;

	/* Need DAC1 if stereo OR if dual-mono mode is enabled */
	need_dac1 = (channels == 2) || dac->dual_mono;

	/* Allocate right channel DMA if needed */
	if (need_dac1 && !prtd->dma_chan_right) {
		struct dma_chan *chan;

		chan = snd_dmaengine_pcm_request_channel(mcf54418_dac_dma_filter,
							  (void *)(uintptr_t)MCF_EDMA_CHAN_DAC1);
		if (IS_ERR(chan)) {
			dev_err(component->dev, "Failed to request DMA channel %d (DAC1): %ld\n",
				MCF_EDMA_CHAN_DAC1, PTR_ERR(chan));
			return PTR_ERR(chan);
		}
		prtd->dma_chan_right = chan;
		dev_info(component->dev, "Allocated DMA channel %d for DAC1 (%s)\n",
			 MCF_EDMA_CHAN_DAC1,
			 dac->dual_mono ? "dual-mono" : "stereo");
	}

	/* Configure left channel DMA (DAC0) */
	memset(&prtd->slave_config_left, 0, sizeof(prtd->slave_config_left));
	prtd->slave_config_left.direction = DMA_MEM_TO_DEV;
	prtd->slave_config_left.dst_addr = 0xFC098002;  /* DAC0 VDACR */
	prtd->slave_config_left.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	prtd->slave_config_left.dst_maxburst = 1;

	ret = dmaengine_slave_config(prtd->dma_chan_left, &prtd->slave_config_left);
	if (ret) {
		dev_err(component->dev, "Failed to configure left DMA: %d\n", ret);
		return ret;
	}

	/* Configure right channel DMA (DAC1) if needed */
	if (need_dac1) {
		memset(&prtd->slave_config_right, 0, sizeof(prtd->slave_config_right));
		prtd->slave_config_right.direction = DMA_MEM_TO_DEV;
		prtd->slave_config_right.dst_addr = 0xFC09C002;  /* DAC1 VDACR */
		prtd->slave_config_right.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		prtd->slave_config_right.dst_maxburst = 1;

		ret = dmaengine_slave_config(prtd->dma_chan_right, &prtd->slave_config_right);
		if (ret) {
			dev_err(component->dev, "Failed to configure right DMA: %d\n", ret);
			return ret;
		}
	}

	/* Allocate cyclic processing buffers for volume application.
	 * Each buffer is 2 * process_buf_size (two halves for cyclic DMA).
	 * DMA loops over the whole buffer; INT_HALF/INT_MAJOR tell us
	 * which half to refill.
	 * For stereo: process_buf_size = period_bytes/2 (deinterleaved).
	 * For mono: process_buf_size = period_bytes.
	 */
	process_buf_size = (channels == 2) ? period_bytes / 2 : period_bytes;

	/* Free old buffers if size changed */
	if (prtd->process_buf_dac0 && prtd->process_buf_size != process_buf_size) {
		dma_free_coherent(component->dev, 2 * prtd->process_buf_size,
				  prtd->process_buf_dac0, prtd->process_dma_dac0);
		prtd->process_buf_dac0 = NULL;
	}
	if (prtd->process_buf_dac1 && prtd->process_buf_size != process_buf_size) {
		dma_free_coherent(component->dev, 2 * prtd->process_buf_size,
				  prtd->process_buf_dac1, prtd->process_dma_dac1);
		prtd->process_buf_dac1 = NULL;
	}

	/* Allocate DAC0 cyclic buffer (2 halves) */
	if (!prtd->process_buf_dac0) {
		prtd->process_buf_dac0 = dma_alloc_coherent(component->dev,
							    2 * process_buf_size,
							    &prtd->process_dma_dac0,
							    GFP_KERNEL);
		if (!prtd->process_buf_dac0) {
			dev_err(component->dev, "Failed to allocate DAC0 cyclic buffer\n");
			return -ENOMEM;
		}
	}

	/* Allocate DAC1 cyclic buffer if needed */
	if (need_dac1 && !prtd->process_buf_dac1) {
		prtd->process_buf_dac1 = dma_alloc_coherent(component->dev,
							    2 * process_buf_size,
							    &prtd->process_dma_dac1,
							    GFP_KERNEL);
		if (!prtd->process_buf_dac1) {
			dev_err(component->dev, "Failed to allocate DAC1 cyclic buffer\n");
			dma_free_coherent(component->dev, 2 * process_buf_size,
					  prtd->process_buf_dac0,
					  prtd->process_dma_dac0);
			prtd->process_buf_dac0 = NULL;
			return -ENOMEM;
		}
	}

	prtd->process_buf_size = process_buf_size;
	prtd->process_buf_idx = 0;

	/* Save period info */
	prtd->period_bytes = period_bytes;
	prtd->total_periods = periods;
	prtd->current_period = 0;
	prtd->dma_addr = substream->runtime->dma_addr;

	dev_info(component->dev, "PCM configured: %d ch, period=%zu, buf_size=%zu, dual_mono=%d\n",
		 channels, period_bytes, process_buf_size, dac->dual_mono);

	return 0;
}

/* Forward declarations */
static void mcf54418_dac_dma_complete(void *data);

/* Start cyclic DMA for one DAC channel.
 * The DMA loops over the process buffer (2 halves) using a single TCD
 * with SLAST auto-wrap + INT_HALF/INT_MAJOR. No gaps between periods.
 */
static int mcf54418_dac_start_cyclic(struct snd_pcm_substream *substream,
				     struct dma_chan *chan,
				     dma_addr_t buf_addr, size_t half_size,
				     struct dma_async_tx_descriptor **out_desc,
				     dma_cookie_t *out_cookie)
{
	struct dma_async_tx_descriptor *desc;

	desc = dmaengine_prep_dma_cyclic(chan, buf_addr,
					 2 * half_size, half_size,
					 DMA_MEM_TO_DEV,
					 DMA_PREP_INTERRUPT);
	if (!desc)
		return -ENOMEM;

	desc->callback = mcf54418_dac_dma_complete;
	desc->callback_param = substream;

	*out_desc = desc;
	*out_cookie = dmaengine_submit(desc);
	if (dma_submit_error(*out_cookie))
		return -EIO;

	dma_async_issue_pending(chan);
	return 0;
}

/* PCM trigger callback */
static int mcf54418_dac_pcm_trigger(struct snd_soc_component *component,
				    struct snd_pcm_substream *substream, int cmd)
{
	struct mcf54418_dac_pcm_runtime *prtd = substream->runtime->private_data;
	struct mcf54418_dac *dac = snd_soc_component_get_drvdata(component);
	unsigned long flags;
	const void *src;
	bool need_dac1;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		need_dac1 = (prtd->channels == 2) || dac->dual_mono;

		spin_lock_irqsave(&prtd->lock, flags);
		prtd->current_period = 0;
		prtd->next_source_period = 0;
		prtd->process_buf_idx = 0;
		prtd->running = true;
		atomic_set(&prtd->dma_complete_count, 0);
		spin_unlock_irqrestore(&prtd->lock, flags);

		/* Fill both halves of the cyclic buffer with first 2 periods */
		src = substream->runtime->dma_area;
		prtd->process_buf_idx = 0;
		mcf54418_dac_process_period(dac, prtd, src, prtd->period_bytes);

		src = substream->runtime->dma_area + prtd->period_bytes;
		prtd->process_buf_idx = 1;
		mcf54418_dac_process_period(dac, prtd, src, prtd->period_bytes);

		/* Next source period to read when a half completes */
		prtd->next_source_period = 2 % prtd->total_periods;
		prtd->process_buf_idx = 0;

		/* Start cyclic DMA on DAC0 */
		ret = mcf54418_dac_start_cyclic(substream,
						prtd->dma_chan_left,
						prtd->process_dma_dac0,
						prtd->process_buf_size,
						&prtd->desc_left,
						&prtd->cookie_left);
		if (ret) {
			dev_err(component->dev,
				"Failed to start DAC0 cyclic DMA: %d\n", ret);
			prtd->running = false;
			return ret;
		}

		/* Start cyclic DMA on DAC1 if needed */
		if (need_dac1 && prtd->dma_chan_right) {
			ret = mcf54418_dac_start_cyclic(substream,
							prtd->dma_chan_right,
							prtd->process_dma_dac1,
							prtd->process_buf_size,
							&prtd->desc_right,
							&prtd->cookie_right);
			if (ret) {
				dev_err(component->dev,
					"Failed to start DAC1 cyclic DMA: %d\n", ret);
				dmaengine_terminate_sync(prtd->dma_chan_left);
				prtd->running = false;
				return ret;
			}
		}

		return 0;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		spin_lock_irqsave(&prtd->lock, flags);
		prtd->running = false;
		spin_unlock_irqrestore(&prtd->lock, flags);

		dmaengine_terminate_sync(prtd->dma_chan_left);
		if (prtd->dma_chan_right)
			dmaengine_terminate_sync(prtd->dma_chan_right);

		return 0;

	default:
		return -EINVAL;
	}
}
static snd_pcm_uframes_t mcf54418_dac_pcm_pointer(struct snd_soc_component *component,
						  struct snd_pcm_substream *substream)
{
	struct mcf54418_dac_pcm_runtime *prtd = substream->runtime->private_data;
	snd_pcm_uframes_t frames;
	unsigned long flags;

	spin_lock_irqsave(&prtd->lock, flags);

	/* Return position at start of current period */
	frames = bytes_to_frames(substream->runtime,
				 prtd->current_period * prtd->period_bytes);

	spin_unlock_irqrestore(&prtd->lock, flags);

	return frames;
}



/* Cyclic DMA callback.
 * Called at INT_HALF (first half played) and INT_MAJOR (second half played).
 * Refills the half that just finished with the next source period.
 * DMA never stops — it loops continuously over the process buffer.
 */
static void mcf54418_dac_dma_complete(void *data)
{
	struct snd_pcm_substream *substream = data;
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *cpu_dai;
	struct mcf54418_dac *dac;
	struct mcf54418_dac_pcm_runtime *prtd;
	unsigned long flags;
	int completed;
	int expected_callbacks;
	const void *src;

	if (!substream || !substream->runtime ||
	    !substream->runtime->private_data)
		return;

	prtd = substream->runtime->private_data;

	rtd = snd_soc_substream_to_rtd(substream);
	cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	dac = snd_soc_dai_get_drvdata(cpu_dai);

	/* For stereo/dual-mono: wait for both DAC channels */
	expected_callbacks = ((prtd->channels == 2) || dac->dual_mono) ? 2 : 1;
	completed = atomic_inc_return(&prtd->dma_complete_count);
	if (completed < expected_callbacks)
		return;
	atomic_set(&prtd->dma_complete_count, 0);

	spin_lock_irqsave(&prtd->lock, flags);
	if (!prtd->running) {
		spin_unlock_irqrestore(&prtd->lock, flags);
		return;
	}

	/* Advance ALSA period pointer */
	prtd->current_period++;
	if (prtd->current_period >= prtd->total_periods)
		prtd->current_period = 0;

	spin_unlock_irqrestore(&prtd->lock, flags);

	/* Notify ALSA that a period elapsed */
	snd_pcm_period_elapsed(substream);

	if (!prtd->running)
		return;

	/* Refill the half that just finished playing with next source period.
	 * process_buf_idx indicates which half to refill.
	 */
	src = substream->runtime->dma_area +
	      (prtd->next_source_period * prtd->period_bytes);

	mcf54418_dac_process_period(dac, prtd, src, prtd->period_bytes);

	/* Advance to next source period and flip half index */
	prtd->next_source_period =
		(prtd->next_source_period + 1) % prtd->total_periods;
	prtd->process_buf_idx ^= 1;
}


static int mcf54418_dac_pcm_construct(struct snd_soc_component *component,
				      struct snd_soc_pcm_runtime *rtd)
{
	struct snd_pcm *pcm = rtd->pcm;
	size_t size = mcf54418_dac_pcm_hardware.buffer_bytes_max;

	return snd_pcm_set_managed_buffer_all(pcm, SNDRV_DMA_TYPE_DEV,
					      component->dev, size, size);
}

/* Custom PCM component driver with trigger ordering
 *
 * Problem Double-trigger issue - both manual call and SOC core call DAI trigger:
 *   [  184.680286] DAI trigger: cmd=1 (START)  <- Manual call from component
 *   [  184.730322] Submitting period 0         <- DMA submitted
 *   [  184.821328] DAI trigger: cmd=1 (START)  <- SOC core call (91ms later!)
 *
 * The second trigger call RESTARTS DTIM3 (writes 0x0000 then 0x002B to DTMR),
 * which disrupts the DMA transfer that's already in progress!
 *
 * Solution: Use trigger_start = SND_SOC_TRIGGER_ORDER_LDC to control call order:
 *   - DEFAULT order: Link → Component → DAI (component trigger calls DAI manually)
 *   - LDC order: Link → DAI → Component (DAI called automatically BEFORE component)
 *
 * With LDC order:
 *   1. SOC core calls DAI trigger (starts DTIM3)
 *   2. SOC core calls component trigger (submits DMA with DTIM3 already running)
 *   3. No double-trigger, correct ordering, no DTIM3 restart!
 */
static const struct snd_soc_component_driver mcf54418_dac_custom_pcm_component = {
	.name			= "mcf54418-dac-pcm",
	.open			= mcf54418_dac_pcm_open,
	.close			= mcf54418_dac_pcm_close,
	/* .ioctl - Let ALSA core handle all ioctls (standard approach) */
	.hw_params		= mcf54418_dac_pcm_hw_params,
	.trigger		= mcf54418_dac_pcm_trigger,
	.pointer		= mcf54418_dac_pcm_pointer,
	.pcm_construct		= mcf54418_dac_pcm_construct,

	.trigger_start		= SND_SOC_TRIGGER_ORDER_LDC,
};

static const struct snd_dmaengine_pcm_config mcf54418_dac_dmaengine_pcm_config = {
	.prepare_slave_config = mcf54418_dac_prepare_slave_config,
	.compat_request_channel = mcf54418_dac_compat_request_channel,
	.pcm_hardware = &mcf54418_dac_pcm_hardware,
};

static int mcf54418_dac_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct mcf54418_dac *dac = snd_soc_dai_get_drvdata(dai);
	unsigned int rate = params_rate(params);
	unsigned int channels = params_channels(params);
	snd_pcm_format_t format = params_format(params);
	u16 cr;
	int i;

	dev_info(dac->dev, "hw_params: rate=%u, channels=%u, format=%d\n",
		rate, channels, format);

	dac->sample_rate = rate;
	dac->channels = channels;
	dac->format = format;

	/* Configure DAC control register(s) - initialize both DAC0 and DAC1 for stereo
	 * Use right-justified format (FORMAT=0) like original driver.
	 * DAC uses bits [11:0] of the 16-bit data register.
	 */
	for (i = 0; i < channels; i++) {
		cr = DAC_CR_WMLVL_2;  /* Watermark=2, right-justified (FORMAT=0) */
		dac_writel(dac, i, DAC_CR, cr);
		dev_info(dac->dev, "Initialized DAC%d: CR=0x%04x\n", i, cr);
	}

	mcf54418_dac_setup_timer(dac, rate);

	return 0;
}

static int mcf54418_dac_prepare(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	struct mcf54418_dac *dac = snd_soc_dai_get_drvdata(dai);

	dev_info(dac->dev, "prepare called\n");
	return 0;
}

static int mcf54418_dac_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	struct mcf54418_dac *dac = snd_soc_dai_get_drvdata(dai);

	/* ITERATION 60: Enhanced logging to track DAI trigger execution */
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/* DMA is handled by dmaengine PCM, just enable DAC hardware */		mcf54418_dac_enable(dac, true);		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* DMA is handled by dmaengine PCM, just disable DAC hardware */		mcf54418_dac_enable(dac, false);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int mcf54418_dac_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct mcf54418_dac *dac = snd_soc_dai_get_drvdata(dai);

	dev_info(dac->dev, "startup called\n");

	/* Set DMA data for dmaengine PCM (prepare_slave_config needs this) */
	snd_soc_dai_init_dma_data(dai, &dac->dma_params_tx, NULL);

	dev_info(dac->dev, "DMA params: addr=0x%pad, width=%d, burst=%d, filter_data=%p\n",
		 &dac->dma_params_tx.addr, dac->dma_params_tx.addr_width,
		 dac->dma_params_tx.maxburst, dac->dma_params_tx.filter_data);

	return 0;
}

static const struct snd_soc_dai_ops mcf54418_dac_dai_ops = {
	.startup	= mcf54418_dac_startup,
	.hw_params	= mcf54418_dac_hw_params,
	.prepare	= mcf54418_dac_prepare,
	.trigger	= mcf54418_dac_trigger,
};

static struct snd_soc_dai_driver mcf54418_dac_dai = {
	.name = "mcf54418-dac",
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 2,  /* Stereo supported with custom TCD programming (SOFF=4) */
		.rates		= SNDRV_PCM_RATE_8000_48000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S16_BE |
				  SNDRV_PCM_FMTBIT_U16_BE,
	},
	.ops = &mcf54418_dac_dai_ops,
};

/* Per-channel volume controls (REQ 05100) */
static int mcf54418_dac_volume_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = DAC_VOLUME_MAX;
	return 0;
}

static int mcf54418_dac0_volume_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct mcf54418_dac *dac = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = dac->volume_dac0;
	return 0;
}

static int mcf54418_dac0_volume_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct mcf54418_dac *dac = snd_soc_component_get_drvdata(component);
	unsigned int vol = ucontrol->value.integer.value[0];

	if (vol > DAC_VOLUME_MAX)
		return -EINVAL;

	if (dac->volume_dac0 == vol)
		return 0;

	dac->volume_dac0 = vol;
	return 1;
}

static int mcf54418_dac1_volume_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct mcf54418_dac *dac = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = dac->volume_dac1;
	return 0;
}

static int mcf54418_dac1_volume_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct mcf54418_dac *dac = snd_soc_component_get_drvdata(component);
	unsigned int vol = ucontrol->value.integer.value[0];

	if (vol > DAC_VOLUME_MAX)
		return -EINVAL;

	if (dac->volume_dac1 == vol)
		return 0;

	dac->volume_dac1 = vol;
	return 1;
}

/* Dual Mono Switch (REQ 05050) */
static int mcf54418_dac_dual_mono_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct mcf54418_dac *dac = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = dac->dual_mono ? 1 : 0;
	return 0;
}

static int mcf54418_dac_dual_mono_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct mcf54418_dac *dac = snd_soc_component_get_drvdata(component);
	bool enable = ucontrol->value.integer.value[0] != 0;

	if (dac->dual_mono == enable)
		return 0;

	dac->dual_mono = enable;
	dev_info(dac->dev, "Dual mono mode %s\n", enable ? "enabled" : "disabled");
	return 1;
}

/* Playback Switch (Mute control) */
static int mcf54418_dac_mute_info(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int mcf54418_dac_mute_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct mcf54418_dac *dac = snd_soc_component_get_drvdata(component);
	int val = gpio_get_value(dac->gpio_mute);

	/* Invert if needed: GPIO HIGH = unmuted = control value 1 */
	ucontrol->value.integer.value[0] = dac->mute_inverted ? !val : val;
	return 0;
}

static int mcf54418_dac_mute_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct mcf54418_dac *dac = snd_soc_component_get_drvdata(component);
	int enable = ucontrol->value.integer.value[0];
	int val = dac->mute_inverted ? !enable : enable;

	gpio_set_value(dac->gpio_mute, val);
	return 1;
}

/* Amplifier Power Switch (Shutdown control) */
static int mcf54418_dac_amp_power_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int mcf54418_dac_amp_power_get(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct mcf54418_dac *dac = snd_soc_component_get_drvdata(component);
	int val = gpio_get_value(dac->gpio_shutdown);

	/* Invert if needed: GPIO HIGH = powered on = control value 1 */
	ucontrol->value.integer.value[0] = dac->shutdown_inverted ? !val : val;
	return 0;
}

static int mcf54418_dac_amp_power_put(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct mcf54418_dac *dac = snd_soc_component_get_drvdata(component);
	int enable = ucontrol->value.integer.value[0];
	int val = dac->shutdown_inverted ? !enable : enable;

	gpio_set_value(dac->gpio_shutdown, val);
	return 1;
}

static const struct snd_kcontrol_new mcf54418_dac_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "DAC0 Playback Volume",
		.info = mcf54418_dac_volume_info,
		.get = mcf54418_dac0_volume_get,
		.put = mcf54418_dac0_volume_put,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "DAC1 Playback Volume",
		.info = mcf54418_dac_volume_info,
		.get = mcf54418_dac1_volume_get,
		.put = mcf54418_dac1_volume_put,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Dual Mono Switch",
		.info = snd_ctl_boolean_mono_info,
		.get = mcf54418_dac_dual_mono_get,
		.put = mcf54418_dac_dual_mono_put,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Playback Switch",
		.info = mcf54418_dac_mute_info,
		.get = mcf54418_dac_mute_get,
		.put = mcf54418_dac_mute_put,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Amplifier Power Switch",
		.info = mcf54418_dac_amp_power_info,
		.get = mcf54418_dac_amp_power_get,
		.put = mcf54418_dac_amp_power_put,
	},
};

/* Add custom PCM ops to DAI component to fix registration conflict.
 * Previously we had TWO components registered on same device (mcfdac.0):
 *   1) mcf54418_dac_component (DAI driver)
 *   2) mcf54418_dac_custom_pcm_component (custom PCM)
 * This caused ALSA to pick wrong component. Solution: Merge PCM ops into DAI component.
 */
static const struct snd_soc_component_driver mcf54418_dac_component = {
	.name			= "mcf54418-dac",
	.controls		= mcf54418_dac_controls,
	.num_controls		= ARRAY_SIZE(mcf54418_dac_controls),

	.open			= mcf54418_dac_pcm_open,
	.close			= mcf54418_dac_pcm_close,
	.hw_params		= mcf54418_dac_pcm_hw_params,
	.trigger		= mcf54418_dac_pcm_trigger,
	.pointer		= mcf54418_dac_pcm_pointer,
	.pcm_construct		= mcf54418_dac_pcm_construct,
};

/* ============================================================
 * debugfs interface for DAC debug and verification
 * ============================================================
 */

#ifdef CONFIG_DEBUG_FS

#define DAC_DEBUGFS_SNAPSHOT_SAMPLES	32

static int dac_state_show(struct seq_file *s, void *data)
{
	struct mcf54418_dac *dac = s->private;
	struct snd_pcm_substream *substream;
	struct mcf54418_dac_pcm_runtime *prtd = NULL;
	u16 cr0, cr1, sr0, sr1;
	u16 dtmr, dtrr_lo, dtrr_hi, dtcn_lo, dtcn_hi;
	u32 dtrr, dtcn;
	u16 dactsr, misccr2;

	/* DAC registers */
	cr0 = dac_readl(dac, 0, DAC_CR);
	cr1 = dac_readl(dac, 1, DAC_CR);
	sr0 = dac_readl(dac, 0, DAC_SR);
	sr1 = dac_readl(dac, 1, DAC_SR);

	/* Timer registers */
	dtmr = readw(dac->dtim_base + DTIM_DTMR);
	dtrr = readl(dac->dtim_base + DTIM_DTRR);
	dtcn = readl(dac->dtim_base + DTIM_DTCN);

	/* CCM registers */
	dactsr = readw(dac->ccm_base + CCM_DACTSR);
	misccr2 = readw(dac->ccm_base + CCM_MISCCR2);

	seq_puts(s, "=== MCF54418 DAC State ===\n\n");

	/* DAC registers */
	seq_printf(s, "DAC0 CR:  0x%04x  [PDN=%d DMAEN=%d SYNC=%d FMT=%d WMLVL=%d]\n",
		   cr0, !!(cr0 & DAC_CR_PDN), !!(cr0 & DAC_CR_DMAEN),
		   !!(cr0 & DAC_CR_SYNC_EN), !!(cr0 & DAC_CR_FORMAT),
		   (cr0 & DAC_CR_WMLVL_MASK) >> DAC_CR_WMLVL_SHIFT);
	seq_printf(s, "DAC1 CR:  0x%04x  [PDN=%d DMAEN=%d SYNC=%d FMT=%d WMLVL=%d]\n",
		   cr1, !!(cr1 & DAC_CR_PDN), !!(cr1 & DAC_CR_DMAEN),
		   !!(cr1 & DAC_CR_SYNC_EN), !!(cr1 & DAC_CR_FORMAT),
		   (cr1 & DAC_CR_WMLVL_MASK) >> DAC_CR_WMLVL_SHIFT);
	seq_printf(s, "DAC0 SR:  0x%04x  [FULL=%d EMPTY=%d]\n",
		   sr0, !!(sr0 & DAC_SR_FULL), !!(sr0 & DAC_SR_EMPTY));
	seq_printf(s, "DAC1 SR:  0x%04x  [FULL=%d EMPTY=%d]\n",
		   sr1, !!(sr1 & DAC_SR_FULL), !!(sr1 & DAC_SR_EMPTY));
	seq_putc(s, '\n');

	/* Timer */
	seq_printf(s, "DTIM3 DTMR: 0x%04x  [RST=%d CLK=%d FRR=%d ORRI=%d OM=%d]\n",
		   dtmr, !!(dtmr & DTIM_DTMR_RST),
		   (dtmr >> 1) & 0x03, !!(dtmr & DTIM_DTMR_FRR),
		   !!(dtmr & DTIM_DTMR_ORRI), !!(dtmr & DTIM_DTMR_OM));
	seq_printf(s, "DTIM3 DTRR: 0x%08x (%u)\n", dtrr, dtrr);
	seq_printf(s, "DTIM3 DTCN: 0x%08x (%u)\n", dtcn, dtcn);
	if (dac->sample_rate && dac->bus_clk_rate) {
		u32 expected = dac->bus_clk_rate / dac->sample_rate;
		seq_printf(s, "Expected DTRR: %u (bus_clk=%u / rate=%u)\n",
			   expected, dac->bus_clk_rate, dac->sample_rate);
	}
	seq_putc(s, '\n');

	/* CCM */
	seq_printf(s, "CCM DACTSR:  0x%04x  [DAC0: ch=%d src=%d | DAC1: ch=%d src=%d]\n",
		   dactsr,
		   (dactsr >> 3) & 0x03, dactsr & 0x07,
		   (dactsr >> 11) & 0x03, (dactsr >> 8) & 0x07);
	seq_printf(s, "CCM MISCCR2: 0x%04x  [DAC0SEL=%d DAC1SEL=%d]\n",
		   misccr2, !!(misccr2 & CCM_MISCCR2_DAC0SEL),
		   !!(misccr2 & CCM_MISCCR2_DAC1SEL));
	seq_putc(s, '\n');

	/* Volume & audio config */
	seq_printf(s, "Volume DAC0: %u/%u\n", dac->volume_dac0, DAC_VOLUME_MAX);
	seq_printf(s, "Volume DAC1: %u/%u\n", dac->volume_dac1, DAC_VOLUME_MAX);
	seq_printf(s, "Sample rate: %u Hz\n", dac->sample_rate);
	seq_printf(s, "Channels:    %u\n", dac->channels);
	seq_printf(s, "Format:      %d\n", dac->format);
	seq_printf(s, "Dual mono:   %s\n", dac->dual_mono ? "yes" : "no");
	seq_printf(s, "Enabled:     %s\n", dac->enabled ? "yes" : "no");
	seq_putc(s, '\n');

	/* GPIO state */
	seq_printf(s, "GPIO mute:     %d (pin %d, inverted=%d)\n",
		   gpio_get_value(dac->gpio_mute), dac->gpio_mute,
		   dac->mute_inverted);
	seq_printf(s, "GPIO shutdown: %d (pin %d, inverted=%d)\n",
		   gpio_get_value(dac->gpio_shutdown), dac->gpio_shutdown,
		   dac->shutdown_inverted);
	seq_putc(s, '\n');

	/* PCM runtime info */
	substream = dac->active_substream;
	if (substream && substream->runtime) {
		prtd = substream->runtime->private_data;
		if (prtd) {
			seq_printf(s, "DMA running:      %s\n",
				   prtd->running ? "yes" : "no");
			seq_printf(s, "Current period:   %u\n",
				   prtd->current_period);
			seq_printf(s, "Next source:      %u\n",
				   prtd->next_source_period);
			seq_printf(s, "Total periods:    %u\n",
				   prtd->total_periods);
			seq_printf(s, "Period bytes:     %zu\n",
				   prtd->period_bytes);
			seq_printf(s, "Process buf size: %zu\n",
				   prtd->process_buf_size);
			seq_printf(s, "DMA addr:         0x%pad\n",
				   &prtd->dma_addr);
			seq_printf(s, "DAC0 slave dst:   0x%pad\n",
				   &prtd->slave_config_left.dst_addr);
			if (prtd->dma_chan_right)
				seq_printf(s, "DAC1 slave dst:   0x%pad\n",
					   &prtd->slave_config_right.dst_addr);
		}
	} else {
		seq_puts(s, "No active PCM substream\n");
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(dac_state);

static ssize_t dma_buffer_read(struct file *file, char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	struct mcf54418_dac *dac = file->private_data;
	struct snd_pcm_substream *substream;
	struct mcf54418_dac_pcm_runtime *prtd;
	struct {
		u32 magic;
		u32 period_bytes;
		u32 channels;
		u32 format;
		u32 current_period;
	} __packed header;
	const void *period_data;
	size_t total_size;
	char *buf;
	ssize_t ret;

	substream = dac->active_substream;
	if (!substream || !substream->runtime ||
	    !substream->runtime->private_data)
		return -ENODEV;

	prtd = substream->runtime->private_data;
	if (!prtd->period_bytes || !substream->runtime->dma_area)
		return -ENODEV;

	header.magic = 0x44414342;  /* "DACB" */
	header.period_bytes = prtd->period_bytes;
	header.channels = prtd->channels;
	header.format = dac->format;
	header.current_period = prtd->current_period;

	total_size = sizeof(header) + prtd->period_bytes;
	buf = kmalloc(total_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, &header, sizeof(header));
	period_data = substream->runtime->dma_area +
		      (prtd->current_period * prtd->period_bytes);
	memcpy(buf + sizeof(header), period_data, prtd->period_bytes);

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, total_size);
	kfree(buf);
	return ret;
}

static const struct file_operations dma_buffer_fops = {
	.open = simple_open,
	.read = dma_buffer_read,
	.llseek = default_llseek,
};

static ssize_t process_buf_dac_read(struct file *file, char __user *user_buf,
				    size_t count, loff_t *ppos, int dac_num)
{
	struct mcf54418_dac *dac = file->private_data;
	struct snd_pcm_substream *substream;
	struct mcf54418_dac_pcm_runtime *prtd;
	void *buf;
	size_t size;

	substream = dac->active_substream;
	if (!substream || !substream->runtime ||
	    !substream->runtime->private_data)
		return -ENODEV;

	prtd = substream->runtime->private_data;
	/* Show the last-written half of the cyclic buffer */
	{
		size_t off = (prtd->process_buf_idx ^ 1) * prtd->process_buf_size;

		buf = dac_num ? prtd->process_buf_dac1 + off
			      : prtd->process_buf_dac0 + off;
	}
	size = prtd->process_buf_size;

	if (!buf || !size)
		return -ENODEV;

	return simple_read_from_buffer(user_buf, count, ppos, buf, size);
}

static ssize_t process_buf_dac0_read(struct file *file, char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	return process_buf_dac_read(file, user_buf, count, ppos, 0);
}

static ssize_t process_buf_dac1_read(struct file *file, char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	return process_buf_dac_read(file, user_buf, count, ppos, 1);
}

static const struct file_operations process_buf_dac0_fops = {
	.open = simple_open,
	.read = process_buf_dac0_read,
	.llseek = default_llseek,
};

static const struct file_operations process_buf_dac1_fops = {
	.open = simple_open,
	.read = process_buf_dac1_read,
	.llseek = default_llseek,
};

static int dac_snapshot_show(struct seq_file *s, void *data)
{
	struct mcf54418_dac *dac = s->private;
	struct snd_pcm_substream *substream;
	struct mcf54418_dac_pcm_runtime *prtd;
	const u16 *alsa_buf;
	const u16 *dac0_buf;
	const u16 *dac1_buf;
	unsigned int n_samples, n_show, i;
	bool all_in_range = true;
	bool need_dac1;

	substream = dac->active_substream;
	if (!substream || !substream->runtime ||
	    !substream->runtime->private_data) {
		seq_puts(s, "No active PCM substream\n");
		return 0;
	}

	prtd = substream->runtime->private_data;
	if (!prtd->period_bytes || !substream->runtime->dma_area) {
		seq_puts(s, "No DMA buffer available\n");
		return 0;
	}

	need_dac1 = (prtd->channels == 2) || dac->dual_mono;

	/* Point to current period in ALSA buffer */
	alsa_buf = (const u16 *)(substream->runtime->dma_area +
				  (prtd->current_period * prtd->period_bytes));
	{
		size_t off = (prtd->process_buf_idx ^ 1) * prtd->process_buf_size;

		dac0_buf = prtd->process_buf_dac0 + off;
		dac1_buf = prtd->process_buf_dac1 + off;
	}

	/* Number of per-channel samples in the process buffer */
	n_samples = prtd->process_buf_size / 2;
	n_show = min_t(unsigned int, n_samples, DAC_DEBUGFS_SNAPSHOT_SAMPLES);

	/* ALSA buffer dump */
	seq_printf(s, "ALSA buffer (period %u, %s %s):\n",
		   prtd->current_period,
		   prtd->channels == 2 ? "stereo" : "mono",
		   "U16_BE");

	if (prtd->channels == 2) {
		for (i = 0; i < n_show; i++)
			seq_printf(s, "  [%04u] L=0x%04x R=0x%04x\n",
				   i, alsa_buf[i * 2], alsa_buf[i * 2 + 1]);
	} else {
		for (i = 0; i < n_show; i++)
			seq_printf(s, "  [%04u] 0x%04x\n", i, alsa_buf[i]);
	}
	seq_putc(s, '\n');

	/* DAC0 process buffer */
	if (dac0_buf) {
		seq_printf(s, "DAC0 process buffer (after volume=%u):\n",
			   dac->volume_dac0);
		for (i = 0; i < n_show; i++) {
			u16 sample = dac0_buf[i];

			seq_printf(s, "  [%04u] 0x%04x", i, sample);
			if (sample > 0x0FFF) {
				seq_puts(s, "  ** OUT OF 12-BIT RANGE **");
				all_in_range = false;
			}
			seq_putc(s, '\n');
		}
		seq_putc(s, '\n');
	}

	/* DAC1 process buffer */
	if (need_dac1 && dac1_buf) {
		seq_printf(s, "DAC1 process buffer (after volume=%u):\n",
			   dac->volume_dac1);
		for (i = 0; i < n_show; i++) {
			u16 sample = dac1_buf[i];

			seq_printf(s, "  [%04u] 0x%04x", i, sample);
			if (sample > 0x0FFF) {
				seq_puts(s, "  ** OUT OF 12-BIT RANGE **");
				all_in_range = false;
			}
			seq_putc(s, '\n');
		}
		seq_putc(s, '\n');
	}

	/* Full range check on all samples */
	if (dac0_buf) {
		for (i = 0; i < n_samples; i++) {
			if (dac0_buf[i] > 0x0FFF) {
				all_in_range = false;
				break;
			}
		}
	}
	if (all_in_range && need_dac1 && dac1_buf) {
		for (i = 0; i < n_samples; i++) {
			if (dac1_buf[i] > 0x0FFF) {
				all_in_range = false;
				break;
			}
		}
	}

	seq_printf(s, "12-bit range check [0x000..0xFFF]: %s\n",
		   all_in_range ? "OK" : "FAIL");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(dac_snapshot);

/*
 * inject_test: write test patterns into the ALSA DMA buffer.
 * Commands: "silence", "ramp", "square <freq>", "sine <freq>"
 */

/* Simple 256-entry sine table (Q15 format, amplitude ~2047 for 12-bit DAC) */
static const s16 sine_table_256[256] = {
	   0,   50,  100,  150,  200,  249,  297,  345,
	 392,  437,  482,  526,  568,  609,  649,  687,
	 724,  759,  792,  823,  852,  879,  904,  927,
	 947,  966,  982,  995, 1007, 1016, 1022, 1026,
	1028, 1026, 1022, 1016, 1007,  995,  982,  966,
	 947,  927,  904,  879,  852,  823,  792,  759,
	 724,  687,  649,  609,  568,  526,  482,  437,
	 392,  345,  297,  249,  200,  150,  100,   50,
	   0,  -50, -100, -150, -200, -249, -297, -345,
	-392, -437, -482, -526, -568, -609, -649, -687,
	-724, -759, -792, -823, -852, -879, -904, -927,
	-947, -966, -982, -995,
	-1007, -1016, -1022, -1026, -1028, -1026, -1022, -1016,
	-1007, -995, -982, -966,
	-947, -927, -904, -879, -852, -823, -792, -759,
	-724, -687, -649, -609, -568, -526, -482, -437,
	-392, -345, -297, -249, -200, -150, -100, -50,
	0, 50, 100, 150, 200, 249, 297, 345,
	392, 437, 482, 526, 568, 609, 649, 687,
	724, 759, 792, 823, 852, 879, 904, 927,
	947, 966, 982, 995, 1007, 1016, 1022, 1026,
	1028, 1026, 1022, 1016, 1007, 995, 982, 966,
	947, 927, 904, 879, 852, 823, 792, 759,
	724, 687, 649, 609, 568, 526, 482, 437,
	392, 345, 297, 249, 200, 150, 100, 50,
	0, -50, -100, -150, -200, -249, -297, -345,
	-392, -437, -482, -526, -568, -609, -649, -687,
	-724, -759, -792, -823, -852, -879, -904, -927,
	-947, -966, -982, -995,
	-1007, -1016, -1022, -1026, -1028, -1026, -1022, -1016,
	-1007, -995, -982, -966,
	-947, -927, -904, -879, -852, -823, -792, -759,
	-724, -687, -649, -609, -568, -526, -482, -437,
	-392, -345, -297, -249, -200, -150, -100, -50,
};

static ssize_t inject_test_write(struct file *file, const char __user *user_buf,
				 size_t count, loff_t *ppos)
{
	struct mcf54418_dac *dac = file->private_data;
	struct snd_pcm_substream *substream;
	struct mcf54418_dac_pcm_runtime *prtd;
	char cmd[64];
	u16 *buf;
	size_t total_samples, i;
	unsigned int freq = 0;
	size_t len;

	if (count >= sizeof(cmd))
		return -EINVAL;

	if (copy_from_user(cmd, user_buf, count))
		return -EFAULT;

	cmd[count] = '\0';
	/* Strip trailing newline */
	len = strlen(cmd);
	while (len > 0 && (cmd[len - 1] == '\n' || cmd[len - 1] == '\r'))
		cmd[--len] = '\0';

	substream = dac->active_substream;
	if (!substream || !substream->runtime ||
	    !substream->runtime->private_data)
		return -ENODEV;

	prtd = substream->runtime->private_data;
	if (!substream->runtime->dma_area || !prtd->period_bytes)
		return -ENODEV;

	/* Fill ALL periods in the buffer */
	buf = (u16 *)substream->runtime->dma_area;
	total_samples = (prtd->period_bytes * prtd->total_periods) / 2;

	if (strcmp(cmd, "silence") == 0) {
		for (i = 0; i < total_samples; i++)
			buf[i] = 0x0800;
		dev_info(dac->dev, "inject_test: silence (0x0800) x %zu\n",
			 total_samples);
	} else if (strcmp(cmd, "ramp") == 0) {
		for (i = 0; i < total_samples; i++)
			buf[i] = (u16)(i & 0x0FFF);
		dev_info(dac->dev, "inject_test: ramp 0x000->0xFFF x %zu\n",
			 total_samples);
	} else if (sscanf(cmd, "square %u", &freq) == 1) {
		unsigned int half_period;

		if (freq == 0 || !dac->sample_rate)
			return -EINVAL;
		half_period = dac->sample_rate / (2 * freq);
		if (half_period == 0)
			half_period = 1;
		for (i = 0; i < total_samples; i++) {
			if ((i / half_period) & 1)
				buf[i] = 0x0200;  /* Low */
			else
				buf[i] = 0x0E00;  /* High */
		}
		dev_info(dac->dev, "inject_test: square %u Hz x %zu\n",
			 freq, total_samples);
	} else if (sscanf(cmd, "sine %u", &freq) == 1) {
		unsigned int phase_inc;

		if (freq == 0 || !dac->sample_rate)
			return -EINVAL;
		/* phase_inc = 256 * freq / sample_rate (fixed point) */
		phase_inc = (256 * freq) / dac->sample_rate;
		if (phase_inc == 0)
			phase_inc = 1;
		for (i = 0; i < total_samples; i++) {
			unsigned int idx = (i * phase_inc) & 0xFF;

			buf[i] = (u16)(sine_table_256[idx] + 0x0800);
		}
		dev_info(dac->dev, "inject_test: sine %u Hz x %zu\n",
			 freq, total_samples);
	} else {
		dev_err(dac->dev, "inject_test: unknown command '%s'\n", cmd);
		return -EINVAL;
	}

	return count;
}

static const struct file_operations inject_test_fops = {
	.open = simple_open,
	.write = inject_test_write,
	.llseek = noop_llseek,
};

static void mcf54418_dac_debugfs_init(struct mcf54418_dac *dac)
{
	struct dentry *root;

	root = debugfs_create_dir("mcf54418-dac", NULL);
	dac->debugfs_root = root;

	debugfs_create_file("dac_state", 0444, root, dac, &dac_state_fops);
	debugfs_create_file("dma_buffer", 0444, root, dac, &dma_buffer_fops);
	debugfs_create_file("process_buf_dac0", 0444, root, dac,
			    &process_buf_dac0_fops);
	debugfs_create_file("process_buf_dac1", 0444, root, dac,
			    &process_buf_dac1_fops);
	debugfs_create_file("dac_snapshot", 0444, root, dac,
			    &dac_snapshot_fops);
	debugfs_create_file("inject_test", 0200, root, dac,
			    &inject_test_fops);
}

static void mcf54418_dac_debugfs_exit(struct mcf54418_dac *dac)
{
	debugfs_remove_recursive(dac->debugfs_root);
}

#else /* !CONFIG_DEBUG_FS */

static inline void mcf54418_dac_debugfs_init(struct mcf54418_dac *dac) {}
static inline void mcf54418_dac_debugfs_exit(struct mcf54418_dac *dac) {}

#endif /* CONFIG_DEBUG_FS */

static int mcf54418_dac_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mcf54418_dac *dac;
	struct resource *res;
	dma_addr_t dac0_phys_addr;
	int ret;

	dac = devm_kzalloc(dev, sizeof(*dac), GFP_KERNEL);
	if (!dac)
		return -ENOMEM;

	dac->dev = dev;
	dac->volume_dac0 = DAC_VOLUME_MAX;  /* Max volume by default */
	dac->volume_dac1 = DAC_VOLUME_MAX;
	dac->dual_mono = false;  /* Disabled by default (REQ 05050) */

	/* Get memory resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dac->dac0_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(dac->dac0_base))
		return PTR_ERR(dac->dac0_base);

	/* Save DAC0 physical address for DMA */
	dac0_phys_addr = res->start;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	dac->dac1_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(dac->dac1_base))
		return PTR_ERR(dac->dac1_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	dac->dtim_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(dac->dtim_base))
		return PTR_ERR(dac->dtim_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	dac->ccm_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(dac->ccm_base))
		return PTR_ERR(dac->ccm_base);

	/* CCM routing moved to mcf54418_dac_enable() */

	/* Get bus clock */
	dac->bus_clk = devm_clk_get(dev, "bus");
	if (IS_ERR(dac->bus_clk)) {
		dev_err(dev, "Failed to get bus clock\n");
		return PTR_ERR(dac->bus_clk);
	}

	ret = clk_prepare_enable(dac->bus_clk);
	if (ret) {
		dev_err(dev, "Failed to enable bus clock\n");
		return ret;
	}

	dac->bus_clk_rate = clk_get_rate(dac->bus_clk);
	dev_info(dev, "Bus clock rate: %u Hz\n", dac->bus_clk_rate);

	/* Enable DAC analog outputs */
	mcf54418_dac_enable_analog_outputs(dac);

	/* Setup GPIO pins based on board type */
	if (strcmp(board_type, "dlcmi20") == 0) {
		/* DLCMI20: Audio on PG2/PH4 */
		dac->gpio_mute = MCF_GPIO_AUDIO_MUTE_DLCMI20;
		dac->gpio_shutdown = MCF_GPIO_AUDIO_SHUTDOWN_DLCMI20;
		dev_info(dev, "Board: DLCMI20 (MUTE=GPIO%d, SHUTDOWN=GPIO%d)\n",
			 dac->gpio_mute, dac->gpio_shutdown);
	} else {
		/* DLCNext (default): Audio on PB0/PC7, mux from CAN1 to GPIO */
		void __iomem *par_cani2c_addr = (void __iomem *)MCFGPIO_PAR_CANI2C;
		u8 par_cani2c;

		par_cani2c = __raw_readb(par_cani2c_addr);
		par_cani2c &= MCF_GPIO_PAR_CANI2C_CAN1TX_MASK;
		par_cani2c &= MCF_GPIO_PAR_CANI2C_CAN1RX_MASK;
		par_cani2c |= MCF_GPIO_PAR_CANI2C_CAN1TX_GPIO;
		par_cani2c |= MCF_GPIO_PAR_CANI2C_CAN1RX_GPIO;
		__raw_writeb(par_cani2c, par_cani2c_addr);

		dac->gpio_mute = MCF_GPIO_AUDIO_MUTE_DLCNEXT;
		dac->gpio_shutdown = MCF_GPIO_AUDIO_SHUTDOWN_DLCNEXT;
		dev_info(dev, "Board: DLCNext (MUTE=GPIO%d, SHUTDOWN=GPIO%d)\n",
			 dac->gpio_mute, dac->gpio_shutdown);
	}

	dac->mute_inverted = false;  /* GPIO HIGH = unmuted */
	dac->shutdown_inverted = false;  /* GPIO HIGH = powered on */

	ret = gpio_request(dac->gpio_mute, "audio-mute");
	if (ret) {
		dev_err(dev, "Failed to request audio mute GPIO %d: %d\n",
			dac->gpio_mute, ret);
		goto err_clk;
	}

	ret = gpio_request(dac->gpio_shutdown, "audio-shutdown");
	if (ret) {
		dev_err(dev, "Failed to request audio shutdown GPIO %d: %d\n",
			dac->gpio_shutdown, ret);
		goto err_gpio_mute;
	}

	gpio_direction_output(dac->gpio_mute, 1);	/* HIGH = unmuted */
	gpio_direction_output(dac->gpio_shutdown, 1);	/* HIGH = powered on */

	/* Setup DMA parameters for compat mode (non-DT platform) */
	dac->dma_params_tx.addr = dac0_phys_addr + DAC_DATA;
	dac->dma_params_tx.maxburst = 1;
	dac->dma_params_tx.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	dac->dma_params_tx.filter_data = (void *)(uintptr_t)62;

	platform_set_drvdata(pdev, dac);

	/* Register component with DAI and custom PCM ops */
	ret = devm_snd_soc_register_component(dev, &mcf54418_dac_component,
					      &mcf54418_dac_dai, 1);
	if (ret) {
		dev_err(dev, "Failed to register component: %d\n", ret);
		goto err_gpio_shutdown;
	}

	mcf54418_dac_debugfs_init(dac);

	dev_info(dev, "MCF54418 DAC driver registered\n");
	return 0;

err_gpio_shutdown:
	gpio_free(dac->gpio_shutdown);
err_gpio_mute:
	gpio_free(dac->gpio_mute);
err_clk:
	clk_disable_unprepare(dac->bus_clk);
	return ret;
}

static void mcf54418_dac_remove(struct platform_device *pdev)
{
	struct mcf54418_dac *dac = platform_get_drvdata(pdev);

	mcf54418_dac_debugfs_exit(dac);
	mcf54418_dac_enable(dac, false);

	gpio_set_value(dac->gpio_mute, 1);	/* HIGH = Mute */
	gpio_set_value(dac->gpio_shutdown, 1);	/* HIGH = Shutdown */
	gpio_free(dac->gpio_shutdown);
	gpio_free(dac->gpio_mute);

	clk_disable_unprepare(dac->bus_clk);
}

static const struct of_device_id mcf54418_dac_dt_ids[] = {
	{ .compatible = "fsl,mcf54418-dac", },
	{ }
};
MODULE_DEVICE_TABLE(of, mcf54418_dac_dt_ids);

static struct platform_driver mcf54418_dac_driver = {
	.probe		= mcf54418_dac_probe,
	.remove		= mcf54418_dac_remove,
	.driver		= {
		.name	= "mcfdac",
		.of_match_table = mcf54418_dac_dt_ids,
	},
};
module_platform_driver(mcf54418_dac_driver);

MODULE_AUTHOR("Wabtec Corporation");
MODULE_DESCRIPTION("Freescale MCF54418 DAC Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mcfdac.0");
