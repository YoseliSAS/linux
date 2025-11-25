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
#define DTIM_DTMR_PS_SHIFT	8 #define DTIM_DTXMR_DMAEN	BIT(7)	/* DMA request enable (vs interrupt) */

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

/* GPIO pins for audio control (Port B pin 0, Port C pin 7) */
#define MCF_GPIO_AUDIO_MUTE	8	/* PB0: Active LOW to unmute (HIGH = muted!) */
#define MCF_GPIO_AUDIO_SHUTDOWN	23	/* PC7: Active LOW to power on (HIGH = shutdown!) */
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
	unsigned int volume;		/* 0-8 range */

	/* GPIO control */
	int gpio_mute;
	int gpio_shutdown;
	bool mute_inverted;		/* If true, LOW = unmuted */
	bool shutdown_inverted;		/* If true, LOW = powered on */

	/* State */
	bool enabled;
	bool dupstream;			/* Stereo mode: DAC0 drives both */
};


struct mcf54418_dac_pcm_runtime {
	struct dma_chan *dma_chan_left;		/* DMA channel for DAC0 (left) */
	struct dma_chan *dma_chan_right;	/* DMA channel for DAC1 (right) - stereo only */
	struct dma_slave_config slave_config_left;	/* DMA config for left */
	struct dma_slave_config slave_config_right;	/* DMA config for right */

	/* Period tracking - double-buffering to avoid gaps */
	unsigned int current_period;		/* Period that just completed (for ALSA) */
	unsigned int next_submit_period;	/* Next period to submit to DMA queue */
	unsigned int total_periods;		/* Total number of periods */
	size_t period_bytes;			/* Size of each period in bytes */
	dma_addr_t dma_addr;			/* DMA buffer physical address */
	unsigned int channels;			/* 1=mono, 2=stereo */

	/* DMA descriptor tracking */
	struct dma_async_tx_descriptor *desc_left;	/* Left channel DMA descriptor */
	struct dma_async_tx_descriptor *desc_right;	/* Right channel DMA descriptor (stereo) */
	dma_cookie_t cookie_left;		/* Left DMA cookie */
	dma_cookie_t cookie_right;		/* Right DMA cookie (stereo) */

	/* State */
	bool running;				/* DMA is running */
	spinlock_t lock;			/* Protect state */
	atomic_t dma_complete_count;		/* Track DMA completions for stereo (0-2) */
};

static const struct snd_pcm_hardware mcf54418_dac_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S16_BE |
				  SNDRV_PCM_FMTBIT_U16_LE |
				  SNDRV_PCM_FMTBIT_U16_BE |
				  SNDRV_PCM_FMTBIT_S8 |
				  SNDRV_PCM_FMTBIT_U8,
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
}static void mcf54418_dac_setup_ccm(struct mcf54418_dac *dac);

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
	u16 cr, cr_readback;

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
			cr &= ~(DAC_CR_PDN | DAC_CR_AUTO | DAC_CR_FORMAT | DAC_CR_WMLVL_MASK);
			cr |= DAC_CR_DMAEN | DAC_CR_SYNC_EN | (0x02 << DAC_CR_WMLVL_SHIFT);  /* WMLVL=2 */
			/* DON'T prime FIFO - let DMA fill it via WMLVL=2 */
		} else {
			cr |= DAC_CR_PDN;
			cr &= ~(DAC_CR_DMAEN | DAC_CR_SYNC_EN);
		}
		dac_writel(dac, i, DAC_CR, cr);
		cr_readback = dac_readl(dac, i, DAC_CR);
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
	return (chan->chan_id == requested_chan);
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

	return 0;

err_free_prtd:
	kfree(prtd);
	return ret;
}

static int mcf54418_dac_pcm_close(struct snd_soc_component *component,
				  struct snd_pcm_substream *substream)
{
	struct mcf54418_dac_pcm_runtime *prtd = substream->runtime->private_data;
	if (prtd) {
		/* Release left channel (always allocated) */
		if (prtd->dma_chan_left) {
			dmaengine_terminate_sync(prtd->dma_chan_left);
			dma_release_channel(prtd->dma_chan_left);
		}
		/* Release right channel (only if stereo) */
		if (prtd->dma_chan_right) {
			dmaengine_terminate_sync(prtd->dma_chan_right);
			dma_release_channel(prtd->dma_chan_right);
		}
		kfree(prtd);
	}

	return 0;
}

static int mcf54418_dac_pcm_hw_params(struct snd_soc_component *component,
				      struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct mcf54418_dac_pcm_runtime *prtd = substream->runtime->private_data;
	size_t period_bytes = params_period_bytes(params);
	unsigned int periods = params_periods(params);
	unsigned int channels = params_channels(params);
	int ret;

	/* Save channel count */
	prtd->channels = channels;

	/* Allocate right channel DMA if stereo */
	if (channels == 2 && !prtd->dma_chan_right) {
		struct dma_chan *chan;

		chan = snd_dmaengine_pcm_request_channel(mcf54418_dac_dma_filter,
							  (void *)(uintptr_t)MCF_EDMA_CHAN_DAC1);
		if (IS_ERR(chan)) {
			dev_err(component->dev, "Failed to request DMA channel %d (DAC1): %ld\n",
				MCF_EDMA_CHAN_DAC1, PTR_ERR(chan));
			return PTR_ERR(chan);
		}
		prtd->dma_chan_right = chan;
		dev_info(component->dev, "Stereo mode: allocated DMA channel %d for DAC1\n",
			 MCF_EDMA_CHAN_DAC1);
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

	/* Configure right channel DMA (DAC1) if stereo */
	if (channels == 2) {
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

	/* Save period info */
	prtd->period_bytes = period_bytes;
	prtd->total_periods = periods;
	prtd->current_period = 0;
	prtd->dma_addr = substream->runtime->dma_addr;

	dev_info(component->dev, "PCM configured: %d channels, period=%zu bytes, periods=%u\n",
		 channels, period_bytes, periods);

	return 0;
}

/* Forward declarations */
static void mcf54418_dac_dma_complete(void *data);

/*
 * Submit interleaved DMA transfer for stereo playback.
 *
 * Stereo audio data is interleaved as [L0][R0][L1][R1]... where each sample
 * is 2 bytes. To send left samples to DAC0 and right samples to DAC1, we use
 * the interleaved DMA API with source inter-chunk gaps (ICG):
 *
 * Left channel:  src_start=period_addr,   src_icg=2 (skip right sample)
 * Right channel: src_start=period_addr+2, src_icg=2 (skip left sample)
 *
 * This results in TCD SOFF=4 (nbytes + src_icg = 2 + 2), which steps through
 * the interleaved buffer correctly.
 */
static int mcf54418_dac_submit_interleaved(struct snd_pcm_substream *substream,
					   struct dma_chan *chan,
					   dma_addr_t src_addr, dma_addr_t dst_addr,
					   unsigned int num_samples)
{
	struct mcf54418_dac_pcm_runtime *prtd = substream->runtime->private_data;
	struct dma_interleaved_template *xt;
	struct dma_async_tx_descriptor *desc;
	unsigned long flags;
	dma_cookie_t cookie;


	/* Allocate interleaved template with 1 chunk.
	 * Use GFP_ATOMIC because this is called from trigger and DMA completion
	 * callbacks which may be in atomic/interrupt context.
	 */
	xt = kzalloc(sizeof(*xt) + sizeof(struct data_chunk), GFP_ATOMIC);
	if (!xt)
		return -ENOMEM;

	/* Configure interleaved transfer */
	xt->src_start = src_addr;
	xt->dst_start = dst_addr;
	xt->dir = DMA_MEM_TO_DEV;
	xt->src_inc = true;		/* Increment source address */
	xt->dst_inc = false;		/* Fixed device address */
	xt->src_sgl = true;		/* Source is scattered (has gaps) */
	xt->dst_sgl = false;		/* Destination is contiguous (device) */
	xt->numf = num_samples;		/* Number of frames (samples) */
	xt->frame_size = 1;		/* One chunk per frame */

	/* Configure chunk: 2 bytes per sample, 2 byte gap (other channel) */
	xt->sgl[0].size = 2;		/* 16-bit sample */
	xt->sgl[0].icg = 0;		/* No general ICG */
	xt->sgl[0].src_icg = 2;		/* Skip 2 bytes (other channel's sample) */
	xt->sgl[0].dst_icg = 0;		/* No destination gap (fixed address) */

	/* Prepare interleaved DMA descriptor */
	desc = dmaengine_prep_interleaved_dma(chan, xt,
					      DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	kfree(xt);  /* Template can be freed after prep */

	if (!desc) {
		dev_err(substream->pcm->card->dev,
			"Failed to prep interleaved DMA (chan=%d)\n",
			chan->chan_id);
		return -ENOMEM;
	}


	/* Set completion callback */
	desc->callback = mcf54418_dac_dma_complete;
	desc->callback_param = substream;

	/* Submit descriptor */
	spin_lock_irqsave(&prtd->lock, flags);
	cookie = dmaengine_submit(desc);
	spin_unlock_irqrestore(&prtd->lock, flags);

	if (dma_submit_error(cookie)) {
		dev_err(substream->pcm->card->dev,
			"Failed to submit interleaved DMA: %d\n", cookie);
		return -EIO;
	}

	return 0;
}

/* DMA Mode 0: Single period submission
 * Uses next_submit_period to track which period to submit next.
 * This enables double-buffering: we can have multiple periods queued
 * so there's no gap when one completes.
 */
static int mcf54418_dac_submit_single(struct snd_pcm_substream *substream)
{
	struct mcf54418_dac_pcm_runtime *prtd = substream->runtime->private_data;
	struct dma_async_tx_descriptor *desc;
	dma_addr_t period_addr;
	unsigned long flags;
	int ret;
	unsigned int submit_period = prtd->next_submit_period;

	/* Calculate address of period to submit */
	period_addr = prtd->dma_addr + (submit_period * prtd->period_bytes);

	/* Advance next_submit_period for next call */
	prtd->next_submit_period = (submit_period + 1) % prtd->total_periods;

	if (prtd->channels == 2) {
		/*
		 * Stereo mode: Use interleaved DMA API.
		 * Buffer contains interleaved [L0][R0][L1][R1]... samples.
		 * Each sample is 2 bytes (16-bit).
		 * Number of samples per channel = period_bytes / 4 (2 bytes * 2 channels)
		 */
		unsigned int samples_per_channel = prtd->period_bytes / 4;

		/* Submit left channel: starts at period_addr */
		ret = mcf54418_dac_submit_interleaved(substream,
						      prtd->dma_chan_left,
						      period_addr,
						      0xFC098002,  /* DAC0 */
						      samples_per_channel);
		if (ret)
			return ret;

		/* Submit right channel: starts at period_addr + 2 (first right sample) */
		ret = mcf54418_dac_submit_interleaved(substream,
						      prtd->dma_chan_right,
						      period_addr + 2,
						      0xFC09C002,  /* DAC1 */
						      samples_per_channel);
		if (ret)
			return ret;

		/* Start both DMA engines */
		dma_async_issue_pending(prtd->dma_chan_left);
		dma_async_issue_pending(prtd->dma_chan_right);
	} else {
		/*
		 * Mono mode: Use simple slave_single transfer.
		 * All samples go to DAC0.
		 */
		desc = dmaengine_prep_slave_single(prtd->dma_chan_left, period_addr,
						   prtd->period_bytes, DMA_MEM_TO_DEV,
						   DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (!desc) {
			dev_err(substream->pcm->card->dev,
				"Failed to prep mono DMA for period %u\n",
				prtd->current_period);
			return -ENOMEM;
		}

		desc->callback = mcf54418_dac_dma_complete;
		desc->callback_param = substream;

		spin_lock_irqsave(&prtd->lock, flags);
		prtd->desc_left = desc;
		prtd->cookie_left = dmaengine_submit(desc);
		spin_unlock_irqrestore(&prtd->lock, flags);

		if (dma_submit_error(prtd->cookie_left)) {
			dev_err(substream->pcm->card->dev,
				"Failed to submit mono DMA: %d\n", prtd->cookie_left);
			return -EIO;
		}

		dma_async_issue_pending(prtd->dma_chan_left);
	}

	return 0;
}

/* PCM trigger callback */
static int mcf54418_dac_pcm_trigger(struct snd_soc_component *component,
				    struct snd_pcm_substream *substream, int cmd)
{
	struct mcf54418_dac_pcm_runtime *prtd = substream->runtime->private_data;
	unsigned long flags;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		spin_lock_irqsave(&prtd->lock, flags);
		prtd->current_period = 0;
		prtd->next_submit_period = 0;  /* Start submitting from period 0 */
		prtd->running = true;
		atomic_set(&prtd->dma_complete_count, 0);  /* Reset completion counter */
		spin_unlock_irqrestore(&prtd->lock, flags);

		/* NOTE: With trigger_start = SND_SOC_TRIGGER_ORDER_LDC, ASoC calls
		 * DAI trigger (which starts DTIM3) BEFORE this component trigger.
		 * So DTIM3 is already running when we get here.
		 */

		/* Double-buffering: submit 2 periods upfront to avoid gaps.
		 * When period 0 completes, period 1 starts immediately.
		 * The completion callback then submits period 2, etc.
		 */
		ret = mcf54418_dac_submit_single(substream);  /* Submit period 0 */
		if (ret) {
			prtd->running = false;
			return ret;
		}
		ret = mcf54418_dac_submit_single(substream);  /* Submit period 1 */
		if (ret)
			prtd->running = false;
		return ret;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		spin_lock_irqsave(&prtd->lock, flags);
		prtd->running = false;
		spin_unlock_irqrestore(&prtd->lock, flags);

		/* Stop DMA - use sync to ensure clean state before restart */
		dmaengine_terminate_sync(prtd->dma_chan_left);
		if (prtd->dma_chan_right) {
			dmaengine_terminate_sync(prtd->dma_chan_right);
		}

		/* NOTE: With trigger_start = SND_SOC_TRIGGER_ORDER_LDC, ASoC
		 * handles DAI trigger STOP after this component trigger.
		 */

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



static void mcf54418_dac_dma_complete(void *data)
{
	struct snd_pcm_substream *substream = data;
	struct mcf54418_dac_pcm_runtime *prtd = substream->runtime->private_data;
	unsigned long flags;
	int ret;
	int completed;

	/* For stereo, both channels must complete before proceeding */
	completed = atomic_inc_return(&prtd->dma_complete_count);

	/* For mono: completed==1, proceed
	 * For stereo: only proceed when completed==2 (both channels done)
	 */
	if (completed < prtd->channels) {
		return;  /* Wait for other channel */
	}

	/* Reset counter for next period */
	atomic_set(&prtd->dma_complete_count, 0);

	spin_lock_irqsave(&prtd->lock, flags);

	if (!prtd->running) {
		spin_unlock_irqrestore(&prtd->lock, flags);
		return;
	}

	/* Advance to next period */
	prtd->current_period++;
	if (prtd->current_period >= prtd->total_periods) {
		prtd->current_period = 0;
	}

	spin_unlock_irqrestore(&prtd->lock, flags);

	/* Notify ALSA that period elapsed */
	snd_pcm_period_elapsed(substream);

	/* Submit next period */
	ret = mcf54418_dac_submit_single(substream);
	if (ret) {
		dev_err(substream->pcm->card->dev,
			"Failed to submit next period: %d\n", ret);
		prtd->running = false;
	}
}

/* SG mode completion callback - called after 4 periods complete */


static int mcf54418_dac_pcm_construct(struct snd_soc_component *component,
				      struct snd_soc_pcm_runtime *rtd)
{
	struct snd_pcm *pcm = rtd->pcm;
	size_t size = mcf54418_dac_pcm_hardware.buffer_bytes_max;
	int ret;
	ret = snd_pcm_set_managed_buffer_all(pcm, SNDRV_DMA_TYPE_DEV,
					      component->dev, size, size);

	
	if (ret == 0 && pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		struct snd_pcm_substream *substream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
		if (substream->dma_buffer.addr) {		}
	}

	return ret;
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
	dac->dupstream = (channels == 2);

	/* Configure DAC control register(s) - initialize both DAC0 and DAC1 for stereo */
	for (i = 0; i < channels; i++) {
		cr = DAC_CR_WMLVL_2;  /* Watermark at 2 words */

		/* Set data format */
		switch (format) {
		case SNDRV_PCM_FORMAT_S16_LE:
		case SNDRV_PCM_FORMAT_S16_BE:
		case SNDRV_PCM_FORMAT_U16_LE:
		case SNDRV_PCM_FORMAT_U16_BE:
			cr |= DAC_CR_FORMAT;  /* Left-justified for 16-bit */
			break;
		default:
			cr &= ~DAC_CR_FORMAT; /* Right-justified for 8-bit */
			break;
		}

		dac_writel(dac, i, DAC_CR, cr);
		dev_info(dac->dev, "Initialized DAC%d: CR=0x%04x\n", i, cr);
	} unsigned int timer_rate = rate * channels;
	mcf54418_dac_setup_timer(dac, timer_rate);

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
				  SNDRV_PCM_FMTBIT_U16_LE |
				  SNDRV_PCM_FMTBIT_U16_BE |
				  SNDRV_PCM_FMTBIT_S8 |
				  SNDRV_PCM_FMTBIT_U8,
	},
	.ops = &mcf54418_dac_dai_ops,
};

/* Volume control */
static int mcf54418_dac_volume_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 8;
	return 0;
}

static int mcf54418_dac_volume_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct mcf54418_dac *dac = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = dac->volume;
	return 0;
}

static int mcf54418_dac_volume_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct mcf54418_dac *dac = snd_soc_component_get_drvdata(component);
	unsigned int vol = ucontrol->value.integer.value[0];

	if (vol > 8)
		return -EINVAL;

	if (dac->volume == vol)
		return 0;

	dac->volume = vol;
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
		.name = "Playback Volume",
		.info = mcf54418_dac_volume_info,
		.get = mcf54418_dac_volume_get,
		.put = mcf54418_dac_volume_put,
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

static int mcf54418_dac_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mcf54418_dac *dac;
	struct resource *res;
	int ret;

	dac = devm_kzalloc(dev, sizeof(*dac), GFP_KERNEL);
	if (!dac)
		return -ENOMEM;

	dac->dev = dev;
	dac->volume = 8;  /* Max volume by default */

	/* Get memory resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dac->dac0_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(dac->dac0_base))
		return PTR_ERR(dac->dac0_base);

	/* Save DAC0 physical address for DMA */
	dma_addr_t dac0_phys_addr = res->start;

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

	{
		void __iomem *par_cani2c_addr = (void __iomem *)MCFGPIO_PAR_CANI2C;
		u8 par_cani2c;

		par_cani2c = __raw_readb(par_cani2c_addr);

		/* Clear CAN1TX[3:2] and CAN1RX[1:0] fields to select GPIO mode */
		par_cani2c &= MCF_GPIO_PAR_CANI2C_CAN1TX_MASK;  /* Clear bits[3:2] */
		par_cani2c &= MCF_GPIO_PAR_CANI2C_CAN1RX_MASK;  /* Clear bits[1:0] */
		par_cani2c |= MCF_GPIO_PAR_CANI2C_CAN1TX_GPIO;  /* Set PB0 = GPIO (0x00) */
		par_cani2c |= MCF_GPIO_PAR_CANI2C_CAN1RX_GPIO;  /* Set PC7 = GPIO (0x00) */

		__raw_writeb(par_cani2c, par_cani2c_addr);
	}

	/* Setup GPIO for audio control */
	dac->gpio_mute = MCF_GPIO_AUDIO_MUTE;
	dac->gpio_shutdown = MCF_GPIO_AUDIO_SHUTDOWN;
	dac->mute_inverted = true;  /* GPIO LOW (0V) = unmuted, so invert control logic */
	dac->shutdown_inverted = true;  /* GPIO LOW (0V) = powered on, so invert control logic */

	ret = gpio_request(dac->gpio_mute, "audio-mute");
	if (ret) {
		dev_err(dev, "Failed to request audio mute GPIO: %d\n", ret);
		goto err_clk;
	}

	ret = gpio_request(dac->gpio_shutdown, "audio-shutdown");
	if (ret) {
		dev_err(dev, "Failed to request audio shutdown GPIO: %d\n", ret);
		goto err_gpio_mute;
	}

	gpio_direction_output(dac->gpio_mute, 0);	/* LOW (0V) = unmuted */
	gpio_direction_output(dac->gpio_shutdown, 0);	/* LOW (0V) = powered on */

	/* Setup DMA parameters for compat mode (non-DT platform) */
	dac->dma_params_tx.addr = dac0_phys_addr + DAC_DATA;
	dac->dma_params_tx.maxburst = 1;  /* 1 word per burst (16-bit) */
	dac->dma_params_tx.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	dac->dma_params_tx.filter_data = (void *)(uintptr_t)62;  /* DMA channel 62 for DAC0 */

	platform_set_drvdata(pdev, dac);

	/* Register component with DAI and custom PCM ops */
	ret = devm_snd_soc_register_component(dev, &mcf54418_dac_component,
					      &mcf54418_dac_dai, 1);
	if (ret) {
		dev_err(dev, "Failed to register component: %d\n", ret);
		goto err_gpio_shutdown;
	}

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

	mcf54418_dac_enable(dac, false);

	/* Disable audio amplifier */
	gpio_set_value(dac->gpio_mute, 0);	/* LOW = Mute */
	gpio_set_value(dac->gpio_shutdown, 0);	/* LOW = Shutdown */

	/* DMA channels are managed by dmaengine PCM layer */

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
