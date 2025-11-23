// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MCF54418 DAC Codec Driver
 *
 * Copyright (C) 2025 Wabtec Corporation
 *
 * This is a virtual codec driver for the MCF54418 internal 12-bit DACs.
 * The actual hardware control is done by the CPU DAI driver.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/soc.h>

static struct snd_soc_dai_driver mcf54418_dac_dai = {
	.name = "mcf54418-dac-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S16_BE |
			   SNDRV_PCM_FMTBIT_U16_LE |
			   SNDRV_PCM_FMTBIT_U16_BE |
			   SNDRV_PCM_FMTBIT_S8 |
			   SNDRV_PCM_FMTBIT_U8,
	},
};

static const struct snd_soc_component_driver soc_component_dev_mcf54418_dac = {
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static int mcf54418_dac_probe(struct platform_device *pdev)
{
	return devm_snd_soc_register_component(&pdev->dev,
			&soc_component_dev_mcf54418_dac,
			&mcf54418_dac_dai, 1);
}

static const struct of_device_id mcf54418_dac_dt_ids[] = {
	{ .compatible = "fsl,mcf54418-dac-codec", },
	{ }
};
MODULE_DEVICE_TABLE(of, mcf54418_dac_dt_ids);

static struct platform_driver mcf54418_dac_codec_driver = {
	.probe		= mcf54418_dac_probe,
	.driver		= {
		.name	= "mcf54418-dac-codec",
		.of_match_table = mcf54418_dac_dt_ids,
	},
};

module_platform_driver(mcf54418_dac_codec_driver);

MODULE_AUTHOR("Wabtec Corporation");
MODULE_DESCRIPTION("MCF54418 DAC Codec Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mcf54418-dac-codec");
