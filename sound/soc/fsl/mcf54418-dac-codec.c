// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MCF54418 DAC CODEC Driver
 *
 * Copyright (C) 2025 Wabtec Corporation
 *
 * This is a dummy codec driver for the MCF54418 internal DAC.
 * The DAC hardware is controlled directly by the CPU DAI driver.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/soc.h>

static const struct snd_soc_dapm_widget mcf54418_dac_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("VOUTL"),
	SND_SOC_DAPM_OUTPUT("VOUTR"),
	SND_SOC_DAPM_DAC("DAC", "Playback", SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route mcf54418_dac_dapm_routes[] = {
	{ "VOUTL", NULL, "DAC" },
	{ "VOUTR", NULL, "DAC" },
};

static const struct snd_soc_component_driver soc_codec_mcf54418_dac = {
	.dapm_widgets		= mcf54418_dac_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(mcf54418_dac_dapm_widgets),
	.dapm_routes		= mcf54418_dac_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(mcf54418_dac_dapm_routes),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static struct snd_soc_dai_driver mcf54418_dac_dai = {
	.name = "mcf54418-dac-hifi",
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_8000_48000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S16_BE |
				  SNDRV_PCM_FMTBIT_U16_BE,
	},
};

static int mcf54418_dac_codec_probe(struct platform_device *pdev)
{
	return devm_snd_soc_register_component(&pdev->dev,
			&soc_codec_mcf54418_dac,
			&mcf54418_dac_dai, 1);
}

static struct platform_driver mcf54418_dac_codec_driver = {
	.probe		= mcf54418_dac_codec_probe,
	.driver		= {
		.name	= "mcf54418-dac-codec",
	},
};
module_platform_driver(mcf54418_dac_codec_driver);

MODULE_AUTHOR("Wabtec Corporation");
MODULE_DESCRIPTION("MCF54418 DAC Codec Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mcf54418-dac-codec");
