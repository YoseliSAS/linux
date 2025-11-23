// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MCF54418 DAC Audio Machine Driver
 *
 * Copyright (C) 2025 Wabtec Corporation
 *
 * This machine driver connects the MCF54418 DAC CPU DAI with the codec.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/soc.h>

static int mcf54418_dac_audio_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	/* Internal DAC doesn't need DAI format configuration */
	return 0;
}

static const struct snd_soc_ops mcf54418_dac_audio_ops = {
	.hw_params = mcf54418_dac_audio_hw_params,
};

SND_SOC_DAILINK_DEFS(mcf54418_dac,
	DAILINK_COMP_ARRAY(COMP_CPU("mcf54418-dac")),
	DAILINK_COMP_ARRAY(COMP_CODEC("mcf54418-dac-codec", "mcf54418-dac-hifi")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("mcfdac.0")));

static struct snd_soc_dai_link mcf54418_dac_dai_link = {
	.name		= "MCF54418-DAC",
	.stream_name	= "MCF54418 DAC Playback",
	.ops		= &mcf54418_dac_audio_ops,
	SND_SOC_DAILINK_REG(mcf54418_dac),
};

static struct snd_soc_card mcf54418_dac_card = {
	.name		= "mcf5441x-dac",
	.owner		= THIS_MODULE,
	.dai_link	= &mcf54418_dac_dai_link,
	.num_links	= 1,
};

static int mcf54418_dac_audio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_card *card = &mcf54418_dac_card;
	int ret;

	card->dev = dev;

	ret = devm_snd_soc_register_card(dev, card);
	if (ret) {
		dev_err(dev, "Failed to register sound card: %d\n", ret);
		return ret;
	}

	dev_info(dev, "MCF54418 DAC audio card registered\n");
	return 0;
}

static const struct of_device_id mcf54418_dac_audio_dt_ids[] = {
	{ .compatible = "fsl,mcf54418-dac-audio", },
	{ }
};
MODULE_DEVICE_TABLE(of, mcf54418_dac_audio_dt_ids);

static struct platform_driver mcf54418_dac_audio_driver = {
	.probe		= mcf54418_dac_audio_probe,
	.driver		= {
		.name	= "mcf54418-dac-audio",
		.of_match_table = mcf54418_dac_audio_dt_ids,
		.pm = &snd_soc_pm_ops,
	},
};
module_platform_driver(mcf54418_dac_audio_driver);

MODULE_AUTHOR("Wabtec Corporation");
MODULE_DESCRIPTION("MCF54418 DAC Audio Machine Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mcf54418-dac-audio");
