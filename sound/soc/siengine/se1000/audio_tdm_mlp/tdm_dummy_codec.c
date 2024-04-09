/*
 * SPDX-License-Identifier: GPL-2.0.
 * sound/soc/siengine/audio_tdm_mlp
 * Copyright (C) 2022-2023 SiEngine or its affiliates
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/soc.h>

static const struct snd_soc_dapm_widget dummy_codec_widgets[] = {
	SND_SOC_DAPM_INPUT("RX"),
	SND_SOC_DAPM_OUTPUT("TX"),
};

static const struct snd_soc_dapm_route dummy_codec_routes[] = {
	{"tdm-dummy-Capture", NULL, "RX"},
	{"TX", NULL, "tdm-dummy-Playback"},
};

static struct snd_soc_dai_driver tdm_dummy_dai_driver[] = {
	{
		.name = "tdm-dummy-codec-dai",
		.playback = {
			.stream_name = "tdm-dummy-Playback",
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE),
		},
		.capture = {
			.stream_name = "tdm-dummy-Capture",
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE),
		},
	},
};

static const struct snd_soc_component_driver tdm_dummy_deriver = {
	.dapm_widgets		= dummy_codec_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(dummy_codec_widgets),
	.dapm_routes		= dummy_codec_routes,
	.num_dapm_routes	= ARRAY_SIZE(dummy_codec_routes),
};

static int dummy_codec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

#ifdef CONFIG_OF
	if (dev->of_node) {
		dev_set_name(dev, "%s", "tdm-dummy-codec");
		pr_debug("%s set dev name %s\n", __func__, dev_name(dev));
	}
#endif

	return devm_snd_soc_register_component(&pdev->dev, &tdm_dummy_deriver,
			tdm_dummy_dai_driver, ARRAY_SIZE(tdm_dummy_dai_driver));
}

static int dummy_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id dummy_codec_of_match[] = {
	{ .compatible = "siengine,tdm-dummy-codec", },
	{},
};
MODULE_DEVICE_TABLE(of, dummy_codec_of_match);
#endif

static struct platform_driver dummy_codec_driver = {
	.driver = {
		.name = "tdm-dummy-codec",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = dummy_codec_of_match,
#endif
	},
	.probe = dummy_codec_probe,
	.remove = dummy_codec_remove,
};

module_platform_driver(dummy_codec_driver);

MODULE_AUTHOR("jian.xu <jian.xu@siengine>");
MODULE_DESCRIPTION("ASoC dummy codec sco link driver");
MODULE_LICENSE("GPL v2");
