/*
 * SPDX-License-Identifier: GPL-2.0.
 * sound/soc/siengine/audioshare
 * Copyright (C) 2021-2022 SiEngine or its affiliates
*/

#include <linux/module.h>
#include <sound/soc.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <sound/soc.h>
#include <sound/pcm.h>

SND_SOC_DAILINK_DEFS(tdm0,
	DAILINK_COMP_ARRAY(COMP_CPU("tdm0")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm1,
	DAILINK_COMP_ARRAY(COMP_CPU("tdm1")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm2,
	DAILINK_COMP_ARRAY(COMP_CPU("tdm2")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm3,
	DAILINK_COMP_ARRAY(COMP_CPU("tdm3")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm4,
	DAILINK_COMP_ARRAY(COMP_CPU("tdm4")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm5,
	DAILINK_COMP_ARRAY(COMP_CPU("tdm5")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm6,
	DAILINK_COMP_ARRAY(COMP_CPU("tdm6")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm7,
	DAILINK_COMP_ARRAY(COMP_CPU("tdm7")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm8,
	DAILINK_COMP_ARRAY(COMP_CPU("tdm8")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm9,
	DAILINK_COMP_ARRAY(COMP_CPU("tdm9")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm10,
	DAILINK_COMP_ARRAY(COMP_CPU("tdm10")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link se1000_dummy_links[] = {
	{
		.name = "se1000-dummy0",
		.stream_name = "se1000-dummy0",
		SND_SOC_DAILINK_REG(tdm0),
	},
	{
		.name = "se1000-dummy1",
		.stream_name = "se1000-dummy1",
		SND_SOC_DAILINK_REG(tdm1),
	},
	{
		.name = "se1000-dummy2",
		.stream_name = "se1000-dummy2",
		SND_SOC_DAILINK_REG(tdm2),
	},
	{
		.name = "se1000-dummy3",
		.stream_name = "se1000-dummy3",
		SND_SOC_DAILINK_REG(tdm3),
	},
	{
		.name = "se1000-dummy4",
		.stream_name = "se1000-dummy4",
		SND_SOC_DAILINK_REG(tdm4),
	},
	{
		.name = "se1000-dummy5",
		.stream_name = "se1000-dummy5",
		SND_SOC_DAILINK_REG(tdm5),
	},
	{
		.name = "se1000-dummy6",
		.stream_name = "se1000-dummy6",
		SND_SOC_DAILINK_REG(tdm6),
	},
	{
		.name = "se1000-dummy7",
		.stream_name = "se1000-dummy7",
		SND_SOC_DAILINK_REG(tdm7),
	},
	{
		.name = "se1000-dummy8",
		.stream_name = "se1000-dummy8",
		SND_SOC_DAILINK_REG(tdm8),
	},
	{
		.name = "se1000-dummy9",
		.stream_name = "se1000-dummy9",
		SND_SOC_DAILINK_REG(tdm9),
	},
	{
		.name = "se1000-dummy10",
		.stream_name = "se1000-dummy10",
		SND_SOC_DAILINK_REG(tdm10),
	},
};

static struct snd_soc_card se1000_soc_card = {
	.name = "se1000-dummy-card",
	.owner = THIS_MODULE,
	.dai_link = se1000_dummy_links,
	.num_links = ARRAY_SIZE(se1000_dummy_links),
};

static int se1000_dummy_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &se1000_soc_card;
	struct device_node *platform_node = NULL;
	struct device *dev = &pdev->dev;
	int i = 0, ret = 0;

	card->dev = dev;

	for (i = 0; i < card->num_links; i++) {
		platform_node = of_parse_phandle(pdev->dev.of_node, "dummy_plat", i);
		if (!platform_node) {
			dev_err(&pdev->dev, "Property 'platform' missing or invalid\n");
			return -EINVAL;
		}

		se1000_dummy_links[i].cpus->of_node = platform_node;
		se1000_dummy_links[i].platforms->of_node = platform_node;
		se1000_dummy_links[i].codecs->name = "snd-soc-dummy";
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret)
		dev_err(&pdev->dev, "%s snd_soc_register_card failed %d\n",
			__func__, ret);

	return ret;
}

static int se1000_dummy_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

/* match dts */
#ifdef CONFIG_OF
static const struct of_device_id se1000_dummy_dt_match[] = {
	{.compatible = "siengine,se1000-dummy-card",},
	{}
};
#endif

static struct platform_driver se1000_dummy_driver = {
	.probe = se1000_dummy_probe,
	.remove = se1000_dummy_remove,
	.driver = {
		.name = "se1000-dummy-driver",
		#ifdef CONFIG_OF
			.of_match_table = se1000_dummy_dt_match,
		#endif
	},
};

module_platform_driver(se1000_dummy_driver);

/* Module information */
MODULE_DESCRIPTION("se1000 ALSA SoC machine driver");
MODULE_AUTHOR("jian.xu <jian.xu@siengine.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("se1000 dummy card");

