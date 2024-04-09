/*
 * SE1000 ALSA SoC machine driver
 *
 * Copyright (c) 2022 Siengine Inc.
 * Author: jian.xu <jian.xu@Siengine.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <sound/soc.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <sound/soc.h>
#include <sound/pcm.h>

#include "se1000-aud-i2s.h"
#include "./audioshare/audio-share-dummy.h"

static int se1000_aud_i2s_ops_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = *(rtd->dais);
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	int ret;
	unsigned int dai_fmt;
	unsigned int rate = params_rate(params);

	struct se_i2s_base *i2s = dev_get_drvdata(cpu_dai->dev);

	snd_soc_dai_set_sysclk(cpu_dai, cpu_dai->id, rate, SND_SOC_CLOCK_OUT);

	if (!strcmp(codec_dai->name, "ES8316 HiFi") && i2s->mclk_enable)
		snd_soc_dai_set_sysclk(codec_dai, 0, rate*i2s->mclk_rate_coef, SND_SOC_CLOCK_IN);

	/* Set I2S fmt */
	dai_fmt = i2s->i2s_config.i2s_format | i2s->i2s_config.i2s_mode
		| i2s->i2s_config.i2s_clk_polar;

	ret = snd_soc_dai_set_fmt(cpu_dai, dai_fmt);
	if (ret < 0) {
		dev_err(cpu_dai->dev,
			"ASoC: Failed to set DAI format: %d\n", ret);
		return ret;
	}

	return 0;
}


static struct snd_soc_ops se1000_aud_be_ops = {
	.hw_params = se1000_aud_i2s_ops_hw_params
};

/*
SND_SOC_DAILINK_DEFS(i2s0,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S0")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "i2s-dummy-codec-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(i2s1,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S1")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "i2s-dummy-codec-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(i2s2,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S2")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "i2s-dummy-codec-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(i2s3,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S3")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "i2s-dummy-codec-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
*/

SND_SOC_DAILINK_DEFS(i2s4,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S4")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "ES8316 HiFi")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

/*
SND_SOC_DAILINK_DEFS(i2s5,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S5")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "i2s-dummy-codec-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(i2s6,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S6")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "i2s-dummy-codec-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(spdif,
	DAILINK_COMP_ARRAY(COMP_CPU("SPDIF")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "i2s-dummy-codec-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));
*/

/* hostless dummy0 ul&dl <--> I2S0 */
/*
SND_SOC_DAILINK_DEFS(hostless_i2s0,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S0")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "i2s-dummy-codec-dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.0")));
*/
/* hostless dummy1 ul&dl <--> I2S1 */
/*
SND_SOC_DAILINK_DEFS(hostless_i2s1,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S1")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "i2s-dummy-codec-dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.0")));
*/
/* hostless dummy2 ul&dl <--> I2S2 */
/*
SND_SOC_DAILINK_DEFS(hostless_i2s2,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S2")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "i2s-dummy-codec-dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.0")));
*/
/* hostless dummy0 ul&dl <--> share memory 0 */
/*
SND_SOC_DAILINK_DEFS(hostless_share_dummy0,
	DAILINK_COMP_ARRAY(COMP_CPU("share-loop-dummy0")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "i2s-dummy-codec-dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.0")));
*/
/* hostless dummy1 ul&dl <--> share memory 1 */
/*
SND_SOC_DAILINK_DEFS(hostless_share_dummy1,
	DAILINK_COMP_ARRAY(COMP_CPU("share-loop-dummy1")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "i2s-dummy-codec-dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.0")));
*/
/* hostless dummy2 ul&dl <--> share memory 2 */
/*
SND_SOC_DAILINK_DEFS(hostless_share_dummy2,
	DAILINK_COMP_ARRAY(COMP_CPU("share-loop-dummy2")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "i2s-dummy-codec-dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.0")));
*/

static struct snd_soc_dai_link se1000_dai_links[] = {
/*	{
		.name = "se1000-I2S0",
		.stream_name = "se1000-I2S0",
		//.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF,
		.ops = &se1000_aud_be_ops,
		SND_SOC_DAILINK_REG(i2s0),
	},
	{
		.name = "se1000-I2S1",
		.stream_name = "se1000-I2S1",
		//.dai_fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF,
		.ops = &se1000_aud_be_ops,
		SND_SOC_DAILINK_REG(i2s1),
	},
	{
		.name = "se1000-I2S2",
		.stream_name = "se1000-I2S2",
		//.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF,
		.ops = &se1000_aud_be_ops,
		SND_SOC_DAILINK_REG(i2s2),
	},
	{
		.name = "se1000-I2S3",
		.stream_name = "se1000-I2S3",
		//.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF,
		.ops = &se1000_aud_be_ops,
		SND_SOC_DAILINK_REG(i2s3),
	},
*/
	{
		.name = "se1000-I2S4",
		.stream_name = "se1000-I2S4",
		//.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF,
		.ops = &se1000_aud_be_ops,
		SND_SOC_DAILINK_REG(i2s4),
	},
/*	{
		.name = "se1000-I2S5",
		.stream_name = "se1000-I2S5",
		//.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF,
		.ops = &se1000_aud_be_ops,
		SND_SOC_DAILINK_REG(i2s5),
	},
	{
		.name = "se1000-I2S6",
		.stream_name = "se1000-I2S6",
		//.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF,
		.ops = &se1000_aud_be_ops,
		SND_SOC_DAILINK_REG(i2s6),
	},
	{
		.name = "se1000-SPDIF",
		.stream_name = "se1000-SPDIF",
		SND_SOC_DAILINK_REG(spdif),
	},
*/
	/* hostless share */
/*
	{
		.name = "se1000-HL-SHR-I2S0",
		.stream_name = "se1000-HL-SHR-I2S0",
		.id = SE1000_HL_SHR_I2S0,
		.ops = &se1000_aud_be_ops,
		SND_SOC_DAILINK_REG(hostless_i2s0),
	},
	{
		.name = "se1000-HL-SHR-I2S1",
		.stream_name = "se1000-HL-SHR-I2S1",
		.id = SE1000_HL_SHR_I2S1,
		.ops = &se1000_aud_be_ops,
		SND_SOC_DAILINK_REG(hostless_i2s1),
	},
	{
		.name = "se1000-HL-SHR-I2S2",
		.stream_name = "se1000-HL-SHR-I2S2",
		.id = SE1000_HL_SHR_I2S2,
		.ops = &se1000_aud_be_ops,
		SND_SOC_DAILINK_REG(hostless_i2s2),
	},
	{
		.name = "se1000-HL-SHR-DUMMY0",
		.stream_name = "se1000-HL-SHR-DUMMY0",
		.id = SE1000_HL_SHR_DUMMY0,
		SND_SOC_DAILINK_REG(hostless_share_dummy0),
	},
	{
		.name = "se1000-HL-SHR-DUMMY1",
		.stream_name = "se1000-HL-SHR-DUMMY1",
		.id = SE1000_HL_SHR_DUMMY1,
		SND_SOC_DAILINK_REG(hostless_share_dummy1),
	},
	{
		.name = "se1000-HL-SHR-DUMMY2",
		.stream_name = "se1000-HL-SHR-DUMMY2",
		.id = SE1000_HL_SHR_DUMMY2,
		SND_SOC_DAILINK_REG(hostless_share_dummy2),
	},
*/
};

static struct snd_soc_card se1000_soc_card = {
	.name = "se1000-sound-card",
	.owner = THIS_MODULE,
	.dai_link = se1000_dai_links,
	.num_links = ARRAY_SIZE(se1000_dai_links),
};

static int parser_card_dai_link_components_node(struct snd_soc_card *card)
{
	int i, index, ret = 0;
	struct device *cdev = card->dev;
	struct snd_soc_dai_link *dai_link = card->dai_link;
	struct device_node *np, *codec_node;
	const char *codec_dai_name;

	if (!cdev) {
		pr_err("%s: Sound card device invalid!\n", __func__);
		return -ENODEV;
	}

	codec_node = of_parse_phandle(cdev->of_node,
					"siengine,audio-codec-i2s", 0);
	if (!codec_node) {
		dev_err(cdev, "Property 'audio-codec' missing or invalid\n");
		return -EINVAL;
	}

	for (i = 0; i < card->num_links; i++) {
		if (dai_link[i].platforms->of_node && dai_link[i].cpus->of_node)
			continue;
		/* parser platform driver */
		if (dai_link[i].platforms->name &&
		    !dai_link[i].platforms->of_node) {
			index = of_property_match_string(cdev->of_node,
						"se-plat-name", dai_link[i].platforms->name);
			if (index < 0) {
				dev_err(cdev, "%s: No match found for platform name: %s\n", \
					__func__, dai_link[i].platforms->name);
				ret = index;
				goto err;
			}
			np = of_parse_phandle(cdev->of_node, "se-plat", index);
			if (!np) {
				dev_err(cdev, "%s: phandle for platform %s, index %d failed\n", \
					__func__, dai_link[i].platforms->name, index);
				ret = -ENODEV;
				goto err;
			}
			dai_link[i].platforms->of_node = np;
			dai_link[i].platforms->name = NULL;
		}

		/* parser cpu dai driver */
		if (dai_link[i].cpus->dai_name && !dai_link[i].cpus->of_node) {
			index = of_property_match_string(cdev->of_node,
						 "se-cpu-name", dai_link[i].cpus->dai_name);
			if (index >= 0) {
				np = of_parse_phandle(cdev->of_node, "se-cpu", index);
				if (!np) {
					dev_err(cdev, "%s: phandle for cpu dai %s failed\n", __func__, \
								dai_link[i].cpus->dai_name);
					ret = -ENODEV;
					goto err;
				}
				dai_link[i].cpus->of_node = np;
				/* parser same platform */
				if (!dai_link[i].platforms->name && \
						!dai_link[i].platforms->of_node) {
					dai_link[i].platforms->of_node = np;
					dai_link[i].platforms->name = NULL;
				}
			}
		}

		if (!dai_link[i].cpus->of_node) {
			dev_err(cdev, "%s: no cpu dai %s found\n", __func__, \
						dai_link[i].cpus->dai_name);
			ret = -ENODEV;
			goto err;
		}
		dai_link[i].codecs->of_node = codec_node;
	}

	codec_node = of_parse_phandle(cdev->of_node, "siengine,audio-codec-i2s4", 0);
	if (codec_node) {
		if (of_property_read_string(cdev->of_node, "codec_dai_name", &codec_dai_name)) {
			dev_err(cdev, "get codec_dai_name fail in dts\n");
			return -EINVAL;
		}
		dai_link[0].codecs->of_node = codec_node;
		dai_link[0].codecs->dai_name = codec_dai_name;
	}

err:
	return ret;
}

static int se1000_machine_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &se1000_soc_card;
	struct device *dev = &pdev->dev;
	int ret = 0;

	card->dev = dev;

	ret = parser_card_dai_link_components_node(card);
	if (ret == -EPROBE_DEFER)
		return ret;
	else if (ret)
		dev_err(&pdev->dev, "%s dai link populate failed %d\n",
			__func__, ret);

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret == -EPROBE_DEFER)
		return ret;
	else if (ret)
		dev_err(&pdev->dev, "%s snd_soc_register_card failed %d\n",
			__func__, ret);
	return ret;
}

static int se1000_machine_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

/* match dts */
#ifdef CONFIG_OF
static const struct of_device_id se1000_machine_dt_match[] = {
	{.compatible = "siengine,se1000-aud-machine",},
	{}
};
#endif

static struct platform_driver se1000_machine_driver = {
	.probe = se1000_machine_probe,
	.remove = se1000_machine_remove,
	.driver = {
		.name = "se1000-machine-driver",
		.pm = &snd_soc_pm_ops,
		#ifdef CONFIG_OF
			.of_match_table = se1000_machine_dt_match,
		#endif
	},
};

module_platform_driver(se1000_machine_driver);

/* Module information */
MODULE_DESCRIPTION("se1000 ALSA SoC machine driver");
MODULE_AUTHOR("jian.xu <jian.xu@siengine.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("se1000 soc card");
