/*
 * SPDX-License-Identifier: GPL-2.0.
 * sound/soc/siengine/audio_tdm_mlp
 * Copyright (C) 2022-2023 SiEngine or its affiliates
*/

#include <linux/module.h>
#include <sound/soc.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <sound/soc.h>
#include <sound/pcm.h>

#include "se1000-multi-tdm.h"
#include "../se1000-aud-i2s.h"
#include "../audioshare/audio-share-dummy.h"

SND_SOC_DAILINK_DEFS(tdm0,
	DAILINK_COMP_ARRAY(COMP_CPU("se1000-multi-tdm0")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm1,
	DAILINK_COMP_ARRAY(COMP_CPU("se1000-multi-tdm1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm2,
	DAILINK_COMP_ARRAY(COMP_CPU("se1000-multi-tdm2")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm3,
	DAILINK_COMP_ARRAY(COMP_CPU("se1000-multi-tdm3")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm4,
	DAILINK_COMP_ARRAY(COMP_CPU("se1000-multi-tdm4")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm5,
	DAILINK_COMP_ARRAY(COMP_CPU("se1000-multi-tdm5")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm6,
	DAILINK_COMP_ARRAY(COMP_CPU("se1000-multi-tdm6")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm7,
	DAILINK_COMP_ARRAY(COMP_CPU("se1000-multi-tdm7")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(tdm_hw,
	DAILINK_COMP_ARRAY(COMP_CPU("tdm_hw")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "tdm-dummy-codec-dai")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* hostless dummy0 ul&dl <--> TDM FE */
SND_SOC_DAILINK_DEFS(hostless_fe_share0,
	DAILINK_COMP_ARRAY(COMP_CPU("se1000-multi-fe-share0")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy1 ul&dl <--> TDM FE */
SND_SOC_DAILINK_DEFS(hostless_fe_share1,
	DAILINK_COMP_ARRAY(COMP_CPU("se1000-multi-fe-share1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy2 ul&dl <--> TDM FE */
SND_SOC_DAILINK_DEFS(hostless_fe_share2,
	DAILINK_COMP_ARRAY(COMP_CPU("se1000-multi-fe-share2")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy3 ul&dl <--> TDM FE */
SND_SOC_DAILINK_DEFS(hostless_fe_share3,
	DAILINK_COMP_ARRAY(COMP_CPU("se1000-multi-fe-share3")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy4 ul&dl <--> TDM FE */
SND_SOC_DAILINK_DEFS(hostless_fe_share4,
	DAILINK_COMP_ARRAY(COMP_CPU("se1000-multi-fe-share4")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy5 ul&dl <--> TDM FE */
SND_SOC_DAILINK_DEFS(hostless_fe_share5,
	DAILINK_COMP_ARRAY(COMP_CPU("se1000-multi-fe-share5")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy6 ul&dl <--> TDM FE */
SND_SOC_DAILINK_DEFS(hostless_fe_share6,
	DAILINK_COMP_ARRAY(COMP_CPU("se1000-multi-fe-share6")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy7 ul&dl <--> TDM FE */
SND_SOC_DAILINK_DEFS(hostless_fe_share7,
	DAILINK_COMP_ARRAY(COMP_CPU("se1000-multi-fe-share7")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy8 ul&dl <--> TDM FE */
SND_SOC_DAILINK_DEFS(hostless_fe_share8,
	DAILINK_COMP_ARRAY(COMP_CPU("se1000-multi-fe-share8")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy9 ul&dl <--> TDM FE */
SND_SOC_DAILINK_DEFS(hostless_fe_share9,
	DAILINK_COMP_ARRAY(COMP_CPU("se1000-multi-fe-share9")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy10 ul&dl <--> TDM FE */
SND_SOC_DAILINK_DEFS(hostless_fe_share10,
	DAILINK_COMP_ARRAY(COMP_CPU("se1000-multi-fe-share10")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy0 ul&dl <--> share memory 0 */
SND_SOC_DAILINK_DEFS(hostless_share_dummy0,
	DAILINK_COMP_ARRAY(COMP_CPU("share-loop-dummy0")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy1 ul&dl <--> share memory 1 */
SND_SOC_DAILINK_DEFS(hostless_share_dummy1,
	DAILINK_COMP_ARRAY(COMP_CPU("share-loop-dummy1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy2 ul&dl <--> share memory 2 */
SND_SOC_DAILINK_DEFS(hostless_share_dummy2,
	DAILINK_COMP_ARRAY(COMP_CPU("share-loop-dummy2")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy3 ul&dl <--> share memory 3 */
SND_SOC_DAILINK_DEFS(hostless_share_dummy3,
	DAILINK_COMP_ARRAY(COMP_CPU("share-loop-dummy3")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy4 ul&dl <--> share memory 4 */
SND_SOC_DAILINK_DEFS(hostless_share_dummy4,
	DAILINK_COMP_ARRAY(COMP_CPU("share-loop-dummy4")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy5 ul&dl <--> share memory 5 */
SND_SOC_DAILINK_DEFS(hostless_share_dummy5,
	DAILINK_COMP_ARRAY(COMP_CPU("share-loop-dummy5")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy6 ul&dl <--> share memory 6 */
SND_SOC_DAILINK_DEFS(hostless_share_dummy6,
	DAILINK_COMP_ARRAY(COMP_CPU("share-loop-dummy6")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy7 ul&dl <--> share memory 7 */
SND_SOC_DAILINK_DEFS(hostless_share_dummy7,
	DAILINK_COMP_ARRAY(COMP_CPU("share-loop-dummy7")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy8 ul&dl <--> share memory 8 */
SND_SOC_DAILINK_DEFS(hostless_share_dummy8,
	DAILINK_COMP_ARRAY(COMP_CPU("share-loop-dummy8")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy9 ul&dl <--> share memory 9 */
SND_SOC_DAILINK_DEFS(hostless_share_dummy9,
	DAILINK_COMP_ARRAY(COMP_CPU("share-loop-dummy9")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

/* hostless dummy10 ul&dl <--> share memory 10 */
SND_SOC_DAILINK_DEFS(hostless_share_dummy10,
	DAILINK_COMP_ARRAY(COMP_CPU("share-loop-dummy10")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("se1000-share-loopback.2")));

static int se1000_be_links_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = *(rtd->dais);
	unsigned int rate = params_rate(params);
	struct se_i2s_base *i2s = dev_get_drvdata(cpu_dai->dev);
	unsigned int dai_fmt;
	int ret = 0;

	snd_soc_dai_set_sysclk(cpu_dai, cpu_dai->id, rate, SND_SOC_CLOCK_OUT);

	/* Set I2S fmt */
	dai_fmt = i2s->i2s_config.i2s_format | i2s->i2s_config.i2s_mode
		| i2s->i2s_config.i2s_clk_polar;

	ret = snd_soc_dai_set_fmt(cpu_dai, dai_fmt);
	if (ret < 0) {
		dev_err(cpu_dai->dev,
			"ASoC: Failed to set DAI format: %d\n", ret);
		return ret;
	}

	return ret;
}

static struct snd_soc_ops se1000_tdm_be_ops = {
	.hw_params = se1000_be_links_hw_params,
};

static int se1000_tdm_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_dai *dai = asoc_rtd_to_cpu(rtd, 0);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);
	struct se_i2s_base *i2s = snd_soc_dai_get_drvdata(dai);

	channels->min = channels->max = i2s->prtd->tdm_channels;

	return 0;
}

static const struct snd_soc_dapm_route se1000_afe_pcm_routes[] = {
	/* sink <-- source */
	{"TDM3_HW Playback", NULL, "tdm0 playback"},
	{"TDM3_HW Playback", NULL, "tdm1 playback"},
	{"TDM3_HW Playback", NULL, "tdm2 playback"},
	{"TDM3_HW Playback", NULL, "tdm3 playback"},
	{"TDM3_HW Playback", NULL, "tdm4 playback"},
	{"TDM3_HW Playback", NULL, "tdm5 playback"},
	{"TDM3_HW Playback", NULL, "tdm6 playback"},
	{"TDM3_HW Playback", NULL, "tdm7 playback"},
	{"TDM3_HW Playback", NULL, "hostless fe share0 playback"},
	{"TDM3_HW Playback", NULL, "hostless fe share1 playback"},
	{"TDM3_HW Playback", NULL, "hostless fe share2 playback"},
	{"TDM3_HW Playback", NULL, "hostless fe share3 playback"},
	{"TDM3_HW Playback", NULL, "hostless fe share4 playback"},
	{"TDM3_HW Playback", NULL, "hostless fe share5 playback"},
	{"TDM3_HW Playback", NULL, "hostless fe share6 playback"},
	{"TDM3_HW Playback", NULL, "hostless fe share7 playback"},
	{"TDM3_HW Playback", NULL, "hostless fe share8 playback"},
	{"TDM3_HW Playback", NULL, "hostless fe share9 playback"},
	{"TDM3_HW Playback", NULL, "hostless fe share10 playback"},
	{"tdm-dummy-Playback", NULL, "TDM3_HW Playback"},

	{"tdm0 capture", NULL, "TDM3_HW Capture"},
	{"tdm1 capture", NULL, "TDM3_HW Capture"},
	{"tdm2 capture", NULL, "TDM3_HW Capture"},
	{"tdm3 capture", NULL, "TDM3_HW Capture"},
	{"tdm4 capture", NULL, "TDM3_HW Capture"},
	{"tdm5 capture", NULL, "TDM3_HW Capture"},
	{"tdm6 capture", NULL, "TDM3_HW Capture"},
	{"tdm7 capture", NULL, "TDM3_HW Capture"},
	{"hostless fe share0 capture", NULL, "TDM3_HW Capture"},
	{"hostless fe share1 capture", NULL, "TDM3_HW Capture"},
	{"hostless fe share2 capture", NULL, "TDM3_HW Capture"},
	{"hostless fe share3 capture", NULL, "TDM3_HW Capture"},
	{"hostless fe share4 capture", NULL, "TDM3_HW Capture"},
	{"hostless fe share5 capture", NULL, "TDM3_HW Capture"},
	{"hostless fe share6 capture", NULL, "TDM3_HW Capture"},
	{"hostless fe share7 capture", NULL, "TDM3_HW Capture"},
	{"hostless fe share8 capture", NULL, "TDM3_HW Capture"},
	{"hostless fe share9 capture", NULL, "TDM3_HW Capture"},
	{"hostless fe share10 capture", NULL, "TDM3_HW Capture"},
	{"TDM3_HW Capture", NULL, "tdm-dummy-Capture"},
};

static struct snd_soc_dai_link se1000_dai_links_tdm[] = {
	/* FrontEnd DAI Links */
	{
		.name = "se1000-multi-tdm0",
		.stream_name = "se1000-multi-tdm0",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		/* binding pcm dai link with fe dai id */
		.id = SE1000_MULTI_TDM0,
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			SND_SOC_DPCM_TRIGGER_PRE},
		SND_SOC_DAILINK_REG(tdm0),
	},
	{
		.name = "se1000-multi-tdm1",
		.stream_name = "se1000-multi-tdm1",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.id = SE1000_MULTI_TDM1,
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			SND_SOC_DPCM_TRIGGER_PRE},
		SND_SOC_DAILINK_REG(tdm1),
	},
	{
		.name = "se1000-multi-tdm2",
		.stream_name = "se1000-multi-tdm2",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.id = SE1000_MULTI_TDM2,
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			SND_SOC_DPCM_TRIGGER_PRE},
		SND_SOC_DAILINK_REG(tdm2),
	},
	{
		.name = "se1000-multi-tdm3",
		.stream_name = "se1000-multi-tdm3",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.id = SE1000_MULTI_TDM3,
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			SND_SOC_DPCM_TRIGGER_PRE},
		SND_SOC_DAILINK_REG(tdm3),
	},
	{
		.name = "se1000-multi-tdm4",
		.stream_name = "se1000-multi-tdm4",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.id = SE1000_MULTI_TDM4,
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			SND_SOC_DPCM_TRIGGER_PRE},
		SND_SOC_DAILINK_REG(tdm4),
	},
	{
		.name = "se1000-multi-tdm5",
		.stream_name = "se1000-multi-tdm5",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.id = SE1000_MULTI_TDM5,
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			SND_SOC_DPCM_TRIGGER_PRE},
		SND_SOC_DAILINK_REG(tdm5),
	},
	{
		.name = "se1000-multi-tdm6",
		.stream_name = "se1000-multi-tdm6",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.id = SE1000_MULTI_TDM6,
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			SND_SOC_DPCM_TRIGGER_PRE},
		SND_SOC_DAILINK_REG(tdm6),
	},
	{
		.name = "se1000-multi-tdm7",
		.stream_name = "se1000-multi-tdm7",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.id = SE1000_MULTI_TDM7,
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			SND_SOC_DPCM_TRIGGER_PRE},
		SND_SOC_DAILINK_REG(tdm7),
	},
	/* hostless loopback playback for audio share */
	{
		.name = "se1000-multi-HL-FE-SHR0",
		.stream_name = "se1000-multi-HL-FE-SHR0",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.id = SE1000_MULTI_HL_FE_SHR0,
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			SND_SOC_DPCM_TRIGGER_PRE},
		SND_SOC_DAILINK_REG(hostless_fe_share0),
	},
	{
		.name = "se1000-multi-HL-FE-SHR1",
		.stream_name = "se1000-multi-HL-FE-SHR1",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.id = SE1000_MULTI_HL_FE_SHR1,
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			SND_SOC_DPCM_TRIGGER_PRE},
		SND_SOC_DAILINK_REG(hostless_fe_share1),
	},
	{
		.name = "se1000-multi-HL-FE-SHR2",
		.stream_name = "se1000-multi-HL-FE-SHR2",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.id = SE1000_MULTI_HL_FE_SHR2,
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			SND_SOC_DPCM_TRIGGER_PRE},
		SND_SOC_DAILINK_REG(hostless_fe_share2),
	},
	{
		.name = "se1000-multi-HL-FE-SHR3",
		.stream_name = "se1000-multi-HL-FE-SHR3",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.id = SE1000_MULTI_HL_FE_SHR3,
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			SND_SOC_DPCM_TRIGGER_PRE},
		SND_SOC_DAILINK_REG(hostless_fe_share3),
	},
	{
		.name = "se1000-multi-HL-FE-SHR4",
		.stream_name = "se1000-multi-HL-FE-SHR4",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.id = SE1000_MULTI_HL_FE_SHR4,
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			SND_SOC_DPCM_TRIGGER_PRE},
		SND_SOC_DAILINK_REG(hostless_fe_share4),
	},
	{
		.name = "se1000-multi-HL-FE-SHR5",
		.stream_name = "se1000-multi-HL-FE-SHR5",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.id = SE1000_MULTI_HL_FE_SHR5,
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			SND_SOC_DPCM_TRIGGER_PRE},
		SND_SOC_DAILINK_REG(hostless_fe_share5),
	},
	{
		.name = "se1000-multi-HL-FE-SHR6",
		.stream_name = "se1000-multi-HL-FE-SHR6",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.id = SE1000_MULTI_HL_FE_SHR6,
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			SND_SOC_DPCM_TRIGGER_PRE},
		SND_SOC_DAILINK_REG(hostless_fe_share6),
	},
	{
		.name = "se1000-multi-HL-FE-SHR7",
		.stream_name = "se1000-multi-HL-FE-SHR7",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.id = SE1000_MULTI_HL_FE_SHR7,
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			SND_SOC_DPCM_TRIGGER_PRE},
		SND_SOC_DAILINK_REG(hostless_fe_share7),
	},
	{
		.name = "se1000-multi-HL-FE-SHR8",
		.stream_name = "se1000-multi-HL-FE-SHR8",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.id = SE1000_MULTI_HL_FE_SHR8,
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			SND_SOC_DPCM_TRIGGER_PRE},
		SND_SOC_DAILINK_REG(hostless_fe_share8),
	},
	{
		.name = "se1000-multi-HL-FE-SHR9",
		.stream_name = "se1000-multi-HL-FE-SHR9",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.id = SE1000_MULTI_HL_FE_SHR9,
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			SND_SOC_DPCM_TRIGGER_PRE},
		SND_SOC_DAILINK_REG(hostless_fe_share9),
	},
	{
		.name = "se1000-multi-HL-FE-SHR10",
		.stream_name = "se1000-multi-HL-FE-SHR10",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.id = SE1000_MULTI_HL_FE_SHR10,
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			SND_SOC_DPCM_TRIGGER_PRE},
		SND_SOC_DAILINK_REG(hostless_fe_share10),
	},
	{
		.name = "se1000-multi-HL-SHR-DUMMY0",
		.stream_name = "se1000-multi-HL-SHR-DUMMY0",
		.id = SE1000_HL_SHR_DUMMY0,
		SND_SOC_DAILINK_REG(hostless_share_dummy0),
	},
	{
		.name = "se1000-multi-HL-SHR-DUMMY1",
		.stream_name = "se1000-multi-HL-SHR-DUMMY1",
		.id = SE1000_HL_SHR_DUMMY1,
		SND_SOC_DAILINK_REG(hostless_share_dummy1),
	},
	{
		.name = "se1000-multi-HL-SHR-DUMMY2",
		.stream_name = "se1000-multi-HL-SHR-DUMMY2",
		.id = SE1000_HL_SHR_DUMMY2,
		SND_SOC_DAILINK_REG(hostless_share_dummy2),
	},
	{
		.name = "se1000-multi-HL-SHR-DUMMY3",
		.stream_name = "se1000-multi-HL-SHR-DUMMY3",
		.id = SE1000_HL_SHR_DUMMY3,
		SND_SOC_DAILINK_REG(hostless_share_dummy3),
	},
	{
		.name = "se1000-multi-HL-SHR-DUMMY4",
		.stream_name = "se1000-multi-HL-SHR-DUMMY4",
		.id = SE1000_HL_SHR_DUMMY4,
		SND_SOC_DAILINK_REG(hostless_share_dummy4),
	},
	{
		.name = "se1000-multi-HL-SHR-DUMMY5",
		.stream_name = "se1000-multi-HL-SHR-DUMMY5",
		.id = SE1000_HL_SHR_DUMMY5,
		SND_SOC_DAILINK_REG(hostless_share_dummy5),
	},
	{
		.name = "se1000-multi-HL-SHR-DUMMY6",
		.stream_name = "se1000-multi-HL-SHR-DUMMY6",
		.id = SE1000_HL_SHR_DUMMY6,
		SND_SOC_DAILINK_REG(hostless_share_dummy6),
	},
	{
		.name = "se1000-multi-HL-SHR-DUMMY7",
		.stream_name = "se1000-multi-HL-SHR-DUMMY7",
		.id = SE1000_HL_SHR_DUMMY7,
		SND_SOC_DAILINK_REG(hostless_share_dummy7),
	},
	{
		.name = "se1000-multi-HL-SHR-DUMMY8",
		.stream_name = "se1000-multi-HL-SHR-DUMMY8",
		.id = SE1000_HL_SHR_DUMMY8,
		SND_SOC_DAILINK_REG(hostless_share_dummy8),
	},
	{
		.name = "se1000-multi-HL-SHR-DUMMY9",
		.stream_name = "se1000-multi-HL-SHR-DUMMY9",
		.id = SE1000_HL_SHR_DUMMY9,
		SND_SOC_DAILINK_REG(hostless_share_dummy9),
	},
	{
		.name = "se1000-multi-HL-SHR-DUMMY10",
		.stream_name = "se1000-multi-HL-SHR-DUMMY10",
		.id = SE1000_HL_SHR_DUMMY10,
		SND_SOC_DAILINK_REG(hostless_share_dummy10),
	},
	/* BE must be the last
	 * Back End DAI links
	 */
	{
		.name = "se1000-tdm-hw",
		.stream_name = "se1000-tdm-hw",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.be_hw_params_fixup = se1000_tdm_be_hw_params_fixup,
		.ops = &se1000_tdm_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(tdm_hw),
	},
};

static struct snd_soc_card se1000_soc_card_tdmslot = {
	.name = "se1000-tdmslot",
	.owner = THIS_MODULE,
	.dai_link = se1000_dai_links_tdm,
	.num_links = ARRAY_SIZE(se1000_dai_links_tdm),
	.dapm_routes = se1000_afe_pcm_routes,
	.num_dapm_routes = ARRAY_SIZE(se1000_afe_pcm_routes),
};

static int parser_card_dai_link_components_node(struct snd_soc_card *card)
{
	int i, index, ret = 0;
	struct device *cdev = card->dev;
	struct snd_soc_dai_link *dai_link = card->dai_link;
	struct device_node *np = NULL;
	struct device_node *default_cpu = NULL;

	if (!cdev) {
		dev_err(cdev, "%s: Sound card device invalid!\n", __func__);
		return -ENODEV;
	}
	for (i = 0; i < card->num_links - 1; i++) {
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
						 "se-fe-cpu-name",
						 dai_link[i].cpus->dai_name);
			if (index >= 0) {
				np = of_parse_phandle(cdev->of_node, "se-fe-cpu", index);
				if (!np) {
					dev_err(cdev, "%s: phandle for cpu dai %s , index %d failed\n", \
						__func__, dai_link[i].cpus->dai_name, index);
					ret = -ENODEV;
					goto err;
				}
				dai_link[i].cpus->of_node = np;
			}
		}

		/* parser default fe dai and platform */
		if (!dai_link[i].cpus->of_node) {
			index = of_property_match_string(cdev->of_node, "se-fe-cpu-name", "se1000-fe-dai");
			if (index >= 0) {
				default_cpu = of_parse_phandle(cdev->of_node, "se-fe-cpu", index);
				if (!default_cpu) {
					dev_err(cdev, "%s: phandle for default cpu dai %s , index %d failed\n", \
						__func__, dai_link[i].cpus->dai_name, index);
					ret = -ENODEV;
					goto err;
				}
				dai_link[i].cpus->of_node = default_cpu;
			}
		}
		/* parser default platform */
		if (!dai_link[i].platforms->name && !dai_link[i].platforms->of_node) {
			dai_link[i].platforms->of_node = default_cpu;
			dai_link[i].platforms->name = NULL;
		}
		dai_link[i].codecs->name = "snd-soc-dummy";
	}

	np = of_parse_phandle(cdev->of_node, "se-plat-be", 0);
	if (!np) {
		dev_err(cdev, "%s: Property 'se-plat-be' missing or invalid\n", __func__);
		return -EINVAL;
	}
	dai_link[card->num_links - 1].cpus->of_node = np;
	dai_link[card->num_links - 1].platforms->of_node = np;

	np = of_parse_phandle(cdev->of_node, "audio-codec", 0);
	if (!np) {
		dev_err(cdev, "%s: Property 'audio-codec' missing or invalid\n", __func__);
		return -EINVAL;
	}
	dai_link[card->num_links - 1].codecs->of_node = np;
	dai_link[card->num_links - 1].codecs->dai_name = "tdm-dummy-codec-dai";
err:
	return ret;
}

static int se1000_tdmslot_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &se1000_soc_card_tdmslot;
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

static int se1000_tdmslot_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

/***match dts***/
#ifdef CONFIG_OF
static const struct of_device_id se1000_tdmslot_dt_match[] = {
	{.compatible = "siengine,se1000-tdm-card",},
	{}
};
#endif

static struct platform_driver se1000_machine_driver = {
	.probe = se1000_tdmslot_probe,
	.remove = se1000_tdmslot_remove,
	.driver = {
		.name = "se1000-tdm-driver",
		#ifdef CONFIG_OF
			.of_match_table = se1000_tdmslot_dt_match,
		#endif
	},
};

module_platform_driver(se1000_machine_driver);

/* Module information */
MODULE_DESCRIPTION("se1000 SoC tdmslot machine driver");
MODULE_AUTHOR("jian.xu <jian.xu@siengine.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("se1000 tdmslot card");

