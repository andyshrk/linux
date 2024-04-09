/*
 * SPDX-License-Identifier: GPL-2.0.
 * sound/soc/siengine/audio_tdm_mlp
 * Copyright (C) 2022-2023 SiEngine or its affiliates
*/

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <sound/soc.h>

#include "se1000-multi-tdm.h"

static const struct snd_pcm_hardware se1000_pcm_fe_hardware = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED
		| SNDRV_PCM_INFO_RESUME | SNDRV_PCM_INFO_MMAP_VALID,
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE
			| SNDRV_PCM_FMTBIT_S32_LE,
	.rate_min = 8000,
	.rate_max = 192000,
	.channels_min = MIN_NUM_CHANNELS,
	.channels_max = MAX_NUM_CHANNELS,
	.periods_min = MIN_NUM_PERIODS,
	.periods_max = MAX_NUM_PERIODS,
	.period_bytes_min = MIN_PERIOD_BYTE,
	.period_bytes_max = MAX_PERIOD_BYTE,
	.buffer_bytes_max = MAX_PERIOD_BYTE * MAX_NUM_PERIODS,
};

static unsigned int supported_sample_rates[] = {
	8000, 12000, 16000, 24000, 32000, 48000, 96000, 192000
};

static struct snd_pcm_hw_constraint_list se1000_constraints_sample_rates = {
	.count = ARRAY_SIZE(supported_sample_rates),
	.list = supported_sample_rates,
	.mask = 0,
};

static const struct snd_dmaengine_pcm_config multi_tdm_be_hardware_config = {
	.prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config,
	.pcm_hardware = &se1000_pcm_fe_hardware,
};

/* static value for fe channel map ctrl */
static char fe_chmap_ctl[SE1000_MULTI_MAX][2][PCM_FORMAT_MAX_NUM_CHANNEL] = {
	{/* FE 0 */
		{0, 1, 0xFF},  /* playback 2ch to slot 0 & 1 */
		{0, 1, 0xFF}   /* capture 2ch to slot 0 & 1 */
	},
	{/* FE 1 */
		{2, 3, 0xFF},  /* playback 2ch to slot 2 & 3 */
		{2, 3, 0xFF}   /* capture 2ch to slot 2 & 3 */
	},
	{/* FE 2 */
		{4, 5, 0xFF},
		{4, 5, 0xFF}
	},
	{/* FE 3 */
		{6, 7, 0xFF},
		{6, 7, 0xFF}
	},
	{/* FE 4 */
		{8, 9, 0xFF},
		{8, 9, 0xFF}
	},
	{/* FE 5 */
		{10, 11, 0xFF},
		{10, 11, 0xFF}
	},
	{/* FE 6 */
		{12, 13, 0xFF},
		{12, 13, 0xFF}
	},
	{/* FE 7 */
		{14, 15, 0xFF},
		{14, 15, 0xFF}
	},
	{/* hostless FE 8 */
		{0, 1, 0xFF},
		{0, 1, 0xFF}
	},
	{/* hostless FE 9 */
		{2, 3, 0xFF},
		{2, 3, 0xFF}
	},
	{/* hostless FE 10 */
		{4, 5, 0xFF},
		{4, 5, 0xFF}
	},
	{/* hostless FE 11 */
		{6, 7, 0xFF},
		{6, 7, 0xFF}
	},
};

static char *se1000_get_chmap(unsigned int fe_dai, unsigned int stream)
{
	if ((fe_dai < SE1000_MULTI_MAX) && (stream < 2))
		return fe_chmap_ctl[fe_dai][stream];
	else
		return NULL;
}

static int se1000_pcm_chmap_ctl_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int i, ch = 0;
	unsigned int fe_dai_id = kcontrol->private_value & 0xFF;
	unsigned int stream = (kcontrol->private_value >> 8) & 0xFF;
	unsigned int chmask = ucontrol->value.integer.value[0];

	pr_debug("%s, Put channel map ctrl mask %x \n", __func__, chmask);
	for (i = 0; i < PCM_FORMAT_MAX_NUM_CHANNEL; i++) {
		if (chmask & (1 << i))
			fe_chmap_ctl[fe_dai_id][stream][ch++] = i;
	}
	if (ch < PCM_FORMAT_MAX_NUM_CHANNEL)
		fe_chmap_ctl[fe_dai_id][stream][ch] = 0xFF;

	return 0;
}

static int se1000_pcm_chmap_ctl_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int i = 0;
	unsigned int fe_dai_id = kcontrol->private_value & 0xFF;
	unsigned int stream = (kcontrol->private_value >> 8) & 0xFF;
	unsigned int chmask = 0;

	for (i = 0; i < PCM_FORMAT_MAX_NUM_CHANNEL; i++) {
		if (fe_chmap_ctl[fe_dai_id][stream][i] == 0xFF)
			break;
		chmask |= (1 << fe_chmap_ctl[fe_dai_id][stream][i]);
	}
	pr_debug("%s, Get channel map ctrl mask %x \n", __func__, chmask);
	ucontrol->value.integer.value[0] = chmask;
	return 0;
}

static int se1000_pcm_chmap_ctl_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = __INT32_MAX__;
	return 0;
}

int se1000_pcm_add_chmap_controls(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_pcm *pcm = rtd->pcm;
	struct snd_pcm_chmap *chmap_info;
	struct snd_kcontrol *kctl;
	char device_num[6];
	int i, ret = 0;

	pr_debug("%s, Channel map ctrl add\n", __func__);

	if (rtd->pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		ret = snd_pcm_add_chmap_ctls(pcm, SNDRV_PCM_STREAM_PLAYBACK,
				NULL,
				PCM_FORMAT_MAX_NUM_CHANNEL,
				/* store fe dai id and stream type and max channel info */
				(rtd->dai_link->id) | (SNDRV_PCM_STREAM_PLAYBACK << 8),
				&chmap_info);
		if (ret < 0) {
			pr_err("%s, channel map ctrl add failed\n", __func__);
			goto fail;
		}
		kctl = chmap_info->kctl;
		for (i = 0; i < kctl->count; i++)
			kctl->vd[i].access |= SNDRV_CTL_ELEM_ACCESS_WRITE;

		snprintf(device_num, sizeof(device_num), " %d", pcm->device);
		strlcat(kctl->id.name, device_num, sizeof(kctl->id.name));

		kctl->put = se1000_pcm_chmap_ctl_put;
		kctl->get = se1000_pcm_chmap_ctl_get;
		kctl->info = se1000_pcm_chmap_ctl_info;
	}

	if (rtd->pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		ret = snd_pcm_add_chmap_ctls(pcm, SNDRV_PCM_STREAM_CAPTURE,
				NULL,
				PCM_FORMAT_MAX_NUM_CHANNEL,
				(rtd->dai_link->id) | (SNDRV_PCM_STREAM_CAPTURE << 8),
				&chmap_info);
		if (ret < 0) {
			pr_err("%s, channel map ctrl add failed\n", __func__);
			goto fail;
		}
		kctl = chmap_info->kctl;
		for (i = 0; i < kctl->count; i++)
			kctl->vd[i].access |= SNDRV_CTL_ELEM_ACCESS_WRITE;

		snprintf(device_num, sizeof(device_num), " %d", pcm->device);
		strlcat(kctl->id.name, device_num, sizeof(kctl->id.name));

		kctl->put = se1000_pcm_chmap_ctl_put;
		kctl->get = se1000_pcm_chmap_ctl_get;
		kctl->info = se1000_pcm_chmap_ctl_info;
	}

	return 0;

fail:
	pr_err("%s: failed add fe chmap ctls, err = %d\n", __func__, ret);
	return ret;
}

static int se1000_fe_tdm_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *fe = asoc_substream_to_rtd(substream);
	struct se1000_multi_tdm *psmt = snd_soc_dai_get_drvdata(dai);
	struct se1000_dma_runtime_data *prtd = psmt->prtd;

	int stream = substream->stream;
	struct snd_soc_dpcm *dpcm;
	struct se_i2s_base *i2s;
	struct device *be_dev;

	if (prtd->tdm_channels == 8) {
		if (dai->id >= 4) {
			dev_err(dai->dev, "tdm channel is 8, this node not support\n");
			return -EINVAL;
		}
	}

	for_each_dpcm_be(fe, stream, dpcm) {
		struct snd_soc_pcm_runtime *be = dpcm->be;
		struct snd_pcm_substream *be_substream;
		struct snd_soc_dai *be_dai = asoc_rtd_to_cpu(be, 0);

		if (dpcm->fe != fe)
			continue;

		be_substream = snd_soc_dpcm_get_substream(be, stream);
		be_dev = be_dai->dev;
		break;
	}

	i2s = dev_get_drvdata(be_dev);
	i2s->prtd = psmt->prtd;

	snd_soc_dai_init_dma_data(dai, &i2s->dma_data[SNDRV_PCM_STREAM_PLAYBACK],
		&i2s->dma_data[SNDRV_PCM_STREAM_CAPTURE]);

	snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE, &se1000_constraints_sample_rates);
	/* limit the period size is power of min step to avoid memcpy buffer overflow */
	snd_pcm_hw_constraint_step(substream->runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
		psmt->prtd->min_period);
	snd_pcm_hw_constraint_step(substream->runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_SIZE,
		psmt->prtd->min_period);

	psmt->substream[dai->id][stream] = substream;

	return 0;
}

static int se1000_fe_tdm_prepare(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct se1000_multi_tdm *psmt = snd_soc_dai_get_drvdata(dai);
	int stream = substream->stream;

	psmt->frame_bytes[dai->id][stream] = runtime->frame_bits/8;
	/* fe channels are variant */
	psmt->min_period_size[dai->id][stream] = MIN_PERIOD_SIZE * psmt->frame_bytes[dai->id][stream];

	/* fe app and hw ptr must be reset after xrun prepare */
	psmt->prtd->pos[dai->id][stream] = 0;

	return 0;
}

void se1000_fe_tdm_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	return;
}

static int se1000_fe_tdm_hw_params(struct snd_pcm_substream *substream,
    struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	char *chmap = NULL;
	struct se1000_multi_tdm *prtd = snd_soc_dai_get_drvdata(dai);

	chmap = se1000_get_chmap(dai->id, substream->stream);
	if (!chmap) {
		pr_err("%s: Invailid channel map parameters !\n", __func__);
		return -EINVAL;
	}

	if (prtd)
		prtd->channel_map[dai->id][substream->stream] = chmap;

	return 0;
}

static int se1000_fe_dai_pcm_new(struct snd_soc_pcm_runtime *rtd, struct snd_soc_dai *dai)
{
	return se1000_pcm_add_chmap_controls(rtd);
}

static void se1000_fe_tdm_ctrl(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai, bool on)
{
	struct se1000_multi_tdm *psmt = snd_soc_dai_get_drvdata(dai);
	int stream = substream->stream;
	char *be_dma_ptr = NULL;
	size_t frame_bytes = 0;
	int chan_off = 0;
	int j = 0;

	if (on) {
		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
			psmt->play[dai->id] = true;
		else
			psmt->capture[dai->id] = true;
	} else {
		if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
			psmt->play[dai->id] = false;
			frame_bytes = READ_ONCE(psmt->frame_bytes[dai->id][substream->stream]);
			if (psmt->channel_map[dai->id][stream] && \
				psmt->channel_map[dai->id][stream][0] < psmt->prtd->tdm_channels)
				chan_off = psmt->channel_map[dai->id][stream][0] * (psmt->prtd->tdm_bytes / psmt->prtd->tdm_channels);

			/* clear current fe channel */
			be_dma_ptr = psmt->dma_buf[stream]->area;
			do {
				memset_io(&(be_dma_ptr[j + chan_off]), 0, frame_bytes);
				j = j + psmt->prtd->tdm_bytes;
			} while (j < psmt->prtd->prealloc_buffer);
		} else
			psmt->capture[dai->id] = false;
	}
}

static int se1000_fe_tdm_trigger(struct snd_pcm_substream *substream, int cmd,
	struct snd_soc_dai *dai)
{
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		se1000_fe_tdm_ctrl(substream, dai, true);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		se1000_fe_tdm_ctrl(substream, dai, false);
		break;
	default:
		dev_err(dai->dev, "unknown cmd\n");
		return -EINVAL;
	}

	return 0;
}

static struct snd_soc_dai_ops se1000_fe_tdm_dai_ops = {
	.startup	= se1000_fe_tdm_startup,
	.prepare	= se1000_fe_tdm_prepare,
	.shutdown 	= se1000_fe_tdm_shutdown,
	.hw_params	= se1000_fe_tdm_hw_params,
	.trigger	= se1000_fe_tdm_trigger,
};

static struct snd_soc_dai_driver se1000_fe_dai_driver[] = {
	{
		.name = "se1000-multi-tdm0",
		.id = SE1000_MULTI_TDM0,
		.playback = {
			.stream_name = "tdm0 playback",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "tdm0 capture",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_fe_tdm_dai_ops,
		.pcm_new = se1000_fe_dai_pcm_new,
		.symmetric_rates = 1,
	},
	{
		.name = "se1000-multi-tdm1",
		.id = SE1000_MULTI_TDM1,
		.playback = {
			.stream_name = "tdm1 playback",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.capture = {
			.stream_name = "tdm1 capture",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.ops = &se1000_fe_tdm_dai_ops,
		.pcm_new = se1000_fe_dai_pcm_new,
		.symmetric_rates = 1,
	},
	{
		.name = "se1000-multi-tdm2",
		.id = SE1000_MULTI_TDM2,
		.playback = {
			.stream_name = "tdm2 playback",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.capture = {
			.stream_name = "tdm2 capture",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.ops = &se1000_fe_tdm_dai_ops,
		.pcm_new = se1000_fe_dai_pcm_new,
		.symmetric_rates = 1,
	},
	{
		.name = "se1000-multi-tdm3",
		.id = SE1000_MULTI_TDM3,
		.playback = {
			.stream_name = "tdm3 playback",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.capture = {
			.stream_name = "tdm3 capture",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.ops = &se1000_fe_tdm_dai_ops,
		.pcm_new = se1000_fe_dai_pcm_new,
		.symmetric_rates = 1,
	},
	{
		.name = "se1000-multi-tdm4",
		.id = SE1000_MULTI_TDM4,
		.playback = {
			.stream_name = "tdm4 playback",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.capture = {
			.stream_name = "tdm4 capture",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.ops = &se1000_fe_tdm_dai_ops,
		.pcm_new = se1000_fe_dai_pcm_new,
		.symmetric_rates = 1,
	},
	{
		.name = "se1000-multi-tdm5",
		.id = SE1000_MULTI_TDM5,
		.playback = {
			.stream_name = "tdm5 playback",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.capture = {
			.stream_name = "tdm5 capture",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.ops = &se1000_fe_tdm_dai_ops,
		.pcm_new = se1000_fe_dai_pcm_new,
		.symmetric_rates = 1,
	},
	{
		.name = "se1000-multi-tdm6",
		.id = SE1000_MULTI_TDM6,
		.playback = {
			.stream_name = "tdm6 playback",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.capture = {
			.stream_name = "tdm6 capture",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.ops = &se1000_fe_tdm_dai_ops,
		.pcm_new = se1000_fe_dai_pcm_new,
		.symmetric_rates = 1,
	},
	{
		.name = "se1000-multi-tdm7",
		.id = SE1000_MULTI_TDM7,
		.playback = {
			.stream_name = "tdm7 playback",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.capture = {
			.stream_name = "tdm7 capture",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.ops = &se1000_fe_tdm_dai_ops,
		.pcm_new = se1000_fe_dai_pcm_new,
		.symmetric_rates = 1,
	},
	/* audio share hostless playback */
	{
		.name = "se1000-multi-fe-share0",
		.id = SE1000_MULTI_HL_FE_SHR0,
		.playback = {
			.stream_name = "hostless fe share0 playback",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "hostless fe share0 capture",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_fe_tdm_dai_ops,
		.pcm_new = se1000_fe_dai_pcm_new,
		.symmetric_rates = 1,
	},
	{
		.name = "se1000-multi-fe-share1",
		.id = SE1000_MULTI_HL_FE_SHR1,
		.playback = {
			.stream_name = "hostless fe share1 playback",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "hostless fe share1 capture",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_fe_tdm_dai_ops,
		.pcm_new = se1000_fe_dai_pcm_new,
		.symmetric_rates = 1,
	},
	{
		.name = "se1000-multi-fe-share2",
		.id = SE1000_MULTI_HL_FE_SHR2,
		.playback = {
			.stream_name = "hostless fe share2 playback",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "hostless fe share2 capture",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_fe_tdm_dai_ops,
		.pcm_new = se1000_fe_dai_pcm_new,
		.symmetric_rates = 1,
	},
	{
		.name = "se1000-multi-fe-share3",
		.id = SE1000_MULTI_HL_FE_SHR3,
		.playback = {
			.stream_name = "hostless fe share3 playback",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "hostless fe share3 capture",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_fe_tdm_dai_ops,
		.pcm_new = se1000_fe_dai_pcm_new,
		.symmetric_rates = 1,
	},
	{
		.name = "se1000-multi-fe-share4",
		.id = SE1000_MULTI_HL_FE_SHR4,
		.playback = {
			.stream_name = "hostless fe share4 playback",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "hostless fe share4 capture",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_fe_tdm_dai_ops,
		.pcm_new = se1000_fe_dai_pcm_new,
		.symmetric_rates = 1,
	},
	{
		.name = "se1000-multi-fe-share5",
		.id = SE1000_MULTI_HL_FE_SHR5,
		.playback = {
			.stream_name = "hostless fe share5 playback",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "hostless fe share5 capture",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_fe_tdm_dai_ops,
		.pcm_new = se1000_fe_dai_pcm_new,
		.symmetric_rates = 1,
	},
	{
		.name = "se1000-multi-fe-share6",
		.id = SE1000_MULTI_HL_FE_SHR6,
		.playback = {
			.stream_name = "hostless fe share6 playback",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "hostless fe share6 capture",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_fe_tdm_dai_ops,
		.pcm_new = se1000_fe_dai_pcm_new,
		.symmetric_rates = 1,
	},
	{
		.name = "se1000-multi-fe-share7",
		.id = SE1000_MULTI_HL_FE_SHR7,
		.playback = {
			.stream_name = "hostless fe share7 playback",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "hostless fe share7 capture",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_fe_tdm_dai_ops,
		.pcm_new = se1000_fe_dai_pcm_new,
		.symmetric_rates = 1,
	},
	{
		.name = "se1000-multi-fe-share8",
		.id = SE1000_MULTI_HL_FE_SHR8,
		.playback = {
			.stream_name = "hostless fe share8 playback",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "hostless fe share8 capture",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_fe_tdm_dai_ops,
		.pcm_new = se1000_fe_dai_pcm_new,
		.symmetric_rates = 1,
	},
	{
		.name = "se1000-multi-fe-share9",
		.id = SE1000_MULTI_HL_FE_SHR9,
		.playback = {
			.stream_name = "hostless fe share9 playback",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "hostless fe share9 capture",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_fe_tdm_dai_ops,
		.pcm_new = se1000_fe_dai_pcm_new,
		.symmetric_rates = 1,
	},
	{
		.name = "se1000-multi-fe-share10",
		.id = SE1000_MULTI_HL_FE_SHR10,
		.playback = {
			.stream_name = "hostless fe share10 playback",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "hostless fe share10 capture",
			.channels_min = MIN_NUM_CHANNELS,
			.channels_max = MAX_NUM_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_fe_tdm_dai_ops,
		.pcm_new = se1000_fe_dai_pcm_new,
		.symmetric_rates = 1,
	},
};

static const struct snd_soc_component_driver se1000_component_fe_driver = {
	.name = "se1000_component_fe",
};

static int se1000_pcm_dev_fe_probe(struct platform_device *pdev)
{
	struct se1000_multi_tdm *psmt;
	struct se1000_dma_runtime_data *prtd;
	struct device_node *node = pdev->dev.of_node;
	struct device *dev;
	int ret = 0, i = 0;

	dev = &pdev->dev;
	psmt = devm_kzalloc(&pdev->dev, sizeof(*psmt), GFP_KERNEL);
	if (!psmt) {
		dev_err(&pdev->dev, "malloc memory failed\n");
		ret = -ENOMEM;
		goto err_free_mem;
	}

	psmt->dev = &pdev->dev;

	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (!prtd) {
		dev_err(&pdev->dev, "malloc memory failed\n");
		ret = -ENOMEM;
		goto err_free_mem;
	}

	prtd->config = &multi_tdm_be_hardware_config;
	psmt->prtd = prtd;

	ret = se1000_dma_pcm_request_chan_of(prtd, dev, prtd->config);
	if (ret) {
		dev_err(&pdev->dev, "request channel failed!\n");
		goto err_free_mem;
	}

	for_each_pcm_streams(i) {
		psmt->dma_buf[i] = kzalloc(sizeof(struct snd_dma_buffer), GFP_KERNEL);
		if (!psmt->dma_buf[i]) {
			kfree(psmt->dma_buf[i]);
			ret = -ENOMEM;
			goto err_free_mem;
		}

		ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV, dev,
				SE1000_DMABUF_BYTE, psmt->dma_buf[i]);
		if (ret) {
			kfree(psmt->dma_buf[i]);
			dev_err(&pdev->dev, "failed to allocate DMA buffer!\n");
			goto err_free_mem;
		}

		memset(psmt->dma_buf[i]->area, 0, sizeof(SE1000_DMABUF_BYTE));
		psmt->dma_hwoff[i] = 0;
	}

	prtd->min_period = MIN_PERIOD_SIZE;
	prtd->prealloc_buffer = psmt->dma_buf[0]->bytes;

	ret = of_property_read_u32(node, "tdm_channels", &prtd->tdm_channels);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to find tdm_channels \n");
		goto err_free_mem;
	}

	spin_lock_init(&psmt->splt);
	platform_set_drvdata(pdev, psmt);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev))
		goto err_dai_component;
	pm_runtime_get_sync(&pdev->dev);

	ret = devm_snd_soc_register_component(&pdev->dev, &se1000_dma_component_driver,
		se1000_fe_dai_driver, ARRAY_SIZE(se1000_fe_dai_driver));
	if (ret) {
		dev_err(&pdev->dev, "err_dai_component\n");
		goto err_dai_component;
	}

	return 0;

err_dai_component:
	snd_soc_unregister_component(&pdev->dev);
err_free_mem:
	kfree(psmt);
	kfree(prtd);

	return ret;
}

static int se1000_pcm_dev_fe_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	snd_soc_unregister_component(&pdev->dev);

	return 0;
}

static const struct of_device_id se1000_pcm_dts_match[] = {
	{ .compatible = "siengine,se1000-fe-dai", },
	{},
};
MODULE_DEVICE_TABLE(of, se1000_pcm_dts_match);

static struct platform_driver se1000_pcm_platform_driver = {
	.driver = {
		.name = "se1000-fe-dai",
		.of_match_table = se1000_pcm_dts_match,
	},
	.probe = se1000_pcm_dev_fe_probe,
	.remove = se1000_pcm_dev_fe_remove,
};

module_platform_driver(se1000_pcm_platform_driver);

MODULE_DESCRIPTION("Siengine SE1000 ALSA SoC platform driver");
MODULE_AUTHOR("jian.xu <jian.xu@siengine.com>");
MODULE_LICENSE("GPL v2");
