/*
 * SPDX-License-Identifier: GPL-2.0.
 * sound/soc/siengine/audioshare
 * Copyright (C) 2021-2023 SiEngine or its affiliates
*/

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <sound/soc.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>

#include <linux/rpmsg.h>
#include <linux/rpmsg/siengine-rpmsg.h>
#include "../../../../../drivers/rpmsg/rpmsg_internal.h"
#include "../../../../../drivers/rpmsg/se1000-rpmsg-dev.h"
#include "../audio_loopback/audio-share-dummy-loop-ipc.h"
#include "audio-share-dummy.h"

static int shmem_data_avail(struct aud_dummy_stream_data *stream_priv);

static const struct snd_pcm_hardware se1000_dummy_hardware = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED
		| SNDRV_PCM_INFO_RESUME | SNDRV_PCM_INFO_MMAP_VALID,
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE
			| SNDRV_PCM_FMTBIT_S32_LE,
	.rate_min = 8000,
	.rate_max = 192000,
	.channels_min = MIN_SHARE_CHANNELS,
	.channels_max = MAX_SHARE_CHANNELS,
	.period_bytes_min = 256,
	.period_bytes_max = PERIOD_BYTE_MAX,
	.periods_min = 2,
	.periods_max = 4096,
	.buffer_bytes_max = BUFFER_BYTE_MAX,
};

#if 0
static void se1000_tdm_dummy_rpmsg_send(struct se1000_aud_dummy *priv, uint32_t opcode, uint32_t offset)
{
	dummy_rpmsg_payload_t payload = {opcode, offset};

	rpmsg_send(priv->ept, &payload, sizeof(dummy_rpmsg_payload_t));
	pr_debug("%s: send dummy share msg session %d opcode %x offset %d\n", \
					__func__, priv->id, opcode, offset);
}
#endif

static int se1000_tdm_dummy_rpmsg_cb(struct rpmsg_device *rpmsg_dev, void *data, int len,
										void *priv, u32 src)
{
	dummy_rpmsg_payload_t *payload = (dummy_rpmsg_payload_t *)data;
	struct se1000_aud_dummy *pdata = (struct se1000_aud_dummy *)priv;
	struct aud_dummy_stream_data *playback = NULL, *capture = NULL;

	switch (payload->opcode) {
	case OPCODE_AUD_SHARE_DATA_WRITE_DONE:
		/*
		 * paired device has done data write, only for UL;
		 */
		capture = pdata->stream_priv[SNDRV_PCM_STREAM_CAPTURE];
		capture->pos += READ_ONCE(payload->offset);
		if (capture->pos >= capture->buffer_size)
			capture->pos = 0;
		snd_pcm_period_elapsed(capture->substream);
		pr_debug("%s: Pair ep rw done session %d off %d pos %d\n", __func__,\
						pdata->id, payload->offset, capture->pos);
		break;
	case OPCODE_AUD_SHARE_DATA_READ_DONE:
		/*
		 * paired device has done data read, only for DL;
		 */
		playback = pdata->stream_priv[SNDRV_PCM_STREAM_PLAYBACK];
		playback->pos += READ_ONCE(payload->offset);
		if (playback->pos >= playback->buffer_size)
			playback->pos = 0;
		snd_pcm_period_elapsed(playback->substream);
		pr_debug("%s: Pair ep rw done session %d off %d pos %d\n", __func__,\
						pdata->id, payload->offset, playback->pos);
		break;
	default:
		break;
	}
	return 0;
}

static int se1000_rpmsg_from_user_cb(struct rpmsg_device *rpmsg_dev, void *data, int len,
										void *priv, u32 src)
{
	struct aud_share_rpmsg_info *rpmsg = (struct aud_share_rpmsg_info *)data;
	struct se1000_aud_dummy *pdata = (struct se1000_aud_dummy *)priv;
	struct aud_dummy_stream_data *playback = NULL, *capture = NULL;

	switch (rpmsg->opcode) {
	case OPCODE_AUD_MESG_FROMUS_SYNC0:
	case OPCODE_AUD_MESG_FROMUS_SYNC1:
	case OPCODE_AUD_MESG_FROMUS_SYNC2:
	case OPCODE_AUD_MESG_FROMUS_SYNC3:
	case OPCODE_AUD_MESG_FROMUS_SYNC4:
	case OPCODE_AUD_MESG_FROMUS_SYNC5:
	case OPCODE_AUD_MESG_FROMUS_SYNC6:
	case OPCODE_AUD_MESG_FROMUS_SYNC7:
	case OPCODE_AUD_MESG_FROMUS_SYNC8:
	case OPCODE_AUD_MESG_FROMUS_SYNC9:
	case OPCODE_AUD_MESG_FROMUS_SYNC10:
		playback = pdata->stream_priv[SNDRV_PCM_STREAM_PLAYBACK];
		if (rpmsg->status == MSG_READY) {
			playback->rpmsg.status = MSG_RUNING;
			playback->rpmsg_event = true;
			wake_up_interruptible(&playback->waitq);
		}

		break;
	case OPCODE_AUD_MESG_FROMUS_UL_SYNC0:
	case OPCODE_AUD_MESG_FROMUS_UL_SYNC1:
	case OPCODE_AUD_MESG_FROMUS_UL_SYNC2:
	case OPCODE_AUD_MESG_FROMUS_UL_SYNC3:
	case OPCODE_AUD_MESG_FROMUS_UL_SYNC4:
	case OPCODE_AUD_MESG_FROMUS_UL_SYNC5:
	case OPCODE_AUD_MESG_FROMUS_UL_SYNC6:
	case OPCODE_AUD_MESG_FROMUS_UL_SYNC7:
	case OPCODE_AUD_MESG_FROMUS_UL_SYNC8:
	case OPCODE_AUD_MESG_FROMUS_UL_SYNC9:
	case OPCODE_AUD_MESG_FROMUS_UL_SYNC10:
		capture = pdata->stream_priv[SNDRV_PCM_STREAM_CAPTURE];
		if (rpmsg->status == MSG_READY) {
			capture->rpmsg.status = MSG_RUNING;
			capture->rpmsg_event = true;
			wake_up_interruptible(&capture->waitq);
		}
		break;
	default:
		pr_err("error: %s, %d\n", __func__, __LINE__);
		break;
	}

	return 0;
}

static int se1000_tdm_dummy_rpmsg_init(struct device *dev, struct se1000_aud_dummy *priv)
{
	struct rpmsg_device *rpdev = NULL;
	uint32_t rpmsg_id = 0;

	struct rpmsg_channel_info ept_info = {
		.name = "aud_tmd_dummy",
	};

	rpdev = find_rpmsg_device_by_phandle(dev->of_node);
	if (!rpdev) {
		dev_err(dev, "find_rpmsg_device_by_phandle failed\n");
		return -EINVAL;
	}

	if (of_property_read_u32(dev->of_node, "se-share-type", &priv->rpmsg_type) != 0) {
		dev_err(dev, "%s: se-share-type missing in DT node\n", __func__);
		return -EINVAL;
	}
	rpmsg_id = AUD_SHR_CP2AP_RPMSG_ID_0 | (priv->rpmsg_type << 4) | priv->id;

	ept_info.src = rpmsg_id;
	ept_info.dst = rpmsg_id;

	priv->ept = rpmsg_create_ept(rpdev, se1000_tdm_dummy_rpmsg_cb, priv, ept_info);
	if (!priv->ept) {
		dev_err(dev, "dummy audio rpmsg_create_ept failed\n");
		return -EPROBE_DEFER;
	}

	rpmsg_id = AUD_SHR_KCP2UAP_RPMSG_ID_0 + (priv->rpmsg_type << 4) + priv->id;
	ept_info.src = rpmsg_id;
	ept_info.dst = rpmsg_id;
	priv->uept = rpmsg_create_ept(rpdev, se1000_rpmsg_from_user_cb, priv, ept_info);
	if (!priv->ept) {
		dev_err(dev, "dummy audio rpmsg_create_ept failed\n");
		return -EPROBE_DEFER;
	}

	return 0;
}

static int se1000_tdm_dummy_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct se1000_aud_dummy *priv = snd_soc_dai_get_drvdata(dai);
	struct aud_dummy_stream_data *stream_priv = priv->stream_priv[substream->stream];
	int ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		WRITE_ONCE(stream_priv->shm_t->write_idx, 0);
		WRITE_ONCE(stream_priv->shm_t->write_flag, 0);
	} else {
		WRITE_ONCE(stream_priv->shm_t->read_idx, 0);
	}

	snd_pcm_set_runtime_buffer(substream, stream_priv->dma_buf);
	substream->dma_buffer = *(stream_priv->dma_buf);
	stream_priv->substream = substream;
	stream_priv->dma_ofs = 0;

	WRITE_ONCE(stream_priv->pos, 0);

	return ret;
}

static void se1000_tdm_dummy_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct se1000_aud_dummy *priv = snd_soc_dai_get_drvdata(dai);
	struct aud_dummy_stream_data *stream_priv = priv->stream_priv[substream->stream];
	int ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		WRITE_ONCE(stream_priv->shm_t->read_idx, 0);
		WRITE_ONCE(stream_priv->shm_t->write_idx, 0);
	}

	stream_priv->pos = 0;
	stream_priv->cap_done = false;
	stream_priv->substream = NULL;

	if (stream_priv->rpmsg.status == MSG_RUNING) {
		stream_priv->rpmsg.status = MSG_STOP;

		ret = rpmsg_send(priv->uept, &(stream_priv->rpmsg), sizeof(struct aud_share_rpmsg_info));
		if (ret < 0)
			pr_err("%s, rpmsg send failed!\n", __func__);
	}

	stream_priv->count = 0;
}

static int se1000_tdm_dummy_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	memset_io(substream->runtime->dma_area, 0, substream->runtime->dma_bytes);
	return 0;
}

static int se1000_tdm_dummy_prepare(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct se1000_aud_dummy *priv = snd_soc_dai_get_drvdata(dai);
	struct aud_dummy_stream_data *stream_priv = priv->stream_priv[substream->stream];
	unsigned long times = msecs_to_jiffies(3000);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int resent = 0;
	int ret = 0;

	stream_priv->period_size = snd_pcm_lib_period_bytes(substream);
	stream_priv->buffer_size = snd_pcm_lib_buffer_bytes(substream);
	stream_priv->boundary = 2 * stream_priv->buffer_size;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		WRITE_ONCE(stream_priv->shm_t->write_flag, 1);

	WRITE_ONCE(stream_priv->count, (stream_priv->count + 1));

	if (priv->internal_loop && stream_priv->count == 1) {
		stream_priv->rpmsg.opcode = OPCODE_AUD_MESG_TOUS_SYNC0 + (substream->stream << 4) + priv->id;
		stream_priv->rpmsg.status = MSG_START;
		/* combine hw param */
		stream_priv->rpmsg.rate_high = RPMSG_INFO_HW_PARAM_RATE_HIGH(runtime->rate);
		stream_priv->rpmsg.rate_low  = RPMSG_INFO_HW_PARAM_RATE_LOW(runtime->rate);
		stream_priv->rpmsg.period    = RPMSG_INFO_HW_PARAM_PERIOD(runtime->period_size);
		stream_priv->rpmsg.chs_and_bits = RPMSG_INFO_HW_PARAM_CHS(runtime->channels) |
									RPMSG_INFO_HW_PARAM_BITS(runtime->sample_bits);
		stream_priv->rpmsg.counts_and_dir = RPMSG_INFO_HW_PARAM_DIR(substream->stream) |
									RPMSG_INFO_HW_PARAM_COUNTS(runtime->periods);
		while (resent++ < AUDIO_SHARE_RESEND_MAX) {
			ret = rpmsg_send(priv->uept, &(stream_priv->rpmsg), sizeof(struct aud_share_rpmsg_info));
			if (ret < 0) {
				pr_err("%s, rpmsg send failed!\n", __func__);
				return -EINVAL;
			}
			ret = wait_event_interruptible_timeout(stream_priv->waitq,
				stream_priv->rpmsg_event, times);
			if (ret > 0) {
				stream_priv->rpmsg_event = false;
				return 0;
			}
		}

		pr_err("%s, audio message sync failed! id=%d\n"
			, __func__, priv->id);
		return -EINVAL;
	}

	return 0;
}

static int se1000_tdm_dummy_trigger(struct snd_pcm_substream *substream, int cmd,
	struct snd_soc_dai *dai)
{
	struct se1000_aud_dummy *priv = snd_soc_dai_get_drvdata(dai);
	bool is_playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	struct aud_dummy_stream_data *stream_priv = priv->stream_priv[substream->stream];

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (is_playback)
			WRITE_ONCE(stream_priv->shm_t->write_flag, 0);
		break;
	default:
		dev_err(dai->dev, "unknown cmd\n");
		return -EINVAL;
	}

	return 0;
}

const struct snd_soc_dai_ops se1000_i2s_dummy_ops = {
	.startup	= se1000_tdm_dummy_startup,
	.shutdown	= se1000_tdm_dummy_shutdown,
	.hw_params	= se1000_tdm_dummy_hw_params,
	.trigger	= se1000_tdm_dummy_trigger,
	.prepare	= se1000_tdm_dummy_prepare,
};

static struct snd_soc_dai_driver se1000_dummy_dai_driver[] = {
	{
		.name = "tdm0",
		.id = DUMMY_TDM0,
		.playback = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_i2s_dummy_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "tdm1",
		.id = DUMMY_TDM1,
		.playback = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.capture = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.ops = &se1000_i2s_dummy_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "tdm2",
		.id = DUMMY_TDM2,
		.playback = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.capture = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.ops = &se1000_i2s_dummy_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "tdm3",
		.id = DUMMY_TDM3,
		.playback = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_i2s_dummy_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "tdm4",
		.id = DUMMY_TDM4,
		.playback = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_i2s_dummy_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "tdm5",
		.id = DUMMY_TDM5,
		.playback = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.capture = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.ops = &se1000_i2s_dummy_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "tdm6",
		.id = DUMMY_TDM6,
		.playback = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.capture = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.ops = &se1000_i2s_dummy_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "tdm7",
		.id = DUMMY_TDM7,
		.playback = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_i2s_dummy_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "tdm8",
		.id = DUMMY_TDM8,
		.playback = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_i2s_dummy_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "tdm9",
		.id = DUMMY_TDM9,
		.playback = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.capture = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.ops = &se1000_i2s_dummy_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "tdm10",
		.id = DUMMY_TDM10,
		.playback = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.capture = {
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.ops = &se1000_i2s_dummy_ops,
		.symmetric_rates = 1,
	},
};

static int shmem_data_avail(struct aud_dummy_stream_data *stream_priv)
{
	uint32_t read_idx, write_idx;
	int avail;

	mutex_lock(&stream_priv->mutex);
	write_idx = READ_ONCE(stream_priv->shm_t->write_idx);
	read_idx = READ_ONCE(stream_priv->shm_t->read_idx);

	if (stream_priv->substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		avail = read_idx + stream_priv->buffer_size - write_idx;
			if (avail < 0)
				avail += stream_priv->boundary;
			else if (avail >= stream_priv->boundary)
				avail -= stream_priv->boundary;
	} else {
			avail = write_idx - read_idx;
			if (avail < 0)
				avail += stream_priv->boundary;
			else if (avail == 0)
				avail = stream_priv->buffer_size;
	}
	mutex_unlock(&stream_priv->mutex);

	return avail;
}

static int shmem_writer_update(struct aud_dummy_stream_data *stream_priv, unsigned long size)
{
	int ret = 0;
	uint32_t write_idx;

	mutex_lock(&stream_priv->mutex);
	write_idx = READ_ONCE(stream_priv->shm_t->write_idx);

	write_idx = write_idx + size;
	if (write_idx >= stream_priv->boundary)
		write_idx = 0;
	WRITE_ONCE(stream_priv->shm_t->write_idx, write_idx);
	barrier();

	mutex_unlock(&stream_priv->mutex);

	return ret;
}

static int shmem_reader_update(struct aud_dummy_stream_data *stream_priv, unsigned long size)
{
	int ret = 0;
	uint32_t read_idx;

	mutex_lock(&stream_priv->mutex);

	read_idx = READ_ONCE(stream_priv->shm_t->read_idx);

	read_idx = read_idx + size;
	if (read_idx >= stream_priv->boundary)
		read_idx = 0;
	WRITE_ONCE(stream_priv->shm_t->read_idx, read_idx);
	barrier();

	mutex_unlock(&stream_priv->mutex);

	return ret;
}

static const struct snd_soc_component_driver se1000_dummy_component_driver = {
	.name = "se1000-dummy-common",
};

snd_pcm_uframes_t se1000_component_dummy_pointer(struct snd_soc_component *component,
				      struct snd_pcm_substream *substream)
{
	struct se1000_aud_dummy *priv = snd_soc_component_get_drvdata(component);
	int pcm_ptr_bytes;

	pcm_ptr_bytes = READ_ONCE(priv->stream_priv[substream->stream]->pos);

	return bytes_to_frames(substream->runtime, pcm_ptr_bytes);
}

static int se1000_component_dummy_new(struct snd_soc_component *component,
		    struct snd_soc_pcm_runtime *rtd)
{
	struct se1000_aud_dummy *priv = snd_soc_component_get_drvdata(component);
	struct snd_card *card = rtd->card->snd_card;
	int i = 0;

	for (i = 0; i < 2; i++) {
		priv->stream_priv[i]->dma_buf = kzalloc(sizeof(struct snd_dma_buffer), GFP_KERNEL);
		priv->stream_priv[i]->dma_buf->dev.type = SNDRV_DMA_TYPE_DEV;
		priv->stream_priv[i]->dma_buf->dev.dev = card->dev;

		priv->stream_priv[i]->dma_buf->area = (unsigned char *)priv->stream_priv[i]->shm_t->buffer;
		priv->stream_priv[i]->dma_buf->addr = priv->stream_priv[i]->shm_addr_phy;
		priv->stream_priv[i]->dma_buf->bytes = se1000_dummy_hardware.buffer_bytes_max;
	}
	return 0;
}

static int se1000_component_copy_user(struct snd_soc_component *component,
		struct snd_pcm_substream *substream, int channel,
		unsigned long hwoff, void __user *buf, unsigned long bytes)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct se1000_aud_dummy *priv = snd_soc_component_get_drvdata(component);
	bool is_playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	struct aud_dummy_stream_data *stream_priv = priv->stream_priv[substream->stream];
	void *dma_ptr = runtime->dma_area + hwoff +
			channel * (runtime->dma_bytes / runtime->channels);
	uint32_t read_idx, write_idx;
	uint32_t avail = 0;

	write_idx = READ_ONCE(stream_priv->shm_t->write_idx);
	read_idx = READ_ONCE(stream_priv->shm_t->read_idx);
	avail = shmem_data_avail(stream_priv);

	if (is_playback) {
		if (write_idx == stream_priv->buffer_size)
			stream_priv->dma_ofs = stream_priv->buffer_size;
		else if (write_idx == stream_priv->boundary)
			stream_priv->dma_ofs = 0;

		dma_ptr = dma_ptr + stream_priv->dma_ofs;

		if (copy_from_user(dma_ptr, buf, bytes))
			return -EFAULT;

		if ((bytes + hwoff) == stream_priv->buffer_size) {
			if ((write_idx + bytes) % stream_priv->buffer_size) {
				if (bytes % stream_priv->period_size)
					bytes = bytes + (stream_priv->period_size - bytes % stream_priv->period_size);
			}
		}

		if (write_idx >= stream_priv->boundary)
			WRITE_ONCE(stream_priv->shm_t->write_idx, 0);

		shmem_writer_update(stream_priv, bytes);
	} else {
		if (read_idx >= stream_priv->boundary)
			WRITE_ONCE(stream_priv->shm_t->read_idx, 0);

		if (read_idx == stream_priv->buffer_size)
			stream_priv->dma_ofs = stream_priv->buffer_size;
		else if (read_idx == stream_priv->boundary)
			stream_priv->dma_ofs = 0;

		dma_ptr = dma_ptr + stream_priv->dma_ofs;

		if (stream_priv->cap_done) {
			avail = shmem_data_avail(stream_priv);
			if (avail > stream_priv->period_size) {
				stream_priv->pos += stream_priv->period_size;
				if (stream_priv->pos >= stream_priv->buffer_size)
					stream_priv->pos = 0;
				snd_pcm_period_elapsed(substream);
			} else {
				bytes = avail;
				if (copy_to_user(buf, dma_ptr, bytes))
					return -EFAULT;
				shmem_reader_update(stream_priv, bytes);

				snd_pcm_release_substream(substream);
				return 0;
			}
		}

		shmem_reader_update(stream_priv, bytes);
		if (copy_to_user(buf, dma_ptr, bytes))
			return -EFAULT;

	}

	return 0;
}

int se1000_component_dummy_open(struct snd_soc_component *component,
		struct snd_pcm_substream *substream)
{
	int ret = 0;

	ret = snd_soc_set_runtime_hwparams(substream, &se1000_dummy_hardware);
	return 0;
}

int se1000_component_dummy_mmap(struct snd_soc_component *component,
		struct snd_pcm_substream *substream, struct vm_area_struct *vma)
{
	return remap_pfn_range(vma, vma->vm_start,
		       substream->dma_buffer.addr >> PAGE_SHIFT,
		       vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

const struct snd_soc_component_driver se1000_pcm_dummy_platform = {
	.name		= "se1000-dummy-platform",
	.probe_order	= SND_SOC_COMP_ORDER_LATE,
	.open = se1000_component_dummy_open,
	.pointer	= se1000_component_dummy_pointer,
	.pcm_construct	= se1000_component_dummy_new,
	.copy_user	= se1000_component_copy_user,
	.mmap = se1000_component_dummy_mmap,
};

static int snd_soc_register_platform(struct device *dev,
		const struct snd_soc_component_driver *component_driver,
		struct snd_soc_dai_driver *dai_drv,
		int num_dai)
{
	struct snd_soc_component *component;
	int ret;

	component = devm_kzalloc(dev, sizeof(*component), GFP_KERNEL);
	if (!component)
		return -ENOMEM;

#ifdef CONFIG_DEBUG_FS
	component->debugfs_prefix = "share";
#endif

	ret = snd_soc_component_initialize(component, component_driver, dev);
	if (ret < 0)
		return ret;

	return snd_soc_add_component(component, dai_drv, num_dai);
}

static int se1000_pcm_dummy_probe(struct platform_device *pdev)
{
	struct device *dev;
	struct se1000_aud_dummy *priv;
	struct aud_dummy_stream_data stream_priv;
	struct device_node *node;
	struct resource res;
	int ret = 0;
	int offset = 0, i = 0;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	memset(priv, 0, sizeof(*priv));
	dev = &pdev->dev;
	priv->dev = dev;

	priv->stream_priv[SNDRV_PCM_STREAM_PLAYBACK] = devm_kzalloc(&pdev->dev, sizeof(stream_priv), GFP_KERNEL);
	if (!priv->stream_priv[SNDRV_PCM_STREAM_PLAYBACK])
		return -ENOMEM;

	priv->stream_priv[SNDRV_PCM_STREAM_CAPTURE] = devm_kzalloc(&pdev->dev, sizeof(stream_priv), GFP_KERNEL);
	if (!priv->stream_priv[SNDRV_PCM_STREAM_CAPTURE])
		return -ENOMEM;

	ret = of_property_read_u32(dev->of_node, "id", &priv->id);
	if (ret < 0)
		return ret;

	if (priv->id >= MAX_SHARE_SESSIONS)
		return -ENOMEM;

	node = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!node) {
		dev_err(dev, "no memory-region specified\n");
		return -EINVAL;
	}

	ret = of_address_to_resource(node, 0, &res);
	if (ret) {
		dev_err(dev, "failed to get reserved region address\n");
		of_node_put(node);
		return ret;
	}

	of_node_put(node);
	stream_priv.mem_size = resource_size(&res);
	stream_priv.shm_addr_phy = res.start;
	stream_priv.shm_addr_virt = (unsigned char *)ioremap_wc(stream_priv.shm_addr_phy, stream_priv.mem_size);

	if (!(stream_priv.shm_addr_virt)) {
		dev_err(&pdev->dev, "audio ioremap failed!\n");
		return -ENOMEM;
	}
	offset = sizeof(struct shmem_data);

	if (of_property_read_u32(dev->of_node, "internal_loop", &priv->internal_loop) != 0) {
		dev_err(dev, "%s: internal_loop missing in DT node\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < 2; i++) {
		*(priv->stream_priv[i]) = stream_priv; /* get the same memory info */
		/* playback use the 0~6 shm id, capture use the 7~13 shm id */
		priv->stream_priv[i]->shm_addr_virt = priv->stream_priv[i]->shm_addr_virt + \
												offset * (priv->id + i * MAX_SHARE_SESSIONS);
		priv->stream_priv[i]->shm_t = (struct shmem_data *)priv->stream_priv[i]->shm_addr_virt;

		WRITE_ONCE(priv->stream_priv[i]->shm_t->write_idx, 0);
		WRITE_ONCE(priv->stream_priv[i]->shm_t->read_idx, 0);
		WRITE_ONCE(priv->stream_priv[i]->shm_t->write_flag, 0);
		mutex_init(&priv->stream_priv[i]->mutex);
		priv->stream_priv[i]->rpmsg_event = false;
		priv->stream_priv[i]->count = 0;
		init_waitqueue_head(&priv->stream_priv[i]->waitq);
	}
	platform_set_drvdata(pdev, priv);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev))
		goto err_pm;
	pm_runtime_get_sync(&pdev->dev);

	ret = snd_soc_register_platform(dev, &se1000_pcm_dummy_platform, NULL, 0);
	if (ret) {
		dev_warn(dev, "err_platform\n");
		goto err_platform;
	}
	ret = se1000_tdm_dummy_rpmsg_init(dev, priv);
	if (ret) {
		dev_warn(dev, "init rpmsg failed\n");
		goto err_platform;
	}

	ret = devm_snd_soc_register_component(dev, &se1000_dummy_component_driver,
		se1000_dummy_dai_driver, ARRAY_SIZE(se1000_dummy_dai_driver));
	if (ret) {
		dev_warn(dev, "err_dai_component\n");
		goto err_dai_component;
	}

	return 0;
err_dai_component:
	snd_soc_unregister_component(&pdev->dev);
err_platform:
	pm_runtime_put_sync(dev);
err_pm:
	pm_runtime_disable(dev);

	return ret;
}

static int se1000_pcm_dummy_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static const struct of_device_id se1000_pcm_dummy_match[] = {
	{ .compatible = "siengine,se1000-dummy", },
	{},
};
MODULE_DEVICE_TABLE(of, se1000_pcm_dummy_match);

static struct platform_driver se1000_pcm_dummy_driver = {
	.driver = {
		 .name = "se1000-dummy-audio",
		 .of_match_table = se1000_pcm_dummy_match,
	},
	.probe = se1000_pcm_dummy_probe,
	.remove = se1000_pcm_dummy_remove,
};

module_platform_driver(se1000_pcm_dummy_driver);
MODULE_DESCRIPTION("Siengine SE1000 ALSA SoC platform driver");
MODULE_AUTHOR("jian.xu <jian.xu@siengine.com>");
MODULE_LICENSE("GPL v2");

