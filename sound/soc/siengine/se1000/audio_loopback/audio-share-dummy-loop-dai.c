/* SPDX-License-Identifier: GPL-2.0 */
/*
 * sound/soc/siengine/audio_loopback
 * Copyright (C) 2023 SiEngine Technology Co., Ltd. All rights reserved
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
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

#include "../audioshare/audio-share-dummy.h"
#include "audio-share-dummy-loop-ipc.h"

static int shmem_data_avail(struct aud_dummy_stream_data *stream_priv);

static const struct snd_pcm_hardware se1000_dummy_loop_hardware = {
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

static void dummy_audio_share_rpmsg_send(struct se1000_aud_dummy *priv, uint32_t opcode, uint32_t offset)
{
	dummy_rpmsg_payload_t payload = {opcode, offset};

	rpmsg_send(priv->ept, &payload, sizeof(dummy_rpmsg_payload_t));
	pr_debug("%s: send dummy share msg session %d opcode %x offset %d\n", \
					__func__, priv->id, opcode, offset);
}

static int dummy_audio_share_rpmsg_cb(struct rpmsg_device *rpmsg_dev, void *data, int len, void *priv, u32 src)
{
	dummy_rpmsg_payload_t *payload = (dummy_rpmsg_payload_t *)data;

	switch (payload->opcode) {
	case OPCODE_AUD_SHARE_DATA_WRITE_DONE:
	case OPCODE_AUD_SHARE_DATA_READ_DONE:
		/* nothing to do */
		break;
	default:
		break;
	}
	return 0;
}

static int dummy_audio_share_rpmsg_init(struct device *dev, struct se1000_aud_dummy *priv)
{
	struct rpmsg_device *rpdev = NULL;
	uint32_t rpmsg_type = 0;
	uint32_t rpmsg_id = 0;

	struct rpmsg_channel_info ept_info = {
		.name = "aud_dummy_share",
	};

	rpdev = find_rpmsg_device_by_phandle(dev->of_node);
	if (!rpdev) {
		dev_err(dev, "find_rpmsg_device_by_phandle failed\n");
		return -EINVAL;
	}

	if (of_property_read_u32(dev->of_node, "se-share-type", &rpmsg_type) != 0) {
		dev_err(dev, "%s: se-share-type missing in DT node\n", __func__);
		return -EINVAL;
	}
	rpmsg_id = AUD_SHR_CP2AP_RPMSG_ID_0 | (rpmsg_type << 4) | priv->id;
	ept_info.src = rpmsg_id;
	ept_info.dst = rpmsg_id;

	priv->ept = rpmsg_create_ept(rpdev, dummy_audio_share_rpmsg_cb, priv, ept_info);
	if (!priv->ept) {
		dev_err(dev, "dummy audio rpmsg_create_ept failed\n");
		return -EPROBE_DEFER;
	}
	return 0;
}

static int se1000_share_dummy_loop_startup(struct snd_pcm_substream *substream,
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

static void se1000_share_dummy_loop_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct se1000_aud_dummy *priv = snd_soc_dai_get_drvdata(dai);
	struct aud_dummy_stream_data *stream_priv = priv->stream_priv[substream->stream];

	dev_dbg(dai->dev, "%s\n", __func__);
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		WRITE_ONCE(stream_priv->shm_t->read_idx, 0);
		WRITE_ONCE(stream_priv->shm_t->write_idx, 0);
	}

	stream_priv->pos = 0;
	stream_priv->cap_done = false;
	stream_priv->substream = NULL;
}

static int se1000_share_dummy_loop_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	dev_dbg(dai->dev, "%s\n", __func__);
	memset_io(substream->runtime->dma_area, 0, substream->runtime->dma_bytes);
	return 0;
}

static int se1000_share_dummy_loop_prepare(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct se1000_aud_dummy *priv = snd_soc_dai_get_drvdata(dai);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aud_dummy_stream_data *stream_priv = priv->stream_priv[substream->stream];

	dev_dbg(dai->dev, "%s\n", __func__);

	/* update hostless dummy loopback sw parameters */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		WRITE_ONCE(stream_priv->shm_t->write_flag, 1);
		runtime->start_threshold = runtime->buffer_size;
	} else
		runtime->start_threshold = runtime->period_size;

	runtime->stop_threshold = __INT32_MAX__;
	stream_priv->period_size = snd_pcm_lib_period_bytes(substream);
	stream_priv->buffer_size = snd_pcm_lib_buffer_bytes(substream);
	stream_priv->boundary = 2 * stream_priv->buffer_size;
	return 0;
}

static int se1000_share_dummy_loop_trigger(struct snd_pcm_substream *substream, int cmd,
	struct snd_soc_dai *dai)
{
	struct se1000_aud_dummy *priv = snd_soc_dai_get_drvdata(dai);
	bool is_playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	struct aud_dummy_stream_data *stream_priv = priv->stream_priv[substream->stream];

	dev_dbg(dai->dev, "%s : cmd %d\n", __func__, cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			stream_priv->pos = snd_pcm_lib_period_bytes(substream);
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

const static struct snd_soc_dai_ops se1000_share_dummy_loop_ops = {
	.startup	= se1000_share_dummy_loop_startup,
	.shutdown	= se1000_share_dummy_loop_shutdown,
	.hw_params	= se1000_share_dummy_loop_hw_params,
	.trigger	= se1000_share_dummy_loop_trigger,
	.prepare	= se1000_share_dummy_loop_prepare,
};

static struct snd_soc_dai_driver se1000_dummy_loop_dai_driver[] = {
	{
		.name = "share-loop-dummy0",
		.id = SE1000_HL_SHR_DUMMY0,
		.playback = {
			.stream_name = "loop dummy 0 playback",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "loop dummy 0 capture",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_share_dummy_loop_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "share-loop-dummy1",
		.id = SE1000_HL_SHR_DUMMY1,
		.playback = {
			.stream_name = "loop dummy 1 playback",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "loop dummy 1 capture",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_share_dummy_loop_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "share-loop-dummy2",
		.id = SE1000_HL_SHR_DUMMY2,
		.playback = {
			.stream_name = "loop dummy 2 playback",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "loop dummy 2 capture",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_share_dummy_loop_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "share-loop-dummy3",
		.id = SE1000_HL_SHR_DUMMY3,
		.playback = {
			.stream_name = "loop dummy 3 playback",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "loop dummy 3 capture",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_share_dummy_loop_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "share-loop-dummy4",
		.id = SE1000_HL_SHR_DUMMY4,
		.playback = {
			.stream_name = "loop dummy 4 playback",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "loop dummy 4 capture",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_share_dummy_loop_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "share-loop-dummy5",
		.id = SE1000_HL_SHR_DUMMY5,
		.playback = {
			.stream_name = "loop dummy 5 playback",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "loop dummy 5 capture",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_share_dummy_loop_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "share-loop-dummy6",
		.id = SE1000_HL_SHR_DUMMY6,
		.playback = {
			.stream_name = "loop dummy 6 playback",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "loop dummy 6 capture",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_share_dummy_loop_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "share-loop-dummy7",
		.id = SE1000_HL_SHR_DUMMY7,
		.playback = {
			.stream_name = "loop dummy 7 playback",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "loop dummy 7 capture",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_share_dummy_loop_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "share-loop-dummy8",
		.id = SE1000_HL_SHR_DUMMY8,
		.playback = {
			.stream_name = "loop dummy 8 playback",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "loop dummy 8 capture",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_share_dummy_loop_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "share-loop-dummy9",
		.id = SE1000_HL_SHR_DUMMY9,
		.playback = {
			.stream_name = "loop dummy 9 playback",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "loop dummy 9 capture",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_share_dummy_loop_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "share-loop-dummy10",
		.id = SE1000_HL_SHR_DUMMY10,
		.playback = {
			.stream_name = "loop dummy 10 playback",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.stream_name = "loop dummy 10 capture",
			.channels_min = MIN_SHARE_CHANNELS,
			.channels_max = MAX_SHARE_CHANNELS,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_share_dummy_loop_ops,
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
			avail += stream_priv->buffer_size;
	}
	mutex_unlock(&stream_priv->mutex);
	pr_debug("%s: avail %d write_idx %d read_idx %d\n", __func__, avail, write_idx, read_idx);

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

static snd_pcm_uframes_t se1000_component_dummy_loop_pointer(struct snd_soc_component *component,
				      struct snd_pcm_substream *substream)
{
	struct se1000_aud_dummy *priv = snd_soc_component_get_drvdata(component);
	int pcm_ptr_bytes;

	pcm_ptr_bytes = READ_ONCE(priv->stream_priv[substream->stream]->pos);

	return bytes_to_frames(substream->runtime, pcm_ptr_bytes);
}

static int se1000_component_dummy_loop_new(struct snd_soc_component *component,
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
		priv->stream_priv[i]->dma_buf->bytes = se1000_dummy_loop_hardware.buffer_bytes_max;
	}
	return 0;
}

static int wait_for_share_avail(struct aud_dummy_stream_data *stream_priv)
{
	uint32_t avail = 0, try_times = 0;
	int ret = 1;
	struct snd_pcm_runtime *runtime = stream_priv->substream->runtime;
	long period = runtime->period_size * 1000 / runtime->rate;

	while (avail < stream_priv->period_size) {
		usleep_range(THREAD_WAIT * 1000, THREAD_WAIT * 1000); /* check every 1ms */
		avail = shmem_data_avail(stream_priv);
		try_times++;
		if (try_times >= period) { /* timeout one period */
			ret = 0;
			break;
		}
	}
	return ret;
}

static int se1000_component_dummy_copy_kernel(struct snd_soc_component *component,
		struct snd_pcm_substream *substream, int channel,
		unsigned long hwoff, void *buf, unsigned long bytes)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct se1000_aud_dummy *priv = snd_soc_component_get_drvdata(component);
	bool is_playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	struct aud_dummy_stream_data *stream_priv = priv->stream_priv[substream->stream];
	void *dma_ptr = runtime->dma_area + hwoff +
			channel * (runtime->dma_bytes / runtime->channels);
	uint32_t read_idx, write_idx, write_flag;
	uint32_t avail = 0;

	write_idx = READ_ONCE(stream_priv->shm_t->write_idx);
	read_idx = READ_ONCE(stream_priv->shm_t->read_idx);
	write_flag = READ_ONCE(stream_priv->shm_t->write_flag);
	avail = shmem_data_avail(stream_priv);

	if (is_playback) {

		if (write_idx == stream_priv->buffer_size)
			stream_priv->dma_ofs = stream_priv->buffer_size;
		else if (write_idx == stream_priv->boundary)
			stream_priv->dma_ofs = 0;

		dma_ptr = dma_ptr + stream_priv->dma_ofs;
		memcpy(dma_ptr, buf, bytes);
		dev_dbg(component->dev, "%s: write bytes: %ld\n", __func__, bytes);
	} else {
		if (avail < bytes) {
			if (wait_for_share_avail(stream_priv) == 0) {
				dev_dbg(component->dev, "%s: wait timeout, supposed write stopped\n", __func__);
				stream_priv->cap_done = true;
			}
		}

		if (read_idx == stream_priv->buffer_size)
			stream_priv->dma_ofs = stream_priv->buffer_size;
		else if (read_idx == stream_priv->boundary)
			stream_priv->dma_ofs = 0;

		dma_ptr = dma_ptr + stream_priv->dma_ofs;
		if (stream_priv->cap_done) {
			avail = shmem_data_avail(stream_priv);
			bytes = avail;
			memcpy(buf, dma_ptr, bytes);
			shmem_reader_update(stream_priv, bytes);
			snd_pcm_release_substream(substream);
			return 0;
		}
		memcpy(buf, dma_ptr, bytes);
		dev_dbg(component->dev, "%s: read bytes: %ld\n", __func__, bytes);
	}

	return 0;
}

static int se1000_component_dummy_loop_open(struct snd_soc_component *component,
		struct snd_pcm_substream *substream)
{
	dev_dbg(component->dev, "%s\n", __func__);
	snd_soc_set_runtime_hwparams(substream, &se1000_dummy_loop_hardware);
	return 0;
}

static void se1000_component_dummy_loop_hw_update(struct snd_soc_component *component,
				      struct snd_pcm_substream *substream, int bytes)
{
	struct se1000_aud_dummy *priv = snd_soc_component_get_drvdata(component);
	struct aud_dummy_stream_data *stream_priv = priv->stream_priv[substream->stream];

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		/* update read idx after write done */
		shmem_reader_update(stream_priv, bytes);
		dummy_audio_share_rpmsg_send(priv, OPCODE_AUD_SHARE_DATA_READ_DONE, bytes);
	} else {
		/* update write idx after write done */
		shmem_writer_update(stream_priv, bytes);
		dummy_audio_share_rpmsg_send(priv, OPCODE_AUD_SHARE_DATA_WRITE_DONE, bytes);
	}
	stream_priv->pos += bytes;
	if (stream_priv->pos >= stream_priv->buffer_size)
		stream_priv->pos = 0;

	snd_pcm_period_elapsed(stream_priv->substream);
	dev_dbg(component->dev, "%s : bytes %d, pos %d\n", __func__, bytes, stream_priv->pos);
}

static void se1000_component_dummy_loop_ready_status(struct snd_soc_component *component,
				      struct snd_pcm_substream *substream, int *status)
{
	struct se1000_aud_dummy *priv = snd_soc_component_get_drvdata(component);
	struct aud_dummy_stream_data *stream_priv = priv->stream_priv[substream->stream];
	uint32_t write_idx, write_flag = 0;

	write_flag = READ_ONCE(stream_priv->shm_t->write_flag);
	write_idx = READ_ONCE(stream_priv->shm_t->write_idx);
	/* wait for pair ep write buffer full */
	if ((write_flag == 1) && (write_idx == stream_priv->buffer_size))
		*status = 1;
	else
		*status = 0;

	dev_dbg(component->dev, "%s : status %d, write_flag %d, write_idx %d\n", __func__, *status, write_flag, write_idx);
}

static int se1000_component_dummy_loop_ioctl(struct snd_soc_component *component,
		     struct snd_pcm_substream *substream,
		     unsigned int cmd, void *arg)
{
	dev_dbg(component->dev, "%s : cmd %x\n", __func__, cmd);

	switch (cmd) {
	case SNDRV_COMP_IOCTL_LOOP_DAI_HW_FORWARD:
		se1000_component_dummy_loop_hw_update(component, substream, *(int *)arg);
		break;
	case SNDRV_COMP_IOCTL_LOOP_DAI_READY_STATUS:
		se1000_component_dummy_loop_ready_status(component, substream, (int *)arg);
		break;
	default:
		break;
	}

	return 0;
}

static const struct snd_soc_component_driver se1000_pcm_dummy_loop_platform = {
	.name           = "se1000-dummy-platform-loop",
	.probe_order    = SND_SOC_COMP_ORDER_LATE,
	.open           = se1000_component_dummy_loop_open,
	.pointer        = se1000_component_dummy_loop_pointer,
	.pcm_construct  = se1000_component_dummy_loop_new,
	.copy_kernel    = se1000_component_dummy_copy_kernel,
	.ioctl          = se1000_component_dummy_loop_ioctl,
};

static int se1000_pcm_dummy_loop_probe(struct platform_device *pdev)
{
	struct device *dev;
	struct se1000_aud_dummy *priv;
	struct aud_dummy_stream_data stream_priv;
	struct device_node *node;
	struct resource res;
	int ret = 0;
	int offset = 0, i = 0;

	dev_dbg(&pdev->dev, "%s: dev name %s\n", __func__, dev_name(&pdev->dev));
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
		return -EINVAL;

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

	for (i = 0; i < 2; i++) {
		*(priv->stream_priv[i]) = stream_priv; /* get the same memory info */
		/* capture use the 0~6 shm id, playback use the 7~13 shm id, to match with paired dummy device */
		priv->stream_priv[i]->shm_addr_virt = priv->stream_priv[i]->shm_addr_virt + \
					offset * (priv->id + (SNDRV_PCM_STREAM_CAPTURE - i) * MAX_SHARE_SESSIONS);
		priv->stream_priv[i]->shm_t = (struct shmem_data *)priv->stream_priv[i]->shm_addr_virt;

		WRITE_ONCE(priv->stream_priv[i]->shm_t->write_idx, 0);
		WRITE_ONCE(priv->stream_priv[i]->shm_t->read_idx, 0);
		WRITE_ONCE(priv->stream_priv[i]->shm_t->write_flag, 0);
		mutex_init(&priv->stream_priv[i]->mutex);
	}
	platform_set_drvdata(pdev, priv);

	ret = dummy_audio_share_rpmsg_init(dev, priv);
	if (ret) {
		dev_err(dev, " %s: init rpmsg failed\n", __func__);
		return ret;
	}

	ret = devm_snd_soc_register_component(dev, &se1000_pcm_dummy_loop_platform,
		se1000_dummy_loop_dai_driver, ARRAY_SIZE(se1000_dummy_loop_dai_driver));
	if (ret) {
		dev_err(dev, " %s: register dai component failed\n", __func__);
		goto err_dai_component;
	}

	return 0;
err_dai_component:
	snd_soc_unregister_component(&pdev->dev);

	return ret;
}

static int se1000_pcm_dummy_loop_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static const struct of_device_id se1000_pcm_dummy_loop_match[] = {
	{ .compatible = "siengine,se1000-share-loop-dai", },
};
MODULE_DEVICE_TABLE(of, se1000_pcm_dummy_loop_match);

static struct platform_driver se1000_pcm_dummy_driver_loop = {
	.driver = {
		 .name = "se1000-share-loop-dai",
		 .of_match_table = se1000_pcm_dummy_loop_match,
	},
	.probe = se1000_pcm_dummy_loop_probe,
	.remove = se1000_pcm_dummy_loop_remove,
};

module_platform_driver(se1000_pcm_dummy_driver_loop);
MODULE_DESCRIPTION("Siengine SE1000 ALSA SoC Hostless Loopback DAI Driver");
MODULE_AUTHOR("ying.chen <ying.chen@siengine.com>");
MODULE_LICENSE("GPL v2");

