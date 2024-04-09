/*
 * SPDX-License-Identifier: GPL-2.0.
 * sound/soc/siengine/audio_tdm_mlp
 * Copyright (C) 2022-2023 SiEngine or its affiliates
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/dmaengine.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <sound/dmaengine_pcm.h>
#include <linux/kthread.h>
#include <asm/current.h>
#include <linux/delay.h>

#include "../se1000-aud-i2s.h"
#include "se1000-multi-tdm.h"

static inline int audio_data_process(struct se1000_multi_tdm *psmt,
		struct snd_pcm_substream *substream, unsigned int dma_hwoff);

static inline void se1000_pcm_dma_complete(void *arg, int stream)
{
	struct se1000_multi_tdm *psmt = arg;
	struct se1000_dma_runtime_data *prtd = psmt->prtd;
	int dma_hwoff = 0;
	struct snd_pcm_substream *substream;
	int i = 0;
	size_t period = 0;
	bool period_multipled;
	bool *stream_state = (stream == SNDRV_PCM_STREAM_PLAYBACK ? psmt->play : psmt->capture);

	/* get current dma hw pos */
	dma_hwoff = READ_ONCE(psmt->dma_hwoff[stream]);

	for (i = 0; i < PCM_NODE_MAX; i++) {
		if (stream_state[i]) {
			substream = psmt->substream[i][stream];
			/* copy buffer from FE */
			audio_data_process(psmt, substream, dma_hwoff);
			/* update FE hw pos based on fe min period byte */
			prtd->pos[i][stream] += READ_ONCE(psmt->min_period_size[i][stream]);
			if (prtd->pos[i][stream] >= snd_pcm_lib_buffer_bytes(substream))
				WRITE_ONCE(prtd->pos[i][stream], 0);

			period = snd_pcm_lib_period_bytes(substream);
			period_multipled = (substream->runtime->period_size % prtd->min_period == 0);
			/* elapsed based on rt period to avoid wake up app frequently if period is multiple to min size */
			if (prtd->pos[i][stream] % (period_multipled ? period : prtd->pos[i][stream]) == 0)
				snd_pcm_period_elapsed(substream);
		}
	}

	/* update dma hw pos */
	dma_hwoff += prtd->period_size;
	WRITE_ONCE(psmt->dma_hwoff[stream], dma_hwoff);

	if (dma_hwoff >= prtd->prealloc_buffer)
		WRITE_ONCE(psmt->dma_hwoff[stream], 0);
}

static void se1000_pcm_dma_complete_play(void *arg)
{
	se1000_pcm_dma_complete(arg, SNDRV_PCM_STREAM_PLAYBACK);
}

static void se1000_pcm_dma_complete_cap(void *arg)
{
	se1000_pcm_dma_complete(arg, SNDRV_PCM_STREAM_CAPTURE);
}

static int se1000_dma_pcm_hw_params(
		struct snd_soc_component *component,
			struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct se1000_multi_tdm *psmt = snd_soc_component_get_drvdata(component);
	struct se1000_dma_runtime_data *prtd = psmt->prtd;

	struct dma_chan *chan = prtd->chan[substream->stream];
	int (*prepare_slave_config)(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params,
			struct dma_slave_config *slave_config);
	struct dma_slave_config slave_config;
	int ret;

	if (READ_ONCE(prtd->active[substream->stream]) > 0) {
		return 0;
	}

	memset(&slave_config, 0, sizeof(slave_config));

	if (prtd->config && prtd->config->prepare_slave_config)
		prepare_slave_config = prtd->config->prepare_slave_config;
	else
		prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config;

	if (prepare_slave_config) {
		ret = prepare_slave_config(substream, params, &slave_config);
		if (ret)
			return ret;

		ret = dmaengine_slave_config(chan, &slave_config);
		if (ret)
			return ret;
	}

	return 0;
}

static int se1000_dma_pcm_open(struct snd_soc_component *component,
			      struct snd_pcm_substream *substream)
{
	struct se1000_multi_tdm *psmt = snd_soc_component_get_drvdata(component);
	struct se1000_dma_runtime_data *prtd = psmt->prtd;
	struct dma_chan *chan = prtd->chan[substream->stream];
	int ret = 0;

	ret = snd_soc_set_runtime_hwparams(substream, prtd->config->pcm_hardware);
	if (ret)
		return ret;

	if (!chan)
		return -ENXIO;

	ret = snd_pcm_hw_constraint_integer(substream->runtime,
					SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	substream->runtime->private_data = psmt;

	return 0;
}

static int se1000_dma_pcm_close(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream)
{
	struct se1000_multi_tdm *psmt = snd_soc_component_get_drvdata(component);
	struct se1000_dma_runtime_data *prtd = psmt->prtd;
	int stream = substream->stream;

	spin_lock(&psmt->splt);
	if (READ_ONCE(prtd->active[stream]) > 0) {
		spin_unlock(&psmt->splt);
		return 0;
	}
	spin_unlock(&psmt->splt);
	prtd->desc[stream] = NULL;
	dmaengine_synchronize(prtd->chan[substream->stream]);
	return 0;
}

static int se1000_pcm_prepare_and_submit(struct snd_pcm_substream *substream,
	struct snd_soc_component *component)
{
	struct se1000_multi_tdm *psmt = snd_soc_component_get_drvdata(component);
	struct se1000_dma_runtime_data *prtd = psmt->prtd;
	bool stream = substream->stream;

	struct dma_chan *chan = prtd->chan[substream->stream];
	enum dma_transfer_direction direction;
	unsigned long flags = DMA_CTRL_ACK;

	direction = snd_pcm_substream_to_dma_direction(substream);

	if (!substream->runtime->no_period_wakeup)
		flags |= DMA_PREP_INTERRUPT;

	prtd->desc[stream] = dmaengine_prep_dma_cyclic(chan,
		psmt->dma_buf[stream]->addr, prtd->prealloc_buffer,
		prtd->period_size, direction, flags);
	if (!prtd->desc[stream])
		return -ENOMEM;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		prtd->desc[stream]->callback = se1000_pcm_dma_complete_play;
	} else {
		prtd->desc[stream]->callback = se1000_pcm_dma_complete_cap;
	}

	prtd->desc[stream]->callback_param = psmt;
	dmaengine_submit(prtd->desc[stream]);
	return 0;
}

static int se1000_dma_pcm_trigger(struct snd_soc_component *component,
				 struct snd_pcm_substream *substream, int cmd)
{
	struct se1000_multi_tdm *psmt = snd_soc_component_get_drvdata(component);
	struct se1000_dma_runtime_data *prtd = psmt->prtd;
	struct dma_chan *dma_chan = prtd->chan[substream->stream];
	int ret;
	int count;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		spin_lock(&psmt->splt);
		count = READ_ONCE(prtd->active[substream->stream]) + 1;
		WRITE_ONCE(prtd->active[substream->stream], count);
		if (count > 1) {
			spin_unlock(&psmt->splt);
			return 0;
		}
		spin_unlock(&psmt->splt);
		ret = se1000_pcm_prepare_and_submit(substream, component);
		if (ret)
			return ret;
		dma_async_issue_pending(dma_chan);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dmaengine_resume(dma_chan);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dmaengine_pause(dma_chan);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		spin_lock(&psmt->splt);
		count = READ_ONCE(prtd->active[substream->stream]);
		if (count == 0) {
			spin_unlock(&psmt->splt);
			return 0;
		}
		count--;
		WRITE_ONCE(prtd->active[substream->stream], count);
		if (count > 0) {
			spin_unlock(&psmt->splt);
			return 0;
		}
		psmt->dma_hwoff[substream->stream] = 0;
		spin_unlock(&psmt->splt);
		dmaengine_terminate_async(dma_chan);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

void se1000_dma_pcm_destruct(struct snd_soc_component *component,
			struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	int i;

	for_each_pcm_streams(i) {
		substream = pcm->streams[i].substream;
		if (!substream)
			continue;

		snd_dma_free_pages(&substream->dma_buffer);
		substream->dma_buffer.area = NULL;
		substream->dma_buffer.addr = 0;
	}
}

static snd_pcm_uframes_t se1000_dma_pcm_pointer(
	struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *fe = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *dai = asoc_rtd_to_cpu(fe, 0);
	struct se1000_multi_tdm *psmt = snd_soc_component_get_drvdata(component);
	struct se1000_dma_runtime_data *prtd = psmt->prtd;
	int stream = substream->stream;

	return bytes_to_frames(substream->runtime, prtd->pos[dai->id][stream]);
}

static inline int audio_data_process(struct se1000_multi_tdm *psmt,
		struct snd_pcm_substream *substream, unsigned int dma_hwoff)
{
	struct snd_soc_pcm_runtime *prun = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *dai = asoc_rtd_to_cpu(prun, 0);
	struct se1000_dma_runtime_data *prtd = psmt->prtd;
	size_t period_size = READ_ONCE(psmt->min_period_size[dai->id][substream->stream]);
	size_t frame_bytes = READ_ONCE(psmt->frame_bytes[dai->id][substream->stream]);
	bool stream = substream->stream;
	int chan_off = dai->id * 8;
	char *be_dma_ptr = NULL, *fe_hw_ptr = NULL;
	int i = 0, j = 0;

	/* update slot mapping info, support contiguous slots map now */
	if (psmt->channel_map[dai->id][stream] && \
		psmt->channel_map[dai->id][stream][0] < prtd->tdm_channels)
		chan_off = psmt->channel_map[dai->id][stream][0] * (prtd->tdm_bytes / prtd->tdm_channels);

	/* dma avail pos */
	be_dma_ptr = psmt->dma_buf[stream]->area + dma_hwoff;
	fe_hw_ptr = substream->runtime->dma_area + prtd->pos[dai->id][stream];
	if (!stream) {
		do {
			memcpy(&(be_dma_ptr[j + chan_off]), &(fe_hw_ptr[i]), frame_bytes);
			i = i + frame_bytes;
			j = j + prtd->tdm_bytes;
		} while (i < period_size);
	} else {
		do {
			memcpy(&(fe_hw_ptr[i]), &(be_dma_ptr[j + chan_off]), frame_bytes);
			i = i + frame_bytes;
			j = j + prtd->tdm_bytes;
		} while (i < period_size);
	}

	return 0;
}

static int se1000_dma_copy_user(struct snd_soc_component *component,
			struct snd_pcm_substream *substream,
				int channel, unsigned long hwoff,
					void __user *buf, unsigned long bytes)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	bool stream = substream->stream;
	char *dma_ptr = runtime->dma_area + hwoff + channel * (runtime->dma_bytes / runtime->channels);

	if (!stream) {
		if (copy_from_user(dma_ptr, buf, bytes))
			return -EFAULT;
	} else {
		if (copy_to_user(buf, dma_ptr, bytes))
			return -EFAULT;
	}

	return 0;
}

/* kernel loopback stream */
static int se1000_dma_copy_kernel(struct snd_soc_component *component,
			struct snd_pcm_substream *substream,
			int channel, unsigned long hwoff,
			void *buf, unsigned long bytes)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	bool stream = substream->stream;
	char *dma_ptr = runtime->dma_area + hwoff + channel * (runtime->dma_bytes / runtime->channels);

	if (!stream)
		memcpy(dma_ptr, buf, bytes);
	else
		memcpy(buf, dma_ptr, bytes);


	return 0;
}

static const char * const se1000_pcm_dma_channel_names[] = {
	[SNDRV_PCM_STREAM_PLAYBACK] = "tx",
	[SNDRV_PCM_STREAM_CAPTURE] = "rx",
};

int se1000_dma_pcm_request_chan_of(
	struct se1000_dma_runtime_data *prtd, struct device *dev,
		const struct snd_dmaengine_pcm_config *config)
{
	unsigned int i;
	const char *name;
	struct dma_chan *chan;

	for_each_pcm_streams(i) {
		name = se1000_pcm_dma_channel_names[i];
		if (config && config->chan_names[i])
			name = config->chan_names[i];
		chan = dma_request_chan(dev, name);
		if (IS_ERR(chan)) {
			/*
			 * Only report probe deferral errors, channels
			 * might not be present for devices that
			 * support only TX or only RX.
			 */
			if (PTR_ERR(chan) == -EPROBE_DEFER)
				return -EPROBE_DEFER;
			prtd->chan[i] = NULL;
		} else {
			prtd->chan[i] = chan;
		}
		if (prtd->flags & SND_DMAENGINE_PCM_FLAG_HALF_DUPLEX)
			break;
	}

	if (prtd->flags & SND_DMAENGINE_PCM_FLAG_HALF_DUPLEX)
		prtd->chan[1] = prtd->chan[0];

	return 0;
}

static int se1000_dma_pcm_new(struct snd_soc_component *component,
			     struct snd_soc_pcm_runtime *rtd)
{
	struct se1000_multi_tdm *psmt = snd_soc_component_get_drvdata(component);
	struct se1000_dma_runtime_data *prtd = psmt->prtd;
	const struct snd_dmaengine_pcm_config *config = prtd->config;

	struct snd_pcm_substream *substream;
	size_t prealloc_buffer_size;
	size_t max_buffer_size;
	unsigned int i;

	prtd->component = component;

	if (config && config->prealloc_buffer_size) {
		prealloc_buffer_size = config->prealloc_buffer_size;
		max_buffer_size = config->pcm_hardware->buffer_bytes_max;
	} else {
		prealloc_buffer_size = 64 * 1024;
		max_buffer_size = 64 * 1024;
	}

	for_each_pcm_streams(i) {
		substream = rtd->pcm->streams[i].substream;
		if (!substream)
			continue;

		if (!prtd->chan[i]) {
			dev_err(component->dev,
				"Missing dma channel for stream: %d\n", i);
			return -EINVAL;
		}

		snd_pcm_set_managed_buffer(substream,
				SNDRV_DMA_TYPE_DEV_IRAM,
				prtd->chan[substream->stream]->device->dev,
				prealloc_buffer_size,
				max_buffer_size);

		if (rtd->pcm->streams[i].pcm->name[0] == '\0') {
			strscpy_pad(rtd->pcm->streams[i].pcm->name,
				rtd->pcm->streams[i].pcm->id,
				sizeof(rtd->pcm->streams[i].pcm->name));
		}
	}

	return 0;
}


const struct snd_soc_component_driver se1000_dma_component_driver = {
	.name		= "se1000_dma_pcm",
	.probe_order	= SND_SOC_COMP_ORDER_NORMAL,
	.open		= se1000_dma_pcm_open,
	.close		= se1000_dma_pcm_close,
	.hw_params	= se1000_dma_pcm_hw_params,
	.trigger	= se1000_dma_pcm_trigger,
	.pointer	= se1000_dma_pcm_pointer,
	.copy_user	= se1000_dma_copy_user,
	.copy_kernel    = se1000_dma_copy_kernel,
	.pcm_construct	= se1000_dma_pcm_new,
	.pcm_destruct = se1000_dma_pcm_destruct,
};

MODULE_DESCRIPTION("Siengine SE1000 audio dma driver");
MODULE_AUTHOR("jian.xu <jian.xu@siengine.com>");
MODULE_LICENSE("GPL v2");
