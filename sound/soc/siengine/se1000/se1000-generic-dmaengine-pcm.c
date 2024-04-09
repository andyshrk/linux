/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Driver for siengine I2S generic dma engine refer to soc-generic-dmaengine-pcm.c
 * Copyright 2023 SiEngine Technology Co., Ltd. All rights reserved
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
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
#include "se1000-aud-i2s.h"
#include "se1000-spdif.h"
#include "../../../../drivers/dma/dw-axi-dmac/dw-axi-dmac-audio.h"

/*
 * The platforms dmaengine driver does not support reporting the amount of
 * bytes that are still left to transfer.
 */
#define SND_DMAENGINE_PCM_FLAG_NO_RESIDUE BIT(31)

static inline struct se1000_dma_runtime_data *component_to_prtd(
	struct snd_soc_component *component)
{
	struct se1000_spdif_base *spdif = NULL;
	struct se_i2s_base *i2s = NULL;
	struct se1000_dma_runtime_data *prtd = NULL;

	if (component->id == SE1000_SPDIF_OUT) {
		spdif = snd_soc_component_get_drvdata(component);
		prtd = spdif->prtd;
	} else {
		i2s = snd_soc_component_get_drvdata(component);
		prtd = i2s->prtd;
	}

	return prtd;
}

static struct device *dmaengine_dma_dev(struct dmaengine_pcm *pcm,
	struct snd_pcm_substream *substream)
{
	if (!pcm->chan[substream->stream])
		return NULL;

	return pcm->chan[substream->stream]->device->dev;
}

int se1000_snd_dmaengine_pcm_prepare_slave_config(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct dma_slave_config *slave_config)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_dmaengine_dai_dma_data *dma_data;
	int ret;

	if (rtd->num_cpus > 1) {
		dev_err(rtd->dev,
			"%s doesn't support Multi CPU yet\n", __func__);
		return -EINVAL;
	}

	dma_data = snd_soc_dai_get_dma_data(asoc_rtd_to_cpu(rtd, 0), substream);

	ret = snd_hwparams_to_dma_slave_config(substream, params, slave_config);
	if (ret)
		return ret;

	snd_dmaengine_pcm_set_config_from_dai_data(substream, dma_data,
		slave_config);

	return 0;
}

static int se1000_dmaengine_pcm_hw_params(struct snd_soc_component *component,
				   struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	struct dmaengine_pcm *pcm = soc_component_to_pcm(component);
	struct dma_chan *chan = pcm->chan[substream->stream];
	int (*prepare_slave_config)(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params,
			struct dma_slave_config *slave_config);
	struct dma_slave_config slave_config;
	int ret;

	memset(&slave_config, 0, sizeof(slave_config));

	if (pcm->config && pcm->config->prepare_slave_config)
		prepare_slave_config = pcm->config->prepare_slave_config;
	else
		prepare_slave_config = se1000_snd_dmaengine_pcm_prepare_slave_config;

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

static int
se1000_dmaengine_pcm_set_runtime_hwparams(struct snd_soc_component *component,
				   struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct dmaengine_pcm *pcm = soc_component_to_pcm(component);
	struct device *dma_dev = dmaengine_dma_dev(pcm, substream);
	struct dma_chan *chan = pcm->chan[substream->stream];
	struct snd_dmaengine_dai_dma_data *dma_data;
	struct snd_pcm_hardware hw;

	if (rtd->num_cpus > 1) {
		dev_err(rtd->dev,
			"%s doesn't support Multi CPU yet\n", __func__);
		return -EINVAL;
	}

	if (pcm->config && pcm->config->pcm_hardware)
		return snd_soc_set_runtime_hwparams(substream,
				pcm->config->pcm_hardware);

	dma_data = snd_soc_dai_get_dma_data(asoc_rtd_to_cpu(rtd, 0), substream);

	memset(&hw, 0, sizeof(hw));
	hw.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_INTERLEAVED;
	hw.periods_min = 2;
	hw.periods_max = UINT_MAX;
	hw.period_bytes_min = 256;
	hw.period_bytes_max = dma_get_max_seg_size(dma_dev);
	hw.buffer_bytes_max = SIZE_MAX;
	hw.fifo_size = dma_data->fifo_size;

	if (pcm->flags & SND_DMAENGINE_PCM_FLAG_NO_RESIDUE)
		hw.info |= SNDRV_PCM_INFO_BATCH;

	/**
	 * FIXME: Remove the return value check to align with the code
	 * before adding snd_dmaengine_pcm_refine_runtime_hwparams
	 * function.
	 */
	snd_dmaengine_pcm_refine_runtime_hwparams(substream,
						  dma_data,
						  &hw,
						  chan);

	return snd_soc_set_runtime_hwparams(substream, &hw);
}

static int se1000_dmaengine_pcm_open(struct snd_soc_component *component,
			      struct snd_pcm_substream *substream)
{
	struct dmaengine_pcm *pcm = soc_component_to_pcm(component);
	struct dma_chan *chan = pcm->chan[substream->stream];
	void *drvdata = snd_soc_component_get_drvdata(component);
	struct se_i2s_base *i2s = NULL;
	struct se1000_spdif_base *spdif = NULL;
	int ret;

	if (component->id == SE1000_SPDIF_OUT)
		spdif = (struct se1000_spdif_base *)drvdata;
	else
		i2s = (struct se_i2s_base *)drvdata;

	ret = se1000_dmaengine_pcm_set_runtime_hwparams(component, substream);
	if (ret)
		return ret;

	if (!chan || !drvdata)
		return -ENXIO;

	ret = snd_pcm_hw_constraint_integer(substream->runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	if (component->id == SE1000_SPDIF_OUT) {
		spdif->prtd->chan[substream->stream] = chan;
		spdif->substream[substream->stream] = substream;
	} else {
		i2s->prtd->chan[substream->stream] = chan;
		i2s->substream[substream->stream] = substream;
	}
	return 0;
}

static int se1000_dmaengine_pcm_close(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream)
{
	struct se1000_dma_runtime_data *prtd = component_to_prtd(component);

	prtd->desc[substream->stream] = NULL;
	prtd->dma_hw_block_len[substream->stream] = 0;
	dmaengine_synchronize(prtd->chan[substream->stream]);
	return 0;
}

static unsigned int se1000_dmaengine_get_actual_hw_block_len(struct dma_chan *chan, dma_cookie_t cookie)
{
	unsigned int block_len = 0;
	struct axi_dma_chan *axi_chan = dchan_to_axi_dma_chan(chan);
	struct virt_dma_desc *vdesc = NULL;
	struct axi_dma_desc *desc = NULL;

	if (!axi_chan) {
		pr_err("%s: get dma device error\n", __func__);
		return 0;
	}

	if (!axi_chan->chip->dma_ringbuff_enable)
		return 0;

	vdesc = vchan_find_desc(&axi_chan->vc, cookie);
	if (!vdesc) {
		pr_err("%s: get dma vdesc error\n", __func__);
		return 0;
	}
	desc = vd_to_axi_desc(vdesc);
	if (!desc) {
		pr_err("%s: get dma desc error\n", __func__);
		return 0;
	}
	block_len = desc->hw_desc[0].len;
	pr_debug("%s: get dma block len %d\n", __func__, block_len);
	return block_len;
}

static inline void se1000_dmaengine_pcm_dma_complete(void *arg, int stream, int if_id)
{
	struct se1000_spdif_base *spdif = NULL;
	struct se_i2s_base *i2s = NULL;
	struct se1000_dma_runtime_data *prtd = NULL;
	struct snd_pcm_substream *substream = NULL;
	size_t period = 0;
	unsigned int block = 0;

	if (!arg)
		return;

	if (if_id == SE1000_SPDIF_OUT) {
		spdif = (struct se1000_spdif_base *)arg;
		substream = spdif->substream[stream];
		prtd = spdif->prtd;
	} else {
		i2s = (struct se_i2s_base *)arg;
		substream = i2s->substream[stream];
		prtd = i2s->prtd;
	}
	block = prtd->dma_hw_block_len[stream];
	period = snd_pcm_lib_period_bytes(substream);

	/* update based on actual dma transfer block */
	prtd->pos[0][stream] += (block > 0 ? block : period);
	if (prtd->pos[0][stream] >= snd_pcm_lib_buffer_bytes(substream))
		prtd->pos[0][stream] = 0;

	if (prtd->pos[0][stream] % period == 0)
		snd_pcm_period_elapsed(substream);
}

static void se1000_dmaengine_pcm_dma_tx_complete(void *arg)
{
	struct se_i2s_base *i2s = (struct se_i2s_base *)arg;
	se1000_dmaengine_pcm_dma_complete(arg, SNDRV_PCM_STREAM_PLAYBACK, i2s->i2s_id);
}

static void se1000_dmaengine_pcm_dma_rx_complete(void *arg)
{
	struct se_i2s_base *i2s = (struct se_i2s_base *)arg;
	se1000_dmaengine_pcm_dma_complete(arg, SNDRV_PCM_STREAM_CAPTURE, i2s->i2s_id);
}

static void se1000_dmaengine_spdif_dma_tx_complete(void *arg)
{
	se1000_dmaengine_pcm_dma_complete(arg, SNDRV_PCM_STREAM_PLAYBACK, SE1000_SPDIF_OUT);
}

static void se1000_dmaengine_spdif_dma_rx_complete(void *arg)
{
	se1000_dmaengine_pcm_dma_complete(arg, SNDRV_PCM_STREAM_CAPTURE, SE1000_SPDIF_OUT);
}

static int se1000_dmaengine_pcm_prepare_and_submit(struct snd_pcm_substream *substream,
			struct snd_soc_component *component)
{
	void *drvdata = snd_soc_component_get_drvdata(component);
	struct se1000_dma_runtime_data *prtd = NULL;
	struct dma_chan *chan = NULL;
	struct se_i2s_base *i2s = NULL;
	struct se1000_spdif_base *spdif = NULL;
	struct dma_async_tx_descriptor *desc;
	enum dma_transfer_direction direction;
	unsigned long flags = DMA_CTRL_ACK;
	dma_addr_t buf_addr = 0;
	size_t buf_len;
	size_t period_len;

	direction = snd_pcm_substream_to_dma_direction(substream);

	if (!substream->runtime->no_period_wakeup)
		flags |= DMA_PREP_INTERRUPT;

	if (drvdata && component->id == SE1000_SPDIF_OUT) {
		spdif = (struct se1000_spdif_base *)drvdata;
		if (spdif->dma_buf[substream->stream] &&
				substream->runtime->format == SNDRV_PCM_FORMAT_S16_LE) {
			dev_info(component->dev, "%s: prepare spdif dma buffer %d\n", __func__, component->id);
			/* spdif 16bit has to align to 4 bytes */
			buf_addr = spdif->dma_buf[substream->stream]->addr;
			buf_len = snd_pcm_lib_buffer_bytes(substream) * 2;
			period_len = snd_pcm_lib_period_bytes(substream) * 2;
		}
		prtd = spdif->prtd;
	} else {
		i2s = (struct se_i2s_base *)drvdata;
		prtd = i2s->prtd;
	}
	chan = prtd->chan[substream->stream];
	prtd->pos[0][substream->stream] = 0;

	if (!buf_addr) {
		buf_addr = substream->runtime->dma_addr;
		buf_len = snd_pcm_lib_buffer_bytes(substream);
		period_len = snd_pcm_lib_period_bytes(substream);
	}
	prtd->period_size = period_len;
	desc = dmaengine_prep_dma_cyclic(chan, buf_addr, buf_len, period_len, direction, flags);

	if (!desc)
		return -ENOMEM;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		desc->callback = (component->id == SE1000_SPDIF_OUT) ? \
				se1000_dmaengine_spdif_dma_tx_complete : se1000_dmaengine_pcm_dma_tx_complete;
	else
		desc->callback = (component->id == SE1000_SPDIF_OUT) ? \
				se1000_dmaengine_spdif_dma_rx_complete : se1000_dmaengine_pcm_dma_rx_complete;

	desc->callback_param = drvdata;
	desc->cookie = dmaengine_submit(desc);
	prtd->desc[substream->stream] = desc;
	return 0;
}

static int se1000_dmaengine_pcm_trigger(struct snd_soc_component *component,
				 struct snd_pcm_substream *substream, int cmd)
{
	struct se1000_dma_runtime_data *prtd = component_to_prtd(component);
	struct dma_chan *chan = prtd->chan[substream->stream];
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		ret = se1000_dmaengine_pcm_prepare_and_submit(substream, component);
		if (ret)
			return ret;
		dma_async_issue_pending(chan);
		/* get the actual dma cal transfer block len */
		prtd->dma_hw_block_len[substream->stream] = \
				se1000_dmaengine_get_actual_hw_block_len(chan, prtd->desc[substream->stream]->cookie);
		prtd->dma_hw_block_len[substream->stream] /= (prtd->period_size / snd_pcm_lib_period_bytes(substream));
		if (prtd->dma_hw_block_len[substream->stream] > snd_pcm_lib_period_bytes(substream)) {
			dev_err(component->dev, "%s: get dma block len (%d) invalid, so use default period\n", __func__, \
							prtd->dma_hw_block_len[substream->stream]);
			prtd->dma_hw_block_len[substream->stream] = 0;
		}
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dmaengine_resume(chan);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dmaengine_pause(chan);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		dmaengine_terminate_async(chan);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int se1000_dmaengine_pcm_new(struct snd_soc_component *component,
			     struct snd_soc_pcm_runtime *rtd)
{
	struct dmaengine_pcm *pcm = soc_component_to_pcm(component);
	const struct snd_dmaengine_pcm_config *config = pcm->config;
	struct device *dev = component->dev;
	struct snd_pcm_substream *substream;
	size_t prealloc_buffer_size;
	size_t max_buffer_size;
	unsigned int i;

	if (config && config->prealloc_buffer_size) {
		prealloc_buffer_size = config->prealloc_buffer_size;
		max_buffer_size = config->pcm_hardware->buffer_bytes_max;
	} else {
		prealloc_buffer_size = 512 * 1024;
		max_buffer_size = SIZE_MAX;
	}

	for_each_pcm_streams(i) {
		substream = rtd->pcm->streams[i].substream;
		if (!substream)
			continue;

		if (!pcm->chan[i] && config && config->chan_names[i])
			pcm->chan[i] = dma_request_slave_channel(dev,
				config->chan_names[i]);

		if (!pcm->chan[i]) {
			dev_err(component->dev,
				"Missing dma channel for stream: %d\n", i);
			return -EINVAL;
		}

		snd_pcm_set_managed_buffer(substream,
				SNDRV_DMA_TYPE_DEV_IRAM,
				dmaengine_dma_dev(pcm, substream),
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

static snd_pcm_uframes_t se1000_dmaengine_pcm_pointer(
	struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct se1000_dma_runtime_data *prtd = component_to_prtd(component);

	return bytes_to_frames(substream->runtime, prtd->pos[0][substream->stream]);
}

static int se1000_dmaengine_spdif_copy_usr(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream,
			       int channel, unsigned long hwoff,
			       void __user *buf, unsigned long bytes)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct se1000_spdif_base *spdif = snd_soc_component_get_drvdata(component);
	bool is_playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	void *dma_ptr = runtime->dma_area + hwoff +
			channel * (runtime->dma_bytes / runtime->channels);
	int i;
	void *be_dma_ptr = NULL;

	if (is_playback) {
		if (copy_from_user(dma_ptr, buf, bytes))
			return -EFAULT;
		/* for 16bit, spdif protocal must be left justified data */
		if (runtime->format == SNDRV_PCM_FORMAT_S16_LE) {
			be_dma_ptr = spdif->dma_buf[SNDRV_PCM_STREAM_PLAYBACK]->area + hwoff * 2;
			be_dma_ptr += 1;
			for (i = 0; i < bytes; i += 2) {
				memcpy(be_dma_ptr, dma_ptr + i, 2);
				be_dma_ptr += 4;
			}
		}
	}

	if (!is_playback) {
		/* for 16bit, spdif protocal must be right justified data */
		if (runtime->format == SNDRV_PCM_FORMAT_S16_LE) {
			be_dma_ptr = spdif->dma_buf[SNDRV_PCM_STREAM_CAPTURE]->area + hwoff * 2;
			be_dma_ptr += 1;
			for (i = 0; i < bytes; i += 2) {
				memcpy(dma_ptr + i, be_dma_ptr, 2);
				be_dma_ptr += 4;
			}
		}
		if (copy_to_user(buf, dma_ptr, bytes))
			return -EFAULT;
	}

	return 0;
}

static const struct snd_soc_component_driver dmaengine_pcm_component = {
	.name           = SND_DMAENGINE_PCM_DRV_NAME,
	.probe_order    = SND_SOC_COMP_ORDER_LATE,
	.open           = se1000_dmaengine_pcm_open,
	.close          = se1000_dmaengine_pcm_close,
	.hw_params      = se1000_dmaengine_pcm_hw_params,
	.trigger        = se1000_dmaengine_pcm_trigger,
	.pointer        = se1000_dmaengine_pcm_pointer,
	.pcm_construct  = se1000_dmaengine_pcm_new,
};

static struct snd_soc_component_driver se1000_i2s_pcm_component[SE1000_I2S_NUM + 1];

static const char * const se1000_dmaengine_pcm_dma_channel_names[] = {
	[SNDRV_PCM_STREAM_PLAYBACK] = "tx",
	[SNDRV_PCM_STREAM_CAPTURE] = "rx",
};

static int se1000_dmaengine_pcm_request_chan_of(struct dmaengine_pcm *pcm,
	struct device *dev, const struct snd_dmaengine_pcm_config *config)
{
	unsigned int i;
	const char *name;
	struct dma_chan *chan;

	if ((pcm->flags & SND_DMAENGINE_PCM_FLAG_NO_DT) || (!dev->of_node &&
	    !(config && config->dma_dev && config->dma_dev->of_node)))
		return 0;

	if (config && config->dma_dev) {
		/*
		 * If this warning is seen, it probably means that your Linux
		 * device structure does not match your HW device structure.
		 * It would be best to refactor the Linux device structure to
		 * correctly match the HW structure.
		 */
		dev_warn(dev, "DMA channels sourced from device %s",
			 dev_name(config->dma_dev));
		dev = config->dma_dev;
	}

	for_each_pcm_streams(i) {
		if (pcm->flags & SND_DMAENGINE_PCM_FLAG_HALF_DUPLEX)
			name = "rx-tx";
		else
			name = se1000_dmaengine_pcm_dma_channel_names[i];
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
			pcm->chan[i] = NULL;
		} else {
			pcm->chan[i] = chan;
		}
		if (pcm->flags & SND_DMAENGINE_PCM_FLAG_HALF_DUPLEX)
			break;
	}

	if (pcm->flags & SND_DMAENGINE_PCM_FLAG_HALF_DUPLEX)
		pcm->chan[1] = pcm->chan[0];

	return 0;
}

static void se1000_dmaengine_pcm_release_chan(struct dmaengine_pcm *pcm)
{
	unsigned int i;

	for_each_pcm_streams(i) {
		if (!pcm->chan[i])
			continue;
		dma_release_channel(pcm->chan[i]);
		if (pcm->flags & SND_DMAENGINE_PCM_FLAG_HALF_DUPLEX)
			break;
	}
}

int se1000_snd_dmaengine_pcm_register(struct device *dev,
	const struct snd_dmaengine_pcm_config *config, unsigned int flags, struct snd_soc_dai_driver *dai_drv,
	const struct snd_kcontrol_new *controls, int num_controls)
{
	struct snd_soc_component_driver *driver;
	struct dmaengine_pcm *pcm;
	int ret;

	pcm = kzalloc(sizeof(*pcm), GFP_KERNEL);
	if (!pcm)
		return -ENOMEM;

#ifdef CONFIG_DEBUG_FS
	pcm->component.debugfs_prefix = "dma";
#endif
	pcm->config = config;
	pcm->flags = flags;

	ret = se1000_dmaengine_pcm_request_chan_of(pcm, dev, config);
	if (ret)
		goto err_free_dma;

	driver = &se1000_i2s_pcm_component[dai_drv->id];
	*driver = dmaengine_pcm_component;
	if (controls) {
		driver->controls = controls;
		driver->num_controls = num_controls;
	}
	if (dai_drv->id == SE1000_SPDIF_OUT)
		driver->copy_user = &se1000_dmaengine_spdif_copy_usr;

	ret = snd_soc_component_initialize(&pcm->component, driver, dev);
	if (ret)
		goto err_free_dma;

	ret = snd_soc_add_component(&pcm->component, dai_drv, 1);
	if (ret)
		goto err_free_dma;

	pcm->component.id = dai_drv->id;

	return 0;
err_free_dma:
	se1000_dmaengine_pcm_release_chan(pcm);
	kfree(pcm);
	return ret;
}

void se1000_snd_dmaengine_pcm_unregister(struct device *dev)
{
	struct snd_soc_component *component;
	struct dmaengine_pcm *pcm;

	component = snd_soc_lookup_component(dev, SND_DMAENGINE_PCM_DRV_NAME);
	if (!component)
		return;

	pcm = soc_component_to_pcm(component);

	snd_soc_unregister_component_by_driver(dev, component->driver);
	se1000_dmaengine_pcm_release_chan(pcm);
	kfree(pcm);
}

MODULE_LICENSE("GPL");
