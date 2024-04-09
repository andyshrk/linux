/*
 * SPDX-License-Identifier: GPL-2.0.
 * sound/soc/siengine/audio_tdm_mlp
 * Copyright (C) 2022-2023 SiEngine or its affiliates
*/

#ifndef _SE1000_DMA_H_
#define _SE1000_DMA_H_

#define PCM_FORMAT_MAX_NUM_CHANNEL 16
#define PCM_NODE_MAX 20


struct se1000_dma_runtime_data {
	struct device *dev;

	struct dma_chan *chan[SNDRV_PCM_STREAM_LAST + 1];
	const struct snd_dmaengine_pcm_config *config;
	struct snd_soc_component *component;
	struct dma_async_tx_descriptor *desc[2];
	unsigned int flags;

	size_t prealloc_buffer;
	size_t period_size;
	int cycle_count;
	size_t min_period;
	unsigned int tdm_channels;
	unsigned int tdm_bytes;
	unsigned int frame_bits;
	unsigned int active[2];
	unsigned int dma_hw_block_len[2];

	unsigned int pos[PCM_NODE_MAX][2];
	void *private;
};

extern const struct snd_soc_component_driver se1000_dma_component_driver;
extern int se1000_dma_pcm_request_chan_of(struct se1000_dma_runtime_data *prtd,
	struct device *dev, const struct snd_dmaengine_pcm_config *config);

#endif

