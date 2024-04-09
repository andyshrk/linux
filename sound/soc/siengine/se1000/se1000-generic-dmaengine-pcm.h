/* SPDX-License-Identifier: GPL-2.0.
 * sound/soc/siengine/se1000/
 * Copyright (C) 2023 SiEngine Technology Co., Ltd. All rights reserved
 */
#ifndef _SE1000_DMA_ENGINE_H_
#define _SE1000_DMA_ENGINE_H_

#include <sound/pcm.h>
#include <sound/soc.h>
#include <linux/dmaengine.h>

int se1000_snd_dmaengine_pcm_prepare_slave_config(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct dma_slave_config *slave_config);

int se1000_snd_dmaengine_pcm_register(struct device *dev,
	const struct snd_dmaengine_pcm_config *config, unsigned int flags, struct snd_soc_dai_driver *dai_drv,
	const struct snd_kcontrol_new *controls, int num_controls);

void se1000_snd_dmaengine_pcm_unregister(struct device *dev);

#endif
