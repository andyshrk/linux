/*
 * SPDX-License-Identifier: GPL-2.0.
 * sound/soc/siengine/audio_tdm_mlp
 * Copyright (C) 2022-2023 SiEngine or its affiliates
*/

#ifndef _SE1000_MULTI_TDM_H_
#define _SE1000_MULTI_TDM_H_

#include "../se1000-aud-i2s.h"
#include "se1000-dma.h"
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>

#define MIN_PERIOD_BYTE 512
#define MAX_PERIOD_BYTE 32768
#define MIN_NUM_PERIODS 2
#define MAX_NUM_PERIODS 16
#define MIN_NUM_CHANNELS 1
#define MAX_NUM_CHANNELS 8

/*
 * pre-allocate the BE DMA buffer as (256 * 16 * 4) * 3,
 * this should be adjusted based on actual tdm format.
 */
#define SE1000_DMABUF_BYTE	(48 * 1024)
#define MIN_PERIOD_SIZE 256

enum {
	SE1000_MULTI_TDM0,
	SE1000_MULTI_TDM1,
	SE1000_MULTI_TDM2,
	SE1000_MULTI_TDM3,
	SE1000_MULTI_TDM4,
	SE1000_MULTI_TDM5,
	SE1000_MULTI_TDM6,
	SE1000_MULTI_TDM7,
	SE1000_MULTI_HL_FE_SHR0,
	SE1000_MULTI_HL_FE_SHR1,
	SE1000_MULTI_HL_FE_SHR2,
	SE1000_MULTI_HL_FE_SHR3,
	SE1000_MULTI_HL_FE_SHR4,
	SE1000_MULTI_HL_FE_SHR5,
	SE1000_MULTI_HL_FE_SHR6,
	SE1000_MULTI_HL_FE_SHR7,
	SE1000_MULTI_HL_FE_SHR8,
	SE1000_MULTI_HL_FE_SHR9,
	SE1000_MULTI_HL_FE_SHR10,
	SE1000_MULTI_MAX
};

struct se1000_multi_tdm {
	struct device *dev;
	struct snd_dma_buffer *dma_buf[2];
	unsigned int dma_hwoff[2];
	/* channel map get the slot map info after hw parameter config */
	char *channel_map[PCM_NODE_MAX][2];
	bool play[PCM_NODE_MAX];
	bool capture[PCM_NODE_MAX];
	unsigned int frame_bytes[PCM_NODE_MAX][2];
	size_t min_period_size[PCM_NODE_MAX][2];
	struct snd_pcm_substream *substream[PCM_NODE_MAX][2];
	spinlock_t splt;
	struct se1000_dma_runtime_data *prtd;
};

#endif

