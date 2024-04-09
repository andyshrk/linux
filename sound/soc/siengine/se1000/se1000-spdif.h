/*
 * ALSA SoC Audio Layer - Siengine SPDIF driver
 *
 * Copyright (c) 2022-2023 Siengine Inc.
 * Author: jian.xu <jian.xu@siengine.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SND_SOC_SE1000_SPDIF_H
#define __SND_SOC_SE1000_SPDIF_H

#include "audio_tdm_mlp/se1000-dma.h"

#define SPDIF_WORK_CLK         (442368000/128)

#define SPDIF_CTRL             0x00
#define SPDIF_INT_REG          0x04
#define SPDIF_FIFO_CTRL        0x08
#define SPDIF_STAT_REG         0x0C
#define SPDIF_FIFO_ADDR        0x10

/* SPDIF_CTRL register, mainly for mask */
#define CTRL_TSAMPLERATE      (0xFF << 0)
#define CTRL_SFR_ENABLE       (1<<8)
#define CTRL_SPDIF_ENABLE     (1<<9)
#define CTRL_FIFO_ENABLE      (1<<10)
#define CTRL_CLK_ENABLE       (1<<11)
#define CTRL_TR_MODE          (1<<12)
#define CTRL_PARITYCHECK      (1<<13)
#define CTRL_PARITYGEN        (1<<14)
#define CTRL_VALIDITYCHECK    (1<<15)
#define CTRL_CHANNEL_MODE     (1<<16)
#define CTRL_DUPLICATE_SET(x) ((x)<<17)
#define CTRL_DUPLICATE        (1<<17)
#define CTRL_SETPREAMBB       (1<<18)
#define CTRL_USE_FIFO_IF      (1<<19)

#define CTRL_PARITY_MASK      (1<<21)
#define CTRL_UNDERR_MASK      (1<<22)
#define CTRL_OVRERR_MASK      (1<<23)
#define CTRL_EMPTY_MASK       (1<<24)
#define CTRL_AEMPTY_MASK      (1<<25)
#define CTRL_FULL_MASK        (1<<26)
#define CTRL_AFULL_MASK       (1<<27)
#define CTRL_SYNCERR_AMSK     (1<<28)
#define CTRL_LOCK_MASK        (1<<29)
#define CTRL_BEGIN_MASK       (1<<30)
#define CTRL_INTREQ_MASK      (1<<31)

/* INT_REG register, mainly for mask */
#define INT_REG_RSAMPLERATE     (0xFF << 0)
#define INT_REG_PREAMBLEDEL     (0x1FFF << 8)
#define INT_REG_PARITYO         (1<<21)
#define INT_REG_TDATA_UNDERR    (1<<22)
#define INT_REG_RDATA_OVRERR    (1<<23)
#define INT_REG_FIFO_EMPTY      (1<<24)
#define INT_REG_FIFO_AEMPTY     (1<<25)
#define INT_REG_FIFO_FULL       (1<<26)
#define INT_REG_FIFO_AFULL      (1<<27)
#define INT_REG_SYNCERR         (1<<28)
#define INT_REG_LOCK            (1<<29)
#define INT_REG_BLOCK_BEGIN     (1<<30)

/* FIFO_CTRL, mainly for mask */
#define FIFO_AEMPTY_TSHOLD_SET(x) (x << 0)
#define FIFO_AEMPTY_THRESHOLD    (0x3F << 0)
#define FIFO_AFULL_TSHOLD_SET(x) (x << 16)
#define FIFO_AFULL_THRESHOLD    (0x3F << 16)

/* FIFO STAT_REG, mainly for mask */
#define STAT_FIFO_LEVEL     (0x7F << 0)
#define STAT_PARITY_FLAG    (1<<21)
#define STAT_UNDERR_FLAG    (1<<22)
#define STAT_OVRERR_FLAG    (1<<23)
#define STAT_EMPTY_FLAG     (1<<24)
#define STAT_AEMPTY_FLAG    (1<<25)
#define STAT_FULL_FLAG      (1<<26)
#define STAT_AFULL_FLAG     (1<<27)
#define STAT_SYNCERR_FLAG   (1<<28)
#define STAT_LOCK_FLAG      (1<<29)
#define STAT_BEGIN_FLAG     (1<<30)
#define STAT_RIGHT_LEFT     (1<<31)

#define SPDIF_PREALLOCATE_BUFF_SIZE (512 * 1024)

struct spdif_params {
	bool paritycheck;
	bool paritygen;
	bool validitycheck;
	bool setpreambb;
	u32 preambledel;
	u32 spdif_fifo_tx_threshold[2];
	u32 spdif_fifo_rx_threshold[2];
};

struct se1000_spdif_base {
	struct device *dev;
	struct regmap *regmap;
	void __iomem *base_addr;
	struct clk *spdif_clk;
	bool playback;
	bool capture;
	struct spdif_params spdif_config;
	struct snd_dmaengine_dai_dma_data dma_data[2];
	struct se1000_dma_runtime_data *prtd;
	struct snd_pcm_substream *substream[2];
	struct snd_dma_buffer *dma_buf[2];
};

#endif
