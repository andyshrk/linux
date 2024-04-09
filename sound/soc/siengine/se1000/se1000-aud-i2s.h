/*
 * Siengine se1000 i2s ctrl definition
 *
 * Copyright (c) 2022-2023 Siengine Inc.
 * Author: jian.xu <jian.xu@siengine.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _SE1000_AUD_I2S_H_
#define _SE1000_AUD_I2S_H_

#include <sound/dmaengine_pcm.h>
#include "se1000-aud-reg.h"
#include "audio_tdm_mlp/se1000-dma.h"

#define AUD_PLL_CLK 442368000
#ifndef USEC_PER_SEC
 #define USEC_PER_SEC 1000000
#endif

#define MONO 1
#define STEREO 2

enum {
	SE1000_I2S0,
	SE1000_I2S1,
	SE1000_I2S2,
	SE1000_I2S3,
	SE1000_I2S4,
	SE1000_I2S5,
	SE1000_I2S6,
	SE1000_I2S_NUM,
	SE1000_SPDIF_OUT = SE1000_I2S_NUM,
	SE1000_HL_SHR_I2S0,
	SE1000_HL_SHR_I2S1,
	SE1000_HL_SHR_I2S2,
};

enum i2s_msmode {
	SLAVE,
	MASTER,
};

enum i2s_tran_mode {
	HALF_DUPLEX_MODE,
	FULL_DUPLEX_MODE,
};

enum i2s_work_format {
	I2S_FORMAT_BEGIN,
	I2S_PHILIPS_FORMAT,
	I2S_LJ_FORMAT,
	I2S_RJ_FORMAT,
	I2S_DSP_A_FORMAT,
	I2S_TDM_FORMAT,
	I2S_FORMAT_END,
};

enum i2s_data_order {
	MSB,
	LSB,
};

struct i2s_params {
	enum i2s_msmode i2s_mode;          /* master/slave */
	enum i2s_tran_mode i2s_transMode;
	enum i2s_work_format i2s_format;
	u32 i2s_data_align;
	u32 i2s_data_delay;                /* data delay */
	enum i2s_data_order i2s_dataOrder; /* 0:MSB 1:LSB */
	u32 i2s_clk_polar;
	bool i2s_loopback;                 /* internel loopback */

	u32 i2s_fifo_tx_threshold[2];
	u32 i2s_fifo_rx_threshold[2];
};

enum se1000_aud_i2s_dir {
	I2S_OUT,
	I2S_IN,
	I2S_DIR_NUM,
};

struct se_i2s_base {
	void __iomem *base_addr;
	void __iomem *ctrl_addr;

	u32 i2s_id;
	struct device *dev;
	struct regmap *regmap;
	struct clk *i2s_clk;
	phys_addr_t base_phys;
	struct mutex irq_alloc_lock; /* dynamic alloc irq lock */

	struct snd_dmaengine_dai_dma_data dma_data[2];
	struct i2s_params i2s_config;

	int irqs;

	bool playback;
	bool capture;
	u32 mclk_enable;
	u32 mclk_rate_coef;

	int latency;
	struct pm_qos_request pm_qos_req;
	const struct snd_pcm_hardware *se_aud_hardware;
	void *platform_priv;

	struct se1000_dma_runtime_data *prtd;
	struct snd_pcm_substream *substream[2];
	unsigned int active[2];
};

extern const struct snd_soc_dai_ops se1000_i2s_ops;
int se1000_i2s_resume(struct snd_soc_component *component);
int se1000_i2s_suspend(struct snd_soc_component *component);
irqreturn_t se1000_i2s_irq_handler(int irq_id, void *dev);
int se1000_loopback_set(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol);
int se1000_loopback_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol);

#endif
