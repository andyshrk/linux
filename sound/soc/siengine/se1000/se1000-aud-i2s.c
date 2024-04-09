/*
 * Siengine se1000 audio i2s ctrl
 *
 * Copyright (c) 2022-2023 Siengine Inc.
 * Author: jian.xu <jian.xu@siengine.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/regmap.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/pm_qos.h>
#include <asm/div64.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/consumer.h>

#include "se1000-aud-i2s.h"

irqreturn_t se1000_i2s_irq_handler(int irq_id, void *dev)
{
	struct se_i2s_base *i2s = dev;
	u32 val = 0;

	regmap_update_bits(i2s->regmap, I2S_CTRL, I2S_INTREQ_MASK, 0);
	regmap_read(i2s->regmap, I2S_STAT, &val);

	if (i2s->i2s_config.i2s_transMode == FULL_DUPLEX_MODE) {
		if (i2s->playback) {
			if (val & I2S_STATUS_TFIFO_EMPTY)
				dev_info(i2s->dev, "I2S playback empty! val =%x \n", val);
		} else {
			if (val & I2S_STATUS_RFIFO_FULL)
				dev_info(i2s->dev, "I2S capture full! val =%x\n", val);
		}
	} else {
		if (val & I2S_STATUS_TFIFO_EMPTY)
			dev_info(i2s->dev, "I2S fifo empty! val =%x\n", val);
	}

	if (val & I2S_STATUS_TDATA_UNDERRUN)
		dev_info(i2s->dev, "I2S playback underrun! val =%x\n", val);

	if (val & I2S_STATUS_RDATA_OVERRUN)
		dev_info(i2s->dev, "I2S capture overrun! val =%x\n", val);

	regmap_write(i2s->regmap, I2S_STAT, 0x0);
	regmap_update_bits(i2s->regmap, I2S_CTRL, I2S_INTREQ_MASK, I2S_INTREQ_MASK);

	return IRQ_HANDLED;
}

static int se1000_aud_i2s_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	u32 stream_dir = substream->stream;
	struct se_i2s_base *i2s = snd_soc_dai_get_drvdata(dai);
	int ret = 0;

	i2s->active[stream_dir] += 1;

	if (READ_ONCE(i2s->active[stream_dir]) > 1)
		return ret;

	/* gpio config i2s function when startup */
	pinctrl_pm_force_default_state(i2s->dev);

	ret = clk_prepare_enable(i2s->i2s_clk);
	if (ret) {
		dev_err(i2s->dev, "i2s pclk enable failed %d\n", ret);
		return ret;
	}

	if (i2s->i2s_config.i2s_transMode == FULL_DUPLEX_MODE) {
		if (stream_dir == SNDRV_PCM_STREAM_PLAYBACK) {
			/* reset TX FIFO */
			regmap_update_bits(i2s->regmap, I2S_CTRL, I2S_FIFO_RST, 0);
			i2s->playback = 1;
			regmap_update_bits(i2s->regmap, I2S_STAT,
				I2S_STATUS_TDATA_UNDERRUN | I2S_STATUS_TFIFO_AFULL |
				I2S_STATUS_TFIFO_FULL | I2S_STATUS_TFIFO_AEMPTY |
				I2S_STATUS_TFIFO_EMPTY, 0);
		} else {
			/* reset FIFO */
			regmap_update_bits(i2s->regmap, I2S_CTRL_FDX, I2S_RFIFO_RST, 0);
			i2s->capture = 1;
			regmap_update_bits(i2s->regmap, I2S_STAT,
				I2S_STATUS_RDATA_OVERRUN | I2S_STATUS_RFIFO_AFULL |
				I2S_STATUS_RFIFO_FULL | I2S_STATUS_RFIFO_AEMPTY |
				I2S_STATUS_RFIFO_EMPTY, 0);
		}

		/* enable I2S clk*/
		regmap_update_bits(i2s->regmap, I2S_CTRL, I2S_STB, 0);
	} else {
		regmap_update_bits(i2s->regmap, I2S_CTRL, I2S_FIFO_RST, 0);
		regmap_write(i2s->regmap, I2S_STAT, 0x0);
	}

	return ret;
}

static void se1000_aud_i2s_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct se_i2s_base *i2s = dev_get_drvdata(dai->dev);
	u32 stream_dir = substream->stream;

	i2s->active[stream_dir] -= 1;

	if (READ_ONCE(i2s->active[stream_dir]) > 1)
		return;

	if (i2s->i2s_config.i2s_transMode == FULL_DUPLEX_MODE) {
		if (stream_dir == SNDRV_PCM_STREAM_PLAYBACK) {
			i2s->playback = 0;

			/* disable TX */
			regmap_update_bits(i2s->regmap, I2S_CTRL_FDX, I2S_FULL_DUPLEX | I2S_FTX_EN, 0);
			regmap_update_bits(i2s->regmap, I2S_STAT,
				I2S_STATUS_TDATA_UNDERRUN | I2S_STATUS_TFIFO_AFULL |
				I2S_STATUS_TFIFO_FULL | I2S_STATUS_TFIFO_AEMPTY |
				I2S_STATUS_TFIFO_EMPTY, 0);
		} else {
			i2s->capture = 0;

			/* disable RX */
			regmap_update_bits(i2s->regmap, I2S_CTRL_FDX, I2S_FULL_DUPLEX | I2S_FRX_EN, 0);
			regmap_update_bits(i2s->regmap, I2S_STAT,
				I2S_STATUS_RDATA_OVERRUN | I2S_STATUS_RFIFO_AFULL |
				I2S_STATUS_RFIFO_FULL | I2S_STATUS_RFIFO_AEMPTY |
				I2S_STATUS_RFIFO_EMPTY, 0);
		}

		if (i2s->playback == i2s->capture) {
			/* disable I2S */
			regmap_update_bits(i2s->regmap, I2S_CTRL, I2S_EN, 0);
			clk_disable_unprepare(i2s->i2s_clk);

			/* reset SFR block */
			regmap_update_bits(i2s->regmap, I2S_CTRL, I2S_SFR_RST, 0);

			/* disable I2S mclk*/
			if (i2s->mclk_enable)
				writel(0x0, i2s->ctrl_addr + AUDIO_I2S_MCLK(dai->id));
		}

	} else {
		/* reset FIFO */
		regmap_update_bits(i2s->regmap, I2S_CTRL, I2S_FIFO_RST, 0);
		/* disable I2S */
		regmap_update_bits(i2s->regmap, I2S_CTRL, I2S_EN, 0);
		clk_disable_unprepare(i2s->i2s_clk);

		/* disable I2S mclk*/
		if (i2s->mclk_enable)
			writel(0x0, i2s->ctrl_addr + AUDIO_I2S_MCLK(dai->id));
	}
	/* gpio config low-power mode when shutdown */
	pinctrl_pm_select_sleep_state(i2s->dev);

	if (cpu_latency_qos_request_active(&i2s->pm_qos_req))
		cpu_latency_qos_remove_request(&i2s->pm_qos_req);

	return;
}

static int se1000_aud_i2s_prepare(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct se_i2s_base *i2s = dev_get_drvdata(dai->dev);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct se1000_dma_runtime_data *prtd = i2s->prtd;
	int channels = prtd->tdm_channels > 0 ? prtd->tdm_channels : runtime->channels;
	int rate = runtime->rate;
	u32 stream_dir = substream->stream;

	if (READ_ONCE(i2s->active[substream->stream]) > 1)
		return 0;

	/*
	 * The DMA must act to a DMA request within latency time (usec) to avoid
	 * under/overflow
	 */

	if (stream_dir == SNDRV_PCM_STREAM_PLAYBACK)
		i2s->latency = i2s->i2s_config.i2s_fifo_tx_threshold[1] * USEC_PER_SEC;
	else
		i2s->latency = i2s->i2s_config.i2s_fifo_rx_threshold[1] * USEC_PER_SEC;

	do_div(i2s->latency, channels * rate);

	if (cpu_latency_qos_request_active(&i2s->pm_qos_req))
		cpu_latency_qos_update_request(&i2s->pm_qos_req, i2s->latency);
	else
		cpu_latency_qos_add_request(&i2s->pm_qos_req, i2s->latency);

	return 0;
}

static int se1000_aud_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct se_i2s_base *i2s = dev_get_drvdata(dai->dev);
	unsigned int val = 0, mask = 0;

	if (i2s->playback) {
		if (READ_ONCE(i2s->active[0]) > 1)
			return 0;
	} else if (i2s->capture) {
		if (READ_ONCE(i2s->active[1]) > 1)
			return 0;
	}
	/* master or slave */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	/* i2s slave */
	case SND_SOC_DAIFMT_CBM_CFM:
		val &= ~I2S_MS_CFG;
		break;
	/* i2s master */
	case SND_SOC_DAIFMT_CBS_CFS:
		val |= I2S_MS_CFG;
		break;
	default:
		val |= I2S_MS_CFG;
		break;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	/* I2S */
	case SND_SOC_DAIFMT_I2S:
		val |= I2S_WS_MODE_SET(1) | I2S_DATA_WS_DEL_SET(1);
		break;
	/* LJ */
	case SND_SOC_DAIFMT_LEFT_J:
		val |= I2S_WS_MODE_SET(1) | I2S_DATA_WS_DEL_SET(0);
		break;
	/* RJ */
	case SND_SOC_DAIFMT_RIGHT_J:
		val |= I2S_WS_MODE_SET(1) | I2S_DATA_ALIGN_SET(1);
		break;
	/* PCM mode */
	case SND_SOC_DAIFMT_DSP_A:
		val &= ~I2S_WS_MODE; /* ws_mode =0 */
		val |= I2S_DATA_WS_DEL_SET(1);
		break;
	/* TDM mode */
	case SND_SOC_DAIFMT_DSP_B:
		val &= ~I2S_WS_MODE;
		val |= I2S_WS_MODE_SET(0) | I2S_DATA_WS_DEL_SET(0);
		break;
	default:
		val |= I2S_WS_MODE_SET(1);
		break;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		val |= I2S_SCK_POLAR_SET(0U) | I2S_WS_POLAR_SET(0U);
		break;
	case SND_SOC_DAIFMT_NB_IF:
		val |= I2S_SCK_POLAR_SET(0U) | I2S_WS_POLAR_SET(1U);
		break;
	case SND_SOC_DAIFMT_IB_NF:
		val |= I2S_SCK_POLAR_SET(1U) | I2S_WS_POLAR_SET(0U);
		break;
	case SND_SOC_DAIFMT_IB_IF:
		val |= I2S_SCK_POLAR_SET(1U) | I2S_WS_POLAR_SET(1U);
		break;
	default:
		val |= I2S_SCK_POLAR_SET(0U) | I2S_WS_POLAR_SET(0U);
		break;
	}

	mask |= I2S_MS_CFG | I2S_WS_MODE | I2S_SCK_POLAR
		| I2S_WS_POLAR | I2S_DATA_WS_DELAY | I2S_DATA_ALIGN;
	regmap_update_bits(i2s->regmap, I2S_CTRL, mask, val);

	if (i2s->i2s_config.i2s_format ==  I2S_TDM_FORMAT) {
		if (i2s->capture && !i2s->playback)
			regmap_update_bits(i2s->regmap, TDM_FD_DIR, TDM_CHN_TXEN, 0);
	}

	return 0;
}

static int se1000_aud_i2s_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct se_i2s_base *i2s = dev_get_drvdata(dai->dev);
	struct se1000_dma_runtime_data *prtd = i2s->prtd;
	snd_pcm_format_t format = params_format(params);
	unsigned int channels = params_channels(params);
	unsigned int rate = params_rate(params);
	int chn_width = params_width(params);
	u32 stream_dir = substream->stream;
	u32 val = 0, mask = 0;
	int fs_setrate, bitwidth;

	if (READ_ONCE(i2s->active[substream->stream]) > 1)
		return 0;

	/* set channel */
	if (i2s->i2s_config.i2s_format == I2S_TDM_FORMAT) {
		regmap_update_bits(i2s->regmap, TDM_CTRL, TDM_CHN_NO, TDM_CHN_NO_SET(channels-1));
		/* enable TDM channel */
		regmap_update_bits(i2s->regmap, TDM_CTRL, TDM_CHN_EN,
			TDM_CHN_EN_SET(TDM_CHN_EN_CAL(channels)));
	} else {
		switch (channels) {
		case MONO:
			val |= I2S_MONO_MODE_SET(0);
			val |= I2S_AUDIO_MODE_SET(1);
			/* i2s mono is actually 2 channels, only one channel is active */
			channels = 2;
			break;
		case STEREO:
			val |= I2S_AUDIO_MODE_SET(0);
			break;
		default:
			dev_err(dai->dev, "error channel setting for I2S mode\n");
			return -EINVAL;
		}

		mask = I2S_MONO_MODE | I2S_AUDIO_MODE;
		regmap_update_bits(i2s->regmap, I2S_CTRL, mask, val);
	}

	switch (chn_width) {
	case 16:
		val |= I2S_CHN_WIDTH_SET(2);
		break;
	case 24:
		val |= I2S_CHN_WIDTH_SET(5);
		break;
	case 32:
		val |= I2S_CHN_WIDTH_SET(7);
		break;

	default:
		dev_err(dai->dev, "error chn_width setting for I2S mode\n");
		return -EINVAL;
	}

	regmap_update_bits(i2s->regmap, I2S_CTRL, I2S_CHN_WIDTH, val);

	bitwidth = snd_pcm_format_width(format);
	if (i2s->i2s_config.i2s_transMode == FULL_DUPLEX_MODE) {
		if (stream_dir == SNDRV_PCM_STREAM_PLAYBACK) {
			/* playback set sample resolution */
			regmap_update_bits(i2s->regmap, I2S_SRES, I2S_TX_SAMP_RES,
				I2S_TX_SAMP_RES_SET(bitwidth - 1));

			/* set TX fifo afull*/
			regmap_update_bits(i2s->regmap, FIFO_AFULL, AFULL_THRESHOLD,
				AFULL_THRESHOLD_SET(i2s->i2s_config.i2s_fifo_tx_threshold[1]));
			/* set TX fifo aempty */
			regmap_update_bits(i2s->regmap, FIFO_AEMPTY, AEMPTY_THRESHOLD,
				AEMPTY_THRESHOLD_SET(i2s->i2s_config.i2s_fifo_tx_threshold[0]));
			if (i2s->i2s_config.i2s_format ==  I2S_TDM_FORMAT)
				/* enable TDM TX muti channel */
				regmap_update_bits(i2s->regmap, TDM_FD_DIR, TDM_CHN_TXEN,
					TDM_CHN_TXEN_SET(TDM_CHN_TXEN_CAL(channels)));
		} else {
			/*capture set sample resolution */
			regmap_update_bits(i2s->regmap, I2S_SRES_FDR, I2S_RX_SAMP_RES,
				I2S_RX_SAMP_RES_SET(bitwidth - 1));

			/* set RX fifo afull */
			regmap_update_bits(i2s->regmap, RX_FIFO_AFULL_FDR, RX_AFULL_THRESHOLD,
				RX_AFULL_THRESHOLD_SET(i2s->i2s_config.i2s_fifo_rx_threshold[1]));
			/* set RX fifo aempty */
			regmap_update_bits(i2s->regmap, RX_FIFO_AEMPTY_FDR, RX_AEMPTY_THRESHOLD,
				RX_AEMPTY_THRESHOLD_SET(i2s->i2s_config.i2s_fifo_rx_threshold[0]));
			if (i2s->i2s_config.i2s_format ==  I2S_TDM_FORMAT)
				/* enable TDM RX muti channel */
				regmap_update_bits(i2s->regmap, TDM_FD_DIR, TDM_CHN_RXEN,
					TDM_CHN_RXEN_SET(TDM_CHN_TXEN_CAL(channels)));
		}
	} else {
		/* half-duplex mode set sample resolution */
		regmap_update_bits(i2s->regmap, I2S_SRES,
			I2S_TX_SAMP_RES, I2S_TX_SAMP_RES_SET(bitwidth));

		if (stream_dir == SNDRV_PCM_STREAM_PLAYBACK) {
			/* set TX fifo aempty */
			regmap_update_bits(i2s->regmap, FIFO_AEMPTY, AEMPTY_THRESHOLD,
				AEMPTY_THRESHOLD_SET(i2s->i2s_config.i2s_fifo_tx_threshold[0]));
			/* set TX fifo afull */
			regmap_update_bits(i2s->regmap, FIFO_AFULL, AFULL_THRESHOLD,
				AFULL_THRESHOLD_SET(i2s->i2s_config.i2s_fifo_tx_threshold[1]));
		} else {
			/* set RX fifo aempty */
			regmap_update_bits(i2s->regmap, FIFO_AEMPTY, AEMPTY_THRESHOLD,
				AEMPTY_THRESHOLD_SET(i2s->i2s_config.i2s_fifo_rx_threshold[0]));
			/* set RX fifo afull */
			regmap_update_bits(i2s->regmap, FIFO_AFULL, AFULL_THRESHOLD,
				AFULL_THRESHOLD_SET(i2s->i2s_config.i2s_fifo_rx_threshold[1]));
		}
	}
	/* used by multi pcm card only */
	prtd->frame_bits = bitwidth * channels;
	prtd->period_size = prtd->min_period * prtd->frame_bits / 8;
	prtd->cycle_count = prtd->prealloc_buffer / prtd->period_size;
	prtd->tdm_bytes = prtd->frame_bits / 8;

	/* set sample rate */
	fs_setrate = AUD_PLL_CLK;
	do_div(fs_setrate, channels * rate * chn_width);

	regmap_update_bits(i2s->regmap, I2S_SRATE, I2S_SAMPLE_RATE,
		I2S_SAMPLE_RATE_SET(fs_setrate));

	return 0;
}

static void se1000_aud_i2s_ctrl(struct snd_pcm_substream *substream,
	struct snd_soc_dai *cpu_dai, int on)
{
	int stream = substream->stream;
	struct se_i2s_base *i2s = dev_get_drvdata(cpu_dai->dev);

	if (on) {
		if (i2s->i2s_config.i2s_transMode == FULL_DUPLEX_MODE) {
			if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
				/* enable TX */
				regmap_update_bits(i2s->regmap, I2S_CTRL_FDX,
					I2S_FULL_DUPLEX | I2S_FTX_EN,
						I2S_FULL_DUPLEX | I2S_FTX_EN);

				/* set TX interrupt mask */
				regmap_update_bits(i2s->regmap, I2S_CTRL,
					I2S_MASK | I2S_FIFO_EMPTY_MASK,
						I2S_MASK | I2S_FIFO_EMPTY_MASK);
			} else {
				/* enable RX */
				regmap_update_bits(i2s->regmap, I2S_CTRL_FDX, I2S_FULL_DUPLEX
					| I2S_FRX_EN, I2S_FULL_DUPLEX | I2S_FRX_EN);

				/* set RX interrupt mask */
				regmap_update_bits(i2s->regmap, I2S_CTRL_FDX,
					I2S_RI2S_MASK | I2S_RFIFO_FULL_MASK,
						I2S_RI2S_MASK | I2S_RFIFO_FULL_MASK);
			}
		} else {
			if (stream == SNDRV_PCM_STREAM_PLAYBACK)
				/* set half dupled mode TX direction */
				regmap_update_bits(i2s->regmap, I2S_CTRL, I2S_DIR_CFG, I2S_DIR_CFG);
			else
				/* set half dupled mode RX direction */
				regmap_update_bits(i2s->regmap, I2S_CTRL, I2S_DIR_CFG, 0);

			/* set interrupt mask */
			regmap_update_bits(i2s->regmap, I2S_CTRL, I2S_MASK | I2S_FIFO_EMPTY_MASK,
							I2S_MASK | I2S_FIFO_EMPTY_MASK);
		}

		if (i2s->i2s_config.i2s_format == I2S_TDM_FORMAT)
			/* enable able TDM */
			regmap_update_bits(i2s->regmap, TDM_CTRL, TDM_EN, TDM_EN);

		/* enable able I2S */
		regmap_update_bits(i2s->regmap, I2S_CTRL, I2S_EN, I2S_EN);

		/* set all interrupt use individual mask */
		regmap_update_bits(i2s->regmap, I2S_CTRL,
			I2S_INTREQ_MASK, I2S_INTREQ_MASK);
	} else {
		if (i2s->i2s_config.i2s_transMode == FULL_DUPLEX_MODE) {
			if (stream == SNDRV_PCM_STREAM_PLAYBACK)
				regmap_update_bits(i2s->regmap, I2S_CTRL,
					I2S_MASK | I2S_FIFO_EMPTY_MASK | I2S_FIFO_FULL_MASK, 0);
			else {
				/* disable RX interrupt mask */
				regmap_update_bits(i2s->regmap, I2S_CTRL_FDX,
					I2S_RI2S_MASK | I2S_RFIFO_FULL_MASK, 0);
			}
		} else {
			regmap_update_bits(i2s->regmap, I2S_CTRL,
				I2S_MASK | I2S_FIFO_EMPTY_MASK | I2S_FIFO_FULL_MASK, 0);
		}
	}
}

static int se1000_aud_i2s_trigger(struct snd_pcm_substream *substream,
	int cmd, struct snd_soc_dai *cpu_dai)
{
	struct se_i2s_base *i2s = dev_get_drvdata(cpu_dai->dev);

	if (READ_ONCE(i2s->active[substream->stream]) > 1)
		return 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		se1000_aud_i2s_ctrl(substream, cpu_dai, 1);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		se1000_aud_i2s_ctrl(substream, cpu_dai, 0);
		break;
	default:
		dev_err(cpu_dai->dev, "unknown cmd\n");
		return -EINVAL;
	}

	return 0;
}

static int se1000_aud_i2s_setmclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	struct se_i2s_base *i2s = dev_get_drvdata(dai->dev);
	int div = 0;
	int reg = 0;

	if (dir != SND_SOC_CLOCK_OUT) {
		dev_err(dai->dev, "mclk only support output\n");
		return -EINVAL;
	}

	if (freq > AUD_PLL_CLK) {
		dev_err(dai->dev, "mclk must be less than %d\n", AUD_PLL_CLK);
		return -EINVAL;
	}

	if (i2s->mclk_enable) {
		if (i2s->playback != i2s->capture) {
			/* set mclk div */
			div = AUD_PLL_CLK/freq/i2s->mclk_rate_coef/2;

			reg = readl(i2s->ctrl_addr + AUDIO_I2S_MCLK(clk_id));
			reg = reg | I2S_MCLK_DIV_SET(div) | I2S_MCLK_EN;
			writel(reg, i2s->ctrl_addr + AUDIO_I2S_MCLK(clk_id));
		}
	}

	return 0;
}

int se1000_loopback_set(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct se_i2s_base *i2s = snd_soc_component_get_drvdata(component);
	u32 value, reg;

	value = ucontrol->value.integer.value[0];

	if (value > 1)
		return -EINVAL;

	if (i2s->i2s_config.i2s_loopback == value)
		return 0;

	i2s->i2s_config.i2s_loopback = value;

	reg = readl(i2s->ctrl_addr + AUDIO_I2S_LOOPBACK);

	if (value)
		reg = reg | (value << i2s->i2s_id);
	else
		reg = reg & (~(0x1 << i2s->i2s_id));

	writel(reg, i2s->ctrl_addr + AUDIO_I2S_LOOPBACK);

	return 1;
}

int se1000_loopback_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct se_i2s_base *i2s = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = i2s->i2s_config.i2s_loopback;

	return 0;
}

/* I2S BE DAIs */
const struct snd_soc_dai_ops se1000_i2s_ops = {
	.startup	= se1000_aud_i2s_startup,
	.shutdown	= se1000_aud_i2s_shutdown,
	.prepare	= se1000_aud_i2s_prepare,
	.set_fmt	= se1000_aud_i2s_set_fmt,
	.hw_params	= se1000_aud_i2s_hw_params,
	.trigger	= se1000_aud_i2s_trigger,
	.set_sysclk	= se1000_aud_i2s_setmclk,
};

MODULE_DESCRIPTION("SE1000 aud i2s control");
MODULE_AUTHOR("jian.xu <jian.xu@siengine.com>");
MODULE_LICENSE("GPL v2");
