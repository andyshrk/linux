/*
 * Siengine ALSA SoC platform driver for SE1000
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

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>
#include <linux/dma-mapping.h>
#include <linux/clk-provider.h>
#include "se1000-aud-i2s.h"
#include "se1000-generic-dmaengine-pcm.h"

void se1000_i2s_early_clk(struct se_i2s_base *i2s);

static const struct snd_pcm_hardware se1000_aud_hardware = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED
		| SNDRV_PCM_INFO_RESUME | SNDRV_PCM_INFO_MMAP_VALID,
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE
			| SNDRV_PCM_FMTBIT_S32_LE,
	.rate_min = 8000,
	.rate_max = 192000,
	.channels_min = 1,
	.channels_max = 16,
	.period_bytes_min = 256,
	.period_bytes_max = 1024 * 64,
	.periods_min = 2,
	.periods_max = 4096,
	.buffer_bytes_max = SIZE_MAX,
	.fifo_size = 256,
};

static int se1000_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct se_i2s_base *i2s = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &i2s->dma_data[SNDRV_PCM_STREAM_PLAYBACK],
		&i2s->dma_data[SNDRV_PCM_STREAM_CAPTURE]);

	return 0;
}

static struct snd_soc_dai_driver se1000_pcm_dai_driver[] = {
	{
		.name = "I2S0",
		.id = SE1000_I2S0,
		.probe = se1000_i2s_dai_probe,
		.playback = {
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.capture = {
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
		},
		.ops = &se1000_i2s_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "I2S1",
		.id = SE1000_I2S1,
		.probe = se1000_i2s_dai_probe,
		.playback = {
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.capture = {
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.ops = &se1000_i2s_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "I2S2",
		.id = SE1000_I2S2,
		.probe = se1000_i2s_dai_probe,
		.playback = {
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.capture = {
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.ops = &se1000_i2s_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "I2S3",
		.id = SE1000_I2S3,
		.probe = se1000_i2s_dai_probe,
		.playback = {
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.capture = {
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.ops = &se1000_i2s_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "I2S4",
		.id = SE1000_I2S4,
		.probe = se1000_i2s_dai_probe,
		.playback = {
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.capture = {
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE)
			},
		.ops = &se1000_i2s_ops,
	},
	{
	.name = "I2S5",
	.id = SE1000_I2S5,
	.probe = se1000_i2s_dai_probe,
	.playback = {
		.channels_min = 1,
		.channels_max = 16,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = (SNDRV_PCM_FMTBIT_S16_LE
			| SNDRV_PCM_FMTBIT_S24_LE
			| SNDRV_PCM_FMTBIT_S32_LE)
		},
	.capture = {
		.channels_min = 1,
		.channels_max = 16,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = (SNDRV_PCM_FMTBIT_S16_LE
			| SNDRV_PCM_FMTBIT_S24_LE
			| SNDRV_PCM_FMTBIT_S32_LE)
		},
	.ops = &se1000_i2s_ops,
	.symmetric_rates = 1,
	},
	{
	.name = "I2S6",
	.id = SE1000_I2S6,
	.probe = se1000_i2s_dai_probe,
	.playback = {
		.channels_min = 1,
		.channels_max = 16,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = (SNDRV_PCM_FMTBIT_S16_LE
			| SNDRV_PCM_FMTBIT_S24_LE
			| SNDRV_PCM_FMTBIT_S32_LE)
		},
	.capture = {
		.channels_min = 1,
		.channels_max = 16,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = (SNDRV_PCM_FMTBIT_S16_LE
			| SNDRV_PCM_FMTBIT_S24_LE
			| SNDRV_PCM_FMTBIT_S32_LE)
		},
	.ops = &se1000_i2s_ops,
	.symmetric_rates = 1,
	},
};

static const struct snd_kcontrol_new se1000_i2s_control[] = {
	SOC_SINGLE_BOOL_EXT("I2S0 loopback Switch", 0, se1000_loopback_get,
			se1000_loopback_set),
	SOC_SINGLE_BOOL_EXT("I2S1 loopback Switch", 1, se1000_loopback_get,
			se1000_loopback_set),
	SOC_SINGLE_BOOL_EXT("I2S2 loopback Switch", 2, se1000_loopback_get,
			se1000_loopback_set),
	SOC_SINGLE_BOOL_EXT("I2S3 loopback Switch", 3, se1000_loopback_get,
			se1000_loopback_set),
	SOC_SINGLE_BOOL_EXT("I2S4 loopback Switch", 4, se1000_loopback_get,
			se1000_loopback_set),
	SOC_SINGLE_BOOL_EXT("I2S5 loopback Switch", 5, se1000_loopback_get,
			se1000_loopback_set),
};

#if 0
int se1000_i2s_suspend(struct snd_soc_component *component)
{
	struct se_i2s_base *i2s = snd_soc_component_get_drvdata(component);

	if (i2s->playback || i2s->capture) {
		if (__clk_is_enabled(i2s->i2s_clk)) {
			clk_disable_unprepare(i2s->i2s_clk);
			dev_err(i2s->dev, "%s clk_disable_unprepare %d\n", __func__, __LINE__);
		}
	}

	pinctrl_pm_select_sleep_state(i2s->dev);

	return 0;
};

int se1000_i2s_resume(struct snd_soc_component *component)
{
	struct se_i2s_base *i2s = snd_soc_component_get_drvdata(component);
	int ret;

	#ifdef CONFIG_SE1000_STR
	pinctrl_pm_force_default_state(i2s->dev);
	#else
	pinctrl_pm_select_default_state(i2s->dev);
	#endif

	if (i2s->playback || i2s->capture) {
		se1000_i2s_early_clk(i2s);
		ret = clk_prepare_enable(i2s->i2s_clk);
		if (ret) {
			dev_err(i2s->dev, "clk_enable failed: %d\n", ret);
			return ret;
		}
	}

	return 0;
};

static const struct snd_soc_component_driver se1000_aud_component_driver[] = {
	{
		.name = "se1000-pcm-common0",
		.controls = &se1000_i2s_control[0],
		.num_controls = 1,
		.resume = se1000_i2s_resume,
		.suspend = se1000_i2s_suspend,
	},
	{
		.name = "se1000-pcm-common1",
		.controls = &se1000_i2s_control[1],
		.num_controls = 1,
		.resume = se1000_i2s_resume,
		.suspend = se1000_i2s_suspend,
	},
	{
		.name = "se1000-pcm-common2",
		.controls = &se1000_i2s_control[2],
		.num_controls = 1,
		.resume = se1000_i2s_resume,
		.suspend = se1000_i2s_suspend,
	},
	{
		.name = "se1000-pcm-common3",
		.controls = &se1000_i2s_control[3],
		.num_controls = 1,
		.resume = se1000_i2s_resume,
		.suspend = se1000_i2s_suspend,
	},
	{
		.name = "se1000-pcm-common4",
		.controls = &se1000_i2s_control[4],
		.num_controls = 1,
		.resume = se1000_i2s_resume,
		.suspend = se1000_i2s_suspend,
	},
	{
		.name = "se1000-pcm-common5",
		.controls = &se1000_i2s_control[5],
		.num_controls = 1,
		.resume = se1000_i2s_resume,
		.suspend = se1000_i2s_suspend,
	},
};
#endif

static const struct regmap_config se1000_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.cache_type = REGCACHE_NONE,
};

void se1000_i2s_early_clk(struct se_i2s_base *i2s)
{
	u32 reg = 0, val = 0;

	if (i2s->i2s_id == 0) {
		/* enable I2S0 clk */
		reg = readl(i2s->base_addr + AUDIO_DEVCKE_SET);
		reg = reg | I2S_CLK_EN_SET(0);
		writel(reg, i2s->base_addr);

		/* enable I2S0 bck clk */
		val = val | I2S_EN | I2S_DIR_CFG | I2S_MS_CFG | I2S_CHN_WIDTH_SET(2) |
		I2S_SFR_RST | I2S_WS_MODE_SET(1) | I2S_AUDIO_MODE_SET(0) |
			I2S_SCK_POLAR_SET(1) | I2S_WS_POLAR_SET(1);

		regmap_write(i2s->regmap, I2S_CTRL, val);
		regmap_update_bits(i2s->regmap, I2S_SRES, I2S_TX_SAMP_RES,
			I2S_TX_SAMP_RES_SET(16 - 1));

		/* set sample rate 48k */
		regmap_update_bits(i2s->regmap, I2S_SRATE, I2S_SAMPLE_RATE,
			I2S_SAMPLE_RATE_SET(288));
	}
}

static const struct snd_dmaengine_pcm_config pcm_hardware_config = {
	.prepare_slave_config = se1000_snd_dmaengine_pcm_prepare_slave_config,
	.pcm_hardware = &se1000_aud_hardware,
};

static int se1000_pcm_dev_probe(struct platform_device *pdev)
{
	struct se_i2s_base *i2s;
	struct resource *res;
	struct device *dev;
	const struct device_node *node = pdev->dev.of_node;
	struct device_node *control_node;
	u32 dma_handshake_id[2];
	int irq_id, ret;
	u32 i2s_trmode;
	struct se1000_dma_runtime_data *prtd;
	u32 i2s_mclk_parms[2];
	u32 i2s_dai_fmt_parms[3];
	u32 irq_enable = 0;

	i2s = devm_kzalloc(&pdev->dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;

	i2s->dev = &pdev->dev;
	dev = i2s->dev;

	ret = of_property_read_u32(node, "i2s-id", &i2s->i2s_id);
	if (ret < 0)
		return ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	i2s->base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(i2s->base_addr))
		return PTR_ERR(i2s->base_addr);

	i2s->regmap = devm_regmap_init_mmio(&pdev->dev, i2s->base_addr, &se1000_regmap_config);
	if (IS_ERR(i2s->regmap)) {
		dev_err(&pdev->dev, "error initializing regmap: %ld\n",
			PTR_ERR(i2s->regmap));
		return PTR_ERR(i2s->regmap);
	}

	mutex_init(&i2s->irq_alloc_lock);

	i2s->se_aud_hardware = &se1000_aud_hardware;

	ret = of_property_read_u32(node, "irq-enable", &irq_enable);
	if (ret < 0)
		irq_enable = 0;

	if (irq_enable) {
		/* register IRQ */
		irq_id = platform_get_irq(pdev, 0);
		if (irq_id < 0) {
			dev_err(dev, "unable to get i2s IRQ number\n");
			ret = irq_id;
			goto err_free_mem;
		}

		ret = devm_request_irq(dev, irq_id, se1000_i2s_irq_handler,
					IRQF_TRIGGER_HIGH, pdev->name, (void *)i2s);
		if (ret) {
			dev_err(dev, "could not request_irq for i2s-irq\n");
			goto err_free_mem;
		}
	}

	ret = device_property_read_u32_array(dev, "i2s-fifo-tx-threshold",
		i2s->i2s_config.i2s_fifo_tx_threshold, 2);
	if (ret) {
		dev_err(dev, "get i2s-fifo-tx-threshold value failed\n");
		goto err_free_mem;
	}

	ret = device_property_read_u32_array(dev, "i2s-fifo-rx-threshold",
		i2s->i2s_config.i2s_fifo_rx_threshold, 2);
	if (ret) {
		dev_err(dev, "get i2s-fifo-rx-threshold value failed\n");
		goto err_free_mem;
	}

	/*set i2s transfer mode */
	ret = device_property_read_u32(dev, "i2s-transMode", &i2s_trmode);
	if (ret)
		return ret;
	i2s->i2s_config.i2s_transMode = i2s_trmode;

	ret = device_property_read_u32_array(dev, "i2s-mclk", i2s_mclk_parms, 2);
	if (ret) {
		dev_err(&pdev->dev, "Property 'i2s-mclk' missing or invalid\n");
		return ret;
	}

	i2s->mclk_enable = i2s_mclk_parms[0];
	i2s->mclk_rate_coef = i2s_mclk_parms[1];

	/* get I2S handshake id */
	ret = device_property_read_u32_array(dev, "dma-handshake-id", dma_handshake_id, 2);
	if (ret)
		return ret;

	/* get i2s control regmap */
	control_node = of_parse_phandle(pdev->dev.of_node,
					 "siengine,i2sctrl", 0);
	if (!control_node) {
		dev_err(&pdev->dev, "Property 'control_node' missing or invalid\n");
		return -EINVAL;
	}

	ret = device_property_read_u32_array(dev, "i2s-dai-fmt", i2s_dai_fmt_parms, 3);
	if (ret) {
		dev_err(&pdev->dev, "Property 'i2s-dai-fmt' missing or invalid\n");
		return ret;
	} else {
		i2s->i2s_config.i2s_format = i2s_dai_fmt_parms[0];
		i2s->i2s_config.i2s_mode = i2s_dai_fmt_parms[1];
		i2s->i2s_config.i2s_clk_polar = i2s_dai_fmt_parms[2];
	}

	i2s->ctrl_addr = of_iomap(control_node, 0);
	if (IS_ERR(i2s->base_addr))
		return PTR_ERR(i2s->base_addr);

	i2s->i2s_clk = devm_clk_get(&pdev->dev, "i2s_clk");
	if (IS_ERR(i2s->i2s_clk)) {
		dev_err(&pdev->dev, "Can't retrieve i2s i2s_clk\n");
		return PTR_ERR(i2s->i2s_clk);
	}

#ifdef SE1000_AUD_I2S_EARLY_CLK
	se1000_i2s_early_clk(i2s);
#endif

	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (!prtd) {
		dev_err(&pdev->dev, "malloc memory failed\n");
		return -ENOMEM;
	}
	prtd->tdm_channels = 0; /* legacy card don't limit tdm slots */
	i2s->prtd = prtd;

	/* initialize DMA */
	i2s->dma_data[SNDRV_PCM_STREAM_PLAYBACK].addr = res->start + I2S_FIF0_ADDR;
	i2s->dma_data[SNDRV_PCM_STREAM_PLAYBACK].slave_id = dma_handshake_id[SNDRV_PCM_STREAM_PLAYBACK];
	i2s->dma_data[SNDRV_PCM_STREAM_PLAYBACK].maxburst = 4;
	i2s->dma_data[SNDRV_PCM_STREAM_PLAYBACK].fifo_size = 256;

	i2s->dma_data[SNDRV_PCM_STREAM_CAPTURE].addr = res->start + I2S_FIF0_ADDR;
	i2s->dma_data[SNDRV_PCM_STREAM_CAPTURE].slave_id = dma_handshake_id[SNDRV_PCM_STREAM_CAPTURE];
	i2s->dma_data[SNDRV_PCM_STREAM_CAPTURE].maxburst = 4;
	i2s->dma_data[SNDRV_PCM_STREAM_CAPTURE].fifo_size = 256;

	platform_set_drvdata(pdev, i2s);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev))
		goto err_pm_disable;
	pm_runtime_get_sync(&pdev->dev);

	ret = se1000_snd_dmaengine_pcm_register(&pdev->dev, &pcm_hardware_config, 0, &se1000_pcm_dai_driver[i2s->i2s_id],
								&se1000_i2s_control[i2s->i2s_id], 1);

	if (ret) {
		dev_err(&pdev->dev, "dmaengine register failed!\n");
		goto err_pm_disable;
	}
	return 0;

err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_free_mem:
	kfree(i2s);

	return ret;
}

static int se1000_pcm_dev_remove(struct platform_device *pdev)
{
	struct se_i2s_base *i2s = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);

	se1000_snd_dmaengine_pcm_unregister(&pdev->dev);
	snd_soc_unregister_component(&pdev->dev);
	kfree(i2s->prtd);
	kfree(i2s);
	return 0;
}

static const struct of_device_id se1000_pcm_dts_match[] = {
	{ .compatible = "siengine,se1000-i2s", },
	{},
};
MODULE_DEVICE_TABLE(of, se1000_pcm_dts_match);

static struct platform_driver se1000_pcm_platform_driver = {
	.driver = {
		   .name = "se1000-audio",
		   .of_match_table = se1000_pcm_dts_match,
	},
	.probe = se1000_pcm_dev_probe,
	.remove = se1000_pcm_dev_remove,
};

module_platform_driver(se1000_pcm_platform_driver);

MODULE_DESCRIPTION("Siengine SE1000 ALSA SoC platform driver");
MODULE_AUTHOR("jian.xu <jian.xu@siengine.com>");
MODULE_LICENSE("GPL v2");
