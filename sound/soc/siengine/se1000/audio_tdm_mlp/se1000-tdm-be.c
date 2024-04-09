/*
 * SPDX-License-Identifier: GPL-2.0.
 * sound/soc/siengine/audio_tdm_mlp
 * Copyright (C) 2022-2023 SiEngine or its affiliates
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
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/consumer.h>

#include "../se1000-aud-i2s.h"

static int se1000_tdm_be_dai_probe(struct snd_soc_dai *dai)
{
	struct se_i2s_base *i2s = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &i2s->dma_data[SNDRV_PCM_STREAM_PLAYBACK],
		&i2s->dma_data[SNDRV_PCM_STREAM_CAPTURE]);

	return 0;
}

static struct snd_soc_dai_driver se1000_be_dai_driver = {
	.name = "tdm_hw",
	.id = SE1000_I2S3,
	.probe = se1000_tdm_be_dai_probe,
	.playback = {
		.stream_name = "TDM3_HW Playback",
		.channels_min = 1,
		.channels_max = 16,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = (SNDRV_PCM_FMTBIT_S16_LE
			| SNDRV_PCM_FMTBIT_S24_LE
			| SNDRV_PCM_FMTBIT_S32_LE)
		},
	.capture = {
		.stream_name = "TDM3_HW Capture",
		.channels_min = 1,
		.channels_max = 16,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = (SNDRV_PCM_FMTBIT_S16_LE
			| SNDRV_PCM_FMTBIT_S24_LE
			| SNDRV_PCM_FMTBIT_S32_LE)
		},
	.ops = &se1000_i2s_ops,
	.symmetric_rates = 1,
};

static const struct snd_kcontrol_new se1000_tdm_be_control[] = {
	SOC_SINGLE_BOOL_EXT("TDM Be loopback Switch", 0, se1000_loopback_get,
			se1000_loopback_set),
};

static const struct snd_soc_component_driver se1000_be_component_driver = {
	.name = "se1000-be-common",
	.controls = &se1000_tdm_be_control[0],
	.num_controls = ARRAY_SIZE(se1000_tdm_be_control),
};

static const struct regmap_config se1000_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.cache_type = REGCACHE_NONE,
};

static int se1000_pcm_dev_be_probe(struct platform_device *pdev)
{
	struct se_i2s_base *i2s;
	struct resource *res;
	struct device *dev;
	const struct device_node *node = pdev->dev.of_node;
	struct device_node *control_node;
	u32 dma_handshake_id[2];
	int irq_id, ret;
	u32 i2s_trmode;
	u32 i2s_mclk_parms[2];
	u32 i2s_dai_fmt_parms[3];
	u32 irq_enable;

	i2s = devm_kzalloc(&pdev->dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;

	i2s->dev = &pdev->dev;
	dev = i2s->dev;

	ret = of_property_read_u32(node, "i2s-id", &i2s->i2s_id);
	if (ret < 0) {
		dev_err(dev, "get i2s id failed\n");
		goto err_free_mem;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i2s->base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(i2s->base_addr)) {
		dev_err(dev, "i2s base_addr ioremap failed\n");
		ret = PTR_ERR(i2s->base_addr);
		goto err_free_mem;
	}

	i2s->regmap = devm_regmap_init_mmio(&pdev->dev, i2s->base_addr, &se1000_regmap_config);
	if (IS_ERR(i2s->regmap)) {
		dev_err(&pdev->dev, "error initializing regmap: %ld\n",
			PTR_ERR(i2s->regmap));
		ret = PTR_ERR(i2s->regmap);
		goto err_free_mem;
	}

	ret = of_property_read_u32(node, "irq-enable", &irq_enable);
	if (ret < 0) {
		dev_err(dev, "get irq_enable failed\n");
		goto err_free_mem;
	}

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

	mutex_init(&i2s->irq_alloc_lock);

	ret = device_property_read_u32_array(dev, "i2s-fifo-tx-threshold",
		i2s->i2s_config.i2s_fifo_tx_threshold, 2);
	if (ret) {
		dev_err(dev, "get i2s-fifo-aempty value failed\n");
		goto err_free_mem;
	}

	ret = device_property_read_u32_array(dev, "i2s-fifo-rx-threshold",
		i2s->i2s_config.i2s_fifo_rx_threshold, 2);
	if (ret) {
		dev_err(dev, "get i2s-fifo-aempty value failed\n");
		goto err_free_mem;
	}

	/*set i2s transfer mode */
	ret = device_property_read_u32(dev, "i2s-transMode", &i2s_trmode);
	if (ret) {
		dev_err(dev, "get i2s-transMode value failed\n");
		goto err_free_mem;
	}

	i2s->i2s_config.i2s_transMode = i2s_trmode;

	ret = device_property_read_u32_array(dev, "i2s-mclk", i2s_mclk_parms, 2);
	if (ret) {
		dev_err(&pdev->dev, "Property 'i2s-mclk' missing or invalid\n");
		goto err_free_mem;
	}

	i2s->mclk_enable = i2s_mclk_parms[0];
	i2s->mclk_rate_coef = i2s_mclk_parms[1];

	/* get I2S handshake id */
	ret = device_property_read_u32_array(dev, "dma-handshake-id", dma_handshake_id, 2);
	if (ret) {
		dev_err(&pdev->dev, "Property 'dma-handshake-id' missing or invalid\n");
		goto err_free_mem;
	}

	/* get i2s control regmap */
	control_node = of_parse_phandle(pdev->dev.of_node,
					 "siengine,i2sctrl", 0);
	if (!control_node) {
		dev_err(&pdev->dev, "Property 'control_node' missing or invalid\n");
		goto err_free_mem;
	}

	ret = device_property_read_u32_array(dev, "i2s-dai-fmt", i2s_dai_fmt_parms, 3);
	if (ret) {
		dev_err(&pdev->dev, "Property 'i2s-dai-fmt' missing or invalid\n");
		goto err_free_mem;
	} else {
		i2s->i2s_config.i2s_format = i2s_dai_fmt_parms[0];
		i2s->i2s_config.i2s_mode = i2s_dai_fmt_parms[1];
		i2s->i2s_config.i2s_clk_polar = i2s_dai_fmt_parms[2];
	}

	i2s->ctrl_addr = of_iomap(control_node, 0);
	if (IS_ERR(i2s->ctrl_addr)) {
		dev_err(dev, "i2s base_addr ioremap failed\n");
		ret = PTR_ERR(i2s->ctrl_addr);
		goto err_free_mem;
	}

	i2s->i2s_clk = devm_clk_get(&pdev->dev, "i2s_clk");
	if (IS_ERR(i2s->i2s_clk)) {
		dev_err(&pdev->dev, "Can't retrieve i2s i2s_clk\n");
		ret = PTR_ERR(i2s->i2s_clk);
		goto err_free_mem;
	}

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

	ret = devm_snd_soc_register_component(&pdev->dev, &se1000_be_component_driver,
		&se1000_be_dai_driver, 1);
	if (ret) {
		dev_warn(dev, "err_dai_component\n");
		goto err_dai_component;
	}

	return 0;

err_dai_component:
	snd_soc_unregister_component(&pdev->dev);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_free_mem:
	kfree(i2s);

	return ret;
}

static int se1000_pcm_dev_be_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	snd_dmaengine_pcm_unregister(&pdev->dev);
	snd_soc_unregister_component(&pdev->dev);

	return 0;
}

static const struct of_device_id se1000_pcm_dts_match[] = {
	{ .compatible = "siengine,se1000-i2s-be", },
	{},
};
MODULE_DEVICE_TABLE(of, se1000_pcm_dts_match);

static struct platform_driver se1000_pcm_platform_driver = {
	.driver = {
		.name = "se1000-audio-be",
		.of_match_table = se1000_pcm_dts_match,
	},
	.probe = se1000_pcm_dev_be_probe,
	.remove = se1000_pcm_dev_be_remove,
};

module_platform_driver(se1000_pcm_platform_driver);

MODULE_DESCRIPTION("Siengine SE1000 ALSA SoC platform driver");
MODULE_AUTHOR("jian.xu <jian.xu@siengine.com>");
MODULE_LICENSE("GPL v2");
