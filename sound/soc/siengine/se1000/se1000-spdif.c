/*
* ALSA SoC platform driver - Siengine SPDIF driver
*
* Copyright (c) 2022-2023 Siengine Inc.
* Author: jian.xu <jian.xu@siengine.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*/

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/pm_runtime.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>
#include <linux/clk-provider.h>
#include "se1000-aud-i2s.h"
#include "se1000-spdif.h"
#include "se1000-generic-dmaengine-pcm.h"

static int se1000_spdif_startup(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct se1000_spdif_base *spdif = snd_soc_dai_get_drvdata(dai);
	int ret;

	regmap_update_bits(spdif->regmap, SPDIF_CTRL, CTRL_CLK_ENABLE, 0);

	ret = clk_prepare_enable(spdif->spdif_clk);
	if (ret) {
		dev_err(spdif->dev, "spdif clk enable failed %d\n", ret);
		return ret;
	}
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		spdif->playback = 1;
	else
		spdif->capture = 1;

	if (spdif->dma_buf[substream->stream])
		memset_io(spdif->dma_buf[substream->stream]->area, 0, SPDIF_PREALLOCATE_BUFF_SIZE);
	return 0;
}

static int se1000_spdif_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct se1000_spdif_base *spdif = snd_soc_dai_get_drvdata(dai);
	unsigned int channels = params_channels(params);
	unsigned int rate = params_rate(params);
	u32 val;

	if (spdif->spdif_config.validitycheck)
		regmap_update_bits(spdif->regmap, SPDIF_CTRL,
			CTRL_VALIDITYCHECK, CTRL_VALIDITYCHECK);

	/* set transmitter sample rate */
	val = SPDIF_WORK_CLK / rate - 1;

	/* set sample rate */
	regmap_update_bits(spdif->regmap, SPDIF_CTRL, CTRL_TSAMPLERATE, val);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* set direction */
		regmap_update_bits(spdif->regmap, SPDIF_CTRL, CTRL_TR_MODE, CTRL_TR_MODE);

		/* set setpreambb */
		regmap_update_bits(spdif->regmap, SPDIF_CTRL, CTRL_SETPREAMBB, 0);

		/* set interrupt */
		regmap_update_bits(spdif->regmap, SPDIF_CTRL,
			CTRL_UNDERR_MASK | CTRL_EMPTY_MASK | CTRL_FULL_MASK | CTRL_SYNCERR_AMSK,
				CTRL_UNDERR_MASK | CTRL_EMPTY_MASK | CTRL_FULL_MASK | CTRL_SYNCERR_AMSK);
	} else {
		/* set direction */
		regmap_update_bits(spdif->regmap, SPDIF_CTRL, CTRL_TR_MODE, 0);

		regmap_update_bits(spdif->regmap, SPDIF_CTRL,
			CTRL_OVRERR_MASK | CTRL_EMPTY_MASK | CTRL_FULL_MASK | CTRL_SYNCERR_AMSK,
				CTRL_OVRERR_MASK | CTRL_EMPTY_MASK | CTRL_FULL_MASK | CTRL_SYNCERR_AMSK);
	}

	switch (channels) {
	case 1:
		regmap_update_bits(spdif->regmap, SPDIF_CTRL, CTRL_CHANNEL_MODE | CTRL_DUPLICATE,
			CTRL_CHANNEL_MODE | CTRL_DUPLICATE_SET(0));
		break;
	case 2:
		regmap_update_bits(spdif->regmap, SPDIF_CTRL, CTRL_CHANNEL_MODE, 0);
		break;
	default:
		dev_err(dai->dev, "error channel setting for SPDIF mode\n");
		return -EINVAL;
	}

	if (spdif->spdif_config.paritycheck)
		regmap_update_bits(spdif->regmap, SPDIF_CTRL, CTRL_PARITYCHECK, CTRL_PARITYCHECK);

	if (spdif->spdif_config.paritygen)
		regmap_update_bits(spdif->regmap, SPDIF_CTRL, CTRL_PARITYGEN, CTRL_PARITYGEN);

	if (spdif->spdif_config.validitycheck)
		regmap_update_bits(spdif->regmap, SPDIF_CTRL,
			CTRL_VALIDITYCHECK, CTRL_VALIDITYCHECK);

	if (spdif->spdif_config.setpreambb) {
		regmap_update_bits(spdif->regmap, SPDIF_CTRL, CTRL_SETPREAMBB, CTRL_SETPREAMBB);
		regmap_update_bits(spdif->regmap, SPDIF_INT_REG, INT_REG_PREAMBLEDEL,
			spdif->spdif_config.preambledel);
	}

	/* set use fifo if */
	regmap_update_bits(spdif->regmap, SPDIF_CTRL, CTRL_USE_FIFO_IF, 0);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* set tx fifo control */
		regmap_update_bits(spdif->regmap, SPDIF_FIFO_CTRL, FIFO_AEMPTY_THRESHOLD,
			FIFO_AEMPTY_TSHOLD_SET(spdif->spdif_config.spdif_fifo_tx_threshold[0]));
		regmap_update_bits(spdif->regmap, SPDIF_FIFO_CTRL, FIFO_AFULL_THRESHOLD,
			FIFO_AFULL_TSHOLD_SET(spdif->spdif_config.spdif_fifo_tx_threshold[1]));
	} else {
		/* set rx fifo control */
		regmap_update_bits(spdif->regmap, SPDIF_FIFO_CTRL, FIFO_AEMPTY_THRESHOLD,
			FIFO_AEMPTY_TSHOLD_SET(spdif->spdif_config.spdif_fifo_rx_threshold[0]));
		regmap_update_bits(spdif->regmap, SPDIF_FIFO_CTRL, FIFO_AFULL_THRESHOLD,
			FIFO_AFULL_TSHOLD_SET(spdif->spdif_config.spdif_fifo_rx_threshold[1]));
	}

	return 0;
}

static void se1000_spdif_ctrl(struct snd_soc_dai *dai, int on)
{
	struct se1000_spdif_base *spdif = snd_soc_dai_get_drvdata(dai);

	if (on)
		regmap_update_bits(spdif->regmap, SPDIF_CTRL, CTRL_SPDIF_ENABLE, CTRL_SPDIF_ENABLE);
	else
		/* clear interrupt mask */
		regmap_update_bits(spdif->regmap, SPDIF_CTRL,
			CTRL_OVRERR_MASK | CTRL_UNDERR_MASK | CTRL_EMPTY_MASK | CTRL_FULL_MASK | CTRL_SYNCERR_AMSK, 0);

}

static int se1000_spdif_trigger(struct snd_pcm_substream *substream,
			int cmd, struct snd_soc_dai *dai)
{
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		se1000_spdif_ctrl(dai, 1);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		se1000_spdif_ctrl(dai, 0);
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		break;
	default:
		dev_err(dai->dev, "unknown cmd\n");
		return -EINVAL;
	}

	return 0;
}

static void se1000_spdif_shutdown(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct se1000_spdif_base *spdif = snd_soc_dai_get_drvdata(dai);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		spdif->playback = 0;
	else
		spdif->capture = 0;

	/* reset SFR registers */
	regmap_update_bits(spdif->regmap, SPDIF_CTRL, CTRL_SPDIF_ENABLE, 0);
	regmap_update_bits(spdif->regmap, SPDIF_CTRL, CTRL_SFR_ENABLE, 0);

	clk_disable_unprepare(spdif->spdif_clk);
}

static const struct snd_soc_dai_ops se1000_spdif_dai_ops = {
	.startup	= se1000_spdif_startup,
	.trigger	= se1000_spdif_trigger,
	.hw_params	= se1000_spdif_hw_params,
	.shutdown	= se1000_spdif_shutdown,
};

static int se1000_spdif_dai_probe(struct snd_soc_dai *dai)
{
	struct se1000_spdif_base *spdif = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &spdif->dma_data[SNDRV_PCM_STREAM_PLAYBACK],
		&spdif->dma_data[SNDRV_PCM_STREAM_CAPTURE]);

	return 0;
}

static struct snd_soc_dai_driver se1000_spdif_dai = {
	.name = "SPDIF",
	.probe = se1000_spdif_dai_probe,
	.id = SE1000_SPDIF_OUT,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE
			| SNDRV_PCM_FMTBIT_S24_LE,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE
			| SNDRV_PCM_FMTBIT_S24_LE,
	},

	.ops = &se1000_spdif_dai_ops,
};

static const struct regmap_config se1000_spdif_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.cache_type = REGCACHE_NONE,
};

static irqreturn_t se1000_spdif_irq_handler(int irq_id, void *dev)
{
	struct se1000_spdif_base *spdif = dev;
	u32 val = 0;

	regmap_update_bits(spdif->regmap, SPDIF_CTRL, CTRL_INTREQ_MASK, 0);

	regmap_read(spdif->regmap, SPDIF_INT_REG, &val);
	if (val & INT_REG_PARITYO) {
		regmap_update_bits(spdif->regmap, SPDIF_INT_REG, INT_REG_PARITYO, 0);
		dev_err(spdif->dev, "SPDIF parity error happen!\n");
	}

	if (val & INT_REG_TDATA_UNDERR) {
		regmap_update_bits(spdif->regmap, SPDIF_INT_REG, INT_REG_TDATA_UNDERR, 0);
		dev_err(spdif->dev, "SPDIF underrun error!\n");
	}

	if (val & INT_REG_RDATA_OVRERR) {
		regmap_update_bits(spdif->regmap, SPDIF_INT_REG, INT_REG_RDATA_OVRERR, 0);
		dev_err(spdif->dev, "SPDIF overrun error!\n");
	}

	if (val & INT_REG_FIFO_EMPTY) {
		regmap_update_bits(spdif->regmap, SPDIF_INT_REG, INT_REG_FIFO_EMPTY, 0);
		dev_err(spdif->dev, "SPDIF FIFO empty!\n");
	}

	if (val & INT_REG_FIFO_FULL) {
		regmap_update_bits(spdif->regmap, SPDIF_INT_REG, INT_REG_FIFO_FULL, 0);
		dev_err(spdif->dev, "SPDIF FIFO full!\n");
	}

	if (val & INT_REG_SYNCERR) {
		regmap_update_bits(spdif->regmap, SPDIF_INT_REG, INT_REG_SYNCERR, 0);
		dev_err(spdif->dev, "receiver synchronization error!\n");
	}

	if (val & INT_REG_LOCK) {
		regmap_update_bits(spdif->regmap, SPDIF_INT_REG, INT_REG_LOCK, 0);
		dev_err(spdif->dev, "receiver become synchronized with input data stream\n");
	}

	if (val & INT_REG_BLOCK_BEGIN) {
		regmap_update_bits(spdif->regmap, SPDIF_INT_REG, INT_REG_BLOCK_BEGIN, 0);
		dev_err(spdif->dev, "start of a new block in received data\n");
	}

	regmap_update_bits(spdif->regmap, SPDIF_CTRL, CTRL_INTREQ_MASK,
		CTRL_INTREQ_MASK);

	return IRQ_HANDLED;
}

static int se1000_spdif_probe(struct platform_device *pdev)
{
	struct se1000_spdif_base *spdif;
	struct resource *res;
	struct device *dev;
	struct device_node *node = pdev->dev.of_node;
	struct se1000_dma_runtime_data *prtd;
	u32 irq_id, ret, dma_handshake_id;
	u32 val;
	int i;

	spdif = devm_kzalloc(&pdev->dev, sizeof(*spdif), GFP_KERNEL);
	if (!spdif)
		return -ENOMEM;

	spdif->dev = &pdev->dev;
	dev = spdif->dev;

	/* register IRQ */
	irq_id = platform_get_irq(pdev, 0);
	if (irq_id < 0) {
		dev_err(dev, "unable to get spdif IRQ number\n");
		return irq_id;
	}

	ret = devm_request_irq(dev, irq_id, se1000_spdif_irq_handler,
			IRQF_TRIGGER_HIGH, pdev->name, (void *)spdif);
	if (ret) {
		dev_err(dev, "could not request_irq for spdif irq\n");
		return ret;
	}

	/* prepare related clocks */
	spdif->spdif_clk = devm_clk_get(&pdev->dev, "spdif_clk");
	if (IS_ERR(spdif->spdif_clk)) {
		dev_err(&pdev->dev, "Can't retrieve spdif bus clock\n");
		return PTR_ERR(spdif->spdif_clk);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	spdif->base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(spdif->base_addr))
		return PTR_ERR(spdif->base_addr);

	spdif->regmap = devm_regmap_init_mmio(&pdev->dev, spdif->base_addr,
			&se1000_spdif_regmap_config);
	if (IS_ERR(spdif->regmap)) {
		dev_err(&pdev->dev, "error initializing regmap: %ld\n",
			PTR_ERR(spdif->regmap));
		return PTR_ERR(spdif->regmap);
	}

	ret = device_property_read_u32_array(dev, "spdif-fifo-tx-threshold",
		spdif->spdif_config.spdif_fifo_tx_threshold, 2);
	if (ret) {
		dev_err(dev, "get spdif-fifo-tx-threshold value failed\n");
		goto err_free_mem;
	}

	ret = device_property_read_u32_array(dev, "spdif-fifo-rx-threshold",
		spdif->spdif_config.spdif_fifo_rx_threshold, 2);
	if (ret) {
		dev_err(dev, "get spdif-fifo-rx-threshold value failed\n");
		goto err_free_mem;
	}

	/* get DMA handshake id */
	ret = device_property_read_u32(dev, "dma-handshake-id", &dma_handshake_id);
	if (ret)
		return ret;

	if (!of_property_read_u32(node, "TX_PARITYGEN", &val))
		spdif->spdif_config.paritygen = val;

	if (!of_property_read_u32(node, "RX_PARITYCHECK", &val))
		spdif->spdif_config.paritycheck = val;

	if (!of_property_read_u32(node, "RX_VALIDITYCHECK", &val))
		spdif->spdif_config.validitycheck = val;

	if (!of_property_read_u32(node, "TX_SETPREAMBB", &val))
		spdif->spdif_config.setpreambb = val;

	if (!of_property_read_u32(node, "TX_PREAMBLEDEL", &val))
		spdif->spdif_config.preambledel = val;

	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (!prtd) {
		dev_err(&pdev->dev, "malloc memory failed\n");
		goto err_free_mem;
	}
	spdif->prtd = prtd;

	/* initialize DMA */
	spdif->dma_data[SNDRV_PCM_STREAM_PLAYBACK].addr = res->start + SPDIF_FIFO_ADDR;
	spdif->dma_data[SNDRV_PCM_STREAM_PLAYBACK].slave_id = dma_handshake_id;
	spdif->dma_data[SNDRV_PCM_STREAM_PLAYBACK].maxburst = 4;
	spdif->dma_data[SNDRV_PCM_STREAM_PLAYBACK].fifo_size = 256;
	spdif->dma_data[SNDRV_PCM_STREAM_PLAYBACK].addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;

	spdif->dma_data[SNDRV_PCM_STREAM_CAPTURE].addr = res->start + SPDIF_FIFO_ADDR;
	spdif->dma_data[SNDRV_PCM_STREAM_CAPTURE].slave_id = dma_handshake_id;
	spdif->dma_data[SNDRV_PCM_STREAM_CAPTURE].maxburst = 4;
	spdif->dma_data[SNDRV_PCM_STREAM_CAPTURE].fifo_size = 256;
	spdif->dma_data[SNDRV_PCM_STREAM_PLAYBACK].addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;

	for_each_pcm_streams(i) {
		spdif->dma_buf[i] = kzalloc(sizeof(struct snd_dma_buffer), GFP_KERNEL);
		if (!spdif->dma_buf[i]) {
			ret = -ENOMEM;
			goto err_free_mem;
		}

		ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV, dev,
				SPDIF_PREALLOCATE_BUFF_SIZE, spdif->dma_buf[i]);
		if (ret) {
			dev_err(&pdev->dev, "failed to allocate DMA buffer!\n");
			goto err_free_mem;
		}
		memset_io(spdif->dma_buf[i]->area, 0, SPDIF_PREALLOCATE_BUFF_SIZE);
	}
	platform_set_drvdata(pdev, spdif);
	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev))
		goto err_pm_disable;
	pm_runtime_get_sync(&pdev->dev);

	ret = se1000_snd_dmaengine_pcm_register(&pdev->dev, NULL, SND_DMAENGINE_PCM_FLAG_HALF_DUPLEX, &se1000_spdif_dai,
								NULL, 0);
	if (ret) {
		dev_err(&pdev->dev, "Could not dmaengine register failed!\n");
		goto err_pm_disable;
	}

	return 0;

err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_free_mem:
	if (prtd)
		kfree(prtd);
	if (spdif->dma_buf[0])
		kfree(spdif->dma_buf[0]);
	if (spdif->dma_buf[1])
		kfree(spdif->dma_buf[1]);
	if (spdif)
		kfree(spdif);

	return ret;
}

static int se1000_spdif_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	snd_dmaengine_pcm_unregister(&pdev->dev);
	snd_soc_unregister_component(&pdev->dev);

	return 0;
}

static const struct of_device_id se1000_spdif_match[] = {
	{ .compatible = "siengine,se1000-spdif", },
	{},
};

MODULE_DEVICE_TABLE(of, se1000_spdif_match);

static struct platform_driver se1000_spdif_driver = {
	.probe = se1000_spdif_probe,
	.remove = se1000_spdif_remove,
	.driver = {
		.name = "siengine-spdif",
		.of_match_table = se1000_spdif_match,
	},
};

module_platform_driver(se1000_spdif_driver);
MODULE_DESCRIPTION("Siengine SPDIF Controller Driver");
MODULE_AUTHOR("jian.xu, <jian.xu@siengine.com>");
MODULE_LICENSE("GPL v2");
