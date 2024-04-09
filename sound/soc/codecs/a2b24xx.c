/*
 * A2B24XX driver
 *
 * Copyright 2019 Analog Devices Inc.
 *  Author: ADI Automotive Software Team, Bangalore
 *
 * Licensed under the GPL-2.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#ifdef A2B_SETUP_ALSA
#include "adi_a2b_i2c_commandlist.h"
#endif
#include "a2b24xx.h"

struct a2b24xx {
	struct regmap *regmap;
	unsigned int a2b_id;
	unsigned int sysclk;
	enum a2b24xx_sysclk_src sysclk_src;
	enum a2b24xx_type type;

	struct snd_pcm_hw_constraint_list constraints;

	struct device *dev;
	void (*switch_mode)(struct device *dev);

	unsigned int max_master_fs;
	bool master;
};

static const struct reg_default a2b24xx_reg_defaults[] = { {0x00, 0x50} };

/* An example control -  no specific functionality */
static const DECLARE_TLV_DB_MINMAX_MUTE(a2b24xx_control, 0, 0);

#define A2B24XX_CONTROL(x) \
	SOC_SINGLE_TLV("A2B" #x "Template", \
		2, \
		0, 255, 1, a2b24xx_control)

/* example control */
static const struct snd_kcontrol_new a2b24xx_snd_controls[] = {A2B24XX_CONTROL(0),
		A2B24XX_CONTROL(1), };

#ifdef A2B_SETUP_ALSA
/****************************************************************************/
/*
 @brief          This function calculates reg value based on width and adds
 it to the data array

 @param [in]     pDstBuf		Pointer to destination array
 @param [in]     nAddrwidth		Data unpacking boundary(1 byte / 2 byte /4 byte )
 @param [in]     nAddr			Number of words to be copied

 @return          Return code
 - 0: Success
 - 1: Failure
 */
/********************************************************************************/
static void adi_a2b_Concat_Addr_Data(unsigned char pDstBuf[],
		unsigned int nAddrwidth, unsigned int nAddr)
{
	/* Store the read values in the place holder */
	switch (nAddrwidth) {
	/* Byte */
	case 1u:
		pDstBuf[0u] = (unsigned char)nAddr;
		break;
		/* 16 bit word*/
	case 2u:

		pDstBuf[0u] = (unsigned char)(nAddr >> 8u);
		pDstBuf[1u] = (unsigned char)(nAddr & 0xFFu);

		break;
		/* 24 bit word */
	case 3u:
		pDstBuf[0u] = (unsigned char)((nAddr & 0xFF0000u) >> 16u);
		pDstBuf[1u] = (unsigned char)((nAddr & 0xFF00u) >> 8u);
		pDstBuf[2u] = (unsigned char)(nAddr & 0xFFu);
		break;

		/* 32 bit word */
	case 4u:
		pDstBuf[0u] = (unsigned char)(nAddr >> 24u);
		pDstBuf[1u] = (unsigned char)((nAddr & 0xFF0000u) >> 16u);
		pDstBuf[2u] = (unsigned char)((nAddr & 0xFF00u) >> 8u);
		pDstBuf[3u] = (unsigned char)(nAddr & 0xFFu);
		break;

	default:
		break;
	}
}

static int adi_a2b_I2CWrite(struct device *dev, unsigned short devAddr,
		unsigned short count, char *bytes)
{
	struct i2c_client *client = to_i2c_client(dev);

	client->addr = devAddr;
	return i2c_master_send(client, bytes, count);
}

/****************************************************************************/
/*
 @brief          This function does A2B network discovery
 and the peripheral configuration
 @return          None

*/
/********************************************************************************/
static void adi_a2b_NetworkSetup(struct device *dev)
{
	ADI_A2B_DISCOVERY_CONFIG *pOPUnit;
	unsigned int nIndex, nIndex1;
	/* Maximum number of writes */
	static unsigned char aDataBuffer[6000];
	static unsigned char aDataWriteReadBuf[4u];
	unsigned int nDelayVal;

	/* Loop over all the configuration */
	for (nIndex = 0; nIndex < CONFIG_LEN; nIndex++) {
		pOPUnit = &gaA2BConfig[nIndex];
		/* Operation code*/
		switch (pOPUnit->eOpCode) {
		/* Write */
		case WRITE_VALUE:
			adi_a2b_Concat_Addr_Data(&aDataBuffer[0u],
				pOPUnit->nAddrWidth, pOPUnit->nAddr);
			(void)memcpy(&aDataBuffer[pOPUnit->nAddrWidth],
				pOPUnit->paConfigData, pOPUnit->nDataCount);
			//printk("Operation number %d\n", nIndex);
			/* PAL Call, replace with custom implementation */
			adi_a2b_I2CWrite(dev, pOPUnit->nDeviceAddr,
				(pOPUnit->nAddrWidth + pOPUnit->nDataCount), &aDataBuffer[0u]);
			break;

		/* Read */
		case READ_VALUE:
			(void)memset(&aDataBuffer[0u], 0u, pOPUnit->nDataCount);
			adi_a2b_Concat_Addr_Data(&aDataWriteReadBuf[0u],
				pOPUnit->nAddrWidth, pOPUnit->nAddr);
			/* Couple of milli seconds should be OK */
			mdelay(2);
			break;

		/* Delay */
		case DELAY:
			nDelayVal = 0u;
			for (nIndex1 = 0u; nIndex1 < pOPUnit->nDataCount; nIndex1++) {
				nDelayVal = pOPUnit->paConfigData[nIndex1] | nDelayVal << 8u;
			}

			mdelay(nDelayVal);
			break;

		default:
			break;
		}
	}
}

#endif

/* Template functions */
static int a2b24xx_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	//struct snd_soc_component *component = dai->component;
	//struct a2b24xx *a2b24xx = snd_soc_component_get_drvdata(component);
	//unsigned int rate = params_rate(params);
	int ret = 0u;

	/* Add custom functionality */

	return ret;
}


static int a2b24xx_set_tdm_slot(struct snd_soc_dai *dai,
		unsigned int tx_mask, unsigned int rx_mask, int slots, int width)
{
	/* Add custom functionality */

	return 0;
}

static int a2b24xx_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	//struct a2b24xx *a2b24xx = snd_soc_component_get_drvdata(dai->component);

	/* Add custom functionality */

	return 0;
}

static int a2b24xx_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	//struct a2b24xx *a2b24xx = snd_soc_component_get_drvdata(dai->component);
	int ret = 0;

	return ret;

}

static int a2b24xx_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	//struct a2b24xx *a2b24xx = snd_soc_component_get_drvdata(dai->component);

	/* Add custom functionality */
	return 0;
}

static const struct snd_soc_dai_ops a2b24xx_dai_ops = {
	.startup = a2b24xx_startup,
	.hw_params = a2b24xx_hw_params,
	.mute_stream = a2b24xx_mute,
	.set_fmt = a2b24xx_set_dai_fmt,
	.set_tdm_slot = a2b24xx_set_tdm_slot,
};

static struct snd_soc_dai_driver a2b24xx_dai = {
	.name = "a2b24xx-hifi",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 32,
		.rates = SNDRV_PCM_RATE_KNOT,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE
			| SNDRV_PCM_FMTBIT_S32_LE,
		.sig_bits = 24,
	},
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 32,
		.rates = SNDRV_PCM_RATE_KNOT,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE
			| SNDRV_PCM_FMTBIT_S32_LE,
		.sig_bits = 24,
	},
	.ops = &a2b24xx_dai_ops,
};

/* Supported rates */
static const unsigned int a2b24xx_rates[] = { 1500,
	2000, 2400, 3000, 8000, 12000, 24000, 48000, };

/* set system clock */
static int a2b24xx_set_sysclk(struct snd_soc_component *component, int clk_id,
		int source, unsigned int freq, int dir)
{
	//struct a2b24xx *a2b24xx = snd_soc_component_get_drvdata(component);
	//unsigned int mask = 0;
	//unsigned int clk_src;
	unsigned int ret = 0;

	/* No functionality */

	return ret;
}

/* Codec probe */
static int a2b24xx_component_probe(struct snd_soc_component *component)
{
	//struct a2b24xx *a2b24xx = snd_soc_component_get_drvdata(component);
	int ret = 0;

#ifdef A2B_SETUP_ALSA
	/* Setting up A2B network */
	adi_a2b_NetworkSetup(component->dev);
#endif

	return ret;
}

static const struct snd_soc_component_driver a2b24xx_component_driver[] = {
	{
		.probe = a2b24xx_component_probe,
		.set_sysclk = a2b24xx_set_sysclk,
		.controls = &a2b24xx_snd_controls[0],
		.num_controls = 1,
	},
	{
		.probe = a2b24xx_component_probe,
		.set_sysclk = a2b24xx_set_sysclk,
		.controls = &a2b24xx_snd_controls[1],
		.num_controls = 1,
	},
};

/* driver probe */
int a2b24xx_probe(struct device *dev, struct regmap *regmap,
	enum a2b24xx_type type, void (*switch_mode)(struct device *dev))
{
	struct a2b24xx *a2b24xx;
	int ret;

	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	a2b24xx = devm_kzalloc(dev, sizeof(*a2b24xx), GFP_KERNEL);
	if (a2b24xx == NULL)
		return -ENOMEM;

	ret = of_property_read_u32(dev->of_node, "a2b-id", &a2b24xx->a2b_id);
	if (ret < 0) {
		dev_err(dev, "Failed to get a2b id, return:%d\n", ret);
		return ret;
	}

	a2b24xx->dev = dev;
	a2b24xx->type = type;
	a2b24xx->regmap = regmap;
	a2b24xx->switch_mode = switch_mode;
	a2b24xx->max_master_fs = 48000;

	a2b24xx->constraints.list = a2b24xx_rates;
	a2b24xx->constraints.count = ARRAY_SIZE(a2b24xx_rates);

	//dev_set_drvdata(dev, a2b24xx);

	return devm_snd_soc_register_component(dev,
		&a2b24xx_component_driver[a2b24xx->a2b_id], &a2b24xx_dai, 1);

}

EXPORT_SYMBOL_GPL(a2b24xx_probe);

static bool a2b24xx_register_volatile(struct device *dev, unsigned int reg)
{
	return true;
}

const struct regmap_config a2b24xx_regmap_config = {
	.max_register = 255,
	.volatile_reg = a2b24xx_register_volatile,
	.cache_type = REGCACHE_NONE,
	.reg_defaults = a2b24xx_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(a2b24xx_reg_defaults),
};

EXPORT_SYMBOL_GPL(a2b24xx_regmap_config);

MODULE_DESCRIPTION("ASoC A2B24XX driver");
MODULE_AUTHOR("ADI Automotive Software Team, Bangalore");
MODULE_LICENSE("GPL");
