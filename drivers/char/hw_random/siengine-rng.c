/*
 * siengine-rng.c - Random Number Generator driver for the Siengine
 *
 * Copyright (C) 2021 Siengine
 * Mingfei Wu <mingfei.wu@siengine.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include "siengine-rng.h"

/*
 * In polling mode, do not wait infinitely for the engine to finish the work.
 */
#define SIENGINE_RNG_WAIT_RETRIES			   100
#define to_siengine_rng(p)	container_of(p, struct siengine_rng, rng)

struct siengine_rng {
	void __iomem *base;
	struct hwrng rng;
	u8 nr_rand_regs;
};

static u32 siengine_rng_readl(struct siengine_rng *rng, u32 offset)
{
	return readl_relaxed(rng->base + offset);
}

static void siengine_rng_writel(struct siengine_rng *rng, u32 val, u32 offset)
{
	writel_relaxed(val, rng->base + offset);
}

static int siengine_rng_istat_wait(struct siengine_rng *rng, u32 flag)
{
	u32 istat;
	int retry = SIENGINE_RNG_WAIT_RETRIES;

	do {
		istat = siengine_rng_readl(rng, SIENGINE_TRNG_REG_ISTAT);
		if (istat & flag)
			break;

			cpu_relax();
	} while (--retry);

		/* Clear status bit */
	siengine_rng_writel(rng, flag, SIENGINE_TRNG_REG_ISTAT);

	if (!retry)
		return -ETIMEDOUT;

	return 0;
}

/*
 * Read from output registers and put the data under 'dst' array,
 * up to dlen bytes.
 *
 * Returns number of bytes actually stored in 'dst'.
 */
static unsigned int siengine_rng_copy_random(struct siengine_rng *rng,
		   u8 *dst, unsigned int dlen)
{
	unsigned int cnt = 0;
	int i, j;
	u32 val;

	for (j = 0; j < rng->nr_rand_regs; j++) {
		val = siengine_rng_readl(rng, SIENGINE_RNG_OUT(j));

		for (i = 0; i < 4; i++) {
			dst[cnt] = val & 0xff;
			val >>= 8;
			if (++cnt >= dlen)
				return cnt;
		}
	}

	return cnt;
}

/*
 * Start the engine and poll for finish.  Then read from output registers
 * filling the 'dst' buffer up to 'dlen' bytes or up to size of generated
 * random data .
 *
 * On success: return 0 and store number of read bytes under 'read' address.
 * On error: return -ERRNO.
 */
static int siengine_rng_get_random(struct siengine_rng *rng,
		 u8 *dst, unsigned int dlen, unsigned int *read)
{
	int ret;

	siengine_rng_writel(rng, TRNG_REG_CTRL_CMD_GEN_RAND,
				  SIENGINE_TRNG_REG_CTRL);

	ret = siengine_rng_istat_wait(rng, TRNG_REG_ISTAT_RAND_RDY);
	if (ret)
		return ret;

	*read = siengine_rng_copy_random(rng, dst, dlen);

	return 0;
}

static int siengine_rng_read(struct hwrng *rng, void *buf, size_t max, bool wait)
{
	struct siengine_rng *hrng = to_siengine_rng(rng);
	unsigned int read = 0;
	unsigned int total_read = max;
	int ret;

	do {
		ret = siengine_rng_get_random(hrng, buf, max, &read);
		if (ret)
			return ret;

		max -= read;
		buf += read;
	} while (max > 0);

	return total_read;
}
static int siengine_rng_init(struct hwrng *rng)
{
	u32 cfg0;
	u32 mode;
	struct siengine_rng *hrng = to_siengine_rng(rng);
	/* detect 128/256 bits seed */
	mode = siengine_rng_readl(hrng, SIENGINE_TRNG_REG_MODE);
	cfg0 = siengine_rng_readl(hrng, SIENGINE_TRNG_REG_BUILD_CFG0);
	if (cfg0 & TRNG_REG_BUILD_CFG0_MAX_PRNG_LEN) {
		hrng->nr_rand_regs = SIENGINE_RNG_RAND_REG_8_NUM;
		mode |= TRNG_REG_MODE_R256;
	} else {
		hrng->nr_rand_regs = SIENGINE_RNG_RAND_REG_4_NUM;
		mode &= ~TRNG_REG_MODE_R256;
	}
	siengine_rng_writel(hrng, mode, SIENGINE_TRNG_REG_MODE);

	siengine_rng_writel(hrng, 0, SIENGINE_TRNG_REG_AUTO_RQSTS);
	siengine_rng_writel(hrng, 0, SIENGINE_TRNG_REG_AUTO_AGE);
	siengine_rng_writel(hrng, 0xffffffff, SIENGINE_TRNG_REG_ISTAT);

	return 0;
}

static int siengine_rng_probe(struct platform_device *pdev)
{
	struct siengine_rng *rng;
	struct resource *res;
	int ret;

	rng = devm_kzalloc(&pdev->dev, sizeof(*rng), GFP_KERNEL);
	if (!rng)
		return -ENOMEM;

	platform_set_drvdata(pdev, rng);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rng->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rng->base))
		return PTR_ERR(rng->base);

	rng->rng.name = pdev->name;
	rng->rng.init = siengine_rng_init;
	rng->rng.read = siengine_rng_read;

	ret = devm_hwrng_register(&pdev->dev, &rng->rng);
	if (ret) {
		dev_err(&pdev->dev, "failed to register hwrng\n");
		return ret;
	}

	return 0;
}

#ifdef CONFIG_PM
static int siengine_trng_suspend(struct device *dev)
{
	return 0;
}

static int siengine_trng_resume(struct device *dev)
{
	struct siengine_rng *rng = dev_get_drvdata(dev);

	siengine_rng_writel(rng, 0xffffffff, SIENGINE_TRNG_REG_ISTAT);
	return 0;
}

static const struct dev_pm_ops siengine_trng_pm_ops = {
	.suspend	= siengine_trng_suspend,
	.resume		= siengine_trng_resume,
};
#endif /* CONFIG_PM */

static const struct of_device_id siengine_hwrng_dt_match[] = {
	{
		.compatible = "siengine,se1000-rng",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, siengine_hwrng_dt_match);

static struct platform_driver siengine_rng_driver = {
	.driver = {
		.name   = "siengine-rng",
#ifdef CONFIG_PM
		.pm	= &siengine_trng_pm_ops,
#endif /* CONFIG_PM */
		.of_match_table = siengine_hwrng_dt_match,
	},
	.probe = siengine_rng_probe,
};

module_platform_driver(siengine_rng_driver);

MODULE_DESCRIPTION("siengine H/W Random Number Generator driver");
MODULE_AUTHOR("Mingfei Wu <mingfei.wu@siengine.com>");
MODULE_LICENSE("GPL");
