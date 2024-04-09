// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains the driver registration and start related operations,
 * which is the main entry file of the driver.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/crypto.h>
#include <crypto/skcipher.h>
#include <crypto/internal/skcipher.h>
#include <crypto/internal/hash.h>
#include <crypto/gcm.h>

#include "crypto_smx_init.h"
#include "crypto_smx_internal.h"
#include "crypto_smx_sm3.h"
#include "crypto_smx_sm4.h"
#include "crypto_res_mgr.h"
#include "smx_irq.h"

static int smx_probe(struct platform_device *pdev)
{
	int irq;
	int err;

	/* Set device DMA access capability */
	pdev->dev.bus_dma_limit = DMA_BIT_MASK(64);
	*(pdev->dev.dma_mask) = DMA_BIT_MASK(64);

	irq = platform_get_irq(pdev, 0);
	smx_irq_init(irq);

	err = smx_res_init(&pdev->dev);
	if (err < 0)
		return err;

	err = smx_dev_init();
	if (err < 0) {
		dev_err(&pdev->dev, "smx_dev_init failed.\n");
		return -ENODEV;
	}

	register_sm3();
	register_sm4();

	return 0;
}

static int smx_remove(struct platform_device *pdev)
{
	int irq;

	unregister_sm3();
	unregister_sm4();

	irq = platform_get_irq(pdev, 0);
	smx_irq_uninit(irq);

	smx_free();
	return 0;
}

static int smx_suspend(struct platform_device *pdev, pm_message_t state)
{
	return smx_free_vfx();
}

static int smx_resume(struct platform_device *pdev)
{
	int err;

	err = smx_dev_init();
	if (err < 0) {
		dev_err(&pdev->dev, "smx_dev_init failed.\n");
		return -ENODEV;
	}
	return 0;
}


static const struct of_device_id se1000_smx_match[] = {
	{.compatible = "siengine,se1000-smx",},
	{},
};
MODULE_DEVICE_TABLE(of, se1000_smx_match);

static struct platform_driver se1000_smx_drv = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "se1000_smx",
		.of_match_table = se1000_smx_match,
	},

	.probe = smx_probe,
	.remove = smx_remove,
	.suspend = smx_suspend,
	.resume = smx_resume,
};

module_platform_driver(se1000_smx_drv);

MODULE_LICENSE("GPL");
