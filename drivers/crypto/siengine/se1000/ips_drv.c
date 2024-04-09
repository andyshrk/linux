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
#include <crypto/aes.h>
#include <crypto/gcm.h>

#include "crypto_ips_init.h"
#include "crypto_ips_internal.h"
#include "crypto_ips_aes.h"
#include "crypto_ips_sha.h"
#include "crypto_res_mgr.h"
#include "ips_irq.h"

void ipsec_test(void)
{
	pr_info("==== ipsec outbound test ====");
	ips_ipsec_out_cmd_setup();

	pr_info("==== ipsec inbound test ====");
	ips_ipsec_in_cmd_setup();
}

static int ips_probe(struct platform_device *pdev)
{
	int irq;
	int err;

	/* Set device DMA access capability, IPS can access whole address space */
	pdev->dev.bus_dma_limit = DMA_BIT_MASK(64);
	*(pdev->dev.dma_mask) = DMA_BIT_MASK(64);

	irq = platform_get_irq(pdev, 0);
	ips_set_irq_num(irq);

	err = ips_res_init(&(pdev->dev));
	if (err < 0)
		return err;

	err = ips_dev_init();
	if (err < 0) {
		dev_err(&pdev->dev, "ips_dev_init failed.\n");
		return -ENODEV;
	}

	register_aes();
	register_sha();
	ipsec_test();
	return err;
}

static int ips_remove(struct platform_device *pdev)
{
	unregister_aes();
	unregister_sha();

	(void)ips_free();
	return 0;
}

static int ips_suspend(struct platform_device *pdev, pm_message_t state)
{
	return ips_free_vfx();
}

static int ips_resume(struct platform_device *pdev)
{
	int err;

	err = ips_dev_init();
	if (err < 0) {
		dev_err(&pdev->dev, "ips_dev_init failed.\n");
		return -ENODEV;
	}
	return 0;
}

static const struct of_device_id se1000_ips_match[] = {
	{.compatible = "siengine,se1000-ips",},
	{},
};
MODULE_DEVICE_TABLE(of, se1000_ips_match);

static struct platform_driver se1000_ips_drv = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "se1000_ips",
		.of_match_table = se1000_ips_match,
	},

	.probe = ips_probe,
	.remove = ips_remove,
	.suspend = ips_suspend,
	.resume = ips_resume,
};

module_platform_driver(se1000_ips_drv);

MODULE_LICENSE("GPL");
