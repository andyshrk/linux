// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018-2019 Synopsys, Inc. and/or its affiliates.
 * Copyright (c) 2019-2021 Siengine, Inc. and/or its affiliates.
 *
 * Synopsys DesignWare MIPI D-PHY controller driver.
 * Platform driver
 *
 * Author: Luis Oliveira <luis.oliveira@synopsys.com>
 * Author: Mingrui Zhou <Mingrui.zhou@siengine.com>
 */

#include <media/dwc/dw-dphy-data.h>
#include <media/dwc/dw-csi-data.h>

#include "dw-dphy-rx.h"
#include "siengine_cif_misc_regs.h"

/* Global variable for compatibility mode, this could be override later */
static int phy_type = 1;

static struct siengine_cif_misc *cif_misc = NULL;

module_param(phy_type, int, 0664);
MODULE_PARM_DESC(phy_type, "Disable compatibility mode for D-PHY G128");

static struct phy_ops dw_dphy_ops = {
	.init = dw_dphy_init,
	.reset = dw_dphy_reset,
	.power_on = dw_dphy_power_on,
	.power_off = dw_dphy_power_off,
	.owner = THIS_MODULE,
};

static struct phy_provider *phy_provider;

#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
static u8 get_config_8l(struct device *dev, struct dw_dphy_rx *dphy)
{
	if (IS_ENABLED(CONFIG_OF) && dev->of_node) {
		dphy->config_8l = of_get_gpio(dev->of_node, 0);
		if (!gpio_is_valid(dphy->config_8l)) {
			dev_warn(dev, "failed to parse 8l config, default is 0\n");
			dphy->config_8l = 0;
		}
	} else {
		struct dw_phy_pdata *pdata = dev->platform_data;

		dphy->config_8l = pdata->config_8l;
	}
	return dphy->config_8l;
}
#endif

static int get_resources(struct device *dev, struct dw_dphy_rx *dphy)
{
	int ret = 0;

	if (IS_ENABLED(CONFIG_OF) && dev->of_node) {
		if (of_property_read_u32(dev->of_node, "snps,dphy-frequency",
					 &dphy->dphy_freq)) {
			dev_err(dev, "failed to find dphy frequency\n");
			ret = -EINVAL;
		}
		if (of_property_read_u32(dev->of_node, "bus-width",
					 &dphy->dphy_te_len)) {
			dev_err(dev, "failed to find dphy te length\n");
			ret = -EINVAL;
		}
		if (of_property_read_u32(dev->of_node, "snps,phy_type",
					 &dphy->phy_type)) {
			dev_err(dev, "failed to find dphy type\n");
			ret = -EINVAL;
		}

		dphy->dphy_gen = GEN3;
	} else {
		struct dw_phy_pdata *pdata = dev->platform_data;

		dphy->dphy_freq = pdata->dphy_frequency;
		dphy->dphy_te_len = pdata->dphy_te_len;
		dphy->dphy_gen = pdata->dphy_gen;
	}
	dev_set_drvdata(dev, dphy);

	return ret;
}

static int phy_register(struct device *dev)
{
	int ret = 0;

	if (IS_ENABLED(CONFIG_OF) && dev->of_node) {
		phy_provider = devm_of_phy_provider_register(dev, dw_dphy_xlate);
		if (IS_ERR(phy_provider)) {
			dev_err(dev, "error getting phy provider\n");
			ret = PTR_ERR(phy_provider);
		}
	} else {
		struct dw_phy_pdata *pdata = dev->platform_data;
		struct dw_dphy_rx *dphy = dev_get_drvdata(dev);

		ret = phy_create_lookup(dphy->phy,
					phys[pdata->id].name,
					csis[pdata->id].name);
		if (ret)
			dev_err(dev, "Failed to create dphy lookup\n");
		else
			dev_warn(dev, "Created dphy lookup [%s] --> [%s]\n",
				 phys[pdata->id].name, csis[pdata->id].name);
	}
	return ret;
}

static void phy_unregister(struct device *dev)
{
	if (!dev->of_node) {
		struct dw_phy_pdata *pdata = dev->platform_data;
		struct dw_dphy_rx *dphy = dev_get_drvdata(dev);

		phy_remove_lookup(dphy->phy, phys[pdata->id].name, csis[pdata->id].name);
		dev_warn(dev, "Removed dphy lookup [%s] --> [%s]\n",
			 phys[pdata->id].name, csis[pdata->id].name);
	}
}

static int siengine_cif_misc_init(struct platform_device *pdev)
{
	int ret = 0;
	u32 cfgclk;
	void __iomem* addr;
	struct device_node *misc_node;
	u32 cfgclk_freqrange;

	if (cif_misc)
		return 0;

	misc_node = of_parse_phandle(pdev->dev.of_node, "cif-misc", 0);
	if (!misc_node) {
		pr_err("can't find phandle cif-misc\n");
		return -EINVAL;
	}

	cif_misc = kzalloc(sizeof(*cif_misc), GFP_KERNEL);
	if (!cif_misc) {
		pr_err("cif_misc kzalloc failed\n");

		ret = -ENOMEM;
		goto err_kzalloc;
	}

	addr = of_iomap(misc_node, 0);
	of_node_put(misc_node);
	if (!addr) {
		pr_err("misc_node could not map cif-misc registers\n");

		ret = -ENOMEM;
		goto err_of_iomap;
	}

	ret = of_property_read_u32(misc_node, "siengine,cfg-clk", &cfgclk);
	if (ret) {
		pr_err("misc_node could not found cfg-clk\n");

		ret = -EINVAL;
		goto err_cfgclk;
	}

	spin_lock_init(&cif_misc->lock);

	cif_misc->base_address = addr;
	cif_misc->of_node = misc_node;
	cif_misc->cfg_clk = cfgclk;
	cfgclk_freqrange = (cif_misc->cfg_clk / 1000000 - 17) * 4;

	iowrite32(cfgclk_freqrange, cif_misc->base_address + CSI_DPHY_CFGCLKREQRANGE);

	return 0;

err_cfgclk:
	iounmap(addr);

err_of_iomap:
	kfree(cif_misc);

err_kzalloc:
	return ret;
}

static int dphy_clk_enable(struct dw_dphy_rx *dphy, bool enable)
{
	unsigned long flags;
	if (enable) {
		spin_lock_irqsave(&dphy->clk_lock, flags);
		if (clk_prepare_enable(dphy->clk_cfg)) {
			dev_warn(dphy->dev, "try to enable dphy clk failed!\n");
			spin_unlock_irqrestore(&dphy->clk_lock, flags);
			return -EINVAL;
		}
		spin_unlock_irqrestore(&dphy->clk_lock, flags);
		dev_warn(dphy->dev, "call clk enable");
	} else {
		spin_lock_irqsave(&dphy->clk_lock, flags);
		clk_disable_unprepare(dphy->clk_cfg);
		spin_unlock_irqrestore(&dphy->clk_lock, flags);
		dev_warn(dphy->dev, "call clk disable");
	}
	return 0;
}

static int dw_dphy_rx_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct dw_dphy_rx *dphy;
	struct resource *res;

	ret = siengine_cif_misc_init(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Can not init cif-misc\n");
		return ret;
	}

	dphy = devm_kzalloc(&pdev->dev, sizeof(*dphy), GFP_KERNEL);
	if (!dphy)
		return -ENOMEM;

	dphy->dev = dev;
	spin_lock_init(&dphy->clk_lock);

	dphy->clk_cfg = of_clk_get_by_name(pdev->dev.of_node, CFG_CLK_NAME);
	if (IS_ERR(dphy->clk_cfg))
		return PTR_ERR(dphy->clk_cfg);

	ret = dphy_clk_enable(dphy, true);
	if (ret != 0) {
		dev_err(&pdev->dev, "enable dphy clk failed.\n");
		return -EINVAL;
	}
	dev_warn(&pdev->dev, "enable dphy clk success.\n");
#if 0
	ret = clk_set_rate(dphy->clk_cfg, cif_misc->cfg_clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to set cisp_dphy_cfg_clk rate %d\n", cif_misc->cfg_clk);
		return ret;
	}
	dev_info(&pdev->dev, "Set cisp_dphy_cfg_clk rate %d\n", cif_misc->cfg_clk);
#endif
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR(res)) {
		dev_err(&pdev->dev, "error requesting IORESOURCE_MEM 0\n");
		return PTR_ERR(res);
	}

	dphy->base_address = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR(dphy->base_address)) {
		dev_err(&pdev->dev, "error requesting base address\n");
		return PTR_ERR(dphy->base_address);
	}

#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);

	dphy->dphy1_if_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dphy->dphy1_if_addr)) {
		dev_err(&pdev->dev, "error requesting dphy 1 if regbank\n");
		return PTR_ERR(dphy->dphy1_if_addr);
	}

	dphy->max_lanes = dw_dphy_if_read_msk(dphy, DPHYID, DPHY_ID_LANE_SUPPORT, 4);

	dphy->dphy_gen = dw_dphy_if_read_msk(dphy, DPHYID, DPHY_ID_GEN, 4);

	dev_info(&pdev->dev, "DPHY GEN %s with maximum %s lanes\n",
		 dphy->dphy_gen == GEN3 ? "3" : "2",
		 dphy->max_lanes == CTRL_8_LANES ? "8" : "4");

	if (dphy->max_lanes == CTRL_8_LANES) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
		dphy->dphy2_if_addr =
			devm_ioremap(&pdev->dev, res->start, resource_size(res));

		if (IS_ERR(dphy->dphy2_if_addr)) {
			dev_err(&pdev->dev, "error requesting dphy 2 if regbank\n");
			return PTR_ERR(dphy->dphy2_if_addr);
		}
		dphy->config_8l = get_config_8l(&pdev->dev, dphy);
	}
#endif
	if (get_resources(dev, dphy)) {
		dev_err(dev, "failed to parse PHY resources\n");
		return -EINVAL;
	}

	dphy->phy = devm_phy_create(dev, NULL, &dw_dphy_ops);
	if (IS_ERR(dphy->phy)) {
		dev_err(dev, "failed to create PHY\n");
		return PTR_ERR(dphy->phy);
	}

	ret = of_property_read_u32(dev->of_node, "idx", &dphy->idx);
	if (ret) {
		pr_err("idx is missing at node\n");
		return ret;
	}

	platform_set_drvdata(pdev, dphy);
	phy_set_drvdata(dphy->phy, dphy);

	if (phy_register(dev)) {
		dev_err(dev, "failed to register PHY\n");
		return -EINVAL;
	}

	dphy->cif_misc = cif_misc;

	dphy->lp_time = 1000; /* 1000 ns */

	dphy->lanes_config = dw_dphy_setup_config(dphy);

	dev_info(&dphy->phy->dev, "Probing dphy finished\n");

#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
	dw_dphy_create_capabilities_sysfs(pdev);
#endif

	return 0;
}

static int dw_dphy_rx_remove(struct platform_device *pdev)
{
	struct dw_dphy_rx *dphy = platform_get_drvdata(pdev);

	dev_info(&dphy->phy->dev, "phy removed\n");
	phy_unregister(&pdev->dev);

	/* disable clk */

	dphy_clk_enable(dphy, false);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int dphy_runtime_suspend(struct device *dev)
{
	struct dw_dphy_rx *dphy = dev_get_drvdata(dev);

	dphy_clk_enable(dphy, false);
	return 0;//pm_runtime_force_suspend(dev);
}

static int dphy_runtime_resume(struct device *dev)
{
	struct dw_dphy_rx *dphy = dev_get_drvdata(dev);

	//pm_runtime_force_suspend(dev);
	dphy_clk_enable(dphy, true);
	return 0;
}

#else
static int dphy_runtime_suspend(struct device *dev)
{
	return 0;
}

static int dphy_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops se_dphy_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dphy_runtime_suspend,
					dphy_runtime_resume)
};

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id dw_dphy_rx_of_match[] = {
	{ .compatible = "snps,dw-dphy-rx" },
	{},
};

MODULE_DEVICE_TABLE(of, dw_dphy_rx_of_match);
#endif

static struct platform_driver dw_dphy_rx_driver = {
	.probe = dw_dphy_rx_probe,
	.remove = dw_dphy_rx_remove,
	.driver = {
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = of_match_ptr(dw_dphy_rx_of_match),
#endif
		.name = "dw-dphy",
		.owner = THIS_MODULE,
		.pm = &se_dphy_pm_ops,
	}
};

static int __init dw_dphy_rx_driver_init(void)
{
	return platform_driver_register(&dw_dphy_rx_driver);
}
subsys_initcall(dw_dphy_rx_driver_init);

MODULE_DESCRIPTION("Synopsys DesignWare MIPI DPHY Rx driver");
MODULE_AUTHOR("Luis Oliveira <lolivei@synopsys.com>");
MODULE_AUTHOR("Mingrui Zhou <Mingrui.Zhou@siengine.com>");
MODULE_LICENSE("GPL v2");
