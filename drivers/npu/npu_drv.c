// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2018-2021 Arm Technology (China) Co., Ltd. All rights reserved. */

/**
 * SoC: Allwinner R329 platform
 */

#include <linux/sizes.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/devfreq.h>
#include <linux/devfreq_cooling.h>
#include "aipu_io.h"
#include "soc.h"

static int se1000_aipu_devfreq_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct dev_pm_opp *opp;
	int err;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	dev_pm_opp_put(opp);

	err = dev_pm_opp_set_rate(dev, *freq);
	if (err)
		return err;

	return 0;
}

static int se1000_aipu_devfreq_get_dev_status(struct device *dev, struct devfreq_dev_status *status)
{
	struct clk *clk = dev_get_drvdata(dev);

	status->busy_time = 0;
	status->total_time = 0;
	status->current_frequency = clk_get_rate(clk);

	return 0;
}

static struct devfreq_dev_profile se1000_aipu_devfreq_profile = {
	.polling_ms = 1000, /* 1 second */
	.target = se1000_aipu_devfreq_target,
	.get_dev_status = se1000_aipu_devfreq_get_dev_status,
};

static int  se1000_aipu_devfreq_register(struct platform_device *pdev)
{
	int i;
	struct clk *clk;
	struct device_node *np;
	struct device *dev = &pdev->dev;
	struct opp_table *opp_table;
	unsigned long max_freq;
	unsigned long cur_freq;
	struct dev_pm_opp *opp;
	struct devfreq *devfreq;
	struct thermal_cooling_device *cooling;

	np = of_node_get(dev->of_node);
	clk = of_clk_get(np, 0);
	if (IS_ERR(clk)) {
		dev_warn(dev, "could not get clk\n");
		return PTR_ERR(clk);
	}

	dev_set_drvdata(dev, clk);

	cur_freq = max_freq = clk_get_rate(clk);

	opp_table = dev_pm_opp_set_clkname(dev, __clk_get_name(clk));
	if (IS_ERR(opp_table)) {
		dev_warn(dev, "could not get alloc opp_table\n");
		return PTR_ERR(opp_table);
	}

#define DEV_PM_OPP_MAX 4
	for (i = 0; i < DEV_PM_OPP_MAX; i++) {
		dev_pm_opp_add(dev, cur_freq, DEV_PM_OPP_MAX - i);
		cur_freq >>=1;
	}

	opp = devfreq_recommended_opp(dev, &max_freq, 0);
	if (IS_ERR(opp)) {
		dev_warn(dev, "could not devfreq_recommended_opp\n");
		return PTR_ERR(opp);
	}

	se1000_aipu_devfreq_profile.initial_freq = max_freq;
	dev_pm_opp_put(opp);

	devfreq = devm_devfreq_add_device(dev, &se1000_aipu_devfreq_profile,
							DEVFREQ_GOV_SIMPLE_ONDEMAND, NULL);
	if (IS_ERR(devfreq)) {
		dev_warn(dev, "Couldn't initialize npu1 devfreq\n");
		return PTR_ERR(devfreq);
	}

	cooling = of_devfreq_cooling_register(dev->of_node, devfreq);
	if (IS_ERR(cooling))
		dev_warn(dev, "Failed to register cooling device\n");

	return 0;
}

static struct aipu_soc se1000_aipu = {
	.priv = NULL,
};

static int se1000_aipu_enable_clk(struct device *dev, struct aipu_soc *soc)
{
	struct device_node *dev_node = NULL;
	void __iomem *sysctrl_mapaddr;
	struct resource *res = NULL;
	struct clk *clk_ss_aipu = NULL;
	struct clk *clk_tt_aipu = NULL;
	struct clk *clk_pll_aipu = NULL;
	unsigned long clk_pll_rate = 0, clk_tt_rate = 0,clk_ss_rate = 0 ;
	struct platform_device *p_dev = container_of(dev, struct platform_device, dev);
	u64 base = 0;
	u64 size = 0;

	WARN_ON(!dev);
	dev_node = dev->of_node;

	dev_dbg(dev, "aipu get clock by name\n");
	res = platform_get_resource(p_dev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(dev, "get aipu IO region 1 failed\n");
	}
	base = res->start;
	size = res->end - res->start + 1;

	dev_dbg(dev, "sys reg phy & size: 0x%llx, 0x%llx\n", base,size);
	sysctrl_mapaddr = devm_ioremap(dev, base, sizeof(u32));
	if (!sysctrl_mapaddr) {
		dev_err(dev, "ioremap failed");
	} else {
		dev_dbg(dev, "aipu sys-reg ioremap successful\n");
	}
	/*select npu clock  *
	 *clk_sel 0  npu_ss *
	 *clk_sel 1  npu_tt *
	 *clk_sel 2  npu_pll */

	clk_ss_aipu = of_clk_get_by_name(dev_node, "npu1_ss_clk");
	if (IS_ERR_OR_NULL(clk_ss_aipu)) {
		dev_dbg(dev, "clk_ss_aipu get failed\n");
	} else{
		iowrite32(AIPU_CLK_SEL_SS, sysctrl_mapaddr);
		if (!clk_prepare_enable(clk_ss_aipu)) {
			clk_ss_rate = clk_get_rate(clk_ss_aipu);
			dev_dbg(dev, "aipu clk_ss_aipu:%ld \n",clk_ss_rate);
		}
	}

	clk_tt_aipu = of_clk_get_by_name(dev_node, "npu1_tt_clk");
	if (IS_ERR_OR_NULL(clk_tt_aipu)) {
		dev_dbg(dev, "clk_tt_aipu get failed\n");
	} else{
		iowrite32(AIPU_CLK_SEL_TT, sysctrl_mapaddr);
		if (!clk_prepare_enable(clk_tt_aipu)) {
			clk_tt_rate = clk_get_rate(clk_tt_aipu);
		}
		dev_dbg(dev, "aipu clk_tt_aipu:%ld \n",clk_tt_rate);
	}

	clk_pll_aipu = of_clk_get_by_name(dev_node, "npu1_pll_clk");
	if (IS_ERR_OR_NULL(clk_pll_aipu)) {
		dev_dbg(dev, "clk_pll_aipu get failed\n");
	} else{
		iowrite32(AIPU_CLK_SEL_PLL, sysctrl_mapaddr);
		if (!clk_prepare_enable(clk_pll_aipu)) {
			clk_pll_rate = clk_get_rate(clk_pll_aipu);
		}
		dev_dbg(dev, "aipu clk_pll_rate:%ld \n",clk_pll_rate);
	}

	dev_info(dev, "enable SE1000 AIPU clock done\n");
	return 0;
}

static int se1000_aipu_disable_clk(struct device *dev, struct aipu_soc *soc)
{
	struct clk *clk_ss_aipu = NULL;
	struct clk *clk_tt_aipu = NULL;
	struct clk *clk_pll_aipu = NULL;
	struct device_node *dev_node = NULL;

	WARN_ON(!dev);
	dev_node = dev->of_node;

	clk_ss_aipu = of_clk_get_by_name(dev_node, "npu1_ss_clk");
	if (IS_ERR_OR_NULL(clk_ss_aipu)) {
		dev_dbg(dev, "clk_ss_aipu get failed\n");
	} else{
		clk_disable_unprepare(clk_ss_aipu);
		dev_dbg(dev, "aipu clk_ss_aipu disabled \n");
	}

	clk_tt_aipu = of_clk_get_by_name(dev_node, "npu1_tt_clk");
	if (IS_ERR_OR_NULL(clk_tt_aipu)) {
		dev_dbg(dev, "clk_tt_aipu get failed\n");
	} else{
		clk_disable_unprepare(clk_tt_aipu);
		dev_dbg(dev, "aipu clk_tt_aipu disabled \n");
	}

	clk_pll_aipu = of_clk_get_by_name(dev_node, "npu1_pll_clk");
	if (IS_ERR_OR_NULL(clk_pll_aipu)) {
		dev_dbg(dev, "clk_pll_aipu get failed\n");
	} else{
		clk_disable_unprepare(clk_pll_aipu);
		dev_dbg(dev, "aipu clk_pll_rate disabled \n");
	}

	dev_info(dev, "disable SE1000 AIPU clock done\n");
	return 0;
}

static struct aipu_soc_operations se1000_aipu_ops = {
	.start_bw_profiling = NULL,
	.stop_bw_profiling = NULL,
	.read_profiling_reg = NULL,
	.enable_clk = se1000_aipu_enable_clk,
	.disable_clk = se1000_aipu_disable_clk,
	.is_clk_enabled = NULL,
	.is_aipu_irq = NULL,
};

static int se1000_aipu_probe(struct platform_device *p_dev)
{
	se1000_aipu_devfreq_register(p_dev);
	return armchina_aipu_probe(p_dev, &se1000_aipu, &se1000_aipu_ops);
}

static int se1000_aipu_remove(struct platform_device *p_dev)
{
	return armchina_aipu_remove(p_dev);
}

static int se1000_aipu_suspend(struct platform_device *p_dev, pm_message_t state)
{
	return armchina_aipu_suspend(p_dev, state);
}

static int se1000_aipu_resume(struct platform_device *p_dev)
{
	return armchina_aipu_resume(p_dev);
}

#ifdef CONFIG_OF
static const struct of_device_id aipu_of_match[] = {
	{
		.compatible = "armchina,zhouyi-v1",
	},
	{
		.compatible = "armchina,zhouyi-v2",
	},
	{
		.compatible = "armchina,zhouyi",
	},
	{ }
};

MODULE_DEVICE_TABLE(of, aipu_of_match);
#endif

static struct platform_driver aipu_platform_driver = {
	.probe = se1000_aipu_probe,
	.remove = se1000_aipu_remove,
	.suspend = se1000_aipu_suspend,
	.resume  = se1000_aipu_resume,
	.driver = {
		.name = "armchina",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(aipu_of_match),
#endif
	},
};

module_platform_driver(aipu_platform_driver);
MODULE_LICENSE("GPL v2");
