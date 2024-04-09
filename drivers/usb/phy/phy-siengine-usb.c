/*
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>

#include <dt-bindings/phy/phy.h>

/*  PHY Misc registers */
#define  DWC3INTR_U3PHYMISC                    0x0      //reset:0x10000206

#define  DWC3INTR_TESTFLYOVER                  0x4       //reset:0x0
#define  DWC3INTR_PROTLOVRD_0                  0x8       //protocol override0
#define  DWC3INTR_PROTLOVRD_1                  0xc       //protocol override1
#define  DWC3INTR_PROTLOVRD_2                  0x10      //protocol override2
#define  DWC3INTR_PROTLOVRD_3                  0x14      //protocol override3
#define  DWC3INTR_PROTLOVRD_4                  0x18      //protocol override4
#define  DWC3INTR_PROTLOVRD_5                  0x1C      //protocol override5
#define  DWC3INTR_PROTLOVRD_6                  0x20      //protocol override6
#define  DWC3INTR_PROTLOVRD_7                  0x24      //protocol override7
#define  DWC3INTR_PROTLOVRD_8                  0x28      //protocol override8
#define  DWC3INTR_PROTLOVRD_9                  0x2c      //protocol override9
#define  DWC3INTR_PROTLOVRD_10                 0x30      //protocol override10
#define  DWC3INTR_PROTLOVRD_11                 0x34      //protocol override11
#define  DWC3INTR_PROTLOVRD_12                 0x38      //protocol override12
#define  DWC3INTR_PROTLOVRD_13                 0x3C      //protocol override13
#define  DWC3INTR_PROTLOVRD_14                 0x40      //protocol override14
#define  DWC3INTR_USB3CTRL                     0x44      //reset: 0x20
#define  DWC3INTR_U2PHYCTL_1                   0x48      //reset: 0x166a3583
#define  DWC3INTR_U2PHYCTL_2                   0x4c      //reset: 0xF0
#define  DWC3INTR_U3NEWFUNC                    0x50      //reset: 0x0000
#define  DWC3INTR_U3OBSERVE                    0x54      //read-only: 0x0000
#define  DWC3INTR_U3MONOUT_0                   0x58      //read-only: 0x0000
#define  DWC3INTR_U3MONOUT_1                   0x5c      //read-only: 0x0000
#define  DWC3INTR_DMAWRPRI                     0x60      //reset: 0x0000
#define  DWC3INTR_DMARDPRI                     0x64      //reset: 0x0000
#define  DWC3INTR_PTMTIME_0                    0x68      //reset: 0x0000
#define  DWC3INTR_PTMTIME_1                    0x6c      //reset: 0x0000
#define  DWC3INTR_SYSBUSBLOK                   0x70      //reset: 0x0000
#define  DWC3INTR_SOCBANDWIDTH                 0x74      //reset: 0x012c0258

#define  DWC3INTR_U3PHYMISC_SRAMBYPASS         BIT(25)
#define  DWC3INTR_U3PHYMISC_REFEXTCLK          BIT(2)
#define  DWC3INTR_U3PHYMISC_RECALCNT           BIT(1)

#define  DWC3INTR_U3PHYMISC_UPCSPIPECFG        BIT(9)


/*  PHY TCA registers */
#define  DWC3TCA_TCPC                          0x14      //for config usb interface
#define  DWC3TCA_TCPC_MUX_CTRLS                0x3       //field mask
#define  DWC3TCA_TCPC_MUX_CTRL_USB31           0x01      //usb3.1: 2'01
#define  DWC3TCA_TCPC_MUX_CTRL_TCPCVALID       0x10      //Xbar


struct phy_init_tbl {
	unsigned int offset;
	unsigned int val;
	/*
	 * register part of layout
	 * if yes, then offset gives index in the reg-layout
	 */
};

#define PHY_INIT_CFG(o, v)		\
	{				\
		.offset = o,		\
		.val = v,		\
	}


static const struct phy_init_tbl siengine_usb3_firmware_tbl[] = {
	#include "dwcusb31fw.h"
};

/* struct usb3_phy_cfg - initialization config */
struct usb3_phy_cfg {
	/* number of lanes provided by phy */
	int nlanes;

	/* clock ids to be requested */
	const char * const *clk_list;
	int num_clks;
	/* resets to be requested */
	const char * const *reset_list;
	int num_resets;
	/* regulators to be requested */
	const char * const *vreg_list;
	int num_vregs;
	int has_lane_rst;
	/* Init sequence for PHY blocks - firmware download from crapb reg*/
	const struct phy_init_tbl *firmware_tbl;
	int firmware_tbl_num;
};

/**
 * struct Siengine_phy -  phy descriptor
 *
 * @phy: generic phy
 * @tx: iomapped memory space for phy int misc reg
 * @rx: iomapped memory space for phy pcs crapb reg
 * @pcs: iomapped memory space for tca reg
 * @index: lane index
 * @rst: phy's reset controller
 */
struct sieng_usb_phy {
	struct phy *phy;
	void __iomem *misc_reg;
	void __iomem *phy_reg;
	void __iomem *tca_reg;
	unsigned int index;
	struct sieng_phydev *phydev;
	struct reset_control *lane_rst;
};

/**
 * struct Sieng_phydev - structure holding phy block attributes
 *
 * @dev: device
 * @serdes: iomapped memory space for phy's serdes
 *
 * @clks: array of clocks required by phy
 * @resets: array of resets required by phy
 * @vregs: regulator supplies bulk data
 *
 * @cfg: phy specific configuration
 * @phys: array of per-lane phy descriptors
 * @phy_mutex: mutex lock for PHY common block initialization
 * @init_count: phy common block initialization count
 */
struct sieng_phydev {
	struct device *dev;
	struct clk **clks;
	struct reset_control **resets;
	struct regulator_bulk_data *vregs;

	const struct usb3_phy_cfg *cfg;
	struct sieng_usb_phy **phys;

	struct mutex phy_mutex;
	int init_count;
};


static u32 miscreg_read(void __iomem *base, u32 offset)
{

	unsigned long addr = (unsigned long )base + offset;

	/* ensure that above write is through */
	return readl((void __iomem *)addr);
}

static inline void miscreg_setbits(void __iomem *base, u32 offset, u32 val)
{
	u32 reg;
	unsigned long addr = (unsigned long ) base + offset;
	reg = readl((void __iomem *)addr);
	reg |= val;
	writel(reg, (void __iomem *)addr);

	/* ensure that above write is through */
	readl(base + offset);
}


/* list of resets */
static const char * const usb3phy_reset_l[] = {

};

static const struct usb3_phy_cfg siengine_usb3phy_cfg = {

	.firmware_tbl		= siengine_usb3_firmware_tbl,
	.firmware_tbl_num		= ARRAY_SIZE(siengine_usb3_firmware_tbl),
	.reset_list		= usb3phy_reset_l,
	.num_clks		= 0,
	.num_resets		= 0,
	.num_vregs		= 0,
	.has_lane_rst		= 0,
	.nlanes		= 1,
};


static void siengine_usbphy_configure(void __iomem *base,
				const struct phy_init_tbl tbl[],
				int num)
{
	int i;
	const struct phy_init_tbl *t = tbl;

	if (!t)
		return;

	printk("firware downloading.. ");
	for (i = 0; i < num; i++) {
		writew(t->val, base + ((t->offset)<<1));
		t++;
	}
	printk("finished. ");


}

static int siengine_usbphy_poweron(struct phy *phy)
{
    return 0;
}

static int siengine_usbphy_poweroff(struct phy *phy)
{
    return 0;
}


/* PHY Initialization */
static int siengine_usbphy_init(struct phy *phy)
{
	struct sieng_usb_phy *siengphy = phy_get_drvdata(phy);
	struct sieng_phydev *phydev = siengphy->phydev;
	const struct usb3_phy_cfg *cfg = phydev->cfg;
	void __iomem *miscreg = siengphy->misc_reg;
	void __iomem *phyreg = siengphy->phy_reg;
	void __iomem *tcareg = siengphy->tca_reg;
	unsigned int val;
	int ret, i;

	dev_vdbg(phydev->dev, "Initializing Siengine Usb3 phy\n");


	for (i = 0; i < cfg->num_resets; i++) {
		ret = reset_control_deassert(phydev->resets[i]);
		if (ret) {
			dev_err(phydev->dev, "%s reset deassert failed\n",
				phydev->cfg->reset_list[i]);
			goto err_rst;
		}
	}

	for (i = 0; i < cfg->num_clks; i++) {
		ret = clk_prepare_enable(phydev->clks[i]);
		if (ret) {
			dev_err(phydev->dev, "failed to enable %s clk, err=%d\n",
				phydev->cfg->clk_list[i], ret);
			goto err_clk;
		}
	}

	if (cfg->has_lane_rst) {
		ret = reset_control_deassert(siengphy->lane_rst);
		if (ret) {
			dev_err(phydev->dev, "lane%d reset deassert failed\n",
				siengphy->index);
			goto err_lane_rst;
		}
	}

	miscreg_setbits(miscreg, DWC3INTR_U3PHYMISC, 0x0000002);

	/* Tx, Rx, and PCS configurations */
	//lanes special config
	/* firmware download */
	siengine_usbphy_configure(phyreg, cfg->firmware_tbl,
			       cfg->firmware_tbl_num);


	miscreg_setbits(miscreg, DWC3INTR_U2PHYCTL_2,  0x1f0);
	//Tx deemph config
	miscreg_setbits(miscreg, DWC3INTR_PROTLOVRD_2,  0x1188b);


	while((miscreg_read(miscreg, DWC3INTR_U3PHYMISC)&(1<<27))==0)
	{
		dev_err(phydev->dev, "phy sram init not done");
	}

	miscreg_setbits(miscreg, DWC3INTR_U3PHYMISC, 0x4000002);

	miscreg_setbits(miscreg, DWC3INTR_PROTLOVRD_13, 0x4);

	//tca register initial:
	val = readl(tcareg + DWC3TCA_TCPC);
	val &= ~DWC3TCA_TCPC_MUX_CTRLS;
	val |= (DWC3TCA_TCPC_MUX_CTRL_USB31|DWC3TCA_TCPC_MUX_CTRL_TCPCVALID);
	writel(val, tcareg + DWC3TCA_TCPC);


	if (ret) {
		dev_err(phydev->dev, "phy initialization timed-out\n");
	}

err_clk:
err_rst:
err_lane_rst:
	return ret;
}

static int siengine_usbphy_exit(struct phy *phy)
{
#if 0
	struct sieng_usb_phy *siengphy = phy_get_drvdata(phy);
	struct sieng_phydev *phydev = siengphy->phydev;
	const struct usb3_phy_cfg *cfg = phydev->cfg;


	clk_disable_unprepare(siengphy->pipe_clk);

	if (cfg->has_lane_rst)
		reset_control_assert(siengphy->lane_rst);

	siengine_usbphy_com_exit(phydev);

	while (--i >= 0)
		clk_disable_unprepare(phydev->clks[i]);
#endif
	return 0;
}


static int siengine_usbphy_reset_init(struct device *dev)
{
	struct  sieng_phydev *phydev = dev_get_drvdata(dev);
	int i;

	phydev->resets = devm_kcalloc(dev, phydev->cfg->num_resets,
		sizeof(*phydev->resets), GFP_KERNEL);
	if (!phydev->resets)
		return -ENOMEM;

	for (i = 0; i < phydev->cfg->num_resets; i++) {
		struct reset_control *rst;
		const char *name = phydev->cfg->reset_list[i];

		rst = devm_reset_control_get(dev, name);
		if (IS_ERR(rst)) {
			dev_err(dev, "failed to get %s reset\n", name);
			return PTR_ERR(rst);
		}
		phydev->resets[i] = rst;
	}
	return 0;
}

/*
 * Register a fixed rate pipe clock.
 *
 */
static int phy_pipe_clk_register(struct sieng_phydev *phydev, struct device_node *np)
{
	return 0;
}

static const struct phy_ops siengine_usbphy_gen_ops = {
	.init  = siengine_usbphy_init,
	.exit  = siengine_usbphy_exit,
	.power_on  = siengine_usbphy_poweron,
	.power_off  = siengine_usbphy_poweroff,
	.owner  = THIS_MODULE,
};

static
int siengine_usbphy_create(struct device *dev, struct device_node *np, int id)
{
	struct sieng_phydev *phydev = dev_get_drvdata(dev);
	struct phy *generic_phy;
	struct sieng_usb_phy *siengphy;
	char prop_name[50];
	int ret;

	siengphy = devm_kzalloc(dev, sizeof(*siengphy), GFP_KERNEL);
	if (!siengphy)
		return -ENOMEM;


	/*
	 * Get memory resources for each phy lane:
	 * Resources are indexed as: tx -> 0; rx -> 1; pcs -> 2.
	 */
	siengphy->tca_reg = of_iomap(np, 0);
	if (!siengphy->tca_reg)
		return -ENOMEM;

	siengphy->misc_reg = of_iomap(np, 1);
	if (!siengphy->misc_reg)
		return -ENOMEM;

	siengphy->phy_reg = of_iomap(np, 2);
	if (!siengphy->phy_reg)
		return -ENOMEM;

	/* Get lane reset, if any */
	if (phydev->cfg->has_lane_rst) {
		snprintf(prop_name, sizeof(prop_name), "lane%d", id);
		siengphy->lane_rst = of_reset_control_get(np, prop_name);
		if (IS_ERR(siengphy->lane_rst)) {
			dev_err(dev, "failed to get lane%d reset\n", id);
			return PTR_ERR(siengphy->lane_rst);
		}
	}

	generic_phy = devm_phy_create(dev, np, &siengine_usbphy_gen_ops);
	if (IS_ERR(generic_phy)) {
		ret = PTR_ERR(generic_phy);
		dev_err(dev, "failed to create qphy %d\n", ret);
		return ret;
	}

	siengphy->phy = generic_phy;
	siengphy->index = id;
	siengphy->phydev = phydev;
	phydev->phys[id] = siengphy;
	phy_set_drvdata(generic_phy, siengphy);

	return 0;
}

static const struct of_device_id Siengine_usbphy_of_match_table[] = {
	{
		.compatible = "siengine,usb3-phy",
		.data = &siengine_usb3phy_cfg,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, Siengine_usbphy_of_match_table);

static int Siengine_usbphy_probe(struct platform_device *pdev)
{
	struct sieng_phydev *phydev;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct device_node *child;
	struct phy_provider *phy_provider;
	int num, id;
	int ret;

	dev_info(dev, "begin  Siengine_usbphy_probe..");
	phydev = devm_kzalloc(dev, sizeof(*phydev), GFP_KERNEL);
	if (!phydev)
		return -ENOMEM;

	phydev->dev = dev;
	dev_set_drvdata(dev, phydev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	mutex_init(&phydev->phy_mutex);

	/* Get the specific init parameters of siengine phy */
	phydev->cfg = of_device_get_match_data(dev);


	ret = siengine_usbphy_reset_init(dev);
	if (ret)
		return ret;


	num = of_get_available_child_count(dev->of_node);
	/* do we have a rogue child node ? */
	if (num > phydev->cfg->nlanes)
		return -EINVAL;

	phydev->phys = devm_kcalloc(dev, num, sizeof(*phydev->phys), GFP_KERNEL);
	if (!phydev->phys)
		return -ENOMEM;

	id = 0;
	for_each_available_child_of_node(dev->of_node, child) {
		/* Create per-lane phy */
		ret = siengine_usbphy_create(dev, child, id);
		if (ret) {
			dev_err(dev, "failed to create lane%d phy, %d\n",
				id, ret);
			return ret;
		}

		/*
		 * Register the pipe clock provided by phy.
		 * See function description to see details of this pipe clock.
		 */
		ret = phy_pipe_clk_register(phydev, child);
		if (ret) {
			dev_err(phydev->dev,
				"failed to register pipe clock source\n");
			return ret;
		}
		id++;
	}

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (!IS_ERR(phy_provider))
		dev_info(dev, "Registered Siegnine USB phy success\n");

	return PTR_ERR_OR_ZERO(phy_provider);
}

static struct platform_driver siengine_usbphy_driver = {
	.probe		= Siengine_usbphy_probe,
	.driver = {
		.name	= "siengine-usb3-phy",
		.of_match_table = Siengine_usbphy_of_match_table,
	},
};

module_platform_driver(siengine_usbphy_driver);
MODULE_AUTHOR("Siengine");
MODULE_DESCRIPTION("Siengine Usb3 PHY driver");
MODULE_LICENSE("GPL v2");
