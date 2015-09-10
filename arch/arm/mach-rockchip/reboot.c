/*
 * Copyright (c) 2015, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include "loader.h"

#define RK3188_PMU_SYS_REG0             0x40
#define RK3288_PMU_SYS_REG0             0x94

struct regmap *regmap;
int flag_reg;

static int rockchip_get_pmu_regmap(void)
{
	struct device_node *node;

	node = of_find_node_by_path("/cpus");
	if (!node)
		return -ENODEV;

	regmap = syscon_regmap_lookup_by_phandle(node, "rockchip,pmu");
	of_node_put(node);
	if (!IS_ERR(regmap))
		return 0;

	regmap = syscon_regmap_lookup_by_compatible("rockchip,rk3066-pmu");
	if (!IS_ERR(regmap))
		return 0;

	return -ENODEV;
}

static int rockchip_get_reboot_flag_regmap(void)
{
	int ret = rockchip_get_pmu_regmap();

	if (ret < 0)
		return ret;

	if (of_machine_is_compatible("rockchip,rk3288")) {
		flag_reg = RK3288_PMU_SYS_REG0;
		return 0;
	} else if (of_machine_is_compatible("rockchip,rk3066a") ||
		   of_machine_is_compatible("rockchip,rk3066b") ||
		   of_machine_is_compatible("rockchip,rk3188")) {
		flag_reg = RK3188_PMU_SYS_REG0;
		return 0;
	}

	return -ENODEV;
}

static void rockchip_get_reboot_flag(const char *cmd, u32 *flag)
{
	*flag = SYS_LOADER_REBOOT_FLAG + BOOT_NORMAL;

	if (cmd) {
		if (!strcmp(cmd, "loader") || !strcmp(cmd, "bootloader"))
			*flag = SYS_LOADER_REBOOT_FLAG + BOOT_LOADER;
		else if (!strcmp(cmd, "recovery"))
			*flag = SYS_LOADER_REBOOT_FLAG + BOOT_RECOVER;
		else if (!strcmp(cmd, "charge"))
			*flag = SYS_LOADER_REBOOT_FLAG + BOOT_CHARGING;
	}
}

static int rockchip_reboot_notify(struct notifier_block *this,
				  unsigned long mode, void *cmd)
{
	u32 flag;

	rockchip_get_reboot_flag(cmd, &flag);
	regmap_write(regmap, flag_reg, flag);

	return NOTIFY_DONE;
}

static struct notifier_block rockchip_reboot_handler = {
	.notifier_call = rockchip_reboot_notify,
};

static int __init rockchip_reboot_init(void)
{
	int ret = 0;

	if (!rockchip_get_reboot_flag_regmap()) {
		ret = register_reboot_notifier(&rockchip_reboot_handler);
		if (ret)
			pr_err("%s: cannot register reboot handler, %d\n",
			       __func__, ret);
	}

	return ret;
}

module_init(rockchip_reboot_init);
MODULE_AUTHOR("Andy Yan <andy.yan@rock-chips.com");
MODULE_DESCRIPTION("Rockchip platform reboot notifier driver");
MODULE_LICENSE("GPL");
