/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __RFKILL_WLAN_H
#define __RFKILL_WLAN_H

#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/clk.h>

#define RFKILL_GPIO_NAME_SIZE	64
#define RFKILL_WLAN_NAME		"NF3219"

struct rfkill_wlan_gpio {
	int io;
	int enable;
	char name[RFKILL_GPIO_NAME_SIZE];
};

struct rfkill_wlan_data {
	char *name;
	struct rfkill_wlan_gpio reset_gpio;
};
#endif
