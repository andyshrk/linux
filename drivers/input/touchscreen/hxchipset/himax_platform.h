/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for QCT platform
 *
 *  Copyright (C) 2021 Himax Corporation.
 *
 *  This software is licensed under the terms of the GNU General Public
 *  License version 2,  as published by the Free Software Foundation,  and
 *  may be copied,  distributed,  and modified under those terms.
 *
 *  This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#ifndef HIMAX_PLATFORM_H
#define HIMAX_PLATFORM_H

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>

#define HIMAX_I2C_PLATFORM
#define HIMAX_I2C_RETRY_TIMES 3
#define BUS_RW_MAX_LEN 256

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
#define D(x...) pr_debug("[HXTP] " x)
#define I(x...) pr_info("[HXTP] " x)
#define W(x...) pr_warn("[HXTP][WARNING] " x)
#define E(x...) pr_err("[HXTP][ERROR] " x)
#define DIF(x...)                                                              \
	do {                                                                   \
		if (debug_flag)                                                \
			pr_debug("[HXTP][DEBUG] " x)                           \
	} while (0)
#else

#define D(x...)
#define I(x...)
#define W(x...)
#define E(x...)
#define DIF(x...)
#endif

#define HIMAX_common_NAME "himax_tp"
#define INPUT_DEV_NAME "himax-touchscreen"

struct himax_i2c_platform_data {
	int abs_x_min;
	int abs_x_max;
	int abs_x_fuzz;
	int abs_y_min;
	int abs_y_max;
	int abs_y_fuzz;
	int abs_pressure_min;
	int abs_pressure_max;
	int abs_pressure_fuzz;
	int abs_width_min;
	int abs_width_max;
	int screenWidth;
	int screenHeight;
	uint8_t protocol_type;
	int TSIX;
	int fail_det;
	int tp_ext_rstn;
	int PON;
	int RESX;
	int (*power)(int on);
	void (*reset)(void);
	int hx_config_size;
	int g_customer_control_tp_reset;
	const char *phys_port;
};

#endif
