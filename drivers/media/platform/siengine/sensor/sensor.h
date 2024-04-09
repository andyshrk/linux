/*
 * Copyright (c) 2021-2022 Siengine Technology, Inc.
 *
 * Sienginie Camera Interface driver
 *
 * Author: Siengine Technology, Inc.
 */
#ifndef SENSOR_H_
#define SENSOR_H_

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>

#define WRITE_VERIFY    0
#define TABLE_END	0xFFFF

struct reg_value16 {
	unsigned short reg_addr;
	unsigned short val;
	unsigned int delay_ms;
};

struct reg_value8 {
	unsigned short reg_addr;
	unsigned char val;
	unsigned int delay_ms;
};

int sensor_read_reg8(struct i2c_client *i2c_client, unsigned short addr,
		unsigned short reg, unsigned char *val);
int sensor_write_reg8(struct i2c_client *i2c_client, unsigned short addr,
		unsigned short reg, unsigned char val);
int sensor_read_reg16(struct i2c_client *i2c_client, unsigned short addr,
		unsigned short reg, unsigned short *val);
int sensor_write_reg16(struct i2c_client *i2c_client, unsigned short addr,
		unsigned short reg, unsigned short val);

#endif
