/*
 * Copyright (c) 2021-2022 Siengine Technology, Inc.
 *
 * Sienginie Camera Interface driver
 *
 * Author: Siengine Technology, Inc.
 */
#ifndef AR0231_RAW_H_
#define AR0231_RAW_H_

#include "sensor.h"

extern struct reg_value16 ar0231_mode_1920X1080_12bit_linear_30fps[];
extern struct reg_value8 max9296_9295_linkA_config[];
extern struct reg_value8 max96722_9295_linkA_config[];
extern struct reg_value8 max96722_9295_linkB_config[];
extern struct reg_value8 max96722_9295_linkC_config[];
extern struct reg_value8 max96722_9295_linkD_config[];

int ar0231_initialize(struct i2c_client *i2c_client, struct reg_value16 *config_data);
int max9295_initialize(struct i2c_client *i2c_client, struct reg_value8 *config_data);
int ar0231_write_reg(struct i2c_client *i2c_client, unsigned short reg, unsigned short val);
int ar0231_read_reg(struct i2c_client *i2c_client, unsigned short reg, unsigned short *val);
int max9295_write_reg(struct i2c_client *i2c_client,  unsigned short reg, unsigned char val);
int max9295_read_dev_id(struct i2c_client *i2c_client);

#endif
