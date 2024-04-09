/*
 * Copyright (c) 2021-2022 Siengine Technology, Inc.
 *
 * Sienginie Camera Interface driver
 *
 * Author: Siengine Technology, Inc.
 */
#ifndef OV2311_H_
#define OV2311_H_

#include "sensor.h"

extern struct reg_value8 ov2311_1600x1300_yuv_60fps[];
extern struct reg_value8 max9295a_ov2311_config[];

int ov2311_initialize(struct i2c_client *i2c_client, struct reg_value8 *config_data);

#endif
