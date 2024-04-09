// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains operations related to IPS driver initialization.
 */

#ifndef __CRYPTO_IPS_INIT_H__
#define __CRYPTO_IPS_INIT_H__

#include <linux/types.h>
#include <linux/device.h>

int ips_res_init(struct device *dev);
int ips_dev_init(void);

int ips_free_vfx(void);
int ips_free(void);
#endif
