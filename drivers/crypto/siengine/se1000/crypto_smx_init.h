// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains operations related to SMX driver initialization.
 */

#ifndef __CRYPTO_SMX_INIT_H__
#define __CRYPTO_SMX_INIT_H__

#include <linux/types.h>
#include <linux/device.h>

int smx_res_init(struct device *dev);
int smx_dev_init(void);

int smx_free_vfx(void);
void smx_free(void);

#endif
