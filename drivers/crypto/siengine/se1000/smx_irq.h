// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains interrupt handling operations for drivers.
 */

#ifndef __SMX_IRQ_H__
#define __SMX_IRQ_H__

#include <linux/crypto.h>

#include "ips_irq.h"

struct smx_irq_args {
	struct completion wait;
	uint8_t ret_code;
	struct crypto_async_request *async_req;
};

inline struct smx_irq_args *get_smx_irq_args(int index);
void smx_irq_init(int irq);
void smx_irq_uninit(int irq);
#endif
