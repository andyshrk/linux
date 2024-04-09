// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains interrupt handling operations for drivers.
 */

#ifndef __IPS_IRQ_H__
#define __IPS_IRQ_H__

#include <linux/types.h>
#include <linux/completion.h>

typedef struct {
	int STS_ERR_EN_0: 1;
	int STS_ERR_EN_1: 1;
	int CMD_EN_0: 1;
	int CMD_EN_1: 1;
	int STAT_EN_0: 1;
	int STAT_EN_1: 1;
	int IOC_EN_0: 1;
	int IOC_EN_1: 1;
	int STAT_WD_EN_0: 1;
	int STAT_WD_EN_1: 1;
	int RD_CMD_ERR_IRQ_0: 1;
	int RD_CMD_ERR_IRQ_1: 1;
	int WR_STS_ERR_EN_0: 1;
	int WR_STS_ERR_EN_1: 1;
	int reserve: 17;
	int GLBL_EN: 1;
} CRYPTO_IRQ_EN; /* RW */

typedef struct {
	int STS_ERR_RAW_0: 1;
	int STS_ERR_RAW_1: 1;
	int CMD_RAW_0: 1;
	int CMD_RAW_1: 1;
	int STAT_RAW_0: 1;
	int STAT_RAW_1: 1;
	int IOC_RAW_0: 1;
	int IOC_RAW_1: 1;
	int STAT_WD_RAW_0: 1;
	int STAT_WD_RAW_1: 1;
	int RD_CMD_ERR_RAW_0: 1;
	int RD_CMD_ERR_RAW_1: 1;
	int WR_STS_ERR_RAW_0: 1;
	int WR_STS_ERR_RAW_1: 1;
	int reserve: 18;
} CRYPTO_IRQ_RAW; /* RO */

typedef struct {
	int STS_ERR_STAT_0: 1;
	int STS_ERR_STAT_1: 1;
	int CMD_STAT_0: 1;
	int CMD_STAT_1: 1;
	int STAT_STAT_0: 1;
	int STAT_STAT_1: 1;
	int IOC_STAT_0: 1;
	int IOC_STAT_1: 1;
	int STAT_WD_STAT_0: 1;
	int STAT_WD_STAT_1: 1;
	int RD_CMD_ERR_STAT_0: 1;
	int RD_CMD_ERR_STAT_1: 1;
	int WR_STS_ERR_STAT_0: 1;
	int WR_STS_ERR_STAT_1: 1;
	int reserve: 18;
} CRYPTO_IRQ_STAT; /* RO */

typedef struct {
	int STS_ERR_CLR_0: 1;
	int STS_ERR_CLR_1: 1;
	int CMD_CLR_0: 1;
	int CMD_CLR_1: 1;
	int STAT_CLR_0: 1;
	int STAT_CLR_1: 1;
	int IOC_CLR_0: 1;
	int IOC_CLR_1: 1;
	int STAT_WD_CLR_0: 1;
	int STAT_WD_CLR_1: 1;
	int RD_CMD_ERR_CLR_0: 1;
	int RD_CMD_ERR_CLR_1: 1;
	int WR_STS_ERR_CLR_0: 1;
	int WR_STS_ERR_CLR_1: 1;
	int reserve: 18;
} CRYPTO_IRQ_CLR; /* W1C */

struct ips_irq_args {
	struct completion wait;
	int8_t ret_code;
	struct crypto_async_request *async_req;
};

inline struct ips_irq_args *get_irq_args(int index);
void ips_set_irq_num(int irq);
void ips_irq_init(void);
void ips_irq_uninit(void);
#endif
