// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains the SM3 hash algorithm related operation functions.
 */

#ifndef __CRYPTO_SMX_SM3_H__
#define __CRYPTO_SMX_SM3_H__

#include <crypto/sm3.h>
#include <linux/pm_qos.h>

struct sm3_ctx {
	int8_t sram_index;
	uint8_t cmd_idx;
	uint8_t msg_flag;
	union {
		struct scatterlist *dma_src;
		struct ddt_pdu *ddt_src;
	};
	uint32_t src_len;
	struct pm_qos_request req;
};

inline void register_sm3(void);
inline void unregister_sm3(void);
#endif
