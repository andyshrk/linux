// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains the implementation of SM4 algorithm related operations.
 */

#ifndef __CRYPTO_SMX_SM4_H__
#define __CRYPTO_SMX_SM4_H__

#include <crypto/sm4.h>
#include <linux/pm_qos.h>

struct sm4_ctx {
	/* input args */
	uint8_t sm4_mode;
	uint8_t cbc_cs_sel;
	uint8_t auth_size;

	/* resource index */
	int8_t key_index;
	int8_t sram_index;
	uint8_t cmd_idx;
	uint8_t msg_flag;

	union {
		struct scatterlist *dma_src;
		struct ddt_pdu *ddt_src;
	};
	union {
		struct scatterlist *dma_dst;
		struct ddt_pdu *ddt_dst;
	};
	uint32_t src_len;
	uint32_t dst_len;
	uint32_t aad_len;
	struct pm_qos_request req;
};

inline void register_sm4(void);
inline void unregister_sm4(void);
#endif
