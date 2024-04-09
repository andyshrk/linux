// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains the implementation of AES algorithm related operations.
 */

#ifndef __CRYPTO_IPS_AES_H__
#define __CRYPTO_IPS_AES_H__

#include <linux/crypto.h>
#include <crypto/aes.h>
#include <crypto/internal/skcipher.h>
#include <crypto/internal/aead.h>
#include <linux/pm_qos.h>

#include "crypto_util.h"

struct ips_aes_ctx {
	/* input args */
	uint8_t cipher_alg;
	uint8_t cipher_mode;
	uint8_t cbc_cs_sel;
	uint8_t auth_size;

	/* resource index */
	int8_t key_index;
	int8_t dram_index;
	uint8_t cmd_idx;

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
	uint8_t iv[AES_BLOCK_SIZE];
	struct pm_qos_request req;
	bool more;
};

inline void register_aes(void);
inline void unregister_aes(void);

#endif
