// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains the SHA hash algorithm related operation functions.
 */

#ifndef __CRYPTO_IPS_SHA_H__
#define __CRYPTO_IPS_SHA_H__

#include <crypto/sha.h>
#include <crypto/internal/hash.h>
#include <linux/pm_qos.h>

#include "crypto_util.h"

struct ips_sha_ctx {
	/* input args */
	uint8_t hash_alg;
	uint8_t hash_mode;
	uint8_t digest_size;

	/* resource index */
	int8_t key_index;
	uint8_t key_size;
	int8_t sram_index;

	uint8_t cmd_idx;
	union {
		struct scatterlist *dma_src;
		struct ddt_pdu *ddt_src;
	};
	uint32_t src_len;
	struct pm_qos_request req;
};

inline void register_sha(void);
inline void unregister_sha(void);
#endif
