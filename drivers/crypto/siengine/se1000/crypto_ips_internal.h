// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains IPS driver internal functions.
 */

#ifndef __CRYPTO_IPS_INTERNAL_H__
#define __CRYPTO_IPS_INTERNAL_H__

#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <crypto/if_alg.h>

#include "crypto_ips_init.h"
#include "crypto_ips_aes.h"
#include "crypto_ips_sha.h"
#include "ips_reg.h"

struct ips_command {
	unsigned int src_pkt_addr_l;
	unsigned int src_pkt_addr_h;
	unsigned int dst_pkt_addr_l;
	unsigned int dst_pkt_addr_h;
	unsigned int dst_pkt_off : 16;
	unsigned int src_pkt_off : 16;
	unsigned int pre_aad_len;
	unsigned int post_aad_len;
	unsigned int proc_len;
	unsigned int icv_offset;
	unsigned int iv_offset : 31;
	unsigned int iv_enable : 1;
	/*control  */
	unsigned int hm_key_size : 8;
	unsigned int reserved : 2;
	unsigned int key_exp : 1;
	unsigned int icv_append : 1;
	unsigned int icv_enc : 1;
	unsigned int icv_pt : 1;
	unsigned int aad_copy : 1;
	unsigned int encrypt : 1;
	unsigned int ctx_idx : 3;
	unsigned int ddt_mode : 1;
	unsigned int hash_mode : 1;
	unsigned int cipher_mode : 3;
	unsigned int hash_alg : 4;
	unsigned int cipher_alg : 3;
	unsigned int ioc : 1;
	/* aux_info  */
	unsigned int icv_len : 7;
	unsigned int cbc_cs_sel : 2;
	unsigned int bk_sz_cfb : 7;
	unsigned int sw_id : 16;
	unsigned int reserved1;
	unsigned int reserved2;
	unsigned int reserved3;
	unsigned int reserved4;
};

struct ips_status {
	unsigned int res : 8;
	unsigned int ret_code : 6;
	unsigned int unum : 2;
	unsigned int sw_id : 16;
	unsigned int reserved;
};

inline bool get_ddt_mode(void);
inline void set_ddt_mode(uint32_t ddt);

inline struct device *get_ips_dev(void);
inline void set_ips_dev(struct device *dev);

inline void ips_set_aes_key(uint8_t key_index, const uint8_t *aes_key,
							uint8_t key_len);
inline void ips_set_aes_iv(uint8_t key_index, const uint8_t *iv);
inline void ips_clear_aes_key_iv(uint8_t key_index);

inline void ips_set_hmac_key(uint8_t key_index, const uint8_t *hmac_key,
							 uint8_t key_len);
inline void ips_clear_hmac_key(uint8_t key_index);

int ips_aes_cmd_setup(struct ips_aes_ctx *ctx, int crypt_mode, int mem_mode);
int ips_aes_cmd_finish(struct ips_aes_ctx *ctx, int mem_mode);

int ips_sha_cmd_setup(struct ips_sha_ctx *ctx, int mem_mode, struct crypto_async_request *async_req);
int ips_sha_cmd_finish(struct ips_sha_ctx *ctx, int mem_mode, uint8_t *result);

int ips_ipsec_out_cmd_setup(void);
int ips_ipsec_in_cmd_setup(void);
#endif
