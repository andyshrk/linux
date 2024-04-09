// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains SMX internal functions definition.
 */

#ifndef __CRYPTO_SMX_INTERNAL_H__
#define __CRYPTO_SMX_INTERNAL_H__

#include "smx_reg.h"
#include "smx_irq.h"
#include "crypto_smx_sm4.h"
#include "crypto_smx_sm3.h"

#define MSG_BEGIN	0x01
#define MSG_MID		0x00
#define MSG_END		0x02
#define MSG_ONESHOT	(MSG_BEGIN | MSG_END)

struct smx_command {
	uint32_t src_pkt_addr_l;
	uint32_t src_pkt_addr_h;
	uint32_t dst_pkt_addr_l;
	uint32_t dst_pkt_addr_h;
	uint32_t dst_pkt_off : 16;
	uint32_t src_pkt_off : 16;
	uint32_t pre_aad_len;
	uint32_t post_aad_len;
	uint32_t proc_len;
	uint32_t icv_offset;
	uint32_t iv_offset : 31;
	uint32_t iv_enable : 1;

	/* control */
	uint32_t reserved : 8;
	uint32_t msg_end : 1;
	uint32_t msg_begin : 1;
	uint32_t key_exp : 1;
	uint32_t icv_append : 1;
	uint32_t icv_enc : 1;
	uint32_t icv_pt : 1;
	uint32_t aad_copy : 1;
	uint32_t encrypt : 1;
	uint32_t ctx_idx : 3;
	uint32_t ddt_mode : 1;
	uint32_t key_port : 1;
	uint32_t reserved6 : 6;
	uint32_t sm3_mode : 1;
	uint32_t sm4_mode : 3;
	uint32_t ioc : 1;

	/* aux_info  */
	uint32_t icv_len : 6;
	uint32_t cbc_cs_sel : 2;
	uint32_t bk_sz_cfb : 7;
	uint32_t reserved5 : 1;
	uint32_t sw_id : 16;
	uint32_t reserved1;
	uint32_t reserved2;
	uint32_t reserved3;
	uint32_t reserved4;
};


struct smx_status {
	uint32_t res : 8;
	uint32_t ret_code : 6;
	uint32_t unum : 2;
	uint32_t sw_id : 16;
	uint32_t reserved;
};

inline struct device *get_smx_dev(void);
inline void set_smx_dev(struct device *dev);
inline bool get_smx_ddt_mode(void);
inline void set_smx_ddt_mode(uint32_t ddt);

void smx_set_sm4_key(uint8_t key_index, const uint8_t *sm4key);
void smx_set_sm4_iv(uint8_t key_index, const uint8_t *iv);
void smx_clear_sm4_key_iv(uint8_t key_index);

int sm4_cmd_setup(struct sm4_ctx *ctx, int crypt_mode, int mem_mode);
int sm4_cmd_finish(struct sm4_ctx *ctx, int mem_mode);
int sm3_cmd_setup(struct sm3_ctx *ctx, int mem_mode, struct crypto_async_request *async_req);
int sm3_cmd_finish(struct sm3_ctx *ctx, int mem_mode, uint8_t *result);
#endif
