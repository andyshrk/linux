// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains IPS driver internal functions.
 */

#include <linux/kernel.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/completion.h>
#include <linux/pm_qos.h>

#include "crypto_ips_internal.h"
#include "crypto_ips_init.h"
#include "crypto_ips_aes.h"
#include "ips_irq.h"
#include "ips_reg.h"
#include "crypto_util.h"
#include "crypto_res_mgr.h"

uint32_t g_use_vf;
bool g_ddt_mode; /* is current domain support ddt mode ? */

/* C0_MAILBOX_0 reg lock */
static spinlock_t g_cmd_reg_lock = __SPIN_LOCK_UNLOCKED(g_cmd_reg_lock);

/* ips device */
static struct device *g_ips_device;

inline struct device *get_ips_dev(void)
{
	return g_ips_device;
}

inline void set_ips_dev(struct device *dev)
{
	g_ips_device = dev;
}

inline bool get_ddt_mode(void)
{
	return g_ddt_mode;
}

inline void set_ddt_mode(uint32_t ddt)
{
	g_ddt_mode = ddt > 0 ? true : false;
}

inline void ips_set_aes_key(uint8_t key_index, const uint8_t *aes_key,
							uint8_t key_len)
{
	uint32_t *key = (uint32_t *)aes_key;
	int i;

	for (i = 0; i < key_len / 4; i++)
		writel(key[i], IPS_VFx_KEYx(key_index) + 0x4 * i);
}

inline void ips_set_aes_iv(uint8_t key_index, const uint8_t *iv)
{
	const uint32_t *_iv = (const uint32_t *)iv;
	int i;

	for (i = 0; i < 4; i++)
		writel(_iv[i], IPS_VFx_IVx(key_index) + 0x4 * i);
}

inline void ips_clear_aes_key_iv(uint8_t key_index)
{
	memset_io(IPS_VFx_KEYx(key_index), 0, 0x20);
	memset_io(IPS_VFx_IVx(key_index), 0, 0x10);
	free_aes_keybuffer(key_index);
}

inline void ips_set_hmac_key(uint8_t key_index, const uint8_t *hmac_key,
							 uint8_t key_len)
{
	uint32_t *key = (uint32_t *)hmac_key;
	int i;

	for (i = 0; i < key_len / 4; i++)
		writel(key[i], IPS_VFx_HASHKEYx(key_index) + 0x4 * i);
}

inline void ips_clear_hmac_key(uint8_t key_index)
{
	memset_io(IPS_VFx_HASHKEYx(key_index), 0, 0x80);
	free_hmac_keybuffer(key_index);
}

static int get_next_cmd_index(void)
{
	uint32_t producer_idx;
	uint32_t consumer_idx;

	producer_idx = READ_REG(IPS_VFx_REG_C0_MAILBOX_0);
	consumer_idx = READ_REG(IPS_VFx_REG_C0_MAILBOX_1);

	/* fifo status full */
	if ((producer_idx + 1) % VFx_CMD_COUNT == consumer_idx) {
		pr_err("command ring is full!");
		return -EBUSY;
	}

	return producer_idx;
}

int ips_aes_cmd_setup(struct ips_aes_ctx *ctx, int crypt_mode, int mem_mode)
{
	int cmd_idx;
	struct ips_command *cmd = NULL;
	uint64_t src_phys;
	uint64_t dst_phys;
	struct ips_irq_args *args = NULL;

	/* wait for cmd available */
	if (alloc_cmd_desc() < 0) {
		pr_err("ips no cmd aviliable.\n");
		return -EBUSY;
	}

	if (ctx->dram_index < 0) {
		ctx->dram_index = alloc_vfx_shmem();
		if (ctx->dram_index < 0)
			return ctx->dram_index;
	}

	/* copy user input */
	if (mem_mode == MEM_DDT_MODE) {
		memcpy(SHMEM_SRC(ctx->dram_index), ctx->ddt_src, DDT_LIST_SIZE(ctx->ddt_src));
		memcpy(SHMEM_DST(ctx->dram_index), ctx->ddt_dst, DDT_LIST_SIZE(ctx->ddt_dst));
	} else
		sg_copy_to_buffer(ctx->dma_src, sg_nents(ctx->dma_src),
			SHMEM_SRC(ctx->dram_index), ctx->src_len);

	spin_lock(&g_cmd_reg_lock);
	cmd_idx = get_next_cmd_index();
	if (unlikely(cmd_idx < 0)) {
		spin_unlock(&g_cmd_reg_lock);
		free_vfx_shmem(ctx->dram_index);
		return -EBUSY;
	}
	ctx->cmd_idx = cmd_idx;

	/* init cmd args */
	args = get_irq_args(cmd_idx);
	args->async_req = NULL;
	reinit_completion(&args->wait);

	cmd = SHMEM_IPS_CMD(cmd_idx);
	memset_io(cmd, 0, sizeof(struct ips_command));
	src_phys = SHMEM_SRC_PHYS(ctx->dram_index);
	dst_phys = SHMEM_DST_PHYS(ctx->dram_index);

	/* fill parameters */
	cmd->src_pkt_addr_l = (uint32_t)src_phys;
	cmd->src_pkt_addr_h = (uint32_t)(src_phys >> 32);
	cmd->dst_pkt_addr_l = (uint32_t)dst_phys;
	cmd->dst_pkt_addr_h = (uint32_t)(dst_phys >> 32);
	cmd->proc_len = ctx->src_len;

	if (ctx->cipher_mode == CIPH_MODE_GCM) {
		cmd->pre_aad_len = ctx->aad_len;
		cmd->aad_copy = 1;

		if (crypt_mode == DECRYPT_MODE) {
			cmd->proc_len -= ctx->auth_size;
			ctx->dst_len -= ctx->auth_size;
		} else
			ctx->dst_len += ctx->auth_size;

		cmd->icv_offset = cmd->proc_len;
	}

	cmd->key_exp = 1;
	cmd->icv_len = ctx->auth_size;
	cmd->encrypt = crypt_mode;
	cmd->ctx_idx = ctx->key_index;
	cmd->ddt_mode = mem_mode;
	cmd->cipher_mode = ctx->cipher_mode;
	cmd->cipher_alg = ctx->cipher_alg;
	cmd->ioc = 1; /* this cmd finished will trigger an interrupt */
	cmd->cbc_cs_sel = ctx->cbc_cs_sel;
	cmd->sw_id = cmd_idx;

	WRITE_REG(IPS_VFx_REG_C0_MAILBOX_0, (cmd_idx + 1) % VFx_CMD_COUNT);
	spin_unlock(&g_cmd_reg_lock);

	return 0;
}

int ips_aes_cmd_finish(struct ips_aes_ctx *ctx, int mem_mode)
{
	struct ips_irq_args *args = get_irq_args(ctx->cmd_idx);

	/* wait for interrupt */
	if (!wait_for_completion_timeout(&args->wait, 5 * HZ)) {
		args->ret_code = -ETIME;
		pr_err("ips aes cmd timeout!\n");
	}

	if (args->ret_code == 0 && mem_mode == MEM_DMA_MODE) {
		sg_copy_from_buffer(ctx->dma_dst, sg_nents(ctx->dma_dst), SHMEM_DST(ctx->dram_index), ctx->dst_len);
	}

	free_vfx_shmem(ctx->dram_index);
	ctx->dram_index = -1;

	if (args->ret_code != 0 && args->ret_code != -ETIME) {
		pr_err("ips aes cmd failed %02x.\n", args->ret_code);
		args->ret_code =  -EINVAL;
	}

	return args->ret_code;
}

int ips_sha_cmd_setup(struct ips_sha_ctx *ctx, int mem_mode, struct crypto_async_request *async_req)
{
	int cmd_idx;
	struct ips_command *cmd = NULL;
	uint64_t src_phys;
	uint64_t dst_phys;
	struct ips_irq_args *args = NULL;
	struct ddt_pdu *dst_ddt = NULL;
	uint32_t ddtlist_size;
	uint32_t ddt_len;
	uint8_t *digiest_addr = NULL;

	/* wait for cmd available */
	alloc_cmd_desc();

	if (ctx->sram_index < 0) {
		ctx->sram_index = alloc_vfx_shmem();
		if (ctx->sram_index < 0)
			return ctx->sram_index;
	}

	spin_lock(&g_cmd_reg_lock);
	cmd_idx = get_next_cmd_index();
	if (unlikely(cmd_idx < 0)) {
		spin_unlock(&g_cmd_reg_lock);
		free_vfx_shmem(ctx->sram_index);
		return -EBUSY;
	}
	ctx->cmd_idx = cmd_idx;

	/* init cmd args */
	args = get_irq_args(cmd_idx);
	args->async_req = async_req;
	reinit_completion(&args->wait);

	cmd = SHMEM_IPS_CMD(cmd_idx);
	memset(cmd, 0, sizeof(struct ips_command));
	src_phys = SHMEM_SRC_PHYS(ctx->sram_index);
	dst_phys = SHMEM_DST_PHYS(ctx->sram_index);

	/* fill parameters */
	cmd->src_pkt_addr_l = (uint32_t)src_phys;
	cmd->src_pkt_addr_h = (uint32_t)(src_phys >> 32);
	cmd->dst_pkt_addr_l = (uint32_t)dst_phys;
	cmd->dst_pkt_addr_h = (uint32_t)(dst_phys >> 32);
	cmd->encrypt = 1;
	cmd->proc_len = ctx->src_len;
	cmd->icv_offset = 0;
	cmd->hm_key_size = ctx->key_size;
	cmd->key_exp = 1;
	cmd->ctx_idx = ctx->key_index;
	cmd->ddt_mode = mem_mode;
	cmd->hash_mode = ctx->hash_mode;
	cmd->hash_alg = ctx->hash_alg;
	cmd->ioc = 1;
	cmd->sw_id = cmd_idx;

	/* copy user input */
	if (mem_mode == MEM_DDT_MODE) {
		ddtlist_size = DDT_LIST_SIZE(ctx->ddt_src);
		ddt_len = get_ddt_len(ctx->ddt_src);
		memcpy(SHMEM_SRC(ctx->sram_index), ctx->ddt_src, ddtlist_size);
		memcpy(SHMEM_DST(ctx->sram_index), ctx->ddt_src, ddtlist_size);

		dst_ddt = (struct ddt_pdu *)SHMEM_DST(ctx->sram_index);
		digiest_addr = SHMEM_DST(ctx->sram_index) + DRAM_MEM_SIZE - ctx->digest_size;
		dst_ddt[ddt_len].ptr = (uint8_t *)SHMEM_VIRT_TO_PHYS(digiest_addr);
		dst_ddt[ddt_len].len = ctx->digest_size;
		dst_ddt[ddt_len + 1].ptr = NULL;
		dst_ddt[ddt_len + 1].len = 0;

		cmd->icv_offset = cmd->proc_len;
	} else
		sg_copy_to_buffer(ctx->dma_src, sg_nents(ctx->dma_src),
			SHMEM_SRC(ctx->sram_index), ctx->src_len);

	WRITE_REG(IPS_VFx_REG_C0_MAILBOX_0, (cmd_idx + 1) % VFx_CMD_COUNT);
	spin_unlock(&g_cmd_reg_lock);

	return 0;
}

int ips_sha_cmd_finish(struct ips_sha_ctx *ctx, int mem_mode, uint8_t *result)
{
	struct ips_irq_args *args = get_irq_args(ctx->cmd_idx);
	uint8_t *digiest_addr = NULL;

	/* wait for interrupt */
	if (!wait_for_completion_timeout(&args->wait, 5 * HZ)) {
		args->ret_code = -ETIME;
		pr_err("ips sha cmd timeout.\n");
	}

	if (args->ret_code != 0) {
		free_vfx_shmem(ctx->sram_index);
		ctx->sram_index = -1;
		return args->ret_code;
	}

	if (mem_mode == MEM_DMA_MODE)
		memcpy(result, SHMEM_DST(ctx->sram_index), ctx->digest_size);
	else {
		digiest_addr = SHMEM_DST(ctx->sram_index) + DRAM_MEM_SIZE - ctx->digest_size;
		memcpy(result, digiest_addr, ctx->digest_size);
	}
	free_vfx_shmem(ctx->sram_index);
	ctx->sram_index = -1;

	return 0;
}

static uint8_t sampleCipherKey[] = {
	0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
	0x88, 0x99, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55
};

static uint8_t sampleCipherIv[] = {
	0xca, 0xfe, 0xba, 0xbe, 0xfa, 0xce, 0xdb, 0xad,
	0xde, 0xca, 0xf8, 0x88, 0x3d, 0x11, 0x59, 0x04
};

static uint8_t sampleAuthKey[] = {
	0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
	0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
	0xDE, 0xAD, 0xBE, 0xEF
};

static uint8_t sampleEspHdrData[] = {
	0x00, 0x00, 0x01, 0x2c, 0x00, 0x00, 0x00, 0x05
};

static uint8_t samplePayload[] = {
	0xd9, 0x31, 0x32, 0x25, 0xf8, 0x84, 0x06, 0xe5,
	0xa5, 0x59, 0x09, 0xc5, 0xaf, 0xf5, 0x26, 0x9a,
	0x86, 0xa7, 0xa9, 0x53, 0x15, 0x34, 0xf7, 0xda,
	0x2e, 0x4c, 0x30, 0x3d, 0x8a, 0x31, 0x8a, 0x72,
	0x1c, 0x3c, 0x0c, 0x95, 0x95, 0x68, 0x09, 0x53,
	0x2f, 0xcf, 0x0e, 0x24, 0x49, 0xa6, 0xb5, 0x25,
	0xb1, 0x6a, 0xed, 0xf5, 0xaa, 0x0d, 0xe6, 0x57,
	0xba, 0x63, 0x7b, 0x39, 0x01, 0x02, 0x02, 0x61
};

static uint8_t expectedOutput[] = {
	/* ESP header unmodified */
	0x00, 0x00, 0x01, 0x2c, 0x00, 0x00, 0x00, 0x05,
	/* IV unmodified */
	0xca, 0xfe, 0xba, 0xbe, 0xfa, 0xce, 0xdb, 0xad,
	0xde, 0xca, 0xf8, 0x88, 0x3d, 0x11, 0x59, 0x04,
	/* Ciphertext */
	0x39, 0x8E, 0x4C, 0x1B, 0x7B, 0x28, 0x94, 0x52,
	0x97, 0xAD, 0x95, 0x97, 0xD7, 0xF9, 0xB9, 0x4A,
	0x49, 0x03, 0x51, 0x47, 0x45, 0xC7, 0x58, 0x6A,
	0x9A, 0x48, 0xB6, 0x38, 0xB4, 0xD5, 0xEE, 0x42,
	0x4F, 0x39, 0x09, 0x3D, 0xAB, 0x1E, 0xB3, 0x6A,
	0x71, 0x0B, 0xFC, 0x80, 0xAD, 0x2E, 0x4C, 0xA5,
	0xAB, 0x78, 0xB8, 0xAB, 0x87, 0xCC, 0x37, 0xF0,
	0xB9, 0x61, 0xDC, 0xB1, 0xA7, 0x24, 0x26, 0x23,
	/* ICV */
	0xE6, 0x55, 0xBD, 0x90, 0x33, 0x2D, 0x04, 0x8C,
	0x34, 0x06, 0xE3, 0x2D
};

int ips_ipsec_out_cmd_finish(uint8_t cmd_idx, uint8_t sram_idx)
{
	struct ips_irq_args *args = NULL;
	int ret = 0;
	uint8_t *dst;

	/* wait for interrupt */
	args = get_irq_args(cmd_idx);
	wait_for_completion_timeout(&args->wait, 5 * HZ);

	ips_irq_uninit();

	dst = SHMEM_DST(sram_idx);
	print_hex_dump(KERN_INFO, " send  outbound packet: ", DUMP_PREFIX_OFFSET, 16, 1,
				   dst, sizeof(expectedOutput), 0);

	ret = memcmp(expectedOutput, dst, sizeof(expectedOutput));
	if (ret == 0)
		pr_info("ipsec outbound packet check success.");
	else
		pr_info("ipsec outbound packet check failed.");

	free_vfx_shmem(sram_idx);
	return args->ret_code;
}

int ips_ipsec_out_cmd_setup(void)
{
	int8_t key_index;
	int8_t sram_index;
	int cmd_idx;
	struct ips_command *cmd = NULL;
	uint64_t src_phys;
	uint64_t dst_phys;
	struct ips_irq_args *args = NULL;
	uint8_t src[100];
	uint8_t src_len = 0;

	sram_index = alloc_vfx_shmem();
	if (sram_index < 0)
		return -EBUSY;

	/* wait for cmd available */
	alloc_cmd_desc();
	ips_irq_init();

	key_index = alloc_ipsec_keybuffer();
	if (key_index < 0)
		return key_index;

	ips_set_aes_key(key_index, sampleCipherKey, 16);
	ips_set_hmac_key(key_index, sampleAuthKey, 20);

	/* copy user input */
	memcpy(src + src_len, sampleEspHdrData, sizeof(sampleEspHdrData));
	src_len += sizeof(sampleEspHdrData);
	memcpy(src + src_len, sampleCipherIv, sizeof(sampleCipherIv));
	src_len += sizeof(sampleCipherIv);
	memcpy(src + src_len, samplePayload, sizeof(samplePayload));
	src_len += sizeof(samplePayload);
	memcpy(SHMEM_SRC(sram_index), src, src_len);
	print_hex_dump(KERN_INFO, "origin outbound packet: ", DUMP_PREFIX_OFFSET, 16, 1,
				   src, src_len, 0);

	spin_lock(&g_cmd_reg_lock);
	cmd_idx = get_next_cmd_index();
	if (unlikely(cmd_idx < 0)) {
		spin_unlock(&g_cmd_reg_lock);
		return -EBUSY;
	}

	/* init cmd args */
	args = get_irq_args(cmd_idx);
	reinit_completion(&args->wait);
	cmd = SHMEM_IPS_CMD(cmd_idx);
	memset(cmd, 0, sizeof(struct ips_command));

	/* fill parameters */
	src_phys = SHMEM_SRC_PHYS(sram_index);
	dst_phys = SHMEM_DST_PHYS(sram_index);
	cmd->src_pkt_addr_l = (uint32_t)src_phys;
	cmd->src_pkt_addr_h = (uint32_t)(src_phys >> 32);
	cmd->dst_pkt_addr_l = (uint32_t)dst_phys;
	cmd->dst_pkt_addr_h = (uint32_t)(dst_phys >> 32);
	cmd->src_pkt_off = 0;
	cmd->dst_pkt_off = 0;
	cmd->pre_aad_len = sizeof(sampleEspHdrData) + sizeof(sampleCipherIv);
	cmd->proc_len = src_len;
	cmd->iv_enable = 1;
	cmd->iv_offset = sizeof(sampleEspHdrData);
	cmd->aad_copy = 1;
	cmd->encrypt = 1;
	cmd->icv_pt = 0;
	cmd->icv_offset = src_len;
	cmd->icv_len = 12;
	cmd->ctx_idx = key_index;
	cmd->ddt_mode = MEM_DMA_MODE;
	cmd->cipher_alg = CIPH_ALG_AES_128;
	cmd->cipher_mode = CIPH_MODE_CBC;
	cmd->hash_mode = HASH_MODE_HMAC;
	cmd->hm_key_size = 20;
	cmd->hash_alg = HASH_ALG_SHA_1;
	cmd->ioc = 1;
	cmd->sw_id = cmd_idx;

	WRITE_REG(IPS_VFx_REG_C0_MAILBOX_0, (cmd_idx + 1) % VFx_CMD_COUNT);
	spin_unlock(&g_cmd_reg_lock);

	return ips_ipsec_out_cmd_finish(cmd_idx, sram_index);
}

int ips_ipsec_in_cmd_finish(uint8_t cmd_idx, uint8_t sram_idx)
{
	struct ips_irq_args *args = NULL;
	int ret = 0;
	uint8_t *dst;
	uint32_t i = 0;
	uint8_t expectedResult[200];

	memcpy(expectedResult + i, sampleEspHdrData, sizeof(sampleEspHdrData));
	i += sizeof(sampleEspHdrData);
	memcpy(expectedResult + i, sampleCipherIv, sizeof(sampleCipherIv));
	i += sizeof(sampleCipherIv);
	memcpy(expectedResult + i, samplePayload, sizeof(samplePayload));
	i += sizeof(samplePayload);
	memcpy(expectedResult + i, expectedOutput + i, 12);
	i += 12;

	/* wait for interrupt */
	args = get_irq_args(cmd_idx);
	wait_for_completion_timeout(&args->wait, 5 * HZ);
	ips_irq_uninit();

	if (args->ret_code) {
		return args->ret_code;
	}

	dst = SHMEM_DST(sram_idx);
	print_hex_dump(KERN_INFO, " recv  inbound packet: ", DUMP_PREFIX_OFFSET, 16, 1,
				   dst, sizeof(expectedOutput), 0);

	ret = memcmp(expectedResult, dst, sizeof(expectedOutput));
	if (ret == 0) {
		pr_info("ipsec inbound packet check success.");
	} else {
		pr_info("ipsec inbound packet check failed.");
	}

	free_vfx_shmem(sram_idx);
	return args->ret_code;
}

int ips_ipsec_in_cmd_setup(void)
{
	int8_t key_index;
	int8_t sram_index;
	int cmd_idx;
	struct ips_command *cmd = NULL;
	uint64_t src_phys;
	uint64_t dst_phys;
	struct ips_irq_args *args = NULL;

	sram_index = alloc_vfx_shmem();
	if (sram_index < 0)
		return -EBUSY;

	/* wait for cmd available */
	alloc_cmd_desc();
	ips_irq_init();

	key_index = alloc_ipsec_keybuffer();
	ips_set_aes_key(key_index, sampleCipherKey, 16);
	ips_set_hmac_key(key_index, sampleAuthKey, 20);

	/* copy ipsec inbound packets */
	print_hex_dump(KERN_INFO, "origin inbound packet: ", DUMP_PREFIX_OFFSET, 16, 1,
				   expectedOutput, sizeof(expectedOutput), 0);
	memcpy(SHMEM_SRC(sram_index), expectedOutput, sizeof(expectedOutput));

	spin_lock(&g_cmd_reg_lock);
	cmd_idx = get_next_cmd_index();
	if (unlikely(cmd_idx < 0)) {
		spin_unlock(&g_cmd_reg_lock);
		return -EBUSY;
	}

	/* init irq args */
	args = get_irq_args(cmd_idx);
	reinit_completion(&args->wait);

	cmd = SHMEM_IPS_CMD(cmd_idx);
	memset(cmd, 0, sizeof(struct ips_command));

	/* fill parameters */
	src_phys = SHMEM_SRC_PHYS(sram_index);
	dst_phys = SHMEM_DST_PHYS(sram_index);
	cmd->src_pkt_addr_l = (uint32_t)src_phys;
	cmd->src_pkt_addr_h = (uint32_t)(src_phys >> 32);
	cmd->dst_pkt_addr_l = (uint32_t)dst_phys;
	cmd->dst_pkt_addr_h = (uint32_t)(dst_phys >> 32);
	cmd->src_pkt_off = 0;
	cmd->dst_pkt_off = 0;
	cmd->pre_aad_len = sizeof(sampleEspHdrData) + sizeof(sampleCipherIv);
	cmd->proc_len = cmd->pre_aad_len + sizeof(samplePayload);
	cmd->iv_enable = 1;
	cmd->iv_offset = sizeof(sampleEspHdrData);
	cmd->aad_copy = 1;
	cmd->encrypt = 0;
	cmd->icv_pt = 0;
	cmd->icv_offset = cmd->proc_len;
	cmd->icv_len = 12;
	cmd->icv_append = 1;
	cmd->ctx_idx = key_index;
	cmd->key_exp = 1;
	cmd->ddt_mode = MEM_DMA_MODE;
	cmd->cipher_alg = CIPH_ALG_AES_128;
	cmd->cipher_mode = CIPH_MODE_CBC;
	cmd->hash_mode = HASH_MODE_HMAC;
	cmd->hm_key_size = 20;
	cmd->hash_alg = HASH_ALG_SHA_1;
	cmd->ioc = 1;
	cmd->sw_id = cmd_idx;

	WRITE_REG(IPS_VFx_REG_C0_MAILBOX_0, (cmd_idx + 1) % VFx_CMD_COUNT);
	spin_unlock(&g_cmd_reg_lock);

	return ips_ipsec_in_cmd_finish(cmd_idx, sram_index);
}
