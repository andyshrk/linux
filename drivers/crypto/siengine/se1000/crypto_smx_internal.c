// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains SMX internal functions.
 */

#include "smx_reg.h"
#include "crypto_smx_internal.h"
#include "crypto_smx_sm4.h"
#include "crypto_res_mgr.h"
#include "crypto_util.h"

/* according to SFC_IPS performance data: 200MB/s */
#define SMX_PROCESS_LEN_PER_US	200
#define SMX_DEFAULT_LATENCY		5  /* for data less than 1K */

#if DESC("SMX RESOURCES")
/* smx device */
static struct device *g_smx_device;
uint32_t g_smx_use_vf;
bool g_smx_ddt_mode; /* is current domain support ddt mode ? */

/* C0_MAILBOX_0 reg lock */
static spinlock_t g_smx_mailbox0_lock = __SPIN_LOCK_UNLOCKED(g_smx_mailbox0_lock);

inline struct device *get_smx_dev(void)
{
	return g_smx_device;
}

inline void set_smx_dev(struct device *dev)
{
	g_smx_device = dev;
}

inline bool get_smx_ddt_mode(void)
{
	return g_smx_ddt_mode;
}

inline void set_smx_ddt_mode(uint32_t ddt)
{
	g_smx_ddt_mode = ddt > 0 ? true : false;
}

#endif

#if DESC("SMX Key Operation")
void smx_set_sm4_key(uint8_t key_index, const uint8_t *sm4key)
{
	uint32_t *key = (uint32_t *)sm4key;
	int i;

	for (i = 0; i < 4; i++)
		writel(key[i], SMX_VFx_KEYx(key_index) + 0x4 * i);
}

void smx_set_sm4_iv(uint8_t key_index, const uint8_t *iv)
{
	const uint32_t *_iv = (const uint32_t *)iv;
	int i;

	for (i = 0; i < 4; i++)
		writel(_iv[i], SMX_VFx_IVx(key_index) + 0x4 * i);
}

void smx_clear_sm4_key_iv(uint8_t key_index)
{
	memset_io(SMX_VFx_KEYx(key_index), 0, 0x20);
	memset_io(SMX_VFx_IVx(key_index), 0, 0x10);
	free_sm4_keybuffer(key_index);
}
#endif

static int get_smx_next_cmd(void)
{
	uint32_t producer_idx;
	uint32_t consumer_idx;

	producer_idx = READ_REG(SMX_VFx_REG_C0_MAILBOX_0);
	consumer_idx = READ_REG(SMX_VFx_REG_C0_MAILBOX_1);

	/* fifo status full */
	if ((producer_idx + 1) % VFx_CMD_COUNT == consumer_idx) {
		pr_err("command ring is full!");
		return -EBUSY;
	}

	return producer_idx;
}

int sm4_cmd_setup(struct sm4_ctx *ctx, int crypt_mode, int mem_mode)
{
	int cmd_idx;
	struct smx_command cmd = {0};
	uint64_t src_phys;
	uint64_t dst_phys;
	struct smx_irq_args *args = NULL;

	/* wait for cmd available */
	if (alloc_smx_cmd() < 0) {
		pr_err("smx no cmd aviliable.\n");
		return -EBUSY;
	}

	ctx->sram_index = alloc_vfx_shmem();
	if (ctx->sram_index < 0) {
		pr_err("alloc shmem failed %d\n", ctx->sram_index);
		return ctx->sram_index;
	}

	/* copy user input */
	if (mem_mode == MEM_DDT_MODE) {
		memcpy(SHMEM_SRC(ctx->sram_index), ctx->ddt_src, DDT_LIST_SIZE(ctx->ddt_src));
		memcpy(SHMEM_DST(ctx->sram_index), ctx->ddt_dst, DDT_LIST_SIZE(ctx->ddt_dst));
	} else
		sg_copy_to_buffer(ctx->dma_src, sg_nents(ctx->dma_src), SHMEM_SRC(ctx->sram_index), ctx->src_len);

	/* fill parameters */
	src_phys = SHMEM_SRC_PHYS(ctx->sram_index);
	dst_phys = SHMEM_DST_PHYS(ctx->sram_index);
	cmd.src_pkt_addr_l = (uint32_t)src_phys;
	cmd.src_pkt_addr_h = (uint32_t)(src_phys >> 32);
	cmd.dst_pkt_addr_l = (uint32_t)dst_phys;
	cmd.dst_pkt_addr_h = (uint32_t)(dst_phys >> 32);
	cmd.proc_len = ctx->src_len;

	if (ctx->sm4_mode == SM4_GCM) {
		cmd.pre_aad_len = ctx->aad_len;
		cmd.aad_copy = 1;

		if (crypt_mode == DECRYPT_MODE) {
			cmd.proc_len -= ctx->auth_size;
			ctx->dst_len -= ctx->auth_size;
		} else
			ctx->dst_len += ctx->auth_size;

		cmd.icv_offset = cmd.proc_len;
	}

	cmd.key_exp = 1;
	cmd.icv_len = ctx->auth_size;
	cmd.encrypt = crypt_mode;
	cmd.ctx_idx = ctx->key_index;
	cmd.ddt_mode = mem_mode;
	cmd.sm4_mode = ctx->sm4_mode;
	cmd.sm3_mode = 0;
	cmd.ioc = 1; /* this cmd finished will trigger an interrupt */
	cmd.cbc_cs_sel = ctx->cbc_cs_sel;
	if (ctx->msg_flag & MSG_BEGIN) {
		cmd.msg_begin = 1;
		ctx->msg_flag &= ~MSG_BEGIN;
	}
	if (ctx->msg_flag & MSG_END)
		cmd.msg_end = 1;

	spin_lock(&g_smx_mailbox0_lock);
	cmd_idx = get_smx_next_cmd();
	if (unlikely(cmd_idx < 0)) {
		spin_unlock(&g_smx_mailbox0_lock);
		free_vfx_shmem(ctx->sram_index);
		return -EBUSY;
	}

	/* init cmd args */
	args = get_smx_irq_args(cmd_idx);
	reinit_completion(&args->wait);
	args->async_req = NULL;

	ctx->cmd_idx = cmd_idx;
	cmd.sw_id = cmd_idx;
	memcpy(SHMEM_SMX_CMD(cmd_idx), &cmd, sizeof(cmd));
	WRITE_REG(SMX_VFx_REG_C0_MAILBOX_0, (cmd_idx + 1) % VFx_CMD_COUNT);
	spin_unlock(&g_smx_mailbox0_lock);

	return 0;
}

int sm4_cmd_finish(struct sm4_ctx *ctx, int mem_mode)
{
	int ret = 0;
	struct smx_irq_args *args = get_smx_irq_args(ctx->cmd_idx);

	do {
		/* wait for interrupt */
		if (!wait_for_completion_timeout(&args->wait, 5 * HZ)) {
			ret = -ETIME;
			pr_err("sm4 cmd timeout.\n");
			break;
		}

		if (args->ret_code != 0) {
			ret = -EINVAL;
			pr_err("sm4 cmd failed %02x.\n", args->ret_code);
			break;
		}

		if (args->ret_code == 0 && mem_mode == MEM_DMA_MODE)
			sg_copy_from_buffer(ctx->dma_dst, sg_nents(ctx->dma_dst), SHMEM_DST(ctx->sram_index), ctx->dst_len);
	} while (0);
	free_vfx_shmem(ctx->sram_index);
	ctx->sram_index = -1;

	return ret;
}

int sm3_cmd_setup(struct sm3_ctx *ctx, int mem_mode, struct crypto_async_request *async_req)
{
	int cmd_idx;
	struct smx_command cmd = {0};
	uint64_t src_phys;
	uint64_t dst_phys;
	struct smx_irq_args *args = NULL;
	struct ddt_pdu *dst_ddt = NULL;
	uint32_t ddtlist_size;
	uint32_t ddt_len;
	uint8_t *digiest_addr = NULL;

	/* wait for cmd available */
	alloc_smx_cmd();

	if (ctx->sram_index < 0) {
		ctx->sram_index = alloc_vfx_shmem();
		if (ctx->sram_index < 0)
			return ctx->sram_index;
	}

	/* copy user input */
	if (mem_mode == MEM_DDT_MODE) {
		ddtlist_size = DDT_LIST_SIZE(ctx->ddt_src);
		ddt_len = get_ddt_len(ctx->ddt_src);
		memcpy_toio(SHMEM_SRC(ctx->sram_index), ctx->ddt_src, ddtlist_size);
		memcpy_toio(SHMEM_DST(ctx->sram_index), ctx->ddt_src, ddtlist_size);

		dst_ddt = (struct ddt_pdu *)SHMEM_DST(ctx->sram_index);
		digiest_addr = SHMEM_DST(ctx->sram_index) + DRAM_MEM_SIZE - SM3_DIGEST_SIZE;
		dst_ddt[ddt_len].ptr = (uint8_t *)SHMEM_VIRT_TO_PHYS(digiest_addr);
		dst_ddt[ddt_len].len = SM3_DIGEST_SIZE;
		dst_ddt[ddt_len + 1].ptr = NULL;
		dst_ddt[ddt_len + 1].len = 0;

		cmd.icv_offset = ctx->src_len;
	} else {
		sg_copy_to_buffer(ctx->dma_src, sg_nents(ctx->dma_src), SHMEM_SRC(ctx->sram_index), ctx->src_len);
		cmd.icv_offset = 0;
	}

	/* fill parameters */
	src_phys = SHMEM_SRC_PHYS(ctx->sram_index);
	dst_phys = SHMEM_DST_PHYS(ctx->sram_index);
	cmd.src_pkt_addr_l = (uint32_t)src_phys;
	cmd.src_pkt_addr_h = (uint32_t)(src_phys >> 32);
	cmd.dst_pkt_addr_l = (uint32_t)dst_phys;
	cmd.dst_pkt_addr_h = (uint32_t)(dst_phys >> 32);
	cmd.encrypt = 1;
	cmd.proc_len = ctx->src_len;
	cmd.ddt_mode = mem_mode;
	cmd.sm3_mode = SM3_RAW_MODE;
	cmd.ioc = 1;
	if (ctx->msg_flag & MSG_BEGIN) {
		cmd.msg_begin = 1;
		ctx->msg_flag &= ~MSG_BEGIN;
	}
	if (ctx->msg_flag & MSG_END)
		cmd.msg_end = 1;

	spin_lock(&g_smx_mailbox0_lock);

	cmd_idx = get_smx_next_cmd();
	if (unlikely(cmd_idx < 0)) {
		spin_unlock(&g_smx_mailbox0_lock);
		free_vfx_shmem(ctx->sram_index);
		return -EBUSY;
	}
	ctx->cmd_idx = cmd_idx;
	cmd.sw_id = cmd_idx;

	/* init cmd args */
	args = get_smx_irq_args(cmd_idx);
	reinit_completion(&args->wait);
	args->async_req = async_req;

	memcpy(SHMEM_SMX_CMD(cmd_idx), &cmd, sizeof(cmd));
	WRITE_REG(SMX_VFx_REG_C0_MAILBOX_0, (cmd_idx + 1) % VFx_CMD_COUNT);
	spin_unlock(&g_smx_mailbox0_lock);

	return 0;
}

int sm3_cmd_finish(struct sm3_ctx *ctx, int mem_mode, uint8_t *result)
{
	struct smx_irq_args *args = NULL;
	uint8_t *digiest_addr = NULL;

	args = get_smx_irq_args(ctx->cmd_idx);
	wait_for_completion_timeout(&args->wait, 5 * HZ);

	if (args->ret_code != 0) {
		free_vfx_shmem(ctx->sram_index);
		ctx->sram_index = -1;
		return args->ret_code;
	}

	if (mem_mode == MEM_DMA_MODE)
		memcpy(result, SHMEM_DST(ctx->sram_index), SM3_DIGEST_SIZE);
	else {
		digiest_addr = SHMEM_DST(ctx->sram_index) + DRAM_MEM_SIZE - SM3_DIGEST_SIZE;
		memcpy(result, digiest_addr, SM3_DIGEST_SIZE);
	}
	free_vfx_shmem(ctx->sram_index);
	ctx->sram_index = -1;

	return 0;
}
