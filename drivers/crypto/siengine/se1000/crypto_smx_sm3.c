// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains the SM3 hash algorithm related operation functions.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <crypto/hash.h>
#include <crypto/if_alg.h>

#include "crypto_smx_sm3.h"
#include "crypto_ips_internal.h"
#include "crypto_res_mgr.h"
#include "smx_irq.h"

#if DESC("internal API")
inline void sm3_init_ctx(struct sm3_ctx *ctx)
{
	if (!cpu_latency_qos_request_active(&ctx->req))
		cpu_latency_qos_add_request(&ctx->req, 0);
	ctx->sram_index = -1;
	ctx->msg_flag = MSG_BEGIN;
}

void sm3_free_ctx_res(struct sm3_ctx *ctx)
{
	if (ctx->sram_index >= 0) {
		free_vfx_shmem(ctx->sram_index);
		ctx->sram_index = -1;
	}

	if (cpu_latency_qos_request_active(&ctx->req))
		cpu_latency_qos_remove_request(&ctx->req);
}
#endif

#if DESC("SHA API")
int hash_sm3_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct sm3_ctx *ctx = crypto_ahash_ctx(tfm);

	sm3_init_ctx(ctx);
	return 0;
}

static inline int hash_sm3_update_dma(struct sm3_ctx *ctx,
	struct scatterlist *src, uint32_t src_len, struct crypto_async_request *async_req)
{
	int ret;

	ctx->dma_src = src;
	ctx->src_len = src_len;

	ret = sm3_cmd_setup(ctx, MEM_DMA_MODE, async_req);
	if (ret < 0) {
		pr_err("sm3_cmd_setup failed: %02x\n", ret);
		return ret;
	}
	return -EINPROGRESS;
}

static int hash_sm3_update_ddt(struct sm3_ctx *ctx,
	struct scatterlist *src, uint32_t src_len, struct crypto_async_request *async_req)
{
	int ret, sg_size;
	struct device *dev = get_smx_dev();

	sg_size = sg_nents(src);
	ctx->ddt_src = sg_to_ddt(dev, src, DMA_TO_DEVICE);
	ctx->src_len = src_len;
	dma_sync_sg_for_device(dev, src, sg_size, DMA_TO_DEVICE);

	ret = sm3_cmd_setup(ctx, MEM_DDT_MODE, async_req);
	dma_unmap_sg(dev, src, sg_size, DMA_TO_DEVICE);
	kfree(ctx->ddt_src);
	ctx->ddt_src = NULL;

	if (ret < 0) {
		pr_err("sm3_cmd_setup failed: %02x\n", ret);
		return ret;
	}

	return -EINPROGRESS;
}

int hash_sm3_update(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct sm3_ctx *ctx = crypto_ahash_ctx(tfm);

	if (!check_sg_valid(req->src) || (req->base.msg_more && req->nbytes < SM3_BLOCK_SIZE))
		return -EINVAL;

	if (!req->base.msg_more)
		ctx->msg_flag |= MSG_END;

	if (get_smx_ddt_mode())
		return hash_sm3_update_ddt(ctx, req->src, req->nbytes, &(req->base));
	else
		return hash_sm3_update_dma(ctx, req->src, req->nbytes, &(req->base));
}

int hash_sm3_final(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct sm3_ctx *ctx = crypto_ahash_ctx(tfm);
	int ret;

	if (get_smx_ddt_mode())
		ret = sm3_cmd_finish(ctx, MEM_DDT_MODE, req->result);
	else
		ret = sm3_cmd_finish(ctx, MEM_DMA_MODE, req->result);
	sm3_free_ctx_res(ctx);

	return ret;
}

void hash_sm3_exit_tfm(struct crypto_ahash *tfm)
{
	struct sm3_ctx *ctx = crypto_ahash_ctx(tfm);

	sm3_free_ctx_res(ctx);
}

static struct ahash_alg hash_algs[] = {
	{
		.init                   = hash_sm3_init,
		.update                 = hash_sm3_update,
		.final                  = hash_sm3_final,
		.exit_tfm               = hash_sm3_exit_tfm,
		.halg.digestsize        = SM3_DIGEST_SIZE,
		.halg.statesize         = sizeof(struct shash_desc),
		.halg.base.cra_ctxsize      = sizeof(struct sm3_ctx),
		.halg.base.cra_name         = "se-sm3",
		.halg.base.cra_driver_name  = "siengine crypto driver",
		.halg.base.cra_blocksize    = SM3_BLOCK_SIZE,
		.halg.base.cra_module       = THIS_MODULE,
		.halg.base.cra_priority     = 200,
	},
};

#endif

inline void register_sm3(void)
{
	crypto_register_ahashes(hash_algs, ARRAY_SIZE(hash_algs));
}

inline void unregister_sm3(void)
{
	crypto_unregister_ahashes(hash_algs, ARRAY_SIZE(hash_algs));
}
