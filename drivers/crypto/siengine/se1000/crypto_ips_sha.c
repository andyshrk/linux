// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains the SHA hash algorithm related operation functions.
 */

#include "crypto_ips_sha.h"

#include <linux/kernel.h>
#include <linux/slab.h>

#include "crypto_ips_init.h"
#include "crypto_ips_internal.h"
#include "crypto_res_mgr.h"
#include "ips_irq.h"

#if DESC("internal API")
static inline int get_sha_alg(u32 digest_size)
{
	switch (digest_size) {
	case SHA1_DIGEST_SIZE:
		return HASH_ALG_SHA_1;
	case SHA224_DIGEST_SIZE:
		return HASH_ALG_SHA_224;
	case SHA256_DIGEST_SIZE:
		return HASH_ALG_SHA_256;
	case SHA384_DIGEST_SIZE:
		return HASH_ALG_SHA_384;
	case SHA512_DIGEST_SIZE:
		return HASH_ALG_SHA_512;
	default:
		return -EINVAL;
	}
}

inline void ips_sha_init_ctx(struct ips_sha_ctx *sha_ctx, uint8_t digest_size)
{
	if (!cpu_latency_qos_request_active(&sha_ctx->req))
		cpu_latency_qos_add_request(&sha_ctx->req, 0);

	sha_ctx->digest_size = digest_size;
	sha_ctx->hash_alg = get_sha_alg(digest_size);
	sha_ctx->hash_mode = HASH_MODE_RAW;
	sha_ctx->key_index = -1;
	sha_ctx->key_size = 0;
	sha_ctx->sram_index = -1;
}

void ips_sha_free_ctx_res(struct ips_sha_ctx *sha_ctx)
{
	if (sha_ctx->key_index >= 0) {
		ips_clear_hmac_key(sha_ctx->key_index);
		sha_ctx->key_index = -1;
	}

	if (sha_ctx->sram_index >= 0) {
		free_vfx_shmem(sha_ctx->sram_index);
		sha_ctx->sram_index = -1;
	}

	if (cpu_latency_qos_request_active(&sha_ctx->req))
		cpu_latency_qos_remove_request(&sha_ctx->req);
}
#endif

#if DESC("SHA API")
int sha_hash_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct ips_sha_ctx *sha_ctx = crypto_ahash_ctx(tfm);

	ips_sha_init_ctx(sha_ctx, (uint8_t)crypto_ahash_digestsize(tfm));
	return 0;
}

static inline int sha_hash_update_dma(struct ips_sha_ctx *sha_ctx,
	struct scatterlist *src, uint32_t src_len, struct crypto_async_request *async_req)
{
	int ret;

	sha_ctx->dma_src = src;
	sha_ctx->src_len = src_len;

	ret = ips_sha_cmd_setup(sha_ctx, MEM_DMA_MODE, async_req);
	if (ret < 0) {
		pr_err("ips_sha_cmd_setup failed: %02x\n", ret);
		return ret;
	}
	return -EINPROGRESS;
}

static int sha_hash_update_ddt(struct ips_sha_ctx *sha_ctx,
	struct scatterlist *src, uint32_t src_len, struct crypto_async_request *async_req)
{
	int ret, sg_size;
	struct device *dev = get_ips_dev();

	sg_size = sg_nents(src);
	sha_ctx->ddt_src = sg_to_ddt(dev, src, DMA_TO_DEVICE);
	sha_ctx->src_len = src_len;
	dma_sync_sg_for_device(dev, src, sg_size, DMA_TO_DEVICE);

	ret = ips_sha_cmd_setup(sha_ctx, MEM_DDT_MODE, async_req);
	dma_unmap_sg(dev, src, sg_size, DMA_TO_DEVICE);
	kfree(sha_ctx->ddt_src);
	sha_ctx->ddt_src = NULL;

	if (ret < 0) {
		pr_err("ips_sha_cmd_setup failed: %02x\n", ret);
		return ret;
	}
	return -EINPROGRESS;
}

int sha_hash_update(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct ips_sha_ctx *sha_ctx = crypto_ahash_ctx(tfm);

	if (!check_sg_valid(req->src))
		return -EINVAL;

	if (get_ddt_mode())
		return sha_hash_update_ddt(sha_ctx, req->src, req->nbytes, &(req->base));
	else
		return sha_hash_update_dma(sha_ctx, req->src, req->nbytes, &(req->base));
}

int sha_hash_final(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct ips_sha_ctx *sha_ctx = crypto_ahash_ctx(tfm);
	int ret;

	if (get_ddt_mode())
		ret = ips_sha_cmd_finish(sha_ctx, MEM_DDT_MODE, req->result);
	else
		ret = ips_sha_cmd_finish(sha_ctx, MEM_DMA_MODE, req->result);
	ips_sha_free_ctx_res(sha_ctx);

	return ret;
}

int sha_init_tfm(struct crypto_ahash *tfm)
{
	ips_irq_init();
	return 0;
}

void sha_exit_tfm(struct crypto_ahash *tfm)
{
	struct ips_sha_ctx *sha_ctx = crypto_ahash_ctx(tfm);

	ips_sha_free_ctx_res(sha_ctx);
	ips_irq_uninit();
}

static struct ahash_alg hash_algs[] = {
	{
		.init                   = sha_hash_init,
		.update                 = sha_hash_update,
		.final                  = sha_hash_final,
		.init_tfm               = sha_init_tfm,
		.exit_tfm               = sha_exit_tfm,
		.halg.digestsize        = SHA1_DIGEST_SIZE,
		.halg.statesize         = sizeof(struct shash_desc),
		.halg.base.cra_ctxsize      = sizeof(struct ips_sha_ctx),
		.halg.base.cra_name         = "se-sha1",
		.halg.base.cra_driver_name  = "siengine crypto driver",
		.halg.base.cra_blocksize    = SHA1_BLOCK_SIZE,
		.halg.base.cra_module       = THIS_MODULE,
		.halg.base.cra_priority     = 200,
	},

	{
		.init                   = sha_hash_init,
		.update                 = sha_hash_update,
		.final                  = sha_hash_final,
		.init_tfm               = sha_init_tfm,
		.exit_tfm               = sha_exit_tfm,
		.halg.digestsize        = SHA224_DIGEST_SIZE,
		.halg.statesize         = sizeof(struct shash_desc),
		.halg.base.cra_ctxsize      = sizeof(struct ips_sha_ctx),
		.halg.base.cra_name         = "se-sha224",
		.halg.base.cra_driver_name  = "siengine crypto driver",
		.halg.base.cra_blocksize    = SHA224_BLOCK_SIZE,
		.halg.base.cra_module       = THIS_MODULE,
		.halg.base.cra_priority     = 200,
	},

	{
		.init                   = sha_hash_init,
		.update                 = sha_hash_update,
		.final                  = sha_hash_final,
		.init_tfm               = sha_init_tfm,
		.exit_tfm               = sha_exit_tfm,
		.halg.digestsize        = SHA256_DIGEST_SIZE,
		.halg.statesize         = sizeof(struct shash_desc),
		.halg.base.cra_ctxsize      = sizeof(struct ips_sha_ctx),
		.halg.base.cra_name         = "se-sha256",
		.halg.base.cra_driver_name  = "siengine crypto driver",
		.halg.base.cra_blocksize    = SHA256_BLOCK_SIZE,
		.halg.base.cra_module       = THIS_MODULE,
		.halg.base.cra_priority     = 200,
	},

	{
		.init                   = sha_hash_init,
		.update                 = sha_hash_update,
		.final                  = sha_hash_final,
		.init_tfm               = sha_init_tfm,
		.exit_tfm               = sha_exit_tfm,
		.halg.digestsize        = SHA384_DIGEST_SIZE,
		.halg.statesize         = sizeof(struct shash_desc),
		.halg.base.cra_ctxsize      = sizeof(struct ips_sha_ctx),
		.halg.base.cra_name         = "se-sha384",
		.halg.base.cra_driver_name  = "siengine crypto driver",
		.halg.base.cra_blocksize    = SHA384_BLOCK_SIZE,
		.halg.base.cra_module       = THIS_MODULE,
		.halg.base.cra_priority     = 200,
	},

	{
		.init                   = sha_hash_init,
		.update                 = sha_hash_update,
		.final                  = sha_hash_final,
		.init_tfm               = sha_init_tfm,
		.exit_tfm               = sha_exit_tfm,
		.halg.digestsize        = SHA512_DIGEST_SIZE,
		.halg.statesize         = sizeof(struct shash_desc),
		.halg.base.cra_ctxsize      = sizeof(struct ips_sha_ctx),
		.halg.base.cra_name         = "se-sha512",
		.halg.base.cra_driver_name  = "siengine crypto driver",
		.halg.base.cra_blocksize    = SHA512_BLOCK_SIZE,
		.halg.base.cra_module       = THIS_MODULE,
		.halg.base.cra_priority     = 200,
	},
};

#endif

inline void register_sha(void)
{
	crypto_register_ahashes(hash_algs, ARRAY_SIZE(hash_algs));
}

inline void unregister_sha(void)
{
	crypto_unregister_ahashes(hash_algs, ARRAY_SIZE(hash_algs));
}
