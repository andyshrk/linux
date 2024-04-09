// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains the implementation of SM4 algorithm related operations.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <crypto/gcm.h>
#include <crypto/if_alg.h>

#include "crypto_smx_sm4.h"
#include "crypto_smx_init.h"
#include "crypto_smx_internal.h"
#include "crypto_res_mgr.h"
#include "smx_irq.h"

#if DESC("internal API")
int sm4_init_ctx(struct sm4_ctx *ctx, uint8_t sm4_mode)
{
	if (!ctx)
		return -EINVAL;

	if (!cpu_latency_qos_request_active(&ctx->req))
		cpu_latency_qos_add_request(&ctx->req, 0);

	ctx->sm4_mode = sm4_mode;
	ctx->cbc_cs_sel = 0;
	ctx->key_index = -1;
	ctx->sram_index = -1;
	ctx->msg_flag = MSG_BEGIN;
	if (sm4_mode == SM4_GCM)
		ctx->msg_flag = MSG_ONESHOT;

	return 0;
}

int sm4_set_key_iv(struct sm4_ctx *ctx, const uint8_t *key, const uint8_t *iv)
{
	int8_t key_index;

	if (ctx == NULL || key == NULL)
		return -EINVAL;

	key_index = alloc_sm4_keybuffer();
	if (key_index < 0) {
		pr_err("alloc sm4 keybuffer fail %d.\n", key_index);
		return key_index;
	}

	smx_set_sm4_key(key_index, key);
	if (iv != NULL)
		smx_set_sm4_iv(key_index, iv);
	ctx->key_index = key_index;
	ctx->msg_flag = MSG_BEGIN;

	return 0;
}

int sm4_update_dma(struct sm4_ctx *ctx,
	struct scatterlist *src, struct scatterlist *dst, uint32_t proc_len, int crypt_mode)
{
	int ret;

	ctx->dma_src = src;
	ctx->dma_dst = dst;
	ctx->src_len = proc_len;
	ctx->dst_len = proc_len;

	ret = sm4_cmd_setup(ctx, crypt_mode, MEM_DMA_MODE);
	if (ret < 0) {
		pr_err("sm4_cmd_setup failed with:%d\n", ret);
		return ret;
	}

	ret = sm4_cmd_finish(ctx, MEM_DMA_MODE);
	if (ret < 0) {
		pr_err("sm4_cmd_finish failed with:%d\n", ret);
		return ret;
	}

	return ret;
}

int sm4_update_ddt(struct sm4_ctx *ctx, struct scatterlist *src,
	struct scatterlist *dst, uint32_t proc_len, int crypt_mode)
{
	struct ddt_pdu *src_ddt_list = NULL;
	struct ddt_pdu *dst_ddt_list = NULL;
	struct device *dev = get_smx_dev();
	int src_size, dst_size;
	int ret;

	src_size = sg_nents(src);
	dst_size = sg_nents(dst);
	src_ddt_list = sg_to_ddt(dev, src, DMA_TO_DEVICE);
	dst_ddt_list = sg_to_ddt(dev, dst, DMA_FROM_DEVICE);

	ctx->ddt_src = src_ddt_list;
	ctx->src_len = proc_len;
	ctx->ddt_dst = dst_ddt_list;
	ctx->dst_len = proc_len;

	ret = sm4_cmd_setup(ctx, crypt_mode, MEM_DDT_MODE);
	if (ret < 0) {
		pr_err("sm4_cmd_setup failed with:%d\n", ret);
		return ret;
	}

	dma_unmap_sg(dev, src, src_size, DMA_TO_DEVICE);
	dma_unmap_sg(dev, dst, dst_size, DMA_FROM_DEVICE);
	kfree(src_ddt_list);
	kfree(dst_ddt_list);

	ret = sm4_cmd_finish(ctx, MEM_DDT_MODE);
	if (ret < 0) {
		pr_err("sm4_cmd_finish failed with:%d\n", ret);
		return ret;
	}

	return ret;
}

inline int sm4_update(struct sm4_ctx *ctx, struct scatterlist *src,
	struct scatterlist *dst, uint32_t proc_len, int crypt_mode)
{
	if (!check_sg_valid(src) || !check_sg_valid(dst))
		return -EINVAL;

	if (get_smx_ddt_mode())
		return sm4_update_ddt(ctx, src, dst, proc_len, crypt_mode);
	else
		return sm4_update_dma(ctx, src, dst, proc_len, crypt_mode);
}

inline void sm4_free_ctx_res(struct sm4_ctx *ctx)
{
	if (ctx->sram_index >= 0) {
		free_vfx_shmem(ctx->sram_index);
		ctx->sram_index = -1;
	}

	if (ctx->key_index >= 0) {
		smx_clear_sm4_key_iv(ctx->key_index);
		ctx->key_index = -1;
	}

	if (cpu_latency_qos_request_active(&ctx->req))
		cpu_latency_qos_remove_request(&ctx->req);
}
#endif

#if DESC("skcipher API")
int skcipher_sm4_setkey(struct crypto_skcipher *tfm, const u8 *key,
	unsigned int keylen)
{
	struct sm4_ctx *ctx = crypto_skcipher_ctx(tfm);

	return sm4_set_key_iv(ctx, key, NULL);
}

int skcipher_sm4_encrypt(struct skcipher_request *req)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct sm4_ctx *ctx = crypto_skcipher_ctx(tfm);

	if (!req->base.msg_more)
		ctx->msg_flag |= MSG_END;

	if (req->iv && crypto_skcipher_ivsize(tfm) != 0)
		smx_set_sm4_iv(ctx->key_index, req->iv);

	return sm4_update(ctx, req->src, req->dst, req->cryptlen, ENCRYPT_MODE);
}

int skcipher_sm4_decrypt(struct skcipher_request *req)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct sm4_ctx *ctx = crypto_skcipher_ctx(tfm);

	if (!req->base.msg_more)
		ctx->msg_flag |= MSG_END;

	if (req->iv && crypto_skcipher_ivsize(tfm) != 0)
		smx_set_sm4_iv(ctx->key_index, req->iv);

	return sm4_update(ctx, req->src, req->dst, req->cryptlen, DECRYPT_MODE);
}

int skcipher_sm4_ctx_init(struct crypto_skcipher *tfm)
{
	struct sm4_ctx *ctx = crypto_skcipher_ctx(tfm);
	const char *alg_name = crypto_tfm_alg_name(crypto_skcipher_tfm(tfm));
	uint8_t mode;

	if (strcmp(alg_name, "ecb(se-sm4)") == 0)
		mode = SM4_ECB;
	else if (strcmp(alg_name, "cbc(se-sm4)") == 0)
		mode = SM4_CBC;
	else if (strcmp(alg_name, "ctr(se-sm4)") == 0)
		mode = SM4_CTR;
	else if (strcmp(alg_name, "cfb(se-sm4)") == 0)
		mode = SM4_CFB;
	else if (strcmp(alg_name, "ofb(se-sm4)") == 0)
		mode = SM4_OFB;
	else
		return -ENOENT;

	return sm4_init_ctx(ctx, mode);
}

void skcipher_sm4_ctx_uninit(struct crypto_skcipher *tfm)
{
	struct sm4_ctx *ctx = crypto_skcipher_ctx(tfm);

	sm4_free_ctx_res(ctx);
}

static struct skcipher_alg sm4_algs[] = {
	{
		.base.cra_name          = "ecb(se-sm4)",
		.base.cra_driver_name   = "siengine crypto driver",
		.base.cra_priority      = 100,
		.base.cra_blocksize     = SM4_BLOCK_SIZE,
		.base.cra_ctxsize       = sizeof(struct sm4_ctx),
		.base.cra_module        = THIS_MODULE,
		.min_keysize            = SM4_KEY_SIZE,
		.max_keysize            = SM4_KEY_SIZE,
		.setkey                 = skcipher_sm4_setkey,
		.encrypt                = skcipher_sm4_encrypt,
		.decrypt                = skcipher_sm4_decrypt,
		.init                   = skcipher_sm4_ctx_init,
		.exit                   = skcipher_sm4_ctx_uninit,
	},

	{
		.base.cra_name          = "cbc(se-sm4)",
		.base.cra_driver_name   = "siengine crypto driver",
		.base.cra_priority      = 100,
		.base.cra_blocksize     = SM4_BLOCK_SIZE,
		.base.cra_ctxsize       = sizeof(struct sm4_ctx),
		.base.cra_module        = THIS_MODULE,
		.min_keysize            = SM4_KEY_SIZE,
		.max_keysize            = SM4_KEY_SIZE,
		.ivsize                 = SM4_BLOCK_SIZE,
		.setkey                 = skcipher_sm4_setkey,
		.encrypt                = skcipher_sm4_encrypt,
		.decrypt                = skcipher_sm4_decrypt,
		.init                   = skcipher_sm4_ctx_init,
		.exit                   = skcipher_sm4_ctx_uninit,
	},

	{
		.base.cra_name          = "ctr(se-sm4)",
		.base.cra_driver_name   = "siengine crypto driver",
		.base.cra_priority      = 100,
		.base.cra_blocksize     = 1,
		.base.cra_ctxsize       = sizeof(struct sm4_ctx),
		.base.cra_module        = THIS_MODULE,
		.min_keysize            = SM4_KEY_SIZE,
		.max_keysize            = SM4_KEY_SIZE,
		.ivsize                 = SM4_BLOCK_SIZE,
		.setkey                 = skcipher_sm4_setkey,
		.encrypt                = skcipher_sm4_encrypt,
		.decrypt                = skcipher_sm4_decrypt,
		.init                   = skcipher_sm4_ctx_init,
		.exit                   = skcipher_sm4_ctx_uninit,
	},

	{
		.base.cra_name          = "cfb(se-sm4)",
		.base.cra_driver_name   = "siengine crypto driver",
		.base.cra_priority      = 100,
		.base.cra_blocksize     = 1,
		.base.cra_ctxsize       = sizeof(struct sm4_ctx),
		.base.cra_module        = THIS_MODULE,
		.min_keysize            = SM4_KEY_SIZE,
		.max_keysize            = SM4_KEY_SIZE,
		.ivsize                 = SM4_BLOCK_SIZE,
		.setkey                 = skcipher_sm4_setkey,
		.encrypt                = skcipher_sm4_encrypt,
		.decrypt                = skcipher_sm4_decrypt,
		.init                   = skcipher_sm4_ctx_init,
		.exit                   = skcipher_sm4_ctx_uninit,
	},

	{
		.base.cra_name          = "ofb(se-sm4)",
		.base.cra_driver_name   = "siengine crypto driver",
		.base.cra_priority      = 100,
		.base.cra_blocksize     = SM4_BLOCK_SIZE,
		.base.cra_ctxsize       = sizeof(struct sm4_ctx),
		.base.cra_module        = THIS_MODULE,
		.min_keysize            = SM4_KEY_SIZE,
		.max_keysize            = SM4_KEY_SIZE,
		.ivsize                 = SM4_BLOCK_SIZE,
		.setkey                 = skcipher_sm4_setkey,
		.encrypt                = skcipher_sm4_encrypt,
		.decrypt                = skcipher_sm4_decrypt,
		.init                   = skcipher_sm4_ctx_init,
		.exit                   = skcipher_sm4_ctx_uninit,
	},
};
#endif

#if DESC("AEAD API")
int aead_sm4_ctx_init(struct crypto_aead *tfm)
{
	struct sm4_ctx *ctx = crypto_aead_ctx(tfm);

	ctx->auth_size = crypto_aead_maxauthsize(tfm);
	return sm4_init_ctx(ctx, SM4_GCM);
}

void aead_sm4_ctx_uninit(struct crypto_aead *tfm)
{
	struct sm4_ctx *ctx = crypto_aead_ctx(tfm);

	sm4_free_ctx_res(ctx);
}

int aead_sm4_setkey(struct crypto_aead *tfm, const u8 *key,
					unsigned int keylen)
{
	struct sm4_ctx *ctx = crypto_aead_ctx(tfm);

	return sm4_set_key_iv(ctx, key, NULL);
}

int aead_sm4_setauthsize(struct crypto_aead *tfm, unsigned int authsize)
{
	struct sm4_ctx *ctx = crypto_aead_ctx(tfm);

	if (crypto_gcm_check_authsize(authsize) < 0)
		return -EINVAL;

	ctx->auth_size = authsize;
	return 0;
}

/*
 * The memory structure for cipher operation has the following structure:
 * AEAD encryption input: assoc data || plaintext
 * AEAD encryption output: assoc data || cipherntext || auth tag
 * AEAD decryption input: assoc data || ciphertext || auth tag
 * AEAD decryption output: assoc data || plaintext
 */
int aead_sm4_encrypt(struct aead_request *req)
{
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct sm4_ctx *ctx = crypto_aead_ctx(tfm);
	uint8_t iv[16] = {0};

	ctx->aad_len = req->assoclen;
	if (req->iv && crypto_aead_ivsize(tfm) != 0) {
		/* IV size==96bit, set IV||00000...|| 1 to IPS */
		memcpy(iv, req->iv, crypto_aead_ivsize(tfm));
		iv[15] = 0x01;
		smx_set_sm4_iv(ctx->key_index, iv);
	}

	return sm4_update(ctx, req->src, req->dst, req->cryptlen + req->assoclen, ENCRYPT_MODE);
}

int aead_sm4_decrypt(struct aead_request *req)
{
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct sm4_ctx *ctx = crypto_aead_ctx(tfm);
	uint8_t iv[16] = {0};

	ctx->aad_len = req->assoclen;
	if (req->iv && crypto_aead_ivsize(tfm) != 0) {
		/* IV size==96bit, set IV||00000...|| 1 to IPS */
		memcpy(iv, req->iv, crypto_aead_ivsize(tfm));
		iv[15] = 0x01;
		smx_set_sm4_iv(ctx->key_index, iv);
	}

	return sm4_update(ctx, req->src, req->dst, req->cryptlen + req->assoclen, DECRYPT_MODE);
}

static struct aead_alg aead_algs[] = {
	{
		.base.cra_name          = "se-sm4_gcm",
		.base.cra_driver_name   = "siengine crypto driver",
		.base.cra_priority      = 100,
		.base.cra_blocksize     = 1,
		.base.cra_ctxsize       = sizeof(struct sm4_ctx),
		.base.cra_module        = THIS_MODULE,
		.ivsize                 = GCM_AES_IV_SIZE,
		.maxauthsize            = 16,
		.setkey                 = aead_sm4_setkey,
		.setauthsize            = aead_sm4_setauthsize,
		.encrypt                = aead_sm4_encrypt,
		.decrypt                = aead_sm4_decrypt,
		.init                   = aead_sm4_ctx_init,
		.exit                   = aead_sm4_ctx_uninit,
	},
};
#endif

inline void register_sm4(void)
{
	crypto_register_skciphers(sm4_algs, ARRAY_SIZE(sm4_algs));
	crypto_register_aeads(aead_algs, ARRAY_SIZE(aead_algs));
}

inline void unregister_sm4(void)
{
	crypto_unregister_skciphers(sm4_algs, ARRAY_SIZE(sm4_algs));
	crypto_unregister_aeads(aead_algs, ARRAY_SIZE(aead_algs));
}
