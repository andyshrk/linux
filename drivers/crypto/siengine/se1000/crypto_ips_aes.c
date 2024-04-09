// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains the implementation of AES algorithm related operations.
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <crypto/gcm.h>
#include <crypto/if_alg.h>

#include "crypto_ips_aes.h"
#include "crypto_ips_init.h"
#include "crypto_ips_internal.h"
#include "crypto_res_mgr.h"
#include "ips_irq.h"

#if DESC("internal API")
int ips_aes_init_ctx(struct ips_aes_ctx *ctx, uint8_t ciph_mode)
{
	if (!ctx) {
		return -EINVAL;
	}
	if (!cpu_latency_qos_request_active(&ctx->req))
		cpu_latency_qos_add_request(&ctx->req, 0);

	ctx->cipher_mode = ciph_mode;
	ctx->cbc_cs_sel = 0;
	ctx->key_index = -1;
	ctx->dram_index = -1;
	memset(ctx->iv, 0, AES_BLOCK_SIZE);
	ctx->more = false;

	ips_irq_init();
	return 0;
}

static inline int get_aes_alg(uint32_t key_len)
{
	switch (key_len) {
	case AES_KEYSIZE_128:
		return CIPH_ALG_AES_128;
	case AES_KEYSIZE_192:
		return CIPH_ALG_AES_192;
	case AES_KEYSIZE_256:
		return CIPH_ALG_AES_256;
	default:
		return -EINVAL;
	}
}

int ips_aes_set_key_iv(struct ips_aes_ctx *ctx, const uint8_t *key, uint32_t key_len,
	const uint8_t *iv)
{
	int alg;
	int8_t key_index;

	if (ctx == NULL || key == NULL) {
		return -EINVAL;
	}

	alg = get_aes_alg(key_len);
	if (alg < 0)
		return -EINVAL;
	ctx->cipher_alg = alg;

	key_index = alloc_aes_keybuffer();
	if (key_index < 0) {
		pr_err("alloc aes keybuffer fail %d.\n", key_index);
		return key_index;
	}

	ips_set_aes_key(key_index, key, key_len);
	if (iv != NULL) {
		memcpy(ctx->iv, iv, AES_BLOCK_SIZE);
		ips_set_aes_iv(key_index, iv);
	}
	ctx->key_index = key_index;

	return 0;
}

static void get_last_block(struct scatterlist *src, uint32_t len, uint8_t *block)
{
	uint32_t offset = len - AES_BLOCK_SIZE;
	struct scatterlist *sg = NULL;
	uint32_t count = 0;
	int32_t copy = 0;
	int i;

	for_each_sg(src, sg, sg_nents(src), i) {
		copy = count + sg->length - offset;
		if (copy > 0) {
			copy = copy > AES_BLOCK_SIZE ? AES_BLOCK_SIZE : copy;
			memcpy(block, sg_virt(sg) + sg->length - copy, copy);

			if (copy < AES_BLOCK_SIZE)
				memcpy(block + copy, sg_virt(sg_next(sg)), AES_BLOCK_SIZE - copy);
			break;
		}
		count += sg->length;
	}
}

void ips_set_aes_next_iv(struct ips_aes_ctx *ctx, struct scatterlist *src,
	struct scatterlist *dst, uint32_t proc_len)
{
	uint8_t iv[AES_BLOCK_SIZE] = {0};
	uint8_t src_last[AES_BLOCK_SIZE] = {0};
	uint8_t dst_last[AES_BLOCK_SIZE] = {0};
	int i;

	if (ctx->cipher_mode == CIPH_MODE_ECB)
		return;

	if (ctx->cipher_mode == CIPH_MODE_CBC || ctx->cipher_mode == CIPH_MODE_CFB) {
		get_last_block(dst, proc_len, dst_last);
		ips_set_aes_iv(ctx->key_index, dst_last);
	}

	if (ctx->cipher_mode == CIPH_MODE_CTR) {
		memcpy(iv, ctx->iv, sizeof(iv));
		for (i = 0; i < ctx->src_len / AES_BLOCK_SIZE; i++) {
			crypto_inc(iv, AES_BLOCK_SIZE);
		}
		ips_set_aes_iv(ctx->key_index, iv);
	}

	if (ctx->cipher_mode == CIPH_MODE_OFB) {
		get_last_block(src, proc_len, src_last);
		get_last_block(dst, proc_len, dst_last);
		crypto_xor_cpy(iv, dst_last, src_last, AES_BLOCK_SIZE);
		ips_set_aes_iv(ctx->key_index, iv);
	}
}

int ips_aes_update_dma(struct ips_aes_ctx *ctx,
	struct scatterlist *src, struct scatterlist *dst, uint32_t proc_len, int crypt_mode)
{
	int ret;

	ctx->dma_src = src;
	ctx->dma_dst = dst;
	ctx->src_len = proc_len;
	ctx->dst_len = proc_len;

	ret = ips_aes_cmd_setup(ctx, crypt_mode, MEM_DMA_MODE);
	if (ret < 0) {
		pr_err("ips_aes_cmd_setup failed with:%d\n", ret);
		return ret;
	}

	ret = ips_aes_cmd_finish(ctx, MEM_DMA_MODE);
	if (ret < 0) {
		pr_err("ips_aes_cmd_finish failed with:%d\n", ret);
		return ret;
	}

	/* In encrypted mode, software update IV is required to achieve segmented encryption */
	if (ret == 0 && crypt_mode == ENCRYPT_MODE && ctx->more)
		ips_set_aes_next_iv(ctx, src, dst, proc_len);

	return ret;
}

int ips_aes_update_ddt(struct ips_aes_ctx *ctx,
	struct scatterlist *src, struct scatterlist *dst, uint32_t proc_len, int crypt_mode)
{
	struct ddt_pdu *src_ddt_list = NULL;
	struct ddt_pdu *dst_ddt_list = NULL;
	struct device *dev = get_ips_dev();
	int src_size, dst_size;
	int ret;

	src_size = sg_nents(src);
	dst_size = sg_nents(dst);
	src_ddt_list = sg_to_ddt(dev, src, DMA_TO_DEVICE);
	dst_ddt_list = sg_to_ddt(dev, dst, DMA_FROM_DEVICE);
	dma_unmap_sg(dev, src, src_size, DMA_TO_DEVICE);

	ctx->ddt_src = src_ddt_list;
	ctx->src_len = proc_len;
	ctx->ddt_dst = dst_ddt_list;
	ctx->dst_len = proc_len;

	ret = ips_aes_cmd_setup(ctx, crypt_mode, MEM_DDT_MODE);
	kfree(src_ddt_list);
	kfree(dst_ddt_list);
	if (ret < 0) {
		pr_err("ips_aes_cmd_setup failed with:%d\n", ret);
		return ret;
	}

	ret = ips_aes_cmd_finish(ctx, MEM_DDT_MODE);
	if (ret < 0) {
		pr_err("ips_aes_cmd_finish failed with:%d\n", ret);
		return ret;
	}
	dma_unmap_sg(dev, dst, dst_size, DMA_FROM_DEVICE);

	/* In encrypted mode, software update IV is required to achieve segmented encryption */
	if (ret == 0 && crypt_mode == ENCRYPT_MODE && ctx->more)
		ips_set_aes_next_iv(ctx, src, dst, proc_len);

	return ret;
}

inline int ips_aes_update(struct ips_aes_ctx *ctx,
	struct scatterlist *src, struct scatterlist *dst, uint32_t proc_len, int crypt_mode)
{
	if (!check_sg_valid(src) || !check_sg_valid(dst)) {
		pr_err("ips aes invalid input.\n");
		return -EINVAL;
	}

	if (get_ddt_mode())
		return ips_aes_update_ddt(ctx, src, dst, proc_len, crypt_mode);
	else
		return ips_aes_update_dma(ctx, src, dst, proc_len, crypt_mode);
}

inline void ips_aes_free_ctx_res(struct ips_aes_ctx *ctx)
{
	ips_irq_uninit();

	if (ctx->dram_index >= 0) {
		free_vfx_shmem(ctx->dram_index);
		ctx->dram_index = -1;
	}

	if (ctx->key_index >= 0) {
		ips_clear_aes_key_iv(ctx->key_index);
		ctx->key_index = -1;
	}

	if (cpu_latency_qos_request_active(&ctx->req))
		cpu_latency_qos_remove_request(&ctx->req);
}
#endif

#if DESC("skcipher API")
int skcipher_aes_setkey(struct crypto_skcipher *tfm, const u8 *key,
						unsigned int keylen)
{
	struct ips_aes_ctx *ctx = crypto_skcipher_ctx(tfm);
	return ips_aes_set_key_iv(ctx, key, keylen, NULL);
}

int skcipher_aes_encrypt(struct skcipher_request *req)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct ips_aes_ctx *ctx = crypto_skcipher_ctx(tfm);
	int err;

	ctx->more = req->base.msg_more;
	if (req->iv && crypto_skcipher_ivsize(tfm) != 0) {
		memcpy(ctx->iv, req->iv, AES_BLOCK_SIZE);
		ips_set_aes_iv(ctx->key_index, ctx->iv);
	}

	err = ips_aes_update(ctx, req->src, req->dst, req->cryptlen, ENCRYPT_MODE);
	return err;
}

int skcipher_aes_decrypt(struct skcipher_request *req)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct ips_aes_ctx *ctx = crypto_skcipher_ctx(tfm);
	int err;

	ctx->more = req->base.msg_more;
	if (req->iv && crypto_skcipher_ivsize(tfm) != 0) {
		memcpy(ctx->iv, req->iv, AES_BLOCK_SIZE);
		ips_set_aes_iv(ctx->key_index, ctx->iv);
	}

	err = ips_aes_update(ctx, req->src, req->dst, req->cryptlen, DECRYPT_MODE);
	return err;
}

int skcipher_aes_ctx_init(struct crypto_skcipher *tfm)
{
	struct ips_aes_ctx *ctx = crypto_skcipher_ctx(tfm);
	const char *alg_name = crypto_tfm_alg_name(crypto_skcipher_tfm(tfm));
	uint8_t mode;

	if (strcmp(alg_name, "ecb(se-aes)") == 0)
		mode = CIPH_MODE_ECB;
	else if (strcmp(alg_name, "cbc(se-aes)") == 0)
		mode = CIPH_MODE_CBC;
	else if (strcmp(alg_name, "ctr(se-aes)") == 0)
		mode = CIPH_MODE_CTR;
	else if (strcmp(alg_name, "cfb(se-aes)") == 0)
		mode = CIPH_MODE_CFB;
	else if (strcmp(alg_name, "ofb(se-aes)") == 0)
		mode = CIPH_MODE_OFB;
	else
		return -ENOENT;

	return ips_aes_init_ctx(ctx, mode);
}

void skcipher_aes_ctx_uninit(struct crypto_skcipher *tfm)
{
	struct ips_aes_ctx *ctx = crypto_skcipher_ctx(tfm);
	ips_aes_free_ctx_res(ctx);
}

static struct skcipher_alg aes_algs[] = {
	{
		.base.cra_name          = "ecb(se-aes)",
		.base.cra_driver_name   = "siengine crypto driver",
		.base.cra_priority      = 100,
		.base.cra_blocksize     = AES_BLOCK_SIZE,
		.base.cra_ctxsize       = sizeof(struct ips_aes_ctx),
		.base.cra_module        = THIS_MODULE,
		.min_keysize            = AES_MIN_KEY_SIZE,
		.max_keysize            = AES_MAX_KEY_SIZE,
		.setkey                 = skcipher_aes_setkey,
		.encrypt                = skcipher_aes_encrypt,
		.decrypt                = skcipher_aes_decrypt,
		.init                   = skcipher_aes_ctx_init,
		.exit                   = skcipher_aes_ctx_uninit,
	},

	{
		.base.cra_name          = "cbc(se-aes)",
		.base.cra_driver_name   = "siengine crypto driver",
		.base.cra_priority      = 100,
		.base.cra_blocksize     = AES_BLOCK_SIZE,
		.base.cra_ctxsize       = sizeof(struct ips_aes_ctx),
		.base.cra_module        = THIS_MODULE,
		.min_keysize            = AES_MIN_KEY_SIZE,
		.max_keysize            = AES_MAX_KEY_SIZE,
		.ivsize                 = AES_BLOCK_SIZE,
		.setkey                 = skcipher_aes_setkey,
		.encrypt                = skcipher_aes_encrypt,
		.decrypt                = skcipher_aes_decrypt,
		.init                   = skcipher_aes_ctx_init,
		.exit                   = skcipher_aes_ctx_uninit,
	},

	{
		.base.cra_name          = "ctr(se-aes)",
		.base.cra_driver_name   = "siengine crypto driver",
		.base.cra_priority      = 100,
		.base.cra_blocksize     = 1,
		.base.cra_ctxsize       = sizeof(struct ips_aes_ctx),
		.base.cra_module        = THIS_MODULE,
		.min_keysize            = AES_MIN_KEY_SIZE,
		.max_keysize            = AES_MAX_KEY_SIZE,
		.ivsize                 = AES_BLOCK_SIZE,
		.setkey                 = skcipher_aes_setkey,
		.encrypt                = skcipher_aes_encrypt,
		.decrypt                = skcipher_aes_decrypt,
		.init                   = skcipher_aes_ctx_init,
		.exit                   = skcipher_aes_ctx_uninit,
	},

	{
		.base.cra_name          = "cfb(se-aes)",
		.base.cra_driver_name   = "siengine crypto driver",
		.base.cra_priority      = 100,
		.base.cra_blocksize     = 1,
		.base.cra_ctxsize       = sizeof(struct ips_aes_ctx),
		.base.cra_module        = THIS_MODULE,
		.min_keysize            = AES_MIN_KEY_SIZE,
		.max_keysize            = AES_MAX_KEY_SIZE,
		.ivsize                 = AES_BLOCK_SIZE,
		.setkey                 = skcipher_aes_setkey,
		.encrypt                = skcipher_aes_encrypt,
		.decrypt                = skcipher_aes_decrypt,
		.init                   = skcipher_aes_ctx_init,
		.exit                   = skcipher_aes_ctx_uninit,
	},

	{
		.base.cra_name          = "ofb(se-aes)",
		.base.cra_driver_name   = "siengine crypto driver",
		.base.cra_priority      = 100,
		.base.cra_blocksize     = 1,
		.base.cra_ctxsize       = sizeof(struct ips_aes_ctx),
		.base.cra_module        = THIS_MODULE,
		.min_keysize            = AES_MIN_KEY_SIZE,
		.max_keysize            = AES_MAX_KEY_SIZE,
		.ivsize                 = AES_BLOCK_SIZE,
		.setkey                 = skcipher_aes_setkey,
		.encrypt                = skcipher_aes_encrypt,
		.decrypt                = skcipher_aes_decrypt,
		.init                   = skcipher_aes_ctx_init,
		.exit                   = skcipher_aes_ctx_uninit,
	},
};

#endif

#if DESC("AEAD API")
int aead_aes_ctx_init(struct crypto_aead *tfm)
{
	struct ips_aes_ctx *ctx = crypto_aead_ctx(tfm);

	ctx->auth_size = crypto_aead_maxauthsize(tfm);
	return ips_aes_init_ctx(ctx, CIPH_MODE_GCM);
}

void aead_aes_ctx_uninit(struct crypto_aead *tfm)
{
	struct ips_aes_ctx *ctx = crypto_aead_ctx(tfm);

	ips_aes_free_ctx_res(ctx);
}

int aead_aes_setkey(struct crypto_aead *tfm, const u8 *key,
					unsigned int keylen)
{
	struct ips_aes_ctx *ctx = crypto_aead_ctx(tfm);

	return ips_aes_set_key_iv(ctx, key, keylen, NULL);
}

int aead_aes_setauthsize(struct crypto_aead *tfm, unsigned int authsize)
{
	struct ips_aes_ctx *ctx = crypto_aead_ctx(tfm);

	if (ctx->cipher_mode == CIPH_MODE_GCM &&
		crypto_gcm_check_authsize(authsize) < 0) {
		return -EINVAL;
	}

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
int aead_aes_encrypt(struct aead_request *req)
{
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct ips_aes_ctx *ctx = crypto_aead_ctx(tfm);

	ctx->aad_len = req->assoclen;
	if (req->iv && crypto_aead_ivsize(tfm) != 0) {
		/* IV size==96bit, set IV||00000...|| 1 to IPS */
		memset(ctx->iv, 0, sizeof(ctx->iv));
		memcpy(ctx->iv, req->iv, crypto_aead_ivsize(tfm));
		ctx->iv[AES_BLOCK_SIZE - 1] = 0x01;
		ips_set_aes_iv(ctx->key_index, ctx->iv);
	}

	return ips_aes_update(ctx, req->src, req->dst, req->cryptlen + req->assoclen, ENCRYPT_MODE);
}

int aead_aes_decrypt(struct aead_request *req)
{
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct ips_aes_ctx *ctx = crypto_aead_ctx(tfm);

	ctx->aad_len = req->assoclen;
	if (req->iv && crypto_aead_ivsize(tfm) != 0) {
		/* IV size==96bit, set IV||00000...|| 1 to IPS */
		memset(ctx->iv, 0, sizeof(ctx->iv));
		memcpy(ctx->iv, req->iv, crypto_aead_ivsize(tfm));
		ctx->iv[AES_BLOCK_SIZE - 1] = 0x01;
		ips_set_aes_iv(ctx->key_index, ctx->iv);
	}

	return ips_aes_update(ctx, req->src, req->dst, req->cryptlen + req->assoclen, DECRYPT_MODE);
}

static struct aead_alg aead_algs[] = {
	{
		.base.cra_name          = "se-aes_gcm",
		.base.cra_driver_name   = "siengine crypto driver",
		.base.cra_priority      = 100,
		.base.cra_blocksize     = 1,
		.base.cra_ctxsize       = sizeof(struct ips_aes_ctx),
		.base.cra_module        = THIS_MODULE,
		.ivsize                 = GCM_AES_IV_SIZE,
		.maxauthsize            = 16,
		.setkey                 = aead_aes_setkey,
		.setauthsize            = aead_aes_setauthsize,
		.encrypt                = aead_aes_encrypt,
		.decrypt                = aead_aes_decrypt,
		.init                   = aead_aes_ctx_init,
		.exit                   = aead_aes_ctx_uninit,
	},
};

#endif

inline void register_aes(void)
{
	crypto_register_skciphers(aes_algs, ARRAY_SIZE(aes_algs));
	crypto_register_aeads(aead_algs, ARRAY_SIZE(aead_algs));
}

inline void unregister_aes(void)
{
	crypto_unregister_skciphers(aes_algs, ARRAY_SIZE(aes_algs));
	crypto_unregister_aeads(aead_algs, ARRAY_SIZE(aead_algs));
}
