// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains the internal resource management operations of the driver.
 */

#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>

#include "crypto_ips_internal.h"
#include "crypto_res_mgr.h"

uint64_t g_shmem_phys_addr;

static inline void init_res(struct res_flag *res, uint8_t max_size)
{
	res->flag = 0;
	spin_lock_init(&res->lock);
	res->max_size = max_size;
	sema_init(&res->sem, max_size);
}

static int8_t alloc_res(struct res_flag *res)
{
	int8_t index;

	/* lock the resource size, resource not available will sleep */
	if (down_timeout(&res->sem, 5 * HZ) < 0)
		return -ETIME;

	/* lock the flag, avoid multi-access to flag */
	spin_lock(&res->lock);

	/* allocate resource */
	for (index = 0; index < res->max_size; index++) {
		if (GET_BIT(res->flag, index) == 0) {
			SET_BIT(res->flag, index);
			break;
		}
	}
	spin_unlock(&res->lock);
	if (index >= res->max_size)
		return -1;

	return index;
}

static inline void free_res(struct res_flag *res, int8_t index)
{
	if (index < 0)
		return;

	spin_lock(&res->lock);
	CLR_BIT(res->flag, index);
	spin_unlock(&res->lock);
	up(&res->sem);
}

#if DESC("shmem resources")
/* shmem resource flag */
static struct res_flag g_shmem_flag;
struct crypto_shmem *g_vfx_shmem;

inline int8_t alloc_vfx_shmem(void)
{
	return alloc_res(&g_shmem_flag);
}

inline void free_vfx_shmem(int8_t index)
{
	free_res(&g_shmem_flag, index);
}
#endif

#if DESC("IPS resources")
/* VFx key buffer resource flag */
static struct res_flag g_aes_key_buffer;
static struct res_flag g_hash_key_buffer;

static struct semaphore g_cmd_sem;

inline int8_t alloc_aes_keybuffer(void)
{
	return alloc_res(&g_aes_key_buffer);
}

inline void free_aes_keybuffer(uint8_t index)
{
	free_res(&g_aes_key_buffer, index);
}

inline int8_t alloc_hmac_keybuffer(void)
{
	return alloc_res(&g_hash_key_buffer);
}

inline void free_hmac_keybuffer(uint8_t index)
{
	free_res(&g_hash_key_buffer, index);
}

inline int8_t alloc_ipsec_keybuffer(void)
{
	uint8_t index;

	down(&g_aes_key_buffer.sem);
	down(&g_hash_key_buffer.sem);

	spin_lock(&g_aes_key_buffer.lock);
	spin_lock_nested(&g_hash_key_buffer.lock, MAX_LOCKDEP_SUBCLASSES - 1);

	for (index = 0; index < IPS_VFx_KEY_BUFFER_NUM; index++) {
		if (GET_BIT(g_aes_key_buffer.flag, index) == 0 &&
			GET_BIT(g_hash_key_buffer.flag, index) == 0) {

			SET_BIT(g_aes_key_buffer.flag, index);
			SET_BIT(g_hash_key_buffer.flag, index);
			break;
		}
	}
	spin_unlock(&g_hash_key_buffer.lock);
	spin_unlock(&g_aes_key_buffer.lock);

	if (index == IPS_VFx_KEY_BUFFER_NUM) {
		return -EBUSY;
	}

	return index;
}

inline void free_ipsec_keybuffer(uint8_t index)
{
	free_res(&g_aes_key_buffer, index);
	free_res(&g_hash_key_buffer, index);
}

inline int alloc_cmd_desc(void)
{
	return down_timeout(&g_cmd_sem, 5 * HZ);
}

inline void free_cmd_desc(void)
{
	up(&g_cmd_sem);
}
#endif

#if DESC("SMX RESOURCES")
/* VFx key buffer resource flag */
static struct res_flag g_sm4_key_buffer;
static struct semaphore g_smx_cmd_sem;

inline int8_t alloc_sm4_keybuffer(void)
{
	return alloc_res(&g_sm4_key_buffer);
}

inline void free_sm4_keybuffer(int8_t index)
{
	free_res(&g_sm4_key_buffer, index);
}

inline int alloc_smx_cmd(void)
{
	return down_timeout(&g_smx_cmd_sem, 5 * HZ);
}

inline void free_smx_cmd(void)
{
	up(&g_smx_cmd_sem);
}

#endif

static inline void shmem_init(phys_addr_t shmem_start, size_t size)
{
	if (g_vfx_shmem == NULL) {
		g_vfx_shmem = (struct crypto_shmem *)ioremap_wc(shmem_start, size);
		init_res(&g_shmem_flag, DRAM_MEM_COUNT);
		g_shmem_phys_addr = shmem_start;
	}
}

void crypto_ips_res_init(phys_addr_t shmem_start, size_t size)
{
	shmem_init(shmem_start, size);
	init_res(&g_aes_key_buffer, IPS_VFx_KEY_BUFFER_NUM);
	init_res(&g_hash_key_buffer, IPS_VFx_KEY_BUFFER_NUM);
	sema_init(&g_cmd_sem, VFx_CMD_COUNT - 1);
}

void crypto_smx_res_init(phys_addr_t shmem_start, size_t size)
{
	shmem_init(shmem_start, size);
	init_res(&g_sm4_key_buffer, SMX_VFx_KEY_BUFFER_NUM);
	sema_init(&g_smx_cmd_sem, VFx_CMD_COUNT - 1);
}

