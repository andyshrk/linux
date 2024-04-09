// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains the internal resource management operations of the driver.
 *
 */

#ifndef __CRYPTO_RES_MGR_H__
#define __CRYPTO_RES_MGR_H__

#include <linux/semaphore.h>

#include "ips_reg.h"
#include "crypto_util.h"
#include "crypto_ips_internal.h"
#include "crypto_smx_internal.h"

#define SET_BIT(flag, i)    (flag |= (1UL << (i)))
#define CLR_BIT(flag, i)    (flag &= ~(1UL << (i)))
#define GET_BIT(flag, i)    ((flag >> (i)) & 1UL)

#define DRAM_MEM_SIZE       0x4000
#define DRAM_MEM_COUNT      0x20

extern uint64_t g_shmem_phys_addr;
#define VFx_SHMEM_BASE_ADDR g_shmem_phys_addr

#define VFx_SHMEM_SIZE           0x00140000
#define IPS_VFx_CMD_OFFSET     offsetof(struct crypto_shmem, cmd)
#define IPS_VFx_STATUS_OFFSET  offsetof(struct crypto_shmem, status)
#define SMX_VFx_CMD_OFFSET     offsetof(struct crypto_shmem, smx_cmd)
#define SMX_VFx_STATUS_OFFSET  offsetof(struct crypto_shmem, smx_status)

#define VFx_CMD_COUNT      0x20
#define VFx_STATUS_COUNT   0x20

struct crypto_shmem {
	struct ips_command cmd[VFx_CMD_COUNT];          /* 0x000000 - 0x000800 */
	struct ips_status status[VFx_STATUS_COUNT];     /* 0x000800 - 0x000900 */
	struct smx_command smx_cmd[VFx_CMD_COUNT];      /* 0x000900 - 0x001100 */
	struct smx_status smx_status[VFx_STATUS_COUNT]; /* 0x001100 - 0x001200 */
	uint8_t reserv1[0xEE00];                        /* 0x001200 - 0x010000 */
	uint8_t src[DRAM_MEM_COUNT][DRAM_MEM_SIZE];     /* 0x010000 - 0x090000 */
	uint8_t dst[DRAM_MEM_COUNT][DRAM_MEM_SIZE];     /* 0x090000 - 0x110000 */
	uint8_t reserv2[0x030000];                      /* 0x110000 - 0x140000 */
};

extern struct crypto_shmem *g_vfx_shmem;
#define SHMEM_IPS_CMD(cmd_idx)	(&(g_vfx_shmem->cmd[cmd_idx]))
#define SHMEM_IPS_STS(cmd_idx)	(&(g_vfx_shmem->status[cmd_idx]))
#define SHMEM_SMX_CMD(cmd_idx)	(&(g_vfx_shmem->smx_cmd[cmd_idx]))
#define SHMEM_SMX_STS(cmd_idx)	(&(g_vfx_shmem->smx_status[cmd_idx]))
#define SHMEM_SRC(dram_index)	g_vfx_shmem->src[dram_index]
#define SHMEM_DST(dram_index)	g_vfx_shmem->dst[dram_index]
#define SHMEM_VIRT_TO_PHYS(vaddr)	((vaddr) - (uint8_t *)g_vfx_shmem + VFx_SHMEM_BASE_ADDR)
#define SHMEM_SRC_PHYS(dram_index)	SHMEM_VIRT_TO_PHYS(SHMEM_SRC(dram_index))
#define SHMEM_DST_PHYS(dram_index)	SHMEM_VIRT_TO_PHYS(SHMEM_DST(dram_index))

struct res_flag {
	uint64_t flag;          /* allocate bitmap */
	spinlock_t lock;        /* lock flag read/write */
	uint8_t max_size;       /* resource max size */
	struct semaphore sem;   /* allocate semaphore */
};

inline int8_t alloc_vfx_shmem(void);
inline void free_vfx_shmem(int8_t index);

inline int8_t alloc_aes_keybuffer(void);
inline void free_aes_keybuffer(uint8_t index);

inline int8_t alloc_hmac_keybuffer(void);
inline void free_hmac_keybuffer(uint8_t index);

inline int8_t alloc_ipsec_keybuffer(void);
inline void free_ipsec_keybuffer(uint8_t index);

inline int alloc_cmd_desc(void);
inline void free_cmd_desc(void);

inline int8_t alloc_sm4_keybuffer(void);
inline void free_sm4_keybuffer(int8_t index);
inline int alloc_smx_cmd(void);
inline void free_smx_cmd(void);

void crypto_ips_res_init(phys_addr_t shmem_start, size_t size);
void crypto_smx_res_init(phys_addr_t shmem_start, size_t size);
#endif
