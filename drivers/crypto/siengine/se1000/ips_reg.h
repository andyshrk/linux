// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains IPS register definitions.
 */

#ifndef __CRYPTO_IPS_REG_H__
#define __CRYPTO_IPS_REG_H__

#include <linux/types.h>

extern uint8_t *sfc_ips_virt_base;
extern void *vfx_regs[4];
extern void *vfx_keys[4];
extern struct crypto_shmem *g_vfx_shmem;
extern uint32_t g_use_vf;

#define SFC_IPS_PHY_BASE_ADDR   0xe3800000
#define SFC_IPS_SIZE            0x00400000
#define SFC_IPS_BASE            (sfc_ips_virt_base)

/* address mapping */
#define IPS_XPU         (SFC_IPS_BASE + (0x00000000))
#define IPS_SMMU        (SFC_IPS_BASE + (0x00800000))
#define IPS_VF0_REG     (SFC_IPS_BASE + (0x00130000))
#define IPS_VF0_KEY     (SFC_IPS_BASE + (0x00140000))
#define IPS_VF1_REG     (SFC_IPS_BASE + (0x00150000))
#define IPS_VF1_KEY     (SFC_IPS_BASE + (0x00160000))
#define IPS_VF2_REG     (SFC_IPS_BASE + (0x00170000))
#define IPS_VF2_KEY     (SFC_IPS_BASE + (0x00180000))
#define IPS_VF3_REG     (SFC_IPS_BASE + (0x00190000))
#define IPS_VF3_KEY     (SFC_IPS_BASE + (0x001a0000))
#define IPS_GLB         (SFC_IPS_BASE + (0x001b0000))
#define IPS_MSI_CFG     (SFC_IPS_BASE + (0x001c0000))
#define IPS_MSI_CTRL    (SFC_IPS_BASE + (0x001d0000))
#define IPS_TOP_CTRL    (SFC_IPS_BASE + (0x00230000))
#define IPS_NS_IRQ      (SFC_IPS_BASE + (0x00250000))
#define IPS_SEC_IRQ     (SFC_IPS_BASE + (0x00260000))
#define IPS_MSA_IRQ     (SFC_IPS_BASE + (0x00270000))
#define IPS_DBG_CTRL    (SFC_IPS_BASE + (0x00280000))
#define IPS_ID          (SFC_IPS_BASE + (0x00320000))

#define IPS_VFx_REG_BASE    (vfx_regs[g_use_vf])
#define IPS_VFx_KEY_BASE    (vfx_keys[g_use_vf])

/* Register address for VF0 */
#define IPS_VFx_REG_IRQ_EN          (IPS_VFx_REG_BASE + 0x0)
#define IPS_VFx_REG_IRQ_RAW         (IPS_VFx_REG_BASE + 0x4)
#define IPS_VFx_REG_IRQ_STAT        (IPS_VFx_REG_BASE + 0x8)
#define IPS_VFx_REG_IRQ_CLR         (IPS_VFx_REG_BASE + 0xc)
#define IPS_VFx_REG_IRQ_CTRL_0      (IPS_VFx_REG_BASE + 0x10)
#define IPS_VFx_REG_IRQ_CTRL_1      (IPS_VFx_REG_BASE + 0x14)
#define IPS_VFx_WD_CTRL_0           (IPS_VFx_REG_BASE + 0x18)
#define IPS_VFx_WD_CTRL_1           (IPS_VFx_REG_BASE + 0x1c)
#define IPS_VFx_C0_IRQ_INFO_1       (IPS_VFx_REG_BASE + 0x24)
#define IPS_VFx_C1_IRQ_INFO_1       (IPS_VFx_REG_BASE + 0x2c)
#define IPS_VFx_REG_RING_CTRL_0     (IPS_VFx_REG_BASE + 0x34)
#define IPS_VFx_REG_RING_CTRL_1     (IPS_VFx_REG_BASE + 0x38)
#define IPS_VFx_REG_RING_CTRL_2     (IPS_VFx_REG_BASE + 0x3c)
#define IPS_VFx_REG_RING_CTRL_3     (IPS_VFx_REG_BASE + 0x40)
#define IPS_VFx_REG_RING_CTRL_4     (IPS_VFx_REG_BASE + 0x44)
#define IPS_VFx_REG_RING_CTRL_5     (IPS_VFx_REG_BASE + 0x48)
#define IPS_VFx_REG_RING_CTRL_6     (IPS_VFx_REG_BASE + 0x4c)
#define IPS_VFx_REG_RING_CTRL_7     (IPS_VFx_REG_BASE + 0x50)
#define IPS_VFx_REG_RING_CTRL_8     (IPS_VFx_REG_BASE + 0x54)
#define IPS_VFx_REG_RING_CTRL_9     (IPS_VFx_REG_BASE + 0x58)
#define IPS_VFx_REG_C0_MAILBOX_0    (IPS_VFx_REG_BASE + 0x5c)
#define IPS_VFx_REG_C0_MAILBOX_1    (IPS_VFx_REG_BASE + 0X60)
#define IPS_VFx_REG_C0_MAILBOX_2    (IPS_VFx_REG_BASE + 0x64)
#define IPS_VFx_REG_C0_MAILBOX_3    (IPS_VFx_REG_BASE + 0x68)
#define IPS_VFx_REG_C1_MAILBOX_0    (IPS_VFx_REG_BASE + 0x6c)
#define IPS_VFx_REG_C1_MAILBOX_1    (IPS_VFx_REG_BASE + 0X70)
#define IPS_VFx_REG_C1_MAILBOX_2    (IPS_VFx_REG_BASE + 0x74)
#define IPS_VFx_REG_C1_MAILBOX_3    (IPS_VFx_REG_BASE + 0x78)
#define IPS_VFx_REG_C0_RING_STS_0   (IPS_VFx_REG_BASE + 0x7c)
#define IPS_VFx_REG_C0_RING_STS_1   (IPS_VFx_REG_BASE + 0x80)
#define IPS_VFx_REG_C1_RING_STS_0   (IPS_VFx_REG_BASE + 0x84)
#define IPS_VFx_REG_C1_RING_STS_1   (IPS_VFx_REG_BASE + 0x88)
#define IPS_VFx_REG_RING_INIT_0     (IPS_VFx_REG_BASE + 0x8c)
#define IPS_VFx_REG_RING_INIT_1     (IPS_VFx_REG_BASE + 0x90)
#define IPS_VFx_REG_RING_INIT_2     (IPS_VFx_REG_BASE + 0x94)
#define IPS_VFx_REG_RING_INIT_3     (IPS_VFx_REG_BASE + 0x98)
#define IPS_VFx_REG_CH_PRIOR        (IPS_VFx_REG_BASE + 0xa4)
#define IPS_VFx_REG_LOCAL_MSI_EN    (IPS_VFx_REG_BASE + 0xa8)
#define IPS_VFx_REG_LOCAL_MSI_STAT  (IPS_VFx_REG_BASE + 0xac)
#define IPS_VFx_REG_SMX_SPARE       (IPS_VFx_REG_BASE + 0xb0)
#define IPS_VFx_REG_VF_ALLOC        (IPS_VFx_REG_BASE + 0x200)
#define IPS_VFx_REG_VF_FREE         (IPS_VFx_REG_BASE + 0x204)

/* Key buffer address */
#define IPS_VFx_KEYx(i)         (IPS_VFx_KEY_BASE + 0X00 + 0x30 * (i))
#define IPS_VFx_IVx(i)          (IPS_VFx_KEY_BASE + 0X20 + 0x30 * (i))
#define IPS_VFx_HASHKEYx(i)     (IPS_VFx_KEY_BASE + 0x1000 + 0x80 * (i))
#define IPS_VFx_KEY_BUFFER_NUM  8

/* Register address for Global */
#define IPS_VF_REG_WEIGHT_0 (IPS_GLB + 0x0)
#define IPS_VF_REG_WEIGHT_1 (IPS_GLB + 0x4)
#define IPS_VF_REG_DMA_CTRL (IPS_GLB + 0xc)
#define IPS_VF_REG_GLB_OPT (IPS_GLB + 0x10)

/* Register address for Debug */
#define IPS_DBG_CTRL_DEBUG_BUS_SEL (IPS_DBG_CTRL + 0x0)
#define IPS_DBG_CTRL_DEBUG_RST (IPS_DBG_CTRL + 0x4)

/* Register address for Top */
#define IPS_TOP_CTRL_CORE_CLK_CTRL (IPS_TOP_CTRL + 0x0)
#define IPS_TOP_CTRL_CORE_RST_CTRL (IPS_TOP_CTRL + 0x4)

/* Register address fro MSI_CFG */
#define IPS_VF_REG_MSI_ADDR_0 (IPS_MSI_CFG + 0x0)
#define IPS_VF_REG_MSI_ADDR_1 (IPS_MSI_CFG + 0x4)
#define IPS_VF_REG_MSI_INT_ID_BASE (IPS_MSI_CFG + 0x8)
#define IPS_VF_REG_MSI_RID_GLB (IPS_MSI_CFG + 0xc)
#define IPS_VF_REG_MSI_RID_LOCAL (IPS_MSI_CFG + 0x10)

/* Register address for MSI_CTRL */
#define IPS_VF_REG_GLB_MSI_OVERRIDE (IPS_MSI_CTRL + 0x0)
#define IPS_VF_REG_GLB_MSI_EN (IPS_MSI_CTRL + 0x4)
#define IPS_VF_REG_GLB_MSI_STAT (IPS_MSI_CTRL + 0x8)

/* CIPH_ALG */
#define CIPH_ALG_NULL       0
#define CIPH_ALG_AES_128    1
#define CIPH_ALG_AES_192    2
#define CIPH_ALG_AES_256    3

/* HASH_ALG */
#define HASH_ALG_NULL           0
#define HASH_ALG_SHA_1          1
#define HASH_ALG_SHA_224        2
#define HASH_ALG_SHA_256        3
#define HASH_ALG_SHA_384        4
#define HASH_ALG_SHA_512        5
#define HASH_ALG_SHA_512_224    6
#define HASH_ALG_SHA_512_256    7

/* CIPH_MODE */
#define CIPH_MODE_ECB   0
#define CIPH_MODE_CBC   1
#define CIPH_MODE_CTR   2
#define CIPH_MODE_OFB   3
#define CIPH_MODE_CFB   4
#define CIPH_MODE_GCM   5

/* HASH_MODE */
#define HASH_MODE_RAW   0
#define HASH_MODE_HMAC  1

/* ENCRYPT */
#define ENCRYPT_MODE 1
#define DECRYPT_MODE 0

/* CBC_CS_SEL */
#define CBC_CS_NONE 0
#define CBC_CS1     1
#define CBC_CS2     2
#define CBC_CS3     3

/* memory access  mode */
#define MEM_DDT_MODE 1
#define MEM_DMA_MODE 0

#endif
