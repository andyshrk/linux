// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains SMX register definitions.
 */

#ifndef __CRYPTO_SMX_REG_H__
#define __CRYPTO_SMX_REG_H__

#include <linux/types.h>

extern uint8_t *sfc_smx_virt_base;
extern void *smx_vfx_regs[4];
extern uint32_t g_smx_use_vf;

#define SFC_SMX_PHY_BASE_ADDR	0xe3580000
#define SFC_SMX_SIZE			0x00100000
#define SFC_SMX_BASE			(sfc_smx_virt_base)

/* address mapping */
#define SMX_VF0_REG		(SFC_SMX_BASE + (0x000000))
#define SMX_VF0_KEY		(SFC_SMX_BASE + (0x010000))
#define SMX_VF1_REG		(SFC_SMX_BASE + (0x020000))
#define SMX_VF1_KEY		(SFC_SMX_BASE + (0x030000))
#define SMX_VF2_REG		(SFC_SMX_BASE + (0x040000))
#define SMX_VF2_KEY		(SFC_SMX_BASE + (0x050000))
#define SMX_VF3_REG		(SFC_SMX_BASE + (0x060000))
#define SMX_VF3_KEY		(SFC_SMX_BASE + (0x070000))
#define SMX_GLOBAL		(SFC_SMX_BASE + (0x080000))
#define SMX_MSI_CFG		(SFC_SMX_BASE + (0x090000))
#define SMX_MSI_CTRL	(SFC_SMX_BASE + (0x0a0000))

#define SMX_VFx_REG		(smx_vfx_regs[g_smx_use_vf])
#define SMX_VFx_KEY		(smx_vfx_regs[g_smx_use_vf] + (0x010000))

/* Key buffer address */
#define SMX_VFx_KEYx(i)	(SMX_VFx_KEY + 0X00 + 0x30 * (i))
#define SMX_VFx_IVx(i)	(SMX_VFx_KEY + 0X10 + 0x30 * (i))
#define SMX_VFx_KEY_BUFFER_NUM	8

/* Register address for VF0 */
#define SMX_VFx_REG_IRQ_EN			(SMX_VFx_REG + 0x0)
#define SMX_VFx_REG_IRQ_RAW			(SMX_VFx_REG + 0x4)
#define SMX_VFx_REG_IRQ_STAT		(SMX_VFx_REG + 0x8)
#define SMX_VFx_REG_IRQ_CLR			(SMX_VFx_REG + 0xc)
#define SMX_VFx_REG_IRQ_CTRL_0		(SMX_VFx_REG + 0x10)
#define SMX_VFx_REG_IRQ_CTRL_1		(SMX_VFx_REG + 0x14)
#define SMX_VFx_WD_CTRL_0			(SMX_VFx_REG + 0x18)
#define SMX_VFx_WD_CTRL_1			(SMX_VFx_REG + 0x1c)
#define SMX_VFx_C0_IRQ_INFO_1		(SMX_VFx_REG + 0x24)
#define SMX_VFx_C1_IRQ_INFO_1		(SMX_VFx_REG + 0x2c)
#define SMX_VFx_REG_RING_CTRL_0		(SMX_VFx_REG + 0x34)
#define SMX_VFx_REG_RING_CTRL_1		(SMX_VFx_REG + 0x38)
#define SMX_VFx_REG_RING_CTRL_2		(SMX_VFx_REG + 0x3c)
#define SMX_VFx_REG_RING_CTRL_3		(SMX_VFx_REG + 0x40)
#define SMX_VFx_REG_RING_CTRL_4		(SMX_VFx_REG + 0x44)
#define SMX_VFx_REG_RING_CTRL_5		(SMX_VFx_REG + 0x48)
#define SMX_VFx_REG_RING_CTRL_6		(SMX_VFx_REG + 0x4c)
#define SMX_VFx_REG_RING_CTRL_7		(SMX_VFx_REG + 0x50)
#define SMX_VFx_REG_RING_CTRL_8		(SMX_VFx_REG + 0x54)
#define SMX_VFx_REG_RING_CTRL_9		(SMX_VFx_REG + 0x58)
#define SMX_VFx_REG_C0_MAILBOX_0	(SMX_VFx_REG + 0x5c)
#define SMX_VFx_REG_C0_MAILBOX_1	(SMX_VFx_REG + 0X60)
#define SMX_VFx_REG_C0_MAILBOX_2	(SMX_VFx_REG + 0x64)
#define SMX_VFx_REG_C0_MAILBOX_3	(SMX_VFx_REG + 0x68)
#define SMX_VFx_REG_C1_MAILBOX_0	(SMX_VFx_REG + 0x6c)
#define SMX_VFx_REG_C1_MAILBOX_1	(SMX_VFx_REG + 0X70)
#define SMX_VFx_REG_C1_MAILBOX_2	(SMX_VFx_REG + 0x74)
#define SMX_VFx_REG_C1_MAILBOX_3	(SMX_VFx_REG + 0x78)
#define SMX_VFx_REG_C0_RING_STS_0	(SMX_VFx_REG + 0x7c)
#define SMX_VFx_REG_C0_RING_STS_1	(SMX_VFx_REG + 0x80)
#define SMX_VFx_REG_C1_RING_STS_0	(SMX_VFx_REG + 0x84)
#define SMX_VFx_REG_C1_RING_STS_1	(SMX_VFx_REG + 0x88)
#define SMX_VFx_REG_RING_INIT_0		(SMX_VFx_REG + 0x8c)
#define SMX_VFx_REG_RING_INIT_1		(SMX_VFx_REG + 0x90)
#define SMX_VFx_REG_RING_INIT_2		(SMX_VFx_REG + 0x94)
#define SMX_VFx_REG_RING_INIT_3		(SMX_VFx_REG + 0x98)
#define SMX_VFx_REG_CH_PRIOR		(SMX_VFx_REG + 0xa4)
#define SMX_VFx_REG_LOCAL_MSI_EN	(SMX_VFx_REG + 0xa8)
#define SMX_VFx_REG_LOCAL_MSI_STAT	(SMX_VFx_REG + 0xac)
#define SMX_VFx_REG_SMX_SPARE		(SMX_VFx_REG + 0xb0)
#define SMX_VFx_REG_VF_ALLOC		(SMX_VFx_REG + 0x200)
#define SMX_VFx_REG_VF_FREE			(SMX_VFx_REG + 0x204)

/* Register address for Global */
#define SMX_VF_REG_WEIGHT_0			(SMX_GLOBAL + 0x0)
#define SMX_VF_REG_WEIGHT_1			(SMX_GLOBAL + 0x4)
#define SMX_VF_REG_DMA_CTRL			(SMX_GLOBAL + 0xc)
#define SMX_VF_REG_GLB_OPT			(SMX_GLOBAL + 0x10)

/* Register address fro MSI_CFG */
#define SMX_VF_REG_MSI_ADDR_0		(SMX_MSI_CFG + 0x0)
#define SMX_VF_REG_MSI_ADDR_1		(SMX_MSI_CFG + 0x4)
#define SMX_VF_REG_MSI_INT_ID_BASE	(SMX_MSI_CFG + 0x8)
#define SMX_VF_REG_MSI_RID_GLB		(SMX_MSI_CFG + 0xc)
#define SMX_VF_REG_MSI_RID_LOCAL	(SMX_MSI_CFG + 0x10)

/* Register address for MSI_CTRL */
#define SMX_VF_REG_GLB_MSI_OVERRIDE	(SMX_MSI_CTRL + 0x0)
#define SMX_VF_REG_GLB_MSI_EN		(SMX_MSI_CTRL + 0x4)
#define SMX_VF_REG_GLB_MSI_STAT		(SMX_MSI_CTRL + 0x8)

#define SM3_NULL_MODE	0
#define SM3_RAW_MODE	1

#define	SM4_NULL		0x00
#define SM4_ECB			0x01
#define SM4_CBC			0x02
#define SM4_CTR			0x03
#define SM4_OFB			0x04
#define SM4_CFB			0x05
#define SM4_GCM			0x06

/* memory access  mode */
#define MEM_DDT_MODE 1
#define MEM_DMA_MODE 0

/* ENCRYPT */
#define ENCRYPT_MODE 1
#define DECRYPT_MODE 0

/* CBC_CS_SEL */
#define CBC_CS_NONE 0
#define CBC_CS1     1
#define CBC_CS2     2
#define CBC_CS3     3
#endif