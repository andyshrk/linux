// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains SMX register definitions.
 */

#include "crypto_smx_init.h"
#include "crypto_smx_internal.h"
#include "crypto_res_mgr.h"
#include "crypto_util.h"
#include "smx_reg.h"

uint8_t *sfc_smx_virt_base;
void *smx_vfx_regs[4];

static int smx_init_clk(void)
{
	int ret_value;

	/* initialzie clock enalbe for SMX-SRAM clock and SMX-FUNCTION to save power */
	ret_value = READ_REG(SMX_VF_REG_GLB_OPT);
	if (ret_value != 0)
		WRITE_REG(SMX_VF_REG_GLB_OPT, 0x0);

	return 0;
}

static int smx_alloc_vfx(void)
{
	int ret_value;

	/* vf_free */
	ret_value = READ_REG(SMX_VFx_REG_VF_FREE);
	if (ret_value == 1) {
		/* VF ALLOC */
		WRITE_REG(SMX_VFx_REG_VF_ALLOC, 1);
		ret_value = READ_REG(SMX_VFx_REG_VF_ALLOC);
		if (ret_value != 1) {
			pr_err("security attribute not consistent!\n");
			return -1;
		}
	}
	return 0;
}

static int smx_reset_vfx(void)
{
	int ret_value;
	int count = 0;

	ret_value = READ_REG(SMX_VFx_REG_RING_INIT_1);
	while ((ret_value != 3) && (count != 100)) {
		ret_value = READ_REG(SMX_VFx_REG_RING_INIT_1);
		count++;
	}
	if (count == 100)
		return -1;

	/* disable channel 0 */
	ret_value = READ_REG(SMX_VFx_REG_RING_INIT_0);
	if (ret_value)
		WRITE_REG(SMX_VFx_REG_RING_INIT_0, 0);

	WRITE_REG(SMX_VFx_REG_RING_INIT_2, 1);
	WRITE_REG(SMX_VFx_REG_RING_INIT_3, 1);

	return 0;
}

int smx_free_vfx(void)
{
	int ret_value;

	/* vf_free */
	ret_value = READ_REG(SMX_VFx_REG_VF_FREE);
	if (ret_value == 1)
		return 0;

	/* disable channel 0 */
	WRITE_REG(SMX_VFx_REG_RING_INIT_0, 0);

	ret_value = smx_reset_vfx();
	if (ret_value != 0) {
		pr_err("ips_reset_vfx failed!\n");
		return ret_value;
	}

	WRITE_REG(SMX_VFx_REG_VF_FREE, 1);
	ret_value = READ_REG(SMX_VFx_REG_VF_FREE);
	if (ret_value != 1) {
		pr_err("security attribute not consistent!\n");
		return -1;
	}

	return 0;
}

static inline void set_vfx_base(void)
{
	if (sfc_smx_virt_base) {
		smx_vfx_regs[0] = SMX_VF0_REG;
		smx_vfx_regs[1] = SMX_VF1_REG;
		smx_vfx_regs[2] = SMX_VF2_REG;
		smx_vfx_regs[3] = SMX_VF3_REG;
	}
}

/* parse dts to do reource init */
int smx_res_init(struct device *dev)
{
	int err;
	u32 ddt;
	struct resource shmem_res;

	err = parse_dts(dev, &sfc_smx_virt_base, &g_smx_use_vf, &ddt, &shmem_res);
	if (err < 0)
		return err;

	set_vfx_base();
	set_smx_ddt_mode(ddt);
	crypto_smx_res_init(shmem_res.start, shmem_res.end - shmem_res.start);
	set_smx_dev(dev);

	return 0;
}

int smx_dev_init(void)
{
	int ret_value;
	uint64_t cmd_base;
	uint64_t status_base;
	uint32_t cmd_status_count;
	CRYPTO_IRQ_EN irq_enable = {0};

	do {
		ret_value = smx_init_clk();
		if (ret_value < 0) {
			pr_err("smx_init_clk failed with %d\n", ret_value);
			break;
		}

		ret_value = smx_alloc_vfx();
		if (ret_value < 0) {
			pr_err("smx_alloc_vfx failed with %d\n", ret_value);
			break;
		}

		ret_value = smx_reset_vfx();
		if (ret_value < 0) {
			pr_err("smx_reset_vfx failed with %d\n", ret_value);
			break;
		}
	} while (0);
	if (ret_value < 0) {
		smx_free();
		return ret_value;
	}

	/* set cmd and status base addr */
	cmd_base = SMX_VFx_CMD_OFFSET + VFx_SHMEM_BASE_ADDR;
	status_base = SMX_VFx_STATUS_OFFSET + VFx_SHMEM_BASE_ADDR;
	WRITE_REG(SMX_VFx_REG_RING_CTRL_0, (uint32_t)cmd_base);
	WRITE_REG(SMX_VFx_REG_RING_CTRL_1, (uint32_t)(cmd_base >> 32));
	WRITE_REG(SMX_VFx_REG_RING_CTRL_4, (uint32_t)status_base);
	WRITE_REG(SMX_VFx_REG_RING_CTRL_5, (uint32_t)(status_base >> 32));

	/* set cmd/status count */
	cmd_status_count = (VFx_STATUS_COUNT - 1) << 16;
	cmd_status_count |= (VFx_CMD_COUNT - 1);
	WRITE_REG(SMX_VFx_REG_RING_CTRL_8, cmd_status_count);

	/* enable irq channel 0 */
	irq_enable.IOC_EN_0 = 1,
	irq_enable.STS_ERR_EN_0 = 1;
	irq_enable.GLBL_EN = 1;
	WRITE_REG(SMX_VFx_REG_IRQ_EN, REG_TO_UINT32(irq_enable));

	/* enable channel 0 */
	WRITE_REG(SMX_VFx_REG_RING_INIT_0, 1);

	return 0;
}

void smx_free(void)
{
	if (sfc_smx_virt_base) {
		smx_free_vfx();
		iounmap(sfc_smx_virt_base);
		sfc_smx_virt_base = NULL;
	}
}
