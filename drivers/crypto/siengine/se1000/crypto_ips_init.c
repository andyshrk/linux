// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains operations related to IPS driver initialization.
 */

#include <linux/kernel.h>
#include <linux/io.h>

#include "ips_reg.h"
#include "ips_irq.h"
#include "crypto_ips_init.h"
#include "crypto_util.h"
#include "crypto_ips_internal.h"
#include "crypto_res_mgr.h"

uint8_t *sfc_ips_virt_base;
void *vfx_regs[4] = {0};
void *vfx_keys[4] = {0};

static int ips_init_clk(void)
{
	int ret_value;

	/* initialzie IPS-CORE clock */
	if (READ_REG(IPS_TOP_CTRL_CORE_CLK_CTRL) != 0x1) {
		WRITE_REG(IPS_TOP_CTRL_CORE_CLK_CTRL, 0x1);
		ret_value = READ_REG(IPS_TOP_CTRL_CORE_CLK_CTRL);
		if (ret_value != 1) {
			pr_err("enable ips core-clock failed");
			return -1;
		}
	}

	/* initialzie clock enalbe for IPS-SRAM clock and IPS-FUNCTION to save power */
	WRITE_REG(IPS_VF_REG_GLB_OPT, 0x0);
	ret_value = READ_REG(IPS_VF_REG_GLB_OPT);
	if (ret_value != 0) {
		pr_err("enable ips ram-clock failed!");
		return -1;
	}

	return 0;
}

/* According to the rules of VF allocation:
 * the Allocate state of the current VF can only be occupied by the current OS,
 * this is not considered as an exception.
 */
static int ips_alloc_vfx(void)
{
	int ret_value;

	/* vf_free */
	ret_value = READ_REG(IPS_VFx_REG_VF_FREE);
	if (ret_value == 1) {
		/* VF ALLOC */
		WRITE_REG(IPS_VFx_REG_VF_ALLOC, 1);
		ret_value = READ_REG(IPS_VFx_REG_VF_ALLOC);
		if (ret_value != 1) {
			pr_err("security attribute not consistent!\n");
			return -1;
		}
	}
	return 0;
}

int ips_reset_vfx(void)
{
	int ret_value;
	int count = 0;

	/* wait for VFx IDLE */
	ret_value = READ_REG(IPS_VFx_REG_RING_INIT_1);
	while ((ret_value != 3) && (count != 100)) {
		ret_value = READ_REG(IPS_VFx_REG_RING_INIT_1);
		count++;
	}
	if (count == 100) {
		pr_err("count overflow!");
		return -1;
	}

	/* disable channel 0 */
	WRITE_REG(IPS_VFx_REG_RING_INIT_0, 0);

	/* clear cmd/status index */
	WRITE_REG(IPS_VFx_REG_RING_INIT_2, 1);
	WRITE_REG(IPS_VFx_REG_RING_INIT_3, 1);
	return 0;
}

int ips_free_vfx(void)
{
	int ret_value;

	/* vf_free */
	ret_value = READ_REG(IPS_VFx_REG_VF_FREE);
	if (ret_value == 1)
		return 0;

	/* disable channel 0 */
	WRITE_REG(IPS_VFx_REG_RING_INIT_0, 0);

	ret_value = ips_reset_vfx();
	if (ret_value != 0) {
		pr_err("ips_reset_vfx failed!\n");
		return ret_value;
	}

	WRITE_REG(IPS_VFx_REG_VF_FREE, 1);
	ret_value = READ_REG(IPS_VFx_REG_VF_FREE);
	if (ret_value != 1) {
		pr_err("security attribute not consistent!\n");
		return -1;
	}

	return 0;
}

static inline void set_vfx_base(void)
{
	if (sfc_ips_virt_base) {
		vfx_regs[0] = IPS_VF0_REG;
		vfx_regs[1] = IPS_VF1_REG;
		vfx_regs[2] = IPS_VF2_REG;
		vfx_regs[3] = IPS_VF3_REG;

		vfx_keys[0] = IPS_VF0_KEY;
		vfx_keys[1] = IPS_VF1_KEY;
		vfx_keys[2] = IPS_VF2_KEY;
		vfx_keys[3] = IPS_VF3_KEY;
	}
}

/* success: return 0, failed: return 1 */
static uint32_t set_vfx_cmd_status_addr(void *high_addr_reg,
										void *low_addr_reg, size_t phys_addr)
{
	uint32_t phys_low_addr, phys_high_addr;
	uint32_t ret_value;

	phys_low_addr = (uint32_t)phys_addr;
	phys_high_addr = (uint32_t)(phys_addr >> 32);

	WRITE_REG(low_addr_reg, phys_low_addr);
	ret_value = READ_REG(low_addr_reg);
	if (ret_value != phys_low_addr) {
		pr_err("set base_low failed");
		return -1;
	}

	WRITE_REG(high_addr_reg, phys_high_addr);
	ret_value = READ_REG(high_addr_reg);
	if (ret_value != phys_high_addr) {
		pr_err("set base_high failed");
		return -1;
	}

	return 0;
}

/* parse dts to do reource init */
int ips_res_init(struct device *dev)
{
	int err;
	u32 ddt;
	struct resource shmem_res;

	err = parse_dts(dev, &sfc_ips_virt_base, &g_use_vf, &ddt, &shmem_res);
	if (err < 0)
		return err;

	set_vfx_base();
	set_ddt_mode(ddt);
	crypto_ips_res_init(shmem_res.start, shmem_res.end - shmem_res.start);
	set_ips_dev(dev);

	return 0;
}

/* ips device initialzie, set baseaddr, enable ioc irq, select channel */
int ips_dev_init(void)
{
	int ret_value = 0;
	int cmd_status_count;
	CRYPTO_IRQ_EN irq_enable = {0};

	do {
		ret_value = ips_init_clk();
		if (ret_value < 0) {
			pr_err("ips_init_clk failed with %d\n", ret_value);
			break;
		}

		ret_value = ips_alloc_vfx();
		if (ret_value < 0) {
			pr_err("ips_alloc_vfx failed with %d\n", ret_value);
			break;
		}

		ret_value = ips_reset_vfx();
		if (ret_value < 0) {
			pr_err("ips_reset_vfx failed with %d\n", ret_value);
			break;
		}

		/*set channel 0 cmd and status base addr*/
		ret_value = set_vfx_cmd_status_addr(IPS_VFx_REG_RING_CTRL_1,
			IPS_VFx_REG_RING_CTRL_0, VFx_SHMEM_BASE_ADDR + IPS_VFx_CMD_OFFSET);
		if (ret_value < 0) {
			ret_value = -1;
			break;
		}

		ret_value = set_vfx_cmd_status_addr(IPS_VFx_REG_RING_CTRL_5,
			IPS_VFx_REG_RING_CTRL_4, VFx_SHMEM_BASE_ADDR + IPS_VFx_STATUS_OFFSET);
		if (ret_value < 0) {
			ret_value = -1;
			break;
		}

		/* set cmd/status ring descriptior count */
		cmd_status_count = (VFx_STATUS_COUNT - 1) << 16;
		cmd_status_count |= (VFx_CMD_COUNT - 1);
		WRITE_REG(IPS_VFx_REG_RING_CTRL_8, cmd_status_count);
		ret_value = READ_REG(IPS_VFx_REG_RING_CTRL_8);
		if (ret_value != cmd_status_count) {
			pr_err("set descriptior count failed");
			ret_value = -1;
			break;
		}

		/* enable irq channel 0 */
		irq_enable.IOC_EN_0 = 1;
		irq_enable.STS_ERR_EN_0 = 1;
		irq_enable.GLBL_EN = 1;
		WRITE_REG(IPS_VFx_REG_IRQ_EN, REG_TO_UINT32(irq_enable));
		ret_value = READ_REG(IPS_VFx_REG_IRQ_EN);
		if (ret_value != REG_TO_UINT32(irq_enable)) {
			pr_err("enable irq channel 0 failed");
			ret_value = -1;
			break;
		}

		/* enable channel 0 */
		WRITE_REG(IPS_VFx_REG_RING_INIT_0, 1);
		ret_value = READ_REG(IPS_VFx_REG_RING_INIT_0);
		if (ret_value != 1) {
			pr_err("channel 0 enable failed");
			ret_value = -1;
			break;
		}
	} while (0);

	if (ret_value < 0) {
		ips_free();
		return ret_value;
	}

	return 0;
}

int ips_free(void)
{
	if (sfc_ips_virt_base) {
		ips_free_vfx();
		iounmap(sfc_ips_virt_base);
		sfc_ips_virt_base = NULL;
	}

	if (g_vfx_shmem) {
		iounmap(g_vfx_shmem);
		g_vfx_shmem = NULL;
	}

	return 0;
}
