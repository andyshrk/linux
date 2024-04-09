// SPDX-License-Identifier: GPL-2.0-only
/*
 * SE1000 clock reset control driver
 *
 * Copyright (C) 2020 siengine
 *
 * Jun Wang <jun.wang@siengine.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#define pr_fmt(fmt)	"clk-se1000: " fmt

#include <linux/kernel.h>
#include <linux/clk-provider.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/log2.h>
#include <linux/reset.h>
#include <linux/reset-controller.h>
#include <linux/syscore_ops.h>
#include "clk-private.h"
#include "dt-bindings/clock/se1000-clock.h"

#define MDP_PLL_BASE_OFFSET  0xE4000
#define MDP_PLL0_CFG4 (MDP_PLL_BASE_OFFSET + 0x10)
#define MDP_PLL0_CFG3 (MDP_PLL_BASE_OFFSET + 0xc)
#define MDP_PLL0_CFG2 (MDP_PLL_BASE_OFFSET + 0x8)
#define MDP_PLL0_CFG1 (MDP_PLL_BASE_OFFSET + 0x4)

/*
 * RESET_YES: deassert the reset signal after enabling clk or pll set
 * OSC_CLK_OUTPUT: sub-clock select 25Mhz osc clock output
 * PLL_CLK_OUTPUT: sub-clock select PLL clock output
 */

enum {
	RESET_NO = 0,
	RESET_YES = 1,
	OSC_CLK_OUTPUT = (1 << 1),
	PLL_CLK_OUTPUT = (1 << 2),
	CLK_HACK = (1 << 3),
	CLK_FIX_EN_DIS = (1 << 4),
};

#define MDP_DPUn_PIXEL_CLK(n)	(n) /* select Pixel clock output of DPU#n  */
#define AUD_I2Sn_PLL_CLK(n)	(n) /* select PLL clock output of AUD#n  */
#define SAF_CLK_EN_MASK(n)	BIT(n) /*select saf clk device mask */

enum FOUTPHASE_ID {
	FOUTPOSTDIV = 1,
	FOUT1PH0 = 2,/* don't change to other number */
	FOUT2 = 4,   /* don't change to other number */
	FOUT3 = 6,   /* don't change to other number */
	FOUT4 = 8,   /* don't change to other number */
};

struct se1000_core_clk {
	u32			id;
	const char		*clk_name;
	const char		*parent_name;
	u32			flag; /* set 1 if need to dessert this clk dev */
	int			clkaddr_offset;
	int			rstaddr_offset;
	int			pll_cfg0;
	int			pll_cfg1;
	u32			pll_cfg2;
	u32			pll_cfg3;
	u32			pll_cfg4;
	u32			pll_cfg5;
	u32			pll_fout4phase;
};

struct se_fixed_rate_clock {
	u32			id;
	char			*clk_name;
	const char		*parent_name;
	unsigned long		flags;
	unsigned long		fixed_rate;
};

struct pll_div_regconfig {
	unsigned int fbdiv;
	unsigned int frac;
	unsigned int postdiv1;
	unsigned int postdiv2;
};

struct mdp_clkout {
	void __iomem *crg_base;
	struct pll_div_regconfig *pll_reg;
	unsigned int pixel_rate;
};

static struct mdp_clkout *mdp_clk;

struct clkrst_core {
	struct clk_hw hw;
	void __iomem  *clk_reg;
	void __iomem  *rst_reg;
	spinlock_t     lock_clk;
	spinlock_t     lock_rst;
	u32            clk_flag;
	u32            pll_default;
	const char    *clk_name;
	void __iomem  *pll_cfg0;
	void __iomem  *pll_cfg1;
	void __iomem  *pll_cfg2;
	void __iomem  *pll_cfg3;
	void __iomem  *pll_cfg4;
	void __iomem  *pll_cfg5;
	u32           pll_fout4phase;
};

struct se1000_crg_data {
	struct clkrst_core		*clkrst_data;
	struct reset_controller_dev	rcdev;
};

static struct clk_onecell_data clk_data;
static struct se1000_crg_data crg_ctrl_data;

static struct se_fixed_rate_clock clks_fixed_group[] = {
	{FIXED_1188M, "1188m", NULL, 0, 1188000000},
	{FIXED_PIXEL, "mdp_dynamic", NULL, 0, 150000000},
	{OSC_25M_CLK, "osc_25m", NULL, 0, 25000000},
	{OSC_1M_CLK, "osc_1m", NULL, 0, 1000000},
	{OSC_19P2M_CLK, "osc_19p2m", NULL, 0, 19200000},
};

static struct se1000_core_clk pll_clks_group[] = {
	{ APSS0_PLL0_OUT, "ap0_pll0_out", "osc_25m", 0,
		APSS0_PLL0_PLLCFG4, 0,
		APSS0_PLL0_PLLCFG0, APSS0_PLL0_PLLCFG1,
		APSS0_PLL0_PLLCFG2, APSS0_PLL0_PLLCFG3,
		APSS0_PLL0_PLLCFG4, APSS0_PLL0_PLLCFG5,
	},

	{ APSS0_PLL1_OUT, "ap0_pll1_out", "osc_25m", 0,
		APSS0_PLL1_PLLCFG4, 0,
		APSS0_PLL1_PLLCFG0, APSS0_PLL1_PLLCFG1,
		APSS0_PLL1_PLLCFG2, APSS0_PLL1_PLLCFG3,
		APSS0_PLL1_PLLCFG4, APSS0_PLL1_PLLCFG5,
	},

	{ APSS0_PLL2_OUT, "ap0_pll2_out", "osc_25m", 0,
		APSS0_PLL2_PLLCFG4, 0,
		APSS0_PLL2_PLLCFG0, APSS0_PLL2_PLLCFG1,
		APSS0_PLL2_PLLCFG2, APSS0_PLL2_PLLCFG3,
		APSS0_PLL2_PLLCFG4, APSS0_PLL2_PLLCFG5,
	},

	{ APSS1_PLL0_OUT, "ap1_pll0_out", "osc_25m", 0,
		APSS1_PLL0_PLLCFG4, 0,
		APSS1_PLL0_PLLCFG0, APSS1_PLL0_PLLCFG1,
		APSS1_PLL0_PLLCFG2, APSS1_PLL0_PLLCFG3,
		APSS1_PLL0_PLLCFG4, APSS1_PLL0_PLLCFG5,
	},

	{ APSS1_PLL1_OUT, "ap1_pll1_out", "osc_25m", 0,
		APSS1_PLL1_PLLCFG4, 0,
		APSS1_PLL1_PLLCFG0, APSS1_PLL1_PLLCFG1,
		APSS1_PLL1_PLLCFG2, APSS1_PLL1_PLLCFG3,
		APSS1_PLL1_PLLCFG4, APSS1_PLL1_PLLCFG5,
	},

	{ DDR_PLL_OUT, "ddr_pll_out", "osc_25m", 0,
		DDR_PLL_PLLCFG4, 0,
		DDR_PLL_PLLCFG0, DDR_PLL_PLLCFG1,
		DDR_PLL_PLLCFG2, DDR_PLL_PLLCFG3,
		DDR_PLL_PLLCFG4, DDR_PLL_PLLCFG5,
	},

	{ DDR_PLL_OUT_FOUT1PH0, "ddr_pll_out_fout1ph0", "ddr_pll_out", 0,
		DDR_PLL_PLLCFG4, 0,
		DDR_PLL_PLLCFG0, 0,
		DDR_PLL_PLLCFG2, 0,
		DDR_PLL_PLLCFG4, 0, FOUT1PH0,
	},

	{ DDR_PLL_OUT_FOUT2, "ddr_pll_out_fout2", "ddr_pll_out", 0,
		DDR_PLL_PLLCFG4, 0,
		DDR_PLL_PLLCFG0, 0,
		DDR_PLL_PLLCFG2, 0,
		DDR_PLL_PLLCFG4, 0, FOUT2,
	},

	{ DDR_PLL_OUT_FOUT4, "ddr_pll_out_fout4", "ddr_pll_out", 0,
		DDR_PLL_PLLCFG4, 0,
		DDR_PLL_PLLCFG0, 0,
		DDR_PLL_PLLCFG2, 0,
		DDR_PLL_PLLCFG4, 0, FOUT4,
	},

	{ GPU0_PLL_OUT, "gpu0_pll_out", "osc_25m", 0,
		GPU0_PLL_PLLCFG4, 0,
		GPU0_PLL_PLLCFG0, GPU0_PLL_PLLCFG1,
		GPU0_PLL_PLLCFG2, GPU0_PLL_PLLCFG3,
		GPU0_PLL_PLLCFG4, GPU0_PLL_PLLCFG5,
	},

	{ GPU1_PLL_OUT, "gpu1_pll_out", "osc_25m", 0,
		GPU1_PLL_PLLCFG4, 0,
		GPU1_PLL_PLLCFG0, GPU1_PLL_PLLCFG1,
		GPU1_PLL_PLLCFG2, GPU1_PLL_PLLCFG3,
		GPU1_PLL_PLLCFG4, GPU1_PLL_PLLCFG5,
	},

	{ AUDIO_PLL_OUT, "audio_pll_out", "osc_25m", 0,
		AUDIO_PLL_PLLCFG4, 0,
		AUDIO_PLL_PLLCFG0, AUDIO_PLL_PLLCFG1,
		AUDIO_PLL_PLLCFG2, AUDIO_PLL_PLLCFG3,
		AUDIO_PLL_PLLCFG4, AUDIO_PLL_PLLCFG5,
	},

	/* don't modifed pcie_pll_out to other name */
	{ PCIE_PLL_OUT, "pcie_pll_out", "osc_19p2m", 0,
		PCIE_PLL_PLLCFG4, 0,
		PCIE_PLL_PLLCFG0, PCIE_PLL_PLLCFG1,
		PCIE_PLL_PLLCFG2, PCIE_PLL_PLLCFG3,
		PCIE_PLL_PLLCFG4, PCIE_PLL_PLLCFG5,
	},

	{ USB_PLL_OUT, "usb_pll_out", "osc_25m", 0,
		USB_PLL_PLLCFG4, 0,
		USB_PLL_PLLCFG0, USB_PLL_PLLCFG1,
		USB_PLL_PLLCFG2, USB_PLL_PLLCFG3,
		USB_PLL_PLLCFG4, USB_PLL_PLLCFG5,
	},

	{ LH_PLL0_OUT, "lh_pll0_out", "osc_25m", 0,
		LH_PLL0_PLLCFG4, 0,
		LH_PLL0_PLLCFG0, LH_PLL0_PLLCFG1,
		LH_PLL0_PLLCFG2, LH_PLL0_PLLCFG3,
		LH_PLL0_PLLCFG4, LH_PLL0_PLLCFG5,
	},

	{ LH_PLL0_OUT_FOUT1PH0, "lh_pll0_out_fout1ph0", "lh_pll0_out", 0,
		LH_PLL0_PLLCFG4, 0,
		LH_PLL0_PLLCFG0, 0,
		LH_PLL0_PLLCFG2, 0,
		LH_PLL0_PLLCFG4, 0, FOUT1PH0,
	},

	{ LH_PLL0_OUT_FOUT2, "lh_pll0_out_fout2", "lh_pll0_out", 0,
		LH_PLL0_PLLCFG4, 0,
		LH_PLL0_PLLCFG0, 0,
		LH_PLL0_PLLCFG2, 0,
		LH_PLL0_PLLCFG4, 0, FOUT2,
	},

	{ LH_PLL0_OUT_FOUT3, "lh_pll0_out_fout3", "lh_pll0_out", 0,
		LH_PLL0_PLLCFG4, 0,
		LH_PLL0_PLLCFG0, 0,
		LH_PLL0_PLLCFG2, 0,
		LH_PLL0_PLLCFG4, 0, FOUT3,
	},

	{ LH_PLL0_OUT_FOUT4, "lh_pll0_out_fout4", "lh_pll0_out", 0,
		LH_PLL0_PLLCFG4, 0,
		LH_PLL0_PLLCFG0, 0,
		LH_PLL0_PLLCFG2, 0,
		LH_PLL0_PLLCFG4, 0, FOUT4,
	},

	{ LH_PLL1_OUT, "lh_pll1_out", "osc_25m", 0,
		LH_PLL1_PLLCFG4, 0,
		LH_PLL1_PLLCFG0, LH_PLL1_PLLCFG1,
		LH_PLL1_PLLCFG2, LH_PLL1_PLLCFG3,
		LH_PLL1_PLLCFG4, LH_PLL1_PLLCFG5,
	},

	{ LH_PLL1_OUT_FOUT1PH0, "lh_pll1_out_fout1ph0", "lh_pll1_out", 0,
		LH_PLL1_PLLCFG4, 0,
		LH_PLL1_PLLCFG0, 0,
		LH_PLL1_PLLCFG2, 0,
		LH_PLL1_PLLCFG4, 0, FOUT1PH0,
	},

	{ LH_PLL1_OUT_FOUT2, "lh_pll1_out_fout2", "lh_pll1_out", 0,
		LH_PLL1_PLLCFG4, 0,
		LH_PLL1_PLLCFG0, 0,
		LH_PLL1_PLLCFG2, 0,
		LH_PLL1_PLLCFG4, 0, FOUT2,
	},

	{ LH_PLL1_OUT_FOUT3, "lh_pll1_out_fout3", "lh_pll1_out", 0,
		LH_PLL1_PLLCFG4, 0,
		LH_PLL1_PLLCFG0, 0,
		LH_PLL1_PLLCFG2, 0,
		LH_PLL1_PLLCFG4, 0, FOUT3,
	},

	{ LH_PLL1_OUT_FOUT4, "lh_pll1_out_fout4", "lh_pll1_out", 0,
		LH_PLL1_PLLCFG4, 0,
		LH_PLL1_PLLCFG0, 0,
		LH_PLL1_PLLCFG2, 0,
		LH_PLL1_PLLCFG4, 0, FOUT4,
	},

	{ LH_PLL2_OUT, "lh_pll2_out", "osc_25m", 0,
		LH_PLL2_PLLCFG4, 0,
		LH_PLL2_PLLCFG0, LH_PLL2_PLLCFG1,
		LH_PLL2_PLLCFG2, LH_PLL2_PLLCFG3,
		LH_PLL2_PLLCFG4, LH_PLL2_PLLCFG5,
	},

	{ LH_PLL2_OUT_FOUT1PH0, "lh_pll2_out_fout1ph0", "lh_pll2_out", 0,
		LH_PLL2_PLLCFG4, 0,
		LH_PLL2_PLLCFG0, 0,
		LH_PLL2_PLLCFG2, 0,
		LH_PLL2_PLLCFG4, 0, FOUT1PH0,
	},

	{ LH_PLL2_OUT_FOUT2, "lh_pll2_out_fout2", "lh_pll2_out", 0,
		LH_PLL2_PLLCFG4, 0,
		LH_PLL2_PLLCFG0, 0,
		LH_PLL2_PLLCFG2, 0,
		LH_PLL2_PLLCFG4, 0, FOUT2,
	},

	{ LH_PLL2_OUT_FOUT3, "lh_pll2_out_fout3", "lh_pll2_out", 0,
		LH_PLL2_PLLCFG4, 0,
		LH_PLL2_PLLCFG0, 0,
		LH_PLL2_PLLCFG2, 0,
		LH_PLL2_PLLCFG4, 0, FOUT3,
	},

	{ LH_PLL2_OUT_FOUT4, "lh_pll2_out_fout4", "lh_pll2_out", 0,
		LH_PLL2_PLLCFG4, 0,
		LH_PLL2_PLLCFG0, 0,
		LH_PLL2_PLLCFG2, 0,
		LH_PLL2_PLLCFG4, 0, FOUT4,
	},

	{ GPU2_PLL_OUT, "gpu2_pll_out", "osc_25m", 0,
		GPU2_PLL_PLLCFG4, 0,
		GPU2_PLL_PLLCFG0, GPU2_PLL_PLLCFG1,
		GPU2_PLL_PLLCFG2, GPU2_PLL_PLLCFG3,
		GPU2_PLL_PLLCFG4, GPU2_PLL_PLLCFG5,
	},

	{ GPU2_PLL_OUT_FOUT1PH0, "gpu2_pll_out_fout1ph0", "gpu2_pll_out", 0,
		GPU2_PLL_PLLCFG4, 0,
		GPU2_PLL_PLLCFG0, 0,
		GPU2_PLL_PLLCFG2, 0,
		GPU2_PLL_PLLCFG4, 0, FOUT1PH0,
	},

	{ GPU2_PLL_OUT_FOUT2, "gpu2_pll_out_fout2", "gpu2_pll_out", 0,
		GPU2_PLL_PLLCFG4, 0,
		GPU2_PLL_PLLCFG0, 0,
		GPU2_PLL_PLLCFG2, 0,
		GPU2_PLL_PLLCFG4, 0, FOUT2,
	},

	{ GPU2_PLL_OUT_FOUT4, "gpu2_pll_out_fout4", "gpu2_pll_out", 0,
		GPU2_PLL_PLLCFG4, 0,
		GPU2_PLL_PLLCFG0, 0,
		GPU2_PLL_PLLCFG2, 0,
		GPU2_PLL_PLLCFG4, 0, FOUT4,
	},

	{ DP_PLL_OUT, "dp_pll_out", "osc_25m", RESET_YES,
		DP_PLL_PLLCFG4, 0,
		DP_PLL_PLLCFG0, DP_PLL_PLLCFG1,
		DP_PLL_PLLCFG2, DP_PLL_PLLCFG3,
		DP_PLL_PLLCFG4, DP_PLL_PLLCFG5,
	},

	{ DP_PLL_OUT_FOUT1PH0, "dp_pll_out_fout1ph0", "dp_pll_out", RESET_YES,
		DP_PLL_PLLCFG4, 0,
		DP_PLL_PLLCFG0, 0,
		DP_PLL_PLLCFG2, 0,
		DP_PLL_PLLCFG4, 0, FOUT1PH0,
	},

	{ DP_PLL_OUT_FOUT2, "dp_pll_out_fout2", "dp_pll_out", RESET_YES,
		DP_PLL_PLLCFG4, 0,
		DP_PLL_PLLCFG0, 0,
		DP_PLL_PLLCFG2, 0,
		DP_PLL_PLLCFG4, 0, FOUT2,
	},

	{ DP_PLL_OUT_FOUT4, "dp_pll_out_fout4", "dp_pll_out", RESET_YES,
		DP_PLL_PLLCFG4, 0,
		DP_PLL_PLLCFG0, 0,
		DP_PLL_PLLCFG2, 0,
		DP_PLL_PLLCFG4, 0, FOUT4,
	},

	{ NPU_PLL_OUT, "npu_pll_out", "osc_25m", 0,
		NPU_PLL_PLLCFG4, 0,
		NPU_PLL_PLLCFG0, NPU_PLL_PLLCFG1,
		NPU_PLL_PLLCFG2, NPU_PLL_PLLCFG3,
		NPU_PLL_PLLCFG4, NPU_PLL_PLLCFG5,
	},

	{ NPU_PLL_OUT_FOUT1PH0, "npu_pll_out_fout1ph0", "npu_pll_out", 0,
		NPU_PLL_PLLCFG4, 0,
		NPU_PLL_PLLCFG0, 0,
		NPU_PLL_PLLCFG2, 0,
		NPU_PLL_PLLCFG4, 0, FOUT1PH0,
	},

	{ NPU_PLL_OUT_FOUT2, "npu_pll_out_fout2", "npu_pll_out", 0,
		NPU_PLL_PLLCFG4, 0,
		NPU_PLL_PLLCFG0, 0,
		NPU_PLL_PLLCFG2, 0,
		NPU_PLL_PLLCFG4, 0, FOUT2,
	},

	{ NPU_PLL_OUT_FOUT3, "npu_pll_out_fout3", "npu_pll_out", 0,
		NPU_PLL_PLLCFG4, 0,
		NPU_PLL_PLLCFG0, 0,
		NPU_PLL_PLLCFG2, 0,
		NPU_PLL_PLLCFG4, 0, FOUT3,
	},

	{ NPU_PLL_OUT_FOUT4, "npu_pll_out_fout4", "npu_pll_out", 0,
		NPU_PLL_PLLCFG4, 0,
		NPU_PLL_PLLCFG0, 0,
		NPU_PLL_PLLCFG2, 0,
		NPU_PLL_PLLCFG4, 0, FOUT4,
	},
};

static struct se1000_core_clk saf_pll_clks_group[] = {
	{ SAF_PLL0_OUT, "saf_pll0_out", "osc_1m", 0,
		SAF_PLL0_CTRL, 0,
		SAF_PLL0_CTRL, SAF_PLL0_FBDIV,
		SAF_PLL0_FRAC, SAF_PLL0_CTRL,
		SAF_PLL0_CTRL, 0,
	},

	{ SAF_PLL1_OUT, "saf_pll1_out", "osc_25m", 0,
		SAF_PLL1_CTRL, 0,
		SAF_PLL1_CTRL, SAF_PLL1_FBDIV,
		SAF_PLL1_FRAC, SAF_PLL1_CTRL,
		SAF_PLL1_CTRL, 0,
	},

	{ SAF_PLL2_OUT, "saf_pll2_out", "osc_25m", 0,
		SAF_PLL2_CTRL, 0,
		SAF_PLL2_CTRL, SAF_PLL2_FBDIV,
		SAF_PLL2_FRAC, SAF_PLL2_CTRL,
		SAF_PLL2_CTRL, 0,
	},

	{ SAF_PLL3_OUT, "saf_pll3_out", "osc_25m", 0,
		SAF_PLL3_CTRL, 0,
		SAF_PLL3_CTRL, SAF_PLL3_FBDIV,
		SAF_PLL3_FRAC, SAF_PLL3_CTRL,
		SAF_PLL3_CTRL, 0,
	},

	{ SAF_PLL3_OUT_FOUT1PH0, "saf_pll3_out_fout1ph0", "saf_pll3_out", 0,
		SAF_PLL3_CTRL, 0,
		SAF_PLL3_CTRL, 0,
		SAF_PLL3_FRAC, 0,
		SAF_PLL3_CTRL, 0, FOUT1PH0,
	},

	{ SAF_PLL3_OUT_FOUT2, "saf_pll3_out_fout2", "saf_pll3_out", 0,
		SAF_PLL3_CTRL, 0,
		SAF_PLL3_CTRL, 0,
		SAF_PLL3_FRAC, 0,
		SAF_PLL3_CTRL, 0, FOUT2,
	},

	{ SAF_PLL3_OUT_FOUT3, "saf_pll3_out_fout3", "saf_pll3_out", 0,
		SAF_PLL3_CTRL, 0,
		SAF_PLL3_CTRL, 0,
		SAF_PLL3_FRAC, 0,
		SAF_PLL3_CTRL, 0, FOUT3,
	},
};

static struct se1000_core_clk saf_clks_group[] = {
	/* saf_bus_clk clk_flag must set 0 */
	{ SAF_DEVSEL_OUT, "saf_dev_clk", "saf_pll1_out", 0,
		SAF_CLKSEL, 0,
	},
	{ SAF_BUS_CLK, "saf_bus_clk", "saf_pll1_out", 0,
		SAF_CLKSEL, 0,
	},
	{ SAF_ETH1_REF_CLK, "saf_eth1_ref_clk", "saf_pll2_out", SAF_CLK_EN_MASK(4),
		SAF_CLKEN, 0,
	},

	{ SAF_ETH0_REF_CLK, "saf_eth0_ref_clk", "saf_pll2_out", SAF_CLK_EN_MASK(3),
		SAF_CLKEN, 0,
	},

	{ SAF_QSPI_REF_CLK, "saf_qspi_ref_clk", "saf_pll2_out", SAF_CLK_EN_MASK(2),
		SAF_CLKEN, 0,
		SAF_CLKSEL,
	},

	{ SAF_SBISTC_CLK, "saf_sbistc_clk", "saf_bus_clk", SAF_CLK_EN_MASK(1),
		SAF_CLKEN, 0,
	},

	{ SAF_RAM_CLK, "saf_ram_clk", "saf_bus_clk", SAF_CLK_EN_MASK(0),
		SAF_CLKEN, 0,
	},

	{ SAF_MMU0_CLK, "saf_mmu0_clk", "saf_dev_clk", SAF_CLK_EN_MASK(17),
		SAF_DEVCKEN, 0,
	},

	{ SAF_ETH1_CLK, "saf_eth1_clk", "saf_dev_clk", SAF_CLK_EN_MASK(16),
		SAF_DEVCKEN, 0,
	},

	{ SAF_ETH0_CLK, "saf_eth0_clk", "saf_dev_clk", SAF_CLK_EN_MASK(15),
		SAF_DEVCKEN, 0,
	},

	{ SAF_QSPI_CLK, "saf_qspi_clk", "saf_dev_clk", SAF_CLK_EN_MASK(14),
		SAF_DEVCKEN, 0,
	},

	{ SAF_TIMERS_CLK, "saf_timers_clk", "saf_dev_clk", SAF_CLK_EN_MASK(13),
		SAF_DEVCKEN, 0,
	},

	{ SAF_I2C5_CLK, "saf_i2c5_clk", "saf_dev_clk", SAF_CLK_EN_MASK(12),
		SAF_DEVCKEN, 0,
	},

	{ SAF_I2C4_CLK, "saf_i2c4_clk", "saf_dev_clk", SAF_CLK_EN_MASK(11),
		SAF_DEVCKEN, 0,
	},

	{ SAF_I2C3_CLK, "saf_i2c3_clk", "saf_dev_clk", SAF_CLK_EN_MASK(10),
		SAF_DEVCKEN, 0,
	},

	{ SAF_I2C2_CLK, "saf_i2c2_clk", "saf_dev_clk", SAF_CLK_EN_MASK(9),
		SAF_DEVCKEN, 0,
	},

	{ SAF_I2C1_CLK, "saf_i2c1_clk", "saf_dev_clk", SAF_CLK_EN_MASK(8),
		SAF_DEVCKEN, 0,
	},

	{ SAF_I2C0_CLK, "saf_i2c0_clk", "saf_dev_clk", SAF_CLK_EN_MASK(7),
		SAF_DEVCKEN, 0,
	},

	{ SAF_SSPI0_CLK, "saf_sspi0_clk", "saf_bus_clk", SAF_CLK_EN_MASK(6),
		SAF_DEVCKEN, 0,
	},

	{ SAF_MSPI1_CLK, "saf_mspi1_clk", "saf_bus_clk", SAF_CLK_EN_MASK(5),
		SAF_DEVCKEN, 0,
	},

	{ SAF_MSPI0_CLK, "saf_mspi0_clk", "saf_bus_clk", SAF_CLK_EN_MASK(4),
		SAF_DEVCKEN, 0,
	},

	{ SAF_UART2_CLK, "saf_uart2_clk", "saf_bus_clk", SAF_CLK_EN_MASK(3),
		SAF_DEVCKEN, 0,
	},

	{ SAF_UART1_CLK, "saf_uart1_clk", "saf_bus_clk", SAF_CLK_EN_MASK(2),
		SAF_DEVCKEN, 0,
	},

	{ SAF_UART0_CLK, "saf_uart0_clk", "saf_bus_clk", SAF_CLK_EN_MASK(1),
		SAF_DEVCKEN, 0,
	},

	{ SAF_GPIO_CLK, "saf_gpio_clk", "saf_bus_clk", SAF_CLK_EN_MASK(0),
		SAF_DEVCKEN, 0,
	},

	/* don't change saf_sadp_pix_clk to other name */
	{ SAF_SADP_PIX_CLK, "saf_sadp_pix_clk", "saf_pll3_out_fout1ph0", SAF_CLK_EN_MASK(1),
		SAF_SADPCTL, 0,
	},

	/* don't change saf_sadp_axi_clk to other name */
	{SAF_SADP_AXI_CLK, "saf_sadp_axi_clk", "saf_pll3_out", SAF_CLK_EN_MASK(2),
		SAF_SADPCTL, 0,
	},

	/* don't change saf_sadp_apb_clk to other name */
	{ SAF_SADP_APB_CLK, "saf_sadp_apb_clk", "saf_pll3_out_fout1ph0", SAF_CLK_EN_MASK(3),
		SAF_SADPCTL, 0,
	},

	{ SAF_SADP_DPHY_CFG_CLK, "sadp_dphy_cfg_clk", "osc_25m", 0,
		-1, 0,
	},

	{ SAF_SADP_DPHY_REF_CLK, "sadp_dphy_ref_clk", "osc_25m", 0,
		-1, 0,
	},
};

static struct se1000_core_clk clks_group[] = {
	/*
	 * PERI1 UART 0 shared this APB clock, PLL output is 400Mhz,
	 * clk controller default divider value is 4,
	 * final APB output is 400/4=100Mhz
	 */
	{ PERI0_APB_CLK, "peri0_apb_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | RESET_YES,
		PERI0_SS_PERI0_APB_CLK, PERI0_SS_PERI0_APB_RST,
	},

	{ PERI1_APB_CLK, "peri1_apb_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | RESET_YES,
		PERI1_SS_PERI1_APB_CLK, PERI1_SS_PERI1_APB_RST,
	},

	{ PERI2_APB_CLK, "peri2_apb_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | RESET_YES,
		PERI2_SS_PERI2_APB_CLK, PERI2_SS_PERI2_APB_RST,
	},

	{ NPU0_CLK, "npu0_clk", "npu_pll_out", PLL_CLK_OUTPUT | RESET_YES,
		NPU0_SS_NPU0_CLK, NPU0_SS_NPU0_RST,
	},

	{ USB3_UTMI_REF_CLK, "usb3_utmi_ref_clk", "usb_pll_out", PLL_CLK_OUTPUT,
		USB3_SS_USB3_UTMI_REF_CLK, 0,
	},

	{ USB2_PHY_CLK, "usb2_phy_clk", "usb_pll_out", PLL_CLK_OUTPUT | RESET_YES,
		USB2_SS_USB2_PHY_CLK, USB2_SS_USB2_PRST,
	},

	{ VPU_DEC_AXI_CLK, "vpu_dec_axi_clk", "lh_pll2_out_fout2", PLL_CLK_OUTPUT, // lh_pll2, div = 1
		VPU_SS_VPU_DEC_AXI_CLK, VPU_SS_VPU_DEC_AXI_RST,
	},

	{ VPU_DEC_CORE_CLK, "vpu_dec_core_clk", "lh_pll2_out_fout2", PLL_CLK_OUTPUT, // lh_pll2, div = 1
		VPU_SS_VPU_DEC_CORE_CLK, VPU_SS_VPU_DEC_CORE_RST,
	},

	{ VPU_DEC_APB_CLK, "vpu_dec_apb_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT, // lh_pll0, div = 2
		VPU_SS_VPU_DEC_APB_CLK, VPU_SS_VPU_DEC_APB_RST,
	},

	{ VPU_ENC_AXI_CLK, "vpu_enc_axi_clk", "lh_pll1_out_fout2", PLL_CLK_OUTPUT, // lh_pll1, div = 1
		VPU_SS_VPU_ENC_AXI_CLK, VPU_SS_VPU_ENC_AXI_RST,
	},

	{ VPU_ENC_CORE_CLK, "vpu_enc_core_clk", "lh_pll1_out_fout2", PLL_CLK_OUTPUT, // lh_pll1, div = 1
		VPU_SS_VPU_ENC_CORE_CLK, VPU_SS_VPU_ENC_CORE_RST,
	},

	{ VPU_ENC_APB_CLK, "vpu_enc_apb_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT, // lh_pll0, div = 2
		VPU_SS_VPU_ENC_APB_CLK, VPU_SS_VPU_ENC_APB_RST,
	},


	{ GPU2_R2D_CLK, "gpu2_r2d_clk", "gpu2_pll_out_fout1ph0", PLL_CLK_OUTPUT,
		GPU2_SS_GPU2_R2D_CLK, GPU2_SS_GPU2_R2D_RSTN,
	},

	{ GPU2_R2D_PCLK, "gpu2_r2d_pclk", "gpu2_pll_out_fout2", PLL_CLK_OUTPUT,
		GPU2_SS_GPU2_R2D_PCLK, 0,
	},

	{ GPU2_V2D_CLK, "gpu2_v2d_clk", "gpu2_pll_out", PLL_CLK_OUTPUT,
		GPU2_SS_GPU2_V2D_CLK, GPU2_SS_GPU2_V2D_RSTN,
	},

	{ MDP_DP_AUX16MHZ_CLK, "mdp_dp_aux16mhz_clk", "gpu2_pll_out_fout1ph0", PLL_CLK_OUTPUT,
		MDP_SS_DP_AUX16MHZ_CLK, 0,
	},

	{ MDP_DPU_AXI0_CLK, "dpu_d71_axi0_clk", "lh_pll0_out_fout2", PLL_CLK_OUTPUT,
		MDP_SS_D71_AXI0_CLK, 0,
	},

	{ MDP_DPU_AXI1_CLK, "dpu_d71_axi1_clk", "lh_pll0_out_fout2", PLL_CLK_OUTPUT,
		MDP_SS_D71_AXI1_CLK, 0,
	},

	{ MDP_DPU_AXI2_CLK, "dpu_d71_axi2_clk", "lh_pll0_out_fout2", PLL_CLK_OUTPUT,
		MDP_SS_D71_AXI2_CLK, 0,
	},

	{ MDP_APB_CLK, "mdp_apb_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT,
		MDP_SS_MDP_APB_CLK, 0,
	},

	{ HDCP_AXI_CLK, "hdcp_axi_clk", "lh_pll2_out_fout4", PLL_CLK_OUTPUT,
		MDP_SS_HDCP_AXI_CLK, 0,
	},

	{ MDP_DPHY_REL_CLK, "mdp_dphy_rel_clk", "lh_pll0_out_fout1ph0", PLL_CLK_OUTPUT,
		MDP_SS_DPHY_REF_CLK, 0,
	},

	{ MDP_DPHY_CFG_CLK, "mdp_dphy_cfg_clk", "lh_pll0_out_fout4", PLL_CLK_OUTPUT,
		MDP_SS_DPHY_CFG_CLK, 0,
	},

	{ DPTXPHY_MBIST_CLK0, "dptxphy_mbist_clk0", "lh_pll0_out_fout3", PLL_CLK_OUTPUT,
		MDP_SS_DPTXPHY_MBIST_CLK0, 0,
	},

	{ DPHY_MBIST_CLK, "dphy_mbist_clk", "lh_pll0_out_fout4", PLL_CLK_OUTPUT,
		MDP_SS_DPHY_MBIST_CLK, 0,
	},

	{ PCIE_PHY_REF_ALT_CLK, "pcie_phy_ref_alt_clk", "pcie_pll_out", PLL_CLK_OUTPUT, /* lh_pll0_div3 */
		PCIE_SS_PCIE_PHY_REF_ALT_CLK, 0,
	},

/* pcie ss start */
	{ PCIE_CTR_AUX_CLK, "pcie_ctr_aux_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT, /* lh_pll0_div3 */
		PCIE_SS_PCIE_CTR_AUX_CLK, 0,
	},

	{ PCIE_CTR_AXI_ACLK, "pcie_ctr_axi_aclk", "lh_pll1_out_fout2", PLL_CLK_OUTPUT, /* lh_pll1_div2 */
		PCIE_SS_PCIE_CTR_AXI_ACLK, 0,
	},

	{ PCIE_CTR_AHB_HCLK, "pcie_ctr_ahb_hclk", "lh_pll1_out_fout2", PLL_CLK_OUTPUT | RESET_YES, /* lh_pll1_div2 */
		PCIE_SS_PCIE_CTR_AHB_HCLK, PCIE_SS_PCIE_AHB_HRESETN,
	},

	{ PCIE_PHY_APB_PCLK, "pcie_phy_apb_pclk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | RESET_YES, /* lh_pll0_div3 */
		PCIE_SS_PCIE_PHY_APB_PCLK, PCIE_SS_PCIE_APB_PESETN,
	},

	{ PCIE_PHY_RESETN, "pcie_phy_resetn", "lh_pll0_out_fout3", PLL_CLK_OUTPUT, /* reset only */
		-1, PCIE_SS_PCIE_PHY_RESETN,
	},

	{ PCIE_CTR0_RESET_N, "pcie_ctr0_resetn", "lh_pll0_out_fout3", PLL_CLK_OUTPUT, /* reset only */
		-1, PCIE_SS_PCIE_CTR0_PESET_N,
	},

	{ PCIE_CTR1_RESET_N, "pcie_ctr1_resetn", "lh_pll0_out_fout3", PLL_CLK_OUTPUT, /* reset only */
		-1, PCIE_SS_PCIE_CTR1_RESET_N,
	},

	{ PCIE_CTR2_RESET_N, "pcie_ctr2_resetn", "lh_pll0_out_fout3", PLL_CLK_OUTPUT, /* reset only */
		-1, PCIE_SS_PCIE_CTR2_RESET_N,
	},

	{ PCIE_CTR3_RESET_N, "pcie_ctr3_resetn", "lh_pll0_out_fout3", PLL_CLK_OUTPUT, /* reset only */
		-1, PCIE_SS_PCIE_CTR3_RESET_N,
	},

	{ PCIE_CTR0_POWER_UP_RST_N, "pcie_ctr0_pwr_up_rst", "lh_pll0_out_fout3", PLL_CLK_OUTPUT, /* reset only */
		-1, PCIE_SS_PCIE_CTR0_POWER_UP_RST,
	},

	{ PCIE_CTR1_POWER_UP_RST_N, "pcie_ctr1_pwr_up_rst", "lh_pll0_out_fout3", PLL_CLK_OUTPUT, /* reset only */
		-1, PCIE_SS_PCIE_CTR1_POWER_UP_RST,
	},

	{ PCIE_CTR2_POWER_UP_RST_N, "pcie_ctr2_pwr_up_rst", "lh_pll0_out_fout3", PLL_CLK_OUTPUT, /* reset only */
		-1, PCIE_SS_PCIE_CTR2_POWER_UP_RST,
	},

	{ PCIE_CTR3_POWER_UP_RST_N, "pcie_ctr3_pwr_up_rst", "lh_pll0_out_fout3", PLL_CLK_OUTPUT, /* reset only */
		-1, PCIE_SS_PCIE_CTR3_POWER_UP_RST,
	},

	{ PCIE_CTR0_BUTTON_RST_N, "pcie_ctr0_button_rst", "lh_pll0_out_fout3", PLL_CLK_OUTPUT, /* reset only */
		-1, PCIE_SS_PCIE_CTR0_BUTTON_RST,
	},

	{ PCIE_CTR1_BUTTON_RST_N, "pcie_ctr1_button_rst", "lh_pll0_out_fout3", PLL_CLK_OUTPUT, /* reset only */
		-1, PCIE_SS_PCIE_CTR1_BUTTON_RST,
	},

	{ PCIE_CTR2_BUTTON_RST_N, "pcie_ctr2_button_rst", "lh_pll0_out_fout3", PLL_CLK_OUTPUT, /* reset only */
		-1, PCIE_SS_PCIE_CTR2_BUTTON_RST,
	},

	{ PCIE_CTR3_BUTTON_RST_N, "pcie_ctr3_button_rst", "lh_pll0_out_fout3", PLL_CLK_OUTPUT, /* reset only */
		-1, PCIE_SS_PCIE_CTR3_BUTTON_RST,
	},
/* pcie ss end */

	{ SDHCI_CTRL_CLK, "mshcclk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | RESET_YES,
		SFC_SS_MSHCCLK, SFC_SS_MSHCRESETN,
	},

	{ SDHCI_TX_CLK, "mshc_tx_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | RESET_YES,
		SFC_SS_MSHC_TX_CLK, SFC_SS_MSHC_TX_RESETN,
	},

	{ SFC_AHB_SLV_CLK, "sfc_ahb_slv_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | RESET_YES,
		SFC_SS_SFC_AHB_SLV_CLK, SFC_SS_SFC_AHB_SLV_RESETN,
	},

	{ SFC_UFS_TX_MBIST_CLK, "sfc_ufs_tx_mbist_clk", "lh_pll1_out_fout2", PLL_CLK_OUTPUT,
		SFC_SS_SFC_UFS_TX_MBIST_CLK, 0,
	},

	{ SFC_UFS_RX_MBIST_CLK, "sfc_ufs_rx_mbist_clk", "lh_pll0_out_fout4", PLL_CLK_OUTPUT,
		SFC_SS_SFC_UFS_RX_MBIST_CLK, 0,
	},
	{ MAINCLK, "mainclk", "lh_pll1_out_fout2", PLL_CLK_OUTPUT | RESET_YES,
		SFC_SS_MAINCLK, SFC_SS_MAINRESETN,
	},

	{ UFSHCCLK, "ufshcclk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | RESET_YES,
		SFC_SS_UFSHCCLK, SFC_SS_SFC_UFSHC_RESETN,
	},

	{ IPSCLK, "ipsclk", "lh_pll0_out_fout4", PLL_CLK_OUTPUT | RESET_YES,
		SFC_SS_IPSCLK, SFC_SS_IPSRESETN,
	},

	{ GPU0_CLK, "gpu0_clk", "gpu0_pll_out", PLL_CLK_OUTPUT | RESET_YES,
		GPU0_SS_GPU0_CLK, GPU0_SS_GPU0_RESETN,
	},

	{ GPU1_CLK, "gpu1_clk", "gpu1_pll_out", PLL_CLK_OUTPUT | RESET_YES,
		GPU1_SS_GPU1_CLK, GPU1_SS_GPU1_RESETN,
	},

	{ USB2_AXI_CLK, "usb2_axi_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | RESET_YES,
		USB2_SS_USB2_AXI_CLK, USB2_SS_USB2_AXI_RST,
	},

	{ USB3_AXI_CLK, "usb3_bus_clk", "lh_pll0_out_fout4", PLL_CLK_OUTPUT | RESET_YES,
		USB3_SS_USB3_BUS_CLK, USB3_SS_USB3_VCC_RST,
	},

	{ USB3_APB_CLK, "usb3_apb_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | RESET_YES,
		USB3_SS_USB3_APB_CLK, USB3_SS_USB3_APB_RST,
	},

	{ USB3_PIPEREF_CLK, "usb3_ctr_ref_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT|RESET_YES,
		USB3_SS_USB3_CTR_REF_CLK, USB3_SS_USB3_APB_RST,
	},

	{ USB3_UTMIREF_CLK, "usb3_ctr_mbist_clk", "lh_pll0_out_fout4", PLL_CLK_OUTPUT,
		USB3_SS_USB3_CTRL_MBIST_CLK, 0,
	},

	{ USB3_POR_RESET, "usb_por", "lh_pll0_out_fout3", PLL_CLK_OUTPUT, /* reset only */
		-1, USB3_SS_USB3_POR,
	},

	{ SMP_CM4_CLK, "smp_cm4_clk", "lh_pll1_out_fout2", PLL_CLK_OUTPUT,
		SMP_SS_SMP_CM4_CLK, 0,
	},

	{ SMP_CLK, "smp_clk", "lh_pll0_out_fout4", PLL_CLK_OUTPUT,
		SMP_SS_SMP_CLK, 0,
	},

	{ SMP_SC_CLK, "smp_sc_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT,
		SMP_SS_SMP_SC_CLK, 0,
	},

	{ TOP_MODULE_CLK, "top_module_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT,
		SMP_SS_TOP_MODULE_CLK, 0,
	},

	{ PERI0_AHB_CLK, "peri0_ahb_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | RESET_YES,
		PERI0_SS_PERI0_AHB_CLK, PERI0_SS_PERI0_AHB_RST,
	},

	{ PERI1_AHB_CLK, "peri1_ahb_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | RESET_YES,
		PERI1_SS_PERI1_AHB_CLK, PERI1_SS_PERI1_AHB_RST,
	},

	{ PERI2_AHB_CLK, "peri2_ahb_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | RESET_YES,
		PERI2_SS_PERI2_AHB_CLK, PERI2_SS_PERI2_AHB_RST,
	},

	{ PERI1_AXI_CLK, "peri1_axi_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | RESET_YES,
		PERI1_SS_PERI1_AXI_CLK, PERI1_SS_PERI1_AXI_RST,
	},

	{ PERI1_TIM_CLK, "peri1_tim_clk", "lh_pll0_out_fout4", PLL_CLK_OUTPUT | RESET_YES,
		PERI1_SS_PERI1_TIM_CLK, PERI1_SS_PERI1_TIM_RST,
	},

	{ PERI1_REF_CLK, "peri1_ref_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | RESET_YES,
		PERI1_SS_PERI1_REF_CLK, PERI1_SS_PERI1_REF_RST,
	},

	{ CLK_CS, "clk_cs", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | RESET_YES,
		DBG_SS_CLK_CS, DBG_SS_RST_N_CS,
	},

	{ TRACECLK, "traceclk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | RESET_YES,
		DBG_SS_TRACECLK, DBG_SS_TRESET_N,
	},

	/*pinmux clk default seleet pll2*/
	{ GPCLK0, "gpclk0", "lh_pll2_out_fout2", PLL_CLK_OUTPUT,
		PINMUX_GPCLK0, 0,
	},

	{ GPCLK1, "gpclk1", "lh_pll2_out_fout2", PLL_CLK_OUTPUT,
		PINMUX_GPCLK1, 0,
	},

	{ GPCLK2, "gpclk2", "lh_pll2_out_fout2", PLL_CLK_OUTPUT,
		PINMUX_GPCLK2, 0,
	},

	{ GPCLK3, "gpclk3", "lh_pll2_out_fout2", PLL_CLK_OUTPUT,
		PINMUX_GPCLK3, 0,
	},

	{ GPCLK4, "gpclk4", "lh_pll2_out_fout2", PLL_CLK_OUTPUT,
		PINMUX_GPCLK4, 0,
	},

	{ GPCLK5, "gpclk5", "lh_pll2_out_fout2", PLL_CLK_OUTPUT,
		PINMUX_GPCLK5, 0,
	},

	{ GPCLK6, "gpclk6", "lh_pll2_out_fout2", PLL_CLK_OUTPUT,
		PINMUX_GPCLK6, 0,
	},

	{ GPCLK7, "gpclk7", "lh_pll2_out_fout2", PLL_CLK_OUTPUT,
		PINMUX_GPCLK7, 0,
	},

	{ SYSNOC_MAIN_CLK, "sysnoc_main_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT,
		NOC_WRAP_SYSNOC_MAIN_CLK, 0,
	},

	{ AP0NOC_SERVICE_CLK, "ap0noc_service_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT,
		NOC_WRAP_AP0NOC_SERVICE_CLK, 0,
	},

	{ AP1NOC_SERVICE_CLK, "ap1noc_service_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT,
		NOC_WRAP_AP1NOC_SERVICE_CLK, 0,
	},

	{ NPU1_TT_CLK, "npu1_tt_clk", "lh_pll1_out_fout1ph0", PLL_CLK_OUTPUT | RESET_YES,
		NPU1_SS_NPU1_TT_CLK, NPU1_SS_NPU1_RST,
	},

	{ NPU1_DBG_CLK, "npu1_dbg_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT,
		NPU1_SS_NPU1_DBG_CLK, 0,
	},

	{ NPU1_PLL_CLK, "npu1_pll_clk", "npu_pll_out", PLL_CLK_OUTPUT | RESET_YES,
		NPU1_SS_NPU1_PLL_CLK, NPU1_SS_NPU1_RST,
	},

	{ NPU1_SS_CLK, "npu1_ss_clk", "gpu2_pll_out_fout1ph0", PLL_CLK_OUTPUT | RESET_YES,
		NPU1_SS_NPU1_SS_CLK, NPU1_SS_NPU1_RST,
	},

	{ SFC_SMMU_CLK, "sfc_smmu_clk", "lh_pll1_out_fout2", PLL_CLK_OUTPUT,
		SFC_SS_SFC_SMMU_CLK, 0,
	},

	{ AUD_PLL_CLK, "aud_pll_clk", "audio_pll_out", 0,
		AUD_SS_AUD_PLL_CLK, 0,
	},

	{ AUD_DSP_CLK, "aud_dsp_clk", "lh_pll1_out_fout1ph0", PLL_CLK_OUTPUT | RESET_YES,
		AUD_SS_AUD_DSP_CLK, AUD_SS_AUD_DSP0_RST,
	},

	{ AUD_CLK, "aud_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | RESET_YES,
		AUD_SS_AUD_CLK, AUD_SS_AUD_RST,
	},

	{ CISP_AXI_CLK, "cisp_axi_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | CLK_FIX_EN_DIS,
		CISP_SS_CISP_AXI_CLK, 0,
	},

	{ CISP_APB_CLK, "cisp_apb_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | CLK_FIX_EN_DIS,
		CISP_SS_CISP_APB_CLK, 0,
	},

	{ CISP_AHB_CLK, "cisp_ahb_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | CLK_FIX_EN_DIS,
		CISP_SS_CISP_AHB_CLK, 0,
	},

	{ CISP_PIXEL_CLK, "cisp_pixel_clk", "lh_pll1_out_fout3", PLL_CLK_OUTPUT | CLK_FIX_EN_DIS,
		CISP_SS_CISP_PIXEL_CLK, 0,
	},

	{ CISP_DPHY_CFG_CLK, "cisp_dphy_cfg_clk", "lh_pll0_out_fout3", PLL_CLK_OUTPUT | CLK_FIX_EN_DIS,
		CISP_SS_CISP_DPHY_CFG_CLK, 0,
	},

	{ CISP_DPHY_REF_CLK, "cisp_dphy_ref_clk", "lh_pll0_out_fout1ph0", PLL_CLK_OUTPUT | CLK_FIX_EN_DIS,
		CISP_SS_CISP_DPHY_REF_CLK, 0,
	},

	{ CISP_ISP_CORE_CLK, "cisp_isp_core_clk", "gpu2_pll_out_fout1ph0", PLL_CLK_OUTPUT | CLK_FIX_EN_DIS,
		CISP_SS_CISP_ISP_CORE_CLK, 0,
	},

	{ CISP_GDC_CORE_CLK, "cisp_gdc_core_clk", "gpu2_pll_out_fout1ph0", PLL_CLK_OUTPUT | CLK_FIX_EN_DIS,
		CISP_SS_CISP_GDC_CORE_CLK, 0,
	},

	{ AP0_CLK_CORE0, "cpu0_clk", "ap0_pll1_out", PLL_CLK_OUTPUT | RESET_YES,
		 AP0_SS_CLK_CORE0, AP0_SS_RST_N_CORE0,
	},

	{ AP0_CLK_CORE1, "cpu1_clk", "ap0_pll1_out", PLL_CLK_OUTPUT | RESET_YES,
		 AP0_SS_CLK_CORE1, AP0_SS_RST_N_CORE1,
	},

	{ AP0_CLK_CORE2, "cpu2_clk", "ap0_pll0_out", PLL_CLK_OUTPUT | RESET_YES,
		AP0_SS_CLK_CORE2, AP0_SS_RST_N_CORE2,
	},

	{ AP0_CLK_CORE3, "cpu3_clk", "ap0_pll0_out", PLL_CLK_OUTPUT | RESET_YES,
		 AP0_SS_CLK_CORE3, AP0_SS_RST_N_CORE3,
	},

	{ AP0_CLK_CORE4, "cpu4_clk", "ap0_pll0_out", PLL_CLK_OUTPUT | RESET_YES,
		AP0_SS_CLK_CORE4, AP0_SS_RST_N_CORE4,
	},

	{ AP0_CLK_CORE5, "cpu5_clk", "ap0_pll0_out", PLL_CLK_OUTPUT | RESET_YES,
		AP0_SS_CLK_CORE5, AP0_SS_RST_N_CORE5,
	},

	{ AP0_CLK_SCLK, "ap0_clk_sclk", "ap0_pll2_out", PLL_CLK_OUTPUT | RESET_YES,
		AP0_SS_CLK_SCLK, AP0_SS_RST_N_SCLK,
	},

	{ AP0_CLK_PCLK, "ap0_clk_pclk", "ap0_pll2_out", PLL_CLK_OUTPUT | RESET_YES,
		AP0_SS_CLK_PCLK, AP0_SS_RST_N_PCLK,
	},

	{ AP0_CLK_ATCLK, "ap0_clk_atclk", "ap0_pll2_out", PLL_CLK_OUTPUT | RESET_YES,
		AP0_SS_CLK_ATCLK, AP0_SS_RST_N_ATCLK,
	},

	{ AP0_CLK_GIC, "ap0_clk_gic", "ap0_pll2_out", PLL_CLK_OUTPUT | RESET_YES,
		AP0_SS_CLK_GIC, AP0_SS_RST_N_GIC,
	},

	{ AP0_CLK_PERIPH, "ap0_clk_periph", "ap0_pll2_out", PLL_CLK_OUTPUT | RESET_YES,
		AP0_SS_CLK_PERIPH, AP0_SS_RST_N_PERIPH,
	},

	{ AP0_CLK_PDBG, "ap0_clk_pdbg", "ap0_pll2_out", PLL_CLK_OUTPUT | RESET_YES,
		AP0_SS_CLK_PDBG, AP0_SS_RST_N_PDBG,
	},

	{ AP1_CLK_CORE0, "cp_cpu0_clk", "ap1_pll0_out", PLL_CLK_OUTPUT | RESET_YES,
		 AP1_SS_CLK_CORE0, AP1_SS_RST_N_CORE0,
	},

	{ AP1_CLK_CORE1, "cp_cpu1_clk", "ap1_pll0_out", PLL_CLK_OUTPUT | RESET_YES,
		 AP1_SS_CLK_CORE1, AP1_SS_RST_N_CORE1,
	},

	{ AP1_CLK_SCLK, "ap1_clk_sclk", "ap1_pll1_out", PLL_CLK_OUTPUT | RESET_YES,
		AP1_SS_CLK_SCLK, AP1_SS_RST_N_SCLK,
	},
	{ AP1_CLK_PCLK, "ap1_clk_pclk", "ap1_pll1_out", PLL_CLK_OUTPUT | RESET_YES,
		AP1_SS_CLK_PCLK, AP1_SS_RST_N_PCLK,
	},

	{ AP1_CLK_ATCLK, "ap1_clk_atclk", "ap1_pll1_out", PLL_CLK_OUTPUT | RESET_YES,
		AP1_SS_CLK_ATCLK, AP1_SS_RST_N_ATCLK,
	},

	{ AP1_CLK_GIC, "ap1_clk_gic", "ap1_pll1_out", PLL_CLK_OUTPUT | RESET_YES,
		AP1_SS_CLK_GIC, AP1_SS_RST_N_GIC,
	},

	{ AP1_CLK_PERIPH, "ap1_clk_periph", "ap1_pll1_out", PLL_CLK_OUTPUT | RESET_YES,
		AP1_SS_CLK_PERIPH, AP1_SS_RST_N_PERIPH,
	},

	{ AP1_CLK_PDBG, "ap1_clk_pdbg", "ap1_pll1_out", PLL_CLK_OUTPUT | RESET_YES,
		AP1_SS_CLK_PDBG, AP1_SS_RST_N_PDBG,
	},

	{ DDR0_DDR_CFG_CLK, "ddr0_clk_cfg", "ddr_pll_out", PLL_CLK_OUTPUT | RESET_YES,
		DDR0_SS_DDR_CFG_CLK, DDR0_SS_DDR_CTL_CFG_RST,
	},

	{ DDR0_DDR_LOW_CLK, "ddr0_clk_low", "ddr_pll_out", PLL_CLK_OUTPUT,
		DDR0_SS_DDR_LOW_CLK, 0,
	},

	{ DDR1_DDR_CFG_CLK, "ddr1_clk_cfg", "ddr_pll_out", PLL_CLK_OUTPUT | RESET_YES,
		DDR1_SS_DDR_CFG_CLK, DDR1_SS_DDR_CTL_CFG_RST,
	},

	{ DDR1_DDR_LOW_CLK, "ddr1_clk_low", "ddr_pll_out", PLL_CLK_OUTPUT,
		DDR1_SS_DDR_LOW_CLK, 0,
	},

	{ DDRNOC_MAIN_CLK, "ddrnoc_main_clk", "ddr_pll_out", PLL_CLK_OUTPUT,
		NOC_WRAP_DDRNOC_MAIN_CLK, 0,
	},

	{ GPUNOC_MAIN_CLK, "gpunoc_main_clk", "ddr_pll_out", PLL_CLK_OUTPUT,
		NOC_WRAP_GPUNOC_MAIN_CLK, 0,
	},

	{ DDR_LINK_CLK, "ddr_link_clk", "ddr_pll_out", PLL_CLK_OUTPUT,
		NOC_WRAP_DDR_LINK_CLK, 0,
	},

	/* fix clock*/
	{ DDR_CORE_CLK, "ddr_core_clk", "ddr_pll_out", 0,
		-1, 0,
	},

	{ DDR_PLLBY_CLK, "ddr_pllby_clk", "ddr_pll_out_fout4", 0,
		-1, 0,
	},

};

static struct se1000_core_clk step_clks_group[] = {
	/*step clock config*/
	{ STEP_AP0_CLK_CORE0, "step_cpu0_clk", "ap0_pll1_out", PLL_CLK_OUTPUT,
		 AP0_SS_CLK_CORE0, 0,
		 AP0_SS_CLK_CORE0_I_STEPCFG0, AP0_SS_CLK_CORE0_I_STEPCFG1,
		 AP0_SS_CLK_CORE0_I_STEPCFG2, AP0_SS_CLK_CORE0_I_STEPCFG3,
	},

	{ STEP_AP0_CLK_CORE1, "step_cpu1_clk", "ap0_pll1_out", PLL_CLK_OUTPUT,
		 AP0_SS_CLK_CORE1, 0,
		 AP0_SS_CLK_CORE1_I_STEPCFG0, AP0_SS_CLK_CORE1_I_STEPCFG1,
		 AP0_SS_CLK_CORE1_I_STEPCFG2, AP0_SS_CLK_CORE1_I_STEPCFG3,
	},

	{ STEP_AP0_CLK_CORE2, "step_cpu2_clk", "ap0_pll0_out", PLL_CLK_OUTPUT,
		 AP0_SS_CLK_CORE2, 0,
		 AP0_SS_CLK_CORE2_I_STEPCFG0, AP0_SS_CLK_CORE2_I_STEPCFG1,
		 AP0_SS_CLK_CORE2_I_STEPCFG2, AP0_SS_CLK_CORE2_I_STEPCFG3,
	},

	{ STEP_AP0_CLK_CORE3, "step_cpu3_clk", "ap0_pll0_out", PLL_CLK_OUTPUT,
		 AP0_SS_CLK_CORE3, 0,
		 AP0_SS_CLK_CORE3_I_STEPCFG0, AP0_SS_CLK_CORE3_I_STEPCFG1,
		 AP0_SS_CLK_CORE3_I_STEPCFG2, AP0_SS_CLK_CORE3_I_STEPCFG3,
	},

	{ STEP_AP0_CLK_CORE4, "step_cpu4_clk", "ap0_pll0_out", PLL_CLK_OUTPUT,
		 AP0_SS_CLK_CORE4, 0,
		 AP0_SS_CLK_CORE4_I_STEPCFG0, AP0_SS_CLK_CORE4_I_STEPCFG1,
		 AP0_SS_CLK_CORE4_I_STEPCFG2, AP0_SS_CLK_CORE4_I_STEPCFG3,
	},

	{ STEP_AP0_CLK_CORE5, "step_cpu5_clk", "ap0_pll0_out", PLL_CLK_OUTPUT,
		AP0_SS_CLK_CORE5, 0,
		AP0_SS_CLK_CORE5_I_STEPCFG0, AP0_SS_CLK_CORE5_I_STEPCFG1,
		AP0_SS_CLK_CORE5_I_STEPCFG2, AP0_SS_CLK_CORE5_I_STEPCFG3,
	},

	{ STEP_AP0_CLK_SCLK, "step_ap0_clk_sclk", "ap0_pll2_out", PLL_CLK_OUTPUT,
		AP0_SS_CLK_SCLK, 0,
		AP0_SS_CLK_SCLK_I_STEPCFG0, AP0_SS_CLK_SCLK_I_STEPCFG1,
		AP0_SS_CLK_SCLK_I_STEPCFG2, AP0_SS_CLK_SCLK_I_STEPCFG3,
	},

	{ STEP_AP1_CLK_CORE0, "step_cpu0_clk_cp", "ap1_pll0_out", PLL_CLK_OUTPUT,
		 AP1_SS_CLK_CORE0, 0,
		 AP1_SS_CLK_CORE0_I_STEPCFG0, AP1_SS_CLK_CORE0_I_STEPCFG1,
		 AP1_SS_CLK_CORE0_I_STEPCFG2, AP1_SS_CLK_CORE0_I_STEPCFG3,
	},

	{ STEP_AP1_CLK_CORE1, "step_cpu1_clk_cp", "ap1_pll0_out", PLL_CLK_OUTPUT,
		 AP1_SS_CLK_CORE1, 0,
		 AP1_SS_CLK_CORE1_I_STEPCFG0, AP1_SS_CLK_CORE1_I_STEPCFG1,
		 AP1_SS_CLK_CORE1_I_STEPCFG2, AP1_SS_CLK_CORE1_I_STEPCFG3,
	},

	{ STEP_AP1_CLK_SCLK, "step_ap1_clk_sclk", "ap1_pll1_out", PLL_CLK_OUTPUT,
		AP1_SS_CLK_SCLK, 0,
		AP1_SS_CLK_SCLK_I_STEPCFG0, AP1_SS_CLK_SCLK_I_STEPCFG1,
		AP1_SS_CLK_SCLK_I_STEPCFG2, AP1_SS_CLK_SCLK_I_STEPCFG3,
	},

	{ STEP_GPU0_CLK, "step_gpu0_clk", "gpu0_pll_out", PLL_CLK_OUTPUT,
		GPU0_SS_GPU0_CLK, 0,
		GPU0_SS_GPU0_CLK_STEPCFG0, GPU0_SS_GPU0_CLK_STEPCFG1,
		GPU0_SS_GPU0_CLK_STEPCFG2, GPU0_SS_GPU0_CLK_STEPCFG3,
	},

	{ STEP_GPU1_CLK, "step_gpu1_clk", "gpu1_pll_out", PLL_CLK_OUTPUT,
		GPU1_SS_GPU1_CLK, 0,
		GPU1_SS_GPU1_CLK_STEPCFG0, GPU1_SS_GPU1_CLK_STEPCFG1,
		GPU1_SS_GPU1_CLK_STEPCFG2, GPU1_SS_GPU1_CLK_STEPCFG3,
	},

	{ STEP_GPU2_R2D_CLK, "step_gpu2_r2d_clk", "gpu2_pll_out_fout1ph0", PLL_CLK_OUTPUT,
		GPU2_SS_GPU2_R2D_CLK, 0,
		GPU2_SS_GPU2_R2D_CLK_STEPCFG0, GPU2_SS_GPU2_R2D_CLK_STEPCFG1,
		GPU2_SS_GPU2_R2D_CLK_STEPCFG2, GPU2_SS_GPU2_R2D_CLK_STEPCFG3,
	},

	{ STEP_GPU2_V2D_CLK, "step_gpu2_v2d_clk", "gpu2_pll_out", PLL_CLK_OUTPUT,
		GPU2_SS_GPU2_V2D_CLK, 0,
		GPU2_SS_GPU2_V2D_CLK_STEPCFG0, GPU2_SS_GPU2_V2D_CLK_STEPCFG1,
		GPU2_SS_GPU2_V2D_CLK_STEPCFG2, GPU2_SS_GPU2_V2D_CLK_STEPCFG3,
	},

	{ STEP_CISP_ISP_CORE_CLK, "step_cisp_isp_core_clk", "gpu2_pll_out_fout1ph0", PLL_CLK_OUTPUT,
		CISP_SS_CISP_ISP_CORE_CLK, 0,
		CISP_SS_CISP_ISP_CORE_CLK_STEPCFG0, CISP_SS_CISP_ISP_CORE_CLK_STEPCFG1,
		CISP_SS_CISP_ISP_CORE_CLK_STEPCFG2, CISP_SS_CISP_ISP_CORE_CLK_STEPCFG3,
	},

	{ STEP_NPU1_SS_CLK, "step_npu1_ss_clk", "gpu2_pll_out_fout1ph0", PLL_CLK_OUTPUT,
		NPU1_SS_NPU1_SS_CLK, 0,
		NPU1_SS_NPU1_SS_CLK_STEPCFG0, NPU1_SS_NPU1_SS_CLK_STEPCFG1,
		NPU1_SS_NPU1_SS_CLK_STEPCFG2, NPU1_SS_NPU1_SS_CLK_STEPCFG3,
	},

	{ STEP_NPU0_CLK, "step_npu0_clk", "npu_pll_out", PLL_CLK_OUTPUT,
		NPU0_SS_NPU0_CLK, 0,
		NPU0_SS_NPU0_CLK_STEPCFG0, NPU0_SS_NPU0_CLK_STEPCFG1,
		NPU0_SS_NPU0_CLK_STEPCFG2, NPU0_SS_NPU0_CLK_STEPCFG3,
	},

	{ STEP_NPU1_PLL_CLK, "step_npu1_pll_clk", "npu_pll_out", PLL_CLK_OUTPUT,
		NPU1_SS_NPU1_PLL_CLK, 0,
		NPU1_SS_NPU1_PLL_CLK_STEPCFG0, NPU1_SS_NPU1_PLL_CLK_STEPCFG1,
		NPU1_SS_NPU1_PLL_CLK_STEPCFG2, NPU1_SS_NPU1_PLL_CLK_STEPCFG3,
	},

	{ STEP_VPU_DEC_CORE_CLK, "step_vpu_dec_core_clk", "lh_pll2_out_fout2", PLL_CLK_OUTPUT,
		VPU_SS_VPU_DEC_CORE_CLK, 0,
		VPU_SS_VPU_DEC_CORE_CLK_STEPCFG0, VPU_SS_VPU_DEC_CORE_CLK_STEPCFG1,
		VPU_SS_VPU_DEC_CORE_CLK_STEPCFG2, VPU_SS_VPU_DEC_CORE_CLK_STEPCFG3,
	},

	{ STEP_NPU1_TT_CLK, "step_npu1_tt_clk", "lh_pll1_out_fout1ph0", PLL_CLK_OUTPUT,
		NPU1_SS_NPU1_TT_CLK, 0,
		NPU1_SS_NPU1_TT_CLK_STEPCFG0, NPU1_SS_NPU1_TT_CLK_STEPCFG1,
		NPU1_SS_NPU1_TT_CLK_STEPCFG2, NPU1_SS_NPU1_TT_CLK_STEPCFG3,
	},

	{ STEP_AUD_DSP_CLK, "step_aud_dsp_clk", "lh_pll1_out_fout1ph0", PLL_CLK_OUTPUT,
		AUD_SS_AUD_DSP_CLK, 0,
		AUD_SS_AUD_DSP_CLK_STEPCFG0, AUD_SS_AUD_DSP_CLK_STEPCFG1,
		AUD_SS_AUD_DSP_CLK_STEPCFG2, AUD_SS_AUD_DSP_CLK_STEPCFG3,
	},

	{ STEP_SMP_CM4_CLK, "step_smp_cm4_clk", "lh_pll1_out_fout2", PLL_CLK_OUTPUT,
		SMP_SS_SMP_CM4_CLK, 0,
		SMP_SS_SMP_CM4_CLK_STEPCFG0, SMP_SS_SMP_CM4_CLK_STEPCFG1,
		SMP_SS_SMP_CM4_CLK_STEPCFG2, SMP_SS_SMP_CM4_CLK_STEPCFG3,
	},
};

static struct se1000_core_clk mdpclk_group[] = {
	{ MDP_DPU0_PIXEL_CLK, "dpu0_pixel_clk",
		"1188m", MDP_DPUn_PIXEL_CLK(0),
		MDP_SS_CLK_MISC_DIV, MDP_SS_CLK_MISC_EN,
	},//DPU0

	{ MDP_DPU1_PIPE0_PIXEL_CLK, "dpu1_pipe0_pixel_clk",
		"1188m", MDP_DPUn_PIXEL_CLK(1),
		MDP_SS_CLK_MISC_DIV, MDP_SS_CLK_MISC_EN,
	},//DPU1 pipe0

	{ MDP_DPU1_PIPE1_PIXEL_CLK, "dpu1_pipe1_pixel_clk",
		"1188m", MDP_DPUn_PIXEL_CLK(2),
		MDP_SS_CLK_MISC_DIV, MDP_SS_CLK_MISC_EN,
	},//DPU1 pipe1

	{ MDP_DPU2_PIPE0_PIXEL_CLK, "dpu2_pipe0_pixel_clk",
		"1188m", MDP_DPUn_PIXEL_CLK(3),
		MDP_SS_CLK_MISC_DIV, MDP_SS_CLK_MISC_EN, },//DPU2 pipe0

	{ MDP_DPU2_PIPE1_PIXEL_CLK, "dpu2_pipe1_pixel_clk",
		"1188m", MDP_DPUn_PIXEL_CLK(4),
		MDP_SS_CLK_MISC_DIV_4, MDP_SS_CLK_MISC_EN,
	},//DPU2 pipe1
};

static struct se1000_core_clk mdpclk_group_var[] = {
	{ MDP_DPU0_PIXEL_CLK_VAR, "dpu0_pixel_clk_var",
		"mdp_dynamic", MDP_DPUn_PIXEL_CLK(0),
		MDP_SS_CLK_MISC_DIV, MDP_SS_CLK_MISC_EN,
	},//DPU0

	{ MDP_DPU1_PIPE0_PIXEL_CLK_VAR, "dpu1_pipe0_pixel_clk_var",
		"mdp_dynamic", MDP_DPUn_PIXEL_CLK(1),
		MDP_SS_CLK_MISC_DIV, MDP_SS_CLK_MISC_EN,
	},//DPU1 pipe0

	{ MDP_DPU1_PIPE1_PIXEL_CLK_VAR, "dpu1_pipe1_pixel_clk_var",
		"mdp_dynamic", MDP_DPUn_PIXEL_CLK(2),
		MDP_SS_CLK_MISC_DIV, MDP_SS_CLK_MISC_EN,
	},//DPU1 pipe1

	{ MDP_DPU2_PIPE0_PIXEL_CLK_VAR, "dpu2_pipe0_pixel_clk_var",
		"mdp_dynamic", MDP_DPUn_PIXEL_CLK(3),
		MDP_SS_CLK_MISC_DIV, MDP_SS_CLK_MISC_EN,
	},//DPU2 pipe0

	{ MDP_DPU2_PIPE1_PIXEL_CLK_VAR, "dpu2_pipe1_pixel_clk_var",
		"mdp_dynamic", MDP_DPUn_PIXEL_CLK(4),
		MDP_SS_CLK_MISC_DIV_4, MDP_SS_CLK_MISC_EN,
	},//DPU2 pipe1
};


static struct se1000_core_clk audclk_group[] = {
	{ AUD_I2S0_CLK, "aud_i2s0_clk", "aud_pll_clk", AUD_I2Sn_PLL_CLK(0),
		AUD_SS_I2S_EDVCLK, AUD_SS_I2S_EDVCLC,
	},//I2S0

	{ AUD_I2S1_CLK, "aud_i2s1_clk", "aud_pll_clk", AUD_I2Sn_PLL_CLK(1),
		AUD_SS_I2S_EDVCLK, AUD_SS_I2S_EDVCLC,
	},//I2S1

	{ AUD_I2S2_CLK, "aud_i2s2_clk", "aud_pll_clk", AUD_I2Sn_PLL_CLK(2),
		AUD_SS_I2S_EDVCLK, AUD_SS_I2S_EDVCLC,
	},//I2S2

	{ AUD_I2S3_CLK, "aud_i2s3_clk", "aud_pll_clk", AUD_I2Sn_PLL_CLK(3),
		AUD_SS_I2S_EDVCLK, AUD_SS_I2S_EDVCLC,
	},//I2S3

	{ AUD_I2S4_CLK, "aud_i2s4_clk", "aud_pll_clk", AUD_I2Sn_PLL_CLK(4),
		AUD_SS_I2S_EDVCLK, AUD_SS_I2S_EDVCLC,
	},//I2S4

	{ AUD_I2S5_CLK, "aud_i2s5_clk", "aud_pll_clk", AUD_I2Sn_PLL_CLK(5),
		AUD_SS_I2S_EDVCLK, AUD_SS_I2S_EDVCLC,
	},//I2S5

	{ AUD_I2S6_CLK, "aud_i2s6_clk", "aud_pll_clk", AUD_I2Sn_PLL_CLK(6),
		AUD_SS_I2S_EDVCLK, AUD_SS_I2S_EDVCLC,
	},//I2S6

	{ AUD_SPDIF_CLK, "aud_spdif_clk", "aud_pll_clk", AUD_I2Sn_PLL_CLK(7),
		AUD_SS_I2S_EDVCLK, AUD_SS_I2S_EDVCLC,
	},//SPDIF
};

#define to_core_from_clk(p) container_of(p, struct clkrst_core, hw)

static int se1000_reset_assert(struct reset_controller_dev *rcdev,
			unsigned long id)
{
	struct clkrst_core *rstc = crg_ctrl_data.clkrst_data;
	unsigned long flags = 0;

	if (id >= MAX_CLK_DEV) {
		pr_err("%s: can't get the id=%lu clkrst info\n", __func__, id);
		return -1;
	}

	rstc += id;
	pr_devel("%s: id=%lu, rstc=%p\n", __func__, id, rstc);

	spin_lock_irqsave(&rstc->lock_rst, flags);

	if (rstc->rst_reg != NULL)
		writel(RST_ASSERT, rstc->rst_reg);

	spin_unlock_irqrestore(&rstc->lock_rst, flags);

	return 0;
}

static int se1000_reset_deassert(struct reset_controller_dev *rcdev,
			unsigned long id)
{
	struct clkrst_core *rstc = crg_ctrl_data.clkrst_data;
	unsigned long flags = 0;

	if (id >= MAX_CLK_DEV) {
		pr_err("%s: can't get the id=%lu clkrst info\n", __func__, id);
		return -1;
	}

	rstc += id;
	pr_devel("%s: id=%lu, rstc=%p\n", __func__, id, rstc);

	spin_lock_irqsave(&rstc->lock_rst, flags);

	if (rstc->rst_reg != NULL)
		writel(RST_DEASSERT, rstc->rst_reg);

	spin_unlock_irqrestore(&rstc->lock_rst, flags);

	return 0;
}

static int se1000_reset(struct reset_controller_dev *rcdev,
			unsigned long id)
{
	int err;

	err = se1000_reset_assert(rcdev, id);
	if (err)
		return err;

	udelay(1);

	return se1000_reset_deassert(rcdev, id);
}

void se_clk_unregister_fixed_rate(struct clk **clks)
{
	struct clk *clk;
	int i;

	for (i = 0; i < ARRAY_SIZE(clks_fixed_group); i++) {
		clk = clks[clks_fixed_group[i].id];
		if (clk)
			clk_unregister_fixed_rate(clk);
	}
}

void se_clk_unregister_core(struct clk **clks)
{
	struct clk *clk;
	int i;

	for (i = 0; i < ARRAY_SIZE(clks_group); i++) {
		clk = clks[clks_group[i].id];
		if (clk)
			clk_unregister(clk);
	}

	for (i = 0; i < ARRAY_SIZE(step_clks_group); i++) {
		clk = clks[step_clks_group[i].id];
		if (clk)
			clk_unregister(clk);
	}

	for (i = 0; i < ARRAY_SIZE(mdpclk_group); i++) {
		clk = clks[mdpclk_group[i].id];
		if (clk)
			clk_unregister(clk);
	}

	for (i = 0; i < ARRAY_SIZE(audclk_group); i++) {
		clk = clks[audclk_group[i].id];
		if (clk)
			clk_unregister(clk);
	}

	for (i = 0; i < ARRAY_SIZE(pll_clks_group); i++) {
		clk = clks[pll_clks_group[i].id];
		if (clk)
			clk_unregister(clk);
	}

	for (i = 0; i < ARRAY_SIZE(saf_clks_group); i++) {
		clk = clks[saf_clks_group[i].id];
		if (clk)
			clk_unregister(clk);
	}

	for (i = 0; i < ARRAY_SIZE(saf_pll_clks_group); i++) {
		clk = clks[saf_pll_clks_group[i].id];
		if (clk)
			clk_unregister(clk);
	}
}

static int clk_is_enabled(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	u32 enable_mask = BIT(CLK_GATE_POS);
	int ret = 0;

	if (corecrg->clk_reg != NULL)
		ret = !!(readl(corecrg->clk_reg) & enable_mask);
	else
		pr_debug("%s: %s clk_reg is NULL\n",
		 __func__, corecrg->clk_name);

	return ret;
}

static int mdp_clk_is_enabled(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	/* set mask of in-out clk enable bit (b11) for each dpu-n */
	u32 enable_mask = 0x3 << (corecrg->clk_flag * 2);
	int ret = 0;

	if (corecrg->rst_reg != NULL)
		ret = !!(readl(corecrg->rst_reg) & enable_mask);
	else
		pr_warn("%s: %s rst_reg is NULL\n",
			__func__, corecrg->clk_name);

	return ret;
}

static int aud_clk_is_enabled(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	u32 enable_mask = 0x1 << corecrg->clk_flag;
	int ret = 0;

	if (corecrg->clk_reg != NULL)
		ret = !!(readl(corecrg->clk_reg) & enable_mask);
	else
		pr_debug("%s: %s clk_reg is NULL\n",
			 __func__, corecrg->clk_name);

	return ret;
}

static int clk_enable(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 reg = 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->clk_reg != NULL) {
		reg = readl(corecrg->clk_reg);

		/* shirk: select PLL clk output */
		if ((corecrg->clk_flag & PLL_CLK_OUTPUT) != 0)
			reg |=  (BIT(CLK_SEL_POS));

		reg |= (BIT(CLK_GATE_POS));

		if ((corecrg->clk_flag & CLK_FIX_EN_DIS) == 0)
			writel(reg, corecrg->clk_reg);
	} else {
		pr_debug("%s: %s clk_reg is NULL\n",
			__func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);

	if (corecrg->clk_flag & RESET_YES) {
		if (corecrg->rst_reg != NULL) {
			spin_lock_irqsave(&corecrg->lock_rst, flags);
			writel(RST_DEASSERT, corecrg->rst_reg);
			spin_unlock_irqrestore(&corecrg->lock_rst, flags);
		} else {
			pr_warn("%s: reset not successfully\n", __func__);
		}
	}

	return 0;
}

static int mdp_clk_enable(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 reg = 0;

	/* set mask of in-out clk enable bit (b11) for each dpu-n */
	u32 enable_mask = 0x3 << (corecrg->clk_flag * 2);

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->rst_reg != NULL) {
		reg = readl(corecrg->rst_reg);
		reg |= enable_mask;
		writel(reg, corecrg->rst_reg);
	} else {
		pr_warn("%s: %s rst_reg is NULL\n",
			__func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);

	return 0;
}

static int aud_clk_enable(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 reg = 0;

	u32 enable_mask = 0x01 << corecrg->clk_flag;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->clk_reg != NULL) {
		reg = readl(corecrg->clk_reg);
		reg |= enable_mask;
		writel(reg, corecrg->clk_reg);
	} else {
		pr_warn("%s: %s clk_reg is NULL\n",
			__func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);

	return 0;
}

static void clk_disable(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 reg = 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->clk_reg != NULL) {
		reg = readl(corecrg->clk_reg);
		reg &= ~(BIT(CLK_GATE_POS));

		if ((corecrg->clk_flag & CLK_FIX_EN_DIS) == 0)
			writel(reg, corecrg->clk_reg);
	} else {
		pr_debug("%s: %s clk_reg is NULL\n",
			__func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);
}

static void mdp_clk_disable(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	u32 enable_mask = 0x3 << (corecrg->clk_flag * 2);
	unsigned long flags = 0;
	u32 reg = 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->rst_reg != NULL) {
		reg = readl(corecrg->rst_reg);
		reg &= ~(enable_mask);
		writel(reg, corecrg->rst_reg);
	} else {
		pr_warn("%s: %s rst_reg is NULL\n",
			__func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);
}

static void aud_clk_disable(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	u32 enable_mask = 0x1 << corecrg->clk_flag;
	unsigned long flags = 0;
	u32 reg = 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->clk_reg != NULL) {
		reg = readl(corecrg->clk_reg);
		reg &= ~(enable_mask);
		writel(reg, corecrg->clk_reg);
	} else {
		pr_warn("%s: %s clk_reg is NULL\n",
			__func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);
}

/* clksrc: 0 for reference clk output, 1 for PLL clk out */
static int clk_mux_set_parent(struct clk_hw *hwclk, u8 clksrc)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 reg = 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->clk_reg != NULL) {
		reg = readl(corecrg->clk_reg);
		reg |= (clksrc << CLK_SEL_POS);
		writel(reg, corecrg->clk_reg);
	} else {
		pr_warn("%s: %s clk_reg is NULL\n",
			__func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);

	return 0;
}

static int mdp_clk_mux_set_parent(struct clk_hw *hwclk, u8 clksrc)
{
	return 0;
}

static int aud_clk_mux_set_parent(struct clk_hw *hwclk, u8 clksrc)
{
	return 0;
}

static u8 clk_mux_get_parent(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 reg = 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->clk_reg != NULL) {
		reg = readl(corecrg->clk_reg);
		reg &= (BIT(CLK_SEL_POS));
		reg = reg >> CLK_SEL_POS;
	} else {
		pr_warn("%s: %s clk_reg is NULL\n",
			__func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);

	return reg;
}

static u8 mdp_clk_mux_get_parent(struct clk_hw *hwclk)
{
	return 0;
}

static u8 aud_clk_mux_get_parent(struct clk_hw *hwclk)
{
	return 0;
}

static int clk_set_rate(struct clk_hw *hwclk, unsigned long rate,
			    unsigned long parent_rate)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 reg = 0, div = 1;

	div = parent_rate / rate;
	pr_debug("%s: clk_name: %s, parent_rate = %ld, rate = %ld, div = %d\n",
		__func__, corecrg->clk_name, parent_rate, rate, div);

	div /= 2;

	spin_lock_irqsave(&corecrg->lock_clk, flags);

	/*
	 * Write new divider to the divider ratio register.
	 * It is safe to set rate(divider) directly,
	 * no extra glitch generated. Confirm with DE
	 */

	if (corecrg->clk_reg != NULL) {
		reg = readl(corecrg->clk_reg);
		reg &= ~(CLK_DIVIDER_MASK);
		reg |= (div & CLK_DIVIDER_MASK);
		writel(reg, corecrg->clk_reg);
	} else {
		pr_debug("%s: %s clk_reg is NULL\n",
			 __func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);

	return 0;
}

static int mdp_clk_set_rate(struct clk_hw *hwclk, unsigned long rate,
			    unsigned long parent_rate)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 reg = 0, div = 0, div_shift = 0;
	u32 div_mask = 0;

	if (corecrg->clk_flag != 4) {
		div_mask = 0xff << (corecrg->clk_flag * 8);
		div = parent_rate / rate;
		div_shift = div << (corecrg->clk_flag * 8);
	} else {
		div_mask = 0xff;
		div = (parent_rate / rate);
		div_shift = div;
	}

	pr_debug("%s: clk_name: %s, parent_rate = %ld, rate = %ld, div = %d, div_shift = 0x%x\n",
		__func__, corecrg->clk_name, parent_rate, rate, div, div_shift);

	spin_lock_irqsave(&corecrg->lock_clk, flags);

	/*
	 * Write new divider to the divider ratio register.
	 * It is safe to set rate(divider) directly,
	 * no extra glitch generated. Confirm with DE
	 */
	if (corecrg->clk_reg != NULL) {
		reg = readl(corecrg->clk_reg);
		reg &= ~(div_mask);
		reg |= (div_shift & div_mask);
		writel(reg, corecrg->clk_reg);
	} else {
		pr_debug("%s: %s clk_reg is NULL\n",
			__func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);

	return 0;
}

static int aud_clk_set_rate(struct clk_hw *hwclk, unsigned long rate,
			    unsigned long parent_rate)
{
	return 0;
}

static unsigned long clk_recalc_rate(struct clk_hw *hwclk,
		unsigned long parent_rate)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	u32 reg = 0, div = 1, divider_pow = 0;
	unsigned long long rate = 0;
	unsigned long flags = 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->clk_reg != NULL) {
		reg = readl(corecrg->clk_reg);
		reg &= (CLK_DIVIDER_MASK);
		divider_pow = (reg >> CLK_DIVIDER_POS);
	} else {
		pr_debug("%s: %s clk_reg is not set\n",
			__func__, corecrg->clk_name);
	}
	if (divider_pow == 0)
		div = 1;
	else
		div = 2 * divider_pow;

	rate = parent_rate;
	do_div(rate, div);

	pr_debug("%s: clk_name: %s, clock rate is %lu, div=%u\n",
		__func__, corecrg->clk_name, (unsigned long)rate, div);
	if (corecrg->clk_flag & CLK_HACK) {
		//clk_enable(hwclk);//enable perix apb
		rate = 1843200;
		pr_warn("hack the uart clk to %lu\n", (unsigned long)rate);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);
	return (unsigned long)rate;
}

static unsigned long mdp_clk_recalc_rate(struct clk_hw *hwclk,
		unsigned long parent_rate)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	u32 reg = 0, div = 1;
	unsigned long long rate = 1;
	/* set 8bits mask of clk div for each dpu-n */
	u32 div_mask = 0xff << (corecrg->clk_flag * 8);

	if (corecrg->clk_reg != NULL) {
		reg = readl(corecrg->clk_reg);
		reg &= (div_mask);
		div = (reg >> (corecrg->clk_flag * 8));
	} else {
		pr_warn("%s: %s clk_reg is NULL\n",
			__func__, corecrg->clk_name);
	}
	rate = parent_rate;
	do_div(rate, div);

	pr_debug("%s: clk_name: %s, clock rate is %lu, div=%u\n",
		__func__, corecrg->clk_name, (unsigned long)rate, div);

	return (unsigned long)rate;
}

static unsigned long aud_clk_recalc_rate(struct clk_hw *hwclk,
		unsigned long parent_rate)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	u32 div = 1;
	unsigned long long rate;

	div = 1;

	rate = parent_rate;
	do_div(rate, div);

	pr_debug("%s: clk_name: %s, clock rate is %lu, div = %u\n",
		__func__, corecrg->clk_name, (unsigned long)rate, div);

	return (unsigned long)rate;
}

static long clk_round_rate(struct clk_hw *hwclk, unsigned long rate,
		unsigned long *prate)
{
	return rate;
}

static long mdp_clk_round_rate(struct clk_hw *hwclk, unsigned long rate,
		unsigned long *prate)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	u32 div = 1;

	div = *prate / rate;
	pr_debug("%s: parent_rate = %ld, expect_rate = %ld, div = %d\n",
		__func__, *prate, rate, div);

	pr_debug("%s: clk_name: %s, real clock rate is %lu\n",
	 __func__, corecrg->clk_name, (unsigned long)*prate / div);

	return (*prate / div);
}

static long aud_clk_round_rate(struct clk_hw *hwclk, unsigned long rate,
		unsigned long *prate)
{
	return rate;
}

static int step_clk_is_enabled(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	u32 enable_mask = STEP_CLK_ON_MASK;
	int ret = 0;

	if (corecrg->pll_cfg2 != NULL)
		ret = !!(readl(corecrg->pll_cfg2) & enable_mask);

	return ret;
}

static int step_clk_enable(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 reg = 0, clk_reg = 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->pll_cfg2 != NULL) {
		/* select PLL clk output */
		clk_reg = readl(corecrg->clk_reg);
		if ((corecrg->clk_flag & PLL_CLK_OUTPUT))
			clk_reg |=  (BIT(CLK_SEL_POS));
		writel(clk_reg, corecrg->clk_reg);

		reg = readl(corecrg->pll_cfg2);
		reg |= STEP_CLK_ON_MASK;
		writel(reg, corecrg->pll_cfg2);
	} else {
		pr_debug("%s: %s pll_cfg2 is NULL\n",
			__func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);

	return 0;
}

static void step_clk_disable(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 reg = 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->pll_cfg2 != NULL) {
		reg = readl(corecrg->pll_cfg2);
		reg &= ~(STEP_CLK_ON_MASK);
		writel(reg, corecrg->pll_cfg2);
	} else {
		pr_debug("%s: %s pll_cfg2 is NULL\n",
			__func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);
}

static int step_clk_mux_set_parent(struct clk_hw *hwclk, u8 clksrc)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 reg = 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->clk_reg != NULL) {
		reg = readl(corecrg->clk_reg);
		reg |= (clksrc << CLK_SEL_POS);
		writel(reg, corecrg->clk_reg);
	} else {
		pr_warn("%s: %s clk_reg is NULL\n",
			__func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);
	return 0;
}

static u8 step_clk_mux_get_parent(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 reg = 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->clk_reg != NULL) {
		reg = readl(corecrg->clk_reg);
		reg &= (BIT(CLK_SEL_POS));
		reg = reg >> CLK_SEL_POS;
	} else {
		pr_warn("%s: %s clk_reg is NULL\n",
			__func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);

	return reg;
}

static void step_clk_get_step_num(struct clk_hw *hwclk,
			 unsigned long parent_rate, u32 *step_num, u32 *max_step)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	u32 fix_step_num = STEP_NUM(parent_rate);
	u32 fix_max_step = MAX_STEP(parent_rate);

	if ((strncmp(corecrg->clk_name, "step_cpu0", 9) == 0) ||
			 (strncmp(corecrg->clk_name, "step_cpu1", 9) == 0)) {
		fix_step_num = LCPU_STEP_NUM(parent_rate);
		fix_max_step = LCPU_MAX_STEP(parent_rate);
	} else if ((strncmp(corecrg->clk_name, "step_cpu2", 9) == 0) ||
				(strncmp(corecrg->clk_name, "step_cpu3", 9) == 0) ||
				(strncmp(corecrg->clk_name, "step_cpu4", 9) == 0) ||
				(strncmp(corecrg->clk_name, "step_cpu5", 9) == 0)) {
		fix_step_num = BCPU_STEP_NUM(parent_rate);
		fix_max_step = BCPU_MAX_STEP(parent_rate);
	} else {
		fix_step_num = STEP_NUM(parent_rate);
		fix_max_step = MAX_STEP(parent_rate);
	}
	*step_num = fix_step_num;
	*max_step = fix_max_step;

	return;
}

static int step_clk_set_rate(struct clk_hw *hwclk, unsigned long rate,
			    unsigned long parent_rate)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 val = 0, reg = 0;
	u32 step_num = 0, max_step = 0;
	u32 fix_step_num = STEP_NUM(parent_rate);
	u32 fix_max_step = MAX_STEP(parent_rate);

	if ((rate <= 0) || (parent_rate <= 0))
		return 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);

	if ((corecrg->pll_cfg0 != NULL) && (corecrg->pll_cfg1 != NULL)) {
		val = readl(corecrg->pll_cfg2);
		reg = readl(corecrg->pll_cfg0);
		step_num = reg & 0xFF;
		if ((val & STEP_BYPASS_ON) || (step_num == 0)) {
			step_clk_get_step_num(hwclk, parent_rate,
				&fix_step_num, &fix_max_step);
			if (corecrg->clk_reg != NULL) {
				/*select divided clock*/
				reg = readl(corecrg->clk_reg);
				reg |=  (BIT(CLK_SEL_POS));
				writel(reg, corecrg->clk_reg);
			}
			writel(STEP_BYPASS_ON, corecrg->pll_cfg2);/*bypass on/ clk off = 0*/
			reg = (STEP_LEN << 8) | fix_step_num;
			writel(reg, corecrg->pll_cfg0);
			writel(fix_max_step, corecrg->pll_cfg1);
			writel(STEP_BYPASS_OFF | STEP_CLK_ON_MASK, corecrg->pll_cfg2);/*bypass off, step clk on = 1*/
		}

		reg = readl(corecrg->pll_cfg0);
		step_num = reg & 0xFF;
		max_step = (rate * step_num) / parent_rate;

		writel(max_step, corecrg->pll_cfg1);

		writel(STEP_CLK_CHANGE, corecrg->pll_cfg3);
	}

	spin_unlock_irqrestore(&corecrg->lock_clk, flags);

	return 0;
}

static unsigned long step_clk_recalc_rate(struct clk_hw *hwclk,
		unsigned long parent_rate)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	u32 reg = 0, step_len = 1, step_num = 1, max_step = 1;
	unsigned long long rate = parent_rate;
	unsigned long flags = 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if ((corecrg->pll_cfg0 != NULL) && (corecrg->pll_cfg1 != NULL)) {
		reg = readl(corecrg->pll_cfg0);
		step_len = (reg >> 8) & 0xFF;
		step_num = reg & 0xFF;
		reg = readl(corecrg->pll_cfg1);
		max_step = reg & 0xFF;
	}
	if (step_num)
		rate = (parent_rate * max_step) / step_num;
	else
		rate = parent_rate;

	//pr_debug("%s: clk_name: %s, clock rate is %lu, max_step = %d, step_num = %d\n",
	//	__func__, corecrg->clk_name, (unsigned long)rate, max_step, step_num);

	spin_unlock_irqrestore(&corecrg->lock_clk, flags);

	return (unsigned long)rate;
}

const struct clk_ops step_se1000_ops = {
	.enable      = step_clk_enable,
	.disable     = step_clk_disable,
	.is_enabled  = step_clk_is_enabled,
	.set_rate    = step_clk_set_rate,
	.get_parent  = step_clk_mux_get_parent,
	.set_parent  = step_clk_mux_set_parent,
	.recalc_rate = step_clk_recalc_rate,
	.round_rate  = clk_round_rate,
};

const struct clk_ops se1000_ops = {
	.enable      = clk_enable,
	.disable     = clk_disable,
	.is_enabled  = clk_is_enabled,
	.set_rate    = clk_set_rate,
	.get_parent  = clk_mux_get_parent,
	.set_parent  = clk_mux_set_parent,
	.recalc_rate = clk_recalc_rate,
	.round_rate  = clk_round_rate,
};

const struct clk_ops se1000_mdp_ops = {
	.enable      = mdp_clk_enable,
	.disable     = mdp_clk_disable,
	.is_enabled  = mdp_clk_is_enabled,
	.set_rate    = mdp_clk_set_rate,
	.get_parent  = mdp_clk_mux_get_parent,
	.set_parent  = mdp_clk_mux_set_parent,
	.recalc_rate = mdp_clk_recalc_rate,
	.round_rate  = mdp_clk_round_rate,
};

const struct clk_ops se1000_aud_ops = {
	.enable      = aud_clk_enable,
	.disable     = aud_clk_disable,
	.is_enabled  = aud_clk_is_enabled,
	.set_rate    = aud_clk_set_rate,
	.get_parent  = aud_clk_mux_get_parent,
	.set_parent  = aud_clk_mux_set_parent,
	.recalc_rate = aud_clk_recalc_rate,
	.round_rate  = aud_clk_round_rate,
};

static int pll_clk_enable(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 reg = 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->pll_cfg4 != NULL) {
		reg = readl(corecrg->pll_cfg4);
		reg |= PLLEN;
		writel(reg, corecrg->pll_cfg4);
	} else {
		pr_debug("%s: %s pll_cfg4 is NULL\n",
			 __func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);

	return 0;
}

static void pll_clk_disable(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 reg = 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->pll_cfg4 != NULL) {
		reg = readl(corecrg->pll_cfg4);
		reg &= ~(PLLEN);
		if (corecrg->clk_flag & RESET_YES)
			writel(reg, corecrg->pll_cfg4);
	} else {
		pr_debug("%s: %s pll_cfg4 is NULL\n",
			__func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);
}

static int pll_clk_is_enabled(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	int ret = 0;

	if (corecrg->pll_cfg4 != NULL)
		ret = !!(readl(corecrg->pll_cfg4) & PLLEN);
	else
		pr_debug("%s: %s pll_cfg4 is NULL\n",
			__func__, corecrg->clk_name);

	return ret;
}

/*
 * Wait for the pll to reach the locked state.
 * The calling set_rate function is responsible for making sure the
 * hwclk is available.
 */
static int pll_wait_lock(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	int delay = 24000000, reg_value = 0;

	if (corecrg->pll_cfg5 == NULL)
		return 0;

	while (delay > 0) {
		reg_value = readl(corecrg->pll_cfg5);
		if (reg_value & PLLLOCK)
			return 0;
		delay--;
	}

	pr_err("%s: timeout waiting for pll to lock\n", __func__);
	return -ETIMEDOUT;
}

static int pll_clk_set_rate(struct clk_hw *hwclk,
		 unsigned long rate, unsigned long parent_rate)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	int refdiv = 1;
	int fbdiv = 0;
	u64 frac = 0;
	int postdiv1 = 0;
	int postdiv2 = 0;
	int refclk = parent_rate;
	unsigned long fvco = 0;
	u32 reg_value  = 0;
	u32 foutpostdiv = 0;
	u32 fout4phase = 0;
	int ret = 0;

	pr_debug("%s: %s: rate = %lu, parent_rate = %lu\n",
		__func__, corecrg->clk_name, rate, parent_rate);

	if (corecrg->pll_cfg1 == NULL)
		return 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->clk_flag & RESET_YES) {
		/*entire pll power down*/
		writel(0, corecrg->pll_cfg4);
		reg_value = readl(corecrg->pll_cfg3);
		postdiv1 = reg_value & POSTDIV1_MASK;
		postdiv2 = (reg_value & POSTDIV2_MASK) >> POSTDIV2_SHIFT;

		reg_value = readl(corecrg->pll_cfg0);
		refdiv = reg_value & REFDIV_MASK;
		foutpostdiv = (reg_value & FOUTPOSTDIVEN);
		fout4phase = (reg_value & FOUT4HASEEN);

		if (corecrg->pll_fout4phase >= FOUT1PH0) {
			if (fout4phase)
				rate = rate * corecrg->pll_fout4phase;
		}

		if (strncmp(corecrg->clk_name, "pcie_pll_out", 12) == 0)/*pcie pll*/
			fvco = rate * (postdiv1 + 1) * (postdiv2 + 1) * 4 * refdiv;
		else
			fvco = rate * postdiv1 * postdiv2 * refdiv;

		if (fvco > (3200000000)) {
			pr_info("%s: %s: fvco = %lu\n",
				__func__, corecrg->clk_name, fvco);
			spin_unlock_irqrestore(&corecrg->lock_clk, flags);
			return -1;
		}

		if (0 == ((fvco) % refclk)) {
			fbdiv = (fvco) / refclk;
		} else {
			/* Frac takes the decimal point, multiplying it by 2^24,
			 * and rounding off the whole number frac[23:0] 24bit
			 */
			fbdiv = (fvco) / refclk;
			frac = (fvco) % refclk;

			frac = frac * (1 << 24);//2^24

			frac = frac / refclk;
			frac = frac & FRAC_MASK;

		}
		fbdiv = fbdiv & FBDIV_MASK;
		reg_value = readl(corecrg->pll_cfg0);
		refdiv = reg_value & REFDIV_MASK;
		reg_value = reg_value | PLL_DSMEN | PLL_DACEN;
		writel(reg_value, corecrg->pll_cfg0);
		writel(fbdiv, corecrg->pll_cfg1);
		writel(frac, corecrg->pll_cfg2);

		/*enable entire pll */
		writel(PLLEN, corecrg->pll_cfg4);

		/* Wait until the PLL is locked if it is enabled. */
		ret = pll_wait_lock(hwclk);
		if (ret)
			pr_warn("%s: pll update unsuccessful\n", __func__);

		pr_debug("%s: clk: %s, rate = %lu, refdiv = %d, fbdiv = %d, postdiv1 = %d, postdiv2 = %d, frac = %llu\n",
		__func__, corecrg->clk_name, rate, refdiv, fbdiv, postdiv1, postdiv2, frac);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);

	return 0;
}

static unsigned long pll_clk_recalc_rate(struct clk_hw *hwclk,
		unsigned long parent_rate)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	int refdiv = 0;
	u64 frac = 0;
	int fbdiv = 0;
	int postdiv1 = 0;
	int postdiv2 = 0;
	u32 reg_value  = 0;
	u32 foutpostdiv = 0;
	u32 fout4phase = 0;
	unsigned long fraction = 0;
	unsigned long long rate;

	if (corecrg->pll_cfg1 == NULL) {
		reg_value = readl(corecrg->pll_cfg0);
		foutpostdiv = reg_value & FOUTPOSTDIVEN;
		fout4phase = reg_value & FOUT4HASEEN;
		if (corecrg->pll_fout4phase >= FOUT1PH0) {
			if (fout4phase)
				parent_rate = parent_rate / corecrg->pll_fout4phase;
		}
		return parent_rate;
	}

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	reg_value = readl(corecrg->pll_cfg0);
	refdiv = reg_value & (REFDIV_MASK);
	foutpostdiv = reg_value & FOUTPOSTDIVEN;
	fout4phase = reg_value & FOUT4HASEEN;
	reg_value = readl(corecrg->pll_cfg1);
	fbdiv = reg_value & (FBDIV_MASK);

	reg_value = readl(corecrg->pll_cfg2);
	frac = reg_value & (FRAC_MASK);

	reg_value = readl(corecrg->pll_cfg3);
	postdiv1 = reg_value & POSTDIV1_MASK;
	postdiv2 = (reg_value & POSTDIV2_MASK) >> POSTDIV2_SHIFT;

	rate = parent_rate;
	if (((rate * frac) % (1 << 24)) == 0)
		fraction = (rate * frac) / (1 << 24);
	else
		fraction = (rate * frac) / (1 << 24) + 1;

	if (strncmp(corecrg->clk_name, "pcie_pll_out", 12) == 0)/*pcie pll*/
		rate = ((rate * fbdiv) +  fraction) / (refdiv * (postdiv1 + 1) * (postdiv2 + 1) * 4);
	else
		rate = ((rate * fbdiv) +  fraction) / (refdiv * postdiv1 * postdiv2);

	if (corecrg->pll_fout4phase >= FOUT1PH0) {
		if (fout4phase)
			rate = rate / corecrg->pll_fout4phase;
	}

	pr_debug("%s: clk: %s, recalc_rate = %llu, refdiv = %d, fbdiv = %d, postdiv1 = %d, postdiv2 = %d, frac = %llu\n",
		__func__, corecrg->clk_name, rate, refdiv, fbdiv, postdiv1, postdiv2, frac);
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);
	return (unsigned long)rate;
}

static u8 pll_clk_mux_get_parent(struct clk_hw *hwclk)
{
	return 0;
}

static int pll_clk_mux_set_parent(struct clk_hw *hwclk, u8 clksrc)
{
	return 0;
}

static long pll_clk_round_rate(struct clk_hw *hwclk,
		 unsigned long rate, unsigned long *prate)
{
	return rate;
}

const struct clk_ops se1000_pll_ops = {
	.enable      = pll_clk_enable,
	.disable     = pll_clk_disable,
	.is_enabled  = pll_clk_is_enabled,
	.set_rate    = pll_clk_set_rate,
	.recalc_rate = pll_clk_recalc_rate,
	.get_parent  = pll_clk_mux_get_parent,
	.set_parent  = pll_clk_mux_set_parent,
	.round_rate  = pll_clk_round_rate,
};

static int saf_pll_clk_set_rate(struct clk_hw *hwclk,
		 unsigned long rate, unsigned long parent_rate)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	int refdiv = 1;
	int fbdiv = 0;
	u64 frac = 0;
	int postdiv1 = 0;
	int postdiv2 = 0;
	int refclk = parent_rate;
	unsigned long fvco = 0;
	u32 reg_value  = 0;
	u32 foutpostdiv = 0;
	u32 fout4phase = 0;

	pr_debug("%s: %s: rate = %lu, parent_rate = %lu\n",
		__func__, corecrg->clk_name, rate, parent_rate);

	if (corecrg->pll_cfg1 == NULL)
		return 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->clk_flag & RESET_YES) {
		reg_value = readl(corecrg->pll_cfg4);
		refdiv = (reg_value & SAF_REFDIV_MASK) >> SAF_REFDIV_SHIFT;
		postdiv1 = (reg_value & SAF_POSTDIV1_MASK) >> SAF_POSTDIV1_SHIFT;
		postdiv2 = (reg_value & SAF_POSTDIV2_MASK) >> SAF_POSTDIV2_SHIFT;
		foutpostdiv = (reg_value & SAF_FOUTPOSTDIVEN);
		fout4phase = (reg_value & SAF_FOUT4HASEEN);
		/*entire pll power down*/
		reg_value &= ~SAF_PLLEN;
		writel(reg_value, corecrg->pll_cfg4);

		if (corecrg->pll_fout4phase >= FOUT1PH0) {
			if (fout4phase)
				rate = rate * corecrg->pll_fout4phase;
		}

		fvco = rate * postdiv1 * postdiv2 * refdiv;
		if (fvco > (3200000000)) {
			pr_info("%s: %s: fvco = %lu\n",
				__func__, corecrg->clk_name, fvco);
			spin_unlock_irqrestore(&corecrg->lock_clk, flags);
			return -1;
		}

		if (0 == ((fvco) % refclk)) {
			fbdiv = (fvco) / refclk;
		} else {
			/* Frac takes the decimal point, multiplying it by 2^24,
			 * and rounding off the whole number frac[23:0] 24bit
			 */
			fbdiv = (fvco) / refclk;
			frac = (fvco) % refclk;
			frac = frac * (1 << 24);//2^24
			frac = frac / refclk;
			frac = frac & FRAC_MASK;
		}
		fbdiv = fbdiv & FBDIV_MASK;
		writel(fbdiv, corecrg->pll_cfg1);
		//frac = frac | SAF_PLL_DSMEN | SAF_PLL_DACEN;
		writel(frac, corecrg->pll_cfg2);

		reg_value = readl(corecrg->pll_cfg4);
		reg_value |= (refdiv << SAF_REFDIV_SHIFT)
			   | (postdiv1 << SAF_POSTDIV1_SHIFT)
			   | (postdiv2 << SAF_POSTDIV2_SHIFT);
		writel(reg_value, corecrg->pll_cfg4);

		/* enable entire pll */
		reg_value |= SAF_PLLEN;
		writel(reg_value, corecrg->pll_cfg4);

		pr_debug("%s: clk: %s, rate = %lu, refdiv = %d, fbdiv = %d, postdiv1 = %d, postdiv2 = %d, frac = %llu\n",
		__func__, corecrg->clk_name, rate, refdiv, fbdiv, postdiv1, postdiv2, frac);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);

	return 0;
}

static unsigned long saf_pll_clk_recalc_rate(struct clk_hw *hwclk,
		unsigned long parent_rate)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	int refdiv = 0;
	u64 frac = 0;
	int fbdiv = 0;
	int postdiv1 = 0;
	int postdiv2 = 0;
	u32 reg_value  = 0;
	u32 foutpostdiv = 0;
	u32 fout4phase = 0;
	unsigned long fraction = 0;
	unsigned long long rate;

	if (corecrg->pll_cfg1 == NULL) {
		reg_value = readl(corecrg->pll_cfg0);
		foutpostdiv = (reg_value & SAF_FOUTPOSTDIVEN);
		fout4phase = (reg_value & SAF_FOUT4HASEEN);
		if (corecrg->pll_fout4phase >= FOUT1PH0) {
			if (fout4phase)
				parent_rate = parent_rate / corecrg->pll_fout4phase;
		}
		return parent_rate;
	}

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	reg_value = readl(corecrg->pll_cfg4);
	refdiv = (reg_value & SAF_REFDIV_MASK) >> SAF_REFDIV_SHIFT;
	postdiv1 = (reg_value & SAF_POSTDIV1_MASK) >> SAF_POSTDIV1_SHIFT;
	postdiv2 = (reg_value & SAF_POSTDIV2_MASK) >> SAF_POSTDIV2_SHIFT;
	foutpostdiv = (reg_value & SAF_FOUTPOSTDIVEN);
	fout4phase = (reg_value & SAF_FOUT4HASEEN);

	reg_value = readl(corecrg->pll_cfg1);
	fbdiv = reg_value & (FBDIV_MASK);

	reg_value = readl(corecrg->pll_cfg2);
	frac = reg_value & (FRAC_MASK);

	rate = parent_rate;

	fraction = ((rate * frac) + (1 << 24) - 1) / (1 << 24);

	rate = ((rate * fbdiv) + fraction) / (refdiv * postdiv1 * postdiv2);

	if (corecrg->pll_fout4phase >= FOUT1PH0) {
		if (fout4phase)
			rate = rate / corecrg->pll_fout4phase;
	}

	pr_debug("%s: clk: %s, recalc_rate = %llu, refdiv = %d, fbdiv = %d, postdiv1 = %d, postdiv2 = %d, frac = %llu\n",
		__func__, corecrg->clk_name, rate, refdiv, fbdiv, postdiv1, postdiv2, frac);
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);
	return (unsigned long)rate;
}

const struct clk_ops se1000_saf_pll_ops = {
	.enable      = pll_clk_enable,
	.disable     = pll_clk_disable,
	.is_enabled  = pll_clk_is_enabled,
	.set_rate    = saf_pll_clk_set_rate,
	.recalc_rate = saf_pll_clk_recalc_rate,
	.get_parent  = pll_clk_mux_get_parent,
	.set_parent  = pll_clk_mux_set_parent,
	.round_rate  = pll_clk_round_rate,
};

static int saf_clk_enable(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 reg = 0;
	u32 enable_mask = corecrg->clk_flag & SAF_DEVCKEN_MASK;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->clk_reg != NULL) {
		if (corecrg->clk_flag != 0) {
			reg = readl(corecrg->clk_reg);
			reg |= enable_mask;
			writel(reg, corecrg->clk_reg);
		}
	} else {
		pr_debug("%s: %s clk_reg is NULL\n",
			__func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);

	return 0;
}

static void saf_clk_disable(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 reg = 0;
	u32 enable_mask = corecrg->clk_flag & SAF_DEVCKEN_MASK;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->clk_reg != NULL) {
		if (corecrg->clk_flag != 0) {
			reg = readl(corecrg->clk_reg);
			reg &= ~(enable_mask);
			writel(reg, corecrg->clk_reg);
		}
	} else {
		pr_debug("%s: %s clk_reg is NULL\n",
			__func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);
}

static int saf_clk_is_enabled(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	u32 enable_mask = corecrg->clk_flag & SAF_DEVCKEN_MASK;
	int ret = 0;

	if (corecrg->clk_reg != NULL) {
		if (corecrg->clk_flag != 0)
			ret = !!(readl(corecrg->clk_reg) & enable_mask);
	} else {
		pr_debug("%s: %s clk_reg is NULL\n",
			__func__, corecrg->clk_name);
	}
	return ret;
}

static int saf_clk_set_rate(struct clk_hw *hwclk,
		 unsigned long rate, unsigned long parent_rate)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 reg = 0, div = 1, divider_pow = 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->clk_flag == 0) {
		if (strncmp(corecrg->clk_name, "saf_bus_clk", 11) == 0) {
			spin_unlock_irqrestore(&corecrg->lock_clk, flags);
			return 0;
		}
		/*only SAF CLKSEL use*/
		div = parent_rate / rate;
		pr_info("%s: clk_name: %s, parent_rate = %ld, rate = %ld, div = %d\n",
			__func__, corecrg->clk_name, parent_rate, rate, div);
		if (div == 4)
			divider_pow = 0;
		else if (div == 2)
			divider_pow = 1;
		else
			divider_pow = 0;
		/*
		 * Write new divider to the divider ratio register.
		 * It is safe to set rate(divider) directly,
		 * no extra glitch generated. Confirm with DE
		 */
		if (corecrg->clk_reg != NULL) {
			reg = readl(corecrg->clk_reg);
			reg &= ~(SAF_CLKSEL_DIV_MASK);
			reg |= ((div << SAF_CLKSEL_DIV_SHIFT) & SAF_CLKSEL_DIV_MASK);
			writel(reg, corecrg->clk_reg);
		}
	} else {
		if (strncmp(corecrg->clk_name, "saf_qspi_ref", 12) == 0) {/*qspi_ref_clk*/
			div = parent_rate / rate;
			div = div / 2;
			if (corecrg->pll_cfg0 != NULL) {
				reg = readl(corecrg->pll_cfg0);
				reg &= ~(SAF_QSPI_REF_DIV_MASK);
				reg |= ((div << SAF_QSPI_REF_DIV_SHIFT) & SAF_QSPI_REF_DIV_MASK);
				writel(reg, corecrg->pll_cfg0);
			}
		}
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);

	return 0;
}

static unsigned long saf_clk_recalc_rate(struct clk_hw *hwclk,
		unsigned long parent_rate)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long long rate = 0;
	unsigned long flags = 0;
	u32 reg = 0, div = 1, divider_pow = 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);

	if (corecrg->clk_reg != NULL) {
		if (strncmp(corecrg->clk_name, "saf_bus_clk", 11) == 0) {
			rate = parent_rate / 2;
			spin_unlock_irqrestore(&corecrg->lock_clk, flags);
			return (unsigned long)rate;
		}
		/*only SAF CLKSEL use*/
		if (corecrg->clk_flag == 0) {
			if (corecrg->clk_reg != NULL)
				reg = readl(corecrg->clk_reg);
			reg &= (SAF_CLKSEL_DIV_MASK);
			divider_pow = (reg >> SAF_CLKSEL_DIV_SHIFT);
			if (divider_pow == 0)
				div = 4;
			else
				div = 2;
		} else {
			if (strncmp(corecrg->clk_name, "saf_qspi_ref", 12) == 0) {/*qspi_ref_clk*/
				if (corecrg->pll_cfg0 != NULL)
					reg = readl(corecrg->pll_cfg0);

				reg &= (SAF_QSPI_REF_DIV_MASK);
				divider_pow = (reg >> SAF_QSPI_REF_DIV_SHIFT);
				if (divider_pow == 0)
					div = 1;
				else
					div = 2 * divider_pow;
			} else {
				div = 1;
			}
		}
	}
	rate = parent_rate / div;
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);

	return (unsigned long)rate;
}

static u8 saf_clk_mux_get_parent(struct clk_hw *hwclk)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 reg = 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->clk_reg != NULL) {
		/*only SAF CLKSEL use*/
		if (corecrg->clk_flag == 0) {
			reg = readl(corecrg->clk_reg);
			reg &= (BIT(0));
			reg = reg >> 0;
		} else {
			if (strncmp(corecrg->clk_name, "saf_sadp", 8) == 0) {
				reg = readl(corecrg->clk_reg);
				if (corecrg->clk_flag == BIT(1))
					reg = reg >> 4;
				else if (corecrg->clk_flag == BIT(2))
					reg = reg >> 6;
				else if (corecrg->clk_flag == BIT(3))
					reg = reg >> 8;
				else
					reg = 0;
				reg = (reg & 0x3);
			}
		}
	} else {
		pr_warn("%s: %s clk_reg is NULL\n",
			__func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);

	return reg;
}

static int saf_clk_mux_set_parent(struct clk_hw *hwclk, u8 clksrc)
{
	struct clkrst_core *corecrg = to_core_from_clk(hwclk);
	unsigned long flags = 0;
	u32 reg = 0;

	spin_lock_irqsave(&corecrg->lock_clk, flags);
	if (corecrg->clk_reg != NULL) {
		/*only SAF CLKSEL use*/
		if (corecrg->clk_flag == 0) {
			reg = readl(corecrg->clk_reg);
			reg |= ((clksrc & 0x1) << 0);
			writel(reg, corecrg->clk_reg);
		} else {
			if (strncmp(corecrg->clk_name, "saf_sadp", 8) == 0) {
				reg = readl(corecrg->clk_reg);
				if (corecrg->clk_flag == BIT(1))
					reg |= ((clksrc & 0x3) << 4);
				else if (corecrg->clk_flag == BIT(2))
					reg |= ((clksrc & 0x3) << 6);
				else if (corecrg->clk_flag == BIT(3))
					reg |= ((clksrc & 0x3) << 8);
				else
					reg |= 0;
				writel(reg, corecrg->clk_reg);
			}
		}
	} else {
		pr_warn("%s: %s clk_reg is NULL\n",
			__func__, corecrg->clk_name);
	}
	spin_unlock_irqrestore(&corecrg->lock_clk, flags);

	return 0;
}

static long saf_clk_round_rate(struct clk_hw *hwclk,
		 unsigned long rate, unsigned long *prate)
{
	return rate;
}

const struct clk_ops se1000_saf_ops = {
	.enable      = saf_clk_enable,
	.disable     = saf_clk_disable,
	.is_enabled  = saf_clk_is_enabled,
	.set_rate    = saf_clk_set_rate,
	.recalc_rate = saf_clk_recalc_rate,
	.get_parent  = saf_clk_mux_get_parent,
	.set_parent  = saf_clk_mux_set_parent,
	.round_rate  = saf_clk_round_rate,
};

static const struct reset_control_ops se1000_reset_ops = {
	.assert		= se1000_reset_assert,
	.deassert	= se1000_reset_deassert,
	.reset		= se1000_reset,
};

static void __init
se1000_rst_init(struct device_node *node)
{
	crg_ctrl_data.rcdev.of_node = node;
	crg_ctrl_data.rcdev.owner = THIS_MODULE;
	crg_ctrl_data.rcdev.ops = &se1000_reset_ops;
	crg_ctrl_data.rcdev.of_node = node;
	crg_ctrl_data.rcdev.of_reset_n_cells = 1;
	crg_ctrl_data.rcdev.nr_resets = MAX_CLK_DEV;
	reset_controller_register(&crg_ctrl_data.rcdev);
}

#define CONFIG_CLK_CELLS(base_addr, clk_ops, config_grp) do {\
	spin_lock_init(&corecrg[config_grp[i].id].lock_clk);\
	spin_lock_init(&corecrg[config_grp[i].id].lock_rst);\
	\
	init.num_parents = 1;\
	init.parent_names = &config_grp[i].parent_name;\
	init.name = config_grp[i].clk_name;\
	init.ops = &clk_ops;\
	init.flags = 0;\
	\
	if (config_grp[i].clkaddr_offset >= 0)\
		corecrg[config_grp[i].id].clk_reg = base_addr\
		+ config_grp[i].clkaddr_offset;\
	\
	if (config_grp[i].pll_cfg0 >= 0)\
		corecrg[config_grp[i].id].pll_cfg0 = base_addr\
		+ config_grp[i].pll_cfg0;\
	\
	if (config_grp[i].pll_cfg1 > 0)\
		corecrg[config_grp[i].id].pll_cfg1 = base_addr\
		+ config_grp[i].pll_cfg1;\
	\
	if (config_grp[i].pll_cfg2 > 0)\
		corecrg[config_grp[i].id].pll_cfg2 = base_addr\
		+ config_grp[i].pll_cfg2;\
	\
	if (config_grp[i].pll_cfg3 > 0)\
		corecrg[config_grp[i].id].pll_cfg3 = base_addr\
		+ config_grp[i].pll_cfg3;\
	\
	if (config_grp[i].pll_cfg4 > 0)\
		corecrg[config_grp[i].id].pll_cfg4 = base_addr\
		+ config_grp[i].pll_cfg4;\
	\
	if (config_grp[i].pll_cfg5 > 0)\
		corecrg[config_grp[i].id].pll_cfg5 = base_addr\
		+ config_grp[i].pll_cfg5;\
	\
	corecrg[config_grp[i].id].clk_name = config_grp[i].clk_name;\
	corecrg[config_grp[i].id].clk_flag = config_grp[i].flag;\
	corecrg[config_grp[i].id].pll_fout4phase = config_grp[i].pll_fout4phase;\
	corecrg[config_grp[i].id].hw.init = &init;\
	\
	if (config_grp[i].rstaddr_offset > 0)\
		corecrg[config_grp[i].id].rst_reg = base_addr +\
		config_grp[i].rstaddr_offset;\
	\
	clks[config_grp[i].id] = clk_register(NULL,\
		&corecrg[config_grp[i].id].hw);\
	if (IS_ERR(clks[config_grp[i].id]))\
		goto err_core_clk;\
} while (0)


void pll_calc_handler(unsigned int fout, struct pll_div_regconfig *pll_reg)
{
	unsigned int base_freq = 25; //MHZ
	unsigned int min_div1 = 1, min_div2 = 1;
	unsigned int fractpart = 0;
	unsigned int div1 = 1, div2 = 1;
	unsigned int tmp_fractpart = 0;
	unsigned int min_fractval = 0;

	fractpart = fout % base_freq;
	min_fractval = fractpart;
	for (div1 = 1; div1 < 8; div1++) {
		for (div2 = 1; div2 < 8; div2++) {
			tmp_fractpart = fractpart * div1 * div2 - fractpart * div1 * div2 / base_freq * base_freq;
			if ((min_fractval > tmp_fractpart) &&
				(fout * div1 * div2 / base_freq < (1 << 12)) &&
				(fout * div1 * div2 < 3200)) {
				min_fractval = tmp_fractpart;
				min_div1 = div1;
				min_div2 = div2;
			}
		}
	}
	pll_reg->fbdiv = fout * min_div1 * min_div2 / base_freq;
	pll_reg->frac = min_fractval * (1 << 24) / base_freq;
	pll_reg->postdiv1 = min_div1;
	pll_reg->postdiv2 = min_div2;

}

bool pll_cfg_check(void __iomem *crg_base, struct pll_div_regconfig *pll_reg)
{
	u32 cur_pll_cfg3,cur_pll_cfg2,cur_pll_cfg1;
	u32 real_pll_cfg3,real_pll_cfg2,real_pll_cfg1;

	real_pll_cfg3 = (pll_reg->postdiv2 << 4) + pll_reg->postdiv1;
	real_pll_cfg1 = pll_reg->fbdiv;
	real_pll_cfg2 = pll_reg->frac;

	cur_pll_cfg3 = readl(crg_base + MDP_PLL0_CFG3);
	cur_pll_cfg2 = readl(crg_base + MDP_PLL0_CFG2);
	cur_pll_cfg1 = readl(crg_base + MDP_PLL0_CFG1);

	if((cur_pll_cfg3 != real_pll_cfg3) || (cur_pll_cfg2 != real_pll_cfg2) || (cur_pll_cfg1 != real_pll_cfg1)){
		pr_info("%s,cur_cfg3=%d,real_cfg3=%d,cur_cfg2=%d,real_cfg2=%d,cur_cfg1=%d,real_cfg1=%d\r\n",
				__func__,cur_pll_cfg3,real_pll_cfg3,cur_pll_cfg2,real_pll_cfg2,cur_pll_cfg1,real_pll_cfg1);
		return true;
	}
	else
		return false;
}


void pll_cfg_dp(void __iomem *crg_base, struct pll_div_regconfig *pll_reg)
{
	if(pll_cfg_check(crg_base, pll_reg)){
		/*entire pll power down*/
		writel(0, crg_base + MDP_PLL0_CFG4);//PLL0_PLLCFG4

		writel((pll_reg->postdiv2 << 4) + pll_reg->postdiv1, crg_base + MDP_PLL0_CFG3);//PLL0_PLLCFG3
		writel(pll_reg->fbdiv, crg_base + MDP_PLL0_CFG1);//PLL0_PLLCFG1
		writel(pll_reg->frac, crg_base + MDP_PLL0_CFG2);//PLL0_PLLCFG2

		writel(1, crg_base + MDP_PLL0_CFG4);//PLL0_PLLCFG4
	}
}

static int mdp_clkout_suspend(void)
{
	return 0;
}

static void mdp_clkout_resume(void)
{
	pll_calc_handler(mdp_clk->pixel_rate, mdp_clk->pll_reg);
	pll_cfg_dp(mdp_clk->crg_base, mdp_clk->pll_reg);
}

static struct syscore_ops mdp_clkout_syscore_ops = {
	.suspend = mdp_clkout_suspend,
	.resume = mdp_clkout_resume,
};



static bool __init
se1000_clk_init(struct device_node *node, void __iomem *crg_base,
		void __iomem *mdp_base, void __iomem *aud_base,
		void __iomem *saf_pll_base, struct clkrst_core *corecrg)
{
	struct clk_init_data init;
	struct clk **clks;
	struct clk *clk;
	int i;

	struct property *prop;
	unsigned int pixel_rate;
	struct pll_div_regconfig *pll_reg = NULL;

	/* clks holds the clock array */
	clks = kcalloc(clk_data.clk_num, sizeof(struct clk *),
				GFP_KERNEL);
	if (WARN_ON(!clks))
		return false;

	//siengine,mdp_dividers
	prop = of_find_property(node, "siengine,mdp_dp_pll", NULL);

	if (prop) {
		pll_reg = kzalloc(sizeof(struct pll_div_regconfig), GFP_KERNEL);
		if (WARN_ON(!pll_reg)) {
			pll_reg = NULL;
			goto err_unmap;
		}
		of_property_read_u32_index(node, "siengine,mdp_dp_pll", 0, &pixel_rate);
		pll_calc_handler(pixel_rate, pll_reg);
		pll_cfg_dp(crg_base, pll_reg);

		mdp_clk = kzalloc(sizeof(struct mdp_clkout), GFP_KERNEL);
		mdp_clk->pll_reg = pll_reg;
		mdp_clk->pixel_rate = pixel_rate;
		mdp_clk->crg_base = crg_base;
		register_syscore_ops(&mdp_clkout_syscore_ops);
	}

	/* register fixed clk */
	for (i = 0; i < ARRAY_SIZE(clks_fixed_group); i++) {
		if (prop) {
			if (clks_fixed_group[i].id == FIXED_PIXEL)
				clks_fixed_group[i].fixed_rate = pixel_rate * 1000000;
		}

		clk = clk_register_fixed_rate(NULL, clks_fixed_group[i].clk_name,
					      clks_fixed_group[i].parent_name,
					      clks_fixed_group[i].flags,
					      clks_fixed_group[i].fixed_rate);
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register fixed clock %s\n",
			       __func__, clks_fixed_group[i].clk_name);
			goto err_fixed_clk;
		}
		clks[clks_fixed_group[i].id] = clk;
	}

	/* register pll related clks */
	for (i = 0; i < ARRAY_SIZE(pll_clks_group); i++)
		CONFIG_CLK_CELLS(crg_base, se1000_pll_ops, pll_clks_group);

	for (i = 0; i < ARRAY_SIZE(saf_pll_clks_group); i++)
		CONFIG_CLK_CELLS(saf_pll_base,
			 se1000_saf_pll_ops, saf_pll_clks_group);

	for (i = 0; i < ARRAY_SIZE(saf_clks_group); i++)
		CONFIG_CLK_CELLS(saf_pll_base,
			 se1000_saf_ops, saf_clks_group);

	/* register core clk, gate/mux/divider */
	for (i = 0; i < ARRAY_SIZE(clks_group); i++)
		CONFIG_CLK_CELLS(crg_base, se1000_ops, clks_group);

	for (i = 0; i < ARRAY_SIZE(step_clks_group); i++)
		CONFIG_CLK_CELLS(crg_base, step_se1000_ops, step_clks_group);

	if (prop) {
		/* register mdp 150m related clks */
		for (i = 0; i < ARRAY_SIZE(mdpclk_group_var); i++)
			CONFIG_CLK_CELLS(mdp_base, se1000_mdp_ops, mdpclk_group_var);
	} else {
		/* register mdp related clks */
		for (i = 0; i < ARRAY_SIZE(mdpclk_group); i++)
			CONFIG_CLK_CELLS(mdp_base, se1000_mdp_ops, mdpclk_group);
	}

	/* register aud related clks */
	for (i = 0; i < ARRAY_SIZE(audclk_group); i++)
		CONFIG_CLK_CELLS(aud_base, se1000_aud_ops, audclk_group);

	clk_data.clks = clks;
	of_clk_add_provider(node, of_clk_src_onecell_get, &clk_data);

	return true;
err_core_clk:
	se_clk_unregister_core(clks);
err_fixed_clk:
	se_clk_unregister_fixed_rate(clks);
	if (pll_reg != NULL)
		kfree(pll_reg);
err_unmap:
	kfree(clks);



	return false;
}

static void __init
se1000_clkrst_init(struct device_node *node)
{
	void __iomem *crg_base, *mdp_base, *aud_base;
	void __iomem *saf_pll_base = NULL;
	struct clkrst_core *corecrg;

	crg_base = of_iomap(node, 0);
	mdp_base = of_iomap(node, 1);
	aud_base = of_iomap(node, 2);
	saf_pll_base = of_iomap(node, 3);

	if (WARN_ON((!crg_base) || (!mdp_base) || (!aud_base) || (!saf_pll_base)))
		return;

	clk_data.clk_num = MAX_CLK_DEV;
	/* corediv holds the clock specific array */
	corecrg = kcalloc(clk_data.clk_num, sizeof(struct clkrst_core),
				GFP_KERNEL);
	if (WARN_ON(!corecrg))
		goto err_unmap;

	crg_ctrl_data.clkrst_data = corecrg;

	if (false == se1000_clk_init(node, crg_base, mdp_base, aud_base, saf_pll_base, corecrg)) {
		pr_err("%s: failed to init core clock\n", __func__);
		kfree(corecrg);
		goto err_unmap;
	}

	se1000_rst_init(node);
	pr_info("%s: init success\n", __func__);

	return;
	/* how to get a change to unmap io when clkrst init failed */

err_unmap:
	iounmap(crg_base);
	iounmap(mdp_base);
	iounmap(aud_base);
	iounmap(saf_pll_base);
}

CLK_OF_DECLARE(se1000_clkrst, "siengine,se1000-crg-clock",
			se1000_clkrst_init);
