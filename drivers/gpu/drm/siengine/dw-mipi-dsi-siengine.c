// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) Wuhan SiEngine Co.Ltd
 * Author:
 *		Xiaohu Qian <xiaohu.qian@siengine.com>
 */
#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/component.h>

#include <video/mipi_display.h>

#include <drm/drm_encoder.h>
#include <drm/drm_crtc.h>
#include <drm/drm_print.h>
#include <drm/drm_atomic_helper.h>
#include <drm/bridge/dw_mipi_dsi.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>

#define MDP_MISC_IVI_REQ	0x214
#define MDP_MISC_RVC_REQ	0x218


#define MDP_MISC_DPU0_VTOTAL	0x21c
#define MDP_MISC_DPU0_HTOTAL	0x220

/* MDP_MISC_REG */
#define DPHY_PLL_CLKSEL			   0x1c0
#define DPHY_PLL_SHADOW_CONTROL	   0x1c8
#define DPHY_TEST_SW			   0x204
#define DPHY_CFGCLKFREQRANGE_DPHY0 0x1f4
#define DPHY_CFGCLKFREQRANGE_DPHY1 0x1f8
#define DPHY_HSFREQRANGE_DPHY0 0x1B0
#define DPHY_HSFREQRANGE_DPHY1 0x1B4
#define DPHY_PLL_N 0x1B8
#define DPHY_PLL_M 0x1BC
#define DPHY_PLL_VCO_CNTRL 0x1D4


#define MSB_DPHY_TESTCODE(a)	((a & 0xf00) >> 8)
#define LSB_DPHY_TESTCODE(a)	(a & 0xff)

#define DPHY_TX_SYS_0	0x001
#define DPHY_TX_SYS_1	0x002
#define DPHY_TX_DUAL_PHY_0 0x133
#define DPHY_TX_PLL_25	0x176
#define DPHY_TX_PLL_31	0x17c
#define DPHY_TX_CB2		0x1ac
#define DPHY_TX_CB3		0x1ad
#define DPHY_TX_SLEW_0	0x26b
#define DPHY_TX_SLEW_7	0x272

#define DPHY2TXTESTER_DIG_RDWR_TX_PLL_9			0x166
#define DPHY2TXTESTER_DIG_RDWR_TX_PLL_22		0x173
#define DPHY2TXTESTER_DIG_RDWR_TX_PLL_23		0x174

#define DPHY2TXTESTER_DIG_RDWR_TX_PLL_27	0x178
#define DPHY2TXTESTER_DIG_RDWR_TX_PLL_28	0x179
#define DPHY2TXTESTER_DIG_RDWR_TX_PLL_29	0x17A
#define DPHY2TXTESTER_DIG_RDWR_TX_PLL_30	0x17B
#define DPHY2TXTESTER_DIG_RDWR_TX_SYSTIMERS_22	0x64
#define DPHY2TXTESTER_DIG_RDWR_TX_SYSTIMERS_17	0x5f
#define DPHY2TXTESTER_DIG_RDWR_TX_SYSTIMERS_14	0x5c

#define DPHY2TXTESTER_DIG_RDWR_TX_SLEW_5 0x270
#define DPHY2TXTESTER_DIG_RDWR_TX_SLEW_6 0x271
#define DPHY2TXTESTER_DIG_RDWR_TX_SLEW_7 0x272

#define PLL_OVR_RW_M_EN BIT(0)
#define PLL_OVR_RW_VCO_CTRL_EN BIT(7)
#define PLL_OVR_RW_VCO_CTRL(val) (((val) & 0x3f) << 1)
#define PLL_OVR_RW_M_7_0(val) ((val) & 0xff)
#define PLL_OVR_RW_M_11_8(val) ((val >> 8) & 0xf)
#define PLL_TSTPLLDIG_RW_2_0 0
#define PLL_OVR_RW_N_EN BIT(7)
#define PLL_OVR_RW_N_3_0(val) (((val) & 0xf) << 3)

#define DSI_PHY_TST_CTRL0		0xb4
#define PHY_TESTCLK			BIT(1)
#define PHY_UNTESTCLK			0
#define PHY_TESTCLR			BIT(0)
#define PHY_UNTESTCLR			0

#define DSI_PHY_TST_CTRL1		0xb8
#define PHY_TESTEN			BIT(16)
#define PHY_UNTESTEN			0
#define PHY_TESTDOUT(n)			(((n) & 0xff) << 8)
#define PHY_TESTDIN(n)			((n) & 0xff)

/* If the pll clock is not configured on the r52 side, you need to define this macro switch*/
#define CONFIG_SADP_INSTANCE 1


/*
 *
 */
struct dw_mipi_dsi_se {
	struct device *dev;
	struct drm_encoder encoder;
	void __iomem *base;

	struct clk *pllref_clk;
	struct clk *phy_cfg_clk;

	unsigned int lane_mbps; /* per lane */
	unsigned int hsfreqrange;
	u32 format;

	struct dw_mipi_dsi *dmd;
	struct dw_mipi_dsi_plat_data pdata;

	struct regmap *misc_base;

	unsigned mdp_flags ;
	u8 input_div;/*N*/
	u16 feedback_div;/*M*/
	u8 vco_cntrl;/**/
	u8 factor_p;/*P*/
};
#define to_dsi(nm)	container_of(nm, struct dw_mipi_dsi_se, nm)


struct dphy_pll_parameter_map {
	unsigned int max_mbps;/*max_mbps:table value is max_mbps * 1000*/
	u8 vco_cntrl;
	u8 factor_p;
};

/* The table is based on 24MHz DPHY pll reference clock. */
static const struct dphy_pll_parameter_map dppa_map[] = {
	{ 46400, 0x2b, 64 },
	{ 64000, 0x28, 64 },
	{ 76870, 0x27, 32 },
	{ 92900, 0x23, 32 },
	{ 125120, 0x20, 32 },
	{ 153750, 0x1f, 16 },
	{ 185780, 0x1b, 16 },
	{ 256250, 0x18, 16 },
	{ 307500, 0x17, 8 },
	{ 371500, 0x13, 8 },
	{ 484400, 0x10, 8 },
	{ 512500, 0x10, 8 },
	{ 615000, 0x0f, 4 },
	{ 743125, 0x0b, 4 },
	{ 1025000, 0x08, 4 },
	{ 1230000, 0x07, 2 },
	{ 1486260, 0x03, 2 },
	{ 2250000, 0x0, 2 },
};


static int max_mbps_to_parameter(unsigned int max_mbps)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dppa_map); i++)
		if (dppa_map[i].max_mbps >= (max_mbps / 2 * 1000))
			return i;

	return -EINVAL;
}

static int dw_mipi_dsi_get_vco_ctrl(struct dw_mipi_dsi_se *host, unsigned int lane_mbps)
{
	int i;
	i = max_mbps_to_parameter(lane_mbps);
	if (i < 0) {
		DRM_DEV_ERROR(host->dev, "failed to get parameter for %dmbps clock\n",
		lane_mbps);
		return i;
	}

	host->vco_cntrl = dppa_map[i].vco_cntrl;

	return 0 ;
}


static inline void dsi_write(struct dw_mipi_dsi_se *dsi, u32 reg, u32 val)
{
	writel(val, dsi->base + reg);
}

static inline u32 dsi_read(struct dw_mipi_dsi_se *dsi, u32 reg)
{
	return readl(dsi->base + reg);
}

static inline void dsi_set(struct dw_mipi_dsi_se *dsi, u32 reg, u32 mask)
{
	dsi_write(dsi, reg, dsi_read(dsi, reg) | mask);
}

static inline void dsi_update_bits(struct dw_mipi_dsi_se *dsi, u32 reg,
				   u32 mask, u32 val)
{
	dsi_write(dsi, reg, (dsi_read(dsi, reg) & ~mask) | val);
}

static void dsi_clk_enable(struct  dw_mipi_dsi_se *dsi, bool enable)
{
	if (enable) {
		if (dsi->pllref_clk)
		clk_prepare_enable(dsi->pllref_clk);

		clk_prepare_enable(dsi->phy_cfg_clk);
	} else {
		if (dsi->pllref_clk)
		clk_disable_unprepare(dsi->pllref_clk);

		clk_disable_unprepare(dsi->phy_cfg_clk);
	}
}

static void dw_mipi_dsi_dphy_psel(struct dw_mipi_dsi_se *dsi,
								  u8 sel)
{
	/*ensure testclk and testen is set to low*/
	dsi_write(dsi, DSI_PHY_TST_CTRL1, PHY_UNTESTEN);
	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_UNTESTCLK);

	/*write 4bdsiestcode MSB*/
	dsi_write(dsi, DSI_PHY_TST_CTRL1, PHY_TESTEN);

	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_TESTCLK);
	// dsi_wridsiost, DSI_PHY_TST_CTRL0, PHY_UNTESTCLR );//sunpan databook

	/*place 0xdsin testdin*/
	dsi_write(dsi, DSI_PHY_TST_CTRL1, PHY_TESTEN | 0);
	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_UNTESTCLK);
	dsi_write(dsi, DSI_PHY_TST_CTRL1, PHY_UNTESTEN);

	/*write 8bdsiSB to testdin*/
	dsi_write(dsi, DSI_PHY_TST_CTRL1, sel);//PHY_UNTESTEN | sel);//sunpan databook

	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_TESTCLK);
	// dsi_write(host, DSI_PHY_TST_CTRL0, PHY_UNTESTCLR);
}

static void dw_mipi_dsi_phy_write(struct dw_mipi_dsi_se *dsi, u16 test_code,
								  u8 test_data)
{
	/*
	* With the falling edge on TESTCLK, the TESTDIN[7:0] signal content
	* is latched internally as the current test code. Test data is
	* programmed internally by rising edge on TESTCLK.
	*/
	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_UNTESTCLK);
	dsi_write(dsi, DSI_PHY_TST_CTRL1, PHY_TESTEN | LSB_DPHY_TESTCODE(test_code));//sunpan databook
	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_TESTCLK);
	// dsi_write(dsi, DSI_PHY_TST_CTRL1, PHY_TESTEN | test_code );///////////////////////
	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_UNTESTCLK);
	dsi_write(dsi, DSI_PHY_TST_CTRL1, PHY_UNTESTEN | PHY_TESTDIN(test_data));

	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_TESTCLK);//
}


static int dw_mipi_dsi_phy_read(struct dw_mipi_dsi_se *dsi, u16 test_code)
{
	u32 ret;

	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_UNTESTCLK);
	dsi_write(dsi, DSI_PHY_TST_CTRL1, PHY_TESTEN | LSB_DPHY_TESTCODE(test_code));//sunpan databook
	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_TESTCLK);
	// dsi_write(dsi, DSI_PHY_TST_CTRL1, PHY_TESTEN | test_code );///////////////////////
	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_UNTESTCLK);
	dsi_write(dsi, DSI_PHY_TST_CTRL1, PHY_UNTESTEN);

	//dsi_write(dsi, DSI_PHY_TST_CTRL1, 0x00);

	//dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_TESTCLK);

	ret = (dsi_read(dsi, DSI_PHY_TST_CTRL1) >> 8) & 0xff;
	//dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_UNTESTCLK);
	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_TESTCLK);

	printk("test_code %x  value %x", test_code, ret);

	return ret;
}


static void mdp_misc_reg_set(struct dw_mipi_dsi_se *dsi, int offset, int value)
{
	regmap_write(dsi->misc_base, offset, value);
}


static int dw_mipi_dsi_phy_read_test(struct dw_mipi_dsi_se *host, int cs)
{
	uint32_t val = 0;
	unsigned int count = 10;
	uint32_t i = 0;
	/* configure u0_dphy */
	mdp_misc_reg_set(host, DPHY_TEST_SW, cs);

	while (count) {
		udelay(5);
		count--;
	}

	/* configure u0_dphy */
	dw_mipi_dsi_dphy_psel(host, 0x1);

	dw_mipi_dsi_phy_read(host, DPHY_TX_PLL_31);
	dw_mipi_dsi_phy_read(host, DPHY2TXTESTER_DIG_RDWR_TX_PLL_9);
	dw_mipi_dsi_phy_read(host, DPHY2TXTESTER_DIG_RDWR_TX_PLL_22);
	dw_mipi_dsi_phy_read(host, DPHY2TXTESTER_DIG_RDWR_TX_PLL_23);

	/*int_cntrl*/
	dw_mipi_dsi_phy_read(host, 0x162);
	/*prop_cntrl*/
	dw_mipi_dsi_phy_read(host, 0x16e);
	/*cb_vref_mpll_reg_rel*/
	dw_mipi_dsi_phy_read(host, 0x1ad);


	val = dw_mipi_dsi_phy_read(host, 0x133);
	// val = dw_mipi_dsi_phy_read(host, 0x148);
	for (i = 0; i < 32; i++) {
		val = dw_mipi_dsi_phy_read(host, 0x15d + i);
	}

	val = dw_mipi_dsi_phy_read(host, 0x191);
	val = dw_mipi_dsi_phy_read(host, 0x192);
	val = dw_mipi_dsi_phy_read(host, 0x193);
	val = dw_mipi_dsi_phy_read(host, 0x194);
	val = dw_mipi_dsi_phy_read(host, 0x195);

	for (i = 0; i < 7; i++) {
		val = dw_mipi_dsi_phy_read(host, 0x1aa + i);
	}

	for (i = 0; i < 4; i++) {
		val = dw_mipi_dsi_phy_read(host, 0x1c4 + i);
	}

	dw_mipi_dsi_dphy_psel(host, 0x0);

	dw_mipi_dsi_phy_read(host, DPHY_TX_SYS_0);
	dw_mipi_dsi_phy_read(host, DPHY_TX_SYS_1);
	// val = dw_mipi_dsi_phy_read(host, 0x1f);
	// val = dw_mipi_dsi_phy_read(host, 0x2);
	val = dw_mipi_dsi_phy_read(host, 0x1e);
	val = dw_mipi_dsi_phy_read(host, 0x9c);

	dw_mipi_dsi_dphy_psel(host, 0x3);
	val = dw_mipi_dsi_phy_read(host, 0x31a);
	val = dw_mipi_dsi_phy_read(host, 0x31b);
	val = dw_mipi_dsi_phy_read(host, 0x31c);
	val = dw_mipi_dsi_phy_read(host, 0x31d);
	val = dw_mipi_dsi_phy_read(host, 0x32b);
	val = dw_mipi_dsi_phy_read(host, 0x32c);
	val = dw_mipi_dsi_phy_read(host, 0x354);
	val = dw_mipi_dsi_phy_read(host, 0x554);
	val = dw_mipi_dsi_phy_read(host, 0x355);
	val = dw_mipi_dsi_phy_read(host, 0x356);
	val = dw_mipi_dsi_phy_read(host, 0x357);

	dw_mipi_dsi_dphy_psel(host, 0x2);
	val = dw_mipi_dsi_phy_read(host, 0x209);
	val = dw_mipi_dsi_phy_read(host, 0x20a);
	val = dw_mipi_dsi_phy_read(host, 0x220);
	val = dw_mipi_dsi_phy_read(host, 0x221);
	val = dw_mipi_dsi_phy_read(host, 0x222);
	val = dw_mipi_dsi_phy_read(host, 0x280);

	dw_mipi_dsi_phy_read(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_5);
	dw_mipi_dsi_phy_read(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_6);
	dw_mipi_dsi_phy_read(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_7);

	return 0;
}



static void dphy_misc_config(struct dw_mipi_dsi_se *dsi)
{
	if (dsi->mdp_flags) {
		//void __iomem *addr =	dsi->misc_base;
		regmap_write(dsi->misc_base, DPHY_PLL_SHADOW_CONTROL, 1);
		regmap_write(dsi->misc_base, DPHY_PLL_CLKSEL, 1);

		regmap_write(dsi->misc_base, DPHY_CFGCLKFREQRANGE_DPHY0, 0x20);
		regmap_write(dsi->misc_base, DPHY_CFGCLKFREQRANGE_DPHY1, 0x20);
	} else {
		regmap_write(dsi->misc_base, 0x7c, 1);

		regmap_write(dsi->misc_base, 0x34, 1);
		regmap_write(dsi->misc_base, 0x2c, 1);

		regmap_write(dsi->misc_base, 0x8, 0x2020);
	}
}

static int dmd_se_phy_init(void *priv_data)
{
	int i;
	struct dw_mipi_dsi_se *host = priv_data;

	dphy_misc_config(host);
	i = max_mbps_to_parameter(host->lane_mbps);

	if (i < 0) {
		DRM_DEV_ERROR(host->dev,
				 "failed to get parameter for %dmbps clock\n",
				host->lane_mbps);
		return i;
	}

	if (host->mdp_flags) {
		/* configure u0_dphy */
		mdp_misc_reg_set(host, DPHY_TEST_SW, 0);

		dw_mipi_dsi_dphy_psel(host, 0x0);
		dw_mipi_dsi_phy_write(host, DPHY_TX_SYS_0, 0x20);
		dw_mipi_dsi_phy_write(host, DPHY_TX_SYS_1, host->hsfreqrange);

		dw_mipi_dsi_dphy_psel(host, 0x2);
		// dw_mipi_dsi_phy_write(host, DPHY_TX_SLEW_0, 0x04);
		// dw_mipi_dsi_phy_write(host, DPHY_TX_SLEW_7, 0x00);
		if (host->lane_mbps >= 1500) {
			dw_mipi_dsi_phy_write(host, 0x26b, 0x4);
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_7, 0x0);
		} else if ((host->lane_mbps > 1000) && (host->lane_mbps < 1500)) {
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_5, 0x98);
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_6, 0x3);
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_7, 0x0);
		} else {
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_5, 0x91);
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_6, 0x2);
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_7, 0x1);
		}

		dw_mipi_dsi_dphy_psel(host, 0x1);
		dw_mipi_dsi_phy_write(host, DPHY_TX_CB3, 0x02);
		// dw_mipi_dsi_phy_write(host, DPHY_TX_CB2, 0x10);
		dw_mipi_dsi_phy_write(host, DPHY_TX_PLL_31, 0x3);
		dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_PLL_9, 0x04);
		dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_PLL_22, 0x03);
		dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_PLL_23, 0x0);

		/*databook fixed value*/
		/*int_cntrl*/
		dw_mipi_dsi_phy_write(host, 0x162, 0x20);
		/*prop_cntrl*/
		dw_mipi_dsi_phy_write(host, 0x16e, 0xa);
		/*cb_vref_mpll_reg_rel*/
		dw_mipi_dsi_phy_write(host, DPHY_TX_CB3, 0x2);
		dw_mipi_dsi_phy_write(host, DPHY_TX_CB2, 0x1b);

		if (host->vco_cntrl > 0) {
		/*VCO_CTRL bit0:ovr_rw_m_en bit[6:1]:ovr_rw_vco_ctrl bit7:ovr_rw_vco_cntrl_en */
		dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_PLL_30, PLL_OVR_RW_M_EN
				| PLL_OVR_RW_VCO_CTRL(host->vco_cntrl) | PLL_OVR_RW_VCO_CTRL_EN);
		}

		if (host->feedback_div > 0) {
		/*bit[7:0] pll_m_ovr_rw__7__0__*/
		dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_PLL_28,
				PLL_OVR_RW_M_7_0(host->feedback_div));
		/*bit[3:0]: pll_m_ovr_rw__11__8__*/
		dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_PLL_29,
				PLL_OVR_RW_M_11_8(host->feedback_div));
		}

		if (host->input_div > 0) {
		/* bit[2:0] pll_tstplldig_rw__2__0__  bit[6:3]
		 * pll_n_ovr_rw__3__0__ bit[7]pll_n_ovr_en_rw N=n+1*/
		dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_PLL_27, PLL_TSTPLLDIG_RW_2_0
				| PLL_OVR_RW_N_3_0(host->input_div - 1) | PLL_OVR_RW_N_EN);
		}

		DRM_DEV_DEBUG(host->dev, "mbps =%d, range= 0x%x, p= 0x%x, m=%d, n=%d\n",
				host->lane_mbps, host->hsfreqrange,
				 host->vco_cntrl, host->feedback_div, host->input_div);

		/* configure u1_dphy */
		mdp_misc_reg_set(host, DPHY_TEST_SW, 1);
		dsi_write(host, DSI_PHY_TST_CTRL0, PHY_UNTESTCLR);
		dsi_write(host, DSI_PHY_TST_CTRL0, PHY_TESTCLR);
		//udelay(1);//sunpan databook wait for 15ns
		dsi_write(host, DSI_PHY_TST_CTRL0, PHY_UNTESTCLR);

		dw_mipi_dsi_dphy_psel(host, 0x0);
		dw_mipi_dsi_phy_write(host, DPHY_TX_SYS_0, 0x20);
		dw_mipi_dsi_phy_write(host, DPHY_TX_SYS_1, host->hsfreqrange);

		dw_mipi_dsi_dphy_psel(host, 0x2);
		// dw_mipi_dsi_phy_write(host, DPHY_TX_SLEW_0, 0x04);
		// dw_mipi_dsi_phy_write(host, DPHY_TX_SLEW_7, 0x00);
		if (host->lane_mbps >= 1500) {
			dw_mipi_dsi_phy_write(host, 0x26b, 0x4);
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_7, 0x0);
		} else if ((host->lane_mbps > 1000) && (host->lane_mbps < 1500)) {
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_5, 0x98);
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_6, 0x3);
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_7, 0x0);
		} else {
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_5, 0x91);
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_6, 0x2);
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_7, 0x1);
		}

		dw_mipi_dsi_dphy_psel(host, 0x1);
		dw_mipi_dsi_phy_write(host, DPHY_TX_CB3, 0x02);
		dw_mipi_dsi_phy_write(host, DPHY_TX_CB2, 0x1b);
		dw_mipi_dsi_phy_write(host, DPHY_TX_DUAL_PHY_0, 0x0);
	} else {
		/* configure u0_dphy */
		mdp_misc_reg_set(host, 0x68, 0);

		dw_mipi_dsi_dphy_psel(host, 0x0);
		dw_mipi_dsi_phy_write(host, DPHY_TX_SYS_0, 0x20);
		dw_mipi_dsi_phy_write(host, DPHY_TX_SYS_1, host->hsfreqrange);

		dw_mipi_dsi_dphy_psel(host, 0x2);
		// dw_mipi_dsi_phy_write(host, DPHY_TX_SLEW_0, 0x04);
		// dw_mipi_dsi_phy_write(host, DPHY_TX_SLEW_7, 0x00);
		if (host->lane_mbps >= 1500) {
			dw_mipi_dsi_phy_write(host, 0x26b, 0x4);
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_7, 0x0);
		} else if ((host->lane_mbps > 1000) && (host->lane_mbps < 1500)) {
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_5, 0x91);
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_6, 0x2);
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_7, 0x1);
		} else {
			  dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_5, 0x98);
			  dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_6, 0x3);
			  dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_7, 0x0);
		}

		dw_mipi_dsi_dphy_psel(host, 0x1);
		dw_mipi_dsi_phy_write(host, DPHY_TX_CB3, 0x02);
		// dw_mipi_dsi_phy_write(host, DPHY_TX_CB2, 0x10);
		dw_mipi_dsi_phy_write(host, DPHY_TX_PLL_31, 0x3);
		dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_PLL_9, 0x04);
		dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_PLL_22, 0x03);
		dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_PLL_23, 0x0);

		/*databook fixed value*/
		/*int_cntrl*/
		dw_mipi_dsi_phy_write(host, 0x162, 0x20);
		/*prop_cntrl*/
		dw_mipi_dsi_phy_write(host, 0x16e, 0xa);
		/*cb_vref_mpll_reg_rel*/
		dw_mipi_dsi_phy_write(host, DPHY_TX_CB3, 0x2);
		dw_mipi_dsi_phy_write(host, DPHY_TX_CB2, 0x1b);

		if (host->vco_cntrl > 0) {
		/*VCO_CTRL bit0:ovr_rw_m_en bit[6:1]:ovr_rw_vco_ctrl bit7:ovr_rw_vco_cntrl_en */
		dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_PLL_30, PLL_OVR_RW_M_EN | PLL_OVR_RW_VCO_CTRL(host->vco_cntrl) | PLL_OVR_RW_VCO_CTRL_EN);
		}

		if (host->feedback_div > 0) {
		/*bit[7:0] pll_m_ovr_rw__7__0__*/
		dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_PLL_28, PLL_OVR_RW_M_7_0(host->feedback_div));
		/*bit[3:0]: pll_m_ovr_rw__11__8__*/
		dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_PLL_29, PLL_OVR_RW_M_11_8(host->feedback_div));
		}

		if (host->input_div > 0) {
		/* bit[2:0] pll_tstplldig_rw__2__0__  bit[6:3] pll_n_ovr_rw__3__0__ bit[7]pll_n_ovr_en_rw N=n+1*/
		dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_PLL_27, PLL_TSTPLLDIG_RW_2_0 | PLL_OVR_RW_N_3_0(host->input_div - 1) | PLL_OVR_RW_N_EN);
		}

		DRM_DEV_DEBUG(host->dev, "mbps =%d, range= 0x%x, p= 0x%x, m=%d, n=%d\n", host->lane_mbps, host->hsfreqrange,
		host->vco_cntrl, host->feedback_div, host->input_div);

		/* configure u1_dphy */
		mdp_misc_reg_set(host, 0x68, 1);

		dsi_write(host, DSI_PHY_TST_CTRL0, PHY_UNTESTCLR);
		dsi_write(host, DSI_PHY_TST_CTRL0, PHY_TESTCLR);
		//udelay(1);//sunpan databook wait for 15ns
		dsi_write(host, DSI_PHY_TST_CTRL0, PHY_UNTESTCLR);

		dw_mipi_dsi_dphy_psel(host, 0x0);
		dw_mipi_dsi_phy_write(host, DPHY_TX_SYS_0, 0x20);
		dw_mipi_dsi_phy_write(host, DPHY_TX_SYS_1, host->hsfreqrange);

		dw_mipi_dsi_dphy_psel(host, 0x2);
		// dw_mipi_dsi_phy_write(host, DPHY_TX_SLEW_0, 0x04);
		// dw_mipi_dsi_phy_write(host, DPHY_TX_SLEW_7, 0x00);
		if (host->lane_mbps >= 1500) {
			dw_mipi_dsi_phy_write(host, 0x26b, 0x4);
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_7, 0x0);
		} else if ((host->lane_mbps > 1000) && (host->lane_mbps < 1500)) {
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_5, 0x98);
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_6, 0x3);
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_7, 0x0);
		} else {
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_5, 0x91);
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_6, 0x2);
			dw_mipi_dsi_phy_write(host, DPHY2TXTESTER_DIG_RDWR_TX_SLEW_7, 0x1);
		}

		dw_mipi_dsi_dphy_psel(host, 0x1);
		dw_mipi_dsi_phy_write(host, DPHY_TX_CB3, 0x02);
		dw_mipi_dsi_phy_write(host, DPHY_TX_CB2, 0x1b);
		dw_mipi_dsi_phy_write(host, DPHY_TX_DUAL_PHY_0, 0x0);
	}

	return 0;
}

struct hstt {
	unsigned int maxfreq;
	u8 hsfreqrange;
	struct dw_mipi_dsi_dphy_timing timing;
};

#define HSTT(_maxfreq, _hsfreqrange, _c_lp2hs, _c_hs2lp, _d_lp2hs, _dc_lp2hs, _d_hs2lp)	\
{					\
	.maxfreq = _maxfreq,		\
	.hsfreqrange = _hsfreqrange,		\
	.timing = {			\
	.clk_lp2hs = _c_lp2hs,	\
	.clk_hs2lp = _c_hs2lp,	\
	.data_lp2hs = _d_lp2hs,	\
	.dataclk_lp2hs = _dc_lp2hs,	\
	.data_hs2lp = _d_hs2lp,	\
	}				\
}

/* Table A-3 High-Speed Transition Times */
static struct hstt hstt_table[] = {
	HSTT(80, 0x00, 21, 17, 15, 35, 10),
	HSTT(90, 0x10, 23, 17, 16, 39, 10),
	HSTT(100, 0x20, 22, 17, 16, 37, 10),
	HSTT(110, 0x30, 25, 18, 17, 43, 11),
	HSTT(120, 0x01, 26, 20, 18, 46, 11),
	HSTT(130, 0x11, 27, 19, 19, 46, 11),
	HSTT(140, 0x21, 27, 19, 19, 46, 11),
	HSTT(150, 0x31, 28, 20, 20, 47, 12),
	HSTT(160, 0x02, 30, 21, 22, 53, 13),
	HSTT(170, 0x12, 30, 21, 23, 55, 13),
	HSTT(180, 0x22, 31, 21, 23, 53, 13),
	HSTT(190, 0x32, 32, 22, 24, 58, 13),
	HSTT(205, 0x03, 35, 22, 25, 58, 13),
	HSTT(220, 0x13, 37, 26, 27, 63, 15),
	HSTT(235, 0x23, 38, 28, 27, 65, 16),
	HSTT(250, 0x33, 41, 29, 30, 71, 17),
	HSTT(275, 0x04, 43, 29, 32, 74, 18),
	HSTT(300, 0x14, 45, 32, 35, 80, 19),
	HSTT(325, 0x25, 48, 33, 36, 86, 18),
	HSTT(350, 0x35, 51, 35, 40, 91, 20),
	HSTT(400, 0x05, 59, 37, 44, 102, 21),
	HSTT(450, 0x16, 65, 40, 49, 115, 23),
	HSTT(500, 0x26, 71, 41, 54, 126, 24),
	HSTT(550, 0x37, 77, 44, 57, 135, 26),
	HSTT(600, 0x07, 82, 46, 64, 147, 27),
	HSTT(650, 0x18, 87, 48, 67, 156, 28),
	HSTT(700, 0x28, 94, 52, 71, 166, 29),
	HSTT(750, 0x39, 99, 52, 75, 175, 31),
	HSTT(800, 0x09, 105, 55, 82, 187, 32),
	HSTT(850, 0x19, 110, 58, 85, 196, 32),
	HSTT(900, 0x29, 115, 58, 88, 206, 35),
	HSTT(950, 0x3A, 120, 62, 93, 213, 36),
	HSTT(1000, 0x0A, 128, 63, 99, 225, 38),
	HSTT(1050, 0x1A, 132, 65, 102, 234, 38),
	HSTT(1100, 0x2A, 138, 67, 106, 243, 39),
	HSTT(1150, 0x3B, 146, 69, 112, 259, 42),
	HSTT(1200, 0x0B, 151, 71, 117, 269, 43),
	HSTT(1250, 0x1B, 153, 74, 120, 273, 45),
	HSTT(1300, 0x2B, 160, 73, 124, 282, 46),
	HSTT(1350, 0x3C, 165, 76, 130, 294, 47),
	HSTT(1400, 0x0C, 172, 78, 134, 304, 49),
	HSTT(1450, 0x1C, 177, 80, 138, 314, 49),
	HSTT(1500, 0x2C, 183, 81, 143, 326, 52),
	HSTT(1550, 0x3D, 191, 84, 147, 339, 52),
	HSTT(1600, 0x0D, 194, 85, 152, 345, 52),
	HSTT(1650, 0x1D, 201, 86, 155, 355, 53),
	HSTT(1700, 0x2E, 208, 88, 161, 368, 53),
	HSTT(1750, 0x3E, 212, 89, 165, 378, 53),
	HSTT(1800, 0x0E, 220, 90, 171, 389, 54),
	HSTT(1850, 0x1E, 223, 92, 175, 401, 54),
	HSTT(1900, 0x2F, 231, 91, 180, 413, 55),
	HSTT(1950, 0x3F, 236, 95, 185, 422, 56),
	HSTT(2000, 0x0F, 243, 97, 190, 432, 56),
};

static int
dw_mipi_dsi_get_hsfreqrange(struct dw_mipi_dsi_se *dsi, unsigned int lane_mbps)
{
	int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(hstt_table); i++)
		if (lane_mbps < hstt_table[i].maxfreq)
			break;

	//if (i == ARRAY_SIZE(hstt_table))
		/*reduce lane_mbps to entry lp*/
	if (i > 0)
		i--;

	dsi->hsfreqrange = hstt_table[i].hsfreqrange;
	ret = hstt_table[i].maxfreq;

	return ret;
}

static int
dw_mipi_dsi_phy_get_timing(void *priv_data, unsigned int lane_mbps,
			   struct dw_mipi_dsi_dphy_timing *timing)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hstt_table); i++)
		if (lane_mbps < hstt_table[i].maxfreq)
			break;

	//if (i == ARRAY_SIZE(hstt_table))
		/*reduce lane_mbps to entry lp*/
	if (i > 0)
		i--;

	*timing = hstt_table[i].timing;

	return 0;
}


static int dw_mipi_dsi_phy_get_lane_mbps_2(struct dw_mipi_dsi_se *host, unsigned int lane_mbps)
{

	unsigned int target_mbps = lane_mbps;
	//unsigned int max_mbps = dppa_map[ARRAY_SIZE(dppa_map) - 1].max_mbps;
	unsigned long fvco_min, fvco_max, fin, fout;
	unsigned int min_prediv, max_prediv;
	unsigned int _prediv, best_prediv;
	unsigned int min_fbdiv, max_fbdiv;
	unsigned long _fbdiv, best_fbdiv;
	unsigned int _factor_p = 0;
	//unsigned long min_delta = ULONG_MAX;

	if (host->mdp_flags)
		fin = 24000000;
	else
		fin = 25000000;/*ref_clk*/

	fout = target_mbps  / 2;

	/* constraint: 8Mhz <= Fref / N <= 24MHz */
	min_prediv = DIV_ROUND_UP(fin, 24 * USEC_PER_SEC);
	max_prediv = fin / (8 * USEC_PER_SEC);

	/* constraint: 2000MHz <= Fvco <= 4000Mhz */
	fvco_min = 2000 * USEC_PER_SEC;
	fvco_max = 4000 * USEC_PER_SEC;

	if ((fout > 40) && (fout <= 62)) {
		_factor_p = 64;
	}
	if ((fout > 62) && (fout <= 125)) {
		_factor_p = 32;
	} else if ((fout > 125) && (fout <= 250)) {
		_factor_p = 16;
	} else if ((fout > 250) && (fout <= 500)) {
		_factor_p = 8;
	} else if ((fout > 500) && (fout <= 1000)) {
		_factor_p = 4;
	} else if ((fout > 1000)) {
		_factor_p = 2;
		fvco_max = 4500 * USEC_PER_SEC;
	} else {
		_factor_p = 8;
	}

	for (_prediv = max_prediv; _prediv >= min_prediv; _prediv--) {
		u64 tmp;

		/* Fvco = Fref * M / N / 2 */
		min_fbdiv = (u64)fvco_min * _prediv * 2 / fin;
		max_fbdiv = (u64)fvco_max * _prediv * 2 / fin;

		tmp = (u64)fout * _prediv * USEC_PER_SEC;

		tmp = tmp * 2 * _factor_p;

		tmp = tmp / fin;

		// do_div(tmp, fin);
		_fbdiv = tmp;

		if ((_fbdiv >= min_fbdiv) && (_fbdiv <= max_fbdiv)) {
			best_fbdiv = _fbdiv;
		} else {
			continue;
		}

		best_prediv = _prediv;

		if (best_prediv) {
			host->input_div = best_prediv;
			host->feedback_div = best_fbdiv;
			host->factor_p = _factor_p;
			DRM_DEV_DEBUG(host->dev, "n:%d, m:%d, p:%d\n", host->input_div, host->feedback_div, host->factor_p);
			break;
		} else {
			DRM_DEV_ERROR(host->dev, "Can not find best_fbdiv for DPHY\n");
			return -EINVAL;
		}
	}

	return 0;
}


/**
 * formula from synopsys
 * lbc_period * vid_num_chunks * (vid_pkt_size*Bpp+12+vid_null_size) / number_of_lanes =
 * pixclk_period * h_total
 */
static int
dmd_se_get_lane_mbps(void *priv_data, const struct drm_display_mode *mode,
			  unsigned long mode_flags, u32 lanes, u32 format,
			  unsigned int *lane_mbps)
{
	/*
	 * 1. bandwidth = mode->clock * bpp
	 * 2. calculate pll parameters based on phy.pllref_clk and bandwidth
	 */
	struct dw_mipi_dsi_se *dsi = priv_data;
	int bpp, lbc;
	unsigned long mpclk;

	dsi->format = format;
	bpp = mipi_dsi_pixel_format_to_bpp(format);
	//lbc = (mode->htotal * bpp / 8 + 6 ) * mode->vtotal * mode->vrefresh / lanes;

	mpclk = DIV_ROUND_UP(mode->clock, MSEC_PER_SEC);
	*lane_mbps =  mpclk * (bpp / lanes) * 10 / 8;

	if (dsi->lane_mbps == 0) {
	dsi->lane_mbps = *lane_mbps;
	//dw_mipi_dsi_phy_get_lane_mbps_2(dsi , *lane_mbps);
	//dw_mipi_dsi_get_vco_ctrl(dsi, *lane_mbps);
	} else {
	*lane_mbps = dsi->lane_mbps;
	lbc = *lane_mbps * 1000000 / 8;
	}

	*lane_mbps = dw_mipi_dsi_get_hsfreqrange(dsi, *lane_mbps);

	dw_mipi_dsi_phy_get_lane_mbps_2(dsi, *lane_mbps);

	dw_mipi_dsi_get_vco_ctrl(dsi, *lane_mbps);

	if (dsi->mdp_flags) {
		mdp_misc_reg_set(dsi, MDP_MISC_DPU0_VTOTAL, mode->vtotal);
		mdp_misc_reg_set(dsi, MDP_MISC_DPU0_HTOTAL, mode->htotal + 0);
	}

	//mdp_misc_reg_set(dsi , MDP_MISC_IVI_REQ , 3);
	 DRM_DEV_DEBUG(dsi->dev, "mode->htotal=%d lbc=%d lane_mbps=%d hsfreqrange %x\n",
		mode->htotal, lbc, *lane_mbps, dsi->hsfreqrange);

	return 0;
}


#define FUNCTION_TRACE do {dev_dbg(dsi->dev, "%s\n", __FUNCTION__); } while (0)
static int
dmd_se_encoder_atomic_check(struct drm_encoder *encoder,
				 struct drm_crtc_state *crtc_state,
				 struct drm_connector_state *conn_state)
{
	struct dw_mipi_dsi_se *dsi = to_dsi(encoder);

	FUNCTION_TRACE;

	return 0;
}

static void dmd_se_encoder_enable(struct drm_encoder *encoder)
{
	struct dw_mipi_dsi_se *dsi = to_dsi(encoder);
	int mux;

	mux = drm_of_encoder_active_endpoint_id(dsi->dev->of_node,
						&dsi->encoder);
	if (mux < 0)
		return;

	if (dsi->mdp_flags)
	dsi_clk_enable(dsi, true);

	pm_runtime_get_sync(dsi->dev);

	FUNCTION_TRACE;
}

static void dmd_se_encoder_disable(struct drm_encoder *encoder)
{
	struct dw_mipi_dsi_se *dsi = to_dsi(encoder);

	//dsi_clk_enable(dsi, false);
	pm_runtime_put(dsi->dev);

	FUNCTION_TRACE;
}

static const struct drm_encoder_helper_funcs
dmd_se_encoder_helper_funcs = {
	.atomic_check = dmd_se_encoder_atomic_check,
	.enable = dmd_se_encoder_enable,
	.disable = dmd_se_encoder_disable,
};

static const struct drm_encoder_funcs dmd_se_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int dmd_se_drm_create_encoder(struct dw_mipi_dsi_se *dsi,
					   struct drm_device *drm_dev)
{
	struct drm_encoder *encoder = &dsi->encoder;
	int ret;

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm_dev,
								 dsi->dev->of_node);

	ret = drm_encoder_init(drm_dev, encoder, &dmd_se_encoder_funcs,
				   DRM_MODE_ENCODER_DSI, NULL);
	if (ret) {
		DRM_DEV_ERROR(dsi->dev, "Failed to initialize encoder with drm\n");
		return ret;
	}

	drm_encoder_helper_add(encoder, &dmd_se_encoder_helper_funcs);

	return 0;
}

static int dmd_se_comp_bind(struct device *dev,
					struct device *master,
					void *data)
{
	struct dw_mipi_dsi_se *dsi = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	int ret;

	ret = dmd_se_drm_create_encoder(dsi, drm_dev);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to create drm encoder\n");
		return ret;
	}

	ret = dw_mipi_dsi_bind(dsi->dmd, &dsi->encoder);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to bind: %d\n", ret);
		return ret;
	}

	return 0;
}

static void dmd_se_comp_unbind(struct device *dev,
				   struct device *master,
				   void *data)
{
	struct dw_mipi_dsi_se *dsi = dev_get_drvdata(dev);

	dw_mipi_dsi_unbind(dsi->dmd);
}

static const struct component_ops dmd_se_component_ops = {
	.bind	= dmd_se_comp_bind,
	.unbind = dmd_se_comp_unbind,
};

static int dmd_se_host_attach(void *priv_data,
						struct mipi_dsi_device *device)
{
#if 0
	struct dw_mipi_dsi_se *dsi = priv_data;
	int ret;

	ret = component_add(dsi->dev, &dmd_se_component_ops);
	if (ret) {
		DRM_DEV_ERROR(dsi->dev, "Failed to register component: %d\n",
					ret);
		return ret;
	}
#endif

	return 0;
}

static int dmd_se_host_detach(void *priv_data,
							  struct mipi_dsi_device *device)
{
	struct dw_mipi_dsi_se *dsi = priv_data;

	component_del(dsi->dev, &dmd_se_component_ops);

	return 0;
}

static const struct dw_mipi_dsi_host_ops dmd_se_host_ops = {
	.attach = dmd_se_host_attach,
	.detach = dmd_se_host_detach,
};



static int
dw_mipi_read_test(void *priv_data)
{
struct dw_mipi_dsi_se *dsi = priv_data;

dw_mipi_dsi_phy_read_test(dsi, 1);

dw_mipi_dsi_phy_read_test(dsi, 0);

return 0;
}


static const struct dw_mipi_dsi_phy_ops dmd_se_phy_ops = {
	.init = dmd_se_phy_init,
	.get_lane_mbps = dmd_se_get_lane_mbps,
	.get_timing = dw_mipi_dsi_phy_get_timing,
	.read_config = dw_mipi_read_test,
};


static struct dw_mipi_dsi_plat_data se1000_chip_data = {
	//.mdp_misc_base = 0x0E6D50000ULL,
	.phy_ops = &dmd_se_phy_ops,
	.host_ops =  &dmd_se_host_ops,
	.max_data_lanes = 4,
};


static int dmd_se_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_mipi_dsi_se *dsi;
	struct resource *res;

	struct device_node *misc_np;
#ifdef CONFIG_SADP_INSTANCE
	void __iomem *tmpcheck;
	u32 val;
#endif
	int ret;
	const struct dw_mipi_dsi_plat_data *data;

	struct property *prop;
	unsigned int lane_mbps; /* per lane */
	struct clk *pxclk;//used for sadp
	long clk_freq;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dsi->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(dsi->base)) {
		DRM_DEV_ERROR(dev, "Unable to get dsi registers\n");
		return PTR_ERR(dsi->base);
	}

	dsi->pllref_clk = devm_clk_get(dev, "ref");

	if (IS_ERR(dsi->pllref_clk)) {
		dsi->mdp_flags = 0;
	} else
		dsi->mdp_flags = 1;


	dsi->phy_cfg_clk = devm_clk_get(dev, "phy_cfg");
	if (IS_ERR(dsi->phy_cfg_clk)) {
		ret = PTR_ERR(dsi->phy_cfg_clk);
		DRM_DEV_ERROR(dev,
				  "Unable to get phy_cfg_clk: %d\n", ret);
		return ret;
	}

/*add misc dts parse for config dpy misc*/
	misc_np = of_parse_phandle(dev->of_node, "misc-syscon", 0);
	if (!misc_np) {
		dev_err(dev, "%s no dsi_common in dts\n", __func__);
		return -ENODEV;
	}

	dsi->misc_base = syscon_node_to_regmap(misc_np);
	if (IS_ERR(dsi->misc_base))
		return -ENODEV;

	of_node_put(misc_np);

	if (dsi->mdp_flags) {
		mdp_misc_reg_set(dsi, MDP_MISC_IVI_REQ, 3);
		mdp_misc_reg_set(dsi, MDP_MISC_RVC_REQ, 0);

		clk_prepare_enable(dsi->pllref_clk);
		clk_prepare_enable(dsi->phy_cfg_clk);
	}

	prop = of_find_property(dev->of_node, "lane_mbps", NULL);
	if (prop) {
		ret = of_property_read_u32(dev->of_node, "lane_mbps", &lane_mbps);
		if (ret) {
			dev_err(dev, "get lane_mbps fail, ret %d\n", ret);
			return -ENODEV;
		}
		dsi->lane_mbps = lane_mbps;
	}


	dsi->dev = dev;
	data = of_device_get_match_data(dev);
	dsi->pdata = *data;
	dsi->pdata.base = dsi->base;
	dsi->pdata.priv_data = dsi;

	platform_set_drvdata(pdev, dsi);

#ifdef CONFIG_SADP_INSTANCE

	if (dsi->mdp_flags == 0) {
#define SAF_CSR_BASE (0x32070000)
		tmpcheck = devm_ioremap(dev, SAF_CSR_BASE, PAGE_SIZE);
		if (!tmpcheck) {
			 DRM_DEV_ERROR(dev, "Failed to map saf io\n");
			 return -ENODEV;
		}

		pxclk = devm_clk_get(dev, "pxclk");
		if (IS_ERR(pxclk)) {
			ret = PTR_ERR(pxclk);
			DRM_DEV_ERROR(dev,
				"Unable to get pxclk: %d\n", ret);
			return ret;
		}

		/*config sadp clk*/
		clk_freq = clk_round_rate(pxclk, clk_freq) ;
		if (clk_freq == 300000000) {
			writel(0x11f, tmpcheck + 0x0c);//300M
		} else if (clk_freq == 150000000) {
			writel(0x12f, tmpcheck + 0x0c);//150M
		} else if (clk_freq == 600000000) {
			writel(0x10f, tmpcheck + 0x0c);//600M
		} else {
			writel(0x13f, tmpcheck + 0x0c);//100M
		}

		ret = readl_poll_timeout(tmpcheck + 0x0c,
								 val, ((val & 0xf) == 0xf),
								 1000, 10000);
		if (ret) {
			DRM_DEV_ERROR(dev,
							 "Failed to probe SAF csr setting %d\n", ret);
			return -ENODEV;
		}

		devm_iounmap(dev, tmpcheck);
	}
#endif
	dsi->dmd = dw_mipi_dsi_probe(pdev, &dsi->pdata);
	if (IS_ERR(dsi->dmd)) {
		ret = PTR_ERR(dsi->dmd);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dev,
					  "Failed to probe dw_mipi_dsi: %d\n", ret);
		goto err_clkdisable;
	}

	component_add(dsi->dev, &dmd_se_component_ops);

	return 0;

err_clkdisable:
//	clk_disable_unprepare(dsi->pllref_clk);
	return ret;
}

/*reset dsi dphy ,disable dphy.*/
static void  dmd_se_shutdown(struct platform_device *pdev)
{
	struct dw_mipi_dsi_se *dsi = platform_get_drvdata(pdev);

	dw_mipi_dsi_shutdown(dsi->dmd);
}

static int dmd_se_remove(struct platform_device *pdev)
{
	struct dw_mipi_dsi_se *dsi = platform_get_drvdata(pdev);

	component_del(dsi->dev, &dmd_se_component_ops);

	dw_mipi_dsi_remove(dsi->dmd);

	return 0;
}

static const struct of_device_id dmd_se_dt_ids[] = {
	{
		.compatible = "siengine,se1000-mipi-dsi",
		.data = &se1000_chip_data,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dmd_se_dt_ids);

struct platform_driver dmd_se_driver = {
	.probe		= dmd_se_probe,
	.remove		= dmd_se_remove,
	.shutdown       = dmd_se_shutdown,
	.driver		= {
		.of_match_table = dmd_se_dt_ids,
		.name	= "dw-mipi-dsi-siengine",
	},
};

module_platform_driver(dmd_se_driver);
MODULE_AUTHOR("Xiaohu Qian <xiaohu.qian@siengine.com>");
MODULE_DESCRIPTION("Se1000 DW-MIPI-DSI");
MODULE_LICENSE("GPL");
