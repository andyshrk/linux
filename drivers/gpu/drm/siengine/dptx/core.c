/*
 * Copyright (c) 2016 Synopsys, Inc.
 *
 * Synopsys DP TX Linux Software Driver and documentation (hereinafter,
 * "Software") is an Unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto. You are permitted to use and
 * redistribute this Software in source and binary forms, with or without
 * modification, provided that redistributions of source code must retain this
 * notice. You may not view, use, disclose, copy or distribute this file or
 * any information contained herein except pursuant to this license grant from
 * Synopsys. If you do not agree with this notice, including the disclaimer
 * below, then you are not authorized to use the Software.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#include "se_dptx.h"

/*
 * Core Access Layer
 *
 * Provides low-level register access to the DPTX core.
 */

/**
 * dptx_intr_en() - Enables interrupts
 * @dptx: The dptx struct
 * @bits: The interrupts to enable
 *
 * This function enables (unmasks) all interrupts in the INTERRUPT
 * register specified by @bits.
 */
static void dptx_intr_en(struct dptx *dptx, u32 bits)
{
	u32 ien;

	ien = dptx_readl(dptx, DPTX_IEN);
	ien |= bits;
	dptx_writel(dptx, DPTX_IEN, ien);
}

/**
 * dptx_intr_dis() - Disables interrupts
 * @dptx: The dptx struct
 * @bits: The interrupts to disable
 *
 * This function disables (masks) all interrupts in the INTERRUPT
 * register specified by @bits.
 */
static void dptx_intr_dis(struct dptx *dptx, u32 bits)
{
	u32 ien;

	ien = dptx_readl(dptx, DPTX_IEN);
	ien &= ~bits;
	dptx_writel(dptx, DPTX_IEN, ien);
}

/**
 * dptx_global_intr_en() - Enables top-level interrupts
 * @dptx: The dptx struct
 *
 * Enables (unmasks) all top-level interrupts.
 */
void dptx_global_intr_en(struct dptx *dptx)
{
	dptx_intr_en(dptx, DPTX_IEN_ALL_INTR &
		~(DPTX_ISTS_AUX_REPLY | DPTX_ISTS_AUX_CMD_INVALID));
}

/**
 * dptx_global_intr_dis() - Disables top-level interrupts
 * @dptx: The dptx struct
 *
 * Disables (masks) all top-level interrupts.
 */
void dptx_global_intr_dis(struct dptx *dptx)
{
	dptx_intr_dis(dptx, DPTX_IEN_ALL_INTR);
}

void dptx_hpd_intr_en(struct dptx *dptx)
{
	u32 hpd_ien;

	/* Enable all HPD interrupts */
	hpd_ien = dptx_readl(dptx, DPTX_HPD_IEN);
	hpd_ien |= (DPTX_HPD_IEN_IRQ_EN |
		    DPTX_HPD_IEN_HOT_PLUG_EN |
		    DPTX_HPD_IEN_HOT_UNPLUG_EN);

	dptx_writel(dptx, DPTX_HPD_IEN, hpd_ien);

	/* Enable hpd top-level interrupts */
	dptx_intr_en(dptx, DPTX_IEN_HPD);
}

void dptx_disable_fast_link_training(struct dptx *dptx)
{
	u32 val;

	val = dptx_readl(dptx, DPTX_CCTL);
	val &= ~DPTX_CCTL_FAST_LINK_TRAIN_EN;
	dptx_writel(dptx, DPTX_CCTL, val);
}

void dptx_mst_en(struct dptx *dptx)
{
	u32 val;

	val = dptx_readl(dptx, DPTX_CCTL);
	val |= DPTX_CCTL_ENABLE_MST_MODE;
	dptx_writel(dptx, DPTX_CCTL, val);
}

void dptx_mst_dis(struct dptx *dptx)
{
	u32 val;

	val = dptx_readl(dptx, DPTX_CCTL);
	val &= ~DPTX_CCTL_ENABLE_MST_MODE;
	dptx_writel(dptx, DPTX_CCTL, val);
}

int dptx_get_sink_cap_by_dpcd(struct dptx *dptx)
{
	int retval;

	memset(dptx->rx_caps, 0, DP_RECEIVER_CAP_SIZE);

	retval = dptx_read_bytes_from_dpcd(dptx, DP_DPCD_REV,
					   dptx->rx_caps,
					   DP_RECEIVER_CAP_SIZE);

	if (dptx->rx_caps[DP_TRAINING_AUX_RD_INTERVAL] &
		DP_EXTENDED_RECEIVER_CAP_FIELD_PRESENT) {
		retval = dptx_read_bytes_from_dpcd(dptx, 0x2200,
						   dptx->rx_caps,
						   DP_RECEIVER_CAP_SIZE);
	}

	return retval;
}

void dptx_enh_frame_en(struct dptx *dptx)
{
	u32 val;

	val = dptx_readl(dptx, DPTX_CCTL);
	val |= DPTX_CCTL_ENH_FRAME_EN;
	dptx_writel(dptx, DPTX_CCTL, val);
}

/**
 * dptx_soft_reset() - Performs a core soft reset
 * @dptx: The dptx struct
 * @bits: The components to reset
 *
 * Resets specified parts of the core by writing @bits into the core
 * soft reset control register and clearing them 10-20 microseconds
 * later.
 */
void dptx_soft_reset(struct dptx *dptx, u32 bits)
{
	u32 rst;

	bits &= (DPTX_SRST_CTRL_ALL);

	/* Set reset bits */
	rst = dptx_readl(dptx, DPTX_SRST_CTRL);
	rst |= bits;
	dptx_writel(dptx, DPTX_SRST_CTRL, rst);

	usleep_range(10, 20);

	/* Clear reset bits */
	rst = dptx_readl(dptx, DPTX_SRST_CTRL);
	rst &= ~bits;
	dptx_writel(dptx, DPTX_SRST_CTRL, rst);
}

/**
 * dptx_soft_reset_all() - Reset all core modules
 * @dptx: The dptx struct
 */
void dptx_soft_reset_all(struct dptx *dptx)
{
	dptx_soft_reset(dptx, DPTX_SRST_CTRL_ALL);
}

void dptx_phy_soft_reset(struct dptx *dptx)
{
	dptx_soft_reset(dptx, DPTX_SRST_CTRL_PHY);
}

/**
 * dptx_core_init_phy() - Initializes the DP TX PHY module
 * @dptx: The dptx struct
 *
 * Initializes the PHY layer of the core. This needs to be called
 * whenever the PHY layer is reset.
 */
void dptx_core_init_phy(struct dptx *dptx)
{
	u32 phyifctrl;

#ifdef DPTX_COMBO_PHY
	phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);
	phyifctrl &= ~DPTX_PHYIF_CTRL_WIDTH;
	dptx_writel(dptx, DPTX_PHYIF_CTRL, phyifctrl);
#else
	phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);
	phyifctrl |= DPTX_PHYIF_CTRL_WIDTH;
	dptx_writel(dptx, DPTX_PHYIF_CTRL, phyifctrl);
#endif
}

/**
* dptx_sink_enabled_ssc() - Returns true, if sink is enabled ssc
* @dptx: The dptx struct
*
*/
bool dptx_sink_enabled_ssc(struct dptx *dptx)
{
	u8 byte;

	dptx_read_dpcd(dptx, DP_MAX_DOWNSPREAD, &byte);

	return byte & 1;
}

/**
 * dptx_core_program_ssc() - Move phy to P3 state and programs SSC
 * @dptx: The dptx struct
 *
 * Enables SSC should be called during hot plug.
 *
 */
int dptx_core_program_ssc(struct dptx *dptx, bool sink_ssc)
{
	u32 phyifctrl;
	u8 retval;

	/* Enable 4 lanes, before programming SSC */
	dptx_phy_set_lanes(dptx, 4);

	/* Move PHY to P3 to program SSC */
	phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);
	phyifctrl |= (3 << DPTX_PHYIF_CTRL_LANE_PWRDOWN_SHIFT);// move phy to P3 state
	dptx_writel(dptx, DPTX_PHYIF_CTRL, phyifctrl);

	retval = dptx_phy_wait_busy(dptx, DPTX_MAX_LINK_LANES);
	if (retval) {
		dptx_err(dptx, "Timed out waiting for PHY BUSY\n");
		return retval;
	}

	phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);
	if(dptx->ssc_en && sink_ssc)
		phyifctrl &= ~DPTX_PHYIF_CTRL_SSC_DIS;
	else
		phyifctrl |= DPTX_PHYIF_CTRL_SSC_DIS;

	dptx_writel(dptx, DPTX_PHYIF_CTRL, phyifctrl);

	retval = dptx_phy_wait_busy(dptx, DPTX_MAX_LINK_LANES);
	if (retval) {
		dptx_err(dptx, "Timed out waiting for PHY BUSY\n");
		return retval;
	 }

	return 0;
}

/**
 * dptx_check_dptx_id() - Check value of DPTX_ID register
 * @dptx: The dptx struct
 *
 * Returns True if DPTX core correctly identifyed.
 */
bool dptx_check_dptx_id(struct dptx *dptx)
{
	u32 dptx_id;

	dptx_id = dptx_readl(dptx, DPTX_ID);
	if (dptx_id != ((DPTX_ID_DEVICE_ID << DPTX_ID_DEVICE_ID_SHIFT) |
			DPTX_ID_VENDOR_ID))
		return false;

	return true;
}

/**
* dptx_enable_ssc() - Enables SSC based on automation request,
*		      if DPTX controller enables ssc
* @dptx: The dptx struct
*
*/
void dptx_enable_ssc(struct dptx *dptx)
{
	bool sink_ssc = dptx_sink_enabled_ssc(dptx);
	if(sink_ssc)
		dev_dbg(dptx->dev, "%s: SSC enable on the sink side\n", __func__);
	else
		dev_dbg(dptx->dev, "%s: SSC disabled on the sink side\n", __func__);

	dptx_core_program_ssc(dptx, sink_ssc);
}

void dptx_init_hwparams(struct dptx *dptx)
{
	u32 reg;

	reg = dptx_readl(dptx, DPTX_CONFIG1);

	/* Num MST streams */
	dptx->streams = (reg & DPTX_CONFIG1_NUM_STREAMS_MASK) >>
			DPTX_CONFIG1_NUM_STREAMS_SHIFT;

	/* Combo PHY */
	dptx->hwparams.gen2phy = !!(reg & DPTX_CONFIG1_GEN2_PHY);

	/* DSC */
	dptx->hwparams.dsc = !!(reg & DPTX_CONFIG1_DSC_EN);

	/* Multi pixel mode */
	switch ((reg & DPTX_CONFIG1_MP_MODE_MASK) >> DPTX_CONFIG1_MP_MODE_SHIFT) {
	case DPTX_CONFIG1_MP_MODE_QUAD:
		dptx->hwparams.multipixel = DPTX_MP_QUAD_PIXEL;
		break;
	case DPTX_CONFIG1_MP_MODE_DUAL:
		dptx->hwparams.multipixel = DPTX_MP_DUAL_PIXEL;
		break;
	case DPTX_CONFIG1_MP_MODE_SINGLE:
	default:
		dptx->hwparams.multipixel = DPTX_MP_SINGLE_PIXEL;
		break;
	}
}

/**
 * dptx_core_init() - Initializes the DP TX core
 * @dptx: The dptx struct
 *
 * Initialize the DP TX core and put it in a known state.
 */
int dptx_core_init(struct dptx *dptx)
{
	char str[15];
	u32 version;
	u32 hpd_ien;

	dptx_soft_reset(dptx, DPTX_SRST_CTRL_AUX);
	dptx_aux_misc_init(dptx);

	/* Check the core version */
	memset(str, 0, sizeof(str));
	version = dptx_readl(dptx, DPTX_VER_NUMBER);
	str[0] = (version >> 24) & 0xff;
	str[1] = '.';
	str[2] = (version >> 16) & 0xff;
	str[3] = (version >> 8) & 0xff;
	str[4] = version & 0xff;
	dptx->version = version;

	version = dptx_readl(dptx, DPTX_VER_TYPE);
	str[5] = '-';
	str[6] = (version >> 24) & 0xff;
	str[7] = (version >> 16) & 0xff;
	str[8] = (version >> 8) & 0xff;
	str[9] = version & 0xff;

	dptx_info(dptx, "Core version: %s\n", str);

	dptx_phy_reset_init(dptx);

	/* Enable all HPD interrupts */
	hpd_ien = dptx_readl(dptx, DPTX_HPD_IEN);
	hpd_ien |= (DPTX_HPD_IEN_IRQ_EN |
		DPTX_HPD_IEN_HOT_PLUG_EN |
		DPTX_HPD_IEN_HOT_UNPLUG_EN);

	dptx_writel(dptx, DPTX_HPD_IEN, hpd_ien);

	/* Enable all top-level interrupts */
	dptx_global_intr_en(dptx);

	return 0;
}

/**
 * dptx_core_deinit() - Deinitialize the core
 * @dptx: The dptx struct
 *
 * Disable the core in preparation for module shutdown.
 */
int dptx_core_deinit(struct dptx *dptx)
{
	dptx_global_intr_dis(dptx);
	dptx_soft_reset_all(dptx);
	return 0;
}

/*
 * PHYIF core access functions
 */

unsigned int dptx_phy_get_lanes(struct dptx *dptx)
{
	u32 phyifctrl;
	u32 val;

	phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);
	val = (phyifctrl & DPTX_PHYIF_CTRL_LANES_MASK) >>
		DPTX_PHYIF_CTRL_LANES_SHIFT;

	return (1 << val);
}

void dptx_phy_set_lanes(struct dptx *dptx, unsigned int lanes)
{
	u32 phyifctrl;
	u32 val;

	switch (lanes) {
	case 1:
		val = 0;
		break;
	case 2:
		val = 1;
		break;
	case 4:
		val = 2;
		break;
	default:
		return;
	}

	phyifctrl = 0;
	phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);
	phyifctrl &= ~DPTX_PHYIF_CTRL_LANES_MASK;
	phyifctrl |= (val << DPTX_PHYIF_CTRL_LANES_SHIFT);
	dptx_writel(dptx, DPTX_PHYIF_CTRL, phyifctrl);
}

void dptx_phy_set_rate(struct dptx *dptx, unsigned int rate)
{
	u32 phyifctrl;

	phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);

#ifdef DPTX_COMBO_PHY
	switch (rate) {
	case DPTX_PHYIF_CTRL_RATE_RBR:
	case DPTX_PHYIF_CTRL_RATE_HBR:
		/* Set 20-bit PHY width */
		phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);
		phyifctrl &= ~DPTX_PHYIF_CTRL_WIDTH;
		dptx_writel(dptx, DPTX_PHYIF_CTRL, phyifctrl);
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR2:
	case DPTX_PHYIF_CTRL_RATE_HBR3:
		/* Set 40-bit PHY width */
		phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);
		phyifctrl |= DPTX_PHYIF_CTRL_WIDTH;
		dptx_writel(dptx, DPTX_PHYIF_CTRL, phyifctrl);
		break;
	default:
		break;
	}
#else
	/* Set 20-bit PHY width */
	phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);
	phyifctrl &= ~DPTX_PHYIF_CTRL_WIDTH;
	dptx_writel(dptx, DPTX_PHYIF_CTRL, phyifctrl);
#endif

	phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);
	phyifctrl &= ~DPTX_PHYIF_CTRL_RATE_MASK;
	phyifctrl |= rate << DPTX_PHYIF_CTRL_RATE_SHIFT;
	dptx_writel(dptx, DPTX_PHYIF_CTRL, phyifctrl);
}

unsigned int dwc_phy_get_rate(struct dptx *dptx)
{
	u32 phyifctrl;
	u32 rate;

	phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);
	rate = (phyifctrl & DPTX_PHYIF_CTRL_RATE_MASK) >>
		DPTX_PHYIF_CTRL_RATE_SHIFT;

	return rate;
}

int dptx_phy_wait_busy(struct dptx *dptx, unsigned int lanes)
{
	unsigned int count;
	u32 phyifctrl;
	u32 mask = 0;

	switch (lanes) {
	case 4:
		mask |= DPTX_PHYIF_CTRL_BUSY(3);
		mask |= DPTX_PHYIF_CTRL_BUSY(2);
		mask |= DPTX_PHYIF_CTRL_BUSY(1);
		mask |= DPTX_PHYIF_CTRL_BUSY(0);
		break;
	case 2:
		mask |= DPTX_PHYIF_CTRL_BUSY(1);
		mask |= DPTX_PHYIF_CTRL_BUSY(0);
		break;
	case 1:
		mask |= DPTX_PHYIF_CTRL_BUSY(0);
		break;
	default:
		break;
	}

	count = 0;

	while (1) {
		phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);
		if (!(phyifctrl & mask))
			break;

		count++;
		if (count > 20)
			return -EBUSY;
		mdelay(1);
		dptx_dbg(dptx, "PHY_BUSY wait\n");
	}

	return 0;
}

void dptx_phy_set_pre_emphasis(struct dptx *dptx,
			       unsigned int lane,
			       unsigned int level)
{
	u32 phytxeq;

	if (lane > 3)
		return;

	if (level > 3)
		level = 3;

	phytxeq = dptx_readl(dptx, DPTX_PHY_TX_EQ);
	phytxeq &= ~DPTX_PHY_TX_EQ_PREEMP_MASK(lane);
	phytxeq |= (level << DPTX_PHY_TX_EQ_PREEMP_SHIFT(lane)) &
		DPTX_PHY_TX_EQ_PREEMP_MASK(lane);

	dptx_writel(dptx, DPTX_PHY_TX_EQ, phytxeq);
}

void dptx_phy_set_vswing(struct dptx *dptx,
			 unsigned int lane,
			 unsigned int level)
{
	u32 phytxeq;

	if (lane > (DPTX_MAX_LINK_LANES - 1))
		return;

	if (level > 3)
		level = 3;

	phytxeq = dptx_readl(dptx, DPTX_PHY_TX_EQ);
	phytxeq &= ~DPTX_PHY_TX_EQ_VSWING_MASK(lane);
	phytxeq |= (level << DPTX_PHY_TX_EQ_VSWING_SHIFT(lane)) &
		DPTX_PHY_TX_EQ_VSWING_MASK(lane);

	dptx_writel(dptx, DPTX_PHY_TX_EQ, phytxeq);
}

void dptx_phy_set_pattern(struct dptx *dptx,
			  unsigned int pattern)
{
	u32 phyifctrl = 0;

	phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);
	phyifctrl &= ~DPTX_PHYIF_CTRL_TPS_SEL_MASK;
	phyifctrl |= ((pattern << DPTX_PHYIF_CTRL_TPS_SEL_SHIFT) &
		DPTX_PHYIF_CTRL_TPS_SEL_MASK);
	dptx_writel(dptx, DPTX_PHYIF_CTRL, phyifctrl);
}

void dptx_phy_enable_xmit(struct dptx *dptx, unsigned int lanes, bool enable)
{
	u32 phyifctrl;
	u32 mask = 0;

	phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);

	switch (lanes) {
	case 4:
		mask |= DPTX_PHYIF_CTRL_XMIT_EN(3);
		mask |= DPTX_PHYIF_CTRL_XMIT_EN(2);
		mask |= DPTX_PHYIF_CTRL_XMIT_EN(1);
		mask |= DPTX_PHYIF_CTRL_XMIT_EN(0);
		break;
	case 2:
		mask |= DPTX_PHYIF_CTRL_XMIT_EN(1);
		mask |= DPTX_PHYIF_CTRL_XMIT_EN(0);
		break;
	case 1:
		mask |= DPTX_PHYIF_CTRL_XMIT_EN(0);
		break;
	default:
		break;
	}

	if (enable)
		phyifctrl |= mask;
	else
		phyifctrl &= ~mask;

	dptx_writel(dptx, DPTX_PHYIF_CTRL, phyifctrl);
}

int dptx_phy_rate_to_bw(unsigned int rate)
{
	switch (rate) {
	case DPTX_PHYIF_CTRL_RATE_RBR:
		return DP_LINK_BW_1_62;
	case DPTX_PHYIF_CTRL_RATE_HBR:
		return DP_LINK_BW_2_7;
	case DPTX_PHYIF_CTRL_RATE_HBR2:
		return DP_LINK_BW_5_4;
	case DPTX_PHYIF_CTRL_RATE_HBR3:
		return DP_LINK_BW_8_1;
	default:
		return -EINVAL;
	}
}

int dptx_bw_to_phy_rate(unsigned int bw)
{
	switch (bw) {
	case DP_LINK_BW_1_62:
		return DPTX_PHYIF_CTRL_RATE_RBR;
	case DP_LINK_BW_2_7:
		return DPTX_PHYIF_CTRL_RATE_HBR;
	case DP_LINK_BW_5_4:
		return DPTX_PHYIF_CTRL_RATE_HBR2;
	case DP_LINK_BW_8_1:
		return DPTX_PHYIF_CTRL_RATE_HBR3;
	default:
		return -EINVAL;
	}
}

int dptx_bw_to_link_rate(unsigned int bw)
{
	switch (bw) {
	case DP_LINK_BW_1_62:
		return 162000;
	case DP_LINK_BW_2_7:
		return 270000;
	case DP_LINK_BW_5_4:
		return 540000;
	case DP_LINK_BW_8_1:
		return 810000;
	default:
		return -EINVAL;
	}
}

void dptx_reg_dump(struct dptx *dptx, int stream)
{
	int i = 0;

	REG_DUMP(dptx, DPTX_SRST_CTRL);

	REG_DUMP(dptx, DPTX_SDP_VERTICAL_CTRL);

	REG_DUMP(dptx, DPTX_SDP_HORIZONTAL_CTRL);
	REG_DUMP(dptx, DPTX_IEN);
	REG_DUMP(dptx, DPTX_HPD_IEN);

	REG_DUMP(dptx, DPTX_TYPE_C_CTRL);

	REG_DUMP(dptx, DPTX_HPDSTS);
	REG_DUMP(dptx, DPTX_ISTS);

	REG_DUMP(dptx, DPTX_PHYIF_CTRL);
	for (i = 0; i < 8; ++i)
		REG_DUMP(dptx, DPTX_MST_VCP_TABLE_REG_N(i));
	REG_DUMP(dptx, DPTX_CCTL);

	dptx_info(dptx, "+++++dump stream:%d reg", stream);
	REG_DUMP(dptx, DPTX_VIDEO_HBLANK_INTERVAL_N(stream));
	REG_DUMP(dptx, DPTX_VSAMPLE_POLARITY_CTRL_N(stream));
	REG_DUMP(dptx, DPTX_VIDEO_MSA1_N(stream));
	REG_DUMP(dptx, DPTX_VIDEO_MSA2_N(stream));
	REG_DUMP(dptx, DPTX_VIDEO_MSA3_N(stream));
	REG_DUMP(dptx, DPTX_VIDEO_CONFIG1_N(stream));
	REG_DUMP(dptx, DPTX_VIDEO_CONFIG2_N(stream));
	REG_DUMP(dptx, DPTX_VIDEO_CONFIG3_N(stream));
	REG_DUMP(dptx, DPTX_VIDEO_CONFIG4_N(stream));
	REG_DUMP(dptx, DPTX_VIDEO_CONFIG5_N(stream));
	REG_DUMP(dptx, DPTX_VSAMPLE_CTRL_N(stream));
	dptx_info(dptx, "------dump stream:%d reg", stream);
}

void dptx_aux_misc_init(struct dptx *dptx)
{
	regmap_write(dptx->misc_base, DPTX_AUX_MODE, 1);
	regmap_write(dptx->misc_base, DPTX_AUX_PHY_PWDNB, 1);
	regmap_write(dptx->misc_base, DPTX_AUX_HYS_TUNE, 2);
	regmap_write(dptx->misc_base, DPTX_AUX_VOD_TUNE, 1);
	regmap_write(dptx->misc_base, DPTX_AUX_CTRL, 0);
}

void dptx_phy_srst_set(struct dptx *dptx)
{
	u32 phy_rst;
	phy_rst = dptx_readl(dptx, DPTX_SRST_CTRL);
	phy_rst |= DPTX_SRST_CTRL_PHY;
	dptx_writel(dptx, DPTX_SRST_CTRL, phy_rst);
	udelay(50);
}

void dptx_phy_srst_clear(struct dptx *dptx)
{
	u32 phy_rst;
	phy_rst = dptx_readl(dptx, DPTX_SRST_CTRL);
	phy_rst &= ~DPTX_SRST_CTRL_PHY;
	dptx_writel(dptx, DPTX_SRST_CTRL, phy_rst);
	udelay(50);
}

void dptx_phy_misc_init(struct dptx *dptx)
{
	u32 val;
	int tries = 0;

	// PHY_RESET assertion
	dptx_phy_srst_set(dptx);

	regmap_write(dptx->misc_base, DPTX_PHY_REF_USE_PAD, PHY_REF_USE_PAD_INTERNAL);

	regmap_write(dptx->misc_base, DPTX_PHY_REF_RANGE, 0x0); // 0x138 dp_phy_ref_range
	regmap_write(dptx->misc_base, DPTX_DCO_FINETUNE, 0x11); // 0x12c dco_finetune
	regmap_write(dptx->misc_base, DPTX_DCO_RANGE, 0x0); // 0x130 dco range
	regmap_write(dptx->misc_base, DPTX_PHY_RX_VEF_CTRL, 0x5); // 0x13c dp_phy_rx_vef_ctrl
	regmap_write(dptx->misc_base, DPTX_PHY_TX_VBOOST_LVL, 0x5); // 0x17c dp_phy_tx_vboost_lvl
	regmap_write(dptx->misc_base, DPTX_PHY_SRAM_BYPASS, 0x0); // 0x208 dp_phy_sram_bypass
	regmap_write(dptx->misc_base, DPTX_PHY_SRAM_EXT_LD_DONE, 0x0); // 0x20c dp_phy_sram_ext_ld_done
	udelay(5);

	// PHY_RESET de-assertion
	dptx_phy_srst_clear(dptx);

	// wait for sram_init_done
	while (1) {
		regmap_read(dptx->misc_base, DPTX_PHY_SRAM_INIT_DONE, &val); // 0x210  dp_phy_sram_init_done
		if (val)
			break;

		tries++;
		if (tries > 2000) {
			dptx_err(dptx, "failed to wait for dp phy sram init done\n");
			break;
		}
		udelay(5);
	}
}

int dptx_phyreg_cmd_rw(struct dptx *dptx, u16 phy_address, bool rw, u16 phy_data)
{
	int val = -1, tries = 0;

	if (rw == 1) { //read
		dptx_writel(dptx, DPTX_PHYREG_CMDADDR, phy_address | DPTX_PHYREG_CMD_READ);
		while (1) {
			val = dptx_readl(dptx, DPTX_PHYREG_DATA);
			if (val & PHYREG_DATA_PHY_DONE)
				break;
			tries++;
			if (tries > 2000) {
				dptx_err(dptx, "failed to wait phy done\n");
				return -1;
			}
		}
		return (val & PHYREG_DATA_PHY_DATA);
	} else if (rw == 0) { //write
		dptx_writel(dptx, DPTX_PHYREG_DATA, phy_data);
		dptx_writel(dptx, DPTX_PHYREG_CMDADDR, phy_address | DPTX_PHYREG_CMD_WRITE);
		while (1) {
			val = dptx_readl(dptx, DPTX_PHYREG_CMDADDR);
			if (!(val & DPTX_PHYREG_CMD_WRITE))
				break;
			tries++;
			if (tries > 2000)
				return -1;
		}
		return 0;
	}
	return -1;
}

void dptx_phy_corereg_init(struct dptx *dptx)
{
	int val = 0, i;
	//int idcode_lo, idcode_hi;

	//idcode_lo = dptx_phyreg_cmd_rw(dptx, 0x00, 1, 0x00);
	//idcode_hi = dptx_phyreg_cmd_rw(dptx, 0x01, 1, 0x00);

	for (i = 0; i < dptx->max_lanes; i++) {
		val = dptx_phyreg_cmd_rw(dptx, RAWLANEN_DIG_PCS_XF_ATE_OVRD_IN(i), 1, 0x00);
		dptx_phyreg_cmd_rw(dptx, RAWLANEN_DIG_PCS_XF_ATE_OVRD_IN(i), 0,
			val | (RX_ADAPT_DFE_EN_OVRD_VAL | RX_ADAPT_DFE_EN_OVRD_EN |
			RX_ADAPT_AFE_EN_OVRD_EN | RX_ADAPT_AFE_EN_OVRD_VAL));// 0x138 dp_phy_ref_range
	}

	val = dptx_phyreg_cmd_rw(dptx, SUP_DIG_MPLLB_MPLL_PWR_CTL_MPLL_DAC_MAXRANGE, 1, 0x00);
	val &= ~DAC_IN_SHIFT_MASK;
	dptx_phyreg_cmd_rw(dptx, SUP_DIG_MPLLB_MPLL_PWR_CTL_MPLL_DAC_MAXRANGE, 0, val | (16 << DAC_IN_SHIFT));

	//val = dptx_phyreg_cmd_rw(dptx, SUP_DIG_MPLLB_MPLL_PWR_CTL_MPLL_DAC_MAXRANGE, 1, 0x00 );
}

void dptx_phy_ovrd(struct dptx *dptx)
{
	int val = 0, i;

	for (i = 0; i < dptx->max_lanes; i++) {
		val = dptx_phyreg_cmd_rw(dptx, RAWLANEN_DIG_PCS_XF_LANE_XCVR_MODE_OVRD_IN(i), 1, 0x00);
		val &= ~LANE_XCVR_MODE_OVRD_VAL_MASK;
		dptx_phyreg_cmd_rw(dptx, RAWLANEN_DIG_PCS_XF_LANE_XCVR_MODE_OVRD_IN(i), 0, val | LANE_XCVR_MODE_OVRD_EN);
	}
}

/*
 * refer to P239. using ROM contents as basis
 */
void dptx_phy_reset_init(struct dptx *dptx)
{
	dptx_phy_misc_init(dptx);
	dptx_phy_corereg_init(dptx);
	dptx_phy_ovrd(dptx);

	regmap_write(dptx->misc_base, DPTX_PHY_SRAM_EXT_LD_DONE, 0x1); // 0x20c dp_phy_sram_ext_ld_done

	//mdelay(500);

    dptx_phy_wait_busy(dptx, DPTX_MAX_LINK_LANES);
}

