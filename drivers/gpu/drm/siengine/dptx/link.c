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

static struct dptx_eq_setting eq_settings[] = {
	{0, 0, 20, 0, 0},
	{0, 1, 25, 5, 0},
	{0, 2, 31, 10, 0},
	{0, 3, 40, 20, 0},

	{1, 0, 30, 0, 0},
	{1, 1, 38, 8, 0},
	{1, 2, 45, 16, 0},

	{2, 0, 42, 0, 0},
	{2, 1, 50, 10, 0},

	{3, 0, 52, 0, 0},
};

#ifdef UNUSE_DRM
static bool drm_dp_tps4_supported(const u8 dpcd[DP_RECEIVER_CAP_SIZE])
{
	return dpcd[DP_DPCD_REV] >= 0x14 &&
		dpcd[DP_MAX_DOWNSPREAD] & DP_TPS4_SUPPORTED;
}

static bool drm_dp_tps3_supported(const u8 dpcd[DP_RECEIVER_CAP_SIZE])
{
	return dpcd[DP_DPCD_REV] >= 0x12 &&
		dpcd[DP_MAX_LANE_COUNT] & DP_TPS3_SUPPORTED;
}

static u8 drm_dp_max_lane_count(const u8 dpcd[DP_RECEIVER_CAP_SIZE])
{
	return dpcd[DP_MAX_LANE_COUNT] & DP_MAX_LANE_COUNT_MASK;
}

static bool drm_dp_enhanced_frame_cap(const u8 dpcd[DP_RECEIVER_CAP_SIZE])
{
	return dpcd[DP_DPCD_REV] >= 0x11 &&
		(dpcd[DP_MAX_LANE_COUNT] & DP_ENHANCED_FRAME_CAP);
}

static u8 dp_link_status(const u8 link_status[DP_LINK_STATUS_SIZE], int r)
{
	return link_status[r - DP_LANE0_1_STATUS];
}

static u8 dp_get_lane_status(const u8 link_status[DP_LINK_STATUS_SIZE],
				int lane)
{
	int i = DP_LANE0_1_STATUS + (lane >> 1);
	int s = (lane & 1) * 4;
	u8 l = dp_link_status(link_status, i);
	return (l >> s) & 0xf;
}

static bool drm_dp_clock_recovery_ok(const u8 link_status[DP_LINK_STATUS_SIZE],
				int lane_count)
{
	int lane;
	u8 lane_status;
	for (lane = 0; lane < lane_count; lane++) {
		lane_status = dp_get_lane_status(link_status, lane);
		if ((lane_status & DP_LANE_CR_DONE) == 0)
			return false;
	}
	return true;
}

static bool drm_dp_channel_eq_ok(const u8 link_status[DP_LINK_STATUS_SIZE],
				int lane_count)
{
	u8 lane_align;
	u8 lane_status;
	int lane;
	lane_align = dp_link_status(link_status,
		DP_LANE_ALIGN_STATUS_UPDATED);
	if ((lane_align & DP_INTERLANE_ALIGN_DONE) == 0) {
		return false;
	}
	for (lane = 0; lane < lane_count; lane++) {
		lane_status = dp_get_lane_status(link_status, lane);
		if ((lane_status & DP_CHANNEL_EQ_BITS) != DP_CHANNEL_EQ_BITS)
			return false;
	}
	return true;
}
#endif

static int dptx_link_read_status(struct dptx *dptx)
{
	return dptx_read_bytes_from_dpcd(dptx, DP_LANE0_1_STATUS,
		dptx->link.status, DP_LINK_STATUS_SIZE);
}

static int dptx_link_check_cr_done(struct dptx *dptx, bool *out_done)
{
	int retval;
	u8 byte;

	if (!out_done)
		return -EINVAL;

	*out_done = false;

	retval = dptx_read_dpcd(dptx, DP_TRAINING_AUX_RD_INTERVAL, &byte);
	if (retval)
		return retval;

	udelay(100);    // need refer to DP training spec. to fix this delay

	retval = dptx_link_read_status(dptx);
	if (retval)
		return retval;

	*out_done = drm_dp_clock_recovery_ok(dptx->link.status,
					     dptx->link.lanes);

	return 0;
}

static int dptx_link_check_ch_eq_done(struct dptx *dptx,
				      bool *out_cr_done,
				      bool *out_ch_eq_done)
{
	int retval;
	bool done;
	u8 byte;
	u32 eq = 400;

	if (!out_cr_done || !out_ch_eq_done)
		return -EINVAL;

	*out_cr_done = false;
	*out_ch_eq_done = false;

	retval = dptx_read_dpcd(dptx, DP_TRAINING_AUX_RD_INTERVAL, &byte);
	if (retval)
		return retval;

	switch (byte&0x7f) {
		case 0:
			eq = 400; //400us
			break;
		case 1:
			eq = 4000;
			break;
		case 2:
			eq = 8000;
			break;
		case 3:
			eq = 12000;
			break;
		case 4:
			eq = 16000;
			break;
		default:
			break;
	}

	udelay(eq);

	retval = dptx_link_read_status(dptx);
	if (retval)
		return retval;

	done = drm_dp_clock_recovery_ok(dptx->link.status,
					     dptx->link.lanes);
	if (!done)
		return 0;

	*out_cr_done = true;
	*out_ch_eq_done = drm_dp_channel_eq_ok(dptx->link.status,
					       dptx->link.lanes);

	dptx_dbg(dptx, "%s: CH_EQ_DONE = %d\n", __func__, *out_ch_eq_done);

	return 0;
}

void dptx_link_set_preemp_vswing(struct dptx *dptx)
{
	int i;
	u8 pe = 0, vs = 0;

	for (i = 0; i < dptx->link.lanes; i++) {
		if (dptx->tx_eq_en) {
			pe = dptx->eq_setting.preemp_level;
			vs = dptx->eq_setting.vswing_level;
		} else {
			pe = dptx->link.preemp_level[i];
			vs = dptx->link.vswing_level[i];
		}

		dptx_phy_set_pre_emphasis(dptx, i, pe);
		dptx_phy_set_vswing(dptx, i, vs);
	}

	dptx_link_set_eq(dptx, vs, pe);
}

int dptx_link_training_lanes_set(struct dptx *dptx)
{
	int retval;
	unsigned int i;
	u8 bytes[4] = { 0xff, 0xff, 0xff, 0xff };

	for (i = 0; i < dptx->link.lanes; i++) {
		u8 byte = 0;

		byte |= ((dptx->link.vswing_level[i] <<
			 DP_TRAIN_VOLTAGE_SWING_SHIFT) &
			 DP_TRAIN_VOLTAGE_SWING_MASK);

		if (dptx->link.vswing_level[i] == 3)
			byte |= DP_TRAIN_MAX_SWING_REACHED;

		byte |= ((dptx->link.preemp_level[i] <<
			 DP_TRAIN_PRE_EMPHASIS_SHIFT) &
			 DP_TRAIN_PRE_EMPHASIS_MASK);

		if (dptx->link.preemp_level[i] == 2)
			byte |= DP_TRAIN_MAX_PRE_EMPHASIS_REACHED;

		bytes[i] = byte;
	}

	retval = dptx_write_bytes_to_dpcd(dptx, DP_TRAINING_LANE0_SET, bytes,
					  dptx->link.lanes);
	if (retval)
		return retval;

	return 0;
}

void dptx_link_set_eq(struct dptx *dptx, u8 vs, u8 pe)
{
	int i, count = sizeof(eq_settings)/sizeof(struct dptx_eq_setting);
	u8 main_eq, post_eq, pre_eq;
	bool get_eq_value = false;

	if (dptx->tx_eq_en) {
		main_eq = dptx->eq_setting.tx_eq_main;
		post_eq = dptx->eq_setting.tx_eq_post;
		pre_eq = dptx->eq_setting.tx_eq_pre;
		get_eq_value = true;
	} else {
		for (i = 0; i < count; i++) {
			if (eq_settings[i].vswing_level == vs &&
				eq_settings[i].preemp_level == pe) {
				main_eq = eq_settings[i].tx_eq_main;
				post_eq = eq_settings[i].tx_eq_post;
				pre_eq = eq_settings[i].tx_eq_pre;
				get_eq_value = true;
			}
		}
	}

	if (get_eq_value) {
		regmap_write(dptx->misc_base, DPTX_EXT_TX_EQ_OVRD_G1, 1);
		regmap_write(dptx->misc_base, DPTX_EXT_TX_EQ_OVRD_G2, 1);
		regmap_write(dptx->misc_base, DPTX_EXT_TX_EQ_OVRD_G3, 1);
		regmap_write(dptx->misc_base, DPTX_EXT_TX_EQ_OVRD_G4, 1);

		regmap_write(dptx->misc_base, DPTX_EXT_TX_EQ_MAIN_G1, main_eq);
		regmap_write(dptx->misc_base, DPTX_EXT_TX_EQ_MAIN_G2, main_eq);
		regmap_write(dptx->misc_base, DPTX_EXT_TX_EQ_MAIN_G3, main_eq);
		regmap_write(dptx->misc_base, DPTX_EXT_TX_EQ_MAIN_G4, main_eq);

		regmap_write(dptx->misc_base, DPTX_EXT_TX_EQ_POST_G1, post_eq);
		regmap_write(dptx->misc_base, DPTX_EXT_TX_EQ_POST_G2, post_eq);
		regmap_write(dptx->misc_base, DPTX_EXT_TX_EQ_POST_G3, post_eq);
		regmap_write(dptx->misc_base, DPTX_EXT_TX_EQ_POST_G4, post_eq);

		regmap_write(dptx->misc_base, DPTX_EXT_TX_EQ_PRE_G1, pre_eq);
		regmap_write(dptx->misc_base, DPTX_EXT_TX_EQ_PRE_G2, pre_eq);
		regmap_write(dptx->misc_base, DPTX_EXT_TX_EQ_PRE_G3, pre_eq);
		regmap_write(dptx->misc_base, DPTX_EXT_TX_EQ_PRE_G4, pre_eq);
	}
}

int dptx_link_adjust_drive_settings(struct dptx *dptx, int *out_changed)
{
	int retval, i, lanes;
	u8 byte;
	u8 adj[4] = { 0, };
	int changed = false;

	lanes = dptx->link.lanes;

	switch (lanes) {
	case 4:
		retval = dptx_read_dpcd(dptx, DP_ADJUST_REQUEST_LANE2_3, &byte);
		if (retval)
			return retval;

		adj[2] = byte & 0x0f;
		adj[3] = (byte & 0xf0) >> 4;

		retval = dptx_read_dpcd(dptx, DP_ADJUST_REQUEST_LANE0_1, &byte);
		if (retval)
			return retval;

		adj[0] = byte & 0x0f;
		adj[1] = (byte & 0xf0) >> 4;
		break;
	case 2:
	case 1:
		retval = dptx_read_dpcd(dptx, DP_ADJUST_REQUEST_LANE0_1, &byte);
		if (retval)
			return retval;

		adj[0] = byte & 0x0f;
		adj[1] = (byte & 0xf0) >> 4;
		break;
	default:
		return -EINVAL;
	}

	/* Save the drive settings */
	for (i = 0; i < lanes; i++) {
		u8 vs = adj[i] & 0x3;
		u8 pe = (adj[i] & 0xc) >> 2;

		if (dptx->link.vswing_level[i] != vs)
			changed = true;

		dptx->link.vswing_level[i] = vs;
		dptx->link.preemp_level[i] = pe;
	}

	dptx_link_set_preemp_vswing(dptx);

	retval = dptx_link_training_lanes_set(dptx);
	if (retval)
		return retval;

	if (out_changed)
		*out_changed = changed;

	return 0;
}

static int dptx_link_training_init(struct dptx *dptx,
				   u8 rate,
				   u8 lanes)
{
	u8 sink_max_rate;
	u8 sink_max_lanes;

	if (rate > DPTX_PHYIF_CTRL_RATE_HBR3)
		rate = DPTX_PHYIF_CTRL_RATE_RBR;

	if ((!lanes) || (lanes == 3) || (lanes > 4))
		lanes = 1;

	/* Initialize link parameters */
	if (!dptx->automated_test) {
		memset(dptx->link.preemp_level, 0, sizeof(u8) * 4);
		memset(dptx->link.vswing_level, 0, sizeof(u8) * 4);
	}
	memset(dptx->link.status, 0, DP_LINK_STATUS_SIZE);

	if (!dptx->fixed_lanes) {
		sink_max_lanes = drm_dp_max_lane_count(dptx->rx_caps);

		if (lanes > sink_max_lanes)
			lanes = sink_max_lanes;

		sink_max_rate = dptx->rx_caps[DP_MAX_LINK_RATE];
		sink_max_rate = dptx_bw_to_phy_rate(sink_max_rate);

		if (rate > sink_max_rate)
			rate = sink_max_rate;
	}

	dptx->link.lanes = lanes;
	dptx->link.rate = rate;
	dptx->link.trained = false;

	return 0;
}

int dptx_link_training_pattern_set(struct dptx *dptx, u8 pattern)
{
	int retval;

	retval = dptx_write_dpcd(dptx, DP_TRAINING_PATTERN_SET, pattern);
	if (retval)
		return retval;

	return 0;
}

static int dptx_link_training_start(struct dptx *dptx)
{
	int retval;
	u32 cctl, phyifctrl;
	u8 byte;

	/* Initialize PHY */

	/* Move PHY to P3 to program SSC */
	phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);
	phyifctrl &= ~DPTX_PHYIF_CTRL_LANE_PWRDOWN_MASK;// P0
	phyifctrl |= (3 << DPTX_PHYIF_CTRL_LANE_PWRDOWN_SHIFT);// move phy to P3 state
	dptx_writel(dptx, DPTX_PHYIF_CTRL, phyifctrl);

	retval = dptx_phy_wait_busy(dptx, DPTX_MAX_LINK_LANES);
	if (retval)
		return retval;

	dptx_phy_set_lanes(dptx, dptx->link.lanes);
	dptx_phy_set_rate(dptx, dptx->link.rate);

	phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);
	if (dptx->ssc_en && dptx_sink_enabled_ssc(dptx))
		phyifctrl &= ~DPTX_PHYIF_CTRL_SSC_DIS;
	else
		phyifctrl |= DPTX_PHYIF_CTRL_SSC_DIS;
	dptx_writel(dptx, DPTX_PHYIF_CTRL, phyifctrl);

	retval = dptx_phy_wait_busy(dptx, DPTX_MAX_LINK_LANES);
	if (retval)
		return retval;

	phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);
	phyifctrl &= ~DPTX_PHYIF_CTRL_LANE_PWRDOWN_MASK;// P0
	dptx_writel(dptx, DPTX_PHYIF_CTRL, phyifctrl);

	/* Wait for PHY busy */
	retval = dptx_phy_wait_busy(dptx, DPTX_MAX_LINK_LANES);
	if (retval)
		return retval;

	/* Set PHY_TX_EQ */
	dptx_link_set_preemp_vswing(dptx);

	dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_NONE);
	retval = dptx_link_training_pattern_set(dptx,
						DP_TRAINING_PATTERN_DISABLE);
	if (retval)
		return retval;

	dptx_phy_enable_xmit(dptx, dptx->link.lanes, true);

	retval = dptx_phy_rate_to_bw(dptx->link.rate);
	if (retval < 0)
		return retval;

	byte = retval;
	retval = dptx_write_dpcd(dptx, DP_LINK_BW_SET, byte);
	if (retval)
		return retval;

	byte = dptx->link.lanes;
	cctl = dptx_readl(dptx, DPTX_CCTL);

	if (drm_dp_enhanced_frame_cap(dptx->rx_caps)) {
		byte |= DP_ENHANCED_FRAME_CAP;
		cctl |= DPTX_CCTL_ENH_FRAME_EN;
	} else {
		cctl &= ~DPTX_CCTL_ENH_FRAME_EN;
	}

	dptx_writel(dptx, DPTX_CCTL, cctl);

	retval = dptx_write_dpcd(dptx, DP_LANE_COUNT_SET, byte);
	if (retval)
		return retval;

	/* C10 PHY ... check if SSC is enabled and program DPCD*/
	if (dptx->ssc_en && dptx_sink_enabled_ssc(dptx))
		byte = DP_SPREAD_AMP_0_5;
	else
		byte = 0;

	retval = dptx_write_dpcd(dptx, DP_DOWNSPREAD_CTRL, byte);
	if (retval)
		return retval;

	byte = 1;
	retval = dptx_write_dpcd(dptx, DP_MAIN_LINK_CHANNEL_CODING_SET, byte);
	if (retval)
		return retval;

	retval = dptx_link_training_lanes_set(dptx);
	if (retval)
		return retval;

	return 0;
}

int dptx_link_wait_cr_and_adjust(struct dptx *dptx, bool ch_eq)
{
	int i;
	int retval;
	int changed = 0;
	bool done = false;

	retval = dptx_link_check_cr_done(dptx, &done);
	if (retval)
		return retval;

	if (done)
		return 0;

	/* Try each adjustment setting 5 times */
	for (i = 0; i < 5; i++) {
		retval = dptx_link_adjust_drive_settings(dptx, &changed);
		if (retval)
			return retval;

		/* Reset iteration count if we changed settings */
		if (changed)
			i = 0;

		retval = dptx_link_check_cr_done(dptx, &done);
		if (retval)
			return retval;

		if (done)
			return 0;

		/* TODO check for all lanes? */
		/* Failed and reached the maximum voltage swing */
		if (dptx->link.vswing_level[0] == 3)
			return -EPROTO;
	}

	return -EPROTO;
}

int dptx_link_cr(struct dptx *dptx)
{
	int retval;

	dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_1);

	retval = dptx_link_training_pattern_set(dptx,
						DP_TRAINING_PATTERN_1 | 0x20);
	if (retval)
		return retval;

	return dptx_link_wait_cr_and_adjust(dptx, false);
}

int dptx_link_ch_eq(struct dptx *dptx)
{
	int retval;
	bool cr_done;
	bool ch_eq_done;
	unsigned int pattern;
	unsigned int i;
	u8 dp_pattern;

	switch (dptx->max_rate) {
	case DPTX_PHYIF_CTRL_RATE_HBR3:
		if (drm_dp_tps4_supported(dptx->rx_caps)) {
			pattern = DPTX_PHYIF_CTRL_TPS_4;
			dp_pattern = DP_TRAINING_PATTERN_4;
			break;
		}
	/* Fall through */
	case DPTX_PHYIF_CTRL_RATE_HBR2:
		if (drm_dp_tps3_supported(dptx->rx_caps)) {
			pattern = DPTX_PHYIF_CTRL_TPS_3;
			dp_pattern = DP_TRAINING_PATTERN_3;
			break;
		}
	/* Fall through */
	case DPTX_PHYIF_CTRL_RATE_RBR:
	case DPTX_PHYIF_CTRL_RATE_HBR:
		pattern = DPTX_PHYIF_CTRL_TPS_2;
		dp_pattern = DP_TRAINING_PATTERN_2;
		break;
	default:
		return -EINVAL;
	}

	dptx_phy_set_pattern(dptx, pattern);

	/* TODO this needs to be different for other versions of DPRX */
	if(dp_pattern != DP_TRAINING_PATTERN_4) {
		retval = dptx_link_training_pattern_set(dptx, dp_pattern | 0x20);
	} else {
		retval = dptx_link_training_pattern_set(dptx, dp_pattern);

		dptx_dbg(dptx, "%s:  Enabling scrambling for TPS4\n", __func__);
	}
	if (retval)
		return retval;

	for (i = 0; i < 6; i++) {
		retval = dptx_link_check_ch_eq_done(dptx, &cr_done, &ch_eq_done);
		if (retval)
			return retval;

		dptx->cr_fail = false;

		if (!cr_done) {
			dptx->cr_fail = true;
			return -EPROTO;
		}

		if (ch_eq_done)
			return 0;

		retval = dptx_link_adjust_drive_settings(dptx, NULL);
		if (retval)
			return retval;
	}

	return -EPROTO;
}

int dptx_link_reduce_rate(struct dptx *dptx)
{
	unsigned int rate = dptx->link.rate;

	switch (rate) {
	case DPTX_PHYIF_CTRL_RATE_RBR:
		return -EPROTO;
	case DPTX_PHYIF_CTRL_RATE_HBR:
		rate = DPTX_PHYIF_CTRL_RATE_RBR;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR2:
		rate = DPTX_PHYIF_CTRL_RATE_HBR;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR3:
		rate = DPTX_PHYIF_CTRL_RATE_HBR2;
		break;
	}

	dptx->link.rate = rate;
	return 0;
}

int dptx_link_reduce_lanes(struct dptx *dptx)
{
	unsigned int lanes;

	switch (dptx->link.lanes) {
	case 4:
		lanes = 2;
		break;
	case 2:
		lanes = 1;
		break;
	case 1:
	default:
		return -EPROTO;
	}

	dptx->link.lanes = lanes;
	dptx->link.rate = dptx->max_rate;
	return 0;
}

int dptx_link_training(struct dptx *dptx, u8 rate, u8 lanes)
{
	int retval, retval1, tries = 0;
	u8 byte;
	u32 hpd_sts;

	retval = dptx_link_training_init(dptx, rate, lanes);
	if (retval)
		goto fail;

again:
	dptx_info(dptx, "Starting link training rate:%d lane:%d", dptx->link.rate, dptx->link.lanes);
	retval = dptx_link_training_start(dptx);
	if (retval)
		goto fail;

	retval = dptx_link_cr(dptx);
	if (retval) {
		if (retval == -EPROTO) {
			if (!dptx->fixed_lanes) {
				if (dptx_link_reduce_rate(dptx)) {
					/* TODO If CR_DONE bits for some lanes
					 * are set, we should reduce lanes to
					 * those lanes.
					 */
					if (dptx_link_reduce_lanes(dptx)) {
						retval = -EPROTO;
						goto fail;
					} else {
						/*
						 * Check clock recovery status (LANE0_CR_DONE bit) in
						 * LANE0_1_STATUS DPCD register
						 * and fail training, if clock recovery failed for Lane 0
						 */
						if (!(dptx->link.status[0] & 1))
							goto fail;
					}
				}
			} else {
				tries++;
				if (tries >= 3)
					goto fail;
			}
			dptx_link_training_init(dptx, dptx->link.rate, dptx->link.lanes);
			goto again;
		} else {
			goto fail;
		}
	}

	retval = dptx_link_ch_eq(dptx);
	if (retval) {
		if (retval == -EPROTO) {
			if (!dptx->fixed_lanes) {
				if (!dptx->cr_fail) {
					dptx_err(dptx, "link training failure %0x, %0x", retval, -EPROTO);
					if (dptx->link.lanes == 1) {
						if(dptx_link_reduce_rate(dptx))
							goto fail;

						dptx->link.lanes = dptx->max_lanes;
					 } else {
						dptx_link_reduce_lanes(dptx);
					 }
				} else {
					if (dptx_link_reduce_rate(dptx))
						if (dptx_link_reduce_lanes(dptx)) {
							retval = -EPROTO;
							goto fail;
						}
				}
			} else {
				tries++;
				if (tries >= 3)
					goto fail;
			}
			dptx_link_training_init(dptx, dptx->link.rate, dptx->link.lanes);
			goto again;
		} else {
			goto fail;
		}
	}

	dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_NONE);

	retval = dptx_link_training_pattern_set(dptx, DP_TRAINING_PATTERN_DISABLE);
	if (retval)
		goto fail;

	dptx->link.trained = true;

	/* Branch device detection */
	retval = dptx_read_dpcd(dptx, DP_SINK_COUNT, &byte);
	if (retval)
		return retval;

	retval = dptx_read_dpcd(dptx, 0x2002, &byte);
	if (retval)
		return retval;

	dptx_info(dptx, "Link training succeeded rate=%d lanes=%d trained=%d",
		dptx->link.rate, dptx->link.lanes, dptx->link.trained);
	dptx_info(dptx, "Link/Sink Device status(DPCD 202h~207h): 0x%X,0x%X,0x%X,0x%X,0x%X,0x%X",
		dptx->link.status[0],dptx->link.status[1],dptx->link.status[2],
		dptx->link.status[3],dptx->link.status[4],dptx->link.status[5]);
	dptx_info(dptx, "lane preemp_level: %d,%d,%d,%d",
		 dptx->link.preemp_level[0], dptx->link.preemp_level[1],
		 dptx->link.preemp_level[2], dptx->link.preemp_level[3]);
	dptx_info(dptx, "lane vswing_level: %d,%d,%d,%d",
		 dptx->link.vswing_level[0], dptx->link.vswing_level[1],
		 dptx->link.vswing_level[2], dptx->link.vswing_level[3]);

	return 0;

fail:
	hpd_sts = dptx_readl(dptx, DPTX_HPDSTS);

	if (hpd_sts & DPTX_HPDSTS_STATUS) {
		dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_NONE);
		retval1 = dptx_link_training_pattern_set(dptx, DP_TRAINING_PATTERN_DISABLE);
		if (retval1)
			return retval1;

		dptx_err(dptx, "Link training failed %d\n", retval);
	} else {
		dptx_err(dptx, "Link training failed  as sink is disconnected %d\n", retval);
	}

	return retval;
}

bool dptx_link_check_status(struct dptx *dptx)
{
	int retval;
	u8 byte, bytes[2];

	retval = dptx_read_bytes_from_dpcd(dptx, DP_SINK_COUNT, bytes, 2);
	if (retval)
		return false;

	retval = dptx_link_read_status(dptx);
	if (retval)
		return false;

	byte = dptx->link.status[DP_LANE_ALIGN_STATUS_UPDATED - DP_LANE0_1_STATUS];

	if (!(byte & DP_LINK_STATUS_UPDATED))
		return true;

	return drm_dp_channel_eq_ok(dptx->link.status, dptx->link.lanes);
}
