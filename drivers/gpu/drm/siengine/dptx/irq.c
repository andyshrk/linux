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

#include <drm/drm_crtc_helper.h>
#include <drm/drm_probe_helper.h>
#include "se_dptx.h"

int dptx_handle_hotplug(struct dptx *dptx)
{
	int i, pipe, retval = 0, streams = 0;
	u8 byte, vector, lanes, rate;
	u32 reg, phyifctrl;

	/* Move to P0 */
	phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);
	phyifctrl &= ~DPTX_PHYIF_CTRL_LANE_PWRDOWN_MASK;
	dptx_writel(dptx, DPTX_PHYIF_CTRL, phyifctrl);

	retval = dptx_phy_wait_busy(dptx, DPTX_MAX_LINK_LANES);
	if (retval) {
		dptx_err(dptx, "Timed out waiting for PHY BUSY\n");
		goto out;
	}

	retval = dptx_write_dpcd(dptx, 0x600, 1);
	if (retval)
		goto out;

	retval = dptx_read_dpcd(dptx, DP_MSTM_CAP, &byte);
	if (retval)
		goto out;
	if (byte & DP_MST_CAP) {
		if (dptx->total_streams > 1) {
			retval = dptx_write_dpcd(dptx, DP_MSTM_CTRL, 0x07);
			if (retval)
				goto out;
			dptx->mst = true;
		} else {
			retval = dptx_write_dpcd(dptx, DP_MSTM_CTRL, 0x00);
			if (retval)
				goto out;
			dptx->mst = false;
		}
	} else {
		dptx->mst = false;
	}

	if (dptx->mst) {
		dptx_mst_en(dptx);
		dptx->streams = dptx->total_streams;
	} else {
		dptx_mst_dis(dptx);
		dptx_clear_vcpid_table(dptx);
		dptx->streams = 1;
	}

	if (!dptx->display_timings_fixed) {
		retval = dptx_read_dpcd(dptx, DP_RECEIVE_PORT_0_CAP_0, &byte);
		if (retval)
			byte = 0;
		if (byte & DP_LOCAL_EDID_PRESENT) {
			for (i = 0; i < dptx->total_streams; i++) {
				pipe = dptx->stream_id[i];
				if (dptx->ppls[pipe].edid4drm) {
					kfree(dptx->ppls[pipe].edid4drm);
					dptx->ppls[pipe].edid4drm = NULL;
				}
				dptx->ppls[pipe].edid4drm =
					drm_do_get_edid(&dptx->ppls[pipe].conn.base,
						dptx_read_edid_block, dptx);
			}
		}
	}

	retval = dptx_get_sink_cap_by_dpcd(dptx);
	if (retval)
		goto out;

	retval = dptx_read_dpcd(dptx, DP_DEVICE_SERVICE_IRQ_VECTOR, &vector);
	if (retval)
		goto out;

	/*
	 * The TEST_EDID_READ is asserted on HOTPLUG. Check for it and
	 * handle it here.
	 */
	if (vector & DP_AUTOMATED_TEST_REQUEST) {
		u8 test;

		dptx_info(dptx, "%s: DP_AUTOMATED_TEST_REQUEST", __func__);

		retval = dptx_read_dpcd(dptx, DP_TEST_REQUEST, &test);
		if (retval)
			goto out;

		if (test & DP_TEST_LINK_EDID_READ) {
			u8 checksum = 0;
			u8 blocks = 0;

			blocks = dptx->edid[126];
			checksum = dptx->edid[127 + 128 * blocks];

			retval = dptx_write_dpcd(dptx,
						DP_TEST_EDID_CHECKSUM, checksum);
			if (retval)
				goto out;

			retval = dptx_write_dpcd(dptx, DP_TEST_RESPONSE,
						DP_TEST_EDID_CHECKSUM_WRITE);
			if (retval)
				goto out;
		}
	}

	/* disable fast link training */
	dptx_disable_fast_link_training(dptx);

	/* mute audio */
	dptx_audio_mute(dptx);

	/* disable video stream */
	for (i = 0; i < DPTX_MAX_STREAM_NUM; i++)
		dptx_disable_default_video_stream(dptx, i);

	if (dptx->fixed_rate) {
		rate = dptx->fixed_rate;
		lanes = dptx->fixed_lanes;
	} else {
		rate = dptx->max_rate;
		lanes = dptx->max_lanes;
	}

	if (dptx->fec) {
		reg = dptx_readl(dptx, DPTX_CCTL);
		reg |= DPTX_CCTL_ENH_FRAME_FEC_EN;
		dptx_writel(dptx, DPTX_CCTL, reg);

		retval = dptx_write_dpcd(dptx, DP_FEC_CONFIGURATION, DP_FEC_READY);
		if (retval)
			goto out;
	}

	retval = dptx_link_training(dptx, rate, lanes);
	if (retval)
		goto out;

	msleep(1);

	if (dptx->fec) {
		reg = dptx_readl(dptx, DPTX_CCTL);
		reg |= DPTX_CCTL_ENABLE_FEC;
		dptx_writel(dptx, DPTX_CCTL, reg);
	}

	for (i = 0; i < dptx->streams; i++)
		streams |= 1 << dptx->stream_id[i];
	dptx_update_video_config(dptx, streams);

	if (dptx->mst) {
		dptx_vcpid_init(dptx);
		dptx_set_xmit_act(dptx);
	}

	for (i = 0; i < dptx->streams; i++) {
		pipe = dptx->stream_id[i];
		if (dptx->ppls[pipe].enabled)
			dptx_enable_default_video_stream(dptx, pipe);
	}

out:
	return retval;
}

int dptx_handle_hotunplug(struct dptx *dptx)
{
	int i, pipe, retval;
	u32 phyifctrl, reg;

	if (dptx->fec) {
		/* Disable forward error correction */
		reg = dptx_readl(dptx, DPTX_CCTL);
		reg &= ~DPTX_CCTL_ENABLE_FEC;
		dptx_writel(dptx, DPTX_CCTL, reg);

		msleep(100);
	}

	/* Clear xmit enables */
	phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);
	phyifctrl &= ~DPTX_PHYIF_CTRL_XMIT_EN_MASK;

	/* Move PHY to P3 state */
	phyifctrl |= (3 << DPTX_PHYIF_CTRL_LANE_PWRDOWN_SHIFT);
	dptx_writel(dptx, DPTX_PHYIF_CTRL, phyifctrl);

	retval = dptx_phy_wait_busy(dptx, dptx->link.lanes);
	if (retval) {
		 dptx_err(dptx, "Timed out waiting for PHY BUSY\n");
		 return retval;
	}

	/* Power down all lanes */
	/* TODO */

	for (i = 0; i < dptx->total_streams; i++) {
		pipe = dptx->stream_id[i];
		if (dptx->ppls[pipe].edid4drm) {
			kfree(dptx->ppls[pipe].edid4drm);
			dptx->ppls[pipe].edid4drm = NULL;
		}
	}

	atomic_set(&dptx->sink_request, 0);
	dptx->link.trained = false;

	return 0;
}

static int dptx_set_custom_pattern(struct dptx *dptx)
{
	int retval;
	u8 pattern0 , pattern1, pattern2, pattern3, pattern4, pattern5,
		pattern6, pattern7, pattern8, pattern9;
	u32 custompat0;
	u32 custompat1;
	u32 custompat2;

	retval = dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_7_0, &pattern0);
	if (retval)
		return retval;

	retval = dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_15_8, &pattern1);
	if (retval)
		return retval;

	retval = dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_23_16, &pattern2);
	if (retval)
		return retval;

	retval = dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_31_24, &pattern3);
	if (retval)
		return retval;

	retval = dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_39_32, &pattern4);
	if (retval)
		return retval;

	retval = dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_47_40, &pattern5);
	if (retval)
		return retval;

	retval = dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_55_48, &pattern6);
	if (retval)
		return retval;

	retval = dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_63_56, &pattern7);
	if (retval)
		return retval;

	retval = dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_71_64, &pattern8);
	if (retval)
		return retval;

	retval = dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_79_72, &pattern9);
	if (retval)
		return retval;

	/*
	 *  Calculate 30,30 and 20 bits custom patterns depending on TEST_80BIT_CUSTOM_PATTERN sequence
	 */
	custompat0 = ((((((pattern3 & (0xff >> 2)) << 8) | pattern2) << 8) | pattern1) << 8) | pattern0;
	custompat1 = ((((((((pattern7 & (0xf)) << 8) | pattern6) << 8) | pattern5) << 8) | pattern4) << 2) | ((pattern3 >> 6) & 0x3);
	custompat2 = (((pattern9 << 8) | pattern8) << 4) | ((pattern7 >> 4) & 0xf);

	dptx_writel(dptx, DPTX_CUSTOMPAT0, custompat0);
	dptx_writel(dptx, DPTX_CUSTOMPAT1, custompat1);
	dptx_writel(dptx, DPTX_CUSTOMPAT2, custompat2);

	return 0;
}

static int dptx_adjust_vswing_and_preemphasis(struct dptx *dptx)
{
	int retval, i;
	u8 lane_01, lane_23;
	u8 pe = 0, vs = 0;

	retval = dptx_read_dpcd(dptx, DP_ADJUST_REQUEST_LANE0_1, &lane_01);
	if (retval)
		return retval;

	retval = dptx_read_dpcd(dptx, DP_ADJUST_REQUEST_LANE2_3, &lane_23);
	if (retval)
		return retval;

	for (i = 0; i < dptx->link.lanes; i++) {
		switch (i) {
			case 0:
				pe = (lane_01 & DP_ADJUST_PRE_EMPHASIS_LANE0_MASK)
					>> DP_ADJUST_PRE_EMPHASIS_LANE0_SHIFT;
				vs = (lane_01 & DP_ADJUST_VOLTAGE_SWING_LANE0_MASK)
					>> DP_ADJUST_VOLTAGE_SWING_LANE0_SHIFT;
				break;
			case 1:
				pe = (lane_01 & DP_ADJUST_PRE_EMPHASIS_LANE1_MASK)
					>> DP_ADJUST_PRE_EMPHASIS_LANE1_SHIFT;
				vs = (lane_01 & DP_ADJUST_VOLTAGE_SWING_LANE1_MASK)
					>> DP_ADJUST_VOLTAGE_SWING_LANE1_SHIFT;
				break;
			case 2:
				pe = (lane_23 & DP_ADJUST_PRE_EMPHASIS_LANE0_MASK)
					>> DP_ADJUST_PRE_EMPHASIS_LANE0_SHIFT;
				vs = (lane_23 & DP_ADJUST_VOLTAGE_SWING_LANE0_MASK)
					>> DP_ADJUST_VOLTAGE_SWING_LANE0_SHIFT;
				break;
			case 3:
				pe = (lane_23 & DP_ADJUST_PRE_EMPHASIS_LANE1_MASK)
					>> DP_ADJUST_PRE_EMPHASIS_LANE1_SHIFT;
				vs = (lane_23 & DP_ADJUST_VOLTAGE_SWING_LANE1_MASK)
					>> DP_ADJUST_VOLTAGE_SWING_LANE1_SHIFT;
				break;
		default:
			break;
		}

		dptx_phy_set_pre_emphasis(dptx, i, pe);
		dptx_phy_set_vswing(dptx, i, vs);
	}

	dptx_link_set_eq(dptx, vs, pe);

	return 0;
}

static int dptx_handle_test_phy_pattern(struct dptx *dptx)
{
	u8 pattern;
	int retval;
	u8 pattern_set;
	u8 zero_bytes[4] = {0, 0, 0, 0};

	retval = dptx_read_dpcd(dptx, DP_PHY_TEST_PATTERN, &pattern);
	if (retval)
		return retval;

	pattern &= DP_PHY_TEST_PATTERN_SEL_MASK;

	switch (pattern) {
	case DP_PHY_TEST_PATTERN_NONE:
		retval = dptx_adjust_vswing_and_preemphasis(dptx);
		if (retval)
			return retval;
		dptx_info(dptx, "No test pattern selected\n");
		dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_NONE);

		retval = dptx_read_dpcd(dptx, DP_TRAINING_PATTERN_SET, &pattern_set);
		if (retval)
			return retval;
		dptx_write_bytes_to_dpcd(dptx, DP_LINK_QUAL_LANE0_SET, zero_bytes, DPTX_MAX_LINK_LANES);
		dptx_write_dpcd(dptx, DP_TRAINING_PATTERN_SET, pattern_set);
		dptx_write_bytes_to_dpcd(dptx, DP_TRAINING_LANE0_SET, zero_bytes, DPTX_MAX_LINK_LANES);
		break;
	case DP_PHY_TEST_PATTERN_D10_2:
		retval = dptx_adjust_vswing_and_preemphasis(dptx);
		if (retval)
			return retval;
		dptx_info(dptx, "D10.2 without scrambling test phy pattern\n");
		dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_1);
		break;
	case DP_PHY_TEST_PATTERN_ERROR_COUNT:
		retval = dptx_adjust_vswing_and_preemphasis(dptx);
		if (retval)
			return retval;
		dptx_info(dptx, "Symbol error measurement count test phy pattern\n");
		dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_SYM_ERM);
		break;
	case DP_PHY_TEST_PATTERN_PRBS7:
		retval = dptx_adjust_vswing_and_preemphasis(dptx);
		if (retval)
			return retval;
		dptx_info(dptx, "PRBS7 test phy pattern\n");
		dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_PRBS7);
		break;
	case DP_PHY_TEST_PATTERN_80BIT_CUSTOM:
		retval = dptx_adjust_vswing_and_preemphasis(dptx);
		if (retval)
			return retval;
		dptx_info(dptx, "80-bit custom pattern transmitted test phy pattern\n");

		retval = dptx_set_custom_pattern(dptx);
		if (retval)
			return retval;
		dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_CUSTOM80);
		break;
	case DP_PHY_TEST_PATTERN_CP2520:
		retval = dptx_adjust_vswing_and_preemphasis(dptx);
		if (retval)
			return retval;
		dptx_info(dptx, "CP2520_1 - HBR2 Compliance EYE pattern\n");
		dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_CP2520_1);
		break;
	/* CP2520_2 */
	case 6:
		retval = dptx_adjust_vswing_and_preemphasis(dptx);
		if (retval)
			return retval;
		dptx_info(dptx, "CP2520_2 - pattern\n");
		dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_CP2520_2);
		break;
	/* CP2520_3_TPS4 */
	case 7:
		retval = dptx_adjust_vswing_and_preemphasis(dptx);
		if (retval)
			return retval;
		dptx_info(dptx, "DP_TEST_PHY_PATTERN_CP2520_3_TPS4 - pattern\n");
		dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_4);
		break;
	default:
		dptx_info(dptx, "Invalid TEST_PHY_PATTERN\n");
		return -EINVAL;
	}

	//retval = dptx_write_dpcd(dptx, DP_TEST_RESPONSE, DP_TEST_ACK);

	return retval;
}

static int dptx_handle_test_link_training(struct dptx *dptx)
{
	int retval;
	u8 lanes;
	u8 rate;
	u32 phyifctrl;
	u8 saved_fixed_rate;
	u8 saved_fixed_lanes;
	u8 saved_max_rate;
	u8 saved_max_lanes;
	u8 saved_ssc_en;

	/* Move to P0 */
	phyifctrl = dptx_readl(dptx, DPTX_PHYIF_CTRL);
	phyifctrl &= ~DPTX_PHYIF_CTRL_LANE_PWRDOWN_MASK;
	dptx_writel(dptx, DPTX_PHYIF_CTRL, phyifctrl);

	retval = dptx_read_dpcd(dptx, DP_TEST_LINK_RATE, &rate);
	if (retval)
		return retval;

	retval = dptx_bw_to_phy_rate(rate);
	if (retval < 0)
		return retval;
	rate = retval;

	retval = dptx_read_dpcd(dptx, DP_TEST_LANE_COUNT, &lanes);
	if (retval)
		return retval;

	dptx_info(dptx, "%s: Strating link training rate=%d, lanes=%d\n",
		 __func__, rate, lanes);

	saved_fixed_rate = dptx->fixed_rate;
	saved_fixed_lanes = dptx->fixed_lanes;
	saved_max_rate = dptx->max_rate;
	saved_max_lanes = dptx->max_lanes;
	saved_ssc_en = dptx->ssc_en;
	dptx->fixed_rate = 0;
	dptx->fixed_lanes = 0;
	dptx->max_rate = rate;
	dptx->max_lanes = lanes;
	dptx->ssc_en = true;
	dptx->automated_test = true;

	retval = dptx_link_training(dptx, rate, lanes);
	if (retval)
		dptx_err(dptx, "Link training failed %d\n", retval);
	else
		dptx_dbg(dptx, "Link training succeeded\n");

	dptx->automated_test = false;
	dptx->fixed_rate = saved_fixed_rate;
	dptx->fixed_lanes = saved_fixed_lanes;
	dptx->max_rate = saved_max_rate;
	dptx->max_lanes = saved_max_lanes;
	dptx->ssc_en = saved_ssc_en;

	return retval;
}

static int dptx_handle_automated_test_request(struct dptx *dptx)
{
	int retval;
	u8 test;

	retval = dptx_read_dpcd(dptx, DP_TEST_REQUEST, &test);
	if (retval)
		return retval;

	if (test & DP_TEST_LINK_TRAINING) {
		dptx_info(dptx, "%s: DP_TEST_LINK_TRAINING\n", __func__);
		retval = dptx_write_dpcd(dptx, DP_TEST_RESPONSE, DP_TEST_ACK);
		if (retval)
			return retval;

		retval = dptx_handle_test_link_training(dptx);
		if (retval)
			return retval;
	}

	if (test & DP_TEST_LINK_EDID_READ) {
		/* Invalid, this should happen on HOTPLUG */
		dptx_info(dptx, "%s:DP_TEST_LINK_EDID_READ\n", __func__);
		return -ENOTSUPP;
	}

	if (test & DP_TEST_LINK_PHY_TEST_PATTERN) {
		dptx_info(dptx, "%s:DP_TEST_LINK_PHY_TEST_PATTERN\n", __func__);
		retval = dptx_handle_test_phy_pattern(dptx);
		if (retval)
			return retval;
	}
	return 0;
}

int dptx_handle_sink_request(struct dptx *dptx)
{
	int i, retval, streams = 0;
	u8 vector, lanes, rate;

	if (!dptx_link_check_status(dptx)) {
		if (dptx->fixed_rate) {
			rate = dptx->fixed_rate;
			lanes = dptx->fixed_lanes;
		} else {
			rate = dptx->max_rate;
			lanes = dptx->max_lanes;
		}

		dptx_link_training(dptx, rate, lanes);

		for (i = 0; i < dptx->streams; i++)
			streams |= 1 << dptx->stream_id[i];
		dptx_update_video_config(dptx, streams);
	}

	retval = dptx_read_dpcd(dptx, DP_DEVICE_SERVICE_IRQ_VECTOR, &vector);
	if (retval)
		return retval;

    /* TODO handle sink interrupts */
	if (!vector)
		return 0;

	/* clear IRQ vector */
	dptx_write_dpcd(dptx, DP_DEVICE_SERVICE_IRQ_VECTOR, vector);

	if (vector & DP_REMOTE_CONTROL_COMMAND_PENDING) {
		/* TODO */
		dptx_warn(dptx,
			"%s: DP_REMOTE_CONTROL_COMMAND_PENDING: Not yet implemented",
			__func__);
	}

	if (vector & DP_AUTOMATED_TEST_REQUEST) {
		dptx_info(dptx, "%s: DP_AUTOMATED_TEST_REQUEST", __func__);
		retval = dptx_handle_automated_test_request(dptx);
		if (retval) {
			dptx_err(dptx, "Automated test request failed\n");
			if (retval == -ENOTSUPP) {
				retval = dptx_write_dpcd(dptx, DP_TEST_RESPONSE,
							 DP_TEST_NAK);
				if (retval)
					return retval;
			}
		}
	}

	if (vector & DP_CP_IRQ) {
		dptx_warn(dptx, "%s: DP_CP_IRQ", __func__);
	#if 0
		retval = dptx_write_dpcd(dptx, DP_DEVICE_SERVICE_IRQ_VECTOR,
					 DP_CP_IRQ);
		reg = dptx_readl(dptx, DPTX_HDCP_CONFIG);
		reg |= DPTX_CFG_CP_IRQ;

		dptx_writel(dptx, DPTX_HDCP_CONFIG, reg);
		reg = dptx_readl(dptx, DPTX_HDCP_CONFIG);
		dptx_warn(dptx, "%s: DP_CP_IRQ1--- 0x%x", __func__, reg);

		reg &= ~DPTX_CFG_CP_IRQ;
		dptx_writel(dptx, DPTX_HDCP_CONFIG, reg);
		if (retval)
			return retval;
	#else
		retval = -ENOTSUPP;
	#endif
	}

	if (vector & DP_MCCS_IRQ) {
		/* TODO */
		dptx_warn(dptx, "%s: DP_MCCS_IRQ: Not yet implemented", __func__);
		retval = -ENOTSUPP;
	}

	if (vector & DP_UP_REQ_MSG_RDY) {
		dptx_warn(dptx, "%s: DP_UP_REQ_MSG_RDY", __func__);
		retval = -ENOTSUPP;
	}

	if (vector & DP_SINK_SPECIFIC_IRQ) {
		/* TODO */
		dptx_warn(dptx, "%s: DP_SINK_SPECIFIC_IRQ: Not yet implemented", __func__);
		retval = -ENOTSUPP;
	}

	return retval;
}

irqreturn_t dptx_irq(int irq, void *dev)
{
	irqreturn_t retval = IRQ_HANDLED;
	struct dptx *dptx = dev;
	u32 ists;
	static int hpd_plug_cnt = 0;

	ists = dptx_readl(dptx, DPTX_ISTS);
	dptx_dbg(dptx, "%s: >>>> ISTS=0x%08x\n", __func__, ists);
	dptx_writel(dptx, DPTX_ISTS, ists);

	if (!(ists & DPTX_ISTS_ALL_INTR)) {
		retval = IRQ_NONE;
		goto done;
	}

	if (ists & DPTX_ISTS_VIDEO_FIFO_OVERFLOW_STREAM0) {
		//TODO
	}

	if (ists & DPTX_ISTS_VIDEO_FIFO_OVERFLOW_STREAM1) {
		//TODO
	}

	if (ists & DPTX_ISTS_HPD) {
		u32 hpdsts;

		hpdsts = dptx_readl(dptx, DPTX_HPDSTS);
		dptx_dbg(dptx, "%s: HPDSTS = 0x%08x\n", __func__, hpdsts);

		if (hpdsts & DPTX_HPDSTS_IRQ) {
			dptx_writel(dptx, DPTX_HPDSTS, DPTX_HPDSTS_IRQ);
			atomic_set(&dptx->sink_request, 1);
			retval = IRQ_WAKE_THREAD;
		}

		if (hpdsts & DPTX_HPDSTS_HOT_PLUG) {
			hpd_plug_cnt++;
			dptx_writel(dptx, DPTX_HPDSTS, DPTX_HPDSTS_HOT_PLUG);
			if (!dptx->no_hpd_irq || hpd_plug_cnt > 1) {
				atomic_set(&dptx->hpd, 1);
				retval = IRQ_WAKE_THREAD;
			}
		}

		if (hpdsts & DPTX_HPDSTS_HOT_UNPLUG) {
			dptx_writel(dptx, DPTX_HPDSTS, DPTX_HPDSTS_HOT_UNPLUG);
			atomic_set(&dptx->hpd, 1);
			retval = IRQ_WAKE_THREAD;
		}

		if (hpdsts & 0x80)
			dptx_writel(dptx, DPTX_HPDSTS, 0x80 | DPTX_HPDSTS_HOT_UNPLUG);
	}

done:
	return retval;
}

irqreturn_t dptx_threaded_irq(int irq, void *dev)
{
	int i, pipe, retval;
	struct dptx *dptx = dev;
	u32 hpdsts, next_status;
	struct drm_connector *connector;
	enum drm_connector_status old_status;
	bool update_event = false;

	mutex_lock(&dptx->lock);

	hpdsts = dptx_readl(dptx, DPTX_HPDSTS);
	dptx_dbg(dptx, "%s: HPDSTS = 0x%08x\n", __func__, hpdsts);

	if (atomic_read(&dptx->hpd)) {
		atomic_set(&dptx->hpd, 0);
		next_status = !!(hpdsts & DPTX_HPDSTS_STATUS);
		if (next_status == atomic_read(&dptx->connect)) {
			mutex_unlock(&dptx->lock);
			return IRQ_HANDLED;
		}
		atomic_set(&dptx->connect, next_status);

		if (hpdsts & DPTX_HPDSTS_STATUS)
			retval = dptx_handle_hotplug(dptx);
		else
			retval = dptx_handle_hotunplug(dptx);

		if (retval) {
			atomic_set(&dptx->connect, 0);
			mutex_unlock(&dptx->lock);
			return IRQ_HANDLED;
		}
	}

	if (atomic_read(&dptx->sink_request)) {
		atomic_set(&dptx->sink_request, 0);
		retval = dptx_handle_sink_request(dptx);
		if (retval)
			dptx_err(dptx, "failed to handle sink request %d\n", retval);
	}

	mutex_unlock(&dptx->lock);

	for (i = 0; i < dptx->streams; i++) {
		pipe = dptx->stream_id[i];
		connector = &dptx->ppls[pipe].conn.base;
		old_status = connector->status;
		connector->status = connector->funcs->detect(connector, false);
		if (old_status != connector->status)
			update_event = true;
	}

	if (update_event)
		drm_kms_helper_hotplug_event(dptx->drm_dev);

	return IRQ_HANDLED;
}

