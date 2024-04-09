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

/* MISC0: Synchronous mode of operation.
 * Pixel source reference clock and link reference clock are the same.
 */
#define SYNC_MODE_OPERATION
#define MSA_NVID				(0x008000)

void dptx_video_params_reset(struct dptx *dptx, int stream)
{
	struct video_params *params = &dptx->ppls[stream].vparams;

	params->pix_enc = RGB;
	params->mode = 0; /* 1280x720p @ 59.94/60Hz 16:9 */
	params->bpc = COLOR_DEPTH_8;
	params->colorimetry = ITU601;
	params->dynamic_range = CEA;
	params->aver_bytes_per_tu = 30;
	params->aver_bytes_per_tu_frac = 0;
	params->init_threshold = 15;
	params->pattern_mode = RAMP;
	params->refresh_rate = 60000;
	params->video_format = VCEA;
}

static int dptx_get_link_rate(int rate)
{
	int link_clk;
	int link_rate;

	switch (rate) {
	case DPTX_PHYIF_CTRL_RATE_RBR:
		link_rate = 162;
		link_clk = 40500;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR:
		link_rate = 270;
		link_clk = 67500;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR2:
		link_rate = 540;
		link_clk = 135000;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR3:
		link_rate = 810;
		link_clk = 202500;
		break;
	default:
        return -EINVAL;
        break;
	}

	return link_rate;
}

void dptx_video_set_sink_col(struct dptx *dptx, int stream)
{
	u32 reg_msa2;
	u8 col_mapping;
	u8 colorimetry;
	u8 dynamic_range;
	struct video_params *vparams;
	enum pixel_enc_type pix_enc;

	vparams = &dptx->ppls[stream].vparams;
	pix_enc = vparams->pix_enc;
	colorimetry = vparams->colorimetry;
	dynamic_range = vparams->dynamic_range;

	reg_msa2 = dptx_readl(dptx, DPTX_VIDEO_MSA2_N(stream));
	reg_msa2 &= ~DPTX_VIDEO_VMSA2_COL_MASK;

	col_mapping = 0;

	/* According to Table 2-94 of DisplayPort spec 1.3 */
	switch (pix_enc) {
	case RGB:
		if (dynamic_range == CEA)
			col_mapping = 4;
		else if (dynamic_range == VESA)
			col_mapping = 0;
		break;
	case YCBCR422:
		if (colorimetry == ITU601)
			col_mapping = 5;
		else if (colorimetry == ITU709)
			col_mapping = 13;
		break;
	case YCBCR444:
		if (colorimetry == ITU601)
			col_mapping = 6;
		else if (colorimetry == ITU709)
			col_mapping = 14;
		break;
	case RAW:
		col_mapping = 1;
		break;
	case YCBCR420:
	case YONLY:
		break;
	}

	reg_msa2 |= (col_mapping << DPTX_VIDEO_VMSA2_COL_SHIFT);
	dptx_writel(dptx, DPTX_VIDEO_MSA2_N(stream), reg_msa2);
}

void dptx_video_set_sink_bpc(struct dptx *dptx, int stream)
{
	u32 reg_msa2, reg_msa3;
	u8 bpc_mapping = 0, bpc = 0;
	struct video_params *vparams;
#ifdef UNUSE_DRM
	struct display_timing *dt;
#else
	struct drm_display_mode *mode;
#endif
	enum pixel_enc_type pix_enc;
	u32 msa_mvid;

	vparams = &dptx->ppls[stream].vparams;
#ifdef UNUSE_DRM
	dt = &dptx->ppls[stream].dt;
#else
	mode = &dptx->ppls[stream].mode;
#endif
	pix_enc = vparams->pix_enc;
	bpc = vparams->bpc;

	reg_msa2 = dptx_readl(dptx, DPTX_VIDEO_MSA2_N(stream));
	reg_msa3 = dptx_readl(dptx, DPTX_VIDEO_MSA3_N(stream));

	reg_msa2 &= ~(DPTX_VIDEO_VMSA2_BPC_MASK | DPTX_VIDEO_VMSA2_MVID_MASK);
	reg_msa3 &= ~DPTX_VIDEO_VMSA3_PIX_ENC_MASK;

	switch (pix_enc) {
	case RGB:
		if (bpc == COLOR_DEPTH_6)
			bpc_mapping = 0;
		else if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 1;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 2;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 3;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 4;
		break;

	case YCBCR444:
		if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 1;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 2;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 3;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 4;
		break;

	case YCBCR422:
		if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 1;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 2;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 3;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 4;
		break;

	case YCBCR420:
		reg_msa3 |= 1 << DPTX_VIDEO_VMSA3_PIX_ENC_YCBCR420_SHIFT;
		break;

	case YONLY:
		/* According to Table 2-94 of DisplayPort spec 1.3 */
		reg_msa3 |= 1 << DPTX_VIDEO_VMSA3_PIX_ENC_SHIFT;

		if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 1;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 2;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 3;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 4;
		break;

	case RAW:
		 /* According to Table 2-94 of DisplayPort spec 1.3 */
		reg_msa3 |= (1 << DPTX_VIDEO_VMSA3_PIX_ENC_SHIFT);

		if (bpc == COLOR_DEPTH_6)
			bpc_mapping = 1;
		else if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 3;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 4;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 5;
		else if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 7;
		break;
	}

	if (dptx->dsc)
		reg_msa2 |= (1 << DPTX_VIDEO_VMSA2_BPC_SHIFT); // DSC 10bpc
	else
		reg_msa2 |= (bpc_mapping << DPTX_VIDEO_VMSA2_BPC_SHIFT);

#ifdef SYNC_MODE_OPERATION
	#ifdef UNUSE_DRM
		msa_mvid = ((dt->pixelclock.typ / (1000 * 100)) * MSA_NVID) /
			(dptx_get_link_rate(dptx->link.rate) * 10);
	#else
		msa_mvid = ((mode->clock / 100) * MSA_NVID) /
	    	(dptx_get_link_rate(dptx->link.rate) * 10);
	#endif
	reg_msa2 |= DPTX_VIDEO_VMSA2_MISC0_SYNCMODE_EN |
		(msa_mvid << DPTX_VIDEO_VMSA2_MVID_SHIFT);
#else
    //If the clocks are asynchronous, controller internally calculates MSA_MVID & MSA_NVID.
	reg_msa2 &= ~DPTX_VIDEO_VMSA2_MISC0_SYNCMODE_EN;
#endif
	reg_msa3 = MSA_NVID;

	dptx_writel(dptx, DPTX_VIDEO_MSA2_N(stream), reg_msa2);
	dptx_writel(dptx, DPTX_VIDEO_MSA3_N(stream), reg_msa3);
	//dptx_video_set_sink_col(dptx, stream);
}

u8 dptx_calculate_dsc_init_threshold(struct dptx* dptx, int stream)
{
	int link_pixel_clock_ratio;
	int pixle_push_rate;
	int lanes;
	int tu;
	int slot_count;
	int fec_slot_count;
	int link_clk;
	int pixel_clk;
	u16 dsc_bpp;
	u8 rate;

	tu = dptx->ppls[stream].vparams.aver_bytes_per_tu;
	lanes = dptx->link.lanes;
	dsc_bpp = dptx->ppls[stream].vparams.dsc_bpp;

	if(dptx->fec) {
		if(lanes == 1)
			fec_slot_count = 13;
		else
			fec_slot_count = 7;
	} else {
		fec_slot_count = 0;
	}

	pixle_push_rate = (8 / dsc_bpp) * lanes;
	if(tu > 0)
		slot_count = tu + 1 + fec_slot_count;
	else
		slot_count = tu + fec_slot_count;

	slot_count = ROUND_UP_TO_NEAREST(slot_count, 4);

	pixel_clk = dptx->ppls[stream].vparams.mdtd.pixel_clock;

	rate = dptx->link.rate;

	switch (rate) {
	case DPTX_PHYIF_CTRL_RATE_RBR:
		link_clk = 40500;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR:
		link_clk = 67500;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR2:
		link_clk = 135000;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR3:
		link_clk = 202500;
		break;
	default:
		return -EINVAL;
	}

	link_pixel_clock_ratio = link_clk / pixel_clk;

	return (pixle_push_rate * link_pixel_clock_ratio * slot_count);
}

int dptx_video_ts_calculate(struct dptx *dptx, int lane_num, int rate,
			    int bpc, int encoding, int pixel_clock, int stream)
{
	struct video_params *vparams;
	struct dtd *mdtd;
	int link_rate;
	int link_clk;
	int retval = 0;
	int ts;
	int T1;
	int T2;
	int tu;
	int tu_frac;
	int color_dep;

	vparams = &dptx->ppls[stream].vparams;
	mdtd = &vparams->mdtd;

	switch (rate) {
	case DPTX_PHYIF_CTRL_RATE_RBR:
		link_rate = 162;
		link_clk = 40500;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR:
		link_rate = 270;
		link_clk = 67500;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR2:
		link_rate = 540;
		link_clk = 135000;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR3:
		link_rate = 810;
		link_clk = 202500;
		break;
	default:
		return -EINVAL;
	}

	switch (bpc) {
	case COLOR_DEPTH_6:
		color_dep = 18;
		break;
	case COLOR_DEPTH_8:
		if (encoding == YCBCR420)
			color_dep  = 12;
		else if (encoding == YCBCR422)
			color_dep = 16;
		else if (encoding == YONLY)
			color_dep = 8;
		else
			color_dep = 24;
		break;
	case COLOR_DEPTH_10:
		if (encoding == YCBCR420)
			color_dep = 15;
		else if (encoding == YCBCR422)
			color_dep = 20;
		else if (encoding  == YONLY)
			color_dep = 10;
		else
			color_dep = 30;
		break;
	case COLOR_DEPTH_12:
		if (encoding == YCBCR420)
			color_dep = 18;
		else if (encoding == YCBCR422)
			color_dep = 24;
		else if (encoding == YONLY)
			color_dep = 12;
		else
			color_dep = 36;
		break;
	case COLOR_DEPTH_16:
		if (encoding == YCBCR420)
			color_dep = 24;
		else if (encoding == YCBCR422)
			color_dep = 32;
		else if (encoding == YONLY)
			color_dep = 16;
		else
			color_dep = 48;
		break;
	default:
		color_dep = 18;
		break;
	}

	// Calculate average_bytes_per_tu based on compressed bpp
	if (dptx->dsc)
		color_dep = vparams->dsc_bpp;

	ts = (8 * color_dep * pixel_clock) / (lane_num * link_rate);

	tu  = ts / 1000;
	if (tu >= 65)
		return -EINVAL;

	tu_frac = ts / 100 - tu * 10;

	dptx_dbg(dptx, "%s stream:%d color_dep:%d, pixel_clock:%d, lane_num:%d link_rate:%d\n",
			__func__, stream, color_dep, pixel_clock, lane_num, link_rate);
	// Calculate init_threshold for DSC mode
	if (dptx->dsc) {
		vparams->init_threshold = dptx_calculate_dsc_init_threshold(dptx, stream);
		if (vparams->init_threshold < 32)
			vparams->init_threshold = 32;
	// Calculate init_threshold for non DSC mode
	} else {
		T1 = 0;
		T2 = 0;
		//Single Pixel Mode
		if (dptx->multipixel == DPTX_MP_SINGLE_PIXEL) {
			if (tu < 6)
				vparams->init_threshold = 32;
			else if (mdtd->h_blanking <= 40 && encoding == YCBCR420)
				vparams->init_threshold = 3;
			else if (mdtd->h_blanking <= 80  && encoding != YCBCR420)
				vparams->init_threshold = 12;
			else
				vparams->init_threshold = 16;

		//Multiple Pixel Mode
		} else {
			switch (bpc) {
			case COLOR_DEPTH_6:
				T1 = (4 * 1000 / 9) * lane_num;
				break;
			case COLOR_DEPTH_8:
				if (encoding == YCBCR422)
					T1 = (1000 / 2) * lane_num;
				else if (encoding == YONLY)
					T1 = lane_num * 1000;
				else
					if (dptx->multipixel == DPTX_MP_DUAL_PIXEL)
						T1 = (1000 / 3) * lane_num;
					else
						T1 = (3000 / 16) * lane_num;
				break;
			case COLOR_DEPTH_10:
				if (encoding == YCBCR422)
					T1 = (2000 / 5) * lane_num;
				else if (encoding == YONLY)
					T1 = (4000 / 5) * lane_num;
				else
					T1 = (4000 / 15) * lane_num;
				break;
			case COLOR_DEPTH_12:
				if (encoding == YCBCR422)
					if (dptx->multipixel == DPTX_MP_DUAL_PIXEL)
						T1 = (1000 / 6) * lane_num;
					else
						T1 = (1000 / 3) * lane_num;
				else if (encoding == YONLY)
					T1 = (2000 / 3) * lane_num;
				else
					T1 = (2000 / 9) * lane_num;
				break;
			case COLOR_DEPTH_16:
				if (encoding == YONLY)
					T1 = (1000 / 2) * lane_num;
				if ((encoding != YONLY) && (encoding != YCBCR422) &&
					(dptx->multipixel == DPTX_MP_DUAL_PIXEL))
					T1 = (1000 / 6) * lane_num;
				else
					T1 = (1000 / 4) * lane_num;
				break;
			default:
				dptx_err(dptx, "Invalid param BPC = %d\n" , bpc);
				return -EINVAL;
			}

			if (encoding == YCBCR420)
				pixel_clock = pixel_clock / 2;

			T2 = (link_clk * 1000 /  pixel_clock);

			vparams->init_threshold = T1 * T2 * tu / (1000*1000);
		}
	}

	if (dptx->mst) {
		vparams->init_threshold = 16;
		tu_frac = tu_frac * 64 / 10;
	}

	vparams->aver_bytes_per_tu = tu;
	vparams->aver_bytes_per_tu_frac = tu_frac;

	dptx_info(dptx, "%s stream:%d ts = 0x%x, tu = 0x%x, tu_frac = 0x%x init_threshold:0x%x\n",
			__func__, stream, ts, tu, tu_frac, vparams->init_threshold);

	return retval;
}

void dptx_video_ts_change(struct dptx *dptx, int stream)
{
	u32 reg;
	struct video_params *vparams;

	vparams = &dptx->ppls[stream].vparams;

	reg = dptx_readl(dptx, DPTX_VIDEO_CONFIG5_N(stream));
	reg = reg & (~DPTX_VIDEO_CONFIG5_TU_MASK);
	reg = reg | (vparams->aver_bytes_per_tu << DPTX_VIDEO_CONFIG5_TU_SHIFT);
	if (dptx->mst) {
		reg = reg & (~DPTX_VIDEO_CONFIG5_TU_FRAC_MASK_MST);
		reg = reg | (vparams->aver_bytes_per_tu_frac <<
			DPTX_VIDEO_CONFIG5_TU_FRAC_SHIFT_MST);
	} else {
		reg = reg & (~DPTX_VIDEO_CONFIG5_TU_FRAC_MASK_SST);
		reg = reg | (vparams->aver_bytes_per_tu_frac <<
			DPTX_VIDEO_CONFIG5_TU_FRAC_SHIFT_SST);
	}
	reg = reg & (~DPTX_VIDEO_CONFIG5_INIT_THRESHOLD_MASK);
	reg = reg | (vparams->init_threshold <<
		DPTX_VIDEO_CONFIG5_INIT_THRESHOLD_SHIFT);

	dptx_dbg(dptx, "%s %d stream:%d tu:%d frac:%d init:%d reg:0x%x\n", __func__, __LINE__,
		stream, vparams->aver_bytes_per_tu, vparams->aver_bytes_per_tu_frac,
		vparams->init_threshold, reg);
	dptx_writel(dptx, DPTX_VIDEO_CONFIG5_N(stream), reg);
}

void dptx_video_set_core_bpc(struct dptx *dptx, int stream)
{
	u32 reg;
	u8 bpc_mapping = 0, bpc = 0;
	enum pixel_enc_type pix_enc;
	struct video_params *vparams;

	vparams = &dptx->ppls[stream].vparams;
	bpc = vparams->bpc;
	pix_enc = vparams->pix_enc;

	reg = dptx_readl(dptx, DPTX_VSAMPLE_CTRL_N(stream));
	reg &= ~DPTX_VSAMPLE_CTRL_VMAP_BPC_MASK;

	switch (pix_enc) {
	case RGB:
		if (bpc == COLOR_DEPTH_6)
			bpc_mapping = 0;
		else if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 1;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 2;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 3;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 4;
		break;
	case YCBCR444:
		if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 5;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 6;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 7;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 8;
		break;
	case YCBCR422:
		if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 9;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 10;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 11;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 12;
		break;
	case YCBCR420:
		if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 13;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 14;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 15;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 16;
		break;
	case YONLY:
		if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 17;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 18;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 19;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 20;
		break;
	case RAW:
		if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 23;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 24;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 25;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 27;
		break;
	}

	/* TODO only for RGB 8bpc */
	if (dptx->dsc)
		reg |= (1 << DPTX_VG_CONFIG1_BPC_SHIFT);
	else
		reg |= (bpc_mapping << DPTX_VSAMPLE_CTRL_VMAP_BPC_SHIFT);
	dptx_writel(dptx, DPTX_VSAMPLE_CTRL_N(stream), reg);
}

int dptx_calculate_hblank_interval(struct dptx* dptx, int stream)
{
	struct video_params *vparams;
	int pixel_clk;
	u16 h_blank;
	u32 link_clk;
	u8 rate;
	int hblank_interval;
	u8 tu;

	vparams = &dptx->ppls[stream].vparams;
	pixel_clk = vparams->mdtd.pixel_clock;
	h_blank = vparams->mdtd.h_blanking;
	rate = dptx->link.rate;
	tu = vparams->aver_bytes_per_tu;

	switch (rate) {
	case DPTX_PHYIF_CTRL_RATE_RBR:
		link_clk = 40500;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR:
		link_clk = 67500;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR2:
		link_clk = 135000;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR3:
		link_clk = 202500;
		break;
	default:
		return -EINVAL;
	}

	if (dptx->mst)
		hblank_interval = (h_blank / 16) * tu * link_clk * 4 / pixel_clk + 1;
	else
		hblank_interval = h_blank * link_clk / pixel_clk;

	return hblank_interval;
}

void dptx_video_core_config(struct dptx *dptx, int stream)
{
	u32 reg = 0;
	//u8 vmode;

	struct video_params *vparams;
	struct dtd *mdtd;

	vparams = &dptx->ppls[stream].vparams;
	mdtd = &vparams->mdtd;
	//vmode = vparams->mode;

	dptx_video_set_core_bpc(dptx, stream);

	/* Configure video_msa1 register */
	reg = 0;
	reg |= (mdtd->h_blanking - mdtd->h_sync_offset)
		<< DPTX_VIDEO_MSA1_H_START_SHIFT;
	reg |= (mdtd->v_blanking - mdtd->v_sync_offset)
		<< DPTX_VIDEO_MSA1_V_START_SHIFT;
	dptx_writel(dptx, DPTX_VIDEO_MSA1_N(stream), reg);
#if 0
	/* Single, dual, or quad pixel */
	reg = dptx_readl(dptx, DPTX_VSAMPLE_CTRL_N(stream));
	reg &= ~DPTX_VSAMPLE_CTRL_MULTI_PIXEL_MASK;
	reg |= dptx->multipixel << DPTX_VSAMPLE_CTRL_MULTI_PIXEL_SHIFT;
	dptx_writel(dptx, DPTX_VSAMPLE_CTRL_N(stream), reg);
#endif
	/* Configure DPTX_VSAMPLE_POLARITY_CTRL register */
	reg = 0;
	if (mdtd->h_sync_polarity == 1)
		reg |= DPTX_POL_CTRL_H_SYNC_POL_EN;
	if (mdtd->v_sync_polarity == 1)
		reg |= DPTX_POL_CTRL_V_SYNC_POL_EN;
	dptx_writel(dptx, DPTX_VSAMPLE_POLARITY_CTRL_N(stream), reg);

	dptx_video_set_sink_bpc(dptx, stream);

	/* Configure video_config1 register */
	reg = 0;
#if 0
	if (vparams->video_format == VCEA) {
		if (vmode == 5 || vmode == 6 || vmode == 7 ||
			vmode == 10 || vmode == 11 || vmode == 20 ||
			vmode == 21 || vmode == 22 || vmode == 39 ||
			vmode == 25 || vmode == 26 || vmode == 40 ||
			vmode == 44 || vmode == 45 || vmode == 46 ||
			vmode == 50 || vmode == 51 || vmode == 54 ||
			vmode == 55 || vmode == 58 || vmode  == 59)
			reg |= DPTX_VIDEO_CONFIG1_IN_OSC_EN;
	}

	if (mdtd->interlaced == 1)
		reg |= DPTX_VIDEO_CONFIG1_O_IP_EN;
#endif
	reg |= mdtd->h_active << DPTX_VIDEO_H_ACTIVE_SHIFT;
	reg |= mdtd->h_blanking << DPTX_VIDEO_H_BLANK_SHIFT;
	dptx_writel(dptx, DPTX_VIDEO_CONFIG1_N(stream), reg);

	/* Configure video_config2 register */
	reg = 0;
	reg |= mdtd->v_active << DPTX_VIDEO_V_ACTIVE_SHIFT;
	reg |= mdtd->v_blanking << DPTX_VIDEO_V_BLANK_SHIFT;
	dptx_writel(dptx, DPTX_VIDEO_CONFIG2_N(stream), reg);

	/* Configure video_config3 register */
	reg = 0;
	reg |= mdtd->h_sync_offset << DPTX_VIDEO_H_FRONT_PORCH;
	reg |= mdtd->h_sync_pulse_width << DPTX_VIDEO_H_SYNC_WIDTH;
	dptx_writel(dptx, DPTX_VIDEO_CONFIG3_N(stream), reg);

	/* Configure video_config4 register */
	reg = 0;
	reg |= mdtd->v_sync_offset << DPTX_VIDEO_V_FRONT_PORCH;
	reg |= mdtd->v_sync_pulse_width << DPTX_VIDEO_V_SYNC_WIDTH;
	dptx_writel(dptx, DPTX_VIDEO_CONFIG4_N(stream), reg);

	/* Configure video_config5 register */
	dptx_video_ts_change(dptx, stream);

	reg = dptx_calculate_hblank_interval(dptx, stream);
	//reg |= (DPTX_VIDEO_HBLANK_INTERVAL_ENABLE << DPTX_VIDEO_HBLANK_INTERVAL_SHIFT);
	dptx_writel(dptx, DPTX_VIDEO_HBLANK_INTERVAL_N(stream), reg);
}

#if 0
static int dptx_get_link_clk(int rate)
{
	int link_clk;
	int link_rate;

	switch (rate) {
	case DPTX_PHYIF_CTRL_RATE_RBR:
		link_rate = 162;
		link_clk = 40500;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR:
		link_rate = 270;
		link_clk = 67500;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR2:
		link_rate = 540;
		link_clk = 135000;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR3:
		link_rate = 810;
		link_clk = 202500;
		break;
	default:
        	//dptx_dbg(dptx, "Invalid rate param = %d\n" , rate);
        	printk("Invalid rate param = %d\n" , rate);
        	return -EINVAL;
        break;
	}

	return link_clk;
}

void dwc_dptx_dtd_reset(struct dtd *mdtd)
{
	mdtd->pixel_repetition_input = 0;
	mdtd->pixel_clock  = 0;
	mdtd->h_active = 0;
	mdtd->h_blanking = 0;
	mdtd->h_sync_offset = 0;
	mdtd->h_sync_pulse_width = 0;
	mdtd->h_image_size = 0;
	mdtd->v_active = 0;
	mdtd->v_blanking = 0;
	mdtd->v_sync_offset = 0;
	mdtd->v_sync_pulse_width = 0;
	mdtd->v_image_size = 0;
	mdtd->interlaced = 0;
	mdtd->v_sync_polarity = 0;
	mdtd->h_sync_polarity = 0;
}

int dptx_dtd_fill(struct dtd *mdtd, u8 code, u32 refresh_rate,
		  u8 video_format)
{
	dwc_dptx_dtd_reset(mdtd);

	mdtd->h_image_size = 16;
	mdtd->v_image_size = 9;

	if (video_format == VCEA) {
		switch (code) {
		case 1: /* 640x480p @ 59.94/60Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 640;
			mdtd->v_active = 480;
			mdtd->h_blanking = 160;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 16;
			mdtd->v_sync_offset = 10;
			mdtd->h_sync_pulse_width = 96;
			mdtd->v_sync_pulse_width = 2;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 25175;
			break;
		case 69:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
		case 4: /* 1280x720p @ 59.94/60Hz 16:9 */
			mdtd->h_active = 1280;
			mdtd->v_active = 720;
			mdtd->h_blanking = 370;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 110;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 40;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 74250;
			break;
		case 76:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
		case 16: /* 1920x1080p @ 59.94/60Hz 16:9 */
			mdtd->h_active = 1920;
			mdtd->v_active = 1080;
			mdtd->h_blanking = 280;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 88;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 148500;
			break;
		case 102:
			mdtd->h_active = 4096;
			mdtd->v_active = 2160;
			mdtd->h_blanking = 304;
			mdtd->v_blanking = 90;
			mdtd->h_sync_offset = 88;
			mdtd->v_sync_offset = 8;
			mdtd->h_sync_pulse_width = 88;
			mdtd->v_sync_pulse_width = 10;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 594000;
			break;
		default:
			return false;
		}
	}

	return true;
}

void dptx_video_reset(struct dptx *dptx, int enable, int stream)
{
	u32 reg;

	reg = dptx_readl(dptx, DPTX_SRST_CTRL);

	if (enable)
		reg |= DPTX_SRST_VIDEO_RESET_N(stream);
	else
		reg &= ~DPTX_SRST_VIDEO_RESET_N(stream);

	dptx_writel(dptx, DPTX_SRST_CTRL, reg);
}

static u8 dptx_bit_field(const u16 data, u8 shift, u8 width)
{
	return ((data >> shift) & ((((u16)1) << width) - 1));
}

static u16 dptx_concat_bits(u8 bhi, u8 ohi, u8 nhi, u8 blo, u8 olo, u8 nlo)
{
	return (dptx_bit_field(bhi, ohi, nhi) << nlo) |
		dptx_bit_field(blo, olo, nlo);
}

static u16 dptx_byte_to_word(const u8 hi, const u8 lo)
{
	return dptx_concat_bits(hi, 0, 8, lo, 0, 8);
}

static int dptx_dtd_parse(struct dptx *dptx, struct dtd *mdtd, u8 *data)
{
	mdtd->pixel_repetition_input = 0;

	mdtd->pixel_clock = dptx_byte_to_word(data[1], data[0]);
	if (mdtd->pixel_clock < 0x01)
		return -EINVAL;

	mdtd->h_active = dptx_concat_bits(data[4], 4, 4, data[2], 0, 8);
	mdtd->h_blanking = dptx_concat_bits(data[4], 0, 4, data[3], 0, 8);
	mdtd->h_sync_offset = dptx_concat_bits(data[11], 6, 2, data[8], 0, 8);
	mdtd->h_sync_pulse_width = dptx_concat_bits(data[11], 4, 2, data[9],
							0, 8);
	mdtd->h_image_size = dptx_concat_bits(data[14], 4, 4, data[12], 0, 8);

	mdtd->v_active = dptx_concat_bits(data[7], 4, 4, data[5], 0, 8);
	mdtd->v_blanking = dptx_concat_bits(data[7], 0, 4, data[6], 0, 8);
	mdtd->v_sync_offset = dptx_concat_bits(data[11], 2, 2, data[10], 4, 4);
	mdtd->v_sync_pulse_width = dptx_concat_bits(data[11], 0, 2,
							data[10], 0, 4);
	mdtd->v_image_size = dptx_concat_bits(data[14], 0, 4, data[13], 0, 8);
	if (dptx_bit_field(data[17], 4, 1) != 1)
		return -EINVAL;
	if (dptx_bit_field(data[17], 3, 1) != 1)
		return -EINVAL;

	mdtd->interlaced = dptx_bit_field(data[17], 7, 1) == 1;
	mdtd->v_sync_polarity = dptx_bit_field(data[17], 2, 1) == 0;
	mdtd->h_sync_polarity = dptx_bit_field(data[17], 1, 1) == 0;
	if (mdtd->interlaced == 1)
		mdtd->v_active /= 2;
	mdtd->pixel_clock *= 10;
	return 0;
}

static void dptx_update_mode_from_dtd(struct dptx *dptx, int stream)
{
	struct drm_display_mode *dmode = &dptx->ppls[stream].mode;
	struct dtd *mdtd = &dptx->ppls[stream].vparams.mdtd;
	char name[DRM_DISPLAY_MODE_LEN];

	dmode->hdisplay = mdtd->h_active;
	dmode->vdisplay = mdtd->v_active;
	dmode->htotal = mdtd->h_blanking + dmode->hdisplay;
	dmode->vtotal = mdtd->v_blanking + dmode->vdisplay;
	dmode->hsync_start = mdtd->h_sync_offset + dmode->hdisplay;
	dmode->vsync_start = mdtd->v_sync_offset + dmode->vdisplay;
	dmode->hsync_end = dmode->hsync_start + mdtd->h_sync_pulse_width;
	dmode->vsync_end = dmode->vsync_start + mdtd->v_sync_pulse_width;
	dmode->flags = (mdtd->h_sync_polarity << DRM_MODE_FLAG_PHSYNC) &
			(mdtd->v_sync_polarity << DRM_MODE_FLAG_PVSYNC);
	dmode->clock = mdtd->pixel_clock;

	memset(name, 0, DRM_DISPLAY_MODE_LEN);
	sprintf(name, "DP_EDID_%dP", dmode->vdisplay);
	memcpy(dmode->name, name, DRM_DISPLAY_MODE_LEN);
}

static int dptx_check_edid(struct dptx *dptx)
{
	int i;
	u32 edid_sum = 0;
	u8 *edid = (u8 *)dptx->edid;

	for (i = 0; i < 128; i++)
		edid_sum += edid[i];

	if (edid_sum & 0xFF)
		return -EINVAL;

	return 0;
}

int dptx_update_mode_from_edid(struct dptx *dptx)
{
	int retval = 0;

	retval = dptx_check_edid(dptx);
	if (retval) {
		dptx_err(dptx, "edid check error. use mode config");
		return -1;
	} else {
		struct dtd mdtd;
		u8 *edid = (u8 *)dptx->edid;

		retval = dptx_dtd_parse(dptx, &mdtd, &edid[0x36]);
		if (retval) {
			dptx_err(dptx, "dtd parse error. use mode config");
			return -1 ;
		}
		dptx->ppls[0].vparams.mdtd = mdtd;
		dptx_update_mode_from_dtd(dptx, 0);
	}

	return 0;
}
#endif

void dptx_disable_default_video_stream(struct dptx *dptx, int stream)
{
	u32 vsamplectrl;

	vsamplectrl = dptx_readl(dptx, DPTX_VSAMPLE_CTRL_N(stream));
	vsamplectrl &= ~DPTX_VSAMPLE_CTRL_STREAM_EN;
	dptx_writel(dptx, DPTX_VSAMPLE_CTRL_N(stream), vsamplectrl);
}

void dptx_enable_default_video_stream(struct dptx *dptx, int stream)
{
	u32 vsamplectrl;

	vsamplectrl = dptx_readl(dptx, DPTX_VSAMPLE_CTRL_N(stream));
	vsamplectrl |= DPTX_VSAMPLE_CTRL_STREAM_EN;
	dptx_writel(dptx, DPTX_VSAMPLE_CTRL_N(stream), vsamplectrl);
}

void dptx_update_video_config(struct dptx *dptx, int streams)
{
	int i, pipe;

	for (i = 0; i < dptx->streams; i++) {
		pipe = dptx->stream_id[i];
		if ((1<<pipe) & streams) {
			dptx_video_ts_calculate(dptx,
				dptx->link.lanes,
				dptx->link.rate,
				dptx->ppls[pipe].vparams.bpc,
				dptx->ppls[pipe].vparams.pix_enc,
				dptx->ppls[pipe].mode.clock,
				pipe);
			dptx_video_core_config(dptx, pipe);
		}
	}
}

void dptx_dtd_update(struct dptx *dptx, int stream)
{
	struct dtd *mdtd = &dptx->ppls[stream].vparams.mdtd;
#ifdef UNUSE_DRM
	struct display_timing *dt = &dptx->ppls[stream].dt;
	mdtd->h_image_size = 16;
	mdtd->v_image_size = 9;
	mdtd->h_active = dt->hactive.typ;
	mdtd->v_active = dt->vactive.typ;
	mdtd->h_blanking = dt->hfront_porch.typ + dt->hsync_len.typ +
		dt->hback_porch.typ;
	mdtd->v_blanking = dt->vfront_porch.typ + dt->vsync_len.typ +
		dt->vback_porch.typ;
	mdtd->h_sync_offset = dt->hfront_porch.typ;
	mdtd->v_sync_offset = dt->vfront_porch.typ;
	mdtd->h_sync_pulse_width = dt->hsync_len.typ;
	mdtd->v_sync_pulse_width = dt->vsync_len.typ;
	mdtd->h_sync_polarity = !!(dt->flags & DISPLAY_FLAGS_HSYNC_HIGH);
	mdtd->v_sync_polarity = !!(dt->flags & DISPLAY_FLAGS_VSYNC_HIGH);
	mdtd->interlaced = 0; /* (progressive_nI) */
	mdtd->pixel_clock = dt->pixelclock.typ / 1000;
#else
	struct drm_display_mode *dmode = &dptx->ppls[stream].mode;
	mdtd->h_image_size = 16;
	mdtd->v_image_size = 9;
	mdtd->h_active = dmode->hdisplay;
	mdtd->v_active = dmode->vdisplay;
	mdtd->h_blanking = dmode->htotal - dmode->hdisplay;
	mdtd->v_blanking = dmode->vtotal - dmode->vdisplay;
	mdtd->h_sync_offset = dmode->hsync_start - dmode->hdisplay;
	mdtd->v_sync_offset = dmode->vsync_start - dmode->vdisplay;
	mdtd->h_sync_pulse_width = dmode->hsync_end - dmode->hsync_start;
	mdtd->v_sync_pulse_width = dmode->vsync_end - dmode->vsync_start;
	mdtd->h_sync_polarity = !!(dmode->flags & DRM_MODE_FLAG_PHSYNC);
	mdtd->v_sync_polarity = !!(dmode->flags & DRM_MODE_FLAG_PVSYNC);
	mdtd->interlaced = 0; /* (progressive_nI) */
	mdtd->pixel_clock = dmode->clock;
#endif
}

void dptx_audio_mute(struct dptx *dptx)
{
	int i, pipe;
	u32 reg = 0;

	for (i = 0; i < dptx->streams; i++) {
		pipe = dptx->stream_id[i];
		reg = dptx_readl(dptx, DPTX_AUD_CONFIG1_N(i));
		reg |= DPTX_AUDIO_MUTE;
		dptx_writel(dptx, DPTX_AUD_CONFIG1_N(i), reg);
	}
}

