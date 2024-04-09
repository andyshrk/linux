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

#ifndef __DPTX_AVGEN_H__
#define __DPTX_AVGEN_H__

#include <asm/io.h>

struct dptx;

enum pixel_enc_type {
	RGB = 0,
	YCBCR420 = 1,
	YCBCR422 = 2,
	YCBCR444 = 3,
	YONLY = 4,
	RAW = 5
};

enum color_depth {
	COLOR_DEPTH_INVALID = 0,
	COLOR_DEPTH_6 = 6,
	COLOR_DEPTH_8 = 8,
	COLOR_DEPTH_10 = 10,
	COLOR_DEPTH_12 = 12,
	COLOR_DEPTH_16 = 16
};

enum pattern_mode {
	TILE = 0,
	RAMP = 1,
	CHESS = 2,
	COLRAMP = 3
};

enum dynamic_range_type {
	CEA = 1,
	VESA = 2
};

enum colorimetry_type {
	ITU601 = 1,
	ITU709 = 2
};

enum video_format_type {
	VCEA = 0,
	CVT = 1,
	DMT = 2
};

struct dtd {
	u16 pixel_repetition_input;
	int pixel_clock;
	/** 1 for interlaced, 0 progressive */
	u8 interlaced;
	u16 h_active;
	u16 h_blanking;
	u16 h_image_size;
	u16 h_sync_offset;
	u16 h_sync_pulse_width;
	u8 h_sync_polarity;
	u16 v_active;
	u16 v_blanking;
	u16 v_image_size;
	u16 v_sync_offset;
	u16 v_sync_pulse_width;
	u8 v_sync_polarity;
};

struct video_params {
	u8 pix_enc;
	u8 pattern_mode;
	struct dtd mdtd;
	u8 mode;
	u8 bpc;
	u8 colorimetry;
	u8 dynamic_range;
	u8 aver_bytes_per_tu;
	u8 aver_bytes_per_tu_frac;
	u8 init_threshold;
	u32 refresh_rate;
	u8 video_format;
	// DSC staff here TODO: move to special DSC struct
	u16 slice_width;
	u16 chunk_size;
	u16 slice_height;
	int encoders;
	u8 first_line_bpg_offset;
	u32 minRateBufferSize;
	u16 dsc_bpp;
	u16 dsc_bpc;
	u32 hrddelay;
	u32 initial_dec_delay;
	u16 initial_scale_value;
	u32 scale_decrement_interval;
};

void dptx_video_params_reset(struct dptx *dptx, int stream);
void dptx_video_set_sink_col(struct dptx *dptx, int stream);
void dptx_video_set_sink_bpc(struct dptx *dptx, int stream);
int dptx_video_ts_calculate(struct dptx *dptx, int lane_num, int rate,
			    int bpc, int encoding, int pixel_clock, int stream);
u8 dptx_calculate_dsc_init_threshold(struct dptx* dptx,  int stream);
void dptx_video_ts_change(struct dptx *dptx, int stream);
void dptx_video_set_core_bpc(struct dptx *dptx, int stream);
int dptx_calculate_hblank_interval(struct dptx* dptx, int stream);
void dptx_video_core_config(struct dptx *dptx, int stream);
void dptx_disable_default_video_stream(struct dptx *dptx, int stream);
void dptx_enable_default_video_stream(struct dptx *dptx, int stream);
void dptx_update_video_config(struct dptx *dptx, int streams);
void dptx_dtd_update(struct dptx *dptx, int stream);
void dptx_audio_mute(struct dptx *dptx);
#if 0
void dwc_dptx_dtd_reset(struct dtd *mdtd);
int dptx_dtd_fill(struct dtd *mdtd, u8 code, u32 refresh_rate,
		  u8 video_format);
int dptx_update_mode_from_edid(struct dptx *dptx);
#endif
#endif
