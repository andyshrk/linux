// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) Rockchip Electronics Co.Ltd
 * Author: Andy Yan <andy.yan@rock-chips.com>
 */

#include <linux/kernel.h>
#include <linux/component.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <drm/drm_blend.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_plane.h>
#include <drm/drm_print.h>

#include "rockchip_drm_vop2.h"

union vop2_alpha_ctrl {
	u32 val;
	struct {
		/* [0:1] */
		u32 color_mode:1;
		u32 alpha_mode:1;
		/* [2:3] */
		u32 blend_mode:2;
		u32 alpha_cal_mode:1;
		/* [5:7] */
		u32 factor_mode:3;
		/* [8:9] */
		u32 alpha_en:1;
		u32 src_dst_swap:1;
		u32 reserved:6;
		/* [16:23] */
		u32 glb_alpha:8;
	} bits;
};

struct vop2_alpha {
	union vop2_alpha_ctrl src_color_ctrl;
	union vop2_alpha_ctrl dst_color_ctrl;
	union vop2_alpha_ctrl src_alpha_ctrl;
	union vop2_alpha_ctrl dst_alpha_ctrl;
};

struct vop2_alpha_config {
	bool src_premulti_en;
	bool dst_premulti_en;
	bool src_pixel_alpha_en;
	bool dst_pixel_alpha_en;
	u16 src_glb_alpha_value;
	u16 dst_glb_alpha_value;
};

static const uint32_t formats_cluster[] = {
	DRM_FORMAT_XRGB2101010,
	DRM_FORMAT_XBGR2101010,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_BGR888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_YUV420_8BIT, /* yuv420_8bit non-Linear mode only */
	DRM_FORMAT_YUV420_10BIT, /* yuv420_10bit non-Linear mode only */
	DRM_FORMAT_YUYV, /* yuv422_8bit non-Linear mode only*/
	DRM_FORMAT_Y210, /* yuv422_10bit non-Linear mode only */
};

static const uint32_t formats_esmart[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_BGR888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_NV12, /* yuv420_8bit linear mode, 2 plane */
	DRM_FORMAT_NV21, /* yvu420_8bit linear mode, 2 plane */
	DRM_FORMAT_NV16, /* yuv422_8bit linear mode, 2 plane */
	DRM_FORMAT_NV61, /* yvu422_8bit linear mode, 2 plane */
	DRM_FORMAT_NV20, /* yuv422_10bit linear mode, 2 plane, no padding */
	DRM_FORMAT_NV24, /* yuv444_8bit linear mode, 2 plane */
	DRM_FORMAT_NV42, /* yvu444_8bit linear mode, 2 plane */
	DRM_FORMAT_NV30, /* yuv444_10bit linear mode, 2 plane, no padding */
	DRM_FORMAT_NV15, /* yuv420_10bit linear mode, 2 plane, no padding */
	DRM_FORMAT_YVYU, /* yuv422_8bit[YVYU] linear mode */
	DRM_FORMAT_VYUY, /* yuv422_8bit[VYUY] linear mode */
	DRM_FORMAT_YUYV, /* yuv422_8bit[YUYV] linear mode */
	DRM_FORMAT_UYVY, /* yuv422_8bit[UYVY] linear mode */
};

static const uint32_t formats_rk356x_esmart[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_BGR888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_NV12, /* yuv420_8bit linear mode, 2 plane */
	DRM_FORMAT_NV21, /* yuv420_8bit linear mode, 2 plane */
	DRM_FORMAT_NV15, /* yuv420_10bit linear mode, 2 plane, no padding */
	DRM_FORMAT_NV16, /* yuv422_8bit linear mode, 2 plane */
	DRM_FORMAT_NV61, /* yuv422_8bit linear mode, 2 plane */
	DRM_FORMAT_NV20, /* yuv422_10bit linear mode, 2 plane, no padding */
	DRM_FORMAT_NV24, /* yuv444_8bit linear mode, 2 plane */
	DRM_FORMAT_NV42, /* yuv444_8bit linear mode, 2 plane */
	DRM_FORMAT_NV30, /* yuv444_10bit linear mode, 2 plane, no padding */
	DRM_FORMAT_YVYU, /* yuv422_8bit[YVYU] linear mode */
	DRM_FORMAT_VYUY, /* yuv422_8bit[VYUY] linear mode */
};

static const uint32_t formats_smart[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_BGR888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGR565,
};

static const uint64_t format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

static const uint64_t format_modifiers_afbc[] = {
	DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16),

	DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
				AFBC_FORMAT_MOD_SPARSE),

	DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
				AFBC_FORMAT_MOD_YTR),

	DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
				AFBC_FORMAT_MOD_CBR),

	DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
				AFBC_FORMAT_MOD_YTR |
				AFBC_FORMAT_MOD_SPARSE),

	DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
				AFBC_FORMAT_MOD_CBR |
				AFBC_FORMAT_MOD_SPARSE),

	DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
				AFBC_FORMAT_MOD_YTR |
				AFBC_FORMAT_MOD_CBR),

	DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
				AFBC_FORMAT_MOD_YTR |
				AFBC_FORMAT_MOD_CBR |
				AFBC_FORMAT_MOD_SPARSE),

	/* SPLIT mandates SPARSE, RGB modes mandates YTR */
	DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
				AFBC_FORMAT_MOD_YTR |
				AFBC_FORMAT_MOD_SPARSE |
				AFBC_FORMAT_MOD_SPLIT),
	DRM_FORMAT_MOD_INVALID,
};

static const struct vop2_video_port_data rk3568_vop_video_ports[] = {
	{
		.id = 0,
		.feature = VOP2_VP_FEATURE_OUTPUT_10BIT,
		.gamma_lut_len = 1024,
		.cubic_lut_len = 9 * 9 * 9,
		.max_output = { 4096, 2304 },
		.pre_scan_max_dly = { 69, 53, 53, 42 },
		.offset = 0xc00,
	}, {
		.id = 1,
		.gamma_lut_len = 1024,
		.max_output = { 2048, 1536 },
		.pre_scan_max_dly = { 40, 40, 40, 40 },
		.offset = 0xd00,
	}, {
		.id = 2,
		.gamma_lut_len = 1024,
		.max_output = { 1920, 1080 },
		.pre_scan_max_dly = { 40, 40, 40, 40 },
		.offset = 0xe00,
	},
};

/*
 * rk3568 vop with 2 cluster, 2 esmart win, 2 smart win.
 * Every cluster can work as 4K win or split into two win.
 * All win in cluster support AFBCD.
 *
 * Every esmart win and smart win support 4 Multi-region.
 *
 * Scale filter mode:
 *
 * * Cluster:  bicubic for horizontal scale up, others use bilinear
 * * ESmart:
 *    * nearest-neighbor/bilinear/bicubic for scale up
 *    * nearest-neighbor/bilinear/average for scale down
 *
 *
 * @TODO describe the wind like cpu-map dt nodes;
 */
static const struct vop2_win_data rk3568_vop_win_data[] = {
	{
		.name = "Smart0-win0",
		.phys_id = ROCKCHIP_VOP2_SMART0,
		.base = 0x1c00,
		.formats = formats_smart,
		.nformats = ARRAY_SIZE(formats_smart),
		.format_modifiers = format_modifiers,
		.layer_sel_id = { 3, 3, 3, 0xffff },
		.supported_rotations = DRM_MODE_REFLECT_Y,
		.type = DRM_PLANE_TYPE_PRIMARY,
		.max_upscale_factor = 8,
		.max_downscale_factor = 8,
		.dly = { 20, 47, 41 },
	}, {
		.name = "Smart1-win0",
		.phys_id = ROCKCHIP_VOP2_SMART1,
		.formats = formats_smart,
		.nformats = ARRAY_SIZE(formats_smart),
		.format_modifiers = format_modifiers,
		.base = 0x1e00,
		.layer_sel_id = { 7, 7, 7, 0xffff },
		.supported_rotations = DRM_MODE_REFLECT_Y,
		.type = DRM_PLANE_TYPE_PRIMARY,
		.max_upscale_factor = 8,
		.max_downscale_factor = 8,
		.dly = { 20, 47, 41 },
	}, {
		.name = "Esmart1-win0",
		.phys_id = ROCKCHIP_VOP2_ESMART1,
		.formats = formats_rk356x_esmart,
		.nformats = ARRAY_SIZE(formats_rk356x_esmart),
		.format_modifiers = format_modifiers,
		.base = 0x1a00,
		.layer_sel_id = { 6, 6, 6, 0xffff },
		.supported_rotations = DRM_MODE_REFLECT_Y,
		.type = DRM_PLANE_TYPE_PRIMARY,
		.max_upscale_factor = 8,
		.max_downscale_factor = 8,
		.dly = { 20, 47, 41 },
	}, {
		.name = "Esmart0-win0",
		.phys_id = ROCKCHIP_VOP2_ESMART0,
		.formats = formats_rk356x_esmart,
		.nformats = ARRAY_SIZE(formats_rk356x_esmart),
		.format_modifiers = format_modifiers,
		.base = 0x1800,
		.layer_sel_id = { 2, 2, 2, 0xffff },
		.supported_rotations = DRM_MODE_REFLECT_Y,
		.type = DRM_PLANE_TYPE_PRIMARY,
		.max_upscale_factor = 8,
		.max_downscale_factor = 8,
		.dly = { 20, 47, 41 },
	}, {
		.name = "Cluster0-win0",
		.phys_id = ROCKCHIP_VOP2_CLUSTER0,
		.base = 0x1000,
		.formats = formats_cluster,
		.nformats = ARRAY_SIZE(formats_cluster),
		.format_modifiers = format_modifiers_afbc,
		.layer_sel_id = { 0, 0, 0, 0xffff },
		.supported_rotations = DRM_MODE_ROTATE_90 | DRM_MODE_ROTATE_270 |
					DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y,
		.max_upscale_factor = 4,
		.max_downscale_factor = 4,
		.dly = { 0, 27, 21 },
		.type = DRM_PLANE_TYPE_OVERLAY,
		.feature = WIN_FEATURE_AFBDC | WIN_FEATURE_CLUSTER,
	}, {
		.name = "Cluster1-win0",
		.phys_id = ROCKCHIP_VOP2_CLUSTER1,
		.base = 0x1200,
		.formats = formats_cluster,
		.nformats = ARRAY_SIZE(formats_cluster),
		.format_modifiers = format_modifiers_afbc,
		.layer_sel_id = { 1, 1, 1, 0xffff },
		.supported_rotations = DRM_MODE_ROTATE_90 | DRM_MODE_ROTATE_270 |
					DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y,
		.type = DRM_PLANE_TYPE_OVERLAY,
		.max_upscale_factor = 4,
		.max_downscale_factor = 4,
		.dly = { 0, 27, 21 },
		.feature = WIN_FEATURE_AFBDC | WIN_FEATURE_CLUSTER,
	},
};

static const struct vop2_regs_dump rk3568_regs_dump[] = {
	{
		.name = "SYS",
		.base = RK3568_REG_CFG_DONE,
		.size = 0x100,
		.en_reg  = 0,
		.en_val = 0,
		.en_mask = 0
	}, {
		.name = "OVL",
		.base = RK3568_OVL_CTRL,
		.size = 0x100,
		.en_reg = 0,
		.en_val = 0,
		.en_mask = 0,
	}, {
		.name = "VP0",
		.base = RK3568_VP0_CTRL_BASE,
		.size = 0x100,
		.en_reg = RK3568_VP_DSP_CTRL,
		.en_val = 0,
		.en_mask = RK3568_VP_DSP_CTRL__STANDBY,
	}, {
		.name = "VP1",
		.base = RK3568_VP1_CTRL_BASE,
		.size = 0x100,
		.en_reg = RK3568_VP_DSP_CTRL,
		.en_val = 0,
		.en_mask = RK3568_VP_DSP_CTRL__STANDBY,
	}, {
		.name = "VP2",
		.base = RK3568_VP2_CTRL_BASE,
		.size = 0x100,
		.en_reg = RK3568_VP_DSP_CTRL,
		.en_val = 0,
		.en_mask = RK3568_VP_DSP_CTRL__STANDBY,

	}, {
		.name = "Cluster0",
		.base = RK3568_CLUSTER0_CTRL_BASE,
		.size = 0x110,
		.en_reg = RK3568_CLUSTER_WIN_CTRL0,
		.en_val = RK3568_CLUSTER_WIN_CTRL0__WIN0_EN,
		.en_mask = RK3568_CLUSTER_WIN_CTRL0__WIN0_EN,
	}, {
		.name = "Cluster1",
		.base = RK3568_CLUSTER1_CTRL_BASE,
		.size = 0x110,
		.en_reg = RK3568_CLUSTER_WIN_CTRL0,
		.en_val = RK3568_CLUSTER_WIN_CTRL0__WIN0_EN,
		.en_mask = RK3568_CLUSTER_WIN_CTRL0__WIN0_EN,
	}, {
		.name = "Esmart0",
		.base = RK3568_ESMART0_CTRL_BASE,
		.size = 0xf0,
		.en_reg = RK3568_SMART_REGION0_CTRL,
		.en_val = RK3568_SMART_REGION0_CTRL__WIN0_EN,
		.en_mask = RK3568_SMART_REGION0_CTRL__WIN0_EN,
	}, {
		.name = "Esmart1",
		.base = RK3568_ESMART1_CTRL_BASE,
		.size = 0xf0,
		.en_reg = RK3568_SMART_REGION0_CTRL,
		.en_val = RK3568_SMART_REGION0_CTRL__WIN0_EN,
		.en_mask = RK3568_SMART_REGION0_CTRL__WIN0_EN,
	}, {
		.name = "Smart0",
		.base = RK3568_SMART0_CTRL_BASE,
		.size = 0xf0,
		.en_reg = RK3568_SMART_REGION0_CTRL,
		.en_val = RK3568_SMART_REGION0_CTRL__WIN0_EN,
		.en_mask = RK3568_SMART_REGION0_CTRL__WIN0_EN,
	}, {
		.name = "Smart1",
		.base = RK3568_SMART1_CTRL_BASE,
		.size = 0xf0,
		.en_reg = RK3568_SMART_REGION0_CTRL,
		.en_val = RK3568_SMART_REGION0_CTRL__WIN0_EN,
		.en_mask = RK3568_SMART_REGION0_CTRL__WIN0_EN,
	},
};

static const struct vop2_video_port_data rk3588_vop_video_ports[] = {
	{
		.id = 0,
		.feature = VOP2_VP_FEATURE_OUTPUT_10BIT,
		.gamma_lut_len = 1024,
		.cubic_lut_len = 9 * 9 * 9, /* 9x9x9 */
		.max_output = { 4096, 2304 },
		/* hdr2sdr sdr2hdr hdr2hdr sdr2sdr */
		.pre_scan_max_dly = { 76, 65, 65, 54 },
		.offset = 0xc00,
	}, {
		.id = 1,
		.feature = VOP2_VP_FEATURE_OUTPUT_10BIT,
		.gamma_lut_len = 1024,
		.cubic_lut_len = 729, /* 9x9x9 */
		.max_output = { 4096, 2304 },
		.pre_scan_max_dly = { 76, 65, 65, 54 },
		.offset = 0xd00,
	}, {
		.id = 2,
		.feature = VOP2_VP_FEATURE_OUTPUT_10BIT,
		.gamma_lut_len = 1024,
		.cubic_lut_len = 17 * 17 * 17, /* 17x17x17 */
		.max_output = { 4096, 2304 },
		.pre_scan_max_dly = { 52, 52, 52, 52 },
		.offset = 0xe00,
	}, {
		.id = 3,
		.gamma_lut_len = 1024,
		.max_output = { 2048, 1536 },
		.pre_scan_max_dly = { 52, 52, 52, 52 },
		.offset = 0xf00,
	},
};

/*
 * rk3588 vop with 4 cluster, 4 esmart win.
 * Every cluster can work as 4K win or split into two win.
 * All win in cluster support AFBCD.
 *
 * Every esmart win and smart win support 4 Multi-region.
 *
 * Scale filter mode:
 *
 * * Cluster:  bicubic for horizontal scale up, others use bilinear
 * * ESmart:
 *    * nearest-neighbor/bilinear/bicubic for scale up
 *    * nearest-neighbor/bilinear/average for scale down
 *
 * AXI Read ID assignment:
 * Two AXI bus:
 * AXI0 is a read/write bus with a higher performance.
 * AXI1 is a read only bus.
 *
 * Every window on a AXI bus must assigned two unique
 * read id(yrgb_id/uv_id, valid id are 0x1~0xe).
 *
 * AXI0:
 * Cluster0/1, Esmart0/1, WriteBack
 *
 * AXI 1:
 * Cluster2/3, Esmart2/3
 *
 */
static const struct vop2_win_data rk3588_vop_win_data[] = {
	{
		.name = "Cluster0-win0",
		.phys_id = ROCKCHIP_VOP2_CLUSTER0,
		.base = 0x1000,
		.formats = formats_cluster,
		.nformats = ARRAY_SIZE(formats_cluster),
		.format_modifiers = format_modifiers_afbc,
		.layer_sel_id = { 0, 0, 0, 0 },
		.supported_rotations = DRM_MODE_ROTATE_90 | DRM_MODE_ROTATE_270 |
				       DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y,
		.max_upscale_factor = 4,
		.max_downscale_factor = 4,
		.dly = { 4, 26, 29 },
		.type = DRM_PLANE_TYPE_PRIMARY,
		.feature = WIN_FEATURE_AFBDC | WIN_FEATURE_CLUSTER,
	}, {
		.name = "Cluster1-win0",
		.phys_id = ROCKCHIP_VOP2_CLUSTER1,
		.base = 0x1200,
		.formats = formats_cluster,
		.nformats = ARRAY_SIZE(formats_cluster),
		.format_modifiers = format_modifiers_afbc,
		.layer_sel_id = { 1, 1, 1, 1 },
		.supported_rotations = DRM_MODE_ROTATE_90 | DRM_MODE_ROTATE_270 |
				       DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y,
		.type = DRM_PLANE_TYPE_PRIMARY,
		.max_upscale_factor = 4,
		.max_downscale_factor = 4,
		.dly = { 4, 26, 29 },
		.feature = WIN_FEATURE_AFBDC | WIN_FEATURE_CLUSTER,
	}, {
		.name = "Cluster2-win0",
		.phys_id = ROCKCHIP_VOP2_CLUSTER2,
		.base = 0x1400,
		.formats = formats_cluster,
		.nformats = ARRAY_SIZE(formats_cluster),
		.format_modifiers = format_modifiers_afbc,
		.layer_sel_id = { 4, 4, 4, 4 },
		.supported_rotations = DRM_MODE_ROTATE_90 | DRM_MODE_ROTATE_270 |
				       DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y,
		.type = DRM_PLANE_TYPE_PRIMARY,
		.max_upscale_factor = 4,
		.max_downscale_factor = 4,
		.dly = { 4, 26, 29 },
		.feature = WIN_FEATURE_AFBDC | WIN_FEATURE_CLUSTER,
	}, {
		.name = "Cluster3-win0",
		.phys_id = ROCKCHIP_VOP2_CLUSTER3,
		.base = 0x1600,
		.formats = formats_cluster,
		.nformats = ARRAY_SIZE(formats_cluster),
		.format_modifiers = format_modifiers_afbc,
		.layer_sel_id =  { 5, 5, 5, 5 },
		.supported_rotations = DRM_MODE_ROTATE_90 | DRM_MODE_ROTATE_270 |
				       DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y,
		.type = DRM_PLANE_TYPE_PRIMARY,
		.max_upscale_factor = 4,
		.max_downscale_factor = 4,
		.dly = { 4, 26, 29 },
		.feature = WIN_FEATURE_AFBDC | WIN_FEATURE_CLUSTER,
	}, {
		.name = "Esmart0-win0",
		.phys_id = ROCKCHIP_VOP2_ESMART0,
		.formats = formats_esmart,
		.nformats = ARRAY_SIZE(formats_esmart),
		.format_modifiers = format_modifiers,
		.base = 0x1800,
		.layer_sel_id = { 2, 2, 2, 2 },
		.supported_rotations = DRM_MODE_REFLECT_Y,
		.type = DRM_PLANE_TYPE_OVERLAY,
		.max_upscale_factor = 8,
		.max_downscale_factor = 8,
		.dly = { 23, 45, 48 },
	}, {
		.name = "Esmart1-win0",
		.phys_id = ROCKCHIP_VOP2_ESMART1,
		.formats = formats_esmart,
		.nformats = ARRAY_SIZE(formats_esmart),
		.format_modifiers = format_modifiers,
		.base = 0x1a00,
		.layer_sel_id = { 3, 3, 3, 3 },
		.supported_rotations = DRM_MODE_REFLECT_Y,
		.type = DRM_PLANE_TYPE_OVERLAY,
		.max_upscale_factor = 8,
		.max_downscale_factor = 8,
		.dly = { 23, 45, 48 },
	}, {
		.name = "Esmart2-win0",
		.phys_id = ROCKCHIP_VOP2_ESMART2,
		.base = 0x1c00,
		.formats = formats_esmart,
		.nformats = ARRAY_SIZE(formats_esmart),
		.format_modifiers = format_modifiers,
		.layer_sel_id =  { 6, 6, 6, 6 },
		.supported_rotations = DRM_MODE_REFLECT_Y,
		.type = DRM_PLANE_TYPE_OVERLAY,
		.max_upscale_factor = 8,
		.max_downscale_factor = 8,
		.dly = { 23, 45, 48 },
	}, {
		.name = "Esmart3-win0",
		.phys_id = ROCKCHIP_VOP2_ESMART3,
		.formats = formats_esmart,
		.nformats = ARRAY_SIZE(formats_esmart),
		.format_modifiers = format_modifiers,
		.base = 0x1e00,
		.layer_sel_id =  { 7, 7, 7, 7 },
		.supported_rotations = DRM_MODE_REFLECT_Y,
		.type = DRM_PLANE_TYPE_OVERLAY,
		.max_upscale_factor = 8,
		.max_downscale_factor = 8,
		.dly = { 23, 45, 48 },
	},
};

static const struct vop2_regs_dump rk3588_regs_dump[] = {
	{
		.name = "SYS",
		.base = RK3568_REG_CFG_DONE,
		.size = 0x100,
		.en_reg  = 0,
		.en_val = 0,
		.en_mask = 0
	}, {
		.name = "OVL",
		.base = RK3568_OVL_CTRL,
		.size = 0x100,
		.en_reg = 0,
		.en_val = 0,
		.en_mask = 0,
	}, {
		.name = "VP0",
		.base = RK3568_VP0_CTRL_BASE,
		.size = 0x100,
		.en_reg = RK3568_VP_DSP_CTRL,
		.en_val = 0,
		.en_mask = RK3568_VP_DSP_CTRL__STANDBY,
	}, {
		.name = "VP1",
		.base = RK3568_VP1_CTRL_BASE,
		.size = 0x100,
		.en_reg = RK3568_VP_DSP_CTRL,
		.en_val = 0,
		.en_mask = RK3568_VP_DSP_CTRL__STANDBY,
	}, {
		.name = "VP2",
		.base = RK3568_VP2_CTRL_BASE,
		.size = 0x100,
		.en_reg = RK3568_VP_DSP_CTRL,
		.en_val = 0,
		.en_mask = RK3568_VP_DSP_CTRL__STANDBY,

	}, {
		.name = "VP3",
		.base = RK3588_VP3_CTRL_BASE,
		.size = 0x100,
		.en_reg = RK3568_VP_DSP_CTRL,
		.en_val = 0,
		.en_mask = RK3568_VP_DSP_CTRL__STANDBY,
	}, {
		.name = "Cluster0",
		.base = RK3568_CLUSTER0_CTRL_BASE,
		.size = 0x110,
		.en_reg = RK3568_CLUSTER_WIN_CTRL0,
		.en_val = RK3568_CLUSTER_WIN_CTRL0__WIN0_EN,
		.en_mask = RK3568_CLUSTER_WIN_CTRL0__WIN0_EN,
	}, {
		.name = "Cluster1",
		.base = RK3568_CLUSTER1_CTRL_BASE,
		.size = 0x110,
		.en_reg = RK3568_CLUSTER_WIN_CTRL0,
		.en_val = RK3568_CLUSTER_WIN_CTRL0__WIN0_EN,
		.en_mask = RK3568_CLUSTER_WIN_CTRL0__WIN0_EN,
	}, {
		.name = "Cluster2",
		.base = RK3588_CLUSTER2_CTRL_BASE,
		.size = 0x110,
		.en_reg = RK3568_CLUSTER_WIN_CTRL0,
		.en_val = RK3568_CLUSTER_WIN_CTRL0__WIN0_EN,
		.en_mask = RK3568_CLUSTER_WIN_CTRL0__WIN0_EN,
	}, {
		.name = "Cluster3",
		.base = RK3588_CLUSTER3_CTRL_BASE,
		.size = 0x110,
		.en_reg = RK3568_CLUSTER_WIN_CTRL0,
		.en_val = RK3568_CLUSTER_WIN_CTRL0__WIN0_EN,
		.en_mask = RK3568_CLUSTER_WIN_CTRL0__WIN0_EN,
	}, {
		.name = "Esmart0",
		.base = RK3568_ESMART0_CTRL_BASE,
		.size = 0xf0,
		.en_reg = RK3568_SMART_REGION0_CTRL,
		.en_val = RK3568_SMART_REGION0_CTRL__WIN0_EN,
		.en_mask = RK3568_SMART_REGION0_CTRL__WIN0_EN,
	}, {
		.name = "Esmart1",
		.base = RK3568_ESMART1_CTRL_BASE,
		.size = 0xf0,
		.en_reg = RK3568_SMART_REGION0_CTRL,
		.en_val = RK3568_SMART_REGION0_CTRL__WIN0_EN,
		.en_mask = RK3568_SMART_REGION0_CTRL__WIN0_EN,
	}, {
		.name = "Esmart2",
		.base = RK3588_ESMART2_CTRL_BASE,
		.size = 0xf0,
		.en_reg = RK3568_SMART_REGION0_CTRL,
		.en_val = RK3568_SMART_REGION0_CTRL__WIN0_EN,
		.en_mask = RK3568_SMART_REGION0_CTRL__WIN0_EN,
	}, {
		.name = "Esmart3",
		.base = RK3588_ESMART3_CTRL_BASE,
		.size = 0xf0,
		.en_reg = RK3568_SMART_REGION0_CTRL,
		.en_val = RK3568_SMART_REGION0_CTRL__WIN0_EN,
		.en_mask = RK3568_SMART_REGION0_CTRL__WIN0_EN,
	},
};

static unsigned long rk3568_set_intf_mux(struct vop2_video_port *vp, int id, u32 polflags)
{
	struct vop2 *vop2 = vp->vop2;
	struct drm_crtc *crtc = &vp->crtc;
	u32 die, dip;

	die = vop2_readl(vop2, RK3568_DSP_IF_EN);
	dip = vop2_readl(vop2, RK3568_DSP_IF_POL);

	switch (id) {
	case ROCKCHIP_VOP2_EP_RGB0:
		die &= ~RK3568_SYS_DSP_INFACE_EN_RGB_MUX;
		die |= RK3568_SYS_DSP_INFACE_EN_RGB |
			   FIELD_PREP(RK3568_SYS_DSP_INFACE_EN_RGB_MUX, vp->id);
		dip &= ~RK3568_DSP_IF_POL__RGB_LVDS_PIN_POL;
		dip |= FIELD_PREP(RK3568_DSP_IF_POL__RGB_LVDS_PIN_POL, polflags);
		if (polflags & POLFLAG_DCLK_INV)
			regmap_write(vop2->sys_grf, RK3568_GRF_VO_CON1, BIT(3 + 16) | BIT(3));
		else
			regmap_write(vop2->sys_grf, RK3568_GRF_VO_CON1, BIT(3 + 16));
		break;
	case ROCKCHIP_VOP2_EP_HDMI0:
		die &= ~RK3568_SYS_DSP_INFACE_EN_HDMI_MUX;
		die |= RK3568_SYS_DSP_INFACE_EN_HDMI |
			   FIELD_PREP(RK3568_SYS_DSP_INFACE_EN_HDMI_MUX, vp->id);
		dip &= ~RK3568_DSP_IF_POL__HDMI_PIN_POL;
		dip |= FIELD_PREP(RK3568_DSP_IF_POL__HDMI_PIN_POL, polflags);
		break;
	case ROCKCHIP_VOP2_EP_EDP0:
		die &= ~RK3568_SYS_DSP_INFACE_EN_EDP_MUX;
		die |= RK3568_SYS_DSP_INFACE_EN_EDP |
			   FIELD_PREP(RK3568_SYS_DSP_INFACE_EN_EDP_MUX, vp->id);
		dip &= ~RK3568_DSP_IF_POL__EDP_PIN_POL;
		dip |= FIELD_PREP(RK3568_DSP_IF_POL__EDP_PIN_POL, polflags);
		break;
	case ROCKCHIP_VOP2_EP_MIPI0:
		die &= ~RK3568_SYS_DSP_INFACE_EN_MIPI0_MUX;
		die |= RK3568_SYS_DSP_INFACE_EN_MIPI0 |
			   FIELD_PREP(RK3568_SYS_DSP_INFACE_EN_MIPI0_MUX, vp->id);
		dip &= ~RK3568_DSP_IF_POL__MIPI_PIN_POL;
		dip |= FIELD_PREP(RK3568_DSP_IF_POL__MIPI_PIN_POL, polflags);
		break;
	case ROCKCHIP_VOP2_EP_MIPI1:
		die &= ~RK3568_SYS_DSP_INFACE_EN_MIPI1_MUX;
		die |= RK3568_SYS_DSP_INFACE_EN_MIPI1 |
			   FIELD_PREP(RK3568_SYS_DSP_INFACE_EN_MIPI1_MUX, vp->id);
		dip &= ~RK3568_DSP_IF_POL__MIPI_PIN_POL;
		dip |= FIELD_PREP(RK3568_DSP_IF_POL__MIPI_PIN_POL, polflags);
		break;
	case ROCKCHIP_VOP2_EP_LVDS0:
		die &= ~RK3568_SYS_DSP_INFACE_EN_LVDS0_MUX;
		die |= RK3568_SYS_DSP_INFACE_EN_LVDS0 |
			   FIELD_PREP(RK3568_SYS_DSP_INFACE_EN_LVDS0_MUX, vp->id);
		dip &= ~RK3568_DSP_IF_POL__RGB_LVDS_PIN_POL;
		dip |= FIELD_PREP(RK3568_DSP_IF_POL__RGB_LVDS_PIN_POL, polflags);
		break;
	case ROCKCHIP_VOP2_EP_LVDS1:
		die &= ~RK3568_SYS_DSP_INFACE_EN_LVDS1_MUX;
		die |= RK3568_SYS_DSP_INFACE_EN_LVDS1 |
			   FIELD_PREP(RK3568_SYS_DSP_INFACE_EN_LVDS1_MUX, vp->id);
		dip &= ~RK3568_DSP_IF_POL__RGB_LVDS_PIN_POL;
		dip |= FIELD_PREP(RK3568_DSP_IF_POL__RGB_LVDS_PIN_POL, polflags);
		break;
	default:
		drm_err(vop2->drm, "Invalid interface id %d on vp%d\n", id, vp->id);
		return 0;
	}

	dip |= RK3568_DSP_IF_POL__CFG_DONE_IMD;

	vop2_writel(vop2, RK3568_DSP_IF_EN, die);
	vop2_writel(vop2, RK3568_DSP_IF_POL, dip);

	return crtc->state->adjusted_mode.crtc_clock  * 1000LL;
}

/*
 * calc the dclk on rk3588
 * the available div of dclk is 1, 2, 4
 */
static unsigned long rk3588_calc_dclk(unsigned long child_clk, unsigned long max_dclk)
{
	if (child_clk * 4 <= max_dclk)
		return child_clk * 4;
	else if (child_clk * 2 <= max_dclk)
		return child_clk * 2;
	else if (child_clk <= max_dclk)
		return child_clk;
	else
		return 0;
}

/*
 * 4 pixclk/cycle on rk3588
 * RGB/eDP/HDMI: if_pixclk >= dclk_core
 * DP: dp_pixclk = dclk_out <= dclk_core
 * DSI: mipi_pixclk <= dclk_out <= dclk_core
 */
static unsigned long rk3588_calc_cru_cfg(struct vop2_video_port *vp, int id,
					 int *dclk_core_div, int *dclk_out_div,
					 int *if_pixclk_div, int *if_dclk_div)
{
	struct vop2 *vop2 = vp->vop2;
	struct drm_crtc *crtc = &vp->crtc;
	struct drm_display_mode *adjusted_mode = &crtc->state->adjusted_mode;
	struct rockchip_crtc_state *vcstate = to_rockchip_crtc_state(crtc->state);
	int output_mode = vcstate->output_mode;
	unsigned long v_pixclk = adjusted_mode->crtc_clock * 1000LL; /* video timing pixclk */
	unsigned long dclk_core_rate = v_pixclk >> 2;
	unsigned long dclk_rate = v_pixclk;
	unsigned long dclk_out_rate;
	unsigned long if_pixclk_rate;
	int K = 1;

	if (vop2_output_if_is_hdmi(id)) {
		/*
		 * K = 2: dclk_core = if_pixclk_rate > if_dclk_rate
		 * K = 1: dclk_core = hdmie_edp_dclk > if_pixclk_rate
		 */
		if (output_mode == ROCKCHIP_OUT_MODE_YUV420) {
			dclk_rate = dclk_rate >> 1;
			K = 2;
		}

		if_pixclk_rate = (dclk_core_rate << 1) / K;
		/*
		 * if_dclk_rate = dclk_core_rate / K;
		 * *if_pixclk_div = dclk_rate / if_pixclk_rate;
		 * *if_dclk_div = dclk_rate / if_dclk_rate;
		 */
		*if_pixclk_div = 2;
		*if_dclk_div = 4;
	} else if (vop2_output_if_is_edp(id)) {
		/*
		 * edp_pixclk = edp_dclk > dclk_core
		 */
		if_pixclk_rate = v_pixclk / K;
		dclk_rate = if_pixclk_rate * K;
		/*
		 * *if_pixclk_div = dclk_rate / if_pixclk_rate;
		 * *if_dclk_div = *if_pixclk_div;
		 */
		*if_pixclk_div = K;
		*if_dclk_div = K;
	} else if (vop2_output_if_is_dp(id)) {
		if (output_mode == ROCKCHIP_OUT_MODE_YUV420)
			dclk_out_rate = v_pixclk >> 3;
		else
			dclk_out_rate = v_pixclk >> 2;

		dclk_rate = rk3588_calc_dclk(dclk_out_rate, 600000);
		if (!dclk_rate) {
			drm_err(vop2->drm, "DP dclk_out_rate out of range, dclk_out_rate: %ld KHZ\n",
				dclk_out_rate);
			return 0;
		}
		*dclk_out_div = dclk_rate / dclk_out_rate;
	} else if (vop2_output_if_is_mipi(id)) {
		if_pixclk_rate = dclk_core_rate / K;
		/*
		 * dclk_core = dclk_out * K = if_pixclk * K = v_pixclk / 4
		 */
		dclk_out_rate = if_pixclk_rate;
		/*
		 * dclk_rate = N * dclk_core_rate N = (1,2,4 ),
		 * we get a little factor here
		 */
		dclk_rate = rk3588_calc_dclk(dclk_out_rate, 600000);
		if (!dclk_rate) {
			drm_err(vop2->drm, "MIPI dclk out of range, dclk_out_rate: %ld KHZ\n",
				dclk_out_rate);
			return 0;
		}
		*dclk_out_div = dclk_rate / dclk_out_rate;
		/*
		 * mipi pixclk == dclk_out
		 */
		*if_pixclk_div = 1;
	} else if (vop2_output_if_is_dpi(id)) {
		dclk_rate = v_pixclk;
	}

	*dclk_core_div = dclk_rate / dclk_core_rate;
	*if_pixclk_div = ilog2(*if_pixclk_div);
	*if_dclk_div = ilog2(*if_dclk_div);
	*dclk_core_div = ilog2(*dclk_core_div);
	*dclk_out_div = ilog2(*dclk_out_div);

	drm_dbg(vop2->drm, "dclk: %ld, pixclk_div: %d, dclk_div: %d\n",
		dclk_rate, *if_pixclk_div, *if_dclk_div);

	return dclk_rate;
}

/*
 * MIPI port mux on rk3588:
 * 0: Video Port2
 * 1: Video Port3
 * 3: Video Port 1(MIPI1 only)
 */
static u32 rk3588_get_mipi_port_mux(int vp_id)
{
	if (vp_id == 1)
		return 3;
	else if (vp_id == 3)
		return 1;
	else
		return 0;
}

static u32 rk3588_get_hdmi_pol(u32 flags)
{
	u32 val;

	val = (flags & DRM_MODE_FLAG_NHSYNC) ? BIT(HSYNC_POSITIVE) : 0;
	val |= (flags & DRM_MODE_FLAG_NVSYNC) ? BIT(VSYNC_POSITIVE) : 0;

	return val;
}

static unsigned long rk3588_set_intf_mux(struct vop2_video_port *vp, int id, u32 polflags)
{
	struct vop2 *vop2 = vp->vop2;
	int dclk_core_div, dclk_out_div, if_pixclk_div, if_dclk_div;
	unsigned long clock;
	u32 die, dip, div, vp_clk_div, val;

	clock = rk3588_calc_cru_cfg(vp, id, &dclk_core_div, &dclk_out_div,
				    &if_pixclk_div, &if_dclk_div);
	if (!clock)
		return 0;

	vp_clk_div = FIELD_PREP(RK3588_VP_CLK_CTRL__DCLK_CORE_DIV, dclk_core_div);
	vp_clk_div |= FIELD_PREP(RK3588_VP_CLK_CTRL__DCLK_OUT_DIV, dclk_out_div);

	die = vop2_readl(vop2, RK3568_DSP_IF_EN);
	dip = vop2_readl(vop2, RK3568_DSP_IF_POL);
	div = vop2_readl(vop2, RK3568_DSP_IF_CTRL);

	switch (id) {
	case ROCKCHIP_VOP2_EP_HDMI0:
		div &= ~RK3588_DSP_IF_EDP_HDMI0_DCLK_DIV;
		div &= ~RK3588_DSP_IF_EDP_HDMI0_PCLK_DIV;
		div |= FIELD_PREP(RK3588_DSP_IF_EDP_HDMI0_DCLK_DIV, if_dclk_div);
		div |= FIELD_PREP(RK3588_DSP_IF_EDP_HDMI0_PCLK_DIV, if_pixclk_div);
		die &= ~RK3588_SYS_DSP_INFACE_EN_EDP_HDMI0_MUX;
		die |= RK3588_SYS_DSP_INFACE_EN_HDMI0 |
			    FIELD_PREP(RK3588_SYS_DSP_INFACE_EN_EDP_HDMI0_MUX, vp->id);
		val = rk3588_get_hdmi_pol(polflags);
		regmap_write(vop2->vop_grf, RK3588_GRF_VOP_CON2, HIWORD_UPDATE(1, 1, 1));
		regmap_write(vop2->vo1_grf, RK3588_GRF_VO1_CON0, HIWORD_UPDATE(val, 6, 5));
		break;
	case ROCKCHIP_VOP2_EP_HDMI1:
		div &= ~RK3588_DSP_IF_EDP_HDMI1_DCLK_DIV;
		div &= ~RK3588_DSP_IF_EDP_HDMI1_PCLK_DIV;
		div |= FIELD_PREP(RK3588_DSP_IF_EDP_HDMI1_DCLK_DIV, if_dclk_div);
		div |= FIELD_PREP(RK3588_DSP_IF_EDP_HDMI1_PCLK_DIV, if_pixclk_div);
		die &= ~RK3588_SYS_DSP_INFACE_EN_EDP_HDMI1_MUX;
		die |= RK3588_SYS_DSP_INFACE_EN_HDMI1 |
			    FIELD_PREP(RK3588_SYS_DSP_INFACE_EN_EDP_HDMI1_MUX, vp->id);
		val = rk3588_get_hdmi_pol(polflags);
		regmap_write(vop2->vop_grf, RK3588_GRF_VOP_CON2, HIWORD_UPDATE(1, 4, 4));
		regmap_write(vop2->vo1_grf, RK3588_GRF_VO1_CON0, HIWORD_UPDATE(val, 8, 7));
		break;
	case ROCKCHIP_VOP2_EP_EDP0:
		div &= ~RK3588_DSP_IF_EDP_HDMI0_DCLK_DIV;
		div &= ~RK3588_DSP_IF_EDP_HDMI0_PCLK_DIV;
		div |= FIELD_PREP(RK3588_DSP_IF_EDP_HDMI0_DCLK_DIV, if_dclk_div);
		div |= FIELD_PREP(RK3588_DSP_IF_EDP_HDMI0_PCLK_DIV, if_pixclk_div);
		die &= ~RK3588_SYS_DSP_INFACE_EN_EDP_HDMI0_MUX;
		die |= RK3588_SYS_DSP_INFACE_EN_EDP0 |
			   FIELD_PREP(RK3588_SYS_DSP_INFACE_EN_EDP_HDMI0_MUX, vp->id);
		regmap_write(vop2->vop_grf, RK3588_GRF_VOP_CON2, HIWORD_UPDATE(1, 0, 0));
		break;
	case ROCKCHIP_VOP2_EP_EDP1:
		div &= ~RK3588_DSP_IF_EDP_HDMI1_DCLK_DIV;
		div &= ~RK3588_DSP_IF_EDP_HDMI1_PCLK_DIV;
		div |= FIELD_PREP(RK3588_DSP_IF_EDP_HDMI0_DCLK_DIV, if_dclk_div);
		div |= FIELD_PREP(RK3588_DSP_IF_EDP_HDMI0_PCLK_DIV, if_pixclk_div);
		die &= ~RK3588_SYS_DSP_INFACE_EN_EDP_HDMI1_MUX;
		die |= RK3588_SYS_DSP_INFACE_EN_EDP1 |
			   FIELD_PREP(RK3588_SYS_DSP_INFACE_EN_EDP_HDMI1_MUX, vp->id);
		regmap_write(vop2->vop_grf, RK3588_GRF_VOP_CON2, HIWORD_UPDATE(1, 3, 3));
		break;
	case ROCKCHIP_VOP2_EP_MIPI0:
		div &= ~RK3588_DSP_IF_MIPI0_PCLK_DIV;
		div |= FIELD_PREP(RK3588_DSP_IF_MIPI0_PCLK_DIV, if_pixclk_div);
		die &= ~RK3588_SYS_DSP_INFACE_EN_MIPI0_MUX;
		val = rk3588_get_mipi_port_mux(vp->id);
		die |= RK3588_SYS_DSP_INFACE_EN_MIPI0 |
			   FIELD_PREP(RK3588_SYS_DSP_INFACE_EN_MIPI0_MUX, !!val);
		break;
	case ROCKCHIP_VOP2_EP_MIPI1:
		div &= ~RK3588_DSP_IF_MIPI1_PCLK_DIV;
		div |= FIELD_PREP(RK3588_DSP_IF_MIPI1_PCLK_DIV, if_pixclk_div);
		die &= ~RK3588_SYS_DSP_INFACE_EN_MIPI1_MUX;
		val = rk3588_get_mipi_port_mux(vp->id);
		die |= RK3588_SYS_DSP_INFACE_EN_MIPI1 |
			   FIELD_PREP(RK3588_SYS_DSP_INFACE_EN_MIPI1_MUX, val);
		break;
	case ROCKCHIP_VOP2_EP_DP0:
		die &= ~RK3588_SYS_DSP_INFACE_EN_DP0_MUX;
		die |= RK3588_SYS_DSP_INFACE_EN_DP0 |
			   FIELD_PREP(RK3588_SYS_DSP_INFACE_EN_DP0_MUX, vp->id);
		dip &= ~RK3588_DSP_IF_POL__DP0_PIN_POL;
		dip |= FIELD_PREP(RK3588_DSP_IF_POL__DP0_PIN_POL, polflags);
		break;
	case ROCKCHIP_VOP2_EP_DP1:
		die &= ~RK3588_SYS_DSP_INFACE_EN_MIPI1_MUX;
		die |= RK3588_SYS_DSP_INFACE_EN_MIPI1 |
			   FIELD_PREP(RK3588_SYS_DSP_INFACE_EN_MIPI1_MUX, vp->id);
		dip &= ~RK3588_DSP_IF_POL__DP1_PIN_POL;
		dip |= FIELD_PREP(RK3588_DSP_IF_POL__DP1_PIN_POL, polflags);
		break;
	default:
		drm_err(vop2->drm, "Invalid interface id %d on vp%d\n", id, vp->id);
		return 0;
	}

	dip |= RK3568_DSP_IF_POL__CFG_DONE_IMD;

	vop2_vp_write(vp, RK3588_VP_CLK_CTRL, vp_clk_div);
	vop2_writel(vop2, RK3568_DSP_IF_EN, die);
	vop2_writel(vop2, RK3568_DSP_IF_CTRL, div);
	vop2_writel(vop2, RK3568_DSP_IF_POL, dip);

	return clock;
}

static bool is_opaque(u16 alpha)
{
	return (alpha >> 8) == 0xff;
}

static void vop2_parse_alpha(struct vop2_alpha_config *alpha_config,
			     struct vop2_alpha *alpha)
{
	int src_glb_alpha_en = is_opaque(alpha_config->src_glb_alpha_value) ? 0 : 1;
	int dst_glb_alpha_en = is_opaque(alpha_config->dst_glb_alpha_value) ? 0 : 1;
	int src_color_mode = alpha_config->src_premulti_en ?
				ALPHA_SRC_PRE_MUL : ALPHA_SRC_NO_PRE_MUL;
	int dst_color_mode = alpha_config->dst_premulti_en ?
				ALPHA_SRC_PRE_MUL : ALPHA_SRC_NO_PRE_MUL;

	alpha->src_color_ctrl.val = 0;
	alpha->dst_color_ctrl.val = 0;
	alpha->src_alpha_ctrl.val = 0;
	alpha->dst_alpha_ctrl.val = 0;

	if (!alpha_config->src_pixel_alpha_en)
		alpha->src_color_ctrl.bits.blend_mode = ALPHA_GLOBAL;
	else if (alpha_config->src_pixel_alpha_en && !src_glb_alpha_en)
		alpha->src_color_ctrl.bits.blend_mode = ALPHA_PER_PIX;
	else
		alpha->src_color_ctrl.bits.blend_mode = ALPHA_PER_PIX_GLOBAL;

	alpha->src_color_ctrl.bits.alpha_en = 1;

	if (alpha->src_color_ctrl.bits.blend_mode == ALPHA_GLOBAL) {
		alpha->src_color_ctrl.bits.color_mode = src_color_mode;
		alpha->src_color_ctrl.bits.factor_mode = SRC_FAC_ALPHA_SRC_GLOBAL;
	} else if (alpha->src_color_ctrl.bits.blend_mode == ALPHA_PER_PIX) {
		alpha->src_color_ctrl.bits.color_mode = src_color_mode;
		alpha->src_color_ctrl.bits.factor_mode = SRC_FAC_ALPHA_ONE;
	} else {
		alpha->src_color_ctrl.bits.color_mode = ALPHA_SRC_PRE_MUL;
		alpha->src_color_ctrl.bits.factor_mode = SRC_FAC_ALPHA_SRC_GLOBAL;
	}
	alpha->src_color_ctrl.bits.glb_alpha = alpha_config->src_glb_alpha_value >> 8;
	alpha->src_color_ctrl.bits.alpha_mode = ALPHA_STRAIGHT;
	alpha->src_color_ctrl.bits.alpha_cal_mode = ALPHA_SATURATION;

	alpha->dst_color_ctrl.bits.alpha_mode = ALPHA_STRAIGHT;
	alpha->dst_color_ctrl.bits.alpha_cal_mode = ALPHA_SATURATION;
	alpha->dst_color_ctrl.bits.blend_mode = ALPHA_GLOBAL;
	alpha->dst_color_ctrl.bits.glb_alpha = alpha_config->dst_glb_alpha_value >> 8;
	alpha->dst_color_ctrl.bits.color_mode = dst_color_mode;
	alpha->dst_color_ctrl.bits.factor_mode = ALPHA_SRC_INVERSE;

	alpha->src_alpha_ctrl.bits.alpha_mode = ALPHA_STRAIGHT;
	alpha->src_alpha_ctrl.bits.blend_mode = alpha->src_color_ctrl.bits.blend_mode;
	alpha->src_alpha_ctrl.bits.alpha_cal_mode = ALPHA_SATURATION;
	alpha->src_alpha_ctrl.bits.factor_mode = ALPHA_ONE;

	alpha->dst_alpha_ctrl.bits.alpha_mode = ALPHA_STRAIGHT;
	if (alpha_config->dst_pixel_alpha_en && !dst_glb_alpha_en)
		alpha->dst_alpha_ctrl.bits.blend_mode = ALPHA_PER_PIX;
	else
		alpha->dst_alpha_ctrl.bits.blend_mode = ALPHA_PER_PIX_GLOBAL;
	alpha->dst_alpha_ctrl.bits.alpha_cal_mode = ALPHA_NO_SATURATION;
	alpha->dst_alpha_ctrl.bits.factor_mode = ALPHA_SRC_INVERSE;
}

static int vop2_find_start_mixer_id_for_vp(struct vop2 *vop2, u8 port_id)
{
	struct vop2_video_port *vp;
	int used_layer = 0;
	int i;

	for (i = 0; i < port_id; i++) {
		vp = &vop2->vps[i];
		used_layer += hweight32(vp->win_mask);
	}

	return used_layer;
}

static void vop2_setup_cluster_alpha(struct vop2 *vop2, struct vop2_win *main_win)
{
	u32 offset = (main_win->data->phys_id * 0x10);
	struct vop2_alpha_config alpha_config;
	struct vop2_alpha alpha;
	struct drm_plane_state *bottom_win_pstate;
	bool src_pixel_alpha_en = false;
	u16 src_glb_alpha_val, dst_glb_alpha_val;
	bool premulti_en = false;
	bool swap = false;

	/* At one win mode, win0 is dst/bottom win, and win1 is a all zero src/top win */
	bottom_win_pstate = main_win->base.state;
	src_glb_alpha_val = 0;
	dst_glb_alpha_val = main_win->base.state->alpha;

	if (!bottom_win_pstate->fb)
		return;

	alpha_config.src_premulti_en = premulti_en;
	alpha_config.dst_premulti_en = false;
	alpha_config.src_pixel_alpha_en = src_pixel_alpha_en;
	alpha_config.dst_pixel_alpha_en = true; /* alpha value need transfer to next mix */
	alpha_config.src_glb_alpha_value = src_glb_alpha_val;
	alpha_config.dst_glb_alpha_value = dst_glb_alpha_val;
	vop2_parse_alpha(&alpha_config, &alpha);

	alpha.src_color_ctrl.bits.src_dst_swap = swap;
	vop2_writel(vop2, RK3568_CLUSTER0_MIX_SRC_COLOR_CTRL + offset,
		    alpha.src_color_ctrl.val);
	vop2_writel(vop2, RK3568_CLUSTER0_MIX_DST_COLOR_CTRL + offset,
		    alpha.dst_color_ctrl.val);
	vop2_writel(vop2, RK3568_CLUSTER0_MIX_SRC_ALPHA_CTRL + offset,
		    alpha.src_alpha_ctrl.val);
	vop2_writel(vop2, RK3568_CLUSTER0_MIX_DST_ALPHA_CTRL + offset,
		    alpha.dst_alpha_ctrl.val);
}

static void rk3568_vop2_setup_alpha(struct vop2_video_port *vp)
{
	struct vop2 *vop2 = vp->vop2;
	struct drm_framebuffer *fb;
	struct vop2_alpha_config alpha_config;
	struct vop2_alpha alpha;
	struct drm_plane *plane;
	int pixel_alpha_en;
	int premulti_en, gpremulti_en = 0;
	int mixer_id;
	u32 offset;
	bool bottom_layer_alpha_en = false;
	u32 dst_global_alpha = DRM_BLEND_ALPHA_OPAQUE;

	mixer_id = vop2_find_start_mixer_id_for_vp(vop2, vp->id);
	alpha_config.dst_pixel_alpha_en = true; /* alpha value need transfer to next mix */

	drm_atomic_crtc_for_each_plane(plane, &vp->crtc) {
		struct vop2_win *win = to_vop2_win(plane);

		if (plane->state->normalized_zpos == 0 &&
		    !is_opaque(plane->state->alpha) &&
		    !vop2_cluster_window(win)) {
			/*
			 * If bottom layer have global alpha effect [except cluster layer,
			 * because cluster have deal with bottom layer global alpha value
			 * at cluster mix], bottom layer mix need deal with global alpha.
			 */
			bottom_layer_alpha_en = true;
			dst_global_alpha = plane->state->alpha;
		}
	}

	drm_atomic_crtc_for_each_plane(plane, &vp->crtc) {
		struct vop2_win *win = to_vop2_win(plane);
		int zpos = plane->state->normalized_zpos;

		if (plane->state->pixel_blend_mode == DRM_MODE_BLEND_PREMULTI)
			premulti_en = 1;
		else
			premulti_en = 0;

		plane = &win->base;
		fb = plane->state->fb;

		pixel_alpha_en = fb->format->has_alpha;

		alpha_config.src_premulti_en = premulti_en;

		if (bottom_layer_alpha_en && zpos == 1) {
			gpremulti_en = premulti_en;
			/* Cd = Cs + (1 - As) * Cd * Agd */
			alpha_config.dst_premulti_en = false;
			alpha_config.src_pixel_alpha_en = pixel_alpha_en;
			alpha_config.src_glb_alpha_value = plane->state->alpha;
			alpha_config.dst_glb_alpha_value = dst_global_alpha;
		} else if (vop2_cluster_window(win)) {
			/* Mix output data only have pixel alpha */
			alpha_config.dst_premulti_en = true;
			alpha_config.src_pixel_alpha_en = true;
			alpha_config.src_glb_alpha_value = DRM_BLEND_ALPHA_OPAQUE;
			alpha_config.dst_glb_alpha_value = DRM_BLEND_ALPHA_OPAQUE;
		} else {
			/* Cd = Cs + (1 - As) * Cd */
			alpha_config.dst_premulti_en = true;
			alpha_config.src_pixel_alpha_en = pixel_alpha_en;
			alpha_config.src_glb_alpha_value = plane->state->alpha;
			alpha_config.dst_glb_alpha_value = DRM_BLEND_ALPHA_OPAQUE;
		}

		vop2_parse_alpha(&alpha_config, &alpha);

		offset = (mixer_id + zpos - 1) * 0x10;
		vop2_writel(vop2, RK3568_MIX0_SRC_COLOR_CTRL + offset,
			    alpha.src_color_ctrl.val);
		vop2_writel(vop2, RK3568_MIX0_DST_COLOR_CTRL + offset,
			    alpha.dst_color_ctrl.val);
		vop2_writel(vop2, RK3568_MIX0_SRC_ALPHA_CTRL + offset,
			    alpha.src_alpha_ctrl.val);
		vop2_writel(vop2, RK3568_MIX0_DST_ALPHA_CTRL + offset,
			    alpha.dst_alpha_ctrl.val);
	}

	if (vp->id == 0) {
		if (bottom_layer_alpha_en) {
			/* Transfer pixel alpha to hdr mix */
			alpha_config.src_premulti_en = gpremulti_en;
			alpha_config.dst_premulti_en = true;
			alpha_config.src_pixel_alpha_en = true;
			alpha_config.src_glb_alpha_value = DRM_BLEND_ALPHA_OPAQUE;
			alpha_config.dst_glb_alpha_value = DRM_BLEND_ALPHA_OPAQUE;
			vop2_parse_alpha(&alpha_config, &alpha);

			vop2_writel(vop2, RK3568_HDR0_SRC_COLOR_CTRL,
				    alpha.src_color_ctrl.val);
			vop2_writel(vop2, RK3568_HDR0_DST_COLOR_CTRL,
				    alpha.dst_color_ctrl.val);
			vop2_writel(vop2, RK3568_HDR0_SRC_ALPHA_CTRL,
				    alpha.src_alpha_ctrl.val);
			vop2_writel(vop2, RK3568_HDR0_DST_ALPHA_CTRL,
				    alpha.dst_alpha_ctrl.val);
		} else {
			vop2_writel(vop2, RK3568_HDR0_SRC_COLOR_CTRL, 0);
		}
	}
}

static void rk3568_vop2_setup_layer_mixer(struct vop2_video_port *vp)
{
	struct vop2 *vop2 = vp->vop2;
	struct drm_plane *plane;
	u32 layer_sel = 0;
	u32 port_sel;
	unsigned int nlayer, ofs;
	u32 ovl_ctrl;
	int i;
	struct vop2_video_port *vp0 = &vop2->vps[0];
	struct vop2_video_port *vp1 = &vop2->vps[1];
	struct vop2_video_port *vp2 = &vop2->vps[2];
	struct rockchip_crtc_state *vcstate = to_rockchip_crtc_state(vp->crtc.state);

	ovl_ctrl = vop2_readl(vop2, RK3568_OVL_CTRL);
	ovl_ctrl |= RK3568_OVL_CTRL__LAYERSEL_REGDONE_IMD;
	if (vcstate->yuv_overlay)
		ovl_ctrl |= RK3568_OVL_CTRL__YUV_MODE(vp->id);
	else
		ovl_ctrl &= ~RK3568_OVL_CTRL__YUV_MODE(vp->id);

	vop2_writel(vop2, RK3568_OVL_CTRL, ovl_ctrl);

	port_sel = vop2_readl(vop2, RK3568_OVL_PORT_SEL);
	port_sel &= RK3568_OVL_PORT_SEL__SEL_PORT;

	if (vp0->nlayers)
		port_sel |= FIELD_PREP(RK3568_OVL_PORT_SET__PORT0_MUX,
				     vp0->nlayers - 1);
	else
		port_sel |= FIELD_PREP(RK3568_OVL_PORT_SET__PORT0_MUX, 8);

	if (vp1->nlayers)
		port_sel |= FIELD_PREP(RK3568_OVL_PORT_SET__PORT1_MUX,
				     (vp0->nlayers + vp1->nlayers - 1));
	else
		port_sel |= FIELD_PREP(RK3568_OVL_PORT_SET__PORT1_MUX, 8);

	if (vp2->nlayers)
		port_sel |= FIELD_PREP(RK3568_OVL_PORT_SET__PORT2_MUX,
			(vp2->nlayers + vp1->nlayers + vp0->nlayers - 1));
	else
		port_sel |= FIELD_PREP(RK3568_OVL_PORT_SET__PORT2_MUX, 8);

	layer_sel = vop2_readl(vop2, RK3568_OVL_LAYER_SEL);

	ofs = 0;
	for (i = 0; i < vp->id; i++)
		ofs += vop2->vps[i].nlayers;

	nlayer = 0;
	drm_atomic_crtc_for_each_plane(plane, &vp->crtc) {
		struct vop2_win *win = to_vop2_win(plane);

		switch (win->data->phys_id) {
		case ROCKCHIP_VOP2_CLUSTER0:
			port_sel &= ~RK3568_OVL_PORT_SEL__CLUSTER0;
			port_sel |= FIELD_PREP(RK3568_OVL_PORT_SEL__CLUSTER0, vp->id);
			break;
		case ROCKCHIP_VOP2_CLUSTER1:
			port_sel &= ~RK3568_OVL_PORT_SEL__CLUSTER1;
			port_sel |= FIELD_PREP(RK3568_OVL_PORT_SEL__CLUSTER1, vp->id);
			break;
		case ROCKCHIP_VOP2_CLUSTER2:
			port_sel &= ~RK3588_OVL_PORT_SEL__CLUSTER2;
			port_sel |= FIELD_PREP(RK3588_OVL_PORT_SEL__CLUSTER2, vp->id);
			break;
		case ROCKCHIP_VOP2_CLUSTER3:
			port_sel &= ~RK3588_OVL_PORT_SEL__CLUSTER3;
			port_sel |= FIELD_PREP(RK3588_OVL_PORT_SEL__CLUSTER3, vp->id);
			break;
		case ROCKCHIP_VOP2_ESMART0:
			port_sel &= ~RK3568_OVL_PORT_SEL__ESMART0;
			port_sel |= FIELD_PREP(RK3568_OVL_PORT_SEL__ESMART0, vp->id);
			break;
		case ROCKCHIP_VOP2_ESMART1:
			port_sel &= ~RK3568_OVL_PORT_SEL__ESMART1;
			port_sel |= FIELD_PREP(RK3568_OVL_PORT_SEL__ESMART1, vp->id);
			break;
		case ROCKCHIP_VOP2_ESMART2:
			port_sel &= ~RK3588_OVL_PORT_SEL__ESMART2;
			port_sel |= FIELD_PREP(RK3588_OVL_PORT_SEL__ESMART2, vp->id);
			break;
		case ROCKCHIP_VOP2_ESMART3:
			port_sel &= ~RK3588_OVL_PORT_SEL__ESMART3;
			port_sel |= FIELD_PREP(RK3588_OVL_PORT_SEL__ESMART3, vp->id);
			break;
		case ROCKCHIP_VOP2_SMART0:
			port_sel &= ~RK3568_OVL_PORT_SEL__SMART0;
			port_sel |= FIELD_PREP(RK3568_OVL_PORT_SEL__SMART0, vp->id);
			break;
		case ROCKCHIP_VOP2_SMART1:
			port_sel &= ~RK3568_OVL_PORT_SEL__SMART1;
			port_sel |= FIELD_PREP(RK3568_OVL_PORT_SEL__SMART1, vp->id);
			break;
		}

		layer_sel &= ~RK3568_OVL_LAYER_SEL__LAYER(plane->state->normalized_zpos + ofs,
							  0x7);
		layer_sel |= RK3568_OVL_LAYER_SEL__LAYER(plane->state->normalized_zpos + ofs,
							 win->data->layer_sel_id[vp->id]);
		nlayer++;
	}

	/* configure unused layers to 0x5 (reserved) */
	for (; nlayer < vp->nlayers; nlayer++) {
		layer_sel &= ~RK3568_OVL_LAYER_SEL__LAYER(nlayer + ofs, 0x7);
		layer_sel |= RK3568_OVL_LAYER_SEL__LAYER(nlayer + ofs, 5);
	}

	vop2_writel(vop2, RK3568_OVL_LAYER_SEL, layer_sel);
	vop2_writel(vop2, RK3568_OVL_PORT_SEL, port_sel);
}

static void rk3568_vop2_setup_dly_for_windows(struct vop2_video_port *vp)
{
	struct vop2 *vop2 = vp->vop2;
	struct vop2_win *win;
	int i = 0;
	u32 cdly = 0, sdly = 0;

	for (i = 0; i < vop2->data->win_size; i++) {
		u32 dly;

		win = &vop2->win[i];
		dly = win->delay;

		switch (win->data->phys_id) {
		case ROCKCHIP_VOP2_CLUSTER0:
			cdly |= FIELD_PREP(RK3568_CLUSTER_DLY_NUM__CLUSTER0_0, dly);
			cdly |= FIELD_PREP(RK3568_CLUSTER_DLY_NUM__CLUSTER0_1, dly);
			break;
		case ROCKCHIP_VOP2_CLUSTER1:
			cdly |= FIELD_PREP(RK3568_CLUSTER_DLY_NUM__CLUSTER1_0, dly);
			cdly |= FIELD_PREP(RK3568_CLUSTER_DLY_NUM__CLUSTER1_1, dly);
			break;
		case ROCKCHIP_VOP2_ESMART0:
			sdly |= FIELD_PREP(RK3568_SMART_DLY_NUM__ESMART0, dly);
			break;
		case ROCKCHIP_VOP2_ESMART1:
			sdly |= FIELD_PREP(RK3568_SMART_DLY_NUM__ESMART1, dly);
			break;
		case ROCKCHIP_VOP2_SMART0:
			sdly |= FIELD_PREP(RK3568_SMART_DLY_NUM__SMART0, dly);
			break;
		case ROCKCHIP_VOP2_SMART1:
			sdly |= FIELD_PREP(RK3568_SMART_DLY_NUM__SMART1, dly);
			break;
		}
	}

	vop2_writel(vop2, RK3568_CLUSTER_DLY_NUM, cdly);
	vop2_writel(vop2, RK3568_SMART_DLY_NUM, sdly);
}

static void rk3568_vop2_setup_overlay(struct vop2_video_port *vp)
{
	struct vop2 *vop2 = vp->vop2;
	struct drm_crtc *crtc = &vp->crtc;
	struct drm_plane *plane;

	vp->win_mask = 0;

	drm_atomic_crtc_for_each_plane(plane, crtc) {
		struct vop2_win *win = to_vop2_win(plane);

		win->delay = win->data->dly[VOP2_DLY_MODE_DEFAULT];

		vp->win_mask |= BIT(win->data->phys_id);

		if (vop2_cluster_window(win))
			vop2_setup_cluster_alpha(vop2, win);
	}

	if (!vp->win_mask)
		return;

	rk3568_vop2_setup_layer_mixer(vp);
	rk3568_vop2_setup_alpha(vp);
	rk3568_vop2_setup_dly_for_windows(vp);
}

static void rk3568_vop2_setup_bg_dly(struct vop2_video_port *vp)
{
	struct drm_crtc *crtc = &vp->crtc;
	struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	u16 hdisplay = mode->crtc_hdisplay;
	u16 hsync_len = mode->crtc_hsync_end - mode->crtc_hsync_start;
	u32 bg_dly;
	u32 pre_scan_dly;

	bg_dly = vp->data->pre_scan_max_dly[3];
	vop2_writel(vp->vop2, RK3568_VP_BG_MIX_CTRL(vp->id),
		    FIELD_PREP(RK3568_VP_BG_MIX_CTRL__BG_DLY, bg_dly));

	pre_scan_dly = ((bg_dly + (hdisplay >> 1) - 1) << 16) | hsync_len;
	vop2_vp_write(vp, RK3568_VP_PRE_SCAN_HTIMING, pre_scan_dly);
}

static const struct vop2_ops rk3568_vop_ops = {
	.setup_intf_mux = rk3568_set_intf_mux,
	.setup_bg_dly = rk3568_vop2_setup_bg_dly,
	.setup_overlay = rk3568_vop2_setup_overlay,
};

static const struct vop2_ops rk3588_vop_ops = {
	.setup_intf_mux = rk3588_set_intf_mux,
	.setup_bg_dly = rk3568_vop2_setup_bg_dly,
	.setup_overlay = rk3568_vop2_setup_overlay,
};

static const struct vop2_data rk3566_vop = {
	.feature = VOP2_FEATURE_HAS_SYS_GRF,
	.nr_vps = 3,
	.max_input = { 4096, 2304 },
	.max_output = { 4096, 2304 },
	.vp = rk3568_vop_video_ports,
	.win = rk3568_vop_win_data,
	.win_size = ARRAY_SIZE(rk3568_vop_win_data),
	.regs_dump = rk3568_regs_dump,
	.regs_dump_size = ARRAY_SIZE(rk3568_regs_dump),
	.ops = &rk3568_vop_ops,
	.soc_id = 3566,
};

static const struct vop2_data rk3568_vop = {
	.feature = VOP2_FEATURE_HAS_SYS_GRF,
	.nr_vps = 3,
	.max_input = { 4096, 2304 },
	.max_output = { 4096, 2304 },
	.vp = rk3568_vop_video_ports,
	.win = rk3568_vop_win_data,
	.win_size = ARRAY_SIZE(rk3568_vop_win_data),
	.regs_dump = rk3568_regs_dump,
	.regs_dump_size = ARRAY_SIZE(rk3568_regs_dump),
	.ops = &rk3568_vop_ops,
	.soc_id = 3568,
};

static const struct vop2_data rk3588_vop = {
	.feature = VOP2_FEATURE_HAS_SYS_GRF | VOP2_FEATURE_HAS_VO1_GRF |
		   VOP2_FEATURE_HAS_VOP_GRF | VOP2_FEATURE_HAS_SYS_PMU,
	.nr_vps = 4,
	.max_input = { 4096, 4320 },
	.max_output = { 4096, 4320 },
	.vp = rk3588_vop_video_ports,
	.win = rk3588_vop_win_data,
	.win_size = ARRAY_SIZE(rk3588_vop_win_data),
	.regs_dump = rk3588_regs_dump,
	.regs_dump_size = ARRAY_SIZE(rk3588_regs_dump),
	.ops = &rk3588_vop_ops,
	.soc_id = 3588,
};

static const struct of_device_id vop2_dt_match[] = {
	{
		.compatible = "rockchip,rk3566-vop",
		.data = &rk3566_vop,
	}, {
		.compatible = "rockchip,rk3568-vop",
		.data = &rk3568_vop,
	}, {
		.compatible = "rockchip,rk3588-vop",
		.data = &rk3588_vop
	}, {
	},
};
MODULE_DEVICE_TABLE(of, vop2_dt_match);

static int vop2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	return component_add(dev, &vop2_component_ops);
}

static void vop2_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &vop2_component_ops);
}

struct platform_driver vop2_platform_driver = {
	.probe = vop2_probe,
	.remove_new = vop2_remove,
	.driver = {
		.name = "rockchip-vop2",
		.of_match_table = vop2_dt_match,
	},
};
