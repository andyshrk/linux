// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021-2022 Siengine Technology, Inc.
 *
 * Sienginie Camera Interface driver
 *
 * Author: Siengine Technology, Inc.
 */

#ifndef SE_NR_REG_H_
#define SE_NR_REG_H

/*! Register: vi_iccl: Internal clock  control register (rw) MRV_BASE + 0x0000001c */

/*! Slice: vi_cp_clk_enable:*/
/*! color processing clock enable */
/* 1: processing mode */
/* 0: power safe */
#define MRV_VI_CP_CLK_ENABLE
#define MRV_VI_CP_CLK_ENABLE_MASK 0x00000002U
#define MRV_VI_CP_CLK_ENABLE_SHIFT 1U
/*! Slice: vi_isp_clk_enable:*/
/*! isp processing clock enable */
/* 1: processing mode */
/* 0: power safe */
#define MRV_VI_ISP_CLK_ENABLE
#define MRV_VI_ISP_CLK_ENABLE_MASK 0x00000001U
#define MRV_VI_ISP_CLK_ENABLE_SHIFT 0U

/*! Slice: ISP_GEN_CFG_UPD:*/
/*! 1: generate frame synchronous configuration signal at the output
 *of ISP for shadow registers of the following processing modules,
 *write only.
 */
#define NR200_ISP_CTRL_GEN_CFG_UPD
#define NR200_ISP_CTRL_GEN_CFG_UPD_MASK	0x00000400U
#define NR200_ISP_CTRL_GEN_CFG_UPD_SHIFT 10U

/*! Slice: ISP_CFG_UPD:*/
/*! 1: immediately configure (update) shadow registers, write only */
#define NR200_ISP_CTRL_CFG_UPD
#define NR200_ISP_CTRL_CFG_UPD_MASK 0x00000200U
#define NR200_ISP_CTRL_CFG_UPD_SHIFT 9U

/*! Slice: ISP_CFG_UPD_PERMANENT:*/
/*! 1: permanent configure (update) shadow registers on frame end.*/
#define NR200_ISP_CFG_UPD_PERMANENT
#define NR200_ISP_CFG_UPD_PERMANENT_MASK 0x00000100U
#define NR200_ISP_CFG_UPD_PERMANENT_SHIFT 8U

/*! Slice: ISP_CFG_BAYER_PATTERN_PERMANENT:*/
/*! 1: permanent configure (update) shadow registers on frame end.*/
#define NR200_ISP_BAYER_PATTERN
#define NR200_ISP_BAYER_PATTERN_MASK 0x00000018U
#define NR200_ISP_BAYER_PATTERN_SHIFT 3U

/*! Slice: ISP_ENABLE:*/
/*! 1: ISP data output enabled */
/* 0: ISP data output disabled */
/*
 *Controls output formatter frame synchronously, if isp_gen_cfg_upd
 *is used to activate this bit. For immediate update isp_cfg_upd
 *must be used.
 */
#define NR200_ISP_ENABLE
#define NR200_ISP_ENABLE_MASK 0x00000001U
#define NR200_ISP_ENABLE_SHIFT 0U

#define NR200_ISP_OUT_H_SIZE
#define NR200_ISP_OUT_H_SIZE_MASK	0x00007FFFU
#define NR200_ISP_OUT_H_SIZE_SHIFT	0U

#define NR200_ISP_OUT_V_SIZE
#define NR200_ISP_OUT_V_SIZE_MASK	0x00003FFFU
#define NR200_ISP_OUT_V_SIZE_SHIFT	0U



/* Register: ISP_DENOISE3D_CTRL     0x00003700 */
/* Slice: 5:5 denoise3d_write_ref_en */
#define DENOISE3D_WRITE_REF_EN
#define DENOISE3D_WRITE_REF_EN_MASK 0x00000080U
#define DENOISE3D_WRITE_REF_EN_SHIFT 7U

/* Register: ISP_DENOISE3D_CTRL     0x00003700 */
/* Slice: 5:5 denoise3d_soft_reset */
#define DENOISE3D_SOFT_RESET
#define DENOISE3D_SOFT_RESET_MASK  0x00000020U
#define DENOISE3D_SOFT_RESET_SHIFT 5U
/* Slice: 4:4 denoise3d_horizontal_en */
#define DENOISE3D_HORIZONTAL_EN
#define DENOISE3D_HORIZONTAL_EN_MASK 0x00000010U
#define DENOISE3D_HORIZONTAL_EN_SHIFT 4U
/* Slice: 3:3 denoise3d_vertical_en */
#define DENOISE3D_VERTICAL_EN
#define DENOISE3D_VERTICAL_EN_MASK 0x00000008U
#define DENOISE3D_VERTICAL_EN_SHIFT 3U
/* Slice: 2:2 denoise3d_temperal_en */
#define DENOISE3D_TEMPERAL_EN
#define DENOISE3D_TEMPERAL_EN_MASK 0x00000004U
#define DENOISE3D_TEMPERAL_EN_SHIFT 2U
/* Slice: 1:1 denoise3d_dilate_en */
#define DENOISE3D_DILATE_EN
#define DENOISE3D_DILATE_EN_MASK 0x00000002U
#define DENOISE3D_DILATE_EN_SHIFT 1U
/* Slice: 0:0 denoise3d_enable */
#define DENOISE3D_ENABLE
#define DENOISE3D_ENABLE_MASK 0x00000001U
#define DENOISE3D_ENABLE_SHIFT 0U
/* Register: ISP_DENOISE3D_STRENGTH     0x00003704       */
/* Slice: 29:19 denoise3d_update_temperal */
#define DENOISE3D_UPDATE_TEMPERAL
#define DENOISE3D_UPDATE_TEMPERAL_MASK  0x3FF80000U
#define DENOISE3D_UPDATE_TEMPERAL_SHIFT 19U
/* Slice: 18 : 8 denoise3d_update_spacial */
#define DENOISE3D_UPDATE_SPACIAL
#define DENOISE3D_UPDATE_SPACIAL_MASK 0x0007FF00U
#define DENOISE3D_UPDATE_SPACIAL_SHIFT 8U
/* Slice: 7 : 0 denoise3d_strength */
#define DENOISE3D_STRENGTH
#define DENOISE3D_STRENGTH_MASK 0x000000FFU
#define DENOISE3D_STRENGTH_SHIFT 0U
/* Register: ISP_DENOISE3D_EDGE_H       0x00003708 */
/* Slice: 27:20 denoise3d_strength_curve_spacial */
#define DENOISE3D_STRENGTH_CURVE_SPACIAL
#define DENOISE3D_STRENGTH_CURVE_SPACIAL_MASK 0x0FF00000U
#define DENOISE3D_STRENGTH_CURVE_SPACIAL_SHIFT 20U
/* Slice: 19 : 0        denoise3d_thr_edge_h_inv */
#define DENOISE3D_THR_EDGE_H_INV
#define DENOISE3D_THR_EDGE_H_INV_MASK 0x000FFFFFU
#define DENOISE3D_THR_EDGE_H_INV_SHIFT 0U
/* Register: ISP_DENOISE3D_EDGE_V       0x0000370C       */
/* Slice: 27:20 denoise3d_strength_curve_temperal */
#define DENOISE3D_STRENGTH_CURVE_TEMPERAL
#define DENOISE3D_STRENGTH_CURVE_TEMPERAL_MASK 0x0FF00000U
#define DENOISE3D_STRENGTH_CURVE_TEMPERAL_SHIFT 20U
/* Slice: 19 : 0        denoise3d_thr_edge_v_inv */
#define DENOISE3D_THR_EDGE_V_INV
#define DENOISE3D_THR_EDGE_V_INV_MASK 0x000FFFFFU
#define DENOISE3D_THR_EDGE_V_INV_SHIFT 0U
/* Register: ISP_DENOISE3D_RANGE_S      0x00003710       */
/* Slice: 19:0  denoise3d_range_s_inv */
#define DENOISE3D_RANGE_S_INV
#define DENOISE3D_RANGE_S_INV_MASK 0x000FFFFFU
#define DENOISE3D_RANGE_S_INV_SHIFT 0U
/* Register: ISP_DENOISE3D_RANGE_T      0x00003714       */
/* Slice: 29:25 denoise3d_range_t_h */
#define DENOISE3D_RANGE_T_H
#define DENOISE3D_RANGE_T_H_MASK 0x3E000000U
#define DENOISE3D_RANGE_T_H_SHIFT 25U
/* Slice: 24:20 denoise3d_range_t_v */
#define DENOISE3D_RANGE_T_V
#define DENOISE3D_RANGE_T_V_MASK 0x01F00000U
#define DENOISE3D_RANGE_T_V_SHIFT 20U
/* Slice: 19 : 0        denoise3d_range_t_inv */
#define DENOISE3D_RANGE_T_INV
#define DENOISE3D_RANGE_T_INV_MASK 0x000FFFFFU
#define DENOISE3D_RANGE_T_INV_SHIFT 0U
/* Register: ISP_DENOISE3D_MOTION       0x00003718 */
/* Slice: 24:20 denoise3d_range_d */
#define DENOISE3D_RANGE_D
#define DENOISE3D_RANGE_D_MASK 0x01F00000U
#define DENOISE3D_RANGE_D_SHIFT 20U
/* Slice: 19 : 0        denoise3d_motion_inv */
#define DENOISE3D_MOTION_INV
#define DENOISE3D_MOTION_INV_MASK 0x000FFFFFU
#define DENOISE3D_MOTION_INV_SHIFT 0U
/* Register: ISP_DENOISE3D_DELTA_INV    0x0000371C       */
/* Slice: 29:20 denoise3d_delta_h_inv */
#define DENOISE3D_DELTA_H_INV
#define DENOISE3D_DELTA_H_INV_MASK 0x3FF00000U
#define DENOISE3D_DELTA_H_INV_SHIFT 20U
/* Slice: 19 : 10       denoise3d_delta_v_inv */
#define DENOISE3D_DELTA_V_INV
#define DENOISE3D_DELTA_V_INV_MASK 0x000FFC00U
#define DENOISE3D_DELTA_V_INV_SHIFT 10U
/* Slice: 9 : 0 denoise3d_delta_t_inv */
#define DENOISE3D_DELTA_T_INV
#define DENOISE3D_DELTA_T_INV_MASK 0x000003FFU
#define DENOISE3D_DELTA_T_INV_SHIFT 0U
/* Register: ISP_DENOISE3D_CURVE_S_0    0x00003720       */
/* Slice: 29:20 denoise3d_spacial_curve0 */
#define DENOISE3D_SPACIAL_CURVE0
#define DENOISE3D_SPACIAL_CURVE0_MASK 0x3FF00000U
#define DENOISE3D_SPACIAL_CURVE0_SHIFT 20U
/* Slice: 19 : 10       denoise3d_spacial_curve1 */
#define DENOISE3D_SPACIAL_CURVE1
#define DENOISE3D_SPACIAL_CURVE1_MASK 0x000FFC00U
#define DENOISE3D_SPACIAL_CURVE1_SHIFT 10U
/* Slice: 9 : 0 denoise3d_spacial_curve2 */
#define DENOISE3D_SPACIAL_CURVE2
#define DENOISE3D_SPACIAL_CURVE2_MASK 0x000003FFU
#define DENOISE3D_SPACIAL_CURVE2_SHIFT 0U
/* Register: ISP_DENOISE3D_CURVE_S_1    0x00003724       */
/* Slice: 29 : 20       denoise3d_spacial_curve3 */
#define DENOISE3D_SPACIAL_CURVE3
#define DENOISE3D_SPACIAL_CURVE3_MASK 0x3FF00000U
#define DENOISE3D_SPACIAL_CURVE3_SHIFT 20U
/* Slice: 19 : 10       denoise3d_spacial_curve4 */
#define DENOISE3D_SPACIAL_CURVE4
#define DENOISE3D_SPACIAL_CURVE4_MASK 0x000FFC00U
#define DENOISE3D_SPACIAL_CURVE4_SHIFT 10U
/* Slice: 9 : 0 denoise3d_spacial_curve5 */
#define DENOISE3D_SPACIAL_CURVE5
#define DENOISE3D_SPACIAL_CURVE5_MASK 0x000003FFU
#define DENOISE3D_SPACIAL_CURVE5_SHIFT 0U
/* Register: ISP_DENOISE3D_CURVE_S_2    0x00003728       */
/* Slice: 29 : 20       denoise3d_spacial_curve6 */
#define DENOISE3D_SPACIAL_CURVE6
#define DENOISE3D_SPACIAL_CURVE6_MASK 0x3FF00000U
#define DENOISE3D_SPACIAL_CURVE6_SHIFT 20U
/* Slice: 19 : 10       denoise3d_spacial_curve7 */
#define DENOISE3D_SPACIAL_CURVE7
#define DENOISE3D_SPACIAL_CURVE7_MASK 0x000FFC00U
#define DENOISE3D_SPACIAL_CURVE7_SHIFT 10U
/* Slice: 9 : 0 denoise3d_spacial_curve8 */
#define DENOISE3D_SPACIAL_CURVE8
#define DENOISE3D_SPACIAL_CURVE8_MASK 0x000003FFU
#define DENOISE3D_SPACIAL_CURVE8_SHIFT 0U
/* Register: ISP_DENOISE3D_CURVE_S_3    0x0000372C       */
/* Slice: 29 : 20       denoise3d_spacial_curve9 */
#define DENOISE3D_SPACIAL_CURVE9
#define DENOISE3D_SPACIAL_CURVE9_MASK 0x3FF00000U
#define DENOISE3D_SPACIAL_CURVE9_SHIFT 20U
/* Slice: 19 : 10       denoise3d_spacial_curve10 */
#define DENOISE3D_SPACIAL_CURVE10
#define DENOISE3D_SPACIAL_CURVE10_MASK 0x000FFC00U
#define DENOISE3D_SPACIAL_CURVE10_SHIFT 10U
/* Slice: 9 : 0 denoise3d_spacial_curve11 */
#define DENOISE3D_SPACIAL_CURVE11
#define DENOISE3D_SPACIAL_CURVE11_MASK 0x000003FFU
#define DENOISE3D_SPACIAL_CURVE11_SHIFT 0U
/* Register: ISP_DENOISE3D_CURVE_S_4    0x00003730       */
/* Slice: 29 : 20       denoise3d_spacial_curve12 */
#define DENOISE3D_SPACIAL_CURVE12
#define DENOISE3D_SPACIAL_CURVE12_MASK 0x3FF00000U
#define DENOISE3D_SPACIAL_CURVE12_SHIFT 20U
/* Slice: 19 : 10       denoise3d_spacial_curve13 */
#define DENOISE3D_SPACIAL_CURVE13
#define DENOISE3D_SPACIAL_CURVE13_MASK 0x000FFC00U
#define DENOISE3D_SPACIAL_CURVE13_SHIFT 10U
/* Slice: 9 : 0 denoise3d_spacial_curve14 */
#define DENOISE3D_SPACIAL_CURVE14
#define DENOISE3D_SPACIAL_CURVE14_MASK 0x000003FFU
#define DENOISE3D_SPACIAL_CURVE14_SHIFT 0U
/* Register: ISP_DENOISE3D_CURVE_S_5    0x00003734       */
/* Slice: 19 : 10       denoise3d_spacial_curve15 */
#define DENOISE3D_SPACIAL_CURVE15
#define DENOISE3D_SPACIAL_CURVE15_MASK 0x000FFC00U
#define DENOISE3D_SPACIAL_CURVE15_SHIFT 10U
/* Slice: 9 : 0 denoise3d_spacial_curve16 */
#define DENOISE3D_SPACIAL_CURVE16
#define DENOISE3D_SPACIAL_CURVE16_MASK 0x000003FFU
#define DENOISE3D_SPACIAL_CURVE16_SHIFT 0U
/* Register: ISP_DENOISE3D_CURVE_T_0    0x00003738       */
/* Slice: 29 : 20       denoise3d_temperal_curve0 */
#define DENOISE3D_TEMPERAL_CURVE0
#define DENOISE3D_TEMPERAL_CURVE0_MASK 0x3FF00000U
#define DENOISE3D_TEMPERAL_CURVE0_SHIFT 20U
/* Slice: 19 : 10       denoise3d_temperal_curve1 */
#define DENOISE3D_TEMPERAL_CURVE1
#define DENOISE3D_TEMPERAL_CURVE1_MASK 0x000FFC00U
#define DENOISE3D_TEMPERAL_CURVE1_SHIFT 10U
/* Slice: 9 : 0 denoise3d_temperal_curve2 */
#define DENOISE3D_TEMPERAL_CURVE2
#define DENOISE3D_TEMPERAL_CURVE2_MASK 0x000003FFU
#define DENOISE3D_TEMPERAL_CURVE2_SHIFT 0U
/* Register: ISP_DENOISE3D_CURVE_T_1    0x0000373C       */
/* Slice: 29 : 20       denoise3d_temperal_curve3 */
#define DENOISE3D_TEMPERAL_CURVE3
#define DENOISE3D_TEMPERAL_CURVE3_MASK 0x3FF00000U
#define DENOISE3D_TEMPERAL_CURVE3_SHIFT 20U
/* Slice: 19 : 10       denoise3d_temperal_curve4 */
#define DENOISE3D_TEMPERAL_CURVE4
#define DENOISE3D_TEMPERAL_CURVE4_MASK 0x000FFC00U
#define DENOISE3D_TEMPERAL_CURVE4_SHIFT 10U
/* Slice: 9 : 0 denoise3d_temperal_curve5 */
#define DENOISE3D_TEMPERAL_CURVE5
#define DENOISE3D_TEMPERAL_CURVE5_MASK 0x000003FFU
#define DENOISE3D_TEMPERAL_CURVE5_SHIFT 0U
/* Register: ISP_DENOISE3D_CURVE_T_2    0x00003740       */
/* Slice: 29 : 20       denoise3d_temperal_curve6 */
#define DENOISE3D_TEMPERAL_CURVE6
#define DENOISE3D_TEMPERAL_CURVE6_MASK 0x3FF00000U
#define DENOISE3D_TEMPERAL_CURVE6_SHIFT 20U
/* Slice: 19 : 10       denoise3d_temperal_curve7 */
#define DENOISE3D_TEMPERAL_CURVE7
#define DENOISE3D_TEMPERAL_CURVE7_MASK 0x000FFC00U
#define DENOISE3D_TEMPERAL_CURVE7_SHIFT 10U
/* Slice: 9 : 0 denoise3d_temperal_curve8 */
#define DENOISE3D_TEMPERAL_CURVE8
#define DENOISE3D_TEMPERAL_CURVE8_MASK 0x000003FFU
#define DENOISE3D_TEMPERAL_CURVE8_SHIFT 0U
/* Register: ISP_DENOISE3D_CURVE_T_3    0x00003744       */
/* Slice: 29 : 20       denoise3d_temperal_curve9 */
#define DENOISE3D_TEMPERAL_CURVE9
#define DENOISE3D_TEMPERAL_CURVE9_MASK 0x3FF00000U
#define DENOISE3D_TEMPERAL_CURVE9_SHIFT 20U
/* Slice: 19 : 10       denoise3d_temperal_curve10 */
#define DENOISE3D_TEMPERAL_CURVE10
#define DENOISE3D_TEMPERAL_CURVE10_MASK 0x000FFC00U
#define DENOISE3D_TEMPERAL_CURVE10_SHIFT 10U
/* Slice: 9 : 0 denoise3d_temperal_curve11 */
#define DENOISE3D_TEMPERAL_CURVE11
#define DENOISE3D_TEMPERAL_CURVE11_MASK 0x000003FFU
#define DENOISE3D_TEMPERAL_CURVE11_SHIFT 0U
/* Register: ISP_DENOISE3D_CURVE_T_4    0x00003748       */
/* Slice: 29 : 20       denoise3d_temperal_curve12 */
#define DENOISE3D_TEMPERAL_CURVE12
#define DENOISE3D_TEMPERAL_CURVE12_MASK 0x3FF00000U
#define DENOISE3D_TEMPERAL_CURVE12_SHIFT 20U
/* Slice: 19 : 10       denoise3d_temperal_curve13 */
#define DENOISE3D_TEMPERAL_CURVE13
#define DENOISE3D_TEMPERAL_CURVE13_MASK 0x000FFC00U
#define DENOISE3D_TEMPERAL_CURVE13_SHIFT 10U
/* Slice: 9 : 0 denoise3d_temperal_curve14 */
#define DENOISE3D_TEMPERAL_CURVE14
#define DENOISE3D_TEMPERAL_CURVE14_MASK 0x000003FFU
#define DENOISE3D_TEMPERAL_CURVE14_SHIFT 0U
/* Register: ISP_DENOISE3D_CURVE_T_5    0x0000374C       */
/* Slice: 19 : 10       denoise3d_temperal_curve15 */
#define DENOISE3D_TEMPERAL_CURVE15
#define DENOISE3D_TEMPERAL_CURVE15_MASK 0x000FFC00U
#define DENOISE3D_TEMPERAL_CURVE15_SHIFT 10U
/* Slice: 9 : 0 denoise3d_temperal_curve16 */
#define DENOISE3D_TEMPERAL_CURVE16
#define DENOISE3D_TEMPERAL_CURVE16_MASK 0x000003FFU
#define DENOISE3D_TEMPERAL_CURVE16_SHIFT 0U
/* Register: ISP_DENOISE3D_AVERAGE      0x00003750       */
/* Slice: 31 : 0        denoise3d_frame_average */
#define DENOISE3D_FRAME_AVERAGE
#define DENOISE3D_FRAME_AVERAGE_MASK 0xFFFFFFFFU
#define DENOISE3D_FRAME_AVERAGE_SHIFT 0U
/* Register: ISP_DENOISE3D_STRENGTH_SHD 0x00003754       */
/* Slice: 29 : 19       denoise3d_update_temperal_shd */
#define DENOISE3D_UPDATE_TEMPERAL_SHD
#define DENOISE3D_UPDATE_TEMPERAL_SHD_MASK 0x3FF80000U
#define DENOISE3D_UPDATE_TEMPERAL_SHD_SHIFT 19U
/* Slice: 18 : 8        denoise3d_update_spacial_shd */
#define DENOISE3D_UPDATE_SPACIAL_SHD
#define DENOISE3D_UPDATE_SPACIAL_SHD_MASK 0x0007FF00U
#define DENOISE3D_UPDATE_SPACIAL_SHD_SHIFT 8U
/* Slice: 7 : 0 denoise3d_strength_shd */
#define DENOISE3D_STRENGTH_SHD
#define DENOISE3D_STRENGTH_SHD_MASK 0x0000000FU
#define DENOISE3D_STRENGTH_SHD_SHIFT 0U
/* Register: ISP_DENOISE3D_EDGE_H_SHD   0x00003758       */
/* Slice: 27 : 20       denoise3d_strength_curve_spacial_shd */
#define DENOISE3D_STRENGTH_CURVE_SPACIAL_SHD
#define DENOISE3D_STRENGTH_CURVE_SPACIAL_SHD_MASK 0x0FF00000U
#define DENOISE3D_STRENGTH_CURVE_SPACIAL_SHD_SHIFT 20U
/* Slice: 19 : 0        denoise3d_thr_edge_h_inv_shd */
#define DENOISE3D_THR_EDGE_H_INV_SHD
#define DENOISE3D_THR_EDGE_H_INV_SHD_MASK 0x000FFFFFU
#define DENOISE3D_THR_EDGE_H_INV_SHD_SHIFT 0U
/* Register: ISP_DENOISE3D_EDGE_V_SHD   0x0000375C       */
/* Slice: 27 : 20       denoise3d_strength_curve_temperal_shd */
#define DENOISE3D_STRENGTH_CURVE_TEMPERAL_SHD
#define DENOISE3D_STRENGTH_CURVE_TEMPERAL_SHD_MASK 0x0FF00000U
#define DENOISE3D_STRENGTH_CURVE_TEMPERAL_SHD_SHIFT 20U
/* Slice: 19 : 0        denoise3d_thr_edge_v_inv_shd */
#define DENOISE3D_THR_EDGE_V_INV_SHD
#define DENOISE3D_THR_EDGE_V_INV_SHD_MASK 0x000FFFFFU
#define DENOISE3D_THR_EDGE_V_INV_SHD_SHIFT 0U
/* Register: ISP_DENOISE3D_RANGE_S_SHD  0x00003760       */
/* Slice: 19 : 0        denoise3d_range_s_inv_shd */
#define DENOISE3D_RANGE_S_INV_SHD
#define DENOISE3D_RANGE_S_INV_SHD_MASK 0x000FFFFFU
#define DENOISE3D_RANGE_S_INV_SHD_SHIFT 0U
/* Register: ISP_DENOISE3D_RANGE_T_SHD  0x00003764       */
/* Slice: 29 : 25       denoise3d_range_t_h_shd */
#define DENOISE3D_RANGE_T_H_SHD
#define DENOISE3D_RANGE_T_H_SHD_MASK 0x3E000000U
#define DENOISE3D_RANGE_T_H_SHD_SHIFT 25U
/* Slice: 24 : 20       denoise3d_range_t_v_shd */
#define DENOISE3D_RANGE_T_V_SHD
#define DENOISE3D_RANGE_T_V_SHD_MASK 0x01F00000U
#define DENOISE3D_RANGE_T_V_SHD_SHIFT 20U
/* Slice: 19 : 0        denoise3d_range_t_inv_shd */
#define DENOISE3D_RANGE_T_INV_SHD
#define DENOISE3D_RANGE_T_INV_SHD_MASK 0x000FFFFFU
#define DENOISE3D_RANGE_T_INV_SHD_SHIFT 0U
/* Register: ISP_DENOISE3D_MOTION_SHD   0x00003768       */
/* Slice: 24 : 20       denoise3d_range_d_shd */
#define DENOISE3D_RANGE_D_SHD
#define DENOISE3D_RANGE_D_SHD_MASK 0x01F00000U
#define DENOISE3D_RANGE_D_SHD_SHIFT 20U
/* Slice: 19 : 0        denoise3d_motion_inv_shd */
#define DENOISE3D_MOTION_INV_SHD
#define DENOISE3D_MOTION_INV_SHD_MASK 0x000FFFFFU
#define DENOISE3D_MOTION_INV_SHD_SHIFT 0U
/* Register: ISP_DENOISE3D_DELTA_INV_SHD        0x0000376C       */
/* Slice: 29 : 20       denoise3d_delta_h_inv_shd */
#define DENOISE3D_DELTA_H_INV_SHD
#define DENOISE3D_DELTA_H_INV_SHD_MASK 0x3FF00000U
#define DENOISE3D_DELTA_H_INV_SHD_SHIFT 20U
/* Slice: 19 : 10       denoise3d_delta_v_inv_shd */
#define DENOISE3D_DELTA_V_INV_SHD
#define DENOISE3D_DELTA_V_INV_SHD_MASK 0x000FFFFFU
#define DENOISE3D_DELTA_V_INV_SHD_SHIFT 10U
/* Slice: 9 : 0 denoise3d_delta_t_inv_shd */
#define DENOISE3D_DELTA_T_INV_SHD
#define DENOISE3D_DELTA_T_INV_SHD_MASK 0x000003FFU
#define DENOISE3D_DELTA_T_INV_SHD_SHIFT 0U
/* Register: ISP_DENOISE3D_DUMMY_HBLANK 0x00003770       */
/* Slice: 14 : 0        denoise3d_H_Blank */
#define DENOISE3D_H_BLANK
#define DENOISE3D_H_BLANK_MASK 0x0000FFFFU
#define DENOISE3D_H_BLANK_SHIFT 0U
/* Register: ISP_DENOISE3D_CTRL         0x00003700 */
/* Slice: 5:5 denoise3d_soft_reset */
#define DENOISE3D_SOFT_RESET
#define DENOISE3D_SOFT_RESET_MASK  0x00000020U
#define DENOISE3D_SOFT_RESET_SHIFT 5U
/* Slice: 4:4 denoise3d_horizontal_en */
#define DENOISE3D_HORIZONTAL_EN
#define DENOISE3D_HORIZONTAL_EN_MASK 0x00000010U
#define DENOISE3D_HORIZONTAL_EN_SHIFT 4U
/* Slice: 3:3 denoise3d_vertical_en */
#define DENOISE3D_VERTICAL_EN
#define DENOISE3D_VERTICAL_EN_MASK 0x00000008U
#define DENOISE3D_VERTICAL_EN_SHIFT 3U
/* Slice: 2:2 denoise3d_temperal_en */
#define DENOISE3D_TEMPERAL_EN
#define DENOISE3D_TEMPERAL_EN_MASK 0x00000004U
#define DENOISE3D_TEMPERAL_EN_SHIFT 2U
/* Slice: 1:1 denoise3d_dilate_en */
#define DENOISE3D_DILATE_EN
#define DENOISE3D_DILATE_EN_MASK 0x00000002U
#define DENOISE3D_DILATE_EN_SHIFT 1U
/* Slice: 0:0 denoise3d_enable */
#define DENOISE3D_ENABLE
#define DENOISE3D_ENABLE_MASK 0x00000001U
#define DENOISE3D_ENABLE_SHIFT 0U
/* Register: ISP_DENOISE3D_STRENGTH 0x00003704 */
/* Slice: 29:19 denoise3d_update_temperal */
#define DENOISE3D_UPDATE_TEMPERAL
#define DENOISE3D_UPDATE_TEMPERAL_MASK  0x3FF80000U
#define DENOISE3D_UPDATE_TEMPERAL_SHIFT 19U
/* Slice: 18 : 8 denoise3d_update_spacial */
#define DENOISE3D_UPDATE_SPACIAL
#define DENOISE3D_UPDATE_SPACIAL_MASK 0x0007FF00U
#define DENOISE3D_UPDATE_SPACIAL_SHIFT 8U
/* Slice: 7 : 0 denoise3d_strength */
#define DENOISE3D_STRENGTH
#define DENOISE3D_STRENGTH_MASK 0x000000FFU
#define DENOISE3D_STRENGTH_SHIFT 0U
/* Register: ISP_DENOISE3D_EDGE_H   0x00003708 */
/* Slice: 27:20 denoise3d_strength_curve_spacial */
#define DENOISE3D_STRENGTH_CURVE_SPACIAL
#define DENOISE3D_STRENGTH_CURVE_SPACIAL_MASK 0x0FF00000U
#define DENOISE3D_STRENGTH_CURVE_SPACIAL_SHIFT 20U
/* Slice: 19 : 0    denoise3d_thr_edge_h_inv */
#define DENOISE3D_THR_EDGE_H_INV
#define DENOISE3D_THR_EDGE_H_INV_MASK 0x000FFFFFU
#define DENOISE3D_THR_EDGE_H_INV_SHIFT 0U
/* Register: ISP_DENOISE3D_EDGE_V   0x0000370C */
/* Slice: 27:20 denoise3d_strength_curve_temperal */
#define DENOISE3D_STRENGTH_CURVE_TEMPERAL
#define DENOISE3D_STRENGTH_CURVE_TEMPERAL_MASK 0x0FF00000U
#define DENOISE3D_STRENGTH_CURVE_TEMPERAL_SHIFT 20U
/* Slice: 19 : 0    denoise3d_thr_edge_v_inv */
#define DENOISE3D_THR_EDGE_V_INV
#define DENOISE3D_THR_EDGE_V_INV_MASK 0x000FFFFFU
#define DENOISE3D_THR_EDGE_V_INV_SHIFT 0U
/* Register: ISP_DENOISE3D_RANGE_S  0x00003710 */
/* Slice: 19:0  denoise3d_range_s_inv */
#define DENOISE3D_RANGE_S_INV
#define DENOISE3D_RANGE_S_INV_MASK 0x000FFFFFU
#define DENOISE3D_RANGE_S_INV_SHIFT 0U
/* Register: ISP_DENOISE3D_RANGE_T  0x00003714 */
/* Slice: 29:25 denoise3d_range_t_h */
#define DENOISE3D_RANGE_T_H
#define DENOISE3D_RANGE_T_H_MASK 0x3E000000U
#define DENOISE3D_RANGE_T_H_SHIFT 25U
/* Slice: 24:20 denoise3d_range_t_v */
#define DENOISE3D_RANGE_T_V
#define DENOISE3D_RANGE_T_V_MASK 0x01F00000U
#define DENOISE3D_RANGE_T_V_SHIFT 20U
/* Slice: 19 : 0    denoise3d_range_t_inv */
#define DENOISE3D_RANGE_T_INV
#define DENOISE3D_RANGE_T_INV_MASK 0x000FFFFFU
#define DENOISE3D_RANGE_T_INV_SHIFT 0U
/* Register: ISP_DENOISE3D_MOTION   0x00003718 */
/* Slice: 24:20 denoise3d_range_d */
#define DENOISE3D_RANGE_D
#define DENOISE3D_RANGE_D_MASK 0x01F00000U
#define DENOISE3D_RANGE_D_SHIFT 20U
/* Slice: 19 : 0    denoise3d_motion_inv */
#define DENOISE3D_MOTION_INV
#define DENOISE3D_MOTION_INV_MASK 0x000FFFFFU
#define DENOISE3D_MOTION_INV_SHIFT 0U
/* Register: ISP_DENOISE3D_DELTA_INV    0x0000371C */
/* Slice: 29:20 denoise3d_delta_h_inv */
#define DENOISE3D_DELTA_H_INV
#define DENOISE3D_DELTA_H_INV_MASK 0x3FF00000U
#define DENOISE3D_DELTA_H_INV_SHIFT 20U
/* Slice: 19 : 10   denoise3d_delta_v_inv */
#define DENOISE3D_DELTA_V_INV
#define DENOISE3D_DELTA_V_INV_MASK 0x000FFC00U
#define DENOISE3D_DELTA_V_INV_SHIFT 10U
/* Slice: 9 : 0 denoise3d_delta_t_inv */
#define DENOISE3D_DELTA_T_INV
#define DENOISE3D_DELTA_T_INV_MASK 0x000003FFU
#define DENOISE3D_DELTA_T_INV_SHIFT 0U
/* Register: ISP_DENOISE3D_CURVE_S_0    0x00003720 */
/* Slice: 29:20 denoise3d_spacial_curve0 */
#define DENOISE3D_SPACIAL_CURVE0
#define DENOISE3D_SPACIAL_CURVE0_MASK 0x3FF00000U
#define DENOISE3D_SPACIAL_CURVE0_SHIFT 20U
/* Slice: 19 : 10   denoise3d_spacial_curve1 */
#define DENOISE3D_SPACIAL_CURVE1
#define DENOISE3D_SPACIAL_CURVE1_MASK 0x000FFC00U
#define DENOISE3D_SPACIAL_CURVE1_SHIFT 10U
/* Slice: 9 : 0 denoise3d_spacial_curve2 */
#define DENOISE3D_SPACIAL_CURVE2
#define DENOISE3D_SPACIAL_CURVE2_MASK 0x000003FFU
#define DENOISE3D_SPACIAL_CURVE2_SHIFT 0U
/* Register: ISP_DENOISE3D_CURVE_T_0    0x00003738 */
/* Slice: 29 : 20   denoise3d_temperal_curve0 */
#define DENOISE3D_TEMPERAL_CURVE0
#define DENOISE3D_TEMPERAL_CURVE0_MASK 0x3FF00000U
#define DENOISE3D_TEMPERAL_CURVE0_SHIFT 20U
/* Slice: 19 : 10   denoise3d_temperal_curve1 */
#define DENOISE3D_TEMPERAL_CURVE1
#define DENOISE3D_TEMPERAL_CURVE1_MASK 0x000FFC00U
#define DENOISE3D_TEMPERAL_CURVE1_SHIFT 10U
/* Slice: 9 : 0 denoise3d_temperal_curve2 */
#define DENOISE3D_TEMPERAL_CURVE2
#define DENOISE3D_TEMPERAL_CURVE2_MASK 0x000003FFU
#define DENOISE3D_TEMPERAL_CURVE2_SHIFT 0U
/* Register: ISP_DENOISE3D_AVERAGE  0x00003750 */
/* Slice: 31 : 0    denoise3d_frame_average */
#define DENOISE3D_FRAME_AVERAGE
#define DENOISE3D_FRAME_AVERAGE_MASK 0xFFFFFFFFU
#define DENOISE3D_FRAME_AVERAGE_SHIFT 0U
/* Register: ISP_DENOISE3D_STRENGTH_SHD 0x00003754 */
/* Slice: 29 : 19   denoise3d_update_temperal_shd */
#define DENOISE3D_UPDATE_TEMPERAL_SHD
#define DENOISE3D_UPDATE_TEMPERAL_SHD_MASK 0x3FF80000U
#define DENOISE3D_UPDATE_TEMPERAL_SHD_SHIFT 19U
/* Slice: 18 : 8    denoise3d_update_spacial_shd */
#define DENOISE3D_UPDATE_SPACIAL_SHD
#define DENOISE3D_UPDATE_SPACIAL_SHD_MASK 0x0007FF00U
#define DENOISE3D_UPDATE_SPACIAL_SHD_SHIFT 8U
/* Slice: 7 : 0 denoise3d_strength_shd */
#define DENOISE3D_STRENGTH_SHD
#define DENOISE3D_STRENGTH_SHD_MASK 0x0000000FU
#define DENOISE3D_STRENGTH_SHD_SHIFT 0U
/* Register: ISP_DENOISE3D_EDGE_H_SHD   0x00003758 */
/* Slice: 27 : 20   denoise3d_strength_curve_spacial_shd */
#define DENOISE3D_STRENGTH_CURVE_SPACIAL_SHD
#define DENOISE3D_STRENGTH_CURVE_SPACIAL_SHD_MASK 0x0FF00000U
#define DENOISE3D_STRENGTH_CURVE_SPACIAL_SHD_SHIFT 20U
/* Slice: 19 : 0    denoise3d_thr_edge_h_inv_shd */
#define DENOISE3D_THR_EDGE_H_INV_SHD
#define DENOISE3D_THR_EDGE_H_INV_SHD_MASK 0x000FFFFFU
#define DENOISE3D_THR_EDGE_H_INV_SHD_SHIFT 0U
/* Register: ISP_DENOISE3D_EDGE_V_SHD   0x0000375C */
/* Slice: 27 : 20   denoise3d_strength_curve_temperal_shd */
#define DENOISE3D_STRENGTH_CURVE_TEMPERAL_SHD
#define DENOISE3D_STRENGTH_CURVE_TEMPERAL_SHD_MASK 0x0FF00000U
#define DENOISE3D_STRENGTH_CURVE_TEMPERAL_SHD_SHIFT 20U
/* Slice: 19 : 0    denoise3d_thr_edge_v_inv_shd */
#define DENOISE3D_THR_EDGE_V_INV_SHD
#define DENOISE3D_THR_EDGE_V_INV_SHD_MASK 0x000FFFFFU
#define DENOISE3D_THR_EDGE_V_INV_SHD_SHIFT 0U
/* Register: ISP_DENOISE3D_RANGE_S_SHD  0x00003760 */
/* Slice: 19 : 0    denoise3d_range_s_inv_shd */
#define DENOISE3D_RANGE_S_INV_SHD
#define DENOISE3D_RANGE_S_INV_SHD_MASK 0x000FFFFFU
#define DENOISE3D_RANGE_S_INV_SHD_SHIFT 0U
/* Register: ISP_DENOISE3D_RANGE_T_SHD  0x00003764 */
/* Slice: 29 : 25   denoise3d_range_t_h_shd */
#define DENOISE3D_RANGE_T_H_SHD
#define DENOISE3D_RANGE_T_H_SHD_MASK 0x3E000000U
#define DENOISE3D_RANGE_T_H_SHD_SHIFT 25U
/* Slice: 24 : 20   denoise3d_range_t_v_shd */
#define DENOISE3D_RANGE_T_V_SHD
#define DENOISE3D_RANGE_T_V_SHD_MASK 0x01F00000U
#define DENOISE3D_RANGE_T_V_SHD_SHIFT 20U
/* Slice: 19 : 0    denoise3d_range_t_inv_shd */
#define DENOISE3D_RANGE_T_INV_SHD
#define DENOISE3D_RANGE_T_INV_SHD_MASK 0x000FFFFFU
#define DENOISE3D_RANGE_T_INV_SHD_SHIFT 0U
/* Register: ISP_DENOISE3D_MOTION_SHD   0x00003768 */
/* Slice: 24 : 20   denoise3d_range_d_shd */
#define DENOISE3D_RANGE_D_SHD
#define DENOISE3D_RANGE_D_SHD_MASK 0x01F00000U
#define DENOISE3D_RANGE_D_SHD_SHIFT 20U
/* Slice: 19 : 0    denoise3d_motion_inv_shd */
#define DENOISE3D_MOTION_INV_SHD
#define DENOISE3D_MOTION_INV_SHD_MASK 0x000FFFFFU
#define DENOISE3D_MOTION_INV_SHD_SHIFT 0U
/* Register: ISP_DENOISE3D_DELTA_INV_SHD    0x0000376C */
/* Slice: 29 : 20   denoise3d_delta_h_inv_shd */
#define DENOISE3D_DELTA_H_INV_SHD
#define DENOISE3D_DELTA_H_INV_SHD_MASK 0x3FF00000U
#define DENOISE3D_DELTA_H_INV_SHD_SHIFT 20U
/* Slice: 19 : 10   denoise3d_delta_v_inv_shd */
#define DENOISE3D_DELTA_V_INV_SHD
#define DENOISE3D_DELTA_V_INV_SHD_MASK 0x000FFFFFU
#define DENOISE3D_DELTA_V_INV_SHD_SHIFT 10U
/* Slice: 9 : 0 denoise3d_delta_t_inv_shd */
#define DENOISE3D_DELTA_T_INV_SHD
#define DENOISE3D_DELTA_T_INV_SHD_MASK 0x000003FFU
#define DENOISE3D_DELTA_T_INV_SHD_SHIFT 0U
/* Register: ISP_DENOISE3D_DUMMY_HBLANK 0x00003770 */
/* Slice: 14 : 0    denoise3d_H_Blank */
#define DENOISE3D_H_BLANK
#define DENOISE3D_H_BLANK_MASK 0x0000FFFFU
#define DENOISE3D_H_BLANK_SHIFT 0U

/* Register: ISP_DENOISE3D_WEIGHT1 0x00003778 */
/* Slice:  denoise3d_weight_up_y0 */
#define DENOISE3D_WEIGHT_UP_Y0
#define DENOISE3D_WEIGHT_UP_Y0_MASK 0x00F00000U
#define DENOISE3D_WEIGHT_UP_Y0_SHIFT 20U

#define DENOISE3D_WEIGHT_UP_Y1
#define DENOISE3D_WEIGHT_UP_Y1_MASK 0x000F0000U
#define DENOISE3D_WEIGHT_UP_Y1_SHIFT 16U

#define DENOISE3D_WEIGHT
#define DENOISE3D_WEIGHT_MASK 0x0000000FU
#define DENOISE3D_WEIGHT_SHIFT 0U


/*! Register: mi_sp2_fmt (0x000014e8)*/
/*! Slice: sp2_rd_yuv_nvy:*/
#define SP2_RD_YUV_NVY
#define SP2_RD_YUV_NVY_MASK 0x0C000000U
#define SP2_RD_YUV_NVY_SHIFT 26U
/*! Slice: sp2_rd_yuv_nv21:*/
#define SP2_RD_YUV_NV21
#define SP2_RD_YUV_NV21_MASK 0x02000000U
#define SP2_RD_YUV_NV21_SHIFT 25U
/*! Slice: sp2_rd_raw_aligned:*/
#define SP2_RD_RAW_ALIGNED
#define SP2_RD_RAW_ALIGNED_MASK 0x01800000U
#define SP2_RD_RAW_ALIGNED_SHIFT 23U
/*! Slice: sp2_rd_yuv_aligned:*/
#define SP2_RD_YUV_ALIGNED
#define SP2_RD_YUV_ALIGNED_MASK 0x00400000U
#define SP2_RD_YUV_ALIGNED_SHIFT 22U
/*! Slice: sp2_rd_raw_bit:*/
#define SP2_RD_RAW_BIT
#define SP2_RD_RAW_BIT_MASK 0x00380000U
#define SP2_RD_RAW_BIT_SHIFT 19U
/*! Slice: sp2_rd_yuv_str:*/
#define SP2_RD_YUV_STR
#define SP2_RD_YUV_STR_MASK 0x00060000U
#define SP2_RD_YUV_STR_SHIFT 17U
/*! Slice: sp2_rd_yuv_fmt:*/
#define SP2_RD_YUV_FMT
#define SP2_RD_YUV_FMT_MASK 0x00018000U
#define SP2_RD_YUV_FMT_SHIFT 15U
/*! Slice: sp2_rd_yuv_bit:*/
#define SP2_RD_YUV_BIT
#define SP2_RD_YUV_BIT_MASK 0x00004000U
#define SP2_RD_YUV_BIT_SHIFT 14U
/*! Slice: sp2_wr_yuv_nvy:*/
#define SP2_WR_YUV_NVY
#define SP2_WR_YUV_NVY_MASK 0x00003000U
#define SP2_WR_YUV_NVY_SHIFT 12U
/*! Slice: sp2_wr_yuv_nv21:*/
#define SP2_WR_YUV_NV21
#define SP2_WR_YUV_NV21_MASK 0x00000800U
#define SP2_WR_YUV_NV21_SHIFT 11U
/*! Slice: sp2_wr_raw_aligned:*/
#define SP2_WR_RAW_ALIGNED
#define SP2_WR_RAW_ALIGNED_MASK 0x00000600U
#define SP2_WR_RAW_ALIGNED_SHIFT 9U
/*! Slice: sp2_wr_yuv_aligned:*/
#define SP2_WR_YUV_ALIGNED
#define SP2_WR_YUV_ALIGNED_MASK 0x00000100U
#define SP2_WR_YUV_ALIGNED_SHIFT 8U
/*! Slice: sp2_wr_raw_bit:*/
#define SP2_WR_RAW_BIT
#define SP2_WR_RAW_BIT_MASK 0x000000E0U
#define SP2_WR_RAW_BIT_SHIFT 5U
/*! Slice: sp2_wr_yuv_str:*/
#define SP2_WR_YUV_STR
#define SP2_WR_YUV_STR_MASK 0x00000018U
#define SP2_WR_YUV_STR_SHIFT 3U
/*! Slice: sp2_wr_yuv_fmt:*/
#define SP2_WR_YUV_FMT
#define SP2_WR_YUV_FMT_MASK 0x00000006U
#define SP2_WR_YUV_FMT_SHIFT 1U
/*! Slice: sp2_wr_yuv_bit:*/
#define SP2_WR_YUV_BIT
#define SP2_WR_YUV_BIT_MASK 0x00000001U
#define SP2_WR_YUV_BIT_SHIFT 0U

/*! Register: mi_sp2_bus_id (0x000014f0)*/
/*! Slice: sp2_bus_sw_en:*/
#define SP2_BUS_SW_EN
#define SP2_BUS_SW_EN_MASK 0x08000000U
#define SP2_BUS_SW_EN_SHIFT 27U
/*! Slice: sp2_rd_issue_cap:*/
#define SP2_RD_ISSUE_CAP
#define SP2_RD_ISSUE_CAP_MASK 0x06000000U
#define SP2_RD_ISSUE_CAP_SHIFT 25U
/*! Slice: sp2_wr_issue_cap:*/
#define SP2_WR_ISSUE_CAP
#define SP2_WR_ISSUE_CAP_MASK 0x01800000U
#define SP2_WR_ISSUE_CAP_SHIFT 23U
/*! Slice: sp2_rd_burst_len:*/
#define SP2_RD_BURST_LEN
#define SP2_RD_BURST_LEN_MASK 0x00600000U
#define SP2_RD_BURST_LEN_SHIFT 21U
/*! Slice: sp2_wr_burst_len:*/
#define SP2_WR_BURST_LEN
#define SP2_WR_BURST_LEN_MASK 0x00180000U
#define SP2_WR_BURST_LEN_SHIFT 19U
/*! Slice: sp2_rd_id_en:*/
#define SP2_RD_ID_EN
#define SP2_RD_ID_EN_MASK 0x00040000U
#define SP2_RD_ID_EN_SHIFT 18U
/*! Slice: sp2_rd_id_cfg:*/
#define SP2_RD_ID_CFG
#define SP2_RD_ID_CFG_MASK 0x0003FC00U
#define SP2_RD_ID_CFG_SHIFT 10U
/*! Slice: sp2_wr_id_en:*/
#define SP2_WR_ID_EN
#define SP2_WR_ID_EN_MASK 0x00000100U
#define SP2_WR_ID_EN_SHIFT 8U
/*! Slice: sp2_wr_id_cfg:*/
#define SP2_WR_ID_CFG
#define SP2_WR_ID_CFG_MASK 0x000000FFU
#define SP2_WR_ID_CFG_SHIFT 0U

/*! Register: mi_ctrl (0x00001300)*/
/*! Slice: mcm_raw_rdma_start_con:*/
#define MCM_RAW_RDMA_START_CON
#define MCM_RAW_RDMA_START_CON_MASK 0x00010000U
#define MCM_RAW_RDMA_START_CON_SHIFT 16U
/*! Slice: mcm_raw_rdma_start:*/
#define MCM_RAW_RDMA_START
#define MCM_RAW_RDMA_START_MASK 0x00008000U
#define MCM_RAW_RDMA_START_SHIFT 15U
/*! Slice: mcm_raw_rdma_path_enable:*/
#define MCM_RAW_RDMA_PATH_ENABLE
#define MCM_RAW_RDMA_PATH_ENABLE_MASK 0x00004000U
#define MCM_RAW_RDMA_PATH_ENABLE_SHIFT 14U
/*! Slice: sp2_raw_rdma_start_con:*/
#define SP2_RAW_RDMA_START_CON
#define SP2_RAW_RDMA_START_CON_MASK 0x00002000U
#define SP2_RAW_RDMA_START_CON_SHIFT 13U
/*! Slice: sp2_raw_rdma_start:*/
#define SP2_RAW_RDMA_START
#define SP2_RAW_RDMA_START_MASK 0x00001000U
#define SP2_RAW_RDMA_START_SHIFT 12U
/*! Slice: sp2_raw_rdma_path_enable:*/
#define SP2_RAW_RDMA_PATH_ENABLE
#define SP2_RAW_RDMA_PATH_ENABLE_MASK 0x00000800U
#define SP2_RAW_RDMA_PATH_ENABLE_SHIFT 11U
/*! Slice: sp2_ycbcr_rdma_start_con:*/
#define SP2_YCBCR_RDMA_START_CON
#define SP2_YCBCR_RDMA_START_CON_MASK 0x00000400U
#define SP2_YCBCR_RDMA_START_CON_SHIFT 10U
/*! Slice: sp2_ycbcr_rdma_start:*/
#define SP2_YCBCR_RDMA_START
#define SP2_YCBCR_RDMA_START_MASK 0x00000200U
#define SP2_YCBCR_RDMA_START_SHIFT 9U
/*! Slice: sp2_ycbcr_rdma_path_enable:*/
#define SP2_YCBCR_RDMA_PATH_ENABLE
#define SP2_YCBCR_RDMA_PATH_ENABLE_MASK 0x00000100U
#define SP2_YCBCR_RDMA_PATH_ENABLE_SHIFT 8U
/*! Slice: mcm_raw1_path_enable:*/
#define MCM_RAW1_PATH_ENABLE
#define MCM_RAW1_PATH_ENABLE_MASK 0x00000080U
#define MCM_RAW1_PATH_ENABLE_SHIFT 7U
/*! Slice: mcm_raw0_path_enable:*/
#define MCM_RAW0_PATH_ENABLE
#define MCM_RAW0_PATH_ENABLE_MASK 0x00000040U
#define MCM_RAW0_PATH_ENABLE_SHIFT 6U
/*! Slice: sp2_raw_path_enable:*/
#define SP2_RAW_PATH_ENABLE
#define SP2_RAW_PATH_ENABLE_MASK 0x00000020U
#define SP2_RAW_PATH_ENABLE_SHIFT 5U
/*! Slice: sp2_ycbcr_path_enable:*/
#define SP2_YCBCR_PATH_ENABLE
#define SP2_YCBCR_PATH_ENABLE_MASK 0x00000010U
#define SP2_YCBCR_PATH_ENABLE_SHIFT 4U
/*! Slice: sp1_ycbcr_path_enable:*/
#define SP1_YCBCR_PATH_ENABLE
#define SP1_YCBCR_PATH_ENABLE_MASK 0x00000008U
#define SP1_YCBCR_PATH_ENABLE_SHIFT 3U
/*! Slice: mp_jdp_path_enable:*/
#define MP_JDP_PATH_ENABLE
#define MP_JDP_PATH_ENABLE_MASK 0x00000004U
#define MP_JDP_PATH_ENABLE_SHIFT 2U
/*! Slice: mp_raw_path_enable:*/
#define MP_RAW_PATH_ENABLE
#define MP_RAW_PATH_ENABLE_MASK 0x00000002U
#define MP_RAW_PATH_ENABLE_SHIFT 1U
/*! Slice: mp_ycbcr_path_enable:*/
#define MP_YCBCR_PATH_ENABLE
#define MP_YCBCR_PATH_ENABLE_MASK 0x00000001U
#define MP_YCBCR_PATH_ENABLE_SHIFT 0U

/*! Register: mi_sp2_raw_size_init (rw) (MRV_BASE + 0x36c)*/
/*! Slice: SP2_RAW_SIZE:*/
#define SP2_RAW_SIZE
#define SP2_RAW_SIZE_MASK 0x1FFFFFF0U
#define SP2_RAW_SIZE_SHIFT 4U

/*! Register: mi_sp2_raw_offs_cnt_init (rw) (MRV_BASE + 0x370)*/
/*! Slice: SP2_RAW_OFFS_CNT:*/
#define SP2_RAW_OFFS_CNT
#define SP2_RAW_OFFS_CNT_MASK 0x1FFFFFF0U
#define SP2_RAW_OFFS_CNT_SHIFT 4U

/*! Register: mi_sp2_raw_llength (rw) (MRV_BASE + 0x374)*/
/*! Slice: SP2_RAW_LLENGTH:*/
#define SP2_RAW_LLENGTH
#define SP2_RAW_LLENGTH_MASK 0xFFFFFFFFU
#define SP2_RAW_LLENGTH_SHIFT 0U


/*! Register: mi_sp2_raw_pic_llength (rw) (MRV_BASE + 0x3c4)*/
/*! Slice: SP2_RAW_PIC LLENGTH:*/
#define SP2_RAW_PIC_LLENGTH
#define SP2_RAW_LLENGTH_MASK 0xFFFFFFFFU
#define SP2_RAW_LLENGTH_SHIFT 0U

/*! Register: mi_sp2_raw_pix_width(rw) (MRV_BASE + 0x3c0)*/
/*! Slice: SP2_RAW_PIC_WIDTH:*/
#define SP2_RAW_PIC_WIDTH
#define SP2_RAW_PIC_WIDTH_MASK 0x00007FFFU
#define SP2_RAW_PIC_WIDTH_SHIFT 0U

/*! Register: mi_sp2_raw_pic_height (rw) (MRV_BASE + 0x37c)*/
/*! Slice: SP2_RAW_PIC_HEIGHT:*/
#define SP2_RAW_PIC_HEIGHT
#define SP2_RAW_PIC_HEIGHT_MASK 0xFFFFFFFFU
#define SP2_RAW_PIC_HEIGHT_SHIFT 0U

/*! Register: mi_sp2_raw_pic_size (rw) (MRV_BASE + 0x380)*/
/*! Slice: SP2_RAW_PIC_SIZE:*/
#define SP2_RAW_PIC_SIZE
#define SP2_RAW_PIC_SIZE_MASK 0xFFFFFFFFU
#define SP2_RAW_PIC_SIZE_SHIFT 0U

/*! Register: mi_sp2_dma_raw_pic_start_ad (0x000015bc)*/
/*! Slice: sp2_dma_raw_pic_start_ad:*/
#define SP2_DMA_RAW_PIC_START_AD
#define SP2_DMA_RAW_PIC_START_AD_MASK 0xFFFFFFF0U
#define SP2_DMA_RAW_PIC_START_AD_SHIFT 4U
/*! Register: mi_sp2_dma_raw_pic_width (0x000015c0)*/
/*! Slice: sp2_dma_raw_pic_width:*/
#define SP2_DMA_RAW_PIC_WIDTH
#define SP2_DMA_RAW_PIC_WIDTH_MASK 0x00007FFFU
#define SP2_DMA_RAW_PIC_WIDTH_SHIFT 0U
/*! Register: mi_sp2_dma_raw_pic_llength (0x000015c4)*/
/*! Slice: sp2_dma_raw_pic_llength:*/
#define SP2_DMA_RAW_PIC_LLENGTH
#define SP2_DMA_RAW_PIC_LLENGTH_MASK 0x00007FFFU
#define SP2_DMA_RAW_PIC_LLENGTH_SHIFT 0U
/*! Register: mi_sp2_dma_raw_pic_size (0x000015c8)*/
/*! Slice: sp2_dma_raw_pic_size:*/
#define SP2_DMA_RAW_PIC_SIZE
#define SP2_DMA_RAW_PIC_SIZE_MASK 0x0FFFFFFFU
#define SP2_DMA_RAW_PIC_SIZE_SHIFT 0U
/*! Register: mi_sp2_dma_raw_pic_start_ad_shd (0x000015cc)*/
/*! Slice: sp2_dma_raw_pic_start_ad:*/
#define SP2_DMA_RAW_PIC_START_AD
#define SP2_DMA_RAW_PIC_START_AD_MASK 0xFFFFFFF0U
#define SP2_DMA_RAW_PIC_START_AD_SHIFT 4U

/*! Register: mi_sp2_dma_raw_pic_start_ad_shd (0x000015cc)*/
/*! Slice: sp2_dma_raw_pic_start_ad:*/
#define SP2_DMA_RAW_WIDTH_BYTES
#define SP2_DMA_RAW_WIDTH_BYTES_MASK 0x0000FFFFU
#define SP2_DMA_RAW_WIDTH_BYTES_SHIFT 0U

/*! Register: mi_sp2_ctrl (0x000014e4)*/
/*! Slice: sp2_rd_raw_cfg_update */
#define SP2_RD_RAW_CFG_UPDATE
#define SP2_RD_RAW_CFG_UPDATE_MASK 0x00000200U
#define SP2_RD_RAW_CFG_UPDATE_SHIFT 9U
/*! Slice: sp2_rd_raw_auto_update */
#define SP2_RD_RAW_AUTO_UPDATE
#define SP2_RD_RAW_AUTO_UPDATE_MASK 0x00000100U
#define SP2_RD_RAW_AUTO_UPDATE_SHIFT 8U
/*! Slice: sp2_rd_yuv_cfg_update */
#define SP2_RD_YUV_CFG_UPDATE
#define SP2_RD_YUV_CFG_UPDATE_MASK 0x00000080U
#define SP2_RD_YUV_CFG_UPDATE_SHIFT 7U
/*! Slice: sp2_rd_yuv_auto_update */
#define SP2_RD_YUV_AUTO_UPDATE
#define SP2_RD_YUV_AUTO_UPDATE_MASK 0x00000040U
#define SP2_RD_YUV_AUTO_UPDATE_SHIFT 6U
/*! Slice: sp2_init_offset_en:*/
#define SP2_INIT_OFFSET_EN
#define SP2_INIT_OFFSET_EN_MASK 0x00000020U
#define SP2_INIT_OFFSET_EN_SHIFT 5U
/*! Slice: sp2_init_base_en:*/
#define SP2_INIT_BASE_EN
#define SP2_INIT_BASE_EN_MASK 0x00000010U
#define SP2_INIT_BASE_EN_SHIFT 4U
/*! Slice: sp2_mi_cfg_upd:*/
#define SP2_MI_CFG_UPD
#define SP2_MI_CFG_UPD_MASK 0x00000008U
#define SP2_MI_CFG_UPD_SHIFT 3U
/*! Slice: sp2_mi_skip:*/
#define SP2_MI_SKIP
#define SP2_MI_SKIP_MASK 0x00000004U
#define SP2_MI_SKIP_SHIFT 2U
/*! Slice: sp2_auto_update:*/
#define SP2_AUTO_UPDATE
#define SP2_AUTO_UPDATE_MASK 0x00000002U
#define SP2_AUTO_UPDATE_SHIFT 1U
/*! Slice: sp2_pingpong_enable:*/
#define SP2_PINGPONG_ENABLE
#define SP2_PINGPONG_ENABLE_MASK 0x00000001U
#define SP2_PINGPONG_ENABLE_SHIFT 0U

#endif

