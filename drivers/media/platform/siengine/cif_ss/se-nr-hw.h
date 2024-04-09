// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021-2022 Siengine Technology, Inc.
 *
 * Sienginie Camera Interface driver
 *
 * Author: Siengine Technology, Inc.
 */

#ifndef SE_NR_HW_H_
#define SE_NR_HW_H_

#include "se-nr-reg.h"
#include <linux/types.h>
#include <asm/io.h>
//#include "se_cif_hw.h"

#ifndef ENABLE
#define ENABLE	1
#define DISABLE 0
#endif

#define	CISP_NR_ENABLE	1
#define CISP_NR_DISABLE	0
#define CISP_NR_ENABLE_OFFSET

#define CISP_NR_DENOISE3D_ENABLE	1
#define CISP_NR_DENOISE3D_DISABLE	0
#define CISP_NR_DENOISE3D_OFFSET

#define CISP_NR_INTERRUPT_ENABLE	1
#define CISP_NR_INTERRUPT_DISABLE	0
#define CISP_NR_INTERRUPT_OFFSET

#define CISP_NR_RIS_VEND_MASK		0x2
#define CISP_NR_RIS_TDNR_MASK		0x1
#define CISP_NR_MIS_VEND_MASK		0x2
#define CISP_NR_MIS_TDNR_MASK		0x1

#define TEMPORAL_MAX						1024
#define SPACIAL_MAX							1024
#define STRENGTH_MAX						128
#define STRENGTH_CURVE_TEMPORAL_MAX			128
#define STRENGTH_CURVE_SPATIAL_MAX			128

#define THR_MOTION_INV_MAX					1048575
#define DELTA_T_INV_MAX						1023

#define ISP_DENOISE3D_WRITE_REG                8

#define ISP_DENOISE3D_STRENGTH_UPDATE_STRENGTH 0
#define ISP_DENOISE3D_STRENGTH_UPDATE_SPACIAL  1
#define ISP_DENOISE3D_STRENGTH_UPDATE_TEMPERAL 2
#define ISP_DENOISE3D_STRENGTH_GET_REG         3

#define ISP_DENOISE3D_EDGE_H_UPDATE_INV        0
#define ISP_DENOISE3D_EDGE_H_UPDATE_CURVE_SPACIAL 1
#define ISP_DENOISE3D_EDGE_H_GET_REG           2

#define ISP_DENOISE3D_EDGE_V_UPDATE_INV        0
#define ISP_DENOISE3D_EDGE_V_UPDATE_CURVE_SPACIAL 1
#define ISP_DENOISE3D_EDGE_V_GET_REG           2

#define ISP_DENOISE3D_RANGE_S_UPDATE_INV        0
#define ISP_DENOISE3D_RANGE_S_GET_REG           1

#define ISP_DENOISE3D_RANGE_T_UPDATE_INV        0
#define ISP_DENOISE3D_RANGE_T_UPDATE_V          1
#define ISP_DENOISE3D_RANGE_T_UPDATE_H          2
#define ISP_DENOISE3D_RANGE_T_GET_REG           3

#define ISP_DENOISE3D_MOTION_UPDATE_INV         0
#define ISP_DENOISE3D_MOTION_UPDATE_RANGE_D     1
#define ISP_DENOISE3D_MOTION_GET_REG            2

#define ISP_DENOISE3D_DELTA_UPDATE_T_INV        0
#define ISP_DENOISE3D_DELTA_UPDATE_V_INV        1
#define ISP_DENOISE3D_DELTA_UPDATE_H_INV        2
#define ISP_DENOISE3D_DELTA_GET_REG             3

#define ISP_DENOISE3D_CURVE_UPDATE_BIT9_0       0
#define ISP_DENOISE3D_CURVE_UPDATE_BIT19_10     1
#define ISP_DENOISE3D_CURVE_UPDATE_BIT29_20     2
#define ISP_DENOISE3D_CURVE_GET_REG             3
#define ISP_DENOISE3D_CURVE_S5_T5_GET_REG       2

#define ISP_DENOISE3D_DUMMY_HBLANK_BIT15_0      0
#define ISP_DENOISE3D_DUMMY_HBLANK_GET_REG      1

typedef enum NR_BAYER_PATTERN {
	RGRG_GBGB,
	GRGR_BGBG,
	GBGB_RGRG,
	BGBG_GRGR,
	PATTERN_MAX,
} NR_BAYER_PATTTERN;

typedef enum RDMA_MODE {
	NORMAL_MODE,
	CONTINUOUS_MODE,
	RDMA_MODE_MAX,
} RDMA_MODE;
typedef enum NR_BURST_LEN {
	BURST_4,
	BURST_8,
	BURST_16,
	BURST_RESERVED,
	BURST_MAX,
} NR_BURST_LEN;

typedef enum aligned_mode {
	UNLIGNED,//00
	ALIGNED_MODE_0, //01
	ALIGNED_MODE_1, //10
	RESERVED,//11
	ALIGNED_MODE_MAX,
} ALIGNED_MODE;

typedef enum NR_RAW_BIT {
	RAW8, //000
	RAW10, //001
	RAW12, //010
	RAW14, //011
	RAW16, //100
	RAW20, //101
	RAW_BIT_MAX,
} NR_RAW_BIT;

typedef enum NR_YUV_BIT {
	YUV_RGB_8, //0
	YUV_RGB_10, //1
	YUV_BIT_MAX,
} NR_YUV_BIT;

typedef enum NR_YUV_FMT {
	YUV420, //00
	YUV422, //01
	YUV444, //10
	YUV_FMT_RESERVED, //11
	YUV_FMT_MAX,
} NR_YUV_FMT;

#define SUCCESS			0
#define PARAM_ERROR		-1
#define ERROR			-2

#define REG_GET_SLICE(reg, name) \
		(((reg) & (name##_MASK)) >> (name##_SHIFT))

#define REG_SET_SLICE(reg, name, value) \
    { \
	((reg) = (((reg) & ~(name##_MASK)) | (((value) << (name##_SHIFT)) & (name##_MASK))));\
    }

//#define NR_WRITE_DBG

#ifdef NR_WRITE_DBG
#define nr_write_dbg(val, addr) \
   do { \
	   printk("write: %p = 0x%08x\n", addr, val); \
	   writel(val, addr); \
   } while (0)

#else
#define nr_write_dbg(val, addr) writel(val, addr)

#endif

typedef struct nr_setting {
	u32 reg_addr_offset;
	u32 reg_value;
} nr_setting;

typedef struct isp_3dnr_update {
	u32 thr_edge_h_inv;
	u32 thr_edge_v_inv;
	u32 thr_motion_inv;
	u32 thr_range_s_inv;
	u32 range_t_h;
	u32 range_t_v;
	u32 range_d;
	u32 thr_range_t_inv;
	u32 thr_delta_h_inv;
	u32 thr_delta_v_inv;
	u32 thr_delta_t_inv;
} denoise3d_update;

typedef struct denoise3d_params{
	u32 enable_h;
	u32 enable_v;
	u32 enable_temperal;
	u32 enable_dilate;
	//spacial
	u32 update_spatial;
	u32 strength_curve_spatial;
	u32 thr_edge_h_inv;
	u32 thr_edge_v_inv;
	u32 thr_range_s_inv;
	//temperal
	u32 update_temperal;
	u32 strength_curve_temperal;
	u32 range_t_h;
	u32 range_t_v;
	u32 range_d;
	u32 thr_range_t_inv;
	u32 thr_delta_h_inv;
	u32 thr_delta_v_inv;
	u32 thr_delta_t_inv;
	u32 thr_motion_inv;
	u32 strength;
	u32 h_blank;
	u32 spacial_curve[17];
	u32 temperal_curve[17];
} denoise3d_params;

typedef struct se_nr_dev {
	u32 cpproc_clk_enable;
	u32 isp_clk_enable;
	void __iomem *base; //base address of NR200 register
	void __iomem *virt_rf_wr_addr;  //reference frame write virtual address
	void __iomem *virt_rf_rd_addr; //reference frame read virtual address
	//reference frame write address, if enable smmu, should be iova, otherewise,
	//it should be pa
	u32 rf_wr_addr;
	//reference frame read address, if enable smmu, should be iova, otherewise,
	//it should be pa
	u32 rf_rd_addr;
	u32 nr200_enable; //control bypass nr200 or not
	u32 enable_denoise3d;  //control enable 3d noise reduce submodule or not
	u32 init; //indicate the nr hw init or not
	u32 idx; //should be 0 - 17
	u32 out_h_size; //the image width
	u32 out_v_size; // the image height
	u32 out_size; //the outsize should be the refrence frame size
	u32 pattern; //define the bayer pattern, default is RGRG-GBGB
	u32 rdma_mode; //rdma mode, default is normal mode
	denoise3d_params params;
} se_nr_dev;

typedef struct NR200 {
	u32 INTERNAL_CLK_CTRL;
	u32 VI_IRCL;
	u32 ISP_CTRL;
	u32 ISP_OUT_H_SIZE;
	u32 ISP_OUT_V_SIZE;
	u32 MI_CTRL;
	u32 MI_QOS;
	u32 MI_SP2_CTRL;
	u32 MI_SP2_FMT;
	u32 MI_SP2_BUS_CFG;
	u32 MI_SP2_BUS_ID;
	u32 MI_SP2_RAW_BASE_AD;
	u32 MI_SP2_RAW_SIZE;
	u32 MI_SP2_RAW_OFFS_CNT;
	u32 MI_SP2_RAW_LLENGTH;
	u32 MI_SP2_RAW_PIC_WIDTH;
	u32 MI_SP2_RAW_PIC_HEIGHT;
	u32 MI_SP2_RAW_PIC_SIZE;
	u32 MI_SP2_DMA_RAW_PIC_START_AD;
	u32 MI_SP2_DMA_RAW_PIC_WIDTH;
	u32 MI_SP2_DMA_RAW_PIC_LLENGTH;
	u32 MI_SP2_DMA_RAW_PIC_SIZE;
	u32 MI_SP2_DMA_RAW_PIC_LVAL;
	u32 DENOISE3D_CTRL;
	u32 DENOISE3D_STRENGTH_REG;
	u32 DENOISE3D_EDGE_H;
	u32 DENOISE3D_EDGE_V;
	u32 DENOISE3D_RANGE_S;
	u32 DENOISE3D_RANGE_T;
	u32 DENOISE3D_MOTION;
	u32 DENOISE3D_DELTA_INV;
	u32 DENOISE3D_CURVE_S_0;
	u32 DENOISE3D_CURVE_S_1;
	u32 DENOISE3D_CURVE_S_2;
	u32 DENOISE3D_CURVE_S_3;
	u32 DENOISE3D_CURVE_S_4;
	u32 DENOISE3D_CURVE_S_5;
	u32 DENOISE3D_CURVE_T_0;
	u32 DENOISE3D_CURVE_T_1;
	u32 DENOISE3D_CURVE_T_2;
	u32 DENOISE3D_CURVE_T_3;
	u32 DENOISE3D_CURVE_T_4;
	u32 DENOISE3D_CURVE_T_5;
	u32 DENOISE3D_AVERAGE;
	u32 DENOISE3D_STRENGT_SHD;
	u32 DENOISE3D_EDGE_H_SHD;
	u32 DENOISE3D_EDGE_V_SHD;
	u32 DENOISE3D_RANGE_S_SHD;
	u32 DENOISE3D_RANGE_T_SHD;
	u32 DENOISE3D_MOTION_SHD;
	u32 DENOISE3D_DELTA_INV_SHD;
	u32 DENOISE3D_DUMMY_HBLANK;
	u32 DENOISE3D_CTRL_SHD;
	u32 ISP_IMSC_INTERRUPT;
	u32 ISP_MIS_INTERRUPT;
	u32 ISP_RIS_INTERRUPT;
	u32 ISP_ICR_INTERRUPT;
	u32 MI_IMSC_INTERRUPT;
	u32 MI_IMSC1_INTERRUPT;
	u32 MI_RIS_INTERRUPT;
	u32 MI_RIS1_INTERRUPT;
	u32 MI_MIS_INTERRUPT;
	u32 MI_MIS1_INTERRUPT;
	u32 MI_ICR_INTERRUPT;
	u32 MI_ICR1_INTERRUPT;
} NR200;

typedef struct se_nr_para {
	u32 para_type;
	u32 para_data;
} se_nr_data_t;

int cisp_nr_enable_all_interrupt(se_nr_dev *nr_dev);

int cisp_nr_disable_all_interrupt(se_nr_dev *nr_dev);

int cisp_nr_process_irq(se_nr_dev *nr_dev);

int cisp_nr_internal_clk_enable(se_nr_dev *nr_dev, u32 isp_clk_enable);

int cisp_nr_get_average(se_nr_dev *nr_dev, u32 *avg);

int cisp_nr_set_idle(se_nr_dev *nr_dev);

int cisp_nr_set_size(se_nr_dev *nr_dev, u32 width, u32 height);

int cisp_nr_set_rdma_mode(se_nr_dev *nr_dev, u32 mode);

int cisp_nr_set_bayer_pattern(se_nr_dev *nr_dev, u32 pattern);

int cisp_nr_denoise3d_dilate_enable(se_nr_dev *nr_dev, u32 enable);

int cisp_nr_denoise3d_temperal_enable(se_nr_dev *nr_dev, u32 enable);

int cisp_nr_denoise3d_vertical_enable(se_nr_dev *nr_dev, u32 enable);

int cisp_nr_denoise3d_horizontal_enable(se_nr_dev *nr_dev, u32 enable);

int cisp_nr_sp2_wr_memory_config(se_nr_dev *nr_dev);

int cisp_nr_sp2_rd_memory_config(se_nr_dev *nr_dev);

int cisp_nr_update_delta(se_nr_dev *nr_dev, void *arg);

int cisp_nr_update_strength(se_nr_dev *nr_dev, void *arg);

int cisp_nr_update_motion(se_nr_dev *nr_dev, void *arg);

int cisp_nr_update_denoise3d_range_s(se_nr_dev *nr_dev, void *arg);

int cisp_nr_update_denoise3d_range_t(se_nr_dev *nr_dev, void *arg);

int cisp_nr_update_denoise3d_edge_v(se_nr_dev *nr_dev, void *arg);

int cisp_nr_update_denoise3d_edge_h(se_nr_dev *nr_dev, void *arg);

int cisp_nr_update_denoise3d(se_nr_dev *nr_dev, denoise3d_update *update);

int cisp_3dnr_params_init(se_nr_dev *nr_dev, denoise3d_params *config);

int cisp_nr_denoise3d_enable(se_nr_dev *nr_dev, u32 enable);

int cisp_nr_denoise3d_set_params(se_nr_dev *nr_dev, denoise3d_params params);

int cisp_nr_denoise3d_get_params(se_nr_dev *nr_dev, denoise3d_params *params);

int set_h_blank(se_nr_dev *nr_dev, u32 hblank);

int mi_sp2_config_update(se_nr_dev *nr_dev);

int cisp_nr_update_config(se_nr_dev *nr_dev);

int cisp_nr_enable(se_nr_dev *nr_dev, u32 enable);

void cisp_nr_reg_dump(se_nr_dev *nr_dev);

int cisp_nr_get_param(se_nr_dev *nr_dev);

int cisp_nr_init(se_nr_dev *nr_dev);

void cisp_nr_set_default(se_nr_dev *nr_dev);

int cisp_nr_start_rdma(se_nr_dev *nr_dev);

int cisp_nr_update_denoise3d_curve_s_0(se_nr_dev *nr_dev, void *arg);

int cisp_nr_update_denoise3d_curve_s_1(se_nr_dev *nr_dev, void *arg);

int cisp_nr_update_denoise3d_curve_s_2(se_nr_dev *nr_dev, void *arg);

int cisp_nr_update_denoise3d_curve_s_3(se_nr_dev *nr_dev, void *arg);

int cisp_nr_update_denoise3d_curve_s_4(se_nr_dev *nr_dev, void *arg);

int cisp_nr_update_denoise3d_curve_s_5(se_nr_dev *nr_dev, void *arg);

int cisp_nr_update_denoise3d_curve_t_0(se_nr_dev *nr_dev, void *arg);

int cisp_nr_update_denoise3d_curve_t_1(se_nr_dev *nr_dev, void *arg);

int cisp_nr_update_denoise3d_curve_t_2(se_nr_dev *nr_dev, void *arg);

int cisp_nr_update_denoise3d_curve_t_3(se_nr_dev *nr_dev, void *arg);

int cisp_nr_update_denoise3d_curve_t_4(se_nr_dev *nr_dev, void *arg);

int cisp_nr_update_denoise3d_curve_t_5(se_nr_dev *nr_dev, void *arg);

int cisp_nr_get_denoise3d_strength_shd(se_nr_dev *nr_dev, u32 *arg);

int cisp_nr_get_denoise3d_edge_h_shd(se_nr_dev *nr_dev, u32 *arg);

int cisp_nr_get_denoise3d_edge_v_shd(se_nr_dev *nr_dev, u32 *arg);

int cisp_nr_get_range_s_shd(se_nr_dev *nr_dev, u32 *arg);

int cisp_nr_get_range_t_shd(se_nr_dev *nr_dev, u32 *arg);

int cisp_nr_get_motion_shd(se_nr_dev *nr_dev, u32 *arg);

int cisp_nr_get_delta_iva_shd(se_nr_dev *nr_dev, u32 *arg);

int cisp_nr_update_dummy_hblank(se_nr_dev *nr_dev, void *arg);

int cisp_nr_get_ctrl_shd(se_nr_dev *nr_dev, u32 *arg);

#endif
