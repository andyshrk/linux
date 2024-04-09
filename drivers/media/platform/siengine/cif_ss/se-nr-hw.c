// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021-2022 Siengine Technology, Inc.
 *
 * Sienginie Camera Interface driver
 *
 * Author: Siengine Technology, Inc.
 */

#include <linux/delay.h>
#include "se-nr-hw.h"
#include "log.h"

int num, ret = 0;

#define GOLDEN_TEST		0
#define SELF_TEST		1
#define GOLDEN_TEST_1080P	1
#define GOLDEN_TEST_480P	0
#define GOLDEN_TEST_160P	0

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

NR200 nr200_reg = {
	.INTERNAL_CLK_CTRL			= 0x01c,
	.VI_IRCL				= 0x020,
	.ISP_CTRL				= 0x024,
	.ISP_OUT_H_SIZE				= 0x028,
	.ISP_OUT_V_SIZE				= 0x02C,
	.MI_CTRL				= 0x100,
	.MI_QOS					= 0x108,
	.MI_SP2_CTRL				= 0x2E4,
	.MI_SP2_FMT				= 0x2E8,
	.MI_SP2_BUS_CFG				= 0x2EC,
	.MI_SP2_BUS_ID				= 0x2F0,
	.MI_SP2_RAW_BASE_AD			= 0x368, //rf write addr
	.MI_SP2_RAW_SIZE			= 0x36C,
	.MI_SP2_RAW_OFFS_CNT			= 0x370,
	.MI_SP2_RAW_LLENGTH			= 0x374,
	.MI_SP2_RAW_PIC_WIDTH			= 0x378,
	.MI_SP2_RAW_PIC_HEIGHT			= 0x37C,
	.MI_SP2_RAW_PIC_SIZE			= 0x380,
	.MI_SP2_DMA_RAW_PIC_START_AD		= 0x3BC, //rf read addr
	.MI_SP2_DMA_RAW_PIC_WIDTH		= 0x3C0,
	.MI_SP2_DMA_RAW_PIC_LLENGTH		= 0x3C4,
	.MI_SP2_DMA_RAW_PIC_SIZE		= 0x3C8,
	.MI_SP2_DMA_RAW_PIC_LVAL		= 0x3EC,
	.DENOISE3D_CTRL				= 0x600,
	.DENOISE3D_STRENGTH_REG			= 0x604,
	.DENOISE3D_EDGE_H			= 0x608,
	.DENOISE3D_EDGE_V			= 0x60C,
	.DENOISE3D_RANGE_S			= 0x610,
	.DENOISE3D_RANGE_T			= 0x614,
	.DENOISE3D_MOTION			= 0x618,
	.DENOISE3D_DELTA_INV			= 0x61C,
	.DENOISE3D_CURVE_S_0			= 0x620,
	.DENOISE3D_CURVE_S_1			= 0x624,
	.DENOISE3D_CURVE_S_2			= 0x628,
	.DENOISE3D_CURVE_S_3			= 0x62C,
	.DENOISE3D_CURVE_S_4			= 0x630,
	.DENOISE3D_CURVE_S_5			= 0x634,
	.DENOISE3D_CURVE_T_0			= 0x638,
	.DENOISE3D_CURVE_T_1			= 0x63C,
	.DENOISE3D_CURVE_T_2			= 0x640,
	.DENOISE3D_CURVE_T_3			= 0x644,
	.DENOISE3D_CURVE_T_4			= 0x648,
	.DENOISE3D_CURVE_T_5			= 0x64C,
	.DENOISE3D_AVERAGE			= 0x650,
	.DENOISE3D_STRENGT_SHD			= 0x654,
	.DENOISE3D_EDGE_H_SHD			= 0x658,
	.DENOISE3D_EDGE_V_SHD			= 0x65C,
	.DENOISE3D_RANGE_S_SHD			= 0x660,
	.DENOISE3D_RANGE_T_SHD			= 0x664,
	.DENOISE3D_MOTION_SHD			= 0x668,
	.DENOISE3D_DELTA_INV_SHD		= 0x66C,
	.DENOISE3D_DUMMY_HBLANK			= 0x670,
	.DENOISE3D_CTRL_SHD			= 0x674,
	.ISP_IMSC_INTERRUPT			= 0x040,
	.ISP_RIS_INTERRUPT			= 0x044,
	.ISP_MIS_INTERRUPT			= 0x048,
	.ISP_ICR_INTERRUPT			= 0x04c,
	.MI_IMSC_INTERRUPT			= 0x4c0,
	.MI_IMSC1_INTERRUPT			= 0x4c4,
	.MI_RIS_INTERRUPT			= 0x4e0,
	.MI_RIS1_INTERRUPT			= 0x4e4,
	.MI_MIS_INTERRUPT			= 0x4d0,
	.MI_MIS1_INTERRUPT			= 0x4d4,
	.MI_ICR_INTERRUPT			= 0x4d8,
	.MI_ICR1_INTERRUPT			= 0x4dc,
};

/*
 Immediate ISP configuration update. Set ISP_CTRL.isp_cfg_upd[bit 9 = 1].
 */
int cisp_nr_update_config(se_nr_dev *nr_dev)
{
	void* isp_ctrl_addr = nr_dev->base + nr200_reg.ISP_CTRL;
	u32 isp_ctrl_val = 0x00;
	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	isp_ctrl_val= readl(isp_ctrl_addr);
	REG_SET_SLICE(isp_ctrl_val, NR200_ISP_CTRL_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl_val, isp_ctrl_addr);

	return 0;
}

/*
 The NR interrupt will raise the CIF interrupt and the CIF is not design a
 independent IRQ source for this situation. So we should disable the NR interrupt.
 */
int cisp_nr_disable_all_interrupt(se_nr_dev *nr_dev)
{
	void *reg_addr;
	u32 reg_value = 0x00000000;

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	REPORT_FUNC();

	reg_addr= nr_dev->base + nr200_reg.ISP_IMSC_INTERRUPT;
	CIF_LOG(LOG_DEBUG, "set DENOISE3D_ISP_MISC_INTERRUPT %p = 0x%0x", reg_addr, reg_value);
	nr_write_dbg(reg_value, reg_addr);

	reg_addr= nr_dev->base + nr200_reg.MI_IMSC_INTERRUPT;
	CIF_LOG(LOG_DEBUG, "set DENOISE3D_MI_IMSC_INTERRUPT %p = 0x%0x", reg_addr, reg_value);
	nr_write_dbg(reg_value, reg_addr);

	reg_addr= nr_dev->base + nr200_reg.MI_IMSC1_INTERRUPT;
	CIF_LOG(LOG_DEBUG, "set DENOISE3D_MI_IMSC1_INTERRUPT %p = 0x%0x", reg_addr, reg_value);
	nr_write_dbg(reg_value, reg_addr);

	return 0;
}


int cisp_nr_enable_all_interrupt(se_nr_dev *nr_dev)
{
	void *reg_addr;
	u32 reg_value = 0x00000000;

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	REPORT_FUNC();

	reg_addr= nr_dev->base + nr200_reg.ISP_IMSC_INTERRUPT;
	reg_value = 0x00000003;
	CIF_LOG(LOG_DEBUG, "set DENOISE3D_ISP_MISC_INTERRUPT %p = 0x%0x", reg_addr, reg_value);
	nr_write_dbg(reg_value, reg_addr);

	reg_value = 0x00000000 | (1 << 5) | (1 << 19) | (1 << 23);
	reg_addr= nr_dev->base + nr200_reg.MI_IMSC_INTERRUPT;
	CIF_LOG(LOG_DEBUG, "set DENOISE3D_MI_IMSC_INTERRUPT %p = 0x%0x", reg_addr, reg_value);
	nr_write_dbg(reg_value, reg_addr);

	reg_value = 0x00000000 | (1 << 19) | (1 << 25);
	reg_addr= nr_dev->base + nr200_reg.MI_IMSC1_INTERRUPT;
	CIF_LOG(LOG_DEBUG, "set DENOISE3D_MI_IMSC1_INTERRUPT %p = 0x%0x", reg_addr, reg_value);
	nr_write_dbg(reg_value, reg_addr);

	return 0;
}

int cisp_nr_process_irq(se_nr_dev *nr_dev)
{
	void *reg_addr;
	u32 status;

	reg_addr = nr_dev->base + nr200_reg.MI_RIS_INTERRUPT;
	status = readl(reg_addr);

	reg_addr = nr_dev->base + nr200_reg.MI_ICR_INTERRUPT;
	nr_write_dbg(status, reg_addr);
	if (status & 0x20) {
		//cisp_nr_start_rdma(nr_dev);
		pr_debug("%s:Self Path2 RAW end of frame interrupt\n", __func__);
	}

	if (status & 0x800000)
		pr_debug("%s:Sp2_dma RAW ready interrupt\n", __func__);

	reg_addr = nr_dev->base + nr200_reg.ISP_MIS_INTERRUPT;
	status = readl(reg_addr);
	if (status)
		pr_debug("ISP_MIS_INTERRUPT(0x%0x) status:0x%0x",
			nr200_reg.ISP_MIS_INTERRUPT, status);
	reg_addr = nr_dev->base + nr200_reg.ISP_RIS_INTERRUPT;
	status = readl(reg_addr);

	reg_addr = nr_dev->base + nr200_reg.ISP_ICR_INTERRUPT;
	nr_write_dbg(status, reg_addr);
	if (status)
		pr_debug("ISP_RIS_INTERRUPT(0x%0x) status:0x%0x",
			nr200_reg.ISP_RIS_INTERRUPT, status);
	return 0;
}

int cisp_nr_internal_clk_enable(se_nr_dev *nr_dev, u32 enable)
{
	void* internal_addr = nr_dev->base + nr200_reg.INTERNAL_CLK_CTRL;
	u32 internal_clk_val = readl(internal_addr);

	REPORT_FUNC();

	REG_SET_SLICE(internal_clk_val, MRV_VI_CP_CLK_ENABLE, enable);
	REG_SET_SLICE(internal_clk_val, MRV_VI_ISP_CLK_ENABLE, enable);
	nr_write_dbg(internal_clk_val, internal_addr);

	nr_dev->cpproc_clk_enable = enable;
	nr_dev->isp_clk_enable = enable;

	return 0;
}

/*
 set NR200 to idle state(ISP_CTRL.nr200_enable[bit0 = 0])
 */
int cisp_nr_set_idle(se_nr_dev *nr_dev)
{
#if 0
	void* isp_ctrl_addr = nr_dev->base + nr200_reg.ISP_CTRL;
	u32 isp_ctrl_val = 0x00;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	isp_ctrl_val = readl(isp_ctrl_addr);

	CIF_LOG(LOG_DEBUG, "read isp_ctrl ->%p = 0x%0x", isp_ctrl_addr, isp_ctrl_val);
	REG_SET_SLICE(isp_ctrl_val, NR200_ISP_ENABLE, 0);
	nr_write_dbg(isp_ctrl_val, isp_ctrl_addr);

	cisp_nr_update_config(nr_dev);
#endif
	void* vi_ircl_addr = nr_dev->base + nr200_reg.VI_IRCL;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	nr_write_dbg(0x07, vi_ircl_addr); //Applying software resets to all sub-modules
	msleep(15);
	CIF_LOG(LOG_INFO, "%s reset success, nr in idle mode\n", __func__);
	return 0;
}


/*
 Image acquisition configuration:
 Set the output acquisition window depending on the attached camera device
 ISP_OUT_H_SIZE
 ISP_OUT_V_SIZE
 */
int cisp_nr_set_size(se_nr_dev *nr_dev, u32 width, u32 height)
{
	void* out_width_addr = nr_dev->base + nr200_reg.ISP_OUT_H_SIZE;
	void* out_height_addr = nr_dev->base + nr200_reg.ISP_OUT_V_SIZE;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	nr_write_dbg(width, out_width_addr);
	nr_write_dbg(height, out_height_addr);

	nr_dev->out_h_size = width;
	nr_dev->out_v_size = height;
	return 0;
}

/*
 set rdma mode
 0 = normal mode, should trigger rdma start before every frame start
 1 = continuous mode, will trigger rdma start automatic
 */
int cisp_nr_set_rdma_mode(se_nr_dev *nr_dev, u32 mode)
{
	void* reg_addr = nr_dev->base + nr200_reg.MI_CTRL;
	u32 reg_val = 0x00;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	if (mode >= RDMA_MODE_MAX) {
		CIF_LOG(LOG_ERROR, "rdma mode value invalid, will use normal mode default");
		mode = NORMAL_MODE;
	}

	reg_val= readl(reg_addr);
	REG_SET_SLICE(reg_val, SP2_RAW_RDMA_START_CON, mode);
	nr_write_dbg(reg_val, reg_addr);
	reg_addr = nr_dev->base + nr200_reg.ISP_CTRL;

	//update rdma mode config
	reg_val = readl(reg_addr);
	REG_SET_SLICE(reg_val, NR200_ISP_CTRL_CFG_UPD, 1);
	nr_write_dbg(reg_val, reg_addr);

	nr_dev->rdma_mode = mode;
	CIF_LOG(LOG_INFO, "%s: %s\n", __func__, mode ? "Continuous":"Normal");
	return 0;
}
/*
 Define Bayer pattern layout(ISP_CTRL.bayer_pat)
 00 = first line: RGRG..., second line: GBGB..., etc
 01 = first line: GRGR..., second line: BGBG..., etc
 10 = first line: GBGB..., second line: RGRG..., etc
 11 = first line: BGBG..., second line: GRGR..., etc
 This configuration applies for the black level area after croping by the input formatter
 */
int cisp_nr_set_bayer_pattern(se_nr_dev *nr_dev, u32 pattern)
{
	void* isp_ctrl_addr = nr_dev->base + nr200_reg.ISP_CTRL;
	u32 isp_ctrl_val = 0x00;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	if (pattern >= PATTERN_MAX) {
		CIF_LOG(LOG_ERROR, "pattern value invalid, will use RGRG-GBGB default");
		pattern = RGRG_GBGB;
	}

	isp_ctrl_val = readl(isp_ctrl_addr);
	REG_SET_SLICE(isp_ctrl_val, NR200_ISP_BAYER_PATTERN, pattern);
	nr_write_dbg(isp_ctrl_val, isp_ctrl_addr);

	isp_ctrl_val = readl(isp_ctrl_addr);
	REG_SET_SLICE(isp_ctrl_val, NR200_ISP_CTRL_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl_val, isp_ctrl_addr);

	return 0;
}

int cisp_nr_denoise3d_dilate_enable(se_nr_dev *nr_dev, u32 enable)
{
	void* denoise3d_ctrl_addr = nr_dev->base + nr200_reg.DENOISE3D_CTRL;
	u32 denoise3d_ctrl_val = 0x00;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	enable = enable ? 1: 0;
	denoise3d_ctrl_val = readl(denoise3d_ctrl_addr);
	REG_SET_SLICE(denoise3d_ctrl_val, DENOISE3D_DILATE_EN, enable);
	nr_write_dbg(denoise3d_ctrl_val, denoise3d_ctrl_addr);

	return 0;
}

int cisp_nr_denoise3d_temperal_enable(se_nr_dev *nr_dev, u32 enable)
{
	void* denoise3d_ctrl_addr = nr_dev->base + nr200_reg.DENOISE3D_CTRL;
	u32 denoise3d_ctrl_val = 0x00;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	enable = enable ? 1: 0;
	denoise3d_ctrl_val = readl(denoise3d_ctrl_addr);
	REG_SET_SLICE(denoise3d_ctrl_val, DENOISE3D_TEMPERAL_EN, enable);
	nr_write_dbg(denoise3d_ctrl_val, denoise3d_ctrl_addr);

	return 0;
}

int cisp_nr_denoise3d_vertical_enable(se_nr_dev *nr_dev, u32 enable)
{
	void* denoise3d_ctrl_addr = nr_dev->base + nr200_reg.DENOISE3D_CTRL;
	u32 denoise3d_ctrl_val = 0x00;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	enable = enable ? 1: 0;
	denoise3d_ctrl_val = readl(denoise3d_ctrl_addr);
	REG_SET_SLICE(denoise3d_ctrl_val, DENOISE3D_VERTICAL_EN, enable);
	nr_write_dbg(denoise3d_ctrl_val, denoise3d_ctrl_addr);

	return 0;
}

int cisp_nr_denoise3d_horizontal_enable(se_nr_dev *nr_dev, u32 enable)
{
	void* denoise3d_ctrl_addr = nr_dev->base + nr200_reg.DENOISE3D_CTRL;
	u32 denoise3d_ctrl_val = 0x00;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	enable = enable ? 1: 0;
	denoise3d_ctrl_val = readl(denoise3d_ctrl_addr);
	REG_SET_SLICE(denoise3d_ctrl_val, DENOISE3D_HORIZONTAL_EN, enable);
	nr_write_dbg(denoise3d_ctrl_val, denoise3d_ctrl_addr);

	return SUCCESS;
}

//set 3dnr reference frame write address
int cisp_nr_sp2_wr_memory_config(se_nr_dev *nr_dev)
{
	u32 mi_ctrl_value = 0x00, in_width = 0x00, in_height = 0x00;
	u32 lval= 0, size = 0;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	in_width = readl(nr_dev->base + nr200_reg.ISP_OUT_H_SIZE);
	in_height = readl(nr_dev->base + nr200_reg.ISP_OUT_V_SIZE);

	/*prepare for the sp2 reference frame*/
	lval = (in_width * 12 + 127)/ 128;
	lval <<= 4;
	size = in_height * lval;

	nr_write_dbg(nr_dev->rf_wr_addr, nr_dev->base + nr200_reg.MI_SP2_RAW_BASE_AD);
	nr_write_dbg(size, nr_dev->base + nr200_reg.MI_SP2_RAW_SIZE);//size);
	nr_write_dbg(0, nr_dev->base + nr200_reg.MI_SP2_RAW_OFFS_CNT);
	nr_write_dbg(lval, nr_dev->base + nr200_reg.MI_SP2_RAW_LLENGTH);
	nr_write_dbg(in_width, nr_dev->base + nr200_reg.MI_SP2_RAW_PIC_WIDTH);
	nr_write_dbg(in_height, nr_dev->base + nr200_reg.MI_SP2_RAW_PIC_HEIGHT);
	nr_write_dbg(size, nr_dev->base + nr200_reg.MI_SP2_RAW_PIC_SIZE);

	mi_ctrl_value = readl(nr_dev->base + nr200_reg.MI_CTRL);
	REG_SET_SLICE(mi_ctrl_value, SP2_RAW_PATH_ENABLE, 1);
	nr_write_dbg(mi_ctrl_value, nr_dev->base + nr200_reg.MI_CTRL);

	nr_write_dbg(0x084fd56c, nr_dev->base + nr200_reg.MI_SP2_BUS_ID);
	nr_write_dbg(0x0015004a, nr_dev->base + nr200_reg.MI_SP2_FMT);
	nr_write_dbg(0x238, nr_dev->base + nr200_reg.MI_SP2_CTRL);

	return 0;
}

//set 3dnr reference frame read address
int cisp_nr_sp2_rd_memory_config(se_nr_dev *nr_dev)
{
	u32 mi_ctrl_value = 0x00, mi_sp2_ctrl_value = 0x00;
	u32 in_width = 0x00, in_height = 0x00, lval = 0, size = 0;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	mi_ctrl_value = readl(nr_dev->base + nr200_reg.MI_CTRL);
	mi_sp2_ctrl_value = readl(nr_dev->base + nr200_reg.MI_SP2_CTRL);
	//u32 mi_imsc_value = readl(nr_dev->base + nr200_reg.MI_IMSC);
	in_width = readl(nr_dev->base + nr200_reg.ISP_OUT_H_SIZE);
	in_height = readl(nr_dev->base + nr200_reg.ISP_OUT_V_SIZE);

	/*prepare for the sp2 reference frame*/
	lval = (in_width * 12 + 127)/ 128;
	lval <<= 4;
	size = in_height * lval;

	REG_SET_SLICE(mi_ctrl_value, SP2_RAW_RDMA_PATH_ENABLE, 1);
	REG_SET_SLICE(mi_ctrl_value, SP2_RAW_PATH_ENABLE, 1);
	nr_write_dbg(nr_dev->rf_rd_addr, nr_dev->base + nr200_reg.MI_SP2_DMA_RAW_PIC_START_AD);
	nr_write_dbg(lval, nr_dev->base + nr200_reg.MI_SP2_DMA_RAW_PIC_LLENGTH);
	nr_write_dbg(in_width, nr_dev->base + nr200_reg.MI_SP2_DMA_RAW_PIC_WIDTH);
	nr_write_dbg(lval, nr_dev->base + nr200_reg.MI_SP2_DMA_RAW_PIC_LVAL);
	nr_write_dbg(size, nr_dev->base + nr200_reg.MI_SP2_DMA_RAW_PIC_SIZE);

	REG_SET_SLICE(mi_sp2_ctrl_value, SP2_RD_RAW_CFG_UPDATE, 1);
	REG_SET_SLICE(mi_sp2_ctrl_value, SP2_RD_RAW_AUTO_UPDATE, 1);
	REG_SET_SLICE(mi_sp2_ctrl_value, SP2_MI_CFG_UPD, 1);

	//mi_imsc_value |= SP2_DMA_RAW_READY_MASK;

	nr_write_dbg(mi_sp2_ctrl_value, nr_dev->base + nr200_reg.MI_SP2_CTRL);
	//nr_write_dbg(nr_dev->base + nr200_reg.MI_IMSC, mi_imsc_value);
	nr_write_dbg(mi_ctrl_value, nr_dev->base + nr200_reg.MI_CTRL);

	return 0;
}

int cisp_nr_update_delta(se_nr_dev *nr_dev, void *arg)
{
	u32  isp_denoise3d_delta_inv = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set ISP_DENOISE3D_DELTA type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_DELTA_UPDATE_T_INV:
			set_val = MIN(set_val, DELTA_T_INV_MAX);
			isp_denoise3d_delta_inv = readl(nr_dev->base + nr200_reg.DENOISE3D_DELTA_INV);
			REG_SET_SLICE(isp_denoise3d_delta_inv, DENOISE3D_DELTA_T_INV, set_val);
			nr_write_dbg(isp_denoise3d_delta_inv, nr_dev->base + nr200_reg.DENOISE3D_DELTA_INV);
			break;

		case ISP_DENOISE3D_DELTA_UPDATE_V_INV:
			set_val = MIN(set_val, DELTA_T_INV_MAX);
			isp_denoise3d_delta_inv = readl(nr_dev->base + nr200_reg.DENOISE3D_DELTA_INV);
			REG_SET_SLICE(isp_denoise3d_delta_inv, DENOISE3D_DELTA_V_INV, set_val);
			nr_write_dbg(isp_denoise3d_delta_inv, nr_dev->base + nr200_reg.DENOISE3D_DELTA_INV);
			break;

		case ISP_DENOISE3D_DELTA_UPDATE_H_INV:
			set_val = MIN(set_val, DELTA_T_INV_MAX);
			isp_denoise3d_delta_inv = readl(nr_dev->base + nr200_reg.DENOISE3D_DELTA_INV);
			REG_SET_SLICE(isp_denoise3d_delta_inv, DENOISE3D_DELTA_H_INV, set_val);
			nr_write_dbg(isp_denoise3d_delta_inv, nr_dev->base + nr200_reg.DENOISE3D_DELTA_INV);
			break;

		case ISP_DENOISE3D_DELTA_GET_REG:
			isp_denoise3d_delta_inv = readl(nr_dev->base + nr200_reg.DENOISE3D_DELTA_INV);
			nr_para->para_data = isp_denoise3d_delta_inv;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_DELTA_INV);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);
	nr_dev->params.thr_delta_t_inv = isp_denoise3d_delta_inv;

	return 0;
}

int cisp_nr_update_strength(se_nr_dev *nr_dev, void *arg)
{
	u32  isp_denoise3d_strength = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set ISP_DENOISE3D_STRENGTH type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_STRENGTH_UPDATE_STRENGTH:
			isp_denoise3d_strength = readl(nr_dev->base + nr200_reg.DENOISE3D_STRENGTH_REG);
			REG_SET_SLICE(isp_denoise3d_strength, DENOISE3D_STRENGTH, set_val);
			nr_write_dbg(isp_denoise3d_strength, nr_dev->base + nr200_reg.DENOISE3D_STRENGTH_REG);
			break;

		case ISP_DENOISE3D_STRENGTH_UPDATE_SPACIAL:
			isp_denoise3d_strength = readl(nr_dev->base + nr200_reg.DENOISE3D_STRENGTH_REG);
			REG_SET_SLICE(isp_denoise3d_strength, DENOISE3D_UPDATE_SPACIAL, set_val);
			nr_write_dbg(isp_denoise3d_strength, nr_dev->base + nr200_reg.DENOISE3D_STRENGTH_REG);
			break;

		case ISP_DENOISE3D_STRENGTH_UPDATE_TEMPERAL:
			isp_denoise3d_strength = readl(nr_dev->base + nr200_reg.DENOISE3D_STRENGTH_REG);
			REG_SET_SLICE(isp_denoise3d_strength, DENOISE3D_UPDATE_TEMPERAL, set_val);
			nr_write_dbg(isp_denoise3d_strength, nr_dev->base + nr200_reg.DENOISE3D_STRENGTH_REG);
			break;

		case ISP_DENOISE3D_STRENGTH_GET_REG:
			isp_denoise3d_strength = readl(nr_dev->base + nr200_reg.DENOISE3D_STRENGTH_REG);
			nr_para->para_data = isp_denoise3d_strength;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_STRENGTH_REG);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);
	nr_dev->params.strength = isp_denoise3d_strength;

	return 0;
}

int cisp_nr_update_motion(se_nr_dev *nr_dev, void *arg)
{
	u32 isp_denoise3d_motion = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set ISP_DENOISE3D_MOTION type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_MOTION_UPDATE_INV:
			set_val = MIN(set_val, THR_MOTION_INV_MAX);
			isp_denoise3d_motion = readl(nr_dev->base + nr200_reg.DENOISE3D_MOTION);
			REG_SET_SLICE(isp_denoise3d_motion, DENOISE3D_MOTION_INV, set_val);
			nr_write_dbg(isp_denoise3d_motion, nr_dev->base + nr200_reg.DENOISE3D_MOTION);
			break;

		case ISP_DENOISE3D_MOTION_UPDATE_RANGE_D:
			set_val = MIN(set_val, THR_MOTION_INV_MAX);
			isp_denoise3d_motion = readl(nr_dev->base + nr200_reg.DENOISE3D_MOTION);
			REG_SET_SLICE(isp_denoise3d_motion, DENOISE3D_RANGE_D, set_val);
			nr_write_dbg(isp_denoise3d_motion, nr_dev->base + nr200_reg.DENOISE3D_MOTION);
			break;

		case ISP_DENOISE3D_MOTION_GET_REG:
			isp_denoise3d_motion = readl(nr_dev->base + nr200_reg.DENOISE3D_MOTION);
			nr_para->para_data = isp_denoise3d_motion;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_MOTION);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);
	nr_dev->params.thr_motion_inv = isp_denoise3d_motion;

	return 0;
}

int cisp_nr_get_average(se_nr_dev *nr_dev, u32 *avg)
{
	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	*avg = readl(nr_dev->base + nr200_reg.DENOISE3D_AVERAGE);
	return 0;
}

int cisp_nr_get_denoise3d_strength_shd(se_nr_dev *nr_dev, u32 *arg)
{
	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	*arg = readl(nr_dev->base + nr200_reg.DENOISE3D_STRENGT_SHD);
	return 0;
}

int cisp_nr_get_denoise3d_edge_h_shd(se_nr_dev *nr_dev, u32 *arg)
{
	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	*arg = readl(nr_dev->base + nr200_reg.DENOISE3D_EDGE_H_SHD);
	return 0;
}

int cisp_nr_get_denoise3d_edge_v_shd(se_nr_dev *nr_dev, u32 *arg)
{
	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	*arg = readl(nr_dev->base + nr200_reg.DENOISE3D_EDGE_V_SHD);
	return 0;
}

int cisp_nr_get_range_s_shd(se_nr_dev *nr_dev, u32 *arg)
{
	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	*arg = readl(nr_dev->base + nr200_reg.DENOISE3D_RANGE_S_SHD);
	return 0;
}

int cisp_nr_get_range_t_shd(se_nr_dev *nr_dev, u32 *arg)
{
	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	*arg = readl(nr_dev->base + nr200_reg.DENOISE3D_RANGE_T_SHD);
	return 0;
}

int cisp_nr_get_motion_shd(se_nr_dev *nr_dev, u32 *arg)
{
	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	*arg = readl(nr_dev->base + nr200_reg.DENOISE3D_MOTION_SHD);
	return 0;
}

int cisp_nr_get_delta_iva_shd(se_nr_dev *nr_dev, u32 *arg)
{
	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	*arg = readl(nr_dev->base + nr200_reg.DENOISE3D_DELTA_INV_SHD);
	return 0;
}

int cisp_nr_update_dummy_hblank(se_nr_dev *nr_dev, void *arg)
{
	u32  isp_dummy_hblank = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set dummy hblank type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_DUMMY_HBLANK_BIT15_0:
			isp_dummy_hblank = readl(nr_dev->base + nr200_reg.DENOISE3D_DUMMY_HBLANK);
			REG_SET_SLICE(isp_dummy_hblank, DENOISE3D_RANGE_S_INV, set_val);
			nr_write_dbg(isp_dummy_hblank, nr_dev->base + nr200_reg.DENOISE3D_DUMMY_HBLANK);
			break;

		case ISP_DENOISE3D_DUMMY_HBLANK_GET_REG:
			isp_dummy_hblank = readl(nr_dev->base + nr200_reg.DENOISE3D_DUMMY_HBLANK);
			nr_para->para_data = isp_dummy_hblank;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_DUMMY_HBLANK);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);
	nr_dev->params.h_blank = isp_dummy_hblank;
	return 0;
}


int cisp_nr_get_ctrl_shd(se_nr_dev *nr_dev, u32 *arg)
{
	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	*arg = readl(nr_dev->base + nr200_reg.DENOISE3D_CTRL_SHD);
	return 0;
}

static void cisp_nr_update_params(se_nr_dev *nr_dev, denoise3d_update *dnr3_update)
{
	REPORT_FUNC();

	nr_dev->params.thr_delta_h_inv = dnr3_update->thr_delta_h_inv;
	nr_dev->params.thr_delta_v_inv = dnr3_update->thr_delta_v_inv;
	nr_dev->params.thr_motion_inv = dnr3_update->thr_motion_inv;
	nr_dev->params.thr_range_s_inv = dnr3_update->thr_range_s_inv;
	nr_dev->params.range_t_h = dnr3_update->range_t_h;
	nr_dev->params.range_t_v = dnr3_update->range_t_v;
	nr_dev->params.range_d = dnr3_update->range_d;
	nr_dev->params.thr_range_t_inv = dnr3_update->thr_range_t_inv;
	nr_dev->params.thr_delta_h_inv = dnr3_update->thr_delta_h_inv;
	nr_dev->params.thr_delta_v_inv = dnr3_update->thr_delta_v_inv;
	nr_dev->params.thr_delta_t_inv = dnr3_update->thr_delta_t_inv;
}

int cisp_nr_update_denoise3d_range_s(se_nr_dev *nr_dev, void *arg)
{
	u32  isp_denoise3d_range_s = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set ISP_DENOISE3D_EDGE_V type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_RANGE_S_UPDATE_INV:
			isp_denoise3d_range_s = readl(nr_dev->base + nr200_reg.DENOISE3D_RANGE_S);
			REG_SET_SLICE(isp_denoise3d_range_s, DENOISE3D_RANGE_S_INV, set_val);
			nr_write_dbg(isp_denoise3d_range_s, nr_dev->base + nr200_reg.DENOISE3D_RANGE_S);
			break;

		case ISP_DENOISE3D_RANGE_S_GET_REG:
			isp_denoise3d_range_s = readl(nr_dev->base + nr200_reg.DENOISE3D_RANGE_S);
			nr_para->para_data = isp_denoise3d_range_s;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_RANGE_S);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);
	nr_dev->params.thr_range_s_inv = isp_denoise3d_range_s;
	return 0;
}

int cisp_nr_update_denoise3d_range_t(se_nr_dev *nr_dev, void *arg)
{
	u32  isp_denoise3d_range_t = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set ISP_DENOISE3D_EDGE_V type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_RANGE_T_UPDATE_INV:
			isp_denoise3d_range_t = readl(nr_dev->base + nr200_reg.DENOISE3D_RANGE_T);
			REG_SET_SLICE(isp_denoise3d_range_t, DENOISE3D_RANGE_T_INV, set_val);
			nr_write_dbg(isp_denoise3d_range_t, nr_dev->base + nr200_reg.DENOISE3D_RANGE_T);
			break;

		case ISP_DENOISE3D_RANGE_T_UPDATE_V:
			isp_denoise3d_range_t = readl(nr_dev->base + nr200_reg.DENOISE3D_RANGE_T);
			REG_SET_SLICE(isp_denoise3d_range_t, DENOISE3D_RANGE_T_V, set_val);
			nr_write_dbg(isp_denoise3d_range_t, nr_dev->base + nr200_reg.DENOISE3D_RANGE_T);
			break;

		case ISP_DENOISE3D_RANGE_T_UPDATE_H:
			isp_denoise3d_range_t = readl(nr_dev->base + nr200_reg.DENOISE3D_RANGE_T);
			REG_SET_SLICE(isp_denoise3d_range_t, DENOISE3D_RANGE_T_H, set_val);
			nr_write_dbg(isp_denoise3d_range_t, nr_dev->base + nr200_reg.DENOISE3D_RANGE_T);
			break;

		case ISP_DENOISE3D_RANGE_T_GET_REG:
			isp_denoise3d_range_t = readl(nr_dev->base + nr200_reg.DENOISE3D_RANGE_T);
			nr_para->para_data = isp_denoise3d_range_t;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_RANGE_T);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);
	nr_dev->params.thr_range_s_inv = isp_denoise3d_range_t;
	return 0;
}

int cisp_nr_update_denoise3d_edge_v(se_nr_dev *nr_dev, void *arg)
{
	u32  isp_denoise3d_edge_v = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set ISP_DENOISE3D_EDGE_V type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_EDGE_V_UPDATE_INV:
			isp_denoise3d_edge_v = readl(nr_dev->base + nr200_reg.DENOISE3D_EDGE_V);
			REG_SET_SLICE(isp_denoise3d_edge_v, DENOISE3D_THR_EDGE_V_INV, set_val);
			nr_write_dbg(isp_denoise3d_edge_v, nr_dev->base + nr200_reg.DENOISE3D_EDGE_V);
			break;

		case ISP_DENOISE3D_EDGE_V_UPDATE_CURVE_SPACIAL:
			isp_denoise3d_edge_v = readl(nr_dev->base + nr200_reg.DENOISE3D_EDGE_V);
			REG_SET_SLICE(isp_denoise3d_edge_v, DENOISE3D_STRENGTH_CURVE_SPACIAL, set_val);
			nr_write_dbg(isp_denoise3d_edge_v, nr_dev->base + nr200_reg.DENOISE3D_EDGE_V);
			break;

		case ISP_DENOISE3D_EDGE_V_GET_REG:
			isp_denoise3d_edge_v = readl(nr_dev->base + nr200_reg.DENOISE3D_EDGE_V);
			nr_para->para_data = isp_denoise3d_edge_v;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_EDGE_V);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);
	nr_dev->params.thr_edge_v_inv = isp_denoise3d_edge_v;
	return 0;
}

int cisp_nr_update_denoise3d_edge_h(se_nr_dev *nr_dev, void *arg)
{
	u32  isp_denoise3d_edge_h = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set ISP_DENOISE3D_EDGE_H type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_EDGE_H_UPDATE_INV:
			isp_denoise3d_edge_h = readl(nr_dev->base + nr200_reg.DENOISE3D_EDGE_H);
			REG_SET_SLICE(isp_denoise3d_edge_h, DENOISE3D_THR_EDGE_H_INV, set_val);
			nr_write_dbg(isp_denoise3d_edge_h, nr_dev->base + nr200_reg.DENOISE3D_EDGE_H);
			break;

		case ISP_DENOISE3D_EDGE_H_UPDATE_CURVE_SPACIAL:
			isp_denoise3d_edge_h = readl(nr_dev->base + nr200_reg.DENOISE3D_EDGE_H);
			REG_SET_SLICE(isp_denoise3d_edge_h, DENOISE3D_STRENGTH_CURVE_SPACIAL, set_val);
			nr_write_dbg(isp_denoise3d_edge_h, nr_dev->base + nr200_reg.DENOISE3D_EDGE_H);
			break;

		case ISP_DENOISE3D_EDGE_H_GET_REG:
			isp_denoise3d_edge_h = readl(nr_dev->base + nr200_reg.DENOISE3D_EDGE_H);
			nr_para->para_data = isp_denoise3d_edge_h;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_EDGE_H);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);
	nr_dev->params.thr_edge_h_inv = isp_denoise3d_edge_h;

	return 0;
}

int cisp_nr_update_denoise3d_curve_s_0(se_nr_dev *nr_dev, void *arg)
{
	u32  isp_denoise3d_curve_s = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set ISP_DENOISE3D_CURVE_S_0 type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_CURVE_UPDATE_BIT9_0:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_0);
			REG_SET_SLICE(isp_denoise3d_curve_s, DENOISE3D_SPACIAL_CURVE2, set_val);
			nr_write_dbg(isp_denoise3d_curve_s, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_0);
			nr_dev->params.spacial_curve[2] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT19_10:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_0);
			REG_SET_SLICE(isp_denoise3d_curve_s, DENOISE3D_SPACIAL_CURVE1, set_val);
			nr_write_dbg(isp_denoise3d_curve_s, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_0);
			nr_dev->params.spacial_curve[1] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT29_20:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_0);
			REG_SET_SLICE(isp_denoise3d_curve_s, DENOISE3D_SPACIAL_CURVE0, set_val);
			nr_write_dbg(isp_denoise3d_curve_s, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_0);
			nr_dev->params.spacial_curve[0] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_GET_REG:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_0);
			nr_para->para_data = isp_denoise3d_curve_s;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_0);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);

	return 0;
}


int cisp_nr_update_denoise3d_curve_s_1(se_nr_dev *nr_dev, void *arg)
{
	u32  isp_denoise3d_curve_s = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set ISP_DENOISE3D_CURVE_S_1 type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_CURVE_UPDATE_BIT9_0:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_1);
			REG_SET_SLICE(isp_denoise3d_curve_s, DENOISE3D_SPACIAL_CURVE5, set_val);
			nr_write_dbg(isp_denoise3d_curve_s, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_1);
			nr_dev->params.spacial_curve[5] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT19_10:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_1);
			REG_SET_SLICE(isp_denoise3d_curve_s, DENOISE3D_SPACIAL_CURVE4, set_val);
			nr_write_dbg(isp_denoise3d_curve_s, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_1);
			nr_dev->params.spacial_curve[4] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT29_20:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_1);
			REG_SET_SLICE(isp_denoise3d_curve_s, DENOISE3D_SPACIAL_CURVE3, set_val);
			nr_write_dbg(isp_denoise3d_curve_s, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_1);
			nr_dev->params.spacial_curve[3] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_GET_REG:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_1);
			nr_para->para_data = isp_denoise3d_curve_s;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_1);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);

	return 0;
}

int cisp_nr_update_denoise3d_curve_s_2(se_nr_dev *nr_dev, void *arg)
{
	u32  isp_denoise3d_curve_s = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set ISP_DENOISE3D_CURVE_S_2 type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_CURVE_UPDATE_BIT9_0:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_2);
			REG_SET_SLICE(isp_denoise3d_curve_s, DENOISE3D_SPACIAL_CURVE8, set_val);
			nr_write_dbg(isp_denoise3d_curve_s, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_2);
			nr_dev->params.spacial_curve[8] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT19_10:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_2);
			REG_SET_SLICE(isp_denoise3d_curve_s, DENOISE3D_SPACIAL_CURVE7, set_val);
			nr_write_dbg(isp_denoise3d_curve_s, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_2);
			nr_dev->params.spacial_curve[7] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT29_20:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_2);
			REG_SET_SLICE(isp_denoise3d_curve_s, DENOISE3D_SPACIAL_CURVE6, set_val);
			nr_write_dbg(isp_denoise3d_curve_s, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_2);
			nr_dev->params.spacial_curve[6] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_GET_REG:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_2);
			nr_para->para_data = isp_denoise3d_curve_s;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_2);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);

	return 0;
}

int cisp_nr_update_denoise3d_curve_s_3(se_nr_dev *nr_dev, void *arg)
{
	u32  isp_denoise3d_curve_s = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set ISP_DENOISE3D_CURVE_S_3 type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_CURVE_UPDATE_BIT9_0:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_3);
			REG_SET_SLICE(isp_denoise3d_curve_s, DENOISE3D_SPACIAL_CURVE11, set_val);
			nr_write_dbg(isp_denoise3d_curve_s, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_3);
			nr_dev->params.spacial_curve[11] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT19_10:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_3);
			REG_SET_SLICE(isp_denoise3d_curve_s, DENOISE3D_SPACIAL_CURVE10, set_val);
			nr_write_dbg(isp_denoise3d_curve_s, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_3);
			nr_dev->params.spacial_curve[10] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT29_20:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_3);
			REG_SET_SLICE(isp_denoise3d_curve_s, DENOISE3D_SPACIAL_CURVE9, set_val);
			nr_write_dbg(isp_denoise3d_curve_s, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_3);
			nr_dev->params.spacial_curve[9] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_GET_REG:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_3);
			nr_para->para_data = isp_denoise3d_curve_s;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_3);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);

	return 0;
}

int cisp_nr_update_denoise3d_curve_s_4(se_nr_dev *nr_dev, void *arg)
{
	u32  isp_denoise3d_curve_s = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set ISP_DENOISE3D_CURVE_S_4 type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_CURVE_UPDATE_BIT9_0:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_4);
			REG_SET_SLICE(isp_denoise3d_curve_s, DENOISE3D_SPACIAL_CURVE14, set_val);
			nr_write_dbg(isp_denoise3d_curve_s, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_4);
			nr_dev->params.spacial_curve[14] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT19_10:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_4);
			REG_SET_SLICE(isp_denoise3d_curve_s, DENOISE3D_SPACIAL_CURVE13, set_val);
			nr_write_dbg(isp_denoise3d_curve_s, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_4);
			nr_dev->params.spacial_curve[13] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT29_20:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_4);
			REG_SET_SLICE(isp_denoise3d_curve_s, DENOISE3D_SPACIAL_CURVE12, set_val);
			nr_write_dbg(isp_denoise3d_curve_s, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_4);
			nr_dev->params.spacial_curve[12] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_GET_REG:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_4);
			nr_para->para_data = isp_denoise3d_curve_s;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_4);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);

	return 0;
}

int cisp_nr_update_denoise3d_curve_s_5(se_nr_dev *nr_dev, void *arg)
{
	u32  isp_denoise3d_curve_s = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set ISP_DENOISE3D_CURVE_S_5 type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_CURVE_UPDATE_BIT9_0:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_5);
			REG_SET_SLICE(isp_denoise3d_curve_s, DENOISE3D_SPACIAL_CURVE16, set_val);
			nr_write_dbg(isp_denoise3d_curve_s, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_5);
			nr_dev->params.spacial_curve[16] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT19_10:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_5);
			REG_SET_SLICE(isp_denoise3d_curve_s, DENOISE3D_SPACIAL_CURVE15, set_val);
			nr_write_dbg(isp_denoise3d_curve_s, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_5);
			nr_dev->params.spacial_curve[15] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_S5_T5_GET_REG:
			isp_denoise3d_curve_s = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_5);
			nr_para->para_data = isp_denoise3d_curve_s;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_5);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);

	return 0;
}

int cisp_nr_update_denoise3d_curve_t_0(se_nr_dev *nr_dev, void *arg)
{
	u32  isp_denoise3d_curve_t = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set ISP_DENOISE3D_CURVE_T_0 type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_CURVE_UPDATE_BIT9_0:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_0);
			REG_SET_SLICE(isp_denoise3d_curve_t, DENOISE3D_TEMPERAL_CURVE2, set_val);
			nr_write_dbg(isp_denoise3d_curve_t, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_0);
			nr_dev->params.temperal_curve[2] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT19_10:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_0);
			REG_SET_SLICE(isp_denoise3d_curve_t, DENOISE3D_TEMPERAL_CURVE1, set_val);
			nr_write_dbg(isp_denoise3d_curve_t, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_0);
			nr_dev->params.temperal_curve[2] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT29_20:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_0);
			REG_SET_SLICE(isp_denoise3d_curve_t, DENOISE3D_TEMPERAL_CURVE0, set_val);
			nr_write_dbg(isp_denoise3d_curve_t, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_0);
			nr_dev->params.temperal_curve[0] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_GET_REG:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_0);
			nr_para->para_data = isp_denoise3d_curve_t;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_0);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);

	return 0;
}

int cisp_nr_update_denoise3d_curve_t_1(se_nr_dev *nr_dev, void *arg)
{
	u32  isp_denoise3d_curve_t = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set ISP_DENOISE3D_CURVE_T_1 type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_CURVE_UPDATE_BIT9_0:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_1);
			REG_SET_SLICE(isp_denoise3d_curve_t, DENOISE3D_TEMPERAL_CURVE5, set_val);
			nr_write_dbg(isp_denoise3d_curve_t, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_1);
			nr_dev->params.temperal_curve[5] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT19_10:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_1);
			REG_SET_SLICE(isp_denoise3d_curve_t, DENOISE3D_TEMPERAL_CURVE4, set_val);
			nr_write_dbg(isp_denoise3d_curve_t, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_1);
			nr_dev->params.temperal_curve[4] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT29_20:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_1);
			REG_SET_SLICE(isp_denoise3d_curve_t, DENOISE3D_TEMPERAL_CURVE3, set_val);
			nr_write_dbg(isp_denoise3d_curve_t, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_1);
			nr_dev->params.temperal_curve[3] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_GET_REG:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_1);
			nr_para->para_data = isp_denoise3d_curve_t;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_1);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);

	return 0;
}

int cisp_nr_update_denoise3d_curve_t_2(se_nr_dev *nr_dev, void *arg)
{
	u32  isp_denoise3d_curve_t = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set ISP_DENOISE3D_CURVE_T_2 type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_CURVE_UPDATE_BIT9_0:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_2);
			REG_SET_SLICE(isp_denoise3d_curve_t, DENOISE3D_TEMPERAL_CURVE8, set_val);
			nr_write_dbg(isp_denoise3d_curve_t, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_2);
			nr_dev->params.temperal_curve[8] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT19_10:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_2);
			REG_SET_SLICE(isp_denoise3d_curve_t, DENOISE3D_TEMPERAL_CURVE7, set_val);
			nr_write_dbg(isp_denoise3d_curve_t, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_2);
			nr_dev->params.temperal_curve[7] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT29_20:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_2);
			REG_SET_SLICE(isp_denoise3d_curve_t, DENOISE3D_TEMPERAL_CURVE6, set_val);
			nr_write_dbg(isp_denoise3d_curve_t, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_2);
			nr_dev->params.temperal_curve[6] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_GET_REG:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_2);
			nr_para->para_data = isp_denoise3d_curve_t;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_2);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);

	return 0;
}

int cisp_nr_update_denoise3d_curve_t_3(se_nr_dev *nr_dev, void *arg)
{
	u32  isp_denoise3d_curve_t = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set ISP_DENOISE3D_CURVE_T_3 type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_CURVE_UPDATE_BIT9_0:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_3);
			REG_SET_SLICE(isp_denoise3d_curve_t, DENOISE3D_TEMPERAL_CURVE11, set_val);
			nr_write_dbg(isp_denoise3d_curve_t, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_3);
			nr_dev->params.temperal_curve[11] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT19_10:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_3);
			REG_SET_SLICE(isp_denoise3d_curve_t, DENOISE3D_TEMPERAL_CURVE10, set_val);
			nr_write_dbg(isp_denoise3d_curve_t, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_3);
			nr_dev->params.temperal_curve[10] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT29_20:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_3);
			REG_SET_SLICE(isp_denoise3d_curve_t, DENOISE3D_TEMPERAL_CURVE9, set_val);
			nr_write_dbg(isp_denoise3d_curve_t, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_3);
			nr_dev->params.temperal_curve[9] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_GET_REG:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_3);
			nr_para->para_data = isp_denoise3d_curve_t;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_3);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);

	return 0;
}

int cisp_nr_update_denoise3d_curve_t_4(se_nr_dev *nr_dev, void *arg)
{
	u32  isp_denoise3d_curve_t = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set ISP_DENOISE3D_CURVE_T_4 type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_CURVE_UPDATE_BIT9_0:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_4);
			REG_SET_SLICE(isp_denoise3d_curve_t, DENOISE3D_TEMPERAL_CURVE14, set_val);
			nr_write_dbg(isp_denoise3d_curve_t, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_4);
			nr_dev->params.temperal_curve[14] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT19_10:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_4);
			REG_SET_SLICE(isp_denoise3d_curve_t, DENOISE3D_TEMPERAL_CURVE13, set_val);
			nr_write_dbg(isp_denoise3d_curve_t, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_4);
			nr_dev->params.temperal_curve[13] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT29_20:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_4);
			REG_SET_SLICE(isp_denoise3d_curve_t, DENOISE3D_TEMPERAL_CURVE12, set_val);
			nr_write_dbg(isp_denoise3d_curve_t, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_4);
			nr_dev->params.temperal_curve[12] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_GET_REG:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_4);
			nr_para->para_data = isp_denoise3d_curve_t;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_4);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);

	return 0;
}

int cisp_nr_update_denoise3d_curve_t_5(se_nr_dev *nr_dev, void *arg)
{
	u32  isp_denoise3d_curve_t = 0x00, isp_ctrl = 0x00, set_val = 0;
	struct se_nr_para *nr_para = (struct se_nr_para *)arg;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	set_val = nr_para->para_data;
	CIF_LOG(LOG_INFO, "set ISP_DENOISE3D_CURVE_T_5 type = %d; data = 0x%x\n", nr_para->para_type, nr_para->para_data);

	switch(nr_para->para_type) {
		case ISP_DENOISE3D_CURVE_UPDATE_BIT9_0:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_5);
			REG_SET_SLICE(isp_denoise3d_curve_t, DENOISE3D_TEMPERAL_CURVE16, set_val);
			nr_write_dbg(isp_denoise3d_curve_t, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_5);
			nr_dev->params.temperal_curve[16] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_UPDATE_BIT19_10:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_5);
			REG_SET_SLICE(isp_denoise3d_curve_t, DENOISE3D_TEMPERAL_CURVE15, set_val);
			nr_write_dbg(isp_denoise3d_curve_t, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_5);
			nr_dev->params.temperal_curve[15] = set_val;
			break;

		case ISP_DENOISE3D_CURVE_GET_REG:
			isp_denoise3d_curve_t = readl(nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_5);
			nr_para->para_data = isp_denoise3d_curve_t;
			return 0;
			//break;

		case ISP_DENOISE3D_WRITE_REG:
			nr_write_dbg(set_val, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_5);
			break;

		default:
			return -1;
			//break;
	}

	isp_ctrl = readl(nr_dev->base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, nr_dev->base + nr200_reg.ISP_CTRL);

	return 0;
}

int cisp_nr_update_denoise3d(se_nr_dev *nr_dev, denoise3d_update *dnr3_update)
{
	u32 reg_val = 0;
	u32 isp_ctrl = 0;
	void __iomem *base = NULL;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	base = nr_dev->base;

	reg_val = readl(base + nr200_reg.DENOISE3D_STRENGTH_REG);
	REG_SET_SLICE(reg_val, DENOISE3D_STRENGTH, nr_dev->params.strength);
	nr_write_dbg(reg_val, base + nr200_reg.DENOISE3D_STRENGTH_REG);

	reg_val = 0;
	reg_val = readl(base + nr200_reg.DENOISE3D_EDGE_H);
	REG_SET_SLICE(reg_val, DENOISE3D_THR_EDGE_H_INV, dnr3_update->thr_edge_h_inv);
	nr_write_dbg(reg_val, base + nr200_reg.DENOISE3D_EDGE_H);

	reg_val = 0;
	reg_val = readl(base + nr200_reg.DENOISE3D_EDGE_V);
	REG_SET_SLICE(reg_val, DENOISE3D_THR_EDGE_V_INV, dnr3_update->thr_edge_v_inv);
	nr_write_dbg(reg_val, base + nr200_reg.DENOISE3D_EDGE_V);

	reg_val = 0;
	reg_val = readl(base + nr200_reg.DENOISE3D_RANGE_S);
	REG_SET_SLICE(reg_val, DENOISE3D_RANGE_S_INV, dnr3_update->thr_range_s_inv);
	nr_write_dbg(reg_val, base + nr200_reg.DENOISE3D_RANGE_S);

	reg_val = 0;
	reg_val = readl(base + nr200_reg.DENOISE3D_RANGE_T);
	REG_SET_SLICE(reg_val, DENOISE3D_RANGE_T_H, dnr3_update->range_t_h);
	REG_SET_SLICE(reg_val, DENOISE3D_RANGE_T_V, dnr3_update->range_t_v);
	REG_SET_SLICE(reg_val, DENOISE3D_RANGE_T_INV, dnr3_update->thr_range_t_inv);
	nr_write_dbg(reg_val, base + nr200_reg.DENOISE3D_RANGE_T);

	reg_val = 0;
	reg_val = readl(base + nr200_reg.DENOISE3D_MOTION);
	REG_SET_SLICE(reg_val, DENOISE3D_RANGE_D, dnr3_update->range_d);
	REG_SET_SLICE(reg_val, DENOISE3D_MOTION_INV, dnr3_update->thr_motion_inv);
	nr_write_dbg(reg_val, base + nr200_reg.DENOISE3D_MOTION);

	reg_val = 0;
	reg_val = readl(base + nr200_reg.DENOISE3D_DELTA_INV);
	REG_SET_SLICE(reg_val, DENOISE3D_DELTA_H_INV, dnr3_update->thr_delta_h_inv);
	REG_SET_SLICE(reg_val, DENOISE3D_DELTA_V_INV, dnr3_update->thr_delta_v_inv);
	REG_SET_SLICE(reg_val, DENOISE3D_DELTA_T_INV, dnr3_update->thr_delta_t_inv);
	nr_write_dbg(reg_val, base + nr200_reg.DENOISE3D_DELTA_INV);

	//update the config
	isp_ctrl = readl(base + nr200_reg.ISP_CTRL);
	REG_SET_SLICE(isp_ctrl, NR200_ISP_CTRL_GEN_CFG_UPD, 1);
	nr_write_dbg(isp_ctrl, base + nr200_reg.ISP_CTRL);

	cisp_nr_update_params(nr_dev, dnr3_update);

	return 0;
}


int cisp_3dnr_params_init(se_nr_dev *nr_dev, denoise3d_params *config)
{
	u32 i = 0;

	REPORT_FUNC();

	nr_dev->params.enable_h = config->enable_h;
	nr_dev->params.enable_v = config->enable_v;
	nr_dev->params.enable_temperal = config->enable_temperal;
	nr_dev->params.enable_dilate = config->enable_dilate;

	nr_dev->params.update_spatial = config->update_spatial;
	nr_dev->params.strength_curve_spatial = config->strength_curve_spatial;
	nr_dev->params.thr_edge_h_inv = config->thr_edge_h_inv;
	nr_dev->params.thr_edge_v_inv = config->thr_edge_v_inv;
	nr_dev->params.thr_range_s_inv = config->thr_range_s_inv;

	nr_dev->params.update_temperal = config->update_temperal;
	nr_dev->params.strength_curve_temperal = config->strength_curve_temperal;
	nr_dev->params.range_t_h = config->range_t_h;
	nr_dev->params.range_t_v = config->range_t_v;
	nr_dev->params.range_d = config->range_d;
	nr_dev->params.thr_range_t_inv = config->thr_range_t_inv;
	nr_dev->params.thr_delta_h_inv = config->thr_delta_h_inv;
	nr_dev->params.thr_delta_v_inv = config->thr_delta_v_inv;
	nr_dev->params.thr_delta_t_inv = config->thr_delta_t_inv;
	nr_dev->params.thr_motion_inv = config->thr_motion_inv;
	nr_dev->params.strength = config->strength;
	nr_dev->params.h_blank = config->h_blank;
	for (i = 0; i < 17; i++) {
		nr_dev->params.spacial_curve[i] = config->spacial_curve[i];
		nr_dev->params.temperal_curve[i] = config->temperal_curve[i];
	}

	CIF_LOG(LOG_INFO, "denoise3d params:\n"
	"enable_h: %d\n"
	"enable_v: %d\n"
	"enable_spacial: %d\n"
	"enable_dilate: %d\n"
	"update_spatial: %d\n"
	"strength_curve_spatial: %d\n"
	"thr_edge_h_inv: %d\n"
	"thr_edge_v_inv: %d\n"
	"thr_range_s_inv: %d\n"
	"uddate_temperal: %d\n"
	"strength_curve_temperal: %d\n"
	"range_t_h: %d\n"
	"range_t_v: %d\n"
	"range_d: %d\n"
	"thr_range_t_inv: %d\n"
	"thr_delta_h_inv: %d\n"
	"thr_delta_v_inv: %d\n"
	"thr_delta_t_inv: %d\n"
	"thr_motion_inv: %d\n"
	"strength: %d\n"
	"h_blank: %d",
	config->enable_h, config->enable_v, config->enable_temperal,
	config->enable_dilate, config->update_spatial, config->strength_curve_spatial,
	config->thr_edge_h_inv, config->thr_edge_v_inv, config->thr_range_s_inv,
	config->update_temperal, config->strength_curve_temperal,config->range_t_h,
	config->range_t_v, config->range_d, config->thr_range_t_inv,
	config->thr_delta_h_inv, config->thr_delta_v_inv, config->thr_delta_t_inv,
	config->thr_motion_inv, config->strength, config->h_blank);

	return 0;
}

static int set_update_temporal(se_nr_dev *nr_dev, u32 update_temporal)
{
	void __iomem *denoise3d_strength_addr = 0x00;
	u32 strength_val = 0x00;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	if (update_temporal > TEMPORAL_MAX) {
		CIF_LOG(LOG_ERROR, "temporal parameter invalid, should be [0: 1024], set to 1024");
		update_temporal = 1024;
	}

	denoise3d_strength_addr = nr_dev->base + nr200_reg.DENOISE3D_STRENGTH_REG;
	strength_val = readl(denoise3d_strength_addr);
	REG_SET_SLICE(strength_val, DENOISE3D_UPDATE_TEMPERAL, update_temporal);
	nr_write_dbg(strength_val, denoise3d_strength_addr);

	return SUCCESS;
}

static int set_update_spatial(se_nr_dev *nr_dev, u32 update_spatial)
{
	void __iomem *denoise3d_strength_addr = NULL;
	u32 strength_val = 0x00;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	if (update_spatial > SPACIAL_MAX) {
		CIF_LOG(LOG_ERROR, "spacial parameter invalid, should be [0: 1024], set to 1024");
		update_spatial = 1024;
	}

	denoise3d_strength_addr = nr_dev->base + nr200_reg.DENOISE3D_STRENGTH_REG;
	strength_val = readl(denoise3d_strength_addr);
	REG_SET_SLICE(strength_val, DENOISE3D_UPDATE_SPACIAL, update_spatial);
	nr_write_dbg(strength_val, denoise3d_strength_addr);

	return SUCCESS;
}

static int set_strength(se_nr_dev *nr_dev, u32 stregth)
{
	void __iomem *denoise3d_strength_addr = NULL;
	u32 strength_val;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	if (stregth > STRENGTH_MAX) {
		CIF_LOG(LOG_ERROR, "strength parameter invalid, should be [0: 128]");
		return PARAM_ERROR;
	}

	denoise3d_strength_addr = nr_dev->base + nr200_reg.DENOISE3D_STRENGTH_REG;
	strength_val = readl(denoise3d_strength_addr);
	REG_SET_SLICE(strength_val, DENOISE3D_STRENGTH, stregth);
	nr_write_dbg(strength_val, denoise3d_strength_addr);

	return SUCCESS;

}

static int set_strength_curve_spatial(se_nr_dev *nr_dev, u32 strength_curve_spatial)
{
	void __iomem *denoise3d_edge_h_addr = NULL;
	u32 edge_h_val = 0x00;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	if (strength_curve_spatial > STRENGTH_CURVE_SPATIAL_MAX) {
		CIF_LOG(LOG_ERROR, "spatial curve parameter invalid, set to 128");
		strength_curve_spatial = 128;
	}

	denoise3d_edge_h_addr = nr_dev->base + nr200_reg.DENOISE3D_EDGE_H;
	edge_h_val = readl(denoise3d_edge_h_addr);
	REG_SET_SLICE(edge_h_val, DENOISE3D_STRENGTH_CURVE_SPACIAL, strength_curve_spatial);
	nr_write_dbg(edge_h_val, denoise3d_edge_h_addr);

	return SUCCESS;
}

static int set_thr_edge_h_inv(se_nr_dev *nr_dev, u32 thr_edge_h_inv)
{
	void __iomem *denoise3d_edge_h_addr = NULL;
	u32 edge_h_val = 0x0;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	denoise3d_edge_h_addr = nr_dev->base + nr200_reg.DENOISE3D_EDGE_H;
	edge_h_val = readl(denoise3d_edge_h_addr);
	REG_SET_SLICE(edge_h_val, DENOISE3D_THR_EDGE_H_INV, thr_edge_h_inv);
	nr_write_dbg(edge_h_val, denoise3d_edge_h_addr);

	return SUCCESS;
}


static int set_strength_curve_temporal(se_nr_dev *nr_dev, u32 strength_curve_temporal)
{
	void __iomem *denoise3d_edge_v_addr = NULL;
	u32 edge_v_val = 0x0;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	if (strength_curve_temporal > STRENGTH_CURVE_TEMPORAL_MAX) {
		CIF_LOG(LOG_ERROR, "spatial curve parameter invalid, set to 128");
		strength_curve_temporal = 128;
	}

	denoise3d_edge_v_addr = nr_dev->base + nr200_reg.DENOISE3D_EDGE_V;
	edge_v_val = readl(denoise3d_edge_v_addr);
	REG_SET_SLICE(edge_v_val, DENOISE3D_STRENGTH_CURVE_TEMPERAL, strength_curve_temporal);
	nr_write_dbg(edge_v_val, denoise3d_edge_v_addr);

	return SUCCESS;
}

static int set_thr_edge_v_inv(se_nr_dev *nr_dev, u32 thr_edge_v_inv)
{
	void __iomem *denoise3d_edge_v_addr = NULL;
	u32 edge_v_val = 0x0;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	denoise3d_edge_v_addr = nr_dev->base + nr200_reg.DENOISE3D_EDGE_V;
	edge_v_val = readl(denoise3d_edge_v_addr);
	REG_SET_SLICE(edge_v_val, DENOISE3D_THR_EDGE_V_INV, thr_edge_v_inv);
	nr_write_dbg(edge_v_val, denoise3d_edge_v_addr);

	return SUCCESS;
}

static int set_range_s_inv(se_nr_dev *nr_dev, u32 range_s_inv)
{
	void __iomem *denoise3d_range_s_addr = NULL;
	u32 range_s_val = 0x0;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	denoise3d_range_s_addr = nr_dev->base + nr200_reg.DENOISE3D_RANGE_S;
	range_s_val = readl(denoise3d_range_s_addr);
	REG_SET_SLICE(range_s_val, DENOISE3D_RANGE_S_INV, range_s_inv);
	nr_write_dbg(range_s_val, denoise3d_range_s_addr);

	return SUCCESS;
}

static int set_range_t_h(se_nr_dev *nr_dev, u32 range_t_h)
{
	void __iomem *denoise3d_range_t_addr = NULL;
	u32 range_t_val = 0x0;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	denoise3d_range_t_addr = nr_dev->base + nr200_reg.DENOISE3D_RANGE_T;
	range_t_val = readl(denoise3d_range_t_addr);
	REG_SET_SLICE(range_t_val, DENOISE3D_RANGE_T_H, range_t_h);
	nr_write_dbg(range_t_val, denoise3d_range_t_addr);

	return SUCCESS;
}

static int set_range_t_v(se_nr_dev *nr_dev, u32 range_t_v)
{
	void __iomem *denoise3d_range_t_addr = NULL;
	u32 range_t_val = 0x0;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	denoise3d_range_t_addr = nr_dev->base + nr200_reg.DENOISE3D_RANGE_T;
	range_t_val = readl(denoise3d_range_t_addr);
	REG_SET_SLICE(range_t_val, DENOISE3D_RANGE_T_V, range_t_v);
	nr_write_dbg(range_t_val, denoise3d_range_t_addr);

	return SUCCESS;
}

static int set_range_t_inv(se_nr_dev *nr_dev, u32 range_t_inv)
{
	void __iomem *denoise3d_range_t_addr = NULL;
	u32 range_t_val = 0x0;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	denoise3d_range_t_addr = nr_dev->base + nr200_reg.DENOISE3D_RANGE_T;
	range_t_val = readl(denoise3d_range_t_addr);
	REG_SET_SLICE(range_t_val, DENOISE3D_RANGE_T_INV, range_t_inv);
	nr_write_dbg(range_t_val, denoise3d_range_t_addr);

	return SUCCESS;
}

static int set_range_d(se_nr_dev *nr_dev, u32 range_d)
{
	void __iomem *denoise3d_motion_addr = NULL;
	u32 motion_val = 0x0;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	denoise3d_motion_addr = nr_dev->base + nr200_reg.DENOISE3D_MOTION;
	motion_val = readl(denoise3d_motion_addr);
	REG_SET_SLICE(motion_val, DENOISE3D_RANGE_D, range_d);
	nr_write_dbg(motion_val, denoise3d_motion_addr);

	return SUCCESS;
}

static int set_motion_inv(se_nr_dev *nr_dev, u32 motion_inv)
{
	void __iomem *denoise3d_motion_addr = NULL;
	u32 motion_val = 0x0;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	denoise3d_motion_addr = nr_dev->base + nr200_reg.DENOISE3D_MOTION;
	motion_val = readl(denoise3d_motion_addr);
	REG_SET_SLICE(motion_val, DENOISE3D_MOTION_INV, motion_inv);
	nr_write_dbg(motion_val, denoise3d_motion_addr);

	return SUCCESS;
}

static int set_delta_t_inv(se_nr_dev *nr_dev, u32 delta_t_inv)
{
	void __iomem *denoise3d_delta_inv_addr = NULL;
	u32 delta_inv_val = 0x0;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	denoise3d_delta_inv_addr = nr_dev->base + nr200_reg.DENOISE3D_DELTA_INV;
	delta_inv_val = readl(denoise3d_delta_inv_addr);
	REG_SET_SLICE(delta_inv_val, DENOISE3D_DELTA_T_INV, delta_t_inv);
	nr_write_dbg(delta_inv_val, denoise3d_delta_inv_addr);

	return SUCCESS;
}

static int set_delta_h_inv(se_nr_dev *nr_dev, u32 delta_h_inv)
{
	void __iomem *denoise3d_delta_inv_addr = NULL;
	u32 delta_inv_val = 0x0;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	denoise3d_delta_inv_addr = nr_dev->base + nr200_reg.DENOISE3D_DELTA_INV;
	delta_inv_val = readl(denoise3d_delta_inv_addr);
	REG_SET_SLICE(delta_inv_val, DENOISE3D_DELTA_H_INV, delta_h_inv);
	nr_write_dbg(delta_inv_val, denoise3d_delta_inv_addr);

	return SUCCESS;
}

static int set_delta_v_inv(se_nr_dev *nr_dev, u32 delta_v_inv)
{
	void __iomem *denoise3d_delta_inv_addr = NULL;
	u32 delta_inv_val = 0x0;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	denoise3d_delta_inv_addr = nr_dev->base + nr200_reg.DENOISE3D_DELTA_INV;
	delta_inv_val = readl(denoise3d_delta_inv_addr);
	REG_SET_SLICE(delta_inv_val, DENOISE3D_DELTA_V_INV, delta_v_inv);
	nr_write_dbg(delta_inv_val, denoise3d_delta_inv_addr);

	return SUCCESS;
}

static int set_spatial_curve(se_nr_dev *nr_dev, u32 *curve_array, u32 size)
{
	int i = 0;
	int pos = 0;
	u32 reg_val = 0;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	for (i = 0; i < 6; i++) {
		reg_val = 0;
		pos = i * 3;
		if (i < 5) {
			REG_SET_SLICE(reg_val, DENOISE3D_SPACIAL_CURVE0,
				curve_array[pos + 0]);
			REG_SET_SLICE(reg_val, DENOISE3D_SPACIAL_CURVE1,
				curve_array[pos + 1]);
			REG_SET_SLICE(reg_val, DENOISE3D_SPACIAL_CURVE2,
				curve_array[pos + 2]);
		} else {
			REG_SET_SLICE(reg_val, DENOISE3D_SPACIAL_CURVE1,
				curve_array[pos + 0]);
			REG_SET_SLICE(reg_val, DENOISE3D_SPACIAL_CURVE2,
				curve_array[pos + 1]);
		}
		nr_write_dbg(reg_val, nr_dev->base + nr200_reg.DENOISE3D_CURVE_S_0 + 4 * i);
	}

	return SUCCESS;
}

static int set_temporal_curve(se_nr_dev *nr_dev, u32 *curve_array, u32 size)
{
	int i = 0;
	int pos = 0;
	u32 reg_val = 0;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	for (i = 0; i < 6; i++) {
		reg_val = 0;
		pos = i * 3;
		if (i < 5) {
			REG_SET_SLICE(reg_val, DENOISE3D_TEMPERAL_CURVE0,
				curve_array[pos + 0]);
			REG_SET_SLICE(reg_val, DENOISE3D_TEMPERAL_CURVE1,
				curve_array[pos + 1]);
			REG_SET_SLICE(reg_val, DENOISE3D_TEMPERAL_CURVE2,
				curve_array[pos + 2]);
		} else {
			REG_SET_SLICE(reg_val, DENOISE3D_TEMPERAL_CURVE1,
				curve_array[pos + 0]);
			REG_SET_SLICE(reg_val, DENOISE3D_TEMPERAL_CURVE2,
				curve_array[pos + 1]);
		}
		nr_write_dbg(reg_val, nr_dev->base + nr200_reg.DENOISE3D_CURVE_T_0 + 4 * i);
	}

	return SUCCESS;
}

int set_h_blank(se_nr_dev *nr_dev, u32 hblank)
{
	void __iomem *denoise3d_dummy_hblank_addr = NULL;
	u32 dummy_hblank_val = 0x0;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	denoise3d_dummy_hblank_addr = nr_dev->base + nr200_reg.DENOISE3D_DUMMY_HBLANK;
	dummy_hblank_val = readl(denoise3d_dummy_hblank_addr);
	REG_SET_SLICE(dummy_hblank_val, DENOISE3D_H_BLANK, hblank);
	nr_write_dbg(dummy_hblank_val, denoise3d_dummy_hblank_addr);

	return SUCCESS;
}

int mi_sp2_config_update(se_nr_dev *nr_dev)
{
	void __iomem *mi_ctrl_addr = NULL;
	u32 ctrl_val = 0x0;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	mi_ctrl_addr = nr_dev->base + nr200_reg.MI_SP2_CTRL;
	ctrl_val = readl(mi_ctrl_addr);
	REG_SET_SLICE(ctrl_val, SP2_RD_RAW_CFG_UPDATE, ENABLE); //bit9
	REG_SET_SLICE(ctrl_val, SP2_RD_RAW_AUTO_UPDATE, ENABLE);  //bit7
	REG_SET_SLICE(ctrl_val, SP2_MI_CFG_UPD, ENABLE);
	nr_write_dbg(ctrl_val, mi_ctrl_addr); //bit3

	return SUCCESS;
}

static u32 dnr_para[] = {
	0x0000001f, //CTRL
	0x1C204060, //STRENGTH
	0x01000900, //EDGE_H
	0x08000100, //EDGE_V
	0x00000800, //RANGE_S
	0x0A200A79, //RANGE_T
	0x00910000, //MOTION
	0x00000001, //DELTA_INV
	0x00000000, //CURVE_S_0
	0x00000000, //CURVE_S_1
	0x0003E3FF, //CURVE_S_2
	0x2D0005F0, //CURVE_S_3
	0x3FF473FF, //CURVE_S_4
	0x000783FF, //CURVE_S_5
	0x10400220, //CURVE_T_0
	0x3FFB33FF, //CURVE_T_1
	0x0C800000, //CURVE_T_2
	0x000883FF, //CURVE_T_3
	0x2E8FFCC8, //CURVE_T_4
	0x00000000, //CURVE_T_5
};

/*
 3DNR configuration: load 3DNR parameters(Strength, Motion, Delta).
 */
static void cisp_nr_load_denoise3d_params(se_nr_dev *nr_dev)
{
	int para_len = sizeof(dnr_para)/sizeof(u32);
	void __iomem *denoise3d_start_addr = NULL;
	int i = 0;
	u32 *padd = &nr200_reg.DENOISE3D_CTRL;

	for (i = 0; i < para_len; i++) {
		denoise3d_start_addr = nr_dev->base + *padd;
		nr_write_dbg(dnr_para[i], denoise3d_start_addr);
		padd++;
	}
}

/*
 3DNR configuration: set 3DNR parameters(Strength, Motion, Delta).
 */
int cisp_nr_denoise3d_set_params(se_nr_dev *nr_dev, denoise3d_params params)
{
	u32 update_spacial = params.update_spatial;
	u32 strength_curve_spacial = params.strength_curve_spatial;
	u32 thr_edge_v_inv = params.thr_edge_v_inv;
	u32 thr_edge_h_inv = params.thr_edge_h_inv;
	u32 thr_range_s_inv = params.thr_range_s_inv;
	/* temperal */
	u32 update_temperal = params.update_temperal;
	u32 strength_curve_temperal = params.strength_curve_temperal;
	u32 range_t_h = params.range_t_h;
	u32 range_t_v = params.range_t_v;
	u32 thr_range_t_inv = params.thr_range_t_inv;
	u32 thr_motion_inv = params.thr_motion_inv;
	u32 range_d = params.range_d;
	u32 thr_delta_h_inv = params.thr_delta_h_inv;
	u32 thr_delta_v_inv = params.thr_delta_v_inv;
	u32 thr_delta_t_inv = params.thr_delta_t_inv;
	u32 strength = params.strength;
	u32 h_blank = params.h_blank;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	strength = MIN(MAX(strength, 0), 128);
	params.strength = strength;

	set_strength_curve_spatial(nr_dev, strength_curve_spacial);  //set spatial curve

	set_thr_edge_h_inv(nr_dev, thr_edge_h_inv);//set horizontal edge detection

	set_strength_curve_temporal(nr_dev, strength_curve_temperal);  //set temporal curve

	set_thr_edge_v_inv(nr_dev, thr_edge_v_inv);//set vertical edge detection

	set_range_s_inv(nr_dev, thr_range_s_inv); //set spatial range calculation

	set_range_t_h(nr_dev, range_t_h); //set temporal horizontal range

	set_range_t_v(nr_dev, range_t_v); //set temporal vertical range

	set_range_t_inv(nr_dev, thr_range_t_inv); //set temporal range calculation

	set_range_d(nr_dev, range_d); //set dilate range

	set_motion_inv(nr_dev, thr_motion_inv); //set motion detection

	set_delta_h_inv(nr_dev, thr_delta_h_inv); //set spatial horizontal delta

	set_delta_v_inv(nr_dev, thr_delta_v_inv); // set spatial vertical delta

	set_delta_t_inv(nr_dev, thr_delta_t_inv); //set temporal delta

	/* spacial */
	/*
	 spatial_curve:
	 0, 255, 361, 443, 511, 572, 626, 676, 767, 809, 848, 886, 928, 960, 976, 992, 1023
	 temperal_curve:
	 0,   4,  16,  36,  64, 144, 195, 256, 324, 400, 484, 576, 672, 800, 928, 976, 1023
	 */
	set_spatial_curve(nr_dev, params.spacial_curve, 17);

	set_temporal_curve(nr_dev, params.temperal_curve, 17);

	//set strength
	set_update_temporal(nr_dev, update_temperal);//set temporal parameter, 132

	set_update_spatial(nr_dev, update_spacial); // set spatial parameter, 900

	set_strength(nr_dev, strength); //set general strength, 128

	set_h_blank(nr_dev, h_blank); //set h_blank

	nr_dev->params = params;
	//nr_write_dbg(0x1c238400, nr_dev->base + nr200_reg.DENOISE3D_STRENGTH_REG);
	return 0;

}

/*
 3DNR configuration: get 3DNR parameters(Strength, Motion, Delta).
 */
int cisp_nr_denoise3d_get_params(se_nr_dev *nr_dev, denoise3d_params *params)
{
	*params = nr_dev->params;

	return 0;
}

/*
 Set 3DNR enable(ISP_DENOISE3D_CTRL.denoise3d_enable[bit 0 = 1])
 */
int cisp_nr_denoise3d_enable(se_nr_dev *nr_dev, u32 enable)
{
	void __iomem *denoise3d_ctrl_addr = NULL;
	u32 denoise3d_ctrl_val = 0x00;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	enable = enable ? 1: 0;
	denoise3d_ctrl_addr = nr_dev->base + nr200_reg.DENOISE3D_CTRL;
	denoise3d_ctrl_val = readl(denoise3d_ctrl_addr);
	REG_SET_SLICE(denoise3d_ctrl_val, DENOISE3D_ENABLE, enable);
	nr_write_dbg(denoise3d_ctrl_val, denoise3d_ctrl_addr);

	nr_dev->enable_denoise3d = enable;

	return 0;
}

int cisp_nr_denoise3d_reset(se_nr_dev *nr_dev)
{
	void __iomem *denoise3d_ctrl_addr = NULL;
	u32 denoise3d_ctrl_val = 0x00;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	denoise3d_ctrl_addr = nr_dev->base + nr200_reg.DENOISE3D_CTRL;
	denoise3d_ctrl_val = readl(denoise3d_ctrl_addr);
	REG_SET_SLICE(denoise3d_ctrl_val, DENOISE3D_SOFT_RESET, 1);
	CIF_LOG(LOG_DEBUG, "reset denoise3d , denoise3d_ctrl -> %p = 0x%0x",
		denoise3d_ctrl_addr, denoise3d_ctrl_val);
	nr_write_dbg(denoise3d_ctrl_val, denoise3d_ctrl_addr);

	return 0;
}


/*
 Enable/disable NR200: ISP_CTRL.nr200_enable[bit 0 = 1]
 */
int cisp_nr_enable(se_nr_dev *nr_dev, u32 enable)
{
	void __iomem *isp_ctrl_addr = NULL;
	u32 isp_ctrl_val = 0x00;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	enable = enable ? 1 : 0;
	isp_ctrl_addr = nr_dev->base + nr200_reg.ISP_CTRL;
	isp_ctrl_val = readl(isp_ctrl_addr);
	REG_SET_SLICE(isp_ctrl_val, NR200_ISP_ENABLE, enable);
	nr_write_dbg(isp_ctrl_val, isp_ctrl_addr);
	nr_dev->nr200_enable = enable;

	return 0;
}

void cisp_nr_reg_dump(se_nr_dev *nr_dev)
{
	u32 reg_val = 0U;
	void* reg_base_addr = (void *)0;
	void* reg_addr = (void *)0;

	REPORT_FUNC();

	if (nr_dev->cpproc_clk_enable == DISABLE || nr_dev->isp_clk_enable == DISABLE) {
		cisp_nr_internal_clk_enable(nr_dev, ENABLE);
	}

	reg_base_addr = nr_dev->base;
	CIF_LOG(LOG_INFO,"+++++++++++++++++NR-%d register dump, base addr = %p+++++++++++++++", nr_dev->idx, reg_base_addr);
	reg_addr = reg_base_addr + nr200_reg.INTERNAL_CLK_CTRL;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.iinternal_clk_ctrl: 0x%0x = 0x%0x",
			nr200_reg.INTERNAL_CLK_CTRL, reg_val);

	reg_addr = reg_base_addr + nr200_reg.ISP_CTRL;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.isp_ctl: 0x%0x = 0x%0x", nr200_reg.ISP_CTRL, reg_val);

	reg_addr = reg_base_addr + nr200_reg.ISP_OUT_H_SIZE;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.isp_out_h_size: 0x%0x = 0x%0x",
			nr200_reg.ISP_OUT_H_SIZE, reg_val);

	reg_addr = reg_base_addr + nr200_reg.ISP_OUT_V_SIZE;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.isp_out_v_size: 0x%0x = 0x%0x",
			nr200_reg.ISP_OUT_V_SIZE, reg_val);

	reg_addr = reg_base_addr + nr200_reg.MI_CTRL;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.mi_ctrl: 0x%0x = 0x%0x", nr200_reg.MI_CTRL, reg_val);

	reg_addr = reg_base_addr + nr200_reg.MI_QOS;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.mi_qos: 0x%0x = 0x%0x", nr200_reg.MI_QOS, reg_val);

	reg_addr = reg_base_addr + nr200_reg.MI_SP2_CTRL;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.mi_sp2_ctrl: 0x%0x = 0x%0x", nr200_reg.MI_SP2_CTRL, reg_val);

	reg_addr = reg_base_addr + nr200_reg.MI_SP2_FMT;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.mi_sp2_fmt: 0x%0x = 0x%0x", nr200_reg.MI_SP2_FMT, reg_val);

	reg_addr = reg_base_addr + nr200_reg.MI_SP2_BUS_CFG;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.mi_sp2_bus_cfg: 0x%0x = 0x%0x",
			nr200_reg.MI_SP2_BUS_CFG, reg_val);

	reg_addr = reg_base_addr + nr200_reg.MI_SP2_BUS_ID;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.mi_sp2_bus_id: 0x%0x = 0x%0x", nr200_reg.MI_SP2_BUS_ID, reg_val);

	reg_addr = reg_base_addr + nr200_reg.MI_SP2_RAW_BASE_AD;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.mi_sp2_raw_base_ad: 0x%0x = 0x%0x",
			nr200_reg.MI_SP2_RAW_BASE_AD, reg_val);

	reg_addr = reg_base_addr + nr200_reg.MI_SP2_RAW_SIZE;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.mi_sp2_raw_size: 0x%0x = 0x%0x",
			nr200_reg.MI_SP2_RAW_SIZE, reg_val);

	reg_addr = reg_base_addr + nr200_reg.MI_SP2_RAW_OFFS_CNT;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.mi_sp2_raw_offs_cnt: 0x%0x = 0x%0x",
			nr200_reg.MI_SP2_RAW_OFFS_CNT, reg_val);

	reg_addr = reg_base_addr + nr200_reg.MI_SP2_RAW_LLENGTH;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.mi_sp2_raw_llength: 0x%0x = 0x%0x",
			nr200_reg.MI_SP2_RAW_LLENGTH, reg_val);

	reg_addr = reg_base_addr + nr200_reg.MI_SP2_RAW_PIC_WIDTH;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.mi_sp2_raw_pic_width: 0x%0x = 0x%0x",
			nr200_reg.MI_SP2_RAW_PIC_WIDTH, reg_val);

	reg_addr = reg_base_addr + nr200_reg.MI_SP2_RAW_PIC_HEIGHT;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.mi_sp2_raw_pic_height: 0x%0x = 0x%0x",
			nr200_reg.MI_SP2_RAW_PIC_HEIGHT, reg_val);

	reg_addr = reg_base_addr + nr200_reg.MI_SP2_RAW_PIC_SIZE;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.mi_sp2_raw_pic_size: 0x%0x = 0x%0x",
			nr200_reg.MI_SP2_RAW_PIC_SIZE, reg_val);

	reg_addr = reg_base_addr + nr200_reg.MI_SP2_DMA_RAW_PIC_START_AD;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.mi_sp2_dma_raw_pic_start_ad: 0x%0x = 0x%0x",
			nr200_reg.MI_SP2_DMA_RAW_PIC_START_AD, reg_val);

	reg_addr = reg_base_addr + nr200_reg.MI_SP2_DMA_RAW_PIC_WIDTH;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.mi_sp2_dma_raw_pic_width: 0x%0x = 0x%0x",
			nr200_reg.MI_SP2_DMA_RAW_PIC_WIDTH, reg_val);

	reg_addr = reg_base_addr + nr200_reg.MI_SP2_DMA_RAW_PIC_LLENGTH;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.mi_sp2_dma_raw_pic_llength: 0x%0x = 0x%0x",
			nr200_reg.MI_SP2_DMA_RAW_PIC_LLENGTH, reg_val);

		reg_addr = reg_base_addr + nr200_reg.MI_SP2_DMA_RAW_PIC_SIZE;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.mi_sp2_dma_raw_pic_size: 0x%0x = 0x%0x",
			nr200_reg.MI_SP2_DMA_RAW_PIC_SIZE, reg_val);

	reg_addr = reg_base_addr + nr200_reg.MI_SP2_DMA_RAW_PIC_LVAL;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.mi_sp2_dma_raw_pic_lval: 0x%0x = 0x%0x",
			nr200_reg.MI_SP2_DMA_RAW_PIC_LVAL, reg_val);

	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_CTRL;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_ctrl: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_CTRL, reg_val);

	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_STRENGTH_REG;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_stregth: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_STRENGTH_REG, reg_val);

	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_EDGE_H;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_edge_h: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_EDGE_H, reg_val);


	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_EDGE_V;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_edge_v: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_EDGE_V, reg_val);

	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_RANGE_S;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_range_s: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_RANGE_S, reg_val);


	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_RANGE_T;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_range_t: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_RANGE_T, reg_val);

	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_MOTION;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_motion: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_MOTION, reg_val);

	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_DELTA_INV;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_delta_inv: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_DELTA_INV, reg_val);

	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_CURVE_S_0;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_curve_s_0: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_CURVE_S_0, reg_val);

	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_CURVE_S_1;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_curve_s_1: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_CURVE_S_1, reg_val);

	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_CURVE_S_2;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_curve_s_2: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_CURVE_S_2, reg_val);

	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_CURVE_S_3;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_curve_s_3: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_CURVE_S_3, reg_val);

	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_CURVE_S_4;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_curve_s_4: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_CURVE_S_4, reg_val);

	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_CURVE_S_5;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_curve_s_5: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_CURVE_S_5, reg_val);

	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_CURVE_T_0;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_curve_t_0: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_CURVE_T_0, reg_val);

	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_CURVE_T_1;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_curve_t_1: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_CURVE_T_1, reg_val);

	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_CURVE_T_2;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_curve_t_2: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_CURVE_T_2, reg_val);

	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_CURVE_T_3;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_curve_t_3: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_CURVE_T_3, reg_val);

	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_CURVE_T_4;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_curve_t_4: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_CURVE_T_4, reg_val);

	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_CURVE_T_5;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_curve_t_5: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_CURVE_T_5, reg_val);

	reg_addr = reg_base_addr + nr200_reg.DENOISE3D_DUMMY_HBLANK;
	reg_val = readl(reg_addr);
	CIF_LOG(LOG_INFO, "nr.denoise3d_dummy_hblank: 0x%0x = 0x%0x",
			nr200_reg.DENOISE3D_DUMMY_HBLANK, reg_val);
	CIF_LOG(LOG_INFO,"+++++++++++++++++NR%d register dump end+++++++++++++++", nr_dev->idx);
	/*
	printf("======================================================\n");
	for (int i = 0; i < 674; i++) {
		void *reg_addr = nr_dev->base + 4 * i;
		CIF_LOG(LOG_INFO, "0x%p ======> 0x%x", reg_addr, readl(reg_addr));
	}
	printf("=====================================================\n");
	*/
}

int cisp_nr_start_rdma(se_nr_dev *nr_dev)
{
	u32 mi_ctrl_val = readl(nr_dev->base + nr200_reg.MI_CTRL);

	//sp2_raw_rdma_start
	REG_SET_SLICE(mi_ctrl_val, SP2_RAW_RDMA_START, 1);
	nr_write_dbg(mi_ctrl_val, nr_dev->base + nr200_reg.MI_CTRL);
	REG_SET_SLICE(mi_ctrl_val, SP2_RAW_RDMA_START, 0);
	nr_write_dbg(mi_ctrl_val, nr_dev->base + nr200_reg.MI_CTRL);

	return SUCCESS;
}

int cisp_nr_init(se_nr_dev *nr_dev)
{
	void __iomem *nr_base = nr_dev->base;

	cisp_nr_internal_clk_enable(nr_dev, ENABLE);

	CIF_LOG(LOG_INFO, "nr init %d, base 0x%llx", nr_dev->idx, (uint64_t)(nr_dev->base));
	CIF_LOG(LOG_INFO, "customerID: 0x%0x\n", readl(nr_base + 0x004));
	CIF_LOG(LOG_INFO, "project ID: 0x%0x\n", readl(nr_base + 0x008));
	CIF_LOG(LOG_INFO, "product ID: 0x%0x\n", readl(nr_base + 0x00C));

	//[ISP]set vivante NR200 to idle state
	cisp_nr_set_idle(nr_dev);

	cisp_nr_set_bayer_pattern(nr_dev, nr_dev->pattern);

	cisp_nr_set_rdma_mode(nr_dev, nr_dev->rdma_mode);

	//[ISP]3DNR configuration
	cisp_nr_set_size(nr_dev, nr_dev->out_h_size, nr_dev->out_v_size);

	cisp_nr_denoise3d_horizontal_enable(nr_dev, ENABLE);

	cisp_nr_denoise3d_vertical_enable(nr_dev, ENABLE);

	cisp_nr_denoise3d_temperal_enable(nr_dev, ENABLE);

	cisp_nr_denoise3d_dilate_enable(nr_dev, ENABLE);

	cisp_nr_denoise3d_set_params(nr_dev, nr_dev->params);

	cisp_nr_load_denoise3d_params(nr_dev);

	//[ISP]set 3DNR enable]
	cisp_nr_denoise3d_enable(nr_dev, ENABLE);

	//[ISP]Immediate ISP configuration update
	cisp_nr_update_config(nr_dev);

	//[MI]Self Path 2 raw write memory buffer configuraton
	cisp_nr_sp2_wr_memory_config(nr_dev);

	//[MI] Self Path 2 raw read memory buffer configuration
	cisp_nr_sp2_rd_memory_config(nr_dev);

	//[ISP] Enable/disable interrupts

	//[ISP] Enable NR200
	cisp_nr_enable(nr_dev, ENABLE);

	//[ISP]Immediate ISP configuration update
	cisp_nr_update_config(nr_dev);
	return 0;
}

void cisp_nr_set_default(se_nr_dev *nr_dev)
{
	nr_dev->init = 0;
	nr_dev->nr200_enable = DISABLE;
	nr_dev->cpproc_clk_enable = DISABLE;
	nr_dev->isp_clk_enable = DISABLE;

	nr_dev->pattern = RGRG_GBGB;
	nr_dev->out_h_size = 1920;
	nr_dev->out_v_size = 1080;
	nr_dev->base = NULL;
	nr_dev->virt_rf_rd_addr = NULL;
	nr_dev->virt_rf_wr_addr = NULL;
	nr_dev->rf_rd_addr = 0x00;
	nr_dev->rf_wr_addr = 0x00;

	nr_dev->rdma_mode = NORMAL_MODE;

	nr_dev->enable_denoise3d = DISABLE;

	nr_dev->params.enable_dilate = DISABLE;
	nr_dev->params.enable_temperal = DISABLE;
	nr_dev->params.enable_h = DISABLE;
	nr_dev->params.enable_v = DISABLE;

	nr_dev->params.strength = 100;
	nr_dev->params.h_blank = 0x80;

	//spacial
	nr_dev->params.update_spatial = 900;
	nr_dev->params.strength_curve_spatial = 64;
	nr_dev->params.thr_edge_v_inv = 1024;
	nr_dev->params.thr_edge_h_inv = 1024;
	nr_dev->params.thr_range_s_inv = 1024;

	//temperal
	nr_dev->params.update_temperal = 900;
	nr_dev->params.strength_curve_temperal = 64;
	nr_dev->params.range_t_h = 1;
	nr_dev->params.range_t_v = 1;
	nr_dev->params.thr_range_t_inv = 1024;
	nr_dev->params.thr_motion_inv = 1024;
	nr_dev->params.range_d = 1;
	nr_dev->params.thr_delta_t_inv = 1023;
	nr_dev->params.thr_delta_h_inv = 1023;
	nr_dev->params.thr_delta_v_inv = 1023;
}

