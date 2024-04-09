// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021-2022 Siengine Technology, Inc.
 *
 * Sienginie Camera Interface driver
 *
 * Author: Siengine Technology, Inc.
 */
#include "se-cif-hw.h"
#include "dw-mipi-csi.h"
void dump_cif_regs(struct se_cif_dev *se_cif)
{
	struct device *dev = &se_cif->pdev->dev;

	dev_dbg(dev, "CIF register dump, cif%d\n", se_cif->id);
	dev_dbg(dev, "CIF_CLOCK_EN                  0x0h  = 0x%8x\n", readl(se_cif->regs + 0x0));
	dev_dbg(dev, "CIF_STATUS                    0x4h  = 0x%8x\n", readl(se_cif->regs + 0x4));
	dev_dbg(dev, "CIF_SETTING                   0x8h  = 0x%8x\n", readl(se_cif->regs + 0x8));
	dev_dbg(dev, "CIF_SOFT_RESET                0xCh  = 0x%8x\n", readl(se_cif->regs + 0xC));
	dev_dbg(dev, "CIF_THRESHOLD                 0x10h = 0x%8x\n", readl(se_cif->regs + 0x10));
	dev_dbg(dev, "CIF_COEF_1                    0x14h = 0x%8x\n", readl(se_cif->regs + 0x14));
	dev_dbg(dev, "CIF_COEF_2                    0x18h = 0x%8x\n", readl(se_cif->regs + 0x18));
	dev_dbg(dev, "CIF_COEF_3                    0x1Ch = 0x%8x\n", readl(se_cif->regs + 0x1C));
	dev_dbg(dev, "CIF_COEF_4                    0x20h = 0x%8x\n", readl(se_cif->regs + 0x20));
	dev_dbg(dev, "CIF_COEF_5                    0x24h = 0x%8x\n", readl(se_cif->regs + 0x24));
	dev_dbg(dev, "CIF_BUFFER_BASE_ADDRESS_A     0x28h = 0x%8x\n", readl(se_cif->regs + 0x28));
	dev_dbg(dev, "CIF_BUFFER_BASE_ADDRESS_B     0x2Ch = 0x%8x\n", readl(se_cif->regs + 0x2C));
	dev_dbg(dev, "CIF_GEOMETRY                  0x30h = 0x%8x\n", readl(se_cif->regs + 0x30));
	dev_dbg(dev, "CIF_INT_SOURCE                0x34h = 0x%8x\n", readl(se_cif->regs + 0x34));
	dev_dbg(dev, "CIF_INT_MASK                  0x38h = 0x%8x\n", readl(se_cif->regs + 0x38));
	dev_dbg(dev, "CIF_INT_STATUS                0x3Ch = 0x%8x\n", readl(se_cif->regs + 0x3C));
	dev_dbg(dev, "CIF_INT_CLEAR                 0x40h = 0x%8x\n", readl(se_cif->regs + 0x40));
	dev_dbg(dev, "CIF_IPI_SELECT                0x44h = 0x%8x\n", readl(se_cif->regs + 0x44));
}

/*
 *se_rgbir422_enable - enable RGBIr function ,only valid in CIF 18, 19, 20, 21
 *@reg_base: the base address of CIF
 *should call before streamon
 */
int se_rgbir422_enable(struct se_cif_dev *se_cif, bool enable)
{
	if (se_cif->id < 18) {
		dev_err(&se_cif->pdev->dev,
			"set cif %d rgbir enable fail, channel 0-17 has no rgbir moudle\n",
			se_cif->id);
		return -1;
	}

	se_cif->cif_cap.se_rgbir422_enable = true;
	return 0;
}

/*
 *se_rgbir422_set_params - set rgbir422 params, only valid in CIF 18, 19, 20, 21
 *@reg_base: the base address of CIF
 *should call before streamon
 */
int se_rgbir422_set_params(struct se_cif_dev *se_cif,
		rgbir422_coef_param *rgbir422_param)
{
	rgbir422_coef_param *params = &(se_cif->cif_cap.rgbir422_param);

	if (se_cif->id < 18) {
		dev_err(&se_cif->pdev->dev,
			"set cif %d rgbir params fail, channel 0-17 has no rgbir module\n",
			se_cif->id);
		return -1;
	}

	params->reg_avg_i_mode = rgbir422_param->reg_avg_i_mode;
	params->reg_delta_offset_R = rgbir422_param->reg_delta_offset_R;
	params->reg_delta_offset_B = rgbir422_param->reg_delta_offset_B;
	params->reg_delta_offset_Ir = rgbir422_param->reg_delta_offset_Ir;
	params->reg_delta_dn_shiftbit_R = rgbir422_param->reg_delta_dn_shiftbit_R;
	params->reg_delta_dn_shiftbit_B = rgbir422_param->reg_delta_dn_shiftbit_B;
	params->reg_delta_dn_shiftbit_Ir = rgbir422_param->reg_delta_dn_shiftbit_Ir;

	params->reg_delta_gain_R = rgbir422_param->reg_delta_gain_R;
	params->reg_delta_gain_B = rgbir422_param->reg_delta_gain_B;
	params->reg_delta_gain_Ir = rgbir422_param->reg_delta_gain_Ir;

	params->array_R.delta_coef_center = rgbir422_param->array_R.delta_coef_center;
	params->array_R.delta_coef_midcircle = rgbir422_param->array_R.delta_coef_midcircle;
	params->array_R.delta_coef_outercenter = rgbir422_param->array_R.delta_coef_outercenter;

	params->array_B.delta_coef_center = rgbir422_param->array_B.delta_coef_center;
	params->array_B.delta_coef_midcircle = rgbir422_param->array_B.delta_coef_midcircle;
	params->array_B.delta_coef_outercenter = rgbir422_param->array_B.delta_coef_outercenter;

	params->array_Ir.delta_coef_center = rgbir422_param->array_Ir.delta_coef_center;
	params->array_Ir.delta_coef_midcircle = rgbir422_param->array_Ir.delta_coef_midcircle;
	params->array_Ir.delta_coef_outercenter = rgbir422_param->array_Ir.delta_coef_outercenter;

	return 0;
}

int se_rgbir422_get_params(struct se_cif_dev *se_cif, rgbir422_coef_param *rgbir422_param)
{
	rgbir422_coef_param *params = &(se_cif->cif_cap.rgbir422_param);

	if (se_cif->id < 18) {
		dev_err(&se_cif->pdev->dev,
			"cif %d rgbir params fail, channel 0-17 has no rgbir module\n",
			se_cif->id);
		return -1;
	}

	rgbir422_param->reg_avg_i_mode = params->reg_avg_i_mode;
	rgbir422_param->reg_delta_offset_R = params->reg_delta_offset_R;
	rgbir422_param->reg_delta_offset_B = params->reg_delta_offset_B;
	rgbir422_param->reg_delta_offset_Ir = params->reg_delta_offset_Ir;
	rgbir422_param->reg_delta_dn_shiftbit_R = params->reg_delta_dn_shiftbit_R;
	rgbir422_param->reg_delta_dn_shiftbit_B = params->reg_delta_dn_shiftbit_B;
	rgbir422_param->reg_delta_dn_shiftbit_Ir = params->reg_delta_dn_shiftbit_Ir;

	rgbir422_param->reg_delta_gain_R = params->reg_delta_gain_R;
	rgbir422_param->reg_delta_gain_B = params->reg_delta_gain_B;
	rgbir422_param->reg_delta_gain_Ir = params->reg_delta_gain_Ir;

	rgbir422_param->array_R.delta_coef_center = params->array_R.delta_coef_center;
	rgbir422_param->array_R.delta_coef_midcircle = params->array_R.delta_coef_midcircle;
	rgbir422_param->array_R.delta_coef_outercenter = params->array_R.delta_coef_outercenter;

	rgbir422_param->array_B.delta_coef_center = params->array_B.delta_coef_center;
	rgbir422_param->array_B.delta_coef_midcircle = params->array_B.delta_coef_midcircle;
	rgbir422_param->array_B.delta_coef_outercenter = params->array_B.delta_coef_outercenter;

	rgbir422_param->array_Ir.delta_coef_center = params->array_Ir.delta_coef_center;
	rgbir422_param->array_Ir.delta_coef_midcircle = params->array_Ir.delta_coef_midcircle;
	rgbir422_param->array_Ir.delta_coef_outercenter = params->array_Ir.delta_coef_outercenter;

	return 0;
}

void se_cif_channel_set_outbuf(struct se_cif_dev *se_cif, struct se_cif_buffer *buf)
{
	struct vb2_buffer *vb2_buf = &buf->v4l2_buf.vb2_buf;
	struct frame_addr *paddr = &buf->paddr;
	u8 buf_id = se_cif->buf.id;
	switch (buf_id) {
	case SE_CIF_BUFA:
		if (buf->discard) {
			dev_dbg(&se_cif->pdev->dev, "buf->discard is true\n");
			paddr->a = se_cif->discard_buffer_dma[0];
			dev_dbg(&se_cif->pdev->dev, "paddr->a: 0x%8x\n", paddr->a);
			writel((paddr->a) >> 4, se_cif->regs + CIF_BUFFER_BASE_ADDRESS_A);
		}
		else
		{
			paddr->a = vb2_dma_contig_plane_dma_addr(vb2_buf, 0) + vb2_buf->planes[0].data_offset - CIF_BUFFER_HEADER_LEN;
			dev_dbg(&se_cif->pdev->dev, "paddr->a: 0x%8x\n", paddr->a);
			writel((paddr->a) >> 4, se_cif->regs + CIF_BUFFER_BASE_ADDRESS_A);
		}
		break;
	case SE_CIF_BUFB:
		if (buf->discard) {
			dev_dbg(&se_cif->pdev->dev, "buf->discard is true\n");
			paddr->b = se_cif->discard_buffer_dma[0];
			dev_dbg(&se_cif->pdev->dev, "paddr->b: 0x%8x\n", paddr->b);
			writel((paddr->b) >> 4, se_cif->regs + CIF_BUFFER_BASE_ADDRESS_B);
		}
		else
		{
			paddr->b = vb2_dma_contig_plane_dma_addr(vb2_buf, 0) + vb2_buf->planes[0].data_offset - CIF_BUFFER_HEADER_LEN;
			dev_dbg(&se_cif->pdev->dev, "paddr->b: 0x%8x\n", paddr->b);
			writel((paddr->b) >> 4, se_cif->regs + CIF_BUFFER_BASE_ADDRESS_B);
		}
		break;
	default:
		break;
	}
}

void se_cif_channel_set_settings(struct se_cif_dev *se_cif, struct se_cif_fmt *f)
{
	u32 val = 0;

	if(f->color == CIF_OUT_FMT_RAW12 && se_cif->cif_cap.se_rgbir422_enable) {
		dev_info(&se_cif->pdev->dev, "set CIF_OUT_FMT_RAW12, RGBIr422 enable\n");
		val = (CIF_SETTING_FORMAT_RAW << CIF_SETTING_FORMAT_OFFSET)
				| (CIF_SETTING_PIXEL_WIDTH_12B << CIF_SETTING_PIXEL_WIDTH_OFFSET)
				| (CIF_SETTING_RGB422_CONV_ENABLE << CIF_SETTING_RGB422_CONV_OFFSET)
				| (CIF_SETTING_IPI16 << CIF_SETTING_IPI48_OFFSET);
	} else if(f->color == CIF_OUT_FMT_RAW12 && se_cif->cif_cap.se_nr_enable) {
		dev_info(&se_cif->pdev->dev, "set CIF_OUT_FMT_RAW12, NR enable\n");
		val = (CIF_SETTING_FORMAT_RAW << CIF_SETTING_FORMAT_OFFSET)
				| (CIF_SETTING_PIXEL_WIDTH_12B << CIF_SETTING_PIXEL_WIDTH_OFFSET)
				| (CIF_SETTING_3DNR_ENABLE << CIF_SETTING_3DNR_OFFSET)
				| (CIF_SETTING_IPI16 << CIF_SETTING_IPI48_OFFSET);
	} else if(f->color == CIF_OUT_FMT_RAW12) {
		dev_info(&se_cif->pdev->dev, "set CIF_OUT_FMT_RAW12\n");
		val = (CIF_SETTING_FORMAT_RAW << CIF_SETTING_FORMAT_OFFSET)
				| (CIF_SETTING_PIXEL_WIDTH_12B << CIF_SETTING_PIXEL_WIDTH_OFFSET)
				| (CIF_SETTING_IPI48 << CIF_SETTING_IPI48_OFFSET);
	} else if(f->color == CIF_OUT_FMT_RAW8) {
		dev_info(&se_cif->pdev->dev, "set CIF_OUT_FMT_RAW8\n");
		val = (CIF_SETTING_FORMAT_RAW << CIF_SETTING_FORMAT_OFFSET)
				| (CIF_SETTING_PIXEL_WIDTH_8B << CIF_SETTING_PIXEL_WIDTH_OFFSET)
				| (CIF_SETTING_IPI48 << CIF_SETTING_IPI48_OFFSET);
	} else if (f->color == CIF_OUT_FMT_ARGB8888) {
		dev_info(&se_cif->pdev->dev, "set CIF_OUT_FMT_ARGB8888\n");
		val = (CIF_SETTING_FORMAT_RGB888 << CIF_SETTING_FORMAT_OFFSET)
				| (CIF_SETTING_PIXEL_WIDTH_8B << CIF_SETTING_PIXEL_WIDTH_OFFSET)
				| (CIF_SETTING_IPI48 << CIF_SETTING_IPI48_OFFSET);
	} else if (f->color == CIF_OUT_FMT_YUV422) {
		dev_info(&se_cif->pdev->dev, "set CIF_OUT_FMT_YUV422\n");
		val = (CIF_SETTING_FORMAT_YUV422 << CIF_SETTING_FORMAT_OFFSET)
				| (CIF_SETTING_PIXEL_WIDTH_8B << CIF_SETTING_PIXEL_WIDTH_OFFSET)
				| (CIF_SETTING_IPI48 << CIF_SETTING_IPI48_OFFSET);
	}
	writel(val, se_cif->regs + CIF_SETTING);
}

void se_cif_channel_set_geometry(struct se_cif_dev *se_cif, struct se_cif_frame *f)
{

	u32 val;
	dev_dbg(&se_cif->pdev->dev, "%s: hw_width=%d, height=%d\n", __func__, f->hw_width, f->height);
	val = (f->hw_width << CIF_GEOMETRY_ACTIVE_WIDTH_OFFSET) |\
		  (f->height << CIF_GEOMETRY_ACTIVE_HEIGHT_OFFSET);
	writel(val, se_cif->regs + CIF_GEOMETRY);
}

void se_cif_ipi_select(struct se_cif_dev *se_cif, u32 val)
{
	u32 bit = 0;
	val = 0;

	if (se_cif->id >= CIF_CONFIGABLE_MINI_CHANNLE_NUM) {
		bit = se_cif->interface[0]*MAX_VC_NUM + se_cif->interface[1];
		dev_dbg(&se_cif->pdev->dev, "%s: se_cif id=%d, bit=%d\n",
			__func__, se_cif->id, bit);
		val = 0x01 << bit;
		writel(val,se_cif->regs + CIF_IPI_SELECT);
	}
}

void se_cif_channel_set_threshold(struct se_cif_dev *se_cif, int threshold)
{
	/*Need update*/
	return;
}

void se_cif_channel_set_coef(struct se_cif_dev *se_cif)
{
	/*Need update*/
	return;
}

int se_cif_channel_is_working(struct se_cif_dev *se_cif)
{
	u32 val;

	val = readl(se_cif->regs + CIF_STATUS);
	val &= CIF_STATUS_WORKING_MASK;
	if (val == CIF_STATUS_WORKING_ENABLED << CIF_STATUS_WORKING_OFFSET)
		return 1;

	return 0;
}

 void se_cif_dm_config(struct se_cif_dev *se_cif)
 {
	u32 id = se_cif->id;
	u32 n = 0;

	if (id <18)
		n = id / 3;
	else if (id < 20)
		n = 6;
	else if (id < 22)
		n = 7;
	else
		dev_info(&se_cif->pdev->dev, "wrong channel number %d\n", id);

	writel(1 << n, se_cif->dm_regs);

	dev_info(&se_cif->pdev->dev, "configure DM, val=0x%x.\n", 1 << n);
 }

void se_cif_clock_enable(struct se_cif_dev *se_cif)
{
	u32 val = 0;

	val |= (CIF_CLOCK_EN_ENABLE << CIF_CLOCK_EN_OFFSET);
	writel(val, se_cif->regs + CIF_CLOCK_EN);
	//mdelay(5);
}

void se_cif_channel_softreset(struct se_cif_dev *se_cif)
{
	u32 val = 0;

	dev_info(&se_cif->pdev->dev, "cif channel %d soft-reset\n", se_cif->id);

	/* starting to do soft reset */
	val |= (CIF_SOFT_RESET_EXECUTE << CIF_SOFT_RESET_OFFSET);
	writel(val, se_cif->regs + CIF_SOFT_RESET);
	/* Check if complete data transmission */
#if 0
	while(se_cif_channel_is_working(se_cif->id))
			mdelay(5);
#endif
	val = readl(se_cif->regs + CIF_STATUS);

	dev_dbg(&se_cif->pdev->dev, "STATUS=0x%2x\n", val);
}

void se_cif_channel_start_working(struct se_cif_dev *se_cif)
{
	u32 val = 0;

	dev_info(&se_cif->pdev->dev, "enter %s\n", __func__);

	val |= (CIF_STATUS_WORKING_ENABLED << CIF_STATUS_WORKING_OFFSET);
	writel(val, se_cif->regs + CIF_STATUS);
}

void se_cif_clock_disable(struct se_cif_dev *se_cif)
{
	u32 val = 0;

	dev_info(&se_cif->pdev->dev, "enter %s\n", __func__);

	val |= (CIF_CLOCK_EN_DISABLE << CIF_CLOCK_EN_OFFSET);
	writel(val, se_cif->regs + CIF_CLOCK_EN);
	mdelay(5);
}

void se_cif_channel_disable(struct se_cif_dev *se_cif)
{
	u32 val = 0;
	u8 timeout = 0;

	dev_info(&se_cif->pdev->dev, "enter %s\n", __func__);

	/* starting to do soft reset */
	val |= (CIF_SOFT_RESET_EXECUTE << CIF_SOFT_RESET_OFFSET);
	writel(val, se_cif->regs + CIF_SOFT_RESET);
	//mdelay(5);
	/* Check if complete data transmission */

	while(se_cif_channel_is_working(se_cif) && (timeout < 50)) {
		msleep(1);
		timeout++;
	}

	/* Disable clock */
	se_cif_clock_disable(se_cif);
}


void se_cif_channel_config(struct se_cif_dev *se_cif)
{
	struct se_cif_frame *src_f = &se_cif->cif_cap.src_f;
	struct se_cif_fmt *fmt = src_f->fmt;

	dev_dbg(&se_cif->pdev->dev, "enter %s\n", __func__);

	/* set Geometry */
	dev_dbg(&se_cif->pdev->dev, "se_cif_channel_set_geometry\n");
	se_cif_channel_set_geometry(se_cif,src_f);

	/* set setting */
	dev_dbg(&se_cif->pdev->dev, "se_cif_channel_set_settings\n");
	se_cif_channel_set_settings(se_cif, fmt);

	/* set coef when channel > 17 */
	if (se_cif->id > CIF_CONFIG_COEF_MINI_ID)
	{
		se_cif_channel_set_coef(se_cif);
	}

	/* set ipi select */
	se_cif_ipi_select(se_cif, CIF_DEFAULT_CONFIG_CHANNLE_NUM);
	dev_dbg(&se_cif->pdev->dev, "dump_cif_regs\n");
	//dump_cif_regs(se_cif);
}

void  se_cif_enable_irq(struct se_cif_dev *se_cif)
{
	dev_dbg(&se_cif->pdev->dev, "enter %s\n", __func__);
	writel(0x3f, se_cif->regs + CIF_INT_MASK);
}

void se_cif_disable_irq(struct se_cif_dev *se_cif)
{
	writel(0, se_cif->regs + CIF_INT_MASK);
}

u32 se_cif_get_irq_status(struct se_cif_dev *se_cif)
{
	return readl(se_cif->regs + CIF_INT_STATUS);
}

void se_cif_clean_irq_status(struct se_cif_dev *se_cif, u32 val)
{
	writel(val, se_cif->regs + CIF_INT_CLEAR);
}

void se_cif_clean_irq_status_bk(struct se_cif_dev *se_cif, u32 val)
{
	writel(val, se_cif->regs + CIF_INT_CLEAR);
}

u32 se_cif_read_buffer_act_a(struct se_cif_dev *se_cif)
{
	return readl(se_cif->regs + CIF_BUFFER_BASE_ADDRESS_ACT_A);
}

u32 se_cif_read_buffer_act_b(struct se_cif_dev *se_cif)
{
	return readl(se_cif->regs + CIF_BUFFER_BASE_ADDRESS_ACT_B);
}
