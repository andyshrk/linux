// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021-2022 Synopsys, Inc.
 *
 * Sienginie Camera Interface driver
 *
 * Author: Siengine Technology, Inc.
 */
#ifndef SE_CIF_HW_H_
#define SE_CIF_HW_H_

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/bug.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>

#include "se-cif-core.h"

#define CIF_BUFFER_HEADER_LEN					0x20
#define CIF_DATA_OFFSET						0x40
/*
 *CIF Registers Define
*/
#define CIF_DEVICE_MANAGER 0xE8020000
/* Clock Controller Register */
#define CIF_CLOCK_EN                                            0x0
#define CIF_CLOCK_EN_OFFSET                                     0
#define CIF_CLOCK_EN_MASK                                       0x1
#define CIF_CLOCK_EN_ENABLE                                     1
#define CIF_CLOCK_EN_DISABLE                                    0

/* CIF Channel Status Register */
#define CIF_STATUS                                              0x4
#define CIF_STATUS_WORKING_OFFSET                               0
#define CIF_STATUS_WORKING_MASK                                 0x1
#define CIF_STATUS_WORKING_ENABLED                              1
#define CIF_STATUS_WORKING_DISABLED                             0
#define CIF_STATUS_BUFFER_OFFSET                                1
#define CIF_STATUS_BUFFER_MASK                                  0x2
#define CIF_STATUS_BUFFER_WR_A                                  0
#define CIF_STATUS_BUFFER_WR_B                                  1

/* CIF Channel Setting Register */
#define CIF_SETTING                                             0x8
#define CIF_SETTING_FORMAT_OFFSET                               0
#define CIF_SETTING_FORMAT_MASK                                 0x3
#define CIF_SETTING_FORMAT_RAW                                  0
#define CIF_SETTING_FORMAT_RGB888                               1
#define CIF_SETTING_FORMAT_YUV422                               2
#define CIF_SETTING_PIXEL_WIDTH_OFFSET                          2
#define CIF_SETTING_PIXEL_WIDTH_MASK                            0xFC
#define CIF_SETTING_PIXEL_WIDTH_8B                              8
#define CIF_SETTING_PIXEL_WIDTH_10B                             10
#define CIF_SETTING_PIXEL_WIDTH_12B                             12
#define CIF_SETTING_PIXEL_WIDTH_14B                             14
#define CIF_SETTING_PIXEL_WIDTH_16B                             16
#define CIF_SETTING_RGB422_CONV_OFFSET                          8
#define CIF_SETTING_RGB422_CONV_MASK                            0x100
#define CIF_SETTING_RGB422_CONV_ENABLE                          1
#define CIF_SETTING_RGB422_CONV_DISABLE                         0
#define CIF_SETTING_3DNR_OFFSET                                 16
#define CIF_SETTING_3DNR_ENABLE                                 1
#define CIF_SETTING_3DNR_DISABLE                                0
#define CIF_SETTING_3DNR_MASK                                   0x10000

#define CIF_SETTING_IPI48_OFFSET                                20
#define CIF_SETTING_IPI48_MASK                                  0x100000
#define CIF_SETTING_IPI16                                       0
#define CIF_SETTING_IPI48                                       1

/* CIF Channel Soft Rest Register */
#define CIF_SOFT_RESET                                          0xC
#define CIF_SOFT_RESET_OFFSET                                   0
#define CIF_SOFT_RESET_EXECUTE                                  1

/* CIF Channel Threshold Register */
#define CIF_THRESHOLD                                           0x10
#define CIF_THRH_BURST_LEN_OFFSET                               0
#define CIF_THRH_BURST_LEN_MASK                                 0xFF
#define CIF_THRH_10_OFFSET                                      8
#define CIF_THRH_10_MASK                                        0xFF00
#define CIF_THRH_100_OFFSET                                     16
#define CIF_THRH_100_MASK                                       0xFF0000
#define CIF_THRH_1111_OFFSET                                    24
#define CIF_THRH_1111_MASK                                      0xFF000000

/* CIF Channel Coef 1 Register */
#define CIF_COEF_1                                              0x14
#define CIF_COEF_1_REG_AVG_IMODE_OFFSET                         0
#define CIF_COEF_1_REG_AVG_IMODE_MASK                           0x1
#define CIF_COEF_1_REG_DELTA_OFFSET_R_OFFSET                    1
#define CIF_COEF_1_REG_DELTA_OFFSET_R_MASK                      0xFE
#define CIF_COEF_1_REG_DELTA_OFFSET_B_OFFSET                    8
#define CIF_COEF_1_REG_DELTA_OFFSET_B_MASK                      0x7F00
#define CIF_COEF_1_REG_DELTA_OFFSET_IR_OFFSET                   15
#define CIF_COEF_1_REG_DELTA_OFFSET_IR_MASK                     0x3F8000
#define CIF_COEF_1_REG_DELTA_DN_SH_BIT_R_OFFSET                 23
#define CIF_COEF_1_REG_DELTA_DN_SH_BIT_R_MASK                   0x3800000
#define CIF_COEF_1_REG_DELTA_DN_SH_BIT_B_OFFSET                 26
#define CIF_COEF_1_REG_DELTA_DN_SH_BIT_B_MASK                   0x1C000000
#define CIF_COEF_1_REG_DELTA_DN_SH_BIT_IR_OFFSET                26
#define CIF_COEF_1_REG_DELTA_DN_SH_BIT_IR_MASK                  0xE0000000

/* CIF Channel Coef 2 Register */
#define CIF_COEF_2                                              0x18
#define CIF_COEF_2_REG_DELTA_GAIN_R_OFFSET                      0
#define CIF_COEF_2_REG_DELTA_GAIN_R_MASK                        0xFF
#define CIF_COEF_2_REG_DELTA_GAIN_B_OFFSET                      8
#define CIF_COEF_2_REG_DELTA_GAIN_B_MASK                        0xFF00
#define CIF_COEF_2_REG_DELTA_GAIN_IR_OFFSET                     16
#define CIF_COEF_2_REG_DELTA_GAIN_IR_MASK                       0xFF0000

/* CIF Channel Coef 3 Register */
#define CIF_COEF_3                                              0x1C
#define CIF_COEFF_3_DELTA_COEF_CENTER_R_OFFSET                  9
#define CIF_COEFF_3_DELTA_COEF_CENTER_R_MASK                    0xFE00
#define CIF_COEFF_3_DELTA_COEF_MIDCIRCLE_R_OFFSET               16
#define CIF_COEFF_3_DELTA_COEF_MIDCIRCLE_R_MASK                 0xF0000
#define CIF_COEFF_3_DELTA_COEF_OUTERCIRCLE_R_OFFSET             20
#define CIF_COEFF_3_DELTA_COEF_OUTERCIRCLE_R_MASK               0x700000

/* CIF Channel Coef 4 Register */
#define CIF_COEF_4                                              0x20
#define CIF_COEFF_4_DELTA_COEF_CENTER_B_OFFSET                  9
#define CIF_COEFF_4_DELTA_COEF_CENTER_B_MASK                    0xFE00
#define CIF_COEFF_4_DELTA_COEF_MIDCIRCLE_B_OFFSET               16
#define CIF_COEFF_4_DELTA_COEF_MIDCIRCLE_B_MASK                 0xF0000
#define CIF_COEFF_4_DELTA_COEF_OUTERCIRCLE_B_OFFSET             20
#define CIF_COEFF_4_DELTA_COEF_OUTERCIRCLE_B_MASK               0x700000

/* CIF Channel Coef 5 Register */
#define CIF_COEF_5                                              0x24
#define CIF_COEFF_5_DELTA_COEF_CENTER_IR_OFFSET                 9
#define CIF_COEFF_5_DELTA_COEF_CENTER_IR_MASK                   0xFE00
#define CIF_COEFF_5_DELTA_COEF_MIDCIRCLE_IR_OFFSET              16
#define CIF_COEFF_5_DELTA_COEF_MIDCIRCLE_IR_MASK                0xF0000
#define CIF_COEFF_5_DELTA_COEF_OUTERCIRCLE_IR_OFFSET            20
#define CIF_COEFF_5_DELTA_COEF_OUTERCIRCLE_IR_MASK              0x700000

/* CIF Channel Buffer Base Address A Register */
#define CIF_BUFFER_BASE_ADDRESS_A                               0x28
#define CIF_BUFFER_BASE_ADDRESS_A_OFFSET                        1
#define CIF_BUFFER_BASE_ADDRESS_A_MASK                          0xFFFFFFFE
#define CIF_BUFFER_BASE_ADDRESS_ACT_A                           0x50

/* CIF Channel Buffer Base Address B Register */
#define CIF_BUFFER_BASE_ADDRESS_B                               0x2C
#define CIF_BUFFER_BASE_ADDRESS_B_OFFSET                        1
#define CIF_BUFFER_BASE_ADDRESS_B_MASK                          0xFFFFFFFE
#define CIF_BUFFER_BASE_ADDRESS_ACT_B                           0x54

/* CIF Channel Geometry Register */
#define CIF_GEOMETRY                                            0x30
#define CIF_GEOMETRY_ACTIVE_WIDTH_OFFSET                        0
#define CIF_GEOMETRY_ACTIVE_WIDTH_MASK                          0x1FFF
#define CIF_GEOMETRY_ACTIVE_HEIGHT_OFFSET                       16
#define CIF_GEOMETRY_ACTIVE_HEIGHT_MASK                         0x1FFF0000

/* CIF Channel Interrupt Source Register */
#define CIF_INT_SOURCE                                          0x34
#define CIF_INT_SOURCE_BUF_A2B_OFFSET                           0
#define CIF_INT_SOURCE_BUF_A2B_MASK                             0x1
#define CIF_INT_SOURCE_BUF_B2A_OFFSET                           1
#define CIF_INT_SOURCE_BUF_B2A_MASK                             0x2
#define CIF_INT_SOURCE_DISCARD_OFFSET                           2
#define CIF_INT_SOURCE_DISCARD_MASK                             0x4
#define CIF_INT_SOURCE_WRONG_WIDTH_OFFSET                       3
#define CIF_INT_SOURCE_WRONG_WIDTH_MASK                         0x8
#define CIF_INT_SOURCE_WRONG_HEIGHT_OFFSET                      4
#define CIF_INT_SOURCE_WRONG_HEIGHT_MASK                        0x10
#define CIF_INT_SOURCE_ERR_AXI_RESPONSE_OFFSET                  5
#define CIF_INT_SOURCE_ERR_AXI_RESPONSE_MASK                    0x20

/* CIF Channel Interrupt Mask Register */
#define CIF_INT_MASK                                            0x38
#define CIF_INT_MASK_BUF_A2B_OFFSET                             0
#define CIF_INT_MASK_BUF_A2B_MASK                               0x1
#define CIF_INT_MASK_BUF_A2B_ENABLE                             1
#define CIF_INT_MASK_BUF_A2B_DISABLE                            0
#define CIF_INT_MASK_BUF_B2A_OFFSET                             1
#define CIF_INT_MASK_BUF_B2A_MASK                               0x2
#define CIF_INT_MASK_BUF_B2A_ENABLE                             1
#define CIF_INT_MASK_BUF_B2A_DISABLE                            0
#define CIF_INT_MASK_DISCARD_OFFSET                             2
#define CIF_INT_MASK_DISCARD_MASK                               0x4
#define CIF_INT_MASK_DISCARD_ENABLE                             1
#define CIF_INT_MASK_DISCARD_DISABLE                            0
#define CIF_INT_MASK_WRONG_WIDTH_OFFSET                         3
#define CIF_INT_MASK_WRONG_WIDTH_MASK                           0x8
#define CIF_INT_MASK_WRONG_WIDTH_ENABLE                         1
#define CIF_INT_MASK_WRONG_WIDTH_DISABLE                        0
#define CIF_INT_MASK_WRONG_HEIGHT_OFFSET                        4
#define CIF_INT_MASK_WRONG_HEIGHT_MASK                          0x10
#define CIF_INT_MASK_WRONG_HEIGHT_ENABLE                        1
#define CIF_INT_MASK_WRONG_HEIGHT_DISABLE                       0
#define CIF_INT_MASK_ERR_AXI_RESPONSE_OFFSET                    5
#define CIF_INT_MASK_ERR_AXI_RESPONSE_MASK                      0x20
#define CIF_INT_MASK_ERR_AXI_RESPONSE_ENABLE                    1
#define CIF_INT_MASK_ERR_AXI_RESPONSE_DISABLE                   0

/* CIF Channel Interrupt Status Register */
#define CIF_INT_STATUS                                          0x3C
#define CIF_INT_STATUS_BUF_A2B_OFFSET                           0
#define CIF_INT_STATUS_BUF_A2B_MASK                             0x1
#define CIF_INT_STATUS_BUF_B2A_OFFSET                           1
#define CIF_INT_STATUS_BUF_B2A_MASK                             0x2
#define CIF_INT_STATUS_DISCARD_OFFSET                           2
#define CIF_INT_STATUS_DISCARD_MASK                             0x4
#define CIF_INT_STATUS_WRONG_WIDTH_OFFSET                       3
#define CIF_INT_STATUS_WRONG_WIDTH_MASK                         0x8
#define CIF_INT_STATUS_WRONG_HEIGHT_OFFSET                      4
#define CIF_INT_STATUS_WRONG_HEIGHT_MASK                        0x10
#define CIF_INT_STATUS_ERR_AXI_RESPONSE_OFFSET                  5
#define CIF_INT_STATUS_ERR_AXI_RESPONSE_MASK                    0x20

/* CIF Channel Interrupt Clear Register */
#define CIF_INT_CLEAR                                           0x40
#define CIF_INT_CLEAR_BUF_A2B_OFFSET                            0
#define CIF_INT_CLEAR_BUF_A2B_MASK                              0x1
#define CIF_INT_CLEAR_BUF_A2B                                   1
#define CIF_INT_CLEAR_BUF_B2A_OFFSET                            1
#define CIF_INT_CLEAR_BUF_B2A_MASK                              0x2
#define CIF_INT_CLEAR_BUF_B2A                                   1
#define CIF_INT_CLEAR_DISCARD_OFFSET                            2
#define CIF_INT_CLEAR_DISCARD_MASK                              0x4
#define CIF_INT_CLEAR_DISCARD                                   1
#define CIF_INT_CLEAR_WRONG_WIDTH_OFFSET                        3
#define CIF_INT_CLEAR_WRONG_WIDTH_MASK                          0x8
#define CIF_INT_CLEAR_WRONG_WIDTH                               1
#define CIF_INT_CLEAR_WRONG_HEIGHT_OFFSET                       4
#define CIF_INT_CLEAR_WRONG_HEIGHT_MASK                         0x10
#define CIF_INT_CLEAR_WRONG_HEIGHT                              1
#define CIF_INT_CLEAR_ERR_AXI_RESPONSE_OFFSET                   5
#define CIF_INT_CLEAR_ERR_AXI_RESPONSE_MASK                     0x20
#define CIF_INT_CLEAR_ERR_AXI_RESPONSE                          1

/* CIF Channel IPI Select Register */
#define CIF_IPI_SELECT                                          0x44
#define CIF_IPI_SELECT_IPIN_EN                                  1
#define CIF_IPI_SELECT_IPI0_OFFSET                              0
#define CIF_IPI_SELECT_IPI0_MASK                                0x1
#define CIF_IPI_SELECT_IPI1_OFFSET                              1
#define CIF_IPI_SELECT_IPI1_MASK                                0x2
#define CIF_IPI_SELECT_IPI2_OFFSET                              2
#define CIF_IPI_SELECT_IPI2_MASK                                0x4
#define CIF_IPI_SELECT_IPI3_OFFSET                              3
#define CIF_IPI_SELECT_IPI3_MASK                                0x8
#define CIF_IPI_SELECT_IPI4_OFFSET                              4
#define CIF_IPI_SELECT_IPI4_MASK                                0x10
#define CIF_IPI_SELECT_IPI5_OFFSET                              5
#define CIF_IPI_SELECT_IPI5_MASK                                0x20
#define CIF_IPI_SELECT_IPI6_OFFSET                              6
#define CIF_IPI_SELECT_IPI6_MASK                                0x40
#define CIF_IPI_SELECT_IPI7_OFFSET                              7
#define CIF_IPI_SELECT_IPI7_MASK                                0x80
#define CIF_IPI_SELECT_IPI8_OFFSET                              8
#define CIF_IPI_SELECT_IPI8_MASK                                0x100
#define CIF_IPI_SELECT_IPI9_OFFSET                              9
#define CIF_IPI_SELECT_IPI9_MASK                                0x200
#define CIF_IPI_SELECT_IPI10_OFFSET                             10
#define CIF_IPI_SELECT_IPI10_MASK                               0x400
#define CIF_IPI_SELECT_IPI11_OFFSET                             11
#define CIF_IPI_SELECT_IPI11_MASK                               0x800
#define CIF_IPI_SELECT_IPI12_OFFSET                             12
#define CIF_IPI_SELECT_IPI12_MASK                               0x1000
#define CIF_IPI_SELECT_IPI13_OFFSET                             13
#define CIF_IPI_SELECT_IPI13_MASK                               0x2000
#define CIF_IPI_SELECT_IPI14_OFFSET                             14
#define CIF_IPI_SELECT_IPI14_MASK                               0x4000
#define CIF_IPI_SELECT_IPI15_OFFSET                             15
#define CIF_IPI_SELECT_IPI15_MASK                               0x8000
#define CIF_IPI_SELECT_IPI16_OFFSET                             16
#define CIF_IPI_SELECT_IPI16_MASK                               0x10000
#define CIF_IPI_SELECT_IPI17_OFFSET                             17
#define CIF_IPI_SELECT_IPI17_MASK                               0x20000
#define CIF_IPI_SELECT_IPI18_OFFSET                             18
#define CIF_IPI_SELECT_IPI18_MASK                               0x40000
#define CIF_IPI_SELECT_IPI19_OFFSET                             19
#define CIF_IPI_SELECT_IPI19_MASK                               0x80000
#define CIF_IPI_SELECT_IPI20_OFFSET                             20
#define CIF_IPI_SELECT_IPI20_MASK                               0x100000
#define CIF_IPI_SELECT_IPI21_OFFSET                             21
#define CIF_IPI_SELECT_IPI21_MASK                               0x200000
#define CIF_IPI_SELECT_IPI22_OFFSET                             22
#define CIF_IPI_SELECT_IPI22_MASK                               0x400000
#define CIF_IPI_SELECT_IPI23_OFFSET                             23
#define CIF_IPI_SELECT_IPI23_MASK                               0x800000
#define CIF_IPI_SELECT_IPI24_OFFSET                             24
#define CIF_IPI_SELECT_IPI24_MASK                               0x1000000
#define CIF_IPI_SELECT_IPI25_OFFSET                             25
#define CIF_IPI_SELECT_IPI25_MASK                               0x2000000
#define CIF_IPI_SELECT_IPI26_OFFSET                             26
#define CIF_IPI_SELECT_IPI26_MASK                               0x4000000
#define CIF_IPI_SELECT_IPI27_OFFSET                             27
#define CIF_IPI_SELECT_IPI27_MASK                               0x8000000
#define CIF_IPI_SELECT_IPI28_OFFSET                             28
#define CIF_IPI_SELECT_IPI28_MASK                               0x10000000
#define CIF_IPI_SELECT_IPI29_OFFSET                             29
#define CIF_IPI_SELECT_IPI29_MASK                               0x20000000
#define CIF_IPI_SELECT_IPI30_OFFSET                             30
#define CIF_IPI_SELECT_IPI30_MASK                               0x40000000
#define CIF_IPI_SELECT_IPI31_OFFSET                             31
#define CIF_IPI_SELECT_IPI31_MASK                               0x80000000

#define CIF_CONFIGABLE_MINI_CHANNLE_NUM                         12
#define CIF_CONFIG_COEF_MINI_ID                                 17
#define CIF_DEFAULT_CONFIG_CHANNLE_NUM                          0

void se_cif_ipi_select(struct se_cif_dev *se_cif, u32 val);

void se_cif_channel_set_outbuf(struct se_cif_dev *se_cif,
			struct se_cif_buffer *buf);

void se_cif_channel_set_geometry(struct se_cif_dev *se_cif, struct se_cif_frame *f);

void se_cif_channel_set_threshold(struct se_cif_dev *se_cif, int threshold);

void se_cif_channel_set_coef(struct se_cif_dev *se_cif);

void se_cif_channel_disable(struct se_cif_dev *se_cif);

void se_cif_channel_config(struct se_cif_dev *se_cif);

void se_cif_clock_enable(struct se_cif_dev *se_cif);

void se_cif_clock_disable(struct se_cif_dev *se_cif);

void se_cif_channel_start_working(struct se_cif_dev *se_cif);

void se_cif_clean_irq_status(struct se_cif_dev *se_cif, u32 val);

void se_cif_clean_irq_status_bk(struct se_cif_dev *se_cif, u32 val);

u32 se_cif_get_irq_status(struct se_cif_dev *se_cif);

void se_cif_enable_irq(struct se_cif_dev *se_cif);

void se_cif_disable_irq(struct se_cif_dev *se_cif);

void se_cif_cap_frame_write_done(struct se_cif_dev *se_cif);

void se_cif_channel_softreset(struct se_cif_dev *se_cif);

void dump_cif_regs(struct se_cif_dev *se_cif);

u32 se_cif_read_buffer_act_a(struct se_cif_dev *se_cif);

u32 se_cif_read_buffer_act_b(struct se_cif_dev *se_cif);

void se_cif_dm_config(struct se_cif_dev *se_cif);

int se_rgbir422_enable(struct se_cif_dev *se_cif, bool enable);

int se_rgbir422_set_params(struct se_cif_dev *se_cif, rgbir422_coef_param *rgbir422_param);

int se_rgbir422_get_params(struct se_cif_dev *se_cif, rgbir422_coef_param *rgbir422_param);

#endif /* SE_CIF_HW_H_ */
