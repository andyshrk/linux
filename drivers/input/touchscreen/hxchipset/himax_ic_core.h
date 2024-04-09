/* SPDX-License-Identifier: GPL-2.0 */
/*	Himax Android Driver Sample Code for ic core functions
 *
 *	Copyright (C) 2021 Himax Corporation.
 *
 *	This software is licensed under the terms of the GNU General Public
 *	License version 2,	as published by the Free Software Foundation,  and
 *	may be copied,	distributed,  and modified under those terms.
 *
 *	This program is distributed in the hope that it will be useful,
 *	 but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 */

#ifndef __HIMAX_IC_CORE_H__
#define __HIMAX_IC_CORE_H__

//#include "himax_platform.h"
//#include "himax_common.h"
#include <linux/slab.h>

#define DATA_LEN_8				8
#define DATA_LEN_4				4
#define ADDR_LEN_4				4
#define FLASH_RW_MAX_LEN		256
#define FLASH_WRITE_BURST_SZ	8
#define MAX_I2C_TRANS_SZ		128
#define HIMAX_TOUCH_DATA_SIZE	128
#define HIMAX_REG_RETRY_TIMES	5

#define FW_SECTOR_PER_BLOCK		8
#define FW_PAGE_PER_SECTOR		64
#define FW_PAGE_SZ				128
#define HX1K					0x400
#define HX64K					0x10000

#define HX_RW_REG_FAIL			(-1)
#define HX_DRIVER_MAX_IC_NUM	5

#if defined(__HIMAX_HX83192_MOD__)
#define HX_MOD_KSYM_HX83192 HX_MOD_KSYM_HX83192
#endif

#if defined(__HIMAX_HX83193_MOD__)
#define HX_MOD_KSYM_HX83193 HX_MOD_KSYM_HX83193
#endif

enum AHB_Interface_Command_Table {
	addr_AHB_address_byte_0		=	0x00,
	addr_AHB_rdata_byte_0		=	0x08,
	addr_AHB_access_direction	=	0x0C,
	addr_AHB_continous			=	0x13,
	addr_AHB_INC4				=	0x0D,
	addr_sense_on_off_0			=	0x31,
	addr_sense_on_off_1			=	0x32,
	addr_read_event_stack		=	0x30,
	para_AHB_access_direction_read	=	0x00,
	para_AHB_continous			=	0x31,
	para_AHB_INC4				=	0x10,
	para_sense_off_0			=	0x27,
	para_sense_off_1			=	0x95,
	addr_CONV_I2C_cmd			=	0x80,
};
/* CORE_FW */
	#define addr_fw_state						0x800204DC
	#define addr_scu_reload_control				0x90000080
	#define addr_psl							0x900000A0
	#define addr_cs_central_state				0x900000A8
	#define addr_flag_reset_event				0x900000E4
	#define addr_chk_dd_status					0x900000E8
	#define addr_osc_en							0x9000009C
	#define addr_osc_pw							0x90000280
	#define addr_system_reset					0x90000018
	#define addr_ctrl_fw						0x9000005C
	#define addr_icid_addr						0x900000D0
	#define addr_program_reload_from			0x00000000
	#define addr_reload_status					0x80050000
	#define addr_reload_crc32_result			0x80050018
	#define addr_reload_addr_from				0x80050020
	#define addr_reload_addr_cmd_beat			0x80050028
	#define data_system_reset					0x00000055
	#define data_clear							0x00000000
	#define addr_raw_out_sel					0x100072EC
	#define addr_set_frame_addr					0x10007294
	#define addr_sorting_mode_en				0x10007F04
	#define addr_fw_mode_status					0x10007088
	#define addr_fw_architecture_version		0x10007004
	#define addr_fw_config_date					0x10007038
	#define addr_fw_config_version				0x10007084
	#define addr_fw_CID							0x10007000
	#define addr_fw_customer					0x10007008
	#define addr_fw_project_name				0x10007014
	#define addr_fw_remark1						0x10007020
	#define addr_fw_remark2						0x1000702C
	#define addr_fw_ticket						0x10007050
	#define addr_fw_dbg_msg_addr				0x10007F40
	#define addr_HX_ID_EN						0x10007134
	#define addr_fw_define_flash_reload			0x10007f00
	#define addr_fw_define_rawdata_normalize	0x10007130
	#define addr_fw_define_2nd_flash_reload		0x100072c0
	#define data_fw_define_flash_reload_dis		0x0000a55a
	#define data_fw_define_flash_reload_en		0x00000000
	#define addr_fw_define_int_is_edge			0x10007088
	#define addr_fw_define_rxnum_txnum_maxpt	0x100070f4
	#define addr_fw_define_xy_res				0x100070f8
	#define addr_rawdata						0x10000000
	#define addr_fail_det_GPIO1_msg				0x100074C0
	#define addr_retry_wrapper_clr_pw			0x900002A0


/* CORE_FLASH */
	#define addr_ctrl_base						0x80000000
	#define addr_spi200_trans_fmt				(addr_ctrl_base + 0x10)
	#define addr_spi200_trans_ctrl				(addr_ctrl_base + 0x20)
	#define addr_spi200_cmd						(addr_ctrl_base + 0x24)
	#define addr_spi200_addr					(addr_ctrl_base + 0x28)
	#define addr_spi200_data					(addr_ctrl_base + 0x2c)
	#define addr_spi200_fifo_rst				(addr_ctrl_base + 0x30)
	#define addr_spi200_rst_status				(addr_ctrl_base + 0x34)
	#define addr_spi200_flash_speed				(addr_ctrl_base + 0x40)
	#define data_spi200_txfifo_rst				0x00000004
	#define data_spi200_rxfifo_rst				0x00000002
	#define data_spi200_trans_fmt				0x00020780
	#define data_spi200_trans_ctrl_1			0x42000003
	#define data_spi200_trans_ctrl_2			0x47000000
	#define data_spi200_trans_ctrl_3			0x67000000
	#define data_spi200_trans_ctrl_4			0x610ff000
	#define data_spi200_trans_ctrl_6			0x42000000
	#define data_spi200_trans_ctrl_7			0x6940020f
	#define data_spi200_cmd_1					0x00000005
	#define data_spi200_cmd_2					0x00000006
	#define data_spi200_cmd_3					0x000000C7
	#define data_spi200_cmd_4					0x000000D8
	#define data_spi200_cmd_6					0x00000002
	#define data_spi200_cmd_7					0x0000003b
	#define data_spi200_cmd_8					0x00000003
	#define data_set_flash_speed				0x00000001
	#define addr_WP_pin_base					0x90028000
	#define addr_WP_gpio0_cmd_04				(addr_WP_pin_base + 0x04)
	#define addr_WP_gpio0_cmd_0C				(addr_WP_pin_base + 0x0C)
	#define data_WP_gpio0_cmd_00				0x00000000
	#define data_WP_gpio0_cmd_01				0x00000001
	#define addr_WP_gpio4_cmd_04				(addr_WP_pin_base + 0x04)
	#define addr_WP_gpio4_cmd_1C				(addr_WP_pin_base + 0x1C)
	#define data_WP_gpio4_cmd_00				0x00000000
	#define data_WP_gpio4_cmd_01				0x00000001
	#define data_WP_gpio4_cmd_10				0x00000010
	#define addr_WP_pin_HX83192D				0x90000230
	#define data_WP_disable_HX83192D			0x00000003
	#define data_WP_enable_HX83192D			0x00000002
	#define addr_BP_lock_base					0x80000000
	#define addr_BP_lock_cmd_10					(addr_BP_lock_base + 0x10)
	#define addr_BP_lock_cmd_20					(addr_BP_lock_base + 0x20)
	#define addr_BP_lock_cmd_24					(addr_BP_lock_base + 0x24)
	#define addr_BP_lock_cmd_2C					(addr_BP_lock_base + 0x2C)
	#define data_BP_lock_cmd_1					0x00020780
	#define data_BP_lock_cmd_2					0x47000000
	#define data_BP_lock_cmd_3					0x00000006
	#define data_BP_lock_cmd_4					0x41000000
	#define data_BP_lock_cmd_5					0x00000000
	#define data_BP_lock_cmd_6					0x00000001
	#define data_BP_lock_cmd_7					0x0000009C
	#define data_BP_check_cmd_1					0x42000000
	#define data_BP_check_cmd_2					0x00000005
	#define data_BP_check_cmd_3					0x42000002
	#define data_BP_check_cmd_4					0x0000009F

enum bin_desc_map_table {
	TP_CONFIG_TABLE = 0x0000000A,
	FW_CID = 0x10000000,
	FW_VER = 0x10000100,
	CFG_VER = 0x30000000,//0x10000005,
};

struct himax_core_fp {
	void (*fp_sense_on)(void *handle);
	bool (*fp_sense_off)(void *handle);
	bool (*fp_dd_clk_set)(void *handle, bool enable);
	void (*fp_dd_reg_en)(void *handle, bool enable);
};

#endif

