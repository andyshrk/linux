// SPDX-License-Identifier: GPL-2.0
/*  Himax Android Driver Sample Code for HX83193 chipset
 *
 *  Copyright (C) 2021 Himax Corporation.
 *
 *  This software is licensed under the terms of the GNU General Public
 *  License version 2,  as published by the Free Software Foundation,  and
 *  may be copied,  distributed,  and modified under those terms.
 *
 *  This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include "himax_common.h"

static void hx83193_sense_on(void *handle)
{
	struct himax_device *hdev = (struct himax_device *)handle;
	uint8_t tmp_data[DATA_LEN_4];
	int retry = 0;
	int ret = 0;

	I("%s Enter\n", __func__);

	do {
		himax_parse_assign_cmd(data_clear, tmp_data,
					   sizeof(tmp_data));
		himax_mcu_register_write(hdev, addr_ctrl_fw, DATA_LEN_4,
					 tmp_data);

		usleep_range(20000, 21000);

		himax_mcu_register_read(hdev, addr_ctrl_fw, DATA_LEN_4,
					tmp_data);

		I("%s:Read status from IC = 0x%02X,0x%02X\n", __func__,
		  tmp_data[0], tmp_data[1]);
	} while (tmp_data[0] != 0x00 && retry++ < 5);

	if (retry >= 5) {
		E("%s: Fail:\n", __func__);
		himax_mcu_tp_reset(hdev);
	} else {
		/* reset code*/
		tmp_data[0] = 0x00;
		tmp_data[1] = 0x00;

		ret = himax_bus_write(hdev, addr_sense_on_off_0, tmp_data, 2,
					  HIMAX_I2C_RETRY_TIMES);
		if (ret < 0)
			E("%s: i2c access fail!\n", __func__);

	}
	usleep_range(20000, 21000);

#if defined(HIMAX_I2C_PLATFORM)
	himax_mcu_interface_on(hdev);
#endif

}

static bool hx83193_sense_off(void *handle)
{
	struct himax_device *hdev = (struct himax_device *)handle;
	struct himax_ts_data *ts = hdev->private_ts;
	bool result = true;
	uint8_t cnt = 0;
	uint8_t tmp_data[DATA_LEN_4] = { 0 };
	uint8_t cMax = 14;
	uint8_t check = 0x87;
	int ret = 0;

	usleep_range(20000, 21000);

	himax_mcu_register_read(hdev, addr_cs_central_state, DATA_LEN_4, tmp_data);

	if (tmp_data[0] == 0x05) {
		do {
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0xA5;
			himax_mcu_register_write(hdev, addr_ctrl_fw, DATA_LEN_4,
						 tmp_data);

			usleep_range(20000, 21000);
			himax_mcu_register_read(hdev, addr_ctrl_fw, DATA_LEN_4,
						tmp_data);
			if (cnt++ >= cMax)
				break;
		} while (tmp_data[0] != check);
		I("%s: 9000005C data[0]=0x%02X, Retry times = %d\n", __func__,
		  tmp_data[0], cnt);
	}
	if (ts->slave_ic_num >= 1) {
		cnt = 0;
		do {
			tmp_data[0] = para_sense_off_0;
			tmp_data[1] = para_sense_off_1;

			ret = himax_bus_write_slave(hdev, IC_SLAVE_1, addr_sense_on_off_0, tmp_data, 2,
						  HIMAX_I2C_RETRY_TIMES);
			if (ret < 0) {
				W("[IC_SLAVE_1] %s: i2c access fail!\n", __func__);
				break;
			}
			himax_mcu_register_read_slave(hdev, IC_SLAVE_1, addr_cs_central_state, DATA_LEN_4, tmp_data);
			I("[IC_SLAVE_1] %s: Check enter_save_mode data[0]=0x%02X\n", __func__,
			  tmp_data[0]);

			if (tmp_data[0] == 0x0C)
				break;

		} while (cnt++ < 15);
	}
	if (ts->slave_ic_num == 2) {
		cnt = 0;
		do {
			tmp_data[0] = para_sense_off_0;
			tmp_data[1] = para_sense_off_1;

			ret = himax_bus_write_slave(hdev, IC_SLAVE_2, addr_sense_on_off_0, tmp_data, 2,
						  HIMAX_I2C_RETRY_TIMES);
			if (ret < 0) {
				W("[IC_SLAVE_2] %s: i2c access fail!\n", __func__);
				break;
			}
			himax_mcu_register_read_slave(hdev, IC_SLAVE_2, addr_cs_central_state, DATA_LEN_4, tmp_data);
			I("[IC_SLAVE_2] %s: Check enter_save_mode data[0]=0x%02X\n", __func__,
			  tmp_data[0]);

			if (tmp_data[0] == 0x0C)
				break;

		} while (cnt++ < 15);
	}
	cnt = 0;
	do {
		tmp_data[0] = para_sense_off_0;
		tmp_data[1] = para_sense_off_1;

		ret = himax_bus_write(hdev, addr_sense_on_off_0, tmp_data, 2,
					  HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		himax_mcu_register_read(hdev, addr_cs_central_state, DATA_LEN_4,
					tmp_data);
		I("%s: Check enter_save_mode data[0]=0x%02X\n", __func__,
		  tmp_data[0]);

		if (tmp_data[0] == 0x0C) {
			return true;
		} else if (cnt == 6) {
			usleep_range(10000, 11000);
			himax_mcu_tp_reset(hdev);
		}

	} while (cnt++ < 15);

	return result;
}

static bool hx83193_mcu_dd_clk_set(void *handle, bool enable)
{
	struct himax_device *hdev = (struct himax_device *)handle;
	struct himax_ic_data *ic_data = hdev->ic_data;
	uint8_t data[DATA_LEN_4] = { 0 };
	uint8_t check_STOP_FW = 0x6A;
	uint8_t cMax = 7;
	uint8_t cnt = 0;

	if (ic_data->STOP_FW_BY_HOST_EN) {
		if (enable) {
			do {
				data[0] = 0x5A;
				himax_mcu_register_write(hdev, addr_ctrl_fw, DATA_LEN_4,
							 data);

				usleep_range(20000, 21000);
				himax_mcu_register_read(hdev, addr_ctrl_fw, DATA_LEN_4, data);
				I("%s: Check 9000005C data[0]=0x%02X\n", __func__, data[0]);
				if (cnt++ >= cMax)
					break;
			} while (data[0] != check_STOP_FW);
			if (data[0] == check_STOP_FW)
				I("%s: STOP_FW_BY_HOST finished!\n", __func__);
			else
				W("%s: STOP_FW_BY_HOST Fail!\n", __func__);
		} else {
			do {
				data[0] = 0x00;
				himax_mcu_register_write(hdev, addr_ctrl_fw, DATA_LEN_4,
							 data);

				usleep_range(20000, 21000);
				himax_mcu_register_read(hdev, addr_ctrl_fw, DATA_LEN_4, data);
				I("%s: Check 9000005C data[0]=0x%02X\n", __func__, data[0]);
				if (cnt++ >= cMax)
					break;
			} while (data[0] == check_STOP_FW);
			if (data[0] != check_STOP_FW)
				I("%s: START_FW_BY_HOST finished!\n", __func__);
			else
				E("%s: START_FW_BY_HOST Fail!\n", __func__);
		}
	}

	data[0] = (enable) ? 0xDD : 0x00;
	return (himax_mcu_register_write(hdev, addr_osc_en, DATA_LEN_4, data) ==
		NO_ERR);
}

static void hx83193_mcu_dd_reg_en(void *handle, bool enable)
{
	struct himax_device *hdev = (struct himax_device *)handle;
	uint8_t data[DATA_LEN_4] = { 0 };

	data[0] = 0xA5;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0x00;

	himax_mcu_register_write(hdev, addr_osc_pw, DATA_LEN_4, data);
	data[0] = 0x00;
	data[1] = 0x55;
	data[2] = 0x66;
	data[3] = 0xCC;

	himax_mcu_dd_reg_write(hdev, 0xEB, 0, 4, data, 0);
	data[0] = 0x00;
	data[1] = 0x83;
	data[2] = 0x19;
	data[3] = 0x3A;
	himax_mcu_dd_reg_write(hdev, 0xB9, 0, 4, data, 0);
}

static void hx83193_func_re_init(struct himax_device *hdev)
{
	struct himax_core_fp core_fp = hdev->core_fp;

	core_fp.fp_sense_on = hx83193_sense_on;
	core_fp.fp_sense_off = hx83193_sense_off;
	core_fp.fp_dd_clk_set = hx83193_mcu_dd_clk_set;
	core_fp.fp_dd_reg_en = hx83193_mcu_dd_reg_en;
}

static bool hx83193_chip_detect(struct himax_device *hdev)
{
	struct himax_ts_data *ts = hdev->private_ts;
	uint8_t tmp_data[DATA_LEN_4] = { 0 };
	bool ret_data = false;
	int ret = 0;
	int i = 0;

	ret = himax_mcu_in_cmd_struct_init(hdev);
	if (ret < 0) {
		ret_data = false;
		E("%s:cmd_struct_init Fail:\n", __func__);
		return ret_data;
	}

	hx83193_func_re_init(hdev);

	hx83193_sense_off(hdev);

	for (i = 0; i < 5; i++) {
		himax_mcu_register_read(hdev, addr_icid_addr, DATA_LEN_4, tmp_data);
		I("%s:Read driver IC ID = HX%X%X%X\n", __func__, tmp_data[3],
		  tmp_data[2], tmp_data[1]);

		if ((tmp_data[3] == 0x83) && (tmp_data[2] == 0x19) &&
		    (tmp_data[1] == 0x3a)) {
			strlcpy(ts->chip_name, HX_83193A_SERIES_PWON,
				30);
			ret_data = true;
			return ret_data;
		}
	}
	ret_data = false;
	E("%s:Read driver ID register Fail:\n", __func__);
	E("Could NOT find Himax Chipset\n");
	E("Please check 1.VCCD,VCCA,VSP,VSN\n");
	E("2.LCM_RST,TP_RST\n");
	E("3.Power On Sequence\n");

	return ret_data;
}

bool _hx83193_init(struct himax_device *hdev)
{
	bool ret = false;

	I("%s\n", __func__);
	ret = hx83193_chip_detect(hdev);
	return ret;
}
