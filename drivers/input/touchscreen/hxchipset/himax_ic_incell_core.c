// SPDX-License-Identifier: GPL-2.0
/*  Himax Android Driver Sample Code for incell ic core functions
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

uint32_t dbg_reg_ary[6] = { addr_fw_dbg_msg_addr, addr_fw_state, addr_scu_reload_control, addr_cs_central_state,
				 addr_chk_dd_status, addr_flag_reset_event };

void himax_mcu_burst_enable(struct himax_device *hdev, uint8_t auto_add_4_byte)
{
	uint8_t tmp_data[DATA_LEN_4];
	int ret;

	tmp_data[0] = (para_AHB_INC4 | auto_add_4_byte);

	ret = himax_bus_write(hdev, addr_AHB_INC4, tmp_data, 1,
			      HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return;
	}
}

int himax_mcu_register_read(struct himax_device *hdev, uint32_t read_addr,
			uint32_t read_length, uint8_t *read_data)
{
	uint8_t tmp_data[DATA_LEN_4];
	int ret = 0;

	/*I("%s,Entering\n",__func__);*/

	if (read_length > FLASH_RW_MAX_LEN) {
		E("%s: read len over %d!\n", __func__, FLASH_RW_MAX_LEN);
		return LENGTH_FAIL;
	}

	if (read_length > DATA_LEN_4)
		himax_mcu_burst_enable(hdev, 1);
	else
		himax_mcu_burst_enable(hdev, 0);

	himax_parse_assign_cmd(read_addr, tmp_data, sizeof(tmp_data));

	ret = himax_bus_write(hdev, addr_AHB_address_byte_0, tmp_data, DATA_LEN_4,
			      HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return I2C_FAIL;
	}

	tmp_data[0] = para_AHB_access_direction_read;

	ret = himax_bus_write(hdev, addr_AHB_access_direction, tmp_data, 1,
			      HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return I2C_FAIL;
	}

	ret = himax_bus_read(hdev, addr_AHB_rdata_byte_0, read_data, read_length,
			     HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return I2C_FAIL;
	}

	if (read_length > DATA_LEN_4)
		himax_mcu_burst_enable(hdev, 0);

	return NO_ERR;
}

int himax_mcu_register_read_slave(struct himax_device *hdev, uint8_t device,
			uint32_t read_addr, uint32_t read_length, uint8_t *read_data)
{
	uint8_t tmp_data[DATA_LEN_4];
	int ret = 0;

	/*I("%s,Entering\n",__func__);*/

	if (read_length > FLASH_RW_MAX_LEN) {
		E("%s: read len over %d!\n", __func__, FLASH_RW_MAX_LEN);
		return LENGTH_FAIL;
	}

	if (read_length > DATA_LEN_4)
		himax_mcu_burst_enable(hdev, 1);
	else
		himax_mcu_burst_enable(hdev, 0);

	himax_parse_assign_cmd(read_addr, tmp_data, sizeof(tmp_data));

	ret = himax_bus_write_slave(hdev, device, addr_AHB_address_byte_0, tmp_data, DATA_LEN_4,
			      HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return I2C_FAIL;
	}

	tmp_data[0] = para_AHB_access_direction_read;

	ret = himax_bus_write_slave(hdev, device, addr_AHB_access_direction, tmp_data, 1,
			      HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return I2C_FAIL;
	}

	ret = himax_bus_read_slave(hdev, device, addr_AHB_rdata_byte_0, read_data, read_length,
			     HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return I2C_FAIL;
	}

	if (read_length > DATA_LEN_4)
		himax_mcu_burst_enable(hdev, 0);

	return NO_ERR;
}


static int himax_mcu_flash_write_burst_lenth(struct himax_device *hdev,
			uint8_t *reg_byte, uint8_t *write_data, uint32_t length)
{
	uint8_t *data_byte;
	int ret = 0;

	if (hdev->internal_buffer == NULL) {
		E("%s: internal buffer not initialized!\n", __func__);
		return MEM_ALLOC_FAIL;
	}
	data_byte = hdev->internal_buffer;

	/* assign addr 4bytes */
	memcpy(data_byte, reg_byte, ADDR_LEN_4);
	/* assign data n bytes */
	memcpy(data_byte + ADDR_LEN_4, write_data, length);

	ret = himax_bus_write(hdev, addr_AHB_address_byte_0, data_byte,
			      length + ADDR_LEN_4, HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: xfer fail!\n", __func__);
		return I2C_FAIL;
	}

	return NO_ERR;
}

int himax_mcu_register_write(struct himax_device *hdev, uint32_t write_addr,
			uint32_t write_length, uint8_t *write_data)
{
	int address;
	uint8_t tmp_addr[4];
	uint8_t *tmp_data;
	int total_read_times = 0;
	uint32_t max_bus_size = MAX_I2C_TRANS_SZ;
	uint32_t total_size_temp = 0;
	unsigned int i = 0;
	int ret = 0;

	/*I("%s,Entering\n", __func__);*/

	total_size_temp = write_length;

	himax_parse_assign_cmd(write_addr, tmp_addr, sizeof(tmp_addr));

	if (total_size_temp % max_bus_size == 0)
		total_read_times = total_size_temp / max_bus_size;
	else
		total_read_times = total_size_temp / max_bus_size + 1;

	if (write_length > DATA_LEN_4)
		himax_mcu_burst_enable(hdev, 1);
	else
		himax_mcu_burst_enable(hdev, 0);

	for (i = 0; i < (total_read_times); i++) {
		/* I("[log]write %d time start!\n", i);
		 * I("[log]addr[3]=0x%02X, addr[2]=0x%02X,
			addr[1]=0x%02X,	addr[0]=0x%02X!\n",
			tmp_addr[3], tmp_addr[2],
			tmp_addr[1], tmp_addr[0]);
		 * I("%s, write addr = 0x%02X%02X%02X%02X\n",
			__func__, tmp_addr[3], tmp_addr[2],
			tmp_addr[1], tmp_addr[0]);
		 */

		if (total_size_temp >= max_bus_size) {
			tmp_data = write_data + (i * max_bus_size);

			ret = himax_mcu_flash_write_burst_lenth(hdev,
				tmp_addr, tmp_data, max_bus_size);
			if (ret < 0) {
				I("%s: i2c access fail!\n", __func__);
				return I2C_FAIL;
			}
			total_size_temp = total_size_temp - max_bus_size;
		} else {
			tmp_data = write_data + (i * max_bus_size);
			/* I("last total_size_temp=%d\n",
			 *	total_size_temp % max_bus_size);
			 */
			ret = himax_mcu_flash_write_burst_lenth(hdev,
				tmp_addr, tmp_data, total_size_temp);
			if (ret < 0) {
				I("%s: i2c access fail!\n", __func__);
				return I2C_FAIL;
			}
		}

		/*I("[log]write %d time end!\n", i);*/
		address = ((i + 1) * max_bus_size);
		tmp_addr[0] = (write_addr & 0xFF) + (uint8_t)((address)&0x00FF);

		if (tmp_addr[0] < (write_addr & 0xFF))
			tmp_addr[1] = ((write_addr >> 8) & 0xFF) +
				      (uint8_t)((address >> 8) & 0x00FF) + 1;
		else
			tmp_addr[1] = ((write_addr >> 8) & 0xFF) +
				      (uint8_t)((address >> 8) & 0x00FF);

		udelay(100);
	}

	return NO_ERR;
}

void himax_mcu_interface_on(struct himax_device *hdev)
{
	uint8_t tmp_data[DATA_LEN_4];
	int ret = 0;

	/* Read a dummy register to wake up I2C.*/
	ret = himax_bus_read(hdev, addr_CONV_I2C_cmd, tmp_data, DATA_LEN_4,
			     HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) { /* to knock I2C*/
		E("%s: i2c access fail!\n", __func__);
		return;
	}
}

void himax_mcu_polling_cascade_ic(struct himax_device *hdev)
{
	uint8_t tmp_data[DATA_LEN_4];
	struct himax_ts_data *ts = hdev->private_ts;
	int ret = 0;

	ts->slave_ic_num = 0;
#if defined(HIMAX_I2C_PLATFORM)
	ret = himax_bus_read_slave(hdev, IC_SLAVE_1,
		addr_CONV_I2C_cmd, tmp_data, DATA_LEN_4, 2);
	if (ret >= 0)
		ts->slave_ic_num = 1;

	ret = himax_bus_read_slave(hdev, IC_SLAVE_2,
		addr_CONV_I2C_cmd, tmp_data, DATA_LEN_4, 2);
	if (ret >= 0)
		ts->slave_ic_num = 2;

	ret = himax_bus_read(hdev, addr_CONV_I2C_cmd, tmp_data, DATA_LEN_4,
			     HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return;
	}
#else
	for (ret = 0; ret < 5; ret++) {
		himax_mcu_register_read_slave(hdev, IC_SLAVE_1, addr_icid_addr, DATA_LEN_4, tmp_data);
		if (tmp_data[3] == 0x83)
			ts->slave_ic_num = 1;
	}
	for (ret = 0; ret < 5; ret++) {
		himax_mcu_register_read_slave(hdev, IC_SLAVE_2, addr_icid_addr, DATA_LEN_4, tmp_data);
		if (tmp_data[3] == 0x83)
			ts->slave_ic_num = 2;
	}
#endif
	I("CASCADE_IC_NUM : %d\n", (ts->slave_ic_num + 1));
}

void himax_mcu_power_on_init(struct himax_device *hdev)
{
	uint8_t data[DATA_LEN_4] = { 0 };

	I("%s:entering\n", __func__);
	himax_parse_assign_cmd(data_clear, data, sizeof(data));
	/*RawOut select initial*/
	himax_mcu_register_write(hdev, addr_raw_out_sel, DATA_LEN_4, data);
	/*DSRAM func initial*/
	himax_mcu_assign_sorting_mode(hdev, data);
	/* N frame initial :　reset N frame back to default value 1 for normal mode
	 */
	himax_mcu_register_write(hdev, addr_set_frame_addr, DATA_LEN_4, data);
	/*FW reload done initial*/
	himax_mcu_register_write(hdev, addr_fw_define_2nd_flash_reload, DATA_LEN_4,
				 data);
	himax_mcu_tp_reset(hdev);
}

bool himax_mcu_dd_reg_write(struct himax_device *hdev, uint8_t addr,
			uint8_t pa_num, int len, uint8_t *data, uint8_t bank)
{
	/*Calculate total write length*/
	uint32_t data_len = (((len + pa_num - 1) / 4 - pa_num / 4) + 1) * 4;
	uint8_t w_data[500];
	uint32_t tmp_addr_32 = 0;
	uint8_t tmp_addr[4] = { 0 };
	uint8_t tmp_data[4] = { 0 };
	bool *chk_data;
	uint32_t chk_idx = 0;
	int i = 0;

	chk_data = kcalloc(data_len, sizeof(bool), GFP_KERNEL);
	if (chk_data == NULL) {
		E("%s Allocate chk buf failed\n", __func__);
		return false;
	}

	memset(w_data, 0, 500);

	/*put input data*/
	chk_idx = pa_num % 4;
	for (i = 0; i < len; i++) {
		w_data[chk_idx] = data[i];
		chk_data[chk_idx++] = true;
	}

	/*get original data*/
	chk_idx = (pa_num / 4) * 4;
	for (i = 0; i < data_len; i++) {
		if (!chk_data[i]) {
			himax_mcu_dd_reg_read(hdev, addr, (uint8_t)(chk_idx + i), 1,
					      tmp_data, bank, IC_MASTER);

			w_data[i] = tmp_data[0];
			chk_data[i] = true;
		}
		D("%s w_data[%d] = %2X\n", __func__, i, w_data[i]);
	}

	tmp_addr[3] = 0x30;
	tmp_addr[2] = addr >> 4;
	tmp_addr[1] = (addr << 4) | bank;
	tmp_addr[0] = chk_idx;
	kfree(chk_data);

	tmp_addr_32 = tmp_addr[3] << 24 | tmp_addr[2] << 16 | tmp_addr[1] << 8 |
		      tmp_addr[0];
	D("%s Addr = 0x%08X.\n", __func__, tmp_addr_32);

	return (himax_mcu_register_write(hdev, tmp_addr_32, data_len, w_data) ==
		NO_ERR);
}

bool himax_mcu_dd_reg_read(struct himax_device *hdev, uint8_t addr,
			uint8_t pa_num, int len, uint8_t *data, uint8_t bank, uint8_t ic_device)
{
	uint32_t tmp_addr_32 = 0;
	uint8_t tmp_addr[4] = { 0 };
	uint8_t tmp_data[4] = { 0 };
	int i = 0;

	if (len > 16) {
		I("%s length = %d is over limitation\n", __func__, len);
		return false;
	}
	if (ic_device == IC_MASTER) {
		for (i = 0; i < len; i++) {
			tmp_addr[3] = 0x30;
			tmp_addr[2] = addr >> 4;
			tmp_addr[1] = (addr << 4) | bank;
			tmp_addr[0] = pa_num + i;

			tmp_addr_32 = tmp_addr[3] << 24 | tmp_addr[2] << 16 |
					  tmp_addr[1] << 8 | tmp_addr[0];
			if (himax_mcu_register_read(hdev, tmp_addr_32, DATA_LEN_4, tmp_data))
				goto READ_FAIL;

			data[i] = tmp_data[(i % 4)];

			D("%s Addr = 0x%08X .data = %2X\n", __func__,
			  tmp_addr_32, data[i]);
		}
		return true;
	} else if (ic_device == IC_SLAVE_1) {
		for (i = 0; i < len; i++) {
			tmp_addr[3] = 0x30;
			tmp_addr[2] = addr >> 4;
			tmp_addr[1] = (addr << 4) | bank;
			tmp_addr[0] = pa_num + i;

			tmp_addr_32 = tmp_addr[3] << 24 | tmp_addr[2] << 16 |
						tmp_addr[1] << 8 | tmp_addr[0];
			if (himax_mcu_register_read_slave(hdev, IC_SLAVE_1, tmp_addr_32, DATA_LEN_4, tmp_data))
				goto READ_FAIL;

			data[i] = tmp_data[(i % 4)];

			D("%s Addr = 0x%08X .data = %2X\n", __func__,
				tmp_addr_32, data[i]);
		}
		return true;
	} else if (ic_device == IC_SLAVE_2) {
		for (i = 0; i < len; i++) {
			tmp_addr[3] = 0x30;
			tmp_addr[2] = addr >> 4;
			tmp_addr[1] = (addr << 4) | bank;
			tmp_addr[0] = pa_num + i;

			tmp_addr_32 = tmp_addr[3] << 24 | tmp_addr[2] << 16 |
					  tmp_addr[1] << 8 | tmp_addr[0];
			if (himax_mcu_register_read_slave(hdev, IC_SLAVE_2, tmp_addr_32, DATA_LEN_4, tmp_data))
				goto READ_FAIL;

			data[i] = tmp_data[(i % 4)];

			D("%s Addr = 0x%08X .data = %2X\n", __func__,
			  tmp_addr_32, data[i]);
		}
		return true;
	}

READ_FAIL:
	E("%s Read DD reg Failed.\n", __func__);
	return false;
}

/*-------------------------------------------------------------------------
 *
 *	Create: Unknown
 *
 *	Description:  Do software reset by setting addr 0x90000018 register with
 *value 0x55. Parameters: void
 *
 *	Returns: void
 *
 *	Side effects: None
 *
 */
void himax_mcu_system_reset(struct himax_device *hdev)
{
	uint8_t data[DATA_LEN_4] = { 0 };

	I("%s: Entering!\n", __func__);

	himax_parse_assign_cmd(data_system_reset, data, sizeof(data));
	himax_mcu_register_write(hdev, addr_system_reset, DATA_LEN_4, data);

	msleep(100);

#if defined(HIMAX_I2C_PLATFORM)
	himax_mcu_interface_on(hdev);
#endif
}

void himax_mcu_command_reset(struct himax_device *hdev)
{
	uint8_t tmp_data[DATA_LEN_4] = { 0 };
	int ret = 0;

	I("%s: Entering!\n", __func__);

	/* reset code*/
	tmp_data[0] = 0x00;
	tmp_data[1] = 0x00;

	ret = himax_bus_write(hdev, addr_sense_on_off_0, tmp_data, 1,
			      HIMAX_I2C_RETRY_TIMES);
	if (ret < 0)
		E("%s: i2c access fail!\n", __func__);

	ret = himax_bus_write(hdev, addr_sense_on_off_1, tmp_data, 1,
			      HIMAX_I2C_RETRY_TIMES);
	if (ret < 0)
		E("%s: i2c access fail!\n", __func__);

	msleep(20);

#if defined(HIMAX_I2C_PLATFORM)
	himax_mcu_interface_on(hdev);
#endif
}

uint32_t himax_mcu_check_CRC(struct himax_device *hdev, uint8_t *start_addr,
			int reload_length)
{
	uint32_t result = 0;
	uint8_t tmp_data[DATA_LEN_4] = { 0 };
	int i_counter = 0, ret = 0;
	int length = reload_length / DATA_LEN_4;

	tmp_data[0] = 0xA5;


	/* Disable retry wrapper to avoid I2C CLK low issue */
	himax_mcu_register_write(hdev, addr_retry_wrapper_clr_pw, 4, tmp_data);
	I("%s: Disable retry wrapper for flash read.\n", __func__);

	ret = himax_mcu_register_write(hdev, addr_reload_addr_from, DATA_LEN_4,
					start_addr);
	if (ret < NO_ERR) {
		E("%s: i2c access fail!\n", __func__);
		return HW_CRC_FAIL;
	}

	tmp_data[3] = 0x00;
	tmp_data[2] = 0x99;
	tmp_data[1] = (length >> 8);
	tmp_data[0] = length;

	ret = himax_mcu_register_write(hdev, addr_reload_addr_cmd_beat, DATA_LEN_4,
					tmp_data);
	if (ret < NO_ERR) {
		E("%s: i2c access fail!\n", __func__);
		return HW_CRC_FAIL;
	}
	ret = himax_mcu_register_read(hdev, addr_reload_status, DATA_LEN_4,
						tmp_data);
	if (ret < NO_ERR) {
		E("%s: i2c access fail!\n", __func__);
		return HW_CRC_FAIL;
	}


	if (tmp_data[1] != 0x99) {
		E("%s: Reload status cmd fail and out of retry count!\n", __func__);
		return HW_CRC_FAIL;
	}
	I("%s:8005_0000 read data[3]=0x%02X,data[2]=0x%02X,data[1]=0x%02X,data[0]=0x%02X\n",
			  __func__, tmp_data[3], tmp_data[2], tmp_data[1],
			  tmp_data[0]);

	i_counter = 0;

	do {

		ret = himax_mcu_register_read(hdev, addr_reload_status, DATA_LEN_4,
							tmp_data);
		if (ret < NO_ERR) {
			E("%s: i2c access fail!\n", __func__);
			return HW_CRC_FAIL;
		}

		if ((tmp_data[0] & 0x01) != 0x01) {
			ret = himax_mcu_register_read(hdev, addr_reload_crc32_result,
						      DATA_LEN_4, tmp_data);

			if (ret < NO_ERR) {
				E("%s: i2c access fail!\n", __func__);
				return HW_CRC_FAIL;
			}
			I("%s:data[3]=0x%02X,data[2]=0x%02X,data[1]=0x%02X,data[0]=0x%02X\n",
			  __func__, tmp_data[3], tmp_data[2], tmp_data[1],
			  tmp_data[0]);
			result = ((tmp_data[3] << 24) + (tmp_data[2] << 16) +
				  (tmp_data[1] << 8) + tmp_data[0]);
			goto END;
		} else {
			usleep_range(1000, 1100);
			if (i_counter >= 100)
				himax_mcu_read_FW_status(hdev);
		}

	} while (i_counter++ < 100);
END:
	return result;
}

#define PRT_TMP_DATA "%s:[0]=0x%2X,[1]=0x%2X,	[2]=0x%2X,[3]=0x%2X\n"
void himax_mcu_reload_disable(struct himax_device *hdev, int disable)
{
	uint8_t data[DATA_LEN_4] = { 0 };

	if (disable) { /*reload disable*/
		himax_parse_assign_cmd(data_fw_define_flash_reload_dis, data,
				       sizeof(data));
		himax_mcu_register_write(hdev, addr_fw_define_flash_reload,
					 DATA_LEN_4, data);
	} else { /*reload enable*/
		himax_parse_assign_cmd(data_fw_define_flash_reload_en, data,
				       sizeof(data));
		himax_mcu_register_write(hdev, addr_fw_define_flash_reload,
					 DATA_LEN_4, data);
	}

	I("%s: setting OK!\n", __func__);
}

void himax_mcu_read_FW_ver(struct himax_device *hdev)
{
	struct himax_ic_data *ic_data = hdev->ic_data;
	uint8_t data[12] = { 0 };
	uint8_t pswd[DATA_LEN_4] = { 0 };
	uint8_t retry = 0;
	uint8_t reload_status = 0;

	while (reload_status == 0) {
		himax_mcu_register_read(hdev, addr_fw_define_2nd_flash_reload,
					DATA_LEN_4, pswd);

		if (pswd[1] == 0x72 && pswd[0] == 0xC0) {
			I("%s: FW finish reload done %d times\n", __func__,
			  retry);
			reload_status = 1;
			break;
		} else if (retry == 200) {
			E("%s: FW fail reload done !!!!!\n", __func__);
			himax_mcu_read_FW_status(hdev);
			ic_data->vendor_panel_ver = 0;
			ic_data->vendor_arch_ver = 0;
			ic_data->vendor_config_ver = 0;
			ic_data->vendor_touch_cfg_ver = 0;
			ic_data->vendor_display_cfg_ver = 0;
			ic_data->vendor_cid_maj_ver = 0;
			ic_data->vendor_cid_min_ver = 0;
			goto END;
		} else {
			retry++;
			usleep_range(10000, 11000);
		}
	}
	/*I("%s:pswd[0]=0x%2.2X,pswd[1]=0x%2.2X\n", __func__, pswd[0],
	 * pswd[1]);
	 */
	/*
	 * Read FW version
	 */
	himax_mcu_register_read(hdev, addr_fw_architecture_version, DATA_LEN_4, data);
	ic_data->vendor_panel_ver = data[0];
	ic_data->vendor_arch_ver = data[1] << 8 | data[2];

	himax_mcu_register_read(hdev, addr_fw_config_version, DATA_LEN_4, data);
	ic_data->vendor_config_ver = data[2] << 8 | data[3];
	ic_data->vendor_touch_cfg_ver = data[2];
	ic_data->vendor_display_cfg_ver = data[3];

	himax_mcu_register_read(hdev, addr_fw_CID, DATA_LEN_4, data);
	ic_data->vendor_cid_maj_ver = data[2];
	ic_data->vendor_cid_min_ver = data[3];

	himax_mcu_register_read(hdev, addr_fw_customer, 12, data);
	memcpy(ic_data->vendor_cus_info, data, 12);

	himax_mcu_register_read(hdev, addr_fw_project_name, 12, data);
	memcpy(ic_data->vendor_proj_info, data, 12);

	himax_mcu_register_read(hdev, addr_fw_config_date, 12, data);
	memcpy(ic_data->vendor_config_date, data, 12);

	if (ic_data->vendor_arch_ver >= 0x8098) {
		himax_mcu_register_read(hdev, addr_fw_remark1, 12, data);
		memcpy(ic_data->vendor_remark1, data, 12);

		himax_mcu_register_read(hdev, addr_fw_remark2, 12, data);
		memcpy(ic_data->vendor_remark2, data, 12);

		himax_mcu_register_read(hdev, addr_fw_ticket, 12, data);
		memcpy(ic_data->vendor_ticket, data, 12);
	}

	I("FW Architecture Version : %04X\n", ic_data->vendor_arch_ver);
	I("CID : %04X\n",
	  (ic_data->vendor_cid_maj_ver << 8 | ic_data->vendor_cid_min_ver));
	I("FW Display Config Version : D%02X\n", ic_data->vendor_display_cfg_ver);
	I("FW Touch Config Version : C%02X\n", ic_data->vendor_touch_cfg_ver);
	I("Panel Version : 0x%02X\n", ic_data->vendor_panel_ver);

	if (ic_data->vendor_arch_ver >= 0x8098) {
		I("Remark 1 : %s\n", ic_data->vendor_remark1);
		I("Remark 2 : %s\n", ic_data->vendor_remark2);
		I("Himax Ticket : %s\n", ic_data->vendor_ticket);
	}

	I("FW Config Date = %s\n", ic_data->vendor_config_date);
	I("Project = %s\n", ic_data->vendor_proj_info);
	I("Customer = %s\n", ic_data->vendor_cus_info);
	I("Himax Touch Driver Version = %s\n", HIMAX_DRIVER_VER);

END:
	return;
}

void himax_print_define_function(struct himax_device *hdev)
{
	struct himax_ts_data *ts = hdev->private_ts;

	I("HX_BOOT_UPGRADE : %d\n", HX_BOOT_UPGRADE);
	I("HX_EXCP_RECOVERY : %d\n", HX_EXCP_RECOVERY);
	I("HX_PROTOCOL_A : %d\n", HX_PROTOCOL_A);
	I("HX_PROTOCOL_B_3PA : %d\n", HX_PROTOCOL_B_3PA);
	I("HX_RST_PIN_FUNC : %d\n", HX_RST_PIN_FUNC);
	I("HX_TP_INSPECT_MODE : %d\n", HX_TP_INSPECT_MODE);
	I("HX_FIX_TOUCH_INFO : %d\n", HX_FIX_TOUCH_INFO);
	I("HX_WPBP_ENABLE : %d\n", HX_WPBP_ENABLE);
	I("HX_SMART_WAKEUP : %d\n", HX_SMART_WAKEUP);
	I("HX_GESTURE_TRACK : %d\n", HX_GESTURE_TRACK);
	I("GTS_range : %d\n", ts->GTS_range);
}

bool himax_mcu_read_event_stack(struct himax_device *hdev, uint8_t *buf,
			uint8_t length)
{
	struct himax_ts_data *ts = hdev->private_ts;
	int len = length;
	int i2c_speed = 0;
	ktime_t now;
	struct timespec64 timeStart, timeEnd, timeDelta;

	now = ktime_get();
	if (ts->debug_log_level & BIT(2))
		timeStart = ktime_to_timespec64(now);

	himax_bus_read(hdev, addr_read_event_stack, buf, length,
		       HIMAX_I2C_RETRY_TIMES);

	if (ts->debug_log_level & BIT(2)) {
		now = ktime_get();
		timeEnd = ktime_to_timespec64(now);
		timeDelta.tv_nsec = timeEnd.tv_nsec - timeStart.tv_nsec;

		i2c_speed =
			(len * 9 * 1000000 / (int)timeDelta.tv_nsec) * 13 / 10;
		ts->bus_speed = (int)i2c_speed;
	}

	return 1;
}

bool himax_mcu_calculateChecksum(struct himax_device *hdev, uint32_t size)
{
	uint8_t CRC_result = 0;
	uint8_t tmp_addr[DATA_LEN_4] = { 0 };

	I("%s:Now size= %dk\n", __func__, (size / 1024));
	himax_parse_assign_cmd(addr_program_reload_from, tmp_addr,
			       sizeof(tmp_addr));


	CRC_result = himax_mcu_check_CRC(hdev, tmp_addr, size);
	msleep(50);

	if (CRC_result != 0)
		I("%s: CRC Fail=%d\n", __func__, CRC_result);


	return (CRC_result == 0) ? true : false;
}

void himax_mcu_read_FW_status(struct himax_device *hdev)
{
	uint8_t len = 0;
	uint8_t i = 0;
	uint8_t data[DATA_LEN_4] = { 0 };

	len = (uint8_t)(sizeof(dbg_reg_ary) / sizeof(uint32_t));

	for (i = 0; i < len; i++) {
		himax_mcu_register_read(hdev, dbg_reg_ary[i], DATA_LEN_4, data);

		I("reg[0-3] : 0x%08X = 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
		  dbg_reg_ary[i], data[0], data[1], data[2], data[3]);
	}
}

void himax_mcu_irq_switch(struct himax_device *hdev, int switch_on)
{
	struct himax_ts_data *ts = hdev->private_ts;

	if (switch_on) {
		if (ts->use_irq)
			himax_int_enable(hdev, switch_on);
		else
			hrtimer_start(&ts->timer, ktime_set(1, 0),
				      HRTIMER_MODE_REL);
	} else {
		if (ts->use_irq)
			himax_int_enable(hdev, switch_on);
		else {
			hrtimer_cancel(&ts->timer);
			cancel_work_sync(&ts->work);
		}
	}
}

int himax_mcu_assign_sorting_mode(struct himax_device *hdev, uint8_t *tmp_data)
{
	I("%s:data[1]=0x%02X,data[0]=0x%02X\n", __func__, tmp_data[1],
	  tmp_data[0]);

	himax_mcu_register_write(hdev, addr_sorting_mode_en, DATA_LEN_4, tmp_data);

	return NO_ERR;
}

int himax_mcu_check_sorting_mode(struct himax_device *hdev, uint8_t *tmp_data)
{
	himax_mcu_register_read(hdev, addr_sorting_mode_en, DATA_LEN_4, tmp_data);
	I("%s: tmp_data[0]=0x%02X,tmp_data[1]=0x%02X\n", __func__, tmp_data[0],
	  tmp_data[1]);

	return NO_ERR;
}

#if (HX_RST_PIN_FUNC == 0x01)
void himax_mcu_tp_lcm_pin_reset(struct himax_device *hdev)
{
	struct himax_ts_data *ts = hdev->private_ts;

	I("%s: Now reset the Touch chip and LCM.\n", __func__);
	himax_gpio_set(ts->rst_gpio, 0);
	himax_gpio_set(ts->lcm_gpio, 0);
	msleep(60);
	himax_gpio_set(ts->lcm_gpio, 1);
	msleep(110);
	himax_gpio_set(ts->rst_gpio, 1);
	msleep(20); // because i2c will fail 1 time if no delay
}

static void himax_mcu_toggle_rst_gpio(struct himax_device *hdev)
{
	struct himax_ts_data *ts = hdev->private_ts;

	I("%s: Now reset the Touch chip.\n", __func__);
	if (ts->pdata->g_customer_control_tp_reset == 1) {
		/* please add control TP_EXT_RSTN function in here */
		E("%s: unable control TP_EXT_RSTN, please check it\n", __func__);
	} else {
		himax_gpio_set(ts->rst_gpio, 0);
		msleep(20);
		himax_gpio_set(ts->rst_gpio, 1);
		msleep(50);
	}
}

void himax_mcu_hw_reset(struct himax_device *hdev, uint8_t int_off)
{
	struct himax_ts_data *ts = hdev->private_ts;

	I("%s: int_off=%d\n", __func__, int_off);

	if (ts->rst_gpio >= 0) {
		if (int_off)
			himax_mcu_irq_switch(hdev, 0);

		himax_mcu_toggle_rst_gpio(hdev);

		if (int_off)
			himax_mcu_irq_switch(hdev, 1);
	}
#if defined(HIMAX_I2C_PLATFORM)
	himax_mcu_interface_on(hdev);
#endif
}
#elif (HX_RST_PIN_FUNC == 0x02)
	/* Need Customer set TP reset pin and follow sequence time
	 * Need at least two function control
	 * 1.himax_mcu_tp_lcm_pin_reset control TP and LCM reset pin for AP recovery
	 * 2.himax_mcu_toggle_rst_gpio simple tp hardware reset pin
	 */
#endif

void himax_mcu_tp_reset(struct himax_device *hdev)
{
	I("%s,Enter\n", __func__);
#if (HX_RST_PIN_FUNC == 0x01)
	himax_mcu_hw_reset(hdev, false);
#elif (HX_RST_PIN_FUNC == 0x02)
	/* Need Customer do TP reset pin */
#else
	himax_mcu_system_reset(hdev);
#endif
}
/*-------------------------------------------------------------------------
 *
 *	Create: Unknown
 *
 *	Description: Read related touch information from mcu or assign fixed values
 *				to ic_data value.
 *	Parameters: void
 *
 *	Returns: void
 *
 *	Side effects: None
 *
 */
void himax_mcu_touch_information(struct himax_device *hdev)
{
	struct himax_ts_data *ts = hdev->private_ts;
	struct himax_ic_data *ic_data = hdev->ic_data;
	uint8_t data[DATA_LEN_8] = { 0 };
#if (HX_FIX_TOUCH_INFO == 0x00)
	himax_mcu_register_read(hdev, addr_fw_define_rxnum_txnum_maxpt, DATA_LEN_8,
				data);
	ic_data->HX_RX_NUM = data[2];
	ic_data->HX_TX_NUM = data[3];
	ic_data->HX_MAX_PT = data[4];

	himax_mcu_register_read(hdev, addr_fw_define_xy_res, DATA_LEN_8, data);

	ic_data->HX_Y_RES = (data[6] << 8) | data[7];
	ic_data->HX_X_RES = (data[4] << 8) | data[5];

	himax_mcu_register_read(hdev, addr_fw_define_int_is_edge, DATA_LEN_4, data);
	ic_data->HX_INT_IS_EDGE = (data[1] & 0x01);

	himax_mcu_register_read(hdev, addr_HX_ID_EN, DATA_LEN_4, data);
	ic_data->HX_IS_ID_EN = ((data[1] & 0x02) >> 1);
	ic_data->HX_ID_PALM_EN = ((data[1] & 0x80) >> 7);

#else
	ic_data->HX_RX_NUM = FIX_HX_RX_NUM;
	ic_data->HX_TX_NUM = FIX_HX_TX_NUM;
	ic_data->HX_MAX_PT = FIX_HX_MAX_PT;
	ic_data->HX_INT_IS_EDGE = FIX_HX_INT_IS_EDGE;
	ic_data->HX_Y_RES = ts->pdata->screenHeight;
	ic_data->HX_X_RES = ts->pdata->screenWidth;
	ic_data->HX_IS_ID_EN = FIX_HX_IS_ID_EN;
	ic_data->HX_ID_PALM_EN = FIX_HX_ID_PALM_EN;
#endif

	ts->pdata->abs_x_min = 0;
	ts->pdata->abs_x_max = (ic_data->HX_X_RES - 1);
	ts->pdata->abs_y_min = 0;
	ts->pdata->abs_y_max = (ic_data->HX_Y_RES - 1);

	I("%s:HX_RX_NUM =%d,HX_TX_NUM =%d\n", __func__, ic_data->HX_RX_NUM,
	  ic_data->HX_TX_NUM);
	I("%s:HX_Y_RES=%d,HX_X_RES =%d,HX_INT_IS_EDGE =%d,\n", __func__,
	  ic_data->HX_Y_RES, ic_data->HX_X_RES, ic_data->HX_INT_IS_EDGE);
	I("%s:HX_IS_ID_EN=%d,HX_ID_PALM_EN =%d\n", __func__,
	  ic_data->HX_IS_ID_EN, ic_data->HX_ID_PALM_EN);

	himax_mcu_register_read(hdev, addr_HX_ID_EN, DATA_LEN_4, data);
	ic_data->STOP_FW_BY_HOST_EN = (data[1] & 0x01);
	I("%s:STOP_FW_BY_HOST_EN=%d\n", __func__,
	  ic_data->STOP_FW_BY_HOST_EN);
}

int himax_mcu_get_touch_data_size(void)
{
	return HIMAX_TOUCH_DATA_SIZE;
}

int himax_mcu_cal_data_len(int raw_cnt_rmd, int HX_MAX_PT, int raw_cnt_max)
{
	int RawDataLen;

	if (raw_cnt_rmd != 0x00)
		RawDataLen = MAX_I2C_TRANS_SZ -
			     ((HX_MAX_PT + raw_cnt_max + 3) * 4) - 1;
	else
		RawDataLen = MAX_I2C_TRANS_SZ -
			     ((HX_MAX_PT + raw_cnt_max + 2) * 4) - 1;

	return RawDataLen;
}

/* CORE_INIT */

int himax_mcu_in_cmd_struct_init(struct himax_device *hdev)
{
	int err = 0;

	hdev->internal_buffer =
		kzalloc(sizeof(uint8_t) * HX_MAX_WRITE_SZ, GFP_KERNEL);

	if (hdev->internal_buffer == NULL) {
		err = -ENOMEM;
		goto err_g_core_cmd_op_g_internal_buffer_fail;
	}

	return NO_ERR;

err_g_core_cmd_op_g_internal_buffer_fail:

	return err;
}

void himax_mcu_in_cmd_struct_free(struct himax_device *hdev)
{
	kfree(hdev->internal_buffer);
	hdev->internal_buffer = NULL;

	I("%s: release completed\n", __func__);
}

