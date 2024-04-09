// SPDX-License-Identifier: GPL-2.0
/*  Himax Android Driver Sample Code for common functions
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

#define SUPPORT_FINGER_DATA_CHECKSUM 0x0F
#define TS_WAKE_LOCK_TIMEOUT (5000)
#define FRAME_COUNT 5

int g_ts_dbg;

struct timespec64 time_diff(struct timespec64 start, struct timespec64 end)
{
	struct timespec64 delta;

	if ((end.tv_nsec - start.tv_nsec) < 0) {
		delta.tv_sec = end.tv_sec - start.tv_sec - 1;
		delta.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
	} else {
		delta.tv_sec = end.tv_sec - start.tv_sec;
		delta.tv_nsec = end.tv_nsec - start.tv_nsec;
	}
	return delta;
}

void himax_parse_assign_cmd(uint32_t addr, uint8_t *cmd, int len)
{
	/*I("%s: Entering!\n", __func__);*/
	switch (len) {
	case 1:
		cmd[0] = addr;
		/*I("%s: cmd[0] = 0x%02X\n", __func__, cmd[0]);*/
		break;
	case 2:
		cmd[0] = addr % 0x100;
		cmd[1] = (addr >> 8) % 0x100;
		/*I("%s: cmd[0] = 0x%02X,cmd[1] = 0x%02X\n",*/
		/*	__func__, cmd[0], cmd[1]);*/
		break;
	case 4:
		cmd[0] = addr % 0x100;
		cmd[1] = (addr >> 8) % 0x100;
		cmd[2] = (addr >> 16) % 0x100;
		cmd[3] = addr / 0x1000000;
		/*  I("%s: cmd[0] = 0x%02X,cmd[1] = 0x%02X,*/
		/*cmd[2] = 0x%02X,cmd[3] = 0x%02X\n", */
		/* __func__, cmd[0], cmd[1], cmd[2], cmd[3]);*/
		break;
	default:
		E("%s: input length fault,len = %d!\n", __func__, len);
	}
}
//EXPORT_SYMBOL(himax_parse_assign_cmd);

int himax_input_register(struct himax_ts_data *ts)
{
	int ret = 0;

	ret = himax_dev_set(ts);

	if (ret < 0) {
		I("%s, input device register fail!\n", __func__);
		ret = INPUT_REGISTER_FAIL;
		goto input_device_fail;
	}

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(KEY_APPSELECT, ts->input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
#if (HX_PROTOCOL_A == 0x01)
	/*ts->input_dev->mtsize = ts->nFinger_support;*/
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 3, 0, 0);
#else
	set_bit(MT_TOOL_FINGER, ts->input_dev->keybit);
#if (HX_PROTOCOL_B_3PA == 0x01)
	input_mt_init_slots(ts->input_dev, ts->nFinger_support,
			    INPUT_MT_DIRECT);
#else
	input_mt_init_slots(ts->input_dev, ts->nFinger_support);
#endif
#endif
	I("%s: mix_x %d, max_x %d, min_y %d, max_y %d\n", __func__,
	  ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min,
	  ts->pdata->abs_y_max);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
			     ts->pdata->abs_x_min, ts->pdata->abs_x_max,
			     ts->pdata->abs_x_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
			     ts->pdata->abs_y_min, ts->pdata->abs_y_max,
			     ts->pdata->abs_y_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
			     ts->pdata->abs_pressure_min,
			     ts->pdata->abs_pressure_max,
			     ts->pdata->abs_pressure_fuzz, 0);
#if (HX_PROTOCOL_A == 0x00)
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE,
			     ts->pdata->abs_pressure_min,
			     ts->pdata->abs_pressure_max,
			     ts->pdata->abs_pressure_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,
			     ts->pdata->abs_width_min, ts->pdata->abs_width_max,
			     ts->pdata->abs_pressure_fuzz, 0);
#endif
	/*	input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE, 0,*/
	/*			((ts->pdata->abs_pressure_max << 16)*/
	/*			| ts->pdata->abs_width_max),*/
	/*			0, 0);*/
	/*	input_set_abs_params(ts->input_dev, ABS_MT_POSITION,*/
	/*			0, (BIT(31)*/
	/*			| (ts->pdata->abs_x_max << 16)*/
	/*			| ts->pdata->abs_y_max),*/
	/*			0, 0);*/

	if (himax_input_register_device(ts->input_dev) == 0) {
		ret = NO_ERR;
	} else {
		E("%s: input register fail\n", __func__);
		ret = INPUT_REGISTER_FAIL;
		goto input_device_fail;
	}

input_device_fail:
	return ret;
}
//EXPORT_SYMBOL(himax_input_register);

static void himax_calcDataSize(struct himax_device *hdev)
{
	struct himax_ts_data *ts_data = hdev->private_ts;
	struct himax_ic_data *ic_data = hdev->ic_data;

	ts_data->x_channel = ic_data->HX_RX_NUM;
	ts_data->y_channel = ic_data->HX_TX_NUM;
	ts_data->nFinger_support = ic_data->HX_MAX_PT;

	ts_data->coord_data_size = 4 * ts_data->nFinger_support;
	ts_data->area_data_size = ((ts_data->nFinger_support / 4) +
				   (ts_data->nFinger_support % 4 ? 1 : 0)) *
				  4;
	ts_data->coordInfoSize =
		ts_data->coord_data_size + ts_data->area_data_size + 4;
	ts_data->raw_data_frame_size = 128 - ts_data->coord_data_size -
				       ts_data->area_data_size - 4 - 4 - 1;

	if (ts_data->raw_data_frame_size == 0) {
		E("%s: could NOT calculate!\n", __func__);
		return;
	}

	I("%s: coord_dsz:%d,area_dsz:%d,raw_data_fsz:%d", __func__,
	  ts_data->coord_data_size, ts_data->area_data_size,
	  ts_data->raw_data_frame_size);
}

static void calculate_point_number(struct himax_device *hdev)
{
	struct himax_ic_data *ic_data = hdev->ic_data;

	hdev->hx_touch_info_point_cnt = ic_data->HX_MAX_PT * 4;

	if ((ic_data->HX_MAX_PT % 4) == 0)
		hdev->hx_touch_info_point_cnt += (ic_data->HX_MAX_PT / 4) * 4;
	else
		hdev->hx_touch_info_point_cnt += ((ic_data->HX_MAX_PT / 4) + 1) * 4;
}

/*
 *static int himax_loadSensorConfig(struct himax_i2c_platform_data *pdata)
 *{
 *	I("%s: initialization complete\n", __func__);
 *	return NO_ERR;
 *}
 */

int himax_report_data_init(struct himax_device *hdev)
{
	struct himax_report_data *hx_touch_data = hdev->hx_touch_data;
	struct himax_ic_data *ic_data = hdev->ic_data;

	if (hx_touch_data->hx_coord_buf != NULL) {
		kfree(hx_touch_data->hx_coord_buf);
		hx_touch_data->hx_coord_buf = NULL;
	}
	if (hx_touch_data->hx_rawdata_buf != NULL) {
		kfree(hx_touch_data->hx_rawdata_buf);
		hx_touch_data->hx_rawdata_buf = NULL;
	}

	hx_touch_data->touch_all_size = himax_mcu_get_touch_data_size();
	hx_touch_data->raw_cnt_max = ic_data->HX_MAX_PT / 4;
	hx_touch_data->raw_cnt_rmd = ic_data->HX_MAX_PT % 4;
	/* more than 4 fingers */
	if (hx_touch_data->raw_cnt_rmd != 0x00) {
		hx_touch_data->rawdata_size =
			himax_mcu_cal_data_len(hx_touch_data->raw_cnt_rmd,
					       ic_data->HX_MAX_PT,
					       hx_touch_data->raw_cnt_max);

		hx_touch_data->touch_info_size =
			(ic_data->HX_MAX_PT + hx_touch_data->raw_cnt_max + 2) *
			4;
	} else { /* less than 4 fingers */
		hx_touch_data->rawdata_size =
			himax_mcu_cal_data_len(hx_touch_data->raw_cnt_rmd,
					       ic_data->HX_MAX_PT,
					       hx_touch_data->raw_cnt_max);

		hx_touch_data->touch_info_size =
			(ic_data->HX_MAX_PT + hx_touch_data->raw_cnt_max + 1) *
			4;
	}
	if ((ic_data->HX_TX_NUM * ic_data->HX_RX_NUM + ic_data->HX_TX_NUM +
	     ic_data->HX_RX_NUM) %
		    hx_touch_data->rawdata_size ==
	    0)
		hx_touch_data->rawdata_frame_size =
			(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM +
			 ic_data->HX_TX_NUM + ic_data->HX_RX_NUM) /
			hx_touch_data->rawdata_size;
	else
		hx_touch_data->rawdata_frame_size =
			(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM +
			 ic_data->HX_TX_NUM + ic_data->HX_RX_NUM) /
				hx_touch_data->rawdata_size +
			1;

	I("%s:rawdata_fsz = %d,HX_MAX_PT:%d,hx_raw_cnt_max:%d\n", __func__,
	  hx_touch_data->rawdata_frame_size, ic_data->HX_MAX_PT,
	  hx_touch_data->raw_cnt_max);
	I("%s:hx_raw_cnt_rmd:%d,g_hx_rawdata_size:%d,touch_info_size:%d\n",
	  __func__, hx_touch_data->raw_cnt_rmd, hx_touch_data->rawdata_size,
	  hx_touch_data->touch_info_size);

	hx_touch_data->hx_coord_buf = kzalloc(
		sizeof(uint8_t) * (hx_touch_data->touch_info_size), GFP_KERNEL);

	if (hx_touch_data->hx_coord_buf == NULL)
		goto mem_alloc_fail_coord_buf;

	hx_touch_data->hx_rawdata_buf =
		kzalloc(sizeof(uint8_t) * (hx_touch_data->touch_all_size -
					   hx_touch_data->touch_info_size),
			GFP_KERNEL);
	if (hx_touch_data->hx_rawdata_buf == NULL)
		goto mem_alloc_fail_rawdata_buf;

	if (hdev->target_report_data == NULL) {
		hdev->target_report_data = kzalloc(
			sizeof(struct himax_target_report_data), GFP_KERNEL);
		if (hdev->target_report_data == NULL)
			goto mem_alloc_fail_report_data;

		hdev->target_report_data->x =
			kzalloc(sizeof(int) * (ic_data->HX_MAX_PT), GFP_KERNEL);
		if (hdev->target_report_data->x == NULL)
			goto mem_alloc_fail_report_data_x;

		hdev->target_report_data->y =
			kzalloc(sizeof(int) * (ic_data->HX_MAX_PT), GFP_KERNEL);
		if (hdev->target_report_data->y == NULL)
			goto mem_alloc_fail_report_data_y;

		hdev->target_report_data->w =
			kzalloc(sizeof(int) * (ic_data->HX_MAX_PT), GFP_KERNEL);
		if (hdev->target_report_data->w == NULL)
			goto mem_alloc_fail_report_data_w;

		hdev->target_report_data->finger_id =
			kzalloc(sizeof(int) * (ic_data->HX_MAX_PT), GFP_KERNEL);
		if (hdev->target_report_data->finger_id == NULL)
			goto mem_alloc_fail_report_data_fid;
	}
	if (hdev->fixed_point_label == NULL) {
		hdev->fixed_point_label = kzalloc(
			sizeof(struct himax_target_report_data), GFP_KERNEL);
		if (hdev->fixed_point_label == NULL)
			goto mem_alloc_fail_pre_report_data;

		hdev->fixed_point_label->x =
			kzalloc(sizeof(int) * (ic_data->HX_MAX_PT), GFP_KERNEL);
		if (hdev->fixed_point_label->x == NULL)
			goto mem_alloc_fail_pre_report_data_x;

		hdev->fixed_point_label->y =
			kzalloc(sizeof(int) * (ic_data->HX_MAX_PT), GFP_KERNEL);
		if (hdev->fixed_point_label->y == NULL)
			goto mem_alloc_fail_pre_report_data_y;

		hdev->fixed_point_label->finger_id =
			kzalloc(sizeof(int) * (ic_data->HX_MAX_PT), GFP_KERNEL);
		if (hdev->fixed_point_label->finger_id == NULL)
			goto mem_alloc_fail_pre_report_data_fid;
		hdev->fixed_point_label->fpt_cnt =
			kzalloc(sizeof(int) * (ic_data->HX_MAX_PT), GFP_KERNEL);
		if (hdev->fixed_point_label->fpt_cnt == NULL)
			goto mem_alloc_fail_pre_report_data_fpt_cnt;
	}
	memset(hdev->fixed_point_label->x, 0xFFFF, ic_data->HX_MAX_PT);
	memset(hdev->fixed_point_label->y, 0xFFFF, ic_data->HX_MAX_PT);
	return NO_ERR;
mem_alloc_fail_pre_report_data_fpt_cnt:
	kfree(hdev->fixed_point_label->finger_id);
	hdev->fixed_point_label->finger_id = NULL;
mem_alloc_fail_pre_report_data_fid:
	kfree(hdev->fixed_point_label->y);
	hdev->fixed_point_label->y = NULL;
mem_alloc_fail_pre_report_data_y:
	kfree(hdev->fixed_point_label->x);
	hdev->fixed_point_label->x = NULL;
mem_alloc_fail_pre_report_data_x:
	kfree(hdev->fixed_point_label);
	hdev->fixed_point_label = NULL;
mem_alloc_fail_pre_report_data:
	kfree(hdev->target_report_data->finger_id);
	hdev->target_report_data->finger_id = NULL;
mem_alloc_fail_report_data_fid:
	kfree(hdev->target_report_data->w);
	hdev->target_report_data->w = NULL;
mem_alloc_fail_report_data_w:
	kfree(hdev->target_report_data->y);
	hdev->target_report_data->y = NULL;
mem_alloc_fail_report_data_y:
	kfree(hdev->target_report_data->x);
	hdev->target_report_data->x = NULL;
mem_alloc_fail_report_data_x:
	kfree(hdev->target_report_data);
	hdev->target_report_data = NULL;
mem_alloc_fail_report_data:
	kfree(hx_touch_data->hx_rawdata_buf);
	hx_touch_data->hx_rawdata_buf = NULL;
mem_alloc_fail_rawdata_buf:
	kfree(hx_touch_data->hx_coord_buf);
	hx_touch_data->hx_coord_buf = NULL;
mem_alloc_fail_coord_buf:

	E("%s: Failed to allocate memory\n", __func__);
	return MEM_ALLOC_FAIL;
}
//EXPORT_SYMBOL(himax_report_data_init);

void himax_report_data_deinit(struct himax_device *hdev)
{
	struct himax_target_report_data *target_report_data = hdev->target_report_data;
	struct himax_target_report_data *fixed_point_label = hdev->fixed_point_label;
	struct himax_report_data *hx_touch_data = hdev->hx_touch_data;

	kfree(target_report_data->finger_id);
	target_report_data->finger_id = NULL;
	kfree(target_report_data->w);
	target_report_data->w = NULL;
	kfree(target_report_data->y);
	target_report_data->y = NULL;
	kfree(target_report_data->x);
	target_report_data->x = NULL;
	kfree(target_report_data);
	target_report_data = NULL;

	kfree(fixed_point_label->fpt_cnt);
	fixed_point_label->fpt_cnt = NULL;
	kfree(fixed_point_label->finger_id);
	fixed_point_label->finger_id = NULL;
	kfree(fixed_point_label->y);
	fixed_point_label->y = NULL;
	kfree(fixed_point_label->x);
	fixed_point_label->x = NULL;
	kfree(fixed_point_label);
	fixed_point_label = NULL;
	kfree(hx_touch_data->hx_rawdata_buf);
	hx_touch_data->hx_rawdata_buf = NULL;
	kfree(hx_touch_data->hx_coord_buf);
	hx_touch_data->hx_coord_buf = NULL;
}

static int himax_ts_work_status(struct himax_device *hdev)
{
	struct himax_ts_data *ts = hdev->private_ts;
	struct himax_report_data *hx_touch_data = hdev->hx_touch_data;

	/* 1: normal */
	int result = HX_REPORT_COORD;

	hx_touch_data->diag_cmd = ts->diag_cmd;
	if (hx_touch_data->diag_cmd)
		result = HX_REPORT_COORD_RAWDATA;

	/* I("Now Status is %d\n", result); */
	return result;
}

static int himax_touch_get(struct himax_device *hdev, uint8_t *buf, int ts_path,
			   int ts_status)
{
	struct himax_report_data *hx_touch_data = hdev->hx_touch_data;

	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	switch (ts_path) {
	/*normal*/
	case HX_REPORT_COORD:
		if (!himax_mcu_read_event_stack(hdev,
			    buf, hx_touch_data->touch_info_size)) {
			E("%s: can't read data from chip!\n", __func__);
			ts_status = HX_TS_GET_DATA_FAIL;
		}
		break;

	case HX_REPORT_COORD_RAWDATA:
		if (!himax_mcu_read_event_stack(hdev, buf, 128)) {
			E("%s: can't read data from chip!\n", __func__);
			ts_status = HX_TS_GET_DATA_FAIL;
		}
		break;
	default:
		break;
	}

	return ts_status;
}

/* start error_control*/
static int himax_checksum_cal(struct himax_device *hdev, uint8_t *buf,
			      int ts_path, int ts_status)
{
	struct himax_ts_data *ts = hdev->private_ts;
	struct himax_report_data *hx_touch_data = hdev->hx_touch_data;
	uint16_t check_sum_cal = 0;
	int32_t i = 0;
	int length = 0;
	int zero_cnt = 0;
	int raw_data_sel = 0;
	int ret_val = ts_status;

	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	/* Normal */
	switch (ts_path) {
	case HX_REPORT_COORD:
		length = hx_touch_data->touch_info_size;
		break;
	case HX_REPORT_COORD_RAWDATA:
		length = hx_touch_data->touch_info_size;
		break;
	default:
		I("%s, Normal error!\n", __func__);
		ret_val = HX_PATH_FAIL;
		goto END_FUNCTION;
	}

	for (i = 0; i < length; i++) {
		check_sum_cal += buf[i];
		if (buf[i] == 0x00)
			zero_cnt++;
	}

	if (check_sum_cal % 0x100 != 0) {
		I("point data_checksum not match check_sum_cal: 0x%02X",
		  check_sum_cal);
		ret_val = HX_CHKSUM_FAIL;
	} else if (zero_cnt == length) {
		if (ts->use_irq)
			I("[HIMAX TP MSG] All Zero event\n");

		ret_val = HX_CHKSUM_FAIL;
	} else {
		raw_data_sel = buf[hdev->hx_touch_info_point_cnt] >> 4 & 0x0F;
		/*I("%s:raw_out_sel=%x , hx_touch_data->diag_cmd=%x.\n",*/
		/*		__func__, raw_data_sel,*/
		/*		hx_touch_data->diag_cmd);*/
		/*raw data out not match skip it*/
		if ((raw_data_sel != 0x0F) &&
		    (raw_data_sel != hx_touch_data->diag_cmd)) {
			/*I("%s:raw data out not match.\n", __func__);*/
			if (!hx_touch_data->diag_cmd) {
				/*Need to clear event stack here*/
				himax_mcu_read_event_stack(hdev,
					buf,
					(128 - hx_touch_data->touch_info_size));
				/*I("%s: size =%d, buf[0]=%x ,buf[1]=%x,*/
				/*	buf[2]=%x, buf[3]=%x.\n",*/
				/*	__func__,*/
				/*	(128-hx_touch_data->touch_info_size),*/
				/*	buf[0], buf[1], buf[2], buf[3]);*/
				/*I("%s:also clear event stack.\n", __func__);*/
			}
			ret_val = HX_READY_SERVE;
		}
	}

END_FUNCTION:
	if (g_ts_dbg != 0)
		I("%s: END, ret_val=%d!\n", __func__, ret_val);
	return ret_val;
}

static int himax_err_ctrl(struct himax_device *hdev, uint8_t *buf, int ts_path,
			  int ts_status)
{
	ts_status = himax_checksum_cal(hdev, buf, ts_path, ts_status);
	if (ts_status == HX_CHKSUM_FAIL)
		goto CHK_FAIL;
	else
		goto END_FUNCTION;

CHK_FAIL:
END_FUNCTION:
	if (g_ts_dbg != 0)
		I("%s: END, ts_status=%d!\n", __func__, ts_status);
	return ts_status;
}
/* end error_control*/

/* start distribute_data*/
static int himax_distribute_touch_data(struct himax_device *hdev, uint8_t *buf,
			int ts_path, int ts_status)
{
	struct himax_report_data *hx_touch_data = hdev->hx_touch_data;
	uint8_t hx_state_info_pos = hx_touch_data->touch_info_size - 3;

	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	if (ts_path == HX_REPORT_COORD) {
		memcpy(hx_touch_data->hx_coord_buf, &buf[0],
		       hx_touch_data->touch_info_size);

		if (buf[hx_state_info_pos] != 0xFF &&
		    buf[hx_state_info_pos + 1] != 0xFF)
			memcpy(hx_touch_data->hx_state_info,
			       &buf[hx_state_info_pos], 2);
		else
			memset(hx_touch_data->hx_state_info, 0x00,
			       sizeof(hx_touch_data->hx_state_info));
	} else if (ts_path == HX_REPORT_COORD_RAWDATA) {
		memcpy(hx_touch_data->hx_coord_buf, &buf[0],
		       hx_touch_data->touch_info_size);

		if (buf[hx_state_info_pos] != 0xFF &&
		    buf[hx_state_info_pos + 1] != 0xFF)
			memcpy(hx_touch_data->hx_state_info,
			       &buf[hx_state_info_pos], 2);
		else
			memset(hx_touch_data->hx_state_info, 0x00,
			       sizeof(hx_touch_data->hx_state_info));

		memcpy(hx_touch_data->hx_rawdata_buf,
		       &buf[hx_touch_data->touch_info_size],
		       hx_touch_data->touch_all_size -
			       hx_touch_data->touch_info_size);
	} else {
		E("%s, Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
	}

	if (g_ts_dbg != 0)
		I("%s: End, ts_status=%d!\n", __func__, ts_status);
	return ts_status;
}
/* end assign_data*/

/* start parse_report_data*/
int himax_parse_report_points(struct himax_device *hdev, int ts_path,
			      int ts_status)
{
	struct himax_ts_data *ts = hdev->private_ts;
	struct himax_report_data *hx_touch_data = hdev->hx_touch_data;
	struct himax_ic_data *ic_data = hdev->ic_data;
	struct himax_target_report_data *target_report_data = hdev->target_report_data;
	int x = 0, y = 0, w = 0;
	int base = 0;
	int event_id = 0, palm_id = 0;
	int32_t loop_i = 0;

	if (g_ts_dbg != 0)
		I("%s: start! %p\n", __func__, target_report_data);

	ts->old_finger = ts->pre_finger_mask;
	if (ts->hx_point_num == 0) {
		if (g_ts_dbg != 0)
			I("%s: hx_point_num = 0!\n", __func__);
		return ts_status;
	}
	ts->pre_finger_mask = 0;
	hx_touch_data->finger_on = 1;

	target_report_data->finger_num = ts->hx_point_num;
	target_report_data->finger_on = hx_touch_data->finger_on;

	if (g_ts_dbg != 0)
		I("%s:finger_num = 0x%2X, finger_on = %d\n", __func__, target_report_data->finger_num, target_report_data->finger_on);

	for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
		base = loop_i * 4;
		x = hx_touch_data->hx_coord_buf[base] << 8 |
		    hx_touch_data->hx_coord_buf[base + 1];
		y = (hx_touch_data->hx_coord_buf[base + 2] << 8 |
		     hx_touch_data->hx_coord_buf[base + 3]);

		w = hx_touch_data
			    ->hx_coord_buf[(ts->nFinger_support * 4) + loop_i];

		if (g_ts_dbg != 0)
			D("%s: now parsing[%d]:x=%d, y=%d, w=%d\n", __func__,
			  loop_i, x, y, w);

		if (ic_data->HX_IS_ID_EN) {
			event_id = hx_touch_data->hx_coord_buf[base] >> 0x06;
			x = (hx_touch_data->hx_coord_buf[base] & 0x3f)  << 8 | hx_touch_data->hx_coord_buf[base + 1];

			if (ic_data->HX_ID_PALM_EN) {
				palm_id = hx_touch_data->hx_coord_buf[base + 2] >> 0x06;
				y = (hx_touch_data->hx_coord_buf[base + 2] & 0x3f)  << 8 | hx_touch_data->hx_coord_buf[base + 3];
			}

			if ((event_id == 0) || (event_id == 3)) { /*No touch event or Leave event*/
				x = 0xFFFF;
				y = 0xFFFF;
			}
			if (g_ts_dbg != 0) {
				switch (event_id) {
				case 1:
					I("%s Event Enter!\n", __func__);
					break;
				case 2:
					I("%s Event Moving!\n", __func__);
					break;
				case 3:
					I("%s Event Leave!\n", __func__);
					break;
				default:
					break;
				}

				if (ic_data->HX_ID_PALM_EN) {
					if (palm_id == 1)
						I("Plam event\n");
				}
				I("Parsing[%d]:x=%d, y=%d, event_id=%d, palm_id=%d\n", loop_i, x, y, event_id, palm_id);
			}
		}

		if (x >= 0 && x <= ts->pdata->abs_x_max && y >= 0 &&
		    y <= ts->pdata->abs_y_max) {

			target_report_data->x[loop_i] = x;
			target_report_data->y[loop_i] = y;
			target_report_data->w[loop_i] = w;
			target_report_data->finger_id[loop_i] = 1;

			if (!ts->first_pressed) {
				ts->first_pressed = 1;
				I("S1@%d, %d\n", x, y);
			}

			ts->pre_finger_data[loop_i][0] = x;
			ts->pre_finger_data[loop_i][1] = y;

			ts->pre_finger_mask =
				ts->pre_finger_mask + (1 << loop_i);
		} else { /* report coordinates */
			target_report_data->x[loop_i] = x;
			target_report_data->y[loop_i] = y;
			target_report_data->w[loop_i] = w;
			target_report_data->finger_id[loop_i] = 0;

			if (loop_i == 0 && ts->first_pressed == 1) {
				ts->first_pressed = 2;
				I("E1@%d, %d\n", ts->pre_finger_data[0][0],
				  ts->pre_finger_data[0][1]);
			}
		}
	}

	if (g_ts_dbg != 0) {
		for (loop_i = 0; loop_i < 10; loop_i++)
			D("DBG X=%d  Y=%d ID=%d\n",
			  target_report_data->x[loop_i],
			  target_report_data->y[loop_i],
			  target_report_data->finger_id[loop_i]);

		D("DBG finger number %d\n", target_report_data->finger_num);
	}

	if (g_ts_dbg != 0)
		I("%s: end!\n", __func__);
	return ts_status;
}

static int himax_parse_report_data(struct himax_device *hdev, int ts_path,
				   int ts_status)
{
	struct himax_ts_data *ts = hdev->private_ts;
	struct himax_report_data *hx_touch_data = hdev->hx_touch_data;

	if (g_ts_dbg != 0)
		I("%s: start now_status=%d!\n", __func__, ts_status);

	hdev->en_noise_filter =
		(hx_touch_data->hx_coord_buf[hdev->hx_touch_info_point_cnt + 2] >> 3);
	/* I("EN_NoiseFilter=%d\n", hdev->en_noise_filter); */
	hdev->en_noise_filter = hdev->en_noise_filter & 0x01;
	/* I("EN_NoiseFilter2=%d\n", EN_NoiseFilter); */

	if (hx_touch_data->hx_coord_buf[hdev->hx_touch_info_point_cnt] == 0xff)
		ts->hx_point_num = 0;
	else
		ts->hx_point_num =
			hx_touch_data->hx_coord_buf[hdev->hx_touch_info_point_cnt] &
			0x0f;

	switch (ts_path) {
	case HX_REPORT_COORD:
		ts_status = himax_parse_report_points(hdev, ts_path, ts_status);
		break;
	case HX_REPORT_COORD_RAWDATA:
		/* touch monitor rawdata */
		ts_status = himax_parse_report_points(hdev, ts_path, ts_status);
		break;
	default:
		E("%s:Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
		break;
	}
	if (g_ts_dbg != 0)
		I("%s: end now_status=%d!\n", __func__, ts_status);
	return ts_status;
}

/* end parse_report_data*/

void himax_report_all_leave_event(struct himax_ts_data *ts)
{
	int loop_i = 0;

	for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
#if (HX_PROTOCOL_A == 0x00)
		input_mt_slot(ts->input_dev, loop_i);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
#endif
	}
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_sync(ts->input_dev);
}

void himax_mcu_clear_event_stack(struct himax_device *hdev)
{
	uint32_t addr_clear_event_stack = 0x80060014;
	uint8_t data[DATA_LEN_4] = {0};

	himax_mcu_register_read(hdev, addr_clear_event_stack, DATA_LEN_4, data);
	data[0] |= 0x02;
	himax_mcu_register_write(hdev, addr_clear_event_stack, DATA_LEN_4, data);
}

/* start report_point*/
static void himax_finger_report(struct himax_device *hdev)
{
	struct himax_ts_data *ts = hdev->private_ts;
	struct himax_target_report_data *target_report_data = hdev->target_report_data;
	struct himax_target_report_data *fixed_point_label = hdev->fixed_point_label;
	int i = 0;
	bool valid = false;
	int debounce_cnt = 3620; /* 3620 * 8.3ms = 30 sec. */

	if (g_ts_dbg != 0)
		I("%s:start ts->hx_point_num=%d\n", __func__, ts->hx_point_num);


	for (i = 0; i < ts->nFinger_support; i++) {
		if (ts->GTS_range > 0) {
			if ((target_report_data->x[i] != 0xFFFF) ||
			    (target_report_data->y[i] != 0xFFFF)) {
				/*
				 *I("report_data->x[i]=%d,y[i]=%d",
					target_report_data->x[i],
					target_report_data->y[i]);
				I("pre_report_data->pre_x[i]=%d,pre_y[i]=%d",
					fixed_point_label->x[i],
					fixed_point_label->y[i]);
				*/
				if ((fixed_point_label->x[i] == 0xFFFF) &&
				    (fixed_point_label->y[i] == 0xFFFF)) {
					/*I("fixed point appears\n");*/
					fixed_point_label->fpt_cnt[i] = 0;
					fixed_point_label->x[i] =
						target_report_data->x[i];
					fixed_point_label->y[i] =
						target_report_data->y[i];
				}
				if ((target_report_data->x[i] -
					     fixed_point_label->x[i] <
				     ts->GTS_range) &&
				    (fixed_point_label->x[i] -
					     target_report_data->x[i] <
				     ts->GTS_range) &&
				    (fixed_point_label->y[i] -
					     target_report_data->y[i] <
				     ts->GTS_range) &&
				    (target_report_data->y[i] -
					     fixed_point_label->y[i] <
				     ts->GTS_range)) { /* in range */
					fixed_point_label->fpt_cnt[i]++;
					/*I("fpt_cnt[%d] = %d\n", i, fixed_point_label->fpt_cnt[i]);*/
				} else { /* out of range */
					fixed_point_label->fpt_cnt[i] = 0;
					fixed_point_label->x[i] = 0xFFFF;
					fixed_point_label->y[i] = 0xFFFF;
				}
			} else { /* finger leave */
				fixed_point_label->fpt_cnt[i] = 0;
				fixed_point_label->x[i] = 0xFFFF;
				fixed_point_label->y[i] = 0xFFFF;
			}
			if (fixed_point_label->fpt_cnt[i] >
			    debounce_cnt) { /* 3620 * 8.3ms = 30 sec. */
				I("[Ghost] point happens !!!");
				fixed_point_label->fpt_cnt[i] = 0;
				himax_report_all_leave_event(ts);
				himax_mcu_system_reset(hdev);
				return;
			}
		}
		if (target_report_data->x[i] >= 0 &&
		    target_report_data->x[i] <= ts->pdata->abs_x_max &&
		    target_report_data->y[i] >= 0 &&
		    target_report_data->y[i] <= ts->pdata->abs_y_max)
			valid = true;
		else
			valid = false;
		if (g_ts_dbg != 0)
			I("valid=%d\n", valid);
		if (valid) {
			if (g_ts_dbg != 0) {
				I("report_data->x[i]=%d,y[i]=%d,w[i]=%d",
				  target_report_data->x[i],
				  target_report_data->y[i],
				  target_report_data->w[i]);
			}
#if (HX_PROTOCOL_A == 0x00)
			input_mt_slot(ts->input_dev, i);
#else
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
					 target_report_data->w[i]);
#if (HX_PROTOCOL_A == 0x00)
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
					 target_report_data->w[i]);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
					 target_report_data->w[i]);
#else
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
#endif
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					 target_report_data->x[i]);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					 target_report_data->y[i]);
#if (HX_PROTOCOL_A == 0x00)
			ts->last_slot = i;
			input_mt_report_slot_state(ts->input_dev,
						   MT_TOOL_FINGER, 1);
#else
			input_mt_sync(ts->input_dev);
#endif
		} else {
#if (HX_PROTOCOL_A == 0x00)
			input_mt_slot(ts->input_dev, i);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input_dev,
						   MT_TOOL_FINGER, 0);
#endif
		}
	}
#if (HX_PROTOCOL_A == 0x00)
	input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif
	input_sync(ts->input_dev);

	if (g_ts_dbg != 0)
		I("%s:end\n", __func__);
}

static void himax_finger_leave(struct himax_device *hdev)
{
	struct himax_ts_data *ts = hdev->private_ts;
	struct himax_target_report_data *target_report_data = hdev->target_report_data;
	struct himax_report_data *hx_touch_data = hdev->hx_touch_data;
#if (HX_PROTOCOL_A == 0x00)
	int32_t loop_i = 0;
#endif

	if (g_ts_dbg != 0)
		I("%s: start!\n", __func__);

	hx_touch_data->finger_on = 0;
	target_report_data->finger_on = 0;
	target_report_data->finger_num = 0;

#if (HX_PROTOCOL_A == 0x01)
	input_mt_sync(ts->input_dev);
#endif
#if (HX_PROTOCOL_A == 0x00)
	for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
		input_mt_slot(ts->input_dev, loop_i);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
#endif
	if (ts->pre_finger_mask > 0)
		ts->pre_finger_mask = 0;

	if (ts->first_pressed == 1) {
		ts->first_pressed = 2;
		I("E1@%d, %d\n", ts->pre_finger_data[0][0],
		  ts->pre_finger_data[0][1]);
	}

	/*if (ts->debug_log_level & BIT(1))*/
	/*	himax_log_touch_event(x, y, w, loop_i, EN_NoiseFilter,*/
	/*			HX_FINGER_LEAVE); */

	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_sync(ts->input_dev);

	if (g_ts_dbg != 0)
		I("%s: end!\n", __func__);
}

static void himax_report_points(struct himax_device *hdev)
{
	struct himax_ts_data *ts = hdev->private_ts;

	if (g_ts_dbg != 0)
		I("%s: start!\n", __func__);

	if (ts->hx_point_num != 0)
		himax_finger_report(hdev);
	else
		himax_finger_leave(hdev);

	hdev->last_en_noise_filter = hdev->en_noise_filter;

	if (g_ts_dbg != 0)
		I("%s: end!\n", __func__);
}
/* end report_points*/

int himax_report_data(struct himax_device *hdev, int ts_path, int ts_status)
{
	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	if (ts_path == HX_REPORT_COORD || ts_path == HX_REPORT_COORD_RAWDATA) {
		/* Touch Point information */
		himax_report_points(hdev);
	} else {
		E("%s:Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
	}

	if (g_ts_dbg != 0)
		I("%s: END, ts_status=%d!\n", __func__, ts_status);
	return ts_status;
}
/* end report_data */

static int himax_ts_operation(struct himax_device *hdev, int ts_path,
			      int ts_status)
{
	struct himax_ts_data *ts = hdev->private_ts;

	memset(ts->xfer_buff, 0x00, 128 * sizeof(uint8_t));

	ts_status = himax_touch_get(hdev, ts->xfer_buff, ts_path, ts_status);
	if (ts_status == HX_TS_GET_DATA_FAIL)
		goto END_FUNCTION;

	ts_status =
		himax_distribute_touch_data(hdev, ts->xfer_buff, ts_path, ts_status);
	ts_status = himax_err_ctrl(hdev, ts->xfer_buff, ts_path, ts_status);

	if (ts_status == HX_REPORT_DATA || ts_status == HX_TS_NORMAL_END ||
	    ts_status == HX_READY_SERVE)
		ts_status = himax_parse_report_data(hdev, ts_path, ts_status);
	else
		goto END_FUNCTION;

	ts_status = himax_report_data(hdev, ts_path, ts_status);

END_FUNCTION:
	return ts_status;
}

void himax_fail_det_work(struct himax_device *hdev)
{
	struct himax_core_fp core_fp = hdev->core_fp;
	uint8_t data[8] = { 0 };
	uint8_t tmp_data[DATA_LEN_4] = { 0 };
	/*
	 *	uint8_t addr[4] = {0xD4, 0x74, 0x00, 0x10};
	 *	unit32_t tmp_addr_32 = 0x100074D4;
	 *	Clear Simulation Register
	 *	himax_mcu_register_write(tmp_addr_32, DATA_LEN_4, data);
	 */
	core_fp.fp_dd_clk_set(hdev, true);
	core_fp.fp_dd_reg_en(hdev, true);

	himax_mcu_dd_reg_read(hdev, 0xE5, 0, 8, data, 0 * 4, IC_MASTER);
	I("%s E5_Bank0: para[1]=0x%2.2X,para[2]=0x%2.2X, para[3]=0x%2.2X\n",
	  __func__, data[1], data[2], data[3]);
	I("%s E5_Bank0: para[4]=0x%2.2X,para[5]=0x%2.2X,para[6]=0x%2.2X, para[7]=0x%2.2X\n",
	  __func__, data[4], data[5], data[6], data[7]);

	himax_mcu_dd_reg_read(hdev, 0xE5, 0, 8, data, 1 * 4, IC_MASTER);
	I("%s E5_Bank1: para[1]=0x%2.2X,para[2]=0x%2.2X, para[3]=0x%2.2X\n",
	  __func__, data[1], data[2], data[3]);
	I("%s E5_Bank1: para[4]=0x%2.2X,para[5]=0x%2.2X,para[6]=0x%2.2X, para[7]=0x%2.2X\n",
	  __func__, data[4], data[5], data[6], data[7]);

	himax_mcu_dd_reg_read(hdev, 0xE5, 0, 8, data, 3 * 4, IC_MASTER);
	I("%s E5_Bank3: para[1]=0x%2.2X,para[2]=0x%2.2X, para[3]=0x%2.2X, para[4]=0x%2.2X\n",
	  __func__, data[1], data[2], data[3], data[4]);
	I("%s E5_Bank3: para[5]=0x%2.2X,para[6]=0x%2.2X, para[7]=0x%2.2X\n",
	  __func__, data[5], data[6], data[7]);

	core_fp.fp_dd_clk_set(hdev, false);

	I("%s: now read GPIO[1] Fail information.\n", __func__);

	himax_mcu_register_read(hdev, addr_fail_det_GPIO1_msg, DATA_LEN_4, tmp_data);
	I("%s: 100074C0 value is: tmp_data[1] = 0x%2.2x, tmp_data[1] = 0x%2.2x\n",
	  __func__, tmp_data[0], tmp_data[1]);
	I("%s: 100074C0 value is: tmp_data[2] = 0x%2.2x, tmp_data[3] = 0x%2.2x\n",
	  __func__, tmp_data[2], tmp_data[3]);

	/*	It depends on customer: */
	goto AP_recovery;

AP_recovery:

	I("%s: Now FAIL_DET pulls high means IC need external recovery\n",
	  __func__);
#if (HX_RST_PIN_FUNC == 0x01)
	himax_mcu_tp_lcm_pin_reset(hdev);
#elif (HX_RST_PIN_FUNC == 0x02)
	/* Need Customer do AP recovery */
#endif
}

void himax_ts_work(struct himax_device *hdev)
{
	int ts_status = HX_TS_NORMAL_END;
	int ts_path = 0;

	ts_path = himax_ts_work_status(hdev);

	switch (ts_path) {
	case HX_REPORT_COORD:
		ts_status = himax_ts_operation(hdev, ts_path, ts_status);
		break;
	case HX_REPORT_COORD_RAWDATA:
		ts_status = himax_ts_operation(hdev, ts_path, ts_status);
		break;
	default:
		E("%s:Path Fault! value=%d\n", __func__, ts_path);
		return;
	}

	if (ts_status == HX_TS_GET_DATA_FAIL)
		goto GET_TOUCH_FAIL;
	else
		return;

GET_TOUCH_FAIL:
	I("%s: Now reset the Touch chip.\n", __func__);
#if (HX_RST_PIN_FUNC == 0x01)
	himax_mcu_hw_reset(hdev, true);
#elif (HX_RST_PIN_FUNC == 0x02)
    /* Need Customer do TP reset pin */
#else
	himax_mcu_system_reset(hdev);
#endif
}
/*end ts_work*/
enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer)
{
	struct himax_ts_data *ts;

	ts = container_of(timer, struct himax_ts_data, timer);
	queue_work(ts->himax_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

#if defined(HX_CONTAINER_SPEED_UP)
static void himax_resume_work_func(struct work_struct *work)
{
	struct himax_ts_data *ts =
		container_of(work, struct himax_ts_data, work);
	struct himax_device *hdev =
		container_of(&ts, struct himax_device, private_ts);

	himax_chip_common_resume(hdev);
}

#endif
int hx_ic_register(struct himax_device *hdev)
{
	int ret = !NO_ERR;

#if defined(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83193)
	if (_hx83193_init(hdev)) {
		ret = NO_ERR;
		goto END;
	}
#endif
END:
	if (ret != NO_ERR)
		E("%s: There is no IC!\n", __func__);

	return ret;
}

int himax_chip_common_init(struct himax_device *hdev)
{
	int ret = 0;
	int err = PROBE_FAIL;
	struct himax_ts_data *ts = hdev->private_ts;
	struct himax_i2c_platform_data *pdata;

	ts->GTS_range = 4;
	himax_print_define_function(hdev);

	ts->xfer_buff =
		devm_kzalloc(hdev->dev, 128 * sizeof(uint8_t), GFP_KERNEL);
	if (ts->xfer_buff == NULL) {
		err = -ENOMEM;
		goto err_xfer_buff_fail;
	}

	pdata = kzalloc(sizeof(struct himax_i2c_platform_data), GFP_KERNEL);
	if (pdata == NULL) { /*Allocate Platform data space*/
		err = -ENOMEM;
		goto err_dt_platform_data_fail;
	}

	hdev->ic_data = kzalloc(sizeof(struct himax_ic_data), GFP_KERNEL);
	if (hdev->ic_data == NULL) { /*Allocate IC data space*/
		err = -ENOMEM;
		goto err_dt_ic_data_fail;
	}

	/* allocate report data */
	hdev->hx_touch_data = kzalloc(sizeof(struct himax_report_data), GFP_KERNEL);
	if (hdev->hx_touch_data == NULL) {
		err = -ENOMEM;
		goto err_alloc_touch_data_failed;
	}

	ts->pdata = pdata;
	if (himax_parse_dt(hdev, pdata) < 0) {
		I(" pdata is NULL for DT\n");
		goto err_alloc_dt_pdata_failed;
	}

	ts->lcm_gpio = pdata->RESX;

	ts->pon_gpio = pdata->PON;

#if (HX_RST_PIN_FUNC == 0x01)
	ts->rst_gpio = pdata->tp_ext_rstn;
#endif
	himax_gpio_power_config(hdev, pdata);
#if defined(HIMAX_I2C_PLATFORM)
	himax_mcu_interface_on(hdev);
#endif
	himax_mcu_polling_cascade_ic(hdev);

	hdev->hx_chip_inited = 0;

	if (hx_ic_register(hdev) != NO_ERR) {
		E("%s: can't detect IC!\n", __func__);
		goto error_ic_detect_failed;
	}
	if (!himax_mcu_calculateChecksum(hdev, FW_SIZE_128k)) {
		E("%s: check flash fail, please upgrade FW\n", __func__);

#if (HX_FIX_TOUCH_INFO == 0x01)
		himax_mcu_touch_information(hdev);
#endif

	} else {
		himax_mcu_reload_disable(hdev, 0);
		himax_mcu_power_on_init(hdev);
		himax_mcu_read_FW_ver(hdev);
		himax_mcu_touch_information(hdev);
	}

#if defined(HX_CONTAINER_SPEED_UP)
	ts->ts_int_workqueue =
		create_singlethread_workqueue("himax_ts_resume_wq");
	if (!ts->ts_int_workqueue) {
		E("%s: create ts_resume workqueue failed\n", __func__);
		goto err_create_ts_resume_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->ts_int_work, himax_resume_work_func);
#endif
	calculate_point_number(hdev);

#if defined(CONFIG_OF)
	ts->power = pdata->power;
#endif
	/*calculate the i2c data size*/
	himax_calcDataSize(hdev);

#if defined(CONFIG_OF)
	ts->pdata->abs_pressure_min = 0;
	ts->pdata->abs_pressure_max = 200;
	ts->pdata->abs_width_min = 0;
	ts->pdata->abs_width_max = 200;
#endif
	ts->suspended = false;

#if (HX_PROTOCOL_A == 0x01)
	ts->protocol_type = PROTOCOL_TYPE_A;
#else
	ts->protocol_type = PROTOCOL_TYPE_B;
#endif
	I("%s: Use Protocol Type %c\n", __func__,
	  ts->protocol_type == PROTOCOL_TYPE_A ? 'A' : 'B');

	ret = himax_input_register(ts);
	if (ret) {
		E("%s: Unable to register %s input device\n", __func__,
		  ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	spin_lock_init(&ts->irq_lock);
	ts->initialized = true;

	/*touch data init*/
	err = himax_report_data_init(hdev);

	if (err)
		goto err_report_data_init_failed;

	himax_ts_register_interrupt(hdev);
	himax_fail_det_register_interrupt(hdev);

	hdev->hx_chip_inited = true;
	return 0;

err_report_data_init_failed:
	input_unregister_device(ts->input_dev);
err_input_register_device_failed:
	input_free_device(ts->input_dev);
	/*err_detect_failed:*/

#if defined(HX_CONTAINER_SPEED_UP)
	cancel_delayed_work_sync(&ts->ts_int_work);
	destroy_workqueue(ts->ts_int_workqueue);
err_create_ts_resume_wq_failed:
#endif

error_ic_detect_failed:
	himax_gpio_power_deconfig(pdata);
#if !defined(CONFIG_OF)
err_power_failed:
#endif
err_alloc_dt_pdata_failed:
	kfree(hdev->hx_touch_data);
	hdev->hx_touch_data = NULL;
err_alloc_touch_data_failed:
	kfree(hdev->ic_data);
	hdev->ic_data = NULL;
err_dt_ic_data_fail:
	kfree(pdata);
	pdata = NULL;
err_dt_platform_data_fail:
	devm_kfree(hdev->dev, ts->xfer_buff);
	ts->xfer_buff = NULL;
err_xfer_buff_fail:
	return err;
}

void himax_chip_common_deinit(struct himax_device *hdev)
{
	struct himax_ts_data *ts = hdev->private_ts;

	himax_ts_unregister_interrupt(hdev);
	himax_report_data_deinit(hdev);

	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
#if defined(HX_CONTAINER_SPEED_UP)
	cancel_delayed_work_sync(&ts->ts_int_work);
	destroy_workqueue(ts->ts_int_workqueue);
#endif

	himax_gpio_power_deconfig(ts->pdata);
	himax_mcu_in_cmd_struct_free(hdev);

	kfree(hdev->hx_touch_data);
	hdev->hx_touch_data = NULL;
	kfree(hdev->ic_data);
	hdev->ic_data = NULL;
	devm_kfree(hdev->dev, ts->xfer_buff);
	ts->xfer_buff = NULL;
	kfree(ts->pdata);
	ts->pdata = NULL;
	kfree(ts);
	ts = NULL;

	I("%s: Common section deinited!\n", __func__);
}

int himax_chip_common_suspend(struct himax_device *hdev)
{
	struct himax_ts_data *ts = hdev->private_ts;

	if (ts->suspended) {
		I("%s: Already suspended. Skipped.\n", __func__);
		goto END;
	} else {
		ts->suspended = true;
		I("%s: enter\n", __func__);
	}

	himax_int_enable(hdev, 0);

	if (!ts->use_irq) {
		int32_t cancel_state;

		cancel_state = cancel_work_sync(&ts->work);
		if (cancel_state)
			himax_int_enable(hdev, 1);
	}

	/*ts->first_pressed = 0;*/
	atomic_set(&ts->suspend_mode, 1);
	ts->pre_finger_mask = 0;

END:
	if (ts->in_self_test == 1)
		ts->suspend_resume_done = 1;

	I("%s: END\n", __func__);

	return 0;
}

int himax_chip_common_resume(struct himax_device *hdev)
{
	struct himax_ts_data *ts = hdev->private_ts;

	I("%s: enter\n", __func__);

	if (ts->suspended == false) {
		I("%s: It had entered resume, skip this step\n", __func__);
		goto END;
	} else {
		ts->suspended = false;
	}

	atomic_set(&ts->suspend_mode, 0);
	ts->diag_cmd = 0;

	himax_mcu_tp_reset(hdev);

	himax_report_all_leave_event(ts);

	himax_int_enable(hdev, 1);

END:
	if (ts->in_self_test == 1)
		ts->suspend_resume_done = 1;

	I("%s: END\n", __func__);
	return 0;
}
