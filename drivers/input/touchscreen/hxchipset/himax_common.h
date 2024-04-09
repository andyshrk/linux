/* SPDX-License-Identifier: GPL-2.0 */
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

#ifndef HIMAX_COMMON_H
#define HIMAX_COMMON_H

#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/async.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/firmware.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/buffer_head.h>
#include <linux/pm_wakeup.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/kallsyms.h>
#include <linux/of_gpio.h>
#include <linux/version.h>
#include "himax_platform.h"
#include "himax_ic_core.h"

#define HIMAX_DRIVER_VER "Sample_code_A09.16_#4642_01"

#define FLASH_DUMP_FILE "/sdcard/HX_Flash_Dump.bin"

#define HIMAX_INSTANCE (5)

/*===========Himax Option function=============*/
#define HX_BOOT_UPGRADE					(0x00)
#define HX_EXCP_RECOVERY				(0x00)
#define HX_PROTOCOL_A					(0x00)
#define HX_PROTOCOL_B_3PA				(0x01)
#define HX_RST_PIN_FUNC					(0x00)
#define HX_TP_INSPECT_MODE				(0x00)
#define HX_FIX_TOUCH_INFO				(0x00)
#define HX_WPBP_ENABLE					(0x00)
#define HX_SMART_WAKEUP					(0x00)
#define HX_GESTURE_TRACK				(0x00)
/*=============================================*/
/* Enable it if driver go into suspend/resume twice */
/*#define HX_CONFIG_FB				(0x00)*/
/* Enable it if driver go into suspend/resume twice */
/*#define HX_CONFIG_DRM				(0x00)*/

/* #if (KERNEL_VERSION(4, 14, 0) <= LINUX_VERSION_CODE)*/
/*#define KERNEL_VER_ABOVE_4_14*/
/*#endif*/

/* WP GPIO setting, decided by which pin direct to OS side, WP need pin */
/* high either GPIO0 or GPIO4 */
/* #define WP_GPIO0 */
#define WP_GPIO4

#if defined(HX_CONTAINER_SPEED_UP)
/*Resume queue delay work time after LCM RST (unit:ms)
 */
#define DELAY_TIME 40
#endif

#define HX_MAX_WRITE_SZ    (64 * 1024 + 4)

#define HX_83192D_SERIES_PWON		"HX83192D"
#define HX_83192A_SERIES_PWON		"HX83192A"
#define HX_83193A_SERIES_PWON		"HX83193A"
#define HX_83180A_SERIES_PWON		"HX83180A"

#define HX_TP_BIN_CHECKSUM_SW		1
#define HX_TP_BIN_CHECKSUM_HW		2
#define HX_TP_BIN_CHECKSUM_CRC		3

#define SHIFTBITS			5

#define FW_SIZE_128k		131072
#define FW_SIZE_255k		261120
#define FW_SIZE_FULL		176128

#define NO_ERR					0
#define HX_FAIL					-1
#define I2C_FAIL				-1
#define MEM_ALLOC_FAIL			-2
#define INPUT_REGISTER_FAIL		-3
#define FW_NOT_READY			4
#define LENGTH_FAIL				-5
#define OPEN_FILE_FAIL			-6
#define PROBE_FAIL				-7
#define ERR_WORK_OUT			-8
#define ERR_TEST_FAIL			-9
#define HW_CRC_FAIL				1

#define HX_FINGER_ON			1
#define HX_FINGER_LEAVE			2

enum HX_IC_DEVICE {
	IC_MASTER = 0,
	IC_SLAVE_1,
	IC_SLAVE_2,
};

enum HX_TS_PATH {
	HX_REPORT_COORD = 1,
	HX_REPORT_SMWP_EVENT,
	HX_REPORT_COORD_RAWDATA,
};

enum HX_TS_STATUS {
	HX_TS_GET_DATA_FAIL = -4,
	HX_EXCP_EVENT,
	HX_CHKSUM_FAIL,
	HX_PATH_FAIL,
	HX_TS_NORMAL_END = 0,
	HX_EXCP_REC_OK,
	HX_READY_SERVE,
	HX_REPORT_DATA,
	HX_EXCP_WARNING,
	HX_IC_RUNNING,
	HX_ZERO_EVENT_COUNT,
	HX_RST_OK,
};

#if (HX_FIX_TOUCH_INFO == 0x01)
enum fix_touch_info {
	FIX_HX_RX_NUM = 64,
	FIX_HX_TX_NUM = 40,
	FIX_HX_MAX_PT = 10,
	FIX_HX_INT_IS_EDGE = false,
	FIX_HX_IS_ID_EN = 1,
	FIX_HX_ID_PALM_EN = 0,
};
#endif

struct himax_ic_data {
	int vendor_arch_ver;
	int vendor_config_ver;
	int vendor_touch_cfg_ver;
	int vendor_display_cfg_ver;
	int vendor_cid_maj_ver;
	int vendor_cid_min_ver;
	int vendor_panel_ver;
	uint8_t vendor_remark1[12];
	uint8_t vendor_remark2[12];
	uint8_t vendor_ticket[12];
	uint8_t vendor_config_date[12];
	uint8_t vendor_cus_info[12];
	uint8_t vendor_proj_info[12];
	uint8_t vendor_ic_id[13];
	int HX_RX_NUM;
	int HX_TX_NUM;
	int HX_X_RES;
	int HX_Y_RES;
	int HX_MAX_PT;
	bool HX_INT_IS_EDGE;
	bool HX_IS_ID_EN;
	bool HX_ID_PALM_EN;
	bool DA_PROTOCOL_EN;
	bool STOP_FW_BY_HOST_EN;
};

struct himax_target_report_data {
	int *x;
	int *y;
	int *w;
	int *finger_id;
	int finger_on;
	int finger_num;
	int *fpt_cnt;
};

struct himax_report_data {
	int touch_all_size;
	int raw_cnt_max;
	int raw_cnt_rmd;
	int touch_info_size;
	uint8_t	finger_on;
	uint8_t *hx_coord_buf;
	uint8_t hx_state_info[2];
	int rawdata_size;
	uint8_t diag_cmd;
	uint8_t *hx_rawdata_buf;
	uint8_t rawdata_frame_size;
};

struct himax_ts_data {
	bool initialized;
	bool suspended;
	atomic_t suspend_mode;
	uint8_t x_channel;
	uint8_t y_channel;
	uint8_t useScreenRes;
	uint8_t diag_cmd;
	char chip_name[30];
	uint8_t protocol_type;
	uint8_t first_pressed;
	uint8_t coord_data_size;
	uint8_t area_data_size;
	uint8_t coordInfoSize;
	uint8_t raw_data_frame_size;
	uint8_t nFinger_support;
	uint8_t irq_enabled;
	uint8_t diag_self[50];
	uint16_t finger_pressed;
	uint16_t last_slot;
	uint16_t pre_finger_mask;
	uint16_t old_finger;
	int hx_point_num;

	uint32_t debug_log_level;
	uint32_t widthFactor;
	uint32_t heightFactor;

	int lcm_gpio;
	int rst_gpio;
	int pon_gpio;
	int use_irq;
	int (*power)(int on);
	int pre_finger_data[10][2];

	struct workqueue_struct *himax_wq;
	struct work_struct work;
	struct input_dev *input_dev;

	struct hrtimer timer;
	struct himax_i2c_platform_data *pdata;
	struct mutex rw_lock;
	atomic_t irq_state;
	spinlock_t irq_lock;

/******* SPI-start *******/
	struct spi_device	*spi;
	int hx_irq;
	uint8_t *xfer_buff;
/******* SPI-end *******/

	int in_self_test;
	int suspend_resume_done;
	int bus_speed;

	struct workqueue_struct *flash_wq;
	struct work_struct flash_work;

#if defined(HX_CONTAINER_SPEED_UP)
	struct workqueue_struct *ts_int_workqueue;
	struct delayed_work ts_int_work;
#endif

	struct workqueue_struct *himax_diag_wq;
	struct delayed_work himax_diag_delay_wrok;
	uint8_t GTS_range;
	char self_test_file_ch[30];
	uint8_t slave_ic_num;
};

enum input_protocol_type {
	PROTOCOL_TYPE_A	= 0x00,
	PROTOCOL_TYPE_B	= 0x01,
};

struct himax_device {
	int hx_chip_inited;

	struct device *dev;
	struct i2c_client *client;

	struct himax_ts_data *private_ts;
	struct himax_ic_data *ic_data;
	struct himax_report_data *hx_touch_data;

	struct himax_core_fp core_fp;

	struct himax_target_report_data *target_report_data;
	struct himax_target_report_data *fixed_point_label;

	int hx_touch_info_point_cnt;

	uint8_t en_noise_filter;
	uint8_t last_en_noise_filter;

	uint8_t *gp_rw_buf;
	uint8_t *internal_buffer;

	int hx_fail_det_irq;
};

int himax_chip_common_suspend(struct himax_device *hdev);
int himax_chip_common_resume(struct himax_device *hdev);

int himax_parse_dt(struct himax_device *hdev, struct himax_i2c_platform_data *pdata);
int himax_report_data(struct himax_device *hdev, int ts_path, int ts_status);

int himax_report_data_init(struct himax_device *hdev);

int himax_dev_set(struct himax_ts_data *ts);
int himax_input_register_device(struct input_dev *input_dev);
void himax_report_all_leave_event(struct himax_ts_data *ts);
void himax_mcu_clear_event_stack(struct himax_device *hdev);

void himax_mcu_in_cmd_struct_free(struct himax_device *hdev);
void himax_gpio_set(int pinnum, uint8_t value);

/* CORE_INIT */
int himax_mcu_in_cmd_struct_init(struct himax_device *hdev);
void himax_parse_assign_cmd(uint32_t addr, uint8_t *cmd, int len);
int himax_mcu_register_write(struct himax_device *hdev, uint32_t write_addr, uint32_t write_length, uint8_t *write_data);
int himax_mcu_register_read(struct himax_device *hdev, uint32_t read_addr, uint32_t read_length, uint8_t *read_data);
int himax_mcu_register_read_slave(struct himax_device *hdev, uint8_t device, uint32_t read_addr, uint32_t read_length, uint8_t *read_data);
void himax_mcu_tp_lcm_pin_reset(struct himax_device *hdev);
void himax_mcu_burst_enable(struct himax_device *hdev, uint8_t auto_add_4_byte);
void himax_mcu_interface_on(struct himax_device *hdev);
void himax_mcu_polling_cascade_ic(struct himax_device *hdev);
void himax_mcu_power_on_init(struct himax_device *hdev);
bool himax_mcu_dd_reg_write(struct himax_device *hdev, uint8_t addr, uint8_t pa_num, int len, uint8_t *data, uint8_t bank);
bool himax_mcu_dd_reg_read(struct himax_device *hdev, uint8_t addr, uint8_t pa_num, int len, uint8_t *data, uint8_t bank, uint8_t ic_device);
void himax_mcu_system_reset(struct himax_device *hdev);
void himax_mcu_command_reset(struct himax_device *hdev);
uint32_t himax_mcu_check_CRC(struct himax_device *hdev, uint8_t *start_addr, int reload_length);
void himax_mcu_reload_disable(struct himax_device *hdev, int disable);
void himax_mcu_read_FW_ver(struct himax_device *hdev);
void himax_print_define_function(struct himax_device *hdev);
bool himax_mcu_read_event_stack(struct himax_device *hdev, uint8_t *buf, uint8_t length);
bool himax_mcu_calculateChecksum(struct himax_device *hdev, uint32_t size);
int himax_mcu_assign_sorting_mode(struct himax_device *hdev, uint8_t *tmp_data);
bool himax_mcu_flash_lastdata_check_with_bin(uint32_t size);
int himax_mcu_fw_ver_bin(void);
void himax_mcu_hw_reset(uint8_t int_off);
void himax_mcu_tp_reset(struct himax_device *hdev);
void himax_mcu_touch_information(struct himax_device *hdev);
int himax_mcu_get_touch_data_size(void);
int himax_mcu_cal_data_len(int raw_cnt_rmd, int HX_MAX_PT, int raw_cnt_max);
void himax_mcu_read_FW_status(struct himax_device *hdev);
int himax_mcu_check_sorting_mode(struct himax_device *hdev, uint8_t *tmp_data);
struct timespec64 time_diff(struct timespec64 start, struct timespec64 end);
void himax_mcu_excp_ic_reset(void);
int himax_mcu_ic_excp_recovery(uint32_t hx_excp_event,
		uint32_t hx_zero_event, uint32_t length);
/* CORE_INIT */

int himax_bus_read(struct himax_device *hdev, uint8_t command, uint8_t *data, uint32_t length,
			  uint8_t toRetry);
int himax_bus_write(struct himax_device *hdev, uint8_t command, uint8_t *data, uint32_t length,
			  uint8_t toRetry);
int himax_bus_read_slave(struct himax_device *hdev, uint8_t device, uint8_t command, uint8_t *data, uint32_t length,
			  uint8_t toRetry);
int himax_bus_write_slave(struct himax_device *hdev, uint8_t device, uint8_t command, uint8_t *data, uint32_t length,
			   uint8_t toRetry);
void himax_int_enable(struct himax_device *hdev, int enable);
int himax_ts_register_interrupt(struct himax_device *hdev);
int himax_fail_det_register_interrupt(struct himax_device *hdev);
int himax_ts_unregister_interrupt(struct himax_device *hdev);
uint8_t himax_int_gpio_read(int pinnum);
int himax_gpio_power_config(struct himax_device *hdev,
			   struct himax_i2c_platform_data *pdata);
void himax_gpio_power_deconfig(struct himax_i2c_platform_data *pdata);

void himax_ts_work(struct himax_device *hdev);
void himax_fail_det_work(struct himax_device *hdev);
enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer);
int himax_chip_common_init(struct himax_device *hdev);
void himax_chip_common_deinit(struct himax_device *hdev);

#if defined(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83193)
bool _hx83193_init(struct himax_device *hdev);
#endif

#endif
