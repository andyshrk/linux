// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021-2022 Siengine Technology, Inc.
 *
 * Sienginie Camera Interface driver
 *
 * Author: Siengine Technology, Inc.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/v4l2-mediabus.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-subdev.h>

#include "ar0231_raw.h"

#define MAX96715_MAX_SENSOR_NUM	2
#define CAMERA_USES_15HZ

#define ADDR_MAX9296		0x48
#define ADDR_MAX96715		0x40
#define ADDR_MAX96715_ALL	(ADDR_MAX96715 + 5)  /* Broadcast address */

#define MIPI_CSI2_SENS_VC0_PAD_SOURCE	0
#define MIPI_CSI2_SENS_VC1_PAD_SOURCE	1
#define MIPI_CSI2_SENS_VC2_PAD_SOURCE	2
#define MIPI_CSI2_SENS_VC3_PAD_SOURCE	3
#define MIPI_CSI2_SENS_VCX_PADS_NUM		4

#define MAX_FPS		30
#define MIN_FPS		15
#define DEFAULT_FPS		30

#define ADDR_AR_SENSOR	0x48
#define ADDR_AP_SENSOR	0x5D

#define MAX96715_OV01F10_SENSOR  0
#define MAX9295A_AR0231_SENSOR   0x910354 //max9295a dev_id:0x91, ar0231 chip id:0x0354
#define MAX9295A_OV2778_GW5400_SENSOR   0x91277801

static u32 sensor_select = MAX96715_OV01F10_SENSOR;
/*!
 * Maintains the information on the current state of the sesor.
 */
struct imxdpu_videomode {
	char name[64];		/* may not be needed */

	uint32_t pixelclock;	/* Hz */

	/* htotal (pixels) = hlen + hfp + hsync + hbp */
	uint32_t hlen;
	uint32_t hfp;
	uint32_t hbp;
	uint32_t hsync;

	/* field0 - vtotal (lines) = vlen + vfp + vsync + vbp */
	uint32_t vlen;
	uint32_t vfp;
	uint32_t vbp;
	uint32_t vsync;

	/* field1  */
	uint32_t vlen1;
	uint32_t vfp1;
	uint32_t vbp1;
	uint32_t vsync1;

	uint32_t flags;

	uint32_t format;
	uint32_t dest_format; /*buffer format for capture*/

	int16_t clip_top;
	int16_t clip_left;
	uint16_t clip_width;
	uint16_t clip_height;
};

struct sensor_data {
	struct v4l2_subdev	subdev;
	struct media_pad pads[MIPI_CSI2_SENS_VCX_PADS_NUM];
	struct i2c_client *i2c_client;
	struct v4l2_mbus_framefmt format;
	struct v4l2_captureparm streamcap;
	char running;

	/* control settings */
	int brightness;
	int hue;
	int contrast;
	int saturation;
	int red;
	int green;
	int blue;
	int ae_mode;

	u32 mclk;
	u8 mclk_source;
	struct clk *sensor_clk;
	int v_channel;
	bool is_mipi;
	struct imxdpu_videomode cap_mode;

	unsigned int sensor_num;       /* sensor num connect max96715 */
	unsigned char sensor_is_there; /* Bit 0~3 for 4 cameras, 0b1= is there; 0b0 = is not there */
	int pwn_gpio;
};

struct reg_value {
	unsigned short reg_addr;
	unsigned char val;
	unsigned int delay_ms;
};

enum sensor_mode {
	SENSOR_MODE_MIN = 0,
	SENSOR_MODE_720P_1280_720 = 0,
	SENSOR_MODE_WXGA_1280_800 = 1,
	SENSOR_MODE_WVGA_752_480 = 2,
	SENSOR_MODE_VGA_640_480 = 3,
	SENSOR_MODE_CIF_352_288 = 4,
	SENSOR_MODE_QVGA_320_240 = 5,
	SENSOR_MODE_1080P_1920_1080 = 6,
	SENSOR_MODE_4K_3840_2160 = 7,
	SENSOR_MODE_MAX = SENSOR_MODE_4K_3840_2160,
};

enum sensor_frame_rate {
	SENSOR_15_FPS = 0,
	SENSOR_30_FPS,
};

static int sensor_framerates[] = {
	[SENSOR_15_FPS] = 15,
	[SENSOR_30_FPS] = 30,
};

static struct reg_value max9296_out_sync_config[] = {
	/* Frame Sync for out sync*/
	{ 0x03E2, 0x00, 0 },
	{ 0x03EA, 0x00, 0 },
	{ 0x03EB, 0x00, 0 },
	{ 0x03E8, 0x00, 0 },
	{ 0x03E9, 0x00, 0 },
	{ 0x03E7, 0x07, 0 },
	{ 0x03E6, 0xA1, 0 },
	{ 0x03E5, 0x20, 0 },
	{ 0x03EF, 0x43, 0 }, //frame sync to GMSL1 type
	{ 0x03E0, 0x04, 0 }, //GMSL1 manual frame sync
	{ 0x0B08, 0x11, 0 },
	{ 0x0C08, 0x11, 0 },
};

static struct reg_value max9296_init_data[] = {
	/* disable mipi output */
	{ 0x0313, 0x00, 0 },

	/* Disable all links */
	{ 0x0F00, 0x00, 0 },

	/* set HIM at high and hs/vs input match 96705 */
	{ 0x0B06, 0x86, 0 },
	{ 0x0C06, 0x86, 0 },
	{ 0x0006, 0x1F, 10 },   //change to GMSL1 mode

	/* enable i2c auto ack */
//	{ 0x0B0D, 0x81, 0 },
//	{ 0x0C0D, 0x81, 0 },

	/* DBL=1, HVEN=1 */
	{ 0x0B07, 0x84, 0 },
	{ 0x0C07, 0x84, 0 },

	/* setting mux mode */
	{ 0x0322, 0x30, 0 },

	/* Mapping Control */
	{ 0x040B, 0x07, 0 },
	{ 0x042D, 0x15, 0 },
	{ 0x040D, 0x1E, 0 },
	{ 0x040E, 0x1E, 0 },
	{ 0x040F, 0x00, 0 },
	{ 0x0410, 0x00, 0 },
	{ 0x0411, 0x01, 0 },
	{ 0x0412, 0x01, 0 },

	/* set des in 2x4 mode */
	{ 0x0330, 0x24, 0 },
	/* enable pipeline X&Y */
	{ 0x0002, 0x30, 0},

	/* 4 lane output from MIPI Port A */
	{ 0x044A, 0xD0, 0 },

	/* Set mipi A speed to be 1GMbps/lane */
	{ 0x0320, 0xEA, 0 },

	/* Set DT, VC, BPP  FOR PIPE Y */
	{ 0x0314, 0x10, 0 },
	{ 0x0319, 0x08, 0 },
	{ 0x0316, 0x5E, 0 },
	{ 0x0317, 0x0E, 0 },
	{ 0x031D, 0xEA, 0 },

	/* ignore first frame output */
	{ 0x0325, 0x80, 0 },

	/* Enable processing HS and DE signals */
	{ 0x0B0F, 0x01, 0 },
	{ 0x0C0F, 0x01, 0 },

	/* Enable GMSL1 to GMSL2 color mapping */
	{ 0x0B96, 0x1B, 0 },
	{ 0x0C96, 0x1B, 0 },

	/* Cross VS */
	{ 0x01D9, 0x59, 0 },
	{ 0x01F9, 0x59, 0 },
};

static struct reg_value max9296_9295a_raw12_init_data[] = {
	{ 0x0313, 0x00, 5 }, //disable mipi output
	{ 0x0001, 0x02, 10 }, //Change TX_RATE to 6Gbps

	{ 0x0010, 0x21, 0 }, //linkA setup

	{ 0x0330, 0x04, 0 }, //set portA to 1*4 mode
	{ 0x044A, 0xd0, 0 }, //Set 4 lanes for deserializer

	{ 0x0325, 0x80, 0 }, //ignore first frame output

	{ 0x0051, 0x02, 0 }, //change stream ID
	{ 0x0052, 0x01, 0 },

	/* Set mipi A speed to be 2GMbps/lane */
	{ 0x0320, 0x34, 0 },

	/* Send YUV422, FS, and FE from Pipe X to Controller 1 */
	{ 0x040B, 0x07, 0 }, //Enable 3 Mappings
	{ 0x042D, 0x15, 0 }, //Destionation Controller = Controller 1. Controller 1 sends data to MIPI Port A
	/* For the following MSB 2 bits = VC, LSB 6 bits =DT */
	{ 0x040D, 0x2C, 0 }, //SRC  DT = RAW12
	{ 0x040E, 0x6C, 0 }, //0x2C #DEST DT = RAW12
	{ 0x040F, 0x00, 0 }, //SRC  DT = Frame Start
	{ 0x0410, 0x40, 0 }, //0x00 #DEST DT = Frame Start
	{ 0x0411, 0x01, 0 }, //SRC  DT = Frame End
	{ 0x0412, 0x41, 0 }, //0x01 #DEST DT = Frame End

	/* Send YUV422, FS, and FE from Pipe Y to Controller 1 */
	{ 0x048B, 0x07, 0 }, //Enable 3 Mappings
	{ 0x04AD, 0x15, 0 }, //Destionation Controller = Controller 1. Controller 1 sends data to MIPI Port A
	/* For the following MSB 2 bits = VC, LSB 6 bits =DT */
	{ 0x048D, 0x2C, 0 }, //SRC  DT = RAW12
	{ 0x048E, 0x2C, 0 }, //0x6C #DEST DT = RAW12
	{ 0x048F, 0x00, 0 }, //SRC  DT = Frame Start
	{ 0x0490, 0x00, 0 }, //0x50 #DEST DT = Frame Start
	{ 0x0491, 0x01, 0 }, //SRC  DT = Frame End
	{ 0x0492, 0x01, 100 }, //0x51 #DEST DT = Frame End
};

static struct reg_value max9296_9295a_yuv422_init_data[] = {
	{ 0x0313, 0x00, 5 }, //disable mipi output
	{ 0x0001, 0x01, 10 }, //Change TX_RATE to 3Gbps

	{ 0x0010, 0x21, 0 }, //linkA setup

	{ 0x0330, 0x04, 0 }, //set portA to 1*4 mode
	{ 0x044A, 0xd0, 0 }, //Set 4 lanes for deserializer

	{ 0x0325, 0x80, 0 }, //ignore first frame output

	{ 0x0051, 0x02, 0 }, //change stream ID
	{ 0x0052, 0x01, 0 },

	///* Set mipi A speed to be 2GMbps/lane */
	//{ 0x0320, 0x34, 0 },

	/* Send YUV422, FS, and FE from Pipe X to Controller 1 */
	{ 0x040B, 0x07, 0 }, //Enable 3 Mappings
	{ 0x042D, 0x15, 0 }, //Destionation Controller = Controller 1. Controller 1 sends data to MIPI Port A
	/* For the following MSB 2 bits = VC, LSB 6 bits =DT */
	{ 0x040D, 0x1E, 0 }, //SRC  DT = YUV422
	{ 0x040E, 0x5E, 0 }, //0x2C #DEST DT = YUV422
	{ 0x040F, 0x00, 0 }, //SRC  DT = Frame Start
	{ 0x0410, 0x40, 0 }, //0x00 #DEST DT = Frame Start
	{ 0x0411, 0x01, 0 }, //SRC  DT = Frame End
	{ 0x0412, 0x41, 0 }, //0x01 #DEST DT = Frame End

	/* Send YUV422, FS, and FE from Pipe Y to Controller 1 */
	{ 0x048B, 0x07, 0 }, //Enable 3 Mappings
	{ 0x04AD, 0x15, 0 }, //Destionation Controller = Controller 1. Controller 1 sends data to MIPI Port A
	/* For the following MSB 2 bits = VC, LSB 6 bits =DT */
	{ 0x048D, 0x1E, 0 }, //SRC  DT = YUV422
	{ 0x048E, 0x1E, 0 }, //0x6C #DEST DT = YUV422
	{ 0x048F, 0x00, 0 }, //SRC  DT = Frame Start
	{ 0x0490, 0x00, 0 }, //0x50 #DEST DT = Frame Start
	{ 0x0491, 0x01, 0 }, //SRC  DT = Frame End
	{ 0x0492, 0x01, 100 }, //0x51 #DEST DT = Frame End
};

struct sensor_mode_info {
	enum sensor_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

static struct reg_value  sensor_setting_30fps_WXGA_1280_800[] = {
	{ 0x3024, 0x01, 0 },
	{ 0x3003, 0x20, 0 },
	{ 0x3004, 0x21, 0 },
	{ 0x3005, 0x20, 0 },
	{ 0x3006, 0x91, 0 },
	/* 1280x800 */
	{ 0x3808, 0x05, 0 },
	{ 0x3809, 0x00, 0 },
	{ 0x380a, 0x03, 0 },
	{ 0x380b, 0x20, 0 },
};

static struct reg_value  sensor_setting_30fps_720P_1280_720[] = {
	{ 0x3024, 0x01, 0 },
	{ 0x3003, 0x20, 0 },
	{ 0x3004, 0x21, 0 },
	{ 0x3005, 0x20, 0 },
	{ 0x3006, 0x91, 0 },
	/* 1280x720 */
	{ 0x3808, 0x05, 0 },
	{ 0x3809, 0x00, 0 },
	{ 0x380a, 0x02, 0 },
	{ 0x380b, 0xD0, 0 },
};

static struct reg_value  sensor_setting_30fps_WVGA_752_480[] = {
	{ 0x3024, 0x01, 0 },
	{ 0x3003, 0x20, 0 },
	{ 0x3004, 0x21, 0 },
	{ 0x3005, 0x20, 0 },
	{ 0x3006, 0x91, 0 },
	/* 752x480 */
	{ 0x3808, 0x02, 0 },
	{ 0x3809, 0xF0, 0 },
	{ 0x380a, 0x01, 0 },
	{ 0x380b, 0xE0, 0 },
};

static struct reg_value  sensor_setting_30fps_VGA_640_480[] = {
	{ 0x3024, 0x01, 0 },
	{ 0x3003, 0x20, 0 },
	{ 0x3004, 0x21, 0 },
	{ 0x3005, 0x20, 0 },
	{ 0x3006, 0x91, 0 },
	/* 640x480 */
	{ 0x3808, 0x02, 0 },
	{ 0x3809, 0x80, 0 },
	{ 0x380a, 0x01, 0 },
	{ 0x380b, 0xE0, 0 },
};

static struct reg_value  sensor_setting_30fps_CIF_352_288[] = {
	{ 0x3024, 0x01, 0 },
	{ 0x3003, 0x20, 0 },
	{ 0x3004, 0x21, 0 },
	{ 0x3005, 0x20, 0 },
	{ 0x3006, 0x91, 0 },
	/* 352x288 */
	{ 0x3808, 0x01, 0 },
	{ 0x3809, 0x60, 0 },
	{ 0x380a, 0x01, 0 },
	{ 0x380b, 0x20, 0 },
};

static struct reg_value  sensor_setting_30fps_QVGA_320_240[] = {
	{ 0x3024, 0x01, 0 },
	{ 0x3003, 0x20, 0 },
	{ 0x3004, 0x21, 0 },
	{ 0x3005, 0x20, 0 },
	{ 0x3006, 0x91, 0 },
	/* 320x240 */
	{ 0x3808, 0x01, 0 },
	{ 0x3809, 0x40, 0 },
	{ 0x380a, 0x00, 0 },
	{ 0x380b, 0xF0, 0 },
};

static struct reg_value  sensor_setting_30fps_1080p_1920_1080[] = {
	{ 0x3024, 0x01, 0 },
	{ 0x3003, 0x20, 0 },
	{ 0x3004, 0x21, 0 },
	{ 0x3005, 0x20, 0 },
	{ 0x3006, 0x91, 0 },
	/* 1920x1080 */
	{ 0x3808, 0x07, 0 },
	{ 0x3809, 0x80, 0 },
	{ 0x380a, 0x04, 0 },
	{ 0x380b, 0x38, 0 },
};

static struct reg_value  sensor_setting_30fps_4k_3840_2160[] = {
	{ 0x3024, 0x01, 0 },
	{ 0x3003, 0x20, 0 },
	{ 0x3004, 0x21, 0 },
	{ 0x3005, 0x20, 0 },
	{ 0x3006, 0x91, 0 },
	/* 3840x2160 */
	{ 0x3808, 0x07, 0 },
	{ 0x3809, 0x80, 0 },
	{ 0x380a, 0x04, 0 },
	{ 0x380b, 0x38, 0 },
};

static struct sensor_mode_info sensor_mode_info_data[2][SENSOR_MODE_MAX + 1] = {
	/* 15fps not support */
	{
		{ SENSOR_MODE_WXGA_1280_800, 0, 0, NULL, 0 },
		{ SENSOR_MODE_720P_1280_720, 0, 0, NULL, 0 },
		{ SENSOR_MODE_WVGA_752_480, 0, 0, NULL, 0 },
		{ SENSOR_MODE_VGA_640_480, 0, 0, NULL, 0 },
		{ SENSOR_MODE_CIF_352_288, 0, 0, NULL, 0},
		{ SENSOR_MODE_QVGA_320_240, 0, 0, NULL, 0},
		{ SENSOR_MODE_1080P_1920_1080, 0, 0, NULL, 0},
		{ SENSOR_MODE_4K_3840_2160, 0, 0, NULL, 0},
	},
	/* 30fps */
	{
		{ SENSOR_MODE_720P_1280_720, 1280, 720,
		  sensor_setting_30fps_720P_1280_720,
		  ARRAY_SIZE(sensor_setting_30fps_720P_1280_720)
		},
		{ SENSOR_MODE_WXGA_1280_800, 1280, 800,
		  sensor_setting_30fps_WXGA_1280_800,
		  ARRAY_SIZE(sensor_setting_30fps_WXGA_1280_800)
		},
		{ SENSOR_MODE_WVGA_752_480, 752, 480,
		  sensor_setting_30fps_WVGA_752_480,
		  ARRAY_SIZE(sensor_setting_30fps_WVGA_752_480)
		},
		{ SENSOR_MODE_VGA_640_480, 640, 480,
		  sensor_setting_30fps_VGA_640_480,
		  ARRAY_SIZE(sensor_setting_30fps_VGA_640_480)
		},
		{ SENSOR_MODE_CIF_352_288, 352, 288,
		  sensor_setting_30fps_CIF_352_288,
		  ARRAY_SIZE(sensor_setting_30fps_CIF_352_288)
		},
		{ SENSOR_MODE_QVGA_320_240, 320, 240,
		  sensor_setting_30fps_QVGA_320_240,
		  ARRAY_SIZE(sensor_setting_30fps_QVGA_320_240)
		},
		{ SENSOR_MODE_1080P_1920_1080, 1920, 1080,
		  sensor_setting_30fps_1080p_1920_1080,
		  ARRAY_SIZE(sensor_setting_30fps_1080p_1920_1080)
		},
		{ SENSOR_MODE_4K_3840_2160, 3840, 2160,
		  sensor_setting_30fps_4k_3840_2160,
		  ARRAY_SIZE(sensor_setting_30fps_4k_3840_2160)
		},
	}
};

static void sensor_delay(uint32_t msec)
{
	if (msec <= 20)
		usleep_range(msec * 1000, msec * 1000);
	else
		msleep(msec);
}

static inline struct sensor_data *subdev_to_sensor_data(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sensor_data, subdev);
}

static enum sensor_frame_rate to_sensor_frame_rate(struct v4l2_fract *timeperframe)
{
	enum sensor_frame_rate rate;
	u32 tgt_fps;	/* target frames per secound */

	tgt_fps = timeperframe->denominator / timeperframe->numerator;

	if (tgt_fps == 30)
		rate = SENSOR_30_FPS;
	else if (tgt_fps == 15)
		rate = SENSOR_15_FPS;
	else
		rate = -EINVAL;

	return rate;
}

static inline int sensor_read_reg(struct sensor_data *max9296_data, int index,
											unsigned short reg, unsigned char *val)
{
	unsigned char u8_buf[2] = { 0 };
	unsigned int buf_len = 2;
	int retry, timeout = 10;
	unsigned char u8_val = 0;

	u8_buf[0] = (reg >> 8) & 0xFF;
	u8_buf[1] = reg & 0xFF;

	max9296_data->i2c_client->addr = ADDR_AR_SENSOR + index;

	for (retry = 0; retry < timeout; retry++) {
		if (i2c_master_send(max9296_data->i2c_client, u8_buf, buf_len) < 0) {
			dev_dbg(&max9296_data->i2c_client->dev,
				"%s:read reg error on send: reg=0x%x, retry = %d.\n", __func__, reg, retry);
			sensor_delay(5);
			continue;
		}
		if (i2c_master_recv(max9296_data->i2c_client, &u8_val, 1) != 1) {
			dev_dbg(&max9296_data->i2c_client->dev,
				"%s:read reg error on recv: reg=0x%x, retry = %d.\n", __func__, reg, retry);
			sensor_delay(5);
			continue;
		}
		break;
	}

	if (retry >= timeout) {
		dev_info(&max9296_data->i2c_client->dev,
			"%s:read reg error: reg=0x%x.\n", __func__, reg);
		return -1;
	}

	*val = u8_val;

	return u8_val;
}

static inline int sensor_write_reg(struct sensor_data *max9296_data, int index,
												unsigned short reg, unsigned char val)
{
	unsigned char u8_buf[3] = { 0 };
	unsigned int buf_len = 3;
	int retry, timeout = 10;

	u8_buf[0] = (reg >> 8) & 0xFF;
	u8_buf[1] = reg & 0xFF;
	u8_buf[2] = val;

	max9296_data->i2c_client->addr = ADDR_AR_SENSOR + index;
	for (retry = 0; retry < timeout; retry++) {
		if (i2c_master_send(max9296_data->i2c_client, u8_buf, buf_len) < 0) {
			dev_dbg(&max9296_data->i2c_client->dev,
				"%s:write reg error: reg=0x%x, val=0x%x, retry = %d.\n", __func__, reg, val, retry);
			sensor_delay(5);
			continue;
		}
		break;
	}

	if (retry >= timeout) {
		dev_info(&max9296_data->i2c_client->dev,
			"%s:write reg error: reg=0x%x, val=0x%x.\n", __func__, reg, val);
		return -1;
	}

	return 0;
}

static int sensor_download_firmware(struct sensor_data *max9296_data,
			int index, struct reg_value *pModeSetting, s32 ArySize)
{
	dev_dbg(&max9296_data->i2c_client->dev, "Skip %s, SENSOR supports one kind of format at present. \n", __func__);
	return 0;
}

static inline int max96715_read_reg(struct sensor_data *max9296_data, int index, u8 reg)
{
	int val;
	int retry, timeout = 10;

	max9296_data->i2c_client->addr = ADDR_MAX96715 + index;
	for (retry = 0; retry < timeout; retry++) {
		val = i2c_smbus_read_byte_data(max9296_data->i2c_client, reg);
		if (val < 0)
			sensor_delay(5);
		else
			break;
	}

	if (retry >= timeout) {
		dev_info(&max9296_data->i2c_client->dev,
			"%s:read reg error: reg=%2x\n", __func__, reg);
		return -1;
	}

	return val;
}

static int max96715_write_reg(struct sensor_data *max9296_data, int index, u8 reg, u8 val)
{
	s32 ret;
	int retry, timeout = 10;

	max9296_data->i2c_client->addr = ADDR_MAX96715 + index;
	for (retry = 0; retry < timeout; retry++) {
		ret = i2c_smbus_write_byte_data(max9296_data->i2c_client, reg, val);
		if (val < 0)
			sensor_delay(5);
		else
			break;
	}
	dev_dbg(&max9296_data->i2c_client->dev,
		"%s: addr %02x reg %02x val %02x\n",
		__func__, max9296_data->i2c_client->addr, reg, val);

	if (retry >= timeout) {
		dev_info(&max9296_data->i2c_client->dev,
			"%s:max96715 write reg error:reg=%2x,val=%2x\n", __func__,
			reg, val);
		return -1;
	}

	return 0;
}


static inline int max9296_read_reg(struct sensor_data *max9296_data,
				unsigned short reg, unsigned char *val)
{
	unsigned char u8_buf[2] = { 0 };
	unsigned int buf_len = 2;
	int retry, timeout = 10;
	unsigned char u8_val = 0;

	u8_buf[0] = (reg >> 8) & 0xFF;
	u8_buf[1] = reg & 0xFF;

	max9296_data->i2c_client->addr = ADDR_MAX9296;

	for (retry = 0; retry < timeout; retry++) {
		if (i2c_master_send(max9296_data->i2c_client, u8_buf, buf_len) < 0) {
			dev_dbg(&max9296_data->i2c_client->dev,
				"%s:read reg error on send: reg=0x%x, retry = %d.\n", __func__, reg, retry);
			sensor_delay(5);
			continue;
		}
		if (i2c_master_recv(max9296_data->i2c_client, &u8_val, 1) != 1) {
			dev_dbg(&max9296_data->i2c_client->dev,
				"%s:read reg error on recv: reg=0x%x, retry = %d.\n", __func__, reg, retry);
			sensor_delay(5);
			continue;
		}
		break;
	}

	if (retry >= timeout) {
		dev_info(&max9296_data->i2c_client->dev,
			"%s:read reg error: reg=0x%x.\n", __func__, reg);
		return -1;
	}

	*val = u8_val;

	return u8_val;
}

static inline int max9296_write_reg(struct sensor_data *max9296_data,
				unsigned short reg, unsigned char val)
{
	unsigned char u8_buf[3] = { 0 };
	unsigned int buf_len = 3;
	int retry, timeout = 10;

	u8_buf[0] = (reg >> 8) & 0xFF;
	u8_buf[1] = reg & 0xFF;
	u8_buf[2] = val;

	max9296_data->i2c_client->addr = ADDR_MAX9296;
	for (retry = 0; retry < timeout; retry++) {
		if (i2c_master_send(max9296_data->i2c_client, u8_buf, buf_len) < 0) {
			dev_dbg(&max9296_data->i2c_client->dev,
				"%s:write reg error: reg=0x%x, val=0x%x, retry = %d.\n", __func__, reg, val, retry);
			sensor_delay(5);
			continue;
		}
		break;
	}

	if (retry >= timeout) {
		dev_info(&max9296_data->i2c_client->dev,
			"%s:max9296 write reg error: reg=0x%x, val=0x%x.\n", __func__, reg, val);
		return -1;
	}

	return 0;
}

static int max9296_initialize(struct sensor_data *max9296_data, struct reg_value *config_data,
									unsigned int array_size)
{
	unsigned int i;
	int retval;

	dev_info(&max9296_data->i2c_client->dev, "%s with frame sync\n", __func__);
	for (i = 0; i < array_size; i++) {
		retval = max9296_write_reg(max9296_data,
					config_data[i].reg_addr, config_data[i].val);
		if (retval < 0)
			break;
		if (config_data[i].delay_ms != 0)
			sensor_delay(config_data[i].delay_ms);
	}
	return 0;
}

static int max9296_hardware_init(struct sensor_data *max9296_data)
{
	int retval;
	unsigned int len = 0;
	u8 reg, reg_valA = 0, reg_valB = 0;

	if (sensor_select == MAX9295A_AR0231_SENSOR) {
		len = ARRAY_SIZE(max9296_9295a_raw12_init_data);
		retval = max9296_initialize(max9296_data, max9296_9295a_raw12_init_data, len);
		if (retval < 0) {
			return -EFAULT;
		}
	} else if (sensor_select == MAX9295A_OV2778_GW5400_SENSOR) {
		len = ARRAY_SIZE(max9296_9295a_yuv422_init_data);
		retval = max9296_initialize(max9296_data, max9296_9295a_yuv422_init_data, len);
		if (retval < 0) {
			return -EFAULT;
		}
	} else {
		len = ARRAY_SIZE(max9296_init_data);
		retval = max9296_initialize(max9296_data, max9296_init_data, len);
		if (retval < 0) {
			return -EFAULT;
		}
 	}

	if (sensor_select == MAX9295A_AR0231_SENSOR ||
		sensor_select == MAX9295A_OV2778_GW5400_SENSOR) {
		int sensor_on_linkA = 0;
		int sensor_on_linkB = 0;

		retval = max9295_read_dev_id(max9296_data->i2c_client);
		if (!retval) {
			sensor_on_linkA = 1;
			reg_valA = sensor_on_linkA;

			max9295_write_reg(max9296_data->i2c_client, 0x02BE, 0x90); //Enable sensor power down pin
		}

		retval = max9295_initialize(max9296_data->i2c_client, max9296_9295_linkA_config);
		if (retval != 0) {
			pr_info("max9296_mipi: max9295_initialize failed !\n");
		}

		/* change to linkB */
		max9296_write_reg(max9296_data, 0x0010, 0x22);
		sensor_delay(60);
		retval = max9295_read_dev_id(max9296_data->i2c_client);
		if (!retval) {
			sensor_on_linkB = 1;
			reg_valB = sensor_on_linkB;

			max9295_write_reg(max9296_data->i2c_client, 0x02BE, 0x90); //Enable sensor power down pin
			if (sensor_select == MAX9295A_AR0231_SENSOR) {
				sensor_delay(50);
				max9295_write_reg(max9296_data->i2c_client, 0x02BF, 0x60); //Enable sensor reset pin
			}
		}

		if (sensor_on_linkA & sensor_on_linkB) {
			/* change to splitter mode */
			max9296_write_reg(max9296_data, 0x0010, 0x23);
		} else if (sensor_on_linkA | sensor_on_linkB) {
			max9296_write_reg(max9296_data, 0x0010, sensor_on_linkA | (sensor_on_linkB << 1));
 		}

		if (sensor_on_linkA | sensor_on_linkB) {
			sensor_delay(50);
			if (sensor_select == MAX9295A_AR0231_SENSOR) {
				struct reg_value16 *config_data = ar0231_mode_1920X1080_12bit_linear_30fps;
				retval = ar0231_initialize(max9296_data->i2c_client, config_data);
				if (retval != 0) {
					pr_info("max9296_mipi: ar0231_initialize failed !\n");
				}
				sensor_delay(50);
 #if defined(CONFIG_ISP)
				system_i2c_init(max9296_data->i2c_client);
 #endif
			} else {
				sensor_delay(100);
			}
		}
	} else {
		//write out sync config data
		len = ARRAY_SIZE(max9296_out_sync_config);
		retval = max9296_initialize(max9296_data, max9296_out_sync_config, len);
		if (retval < 0) {
			return -EFAULT;
		}
		reg_valA = max9296_read_reg(max9296_data, 0x1dc, &reg);
		reg_valB = max9296_read_reg(max9296_data, 0x1fc, &reg);
	}
 	pr_info("max9296_mipi: reg_valA = %d, reg_valB = %d. \n", reg_valA, reg_valB);

 	max9296_data->sensor_num = 0;
	if (sensor_select == MAX9295A_AR0231_SENSOR ||
		sensor_select == MAX9295A_OV2778_GW5400_SENSOR) {
		max9296_data->sensor_is_there = (reg_valB & 0x01) | ((reg_valA & 0x01) << 1);
	} else {
		max9296_data->sensor_is_there = (reg_valA & 0x01) | ((reg_valB & 0x01) << 1);
	}

 	if (max9296_data->sensor_is_there & (0x1 << 0))
 		max9296_data->sensor_num += 1;
 	if (max9296_data->sensor_is_there & (0x1 << 1))
 		max9296_data->sensor_num += 1;
 	pr_info("max9296_mipi: sensor number = %d. \n", max9296_data->sensor_num);

	if (sensor_select == MAX9295A_AR0231_SENSOR ||
		sensor_select == MAX9295A_OV2778_GW5400_SENSOR) {
		return 0;
	}

	if (max9296_data->sensor_num > 0) {
		/* Enable all links*/
		max9296_write_reg(max9296_data, 0x0F00, max9296_data->sensor_is_there);
		sensor_delay(5);
		retval = max96715_write_reg(max9296_data, 0, 0x3f, 0x08);
		if (retval < 0) {
			pr_err("%s: can't access sensor address.\n", __func__);
		}
	}

	max9296_write_reg(max9296_data, 0x03EF, max9296_data->sensor_is_there|0x40);
	sensor_delay(10);

	return 0;
}

static int sensor_change_mode(struct sensor_data *max9296_data)
{
	struct reg_value *pModeSetting = NULL;
	enum sensor_mode mode = max9296_data->streamcap.capturemode;
	enum sensor_frame_rate rate =
				to_sensor_frame_rate(&max9296_data->streamcap.timeperframe);
	int ArySize = 0, retval = 0;

	if (mode > SENSOR_MODE_MAX || mode < SENSOR_MODE_MIN) {
		pr_err("Wrong sensor mode detected!\n");
		return -1;
	}

	pModeSetting = sensor_mode_info_data[rate][mode].init_data_ptr;
	ArySize = sensor_mode_info_data[rate][mode].init_data_size;

	max9296_data->format.width = sensor_mode_info_data[rate][mode].width;
	max9296_data->format.height = sensor_mode_info_data[rate][mode].height;

	if (max9296_data->format.width == 0 ||
		max9296_data->format.height == 0 ||
	    pModeSetting == NULL || ArySize == 0) {
		pr_err("Not support mode=%d %s\n", mode,
						(rate == 0) ? "15(fps)" : "30(fps)");
		return -EINVAL;
	}

	retval = sensor_download_firmware(max9296_data, 0, pModeSetting, ArySize);

	return retval;
}

static int max9296_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	struct sensor_data *max9296_data = subdev_to_sensor_data(sd);

	code->code = max9296_data->format.code;
	return 0;
}

/*!
 * max9296_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int max9296_enum_framesizes(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > SENSOR_MODE_MIN) {
		pr_warn("sensor only support one mode present.\n");
		return -EINVAL;
	}

	if (sensor_select == MAX9295A_AR0231_SENSOR ||
		sensor_select == MAX9295A_OV2778_GW5400_SENSOR) {
		fse->max_width = sensor_mode_info_data[fse->index + 1][fse->index + SENSOR_MODE_1080P_1920_1080].width;
		fse->min_width = fse->max_width;

		fse->max_height = sensor_mode_info_data[fse->index + 1][fse->index + SENSOR_MODE_1080P_1920_1080].height;
		fse->min_height = fse->max_height;
	} else {
		fse->max_width = sensor_mode_info_data[fse->index + 1][fse->index + 1].width;
		fse->min_width = fse->max_width;

		fse->max_height = sensor_mode_info_data[fse->index + 1][fse->index + 1].height;
		fse->min_height = fse->max_height;
	}

	return 0;
}
static int max9296_enum_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_interval_enum *fie)
{
	int i, j, count;

	if (fie->index < 0 || fie->index > SENSOR_MODE_MAX)
		return -EINVAL;

	if (fie->width == 0 || fie->height == 0 ||
	    fie->code == 0) {
		pr_warn("Please assign pixel format, width and height.\n");
		return -EINVAL;
	}

	fie->interval.numerator = 1;

	 /* TODO Reserved to extension */
	count = 0;
	for (i = 0; i < ARRAY_SIZE(sensor_framerates); i++) {
		for (j = 0; j < (SENSOR_MODE_MAX + 1); j++) {
			if (fie->width == sensor_mode_info_data[i][j].width
			 && fie->height == sensor_mode_info_data[i][j].height
			 && sensor_mode_info_data[i][j].init_data_ptr != NULL) {
				count++;
			}
			if (fie->index == (count - 1)) {
				fie->interval.denominator = sensor_framerates[i];
				return 0;
			}
		}
	}

	return -EINVAL;
}

static int max9296_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct sensor_data *max9296_data = subdev_to_sensor_data(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;

	if (fmt->pad)
		return -EINVAL;

	mf->code = max9296_data->format.code;
	mf->width =  max9296_data->format.width;
	mf->height = max9296_data->format.height;
	mf->colorspace = max9296_data->format.colorspace;
	mf->field = max9296_data->format.field;
	mf->reserved[0] = max9296_data->format.reserved[0];

	return 0;
}

static struct sensor_mode_info *get_max_resolution(enum sensor_frame_rate rate)
{
	u32 max_width;
	enum sensor_mode mode;
	int i;

	mode = 0;
	max_width  = sensor_mode_info_data[rate][0].width;

	for (i = 0; i < (SENSOR_MODE_MAX + 1); i++) {
		if (sensor_mode_info_data[rate][i].width > max_width) {
			max_width = sensor_mode_info_data[rate][i].width;
			mode = i;
		}
	}
	return &sensor_mode_info_data[rate][mode];
}

static struct sensor_mode_info *match(struct v4l2_mbus_framefmt *fmt,
			enum sensor_frame_rate rate)
{
	struct sensor_mode_info *info;
	int i;

	for (i = 0; i < (SENSOR_MODE_MAX + 1); i++) {
		if (fmt->width == sensor_mode_info_data[rate][i].width &&
			fmt->height == sensor_mode_info_data[rate][i].height) {
			info = &sensor_mode_info_data[rate][i];
			break;
		}
	}
	if (i == SENSOR_MODE_MAX + 1)
		info = NULL;

	return info;
}

static void try_to_find_resolution(struct sensor_data *sensor,
			struct v4l2_mbus_framefmt *mf)
{
	enum sensor_mode mode = sensor->streamcap.capturemode;
	struct v4l2_fract *timeperframe = &sensor->streamcap.timeperframe;
	enum sensor_frame_rate frame_rate = to_sensor_frame_rate(timeperframe);
	struct device *dev = &sensor->i2c_client->dev;
	struct sensor_mode_info *info;
	bool found = false;

	/*printk("%s:%d mode=%d, frame_rate=%d, w/h=(%d,%d)\n", __func__, __LINE__,*/
				/*mode, frame_rate, mf->width, mf->height);*/

	if ((mf->width == sensor_mode_info_data[frame_rate][mode].width) &&
		(mf->height == sensor_mode_info_data[frame_rate][mode].height)) {
			info = &sensor_mode_info_data[frame_rate][mode];
			found = true;
	} else {
		/* get mode info according to frame user's width and height */
		info = match(mf, frame_rate);
		if (info == NULL) {
			frame_rate ^= 0x1;
			info = match(mf, frame_rate);
			if (info) {
				sensor->streamcap.capturemode = -1;
				dev_err(dev, "%s %dx%d only support %s(fps)\n", __func__,
						info->width, info->height,
						(frame_rate == 0) ? "15fps" : "30fps");
				return;
			}
			goto max_resolution;
		}
		found = true;
	}

	/* get max resolution to resize */
max_resolution:
	if (!found) {
		frame_rate ^= 0x1;
		info = get_max_resolution(frame_rate);
	}

	/*printk("%s:%d mode=%d, frame_rate=%d, w/h=(%d,%d)\n", __func__, __LINE__,*/
				/*mode, frame_rate, mf->width, mf->height);*/

	sensor->streamcap.capturemode = info->mode;
	sensor->streamcap.timeperframe.denominator = (frame_rate) ? 30 : 15;
	sensor->format.width  = info->width;
	sensor->format.height = info->height;
}

static int max9296_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *fmt)
{
	struct sensor_data *max9296_data = subdev_to_sensor_data(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	int ret;

	if (fmt->pad)
		return -EINVAL;

	mf->code = max9296_data->format.code;
	mf->colorspace = max9296_data->format.colorspace;
	mf->field = V4L2_FIELD_NONE;

	try_to_find_resolution(max9296_data, mf);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	ret = sensor_change_mode(max9296_data);

	return ret;
}

static int max9296_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_frame_desc *fd)
{
	return 0;
}

static int max9296_set_frame_desc(struct v4l2_subdev *sd,
					unsigned int pad,
					struct v4l2_mbus_frame_desc *fd)
{
	return 0;
}

static int max9296_set_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

static int max9296_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sensor_data *max9296_data = subdev_to_sensor_data(sd);

	dev_info(sd->dev, "stream on with %s\n", __func__);
	if (enable) {
		if (!max9296_data->running) {
			/* Enable MIPI output, set virtual channel according to the link number */
			max9296_write_reg(max9296_data, 0x0313, 0x42);
			sensor_delay(10);
		}
		max9296_data->running++;
	} else {
		max9296_data->running--;
		if (max9296_data->running == 0) {
			/* Disable MIPI Output */
			max9296_write_reg(max9296_data, 0x0313, 0x00);
		}
	}

	return 0;
}

static int max9296_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct v4l2_subdev_pad_ops max9296_pad_ops = {
	.enum_mbus_code		= max9296_enum_mbus_code,
	.enum_frame_size	= max9296_enum_framesizes,
	.enum_frame_interval	= max9296_enum_frame_interval,
	.get_fmt		= max9296_get_fmt,
	.set_fmt		= max9296_set_fmt,
	.get_frame_desc		= max9296_get_frame_desc,
	.set_frame_desc		= max9296_set_frame_desc,
};

static const struct v4l2_subdev_core_ops max9296_core_ops = {
	.s_power	= max9296_set_power,
};

static const struct v4l2_subdev_video_ops max9296_video_ops = {
	.s_stream		= max9296_s_stream,
};

static const struct v4l2_subdev_ops max9296_subdev_ops = {
	.core	= &max9296_core_ops,
	.pad	= &max9296_pad_ops,
	.video	= &max9296_video_ops,
};

static const struct media_entity_operations max9296_sd_media_ops = {
	.link_setup = max9296_link_setup,
};

static ssize_t analog_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sensor_data *max9296_data = subdev_to_sensor_data(sd);
	u8 val = 0;

	sensor_read_reg(max9296_data, 0, 0x370A, &val);
	return sprintf(buf, "%s\n", (val & 0x4) ? "enabled" : "disabled");
}

static ssize_t analog_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sensor_data *max9296_data = subdev_to_sensor_data(sd);
	char enabled[32];

	if (sscanf(buf, "%s", enabled) > 0) {
		if (strcmp(enabled, "enable") == 0)
			sensor_write_reg(max9296_data, 0, 0x370A, 0x4);
		else
			sensor_write_reg(max9296_data, 0, 0x370A, 0x0);
		return count;
	}
	return -EINVAL;
}

static DEVICE_ATTR(analog_test_pattern, 0644, analog_show, analog_store);

static const struct of_device_id max9296_of_match[] = {
	{ .compatible = "maxim,max9296_mipi" },
	{ .compatible = "maxim,max9296_mipi-ar0231" },
	{ .compatible = "maxim,max9296_mipi-ov2778-gw5400" },
	{ /* sentinel */ }
};

/*!
 * max9296 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int max9296_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct sensor_data *max9296_data;
	struct v4l2_subdev *sd;
	int retval;
	u8 reg;

	printk("max9296_probe.");

	max9296_data = devm_kzalloc(dev, sizeof(*max9296_data), GFP_KERNEL);
	if (!max9296_data)
		return -ENOMEM;

//TODO: if it is needed on se1000 with OX01F10
	if(of_device_is_compatible(dev->of_node, max9296_of_match[0].compatible)) {
#if defined(CONFIG_CAMERA_OV01F10)
		pr_info("max9296 device is ov 01f10 sensor");
		sensor_select = MAX96715_OV01F10_SENSOR;
#endif
	} else if(of_device_is_compatible(dev->of_node, max9296_of_match[1].compatible)) {
#if defined(CONFIG_CAMERA_AR0231)
		pr_info("max9296 device is onsemi ar0231 sensor");
		sensor_select = MAX9295A_AR0231_SENSOR;
#endif
	} else if (of_device_is_compatible(dev->of_node, max9296_of_match[2].compatible)) {
#if defined(CONFIG_CAMERA_OV2778_GW5400)
		pr_info("max9296 device is ov2778 gw5400 sensor");
		sensor_select = MAX9295A_OV2778_GW5400_SENSOR;
#endif
	} else {
#if defined(CONFIG_CAMERA_OV01F10)
		pr_info("max9296 Default use ov 01f10 sensor");
		sensor_select = MAX96715_OV01F10_SENSOR;
#endif
	}

	max9296_data->i2c_client = client;
	if (sensor_select == MAX9295A_AR0231_SENSOR) {
		max9296_data->format.code = MEDIA_BUS_FMT_SRGGB12_1X12;
		max9296_data->format.width = sensor_mode_info_data[1][SENSOR_MODE_1080P_1920_1080].width;
		max9296_data->format.height = sensor_mode_info_data[1][SENSOR_MODE_1080P_1920_1080].height;
		max9296_data->cap_mode.clip_height = max9296_data->format.height;
		max9296_data->cap_mode.clip_width = max9296_data->format.width;
	} else if (sensor_select == MAX9295A_OV2778_GW5400_SENSOR) {
		max9296_data->format.code = MEDIA_BUS_FMT_UYVY8_1X16;
		max9296_data->format.width = sensor_mode_info_data[1][SENSOR_MODE_1080P_1920_1080].width;
		max9296_data->format.height = sensor_mode_info_data[1][SENSOR_MODE_1080P_1920_1080].height;
		max9296_data->cap_mode.clip_height = max9296_data->format.height;
		max9296_data->cap_mode.clip_width = max9296_data->format.width;
	} else {
		max9296_data->format.code = MEDIA_BUS_FMT_YUYV8_1X16;
		max9296_data->format.width = sensor_mode_info_data[1][0].width;
		max9296_data->format.height = sensor_mode_info_data[1][0].height;
		max9296_data->cap_mode.clip_height = max9296_data->format.height;
		max9296_data->cap_mode.clip_width = max9296_data->format.width;
	}

	max9296_data->format.colorspace = V4L2_COLORSPACE_JPEG;

	retval = max9296_read_reg(max9296_data, 0x0d, &reg);
	if (retval != 0x94) {
		pr_warn("max9296 is not found, device id: 0x%x\n", retval);
		return -ENODEV;
	}
	/*****************************************
	 * Pass mipi phy clock rate Mbps
	 * fcsi2 = PCLk * WIDTH * CHANNELS / LANES
	 * fsci2 = 72MPCLK * 8 bit * 4 channels / 4 lanes
	 ****************************************/
	max9296_data->format.reserved[0] = 72 * 8;
	max9296_data->format.field = V4L2_FIELD_NONE;
	max9296_data->streamcap.capturemode = 0;
	max9296_data->streamcap.timeperframe.denominator = 30;
	max9296_data->streamcap.timeperframe.numerator = 1;
	max9296_data->is_mipi = 1;

	max9296_data->streamcap.capability = V4L2_CAP_TIMEPERFRAME;
	max9296_data->streamcap.timeperframe.denominator = 30;
	max9296_data->streamcap.timeperframe.numerator = 1;
	max9296_data->v_channel = 0;
	max9296_data->cap_mode.clip_top = 0;
	max9296_data->cap_mode.clip_left = 0;

	max9296_data->cap_mode.hlen = max9296_data->cap_mode.clip_width;

	max9296_data->cap_mode.hfp = 0;
	max9296_data->cap_mode.hbp = 0;
	max9296_data->cap_mode.hsync = 625;
	max9296_data->cap_mode.vlen = 800;
	max9296_data->cap_mode.vfp = 0;
	max9296_data->cap_mode.vbp = 0;
	max9296_data->cap_mode.vsync = 40;
	max9296_data->cap_mode.vlen1 = 0;
	max9296_data->cap_mode.vfp1 = 0;
	max9296_data->cap_mode.vbp1 = 0;
	max9296_data->cap_mode.vsync1 = 0;
	max9296_data->cap_mode.pixelclock = 27000000;

	sd = &max9296_data->subdev;
	v4l2_i2c_subdev_init(sd, client, &max9296_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	max9296_data->pads[MIPI_CSI2_SENS_VC0_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	max9296_data->pads[MIPI_CSI2_SENS_VC1_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	max9296_data->pads[MIPI_CSI2_SENS_VC2_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	max9296_data->pads[MIPI_CSI2_SENS_VC3_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	retval = media_entity_pads_init(&sd->entity, MIPI_CSI2_SENS_VCX_PADS_NUM,
							max9296_data->pads);
	if (retval < 0)
		return retval;

	max9296_data->subdev.entity.ops = &max9296_sd_media_ops;
#if defined(CONFIG_MEDIA_CONTROLLER)
	max9296_data->subdev.entity.flags = MEDIA_ENT_F_CAM_SENSOR;
#endif
	retval = v4l2_async_register_subdev(&max9296_data->subdev);
	if (retval < 0) {
		dev_err(&client->dev,
					"%s--Async register failed, ret=%d\n", __func__, retval);
		media_entity_cleanup(&sd->entity);
	}

	retval = max9296_hardware_init(max9296_data);
	if (retval < 0) {
		dev_err(&client->dev, "camera init failed\n");
		clk_disable_unprepare(max9296_data->sensor_clk);
		media_entity_cleanup(&sd->entity);
		v4l2_async_unregister_subdev(sd);
		return retval;
	}

	max9296_data->running = 0;

	/*Create device attr in sys */
	retval = device_create_file(&client->dev, &dev_attr_analog_test_pattern);
	if (retval < 0) {
		dev_err(&client->dev, "%s: create device file fail\n", __func__);
		return retval;
	}

	dev_set_drvdata(dev, max9296_data);

	dev_info(&max9296_data->i2c_client->dev,
			"max9296_mipi is found, name %s\n", sd->name);
	return retval;
}

/*!
 * max9296 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int max9296_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sensor_data *max9296_data = subdev_to_sensor_data(sd);

	clk_disable_unprepare(max9296_data->sensor_clk);
	device_remove_file(&client->dev, &dev_attr_analog_test_pattern);
	media_entity_cleanup(&sd->entity);
	v4l2_async_unregister_subdev(sd);
	dev_set_drvdata(&client->dev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int mx9296_runtime_suspend(struct device *dev)
{
	return 0;//pm_runtime_force_suspend(dev);
}

static int mx9296_runtime_resume(struct device *dev)
{
	struct sensor_data *max9296_data = dev_get_drvdata(dev);
	int retval;
	u8 reg;

	retval = max9296_read_reg(max9296_data, 0x0d, &reg);
	if (retval != 0x94) {
		pr_warn("max9692 is not found, device id: 0x%x\n", retval);
		return -ENODEV;
	}

	retval = max9296_hardware_init(max9296_data);
	if (retval < 0) {
		pr_warn("camera init failed\n");
		return retval;
	}

	if (max9296_data->running) {
		max9296_write_reg(max9296_data, 0x0313, 0x42);
		pr_info("stream on\n");
		sensor_delay(10);
	}

	pr_info("mx9296_runtime_resume ok\n");

	//pm_runtime_force_suspend(dev);
	return 0;
}

#else
static int mx9296_runtime_suspend(struct device *dev)
{
	return 0;
}

static int mx9296_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops mx9296_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mx9296_runtime_suspend,
						mx9296_runtime_resume)
};

static const struct i2c_device_id max9296_id[] = {
	{},
};

MODULE_DEVICE_TABLE(i2c, max9296_id);

static struct i2c_driver max9296_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name   = "max9296_mipi",
		.pm = &mx9296_pm_ops,
		.of_match_table	= of_match_ptr(max9296_of_match),
	},
	.probe  = max9296_probe,
	.remove = max9296_remove,
	.id_table = max9296_id,
};

module_i2c_driver(max9296_i2c_driver);

MODULE_AUTHOR("Siengine Technology, Inc.");
MODULE_DESCRIPTION("MAX9296 GSML Deserializer Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");

