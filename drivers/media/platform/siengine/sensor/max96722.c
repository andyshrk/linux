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
#include <uapi/linux/media.h>
#include "ar0231_raw.h"
#include "ov2311.h"

#define MAX96715_MAX_SENSOR_NUM	4
#define CAMERA_USES_15HZ

#define MAX96715_OV01F10_SENSOR  0
#define MAX96717F_OV0X03C10_SENSOR   1
#define MAX96705_OV01F10_SENSOR	2
#define MAX9295A_AR0231_SENSOR   0x910354 //max9295a dev_id:0x91, ar0231 chip id:0x0354
#define MAX96717F_OV9284_SENSOR   0xC8928101
#define MAX9295A_LI_OV2311_IR_SENSOR	0x912311

#define ADDR_MAX96722		0x6b
#define ADDR_MAX96715		0x40
#define ADDR_MAX96705		0x40

#define ADDR_MAX96715_ALL	(ADDR_MAX96715 + 5)  /* Broadcast address */
#define ADDR_OX01F10		0x36
#define ADDR_MAX96717		0x40
#define ADDR_MAX96717F_OV9284	0x42

#define MIPI_CSI2_SENS_VC0_PAD_SOURCE	0
#define MIPI_CSI2_SENS_VC1_PAD_SOURCE	1
#define MIPI_CSI2_SENS_VC2_PAD_SOURCE	2
#define MIPI_CSI2_SENS_VC3_PAD_SOURCE	3
#define MIPI_CSI2_SENS_VCX_PADS_NUM		4

#define MAX_FPS		30
#define MIN_FPS		15
#define DEFAULT_FPS		30

#define ADDR_AR_SENSOR	0x29
#define ADDR_AP_SENSOR	0x5D

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
	u32 sensor_select;
	int power_en;
};

#define SENSOR_REG_PID		0x300A
#define SENSOR_REG_VER		0x300B

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
	SENSOR_MODE_WXGA_1920_1536 = 7,
	SENSOR_MODE_WXGA_1600_1300 = 8,
	SENSOR_MODE_MAX = SENSOR_MODE_WXGA_1600_1300,
};

enum sensor_frame_rate {
	SENSOR_15_FPS = 0,
	SENSOR_30_FPS,
};

static int sensor_framerates[] = {
	[SENSOR_15_FPS] = 15,
	[SENSOR_30_FPS] = 30,
};

static struct reg_value max96717_init_data[] = {
//remote serializer ic
//Camera :OV X1F+MAX96717F/9295A，serializer address:0x42 (7bit)
	{ 0x0570, 0x10, 0 },
	{ 0x03F0, 0x59, 0 },
	{ 0x03F1, 0x09, 0 },//set MFP4 output 24MHz
	{ 0x02BE, 0x84, 0 },
	{ 0x02BF, 0xA0, 0 },
	{ 0x02C0, 0x88, 0 },//set frame sync ID at 8
	{ 0x0318, 0x5e, 0 },

 	//Errata only for 96717/F
	{ 0x1417, 0x00, 0 },
	{ 0x1432, 0x7F, 0 },
};

static struct reg_value max96722_40v3c10_init_data[] = {
// Begin preset registers for MAX96722 0x29(7bit)
	{ 0x0013, 0x75, 100 },
	{ 0x040B, 0x00, 5 },	//disable mipi output
	{ 0x0010, 0x11, 0 },
	{ 0x0011, 0x11, 0 },
	{ 0x0006, 0xF0, 5 }, // Disable all links

	//------------update for errata
	{ 0x06C2, 0x10, 0 }, //Increase CMU voltage to for wide temperature range
	{ 0x14D1, 0x03, 0 }, //VGAHiGain
	{ 0x15D1, 0x03, 0 }, //VGAHiGain
	{ 0x16D1, 0x03, 0 }, //VGAHiGain
	{ 0x17D1, 0x03, 0 }, //VGAHiGain

	{ 0x1445, 0x03, 0 }, //disable SSC
	{ 0x1545, 0x03, 0 }, //disable SSC
	{ 0x1645, 0x03, 0 }, //disable SSC
	{ 0x1745, 0x03, 0 }, //disable SSC

	/* PHY0,PHY1,PHY2,PHY3 data rate per lane, PHY0&PHY1&PHY2&PHY3：1.0Gbps, */
	{ 0x0415, 0x2A, 0 },
	{ 0x0418, 0x2A, 0 },
	{ 0x041B, 0x2A, 0 },
	{ 0x041E, 0x2A, 0 },

	{ 0x08A0, 0x04, 0 }, // mipi output set at 2*4
	{ 0x00F4, 0x0F, 0 }, //enable 4 pipeline
	{ 0x08A3, 0xE4, 0 }, //phy0/1 lane map
	{ 0x08A9, 0xC8, 0 }, // Replicate from source Phy C to destination Phy E
	{ 0x090B, 0x07, 0 }, //enable 3 mappings for pipeline 0
	{ 0x092D, 0x15, 0 }, // map to destination controller 1
	{ 0x090D, 0x1E, 0 }, //source data type (1E) and VC(0)
	{ 0x090E, 0x1E, 0 }, //destination dt(1E) and vc(0)
	{ 0x090F, 0x00, 0 }, //source dt(00,frame start) and vc(0)
	{ 0x0910, 0x00, 0 }, //destination dt(00,frame start) and vc(0)
	{ 0x0911, 0x01, 0 }, //source dt(01,frame end) and vc(0)
	{ 0x0912, 0x01, 0 }, //destination dt(01,frame end) and vc(0)
	{ 0x094B, 0x07, 0 }, //enable 3 mappings for pipeline 1
	{ 0x096D, 0x15, 0 }, // CSI2 controller 1
	{ 0x094D, 0x1E, 0 }, //SOURCE VC=0
	{ 0x094E, 0x5E, 0 }, //DES: VC=1
	{ 0x094F, 0x00, 0 }, //Source VC=0
	{ 0x0950, 0x40, 0 }, //DES: VC=1
	{ 0x0951, 0x01, 0 }, //Source VC=1
	{ 0x0952, 0x41, 0 },
	{ 0x098B, 0x07, 0 }, //enable 3 mappings for pipeline 2
	{ 0x09AD, 0x15, 0 }, // CSI2 controller 1
	{ 0x098D, 0x1E, 0 }, //source dt=yuv422 and vc=0
	{ 0x098E, 0x9E, 0 }, //DES DT=YUV422 AND VC=1
	{ 0x098F, 0x00, 0 }, //SOURCE DT AND VC
	{ 0x0990, 0x80, 0 }, //DES DT AND VC
	{ 0x0991, 0x01, 0 }, //SOURCE DT AND VC
	{ 0x0992, 0x81, 0 }, //DES DT AND VC
	{ 0x09CB, 0x07, 0 }, //enable 3 mappings for pipeline 3
	{ 0x09ED, 0x15, 0 }, // CSI2 controller 1
	{ 0x09CD, 0x1E, 0 }, //source dt and vc
	{ 0x09CE, 0xDE, 0 }, //des dt and vc
	{ 0x09CF, 0x00, 0 }, //source dt and vc
	{ 0x09D0, 0xC0, 0 }, //des dt and vc
	{ 0x09D1, 0x01, 0 }, //source dt and vc
	{ 0x09D2, 0xC1, 0 }, //des dt and vc
//----- --------- Frame Sync if needed to trigger the camera--------------/
// Turn off auto master link selection
	{ 0x04A2, 0x00, 0 }, // Disable overlap window
	{ 0x04AA, 0x00, 0 },
	{ 0x04AB, 0x00, 0 }, //disable error threshold
	{ 0x04A8, 0x00, 0 },
	{ 0x04A9, 0x00, 0 }, // set FSYNC period to 25M/30 CLK cycles. PCLK at 25MHz. If it's 25fps, set it at 0x0F 42 40. If it's 30fps, set at 0x0C B7 35.
	{ 0x04A7, 0x0C, 0 },
	{ 0x04A6, 0xB7, 0 },
	{ 0x04A5, 0x35, 0 }, // AUTO_FS_LINKS = 0, FS_USE_XTAL = 1, FS_LINK_[3:0] = 0? GMSL1
	{ 0x04AF, 0xC0, 0 }, // Set frame sync ID at 8
	{ 0x04B1, 0x40, 0 }, // Manual frame sync, output on MFP2
	{ 0x04A0, 0x04, 0 },

	{ 0x0018, 0x0F, 50 },
	{ 0x0006, 0xFF, 100 }, // Enable all links   0x00,0xA0,//delay 100ms
};


static struct reg_value max96705_init_data[] = {
	// Begin of Script
	// MAX96712 Link Initialization to pair with GMSL1 Serializers
	{0x0006, 0x00, 0x0}, // Disable all links

	//Tx/Rx rate Selection
	{0x0010, 0x11, 0x00},// 3Gbps

	// Power up sequence for GMSL1 HIM capable serializers; Also apply to Ser with power up status unknown
	// Turn on HIM on MAX96712
	{0x0C06, 0xEF, 0x0},
	{0x0B06, 0xEF, 0x0},
	{0x0D06, 0xEF, 0x0},
	{0x0E06, 0xEF, 0x0},
	// disable HS/VS processing
	{0x0C0F, 0x01, 0x0},
	{0x0B0F, 0x01, 0x0},
	{0x0D0F, 0x01, 0x0},
	{0x0E0F, 0x01, 0x0},

	{0x0C07, 0x84, 0xa},
	{0x0B07, 0x84, 0x0},
	{0x0D07, 0x84, 0x0},
	{0x0E07, 0x84, 0x0},

	// YUV MUX mode
	{0x041A, 0xF0, 0x0},

	// MAX96712 MIPI settings
	{0x08A0, 0x04, 0x0},
	{0x08A2, 0x30, 0x0},
	{0x00F4, 0x0f, 0x0}, //enable 4 pipeline 0xF, enable pipeline 0 0x1
	{0x094A, 0xc0, 0x0},  // Mipi data lane count b6-7 1/2/4lane
	{0x08A3, 0xE4, 0x0}, //0x44 4 lanes data output, 0x4E lane2/3 data output. 0xE4 lane0/1 data output.

	//BPP for pipe lane 0 set as 1E(YUV422)
	{0x040B, 0x40, 0x0},
	{0x040C, 0x00, 0x0},
	{0x040D, 0x00, 0x0},
	{0x040E, 0x5E, 0x0},
	{0x040F, 0x7E, 0x0},
	{0x0410, 0x7A, 0x0},
	{0x0411, 0x48, 0x0},
	{0x0412, 0x20, 0x0},

	//lane speed set
	{0x0415, 0xEA, 0x0}, //phy0 lane speed set 0xEF for 1.5g
	{0x0418, 0xEA, 0x0}, //phy1 lane speed set bit0-4 for lane speed. 10001 0xf1 for 1.5g

	//Mapping settings
	{0x090B, 0x07, 0x0},
	{0x092D, 0x15, 0x0},
	{0x090D, 0x1E, 0x0},
	{0x090E, 0x1E, 0x0},
	{0x090F, 0x00, 0x0},
	{0x0910, 0x00, 0x0}, //frame sync frame start map
	{0x0911, 0x01, 0x0},
	{0x0912, 0x01, 0x0},

	//{0x0903,0x80, 0x0},//bigger than 1.5Gbps， Deskew can works.

	{0x094B, 0x07, 0x0},
	{0x096D, 0x15, 0x0},
	{0x094D, 0x1E, 0x0},
	{0x094E, 0x5E, 0x0},
	{0x094F, 0x00, 0x0},
	{0x0950, 0x40, 0x0},
	{0x0951, 0x01, 0x0},
	{0x0952, 0x41, 0x0},

	{0x098B, 0x07, 0x0},
	{0x09AD, 0x15, 0x0},
	{0x098D, 0x1E, 0x0},
	{0x098E, 0x9E, 0x0},
	{0x098F, 0x00, 0x0},
	{0x0990, 0x80, 0x0},
	{0x0991, 0x01, 0x0},
	{0x0992, 0x81, 0x0},

	{0x09CB, 0x07, 0x0},
	{0x09ED, 0x15, 0x0},
	{0x09CD, 0x1E, 0x0},
	{0x09CE, 0xDE, 0x0},
	{0x09CF, 0x00, 0x0},
	{0x09D0, 0xC0, 0x0},
	{0x09D1, 0x01, 0x0},
	{0x09D2, 0xC1, 0x0},

	//-------------- Frame Sync --------------/
	// set FSYNC period to 25M/30 CLK cycles. PCLK at 25MHz
	{0x04A2, 0x00, 0x0},
	{0x04A7, 0x0F, 0x0},
	{0x04A6, 0x42, 0x0},
	{0x04A5, 0x40, 0x0},
	{0x04AA, 0x00, 0x0},
	{0x04AB, 0x00, 0x0},

	// AUTO_FS_LINKS = 0, FS_USE_XTAL = 1, FS_LINK_[3:0] = 0? GMSL1
	{0x04AF, 0x4F, 0x0},
	{0x04A0, 0x00, 0x0},
	{0x0B08, 0x10, 0x0},
	{0x0C08, 0x10, 0x0},
	{0x0D08, 0x10, 0x0},
	{0x0E08, 0x10, 0x0},

	{ 0x0018, 0x0F, 20 },
};


static struct reg_value max96722_init_data[] = {
	/* disable mipi output */
	{ 0x040B, 0x00, 0 },
	/* Disable all links */
	{ 0x0006, 0x00, 5 },

	/* Turn on HIM mode on Link A/B/C/D */
	{ 0x0B06, 0xEF, 0 },
	{ 0x0C06, 0xEF, 0 },
	{ 0x0D06, 0xEF, 0 },
	{ 0x0E06, 0xEF, 0 },

//	/* Enable reverse channel config; Turn on local I2C acknowledg */
//	{ 0x0B0D, 0x81, 0 },
//	{ 0x0C0D, 0x81, 0 },
//	{ 0x0D0D, 0x81, 0 },
//	{ 0x0E0D, 0x81, 5 },

	/* disable HS/VS processin */
	{ 0x0B0F, 0x01, 0 },
	{ 0x0C0F, 0x01, 0 },
	{ 0x0D0F, 0x01, 0 },
	{ 0x0E0F, 0x01, 0 },

	/* Enable control link only */
	{ 0x0B07, 0x84, 0 },
	{ 0x0C07, 0x84, 0 },
	{ 0x0D07, 0x84, 0 },
	{ 0x0E07, 0x84, 5 },

	/* YUV MUX mode */
	{ 0x041A, 0xF0, 0 },

	/* MIPI settings */
	{ 0x08A0, 0x04, 0 },
	{ 0x00F4, 0x0f, 0 },
	/*for datalane with 4-lane mapping*/
	{ 0x08A3, 0xE4, 0 },

	/* replicate Port A to Port B */
	{ 0x08A9, 0xC8, 0 },

	{ 0x040B, 0x40, 0 },
	{ 0x040C, 0x00, 0 },
	{ 0x040D, 0x00, 0 },
	{ 0x040E, 0x5E, 0 },
	{ 0x040F, 0x7E, 0 },
	{ 0x0410, 0x7A, 0 },
	{ 0x0411, 0x48, 0 },
	{ 0x0412, 0x20, 0 },

	/* PHY0,PHY1,PHY2,PHY3 data rate per lane, PHY0&PHY1&PHY2&PHY3：1.0Gbps, */
	{ 0x0415, 0xEA, 0 },
	{ 0x0418, 0xEA, 0 },
	{ 0x041B, 0x2A, 0 },
	{ 0x041E, 0x2A, 0 },

	{ 0x090B, 0x07, 0 },
	{ 0x092D, 0x15, 0 },
	{ 0x090D, 0x1E, 0 },
	{ 0x090E, 0x1E, 0 },
	{ 0x090F, 0x00, 0 },
	{ 0x0910, 0x00, 0 },
	{ 0x0911, 0x01, 0 },
	{ 0x0912, 0x01, 0 },

	{ 0x094B, 0x07, 0 },
	{ 0x096D, 0x15, 0 },
	{ 0x094D, 0x1E, 0 },
	{ 0x094E, 0x5E, 0 },
	{ 0x094F, 0x00, 0 },
	{ 0x0950, 0x40, 0 },
	{ 0x0951, 0x01, 0 },
	{ 0x0952, 0x41, 0 },

	{ 0x098B, 0x07, 0 },
	{ 0x09AD, 0x15, 0 },
	{ 0x098D, 0x1E, 0 },
	{ 0x098E, 0x9E, 0 },
	{ 0x098F, 0x00, 0 },
	{ 0x0990, 0x80, 0 },
	{ 0x0991, 0x01, 0 },
	{ 0x0992, 0x81, 0 },

	{ 0x09CB, 0x07, 0 },
	{ 0x09ED, 0x15, 0 },
	{ 0x09CD, 0x1E, 0 },
	{ 0x09CE, 0xDE, 0 },
	{ 0x09CF, 0x00, 0 },
	{ 0x09D0, 0xC0, 0 },
	{ 0x09D1, 0x01, 0 },
	{ 0x09D2, 0xC1, 0 },

	/* Frame Sync */
	{0x04A2, 0x00, 0},
	{0x04AA, 0x00, 0},
	{0x04AB, 0x00, 0},
	{0x04A8, 0xFF, 0},
	{0x04A9, 0x1F, 0},
	{0x04A7, 0x07, 0},
	{0x04A6, 0xA1, 0},
	{0x04A5, 0x20, 0},
	{0x04AF, 0x40, 0},
	{0x04A0, 0x04, 0}, /*0x04*/
	{0x0B08, 0x11, 0},
	{0x0C08, 0x11, 0},
	{0x0D08, 0x11, 0},
	{0x0E08, 0x11, 0},

	/* Cross VS */
	{0x01D9, 0x59, 0},
	{0x01F9, 0x59, 0},
	{0x0219, 0x59, 0},
	{0x0239, 0x59, 5},
	/* one-shot link reset */
	{ 0x0018, 0x0F, 20 },

	{ 0x0006, 0x0F, 5 },
};

static struct reg_value max96722_ar0231_init_data[] = {
	//Begin preset registers for MAX96712
	{ 0x0013, 0x75, 100 },

	{ 0x040B, 0x00, 0 }, //turn off mipi output
	{ 0x0006, 0xF0, 0 }, //GMSL2 mode and disable all links

	{ 0x00F4, 0x0F, 0 }, //turn on pipe 0-3

	//Set DPHY 4-lane
	{ 0x094A, 0xC0, 0 },
	{ 0x098A, 0xC0, 0 },

	{ 0x08A0, 0x04, 0 }, //Set 2x4 mode
	{ 0x08A3, 0xE4, 0 }, //Port A lane mapping for 2x4

	/* replicate Port A to Port B */
	{ 0x08A9, 0xC8, 0 },

	//Pipe to Controller Mapping Setting
	//YUV422 8b, video pipeline 0
	{ 0x090B, 0x07, 0 },
	{ 0x092D, 0x15, 0 }, //CSI2 controller 2 for Port B
	{ 0x090D, 0x2C, 0 }, //0x2C for RAW10
	{ 0x090E, 0x2C, 0 }, //0x6C for VC0
	{ 0x090F, 0x00, 0 },
	{ 0x0910, 0x00, 0 },
	{ 0x0911, 0x01, 0 },
	{ 0x0912, 0x01, 0 },

	//YUV422 8b, video pipeline 1
	{ 0x094B, 0x07, 0 },
	{ 0x096D, 0x15, 0 }, //CSI2 controller 2 for Port B
	{ 0x094D, 0x2C, 0 },
	{ 0x094E, 0x6C, 0 }, //VC1
	{ 0x094F, 0x00, 0 },
	{ 0x0950, 0x40, 0 },
	{ 0x0951, 0x01, 0 },
	{ 0x0952, 0x41, 0 },

	//YUV422 8b, video pipeline 2
	{ 0x098B, 0x07, 0 },
	{ 0x09AD, 0x15, 0 }, //CSI2 controller 2 for Port B
	{ 0x098D, 0x2C, 0 },
	{ 0x098E, 0xAC, 0 }, //VC2
	{ 0x098F, 0x00, 0 },
	{ 0x0990, 0x80, 0 },
	{ 0x0991, 0x01, 0 },
	{ 0x0992, 0x81, 0 },

	//YUV422 8b, video pipeline 3
	{ 0x09CB, 0x07, 0 },
	{ 0x09ED, 0x15, 0 }, //CSI2 controller 2 for Port B
	{ 0x09CD, 0x2C, 0 },
	{ 0x09CE, 0xEC, 0 }, //VC3
	{ 0x09CF, 0x00, 0 },
	{ 0x09D0, 0xC0, 0 },
	{ 0x09D1, 0x01, 0 },
	{ 0x09D2, 0xC1, 0 },

	{ 0x0006, 0xFF, 100}, //enable all links
};

static struct reg_value max96722_9295a_yuv422_init_data[] = {
	//Begin preset registers for MAX96712
	{ 0x0013, 0x75, 100 },

	{ 0x040B, 0x00, 0 }, //turn off mipi output
	{ 0x0006, 0xF0, 0 }, //GMSL2 mode and disable all links

	{ 0x00F4, 0x0F, 0 }, //turn on pipe 0-3

	//Set DPHY 4-lane
	{ 0x094A, 0xC0, 0 },
	{ 0x098A, 0xC0, 0 },

	{ 0x08A0, 0x04, 0 }, //Set 2x4 mode
	{ 0x08A3, 0xE4, 0 }, //Port A lane mapping for 2x4

	/* replicate Port A to Port B */
	{ 0x08A9, 0xC8, 0 },

	//Pipe to Controller Mapping Setting
	//YUV422 8b, video pipeline 0
	{ 0x090B, 0x07, 0 },
	{ 0x092D, 0x15, 0 }, //CSI2 controller 2 for Port B
	{ 0x090D, 0x1E, 0 }, //0x1E for yuv422
	{ 0x090E, 0x1E, 0 }, //0x1E for VC0
	{ 0x090F, 0x00, 0 },
	{ 0x0910, 0x00, 0 },
	{ 0x0911, 0x01, 0 },
	{ 0x0912, 0x01, 0 },

	//YUV422 8b, video pipeline 1
	{ 0x094B, 0x07, 0 },
	{ 0x096D, 0x15, 0 }, //CSI2 controller 2 for Port B
	{ 0x094D, 0x1E, 0 },
	{ 0x094E, 0x5E, 0 }, //VC1
	{ 0x094F, 0x00, 0 },
	{ 0x0950, 0x40, 0 },
	{ 0x0951, 0x01, 0 },
	{ 0x0952, 0x41, 0 },

	//YUV422 8b, video pipeline 2
	{ 0x098B, 0x07, 0 },
	{ 0x09AD, 0x15, 0 }, //CSI2 controller 2 for Port B
	{ 0x098D, 0x1E, 0 },
	{ 0x098E, 0x9E, 0 }, //VC2
	{ 0x098F, 0x00, 0 },
	{ 0x0990, 0x80, 0 },
	{ 0x0991, 0x01, 0 },
	{ 0x0992, 0x81, 0 },

	//YUV422 8b, video pipeline 3
	{ 0x09CB, 0x07, 0 },
	{ 0x09ED, 0x15, 0 }, //CSI2 controller 2 for Port B
	{ 0x09CD, 0x1E, 0 },
	{ 0x09CE, 0xDE, 0 }, //VC3
	{ 0x09CF, 0x00, 0 },
	{ 0x09D0, 0xC0, 0 },
	{ 0x09D1, 0x01, 0 },
	{ 0x09D2, 0xC1, 0 },

	{ 0x0006, 0xFF, 100}, //enable all links
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

static struct reg_value  sensor_setting_30fps_WXGA_1920_1536[] = {
	{ 0x3024, 0x01, 0 },
	{ 0x3003, 0x20, 0 },
	{ 0x3004, 0x21, 0 },
	{ 0x3005, 0x20, 0 },
	{ 0x3006, 0x91, 0 },
	/* 1920x1536 */
	{ 0x3808, 0x07, 0 },
	{ 0x3809, 0x80, 0 },
	{ 0x380a, 0x06, 0 },
	{ 0x380b, 0x00, 0 },
};

static struct reg_value  sensor_setting_30fps_WXGA_1600_1300[] = {
	{ 0x3024, 0x01, 0 },
	{ 0x3003, 0x20, 0 },
	{ 0x3004, 0x21, 0 },
	{ 0x3005, 0x20, 0 },
	{ 0x3006, 0x91, 0 },
	/* 1600x1300 */
	{ 0x3808, 0x06, 0 },
	{ 0x3809, 0x40, 0 },
	{ 0x380a, 0x05, 0 },
	{ 0x380b, 0x14, 0 },
};

static struct reg_value max96705_reg[] = {
	{0x07, 0x84, 100},
	{0x0F, 0xBF, 100},
	{0x43, 0x25, 0},
	{0x45, 0x01, 0},
	{0x47, 0x26, 100},
	{0x67, 0xc4, 200},
};

static struct sensor_mode_info sensor_mode_info_data[2][SENSOR_MODE_MAX + 1] = {
	/* 15fps not support */
	{
		{ SENSOR_MODE_WXGA_1280_800,   0, 0, NULL, 0},
		{ SENSOR_MODE_720P_1280_720,   0, 0, NULL, 0},
		{ SENSOR_MODE_WVGA_752_480,    0, 0, NULL, 0},
		{ SENSOR_MODE_VGA_640_480,     0, 0, NULL, 0},
		{ SENSOR_MODE_CIF_352_288,     0, 0, NULL, 0},
		{ SENSOR_MODE_QVGA_320_240,    0, 0, NULL, 0},
		{ SENSOR_MODE_1080P_1920_1080, 0, 0, NULL, 0},
		{ SENSOR_MODE_WXGA_1920_1536,  0, 0, NULL, 0},
		{ SENSOR_MODE_WXGA_1600_1300, 0, 0, NULL, 0},
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
		{ SENSOR_MODE_WXGA_1920_1536, 1920, 1536,
		  sensor_setting_30fps_WXGA_1920_1536,
		  ARRAY_SIZE(sensor_setting_30fps_WXGA_1920_1536)
		},
		{ SENSOR_MODE_WXGA_1600_1300, 1600, 1300,
		  sensor_setting_30fps_WXGA_1600_1300,
		  ARRAY_SIZE(sensor_setting_30fps_WXGA_1600_1300)
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

static inline int sensor_read_reg(struct sensor_data *max96722_data, int index,
											unsigned short reg, unsigned char *val)
{
	unsigned char u8_buf[2] = { 0 };
	unsigned int buf_len = 2;
	int retry, timeout = 10;
	unsigned char u8_val = 0;

	u8_buf[0] = (reg >> 8) & 0xFF;
	u8_buf[1] = reg & 0xFF;

	max96722_data->i2c_client->addr = ADDR_AR_SENSOR + index;

	for (retry = 0; retry < timeout; retry++) {
		if (i2c_master_send(max96722_data->i2c_client, u8_buf, buf_len) < 0) {
			dev_dbg(&max96722_data->i2c_client->dev,
				"%s:read reg error on send: reg=0x%x, retry = %d.\n", __func__, reg, retry);
			sensor_delay(5);
			continue;
		}
		if (i2c_master_recv(max96722_data->i2c_client, &u8_val, 1) != 1) {
			dev_dbg(&max96722_data->i2c_client->dev,
				"%s:read reg error on recv: reg=0x%x, retry = %d.\n", __func__, reg, retry);
			sensor_delay(5);
			continue;
		}
		break;
	}

	if (retry >= timeout) {
		dev_info(&max96722_data->i2c_client->dev,
			"%s:read reg error: reg=0x%x.\n", __func__, reg);
		return -1;
	}

	*val = u8_val;

	return u8_val;
}

static inline int sensor_write_reg(struct sensor_data *max96722_data, int index,
												unsigned short reg, unsigned char val)
{
	unsigned char u8_buf[3] = { 0 };
	unsigned int buf_len = 3;
	int retry, timeout = 10;

	u8_buf[0] = (reg >> 8) & 0xFF;
	u8_buf[1] = reg & 0xFF;
	u8_buf[2] = val;

	max96722_data->i2c_client->addr = ADDR_AR_SENSOR + index;
	for (retry = 0; retry < timeout; retry++) {
		if (i2c_master_send(max96722_data->i2c_client, u8_buf, buf_len) < 0) {
			dev_dbg(&max96722_data->i2c_client->dev,
				"%s:write reg error: reg=0x%x, val=0x%x, retry = %d.\n", __func__, reg, val, retry);
			sensor_delay(5);
			continue;
		}
		break;
	}

	if (retry >= timeout) {
		dev_info(&max96722_data->i2c_client->dev,
			"%s:write reg error: reg=0x%x, val=0x%x.\n", __func__, reg, val);
		return -1;
	}

	return 0;
}

static int sensor_download_firmware(struct sensor_data *max96722_data,
			int index, struct reg_value *pModeSetting, s32 ArySize)
{
	/*
	register u32 Delay_ms = 0;
	register u16 RegAddr = 0;
	register u8 Val = 0;
	int i, retval = 0;

	for (i = 0; i < ArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->delay_ms;
		RegAddr = pModeSetting->reg_addr;
		Val = pModeSetting->val;

		retval = sensor_write_reg(max96722_data, index, RegAddr, Val);
		if (retval < 0)
			goto err;

		if (Delay_ms)
			sensor_delay(Delay_ms);
	}
err:
	return retval;
	*/
	dev_dbg(&max96722_data->i2c_client->dev, "Skip %s, SENSOR supports one kind of format at present. \n", __func__);
	return 0;
}

static inline int ov_sensor_read_reg(struct sensor_data *max96722_data,
											unsigned short reg, unsigned char *val)
{
	unsigned char u8_buf[2] = { 0 };
	unsigned int buf_len = 2;
	int retry, timeout = 10;
	unsigned char u8_val = 0;

	u8_buf[0] = (reg >> 8) & 0xFF;
	u8_buf[1] = reg & 0xFF;

	max96722_data->i2c_client->addr = ADDR_OX01F10;

	for (retry = 0; retry < timeout; retry++) {
		if (i2c_master_send(max96722_data->i2c_client, u8_buf, buf_len) < 0) {
			dev_dbg(&max96722_data->i2c_client->dev,
				"%s:read reg error on send: reg=0x%x, retry = %d.\n", __func__, reg, retry);
			sensor_delay(5);
			continue;
		}
		if (i2c_master_recv(max96722_data->i2c_client, &u8_val, 1) != 1) {
			dev_dbg(&max96722_data->i2c_client->dev,
				"%s:read reg error on recv: reg=0x%x, retry = %d.\n", __func__, reg, retry);
			sensor_delay(5);
			continue;
		}
		break;
	}

	if (retry >= timeout) {
		dev_info(&max96722_data->i2c_client->dev,
			"%s:read reg error: reg=0x%x.\n", __func__, reg);
		return -1;
	}

	*val = u8_val;

	return u8_val;
}

static inline int max96715_read_reg(struct sensor_data *max96722_data, int index, u8 reg)
{
	int val;
	int retry, timeout = 10;

	max96722_data->i2c_client->addr = ADDR_MAX96715 + index;
	for (retry = 0; retry < timeout; retry++) {
		val = i2c_smbus_read_byte_data(max96722_data->i2c_client, reg);
		if (val < 0)
			sensor_delay(5);
		else
			break;
	}

	if (retry >= timeout) {
		dev_info(&max96722_data->i2c_client->dev,
			"%s:read reg error: reg=%2x\n", __func__, reg);
		return -1;
	}

	return val;
}

static int max96717_write_reg(struct sensor_data *max96722_data, u16 reg, u8 val)
{
	unsigned char u8_buf[3] = { 0 };
	unsigned int buf_len = 3;
	int retry, timeout = 10;
	u32 sensor_select = max96722_data->sensor_select;

	u8_buf[0] = (reg >> 8) & 0xFF;
	u8_buf[1] = reg & 0xFF;
	u8_buf[2] = val;

	max96722_data->i2c_client->addr = (sensor_select == MAX96717F_OV9284_SENSOR) ? ADDR_MAX96717F_OV9284: ADDR_MAX96717;
	for (retry = 0; retry < timeout; retry++) {
		if (i2c_master_send(max96722_data->i2c_client, u8_buf, buf_len) < 0) {
			dev_dbg(&max96722_data->i2c_client->dev,
				"%s:write reg error: reg=0x%x, val=0x%x, retry = %d.\n", __func__, reg, val, retry);
			sensor_delay(5);
			continue;
		}
		break;
	}

	if (retry >= timeout) {
		dev_info(&max96722_data->i2c_client->dev,
			"%s:write reg error: reg=0x%x, val=0x%x.\n", __func__, reg, val);
		return -1;
	}

	return 0;
}

static int max96717_write_init_data(struct sensor_data *max96722_data)
{
	int i, array_size;
	int retval = 0;

	array_size = ARRAY_SIZE(max96717_init_data);
	for (i = 0; i < array_size; i++) {
		retval = max96717_write_reg(max96722_data,
					max96717_init_data[i].reg_addr, max96717_init_data[i].val);
		if (retval < 0)
			break;
		if (max96717_init_data[i].delay_ms != 0)
			sensor_delay(max96717_init_data[i].delay_ms);
	}
	return retval;
}

static int max96715_write_reg(struct sensor_data *max96722_data, int index, u8 reg, u8 val)
{
	s32 ret;
	int retry, timeout = 10;

	max96722_data->i2c_client->addr = ADDR_MAX96715 + index;
	for (retry = 0; retry < timeout; retry++) {
		ret = i2c_smbus_write_byte_data(max96722_data->i2c_client, reg, val);
		if (val < 0)
			sensor_delay(5);
		else
			break;
	}
	dev_dbg(&max96722_data->i2c_client->dev,
		"%s: addr %02x reg %02x val %02x\n",
		__func__, max96722_data->i2c_client->addr, reg, val);

	if (retry >= timeout) {
		dev_info(&max96722_data->i2c_client->dev,
			"%s:write reg error:reg=%2x,val=%2x\n", __func__,
			reg, val);
		return -1;
	}

	return 0;
}

static int max96705_write_reg(struct sensor_data *max96722_data,u8 reg, u8 val)
{
	s32 ret;
	int retry, timeout = 10;

	max96722_data->i2c_client->addr = ADDR_MAX96705;
	for (retry = 0; retry < timeout; retry++) {
		ret = i2c_smbus_write_byte_data(max96722_data->i2c_client, reg, val);
		if (val < 0)
			sensor_delay(5);
		else
			break;
	}
	dev_dbg(&max96722_data->i2c_client->dev,
		"%s: addr %02x reg %02x val %02x\n",
		__func__, max96722_data->i2c_client->addr, reg, val);

	if (retry >= timeout) {
		dev_dbg(&max96722_data->i2c_client->dev,
			"%s:write reg error:reg=%2x,val=%2x\n", __func__,
			reg, val);
		return -1;
	}

	return 0;
}


static inline int max96722_read_reg(struct sensor_data *max96722_data,
											unsigned short reg, unsigned char *val)
{
	unsigned char u8_buf[2] = { 0 };
	unsigned int buf_len = 2;
	int retry, timeout = 10;
	unsigned char u8_val = 0;

	u8_buf[0] = (reg >> 8) & 0xFF;
	u8_buf[1] = reg & 0xFF;

	max96722_data->i2c_client->addr = ADDR_MAX96722;

	for (retry = 0; retry < timeout; retry++) {
		if (i2c_master_send(max96722_data->i2c_client, u8_buf, buf_len) < 0) {
			dev_dbg(&max96722_data->i2c_client->dev,
				"%s:read reg error on send: reg=0x%x, retry = %d.\n", __func__, reg, retry);
			sensor_delay(5);
			continue;
		}
		if (i2c_master_recv(max96722_data->i2c_client, &u8_val, 1) != 1) {
			dev_dbg(&max96722_data->i2c_client->dev,
				"%s:read reg error on recv: reg=0x%x, retry = %d.\n", __func__, reg, retry);
			sensor_delay(5);
			continue;
		}
		break;
	}

	if (retry >= timeout) {
		dev_info(&max96722_data->i2c_client->dev,
			"%s:read reg error: reg=0x%x.\n", __func__, reg);
		return -1;
	}

	*val = u8_val;

	return u8_val;
}

static inline int max96722_write_reg(struct sensor_data *max96722_data,
												unsigned short reg, unsigned char val)
{
	unsigned char u8_buf[3] = { 0 };
	unsigned int buf_len = 3;
	int retry, timeout = 10;

	u8_buf[0] = (reg >> 8) & 0xFF;
	u8_buf[1] = reg & 0xFF;
	u8_buf[2] = val;

	max96722_data->i2c_client->addr = ADDR_MAX96722;
	for (retry = 0; retry < timeout; retry++) {
		if (i2c_master_send(max96722_data->i2c_client, u8_buf, buf_len) < 0) {
			dev_info(&max96722_data->i2c_client->dev,
				"%s:write reg error: reg=0x%x, val=0x%x, retry = %d.\n", __func__, reg, val, retry);
			sensor_delay(5);
			continue;
		}
		break;
	}

	if (retry >= timeout) {
		dev_info(&max96722_data->i2c_client->dev,
			"%s:write reg error: reg=0x%x, val=0x%x.\n", __func__, reg, val);
		return -1;
	}

	return 0;
}

static int max96722_initialize(struct sensor_data *max96722_data)
{
	int i, array_size;
	int retval = 0;
	u32 sensor_select = max96722_data->sensor_select;

	if(sensor_select == MAX96717F_OV0X03C10_SENSOR ||
		sensor_select == MAX96717F_OV9284_SENSOR) {
		array_size = ARRAY_SIZE(max96722_40v3c10_init_data);
		for (i = 0; i < array_size; i++) {
			retval = max96722_write_reg(max96722_data,
						max96722_40v3c10_init_data[i].reg_addr, max96722_40v3c10_init_data[i].val);
			if (retval < 0)
				break;
			if (max96722_40v3c10_init_data[i].delay_ms != 0)
				sensor_delay(max96722_40v3c10_init_data[i].delay_ms);
		}
	} else if(sensor_select == MAX9295A_AR0231_SENSOR) {
		array_size = ARRAY_SIZE(max96722_ar0231_init_data);
		for (i = 0; i < array_size; i++) {
			retval = max96722_write_reg(max96722_data,
						max96722_ar0231_init_data[i].reg_addr, max96722_ar0231_init_data[i].val);
			if (retval < 0)
				break;
			if (max96722_ar0231_init_data[i].delay_ms != 0)
				sensor_delay(max96722_ar0231_init_data[i].delay_ms);
		}
	} else if(sensor_select == MAX9295A_LI_OV2311_IR_SENSOR) {
		pr_info("max96722_mipi, init 96722 for 9296-yuv422\n");
		array_size = ARRAY_SIZE(max96722_9295a_yuv422_init_data);
		for (i = 0; i < array_size; i++) {
			retval = max96722_write_reg(max96722_data,
						max96722_9295a_yuv422_init_data[i].reg_addr, max96722_9295a_yuv422_init_data[i].val);
			if (retval < 0)
				break;
			if (max96722_9295a_yuv422_init_data[i].delay_ms != 0)
				sensor_delay(max96722_9295a_yuv422_init_data[i].delay_ms);
		}
	} else if(sensor_select == MAX96705_OV01F10_SENSOR) {
		array_size = ARRAY_SIZE(max96705_init_data);
		for (i = 0; i < array_size; i++) {
			retval = max96722_write_reg(max96722_data,
						max96705_init_data[i].reg_addr, max96705_init_data[i].val);
			if (retval < 0)
				break;
			if (max96705_init_data[i].delay_ms != 0)
				sensor_delay(max96705_init_data[i].delay_ms);
		}
	}else {
		array_size = ARRAY_SIZE(max96722_init_data);
		for (i = 0; i < array_size; i++) {
			retval = max96722_write_reg(max96722_data,
						max96722_init_data[i].reg_addr, max96722_init_data[i].val);
			if (retval < 0)
				break;
			if (max96722_init_data[i].delay_ms != 0)
				sensor_delay(max96722_init_data[i].delay_ms);
		}
	}
	return retval;
}


static void max96722_max96705_init_ser(struct sensor_data *max96722_data,int id,int max_sensor_number)
{
	//Camera Module Power On/Off
	u8 value;
	int j = 0;
	int array_size;

	max96722_read_reg(max96722_data, 0xbcb+id*0x100, &value);
	dev_dbg(&max96722_data->i2c_client->dev,"====id[%d]===value 0x%x===\r\n",id,value);


	max96722_read_reg(max96722_data, 0xb04+id*0x100,&value);
	max96722_write_reg(max96722_data, 0xb04+id*0x100, value|(1<<1));//open current reverse channel
	usleep_range(100,110);
	//close other reverse channel
	for (j = 0; j < max_sensor_number; j++){
		if (j != id){
			max96722_read_reg(max96722_data, 0xb04+j*0x100,&value);
			max96722_write_reg(max96722_data, 0xb04+j*0x100, value&(0xfd));
			usleep_range(100,110);
		}
	}

	//max96722_write_reg(max96722_data, 0x0, add<<1);	//change i2c adds
	//usleep_range(10000, 11000);

	array_size = ARRAY_SIZE(max96705_reg);
	for (j = 0; j < array_size; j++) {
		max96705_write_reg(max96722_data, max96705_reg[j].reg_addr, max96705_reg[j].val);
			if (max96705_reg[j].delay_ms != 0)
				sensor_delay(max96705_reg[j].delay_ms);
	}
	usleep_range(10000, 11000);

}


static int max96722_hardware_init(struct sensor_data *max96722_data)
{
	int retval;
	int i = 0, max_lock = 50;
	u8 reg, reg_valA = 0, reg_valB = 0, reg_valC = 0, reg_valD = 0;
	u32 sensor_select = max96722_data->sensor_select;
	u8 value;

	retval = max96722_initialize(max96722_data);
	if (retval < 0){
		return -EFAULT;
	}

	if(sensor_select == MAX96705_OV01F10_SENSOR)
	{
		gpio_request(max96722_data->power_en,"power-en");
		gpio_direction_output(max96722_data->power_en, 0);
		usleep_range(10000, 11000);
		gpio_direction_output(max96722_data->power_en, 1);
		msleep(120);
		max96722_write_reg(max96722_data, 0x6, 0x0f);
		usleep_range(10000, 11000);
	}

	max96722_data->sensor_num = 0;
	if((sensor_select == MAX96717F_OV0X03C10_SENSOR) ||
		(sensor_select == MAX96717F_OV9284_SENSOR) ||
		(sensor_select == MAX9295A_AR0231_SENSOR) ||
		(sensor_select == MAX9295A_LI_OV2311_IR_SENSOR)) {
		reg_valA = max96722_read_reg(max96722_data, 0x1a, &reg);
		reg_valB = max96722_read_reg(max96722_data, 0x0a, &reg);
		reg_valC = max96722_read_reg(max96722_data, 0x0b, &reg);
		reg_valD = max96722_read_reg(max96722_data, 0x0c, &reg);
		pr_info("max96722_mipi: reg_valA = %d. \n", reg_valA);

		if (sensor_select == MAX96717F_OV0X03C10_SENSOR)
			reg_valA = (reg_valA & 0x08)||(reg_valA & 0x02);
		else
			reg_valA >>=3;
		reg_valB >>= 3;
		reg_valC >>= 3;
		reg_valD >>= 3;
		pr_info("max96722_mipi: reg_valA = %d. \n", reg_valA);
	} else {
		while((i++ < max_lock) && !(reg_valA & reg_valB & reg_valC & reg_valD)) {
			if(!reg_valA)
				reg_valA = max96722_read_reg(max96722_data, 0xbcb, &reg);
			if(!reg_valB)
				reg_valB = max96722_read_reg(max96722_data, 0xccb, &reg);
			if(!reg_valC)
				reg_valC = max96722_read_reg(max96722_data, 0xdcb, &reg);
			if(!reg_valD)
				reg_valD = max96722_read_reg(max96722_data, 0xecb, &reg);
			sensor_delay(1);
		}
	}

	reg = max96722_read_reg(max96722_data, 0x11F2, &reg);
	max96722_data->sensor_is_there = (reg_valA & 0x01) |
		((reg_valB & 0x01) << 1) |
		((reg_valC & 0x01) << 2) |
		((reg_valD & 0x01) << 3);

	if (max96722_data->sensor_is_there & (0x1 << 0))
		max96722_data->sensor_num += 1;
	if (max96722_data->sensor_is_there & (0x1 << 1))
		max96722_data->sensor_num += 1;
	if (max96722_data->sensor_is_there & (0x1 << 2))
		max96722_data->sensor_num += 1;
	if (max96722_data->sensor_is_there & (0x1 << 3))
		max96722_data->sensor_num += 1;
	pr_info("max96722_mipi: sensor number = %d. \n", max96722_data->sensor_num);
	/* Enable all links*/
	if(sensor_select == MAX96717F_OV0X03C10_SENSOR ||
		sensor_select == MAX96717F_OV9284_SENSOR) {
		max96722_write_reg(max96722_data, 0x0006, max96722_data->sensor_is_there|0xF0);
		max96717_write_reg(max96722_data, 0x0318, 0x5E);
	} else if(sensor_select == MAX9295A_AR0231_SENSOR) {
		max96722_write_reg(max96722_data, 0x0006, max96722_data->sensor_is_there|0xF0);
		if (max96722_data->sensor_num > 0) {
			max9295_write_reg(max96722_data->i2c_client, 0x02be, 0x90); //Enable sensor power down pin
			max9295_write_reg(max96722_data->i2c_client, 0x02bf, 0x60); //Enable sensor reset pin
			max9295_write_reg(max96722_data->i2c_client, 0x0318, 0x6C); //raw 12 input only
			retval = ar0231_initialize(max96722_data->i2c_client, ar0231_mode_1920X1080_12bit_linear_30fps);
			if (retval != 0) {
				pr_info("max9296_mipi: ar0231_initialize failed !\n");
			}
			sensor_delay(50);
#if defined(CONFIG_ISP)
			system_i2c_init(max96722_data->i2c_client);
#endif
		}
	} else if(sensor_select ==MAX9295A_LI_OV2311_IR_SENSOR) {
		max96722_write_reg(max96722_data, 0x0006, max96722_data->sensor_is_there|0xF0);
		if (max96722_data->sensor_num > 0) {
			retval = max9295_initialize(max96722_data->i2c_client, max9295a_ov2311_config);
			if (retval < 0){
				pr_err("%s: A can't init sensor-serlizer ic MAX9295A.\n", __func__);
				return retval;
			}
			retval = ov2311_initialize(max96722_data->i2c_client, ov2311_1600x1300_yuv_60fps);
			if (retval != 0) {
				pr_info("max96722_mipi: ov2311_initialize failed !\n");
			}
			sensor_delay(50);
		}
	} else if(sensor_select == MAX96705_OV01F10_SENSOR){
			/*do nothing*/
	} else {
		max96722_write_reg(max96722_data, 0x0006, max96722_data->sensor_is_there);
		sensor_delay(5);
		if (max96722_data->sensor_num > 0) {
			/* crossbar for HS. needed for 96715 */
			retval = max96715_write_reg(max96722_data, 0, 0x3f, 0x08);
			if (retval < 0){
				pr_err("%s: can't access sensor address.\n", __func__);
				return retval;
			}
		}
	}

	/* Enable linkA*/
	if(reg_valA & 0x1){
		if(sensor_select == MAX96717F_OV0X03C10_SENSOR ||
			sensor_select == MAX96717F_OV9284_SENSOR) {
			max96722_write_reg(max96722_data, 0x0006, 0xF1);
			sensor_delay(5);

			retval = max96717_write_init_data(max96722_data);
			if (retval < 0){
				pr_err("%s: A can't init sensor-serlizer ic MAX96717.\n", __func__);
				return retval;
			}
			pr_info("%s: A init sensor-serlizer ic MAX96717 ok.\n", __func__);
		} else if(sensor_select == MAX9295A_LI_OV2311_IR_SENSOR) {
			/*do nothing*/
		} else if(sensor_select == MAX9295A_AR0231_SENSOR){
			/* disable i2c to link B/C/D */
			max96722_write_reg(max96722_data, 0x0003, 0xFE);
			retval = max9295_initialize(max96722_data->i2c_client, max96722_9295_linkA_config);
			if (retval < 0){
				pr_err("%s: A can't init sensor-serlizer ic MAX9295A.\n", __func__);
				return retval;
			}
			pr_info("%s: A init sensor-serlizer ic MAX9295A ok.\n", __func__);
		} else if(sensor_select == MAX96705_OV01F10_SENSOR){
			max96722_max96705_init_ser(max96722_data,0,max96722_data->sensor_num);
		} else {
			// do nothing
		}
	}

	/* Enable linkB*/
	if(reg_valB & 0x1){
		if(sensor_select == MAX96717F_OV0X03C10_SENSOR ||
			sensor_select == MAX96717F_OV9284_SENSOR) {
			max96722_write_reg(max96722_data, 0x0006, 0xF2);
			sensor_delay(5);

			retval = max96717_write_init_data(max96722_data);
			if (retval < 0){
				pr_err("%s: B can't init sensor-serlizer ic MAX96717.\n", __func__);
				return retval;
			}
			pr_info("%s: B init sensor-serlizer ic MAX96717 ok.\n", __func__);
		} else if(sensor_select == MAX9295A_LI_OV2311_IR_SENSOR) {
			/*do nothing*/
		} else if(sensor_select == MAX9295A_AR0231_SENSOR){
			/* disable i2c to link A/C/D */
			max96722_write_reg(max96722_data, 0x0003, 0xFB);
			retval = max9295_initialize(max96722_data->i2c_client, max96722_9295_linkB_config);
			if (retval < 0){
				pr_err("%s: B can't init sensor-serlizer ic MAX9295A.\n", __func__);
				return retval;
			}
			pr_info("%s: B init sensor-serlizer ic MAX9295A ok.\n", __func__);
		} else if(sensor_select == MAX96705_OV01F10_SENSOR){
			max96722_max96705_init_ser(max96722_data,1,max96722_data->sensor_num);
		} else {
			// do nothing
		}
	}

	/* Enable linkC*/
	if(reg_valC & 0x1){
		if(sensor_select == MAX96717F_OV0X03C10_SENSOR ||
			sensor_select == MAX96717F_OV9284_SENSOR) {
			max96722_write_reg(max96722_data, 0x0006, 0xF4);
			sensor_delay(5);

			retval = max96717_write_init_data(max96722_data);
			if (retval < 0){
				pr_err("%s: C can't init sensor-serlizer ic MAX96717.\n", __func__);
				return retval;
			}
			pr_info("%s: C init sensor-serlizer ic MAX96717 ok.\n", __func__);
		} else if(sensor_select == MAX9295A_LI_OV2311_IR_SENSOR) {
			/*do nothing*/
		} else if(sensor_select == MAX9295A_AR0231_SENSOR){
			/* disable i2c to link A/B/D */
			max96722_write_reg(max96722_data, 0x0003, 0xEF);
			retval = max9295_initialize(max96722_data->i2c_client, max96722_9295_linkC_config);
			if (retval < 0){
				pr_err("%s: C can't init sensor-serlizer ic MAX9295A.\n", __func__);
				return retval;
			}
			pr_info("%s: C init sensor-serlizer ic MAX9295A ok.\n", __func__);
		} else if(sensor_select == MAX96705_OV01F10_SENSOR){
			max96722_max96705_init_ser(max96722_data,2,max96722_data->sensor_num);
		} else {
			// do nothing
		}
	}

	/* Enable linkD*/
	if(reg_valD & 0x1){
		if(sensor_select == MAX96717F_OV0X03C10_SENSOR ||
			sensor_select == MAX96717F_OV9284_SENSOR) {
			max96722_write_reg(max96722_data, 0x0006, 0xF8);
			sensor_delay(5);

			retval = max96717_write_init_data(max96722_data);
			if (retval < 0){
				pr_err("%s: D can't init sensor-serlizer ic MAX96717.\n", __func__);
				return retval;
			}
			pr_info("%s: D init sensor-serlizer ic MAX96717 ok.\n", __func__);
		} else if(sensor_select == MAX9295A_LI_OV2311_IR_SENSOR) {
			/*do nothing*/
		} else if(sensor_select == MAX9295A_AR0231_SENSOR){
			/* disable i2c to link B/C/D */
			max96722_write_reg(max96722_data, 0x0003, 0xBF);
			retval = max9295_initialize(max96722_data->i2c_client, max96722_9295_linkD_config);
			if (retval < 0){
				pr_err("%s: D can't init sensor-serlizer ic MAX9295A.\n", __func__);
				return retval;
			}
			pr_info("%s: D init sensor-serlizer ic MAX9295A ok.\n", __func__);
		} else if(sensor_select == MAX96705_OV01F10_SENSOR){
			max96722_max96705_init_ser(max96722_data,3,max96722_data->sensor_num);
		} else {
			// do nothing
		}
	}

	/* Enable all links*/
    if(sensor_select == MAX96717F_OV0X03C10_SENSOR ||
		sensor_select == MAX96717F_OV9284_SENSOR) {
		max96722_write_reg(max96722_data, 0x0006, max96722_data->sensor_is_there|0xF0);
		sensor_delay(5);

		/* put mipi phy at standby */
		max96722_write_reg(max96722_data, 0x08A2, 0x04);
		sensor_delay(5);

		/* put mipi phy at normal output */
		max96722_write_reg(max96722_data, 0x08A2, 0xF4);
		sensor_delay(10);
	} else if(sensor_select == MAX9295A_AR0231_SENSOR ||
		sensor_select == MAX9295A_LI_OV2311_IR_SENSOR){
		/* enable i2c to all links */
		max96722_write_reg(max96722_data, 0x0003, 0xAA);
		/* One-shot link reset on MAX96712 */
		max96722_write_reg(max96722_data, 0x0018, 0x0F);

		/* put mipi phy at standby */
		max96722_write_reg(max96722_data, 0x08A2, 0x04);
		sensor_delay(5);

		/* put mipi phy at normal output */
		max96722_write_reg(max96722_data, 0x08A2, 0xF4);
		sensor_delay(10);
	} else if(sensor_select == MAX96705_OV01F10_SENSOR){
		for (i = 0; i < max96722_data->sensor_num; i++){//open all the reverse channel
			max96722_read_reg(max96722_data, 0xb04+i*0x100,&value);
			max96722_write_reg(max96722_data, 0xb04+i*0x100, value|(1<<1));
			usleep_range(100,110);
		}
		/* put mipi phy at standby */
		max96722_write_reg(max96722_data, 0x08A2, 0x00);
		sensor_delay(5);

		/* put mipi phy at normal output */
		max96722_write_reg(max96722_data, 0x08A2, 0xF0);
		sensor_delay(10);
	} else {
		/* put mipi phy at standby */
		max96722_write_reg(max96722_data, 0x08A2, 0x00);
		sensor_delay(5);

		/* put mipi phy at normal output */
		max96722_write_reg(max96722_data, 0x08A2, 0xF0);
		sensor_delay(10);
	}

	if(sensor_select == MAX96705_OV01F10_SENSOR){
		max96722_write_reg(max96722_data, 0x08A0, 0x44);
	} else {
		max96722_write_reg(max96722_data, 0x08A0, 0x24);
	}
	return 0;

}

static int sensor_change_mode(struct sensor_data *max96722_data)
{
	struct reg_value *pModeSetting = NULL;
	enum sensor_mode mode = max96722_data->streamcap.capturemode;
	enum sensor_frame_rate rate =
				to_sensor_frame_rate(&max96722_data->streamcap.timeperframe);
	int ArySize = 0, retval = 0;

	if (mode > SENSOR_MODE_MAX || mode < SENSOR_MODE_MIN) {
		pr_err("Wrong sensor mode detected!\n");
		return -1;
	}

	pModeSetting = sensor_mode_info_data[rate][mode].init_data_ptr;
	ArySize = sensor_mode_info_data[rate][mode].init_data_size;

	max96722_data->format.width = sensor_mode_info_data[rate][mode].width;
	max96722_data->format.height = sensor_mode_info_data[rate][mode].height;

	if (max96722_data->format.width == 0 ||
		max96722_data->format.height == 0 ||
	    pModeSetting == NULL || ArySize == 0) {
		pr_err("Not support mode=%d %s\n", mode,
						(rate == 0) ? "15(fps)" : "30(fps)");
		return -EINVAL;
	}

	retval = sensor_download_firmware(max96722_data, 0, pModeSetting, ArySize);

	return retval;
}

static int max96722_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	struct sensor_data *max96722_data = subdev_to_sensor_data(sd);

	code->code = max96722_data->format.code;
	return 0;
}

/*!
 * max96722_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int max96722_enum_framesizes(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_frame_size_enum *fse)
{
	struct sensor_data *max96722_data = subdev_to_sensor_data(sd);
	u32 sensor_select = max96722_data->sensor_select;

	if (fse->index >= 1)
		return -EINVAL;

	if(sensor_select == MAX96717F_OV0X03C10_SENSOR || sensor_select == MAX9295A_AR0231_SENSOR) {
		fse->max_width = sensor_mode_info_data[fse->index + 1][fse->index + SENSOR_MODE_1080P_1920_1080].width;
		fse->min_width = fse->max_width;

		fse->max_height = sensor_mode_info_data[fse->index + 1][fse->index +SENSOR_MODE_1080P_1920_1080].height;
		fse->min_height = fse->max_height;
	} else if (sensor_select == MAX9295A_LI_OV2311_IR_SENSOR) {
		fse->max_width = sensor_mode_info_data[fse->index + 1][fse->index + SENSOR_MODE_WXGA_1600_1300].width;
		fse->min_width = fse->max_width;

		fse->max_height = sensor_mode_info_data[fse->index + 1][fse->index +SENSOR_MODE_WXGA_1600_1300].height;
		fse->min_height = fse->max_height;
	} else if(sensor_select == MAX96705_OV01F10_SENSOR){
		fse->max_width = sensor_mode_info_data[fse->index + 1][fse->index + SENSOR_MODE_720P_1280_720].width;
		fse->min_width = fse->max_width;

		fse->max_height = sensor_mode_info_data[fse->index + 1][fse->index + SENSOR_MODE_720P_1280_720].height;
		fse->min_height = fse->max_height;
	} else {
		fse->max_width = sensor_mode_info_data[fse->index + 1][fse->index + 1].width;
		fse->min_width = fse->max_width;

		fse->max_height = sensor_mode_info_data[fse->index + 1][fse->index + 1].height;
		fse->min_height = fse->max_height;
	}

#if 0
	if (fse->index > SENSOR_MODE_MAX)
		return -EINVAL;

	fse->max_width =
			max(sensor_mode_info_data[0][fse->index].width,
			    sensor_mode_info_data[1][fse->index].width);
	fse->min_width = fse->max_width;

	fse->max_height =
			max(sensor_mode_info_data[0][fse->index].height,
			    sensor_mode_info_data[1][fse->index].height);
	fse->min_height = fse->max_height;
#endif
	return 0;
}
static int max96722_enum_frame_interval(struct v4l2_subdev *sd,
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

static int max96722_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct sensor_data *max96722_data = subdev_to_sensor_data(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;

	if (fmt->pad)
		return -EINVAL;

	mf->code = max96722_data->format.code;
	mf->width =  max96722_data->format.width;
	mf->height = max96722_data->format.height;
	mf->colorspace = max96722_data->format.colorspace;
	mf->field = max96722_data->format.field;
	mf->reserved[0] = max96722_data->format.reserved[0];

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

static int max96722_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *fmt)
{
	struct sensor_data *max96722_data = subdev_to_sensor_data(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	int ret;

	if (fmt->pad)
		return -EINVAL;

	mf->code = max96722_data->format.code;
	mf->colorspace = max96722_data->format.colorspace;
	mf->field = V4L2_FIELD_NONE;

	try_to_find_resolution(max96722_data, mf);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	ret = sensor_change_mode(max96722_data);

	return ret;
}

static int max96722_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_frame_desc *fd)
{
	return 0;
}

static int max96722_set_frame_desc(struct v4l2_subdev *sd,
					unsigned int pad,
					struct v4l2_mbus_frame_desc *fd)
{
	return 0;
}

static int max96722_set_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

static int max96722_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sensor_data *max96722_data = subdev_to_sensor_data(sd);

	dev_info(sd->dev, "%s : %d , running:%d\n", __func__, enable, max96722_data->running);
	if (enable) {
		if (!max96722_data->running) {
			/* Enable MIPI output, set virtual channel according to the link number */
			if(max96722_data->sensor_select == MAX96705_OV01F10_SENSOR){
				max96722_write_reg(max96722_data, 0x08A0, 0xC4);
			} else {
				max96722_write_reg(max96722_data, 0x08A0, 0xA4);
			}
			dev_info(sd->dev, "stream on with %s on\n", __func__);
			sensor_delay(10);
		}
		max96722_data->running++;

	} else {
		max96722_data->running--;
		if (max96722_data->running == 0) {
			/* Disable MIPI Output */
			if(max96722_data->sensor_select == MAX96705_OV01F10_SENSOR){
				max96722_write_reg(max96722_data, 0x08A0, 0x44);
			} else {
				max96722_write_reg(max96722_data, 0x08A0, 0x24);
			}
			dev_info(sd->dev,"stream on with %s off\n", __func__);
		}
	}

	return 0;
}

static int max96722_link_setup(struct media_entity *entity,
				const struct media_pad *local,
				const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct v4l2_subdev_pad_ops max96722_pad_ops = {
	.enum_mbus_code		= max96722_enum_mbus_code,
	.enum_frame_size	= max96722_enum_framesizes,
	.enum_frame_interval	= max96722_enum_frame_interval,
	.get_fmt		= max96722_get_fmt,
	.set_fmt		= max96722_set_fmt,
	.get_frame_desc		= max96722_get_frame_desc,
	.set_frame_desc		= max96722_set_frame_desc,
};

static const struct v4l2_subdev_core_ops max96722_core_ops = {
	.s_power	= max96722_set_power,
};

static const struct v4l2_subdev_video_ops max96722_video_ops = {
	.s_stream		= max96722_s_stream,
};

static const struct v4l2_subdev_ops max96722_subdev_ops = {
	.core	= &max96722_core_ops,
	.pad	= &max96722_pad_ops,
	.video	= &max96722_video_ops,
};

static const struct media_entity_operations max96722_sd_media_ops = {
	.link_setup = max96722_link_setup,
};

static ssize_t analog_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sensor_data *max96722_data = subdev_to_sensor_data(sd);
	u8 val = 0;

	sensor_read_reg(max96722_data, 0, 0x370A, &val);
	return sprintf(buf, "%s\n", (val & 0x4) ? "enabled" : "disabled");
}

static ssize_t analog_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sensor_data *max96722_data = subdev_to_sensor_data(sd);
	char enabled[32];

	if (sscanf(buf, "%s", enabled) > 0) {
		if (strcmp(enabled, "enable") == 0)
			sensor_write_reg(max96722_data, 0, 0x370A, 0x4);
		else
			sensor_write_reg(max96722_data, 0, 0x370A, 0x0);
		return count;
	}
	return -EINVAL;
}

static DEVICE_ATTR(analog_test_pattern, 0644, analog_show, analog_store);

static const struct of_device_id max96722_of_match[] = {
	{ .compatible = "maxim,max96722_mipi" },
	{ .compatible = "maxim,max96722_mipi-x3c10" },
	{ .compatible = "maxim,max96722_mipi-ar0231" },
	{ .compatible = "maxim,max96722_mipi-ov9284" },
	{ .compatible = "maxim,max96722_mipi-li-ov2311-ir" },
	{ .compatible = "maxim,max96722_mipi_max96705" },
	{ /* sentinel */ }
};

/*!
 * max96722 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int max96722_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct sensor_data *max96722_data;
	struct v4l2_subdev *sd;
	int retval,pdn_gpio;
	u8 reg;

	max96722_data = devm_kzalloc(dev, sizeof(*max96722_data), GFP_KERNEL);
	if (!max96722_data)
		return -ENOMEM;

//TODO: if it is needed on se1000 with OX01F10
	if(of_device_is_compatible(dev->of_node, max96722_of_match[0].compatible)) {
#if defined(CONFIG_CAMERA_OV01F10)
		pr_info("max96722 device is ov 01f10 sensor");
		max96722_data->sensor_select = MAX96715_OV01F10_SENSOR;
#endif
	} else if(of_device_is_compatible(dev->of_node, max96722_of_match[1].compatible)) {
#if defined(CONFIG_CAMERA_OVX3C10)
		pr_info("max96722 device is ov OX03C10 sensor");
		max96722_data->sensor_select = MAX96717F_OV0X03C10_SENSOR;
#endif
	} else if(of_device_is_compatible(dev->of_node, max96722_of_match[2].compatible)) {
#if defined(CONFIG_CAMERA_AR0231)
		pr_info("max96722 device is onsemi ar0231 sensor");
		max96722_data->sensor_select = MAX9295A_AR0231_SENSOR;
#endif
	} else if(of_device_is_compatible(dev->of_node, max96722_of_match[3].compatible)) {
#if defined(CONFIG_CAMERA_OV9284)
		pr_info("max96722 device is ov 9284 sensor");
		max96722_data->sensor_select = MAX96717F_OV9284_SENSOR;
#endif
	} else if(of_device_is_compatible(dev->of_node, max96722_of_match[4].compatible)) {
#if defined(CONFIG_CAMERA_OV2311)
		pr_info("max96722 device is leopard ov 2311 ir sensor");
		max96722_data->sensor_select = MAX9295A_LI_OV2311_IR_SENSOR;
#endif
	} else if(of_device_is_compatible(dev->of_node, max96722_of_match[5].compatible)) {
		pr_info("max96722 device is MAX96705_OV01F10_SENSOR");
		max96722_data->sensor_select = MAX96705_OV01F10_SENSOR;
	} else {
#if defined(CONFIG_CAMERA_OV01F10)
		pr_info("max96722 Default use ov 01f10 sensor");
		max96722_data->sensor_select = MAX96715_OV01F10_SENSOR;
#endif
	}

	max96722_data->i2c_client = client;
	if(max96722_data->sensor_select == MAX96717F_OV0X03C10_SENSOR) {
		max96722_data->format.code = MEDIA_BUS_FMT_UYVY8_1X16 ;
		max96722_data->format.width = sensor_mode_info_data[1][SENSOR_MODE_1080P_1920_1080].width;
		max96722_data->format.height = sensor_mode_info_data[1][SENSOR_MODE_1080P_1920_1080].height;
		max96722_data->cap_mode.clip_height = max96722_data->format.height;
		max96722_data->cap_mode.clip_width = max96722_data->format.width;
	} else if (max96722_data->sensor_select == MAX9295A_AR0231_SENSOR) {
		max96722_data->format.code = MEDIA_BUS_FMT_SRGGB12_1X12 ;
		max96722_data->format.width = sensor_mode_info_data[1][SENSOR_MODE_1080P_1920_1080].width;
		max96722_data->format.height = sensor_mode_info_data[1][SENSOR_MODE_1080P_1920_1080].height;
		max96722_data->cap_mode.clip_height = max96722_data->format.height;
		max96722_data->cap_mode.clip_width = max96722_data->format.width;
	} else if (max96722_data->sensor_select == MAX96717F_OV9284_SENSOR) {
		max96722_data->format.code = MEDIA_BUS_FMT_UYVY8_1X16 ;
		max96722_data->format.width = sensor_mode_info_data[1][SENSOR_MODE_WXGA_1280_800].width;
		max96722_data->format.height = sensor_mode_info_data[1][SENSOR_MODE_WXGA_1280_800].height;
		max96722_data->cap_mode.clip_height = max96722_data->format.height;
		max96722_data->cap_mode.clip_width = max96722_data->format.width;
	} else if (max96722_data->sensor_select == MAX9295A_LI_OV2311_IR_SENSOR) {
		max96722_data->format.code = MEDIA_BUS_FMT_UYVY8_1X16;
		max96722_data->format.width = sensor_mode_info_data[1][SENSOR_MODE_WXGA_1600_1300].width;
		max96722_data->format.height = sensor_mode_info_data[1][SENSOR_MODE_WXGA_1600_1300].height;
		max96722_data->cap_mode.clip_height = max96722_data->format.height;
		max96722_data->cap_mode.clip_width = max96722_data->format.width;
	} else if (max96722_data->sensor_select == MAX96705_OV01F10_SENSOR) {
		max96722_data->format.code = MEDIA_BUS_FMT_UYVY8_1X16;
		max96722_data->format.width = sensor_mode_info_data[1][SENSOR_MODE_720P_1280_720].width;
		max96722_data->format.height = sensor_mode_info_data[1][SENSOR_MODE_720P_1280_720].height;
		max96722_data->cap_mode.clip_height = max96722_data->format.height;
		max96722_data->cap_mode.clip_width = max96722_data->format.width;
	} else {
		max96722_data->format.code = MEDIA_BUS_FMT_YUYV8_1X16;
		max96722_data->format.width = sensor_mode_info_data[1][0].width;
		max96722_data->format.height = sensor_mode_info_data[1][0].height;
		max96722_data->cap_mode.clip_height = max96722_data->format.height;
		max96722_data->cap_mode.clip_width = max96722_data->format.width;
	}

	max96722_data->format.colorspace = V4L2_COLORSPACE_JPEG;

	if (max96722_data->sensor_select == MAX96705_OV01F10_SENSOR) {
		pdn_gpio = of_get_named_gpio(dev->of_node, "pdn-gpios", 0);
		if (!gpio_is_valid(pdn_gpio)) {
			dev_err(&client->dev,"failed to parse pdn_gpio gpio\r\n");
		}
		else
		{
			gpio_request(pdn_gpio,"pdn-gpio");
			gpio_direction_output(pdn_gpio, 0);
			usleep_range(20000, 20100);
			gpio_direction_output(pdn_gpio, 1);
			usleep_range(20000, 20100);
		}
		max96722_data->power_en = of_get_named_gpio(dev->of_node, "power-en", 0);
		if (!gpio_is_valid(max96722_data->power_en)) {
				dev_err(&client->dev,"failed to parse power_en gpio\r\n");
		}
	}
	retval = max96722_read_reg(max96722_data, 0x0d, &reg);
	if ((retval != 0xa1)&&(retval != 0xa0)) {
		pr_warn("max96722 is not found, device id: 0x%x\n", retval);
		return -ENODEV;
	}
	/*****************************************
	 * Pass mipi phy clock rate Mbps
	 * fcsi2 = PCLk * WIDTH * CHANNELS / LANES
	 * fsci2 = 72MPCLK * 8 bit * 4 channels / 4 lanes
	 ****************************************/
	max96722_data->format.reserved[0] = 72 * 8;
	max96722_data->format.field = V4L2_FIELD_NONE;
	max96722_data->streamcap.capturemode = 0;
	max96722_data->streamcap.timeperframe.denominator = 30;
	max96722_data->streamcap.timeperframe.numerator = 1;
	max96722_data->is_mipi = 1;

	max96722_data->streamcap.capability = V4L2_CAP_TIMEPERFRAME;
	max96722_data->streamcap.timeperframe.denominator = 30;
	max96722_data->streamcap.timeperframe.numerator = 1;
	max96722_data->v_channel = 0;
	max96722_data->cap_mode.clip_top = 0;
	max96722_data->cap_mode.clip_left = 0;
	max96722_data->cap_mode.hlen = max96722_data->cap_mode.clip_width;

	max96722_data->cap_mode.hfp = 0;
	max96722_data->cap_mode.hbp = 0;
	max96722_data->cap_mode.hsync = 625;
	max96722_data->cap_mode.vlen = 800;
	max96722_data->cap_mode.vfp = 0;
	max96722_data->cap_mode.vbp = 0;
	max96722_data->cap_mode.vsync = 40;
	max96722_data->cap_mode.vlen1 = 0;
	max96722_data->cap_mode.vfp1 = 0;
	max96722_data->cap_mode.vbp1 = 0;
	max96722_data->cap_mode.vsync1 = 0;
	max96722_data->cap_mode.pixelclock = 27000000;

	sd = &max96722_data->subdev;
	v4l2_i2c_subdev_init(sd, client, &max96722_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	max96722_data->pads[MIPI_CSI2_SENS_VC0_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	max96722_data->pads[MIPI_CSI2_SENS_VC1_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	max96722_data->pads[MIPI_CSI2_SENS_VC2_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	max96722_data->pads[MIPI_CSI2_SENS_VC3_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	retval = media_entity_pads_init(&sd->entity, MIPI_CSI2_SENS_VCX_PADS_NUM,
							max96722_data->pads);
	if (retval < 0)
		return retval;

	max96722_data->subdev.entity.ops = &max96722_sd_media_ops;
#if defined(CONFIG_MEDIA_CONTROLLER)
	max96722_data->subdev.entity.flags = MEDIA_ENT_F_CAM_SENSOR;
#endif
	retval = v4l2_async_register_subdev(&max96722_data->subdev);
	if (retval < 0) {
		dev_err(&client->dev,
					"%s--Async register failed, ret=%d\n", __func__, retval);
		media_entity_cleanup(&sd->entity);
	}

	retval = max96722_hardware_init(max96722_data);
	if (retval < 0) {
		dev_err(&client->dev, "camera init failed\n");
		clk_disable_unprepare(max96722_data->sensor_clk);
		media_entity_cleanup(&sd->entity);
		v4l2_async_unregister_subdev(sd);
		return retval;
	}

	max96722_data->running = 0;

	/*Create device attr in sys */
	retval = device_create_file(&client->dev, &dev_attr_analog_test_pattern);
	if (retval < 0) {
		dev_err(&client->dev, "%s: create device file fail\n", __func__);
		return retval;
	}

	dev_set_drvdata(dev, max96722_data);

	dev_info(&max96722_data->i2c_client->dev,
			"max96722_mipi is found, name %s\n", sd->name);
	return retval;
}

/*!
 * max96722 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int max96722_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sensor_data *max96722_data = subdev_to_sensor_data(sd);

	clk_disable_unprepare(max96722_data->sensor_clk);
	device_remove_file(&client->dev, &dev_attr_analog_test_pattern);
	media_entity_cleanup(&sd->entity);
	v4l2_async_unregister_subdev(sd);
	dev_set_drvdata(&client->dev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int mx96722_runtime_suspend(struct device *dev)
{
	return 0;//pm_runtime_force_suspend(dev);
}

static int mx96722_runtime_resume(struct device *dev)
{
	struct sensor_data *max96722_data = dev_get_drvdata(dev);
	int retval;
	u8 reg;

	retval = max96722_read_reg(max96722_data, 0x0d, &reg);
	if (retval != 0xa1) {
		pr_warn("max96722 is not found, device id: 0x%x\n", retval);
		return -ENODEV;
	}

	retval = max96722_hardware_init(max96722_data);
	if (retval < 0) {
		pr_warn("camera init failed\n");
		return retval;
	}

	if (max96722_data->running) {
		if(max96722_data->sensor_select == MAX96705_OV01F10_SENSOR){
			max96722_write_reg(max96722_data, 0x08A0, 0xC4);
		} else {
			max96722_write_reg(max96722_data, 0x08A0, 0xA4);
		}
		pr_info("stream on\n");
		sensor_delay(10);
	}
	pr_info("mx96722_runtime_resume ok\n");

	//pm_runtime_force_suspend(dev);
	return 0;
}

#else
static int mx96722_runtime_suspend(struct device *dev)
{
	return 0;
}

static int mx96722_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops mx96722_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mx96722_runtime_suspend,
						mx96722_runtime_resume)
};


static const struct i2c_device_id max96722_id[] = {
	{},
};

MODULE_DEVICE_TABLE(i2c, max96722_id);

static struct i2c_driver max96722_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name   = "max96722_mipi",
		.pm = &mx96722_pm_ops,
		.of_match_table	= of_match_ptr(max96722_of_match),
	},
	.probe  = max96722_probe,
	.remove = max96722_remove,
	.id_table = max96722_id,
};

module_i2c_driver(max96722_i2c_driver);

MODULE_AUTHOR("Siengine Technology, Inc.");
MODULE_DESCRIPTION("MAX96722 GSML Deserializer Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");


