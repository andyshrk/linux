#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <asm/unaligned.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif
#ifdef CONFIG_IPO_MANAGER
#include <linux/ipo_manager.h>
#endif

static int geely_i2c_probe(struct i2c_client *client,
                            const struct i2c_device_id *id);

#ifdef CONFIG_ECARX_KEYPAD
extern void ecarx_key_value_report(u8 KeyID,u8 KeyValue);
extern void ecarx_vol_slide_report_data(u8 status,s32 x,s32 y);
#else
void ecarx_key_value_report(u8 KeyID,u8 KeyValue){}
void ecarx_vol_slide_report_data(u8 status,s32 x,s32 y){}
#endif

//#define DEBUG

#define csd_printk_error   pr_err
#define csd_printk_warning pr_warn
#define csd_printk_info    pr_info
#define csd_printk_debug   pr_debug


#define DEVICE_NAME "geely-720p-touchscreen"

#define CTP_SLAVE_SYNC 		0x40
#define CTP_SLAVE_POS 		0x41
#define CTP_SLAVE_POS_EXT 	0x42
#define CTP_SLAVE_BACKLIGHT 	0x46
#define CTP_SLAVE_BIST 		0x48
#define CTP_SLAVE_VOLTAGE  	0x49
#define CTP_SLAVE_VER   	0x4B
#define CTP_SLAVE_KEY_BC    	0x4C
#define CTP_SLAVE_KEY_BL    	0x4D
#define CTP_SLAVE_KEY_BR        0x4E
#define CTP_SLAVE_V_STATUS    	0x4F
#define CTP_SLAVE_V_INFO        0x50

#define CTP_SLAVE_MIN CTP_SLAVE_SYNC
#define CTP_SLAVE_MAX CTP_SLAVE_V_INFO

#define CTP_MASTER_SYNC		0x80
#define CTP_MASTER_QUEST	0x81
#define CTP_MASTER_BC		0x91
#define CTP_MASTER_BL		0x92
#define CTP_MASTER_BR		0x93
#define DISP_MASTER_SUBX_BL	0x90

#define CTP_UPGRADE_MMITODISP			0x7E
#define	CTP_UPGRADE_DISPTOMMI			0x7F

#define CTP_SLAVE_INIT_SIZE     (1 + 8 + 1 + 1)
#define CTP_SLAVE_NORMAL_SIZE   (1 + 20 + 1 + 1)
#define CTP_SLAVE_RPS_SIZE      (1 + 1 + 1 + 1)
#define CTP_SLAVE_VS_SIZE       (1 + 3 + 1 + 1)
#define CTP_SLAVE_VI_SIZE       (1 + 2 + 1 + 1)
#define DISP_SLAVE_BL_SIZE      (1 + 8 + 1 + 1)

#define MAX_PACKET_SIZE 256

#define MAX_SUPPORT_POINTS		10

#define UB949_GEN_CFG 0x03
//#define I2C_PASS_THROUGH BIT(3)
#define UB949_SLAVE_ID0 0x07
#define UB949_SLAVE_ALIAS0  0x08


#define CTP_VOLT_NORM 0
#define CTP_VOLT_HIGH 1
#define CTP_VOLT_LOW 2

#define CTP_DEFAULT_X_MAX   1920
#define CTP_DEFAULT_Y_MAX   720

#define CTP_TOUCH_MOVE BIT(4)
#define CTP_TOUCH_UP BIT(5)
#define CTP_TOUCH_DOWN BIT(6)
#define CTP_TOUCH_VALID BIT(7)


#define QUIRK_DROP_UNUSED BIT(0)
#define QUIRK_MT_UNRELIABLE BIT(1)
#define QUIRK_DUP_IRQ BIT(2)

/* csd  V+ / V- key info Mask */
#define CSD_V_KEY_V_INC		0x01
#define CSD_V_KEY_V_DESC	0x02

#define I2C_7BIT_ADDR(x)   (x>>1)

struct geely_data {
	struct i2c_client *client;
	struct input_dev *input;

	struct miscdevice mdev;
	struct file_operations fops;

	int state;

	int ser_addr;
	const char* ser_name;
	int deser_addr;
	const char* deser_name;

	u8 expected_op;

	u32 x_max;
	u32 y_max;

	u8 buf[MAX_PACKET_SIZE];

	bool down[MAX_SUPPORT_POINTS];

	bool wake_irq_enabled;
	bool keep_power_in_suspend;

	u32 quirk;
	char input_name[128];
};


struct ctp_touch_point {
	u8 id;
	u8 status;
	u8 xpos[2];
	u8 ypos[2];
	u8 area;
	u8 pressure;
	u8 direction;
} __attribute__((packed));

struct ctp_touch {
	u8 contact;
	struct ctp_touch_point pts[2];
	u8 reserved;
} __attribute__((packed));

struct ctp_init_status {
	u8 success;
};

struct ctp_bist_status {
	u8 overall;
	u8 display;
	u8 touch;
	u8 temperature;
};

struct ctp_voltage {
	u8 voltage_range;
};

struct ctp_version {
	u8 hw_major;
	u8 hw_minor;
	u8 sw_major;
	u8 sw_minor;
};

struct ctp_key_bc{
	u8 key_backlight_control;
};

struct ctp_key_bl{
	u8 key_backlight_level;
};

struct ctp_key_br{
	u8 key_backlight_brightness;
};


struct ctp_slide_info{
	u8 num_touch;
	u8 key_status;
	u8 x_pos;
};

struct ctp_v_info{
	u8 which_key;
	u8 key_status;
};

static u8 next_state[256] = {
	[CTP_SLAVE_SYNC] = CTP_SLAVE_VER,
	[CTP_SLAVE_BIST] = CTP_SLAVE_POS,
	[CTP_SLAVE_VOLTAGE] = CTP_SLAVE_POS,
	[CTP_SLAVE_VER] = CTP_SLAVE_POS,
	[CTP_SLAVE_POS] = CTP_SLAVE_POS,
	[CTP_SLAVE_POS_EXT] = CTP_SLAVE_POS,
	[CTP_SLAVE_KEY_BC] = CTP_SLAVE_POS,

};

static u8 packet_size[256] = {
	[CTP_SLAVE_SYNC] = CTP_SLAVE_INIT_SIZE,
	[CTP_SLAVE_BIST] = CTP_SLAVE_INIT_SIZE,
	[CTP_SLAVE_VOLTAGE] = CTP_SLAVE_INIT_SIZE,
	[CTP_SLAVE_VER] = CTP_SLAVE_INIT_SIZE,
	[CTP_SLAVE_POS] = CTP_SLAVE_NORMAL_SIZE,
	[CTP_SLAVE_KEY_BC] = CTP_SLAVE_RPS_SIZE,
	[CTP_SLAVE_KEY_BL] = CTP_SLAVE_RPS_SIZE,
	[CTP_SLAVE_KEY_BR] = CTP_SLAVE_RPS_SIZE,
	[CTP_SLAVE_V_STATUS] = CTP_SLAVE_VS_SIZE,
	[CTP_SLAVE_V_INFO] = CTP_SLAVE_VI_SIZE,
};

struct geely_data *g_sub1_ts = NULL;
struct geely_data *g_sub2_ts = NULL;

static const struct i2c_device_id geely_i2c_id[] = {
	{ "geely-generic-ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, geely_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id geely_of_match[] = {
	{ .compatible = "geely,generic-ts" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, geely_of_match);
#endif

static bool ub983_i2c_write_register(struct i2c_client *client, unsigned char addr, unsigned char data)
{
	u8 tmp[2];
	struct i2c_msg msg;
	int res;

	tmp[0] = addr;
	tmp[1] = data;

	msg.addr = client->addr;
	msg.flags = 0;		/* write */
	msg.buf = tmp;
	msg.len = 2;

	res = i2c_transfer(client->adapter, &msg, 1);
	if (res != 1)
	{
		csd_printk_debug("i2c_transfer addr:0x%x val:0x%x failed\n", addr, data);
	}else
		return true;;

	return false;
}

static unsigned char ub983_i2c_read_register(struct i2c_client *client, unsigned char addr)
{
	int i = 0;
	int ret = 0;
	unsigned char buf;

	struct i2c_msg msg[2] =
	{
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &addr,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= &buf,
		}
	};

	for(i = 0; i < 3; i++)
	{
		ret = i2c_transfer(client->adapter, msg, 2);
		if (ret != 2)
		{
			csd_printk_debug("i2c_transfer error ret =%d\n", ret);
		}
		else
			return buf;;
	}
	return false;
}

static bool ub983_write(struct i2c_client *client, unsigned char i2caddr, unsigned char addr, unsigned char data)
{
	int ret = 0;
	client->addr = i2caddr;
	ret = ub983_i2c_write_register(client,addr,data);
	if (ret == 0) {
		csd_printk_debug("ub983_write error! ret(%d) addr(0x%2x) reg(0x%2x) data(0x%2x)\n",ret,i2caddr,addr,data);
	}
	return ret;
}
static unsigned char ub983_read(struct i2c_client *client, unsigned char i2caddr, unsigned char addr)
{
	int ret = 0;
	client->addr = i2caddr;
	ret = ub983_i2c_read_register(client,addr);
	if (ret == 0) {
		csd_printk_debug("ub983_read error! ret(%d) addr(0x%2x) reg(0x%2x)\n",ret,i2caddr,addr);
	}
	return ret;
}

#define SERADDR    (0x18)
#define DESADDR0   (0x58)
#define DESALIAS0  (0x58)

struct ub98x_reg_cmds{
	char i2c_addr;
	char addr;
	char val;
};

const struct ub98x_reg_cmds reg_init_720p_cmd0[] = {
	{SERADDR,0x70,DESADDR0},
	{SERADDR,0x78,DESALIAS0},
	{SERADDR,0x88,0x0},
	// Set DP Config;
	{SERADDR,0x48,0x1},//Enable APB Interface;
	{SERADDR,0x49,0x0},//Force HPD low to configure 983 DP settings;
	{SERADDR,0x4a,0x0},
	{SERADDR,0x4b,0x0},
	{SERADDR,0x4c,0x0},
	{SERADDR,0x4d,0x0},
	{SERADDR,0x4e,0x0},
	{SERADDR,0x49,0x74},//Set max advertised link rate = 2.7Gbps;
	{SERADDR,0x4a,0x0},
	{SERADDR,0x4b,0xa},
	{SERADDR,0x4c,0x0},
	{SERADDR,0x4d,0x0},
	{SERADDR,0x4e,0x0},
	{SERADDR,0x49,0x70},//Set max advertised lane count = 4;
	{SERADDR,0x4a,0x0},
	{SERADDR,0x4b,0x4},
	{SERADDR,0x4c,0x0},
	{SERADDR,0x4d,0x0},
	{SERADDR,0x4e,0x0},
	{SERADDR,0x49,0x14},//Request min VOD swing of 0x02;
	{SERADDR,0x4a,0x2},
	{SERADDR,0x4b,0x2},
	{SERADDR,0x4c,0x0},
	{SERADDR,0x4d,0x0},
	{SERADDR,0x4e,0x0},
	{SERADDR,0x49,0x18},//Set SST/MST mode and DP/eDP Mode;
	{SERADDR,0x4a,0x0},
	{SERADDR,0x4b,0x14},
	{SERADDR,0x4c,0x0},
	{SERADDR,0x4d,0x0},
	{SERADDR,0x4e,0x0},
	{SERADDR,0x49,0x0},//Force HPD high to trigger link training;
	{SERADDR,0x4a,0x0},
	{SERADDR,0x4b,0x1},
	{SERADDR,0x4c,0x0},
	{SERADDR,0x4d,0x0},
	{SERADDR,0x4e,0x0},
};

const struct ub98x_reg_cmds reg_init_720p_cmd1[] = {
	{SERADDR,0x40,0x8},
	{SERADDR,0x41,0x4},
	{SERADDR,0x42,0x9},//Set fractional mash order;
	{SERADDR,0x41,0x13},
	{SERADDR,0x42,0xe0},//Set VCO Post Div = 4, VCO Auto Sel for CS2.0;
	{SERADDR,0x40,0xa},//Set auto increment;
	{SERADDR,0x41,0x5},
	{SERADDR,0x42,0x7c},//Set Ndiv = 124;
	{SERADDR,0x42,0x0},//Set Ndiv = 124;
	{SERADDR,0x41,0x18},
	{SERADDR,0x42,0x76},//Set denominator = 16773750;
	{SERADDR,0x42,0xf2},//Set denominator = 16773750;
	{SERADDR,0x42,0xff},//Set denominator = 16773750;
	{SERADDR,0x41,0x1e},
	{SERADDR,0x42,0xac},//Set numerator = 817068;
	{SERADDR,0x42,0x77},//Set numerator = 817068;
	{SERADDR,0x42,0xc},//Set numerator = 817068;
	{SERADDR,0x1,0x30},//PLL Reset;
};

const struct ub98x_reg_cmds reg_init_720p_cmd2[] = {
	{SERADDR,0x40,0x4},//Set FPD Page to configure BC Settings for Port 0;
	{SERADDR,0x41,0x6},
	{SERADDR,0x42,0xff},
	{SERADDR,0x41,0xd},
	{SERADDR,0x42,0x70},
	{SERADDR,0x41,0xe},
	{SERADDR,0x42,0x70},
	{SERADDR,0x1,0x30},//Reset PLLs;
	// Issue DP video reset after video is available from the DP source;
	{SERADDR,0x49,0x0},//Read back Horizontal Resolution from DP APB;
	{SERADDR,0x4a,0x5},
	{SERADDR,0x48,0x3},
};

const struct ub98x_reg_cmds reg_init_720p_cmd3[] = {
	{SERADDR,0x49,0x54},//Video Input Reset (should be executed after DP video is available from the source},
	{SERADDR,0x4a,0x0},
	{SERADDR,0x4b,0x1},
	{SERADDR,0x4c,0x0},
	{SERADDR,0x4d,0x0},
	{SERADDR,0x4e,0x0},
	// Program VP Configs;
	// Configure VP 0;
	{SERADDR,0x40,0x32},
	{SERADDR,0x41,0x1},
	{SERADDR,0x42,0xa8},//Set VP_SRC_SELECT to Stream 0 for SST Mode;
	{SERADDR,0x41,0x2},
	{SERADDR,0x42,0x80},//VID H Active;
	{SERADDR,0x42,0x7},//VID H Active;
	{SERADDR,0x41,0x10},
	{SERADDR,0x42,0x80},//Horizontal Active;
	{SERADDR,0x42,0x7},//Horizontal Active;
	{SERADDR,0x42,0x38},//Horizontal Back Porch;
	{SERADDR,0x42,0x0},//Horizontal Back Porch;
	{SERADDR,0x42,0x20},//Horizontal Sync;
	{SERADDR,0x42,0x0},//Horizontal Sync;
	{SERADDR,0x42,0xe8},//Horizontal Total;
	{SERADDR,0x42,0x7},//Horizontal Total;
	{SERADDR,0x42,0xd0},//Vertical Active;
	{SERADDR,0x42,0x2},//Vertical Active;
	{SERADDR,0x42,0x10},//Vertical Back Porch;
	{SERADDR,0x42,0x0},//Vertical Back Porch;
	{SERADDR,0x42,0x8},//Vertical Sync;
	{SERADDR,0x42,0x0},//Vertical Sync;
	{SERADDR,0x42,0x2c},//Vertical Front Porch;
	{SERADDR,0x42,0x0},//Vertical Front Porch;
	{SERADDR,0x41,0x27},
	{SERADDR,0x42,0x0},//HSYNC Polarity = +, VSYNC Polarity = +;
	// Enable VPs;
	{SERADDR,0x43,0x0},//Set number of VPs used = 1;
	{SERADDR,0x44,0x1},//Enable video processors;
	// Enable PATGEN;
	{SERADDR,0x40,0x30},
	{SERADDR,0x41,0x29},
	{SERADDR,0x42,0x8},//Set PATGEN Color Depth to 24bpp for VP0;
	{SERADDR,0x41,0x28},
	{SERADDR,0x42,0x91},//Enable PATGEN on VP0 - Comment out this line to disable PATGEN and enable end to end video;
	// Set FPD3 Stream Mapping;
	{SERADDR,0x2d,0x1},//Select FPD TX Port 0;
	{SERADDR,0x57,0x0},//Set FPD TX Port 0 Stream Source = VP0;
	{SERADDR,0x5b,0x2b},//Enable FPD III FIFO;
};

static void ub983_regwrite_seq_handler(struct i2c_client *client,const struct ub98x_reg_cmds *cmds,int len)
{
	int i,ret;
	for(i = 0;i < len;i++){
        ret = ub983_write(client,I2C_7BIT_ADDR(cmds[i].i2c_addr),cmds[i].addr,cmds[i].val);
        if(!ret)
            csd_printk_debug("ub983 reg init has error,line=%d\r\n",__LINE__);
    }
}

static void ub983_reg_init(struct i2c_client *client) 
{
	unsigned char GENERAL_CFG = 0;
	unsigned char FPD3Mask = 0;
	unsigned char GENERAL_CFG_REG = 0;
	unsigned char FPD4_CFG = 0;
	unsigned char FPD4_CFG_REG = 0;
	unsigned char TX_MODE_MASK = 0;
	unsigned char HRes = 0;
	unsigned char VRes = 0;
	unsigned char apbData0 = 0;
	unsigned char apbData1 = 0;
	unsigned char apbData2 = 0;
	unsigned char apbData3 = 0;
	unsigned char apbData = 0;
	unsigned char Reg_value = 0;
// TI Confidential - NDA Restrictions;
// ;
// Copyright 2018 Texas Instruments Incorporated. All rights reserved.;
// ;
// IMPORTANT: Your use of this Software is limited to those specific rights;
// granted under the terms of a software license agreement between the user who;
// downloaded the software, his/her employer (which must be your employer) and;
// Texas Instruments Incorporated (the License). You may not use this Software;
// unless you agree to abide by the terms of the License. The License limits your;
// use, and you acknowledge, that the Software may not be modified, copied or;
// distributed unless embedded on a Texas Instruments microcontroller which is;
// integrated into your product. Other than for the foregoing purpose, you may;
// not use, reproduce, copy, prepare derivative works of, modify, distribute,;
// perform, display or sell this Software and/or its documentation for any;
// purpose.;
// ;
// YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE;
// PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,;
// INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,;
// NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL TEXAS;
// INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,;
// NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL;
// EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT;
// LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL;
// DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS,;
// TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT;
// LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.;
// ;
// Should you have any questions regarding your right to use this Software,;
// contact Texas Instruments Incorporated at www.TI.com.;
// ;
// DS90Ux98x-Q1 Auto Script Generation Output;
// Tool Version 3.0;
//import time ;
// Serializer: DS90Ux983-Q1;
// User Inputs:;
// Serializer I2C Address= 0x18;
// Max DP Lane Count = 4;
// Max DP Lane Rate = 2.7Gbps;
// DPRX SSC Mode Enabled;
// DP SST Mode Enabled;
// DP Mode Enabled;
// FPD-Link Configuration: FPD-Link III Single Port 0;
// Number of Displays = 1;
// Video Processor 0 (Stream 0) Properties:;
// Total Horizontal Pixels = 2024;
// Total Vertical Lines = 788;
// Active Horizontal Pixels = 1920;
// Active Vertical Lines = 720;
// Horizontal Back Porch = 56;
// Vertical Back Porch = 16;
// Horizontal Sync = 32;
// Vertical Sync = 8;
// Horizontal Front Porch = 16;
// Vertical Front Porch = 44;
// Horizontal Sync Polarity = Positive;
// Vertical Sync Polarity = Positive;
// Bits per pixel = 24;
// Pixel Clock = 95.69472MHz;
// PATGEN Disabled;
// *********************************************;
// Set up Variables;
// *********************************************;
	//SERADDR = 0x18;
	//DESADDR0 = 0x58;
	//DESALIAS0 = 0x58;
	ub983_regwrite_seq_handler(client, reg_init_720p_cmd0, ARRAY_SIZE(reg_init_720p_cmd0));
	//mdelay(0.5*10000) ;// Allow time after HPD is pulled high for the source to train and provide video (may need to adjust based on source properties*10000);
	// *********************************************;
	// Set FPD Port Configuration;
	// *********************************************;
	GENERAL_CFG = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x7);
	FPD3Mask = 0x01;
	GENERAL_CFG_REG = GENERAL_CFG | FPD3Mask;
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x07,GENERAL_CFG_REG) ;// Set FPD III Mode;
	FPD4_CFG = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x5);
	TX_MODE_MASK = 0xC3;
	FPD4_CFG_REG = FPD4_CFG & TX_MODE_MASK;
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x05,FPD4_CFG_REG) ;// Set FPD III Mode;
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x59,0x1) ;//Set FPD3_TX_MODE to FPD III Single Port 0;
	// *********************************************;
	// Program PLLs;
	// *********************************************;
	// Program PLL for Port 0: FPD III Mode 3349.3152Mbps;
	ub983_regwrite_seq_handler(client, reg_init_720p_cmd1, ARRAY_SIZE(reg_init_720p_cmd1));
	mdelay(0.1*10000);
	ub983_regwrite_seq_handler(client, reg_init_720p_cmd2, ARRAY_SIZE(reg_init_720p_cmd2));
	apbData0 = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x4b);
	apbData1 = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x4c);
	apbData2 = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x4d);
	apbData3 = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x4e);
	apbData = (apbData3<<24) | (apbData2<<16) | (apbData1<<8) | (apbData0<<0);
	HRes = apbData;
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x49,0x14) ;//Read back Vertical Resolution from DP APB;
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x4a,0x5);
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x48,0x3);
	apbData0 = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x4b);
	apbData1 = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x4c);
	apbData2 = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x4d);
	apbData3 = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x4e);
	apbData = (apbData3<<24) | (apbData2<<16) | (apbData1<<8) | (apbData0<<0);
	VRes = apbData;
	//print "Detected DP Input Resolution: ", HRes, "x", VRes;
	if( HRes == 0 || VRes == 0) {
	//  print "Warning, no DP Video Input to 983 Detected - try adding more delay after HPD is pulled high in the Set DP Config Section";
	}
	ub983_regwrite_seq_handler(client, reg_init_720p_cmd3, ARRAY_SIZE(reg_init_720p_cmd3));
	// Clear CRC errors from initial link process;
	Reg_value = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x2);
	Reg_value = Reg_value | 0x20;
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x2,Reg_value) ;//CRC Error Reset;
	Reg_value = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x2);
	Reg_value = Reg_value & 0xdf;
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x2,Reg_value) ;//CRC Error Reset Clear;
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x2d,0x1);
	ub983_i2c_write_register(client,0x1b,0x88);
	ub983_i2c_write_register(client,0x7 ,0x99);
}

static int geely_ts_suspend(struct device *dev)
{
	int ret = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	csd_printk_debug("disable irq=%d\n", client->irq);

	disable_irq(client->irq);

	ret = pinctrl_pm_select_sleep_state(dev);
	if(ret < 0)
		csd_printk_error("failed to suspend,ret: (%d)\n", ret);

	return ret;
}

static int geely_ts_resume(struct device *dev)
{
	int ret = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	ret = pinctrl_pm_select_default_state(dev);
	if(ret < 0)
		csd_printk_error("failed to resume,ret: (%d)\n", ret);

	csd_printk_debug("enable irq=%d\n", client->irq);

	enable_irq(client->irq);

	return ret;
}

struct dev_pm_ops geely_i2c_pm_ops = {
	.suspend = geely_ts_suspend,
	.resume  = geely_ts_resume,
};

static struct i2c_driver geely_i2c_driver = {
	.probe = geely_i2c_probe,
	.id_table = geely_i2c_id,
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = of_match_ptr(geely_of_match),
		.pm = &geely_i2c_pm_ops,
	},
};

static u8 ctp_master_sync[5] 	= {CTP_MASTER_SYNC, 0x01};
static u8 ctp_master_quest[5] 	= {CTP_MASTER_QUEST};
static u8 ctp_master_key_bc[2] 	= {CTP_MASTER_BC};
static u8 ctp_master_key_bl[2] = {CTP_MASTER_BL};
static u8 ctp_master_key_br[2] 	= {CTP_MASTER_BR};
static u8 disp_master_subx_bl[5]	= {DISP_MASTER_SUBX_BL};

static struct ctp_version ctp_ver;
static struct ctp_key_bc ctp_bc;
static struct ctp_key_bl ctp_bl;
static struct ctp_key_br ctp_br;
static struct ctp_voltage ctp_vol;
static struct ctp_bist_status ctp_bs;

inline bool ctp_mt_unreliable(struct geely_data *ts)
{
	return (ts->quirk & QUIRK_MT_UNRELIABLE);
}

inline bool ctp_drop_unused(struct geely_data *ts)
{
	return (ts->quirk & QUIRK_DROP_UNUSED);
}


inline static u8 ctp_get_checksum(u8* buf, int size)
{
	int i;
	u8 sum = 1;

	for (i = 0; i < size; i++)
		sum += buf[i];

	return sum;
}

static void ctp_checksum(u8* buf, int size)
{
	buf[size - 1] = ctp_get_checksum(buf, size - 1);
}

static bool ctp_validate_checksum(u8* buf, int size)
{
	return buf[size - 1] == ctp_get_checksum(buf, size - 1);
}
/*
static s32 ser_read_reg(struct geely_data *ts, u8 addr)
{
	struct i2c_client * client = ts->client;
	union i2c_smbus_data data;
	int status;

	status = i2c_smbus_xfer(client->adapter, ts->ser_addr, 0,
				I2C_SMBUS_READ, addr,
				I2C_SMBUS_BYTE_DATA, &data);
	return (status < 0) ? status : data.byte;
}
*/

static int ser_write_16bit_addr_reg(struct geely_data *ts, u16 addr, u8 value)
{
	struct i2c_client * client = ts->client;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	int ret = -1;
	u8 buf[3];
	buf[0] = (unsigned char)((addr >> 8) & 0xff);
	buf[1] = (unsigned char)(addr & 0x00ff);
	buf[2] = value;
	//ret = i2c_master_send(client, buf, sizeof(buf));
	msg.addr = ts->ser_addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = sizeof(buf);
	msg.buf = (char *)buf;

	ret = i2c_transfer(adap, &msg, 1);

	/*
	 * If everything went ok (i.e. 1 msg transmitted), return #bytes
	 * transmitted, else error code.
	 */
	ret = (ret == 1) ? sizeof(buf) : ret;

	if(ret < 0)
		csd_printk_error("%s %s error, addr=%04x value=%02x\n", ts->input_name, __func__, addr, value);
	else
		csd_printk_warning("%s %s successfully, addr=%04x value=%02x\n", ts->input_name, __func__, addr, value);
	return ret;
}

static int i2c_device_is_alive(struct geely_data *ts, int addr)
{
	struct i2c_client * client = ts->client;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	int ret;
	u8 tmp[32];

	msg.addr = addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = 1;
	msg.buf = tmp;

	ret = i2c_transfer(adap, &msg, 1);

	/*
	 * If everything went ok (i.e. 1 msg received), return #bytes received,
	 * else error code.
	 */
	return (ret == 1) ? 0 : ret;
}

static void MAX96789_init(struct geely_data *ts)
{
	//ser_write_16bit_addr_reg(ts, 0x01, 0x08);
	//TP interrupt GPIO
	ser_write_16bit_addr_reg(ts, 0x02C4, 0x04);
	ser_write_16bit_addr_reg(ts, 0x02C5, 0x60);
	ser_write_16bit_addr_reg(ts, 0x02C6, 0x01);
}
static int ctp_send_packet_nocksum(struct geely_data *ts, u8 *buf, int size)
{
	struct i2c_client *client = ts->client;
	int ret;

	if (size > MAX_PACKET_SIZE - 1)
		return -E2BIG;

	memcpy(ts->buf, buf, size);
	/* ext len */
	//ts->buf[size++] = 0;
	/* csum */
	//ctp_checksum(ts->buf, ++size);

#ifdef DEBUG
	print_hex_dump(KERN_DEBUG, ">>> ", DUMP_PREFIX_OFFSET, 16, 1,
		ts->buf, size, false);
#endif

	ret = i2c_master_send(client, ts->buf, size);
	if (ret >= 0) {
		if (ret == size)
			ret = 0;
		else
			ret = -EPROTO;
	}

	return ret;
}

static int ctp_send_packet(struct geely_data *ts, u8 *buf, int size)
{
	struct i2c_client *client = ts->client;
	int ret;

	if (size > MAX_PACKET_SIZE - 1)
		return -E2BIG;

	memcpy(ts->buf, buf, size);
	/* ext len */
	ts->buf[size++] = 0;
	/* csum */
	ctp_checksum(ts->buf, ++size);

#ifdef DEBUG
	print_hex_dump(KERN_DEBUG, ">>> ", DUMP_PREFIX_OFFSET, 16, 1,
		ts->buf, size, false);
#endif

	ret = i2c_master_send(client, ts->buf, size);
	if (ret >= 0) {
		if (ret == size)
			ret = 0;
		else
			ret = -EPROTO;
	}

	return ret;
}

static int mmi_quest_csd(struct geely_data *ts)
{
	int ret = -1;

	ret = ctp_send_packet(ts,ctp_master_quest,sizeof(ctp_master_quest));
	if (ret < 0) {
		csd_printk_error("ctp send syn failed %d\n", ret);
	}
	return ret;

}

static int csd_set_key_bc(struct geely_data *ts,int status)
{
	int ret = -1;

	ctp_master_key_bc[1] = !!status;
	ret = ctp_send_packet(ts,ctp_master_key_bc,sizeof(ctp_master_key_bc));
	if (ret < 0) {
		csd_printk_error("ctp send syn failed %d\n", ret);
	}
	return ret;
}

static int csd_set_key_bl(struct geely_data *ts,int level)
{
	int ret = -1;

	ctp_master_key_bl[1] = level;
	ret = ctp_send_packet(ts,ctp_master_key_bl,sizeof(ctp_master_key_bl));
	if (ret < 0) {
		csd_printk_error("ctp send syn failed %d\n", ret);
	}
	return ret;
}

static int disp_set_subx_bl(struct geely_data *ts, u8 level, int disp_mode)
{
	int ret = -1;
	disp_master_subx_bl[1] =  disp_mode;
	disp_master_subx_bl[2] =  (level*100/255)|0x80;

	ret = ctp_send_packet(ts, disp_master_subx_bl, sizeof(disp_master_subx_bl));
	if (ret < 0) {
		csd_printk_error("%s disp_subx_bl failed %d\n", ts->input->name, ret);
	}else {
		csd_printk_debug("%s disp_subx_bl set brightness val[0-255]: %d\n", ts->input->name, level);
	}

	return ret;
}

int bl_set_subx_brightness(u8 level, int disp_mode, const char *devname)
{
	if(strstr(devname, "sub1") && g_sub1_ts)
		return disp_set_subx_bl(g_sub1_ts, level, disp_mode);
	else if(strstr(devname, "sub2") && g_sub2_ts)
		return disp_set_subx_bl(g_sub2_ts, level, 0);
	else
	{
		csd_printk_error("g_subx_ts struct is NULL\n");
		return -ENODEV;
	}
}

static int csd_set_key_br(struct geely_data *ts,int brightness)
{
	int ret = -1;

	ctp_master_key_br[1] = brightness;
	ret = ctp_send_packet(ts,ctp_master_key_br,sizeof(ctp_master_key_br));
	if (ret < 0) {
		csd_printk_error("ctp send syn failed %d\n", ret);
	}
	return ret;
}


static int ctp_recv_packet(struct geely_data *ts)
{
	int len;
	int expected;
	u8 opcode;
	struct i2c_client *client = ts->client;

	expected = packet_size[ts->state];
	if (!expected) {
		expected = CTP_SLAVE_NORMAL_SIZE;
	}

	len = i2c_master_recv(client, ts->buf, expected);
	if (len <= 0){
		csd_printk_error("failed to read data: %d\n",
			 len);
        if (!len)
            len = -EIO;
		return len;
	}

#ifdef DEBUG
	print_hex_dump(KERN_INFO, "<<< ", DUMP_PREFIX_OFFSET, 16, 1,
		ts->buf, len, false);
#endif

	opcode = ts->buf[0];
	if (opcode != ts->state) {
		int size = packet_size[opcode];

		if (opcode == CTP_SLAVE_POS_EXT) {
			csd_printk_error(" not expect opcode %02x here\n",opcode);
			print_hex_dump(KERN_INFO, "OOO ", DUMP_PREFIX_OFFSET, 16, 1,
				ts->buf, len, false);
			return -EINVAL;
		}

		if (!size)
			return -EINVAL;

		if (size > len) {
			csd_printk_error("short read due to state error expect %02x, got %02x\n",
				ts->state, opcode);
			ts->state = opcode;
			return -EAGAIN;
		}

		csd_printk_debug("state %02x => %02x, size %d => %d\n",
			ts->state, opcode, len, size);
		len = size;
		ts->state = opcode;
	}

	if (!ctp_validate_checksum(ts->buf, len)) {
		csd_printk_error(" checksum error, possible overrun\n");
		print_hex_dump(KERN_INFO, "CCC ", DUMP_PREFIX_OFFSET, 16, 1,
			ts->buf, len, false);
		return -EIO;
    	}

	return 0;
}

static int ctp_recv_ext_packet(struct geely_data *ts, int size)
{
	int len;
	struct i2c_client *client = ts->client;

	len = i2c_master_recv(client, ts->buf, size);
	if (len < 0) {
		csd_printk_error("failed to read data: %d\n",
			len);
		return len;
	}

	if (len != size) {
		csd_printk_error(" failed to read data: %d expect %d\n",len, size);
		return -EINVAL;
	}

#ifdef DEBUG
	print_hex_dump(KERN_INFO, "<<< ", DUMP_PREFIX_OFFSET, 16, 1,
		ts->buf, len, false);
#endif

	if (!ctp_validate_checksum(ts->buf, size)) {
		csd_printk_debug("checksum error, possible overrun.\n");
		print_hex_dump(KERN_INFO, "CCC ", DUMP_PREFIX_OFFSET, 16, 1,
			ts->buf, len, false);
		return -EIO;
	}

	return 0;
}

static int ctp_process_one_touch(struct geely_data *ts, struct ctp_touch_point* touch)
{
	struct input_dev* input = ts->input;
	bool up = touch->status & CTP_TOUCH_UP;
	bool down = touch->status & (CTP_TOUCH_MOVE|CTP_TOUCH_DOWN);

	if (!touch->status)
		return 0;
    // if (!(touch->status & CTP_TOUCH_VALID))
    //     return 0;

	if (!(up ^ down)) {
		pr_info("invalid status combination %02x\n", touch->status);
		print_hex_dump(KERN_DEBUG, "BBB ", DUMP_PREFIX_OFFSET, 16, 1,
                        ts->buf, 32, false);
		return -EINVAL;
	}

	if (ctp_mt_unreliable(ts)) {
		if (touch->id <= MAX_SUPPORT_POINTS)
			ts->down[touch->id - 1] = down;
	}


	input_mt_slot(input, touch->id);
	input_mt_report_slot_state(input, MT_TOOL_FINGER, down);

	if (down) {
		input_report_abs(input, ABS_MT_POSITION_X, get_unaligned_be16(&touch->xpos));
		input_report_abs(input, ABS_MT_POSITION_Y, get_unaligned_be16(&touch->ypos));
	}

	return 0;
}


static int ctp_process_touch(struct geely_data *ts)
{
	struct ctp_touch *touch = (struct ctp_touch *)(ts->buf + 1);
	int contact = touch->contact;
	int i;
	u8 extra_size = *(u8*)(touch + 1);

	for (i = 0; i < 2; i++)
	{
		if((touch->pts[i].xpos[0]==0xff) && (touch->pts[i].xpos[1]==0xff) && (touch->pts[i].ypos[0]==0xff) && (touch->pts[i].ypos[1]==0xff))
			return 0;
		(void) ctp_process_one_touch(ts, &touch->pts[i]);
	}

	if (extra_size) {
		if (!ctp_recv_ext_packet(ts, extra_size) && ts->buf[0] == CTP_SLAVE_POS_EXT) {
			struct ctp_touch_point* pts;

			for (pts = (struct ctp_touch_point *)(ts->buf + 1);
				(u8*) pts < ts->buf + extra_size - 3; pts++)
				(void) ctp_process_one_touch(ts, pts);
		}
	}

	/* make sure all pointers are up */
	if (ctp_mt_unreliable(ts) && !contact) {
		for (i = 0; i < MAX_SUPPORT_POINTS; i++) {
			if (ts->down[i]) {
				csd_printk_debug("force release finger %d\n", i + 1);
				ts->down[i] = false;
				input_mt_slot(ts->input, i + 1);
				input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, 0);
			}
		}
	}

	//input_mt_report_pointer_emulation(ts->input, true);
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);

	return 0;
}

static void csd_vol_key_report_data(void *buf,u8 func_id)
{
	int status = -1;
	int which_key = -1;
	int key_code = -1;
	int key_status = -1;
	int slide_val = -1;

	if(CTP_SLAVE_V_INFO == func_id){
        	struct ctp_v_info *vi = (struct ctp_v_info *)buf;
		which_key = vi->which_key;
        	status = vi->key_status;

		csd_printk_debug("***** sdssgl status is 0x%x,which_key is 0x%x\n",status,which_key);
		if(CSD_V_KEY_V_INC == which_key){
			key_code = 0x65;
		}else if(CSD_V_KEY_V_DESC == which_key){
			key_code = 0x66;
		}else{
			csd_printk_debug("sdssgl vol key is not supported.\n");
		}
	}else{
		struct ctp_slide_info *si = (struct ctp_slide_info *)buf;
		status = si->key_status;
		slide_val = si->x_pos;

		csd_printk_debug("sdssgl VOL Slide.\n");
	}

	if(status & 0x40){ /* Press */
		key_status = 1;	/* Down */
	}
	if(status & 0x20){	// release
		key_status = 0;	/* Up */
	}
	if(status & 0x10){	/* Move */
		key_status = 2;
	}

	if(CTP_SLAVE_V_INFO == func_id){
		ecarx_key_value_report(key_code,key_status);
	}else{
		//slide_val = buf[CSD_SLIDE_VALUE_LOC];
		ecarx_vol_slide_report_data(key_status,0,slide_val);	/* DOwn */
		csd_printk_debug("sdssgl slide val is 0x%x,key_status is 0x%x\n",slide_val,key_status);
	}
}

static irqreturn_t geely_i2c_irq(int irq, void *_dev)
{
	struct geely_data *ts = _dev;
	//struct i2c_client *client = ts->client;
	int ret;
	u8 op;

	csd_printk_error("sdssgl enter\n");

again:
	ret = ctp_recv_packet(ts);
	if (ret == -EAGAIN) {
		csd_printk_debug(" enter1\n");
		goto again;
	}
	else if (ret) {
		csd_printk_debug(" enter2\n");
		goto exit;
	}

	op = ts->buf[0];
	if (op < CTP_SLAVE_MIN || op > CTP_SLAVE_MAX) {
		csd_printk_error("sdssgl unknown opcode %02x\n",op);
		goto exit;
	}

	csd_printk_error("sdssgl opcode %02x\n",op);
	switch (op) {
		case CTP_SLAVE_SYNC:
		{
			int ret;
			struct ctp_init_status *cis = (struct ctp_init_status *)(ts->buf + 1);

			csd_printk_debug("ctp sync recv: ready = %d\n", cis->success);
			ret = ctp_send_packet(ts, ctp_master_sync, sizeof(ctp_master_sync));
			if (ret < 0) {
				csd_printk_error("ctp send syn failed %d\n", ret);
			}
		}
		break;
		case CTP_SLAVE_BIST:
		{
			struct ctp_bist_status *bs = (struct ctp_bist_status*)(ts->buf + 1);
			ctp_bs = *bs; 
			csd_printk_debug(" sdssgl ctp overall(%d),ctp display(%d),ctp touch(%d),ctp temperature(%d).\n",bs->overall,bs->display,bs->touch,bs->temperature);
		}
		break;
		case CTP_SLAVE_VOLTAGE:
		{
			struct ctp_voltage *vol = (struct ctp_voltage*)(ts->buf + 1);
			ctp_vol = *vol;
			csd_printk_debug(" sdssgl ctp voltage is %d.\n",vol->voltage_range);
			break;
		}
		default:
			break;
		case CTP_SLAVE_VER:
		{
			struct ctp_version *ver = (struct ctp_version *)(ts->buf + 1);

			ctp_ver = *ver;
			if (ver->hw_major == 5 && ver->hw_minor == 0 && ver->sw_major == 1 && ver->sw_minor == 9) {
				ts->quirk = QUIRK_DROP_UNUSED;
			}
			else if (ver->hw_major == 0 && ver->hw_minor == 3 && ver->sw_major == 0 && ver->sw_minor == 7) {
				ts->quirk = QUIRK_MT_UNRELIABLE | QUIRK_DUP_IRQ;
			}
			else {
				ts->quirk = QUIRK_MT_UNRELIABLE;
			}

			csd_printk_debug("sdssgl ctp version hw: %d.%d, sw: %d.%d quirk: %x\n",
				ver->hw_major, ver->hw_minor, ver->sw_major, ver->sw_minor, ts->quirk);

			/* hack hack hack */
			if (ctp_drop_unused(ts)) {
				ts->input->mt->flags |= INPUT_MT_DROP_UNUSED;
			}
			else {
				ts->input->mt->flags &= ~INPUT_MT_DROP_UNUSED;
			}

			if (ctp_mt_unreliable(ts))
				memset(ts->down, 0, sizeof(ts->down));

			break;
		}
	case CTP_SLAVE_POS:
		ret = ctp_process_touch(ts);
		break;
	case CTP_SLAVE_POS_EXT:
		csd_printk_error("state machine corrupt, drop unexpected ext packet\n");
		break;
	case CTP_SLAVE_KEY_BC:
	{
		struct ctp_key_bc *bc = (struct ctp_key_bc *)(ts->buf + 1);
		ctp_bc = *bc;
		csd_printk_error("sdssgl_ctp key_bc status is : %x\n",bc->key_backlight_control);
		break;

	}
	case CTP_SLAVE_KEY_BL:
	{
		struct ctp_key_bl *bl = (struct ctp_key_bl *)(ts->buf + 1);
		ctp_bl = *bl;
		csd_printk_error("sdssgl_ctp key_bl status is : %x\n",bl->key_backlight_level);
		break;

	}
	case CTP_SLAVE_KEY_BR:
	{
		struct ctp_key_br *br = (struct ctp_key_br *)(ts->buf + 1);
		ctp_br = *br;
		csd_printk_error("sdssgl_ctp key_br status is : %x\n",br->key_backlight_brightness);
		break;
	}
	case CTP_SLAVE_V_STATUS:
	{
		struct ctp_slide_info *si = (struct ctp_slide_info *)(ts->buf + 1);
		csd_printk_error("sdssgl_ctp No.touch is : %x,ski status is 0x%x\n",si->num_touch,si->key_status);
		csd_vol_key_report_data(si,CTP_SLAVE_V_STATUS);
		break;
	}
	case CTP_SLAVE_V_INFO:
	{
		struct ctp_v_info *vi = (struct ctp_v_info *)(ts->buf + 1);
		csd_printk_error("sdssgl_ctp which key is : %x,vki status is 0x%x\n",vi->which_key,vi->key_status);
		csd_vol_key_report_data(vi,CTP_SLAVE_V_INFO);
		break;
	}
	}
	ts->state = next_state[op];

exit:
	csd_printk_debug(" exit\n");

	return IRQ_HANDLED;
}

static ssize_t show_mmi_quest_disp(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct geely_data *ts = dev_get_drvdata(dev);
	mmi_quest_csd(ts);
	msleep(20);
	return sprintf(buf, "%d\n",ctp_bc.key_backlight_control);
}

static ssize_t store_mmi_quest_disp(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	struct geely_data *ts = dev_get_drvdata(dev);
	mmi_quest_csd(ts);
	msleep(20);
	return count;
}

static ssize_t show_csd_disp_sw_v(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct geely_data *ts = dev_get_drvdata(dev);
	mmi_quest_csd(ts);
	msleep(20);
	return sprintf(buf, "%d.%d\n",ctp_ver.sw_major,ctp_ver.sw_minor);
}

static ssize_t show_csd_disp_hw_v(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct geely_data *ts = dev_get_drvdata(dev);
	mmi_quest_csd(ts);
	msleep(20);
	return sprintf(buf, "%d.%d\n",ctp_ver.hw_major,ctp_ver.hw_minor);
}

static ssize_t show_csd_disp_total_func_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct geely_data *ts = dev_get_drvdata(dev); 
	mmi_quest_csd(ts);
	msleep(20);
	//return sprintf(buf, "%d\n",ctp_bs.overall);
	return sprintf(buf, "%s\n",(ctp_bs.overall == 0 ?"NORMAL":"ABNORMAL"));
}

static ssize_t show_csd_disp_func_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct geely_data *ts = dev_get_drvdata(dev);
	mmi_quest_csd(ts);
	msleep(20);
	//return sprintf(buf, "%d\n",ctp_bs.display);
	return sprintf(buf, "%s\n",(ctp_bs.display == 0 ?"NORMAL":"ABNORMAL"));
}

static ssize_t show_csd_disp_tp_func_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct geely_data *ts = dev_get_drvdata(dev);
	mmi_quest_csd(ts);
	msleep(20);
	//return sprintf(buf, "%d\n",ctp_bs.touch);
	return sprintf(buf, "%s\n",(ctp_bs.touch == 0 ?"NORMAL":"ABNORMAL"));
}

static ssize_t show_csd_disp_temp_func_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct geely_data *ts = dev_get_drvdata(dev);
	mmi_quest_csd(ts);
	msleep(20);
	//return sprintf(buf, "%d\n",ctp_bs.temperature);

	return sprintf(buf, "%s\n",(ctp_bs.temperature == 0 ?"NORMAL":"ABNORMAL"));
}

static ssize_t show_csd_disp_vol_range(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct geely_data *ts = dev_get_drvdata(dev);
	mmi_quest_csd(ts);
	msleep(20);
	//return sprintf(buf, "%d\n",ctp_vol.voltage_range);
	return sprintf(buf, "%s\n",(ctp_vol.voltage_range == 0 ?"NORMAL":(ctp_vol.voltage_range == 1? "!!!HIGH!!!":"!!!LOW!!!")));
}

static ssize_t show_csd_notify_disp_status_to_mcu(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct geely_data *ts = dev_get_drvdata(dev);
	mmi_quest_csd(ts);
	msleep(20);
	return 1;
}

static ssize_t show_csd_set_key_bl(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",ctp_bl.key_backlight_level);
}

static ssize_t store_csd_set_key_bl(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{

	unsigned long data;
	int err;

	struct geely_data *ts = dev_get_drvdata(dev);
	err = kstrtoul(buf, 10, &data);


	data = data > 8 ? 8: data;
	data = data < 0 ? 0: data;


	err = csd_set_key_bl(ts,data);
	if(err<0)
		csd_printk_error("sdssgl ,set failed.\n");

	msleep(20);

	return count;
	//return 0;
}

static ssize_t show_csd_set_key_bc(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",ctp_bc.key_backlight_control);
}

static ssize_t store_csd_set_key_bc(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct geely_data *ts = dev_get_drvdata(dev);
	err = kstrtoul(buf, 10, &data);

	csd_printk_debug("sdssgl ,the set data is 0x%lx.\n",data);
	data = !!data;

	if(data == 0x00){
		csd_set_key_bl(ts,0);
		msleep(20);
	}

	err = csd_set_key_bc(ts,data);
	msleep(20);
	if(err<0)
		;
	return count;
	//return 0;
}

static ssize_t show_csd_set_key_br(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",ctp_br.key_backlight_brightness);
}

static ssize_t store_csd_set_key_br(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct geely_data *ts = dev_get_drvdata(dev);
	err = kstrtoul(buf, 10, &data);
	//data = !!data;
	err = csd_set_key_br(ts,data);
	if(err<0)
		;
	return count;
	//return 0;
}

static ssize_t store_csd_debug_en(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}

static ssize_t show_csd_debug_en(struct device *dev,struct device_attribute *attr, char *buf)
{
	//return sprintf(buf, "%d\n",g_debug_en);
	return 0;
}

static DEVICE_ATTR(sdsstest, 0, NULL,NULL);
static DEVICE_ATTR(csd_notify_disp_status_to_mcu, S_IRUGO,show_csd_notify_disp_status_to_mcu,NULL);
static DEVICE_ATTR(csd_mmi_question_disp_status,S_IWUSR | S_IRUGO,show_mmi_quest_disp,store_mmi_quest_disp);
static DEVICE_ATTR(csd_disp_sw_v,S_IRUGO,show_csd_disp_sw_v,NULL);
static DEVICE_ATTR(csd_disp_hw_v,S_IRUGO,show_csd_disp_hw_v,NULL);
static DEVICE_ATTR(csd_disp_total_func_status, S_IRUGO,show_csd_disp_total_func_status,NULL);
static DEVICE_ATTR(csd_disp_func_status, S_IRUGO,show_csd_disp_func_status,NULL);
static DEVICE_ATTR(csd_disp_tp_func_status, S_IRUGO,show_csd_disp_tp_func_status,NULL);
static DEVICE_ATTR(csd_disp_temp_func_status, S_IRUGO,show_csd_disp_temp_func_status,NULL);
static DEVICE_ATTR(csd_disp_vol_range, S_IRUGO,show_csd_disp_vol_range,NULL);
static DEVICE_ATTR(csd_set_key_bl,S_IWUSR | S_IRUGO,show_csd_set_key_bl,store_csd_set_key_bl);
static DEVICE_ATTR(csd_set_key_bc,S_IWUSR | S_IRUGO,show_csd_set_key_bc,store_csd_set_key_bc);
static DEVICE_ATTR(csd_set_key_br,S_IWUSR | S_IRUGO,show_csd_set_key_br,store_csd_set_key_br);
static DEVICE_ATTR(csd_debug_en,S_IWUSR | S_IRUGO,show_csd_debug_en,store_csd_debug_en);

static struct device_attribute *csd_ts_attr_list[] = {
	&dev_attr_sdsstest,
#if 1
	&dev_attr_csd_mmi_question_disp_status,     /* request disp status info */
	&dev_attr_csd_disp_sw_v,     /* csd sw version */
	&dev_attr_csd_disp_hw_v,     /* csd hw version */
	&dev_attr_csd_disp_total_func_status,     /* csd total_func_status */
	&dev_attr_csd_disp_func_status,     /* csd disp_func_status */
	&dev_attr_csd_disp_tp_func_status,     /* csd tp_func_status */
	&dev_attr_csd_disp_temp_func_status,     /* csd temp_func_status */
	&dev_attr_csd_disp_vol_range,     /* csd vol_range */
	&dev_attr_csd_notify_disp_status_to_mcu,     /* csd notify disp status to mcu */
	&dev_attr_csd_set_key_bl,     /* csd set vol bl */
	&dev_attr_csd_set_key_bc,	 /* csd set key bl mode */
	&dev_attr_csd_set_key_br,	 /* csd set key bl br */
	&dev_attr_csd_debug_en,  /* csd debug en */
#endif
};

static int csd_ts_create_attr(struct device *dev)
{
	int idx, err = 0;
	int num = (int)(sizeof(csd_ts_attr_list)/sizeof(csd_ts_attr_list[0]));
	if (dev == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)
	{
		err = device_create_file(dev, csd_ts_attr_list[idx]);
		if(err) {
			csd_printk_debug("driver_create_file (%s) = %d\n", csd_ts_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}


static int csd_upgrade_open(struct inode *inode, struct file *file)
{
	struct geely_data *ts;
	ts = container_of(file->f_op, struct geely_data, fops);
	//csd_printk_debug("sdssgl ts->x_max is %d.\n",ts->x_max);
	file->private_data = ts;
	return 0;
}

static ssize_t csd_upgrade_write(struct file* filp, const char __user *buf, size_t count, loff_t* f_pos)
{
	u8 *tmp;
	struct geely_data *ts = filp->private_data;
	tmp = memdup_user(buf, count);
	//csd_printk_debug("sdssgl ts->y_max is %d.\n",ts->y_max);
	//ret = i2c_dma_write_tp(client,tmp[0],&tmp[1],count-1);
	ctp_send_packet_nocksum(ts,tmp,count);
	return count;
}

int csd_upgrade_release(struct inode * inp, struct file *filp)
{
	return 0;
}


static const struct file_operations csd_upgrade_fops = {
	.open = csd_upgrade_open,
	.write = csd_upgrade_write,
	.release = csd_upgrade_release,
};

struct ub983_data {
	struct i2c_client *client;

	struct miscdevice mdev;
	struct file_operations fops;

	int state;
};

//#define FE6_85M
static int ub983_init(struct geely_data *ts)
{
	int ret;
	int ub983_vdd1v8_en_gpio = 0;
	int ub983_vdd1v1_en_gpio = 0;
	int ub983_pdb = 0;
	int ub983_bl_en1 = 0;
	int ub983_bl_en2 = 0;
	int ub983_bl_pwm2 = 0;

	struct i2c_client *client = ts->client;
	struct device_node *np = client->dev.of_node;
	ub983_vdd1v8_en_gpio = of_get_named_gpio_flags(np, "ub983_vdd1v8_en_gpio", 0, NULL);
	if (!gpio_is_valid(ub983_vdd1v8_en_gpio))
	{
		dev_err(&client->dev, "%s: failed to get ub983_vdd1v8_en_gpio!\n", __func__);
		return -1;
	}
	ret = gpio_request(ub983_vdd1v8_en_gpio, "ub983_vdd1v8_en_gpio");
	if (ret < 0) {
		dev_err(NULL,"request gpio failed\n");
		return ret;
	}
	gpio_direction_output(ub983_vdd1v8_en_gpio, 1);

	ub983_vdd1v1_en_gpio = of_get_named_gpio_flags(np, "ub983_vdd1v1_en_gpio", 0, NULL);
	if (!gpio_is_valid(ub983_vdd1v1_en_gpio))
	{
		dev_err(&client->dev, "%s: failed to get ub983_vdd1v1_en_gpio!\n", __func__);
		return -1;
	}
	ret = gpio_request(ub983_vdd1v1_en_gpio, "ub983_vdd1v1_en_gpio");
	if (ret < 0) {
		dev_err(NULL,"request gpio failed\n");
		return ret;
	}
	gpio_direction_output(ub983_vdd1v1_en_gpio, 1);
	mdelay(1000);
	ub983_pdb = of_get_named_gpio_flags(np, "ub983_pdb", 0, NULL);
	if (!gpio_is_valid(ub983_pdb))
	{
		dev_err(&client->dev, "%s: failed to get ub983_pdb!\n", __func__);
		return -1;
	}
	ret = gpio_request(ub983_pdb, "ub983_pdb");
	if (ret < 0) {
		dev_err(NULL,"request gpio failed\n");
		return ret;
	}
	gpio_direction_output(ub983_pdb, 0);
	mdelay(500);
	gpio_direction_output(ub983_pdb, 1);

	mdelay(500);

	ub983_bl_en1 = of_get_named_gpio_flags(np, "ub983_bl_en1", 0, NULL);
	if (!gpio_is_valid(ub983_bl_en1))
	{
		dev_err(&client->dev, "%s: failed to get ub983_bl_en1!\n", __func__);
		return -1;
	}
	ret = gpio_request(ub983_bl_en1, "ub983_bl_en1");
	if (ret < 0) {
		dev_err(NULL,"request gpio failed\n");
		return ret;
	}
	gpio_direction_output(ub983_bl_en1, 1);
	ub983_bl_en2 = of_get_named_gpio_flags(np, "ub983_bl_en2", 0, NULL);
	if (!gpio_is_valid(ub983_bl_en2))
	{
		dev_err(&client->dev, "%s: failed to get ub983_bl_en2!\n", __func__);
		return -1;
	}
	ret = gpio_request(ub983_bl_en2, "ub983_bl_en2");
	if (ret < 0) {
		dev_err(NULL,"request gpio failed\n");
		return ret;
	}
	gpio_direction_output(ub983_bl_en2, 1);
/*
	ts->gpio_bl_pwm1 = of_get_named_gpio_flags(np, "ub983_bl_pwm1", 0, NULL);
	if (!gpio_is_valid(ts->gpio_bl_pwm1))
	{
		dev_err(&client->dev, "%s: failed to get ub983_bl_pwm1!\n", __func__);
		return -1;
	}
	ret = gpio_request(ts->gpio_bl_pwm1, "gpio_bl_pwm1");
	if (ret < 0) {
		dev_err(NULL,"request gpio failed\n");
		return ret;
	}*/
	//gpio_direction_output(ts->gpio_bl_pwm1, 1);

	ub983_bl_pwm2 = of_get_named_gpio_flags(np, "ub983_bl_pwm2", 0, NULL);
	if (!gpio_is_valid(ub983_bl_pwm2))
	{
		dev_err(&client->dev, "%s: failed to get ub983_bl_pwm2!\n", __func__);
		return -1;
	}
	ret = gpio_request(ub983_bl_pwm2, "ub983_bl_pwm2");
	if (ret < 0) {
		dev_err(NULL,"request gpio failed\n");
		return ret;
	}
	gpio_direction_output(ub983_bl_pwm2, 1);

	ub983_reg_init(ts->client);

	//back light pwm
	//ub983_i2c_write_register(ts->client,0x17,0x02);
	//ub983_i2c_write_register(ts->client,0x18,0x03);
	//ub983_i2c_write_register(ts->client,0xd,0x03);
	//ub983_i2c_write_register(ts->client,0x16,0x10);

	return 0;
}


static int geely_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct geely_data *ts;
	unsigned long irqflags;
	int error;
	int ts_irq_gpio = -1;
	int bl_en_gpios = 0;
	int string_count = 0;
	int i = 0;
	int index = 0;
	const char *name = NULL;
	const char *tmp= NULL;
	char ts_irq_gpio_name[128];
	u32 touch_screen_addr;
	csd_printk_debug("geely_i2c_probe start\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		csd_printk_error("%s: i2c check functionality error\n", DEVICE_NAME);
		return -ENXIO;
	}

	ts = devm_kzalloc(&client->dev, sizeof(struct geely_data), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->x_max = CTP_DEFAULT_X_MAX;
	ts->y_max = CTP_DEFAULT_Y_MAX;
	ts->state = CTP_SLAVE_SYNC;
	ts->client = client;
	i2c_set_clientdata(client, ts);

	ub983_init(ts);

	if (np)
	{
		error = of_property_read_u32(np, "touch_addr", &touch_screen_addr);
		if (error)
		{
			csd_printk_debug( "not find touch_addr\n");
			return -EINVAL;
		}
		client->addr = touch_screen_addr;

		error = of_property_read_string(np,"geely,string", &name);
		if (error)
		{
			csd_printk_debug( "not find geely,string\n");
		}

		if(strlen(name) >= sizeof(ts->input_name))
		{
			csd_printk_error("geely,string too long\n");
			return -EINVAL;
		}
		strncpy(ts->input_name, name, strlen(name));

		if(strncmp(name, "Touchscreen dim", min(strlen(name), 15ul)) && strncmp(name,  "Touchscreen havc", min(strlen(name), 16ul)))
		{	string_count = of_property_count_strings(np, "geely,addr");
				if(string_count < 4 || (string_count%4 !=0))
				{
					csd_printk_error("geely,addr error:%d\n", string_count);
					return -EINVAL;
				}
				csd_printk_debug("ts->input_name:%s, string_count:%d\n", ts->input_name, string_count);
		}

		for(i=0; (i<string_count/4) && strncmp(name, "Touchscreen dim", min(strlen(name), 15ul)) && strncmp(name,  "Touchscreen havc", min(strlen(name), 16ul)); i++)
		{
			index = i*4;
			error = of_property_read_string_index(np, "geely,addr", index, &ts->ser_name);
			if (error)
			{
				csd_printk_error("ser_name error:%d %d\n", index, error);
				return error;
			}

			error = of_property_read_string_index(np, "geely,addr", index+1, &tmp);
			if (error)
			{
				csd_printk_error("ser_addr error:%d %d\n", index+1, error);
				return error;
			}
			sscanf(tmp, "0x%x", &ts->ser_addr);
			if(ts->ser_addr == 0)
			{
				csd_printk_error("ser_addr error:%d, ser_addr must be lower case..\n", index+1);
				return -EINVAL;
			}

			if(i2c_device_is_alive(ts, ts->ser_addr) != 0 )
			{
				csd_printk_warning("can not find i2c ser_addr:0x%02x\n", ts->ser_addr);
				continue;
			}
			error = of_property_read_string_index(np, "geely,addr", index+2, &ts->deser_name);
			if (error)
			{
				csd_printk_error("deser_name error:%d %d\n", index+2, error);
				return error;
			}

			error = of_property_read_string_index(np, "geely,addr", index+3, &tmp);
			if (error)
			{
				csd_printk_error("deser_addr error:%d %d\n", index+3, error);
				return error;
			}
			sscanf(tmp, "0x%x", &ts->deser_addr);

			csd_printk_debug("find ser_name:%s, ser_addr %x, deser_name %s, deser_addr %x\n", ts->ser_name, ts->ser_addr, ts->deser_name, ts->deser_addr);
			break;
		}

		error = of_property_read_u32(np, "geely,x-max", &ts->x_max);
		if (error)
		{
			csd_printk_debug("using default x-max %d\n", ts->x_max);
		}

		error = of_property_read_u32(np, "geely,y-max", &ts->y_max);
		if (error)
		{
			csd_printk_debug( "using default y-max %d\n", ts->y_max);
		}
		memset(ts_irq_gpio_name, 0, sizeof(ts_irq_gpio_name));

		if(strncmp(name, "Touchscreen dim", min(strlen(name), 15ul)) == 0)
			sprintf(ts_irq_gpio_name, "dim-%s", "int-gpios");
		else  if(strncmp(name,  "Touchscreen havc", min(strlen(name), 16ul)) == 0)
			sprintf(ts_irq_gpio_name, "havc-%s", "int-gpios");
		else
			sprintf(ts_irq_gpio_name, "%s-%s", ts->ser_name, "int-gpios");

		ts_irq_gpio = of_get_named_gpio(np, (const char *)ts_irq_gpio_name, 0);
		if (!gpio_is_valid(ts_irq_gpio)) {
			csd_printk_error(" err to get gpios: ret:%x\n",ts_irq_gpio);
			ts_irq_gpio = -1;
			return -ENXIO;
		}
		else {
			csd_printk_debug( " int gpios:%d\n", ts_irq_gpio);
			gpio_request_one(ts_irq_gpio, GPIOF_IN, "ts_irq");
			gpio_direction_input(ts_irq_gpio);
			client->irq = gpio_to_irq(ts_irq_gpio);
		}
		csd_printk_debug( "geely_i2c_probe client->irq %d\n", client->irq);

		bl_en_gpios = of_get_named_gpio(np, "bl-en-gpios", 0);
		if(bl_en_gpios > 0)
		{
			gpio_set_value(bl_en_gpios, 1);
			csd_printk_debug( "find three screen bl power gpio : %d\n",bl_en_gpios);
		}
	}

	if(strncmp(name, "Touchscreen dim", min(strlen(name), 15ul)) && strncmp(name, "Touchscreen havc", min(strlen(name), 16ul)))
	{
		if(ts->ser_name == NULL || ts->deser_name == NULL)
		{
			csd_printk_error("ts->ser_name error:%d , ts->deser_name error: %d ..\n", ts->ser_name == NULL, ts->deser_name == NULL);
			return -EINVAL;
		}
	}

	if(ts->ser_name && 0 == strcmp(ts->ser_name, "MAX96789") && 0 == strcmp(ts->deser_name, "MAX96752"))
	{
		csd_printk_debug( "%s MAX96789_init start...\n",ts->input_name);
		MAX96789_init(ts);
	}
	else if(ts->ser_name && 0 == strcmp(ts->ser_name, "MAX96789"))
	{
		/* TO DO ...*/
	}

	ts->input = devm_input_allocate_device(&client->dev);
	if (!ts->input) {
		csd_printk_error("Failed to allocate input device\n");
		return -ENOMEM;
	}
	ts->input->name = ts->input_name;

	csd_printk_debug( "geely_i2c_probe input name : %s\n",ts->input->name);

	ts->input->id.bustype = BUS_I2C;

	__set_bit(BTN_TOUCH, ts->input->keybit);
	__set_bit(EV_ABS, ts->input->evbit);
	__set_bit(EV_KEY, ts->input->evbit);

	{
		input_set_abs_params(ts->input, ABS_X, 0, ts->x_max, 0, 0);
		input_set_abs_params(ts->input, ABS_Y, 0, ts->y_max, 0, 0);
		input_set_abs_params(ts->input, ABS_PRESSURE, 0, 255, 0, 0);

		input_mt_init_slots(ts->input, MAX_SUPPORT_POINTS, INPUT_MT_DIRECT);

		input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, ts->x_max, 0, 0);
		input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, ts->y_max, 0, 0);
	}

	csd_printk_debug("geely_i2c_probe ----line=%d---\n", __LINE__);

	input_set_drvdata(ts->input, ts);

	error = input_register_device(ts->input);
	if (error) {
		csd_printk_error("unable to register input device: %d\n", error);
		return error;
	}

	/*
	 * Systems using device tree should set up interrupt via DTS,
	 * the rest will use the default falling edge interrupts.
	 */
	irqflags = IRQF_TRIGGER_FALLING;
#ifdef CONFIG_MTK_DUAL_PRIMARY_SUPPORT
#if (CONFIG_MTK_DUAL_PRIMARY_SUPPORT == 2)
	if(ts->ser_name && 0 == strcmp(ts->ser_name, "TI941"))
		irqflags = IRQF_TRIGGER_RISING;
#endif
#endif

	error = devm_request_threaded_irq(&client->dev, client->irq,
					  NULL, geely_i2c_irq,
					  irqflags | IRQF_ONESHOT,
					  client->name, ts);
	if (error) {
		csd_printk_error("Failed to register interrupt\n");
		return error;
	}

	/*
	 * Systems using device tree should set up wakeup via DTS,
	 * the rest will configure device as wakeup source by default.
	 */
	if (!client->dev.of_node)
		device_init_wakeup(&client->dev, true);

	if(!strcmp(ts->input->name,"Geely Touchscreen"))
	{
	    ts->mdev.minor = MISC_DYNAMIC_MINOR;
	    ts->mdev.name = "csd_upgrade";

	    ts->fops.owner = THIS_MODULE;
	    ts->fops.write = csd_upgrade_write;
	    ts->fops.open = csd_upgrade_open;
	    ts->fops.release = csd_upgrade_release;

	    ts->mdev.fops = &ts->fops;

		error = misc_register(&ts->mdev);
		if(error<0){
		   csd_printk_error("register csd_upgrade_dev failed(%d)", error);
		}

		//dev_set_drvdata(csd_upgrade_dev.this_device,ts);
		dev_set_drvdata(ts->mdev.this_device,ts);
		/* Create attributes */
		//csd_ts_create_attr((csd_upgrade_dev.this_device));
		csd_ts_create_attr((ts->mdev.this_device));

		csd_printk_debug("csd update misc driver\n");
	}

#ifdef CONFIG_IPO_MANAGER
	if(ipo_manager_register_device(ts->input_name, &client->dev, IPO_MANAGER_DEVICE_TYPE_TOUCHPAD, true) < 0)
		csd_printk_error("ipo_manager_register_device geely_ts(%s) error\n", ts->input_name);
#endif

	if(!strncmp(ts->input->name, "Touchscreen havc", min(strlen(ts->input->name), strlen("Touchscreen havc"))))
	{
		g_sub1_ts = ts;
	}else if(!strncmp(ts->input->name, "Touchscreen dim", min(strlen(ts->input->name), strlen("Touchscreen dim")))){
		g_sub2_ts = ts;
	}

	csd_printk_debug( "geely_i2c_probe end client->irq %d\n", client->irq);
	return 0;
}

module_i2c_driver(geely_i2c_driver);

MODULE_AUTHOR("liam chen <liam.chen@siengine.com>");
MODULE_AUTHOR("Haijun Ma <haijunma@ecarx.com.cn>");
MODULE_DESCRIPTION("Geely CTP Touchscreen driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
