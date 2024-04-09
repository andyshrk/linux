
/*
 * SPDX-License-Identifier: GPL-2.0+
 * copyright (C) ecarx
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <drm/drm_print.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <linux/serial_8250.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/serial_dw8250.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/pm.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#define CONFIG_SERIAL_DEBUG 1
#ifdef CONFIG_SERIAL_DEBUG
	#define serial_debug(msg...)  printk(KERN_INFO msg)
#else
	#define serial_debug(msg...) do {}while(0)
#endif

unsigned char ub983_dsi_subdev_init_reg[][6] = {
    {0xc, 0x71, 0x90, 0x01, 0x05, 0x00}, //touchscreen i2c 8bit 0x90
    {0xc, 0x79, 0x90, 0x01, 0x05, 0x00},
    {0xc, 0x89, 0x00, 0x01, 0x05, 0x00},
};

static unsigned char ub984_dev_init_reg [][6] = {
//   addr,  reg, value, rbdelay, retry, delay(defore)
	{0x0c, 0x2d,  0x01,    0x01,  0x05, 0 },
	{0x0c, 0x15,  0x10,    0x01,  0x05, 0 },
	{0x0c, 0x0d,  0x03,    0x01,  0x05, 0 },

	{0x2c, 0x12,  0xdc,    0x01,  0x05, 0 }, //GPIO IN CLEAN
	{0x2c, 0x13,  0x00,    0x01,  0x05, 0 }, //GPIO IN CLEAN

	{0x2c, 0x15,  0xc0,    0x01,  0x05, 0 }, //vgh_en    GPIO0 | GPIO0_PIN_CTL Register (Address = 0x15)
	{0x2c, 0x16,  0xc0,    0x01,  0x05, 0 }, //vgl_en	GPIO1 | GPIO1_PIN_CTL Register (Address = 0x16)
	{0x2c, 0x17,  0xc0,    0x01,  0x05, 0 },
	{0x2c, 0x18,  0xc0,    0x01,  0x05, 0 },
	{0x2c, 0x19,  0xc0,    0x01,  0x05, 0 }, //tft_reset GPIO4 | GPIO4_PIN_CTL Register (Address = 0x19)
	{0x2c, 0x1a,  0xc0,    0x01,  0x05, 0 }, //tp_reset  GPIO5 | GPIO5_PIN_CTL Register (Address = 0x1A)
	{0x2c, 0x1b,  0xc0,    0x01,  0x05, 0 }, //tft_pon	GPIO6 | GPIO6_PIN_CTL Register (Address = 0x1B)
	{0x2c, 0x1c,  0xc0,    0x01,  0x05, 0 }, //bl_en     GPIO7 | GPIO7_PIN_CTL Register (Address = 0x1C)
	{0x2c, 0x1d,  0xc0,    0x01,  0x05, 0 },
	{0x2c, 0x1e,  0xc0,    0x01,  0x05, 0 },
	{0x2c, 0x22,  0xc0,    0x01,  0x05, 0 },

	{0x00, 0x00,  0x00,    0x00,  0x00, 30 }, //delay 20ms
	{0x2c, 0x17,  0xc1,    0x01,  0x05, 0x0a }, //vsp_en    GPIO2 | GPIO2_PIN_CTL Register (Address = 0x17)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x22,  0xc1,    0x01,  0x05, 0x00 }, //tcon_resetGPIO13 | GPIO13_PIN_CTL Register (Address = 0x22)
	{0x2c, 0x18,  0xc1,    0x01,  0x05, 0x00 }, //vsn_en	GPIO3 | GPIO3_PIN_CTL Register (Address = 0x18)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x15,  0xc1,    0x01,  0x05, 0x00 }, //vgh_en    GPIO0 | GPIO0_PIN_CTL Register (Address = 0x15)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x16,  0xc1,    0x01,  0x05, 0x00 }, //vgl_en	GPIO1 | GPIO1_PIN_CTL Register (Address = 0x16)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x19,  0xc1,    0x01,  0x05, 0x00 }, //tft_reset GPIO4 | GPIO4_PIN_CTL Register (Address = 0x1

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x1a,  0xc1,    0x01,  0x05, 0x00 }, //tp_reset	GPIO5 | GPIO5_PIN_CTL Register (Address = 0x1A)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x96 }, //delay 150ms
	{0x2c, 0x1b,  0xc1,    0x01,  0x05, 0x00 }, //tft_pon	GPIO6 | GPIO6_PIN_CTL Register (Address = 0x1B)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x1c,  0xc1,    0x01,  0x05, 0x00 }, //bl_en 	GPIO7 | GPIO7_PIN_CTL Register (Address = 0x1C)

	//disable pwm when init
	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x1d,  0xc0,    0x00,  0x05, 0x00 }, //pwm 	GPIO7 | GPIO7_PIN_CTL Register (Address = 0x1C)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x1e,  0xc0,    0x01,  0x05, 0x00 }, //bist_en 	GPIO7 | GPIO7_PIN_CTL Register (Address = 0x1C)

	{0x2c, 0x1f,  0x00,    0x01,  0x05, 0x00 }, //TFT_FAULT_DET_M GPIO10 | GPIO10_PIN_CTL Register (Address = 0x1F) - disable
	{0x2c, 0x20,  0x00,    0x01,  0x05, 0x00 }, //BL_FAULT 	GPIO11 | GPIO11_PIN_CTL Register (Address = 0x20) - disable
	{0x2c, 0x21,  0x00,    0x01,  0x05, 0x00 }, //ADC_READY GPIO12 | GPIO12_PIN_CTL Register (Address = 0x21) - disable

	{0x2c, 0x12,  0x1c,    0x01,  0x05, 0x00 }, //GPIO_IN_EN1 	GPIO8-13 | GPIO_IN_EN1 Register (Address = 0x12)
	//read state from 0x11 GPIO8-13


	{0x2c, 0x44,  0x81,    0x01,  0x05, 0x14 }, //RX_INT_CTL Register

	{0x0c, 0xc6,  0x21,    0x01,  0x05, 0x00 },
	{0x0c, 0x1b,  0x88,    0x01,  0x05, 0x00 },
	{0x0c, 0x51,  0x83,    0x01,  0x05, 0x00 },

	{0x0c, 0x0d,  0x03,    0x01,  0x05, 0x00 }, //Forward Channel GPIO Enable: 0x03 is 4Num
	{0x0c, 0x15,  0x01,    0x01,  0x05, 0x00 }, //Set GPIO1 as FC GPIO0 input && GPIO13 as FC GPIO1 input
	{0x0c, 0x3e,  0xff,    0x01,  0x05, 0x00 }, //Enable GPIO1(all) input

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x1E }, //delay 30ms
	{0x2c, 0x13,  0xfd,    0x01,  0x05, 0x00 }, //Disable GPIO1 input
	//{0x2c, 0x12,  0xdf,    0x01,  0x05, 0x00 }, //Disable GPIO13 input
	{0x2c, 0x1d,  0x94,    0x01,  0x05, 0x00 }, //Enable output of FC GPIO0 on GPIO8 | i2cset -y 12 0x2c 0x1d 0x94 b //tmp

	//{0x2c, 0x1a,  0x95,    0x01,  0x05, 0x00 }, //Enable output of FC GPIO13 on GPIO5
	{0x0c, 0x20,  0xc2,    0x01,  0x05, 0x00 }, //Enable output of BC GPIO9(s) from GPIO10(d)
	{0x0c, 0x28,  0x02,    0x01,  0x05, 0x00 }, //Enable output of BC 16
	{0x2c, 0x1f,  0x1f,	   0x01,  0x05, 0x00 }, //Enable input of GPIO10(d)
	//{0x0c, 0x18,  0x03,    0x01,  0x05, 0x00 },
	//{0x0c, 0x16,  0x10,    0x01,  0x05, 0x00 }, //Set GPIO1
};


struct ds90ubxxx_i2c {
	int m_gpio111_en ;

	u32 ser_addr;
	u32 desdis_addr;
	const char *serializer_type;
	struct i2c_client *i2c_client;
};

static s32 ds90ubxxx_write_reg(struct i2c_client *i2c_client, u8 i2c_addr, u8 reg, u8 val)
{
	u8 au8Buf[2] = {0};
	int i=0;

	au8Buf[0] = reg ;
	au8Buf[1] = val;

	i2c_client->addr = i2c_addr;


	for (i=0;i < 5; i++) {
		if (!(i2c_master_send(i2c_client, au8Buf, 2) < 0)) {
			break;
		}
		usleep_range(10*1000, 10*1000);
	}
	if (i == 5) {
		pr_err("%s:write reg error: i2c_addr:%x,reg=%x,val=%x, i=%d\n",__func__,i2c_client->addr, reg, val,i);
		return -1;
	}

	return 0;
}

static u8 ds90ubxxx_read_reg(struct i2c_client *i2c_client, u8 i2c_addr, u8 reg)
{
	u8 au8RegBuf = reg;
	u8 u8RdVal = 0;
	int i =0;

	i2c_client->addr = i2c_addr;

	for (i=0; i< 3; i++) {
		if (1 != i2c_master_send(i2c_client, &au8RegBuf, 1)) {
			usleep_range(1*1000, 1*1000);
			continue;
		}

		if (1 != i2c_master_recv(i2c_client, &u8RdVal, 1)) {
			usleep_range(1*1000, 1*1000);
			continue;
		}
		break;
	}

	if (i==3) {
		pr_err("%s:read reg error:addr:%x,reg=%x,val=%x\n",__func__, i2c_client->addr, reg, u8RdVal);
		return -1;
	}

	return u8RdVal;
}

inline void ub98x_udelay(u32 delay_rang)
{
	usleep_range(delay_rang*1000, delay_rang*1000);
}

// Serializer: DS90Ux981-Q1
// User Inputs:
// Serializer I2C Address= 0x18
// Port 0 DSI Lanes = 4
// Port 0 DSI Rate = 1200 Mbps/lane
// FPD-Link Configuration: FPD-Link IV Independent - 3.375Gbps Port 0, 3.375Gbps Port 1


// Number of Displays = 2

// Video Processor 0 Properties:
// Total Horizontal Pixels = 2088
// Total Vertical Lines = 763
// Active Horizontal Pixels = 1920
// Active Vertical Lines = 720
// Horizontal Back Porch = 32
// Vertical Back Porch = 24
// Horizontal Sync = 24
// Vertical Sync = 3
// Horizontal Front Porch = 112
// Vertical Front Porch = 16
// Horizontal Sync Polarity = Positive
// Vertical Sync Polarity = Positive
// Bits per pixel = 24
// Pixel Clock = 95.6MHz
// Cropping Enabled: 
// X Start = 0
// X Stop = 1919
// Y Start = 0
// Y Stop = 719
// PATGEN Disabled

// Video Processor 1 Properties:
// Total Horizontal Pixels = 2088
// Total Vertical Lines = 763
// Active Horizontal Pixels = 1920
// Active Vertical Lines = 720
// Horizontal Back Porch = 32
// Vertical Back Porch = 24
// Horizontal Sync = 24
// Vertical Sync = 3
// Horizontal Front Porch = 112
// Vertical Front Porch = 16
// Horizontal Sync Polarity = Positive
// Vertical Sync Polarity = Positive
// Bits per pixel = 24
// Pixel Clock = 95.6MHz
// Cropping Enabled:
// X Start = 1920
// X Stop = 3839
// Y Start = 0
// Y Stop = 719
// PATGEN Disabled

// Deserializer 0: DS90Ux988-Q1
// Note: The 988 should be strapped to 3.375Gbps FPD IV mode using MODE_SEL0
// User Inputs:
// Deserializer I2C Address = 0x70
// Deserializer I2C Alias = 0x70
// 988 OLDI Output Mode
// Dual OLDI Mode
// OLDI Port 0 Bpp = 24
// OLDI Port 1 Bpp = 24
// OLDI Port 0 MAPSEL = H (MSB on D3/D4)
// OLDI Port 1 MAPSEL = H (MSB on D3/D4)
// Dual OLDI Video Source = Serializer Stream 0
// OLDI PATGEN Disabled

// Deserializer 1: DS90Ux988-Q1
// Note: The 988 should be strapped to 3.375Gbps FPD IV mode using MODE_SEL0
// User Inputs:
// Deserializer I2C Address = 0x70
// Deserializer I2C Alias = 0x72
// 988 OLDI Output Mode
// Dual OLDI Mode
// OLDI Port 0 Bpp = 24
// OLDI Port 1 Bpp = 24
// OLDI Port 0 MAPSEL = H (MSB on D3/D4)
// OLDI Port 1 MAPSEL = H (MSB on D3/D4)
// Dual OLDI Video Source = Serializer Stream 1
// OLDI PATGEN Disabled
static int dx11_dis_sdpic_config(struct ds90ubxxx_i2c *ds90ubxxx_i2c)
{
	u8 Reg_value =0;
	struct i2c_client *i2c_client = ds90ubxxx_i2c->i2c_client;
	u8 serAddr = ds90ubxxx_i2c->ser_addr;
	u8 desAlias0 = ds90ubxxx_i2c->desdis_addr;
	u8 desAlias1 = desAlias0 + 0x1; //the keypoint to distinguish between two des same i2c id.
	ds90ubxxx_write_reg(i2c_client,serAddr,0x70,desAlias0 << 1);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x78,desAlias0 << 1);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x88,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x71,desAlias0 << 1);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x79,((desAlias1 << 1) + 1));
	ds90ubxxx_write_reg(i2c_client,serAddr,0x89,0x0);

	//// *********************************************
	//// Program DSI Configs
	//// *********************************************

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value | 0x8;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value); //Disable DSI

	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1); //Select port 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x10); //Change indirect page to page 4
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x20); //Port 0 TSKIP value:16 

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x4f);
	Reg_value = Reg_value & 0x73;
	Reg_value = Reg_value | 0x8c;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x4f,Reg_value); //Set number of lanes and continuous or non-continuous

	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x3); //Select write port 0 and 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0xbd,0x0); //Set DSI source for the Video processors 0 and 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0xbe,0x0); //Set DSI source for the Video processors 2 and 3
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1); //Select port 0


	//// *********************************************
	//// Program SER to FPD-Link IV mode
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x5b,0x23); //Disable FPD3 FIFO pass through
	ds90ubxxx_write_reg(i2c_client,serAddr,0x5,0x3c); //Force FPD4_TX independent mode

	/*******reverse DSI P/N,only adapt for display board in evb*********************/
	ds90ubxxx_write_reg(i2c_client, serAddr,0x40,0x14);
	ds90ubxxx_write_reg(i2c_client, serAddr,0x41,0x16);
	ds90ubxxx_write_reg(i2c_client, serAddr,0x42,0x1f);
	//// *********************************************
	//// Set up FPD IV PLL Settings - This section can be commented out to improve bringup time if 983/981 MODE_SEL0 and MODE_SEL2 are strapped to the correct FPD IV speed
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x8); //Select PLL reg page
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x1b);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x8); //Disable PLL0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5b);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x8); //Disable PLL1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,0xd1); //Enable mode overwrite
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x84);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x2); //Switch encoder from ADAS to IVI on port 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x94);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x2); //Switch encoder from ADAS to IVI on port 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x8); //Select PLL page
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5); //Select Ncount Reg
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x7d); //Set Ncount
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x13); //Select post div reg
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x90); //Set post div for 3.375Gbps
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1); //Select write reg to port 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x6a,0xa); //set BC sampling rate
	ds90ubxxx_write_reg(i2c_client,serAddr,0x6e,0x80); //set BC fractional sampling
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x4); //Select FPD page and set BC settings for FPD IV port 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x6);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0xd);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x34);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0xe);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x53);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x8); //Select PLL page
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x45); //Select Ncount Reg
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x7d); //Set Ncount
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x53); //Select post div reg
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x90); //Set post div for 3.375Gbps
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x12); //Select write reg to port 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x6a,0xa); //set BC sampling rate
	ds90ubxxx_write_reg(i2c_client,serAddr,0x6e,0x80); //set BC fractional sampling
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x4); //Select FPD page and set BC settings for FPD IV port 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x26);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x2d);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x34);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x2e);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x53);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,0x11); //Set HALFRATE_MODE
	//// *********************************************
	//// Zero out PLL fractional - This section can be commented out to improve bringup time if 983/981 MODE_SEL0 and MODE_SEL2 are strapped to the correct FPD IV speed
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x8); //Select PLL page
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x4);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x1);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x1e);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x1f);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x20);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x44);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x1);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5e);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5f);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x60);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	//// *********************************************
	//// Configure and Enable PLLs - This section can be commented out to improve bringup time if 983/981 MODE_SEL0 and MODE_SEL2 are strapped to the correct FPD IV speed
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0xe); //Select VCO reg
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xc7); //Set VCO
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x4e); //Select VCO reg
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xc7); //Set VCO
	ds90ubxxx_write_reg(i2c_client,serAddr,0x1,0x30); //soft reset PLL
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x8); //Select PLL page
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x1b);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Enable PLL0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5b);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Enable PLL1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x1,0x1); //soft reset Ser
	msleep(40);
	//// *********************************************
	//// Enable I2C Passthrough
	//// *********************************************
	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x7);
	Reg_value = Reg_value | 0x08;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x07,Reg_value); //Enable I2C Passthrough

	ds90ubxxx_write_reg(i2c_client,desAlias0,0x1,0x1); //Soft reset Des
	msleep(40);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1); //Select write to port0 reg
	//// *********************************************
	//// Program VP Configs
	//// *********************************************
	// Configure VP 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x32);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x2);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x80); //VID H Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x7); //VID H Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x8);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Crop Start X
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Crop Start X
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Crop Start Y
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Crop Start Y
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x7f); //Crop Stop X
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x7); //Crop Stop X
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xcf); //Crop Stop Y
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x2); //Crop Stop Y
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x10);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x80); //Horizontal Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x7); //Horizontal Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x20); //Horizontal Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Horizontal Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x18); //Horizontal Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Horizontal Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x28); //Horizontal Total
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x8); //Horizontal Total
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xd0); //Vertical Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x2); //Vertical Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x18); //Vertical Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Vertical Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3); //Vertical Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Vertical Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x10); //Vertical Front Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Vertical Front Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x27);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //HSYNC Polarity = +, VSYNC Polarity = +
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x4); //Enable Cropping
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x23); //M/N Register
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x42); //M value
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x24); //M value
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xf); //N value


	// Configure VP 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x32);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x42);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x80); //VID H Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x7); //VID H Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x48);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x80); //Crop Start X
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x7); //Crop Start X
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Crop Start Y
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Crop Start Y
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xff); //Crop Stop X
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xe); //Crop Stop X
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xcf); //Crop Stop Y
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x2); //Crop Stop Y

	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x50);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x80); //Horizontal Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x7); //Horizontal Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x20); //Horizontal Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Horizontal Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x18); //Horizontal Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Horizontal Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x28); //Horizontal Total
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x8); //Horizontal Total
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xd0); //Vertical Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x2); //Vertical Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x18); //Vertical Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Vertical Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3); //Vertical Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Vertical Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x10); //Vertical Front Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Vertical Front Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x67);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //HSYNC Polarity = +, VSYNC Polarity = +
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x40);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x4); //Enable Cropping
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x63); //M/N Register
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x42); //M value
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x24); //M value
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xf); //N value


	//// *********************************************
	//// Enable VPs
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x43,0x1); //Set number of VPs used = 2
	ds90ubxxx_write_reg(i2c_client,serAddr,0x44,0x3); //Enable video processors

	//// *********************************************
	//// Configure Serializer TX Link Layer
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x2e); //Link layer Reg page
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x1); //Select LINK0_STREAM_EN
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x1); //Enable Link Layer 0 Streams
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x6); //Select LINK0_SLOT_REQ0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x41); //Set number of time slots
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x11); //Select LINK1_STREAM_EN
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x2); //Enable Link Layer 1 Streams
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x17); //Select LINK1_SLOT_REQ1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x41); //Set number of time slots
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x12); //Select LINK1_MAP_REG0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x10); //Assign link layer stream 1 map
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x13); //Select LINK1_MAP_REG1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x32); //Assign link layer stream 2/3 map
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x20); //Set Link layer vp bpp
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x55); //Set Link layer vp bpp according to VP Bit per pixel
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x0); //Link layer enable
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xf); //Link layer enable


	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value & 0xf7;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value); //Enable DSI

	//// *********************************************
	//// Clear CRC errors from initial link process
	//// *********************************************

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value | 0x20;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value); //CRC Error Reset

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value & 0xdf;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value); //CRC Error Reset Clear

	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1);

	//// *********************************************
	//// Hold Des DTG in reset
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x40,0x50); //Select DTG Page
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x32);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x6); //Hold Port 0 DTG in reset
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x62);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x6); //Hold Port 1 DTG in reset


	//// *********************************************
	//// Disable Stream Mapping
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xe,0x3); //Select both Output Ports
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xd0,0x0); //Disable FPD4 video forward to Output Port
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xd7,0x0); //Disable FPD3 video forward to Output Port


	//// *********************************************
	//// Setup DTG for port 0
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x40,0x50); //Select DTG Page
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x20);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x53); //Set up DTG BPP, Sync Polarities, and Measurement Type
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x29); //Set Hstart
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x80); //Hstart upper byte
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x2a);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x38); //Hstart lower byte
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x2f); //Set HSW
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x40); //HSW upper byte
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x30);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x18); //HSW lower byte


	//// *********************************************
	//// Map video to display output
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xe,0x3); //Select both Output Ports
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xd0,0xc); //Enable FPD_RX video forward to Output Port
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xd1,0xf); //Every stream forwarded on DC
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xd6,0x0); //Send Stream 0 to Output Port 0 and Send Stream 0 to Output Port 1
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xd7,0x0); //FPD3 mapping disabled
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xe,0x1); //Select Port 0


	//// *********************************************
	//// Configure 988 Display
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x40,0x2c); //Configure OLDI/RGB Port Settings
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x2f);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x2f);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x40,0x2e); //Configure OLDI/RGB PLL
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x8);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x92); //PLL_NUM23_16
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x3); //PLL_NUM15_8
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x97); //PLL_NUM7_0
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0xff); //PLL_DEN23_16
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0xff); //PLL_DEN15_8
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0xa5); //PLL_DEN7_0
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x18);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x31); //PLL_NDIV
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x2d);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x12); //TX_SEL_CLKDIV


	//// *********************************************
	//// Release Des DTG reset
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x40,0x50); //Select DTG Page
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x32);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x4); //Release Port 0 DTG
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x62);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x4); //Release Port 1 DTG

	//// *********************************************
	//// Enable OLDI Output
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x1,0x40); //OLDI Reset
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x40,0x2c); //Enable OLDI/RGB
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x2);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x14);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x2); //Toggle OLDI_SER_EN for Dual OLDI Mode
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x4);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x14);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x20); //P0 TX_EN
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x80);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x22); //P1 TX_EN
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x80);

	//// *********************************************
	//// Hold Des DTG in reset
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x40,0x50); //Select DTG Page
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x41,0x32);
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x6); //Hold Port 0 DTG in reset
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x41,0x62);
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x6); //Hold Port 1 DTG in reset

	//// *********************************************
	//// Disable Stream Mapping
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias1,0xe,0x3); //Select both Output Ports
	ds90ubxxx_write_reg(i2c_client,desAlias1,0xd0,0x0); //Disable FPD4 video forward to Output Port
	ds90ubxxx_write_reg(i2c_client,desAlias1,0xd7,0x0); //Disable FPD3 video forward to Output Port

	//// *********************************************
	//// Setup DTG for port 0
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x40,0x50); //Select DTG Page
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x41,0x20);
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x53); //Set up DTG BPP, Sync Polarities, and Measurement Type
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x41,0x29); //Set Hstart
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x80); //Hstart upper byte
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x41,0x2a);
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x38); //Hstart lower byte
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x41,0x2f); //Set HSW
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x40); //HSW upper byte
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x41,0x30);
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x18); //HSW lower byte

	//// *********************************************
	//// Map video to display output
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias1,0xe,0x3); //Select both Output Ports
	ds90ubxxx_write_reg(i2c_client,desAlias1,0xd0,0xc); //Enable FPD_RX video forward to Output Port
	ds90ubxxx_write_reg(i2c_client,desAlias1,0xd1,0xf); //Every stream forwarded on DC
	ds90ubxxx_write_reg(i2c_client,desAlias1,0xd6,0x9); //Send Stream 1 to Output Port 0 and Send Stream 1 to Output Port 1
	ds90ubxxx_write_reg(i2c_client,desAlias1,0xd7,0x0); //FPD3 mapping disabled
	ds90ubxxx_write_reg(i2c_client,desAlias1,0xe,0x1); //Select Port 0

	//// *********************************************
	//// Configure 988 Display
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x40,0x2c); //Configure OLDI/RGB Port Settings
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x41,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x2f);
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x41,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x2f);
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x40,0x2e); //Configure OLDI/RGB PLL
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x41,0x8);
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x92); //PLL_NUM23_16
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x3); //PLL_NUM15_8
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x97); //PLL_NUM7_0
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0xff); //PLL_DEN23_16
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0xff); //PLL_DEN15_8
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0xa5); //PLL_DEN7_0
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x41,0x18);
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x31); //PLL_NDIV
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x41,0x2d);
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x12); //TX_SEL_CLKDIV

	//// *********************************************
	//// Release Des DTG reset
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x40,0x50); //Select DTG Page
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x41,0x32);
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x4); //Release Port 0 DTG
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x41,0x62);
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x4); //Release Port 1 DTG

	//// *********************************************
	//// Enable OLDI Output
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x1,0x40); //OLDI Reset
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x40,0x2c); //Enable OLDI/RGB
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x41,0x2);
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x14);
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x41,0x2); //Toggle OLDI_SER_EN for Dual OLDI Mode
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x4);
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x14);
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x41,0x20); //P0 TX_EN
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x80);
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x41,0x22); //P1 TX_EN
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x42,0x80);

	//UH988 GPIO13 configs as output and pull high to lightup LCD to adapt new devices.
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x12,0xdf);//GPIO13 input disable
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x22,0xc1);//GPIO13 output enable,pull high.

	ds90ubxxx_write_reg(i2c_client,desAlias1,0x12,0xdf);//GPIO13 input disable
	ds90ubxxx_write_reg(i2c_client,desAlias1,0x22,0xc1);//GPIO13 output enable,pull high.


	return 0;
}


/*support superframe dual rsd in dx11*/
static int dx11_dual_rsd_config(struct ds90ubxxx_i2c *ds90ubxxx_i2c)
{
	u8 Reg_value =0;
	struct i2c_client *i2c_client = ds90ubxxx_i2c->i2c_client;

	u8 serAddr = ds90ubxxx_i2c->ser_addr;

	//u8 desAlias0 = ds90ubxxx_i2c->desdis_addr;
	u8 desAddr0 = 0x58;
	u8 desAlias0 = 0x58;
	u8 desAlias1 = desAlias0 + 2;
	ds90ubxxx_write_reg(i2c_client, serAddr,0x70,desAddr0);
	ds90ubxxx_write_reg(i2c_client, serAddr,0x78,desAlias0);

	ds90ubxxx_write_reg(i2c_client, serAddr,0x88,0x0);
	ds90ubxxx_write_reg(i2c_client, serAddr,0x71,desAddr0);
	ds90ubxxx_write_reg(i2c_client, serAddr,0x79,desAlias1 + 1);
	ds90ubxxx_write_reg(i2c_client, serAddr,0x89,0x0);

	// *********************************************
	// Program DSI Configs
	// *********************************************
	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value | 0x8;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value);//Disable DSI

	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1); //Select port 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x10); //Change indirect page to page 4
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x14); //Port 0 TSKIP value:3

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x4f);
	Reg_value = Reg_value & 0x73;
	Reg_value = Reg_value | 0x8c;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x4f,Reg_value); //Set number of lanes and continuous or non-continuous

	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1); //Select port 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x10); //Change indirect page to page 4
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x14); //Port 0 TSKIP value:3

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x4f);
	Reg_value = Reg_value & 0x73;
	Reg_value = Reg_value | 0x8c;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x4f,Reg_value); //Set number of lanes and continuous or non-continuous

	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x3); //Select write port 0 and 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0xbd,0x0); //Set DSI source for the Video processors 0 and 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0xbe,0x0); //Set DSI source for the Video processors 2 and 3
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1); //Select port 0

	// *********************************************
	// Set FPD Port Configuration
	// *********************************************
	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x7);

	Reg_value = Reg_value & 0xFC;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x07,Reg_value); //Set IVI Mode
	Reg_value= ds90ubxxx_read_reg(i2c_client,serAddr,0x5);

	Reg_value = Reg_value & 0xC3;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x05,0x14); //Set FPD III Mode
	ds90ubxxx_write_reg(i2c_client,serAddr,0x59,0x5); //Set FPD3_TX_MODE to FPD III Single Port 0

	ds90ubxxx_write_reg(i2c_client, serAddr,0x40,0x14);
	ds90ubxxx_write_reg(i2c_client, serAddr,0x41,0x16);
	ds90ubxxx_write_reg(i2c_client, serAddr,0x42,0x1f);

	// *********************************************
	// Program PLLs
	// *********************************************
	//# Program PLL for Port 0: FPD III Mode 2333.31Mbps
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x8);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x4);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x9);//#Set fractional mash order
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x13);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xe0);//#Set VCO Post Div = 4, VCO Auto Sel for CS2.0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0xa); //#Set auto increment
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x56);//Set Ndiv = 86
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //#Set Ndiv = 86
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x18);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xc4);//#Set denominator = 16776900
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xfe);//#Set denominator = 16776900
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xff);//#Set denominator = 16776900
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x1e);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xc9);//#Set numerator = 7027657
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3b);//#Set numerator = 7027657
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x6b);// #Set numerator = 7027657

	//# Program PLL for Port 1: FPD III Mode 2333.31Mbps
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x8);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x44);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x9); //#Set fractional mash order
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x53);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xe0); //#Set VCO Post Div = 4, VCO Auto Sel for CS2.0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0xa); //#Set auto increment
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x45);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x56); //#Set Ndiv = 86
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //#Set Ndiv = 86
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x58);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xc4); //#Set denominator = 16776900
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xfe); //#Set denominator = 16776900
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xff); //Set denominator = 16776900
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5e);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xc9); //#Set numerator = 7027657
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3b); //#Set numerator = 7027657
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x6b); //#Set numerator = 7027657

	//ds90ubxxx_write_reg(i2c_client,serAddr,0x1,0x30); //PLL Reset

	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x4);//Set FPD Page to configure BC Settings for Port 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x6);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xff);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0xd);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x70);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0xe);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x70);

	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x26);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xff);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x2d);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x70);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x2e);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x70);

	ds90ubxxx_write_reg(i2c_client,serAddr,0x1,0x30);//Reset PLLs
	msleep(10);

	ds90ubxxx_write_reg(i2c_client,serAddr,0x7,0x88);
	// *********************************************
	// Program VP Configs
	// *********************************************
	// Configure VP 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x32);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x2);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//VID H Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x5);//VID H Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x8);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//#Crop Start X
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//#Crop Start X
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//#Crop Start Y
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//#Crop Start Y
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xff);// #Crop Stop X
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x4);//#Crop Stop X
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xff);//#Crop Stop Y
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x2);//#Crop Stop Y
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x10);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//Horizontal Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x5);//Horizontal Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x8);//Horizontal Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//Horizontal Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x8);//Horizontal Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//Horizontal Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x96);//Horizontal Total
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x5);//Horizontal Total
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//Vertical Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3);//Vertical Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3);//Vertical Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//Vertical Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3);//Vertical Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//Vertical Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3);//Vertical Front Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//Vertical Front Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x27);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//HSYNC Polarity = +, VSYNC Polarity = +
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x4); //#Enable Cropping

	//# Configure VP 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x32);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x42);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0) ;// #VID H Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x5) ;//#VID H Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x48);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//#Crop Start X
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x5);//#Crop Start X
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//#Crop Start Y
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//#Crop Start Y
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xff);//#Crop Stop X
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x9);//#Crop Stop X
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xff);//#Crop Stop Y
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x2);//#Crop Stop Y
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x50);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0) ;//#Horizontal Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x5) ;//#Horizontal Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x8) ;//#Horizontal Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0) ;//#Horizontal Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x8) ;//#Horizontal Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0) ;//#Horizontal Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x96);//#Horizontal Total
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x5) ;//#Horizontal Total
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0) ;//#Vertical Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3) ;//#Vertical Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3) ;//#Vertical Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0) ;//#Vertical Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3) ;//#Vertical Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0) ;//#Vertical Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3) ;//#Vertical Front Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0) ;//#Vertical Front Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x67);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0) ;//#HSYNC Polarity = +, VSYNC Polarity = +
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x40);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x4);// #Enable Cropping

	// *********************************************
	// Enable VPs
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x43,0x1);//Set number of VPs used = 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x44,0x3);//Enable video processors

	// *********************************************
	// Set FPD3 Stream Mapping
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1);//Select FPD TX Port 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x57,0x0);//Set FPD TX Port 1 Stream Source = VP0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x12);//Select FPD TX Port 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x57,0x1);//Set FPD TX Port 1 Stream Source = VP1

	ds90ubxxx_write_reg(i2c_client,serAddr,0x5b,0x2b); //Enable FPD III FIFO

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value & 0xf7;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value); //Enable DSI

	//*********************************************
	//Clear CRC errors from initial link process
	//*********************************************

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value | 0x20;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value); //CRC Error Reset

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value & 0xdf;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value); //CRC Error Reset Clear

	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1);

	msleep(1000);
	ds90ubxxx_write_reg(i2c_client,0x2c,0x21,0x09);
	ds90ubxxx_write_reg(i2c_client,0x2c,0x20,0x09);
	ds90ubxxx_write_reg(i2c_client,0x2c,0x34,0x02);

	ds90ubxxx_write_reg(i2c_client,0x2c,0x1f,0x09);
	ds90ubxxx_write_reg(i2c_client,0x2c,0x1e,0x99);
	ds90ubxxx_write_reg(i2c_client,0x2c,0x1d,0x19);

	ds90ubxxx_write_reg(i2c_client,0x2d,0x21,0x09);
	ds90ubxxx_write_reg(i2c_client,0x2d,0x20,0x09);
	ds90ubxxx_write_reg(i2c_client,0x2d,0x34,0x02);

	ds90ubxxx_write_reg(i2c_client,0x2d,0x1f,0x09);
	ds90ubxxx_write_reg(i2c_client,0x2d,0x1e,0x99);
	ds90ubxxx_write_reg(i2c_client,0x2d,0x1d,0x19);

	return 0;
}

//// DS90Ux98x-Q1 Auto Script Generation Output
//// Tool Version 3.0
//// Serializer: DS90Ux981-Q1
//// User Inputs:
//// Serializer I2C Address= 0x18
//// Port 0 DSI Lanes = 4
//// Port 0 DSI Rate = 600 Mbps/lane
//// FPD-Link Configuration: FPD-Link IV Single Port 0 - 3.375Gbps


//// Number of Displays = 1

//// Video Processor 0 Properties:
//// Total Horizontal Pixels = 2088
//// Total Vertical Lines = 763
//// Active Horizontal Pixels = 1920
//// Active Vertical Lines = 720
//// Horizontal Back Porch = 32
//// Vertical Back Porch = 24
//// Horizontal Sync = 24
//// Vertical Sync = 3
//// Horizontal Front Porch = 112
//// Vertical Front Porch = 16
//// Horizontal Sync Polarity = Positive
//// Vertical Sync Polarity = Positive
//// Bits per pixel = 24
//// Pixel Clock = 95.6MHz

//// Deserializer 0: DS90Ux988-Q1
//// Note: The 988 should be strapped to 3.375Gbps FPD IV mode using MODE_SEL0
//// User Inputs:
//// Deserializer I2C Address = 0x70
//// Deserializer I2C Alias = 0x70
//// 988 OLDI Output Mode
//// Dual OLDI Mode
//// OLDI Port 0 Bpp = 24
//// OLDI Port 1 Bpp = 24
//// OLDI Port 0 MAPSEL = H (MSB on D3/D4)
//// OLDI Port 1 MAPSEL = H (MSB on D3/D4)
//// Dual OLDI Video Source = Serializer Stream 0
//// OLDI PATGEN Disabled
static int dx11_dis_config(struct ds90ubxxx_i2c *ds90ubxxx_i2c)
{
	u8 Reg_value =0;
	struct i2c_client *i2c_client = ds90ubxxx_i2c->i2c_client;
	u8 serAddr = ds90ubxxx_i2c->ser_addr;
	u8 desAlias0 = ds90ubxxx_i2c->desdis_addr;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x70,desAlias0 << 1);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x78,desAlias0 << 1);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x88,0x0);

	//// *********************************************
	//// Program DSI Configs
	//// *********************************************

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value | 0x8;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value);//Disable DSI

	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1);//Select port 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x10);//Change indirect page to page 4
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xc);//Port 0 TSKIP value:6

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x4f);
	Reg_value = Reg_value & 0x73;
	Reg_value = Reg_value | 0x8c;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x4f,Reg_value);//Set number of lanes and continuous or non-continuous

	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x3);//Select write port 0 and 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0xbd,0x0);//Set DSI source for the Video processors 0 and 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0xbe,0x0);//Set DSI source for the Video processors 2 and 3
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1);//Select port 0

	//// *********************************************
	//// Program SER to FPD-Link IV mode
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x5b,0x23);//Disable FPD3 FIFO pass through
	ds90ubxxx_write_reg(i2c_client,serAddr,0x5,0x2c);//Force FPD4_TX single port 0 mode

	/*******reverse DSI P/N,only adapt for display board in evb*********************/
	ds90ubxxx_write_reg(i2c_client, serAddr,0x40,0x14);
	ds90ubxxx_write_reg(i2c_client, serAddr,0x41,0x16);
	ds90ubxxx_write_reg(i2c_client, serAddr,0x42,0x1f);

	//// *********************************************
	//// Set up FPD IV PLL Settings - This section can be commented out to improve bringup time if 983/981 MODE_SEL0 and MODE_SEL2 are strapped to the correct FPD IV speed
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x8);//Select PLL reg page
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x1b);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x8);//Disable PLL0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5b);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x8);//Disable PLL1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,0xd1);//Enable mode overwrite
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x84);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x2);//Switch encoder from ADAS to IVI on port 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x8);//Select PLL page
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5);//Select Ncount Reg
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x7d);//Set Ncount
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x13);//Select post div reg
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x90);//Set post div for 3.375Gbps
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1);//Select write reg to port 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x6a,0xa);//set BC sampling rate
	ds90ubxxx_write_reg(i2c_client,serAddr,0x6e,0x80);//set BC fractional sampling
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x4);//Select FPD page and set BC settings for FPD IV port 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x6);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0xd);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x34);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0xe);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x53);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x8);//Select PLL page
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x45);//Select Ncount Reg
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x7d);//Set Ncount
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x53);//Select post div reg
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x90);//Set post div for 6.75 Gbps
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x12);//Select write reg to port 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x6a,0xa);//set BC sampling rate
	ds90ubxxx_write_reg(i2c_client,serAddr,0x6e,0x80);//set BC fractional sampling
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x4);//Select FPD page and set BC settings for FPD IV port 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x26);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x2d);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x34);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x2e);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x53);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,0x91);//Set HALFRATE_MODE
	//// *********************************************
	//// Zero out PLL fractional - This section can be commented out to improve bringup time if 983/981 MODE_SEL0 and MODE_SEL2 are strapped to the correct FPD IV speed
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x8);//Select PLL page
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x4);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x1);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x1e);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x1f);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x20);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x44);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x1);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5e);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5f);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x60);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	//// *********************************************
	//// Configure and Enable PLLs - This section can be commented out to improve bringup time if 983/981 MODE_SEL0 and MODE_SEL2 are strapped to the correct FPD IV speed
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0xe);//Select VCO reg
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xc7);//Set VCO
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x4e);//Select VCO reg
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xc7);//Set VCO
	ds90ubxxx_write_reg(i2c_client,serAddr,0x1,0x30);//soft reset PLL
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x8);//Select PLL page
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x1b);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//Enable PLL0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x1,0x1);//soft reset Ser
	msleep(40);
	//// *********************************************
	//// Enable I2C Passthrough
	//// *********************************************
	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x7);
	Reg_value |= 0x08;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x07,Reg_value);//Enable I2C Passthrough

	ds90ubxxx_write_reg(i2c_client,desAlias0,0x1,0x1);//Soft reset Des
	msleep(40);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1);//Select write to port0 reg
	//// *********************************************
	//// Program VP Configs
	//// *********************************************
	// Configure VP 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x32);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x2);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x80);//VID H Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x7);//VID H Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x10);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x80);//Horizontal Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x7);//Horizontal Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x20);//Horizontal Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//Horizontal Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x18);//Horizontal Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//Horizontal Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x28);//Horizontal Total
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x8);//Horizontal Total
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xd0);//Vertical Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x2);//Vertical Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x18);//Vertical Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//Vertical Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3);//Vertical Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//Vertical Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x10);//Vertical Front Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//Vertical Front Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x27);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);//HSYNC Polarity = +, VSYNC Polarity = +
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x23);//M/N Register
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x42);//M value
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x24);//M value
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xf);//N value

	//// *********************************************
	//// Enable VPs
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x43,0x0);//Set number of VPs used = 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x44,0x1);//Enable video processors
#if 0
	//// *********************************************
	//// Enable PATGEN
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x30);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x29);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x8);//Set PATGEN Color Depth to 24bpp for VP0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x28);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x95);//Enable PATGEN on VP0 - Comment out this line to disable PATGEN and enable end to end video
#endif //0
	//// *********************************************
	//// Configure Serializer TX Link Layer
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x2e);//Link layer Reg page
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x1);//Link layer 0 stream enable
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x1);//Link layer 0 stream enable
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x6);//Link layer 0 time slot 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x41);//Link layer 0 time slot
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x20);//Set Link layer vp bpp
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x55);//Set Link layer vp bpp according to VP Bit per pixel
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x0);//Link layer 0 enable
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3);//Link layer 0 enable

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value & 0xf7;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value);//Enable DSI

	//// *********************************************
	//// Clear CRC errors from initial link process
	//// *********************************************

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value | 0x20;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value);//CRC Error Reset

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value & 0xdf;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value);//CRC Error Reset Clear

	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1);

	//// *********************************************
	//// Hold Des DTG in reset
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x40,0x50);//Select DTG Page
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x32);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x6);//Hold Port 0 DTG in reset
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x62);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x6);//Hold Port 1 DTG in reset


	//// *********************************************
	//// Disable Stream Mapping
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xe,0x3);//Select both Output Ports
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xd0,0x0);//Disable FPD4 video forward to Output Port
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xd7,0x0);//Disable FPD3 video forward to Output Port

	//// *********************************************
	//// Setup DTG for port 0
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x40,0x50);//Select DTG Page
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x20);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x53);//Set up DTG BPP, Sync Polarities, and Measurement Type
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x29);//Set Hstart
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x80);//Hstart upper byte
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x2a);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x38);//Hstart lower byte
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x2f);//Set HSW
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x40);//HSW upper byte
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x30);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x18);//HSW lower byte

	//// *********************************************
	//// Map video to display output
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xe,0x3);//Select both Output Ports
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xd0,0xc);//Enable FPD_RX video forward to Output Port
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xd1,0xf);//Every stream forwarded on DC
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xd6,0x0);//Send Stream 0 to Output Port 0 and Send Stream 0 to Output Port 1
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xd7,0x0);//FPD3 mapping disabled
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xe,0x1);//Select Port 0

	//// *********************************************
	//// Configure 988 Display
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x40,0x2c);//Configure OLDI/RGB Port Settings
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x2f);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x2f);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x40,0x2e);//Configure OLDI/RGB PLL
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x8);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x92);//PLL_NUM23_16
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x3);//PLL_NUM15_8
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x97);//PLL_NUM7_0
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0xff);//PLL_DEN23_16
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0xff);//PLL_DEN15_8
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0xa5);//PLL_DEN7_0
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x18);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x31);//PLL_NDIV
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x2d);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x12);//TX_SEL_CLKDIV

	//// *********************************************
	//// Release Des DTG reset
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x40,0x50);//Select DTG Page
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x32);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x4);//Release Port 0 DTG
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x62);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x4);//Release Port 1 DTG

	//// *********************************************
	//// Enable OLDI Output
	//// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x1,0x40);//OLDI Reset
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x40,0x2c);//Enable OLDI/RGB
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x2);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x14);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x2);//Toggle OLDI_SER_EN for Dual OLDI Mode
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x4);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x14);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x20);//P0 TX_EN
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x80);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x22);//P1 TX_EN
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x80);

	//UH988 GPIO13 configs as output and pull high to lightup LCD to adapt new devices.
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x12,0xdf);//GPIO13 input disable
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x22,0xc1);//GPIO13 output enable,pull high.

	return 0;
}

static int dx11_rsd_config(struct ds90ubxxx_i2c *ds90ubxxx_i2c)
{
	u8 Reg_value =0;

	/*u8 GENERAL_CFG = 0;
	u8 IVImask = 0;
	u8 GENERAL_CFG_REG = 0;
	u8 FPD4_CFG = 0;
	u8 TX_MODE_MASK = 0;
	u8 FPD4_CFG_REG = 0;
	u8 I2C_PASS_THROUGH = 0;
	u8 I2C_PASS_THROUGH_REG = 0;
	u8 I2C_PASS_THROUGH_MASK = 0;

	u8 TEMP_FINAL = 0;
	u8 TEMP_FINAL_C = 0;
	u8 Efuse_TS_CODE = 0;
	u8 Ramp_UP_Range_CODES_Needed = 0;
	u8 Ramp_DN_Range_CODES_Needed = 0;
	u8 Ramp_UP_CAP_DELTA = 0;
	u8 Ramp_DN_CAP_DELTA = 0;
	u8 TS_CODE_UP = 0;
	u8 rb = 0;
	u8 TS_CODE_DN = 0;
	u8 desAddr0 = 0x38;*/

	struct i2c_client *i2c_client = ds90ubxxx_i2c->i2c_client;
	u8 serAddr = ds90ubxxx_i2c->ser_addr;

	// *********************************************
	// Program DSI Configs
	// *********************************************

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value | 0x8;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value); //Disable DSI

	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1); //Select port 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x10); //Change indirect page to page 4
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x6); //Port 0 TSKIP value:3

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x4f);
	Reg_value = Reg_value & 0x73;
	Reg_value = Reg_value | 0x8c;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x4f,Reg_value); //Set number of lanes and continuous or non-continuous

	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x12); //Select port 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x18); //Change indirect page to page 4
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x1c); //Port 0 TSKIP value:3

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x4f);
	Reg_value = Reg_value & 0x73;
	Reg_value = Reg_value | 0x8c;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x4f,Reg_value); //Set number of lanes and continuous or non-continuous

	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x3); //Select write port 0 and 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0xbd,0x0); //Set DSI source for the Video processors 0 and 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0xbe,0x0); //Set DSI source for the Video processors 2 and 3
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1); //Select port 0


	// *********************************************
	// Set FPD Port Configuration
	// *********************************************
	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x7);

	Reg_value = Reg_value & 0xFC;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x07,Reg_value); // Set IVI Mode
	Reg_value= ds90ubxxx_read_reg(i2c_client,serAddr,0x5);

	Reg_value = Reg_value & 0xC3;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x05,Reg_value); // Set FPD III Mode
	ds90ubxxx_write_reg(i2c_client,serAddr,0x59,0x1); //Set FPD3_TX_MODE to FPD III Single Port 0

	ds90ubxxx_write_reg(i2c_client, serAddr,0x40,0x14);
	ds90ubxxx_write_reg(i2c_client, serAddr,0x41,0x16);
	ds90ubxxx_write_reg(i2c_client, serAddr,0x42,0x1f);

	// *********************************************
	// Program PLLs
	// *********************************************
	// Program PLL for Port 1: FPD III Mode 2450Mbps
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x8);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x4);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x9); //Set fractional mash order
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x13);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xe0); //Set VCO Post Div = 4, VCO Auto Sel for CS2.0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0xa); //Set auto increment
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x56); //Set Ndiv = 90
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Set Ndiv = 90
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x18);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xc4); //Set denominator = 16777206
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xfe); //Set denominator = 16777206
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xff); //Set denominator = 16777206
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x1e);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xc9); //Set numerator = 12427560
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3b); //Set numerator = 12427560
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x6b); //Set numerator = 12427560

	//ds90ubxxx_write_reg(i2c_client,serAddr,0x1,0x30); //PLL Reset

	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x4); //Set FPD Page to configure BC Settings for Port 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x6);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xff);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0xd);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x70);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0xe);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x70);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x1,0x30); //Reset PLLs
	msleep(10);

	ds90ubxxx_write_reg(i2c_client,serAddr,0x7,0x88);
	// *********************************************
	// Program VP Configs
	// *********************************************
	// Configure VP 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x32);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x2);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //VID H Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x5); //VID H Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x10);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Horizontal Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x5); //Horizontal Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x8); //Horizontal Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Horizontal Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x8); //Horizontal Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Horizontal Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x96); //Horizontal Total
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x5); //Horizontal Total
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Vertical Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3); //Vertical Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3); //Vertical Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Vertical Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3); //Vertical Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Vertical Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3); //Vertical Front Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Vertical Front Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x27);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //HSYNC Polarity = +, VSYNC Polarity = +

	// *********************************************
	// Enable VPs
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x43,0x0); //Set number of VPs used = 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x44,0x1); //Enable video processors

	// *********************************************
	// Set FPD3 Stream Mapping
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1); //Select FPD TX Port 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x57,0x0); //Set FPD TX Port 1 Stream Source = VP0

	ds90ubxxx_write_reg(i2c_client,serAddr,0x5b,0x2b); //Enable FPD III FIFO

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value & 0xf7;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value); //Enable DSI


	// *********************************************
	// Clear CRC errors from initial link process
	// *********************************************

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value | 0x20;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value); //CRC Error Reset

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value & 0xdf;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value); //CRC Error Reset Clear

	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1);

	msleep(1000);
	ds90ubxxx_write_reg(i2c_client,0x2c,0x21,0x09);
	ds90ubxxx_write_reg(i2c_client,0x2c,0x20,0x09);
	ds90ubxxx_write_reg(i2c_client,0x2c,0x34,0x02);

	ds90ubxxx_write_reg(i2c_client,0x2c,0x1f,0x09);
	ds90ubxxx_write_reg(i2c_client,0x2c,0x1e,0x99);
	ds90ubxxx_write_reg(i2c_client,0x2c,0x1d,0x19);


	return 0;
}

int ds90ubxxx_chip_init(struct ds90ubxxx_i2c *ds90ubxxx_i2c)
{
	struct device *dev = &ds90ubxxx_i2c->i2c_client->dev;
	//int m_gpio111_en = 0;
	int retval = 0;

	//m_gpio111 pull on  //gpio111
	ds90ubxxx_i2c->m_gpio111_en = of_get_named_gpio(dev->of_node, "981_pdb", 0);
	if (!gpio_is_valid(ds90ubxxx_i2c->m_gpio111_en))
	{
 		serial_debug("get gpio111 fail");
		return -EINVAL;
	}
	retval = devm_gpio_request_one(dev, ds90ubxxx_i2c->m_gpio111_en, GPIOF_OUT_INIT_HIGH, "m_gpio111_en");
	if (retval != 0)
		return retval;

	retval = gpio_direction_output(ds90ubxxx_i2c->m_gpio111_en, 0);
	if (retval != 0)
	{
		serial_debug("direction output error \n");
		return retval;
	}

	serial_debug("gpio 111 value : %d, %d\n", gpio_get_value(ds90ubxxx_i2c->m_gpio111_en), ds90ubxxx_i2c->m_gpio111_en);
	msleep(50);
	gpio_set_value(ds90ubxxx_i2c->m_gpio111_en, 1);
	serial_debug("gpio 111 value after : %d\n", gpio_get_value(ds90ubxxx_i2c->m_gpio111_en));

	return 0;
}


static int ub98x_i2c_write_with_readback(struct i2c_client *client, u8 addr, u8 reg, u8 data, u64 rbdelay, s8 retry)
{
	int ret = 0;
	u8 readback = 0;
	char remain = 0;

	for (remain=retry+1;remain>0;remain--) {
		ret = ds90ubxxx_write_reg(client,addr,reg, data);
		if(ret == 0) {
			if (rbdelay > 0) {
				mdelay(rbdelay);
			}

			readback = ds90ubxxx_read_reg(client, addr, reg);
			if (readback == data) {
				return 0;
			} else {
				pr_err("%s,readback error,{0x%02x,0x%02x,0x%02x} rb:%d!=data:0x%x, remain:%d\n",
						__func__, addr, reg, data, readback, data, remain);
			}
		} else {
			pr_err("%s,write error, ret:%d,{0x%02x,0x%02x,0x%02x} reg:0x%x, remain:%d\n",
					__func__, addr, reg, data, ret, reg, remain);
		}
	}
	return -1;
}

static int ub98x_reg_array_write(struct i2c_client *client, unsigned char (*array)[6], unsigned int length)
{
	int i = 0;
	int ret = 0;
	for (i=0;i<length;i++) {
		u8 delay = array[i][5];
		if (delay > 0) {
			mdelay(delay);
		}

		if (array[i][0] == 0) {
			//just delay
			continue;
		}

		delay = array[i][3];
		ret = ub98x_i2c_write_with_readback(client, array[i][0], array[i][1], array[i][2], (u64)delay, array[i][4]);
		if (ret < 0) {
			pr_err("%s,ub98x i2c write readback error!!\n",__func__);
			return -1;
		}
	}

	return 0;
}


static void ds90ubxxx_chip_backlight_init(struct i2c_client *i2c_client)
{
	int ret = 0;
	int retry_time = 2;

	do{
		ret = ub98x_reg_array_write(i2c_client, ub984_dev_init_reg, ARRAY_SIZE(ub984_dev_init_reg));
		if (ret < 0)
			pr_err("%s ub984_dev_init_reg array retry! ret:%d ,time:%d\n",__func__,ret,--retry_time);
	}while((ret < 0) && (retry_time > 0));
}

static void ds90ubxxx_chip_touch_enabled(struct i2c_client *i2c_client)
{
	int ret = 0;
	ret = ub98x_reg_array_write(i2c_client, ub983_dsi_subdev_init_reg, ARRAY_SIZE(ub983_dsi_subdev_init_reg));
	if (ret  < 0)
		pr_err("%s, ub98xx touch chip enabled failed!!\n",__func__);
}

/* Serializer: DS90Ux981-Q1
* User Inputs:
* Serializer I2C Address= 0x18
* Port 0 DSI Lanes = 4
* Port 0 DSI Rate = 1629 Mbps/lane
* FPD-Link Configuration: FPD-Link IV Single Port 0 - 6.75Gbps
* Number of Displays = 1
* Video Processor 0 Properties:
* Total Horizontal Pixels = 2756
* Total Vertical Lines = 1642
* Active Horizontal Pixels = 2560
* Active Vertical Lines = 1600
* Horizontal Back Porch = 56
* Vertical Back Porch = 8
* Horizontal Sync = 56
* Vertical Sync = 2
* Horizontal Front Porch = 84
* Vertical Front Porch = 32
* Horizontal Sync Polarity = Positive
* Vertical Sync Polarity = Positive
* Bits per pixel = 24
* Pixel Clock = 271.5MHz
* PATGEN Disabled

* Deserializer 0: DS90Ux984-Q1
* Note: The 984 should be strapped to 6.75Gbps FPD IV mode using MODE_SEL0
* User Inputs:
* Deserializer I2C Address = 0x58
* Deserializer I2C Alias = 0x58
* DP Port 0 Enabled
* DP0 is outputting video 0 from the SER
* DP Port 0 PatGen Disabled
* DP Port 1 Disabled
* DP Port 1 PatGen Disabled
* DP Rate set to 2.7 Gbps
* DP lane number set to 4 lanes
*/

static bool dx11_dsi_2k_config(struct ds90ubxxx_i2c *ds90ubxxx_i2c)
{
	u8 Reg_value =0;
	struct i2c_client *i2c_client = ds90ubxxx_i2c->i2c_client;
	u8 serAddr = ds90ubxxx_i2c->ser_addr;
	u8 desAlias0 = ds90ubxxx_i2c->desdis_addr;

	ds90ubxxx_write_reg(i2c_client,serAddr,0x70,desAlias0 << 1);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x78,desAlias0 << 1);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x88,0x0);

	// *********************************************
	// Program DSI Configs
	// *********************************************
	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value | 0x8;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value); //Disable DSI

	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1); //Select port 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x10); //Change indirect page to page 4
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x2e); //Port 0 TSKIP value:23

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x4f);
	Reg_value = Reg_value & 0x73;
	Reg_value = Reg_value | 0x8c;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x4f,Reg_value); //Set number of lanes and continuous or non-continuous

	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x3); //Select write port 0 and 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0xbd,0x0); //Set DSI source for the Video processors 0 and 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0xbe,0x0); //Set DSI source for the Video processors 2 and 3
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1); //Select port 0

	// *********************************************
	// Program SER to FPD-Link IV mode
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x5b,0x23); //Disable FPD3 FIFO pass through
	ds90ubxxx_write_reg(i2c_client,serAddr,0x5,0x2c); //Force FPD4_TX single port 0 mode

	/*******reverse DSI P/N,only adapt for display board in evb*********************/
	ds90ubxxx_write_reg(i2c_client, serAddr,0x40,0x14);
	ds90ubxxx_write_reg(i2c_client, serAddr,0x41,0x16);
	ds90ubxxx_write_reg(i2c_client, serAddr,0x42,0x1f);

	// *********************************************
	// Set up FPD IV PLL Settings - This section can be commented out to improve bringup time if 983/981 MODE_SEL0
	// and MODE_SEL2 are strapped to the correct FPD IV speed
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x8); //Select PLL reg page
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x1b);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x8); //Disable PLL0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5b);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x8); //Disable PLL1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,0xd1); //Enable mode overwrite
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x84);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x2); //Switch encoder from ADAS to IVI on port 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x8); //Select PLL page
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5); //Select Ncount Reg
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x7d); //Set Ncount
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x13); //Select post div reg
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x90); //Set post div for 6.75 Gbps
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1); //Select write reg to port 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x6a,0xa); //set BC sampling rate
	ds90ubxxx_write_reg(i2c_client,serAddr,0x6e,0x80); //set BC fractional sampling
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x4); //Select FPD page and set BC settings for FPD IV port 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x6);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0xd);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x34);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0xe);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x53);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x8); //Select PLL page
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x45); //Select Ncount Reg
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x7d); //Set Ncount
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x53); //Select post div reg
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x90); //Set post div for 6.75 Gbps
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x12); //Select write reg to port 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x6a,0xa); //set BC sampling rate
	ds90ubxxx_write_reg(i2c_client,serAddr,0x6e,0x80); //set BC fractional sampling
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x4); //Select FPD page and set BC settings for FPD IV port 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x26);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x2d);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x34);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x2e);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x53);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,0xd1); //Set HALFRATE_MODE
	// *********************************************
	// Zero out PLL fractional - This section can be commented out to improve bringup time if 983/981 MODE_SEL0
	// and MODE_SEL2 are strapped to the correct FPD IV speed
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x8); //Select PLL page
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x4);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x1);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x1e);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x1f);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x20);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x44);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x1);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5e);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x5f);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x60);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0);
	// *********************************************
	// Configure and Enable PLLs - This section can be commented out to improve bringup time if 983/981 MODE_SEL0
	// and MODE_SEL2 are strapped to the correct FPD IV speed
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0xe); //Select VCO reg
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xc7); //Set VCO
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x4e); //Select VCO reg
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xc7); //Set VCO
	ds90ubxxx_write_reg(i2c_client,serAddr,0x1,0x30); //soft reset PLL
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x8); //Select PLL page
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x1b);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Enable PLL0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x1,0x1); //soft reset Ser
	msleep(40);
	// *********************************************
	// Enable I2C Passthrough
	// *********************************************
	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x7);
	Reg_value = Reg_value | 0x08;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x07,Reg_value); //Enable I2C Passthrough

	if ((desAlias0 << 1) != ds90ubxxx_read_reg(i2c_client, desAlias0, 0)) {
		pr_err("%s,Deserializer device is not connected to slot.\n",__func__);
		return false;
	}

	ds90ubxxx_write_reg(i2c_client,desAlias0,0x1,0x1); //Soft reset Des
	msleep(40);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1); //Select write to port0 reg

	// *********************************************
	// Program VP Configs
	// *********************************************
	// Configure VP 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x32);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x2);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //VID H Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xa); //VID H Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x10);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Horizontal Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xa); //Horizontal Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x38); //Horizontal Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Horizontal Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x38); //Horizontal Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Horizontal Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xc4); //Horizontal Total
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xa); //Horizontal Total
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x40); //Vertical Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x6); //Vertical Active
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x8); //Vertical Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Vertical Back Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x2); //Vertical Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Vertical Sync
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x20); //Vertical Front Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //Vertical Front Porch
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x27);
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x0); //HSYNC Polarity = +, VSYNC Polarity = +
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x23); //M/N Register
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x7c); //M value
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x33); //M value
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0xf); //N value

	// *********************************************
	// Enable VPs
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x43,0x0); //Set number of VPs used = 1
	ds90ubxxx_write_reg(i2c_client,serAddr,0x44,0x1); //Enable video processors

	// *********************************************
	// Configure Serializer TX Link Layer
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,serAddr,0x40,0x2e); //Link layer Reg page
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x1); //Link layer 0 stream enable
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x1); //Link layer 0 stream enable
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x6); //Link layer 0 time slot 0
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x41); //Link layer 0 time slot
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x20); //Set Link layer vp bpp
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x55); //Set Link layer vp bpp according to VP Bit per pixel
	ds90ubxxx_write_reg(i2c_client,serAddr,0x41,0x0); //Link layer 0 enable
	ds90ubxxx_write_reg(i2c_client,serAddr,0x42,0x3); //Link layer 0 enable

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value & 0xf7;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value); //Enable DSI

	// *********************************************
	// Clear CRC errors from initial link process
	// *********************************************
	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value | 0x20;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value); //CRC Error Reset

	Reg_value = ds90ubxxx_read_reg(i2c_client,serAddr,0x2);
	Reg_value = Reg_value & 0xdf;
	ds90ubxxx_write_reg(i2c_client,serAddr,0x2,Reg_value); //CRC Error Reset Clear

	ds90ubxxx_write_reg(i2c_client,serAddr,0x2d,0x1);

	// *********************************************
	// Hold Des DTG in reset
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x40,0x50); //Select DTG Page
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x32);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x6); //Hold Port 0 DTG in reset
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x62);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x6); //Hold Port 1 DTG in reset


	// *********************************************
	// Disable Stream Mapping
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xe,0x3); //Select both Output Ports
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xd0,0x0); //Disable FPD4 video forward to Output Port
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xd7,0x0); //Disable FPD3 video forward to Output Port


	// *********************************************
	// Force DP Rate
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x40,0x2c); //Select DP Page
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x81);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x60); //Set DP Rate to 2.7Gbps
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x82);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x3); //Enable force DP rate with calibration disabled
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x40,0x2c); //Select DP Page
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x91);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0xc); //Force 4 lanes
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x40,0x30); //Disable DP SSCG
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0xf);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x1,0x40);


	// *********************************************
	// Setup DP ports
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xe,0x12); //Select Port 1 registers
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x46,0x0); //Disable DP Port 1
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xe,0x1); //Select Port 0 registers
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x1,0x40); //DP-TX-PLL RESET Applied


	// *********************************************
	// Map video to display output
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xe,0x3); //Select both Output Ports
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xd0,0xc); //Enable FPD_RX video forward to Output Port
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xd1,0xf); //Every stream forwarded on DC
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xd6,0x0); //Send Stream 0 to Output Port 0 and Send Stream 0 to Output Port 1
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xd7,0x0); //FPD3 mapping disabled
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xe,0x1); //Select Port 0


	// *********************************************
	// Program quad pixel clock for DP port 0
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xe,0x1); //Select Port0 registers
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xb1,0x1); //Enable clock divider
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xb2,0x8c); //Program M value lower byte
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xb3,0x24); //Program M value middle byte
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xb4,0x4); //Program M value upper byte
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xb5,0xc0); //Program N value lower byte
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xb6,0x7a); //Program N value middle byte
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xb7,0x10); //Program N value upper byte
	ds90ubxxx_write_reg(i2c_client,desAlias0,0xe,0x1); //Select Port 0 registers


	// *********************************************
	// Setup DTG for port 0
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x40,0x50); //Select DTG Page
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x20);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x93); //Set up DTG BPP, Sync Polarities, and Measurement Type
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x29); //Set Hstart
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x80); //Hstart upper byte
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x2a);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x70); //Hstart lower byte
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x2f); //Set HSW
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x40); //HSW upper byte
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x30);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x38); //HSW lower byte


	// *********************************************
	// Program DPTX for DP port 0
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x48,0x1); //Enable APB interface
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x48,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x49,0xa4); //Set bit per color
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4a,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4b,0x20);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4c,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4d,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4e,0x0);

	ds90ubxxx_write_reg(i2c_client,desAlias0,0x48,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x49,0xb8); //Set pixel width
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4a,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4b,0x4);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4c,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4d,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4e,0x0);

	ds90ubxxx_write_reg(i2c_client,desAlias0,0x48,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x49,0xac); //Set DP Mvid
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4a,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4b,0xb6);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4c,0x80);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4d,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4e,0x0);

	ds90ubxxx_write_reg(i2c_client,desAlias0,0x48,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x49,0xb4); //Set DP Nvid
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4a,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4b,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4c,0x80);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4d,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4e,0x0);

	ds90ubxxx_write_reg(i2c_client,desAlias0,0x48,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x49,0xc8); //Set TU Mode
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4a,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4b,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4c,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4d,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4e,0x0);

	ds90ubxxx_write_reg(i2c_client,desAlias0,0x48,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x49,0xb0); //Set TU Size
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4a,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4b,0x40);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4c,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4d,0x30);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4e,0x6);

	ds90ubxxx_write_reg(i2c_client,desAlias0,0x48,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x49,0xc8); //Set FIFO Size
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4a,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4b,0x5);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4c,0x40);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4d,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4e,0x0);

	ds90ubxxx_write_reg(i2c_client,desAlias0,0x48,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x49,0xbc); //Set data count
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4a,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4b,0x80);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4c,0x7);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4d,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4e,0x0);

	ds90ubxxx_write_reg(i2c_client,desAlias0,0x48,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x49,0xc0); //Disable STREAM INTERLACED
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4a,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4b,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4c,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4d,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4e,0x0);

	ds90ubxxx_write_reg(i2c_client,desAlias0,0x48,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x49,0xc4); //Set SYNC polarity
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4a,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4b,0xc);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4c,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4d,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4e,0x0);

	// *********************************************
	// Release Des DTG reset
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x40,0x50); //Select DTG Page
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x32);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x4); //Release Port 0 DTG
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x41,0x62);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x42,0x4); //Release Port 1 DTG


	ds90ubxxx_write_reg(i2c_client,desAlias0,0x48,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x49,0x80); //Set Htotal
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4a,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4b,0xc4);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4c,0xa);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4d,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4e,0x0);

	// *********************************************
	// Enable DP 0 output
	// *********************************************
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x48,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x49,0x84); //Enable DP output
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4a,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4b,0x1);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4c,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4d,0x0);
	ds90ubxxx_write_reg(i2c_client,desAlias0,0x4e,0x0);

	return true;
}


static int ds90ubxxx_i2c_probe(struct i2c_client *i2c_client, const struct i2c_device_id *id)
{
	int ret;
	struct device *dev = &i2c_client->dev;
	struct device_node *i2c_np = dev->of_node;
	struct ds90ubxxx_i2c *ds90ubxxx_i2c =  devm_kzalloc(dev, sizeof(*ds90ubxxx_i2c), GFP_KERNEL);
	ds90ubxxx_i2c->i2c_client = i2c_client;
	serial_debug("ds90ubxxx_i2c probe\n");

	ret = of_property_read_string(i2c_np, "serializer_type", &ds90ubxxx_i2c->serializer_type);
	if (ret < 0){
		dev_err(dev, " get serializer-type fail in dts\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(i2c_np, "desdis_addr", &ds90ubxxx_i2c->desdis_addr);
	if (ret < 0){
		dev_err(dev, "get desdis_addr fail in dts\n");
		return -EINVAL;
	}

	ds90ubxxx_i2c->ser_addr = i2c_client->addr;

	dev_set_drvdata(dev, ds90ubxxx_i2c);

	if (!(strcmp(ds90ubxxx_i2c->serializer_type,"dx11_rsd"))){
		ds90ubxxx_chip_init(ds90ubxxx_i2c);

		dx11_rsd_config(ds90ubxxx_i2c);
	}
	else if (!(strcmp(ds90ubxxx_i2c->serializer_type,"dx11_dis"))){
		ds90ubxxx_chip_init(ds90ubxxx_i2c);

		dx11_dis_config(ds90ubxxx_i2c);
	}
	else if (!(strcmp(ds90ubxxx_i2c->serializer_type,"dx11_dis_sdpic"))){
		ds90ubxxx_chip_init(ds90ubxxx_i2c);

		dx11_dis_sdpic_config(ds90ubxxx_i2c);
	}
	else if (!(strcmp(ds90ubxxx_i2c->serializer_type,"dx11_dual_rsd"))){
		ds90ubxxx_chip_init(ds90ubxxx_i2c);

		dx11_dual_rsd_config(ds90ubxxx_i2c);
	} else if (!(strcmp(ds90ubxxx_i2c->serializer_type,"dx11_dsi_2k"))) {
		if (dx11_dsi_2k_config(ds90ubxxx_i2c)) {
			ds90ubxxx_chip_backlight_init(i2c_client);
			ds90ubxxx_chip_touch_enabled(i2c_client);
		}
	}

	return 0;
}

static int ds90ubxxx_i2c_remove(struct i2c_client *i2c_client)
{
	//struct ds90ubxxx_i2c *ds90ubxxx_i2c = dev_get_drvdata(&i2c_client->dev);
	return 0;
}

static int ti981_panel_suspend(struct device *dev)
{
	//struct ds90ubxxx_i2c *ds90ubxxx_i2c = dev_get_drvdata(dev);

	return 0;
}

static int ti981_panel_resume(struct device *dev)
{
	struct ds90ubxxx_i2c *ds90ubxxx_i2c = dev_get_drvdata(dev);

	if (!(strcmp(ds90ubxxx_i2c->serializer_type,"dx11_rsd"))){
		ds90ubxxx_chip_init(ds90ubxxx_i2c);

		dx11_rsd_config(ds90ubxxx_i2c);
	}
	else if (!(strcmp(ds90ubxxx_i2c->serializer_type,"dx11_dis"))){
		ds90ubxxx_chip_init(ds90ubxxx_i2c);

		dx11_dis_config(ds90ubxxx_i2c);
	}
	else if (!(strcmp(ds90ubxxx_i2c->serializer_type,"dx11_dis_sdpic"))){
		ds90ubxxx_chip_init(ds90ubxxx_i2c);

		dx11_dis_sdpic_config(ds90ubxxx_i2c);
	}
	else if (!(strcmp(ds90ubxxx_i2c->serializer_type,"dx11_dual_rsd"))){
		//ds90ubxxx_chip_init(ds90ubxxx_i2c);
		gpio_direction_output(ds90ubxxx_i2c->m_gpio111_en, 0);
		msleep(50);
		gpio_set_value(ds90ubxxx_i2c->m_gpio111_en, 1);

		dx11_dual_rsd_config(ds90ubxxx_i2c);
	}
	else if (!(strcmp(ds90ubxxx_i2c->serializer_type,"dx11_dsi_2k"))) {
		if (dx11_dsi_2k_config(ds90ubxxx_i2c)) {
			ds90ubxxx_chip_backlight_init(ds90ubxxx_i2c->i2c_client);
			ds90ubxxx_chip_touch_enabled(ds90ubxxx_i2c->i2c_client);
		}
	}
	return 0;
}

static const struct of_device_id ds90ubxxx_i2c_of_ids[] = {
	 { .compatible = "ds90ub981_i2c" },
	 { } /* sentinel */
};

static const struct dev_pm_ops ti981_pm_ops = {
	//SET_RUNTIME_PM_OPS(ti981_panel_suspend, ti981_panel_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(ti981_panel_suspend, ti981_panel_resume)
};


MODULE_DEVICE_TABLE(of, ds90ubxxx_i2c_of_ids);

static struct i2c_driver ds90ubxxx_i2c_driver = {
	.probe = ds90ubxxx_i2c_probe,
	.remove = ds90ubxxx_i2c_remove,
	.driver = {
		.name = "ds90ub981_i2c",
		.of_match_table = ds90ubxxx_i2c_of_ids,
		.pm     = &ti981_pm_ops,
	}
};

static int __init panel_init(void)
{
	return i2c_add_driver(&ds90ubxxx_i2c_driver);
}
module_init(panel_init);

static void __exit panel_exit(void)
{
	i2c_del_driver(&ds90ubxxx_i2c_driver);
}
module_exit(panel_exit);

MODULE_AUTHOR("zhang songqin <songqin.zhang@siengine.com>");
MODULE_AUTHOR("siengine");
MODULE_DESCRIPTION("PANEL DS90UB981 DX11 RSD");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
