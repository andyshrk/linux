// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) Wuhan SiEngine Co.Ltd
 * Author: Xiaohu Qian <xiaohu.qian@siengine.com>
 */
// SPDX-License-Identifier: GPL-2.0
/*
 * i.MX drm driver - Raydium MIPI-DSI panel driver
 *
 * Copyright (C) 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
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

/* Write Manufacture Command Set Control */
#define WRMAUCCTR 0xFE

/* Manufacturer Command Set pages (CMD2) */
struct cmd_set_entry {
	u8 cmd;
	u8 param;
};

/*
 * There is no description in the Reference Manual about these commands.
 * We received them from vendor, so just use them as is.
 */
static const struct cmd_set_entry manufacturer_cmd_set[] = {
	{0x8C, 0x8E},
	{0xFD, 0x1F},
	{0xF0, 0xB4},
};

#define MAX96789_UART_READ_ADDR 	0xC5
#define MAX96789_UART_WRITE_ADDR	0xC4
#define MAX96745_UART_READ_ADDR 	0xC5
#define MAX96745_UART_WRITE_ADDR	0xC4
#define MAX96752_UART_READ_ADDR 	0x91
#define MAX96752_UART_WRITE_ADDR	0x90

#define SERI_ID MAX96789_UART_WRITE_ADDR
#define DSER_ID MAX96752_UART_WRITE_ADDR

//asymmetric config for 1920  1080p+720p
unsigned char reg_cmds_asym_startek[][7] = {
	{0x79, SERI_ID, 0x03, 0x80, 0x01, 0x0f, 200}, // serial sync mode auto ,low level active
	//#Asymmetric 9_26_17-23 Using MAX96789/91/F (GMSL-1/2)
	//#Set Stream ID = 0 for GMSL PHY A
	{0x79, SERI_ID, 0x00, 0x53, 0x01, 0x10, 00},
	//#Set Stream ID = 1 for GMSL PHY B
	{0x79, SERI_ID, 0x00, 0x57, 0x01, 0x21, 00},
	//#Set Port A Lane Mapping
	{0x79, SERI_ID, 0x03 , 0x32, 0x01, 0x4E, 00},
	//#Set Port B Lane Mapping
	{0x79, SERI_ID, 0x03 , 0x33, 0x01, 0xE4, 00},
	//#Set GMSL type
	{0x79, SERI_ID, 0x00, 0x04, 0x01, 0xF2, 00},
	//#Clock Select
	{0x79, SERI_ID, 0x03 , 0x08, 0x01, 0x5C, 00},
	//#Start DSI Port
	{0x79, SERI_ID, 0x03 , 0x11, 0x01, 0x03, 00},
	//#Number of Lanes
	{0x79, SERI_ID, 0x03 , 0x31, 0x01, 0x03, 00},
	//#Set phy_config
	{0x79, SERI_ID, 0x03 , 0x30, 0x01, 0x06, 00},
	//#Set soft_dtx_en
	{0x79, SERI_ID, 0x03 , 0x1C, 0x01, 0x98, 00},
	//#Set soft_dtx
	{0x79, SERI_ID, 0x03 , 0x21, 0x01, 0x24, 00},
	//#Set soft_dty_en
	{0x79, SERI_ID, 0x03 , 0x1D, 0x01, 0x98, 00},
	//#Set soft_dty
	{0x79, SERI_ID, 0x03 , 0x22, 0x01, 0x24, 00},
	//#Init Default
	{0x79, SERI_ID, 0x03 , 0x26, 0x01, 0xE4, 00},
	//#HSYNC_WIDTH_L
	{0x79, SERI_ID, 0x03 , 0x85, 0x01, 0x34, 00},
	//#VSYNC_WIDTH_L
	{0x79, SERI_ID, 0x03 , 0x86, 0x01, 0x0C, 00},
	//#HSYNC_WIDTH_H/VSYNC_WIDTH_H
	{0x79, SERI_ID, 0x03 , 0x87, 0x01, 0x00, 00},
	//#VFP_L
	{0x79, SERI_ID, 0x03 , 0xA5, 0x01, 0x45, 00},
	//#VBP_H
	{0x79, SERI_ID, 0x03 , 0xA7, 0x01, 0x00, 00},
	//#VFP_H/VBP_L
	{0x79, SERI_ID, 0x03 , 0xA6, 0x01, 0xC0, 00},
	//#VRES_L
	{0x79, SERI_ID, 0x03 , 0xA8, 0x01, 0x38, 00},
	//#VRES_H
	{0x79, SERI_ID, 0x03 , 0xA9, 0x01, 0x04, 00},
	//#HFP_L
	{0x79, SERI_ID, 0x03 , 0xAA, 0x01, 0xC8, 00},
	//#HBP_H
	{0x79, SERI_ID, 0x03 , 0xAC, 0x01, 0x03, 00},
	//#HFP_H/HBP_L
	{0x79, SERI_ID, 0x03 , 0xAB, 0x01, 0x41, 00},
	//#HRES_L
	{0x79, SERI_ID, 0x03 , 0xAD, 0x01, 0x00, 00},
	//#HRES_H
	{0x79, SERI_ID, 0x03 , 0xAE, 0x01, 0x0F, 00},
	//#FIFO/DESKEW_EN
	{0x79, SERI_ID, 0x03 , 0xA4, 0x01, 0xC1, 00},
	//# ASYM_REGS
	{0x79, SERI_ID, 0x07 , 0x00, 0x01, 0x78, 00},
	{0x79, SERI_ID, 0x07 , 0x01, 0x01, 0x60, 00},
	{0x79, SERI_ID, 0x07 , 0x02, 0x01, 0x27, 00},
	{0x79, SERI_ID, 0x07 , 0x03, 0x01, 0x50, 00},
	{0x79, SERI_ID, 0x07 , 0x04, 0x01, 0x40, 00},
	{0x79, SERI_ID, 0x07 , 0x05, 0x01, 0x1A, 00},
	{0x79, SERI_ID, 0x07 , 0x06, 0x01, 0xF0, 00},
	{0x79, SERI_ID, 0x07 , 0x07, 0x01, 0xC0, 00},
	{0x79, SERI_ID, 0x07 , 0x08, 0x01, 0x4E, 00},
	{0x79, SERI_ID, 0x07 , 0x09, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x0A, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x0B, 0x01, 0x80, 00},
	{0x79, SERI_ID, 0x07 , 0x0C, 0x01, 0x07, 00},
	{0x79, SERI_ID, 0x07 , 0x0D, 0x01, 0x38, 00},
	{0x79, SERI_ID, 0x07 , 0x0E, 0x01, 0x04, 00},
	{0x79, SERI_ID, 0x07 , 0x0F, 0x01, 0x80, 00},
	{0x79, SERI_ID, 0x07 , 0x10, 0x01, 0x07, 00},
	{0x79, SERI_ID, 0x07 , 0x11, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x12, 0x01, 0x0F, 00},
	{0x79, SERI_ID, 0x07 , 0x13, 0x01, 0x38, 00},
	{0x79, SERI_ID, 0x07 , 0x14, 0x01, 0x04, 00},
	{0x79, SERI_ID, 0x07 , 0x15, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x16, 0x01, 0x50, 00},
	{0x79, SERI_ID, 0x07 , 0x17, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x18, 0x01, 0x10, 00},
	//# ASYM_READ_REGS_A
	{0x79, SERI_ID, 0x07 , 0x20, 0x01, 0x38, 00},
	{0x79, SERI_ID, 0x07 , 0x21, 0x01, 0x92, 00},
	{0x79, SERI_ID, 0x07 , 0x22, 0x01, 0x26, 00},
	{0x79, SERI_ID, 0x07 , 0x23, 0x01, 0x20, 00},
	{0x79, SERI_ID, 0x07 , 0x24, 0x01, 0x67, 00},
	{0x79, SERI_ID, 0x07 , 0x25, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x26, 0x01, 0x20, 00},
	{0x79, SERI_ID, 0x07 , 0x27, 0x01, 0x67, 00},
	{0x79, SERI_ID, 0x07 , 0x28, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x29, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x2A, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x2B, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x2C, 0x01, 0x14, 00},
	{0x79, SERI_ID, 0x07 , 0x2D, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x2E, 0x01, 0x84, 00},
	{0x79, SERI_ID, 0x07 , 0x2F, 0x01, 0x08, 00},
	{0x79, SERI_ID, 0x07 , 0x30, 0x01, 0x95, 00},
	{0x79, SERI_ID, 0x07 , 0x31, 0x01, 0x04, 00},
	{0x79, SERI_ID, 0x07 , 0x32, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x33, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x34, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x35, 0x01, 0x28, 00},
	{0x79, SERI_ID, 0x07 , 0x36, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x37, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x38, 0x01, 0x80, 00},
	{0x79, SERI_ID, 0x07 , 0x39, 0x01, 0x07, 00},
	{0x79, SERI_ID, 0x07 , 0x3A, 0x01, 0x18, 00},
	{0x79, SERI_ID, 0x07 , 0x3B, 0x01, 0x01, 00},
	{0x79, SERI_ID, 0x07 , 0x3C, 0x01, 0x38, 00},
	{0x79, SERI_ID, 0x07 , 0x3D, 0x01, 0x04, 00},
	{0x79, SERI_ID, 0x07 , 0x3E, 0x01, 0x10, 00},
	{0x79, SERI_ID, 0x07 , 0x3F, 0x01, 0x1F, 00},
	{0x79, SERI_ID, 0x07 , 0x40, 0x01, 0x03, 00},
	//# ASYM_READ_REGS_B
	{0x79, SERI_ID, 0x07 , 0x50, 0x01, 0xD0, 00},
	{0x79, SERI_ID, 0x07 , 0x51, 0x01, 0xB6, 00},
	{0x79, SERI_ID, 0x07 , 0x52, 0x01, 0x19, 00},
	{0x79, SERI_ID, 0x07 , 0x53, 0x01, 0xC0, 00},
	{0x79, SERI_ID, 0x07 , 0x54, 0x01, 0x44, 00},
	{0x79, SERI_ID, 0x07 , 0x55, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x56, 0x01, 0xC0, 00},
	{0x79, SERI_ID, 0x07 , 0x57, 0x01, 0x44, 00},
	{0x79, SERI_ID, 0x07 , 0x58, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x59, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x5A, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x5B, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x5C, 0x01, 0x20, 00},
	{0x79, SERI_ID, 0x07 , 0x5D, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x5E, 0x01, 0x78, 00},
	{0x79, SERI_ID, 0x07 , 0x5F, 0x01, 0x08, 00},
	{0x79, SERI_ID, 0x07 , 0x60, 0x01, 0x0E, 00},
	{0x79, SERI_ID, 0x07 , 0x61, 0x01, 0x03, 00},
	{0x79, SERI_ID, 0x07 , 0x62, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x63, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x64, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x65, 0x01, 0x40, 00},
	{0x79, SERI_ID, 0x07 , 0x66, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x67, 0x01, 0x00, 00},
	{0x79, SERI_ID, 0x07 , 0x68, 0x01, 0x80, 00},
	{0x79, SERI_ID, 0x07 , 0x69, 0x01, 0x07, 00},
	{0x79, SERI_ID, 0x07 , 0x6A, 0x01, 0x18, 00},
	{0x79, SERI_ID, 0x07 , 0x6B, 0x01, 0x01, 00},
	{0x79, SERI_ID, 0x07 , 0x6C, 0x01, 0xD0, 00},
	{0x79, SERI_ID, 0x07 , 0x6D, 0x01, 0x02, 00},
	{0x79, SERI_ID, 0x07 , 0x6E, 0x01, 0x90, 00},
	{0x79, SERI_ID, 0x07 , 0x6F, 0x01, 0x14, 00},
	{0x79, SERI_ID, 0x07 , 0x70, 0x01, 0x02, 00},
	//#Select LUT pattern/ Enable ASYM
	{0x79, SERI_ID, 0x07 , 0x19, 0x01, 0x57, 00},
	//#Video Pipe Enable
	{0x79, SERI_ID, 0x00, 0x02, 0x01, 0x33, 0},
	{0x79, SERI_ID, 0x00, 0x10, 0x01, 0x23, 100},

	{0x79, SERI_ID, 0x00, 0x10, 0x01, 0x21, 100},
	{0x79, DSER_ID, 0x00, 0x50, 0x01, 0x00, 100},
	{0x79, DSER_ID, 0x01, 0xCE, 0x01, 0x5F, 100},//5F
	{0x79, DSER_ID, 0x00, 0x01, 0x01, 0x02, 0},
	{0x79, SERI_ID, 0x00, 0x01, 0x01, 0x08, 0},


	{0x79, DSER_ID, 0x02, 0x0F, 0x01, 0x10, 0},
	{0x79, DSER_ID, 0x02, 0x10, 0x01, 0xBE, 0},
	{0x79, DSER_ID, 0x02, 0x11, 0x01, 0x18, 0},


	{0x79, SERI_ID, 0x00, 0x10, 0x01, 0x22, 100},
	{0x79, DSER_ID, 0x00, 0x50, 0x01, 0x01, 100},
	{0x79, DSER_ID, 0x00, 0x01, 0x01, 0x02, 0},

	{0x79, SERI_ID, 0x00, 0x10, 0x01, 0x23, 0},
};


//kx11 config
unsigned char reg_cmds_set720_startek[][7] = {
	{0x79, SERI_ID, 0x00, 0x02, 0x01, 0x73,  0},
	{0x79, SERI_ID, 0x00, 0x53, 0x01, 0x10,  0},
	{0x79, SERI_ID, 0x00, 0x57, 0x01, 0x20,  0},
	{0x79, SERI_ID, 0x03, 0x32, 0x01, 0x4e,  0},
	{0x79, SERI_ID, 0x03, 0x33, 0x01, 0xe4,  0},
	{0x79, SERI_ID, 0x00, 0x04, 0x01, 0xf2,  0},
	{0x79, SERI_ID, 0x03, 0x08, 0x01, 0x5c,  0},
	{0x79, SERI_ID, 0x03, 0x11, 0x01, 0x03,  0},
	{0x79, SERI_ID, 0x03, 0x31, 0x01, 0x03,  0},
	{0x79, SERI_ID, 0x03, 0x30, 0x01, 0x06,  0},
	{0x79, SERI_ID, 0x03, 0x1c, 0x01, 0x98,  0},
	{0x79, SERI_ID, 0x03, 0x21, 0x01, 0x24,  0},
	{0x79, SERI_ID, 0x03, 0x1d, 0x01, 0x98,  0},
	{0x79, SERI_ID, 0x03, 0x22, 0x01, 0x24,  0},
	{0x79, SERI_ID, 0x03, 0x26, 0x01, 0xe4,  0},
	{0x79, SERI_ID, 0x03, 0x85, 0x01, 0x40,  0},
	{0x79, SERI_ID, 0x03, 0x86, 0x01, 0x2d,  0},
	{0x79, SERI_ID, 0x03, 0x87, 0x01, 0x00,  0},
	{0x79, SERI_ID, 0x03, 0xa5, 0x01, 0x08,  0},
	{0x79, SERI_ID, 0x03, 0xa7, 0x01, 0x00,  0},
	{0x79, SERI_ID, 0x03, 0xa6, 0x01, 0x80,  0},
	{0x79, SERI_ID, 0x03, 0xa8, 0x01, 0xd0,  0},
	{0x79, SERI_ID, 0x03, 0xa9, 0x01, 0x02,  0},

	{0x79, SERI_ID, 0x03, 0xaa, 0x01, 0x2a,  0},
	{0x79, SERI_ID, 0x03, 0xac, 0x01, 0x04,  0},

	{0x79, SERI_ID, 0x03, 0xab, 0x01, 0x01,  0},
	{0x79, SERI_ID, 0x03, 0xad, 0x01, 0x00,  0},
	{0x79, SERI_ID, 0x03, 0xae, 0x01, 0x0f,  0},
	{0x79, SERI_ID, 0x03, 0xa4, 0x01, 0xc3,  0},//0xc1
	{0x79, SERI_ID, 0x03, 0x2a, 0x01, 0x07,  0},
	{0x79, SERI_ID, 0x00, 0x02, 0x01, 0x73,  0},
	{0x79, SERI_ID, 0x00, 0x10, 0x01, 0x23, 100},

	{0x79, SERI_ID, 0x00, 0x10, 0x01, 0x21, 100},
	{0x79, DSER_ID, 0x01, 0xCE, 0x01, 0x5F, 100},//5F
	{0x79, DSER_ID, 0x00, 0x01, 0x01, 0x02, 0},
	{0x79, DSER_ID, 0x00, 0x73, 0x01, 0x31, 0},
	{0x79, DSER_ID, 0x00, 0x7b, 0x01, 0x31, 0},

	{0x79, DSER_ID, 0x02, 0x10, 0x01, 0xBE, 0},
	{0x79, DSER_ID, 0x02, 0x11, 0x01, 0x18, 0},

	{0x79, SERI_ID, 0x00, 0x10, 0x01, 0x22, 100},
	{0x79, DSER_ID, 0x00, 0x50, 0x01, 0x00, 100},
	{0x79, DSER_ID, 0x01, 0xCE, 0x01, 0x5F, 100},//0x5F
	{0x79, DSER_ID, 0x00, 0x01, 0x01, 0x02, 0},
	{0x79, SERI_ID, 0x00, 0x01, 0x01, 0x08, 0},

	{0x79, DSER_ID, 0x02, 0x10, 0x01, 0xBE, 0},
	{0x79, DSER_ID, 0x02, 0x11, 0x01, 0x18, 0},

	{0x79, SERI_ID, 0x00, 0x10, 0x01, 0x23, 100},

};

//dcy11 config
unsigned char reg_cmds_set1920_startek[][7] = {
	{0x79, SERI_ID, 0x03, 0x80, 0x01, 0x0f, 100}, // serial sync mode auto ,low level active
	{0x79, SERI_ID, 0x00, 0x02, 0x01, 0x73, 0},
	{0x79, SERI_ID, 0x00, 0x53, 0x01, 0x10, 0},
	{0x79, SERI_ID, 0x00, 0x57, 0x01, 0x21, 0},
	{0x79, SERI_ID, 0x03, 0x32, 0x01, 0x4e, 0},
	{0x79, SERI_ID, 0x03, 0x33, 0x01, 0xe4, 0},
	{0x79, SERI_ID, 0x00, 0x04, 0x01, 0xf2, 0},
	{0x79, SERI_ID, 0x03, 0x08, 0x01, 0x5c, 0},
	{0x79, SERI_ID, 0x03, 0x11, 0x01, 0x03, 0},
	{0x79, SERI_ID, 0x03, 0x31, 0x01, 0x03, 0},
	{0x79, SERI_ID, 0x03, 0x30, 0x01, 0x06, 0},
	{0x79, SERI_ID, 0x03, 0x1c, 0x01, 0x98, 0},
	{0x79, SERI_ID, 0x03, 0x21, 0x01, 0x24, 0},
	{0x79, SERI_ID, 0x03, 0x1d, 0x01, 0x98, 0},
	{0x79, SERI_ID, 0x03, 0x22, 0x01, 0x24, 0},
	{0x79, SERI_ID, 0x03, 0x26, 0x01, 0xe4, 0},
	{0x79, SERI_ID, 0x03, 0x85, 0x01, 0x28, 0},
	{0x79, SERI_ID, 0x03, 0x86, 0x01, 0x05, 0},
	{0x79, SERI_ID, 0x03, 0x87, 0x01, 0x00, 0},
	{0x79, SERI_ID, 0x03, 0xa5, 0x01, 0x05, 0},
	{0x79, SERI_ID, 0x03, 0xa7, 0x01, 0x00, 0},
	{0x79, SERI_ID, 0x03, 0xa6, 0x01, 0x50, 0},
	{0x79, SERI_ID, 0x03, 0xa8, 0x01, 0x38, 0},
	{0x79, SERI_ID, 0x03, 0xa9, 0x01, 0x04, 0},

	{0x79, SERI_ID, 0x03, 0xaa, 0x01, 0x88, 0},
	{0x79, SERI_ID, 0x03, 0xac, 0x01, 0x02, 0},
	{0x79, SERI_ID, 0x03, 0xab, 0x01, 0x82, 0},

	{0x79, SERI_ID, 0x03, 0xad, 0x01, 0x00, 0},
	{0x79, SERI_ID, 0x03, 0xae, 0x01, 0x0f, 0},
	{0x79, SERI_ID, 0x03, 0xa4, 0x01, 0xc1, 0},//0xc1
	{0x79, SERI_ID, 0x03, 0x2a, 0x01, 0x07, 0},

	//#ASYM_REGS
	{0x79, SERI_ID, 0x7, 0x00, 0x01, 0x74 , 0x0},
	{0x79, SERI_ID, 0x7, 0x01, 0x01, 0x29 , 0x0},
	{0x79, SERI_ID, 0x7, 0x02, 0x01, 0x26 , 0x0},
	{0x79, SERI_ID, 0x7, 0x03, 0x01, 0x74 , 0x0},
	{0x79, SERI_ID, 0x7, 0x04, 0x01, 0x29 , 0x0},
	{0x79, SERI_ID, 0x7, 0x05, 0x01, 0x26 , 0x0},
	{0x79, SERI_ID, 0x7, 0x06, 0x01, 0xE8 , 0x0},
	{0x79, SERI_ID, 0x7, 0x07, 0x01, 0x52 , 0x0},
	{0x79, SERI_ID, 0x7, 0x08, 0x01, 0x4C , 0x0},
	{0x79, SERI_ID, 0x7, 0x09, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x0A, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x0B, 0x01, 0x80 , 0x0},
	{0x79, SERI_ID, 0x7, 0x0C, 0x01, 0x07 , 0x0},
	{0x79, SERI_ID, 0x7, 0x0D, 0x01, 0x38 , 0x0},
	{0x79, SERI_ID, 0x7, 0x0E, 0x01, 0x04 , 0x0},
	{0x79, SERI_ID, 0x7, 0x0F, 0x01, 0x80 , 0x0},
	{0x79, SERI_ID, 0x7, 0x10, 0x01, 0x07 , 0x0},
	{0x79, SERI_ID, 0x7, 0x11, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x12, 0x01, 0x0F , 0x0},
	{0x79, SERI_ID, 0x7, 0x13, 0x01, 0x38 , 0x0},
	{0x79, SERI_ID, 0x7, 0x14, 0x01, 0x04 , 0x0},
	{0x79, SERI_ID, 0x7, 0x15, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x16, 0x01, 0xd0 , 0x0},
	{0x79, SERI_ID, 0x7, 0x17, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x18, 0x01, 0xd0 , 0x0},

	//#ASYM_READ_REGS_A
	{0x79, SERI_ID, 0x7, 0x20, 0x01, 0x3C , 0x0},
	{0x79, SERI_ID, 0x7, 0x21, 0x01, 0xD0 , 0x0},
	{0x79, SERI_ID, 0x7, 0x22, 0x01, 0x25 , 0x0},
	{0x79, SERI_ID, 0x7, 0x23, 0x01, 0x9C , 0x0},
	{0x79, SERI_ID, 0x7, 0x24, 0x01, 0x2C , 0x0},
	{0x79, SERI_ID, 0x7, 0x25, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x26, 0x01, 0x9C , 0x0},
	{0x79, SERI_ID, 0x7, 0x27, 0x01, 0x2C , 0x0},
	{0x79, SERI_ID, 0x7, 0x28, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x29, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x2A, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x2B, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x2C, 0x01, 0x14 , 0x0},
	{0x79, SERI_ID, 0x7, 0x2D, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x2E, 0x01, 0xD8 , 0x0},
	{0x79, SERI_ID, 0x7, 0x2F, 0x01, 0x08 , 0x0},
	{0x79, SERI_ID, 0x7, 0x30, 0x01, 0x47 , 0x0},
	{0x79, SERI_ID, 0x7, 0x31, 0x01, 0x04 , 0x0},
	{0x79, SERI_ID, 0x7, 0x32, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x33, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x34, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x35, 0x01, 0x28 , 0x0},
	{0x79, SERI_ID, 0x7, 0x36, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x37, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x38, 0x01, 0x80 , 0x0},
	{0x79, SERI_ID, 0x7, 0x39, 0x01, 0x07 , 0x0},
	{0x79, SERI_ID, 0x7, 0x3A, 0x01, 0x6C , 0x0},
	{0x79, SERI_ID, 0x7, 0x3B, 0x01, 0x01 , 0x0},
	{0x79, SERI_ID, 0x7, 0x3C, 0x01, 0x38 , 0x0},
	{0x79, SERI_ID, 0x7, 0x3D, 0x01, 0x04 , 0x0},
	{0x79, SERI_ID, 0x7, 0x3E, 0x01, 0xAC , 0x0},
	{0x79, SERI_ID, 0x7, 0x3F, 0x01, 0x85 , 0x0},
	{0x79, SERI_ID, 0x7, 0x40, 0x01, 0x00 , 0x0},
	//#ASYM_READ_REGS_B
	{0x79, SERI_ID, 0x7, 0x50, 0x01, 0x3C , 0x0},
	{0x79, SERI_ID, 0x7, 0x51, 0x01, 0xD0 , 0x0},
	{0x79, SERI_ID, 0x7, 0x52, 0x01, 0x25 , 0x0},
	{0x79, SERI_ID, 0x7, 0x53, 0x01, 0x9C , 0x0},
	{0x79, SERI_ID, 0x7, 0x54, 0x01, 0x2C , 0x0},
	{0x79, SERI_ID, 0x7, 0x55, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x56, 0x01, 0x9C , 0x0},
	{0x79, SERI_ID, 0x7, 0x57, 0x01, 0x2C , 0x0},
	{0x79, SERI_ID, 0x7, 0x58, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x59, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x5A, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x5B, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x5C, 0x01, 0x14 , 0x0},
	{0x79, SERI_ID, 0x7, 0x5D, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x5E, 0x01, 0xD8 , 0x0},
	{0x79, SERI_ID, 0x7, 0x5F, 0x01, 0x08 , 0x0},
	{0x79, SERI_ID, 0x7, 0x60, 0x01, 0x47 , 0x0},
	{0x79, SERI_ID, 0x7, 0x61, 0x01, 0x04 , 0x0},
	{0x79, SERI_ID, 0x7, 0x62, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x63, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x64, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x65, 0x01, 0x28 , 0x0},
	{0x79, SERI_ID, 0x7, 0x66, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x67, 0x01, 0x00 , 0x0},
	{0x79, SERI_ID, 0x7, 0x68, 0x01, 0x80 , 0x0},
	{0x79, SERI_ID, 0x7, 0x69, 0x01, 0x07 , 0x0},
	{0x79, SERI_ID, 0x7, 0x6A, 0x01, 0x6C , 0x0},
	{0x79, SERI_ID, 0x7, 0x6B, 0x01, 0x01 , 0x0},
	{0x79, SERI_ID, 0x7, 0x6C, 0x01, 0x38 , 0x0},
	{0x79, SERI_ID, 0x7, 0x6D, 0x01, 0x04 , 0x0},
	{0x79, SERI_ID, 0x7, 0x6E, 0x01, 0xAC , 0x0},
	{0x79, SERI_ID, 0x7, 0x6F, 0x01, 0x85 , 0x0},
	{0x79, SERI_ID, 0x7, 0x70, 0x01, 0x00 , 0x0},
	//#Select LUT pattern/ Enable ASYM
	{0x79, SERI_ID, 0x7, 0x19, 0x01, 0x17 , 0x0},

	{0x79, SERI_ID, 0x00, 0x02, 0x01, 0x73, 0},
	{0x79, SERI_ID, 0x00, 0x10, 0x01, 0x23, 50},

	{0x79, SERI_ID, 0x00, 0x10, 0x01, 0x21, 50},
	{0x79, DSER_ID, 0x01, 0xCE, 0x01, 0x5F, 0},//5F
	{0x79, DSER_ID, 0x00, 0x01, 0x01, 0x02, 0},

	//{0x79, DSER_ID, 0x00, 0x73, 0x01, 0x31, 0},
	//{0x79, DSER_ID, 0x00, 0x7b, 0x01, 0x31, 0},

	{0x79, DSER_ID, 0x02, 0x0F, 0x01, 0x10, 0},
	{0x79, DSER_ID, 0x02, 0x10, 0x01, 0xBE, 0},
	{0x79, DSER_ID, 0x02, 0x11, 0x01, 0x18, 0},
	//{0x79, DSER_ID, 0x00, 0x00, 0x01, 0x94, 0},

	{0x79, SERI_ID, 0x00, 0x10, 0x01, 0x22, 50},

	{0x79, SERI_ID, 0x00, 0x03, 0x01, 0x50, 0},
	{0x79, SERI_ID, 0x00, 0x4F, 0x01, 0x08, 0},
	{0x79, SERI_ID, 0x05, 0x48, 0x01, 0x8B, 0},
	{0x79, SERI_ID, 0x05, 0x49, 0x01, 0x02, 0},
	{0x79, SERI_ID, 0x00, 0x4d, 0x01, 0x5e, 0},
	{0x79, DSER_ID, 0x00, 0x03, 0x01, 0x24, 0},
	{0x79, DSER_ID, 0x00, 0x4F, 0x01, 0x08, 0},
	{0x79, DSER_ID, 0x02, 0x4A, 0x01, 0x8B, 0},
	{0x79, DSER_ID, 0x02, 0x4B, 0x01, 0x02, 0},

	{0x79, DSER_ID, 0x00, 0x50, 0x01, 0x01, 0},
	{0x79, DSER_ID, 0x01, 0xCE, 0x01, 0x5F, 0},//0x5F
	{0x79, DSER_ID, 0x00, 0x01, 0x01, 0x02, 0},
	{0x79, SERI_ID, 0x00, 0x01, 0x01, 0x08, 0},

	{0x79, DSER_ID, 0x02, 0x10, 0x01, 0xBE, 0},
	{0x79, DSER_ID, 0x02, 0x11, 0x01, 0x18, 0},
	{0x79, DSER_ID, 0x02, 0x0F, 0x01, 0x10, 0},

	{0x79, SERI_ID, 0x00, 0x10, 0x01, 0x23, 0},
};

unsigned char reg_cmds_sdpic_startek[][7] = {
	{0x79, SERI_ID, 0x00, 0x10, 0x01, 0x23, 100},
	{0x79, DSER_ID, 0x02, 0x0F, 0x01, 0x10, 0},
};

unsigned char reg_cmds_refresh_startek[][7] = {
	{0x79, DSER_ID, 0x00, 0x10, 0x01, 0x31, 100},
	//{0x79, DSER_ID, 0x02, 0x0F, 0x01, 0x10, 0},
};


unsigned char reg_cmds_set_96789_startek[][7] = {
	{0x79, SERI_ID, 0x03, 0x80, 0x01, 0x0f, 0}, // serial sync mode auto ,low level active
	{0x79, DSER_ID, 0x00, 0x01, 0x01, 0x02, 0}, // rx_rate:6gbps
	{0x79, DSER_ID, 0x01, 0xCE, 0x01, 0x5F, 100},   // VESA format
	{0x79, SERI_ID, 0x00, 0x01, 0x01, 0x08, 5},   // tx_rate: 6gbps
	{0x79, SERI_ID, 0x00, 0x10, 0x01, 0x31, 100}, // reset oneshot
	{0x79, DSER_ID, 0x02, 0x0F, 0x01, 0x10, 0}, // enable back light
	{0x79, DSER_ID, 0x02, 0x10, 0x01, 0xBE, 0},
	{0x79, DSER_ID, 0x02, 0x11, 0x01, 0x18, 0},
};

unsigned char reg_cmds_set_kx11_startek[][7] = {
	{0x79, SERI_ID, 0x00, 0x01, 0x01, 0x08,  5},   // tx_rate: 6gbps
	{0x79, SERI_ID, 0x00, 0x10, 0x01, 0x31, 100}, // reset oneshot
};

static const u32 rad_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

struct rad_panel {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;

	struct gpio_desc *reset;
	struct backlight_device *backlight;

	bool prepared;
	bool enabled;
	struct uart_port *port;

	struct videomode vm;
	u32 width_mm;
	u32 height_mm;

	const char *uart_cfg;
	const char *serializer_type;

	bool suspend;
};

static int max9xxx_init(struct device *dev, struct rad_panel *panel);

static inline struct rad_panel *to_rad_panel(struct drm_panel *panel)
{
	return container_of(panel, struct rad_panel, base);
}

#if 0
static int rad_panel_push_cmd_list(struct mipi_dsi_device *dsi)
{
	size_t i;
	size_t count = ARRAY_SIZE(manufacturer_cmd_set);
	int ret = 0;

	for (i = 0; i < count; i++) {
		const struct cmd_set_entry *entry = &manufacturer_cmd_set[i];
		u8 buffer[2] = { entry->cmd, entry->param };

		ret = mipi_dsi_generic_write(dsi, &buffer, sizeof(buffer));
		if (ret < 0)
			return ret;
	}

	return ret;
};
#endif

static int color_format_from_dsi_format(enum mipi_dsi_pixel_format format)
{
	switch (format) {
	case MIPI_DSI_FMT_RGB565:
		return 0x55;
	case MIPI_DSI_FMT_RGB666:
	case MIPI_DSI_FMT_RGB666_PACKED:
		return 0x66;
	case MIPI_DSI_FMT_RGB888:
		return 0x77;
	default:
		return 0x77; /* for backward compatibility */
	}
};

static int rad_panel_prepare(struct drm_panel *panel)
{
	struct rad_panel *rad = to_rad_panel(panel);

	if (rad->prepared)
		return 0;

	if (rad->reset != NULL) {
		gpiod_set_value(rad->reset, 0);
		usleep_range(200, 500);
		gpiod_set_value(rad->reset, 1);
		usleep_range(200, 500);
	}

	rad->prepared = true;

	return 0;
}

static int rad_panel_unprepare(struct drm_panel *panel)
{
	struct rad_panel *rad = to_rad_panel(panel);
	struct device *dev = &rad->dsi->dev;

	if (!rad->prepared)
		return 0;

	if (rad->enabled) {
		DRM_DEV_ERROR(dev, "Panel still enabled!\n");
		return -EPERM;
	}

	if (rad->reset != NULL) {
		gpiod_set_value(rad->reset, 0);
		usleep_range(1500, 1700);
		gpiod_set_value(rad->reset, 1);
	}

	rad->prepared = false;

	return 0;
}

static int rad_panel_enable(struct drm_panel *panel)
{
	struct rad_panel *rad = to_rad_panel(panel);
	struct mipi_dsi_device *dsi = rad->dsi;
	struct device *dev = &dsi->dev;
	int color_format = color_format_from_dsi_format(dsi->format);
#if 0
	int ret;
#endif

	unsigned char rx_buf;
	unsigned int delay_set;
	unsigned int retry, i, j;

	if(rad->suspend) {
		max9xxx_init(dev,rad);
		rad->suspend = FALSE;
	}

	(void) color_format;
	if (rad->enabled)
		return 0;

	if (!rad->prepared) {
		DRM_DEV_ERROR(dev, "Panel not prepared!\n");
		return -EPERM;
	}

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	if (rad->serializer_type) {
		if (!(strcmp(rad->serializer_type, "max96789"))) {
			for (i = 0; i < sizeof(reg_cmds_refresh_startek)/sizeof(reg_cmds_refresh_startek[0]); i++) {
				for (retry = 0; retry < 5; retry++) {
					for (j = 0; j < 6; j++)
						(rad->port)->ops->poll_put_char(rad->port, reg_cmds_refresh_startek[i][j]);

						delay_set = reg_cmds_refresh_startek[i][6];
						if(delay_set)
							msleep(delay_set);
						rx_buf = (rad->port)->ops->poll_get_char(rad->port);
						if (rx_buf == 0xc3)
							break;
				}
				if(!delay_set)
					msleep(1);
			}
		}

		if (!(strcmp(rad->serializer_type, "max96789_1920"))) {
			for (i = 0; i < sizeof(reg_cmds_sdpic_startek)/sizeof(reg_cmds_sdpic_startek[0]); i++) {
				for (retry = 0; retry < 5; retry++) {
					for (j = 0; j < 6; j++)
						(rad->port)->ops->poll_put_char(rad->port, reg_cmds_sdpic_startek[i][j]);

					delay_set = reg_cmds_sdpic_startek[i][6];
					if(delay_set)
						msleep(delay_set);
					rx_buf = (rad->port)->ops->poll_get_char(rad->port);
					if (rx_buf == 0xc3)
						break;
				}
				if(!delay_set)
					msleep(1);
			}
		}
	}
#if 0
	ret = rad_panel_push_cmd_list(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to send MCS (%d)\n", ret);
		goto fail;
	}

	/* Exit sleep mode */
	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to exit sleep mode (%d)\n", ret);
		goto fail;
	}

	usleep_range(5000, 10000);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set display ON (%d)\n", ret);
		goto fail;
	}

	//backlight_enable(rad->backlight);
#endif

	rad->enabled = true;

	return 0;
#if 0
fail:

	return ret;
#endif
}

static int rad_panel_disable(struct drm_panel *panel)
{
	struct rad_panel *rad = to_rad_panel(panel);
	struct mipi_dsi_device *dsi = rad->dsi;
#if 0
	struct device *dev = &dsi->dev;
	int ret;
#endif

	if (!rad->enabled)
		return 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	//backlight_disable(rad->backlight);

	//usleep_range(10000, 15000);

#if 0
	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set display OFF (%d)\n", ret);
		return ret;
	}

	//usleep_range(5000, 10000);
	udelay(1000);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to enter sleep mode (%d)\n", ret);
		return ret;
	}
#endif

	rad->enabled = false;

	return 0;
}

static int rad_panel_get_modes(struct drm_panel *panel,
				struct drm_connector *connector)
{
	struct rad_panel *rad = to_rad_panel(panel);
	struct device *dev = &rad->dsi->dev;
	//struct drm_connector *connector = panel->connector;
	struct drm_display_mode *mode;
	u32 *bus_flags = &connector->display_info.bus_flags;
	int ret;

	mode = drm_mode_create(connector->dev);
	if (!mode) {
		DRM_DEV_ERROR(dev, "Failed to create display mode!\n");
		return 0;
	}

	drm_display_mode_from_videomode(&rad->vm, mode);
	mode->width_mm = rad->width_mm;
	mode->height_mm = rad->height_mm;
	connector->display_info.width_mm = rad->width_mm;
	connector->display_info.height_mm = rad->height_mm;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;


	if (rad->vm.flags & DISPLAY_FLAGS_DE_HIGH)
		*bus_flags |= DRM_BUS_FLAG_DE_HIGH;
	if (rad->vm.flags & DISPLAY_FLAGS_DE_LOW)
		*bus_flags |= DRM_BUS_FLAG_DE_LOW;
	if (rad->vm.flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_NEGEDGE;
	if (rad->vm.flags & DISPLAY_FLAGS_PIXDATA_POSEDGE)
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_POSEDGE;

	ret = drm_display_info_set_bus_formats(&connector->display_info,
			rad_bus_formats, ARRAY_SIZE(rad_bus_formats));
	if (ret)
		return ret;

	drm_mode_probed_add(connector, mode);

	return 1;
}

static int rad_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct rad_panel *rad = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	u16 brightness;
	int ret;

	if (!rad->prepared)
		return 0;

	DRM_DEV_DEBUG_DRIVER(dev, "\n");

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
	if (ret < 0)
		return ret;

	bl->props.brightness = brightness;

	return brightness & 0xff;
}

static int rad_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct rad_panel *rad = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret = 0;

	if (!rad->prepared)
		return 0;

	DRM_DEV_DEBUG_DRIVER(dev, "New brightness: %d\n", bl->props.brightness);

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness(dsi, bl->props.brightness);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct backlight_ops rad_bl_ops = {
	.update_status = rad_bl_update_status,
	.get_brightness = rad_bl_get_brightness,
};

static const struct drm_panel_funcs rad_panel_funcs = {
	.prepare = rad_panel_prepare,
	.unprepare = rad_panel_unprepare,
	.enable = rad_panel_enable,
	.disable = rad_panel_disable,
	.get_modes = rad_panel_get_modes,
};

/*
 * The clock might range from 66MHz (30Hz refresh rate)
 * to 132MHz (60Hz refresh rate)
 */
static const struct display_timing rad_default_timing = {
	.pixelclock = { 66000000, 132000000, 132000000 },
	.hactive = { 1200, 1200, 1200 },
	.hfront_porch = { 110, 110, 133 },
	.hsync_len = { 1, 1, 1 },
	.hback_porch = { 32, 32, 32 },
	.vactive = { 1920, 1920, 1920 },
	.vfront_porch = { 11, 11, 16 },
	.vsync_len = { 1, 1, 1 },
	.vback_porch = { 14, 14, 14 },
	.flags = DISPLAY_FLAGS_HSYNC_LOW |
		 DISPLAY_FLAGS_VSYNC_LOW |
		 DISPLAY_FLAGS_DE_LOW |
		 DISPLAY_FLAGS_PIXDATA_NEGEDGE,
};

static int uart_detect_deserialiser(const struct device *dev, struct uart_port *port)
{
       const static unsigned char detect_des[] = {0x79, MAX96752_UART_READ_ADDR, 0x00, 0x0D, 0x01};
       int i = 0;
       unsigned char rx_buf[2];

       for (i = 0; i < 5; i++)
            port->ops->poll_put_char(port, detect_des[i]);

       mdelay(1);
       rx_buf[0] = port->ops->poll_get_char(port);
       mdelay(1);
       rx_buf[1] = port->ops->poll_get_char(port);
       if (rx_buf[1] == 0x00) {
            dev_err(dev, "not detect desrialiser, device id = 0x%x", rx_buf[1]);
            return -1;
       }
       dev_dbg(dev, "detect desrialiser, device id = 0x%x", rx_buf[1]);
       return 0;
}

static int max9xxx_init(struct device *dev, struct rad_panel *panel)
{
	struct device_node *uart_np;
	struct platform_device *uart_pdev;
	struct uart_port *port;
	u32 i, j;
	unsigned char rx_buf;
	unsigned int delay_set;
	long baudrate = 0;
	const char *cfg;
	int retry;

	uart_np = of_parse_phandle(dev->of_node, "uart-dev", 0);
	if (!uart_np) {
		dev_err(dev, "Failed to get uart-dev \n");
		return -EINVAL;
	}

	uart_pdev = of_find_device_by_node(uart_np);
	of_node_put(uart_np);

	if (!uart_pdev) {
		dev_err(dev, "Failed to get uart pdev \n");
		return -EINVAL;
	}

	port = dw8250_get_port(uart_pdev);
	if (!port) {
		dev_err(dev, "Failed to get uart pdev \n");
		return -EINVAL;
	}
	panel->port = port;

	if (of_property_read_string(dev->of_node, "uart-config", &panel->uart_cfg)) {
		dev_err(dev, " get uart-config fail in dts\n");
		return -EINVAL;
	}

	if (of_property_read_string(dev->of_node, "serializer-type", &panel->serializer_type)) {
		dev_err(dev, " get serializer-type fail in dts\n");
		return -EINVAL;
	}

	cfg = panel->uart_cfg;

	/* Get baud_rate and set it up */
	for (i = 0; i < strlen(cfg); i++) {
		if ('0' > cfg[i] || '9' < cfg[i])
			break;
		baudrate = baudrate * 10 + cfg[i] - '0';
	}

	printk("panel baud %ld\n", baudrate);

	uart_set_options(port, NULL, baudrate, 'e', 8, 'n');

	if (!(strcmp(panel->serializer_type, "max96789"))) {

		for (i = 0; i < sizeof(reg_cmds_set_96789_startek)/sizeof(reg_cmds_set_96789_startek[0]); i++) {
			for (retry = 0; retry < 5; retry++) {
				for (j = 0; j < 6; j++)
					port->ops->poll_put_char(port, reg_cmds_set_96789_startek[i][j]);

				delay_set = reg_cmds_set_96789_startek[i][6];
				if(delay_set)
					msleep(delay_set);

				rx_buf = port->ops->poll_get_char(port);
				if (rx_buf == 0xc3)
					break;

				//dev_info(dev, "write addr:0x%02x%02x val:0x%x, read:0x%x pass\n",
				//reg_cmds_set_96789_startek[i][2], reg_cmds_set_96789_startek[i][3], reg_cmds_set_96789_startek[i][5], rx_buf);
			}
			if(!delay_set)
				msleep(1);
		}

	}


	if (!(strcmp(panel->serializer_type, "max96789_1920"))) {
		if (uart_detect_deserialiser(dev, port) && !panel->suspend )
			return 0;

		for (i = 0; i < sizeof(reg_cmds_set1920_startek)/sizeof(reg_cmds_set1920_startek[0]); i++) {
			for (retry = 0; retry < 5; retry++) {
				for (j = 0; j < 6; j++)
					port->ops->poll_put_char(port, reg_cmds_set1920_startek[i][j]);

				delay_set = reg_cmds_set1920_startek[i][6];
				if(delay_set)
					mdelay(delay_set);
				else
					mdelay(1);

				rx_buf = port->ops->poll_get_char(port);
				if (rx_buf == 0xc3)
					break;

				dev_dbg(dev, "write addr:0x%02x%02x val:0x%x, read:0x%x fail\n",
				reg_cmds_set1920_startek[i][2], reg_cmds_set1920_startek[i][3], reg_cmds_set1920_startek[i][5], rx_buf);
			}

		}
	}

	if (!(strcmp(panel->serializer_type, "max96789_720"))) {
		for (i = 0; i < sizeof(reg_cmds_set720_startek)/sizeof(reg_cmds_set720_startek[0]); i++) {
			for (retry = 0; retry < 5; retry++) {
				for (j = 0; j < 6; j++)
				port->ops->poll_put_char(port, reg_cmds_set720_startek[i][j]);

				delay_set = reg_cmds_set720_startek[i][6];
				if(delay_set)
					msleep(delay_set);

				rx_buf = port->ops->poll_get_char(port);
				if (rx_buf == 0xc3)
					break;

				//dev_info(dev, "write addr:0x%02x%02x val:0x%x, read:0x%x pass\n",
				//reg_cmds_set720_startek[i][2], reg_cmds_set720_startek[i][3], reg_cmds_set720_startek[i][5], rx_buf);
			}
			if(!delay_set)
				msleep(1);
		}
	}

	if (!(strcmp(panel->serializer_type, "max96789_asym"))) {
		for(i = 0; i < sizeof(reg_cmds_asym_startek)/sizeof(reg_cmds_asym_startek[0]); i++) {
			for (retry = 0; retry < 5; retry++) {
				for(j = 0; j < 6; j++)
					port->ops->poll_put_char(port,reg_cmds_asym_startek[i][j]);

				delay_set = reg_cmds_asym_startek[i][6];
				if(delay_set)
					msleep(delay_set);

				rx_buf = port->ops->poll_get_char(port);
				if(rx_buf == 0xc3)
					break;

				//dev_info(dev,"write addr:0x%02x%02x val:0x%x, read:0x%x fail\n",
				//reg_cmds_asym_startek[i][2], reg_cmds_asym_startek[i][3], reg_cmds_asym_startek[i][5], rx_buf);
			}
			if(!delay_set)
				msleep(1);
		}
	}


	if (!(strcmp(panel->serializer_type, "max96789_kx11"))) {
		for (i = 0; i < sizeof(reg_cmds_set_kx11_startek)/sizeof(reg_cmds_set_kx11_startek[0]); i++) {
			for (retry = 0; retry < 5; retry++) {
				for (j = 0; j < 6; j++)
				port->ops->poll_put_char(port, reg_cmds_set_kx11_startek[i][j]);

				delay_set = reg_cmds_set_kx11_startek[i][6];
				if(delay_set)
					msleep(delay_set);

				rx_buf = port->ops->poll_get_char(port);
				if (rx_buf == 0xc3)
					break;

				//dev_info(dev, "write addr:0x%02x%02x val:0x%x, read:0x%x pass\n",
				//reg_cmds_set_kx11_startek[i][2], reg_cmds_set_kx11_startek[i][3], reg_cmds_set_kx11_startek[i][5], rx_buf);
			}
			if(!delay_set)
				msleep(1);
		}
	}

return 0;
}

static int rad_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *np = dev->of_node;
	struct device_node *timings;
	struct rad_panel *panel;
#if 0
	struct backlight_properties bl_props;
#endif
	int ret;
	u32 video_mode;


	panel = devm_kzalloc(&dsi->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, panel);

	panel->dsi = dsi;

	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags =	MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO |
			   MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ret = of_property_read_u32(np, "video-mode", &video_mode);
	if (!ret) {
		switch (video_mode) {
		case 0:
			/* burst mode */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
			printk("mode_flags MIPI_DSI_MODE_VIDEO_BURST \n");
			dev_info(dev, "mode_flags MIPI_DSI_MODE_VIDEO_BURST \n");
			break;
		case 1:
			/* non-burst mode with sync event */
			break;
		case 2:
			/* non-burst mode with sync pulse */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
			break;
		default:
			dev_warn(dev, "invalid video mode %d\n", video_mode);
			break;

		}
	}

	ret = of_property_read_u32(np, "dsi-lanes", &dsi->lanes);
	if (ret < 0) {
		dev_err(dev, "Failed to get dsi-lanes property (%d)\n", ret);
		return ret;
	}

	ret = max9xxx_init(dev, panel);
	if (ret < 0) {
		dev_err(dev, "max96xx init fail (%d)\n", ret);
		//return ret;
	}

	/*
	 * 'display-timings' is optional, so verify if the node is present
	 * before calling of_get_videomode so we won't get console error
	 * messages
	 */
	timings = of_get_child_by_name(np, "display-timings");
	if (timings) {
		of_node_put(timings);
		ret = of_get_videomode(np, &panel->vm, 0);
	} else {
		videomode_from_timing(&rad_default_timing, &panel->vm);
	}

	if (ret < 0) {
		dev_err(dev, "display-timings (%d)\n", ret);
		return ret;
	}

	of_property_read_u32(np, "panel-width-mm", &panel->width_mm);
	of_property_read_u32(np, "panel-height-mm", &panel->height_mm);

	panel->suspend = FALSE;

	panel->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);

	if (IS_ERR(panel->reset))
		panel->reset = NULL;
	else
		gpiod_set_value(panel->reset, 0);

#if 0
	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = 255;
	bl_props.max_brightness = 255;

	panel->backlight = devm_backlight_device_register(
				dev, dev_name(dev),
				dev, dsi,
				&rad_bl_ops, &bl_props);
	if (IS_ERR(panel->backlight)) {
		ret = PTR_ERR(panel->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}
#endif

	drm_panel_init(&panel->base, dev, &rad_panel_funcs,
			DRM_MODE_CONNECTOR_DSI);
	//panel->base.funcs = &rad_panel_funcs;
	//panel->base.dev = dev;
	dev_set_drvdata(dev, panel);

	drm_panel_add(&panel->base);

	if (ret < 0)
		return ret;

	//usleep_range(20000, 25000);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		printk("drm_panel_remove \n");
		drm_panel_remove(&panel->base);
	}
	return ret;
}

static int rad_panel_remove(struct mipi_dsi_device *dsi)
{
	struct rad_panel *rad = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "Failed to detach from host (%d)\n",
			ret);

	drm_panel_remove(&rad->base);

	return 0;
}

static void rad_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct rad_panel *rad = mipi_dsi_get_drvdata(dsi);

	rad_panel_disable(&rad->base);
	rad_panel_unprepare(&rad->base);
}

#ifdef CONFIG_PM
static int rad_panel_suspend(struct device *dev)
{
	struct rad_panel *rad = dev_get_drvdata(dev);

	rad->suspend = TRUE;

	if (!rad->reset)
		return 0;

	devm_gpiod_put(dev, rad->reset);
	rad->reset = NULL;

	return 0;
}

static int rad_panel_resume(struct device *dev)
{
	struct rad_panel *rad = dev_get_drvdata(dev);

	if (rad->reset)
		return 0;

	rad->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(rad->reset))
		rad->reset = NULL;

	return PTR_ERR_OR_ZERO(rad->reset);
}

#endif

static const struct dev_pm_ops rad_pm_ops = {
	SET_RUNTIME_PM_OPS(rad_panel_suspend, rad_panel_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(rad_panel_suspend, rad_panel_resume)
};

static const struct of_device_id rad_of_match[] = {
	{ .compatible = "startek,kd101u", },
	{ }
};
MODULE_DEVICE_TABLE(of, rad_of_match);

static struct mipi_dsi_driver rad_panel_driver = {
	.driver = {
		.name = "panel-startek-kd101u",
		.of_match_table = rad_of_match,
		.pm	= &rad_pm_ops,
	},
	.probe = rad_panel_probe,
	.remove = rad_panel_remove,
	.shutdown = rad_panel_shutdown,
};
module_mipi_dsi_driver(rad_panel_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("startek KD101UXF");
MODULE_LICENSE("GPL v2");


