// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019 Siengine
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_panel.h>
#include <video/of_display_timing.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <linux/string.h>


struct se_panel {
	struct drm_panel base;
	bool prepared;
	bool enabled;

	struct videomode vm;
	u32 width_mm;
	u32 height_mm;
};

struct reg_cmds {
	unsigned char frame_sync;
	unsigned char chip_id;
	unsigned char addr_h_byte;
	unsigned char addr_l_byte;
	unsigned char length;
	unsigned char value;
	unsigned int msleep;
};

struct init_info {
	char *name;
	const struct reg_cmds *cmds;
	int nr_cmds;
	bool backlight;
};

#define MAX96745_UART_READ_ADDR		0xC5
#define MAX96745_UART_WRITE_ADDR	0xC4
#define MAX96752_UART_READ_ADDR		0x91
#define MAX96752_UART_WRITE_ADDR	0x90

#define SERI_ID MAX96745_UART_WRITE_ADDR
#define DSER_ID MAX96752_UART_WRITE_ADDR

const static struct reg_cmds reg_cmds_set_96745_cc11[] = {
	{0x79, DSER_ID, 0x00, 0x01, 0x01, 0x02,  10},   // rx_rate:6gbps
	{0x79, DSER_ID, 0x01, 0xCE, 0x01, 0x5F,  0},    // VESA format
	{0x79, SERI_ID, 0x00, 0x28, 0x01, 0x88,  0},    // linkA: tx_rate: 6gbps
	{0x79, SERI_ID, 0x00, 0x32, 0x01, 0x88,  0},    // linkB: tx_rate: 6gbps
	{0x79, SERI_ID, 0x00, 0x29, 0x01, 0x02,  10},   // linkA: reset oneshot
	{0x79, SERI_ID, 0x00, 0x33, 0x01, 0x02,  10},   // linkB: reset oneshot
	{0x79, SERI_ID, 0x00, 0x76, 0x01, 0x98,  0},    //Remote control channel disabled
	{0x79, SERI_ID, 0x00, 0x86, 0x01, 0x98,  0},    //Remote control channel disabled
	{0x79, SERI_ID, 0x00, 0x9A, 0x01, 0x01,  0},
	{0x79, SERI_ID, 0x01, 0x00, 0x01, 0x65,  0},    //select the link A and link B that video port connects to
	{0x79, SERI_ID, 0x70, 0x00, 0x01, 0x00,  0},    //DPRX disable - HPD = 0
	{0x79, SERI_ID, 0x70, 0x19, 0x01, 0x00,  5},    //MST capablity disabled
	{0x79, SERI_ID, 0x70, 0x74, 0x01, 0x0A,  0},    //Max link rate, 0x0A = 2.7Gbps, 0x14 = 5.4Gbps, 0x1E = 8.1Gbps
	{0x79, SERI_ID, 0x70, 0x70, 0x01, 0x04,  0},    //Max lane count = 4 lanes
	{0x79, SERI_ID, 0x70, 0x00, 0x01, 0x01,  10},   //DPRX enabled - HPD active
	{0x79, SERI_ID, 0x00, 0x05, 0x01, 0xCF,  0},    //Enabled LOCK/ERR output
	{0x79, SERI_ID, 0x00, 0x1E, 0x01, 0xF8,  0},
	{0x79, SERI_ID, 0x00, 0x20, 0x01, 0x30,  0},
	{0x79, SERI_ID, 0x00, 0x1A, 0x01, 0x00,  0},
	{0x79, SERI_ID, 0x00, 0x1C, 0x01, 0x00,  0},
	{0x79, SERI_ID, 0x00, 0x76, 0x01, 0x00,  0},    //Remote control channel enabled
	{0x79, SERI_ID, 0x00, 0x86, 0x01, 0x00,  0},    //Remote control channel enabled
	{0x79, DSER_ID, 0x02, 0x0F, 0x01, 0x10,  0},    // enable back light
	{0x79, DSER_ID, 0x02, 0x10, 0x01, 0xBE,  0},
	{0x79, DSER_ID, 0x02, 0x11, 0x01, 0x18,  0},
};

const static struct reg_cmds reg_cmds_set_96745_cc11_mst[] = {
	{0x79, SERI_ID, 0x00, 0x86, 0x01, 0x98,  0},    // linkB: Remote control channel disabled
	{0x79, DSER_ID, 0x00, 0x01, 0x01, 0x02,  10},   // linkA: rx_rate:6gbps
	{0x79, DSER_ID, 0x01, 0xCE, 0x01, 0x5F,  0},    // linkA: VESA format
	{0x79, SERI_ID, 0x00, 0x86, 0x01, 0x00,  0},    // linkB: Remote control channel enabled
	{0x79, SERI_ID, 0x00, 0x76, 0x01, 0x98,  0},    // linkA: Remote control channel disabled
	{0x79, DSER_ID, 0x00, 0x01, 0x01, 0x02,  10},   // linkB: rx_rate:6gbps
	{0x79, DSER_ID, 0x01, 0xCE, 0x01, 0x5F,  0},    // linkB: VESA format
	{0x79, SERI_ID, 0x00, 0x76, 0x01, 0x00,  0},    // linkA: Remote control channel enabled
	{0x79, SERI_ID, 0x00, 0x28, 0x01, 0x88,  0},    // linkA: tx_rate: 6gbps
	{0x79, SERI_ID, 0x00, 0x32, 0x01, 0x88,  0},    // linkB: tx_rate: 6gbps
	{0x79, SERI_ID, 0x00, 0x29, 0x01, 0x02,  10},   // linkA: reset oneshot
	{0x79, SERI_ID, 0x00, 0x33, 0x01, 0x02,  10},   // linkB: reset oneshot
	{0x79, SERI_ID, 0x00, 0x9A, 0x01, 0x01,  0},    // disable i2c over aux channel
	{0x79, SERI_ID, 0x64, 0x21, 0x01, 0x0F,  0},    // Set pclks to run continuously
	{0x79, SERI_ID, 0x7F, 0x11, 0x01, 0x05,  0},    // Enable MST mode on GM03 - DEBUG
	{0x79, SERI_ID, 0x79, 0x04, 0x01, 0x01,  0},    // Set video payload ID 1 for video output port 0 on GM03
	{0x79, SERI_ID, 0x79, 0x08, 0x01, 0x02,  0},    // Set video payload ID 2 for video output port 1 on GM03
	{0x79, SERI_ID, 0x64, 0x20, 0x01, 0x10,  0},    // Turn off video
	{0x79, SERI_ID, 0x7A, 0x14, 0x01, 0x00,  0},    // Disable MST_VS0_DTG_ENABLE
	{0x79, SERI_ID, 0x7B, 0x14, 0x01, 0x00,  0},    // Disable MST_VS1_DTG_ENABLE
	{0x79, SERI_ID, 0x70, 0x54, 0x01, 0x01,  50},  // Reset DPRX core (VIDEO_INPUT_RESET)
	{0x79, SERI_ID, 0x70, 0x74, 0x01, 0x0A,  0},    // Max link rate, 0x0A = 2.7Gbps, 0x14 = 5.4Gbps, 0x1E = 8.1Gbps
	{0x79, SERI_ID, 0x70, 0x70, 0x01, 0x04,  0},    // Max lane count = 4 lanes
	{0x79, SERI_ID, 0x01, 0x00, 0x01, 0x61,  0},    // VID_LINK_SEL_X, Connect to Link A
	{0x79, SERI_ID, 0x01, 0x10, 0x01, 0x63,  0},    // VID_LINK_SEL_Y, Connect to Link B
	/* Disable MSA reset */
	{0x79, SERI_ID, 0x7A, 0x18, 0x01, 0x05,  0},
	{0x79, SERI_ID, 0x7B, 0x18, 0x01, 0x05,  0},
	{0x79, SERI_ID, 0x7C, 0x18, 0x01, 0x05,  0},
	{0x79, SERI_ID, 0x7D, 0x18, 0x01, 0x05,  0},
	/* Adjust VS0_DMA_HSYNC */
	{0x79, SERI_ID, 0x7A, 0x28, 0x01, 0xFF,  0},
	{0x79, SERI_ID, 0x7A, 0x2A, 0x01, 0xFF,  0},
	/* Adjust VS0_DMA_VSYNC */
	{0x79, SERI_ID, 0x7A, 0x24, 0x01, 0xFF,  0},
	{0x79, SERI_ID, 0x7A, 0x27, 0x01, 0x0F,  0},
	/* Adjust VS1_DMA_HSYNC */
	{0x79, SERI_ID, 0x7B, 0x28, 0x01, 0xFF,  0},
	{0x79, SERI_ID, 0x7B, 0x2A, 0x01, 0xFF,  0},
	/* Adjust VS1_DMA_VSYNC */
	{0x79, SERI_ID, 0x7B, 0x24, 0x01, 0xFF,  0},
	{0x79, SERI_ID, 0x7B, 0x27, 0x01, 0x0F,  0},
	{0x79, SERI_ID, 0x7A, 0x14, 0x01, 0x01,  0},     // Enable MST_VS0_DTG_ENABLE
	{0x79, SERI_ID, 0x7B, 0x14, 0x01, 0x01,  0},     // Enable MST_VS1_DTG_ENABLE
	{0x79, SERI_ID, 0x7B, 0x00, 0x01, 0x01,  0},     // Enable  VS1
	{0x79, SERI_ID, 0x64, 0x20, 0x01, 0x13,  0},     // Turn on video
	{0x79, SERI_ID, 0x64, 0x20, 0x01, 0x10,  0},     // Turn off video
	{0x79, SERI_ID, 0x64, 0x20, 0x01, 0x13,  0},     // Turn on video
	{0x79, SERI_ID, 0x00, 0x76, 0x01, 0x98,  0},     // linkA: Remote control channel disabled
	{0x79, DSER_ID, 0x00, 0x50, 0x01, 0x01,  0},     // config Link_B max96752 filter streamID = 1
	{0x79, DSER_ID, 0x02, 0x0F, 0x01, 0x10,  0},     // linkB: enable back light
	{0x79, DSER_ID, 0x02, 0x10, 0x01, 0xBE,  0},
	{0x79, DSER_ID, 0x02, 0x11, 0x01, 0x18,  0},
	{0x79, SERI_ID, 0x00, 0x89, 0x01, 0x01,  0},     // enable passthrough channel 1 on link B
	{0x79, SERI_ID, 0x00, 0x96, 0x01, 0x00,  0},     // set UART for passthrough channel 1
	{0x79, SERI_ID, 0x00, 0x9B, 0x01, 0x02,  0},     // connect passthrough channel 1 to GMSL link PT1
	{0x79, DSER_ID, 0x00, 0x03, 0x01, 0x24,  0},     // linkB: enable passthrough UART1 on 96752 side
	{0x79, DSER_ID, 0x00, 0x4F, 0x01, 0x08,  0},     // linkB: Use custom UART bit rate in pass-through UART Channel 1
	{0x79, DSER_ID, 0x02, 0x4A, 0x01, 0x8B,  0},     // linkB: Lower byte of custom UART bit length for pass-through UART Channel 1
	{0x79, DSER_ID, 0x02, 0x4B, 0x01, 0x02,  0},     // linkB: Upper bits of custom UART bit length for pass-through UART Channel 1
	{0x79, SERI_ID, 0x00, 0x76, 0x01, 0x00,  0},     // linkA: Remote control channel enabled
	{0x79, DSER_ID, 0x02, 0x0F, 0x01, 0x10,  0},     // linkA: enable back light
	{0x79, DSER_ID, 0x02, 0x10, 0x01, 0xBE,  0},
	{0x79, DSER_ID, 0x02, 0x11, 0x01, 0x18,  0},
};

const static struct reg_cmds reg_cmds_set_96745_kx11[] = {
	{0x79, SERI_ID, 0x00, 0x28, 0x01, 0x88,  0},    // linkA: tx_rate: 6gbps
	{0x79, SERI_ID, 0x00, 0x32, 0x01, 0x88,  0},    // linkB: tx_rate: 6gbps
	{0x79, SERI_ID, 0x00, 0x29, 0x01, 0x02,  10},   // linkA: reset oneshot
	{0x79, SERI_ID, 0x00, 0x33, 0x01, 0x02,  10},   // linkB: reset oneshot
	{0x79, SERI_ID, 0x00, 0x76, 0x01, 0x98,  0},    //Remote control channel disabled
	{0x79, SERI_ID, 0x00, 0x86, 0x01, 0x98,  0},    //Remote control channel disabled
	{0x79, SERI_ID, 0x00, 0x9A, 0x01, 0x01,  0},
	{0x79, SERI_ID, 0x01, 0x00, 0x01, 0x65,  0},    //select the link A and link B that video port connects to
	{0x79, SERI_ID, 0x70, 0x00, 0x01, 0x00,  0},    //DPRX disable - HPD = 0
	{0x79, SERI_ID, 0x70, 0x19, 0x01, 0x00,  0},    //MST capablity disabled
	{0x79, SERI_ID, 0x70, 0x74, 0x01, 0x0A,  0},    //Max link rate, 0x0A = 2.7Gbps, 0x14 = 5.4Gbps, 0x1E = 8.1Gbps
	{0x79, SERI_ID, 0x70, 0x70, 0x01, 0x04,  0},    //Max lane count = 4 lanes
	{0x79, SERI_ID, 0x70, 0x00, 0x01, 0x01,  100},  //DPRX enabled - HPD active
	{0x79, SERI_ID, 0x00, 0x05, 0x01, 0xCF,  0},    //Enabled LOCK/ERR output
	{0x79, SERI_ID, 0x00, 0x1E, 0x01, 0xF8,  0},
	{0x79, SERI_ID, 0x00, 0x20, 0x01, 0x30,  0},
	{0x79, SERI_ID, 0x00, 0x1A, 0x01, 0x00,  0},
	{0x79, SERI_ID, 0x00, 0x1C, 0x01, 0x00,  0},
	{0x79, SERI_ID, 0x00, 0x76, 0x01, 0x00,  0},    //Remote control channel enabled
	{0x79, SERI_ID, 0x00, 0x86, 0x01, 0x00,  0}     //Remote control channel enabled
};

const static struct reg_cmds reg_cmds_set_96745_kx11_mst[] = {
	{0x79, SERI_ID, 0x00, 0x28, 0x01, 0x88,  0},    // linkA: tx_rate: 6gbps
	{0x79, SERI_ID, 0x00, 0x32, 0x01, 0x88,  0},    // linkB: tx_rate: 6gbps
	{0x79, SERI_ID, 0x00, 0x29, 0x01, 0x02,  10},   // linkA: reset oneshot
	{0x79, SERI_ID, 0x00, 0x33, 0x01, 0x02,  10},   // linkB: reset oneshot
	{0x79, SERI_ID, 0x00, 0x9A, 0x01, 0x01,  0},    // disable i2c over aux channel
	{0x79, SERI_ID, 0x64, 0x21, 0x01, 0x0F,  0},    // Set pclks to run continuously
	{0x79, SERI_ID, 0x7F, 0x11, 0x01, 0x05,  0},    // Enable MST mode on GM03 - DEBUG
	{0x79, SERI_ID, 0x79, 0x04, 0x01, 0x01,  0},    // Set video payload ID 1 for video output port 0 on GM03
	{0x79, SERI_ID, 0x79, 0x08, 0x01, 0x02,  0},    // Set video payload ID 2 for video output port 1 on GM03
	{0x79, SERI_ID, 0x64, 0x20, 0x01, 0x10,  0},    // Turn off video
	{0x79, SERI_ID, 0x7A, 0x14, 0x01, 0x00,  0},    // Disable MST_VS0_DTG_ENABLE
	{0x79, SERI_ID, 0x7B, 0x14, 0x01, 0x00,  0},    // Disable MST_VS1_DTG_ENABLE
	{0x79, SERI_ID, 0x70, 0x54, 0x01, 0x01,  50},  // Reset DPRX core (VIDEO_INPUT_RESET)
	{0x79, SERI_ID, 0x70, 0x74, 0x01, 0x0A,  0},    // Max link rate, 0x0A = 2.7Gbps, 0x14 = 5.4Gbps, 0x1E = 8.1Gbps
	{0x79, SERI_ID, 0x70, 0x70, 0x01, 0x04,  0},    // Max lane count = 4 lanes
	{0x79, SERI_ID, 0x01, 0x00, 0x01, 0x61,  0},    // VID_LINK_SEL_X, Connect to Link A
	{0x79, SERI_ID, 0x01, 0x10, 0x01, 0x63,  0},    // VID_LINK_SEL_Y, Connect to Link B
	/* Disable MSA reset */
	{0x79, SERI_ID, 0x7A, 0x18, 0x01, 0x05,  0},
	{0x79, SERI_ID, 0x7B, 0x18, 0x01, 0x05,  0},
	{0x79, SERI_ID, 0x7C, 0x18, 0x01, 0x05,  0},
	{0x79, SERI_ID, 0x7D, 0x18, 0x01, 0x05,  0},
	/* Adjust VS0_DMA_HSYNC */
	{0x79, SERI_ID, 0x7A, 0x28, 0x01, 0xFF,  0},
	{0x79, SERI_ID, 0x7A, 0x2A, 0x01, 0xFF,  0},
	/* Adjust VS0_DMA_VSYNC */
	{0x79, SERI_ID, 0x7A, 0x24, 0x01, 0xFF,  0},
	{0x79, SERI_ID, 0x7A, 0x27, 0x01, 0x0F,  0},
	/* Adjust VS1_DMA_HSYNC */
	{0x79, SERI_ID, 0x7B, 0x28, 0x01, 0xFF,  0},
	{0x79, SERI_ID, 0x7B, 0x2A, 0x01, 0xFF,  0},
	/* Adjust VS1_DMA_VSYNC */
	{0x79, SERI_ID, 0x7B, 0x24, 0x01, 0xFF,  0},
	{0x79, SERI_ID, 0x7B, 0x27, 0x01, 0x0F,  0},
	{0x79, SERI_ID, 0x7A, 0x14, 0x01, 0x01,  0},     // Enable MST_VS0_DTG_ENABLE
	{0x79, SERI_ID, 0x7B, 0x14, 0x01, 0x01,  0},     // Enable MST_VS1_DTG_ENABLE
	{0x79, SERI_ID, 0x7B, 0x00, 0x01, 0x01,  0},     // Enable  VS1
	{0x79, SERI_ID, 0x64, 0x20, 0x01, 0x13,  0},     // Turn on video
	{0x79, SERI_ID, 0x64, 0x20, 0x01, 0x10,  0},     // Turn off video
	{0x79, SERI_ID, 0x64, 0x20, 0x01, 0x13,  0},     // Turn on video
	{0x79, SERI_ID, 0x00, 0x28, 0x01, 0x08,  0},     // disable Link_A
	{0x79, DSER_ID, 0x00, 0x50, 0x01, 0x01,  0},     // config Link_B max96752 filter streamID = 1
	{0x79, SERI_ID, 0x00, 0x28, 0x01, 0x88,  0},     // enable Link_A
	{0x79, SERI_ID, 0x00, 0x86, 0x01, 0x98,  0},     //linkB: Remote control channel disabled
};

struct init_info info[] = {
	{
		.name = "max96745-mst-kx11",
		.cmds = reg_cmds_set_96745_kx11_mst,
		.nr_cmds = ARRAY_SIZE(reg_cmds_set_96745_kx11_mst),
		.backlight = true,
	},

	{
		.name = "max96745-kx11",
		.cmds = reg_cmds_set_96745_kx11,
		.nr_cmds = ARRAY_SIZE(reg_cmds_set_96745_kx11),
		.backlight = true,
	},

	{
		.name = "max96745-mst-cc11",
		.cmds = reg_cmds_set_96745_cc11_mst,
		.nr_cmds = ARRAY_SIZE(reg_cmds_set_96745_cc11_mst),
		.backlight = false,
	},

	{
		.name = "max96745-cc11",
		.cmds = reg_cmds_set_96745_cc11,
		.nr_cmds = ARRAY_SIZE(reg_cmds_set_96745_cc11),
		.backlight = false,
	},
};

static const struct display_timing se_default_timing = {
	.pixelclock = { 148500000, 148500000, 148500000 },
	.hactive = { 1920, 1920, 1920 },
	.hfront_porch = { 88, 88, 88 },
	.hsync_len = { 44, 44, 44 },
	.hback_porch = { 148, 148, 148 },
	.vactive = { 1080, 1080, 1080 },
	.vfront_porch = { 4, 4, 4 },
	.vsync_len = { 5, 5, 5 },
	.vback_porch = { 40, 40, 40 },
	.flags = DISPLAY_FLAGS_HSYNC_LOW |
		 DISPLAY_FLAGS_VSYNC_LOW,
};


static inline struct se_panel *to_se_panel(struct drm_panel *panel)
{
	return container_of(panel, struct se_panel, base);
}

static int se_panel_disable(struct drm_panel *panel)
{
	struct se_panel *p = to_se_panel(panel);

	if (!p->enabled)
		return 0;

	p->enabled = false;

	return 0;
}

static int se_panel_unprepare(struct drm_panel *panel)
{
	struct se_panel *p = to_se_panel(panel);

	if (!p->prepared)
		return 0;

	p->prepared = false;

	return 0;
}

static int se_panel_prepare(struct drm_panel *panel)
{
	struct se_panel *p = to_se_panel(panel);

	if (p->prepared)
		return 0;

	p->prepared = true;

	return 0;
}

static int se_panel_enable(struct drm_panel *panel)
{
	struct se_panel *p = to_se_panel(panel);

	if (p->enabled)
		return 0;

	p->enabled = true;

	return 0;
}

static int se_panel_get_modes(struct drm_panel *panel,
				struct drm_connector *connector)
{
	struct se_panel *p = to_se_panel(panel);
	struct device *dev = panel->dev;
	struct drm_display_mode *mode;
	u32 *bus_flags = &connector->display_info.bus_flags;
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;

	mode = drm_mode_create(connector->dev);
	if (!mode) {
		dev_err(dev, "failed to create display mode\n");
		return 0;
	}

	drm_display_mode_from_videomode(&p->vm, mode);
	mode->width_mm = p->width_mm;
	mode->height_mm = p->height_mm;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);
	drm_connector_list_update(connector);
	drm_mode_sort(&connector->modes);

	if (p->vm.flags & DISPLAY_FLAGS_DE_HIGH)
		*bus_flags |= DRM_BUS_FLAG_DE_HIGH;
	if (p->vm.flags & DISPLAY_FLAGS_DE_LOW)
		*bus_flags |= DRM_BUS_FLAG_DE_LOW;
	if (p->vm.flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_NEGEDGE;
	if (p->vm.flags & DISPLAY_FLAGS_PIXDATA_POSEDGE)
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_POSEDGE;

	connector->display_info.width_mm = p->width_mm;
	connector->display_info.height_mm = p->height_mm;
	connector->display_info.bpc = 8;
	drm_display_info_set_bus_formats(&connector->display_info, &bus_format, 1);

	return 1;
}

static const struct drm_panel_funcs se_panel_funcs = {
	.disable = se_panel_disable,
	.unprepare = se_panel_unprepare,
	.prepare = se_panel_prepare,
	.enable = se_panel_enable,
	.get_modes = se_panel_get_modes,
};

static const struct of_device_id platform_of_match[] = {
	{ .compatible = "siengine,dp-panel", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, platform_of_match);

/* enable kx11 backlight */
static void enable_backlight(const struct device *dev, struct uart_port *port) {
	int i;
	unsigned char backlight_cmd[] = {0x79, 0x6c, 0x00, 0x00,
				0x08, 0x01, 0x00, 0x01, 0x00,
				0xAC, 0x9A, 0xC9, 0x31};
	int len = ARRAY_SIZE(backlight_cmd);

	for (i = 0; i < len; i++)
		port->ops->poll_put_char(port, backlight_cmd[i]);

	dev_info(dev, "enable kx11 backlight done!");
}

static void panel_power_on(const struct device *dev, struct uart_port *port)
{
	int i;
	unsigned char power_on_cmd[] = {0x79, 0x55, 0xaa, 0x00,
					0x00, 0x05, 0x01, 0x00,
					0x01, 0x00, 0x7f};
	int len = ARRAY_SIZE(power_on_cmd);

	for (i = 0; i < len; i++)
		port->ops->poll_put_char(port, power_on_cmd[i]);

	dev_info(dev, "panel power on done!");
}

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

static int max967xx_write_regs(const struct device *dev, struct uart_port *port,
			const struct reg_cmds *cmds, size_t len)
{
	int i, j;
	int retry;
	int m_sleep;
	unsigned char rx_buf;
	unsigned char *cmd_ch;

	if (!cmds || !len)
		return 0;

	if (uart_detect_deserialiser(dev, port))
		return 0;

	for (i = 0; i < len; i++) {
		cmd_ch = (unsigned char *) &cmds[i];
		for (retry = 0; retry < 5; retry++) {

			for (j = 0; j < 6; j++)
				port->ops->poll_put_char(port, cmd_ch[j]);

			m_sleep = cmds[i].msleep;
			if (m_sleep)
				mdelay(m_sleep);
			else
				mdelay(1);

			rx_buf = port->ops->poll_get_char(port);

			if (0xc3 == rx_buf)
				break;

		}
		if(retry == 5) {
			dev_err(dev, "%s: write addr:0x%x 0x%x val:0x%x, fail \n", __func__,
				cmds[i].addr_h_byte, cmds[i].addr_l_byte, cmds[i].value);
		}
	}

	return 0;
}

extern struct uart_port *dw8250_get_port(struct platform_device *pdev);
static int initialized = 0;

static int max96745_init(struct device *dev)
{
	struct device_node *uart_np;
	struct platform_device *uart_pdev;
	struct uart_port *port;
	const char *uart_cfg;
	const char *serializer_type;
	long baudrate = 0;
	const char *cfg;
	int ret, i;

	struct init_info *found_info = NULL;

	/* workaround: only need to init once */
	if (initialized++)
		return 0;

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
		dev_err(dev, "Failed to get uart port \n");
		return -EINVAL;
	}

	if (of_property_read_string(dev->of_node, "uart-config", &uart_cfg)) {
		dev_err(dev, "get uart-config fail in dts \n");
		return -EINVAL;
	}

	if (of_property_read_string(dev->of_node, "serializer-type", &serializer_type)) {
		dev_err(dev, " get serializer-type fail in dts\n");
		return -EINVAL;
	}

	cfg = uart_cfg;

	/* Get baud_rate and set it up */
	for (i = 0; i < strlen(cfg); i++) {
		if ('0'> cfg[i] || '9' < cfg[i])
			break;
		baudrate = baudrate * 10 + cfg[i] - '0';
	}

	dev_info(dev, "panel baud %ld \n", baudrate);

	uart_set_options(port, NULL, baudrate,'e', 8, 'n');

	for (i = 0; i < ARRAY_SIZE(info); i++) {
		struct init_info *pos = &info[i];

		if (!(strcmp(serializer_type, pos->name))) {
			found_info = pos;
			break;
		}
	}

	if (found_info) {
		ret = max967xx_write_regs(dev, port, found_info->cmds,
				found_info->nr_cmds);
		if (ret) {
			dev_err(dev, "%s reg_cmds_set fail, ret:%d\n", __func__, ret);
			return -EINVAL;
		}

		panel_power_on(dev, port);

		if (found_info->backlight)
			enable_backlight(dev, port);
	}


	return 0;
}

static int se_panel_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *timings;
	struct se_panel *panel;
	const char *serializer_type;
	char str[] = "max96745";
	int ret = 0;

	panel = devm_kzalloc(dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	panel->enabled = false;
	panel->prepared = false;

	if (of_property_read_string(np, "serializer-type", &serializer_type)) {
		dev_err(dev, " get serializer-type fail in dts\n");
		return -EINVAL;
	}

	if (strstr(serializer_type, str) != NULL) {
		ret = max96745_init(dev);
		if (ret < 0)
			dev_err(dev, "fail to init max96745 (%d)\n", ret);
	}

	/*
	 * 'display-timings' is optional, so verify if the node is present
	 * before calling of_get_videomode so we won't get console error
	 * messages
	 */
	timings = of_get_child_by_name(np, "display-timings");
	if (timings) {
		of_node_put(timings);
		ret = of_get_videomode(np, &panel->vm, OF_USE_NATIVE_MODE);
	} else {
		videomode_from_timing(&se_default_timing, &panel->vm);
	}

	if (ret < 0)
	{
		dev_err(dev, "display-timings (%d)\n", ret);
		return ret;
	}

	of_property_read_u32(np, "panel-width-mm", &panel->width_mm);
	of_property_read_u32(np, "panel-height-mm", &panel->height_mm);

	drm_panel_init(&panel->base, dev, &se_panel_funcs, DRM_MODE_CONNECTOR_DisplayPort);

	drm_panel_add(&panel->base);

	dev_set_drvdata(dev, panel);

	return 0;
}

static int se_panel_remove(struct platform_device *pdev)
{
	struct se_panel *panel = dev_get_drvdata(&pdev->dev);

	drm_panel_remove(&panel->base);

	se_panel_disable(&panel->base);
	se_panel_unprepare(&panel->base);

	return 0;
}

static void se_panel_shutdown(struct platform_device *pdev)
{
	struct se_panel *panel = dev_get_drvdata(&pdev->dev);

	se_panel_disable(&panel->base);
	se_panel_unprepare(&panel->base);
}

#ifdef CONFIG_PM_SLEEP
static int se_panel_resume(struct device *dev)
{
	struct device_node *np = dev->of_node;
	const char *serializer_type;
	char str[] = "max96745";
	int ret = 0;

	dev_dbg(dev, " se_panel_resume... \n");
	if (of_property_read_string(np, "serializer-type", &serializer_type)) {
		dev_err(dev, " get serializer-type fail in dts\n");
		return -EINVAL;
	}

	if (strstr(serializer_type, str) != NULL) {
		ret = max96745_init(dev);
		if (ret < 0)
			dev_err(dev, "fail to init max96745 (%d)\n", ret);
	}

	mdelay(100);

	return 0;

}

static int se_panel_suspend(struct device *dev)
{
	dev_dbg(dev, " se_panel_suspend... \n");
	initialized = 0;
	return 0;
}

static const struct dev_pm_ops se_panel_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(se_panel_suspend, se_panel_resume)
};
#endif /* CONFIG_PM_SLEEP */

static struct platform_driver se_panel_platform_driver = {
	.driver = {
		.name = "panel-se",
		.of_match_table = platform_of_match,
		#ifdef CONFIG_PM_SLEEP
		.pm = &se_panel_pm,
		#endif
	},
	.probe = se_panel_probe,
	.remove = se_panel_remove,
	.shutdown = se_panel_shutdown,
};

module_platform_driver(se_panel_platform_driver);


MODULE_AUTHOR("Siengine");
MODULE_DESCRIPTION("SE1000 Panel");
MODULE_LICENSE("GPL v2");

