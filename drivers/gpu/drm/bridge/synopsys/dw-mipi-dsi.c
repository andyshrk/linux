// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 * Copyright (C) STMicroelectronics SA 2017
 *
 * Modified by Philippe Cornu <philippe.cornu@st.com>
 * This generic Synopsys DesignWare MIPI DSI host driver is based on the
 * Rockchip version from rockchip/dw-mipi-dsi.c with phy & bridge APIs.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/debugfs.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/mfd/syscon.h>

#include <video/mipi_display.h>

#include <drm/bridge/dw_mipi_dsi.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <linux/regmap.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#define MAX_PIPE_NUM			2
#define HWVER_131			0x31333100	/* IP version 1.31 */

#define DSI_VERSION			0x00
#define VERSION				GENMASK(31, 8)

#define DSI_PWR_UP			0x04
#define RESET				0
#define POWERUP				BIT(0)

#define DSI_CLKMGR_CFG			0x08
#define TO_CLK_DIVISION(div)		(((div) & 0xff) << 8)
#define TX_ESC_CLK_DIVISION(div)	((div) & 0xff)

#define DSI_DPI_VCID			0x0c
#define DPI_VCID(vcid)			((vcid) & 0x3)

#define DSI_DPI_COLOR_CODING		0x10
#define LOOSELY18_EN			BIT(8)
#define DPI_COLOR_CODING_16BIT_1	0x0
#define DPI_COLOR_CODING_16BIT_2	0x1
#define DPI_COLOR_CODING_16BIT_3	0x2
#define DPI_COLOR_CODING_18BIT_1	0x3
#define DPI_COLOR_CODING_18BIT_2	0x4
#define DPI_COLOR_CODING_24BIT		0x5

#define DSI_DPI_CFG_POL			0x14
#define COLORM_ACTIVE_LOW		BIT(4)
#define SHUTD_ACTIVE_LOW		BIT(3)
#define HSYNC_ACTIVE_LOW		BIT(2)
#define VSYNC_ACTIVE_LOW		BIT(1)
#define DATAEN_ACTIVE_LOW		BIT(0)

#define DSI_DPI_LP_CMD_TIM		0x18
#define OUTVACT_LPCMD_TIME(p)		(((p) & 0xff) << 16)
#define INVACT_LPCMD_TIME(p)		((p) & 0xff)

#define DSI_DBI_VCID			0x1c
#define DSI_DBI_CFG			0x20
#define DSI_DBI_PARTITIONING_EN		0x24
#define DSI_DBI_CMDSIZE			0x28

#define DSI_PCKHDL_CFG			0x2c
#define CRC_RX_EN			BIT(4)
#define ECC_RX_EN			BIT(3)
#define BTA_EN				BIT(2)
#define EOTP_RX_EN			BIT(1)
#define EOTP_TX_EN			BIT(0)

#define DSI_GEN_VCID			0x30

#define DSI_MODE_CFG			0x34
#define ENABLE_VIDEO_MODE		0
#define ENABLE_CMD_MODE			BIT(0)

#define DSI_VID_MODE_CFG		0x38
#define ENABLE_LOW_POWER		(0x3f << 8)
#define ENABLE_LOW_POWER_MASK		(0x3f << 8)
#define VID_MODE_TYPE_NON_BURST_SYNC_PULSES	0x0
#define VID_MODE_TYPE_NON_BURST_SYNC_EVENTS	0x1
#define VID_MODE_TYPE_BURST			0x2
#define VID_MODE_TYPE_MASK			0x3
#define VID_MODE_VPG_ENABLE		BIT(16)
#define VID_MODE_VPG_HORIZONTAL		BIT(24)

#define DSI_VID_PKT_SIZE		0x3c
#define VID_PKT_SIZE(p)			((p) & 0x3fff)

#define DSI_VID_NUM_CHUNKS		0x40
#define VID_NUM_CHUNKS(c)		((c) & 0x1fff)

#define DSI_VID_NULL_SIZE		0x44
#define VID_NULL_SIZE(b)		((b) & 0x1fff)

#define DSI_VID_HSA_TIME		0x48
#define DSI_VID_HBP_TIME		0x4c
#define DSI_VID_HLINE_TIME		0x50
#define DSI_VID_VSA_LINES		0x54
#define DSI_VID_VBP_LINES		0x58
#define DSI_VID_VFP_LINES		0x5c
#define DSI_VID_VACTIVE_LINES		0x60
#define DSI_EDPI_CMD_SIZE		0x64

#define DSI_CMD_MODE_CFG		0x68
#define MAX_RD_PKT_SIZE_LP		BIT(24)
#define DCS_LW_TX_LP			BIT(19)
#define DCS_SR_0P_TX_LP			BIT(18)
#define DCS_SW_1P_TX_LP			BIT(17)
#define DCS_SW_0P_TX_LP			BIT(16)
#define GEN_LW_TX_LP			BIT(14)
#define GEN_SR_2P_TX_LP			BIT(13)
#define GEN_SR_1P_TX_LP			BIT(12)
#define GEN_SR_0P_TX_LP			BIT(11)
#define GEN_SW_2P_TX_LP			BIT(10)
#define GEN_SW_1P_TX_LP			BIT(9)
#define GEN_SW_0P_TX_LP			BIT(8)
#define ACK_RQST_EN			BIT(1)
#define TEAR_FX_EN			BIT(0)

#define CMD_MODE_ALL_LP			(MAX_RD_PKT_SIZE_LP | \
					 DCS_LW_TX_LP | \
					 DCS_SR_0P_TX_LP | \
					 DCS_SW_1P_TX_LP | \
					 DCS_SW_0P_TX_LP | \
					 GEN_LW_TX_LP | \
					 GEN_SR_2P_TX_LP | \
					 GEN_SR_1P_TX_LP | \
					 GEN_SR_0P_TX_LP | \
					 GEN_SW_2P_TX_LP | \
					 GEN_SW_1P_TX_LP | \
					 GEN_SW_0P_TX_LP)

#define DSI_GEN_HDR			0x6c
#define DSI_GEN_PLD_DATA		0x70

#define DSI_CMD_PKT_STATUS		0x74
#define GEN_RD_CMD_BUSY			BIT(6)
#define GEN_PLD_R_FULL			BIT(5)
#define GEN_PLD_R_EMPTY			BIT(4)
#define GEN_PLD_W_FULL			BIT(3)
#define GEN_PLD_W_EMPTY			BIT(2)
#define GEN_CMD_FULL			BIT(1)
#define GEN_CMD_EMPTY			BIT(0)

#define DSI_TO_CNT_CFG			0x78
#define HSTX_TO_CNT(p)			(((p) & 0xffff) << 16)
#define LPRX_TO_CNT(p)			((p) & 0xffff)

#define DSI_HS_RD_TO_CNT		0x7c
#define DSI_LP_RD_TO_CNT		0x80
#define DSI_HS_WR_TO_CNT		0x84
#define DSI_LP_WR_TO_CNT		0x88
#define DSI_BTA_TO_CNT			0x8c

#define DSI_LPCLK_CTRL			0x94
#define AUTO_CLKLANE_CTRL		BIT(1)
#define PHY_TXREQUESTCLKHS		BIT(0)

#define DSI_PHY_TMR_LPCLK_CFG		0x98
#define PHY_CLKHS2LP_TIME(lbcc)		(((lbcc) & 0x3ff) << 16)
#define PHY_CLKLP2HS_TIME(lbcc)		((lbcc) & 0x3ff)

#define DSI_PHY_TMR_CFG			0x9c
#define PHY_HS2LP_TIME(lbcc)		(((lbcc) & 0xff) << 24)
#define PHY_LP2HS_TIME(lbcc)		(((lbcc) & 0xff) << 16)
#define MAX_RD_TIME(lbcc)		((lbcc) & 0x7fff)
#define PHY_HS2LP_TIME_V131(lbcc)	(((lbcc) & 0x3ff) << 16)
#define PHY_LP2HS_TIME_V131(lbcc)	((lbcc) & 0x3ff)

#define DSI_PHY_RSTZ			0xa0
#define PHY_DISFORCEPLL			0
#define PHY_ENFORCEPLL			BIT(3)
#define PHY_DISABLECLK			0
#define PHY_ENABLECLK			BIT(2)
#define PHY_RSTZ			0
#define PHY_UNRSTZ			BIT(1)
#define PHY_SHUTDOWNZ			0
#define PHY_UNSHUTDOWNZ			BIT(0)

#define DSI_PHY_IF_CFG			0xa4
#define PHY_STOP_WAIT_TIME(cycle)	(((cycle) & 0xff) << 8)
#define N_LANES(n)			(((n) - 1) & 0x3)

#define DSI_PHY_ULPS_CTRL		0xa8
#define DSI_PHY_TX_TRIGGERS		0xac

#define DSI_PHY_STATUS			0xb0
#define PHY_STOP_STATE_CLK_LANE		BIT(2)
#define PHY_LOCK			BIT(0)

#define DSI_PHY_TST_CTRL0		0xb4
#define PHY_TESTCLK			BIT(1)
#define PHY_UNTESTCLK			0
#define PHY_TESTCLR			BIT(0)
#define PHY_UNTESTCLR			0

#define DSI_PHY_TST_CTRL1		0xb8
#define PHY_TESTEN			BIT(16)
#define PHY_UNTESTEN			0
#define PHY_TESTDOUT(n)			(((n) & 0xff) << 8)
#define PHY_TESTDIN(n)			((n) & 0xff)

#define DSI_INT_ST0			0xbc
#define DSI_INT_ST1			0xc0
#define DSI_INT_MSK0			0xc4
#define DSI_INT_MSK1			0xc8

#define DSI_PHY_TMR_RD_CFG		0xf4
#define MAX_RD_TIME_V131(lbcc)		((lbcc) & 0x7fff)

#define PHY_STATUS_TIMEOUT_US		10000
#define CMD_PKT_STATUS_TIMEOUT_US	20000

#define CONFIG_EXT_BRIDGE_INSTANCE

struct dw_mipi_dsi {
	struct drm_bridge bridge;
	struct mipi_dsi_host dsi_host;
	struct drm_bridge *panel_bridge;
	struct device *dev;
	struct regmap *base;

	struct clk *pclk;
	struct clk *phy_ref;
	struct clk *phy_cfg;

	unsigned int lane_mbps; /* per lane */
	u32 channel;
	u32 lanes;
	u32 format;
	u32 lbc;
	unsigned long mode_flags;
	int pipe_id;  /* stream id input to encoder. config in dts */
	int pipe_num; /* stream number input to encoder. config in dts */

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs;

	bool vpg;
	bool vpg_horizontal;
#endif /* CONFIG_DEBUG_FS */

	struct dw_mipi_dsi *master; /* dual-dsi master ptr */
	struct dw_mipi_dsi *slave; /* dual-dsi slave ptr */

	const struct dw_mipi_dsi_plat_data *plat_data;

	unsigned long mdp_flags;

	struct drm_display_mode *mode;//use for spdic
	int split_flags ;//split view flags
};

/*
 * Check if either a link to a master or slave is present
 */
static inline bool dw_mipi_is_dual_mode(struct dw_mipi_dsi *dsi)
{
	return dsi->slave || dsi->master;
}

/*
 * The controller should generate 2 frames before
 * preparing the peripheral.
 */
static void dw_mipi_dsi_wait_for_two_frames(const struct drm_display_mode *mode)
{
	int refresh, two_frames;

	refresh = drm_mode_vrefresh(mode);
	two_frames = DIV_ROUND_UP(MSEC_PER_SEC, refresh) * 2;
	msleep(two_frames);
}

static inline struct dw_mipi_dsi *host_to_dsi(struct mipi_dsi_host *host)
{
	return container_of(host, struct dw_mipi_dsi, dsi_host);
}

static inline struct dw_mipi_dsi *bridge_to_dsi(struct drm_bridge *bridge)
{
	return container_of(bridge, struct dw_mipi_dsi, bridge);
}

static inline void dsi_write(struct dw_mipi_dsi *dsi, u32 reg, u32 val)
{
	regmap_write(dsi->base, reg, val);
}

static inline u32 dsi_read(struct dw_mipi_dsi *dsi, u32 reg)
{
	u32 val;

	regmap_read(dsi->base, reg, &val);

	return val;
}

static int dw_mipi_dsi_host_attach(struct mipi_dsi_host *host,
				   struct mipi_dsi_device *device)
{
	struct dw_mipi_dsi *dsi = host_to_dsi(host);
	const struct dw_mipi_dsi_plat_data *pdata = dsi->plat_data;
#ifndef CONFIG_EXT_BRIDGE_INSTANCE
	struct drm_bridge *bridge;
	struct drm_panel *panel;
#endif
	int ret;

	if (device->lanes > dsi->plat_data->max_data_lanes) {
		dev_err(dsi->dev, "the number of data lanes(%u) is too many\n",
			device->lanes);
		return -EINVAL;
	}

	dsi->lanes = device->lanes;
	dsi->channel = device->channel;
	dsi->format = device->format;
	dsi->mode_flags = device->mode_flags;

#ifndef CONFIG_EXT_BRIDGE_INSTANCE
	ret = drm_of_find_panel_or_bridge(host->dev->of_node, 1, 0,
					  &panel, &bridge);
	if (ret)
		return ret;

	if (panel) {
		bridge = drm_panel_bridge_add_typed(panel, DRM_MODE_CONNECTOR_DSI);
		if (IS_ERR(bridge))
			return PTR_ERR(bridge);
	}

	dsi->panel_bridge = bridge;

	drm_bridge_add(&dsi->bridge);
#endif

	if (pdata->host_ops && pdata->host_ops->attach) {
		ret = pdata->host_ops->attach(pdata->priv_data, device);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int dw_mipi_dsi_host_detach(struct mipi_dsi_host *host,
				   struct mipi_dsi_device *device)
{
	struct dw_mipi_dsi *dsi = host_to_dsi(host);
	const struct dw_mipi_dsi_plat_data *pdata = dsi->plat_data;
	int ret;

	if (pdata->host_ops && pdata->host_ops->detach) {
		ret = pdata->host_ops->detach(pdata->priv_data, device);
		if (ret < 0)
			return ret;
	}

	drm_of_panel_bridge_remove(host->dev->of_node, 1, 0);

	drm_bridge_remove(&dsi->bridge);

	return 0;
}

static void dw_mipi_message_config(struct dw_mipi_dsi *dsi,
				   const struct mipi_dsi_msg *msg)
{
	bool lpm = msg->flags & MIPI_DSI_MSG_USE_LPM;
	u32 val = 0;

	if (msg->flags & MIPI_DSI_MSG_REQ_ACK)
		val |= ACK_RQST_EN;

	if (lpm)
		val |= CMD_MODE_ALL_LP;

	//dsi_write(dsi, DSI_LPCLK_CTRL, lpm ? 0 : PHY_TXREQUESTCLKHS);
	dsi_write(dsi, DSI_LPCLK_CTRL, PHY_TXREQUESTCLKHS);//0x94
	dsi_write(dsi, DSI_CMD_MODE_CFG, val);
}

static int dw_mipi_dsi_gen_pkt_hdr_write(struct dw_mipi_dsi *dsi, u32 hdr_val)
{
	int ret;
	u32 val, mask;

	ret = regmap_read_poll_timeout(dsi->base,  DSI_CMD_PKT_STATUS,
				 val, !(val & GEN_CMD_FULL), 1000,
				 CMD_PKT_STATUS_TIMEOUT_US);
	if (ret) {
		dev_err(dsi->dev, "failed to get available command FIFO\n");
		return ret;
	}

	dsi_write(dsi, DSI_GEN_HDR, hdr_val);

	mask = GEN_CMD_EMPTY | GEN_PLD_W_EMPTY;
	ret = regmap_read_poll_timeout(dsi->base, DSI_CMD_PKT_STATUS,
			 val, (val & mask) == mask,
			 1000, CMD_PKT_STATUS_TIMEOUT_US);
	if (ret) {
		dev_err(dsi->dev, "failed to write command FIFO\n");
		return ret;
	}

	return 0;
}

static int dw_mipi_dsi_write(struct dw_mipi_dsi *dsi,
			     const struct mipi_dsi_packet *packet)
{
	const u8 *tx_buf = packet->payload;
	int len = packet->payload_length, pld_data_bytes = sizeof(u32), ret;
	__le32 word;
	u32 val;

	while (len) {
		if (len < pld_data_bytes) {
			word = 0;
			memcpy(&word, tx_buf, len);
			dsi_write(dsi, DSI_GEN_PLD_DATA, le32_to_cpu(word));
			len = 0;
		} else {
			memcpy(&word, tx_buf, pld_data_bytes);
			dsi_write(dsi, DSI_GEN_PLD_DATA, le32_to_cpu(word));
			tx_buf += pld_data_bytes;
			len -= pld_data_bytes;
		}
		ret = regmap_read_poll_timeout(dsi->base, DSI_CMD_PKT_STATUS,
					 val, !(val & GEN_PLD_W_FULL), 1000,
					 CMD_PKT_STATUS_TIMEOUT_US);
		if (ret) {
			dev_err(dsi->dev,
				"failed to get available write payload FIFO\n");
			return ret;
		}
	}

	word = 0;
	memcpy(&word, packet->header, sizeof(packet->header));
	return dw_mipi_dsi_gen_pkt_hdr_write(dsi, le32_to_cpu(word));
}

static int dw_mipi_dsi_read(struct dw_mipi_dsi *dsi,
			    const struct mipi_dsi_msg *msg)
{
	int i, j, ret, len = msg->rx_len;
	u8 *buf = msg->rx_buf;
	u32 val;

	/* Wait end of the read operation */
	ret = regmap_read_poll_timeout(dsi->base, DSI_CMD_PKT_STATUS,
				 val, !(val & GEN_RD_CMD_BUSY),
				 1000, CMD_PKT_STATUS_TIMEOUT_US);
	if (ret) {
		dev_err(dsi->dev, "Timeout during read operation\n");
		return ret;
	}

	for (i = 0; i < len; i += 4) {
		/* Read fifo must not be empty before all bytes are read */
		ret = regmap_read_poll_timeout(dsi->base, DSI_CMD_PKT_STATUS,
					 val, !(val & GEN_PLD_R_EMPTY),
					 1000, CMD_PKT_STATUS_TIMEOUT_US);
		if (ret) {
			dev_err(dsi->dev, "Read payload FIFO is empty\n");
			return ret;
		}

		val = dsi_read(dsi, DSI_GEN_PLD_DATA);
		for (j = 0; j < 4 && j + i < len; j++)
			buf[i + j] = val >> (8 * j);
	}

	return ret;
}

static ssize_t dw_mipi_dsi_host_transfer(struct mipi_dsi_host *host,
					 const struct mipi_dsi_msg *msg)
{
	struct dw_mipi_dsi *dsi = host_to_dsi(host);
	struct mipi_dsi_packet packet;
	int ret, nb_bytes;

	ret = mipi_dsi_create_packet(&packet, msg);
	if (ret) {
		dev_err(dsi->dev, "failed to create packet: %d\n", ret);
		return ret;
	}

	dw_mipi_message_config(dsi, msg);
	if (dsi->slave)
		dw_mipi_message_config(dsi->slave, msg);

	ret = dw_mipi_dsi_write(dsi, &packet);
	if (ret)
		return ret;
	if (dsi->slave) {
		ret = dw_mipi_dsi_write(dsi->slave, &packet);
		if (ret)
			return ret;
	}

	if (msg->rx_buf && msg->rx_len) {
		ret = dw_mipi_dsi_read(dsi, msg);
		if (ret)
			return ret;
		nb_bytes = msg->rx_len;
	} else {
		nb_bytes = packet.size;
	}

	return nb_bytes;
}

static const struct mipi_dsi_host_ops dw_mipi_dsi_host_ops = {
	.attach = dw_mipi_dsi_host_attach,
	.detach = dw_mipi_dsi_host_detach,
	.transfer = dw_mipi_dsi_host_transfer,
};

static void dw_mipi_dsi_video_mode_config(struct dw_mipi_dsi *dsi)
{
	u32 val;
	/*
	 * TODO dw drv improvements
	 * enabling low power is panel-dependent, we should use the
	 * panel configuration here...
	 */

	//if(dsi->mdp_flags)
		//val = ENABLE_LOW_POWER;

	val = (0xbf<<8);

	//panel mode flags chose  BURST mod for  siengine  DSI

	if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
		val |= VID_MODE_TYPE_BURST;
	else if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
		val |= VID_MODE_TYPE_NON_BURST_SYNC_PULSES;
	else
		val |= VID_MODE_TYPE_NON_BURST_SYNC_EVENTS;

#ifdef CONFIG_DEBUG_FS
	if (dsi->vpg) {
		val |= VID_MODE_VPG_ENABLE;
		val |= dsi->vpg_horizontal ? VID_MODE_VPG_HORIZONTAL : 0;
	}
#endif /* CONFIG_DEBUG_FS */

	dsi_write(dsi, DSI_VID_MODE_CFG, val);
}

static void dw_mipi_dsi_set_mode(struct dw_mipi_dsi *dsi,
				 unsigned long mode_flags)
{
	dsi_write(dsi, DSI_PWR_UP, RESET);

	if (mode_flags & MIPI_DSI_MODE_VIDEO) {
		dsi_write(dsi, DSI_MODE_CFG, ENABLE_VIDEO_MODE);
		dw_mipi_dsi_video_mode_config(dsi);
		dsi_write(dsi, DSI_LPCLK_CTRL, PHY_TXREQUESTCLKHS);
	} else {
		dsi_write(dsi, DSI_MODE_CFG, ENABLE_CMD_MODE);
	}

	dsi_write(dsi, DSI_PWR_UP, POWERUP);
}

static void dw_mipi_dsi_disable(struct dw_mipi_dsi *dsi)
{
	dsi_write(dsi, DSI_PWR_UP, RESET);
	dsi_write(dsi, DSI_PHY_RSTZ, PHY_RSTZ);
}

static void dw_mipi_dsi_init(struct dw_mipi_dsi *dsi)
{
	/*
	 * The maximum permitted escape clock is 20MHz and it is derived from
	 * lanebyteclk, which is running at "lane_mbps / 8".  Thus we want:
	 *
	 *     (lane_mbps >> 3) / esc_clk_division < 20
	 * which is:
	 *     (lane_mbps >> 3) / 20 > esc_clk_division
	 */
	u32 esc_clk_division = (dsi->lane_mbps >> 3) / 20 + 1;

	//u32 esc_clk_division = 12;//30fps sunpan use

	dsi_write(dsi, DSI_PWR_UP, RESET);

	/*
	 * TODO dw drv improvements
	 * timeout clock division should be computed with the
	 * high speed transmission counter timeout and byte lane...
	 */

	dsi_write(dsi, DSI_CLKMGR_CFG, TO_CLK_DIVISION(10) |
		  TX_ESC_CLK_DIVISION(esc_clk_division));

}

static void dw_mipi_dsi_dpi_config(struct dw_mipi_dsi *dsi,
				   const struct drm_display_mode *mode)
{
	u32 val = 0, color = 0;

	switch (dsi->format) {
	case MIPI_DSI_FMT_RGB888:
		color = DPI_COLOR_CODING_24BIT;
		break;
	case MIPI_DSI_FMT_RGB666:
		color = DPI_COLOR_CODING_18BIT_2 | LOOSELY18_EN;
		break;
	case MIPI_DSI_FMT_RGB666_PACKED:
		color = DPI_COLOR_CODING_18BIT_1;
		break;
	case MIPI_DSI_FMT_RGB565:
		color = DPI_COLOR_CODING_16BIT_1;
		break;
	}

	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		val |= VSYNC_ACTIVE_LOW;
	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		val |= HSYNC_ACTIVE_LOW;

	dsi_write(dsi, DSI_DPI_VCID, DPI_VCID(dsi->channel));
	dsi_write(dsi, DSI_DPI_COLOR_CODING, color);
	dsi_write(dsi, DSI_DPI_CFG_POL, val);
	/*
	 * TODO dw drv improvements
	 * largest packet sizes during hfp or during vsa/vpb/vfp
	 * should be computed according to byte lane, lane number and only
	 * if sending lp cmds in high speed is enable (PHY_TXREQUESTCLKHS)
	 */
	dsi_write(dsi, DSI_DPI_LP_CMD_TIM, OUTVACT_LPCMD_TIME(4)
		  | INVACT_LPCMD_TIME(4));
}

static void dw_mipi_dsi_packet_handler_config(struct dw_mipi_dsi *dsi)
{
	//dsi_write(dsi, DSI_PCKHDL_CFG, CRC_RX_EN | ECC_RX_EN );//| BTA_EN
	dsi_write(dsi, DSI_PCKHDL_CFG, CRC_RX_EN | ECC_RX_EN | EOTP_TX_EN);
}

static void dw_mipi_dsi_video_packet_config(struct dw_mipi_dsi *dsi,
					    const struct drm_display_mode *mode)
{
	struct drm_display_mode *dmode;
	/*
	 * TODO dw drv improvements
	 * only burst mode is supported here. For non-burst video modes,
	 * we should compute DSI_VID_PKT_SIZE, DSI_VCCR.NUMC &
	 * DSI_VNPCR.NPSIZE... especially because this driver supports
	 * non-burst video modes, see dw_mipi_dsi_video_mode_config()...
	 */
	if (dsi->split_flags) {
		dmode = dsi->mode;
		dsi_write(dsi, DSI_VID_PKT_SIZE,
					dw_mipi_is_dual_mode(dsi) ?
					VID_PKT_SIZE(dmode->hdisplay / 2) :
					VID_PKT_SIZE(dmode->hdisplay));
	} else {
		dsi_write(dsi, DSI_VID_PKT_SIZE,
					dw_mipi_is_dual_mode(dsi) ?
					VID_PKT_SIZE(mode->hdisplay / 2) :
					VID_PKT_SIZE(mode->hdisplay));
	}
}

static void dw_mipi_dsi_command_mode_config(struct dw_mipi_dsi *dsi)
{
	/*
	 * TODO dw drv improvements
	 * compute high speed transmission counter timeout according
	 * to the timeout clock division (TO_CLK_DIVISION) and byte lane...
	 */
	//dsi_write(dsi, DSI_TO_CNT_CFG, HSTX_TO_CNT(1000) | LPRX_TO_CNT(1000));

	dsi_write(dsi, DSI_TO_CNT_CFG, HSTX_TO_CNT(1500) | LPRX_TO_CNT(0xFFFF));
	/*
	 * TODO dw drv improvements
	 * the Bus-Turn-Around Timeout Counter should be computed
	 * according to byte lane...
	 */
	dsi_write(dsi, DSI_BTA_TO_CNT, 0xd00);
	dsi_write(dsi, DSI_MODE_CFG, ENABLE_CMD_MODE);
}

/* Get lane byte clock cycles. */
static u32 dw_mipi_dsi_get_hcomponent_lbcc(struct dw_mipi_dsi *dsi,
					   const struct drm_display_mode *mode,
					   u32 hcomponent)
{
	u32 frac, lbcc;

	//lbcc = hcomponent * (dsi->lbc / 1000);
	lbcc = hcomponent * dsi->lane_mbps * MSEC_PER_SEC / 8;

	frac = lbcc % mode->clock;
	lbcc = lbcc / mode->clock;
	if (frac)
		lbcc++;

	return lbcc;
}

static void dw_mipi_dsi_line_timer_config(struct dw_mipi_dsi *dsi,
					  const struct drm_display_mode *mode)
{
	u32 htotal, hsa, hbp, lbcc;
	struct drm_display_mode *dmode;

	if (dsi->split_flags) {
		dmode = dsi->mode;
		htotal = dmode->htotal;
		hsa = dmode->hsync_end - dmode->hsync_start;
		hbp = dmode->htotal - dmode->hsync_end;
	} else {
		htotal = mode->htotal;
		hsa = mode->hsync_end - mode->hsync_start;
		hbp = mode->htotal - mode->hsync_end;
	}

	/*
	 * TODO dw drv improvements
	 * computations below may be improved...
	 */
	 //if(dsi->mdp_flags)

	lbcc = dw_mipi_dsi_get_hcomponent_lbcc(dsi, mode, htotal);
	dsi_write(dsi, DSI_VID_HLINE_TIME, lbcc);

	lbcc = dw_mipi_dsi_get_hcomponent_lbcc(dsi, mode, hsa);
	dsi_write(dsi, DSI_VID_HSA_TIME, lbcc);

	lbcc = dw_mipi_dsi_get_hcomponent_lbcc(dsi, mode, hbp);
	dsi_write(dsi, DSI_VID_HBP_TIME, lbcc);
}

static void dw_mipi_dsi_vertical_timing_config(struct dw_mipi_dsi *dsi,
					const struct drm_display_mode *mode)
{
	u32 vactive, vsa, vfp, vbp;

	vactive = mode->vdisplay;
	vsa = mode->vsync_end - mode->vsync_start;
	vfp = mode->vsync_start - mode->vdisplay;
	vbp = mode->vtotal - mode->vsync_end;

	dsi_write(dsi, DSI_VID_VACTIVE_LINES, vactive);
	dsi_write(dsi, DSI_VID_VSA_LINES, vsa);
	dsi_write(dsi, DSI_VID_VFP_LINES, vfp);
	dsi_write(dsi, DSI_VID_VBP_LINES, vbp);
}

static void dw_mipi_dsi_dphy_timing_config(struct dw_mipi_dsi *dsi)
{
	u32 hw_version;

	struct dw_mipi_dsi_dphy_timing timing;
	const struct dw_mipi_dsi_phy_ops *phy_ops = dsi->plat_data->phy_ops;
	int ret;

	ret = phy_ops->get_timing(dsi->plat_data->priv_data,
							  dsi->lane_mbps, &timing);
	if (ret)
		DRM_DEV_ERROR(dsi->dev, "Retrieving phy timings failed\n");
	/*
	 * TODO dw drv improvements
	 * data & clock lane timers should be computed according to panel
	 * blankings and to the automatic clock lane control mode...
	 * note: DSI_PHY_TMR_CFG.MAX_RD_TIME should be in line with
	 * DSI_CMD_MODE_CFG.MAX_RD_PKT_SIZE_LP (see CMD_MODE_ALL_LP)
	 */

	hw_version = dsi_read(dsi, DSI_VERSION) & VERSION;

	if (hw_version >= HWVER_131) {
		dsi_write(dsi, DSI_PHY_TMR_CFG,
				  PHY_HS2LP_TIME_V131(timing.data_hs2lp/2) |
				  PHY_LP2HS_TIME_V131(timing.data_lp2hs/2));
		//dsi_write(dsi, DSI_PHY_TMR_RD_CFG, MAX_RD_TIME_V131(10000));
		} else {
		dsi_write(dsi, DSI_PHY_TMR_CFG,
				  PHY_HS2LP_TIME(timing.data_hs2lp/2) |
				  PHY_LP2HS_TIME(timing.data_lp2hs/2) |
				  MAX_RD_TIME(10000));
		}

		dsi_write(dsi, DSI_PHY_TMR_LPCLK_CFG,
				  PHY_CLKHS2LP_TIME(timing.clk_hs2lp) |
				  PHY_CLKLP2HS_TIME(timing.clk_lp2hs));

		DRM_DEV_DEBUG(dsi->dev, "timing.data_hs2lp %d, timing.data_lp2hs %d \n",
				timing.data_hs2lp, timing.data_lp2hs);
		DRM_DEV_DEBUG(dsi->dev, "timing.clk_hs2lp %d, timing.clk_lp2hs %d \n",
				timing.clk_hs2lp, timing.clk_lp2hs);

}

static void dw_mipi_dsi_dphy_interface_config(struct dw_mipi_dsi *dsi)
{
	/*
	 * TODO dw drv improvements
	 * stop wait time should be the maximum between host dsi
	 * and panel stop wait times
	 */
	dsi_write(dsi, DSI_PHY_IF_CFG, PHY_STOP_WAIT_TIME(20) |
		  N_LANES(dsi->lanes));
}

static void dw_mipi_dsi_dphy_init(struct dw_mipi_dsi *dsi)
{
	/* Clear PHY state */
	dsi_write(dsi, DSI_PHY_RSTZ, PHY_DISFORCEPLL | PHY_DISABLECLK
		  | PHY_RSTZ | PHY_SHUTDOWNZ);
	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_UNTESTCLR);
	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_TESTCLR);
	//udelay(1);
	dsi_write(dsi, DSI_PHY_TST_CTRL0, PHY_UNTESTCLR);

	//usleep_range(10, 20);
}

static void dw_mipi_dsi_dphy_enable(struct dw_mipi_dsi *dsi)
{
	u32 val;
	int ret;

	dsi_write(dsi, DSI_PHY_RSTZ,  PHY_ENABLECLK);

	//udelay(1);
	dsi_write(dsi, DSI_PHY_RSTZ,  PHY_ENABLECLK | PHY_UNSHUTDOWNZ);
	//udelay(1);
	dsi_write(dsi, DSI_PHY_RSTZ,  PHY_ENABLECLK | PHY_UNSHUTDOWNZ
	|PHY_UNRSTZ);
	//udelay(1);
	dsi_write(dsi, DSI_PHY_RSTZ, PHY_ENFORCEPLL | PHY_ENABLECLK |
		  PHY_UNRSTZ | PHY_UNSHUTDOWNZ);
	//udelay(5);
	/*
	 *  we can't wait phy state, so return directly.
	 *
	 */
	return;

	ret = regmap_read_poll_timeout(dsi->base, DSI_PHY_STATUS, val,
				 val & PHY_LOCK, 1000, PHY_STATUS_TIMEOUT_US);
	if (ret)
		DRM_DEBUG_DRIVER("failed to wait phy lock state\n");

	ret = regmap_read_poll_timeout(dsi->base, DSI_PHY_STATUS,
				 val, val & PHY_STOP_STATE_CLK_LANE, 1000,
				 PHY_STATUS_TIMEOUT_US);
	if (ret)
		DRM_DEBUG_DRIVER("failed to wait phy clk lane stop state\n");
}

static void dw_mipi_dsi_clear_err(struct dw_mipi_dsi *dsi)
{
	dsi_read(dsi, DSI_INT_ST0);
	dsi_read(dsi, DSI_INT_ST1);
	dsi_write(dsi, DSI_INT_MSK0, 0x3fffff); //old value 0 ,config vale sunpan vcs
	dsi_write(dsi, DSI_INT_MSK1, 0x3fffff); //0 ,sunpan vcs
}

static void dw_mipi_dsi_bridge_post_disable(struct drm_bridge *bridge)
{
	struct dw_mipi_dsi *dsi = bridge_to_dsi(bridge);
	const struct dw_mipi_dsi_phy_ops *phy_ops = dsi->plat_data->phy_ops;

	if (phy_ops->power_off)
		phy_ops->power_off(dsi->plat_data->priv_data);

	/*
	 * Switch to command mode before panel-bridge post_disable &
	 * panel unprepare.
	 * Note: panel-bridge disable & panel disable has been called
	 * before by the drm framework.
	 */
	dw_mipi_dsi_set_mode(dsi, 0);

	/*
	 * TODO Only way found to call panel-bridge post_disable &
	 * panel unprepare before the dsi "final" disable...
	 * This needs to be fixed in the drm_bridge framework and the API
	 * needs to be updated to manage our own call chains...
	 */
	if (dsi->panel_bridge->funcs->post_disable)
		dsi->panel_bridge->funcs->post_disable(dsi->panel_bridge);

	if (dsi->slave) {
		dw_mipi_dsi_disable(dsi->slave);

		if (dsi->mdp_flags) {
			clk_disable_unprepare(dsi->slave->pclk);
		}

		pm_runtime_put(dsi->slave->dev);
	}
	dw_mipi_dsi_disable(dsi);

	if (dsi->mdp_flags) {
		clk_disable_unprepare(dsi->pclk);
	}
	pm_runtime_put(dsi->dev);
}

static unsigned int dw_mipi_dsi_get_lanes(struct dw_mipi_dsi *dsi)
{
	/* this instance is the slave, so add the master's lanes */
	if (dsi->master)
		return dsi->master->lanes + dsi->lanes;

	/* this instance is the master, so add the slave's lanes */
	if (dsi->slave)
		return dsi->lanes + dsi->slave->lanes;

	/* single-dsi, so no other instance to consider */
	return dsi->lanes;
}


static void dw_mipi_dsi_mode_set(struct dw_mipi_dsi *dsi,
				 const struct drm_display_mode *mode)
{
	const struct dw_mipi_dsi_phy_ops *phy_ops = dsi->plat_data->phy_ops;
	void *priv_data = dsi->plat_data->priv_data;
	int ret;

	u32 lanes = dw_mipi_dsi_get_lanes(dsi);

	//clk_prepare_enable(dsi->pclk);

	if (dsi->mdp_flags) {
		clk_prepare_enable(dsi->pclk);
		clk_prepare_enable(dsi->phy_ref);
		clk_prepare_enable(dsi->phy_cfg);
	}

	ret = phy_ops->get_lane_mbps(priv_data, mode, dsi->mode_flags,
				     lanes, dsi->format, &dsi->lane_mbps);
	if (ret)
		DRM_DEBUG_DRIVER("Phy get_lane_mbps() failed\n");


	pm_runtime_get_sync(dsi->dev);
	dw_mipi_dsi_init(dsi);
	dw_mipi_dsi_dpi_config(dsi, mode);
	dw_mipi_dsi_packet_handler_config(dsi);
	dw_mipi_dsi_video_mode_config(dsi);
	dw_mipi_dsi_video_packet_config(dsi, mode);
	dw_mipi_dsi_command_mode_config(dsi);
	dw_mipi_dsi_line_timer_config(dsi, mode);
	dw_mipi_dsi_vertical_timing_config(dsi, mode);

	dw_mipi_dsi_dphy_init(dsi);
	dw_mipi_dsi_dphy_timing_config(dsi);
	dw_mipi_dsi_dphy_interface_config(dsi);

	dw_mipi_dsi_clear_err(dsi);

	ret = phy_ops->init(priv_data);
	if (ret)
		DRM_DEBUG_DRIVER("Phy init() failed\n");

	dw_mipi_dsi_dphy_enable(dsi);

	//phy_ops->read_config(priv_data);//test code

	dw_mipi_dsi_wait_for_two_frames(mode);

	/* Switch to cmd mode for panel-bridge pre_enable & panel prepare */
	dw_mipi_dsi_set_mode(dsi, 1);
}

static void dw_mipi_dsi_bridge_mode_set(struct drm_bridge *bridge,
					const struct drm_display_mode *mode,
					const struct drm_display_mode *adjusted_mode)
{
	struct dw_mipi_dsi *dsi = bridge_to_dsi(bridge);
	const struct drm_display_mode *tmp_mode = NULL;

	if (dsi->pipe_num == 1)
		tmp_mode = mode;
	else if ((dsi->pipe_num == MAX_PIPE_NUM) && (dsi->pipe_id == MAX_PIPE_NUM - 1))
		tmp_mode = adjusted_mode;

	if (tmp_mode) {
		dw_mipi_dsi_mode_set(dsi, tmp_mode);
		if (dsi->slave)
			dw_mipi_dsi_mode_set(dsi->slave, tmp_mode);
	}

	return ;
}

static void dw_mipi_dsi_bridge_enable(struct drm_bridge *bridge)
{
	struct dw_mipi_dsi *dsi = bridge_to_dsi(bridge);
	const struct dw_mipi_dsi_phy_ops *phy_ops = dsi->plat_data->phy_ops;

	/* Switch to video mode for panel-bridge enable & panel enable */
	dw_mipi_dsi_set_mode(dsi, MIPI_DSI_MODE_VIDEO);
	if (dsi->slave)
		dw_mipi_dsi_set_mode(dsi->slave, MIPI_DSI_MODE_VIDEO);

	if (phy_ops->power_on)
		phy_ops->power_on(dsi->plat_data->priv_data);
}

static enum drm_mode_status
dw_mipi_dsi_bridge_mode_valid(struct drm_bridge *bridge,
				const struct drm_display_info *info,
				const struct drm_display_mode *mode)
{
	struct dw_mipi_dsi *dsi = bridge_to_dsi(bridge);
	const struct dw_mipi_dsi_plat_data *pdata = dsi->plat_data;
	enum drm_mode_status mode_status = MODE_OK;

	if (pdata->mode_valid)
		mode_status = pdata->mode_valid(pdata->priv_data, mode);

	return mode_status;
}

static int dw_mipi_dsi_bridge_attach(struct drm_bridge *bridge,
					 enum drm_bridge_attach_flags flags)
{
	struct dw_mipi_dsi *dsi = bridge_to_dsi(bridge);
#ifdef CONFIG_EXT_BRIDGE_INSTANCE
	struct drm_bridge *panel_bridge;
	struct drm_panel *panel;
	int ret;
#endif

	if (!bridge->encoder) {
		DRM_ERROR("Parent encoder object not found\n");
		return -ENODEV;
	}

	/* Set the encoder type as caller does not know it */
	bridge->encoder->encoder_type = DRM_MODE_ENCODER_DSI;

#ifdef CONFIG_EXT_BRIDGE_INSTANCE
	ret = drm_of_find_panel_or_bridge(dsi->dev->of_node, 1, 0, &panel,
					&panel_bridge);
	if (ret)
		return ret;

	if (panel) {
		panel_bridge = drm_panel_bridge_add(panel);
		if (IS_ERR(panel_bridge))
			return PTR_ERR(panel_bridge);
	}

	dsi->panel_bridge = panel_bridge;

	if (!dsi->panel_bridge)
		return -EPROBE_DEFER;
#endif

	/* Attach the panel-bridge to the dsi bridge */
	return drm_bridge_attach(bridge->encoder, dsi->panel_bridge, bridge, flags);
}

static const struct drm_bridge_funcs dw_mipi_dsi_bridge_funcs = {
	.mode_set     = dw_mipi_dsi_bridge_mode_set,
	.enable	      = dw_mipi_dsi_bridge_enable,
	.post_disable = dw_mipi_dsi_bridge_post_disable,
	.mode_valid   = dw_mipi_dsi_bridge_mode_valid,
	.attach	      = dw_mipi_dsi_bridge_attach,
};

#ifdef CONFIG_DEBUG_FS

static void dw_mipi_dsi_debugfs_init(struct dw_mipi_dsi *dsi)
{
	dsi->debugfs = debugfs_create_dir(dev_name(dsi->dev), NULL);
	if (IS_ERR(dsi->debugfs)) {
		dev_err(dsi->dev, "failed to create debugfs root\n");
		return;
	}

	debugfs_create_bool("vpg", 0660, dsi->debugfs, &dsi->vpg);
	debugfs_create_bool("vpg_horizontal", 0660, dsi->debugfs,
			    &dsi->vpg_horizontal);
}

static void dw_mipi_dsi_debugfs_remove(struct dw_mipi_dsi *dsi)
{
	debugfs_remove_recursive(dsi->debugfs);
}

#else

static void dw_mipi_dsi_debugfs_init(struct dw_mipi_dsi *dsi) { }
static void dw_mipi_dsi_debugfs_remove(struct dw_mipi_dsi *dsi) { }

#endif /* CONFIG_DEBUG_FS */
static struct dw_mipi_dsi *
__dw_mipi_dsi_probe(struct platform_device *pdev,
		    const struct dw_mipi_dsi_plat_data *plat_data)
{
	struct device *dev = &pdev->dev;
	struct reset_control *apb_rst;
	struct dw_mipi_dsi *dsi;
	struct device_node *np;
	int ret;
	int pipe_id;
	int pipe_num;
	struct device_node *timings;
	struct device_node *child;
	struct videomode vmode0;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return ERR_PTR(-ENOMEM);

	dsi->dev = dev;
	dsi->plat_data = plat_data;

	if (!plat_data->phy_ops->init || !plat_data->phy_ops->get_lane_mbps) {
		DRM_ERROR("Phy not properly configured\n");
		return ERR_PTR(-ENODEV);
	}

	ret = -1;

	for_each_available_child_of_node(dev->of_node, child)
		if (of_node_cmp(child->name, "dsi-timings") == 0) {
			timings = of_get_child_by_name(child, "display-timings");
			if (timings) {
				of_node_put(timings);
				dsi->mode = kzalloc(sizeof(struct drm_display_mode), GFP_KERNEL);
				ret = of_get_videomode(child, &vmode0, 0);
				drm_display_mode_from_videomode(&vmode0, dsi->mode);
			}
		}

	if (ret) {
		dsi->split_flags = 0;
		printk("%s no use spdic\n", __func__);
	} else {
		dsi->split_flags = 1;
		printk("%s use spdic\n", __func__);
	}

	np = of_parse_phandle(dev->of_node, "dsi_common", 0);
	if (!np) {
		dev_err(dev, "%s no dsi_common in dts\n", __func__);
		return NULL;
	}

	dsi->base = syscon_node_to_regmap(np);
	if (IS_ERR(dsi->base))
		return ERR_PTR(-ENODEV);

	of_node_put(np);

	dsi->pclk = devm_clk_get(dev, "pclk");

	dsi->phy_ref = devm_clk_get(dev, "ref");
	if (IS_ERR(dsi->phy_ref)) {
		dsi->mdp_flags = 0;
	} else {
		dsi->mdp_flags = 1;
	}

	dsi->phy_cfg = devm_clk_get(dev, "phy_cfg");
	if (IS_ERR(dsi->phy_cfg)) {
		ret = PTR_ERR(dsi->phy_cfg);
		dev_err(dev, "Unable to get phy_cfg: %d\n", ret);
		return ERR_PTR(ret);
	}
	/*
	 * Note that the reset was not defined in the initial device tree, so
	 * we have to be prepared for it not being found.
	 */
	apb_rst = devm_reset_control_get_optional_exclusive(dev, "apb");
	if (IS_ERR(apb_rst)) {
		ret = PTR_ERR(apb_rst);

		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Unable to get reset control: %d\n", ret);

		return ERR_PTR(ret);
	}

	/*
	 if (apb_rst) {
		ret = clk_prepare_enable(dsi->pclk);
		if (ret) {
			dev_err(dev, "%s: Failed to enable pclk\n", __func__);
			return ERR_PTR(ret);
		}

		reset_control_assert(apb_rst);
		usleep_range(10, 20);
		reset_control_deassert(apb_rst);

		clk_disable_unprepare(dsi->pclk);
	}
	*/

	ret = of_property_read_u32(dev->of_node, "pipe_id", &pipe_id);
	if (ret) {
		dev_err(dev, "get pipe_id fail, ret %d\n", ret);
		return ERR_PTR(ret);
	}

	dsi->pipe_id = pipe_id;
	ret = of_property_read_u32(dev->of_node, "pipe_num", &pipe_num);
	if (ret) {
		dev_err(dev, "get pipe_id fail, ret %d\n", ret);
		return ERR_PTR(ret);
	}

	dsi->pipe_num = pipe_num;

	dw_mipi_dsi_debugfs_init(dsi);
	pm_runtime_enable(dev);

	dsi->dsi_host.ops = &dw_mipi_dsi_host_ops;
	dsi->dsi_host.dev = dev;
	ret = mipi_dsi_host_register(&dsi->dsi_host);
	if (ret) {
		dev_err(dev, "Failed to register MIPI host: %d\n", ret);
		pm_runtime_disable(dev);
		dw_mipi_dsi_debugfs_remove(dsi);
		return ERR_PTR(ret);
	}

	dsi->bridge.driver_private = dsi;
	dsi->bridge.funcs = &dw_mipi_dsi_bridge_funcs;
#ifdef CONFIG_OF
	dsi->bridge.of_node = pdev->dev.of_node;
#endif

#ifdef CONFIG_EXT_BRIDGE_INSTANCE
	drm_bridge_add(&dsi->bridge);
#endif

	return dsi;
}

static void __dw_mipi_dsi_remove(struct dw_mipi_dsi *dsi)
{
	mipi_dsi_host_unregister(&dsi->dsi_host);

	pm_runtime_disable(dsi->dev);
	dw_mipi_dsi_debugfs_remove(dsi);
}

void dw_mipi_dsi_set_slave(struct dw_mipi_dsi *dsi, struct dw_mipi_dsi *slave)
{
	/* introduce controllers to each other */
	dsi->slave = slave;
	dsi->slave->master = dsi;

	/* migrate settings for already attached displays */
	dsi->slave->lanes = dsi->lanes;
	dsi->slave->channel = dsi->channel;
	dsi->slave->format = dsi->format;
	dsi->slave->mode_flags = dsi->mode_flags;
}
EXPORT_SYMBOL_GPL(dw_mipi_dsi_set_slave);

/*
 * Probe/remove API, used from platforms based on the DRM bridge API.
 */
struct dw_mipi_dsi *
dw_mipi_dsi_probe(struct platform_device *pdev,
		  const struct dw_mipi_dsi_plat_data *plat_data)
{
	return __dw_mipi_dsi_probe(pdev, plat_data);
}
EXPORT_SYMBOL_GPL(dw_mipi_dsi_probe);

void dw_mipi_dsi_remove(struct dw_mipi_dsi *dsi)
{
	__dw_mipi_dsi_remove(dsi);
}
EXPORT_SYMBOL_GPL(dw_mipi_dsi_remove);

void dw_mipi_dsi_shutdown(struct dw_mipi_dsi *dsi)
{
	dw_mipi_dsi_disable(dsi);
}
EXPORT_SYMBOL_GPL(dw_mipi_dsi_shutdown);
/*
 * Bind/unbind API, used from platforms based on the component framework.
 */
int dw_mipi_dsi_bind(struct dw_mipi_dsi *dsi, struct drm_encoder *encoder)
{
	int ret;

	ret = drm_bridge_attach(encoder, &dsi->bridge, NULL, 0);
	if (ret) {
		DRM_ERROR("Failed to initialize bridge with drm\n");
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(dw_mipi_dsi_bind);

void dw_mipi_dsi_unbind(struct dw_mipi_dsi *dsi)
{
}
EXPORT_SYMBOL_GPL(dw_mipi_dsi_unbind);

MODULE_AUTHOR("Chris Zhong <zyw@rock-chips.com>");
MODULE_AUTHOR("Philippe Cornu <philippe.cornu@st.com>");
MODULE_DESCRIPTION("DW MIPI DSI host controller driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dw-mipi-dsi");
