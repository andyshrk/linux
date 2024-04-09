/*
 * Copyright (c) 2016 Synopsys, Inc.
 *
 * Synopsys DP TX Linux Software Driver and documentation (hereinafter,
 * "Software") is an Unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto. You are permitted to use and
 * redistribute this Software in source and binary forms, with or without
 * modification, provided that redistributions of source code must retain this
 * notice. You may not view, use, disclose, copy or distribute this file or
 * any information contained herein except pursuant to this license grant from
 * Synopsys. If you do not agree with this notice, including the disclaimer
 * below, then you are not authorized to use the Software.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/component.h>
#include <video/of_display_timing.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include "se_dptx.h"

#define connector_to_dptx(c) \
		container_of(c, struct dptx, connector)

#define encoder_to_dptx(c) \
		container_of(c, struct dptx, encoder)

#define get_encoder(c) \
	container_of(c, struct dptx_encoder, base)

#define get_connector(c) \
	container_of(c, struct dptx_connector, base)

#define IVI_SRC_SEL				(0x228)
#define IVI_REQ					(0x214)
#define RVC_REQ					(0x218)
#define SRC_SEL_BY_DST0			(0x44)
#define SRC_SEL_BY_DST1			(0x48)
#define SWITCH_VTOTAL			(0x21c)
#define SWITCH_HTOTAL			(0x220)

#define MDP_SWITCH_N(n)			(n)
#define MDP_MUX_N(n)			(n)

static const struct drm_display_mode se_dptx_default_mode = {
	.name = "DP_CEA_1080P",
	.clock = 148500,
	.hdisplay = 1920,
	.hsync_start = 1920 + 88,
	.hsync_end = 1920 + 88 + 44,
	.htotal = 1920 + 88 + 44 + 148,
	.vdisplay = 1080,
	.vsync_start = 1080 + 4,
	.vsync_end = 1080 + 4 + 5,
	.vtotal = 1080 + 4 + 5 + 36,
	.flags = DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC,
};


static int se_dptx_clk_enable(struct dptx *dptx)
{
	int ret;

	ret = pm_runtime_get_sync(dptx->dev);
	if (ret < 0) {
		dptx_err(dptx, "failed to get pm runtime %d\n", ret);
		goto err_pm_runtime_get;
	}

	ret = clk_prepare_enable(dptx->aux_clk);
	if (ret < 0) {
		dptx_err(dptx, "failed to enable dp aux %d\n", ret);
		goto err_aux_clk;
	}

	return 0;

err_aux_clk:
	pm_runtime_put(dptx->dev);
err_pm_runtime_get:
	return ret;
}

static void se_dptx_clk_disable(struct dptx *dptx)
{
	pm_runtime_put_sync(dptx->dev);
	clk_disable_unprepare(dptx->aux_clk);
}

static __maybe_unused int se_dptx_sub_clk_enable(struct dptx *dptx, int pipe)
{
	int ret;

	ret = clk_prepare_enable(dptx->ppls[pipe].aclk);
	if (ret < 0) {
		dptx_err(dptx, "failed to enable aclk %d\n", ret);
		goto err_aclk;
	}

	ret = clk_prepare_enable(dptx->ppls[pipe].pxlclk);
	if (ret < 0) {
		dptx_err(dptx, "failed to enable pixel clk %d\n", ret);
		goto err_pxlclk;
	}

	return 0;

err_pxlclk:
	clk_disable_unprepare(dptx->ppls[pipe].aclk);
err_aclk:
	return ret;
}

static __maybe_unused void se_dptx_sub_clk_disable(struct dptx *dptx, int pipe)
{
	clk_disable_unprepare(dptx->ppls[pipe].aclk);
	clk_disable_unprepare(dptx->ppls[pipe].pxlclk);
}

static void se_dptx_encoder_mode_set(struct drm_encoder *encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted)
{
	struct dptx_encoder *dp_encoder = get_encoder(encoder);
	struct dptx *dptx = dp_encoder->dptx;
	int pipe = dp_encoder->pipe;
	struct video_params *vparams = &dptx->ppls[pipe].vparams;
	struct drm_display_info *display_info = &dptx->ppls[pipe].conn.base.display_info;
	int bpc = display_info->bpc;

	vparams->bpc = bpc;
	vparams->pix_enc = RGB;

	memcpy(&dptx->ppls[pipe].mode, adjusted, sizeof(*mode));
	dptx_dtd_update(dptx, pipe);
}

static int se_dptx_encoder_atomic_check(struct drm_encoder *encoder,
				struct drm_crtc_state *crtc_state,
				struct drm_connector_state *conn_state)
{
	return 0;
}

static void se_dptx_config_mdp_switch(struct dptx *dptx, int pipe)
{
	u32 value;
	if (pipe == 0) {
		regmap_write(dptx->misc_base, SWITCH_VTOTAL, dptx->ppls[pipe].mode.vtotal);
		regmap_read(dptx->misc_base, IVI_REQ, &value);
		value |= (1 << MDP_SWITCH_N(1));
		regmap_write(dptx->misc_base, IVI_REQ, value);

		regmap_read(dptx->misc_base, RVC_REQ, &value);
		value &= ~(1 << MDP_SWITCH_N(1));
		regmap_write(dptx->misc_base, RVC_REQ, value);
	} else if (pipe == 2) {
		regmap_read(dptx->misc_base, IVI_SRC_SEL, &value);
		value |= (1 << MDP_MUX_N(0));
		regmap_write(dptx->misc_base, IVI_SRC_SEL, value);

		regmap_write(dptx->misc_base, SRC_SEL_BY_DST0, 0x10);

		regmap_read(dptx->misc_base, IVI_REQ, &value);
		value |= (1 << MDP_SWITCH_N(0));
		regmap_write(dptx->misc_base, IVI_REQ, value);

		regmap_read(dptx->misc_base, RVC_REQ, &value);
		value &= ~(1 << MDP_SWITCH_N(0));
		regmap_write(dptx->misc_base, RVC_REQ, value);
	} else if (pipe == 3) {
		regmap_read(dptx->misc_base, IVI_SRC_SEL, &value);
		value |= (1 << MDP_MUX_N(1));
		regmap_write(dptx->misc_base, IVI_SRC_SEL, value);

		regmap_write(dptx->misc_base, SRC_SEL_BY_DST1, 0x10);

		regmap_read(dptx->misc_base, IVI_REQ, &value);
		value |= (1 << MDP_SWITCH_N(1));
		regmap_write(dptx->misc_base, IVI_REQ, value);

		regmap_read(dptx->misc_base, RVC_REQ, &value);
		value &= ~(1 << MDP_SWITCH_N(1));
		regmap_write(dptx->misc_base, RVC_REQ, value);
	}
}

static void se_dptx_encoder_disable(struct drm_encoder *encoder)
{
	struct dptx_encoder *dp_encoder = get_encoder(encoder);
	struct dptx *dptx = dp_encoder->dptx;
	int pipe = dp_encoder->pipe;

	mutex_lock(&dptx->lock);

	/* disable all video stream */
	dptx_disable_default_video_stream(dptx, pipe);

	dptx->ppls[pipe].enabled = false;

	mutex_unlock(&dptx->lock);
}

static void se_dptx_encoder_enable(struct drm_encoder *encoder)
{
	struct dptx_encoder *dp_encoder = get_encoder(encoder);
	struct dptx *dptx = dp_encoder->dptx;
	int pipe = dp_encoder->pipe;
	u8 lanes, rate;

	mutex_lock(&dptx->lock);

	se_dptx_config_mdp_switch(dptx, pipe);

	if (atomic_read(&dptx->connect)) {
		if (!dptx_link_check_status(dptx)) {
			if (dptx->fixed_rate) {
				rate = dptx->fixed_rate;
				lanes = dptx->fixed_lanes;
			} else {
				rate = dptx->max_rate;
				lanes = dptx->max_lanes;
			}

			dptx_link_training(dptx, rate, lanes);
		}

		dptx_update_video_config(dptx, 1<<pipe);
	}
	dptx_enable_default_video_stream(dptx, pipe);

	dptx->ppls[pipe].enabled = true;

	mutex_unlock(&dptx->lock);
}

static enum drm_connector_status
se_dptx_connector_detect(struct drm_connector *connector, bool force)
{
	struct dptx_connector *dp_connector = get_connector(connector);
	struct dptx *dptx = dp_connector->dptx;
	enum drm_connector_status status = connector_status_disconnected;

	mutex_lock(&dptx->lock);
	if (atomic_read(&dptx->connect))
		status = connector_status_connected;
	mutex_unlock(&dptx->lock);

	dptx_dbg(dptx, "DP[%d] %s\n", dp_connector->base.base.id,
		(status == connector_status_connected) ? "connected" : "disconnected");

	return status;
}

static int se_dptx_connector_get_modes(struct drm_connector *connector)
{
	struct dptx_connector *dp_connector = get_connector(connector);
	struct dptx *dptx = dp_connector->dptx;
	int pipe = dp_connector->pipe;
	struct drm_display_mode *mode;
	struct edid *edid;
	int num_modes = 0;
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;

	mutex_lock(&dptx->lock);

	edid = dptx->ppls[pipe].edid4drm;
	if (edid) {
		dptx_dbg(dptx, "got edid: width[%d] x height[%d]\n",
			edid->width_cm, edid->height_cm);

		drm_connector_update_edid_property(connector, edid);
		num_modes = drm_add_edid_modes(connector, edid);
		//drm_edid_to_eld(connector, edid);
	} else {
		drm_connector_update_edid_property(connector, NULL);
	}

	if (num_modes == 0) {
		if (dptx->ppls[pipe].panel) {
			num_modes = drm_panel_get_modes(dptx->ppls[pipe].panel, connector);
		} else {
			mode = drm_mode_duplicate(connector->dev, &se_dptx_default_mode);
			if (!mode) {
				dptx_err(dptx, "failed to add mode %ux%ux@%u\n",
					dptx->ppls[pipe].mode.hdisplay, dptx->ppls[pipe].mode.vdisplay,
					drm_mode_vrefresh(&dptx->ppls[pipe].mode));
			} else {
				mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
				drm_mode_probed_add(connector, mode);
				drm_connector_list_update(connector);
				drm_mode_sort(&connector->modes);

				connector->display_info.bpc = DPTX_DEFAULT_COLOR_DEPTH;
				drm_display_info_set_bus_formats(&connector->display_info, &bus_format, 1);
				connector->display_info.bus_flags = DRM_BUS_FLAG_PIXDATA_POSEDGE;
				num_modes++;
			}
		}
	}

	mutex_unlock(&dptx->lock);
	return num_modes;
}

static int se_dptx_connector_fill_modes(struct drm_connector *connector,
				uint32_t max_width, uint32_t max_height)
{
	struct dptx_connector *dp_connector = get_connector(connector);
	struct dptx *dptx = dp_connector->dptx;
	int pipe = dp_connector->pipe;
	struct drm_display_mode *mode;
	int num_modes = 0;

	mutex_lock(&dptx->lock);
	if (!atomic_read(&dptx->connect)) {
		list_for_each_entry(mode, &connector->modes, head)
			mode->status = MODE_STALE;
		drm_mode_prune_invalid(connector->dev, &connector->modes, false);

		num_modes = drm_panel_get_modes(dptx->ppls[pipe].panel, connector);
		mutex_unlock(&dptx->lock);
		return num_modes;
	} else {
		mutex_unlock(&dptx->lock);
		return drm_helper_probe_single_connector_modes(connector, max_width, max_height);
	}
}

static struct drm_encoder *
se_dptx_connector_best_encoder(struct drm_connector *connector)
{
	struct dptx_connector *dp_connector = get_connector(connector);
	struct dptx *dptx = dp_connector->dptx;
	int pipe = dp_connector->pipe;

	return &dptx->ppls[pipe].enc.base;
}

static int se_dptx_connector_mode_valid(struct drm_connector *connector,
				struct drm_display_mode *mode)
{
	struct dptx_connector *dp_connector = get_connector(connector);
	struct dptx *dptx = dp_connector->dptx;
	struct drm_display_info *display_info = &dp_connector->base.display_info;
	u32 requested, actual, rate, sink_max, source_max = 0;
	u8 lanes, bpc;

	/* If DP is disconnected, every mode is invalid */
	if (!atomic_read(&dptx->connect))
		return MODE_BAD;

	switch (display_info->bpc) {
	case 10:
		bpc = 10;
		break;
	case 6:
		bpc = 6;
		break;
	default:
		bpc = 8;
		break;
	}

	requested = mode->clock * bpc * 3 / 1000;

	source_max = dptx->link.lanes;
	sink_max = drm_dp_max_lane_count(dptx->rx_caps);
	lanes = min(source_max, sink_max);

	source_max = dptx_bw_to_link_rate(dptx_phy_rate_to_bw(dptx->link.rate));
	sink_max = dptx_bw_to_link_rate(dptx->rx_caps[DP_MAX_LINK_RATE]);
	rate = min(source_max, sink_max);

	actual = rate * lanes / 100;

	/* efficiency is about 0.8 */
	actual = actual * 8 / 10;

	dptx_dbg(dptx, "requested=%d, actual=%d, clock=%d, bpc=%d, rate=%d, lanes=%d\n",
			requested, actual, mode->clock, bpc, rate, lanes);

	if (requested > actual)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static const struct drm_connector_funcs se_dptx_atomic_connector_funcs = {
	.detect = se_dptx_connector_detect,
	.destroy = drm_connector_cleanup,
	.fill_modes = se_dptx_connector_fill_modes,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static struct drm_connector_helper_funcs se_dptx_connector_helper_funcs = {
	.get_modes = se_dptx_connector_get_modes,
	.best_encoder = se_dptx_connector_best_encoder,
	.mode_valid = se_dptx_connector_mode_valid,
};

static const struct drm_encoder_helper_funcs se_dptx_encoder_helper_funcs = {
	.mode_set = se_dptx_encoder_mode_set,
	.atomic_check = se_dptx_encoder_atomic_check,
	.enable = se_dptx_encoder_enable,
	.disable = se_dptx_encoder_disable,
};

static const struct drm_encoder_funcs se_dptx_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int se_dptx_comp_bind(struct device *dev,
				struct device *master, void *data)
{
	struct dptx *dptx = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct dptx_encoder *encoder;
	struct dptx_connector *connector;
	struct drm_panel *panel;
	struct drm_display_mode *scan;
	struct dptx_pipeline *ppl;
	int ret;
	int possible_crtc;
	int i = 0, j = 0;
	int encoder_type;
	int pipe;
	int encoder_id;
	int conn_id;
	int start_id, end_id;
	int modes_num;

	dptx->drm_dev = drm_dev;
	possible_crtc = drm_of_find_possible_crtcs(drm_dev, dev->of_node);
	encoder_type = (dptx->total_streams > 1) ? DRM_MODE_ENCODER_DPMST : DRM_MODE_ENCODER_TMDS;

	if (dptx->dpu_used[1] && !dptx->bind_flag[1]) {
		dptx->bind_flag[1] = true;
		start_id = 2;
		end_id = 3;
	} else if (dptx->dpu_used[0] && !dptx->bind_flag[0]) {
		dptx->bind_flag[0] = true;
		start_id = 0;
		end_id = 1;
	} else {
		return -EINVAL;
	}

	pm_runtime_enable(dev);

	for (i = 0; i < dptx->total_streams; i++) {
		pipe = dptx->stream_id[i];
		ppl = &dptx->ppls[pipe];

		if ((pipe < start_id) || (pipe > end_id))
			continue;

		if (possible_crtc & (1 << j)) {
			encoder = &ppl->enc;
			connector = &ppl->conn;

			encoder->base.possible_crtcs = (1 << j);
			++j;
			encoder->dptx = dptx;
			encoder->pipe = pipe;

			ret = drm_encoder_init(drm_dev, &encoder->base, &se_dptx_encoder_funcs,
					encoder_type, "DP-stream-%d", j);
			if (ret) {
				dptx_err(dptx, "failed to initialize encoder with drm %d\n", ret);
				return ret;
			}

			drm_encoder_helper_add(&encoder->base, &se_dptx_encoder_helper_funcs);

			connector->base.polled = DRM_CONNECTOR_POLL_HPD;
			connector->base.dpms = DRM_MODE_DPMS_OFF;
			connector->pipe = pipe;
			connector->dptx = dptx;

			ret = drm_connector_init(drm_dev, &connector->base,
						&se_dptx_atomic_connector_funcs,
						DRM_MODE_CONNECTOR_DisplayPort);
			if (ret) {
				dptx_err(dptx, "failed to initialize connector with drm %d\n", ret);
				drm_encoder_cleanup(&encoder->base);
				return ret;
			}

			drm_connector_helper_add(&connector->base,
						&se_dptx_connector_helper_funcs);

			ret = drm_connector_attach_encoder(&connector->base,
								&encoder->base);
			if (ret) {
				dptx_err(dptx, "failed to attach connector and encoder %d\n", ret);
				drm_connector_cleanup(&connector->base);
				drm_encoder_cleanup(&encoder->base);
				return ret;
			}

			ret = drm_of_find_panel_or_bridge(dptx->dev->of_node,
						DPTX_MAX_STREAM_NUM + i, 0, &panel, NULL);
			if (ret) {
				dptx_err(dptx, "failed to find panel %d\n", ret);
				drm_connector_cleanup(&connector->base);
				drm_encoder_cleanup(&encoder->base);
				return ret;
			}

			if (panel) {
				ppl->panel = panel;

				/* get panel mode */
				mutex_lock(&drm_dev->mode_config.mutex);
				modes_num = drm_panel_get_modes(panel, &connector->base);
				if (modes_num > 0) {
					list_for_each_entry(scan, &connector->base.modes, head) {
						if ((scan->type & DRM_MODE_TYPE_PREFERRED)) {
							ppl->mode = *scan;
							break;
						}
					}
				} else {
					/* get mode from dts */
					//TODO
				}
				mutex_unlock(&drm_dev->mode_config.mutex);

				ppl->vparams.bpc = COLOR_DEPTH_8;
				ppl->vparams.pix_enc = RGB;
				dptx_dtd_update(dptx, pipe);
			} else {
				/* get mode from EDID */
				ppl->panel = NULL;
			}

			encoder_id = encoder->base.base.id;
			conn_id = connector->base.base.id;
			ppl->bind = true;
			dptx_dbg(dptx, "%s %s stream:%d crtc:%d encoder_id:%d, conn_id:%d\n",
					__func__, drm_dev->unique, pipe,
					encoder->base.possible_crtcs, encoder_id, conn_id);
		} else {
			break;
		}
	}

	if ((!dptx->dpu_used[1] && dptx->bind_flag[0]) ||
		(dptx->dpu_used[1] && dptx->bind_flag[1])) {
		dptx_core_init(dptx);

		if (dptx->no_hpd_irq) {
			atomic_set(&dptx->connect, 1);

			ret = dptx_handle_hotplug(dptx);
			if (ret) {
				atomic_set(&dptx->connect, 0);
				return ret;
			}

			drm_kms_helper_hotplug_event(dptx->drm_dev);
		}
	}

	return 0;
}

static void se_dptx_comp_unbind(struct device *dev, struct device *master, void *data)
{
	struct dptx *dptx = dev_get_drvdata(dev);
	int i = 0;
	int pipe;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	int name_len;
	int dpu_id;
	struct drm_device *drm_dev = data;
	int start_id, end_id;

	name_len = strlen(drm_dev->unique);
	dpu_id = drm_dev->unique[name_len - 1] - '0';
	dptx_dbg(dptx, "%s name:%s id:%d\n", __func__, drm_dev->unique, dpu_id);

	if (dpu_id == 1) {
		start_id = 0;
		end_id = 1;
	} else {
		start_id = 2;
		end_id = 3;
	}

	for (i = 0; i < dptx->total_streams; i++) {
		pipe = dptx->stream_id[i];

		if ((pipe < start_id) || (pipe > end_id))
			continue;

		encoder = &dptx->ppls[pipe].enc.base;
		connector = &dptx->ppls[pipe].conn.base;

		se_dptx_encoder_disable(encoder);
		encoder->funcs->destroy(encoder);
		dptx->ppls[pipe].panel = NULL;
		connector->funcs->destroy(connector);
		dptx->ppls[pipe].enabled = false;
		dptx->ppls[pipe].status = 0;
		dptx->ppls[pipe].bind = false;

		if (dptx->ppls[pipe].edid4drm) {
			kfree(dptx->ppls[pipe].edid4drm);
			dptx->ppls[pipe].edid4drm = NULL;
		}
	}

	pm_runtime_disable(dev);
}

static const struct component_ops se_dptx_component_ops = {
	.bind = se_dptx_comp_bind,
	.unbind = se_dptx_comp_unbind,
};

static int se_dptx_parse_ppl_dt(struct dptx *dptx, struct device_node *np)
{
	struct dptx_pipeline *ppl;
	struct clk *clk;
	u32 stream_id;
	int ret = 0;

	ret = of_property_read_u32(np, "stream-id", &stream_id);
	if (ret) {
		dptx_err(dptx, "failed to get stream id %d\n", ret);
		return -EINVAL;
	}

	if (stream_id >= DPTX_MAX_STREAM_NUM)
		return -EINVAL;

	ppl = &dptx->ppls[stream_id];

	clk = of_clk_get_by_name(np, "aclk");
	if (IS_ERR(clk)) {
		dptx_err(dptx, "failed to get aclk for pipeline %d\n", stream_id);
		return PTR_ERR(clk);
	}
	ppl->aclk = clk;

	clk = of_clk_get_by_name(np, "pxclk");
	if (IS_ERR(clk)) {
		dptx_err(dptx, "failed to get pxclk for pipeline %d\n", stream_id);
		return PTR_ERR(clk);
	}
	ppl->pxlclk = clk;

	ppl->of_node = np;
	ppl->status = 1;
	ppl->bind = false;
	dptx->stream_id[dptx->total_streams] = stream_id;
	dptx->dpu_used[stream_id >> 1] = true;

	dptx->total_streams++;
	ppl->dptx = dptx;

	dptx_dbg(dptx, "%s pipe %d mode:%s\n", __func__, stream_id, ppl->mode.name);

	return 0;
}

static int se_dptx_parse_dt(struct dptx *dptx)
{
	int ret;
	struct device *dev = dptx->dev;
	struct device_node *child, *np = dev->of_node;
	struct device_node *misc_np;
	struct property *prop;
	u32 fixed_lanes;
	u32 fixed_rate;

	dptx->apb_clk = devm_clk_get(dev, "pclk");
	if (IS_ERR(dptx->apb_clk)) {
		ret = PTR_ERR(dptx->apb_clk);
		dptx_err(dptx, "failed to get pclk %d\n", ret);
		return ret;
	}

	dptx->aux_clk = devm_clk_get(dev, "dp_aux16mhz_clk");
	if (IS_ERR(dptx->aux_clk)) {
		ret = PTR_ERR(dptx->aux_clk);
		dptx_err(dptx, "failed to get aux clk %d\n", ret);
		return ret;
	}

	misc_np = of_parse_phandle(dev->of_node, "misc-syscon", 0);
	if (!misc_np) {
		dptx_err(dptx, "failed to get misc-syscon %d\n", ret);
		return -EINVAL;
	}

	dptx->display_timings_fixed = false;
	if (of_property_read_bool(np, "display-timings-fixed"))
		dptx->display_timings_fixed = true;

	dptx->no_hpd_irq = false;
	if (of_property_read_bool(np, "no-hpd-irq"))
		dptx->no_hpd_irq = true;

	dptx->ssc_en = true;
	if (of_property_read_bool(np, "ssc-disabled"))
		dptx->ssc_en = false;

	prop = of_find_property(dev->of_node, "fixed-lanes", NULL);
	if (prop) {
		ret = of_property_read_u32(dev->of_node, "fixed-lanes", &fixed_lanes);
		if (ret) {
			dptx_err(dptx, "failed to get fix lanes: %d\n", ret);
			return ret;
		}
		dptx->fixed_lanes = (u8)fixed_lanes;

		ret = of_property_read_u32(dev->of_node, "fixed-rate", &fixed_rate);
		if (ret) {
			dptx_err(dptx, "failed to get fix rate: %d\n", ret);
			return ret;
		}
		dptx->fixed_rate = (u8)fixed_rate;

		dptx_dbg(dptx, "%s fixed lanes & rate: %d %d\n",
			__func__, dptx->fixed_lanes, dptx->fixed_rate);
	}

	dptx->misc_base = syscon_node_to_regmap(misc_np);
	of_node_put(misc_np);

	//parse pipeline
	for_each_available_child_of_node(np, child) {
		if (of_node_cmp(child->name, "pipeline") == 0) {
			ret = se_dptx_parse_ppl_dt(dptx, child);
			if (ret) {
				dptx_err(dptx, "failed to parse pipeline %d\n", ret);
				of_node_put(child);
				break;
			}
		}
	}

	dptx->streams = dptx->total_streams;

	if (dptx->total_streams == 1 && !dptx->ppls[0].status) {
		dptx_err(dptx, "SST should use stream id 0\n");
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
int se_dptx_suspend(struct device *dev)
{
	struct dptx *dptx = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&dptx->lock);
	if (atomic_read(&dptx->connect)) {
		ret = dptx_handle_hotunplug(dptx);
		atomic_set(&dptx->connect, 0);
	}
	se_dptx_clk_disable(dptx);
	dptx->suspended = true;
	mutex_unlock(&dptx->lock);

	return ret;
}

int se_dptx_resume(struct device *dev)
{
	struct dptx *dptx = dev_get_drvdata(dev);

	mutex_lock(&dptx->lock);
	se_dptx_clk_enable(dptx);
	dptx->suspended = false;
	dptx_core_init(dptx);
	mutex_unlock(&dptx->lock);

	return 0;
}
#endif

static int se_dptx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dptx *dptx;
	struct resource *res;
	int ret = 0;
	u32 i = 0;

	dptx = devm_kzalloc(dev, sizeof(*dptx), GFP_KERNEL);
	if (!dptx)
		return -ENOMEM;

	dptx->dev = dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dptx_err(dptx, "no memory resource\n");
		return -ENODEV;
	}

	dptx->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(dptx->base)) {
		dptx_err(dptx, "failed to map memory resource\n");
		return PTR_ERR(dptx->base);
	}

	ret = se_dptx_parse_dt(dptx);
	if (ret)
		return ret;

	pm_runtime_enable(dev);
	se_dptx_clk_enable(dptx);

	if (!dptx_check_dptx_id(dptx)) {
		dptx_err(dptx, "dptx id not match to 0x%04x:0x%04x\n",
			DPTX_ID_DEVICE_ID, DPTX_ID_VENDOR_ID);
		goto probe_fail;
	}

	dptx->irq = platform_get_irq(pdev, 0);
	if (dptx->irq < 0) {
		dptx_err(dptx, "no irq\n");
		goto probe_fail;
	}

	dptx->ssc_en = false;
	dptx->fec = false;
	dptx->dsc = false;
	dptx->automated_test = false;
	dptx->multipixel = DPTX_MP_SINGLE_PIXEL;
	dptx->link.lanes = 0;
	dptx->link.rate = 0;

	dptx->display_timings_fixed = true;

	mutex_init(&dptx->lock);

	for (i = 0; i < DPTX_MAX_STREAM_NUM; i++)
		dptx_video_params_reset(dptx, i);

	atomic_set(&dptx->hpd, 0);
	atomic_set(&dptx->connect, 0);
	atomic_set(&dptx->sink_request, 0);

	dptx->max_rate = DPTX_DEFAULT_LINK_RATE;
	dptx->max_lanes = DPTX_DEFAULT_LINK_LANES;
	dptx->edid = kzalloc(DPTX_DEFAULT_EDID_BUFLEN, GFP_KERNEL);

	dev_set_drvdata(dev, dptx);

	dptx_global_intr_dis(dptx);
	ret = devm_request_threaded_irq(dev,
					dptx->irq,
					dptx_irq,
					dptx_threaded_irq,
					IRQF_SHARED | IRQ_LEVEL,
					"se_dptx",
					dptx);
	if (ret) {
		dptx_err(dptx, "failed to request for irq %d\n", dptx->irq);
		goto probe_fail;
	}

	dptx_debugfs_init(dptx);

	for (i = 0; i < MAX_DPU_NUM; i++) {
		if (dptx->dpu_used[i]) {
			ret = component_add(dev, &se_dptx_component_ops);
			if (ret) {
				dptx_err(dptx, "failed to register component %d\n", ret);
				goto probe_fail;
			}
		}
	}

	return 0;

probe_fail:
	dptx_debugfs_exit(dptx);
	return ret;
}

static int se_dptx_remove(struct platform_device *pdev)
{
	struct dptx *dptx = platform_get_drvdata(pdev);

	kfree(dptx->edid);
	dptx_core_deinit(dptx);
	se_dptx_clk_disable(dptx);
	dptx_debugfs_exit(dptx);

	return 0;
}

static void se_dptx_shutdown(struct platform_device *pdev)
{
	struct dptx *dptx = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < DPTX_MAX_STREAM_NUM; i++)
		dptx_disable_default_video_stream(dptx, i);
}

static const struct of_device_id se_dptx_dt_ids[] = {
	{
	 .compatible = "siengine,se1000-dptx",
	},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, se_dptx_dt_ids);

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops se_dptx_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(se_dptx_suspend, se_dptx_resume)
};
#endif

struct platform_driver se_dptx_driver = {
	.probe		= se_dptx_probe,
	.remove		= se_dptx_remove,
	.shutdown	= se_dptx_shutdown,
	.driver		= {
		.name	= "siengine-dptx",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(se_dptx_dt_ids),
	#ifdef CONFIG_PM_SLEEP
		.pm = &se_dptx_pm_ops,
	#endif
	},
};

module_platform_driver(se_dptx_driver);

MODULE_AUTHOR("Zhiqiang Zhang <zhiqiang.zhang@siengine.com>");
MODULE_AUTHOR("Synopsys, Inc");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("Synopsys DesignWare DisplayPort TX Example Driver");
