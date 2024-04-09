// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) COPYRIGHT 2021 SiEngine Limited.
 *
 */

#include <linux/of_device.h>
#include <linux/component.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_modes.h>
#include <drm/drm_edid.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_print.h>

#define VIRTUAL_DISPLAY_MAX_STREAM_NUM 2

struct virtual_display_pipeline {
	struct drm_encoder enc;
	struct drm_connector conn;
	struct drm_display_mode *mode;	/*virtual_display_mode, not an array*/
	u32 width_mm;
	u32 height_mm;
};

struct virtual_display{
	struct device *dev;
	struct drm_device *drm_dev;
	struct device_node *of_node; /* pipeline dt node */
	struct virtual_display_pipeline pipe[VIRTUAL_DISPLAY_MAX_STREAM_NUM];
	u8 pipe_num;
};

static int se_add_virtual_mode(struct virtual_display_pipeline *pipe)
{
	struct drm_display_mode *mode;
	mode = drm_mode_duplicate(pipe->conn.dev, pipe->mode);
	mode->width_mm = pipe->width_mm;
	mode->height_mm = pipe->height_mm;
	mode->type |= DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(&pipe->conn, mode);

	return 1;
}

static int virtual_connector_get_modes(struct drm_connector *connector)
{
	struct virtual_display_pipeline *pipe;
	int count,ret;
	u32 rad_bus_formats[] = {
		MEDIA_BUS_FMT_RGB888_1X24,
		MEDIA_BUS_FMT_RGB666_1X18,
		MEDIA_BUS_FMT_RGB565_1X16,
	};

	pipe = container_of(connector, struct virtual_display_pipeline, conn);
	count = se_add_virtual_mode(pipe);

	drm_connector_list_update(connector);
	/* Move the prefered mode first, help apps pick the right mode. */
	drm_mode_sort(&connector->modes);
	connector->display_info.width_mm = pipe->width_mm;
	connector->display_info.height_mm = pipe->height_mm;
	ret = drm_display_info_set_bus_formats(&connector->display_info,
			rad_bus_formats, ARRAY_SIZE(rad_bus_formats));
	if (ret)
		return ret;

	return count;
}

static const struct drm_connector_helper_funcs virtual_connector_helper_funcs = {
	.get_modes = virtual_connector_get_modes,
};

static const struct drm_connector_funcs virtual_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int se_virtual_parse_ppl_dt(struct virtual_display *virtual_display,
			struct device_node *child)
{
	struct device_node *np = child;
	struct drm_display_mode *mode;
	int ret;
	u8 stream;

	stream = virtual_display->pipe_num;

	mode = drm_mode_create(virtual_display->drm_dev);
	if (!mode) {
		DRM_DEV_ERROR(virtual_display->drm_dev->dev, "failed to allocate mode!\n");
		return -EINVAL;
	}

	ret = of_get_drm_display_mode(np, mode, NULL, -1);
	if (ret) {
		DRM_DEV_ERROR(virtual_display->drm_dev->dev, "failed to get display timing!\n");
		drm_mode_destroy(virtual_display->drm_dev, mode);
		return ret;
	}

	of_property_read_u32(np, "panel-width-mm", &virtual_display->pipe[stream].width_mm);
	of_property_read_u32(np, "panel-height-mm", &virtual_display->pipe[stream].height_mm);

	virtual_display->pipe[stream].mode = mode;
	virtual_display->pipe_num ++;

	return 0;
}

static int se_virtual_parse_dt(struct virtual_display *virtual_display)
{
	struct device_node *child, *np = virtual_display->of_node;
	int ret;
	virtual_display->pipe_num = 0;

	for_each_available_child_of_node(np, child)
		if (of_node_cmp(child->name, "sadp-fake-timings") == 0) {
			ret = se_virtual_parse_ppl_dt(virtual_display, child);
			if (ret) {
				DRM_DEV_ERROR(virtual_display->drm_dev->dev,
					"Failed to find fake-timings node!\n");
				return ret;
			}
		}

	return 0;
}

static int virtual_display_comp_bind(struct device *dev,
						struct device *master, void *data)
{
	struct virtual_display *virtual_display = dev_get_drvdata(dev);
	struct virtual_display_pipeline *pipe;

	int possible_crtc;
	int ret;
	int i = 0, j = 0;

	virtual_display->drm_dev = data;

	ret = se_virtual_parse_dt(virtual_display);
	if (ret) {
		DRM_DEV_ERROR(dev, "failed to parse display dt\n");
		return -EINVAL;
	}

	possible_crtc = drm_of_find_possible_crtcs(virtual_display->drm_dev, dev->of_node);

	if(possible_crtc == 2)
		j++;
	else if (possible_crtc == 0) {
		DRM_DEV_ERROR(dev, "failed to find possible_crtcs: %d\n", possible_crtc);
		return -EINVAL;
	}

	for(i = 0; i < virtual_display->pipe_num; i++){
		pipe = &virtual_display->pipe[i];

		if (possible_crtc & BIT(j)){

			pipe->enc.possible_crtcs = 1 << j++;

			drm_simple_encoder_init(virtual_display->drm_dev, &pipe->enc,
						DRM_MODE_ENCODER_DSI);

			drm_connector_init(virtual_display->drm_dev, &pipe->conn,
				&virtual_connector_funcs, DRM_MODE_CONNECTOR_DSI);

			drm_connector_helper_add(&pipe->conn, &virtual_connector_helper_funcs);

			ret = drm_connector_attach_encoder(&pipe->conn, &pipe->enc);
			if (ret) {
				DRM_DEV_ERROR(dev, "failed to attach connnector with encoder: %d\n",ret);
				return -EINVAL;
			}
		}
	}
	return 0;
}

static void virtual_display_comp_unbind(struct device *dev, struct device *master, void *data)
{
	//reserved
}


static const struct component_ops virtual_display_component_ops = {
	.bind = virtual_display_comp_bind,
	.unbind = virtual_display_comp_unbind,
};

static int virtual_display_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct virtual_display *virtual_display;
	int ret;

	virtual_display = devm_kzalloc(dev, sizeof(*virtual_display), GFP_KERNEL);
	if (!virtual_display) {
		DRM_DEV_ERROR(dev, "failed to allocate virtual_display\n");
		return -ENODEV;
	}

	virtual_display->dev = dev;
	virtual_display->of_node = dev->of_node;

	dev_set_drvdata(dev, virtual_display);

	ret = component_add(dev, &virtual_display_component_ops);
	if (ret) {
		DRM_DEV_ERROR(dev, "failed to register component: %d\n",
					ret);
		return -ENODEV;
	}

	return 0;
}

static int virtual_display_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id virtual_display_dt_ids[] = {
	{
	 .compatible = "siengine,se1000-sadp-virtual-dsi",
	},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, virtual_display_dt_ids);

struct platform_driver virtual_display_driver = {
	.probe		= virtual_display_probe,
	.remove		= virtual_display_remove,
	.driver		= {
		.of_match_table = virtual_display_dt_ids,
		.name	= "siengine-sadp-virtual-display",
	},
};

module_platform_driver(virtual_display_driver);

MODULE_AUTHOR("Lucas Liu <lucas.liu@siengine.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SiEngine SE1000 Virtual-Display-Driver");
