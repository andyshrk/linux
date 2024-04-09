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

static const u32 se_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

struct se_panel {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;

	struct gpio_desc *reset;

	bool prepared;
	bool enabled;

	struct videomode vm;
	u32 width_mm;
	u32 height_mm;
};

static inline struct se_panel *to_se_panel(struct drm_panel *panel)
{
	return container_of(panel, struct se_panel, base);
}

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

static int se_panel_prepare(struct drm_panel *panel)
{
	struct se_panel *se = to_se_panel(panel);

	if (se->prepared)
		return 0;

	se->prepared = true;

	return 0;
}

static int se_panel_unprepare(struct drm_panel *panel)
{
	struct se_panel *se = to_se_panel(panel);
	struct device *dev = &se->dsi->dev;

	if (!se->prepared)
		return 0;

	if (se->enabled) {
		DRM_DEV_ERROR(dev, "Panel still enabled!\n");
		return -EPERM;
	}

	se->prepared = false;

	return 0;
}

static int se_panel_enable(struct drm_panel *panel)
{
	struct se_panel *se = to_se_panel(panel);
	struct mipi_dsi_device *dsi = se->dsi;
	struct device *dev = &dsi->dev;
	int color_format = color_format_from_dsi_format(dsi->format);

	(void) color_format;
	if (se->enabled)
		return 0;

	if (!se->prepared) {
		DRM_DEV_ERROR(dev, "Panel not prepared!\n");
		return -EPERM;
	}

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	se->enabled = true;

	return 0;
}

static int se_panel_disable(struct drm_panel *panel)
{
	struct se_panel *se = to_se_panel(panel);
	struct mipi_dsi_device *dsi = se->dsi;

	if (!se->enabled)
		return 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	se->enabled = false;

	return 0;
}

static int se_panel_get_modes(struct drm_panel *panel,
				struct drm_connector *connector)
{
	struct se_panel *se = to_se_panel(panel);
	struct device *dev = &se->dsi->dev;
	//struct drm_connector *connector = panel->connector;
	struct drm_display_mode *mode;
	u32 *bus_flags = &connector->display_info.bus_flags;
	int ret;

	mode = drm_mode_create(connector->dev);
	if (!mode) {
		DRM_DEV_ERROR(dev, "Failed to create display mode!\n");
		return 0;
	}

	drm_display_mode_from_videomode(&se->vm, mode);
	mode->width_mm = se->width_mm;
	mode->height_mm = se->height_mm;
	connector->display_info.width_mm = se->width_mm;
	connector->display_info.height_mm = se->height_mm;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;


	if (se->vm.flags & DISPLAY_FLAGS_DE_HIGH)
		*bus_flags |= DRM_BUS_FLAG_DE_HIGH;
	if (se->vm.flags & DISPLAY_FLAGS_DE_LOW)
		*bus_flags |= DRM_BUS_FLAG_DE_LOW;
	if (se->vm.flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_NEGEDGE;
	if (se->vm.flags & DISPLAY_FLAGS_PIXDATA_POSEDGE)
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_POSEDGE;

	ret = drm_display_info_set_bus_formats(&connector->display_info,
			se_bus_formats, ARRAY_SIZE(se_bus_formats));
	if (ret) {
		drm_mode_destroy(connector->dev, mode);
		return ret;
	}

	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs se_panel_funcs = {
	.prepare = se_panel_prepare,
	.unprepare = se_panel_unprepare,
	.enable = se_panel_enable,
	.disable = se_panel_disable,
	.get_modes = se_panel_get_modes,
};

/*
 * The clock might range from 66MHz (30Hz refresh rate)
 * to 132MHz (60Hz refresh rate)
 */
static const struct display_timing se_default_timing = {
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

static int se_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *np = dev->of_node;
	struct device_node *timings;
	struct se_panel *panel;
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
		videomode_from_timing(&se_default_timing, &panel->vm);
	}

	if (ret < 0) {
		dev_err(dev, "display-timings (%d)\n", ret);
		return ret;
	}

	of_property_read_u32(np, "panel-width-mm", &panel->width_mm);
	of_property_read_u32(np, "panel-height-mm", &panel->height_mm);

	panel->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);

	if (IS_ERR(panel->reset))
		panel->reset = NULL;
	else
		gpiod_set_value(panel->reset, 0);

	drm_panel_init(&panel->base, dev, &se_panel_funcs,
			DRM_MODE_CONNECTOR_DSI);
	dev_set_drvdata(dev, panel);

	drm_panel_add(&panel->base);

	if (ret < 0)
		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		printk("drm_panel_remove \n");
		drm_panel_remove(&panel->base);
	}
	return ret;
}

static int se_panel_remove(struct mipi_dsi_device *dsi)
{
	struct se_panel *se = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "Failed to detach from host (%d)\n",
			ret);

	drm_panel_remove(&se->base);

	return 0;
}

static void se_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct se_panel *se = mipi_dsi_get_drvdata(dsi);

	se_panel_disable(&se->base);
	se_panel_unprepare(&se->base);
}

#ifdef CONFIG_PM
static int se_panel_suspend(struct device *dev)
{
	//struct se_panel *se = dev_get_drvdata(dev);
	return 0;
}

static int se_panel_resume(struct device *dev)
{
	//struct se_panel *se = dev_get_drvdata(dev);
	return 0;
}

#endif

static const struct dev_pm_ops se_pm_ops = {
	SET_RUNTIME_PM_OPS(se_panel_suspend, se_panel_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(se_panel_suspend, se_panel_resume)
};

static const struct of_device_id se_of_match[] = {
	{ .compatible = "siengine,dsi-panel", },
	{ }
};
MODULE_DEVICE_TABLE(of, se_of_match);

static struct mipi_dsi_driver se_panel_driver = {
	.driver = {
		.name = "panel-siengine-dsi",
		.of_match_table = se_of_match,
		.pm	= &se_pm_ops,
	},
	.probe = se_panel_probe,
	.remove = se_panel_remove,
	.shutdown = se_panel_shutdown,
};
module_mipi_dsi_driver(se_panel_driver);

MODULE_AUTHOR("songqin zhang<songqin.zhang@siengine.com>");
MODULE_DESCRIPTION("SE1000 DSI Panel");
MODULE_LICENSE("GPL v2");


