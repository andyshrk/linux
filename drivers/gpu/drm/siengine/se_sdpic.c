#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/component.h>
#include <linux/irq.h>
#include <asm/barrier.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/platform_device.h>
#include <linux/component.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_probe_helper.h>

#include <drm/drm_edid.h>
#include <drm/drm_print.h>

#include <drm/drm_encoder.h>
#include <drm/drm_crtc.h>
#include <drm/drm_print.h>
#include <drm/drm_atomic_helper.h>
#include <drm/bridge/dw_mipi_dsi.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>

#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_connector.h>

#define SDPIC_MAX_PIPE 2

#define SDPIC_VERSION 0x00
#define SDPIC_IRQ_RAW_STATUS 0x04
#define SDPIC_IRQ_CLEAN 0x08
#define SDPIC_IRQ_MASK 0x0C
#define SDPIC_IRQ_STATUS 0x10
#define SDPIC_IRQ_LINE 0x14
#define SDPIC_WORK_MODE 0x20
#define SDPIC_COMMAND 0x24
#define SDPIC_DUMMY_RGB 0x28
#define SDPIC_IMAGE_VACTIVE 0x40
#define SDPIC_IMAGE_HACTIVE 0x44
#define SDPIC_ACTIVESIZE 0x48
#define SDPIC_HINTERVALS 0x4C
#define SDPIC_VINTERVALS 0x50
#define SDPIC_SYNC 0x54
#define SDPIC_FSM_ST 0x60

//work mode
#define BYPASS_MODE_SHIFT 0
#define DPI_HALT_MODE	2
#define IMG0_LINE_MODE_SHIFT 4
#define IMG1_LINE_MODE_SHIFT 5
#define DPI_START_MODE_SHIFT 8
#define IMG_START_MODE_SHIFT 10
#define IMG0_LINE_TOTAL_SHIFT 16
#define IMG0_LINE_ACTIVE_SHIFT 20
#define IMG1_LINE_TOTAL_SHIFT 24
#define IMG1_LINE_ACTIVE_SHIFT 28

//image_hactive
#define IMG1_V_ACTIVE_SHIFT 16
#define IMG0_V_ACTIVE_SHIFT 0
//image_vactive
#define IMG1_H_ACTIVE_SHIFT 16
#define IMG0_H_ACTIVE_SHIFT 0

//h_intervals
#define HBP_SHIFT 16
#define HFP_SHIFT 0

//v_intervals
#define VBP_SHIFT 16
#define VFP_SHIFT 0

//sync
#define V_POLARITY_SHIFT 25
#define H_POLARITY_SHIFT 24
#define V_WIDTH_SHIFT 16
#define H_WIDTH_SHIFT 0

//cmd
#define CMD_ENABLE_SHIFT 0

//bypass mode
enum bypass_mode {
	SUPERFRAME_MODE = 0,
	DPI0_BYPASS_MODE,
	DPI1_BYPASS_MODE,
};

enum img_start_mode {
	DATAEN = 0,
	VSYNC,
};

enum dpi_start_mode {
	DPI0_DPI1_BOTH = 0,
	DPI0_ONLY,
	DPI1_ONLY,
	DPI0_DPI1_NONE,
};

enum img_line_mode {
	VALID_FIRST = 0,
	DUMMY_FIRST,
};

#define SDPIC_IRQ_LINE1 BIT(5)
#define SDPIC_IRQ_LINE0 BIT(4)
#define SDPIC_IRQ_UNDERFLOW1 BIT(1)
#define SDPIC_IRQ_UNDERFLOW0 BIT(0)

#define SDPIC_IRQ_MASK_UNDERFLOW (SDPIC_IRQ_UNDERFLOW1 | SDPIC_IRQ_UNDERFLOW0)
#define SDPIC_IRQ_MASK_ALL (SDPIC_IRQ_LINE1 | SDPIC_IRQ_LINE0 | SDPIC_IRQ_MASK_UNDERFLOW)

#define SDPIC_IRQ_ENBALE_UNDERFLOW (SDPIC_IRQ_UNDERFLOW1 | SDPIC_IRQ_UNDERFLOW0)
#define SDPIC_IRQ_ENABLE_ALL (SDPIC_IRQ_LINE1 | SDPIC_IRQ_LINE0 | SDPIC_IRQ_ENBALE_UNDERFLOW)

#define to_sdpic_encoder(c) \
	container_of(c, struct se_sdpic, enc)

struct se_sdpic {
	struct platform_device *pdev;

	void __iomem *base;
	struct clk *apb_clk;
	struct clk *pxl_clk;

	//struct videomode vm;
	struct videomode vm_adjust;
	int height;
	int irq;

	struct drm_device *drm_dev;
	struct device *dev;

	struct drm_encoder enc;
	struct drm_connector conn;

	struct drm_display_mode *mode;	/*fake_display_mode, not an array*/
	struct drm_display_mode *mode_dsi;
	struct drm_display_mode *mode_fake;
	struct device_node *of_node; /* pipeline dt node */

	struct videomode vm0;
	struct videomode vm1;

	bool dummy_disable;


	u32 width_mm;
	u32 height_mm;
};

static int greatest_common_divisor(int a, int b)
{
	return b ? greatest_common_divisor(b, a % b) : a;
}

static void se_sdpic_merge_timing(struct videomode *vm, struct videomode *vm0,
					struct videomode *vm1, int refresh)
{
	int htotal, vtotal;

	vm->hactive = vm0->hactive + vm1->hactive;
	vm->hfront_porch = vm0->hfront_porch + vm1->hfront_porch;
	vm->hsync_len = vm0->hsync_len +  vm1->hsync_len;
	vm->hback_porch = vm0->hback_porch + vm1->hback_porch;

	vm->vactive = max(vm0->vactive, vm1->vactive);
	vm->vfront_porch = max(vm0->vfront_porch, vm1->vfront_porch);
	vm->vsync_len = max(vm0->vsync_len, vm1->vsync_len);
	vm->vback_porch = max(vm0->vback_porch, vm1->vback_porch);

	vm->flags = vm0->flags;

	htotal = vm->hactive + vm->hfront_porch + vm->hsync_len + vm->hback_porch;
	vtotal = vm->vactive + vm->vfront_porch + vm->vsync_len + vm->vback_porch;

	vm->pixelclock = htotal * vtotal * refresh;
	printk("vm0->hfront_porch %x  vm1->hfront_porch %x\n", vm0->hfront_porch, vm1->hfront_porch);
}

static void cal_sdpic_timing(struct se_sdpic *sdpic)
{

	int refresh = 0;

	struct drm_display_mode *dmode_adjust ;

	dmode_adjust = kzalloc(sizeof(struct drm_display_mode), GFP_KERNEL);
	drm_display_mode_from_videomode(&sdpic->vm0, dmode_adjust);

	refresh = drm_mode_vrefresh(dmode_adjust);
	//refresh = 60;
	kfree(dmode_adjust);

	//mode timing used for dsi
	se_sdpic_merge_timing(&sdpic->vm_adjust, &sdpic->vm0, &sdpic->vm1, refresh);

	sdpic->height = max(sdpic->vm0.vactive, sdpic->vm1.vactive);
}

static inline void sdpic_write(struct se_sdpic *sdpic, u32 reg, u32 val)
{
	writel(val, sdpic->base + reg);
}

static inline u32 sdpic_read(struct  se_sdpic *sdpic, u32 reg)
{
	return readl(sdpic->base + reg);
}

static void sdpic_clk_enable(struct  se_sdpic *sdpic, bool enable)
{

	if (enable) {
		clk_prepare_enable(sdpic->apb_clk);
		clk_prepare_enable(sdpic->pxl_clk);
	} else {
		//clk_disable_unprepare(sdpic->pxl_clk);
		//clk_disable_unprepare(sdpic->apb_clk);
	}

}

static void sdpic_enable_irq(struct se_sdpic *sdpic)
{
	sdpic_write(sdpic, SDPIC_IRQ_MASK, SDPIC_IRQ_MASK_UNDERFLOW);
	sdpic_write(sdpic, SDPIC_IRQ_STATUS, SDPIC_IRQ_ENBALE_UNDERFLOW);
}

static void sdpic_update_reg(struct se_sdpic *sdpic)
{
	struct drm_display_mode *dmode0;
	struct drm_display_mode *dmode1;
	u32 img_vactive_reg = 0;
	u32 img_hactive_reg = 0;
	u32 h_intervals_reg = 0;
	u32 v_intervals_reg = 0;
	u32 sync_reg = 0;
	u32 work_mode_reg = 0;
	int h_polarity;
	int v_polarity;
	int gcd;
	int line_active[SDPIC_MAX_PIPE];
	int line_total;

	dmode0 = kzalloc(sizeof(struct drm_display_mode), GFP_KERNEL);
	dmode1 = kzalloc(sizeof(struct drm_display_mode), GFP_KERNEL);

	drm_display_mode_from_videomode(&sdpic->vm0, dmode1);

	drm_display_mode_from_videomode(&sdpic->vm1, dmode0);

	img_vactive_reg |= (dmode0->vdisplay << IMG0_V_ACTIVE_SHIFT);
	img_vactive_reg |= (dmode1->vdisplay << IMG1_V_ACTIVE_SHIFT);

	sdpic_write(sdpic, SDPIC_IMAGE_VACTIVE, img_vactive_reg);

	img_hactive_reg |= (dmode0->hdisplay << IMG0_H_ACTIVE_SHIFT);
	img_hactive_reg |= (dmode1->hdisplay << IMG1_H_ACTIVE_SHIFT);

	sdpic_write(sdpic, SDPIC_IMAGE_HACTIVE, img_hactive_reg);

	sdpic_write(sdpic, SDPIC_ACTIVESIZE, sdpic->height);

	h_intervals_reg = (sdpic->vm_adjust.hback_porch << HBP_SHIFT);
	h_intervals_reg |= (sdpic->vm_adjust.hfront_porch << HFP_SHIFT);
	sdpic_write(sdpic, SDPIC_HINTERVALS, h_intervals_reg);

	v_intervals_reg = (sdpic->vm_adjust.vback_porch << VBP_SHIFT);
	v_intervals_reg |= (sdpic->vm_adjust.vfront_porch << VFP_SHIFT);
	sdpic_write(sdpic, SDPIC_VINTERVALS, v_intervals_reg);

	sync_reg = (sdpic->vm_adjust.hsync_len << H_WIDTH_SHIFT);
	sync_reg |= (sdpic->vm_adjust.vsync_len << V_WIDTH_SHIFT);

	h_polarity = !!(sdpic->vm_adjust.flags & DISPLAY_FLAGS_HSYNC_HIGH);
	v_polarity = !!(sdpic->vm_adjust.flags & DISPLAY_FLAGS_VSYNC_HIGH);

	sync_reg |= (h_polarity << H_POLARITY_SHIFT);
	sync_reg |= (v_polarity << V_POLARITY_SHIFT);

	sdpic_write(sdpic, SDPIC_SYNC, sync_reg);

	gcd = greatest_common_divisor(dmode0->vdisplay, dmode1->vdisplay);
	line_active[0] = dmode0->vdisplay / gcd;
	line_active[1] = dmode1->vdisplay / gcd;
	line_total = max(line_active[0], line_active[1]);

	if(sdpic->dummy_disable) {
		line_active[0] = line_total;
		line_active[1] = line_total;
	}

	if (line_total > 1) {
		line_total = line_total - 1;

		//if (line_active[0] > 1)
		line_active[0] = line_active[0] - 1;

		//if (line_active[1] > 1)
		line_active[1] = line_active[1] - 1;
	}


	work_mode_reg |= (SUPERFRAME_MODE << BYPASS_MODE_SHIFT);
	work_mode_reg |= (VALID_FIRST << IMG0_LINE_MODE_SHIFT);
	work_mode_reg |= (VALID_FIRST << IMG1_LINE_MODE_SHIFT);
	work_mode_reg |= (DPI0_DPI1_NONE << DPI_START_MODE_SHIFT);
	//work_mode_reg |= (VSYNC << IMG_START_MODE_SHIFT);
	work_mode_reg |= ((line_total) << IMG0_LINE_TOTAL_SHIFT);
	work_mode_reg |= ((line_active[0]) << IMG0_LINE_ACTIVE_SHIFT);
	work_mode_reg |= ((line_total) << IMG1_LINE_TOTAL_SHIFT);
	work_mode_reg |= ((line_active[1]) << IMG1_LINE_ACTIVE_SHIFT);

	sdpic_write(sdpic, SDPIC_WORK_MODE, work_mode_reg);

	kfree(dmode0);
	kfree(dmode1);
}

static void sdpic_enable(struct se_sdpic *sdpic, bool enable)
{
	sdpic_write(sdpic, SDPIC_COMMAND, (enable << CMD_ENABLE_SHIFT));
}

static void se_sdpic_encoder_mode_set(struct drm_encoder *encoder,
				    struct drm_display_mode *mode,
				    struct drm_display_mode *adjusted)
{
	//struct se_sdpic_encoder *sdpic_enc = to_sdpic_encoder(encoder);

	struct se_sdpic *sdpic = to_sdpic_encoder(encoder);

	//sdpic->mode = mode;
	//sdpic_enc->adjust = adjusted;

	clk_disable_unprepare(sdpic->apb_clk);
}

static void se_sdpic_encoder_enable(struct drm_encoder *encoder)
{
	struct se_sdpic *sdpic = to_sdpic_encoder(encoder);
	int val;

	//pm_runtime_get(&sdpic->pdev->dev);

	val = sdpic_read(sdpic, SDPIC_WORK_MODE);

	sdpic_clk_enable(sdpic, true);
}

static void se_sdpic_encoder_disable(struct drm_encoder *encoder)
{
	struct se_sdpic *sdpic = to_sdpic_encoder(encoder);
	int val;

	val = sdpic_read(sdpic, SDPIC_WORK_MODE);

	sdpic_clk_enable(sdpic, false);
	//pm_runtime_put(&sdpic->pdev->dev);
}

static irqreturn_t sdpic_irq(int irq, void *data)
{
	struct se_sdpic *sdpic = data;
	int val;
	//struct device *dev = &sdpic->pdev->dev;

	val = sdpic_read(sdpic, SDPIC_IRQ_RAW_STATUS);

	if (val & SDPIC_IRQ_UNDERFLOW0) {
		sdpic_write(sdpic, SDPIC_COMMAND, 3);
	}

	if (val & SDPIC_IRQ_UNDERFLOW1) {
		sdpic_write(sdpic, SDPIC_COMMAND, 3);
	}

	sdpic_write(sdpic, SDPIC_IRQ_CLEAN, val);

	sdpic_write(sdpic, SDPIC_COMMAND, 1);

	return IRQ_HANDLED;
}

//support only one mode from devicetree current
static int se_add_fake_mode(struct se_sdpic *sdpic)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(sdpic->conn.dev, sdpic->mode);
	mode->width_mm = sdpic->width_mm;
	mode->height_mm = sdpic->height_mm;
	drm_mode_probed_add(&sdpic->conn, mode);

	return 1;
}

static int fake_connector_get_modes(struct drm_connector *connector)
{
	int count,ret;
	struct se_sdpic *sdpic;
	u32 rad_bus_formats[] = {
		MEDIA_BUS_FMT_RGB888_1X24,
		MEDIA_BUS_FMT_RGB666_1X18,
		MEDIA_BUS_FMT_RGB565_1X16,
	};
	sdpic = container_of(connector, struct se_sdpic, conn);

	count = se_add_fake_mode(sdpic);
	drm_set_preferred_mode(&sdpic->conn, sdpic->vm0.hactive, sdpic->vm0.vactive);

	drm_connector_list_update(connector);
	/* Move the prefered mode first, help apps pick the right mode. */
	drm_mode_sort(&connector->modes);

	connector->display_info.width_mm = sdpic->width_mm;
	connector->display_info.height_mm = sdpic->height_mm;
	ret = drm_display_info_set_bus_formats(&connector->display_info,
			rad_bus_formats, ARRAY_SIZE(rad_bus_formats));
	if (ret)
		return ret;
	return count;
}



static enum drm_mode_status fake_conn_mode_valid(struct drm_connector *connector,
								struct drm_display_mode *mode)
{
	return MODE_OK;
};

static const struct drm_connector_helper_funcs fake_connector_helper_funcs = {
	.get_modes = fake_connector_get_modes,
	.mode_valid = fake_conn_mode_valid,
};



static enum drm_connector_status
fake_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected ;
}

static const struct drm_connector_funcs fake_connector_funcs = {
	.detect = fake_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_encoder_helper_funcs fake_encoder_helper_funcs = {
	.mode_set = se_sdpic_encoder_mode_set,
	.enable = se_sdpic_encoder_enable,
	.disable = se_sdpic_encoder_disable,

};

static const struct drm_encoder_funcs fake_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int se_fake_parse_ppl_dt(struct se_sdpic *sdpic,
			struct device_node *child)
{
	struct device_node *np = child;
	struct drm_display_mode *mode;
	struct device_node *timings;
	int ret;

	mode = drm_mode_create(sdpic->drm_dev);
	if (!mode) {
		DRM_DEV_ERROR(sdpic->drm_dev->dev, "Failed to allocate mode!\n");
		return -EINVAL;
	}

	ret = of_get_drm_display_mode(np, mode, NULL, -1);
	if (ret) {
		drm_mode_destroy(sdpic->drm_dev, mode);
		DRM_DEV_ERROR(sdpic->drm_dev->dev, "Failed to get display timing!\n");
		return ret;
	}

	sdpic->mode = mode;

	of_property_read_u32(np, "panel-width-mm", &sdpic->width_mm);
	of_property_read_u32(np, "panel-height-mm", &sdpic->height_mm);
	//mode->width_mm  = sdpic->width_mm;
	//mode->height_mm = sdpic->height_mm;
	timings = of_get_child_by_name(np, "display-timings");
	if (timings) {
			of_node_put(timings);
			ret = of_get_videomode(np, &sdpic->vm0, 0);

			ret = of_get_videomode(np, &sdpic->vm1, 1);
	}

	sdpic->dummy_disable = of_property_read_bool(np, "dummy_disable");

	return 0;
}

static int se_fake_parse_dt(struct se_sdpic *sdpic)
{
	struct device_node *child, *np = sdpic->of_node;
	int ret;

	for_each_available_child_of_node(np, child)
		if (of_node_cmp(child->name, "fake-timings") == 0) {
			ret = se_fake_parse_ppl_dt(sdpic, child);
			if (ret) {
				DRM_DEV_ERROR(sdpic->drm_dev->dev,
					"Failed to find fake-timings node!\n");
				return ret;
			}
		}

	return 0;
}

static int fake_display_comp_bind(struct device *dev,
						struct device *master, void *data)
{
	int possible_crtc;
	int ret;
	struct se_sdpic *sdpic = dev_get_drvdata(dev);

	sdpic->drm_dev = data;

	ret = se_fake_parse_dt(sdpic);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to parse display dt\n");
		return -EINVAL;
	}

	possible_crtc = drm_of_find_possible_crtcs(sdpic->drm_dev, dev->of_node);
	if (possible_crtc) {
		sdpic->enc.possible_crtcs = possible_crtc;
	} else {
		DRM_DEV_ERROR(dev, "Failed to find possible_crtcs: %d\n",
					possible_crtc);
		return -EINVAL;
	}

#if 0
	//fake encoder init and add fake helper
	drm_encoder_init(sdpic->drm_dev, &sdpic->enc,
		&fake_encoder_funcs, DRM_MODE_ENCODER_VIRTUAL, NULL);
#endif

	drm_simple_encoder_init(sdpic->drm_dev, &sdpic->enc, DRM_MODE_ENCODER_DSI);

	drm_encoder_helper_add(&sdpic->enc, &fake_encoder_helper_funcs);

	//fake connector init and add fake helper
	drm_connector_init(sdpic->drm_dev, &sdpic->conn,
		&fake_connector_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_connector_helper_add(&sdpic->conn, &fake_connector_helper_funcs);

	ret = drm_connector_attach_encoder(&sdpic->conn, &sdpic->enc);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to attach connnector with encoder: %d\n",
					ret);
		return -EINVAL;
	}
	clk_prepare_enable(sdpic->apb_clk);
	clk_prepare_enable(sdpic->pxl_clk);

	sdpic_clk_enable(sdpic, true);
	cal_sdpic_timing(sdpic);
	sdpic_enable_irq(sdpic);
	sdpic_enable(sdpic, false);
	sdpic_update_reg(sdpic);
	sdpic_enable(sdpic, true);

	return 0;
}

static void fake_display_comp_unbind(struct device *dev, struct device *master, void *data)
{
	struct se_sdpic *sdpic = dev_get_drvdata(dev);

	sdpic_clk_enable(sdpic, false);
	pm_runtime_put(&sdpic->pdev->dev);
}

//ops of master-component
static const struct component_ops fake_display_component_ops = {
	.bind = fake_display_comp_bind,
	.unbind = fake_display_comp_unbind,
};

static int se_sdpic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct se_sdpic *sdpic;
	int ret;
	struct resource *res;

	sdpic = devm_kzalloc(dev, sizeof(*sdpic), GFP_KERNEL);
	if (!sdpic) {
		DRM_DEV_ERROR(dev, "Failed to allocate sdpic\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "No memory resource\n");
		return -ENODEV;
	}

	sdpic->irq = platform_get_irq(pdev, 0);
	if (sdpic->irq < 0) {
		dev_err(dev, "No IRQ\n");
		return -ENODEV;
	}

	sdpic->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(sdpic->base)) {
		dev_err(dev, "Failed to map memory resource\n");
		return PTR_ERR(sdpic->base);
	}

	sdpic->apb_clk = devm_clk_get(dev, "pclk");
	if (IS_ERR(sdpic->apb_clk)) {
		ret = PTR_ERR(sdpic->apb_clk);
		dev_err(dev, "%s Unable to get pclk: %d\n", __func__, ret);
		return ret;
	}

	sdpic->pxl_clk = devm_clk_get(dev, "pxclk");
	if (IS_ERR(sdpic->pxl_clk)) {
		ret = PTR_ERR(sdpic->pxl_clk);
		dev_err(dev, "%s Unable to get pxclk: %d\n", __func__, ret);
		return ret;
	}

	ret = devm_request_threaded_irq(dev,
				   sdpic->irq,
				   sdpic_irq,
				   NULL,
				   IRQF_SHARED | IRQ_LEVEL,
				   "se_sdpic",
				   sdpic);

	sdpic->dev = dev;
	sdpic->of_node = dev->of_node;

	dev_set_drvdata(dev, sdpic);

	//register to component
	ret = component_add(dev, &fake_display_component_ops);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to register component: %d\n",
					ret);
		return -ENODEV;
	}


	return 0;
}

static int se_sdpic_remove(struct platform_device *pdev)
{
	return 0;
}

static int  se_sdpic_resume(struct device *dev)
{
	struct se_sdpic *sdpic = dev_get_drvdata(dev);

	clk_prepare_enable(sdpic->apb_clk);
	clk_prepare_enable(sdpic->pxl_clk);
	sdpic_clk_enable(sdpic, true);
	cal_sdpic_timing(sdpic);
	sdpic_enable_irq(sdpic);
	sdpic_enable(sdpic, false);
	sdpic_update_reg(sdpic);
	sdpic_enable(sdpic, true);

	return 0;

}

static const struct dev_pm_ops se_sdpic_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(NULL, se_sdpic_resume)
};

static const struct of_device_id se_sdpic_dt_ids[] = {
	{
		.compatible = "siengine,se1000-sdpic",
	},
	{
		/* sentinel */
	}
};

MODULE_DEVICE_TABLE(of, se_sdpic_dt_ids);


struct platform_driver se_sdpic_driver = {
	.probe		= se_sdpic_probe,
	.remove		= se_sdpic_remove,
	.driver		= {
		.of_match_table = se_sdpic_dt_ids,
		.pm     = &se_sdpic_pm_ops,
		.name	= "siengine-sdpic",
	},
};

module_platform_driver(se_sdpic_driver);

MODULE_AUTHOR("Zhiqiang Zhang <zhiqiang.zhang@siengine.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Se1000 sdpic Driver");
