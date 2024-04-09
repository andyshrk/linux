// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018-2019 Synopsys, Inc. and/or its affiliates.
 *
 * Synopsys DesignWare MIPI CSI-2 Host controller driver.
 * Platform driver
 *
 * Author: Luis Oliveira <luis.oliveira@synopsys.com>
 */

#include <media/dwc/dw-csi-data.h>
#include <media/dwc/dw-dphy-data.h>

#include "dw-csi-plat.h"
#include "dw-dphy-rx.h"

//2 raw12  14 rgb888 15 yuv422
#define DEFAULT_FMT_INDEX	14
#define DEFAULT_PIX_HEIGHT	1080
#define DEFAULT_PIX_WIDTH	1920

#define CSI2_IOCTL_CMD_OFFSET	50
#define VIDIOC_CSI2_SET_COLOR_MODE \
	_IOWR('V', BASE_VIDIOC_PRIVATE + CSI2_IOCTL_CMD_OFFSET, unsigned int)

const struct mipi_dt csi_dt[] = {
	{
		.hex = CSI_2_YUV420_8,
		.name = "YUV420_8bits",
	}, {
		.hex = CSI_2_YUV420_10,
		.name = "YUV420_10bits",
	}, {
		.hex = CSI_2_YUV420_8_LEG,
		.name = "YUV420_8bits_LEGACY",
	}, {
		.hex = CSI_2_YUV420_8_SHIFT,
		.name = "YUV420_8bits_SHIFT",
	}, {
		.hex = CSI_2_YUV420_10_SHIFT,
		.name = "YUV420_10bits_SHIFT",
	}, {
		.hex = CSI_2_YUV422_8,
		.name = "YUV442_8bits",
	}, {
		.hex = CSI_2_YUV422_10,
		.name = "YUV442_10bits",
	}, {
		.hex = CSI_2_RGB444,
		.name = "RGB444",
	}, {
		.hex = CSI_2_RGB555,
		.name = "RGB555",
	}, {
		.hex = CSI_2_RGB565,
		.name = "RGB565",
	}, {
		.hex = CSI_2_RGB666,
		.name = "RGB666",
	}, {
		.hex = CSI_2_RGB888,
		.name = "RGB888",
	}, {
		.hex = CSI_2_RAW6,
		.name = "RAW6",
	}, {
		.hex = CSI_2_RAW7,
		.name = "RAW7",
	}, {
		.hex = CSI_2_RAW8,
		.name = "RAW8",
	}, {
		.hex = CSI_2_RAW10,
		.name = "RAW10",
	}, {
		.hex = CSI_2_RAW12,
		.name = "RAW12",
	}, {
		.hex = CSI_2_RAW14,
		.name = "RAW14",
	}, {
		.hex = CSI_2_RAW16,
		.name = "RAW16",
	},
};

static struct mipi_fmt *
find_dw_mipi_csi_format(struct v4l2_mbus_framefmt *mf)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dw_mipi_csi_formats); i++)
		if (mf->code == dw_mipi_csi_formats[i].mbus_code)
			return &dw_mipi_csi_formats[i];

	return NULL;
}

static int dw_mipi_csi_enum_mbus_code(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;

	code->code = dw_mipi_csi_formats[code->index].mbus_code;
	return 0;
}

static struct mipi_fmt *
dw_mipi_csi_try_format(struct v4l2_mbus_framefmt *mf)
{
	struct mipi_fmt *fmt;

	fmt = find_dw_mipi_csi_format(mf);
	if (!fmt)
	{
		/* set fmt RAW12 manually */
		printk("%s manually set fmt\n", __func__);
		fmt = &dw_mipi_csi_formats[DEFAULT_FMT_INDEX];
	}

	mf->code = fmt->mbus_code;

	return fmt;
}

static struct v4l2_mbus_framefmt *
dw_mipi_csi_get_format(struct dw_csi *dev, struct v4l2_subdev_pad_config *cfg,
			enum v4l2_subdev_format_whence which, int ipi_channel)
{
	dev_info(dev->dev, "%s in ipi_channel = %d\n", __func__, ipi_channel);

	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return cfg ? v4l2_subdev_get_try_format(&dev->sd, cfg, 0) : NULL;

	//TODO: get format from sensor, fixed value temporarily

	dev->format[ipi_channel - 1].width = dev->pix_width;
	dev->format[ipi_channel - 1].height = dev->pix_height;


	//MEDIA_BUS_FMT_SRGGB12_1X12 raw12, packed
	//MEDIA_BUS_FMT_ARGB8888_1X32      rgb888, packed
	dev->format[ipi_channel - 1].code = dev->fmt->mbus_code;

	dev_dbg(dev->dev,
		"%s got v4l2_mbus_pixelcode. 0x%x\n", __func__,
		dev->format[ipi_channel - 1].code);
	dev_dbg(dev->dev,
		"%s got width. %d\n", __func__,
		dev->format[ipi_channel - 1].width);
	dev_dbg(dev->dev,
		"%s got height. %d\n", __func__,
		dev->format[ipi_channel - 1].height);
	return &dev->format[ipi_channel - 1];
}

static int
dw_mipi_csi_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_format *fmt)
{
	struct dw_csi *dev = sd_to_mipi_csi_dev(sd);
	struct mipi_fmt *dev_fmt = NULL;
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	int i;

	uint32_t ipi_channel = fmt->pad + 1;

	dev_info(dev->dev,"%s ipi_channel =%d\n", __func__, ipi_channel);
	dev_fmt = dw_mipi_csi_try_format(&fmt->format);

	if (!fmt)
		return -EINVAL;

	dev->vchannel = ipi_channel;
	if (dev_fmt) {
		if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
			dev_dbg(dev->dev,"fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE\n");
			dev->fmt = dev_fmt;
		}

		dev->fmt->mbus_code = mf->code;
		dev_dbg(dev->dev,"mf->code = %d\n", mf->code);
		dev_dbg(dev->dev,"dev->fmt->mbus_code = %d\n", dev->fmt->mbus_code);

		dw_mipi_csi_set_ipi_fmt(dev, ipi_channel);
	}

	if (fmt->format.width > 0 && fmt->format.height > 0) {
		dw_mipi_csi_fill_timings(dev, fmt, ipi_channel);
	} else {
		dev_vdbg(dev->dev, "%s unacceptable values 0x%x.\n",
			__func__, fmt->format.width);
		dev_vdbg(dev->dev, "%s unacceptable values 0x%x.\n",
			__func__, fmt->format.height);
		return -EINVAL;
    }

	dev_dbg(dev->dev,"dev->format[ipi_channel - 1].height =%d\n",
		dev->format[ipi_channel - 1].height);
	dev_dbg(dev->dev,"dev->format[ipi_channel - 1].height =%d\n",
		dev->format[ipi_channel - 1].width);
	dev_dbg(dev->dev,"fmt->format.height; =%d\n", fmt->format.height);
	dev_dbg(dev->dev,"fmt->format.width =%d\n", fmt->format.width);

	dev->format[ipi_channel - 1].height = fmt->format.height;
	dev->format[ipi_channel - 1].width  = fmt->format.width;
	dev->pix_width = fmt->format.width;
	dev->pix_height = fmt->format.height;

	for (i = 0; i < ARRAY_SIZE(csi_dt); i++)
		if (csi_dt[i].hex == dev->hw.ipi_hw[ipi_channel-1].ipi_dt) {
			dev_info(dev->dev, "Using data type %s\n", csi_dt[i].name);
		}
	return 0;
}

static int
dw_mipi_csi_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_format *fmt)
{
	struct dw_csi *dev = sd_to_mipi_csi_dev(sd);
	struct v4l2_mbus_framefmt *mf;
	int ipi_channel = 0;
	ipi_channel = fmt->pad;

	dev_dbg(dev->dev,"enter %s, ipi_channel = %d\n", __func__, ipi_channel);

	mf = dw_mipi_csi_get_format(dev, cfg, fmt->which, ipi_channel);
	if (!mf)
		return -EINVAL;

	mutex_lock(&dev->lock);
	fmt->format = *mf;
	mutex_unlock(&dev->lock);

	dev_dbg(dev->dev,"exit %s\n", __func__);
	return 0;
}

static int
dw_mipi_csi_s_power(struct v4l2_subdev *sd, int on)
{
	struct dw_csi *dev = sd_to_mipi_csi_dev(sd);
	int i =0;

	dev_info(dev->dev, "%s: on=%d\n", __func__, on);
	dev_dbg(dev->dev, "for efficiency, will set color mode to COLOR48 default\n");
	for (i = 0; i < 4; i++) {
		dev->hw.ipi_hw[i].ipi_color_mode = COLOR48;
	}
	return 0;
}

static int
dw_mipi_csi_log_status(struct v4l2_subdev *sd)
{
	struct dw_csi *dev = sd_to_mipi_csi_dev(sd);

	dw_mipi_csi_dump(dev);

	return 0;
}

#if IS_ENABLED(CONFIG_VIDEO_ADV_DEBUG)
static int
dw_mipi_csi_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct dw_csi *dev = sd_to_mipi_csi_dev(sd);

	dev_vdbg(dev->dev, "%s: reg=%llu\n", __func__, reg->reg);
	reg->val = dw_mipi_csi_read(dev, reg->reg);

	return 0;
}
#endif
static int dw_mipi_csi_init_cfg(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg)
{
	struct v4l2_mbus_framefmt *format = v4l2_subdev_get_try_format(sd, cfg, 0);

	format->colorspace = V4L2_COLORSPACE_SRGB;
	format->code = MEDIA_BUS_FMT_ARGB8888_1X32;
	format->field = V4L2_FIELD_NONE;

	return 0;
}

static int dw_mipi_csi2_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct dw_csi *dev = sd_to_mipi_csi_dev(sd);
	int ipi_channel = dev->vchannel;

	dev_dbg(dev->dev, "enter %s\n", __func__);
	dev_info(dev->dev, "ipi_channel:%d, enable:%d\n", ipi_channel, enable);

	if (ipi_channel < 1 || ipi_channel > 4)
	{
		dev_err(dev->dev, "ipi channel = %d is invalid.\n", ipi_channel);
		return -1;
	}
	if (enable)
	{
		mutex_lock(&dev->lock);
		if (dev->dw_power_flag == 0) {
			dw_mipi_csi_hw_stdby(dev);
		}
		dev->dw_power_flag++;
		mutex_unlock(&dev->lock);
		if (!dev->running[ipi_channel-1])
		{
			if (0 == (ipi_channel-1))
			{
				dw_mipi_csi_start_ipi(dev);
			}
			else
			{
				dw_mipi_csi_start_ipin(dev, ipi_channel);
			}
			mutex_lock(&dev->lock);
			dev->running[ipi_channel-1]++;
			mutex_unlock(&dev->lock);
		}
		else
			dev_dbg(dev->dev, "ipi channel = %d is running already\n", ipi_channel);
	}
	else
	{
		mutex_lock(&dev->lock);
		dev->dw_power_flag--;
		if (dev->dw_power_flag == 0) {
			dw_mipi_csi_mask_irq_power_off(dev);
			for (ipi_channel = 1; ipi_channel < 5; ipi_channel++) {
				if (dev->running[ipi_channel-1]) {
					//disable ipi channel
					dev_err(dev->dev, "disable ipi channel %d\n", ipi_channel);
					dw_mipi_csi_disable_ipi(dev, ipi_channel);
					dev->hw.ipi_hw[ipi_channel - 1].ipi_color_mode = COLOR48;
					dev->running[ipi_channel-1]--;
				}
			}
		}
		mutex_unlock(&dev->lock);
	}

	return 0;
}

/* mipi csi2 subdev media entity operations */
static int mipi_csi2_link_setup(struct media_entity *entity,
		const struct media_pad *local,
		const struct media_pad *remote, u32 flags)
{
	return 0;
}

static long dw_mipi_csi2_subdev_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct dw_csi *dev = sd_to_mipi_csi_dev(sd);
	unsigned int csi2_arg;
	int i = 0;

	switch (cmd) {
	case VIDIOC_CSI2_SET_COLOR_MODE:
		csi2_arg = *(unsigned int *)arg;
		dev_info(dev->dev, "call mipicsi set color mode, MODE: %s\n",
			csi2_arg? "COLOR16": "COLOR48");
		for (i = 0; i < 4; i++) {
			if (csi2_arg == COLOR48) {
				dev->hw.ipi_hw[i].ipi_color_mode = COLOR48;
			} else {
				dev->hw.ipi_hw[i].ipi_color_mode = COLOR16;
			}
		}
		break;
	default:
		dev_dbg(dev->dev, "%s error, no such ioctl cmd: 0x%x\n", __func__, cmd);
		break;
	}

	return 0;
}

static struct v4l2_subdev_video_ops dw_mipi_csi2_video_ops = {
	.s_stream = dw_mipi_csi2_s_stream,
};

static struct v4l2_subdev_core_ops dw_mipi_csi_core_ops = {
	.ioctl = dw_mipi_csi2_subdev_ioctl,
	.s_power = dw_mipi_csi_s_power,
	.log_status = dw_mipi_csi_log_status,
#if IS_ENABLED(CONFIG_VIDEO_ADV_DEBUG)
	.g_register = dw_mipi_csi_g_register,
#endif
};

static struct v4l2_subdev_pad_ops dw_mipi_csi_pad_ops = {
	.init_cfg = dw_mipi_csi_init_cfg,
	.enum_mbus_code = dw_mipi_csi_enum_mbus_code,
	.get_fmt = dw_mipi_csi_get_fmt,
	.set_fmt = dw_mipi_csi_set_fmt,
};

static struct v4l2_subdev_ops dw_mipi_csi_subdev_ops = {
	.core = &dw_mipi_csi_core_ops,
	.pad = &dw_mipi_csi_pad_ops,
	.video = &dw_mipi_csi2_video_ops,
};

static const struct media_entity_operations mipi_csi2_sd_media_ops = {
	.link_setup = mipi_csi2_link_setup,
};

static irqreturn_t dw_mipi_csi_irq1(int irq, void *dev_id)
{
	struct dw_csi *csi_dev = dev_id;

	dw_mipi_csi_irq_handler(csi_dev);

	return IRQ_HANDLED;
}

static int
dw_dphy_parse_dt(struct platform_device *pdev, struct dw_csi *csi, struct v4l2_fwnode_endpoint *ep)
{
	int i;
	int cnt;
	struct dw_dphy_rx *dphy_master = NULL;
	struct dw_dphy_rx *dphy_slave = NULL;
	struct device *dev = &pdev->dev;
	unsigned char *data_lanes = ep->bus.mipi_csi2.data_lanes;
	u32 lane_bits = 0;
	u32 lane_usebit = 0;

	dev_info(dev, "of_node = %pOF\n", dev->of_node);
	dev_info(dev, "num_lanes = %d\n", csi->hw.num_lanes);
	cnt = of_count_phandle_with_args(dev->of_node, "phys", "#phy-cells");
	// cnt = of_count_phandle_with_args(dev->of_node, "phys", NULL);
	/* if can not found phy handle */
	if (cnt <= 0) {
		dev_info(dev, "Can not find phys phandle, no phy bind to the controller\n");
		return 0;
	}
	dev_info(dev, "cnt = %d\n", cnt);
	/* only support 1 or 2 phys */
	if (cnt > 2) {
		dev_err(dev, "Can not support more than 2 phys\n");
		return -EINVAL;
	}

	switch (csi->hw.num_lanes) {
	case 4:
	// case 3: /* maybe support 3lane */
		if (2 != cnt) {
			dev_err(dev, "In 4 lane mode, must has 2 phys\n");
			return -EINVAL;
		}
		break;
	case 2:
	// case 1: /* support 1lane */
		if (1 != cnt) {
			dev_err(dev, "In %dlane mode, must has 1 phys\n", csi->hw.num_lanes);
			return -EINVAL;
		}
		break;
	default:
		dev_err(dev, "Can not support %dlane mode\n", csi->hw.num_lanes);
		return -EINVAL;
	}

	csi->phy_master = devm_of_phy_get_by_index(dev, dev->of_node, 0);
	if (IS_ERR(csi->phy_master)) {
		dev_err(dev, "No DPHY0 available\n");
		return PTR_ERR(csi->phy_master);
	}

	dphy_master = phy_get_drvdata(csi->phy_master);
	if (NULL == dphy_master) {
		dev_err(dev, "Can not get dphy0 hander\n");
		return -EINVAL;
	}
	dphy_master->master = true;

	csi->phy_slave = NULL;
	if (2 == cnt) {
		csi->phy_slave = devm_of_phy_get_by_index(dev, dev->of_node, 1);
		if (IS_ERR(csi->phy_slave)) {
			dev_err(dev, "No DPHY1 available\n");
			return PTR_ERR(csi->phy_slave);
		}

		dphy_slave = phy_get_drvdata(csi->phy_slave);
		if (NULL == dphy_slave) {
			dev_err(dev, "Can not get dphy1 hander\n");
			return -EINVAL;
		}

		/* check two d-phy's at the same csi group */
		if (1 != (dphy_master->idx ^ dphy_slave->idx)) {
			dev_err(dev, "%pOF and %pOF can not combine as 4lane\n",
				 csi->phy_master->dev.of_node, csi->phy_slave->dev.of_node);
			return -EINVAL;
		}

		dphy_master->partner = dphy_slave;
		dphy_slave->partner = dphy_master;
		dphy_slave->master = false;
	}

	/*
	 * check lane idx
	 *
	 * good if data_lane = <1 2 3 4>, and at 4lane mode, or
	 * if data_lane = <1 2>, and at 2lane mode, also good.
	 *
	 * bad if data_lane = <1 2 7 4>, and at 4lane mode, or
	 * if data_lane = <1 3>, and at 2lane mode, also bad.
	 */
	lane_usebit = dphy_master->lane_usebit;
	if (dphy_slave) {
		lane_usebit <<= 2;
		lane_usebit |= dphy_slave->lane_usebit;
	}
	for (i = 0 ; i < csi->hw.num_lanes; i++) {
		unsigned char lane_idx = data_lanes[i];

		if (lane_idx)
			lane_bits |= BIT(lane_idx - 1);
	}

	if (lane_bits != lane_usebit) {
		dev_err(dev, "has can not support lane, lane bits = %08x lane_usebit = %08x\n",
			lane_bits, lane_usebit);
		return -EINVAL;
	}

	if (csi->phy_slave) {
		dev_info(dev, "%pOF and %pOF combine as %dlane\n",
				csi->phy_master->dev.of_node, csi->phy_slave->dev.of_node,
				csi->hw.num_lanes);
		dev_info(dev, "port bits = %08x lane bits = %08x\n",
			lane_bits, lane_usebit);
	} else {
		dev_info(dev, "%pOF configure as %dlane\n",
				csi->phy_master->dev.of_node, csi->hw.num_lanes);
		dev_info(dev, "port bits = %08x lane bits = %08x\n",
			lane_bits, lane_usebit);
	}

	return 0;
}

static int
dw_mipi_csi_parse_dt(struct platform_device *pdev, struct dw_csi *dev)
{
	struct device_node *node = pdev->dev.of_node;
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	int ret = 0;

	dev->index = of_alias_get_id(node, "csi");
	dev_info (&pdev->dev, "dev->index: %d\n",dev->index);
	if (of_property_read_u32(node, "output-type",&dev->hw.output)) {
		dev_dbg(&pdev->dev, "hw.output: %d\n", dev->hw.output);
	}

	node = of_graph_get_next_endpoint(node, NULL);
	if (!node) {
		dev_err(&pdev->dev, "No port node at %pOF\n", pdev->dev.of_node);
		return -EINVAL;
	}

	/* Get port node and validate MIPI-CSI channel id. */
	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(node), &ep);
	if (ret)
		goto err;

	dev->hw.num_lanes = ep.bus.mipi_csi2.num_data_lanes;
	dev_info (&pdev->dev, "num of lanes: %d\n", dev->hw.num_lanes);

#if IS_ENABLED(CONFIG_SIENGINE_DWC_MIPI_DPHY_RX)
	ret = dw_dphy_parse_dt(pdev, dev, &ep);
#endif

	/* parse clock */
	dev->clk_axi = of_clk_get_by_name(pdev->dev.of_node, DW_CSI_AXI_CLK);
	if (IS_ERR(dev->clk_axi))
		return PTR_ERR(dev->clk_axi);

	dev->clk_apb = of_clk_get_by_name(pdev->dev.of_node, DW_CSI_APB_CLK);
	if (IS_ERR(dev->clk_apb))
		return PTR_ERR(dev->clk_apb);

	dev->clk_ahb = of_clk_get_by_name(pdev->dev.of_node, DW_CSI_AHB_CLK);
	if (IS_ERR(dev->clk_ahb))
		return PTR_ERR(dev->clk_ahb);

	dev->clk_pixel = of_clk_get_by_name(pdev->dev.of_node, DW_CSI_PIXEL_CLK);
	if (IS_ERR(dev->clk_pixel))
		return PTR_ERR(dev->clk_pixel);

	dev->clk_isp_core = of_clk_get_by_name(pdev->dev.of_node, DW_CSI_ISP_CORE_CLK);
	if (IS_ERR(dev->clk_isp_core))
		return PTR_ERR(dev->clk_isp_core);

err:
	of_node_put(node);
	return ret;
}

static const struct of_device_id dw_mipi_csi_of_match[];

static int dw_clk_enable(struct dw_csi *csi, bool enable)
{
	unsigned long flags;

	if (enable) {
		spin_lock_irqsave(&csi->clk_lock, flags);
		if (clk_prepare_enable(csi->clk_axi) ||
			clk_prepare_enable(csi->clk_apb) ||
			clk_prepare_enable(csi->clk_ahb) ||
			clk_prepare_enable(csi->clk_pixel) ||
			clk_prepare_enable(csi->clk_isp_core)) {

			dev_info(csi->dev, "call clk enable failed\n");
			spin_unlock_irqrestore(&csi->clk_lock, flags);
			return -EINVAL;
		}
		spin_unlock_irqrestore(&csi->clk_lock, flags);
		dev_info(csi->dev, "call clk enable\n");
	} else {
		spin_lock_irqsave(&csi->clk_lock, flags);
		clk_disable_unprepare(csi->clk_axi);
		clk_disable_unprepare(csi->clk_apb);
		clk_disable_unprepare(csi->clk_ahb);
		clk_disable_unprepare(csi->clk_pixel);
		clk_disable_unprepare(csi->clk_isp_core);
		spin_unlock_irqrestore(&csi->clk_lock, flags);
		dev_info(csi->dev, "call clk disable\n");
	}

return 0;
}

static int dw_csi_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id = NULL;
	struct dw_csih_pdata *pdata = NULL;
	struct device *dev = &pdev->dev;
	struct resource *res = NULL;
	struct dw_csi *csi;
	struct v4l2_subdev *sd;
	int i, ret = -1;

	if (!IS_ENABLED(CONFIG_OF))
		pdata = pdev->dev.platform_data;

	dev_vdbg(dev, "Probing started\n");

	/* Resource allocation */
	csi = devm_kzalloc(dev, sizeof(*csi), GFP_KERNEL);
	if (!csi)
	{
		dev_err(dev, "Resource allocation failed.\n");
		return -ENOMEM;
	}

	mutex_init(&csi->lock);
	spin_lock_init(&csi->slock);
	spin_lock_init(&csi->clk_lock);
	csi->dev = dev;

	if (dev->of_node) {
		of_id = of_match_node(dw_mipi_csi_of_match, dev->of_node);
		if (!of_id)
			return -EINVAL;

		ret = dw_mipi_csi_parse_dt(pdev, csi);
		if (ret < 0)
			return ret;
	}

	/* enable clk */
	ret = dw_clk_enable(csi, true);
	if (ret != 0) {
		pr_err("try to enable cisp clk failed!\n");
		return -EINVAL;
	}

	/* Registers mapping */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	csi->base_address = devm_ioremap_resource(dev, res);
	if (IS_ERR(csi->base_address)) {
		dev_err(dev, "base address not set.\n");
		return PTR_ERR(csi->base_address);
	}
	dev_vdbg(dev, "csi baseaddress: 0x%llx\n", (uint64_t)csi->base_address);
	/* get irq resource */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	dev_vdbg(dev, "irq number: %d, irq flags: %d\n", (int)res->start, (int)res->flags);
	ret = devm_request_irq(dev, res->start, dw_mipi_csi_irq1,
		(res->flags & IRQF_TRIGGER_MASK) | IRQF_SHARED, dev_name(dev), csi);
	if (ret) {
		if (dev->of_node)
			dev_err(dev, "irq csi %s failed\n", of_id->name);
		else
			dev_err(dev, "irq csi %d failed\n", pdata->id);

		goto end;
	}
	sd = &csi->sd;
	v4l2_subdev_init(sd, &dw_mipi_csi_subdev_ops);
	csi->sd.owner = THIS_MODULE;

	if (dev->of_node) {
		snprintf(sd->name, sizeof(sd->name), "%s.%d", "dw-csi", csi->index);
		csi->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	} else {
		strlcpy(sd->name, dev_name(dev), sizeof(sd->name));
		strlcpy(csi->v4l2_dev.name, dev_name(dev), sizeof(csi->v4l2_dev.name));
	}

	//add fault solution
	csi->pix_height = DEFAULT_PIX_HEIGHT;
	csi->pix_width  = DEFAULT_PIX_WIDTH;

	csi->fmt = &dw_mipi_csi_formats[DEFAULT_FMT_INDEX];
	for (i=0; i<4; i++)
		csi->format[i].code = dw_mipi_csi_formats[DEFAULT_FMT_INDEX].mbus_code;

	sd->entity.function = MEDIA_ENT_F_IO_V4L;

	if (dev->of_node) {
		dev_info(dev, "media_entity_pads_init\n");
		csi->pads[CSI_PAD_VC0_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
		csi->pads[CSI_PAD_VC1_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
		csi->pads[CSI_PAD_VC2_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
		csi->pads[CSI_PAD_VC3_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
		csi->pads[CSI_PAD_VC0_SINK].flags = MEDIA_PAD_FL_SINK;
		csi->pads[CSI_PAD_VC1_SINK].flags = MEDIA_PAD_FL_SINK;
		csi->pads[CSI_PAD_VC2_SINK].flags = MEDIA_PAD_FL_SINK;
		csi->pads[CSI_PAD_VC3_SINK].flags = MEDIA_PAD_FL_SINK;

		ret = media_entity_pads_init(&csi->sd.entity, CSI_PADS_NUM, csi->pads);
		if (ret < 0) {
			dev_err(dev, "media entity init failed\n");
			goto end;
		}
		csi->sd.entity.ops = &mipi_csi2_sd_media_ops;
	} else {
		csi->hw.num_lanes = pdata->lanes;
		csi->hw.pclk = pdata->pclk;
		csi->hw.fps = pdata->fps;
		csi->hw.dphy_freq = pdata->hs_freq;

		ret = v4l2_device_register(NULL, &csi->v4l2_dev);
		if (ret) {
			dev_err(dev, "failed to register v4l2 device\n");
			goto end;
		}
	}
	dev_vdbg(dev, "v4l2.name: %s\n", csi->v4l2_dev.name);

	v4l2_set_subdevdata(&csi->sd, pdev);
	platform_set_drvdata(pdev, &csi->sd);
	dev_set_drvdata(dev, sd);

	if (csi->rst)
		reset_control_deassert(csi->rst);
#if IS_ENABLED(CONFIG_DWC_MIPI_TC_DPHY_GEN3)
	dw_csi_create_capabilities_sysfs(pdev);
#endif
	dw_mipi_csi_get_version(csi);
	dw_mipi_csi_specific_mappings(csi);
	dw_mipi_csi_mask_irq_power_off(csi);

	dev_info(dev, "DW MIPI CSI-2 Host registered successfully HW v%u.%u\n",
		 csi->hw_version_major, csi->hw_version_minor);
	//dev_info(dev, "DW MIPI CSI-2 Host registered successfully HW\n");

	for (i = 0; i < 4; i++)
		csi->running[i] = 0;

	csi->dw_power_flag = 0;
	csi->ipi_power_flag = 0;
	return 0;
end:
#if IS_ENABLED(CONFIG_OF)
	media_entity_cleanup(&csi->sd.entity);
	return ret;
#endif
	v4l2_device_unregister(csi->vdev.v4l2_dev);
	platform_set_drvdata(pdev, NULL);

	return ret;
}

static int dw_csi_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct dw_csi *mipi_csi = sd_to_mipi_csi_dev(sd);

	if (mipi_csi->rst)
		reset_control_assert(mipi_csi->rst);
#if IS_ENABLED(CONFIG_OF)
	media_entity_cleanup(&mipi_csi->sd.entity);
#else
	v4l2_device_unregister(mipi_csi->vdev.v4l2_dev);
#endif
	/* disable clk */
	dw_clk_enable(mipi_csi, false);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int dw_csi_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct dw_csi *mipi_csi = sd_to_mipi_csi_dev(sd);

	dw_clk_enable(mipi_csi, false);
	return 0;//pm_runtime_force_suspend(dev);
}

static int dw_csi_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct dw_csi *mipi_csi = sd_to_mipi_csi_dev(sd);

	//pm_runtime_force_suspend(dev);
	dw_clk_enable(mipi_csi, true);
	return 0;
}

#else
static int dw_csi_runtime_suspend(struct device *dev)
{
	return 0;
}

static int dw_csi_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops se_dw_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dw_csi_runtime_suspend,
						dw_csi_runtime_resume)
};

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id dw_mipi_csi_of_match[] = {
	{ .compatible = "dw-csi" },
	{},
};

MODULE_DEVICE_TABLE(of, dw_mipi_csi_of_match);
#endif

static struct platform_driver dw_mipi_csi_driver = {
	.remove = dw_csi_remove,
	.probe = dw_csi_probe,
	.driver = {
		.name = "dw-csi",
		.owner = THIS_MODULE,
		.pm = &se_dw_pm_ops,
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = of_match_ptr(dw_mipi_csi_of_match),
#endif
	},
};

module_platform_driver(dw_mipi_csi_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Luis Oliveira <luis.oliveira@synopsys.com>");
MODULE_DESCRIPTION("Synopsys DesignWare MIPI CSI-2 Host Platform driver");
