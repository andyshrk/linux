// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021-2022 Siengine Technology, Inc.
 *
 * Sienginie Camera Interface driver
 *
 * Author: Siengine Technology, Inc.
 */

#ifndef SE_MEDIA_DEV_H_
#define SE_MEDIA_DEV_H_

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/bug.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/list.h>
#include <linux/mfd/syscon.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/dwc/dw-mipi-csi-pltfrm.h>

#include "se-cif-core.h"
#include "dw-mipi-csi.h"
#include "se-cif-hw.h"

#define SE_MD_DRIVER_NAME	"se-md"

/* The media device includes only 1 * MIPI CSI2, 4 * CIF */
#define SE_MAX_SENSORS 8
#define SE_MD_MIPI_CSI2_MAX_DEVS 8
#define SE_MD_CIF_MAX_DEVS 22
#define MIPI_CSI2_SENS_VCX_PADS_NUM 4

/*
 * The subdevices' group IDs.
 */

enum se_subdev_index {
	IDX_SENSOR,
	IDX_CIF,
	IDX_MIPI_CSI2,
	IDX_MAX,
};

struct imxdpu_videomode {
	char name[64];		/* may not be needed */

	uint32_t pixelclock;	/* Hz */

	/* htotal (pixels) = hlen + hfp + hsync + hbp */
	uint32_t hlen;
	uint32_t hfp;
	uint32_t hbp;
	uint32_t hsync;

	/* field0 - vtotal (lines) = vlen + vfp + vsync + vbp */
	uint32_t vlen;
	uint32_t vfp;
	uint32_t vbp;
	uint32_t vsync;

	/* field1  */
	uint32_t vlen1;
	uint32_t vfp1;
	uint32_t vbp1;
	uint32_t vsync1;

	uint32_t flags;

	uint32_t format;
	uint32_t dest_format; /*buffer format for capture*/

	int16_t clip_top;
	int16_t clip_left;
	uint16_t clip_width;
	uint16_t clip_height;
};

struct sensor_data {
	struct v4l2_subdev	subdev;
	struct media_pad pads[MIPI_CSI2_SENS_VCX_PADS_NUM];
	struct i2c_client *i2c_client;
	struct v4l2_mbus_framefmt format;
	struct v4l2_captureparm streamcap;
	char running;

	/* control settings */
	int brightness;
	int hue;
	int contrast;
	int saturation;
	int red;
	int green;
	int blue;
	int ae_mode;

	u32 mclk;
	u8 mclk_source;
	struct clk *sensor_clk;
	int v_channel;
	bool is_mipi;
	struct imxdpu_videomode cap_mode;

	unsigned int sensor_num;       /* sensor num connect max96715 */
	unsigned char sensor_is_there; /* Bit 0~3 for 4 cameras, 0b1= is there; 0b0 = not there */
	int pwn_gpio;
};

struct se_sensor_info {
	int id;
	struct v4l2_subdev *sd;
	struct v4l2_async_subdev asd;
	struct sensor_data *data;
	bool mipi_mode;
};

struct se_md {
	struct se_cif_dev *se_cif[SE_MD_CIF_MAX_DEVS];	//4 CIF dev with 1 CSI dev
	struct dw_csi *mipi_csi2[SE_MD_MIPI_CSI2_MAX_DEVS];
	struct se_sensor_info sensor[SE_MAX_SENSORS];

	int link_status;
	int num_sensors;

	struct media_device media_dev;
	struct v4l2_device v4l2_dev;
	struct platform_device *pdev;

	struct v4l2_async_notifier subdev_notifier;
	struct v4l2_async_subdev *async_subdevs[SE_MAX_SENSORS];
};

static inline struct se_md *notifier_to_se_md(struct v4l2_async_notifier *n)
{
	return container_of(n, struct se_md, subdev_notifier);
};

void se_sensor_notify(struct v4l2_subdev *sd, unsigned int notification,
			void *arg);
int se_cif_initialize_capture_subdev(struct se_cif_dev *se_cif);

void se_cif_unregister_capture_subdev(struct se_cif_dev *se_cif);

#endif
