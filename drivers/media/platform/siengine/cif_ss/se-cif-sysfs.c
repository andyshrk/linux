// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021-2022 Siengine Technology, Inc.
 *
 * Sienginie Camera Interface driver
 * SysFS components for the platform driver
 *
 * Author: Siengine Technology, Inc.
 */

#include <linux/platform_device.h>
#include "se-cif-hw.h"

/* get the value of the 'bit'  */
#define	GET_BIT(x, bit)	((x & (1 << bit)) >> bit)
/* get the value from 'm' to 'n'. Note: m < n */
#define BIT_M_TO_N(x, m, n) ((unsigned int)(x << (31-(n))) >> ((31 - (n)) + (m)))

extern unsigned int cif_perf_enable;
extern unsigned int jitter_limit;

static ssize_t cif_id_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct se_cif_dev *se_cif = platform_get_drvdata(pdev);

	const int SIZE = 15;
	char buffer[SIZE];

	snprintf(buffer, SIZE, "cif_id: %d\n", se_cif->id);

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static ssize_t cif_clock_enable_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct se_cif_dev *se_cif = platform_get_drvdata(pdev);

	const int SIZE = 15;
	char buffer[SIZE];
	u32 value = 0;
	int temp = 0;

	value = readl(se_cif->regs + CIF_CLOCK_EN);
	temp = GET_BIT(value, 0);
	if (temp) {
		snprintf(buffer, SIZE, "cif clock on\n");
	} else {
		snprintf(buffer, SIZE, "cif clock off\n");
	}

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static ssize_t cif_status_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct se_cif_dev *se_cif = platform_get_drvdata(pdev);

	const int SIZE = 30;
	char buffer[SIZE];
	u32 value = 0;
	int temp = 0;

	value = readl(se_cif->regs + CIF_STATUS);
	temp = BIT_M_TO_N(value, 0, 1);
	switch (temp) {
		case 0:
			snprintf(buffer, SIZE, "cif is idle\n");
			break;
		case 1:
			snprintf(buffer, SIZE, "cif working, write buffer a\n");
			break;
		case 3:
			snprintf(buffer, SIZE, "cif working, write buffer b\n");
			break;
	}

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static ssize_t cif_setting_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct se_cif_dev *se_cif = platform_get_drvdata(pdev);

	const int SIZE = 30;
	char buffer[SIZE];
	u32 value = 0;
	int temp = 0;

	value = readl(se_cif->regs + CIF_SETTING);
	temp = BIT_M_TO_N(value, 0, 1);
	switch (temp) {
		case 0:
			snprintf(buffer, SIZE, "RAW ");
			break;
		case 1:
			snprintf(buffer, SIZE, "RGB888 ");
			break;
		case 2:
			snprintf(buffer, SIZE, "YUV422 ");
			break;
	}

	temp = BIT_M_TO_N(value, 2, 7);
	switch (temp) {
		case 8:
			snprintf(buffer + strlen(buffer), 16, "pixel_width:8 \n");
			break;
		case 10:
			snprintf(buffer + strlen(buffer), 16, "pixel_width:10 \n");
			break;
		case 12:
			snprintf(buffer + strlen(buffer), 16, "pixel_width:12 \n");
			break;
		case 14:
			snprintf(buffer + strlen(buffer), 16, "pixel_width:14 \n");
			break;
		case 16:
			snprintf(buffer + strlen(buffer), 16, "pixel_width:16 \n");
			break;
	}

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static ssize_t cif_geometry_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct se_cif_dev *se_cif = platform_get_drvdata(pdev);

	const int SIZE = 50;
	char buffer[SIZE];
	u32 value = 0;
	int temp = 0;

	value = readl(se_cif->regs + CIF_GEOMETRY);
	temp = BIT_M_TO_N(value, 0, 12);
	snprintf(buffer, SIZE, "num_pix_inline: %d ", temp);

	temp = BIT_M_TO_N(value, 16, 28);
	snprintf(buffer + strlen(buffer), 25, "num_pix_incol: %d\n", temp);

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static ssize_t cif_int_source_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct se_cif_dev *se_cif = platform_get_drvdata(pdev);

	const int SIZE = 30;
	char buffer[SIZE];
	u32 value = 0;
	int temp = 0;

	value = readl(se_cif->regs + CIF_INT_SOURCE);
	temp = BIT_M_TO_N(value, 0, 5);
	snprintf(buffer, SIZE, "int_source: 0x%X\n", temp);

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static ssize_t cif_int_mask_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct se_cif_dev *se_cif = platform_get_drvdata(pdev);

	const int SIZE = 30;
	char buffer[SIZE];
	u32 value = 0;
	int temp = 0;

	value = readl(se_cif->regs + CIF_INT_MASK);
	temp = BIT_M_TO_N(value, 0, 5);
	snprintf(buffer, SIZE, "int_mask: 0x%X\n", temp);

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static ssize_t cif_int_status_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct se_cif_dev *se_cif = platform_get_drvdata(pdev);

	const int SIZE = 30;
	char buffer[SIZE];
	u32 value = 0;
	int temp = 0;

	value = readl(se_cif->regs + CIF_INT_STATUS);
	temp = BIT_M_TO_N(value, 0, 5);
	snprintf(buffer, SIZE, "int_status: 0x%X\n", temp);

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static ssize_t cif_ipi_select_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct se_cif_dev *se_cif = platform_get_drvdata(pdev);

	const int SIZE = 15;
	char buffer[SIZE];
	u32 value = 0;
	int temp = 0;
	int i;

	value = readl(se_cif->regs + CIF_IPI_SELECT);
	for (i = 0; i < 32; i++) {
		if (GET_BIT(value, i)) {
			temp = i;
			break;
		}
	}
	snprintf(buffer, SIZE, "ipi_select: %d\n", temp);

	return strlcpy(buf, buffer, PAGE_SIZE);
}

static int cif_perf_dump(struct se_cif_dev *se_cif)
{
	u64 interval = 0;

	se_cif->cif_cap.frame_end_ns = ktime_get_ns();
	interval = (se_cif->cif_cap.frame_end_ns -
						se_cif->cif_cap.frame_start_ns) / 1000000;

	if (se_cif->cif_cap.frame_count < 2) {
		se_cif->cif_cap.total_hw_fps = 0;
		se_cif->cif_cap.total_ac_fps = 0;
	} else {
		se_cif->cif_cap.total_hw_fps = (se_cif->cif_cap.frame_count - 2) *
						1000 / interval;
		se_cif->cif_cap.total_ac_fps = (se_cif->cif_cap.frame_count - 2 -
						se_cif->cif_cap.loss_count) * 1000 / interval;
	}
	return 0;
}

static ssize_t cif_perf_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct se_cif_dev *se_cif = platform_get_drvdata(pdev);

	if (cif_perf_enable) {
		cif_perf_dump(se_cif);
		return sprintf(buf, "*****CIF Performance Status*****\n "
							"---Total FPS---\n\t"
							"hardware FPS: %d\t actual FPS: %d\n\n "
							"---Current FPS---\n\t"
							"hardware FPS: %d\t actual FPS: %d\n\n "
							"---Hardware Frame---\n\t"
							"frame count: %d\n\n "
							"---Loss Frame---\n\t"
							"loss count: %d\n\n "
							"---Jitter Frame---\n\t"
							"jitter count(jitter limit = %d): %d\n\n "
							"---Buffer Time---\n\t"
							"DQBUF time - QBUF time: %d(us)\n"
							"********************************\n",
				se_cif->cif_cap.total_hw_fps, se_cif->cif_cap.total_ac_fps,
				se_cif->cif_cap.curr_hw_fps, se_cif->cif_cap.curr_ac_fps,
				se_cif->cif_cap.frame_count, se_cif->cif_cap.loss_count,
				jitter_limit, se_cif->cif_cap.jitter_count,
				se_cif->cif_cap.interval_qdq);
	} else {
		return sprintf(buf, "cif_perf_enable not set!\n");
	}
}

static DEVICE_ATTR_RO(cif_id);
static DEVICE_ATTR_RO(cif_clock_enable);
static DEVICE_ATTR_RO(cif_status);
static DEVICE_ATTR_RO(cif_setting);
static DEVICE_ATTR_RO(cif_geometry);
static DEVICE_ATTR_RO(cif_int_source);
static DEVICE_ATTR_RO(cif_int_mask);
static DEVICE_ATTR_RO(cif_int_status);
static DEVICE_ATTR_RO(cif_ipi_select);
static DEVICE_ATTR_RO(cif_perf);

int se_cif_create_capabilities_sysfs(struct platform_device *pdev)
{
	device_create_file(&pdev->dev, &dev_attr_cif_id);
	device_create_file(&pdev->dev, &dev_attr_cif_clock_enable);
	device_create_file(&pdev->dev, &dev_attr_cif_status);
	device_create_file(&pdev->dev, &dev_attr_cif_setting);
	device_create_file(&pdev->dev, &dev_attr_cif_geometry);
	device_create_file(&pdev->dev, &dev_attr_cif_int_source);
	device_create_file(&pdev->dev, &dev_attr_cif_int_mask);
	device_create_file(&pdev->dev, &dev_attr_cif_int_status);
	device_create_file(&pdev->dev, &dev_attr_cif_ipi_select);
	device_create_file(&pdev->dev, &dev_attr_cif_perf);

	return 0;
}
