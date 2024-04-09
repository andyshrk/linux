// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Siengine
 */

#ifndef _KOMEDA_KMS_AGENT_COMMON_H
#define _KOMEDA_KMS_AGENT_COMMON_H

#include <linux/types.h>
#include <drm/drm_fourcc.h>

#define BUF_NUM 2
#define MAX_PIPELINE 1
#define MAX_DMA_CHN 2

enum komeda_kms_agent_fmt {
	RGB888 = 0,
	ARGB8888,
	ABGR8888,
	YUYV,
	NV12,
	MAX_FMT,
};

struct komeda_kms_buf_addr {
	union {
		unsigned int offset;
		phys_addr_t paddr;
	} addr;
};

struct komeda_kms_buf_fmt {
	int	src_x;
	int	src_y;
	int	src_buf_w;
	int	src_buf_h;
	int	src_crop_w;
	int	src_crop_h;
	int	dst_x;
	int	dst_y;
	int	dst_w;
	int	dst_h;
	enum komeda_kms_agent_fmt fmt;
	size_t size;
};

struct komeda_kms_buf_info {
	struct komeda_kms_buf_fmt fmt;
	uint32_t drm_fmt;
	size_t size;
	struct komeda_kms_buf_addr addr;
};

#endif