// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Siengine
 */

#ifndef _KOMEDA_KMS_AGENT_H
#define _KOMEDA_KMS_AGENT_H

#include <linux/types.h>
#include <drm/drm_framebuffer.h>
#include "komeda_kms_agent_common.h"

struct komeda_kms_agent;

struct agent_pipeline {
	uint32_t layer_id;
	uint32_t pipe_id;
	struct komeda_kms_ipc *ipc;
	bool allocated;
	struct komeda_kms_buf_info info;
	struct komeda_kms_agent_fb *agent_fb[BUF_NUM];
	struct drm_device *drm_dev;
	struct komeda_kms_agent *agent;
	bool other_plane_disabled;
	bool configed;
	struct mutex mutex;
	uint32_t buf_id;
	struct completion alloc_done[BUF_NUM];
};

struct komeda_kms_agent_fb {
	int index;
	struct drm_framebuffer *fb;
	struct drm_gem_cma_object *cma_obj;
	phys_addr_t paddr;
	size_t size;
	unsigned char *vaddr;
	bool used;
	struct komeda_kms_buf_info info;
};

struct komeda_kms_agent_handler {
	int (*format_update)(struct komeda_kms_agent *agent, unsigned int id,
				struct komeda_kms_buf_fmt *fmt);
	int (*buffer_update)(struct komeda_kms_agent *agent, unsigned int id,
				struct komeda_kms_buf_addr *addr);
	int (*prepare_deregister)(struct komeda_kms_agent *agent,
					unsigned int id);
};

struct komeda_kms_agent_dma {
	struct dma_chan *chan;
	char chn_name[5];
	struct completion xfer_done;
	int id;
};

struct komeda_kms_agent {
	struct device *dev;
	struct drm_device *drm_dev;
	uint32_t ppls_num;
	struct agent_pipeline *ppl;
	dma_addr_t resv_pa;
	void *vaddr;
	size_t resv_size;
	struct list_head list;
	struct device_node *dpu_node;
	struct komeda_kms_agent_handler *handler;
	struct completion bind_done;
	struct komeda_kms_agent_dma agent_dma[MAX_DMA_CHN];
};

int komeda_kms_agent_attach_drm_dev(struct drm_device *dev);

#endif