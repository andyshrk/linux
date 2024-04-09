// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Siengine
 */

#ifndef _KOMEDA_KMS_RPMSG_H
#define _KOMEDA_KMS_RPMSG_H

#include "komeda_kms_agent.h"
#include "komeda_kms_ipc.h"
#include <linux/rpmsg.h>
#include <linux/time64.h>
#include <linux/cdev.h>
#include <linux/rpmsg/siengine-rpmsg.h>
#include <linux/rpmsg.h>

enum rpmsg_id {
	RPMSG_SRC = 0,
	RPMSG_DST = 1,
};

struct komeda_kms_rpmsg_ch {
	u32 src;
	u32 dst;
};

struct komeda_kms_rpmsg {
	struct device_node *np;
	uint32_t slot;
	uint32_t vdomain;
	struct komeda_kms_rpmsg_ch rp_ch;
	struct rpmsg_device *rp_dev;
	struct komeda_kms_buf_info info;
	struct komeda_kms_ipc *ipc;
	void *rpmsg_ept;
	bool is_registered;
	struct completion register_done;
	phys_addr_t last_buf;
	uint64_t rx_buf_num;
	struct komeda_kms_buf_addr *addr;
	struct timespec64 start_tv;
	uint32_t pipe_id;
};

enum komeda_kms_rpmsg_data_type {
	RPMSG_REGISTER = 1,
	RPMSG_DEREGISTER,
	RPMSG_FORMAT_UPDATE,
	RPMSG_BUF_UPDATE,
	RPMSG_BUF_DONE,
	RPMSG_CLIENT_READY,
	RPMSG_CLIENT_NOT_READY,
	RPMSG_CLIENT_NOTIFY,
	RPMSG_PREPARE_DEREGISTER,
	RPMSG_MAX,
};

struct komeda_kms_rpmsg_payload {
	enum komeda_kms_rpmsg_data_type type;
	unsigned int id;
	union {
		struct komeda_kms_buf_addr buf_addr;
		struct komeda_kms_buf_info buf_info;
	} buff;
}__attribute__((aligned(8)));

struct komeda_kms_rpmsg *
komeda_kms_parse_rpmsg_from_dt(struct device_node *np, void *ipc);
int komeda_kms_rpmsg_init(struct komeda_kms_rpmsg *rpmsg);

#endif