// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Siengine
 */

#ifndef _KOMEDA_KMS_IPC_H
#define _KOMEDA_KMS_IPC_H

#include "komeda_kms_agent.h"

#define MAX_IPC_PIPELINE 2

enum komemda_kms_ipc_type {
	IPC_TYPE_RPMSG = 0,
	IPC_TYPE_MAX,
};

struct komeda_kms_ipc_handler {
	int (*format_update)(struct komeda_kms_ipc *ipc, unsigned int id,
				struct komeda_kms_buf_fmt *fmt);
	int (*buffer_update)(struct komeda_kms_ipc *ipc, unsigned int id,
				struct komeda_kms_buf_addr *addr);
	int (*prepare_deregister)(struct komeda_kms_ipc *ipc, unsigned int id);
};

struct komeda_kms_ipc {
	enum komemda_kms_ipc_type ipc_type;
	struct komeda_kms_ipc_handler *handler;
	void *priv;
	struct komeda_kms_agent *agent;
};

struct komeda_kms_ipc *
komeda_kms_parse_ipc_dt(struct komeda_kms_agent *agent,
				struct device_node *np);
int komeda_kms_agent_ipc_init(struct komeda_kms_ipc *ipc);

#endif