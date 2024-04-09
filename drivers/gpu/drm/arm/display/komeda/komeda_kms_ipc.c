// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Siengine
 */

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/sysrq.h>
#include <linux/dev_printk.h>
#include <linux/slab.h>
#include <linux/of.h>
#include "komeda_kms_ipc.h"
#include "komeda_kms_rpmsg.h"
#include "komeda_kms_agent_debug.h"

static char *get_ipc_type_str(enum komemda_kms_ipc_type type)
{
	switch (type) {
	case IPC_TYPE_RPMSG:
		return "RPMSG";
	default:
		KOMEDA_IPC_ERR("get invalid ipc_type %d", type);
		return NULL;
	}
}

static int komeda_kms_ipc_format_update(struct komeda_kms_ipc *ipc,
		unsigned int id, struct komeda_kms_buf_fmt *fmt)
{
	if (!ipc) {
		KOMEDA_IPC_ERR("invalid ipc");
		return -EINVAL;
	}

	if (!fmt) {
		KOMEDA_IPC_ERR("invalid fmt");
		return -EINVAL;
	}

	if (ipc->agent && ipc->agent->handler) {
		struct komeda_kms_agent_handler *handler;

		handler = ipc->agent->handler;
		if (handler->format_update)
			return handler->format_update(ipc->agent, id, fmt);
	}

	KOMEDA_IPC_ERR("agent handler has not been registerd");

	return -EPERM;
}

static int komeda_kms_ipc_buffer_update(struct komeda_kms_ipc *ipc,
		unsigned int id, struct komeda_kms_buf_addr *addr)
{
	if (!ipc) {
		KOMEDA_IPC_ERR("invalid ipc");
		return -EINVAL;
	}

	if (!addr) {
		KOMEDA_IPC_ERR("invalid addr");
		return -EINVAL;
	}

	if (ipc->agent && ipc->agent->handler) {
		struct komeda_kms_agent_handler *handler;

		handler = ipc->agent->handler;
		if (handler->buffer_update)
			return handler->buffer_update(ipc->agent, id, addr);
	}

	KOMEDA_IPC_ERR("buffer update can not be handled");

	return -EPERM;
}

static int
komeda_kms_ipc_prepare_deregister(struct komeda_kms_ipc *ipc,
						unsigned int id)
{
	if (!ipc) {
		KOMEDA_IPC_ERR("invalid ipc");
		return -EINVAL;
	}

	if (ipc->agent && ipc->agent->handler) {
		struct komeda_kms_agent_handler *handler;

		handler = ipc->agent->handler;
		if (handler->prepare_deregister)
			return handler->prepare_deregister(ipc->agent, id);
	}

	KOMEDA_IPC_ERR("prepare_deregister can not be handled");

	return -EPERM;
}

static struct komeda_kms_ipc_handler ipc_handler = {
	.format_update = komeda_kms_ipc_format_update,
	.buffer_update = komeda_kms_ipc_buffer_update,
	.prepare_deregister = komeda_kms_ipc_prepare_deregister,
};

struct komeda_kms_ipc *
komeda_kms_parse_ipc_dt(struct komeda_kms_agent *agent, struct device_node *np)
{
	int ret = 0;
	u32 type;
	struct komeda_kms_ipc *ipc;

	if (!np) {
		KOMEDA_IPC_ERR("invalid device node");
		return NULL;
	}

	ret = of_property_read_u32(np, "type", &type);
	if (ret) {
		KOMEDA_IPC_ERR("get type property fail");
		return NULL;
	}

	if (type >= IPC_TYPE_MAX) {
		KOMEDA_IPC_ERR("get invalid type %u. max is %u",
			type, IPC_TYPE_MAX);
		return NULL;
	}

	ipc = (struct komeda_kms_ipc *)kzalloc(sizeof(*ipc), GFP_KERNEL);
	if (!ipc) {
		KOMEDA_IPC_ERR("fail to alloc mem for ipc");
		return NULL;
	}

	ipc->ipc_type = type;
	KOMEDA_IPC_DEBUG("get ipc_type %s", get_ipc_type_str(type));

	if (ipc->ipc_type == IPC_TYPE_RPMSG) {
		ipc->priv = komeda_kms_parse_rpmsg_from_dt(np, ipc);
		if (!ipc->priv) {
			KOMEDA_IPC_ERR("parse rpmsg fail");
			goto parse_ipc_dt_clean;
		}

		ipc->agent = agent;
	}

	return ipc;

parse_ipc_dt_clean:
	if (ipc)
		kfree(ipc);

	return NULL;
}

int komeda_kms_agent_ipc_init(struct komeda_kms_ipc *ipc)
{
	int ret;

	if (!ipc) {
		KOMEDA_IPC_ERR("invalid ipc");
		return -EINVAL;
	}

	ipc->handler = &ipc_handler;

	if (ipc->ipc_type == IPC_TYPE_RPMSG) {
		ret = komeda_kms_rpmsg_init(ipc->priv);
		if (ret) {
			KOMEDA_IPC_ERR("komeda_kms_rpmsg_init fail");
			return ret;
		}
	} else {
		KOMEDA_IPC_ERR("invalid type %d", ipc->ipc_type);
		return -EINVAL;
	}

	return 0;
}