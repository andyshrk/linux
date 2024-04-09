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
#include <linux/kthread.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/timekeeping.h>
#include "komeda_kms_rpmsg.h"
#include "komeda_kms_agent_debug.h"

#define FPS_NUM 30
#define FPS_DEBUG 0

static char *type_to_str(const enum komeda_kms_rpmsg_data_type type)
{
	switch (type) {
	case RPMSG_REGISTER:
		return "RPMSG_REGISTER";
	case RPMSG_DEREGISTER:
		return "RPMSG_DEREGISTER";
	case RPMSG_FORMAT_UPDATE:
		return "RPMSG_FORMAT_UPDATE";
	case RPMSG_BUF_UPDATE:
		return "RPMSG_BUF_UPDATE";
	case RPMSG_BUF_DONE:
		return "RPMSG_BUF_DONE";
	case RPMSG_CLIENT_READY:
		return "RPMSG_CLIENT_READY";
	case RPMSG_CLIENT_NOT_READY:
		return "RPMSG_CLIENT_NOT_READY";
	case RPMSG_CLIENT_NOTIFY:
		return "RPMSG_CLIENT_NOTIFY";
	case RPMSG_PREPARE_DEREGISTER:
		return "RPMSG_PREPARE_DEREGISTER";
	default:
		return "INVALID";
	}
}

static char *get_fmt_str(const enum komeda_kms_agent_fmt fmt)
{
	switch (fmt) {
	case RGB888:
		return "RGB888";
	case ARGB8888:
		return "ARGB8888";
	case ABGR8888:
		return "ABGR8888";
	case YUYV:
		return "YUYV";
	case NV12:
		return "NV12";
	default:
		return "INVALID";
	}
}

static int
komeda_kms_rpmsg_send_msg(struct komeda_kms_rpmsg *rpmsg, void *payload)
{
	int ret = 0;
	struct komeda_kms_rpmsg_payload *rpmsg_data = payload;

	if (!rpmsg) {
		KOMEDA_RPMSG_ERR("invalid rpmsg");
		return -EINVAL;
	}

	if (!payload) {
		KOMEDA_RPMSG_ERR("invalid payload");
		return -EINVAL;
	}

	ret = rpmsg_send(rpmsg->rpmsg_ept, rpmsg_data, sizeof(*rpmsg_data));
	if (ret < 0)
		KOMEDA_RPMSG_ERR("RPMsg send failed! src:0x%x dst:0x%x "
			"type[%s] id[%d] ret:%d",
			rpmsg->rp_ch.src, rpmsg->rp_ch.dst,
			type_to_str(rpmsg_data->type), rpmsg_data->id, ret);
	else
		KOMEDA_RPMSG_DEBUG("RPMsg send success! src:0x%x dst:0x%x "
			"type[%s] id[%d]",
			rpmsg->rp_ch.src, rpmsg->rp_ch.dst,
			type_to_str(rpmsg_data->type), rpmsg_data->id);

	return ret;
}

static int komeda_kms_rpmsg_register(struct komeda_kms_rpmsg *rpmsg)
{
	struct komeda_kms_rpmsg_payload payload;
	int ret, i;

	if (!rpmsg) {
		KOMEDA_RPMSG_ERR("invalid rpmsg");
		return -EINVAL;
	}

	rpmsg->is_registered = false;
	memset(&payload, 0, sizeof(payload));
	payload.type = RPMSG_REGISTER;
	payload.id = BIT(rpmsg->slot);

	for (i = 0; i < 20; i++) {
		ret = komeda_kms_rpmsg_send_msg(rpmsg, &payload);
		if (!ret || rpmsg->is_registered) {
			KOMEDA_RPMSG_DEBUG("komeda kms rpmsg Register success!");
			rpmsg->is_registered = true;
			return 0;
		}
		msleep(200);
	}
	KOMEDA_RPMSG_ERR("komeda kms rpmsg try register timeout! try %d times", i);
	return -ETIMEDOUT;
}

static int komeda_kms_rpmsg_deregister(struct komeda_kms_rpmsg *rpmsg)
{
	struct komeda_kms_rpmsg_payload payload;
	int ret;

	if (!rpmsg) {
		KOMEDA_RPMSG_ERR("invalid rpmsg");
		return -EINVAL;
	}

	if (!rpmsg->is_registered) {
		KOMEDA_RPMSG_ERR("rpmsg has not been registered");
		return 0;
	}

	memset(&payload, 0, sizeof(payload));
	payload.type = RPMSG_DEREGISTER;
	payload.id = BIT(rpmsg->slot);

	ret = komeda_kms_rpmsg_send_msg(rpmsg, &payload);
	if (ret >= 0) {
		KOMEDA_RPMSG_DEBUG("src[0x%x] dst[0x%x] deregister success!",
			rpmsg->rp_ch.src, rpmsg->rp_ch.dst);
		rpmsg->is_registered = false;
		rpmsg_destroy_ept(rpmsg->rpmsg_ept);
		kfree(rpmsg);
		rpmsg = NULL;

		return 0;
	}

	KOMEDA_RPMSG_ERR("rpmsg deregister fail");
	return -ETIMEDOUT;
}

static int komeda_kms_rpmsg_client_notify(struct komeda_kms_rpmsg *rpmsg,
		struct komeda_kms_rpmsg_payload *payload)
{
	int ret;

	if (!rpmsg) {
		KOMEDA_RPMSG_ERR("invalid rpmsg");
		return -EINVAL;
	}

	if (!payload) {
		KOMEDA_RPMSG_ERR("invalid payload");
		return -EINVAL;
	}

	KOMEDA_RPMSG_DEBUG("pipe%d receive client notify", rpmsg->pipe_id);

	ret = komeda_kms_rpmsg_register(rpmsg);
	if (ret) {
		KOMEDA_RPMSG_ERR("komeda_kms_rpmsg_register failed");
		return ret;
	}

	KOMEDA_RPMSG_DEBUG("pipe%d client notify has been handled", rpmsg->pipe_id);

	return 0;
}

static int komeda_kms_rpmsg_ready(struct komeda_kms_rpmsg *rpmsg)
{
	struct komeda_kms_rpmsg_payload payload;

	if (!rpmsg) {
		KOMEDA_RPMSG_ERR("invalid rpmsg");
		return -EINVAL;
	}

	memset(&payload, 0, sizeof(payload));
	payload.type = RPMSG_CLIENT_READY;
	payload.id = BIT(rpmsg->slot);

	return komeda_kms_rpmsg_send_msg(rpmsg, &payload);
}

static int komeda_kms_rpmsg_format_update(struct komeda_kms_rpmsg *rpmsg,
		struct komeda_kms_rpmsg_payload *payload)
{
	int ret;

	if (!rpmsg) {
		KOMEDA_RPMSG_ERR("invalid rpmsg");
		return -EINVAL;
	}

	if (!payload) {
		KOMEDA_RPMSG_ERR("invalid payload");
		return -EINVAL;
	}

	if (payload->buff.buf_info.fmt.fmt >= MAX_FMT) {
		KOMEDA_RPMSG_ERR("invalid format num %d exceed to %d",
			payload->buff.buf_info.fmt.fmt, MAX_FMT - 1);
		return -EINVAL;
	}

	memcpy(&rpmsg->info.fmt, &payload->buff.buf_info.fmt, sizeof(rpmsg->info.fmt));

	if (!rpmsg->info.fmt.src_buf_w || !rpmsg->info.fmt.src_buf_h ||
		!rpmsg->info.fmt.src_crop_w || !rpmsg->info.fmt.src_crop_h ||
		!rpmsg->info.fmt.dst_w || !rpmsg->info.fmt.dst_h) {
		KOMEDA_RPMSG_ERR("src_buf [%d/%d] src_crop [%d/%d] dst [%d/%d]",
				rpmsg->info.fmt.src_buf_w, rpmsg->info.fmt.src_buf_h,
				rpmsg->info.fmt.src_crop_w, rpmsg->info.fmt.src_crop_h,
				rpmsg->info.fmt.dst_w, rpmsg->info.fmt.dst_h);
		return -EINVAL;
	}

	KOMEDA_RPMSG_DEBUG("pipe%d header crtc [x/y/w/h] [%d/%d/%d/%d] "
		"src [x/y/w/h] [%d/%d/%d/%d] fb [%d/%d] fmt:%s",
		rpmsg->pipe_id,
		rpmsg->info.fmt.dst_x, rpmsg->info.fmt.dst_y,
		rpmsg->info.fmt.dst_w, rpmsg->info.fmt.dst_h,
		rpmsg->info.fmt.src_x, rpmsg->info.fmt.src_y,
		rpmsg->info.fmt.src_crop_w, rpmsg->info.fmt.src_crop_h,
		rpmsg->info.fmt.src_buf_w, rpmsg->info.fmt.src_buf_h,
		get_fmt_str(rpmsg->info.fmt.fmt));

	if (rpmsg->ipc && rpmsg->ipc->handler) {
		struct komeda_kms_ipc_handler *handler = rpmsg->ipc->handler;

		if (handler->format_update) {
			ret = handler->format_update(rpmsg->ipc,
				rpmsg->pipe_id, &payload->buff.buf_info.fmt);
			if (ret) {
				KOMEDA_RPMSG_ERR("pipe%d format update failed",
						rpmsg->pipe_id);
				return ret;
			}

			ret = komeda_kms_rpmsg_ready(rpmsg);
			if (ret) {
				KOMEDA_RPMSG_ERR("komeda_kms_rpmsg_ready failed");
				return ret;
			}

			return 0;
		}
	}

	KOMEDA_RPMSG_ERR("ipc handler has not been registerd");

	return -EPERM;
}

static int komeda_kms_rpmsg_buf_done(struct komeda_kms_rpmsg *rpmsg)
{
	struct komeda_kms_rpmsg_payload payload;

	if (!rpmsg) {
		KOMEDA_RPMSG_ERR("invalid rpmsg");
		return -EINVAL;
	}

	memset(&payload, 0, sizeof(payload));
	payload.type = RPMSG_BUF_DONE;
	payload.id = BIT(rpmsg->slot);
	payload.buff.buf_addr.addr.paddr = rpmsg->last_buf;

	KOMEDA_RPMSG_DEBUG("free pipe%d, addr:0x%llx", rpmsg->pipe_id,
				payload.buff.buf_addr.addr.paddr);

	return komeda_kms_rpmsg_send_msg(rpmsg, &payload);
}

static int komeda_kms_rpmsg_prepare_deregister(struct komeda_kms_rpmsg *rpmsg)
{
	int ret;

	if (!rpmsg) {
		KOMEDA_RPMSG_ERR("invalid rpmsg");
		return -EINVAL;
	}

	if (rpmsg->ipc && rpmsg->ipc->handler) {
		struct komeda_kms_ipc_handler *ipc_handler;

		ipc_handler = rpmsg->ipc->handler;
		if (ipc_handler->prepare_deregister) {
			ret = ipc_handler->prepare_deregister(rpmsg->ipc, rpmsg->pipe_id);
			if (ret) {
				KOMEDA_RPMSG_ERR("destroy fail");
				return ret;
			}

			return 0;
		}
	}

	KOMEDA_RPMSG_ERR("prepare deregister can not be handled");

	return -EPERM;

}

static int
komeda_kms_rpmsg_buffer_update(struct komeda_kms_rpmsg *rpmsg,
			struct komeda_kms_rpmsg_payload *payload)
{
	int ret;
	struct komeda_kms_buf_addr *addr;

	if (!rpmsg) {
		KOMEDA_RPMSG_ERR("invalid rpmsg");
		return -EINVAL;
	}

	if (!payload) {
		KOMEDA_RPMSG_ERR("invalid payload");
		return -EINVAL;
	}

	if (!rpmsg->is_registered) {
		KOMEDA_RPMSG_ERR("src[0x%x] dst[0x%x] has not registered",
			rpmsg->rp_ch.src, rpmsg->rp_ch.dst);
		return -EPERM;
	}

	addr = &payload->buff.buf_addr;
	KOMEDA_RPMSG_DEBUG("receive pipe%d addr:0x%llx",
		rpmsg->pipe_id, addr->addr.paddr);

#if FPS_DEBUG
	if (rpmsg->rx_buf_num == 0)
		ktime_get_boottime_ts64(&rpmsg->start_tv);
#endif
	rpmsg->addr = addr;

	if (rpmsg->ipc && rpmsg->ipc->handler) {
		struct komeda_kms_ipc_handler *ipc_handler;

		ipc_handler = rpmsg->ipc->handler;
		if (ipc_handler->buffer_update) {
			ret = ipc_handler->buffer_update(rpmsg->ipc, rpmsg->pipe_id, addr);
			if (ret) {
				KOMEDA_RPMSG_INFO("buf update fail, ret:%d", ret);
				komeda_kms_rpmsg_deregister(rpmsg);
				return ret;
			}
#if FPS_DEBUG
			rpmsg->rx_buf_num++;
			if ((rpmsg->rx_buf_num % FPS_NUM) == 0) {
				struct timespec64 end_tv;
				int64_t diff, diff_s, diff_ns;
				int fps_int, fps_dec;

				ktime_get_boottime_ts64(&end_tv);
				diff_s = end_tv.tv_sec - rpmsg->start_tv.tv_sec;
				diff_ns = end_tv.tv_nsec - rpmsg->start_tv.tv_nsec;
				diff = diff_s * 1000 + diff_ns / 1000000;
				fps_int = FPS_NUM * 1000 / diff;
				fps_dec = ((FPS_NUM * 1000) % diff) * 100 / diff;
				KOMEDA_RPMSG_ERR("fps:%d.%d", fps_int, fps_dec);
				ktime_get_boottime_ts64(&rpmsg->start_tv);
			}
#endif
			KOMEDA_RPMSG_DEBUG("last buf:%llx", rpmsg->last_buf);
			rpmsg->info.addr.addr.paddr = rpmsg->last_buf;
			rpmsg->last_buf = addr->addr.paddr;
			return komeda_kms_rpmsg_buf_done(rpmsg);
		}
	}

	KOMEDA_RPMSG_ERR("buffer update can not be handled");

	return -EPERM;
}

static int komeda_kms_rpmsg_rx_cb(struct rpmsg_device *rpdev, void *data, int len,
					void *priv, u32 src)
{
	struct komeda_kms_rpmsg *rpmsg = priv;
	struct komeda_kms_rpmsg_payload *payload = data;
	int ret = 0;

	KOMEDA_RPMSG_DEBUG("payload type[%s] id[%u] src[0x%x]",
		type_to_str(payload->type), payload->id, src);

	switch (payload->type) {
	case RPMSG_CLIENT_NOTIFY:
		ret = komeda_kms_rpmsg_client_notify(rpmsg, payload);
		break;
	case RPMSG_FORMAT_UPDATE:
		ret = komeda_kms_rpmsg_format_update(rpmsg, payload);
		break;
	case RPMSG_BUF_UPDATE:
		ret = komeda_kms_rpmsg_buffer_update(rpmsg, payload);
		break;
	case RPMSG_PREPARE_DEREGISTER:
		ret = komeda_kms_rpmsg_prepare_deregister(rpmsg);
		break;
	default:
		KOMEDA_RPMSG_DEBUG("payload unknown type:%d\n", payload->type);
		ret = -EINVAL;
		break;
	}

	KOMEDA_RPMSG_DEBUG("handle payload type[%s] id[%u] ret:%d",
		type_to_str(payload->type), payload->id, ret);

	return ret;
}

int komeda_kms_rpmsg_init(struct komeda_kms_rpmsg *rpmsg)
{
	struct rpmsg_device *se1000_rpdev;
	struct rpmsg_channel_info chinfo;

	if (!rpmsg && !rpmsg->np) {
		KOMEDA_RPMSG_ERR("invalid rpmsg %p, rpmsg->np:%p", rpmsg, rpmsg->np);
		return -EINVAL;
	}

	if (rpmsg->vdomain == 0) {
		se1000_rpdev = find_rpmsg_device_by_phandle(rpmsg->np);
		if (!se1000_rpdev) {
			KOMEDA_RPMSG_ERR("find_rpmsg_dev fail");
			return -EINVAL;
		}
		rpmsg->rp_dev = se1000_rpdev;
		memset(&chinfo, 0, sizeof(chinfo));
		chinfo.src = rpmsg->rp_ch.src;
		chinfo.dst = rpmsg->rp_ch.dst;
		rpmsg->rpmsg_ept = rpmsg_create_ept(rpmsg->rp_dev,
					komeda_kms_rpmsg_rx_cb, rpmsg, chinfo);
		if (!rpmsg->rpmsg_ept) {
			KOMEDA_RPMSG_ERR("rpmsg_create_ept failed");
			return -ENOTTY;
		}

		KOMEDA_RPMSG_DEBUG("rpmsg_create_ept success! src 0x%0x dst 0x%0x",
			chinfo.src, chinfo.dst);
	}

	init_completion(&rpmsg->register_done);

	return 0;
}

struct komeda_kms_rpmsg *
komeda_kms_parse_rpmsg_from_dt(struct device_node *np, void *ipc)
{
	struct komeda_kms_rpmsg *rpmsg = NULL;
	u32 rpmsg_ch[2] = {};
	u32 slot, vdomain, pipe_id;
	int ret;

	rpmsg = (struct komeda_kms_rpmsg *)kzalloc(sizeof(*rpmsg), GFP_KERNEL);
	if (!rpmsg) {
		KOMEDA_RPMSG_ERR("fail to alloc mem for komeda_kms_rpmsg");
		goto clean;
	}

	rpmsg->np = np;

	ret = of_property_read_u32(np, "slot", &slot);
	if (ret) {
		KOMEDA_RPMSG_ERR("get slot property fail");
		goto clean;
	}

	rpmsg->slot = slot;
	KOMEDA_RPMSG_DEBUG("slot:%u", slot);

	ret = of_property_read_u32(np, "pipe_id", &pipe_id);
	if (ret) {
		KOMEDA_RPMSG_ERR("get pipe_id property fail");
		goto clean;
	}

	rpmsg->pipe_id = pipe_id;
	KOMEDA_RPMSG_DEBUG("pipe_id:%u", pipe_id);

	/* vdomain: AP-->0 CP-->1 */
	ret = of_property_read_u32(np, "vdomain", &vdomain);
	if (ret) {
		KOMEDA_RPMSG_ERR("get domain property fail");
		goto clean;
	}

	rpmsg->vdomain = vdomain;
	KOMEDA_RPMSG_DEBUG("vdomain %u", vdomain);

	ret = of_property_read_u32_array(np, "rpmsg-channel", rpmsg_ch, 2);
	if (ret) {
		KOMEDA_RPMSG_ERR("get rpmsg-channel property fail");
		goto clean;
	}

	rpmsg->rx_buf_num = 0;
	rpmsg->rp_ch.src = rpmsg_ch[RPMSG_SRC];
	rpmsg->rp_ch.dst = rpmsg_ch[RPMSG_DST];
	rpmsg->ipc = ipc;

	KOMEDA_RPMSG_DEBUG("rpmsg src:0x%x dst:0x%x",
		rpmsg->rp_ch.src, rpmsg->rp_ch.dst);

	return rpmsg;
clean:
	if (rpmsg)
		kfree(rpmsg);

	return NULL;
}
