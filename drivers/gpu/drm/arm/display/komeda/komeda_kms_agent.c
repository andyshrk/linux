// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Siengine
 */
#include <drm/drm_device.h>
#include <linux/dma-buf.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <linux/dma-mapping.h>
#include <drm/drm_atomic_uapi.h>
#include <drm/drm_drv.h>
#include <drm/drm_encoder.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/sysrq.h>
#include <linux/dev_printk.h>
#include <linux/scatterlist.h>
#include <linux/dmaengine.h>
#include <linux/iommu.h>
#include <linux/sched.h>
#include "komeda_framebuffer.h"
#include "komeda_kms.h"
#include "komeda_dev.h"
#include "komeda_kms_agent.h"
#include "komeda_kms_ipc.h"
#include "komeda_kms_agent_debug.h"

struct komeda_kms_agent *agent = NULL;

#define USE_DMA_COPY 1

static LIST_HEAD(agent_head);

static uint32_t get_drm_format(const enum komeda_kms_agent_fmt fmt)
{
	switch (fmt) {
	case RGB888:
		return DRM_FORMAT_RGB888;
	case ARGB8888:
		return DRM_FORMAT_ARGB8888;
	case ABGR8888:
		return DRM_FORMAT_ABGR8888;
	case YUYV:
		return DRM_FORMAT_YUYV;
	case NV12:
		return DRM_FORMAT_NV12;
	default:
		return 0;
	}
}

static int
komeda_kms_agent_drm_gem_fb_init(struct drm_device *dev,
		struct drm_framebuffer *fb,
		const struct drm_mode_fb_cmd2 *mode_cmd,
		struct drm_gem_object **obj, unsigned int num_planes)
{
	int ret, i;

	drm_helper_mode_fill_fb_struct(dev, fb, mode_cmd);

	for (i = 0; i < num_planes; i++)
		fb->obj[i] = obj[i];

	ret = drm_framebuffer_init(dev, fb, NULL);
	if (ret)
		KOMEDA_AGENT_ERR("Failed to init framebuffer: %d", ret);

	return ret;
}

static struct drm_gem_cma_object *
__drm_gem_cma_create(struct drm_device *drm, size_t size)
{
	struct drm_gem_cma_object *cma_obj;
	struct drm_gem_object *gem_obj;
	int ret;

	if (drm->driver->gem_create_object)
		gem_obj = drm->driver->gem_create_object(drm, size);
	else
		gem_obj = kzalloc(sizeof(*cma_obj), GFP_KERNEL);
	if (!gem_obj)
		return ERR_PTR(-ENOMEM);
	cma_obj = container_of(gem_obj, struct drm_gem_cma_object, base);

	ret = drm_gem_object_init(drm, gem_obj, size);
	if (ret)
		goto error;

	ret = drm_gem_create_mmap_offset(gem_obj);
	if (ret) {
		drm_gem_object_release(gem_obj);
		goto error;
	}

	return cma_obj;

error:
	kfree(cma_obj);
	return ERR_PTR(ret);
}

static void *
dma_alloc_continue(struct device *dev, size_t size,
				 dma_addr_t *dma_addr, gfp_t gfp)
{
	unsigned long attrs = DMA_ATTR_WRITE_COMBINE |
				DMA_ATTR_FORCE_CONTIGUOUS;

	if (gfp & __GFP_NOWARN)
		attrs |= DMA_ATTR_NO_WARN;

	return dma_alloc_attrs(dev, size, dma_addr, gfp, attrs);
}

static struct drm_gem_cma_object *
drm_gem_cma_continue_create(struct drm_device *drm, size_t size)
{
	struct drm_gem_cma_object *cma_obj;
	int ret;

	size = round_up(size, PAGE_SIZE);

	cma_obj = __drm_gem_cma_create(drm, size);
	if (IS_ERR(cma_obj))
		return cma_obj;

	cma_obj->vaddr = dma_alloc_continue(drm->dev, size, &cma_obj->paddr,
				GFP_KERNEL | __GFP_NOWARN);
	if (!cma_obj->vaddr) {
		KOMEDA_AGENT_ERR("failed to allocate buffer with size %zu",
				size);
		ret = -ENOMEM;
		goto error;
	}

	return cma_obj;

error:
	drm_gem_object_put(&cma_obj->base);
	return ERR_PTR(ret);
}

static struct komeda_kms_agent_fb *
komeda_kms_agent_alloc_fb(struct drm_device *dev,
		struct komeda_kms_buf_info *info)
{
	struct drm_gem_cma_object *cma_obj = NULL;
	struct drm_gem_object *gem_obj[4];
	struct drm_mode_fb_cmd2 mode_cmd;
	struct komeda_kms_agent_fb *agent_fb;
	struct komeda_dev *mdev;
	struct komeda_fb *kfb;
	int ret;

	if (!dev) {
		KOMEDA_AGENT_ERR("invalid drm dev");
		return NULL;
	}

	if (!info) {
		KOMEDA_AGENT_ERR("invalid info");
		return NULL;
	}

	if (info->fmt.fmt != NV12) {
		KOMEDA_AGENT_ERR("invalid format:%d, only support NV12",
					info->fmt.fmt);
		return NULL;
	}

	agent_fb = kzalloc(sizeof(*agent_fb), GFP_KERNEL);
	if (!agent_fb) {
		KOMEDA_AGENT_ERR("alloc mem fail for agent_fb");
		return ERR_PTR(-ENOMEM);
	}

	kfb = kzalloc(sizeof(*kfb), GFP_KERNEL);
	if (!kfb) {
		KOMEDA_AGENT_ERR("alloc mem fail for kfb");
		goto free_agent_fb;
	}

	memset(&mode_cmd, 0, sizeof(mode_cmd));

	mode_cmd.width = info->fmt.src_buf_w;
	mode_cmd.height = info->fmt.src_buf_h;
	mode_cmd.pixel_format = get_drm_format(info->fmt.fmt);
	mode_cmd.modifier[0] = 0ULL;

	/* NV12 is 2 plane */
	if (mode_cmd.pixel_format == DRM_FORMAT_NV12) {
		mode_cmd.pitches[0] = mode_cmd.width;
		mode_cmd.offsets[0] = 0;
		mode_cmd.pitches[1] = mode_cmd.pitches[0];
		mode_cmd.offsets[1] = mode_cmd.width * mode_cmd.height;
	}

	mdev = dev->dev_private;
	/* Format caps can only be generated when call create buffer in
	 * userspace. So if you want to commit in kernel, you must get format
	 * caps by yourself.
	 */
	kfb->format_caps = komeda_get_format_caps(&mdev->fmt_tbl,
						  mode_cmd.pixel_format,
						  mode_cmd.modifier[0]);
	if (!kfb->format_caps) {
		KOMEDA_AGENT_ERR("FMT %X is not supported", mode_cmd.pixel_format);
		goto free_kfb;
	}

	/* create gem */
	if (mode_cmd.pixel_format == DRM_FORMAT_NV12) {
		struct iommu_domain *domain;

		agent_fb->size = mode_cmd.width * mode_cmd.height * 3 / 2;
		cma_obj = drm_gem_cma_continue_create(dev, agent_fb->size);
		if (IS_ERR(cma_obj)) {
			KOMEDA_AGENT_ERR("gem_cma_create fail");
			goto free_kfb;
		}
		gem_obj[0] = &cma_obj->base;
		agent_fb->cma_obj = cma_obj;

		domain = iommu_get_domain_for_dev(dev->dev);
		/* Translation is installed only when IOMMU is present */
		agent_fb->paddr = domain ? iommu_iova_to_phys(domain, cma_obj->paddr) : cma_obj->paddr;

		gem_obj[1] =  gem_obj[0];
		KOMEDA_AGENT_DEBUG("phys addr:%llx dma addr:%llx size:%llx, va:%llx",
			agent_fb->paddr,
			agent_fb->cma_obj->paddr,
			agent_fb->size, cma_obj->vaddr);

		ret = komeda_kms_agent_drm_gem_fb_init(dev, &kfb->base,
				&mode_cmd, gem_obj, 2);
		if (ret) {
			KOMEDA_AGENT_ERR("gem_fb_init fail");
			goto free_cma_obj;
		}
	}

	agent_fb->fb = &kfb->base;
	agent_fb->fb->modifier = 0;

	kfb->is_va = mdev->iommu ? true : false;

	return agent_fb;

free_cma_obj:
	if (gem_obj[0])
		drm_gem_cma_free_object(gem_obj[0]);
free_kfb:
	if (kfb)
		kfree(kfb);
free_agent_fb:
	if (agent_fb)
		kfree(agent_fb);

	return NULL;
}

static int
komeda_kms_agent_fb_alloc_thread(void *args)
{
	struct agent_pipeline *ppl = (struct agent_pipeline *)args;
	struct komeda_kms_agent_fb *agent_fb;

	agent_fb = komeda_kms_agent_alloc_fb(ppl->drm_dev, &ppl->info);
	if (!agent_fb)
		return -ENOMEM;

	mutex_lock(&ppl->mutex);
	ppl->agent_fb[ppl->buf_id] = agent_fb;
	complete(&ppl->alloc_done[ppl->buf_id]);
	ppl->buf_id++;
	mutex_unlock(&ppl->mutex);

	return 0;
}

static int
komeda_kms_agent_prepare(struct agent_pipeline *ppl)
{
	int i;

	if (!ppl) {
		KOMEDA_AGENT_ERR("pipeline %d is null", ppl->pipe_id);
		return -EINVAL;
	}

	for (i = 0; i < BUF_NUM; ++i) {
		struct task_struct *fb_alloc_thread = NULL;

		KOMEDA_AGENT_DEBUG("alloc fb %d for pipe:%d", i, ppl->pipe_id);

		init_completion(&ppl->alloc_done[i]);
		fb_alloc_thread = kthread_create(komeda_kms_agent_fb_alloc_thread,
						ppl, "agent-fb-alloc-thread");
		if (!IS_ERR(fb_alloc_thread)) {
			wake_up_process(fb_alloc_thread);
		} else {
			KOMEDA_AGENT_ERR("agent-fb-alloc-thread create failed");
			return PTR_ERR(fb_alloc_thread);
		}
	}

	ppl->allocated = true;

	return 0;
}

static int
komeda_kms_agent_free_fb(struct komeda_kms_agent_fb *agent_fb)
{
	struct komeda_fb *kfb;
	struct drm_gem_cma_object *cma_obj;

	if (!agent_fb) {
		KOMEDA_AGENT_ERR("invalid fb %px", agent_fb);
		return -EINVAL;
	}

	cma_obj = agent_fb->cma_obj;
	if (cma_obj)
		drm_gem_cma_free_object(&cma_obj->base);

	if (agent_fb->fb) {
		kfb = to_kfb(agent_fb->fb);
		kfree(kfb);
		kfb = NULL;
	}

	return 0;
}

/* free pipeline buffer */
static int
komeda_kms_agent_pipe_unprepare(struct agent_pipeline *ppl)
{
	int i;
	int ret;
	struct komeda_kms_agent_fb *agent_fb;

	if (!ppl) {
		KOMEDA_AGENT_ERR("pipeline %d is null", ppl->pipe_id);
		return -EINVAL;
	}

	for (i = 0; i < BUF_NUM; i++) {
		agent_fb = ppl->agent_fb[i];
		ret = komeda_kms_agent_free_fb(agent_fb);
		if (ret)
			return ret;

		kfree(agent_fb);
		agent_fb = NULL;
	}

	return 0;
}

int
komeda_kms_agent_attach_drm_dev(struct drm_device *dev)
{
	struct list_head * pos;

	list_for_each(pos, &agent_head) {
		struct komeda_kms_agent *agent;

		agent = container_of(pos, struct komeda_kms_agent, list);
		if (agent->dpu_node->phandle == dev->dev->of_node->phandle) {
			KOMEDA_AGENT_DEBUG("find agent drm node:%s agent node:%s",
				dev->dev->of_node->name, agent->dpu_node->name);

			agent->drm_dev = dev;
			complete(&agent->bind_done);

			return 0;
		}
	}

	return -ENODEV;
}

static int
komeda_kms_agent_update_output_state(struct drm_atomic_state *state,
		struct drm_crtc *target_crtc)
{
	struct drm_device *dev = target_crtc->dev;
	struct drm_crtc_state *new_crtc_state;
	struct drm_connector *connector, *target_conn = NULL;
	struct drm_crtc *crtc;
	struct drm_connector_state *new_conn_state, *target_conn_state = NULL;
	int ret, i;

	ret = drm_modeset_lock(&dev->mode_config.connection_mutex,
				state->acquire_ctx);
	if (ret) {
		KOMEDA_AGENT_ERR("lock fail ret:%d", ret);
		return ret;
	}

	/* First disable all connectors on the target crtc. */
	ret = drm_atomic_add_affected_connectors(state, target_crtc);
	if (ret) {
		KOMEDA_AGENT_ERR("add_affected_connectors fail, ret:%d", ret);
		return ret;
	}

	for_each_new_connector_in_state(state, connector, new_conn_state, i) {
		if (new_conn_state->crtc == target_crtc) {
			target_conn = connector;
			target_conn_state = new_conn_state;
			ret = drm_atomic_set_crtc_for_connector(new_conn_state,
								NULL);
			if (ret) {
				KOMEDA_AGENT_ERR("set crtc fail, ret:%d", ret);
				return ret;
			}

			/* Make sure legacy setCrtc always re-trains */
			new_conn_state->link_status = DRM_LINK_STATUS_GOOD;
		}
	}

	if (!target_conn || !target_conn_state) {
		KOMEDA_AGENT_ERR("can not find target conn for crtc %s",
					crtc->name);
		return -EINVAL;
	}

	/* Then set all connectors from set->connectors on the target crtc */
	ret = drm_atomic_set_crtc_for_connector(target_conn_state,
						target_crtc);
	if (ret) {
		KOMEDA_AGENT_ERR("set_crtc_for_connector fail, ret:%d", ret);
		return ret;
	}

	for_each_new_crtc_in_state(state, crtc, new_crtc_state, i) {
		/*
		 * Don't update ->enable for the CRTC in the set_config request,
		 * since a mismatch would indicate a bug in the upper layers.
		 * The actual modeset code later on will catch any
		 * inconsistencies here.
		 */
		if (crtc == target_crtc)
			continue;

		if (!new_crtc_state->connector_mask) {
			ret = drm_atomic_set_mode_prop_for_crtc(new_crtc_state,
								NULL);
			if (ret < 0) {
				KOMEDA_AGENT_ERR("set_mode fail, ret:%d", ret);
				return ret;
			}

			new_crtc_state->active = false;
		}
	}

	return 0;
}

static int
komeda_kms_agent_set_config(struct drm_plane *plane,
		struct komeda_kms_buf_fmt *fmt, struct drm_crtc *crtc,
		struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state;
	struct drm_plane_state *plane_state;
	int hdisplay = 0, vdisplay = 0;
	int ret;
	long time_left;
	int id;
	struct komeda_dev *mdev;
	struct komeda_fbdev *fbdev;

	if (!plane || !crtc || !state || !fmt) {
		KOMEDA_AGENT_ERR("invalid param plane:%p crtc:%p "
				"state:%p fmt:%p", plane, crtc, state, fmt);
		return -EINVAL;
	}

	id = drm_crtc_index(crtc);
	mdev = crtc->dev->dev_private;

	fbdev = mdev->fbdev[id];

	if (fbdev) {
		time_left = wait_for_completion_interruptible_timeout(&fbdev->hpd_done, HZ);
		if (!time_left) {
			KOMEDA_AGENT_ERR("timeout waiting for CRTC:%d:%s hpd",
				crtc->base.id, crtc->name);
			return -ETIMEDOUT;
		}
	}

	plane_state = drm_atomic_get_plane_state(state, plane);
	if (IS_ERR(plane_state)) {
		KOMEDA_AGENT_ERR("get_plane_state fail");
		return PTR_ERR(plane_state);
	}

	crtc_state = drm_atomic_get_crtc_state(state, crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	crtc_state->active = true;

	ret = drm_atomic_set_crtc_for_plane(plane_state, crtc);
	if (ret) {
		KOMEDA_AGENT_ERR("set crtc for plane fail");
		return ret;
	}

	if (!plane_state->fb) {
		KOMEDA_AGENT_ERR("plane:%s has no fb", plane->name);
		return -EINVAL;
	}

	hdisplay = crtc->mode.crtc_hdisplay;
	vdisplay = crtc->mode.crtc_vdisplay;
	if (!hdisplay || !vdisplay) {
		KOMEDA_AGENT_ERR("CRTC:%d:%s invalid hdisplay:%d, vdisplay:%d",
			crtc->base.id, crtc->name, hdisplay, vdisplay);
		return -EINVAL;
	}

	plane_state->crtc_x = fmt->dst_x;
	plane_state->crtc_y = fmt->dst_y;
	plane_state->crtc_w = fmt->dst_w;
	plane_state->crtc_h = fmt->dst_h;
	plane_state->src_x = fmt->src_x;
	plane_state->src_y = fmt->src_y;
	plane_state->src_w = (fmt->src_crop_w << 16);
	plane_state->src_h = (fmt->src_crop_h << 16);

	KOMEDA_AGENT_DEBUG("src [x,y,w,h] [%d,%d,%d,%d], "
			"crtc [x,y,w,h] [%d,%d,%d,%d]",
			plane_state->src_x, plane_state->src_y,
			plane_state->src_w >> 16,
			plane_state->src_h >> 16,
			plane_state->crtc_x, plane_state->crtc_y,
			plane_state->crtc_w, plane_state->crtc_h);

	return komeda_kms_agent_update_output_state(state, crtc);
}

static int
komeda_kms_agent_commit(struct agent_pipeline *ppl, struct drm_framebuffer *fb)
{
	struct drm_device *dev = ppl->drm_dev;
	struct drm_plane *plane, *target_plane = NULL;
	struct drm_crtc *crtc;
	struct drm_atomic_state *state;
	uint32_t plane_index;
	uint32_t target_plane_index;
	struct drm_modeset_acquire_ctx ctx;
	struct komeda_kms_agent_dma *agent_dma;
	struct drm_plane_state *plane_state;
	int ret;

	drm_modeset_acquire_init(&ctx, 0);

	state = drm_atomic_state_alloc(dev);
	if (!state) {
		ret = -ENOMEM;
		goto out_ctx;
	}

	state->acquire_ctx = &ctx;

retry:
	drm_for_each_plane(plane, dev) {
		struct drm_plane_state *plane_state;

		plane_state = drm_atomic_get_plane_state(state, plane);
		if (IS_ERR(plane_state)) {
			ret = PTR_ERR(plane_state);
			goto out_state;
		}

		plane_state->rotation = DRM_MODE_ROTATE_0;

		plane_index = drm_plane_index(plane);
		target_plane_index = ppl->layer_id + (ppl->pipe_id << 2);

		/* find target pipeline */
		if ((plane_index >> 2) != ppl->pipe_id)
			continue;

		/* find target plane */
		if (plane_index == target_plane_index) {
			KOMEDA_AGENT_DEBUG("find target plane %s index:%d",
				plane->name, plane_index);
			target_plane = plane;
			if (fb) {
				drm_atomic_set_fb_for_plane(plane_state, fb);

				if (ppl->other_plane_disabled)
					break;
				else
					continue;
			}
		}

		/* disable other planes at first commit */
		ret = drm_atomic_set_crtc_for_plane(plane_state, NULL);
		if (ret != 0)
			goto out_state;

		drm_atomic_set_fb_for_plane(plane_state, NULL);
	}

	if (!target_plane) {
		KOMEDA_AGENT_ERR("can not find target plane");
		ret = -ENOMEM;
		goto out_ctx;
	}

	if (!ppl->configed) {
		drm_for_each_crtc(crtc, dev) {
			struct drm_crtc_state *crtc_state;

			crtc_state = drm_atomic_get_crtc_state(state, crtc);
			if (drm_crtc_index(crtc) != ppl->pipe_id)
				continue;

			KOMEDA_AGENT_DEBUG("find target crtc %s index:%d",
						crtc->name, drm_crtc_index(crtc));
			if (fb) {
				ret = komeda_kms_agent_set_config(target_plane,
						&ppl->info.fmt,	crtc, state);
				if (!ret) {
					ppl->configed = true;
				} else {
					KOMEDA_AGENT_ERR("set_config fail, ret:%d", ret);
					goto out_state;
				}
			} else {
				crtc_state->active = false;

				plane_state = drm_atomic_get_plane_state(state,
					target_plane);
				drm_atomic_set_fb_for_plane(plane_state, NULL);
				ret = drm_atomic_set_crtc_for_plane(plane_state, NULL);
			}

			if (ret != 0)
				goto out_state;
		}
	}

	plane_state = drm_atomic_get_plane_state(state, target_plane);
	plane_state->src_w = min(plane_state->src_w, fb->width << 16);
	plane_state->src_h = min(plane_state->src_h, fb->height << 16);

#if USE_DMA_COPY
	agent_dma = &ppl->agent->agent_dma[ppl->pipe_id];
	ret = wait_for_completion_timeout(&agent_dma->xfer_done, HZ);
	if (ret == 0) {
		KOMEDA_AGENT_ERR("chan %s timed out", agent_dma->chn_name);
		goto out_state;
	}
#endif
	if (!plane_state->crtc) {
		KOMEDA_AGENT_INFO("crtc user has been switched, skip current commit");
		ret = 0;
		goto out_state;
	}

	ret = drm_atomic_nonblocking_commit(state);
	if (ret)
		KOMEDA_AGENT_INFO("pipe:%d [PLANE:%d:%s] src [x,y,w,h] [%d,%d,%d,%d], "
				"crtc [x,y,w,h] [%d,%d,%d,%d], ret:%d",
				ppl->pipe_id, target_plane->base.id, target_plane->name,
				plane_state->src_x, plane_state->src_y,
				plane_state->src_w >> 16,
				plane_state->src_h >> 16,
				plane_state->crtc_x, plane_state->crtc_y,
				plane_state->crtc_w, plane_state->crtc_h, ret);

	if (!ppl->other_plane_disabled && !ret)
		ppl->other_plane_disabled = true;

out_state:
	if (ret == -EDEADLK)
		goto backoff;

	drm_atomic_state_put(state);
out_ctx:
	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);

	return ret;

backoff:
	drm_atomic_state_clear(state);
	drm_modeset_backoff(&ctx);

	goto retry;
}

static int
komeda_kms_get_ppls_from_dt(struct device_node *np)
{
	struct device_node *child;
	int ppls = 0;

	if (!np) {
		KOMEDA_AGENT_ERR(" invalid np");
		return -EINVAL;
	}

	for_each_available_child_of_node(np, child) {
		if (of_node_cmp(child->name, "pipeline") != 0)
			continue;
		ppls++;
	}

	return ppls;
}

static int
komeda_kms_agent_parse_dt(struct komeda_kms_agent *agent)
{
	struct device *dev;
	int ppls;
	int ret = 0, i = 0;
	uint32_t layer_id = 0;
	uint32_t pipe_id = 0;
	struct device_node *child, *sun, *np;
	struct komeda_kms_ipc *ipc;
	struct device_node *reserved_node = NULL;
	struct device_node *dpu_node = NULL;
	struct resource	reserved_res;

	if (!agent) {
		KOMEDA_AGENT_ERR("invalid agent\n");
		return -EINVAL;
	}

	dev = agent->dev;
	if (!dev) {
		KOMEDA_AGENT_ERR("invalid dev\n");
		return -EINVAL;
	}

	np = dev->of_node;

	ppls = komeda_kms_get_ppls_from_dt(np);
	if (ppls <= 0) {
		KOMEDA_AGENT_ERR("get pipeline %d in dts", ppls);
		return -EINVAL;
	}

	KOMEDA_AGENT_DEBUG("get pipeline %d in dts", ppls);
	agent->ppls_num = ppls;
	agent->ppl = kzalloc(sizeof(struct agent_pipeline) * ppls, GFP_KERNEL);
	if (!agent->ppl) {
		KOMEDA_AGENT_ERR("fail to alloc mem for ppl");
		return -ENOMEM;
	}

	dpu_node = of_parse_phandle(np, "dpu-node", 0);
	if (!dpu_node) {
		KOMEDA_AGENT_ERR("Cannot find node:dpu-node");
		ret = -EINVAL;
		return ret;
	}

	agent->dpu_node = dpu_node;

	reserved_node = of_parse_phandle(np, "memory-region", 0);
	if (!reserved_node) {
		KOMEDA_AGENT_ERR("Cannot find node:memory-region");
		ret = -EINVAL;
		return ret;
	}

	if (of_address_to_resource(reserved_node, 0, &reserved_res)) {
		KOMEDA_AGENT_ERR("Failed to get memory-region address");
		of_node_put(reserved_node);
		ret = -EINVAL;
		return ret;
	}
	of_node_put(reserved_node);

	agent->resv_pa = reserved_res.start;
	agent->resv_size = resource_size(&reserved_res);

	agent->vaddr = devm_ioremap_wc(dev, agent->resv_pa, agent->resv_size);
	if (!agent->vaddr) {
		KOMEDA_AGENT_ERR("ioremap fail");
		return -ENOMEM;
	}

	KOMEDA_AGENT_DEBUG("reserve addr:0x%llx size:0x%llx vaddr:%px",
				agent->resv_pa, agent->resv_size,
				(unsigned char *)agent->vaddr);

	for_each_available_child_of_node(np, child) {
		if (of_node_cmp(child->name, "pipeline") != 0)
			continue;

		ret = of_property_read_u32(child, "layer_id", &layer_id);
		if (ret) {
			KOMEDA_AGENT_ERR("get layer_id failed in dts");
			ret = -EINVAL;
			goto parse_dt_clean;
		}
		KOMEDA_AGENT_DEBUG("get layer_id :%u", layer_id);

		agent->ppl[i].layer_id = layer_id;
		agent->ppl[i].pipe_id = pipe_id;
		pipe_id++;
		agent->ppl[i].other_plane_disabled = false;
		agent->ppl[i].configed = false;
		mutex_init(&agent->ppl[i].mutex);
		agent->ppl[i].buf_id = 0;

		for_each_available_child_of_node(child, sun) {
			if (of_node_cmp(sun->name, "ipc_pipe") != 0)
				continue;

			ipc = komeda_kms_parse_ipc_dt(agent, sun);
			if (!ipc) {
				KOMEDA_AGENT_ERR("parse ipc dt error!");
				goto parse_dt_clean;
			}
			agent->ppl[i].ipc = ipc;
			KOMEDA_AGENT_DEBUG("pipe%d ipc:%px", i, agent->ppl[i].ipc);
			/* only support one ipc_pipe node in one pipeline */
			break;
		}

		i++;
	}

	return 0;
parse_dt_clean:
	if (agent->ppl)
		kfree(agent->ppl);

	return ret;
}

int
komeda_kms_agent_pipe_init(struct komeda_kms_agent *agent)
{
	int i, ret;

	if (!agent) {
		KOMEDA_AGENT_ERR("invalid agent");
		return -EINVAL;
	}

	for (i = 0; i < agent->ppls_num; i++) {
		ret = komeda_kms_agent_ipc_init(agent->ppl[i].ipc);
		if (ret) {
			KOMEDA_AGENT_ERR("komeda_kms_agent_ipc_init fail");
			return ret;
		}
		KOMEDA_AGENT_DEBUG("pipeline %d init done", i);
	}
	return 0;
}

static struct komeda_kms_agent_fb *
komeda_kms_get_free_fb(struct agent_pipeline *ppl)
{
	int i;
	int ret;
	struct komeda_kms_buf_info *info;
	phys_addr_t paddr;

	if (!ppl) {
		KOMEDA_AGENT_ERR("pipeline%d is null", ppl->pipe_id);
		return NULL;
	}

	info = &ppl->info;

	for (i = 0; i < BUF_NUM; ++i) {
		if (!ppl->agent_fb[i]) {
			ret = wait_for_completion_timeout(&ppl->alloc_done[i], HZ);
			if (ret == 0) {
				KOMEDA_AGENT_ERR("pipe%d fb %d alloc timed out",
						ppl->pipe_id, i);
				return NULL;
			}
		}
	}

	for (i = 0; i < BUF_NUM; ++i) {
		if (!ppl->agent_fb[i]->used) {
			phys_addr_t offset;

			memcpy(&ppl->agent_fb[i]->info, info, sizeof(*info));
			paddr = info->addr.addr.paddr;
			offset = paddr - ppl->agent->resv_pa;

			ppl->agent_fb[i]->vaddr = ppl->agent->vaddr + offset;

			KOMEDA_AGENT_DEBUG("src addr 0x%llx "
					"vaddr:0x%px index:%d",
					paddr, ppl->agent_fb[i]->vaddr, i);
			ppl->agent_fb[i]->used = true;
			return ppl->agent_fb[i];
		}
	}

	return NULL;
}

static int
komeda_kms_agent_wait_drm_attach(struct komeda_kms_agent *agent)
{
	if (!agent) {
		KOMEDA_AGENT_ERR("invalid agent");
		return -EINVAL;
	}

	if (!agent->drm_dev) {
		unsigned long time_left, timeout;

		KOMEDA_AGENT_DEBUG("waiting for bind to drm device");

		timeout = msecs_to_jiffies(1000);
		time_left = wait_for_completion_timeout(&agent->bind_done, timeout);
		if (!time_left) {
			KOMEDA_AGENT_ERR("timeout waiting for bind drm device");
			return -ETIMEDOUT;
		}

		if (!agent->drm_dev) {
			KOMEDA_AGENT_ERR("not bind to drm device");
			return -EINVAL;
		}

		KOMEDA_AGENT_INFO("bind to drm device");
	}

	return 0;
}

static void
komeda_kms_agent_dma_client_xfer_done(void *args)
{
	struct komeda_kms_agent_dma *agent_dma = (struct komeda_kms_agent_dma *)args;

	complete(&agent_dma->xfer_done);

	KOMEDA_AGENT_DEBUG("chan %d copy done", agent_dma->id);
}

static int komeda_kms_agent_dma_copy(struct komeda_kms_agent *agent,
		phys_addr_t dst, size_t size, phys_addr_t src, int pipe)
{
	int ret;
	struct dma_async_tx_descriptor *desc = NULL;
	dma_cookie_t cookie;

	if (pipe >= MAX_DMA_CHN) {
		KOMEDA_AGENT_ERR("invalid dma channel %d", pipe);
		return -EINVAL;
	}

	KOMEDA_AGENT_DEBUG("dma copy src:%llx dst:%llx, size:%zx",
		src, dst, size);
	desc = dmaengine_prep_dma_memcpy(agent->agent_dma[pipe].chan,
			dst, src, size,	DMA_PREP_INTERRUPT);
	if (!desc) {
		KOMEDA_AGENT_ERR("prep memcpy failed");
		ret = -ENOMEM;
		return ret;
	}

	desc->callback = komeda_kms_agent_dma_client_xfer_done;
	desc->callback_param = (void *)(&agent->agent_dma[pipe]);
	cookie = dmaengine_submit(desc);
	ret = dma_submit_error(cookie);
	if (ret) {
		KOMEDA_AGENT_ERR("channel submit failed");
		return ret;
	}

	dma_async_issue_pending(agent->agent_dma[pipe].chan);

	return 0;
}

static void
komeda_kms_agent_swap_prev_buf_state(struct agent_pipeline *ppl,
	struct komeda_kms_agent_fb *current_fb)
{
	int i;

	for (i = 0; i < BUF_NUM; ++i)
		if (current_fb != ppl->agent_fb[i])
			ppl->agent_fb[i]->used = false;
}

static int
komeda_kms_agent_buffer_update(struct komeda_kms_agent *agent,
	unsigned int id, struct komeda_kms_buf_addr *addr)
{
	struct drm_format_name_buf drm_fmt_name;
	int ret;
	struct komeda_kms_agent_fb *agent_fb;
	struct agent_pipeline *ppl;

	if (!agent) {
		KOMEDA_AGENT_ERR("invalid agent");
		return -EINVAL;
	}

	if (id >= agent->ppls_num) {
		KOMEDA_AGENT_ERR("invalid id %u. Exceed max num %d",
			id, agent->ppls_num);
		return -EINVAL;
	}

	ppl = &agent->ppl[id];
	if (!ppl) {
		KOMEDA_AGENT_ERR("pipeline%d is null", id);
		return -EINVAL;
	}

	if (!addr) {
		KOMEDA_AGENT_ERR("invalid buf addr");
		return -EINVAL;
	}

	if ((addr->addr.paddr < agent->resv_pa) ||
		(addr->addr.paddr > agent->resv_pa + agent->resv_size)) {
		KOMEDA_AGENT_ERR("addr:%llx is out of range [%llx, %llx]",
			addr->addr.paddr,
			agent->resv_pa, agent->resv_pa + agent->resv_size);
		return -EINVAL;
	}
	memcpy(&ppl->info.addr, addr, sizeof(*addr));

	agent_fb = komeda_kms_get_free_fb(ppl);
	if (!agent_fb) {
		KOMEDA_AGENT_ERR("can not find cached fb use index 0 default");
		agent_fb = ppl->agent_fb[0];
	}
#if USE_DMA_COPY
	ret = komeda_kms_agent_dma_copy(agent, agent_fb->paddr, agent_fb->size,
		addr->addr.paddr, id);
	if (ret)
		return ret;
#else
	if (agent_fb->info.drm_fmt == DRM_FORMAT_NV12) {
		int size = 0;

		size = agent_fb->size;
		memcpy(agent_fb->cma_obj->vaddr, agent_fb->vaddr, size);
		KOMEDA_AGENT_DEBUG("src addr:0x%llx dst:%llx size:0x%llx",
					agent_fb->vaddr,
					agent_fb->cma_obj->vaddr, size);
	}
#endif

	ret = komeda_kms_agent_commit(ppl, agent_fb->fb);
	komeda_kms_agent_swap_prev_buf_state(ppl, agent_fb);

	KOMEDA_AGENT_DEBUG("rcv pipe%d buf info crtc [x/y/w/h] [%d/%d/%d/%d] "
		"src [x/y/w/h] [%d/%d/%d/%d] fb [%d/%d] fmt:%s addr:0x%llx  "
		"commit ret:%d",
		id,
		ppl->info.fmt.dst_x, ppl->info.fmt.dst_y,
		ppl->info.fmt.dst_w, ppl->info.fmt.dst_h,
		ppl->info.fmt.src_x, ppl->info.fmt.src_y,
		ppl->info.fmt.src_crop_w, ppl->info.fmt.src_crop_h,
		ppl->info.fmt.src_buf_w, ppl->info.fmt.src_buf_h,
		drm_get_format_name(ppl->info.drm_fmt, &drm_fmt_name),
		ppl->info.addr.addr.paddr, ret);
	return ret;
}

static int
komeda_kms_agent_format_update(struct komeda_kms_agent *agent,
		unsigned int id, struct komeda_kms_buf_fmt *fmt)
{
	struct drm_format_name_buf drm_fmt_name;
	int ret;
	int i;
	struct agent_pipeline *ppl;

	if (!agent) {
		KOMEDA_AGENT_ERR("invalid agent");
		return -EINVAL;
	}

	if (!fmt) {
		KOMEDA_AGENT_ERR("invalid buf fmt");
		return -EINVAL;
	}

	if (id >= agent->ppls_num) {
		KOMEDA_AGENT_ERR("invalid id %u. Exceed max num %d",
				id, agent->ppls_num);
		return -EINVAL;
	}

	ppl = &agent->ppl[id];
	if (!ppl) {
		KOMEDA_AGENT_ERR("pipeline%d is null", id);
		return -EINVAL;
	}

	memcpy(&ppl->info.fmt, fmt, sizeof(*fmt));

	/* wait for bind drm */
	if (!agent->drm_dev) {
		ret = komeda_kms_agent_wait_drm_attach(agent);
		if (ret)
			return ret;
	}

	for (i = 0; i < agent->ppls_num; ++i)
		agent->ppl[i].drm_dev = agent->drm_dev;


	if (!ppl->allocated) {
		ret = komeda_kms_agent_prepare(ppl);
		if (ret) {
			KOMEDA_AGENT_ERR("agent prepare fail");
			return ret;
		}
	}

	ppl->info.drm_fmt = get_drm_format(fmt->fmt);
	ppl->agent = agent;

	KOMEDA_AGENT_DEBUG("rcv pipe%d buf info crtc [x/y/w/h] [%d/%d/%d/%d] "
		"src [x/y/w/h] [%d/%d/%d/%d] fb [%d/%d] fmt:%s",
		id,
		ppl->info.fmt.dst_x, ppl->info.fmt.dst_y,
		ppl->info.fmt.dst_w, ppl->info.fmt.dst_h,
		ppl->info.fmt.src_x, ppl->info.fmt.src_y,
		ppl->info.fmt.src_crop_w, ppl->info.fmt.src_crop_h,
		ppl->info.fmt.src_buf_w, ppl->info.fmt.src_buf_h,
		drm_get_format_name(ppl->info.drm_fmt, &drm_fmt_name));

	return 0;
}

static int
komeda_kms_agent_prepare_deregister(struct komeda_kms_agent *agent,
					unsigned int id)
{
	int ret;
	struct agent_pipeline *ppl;

	if (!agent) {
		KOMEDA_AGENT_ERR("invalid agent");
		return -EINVAL;
	}

	if (id >= agent->ppls_num) {
		KOMEDA_AGENT_ERR("invalid id %u. Exceed max num %d",
			id, agent->ppls_num);
		return -EINVAL;
	}

	ppl = &agent->ppl[id];
	if (!ppl) {
		KOMEDA_AGENT_ERR("pipeline%d is null", id);
		return -EINVAL;
	}

	ret = komeda_kms_agent_commit(ppl, NULL);
	if (ret) {
		KOMEDA_AGENT_ERR("pipe%d commit ret:%d", id, ret);
		return ret;
	}

	KOMEDA_AGENT_DEBUG("pipe%d commit success ret:%d", id, ret);

	ret = komeda_kms_agent_pipe_unprepare(ppl);

	return ret;
}

static struct komeda_kms_agent_handler agent_handler = {
	.format_update = komeda_kms_agent_format_update,
	.buffer_update = komeda_kms_agent_buffer_update,
	.prepare_deregister = komeda_kms_agent_prepare_deregister,
};

static int
komeda_kms_agent_dma_init(struct komeda_kms_agent *agent)
{
	char *chn_name[2] = {"tx0", "tx1"};
	static struct dma_chan *chan = NULL;
	int i;
	int ret;
	struct dma_slave_config slave_config;
	struct komeda_kms_agent_dma *agent_dma;

	if (!agent) {
		KOMEDA_AGENT_ERR("invalid agent");
		return -EINVAL;
	}

	memset(&slave_config, 0, sizeof(slave_config));
	slave_config.direction = DMA_MEM_TO_MEM;

	for (i = 0; i < MAX_DMA_CHN; i++) {
		chan = dma_request_slave_channel(agent->dev,
					chn_name[i]);
		if (!chan) {
			KOMEDA_AGENT_ERR("channel = %s request failed",
					chn_name[i]);
			return -ENODEV;
		}

	    	ret = dmaengine_slave_config(chan, &slave_config);
		if (ret) {
			KOMEDA_AGENT_ERR("channel %s config failed, ret:%d",
					chn_name[i], ret);
			return ret;
		}

		agent_dma = &agent->agent_dma[i];
		agent_dma->chan = chan;
		memset(agent_dma->chn_name, 0, sizeof(agent_dma->chn_name));
		strncpy(agent_dma->chn_name, chn_name[i], strlen(chn_name[i]));

		agent_dma->id = i;

		init_completion(&agent_dma->xfer_done);
	}

	return 0;
}

int komeda_kms_agent_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct komeda_kms_agent *agent;
	int ret;

	agent = kzalloc(sizeof(struct komeda_kms_agent), GFP_KERNEL);
	if (IS_ERR(agent)) {
		KOMEDA_AGENT_ERR("alloc mem failed!");
		return PTR_ERR(agent);
	}

	agent->dev = dev;

	ret = komeda_kms_agent_dma_init(agent);
	if (ret)
		goto clean;

	init_completion(&agent->bind_done);

	dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));

	ret = komeda_kms_agent_parse_dt(agent);
	if (ret) {
		KOMEDA_AGENT_ERR("parse dt failed!");
		goto clean;
	}

	agent->handler = &agent_handler;

	ret = komeda_kms_agent_pipe_init(agent);
	if (ret) {
		KOMEDA_AGENT_ERR("ipc init failed");
		goto clean;
	}

	dev_set_drvdata(dev, agent);

	INIT_LIST_HEAD(&agent->list);

	list_add(&agent->list, &agent_head);

	return 0;

clean:
	if (agent)
		kfree(agent);

	return ret;
}

static int komeda_kms_agent_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct  komeda_kms_agent *agent;
	int i;
	int ret;

	agent = dev_get_drvdata(dev);
	if (!agent) {
		KOMEDA_AGENT_ERR("agent is NULL!");
		return -EAGAIN;
	}

	if (agent->ppl) {
		for (i = 0; i < agent->ppls_num; i++) {
			ret = komeda_kms_agent_pipe_unprepare(&agent->ppl[i]);
			if (ret) {
				KOMEDA_AGENT_ERR("pipe%d unprepare failed", i);
				return ret;
			}
		}

		kfree(agent->ppl);
		agent->ppl = NULL;
	}

	dev_set_drvdata(dev, NULL);

	KOMEDA_AGENT_DEBUG("komeda_kms_agent driver remove success!");

	return 0;
}

static const struct of_device_id komeda_kms_agent_dt_ids[] = {
	{
	.compatible = "siengine,komeda-kms-agent",
	},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, komeda_kms_agent_dt_ids);

static struct platform_driver komeda_kms_agent_driver = {
	.probe	 = komeda_kms_agent_probe,
	.remove	 = komeda_kms_agent_remove,
	.driver	 = {
		.of_match_table = komeda_kms_agent_dt_ids,
		.name = "komeda-kms-agent",
	},
};

static int __init komeda_kms_agent_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&komeda_kms_agent_driver);

	return ret;
}

static void __exit komeda_kms_agent_exit(void)
{
	platform_driver_unregister(&komeda_kms_agent_driver);
}

module_init(komeda_kms_agent_init);
module_exit(komeda_kms_agent_exit);

MODULE_AUTHOR("Zhiqiang Zhang <zhiqiang.zhang@siengine.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SiEngine komeda kms agent driver");
