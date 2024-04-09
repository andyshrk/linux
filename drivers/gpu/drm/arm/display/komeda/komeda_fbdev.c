// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019 Siengine
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
#include <linux/sysrq.h>
#include <linux/dev_printk.h>
#include "komeda_framebuffer.h"
#include "komeda_dev.h"
#include "komeda_fbdev.h"

#ifdef CONFIG_DRM_FBDEV_EMULATION
#define DRM_CLIENT_MAX_CLONED_CONNECTORS    8
#define FBIO_ADD_FB               _IOW('F', 0x22, __u32)
#define FBIO_SET_FB               _IOW('F', 0x23, struct komeda_mode_fb_cmd)
#define FBIO_SET_CROP             _IOW('F', 0x24, struct komeda_fb_crop)
#define FBIO_RESTORE_FB           _IOW('F', 0x25, __u32)
#define MAX_YUV_LINE_SIZE         2048

static bool drm_fbdev_emulation = true;
static int komeda_pan_display_atomic(struct fb_var_screeninfo *var, struct fb_info *info);

static void
komeda_gem_object_put_locked(struct drm_gem_object *obj)
{
	if (!obj)
		return;

	kref_put(&obj->refcount, drm_gem_object_free);
}

static void komeda_fb_helper_fill_info(struct fb_info *info,struct drm_fb_helper *fb_helper)
{
	struct drm_fb_helper_surface_size sizes;

	sizes.fb_width = info->var.xres;
	sizes.fb_height = info->var.yres;
	drm_fb_helper_fill_info(info, fb_helper, &sizes);

}

static int drm_fb_cma_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	return dma_mmap_wc(info->device, vma, info->screen_base,
				     info->fix.smem_start, info->fix.smem_len);
}

static int komeda_fb_helper_check_var(struct fb_var_screeninfo *var,
			    struct fb_info *info)
{
	struct komeda_fbdev *kfbdev = info->par;
	struct drm_fb_helper *fb_helper = info->par;
	struct drm_framebuffer *fb = fb_helper->fb;
	struct drm_mode_set *set = fb_helper->client.modesets;
	int hdisplay, vdisplay;

	if (!fb || !kfbdev) {
		DRM_ERROR("Invalid value!\n");
		return -EINVAL;
	}

	drm_mode_get_hv_timing(set->mode, &hdisplay, &vdisplay);
	if (((kfbdev->crtc_x + kfbdev->crtc_w) > hdisplay)
	    || ((kfbdev->crtc_y + kfbdev->crtc_h) > vdisplay)) {
		DRM_ERROR("Invalid set of crtc crop: %d, %d, %d, %d\n",
			kfbdev->crtc_x, kfbdev->crtc_y, kfbdev->crtc_w, kfbdev->crtc_h);
		return -EINVAL;
	}

	if (((kfbdev->src_x + kfbdev->src_w) > fb->width)
		|| ((kfbdev->src_y + kfbdev->src_h) > fb->height)) {
		DRM_ERROR("Invalid set of src crop: %d, %d, %d, %d\n",
			kfbdev->src_x, kfbdev->src_y, kfbdev->src_w, kfbdev->src_h);
		return -EINVAL;
	}

	return 0;
}

static int komeda_fb_helper_set_par(struct fb_info *info)
{
	struct drm_fb_helper *fb_helper = info->par;
	struct fb_var_screeninfo *var = &info->var;
	struct drm_framebuffer *fb = fb_helper->fb;

	if (var->yoffset >= fb->height) {
		DRM_ERROR("yoffset overflow!\n");
		return -EINVAL;
	}

	switch (fb->format->format) {
		case DRM_FORMAT_RGB888:
			fb->offsets[0] = var->yoffset * var->xres_virtual * 3;
			break;

		case DRM_FORMAT_XRGB8888:
			fb->offsets[0] = var->yoffset * var->xres_virtual * 4;
			break;

		case DRM_FORMAT_YUV420:
			fb->offsets[0] = var->yoffset * var->xres_virtual * 3 / 2;
			fb->offsets[1] = fb->offsets[0] + var->xres_virtual * var->yres;
			fb->offsets[2] = fb->offsets[1] + var->xres_virtual * var->yres / 4 ;
			break;

		case DRM_FORMAT_NV12:
			fb->offsets[0] = var->yoffset * var->xres_virtual * 3 / 2;
			fb->offsets[1] = fb->offsets[0] + var->xres_virtual * var->yres;
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

static int update_output_state(struct drm_atomic_state *state,
					struct drm_mode_set *set)
{
	struct drm_device *dev = set->crtc->dev;
	struct drm_crtc *crtc;
	struct drm_crtc_state *new_crtc_state;
	struct drm_connector *connector;
	struct drm_connector_state *new_conn_state;
	int ret, i;

	ret = drm_modeset_lock(&dev->mode_config.connection_mutex,
				state->acquire_ctx);
	if (ret)
		return ret;

	/* First disable all connectors on the target crtc. */
	ret = drm_atomic_add_affected_connectors(state, set->crtc);
	if (ret)
		return ret;

	for_each_new_connector_in_state(state, connector, new_conn_state, i) {
		if (new_conn_state->crtc == set->crtc) {
			ret = drm_atomic_set_crtc_for_connector(new_conn_state,
								NULL);
			if (ret)
				return ret;

			/* Make sure legacy setCrtc always re-trains */
			new_conn_state->link_status = DRM_LINK_STATUS_GOOD;
		}
	}

	/* Then set all connectors from set->connectors on the target crtc */
	for (i = 0; i < set->num_connectors; i++) {
		new_conn_state = drm_atomic_get_connector_state(state,
								set->connectors[i]);
		if (IS_ERR(new_conn_state))
			return PTR_ERR(new_conn_state);

		ret = drm_atomic_set_crtc_for_connector(new_conn_state,
							set->crtc);
		if (ret)
			return ret;
	}

	for_each_new_crtc_in_state(state, crtc, new_crtc_state, i) {
		/*
		* Don't update ->enable for the CRTC in the set_config request,
		* since a mismatch would indicate a bug in the upper layers.
		* The actual modeset code later on will catch any
		* inconsistencies here.
		*/
		if (crtc == set->crtc)
			continue;

		if (!new_crtc_state->connector_mask) {
			ret = drm_atomic_set_mode_prop_for_crtc(new_crtc_state,
								NULL);
			if (ret < 0)
				return ret;

			new_crtc_state->active = false;
		}
	}

	return 0;
}

static int komeda_fb_helper_set_config(struct drm_fb_helper *fb_helper,
			struct drm_mode_set *set, struct drm_atomic_state *state)
{
	struct komeda_fbdev *kfbdev = container_of(fb_helper, struct komeda_fbdev, helper);
	struct drm_crtc_state *crtc_state;
	struct drm_plane_state *primary_state;
	struct drm_crtc *crtc = set->crtc;
	int ret;

	crtc_state = drm_atomic_get_crtc_state(state, crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	primary_state = drm_atomic_get_plane_state(state, crtc->primary);
	if (IS_ERR(primary_state))
		return PTR_ERR(primary_state);

	if (!set->mode) {
		WARN_ON(set->fb);
		WARN_ON(set->num_connectors);

		ret = drm_atomic_set_mode_for_crtc(crtc_state, NULL);
		if (ret != 0)
			return ret;

		crtc_state->active = false;

		ret = drm_atomic_set_crtc_for_plane(primary_state, NULL);
		if (ret != 0)
			return ret;

		drm_atomic_set_fb_for_plane(primary_state, NULL);

		goto commit;
	}

	WARN_ON(!set->fb);
	WARN_ON(!set->num_connectors);

	ret = drm_atomic_set_mode_for_crtc(crtc_state, set->mode);
	if (ret != 0)
		return ret;

	crtc_state->active = true;

	ret = drm_atomic_set_crtc_for_plane(primary_state, crtc);
	if (ret != 0)
		return ret;

	drm_atomic_set_fb_for_plane(primary_state, set->fb);
	primary_state->crtc_x = kfbdev->crtc_x;
	primary_state->crtc_y = kfbdev->crtc_y;
	primary_state->crtc_w = kfbdev->crtc_w;
	primary_state->crtc_h = kfbdev->crtc_h;
	primary_state->src_x = kfbdev->src_x << 16;
	primary_state->src_y = kfbdev->src_y << 16;
	primary_state->src_w = kfbdev->src_w << 16;
	primary_state->src_h = kfbdev->src_h << 16;

commit:
	ret = update_output_state(state, set);
	if (ret)
		return ret;

	return 0;
}

static int komeda_fbdev_fill_mode_cmd(struct fb_info *info, uint32_t fmt,
			struct drm_mode_fb_cmd2 *mode_cmd, size_t *size)
{
	int ret = 0;
	struct fb_var_screeninfo *var = &info->var;

	switch (fmt) {
		case DRM_FORMAT_RGB888:
			mode_cmd->width = var->xres;
			mode_cmd->height = var->yres * CONFIG_DRM_FBDEV_OVERALLOC / 100;
			mode_cmd->pitches[0] = var->xres * 3;
			mode_cmd->pixel_format = fmt;
			*size = PAGE_ALIGN(mode_cmd->pitches[0] * mode_cmd->height);
			break;

		case DRM_FORMAT_XRGB8888:
			mode_cmd->width = var->xres;
			mode_cmd->height = var->yres * CONFIG_DRM_FBDEV_OVERALLOC / 100;
			mode_cmd->pitches[0] = var->xres * 4;
			mode_cmd->pixel_format = fmt;
			*size = PAGE_ALIGN(mode_cmd->pitches[0] * mode_cmd->height);
			break;

		case DRM_FORMAT_YUV420:
			mode_cmd->width = var->xres;
			mode_cmd->height = var->yres * CONFIG_DRM_FBDEV_OVERALLOC / 100;
			mode_cmd->pitches[0] = var->xres;
			mode_cmd->pitches[1] = var->xres / 2;
			mode_cmd->pitches[2] = var->xres / 2;
			mode_cmd->offsets[0] = 0;
			mode_cmd->offsets[1] = mode_cmd->pitches[0] * var->yres;
			mode_cmd->offsets[2] = mode_cmd->offsets[1] + mode_cmd->pitches[1] * (var->yres / 2);
			mode_cmd->pixel_format = fmt;
			*size = PAGE_ALIGN(mode_cmd->width * mode_cmd->height * 3 / 2);
			break;

		case DRM_FORMAT_NV12:
			mode_cmd->width = var->xres;
			mode_cmd->height = var->yres * CONFIG_DRM_FBDEV_OVERALLOC / 100;
			mode_cmd->pitches[0] = var->xres;
			mode_cmd->pitches[1] = var->xres;
			mode_cmd->offsets[0] = 0;
			mode_cmd->offsets[1] = mode_cmd->pitches[0] * var->yres;
			mode_cmd->pixel_format = fmt;
			*size = PAGE_ALIGN(mode_cmd->width * mode_cmd->height * 3 / 2);
			break;

		default:
			ret = -EINVAL;
			DRM_ERROR("Bad fmt 0x%x\n", fmt);
			break;
	}

	return ret;
}

static void komeda_remove_fb(struct drm_fb_helper *helper)
{
	struct komeda_fbdev *kfbdev = container_of(helper, struct komeda_fbdev, helper);
	struct drm_framebuffer *fb;
	int i;

	if (kfbdev->old_fb) {
		fb = &kfbdev->old_fb->base;
		komeda_gem_object_put_locked(fb->obj[0]);

		for (i = 0; i < fb->format->num_planes; i++)
			fb->obj[i] = NULL;

		list_del_init(&fb->filp_head);
		drm_framebuffer_put(&kfbdev->old_fb->base);
		kfbdev->old_fb = NULL;
		DRM_INFO("%s: success remove drm framebuffer!\n", __func__);
	}
}

static int komeda_fb_create_new_fb(struct fb_info *info, struct drm_mode_fb_cmd2 *mode_cmd,
		struct drm_gem_cma_object *obj, size_t size)
{
	int ret;
	struct komeda_fbdev *kfbdev = info->par;
	struct drm_fb_helper *helper = &kfbdev->helper;

	kfbdev->fb = (struct komeda_fb *)komeda_fbdev_fb_init(helper->dev, mode_cmd, &obj->base);
	if (IS_ERR(kfbdev->fb)) {
		ret = PTR_ERR(kfbdev->fb);
		kfbdev->fb = NULL;
		DRM_ERROR("failed to initialize framebuffer: %d\n", ret);
		return ret;
	}

	kfbdev->size = size;
	helper->fb = &kfbdev->fb->base;

	komeda_fb_helper_fill_info(info,helper);

	info->screen_base = obj->vaddr;
	info->screen_size = size;
	info->fix.smem_start = (unsigned long)obj->paddr;
	info->fix.smem_len = size;

	helper->client.modesets->fb = helper->fb;

	DRM_INFO("%s: success create new drm framebuffer!\n", __func__);
	return 0;
}

static void komeda_fbdev_create_mode_cmd(struct komeda_mode_fb_cmd *fb_cmd, struct drm_mode_fb_cmd2 *mode_cmd)
{
	int i;

	mode_cmd->width = fb_cmd->width;
	mode_cmd->height = fb_cmd->height;
	mode_cmd->pixel_format = fb_cmd->pixel_format;
	mode_cmd->flags = fb_cmd->flags;

	for (i = 0; i < 4; i++) {
		mode_cmd->pitches[i] = fb_cmd->pitches[i];
		mode_cmd->offsets[i] = fb_cmd->offsets[i];
	}
}

static int komeda_fbdev_get_obj(struct fb_info *info, int fd, struct drm_gem_cma_object **obj)
{
	int ret = 0;
	struct dma_buf *dma_buf;
	struct drm_gem_object *gem_obj;
	struct drm_gem_cma_object *cma_obj;
	struct komeda_fbdev *kfbdev = info->par;
	struct drm_device *dev = kfbdev->dev;

	dma_buf = dma_buf_get(fd);
	if (IS_ERR(dma_buf))
		return PTR_ERR(dma_buf);

	gem_obj = dev->driver->gem_prime_import(dev, dma_buf);
	if (IS_ERR(gem_obj)) {
		ret = PTR_ERR(gem_obj);
		goto err;
	}

	cma_obj = container_of(gem_obj, struct drm_gem_cma_object, base);
	*obj = cma_obj;

err:
	dma_buf_put(dma_buf);
	return ret;
}

static struct komeda_fb * komeda_fbdev_find_cache_fb(struct komeda_fbdev *kfbdev, int fd)
{
	struct komeda_fbdev_fb_cache *fb_cache;

	if (kfbdev->fb_cache_header == NULL)
		return NULL;

	fb_cache = (struct komeda_fbdev_fb_cache *)kfbdev->fb_cache_header;
	do {
		if (fb_cache->fd == fd)
			return fb_cache->fb;

		fb_cache = (struct komeda_fbdev_fb_cache *)fb_cache->next;
	} while (fb_cache != NULL);

	return NULL;
}

static int komeda_fbdev_insert_cache_fb(struct komeda_fbdev *kfbdev, struct komeda_fb *fb, int fd)
{
	struct komeda_fbdev_fb_cache *fb_cache;
	struct komeda_fbdev_fb_cache *tmp_cache;

	fb_cache = kzalloc(sizeof(struct komeda_fbdev_fb_cache), GFP_KERNEL);
	if (!fb_cache)
		return -ENOMEM;

	fb_cache->fd = fd;
	fb_cache->fb = fb;
	fb_cache->next = NULL;

	if (kfbdev->fb_cache_header == NULL) {
		kfbdev->fb_cache_header = fb_cache;
		return 0;
	}

	tmp_cache = (struct komeda_fbdev_fb_cache *)kfbdev->fb_cache_header;
	do {
		if (tmp_cache->next == NULL) {
			tmp_cache->next = fb_cache;
			break;
		}
		tmp_cache = (struct komeda_fbdev_fb_cache *)tmp_cache->next;
	} while(1);

	return 0;
}

static int komeda_fbdev_rm_cache_fb(struct komeda_fbdev *kfbdev)
{
	struct komeda_fbdev_fb_cache *fb_cache;
	struct komeda_fbdev_fb_cache *old_fb_cache;
	struct drm_framebuffer *fb;
	int i;

	if (kfbdev->fb_cache_header == NULL)
		return 0;

	fb_cache = (struct komeda_fbdev_fb_cache *)kfbdev->fb_cache_header;
	do {
		old_fb_cache = fb_cache;
		fb_cache = (struct komeda_fbdev_fb_cache *)fb_cache->next;

		fb = &old_fb_cache->fb->base;
		komeda_gem_object_put_locked(fb->obj[0]);

		for (i = 0; i < fb->format->num_planes; i++)
			fb->obj[i] = NULL;

		list_del_init(&fb->filp_head);
		drm_framebuffer_put(fb);
		old_fb_cache->fb = NULL;
		kfree(old_fb_cache);
	} while (fb_cache != NULL);

	kfbdev->fb_cache_header = NULL;
	return 0;
}

static void komeda_fbdev_restore_fb(struct fb_info *info)
{
	size_t size;
	struct komeda_fbdev *kfbdev = info->par;
	struct drm_fb_helper *helper = &kfbdev->helper;
	struct drm_mode_set *set = helper->client.modesets;
	struct drm_mode_fb_cmd2 mode_cmd = {0};
	struct drm_gem_object *gem_obj;
	struct drm_gem_cma_object *cma_obj;
	int hdisplay, vdisplay;

	kfbdev->fb = kfbdev->base_fb;
	helper->fb = &kfbdev->fb->base;

	komeda_fb_helper_fill_info(info,helper);
	gem_obj = kfbdev->fb->base.obj[0];
	cma_obj = container_of(gem_obj, struct drm_gem_cma_object, base);

	komeda_fbdev_fill_mode_cmd(info, kfbdev->fb->base.format->format, &mode_cmd, &size);
	info->screen_base = cma_obj->vaddr;
	info->screen_size = size;
	info->fix.smem_start = (unsigned long)cma_obj->paddr;
	info->fix.smem_len = size;

	helper->client.modesets->fb = helper->fb;

	drm_mode_get_hv_timing(set->mode, &hdisplay, &vdisplay);
	kfbdev->crtc_x = 0;
	kfbdev->crtc_y = 0;
	kfbdev->crtc_w = hdisplay;
	kfbdev->crtc_h = vdisplay;
	kfbdev->src_x = 0;
	kfbdev->src_y = 0;
	kfbdev->src_w = hdisplay;
	kfbdev->src_h = vdisplay;
	komeda_pan_display_atomic(&info->var, info);
	komeda_fbdev_rm_cache_fb(kfbdev);

	DRM_INFO("success restore fb!\n");
}

static int komeda_fb_helper_ioctl(struct fb_info *info, unsigned int cmd,
			unsigned long arg)
{
	int ret = 0;
	void __user *argp = (void __user *)arg;
	struct komeda_fbdev *kfbdev = info->par;
	struct drm_fb_helper *helper = &kfbdev->helper;
	struct drm_crtc *crtc;
	struct drm_plane_state *primary_state;
	mutex_lock(&helper->lock);
	switch (cmd) {
	case FBIO_ADD_FB: {
		uint32_t fmt;
		size_t size;
		struct drm_mode_fb_cmd2 mode_cmd = {0};
		struct drm_gem_cma_object *obj;
		if (copy_from_user(&fmt, argp, sizeof(fmt))) {
			ret = -EFAULT;
			goto unlock;
		}

		if (kfbdev->fb) {
			struct drm_crtc *crtc = helper->client.modesets->crtc;
			kfbdev->old_fb = kfbdev->fb;
			kfbdev->fb = NULL;
			helper->client.modesets->fb = NULL;
			crtc->primary->fb = NULL;
			helper->fb = NULL;
		}

		ret = komeda_fbdev_fill_mode_cmd(info, fmt, &mode_cmd, &size);
		if (ret < 0)
			goto unlock;

		obj = drm_gem_cma_create(helper->dev, size);
		if (IS_ERR(obj)) {
			ret = PTR_ERR(obj);
			DRM_ERROR("failed to create cma: %d\n", ret);
			goto unlock;
		}

		ret = komeda_fb_create_new_fb(info, &mode_cmd, obj, size);
		if (ret < 0) {
			komeda_gem_object_put_locked(&obj->base);
			goto unlock;
		}
	}
	break;

	case FBIO_SET_FB: {
		struct komeda_fb *fb;
		struct komeda_mode_fb_cmd fb_cmd;
		struct drm_gem_cma_object *obj = NULL;
		struct drm_gem_object *gem_obj;
		crtc = helper->client.modesets->crtc;
		primary_state = crtc->primary->state;

		if (copy_from_user(&fb_cmd, argp, sizeof(fb_cmd))) {
			pr_err("%s: failed to copy fb cmd!\n", __func__);
			ret = -EFAULT;
			goto unlock;
		}

		fb = komeda_fbdev_find_cache_fb(kfbdev, fb_cmd.fd);
		if (fb == NULL) {
			struct drm_mode_fb_cmd2 mode_cmd = {0};
			komeda_fbdev_create_mode_cmd(&fb_cmd, &mode_cmd);
			ret = komeda_fbdev_get_obj(info, fb_cmd.fd, &obj);
			if (ret) {
				DRM_ERROR("%s: failed to get obj!\n", __func__);
				ret = -EFAULT;
				goto unlock;
			}

			ret = komeda_fb_create_new_fb(info, &mode_cmd, obj, fb_cmd.size);
			if (ret < 0)
				goto unlock;
			komeda_fbdev_insert_cache_fb(kfbdev, kfbdev->fb, fb_cmd.fd);

			if ((fb_cmd.width > MAX_YUV_LINE_SIZE) &&
				(fb_cmd.pixel_format == DRM_FORMAT_NV12))
				komeda_fbdev_plane_split_enable(primary_state, true);
		} else {
			kfbdev->fb = fb;
			kfbdev->size = fb_cmd.size;
			helper->fb = &kfbdev->fb->base;
			komeda_fb_helper_fill_info(info,helper);
			gem_obj = fb->base.obj[0];
			obj = container_of(gem_obj, struct drm_gem_cma_object, base);
			info->screen_base = obj->vaddr;
			info->screen_size = fb_cmd.size;
			info->fix.smem_start = (unsigned long)obj->paddr;
			info->fix.smem_len = fb_cmd.size;
			helper->client.modesets->fb = helper->fb;
		}
	}
	break;

	case FBIO_SET_CROP: {
		struct komeda_fb_crop fb_crop;
		if (copy_from_user(&fb_crop, argp, sizeof(fb_crop))) {
			ret = -EFAULT;
			goto unlock;
		}
		kfbdev->crtc_x = fb_crop.crtc_x;
		kfbdev->crtc_y = fb_crop.crtc_y;
		kfbdev->crtc_w = fb_crop.crtc_w;
		kfbdev->crtc_h = fb_crop.crtc_h;
		kfbdev->src_x = fb_crop.src_x;
		kfbdev->src_y = fb_crop.src_y;
		kfbdev->src_w = fb_crop.src_w;
		kfbdev->src_h = fb_crop.src_h;
	}
	break;

	case FBIO_RESTORE_FB:
		crtc = helper->client.modesets->crtc;
		primary_state = crtc->primary->state;

		komeda_fbdev_plane_split_enable(primary_state, false);
		komeda_fbdev_restore_fb(info);
		break;

	default:
		ret = -ENOTTY;
		break;
	}

unlock:
	mutex_unlock(&helper->lock);
	return ret;
}
extern int __drm_atomic_helper_disable_plane(struct drm_plane *plane,struct drm_plane_state *plane_state);

static int komeda_client_modeset_commit_atomic(struct drm_fb_helper *fb_helper,struct drm_client_dev *client, bool active, bool check)
{
	struct drm_device *dev = client->dev;
	struct drm_plane *plane;
	struct drm_atomic_state *state;
	struct drm_modeset_acquire_ctx ctx;
	struct drm_mode_set *mode_set;
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
		bool usabled = false;
		drm_client_for_each_modeset(mode_set, client) {
			struct drm_crtc *crtc = mode_set->crtc;
			if(plane->possible_crtcs & drm_crtc_mask(crtc)){
				usabled = true;
			}
		}

		if(!usabled)
			continue;

		plane_state = drm_atomic_get_plane_state(state, plane);
		if (IS_ERR(plane_state)) {
			ret = PTR_ERR(plane_state);
			goto out_state;
		}

		plane_state->rotation = DRM_MODE_ROTATE_0;

		/* disable non-primary: */
		if (plane->type == DRM_PLANE_TYPE_PRIMARY)
			continue;

		ret = __drm_atomic_helper_disable_plane(plane, plane_state);
		if (ret != 0)
			goto out_state;
	}

	drm_client_for_each_modeset(mode_set, client) {
		struct drm_plane *primary = mode_set->crtc->primary;
		unsigned int rotation;

		if (drm_client_rotation(mode_set, &rotation)) {
			struct drm_plane_state *plane_state;

			/* Cannot fail as we've already gotten the plane state above */
			plane_state = drm_atomic_get_new_plane_state(state, primary);
			plane_state->rotation = rotation;
		}

		ret = komeda_fb_helper_set_config(fb_helper,mode_set, state);
		if (ret != 0)
			goto out_state;

		/*
		* __drm_atomic_helper_set_config() sets active when a
		* mode is set, unconditionally clear it if we force DPMS off
		*/
		if (!active) {
			struct drm_crtc *crtc = mode_set->crtc;
			struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

			crtc_state->active = false;
		}
	}

	if (check)
		ret = drm_atomic_check_only(state);
	else
		ret = drm_atomic_commit(state);

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

static int komeda_client_modeset_commit_locked(struct drm_fb_helper *fb_helper,struct drm_client_dev *client)
{
	int ret;

	mutex_lock(&client->modeset_mutex);
	ret = komeda_client_modeset_commit_atomic(fb_helper,client, true, false);
	mutex_unlock(&client->modeset_mutex);

	return ret;
}

static int komeda_pan_display_atomic(struct fb_var_screeninfo *var,struct fb_info *info)
{
	int ret;
	struct drm_fb_helper *fb_helper = info->par;
	ret = komeda_client_modeset_commit_locked(fb_helper,&fb_helper->client);
	komeda_remove_fb(fb_helper);
	return ret;
}

int  komeda_fb_helper_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	int ret;
	struct drm_fb_helper *fb_helper = info->par;
	mutex_lock(&fb_helper->lock);
	ret = komeda_pan_display_atomic(var, info);
	mutex_unlock(&fb_helper->lock);

	return ret;
}

static struct fb_ops komeda_fbdev_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var	= komeda_fb_helper_check_var,
	.fb_set_par	= komeda_fb_helper_set_par,
	.fb_setcmap	= drm_fb_helper_setcmap,
	.fb_blank	= drm_fb_helper_blank,
	.fb_pan_display	= komeda_fb_helper_pan_display,
	.fb_debug_enter = drm_fb_helper_debug_enter,
	.fb_debug_leave = drm_fb_helper_debug_leave,
	.fb_ioctl	= komeda_fb_helper_ioctl,
	.fb_fillrect	= drm_fb_helper_cfb_fillrect,
	.fb_copyarea	= drm_fb_helper_cfb_copyarea,
	.fb_imageblit	= drm_fb_helper_cfb_imageblit,
	.fb_mmap	= drm_fb_cma_mmap,
};

static int komeda_fbdev_fb_create(struct drm_fb_helper *helper,
			       struct drm_fb_helper_surface_size *sizes)
{
	struct komeda_fbdev *kfbdev = container_of(helper, struct komeda_fbdev, helper);
	struct drm_mode_set *set = helper->client.modesets;
	struct fb_info *info;
	struct drm_mode_fb_cmd2 mode_cmd = { 0 };
	struct drm_gem_cma_object *obj;
	int hdisplay, vdisplay;
	int ret = 0;
	size_t size;

	DRM_DEBUG_DRIVER("surface width(%d), height(%d) and bpp(%d)\n",
			 sizes->surface_width, sizes->surface_height,
			 sizes->surface_bpp);

	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;
	mode_cmd.pitches[0] = sizes->surface_width * 4;
	mode_cmd.offsets[0] = 0;
	mode_cmd.pixel_format = DRM_FORMAT_XRGB8888;
	size = sizes->surface_width * sizes->surface_height * 4;

	obj = drm_gem_cma_create(helper->dev, size);
	if (IS_ERR(obj))
		return -ENOMEM;

	info = drm_fb_helper_alloc_fbi(helper);
	if (IS_ERR(info)) {
		ret = PTR_ERR(info);
		DRM_ERROR("failed to allocate fbi: %d\n", ret);
		goto err_gem_free_object;
	}

	info->par = kfbdev;

	kfbdev->fb = (struct komeda_fb *)komeda_fbdev_fb_init(helper->dev, &mode_cmd, &obj->base);
	if (IS_ERR(kfbdev->fb)) {
		ret = PTR_ERR(kfbdev->fb);
		kfbdev->fb = NULL;
		DRM_ERROR("failed to initialize framebuffer: %d\n", ret);
		goto err_fb_info_destroy;
	}

	kfbdev->size = size;
	helper->fb = &kfbdev->fb->base;

	strcpy(info->fix.id, "komedafb");

	info->fbops = &komeda_fbdev_fb_ops;

	drm_fb_helper_fill_info(info,&kfbdev->helper,sizes);

	info->screen_base = obj->vaddr;
	info->screen_size = size;

	info->fix.smem_start = (unsigned long)obj->paddr;
	info->fix.smem_len = size;

	drm_mode_get_hv_timing(set->mode, &hdisplay, &vdisplay);
	kfbdev->crtc_x = 0;
	kfbdev->crtc_y = 0;
	kfbdev->crtc_w = hdisplay;
	kfbdev->crtc_h = vdisplay;
	kfbdev->src_x = 0;
	kfbdev->src_y = 0;
	kfbdev->src_w = hdisplay;
	kfbdev->src_h = vdisplay;

	kfbdev->old_fb = NULL;
	kfbdev->base_fb = kfbdev->fb;
	helper->client.modesets->fb = helper->fb;
	return 0;

err_fb_info_destroy:
	komeda_fbdev_fini(helper);
err_gem_free_object:
	komeda_gem_object_put_locked(&obj->base);

	return ret;
}

static void komeda_fbdev_destroy(struct komeda_fbdev *fbdev)
{
	struct komeda_fb *kfb = fbdev->fb;
	struct drm_fb_helper *fbh = &fbdev->helper;

	drm_fb_helper_unregister_fbi(fbh);

	drm_fb_helper_fini(fbh);

	if (kfb)
		drm_framebuffer_put(&kfb->base);
}

static const struct drm_fb_helper_funcs komeda_fbdev_helper_funcs = {
	.fb_probe = komeda_fbdev_fb_create,
};

static int komeda_drm_client_modeset_create(struct drm_client_dev *client,int index)
{
	struct drm_device *dev = client->dev;
	unsigned int num_crtc = dev->mode_config.num_crtc;
	unsigned int max_connector_count = 1;
	struct drm_mode_set *modeset;
	struct drm_crtc *crtc;
	unsigned int i = 0;
	/* Add terminating zero entry to enable index less iteration */
	client->modesets = kcalloc(num_crtc + 1, sizeof(*client->modesets), GFP_KERNEL);
	if (!client->modesets)
		return -ENOMEM;

	mutex_init(&client->modeset_mutex);

	drm_for_each_crtc(crtc, dev){
		if (index == i++) {
			client->modesets[0].crtc = crtc;
			break;
		}
	}

	/* Cloning is only supported in the single crtc case. */
	if (num_crtc == 1)
		max_connector_count = DRM_CLIENT_MAX_CLONED_CONNECTORS;

	modeset = client->modesets;
	modeset->connectors = kcalloc(1,sizeof(*modeset->connectors), GFP_KERNEL);
	if (!modeset->connectors)
		goto err_free;

	return 0;

err_free:
	drm_client_modeset_free(client);

	return -ENOMEM;
}

extern struct drm_file *drm_file_alloc(struct drm_minor *minor);

static int drm_client_open(struct drm_client_dev *client)
{
	struct drm_device *dev = client->dev;
	struct drm_file *file;

	file = drm_file_alloc(dev->primary);
	if (IS_ERR(file))
		return PTR_ERR(file);

	mutex_lock(&dev->filelist_mutex);
	list_add(&file->lhead, &dev->filelist_internal);
	mutex_unlock(&dev->filelist_mutex);

	client->file = file;

	return 0;
}

static int komeda_drm_client_init(struct drm_device *dev, struct drm_client_dev *client,
			const char *name, const struct drm_client_funcs *funcs,int index)
{
	int ret;

	if (!drm_core_check_feature(dev, DRIVER_MODESET) || !dev->driver->dumb_create)
		return -EOPNOTSUPP;

	if (funcs && !try_module_get(funcs->owner))
		return -ENODEV;

	client->dev = dev;
	client->name = name;
	client->funcs = funcs;

	ret = komeda_drm_client_modeset_create(client,index);
	if (ret)
		goto err_put_module;

	ret = drm_client_open(client);
	if (ret)
		goto err_free;

	drm_dev_get(dev);

	return 0;

err_free:
	drm_client_modeset_free(client);
err_put_module:
	if (funcs)
		module_put(funcs->owner);


	return ret;
}


static int komeda_drm_fb_helper_init(struct drm_device *dev,
			struct drm_fb_helper *fb_helper,int index)
{
	int ret;

	if (!drm_fbdev_emulation) {
		dev->fb_helper = fb_helper;
		return 0;
	}

	/*
	* If this is not the generic fbdev client, initialize a drm_client
	* without callbacks so we can use the modesets.
	*/
	if (!fb_helper->client.funcs) {
		ret = komeda_drm_client_init(dev, &fb_helper->client, "drm_fb_helper", NULL,index);
		if (ret)
			return ret;
	}

	dev->fb_helper = fb_helper;

	return 0;
}

int komeda_fbdev_init(struct drm_device *dev)
{
	int ret, i;
	int crtc_count = dev->mode_config.num_crtc;
	struct komeda_dev *priv = dev->dev_private;
	for (i = 0; i < crtc_count; i++) {
		struct komeda_fbdev *kfbdev;
		struct drm_fb_helper *helper;
		struct fb_var_screeninfo *var;
		struct fb_fix_screeninfo *fix;
		struct drm_mode_set *set;
		int hdisplay, vdisplay;

		kfbdev = devm_kzalloc(priv->dev, sizeof(*kfbdev), GFP_KERNEL);
		if (!kfbdev) {
			DRM_ERROR("failed to allocate hibmc_fbdev\n");
			return -ENOMEM;
		}

		kfbdev->fb_cache_header = NULL;
		kfbdev->dev = dev;
		priv->fbdev[i] = kfbdev;
		priv->fbdev[i]->id = i;
		helper = &kfbdev->helper;
		init_completion(&kfbdev->hpd_done);

		drm_fb_helper_prepare(dev, helper, &komeda_fbdev_helper_funcs);

		ret = komeda_drm_fb_helper_init(dev, helper,i);
		if (ret) {
			DRM_ERROR("failed to initialize fb helper: %d\n", ret);
			devm_kfree(priv->dev, kfbdev);
			return ret;
		}

		ret = drm_fb_helper_initial_config(helper, 32);
		if (ret) {
			DRM_ERROR("failed to setup initial conn config: %d\n", ret);
			drm_fb_helper_fini(helper);
			devm_kfree(priv->dev, kfbdev);
			return ret;
		}

		var = &helper->fbdev->var;
		fix = &helper->fbdev->fix;

		set = helper->client.modesets;
		if(set->mode == NULL)
			return 0;

		drm_mode_get_hv_timing(set->mode, &hdisplay, &vdisplay);
		kfbdev->crtc_x = 0;
		kfbdev->crtc_y = 0;
		kfbdev->crtc_w = hdisplay;
		kfbdev->crtc_h = vdisplay;
		kfbdev->src_x = 0;
		kfbdev->src_y = 0;
		kfbdev->src_w = hdisplay;
		kfbdev->src_h = vdisplay;

		DRM_INFO("%s: successfull add fbdev, crtc id: %d\n",
			__func__, helper->client.modesets->crtc->base.id);
	}

	return 0;
}

void komeda_fbdev_fini(struct drm_fb_helper *helper)
{
	struct komeda_fbdev *kfbdev = container_of(helper, struct komeda_fbdev, helper);
	komeda_fbdev_destroy(kfbdev);
	kfbdev = NULL;
}
#endif
