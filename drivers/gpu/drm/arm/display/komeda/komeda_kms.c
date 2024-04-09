// SPDX-License-Identifier: GPL-2.0
/*
 * (C) COPYRIGHT 2018 ARM Limited. All rights reserved.
 * Author: James.Qian.Wang <james.qian.wang@arm.com>
 *
 */
#include <linux/component.h>
#include <linux/interrupt.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_irq.h>
#include <drm/drm_managed.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>
#include <drm/drm_prime.h>

#include "komeda_dev.h"
#include "komeda_framebuffer.h"
#include "komeda_kms.h"
#include "komeda_fbdev.h"
#include "komeda_kms_agent.h"
DEFINE_DRM_GEM_CMA_FOPS(komeda_cma_fops);

static int komeda_gem_cma_dumb_create(struct drm_file *file,
				      struct drm_device *dev,
				      struct drm_mode_create_dumb *args)
{
	struct komeda_dev *mdev = dev->dev_private;
	u32 pitch = DIV_ROUND_UP(args->width * args->bpp, 8);

	args->pitch = ALIGN(pitch, mdev->chip.bus_width);

	return drm_gem_cma_dumb_create_internal(file, dev, args);
}

static irqreturn_t komeda_kms_irq_handler(int irq, void *data)
{
	struct drm_device *drm = data;
	struct komeda_dev *mdev = drm->dev_private;
	struct komeda_kms_dev *kms = to_kdev(drm);
	struct komeda_events evts;
	irqreturn_t status;
	u32 i;

	/* Call into the CHIP to recognize events */
	memset(&evts, 0, sizeof(evts));
	status = mdev->funcs->irq_handler(mdev, &evts);

	komeda_print_events(&evts, drm);

	/* Notify the crtc to handle the events */
	for (i = 0; i < kms->n_crtcs; i++)
		komeda_crtc_handle_event(&kms->crtcs[i], &evts);

	return status;
}

static void komeda_lastclose(struct drm_device *drm)
{
	struct komeda_dev *mdev = drm->dev_private;
	int i;

	for (i = 0; i < 2; i++) {
		if (mdev->fbdev[i] != NULL)
			drm_fb_helper_restore_fbdev_mode_unlocked(&mdev->fbdev[i]->helper);
	}
}

static struct drm_driver komeda_kms_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.lastclose			= komeda_lastclose,
	DRM_GEM_CMA_DRIVER_OPS_WITH_DUMB_CREATE(komeda_gem_cma_dumb_create),
	.fops = &komeda_cma_fops,
	.name = "komeda",
	.desc = "Arm Komeda Display Processor driver",
	.date = "20181101",
	.major = 0,
	.minor = 1,
	.gem_free_object_unlocked       = drm_gem_cma_free_object,
	.gem_vm_ops                     = &drm_gem_cma_vm_ops,
	.dumb_create                    = komeda_gem_cma_dumb_create,
	.prime_handle_to_fd             = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle             = drm_gem_prime_fd_to_handle,
	.gem_prime_export               = drm_gem_prime_export,
	.gem_prime_import               = drm_gem_prime_import,
	.gem_prime_get_sg_table         = drm_gem_cma_prime_get_sg_table,
	.gem_prime_vmap                 = drm_gem_cma_prime_vmap,
	.gem_prime_vunmap               = drm_gem_cma_prime_vunmap,

};

static void komeda_kms_commit_tail(struct drm_atomic_state *old_state)
{
	struct drm_device *dev = old_state->dev;

	drm_atomic_helper_commit_modeset_disables(dev, old_state);

	drm_atomic_helper_commit_planes(dev, old_state,
					DRM_PLANE_COMMIT_ACTIVE_ONLY);

	drm_atomic_helper_commit_modeset_enables(dev, old_state);

	drm_atomic_helper_wait_for_flip_done(dev, old_state);

	drm_atomic_helper_commit_hw_done(old_state);

	drm_atomic_helper_cleanup_planes(dev, old_state);
}

static const struct drm_mode_config_helper_funcs komeda_mode_config_helpers = {
	.atomic_commit_tail = komeda_kms_commit_tail,
};

static int komeda_plane_state_list_add(struct drm_plane_state *plane_st,
				       struct list_head *zorder_list)
{
	struct komeda_plane_state *new = to_kplane_st(plane_st);
	struct komeda_plane_state *node, *last;

	last = list_empty(zorder_list) ?
	       NULL : list_last_entry(zorder_list, typeof(*last), zlist_node);

	/* Considering the list sequence is zpos increasing, so if list is empty
	 * or the zpos of new node bigger than the last node in list, no need
	 * loop and just insert the new one to the tail of the list.
	 */
	if (!last || (new->base.zpos > last->base.zpos)) {
		list_add_tail(&new->zlist_node, zorder_list);
		return 0;
	}

	/* Build the list by zpos increasing */
	list_for_each_entry(node, zorder_list, zlist_node) {
		if (new->base.zpos < node->base.zpos) {
			list_add_tail(&new->zlist_node, &node->zlist_node);
			break;
		} else if (node->base.zpos == new->base.zpos) {
			struct drm_plane *a = node->base.plane;
			struct drm_plane *b = new->base.plane;

			/* Komeda doesn't support setting a same zpos for
			 * different planes.
			 */
			DRM_DEBUG_ATOMIC("PLANE: %s and PLANE: %s are configured same zpos: %d.\n",
					 a->name, b->name, node->base.zpos);
			return -EINVAL;
		}
	}

	return 0;
}

static int komeda_crtc_normalize_zpos(struct drm_crtc *crtc,
				      struct drm_crtc_state *crtc_st)
{
	struct drm_atomic_state *state = crtc_st->state;
	struct komeda_crtc *kcrtc = to_kcrtc(crtc);
	struct komeda_crtc_state *kcrtc_st = to_kcrtc_st(crtc_st);
	struct komeda_plane_state *kplane_st;
	struct drm_plane_state *plane_st;
	struct drm_plane *plane;
	struct list_head zorder_list;
	int order = 0, err;

	DRM_DEBUG_ATOMIC("[CRTC:%d:%s] calculating normalized zpos values\n",
			 crtc->base.id, crtc->name);

	INIT_LIST_HEAD(&zorder_list);

	/* This loop also added all effected planes into the new state */
	drm_for_each_plane_mask(plane, crtc->dev, crtc_st->plane_mask) {
		plane_st = drm_atomic_get_plane_state(state, plane);
		if (IS_ERR(plane_st))
			return PTR_ERR(plane_st);

		/* Build a list by zpos increasing */
		err = komeda_plane_state_list_add(plane_st, &zorder_list);
		if (err)
			return err;
	}

	kcrtc_st->max_slave_zorder = 0;

	list_for_each_entry(kplane_st, &zorder_list, zlist_node) {
		plane_st = &kplane_st->base;
		plane = plane_st->plane;

		plane_st->normalized_zpos = order++;
		/* When layer_split has been enabled, one plane will be handled
		 * by two separated komeda layers (left/right), which may needs
		 * two zorders.
		 * - zorder: for left_layer for left display part.
		 * - zorder + 1: will be reserved for right layer.
		 */
		if (to_kplane_st(plane_st)->layer_split)
			order++;

		DRM_DEBUG_ATOMIC("[PLANE:%d:%s] zpos:%d, normalized zpos: %d\n",
				 plane->base.id, plane->name,
				 plane_st->zpos, plane_st->normalized_zpos);

		/* calculate max slave zorder */
		if (has_bit(drm_plane_index(plane), kcrtc->slave_planes))
			kcrtc_st->max_slave_zorder =
				max(plane_st->normalized_zpos,
				    kcrtc_st->max_slave_zorder);
	}

	crtc_st->zpos_changed = true;

	return 0;
}

static int komeda_kms_check(struct drm_device *dev,
			    struct drm_atomic_state *state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *new_crtc_st;
	int i, err;

	err = drm_atomic_helper_check_modeset(dev, state);
	if (err)
		return err;

	/* Komeda need to re-calculate resource assumption in every commit
	 * so need to add all affected_planes (even unchanged) to
	 * drm_atomic_state.
	 */
	for_each_new_crtc_in_state(state, crtc, new_crtc_st, i) {
		err = drm_atomic_add_affected_planes(state, crtc);
		if (err)
			return err;

		err = komeda_crtc_normalize_zpos(crtc, new_crtc_st);
		if (err)
			return err;
	}

	err = drm_atomic_helper_check_planes(dev, state);
	if (err)
		return err;

	return 0;
}

static void komeda_commit_tail(struct drm_atomic_state *old_state)
{
	struct drm_device *dev = old_state->dev;
	const struct drm_mode_config_helper_funcs *funcs;

	funcs = dev->mode_config.helper_private;

	drm_atomic_helper_wait_for_fences(dev, old_state, false);

	drm_atomic_helper_wait_for_dependencies(old_state);

	if (funcs && funcs->atomic_commit_tail)
		funcs->atomic_commit_tail(old_state);
	else
		drm_atomic_helper_commit_tail(old_state);

	drm_atomic_helper_commit_cleanup_done(old_state);

	drm_atomic_state_put(old_state);
}

static void komeda_commit_work(struct kthread_work *work)
{
	struct komeda_display_thread *display_thread = container_of(work,
			struct komeda_display_thread, commit_work);

	komeda_commit_tail(display_thread->state);
}

static void komeda_commit_flush_work(struct drm_device *dev,
			      struct drm_atomic_state *state)
{
	int i, j;
	struct drm_crtc *crtc;
	struct drm_crtc_state *crtc_state;
	struct komeda_dev *mdev = dev->dev_private;

	for_each_old_crtc_in_state(state, crtc, crtc_state, i)
		for (j = 0; j < mdev->n_pipelines; j++)
			if ((mdev->display_thread[j].thread != NULL)
				&& (mdev->display_thread[j].crtc_id == crtc->base.id)) {
				kthread_flush_worker(&mdev->display_thread[j].worker);
				return;
			}
}

static int komeda_commit_dispatch_work(struct drm_device *dev,
			      struct drm_atomic_state *state)
{
	int i, j;
	int ret = -ECANCELED;
	struct drm_crtc *crtc;
	struct drm_crtc_state *crtc_state;
	struct komeda_dev *mdev = dev->dev_private;

	for_each_old_crtc_in_state(state, crtc, crtc_state, i)
		for (j = 0; j < mdev->n_pipelines; j++)
			if ((mdev->display_thread[j].thread != NULL)
				&& (mdev->display_thread[j].crtc_id == crtc->base.id)) {
				mdev->display_thread[j].state = state;
				kthread_queue_work(&mdev->display_thread[j].worker,
					&mdev->display_thread[j].commit_work);
				return 0;
			}

	return ret;
}

static int komeda_drm_atomic_helper_commit(struct drm_device *dev,
			     struct drm_atomic_state *state,
			     bool nonblock)
{
	int ret, i;
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state, *new_crtc_state;
	struct komeda_crtc *kcrtc;

	if (state->async_update) {
		ret = drm_atomic_helper_prepare_planes(dev, state);
		if (ret)
			return ret;

		drm_atomic_helper_async_commit(dev, state);
		drm_atomic_helper_cleanup_planes(dev, state);

		return 0;
	}

	if (nonblock)
		komeda_commit_flush_work(dev, state);

	ret = drm_atomic_helper_setup_commit(state, nonblock);
	if (ret)
		return ret;

	for_each_oldnew_crtc_in_state(state, crtc, old_crtc_state, new_crtc_state, i) {
		kcrtc = to_kcrtc(crtc);
		kcrtc->commit = state->crtcs[i].commit;
	}

	ret = drm_atomic_helper_prepare_planes(dev, state);
	if (ret)
		return ret;

	if (!nonblock) {
		ret = drm_atomic_helper_wait_for_fences(dev, state, true);
		if (ret)
			goto err;
	}

	/*
	 * This is the point of no return - everything below never fails except
	 * when the hw goes bonghits. Which means we can commit the new state on
	 * the software side now.
	 */

	ret = drm_atomic_helper_swap_state(state, true);
	if (ret)
		goto err;

	for_each_oldnew_crtc_in_state(state, crtc, old_crtc_state, new_crtc_state, i) {
		kcrtc = to_kcrtc(crtc);
		kcrtc->commit = state->crtcs[i].commit;
	}

	/*
	 * Everything below can be run asynchronously without the need to grab
	 * any modeset locks at all under one condition: It must be guaranteed
	 * that the asynchronous work has either been cancelled (if the driver
	 * supports it, which at least requires that the framebuffers get
	 * cleaned up with drm_atomic_helper_cleanup_planes()) or completed
	 * before the new state gets committed on the software side with
	 * drm_atomic_helper_swap_state().
	 *
	 * This scheme allows new atomic state updates to be prepared and
	 * checked in parallel to the asynchronous completion of the previous
	 * update. Which is important since compositors need to figure out the
	 * composition of the next frame right after having submitted the
	 * current layout.
	 *
	 * NOTE: Commit work has multiple phases, first hardware commit, then
	 * cleanup. We want them to overlap, hence need system_unbound_wq to
	 * make sure work items don't artifically stall on each another.
	 */

	drm_atomic_state_get(state);

	if (nonblock) {
		ret = komeda_commit_dispatch_work(dev, state);
		if (ret < 0) {
			DRM_ERROR("Failed to dispatch work!\n");
			komeda_commit_tail(state);
		}
	} else {
		komeda_commit_tail(state);
	}

	return 0;

err:
	drm_atomic_helper_cleanup_planes(dev, state);
	return ret;
}

static void komeda_output_poll_changed(struct drm_device *drm)
{
	struct komeda_dev *mdev = drm->dev_private;
	int i;
	for (i = 0; i < 2; i++) {
		if (mdev->fbdev[i] != NULL) {
			drm_fb_helper_hotplug_event(&mdev->fbdev[i]->helper);
			complete(&mdev->fbdev[i]->hpd_done);
		}
	}
}

static const struct drm_mode_config_funcs komeda_mode_config_funcs = {
	.fb_create		= komeda_fb_create,
	.atomic_check		= komeda_kms_check,
	.atomic_commit		= komeda_drm_atomic_helper_commit,
	.output_poll_changed    = komeda_output_poll_changed,
};

static void komeda_kms_mode_config_init(struct komeda_kms_dev *kms,
					struct komeda_dev *mdev)
{
	struct drm_mode_config *config = &kms->base.mode_config;

	drm_mode_config_init(&kms->base);

	komeda_kms_setup_crtcs(kms, mdev);

	/* Get value from dev */
	config->min_width	= 0;
	config->min_height	= 0;
	config->max_width	= 4096;
	config->max_height	= 4096;
	config->allow_fb_modifiers = true;

	config->funcs = &komeda_mode_config_funcs;
	config->helper_private = &komeda_mode_config_helpers;
}

static void komeda_display_thread_create(struct komeda_kms_dev *kms, struct komeda_dev *mdev)
{
	int i, ret;
	struct sched_param param = {0};

	param.sched_priority = 16;
	for (i = 0; i < kms->n_crtcs; i++) {
		mdev->display_thread[i].crtc_id = kms->crtcs[i].base.base.id;
		kthread_init_worker(&mdev->display_thread[i].worker);
		kthread_init_work(&mdev->display_thread[i].commit_work, komeda_commit_work);
		mdev->display_thread[i].thread = kthread_run(kthread_worker_fn,
					&mdev->display_thread[i].worker,
					"crtc_commit:%d",mdev->display_thread[i].crtc_id);
		if (IS_ERR(mdev->display_thread[i].thread)) {
			DRM_ERROR("Failed to create crtc commit thread!\n");
			mdev->display_thread[i].thread = NULL;
		}
		ret = sched_setscheduler(mdev->display_thread[i].thread, SCHED_FIFO, &param);
		if (ret)
			DRM_WARN("Failed to set display thread priority!\n");
	}
}


struct komeda_kms_dev *komeda_kms_attach(struct komeda_dev *mdev)
{
	struct komeda_kms_dev *kms;
	struct drm_device *drm;
	int err;

	kms = devm_drm_dev_alloc(mdev->dev, &komeda_kms_driver,
				 struct komeda_kms_dev, base);
	if (IS_ERR(kms))
		return kms;

	drm = &kms->base;

	drm->dev_private = mdev;

	komeda_kms_mode_config_init(kms, mdev);

	err = komeda_kms_add_private_objs(kms, mdev);
	if (err)
		goto cleanup_mode_config;

	err = komeda_kms_add_planes(kms, mdev);
	if (err)
		goto cleanup_mode_config;

	err = drm_vblank_init(drm, kms->n_crtcs);
	if (err)
		goto cleanup_mode_config;

	err = komeda_kms_add_crtcs(kms, mdev);
	if (err)
		goto cleanup_mode_config;

	err = komeda_kms_add_wb_connectors(kms, mdev);
	if (err)
		goto cleanup_mode_config;

	err = component_bind_all(mdev->dev, kms);
	if (err)
		goto cleanup_mode_config;


	drm_mode_config_reset(drm);

	err = devm_request_irq(drm->dev, mdev->irq,
			       komeda_kms_irq_handler, IRQF_SHARED,
			       drm->driver->name, drm);
	if (err)
		goto free_component_binding;

	drm->irq_enabled = true;

	drm_kms_helper_poll_init(drm);

	err = drm_dev_register(drm, 0);
	if (err)
		goto free_interrupts;

	//drm_fbdev_generic_setup(drm, 32);
	komeda_fbdev_init(drm);

#ifdef CONFIG_KOMEDA_KMS_AGENT
	komeda_kms_agent_attach_drm_dev(drm);
#endif

	komeda_display_thread_create(kms, mdev);

	return kms;

free_interrupts:
	drm_kms_helper_poll_fini(drm);
	drm->irq_enabled = false;
free_component_binding:
	component_unbind_all(mdev->dev, drm);
cleanup_mode_config:
	drm_mode_config_cleanup(drm);
	komeda_kms_cleanup_private_objs(kms);
	drm->dev_private = NULL;
	return ERR_PTR(err);
}

void komeda_kms_detach(struct komeda_kms_dev *kms)
{
	struct drm_device *drm = &kms->base;
	struct komeda_dev *mdev = drm->dev_private;

	drm_dev_unregister(drm);
	drm_kms_helper_poll_fini(drm);
	drm_atomic_helper_shutdown(drm);
	drm->irq_enabled = false;
	component_unbind_all(mdev->dev, drm);
	drm_mode_config_cleanup(drm);
	komeda_kms_cleanup_private_objs(kms);
	drm->dev_private = NULL;
}

/* duplicated from kernel */
int drm_atomic_replace_property_blob_from_id(struct drm_device *dev,
					 struct drm_property_blob **blob,
					 uint64_t blob_id,
					 ssize_t expected_size,
					 ssize_t expected_elem_size,
					 bool *replaced)
{
	struct drm_property_blob *new_blob = NULL;

	if (blob_id != 0) {
			new_blob = drm_property_lookup_blob(dev, blob_id);
			if (new_blob == NULL)
					return -EINVAL;

			if (expected_size > 0 &&
				new_blob->length != expected_size) {
					drm_property_blob_put(new_blob);
					return -EINVAL;
			}
			if (expected_elem_size > 0 &&
				new_blob->length % expected_elem_size != 0) {
					drm_property_blob_put(new_blob);
					return -EINVAL;
			}
	}

	*replaced |= drm_property_replace_blob(blob, new_blob);
	drm_property_blob_put(new_blob);

	return 0;
}
