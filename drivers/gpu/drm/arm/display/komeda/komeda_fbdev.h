// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019 Siengine
 */

#ifndef _KOMEDA_FBDEV_H
#define _KOMEDA_FBDEV_H

#ifdef CONFIG_DRM_FBDEV_EMULATION
int komeda_fbdev_init(struct drm_device *dev);
void komeda_fbdev_fini(struct drm_fb_helper *helper);

struct komeda_mode_fb_cmd {
	__u32 width;
	__u32 height;
	__u32 pixel_format;
	__u32 flags;
	__u32 pitches[4];
	__u32 offsets[4];
	size_t size;
	int fd;
};

struct komeda_fb_crop {
	__u32 crtc_x;
	__u32 crtc_y;
	__u32 crtc_w;
	__u32 crtc_h;
	__u32 src_x;
	__u32 src_y;
	__u32 src_w;
	__u32 src_h;
};

#else
static inline int komeda_fbdev_init(struct drm_device *dev)
{
	return 0;
}

static inline void komeda_fbdev_fini(struct drm_fb_helper *helper)
{
}
#endif

#endif /* _KOMEDA_FBDEV_H */
