// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021-2022 Siengine Technology, Inc.
 *
 * Sienginie Camera Interface driver
 *
 * Author: Siengine Technology, Inc.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/bug.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/pm_runtime.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/of_graph.h>

#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include "se-cif-core.h"
#include "se-cif-hw.h"
#include "se-media-dev.h"
#include "se-nr-hw.h"
#include "dw-mipi-csi.h"

#if IS_ENABLED(CONFIG_SE_CIF_DEBUG)
extern unsigned int cif_perf_enable;
extern unsigned int jitter_limit;
u32 fct_per = 0;
u32 fct = 0;
u32 loss_count_tmp = 0;
#endif

static struct media_pad *se_cif_get_remote_source_pad(struct se_cif_dev *se_cif);
static int se_cif_cap_streamoff(struct file *file, void *priv, enum v4l2_buf_type type);

//add other ioctl cmd if neeeded
#define RGBIR422_IOCTL_CMD_OFFSET		40
#define VIDIOC_SE_NR_ENABLE _IOWR('V', BASE_VIDIOC_PRIVATE + 1, unsigned int)
#define VIDIOC_SE_NR_SET_PARAM _IOWR('V', BASE_VIDIOC_PRIVATE + 2, denoise3d_params)
#define VIDIOC_SE_NR_GET_PARAM _IOWR('V', BASE_VIDIOC_PRIVATE + 3, denoise3d_params)
#define VIDIOC_SE_NR_SET_CFA _IOWR('V', BASE_VIDIOC_PRIVATE + 4, unsigned int)
#define VIDIOC_SE_NR_UPDATE_STRENGTH _IOWR('V', BASE_VIDIOC_PRIVATE + 5, se_nr_data_t)
#define VIDIOC_SE_NR_UPDATE_DELTA _IOWR('V', BASE_VIDIOC_PRIVATE + 6, se_nr_data_t)
#define VIDIOC_SE_NR_UPDATE_MOTION _IOWR('V', BASE_VIDIOC_PRIVATE + 7, se_nr_data_t)
#define VIDIOC_SE_NR_UPDATE_DENOISE3D _IOWR('V', BASE_VIDIOC_PRIVATE + 8, denoise3d_update)
#define VIDIOC_SE_NR_GET_AVG _IOWR('V', BASE_VIDIOC_PRIVATE + 9, unsigned int)
#define VIDIOC_SE_NR_SET_RDMA_MODE _IOWR('V', BASE_VIDIOC_PRIVATE + 10, unsigned int)
#define VIDIOC_SE_NR_UPDATE_DENOISE3D_EDGE_H _IOWR('V', BASE_VIDIOC_PRIVATE + 11, se_nr_data_t)
#define VIDIOC_SE_NR_UPDATE_DENOISE3D_EDGE_V _IOWR('V', BASE_VIDIOC_PRIVATE + 12, se_nr_data_t)
#define VIDIOC_SE_NR_UPDATE_DENOISE3D_RANGE_S _IOWR('V', BASE_VIDIOC_PRIVATE + 13, se_nr_data_t)
#define VIDIOC_SE_NR_UPDATE_DENOISE3D_RANGE_T _IOWR('V', BASE_VIDIOC_PRIVATE + 14, se_nr_data_t)
#define VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_S_0 _IOWR('V', BASE_VIDIOC_PRIVATE + 15, se_nr_data_t)
#define VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_S_1 _IOWR('V', BASE_VIDIOC_PRIVATE + 16, se_nr_data_t)
#define VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_S_2 _IOWR('V', BASE_VIDIOC_PRIVATE + 17, se_nr_data_t)
#define VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_S_3 _IOWR('V', BASE_VIDIOC_PRIVATE + 18, se_nr_data_t)
#define VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_S_4 _IOWR('V', BASE_VIDIOC_PRIVATE + 19, se_nr_data_t)
#define VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_S_5 _IOWR('V', BASE_VIDIOC_PRIVATE + 20, se_nr_data_t)
#define VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_T_0 _IOWR('V', BASE_VIDIOC_PRIVATE + 21, se_nr_data_t)
#define VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_T_1 _IOWR('V', BASE_VIDIOC_PRIVATE + 22, se_nr_data_t)
#define VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_T_2 _IOWR('V', BASE_VIDIOC_PRIVATE + 23, se_nr_data_t)
#define VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_T_3 _IOWR('V', BASE_VIDIOC_PRIVATE + 24, se_nr_data_t)
#define VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_T_4 _IOWR('V', BASE_VIDIOC_PRIVATE + 25, se_nr_data_t)
#define VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_T_5 _IOWR('V', BASE_VIDIOC_PRIVATE + 26, se_nr_data_t)
#define VIDIOC_SE_NR_GET_DENOISE3D_STRENGTH_SHD _IOWR('V', BASE_VIDIOC_PRIVATE + 27, unsigned int)
#define VIDIOC_SE_NR_GET_DENOISE3D_EDGE_H_SHD _IOWR('V', BASE_VIDIOC_PRIVATE + 28, unsigned int)
#define VIDIOC_SE_NR_GET_DENOISE3D_EDGE_V_SHD _IOWR('V', BASE_VIDIOC_PRIVATE + 29, unsigned int)
#define VIDIOC_SE_NR_GET_RANGE_S_SHD _IOWR('V', BASE_VIDIOC_PRIVATE + 30, unsigned int)
#define VIDIOC_SE_NR_GET_RANGE_T_SHD _IOWR('V', BASE_VIDIOC_PRIVATE + 31, unsigned int)
#define VIDIOC_SE_NR_GET_MOTION_SHD _IOWR('V', BASE_VIDIOC_PRIVATE + 32, unsigned int)
#define VIDIOC_SE_NR_GET_DELTA_IVA_SHD _IOWR('V', BASE_VIDIOC_PRIVATE + 33, unsigned int)
#define VIDIOC_SE_NR_UPDATE_DUMMY_HBLANK _IOWR('V', BASE_VIDIOC_PRIVATE + 34, se_nr_data_t)
#define VIDIOC_SE_NR_GET_CTRL_SHD _IOWR('V', BASE_VIDIOC_PRIVATE + 35, unsigned int)

#define VIDIOC_SE_RGBIR422_ENABLE _IOWR('V', BASE_VIDIOC_PRIVATE + RGBIR422_IOCTL_CMD_OFFSET, unsigned int)
#define VIDIOC_SE_RGBIR422_SET_PARAM _IOWR('V', BASE_VIDIOC_PRIVATE + RGBIR422_IOCTL_CMD_OFFSET + 1, rgbir422_coef_param)
#define VIDIOC_SE_RGBIR422_GET_PARAM _IOWR('V', BASE_VIDIOC_PRIVATE + RGBIR422_IOCTL_CMD_OFFSET + 2, rgbir422_coef_param)

static int debug;
module_param(debug, int, 0644);

struct se_cif_fmt se_cif_formats[] = {
	{
		.name		= "RGB32",
		.fourcc		= V4L2_PIX_FMT_ARGB32,
		.depth		= 32,
		.color		= CIF_OUT_FMT_ARGB8888,
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_ARGB8888_1X32,
	}, {
		.name		= "YUYV-16",
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.depth		= 16,
		.color		= CIF_OUT_FMT_YUV422,
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_1X16,
	}, {
		.name		= "UYVY-16",
		.fourcc		= V4L2_PIX_FMT_UYVY,
		.depth		= 16,
		.color		= CIF_OUT_FMT_YUV422,
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code	= MEDIA_BUS_FMT_UYVY8_1X16,
	}, {
		.name		= "YVYU-16",
		.fourcc		= V4L2_PIX_FMT_YVYU,
		.depth		= 16,
		.color		= CIF_OUT_FMT_YUV422,
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code	= MEDIA_BUS_FMT_YVYU8_1X16,
	}, {
		.name		= "VYUY-16",
		.fourcc		= V4L2_PIX_FMT_VYUY,
		.depth		= 16,
		.color		= CIF_OUT_FMT_YUV422,
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code	= MEDIA_BUS_FMT_VYUY8_1X16,
	}, {
		.name		= "RAW12",
		.fourcc		= V4L2_PIX_FMT_SRGGB12P,
		.depth		= 12,
		.color		= CIF_OUT_FMT_RAW12,
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code	= MEDIA_BUS_FMT_SRGGB12_1X12,
	}, {
		.name		= "RAW8",
		.fourcc		= V4L2_PIX_FMT_SRGGB8,
		.depth		= 8,
		.color		= CIF_OUT_FMT_RAW8,
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code	= MEDIA_BUS_FMT_SRGGB8_1X8,
	}
};

struct se_cif_fmt *se_cif_get_format(unsigned int index)
{
	return &se_cif_formats[index];
}

/**
 * se_cif_find_format - lookup se_cif color format by fourcc or media bus format
 */
struct se_cif_fmt *se_cif_find_format(const u32 *pixelformat,
						const u32 *mbus_code, int index)
{
	struct se_cif_fmt *fmt, *def_fmt = NULL;
	unsigned int i;
	int id = 0;

	if (index >= (int)ARRAY_SIZE(se_cif_formats))
		return NULL;

	for (i = 0; i < ARRAY_SIZE(se_cif_formats); i++) {
		fmt = &se_cif_formats[i];
		if (pixelformat && fmt->fourcc == *pixelformat)
			return fmt;
		if (mbus_code && fmt->mbus_code == *mbus_code)
			return fmt;
		if (index == id)
			def_fmt = fmt;
		id++;
	}
	return def_fmt;
}

struct se_cif_fmt *se_cif_get_src_fmt(struct se_cif_dev *se_cif,
						struct v4l2_subdev_format *sd_fmt)
{
	u32 index = 0;

	/* two fmt RGB32 and YUV444 from pixellink */
	if (sd_fmt->format.code == MEDIA_BUS_FMT_ARGB8888_1X32)
		index = 0;
	else if (sd_fmt->format.code == MEDIA_BUS_FMT_YUYV8_1X16)
		index = 1;
	else if (sd_fmt->format.code == MEDIA_BUS_FMT_UYVY8_1X16)
		index = 2;
	else if (sd_fmt->format.code == MEDIA_BUS_FMT_YVYU8_1X16)
		index = 3;
	else if (sd_fmt->format.code == MEDIA_BUS_FMT_VYUY8_1X16)
		index = 4;
	else if (sd_fmt->format.code == MEDIA_BUS_FMT_SRGGB12_1X12)
		index = 5;
	else if (sd_fmt->format.code == MEDIA_BUS_FMT_SRGGB8_1X8)
		index = 6;
	else {
		dev_err(&se_cif->pdev->dev, "%s , format is not supported, \
							raw12 is used default\n", __func__);
		index = 2;
	}
	return &se_cif_formats[index];
}

/*
 * se_cif_pipeline_enable() - Enable streaming on a pipeline
 *
 */
static int se_cif_pipeline_enable(struct se_cif_dev *se_cif, bool enable)
{
	struct media_entity *entity = &se_cif->cif_cap.vdev.entity;
	struct media_device *mdev = entity->graph_obj.mdev;
	struct media_graph graph;
	struct v4l2_subdev *subdev;
	struct media_entity *last_entity = NULL;
	struct dw_csi *s_dw_csi = NULL;
	struct media_pad *source_pad = se_cif_get_remote_source_pad(se_cif);
	int ret = 0;

	dev_dbg(&se_cif->pdev->dev, "enter %s\n", __func__);

	mutex_lock(&mdev->graph_mutex);

	ret = media_graph_walk_init(&graph, entity->graph_obj.mdev);
	if (ret) {
		mutex_unlock(&mdev->graph_mutex);
		return ret;
	}

	media_graph_walk_start(&graph, entity);

	while ((entity = media_graph_walk_next(&graph))) {
		if (entity == NULL) {
			dev_dbg(&se_cif->pdev->dev, "%s ,entity is NULL\n", __func__);
			continue;
		}

		if (!is_media_entity_v4l2_subdev(entity)) {
			dev_dbg(&se_cif->pdev->dev,
					"%s ,entity is no v4l2, %s\n", __func__, entity->name);
			continue;
		}

#ifndef CONFIG_CAMERA_OV2775_MIPI
		dev_dbg(&se_cif->pdev->dev, "se_cif_pipeline_enable entity.name=%s\n",
			entity->name);
		if (entity->flags == MEDIA_ENT_F_CAM_SENSOR) {
			last_entity = entity;
			continue;
		}
#endif

		subdev = media_entity_to_v4l2_subdev(entity);
		if (subdev == NULL) {
			dev_dbg(&se_cif->pdev->dev,
					"%s ,%s,subdev is NULL\n", __func__, entity->name);
			continue;
		}

		if ((strncmp(entity->name, DW_COMPATIBLE_NAME,
				strlen(DW_COMPATIBLE_NAME)) == 0)) {
			s_dw_csi = sd_to_mipi_csi_dev(subdev);
			s_dw_csi->vchannel = source_pad->index + 1;
		}

		ret = v4l2_subdev_call(subdev, video, s_stream, enable);
		if (ret < 0 && ret != -ENOIOCTLCMD) {
			dev_err(&se_cif->pdev->dev,
					"%s ,subdev %s s_stream failed\n", __func__, subdev->name);
			break;
		}
	}

	if (last_entity != NULL) {
		dev_dbg(&se_cif->pdev->dev,
				"last call subdev last_entity.name=%s\n", last_entity->name);
		subdev = media_entity_to_v4l2_subdev(last_entity);
		if (subdev == NULL) {
			dev_err(&se_cif->pdev->dev,
					"%s ,%s,subdev is NULL\n", __func__, last_entity->name);
		}
		ret = v4l2_subdev_call(subdev, video, s_stream, enable);
		if (ret < 0 && ret != -ENOIOCTLCMD) {
			dev_err(&se_cif->pdev->dev,
					"%s ,subdev %s s_stream failed\n", __func__, subdev->name);
		}
	}

	mutex_unlock(&mdev->graph_mutex);
	media_graph_walk_cleanup(&graph);

	return ret;
}

static int se_cif_update_buf_paddr(struct se_cif_buffer *buf, int buf_id)
{
	struct frame_addr *paddr = &buf->paddr;
	struct vb2_buffer *vb2 = &buf->v4l2_buf.vb2_buf;
	int ret = 0;

	switch (buf_id) {
	case SE_CIF_BUFA:
		paddr->a = vb2_dma_contig_plane_dma_addr(vb2, 0) + vb2->planes[0].data_offset - CIF_BUFFER_HEADER_LEN;
		break;
	case SE_CIF_BUFB:
		paddr->b = vb2_dma_contig_plane_dma_addr(vb2, 1) + vb2->planes[0].data_offset - CIF_BUFFER_HEADER_LEN;
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

void se_cif_cap_frame_write_done(struct se_cif_dev *se_cif)
{
	struct se_cif_buffer *buf;
	struct vb2_buffer *vb2;
#if IS_ENABLED(CONFIG_SE_CIF_DEBUG)
	u64 interval_per;
	u64 interval;
#endif

	if (list_empty(&se_cif->cif_cap.out_active)) {
		dev_warn(&se_cif->pdev->dev,
				"%s trying to access empty active list\n", __func__);
		return;
	}

	//get filled buffer
	buf = list_first_entry(&se_cif->cif_cap.out_active,
				struct se_cif_buffer, list);

	if (buf->discard) {
		list_move_tail(se_cif->cif_cap.out_active.next,
					&se_cif->cif_cap.out_discard);
#if IS_ENABLED(CONFIG_SE_CIF_DEBUG)
		se_cif->cif_cap.loss_count++;
		dev_info(&se_cif->pdev->dev, "loss_count = %d\n",
					se_cif->cif_cap.loss_count);
#endif
	} else {
		vb2 = &buf->v4l2_buf.vb2_buf;
		list_del_init(&buf->list);
		buf->v4l2_buf.vb2_buf.timestamp = ktime_get_ns();
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_DONE);
	}

	se_cif->cif_cap.frame_count++;

	if (list_empty(&se_cif->cif_cap.out_pending)) {
		if (list_empty(&se_cif->cif_cap.out_discard)) {
			dev_warn(&se_cif->pdev->dev,
					"%s: trying to access empty discard list\n", __func__);
			return;
		}

		buf = list_first_entry(&se_cif->cif_cap.out_discard,
					struct se_cif_buffer, list);
		buf->v4l2_buf.sequence = se_cif->cif_cap.frame_count;
		se_cif_channel_set_outbuf(se_cif, buf);
		list_move_tail(se_cif->cif_cap.out_discard.next,
					&se_cif->cif_cap.out_active);
		return;
	}

	/* Get Queued Buffer and Write The Buffer Address to HW */
	buf = list_first_entry(&se_cif->cif_cap.out_pending,
					struct se_cif_buffer, list);

	buf->v4l2_buf.sequence = se_cif->cif_cap.frame_count;
	se_cif_channel_set_outbuf(se_cif, buf);
	vb2 = &buf->v4l2_buf.vb2_buf;
	vb2->state = VB2_BUF_STATE_ACTIVE;
	list_move_tail(se_cif->cif_cap.out_pending.next, &se_cif->cif_cap.out_active);
#if IS_ENABLED(CONFIG_SE_CIF_DEBUG)
	/* first frame record */
	if (2 == se_cif->cif_cap.frame_count) {
		se_cif->cif_cap.frame_start_ns = ktime_get_ns();
		dev_info(&se_cif->pdev->dev, "first frame time = %lld\n",
					se_cif->cif_cap.frame_start_ns);

		fct_per = se_cif->cif_cap.frame_count;
		fct = se_cif->cif_cap.frame_count;
		se_cif->cif_cap.frame_pre_ns = se_cif->cif_cap.frame_start_ns;
		se_cif->cif_cap.frame_per_s_ns = se_cif->cif_cap.frame_start_ns;
	}

	/* every frame record */
	if ((fct_per != 0) && (1 == se_cif->cif_cap.frame_count - fct_per)) {
		se_cif->cif_cap.frame_per_e_ns = ktime_get_ns();
		interval_per = (se_cif->cif_cap.frame_per_e_ns -
						se_cif->cif_cap.frame_per_s_ns) / 1000000;
		if (interval_per > (1000 / jitter_limit)) {
			se_cif->cif_cap.jitter_count++;
			dev_info(&se_cif->pdev->dev, "jitter_count = %d\n",
						se_cif->cif_cap.jitter_count);
		}
		fct_per = se_cif->cif_cap.frame_count;
		se_cif->cif_cap.frame_per_s_ns = se_cif->cif_cap.frame_per_e_ns;
	}

	/* every 30 frames record */
	if ((fct != 0) && (30 == se_cif->cif_cap.frame_count - fct)) {
		se_cif->cif_cap.frame_curr_ns = ktime_get_ns();
		interval = (se_cif->cif_cap.frame_curr_ns -
					se_cif->cif_cap.frame_pre_ns) / 1000000;
		se_cif->cif_cap.curr_hw_fps = 30 * 1000 / interval;

		se_cif->cif_cap.curr_ac_fps = (30 -
			(se_cif->cif_cap.loss_count - loss_count_tmp)) * 1000 / interval;

		loss_count_tmp = se_cif->cif_cap.loss_count;
		fct = se_cif->cif_cap.frame_count;
		se_cif->cif_cap.frame_pre_ns = se_cif->cif_cap.frame_curr_ns;
	}
#endif
}

static int cap_vb2_queue_setup(struct vb2_queue *q,
			unsigned int *num_buffers, unsigned int *num_planes,
			unsigned int sizes[], struct device *alloc_devs[])
{
	struct se_cif_dev *se_cif = q->drv_priv;
	struct se_cif_frame *dst_f = &se_cif->cif_cap.dst_f;
	struct se_cif_fmt *fmt = dst_f->fmt;
	unsigned long wh;
	int i;

	dev_dbg(&se_cif->pdev->dev,"enter %s\n", __func__);
	if (fmt == NULL)
		return -EINVAL;

	for (i = 0; i < fmt->memplanes; i++)
		alloc_devs[i] = &se_cif->pdev->dev;

	wh = dst_f->width * dst_f->height;

	*num_planes = fmt->memplanes;

	for (i = 0; i < fmt->memplanes; i++) {
		unsigned int size = (wh * fmt->depth) / 8;

		if (i == 1 && fmt->fourcc == V4L2_PIX_FMT_NV12)
			size >>= 1;
		sizes[i] = max_t(u32, size, dst_f->sizeimage);
	}

	dev_info(&se_cif->pdev->dev, "%s, buf_n=%d, size=%d\n",
					__func__,  *num_buffers, sizes[0]);

	return 0;
}

static int cap_vb2_buffer_prepare(struct vb2_buffer *vb2)
{
	struct vb2_queue *q = vb2->vb2_queue;
	struct se_cif_dev *se_cif = q->drv_priv;
	struct se_cif_frame *dst_f = &se_cif->cif_cap.dst_f;
	int i;

	if (se_cif->cif_cap.dst_f.fmt == NULL)
		return -EINVAL;

	for (i = 0; i < dst_f->fmt->memplanes; i++) {
		unsigned long size = dst_f->sizeimage;

		if (vb2_plane_size(vb2, i) < size) {
			v4l2_err(&se_cif->cif_cap.vdev,
				 "User buffer too small (%ld < %ld)\n",
				 vb2_plane_size(vb2, i), size);
			return -EINVAL;
		}
#if 0 //debug only
		if (vb2_plane_vaddr(vb2, i))
			memset((void *)vb2_plane_vaddr(vb2, i), 0xaa,
					vb2_get_plane_payload(vb2, i));
#endif
		if (i < vb2->num_planes) {
			vb2->planes[i].data_offset = CIF_DATA_OFFSET;
			vb2->planes[i].bytesused = size - CIF_DATA_OFFSET;
			vb2->planes[i].length = size;
		}
	}

	return 0;
}

static void cap_vb2_buffer_queue(struct vb2_buffer *vb2)
{
	struct vb2_v4l2_buffer *v4l2_buf;
	struct se_cif_buffer *buf;
	struct se_cif_dev *se_cif;
	unsigned long flags;

	v4l2_buf = to_vb2_v4l2_buffer(vb2);
	buf = container_of(v4l2_buf, struct se_cif_buffer, v4l2_buf);
	se_cif = vb2_get_drv_priv(vb2->vb2_queue);

	spin_lock_irqsave(&se_cif->slock, flags);

	se_cif_update_buf_paddr(buf, se_cif->cif_cap.dst_f.fmt->mdataplanes);
	list_add_tail(&buf->list, &se_cif->cif_cap.out_pending);

	spin_unlock_irqrestore(&se_cif->slock, flags);
#if IS_ENABLED(CONFIG_SE_CIF_DEBUG)
	if (0 == vb2->index) {
		se_cif->cif_cap.q_timestamp = ktime_get_ns();
		dev_info(&se_cif->pdev->dev, "q_timestamp = %lld\n",
					se_cif->cif_cap.q_timestamp);
	}
#endif
}

static void cap_vb2_buffer_finish(struct vb2_buffer *vb2)
{
	struct se_cif_dev *se_cif;

	se_cif = vb2_get_drv_priv(vb2->vb2_queue);
#if IS_ENABLED(CONFIG_SE_CIF_DEBUG)
	if (0 == vb2->index) {
		se_cif->cif_cap.dq_timestamp = ktime_get_ns();
		if (se_cif->cif_cap.dq_timestamp > se_cif->cif_cap.q_timestamp) {
			se_cif->cif_cap.interval_qdq =
			(se_cif->cif_cap.dq_timestamp - se_cif->cif_cap.q_timestamp) / 1000;
		}
	}
#endif
}

static int cap_vb2_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct se_cif_dev *se_cif = q->drv_priv;
	struct se_cif_buffer *buf;
	struct vb2_buffer *vb2;
	unsigned long flags;
	int i, j;

	dev_dbg(&se_cif->pdev->dev, "enter %s\n", __func__);

	/* Create a buffer for discard operation */
	for (i = 0; i < se_cif->pix.num_planes; i++) {
		se_cif->discard_size[i] = se_cif->cif_cap.dst_f.sizeimage;
		se_cif->discard_buffer[i] = dma_alloc_coherent(&se_cif->pdev->dev,
					PAGE_ALIGN(se_cif->discard_size[i]),
					&se_cif->discard_buffer_dma[i], GFP_KERNEL | __GFP_HIGHMEM );
		if (!se_cif->discard_buffer[i]) {
			for (j = 0; j < i; j++) {
				dma_free_coherent(&se_cif->pdev->dev,
							se_cif->discard_size[j],
							se_cif->discard_buffer[j],
							se_cif->discard_buffer_dma[j]);
				dev_err(&se_cif->pdev->dev, "%s: alloc dma buffer_%d fail\n",
							__func__, j);
			}
			return -ENOMEM;
		}
		dev_info(&se_cif->pdev->dev,
				"%s: num_plane=%d discard_size=%d discard_buffer=0x%llx\n"
				, __func__, i,
				(int)se_cif->discard_size[i],
				(uint64_t)se_cif->discard_buffer[i]);
	}

	spin_lock_irqsave(&se_cif->slock, flags);

	/* add two list member to out_discard list head */
	se_cif->buf_discard[0].discard = true;
	list_add_tail(&se_cif->buf_discard[0].list, &se_cif->cif_cap.out_discard);

	se_cif->buf_discard[1].discard = true;
	list_add_tail(&se_cif->buf_discard[1].list, &se_cif->cif_cap.out_discard);

	/* set hw ping-pong buffer address A */
	if (count >= 1)
		buf = list_first_entry(&se_cif->cif_cap.out_pending,
					struct se_cif_buffer, list);
	else
		buf = list_first_entry(&se_cif->cif_cap.out_discard,
					struct se_cif_buffer, list);
	buf->v4l2_buf.sequence = 0;
	buf->id = SE_CIF_BUFA;
	vb2 = &buf->v4l2_buf.vb2_buf;
	vb2->state = VB2_BUF_STATE_ACTIVE;
	se_cif->buf.id = buf->id;
	se_cif_channel_set_outbuf(se_cif, buf);	//set output addr_a
	if (count >= 1)
		list_move_tail(se_cif->cif_cap.out_pending.next, &se_cif->cif_cap.out_active);
	else
		list_move_tail(se_cif->cif_cap.out_discard.next, &se_cif->cif_cap.out_active);

	/* set hw ping-pong buffer address B */
	if (count >= 2)
		buf = list_first_entry(&se_cif->cif_cap.out_pending,
					struct se_cif_buffer, list);
	else
		buf = list_first_entry(&se_cif->cif_cap.out_discard,
					struct se_cif_buffer, list);
	buf->v4l2_buf.sequence = 1;
	buf->id = SE_CIF_BUFB;
	vb2 = &buf->v4l2_buf.vb2_buf;
	vb2->state = VB2_BUF_STATE_ACTIVE;
	se_cif->buf.id = buf->id;
	se_cif_channel_set_outbuf(se_cif, buf);	//set output addr_b
	if (count >= 2)
		list_move_tail(se_cif->cif_cap.out_pending.next, &se_cif->cif_cap.out_active);
	else
		list_move_tail(se_cif->cif_cap.out_discard.next, &se_cif->cif_cap.out_active);


	/* Clear frame count */
	se_cif->cif_cap.frame_count = 1;//TODO: how to link frame count and frame num from cif
	spin_unlock_irqrestore(&se_cif->slock, flags);

	return 0;
}

static void cap_vb2_stop_streaming(struct vb2_queue *q)
{
	struct se_cif_dev *se_cif = q->drv_priv;
	struct se_cif_buffer *buf, *tmp;
	unsigned long flags;
	int i;

	dev_dbg(&se_cif->pdev->dev, "%s\n", __func__);

	se_cif_channel_disable(se_cif);

	spin_lock_irqsave(&se_cif->slock, flags);

	while (!list_empty(&se_cif->cif_cap.out_active)) {
		buf = list_entry(se_cif->cif_cap.out_active.next, struct se_cif_buffer, list);

		list_del(&buf->list);
		if (buf->discard)
			continue;

		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	while (!list_empty(&se_cif->cif_cap.out_pending)) {
		buf = list_entry(se_cif->cif_cap.out_pending.next, struct se_cif_buffer, list);

		list_del(&buf->list);
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	while (!list_empty(&se_cif->cif_cap.out_discard)) {
		buf = list_entry(se_cif->cif_cap.out_discard.next, struct se_cif_buffer, list);
		list_del(&buf->list);
	}

	list_for_each_entry_safe(buf, tmp,
				&se_cif->cif_cap.out_active, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	list_for_each_entry_safe(buf, tmp,
				&se_cif->cif_cap.out_pending, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	INIT_LIST_HEAD(&se_cif->cif_cap.out_active);
	INIT_LIST_HEAD(&se_cif->cif_cap.out_pending);
	INIT_LIST_HEAD(&se_cif->cif_cap.out_discard);

	spin_unlock_irqrestore(&se_cif->slock, flags);

	for (i = 0; i < se_cif->pix.num_planes; i++)
		dma_free_coherent(&se_cif->pdev->dev,
					se_cif->discard_size[i],
					se_cif->discard_buffer[i],
					se_cif->discard_buffer_dma[i]);
}

static struct vb2_ops se_cap_vb2_qops = {
	.queue_setup		= cap_vb2_queue_setup,
	.buf_prepare		= cap_vb2_buffer_prepare,
	.buf_queue			= cap_vb2_buffer_queue,
	.buf_finish			= cap_vb2_buffer_finish,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.start_streaming	= cap_vb2_start_streaming,
	.stop_streaming		= cap_vb2_stop_streaming,
};

static int se_cif_s_ctrl(struct v4l2_ctrl *ctrl)
{
	return 0;
}

static const struct v4l2_ctrl_ops se_cif_ctrl_ops = {
	.s_ctrl = se_cif_s_ctrl,
};

int se_cif_ctrls_create(struct se_cif_dev *se_cif)
{
	return 0;
}

void se_cif_ctrls_delete(struct se_cif_dev *se_cif)
{
	return;
}

static struct media_pad *se_cif_get_remote_source_pad(struct se_cif_dev *se_cif)
{
	struct se_cif_cap_dev *cif_cap = &se_cif->cif_cap;
	struct v4l2_subdev *subdev = &cif_cap->sd;
	struct media_pad *sink_pad, *source_pad;
	int i;

	dev_dbg(&se_cif->pdev->dev, "CIF%d, num_pads = %d\n", se_cif->id, subdev->entity.num_pads);
	while (1) {
		source_pad = NULL;
		for (i = 0; i < subdev->entity.num_pads; i++) {
			sink_pad = &subdev->entity.pads[i];
			if (sink_pad->flags & MEDIA_PAD_FL_SINK) {
				source_pad = media_entity_remote_pad(sink_pad);
				if (source_pad)
					break;
			}
		}
		/* return first pad point in the loop  */
		return source_pad;
	}

	if (i == subdev->entity.num_pads)
		v4l2_err(se_cif->v4l2_dev, "%s, No remote pad found!\n", __func__);

	return NULL;
}

static int se_cif_capture_open(struct file *file)
{
	struct se_cif_dev *se_cif = video_drvdata(file);
	struct media_pad *source_pad;
	struct v4l2_subdev *sd;
	struct device *dev = &se_cif->pdev->dev;
	int ret = -EBUSY;

	dev_info(&se_cif->pdev->dev, "%s, CIF%d\n", __func__, se_cif->id);

	if (atomic_read(&se_cif->open_count) > 0 ) {
		dev_warn(&se_cif->pdev->dev, "%s, CIF%d has been opened, return\n",
			__func__, se_cif->id);
		return ret;
	}

	atomic_inc(&se_cif->open_count);
	//se_cif->is_m2m = 0;

	/* Get remote source pad */
	source_pad = se_cif_get_remote_source_pad(se_cif);
	if (source_pad == NULL) {
		v4l2_err(se_cif->v4l2_dev, "%s, No remote pad found!\n", __func__);
		goto fail;
	}

	v4l2_dbg(1, debug, se_cif->v4l2_dev, "%s, remote pad = %d\n", __func__, source_pad->index);
	/* Get remote source pad subdev */
	sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (sd == NULL) {
		v4l2_err(se_cif->v4l2_dev, "%s, No remote subdev found!\n", __func__);
		goto fail;
	}

	v4l2_dbg(1, debug, se_cif->v4l2_dev, "%s, remote subdev found!\n", __func__);
	mutex_lock(&se_cif->lock);
	ret = v4l2_fh_open(file);
	mutex_unlock(&se_cif->lock);

	if (ret == -ENOMEM) {
		v4l2_err(se_cif->v4l2_dev, "%s v4l2_fh_open fail\n", __func__);
		atomic_dec(&se_cif->open_count);
		return -EINVAL;
	}

	pm_runtime_get_sync(dev);

	ret = v4l2_subdev_call(sd, core, s_power, 1);
	if (ret) {
		v4l2_err(se_cif->v4l2_dev, "%s, Call subdev s_power fail!\n", __func__);
		pm_runtime_put(dev);
		goto fail;
	}

	v4l2_dbg(1, debug, se_cif->v4l2_dev, "%s, Call subdev s_power success!\n", __func__);

	return 0;

fail:
	atomic_dec(&se_cif->open_count);
	return -EINVAL;

	return 0;
}

static int se_cif_capture_release(struct file *file)
{
	struct se_cif_dev *se_cif = video_drvdata(file);
	struct media_pad *source_pad;
	struct v4l2_subdev *sd;
	struct device *dev = &se_cif->pdev->dev;
	se_nr_dev *nr_dev = &se_cif->cif_cap.nr_dev;
	int ret;

	dev_info(&se_cif->pdev->dev, "%s\n", __func__);

	if (atomic_read(&se_cif->open_count) == 0) {
		dev_warn(&se_cif->pdev->dev, "%s, CIF%d not been open, return\n",
			__func__, se_cif->id);
		return -ENODEV;
	}

	if (se_cif->is_streaming) {
		se_cif_cap_streamoff(file, NULL, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	}

	if (se_cif->cif_cap.se_nr_enable) {
		cisp_nr_denoise3d_enable(nr_dev, DISABLE);
		cisp_nr_enable(nr_dev, DISABLE);

		cisp_nr_internal_clk_enable(nr_dev, DISABLE);

		dma_free_coherent(&se_cif->pdev->dev, PAGE_ALIGN(nr_dev->out_size),
			nr_dev->virt_rf_rd_addr, nr_dev->rf_rd_addr);
		se_cif->cif_cap.se_nr_enable = 0;
	}

	/* Get remote source pad */
	source_pad = se_cif_get_remote_source_pad(se_cif);
	if (source_pad == NULL) {
		v4l2_err(se_cif->v4l2_dev, "%s, No remote pad found!\n", __func__);
		ret = -EINVAL;
		goto label;
	}

	/* Get remote source pad subdev */
	sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (sd == NULL) {
		v4l2_err(se_cif->v4l2_dev, "%s, No remote subdev found!\n", __func__);
		ret = -EINVAL;
		goto label;
	}

	mutex_lock(&se_cif->lock);
	ret = _vb2_fop_release(file, NULL);
	if (ret) {
		v4l2_err(se_cif->v4l2_dev, "%s fail\n", __func__);
		mutex_unlock(&se_cif->lock);
		goto label;
	}
	mutex_unlock(&se_cif->lock);

	if (atomic_read(&se_cif->open_count) > 0 &&
		atomic_dec_and_test(&se_cif->open_count))
		//se_cif_channel_deinit(se_cif);

	ret = v4l2_subdev_call(sd, core, s_power, 0);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		v4l2_err(se_cif->v4l2_dev, "%s s_power fail\n", __func__);
		goto label;
	}

label:
	pm_runtime_put(dev);
	return (ret) ? ret : 0;
}

static const struct v4l2_file_operations se_cif_capture_fops = {
	.owner		= THIS_MODULE,
	.open		= se_cif_capture_open,
	.release	= se_cif_capture_release,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vb2_fop_mmap,
};

/*
 * The video node ioctl operations
 */
static int se_cif_cap_querycap(struct file *file, void *priv,
					struct v4l2_capability *cap)
{
	struct se_cif_dev *se_cif = video_drvdata(file);

	dev_dbg(&se_cif->pdev->dev, "enter %s\n", __func__);

	if (se_cif == NULL) {
		v4l2_err(se_cif->v4l2_dev, "%s se_cif === NULL\n", __func__);
		return -ENXIO;
	}

	if (cap == NULL) {
		v4l2_err(se_cif->v4l2_dev, "%s cap == NULL\n", __func__);
		return -EINVAL;
	}

	strlcpy(cap->driver, CIF_DRIVER_NAME, sizeof(cap->driver));
	strlcpy(cap->card, "siengine cif", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:cif.%d", se_cif->id);
	cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE_MPLANE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int se_cif_cap_enum_fmt_mplane(struct file *file, void *priv,
					struct v4l2_fmtdesc *f)
{
	struct se_cif_dev *se_cif = video_drvdata(file);
	struct media_entity *entity = &se_cif->cif_cap.vdev.entity;
	struct media_device *mdev = entity->graph_obj.mdev;
	struct media_graph graph;
	struct v4l2_subdev *sd;
	struct v4l2_subdev_mbus_code_enum code;
	struct se_cif_fmt *fmt;
	int ret;

	dev_dbg(&se_cif->pdev->dev, "enter %s\n", __func__);

	if(f->index >= 1) {
		return -EINVAL;
	}

	mutex_lock(&mdev->graph_mutex);

	ret = media_graph_walk_init(&graph, entity->graph_obj.mdev);
	if (ret) {
		mutex_unlock(&mdev->graph_mutex);
		return ret;
	}

	media_graph_walk_start(&graph, entity);

	while ((entity = media_graph_walk_next(&graph))) {

		if (!is_media_entity_v4l2_subdev(entity)) {
			dev_dbg(&se_cif->pdev->dev,
					"%s ,entity is no v4l2, %s\n", __func__, entity->name);
			continue;
		}

		if (entity->flags == MEDIA_ENT_F_CAM_SENSOR) {
			break;
		}
	}

	mutex_unlock(&mdev->graph_mutex);

	/* Get remote source pad subdev */
	sd = media_entity_to_v4l2_subdev(entity);
	if (sd == NULL) {
		v4l2_err(se_cif->v4l2_dev, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	memset(&code, 0, sizeof(code));
	code.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(sd, pad, enum_mbus_code, NULL, &code);
	if (ret){
		v4l2_err(se_cif->v4l2_dev, "%s, Call subdev fialed!\n", __func__);
		return ret;
	}

	fmt = se_cif_find_format(NULL, &code.code, 0);
	if (!fmt) {
		v4l2_err(se_cif->v4l2_dev, "%s, se_cif_find_format erro!\n", __func__);
		return -EINVAL;
	} else {
		strncpy(f->description, fmt->name, sizeof(f->description) - 1);
		f->pixelformat = fmt->fourcc;
		return 0;
	}

	v4l2_err(se_cif->v4l2_dev, "%s, Subdev return code inval!\n", __func__);

	return -EINVAL;
}

static int se_cif_cap_g_fmt_mplane(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	struct se_cif_dev *se_cif = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct se_cif_frame *dst_f = &se_cif->cif_cap.dst_f;

	dev_dbg(&se_cif->pdev->dev, "enter %s\n", __func__);

	pix->width = dst_f->o_width;
	pix->height = dst_f->o_height;
	pix->field = V4L2_FIELD_ANY;
	pix->pixelformat = dst_f->fmt->fourcc;
	pix->colorspace = dst_f->fmt->colorspace;
	pix->num_planes = dst_f->fmt->memplanes;
	pix->plane_fmt[0].bytesperline = dst_f->bytesperline;
	pix->plane_fmt[0].sizeimage = dst_f->sizeimage;

	return 0;
}

static int se_cif_get_fmt_index(struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct se_cif_fmt *fmt;
	int i;

	for (i = 0; i < ARRAY_SIZE(se_cif_formats); i++) {
		fmt = &se_cif_formats[i];
		if (fmt->fourcc == pix->pixelformat) {
			return i;
		}
	}

	return -1;
}

static int se_cif_cap_try_fmt_mplane(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct se_cif_dev *se_cif = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	int index;

	dev_dbg(&se_cif->pdev->dev, "%s\n", __func__);

	if (pix->width <= 0 || pix->height <= 0) {
		v4l2_err(se_cif->v4l2_dev, "%s, width %d, height %d is not valid\n"
				, __func__, pix->width, pix->height);
		return -EINVAL;
	}

	index = se_cif_get_fmt_index(f);
	if ((index >= ARRAY_SIZE(se_cif_formats)) || (index < 0)) {
		v4l2_err(se_cif->v4l2_dev, "%s, format is not support!\n", __func__);
		return -EINVAL;
	}

	return 0;
}

/* Update input frame size and formate  */
static int se_cif_source_fmt_init(struct se_cif_dev *se_cif)
{
	struct se_cif_frame *src_f = &se_cif->cif_cap.src_f;
	struct se_cif_frame *dst_f = &se_cif->cif_cap.dst_f;
	struct v4l2_subdev_format src_fmt;
	struct media_pad *source_pad;
	struct v4l2_subdev *src_sd;
	int ret;

	/* Get remote source pad */
	source_pad = se_cif_get_remote_source_pad(se_cif);
	if (source_pad == NULL) {
		v4l2_err(se_cif->v4l2_dev, "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	/* Get remote source pad subdev */
	src_sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (src_sd == NULL) {
		v4l2_err(se_cif->v4l2_dev, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	v4l2_dbg(1, debug, se_cif->v4l2_dev, "v4l2_subdev_call set_fmt\n");

	//TODO: how to config source_pad->index?
	src_fmt.pad = source_pad->index;
	v4l2_dbg(1, debug, se_cif->v4l2_dev, "src_fmt.pad = %d\n", src_fmt.pad);
	src_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	src_fmt.format.width = dst_f->width;
	src_fmt.format.height = dst_f->height;

	src_fmt.format.code = src_f->fmt->mbus_code; //MEDIA_BUS_FMT_ARGB8888_1X32

	v4l2_dbg(1, debug, se_cif->v4l2_dev, "src_fmt.format.code = %d\n",
		src_fmt.format.code);
	v4l2_dbg(1, debug, se_cif->v4l2_dev, "src_f->fmt->mbus_code = %d\n",
		src_f->fmt->mbus_code);
	src_fmt.format.height = src_f->height;
	src_fmt.format.width  = src_f->width;
	src_fmt.format.quantization = V4L2_QUANTIZATION_FULL_RANGE;
	if(CIF_OUT_FMT_ARGB8888 == src_f->fmt->color){
		src_fmt.format.quantization = V4L2_QUANTIZATION_LIM_RANGE;
	}
	src_fmt.format.field = V4L2_FIELD_ANY;
	src_fmt.format.colorspace = V4L2_COLORSPACE_SRGB; // ??
	src_fmt.format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;// ??
	src_fmt.format.xfer_func = V4L2_XFER_FUNC_DEFAULT;// ??

	ret = v4l2_subdev_call(src_sd, pad, set_fmt, NULL, &src_fmt);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		v4l2_err(se_cif->v4l2_dev, "%s, set remote fmt fail!\n", __func__);
		return -EINVAL;
	}

	v4l2_dbg(1, debug, se_cif->v4l2_dev, "v4l2_subdev_call get_fmt\n");
	memset(&src_fmt, 0, sizeof(src_fmt));
	src_fmt.pad = source_pad->index;
	src_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(src_sd, pad, get_fmt, NULL, &src_fmt);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		v4l2_err(se_cif->v4l2_dev, "%s, get remote fmt fail!\n", __func__);
		return -EINVAL;
	}

	/* Pixel link master will transfer format to RGB32 or YUV32 */
	src_f->fmt = se_cif_get_src_fmt(se_cif, &src_fmt);
	v4l2_dbg(1, debug, se_cif->v4l2_dev, "format is: %s\n", src_f->fmt->name);

	dev_dbg(&se_cif->pdev->dev, "%s: src:(%d,%d), dst:(%d,%d)\n",
		__func__, src_f->width, src_f->height, dst_f->width, dst_f->height);
	set_frame_bounds(src_f, src_fmt.format.width, src_fmt.format.height);

	if (dst_f->width > src_f->width || dst_f->height > src_f->height) {
		dev_err(&se_cif->pdev->dev,
				"%s: src:(%d,%d), dst:(%d,%d) Not support upscale\n", __func__,
				src_f->width, src_f->height,
				dst_f->width, dst_f->height);
		return -EINVAL;
	}

	return 0;
}

static int se_cif_cap_s_fmt_mplane(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct se_cif_dev *se_cif = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct se_cif_frame *dst_f = &se_cif->cif_cap.dst_f;
	struct se_cif_frame *src_f = &se_cif->cif_cap.src_f;

	struct se_cif_fmt *fmt;
	int bpl;
	int i;

	/* Step1: Check format with output support format list.
	 * Step2: Update output frame information.
	 * Step3: Checkout the format whether is supported by remote subdev
	 *	 Step3.1: If Yes, call remote subdev set_fmt.
	 *	 Step3.2: If NO, call remote subdev get_fmt.
	 * Step4: Update input frame information.
	 * Step5: Update virtual channel configuration.
	 * */

	//dev_info(&se_cif->pdev->dev, "%s, fmt=%.4s\n", __func__,(char *)pix->pixelformat);
	dev_info(&se_cif->pdev->dev, "%s\n", __func__);
	if (vb2_is_busy(&se_cif->cif_cap.vb2_q)) {
		dev_err(&se_cif->pdev->dev, "vb2_is_busy.\n");
		return -EBUSY;
	}

	if (se_cif_cap_try_fmt_mplane(file, priv, f)) {
		dev_err(&se_cif->pdev->dev, "fmt is not support.\n");
		return -EINVAL;
	}

	fmt = &se_cif_formats[se_cif_get_fmt_index(f)];

	v4l2_dbg(1, debug, se_cif->v4l2_dev,"width, height: %d, %d\n", pix->width, pix->height);

	src_f->hw_width = pix->width;

	pix->num_planes = fmt->memplanes;

	for (i = 0; i < pix->num_planes; i++) {
		bpl = pix->plane_fmt[i].bytesperline;

		if ((bpl == 0) || (bpl / (fmt->depth >> 3)) < pix->width)
			pix->plane_fmt[i].bytesperline =
						(pix->width * fmt->depth) >> 3;

		/* Align the width because of 32 bytes Alignment Mechanism */
		if (!IS_ALIGNED(pix->plane_fmt[i].bytesperline, 32)) {
			pix->plane_fmt[i].bytesperline = ALIGN(pix->plane_fmt[i].bytesperline, 32);
			pix->width = pix->plane_fmt[i].bytesperline / (fmt->depth >> 3);
			dev_info(&se_cif->pdev->dev,
				"After Alignment: bytesperline = %d, width = %d, hw_width = %d\n",
				pix->plane_fmt[i].bytesperline, pix->width, src_f->hw_width);
		}

		if (pix->plane_fmt[i].sizeimage == 0) {

			if ((i == 1) && (pix->pixelformat == V4L2_PIX_FMT_NV12))
				pix->plane_fmt[i].sizeimage =
					(pix->width * (pix->height >> 1) * fmt->depth >> 3);
			else
				pix->plane_fmt[i].sizeimage =
					pix->plane_fmt[i].bytesperline * pix->height + CIF_DATA_OFFSET;
		}
	}

	/* update out put frame size and formate */
	dst_f->fmt = fmt;
	dst_f->height = pix->height;
	dst_f->width = pix->width;

	if (pix->num_planes > 1) {
		for (i = 0; i < pix->num_planes; i++) {
			dst_f->bytesperline = pix->plane_fmt[i].bytesperline;
			dst_f->sizeimage = pix->plane_fmt[i].sizeimage;
		}
	} else {
		dst_f->bytesperline = pix->plane_fmt[0].bytesperline;
		dst_f->sizeimage = pix->plane_fmt[0].sizeimage;
	}

	memcpy(&se_cif->pix, pix, sizeof(*pix));

	set_frame_bounds(dst_f, pix->width, pix->height);

	// add se_cif-> src_f
	src_f->fmt = fmt;
	src_f->height = pix->height;
	src_f->width  = pix->width;
	src_f->bytesperline = dst_f->bytesperline;
	src_f->sizeimage = dst_f->sizeimage;

	return 0;
}

static int se_cif_config_parm(struct se_cif_dev *se_cif)
{
	int ret;
	dev_dbg(&se_cif->pdev->dev, "enter %s\n", __func__);
	ret = se_cif_source_fmt_init(se_cif);
	if (ret < 0)
	{
		dev_info(&se_cif->pdev->dev, "se_cif_source_fmt_init failed\n");
		return -EINVAL;
	}
	//se_cif_channel_init(se_cif);
	se_cif_channel_config(se_cif);

	dev_dbg(&se_cif->pdev->dev, "exit %s\n", __func__);
	return 0;
}

static int se_cif_cap_streamon(struct file *file, void *priv,
			enum v4l2_buf_type type)
{
	struct se_cif_dev *se_cif = video_drvdata(file);
	int ret;
	se_nr_dev *nr_dev = NULL;
	dma_addr_t paddr;

	if (se_cif->is_streaming) {
		return 0;
	}
	dev_info(&se_cif->pdev->dev, "%s\n", __func__);
	se_cif_dm_config(se_cif);
	se_cif_clock_enable(se_cif);
	se_cif_channel_softreset(se_cif);
	ret = se_cif_config_parm(se_cif);
	if (ret < 0)
		return -EINVAL;

	if (se_cif->cif_cap.se_nr_enable) {
		dev_info(&se_cif->pdev->dev, "prepare to init nr");
		nr_dev = &se_cif->cif_cap.nr_dev;
		nr_dev->out_h_size = se_cif->cif_cap.dst_f.o_width;
		nr_dev->out_v_size = se_cif->cif_cap.dst_f.o_height;
		nr_dev->out_size =
			(((nr_dev->out_h_size * 12 + 127) / 128) << 4) * nr_dev->out_v_size;
		nr_dev->virt_rf_wr_addr =
			dma_alloc_coherent( &se_cif->pdev->dev, PAGE_ALIGN(nr_dev->out_size),
			&paddr, GFP_KERNEL | __GFP_HIGHMEM);
		if (nr_dev->virt_rf_wr_addr == NULL) {
			dev_info(&se_cif->pdev->dev, "%s: alloc dma buffer fail\n", __func__);
			return -1;
		}
		nr_dev->virt_rf_rd_addr = nr_dev->virt_rf_wr_addr;
		nr_dev->rf_wr_addr = (uint32_t)paddr;
		nr_dev->rf_rd_addr = nr_dev->rf_wr_addr;
		dev_info(&se_cif->pdev->dev, "alloc nr rf buffer, nr width %d, height %d, size %d",
			nr_dev->out_h_size, nr_dev->out_v_size, nr_dev->out_size);
		cisp_nr_init(&(se_cif->cif_cap.nr_dev));
		if (nr_dev->rdma_mode == CONTINUOUS_MODE) {
			cisp_nr_start_rdma(nr_dev);
		}
	}

	ret = vb2_ioctl_streamon(file, priv, type);

	se_cif_enable_irq(se_cif);
	se_cif_channel_start_working(se_cif);

	dev_dbg(&se_cif->pdev->dev, "dump_cif_regs\n");
	dump_cif_regs(se_cif);
	se_cif_pipeline_enable(se_cif, 1);	//call s_stream

	se_cif->is_streaming = 1;

	return ret;
}

static int se_cif_cap_streamoff(struct file *file, void *priv,
				enum v4l2_buf_type type)
{
	struct se_cif_dev *se_cif = video_drvdata(file);
	se_nr_dev *nr_dev = &(se_cif->cif_cap.nr_dev);
	int ret;

	if (!se_cif->is_streaming) {
		return 0;
	}

	dev_info(&se_cif->pdev->dev, "%s\n", __func__);
	if (se_cif->cif_cap.se_nr_enable) {
		cisp_nr_denoise3d_enable(nr_dev, DISABLE);
		cisp_nr_enable(nr_dev, DISABLE);
		cisp_nr_update_config(nr_dev);
		cisp_nr_internal_clk_enable(nr_dev, DISABLE);

		dma_free_coherent(&se_cif->pdev->dev, PAGE_ALIGN(nr_dev->out_size),
			nr_dev->virt_rf_rd_addr, nr_dev->rf_rd_addr);
		se_cif->cif_cap.se_nr_enable = 0;
	}

	se_cif_pipeline_enable(se_cif, 0);
	se_cif_channel_disable(se_cif);
	ret = vb2_ioctl_streamoff(file, priv, type);

	se_cif->cif_cap.se_rgbir422_enable = 0;

	se_cif->is_streaming = 0;

	return ret;
}

static int se_cif_cap_g_selection(struct file *file, void *fh,
				struct v4l2_selection *s)
{
	return 0;
}

static int se_cif_cap_s_selection(struct file *file, void *fh,
				struct v4l2_selection *s)
{
	return 0;
}

static int se_cif_cap_g_parm(struct file *file, void *fh,
			struct v4l2_streamparm *a)
{
	struct se_cif_dev *se_cif = video_drvdata(file);
	struct v4l2_subdev *sd;
	struct media_pad *source_pad;

	source_pad = se_cif_get_remote_source_pad(se_cif);
	if (source_pad == NULL) {
		v4l2_err(se_cif->v4l2_dev, "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	/* Get remote source pad subdev */
	sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (sd == NULL) {
		v4l2_err(se_cif->v4l2_dev, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	a->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	a->parm.capture.readbuffers = 1;

	//return v4l2_subdev_call(sd, video, g_parm, a);
	return 4;
}

static int se_cif_cap_s_parm(struct file *file, void *fh,
			struct v4l2_streamparm *a)
{
#if 0
	struct se_cif_dev *se_cif = video_drvdata(file);
	struct v4l2_subdev *sd;
	struct media_pad *source_pad;

	source_pad = se_cif_get_remote_source_pad(se_cif);
	if (source_pad == NULL) {
		v4l2_err(se_cif->v4l2_dev, "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	/* Get remote source pad subdev */
	sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (sd == NULL) {
		v4l2_err(se_cif->v4l2_dev, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}
	return v4l2_subdev_call(sd, video, s_parm, a);
#endif
	return 0;
}

static int se_cif_cap_enum_framesizes(struct file *file, void *priv,
					struct v4l2_frmsizeenum *fsize)
{
	struct se_cif_fmt *fmt;
    struct se_cif_dev *se_cif = video_drvdata(file);
	struct media_entity *entity = &se_cif->cif_cap.vdev.entity;
	struct media_device *mdev = entity->graph_obj.mdev;
	struct media_graph graph;
	struct v4l2_subdev *sd;
	struct v4l2_subdev_frame_size_enum fse = {
		.index = fsize->index,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	fmt = se_cif_find_format(&fsize->pixel_format, NULL, 0);
	if (!fmt || fmt->fourcc != fsize->pixel_format) {
		v4l2_err(se_cif->v4l2_dev, "%s, se_cif_find_format erro!\n", __func__);
		return -EINVAL;
	}

	fse.code = fmt->mbus_code;

	mutex_lock(&mdev->graph_mutex);

	ret = media_graph_walk_init(&graph, entity->graph_obj.mdev);
	if (ret) {
		mutex_unlock(&mdev->graph_mutex);
		return ret;
	}

	media_graph_walk_start(&graph, entity);

	while ((entity = media_graph_walk_next(&graph))) {

		if (!is_media_entity_v4l2_subdev(entity)) {
			dev_dbg(&se_cif->pdev->dev,
					"%s ,entity is no v4l2, %s\n", __func__, entity->name);
			continue;
		}

		if (entity->flags == MEDIA_ENT_F_CAM_SENSOR) {
			break;
		}
	}
	mutex_unlock(&mdev->graph_mutex);


	/* Get remote source pad subdev */
	sd = media_entity_to_v4l2_subdev(entity);
	if (sd == NULL) {
		v4l2_err(se_cif->v4l2_dev, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	ret = v4l2_subdev_call(sd, pad, enum_frame_size, NULL, &fse);
	if (ret) {
		return ret;
	}

	if (fse.min_width == fse.max_width &&
		fse.min_height == fse.max_height) {
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = fse.max_width;
		fsize->discrete.height = fse.max_height;
	} else {
		fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		fsize->stepwise.min_width   = fse.min_width;
		fsize->stepwise.min_height  = fse.min_height;
		fsize->stepwise.max_width   = fse.max_width;
		fsize->stepwise.max_height  = fse.max_height;
		fsize->stepwise.step_width  = 1;
		fsize->stepwise.step_height = 1;
	}
	return 0;
}

static int se_cif_cap_enum_frameintervals(struct file *file, void *fh,
					  struct v4l2_frmivalenum *interval)
{
	struct se_cif_dev *se_cif = video_drvdata(file);
	struct media_entity *entity = &se_cif->cif_cap.vdev.entity;
	struct media_device *mdev = entity->graph_obj.mdev;
	struct media_graph graph;
	struct v4l2_subdev *sd;
	struct se_cif_fmt *fmt;
	struct v4l2_subdev_frame_interval_enum fie = {
		.index = interval->index,
		.width = interval->width,
		.height = interval->height,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	/* regarding width & height - we support any within range */
	if (interval->width < WIDTH_MIN || interval->width > WIDTH_MAX ||
		interval->height < HEIGHT_MIN || interval->height > HEIGHT_MAX) {
		v4l2_err(se_cif->v4l2_dev, "%s, resolution are erro!\n", __func__);
		return -EINVAL;
	}

	fmt = se_cif_find_format(&interval->pixel_format, NULL, 0);
	if (!fmt || fmt->fourcc != interval->pixel_format) {
		v4l2_err(se_cif->v4l2_dev, "%s, se_cif_find_format erro!\n", __func__);
		return -EINVAL;
	}
	fie.code = fmt->mbus_code;

	mutex_lock(&mdev->graph_mutex);

	ret = media_graph_walk_init(&graph, entity->graph_obj.mdev);
	if (ret) {
		mutex_unlock(&mdev->graph_mutex);
		return ret;
	}
	media_graph_walk_start(&graph, entity);

	while ((entity = media_graph_walk_next(&graph))) {

		if (!is_media_entity_v4l2_subdev(entity)) {
			dev_dbg(&se_cif->pdev->dev,
					"%s ,entity is no v4l2, %s\n", __func__, entity->name);
			continue;
		}

		if (entity->flags == MEDIA_ENT_F_CAM_SENSOR) {
			break;
		}
	}
	mutex_unlock(&mdev->graph_mutex);


	/* Get remote source pad subdev */
	sd = media_entity_to_v4l2_subdev(entity);
	if (sd == NULL) {
		v4l2_err(se_cif->v4l2_dev, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	ret = v4l2_subdev_call(sd, pad, enum_frame_interval, NULL, &fie);
	if (ret) {
		return ret;
	}

	interval->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	interval->discrete.denominator = fie.interval.denominator;
	interval->discrete.numerator   = fie.interval.numerator;

	return 0;
}

static const struct v4l2_ioctl_ops se_cif_capture_ioctl_ops = {
	.vidioc_querycap		= se_cif_cap_querycap,

	.vidioc_enum_fmt_vid_cap	= se_cif_cap_enum_fmt_mplane,
	.vidioc_try_fmt_vid_cap_mplane	= se_cif_cap_try_fmt_mplane,
	.vidioc_s_fmt_vid_cap_mplane	= se_cif_cap_s_fmt_mplane,
	.vidioc_g_fmt_vid_cap_mplane	= se_cif_cap_g_fmt_mplane,

	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,

	.vidioc_enum_framesizes = se_cif_cap_enum_framesizes,
	.vidioc_enum_frameintervals = se_cif_cap_enum_frameintervals,

	.vidioc_streamon		= se_cif_cap_streamon,
	.vidioc_streamoff		= se_cif_cap_streamoff,

	.vidioc_g_selection		= se_cif_cap_g_selection,
	.vidioc_s_selection		= se_cif_cap_s_selection,

	.vidioc_g_parm			= se_cif_cap_g_parm,
	.vidioc_s_parm			= se_cif_cap_s_parm,
};

/* Capture subdev media entity operations */
static int se_cif_link_setup(struct media_entity *entity,
				const struct media_pad *local,
				const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct se_cif_dev *se_cif = v4l2_get_subdevdata(sd);

	if (WARN_ON(se_cif == NULL))
		return 0;

	if (!(flags & MEDIA_LNK_FL_ENABLED)) {
		return 0;
	}
	/* TODO */
	/* Add CIF source and sink pad link configuration */
	if (local->flags & MEDIA_PAD_FL_SOURCE) {
		switch (local->index) {
		case CIF_SD_PAD_SOURCE_MEM:
			break;
		default:
			dev_err(&se_cif->pdev->dev, "%s invalid source pad\n", __func__);
			return -EINVAL;
		}
	} else if (local->flags & MEDIA_PAD_FL_SINK) {
		switch (local->index) {
		case CIF_SD_PAD_SINK_MIPI0_VC0:
		case CIF_SD_PAD_SINK_MIPI0_VC1:
		case CIF_SD_PAD_SINK_MIPI0_VC2:
		case CIF_SD_PAD_SINK_MIPI0_VC3:
			break;
		default:
			dev_err(&se_cif->pdev->dev, "%s invalid sink pad\n", __func__);
			return -EINVAL;
		}
	}

	return 0;
}

static const struct media_entity_operations se_cif_sd_media_ops = {
	.link_setup = se_cif_link_setup,
};

static int se_cif_subdev_enum_mbus_code(struct v4l2_subdev *sd,
					struct v4l2_subdev_pad_config *cfg,
					struct v4l2_subdev_mbus_code_enum *code)
{
	return 0;
}

static int se_cif_subdev_get_fmt(struct v4l2_subdev *sd,
					struct v4l2_subdev_pad_config *cfg,
					struct v4l2_subdev_format *fmt)
{
	struct se_cif_dev *se_cif = v4l2_get_subdevdata(sd);
	struct se_cif_frame *f;
	struct v4l2_mbus_framefmt *mf = &fmt->format;

	mutex_lock(&se_cif->lock);

	switch (fmt->pad) {
	case CIF_SD_PAD_SOURCE_MEM:
		f = &se_cif->cif_cap.dst_f;
		break;
	case CIF_SD_PAD_SINK_MIPI0_VC0:
	case CIF_SD_PAD_SINK_MIPI0_VC1:
	case CIF_SD_PAD_SINK_MIPI0_VC2:
	case CIF_SD_PAD_SINK_MIPI0_VC3:
		f = &se_cif->cif_cap.src_f;
		break;
	default:
		mutex_unlock(&se_cif->lock);
		v4l2_err(se_cif->v4l2_dev, "%s, Pad is not support now!\n", __func__);
		return -1;
	}

	if (!WARN_ON(f->fmt == NULL))
		mf->code = f->fmt->mbus_code;

	/* Source/Sink pads crop rectangle size */
	mf->width = f->width;
	mf->height = f->height;

	mutex_unlock(&se_cif->lock);
	mf->colorspace = V4L2_COLORSPACE_JPEG;

	return 0;
}

static int se_cif_subdev_set_fmt(struct v4l2_subdev *sd,
					struct v4l2_subdev_pad_config *cfg,
					struct v4l2_subdev_format *fmt)
{
	struct se_cif_dev *se_cif = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct se_cif_frame *dst_f = &se_cif->cif_cap.dst_f;
	struct se_cif_fmt *out_fmt;
	int i;

	if (fmt->pad < CIF_SD_PAD_SOURCE_MEM &&
				vb2_is_busy(&se_cif->cif_cap.vb2_q))
		return -EBUSY;

	for (i = 0; i < ARRAY_SIZE(se_cif_formats); i++) {
		out_fmt = &se_cif_formats[i];
		v4l2_dbg(1, debug, se_cif->v4l2_dev, "mf->code: 0x%x", mf->code);
		if (mf->code == out_fmt->mbus_code)
			break;
	}

	if (i >= ARRAY_SIZE(se_cif_formats)) {
		v4l2_err(se_cif->v4l2_dev, "%s, format is not support!\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&se_cif->lock);
	/* update out put frame size and formate */
	dst_f->fmt = &se_cif_formats[i];
	set_frame_bounds(dst_f, mf->width, mf->height);
	mutex_unlock(&se_cif->lock);

	v4l2_dbg(1, debug, se_cif->v4l2_dev, "pad%d: code: 0x%x, %dx%d",
		fmt->pad, mf->code, mf->width, mf->height);

	return 0;
}

static int se_cif_subdev_get_selection(struct v4l2_subdev *sd,
					struct v4l2_subdev_pad_config *cfg,
					struct v4l2_subdev_selection *sel)
{
#if 0
	struct se_cif_dev *se_cif = v4l2_get_subdevdata(sd);
	struct se_cif_frame *f = &se_cif->cif_cap.src_f;
	struct v4l2_rect *r = &sel->r;
	struct v4l2_rect *try_sel;

	mutex_lock(&se_cif->lock);

	switch (sel->target) {
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		f = &se_cif->cif_cap.dst_f;
	case V4L2_SEL_TGT_CROP_BOUNDS:
		r->width = f->o_width;
		r->height = f->o_height;
		r->left = 0;
		r->top = 0;
		mutex_unlock(&se_cif->lock);
		return 0;

	case V4L2_SEL_TGT_CROP:
		try_sel = v4l2_subdev_get_try_crop(sd, cfg, sel->pad);
		break;
	case V4L2_SEL_TGT_COMPOSE:
		try_sel = v4l2_subdev_get_try_compose(sd, cfg, sel->pad);
		f = &se_cif->cif_cap.dst_f;
		break;
	default:
		mutex_unlock(&se_cif->lock);
		return -EINVAL;
	}

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		sel->r = *try_sel;
	} else {
		r->left = f->h_off;
		r->top = f->v_off;
		r->width = f->width;
		r->height = f->height;
	}

	dev_dbg(&se_cif->pdev->dev, "%s, target %#x: l:%d, t:%d, %dx%d, f_w: %d, f_h: %d",
			__func__, sel->pad, r->left, r->top, r->width, r->height,
			f->c_width, f->c_height);

	mutex_unlock(&se_cif->lock);
#endif
	return 0;
}

static int se_cif_subdev_set_selection(struct v4l2_subdev *sd,
					struct v4l2_subdev_pad_config *cfg,
					struct v4l2_subdev_selection *sel)
{
#if 0
	struct se_cif_dev *se_cif = v4l2_get_subdevdata(sd);
	struct se_cif_frame *f = &se_cif->cif_cap.src_f;
	struct v4l2_rect *r = &sel->r;
	struct v4l2_rect *try_sel;
	unsigned long flags;

	mutex_lock(&se_cif->lock);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		try_sel = v4l2_subdev_get_try_crop(sd, cfg, sel->pad);
		break;
	case V4L2_SEL_TGT_COMPOSE:
		try_sel = v4l2_subdev_get_try_compose(sd, cfg, sel->pad);
		f = &se_cif->cif_cap.dst_f;
		break;
	default:
		mutex_unlock(&se_cif->lock);
		return -EINVAL;
	}

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		*try_sel = sel->r;
	} else {
		spin_lock_irqsave(&se_cif->slock, flags);
		set_frame_crop(f, r->left, r->top, r->width, r->height);
		spin_unlock_irqrestore(&se_cif->slock, flags);
	}

	dev_dbg(&se_cif->pdev->dev, "%s, target %#x: (%d,%d)/%dx%d", __func__,
			sel->target, r->left, r->top, r->width, r->height);

	mutex_unlock(&se_cif->lock);
#endif
	return 0;
}

static struct v4l2_subdev_pad_ops se_cif_subdev_pad_ops = {
	.enum_mbus_code = se_cif_subdev_enum_mbus_code,
	.get_selection = se_cif_subdev_get_selection,
	.set_selection = se_cif_subdev_set_selection,
	.get_fmt = se_cif_subdev_get_fmt,
	.set_fmt = se_cif_subdev_set_fmt,
};

static long se_cif_subdev_ioctl(struct v4l2_subdev *sd,
				unsigned int cmd, void *arg)
{
	struct se_cif_dev *se_cif = v4l2_get_subdevdata(sd);
	denoise3d_params *nr_params = NULL;
	unsigned int arg_val = 0;
	//int ret = -1;

	switch (cmd) {
	case VIDIOC_SE_NR_ENABLE:
		arg_val = *(u32 *)arg;
		dev_info(&se_cif->pdev->dev, "subdev ioctl, VIDIOC_SE_NR_ENABLE: %s",
			arg_val ? "enable" : "disable");
		se_cif->cif_cap.se_nr_enable = true;
		break;
	case VIDIOC_SE_NR_SET_PARAM:
		nr_params = (denoise3d_params *)arg;
		dev_info(&se_cif->pdev->dev, "subdev ioctl, VIDIOC_SE_NR_SET_PARAM");
		dev_info(&se_cif->pdev->dev, "params: strength: %d", nr_params->strength);
		cisp_3dnr_params_init(&(se_cif->cif_cap.nr_dev), nr_params);
		se_cif->cif_cap.nr_dev.base = se_cif->regs + 0x1000;
		break;
	case VIDIOC_SE_NR_GET_PARAM:
		dev_dbg(&se_cif->pdev->dev, "3dnr get param");
		cisp_nr_denoise3d_get_params(&(se_cif->cif_cap.nr_dev), (denoise3d_params *)arg);
		break;
	case VIDIOC_SE_NR_GET_AVG:
		dev_dbg(&se_cif->pdev->dev, "nr get avg: %u", *(u32 *)arg);
		cisp_nr_get_average(&(se_cif->cif_cap.nr_dev), (u32 *)arg);
		break;
	case VIDIOC_SE_NR_SET_RDMA_MODE:
		arg_val = *(u32 *)arg;
		dev_dbg(&se_cif->pdev->dev, "nr set rdma mode: %u", arg_val);
		cisp_nr_set_rdma_mode(&(se_cif->cif_cap.nr_dev), arg_val);
		break;
	case VIDIOC_SE_NR_SET_CFA:
		arg_val = *(u32 *)arg;
		dev_dbg(&se_cif->pdev->dev, "nr set CFA: %u", arg_val);
		cisp_nr_set_bayer_pattern(&(se_cif->cif_cap.nr_dev), arg_val);
		break;
	case VIDIOC_SE_NR_UPDATE_DELTA:
		dev_dbg(&se_cif->pdev->dev, "nr update delta");
		cisp_nr_update_delta(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_UPDATE_MOTION:
		dev_dbg(&se_cif->pdev->dev, "nr update motion");
		cisp_nr_update_motion(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_UPDATE_STRENGTH:
		dev_dbg(&se_cif->pdev->dev, "nr update strength");
		cisp_nr_update_strength(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_UPDATE_DENOISE3D:
		dev_dbg(&se_cif->pdev->dev, "nr update denoise3d");
		cisp_nr_update_denoise3d(&(se_cif->cif_cap.nr_dev), (denoise3d_update *)arg);
		break;
	case VIDIOC_SE_NR_UPDATE_DENOISE3D_EDGE_H:
		dev_dbg(&se_cif->pdev->dev, "nr update denoise3d edge_h");
		cisp_nr_update_denoise3d_edge_h(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_UPDATE_DENOISE3D_EDGE_V:
		dev_dbg(&se_cif->pdev->dev, "nr update denoise3d edge_v");
		cisp_nr_update_denoise3d_edge_v(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_UPDATE_DENOISE3D_RANGE_S:
		dev_dbg(&se_cif->pdev->dev, "nr update denoise3d range_s");
		cisp_nr_update_denoise3d_range_s(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_UPDATE_DENOISE3D_RANGE_T:
		dev_dbg(&se_cif->pdev->dev, "nr update denoise3d range_t");
		cisp_nr_update_denoise3d_range_t(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_S_0:
		dev_dbg(&se_cif->pdev->dev, "nr update denoise3d curve_s_0");
		cisp_nr_update_denoise3d_curve_s_0(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_S_1:
		dev_dbg(&se_cif->pdev->dev, "nr update denoise3d curve_s_1");
		cisp_nr_update_denoise3d_curve_s_1(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_S_2:
		dev_dbg(&se_cif->pdev->dev, "nr update denoise3d curve_s_2");
		cisp_nr_update_denoise3d_curve_s_2(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_S_3:
		dev_dbg(&se_cif->pdev->dev, "nr update denoise3d curve_s_3");
		cisp_nr_update_denoise3d_curve_s_3(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_S_4:
		dev_dbg(&se_cif->pdev->dev, "nr update denoise3d curve_s_4");
		cisp_nr_update_denoise3d_curve_s_4(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_S_5:
		dev_dbg(&se_cif->pdev->dev, "nr update denoise3d curve_s_5");
		cisp_nr_update_denoise3d_curve_s_5(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_T_0:
		dev_dbg(&se_cif->pdev->dev, "nr update denoise3d curve_t_0");
		cisp_nr_update_denoise3d_curve_t_0(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_T_1:
		dev_dbg(&se_cif->pdev->dev, "nr update denoise3d curve_t_1");
		cisp_nr_update_denoise3d_curve_t_1(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_T_2:
		dev_dbg(&se_cif->pdev->dev, "nr update denoise3d curve_t_2");
		cisp_nr_update_denoise3d_curve_t_2(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_T_3:
		dev_dbg(&se_cif->pdev->dev, "nr update denoise3d curve_t_3");
		cisp_nr_update_denoise3d_curve_t_3(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_T_4:
		dev_dbg(&se_cif->pdev->dev, "nr update denoise3d curve_t_4");
		cisp_nr_update_denoise3d_curve_t_4(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_UPDATE_DENOISE3D_CURVE_T_5:
		dev_dbg(&se_cif->pdev->dev, "nr update denoise3d curve_t_5");
		cisp_nr_update_denoise3d_curve_t_5(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_GET_DENOISE3D_STRENGTH_SHD:
		dev_dbg(&se_cif->pdev->dev, "nr get_denoise3d_strength_shd");
		cisp_nr_get_denoise3d_strength_shd(&(se_cif->cif_cap.nr_dev), (u32 *)arg);
		break;
	case VIDIOC_SE_NR_GET_DENOISE3D_EDGE_H_SHD:
		dev_dbg(&se_cif->pdev->dev, "nr get_denoise3d_edge_h_shd");
		cisp_nr_get_denoise3d_edge_h_shd(&(se_cif->cif_cap.nr_dev), (u32 *)arg);
		break;
	case VIDIOC_SE_NR_GET_DENOISE3D_EDGE_V_SHD:
		dev_dbg(&se_cif->pdev->dev, "nr get_denoise3d_edge_v_shd");
		cisp_nr_get_denoise3d_edge_v_shd(&(se_cif->cif_cap.nr_dev), (u32 *)arg);
		break;
	case VIDIOC_SE_NR_GET_RANGE_S_SHD:
		dev_dbg(&se_cif->pdev->dev, "nr get_range_s_shd");
		cisp_nr_get_range_s_shd(&(se_cif->cif_cap.nr_dev), (u32 *)arg);
		break;
	case VIDIOC_SE_NR_GET_RANGE_T_SHD:
		dev_dbg(&se_cif->pdev->dev, "nr get_range_t_shd");
		cisp_nr_get_range_t_shd(&(se_cif->cif_cap.nr_dev), (u32 *)arg);
		break;
	case VIDIOC_SE_NR_GET_MOTION_SHD:
		dev_dbg(&se_cif->pdev->dev, "nr get_motion_shd");
		cisp_nr_get_motion_shd(&(se_cif->cif_cap.nr_dev), (u32 *)arg);
		break;
	case VIDIOC_SE_NR_GET_DELTA_IVA_SHD:
		dev_dbg(&se_cif->pdev->dev, "nr get_delta_iva_shd");
		cisp_nr_get_delta_iva_shd(&(se_cif->cif_cap.nr_dev), (u32 *)arg);
		break;
	case VIDIOC_SE_NR_UPDATE_DUMMY_HBLANK:
		dev_dbg(&se_cif->pdev->dev, "nr update_dummy_hblank");
		cisp_nr_update_dummy_hblank(&(se_cif->cif_cap.nr_dev), arg);
		break;
	case VIDIOC_SE_NR_GET_CTRL_SHD:
		dev_dbg(&se_cif->pdev->dev, "nr get_ctrl_shd");
		cisp_nr_get_ctrl_shd(&(se_cif->cif_cap.nr_dev), (u32 *)arg);
		break;
	case VIDIOC_SE_RGBIR422_ENABLE:
		dev_dbg(&se_cif->pdev->dev, "rgbir422 enable");
		se_cif->cif_cap.se_rgbir422_enable = true;
		break;
	case VIDIOC_SE_RGBIR422_SET_PARAM:
		dev_dbg(&se_cif->pdev->dev, "rgbir422 set param");
		se_rgbir422_set_params(se_cif, (rgbir422_coef_param *)arg);
		break;
	case VIDIOC_SE_RGBIR422_GET_PARAM:
		dev_dbg(&se_cif->pdev->dev, "rgbir422 get param");
		se_rgbir422_get_params(se_cif, (rgbir422_coef_param *)arg);
		break;
	default:
		dev_info(&se_cif->pdev->dev, "%s error, no such ioctl cmd: 0x%x\n",
			__func__, cmd);
		break;
	}
	return 0;
}

static struct v4l2_subdev_core_ops se_cif_subdev_core_ops = {
	.ioctl = se_cif_subdev_ioctl,
};

static struct v4l2_subdev_ops se_cif_subdev_ops = {
	.pad = &se_cif_subdev_pad_ops,
	.core = &se_cif_subdev_core_ops,
};

static int se_cif_register_cap_device(struct se_cif_dev *se_cif,
				 struct v4l2_device *v4l2_dev)
{
	struct video_device *vdev = &se_cif->cif_cap.vdev;
	struct vb2_queue *q = &se_cif->cif_cap.vb2_q;
	struct se_cif_cap_dev *cif_cap = &se_cif->cif_cap;
	static char device_names[16];
	int cif_id = -1;
	int ret = -ENOMEM;

	dev_dbg(&se_cif->pdev->dev, "enter %s\n", __func__);
	memset(vdev, 0, sizeof(*vdev));
	snprintf(vdev->name, sizeof(vdev->name), "se_cif.%d.capture", se_cif->id);

	vdev->fops	= &se_cif_capture_fops;
	vdev->ioctl_ops	= &se_cif_capture_ioctl_ops;
	vdev->v4l2_dev	= v4l2_dev;
	vdev->minor	= -1;
	vdev->release	= video_device_release_empty;
	vdev->queue	= q;
	vdev->lock	= &se_cif->lock;
	vdev->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE_MPLANE;

	video_set_drvdata(vdev, se_cif);

	INIT_LIST_HEAD(&se_cif->cif_cap.out_pending);
	INIT_LIST_HEAD(&se_cif->cif_cap.out_active);
	INIT_LIST_HEAD(&se_cif->cif_cap.out_discard);

	memset(q, 0, sizeof(*q));
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	q->drv_priv = se_cif;
	q->ops = &se_cap_vb2_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct se_cif_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &se_cif->lock;

	ret = vb2_queue_init(q);
	if (ret)
		goto err_free_ctx;

	/* Default configuration  */
	cif_cap->dst_f.width = 1920;
	cif_cap->dst_f.height = 1080;
	cif_cap->dst_f.fmt = &se_cif_formats[0];
	cif_cap->src_f.fmt = cif_cap->dst_f.fmt;

	cif_cap->cap_pad.flags = MEDIA_PAD_FL_SINK;
	vdev->entity.function = MEDIA_ENT_F_PROC_VIDEO_SCALER;
	ret = media_entity_pads_init(&vdev->entity, 1, &cif_cap->cap_pad);
	if (ret)
		goto err_free_ctx;

	cif_id = se_cif->id;
	snprintf( device_names, sizeof( device_names ), "video%u%s", cif_id, "cif" );
	vdev->dev.init_name = device_names;
	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_err(v4l2_dev, "video register device failed, ret: %d\n", ret);
		goto err_me_cleanup;
	}

	v4l2_info(v4l2_dev, "Registered %s as /dev/%s\n",
			vdev->name, video_device_node_name(vdev));

	return 0;

err_me_cleanup:
	media_entity_cleanup(&vdev->entity);
err_free_ctx:
	return ret;
}

static int se_cif_subdev_registered(struct v4l2_subdev *sd)
{
	struct se_cif_dev *se_cif = v4l2_get_subdevdata(sd);
	int ret;

	if (se_cif == NULL)
		return -ENXIO;

	dev_dbg(&se_cif->pdev->dev, "%s\n", __func__);

	ret = se_cif_register_cap_device(se_cif, sd->v4l2_dev);
	if (ret < 0)
		return ret;

	return 0;
}

static void se_cif_subdev_unregistered(struct v4l2_subdev *sd)
{
	struct se_cif_dev *se_cif = v4l2_get_subdevdata(sd);
	struct video_device *vdev;

	if (se_cif == NULL)
		return;

	dev_dbg(&se_cif->pdev->dev, "%s\n", __func__);
	mutex_lock(&se_cif->lock);

	vdev = &se_cif->cif_cap.vdev;
	if (video_is_registered(vdev)) {
		video_unregister_device(vdev);
		media_entity_cleanup(&vdev->entity);
	}

	mutex_unlock(&se_cif->lock);
}

static const struct v4l2_subdev_internal_ops se_cif_capture_sd_internal_ops = {
	.registered = se_cif_subdev_registered,
	.unregistered = se_cif_subdev_unregistered,
};

int se_cif_initialize_capture_subdev(struct se_cif_dev *se_cif)
{
	struct v4l2_subdev *sd = &se_cif->cif_cap.sd;
	int ret, i = 0;

	v4l2_subdev_init(sd, &se_cif_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "se_cif.%d", se_cif->id);

	sd->entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;

	/* CIF Sink pads */
	for (i = 0; i < CIF_SD_PAD_SOURCE_MEM; i++)
		se_cif->cif_cap.sd_pads[i].flags = MEDIA_PAD_FL_SINK;

	/* CIF source pads */
	se_cif->cif_cap.sd_pads[CIF_SD_PAD_SOURCE_MEM].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, CIF_SD_PADS_NUM,
				se_cif->cif_cap.sd_pads);
	if (ret)
		return ret;

	sd->entity.ops = &se_cif_sd_media_ops;
	sd->internal_ops = &se_cif_capture_sd_internal_ops;
	v4l2_set_subdevdata(sd, se_cif);

	return 0;
}

void se_cif_unregister_capture_subdev(struct se_cif_dev *se_cif)
{
	struct v4l2_subdev *sd = &se_cif->cif_cap.sd;

	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_set_subdevdata(sd, NULL);
}
