#include <linux/pm_runtime.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-sg.h>
#include <media/videobuf2-v4l2.h>

#include "gdc_driver.h"
#include "system_log.h"

static int gdc_queue_setup(struct vb2_queue *vq,
                           unsigned int *num_buffers, unsigned int *num_planes,
                           unsigned int sizes[], struct device *alloc_devs[])

{
    struct gdc_ctx *ctx = vb2_get_drv_priv(vq);
    struct se_gdc *se_gdc = ctx->gdc;
    struct se_gdc_frame_para *f = gdc_get_format_data(ctx, vq->type);
    int i;
    unsigned int first_plane_size = 0;

    LOG(LOG_INFO, "enter %s\n", __func__);
    if (IS_ERR(f))
        return PTR_ERR(f);

    if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {//input data para
        *num_planes = f->fmt.mdataplanes;
        first_plane_size = f->height * f->width;
        switch(f->fmt.fourcc) {
        case V4L2_PIX_FMT_YUV420:
            sizes[0] = first_plane_size;
            sizes[1] = first_plane_size / 4;
            sizes[2] = sizes[1];
            break;
        case V4L2_PIX_FMT_NV12:
            sizes[0] = first_plane_size;
            sizes[1] = first_plane_size / 2;
            break;
        case V4L2_PIX_FMT_NV16:
            sizes[0] = first_plane_size;
            sizes[1] = sizes[0];
            break;
        default:
            LOG(LOG_ERR, "get wrong format!\n");
            return -1;
        }
        for (i = 0; i < f->fmt.mdataplanes; i++) {
            alloc_devs[i] = &se_gdc->pdev->dev;
        }
    } else {
        *num_planes = f->fmt.mdataplanes;
        first_plane_size = f->height * f->width;
        switch(f->fmt.fourcc) {
        case V4L2_PIX_FMT_YUV420:
            sizes[0] = first_plane_size;
            sizes[1] = first_plane_size / 4;
            sizes[2] = sizes[1];
            break;
        case V4L2_PIX_FMT_NV12:
        case V4L2_PIX_FMT_NV16:  //NV16 INPUT ---> NV12 OUTPUT
            sizes[0] = first_plane_size;
            sizes[1] = first_plane_size / 2;
            break;
        default:
            LOG(LOG_ERR, "get wrong format!\n");
            return -1;
        }
        for (i = 0; i < f->fmt.mdataplanes; i++) {
            alloc_devs[i] = &se_gdc->pdev->dev;
        }
    }
    LOG(LOG_INFO, "%s, buf_n=%d, size=%d\n",
        __func__,  *num_buffers, sizes[0]);

    return 0;
}

static int gdc_buf_prepare(struct vb2_buffer *vb)
{
    struct gdc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
    struct se_gdc_frame_para *f = gdc_get_format_data(ctx, vb->vb2_queue->type);
    unsigned int first_plane_size = 0;

    if (IS_ERR(f))
        return PTR_ERR(f);

    if (vb->vb2_queue->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {//input data para
        first_plane_size = f->height * f->width;
        switch(f->fmt.fourcc) {
        case V4L2_PIX_FMT_YUV420:
            vb2_set_plane_payload(vb, 0, first_plane_size);
            vb2_set_plane_payload(vb, 1, first_plane_size / 4);
            vb2_set_plane_payload(vb, 2, first_plane_size / 4);
            break;
        case V4L2_PIX_FMT_NV12:
            vb2_set_plane_payload(vb, 0, first_plane_size);
            vb2_set_plane_payload(vb, 1, first_plane_size / 2);
            break;
        case V4L2_PIX_FMT_NV16:
            vb2_set_plane_payload(vb, 0, first_plane_size);
            vb2_set_plane_payload(vb, 1, first_plane_size);
            break;
        default:
            LOG(LOG_ERR, "get wrong format!\n");
            return -1;
        }
    } else {//output data para
        first_plane_size = f->height * f->width;
        switch(f->fmt.fourcc) {
        case V4L2_PIX_FMT_YUV420:
            vb2_set_plane_payload(vb, 0, first_plane_size);
            vb2_set_plane_payload(vb, 1, first_plane_size / 4);
            vb2_set_plane_payload(vb, 2, first_plane_size / 4);
            break;
        case V4L2_PIX_FMT_NV12:
        case V4L2_PIX_FMT_NV16:
            vb2_set_plane_payload(vb, 0, first_plane_size);
            vb2_set_plane_payload(vb, 1, first_plane_size / 2);
            break;
        default:
            LOG(LOG_ERR, "get wrong format!\n");
            return -1;
        }
    }
    return 0;
}

static void gdc_buf_queue(struct vb2_buffer *vb)
{
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct gdc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

    v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int gdc_buf_start_streaming(struct vb2_queue *q, unsigned int count)
{
    return 0;
}

static void gdc_buf_stop_streaming(struct vb2_queue *q)
{
    struct gdc_ctx *ctx = vb2_get_drv_priv(q);
    struct vb2_v4l2_buffer *vbuf;

    for (;;) {
        if (V4L2_TYPE_IS_OUTPUT(q->type))
            vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
        else
            vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
        if (!vbuf)
            break;
        v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
    }
}

const struct vb2_ops gdc_qops = {
    .queue_setup = gdc_queue_setup,
    .buf_prepare = gdc_buf_prepare,
    .buf_queue = gdc_buf_queue,
    .wait_prepare = vb2_ops_wait_prepare,
    .wait_finish = vb2_ops_wait_finish,
    .start_streaming = gdc_buf_start_streaming,
    .stop_streaming = gdc_buf_stop_streaming,
};

