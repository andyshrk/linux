/*
 * Copyright (C) Siengine Tech Co.Ltd
 *
 * This software is licensed under the terms of the GNU General Public
 * License, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/timer.h>

#include <linux/platform_device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-sg.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#include "gdc_driver.h"
#include "system_log.h"
#include "gdc_hw.h"

static const unsigned char *fw_id = "se_fw_99a";

unsigned int gdc_log = LOG_INFO;
module_param(gdc_log, uint, 0644);

static int gdc_suspend(struct platform_device *pdev, pm_message_t message);
static int gdc_resume(struct platform_device *pdev);


static const struct se_gdc_fmt se_gdc_formats[] = {
    {
        .name		 = "YUV420P",
        .fourcc		 = V4L2_PIX_FMT_YUV420,
        .depth		 = 12,
        .memplanes	 = 1,
        .mdataplanes = 3,
        .flags       = 0,
        .index       = gdc_yuv420_planar,
    }, {
        .name		 = "NV12",
        .fourcc		 = V4L2_PIX_FMT_NV12,
        .depth		 = 12,
        .memplanes	 = 1,
        .mdataplanes = 2,
        .flags       = 0,
        .index       = gdc_yuv420sp_nv12,
    }, {
        .name		 = "NV16",
        .fourcc		 = V4L2_PIX_FMT_NV16,
        .depth		 = 16,
        .memplanes	 = 1,
        .mdataplanes = 2,
        .flags       = 1,
        .index       = gdc_yuv422sp_nv16,
    }
};

static const struct of_device_id se_gdc_of_match[] = {
    {.compatible = GDC_DRIVER_NAME, },
    { /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, se_gdc_of_match);

static void enable_gdc_manager(struct se_gdc *se_gdc)
{
    writel(1, se_gdc->dm_regs);
    LOG(LOG_INFO, "Enable gdm%d!\n", se_gdc->id);
}

static void disable_gdc_manager(struct se_gdc *se_gdc)
{
    writel(0, se_gdc->dm_regs);
    LOG(LOG_INFO, "Disable gdm%d!\n", se_gdc->id);
}

static void process_bufs(struct se_gdc *se_gdc, int vb_state)
{
    struct vb2_v4l2_buffer *src, *dst;
    struct gdc_ctx *ctx = se_gdc->p_gdc_ctx;

    if(ctx != NULL) {
        src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
        dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
        WARN_ON(!src);
        WARN_ON(!dst);

        dst->timecode = src->timecode;
        dst->vb2_buf.timestamp = ktime_get_ns();
        dst->flags &= ~V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
        dst->flags |= src->flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK;

        spin_lock(&se_gdc->ctrl_lock);
        v4l2_m2m_buf_done(src, vb_state);
        v4l2_m2m_buf_done(dst, vb_state);
        spin_unlock(&se_gdc->ctrl_lock);
    } else {
        LOG(LOG_EMERG, "enter process_bufs ctx is NULL !\n");
    }
    //wake_up(&se_gdc->irq_queue);

    //se_gdc->p_gdc_ctx = NULL;
    gdc_stop(&se_gdc->gdc_settings);
    v4l2_m2m_job_finish(se_gdc->m2m_dev, ctx->fh.m2m_ctx);
    LOG(LOG_INFO, "exit se_gdc_cap_frame_write_done!\n");
}

static int gdc_clk_enable(struct se_gdc *se_gdc, bool enable)
{
    unsigned long flags;

    if (enable) {
        spin_lock_irqsave(&se_gdc->ctrl_lock, flags);
        if (clk_prepare_enable(se_gdc->core_clk)||
        clk_prepare_enable(se_gdc->axi_clk)) {
            se_gdc->clk_state = false;
            LOG(LOG_ERR, "Failed to enabel gdc%d clock resource\n", se_gdc->id);
            spin_unlock_irqrestore(&se_gdc->ctrl_lock, flags);
            return -1;
        }
        se_gdc->clk_state = true;
        spin_unlock_irqrestore(&se_gdc->ctrl_lock, flags);
    } else {
        spin_lock_irqsave(&se_gdc->ctrl_lock, flags);
        clk_disable_unprepare(se_gdc->core_clk);
        clk_disable_unprepare(se_gdc->axi_clk);
        se_gdc->clk_state = false;
        spin_unlock_irqrestore(&se_gdc->ctrl_lock, flags);
    }

    return 0;
}

static void gdc_translate_timeout(struct timer_list *t)
{
    struct se_gdc *gdc_dev = from_timer(gdc_dev, t, irq_timer);

    LOG(LOG_ERR, "gdc_translate_timeout!\n");
    //del_timer(&gdc_dev->irq_timer);
    process_bufs(gdc_dev, VB2_BUF_STATE_ERROR);
}

static void job_abort(void *prv)
{
    struct gdc_ctx *ctx = prv;
    struct se_gdc *se_gdc = ctx->gdc;

    if (!se_gdc->p_gdc_ctx)	/* No job currently running */
        return;

    process_bufs(se_gdc, VB2_BUF_STATE_ERROR);
    //wait_event_timeout(se_gdc->irq_queue,
    //                   !se_gdc->p_gdc_ctx, msecs_to_jiffies(GDC_ABORT_TIMEOUT));
}

/*
* return 0 -- unready
* return 1 -- ready
*/
static int job_ready(void *prv)
{
    struct gdc_ctx *ctx = prv;
    struct se_gdc *se_gdc = ctx->gdc;

    if (!se_gdc->p_gdc_ctx) {/* No job currently running */
        LOG(LOG_ERR, "ctx is NULL!\n");
        return 0;
    }

    if (se_gdc->gdc_settings.is_waiting_gdc) {
        LOG(LOG_ERR, "gdc is processing!\n");
        return 0;
    }

    return 1;
}

static void updata_buffer_address(struct se_gdc *se_gdc, struct vb2_buffer *src, struct vb2_buffer *dst, struct se_gdc_frame_para *src_fram)
{
    se_gdc->gdc_settings.input_address_pa[0] = vb2_dma_contig_plane_dma_addr(src, 0);
    se_gdc->gdc_settings.output_address_pa[0] = vb2_dma_contig_plane_dma_addr(dst, 0);

    switch(src_fram->fmt.fourcc) {
    case V4L2_PIX_FMT_YUV420: //3p
        se_gdc->gdc_settings.output_address_pa[1] = vb2_dma_contig_plane_dma_addr(dst, 1);
        se_gdc->gdc_settings.output_address_pa[2] = vb2_dma_contig_plane_dma_addr(dst, 2);
        se_gdc->gdc_settings.input_address_pa[1] = vb2_dma_contig_plane_dma_addr(src, 1);
        se_gdc->gdc_settings.input_address_pa[2] = vb2_dma_contig_plane_dma_addr(src, 2);
        break;
    case V4L2_PIX_FMT_NV12: //2p
    case V4L2_PIX_FMT_NV16: //2p
        se_gdc->gdc_settings.output_address_pa[1] = vb2_dma_contig_plane_dma_addr(dst, 1);
        se_gdc->gdc_settings.input_address_pa[1] = vb2_dma_contig_plane_dma_addr(src, 1);
        break;
    default:
        LOG(LOG_ERR, "can get avalibel format paramters for vb2 bufs!\n");
        break;
    }
}

static void schedule_irq_timer(struct se_gdc *dev, int msec_timeout)
{
    LOG(LOG_DEBUG, "schedule_irq_timer enter\n");
    mod_timer(&dev->irq_timer, jiffies + msecs_to_jiffies(msec_timeout));
}

static void device_run(void *prv)
{
    struct gdc_ctx *ctx = prv;
    struct se_gdc_frame_para *src_fram = &ctx->in;
    struct se_gdc *se_gdc = ctx->gdc;
    struct vb2_v4l2_buffer *srct, *dstt;
    struct vb2_buffer *src, *dst;
    unsigned long flags;

    spin_lock_irqsave(&se_gdc->ctrl_lock, flags);
    se_gdc->p_gdc_ctx = ctx;
    srct = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
    dstt = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
    src = &(srct->vb2_buf);
    dst = &(dstt->vb2_buf);
    srct = NULL;
    dstt = NULL;
    spin_unlock_irqrestore(&se_gdc->ctrl_lock, flags);

    updata_buffer_address(se_gdc, src, dst, src_fram);
    if(updata_buffer_to_gdc_hw(se_gdc) != 0) {
        LOG(LOG_CRIT, "gdc run failed!");
        gdc_translate_timeout(&se_gdc->irq_timer);
        return;
    }
    schedule_irq_timer(se_gdc, GDC_IRQ_TIMER_TIMEOUT_MS);
    gdc_start(&se_gdc->gdc_settings);
    LOG(LOG_INFO, "device_run done\n");
}

static void se_gdc_cap_frame_write_done(struct se_gdc *se_gdc)
{
    u32 irq_status = 0;
    struct gdc_ctx *ctx = se_gdc->p_gdc_ctx;
    int vb_state = VB2_BUF_STATE_DONE;

    LOG(LOG_INFO, "enter se_gdc_cap_frame_write_done !\n");
    del_timer(&se_gdc->irq_timer);

    irq_status = gdc_get_status(&se_gdc->gdc_settings);
    LOG(LOG_INFO, "IRQ STATUS =0x%x\n", irq_status);
    if(irq_status & INT_REG_ERROR_MASK) {
        LOG(LOG_ERR, "IRQ STATUS: last operation was finished with error\n");
        if(irq_status & INT_REG_CONF_ERROR_MASK) {
            LOG(LOG_ERR, "IRQ STATUS: Configuration error(wrong configration stream).\n");
        }
        if(irq_status & INT_REG_USER_ABORT_MASK) {
            LOG(LOG_ERR, "IRQ STATUS: User abort(stop,reset command).\n");
        }
        if(irq_status & INT_REG_AXI_READER_ERROR_MASK) {
            LOG(LOG_ERR, "IRQ STATUS: AXI reader erro(e.g. error code returned by fabiric).\n");
        }
        if(irq_status & INT_REG_AXI_WRITER_ERROR_MASK) {
            LOG(LOG_ERR, "IRQ STATUS: AXI writer error.\n");
        }
        if(irq_status & INT_REG_UNALIGNED_ACCESSS_ERROR_MASK) {
            LOG(LOG_ERR, "IRQ STATUS: Unaligned access(address pointer is no aligned).\n");
        }
        if(irq_status & INT_REG_INCOMPATIBLE_CONFIG_ERROR_MASK) {
            LOG(LOG_ERR, "IRQ STATUS: Incompatible configuration(request of unimplemented mode, eg. unsuppoted format, mode stream configuration).\n");
        }

        vb_state = VB2_BUF_STATE_ERROR;
    } else if(irq_status & INT_REG_BUSY_MASK) {
        LOG(LOG_ERR, "IRQ STATUS: processing is busy now.\n");
        vb_state = VB2_BUF_STATE_ERROR;
    }
    LOG(LOG_INFO, "IRQ STATUS: ready for next image.\n");

    if (gdc_get_frame(&se_gdc->gdc_settings, se_gdc->gdc_settings.gdc_config.total_planes) != 0) {
        vb_state = VB2_BUF_STATE_ERROR;
    }

    WARN_ON(!ctx);

    process_bufs(se_gdc, vb_state);
}

static irqreturn_t se_gdc_irq_handler(int irq, void *priv)
{
    struct se_gdc *se_gdc = (struct se_gdc *)priv;

    LOG(LOG_INFO, "gdc_irq_handler irq num =%d\n", irq);
    se_gdc_cap_frame_write_done(se_gdc);
    return IRQ_HANDLED;
}


static struct v4l2_m2m_ops gdc_m2m_ops = {
    .device_run = device_run,
    .job_abort = job_abort,
    .job_ready = job_ready,
};

static int queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
    struct gdc_ctx *ctx = priv;
    int ret;

    src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    src_vq->io_modes = VB2_USERPTR | VB2_MMAP | VB2_DMABUF;
    src_vq->drv_priv = ctx;
    src_vq->ops = &gdc_qops;
    src_vq->mem_ops = &vb2_dma_contig_memops;
    src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
    src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
    src_vq->lock = &ctx->gdc->mutex;

    ret = vb2_queue_init(src_vq);
    if (ret)
        return ret;

    dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    dst_vq->io_modes = VB2_USERPTR | VB2_MMAP | VB2_DMABUF;
    dst_vq->drv_priv = ctx;
    dst_vq->ops = &gdc_qops;
    dst_vq->mem_ops = &vb2_dma_contig_memops;
    dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
    dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
    dst_vq->lock = &ctx->gdc->mutex;

    return vb2_queue_init(dst_vq);
}

static int gdc_fmt_find_index(struct v4l2_format *f)
{
    int i;

    LOG(LOG_INFO, "gdc_fmt_find_index f->fmt.pix.pixelformat = 0x%06x\n", f->fmt.pix.pixelformat);
    for (i = 0; i < (int)ARRAY_SIZE(se_gdc_formats); i++) {
        if (se_gdc_formats[i].fourcc == f->fmt.pix_mp.pixelformat)
            return i;
    }
    return -1;
}

struct se_gdc_frame_para *gdc_get_format_data(struct gdc_ctx *ctx, enum v4l2_buf_type type)
{
    switch (type) {
    case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
        return &ctx->out;
    case  V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
        return &ctx->in;
    default:
        return ERR_PTR(-EINVAL);
    }
}

static int gdc_open(struct file *file)
{
    struct se_gdc *se_gdc = video_drvdata(file);
    struct gdc_ctx *ctx = NULL;
    int ret = 0;

    if (atomic_read(&se_gdc->open_count) > 0 ) {
        LOG(LOG_ERR, "gdc has been opened, return\n");
        return -EBUSY;
    }

    atomic_inc(&se_gdc->open_count);
    ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
    if (!ctx) {
        atomic_dec(&se_gdc->open_count);
        return -ENOMEM;
    }

    se_gdc->p_gdc_ctx = ctx;
    ctx->gdc = se_gdc;

    if (mutex_lock_interruptible(&se_gdc->mutex)) {
        return -ERESTARTSYS;
    }

    ctx->gdc = se_gdc;
    ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(se_gdc->m2m_dev, ctx, &queue_init);
    if (IS_ERR(ctx->fh.m2m_ctx)) {
        ret = PTR_ERR(ctx->fh.m2m_ctx);
        mutex_unlock(&se_gdc->mutex);
        atomic_dec(&se_gdc->open_count);
        return ret;
    }

    v4l2_fh_init(&ctx->fh, video_devdata(file));
    file->private_data = &ctx->fh;
    v4l2_fh_add(&ctx->fh);

    mutex_unlock(&se_gdc->mutex);
    enable_gdc_manager(se_gdc);
    gdc_stop(&se_gdc->gdc_settings);

    return 0;
}

static int gdc_release(struct file *file)
{
    struct gdc_ctx *ctx =
        container_of(file->private_data, struct gdc_ctx, fh);
    struct se_gdc *se_gdc = ctx->gdc;
    if (atomic_read(&se_gdc->open_count) == 0) {
        LOG(LOG_ERR, "gdc has not been opened, return\n");
        return -ENODEV;
    }

    if (atomic_read(&se_gdc->open_count) > 0 && atomic_dec_and_test(&se_gdc->open_count))
        LOG(LOG_DEBUG, "device last release\n");

    disable_gdc_manager(se_gdc);
    gdc_stop(&se_gdc->gdc_settings);
    gdc_fw_exit(se_gdc);

    mutex_lock(&se_gdc->mutex);
    v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
    v4l2_fh_del(&ctx->fh);
    v4l2_fh_exit(&ctx->fh);
    mutex_unlock(&se_gdc->mutex);
    kfree(ctx);

    return 0;
}

static struct se_gdc *misc_file_to_se_gdc(struct file *file)
{
    return container_of(file->private_data, struct se_gdc, miscdev);
}

static ssize_t gdc_fw_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    int rc = 0;
    u8 *fw_data = NULL;

    struct se_gdc *se_gdc = misc_file_to_se_gdc(file);

    LOG(LOG_INFO, "v4l2_dev.name = %s!\n", se_gdc->v4l2_dev.name);
    LOG(LOG_INFO, "miscdev.minor = %d!\n", se_gdc->miscdev.minor);
    LOG(LOG_INFO, "miscdev.name = %s!\n", se_gdc->miscdev.name);

    LOG(LOG_INFO, "gdc id = %d\n", se_gdc->id);
    if (count <= 0) {
        LOG(LOG_ERR, "file length is wrong!\n");
        rc = -EINVAL;
        goto result;
    }

    if (se_gdc->p_gdc_ctx != NULL) {
        if (se_gdc->p_gdc_ctx->in.flag_configed == 0) {
            LOG(LOG_ERR, "gdc input format have not set!\n");
            rc = -EINVAL;
            goto result;
        }

        if (se_gdc->p_gdc_ctx->out.flag_configed == 0) {
            LOG(LOG_ERR, "gdc output format have not set!\n");
            rc = -EINVAL;
            goto result;
        }
    } else {
        LOG(LOG_ERR, "device hasn't opened yet!\n");
        rc = -EINVAL;
        goto result;
    }

    fw_data = kzalloc(count, GFP_KERNEL);
    if (fw_data == NULL) {
        LOG(LOG_ERR, "alloc mem failed\n");
        rc = -ENOMEM;
        goto result;
    }

    rc = copy_from_user(fw_data, buf, count);
    if (rc) {
        LOG(LOG_ERR, "copy_from_user failed, not copied: %d, expected: %lu\n", rc, count);
        rc = -EBUSY;
        goto result;
    }

    if (strncmp(fw_id, fw_data, ID_LEN-1) != 0) {
        LOG(LOG_ERR, "fw data id is wrong!\n");
        rc = -EINVAL;
        goto result;
    }

    rc = reload_fw_paramters(se_gdc, fw_data + ID_LEN, count - ID_LEN);
    if (rc) {
        LOG(LOG_ERR, "reload fw data failed!\n");
        goto result;
    }

    LOG(LOG_ERR, "gdc_fw_write successful!\n");

result:
    if (fw_data != NULL) {
        kfree(fw_data);
        fw_data = NULL;
    }
    return rc;
}

static const struct v4l2_file_operations gdc_fops = {
    .owner = THIS_MODULE,
    .open = gdc_open,
    .release = gdc_release,
    .poll = v4l2_m2m_fop_poll,
    .unlocked_ioctl = video_ioctl2,
    .mmap = v4l2_m2m_fop_mmap,
};

static int
vidioc_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
    strlcpy(cap->driver, "siengine gdc", sizeof(cap->driver));
    strlcpy(cap->card, "gdc core", sizeof(cap->card));
    strlcpy(cap->bus_info, "platform:gdc", sizeof(cap->bus_info));
    cap->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING
        | V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_VIDEO_OUTPUT_MPLANE;
    cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

    return 0;
}

static int vidioc_enum_fmt(struct file *file, void *prv, struct v4l2_fmtdesc *f)
{
    if (f->index >= (int)ARRAY_SIZE(se_gdc_formats))
        return -EINVAL;

    f->pixelformat = se_gdc_formats[f->index].fourcc;
    LOG(LOG_INFO, "vidioc_enum_fmt !\n");
    return 0;
}

static int vidioc_g_fmt(struct file *file, void *prv, struct v4l2_format *f)
{
    struct gdc_ctx *ctx = prv;
    struct vb2_queue *vq;
    struct se_gdc_frame_para *frm;

    vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
    if (!vq)
        return -EINVAL;

    frm = gdc_get_format_data(ctx, f->type);
    if (IS_ERR(frm))
        return PTR_ERR(frm);

    LOG(LOG_INFO, "vidioc_g_fmt vb2_queue type = %d\n", vq->type);

    f->fmt.pix.width = frm->width;
    f->fmt.pix.height = frm->height;
    f->fmt.pix.field = V4L2_FIELD_NONE;
    f->fmt.pix.pixelformat = frm->fmt.fourcc;
    f->fmt.pix.sizeimage = frm->total_size;

    return 0;
}

static int try_match_resolution(u32 w, u32 h)
{
    if (w == 1920 && h == 1080)
        return 0;
    else if (w == 1280 && h == 720)
        return 0;
    else if (w == 1280 && h == 800)
        return 0;

    return -1;
}

static int vidioc_try_fmt(struct file *file, void *prv, struct v4l2_format *f)
{
    return 0;
}

static int vidioc_s_fmt(struct file *file, void *prv, struct v4l2_format *f)
{
    struct gdc_ctx *ctx = prv;
    struct se_gdc *se_gdc = ctx->gdc;
    struct v4l2_pix_format_mplane *mp_format = &f->fmt.pix_mp;
    struct vb2_queue *vq;
    struct se_gdc_frame_para *frm;
    int fmt_idex;
    int ret = 0;

    /* Adjust all values accordingly to the hardware capabilities
     * and chosen format.
     */
    LOG(LOG_INFO, "vidioc_s_fmt enter f->type =%d\n", f->type);
    ret = vidioc_try_fmt(file, prv, f);
    if (ret)
        return ret;

    vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
    if (vb2_is_busy(vq)) {
        LOG(LOG_ERR, "queue (%d) bust\n", f->type);
        return -EBUSY;
    }

    ret = try_match_resolution(mp_format->width, mp_format->height);
    if (ret < 0) {
        LOG(LOG_ERR, "Didn't support resolution!\n");
        return -EINVAL;
    }

    frm = gdc_get_format_data(ctx, f->type);
    if (IS_ERR(frm))
        return PTR_ERR(frm);

    fmt_idex = gdc_fmt_find_index(f);
    LOG(LOG_INFO, "vidioc_s_fmt gdc_fmt_find_index fmt_idex = %d\n", fmt_idex);
    if (fmt_idex < 0)
        return -EINVAL;

    frm->width = mp_format->width;
    frm->height = mp_format->height;
    memcpy(&frm->fmt, &se_gdc_formats[fmt_idex], sizeof(struct se_gdc_fmt));
    frm->fmt_index = se_gdc_formats[fmt_idex].index;
    frm->total_size = (mp_format->width * se_gdc_formats[fmt_idex].depth >> PER_BYTE_RR_BITS) * mp_format->height;

    if(f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
        switch(mp_format->pixelformat) {
        case V4L2_PIX_FMT_YUV420:
            mp_format->plane_fmt[0].bytesperline = mp_format->width;
            mp_format->plane_fmt[0].sizeimage = mp_format->height * mp_format->plane_fmt[0].bytesperline;
            mp_format->plane_fmt[1].bytesperline = mp_format->width / 4;
            mp_format->plane_fmt[1].sizeimage = mp_format->height * mp_format->plane_fmt[1].bytesperline;
            mp_format->plane_fmt[2].bytesperline = mp_format->plane_fmt[1].bytesperline;
            mp_format->plane_fmt[2].sizeimage = mp_format->plane_fmt[1].sizeimage;
            break;
        case V4L2_PIX_FMT_NV12:
        case V4L2_PIX_FMT_NV16:
            frm->total_size = (mp_format->width * se_gdc_formats[1].depth >> PER_BYTE_RR_BITS) * mp_format->height;
            mp_format->plane_fmt[0].bytesperline = mp_format->width;
            mp_format->plane_fmt[0].sizeimage = mp_format->height * mp_format->plane_fmt[0].bytesperline;
            mp_format->plane_fmt[1].bytesperline = mp_format->width / 2;
            mp_format->plane_fmt[1].sizeimage =  mp_format->height * mp_format->plane_fmt[1].bytesperline;
            break;
        default:
            LOG(LOG_ERR, "set wrong format!\n");
            return -1;
        }
    } else {
        switch(mp_format->pixelformat) {
        case V4L2_PIX_FMT_YUV420:
            mp_format->plane_fmt[0].bytesperline = mp_format->width;
            mp_format->plane_fmt[0].sizeimage = mp_format->height * mp_format->plane_fmt[0].bytesperline;
            mp_format->plane_fmt[1].bytesperline = mp_format->width / 4;
            mp_format->plane_fmt[1].sizeimage = mp_format->height * mp_format->plane_fmt[1].bytesperline;
            mp_format->plane_fmt[2].bytesperline = mp_format->plane_fmt[1].bytesperline;
            mp_format->plane_fmt[2].sizeimage = mp_format->plane_fmt[1].sizeimage;
            break;
        case V4L2_PIX_FMT_NV12:
            mp_format->plane_fmt[0].bytesperline = mp_format->width;
            mp_format->plane_fmt[0].sizeimage = mp_format->height * mp_format->plane_fmt[0].bytesperline;
            mp_format->plane_fmt[1].bytesperline = mp_format->width / 2;
            mp_format->plane_fmt[1].sizeimage =  mp_format->height * mp_format->plane_fmt[1].bytesperline;
            break;
        case V4L2_PIX_FMT_NV16:
            mp_format->plane_fmt[0].bytesperline = mp_format->width;
            mp_format->plane_fmt[0].sizeimage = mp_format->height * mp_format->plane_fmt[0].bytesperline;
            mp_format->plane_fmt[1].bytesperline = mp_format->plane_fmt[0].bytesperline;
            mp_format->plane_fmt[1].sizeimage =  mp_format->plane_fmt[0].sizeimage;
            break;
        default:
            LOG(LOG_ERR, "set wrong format!\n");
            return -1;
        }
    }

    //indicate frame_para has configured
    frm->flag_configed = 1;

    LOG(LOG_INFO, "vidioc_s_fmt frm->total_size = %d\n", frm->total_size);

    if(se_gdc->p_gdc_ctx->in.flag_configed && se_gdc->p_gdc_ctx->out.flag_configed) {
        if(gdc_fw_init(se_gdc) < 0) {
            LOG(LOG_ERR, "reinit fw data for gdc failed!\n");
            return -EBUSY;
        }
    }
    return 0;
}
static int vidioc_gdc_reqbufs(struct file *file, void *priv,
                              struct v4l2_requestbuffers *reqbufs)
{
    struct gdc_ctx *ctx = priv;

    LOG(LOG_INFO, "reqbufs buf type = %d\n", reqbufs->type);
    return v4l2_m2m_reqbufs(file, ctx->fh.m2m_ctx, reqbufs);
}

static int vidioc_gdc_querybuf(struct file *file, void *priv,
                               struct v4l2_buffer *buf)
{
    struct gdc_ctx *ctx = priv;

    LOG(LOG_INFO, "querybuf buf type = %d\n", buf->type);
    return v4l2_m2m_querybuf(file, ctx->fh.m2m_ctx, buf);
}

static int vidioc_gdc_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
    struct gdc_ctx *ctx = priv;

    LOG(LOG_INFO, "qbuf buf type = %d\n", buf->type);
    return v4l2_m2m_qbuf(file, ctx->fh.m2m_ctx, buf);
}

static int vidioc_gdc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
    struct gdc_ctx *ctx = priv;

    LOG(LOG_INFO, "dqbuf buf type = %d\n", buf->type);
    return v4l2_m2m_dqbuf(file, ctx->fh.m2m_ctx, buf);
}

static int vidioc_s_streamon(struct file *file, void *priv,
                             enum v4l2_buf_type type)
{
    int ret = 0;
    struct gdc_ctx *ctx = priv;

    ret = v4l2_m2m_streamon(file, ctx->fh.m2m_ctx, type);

    LOG(LOG_INFO, "vidioc_s_streamon return = %d\n", ret);
    return ret;
}

static int vidioc_s_streamoff(struct file *file, void *priv,
                              enum v4l2_buf_type type)
{
    int ret = 0;
    struct gdc_ctx *ctx = priv;

    ret = v4l2_m2m_streamoff(file, ctx->fh.m2m_ctx, type);

    LOG(LOG_INFO, "vidioc_s_streamon return = %d\n", ret);
    return ret;
}

static int vidioc_g_selection(struct file *file, void *prv,
                              struct v4l2_selection *s)
{
    return 0;
}

static int vidioc_s_selection(struct file *file, void *prv,
                              struct v4l2_selection *s)
{
    return 0;
}

static const struct v4l2_ioctl_ops gdc_ioctl_ops = {
    .vidioc_querycap = vidioc_querycap,

    .vidioc_enum_fmt_vid_cap = vidioc_enum_fmt,
    .vidioc_g_fmt_vid_cap_mplane = vidioc_g_fmt,
    .vidioc_try_fmt_vid_cap_mplane = vidioc_try_fmt,
    .vidioc_s_fmt_vid_cap_mplane = vidioc_s_fmt,

    .vidioc_enum_fmt_vid_out = vidioc_enum_fmt,
    .vidioc_g_fmt_vid_out_mplane = vidioc_g_fmt,
    .vidioc_try_fmt_vid_out_mplane = vidioc_try_fmt,
    .vidioc_s_fmt_vid_out_mplane = vidioc_s_fmt,

    .vidioc_reqbufs = vidioc_gdc_reqbufs,
    .vidioc_querybuf = vidioc_gdc_querybuf,
    .vidioc_qbuf = vidioc_gdc_qbuf,
    .vidioc_dqbuf = vidioc_gdc_dqbuf,
    .vidioc_streamon = vidioc_s_streamon,
    .vidioc_streamoff = vidioc_s_streamoff,
    .vidioc_expbuf = v4l2_m2m_ioctl_expbuf,

    .vidioc_g_selection = vidioc_g_selection,
    .vidioc_s_selection = vidioc_s_selection,

};

static struct video_device gdc_videodev = {
    .name = "siengine-gdc",
    .fops = &gdc_fops,
    .ioctl_ops = &gdc_ioctl_ops,
    .minor = -1,
    .release = video_device_release,
    .vfl_dir = VFL_DIR_M2M,
    .device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_VIDEO_OUTPUT_MPLANE,
};

int gdc_misc_dev_open(struct inode *inode, struct file *filp)
{
    LOG(LOG_DEBUG, "gdc misc dev open!\n");

    return 0;
}

int gdc_misc_dev_close(struct inode *inode, struct file *filp)
{
    LOG(LOG_DEBUG, "gdc misc dev close!\n");
    return 0;
}

struct file_operations gdc_misc_file_ops = {
    .owner = THIS_MODULE,
    .open = gdc_misc_dev_open,
    .release = gdc_misc_dev_close,
    .write = gdc_fw_write,
};

static int gdc_parse_dt(struct se_gdc *se_gdc)
{
    struct device *dev = &se_gdc->pdev->dev;
    struct device_node *node = dev->of_node;
    struct resource *res;
    int ret = 0;

    se_gdc->id = of_alias_get_id(node, "gdc");
    if(se_gdc->id < 0)
        return se_gdc->id;

    LOG(LOG_INFO, "gdc id: %d\n", se_gdc->id);
    //sprintf(se_gdc->gdc_node_name, "gdc%d", se_gdc->id);
    //LOG(LOG_INFO, "gdc_node_name: %s\n", se_gdc->gdc_node_name);

    se_gdc->irq_res = platform_get_resource(se_gdc->pdev, IORESOURCE_IRQ, 0);
    if (se_gdc->irq_res == NULL) {
        LOG(LOG_ERR, "Failed to get gdc%d IRQ resource\n", se_gdc->id);
        return -ENXIO;
    }
    LOG(LOG_INFO, "irq number: %d\n", (int)se_gdc->irq_res->start);

    res = platform_get_resource_byname(se_gdc->pdev, IORESOURCE_MEM, RES_CTRL_NAME);
    se_gdc->regs = devm_ioremap_resource(dev, res);
    if(IS_ERR(se_gdc->regs)) {
        LOG(LOG_ERR, "Failed to get GDC dm%d register map\n", se_gdc->id);
        return PTR_ERR(se_gdc->regs);
    }
    LOG(LOG_INFO, "se_gdc->regs: 0x%llx\n", (u64)(se_gdc->regs));

    res = platform_get_resource_byname(se_gdc->pdev, IORESOURCE_MEM, RES_DM_NAMD);
    se_gdc->dm_regs = devm_ioremap_resource(dev, res);
    if(IS_ERR(se_gdc->dm_regs)) {
        LOG(LOG_ERR, "Failed to get GDC dm%d register map\n", se_gdc->id);
        return PTR_ERR(se_gdc->dm_regs);
    }
    LOG(LOG_INFO, "se_gdc->dm_regs: 0x%llx\n", (u64)(se_gdc->dm_regs));

    se_gdc->core_clk = of_clk_get_by_name(dev->of_node, GDC_CORE_CLK);
    ret = PTR_ERR(se_gdc->core_clk);
    if (IS_ERR(se_gdc->core_clk)) {
        LOG(LOG_ERR, "Unable to get core_clk: %d\n", ret);
        return ret;
    }

    se_gdc->axi_clk = of_clk_get_by_name(dev->of_node, GDC_AXI_CLK);
    ret = PTR_ERR(se_gdc->axi_clk);
    if (IS_ERR(se_gdc->axi_clk)) {
        LOG(LOG_ERR, "Unable to get axi_clk: %d\n", ret);
        return ret;
    }

    return 0;
}

static int __init gdc_platform_probe(struct platform_device *pdev)
{
    struct se_gdc *se_gdc;
    struct device *dev = &pdev->dev;
    const struct of_device_id *of_id;
    struct video_device *vfd;
    int ret = 0;

    if (!pdev->dev.of_node) {
        LOG(LOG_ERR, "Failed to dt node is NULL\n");
        return -ENODEV;
    }

    of_id = of_match_node(se_gdc_of_match, dev->of_node);
    if(!of_id) {
        LOG(LOG_ERR, "Failed to dt parse\n");
        return -EINVAL;
    }

    se_gdc = devm_kzalloc(&pdev->dev, sizeof(*se_gdc), GFP_KERNEL);
    if(!se_gdc) {
        LOG(LOG_ERR, "Failed to allocate struct se_gdc\n");
        return -ENOMEM;
    }

    se_gdc->pdev = pdev;
    spin_lock_init(&se_gdc->ctrl_lock);
    mutex_init(&se_gdc->mutex);

    init_waitqueue_head(&se_gdc->irq_queue);

    ret = gdc_parse_dt(se_gdc);
    if (ret) {
        LOG(LOG_ERR, "Unable to parse OF data\n");
        return -EINVAL;
    }

    pm_runtime_enable(&pdev->dev);
    ret = gdc_clk_enable(se_gdc, true);
    if (ret < 0) {
        LOG(LOG_ERR, "failed to turn on clk\n");
        goto err_clk;
    }

    ret = devm_request_irq(dev, se_gdc->irq_res->start, se_gdc_irq_handler,
                           (se_gdc->irq_res->flags & IRQF_TRIGGER_MASK) |IRQF_ONESHOT, dev_name(dev), se_gdc);
    if (ret < 0) {
        LOG(LOG_ERR, "failed to install irq (%d)\n", ret);
        goto err_irq;
    }

    disable_irq(se_gdc->irq_res->start);

    LOG(LOG_INFO, "smmu config!\n");
    ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
    if ( ret < 0 ) {
        LOG(LOG_ERR, "dma_set_mask_and_coherent failed.\n");
        goto err_dma_se;
    }

    snprintf(se_gdc->v4l2_dev.name, sizeof(se_gdc->v4l2_dev.name), "%s.%d", GDC_V4L2_MODULE_NAME, se_gdc->id);
    ret = v4l2_device_register(dev, &se_gdc->v4l2_dev);
    if (ret) {
        LOG(LOG_ERR, "Failed to v4l2 device register\n");
        goto err_dma_se;
    }

    vfd = video_device_alloc();
    if (!vfd) {
        LOG(LOG_ERR, "Failed to allocate video device\n");
        ret = -ENOMEM;
        goto unreg_v4l2_dev;
    }

    *vfd = gdc_videodev;
    vfd->lock = &se_gdc->mutex;
    vfd->v4l2_dev = &se_gdc->v4l2_dev;

    video_set_drvdata(vfd, se_gdc);
    snprintf(vfd->name, sizeof(vfd->name), "%s", gdc_videodev.name);
    snprintf(se_gdc->gdc_node_name, sizeof(se_gdc->gdc_node_name), GDC_DEV_NODE_NAME, se_gdc->id);
    vfd->dev.init_name = se_gdc->gdc_node_name;
    se_gdc->vfd = vfd;

    se_gdc->m2m_dev = v4l2_m2m_init(&gdc_m2m_ops);
    if (IS_ERR(se_gdc->m2m_dev)) {
        LOG(LOG_ERR, "Failed to init mem2mem device\n");
        ret = PTR_ERR(se_gdc->m2m_dev);
        goto rel_vdev;
    }

    ret = video_register_device(vfd, VFL_TYPE_VIDEO, -1);
    if (ret) {
        LOG(LOG_ERR, "Failed to register video device\n");
        goto rel_vdev;
    }

    LOG(LOG_INFO, "Registered %s as /dev/%s\n",
        vfd->name, video_device_node_name(vfd));

    snprintf(se_gdc->gdc_misc_name, sizeof(se_gdc->gdc_misc_name), GDC_FW_NODE_NAME, se_gdc->id);
    se_gdc->miscdev.minor = MISC_DYNAMIC_MINOR;
    se_gdc->miscdev.name = se_gdc->gdc_misc_name;
    se_gdc->miscdev.parent = dev;
    se_gdc->miscdev.fops   = &gdc_misc_file_ops;
    ret = misc_register(&se_gdc->miscdev);
    if (ret) {
        LOG(LOG_ERR, "Failed to register video device\n");
        goto msic_er;
    }

    se_gdc->gdc_settings.is_waiting_gdc = 0;       //set when expecting an interrupt from gdc
    se_gdc->gdc_settings.dev = &se_gdc->pdev->dev;
    se_gdc->gdc_settings.base_gdc = (u64)se_gdc->regs;
    gdc_stop(&se_gdc->gdc_settings);

    enable_irq(se_gdc->irq_res->start);
    timer_setup(&se_gdc->irq_timer, gdc_translate_timeout, 0);

    LOG(LOG_INFO, "gdc_probe done!\n");
    platform_set_drvdata(pdev, se_gdc);
    return 0;

msic_er:
    video_unregister_device(vfd);
rel_vdev:
    video_device_release(vfd);
unreg_v4l2_dev:
    v4l2_device_unregister(&se_gdc->v4l2_dev);

err_dma_se:
    devm_free_irq(dev, se_gdc->irq_res->start, se_gdc);

err_irq:
    gdc_clk_enable(se_gdc, false);

err_clk:
    pm_runtime_disable(dev);

    devm_kfree(dev, se_gdc);

    LOG(LOG_ERR, "return error ret = %d", ret);

    return ret;
}

static int gdc_platform_remove(struct platform_device *pdev)
{
    struct se_gdc *se_gdc = platform_get_drvdata(pdev);

    LOG(LOG_INFO, "Removing\n");

    del_timer(&se_gdc->irq_timer);
    kfree(se_gdc->p_gdc_ctx);
    v4l2_m2m_release(se_gdc->m2m_dev);
    misc_deregister(&se_gdc->miscdev);
    video_unregister_device(se_gdc->vfd);
    v4l2_device_unregister(&se_gdc->v4l2_dev);
    gdc_clk_enable(se_gdc, false);
    pm_runtime_disable(&se_gdc->pdev->dev);
    devm_kfree(&se_gdc->pdev->dev, se_gdc);
    platform_set_drvdata(pdev, NULL);

    return 0;
}

#ifdef CONFIG_PM
static int gdc_runtime_suspend(struct device *dev)
{
    struct se_gdc *se_gdc = dev_get_drvdata(dev);

    gdc_clk_enable(se_gdc, false);
    pr_info("gdc_runtime_suspend\n");
    return 0;//pm_runtime_force_suspend(dev);
}

static int gdc_runtime_resume(struct device *dev)
{
    struct se_gdc *se_gdc = dev_get_drvdata(dev);

    //pm_runtime_force_suspend(dev);
    gdc_clk_enable(se_gdc, true);
    pr_info("gdc_runtime_resume\n");
    return 0;
}

static int gdc_suspend(struct platform_device *pdev, pm_message_t message)
{
    struct se_gdc *gdc_dev = platform_get_drvdata(pdev);
    gdc_clk_enable(gdc_dev, false);
    pr_info("gdc_suspend\n");
    return 0;
}

static int gdc_resume(struct platform_device *pdev)
{
    struct se_gdc *gdc_dev = platform_get_drvdata(pdev);
    gdc_clk_enable(gdc_dev, true);
    pr_info("gdc_resume\n");
    return 0;
}

#else
static int gdc_runtime_suspend(struct device *dev)
{
   return 0;
}

static int gdc_runtime_resume(struct device *dev)
{
    return 0;
}

static int gdc_suspend(struct platform_device *pdev, pm_message_t message)
{
    return 0;
}

static int gdc_resume(struct platform_device *pdev)
{
    return 0;
}

#endif

static const struct dev_pm_ops se_gdc_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(gdc_runtime_suspend,
                       gdc_runtime_resume)
};

static struct platform_driver gdc_platform_driver = {
    .remove     = gdc_platform_remove,
    //.suspend	= gdc_suspend,
    //.resume		= gdc_resume,
    .driver = {
        .name           = GDC_DRIVER_NAME,
        .owner          = THIS_MODULE,
        .pm             = &se_gdc_pm_ops,
        .of_match_table = se_gdc_of_match,
    }
};

static int __init gdc_module_init( void )
{
    int32_t rc = 0;

    rc = platform_driver_probe( &gdc_platform_driver,
                                gdc_platform_probe );
    return rc;
}

static void __exit gdc_module_exit( void )
{
    LOG( LOG_INFO, "Juno gdc fw_module_exit\n" );

    platform_driver_unregister( &gdc_platform_driver );
}

module_init( gdc_module_init );
module_exit( gdc_module_exit );

MODULE_AUTHOR("Ramon.li@siengine.com");
MODULE_DESCRIPTION("GDC Subsystem driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("GDC");
MODULE_VERSION("1.0");
