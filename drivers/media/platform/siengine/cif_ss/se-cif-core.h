// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021-2022 Siengine Technology, Inc.
 *
 * Sienginie Camera Interface driver
 *
 * Author: Siengine Technology, Inc.
 */
#ifndef SE_CIF_CORE_H_
#define SE_CIF_CORE_H_

#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>
#include "se-nr-hw.h"

#define SE_CIF_MAX_DEVS 22

#define CIF_OF_NODE_NAME	"cif"
#define CIF_DRIVER_NAME	"se-cif"

#define CIF_SD_PAD_SINK_MIPI0_VC0	0
#define CIF_SD_PAD_SINK_MIPI0_VC1	1
#define CIF_SD_PAD_SINK_MIPI0_VC2	2
#define CIF_SD_PAD_SINK_MIPI0_VC3	3

#define CIF_SD_PAD_SOURCE_MEM	32
#define CIF_SD_PADS_NUM	33

#define MAX_PLANES	1

#define WIDTH_MAX	3840
#define WIDTH_MIN	224

#define HEIGHT_MAX	2160
#define HEIGHT_MIN	224

#define FPS_MIN	1
#define FPS_MAX	60

/* timeperframe: min/max and default */
static const struct v4l2_fract
	tpf_min = {.numerator = 1, .denominator = FPS_MAX},
	tpf_max = {.numerator = 1, .denominator = FPS_MIN},
	tpf_default = {.numerator = 1000, .denominator = 30000};


enum cif_input_interface {
	CIF_INPUT_INTERFACE_MIPI0_CSI2,
	CIF_INPUT_INTERFACE_MIPI1_CSI2,
	CIF_INPUT_INTERFACE_MIPI2_CSI2,
	CIF_INPUT_INTERFACE_MIPI3_CSI2,
	CIF_INPUT_INTERFACE_MIPI4_CSI2,
	CIF_INPUT_INTERFACE_MIPI5_CSI2,
	CIF_INPUT_INTERFACE_MIPI6_CSI2,
	CIF_INPUT_INTERFACE_MIPI7_CSI2,
	CIF_INPUT_INTERFACE_MAX,
};

enum cif_input_sub_interface {
	CIF_INPUT_SUB_INTERFACE_VC0 = 0,
	CIF_INPUT_SUB_INTERFACE_VC1,
	CIF_INPUT_SUB_INTERFACE_VC2,
	CIF_INPUT_SUB_INTERFACE_VC3,
};

enum cif_output_interface {
	CIF_OUTPUT_INTERFACE_MEM,
};

enum {
	IN_PORT,
	SUB_IN_PORT,
	OUT_PORT,
	MAX_PORTS,
};

enum se_cif_buf_id {
	SE_CIF_BUFA	= 0x0,
	SE_CIF_BUFB,
	SE_CIF_BOTH_BUF,
};

struct frame_addr {
	u32 a;
	u32 b;
};

struct se_cif_buffer {
	struct vb2_v4l2_buffer v4l2_buf;
	struct list_head list;
	struct frame_addr paddr;
	enum se_cif_buf_id id;
	bool discard;
};

/**
 * struct se_cif_frame - source/target frame properties
 * width:	out image pixel width
 * height:	out image pixel weight
 * sizeimage: frame size
 * fmt:	color format pointer
 */
struct se_cif_frame {
	u32 o_width;
	u32 o_height;
	u32 width;
	u32 height;
	u32 hw_width;
	unsigned int sizeimage;
	struct se_cif_fmt *fmt;
	unsigned int bytesperline;
};

typedef struct delta_coef_array {
	u8 delta_coef_center;  //0 -> 127, default 31, 127 represent 128, 0 represent 1
	u8 delta_coef_midcircle;  //-5 -> 5, default -4, 2's complement
	u8 delta_coef_outercenter;  // -3 -》 3， default -2, 2's complement
} delta_coef_array;

typedef struct rgbir422_coef_param {
	//coef_n_1
	u8 reg_avg_i_mode;   // 0->1, 0: 6pixel avg, 1: 2 pixel avg, default 0
	u8 reg_delta_offset_R;  //0 -> 127, default 0, 0 represent 0
	u8 reg_delta_offset_B;  //0 -> 127derfault 0, 0 represent 0
	u8 reg_delta_offset_Ir; // 0 -> 127, default 0, 0 represent 0
	u8 reg_delta_dn_shiftbit_R; // 0 -> 7, default 6, 0 present 0
	u8 reg_delta_dn_shiftbit_B; // 0 -> 7, default 6, 0 represent 0
	u8 reg_delta_dn_shiftbit_Ir; //0 -> 7, default 6, 0 represent 0
	//coef_n_2
	u8 reg_delta_gain_R;  //0 -> 255, default 16, 0 represent 0
	u8 reg_delta_gain_B; //0 -> 255, default 16, 0 represent 0
	u8 reg_delta_gain_Ir; //0 -> 255, default 16, 0 represent 0

	delta_coef_array array_R; //coef_n_3
	delta_coef_array array_B; //coef_n_4
	delta_coef_array array_Ir; //coef_n_5
} rgbir422_coef_param;

struct se_cif_cap_dev {
	struct v4l2_subdev sd;
	struct video_device vdev;
	struct v4l2_fh fh;
	struct media_pad cap_pad;
	struct media_pad sd_pads[CIF_SD_PADS_NUM];
	struct vb2_queue vb2_q;
	struct list_head out_pending;
	struct list_head out_active;
	struct list_head out_discard;
	struct se_cif_frame src_f;
	struct se_cif_frame dst_f;
	bool se_nr_enable;
	se_nr_dev nr_dev;
	bool se_rgbir422_enable;
	rgbir422_coef_param rgbir422_param;
	u32 frame_count;
#if IS_ENABLED(CONFIG_SE_CIF_DEBUG)
	unsigned int total_hw_fps;
	unsigned int total_ac_fps;
	unsigned int curr_hw_fps;
	unsigned int curr_ac_fps;
	u64	frame_start_ns;
	u64	frame_end_ns;
	u64	frame_per_s_ns;
	u64	frame_per_e_ns;
	u64	frame_pre_ns;
	u64	frame_curr_ns;
	u32	loss_count;
	u32	jitter_count;
	u64	q_timestamp;
	u64	dq_timestamp;
	u32	interval_qdq;
#endif
	u32 buf_index;
};

struct cif_dm_dev {
	void __iomem            *dm_regs;
};

struct se_cif_dev {
	spinlock_t slock;
	struct mutex lock;
	struct mutex m2m_lock;
	wait_queue_head_t irq_queue;

	int id;
	void __iomem *regs;
	void __iomem *dm_regs;
	unsigned long state;

	struct platform_device *pdev;
	struct v4l2_device *v4l2_dev;
	struct se_cif_cap_dev cif_cap;
	struct clk *clk;

	bool is_streaming;

	u32 interface[MAX_PORTS];
	u32 flags;
	u32 status;
	struct se_cif_buffer buf;
	struct v4l2_pix_format_mplane pix;
	size_t discard_size[MAX_PLANES];
	void *discard_buffer[MAX_PLANES];
	dma_addr_t discard_buffer_dma[MAX_PLANES];
	struct se_cif_buffer buf_discard[2];
	atomic_t open_count;
};

enum cif_out_fmt {
	CIF_OUT_FMT_RAW8,
	CIF_OUT_FMT_RAW10,
	CIF_OUT_FMT_RAW12,
	CIF_OUT_FMT_RAW16,
	CIF_OUT_FMT_YUV422,
	CIF_OUT_FMT_ARGB8888,
};

struct se_cif_fmt {
	char *name;
	u32 mbus_code;
	u32 fourcc;
	u32 color;
	u16 memplanes;
	u16 colplanes;
	u8 colorspace;
	u8 depth;
	u16 mdataplanes;
	u16 flags;
};

static inline void set_frame_bounds(struct se_cif_frame *f, u32 width, u32 height)
{
	f->o_width  = width;
	f->o_height = height;
	f->width  = width;
	f->height = height;
}

#if IS_ENABLED(CONFIG_SE_CIF_DEBUG)
int se_cif_create_capabilities_sysfs(struct platform_device *pdev);
#endif
#endif
