#ifndef __GDC_DRIVER_H__
#define __GDC_DRIVER_H__

#include <linux/platform_device.h>
#include <media/videobuf2-v4l2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/timer.h>
#include <linux/miscdevice.h>
#include <linux/devfreq.h>
#include <linux/devfreq_cooling.h>
#include <linux/pm_opp.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>

#define GDC_DRIVER_NAME  "siengine,gdc-core"
#define RES_CTRL_NAME "gdc-ctrl"
#define RES_DM_NAMD   "gdc-dm"
#define GDC_CORE_CLK "cisp-gdc-core-clk"
#define GDC_AXI_CLK "cisp-axi-clk"
#define GDC_V4L2_MODULE_NAME "gdc-v4l2"
#define GDC_DEV_NODE_NAME "video%dgdc"
#define GDC_FW_NODE_NAME  "gdc%dfw"

#define INT_REG_BUSY_MASK   0x0001
#define INT_REG_ERROR_MASK  0x0002
#define INT_REG_CONF_ERROR_MASK 0x0100
#define INT_REG_USER_ABORT_MASK 0x0200
#define INT_REG_AXI_READER_ERROR_MASK 0x0400
#define INT_REG_AXI_WRITER_ERROR_MASK 0x0800
#define INT_REG_UNALIGNED_ACCESSS_ERROR_MASK 0x1000
#define INT_REG_INCOMPATIBLE_CONFIG_ERROR_MASK 0x2000

#define ACAMERA_GDC_MAX_INPUT 3
#define PLANES_MAX ACAMERA_GDC_MAX_INPUT

#define CONFIG_MEM_ALIGN 4
#define ALIGN_BYTES 16
#define SIZE_ALIGN(s, align) (s + (align - 1))&(~(align - 1))
#define GDC_ABORT_TIMEOUT 500
#define GDC_IRQ_TIMER_TIMEOUT_MS 5000

#define PER_BYTE_RR_BITS 3
#define ID_LEN 10

//gdc cases available
enum gdc_available_format {
    gdc_yuv420_planar,
    gdc_yuv420sp_nv12,
    gdc_yuv422sp_nv16,
    gdc_format_max,
};

// each configuration addresses and size
typedef struct gdc_config {
    u32 config_size;   //gdc config size in 32bit
    u32 fw_alloc_mem_size; // gdc config mem total size
    u32 input_width;  //gdc input width resolution
    u32 input_height; //gdc input height resolution
    u32 output_width;  //gdc output width resolution
    u32 output_height; //gdc output height resolution
    u8  total_planes;
    u8  div_width;     //use in dividing UV dimensions; actually a shift right
    u8  div_height;	//use in dividing UV dimensions; actually a shift right
    u8  sequential_mode; //sequential processing
} gdc_config_t;

// overall gdc settings and state
typedef struct gdc_settings {
    struct device *dev;
    u64 base_gdc;             //writing/reading to gdc base address, currently not read by api
    dma_addr_t fw_address_pa;        //fw dma alloc pa
    void *fw_address_va;           //fw dma alloc va
    dma_addr_t input_address_pa[PLANES_MAX];  //input address 16byte allian
    dma_addr_t output_address_pa[PLANES_MAX]; //output address
    u8 is_waiting_gdc;       //set when expecting an interrupt from gdc
    gdc_config_t gdc_config;       //array of gdc configuration and sizes
} gdc_settings_t;

struct se_gdc_fmt {
    char *name;
    u32  fourcc;
    u8   depth;
    u8   memplanes;
    u8   mdataplanes;
    u8   flags;
    u8   index;
};

struct se_gdc_frame_para {
    u32 width;
    u32 height;
    u32 total_size;
    u8  fmt_index;
    u8  flag_configed;
    struct se_gdc_fmt fmt;
};

struct gdc_ctx {
    struct v4l2_fh fh;
    struct se_gdc *gdc;
    struct se_gdc_frame_para in;
    struct se_gdc_frame_para out;
};

typedef struct se_gdc {
    struct v4l2_device v4l2_dev;
    struct v4l2_m2m_dev *m2m_dev;
    struct video_device *vfd;
    struct miscdevice   miscdev;

    struct platform_device  *pdev;
    void __iomem *regs;
    void __iomem *dm_regs;

    struct clk *axi_clk;
    struct clk *core_clk;

    /* vfd lock */
    struct mutex mutex;
    /* ctrl parm lock */
    spinlock_t ctrl_lock;
    wait_queue_head_t irq_queue;
    struct timer_list irq_timer;

    struct gdc_ctx *p_gdc_ctx;
    //gdc add start
    int id;
    char gdc_node_name[32];
    char gdc_misc_name[32];
    struct resource *irq_res;
    gdc_settings_t gdc_settings;
    bool clk_state;
    atomic_t open_count;
    //gdc add end
} se_gdc_t;

struct se_gdc_frame_para *gdc_get_format_data(struct gdc_ctx *ctx, enum v4l2_buf_type type);

/* GDC Buffers Manage Part */
extern const struct vb2_ops gdc_qops;

#endif
