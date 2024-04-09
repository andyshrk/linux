/*
 *
 *    The GPL License (GPL)
 *
 *    Copyright (C) 2014 - 2021 VERISILICON
 *    Copyright (C) 2022 - 2023 Siengine
 *
 *    This program is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU General Public License
 *    as published by the Free Software Foundation; either version 2
 *    of the License, or (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software Foundation,
 *    Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/wait.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/mod_devicetable.h>
#include <linux/dma-buf.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/compat.h>
#include <linux/devfreq.h>
#include <linux/devfreq_cooling.h>

#include "hantro_dec.h"
#include "dwl_defs.h"
#ifdef SUPPORT_DEVMGR
#include "dec_devicemanager.h"
#endif

#undef PDEBUG
#ifdef HANTRODEC_DEBUG
#  define PDEBUG(fmt, args...) pr_debug("hantrodec: " fmt, ## args)
#else
#  define PDEBUG(fmt, args...)
#endif

/* PCIe hantro driver offset in control register */
#define HANTRO_REG_OFFSET0              0x600000
#define HANTRO_REG_OFFSET1              0x700000

#define HANTRO_G1_DEC_REGS              155 /* G1 total regs */

#define HANTRO_PP_ORG_FIRST_REG         60
#define HANTRO_PP_ORG_LAST_REG          100
#define HANTRO_PP_EXT_FIRST_REG         146
#define HANTRO_PP_EXT_LAST_REG          154

/* hantro G2 reg config */
#define HANTRO_G2_DEC_REGS              337 /* G2 total regs */
#define HANTRO_G2_DEC_FIRST_REG         0
#define HANTRO_G2_DEC_LAST_REG          (HANTRO_G2_DEC_REGS - 1)

/* hantro VC8000D reg config */
#define HANTRO_VC8000D_REGS             472 /* VC8000D total regs */
#define HANTRO_VC8000D_FIRST_REG        0
#define HANTRO_VC8000D_LAST_REG         (HANTRO_VC8000D_REGS - 1)

#define HANTRO_BIGOCEAN_REGS            304 /* 1216 bytes for BigOcean */

/* Logic module IRQs */
#define HXDEC_NO_IRQ                    -1

#define MAX(a, b)                       (((a) > (b)) ? (a) : (b))

#define DEC_IO_SIZE_MAX  (MAX(MAX(HANTRO_G2_DEC_REGS, HANTRO_G1_DEC_REGS), \
			  HANTRO_VC8000D_REGS) * 4)

/*
 * User should modify these configuration if do porting to own platform.
 * Please guarantee the base_addr, io_size, dec_irq belong to same core.
 */

/* Logic module base address */
#define SOCLE_LOGIC_0_BASE              0x38300000
#define SOCLE_LOGIC_1_BASE              0x38310000

#define VEXPRESS_LOGIC_0_BASE           0xFC010000
#define VEXPRESS_LOGIC_1_BASE           0xFC020000

#define DEC_IO_SIZE_0                   DEC_IO_SIZE_MAX /* bytes */
#define DEC_IO_SIZE_1                   DEC_IO_SIZE_MAX /* bytes */

#define DEC_IRQ_0                       HXDEC_NO_IRQ
#define DEC_IRQ_1                       HXDEC_NO_IRQ

#define IS_G1(hw_id)                    (((hw_id) == 0x6731) ? 1 : 0)
#define IS_G2(hw_id)                    (((hw_id) == 0x6732) ? 1 : 0)
#define IS_VC8000D(hw_id)               (((hw_id) == 0x8001) ? 1 : 0)
#define IS_BIGOCEAN(hw_id)              (((hw_id) == 0xB16D) ? 1 : 0)

/*********************local variable declaration*****************/

static const int DecHwId[] = {
	0x6731, /* G1 */
	0x6732, /* G2 */
	0xB16D, /* BigOcean */
	0x8001  /* VC8000D */
};

static unsigned long base_port = -1;
static unsigned int reg_access_opt = 1;

static struct class *hantro_class;
#define DEVICE_NAME "hantro_dec"
#define SUSPEND_DELAY_MS                100

unsigned long multicorebase[HXDEC_MAX_CORES] = {
	HANTRO_REG_OFFSET0,
	HANTRO_REG_OFFSET1,
	0,
	0
};

int irq[HXDEC_MAX_CORES] = {
	DEC_IRQ_0,
	DEC_IRQ_1,
	-1,
	-1
};

unsigned int iosize[HXDEC_MAX_CORES] = {
	DEC_IO_SIZE_0,
	DEC_IO_SIZE_1,
	-1,
	-1
};

int reg_count[HXDEC_MAX_CORES] = {
	0,
	0,
	0,
	0
};

/*
 * Because one core may contain multi-pipeline,
 * so multicore base may be changed
 */
unsigned long multicorebase_actual[HXDEC_MAX_CORES];

struct subsys_config vpu_subsys[MAX_SUBSYS_NUM];

int elements = 2;

/*********************subsystem configuration********************/
/* List of subsystems */
static struct SubsysDesc subsys_array[] = {
	/* {slice_index, index, base} */
	{0, 0, 0xe7800000}  /* SE1000 VDEC */
};

/*
 * List of all HW cores.
 *  SE1000 chip
 *      |-slice 0
 *      `-subsys 0
 *              |- VC8000D
 *              |- L2CACHE
 *              `- SFBC ENC
 */

struct CoreDesc core_array[] = {
	/* {slice, subsys, core_type, offset, iosize, irq} */
	/*
	 * SE1000: VC8000D baseaddress=0xe7802000,
	 * L2Cache baseaddress=0xe7814000
	 */
	{0, 0, HW_VC8000D, 0x02000, 472*4, -1},	/* INTR_VCD = 319 */
	{0, 0, HW_L2CACHE, 0x14000, 231*4, -1},	/* L2Cache offset 0x4000 */
#ifdef SUPPORT_SFBC
	{0, 0, HW_SFBC_ENC, 0x20000, 64*4, -1}	/* SFBC ENC offset 0x20000 */
#endif
};

/* here's all the must remember stuff */
struct dma_buf_info {
	int fd;					/* dma_buf fd */
	dma_addr_t dma_addr;			/* dma address */
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	struct list_head list;
	struct file *owner;			/* owner of used buffer */
};

struct hantrodec {
	struct device *dev;
	char *buffer;
	unsigned int iosize[HXDEC_MAX_CORES];
	/* mapped address to different HW cores regs */
	u8 *hwregs[HXDEC_MAX_CORES][HW_CORE_MAX];
	int irq[HXDEC_MAX_CORES];
	int hw_id[HXDEC_MAX_CORES];
	int cores;
	struct fasync_struct *async_queue_dec;
	struct fasync_struct *async_queue_pp;
	struct list_head dma_buf_list;
	struct mutex dmabuf_list_lock;
	/* clock & reset */
	struct clk *clk_axi;			/* clock axi */
	struct clk *clk_core;			/* clock core */
	struct clk *clk_apb;			/* clock apb */
	struct reset_control *rst_axi;		/* reset axi */
	struct reset_control *rst_core;		/* reset core */
	struct reset_control *rst_apb;		/* reset apb */
	/* devfreq & cooling */
	struct opp_table *opp_table;
	bool opp_table_added;
	struct devfreq *devfreq;
	struct thermal_cooling_device *cooling;
	/* str */
	u32 reg_backup[HANTRO_VC8000D_REGS];
	atomic_t channels;
};

static struct hantrodec hantrodec_data;

/*
 * Covnert core_array to multicore_base/irq/iosize,
 * which are used in hantro_dec.c
 *
 *  - multicorebase[HXDEC_MAX_CORES]
 *  - irq[HXDEC_MAX_CORES]
 *  - iosize[HXDEC_MAX_CORES]
 */
void CheckSubsysCoreArray(struct subsys_config *subsys)
{
	int num = ARRAY_SIZE(subsys_array);
	int i, j;

	memset(subsys, 0, sizeof(subsys[0]) * MAX_SUBSYS_NUM);
	for (i = 0; i < num; i++) {
		subsys[i].base_addr = subsys_array[i].base;
		subsys[i].irq = -1;
		for (j = 0; j < HW_CORE_MAX; j++) {
			subsys[i].submodule_offset[j] = 0xffffffff;
			subsys[i].submodule_iosize[j] = 0;
			subsys[i].submodule_hwregs[j] = NULL;
		}
	}

	for (i = 0; i < ARRAY_SIZE(core_array); i++) {
		if (!subsys[core_array[i].subsys].base_addr)
			continue;

		subsys[core_array[i].subsys].submodule_offset[core_array[i].core_type] =
			core_array[i].offset;
		subsys[core_array[i].subsys].submodule_iosize[core_array[i].core_type] =
			core_array[i].iosize;
		if (subsys[core_array[i].subsys].irq != -1 &&
			core_array[i].irq != -1) {
			if (subsys[core_array[i].subsys].irq !=
				core_array[i].irq)
				pr_info("hantrodec: hw core type %d irq %d != subsystem irq %d\n",
					core_array[i].core_type,
					core_array[i].irq,
					subsys[core_array[i].subsys].irq);
			else
				subsys[core_array[i].subsys].irq =
					core_array[i].irq;
		}
	}

	memset(multicorebase, 0, sizeof(multicorebase[0]) * HXDEC_MAX_CORES);
	for (i = 0; i < num; i++) {
		multicorebase[i] = subsys[i].base_addr +
					subsys[i].submodule_offset[HW_VC8000D];
		irq[i] = subsys[i].irq;
		iosize[i] = subsys[i].submodule_iosize[HW_VC8000D];
		pr_info("hantrodec: [%d] multicorebase 0x%08lx, iosize %d\n",
			i, multicorebase[i], iosize[i]);
	}
}

int GetSubsysCoreArrayFromDeviceTree(struct subsys_config *subsys,
				     struct platform_device *pdev)
{
	int num = ARRAY_SIZE(subsys_array);
	int i, j;
	struct resource *res;
	struct device *dev = &pdev->dev;

	memset(subsys, 0, sizeof(subsys[0]) * MAX_SUBSYS_NUM);
	for (i = 0; i < num; i++) {
		subsys[i].irq = -1;
		for (j = 0; j < HW_CORE_MAX; j++) {
			subsys[i].submodule_offset[j] = 0xffffffff;
			subsys[i].submodule_iosize[j] = 0;
			subsys[i].submodule_hwregs[j] = NULL;
		}
	}

	/* irq */
	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "DEC");
	if (res != NULL) {
		subsys[0].irq = res->start;
		irq[0] = subsys[0].irq;
	}

	/* reg */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "base");
	if (res != NULL)
		subsys[0].base_addr = (u32)res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dec");
	if (res != NULL) {
		subsys[0].submodule_offset[HW_VC8000D] = (u32)res->start;
		subsys[0].submodule_iosize[HW_VC8000D] = resource_size(res);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "l2cache");
	if (res != NULL) {
		subsys[0].submodule_offset[HW_L2CACHE] = (u32)res->start;
		subsys[0].submodule_iosize[HW_L2CACHE] = resource_size(res);
	}

#ifdef SUPPORT_SFBC
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sfbcenc");
	if (res != NULL) {
		subsys[0].submodule_offset[HW_SFBC_ENC] = (u32)res->start;
		subsys[0].submodule_iosize[HW_SFBC_ENC] = resource_size(res);
	}
#endif

	memset(multicorebase, 0, sizeof(multicorebase[0]) * HXDEC_MAX_CORES);
	for (i = 0; i < num; i++) {
		multicorebase[i] = subsys[i].base_addr +
					subsys[i].submodule_offset[HW_VC8000D];
		irq[i] = subsys[i].irq;
		iosize[i] = subsys[i].submodule_iosize[HW_VC8000D];
		pr_info("hantrodec: [%d] multicorebase 0x%08lx, iosize %d\n",
			i, multicorebase[i], iosize[i]);
	}

	/* clock & reset */
	hantrodec_data.clk_axi = of_clk_get_by_name(pdev->dev.of_node, "axi");
	if (IS_ERR(hantrodec_data.clk_axi))
		return PTR_ERR(hantrodec_data.clk_axi);

	hantrodec_data.clk_core = of_clk_get_by_name(pdev->dev.of_node, "core");
	if (IS_ERR(hantrodec_data.clk_core))
		return PTR_ERR(hantrodec_data.clk_core);

	hantrodec_data.clk_apb = of_clk_get_by_name(pdev->dev.of_node, "apb");
	if (IS_ERR(hantrodec_data.clk_apb))
		return PTR_ERR(hantrodec_data.clk_apb);

	hantrodec_data.rst_axi = devm_reset_control_get_optional_shared(dev, "axi");
	if (PTR_ERR(hantrodec_data.rst_axi) == -EPROBE_DEFER)
		return PTR_ERR(hantrodec_data.rst_axi);

	hantrodec_data.rst_core =
		devm_reset_control_get_optional_shared(dev, "core");
	if (PTR_ERR(hantrodec_data.rst_core) == -EPROBE_DEFER)
		return PTR_ERR(hantrodec_data.rst_core);

	hantrodec_data.rst_apb =
		devm_reset_control_get_optional_shared(dev, "apb");
	if (PTR_ERR(hantrodec_data.rst_apb) == -EPROBE_DEFER)
		return PTR_ERR(hantrodec_data.rst_apb);

	return 0;
}

module_param(base_port, ulong, 0644);
module_param_array(irq, int, &elements, 0644);
module_param_array(multicorebase, ulong, &elements, 0644);
module_param(reg_access_opt, uint, 0644);

static unsigned int hantrodec_major; /* dynamic allocation */

struct core_cfg {
	u32 cfg[HXDEC_MAX_CORES];              /* supported format */
	u32 cfg_backup[HXDEC_MAX_CORES];       /* back up of cfg */
	int its_main_core_id[HXDEC_MAX_CORES]; /* indicate if main core exist */
	int its_aux_core_id[HXDEC_MAX_CORES];  /* indicate if aux core exist */
};

static int ReserveIO(void);
static void ReleaseIO(void);

static void ResetAsic(struct hantrodec *dev);

#ifdef HANTRODEC_DEBUG
static void dump_regs(struct hantrodec *dev);
#endif

/* IRQ handler */
static irqreturn_t hantrodec_isr(int irq, void *dev_id);

static u32 dec_regs[HXDEC_MAX_CORES][DEC_IO_SIZE_MAX/4];
/* shadow_regs used to compare whether it's necessary to write to registers */
static u32 shadow_dec_regs[HXDEC_MAX_CORES][DEC_IO_SIZE_MAX/4];

struct semaphore dec_core_sem;
struct semaphore pp_core_sem;

static int dec_irq;
static int pp_irq;

atomic_t irq_rx = ATOMIC_INIT(0);
atomic_t irq_tx = ATOMIC_INIT(0);

static struct file *dec_owner[HXDEC_MAX_CORES];
static struct file *pp_owner[HXDEC_MAX_CORES];
static int CoreHasFormat(const u32 *cfg, int core, u32 format);

DEFINE_SPINLOCK(dec_owner_lock);

DECLARE_WAIT_QUEUE_HEAD(dec_wait_queue);
DECLARE_WAIT_QUEUE_HEAD(pp_wait_queue);
DECLARE_WAIT_QUEUE_HEAD(dec_hw_queue);

#define DWL_CLIENT_TYPE_H264_DEC	1U
#define DWL_CLIENT_TYPE_MPEG4_DEC	2U
#define DWL_CLIENT_TYPE_JPEG_DEC	3U
#define DWL_CLIENT_TYPE_PP		4U
#define DWL_CLIENT_TYPE_VC1_DEC		5U
#define DWL_CLIENT_TYPE_MPEG2_DEC	6U
#define DWL_CLIENT_TYPE_VP6_DEC		7U
#define DWL_CLIENT_TYPE_AVS_DEC		8U
#define DWL_CLIENT_TYPE_RV_DEC		9U
#define DWL_CLIENT_TYPE_VP8_DEC		10U
#define DWL_CLIENT_TYPE_VP9_DEC		11U
#define DWL_CLIENT_TYPE_HEVC_DEC	12U
#define DWL_CLIENT_TYPE_ST_PP		14U
#define DWL_CLIENT_TYPE_H264_MAIN10	15U
#define DWL_CLIENT_TYPE_AVS2_DEC	16U
#define DWL_CLIENT_TYPE_AV1_DEC		17U

#define BIGOCEANDEC_CFG			1
#define BIGOCEANDEC_AV1_E		5

static struct core_cfg config;

/* enable/disable clock */
static void hantrodec_deassert_clk(struct device *dev)
{
	reset_control_deassert(hantrodec_data.rst_axi);
	reset_control_deassert(hantrodec_data.rst_core);
	reset_control_deassert(hantrodec_data.rst_apb);
}

static void hantrodec_enable_clk(struct device *dev)
{
	clk_prepare_enable(hantrodec_data.clk_axi);
	clk_prepare_enable(hantrodec_data.clk_core);
	clk_prepare_enable(hantrodec_data.clk_apb);
}

static void hantrodec_assert_clk(struct device *dev)
{
	reset_control_assert(hantrodec_data.rst_axi);
	reset_control_assert(hantrodec_data.rst_core);
	reset_control_assert(hantrodec_data.rst_apb);
}

static void hantrodec_disable_clk(struct device *dev)
{
	clk_disable_unprepare(hantrodec_data.clk_axi);
	clk_disable_unprepare(hantrodec_data.clk_core);
	clk_disable_unprepare(hantrodec_data.clk_apb);
}

static void wait_delay(unsigned int delay)
{
	if (delay > 0) {
		ktime_t dl = ktime_set((delay / MSEC_PER_SEC),
					(delay % MSEC_PER_SEC) *
					NSEC_PER_MSEC);
		__set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_hrtimeout(&dl, HRTIMER_MODE_REL);
	}
}

static void ReadCoreConfig(struct hantrodec *dev)
{
	int c;
	u32 reg, tmp, mask;

	memset(config.cfg, 0, sizeof(config.cfg));

	for (c = 0; c < dev->cores; c++) {
		/* Decoder configuration */
		if (IS_G1(dev->hw_id[c])) {
			reg = ioread32((void *)(dev->hwregs[c][HW_VC8000D] +
					HANTRODEC_SYNTH_CFG * 4));

			tmp = (reg >> DWL_H264_E) & 0x3U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has H264\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_H264_DEC : 0;

			tmp = (reg >> DWL_JPEG_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has JPEG\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_JPEG_DEC : 0;

			tmp = (reg >> DWL_HJPEG_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has HJPEG\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_JPEG_DEC : 0;

			tmp = (reg >> DWL_MPEG4_E) & 0x3U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has MPEG4\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_MPEG4_DEC : 0;

			tmp = (reg >> DWL_VC1_E) & 0x3U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has VC1\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_VC1_DEC : 0;

			tmp = (reg >> DWL_MPEG2_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has MPEG2\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_MPEG2_DEC : 0;

			tmp = (reg >> DWL_VP6_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has VP6\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_VP6_DEC : 0;

			reg = ioread32((void *)(dev->hwregs[c][HW_VC8000D] +
					HANTRODEC_SYNTH_CFG_2 * 4));

			/* VP7 and WEBP is part of VP8 */
			mask =  (1 << DWL_VP8_E) |
				(1 << DWL_VP7_E) | (1 << DWL_WEBP_E);
			tmp = (reg & mask);
			if (tmp & (1 << DWL_VP8_E))
				pr_debug("hantrodec: core[%d] has VP8\n", c);
			if (tmp & (1 << DWL_VP7_E))
				pr_debug("hantrodec: core[%d] has VP7\n", c);
			if (tmp & (1 << DWL_WEBP_E))
				pr_debug("hantrodec: core[%d] has WebP\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_VP8_DEC : 0;

			tmp = (reg >> DWL_AVS_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has AVS\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_AVS_DEC : 0;

			tmp = (reg >> DWL_RV_E) & 0x03U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has RV\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_RV_DEC : 0;

			/* Post-processor configuration */
			reg = ioread32((void *)(dev->hwregs[c][HW_VC8000D] +
					HANTROPP_SYNTH_CFG * 4));

			tmp = (reg >> DWL_G1_PP_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has PP\n", c);
			config.cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_PP : 0;
		} else if ((IS_G2(dev->hw_id[c]))) {
			reg = ioread32((void *)(dev->hwregs[c][HW_VC8000D] +
					HANTRODEC_CFG_STAT * 4));

			tmp = (reg >> DWL_G2_HEVC_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has HEVC\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_HEVC_DEC : 0;

			tmp = (reg >> DWL_G2_VP9_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has VP9\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_VP9_DEC : 0;

			/* Post-processor configuration */
			reg = ioread32((void *)(dev->hwregs[c][HW_VC8000D] +
					HANTRODECPP_SYNTH_CFG * 4));

			tmp = (reg >> DWL_G2_PP_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has PP\n", c);
			config.cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_PP : 0;
		} else if ((IS_VC8000D(dev->hw_id[c])) &&
				config.its_main_core_id[c] < 0) {
			reg = ioread32((void *)(dev->hwregs[c][HW_VC8000D] +
					HANTRODEC_SYNTH_CFG * 4));

			tmp = (reg >> DWL_H264_E) & 0x3U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has H264\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_H264_DEC : 0;

			tmp = (reg >> DWL_H264HIGH10_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has H264HIGH10\n",
					c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_H264_DEC : 0;

			tmp = (reg >> DWL_AVS2_E) & 0x03U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has AVS2\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_AVS2_DEC : 0;

			tmp = (reg >> DWL_JPEG_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has JPEG\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_JPEG_DEC : 0;

			tmp = (reg >> DWL_HJPEG_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has HJPEG\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_JPEG_DEC : 0;

			tmp = (reg >> DWL_MPEG4_E) & 0x3U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has MPEG4\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_MPEG4_DEC : 0;

			tmp = (reg >> DWL_VC1_E) & 0x3U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has VC1\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_VC1_DEC : 0;

			tmp = (reg >> DWL_MPEG2_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has MPEG2\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_MPEG2_DEC : 0;

			tmp = (reg >> DWL_VP6_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has VP6\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_VP6_DEC : 0;

			reg = ioread32((void *)(dev->hwregs[c][HW_VC8000D] +
					HANTRODEC_SYNTH_CFG_2 * 4));

			/* VP7 and WEBP is part of VP8 */
			mask =  (1 << DWL_VP8_E) |
				(1 << DWL_VP7_E) | (1 << DWL_WEBP_E);
			tmp = (reg & mask);
			if (tmp & (1 << DWL_VP8_E))
				pr_debug("hantrodec: core[%d] has VP8\n", c);
			if (tmp & (1 << DWL_VP7_E))
				pr_debug("hantrodec: core[%d] has VP7\n", c);
			if (tmp & (1 << DWL_WEBP_E))
				pr_debug("hantrodec: core[%d] has WebP\n", c);
			config.cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_VP8_DEC : 0;

			tmp = (reg >> DWL_AVS_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has AVS\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_AVS_DEC : 0;

			tmp = (reg >> DWL_RV_E) & 0x03U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has RV\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_RV_DEC : 0;

			reg = ioread32((void *)(dev->hwregs[c][HW_VC8000D] +
					HANTRODEC_SYNTH_CFG_3 * 4));

			tmp = (reg >> DWL_HEVC_E) & 0x07U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has HEVC\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_HEVC_DEC : 0;

			tmp = (reg >> DWL_VP9_E) & 0x07U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has VP9\n", c);
			config.cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_VP9_DEC : 0;

			/* Post-processor configuration */
			reg = ioread32((void *)(dev->hwregs[c][HW_VC8000D] +
					HANTRODECPP_CFG_STAT * 4));

			tmp = (reg >> DWL_PP_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has PP\n", c);
			config.cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_PP : 0;

			config.cfg[c] |= 1 << DWL_CLIENT_TYPE_ST_PP;

			if (config.its_aux_core_id[c] >= 0) {
				/* set main_core_id and aux_core_id */
				reg = ioread32((void *)(dev->hwregs[c][HW_VC8000D] +
						HANTRODEC_SYNTH_CFG_2 * 4));

				tmp = (reg >> DWL_H264_PIPELINE_E) & 0x01U;
				if (tmp)
					pr_debug("dec: core[%d] has pipeline H264\n",
						c);
				config.cfg[config.its_aux_core_id[c]] |=
					tmp ? 1 << DWL_CLIENT_TYPE_H264_DEC : 0;

				tmp = (reg >> DWL_JPEG_PIPELINE_E) & 0x01U;
				if (tmp)
					pr_debug("dec: core[%d] has pipeline JPEG\n",
						c);
				config.cfg[config.its_aux_core_id[c]] |=
					tmp ? 1 << DWL_CLIENT_TYPE_JPEG_DEC : 0;
			}
		} else if (IS_BIGOCEAN(dev->hw_id[c])) {
			reg = ioread32((void *)(dev->hwregs[c][HW_VC8000D] +
					BIGOCEANDEC_CFG * 4));

			tmp = (reg >> BIGOCEANDEC_AV1_E) & 0x01U;
			if (tmp)
				pr_debug("hantrodec: core[%d] has AV1\n", c);
			config.cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_AV1_DEC : 0;
		}
	}
	memcpy(config.cfg_backup, config.cfg, sizeof(config.cfg));
}

static int CoreHasFormat(const u32 *cfg, int core, u32 format)
{
	return (cfg[core] & (1 << format)) ? 1 : 0;
}

int GetDecCore(long core, struct hantrodec *dev,
		struct file *filp, unsigned long format)
{
	int success = 0;
	unsigned long flags;

	spin_lock_irqsave(&dec_owner_lock, flags);
	if (CoreHasFormat(config.cfg, core, format) &&
		dec_owner[core] == NULL) {
		dec_owner[core] = filp;
		success = 1;

		/*
		 * If one main core takes one format
		 * which doesn't supported by aux core,
		 * set aux core's cfg to none video format support
		 */
		if (config.its_aux_core_id[core] >= 0 &&
			!CoreHasFormat(config.cfg,
				config.its_aux_core_id[core], format)) {
			config.cfg[config.its_aux_core_id[core]] = 0;
		}
		/*
		 * If one aux core takes one format,
		 * set main core's cfg to aux core supported video format
		 */
		else if (config.its_main_core_id[core] >= 0)
			config.cfg[config.its_main_core_id[core]] =
				config.cfg[core];
	}

	spin_unlock_irqrestore(&dec_owner_lock, flags);

	return success;
}

int GetDecCoreAny(long *core, struct hantrodec *dev, struct file *filp,
		  unsigned long format)
{
	int success = 0;
	long c;

	*core = -1;

	for (c = 0; c < dev->cores; c++) {
		/* a free core that has format */
		if (GetDecCore(c, dev, filp, format)) {
			success = 1;
			*core = c;
			break;
		}
	}

	return success;
}

int GetDecCoreID(struct hantrodec *dev, struct file *filp,
		 unsigned long format)
{
	long c;
	unsigned long flags;
	int core_id = -1;

	for (c = 0; c < dev->cores; c++) {
		/* a core that has format */
		spin_lock_irqsave(&dec_owner_lock, flags);
		if (CoreHasFormat(config.cfg, c, format)) {
			core_id = c;
			spin_unlock_irqrestore(&dec_owner_lock, flags);
			break;
		}
		spin_unlock_irqrestore(&dec_owner_lock, flags);
	}
	return core_id;
}

long ReserveDecoder(struct hantrodec *dev,
		    struct file *filp, unsigned long format)
{
	long core = -1;

	/* reserve a core */
	if (down_interruptible(&dec_core_sem))
		return -ERESTARTSYS;

	/* lock a core that has specific format */
	if (wait_event_interruptible(dec_hw_queue,
			GetDecCoreAny(&core, dev, filp, format) != 0))
		return -ERESTARTSYS;

	return core;
}

void ReleaseDecoder(struct hantrodec *dev, long core)
{
	u32 status;
	u32 cache_status;
	u32 shaper_status;
	u32 shaper_irq;
	int max_wait_time = 1000; /* 1s */
	unsigned long flags;

	pr_debug("%s %ld\n", __func__, core);

	status = ioread32((void *)(dev->hwregs[core][HW_VC8000D] +
			HANTRODEC_IRQ_STAT_DEC_OFF));
	/* make sure HW is disabled */
	if (status & HANTRODEC_DEC_E) {
		pr_debug("hantrodec: DEC[%li] still enabled -> reset\n", core);

		/* abort decoder */
		status |= HANTRODEC_DEC_ABORT | HANTRODEC_DEC_IRQ_DISABLE;
		iowrite32(status, (void *)(dev->hwregs[core][HW_VC8000D] +
					HANTRODEC_IRQ_STAT_DEC_OFF));
	}

	cache_status = ioread32((void *)(dev->hwregs[core][HW_L2CACHE] +
					HANTRODEC_CACHE_STAT * 4));
	pr_debug("hantrodec: read cache_status = %x\n", cache_status);
	if (cache_status & HANTRODEC_CACHE_E) {
		pr_debug("hantrodec: cache still enabled -> reset\n");
		iowrite32(0, (void *)(dev->hwregs[core][HW_L2CACHE] +
				HANTRODEC_CACHE_STAT * 4));
		iowrite32(0, (void *)(dev->hwregs[core][HW_L2CACHE] +
				HANTRODEC_CACHE_CFG * 4));
	}

	shaper_status = ioread32((void *)(dev->hwregs[core][HW_L2CACHE] +
				HANTRODEC_SHAPER_STAT * 4));
	pr_debug("hantrodec: read shaper_status = %x\n", shaper_status);
	if (shaper_status & HANTRODEC_SHAPER_E) {
		pr_debug("hantrodec: shaper still enabled -> reset\n");
		iowrite32(0, (void *)(dev->hwregs[core][HW_L2CACHE] +
				HANTRODEC_SHAPER_STAT * 4));

		do {
			shaper_irq = ioread32((void *)(dev->hwregs[core][HW_L2CACHE] +
						HANTRODEC_SHAPER_IRQ_CFG * 4));
			if (shaper_irq & HANTRODEC_SHAPER_IRQ_E) {
				pr_debug("hantrodec: flush shaper done!\n");
				break;
			}
			wait_delay(1);
			max_wait_time--;
		} while (max_wait_time > 0);

		iowrite32(HANTRODEC_SHAPER_IRQ_CLEAN,
			(void *)(dev->hwregs[core][HW_L2CACHE] +
				HANTRODEC_SHAPER_IRQ_CFG * 4));
		pr_debug("hantrodec: clear shaper interrupt bit.\n");
	}

	spin_lock_irqsave(&dec_owner_lock, flags);

	/* If aux core released, revert main core's config back */
	if (config.its_main_core_id[core] >= 0)
		config.cfg[config.its_main_core_id[core]] =
			config.cfg_backup[config.its_main_core_id[core]];

	/* If main core released, revert aux core's config back */
	if (config.its_aux_core_id[core] >= 0)
		config.cfg[config.its_aux_core_id[core]] =
			config.cfg_backup[config.its_aux_core_id[core]];

	dec_owner[core] = NULL;

	spin_unlock_irqrestore(&dec_owner_lock, flags);

	up(&dec_core_sem);

	wake_up_interruptible_all(&dec_hw_queue);
}

long ReservePostProcessor(struct hantrodec *dev, struct file *filp)
{
	unsigned long flags;
	long core = 0;

	/* single core PP only */
	if (down_interruptible(&pp_core_sem))
		return -ERESTARTSYS;

	spin_lock_irqsave(&dec_owner_lock, flags);
	pp_owner[core] = filp;
	spin_unlock_irqrestore(&dec_owner_lock, flags);

	return core;
}

void ReleasePostProcessor(struct hantrodec *dev, long core)
{
	unsigned long flags;

	u32 status = ioread32((void *)(dev->hwregs[core][HW_VC8000D] +
				HANTRO_IRQ_STAT_PP_OFF));

	/* make sure HW is disabled */
	if (status & HANTRO_PP_E) {
		pr_debug("hantrodec: PP[%li] still enabled -> reset\n", core);

		/* disable IRQ */
		status |= HANTRO_PP_IRQ_DISABLE;

		/* disable postprocessor */
		status &= (~HANTRO_PP_E);
		iowrite32(0x10, (void *)(dev->hwregs[core][HW_VC8000D] +
				HANTRO_IRQ_STAT_PP_OFF));
	}

	spin_lock_irqsave(&dec_owner_lock, flags);
	pp_owner[core] = NULL;
	spin_unlock_irqrestore(&dec_owner_lock, flags);

	up(&pp_core_sem);
}

long ReserveDecPp(struct hantrodec *dev,
		  struct file *filp, unsigned long format)
{
	/* reserve core 0, DEC+PP for pipeline */
	unsigned long flags;
	long core = 0;

	/* check that core has the requested dec format */
	if (!CoreHasFormat(config.cfg, core, format))
		return -EFAULT;

	/* check that core has PP */
	if (!CoreHasFormat(config.cfg, core, DWL_CLIENT_TYPE_PP))
		return -EFAULT;

	/* reserve a core */
	if (down_interruptible(&dec_core_sem))
		return -ERESTARTSYS;

	/* wait until the core is available */
	if (wait_event_interruptible(dec_hw_queue,
		GetDecCore(core, dev, filp, format) != 0)) {
		up(&dec_core_sem);
		return -ERESTARTSYS;
	}

	if (down_interruptible(&pp_core_sem)) {
		ReleaseDecoder(dev, core);
		return -ERESTARTSYS;
	}

	spin_lock_irqsave(&dec_owner_lock, flags);
	pp_owner[core] = filp;
	spin_unlock_irqrestore(&dec_owner_lock, flags);

	return core;
}

#ifdef HANTRODEC_DEBUG
static u32 flush_count; /* times of calling of DecFlushRegs */
static u32 flush_regs;  /* total number of registers flushed */
#endif

long DecFlushRegs(struct hantrodec *dev, struct core_desc *core)
{
	long ret = 0, i;
#ifdef HANTRODEC_DEBUG
	int reg_wr = 2;
#endif
	u32 id = core->id;
	u32 type = core->type;

	pr_debug("%s\n", __func__);
	pr_debug("id = %d, type = %d, size = %d, reg_id = %d\n",
			core->id, core->type, core->size, core->reg_id);

	if (id >= MAX_SUBSYS_NUM ||
			!vpu_subsys[id].base_addr ||
			core->type >= HW_CORE_MAX ||
			!vpu_subsys[id].submodule_hwregs[core->type])
		return -EINVAL;

	pr_debug("submodule_iosize = %d\n",
		vpu_subsys[id].submodule_iosize[type]);
	pr_debug("reg count = %d\n", reg_count[id]);

	ret = copy_from_user(dec_regs[id], (void *)core->regs_addr,
				vpu_subsys[id].submodule_iosize[type]);
	if (ret) {
		pr_err("copy_from_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	if (core->type == HW_VC8000D) {
		/* write all regs but the status reg[1] to hardware */
		if (reg_access_opt) {
			for (i = 3; i < reg_count[id]; i++) {
				/* check whether register value is updated. */
				if (dec_regs[id][i] != shadow_dec_regs[id][i]) {
					iowrite32(dec_regs[id][i],
						(void *)(dev->hwregs[id][type] +
						i * 4));
					shadow_dec_regs[id][i] =
							dec_regs[id][i];
#ifdef HANTRODEC_DEBUG
					reg_wr++;
#endif
				}
			}
		} else {
			for (i = 3; i < reg_count[id]; i++) {
				iowrite32(dec_regs[id][i],
					(void *)(dev->hwregs[id][type] +
					i * 4));
#ifdef VALIDATE_REGS_WRITE
				if (dec_regs[id][i] !=
					ioread32((void *)(dev->hwregs[id][type] +
						i * 4)))
					pr_debug("hantrodec: swreg[%ld]: read %08x != write %08x *\n",
						i,
						ioread32((void *)(dev->hwregs[id][type] + i * 4)),
						dec_regs[id][i]);
#endif
			}
#ifdef HANTRODEC_DEBUG
			reg_wr = reg_count[id] - 1;
#endif
		}

		/* write swreg2 for AV1, in which bit0 is the start bit */
		iowrite32(dec_regs[id][2], (void *)(dev->hwregs[id][type] + 8));
		shadow_dec_regs[id][2] = dec_regs[id][2];

		/* write the status register, which may start the decoder */
		iowrite32(dec_regs[id][1], (void *)(dev->hwregs[id][type] + 4));
		shadow_dec_regs[id][1] = dec_regs[id][1];

#ifdef HANTRODEC_DEBUG
		flush_count++;
		flush_regs += reg_wr;

		pr_debug("flushed registers on core %d\n", id);
		pr_debug("%d flushed %d/%d registers (dec_mode = %d, avg %d regs per flush)\n",
			flush_count, reg_wr, flush_regs,
			dec_regs[id][3]>>27, flush_regs/flush_count);
#endif
	} else {
		/* write all regs but the status reg[1] to hardware */
		for (i = 0; i < vpu_subsys[id].submodule_iosize[type] / 4;
			i++) {
			iowrite32(dec_regs[id][i],
				(void *)(dev->hwregs[id][type] + i * 4));
#ifdef VALIDATE_REGS_WRITE
			if (dec_regs[id][i] !=
				ioread32((void *)(dev->hwregs[id][type] +
					i * 4)))
				pr_debug("swreg[%ld]: read %08x != write %08x *\n",
					i,
					ioread32((void *)(dev->hwregs[id][type] + i * 4)),
					dec_regs[id][i]);
#endif
		}
	}

	return 0;
}

long DecWriteRegs(struct hantrodec *dev, struct core_desc *core)
{
	long ret = 0;
	u32 i = core->reg_id;
	u32 id = core->id;

	pr_debug("%s\n", __func__);
	pr_debug("id = %d, type = %d, size = %d, reg_id = %d\n",
		  core->id, core->type, core->size, core->reg_id);

	if (id >= MAX_SUBSYS_NUM ||
			!vpu_subsys[id].base_addr ||
			core->type >= HW_CORE_MAX ||
			!vpu_subsys[id].submodule_hwregs[core->type] ||
			(core->size & 0x3) ||
			core->reg_id * 4 + core->size >
				vpu_subsys[id].submodule_iosize[core->type])
		return -EINVAL;

	ret = copy_from_user(dec_regs[id], (void *)core->regs_addr, core->size);
	if (ret) {
		pr_err("copy_from_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	for (i = core->reg_id; i < core->reg_id + core->size / 4; i++) {
		pr_debug("write %08x to reg[%d] core %d\n",
			dec_regs[id][i-core->reg_id], i, id);
		iowrite32(dec_regs[id][i-core->reg_id],
			(void *)(dev->hwregs[id][core->type] + i * 4));
		if (core->type == HW_VC8000D)
			shadow_dec_regs[id][i] = dec_regs[id][i-core->reg_id];
	}
	return 0;
}

long DecReadRegs(struct hantrodec *dev, struct core_desc *core)
{
	long ret;
	u32 id = core->id;
	u32 i = core->reg_id;

	PDEBUG("%s\n", __func__);
	PDEBUG("id = %d, type = %d, size = %d, reg_id = %d\n",
		core->id, core->type, core->size, core->reg_id);

	if (id >= MAX_SUBSYS_NUM ||
			!vpu_subsys[id].base_addr ||
			core->type >= HW_CORE_MAX ||
			!vpu_subsys[id].submodule_hwregs[core->type] ||
			(core->size & 0x3) ||
			core->reg_id * 4 + core->size >
				vpu_subsys[id].submodule_iosize[core->type])
		return -EINVAL;

	/* read specific registers from hardware */
	for (i = core->reg_id; i < core->reg_id + core->size/4; i++) {
		dec_regs[id][i-core->reg_id] =
			ioread32((void *)(dev->hwregs[id][core->type] + i * 4));
		PDEBUG("hantrodec: read %08x from reg[%d] core %d\n",
			dec_regs[id][i-core->reg_id], i, id);
		if (core->type == HW_VC8000D)
			shadow_dec_regs[id][i] = dec_regs[id][i];
	}

	/* put registers to user space */
	ret = copy_to_user((void *)core->regs_addr, dec_regs[id], core->size);
	if (ret) {
		pr_err("copy_to_user failed, returned %li\n", ret);
		return -EFAULT;
	}
	return 0;
}

long DecRefreshRegs(struct hantrodec *dev, struct core_desc *core)
{
	long ret, i;
	u32 id = core->id;

	PDEBUG("%s\n", __func__);
	PDEBUG("id = %d, type = %d, size = %d, reg_id = %d\n",
		core->id, core->type, core->size, core->reg_id);
	if (id >= MAX_SUBSYS_NUM ||
			!vpu_subsys[id].base_addr ||
			core->type >= HW_CORE_MAX ||
			!vpu_subsys[id].submodule_hwregs[core->type])
		return -EINVAL;

	PDEBUG("submodule_iosize = %d\n",
		vpu_subsys[id].submodule_iosize[core->type]);

	if (!reg_access_opt) {
		for (i = 0; i < vpu_subsys[id].submodule_iosize[core->type] / 4;
			i++)
			dec_regs[id][i] =
				ioread32((void *)(dev->hwregs[id][core->type] +
					i * 4));
	} else {
		/* only need to read swreg1,62(?),63,168,169 */
#define REFRESH_REG(idx)						\
	do {								\
		i = (idx);						\
		shadow_dec_regs[id][i] = dec_regs[id][i] =		\
		ioread32((void *)(dev->hwregs[id][core->type] + i * 4));\
	} while (0)

		REFRESH_REG(0);
		REFRESH_REG(1);
		REFRESH_REG(62);
		REFRESH_REG(63);
		REFRESH_REG(168);
		REFRESH_REG(169);
#undef REFRESH_REG
	}

	ret = copy_to_user((void *)core->regs_addr, dec_regs[id],
			vpu_subsys[id].submodule_iosize[core->type]);
	if (ret) {
		pr_err("copy_to_user failed, returned %li\n", ret);
		return -EFAULT;
	}
	return 0;
}

static int CheckDecIrq(struct hantrodec *dev, int id)
{
	unsigned long flags;
	int rdy = 0;
	const u32 irq_mask = (1 << id);

	spin_lock_irqsave(&dec_owner_lock, flags);

	if (dec_irq & irq_mask) {
		/* reset the wait condition(s) */
		dec_irq &= ~irq_mask;
		rdy = 1;
	}

	spin_unlock_irqrestore(&dec_owner_lock, flags);

	return rdy;
}

long WaitDecReadyAndRefreshRegs(struct hantrodec *dev, struct core_desc *core)
{
	u32 id = core->id;
	long ret;

	PDEBUG("wait_event_interruptible DEC[%d]\n", id);
	ret = wait_event_interruptible(dec_wait_queue, CheckDecIrq(dev, id));
	if (ret) {
		PDEBUG("DEC[%d]  wait_event_interruptible interrupted\n", id);
		return -ERESTARTSYS;
	}
	atomic_inc(&irq_tx);

	/* refresh registers */
	return DecRefreshRegs(dev, core);
}

static int CheckCoreIrq(struct hantrodec *dev,
			const struct file *filp, int *id)
{
	unsigned long flags;
	int rdy = 0, n = 0;

	do {
		u32 irq_mask = (1 << n);

		spin_lock_irqsave(&dec_owner_lock, flags);

		if (dec_irq & irq_mask) {
			if (dec_owner[n] == filp) {
				/*
				 * we have an IRQ for our client
				 * reset the wait condition(s)
				 */
				dec_irq &= ~irq_mask;

				/* signal ready core no. for our client */
				*id = n;
				rdy = 1;

				spin_unlock_irqrestore(&dec_owner_lock, flags);
				break;
			} else if (dec_owner[n] == NULL) {
				/* zombie IRQ */
				pr_debug("IRQ on core[%d], but no owner!\n", n);

				/* reset the wait condition(s) */
				dec_irq &= ~irq_mask;
			}
		}

		spin_unlock_irqrestore(&dec_owner_lock, flags);

		n++; /* next core */
	} while (n < dev->cores);

	return rdy;
}

long WaitCoreReady(struct hantrodec *dev, const struct file *filp, int *id)
{
	long ret;

	PDEBUG("wait_event_interruptible CORE\n");
	ret = wait_event_interruptible(dec_wait_queue,
					CheckCoreIrq(dev, filp, id));
	if (ret) {
		PDEBUG("CORE[%d] wait event interrupted with 0x%lx\n",
			*id, ret);
		return -ERESTARTSYS;
	}
	atomic_inc(&irq_tx);

	return 0;
}

/*
 * --------------------------------------------------------------------------
 * Function name   : hantrodec_ioctl
 * Description     : communication method to/from the user space
 *
 * Return type     : long
 * --------------------------------------------------------------------------
 */
static long hantrodec_ioctl(struct file *filp, unsigned int cmd,
			    unsigned long arg)
{
	int err = 0;
	long tmp;

	PDEBUG("ioctl cmd 0x%08x\n", cmd);
	/*
	 * extract the type and number bitfields, and don't decode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 */
	if (_IOC_TYPE(cmd) != HANTRODEC_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_TYPE(cmd) == HANTRODEC_IOC_MAGIC &&
			_IOC_NR(cmd) > HANTRODEC_IOC_MAXNR)
		return -ENOTTY;

	err = !access_ok((void *) arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(HANTRODEC_IOC_CLI): {
		__u32 id;

		__get_user(id, (__u32 *)arg);

		if (id >= hantrodec_data.cores)
			return -EFAULT;
		disable_irq(hantrodec_data.irq[id]);
		break;
	}
	case _IOC_NR(HANTRODEC_IOC_STI): {
		__u32 id;

		__get_user(id, (__u32 *)arg);

		if (id >= hantrodec_data.cores)
			return -EFAULT;

		enable_irq(hantrodec_data.irq[id]);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCGHWOFFSET): {
		__u32 id;

		__get_user(id, (__u32 *)arg);

		if (id >= hantrodec_data.cores)
			return -EFAULT;

		__put_user(multicorebase_actual[id], (unsigned long *) arg);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCGHWIOSIZE): {
		struct regsize_desc core;

		/* get registers from user space */
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct regsize_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		if (core.id >= MAX_SUBSYS_NUM)  /* hantrodec_data.cores */
			return -EFAULT;

		if (core.type == HW_SHAPER) {
			u32 asic_id;
			/* Shaper is configured with l2cache. */
			if (vpu_subsys[core.id].submodule_hwregs[HW_L2CACHE]) {
				asic_id = ioread32((void *)vpu_subsys[core.id].submodule_hwregs[HW_L2CACHE]);
				switch ((asic_id >> 16) & 0x3) {
				case 1: /* cache only */
					core.size = 0;
					break;
				case 0: /* cache + shaper */
				case 2: /* shaper only */
					core.size = vpu_subsys[core.id].submodule_iosize[HW_L2CACHE];
					break;
				default:
					return -EFAULT;
				}
			} else
				core.size = 0;
		} else
			core.size =
				vpu_subsys[core.id].submodule_iosize[core.type];

		tmp = copy_to_user((u32 *) arg,
				&core, sizeof(struct regsize_desc));
		if (tmp) {
			pr_err("copy_to_user failed, returned %li\n", tmp);
			return -EFAULT;
		}
		break;
	}
	case _IOC_NR(HANTRODEC_IOC_MC_OFFSETS): {
		tmp = copy_to_user((unsigned long *) arg,
				multicorebase_actual,
				sizeof(multicorebase_actual));
		if (err) {
			pr_err("copy_to_user failed, returned %li\n", tmp);
			return -EFAULT;
		}
		break;
	}
	case _IOC_NR(HANTRODEC_IOC_MC_CORES): {
		__put_user(hantrodec_data.cores, (unsigned int *) arg);
		PDEBUG("hantrodec_data.cores=%d\n", hantrodec_data.cores);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCS_DEC_PUSH_REG): {
		struct core_desc core;

		/* get registers from user space */
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		return DecFlushRegs(&hantrodec_data, &core);
	}
	case _IOC_NR(HANTRODEC_IOCS_DEC_WRITE_REG): {
		struct core_desc core;

		/* get registers from user space */
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		return DecWriteRegs(&hantrodec_data, &core);
	}
	case _IOC_NR(HANTRODEC_IOCS_DEC_PULL_REG): {
		struct core_desc core;

		/* get registers from user space */
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		return DecRefreshRegs(&hantrodec_data, &core);
	}
	case _IOC_NR(HANTRODEC_IOCS_DEC_READ_REG): {
		struct core_desc core;

		/* get registers from user space */
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		return DecReadRegs(&hantrodec_data, &core);
	}
	case _IOC_NR(HANTRODEC_IOCH_DEC_RESERVE): {
		PDEBUG("Reserve DEC core, format = %li\n", arg);
		return ReserveDecoder(&hantrodec_data, filp, arg);
	}
	case _IOC_NR(HANTRODEC_IOCT_DEC_RELEASE): {
		if (arg >= hantrodec_data.cores || dec_owner[arg] != filp) {
			PDEBUG("bogus DEC release, core = %li\n", arg);
			return -EFAULT;
		}

		PDEBUG("Release DEC, core = %li\n", arg);
		ReleaseDecoder(&hantrodec_data, arg);

		break;
	}
	case _IOC_NR(HANTRODEC_IOCX_DEC_WAIT): {
		struct core_desc core;

		/* get registers from user space */
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		return WaitDecReadyAndRefreshRegs(&hantrodec_data, &core);
	}
	case _IOC_NR(HANTRODEC_IOCS_PP_PUSH_REG):
	case _IOC_NR(HANTRODEC_IOCS_PP_PULL_REG):
	case _IOC_NR(HANTRODEC_IOCQ_PP_RESERVE):
	case _IOC_NR(HANTRODEC_IOCT_PP_RELEASE):
	case _IOC_NR(HANTRODEC_IOCX_PP_WAIT):
		return -EINVAL;
	case _IOC_NR(HANTRODEC_IOCG_CORE_WAIT): {
		int id;

		tmp = WaitCoreReady(&hantrodec_data, filp, &id);
		__put_user(id, (int *) arg);
		return tmp;
	}
	case _IOC_NR(HANTRODEC_IOX_ASIC_ID): {
		struct core_param core;

		/* get registers from user space */
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_param));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		if (core.id >= MAX_SUBSYS_NUM ||
			!vpu_subsys[core.id].submodule_iosize[core.type]) {
			return -EFAULT;
		}

		core.size = vpu_subsys[core.id].submodule_iosize[core.type];
		if (vpu_subsys[core.id].submodule_hwregs[core.type])
			core.asic_id = ioread32((void *)hantrodec_data.hwregs[core.id][core.type]);
		else
			core.asic_id = 0;
		tmp = copy_to_user((u32 *) arg,
				&core, sizeof(struct core_param));
		if (tmp) {
			pr_err("copy_to_user failed, returned %li\n", tmp);
			return -EFAULT;
		}
		break;
	}
	case _IOC_NR(HANTRODEC_IOCG_CORE_ID): {
		PDEBUG("Get DEC Core_id, format = %li\n", arg);
		return GetDecCoreID(&hantrodec_data, filp, arg);
	}
	case _IOC_NR(HANTRODEC_IOX_ASIC_BUILD_ID): {
		u32 id, hw_id;

		__get_user(id, (u32 *)arg);

		if (id >= hantrodec_data.cores)
			return -EFAULT;

		hw_id = ioread32((void *)(hantrodec_data.hwregs[id][HW_VC8000D]));
		if (IS_G1(hw_id >> 16) || IS_G2(hw_id >> 16) ||
					(IS_VC8000D(hw_id >> 16) &&
					((hw_id & 0xFFFF) == 0x6010)) ||
					(IS_BIGOCEAN(hw_id & 0xFFFF)))
			__put_user(hw_id, (u32 *) arg);
		else {
			hw_id = ioread32((void *)(hantrodec_data.hwregs[id][HW_VC8000D] +
					HANTRODEC_HW_BUILD_ID_OFF));
			__put_user(hw_id, (u32 *) arg);
		}
		return 0;
	}
	case _IOC_NR(HANTRODEC_DEBUG_STATUS): {
		pr_debug("dec_irq     = 0x%08x\n", dec_irq);
		pr_debug("pp_irq      = 0x%08x\n", pp_irq);
		pr_debug("IRQs received/sent2user = %d / %d\n",
			atomic_read(&irq_rx), atomic_read(&irq_tx));

		for (tmp = 0; tmp < hantrodec_data.cores; tmp++) {
			pr_debug("dec_core[%li] %s\n", tmp,
				dec_owner[tmp] == NULL ? "FREE" : "RESERVED");
			pr_debug("pp_core[%li]  %s\n", tmp,
				pp_owner[tmp] == NULL ? "FREE" : "RESERVED");
		}
		return 0;
	}
	case _IOC_NR(HANTRODEC_IOX_SUBSYS): {
		struct subsys_desc subsys = {0};

		subsys.subsys_num = hantrodec_data.cores;
		tmp = copy_to_user((u32 *) arg,
				&subsys, sizeof(struct subsys_desc));
		if (tmp) {
			pr_err("copy_to_user failed, returned %li\n", tmp);
			return -EFAULT;
		}
		break;
	}
	case _IOC_NR(HANTRODEC_IOCX_POLL): {
		hantrodec_isr(0, &hantrodec_data);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCX_DMA_BUF_ATTACH): {
		struct dma_buf *dmabuf;
		struct dma_buf_attachment *attachment = NULL;
		struct dma_buf_desc buf_data;
		struct sg_table *sgt = NULL;
		struct dma_buf_info *mapping_info;

		tmp = copy_from_user(&buf_data,
				(void __user *)arg, sizeof(buf_data));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		buf_data.dma_addr = 0;
		dmabuf = dma_buf_get(buf_data.fd);
		if (!dmabuf || IS_ERR(dmabuf))
			return -EFAULT;

		attachment = dma_buf_attach(dmabuf, hantrodec_data.dev);
		if (!attachment || IS_ERR(attachment))
			return -EFAULT;

		sgt = dma_buf_map_attachment(attachment, DMA_BIDIRECTIONAL);
		if (sgt && !IS_ERR(sgt))
			buf_data.dma_addr = sg_dma_address(sgt->sgl);
		else {
			pr_err("Error: table sgt is null!\n");
			dma_buf_detach(dmabuf, attachment);
			dma_buf_put(dmabuf);
			return -EFAULT;
		}

		/* create and store current dma buf info */
		mapping_info = kzalloc(sizeof(struct dma_buf_info), GFP_KERNEL);

		if (!mapping_info) {
			dma_buf_unmap_attachment(attachment,
						sgt, DMA_BIDIRECTIONAL);
			dma_buf_detach(dmabuf, attachment);
			dma_buf_put(dmabuf);
			return -ENOMEM;
		}

		mapping_info->fd = buf_data.fd;
		mapping_info->dma_addr = buf_data.dma_addr;
		mapping_info->dmabuf = dmabuf;
		mapping_info->attachment = attachment;
		mapping_info->sgt = sgt;
		mapping_info->owner = filp;

		mutex_lock(&hantrodec_data.dmabuf_list_lock);
		list_add(&mapping_info->list, &hantrodec_data.dma_buf_list);
		mutex_unlock(&hantrodec_data.dmabuf_list_lock);

		tmp = copy_to_user((void __user *)arg,
				&buf_data, sizeof(buf_data));
		if (tmp) {
			pr_err("copy_to_user failed, returned %li\n", tmp);
			return -EFAULT;
		}
		break;
	}
	case _IOC_NR(HANTRODEC_IOCX_DMA_BUF_DETACH): {
		struct dma_buf_desc buf_data;
		struct dma_buf_info *mapping_info;
		bool found = false;

		tmp = copy_from_user(&buf_data,
				(void __user *)arg, sizeof(buf_data));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		/*
		 * find the matched struct dma_buf_info,
		 * judged by dma_addr, update the fd.
		 */
		mutex_lock(&hantrodec_data.dmabuf_list_lock);
		list_for_each_entry(mapping_info,
			&hantrodec_data.dma_buf_list, list) {
			if (mapping_info->dma_addr == buf_data.dma_addr) {
				found = true;
				buf_data.fd = mapping_info->fd;
				break;
			}
		}

		if (found) {
			PDEBUG("detach buf addr = 0x%llx\n",
				mapping_info->dma_addr);

			dma_buf_unmap_attachment(mapping_info->attachment,
					mapping_info->sgt, DMA_BIDIRECTIONAL);
			dma_buf_detach(mapping_info->dmabuf,
					mapping_info->attachment);
			dma_buf_put(mapping_info->dmabuf);

			list_del_init(&mapping_info->list);
			kfree(mapping_info);
		}
		mutex_unlock(&hantrodec_data.dmabuf_list_lock);

		tmp = copy_to_user((void __user *)arg,
				&buf_data, sizeof(buf_data));
		if (tmp) {
			pr_err("copy_to_user failed, returned %li\n", tmp);
			return -EFAULT;
		}
		break;
	}
	default:
		return -ENOTTY;
	}

	return 0;
}

#ifdef CONFIG_COMPAT
static long hantrodec_ioctl_compat(struct file *filp,
				   unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	if (_IOC_TYPE(cmd) == HANTRODEC_IOC_MAGIC) {
		switch (_IOC_NR(cmd)) {
		case _IOC_NR(HANTRODEC_IOCS_DEC_PUSH_REG): {
			struct core_desc core;

			/* get registers from user space */
			ret = copy_from_user(&core,
					(void *)arg, sizeof(struct core_desc));
			if (ret) {
				pr_err("copy_from_user failed, err %i\n", ret);
				return -EFAULT;
			}
			core.regs_addr = (__u64)compat_ptr(core.regs_addr);

			return DecFlushRegs(&hantrodec_data, &core);
		}
		case _IOC_NR(HANTRODEC_IOCS_DEC_WRITE_REG): {
			struct core_desc core;

			/* get registers from user space */
			ret = copy_from_user(&core,
					(void *)arg, sizeof(struct core_desc));
			if (ret) {
				pr_err("copy_from_user failed, err %i\n", ret);
				return -EFAULT;
			}
			core.regs_addr = (__u64)compat_ptr(core.regs_addr);

			return DecWriteRegs(&hantrodec_data, &core);
		}
		case _IOC_NR(HANTRODEC_IOCS_DEC_PULL_REG): {
			struct core_desc core;

			/* get registers from user space */
			ret = copy_from_user(&core,
					(void *)arg, sizeof(struct core_desc));
			if (ret) {
				pr_err("copy_from_user failed, err %i\n", ret);
				return -EFAULT;
			}
			core.regs_addr = (__u64)compat_ptr(core.regs_addr);

			return DecRefreshRegs(&hantrodec_data, &core);
		}
		case _IOC_NR(HANTRODEC_IOCS_DEC_READ_REG): {
			struct core_desc core;

			/* get registers from user space */
			ret = copy_from_user(&core,
					(void *)arg, sizeof(struct core_desc));
			if (ret) {
				pr_err("copy_from_user failed, err %i\n", ret);
				return -EFAULT;
			}
			core.regs_addr = (__u64)compat_ptr(core.regs_addr);

			return DecReadRegs(&hantrodec_data, &core);
		}
		case _IOC_NR(HANTRODEC_IOCX_DEC_WAIT): {
			struct core_desc core;

			/* get registers from user space */
			ret = copy_from_user(&core,
					(void *)arg, sizeof(struct core_desc));
			if (ret) {
				pr_err("copy_from_user failed, err %i\n", ret);
				return -EFAULT;
			}
			core.regs_addr = (__u64)compat_ptr(core.regs_addr);

			return WaitDecReadyAndRefreshRegs(&hantrodec_data,
							  &core);
		}
		default:
			break;
		}
	}

	return hantrodec_ioctl(filp, cmd, arg);
}
#endif /* CONFIG_COMPAT */

/*
 * --------------------------------------------------------------------------
 * Function name   : ReleaseMemory
 * Description     : release memory
 *
 * Return type     :
 * --------------------------------------------------------------------------
 */
static void ReleaseMemory(struct file *filp)
{
	struct dma_buf_info *mapping_info, *tmp;

	PDEBUG("memory_release\n");

	mutex_lock(&hantrodec_data.dmabuf_list_lock);
	list_for_each_entry_safe(mapping_info,
		tmp, &hantrodec_data.dma_buf_list, list) {
		if (mapping_info->owner == filp) {
			PDEBUG("detach buf = 0x%llx\n", mapping_info->dma_addr);
			dma_buf_unmap_attachment(mapping_info->attachment,
					mapping_info->sgt, DMA_BIDIRECTIONAL);
			dma_buf_detach(mapping_info->dmabuf,
					mapping_info->attachment);
			dma_buf_put(mapping_info->dmabuf);

			list_del(&mapping_info->list);
			kfree(mapping_info);
		}
	}
	mutex_unlock(&hantrodec_data.dmabuf_list_lock);
}

/*
 * --------------------------------------------------------------------------
 * Function name   : hantrodec_open
 * Description     : open method
 *
 * Return type     : int
 * --------------------------------------------------------------------------
 */
static int hantrodec_open(struct inode *inode, struct file *filp)
{
	PDEBUG("dev opened\n");
	pm_runtime_get_sync(hantrodec_data.dev);
	atomic_inc(&hantrodec_data.channels);
	return 0;
}

/*
 * --------------------------------------------------------------------------
 * Function name   : hantrodec_release
 * Description     : Release driver
 *
 * Return type     : int
 * --------------------------------------------------------------------------
 */
static int hantrodec_release(struct inode *inode, struct file *filp)
{
	int n;
	struct hantrodec *dev = &hantrodec_data;
#ifdef SUPPORT_SFBC
	u32 status;
#endif

	PDEBUG("closing ...\n");

	for (n = 0; n < dev->cores; n++) {
		if (dec_owner[n] == filp) {
			PDEBUG("releasing dec core %i lock\n", n);
			ReleaseDecoder(dev, n);
		}

#ifdef SUPPORT_SFBC
		// soft reset
		status = ioread32((void *)(dev->hwregs[n][HW_SFBC_ENC] +
						HANTRODEC_SFBCENC_IRQ_RAW_STATUS * 4));
		if ((status >> 2) != 0) {
			iowrite32(0x8, (void *)(dev->hwregs[n][HW_SFBC_ENC] +
					HANTRODEC_SFBCENC_COMMAND * 4));
			iowrite32(0x0, (void *)(dev->hwregs[n][HW_SFBC_ENC] +
					HANTRODEC_SFBCENC_COMMAND * 4));
		}
#endif
	}

	for (n = 0; n < 1; n++) {
		if (pp_owner[n] == filp) {
			PDEBUG("releasing pp core %i lock\n", n);
			ReleasePostProcessor(dev, n);
		}
	}

	ReleaseMemory(filp);
	atomic_dec(&hantrodec_data.channels);
	pm_runtime_put_autosuspend(hantrodec_data.dev);
	PDEBUG("closed\n");
	return 0;
}

/*
 * --------------------------------------------------------------------------
 * Function name   : hantro_mmap
 * Description     : memory map interface for hantro file operation
 *
 * Return type     : int
 * --------------------------------------------------------------------------
 */
static int hantro_mmap(struct file *fp, struct vm_area_struct *vm)
{
	if (vm->vm_pgoff == (multicorebase[0] >> PAGE_SHIFT)) {
		vm->vm_flags |= VM_IO;
		vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
		PDEBUG("hantro mmap: size=0x%lX, page off=0x%lX\n",
			(vm->vm_end - vm->vm_start), vm->vm_pgoff);
		return remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff,
					vm->vm_end - vm->vm_start,
					vm->vm_page_prot) ? -EAGAIN : 0;
	} else {
		pr_err("invalid map offset :0x%lX\n", vm->vm_pgoff);
		return -EINVAL;
	}
}

static const struct file_operations hantrodec_fops = {
	.owner = THIS_MODULE,
	.open = hantrodec_open,
	.release = hantrodec_release,
	.unlocked_ioctl = hantrodec_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = hantrodec_ioctl_compat,
#endif
	.fasync = NULL,
	.mmap = hantro_mmap,
};

/*
 * --------------------------------------------------------------------------
 * Function name   : hantrodec_prepare
 * Description     : Prepare the driver
 *
 * Return type     : int
 * --------------------------------------------------------------------------
 */
static int hantrodec_prepare(void)
{
	int result, i;

	PDEBUG("module init\n");

#ifndef CONFIG_OF
	CheckSubsysCoreArray(vpu_subsys);
#endif

	pr_debug("hantrodec: dec/pp kernel module.\n");

	/*
	 * If base_port is set when insmod,
	 * use that for single core legacy mode.
	 */
	if (base_port != -1) {
		multicorebase[0] = base_port;
		elements = 1;
		pr_debug("hantrodec: Init single core at 0x%08lx IRQ=%i\n",
			  multicorebase[0], irq[0]);
	} else {
		pr_debug("hantrodec: Init multi core[0] at 0x%16lx\n"
				"           core[1] at 0x%16lx\n"
				"           core[2] at 0x%16lx\n"
				"            core[3] at 0x%16lx\n"
				"           IRQ_0=%i\n"
				"           IRQ_1=%i\n",
				multicorebase[0], multicorebase[1],
				multicorebase[2], multicorebase[3],
				irq[0], irq[1]);
	}

	hantrodec_data.cores = 0;

	hantrodec_data.iosize[0] = DEC_IO_SIZE_0;
	hantrodec_data.irq[0] = irq[0];
	hantrodec_data.iosize[1] = DEC_IO_SIZE_1;
	hantrodec_data.irq[1] = irq[1];

	for (i = 0; i < HXDEC_MAX_CORES; i++) {
		int j;

		for (j = 0; j < HW_CORE_MAX; j++)
			hantrodec_data.hwregs[i][j] = 0;
		/*
		 * If user gave less core bases that we
		 * have by default,invalidate default bases
		 */
		if (elements && i >= elements)
			multicorebase[i] = 0;
	}

	hantrodec_data.async_queue_dec = NULL;
	hantrodec_data.async_queue_pp = NULL;

	INIT_LIST_HEAD(&hantrodec_data.dma_buf_list);
	mutex_init(&hantrodec_data.dmabuf_list_lock);

	result = register_chrdev(hantrodec_major, "hantrodec", &hantrodec_fops);
	if (result < 0) {
		pr_debug("hantrodec: unable to get major %d\n",
			hantrodec_major);
		goto err;
	} else if (result != 0) /* this is for dynamic major */
		hantrodec_major = result;

	result = ReserveIO();
	if (result < 0)
		goto err;

	memset(dec_owner, 0, sizeof(dec_owner));
	memset(pp_owner, 0, sizeof(pp_owner));

	sema_init(&dec_core_sem, hantrodec_data.cores);
	sema_init(&pp_core_sem, 1);

	/* read configuration fo all cores */
	ReadCoreConfig(&hantrodec_data);

	/* reset hardware */
	ResetAsic(&hantrodec_data);

	/* register irq for each core */
	if (irq[0] > 0) {
		result = request_irq(irq[0], hantrodec_isr,
				IRQF_SHARED,
				"hantro_dec", (void *) &hantrodec_data);
		if (result != 0) {
			if (result == -EINVAL)
				pr_err("hantrodec: Bad irq number or handler\n");
			else if (result == -EBUSY) {
				pr_err("hantrodec: IRQ <%d> busy, change your config\n",
					hantrodec_data.irq[0]);
			}

			ReleaseIO();
			goto err;
		}
	} else
		pr_debug("hantrodec: IRQ not in use!\n");

	if (irq[1] > 0) {
		result = request_irq(irq[1], hantrodec_isr,
				IRQF_SHARED,
				"hantro_dec", (void *) &hantrodec_data);
		if (result != 0) {
			if (result == -EINVAL)
				pr_err("hantrodec: Bad irq number or handler\n");
			else if (result == -EBUSY) {
				pr_err("hantrodec: IRQ <%d> busy, change your config\n",
					hantrodec_data.irq[1]);
			}

			ReleaseIO();
			goto err;
		}
	} else
		pr_debug("hantrodec: IRQ not in use!\n");

	pr_info("hantrodec: module inserted. Major = %d\n", hantrodec_major);

	/*
	 * Please call the TEE functions to
	 * set VC80000D DRM relative registers here
	 */

	return 0;

err:
	ReleaseIO();
	pr_debug("hantrodec: module not inserted\n");
	unregister_chrdev(hantrodec_major, "hantrodec");
	return result;
}

/*
 * --------------------------------------------------------------------------
 * Function name   : hantrodec_cleanup
 * Description     : clean up
 *
 * Return type     :
 * --------------------------------------------------------------------------
 */
static void hantrodec_cleanup(void)
{
	struct hantrodec *dev = &hantrodec_data;
	int n = 0;

	/* reset hardware */
	ResetAsic(dev);

	/* free the IRQ */
	for (n = 0; n < dev->cores; n++) {
		if (dev->irq[n] != -1)
			free_irq(dev->irq[n], (void *) dev);
	}

	ReleaseIO();

	unregister_chrdev(hantrodec_major, "hantrodec");
	pr_debug("hantrodec: module removed\n");
}

/*
 * --------------------------------------------------------------------------
 * Function name   : CheckHwId
 * Return type     : int
 * --------------------------------------------------------------------------
 */
static int CheckHwId(struct hantrodec *dev)
{
	int hwid;
	int i;
	size_t num_hw = sizeof(DecHwId) / sizeof(*DecHwId);
	int found = 0;

	for (i = 0; i < dev->cores; i++) {
		if (dev->hwregs[i][HW_VC8000D] != NULL) {
			hwid = readl(dev->hwregs[i][HW_VC8000D]);
			pr_debug("hantrodec: core %d HW ID=0x%08x\n", i, hwid);
			if (IS_BIGOCEAN(hwid & 0xFFFF)) {
				hwid = (hwid & 0xFFFF);
				reg_count[i] = HANTRO_BIGOCEAN_REGS;
			} else {
				/* product version only */
				hwid = (hwid >> 16) & 0xFFFF;
				reg_count[i] = HANTRO_VC8000D_REGS;
			}

			while (num_hw--) {
				if (hwid == DecHwId[num_hw]) {
					pr_debug("hantrodec: Supported HW found at 0x%16lx\n",
						multicorebase_actual[i]);
					found++;
					dev->hw_id[i] = hwid;
					break;
				}
			}
			if (!found) {
				pr_debug("hantrodec: Unknown HW found at 0x%16lx\n",
					multicorebase_actual[i]);
				return 0;
			}
			found = 0;
			num_hw = sizeof(DecHwId) / sizeof(*DecHwId);
		}
	}

	return 1;
}

/*
 * --------------------------------------------------------------------------
 * Function name   : ReserveIO
 * Description     : IO reserve
 *
 * Return type     : int
 * --------------------------------------------------------------------------
 */
static int ReserveIO(void)
{
	int i, j;
	long int hwid;

	memcpy(multicorebase_actual, multicorebase,
		HXDEC_MAX_CORES * sizeof(unsigned long));
	memcpy((unsigned int *)(hantrodec_data.iosize),
		iosize, HXDEC_MAX_CORES * sizeof(unsigned int));
	memcpy((unsigned int *)(hantrodec_data.irq),
		irq, HXDEC_MAX_CORES * sizeof(int));

	for (i = 0; i < MAX_SUBSYS_NUM; i++) {
		if (!vpu_subsys[i].base_addr)
			continue;

		for (j = 0; j < HW_CORE_MAX; j++) {
			if (vpu_subsys[i].submodule_iosize[j]) {
				pr_debug("hantrodec: base=0x%16lx, iosize=%d\n",
					vpu_subsys[i].base_addr +
					vpu_subsys[i].submodule_offset[j],
					vpu_subsys[i].submodule_iosize[j]);

				if (!request_mem_region(vpu_subsys[i].base_addr + vpu_subsys[i].submodule_offset[j],
						vpu_subsys[i].submodule_iosize[j],
						"hantrodec0")) {
					pr_debug("dec: reserve HW %d regs fail\n",
						j);
					return -EBUSY;
				}

				vpu_subsys[i].submodule_hwregs[j] =
					hantrodec_data.hwregs[i][j] =
					(u8 *) ioremap(vpu_subsys[i].base_addr + vpu_subsys[i].submodule_offset[j],
						vpu_subsys[i].submodule_iosize[j]);

				if (hantrodec_data.hwregs[i][j] == NULL) {
					pr_debug("dec: ioremap HW %d regs fail\n",
						j);
					release_mem_region(vpu_subsys[i].base_addr +
							vpu_subsys[i].submodule_offset[j],
							vpu_subsys[i].submodule_iosize[j]);
					return -EBUSY;
				}
				config.its_main_core_id[i] = -1;
				config.its_aux_core_id[i] = -1;

				pr_debug("hantrodec: HW %d reg[0]=0x%08x\n", j,
					readl(hantrodec_data.hwregs[i][j]));
				/* product version only */
				hwid = ((readl(hantrodec_data.hwregs[i][HW_VC8000D])) >> 16) & 0xFFFF;
			}
		}
		hantrodec_data.cores++;
	}

	/* check for correct HW */
	if (!CheckHwId(&hantrodec_data)) {
		ReleaseIO();
		return -EBUSY;
	}

	return 0;
}

/*
 * --------------------------------------------------------------------------
 * Function name   : releaseIO
 * Description     : release
 *
 * Return type     :
 * --------------------------------------------------------------------------
 */
static void ReleaseIO(void)
{
	int i, j;

	for (i = 0; i < hantrodec_data.cores; i++) {
		for (j = 0; j < HW_CORE_MAX; j++) {
			if (hantrodec_data.hwregs[i][j]) {
				iounmap((void *) hantrodec_data.hwregs[i][j]);
				release_mem_region(vpu_subsys[i].base_addr +
					vpu_subsys[i].submodule_offset[j],
					vpu_subsys[i].submodule_iosize[j]);
				hantrodec_data.hwregs[i][j] = 0;
			}
		}
	}
}

/*
 * --------------------------------------------------------------------------
 * Function name   : hantrodec_isr
 * Description     : interrupt handler
 *
 * Return type     : irqreturn_t
 * --------------------------------------------------------------------------
 */
irqreturn_t hantrodec_isr(int irq, void *dev_id)
{
	unsigned long flags;
	unsigned int handled = 0;
	int i;
	struct hantrodec *dev = (struct hantrodec *) dev_id;
	u32 irq_status_dec;

	spin_lock_irqsave(&dec_owner_lock, flags);

	for (i = 0; i < dev->cores; i++) {
		u8 *hwregs = dev->hwregs[i][HW_VC8000D];

		/* interrupt status register read */
		irq_status_dec = ioread32((void *)(hwregs +
					HANTRODEC_IRQ_STAT_DEC_OFF));

		if (irq_status_dec & HANTRODEC_DEC_IRQ) {
			/* clear dec IRQ */
			irq_status_dec &= (~HANTRODEC_DEC_IRQ);
			iowrite32(irq_status_dec,
				(void *)(hwregs + HANTRODEC_IRQ_STAT_DEC_OFF));

			PDEBUG("decoder IRQ received! core %d\n", i);

			atomic_inc(&irq_rx);

			dec_irq |= (1 << i);

			wake_up_interruptible_all(&dec_wait_queue);
			handled++;
		}
	}

	spin_unlock_irqrestore(&dec_owner_lock, flags);

	if (!handled)
		PDEBUG("IRQ received, but not hantrodec's!\n");

	return IRQ_RETVAL(handled);
}

/*
 * --------------------------------------------------------------------------
 * Function name   : ResetAsic
 * Description     : reset asic
 *
 * Return type     :
 * --------------------------------------------------------------------------
 */
void ResetAsic(struct hantrodec *dev)
{
	int i, j;
	u32 status;

	for (j = 0; j < dev->cores; j++) {
		status = ioread32((void *)(dev->hwregs[j][HW_VC8000D] +
				HANTRODEC_IRQ_STAT_DEC_OFF));
		if (status & HANTRODEC_DEC_E) {
			/* abort with IRQ disabled */
			status = HANTRODEC_DEC_ABORT |
					HANTRODEC_DEC_IRQ_DISABLE;
			iowrite32(status, (void *)(dev->hwregs[j][HW_VC8000D] +
						HANTRODEC_IRQ_STAT_DEC_OFF));
		}

		if (IS_G1(dev->hw_id[j]))
			/* reset PP */
			iowrite32(0, (void *)(dev->hwregs[j][HW_VC8000D] +
					HANTRO_IRQ_STAT_PP_OFF));

		for (i = 4; i < dev->iosize[j]; i += 4)
			iowrite32(0, (void *)(dev->hwregs[j][HW_VC8000D] + i));
	}
}

/*
 * --------------------------------------------------------------------------
 * Function name   : dump_regs
 * Description     : Dump registers
 *
 * Return type     :
 * --------------------------------------------------------------------------
 */
#ifdef HANTRODEC_DEBUG
void dump_regs(struct hantrodec *dev)
{
	int i, c;

	PDEBUG("Reg Dump Start\n");
	for (c = 0; c < dev->cores; c++) {
		for (i = 0; i < dev->iosize[c]; i += 4*4) {
			PDEBUG("\toffset %04X: %08X  %08X  %08X  %08X\n", i,
				ioread32(dev->hwregs[c][HW_VC8000D] + i),
				ioread32(dev->hwregs[c][HW_VC8000D] + i + 4),
				ioread32(dev->hwregs[c][HW_VC8000D] + i + 16),
				ioread32(dev->hwregs[c][HW_VC8000D] + i + 24));
		}
	}
	PDEBUG("Reg Dump End\n");
}
#endif

/* devfreq and thermal cooling support */
static int hantrodec_devfreq_target(struct device *dev,
	unsigned long *freq, u32 flags)
{
	struct dev_pm_opp *opp;
	int err;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	dev_pm_opp_put(opp);

	err = dev_pm_opp_set_rate(dev, *freq);
	if (!err)
		clk_set_rate(hantrodec_data.clk_core, *freq);

	return err;
}

static int hantrodec_devfreq_get_dev_status(struct device *dev,
		struct devfreq_dev_status *status)
{
	struct clk *clk = dev_get_drvdata(dev);

	status->busy_time = 0;
	status->total_time = 0;
	status->current_frequency = clk_get_rate(clk);

	return 0;
}

static struct devfreq_dev_profile hantrodec_devfreq_profile = {
	.polling_ms = 1000, /* 1 second */
	.target = hantrodec_devfreq_target,
	.get_dev_status = hantrodec_devfreq_get_dev_status,
};

static void hantrodec_devfreq_fini(struct hantrodec *hdev)
{
	if (hdev->cooling) {
		devfreq_cooling_unregister(hdev->cooling);
		hdev->cooling = NULL;
	}

	if (hdev->devfreq) {
		devm_devfreq_remove_device(hdev->dev, hdev->devfreq);
		hdev->devfreq = NULL;
	}

	if (hdev->opp_table_added) {
		dev_pm_opp_remove_all_dynamic(hdev->dev);
		hdev->opp_table_added = false;
	}

	if (hdev->opp_table) {
		dev_pm_opp_put_clkname(hdev->opp_table);
		hdev->opp_table = NULL;
	}
}

static int hantrodec_devfreq_init(struct hantrodec *hdev)
{
	int i;
	struct opp_table *opp_table;
	unsigned long max_freq;
	unsigned long cur_freq;
	struct dev_pm_opp *opp;
	struct devfreq *devfreq;
	struct thermal_cooling_device *cooling;
	struct device *dev = hdev->dev;
	int ret;

	dev_set_drvdata(dev, hdev->clk_axi);

	cur_freq = max_freq = clk_get_rate(hdev->clk_axi);

	opp_table = dev_pm_opp_set_clkname(dev, "axi");
	if (IS_ERR(opp_table)) {
		dev_warn(dev, "could not get alloc opp_table\n");
		ret = PTR_ERR(opp_table);
		goto err_fini;
	}
	hdev->opp_table = opp_table;

#define DEV_PM_OPP_MAX 4
	for (i = 0; i < DEV_PM_OPP_MAX; i++) {
		dev_pm_opp_add(dev, cur_freq, DEV_PM_OPP_MAX - i);
		cur_freq >>= 1;
	}
	hdev->opp_table_added = true;

	opp = devfreq_recommended_opp(dev, &max_freq, 0);
	if (IS_ERR(opp)) {
		dev_warn(dev, "could not devfreq_recommended_opp\n");
		ret = PTR_ERR(opp);
		goto err_fini;
	}

	hantrodec_devfreq_profile.initial_freq = max_freq;
	dev_pm_opp_put(opp);

	devfreq = devm_devfreq_add_device(dev, &hantrodec_devfreq_profile,
				DEVFREQ_GOV_SIMPLE_ONDEMAND, NULL);
	if (IS_ERR(devfreq)) {
		dev_warn(dev, "Couldn't initialize hantrodec devfreq\n");
		ret = PTR_ERR(devfreq);
		goto err_fini;
	}
	hdev->devfreq = devfreq;

	cooling = of_devfreq_cooling_register(dev->of_node, devfreq);
	if (IS_ERR(cooling))
		dev_warn(dev, "Failed to register cooling device\n");
	else
		hdev->cooling = cooling;

	return 0;

err_fini:
	hantrodec_devfreq_fini(hdev);
	return ret;
}

/* platform driver interfaces */
static int hantrodec_device_probe(struct platform_device *pdev)
{
	struct device *temp_class;
	int ret;

	hantrodec_data.dev = &pdev->dev;
	atomic_set(&hantrodec_data.channels, 0);

	/* load the default values */
	ret = GetSubsysCoreArrayFromDeviceTree(vpu_subsys, pdev);
	if (ret != 0)
		return ret;
#ifdef SUPPORT_DEVMGR
	dec_dev_alloc();
#endif
	/* initialise runtime power management */
	pm_runtime_set_autosuspend_delay(hantrodec_data.dev,
		SUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(hantrodec_data.dev);
	pm_runtime_enable(hantrodec_data.dev);
	pm_runtime_get_sync(hantrodec_data.dev);

	/* deassert clock */
	hantrodec_deassert_clk(hantrodec_data.dev);

	/* register devfreq */
	ret = hantrodec_devfreq_init(&hantrodec_data);
	if (ret != 0)
		goto error;

	ret = hantrodec_prepare();
	if (ret != 0)
		goto error;

	ret = dma_set_mask_and_coherent(hantrodec_data.dev, 0xFFFFFFFFF);
	if (ret != 0)
		goto error;

	hantro_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(hantro_class)) {
		ret = PTR_ERR(hantro_class);
		goto error;
	}

	temp_class = device_create(hantro_class, NULL,
				MKDEV(hantrodec_major, 0), NULL, DEVICE_NAME);
	if (IS_ERR(temp_class)) {
		ret = PTR_ERR(temp_class);
		goto err_out_class;
	}

	goto out;

err_out_class:
	device_destroy(hantro_class, MKDEV(hantrodec_major, 0));
	class_destroy(hantro_class);

error:
	pr_err("hantrodec probe failed\n");

out:
	pm_runtime_put_autosuspend(hantrodec_data.dev);
	return ret;
}

static int hantrodec_device_remove(struct platform_device *pdev)
{
	pm_runtime_get_sync(&pdev->dev);
	if (hantrodec_major > 0) {
		hantrodec_devfreq_fini(&hantrodec_data);
		device_destroy(hantro_class, MKDEV(hantrodec_major, 0));
		class_destroy(hantro_class);
		hantrodec_cleanup();
		hantrodec_major = 0;
	}

	/* assert clock */
	hantrodec_assert_clk(&pdev->dev);

	pm_runtime_put_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

#ifdef SUPPORT_DEVMGR
	dec_dev_release();
#endif

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id hantrodec_dt_ids[] = {
	{ .compatible = "vpu,vc8000d" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, hantrodec_dt_ids);
#endif /* CONFIG_OF */

#ifdef CONFIG_PM
static int hantrodec_suspend(struct device *dev)
{
	int i;
	u32 dec_status;
	int maxwait = 1000;

	if (hantrodec_data.hwregs[0][0] != NULL) {
		if (atomic_read(&hantrodec_data.channels) <= 0)
			hantrodec_enable_clk(dev);

		while (maxwait--) {
			dec_status = ioread32(hantrodec_data.hwregs[0][HW_VC8000D] + 4);
			if (!(dec_status & HANTRODEC_DEC_IRQ))
				break;
			wait_delay(1);
		}

		if (dec_status & HANTRODEC_DEC_IRQ) {
			pr_info("dec is still enabled, reset it\n");
			ResetAsic(&hantrodec_data);
		}

		for (i = 0; i < HANTRO_VC8000D_REGS; i++)
			hantrodec_data.reg_backup[i] =
				ioread32(hantrodec_data.hwregs[0][HW_VC8000D] + i * 4);

		hantrodec_disable_clk(dev);
#ifdef SUPPORT_DEVMGR
		dec_dev_release();
#endif
	}

	return 0;
}

static int hantrodec_resume(struct device *dev)
{
	int i;

	if (hantrodec_data.hwregs[0][0] != NULL) {
#ifdef SUPPORT_DEVMGR
		dec_dev_alloc();
#endif
		hantrodec_enable_clk(dev);
		for (i = 0; i < HANTRO_VC8000D_REGS; i++)
			iowrite32(hantrodec_data.reg_backup[i],
				(void *)hantrodec_data.hwregs[0][HW_VC8000D] + i * 4);

		if (atomic_read(&hantrodec_data.channels) <= 0)
			hantrodec_disable_clk(dev);
	}

	return 0;
}

static int hantrodec_runtime_suspend(struct device *dev)
{
	hantrodec_disable_clk(dev);
	return 0;
}

static int hantrodec_runtime_resume(struct device *dev)
{
	hantrodec_enable_clk(dev);
	return 0;
}

static const struct dev_pm_ops hantrodec_pm_ops = {
	SET_RUNTIME_PM_OPS(hantrodec_runtime_suspend,
		hantrodec_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(hantrodec_suspend, hantrodec_resume)
};
#endif //CONFIG_PM

static struct platform_driver hantrodec_platform_driver = {
	.probe = hantrodec_device_probe,
	.remove = hantrodec_device_remove,
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(hantrodec_dt_ids),
#ifdef CONFIG_PM
		.pm = &hantrodec_pm_ops,
#endif
	},
};

#ifdef CONFIG_OF
module_platform_driver(hantrodec_platform_driver);
#else
/*
 * CAUTION: Here platform driver mode is only for CONFIG_OF.
 * W/o. CONFIG_OF, VSI original driver processing will be used.
 */
static int __init hantrodec_init(void)
{
	return hantrodec_prepare();
}

static void __exit hantrodec_exit(void)
{
	hantrodec_cleanup();
}

module_init(hantrodec_init);
module_exit(hantrodec_exit);
#endif /* CONFIG_OF */

/* module description */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Google Finland Oy");
MODULE_DESCRIPTION("Driver module for Hantro Decoder/Post-Processor");

