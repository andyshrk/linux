/*
 *
 *    The GPL License (GPL)
 *
 *    Copyright (C) 2014 - 2021 VERISILICON
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/dma-buf.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/uaccess.h>
#include <linux/ioport.h>
#include <asm/irq.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/compat.h>
#include <linux/devfreq.h>
#include <linux/devfreq_cooling.h>

#include "hantro_enc.h"

static struct class *hantro_class;
#define DEVICE_NAME "hantro_enc"
#define SUSPEND_DELAY_MS                100

#ifdef MULTI_THR_TEST

#define WAIT_NODE_NUM 32
struct wait_list_node {
	/* index of the node */
	u32 node_id;
	/* 1:the node is insert to the wait queue list. */
	u32 used_flag;
	/* 1:the source is released and the semphone is uped. */
	u32 sem_used;
	/* the unique semphone for per reserve_encoder thread. */
	struct semaphore wait_sem;
	/* the condition for wait. Equal to the "core_info". */
	u32 wait_cond;
	/* list node. */
	struct list_head wait_list;
};
static struct list_head reserve_header;
static struct wait_list_node res_wait_node[WAIT_NODE_NUM];

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

static u32 request_wait_node(struct wait_list_node **node, u32 start_id)
{
	u32 i;
	struct wait_list_node *temp_node;

	while (1) {
		for (i = start_id; i < WAIT_NODE_NUM; i++) {
			temp_node = &res_wait_node[i];
			if (temp_node->used_flag == 0) {
				temp_node->used_flag = 1;
				*node = temp_node;
				return i;
			}
		}
		wait_delay(10);
	}
}

static void request_wait_sema(struct wait_list_node **node)
{
	u32 i;
	struct wait_list_node *temp_node;

	while (1) {
		for (i = 0; i < WAIT_NODE_NUM; i++) {
			temp_node = &res_wait_node[i];
			if ((temp_node->used_flag == 0) &&
				(temp_node->sem_used == 0)) {
				temp_node->sem_used = 1;
				*node = temp_node;
				return;
			}
		}
		wait_delay(10);
	}
}

static void init_wait_node(struct wait_list_node *node, u32 cond, u32 sem_flag)
{
	node->used_flag = 0;
	node->wait_cond = cond;
	sema_init(&node->wait_sem, sem_flag);
	INIT_LIST_HEAD(&node->wait_list);
	if (sem_flag > 0)
		node->sem_used = 1;
}

static void init_reserve_wait(u32 dev_num)
{
	u32 i;
	u32 cond = 0x80000001;
	u32 sem_flag = 0;
	struct wait_list_node *node;

	INIT_LIST_HEAD(&reserve_header);

	for (i = 0; i < WAIT_NODE_NUM; i++) {
		if (i < dev_num)
			sem_flag = 1;
		else
			sem_flag = 0;
		node = &res_wait_node[i];
		node->node_id = i;
		init_wait_node(node, cond, sem_flag);
	}
}
#endif
/********variables declaration related with race condition**********/

struct semaphore enc_core_sem;
DECLARE_WAIT_QUEUE_HEAD(enc_hw_queue);
DEFINE_SPINLOCK(enc_owner_lock);
DECLARE_WAIT_QUEUE_HEAD(enc_wait_queue);

/***********************PORTING LAYER**********************************/
/* 0:no resource sharing inter subsystems 1: existing resource sharing */
#define RESOURCE_SHARED_INTER_SUBSYS		0
/* customer specify according to own platform */
#define SUBSYS_0_IO_ADDR			0xe7830000
/* bytes */
#define SUBSYS_0_IO_SIZE			(40000 * 4)

#define INT_PIN_SUBSYS_0_VC8000E		-1

struct core_config {
	u32 subsys_idx;
	u32 core_type;
	unsigned long offset;
	u32 reg_size;
	int irq;
};

struct subsys_config {
	u64 base_addr;
	u32 iosize;
	/*
	 * indicate the core share resources with
	 * other cores or not.If 1, means cores can
	 * not work at the same time.
	 */
	u32 resouce_shared;
	struct clk *clk_enc_axi;		/* clock enc axi */
	struct clk *clk_enc_core;		/* clock enc core */
	struct clk *clk_enc_apb;		/* clock enc apb */
	struct reset_control *rst_enc_axi;	/* reset enc axi */
	struct reset_control *rst_enc_core;	/* reset enc core */
	struct reset_control *rst_enc_apb;	/* reset enc apb */
	struct device *dev;
};

struct subsys_data {
	struct subsys_config cfg;
	struct subsys_core_info core_info;
};

/*
 * for all subsystem, the subsys info should be listed here
 * for subsequent use base_addr, iosize, resource_shared
 */
static struct subsys_config subsys_array[] = {
	/* subsys_0 */
	{SUBSYS_0_IO_ADDR, SUBSYS_0_IO_SIZE, RESOURCE_SHARED_INTER_SUBSYS},
};
size_t sizeof_subsys_array = sizeof(subsys_array)/sizeof(struct subsys_config);

/*
 * here config every core in all subsystem
 * NOTE: no matter what format(HEVC/H264/JPEG/AV1/...) is supported
 *       in VC8000E, just use [CORE_VC8000E] to indicate it's a
 *       VC8000E core CUTREE can work standalone, so it can be a
 *       subsytem or just one core of a subsytem.
 *       subsys_idx, core_type, offset, reg_size, irq
 */

static struct core_config core_array[] = {
	/* subsys_0_VC8000E */
	{0, CORE_VC8000E, 0, 500 * 4, INT_PIN_SUBSYS_0_VC8000E},
};
size_t sizeof_core_array = sizeof(subsys_array)/sizeof(struct subsys_config);

/* parse vc8000e dts and update to related structures */
static int getParametersFromDeviceTree(struct platform_device *pdev)
{
	struct resource *res;
	int i;
	unsigned long base_addr = 0;
	unsigned long base_iosize = 0;
	struct device *dev = &pdev->dev;

	if (sizeof_subsys_array <= 0 || sizeof_core_array <= 0)
		return -1;

	/* reg */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "base");
	if (res != NULL) {
		base_addr = res->start;
		base_iosize = resource_size(res);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "enc");
	if (res != NULL) {
		subsys_array[0].base_addr = base_addr + res->start;
		subsys_array[0].iosize = resource_size(res);
		subsys_array[0].resouce_shared = 0; /* not share */
	}

#ifdef SUPPORT_SFBC
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sfbcdec");
	if (res != NULL) {
		unsigned long sfbc_iosize = resource_size(res);
		/* expand the region to include sfbcdec */
		subsys_array[0].iosize =
			base_iosize > (res->start + sfbc_iosize) ?
				base_iosize : (res->start + sfbc_iosize);

		for (i = 0; i < sizeof_core_array; i++) {
			if (core_array[i].subsys_idx == 0 &&
				core_array[i].core_type == CORE_VC8000E) {
				/* valid region */
				core_array[i].reg_size =
					res->start + resource_size(res);
				break;
			}
		}
	}
#endif
	/* irq */
	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "ENC");
	if (res != NULL) {
		for (i = 0; i < sizeof_core_array; i++) {
			if (core_array[i].subsys_idx == 0 &&
				core_array[i].core_type == CORE_VC8000E) {
				core_array[i].irq = res->start;
				break;
			}
		}
	}

	/* clock & reset - enc */
	subsys_array[0].clk_enc_axi =
			of_clk_get_by_name(pdev->dev.of_node, "axi");
	if (IS_ERR(subsys_array[0].clk_enc_axi))
		return PTR_ERR(subsys_array[0].clk_enc_axi);

	subsys_array[0].clk_enc_core =
			of_clk_get_by_name(pdev->dev.of_node, "core");
	if (IS_ERR(subsys_array[0].clk_enc_core))
		return PTR_ERR(subsys_array[0].clk_enc_core);

	subsys_array[0].clk_enc_apb =
			of_clk_get_by_name(pdev->dev.of_node, "apb");
	if (IS_ERR(subsys_array[0].clk_enc_apb))
		return PTR_ERR(subsys_array[0].clk_enc_apb);

	subsys_array[0].rst_enc_axi =
		devm_reset_control_get_optional_exclusive(dev, "axi");
	if (PTR_ERR(subsys_array[0].rst_enc_axi) == -EPROBE_DEFER)
		return PTR_ERR(subsys_array[0].rst_enc_axi);

	subsys_array[0].rst_enc_core =
		devm_reset_control_get_optional_exclusive(dev, "core");
	if (PTR_ERR(subsys_array[0].rst_enc_core) == -EPROBE_DEFER)
		return PTR_ERR(subsys_array[0].rst_enc_core);

	subsys_array[0].rst_enc_apb =
		devm_reset_control_get_optional_exclusive(dev, "apb");
	if (PTR_ERR(subsys_array[0].rst_enc_apb) == -EPROBE_DEFER)
		return PTR_ERR(subsys_array[0].rst_enc_apb);

	subsys_array[0].dev = dev;

	return 0;
}

/*****************************END*************************************/

/***************************TYPE AND FUNCTION DECLARATION****************/

/* here's all the must remember stuff */
struct dma_buf_info {
	int fd;                /* dma_buf fd */
	dma_addr_t dma_addr;   /* dma address */
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	struct list_head list;
	struct file *owner;    /* owner of used buffer */
};

struct hantroenc {
	/* config of each core,such as base addr, iosize,etc */
	struct subsys_data subsys_data;
	/* VC8000E/VC8000EJ hw id to indicate project */
	u32 hw_id;
	/* subsys id for driver and sw internal use */
	u32 subsys_id;
	/* indicate this subsys is hantro's core or not */
	u32 is_valid;
	/* indicate which process is occupying the subsys */
	int pid[CORE_MAX];
	/* record opened fd */
	struct file *enc_owner[CORE_MAX];
	/* indicate this subsys is occupied by user or not */
	u32 is_reserved[CORE_MAX];
	/* indicate which core receives irq */
	u32 irq_received[CORE_MAX];
	/* IRQ status of each core */
	u32 irq_status[CORE_MAX];
	u32 job_id[CORE_MAX];
	char *buffer;
	unsigned int buffsize;
	u8 *hwregs;
	struct fasync_struct *async_queue;
	struct list_head dma_buf_list;
	struct mutex dmabuf_list_lock;
};

static int ReserveIO(void);
static void ReleaseIO(void);

#ifdef HANTROENC_DEBUG
static void dump_regs(unsigned long data);
#endif

static irqreturn_t hantroenc_isr(int irq, void *dev_id);

/*********************local variable declaration*****************/
unsigned long sram_base;
unsigned int sram_size;
/* and this is our MAJOR; use 0 for dynamic allocation (recommended) */
unsigned int hantroenc_major;
static int total_subsys_num;
static int total_core_num;
/* dynamic allocation */
static struct hantroenc *hantroenc_data;
static atomic_t channels = ATOMIC_INIT(0);

/****************************************************************/
static int CheckEncIrq(struct hantroenc *dev, u32 *core_info,
		       u32 *irq_status, u32 *job_id)
{
	unsigned long flags;
	int rdy = 0;
	u8 core_type = 0;
	u8 subsys_idx = 0;

	core_type = (u8)(*core_info & 0x0F);
	subsys_idx = (u8)(*core_info >> 4);

	if (subsys_idx > total_subsys_num - 1) {
		*core_info = -1;
		*irq_status = 0;
		return 1;
	}

	spin_lock_irqsave(&enc_owner_lock, flags);

	if (dev[subsys_idx].irq_received[core_type]) {
		/* reset the wait condition(s) */
		PDEBUG("check subsys[%d][%d] irq ready\n",
			subsys_idx, core_type);
		rdy = 1;
		*core_info = subsys_idx;
		*irq_status = dev[subsys_idx].irq_status[core_type];
		if (job_id != NULL)
			*job_id = dev[subsys_idx].job_id[core_type];
	}

	spin_unlock_irqrestore(&enc_owner_lock, flags);

	return rdy;
}

static unsigned int WaitEncReady(struct hantroenc *dev, u32 *core_info,
				u32 *irq_status)
{
	PDEBUG("%s\n", __func__);

	if (wait_event_interruptible(enc_wait_queue,
				CheckEncIrq(dev,
				core_info, irq_status, NULL))) {
		PDEBUG("ENC wait_event_interruptible interrupted\n");
		return -ERESTARTSYS;
	}

	return 0;
}

static int CheckEncIrqbyPolling(struct hantroenc *dev, u32 *core_info,
				u32 *irq_status, u32 *job_id)
{
	unsigned long flags;
	int rdy = 0;
	u8 core_type = 0;
	u8 subsys_idx = 0;
	u32 irq, hwId, majorId, wClr;
	unsigned long reg_offset = 0;
	u32 loop = 300;
	u32 enable_status = 0;
	/*
	 * change the interval from 100ms to 1ms,
	 * or it may suspend the encoding process too long
	 */
	u32 interval = 1;

	core_type = (u8)(*core_info & 0x0F);
	subsys_idx = (u8)(*core_info >> 4);

	if (subsys_idx > total_subsys_num-1) {
		*core_info = -1;
		*irq_status = 0;
		return 1;
	}

	do {
		spin_lock_irqsave(&enc_owner_lock, flags);
		if (dev[subsys_idx].is_reserved[core_type] == 0) {
			goto end_1;
		} else if (dev[subsys_idx].irq_received[core_type] &&
			  (dev[subsys_idx].irq_status[core_type] &
				(ASIC_STATUS_FUSE_ERROR |
					ASIC_STATUS_HW_TIMEOUT |
					ASIC_STATUS_BUFF_FULL |
					ASIC_STATUS_HW_RESET |
					ASIC_STATUS_ERROR |
					ASIC_STATUS_FRAME_READY))) {
			rdy = 1;
			*core_info = subsys_idx;
			*irq_status = dev[subsys_idx].irq_status[core_type];
			*job_id = dev[subsys_idx].job_id[core_type];
			goto end_1;
		}

		reg_offset =
			dev[subsys_idx].subsys_data.core_info.offset[core_type];
		irq = (u32)ioread32((void *)(dev[subsys_idx].hwregs +
						reg_offset + 0x04));

		enable_status = (u32)ioread32((void *)(dev[subsys_idx].hwregs +
							reg_offset + 20));

		if (irq & ASIC_STATUS_ALL) {
			PDEBUG("check subsys[%d][%d] irq ready\n",
				subsys_idx, core_type);
			if (irq & 0x20)
				iowrite32(0, (void *)(dev[subsys_idx].hwregs +
						reg_offset + 0x14));

			/*
			 * clear all IRQ bits. (hwId >= 0x80006100)
			 * means IRQ is cleared by writting 1
			 */
			hwId = ioread32((void *)dev[subsys_idx].hwregs +
						reg_offset);
			majorId = (hwId & 0x0000FF00) >> 8;
			wClr = (majorId >= 0x61) ? irq : (irq & (~0x1FD));
			iowrite32(wClr, (void *)(dev[subsys_idx].hwregs +
						reg_offset + 0x04));

			rdy = 1;
			*core_info = subsys_idx;
			*irq_status = irq;
			dev[subsys_idx].irq_received[core_type] = 1;
			dev[subsys_idx].irq_status[core_type] = irq;
			*job_id = dev[subsys_idx].job_id[core_type];
			goto end_1;
		}

		spin_unlock_irqrestore(&enc_owner_lock, flags);
		/*
		 * CAUTION: VSI uses busy wait mdelay() here,
		 * for accurate time delay.
		 */
		mdelay(interval);
	} while (loop--);
	goto end_2;

end_1:
	spin_unlock_irqrestore(&enc_owner_lock, flags);
end_2:
	return rdy;
}

static int CheckEncAnyIrq(struct hantroenc *dev, struct core_wait_out *out)
{
	u32 i;
	int rdy = 0;
	u32 core_info, irq_status, job_id;
	u32 core_type = CORE_VC8000E;

	for (i = 0; i < total_subsys_num; i++) {
		if (!(dev[i].subsys_data.core_info.type_info & (1<<core_type)))
			continue;

		core_info = ((i << 4) | core_type);
		if ((CheckEncIrqbyPolling(dev, &core_info,
				&irq_status, &job_id) == 1) &&
				(core_info == i)) {
			out->job_id[out->irq_num] = job_id;
			out->irq_status[out->irq_num] = irq_status;
			out->irq_num++;
			rdy = 1;
		}
	}

	return rdy;
}

static unsigned int WaitEncAnyReady(struct hantroenc *dev,
				    struct core_wait_out *out)
{
	if (wait_event_interruptible(enc_wait_queue,
					CheckEncAnyIrq(dev, out))) {
		PDEBUG("ENC wait_event_interruptible interrupted\n");
		return -ERESTARTSYS;
	}

	return 0;
}

static int CheckCoreOccupation(struct hantroenc *dev, u8 core_type,
				struct file *filp)
{
	int ret = 0;
	unsigned long flags;

	core_type = (core_type == CORE_VC8000EJ ? CORE_VC8000E : core_type);

	spin_lock_irqsave(&enc_owner_lock, flags);
	if (!dev->is_reserved[core_type]) {
		dev->is_reserved[core_type] = 1;
#ifndef MULTI_THR_TEST
		dev->pid[core_type] = current->pid;
		dev->enc_owner[core_type] = filp;
#endif
		ret = 1;
		PDEBUG("%s pid=%d\n", __func__, dev->pid[core_type]);
	}

	spin_unlock_irqrestore(&enc_owner_lock, flags);

	return ret;
}

static int GetWorkableCore(struct hantroenc *dev,
			   u32 *core_info,
			   u32 *core_info_tmp,
			   struct file *filp)
{
	int ret = 0;
	u32 i = 0;
	u32 cores;
	u8 core_type = 0;
	u32 required_num = 0;
	static u32 reserved_job_id;
	unsigned long flags;

	cores = *core_info;
	required_num = ((cores >> CORE_INFO_AMOUNT_OFFSET) & 0x7)+1;
	core_type = (u8)(cores&0xFF);

	if (*core_info_tmp == 0)
		*core_info_tmp = required_num << CORE_INFO_AMOUNT_OFFSET;
	else
		required_num = (*core_info_tmp >> CORE_INFO_AMOUNT_OFFSET);

	PDEBUG("%s:required_num=%d,core_info=%x\n", __func__,
		required_num, *core_info);

	if (required_num == 0) {
		ret = 1;
		return ret;
	}
	/* a valid free Core with specified core type */
	for (i = 0; i < total_subsys_num; i++) {
		if (dev[i].subsys_data.core_info.type_info & (1 << core_type)) {
			core_type = (core_type == CORE_VC8000EJ ?
					CORE_VC8000E : core_type);
			if (dev[i].is_valid &&
				CheckCoreOccupation(&dev[i], core_type, filp)) {
				*core_info_tmp =
					((((*core_info_tmp >> CORE_INFO_AMOUNT_OFFSET)-1) <<
						CORE_INFO_AMOUNT_OFFSET) |
							(*core_info_tmp & 0x0FF));
				*core_info_tmp = (*core_info_tmp | (1 << i));

				if ((*core_info_tmp >> CORE_INFO_AMOUNT_OFFSET) == 0) {
					ret = 1;
					spin_lock_irqsave(&enc_owner_lock, flags);
					*core_info = (reserved_job_id << 16) |
							(*core_info_tmp & 0xFF);
					dev[i].job_id[core_type] = reserved_job_id;
					/* maintain job_id in 16 bits for core_info can
					 * only save job_id in high 16 bits
					 */
					reserved_job_id = (reserved_job_id + 1) & 0xFFFF;
					spin_unlock_irqrestore(&enc_owner_lock, flags);
					*core_info_tmp = 0;
					required_num = 0;
					break;
				}
			}
		}
	}

	PDEBUG("*core_info = %x\n", *core_info);
	return ret;
}

static long ReserveEncoder(struct hantroenc *dev, u32 *core_info,
			   struct file *filp)
{
	u32 core_info_tmp = 0;
#ifdef MULTI_THR_TEST
	struct wait_list_node *wait_node;
	u32 start_id = 0;
#endif

	/*
	 * If HW resources are shared inter cores,
	 * just make sure only one is using the HW
	 */
	if (dev[0].subsys_data.cfg.resouce_shared) {
		if (down_interruptible(&enc_core_sem))
			return -ERESTARTSYS;
	}

#ifdef MULTI_THR_TEST
	while (1) {
		start_id = request_wait_node(&wait_node, start_id);
		if (wait_node->sem_used == 1) {
			if (GetWorkableCore(dev,
					core_info,
					&core_info_tmp, filp)) {
				down_interruptible(&wait_node->wait_sem);
				wait_node->sem_used = 0;
				wait_node->used_flag = 0;
				break;
			}
			start_id++;
		} else {
			wait_node->wait_cond = *core_info;
			list_add_tail(&wait_node->wait_list, &reserve_header);
			down_interruptible(&wait_node->wait_sem);
			*core_info = wait_node->wait_cond;
			list_del(&wait_node->wait_list);
			wait_node->sem_used = 0;
			wait_node->used_flag = 0;
			break;
		}
	}
#else
	/* lock a core that has specified core id */
	if (wait_event_interruptible(enc_hw_queue,
		(GetWorkableCore(dev, core_info, &core_info_tmp, filp) != 0)))
		return -ERESTARTSYS;
#endif
	return 0;
}

static void ReleaseEncoder(struct hantroenc *dev, u32 *core_info,
			   struct file *filp)
{
	unsigned long flags;
	u8 core_type = 0, subsys_idx = 0, unCheckPid = 0;

	unCheckPid = (u8)((*core_info) >> 31);
#ifdef MULTI_THR_TEST
	u32 release_ok = 0;
	struct list_head *node;
	struct wait_list_node *wait_node;
	u32 core_info_tmp = 0;
#endif
	subsys_idx = (u8)((*core_info&0xF0) >> 4);
	core_type = (u8)(*core_info&0x0F);

	PDEBUG("%s:subsys_idx=%d,core_type=%x\n",
		__func__, subsys_idx, core_type);

	/* release specified subsys and core type */
	if (dev[subsys_idx].subsys_data.core_info.type_info &
		(1 << core_type)) {
		core_type = (core_type == CORE_VC8000EJ ?
				CORE_VC8000E : core_type);
		spin_lock_irqsave(&enc_owner_lock, flags);
		PDEBUG("subsys[%d].pid[%d]=%d,current->pid=%d\n",
			subsys_idx,
			core_type,
			dev[subsys_idx].pid[core_type],
			current->pid);
#ifdef MULTI_THR_TEST
		if (dev[subsys_idx].is_reserved[core_type]) {
#else
		if (dev[subsys_idx].is_reserved[core_type] &&
			(dev[subsys_idx].pid[core_type] == current->pid ||
				unCheckPid == 1)) {
#endif
			dev[subsys_idx].pid[core_type] = -1;
			dev[subsys_idx].is_reserved[core_type] = 0;
			dev[subsys_idx].irq_received[core_type] = 0;
			dev[subsys_idx].irq_status[core_type] = 0;
			dev[subsys_idx].job_id[core_type]  = 0;
			spin_unlock_irqrestore(&enc_owner_lock, flags);
#ifdef MULTI_THR_TEST
			release_ok = 0;

			if (list_empty(&reserve_header)) {
				request_wait_sema(&wait_node);
				up(&wait_node->wait_sem);
			} else {
				list_for_each(node, &reserve_header) {
					wait_node = container_of(node, struct wait_list_node, wait_list);
					if ((GetWorkableCore(dev, &wait_node->wait_cond, &core_info_tmp, filp)) &&
							(wait_node->sem_used == 0)) {
						release_ok = 1;
						wait_node->sem_used = 1;
						up(&wait_node->wait_sem);
						break;
					}
				}
				if (release_ok == 0) {
					request_wait_sema(&wait_node);
					up(&wait_node->wait_sem);
				}
			}
#endif

		} else {
			if (dev[subsys_idx].pid[core_type] !=
					current->pid && unCheckPid == 0)
				pr_err("WARNING:pid(%d) is trying to release core reserved by pid(%d)\n",
					current->pid,
					dev[subsys_idx].pid[core_type]);
			spin_unlock_irqrestore(&enc_owner_lock, flags);
		}
	}
#ifndef MULTI_THR_TEST
	wake_up_interruptible_all(&enc_hw_queue);
#endif
	if (dev->subsys_data.cfg.resouce_shared)
		up(&enc_core_sem);
}

static long hantroenc_ioctl(struct file *filp,
			    unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned int tmp;
	long ret = 0;

	PDEBUG("ioctl cmd 0x%08x\n", cmd);
	/*
	 * extract the type and number bitfields, and don't encode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 */
	if (_IOC_TYPE(cmd) != HANTRO_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_TYPE(cmd) == HANTRO_IOC_MAGIC &&
			_IOC_NR(cmd) > HANTRO_IOC_MAXNR)
		return -ENOTTY;

	err = !access_ok((void *) arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(HANTRO_IOCG_HWOFFSET): {
		u32 id;

		__get_user(id, (u32 *)arg);

		if (id >= total_subsys_num)
			return -EFAULT;

		__put_user(hantroenc_data[id].subsys_data.cfg.base_addr,
				(unsigned long *)arg);
		break;
		}
	case _IOC_NR(HANTRO_IOCG_HWIOSIZE): {
		u32 id;
		u32 io_size;

		__get_user(id, (u32 *)arg);

		if (id >= total_subsys_num)
			return -EFAULT;

		io_size = hantroenc_data[id].subsys_data.cfg.iosize;
		__put_user(io_size, (u32 *) arg);

		return 0;
		}
	case _IOC_NR(HANTRO_IOCG_SRAMOFFSET):
		__put_user(sram_base, (unsigned long *) arg);
		break;
	case _IOC_NR(HANTRO_IOCG_SRAMEIOSIZE):
		__put_user(sram_size, (unsigned int *) arg);
		break;
	case _IOC_NR(HANTRO_IOCG_CORE_NUM):
		__put_user(total_subsys_num, (unsigned int *) arg);
		break;
	case _IOC_NR(HANTRO_IOCG_CORE_INFO): {
		u32 idx;
		struct subsys_core_info in_data;

		ret = copy_from_user(&in_data,
				(void *)arg,
				sizeof(struct subsys_core_info));
		if (ret) {
			pr_err("copy_from_user failed, returned %li\n", ret);
			return -EFAULT;
		}
		idx = in_data.type_info;
		if (idx > total_subsys_num - 1)
			return -1;

		ret = copy_to_user((void *)arg,
				&hantroenc_data[idx].subsys_data.core_info,
				sizeof(struct subsys_core_info));
		if (ret) {
			pr_err("copy_from_user failed, returned %li\n", ret);
			return -EFAULT;
		}
		break;
		}
	case _IOC_NR(HANTRO_IOCH_ENC_RESERVE): {
		u32 core_info;
		int ret;

		PDEBUG("Reserve ENC Cores\n");
		__get_user(core_info, (u32 *)arg);
		ret = ReserveEncoder(hantroenc_data, &core_info, filp);
		if (ret == 0)
			__put_user(core_info, (u32 *) arg);
		return ret;
		}
	case _IOC_NR(HANTRO_IOCH_ENC_RELEASE): {
		u32 core_info;

		__get_user(core_info, (u32 *)arg);

		PDEBUG("Release ENC Core\n");

		ReleaseEncoder(hantroenc_data, &core_info, filp);

		break;
		}
	case _IOC_NR(HANTRO_IOCG_CORE_WAIT): {
		u32 core_info;
		u32 irq_status;

		__get_user(core_info, (u32 *)arg);

		tmp = WaitEncReady(hantroenc_data, &core_info, &irq_status);
		if (tmp == 0) {
			__put_user(irq_status, (unsigned int *)arg);
			return core_info;/* return core_id */
		}
		__put_user(0, (unsigned int *)arg);
		return -1;
		}
	case _IOC_NR(HANTRO_IOCG_ANYCORE_WAIT): {
		struct core_wait_out out;

		memset(&out, 0, sizeof(struct core_wait_out));

		tmp = WaitEncAnyReady(hantroenc_data, &out);
		if (tmp == 0) {
			ret = copy_to_user((void *)arg,
						&out,
						sizeof(struct core_wait_out));
			if (ret) {
				pr_err("copy_to_user failed, returned %li\n",
					ret);
				return -EFAULT;
			}
			return 0;
		} else
			return -1;

		break;
		}
	case _IOC_NR(HANTRO_IOCX_DMA_BUF_ATTACH): {
		struct dma_buf *dmabuf;
		struct dma_buf_attachment *attachment = NULL;
		struct dma_buf_desc buf_data;
		struct sg_table *sgt = NULL;
		struct dma_buf_info *mapping_info = NULL;

		if (copy_from_user(&buf_data,
				   (void __user *)arg,
				   sizeof(buf_data)))
			return -EFAULT;

		buf_data.dma_addr = 0;
		dmabuf = dma_buf_get(buf_data.fd);
		if (!dmabuf || IS_ERR(dmabuf))
			return -EFAULT;

		attachment = dma_buf_attach(dmabuf, subsys_array[0].dev);
		if (!attachment || IS_ERR(attachment))
			return -EFAULT;

		/* update buf info */
		sgt = dma_buf_map_attachment(attachment, DMA_BIDIRECTIONAL);
		if (sgt && !IS_ERR(sgt)) {
			buf_data.dma_addr = sg_dma_address(sgt->sgl);
		} else {
			pr_err("Error: table sgt is null!\n");
			dma_buf_detach(dmabuf, attachment);
			dma_buf_put(dmabuf);
			return -EFAULT;
		}

		/* create and store current dma buf info */
		mapping_info = kzalloc(sizeof(struct dma_buf_info),
					GFP_KERNEL);

		if (!mapping_info) {
			pr_err("Error: No enough space!\n");
			dma_buf_unmap_attachment(attachment,
						sgt, DMA_BIDIRECTIONAL);
			dma_buf_detach(dmabuf, attachment);
			dma_buf_put(dmabuf);
			return -EFAULT;
		}

#ifdef DMA_DEBUG_ENC
		struct scatterlist *sgl = sgt->sgl;

		while (sgl != NULL) {
			PDEBUG("[sgl]addr = 0x%lx, size = %d\n",
				sg_dma_address(sgl),
				sg_dma_len(sgl));
			sgl = sg_next(sgl);
		}
#endif
		mapping_info->fd = buf_data.fd;
		mapping_info->dma_addr = buf_data.dma_addr;
		mapping_info->dmabuf = dmabuf;
		mapping_info->attachment = attachment;
		mapping_info->sgt = sgt;
		mapping_info->owner = filp;

		mutex_lock(&hantroenc_data[0].dmabuf_list_lock);
		list_add(&mapping_info->list, &hantroenc_data[0].dma_buf_list);
		mutex_unlock(&hantroenc_data[0].dmabuf_list_lock);

		ret = copy_to_user((void __user *)arg,
				   &buf_data, sizeof(buf_data));
		if (ret) {
			pr_err("copy_to_user failed, returned %li\n", ret);
			return -EFAULT;
		}

		break;
		}
	case _IOC_NR(HANTRO_IOCX_DMA_BUF_DETACH): {
		struct dma_buf_desc buf_data;
		struct dma_buf_info *mapping_info;
		bool found = false;

		if (copy_from_user(&buf_data,
				(void __user *)arg,
				sizeof(buf_data)))
			return -EFAULT;

		/*
		 * find the matched dma_buf_info,
		 * judged by dma_addr, update the fd.
		 */
		mutex_lock(&hantroenc_data[0].dmabuf_list_lock);
		list_for_each_entry(mapping_info,
				&hantroenc_data[0].dma_buf_list,
				list) {
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
						mapping_info->sgt,
						DMA_BIDIRECTIONAL);
			dma_buf_detach(mapping_info->dmabuf,
					mapping_info->attachment);
			dma_buf_put(mapping_info->dmabuf);

			list_del_init(&mapping_info->list);
			kfree(mapping_info);
		}
		mutex_unlock(&hantroenc_data[0].dmabuf_list_lock);

		ret = copy_to_user((void __user *)arg,
				   &buf_data, sizeof(buf_data));
		if (ret) {
			pr_err("copy_to_user failed, returned %li\n", ret);
			return -EFAULT;
		}

		break;
		}

	default:
		break;
	}
	return 0;
}

#ifdef CONFIG_COMPAT
static long hantroenc_ioctl_compat(struct file *filp,
				   unsigned int cmd, unsigned long arg)
{
	return hantroenc_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static void ReleaseMemory(struct file *filp)
{
	struct dma_buf_info *mapping_info, *tmp;

	PDEBUG("memory_release\n");
	mutex_lock(&hantroenc_data[0].dmabuf_list_lock);
	list_for_each_entry_safe(mapping_info, tmp,
				&hantroenc_data[0].dma_buf_list, list) {
		if (mapping_info->owner == filp) {
			PDEBUG("detach buf addr = 0x%llx\n",
				mapping_info->dma_addr);
			dma_buf_unmap_attachment(mapping_info->attachment,
						mapping_info->sgt,
						DMA_BIDIRECTIONAL);
			dma_buf_detach(mapping_info->dmabuf,
					mapping_info->attachment);
			dma_buf_put(mapping_info->dmabuf);

			list_del_init(&mapping_info->list);
			kfree(mapping_info);
		}
	}
	mutex_unlock(&hantroenc_data[0].dmabuf_list_lock);
}

static int hantroenc_open(struct inode *inode, struct file *filp)
{
	int result = 0;
	struct hantroenc *dev = hantroenc_data;

	filp->private_data = (void *) dev;

	pm_runtime_get_sync(subsys_array[0].dev);
	atomic_inc(&channels);

	PDEBUG("dev opened\n");
	return result;
}

static int hantroenc_release(struct inode *inode, struct file *filp)
{
	struct hantroenc *dev = (struct hantroenc *) filp->private_data;
	u32 core_id = 0, i = 0;
	unsigned long flags;

#ifdef SUPPORT_SFBC
	/*
	 * Check if there is an error flag bit set to 1 in the
	 * SFBCDEC_IRQ_RAW_STATUS register, and if so, perform
	 * a quick stop processing.
	 */
	u32 sfbcStatus;
	sfbcStatus = (u32) ioread32((void *)(dev->hwregs +
		HANTRO_SFBCDEC_REG_BASE_OFFSET + HANTRO_SFBCDEC_IRQ_RAW_STATUS));
	if ((sfbcStatus >> 2) != 0) {
		iowrite32(0x4, (void *)(dev->hwregs +
		HANTRO_SFBCDEC_REG_BASE_OFFSET + HANTRO_SFBCDEC_COMMAND));
	}
#endif

#ifdef HANTROENC_DEBUG
	dump_regs((unsigned long) dev); /* dump the regs */
#endif

	PDEBUG("dev closed\n");

	for (i = 0; i < total_subsys_num; i++) {
		for (core_id = 0; core_id < CORE_MAX; core_id++) {
			spin_lock_irqsave(&enc_owner_lock, flags);
			if (dev[i].is_reserved[core_id] == 1 &&
				(dev[i].pid[core_id] == current->pid ||
				dev[i].enc_owner[core_id] == filp)) {
				dev[i].pid[core_id] = -1;
				dev[i].is_reserved[core_id] = 0;
				dev[i].irq_received[core_id] = 0;
				dev[i].irq_status[core_id] = 0;
				dev[i].enc_owner[core_id] = NULL;
				PDEBUG("release reserved core\n");
			}
			spin_unlock_irqrestore(&enc_owner_lock, flags);
		}
	}

	ReleaseMemory(filp);

	wake_up_interruptible_all(&enc_hw_queue);

	if (dev->subsys_data.cfg.resouce_shared)
		up(&enc_core_sem);

	atomic_dec(&channels);
	pm_runtime_put_autosuspend(subsys_array[0].dev);
	return 0;
}

/* enable/disable clock */
static void hantroenc_deassert_clk(struct device *dev)
{
	reset_control_deassert(subsys_array[0].rst_enc_axi);
	reset_control_deassert(subsys_array[0].rst_enc_core);
	reset_control_deassert(subsys_array[0].rst_enc_apb);
}

static void hantroenc_enable_clk(struct device *dev, bool skip_shared)
{
	if (!skip_shared)
		clk_prepare_enable(subsys_array[0].clk_enc_axi);
	clk_prepare_enable(subsys_array[0].clk_enc_core);
	clk_prepare_enable(subsys_array[0].clk_enc_apb);
}

static void hantroenc_assert_clk(struct device *dev)
{
	reset_control_assert(subsys_array[0].rst_enc_axi);
	reset_control_assert(subsys_array[0].rst_enc_core);
	reset_control_assert(subsys_array[0].rst_enc_apb);
}

static void hantroenc_disable_clk(struct device *dev, bool skip_shared)
{
	if (!skip_shared)
		clk_disable_unprepare(subsys_array[0].clk_enc_axi);
	clk_disable_unprepare(subsys_array[0].clk_enc_core);
	clk_disable_unprepare(subsys_array[0].clk_enc_apb);
}

static int hantroenc_mmap(struct file *filp, struct vm_area_struct *vm)
{
	if (vm->vm_pgoff ==
		(hantroenc_data[0].subsys_data.cfg.base_addr >> PAGE_SHIFT)) {
		vm->vm_flags |= VM_IO;
		vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
		PDEBUG("hantroenc mmap: size=0x%lX, page off=0x%lX\n",
			(vm->vm_end - vm->vm_start), vm->vm_pgoff);
		return remap_pfn_range(vm, vm->vm_start,
					vm->vm_pgoff, vm->vm_end - vm->vm_start,
					vm->vm_page_prot) ? -EAGAIN : 0;
	} else {
		pr_err("invalid map offset :0x%lX\n", vm->vm_pgoff);
		return -EINVAL;
	}
}

static const struct file_operations hantroenc_fops = {
	.owner = THIS_MODULE,
	.open = hantroenc_open,
	.release = hantroenc_release,
	.unlocked_ioctl = hantroenc_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = hantroenc_ioctl_compat,
#endif
	.fasync = NULL,
	.mmap = hantroenc_mmap,
};

int hantroenc_normal_init(void)
{
	int result = 0;
	int i, j;

	total_subsys_num = sizeof(subsys_array)/sizeof(struct subsys_config);

	for (i = 0; i < total_subsys_num; i++) {
		pr_info("hantroenc: module init - subsys[%d] addr =0x%llx\n", i,
				subsys_array[i].base_addr);
	}

	hantroenc_data = vmalloc(sizeof(struct hantroenc)*total_subsys_num);
	if (hantroenc_data == NULL)
		goto err1;
	memset(hantroenc_data, 0, sizeof(struct hantroenc)*total_subsys_num);

	for (i = 0; i < total_subsys_num; i++) {
		hantroenc_data[i].subsys_data.cfg = subsys_array[i];
		hantroenc_data[i].async_queue = NULL;
		hantroenc_data[i].hwregs = NULL;
		hantroenc_data[i].subsys_id = i;
		for (j = 0; j < CORE_MAX; j++)
			hantroenc_data[i].subsys_data.core_info.irq[j] = -1;

		INIT_LIST_HEAD(&hantroenc_data[i].dma_buf_list);
		mutex_init(&hantroenc_data[i].dmabuf_list_lock);
	}

	total_core_num = sizeof(core_array)/sizeof(struct core_config);
	for (i = 0; i < total_core_num; i++) {
		hantroenc_data[core_array[i].subsys_idx].subsys_data.core_info.type_info |=
			(1<<(core_array[i].core_type));
		hantroenc_data[core_array[i].subsys_idx].subsys_data.core_info.offset[core_array[i].core_type] =
			core_array[i].offset;
		hantroenc_data[core_array[i].subsys_idx].subsys_data.core_info.regSize[core_array[i].core_type] =
			core_array[i].reg_size;
		hantroenc_data[core_array[i].subsys_idx].subsys_data.core_info.irq[core_array[i].core_type] =
			core_array[i].irq;
	}

	result = register_chrdev(hantroenc_major, DEVICE_NAME, &hantroenc_fops);
	if (result < 0) {
		pr_info("hantroenc: unable to get major <%d>\n",
			 hantroenc_major);
		goto err1;
	} else if (result != 0)    /* this is for dynamic major */
		hantroenc_major = result;

	result = ReserveIO();
	if (result < 0)
		goto err;

	sema_init(&enc_core_sem, 1);

	/* get the IRQ line */
	for (i = 0; i < total_subsys_num; i++) {
		if (hantroenc_data[i].is_valid == 0)
			continue;

		for (j = 0; j < CORE_MAX; j++) {
			if (hantroenc_data[i].subsys_data.core_info.irq[j] !=
				-1) {
				result = request_irq(
						hantroenc_data[i].subsys_data.core_info.irq[j],
						hantroenc_isr,
						IRQF_SHARED,
						DEVICE_NAME,
						(void *)&hantroenc_data[i]);
				if (result == -EINVAL) {
					pr_err("hantroenc: Bad irq number or handler\n");
					ReleaseIO();
					goto err;
				} else if (result == -EBUSY) {
					pr_err("hantroenc: IRQ <%d> busy, change your config\n",
						hantroenc_data[i].subsys_data.core_info.irq[j]);
					ReleaseIO();
					goto err;
				}
			} else
				pr_debug("hantroenc: IRQ not in use!\n");
		}
	}
#ifdef MULTI_THR_TEST
	init_reserve_wait(total_subsys_num);
#endif
	pr_info("hantroenc: module inserted. Major <%d>\n", hantroenc_major);

	return 0;

err:
	unregister_chrdev(hantroenc_major, DEVICE_NAME);
err1:
	if (hantroenc_data != NULL)
		vfree(hantroenc_data);
	pr_info("hantroenc: module not inserted\n");
	return result;
}

void hantroenc_normal_cleanup(void)
{
	int i = 0, j = 0;

	for (i = 0; i < total_subsys_num; i++) {
		if (hantroenc_data[i].is_valid == 0)
			continue;

		/* free the core IRQ */
		for (j = 0; j < total_core_num; j++) {
			if (hantroenc_data[i].subsys_data.core_info.irq[j] !=
					-1)
				free_irq(hantroenc_data[i].subsys_data.core_info.irq[j],
					(void *)&hantroenc_data[i]);
		}
	}

	ReleaseIO();

	vfree(hantroenc_data);
	unregister_chrdev(hantroenc_major, DEVICE_NAME);
	pr_info("hantroenc: module removed\n");
}

static int ReserveIO(void)
{
	u32 hwid;
	int i;
	u32 found_hw = 0, hw_cfg;
	u32 VC8000E_core_idx;

	for (i = 0; i < total_subsys_num; i++) {
		if (!request_mem_region(
				hantroenc_data[i].subsys_data.cfg.base_addr,
				hantroenc_data[i].subsys_data.cfg.iosize,
				DEVICE_NAME)) {
			pr_info("hantroenc: failed to reserve HW regs\n");
			continue;
		}

		hantroenc_data[i].hwregs = (u8 *) ioremap(
							hantroenc_data[i].subsys_data.cfg.base_addr,
							hantroenc_data[i].subsys_data.cfg.iosize);
		if (hantroenc_data[i].hwregs == NULL) {
			pr_info("hantroenc: failed to ioremap HW regs\n");
			ReleaseIO();
			continue;
		}

		/* read hwid and check validness and store it */
		VC8000E_core_idx = GET_ENCODER_IDX(hantroenc_data[0].subsys_data.core_info.type_info);
		hwid = (u32)ioread32((void *)hantroenc_data[i].hwregs +
				hantroenc_data[i].subsys_data.core_info.offset[VC8000E_core_idx]);
		pr_info("hwid=0x%08x\n", hwid);

		/* check for encoder HW ID */
		if (((((hwid >> 16) & 0xFFFF) !=
				((ENC_HW_ID1 >> 16) & 0xFFFF))) &&
			((((hwid >> 16) & 0xFFFF) !=
				((ENC_HW_ID2 >> 16) & 0xFFFF)))) {
			pr_info("hantroenc: HW not found at 0x%llx\n",
				hantroenc_data[i].subsys_data.cfg.base_addr);
#ifdef HANTROENC_DEBUG
			dump_regs((unsigned long) &hantroenc_data);
#endif
			hantroenc_data[i].is_valid = 0;
			ReleaseIO();
			continue;
		}
		hantroenc_data[i].hw_id = hwid;
		hantroenc_data[i].is_valid = 1;
		found_hw = 1;

		hw_cfg = (u32)ioread32((void *)hantroenc_data[i].hwregs +
					hantroenc_data[i].subsys_data.core_info.offset[VC8000E_core_idx] +
					320);
		hantroenc_data[i].subsys_data.core_info.type_info &= 0xFFFFFFFC;
		if (hw_cfg & 0x88000000)
			hantroenc_data[i].subsys_data.core_info.type_info |=
							(1<<CORE_VC8000E);
		if (hw_cfg & 0x00008000)
			hantroenc_data[i].subsys_data.core_info.type_info |=
							(1<<CORE_VC8000EJ);

		pr_info("hantroenc: HW at base <0x%llx> with ID <0x%08x>\n",
			hantroenc_data[i].subsys_data.cfg.base_addr,
			hwid);
	}

	if (found_hw == 0) {
		pr_err("hantroenc: NO ANY HW found!!\n");
		return -1;
	}

	return 0;
}

static void ReleaseIO(void)
{
	u32 i;

	for (i = 0; i <= total_subsys_num; i++) {
		if (hantroenc_data[i].is_valid == 0)
			continue;
		if (hantroenc_data[i].hwregs)
			iounmap((void *) hantroenc_data[i].hwregs);
		release_mem_region(hantroenc_data[i].subsys_data.cfg.base_addr,
				hantroenc_data[i].subsys_data.cfg.iosize);
	}
}

irqreturn_t hantroenc_isr(int irq, void *dev_id)
{
	unsigned int handled = 0;
	struct hantroenc *dev = (struct hantroenc *) dev_id;
	u32 irq_status;
	unsigned long flags;
	u32 core_type = 0, i = 0;
	unsigned long reg_offset = 0;
	u32 hwId, majorId, wClr;

	/* get core id by irq from subsys config */
	for (i = 0; i < CORE_MAX; i++) {
		if (dev->subsys_data.core_info.irq[i] == irq) {
			core_type = i;
			reg_offset = dev->subsys_data.core_info.offset[i];
			break;
		}
	}

	/*
	 * If core is not reserved by any user,
	 * but irq is received, just clean it
	 */
	spin_lock_irqsave(&enc_owner_lock, flags);
	if (!dev->is_reserved[core_type]) {
		pr_debug("%s:received IRQ but core is not reserved!\n",
			__func__);
		irq_status = (u32)ioread32((void *)(dev->hwregs +
					reg_offset + 0x04));
		if (irq_status & 0x01) {
			if (irq_status & 0x20)
				iowrite32(0, (void *)(dev->hwregs +
						reg_offset + 0x14));

			hwId = ioread32((void *)dev->hwregs + reg_offset);
			majorId = (hwId & 0x0000FF00) >> 8;
			wClr = (majorId >= 0x61) ?
					irq_status : (irq_status & (~0x1FD));
			iowrite32(wClr, (void *)(dev->hwregs +
					reg_offset + 0x04));
		}
		spin_unlock_irqrestore(&enc_owner_lock, flags);
		return IRQ_HANDLED;
	}
	spin_unlock_irqrestore(&enc_owner_lock, flags);

	pr_debug("%s:received IRQ!\n", __func__);
	irq_status = (u32)ioread32((void *)(dev->hwregs + reg_offset + 0x04));
	pr_debug("irq_status of subsys %d core %d is:%x\n",
		dev->subsys_id, core_type, irq_status);
	if (irq_status & 0x01) {
		/*
		 * Disable HW when buffer over-flow happen
		 * HW behavior changed in over-flow in-pass,
		 * HW cleanup HWIF_ENC_E auto
		 * new version: ask SW cleanup HWIF_ENC_E when buffer over-flow
		 */
		if (irq_status & 0x20)
			iowrite32(0, (void *)(dev->hwregs + reg_offset + 0x14));

		hwId = ioread32((void *)dev->hwregs + reg_offset);
		majorId = (hwId & 0x0000FF00) >> 8;
		wClr = (majorId >= 0x61) ? irq_status : (irq_status & (~0x1FD));
		iowrite32(wClr, (void *)(dev->hwregs + reg_offset + 0x04));

		spin_lock_irqsave(&enc_owner_lock, flags);
		dev->irq_received[core_type] = 1;
		dev->irq_status[core_type] = irq_status & (~0x01);
		spin_unlock_irqrestore(&enc_owner_lock, flags);

		wake_up_interruptible_all(&enc_wait_queue);
		handled++;
	}
	if (!handled)
		PDEBUG("IRQ received, but not hantro's!\n");

	return IRQ_HANDLED;
}

#ifdef HANTROENC_DEBUG
static void dump_regs(unsigned long data)
{
	struct hantroenc *dev = (struct hantroenc *) data;
	int i, n;

	PDEBUG("Reg Dump Start\n");
	for (n = 0; n < total_subsys_num; n++) {
		if (dev[n].is_valid == 0)
			continue;
		for (i = 0; i < dev[n].subsys_data.cfg.iosize; i += 4)
			PDEBUG("\toffset %02X = %08X\n", i,
				ioread32(dev->hwregs + i));
	}
	PDEBUG("Reg Dump End\n");
}
#endif

/* devfreq and thermal cooling support */
static int hantroenc_devfreq_target(struct device *dev,
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
		clk_set_rate(subsys_array[0].clk_enc_core, *freq);

	return err;
}

static int hantroenc_devfreq_get_dev_status(struct device *dev,
		struct devfreq_dev_status *status)
{
	struct clk *clk = dev_get_drvdata(dev);

	status->busy_time = 0;
	status->total_time = 0;
	status->current_frequency = clk_get_rate(clk);

	return 0;
}

static struct devfreq_dev_profile hantroenc_devfreq_profile = {
	.polling_ms = 1000, /* 1 second */
	.target = hantroenc_devfreq_target,
	.get_dev_status = hantroenc_devfreq_get_dev_status,
};

static int hantroenc_devfreq_register(struct device *dev, struct clk *clk)
{
	int i;
	struct opp_table *opp_table;
	unsigned long max_freq;
	unsigned long cur_freq;
	struct dev_pm_opp *opp;
	struct devfreq *devfreq;
	struct thermal_cooling_device *cooling;

	dev_set_drvdata(dev, clk);

	cur_freq = max_freq = clk_get_rate(clk);

	opp_table = dev_pm_opp_set_clkname(dev, "axi");
	if (IS_ERR(opp_table)) {
		dev_warn(dev, "could not get alloc opp_table\n");
		return PTR_ERR(opp_table);
	}

#define DEV_PM_OPP_MAX 4
	for (i = 0; i < DEV_PM_OPP_MAX; i++) {
		dev_pm_opp_add(dev, cur_freq, DEV_PM_OPP_MAX - i);
		cur_freq >>= 1;
	}

	opp = devfreq_recommended_opp(dev, &max_freq, 0);
	if (IS_ERR(opp)) {
		dev_warn(dev, "could not devfreq_recommended_opp\n");
		return PTR_ERR(opp);
	}

	hantroenc_devfreq_profile.initial_freq = max_freq;
	dev_pm_opp_put(opp);

	devfreq = devm_devfreq_add_device(dev, &hantroenc_devfreq_profile,
				DEVFREQ_GOV_SIMPLE_ONDEMAND, NULL);
	if (IS_ERR(devfreq)) {
		dev_warn(dev, "Couldn't initialize npu1 devfreq\n");
		return PTR_ERR(devfreq);
	}

	cooling = of_devfreq_cooling_register(dev->of_node, devfreq);
	if (IS_ERR(cooling))
		dev_warn(dev, "Failed to register cooling device\n");

	return 0;
}

static int hantroenc_device_probe(struct platform_device *pdev)
{
	struct device *temp_class;

	int ret = getParametersFromDeviceTree(pdev);
	if (ret != 0)
		return ret;

	/* initialise runtime power management */
	pm_runtime_set_autosuspend_delay(&pdev->dev, SUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	/* deassert clock */
	hantroenc_deassert_clk(&pdev->dev);

	/* register devfreq */
	ret = hantroenc_devfreq_register(&pdev->dev, subsys_array[0].clk_enc_axi);
	if (ret != 0)
		goto error;

	ret = hantroenc_normal_init();
	if (ret != 0)
		goto error;

	ret = dma_set_mask_and_coherent(&pdev->dev, 0x7FFFFFFF);
	if (ret != 0)
		goto error;

	hantro_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(hantro_class)) {
		ret = PTR_ERR(hantro_class);
		goto error;
	}

	temp_class = device_create(hantro_class,
				   NULL,
				   MKDEV(hantroenc_major, 0),
				   NULL, DEVICE_NAME);
	if (IS_ERR(temp_class)) {
		ret = PTR_ERR(temp_class);
		goto err_out_class;
	}

	goto out;

err_out_class:
	class_destroy(hantro_class);

error:
	pr_err("hantroenc probe failed\n");

out:
	pm_runtime_put_autosuspend(&pdev->dev);
	return ret;
}

static int hantroenc_device_remove(struct platform_device *pdev)
{
	pm_runtime_get_sync(&pdev->dev);
	if (hantroenc_major > 0) {
		device_destroy(hantro_class, MKDEV(hantroenc_major, 0));
		class_destroy(hantro_class);
		hantroenc_normal_cleanup();
		hantroenc_major = 0;
	}

	/* assert clock */
	hantroenc_assert_clk(&pdev->dev);

	pm_runtime_put_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id hantroenc_dt_ids[] = {
	{ .compatible = "vpu,vc8000e" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, hantroenc_dt_ids);
#endif /* CONFIG_OF */

#ifdef CONFIG_PM
static int hantroenc_suspend(struct device *dev)
{
	if (atomic_read(&channels))
		hantroenc_disable_clk(dev, true);

	return 0;
}

static int hantroenc_resume(struct device *dev)
{
	if (atomic_read(&channels))
		hantroenc_enable_clk(dev, true);

	return 0;
}

static int hantroenc_runtime_suspend(struct device *dev)
{
	hantroenc_disable_clk(dev, true);
	return 0;
}

static int hantroenc_runtime_resume(struct device *dev)
{
	hantroenc_enable_clk(dev, true);
	return 0;
}

static const struct dev_pm_ops hantroenc_pm_ops = {
	SET_RUNTIME_PM_OPS(hantroenc_runtime_suspend,
		hantroenc_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(hantroenc_suspend, hantroenc_resume)
};
#endif //CONFIG_PM

static struct platform_driver hantroenc_platform_driver = {
	.probe = hantroenc_device_probe,
	.remove = hantroenc_device_remove,
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(hantroenc_dt_ids),
#ifdef CONFIG_PM
		.pm = &hantroenc_pm_ops,
#endif
	},
};

#ifdef CONFIG_OF
module_platform_driver(hantroenc_platform_driver);
#else

int __init hantroenc_init(void)
{
	return hantroenc_normal_init();
}

void __exit hantroenc_cleanup(void)
{
	hantroenc_normal_cleanup();
}

module_init(hantroenc_init);
module_exit(hantroenc_cleanup);
#endif /* CONFIG_OF */

/* module description */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Verisilicon");
MODULE_DESCRIPTION("VC8000 driver");

