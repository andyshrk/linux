/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2020 Vivante Corporation
*
*    Permission is hereby granted, free of charge, to any person obtaining a
*    copy of this software and associated documentation files (the "Software"),
*    to deal in the Software without restriction, including without limitation
*    the rights to use, copy, modify, merge, publish, distribute, sublicense,
*    and/or sell copies of the Software, and to permit persons to whom the
*    Software is furnished to do so, subject to the following conditions:
*
*    The above copyright notice and this permission notice shall be included in
*    all copies or substantial portions of the Software.
*
*    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
*    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
*    DEALINGS IN THE SOFTWARE.
*
*****************************************************************************
*
*    The GPL License (GPL)
*
*    Copyright (C) 2014 - 2020 Vivante Corporation
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
*
*****************************************************************************
*
*    Note: This software is released under dual MIT and GPL licenses. A
*    recipient may use this file under the terms of either the MIT license or
*    GPL License. If you wish to use only one license not the other, you can
*    indicate your decision by deleting one of the above license notices in your
*    version of this file.
*
*****************************************************************************/
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>

#include "vg_lite_platform.h"
#include "../vg_lite_kernel.h"
#include "../inc/vg_lite_hal.h"
#include "vg_lite_ioctl.h"
#include "../vg_lite_hw.h"
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/pagemap.h>
#include <linux/version.h>
#include <asm/io.h>
#include <linux/iommu.h>
#include <linux/sched.h>
#include <linux/mman.h>
#include <linux/dma-buf.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,8,0)
#define current_mm_mmap_sem current->mm->mmap_lock
#else
#define current_mm_mmap_sem current->mm->mmap_sem
#endif


MODULE_LICENSE("Dual MIT/GPL");

static void vg_lite_exit(void);

/*#define GPU_REG_START   0x02204000
#define GPU_REG_SIZE    0x00000600
#define GPU_IRQ         43*/
static	uint	registerMemBase	= 0xec100000;
module_param(registerMemBase, uint, 0644);
static	uint	registerMemSize = 0x00000600;
module_param(registerMemSize, uint, 0644);
static	uint	irqLine		= 43;
module_param(irqLine, uint, 0644);

/*There are three ways to allocate internal contiguous buffer, for command buffer usage.
  The order to allocate contiguous memory is
  1. if enable_smmu, allocate dynamically from dma. <contiguousSize> and <heap_size> are ignored;
  2. if contiguousBase != 0, use reserved memory, <contiguousSize> bytes
  3. allocate from heap <heap_size> bytes */
static int enable_smmu = 1; /*alloc from dma first */
module_param(enable_smmu, int, S_IRUGO);
static  uint    contiguousSize = 0x4000000;
module_param(contiguousSize, uint, 0644);
static  uint    contiguousBase = 0;/* default use heap */
module_param(contiguousBase, uint, 0644);
static int heap_size = 32 << 20;    /*SE1000: 1080P suggest 32MB. */
module_param(heap_size, int, S_IRUGO);

static int verbose = 0;
static int cached = 1;

module_param(verbose, int, S_IRUGO);
module_param(cached, int, S_IRUGO);

bool enable_clock(struct device* dev, int enable);

static char* version = "1.0.3";
static ssize_t version_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%s\n", version);
}
static DEVICE_ATTR_RO(version);

static int user_command = 0;
static ssize_t user_command_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", user_command);
}
static ssize_t user_command_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t len)
{
    if (len >0)
        user_command = buf[0] - '0';
    if (user_command == 1) {
        /* wait gpu idle */
        while(VG_LITE_KERNEL_IS_GPU_IDLE() != 1) {
            vg_lite_hal_delay(2);
        }
        /* shutdown gpu */
        kernel_dispatch(VG_LITE_CLOSE, NULL);
        enable_clock(dev, 0);

        pr_info("v2d: suspend success!\n");
   } else if(user_command == 2) {
        /* open power and clock */
        enable_clock(dev, 1);

        /* open gpu interrupt and recovery gpu register */
        kernel_dispatch(VG_LITE_RESET, NULL);
        pr_info("v2d: resume success!\n");
   }

   return len;
}
static DEVICE_ATTR_RW(user_command);
static int init_sysfs(struct device* dev)
{
    int ret;
    ret = sysfs_create_file(&dev->kobj, &dev_attr_version.attr);
    if (ret) {
        pr_err("v2d: %s error = %d\n", __func__, ret);
        return ret;
    }
    ret = sysfs_create_file(&dev->kobj, &dev_attr_user_command.attr);
    if (ret) {
        sysfs_remove_file(&dev->kobj, &dev_attr_version.attr);
    }
    return ret;
}
static void exit_sysfs(struct device* dev)
{
    sysfs_remove_file(&dev->kobj, &dev_attr_version.attr);
}
/******************************************************************************/

#define HEAP_NODE_USED  0xABBAF00D

/* Struct definitions. */
struct heap_node {
    struct list_head list;
    uint32_t offset;
    unsigned long size;
    int32_t status;
};

struct memory_heap {
    uint32_t free;
    struct list_head list;
};

struct mapped_memory {
    void * logical;
    uint32_t physical;
    int page_count;
    struct page ** pages;
    struct sg_table sgt;
};
//SE1000
struct dma_memory {
    void * kva;
    void * uva;
    pid_t process;
    phys_addr_t phys;;
    size_t size;
    struct page ** pages;
    struct vm_area_struct *vma;
    dma_addr_t iova;
};

struct dmabuf_memory {
    uint32_t dmabuf_handle;
    struct sg_table *sgt;
    struct dma_buf_attachment *attachment;
    /* not necessary below*/
    struct dma_buf *dmabuf;
    size_t size;
    uint32_t iova;
};

#define DEVICE_NAME "vg_lite"
struct vg_lite_device {
    void * gpu;             /* Register memory base */
    void * virtual; //kernel virtual address
    int enable_smmu; /* 1 if smmu enabled */
    struct iommu_domain * domain;

    uint32_t physical; //contiguous address
    uint32_t size;

    struct memory_heap heap;
    struct page * pages;
    unsigned int order;

    int irq_enabled;
    volatile uint32_t int_flags;
    wait_queue_head_t int_queue;
    int major;
    struct class * device_class;
    struct device * device;
    struct platform_device *pdev;   /*platform device object */
    struct clk *clk_vg_axi;
    struct clk *clk_pclk;
    struct reset_control *rst_ctrls;
    int clk_state; /* 0 is disbled */
};

struct client_data {
    struct vg_lite_device * vg_obj;
    struct vm_area_struct * vm;
    void * contiguous_mapped;
};



static struct vg_lite_device * vg_device = NULL;
static struct client_data * private_data = NULL;

/***************** SE1000 device tree *****************/
static const struct of_device_id compatibles[] = {
    {  .name = "v2d", .compatible = "siengine,v2d", },
    {},
};
#define MAX_CLOCK_RATE 1600000000
#define AXI_CLOCK "gpu2_v2d_clk"
#define PIXEL_CLK "gpu2_r2d_pclk"
bool enable_clock(struct device* dev, int enable)
{
    pr_info("v2d: enable_clock(%d)\n", enable);
    if (enable) {
        if(vg_device->clk_state == 0) {
            if (dev == NULL || dev->of_node == NULL) {
                pr_err("v2d: device of_node is NULL!\n");
            }
            if (vg_device->clk_vg_axi == NULL) {
                vg_device->clk_vg_axi = clk_get(dev, AXI_CLOCK);
                if (IS_ERR(vg_device->clk_vg_axi)) {
                    vg_device->clk_vg_axi  = NULL;
                    pr_err("v2d: clk_get gpu2_v2d_clk failed!\n");
                    return false;
                }
                clk_prepare(vg_device->clk_vg_axi);
            }
            clk_enable(vg_device->clk_vg_axi);
            /* pclk clock control*/
            if (vg_device->clk_pclk == NULL) {
                vg_device->clk_pclk = clk_get(dev, PIXEL_CLK);
                if (IS_ERR(vg_device->clk_pclk)) {
                    vg_device->clk_pclk  = NULL;
                    pr_err("v2d: clk_get gpu2_r2d_pclk failed!\n");
                    return false;
                }
                clk_prepare(vg_device->clk_pclk);
            }
            clk_enable(vg_device->clk_pclk);

            vg_device->rst_ctrls = devm_reset_control_array_get(dev, true, true);
            if (PTR_ERR(vg_device->rst_ctrls) == -EPROBE_DEFER) {
                pr_err("v2d: devm_reset_control_array_get error!\n");
                return false;
            }
            if (0 != reset_control_deassert(vg_device->rst_ctrls)) {
                pr_err("v2d: devm_reset_control_array_get error!\n");
                return false;
            }

            // wait (32 + 128) cycles for the slowest clock (pclk 200M) before ready, it is about 1us
            udelay(1);
            vg_device->clk_state = 1;
        }
    } else {
        if (vg_device->clk_state) {
            if(vg_device->clk_vg_axi) {
                clk_disable(vg_device->clk_vg_axi);
                clk_unprepare(vg_device->clk_vg_axi);
                clk_put(vg_device->clk_vg_axi);
                vg_device->clk_vg_axi = NULL;
            }
            if(vg_device->clk_pclk) {
                //SE1000: if v2d runs in CP, don't close pclk which is also used by r2d
                clk_disable(vg_device->clk_pclk);
                clk_unprepare(vg_device->clk_pclk);

                clk_put(vg_device->clk_pclk);
                vg_device->clk_pclk = NULL;
            }
            if (!IS_ERR(vg_device->rst_ctrls)) {
                reset_control_assert(vg_device->rst_ctrls);
            }
            vg_device->clk_state = 0;
        }
    }
    return (vg_device->clk_state != 0);
}

void vg_lite_hal_delay(uint32_t milliseconds)
{
    /* Delay the requested amount. */
    msleep(milliseconds);
}

void vg_lite_hal_barrier(void)
{
    /* Memory barrier. */
    smp_mb();
}

/* flush CPU cache to DRAM */
void vg_lite_hal_cache_flush(uint32_t iova, uint32_t size)
{
    struct device* device = &vg_device->pdev->dev;
    if (vg_device->enable_smmu && cached) {
        dma_sync_single_for_device(device, (dma_addr_t)iova, (size_t)size, DMA_BIDIRECTIONAL);
    }
}
/* invalidate CPU cache */
void vg_lite_hal_cache_invalidate(uint32_t iova, uint32_t size)
{
    struct device* device = &vg_device->pdev->dev;
    if (vg_device->enable_smmu) {
        dma_sync_single_for_cpu(device, (dma_addr_t)iova, (size_t)size, DMA_BIDIRECTIONAL);
    }
}

void vg_lite_hal_initialize(void)
{
    /* TODO: Turn on the power. */

    /* TODO: Turn on the clock. */
}

void vg_lite_hal_deinitialize(void)
{
    /* TODO: Remove clock. */

    /* TODO: Remove power. */
}

static int split_node(struct heap_node * node, unsigned long size)
{
    struct heap_node * split;

    /* Allocate a new node. */
    split = kmalloc(sizeof(struct heap_node), GFP_KERNEL);
    if (split == NULL)
        return -1;

    /* Fill in the data of this node of the remaning size. */
    split->offset = node->offset + size;
    split->size = node->size - size;
    split->status = 0;

    /* Add the new node behind the current node. */
    list_add(&split->list, &node->list);

    /* Adjust the size of the current node. */
    node->size = size;
    /* No error. */
    return 0;
}

vg_lite_error_t vg_lite_hal_allocate_contiguous(unsigned long size, void ** logical, uint32_t * physical,void ** node)
{
    unsigned long aligned_size;

    /* Align the size to 64 bytes. */
    aligned_size = VG_LITE_ALIGN(size, VGLITE_MEM_ALIGNMENT);

    /* dynamically allocate memory by smmu */
    if (vg_device->enable_smmu) {
        dma_addr_t iova;
        int ret;
        uint32_t num_pages = (size + PAGE_SIZE -1 )/PAGE_SIZE;
        struct dma_memory* handle;
        handle = (struct dma_memory*) kmalloc(sizeof(struct dma_memory), GFP_KERNEL);
        if (handle == NULL) {
             return VG_LITE_OUT_OF_MEMORY;
        }
        handle->size = num_pages* PAGE_SIZE;
        if (!cached) {
            handle->kva = dma_alloc_coherent(vg_device->device, handle->size, &iova, __GFP_DMA32|GFP_KERNEL );
        } else {
            handle->kva = dma_alloc_wc(vg_device->device, handle->size, &iova, __GFP_DMA32|GFP_KERNEL);
        }

        handle->iova = iova;
        handle->uva = (void*) vm_mmap(NULL, 0L, handle->size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_NORESERVE, 0);
        if (IS_ERR(handle->uva)) {
            //kfree(handle->pages);
            kfree(handle);
            return VG_LITE_OUT_OF_MEMORY;
        }

#if LINUX_VERSION_CODE >= KERNEL_VERSION (5,8,0)
        down_write(&current->mm->mmap_lock);
#else
        down_write(&current->mm->mmap_sem);
#endif
        do
        {
            struct vm_area_struct *vma = find_vma(current->mm, (unsigned long)handle->uva);
            pgprot_t pgprot;
            ret = VG_LITE_OUT_OF_MEMORY;
            if (vma == NULL)
            {
                vm_munmap((unsigned long)handle->uva, handle->size);
                kfree(handle);
                handle = NULL;
                break;
            }
            if (cached) {
                pgprot = pgprot_writecombine(PAGE_KERNEL);
                ret = dma_mmap_wc(vg_device->device, vma, handle->kva, iova, handle->size);
            } else {
                pgprot = pgprot_noncached(PAGE_KERNEL);
                ret = dma_mmap_coherent(vg_device->device, vma, handle->kva, iova, handle->size);
            }
            if (ret < 0) {
                break;
            }

            handle->vma = vma;
        } while (0);
#if LINUX_VERSION_CODE >= KERNEL_VERSION (5,8,0)
        up_write(&current->mm->mmap_lock);
#else
        up_write(&current->mm->mmap_sem);
#endif

        if (ret) {
            if (handle)
                kfree(handle);
            pr_err("v2d: %s error=%d\n", __func__, ret);
            return ret;
        }
        *physical = iova;
        /* convert phy to user virtual */
        handle->phys = iommu_iova_to_phys(vg_device->domain, iova);

        *logical = handle->uva;
        handle->process =  task_tgid_vnr(current);
        *node = handle;
        return VG_LITE_SUCCESS;
    }
    /* Check if there is enough free memory available. */
    if (aligned_size > vg_device->heap.free) {
        return VG_LITE_OUT_OF_MEMORY;
    } else {
        /* Walk the heap backwards. */
        struct heap_node * pos;
        list_for_each_entry_reverse(pos, &vg_device->heap.list, list) {
            /* Check if the current node is free and is big enough. */
            if (pos->status == 0 && pos->size >= aligned_size) {
                /* See if we the current node is big enough to split. */
                if (pos->size - aligned_size >= VGLITE_MEM_ALIGNMENT)
                {
                    if (0 != split_node(pos, aligned_size))
                    {
                        return VG_LITE_OUT_OF_RESOURCES;
                    }
                }
                /* Mark the current node as used. */
                pos->status = HEAP_NODE_USED;
                /* Return the logical/iova address. */
                *logical = (uint8_t *) private_data->contiguous_mapped + pos->offset;
                *physical = vg_device->physical + pos->offset;
                /* Update the heap free size. */
                vg_device->heap.free -= aligned_size;
                *node = pos;
                return VG_LITE_SUCCESS;
            }
        }
    }

    /* Out of memory. */
    return VG_LITE_OUT_OF_MEMORY;
}

vg_lite_error_t vg_lite_hal_wrap_dmabuf(uint32_t dmabuf_handle, unsigned long size, uint32_t * iova, void ** memory_descriptor)
{
    struct dmabuf_memory* dm = (struct dmabuf_memory*) kmalloc(sizeof(struct dmabuf_memory), GFP_KERNEL);
    struct sg_table *sgt = NULL;
    struct dma_buf_attachment *attachment = NULL;
    struct dma_buf *dmabuf = dma_buf_get(dmabuf_handle);
    dma_addr_t addr;

    attachment = dma_buf_attach(dmabuf, vg_device->device);
    sgt = dma_buf_map_attachment(attachment, DMA_BIDIRECTIONAL);
    addr = sg_dma_address(sgt->sgl);
    *iova = (uint32_t) addr;
    dm->dmabuf = dmabuf;
    dm->attachment = attachment;
    dm->sgt = sgt;
    /* debug */
    dm->iova = *iova;
    dm->dmabuf_handle = dmabuf_handle;
    dm->size = size;
    * memory_descriptor = dm;
    return VG_LITE_SUCCESS;
}
void vg_lite_hal_unwrap_dmabuf(void * memory_descriptor)
{
    struct dmabuf_memory* dm = (struct dmabuf_memory*)memory_descriptor;

    if (dm) {
        dma_buf_unmap_attachment(dm->attachment, dm->sgt, DMA_BIDIRECTIONAL);
        dma_buf_detach(dm->dmabuf, dm->attachment);
        kfree (dm);
    }
}

static void _free_heap_node(struct heap_node * node)
{
    struct heap_node * pos;
    if (node->status != HEAP_NODE_USED) {
        if (verbose)
            pr_err("v2d: ignoring heap node in used.\n");
        return;
    }

    /* Mark node as free. */
    node->status = 0;
    /* Add node size to free count. */
    vg_device->heap.free += node->size;
    /* Check if next node is free. */
    pos = node;
    list_for_each_entry_continue(pos, &vg_device->heap.list, list) {
        if (pos->status == 0) {
            /* Merge the nodes. */
            node->size += pos->size;
            /* Delete the next node from the list. */
            list_del(&pos->list);
            kfree(pos);
        }
        break;
    }
    /* Check if the previous node is free. */
    pos = node;
    list_for_each_entry_continue_reverse(pos, &vg_device->heap.list, list) {
        if (pos->status == 0) {
            /* Merge the nodes. */
            pos->size += node->size;
            /* Delete the current node from the list. */
            list_del(&node->list);
            kfree(node);
        }
        break;
    }
    /* TODO:the memory manager still have problem,we will refine it later.*/
    /*if(node->list.next == &device->heap.list && node->list.prev == &device->heap.list)
        kfree(node);*/
}

void vg_lite_hal_free_contiguous(void * memory_handle)
{
    struct heap_node * node = (struct heap_node *)memory_handle;
    if (vg_device->enable_smmu) {
        struct dma_memory* handle = memory_handle;
        struct device* dev = vg_device->device;
        int ret;

        if (handle->uva) {
            ret = vm_munmap((unsigned long)handle->uva, handle->size);
            //vunmap(handle->uva); //release virtual mapping obtained by vmap()
        }
        if (!cached) {
            dma_free_coherent(dev, handle->size, handle->kva, handle->iova);
        } else {
            dma_free_wc(dev, handle->size, handle->kva, handle->iova);
        }
        kfree (handle);
        return;
    }
    /* Get pointer to node. */
    if(node != NULL) {
        _free_heap_node(node);
    };
}

void vg_lite_hal_free_os_heap(void)
{
    /* TODO: Remove unfree node. */
}

uint32_t vg_lite_hal_peek(uint32_t address)
{
    /* Read data from the GPU register. */
    return *(uint32_t *) (uint8_t *) (vg_device->gpu + address);
}

void vg_lite_hal_poke(uint32_t address, uint32_t data)
{
    /* Write data to the GPU register. */
    *(uint32_t *) (uint8_t *) (vg_device->gpu + address) = data;
}

vg_lite_error_t vg_lite_hal_query_mem(vg_lite_kernel_mem_t *mem)
{
    if(vg_device != NULL){
        mem->bytes = vg_device->heap.free;
        return VG_LITE_SUCCESS;
    }
    mem->bytes = 0;
    return VG_LITE_NO_CONTEXT;
}

int32_t vg_lite_hal_wait_interrupt(uint32_t timeout, uint32_t mask, uint32_t * value)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
    struct timespec64 tv;
#else
    struct timeval tv;
#endif
    unsigned long jiffies;
    unsigned long result;

    if (timeout == VG_LITE_INFINITE) {
        /* Set 1 second timeout. */
        tv.tv_sec = 1;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
        tv.tv_nsec = 0;
#else
        tv.tv_usec = 0;
#endif
    } else {
        /* Convert timeout in ms to timeval. */
        tv.tv_sec = timeout / 1000;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
        tv.tv_nsec = (timeout % 1000) * 1000000;
#else
        tv.tv_usec = (timeout % 1000) * 1000;
#endif
    }

    /* Convert timeval to jiffies. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
    jiffies = timespec64_to_jiffies(&tv);
#else
    jiffies = timeval_to_jiffies(&tv);
#endif
    /* Wait for interrupt, ignoring timeout. */
    do {
        result = wait_event_interruptible_timeout(vg_device->int_queue, vg_device->int_flags & mask, jiffies);
    } while (0);
    if (result == 0) {
        pr_debug("v2d: %s %d ms timeout!\n",__func__, timeout);
    }

    /* Report the event(s) got. */
    if (value != NULL) {
        *value = vg_device->int_flags & mask;
    }

    vg_device->int_flags = 0;
    return (result != 0);
}
/* mape user logical or physical to iova and return a mapped_handle */
void * vg_lite_hal_map(unsigned long bytes, void * logical, uint32_t physical, uint32_t * gpu)
{
    struct mapped_memory * mapped;
    int result;
    int i;

    mapped = kmalloc(sizeof(struct mapped_memory), GFP_KERNEL);
    if (mapped == NULL) {
        return NULL;
    }

    mapped->logical = logical;
    mapped->physical = physical;
    mapped->page_count = 0;
    mapped->pages = NULL;
    if (logical == NULL) {
        if (enable_smmu) {
            /* physical don't use pages */
            dma_addr_t dmaHandle;
            unsigned long pfn = physical >> PAGE_SHIFT;
            if (!pfn_valid(pfn)) {
                pr_err("v2d: %s error pfn %lx\n", __func__, pfn);
                goto ON_ERROR;
            }
            dmaHandle = dma_map_page(vg_device->device, pfn_to_page(pfn), 0, bytes, DMA_BIDIRECTIONAL);
            if (dma_mapping_error(vg_device->device, dmaHandle) != 0) {
                pr_err("v2d: %s error iova=%llX\n", __func__, dmaHandle);
                goto ON_ERROR;
            }
            *gpu = dmaHandle;
            mapped->physical = (uint32_t)dmaHandle;
        }else {
            *gpu = physical;
        }
    } else {
        unsigned long start, end;
        start = (unsigned long) logical >> PAGE_SHIFT;
        end = ((unsigned long) logical + bytes + PAGE_SIZE - 1) >> PAGE_SHIFT;
        mapped->page_count = end - start;
        mapped->pages = kmalloc( mapped->page_count * sizeof(struct page *), GFP_KERNEL);
        if (mapped->pages == NULL) {
            pr_err("v2d: %s out of memory\n", __func__);
            goto ON_ERROR;
        }
        down_read(&current_mm_mmap_sem);
#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 6, 0)
        result = pin_user_pages((unsigned long) logical & PAGE_MASK, mapped->page_count, FOLL_GET|FOLL_WRITE, mapped->pages, NULL);
#elif  LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,1)
        result = get_user_pages((unsigned long) logical & PAGE_MASK, mapped->page_count, FOLL_GET|FOLL_WRITE, mapped->pages, NULL);
#else
        get_user_pages(current, current->mm, (unsigned long) logical & PAGE_MASK, mapped->page_count, 1, 0, mapped->pages, NULL);
#endif
        up_read(&current_mm_mmap_sem);
        if (result < mapped->page_count) {
            pr_err("v2d: %s get_user_pages error=0x%x\n", __func__, result);
            for (i=0; i<result; i++) {
                if (mapped->pages[i]) {
                    put_page(mapped->pages[i]);
                }
            }
            mapped->page_count = 0;
            goto ON_ERROR;
        }
        result = sg_alloc_table_from_pages(&mapped->sgt, mapped->pages, mapped->page_count,
                (unsigned long)logical & ~PAGE_MASK, mapped->page_count*PAGE_SIZE, GFP_KERNEL);
        if (unlikely(result < 0))
        {
            pr_err("v2d: %s: sg_alloc_table_from_pages failed, %d\n", __func__, result);
            goto ON_ERROR;
        }
        result = dma_map_sg(vg_device->device, mapped->sgt.sgl, mapped->sgt.nents, DMA_FROM_DEVICE);

        *gpu = sg_dma_address(mapped->sgt.sgl);
        dma_sync_sg_for_cpu(vg_device->device, mapped->sgt.sgl, mapped->sgt.nents, DMA_FROM_DEVICE);
    }

    return mapped;
ON_ERROR:
    if (mapped) {
        if (mapped->pages) {
            for (i=0; i <mapped->page_count; i++) {
                if (mapped->pages[i]) {
                    put_page(mapped->pages[i]);
                }
            }
            kfree(mapped->pages);
        }
        kfree(mapped);
    }
    return NULL;
}

void vg_lite_hal_unmap(void * handle)
{

    struct mapped_memory * mapped = handle;
    int i;

    if (mapped->page_count && mapped->pages) {
        for (i = 0; i < mapped->page_count; i++) {
            if (mapped->pages[i] != NULL)
#if  LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
                unpin_user_page(mapped->pages[i]);
#elif  LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,1)
                put_page(mapped->pages[i]);
#else
                page_cache_release(mapped->pages[i]);
#endif
        }
        dma_unmap_sg(vg_device->device, mapped->sgt.sgl, mapped->sgt.nents, DMA_FROM_DEVICE);
        sg_free_table(&mapped->sgt);
        kfree(mapped->pages);
    }
    kfree(mapped);
}

static int drv_open(struct inode * inode, struct file * file)
{
    struct client_data * data;

    data = kmalloc(sizeof(struct client_data), GFP_KERNEL);
    if (data == NULL)
        return -1;

    data->vg_obj = vg_device;
    data->contiguous_mapped = NULL;

    file->private_data = data;

    return 0;
}

static int drv_release(struct inode * inode, struct file * file)
{
    struct client_data * data = (struct client_data *) file->private_data;

    if (data != NULL) {
        if (data->contiguous_mapped != NULL) {
            data->contiguous_mapped = NULL;
        }

        kfree(data);
        file->private_data = NULL;
    }

    return 0;
}

static long drv_ioctl(struct file * file, unsigned int ioctl_code, unsigned long arg)
{
    struct ioctl_data arguments;
    void * data;

    private_data = (struct client_data *) file->private_data;
    if (private_data == NULL)
    {
        return -1;
    }

    if (ioctl_code != VG_LITE_IOCTL) {
        return -1;
    }

    if ((void *) arg == NULL)
    {
        return -1;
    }

    if (copy_from_user(&arguments, (void *) arg, sizeof(arguments)) != 0) {
        return -1;
    }

    data = kmalloc(arguments.bytes, GFP_KERNEL);
    if (data == NULL)
        return -1;

    if (copy_from_user(data, arguments.buffer, arguments.bytes) != 0)
        goto error;

    arguments.error = kernel_dispatch(arguments.command, data);

    if (copy_to_user(arguments.buffer, data, arguments.bytes) != 0)
        goto error;

    kfree(data);

    if (copy_to_user((void *) arg, &arguments, sizeof(arguments)) != 0)
        return -1;

    return 0;

error:
    kfree(data);
    return -1;
}

static ssize_t drv_read(struct file * file, char * buffer, size_t length, loff_t * offset)
{
    struct client_data * private = (struct client_data *) file->private_data;

    if (length != 4) {
        return 0;
    }

    if (copy_to_user((void __user *) buffer, (const void *) &private->vg_obj->size, sizeof(private->vg_obj->size)) != 0)
    {
        return 0;
    }

    return 4;
}

static int drv_mmap(struct file * file, struct vm_area_struct * vm)
{
    unsigned long size;
    struct client_data * private = (struct client_data *) file->private_data;

    if (vg_device->enable_smmu) {
        return 0;
    }
    if (!cached)
        vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
    else
        vm->vm_page_prot = pgprot_writecombine(vm->vm_page_prot);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
    vm->vm_flags |= VM_DONTDUMP;
#else
    vm->vm_flags |= (VM_DONTEXPAND | VM_DONTDUMP);
#endif
    vm->vm_pgoff = 0;

    size = vm->vm_end - vm->vm_start;
    if (size > private->vg_obj->size)
        size = private->vg_obj->size;

    if (remap_pfn_range(vm, vm->vm_start, private->vg_obj->physical >> PAGE_SHIFT, size, vm->vm_page_prot) < 0) {
        pr_err("v2d: remap_pfn_range failed\n");
        return -1;
    }

    private->vm = vm;
    private->contiguous_mapped = (void *) vm->vm_start;

    if (verbose)
        pr_info("v2d: mapped %scached contiguous memory to %llx\n", cached ? "" : "non-", (unsigned long long)private->contiguous_mapped);

    return 0;
}

static struct file_operations file_operations =
{
    .owner          = THIS_MODULE,
    .open           = drv_open,
    .release        = drv_release,
    .read           = drv_read,
    .unlocked_ioctl = drv_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl   = drv_ioctl,
#endif
    .mmap           = drv_mmap,
};

static irqreturn_t irq_hander(int irq, void * context)
{
    struct vg_lite_device * device = context;

    /* Read interrupt status. */
    uint32_t flags = *(uint32_t *) (uint8_t *) (device->gpu + VG_LITE_INTR_STATUS);
    if (flags) {
        /* Combine with current interrupt flags. */
        device->int_flags |= flags;

        /* Wake up any waiters. */
        wake_up_interruptible(&device->int_queue);

        /* We handled the IRQ. */
        return IRQ_HANDLED;
    }

    /* Not our IRQ. */
    return IRQ_NONE;
}

static void vg_lite_exit(void)
{
    struct heap_node * pos;
    struct heap_node * n;

    /* Check for valid vg_device. */
    if (vg_device != NULL) {
        if (vg_device->gpu != NULL) {
            /* Unmap the GPU registers. */
            iounmap(vg_device->gpu);
            vg_device->gpu = NULL;
        }
        if (vg_device->enable_smmu) {
            struct device* dev = vg_device->device;
            if (!cached) {
                dma_free_coherent(dev, vg_device->size, vg_device->virtual, vg_device->physical);
            } else {
                dma_free_wc(dev, vg_device->size, vg_device->virtual, vg_device->physical);
            }
        } else {

            if (vg_device->pages != NULL) {
                /* Free the contiguous memory. */
                __free_pages(vg_device->pages, vg_device->order);
            }

            /* Process each node. */
            list_for_each_entry_safe(pos, n, &vg_device->heap.list, list) {
                /* Remove it from the linked list. */
                list_del(&pos->list);
                /* Free up the memory. */
                kfree(pos);
            }
        }
        if (vg_device->irq_enabled) {
            /* Free the IRQ. */
            free_irq(irqLine/*GPU_IRQ*/, vg_device);
        }
        exit_sysfs(vg_device->device);
        if (vg_device->device_class) {
            /* Destroy the vg_device. */
            device_destroy(vg_device->device_class, MKDEV(vg_device->major, 0));
            class_destroy(vg_device->device_class);
        }
        if (vg_device->major >= 0) {
            unregister_chrdev(vg_device->major, DEVICE_NAME);
        }

        /* Free up the vg_device structure. */
        kfree(vg_device);
    }
}

static int vg_lite_init_kernel(struct platform_device *pdev)
{
    struct heap_node * node;

    /* Create vg_device structure. */
    vg_device = kmalloc(sizeof(struct vg_lite_device), GFP_KERNEL);
    if (vg_device == NULL) {
        pr_err("v2d: kmalloc failed\n");
        return -1;
    }
    /* Zero out the enture structure. */
    memset(vg_device, 0, sizeof(struct vg_lite_device));

    vg_device->pdev = pdev;
    pr_debug("v2d: %s\n", __func__);
    /* Map the GPU registers. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
    vg_device->gpu = ioremap(registerMemBase/*GPU_REG_START*/, registerMemSize/*GPU_REG_SIZE*/);
#else
    vg_device->gpu = ioremap_nocache(registerMemBase/*GPU_REG_START*/, registerMemSize/*GPU_REG_SIZE*/);
#endif
    if (vg_device->gpu == NULL) {
        pr_err("v2d: ioremap failed\n");
        return -1;
    }
    vg_device->device = &pdev->dev;
    //SE1000: enable clock and smmu
    enable_clock( &pdev->dev, 1);
    /* Initialize the wait queue. */
    init_waitqueue_head(&vg_device->int_queue);

    /* Install IRQ. */
    if (request_irq(irqLine/*GPU_IRQ*/, irq_hander, 0, "vg_lite_irq", vg_device)) {
        pr_err("v2d: request_irq failed\n");
        vg_lite_exit();
        return -1;
    }
    vg_device->irq_enabled = 1;

    /* Register vg_device. */
    vg_device->major = register_chrdev(0, DEVICE_NAME, &file_operations);
    if (vg_device->major < 0) {
        pr_err("v2d: register_chrdev failed\n");
        vg_lite_exit();
        return -1;
    }

    /* Create the graphics class. */
    vg_device->device_class = class_create(THIS_MODULE, "vg_lite_class");
    if (vg_device->device_class == NULL) {
        pr_err("v2d: class_create failed\n");
        vg_lite_exit();
        return -1;
    }

    /* Create the vg_device. */
    if ( device_create(vg_device->device_class, NULL, MKDEV(vg_device->major, 0), NULL, DEVICE_NAME) == NULL) {
        pr_err("v2d: device_create failed\n");
        device_destroy(vg_device->device_class, MKDEV(vg_device->major, 0));
        vg_device->device_class = NULL;
        vg_lite_exit();
        return -1;
    }
    init_sysfs(vg_device->device);
    pr_info("v2d: created /dev/vg_lite device\n");
    /* Success. */
    if (enable_smmu) {
        struct device* dev = vg_device->device;

        dev->coherent_dma_mask = DMA_BIT_MASK(31);
        dev->dma_mask = &dev->coherent_dma_mask;
        vg_device->domain = iommu_get_domain_for_dev(dev);
        vg_device->enable_smmu = enable_smmu;
        pr_debug("v2d: %s enable smmu\n", __func__);
        return 0;
    }
    else {
        //none smmu, check reserved memory contiguousBase
        if (contiguousBase == 0){
            /* Allocate the contiguous memory from heap. */
            for (vg_device->order = get_order(heap_size); vg_device->order > 0; vg_device->order--) {
                /* Allocate with the current amount. */
                vg_device->pages = alloc_pages(GFP_KERNEL, vg_device->order);
                if (vg_device->pages != NULL)
                    break;
            }
            /* Check if we allocated any contiguous memory or not. */
            if (vg_device->pages == NULL) {
                pr_err("v2d: alloc_pages failed\n");
                vg_lite_exit();
                return -1;
            }
            /* Save contiguous memory. */
            vg_device->virtual = page_address(vg_device->pages);
            vg_device->physical = virt_to_phys(vg_device->virtual);
            vg_device->size = 1 << (vg_device->order + PAGE_SHIFT);
            if (verbose)
                pr_info("v2d: contiguous memory 0x%x,  0x%x bytes\n", vg_device->physical, vg_device->size);
        } else {
            //from reserved memory
            vg_device->virtual = 0;
            vg_device->physical = contiguousBase;
            vg_device->size = contiguousSize;

            if (verbose) {
                uint32_t size = (vg_device->size >> 20) ? (vg_device->size >> 20) : (vg_device->size >> 10);
                char c = (vg_device->size >> 20) ? 'M' : 'k';

                pr_info("v2d: allocated a %u%cB heap at 0x%08x\n", size, c, vg_device->physical);
            }
        }
        /* Create the heap. */
        INIT_LIST_HEAD(&vg_device->heap.list);
        vg_device->heap.free = vg_device->size;
        node = kmalloc(sizeof(struct heap_node), GFP_KERNEL);
        if (node == NULL) {
            pr_err("v2d: kmalloc failed\n");
            vg_lite_exit();
            return -1;
        }
        node->offset = 0;
        node->size = vg_device->size;
        node->status = 0;
        list_add(&node->list, &vg_device->heap.list);
    }
    return 0;
}

static int gpu_probe(struct platform_device *pdev)
{
    struct resource* res;
    pr_debug("v2d: %s\n", __func__);
    res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "V2D");
    if (res)
    {
        irqLine =  res->start;
    }
    res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "contiguous_mem");

    if (res)
    {
         contiguousBase = res->start;
         contiguousSize = res->end - res->start + 1;
         pr_debug("v2d: contiguousBase=0x%x contiguousSize=0x%x\n", contiguousBase, contiguousSize);
    }
    res = platform_get_resource( pdev, IORESOURCE_MEM, 0 );
    if (res) {
        registerMemBase = res->start;
        registerMemSize = res->end - res->start + 1;
    }

    if(vg_lite_init_kernel(pdev)) {
        return -ENODEV;
    }

    return 0;
}

static int gpu_remove(struct platform_device *pdev)
{
    if (!vg_device) {
        return 0;
    }

    vg_lite_exit();
    /* disable clock. */
    enable_clock(&pdev->dev, 0);
    return 0;
}

static int gpu_suspend(struct platform_device *pdev, pm_message_t state)
{
    pr_info("v2d: entering %s\n", __func__);
    /* wait gpu idle */
    while(VG_LITE_KERNEL_IS_GPU_IDLE() != 1) {
        vg_lite_hal_delay(2);
    }
    /* shutdown gpu */
    kernel_dispatch(VG_LITE_CLOSE, NULL);

    enable_clock(&pdev->dev, 0);

    return 0;
}

static int gpu_resume(struct platform_device *pdev)
{
    /* open power and clock */
    enable_clock(&pdev->dev, 1);

    /* open gpu interrupt and recovery gpu register */
    kernel_dispatch(VG_LITE_RESET, NULL);

    pr_info("v2d: %s success!\n", __func__);

    return 0;
}

# if defined(CONFIG_PM) && LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 30)
#  ifdef CONFIG_PM_SLEEP
static int gpu_system_suspend(struct device *dev)
{
    pm_message_t state = { 0 };

    return gpu_suspend(to_platform_device(dev), state);
}

static int gpu_system_resume(struct device *dev)
{
    return gpu_resume(to_platform_device(dev));
}

static const struct dev_pm_ops gpu_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(gpu_system_suspend, gpu_system_resume)
};
#  endif
# endif
static struct platform_driver gpu_driver = {
    .probe      = gpu_probe,
    .remove     = gpu_remove,
    .suspend    = gpu_suspend,
    .resume     = gpu_resume,
    .driver     = {
        .owner = THIS_MODULE,
        .name  = DEVICE_NAME,
#if defined(CONFIG_PM) && LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 30)
        .pm = &gpu_pm_ops,
#endif
        .of_match_table = compatibles,
    }
};

static int __init gpu_init(void)
{
    int err;

    err = platform_driver_register(&gpu_driver);
    if (err) {
        return -ENODEV;
    }

    return 0;
}

static void __exit gpu_exit(void)
{
    platform_driver_unregister(&gpu_driver);
}

module_init(gpu_init);
module_exit(gpu_exit);
