/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2012 - 2022 Vivante Corporation
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
*    Copyright (C) 2012 - 2022 Vivante Corporation
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/pagemap.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/pm_runtime.h>
#include <asm/io.h>
#include <linux/kthread.h>

#include "nano2D_types.h"
#include "nano2D_dispatch.h"
#include "nano2D_kernel.h"
#include "nano2D_kernel_platform.h"
#include "nano2D_kernel_hardware.h"
#include "nano2D_kernel_event.h"
#include "nano2D_kernel_os.h"
#include "nano2D_kernel_db.h"
#include "nano2D_kernel_iommu.h"
#include "nano2D_kernel_driver.h"

#define SUSPEND_DELAY_MS 5000

#if USE_LINUX_PCIE
#include <linux/cdev.h>
#endif

MODULE_DESCRIPTION("Vivante Graphics Driver");
MODULE_LICENSE("GPL");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0)
MODULE_IMPORT_NS(DMA_BUF);
#endif

static int device_index[NANO2D_CORE_MAX] = {[0 ... NANO2D_CORE_MAX - 1] = -1};
module_param_array(device_index, int, NULL, 0644);
MODULE_PARM_DESC(device_index, "For multi devices, the device id of each core.");

static uint registerAPB = 0x800;
module_param(registerAPB, uint, 0644);
MODULE_PARM_DESC(registerAPB, "The offset of APB register to the register base address.");

static ulong register_base2D = 0x0;
module_param(register_base2D, ulong, 0644);
MODULE_PARM_DESC(register_base2D, "Address of AHB bus of the 2D register.");

static uint register_size2D = 0x2000;
module_param(register_size2D, uint, 0644);
MODULE_PARM_DESC(register_size2D, "Size of AHB bus address range of 2D register.");

static int irq2D = -1;
module_param(irq2D, int, 0644);
MODULE_PARM_DESC(irq2D, "Irq number of 2D.");

static int irqs[NANO2D_CORE_MAX] = {[0 ... NANO2D_CORE_MAX - 1] = -1};
module_param_array(irqs, int, NULL, 0644);
MODULE_PARM_DESC(irqs, "Array of IRQ numbers of multi-2D");

static ulong register_bases[NANO2D_CORE_MAX];
module_param_array(register_bases, ulong, NULL, 0644);
MODULE_PARM_DESC(register_bases, "Array of bases of bus address of register of multi-2D");

static ulong register_sizes[NANO2D_CORE_MAX];
module_param_array(register_sizes, ulong, NULL, 0644);
MODULE_PARM_DESC(register_sizes, "Array of sizes of bus address of register of multi-2D");

static int bars[NANO2D_CORE_MAX] = {[0 ... NANO2D_CORE_MAX - 1] = -1};
module_param_array(bars, int, NULL, 0644);
MODULE_PARM_DESC(bars, "Array of PCI bar numbers of multi-2D");

static int bar2D = -1;
module_param(bar2D, int, 0644);
MODULE_PARM_DESC(bar2D, "PCIE bar index of 2D.");

static ulong contiguous_base = 0x0;
module_param(contiguous_base, ulong, 0644);
MODULE_PARM_DESC(contiguous_base, "Contiguous base of reserved memory of 2D.");

static ulong contiguous_size = 0x0;
module_param(contiguous_size, ulong, 0644);
MODULE_PARM_DESC(contiguous_size, "Contiguous size of reserved memory of 2D.");

static ulong command_contiguous_base = 0x0;
module_param(command_contiguous_base, ulong, 0644);
MODULE_PARM_DESC(command_contiguous_base, "Command contiguous base of reserved memory of 2D.");

static ulong command_contiguous_size = 0x0;
module_param(command_contiguous_size, ulong, 0644);
MODULE_PARM_DESC(command_contiguous_size, "Command contiguous size of reserved memory of 2D.");

static uint polling = 0;
module_param(polling, uint, 0644);
MODULE_PARM_DESC(polling, "Default 0 means disable polling, 1 means polling register status when the interrupt is not available.");

static bool iommu = 0;
module_param(iommu, bool, 0644);
MODULE_PARM_DESC(iommu, "Default 0 disable iommu/smmu, 1 means system has iommu/smmu and enabled.");

#define HEAP_NODE_USED 0xABBAF00D

static uint major = 199;
static struct class* device_class = NULL;

static n2d_linux_platform_t *platform = N2D_NULL;
n2d_linux_module_parameters_t global_param = {0};
static struct n2d_gl_device *global_device = NULL;
unsigned int dump_core_mask = 0x00000001;

struct client_data
{
    n2d_device_t *device;
    int refcount;
};

static const char *isr_names[NANO2D_DEVICE_MAX][NANO2D_DEV_CORE_COUNT] =
{
    {
        "galcore:dev0_2d0",
    },
};

/********************************************************************/
#ifdef CONFIG_DEBUG_FS
# if NANO2D_MMU_ENABLE
static int mmuinfo_show(struct seq_file *m, void *data)
{
    n2d_mmu_t *mmu = &global_device->kernel->sub_dev[0]->hardware[0]->mmu;
    n2d_uint32_t i, line, left;
    n2d_uint32_t ext_mtlb, config;
    n2d_uint32_t *buf = (n2d_uint32_t *)mmu->config->mmu_init_buffer_logical;
    n2d_uint32_t reserve_bytes = mmu->config->node->size;
    n2d_uint64_t physical;
    n2d_uint64_t descriptor = 0;

    physical = mmu->config->mtlb_physical;
    config = (n2d_uint32_t)(physical & 0xFFFFFFFF);
    ext_mtlb = (n2d_uint32_t)(physical >> 32);
    config |= gcmSETFIELDVALUE(0, GCREG_MMU_CONFIGURATION, MODE, MODE4_K);

    descriptor = ((n2d_uint64_t)ext_mtlb << 32) | (n2d_uint64_t)config;

    n2d_kernel_os_print("MMU descriptor: 0x%016X\n", descriptor);

    line = reserve_bytes / 32;
    left = reserve_bytes % 32;
    gcmkASSERT(left == 0);

    n2d_kernel_os_print("MMU init cmd buffer:\n");
    for (i = 0; i < line; i++) {
        n2d_kernel_os_print("  %08X %08X %08X %08X %08X %08X %08X %08X\n",
                            buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
        buf += 8;
    }

    /* Print MMU page table. */
    n2d_mmu_dump_page_table(mmu);

    return 0;
}
# endif
static int dump_trigger_show(struct seq_file *m, void *data)
{
    int i = 0, j = 0;

    for (i = 0; i < global_device->kernel->dev_num; i++) {
        for (j = 0; j < global_device->kernel->dev_core_num; j++) {
            if (global_device->kernel->sub_dev[i]->hardware[j])
                n2d_kernel_hardware_dump_gpu_state(global_device->kernel->sub_dev[i]->hardware[j]);
        }
    }

    return 0;
}

static int dump_trigger_store(const char __user *buf, size_t count, void *data)
{
    unsigned int mask = 0;

    sscanf(buf, "%d", &mask);

    if (mask) {
        dump_core_mask = mask;
    } else {
        n2d_kernel_os_print("Each bit represents a core!\n");
    }

    return count;
}

static n2d_debug_info_t info_list[] = {
# if NANO2D_MMU_ENABLE
    { "mmu", mmuinfo_show },
# endif
    { "dump_trigger", dump_trigger_show, dump_trigger_store },
};
#endif

static n2d_error_t create_debug_fs(struct n2d_gl_device *gl_device)
{
    n2d_error_t error = N2D_SUCCESS;

#ifdef CONFIG_DEBUG_FS
    n2d_debugfs_dir_t *dir = &gl_device->debuugfs_dir;

    ONERROR(n2d_debugfs_dir_init(dir, N2D_NULL, "n2d"));
    ONERROR(n2d_debugfs_dir_crestefiles(dir, info_list, N2D_COUNTOF(info_list), gl_device));
#endif

on_error:
    return error;
}

static n2d_error_t destroy_debug_fs(struct n2d_gl_device *gl_device)
{
#ifdef CONFIG_DEBUG_FS
    n2d_debugfs_dir_t *dir = &gl_device->debuugfs_dir;

    if (dir->root) {
        n2d_debugfs_dir_removefiles(dir, info_list, N2D_COUNTOF(info_list));

        n2d_debugfs_dir_deinit(dir);
    }
#endif
    return N2D_SUCCESS;
}

/********************************************************************/
static irqreturn_t isr_routine(int irq, void *ctxt)
{
    n2d_error_t error = N2D_SUCCESS;
    n2d_device_t *device = global_device;
    n2d_device_id_t dev_id = 0;
    n2d_core_id_t core_id = 0;
    /*
     * VIV: match the input of request_irq function.
     */
    n2d_core_id_t core = (n2d_core_id_t)gcmPTR2INT(ctxt) - 1;

    /* Find this core's sub device id and core id. */
    dev_id = core / device->kernel->dev_core_num;
    core_id = core % device->kernel->dev_core_num;

    /* Call kernel interrupt notification. */
    error = n2d_kernel_hardware_interrupt(device->kernel->sub_dev[dev_id]->hardware[core_id]);

    if (N2D_IS_SUCCESS(error)) {
        up(&device->semas[core]);
        return IRQ_HANDLED;
    }

    return IRQ_NONE;
}

static int polling_isr_state_routine(void *ctxt)
{
    n2d_error_t error = N2D_SUCCESS;
    n2d_device_t *device = global_device;
    n2d_core_id_t core = (n2d_core_id_t)gcmPTR2INT(ctxt);
    n2d_hardware_t *hardware = N2D_NULL;
    n2d_uint32_t data = 0;
    n2d_device_id_t dev_id = 0;
    n2d_core_id_t core_id = 0;

    /* Find this core's sub device id and core id. */
    dev_id = core / device->kernel->dev_core_num;
    core_id = core % device->kernel->dev_core_num;
    hardware = device->kernel->sub_dev[dev_id]->hardware[core_id];

    error = n2d_kernel_hardware_interrupt(hardware);
    /* Read irq status register. */

    for (;;) {
        if (unlikely(global_device->kill_thread)) {
            /* The daemon exits. */
            while (!kthread_should_stop()) {
                n2d_kernel_os_delay(global_device->os, 1);
            }
            return 0;
        }

        data = n2d_kernel_os_peek_with_core(hardware->os, hardware->core, 0x00010);

        if (data) {
            /* Inform event center of the interrupt. */
            error = n2d_kernel_event_interrupt(hardware->event_center, data);
        }

        if (N2D_IS_SUCCESS(error))
            up(&device->semas[core]);
    }

    /* Return the error. */
    return error;
}

static int thread_routine(void *ctxt)
{
    n2d_device_t *device = global_device;
    n2d_core_id_t core = (n2d_core_id_t)gcmPTR2INT(ctxt);
    n2d_device_id_t dev_id = 0;
    n2d_core_id_t core_id = 0;

    /* Find this core's sub device id and core id. */
    dev_id = core / device->kernel->dev_core_num;
    core_id = core % device->kernel->dev_core_num;

    for (;;) {
        int down;

        down = down_interruptible(&device->semas[core]);
        if (down && down != -EINTR)
            return down;

        if (unlikely(global_device->kill_thread)) {
            /* The daemon exits. */
            while (!kthread_should_stop()) {
                n2d_kernel_os_delay(global_device->os, 1);
            }
            return 0;
        }

        n2d_kernel_hardware_notify(device->kernel->sub_dev[dev_id]->hardware[core_id]);
    }
}

static n2d_error_t
_start_thread(n2d_device_t *device, n2d_device_id_t dev_id, n2d_core_id_t core_id)
{
    n2d_error_t error = N2D_SUCCESS;
    struct task_struct * task;
    n2d_uint64_t global_core_id = 0;

    /* Find this core's global core id. */
    global_core_id = dev_id * NANO2D_DEVICE_MAX + core_id;

    if (device->kernel->sub_dev[dev_id]->hardware[core_id] != N2D_NULL) {
        /* Start the kernel thread. */
        task = kthread_run(thread_routine, (void *)global_core_id,
                "galcore_deamon/%lld", global_core_id);

        if (IS_ERR(task))
            ONERROR(N2D_GENERIC_IO);

        device->thread_task[global_core_id] = task;
        /*VIV: Set highest non RT priority, same as work queues.*/
        set_user_nice(task, -20);
    }
on_error:
    return error;
}

static int _stop_thread(n2d_device_t *device, n2d_device_id_t dev_id, n2d_core_id_t core_id)
{
    n2d_uint32_t global_core_id = dev_id * NANO2D_DEVICE_MAX + core_id;

    device->kill_thread = N2D_TRUE;
    up(&device->semas[global_core_id]);
    kthread_stop(device->thread_task[global_core_id]);
    device->thread_task[global_core_id] = N2D_NULL;

    return N2D_SUCCESS;
}

static n2d_error_t
_setup_isr(n2d_device_t *device, n2d_device_id_t dev_id, n2d_core_id_t core_id)
{
    n2d_int32_t ret = 0;
    n2d_error_t error = N2D_SUCCESS;
    irq_handler_t handler;
    n2d_uint32_t global_core_id = dev_id * NANO2D_DEVICE_MAX + core_id;

    handler = isr_routine;

    /*
     * VIV: Avoid request_irq parameter dev_id is null. here add 1 to core id.
     */
    ret = request_irq(
        device->irq_line[global_core_id], handler, IRQF_SHARED,
        isr_names[dev_id][core_id], (void *)(uintptr_t)(global_core_id + 1)
        );

    device->irq_enabled[global_core_id] = 1;

    if (ret != 0)
        ONERROR(N2D_GENERIC_IO);

on_error:
    return error;
}

static int _release_isr(n2d_device_t *device, n2d_device_id_t dev_id, n2d_core_id_t core_id)
{
    n2d_uint32_t global_core_id = dev_id * NANO2D_DEVICE_MAX + core_id;
    /*
     * VIV: Avoid free_irq parameter dev_id is null. here add 1 to core id.
     */
    free_irq(device->irq_line[global_core_id], (void *)(uintptr_t)(global_core_id + 1));

    return N2D_SUCCESS;
}

static n2d_error_t
_start_thread_polling_irq_status(n2d_device_t *device,
    n2d_device_id_t dev_id, n2d_core_id_t core_id)
{
    n2d_error_t error = N2D_SUCCESS;
    struct task_struct * task;
    /* This core's global core id. */
    n2d_uint64_t global_core_id = dev_id * NANO2D_DEVICE_MAX + core_id;

    if (device->kernel->sub_dev[dev_id]->hardware[core_id] != N2D_NULL) {
        /* Start the kernel thread. */
        task = kthread_run(polling_isr_state_routine, (void *)global_core_id,
                "polling_irq_status_deamon/%lld", global_core_id);

        if (IS_ERR(task))
            ONERROR(N2D_GENERIC_IO);

        device->thread_task_polling_irq_state[global_core_id] = task;
        /*VIV: Set highest non RT priority, same as work queues.*/
        set_user_nice(task, -20);
    }
on_error:
    return error;
}

static int _stop_thread_polling_irq_status(
    n2d_device_t *device, n2d_device_id_t dev_id, n2d_core_id_t core_id)
{
    n2d_uint32_t global_core_id = dev_id * NANO2D_DEVICE_MAX + core_id;

    device->kill_thread = N2D_TRUE;
    kthread_stop(device->thread_task_polling_irq_state[global_core_id]);
    device->thread_task_polling_irq_state[global_core_id] = N2D_NULL;

    return N2D_SUCCESS;
}

static int start_device(n2d_device_t *device)
{
    n2d_error_t error = N2D_SUCCESS;
    n2d_device_id_t dev_id = 0;
    n2d_core_id_t core_id = 0;
    n2d_uint32_t global_core_id = 0;

    for (dev_id = 0; dev_id < NANO2D_DEVICE_MAX; dev_id++) {
        for (core_id = 0; core_id < NANO2D_DEV_CORE_COUNT; core_id++) {
            if (device->irq_line[global_core_id] == -1)
                continue;

            gcmkASSERT(global_core_id == (dev_id * NANO2D_DEVICE_MAX + core_id));

            sema_init(&device->semas[global_core_id], 0);
            ONERROR(_start_thread(device, dev_id, core_id));

            if (polling)
                ONERROR(_start_thread_polling_irq_status(device, dev_id, core_id));
            else
                ONERROR(_setup_isr(device, dev_id, core_id));

            global_core_id++;
        }
    }
    return 0;
on_error:
    return -1;
}

static int stop_device(n2d_device_t *device)
{
    n2d_device_id_t dev_id = 0;
    n2d_core_id_t core_id = 0;
    n2d_uint32_t global_core_id = 0;

    for (dev_id = 0; dev_id < NANO2D_DEVICE_MAX; dev_id++) {
        for (core_id = 0; core_id < NANO2D_DEV_CORE_COUNT; core_id++) {
            if (device->irq_line[global_core_id] == -1)
                continue;

            gcmkASSERT(global_core_id == (dev_id * NANO2D_DEVICE_MAX + core_id));

            if (polling)
                _stop_thread_polling_irq_status(device, dev_id, core_id);
            else
                _release_isr(device, dev_id, core_id);

            _stop_thread(device, dev_id, core_id);

            global_core_id++;
        }
    }

    return N2D_SUCCESS;
}

static int init_param(n2d_linux_module_parameters_t *param)
{
    int i = 0;

    if (!param)
        return N2D_INVALID_ARGUMENT;

    for (i = 0; i < NANO2D_CORE_MAX; i++) {
        param->irq_line[i] = -1;
        param->bars[i] = -1;
        param->device_index[i] = -1;
        param->register_bases[i] = 0;
    }

    param->contiguous_size = 0;
    param->contiguous_base = 0;
    param->command_contiguous_size = 0;
    param->command_contiguous_base = 0;

    return N2D_SUCCESS;
}

static int sync_input_param(n2d_linux_module_parameters_t *param)
{
    int i = 0;

    param->iommu = iommu;

    if (irq2D != -1) {
        param->irq_line[0] = irq2D;
        param->bars[0] = bar2D;
        param->register_bases[0] = register_base2D;
        param->register_sizes[0] = register_size2D;
    } else {
        for (i = 0; i < NANO2D_CORE_MAX; i++) {
            if (irqs[i] != -1) {
                param->irq_line[i] = irqs[i];
                param->bars[i] = bars[i];
                param->device_index[i] = device_index[i];
                param->register_bases[i] = register_bases[i];
                param->register_sizes[i] = register_sizes[i];
            }
        }
    }

    param->registerAPB = registerAPB;
    param->contiguous_base = contiguous_base;
    param->contiguous_size = contiguous_size;
    param->command_contiguous_base = command_contiguous_base;
    param->command_contiguous_size = command_contiguous_size;

    return N2D_SUCCESS;
}

static int sync_param(n2d_linux_module_parameters_t *param)
{
    int i = 0;

    if (global_device == NULL || param == NULL)
        return 0;

    global_device->iommu = param->iommu;

    for (i = 0; i < NANO2D_CORE_MAX; i++) {
        global_device->register_bases[i] = param->register_bases[i];
        global_device->register_bases_mapped[i] = param->register_bases_mapped[i];
        global_device->register_sizes[i] = param->register_sizes[i];
        global_device->bars[i] = param->bars[i];
        global_device->device_index[i] = param->device_index[i];
        global_device->irq_line[i] = param->irq_line[i];
    }

    global_device->contiguous_size = param->contiguous_size;
    global_device->contiguous_base = param->contiguous_base;
    global_device->contiguous_requested = param->contiguous_requested;

    global_device->command_contiguous_size = param->command_contiguous_size;
    global_device->command_contiguous_base = param->command_contiguous_base;
    global_device->command_contiguous_requested = param->command_contiguous_requested;

    global_device->base_address = param->base_address;
    global_device->registerAPB = param->registerAPB;

    return N2D_SUCCESS;
}

static void show_param(n2d_linux_module_parameters_t *param, n2d_device_t *device)
{
    int i = 0;
    n2d_device_id_t dev_id = 0;
    n2d_core_id_t core_id = 0;
    n2d_uint32_t core_num = device->core_num;

    /* Find this core's sub device id and core id. */
    dev_id = core_num / device->kernel->dev_core_num;
    core_id = core_num % device->kernel->dev_core_num;

    printk("Insmod parameters:\n");
    for (i = 0; i < core_num; i++) {
        dev_id = i / device->kernel->dev_core_num;
        core_id = i % device->kernel->dev_core_num;

        printk("device[%d] core[%d]\n", dev_id, core_id);
        printk("  irq line:%d\n", param->irq_line[i]);
        printk("  register base:0x%llx\n", (n2d_uint64_t)param->register_bases[i]);
        printk("  register size:0x%llx\n", (n2d_uint64_t)param->register_sizes[i]);
    }
    printk("\n");
    if (param->iommu)
        printk("  iommu enabled\n");

    printk("  contiguous base:0x%llx\n", param->contiguous_base);
    printk("  contiguous size:0x%llx\n", param->contiguous_size);
    printk("  command contiguous base:0x%llx\n", param->command_contiguous_base);
    printk("  command contiguous size:0x%llx\n", param->command_contiguous_size);
    printk("\n");
}

int setup_contiguous_memory(struct memory_heap *heap, n2d_uint64_t base, n2d_size_t size)
{
    n2d_error_t error = N2D_SUCCESS;
    struct resource *region = NULL;
    struct heap_node * node;

    if (base == 0 || size == 0) {
        heap->free = 0;
        return N2D_SUCCESS;
    }

    region = request_mem_region(base, size, "nano2d contiguous memory");

    if (!region) {
        printk("request mem %s(0x%llx - 0x%llx) failed\n",
            "nano2d contiguous memory", base, base + size - 1);

        ONERROR(N2D_OUT_OF_RESOURCES);
    }

    heap->physical = base;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
    heap->klogical = ioremap(base, size);
#else
    heap->klogical = ioremap_nocache(base, size);
#endif

    if (!heap->klogical) {
        release_mem_region(base, size);
        n2d_kernel_os_print("map contiguous memory(base:0x%lx size:0x%x) failed.\n", base, size);
        ONERROR(N2D_OUT_OF_RESOURCES);
    }

    n2d_kernel_os_trace(
            "allocated a %uB heap at physical:0x%08x, virtual:0x%llx\n",
            size, base, (n2d_uintptr_t)heap->klogical);

    /* Create the heap. */
    INIT_LIST_HEAD(&heap->list);
    heap->size = heap->free = size;
    heap->address = 0;
    if ((base + size) <= 0xFFFFFFFF)
        heap->limit_4G = N2D_TRUE;
    else
        heap->limit_4G = N2D_FALSE;

    node = n2d_kmalloc(sizeof(struct heap_node), GFP_KERNEL);
    if (node == NULL) {
        n2d_kernel_os_print("allocate heap node failed.\n");
        ONERROR(N2D_OUT_OF_RESOURCES);
    }

    node->offset = 0;
    node->size = size;
    node->status = 0;
    list_add(&node->list, &heap->list);

on_error:
    return error;
}

int release_contiguous_memory(struct memory_heap *heap)
{
    struct heap_node *pos;
    struct heap_node *n;

    if (heap->physical == 0 || heap->size == 0)
        return N2D_SUCCESS;

    if (heap->klogical != NULL) {
        release_mem_region(heap->physical, heap->size);
        iounmap((void *)heap->klogical);
    }

    /* Process each node. */
    list_for_each_entry_safe(pos, n, &heap->list, list) {
        /* Remove it from the linked list. */
        list_del(&pos->list);

        /* Free up the memory. */
        n2d_kfree(pos);
    }

    return N2D_SUCCESS;
}

int drv_open(struct inode * inode, struct file * file)
{
    n2d_error_t error = N2D_SUCCESS;
#if CONFIG_PM
    pm_runtime_get_sync((struct device *)global_device->dev);
#endif
    ONERROR(n2d_kernel_dispatch(global_device->kernel, N2D_KERNEL_COMMAND_OPEN, 0, 0, N2D_NULL));

on_error:
#if CONFIG_PM
    pm_runtime_mark_last_busy((struct device *)global_device->dev);
    pm_runtime_put_autosuspend((struct device *)global_device->dev);
#endif
    return error;
}

int drv_release(struct inode * inode, struct file * file)
{
    int allocate_count = 0;

#if CONFIG_PM
    pm_runtime_get_sync((struct device *)global_device->dev);
#endif
    n2d_kernel_dispatch(global_device->kernel, N2D_KERNEL_COMMAND_CLOSE, 0, 0, N2D_NULL);
    n2d_check_allocate_count(&allocate_count);
    if (allocate_count)
        n2d_kernel_os_print("Allocated %d memory\n", allocate_count);

#if CONFIG_PM
    pm_runtime_mark_last_busy((struct device *)global_device->dev);
    pm_runtime_put_autosuspend((struct device *)global_device->dev);
#endif
    return 0;
}

long drv_ioctl(struct file * file, unsigned int ioctl_code, unsigned long arg)
{
    int ret = 0;
    n2d_error_t error;
    n2d_ioctl_interface_t _iface, *iface;

    iface = &_iface;
    memset(iface, 0, sizeof(n2d_ioctl_interface_t));

    /*user data converted to kernel data*/
    if(copy_from_user(iface, (n2d_pointer)arg, sizeof(n2d_ioctl_interface_t))) {
        n2d_kernel_os_print("ioctl: failed to read data.\n");
        return -ENOTTY;
    }

    if (iface->command == N2D_KERNEL_COMMAND_OPEN || iface->command == N2D_KERNEL_COMMAND_CLOSE)
        return 0;

#if CONFIG_PM
    pm_runtime_get_sync((struct device *)global_device->dev);
#endif

    iface->error = n2d_kernel_dispatch(global_device->kernel, iface->command,
                                       iface->dev_id, iface->core, &iface->u);
    if (iface->error == N2D_INTERRUPTED) {
        ret = -ERESTARTSYS;
        ONERROR(iface->error);
    }

    /*kernel data converted to user data*/
    if(copy_to_user((n2d_pointer)arg, iface, sizeof(n2d_ioctl_interface_t))) {
        n2d_kernel_os_print("ioctl: failed to write data.\n");
        ret = -ENOTTY;
    }

on_error:
#if CONFIG_PM
    pm_runtime_mark_last_busy((struct device *)global_device->dev);
    pm_runtime_put_autosuspend((struct device *)global_device->dev);
#endif
    return ret;
}

ssize_t drv_read(struct file * file, char * buffer, size_t length, loff_t * offset)
{
    return 0;
}

int drv_mmap(struct file * file, struct vm_area_struct * vm)
{
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
    .compat_ioctl = drv_ioctl,
#endif
    .mmap           = drv_mmap,
};


static int drv_init(void)
{
    n2d_error_t error = N2D_SUCCESS;
    int i;
    unsigned long physical;
    unsigned int size;
    unsigned int irq_line;

    for(i = 0; i < NANO2D_CORE_MAX; i++ ) {
        if (global_device->irq_line[i] == -1)
            continue;

        physical = global_device->register_bases[i];
        size = global_device->register_sizes[i];
        irq_line = global_device->irq_line[i];

        if (physical != 0) {
            if (!global_device->register_bases_mapped[i]) {

                if(!request_mem_region(physical, size, "nano2d register region")) {
                    n2d_kernel_os_print("claim gpu:%d registers failed.\n", i);
                    return -1;
                }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
                /* Map the GPU registers. */
                global_device->register_bases_mapped[i] = (n2d_pointer)ioremap(physical, size);
#else
                global_device->register_bases_mapped[i] = (n2d_pointer)ioremap_nocache(physical, size);
#endif

                if (!global_device->register_bases_mapped[i]) {
                    n2d_kernel_os_print("map the gpu:%d registers:0x%llx size:0x%x failed.\n", i, physical, (n2d_uint32_t)size);
                    return -1;
                }
                global_device->reg_mem_requested[i] = 0;
            } else {
                global_device->reg_mem_requested[i] = (n2d_uint64_t)global_device->register_bases_mapped[i];
            }
        }
        global_device->core_num++;
    }

    if (global_device->contiguous_size > 0)
        ONERROR(setup_contiguous_memory(&global_device->heap, global_device->contiguous_base, global_device->contiguous_size));
    if (global_device->command_contiguous_size > 0)
        ONERROR(setup_contiguous_memory(&global_device->command_heap, global_device->command_contiguous_base, global_device->command_contiguous_size));

    ONERROR(n2d_kernel_os_construct(global_device, &global_device->os));
    ONERROR(n2d_kernel_iommu_construct(global_device, global_device->os));
    ONERROR(n2d_kernel_construct(global_device, &global_device->kernel));

    ONERROR(start_device(global_device));

    /* Register deviceex. */
    major = register_chrdev(0, N2D_DEVICE_NAME, &file_operations);
    if (major < 0) {
        n2d_kernel_os_print("register_chrdev failed.\n");
        return -1;
    }

    /* Create the graphics class. */
    device_class = class_create(THIS_MODULE, "nano2d_class");
    if (device_class == NULL) {
        n2d_kernel_os_print("class_create failed.\n");
        return -1;
    }

    /* Create the deviceex. */
    if (device_create(device_class, NULL, MKDEV(major, 0), NULL, N2D_DEVICE_NAME) == NULL) {
        n2d_kernel_os_print("device_create failed.\n");
        return -1;
    }

    n2d_kernel_os_print("create /dev/%s device.\n", N2D_DEVICE_NAME);

    return 0;
on_error:
    n2d_kernel_os_trace("create device failed\n");
    return -1;
}

static void drv_exit(void)
{
    int i = 0;

    destroy_debug_fs(global_device);
    stop_device(global_device);
    n2d_kernel_destroy(global_device->kernel);
    n2d_kernel_os_destroy(global_device->os);
    if (global_device->contiguous_size > 0)
        release_contiguous_memory(&global_device->heap);
    if (global_device->command_contiguous_size > 0)
        release_contiguous_memory(&global_device->command_heap);

    for(i = 0 ;i < NANO2D_CORE_MAX; i++ ) {
        if (global_device->irq_line[i] == -1)
            continue;
        if (global_device->reg_mem_requested[i] == 0) {
            if (global_device->register_bases_mapped[i] != NULL) {
                /* Unmap the GPU registers. */
                iounmap(global_device->register_bases_mapped[i]);
                release_mem_region((n2d_uint64_t)global_device->register_bases[i], global_device->register_sizes[i]);
            }
        }
    }

    if (device_class != NULL) {
        device_destroy(device_class, MKDEV(major, 0));
        /* Destroy the class. */
        class_destroy(device_class);
        unregister_chrdev(major, N2D_DEVICE_NAME);
    }

    n2d_kernel_os_print("nano2D exit.\n");
}

#if CONFIG_PM
static void gpu_pm_runtime_init(struct device *dev, int delay_ms)
{
    pm_runtime_use_autosuspend(dev);
    pm_runtime_set_autosuspend_delay(dev, delay_ms);
    pm_runtime_set_active(dev);
    pm_runtime_enable(dev);
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
static int gpu_probe(struct platform_device *pdev)
#else
static int __devinit gpu_probe(struct platform_device *pdev)
#endif
{
    n2d_error_t error;
    n2d_uint32_t ret;
    static u64 dma_mask = 0;
    struct device *dev_p = &(pdev->dev);

    platform->device = pdev;

    ret = n2d_kernel_platform_resource_init(platform);
    if (ret) {
        pr_err("%s: failed to init platform resource!\n", __func__);
        return ret;
    }

    global_device = n2d_kmalloc(sizeof(n2d_device_t), GFP_KERNEL);
    if (NULL == global_device) {
        n2d_kernel_os_print("allocate device structure failed.\n");
        return -1;
    }
    memset(global_device, 0, sizeof(n2d_device_t));

#if NANO2D_MMU_ENABLE
    dma_mask = DMA_BIT_MASK(40);
#else
    dma_mask = DMA_BIT_MASK(32);
#endif
    /* Power and clock. */
    if (platform->ops->getPower) {
        ONERROR(platform->ops->getPower(platform));

        ONERROR(platform->ops->set_power(platform, 1, N2D_TRUE));

        ONERROR(platform->ops->setClock(platform, 1, N2D_TRUE));
    }

    init_param(&global_param);

    sync_input_param(&global_param);

    /* Override default module param. */
    if (platform->ops->adjust_param) {
        /* Power and clock. */
        platform->ops->adjust_param(platform, &global_param);
    }

    ONERROR(sync_param(&global_param));

    if (global_param.iommu)
        dma_mask = DMA_BIT_MASK(32);

    dev_p->dma_mask = &dma_mask;
    dev_p->coherent_dma_mask = dma_mask;

    global_device->dev = (n2d_pointer)dev_p;
    global_device->platform = platform;

    ret = drv_init();
    if (ret != 0)
        return ret;

    ONERROR(create_debug_fs(global_device));

    show_param(&global_param, global_device);

#if CONFIG_PM
    gpu_pm_runtime_init(&pdev->dev, SUSPEND_DELAY_MS);
#endif
    return 0;

on_error:
    if (global_device)
        n2d_kfree(global_device);

    return 1;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
static int gpu_remove(struct platform_device *pdev)
#else
static int __devexit gpu_remove(struct platform_device *pdev)
#endif
{
    int allocate_count = 0;

#if CONFIG_PM
    pm_runtime_get_sync(&pdev->dev);
#endif
    drv_exit();
#if CONFIG_PM
    pm_runtime_dont_use_autosuspend(&pdev->dev);
    pm_runtime_put_sync(&pdev->dev);
    pm_runtime_disable(&pdev->dev);
#endif

    if (global_device) {
        n2d_kfree(global_device);
        global_device = N2D_NULL;
    }

    /* Power and clock. */
    if (platform->ops->putPower) {
        platform->ops->setClock(platform, 1, N2D_FALSE);
        platform->ops->set_power(platform, 1, N2D_FALSE);
        platform->ops->putPower(platform);
    }

    n2d_check_allocate_count(&allocate_count);
    if (allocate_count)
        n2d_kernel_os_print("The remaining %d memory are not freed\n", allocate_count);

    return 0;
}

static int gpu_suspend(struct platform_device *dev, pm_message_t state)
{
    n2d_error_t error = N2D_SUCCESS;
    n2d_uint32_t i = 0, j = 0;

#ifdef CONFIG_PM
    pm_runtime_get_sync(&dev->dev);
#endif

    for (i = 0; i < global_device->kernel->dev_num; i++) {
        gcmkASSERT(global_device->kernel->sub_dev[i]->id == i);

        for (j = 0; j < global_device->kernel->dev_core_num; j++)
            ONERROR(n2d_kernel_hardware_set_power(global_device->kernel->sub_dev[i]->hardware[j], N2D_POWER_OFF));
    }

on_error:
#ifdef CONFIG_PM
    pm_runtime_mark_last_busy(&dev->dev);
    pm_runtime_put_autosuspend(&dev->dev);
#endif
    return error;
}

static int gpu_resume(struct platform_device *dev)
{
    n2d_error_t error = N2D_SUCCESS;
    n2d_uint32_t i = 0, j = 0;

#ifdef CONFIG_PM
    pm_runtime_get_sync(&dev->dev);
#endif

    for (i = 0; i < global_device->kernel->dev_num; i++) {
        gcmkASSERT(global_device->kernel->sub_dev[i]->id == i);

        for (j = 0; j < global_device->kernel->dev_core_num; j++)
            ONERROR(n2d_kernel_hardware_set_power(global_device->kernel->sub_dev[i]->hardware[j], N2D_POWER_ON));
    }

on_error:
#ifdef CONFIG_PM
    pm_runtime_mark_last_busy(&dev->dev);
    pm_runtime_put_autosuspend(&dev->dev);
#endif
    return error;
}

#ifdef CONFIG_PM
#ifdef CONFIG_PM_SLEEP
static int gpu_system_suspend(struct device *dev)
{
    pm_message_t state = {0};
    return gpu_suspend(to_platform_device(dev), state);
}

static int gpu_system_resume(struct device *dev)
{
    return gpu_resume(to_platform_device(dev));
}
#endif

static int gpu_system_runtime_suspend(struct device *dev)
{
    if (platform)
        return n2d_disable_clock(platform);
    else
        return 0;
}

static int gpu_system_runtime_resume(struct device *dev)
{
    if (platform)
        return n2d_enable_clock(platform);
    else
        return 0;
}

static int gpu_system_runtime_idle(struct device *dev)
{
    pm_runtime_mark_last_busy(dev);
    pm_runtime_autosuspend(dev);
    /* we don't want the main rpm_idle to call suspend - we want to autosuspend */
    return 1;
}

static const struct dev_pm_ops default_gpu_pm_ops =
{
    SET_SYSTEM_SLEEP_PM_OPS(gpu_system_suspend, gpu_system_resume)
    SET_RUNTIME_PM_OPS(gpu_system_runtime_suspend, gpu_system_runtime_resume,
                       gpu_system_runtime_idle)
};
#endif

static struct platform_driver gpu_driver =
{
    .probe      = gpu_probe,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
    .remove     = gpu_remove,
#else
    .remove     = __devexit_p(gpu_remove),
#endif

    .suspend    = gpu_suspend,
    .resume     = gpu_resume,

    .driver     =
    {
        .name   = N2D_DEVICE_NAME,
#if CONFIG_PM
        .pm     = &default_gpu_pm_ops,
#endif
    }
};

static int gpu_init(void)
{
    int ret;

    ret = n2d_kernel_platform_init(&gpu_driver,&platform);
    if (ret || !platform)
    {
        printk(KERN_ERR "galcore: Soc platform init failed.\n");
        return -ENODEV;
    }

    n2d_kernel_os_query_operations(&platform->ops);

    ret = platform_driver_register(&gpu_driver);

    if (ret)
    {
        printk(KERN_ERR "galcore: gpu_init() failed to register driver!\n");
        n2d_kernel_platform_terminate(platform);
        platform = NULL;
        return -ENODEV;
    }

    platform->driver = &gpu_driver;

    return 0;

}

static void gpu_exit(void)
{
    platform_driver_unregister(platform->driver);
    n2d_kernel_platform_terminate(platform);
    platform = N2D_NULL;
}

module_init(gpu_init);
module_exit(gpu_exit);

