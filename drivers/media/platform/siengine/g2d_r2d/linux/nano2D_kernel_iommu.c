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
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include "nano2D_enum.h"
#include "nano2D_types.h"
#include "nano2D_kernel_driver.h"
#include "nano2D_kernel_base.h"
#include "nano2D_kernel_iommu.h"

n2d_error_t n2d_kernel_iommu_check_enable(struct device *dev, struct iommu_domain *domain)
{
    struct page *page;
    uint64_t phys;
    dma_addr_t dmaHandle;

    page = alloc_page(GFP_KERNEL);
    if (!page)
        return N2D_OUT_OF_MEMORY;

    phys = page_to_phys(page);
    dmaHandle = dma_map_page(dev, page, 0, PAGE_SIZE, DMA_TO_DEVICE);
    if (dmaHandle)
        dma_unmap_page(dev, dmaHandle, PAGE_SIZE, DMA_FROM_DEVICE);

    __free_page(page);
    if (phys == dmaHandle)
        return N2D_NO_CONTEXT;

    return N2D_SUCCESS;
}

n2d_error_t n2d_kernel_iommu_construct(n2d_device_t *n2d_device, n2d_os_t *os)
{
    struct device *dev;
    n2d_iommu_t *iommu;
    struct iommu_domain *domain;
    n2d_error_t error = N2D_SUCCESS;

    dev = (struct device *)n2d_device->dev;
    domain = iommu_get_domain_for_dev(dev);
    if (!domain) {
        os->iommu = NULL;
        pr_info("%s: no domain exist!\n", __func__);
        return error;
    }

    if (n2d_kernel_iommu_check_enable(dev, domain)) {
        os->iommu = NULL;
        pr_info("%s: iommu not enabled!\n", __func__);
        return error;
    }

    ONERROR(n2d_kernel_os_allocate(os, sizeof(n2d_iommu_t), (n2d_pointer)&iommu));
    ONERROR(n2d_kernel_os_memory_fill(os, iommu, 0, sizeof(n2d_iommu_t)));

    iommu->domain = domain;
    iommu->device = dev;
    os->iommu = iommu;

    pr_info("%s: iommu enabled!\n", __func__);
    return N2D_SUCCESS;

on_error:
    if (iommu != NULL) {
        n2d_kernel_os_free(os, iommu);
        os->iommu = NULL;
    }

    return error;
}