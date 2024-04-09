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
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/delay.h>
#include <linux/pm_opp.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/devfreq.h>
#include <linux/devfreq_cooling.h>
#include <linux/iommu.h>
#include "nano2D_kernel_platform.h"

struct n2d_platform_data {
    bool is_clock_on;
    int num_clocks;
    unsigned long clk_max_rate;
    unsigned long clk_cur_rate;
    struct clk_bulk_data *clocks;
    struct reset_control *rst_ctrls;
};

static const struct of_device_id compatibles[] = {
    {  .name = "r2d", .compatible = "siengine,r2d", },
    {},
};

//return non-zero if match the device
static int match_id(struct device *dev, const void *data)
{
    const struct of_device_id *match;
    match = of_match_device((struct of_device_id*) data, dev);
    return (int) (match !=NULL);
}

int n2d_enable_clock(n2d_linux_platform_t *platform)
{
    struct n2d_platform_data *data = (struct n2d_platform_data *)platform->priv;
    int err = 0;

    if (data->is_clock_on)
        return 0;

    pr_info("%s\n", __func__);
    do {
        if (clk_bulk_enable(data->num_clocks, data->clocks)) {
            pr_err("%s: clk_bulk_enable error!\n", __func__);
            err = -EBUSY;
            break;
        }
        if (reset_control_deassert(data->rst_ctrls)) {
            pr_err("%s: reset control deassert error!\n", __func__);
            err = -EBUSY;
            break;
        }
        // wait (32 + 128) cycles for the slowest clock (pclk 200M) before ready, it is about 1us
        udelay(1);
        clk_set_rate(data->clocks[0].clk, data->clk_cur_rate);
        data->is_clock_on = true;
    } while (0);

    return err;
}

int n2d_disable_clock(n2d_linux_platform_t *platform)
{
    struct n2d_platform_data *data = (struct n2d_platform_data *)platform->priv;

    if (!data->is_clock_on)
        return 0;

#ifndef CONFIG_DISABLE_R2D_POWER_SAVING
    pr_info("%s\n", __func__);
    data->clk_cur_rate = clk_get_rate(data->clocks[0].clk);
    clk_set_rate(data->clocks[0].clk, data->clk_max_rate >> 6);
    clk_bulk_disable(data->num_clocks, data->clocks);
    if(reset_control_assert(data->rst_ctrls))
        pr_warn("%s: reset control assert error!\n", __func__);
    data->is_clock_on = false;
#endif

    return 0;
}

n2d_error_t _get_power(IN n2d_linux_platform_t *Platform)
{
    (void)Platform;
    return N2D_SUCCESS;
}

n2d_error_t _put_power(IN n2d_linux_platform_t *Platform)
{
    (void)Platform;
    return N2D_SUCCESS;
}

n2d_error_t _set_power(IN n2d_linux_platform_t *Platform,
    IN n2d_int32_t GPU, IN n2d_bool_t Enable)
{
    if (Enable == N2D_TRUE)
        return n2d_enable_clock(Platform);
    else
        return n2d_disable_clock(Platform);
    return N2D_SUCCESS;
}

n2d_error_t _set_clock(IN n2d_linux_platform_t *Platform,
    IN n2d_int32_t GPU,IN n2d_bool_t Enable)
{
    if (Enable == N2D_TRUE)
        return n2d_enable_clock(Platform);
    else
        return n2d_disable_clock(Platform);
}

n2d_error_t _reset(IN n2d_linux_platform_t *Platform,
    IN n2d_int32_t GPU)
{
    return N2D_SUCCESS;
}

n2d_error_t _adjust_param(IN n2d_linux_platform_t *Platform,
    OUT n2d_linux_module_parameters_t *Args)
{
    struct device_node *node;
    struct resource *res;
    struct resource r;
    int ret;
    struct platform_device *pdev;
    struct device *dev = &Platform->device->dev;

    pdev = of_find_device_by_node(dev->of_node);
    if (!pdev)
        return -ENOENT;

    res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "R2D");
    if (!res) {
        pr_err("%s: no irq resource find!\n", __func__);
        return -ENOENT;
    }
    Args->irq_line[0] = res->start;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0); //R2D: 0xEC000000
    if (!res) {
        pr_err("%s: no mem resource find!\n", __func__);
        return -ENOENT;
    }
    Args->register_bases[0] = (ulong)res->start;
    Args->register_sizes[0] = (ulong)resource_size(res);

    node = of_parse_phandle(dev->of_node, "memory-region", 0);
    if (!node) {
        pr_info("%s: no memory-region specified\n", __func__);
        return 0;
    }
    ret = of_address_to_resource(node, 0, &r);
    if (ret) {
        pr_err("%s: get resource failed!\n", __func__);
        return ret;
    }
    of_node_put(node);
    Args->contiguous_size = resource_size(&r);
    Args->contiguous_base = r.start;

    if (iommu_get_domain_for_dev(dev))
        Args->iommu = true;
    else
        Args->iommu = false;

    pr_info("%s:irq:%d,register_base:0x%llx,size:0x%x,memory_base:0x%llx,size:0x%llx,iommu:%d\n",
            __func__, Args->irq_line[0], Args->register_bases[0], Args->register_sizes[0],
            Args->contiguous_base, Args->contiguous_size, Args->iommu);

    return 0;
}

n2d_error_t _get_gpu_physical(IN n2d_linux_platform_t *Platform,
    IN n2d_uint64_t CPUPhysical, OUT n2d_uint64_t *GPUPhysical)
{
    *GPUPhysical = CPUPhysical;
    return N2D_SUCCESS;
}

n2d_error_t _get_cpu_physical(IN n2d_linux_platform_t *Platform,
    IN n2d_uint64_t GPUPhysical, OUT n2d_uint64_t *CPUPhysical)
{
    *CPUPhysical = GPUPhysical;
    return N2D_SUCCESS;
}

static struct n2d_linux_operations default_ops =
{
    .adjust_param   = _adjust_param,
    .get_gpu_physical = _get_gpu_physical,
    .get_cpu_physical = _get_cpu_physical,
    .getPower = _get_power,
    .set_power = _set_power,
    .putPower = _put_power,
    .setClock = _set_clock,
    .reset = _reset,
};

/******************* SE1000 G2D DEVFREQ support *******************/
#ifndef CONFIG_DISABLE_R2D_POWER_SAVING
static int n2d_devfreq_target(struct device *dev, unsigned long *freq,
    u32 flags)
{
    struct dev_pm_opp *opp;
    int err;

    opp = devfreq_recommended_opp(dev, freq, flags);
    if (IS_ERR(opp))
        return PTR_ERR(opp);
    dev_pm_opp_put(opp);

    err = dev_pm_opp_set_rate(dev, *freq);

    return err;
}

static int n2d_devfreq_get_dev_status(struct device *dev,
    struct devfreq_dev_status *status)
{
    struct clk *clk = dev_get_drvdata(dev);

    status->busy_time = 0;
    status->total_time = 0;
    status->current_frequency = clk_get_rate(clk);

    return 0;
}

static struct devfreq_dev_profile demo_devfreq_profile = {
    .polling_ms = 1000, /* 1 second */
    .target = n2d_devfreq_target,
    .get_dev_status = n2d_devfreq_get_dev_status,
};

static int n2d_init_opp(struct device* dev, struct n2d_platform_data *data)
{
    int i;
    struct opp_table *opp_table;
    struct dev_pm_opp *opp;
    unsigned long max_freq;
    unsigned long cur_freq;
    struct devfreq *devfreq;
    struct thermal_cooling_device *cooling;

    opp_table = dev_pm_opp_set_clkname(dev, data->clocks[0].id);
    if (IS_ERR(opp_table)) {
        dev_warn(dev, "could not get alloc opp_table\n");
        return PTR_ERR(opp_table);
    }
#define DEV_PM_OPP_MAX 4
    cur_freq = max_freq = data->clk_max_rate = clk_get_rate(data->clocks[0].clk);
    data->clk_cur_rate = data->clk_max_rate;
    for (i = 0; i < DEV_PM_OPP_MAX; i++) {
        dev_pm_opp_add(dev, cur_freq, DEV_PM_OPP_MAX - i);
        cur_freq >>= 1;
    }

    opp = devfreq_recommended_opp(dev, &max_freq, 0);
    if (IS_ERR(opp)) {
        dev_warn(dev, "could not devfreq_recommended_opp\n");
        return PTR_ERR(opp);
    }

    demo_devfreq_profile.initial_freq = max_freq;
    dev_pm_opp_put(opp);

    devfreq = devm_devfreq_add_device(dev, &demo_devfreq_profile,
                                  DEVFREQ_GOV_SIMPLE_ONDEMAND, NULL);
    if (IS_ERR(devfreq)) {
        dev_warn(dev, "Couldn't initialize GPU devfreq\n");
        return PTR_ERR(devfreq);
    }

    cooling = of_devfreq_cooling_register(dev->of_node, devfreq);
    if (IS_ERR(cooling))
        dev_warn(dev, "Failed to register cooling device\n");
    return 0;
}
#endif

static n2d_linux_platform_t default_platform =
{
    .name = __FILE__,
    .ops  = &default_ops,
};

static int n2d_parse_bulk_data(struct device *dev,
    struct n2d_platform_data *data)
{
    struct clk_bulk_data *bulk_data;
    struct property *prop;
    const char *name;
    int count, ret, i = 0;

    count = of_property_count_strings(dev->of_node, "clock-names");
    if (count < 1) {
        pr_err("%s: no defined of clocks!\n", __func__);
        return -EINVAL;
    }

    bulk_data = devm_kzalloc(dev, sizeof(struct clk_bulk_data) * count, GFP_KERNEL);
    if (!bulk_data) {
        pr_err("%s: failed to alloc bulk data!\n", __func__);
        return -ENOMEM;
    }

    of_property_for_each_string(dev->of_node, "clock-names", prop, name) {
        bulk_data[i].id = devm_kstrdup(dev, name, GFP_KERNEL);
        if (!bulk_data[i].id) {
            ret = -ENOMEM;
            goto on_error;
        }
        i++;
    }

    ret = devm_clk_bulk_get(dev, count, bulk_data);
    if (ret) {
        pr_err("%s: failed to get bulk clk!\n", __func__);
        goto bulk_get_error;
    }

    ret = clk_bulk_prepare(count, bulk_data);
    if (ret) {
        pr_err("%s: failed to prepare bulk clk!\n", __func__);
        goto bulk_prepare_error;
    }

    data->is_clock_on = false;
    data->num_clocks = count;
    data->clocks = bulk_data;
    return 0;

bulk_prepare_error:
    clk_bulk_put(count, bulk_data);
bulk_get_error:
    for (i = 0; i < count; i++)
        if (bulk_data[i].id)
            devm_kfree(dev, (void *)bulk_data[i].id);
on_error:
    if (bulk_data)
        devm_kfree(dev, bulk_data);
    return ret;
}

static void n2d_release_bulk_data(struct device *dev,
    struct n2d_platform_data *data)
{
    if (!data)
        return;

    if (data->clocks) {
        int i;

        clk_bulk_unprepare(data->num_clocks, data->clocks);
        clk_bulk_put(data->num_clocks, data->clocks);
        for (i = 0; i < data->num_clocks; i++) {
            if (data->clocks[i].id)
                devm_kfree(dev, (void *)data->clocks[i].id);
        }
        devm_kfree(dev, data->clocks);
    }
}

static int n2d_parse_reset_controls(struct device *dev,
    struct n2d_platform_data *data)
{
    data->rst_ctrls = devm_reset_control_array_get(dev, true, true);
    if (PTR_ERR(data->rst_ctrls) == -EPROBE_DEFER) {
        pr_err("%s: devm_reset_control_array_get error!\n", __func__);
        return -EINVAL;
    }

    return 0;
}

n2d_int32_t n2d_kernel_platform_resource_init(n2d_linux_platform_t *platform)
{
    struct device *dev = &platform->device->dev;
    struct n2d_platform_data *data;
    n2d_int32_t ret;

    data = devm_kzalloc(dev, sizeof(struct n2d_platform_data), GFP_KERNEL);
    if (!data) {
        pr_err("%s: failed to alloc platform data!\n", __func__);
        return -ENOMEM;
    }

    ret = n2d_parse_bulk_data(dev, data);
    if (ret) {
        pr_err("%s: failed to parse bulk data!\n", __func__);
        goto parse_bulk_err;
    }

    ret = n2d_parse_reset_controls(dev, data);
    if (ret) {
        pr_err("%s: failed to parse reset controls!\n", __func__);
        goto parse_rst_ctl_err;
    }

#ifndef CONFIG_DISABLE_R2D_POWER_SAVING
    n2d_init_opp(dev, data);
    pr_info("n2d: PM OOP enabled.\n");
#endif

    platform->priv = (void *)data;
    return 0;

parse_rst_ctl_err:
    n2d_release_bulk_data(dev, data);
parse_bulk_err:
    devm_kfree(dev, data);
    return ret;
}

static struct platform_device *default_dev;

n2d_int32_t n2d_kernel_platform_init(struct platform_driver *pdrv,
    n2d_linux_platform_t **platform)
{
    n2d_int32_t ret;
    struct device *dev;
    struct device *ofdev;

    default_dev = platform_device_alloc(pdrv->driver.name, -1);
    if (!default_dev) {
        pr_err("%s: platform_device_alloc failed.\n", __func__);
        return -ENOMEM;
    }

    /* Add device */
    ret = platform_device_add(default_dev);
    if (ret) {
        pr_err("%s: platform_device_add failed.\n", __func__);
        goto on_error;
    }

    dev = &default_dev->dev;
    ofdev = bus_find_device(dev->bus, NULL, (void*)compatibles, match_id);
    if (ofdev) {
        dev->of_node = ofdev->of_node;
    } else {
        pr_err("%s: of node not found\n", __func__);
        ret = -ENOENT;
        goto find_device_err;
    }

    *platform = (n2d_linux_platform_t *)&default_platform;
    return 0;

find_device_err:
    platform_device_del(default_dev);
on_error:
    platform_device_put(default_dev);
    default_dev = NULL;
    return ret;
}

n2d_int32_t n2d_kernel_platform_terminate(n2d_linux_platform_t *platform)
{
    if (default_dev) {
        if (platform->priv) {
            struct n2d_platform_data *data = (struct n2d_platform_data *)platform->priv;
            n2d_release_bulk_data(&default_dev->dev, data);
            devm_kfree(&default_dev->dev, data);
            platform->priv = NULL;
        }
        platform_device_unregister(default_dev);
        default_dev = NULL;
    }

    return 0;
}

void n2d_kernel_os_query_operations(n2d_linux_operations_t **ops)
{
    *ops = &default_ops;
}
