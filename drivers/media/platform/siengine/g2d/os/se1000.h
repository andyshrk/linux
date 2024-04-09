/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2020 Siengine Corporation
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
*****************************************************************************/
/* SE1000 example of GPU2 DTS for supporting SMMU

/ {
        smmu_gpu2_r2d:smmu_gpu2_r2d@0xec200000 {
                compatible = "arm,smmu-v3";
                reg = <0x0 0xec200000 0x0 0x200000>;
                interrupts = < 0 371 1>, <0 369 1>, <0 367 1>;
                interrupt-names = "eventq", "cmdq-sync", "gerror";
                #iommu-cells = <1>;
                clocks = <&crg_clk GPU2_R2D_CLK>, <&crg_clk GPU2_R2D_PCLK>, <&crg_clk GPU2_V2D_CLK>;
                clock-names = "gpu2_r2d_clk", "gpu2_r2d_pclk", "gpu2_v2d_clk";
                resets = <&crg_clk GPU2_R2D_CLK>;
                reset-names = "gpu2_r2d_clk";

        };

        smmu_gpu2_v2d:smmu_gpu2_v2d@0xec400000 {
                compatible = "arm,smmu-v3";
                reg = <0x0 0xec400000 0x0 0x200000>;
                interrupts = < 0 360 1>, <0 358 1>, <0 356 1>;
                interrupt-names = "eventq", "cmdq-sync", "gerror";
                #iommu-cells = <1>;
                clocks = <&crg_clk GPU2_V2D_CLK>;
                clock-names = "gpu2_v2d_clk";
                resets = <&crg_clk GPU2_V2D_CLK>;
                reset-names = "gpu2_v2d_clk";

        };

        gpu2_r2d:r2d@ec000000 {
                interrupts = <0 252 4>;
                interrupt-names = "R2D";
                reg = <0x0 0xec000000 0x0 0x100000 >;
                #size-cells = <0x2>;
                #address-cells = <0x2>;
                clocks = <&crg_clk GPU2_R2D_CLK>, <&crg_clk GPU2_R2D_PCLK>, <&crg_clk GPU2_V2D_CLK>;
                clock-names = "gpu2_r2d_clk", "gpu2_r2d_pclk", "gpu2_v2d_clk";
                resets = <&crg_clk GPU2_R2D_CLK>;
                reset-names = "gpu2_r2d_clk";
                compatible = "siengine,r2d";
                iommus = <&smmu_gpu2_r2d 0>;
        };

        gpu2_v2d:v2d@ec100000 {
                interrupts = <0 255 4>;
                interrupt-names = "V2D";
                reg = < 0x0 0xec100000 0x0 0x100000>;
                #size-cells = <0x2>;
                #address-cells = <0x2>;
                clocks = <&crg_clk GPU2_R2D_CLK>, <&crg_clk GPU2_R2D_PCLK>, <&crg_clk GPU2_V2D_CLK>;
                clock-names = "gpu2_r2d_clk", "gpu2_r2d_pclk", "gpu2_v2d_clk";
                resets = <&crg_clk GPU2_V2D_CLK>;
                reset-names = "gpu2_v2d_clk";
                compatible = "siengine,v2d";
                iommus = <&smmu_gpu2_v2d 0>;
        };

};

*****************************************************************************/
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/pm_opp.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/devfreq.h>
#include <linux/devfreq_cooling.h>
static int is_clk_on = 0;
static struct timer_list clk_timer;
DEFINE_SPINLOCK(g2d_clk_lock);

#if gcdENABLE_VG
#define MAX_CLOCK_RATE 1600000000
#define MAX_CLK_SOURCES 1
static struct clk_bulk_data clks[MAX_CLK_SOURCES] = {
    {"gpu2_v2d_clk", NULL}
};

#else
#define MAX_CLOCK_RATE 800000000
#define MAX_CLK_SOURCES 2
static struct clk_bulk_data clks[MAX_CLK_SOURCES] = {
    {"gpu2_r2d_clk",NULL}, {"gpu2_r2d_pclk", NULL}
};
#endif

/******************* SE1000 G2D DEVFREQ support *******************/
#ifndef CONFIG_DISABLE_R2D_POWER_SAVING
static int init_opp(struct device* dev);
static int g2d_devfreq_target(struct device *dev, unsigned long *freq,
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

static int g2d_devfreq_get_dev_status(struct device *dev,
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
    .target = g2d_devfreq_target,
    .get_dev_status = g2d_devfreq_get_dev_status,
};

static int init_opp(struct device* dev)
{
    int i;
    struct opp_table *opp_table;
    struct dev_pm_opp *opp;
    unsigned long max_freq;
    unsigned long cur_freq;
    struct devfreq *devfreq;
    struct thermal_cooling_device *cooling;

    opp_table = dev_pm_opp_set_clkname(dev, clks[0].id);
    if (IS_ERR(opp_table)) {
        dev_warn(dev, "could not get alloc opp_table\n");
        return PTR_ERR(opp_table);
    }
#define DEV_PM_OPP_MAX 4
    cur_freq = MAX_CLOCK_RATE;
    for (i = 0; i < DEV_PM_OPP_MAX; i++) {
        dev_pm_opp_add(dev, cur_freq, DEV_PM_OPP_MAX - i);
        cur_freq >>= 1;
    }
    max_freq = MAX_CLOCK_RATE;
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
/******************* SE1000 G2D Clock control *******************/
static struct reset_control *rst_ctrls = NULL;
static int CLK_TIMEOUT = 1;
static int set_clock_rate(int shifter    )
{
    if (clks[0].clk)
        return clk_set_rate(clks[0].clk, MAX_CLOCK_RATE>>shifter);
    return -1;
}
static void _enable_clock(void)
{
    unsigned long flags;
    if(is_clk_on)
        return;

    spin_lock_irqsave(&g2d_clk_lock, flags);
    do {
        // clock
        if( 0 != devm_clk_bulk_get(galcore_device, MAX_CLK_SOURCES, clks)){
            pr_err("enable_clock: devm_clk_bulk_get failed!\n");
            break;
        }
        if ( 0 != clk_bulk_prepare(MAX_CLK_SOURCES, clks)) {
            pr_err("enable_clock: clk_bulk_enable error!\n");
            break;
        }
        if ( 0 != set_clock_rate(0) ) {
            pr_warn("enable_clock: clk_set_rate error!\n");
        }
        if( 0 != clk_bulk_enable(MAX_CLK_SOURCES, clks)) {
            pr_err("enable_clock: clk_bulk_enable error!\n");
            break;
        }

        // reset
        if (rst_ctrls == NULL) {
            rst_ctrls = devm_reset_control_array_get(galcore_device, true, true);
            if (PTR_ERR(rst_ctrls) == -EPROBE_DEFER) {
                rst_ctrls = NULL;
                pr_err("enable_clock: devm_reset_control_array_get error!\n");
                break;
            }
        }
        if (0 != reset_control_deassert(rst_ctrls)) {
            pr_err("enable_clock: devm_reset_control_array_get error!\n");
            break;
        }
        // wait (32 + 128) cycles for the slowest clock (pclk 200M) before ready, it is about 1us
        udelay(1);
        is_clk_on = 1;
        pr_info("g2d: _enable_clock\n");
    }while(0);

    spin_unlock_irqrestore(&g2d_clk_lock, flags);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,20,0)
static void _disable_clock(struct timer_list * value)
#else
static void _disable_clock(unsigned long value)
#endif
{
    (void)value;
    if (is_clk_on) {
        unsigned long flags;
        spin_lock_irqsave(&g2d_clk_lock, flags);
#ifndef CONFIG_DISABLE_R2D_POWER_SAVING
        // clock
        set_clock_rate(6);
        clk_bulk_disable(MAX_CLK_SOURCES, clks);
        clk_bulk_unprepare(MAX_CLK_SOURCES, clks);
#endif
        clk_bulk_put(MAX_CLK_SOURCES, clks);

        // reset
        if (rst_ctrls) {
#ifndef CONFIG_DISABLE_R2D_POWER_SAVING
            reset_control_assert(rst_ctrls);
            pr_info("g2d: _disable_clock\n");
#endif
            reset_control_put(rst_ctrls);
            rst_ctrls = NULL;
        }
        is_clk_on = 0;
        spin_unlock_irqrestore(&g2d_clk_lock, flags);
    }
}


void init_clock(void)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,20,0)
    timer_setup(&clk_timer, _disable_clock, 0);
#else
    init_timer(&clk_timer);
    clk_timer.function = &_disable_clock;
#endif
    mod_timer(&clk_timer, jiffies + CLK_TIMEOUT * HZ);
    _enable_clock();
}

void delete_clock(void)
{
    _disable_clock(0);
    del_timer(&clk_timer);
}

void check_clock(void)
{
    _enable_clock();
    mod_timer(&clk_timer, jiffies + CLK_TIMEOUT * HZ); /* the interval is 10s */
}

static const struct of_device_id compatibles[] = {
#if gcdENABLE_VG
    {  .name = "v2d", .compatible = "siengine,v2d", },
#else
    {  .name = "r2d", .compatible = "siengine,r2d", },
#endif
    {},
};
//return non-zero if match the device
static int match_id(struct device *dev, const void *data)
{
    const struct of_device_id *match;
    match = of_match_device((struct of_device_id*) data, dev);
    return (int) (match !=NULL);
}
inline int config_from_of(struct device* dev)
{
    struct resource *res;
    struct platform_device *pdev;
    int ret = 0;
    if(!dev->of_node){
        struct device* ofdev;
        ofdev = bus_find_device(dev->bus, NULL, (void*) compatibles, match_id);
        if (ofdev) {
            dev->of_node = ofdev->of_node;
        } else {
            pr_err("[galcore] of not found\n");
            return -1;
        }
    }
    pdev = of_find_device_by_node(dev->of_node);
    if (!pdev)
        return -1;

    irqLine2D = -1;
    irqLineVG = -1;
#if gcdENABLE_2D
    res = platform_get_resource_byname( pdev, IORESOURCE_IRQ, "R2D" );
    if (res) {
        irqLine2D = res->start;
        res = platform_get_resource( pdev, IORESOURCE_MEM, 0 ); //R2D: 0xEC000000
    }
    if (res) {
        registerMemBase2D = (ulong) res->start;
        registerMemSize2D = (ulong) (res->end - res->start);
    }
    pr_info("irq2D=%d, registerMemBase2D = 0x%lx, size=0x%lx\n", irqLine2D, registerMemBase2D, registerMemSize2D);
#elif gcdENABLE_VG
    res = platform_get_resource_byname( pdev, IORESOURCE_IRQ, "V2D" );
    if (res) {
        irqLineVG = res->start;
        res = platform_get_resource( pdev, IORESOURCE_MEM, 0 ); //V2D: 0xEC000000
    }
    if (res) {
        registerMemBaseVG = (ulong) res->start;
        registerMemSizeVG = (ulong) (res->end - res->start);
    }
    pr_info("irqVG=%d, registerMemBaseVG = 0x%lx, size=0x%lx\n", irqLineVG, registerMemBaseVG, registerMemSizeVG);
#endif
    /*SE1000: clock must be enabled after config_of and before smmu of configure */
    of_property_read_u32(dev->of_node, "idle-timeout-sec", &CLK_TIMEOUT);
    pr_info("set idle-timeout-sec %d seconds\n", CLK_TIMEOUT);
    init_clock();
    /* configure DMA */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
    /* set TRUE regardless of "dma-ranges" */
    ret = of_dma_configure(dev, dev->of_node, true);
#else
    ret = of_dma_configure(dev, dev->of_node);
#endif
#ifndef CONFIG_DISABLE_R2D_POWER_SAVING
    init_opp(dev);
    pr_info("g2d: PM OOP enabled.\n");
#endif
    return ret;
}

