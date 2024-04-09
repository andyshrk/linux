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
*    Copyright (C) 2021 Siengine Corporation
*
*    GPL License. If you wish to use only one license not the other, you can
*    indicate your decision by deleting one of the above license notices in your
*    version of this file.
*
*****************************************************************************/
#include <linux/clk.h>
#include <linux/reset.h>

#include "gc_hal_kernel_linux.h"
#include "gc_hal_kernel_platform.h"




static gceSTATUS _AdjustParam(IN gcsPLATFORM *Platform, OUT gcsMODULE_PARAMETERS *Args);
static gceSTATUS _GetPower(IN gcsPLATFORM * Platform);
static gceSTATUS _PutPower(IN gcsPLATFORM * Platform);
static gceSTATUS _GetGPUPhysical(IN gcsPLATFORM * Platform, IN gctPHYS_ADDR_T CPUPhysical,
        OUT gctPHYS_ADDR_T *GPUPhysical);
static gceSTATUS  _ExternalCacheOperation(IN gcsPLATFORM *Platform, IN gceCACHEOPERATION Operation);
static struct _gcsPLATFORM_OPERATIONS default_ops =
{
    .adjustParam   = _AdjustParam,
    .getPower = _GetPower,
    .putPower = _PutPower,
    .getGPUPhysical = _GetGPUPhysical,
     externalCacheOperation = _ExternalCacheOperation,
};

static struct _gcsPLATFORM default_platform =
{
    .name = __FILE__,
    .ops  = &default_ops,
};
static gceSTATUS _AdjustParam(IN gcsPLATFORM *Platform, OUT gcsMODULE_PARAMETERS *Args)
{
    return gcvSTATUS_OK;
}
//#define DIRECT_CLOCK
#ifdef DIRECT_CLOCK
extern uint gClockDivisor;
/* directly control CLOCK registers */
#define CR_WRAP_TOP_BASE_ADDRESS        0x581e0000
#define CR_WRAP_TOP_BASE_LENGTH         0x00003010
#define CLK_R2D_CORE                    0x00002000
#define CLK_R2D_PCLK                    0x00002004
#define CLK_V2D_CORE                    0x00002008
#define CLK_CG_EN                       0x00010000
#define CLK_OUT_SEL                     0x00000100
#define CLK_DIV_MASK                    0x000000FF /*clock divided by 2^DIV */

#define REG_R2D_AXI_RST_N               0x00003000
#define REG_V2D_AXI_RST_N               0x00003004

#define RST_N_MASK                      0xfffffffe
#define RST_N_ASSERT                    0x00000001

#define ENABLE_CLOCK(clk_addr) \
    iowrite32(((u32)ioread32((void*)(clk_regs + clk_addr))) | CLK_CG_EN | CLK_OUT_SEL | (CLK_DIV_MASK & gClockDivisor), (void*)(clk_regs + clk_addr))

#define RESET_DEVICE(address, reset) \
    (reset)? writel(0, (void* )(clk_regs + address)) \
            : writel(RST_N_ASSERT, (void* )(clk_regs + address))


static gceSTATUS _GetPower(IN gcsPLATFORM * Platform)
{
    volatile unsigned char *clk_regs = (volatile u8 *) ioremap_nocache(CR_WRAP_TOP_BASE_ADDRESS, CR_WRAP_TOP_BASE_LENGTH);
    printk("%s: directly control clock. clock_divisor=2^%x\n", __FUNCTION__, gClockDivisor);
    ENABLE_CLOCK(CLK_R2D_CORE);
    ENABLE_CLOCK(CLK_R2D_PCLK);
    ENABLE_CLOCK(CLK_V2D_CORE);
    RESET_DEVICE(REG_R2D_AXI_RST_N, 1);
    RESET_DEVICE(REG_V2D_AXI_RST_N, 1);
    iounmap((void *) clk_regs);
    return gcvSTATUS_OK;
}
static gceSTATUS _PutPower(IN gcsPLATFORM * Platform)
{
    volatile unsigned char *clk_regs = (volatile u8 *) ioremap_nocache(CR_WRAP_TOP_BASE_ADDRESS, CR_WRAP_TOP_BASE_LENGTH);

    RESET_DEVICE(REG_R2D_AXI_RST_N, 0);
    RESET_DEVICE(REG_V2D_AXI_RST_N, 0);
    return gcvSTATUS_OK;
}
#else
#define MAX_CLK_SOURCES 3
static struct clk_bulk_data clks[MAX_CLK_SOURCES] = {
        {"gpu2_r2d_clk",NULL}, {"gpu2_r2d_pclk", NULL}, {"gpu2_v2d_clk", NULL}
};

static struct reset_control *rst_ctrls = NULL;
static gceSTATUS _GetPower(IN gcsPLATFORM * Platform)
{
    struct device* dev = &Platform->device->dev;
    if( 0 != devm_clk_bulk_get(dev, MAX_CLK_SOURCES, clks)){
        printk("devm_clk_bulk_get failed!\n");
        return gcvSTATUS_INVALID_ARGUMENT;
    }
    if ( 0 != clk_bulk_prepare(MAX_CLK_SOURCES, clks)) {
        printk("clk_bulk_enable error!\n");
        return gcvSTATUS_INVALID_ARGUMENT;
    }
    if( 0 != clk_bulk_enable(MAX_CLK_SOURCES, clks)) {
        printk("clk_bulk_enable error!\n");
        return gcvSTATUS_INVALID_ARGUMENT;
    }

    rst_ctrls = devm_reset_control_array_get(dev, false, false);
    if (rst_ctrls == NULL || 0 != reset_control_deassert(rst_ctrls)) {
        printk("devm_reset_control_array_get error!\n");
        return gcvSTATUS_INVALID_ARGUMENT;
    }

    return gcvSTATUS_OK;
}

static gceSTATUS _PutPower(IN gcsPLATFORM * Platform)
{
    if (rst_ctrls) {
        clk_bulk_disable(MAX_CLK_SOURCES, clks);
        clk_bulk_unprepare(MAX_CLK_SOURCES, clks);
        clk_bulk_put(MAX_CLK_SOURCES, clks);
        reset_control_assert(rst_ctrls);
        rst_ctrls = NULL;
    }
    return gcvSTATUS_OK;
}
#endif /* CONFIG_OF */
static gceSTATUS _GetGPUPhysical(IN gcsPLATFORM * Platform, IN gctPHYS_ADDR_T CPUPhysical,
        OUT gctPHYS_ADDR_T *GPUPhysical)
{
    *GPUPhysical = CPUPhysical;
    return gcvSTATUS_OK;
}

int gckPLATFORM_Init(struct platform_driver *pdrv, struct _gcsPLATFORM **platform)
{
    *platform = (gcsPLATFORM *)&default_platform;
    (*platform)->driver = pdrv;

    return 0;
}

int gckPLATFORM_Terminate(struct _gcsPLATFORM *platform)
{
    platform->driver = gcvNULL;
    return 0;
}
static gceSTATUS  _ExternalCacheOperation(IN gcsPLATFORM *Platform, IN gceCACHEOPERATION Operation)
{
    return gcvSTATUS_OK;
}
