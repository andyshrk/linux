/****************************************************************************
*
*    Copyright 2012 - 2022 Vivante Corporation, Santa Clara, California.
*    All Rights Reserved.
*
*    Permission is hereby granted, free of charge, to any person obtaining
*    a copy of this software and associated documentation files (the
*    'Software'), to deal in the Software without restriction, including
*    without limitation the rights to use, copy, modify, merge, publish,
*    distribute, sub license, and/or sell copies of the Software, and to
*    permit persons to whom the Software is furnished to do so, subject
*    to the following conditions:
*
*    The above copyright notice and this permission notice (including the
*    next paragraph) shall be included in all copies or substantial
*    portions of the Software.
*
*    THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
*    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
*    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
*    IN NO EVENT SHALL VIVANTE AND/OR ITS SUPPLIERS BE LIABLE FOR ANY
*    CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
*    TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
*    SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
*****************************************************************************/

#ifndef _nano2D_feature_database_h_
#define _nano2D_feature_databse_h_

typedef struct
{
    /* Chip ID. */
    n2d_uint32_t    chip_id;
    n2d_uint32_t    chip_version;
    n2d_uint32_t    pid;
    n2d_uint32_t    cid;

    n2d_uint32_t    N2D_YUV420_OUTPUT:1;
    n2d_uint32_t    PE2D_LINEAR_YUV420_10BIT:1;
    n2d_uint32_t    PE2D_MAJOR_SUPER_TILE:1;
    n2d_uint32_t    G2D_DEC400EX:1;
    n2d_uint32_t    REG_AndroidOnly:1;
    n2d_uint32_t    REG_OnePass2DFilter:1;
    n2d_uint32_t    REG_DESupertile:1;
    n2d_uint32_t    REG_NewFeatures0:1;
    n2d_uint32_t    REG_DEEnhancements1:1;
    n2d_uint32_t    REG_Compression2D:1;
    n2d_uint32_t    REG_MultiSrcV2:1;
    n2d_uint32_t    RGB_PLANAR:1;
    n2d_uint32_t    REG_NoScaler:1;
    n2d_uint32_t    REG_DualPipeOPF:1;
    n2d_uint32_t    REG_SeperateSRCAndDstCache:1;
    n2d_uint32_t    N2D_FEATURE_AXI_FE:1;
    n2d_uint32_t    N2D_FEATURE_CSC_PROGRAMMABLE:1;
    n2d_uint32_t    N2D_FEATURE_DEC400_FC:1;
    n2d_uint32_t    N2D_FEATURE_MASK:1;
    n2d_uint32_t    N2D_FEATURE_COLORKEY:1;
    n2d_uint32_t    N2D_FEATURE_NORMALIZATION:1;
    n2d_uint32_t    N2D_FEATURE_NORMALIZATION_QUANTIZATION:1;
    n2d_uint32_t    N2D_FEATURE_HISTOGRAM:1;
    n2d_uint32_t    N2D_FEATURE_BRIGHTNESS_SATURATION:1;
    n2d_uint32_t    N2D_FEATURE_64BIT_ADDRESS:1;
    n2d_uint32_t    N2D_FEATURE_CONTEXT_ID:1;
    n2d_uint32_t    N2D_FEATURE_SECURE_BUFFER:1;

    /*for kernel*/
    n2d_uint32_t    N2D_FEATURE_MMU_PAGE_DESCRIPTOR:1;
    n2d_uint32_t    N2D_FEATURE_SECURITY_AHB:1;
    n2d_uint32_t    N2D_FEATURE_FRAME_DONE_INTR:1;
}
n2d_feature_database;

static n2d_feature_database n2d_chip_features[] = {
    /*gc300 0x4650 rc1b*/
    {
        0x300,     /*chip_id*/
        0x4650,    /*chip_version*/
        0x5203,    /*pid*/
        0x0,       /*cid*/
        0,         /*N2D_YUV420_OUTPUT*/
        0,         /*PE2D_LINEAR_YUV420_10BIT*/
        0,         /*PE2D_MAJOR_SUPER_TILE*/
        0,         /*G2D_DEC400EX*/
        0,         /*android only*/
        0,         /*OnePassFilter*/
        1,         /*DE supertile*/
        0,         /*REG_NewFeatures0*/
        0,         /*REG_DEEnhancements1*/
        0,         /*Compression2D*/
        0,         /*REG_MultiSrcV2*/
        0,         /*BGR_PLANAR*/
        0,         /*REG_NoScaler*/
        0,         /*DualPipeOPF*/
        0,         /*REG_SeperateSRCAndDstCache*/
        0,         /*AXI-FE*/
        0,         /*CSC*/
        0,         /*FC*/
        1,         /*Mask*/
        1,         /*ColorKey*/
        0,         /*Normalization*/
        0,         /*Quantization*/
        0,         /*Histogram*/
        0,         /*Brightness_Saturation*/
        0,         /*64BitGpuAddress*/
        0,         /*contextID*/
        0,         /*securebuffer*/
        /*kernel*/
        0,         /*MMU PD MODE*/
        0,         /*SECURITY_AHB*/
        0,         /*FRAME_DONE_INTR*/
    },
    /*gc320l 0x534 rc1b*/
    {
        0x320,      /*chip_id*/
        0x5341,     /*chip_version*/
        0x3202,     /*pid*/
        0x0,        /*cid*/
        0,          /*N2D_YUV420_OUTPUT*/
        0,          /*PE2D_LINEAR_YUV420_10BIT*/
        0,          /*PE2D_MAJOR_SUPER_TILE*/
        0,          /*G2D_DEC400EX*/
        0,          /*android only*/
        1,          /*OnePassFilter*/
        0,          /*DE supertile*/
        1,          /*REG_NewFeatures0*/
        0,          /*REG_DEEnhancements1*/
        0,          /*Compression2D*/
        0,          /*REG_MultiSrcV2*/
        0,          /*BGR_PLANAR */
        0,          /*REG_NoScaler*/
        0,          /*DualPipeOPF*/
        0,          /*REG_SeperateSRCAndDstCache*/
        0,          /*AXI-FE*/
        0,          /*CSC*/
        0,          /*FC*/
        1,          /*Mask*/
        1,          /*ColorKey*/
        0,          /*Normalization*/
        0,          /*Quantization*/
        0,         /*Histogram*/
        0,         /*Brightness_Saturation*/
        0,         /*64BitGpuAddress*/
        0,         /*contextID*/
        0,         /*securebuffer*/
        /*kernel*/
        0,         /*MMU PD MODE*/
        0,         /*SECURITY_AHB*/
        0,         /*FRAME_DONE_INTR*/
    },
    /*gc520l 0x534 rc1i*/
    {
        0x520,     /*chip_id*/
        0x5341,    /*chip_version*/
        0x5202,    /*pid*/
        0x204,     /*cid*/
        0,         /*N2D_YUV420_OUTPUT*/
        0,         /*PE2D_LINEAR_YUV420_10BIT*/
        0,         /*PE2D_MAJOR_SUPER_TILE*/
        0,         /*G2D_DEC400EX*/
        0,         /*android only*/
        1,         /*OnePassFilter*/
        0,         /*DE supertile*/
        1,         /*REG_NewFeatures0*/
        0,         /*REG_DEEnhancements1*/
        0,         /*Compression2D*/
        0,         /*REG_MultiSrcV2*/
        0,         /*BGR_PLANAR */
        0,         /*REG_NoScaler*/
        0,         /*DualPipeOPF*/
        0,         /*REG_SeperateSRCAndDstCache*/
        0,         /*AXI-FE*/
        0,         /*CSC*/
        0,         /*FC*/
        1,         /*Mask*/
        1,         /*ColorKey*/
        0,         /*Normalization*/
        0,         /*Quantization*/
        0,         /*Histogram*/
        0,         /*Brightness_Saturation*/
        0,         /*64BitGpuAddress*/
        0,         /*contextID*/
        0,         /*securebuffer*/
        /*kernel*/
        0,         /*MMU PD MODE*/
        0,         /*SECURITY_AHB*/
        0,         /*FRAME_DONE_INTR*/
    },
    /*gc620s 0x5551 rc1b*/
    {
        0x620,     /*chip_id*/
        0x5551,    /*chip_version*/
        0x6200,    /*pid*/
        0x205,     /*cid*/
        1,         /*N2D_YUV420_OUTPUT*/
        1,         /*PE2D_LINEAR_YUV420_10BIT*/
        0,         /*PE2D_MAJOR_SUPER_TILE*/
        1,         /*G2D_DEC400EX*/
        1,         /*android only*/
        1,         /*OnePassFilter*/
        1,         /*DE supertile*/
        1,         /*REG_NewFeatures0*/
        1,         /*REG_DEEnhancements1*/
        0,         /*Compression2D*/
        1,         /*REG_MultiSrcV2*/
        0,         /*BGR_PLANAR*/
        1,         /*REG_NoScaler*/
        1,         /*DualPipeOPF*/
        1,         /*REG_SeperateSRCAndDstCache*/
        0,         /*AXI-FE*/
        0,         /*CSC*/
        0,         /*FC*/
        0,         /*Mask*/
        0,         /*ColorKey*/
        0,         /*Normalization*/
        0,         /*Quantization*/
        0,         /*Histogram*/
        0,         /*Brightness_Saturation*/
        0,         /*64BitGpuAddress*/
        0,         /*contextID*/
        0,         /*securebuffer*/
        /*kernel*/
        0,         /*MMU PD MODE*/
        1,         /*SECURITY_AHB*/
        0,         /*FRAME_DONE_INTR*/
    },
    /*gc620s 0x5552 rc1b*/
    {
        0x620,     /*chip_id*/
        0x5552,    /*chip_version*/
        0x6200,    /*pid*/
        0x205,     /*cid*/
        1,         /*N2D_YUV420_OUTPUT*/
        1,         /*PE2D_LINEAR_YUV420_10BIT*/
        0,         /*PE2D_MAJOR_SUPER_TILE*/
        1,         /*G2D_DEC400EX*/
        1,         /*android only*/
        1,         /*OnePassFilter*/
        1,         /*DE supertile*/
        1,         /*REG_NewFeatures0*/
        1,         /*REG_DEEnhancements1*/
        0,         /*Compression2D*/
        1,         /*REG_MultiSrcV2*/
        1,         /*BGR_PLANAR*/
        1,         /*REG_NoScaler*/
        1,         /*DualPipeOPF*/
        1,         /*REG_SeperateSRCAndDstCache*/
        0,         /*AXI-FE*/
        0,         /*CSC*/
        0,         /*FC*/
        0,         /*Mask*/
        0,         /*ColorKey*/
        0,         /*Normalization*/
        0,         /*Quantization*/
        0,         /*Histogram*/
        0,         /*Brightness_Saturation*/
        0,         /*64BitGpuAddress*/
        0,         /*contextID*/
        0,         /*securebuffer*/
        /*kernel*/
        0,         /*MMU PD MODE*/
        1,         /*SECURITY_AHB*/
        0,         /*FRAME_DONE_INTR*/
    },
    /*gc620s 0x5552 rc1b*/
    {
        0x620,     /*chip_id*/
        0x5552,    /*chip_version*/
        0x6200,    /*pid*/
        0x207,     /*cid*/
        1,         /*N2D_YUV420_OUTPUT*/
        1,         /*PE2D_LINEAR_YUV420_10BIT*/
        0,         /*PE2D_MAJOR_SUPER_TILE*/
        0,         /*G2D_DEC400EX*/
        1,         /*android only*/
        1,         /*OnePassFilter*/
        1,         /*DE supertile*/
        1,         /*REG_NewFeatures0*/
        1,         /*REG_DEEnhancements1*/
        0,         /*Compression2D*/
        1,         /*REG_MultiSrcV2*/
        0,         /*RGB_PLANAR */
        1,         /*REG_NoScaler*/
        1,         /*DualPipeOPF*/
        1,         /*REG_SeperateSRCAndDstCache*/
        0,         /*AXI-FE*/
        0,         /*CSC*/
        0,         /*FC*/
        0,         /*Mask*/
        0,         /*ColorKey*/
        0,         /*Normalization*/
        0,         /*Quantization*/
        0,         /*Histogram*/
        0,         /*Brightness_Saturation*/
        0,         /*64BitGpuAddress*/
        0,         /*contextID*/
        0,         /*securebuffer*/
        /*kernel*/
        0,         /*MMU PD MODE*/
        0,         /*SECURITY_AHB*/
        0,         /*FRAME_DONE_INTR*/
    },
    /*gc620s 0x5552 rc2p*/
    {
        0x620,     /*chip_id*/
        0x5552,    /*chip_version*/
        0x6200,    /*pid*/
        0x20B,     /*cid*/
        1,         /*N2D_YUV420_OUTPUT*/
        1,         /*PE2D_LINEAR_YUV420_10BIT*/
        1,         /*PE2D_MAJOR_SUPER_TILE*/
        1,         /*G2D_DEC400EX*/
        1,         /*android only*/
        1,         /*OnePassFilter*/
        1,         /*DE supertile*/
        1,         /*REG_NewFeatures0*/
        1,         /*REG_DEEnhancements1*/
        0,         /*Compression2D*/
        1,         /*REG_MultiSrcV2*/
        1,         /*RGB_PLANAR */
        1,         /*REG_NoScaler*/
        1,         /*DualPipeOPF*/
        1,         /*REG_SeperateSRCAndDstCache*/
        0,         /*AXI-FE*/
        1,         /*CSC*/
        0,         /*FC*/
        0,         /*Mask*/
        0,         /*ColorKey*/
        0,         /*Normalization*/
        0,         /*Quantization*/
        0,         /*Histogram*/
        0,         /*Brightness_Saturation*/
        0,         /*64BitGpuAddress*/
        0,         /*contextID*/
        0,         /*securebuffer*/
        /*kernel*/
        0,         /*MMU PD MODE*/
        1,         /*SECURITY_AHB*/
        0,         /*FRAME_DONE_INTR*/
    },
    /*gc620s 0x5552 rc4k*/
    {
        0x620,     /*chip_id*/
        0x5554,    /*chip_version*/
        0x6200,    /*pid*/
        0x20D,     /*cid*/
        1,         /*N2D_YUV420_OUTPUT*/
        1,         /*PE2D_LINEAR_YUV420_10BIT*/
        0,         /*PE2D_MAJOR_SUPER_TILE*/
        1,         /*G2D_DEC400EX*/
        1,         /*android only*/
        1,         /*OnePassFilter*/
        1,         /*DE supertile*/
        1,         /*REG_NewFeatures0*/
        1,         /*REG_DEEnhancements1*/
        0,         /*Compression2D*/
        1,         /*REG_MultiSrcV2*/
        1,         /*RGB_PLANAR */
        1,         /*REG_NoScaler*/
        1,         /*DualPipeOPF*/
        1,         /*REG_SeperateSRCAndDstCache*/
        0,         /*AXI-FE*/
        0,         /*CSC*/
        1,         /*FC*/
        0,         /*Mask*/
        0,         /*ColorKey*/
        0,         /*Normalization*/
        0,         /*Quantization*/
        0,         /*Histogram*/
        0,         /*Brightness_Saturation*/
        0,         /*64BitGpuAddress*/
        0,         /*contextID*/
        0,         /*securebuffer*/
        /*kernel*/
        1,         /*MMU PD MODE*/
        1,         /*SECURITY_AHB*/
        0,         /*FRAME_DONE_INTR*/
    },
    /*gc620s 0x5553 rc3h*/
    {
        0x620,     /*chip_id*/
        0x5553,    /*chip_version*/
        0x6200,    /*pid*/
        0x210,     /*cid*/
        1,         /*N2D_YUV420_OUTPUT*/
        1,         /*PE2D_LINEAR_YUV420_10BIT*/
        0,         /*PE2D_MAJOR_SUPER_TILE*/
        0,         /*G2D_DEC400EX*/
        1,         /*android only*/
        1,         /*OnePassFilter*/
        1,         /*DE supertile*/
        1,         /*REG_NewFeatures0*/
        1,         /*REG_DEEnhancements1*/
        0,         /*Compression2D*/
        1,         /*REG_MultiSrcV2*/
        1,         /*RGB_PLANAR */
        1,         /*REG_NoScaler*/
        1,         /*DualPipeOPF*/
        1,         /*REG_SeperateSRCAndDstCache*/
        1,         /*AXI-FE*/
        0,         /*CSC*/
        0,         /*FC*/
        0,         /*Mask*/
        0,         /*ColorKey*/
        0,         /*Normalization*/
        0,         /*Quantization*/
        0,         /*Histogram*/
        0,         /*Brightness_Saturation*/
        0,         /*64BitGpuAddress*/
        0,         /*contextID*/
        0,         /*securebuffer*/
        /*kernel*/
        0,         /*MMU PD MODE*/
        0,         /*SECURITY_AHB*/
        0,         /*FRAME_DONE_INTR*/
    },
    /*gc620s 0x5554 rc4b*/
    {
        0x620,     /*chip_id*/
        0x5554,    /*chip_version*/
        0x6200,    /*pid*/
        0x210,     /*cid*/
        1,         /*N2D_YUV420_OUTPUT*/
        1,         /*PE2D_LINEAR_YUV420_10BIT*/
        0,         /*PE2D_MAJOR_SUPER_TILE*/
        0,         /*G2D_DEC400EX*/
        1,         /*android only*/
        1,         /*OnePassFilter*/
        1,         /*DE supertile*/
        1,         /*REG_NewFeatures0*/
        1,         /*REG_DEEnhancements1*/
        0,         /*Compression2D*/
        1,         /*REG_MultiSrcV2*/
        1,         /*RGB_PLANAR*/
        1,         /*REG_NoScaler*/
        1,         /*DualPipeOPF*/
        1,         /*REG_SeperateSRCAndDstCache*/
        1,         /*AXI-FE*/
        0,         /*CSC*/
        0,         /*FC*/
        0,         /*Mask*/
        0,         /*ColorKey*/
        0,         /*Normalization*/
        0,         /*Quantization*/
        0,         /*Histogram*/
        0,         /*Brightness_Saturation*/
        0,         /*64BitGpuAddress*/
        0,         /*contextID*/
        0,         /*securebuffer*/
        /*kernel*/
        0,         /*MMU PD MODE*/
        0,         /*SECURITY_AHB*/
        0,         /*FRAME_DONE_INTR*/
    },
    /*gc620s 0x5554 rc4f*/
    {
        0x620,     /*chip_id*/
        0x5554,    /*chip_version*/
        0x6200,    /*pid*/
        0x214,     /*cid*/
        1,         /*N2D_YUV420_OUTPUT*/
        1,         /*PE2D_LINEAR_YUV420_10BIT*/
        0,         /*PE2D_MAJOR_SUPER_TILE*/
        1,         /*G2D_DEC400EX*/
        1,         /*android only*/
        1,         /*OnePassFilter*/
        1,         /*DE supertile*/
        1,         /*REG_NewFeatures0*/
        1,         /*REG_DEEnhancements1*/
        0,         /*Compression2D*/
        1,         /*REG_MultiSrcV2*/
        1,         /*RGB_PLANAR */
        1,         /*REG_NoScaler*/
        1,         /*DualPipeOPF*/
        1,         /*REG_SeperateSRCAndDstCache*/
        0,         /*AXI-FE*/
        1,         /*CSC*/
        1,         /*FC*/
        1,         /*Mask*/
        1,         /*ColorKey*/
        0,         /*Normalization*/
        0,         /*Quantization*/
        0,         /*Histogram*/
        0,         /*Brightness_Saturation*/
        0,         /*64BitGpuAddress*/
        0,         /*contextID*/
        0,         /*securebuffer*/
        /*kernel*/
        1,         /*MMU PD MODE*/
        1,         /*SECURITY_AHB*/
        0,         /*FRAME_DONE_INTR*/
    },
    /*gc820H 0x5630 rc0e*/
    {
        0x820,     /*chip_id*/
        0x5630,    /*chip_version*/
        0x8200,    /*pid*/
        0x215,     /*cid*/
        1,         /*N2D_YUV420_OUTPUT*/
        1,         /*PE2D_LINEAR_YUV420_10BIT*/
        0,         /*PE2D_MAJOR_SUPER_TILE*/
        0,         /*G2D_DEC400EX*/
        1,         /*android only*/
        1,         /*OnePassFilter*/
        1,         /*DE supertile*/
        1,         /*REG_NewFeatures0*/
        1,         /*REG_DEEnhancements1*/
        0,         /*Compression2D*/
        1,         /*REG_MultiSrcV2*/
        1,         /*RGB_PLANAR */
        1,         /*REG_NoScaler*/
        1,         /*DualPipeOPF*/
        1,         /*REG_SeperateSRCAndDstCache*/
        1,         /*AXI-FE*/
        1,         /*CSC*/
        0,         /*FC*/
        1,         /*Mask*/
        1,         /*ColorKey*/
        1,         /*Normalization*/
        1,         /*Quantization*/
        0,         /*Histogram*/
        0,         /*Brightness_Saturation*/
        0,         /*64BitGpuAddress*/
        0,         /*contextID*/
        0,         /*securebuffer*/
        /*kernel*/
        0,         /*MMU PD MODE*/
        0,         /*SECURITY_AHB*/
        1,         /*FRAME_DONE_INTR*/
    },
    /*gc820 0x5630 rc1r*/
    {
        0x820,     /*chip_id*/
        0x5631,    /*chip_version*/
        0x8200,    /*pid*/
        0x216,     /*cid*/
        1,         /*N2D_YUV420_OUTPUT*/
        1,         /*PE2D_LINEAR_YUV420_10BIT*/
        0,         /*PE2D_MAJOR_SUPER_TILE*/
        1,         /*G2D_DEC400EX*/
        1,         /*android only*/
        1,         /*OnePassFilter*/
        1,         /*DE supertile*/
        1,         /*REG_NewFeatures0*/
        1,         /*REG_DEEnhancements1*/
        0,         /*Compression2D*/
        1,         /*REG_MultiSrcV2*/
        1,         /*RGB_PLANAR */
        1,         /*REG_NoScaler*/
        1,         /*DualPipeOPF*/
        1,         /*REG_SeperateSRCAndDstCache*/
        1,         /*AXI-FE*/
        1,         /*CSC*/
        1,         /*FC*/
        0,         /*Mask*/
        0,         /*ColorKey*/
        1,         /*Normalization*/
        1,         /*Quantization*/
        0,         /*Histogram*/
        0,         /*Brightness_Saturation*/
        0,         /*64BitGpuAddress*/
        0,         /*contextID*/
        1,         /*securebuffer*/
        /*kernel*/
        0,         /*MMU PD MODE*/
        0,         /*SECURITY_AHB*/
        0,         /*FRAME_DONE_INTR*/
    },
    /*gc820H 0x5630 rc1w*/
    {
        0x820,     /*chip_id*/
        0x5631,    /*chip_version*/
        0x8200,    /*pid*/
        0x217,     /*cid*/
        1,         /*N2D_YUV420_OUTPUT*/
        1,         /*PE2D_LINEAR_YUV420_10BIT*/
        0,         /*PE2D_MAJOR_SUPER_TILE*/
        1,         /*G2D_DEC400EX*/
        1,         /*android only*/
        1,         /*OnePassFilter*/
        1,         /*DE supertile*/
        1,         /*REG_NewFeatures0*/
        1,         /*REG_DEEnhancements1*/
        0,         /*Compression2D*/
        1,         /*REG_MultiSrcV2*/
        1,         /*RGB_PLANAR */
        1,         /*REG_NoScaler*/
        1,         /*DualPipeOPF*/
        1,         /*REG_SeperateSRCAndDstCache*/
        0,         /*AXI-FE*/
        1,         /*CSC*/
        1,         /*FC*/
        0,         /*Mask*/
        0,         /*ColorKey*/
        1,         /*Normalization*/
        1,         /*Quantization*/
        1,         /*Histogram*/
        1,         /*Brightness_Saturation*/
        1,         /*64BitGpuAddress*/
        1,         /*contextID*/
        0,         /*securebuffer*/
        /*kernel*/
        0,         /*MMU PD MODE*/
        0,         /*SECURITY_AHB*/
        1,         /*FRAME_DONE_INTR*/
    },
    /*gc820 0x5630 rc0c*/
    {
        0x820,     /*chip_id*/
        0x5631,    /*chip_version*/
        0x8200,    /*pid*/
        0x218,     /*cid*/
        1,         /*N2D_YUV420_OUTPUT*/
        1,         /*PE2D_LINEAR_YUV420_10BIT*/
        0,         /*PE2D_MAJOR_SUPER_TILE*/
        0,         /*G2D_DEC400EX*/
        1,         /*android only*/
        1,         /*OnePassFilter*/
        1,         /*DE supertile*/
        1,         /*REG_NewFeatures0*/
        1,         /*REG_DEEnhancements1*/
        0,         /*Compression2D*/
        1,         /*REG_MultiSrcV2*/
        1,         /*RGB_PLANAR */
        1,         /*REG_NoScaler*/
        1,         /*DualPipeOPF*/
        1,         /*REG_SeperateSRCAndDstCache*/
        0,         /*AXI-FE*/
        1,         /*CSC*/
        0,         /*FC*/
        0,         /*Mask*/
        0,         /*ColorKey*/
        0,         /*Normalization*/
        0,         /*Quantization*/
        0,         /*Histogram*/
        0,         /*Brightness_Saturation*/
        0,         /*64BitGpuAddress*/
        0,         /*contextID*/
        0,         /*securebuffer*/
        /*kernel*/
        0,         /*MMU PD MODE*/
        0,         /*SECURITY_AHB*/
        0,         /*FRAME_DONE_INTR*/
    },
};

static n2d_feature_database * query_features(
    n2d_uint32_t chip_id,
    n2d_uint32_t chip_version,
    n2d_uint32_t product_id,
    n2d_uint32_t cid
    )
{
    n2d_uint32_t size = sizeof(n2d_chip_features) / sizeof(n2d_chip_features[0]);
    n2d_uint32_t i;
    /* check formal release entries */
    for (i = 0; i < size; ++i)
    {

        if ((n2d_chip_features[i].chip_id == chip_id)
            && (n2d_chip_features[i].chip_version == chip_version)
            && (n2d_chip_features[i].pid == product_id)
            && (n2d_chip_features[i].cid == cid)
           )
        {
            return &n2d_chip_features[i];
        }
    }
    return N2D_NULL;
}

#endif /* _nano2D_feature_database_h_ */
