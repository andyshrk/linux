# SPDX-License-Identifier: GPL-2.0
#
# Makefile for npu1 devices
#

ccflags-y := -DCONFIG_ARMCHINA_NPU_ARCH_V2
ccflags-y += -DKMD_VERSION=\"r1p3\"

ccflags-y += -I$(srctree)/drivers/npu
ccflags-y += -I$(srctree)/drivers/npu/include
ccflags-y += -I$(srctree)/drivers/npu/zhouyi

aipudrv-y += aipu_irq.o
aipudrv-y += aipu_io.o
aipudrv-y += aipu_mm.o
aipudrv-y += aipu_priv.o
aipudrv-y += aipu_job_manager.o
aipudrv-y += aipu.o
aipudrv-y += aipu_common.o
aipudrv-y += aipu_dma_buf.o
aipudrv-y += aipu_tcb.o
aipudrv-y += npu_drv.o

include $(src)/zhouyi/Makefile

obj-$(CONFIG_NPU1_ZHOUYI) = aipudrv.o
#ccflags-y +=-DDEBUG
