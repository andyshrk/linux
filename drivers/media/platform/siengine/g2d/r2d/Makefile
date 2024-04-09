# Project Options

DEBUG := 1
VIVANTE_ENABLE_2D := 1
VIVANTE_ENABLE_VG := 0
soc_vendor := default
soc_board := default

# Module name
MODULE_NAME := r2dcore
obj-$(CONFIG_SIENGINE_G2D_R2D) += $(MODULE_NAME).o

# Source files tree
OS_KERNEL_DIR   := ../os
ARCH_KERNEL_DIR := ../arch
ARCH_VG_KERNEL_DIR := ../archvg
HAL_KERNEL_DIR  := ..
TA_DIR          := ../security_v1

ccflags-y		:= -I$(src)/$(HAL_KERNEL_DIR)/inc -I$(src)/$(HAL_KERNEL_DIR) \
					-I$(src)/$(ARCH_KERNEL_DIR) \
					-I$(src)/$(OS_KERNEL_DIR) -I$(src)/$(OS_KERNEL_DIR)/allocator/default \
					-I$(src)/$(TA_DIR)

ALLOCATOR_ARRAY_H_LOCATION ?= $(OS_KERNEL_DIR)/allocator/default/
SOC_PLATFORM_LOCATION      ?= $(OS_KERNEL_DIR)/platform/$(soc_vendor)

# Set common platform driver source
SOC_PLATFORM_OBJS ?= $(SOC_PLATFORM_LOCATION)/gc_hal_kernel_platform_$(soc_board).o

COMMON_OBJS := $(OS_KERNEL_DIR)/gc_hal_kernel_device.o \
        $(OS_KERNEL_DIR)/gc_hal_kernel_linux.o \
        $(OS_KERNEL_DIR)/gc_hal_kernel_math.o \
        $(OS_KERNEL_DIR)/gc_hal_kernel_os.o \
        $(OS_KERNEL_DIR)/gc_hal_kernel_debugfs.o \
        $(OS_KERNEL_DIR)/gc_hal_kernel_allocator.o \
        $(OS_KERNEL_DIR)/allocator/default/gc_hal_kernel_allocator_user_memory.o \
        $(OS_KERNEL_DIR)/allocator/default/gc_hal_kernel_allocator_dma.o \
        $(OS_KERNEL_DIR)/allocator/default/gc_hal_kernel_allocator_gfp.o \
        $(OS_KERNEL_DIR)/allocator/default/gc_hal_kernel_allocator_reserved_mem.o \
        $(OS_KERNEL_DIR)/gc_hal_kernel_driver.o

COMMON_OBJS += $(SOC_PLATFORM_OBJS)

COMMON_OBJS += $(HAL_KERNEL_DIR)/gc_hal_kernel.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel_command.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel_db.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel_debug.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel_event.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel_heap.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel_mmu.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel_video_memory.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel_power.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel_security_v1.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel_preemption.o

COMMON_OBJS += $(ARCH_KERNEL_DIR)/gc_hal_kernel_context.o \
        $(ARCH_KERNEL_DIR)/gc_hal_kernel_hardware.o \
        $(ARCH_KERNEL_DIR)/gc_hal_kernel_hardware_func.o \
        $(ARCH_KERNEL_DIR)/gc_hal_kernel_hardware_func_flop_reset.o \
        $(ARCH_KERNEL_DIR)/gc_hal_kernel_hardware_async_fe.o \
        $(ARCH_KERNEL_DIR)/gc_hal_kernel_hardware_mc_fe.o \
        $(ARCH_KERNEL_DIR)/gc_hal_kernel_hardware_waitlink_fe.o

COMMON_OBJS += $(OS_KERNEL_DIR)/gc_hal_kernel_security_channel_emulator.o \
        $(TA_DIR)/gc_hal_ta.o \
        $(TA_DIR)/gc_hal_ta_hardware.o \
        $(TA_DIR)/gc_hal_ta_mmu.o \
        $(TA_DIR)/os/emulator/gc_hal_ta_emulator.o

$(MODULE_NAME)-y := $(COMMON_OBJS)

ccflags-y$(CONFIG_KASAN) += -Wno-implicit-fallthrough

$(MODULE_NAME)-$(CONFIG_DMA_SHARED_BUFFER) += $(OS_KERNEL_DIR)/allocator/default/gc_hal_kernel_allocator_dmabuf.o

$(MODULE_NAME)-$(CONFIG_IOMMU_SUPPORT) += $(OS_KERNEL_DIR)/gc_hal_kernel_iommu.o

$(MODULE_NAME)-$(CONFIG_DRM) += $(OS_KERNEL_DIR)/gc_hal_kernel_drm.o

ccflags-$(CONFIG_SYNC_FILE) += -DgcdLINUX_SYNC_FILE=1
$(MODULE_NAME)-$(CONFIG_SYNC_FILE) += $(OS_KERNEL_DIR)/gc_hal_kernel_sync.o

ccflags-y   += -DLINUX -DDRIVER

ifeq ($(DEBUG), 1)
ccflags-y   += -DDBG=1 -DDEBUG -D_DEBUG
else
ccflags-y   += -DDBG=0
endif

ccflags-y  += -DNO_DMA_COHERENT

ccflags-y += -DUSE_PLATFORM_DRIVER=1

ccflags-y += -DVIVANTE_PROFILER=1
ccflags-y += -DVIVANTE_PROFILER_CONTEXT=1

ccflags-y += -DENABLE_GPU_CLOCK_BY_DRIVER=0
ccflags-y += -DUSE_NEW_LINUX_SIGNAL=0
ccflags-y += -DUSE_LINUX_PCIE=0

ccflags-y += -DgcdCACHE_FUNCTION_UNIMPLEMENTED=0
ccflags-y += -DgcdENABLE_3D=0

ifeq ($(VIVANTE_ENABLE_2D),0)
ccflags-y += -DgcdENABLE_2D=0
else
ccflags-y += -DgcdENABLE_2D=1
endif

ifeq ($(VIVANTE_ENABLE_VG),0)
ccflags-y += -DgcdENABLE_VG=0
else
ccflags-y += -DgcdENABLE_VG=1
ccflags-y += -I$(src)/$(ARCH_VG_KERNEL_DIR)
endif

ccflags-y  += -DgcdFPGA_BUILD=0
ccflags-$(CONFIG_DRM) += -DgcdENABLE_DRM=0
ccflags-$(CONFIG_ARM) += -Iarch/arm/mm

HOST := $(shell hostname)
ccflags-y += -DHOST=\"$(HOST)\" 
ccflags-y += -DgcdENABLE_TRUST_APPLICATION=1

