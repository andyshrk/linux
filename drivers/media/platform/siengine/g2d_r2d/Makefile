OBJECTS += nano2D_kernel.o
OBJECTS += nano2D_kernel_hardware.o
OBJECTS += nano2D_kernel_mmu.o
OBJECTS += linux/nano2D_kernel_os.o
OBJECTS += linux/nano2D_kernel_driver.o
OBJECTS += linux/nano2D_kernel_debugfs.o
OBJECTS += linux/nano2D_kernel_iommu.o
OBJECTS += linux/platform/se1000.o
OBJECTS += nano2D_kernel_vidmem.o
OBJECTS += nano2D_kernel_common.o
OBJECTS += nano2D_kernel_db.o
OBJECTS += nano2D_kernel_event.o

ifeq ($(KERNELRELEASE),)

ARCH ?= arm

PWD := $(shell pwd)

all:$(OUTDIR)
	@mkdir -p out
	@$(MAKE) -C $(KERNEL_DIR) M=$(PWD) ARCH=$(ARCH) V=$(V) modules
	@cp nano2d.ko out

clean:
	@rm -rf $(OBJECTS) *.o nano2d.ko nano2d.mod.c .tmp_versions out $(OUTDIR)/nano2d.ko
	@rm -rf modules.order Module.symvers
	@find $(AQROOT) -name ".*.cmd" | xargs rm -f

install:all
	@cp out/nano2d.ko $(OUTDIR)

$(OUTDIR):
	@mkdir -p $(OUTDIR)

.PHONY: modules clean install

else

obj-y := nano2d.o
nano2d-objs := $(OBJECTS)

ccflags-y += -Idrivers/media/platform/siengine/g2d_r2d
ccflags-y += -Idrivers/media/platform/siengine/g2d_r2d/inc
ccflags-y += -Idrivers/media/platform/siengine/g2d_r2d/linux
ccflags-y += -Idrivers/media/platform/siengine/g2d_r2d/inc/shared

ifneq ($(wildcard $(AQROOT)/nano2D/nano2D_option.config),)
    -include $(AQROOT)/nano2D/nano2D_option.config
EXTRA_CFLAGS += ${PLATFORM_FLAGS}
endif

ifeq ($(USE_LINUX_PCIE), 1)
ccflags-y += -DUSE_LINUX_PCIE=1
else
ccflags-y += -DUSE_LINUX_PCIE=0
endif

ifeq ($(USE_ARM_VDK), 1)
ccflags-y += -DUSE_ARM_VDK=1
else
ccflags-y += -DUSE_ARM_VDK=0
endif

ccflags-y += -DUSE_ARCH64=1

#ccflags-y += -DNANO2D_MMU_ENABLE=1

endif
