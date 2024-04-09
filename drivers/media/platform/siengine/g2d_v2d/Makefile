MODULE_NAME := vg_lite
obj-$(CONFIG_SIENGINE_G2D_V2D) += $(MODULE_NAME).o
$(MODULE_NAME)-y := linux/vg_lite_hal.o vg_lite_kernel.o
ccflags-y += -O2 -DEMULATOR=0 -I$(src)/inc -I$(src)/linux -I$(src)
