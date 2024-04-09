# Makefile for the Himax touchscreen drivers.

ifneq ($(filter m, $(CONFIG_TOUCHSCREEN_HIMAX_COMMON)),)
  ccflags-y += -D__HIMAX_MOD__
endif

ifneq ($(filter y m, $(CONFIG_TOUCHSCREEN_HIMAX_INCELL)),)
	himax_mmi-objs += himax_ic_incell_core.o
endif

ifneq ($(filter y m, $(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83193)),)
  ccflags-y += -D__HIMAX_HX83193_MOD__
	himax_mmi-objs += himax_ic_HX83193.o
endif

ifneq ($(filter m, $(CONFIG_TOUCHSCREEN_HIMAX_COMMON)),)
  ccflags-y += -D__HIMAX_MOD__
  himax_mmi-objs += himax_common.o
  himax_mmi-objs += himax_platform.o
  obj-m += himax_mmi.o
endif

ifneq ($(filter y, $(CONFIG_TOUCHSCREEN_HIMAX_COMMON)),)
  himax_mmi-objs += himax_common.o
  himax_mmi-objs += himax_platform.o
  obj-y += himax_mmi.o
endif

