# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Siengine VPU DEC drivers
#
ccflags-$(CONFIG_DEC_SFBC_SUPPORT)	+= -DSUPPORT_SFBC
ccflags-$(CONFIG_DEC_DEVMGR_SUPPORT)	+= -DSUPPORT_DEVMGR

hantrodec-objs				+= dec_devicemanager.o hantro_dec.o
obj-$(CONFIG_HANTRO_VC8000D)		+= hantrodec.o


