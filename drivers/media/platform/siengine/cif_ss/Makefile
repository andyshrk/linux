# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Siengine CIF driver
#
obj-$(CONFIG_MIPI_CSI2) += csi.o
obj-$(CONFIG_SE_CIF) += cif.o
obj-$(CONFIG_SE_CIF_DEBUG) += se-cif-sysfs.o
obj-$(CONFIG_SE_MEDIA_DEV) += media_dev.o
obj-$(CONFIG_SIENGINE_DWC_MIPI_DPHY_RX) += dw-dphy-plat.o dw-dphy-rx.o
csi-objs += dw-csi-plat.o dw-mipi-csi.o cisp-devfreq-dev.o
cif-objs += se-cif-core.o se-cif-hw.o se-cif-cap.o se-nr-hw.o
media_dev-objs += se-media-dev.o
