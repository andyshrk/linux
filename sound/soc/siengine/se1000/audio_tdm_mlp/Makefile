#
# Copyright (C) 2022 Siengine Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

# audio dpcm

# dummy codec
obj-$(CONFIG_SND_SOC_SE1000_TDM_MLP) += tdm_dummy_codec.o

# platform driver
snd-soc-se1000-multi-tdm-objs := se1000-tdm-fe.o se1000-tdm-be.o se1000-dma.o
obj-$(CONFIG_SND_SOC_SE1000_TDM_MLP) += snd-soc-se1000-multi-tdm.o

obj-$(CONFIG_SND_SOC_SE1000_TDM_MLP) += se1000-tdm-card.o