#
# Copyright (C) 2020 Siengine Inc.
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

# dummy codec
obj-$(CONFIG_SND_SOC_SE1000) += dummy_codec.o

# platform driver
snd-soc-se1000-platform-objs := se1000-aud-i2s.o se1000-platform.o se1000-generic-dmaengine-pcm.o
snd-soc-se1000-spdif-objs := se1000-spdif.o
obj-$(CONFIG_SND_SOC_SE1000) += snd-soc-se1000-platform.o
obj-$(CONFIG_SND_SOC_SE1000) += snd-soc-se1000-spdif.o

# machine driver
obj-$(CONFIG_SND_SOC_SE1000) += se1000-machine.o

# audio share
obj-$(CONFIG_SND_SOC_SE1000_SHARE) += audioshare/

# audio tdm multiplexer
obj-$(CONFIG_SND_SOC_SE1000_TDM_MLP) += audio_tdm_mlp/

obj-$(CONFIG_SND_SOC_SE1000_LOOPBACK) += audio_loopback/