# SPDX-License-Identifier: GPL-2.0
dtbo-$(CONFIG_ARCH_ROCKCHIP) += \
	devspi2.dtbo \
	spi1-flash.dtbo \
	rpi-camera-v1p3.dtbo \
	rpi-camera-v2.dtbo \

dtbotxt-$(CONFIG_ARCH_ROCKCHIP) += \
	README.rockchip-overlays

targets += $(dtbo-y) $(dtbotxt-y)

always-y		:= $(dtbo-y) $(dtbotxt-y)
clean-files		:= *.dtbo

