#
# Copyright 2016 Freescale Semiconductor
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += imx8mq_ar8mxm.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-y += ddr/helper.o
ifdef CONFIG_TARGET_IMX8MQ_AR8MXM_4GB
obj-y += ddr/ddr4_4gb/ddr_init.o ddr/ddr4_4gb/ddrphy_train.o
else
obj-y += ddr/ddr4_2gb/ddr_init.o ddr/ddr4_2gb/ddrphy_train.o
endif
obj-$(CONFIG_IMX8M_DDR4) += ddr4_timing.o
endif
