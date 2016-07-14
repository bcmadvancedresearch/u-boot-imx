/*
 * Copyright (C) 2012-2015 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreAuto board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6QGR60MXD_CONFIG_H
#define __MX6QGR60MXD_CONFIG_H

#define CONFIG_MACH_TYPE		8894
#define CONFIG_MXC_UART_BASE	UART4_BASE
#define CONFIG_CONSOLE_DEV		"ttymxc3"
#define CONFIG_MMCROOT			"/dev/mmcblk0p2"  /* SDHC4 */
#if defined CONFIG_MX6Q
#define CONFIG_DEFAULT_FDT_FILE	"imx6q-gr60mxd.dtb"
#define PHYS_SDRAM_SIZE		(2u * 1024 * 1024 * 1024)
#endif

#include "mx6sabre_common.h"
#include <asm/imx-common/gpio.h>

/* Overrides setting */
#undef 	CONFIG_PHY_ATHEROS
#define CONFIG_PHY_MICREL
#define	CONFIG_ANDROID_RECOVERY

#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

/* USB Configs */
#define CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET	/* For OTG port */
#define CONFIG_MXC_USB_PORTSC	(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS	0

#define CONFIG_MFG_NAND_PARTITION ""

#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_MMC_ENV_DEV		1  /* eMMC */
#define CONFIG_SYS_MMC_ENV_PART     0  /* user partition */

#if 0
/* I2C Configs */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000
#endif

#endif                         /* __MX6QGR60MXD_CONFIG_H */
