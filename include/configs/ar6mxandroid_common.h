/*
 * Copyright (C) 2013-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef AR6MX_ANDROID_COMMON_H
#define AR6MX_ANDROID_COMMON_H
#include "mx_android_common.h"

#define CONFIG_CMD_FASTBOOT
#define CONFIG_ANDROID_BOOT_IMAGE
#define CONFIG_FASTBOOT_LOCK
#define FSL_FASTBOOT_FB_DEV "mmc"
#define FASTBOOT_ENCRYPT_LOCK
#define CONFIG_FSL_CAAM_KB
#define CONFIG_CMD_FSL_CAAM_KB
#define CONFIG_SHA1
#define CONFIG_SHA256

#define CONFIG_ANDROID_RECOVERY

#ifdef CONFIG_SYS_CBSIZE
#undef CONFIG_SYS_CBSIZE
#define CONFIG_SYS_CBSIZE 2048
#endif

#ifdef CONFIG_SYS_MALLOC_LEN
#undef CONFIG_SYS_MALLOC_LEN
#define CONFIG_SYS_MALLOC_LEN           (96 * SZ_1M)
#endif

#undef CONFIG_EXTRA_ENV_SETTINGS

#ifdef CONFIG_TARGET_BCM_BH6MX
#define SERIAL_CONSOLE      "ttymxc3"
#define CMA_SIZE "512M"
#define CONTI_SIZE "67108864"
#define RGB_FORMAT "RGB24"
#else
#define SERIAL_CONSOLE      "ttymxc0"
#define CMA_SIZE "448M"
#define CONTI_SIZE "33554432"
#define RGB_FORMAT "RGB666"
#endif

#define CONFIG_EXTRA_ENV_SETTINGS					\
	"splashpos=m,m\0"	  \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
		"bootargs=console=" SERIAL_CONSOLE ",115200 "	\
			"video=mxcfb1:off video=mxcfb2:off video=mxcfb3:off vmalloc=128M "	\
			"cma=" CMA_SIZE " galcore.contiguousSize=" CONTI_SIZE "\0"	\
		"bootargs_an=androidboot.console=" SERIAL_CONSOLE \
			" androidboot.selinux=permissive "	\
			"androidboot.console=ttymxc0 androidboot.hardware=freescale\0"	\
		"bootargs_ldb=setenv bootargs ${bootargs} init=/init "	\
			"video=mxcfb0:dev=ldb,if=" RGB_FORMAT ",bpp=32 " \
			"${bootargs_an}\0"	\
		"bootargs_hdmi=setenv bootargs ${bootargs} init=/init "	\
			"video=mxcfb0:dev=hdmi,1280x720M@60,if=RGB24,bpp=32 " \
			"${bootargs_an}\0"	\

#endif /* AR6MX_ANDROID_COMMON_H */
