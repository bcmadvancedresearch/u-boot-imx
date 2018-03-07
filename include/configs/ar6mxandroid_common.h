/*
 * Copyright (C) 2013-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef AR6MX_ANDROID_COMMON_H
#define AR6MX_ANDROID_COMMON_H

#define CONFIG_SERIAL_TAG

#define CONFIG_USB_DEVICE
#define CONFIG_IMX_UDC		       1

#define CONFIG_FASTBOOT		       1
#define CONFIG_FASTBOOT_VENDOR_ID      0x18d1
#define CONFIG_FASTBOOT_PRODUCT_ID     0x0d02
#define CONFIG_FASTBOOT_BCD_DEVICE     0x311
#define CONFIG_FASTBOOT_MANUFACTURER_STR  "Freescale"
#define CONFIG_FASTBOOT_PRODUCT_NAME_STR "i.mx6 AR6MX Board"
#define CONFIG_FASTBOOT_INTERFACE_STR	 "Android fastboot"
#define CONFIG_FASTBOOT_CONFIGURATION_STR  "Android fastboot"
#define CONFIG_FASTBOOT_SERIAL_NUM	"12345"
#define CONFIG_FASTBOOT_SATA_NO		 0

#if defined CONFIG_SYS_BOOT_NAND
#define CONFIG_FASTBOOT_STORAGE_NAND
#elif defined CONFIG_SYS_BOOT_SATA
#define CONFIG_FASTBOOT_STORAGE_SATA
#else
#define CONFIG_FASTBOOT_STORAGE_MMC
#endif

/*  For system.img growing up more than 256MB, more buffer needs
*   to receive the system.img*/
#define CONFIG_FASTBOOT_TRANSFER_BUF	0x2c000000
#define CONFIG_FASTBOOT_TRANSFER_BUF_SIZE 0x19000000 /* 400M byte */


/*#define CONFIG_ANDROID_RECOVERY*/
/* which mmc bus is your main storage ? */
#define CONFIG_ANDROID_MAIN_MMC_BUS 3
#define CONFIG_ANDROID_BOOT_PARTITION_MMC 1
#define CONFIG_ANDROID_SYSTEM_PARTITION_MMC 5
#define CONFIG_ANDROID_RECOVERY_PARTITION_MMC 2
#define CONFIG_ANDROID_CACHE_PARTITION_MMC 6

#define CONFIG_CMD_BOOTA
#define CONFIG_SUPPORT_RAW_INITRD
#define CONFIG_SERIAL_TAG

#undef CONFIG_EXTRA_ENV_SETTINGS
#undef CONFIG_BOOTCOMMAND

#define CONFIG_EXTRA_ENV_SETTINGS					\
	"splashpos=m,m\0"	  \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
		"bootargs=console=ttymxc0,115200\0"	\
		"bootargs_ldb=setenv bootargs ${bootargs} init=/init "	\
			"video=mxcfb0:dev=ldb,1024x600M@60,if=RGB666,bpp=32 " \
			"video=mxcfb1:off video=mxcfb2:off video=mxcfb3:off vmalloc=400M "	\
			"androidboot.selinux=disabled "	\
			"androidboot.console=ttymxc0 androidboot.hardware=freescale\0"	\
		"bootargs_hdmi=setenv bootargs ${bootargs} init=/init "	\
			"video=mxcfb0:dev=hdmi,1920x1080M@60,if=RGB24,bpp=32 " \
			"video=mxcfb1:off video=mxcfb2:off video=mxcfb3:off vmalloc=400M "	\
			"androidboot.selinux=disabled "	\
			"androidboot.console=ttymxc0 androidboot.hardware=freescale\0"	\
		"bootargs_lcd=setenv bootargs ${bootargs} init=/init "	\
			"video=mxcfb0:dev=lcd,TIANMA,if=RGB24,bpp=32 " \
			"video=mxcfb1:off " \
			"video=mxcfb2:off video=mxcfb3:off vmalloc=400M "	\
			"androidboot.selinux=disabled "	\
			"androidboot.console=ttymxc0 androidboot.hardware=freescale\0"


#endif /* AR6MX_ANDROID_COMMON_H */
