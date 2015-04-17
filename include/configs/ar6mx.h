/*
 * Config file for BCM AR6MX boards
 *
 * Copyright (C) 2014, BCM Advanced Research
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_AR6MX_H
#define __CONFIG_AR6MX_H

#include <asm/arch/imx-regs.h>
#include "mx6_common.h"

/* Machine config */
#define CONFIG_MX6
#define CONFIG_SYS_LITTLE_ENDIAN
#define CONFIG_MACH_TYPE		4517
#define CONFIG_SYS_HZ			1000

/* Display information on boot */
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO
#define CONFIG_TIMESTAMP

/* CMD */
#include <config_cmd_default.h>
#define CONFIG_CMD_GREPENV
#undef CONFIG_CMD_FLASH
#undef CONFIG_CMD_LOADB
#undef CONFIG_CMD_LOADS
#undef CONFIG_CMD_XIMG
#undef CONFIG_CMD_FPGA
#undef CONFIG_CMD_IMLS

/* MMC */
#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC2_BASE_ADDR

/* Filesystem support */
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_EXT4
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION
#define CONFIG_CMD_FS_GENERIC

/* RAM */
#define PHYS_SDRAM_1			MMDC0_ARB_BASE_ADDR
#define PHYS_SDRAM_2			MMDC1_ARB_BASE_ADDR
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM_1
#define CONFIG_NR_DRAM_BANKS		1
#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		0x10010000
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE
#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Serial console */
#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE
#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{9600, 19200, 38400, 57600, 115200}

/* Shell */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_PROMPT	"AR6MX > "
#define CONFIG_SYS_CBSIZE	1024
#define CONFIG_SYS_MAXARGS	16
#define CONFIG_CMDLINE_EDITING
#define CONFIG_SYS_BARGSIZE	CONFIG_SYS_CBSIZE
#define CONFIG_SYS_PBSIZE	(CONFIG_SYS_CBSIZE + \
					sizeof(CONFIG_SYS_PROMPT) + 16)

/* SPI flash */
#define CONFIG_SYS_NO_FLASH
#define CONFIG_CMD_SF
#define CONFIG_SF_DEFAULT_BUS		2
#define CONFIG_SF_DEFAULT_CS		0
#define CONFIG_SF_DEFAULT_SPEED		25000000
#define CONFIG_SF_DEFAULT_MODE		(SPI_MODE_0)

/* Environment */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_ENV_IS_IN_MMC
#if defined(CONFIG_ENV_IS_IN_MMC)
  #define CONFIG_ENV_OFFSET              (8 * 64 * 1024)
  #define CONFIG_ENV_SIZE                (8 * 1024)
  #define CONFIG_SYS_MMC_ENV_DEV         0
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
  #define CONFIG_ENV_OFFSET              (512 * 1024)
  #define CONFIG_ENV_SECT_SIZE           (64 * 1024)
  #define CONFIG_ENV_SIZE                (8 * 1024)
  #define CONFIG_ENV_SPI_BUS             CONFIG_SF_DEFAULT_BUS
  #define CONFIG_ENV_SPI_CS              CONFIG_SF_DEFAULT_CS
  #define CONFIG_ENV_SPI_MODE            CONFIG_SF_DEFAULT_MODE
  #define CONFIG_ENV_SPI_MAX_HZ          CONFIG_SF_DEFAULT_SPEED
#endif

#define CONFIG_BOOTDELAY          3
#define CONFIG_IPADDR             10.1.2.156
#define CONFIG_SERVERIP           10.1.2.189
#define CONFIG_MMCROOT      "/dev/mmcblk0p2"
#define CONFIG_CONSOLE_DEV    "ttymxc0"

#define CONFIG_EXTRA_ENV_SETTINGS \
	"script=boot.scr\0" \
	"image=uImage\0" \
	"boot_fdt=try\0" \
	"console=" CONFIG_CONSOLE_DEV "\0" \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=1\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait rw\0" \
	"bootdevs=mmc usb sata\0" \
	"fdt_addr=0x18000000\0" \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
	"bootargs_mmc1=console=ttymxc0,115200 console=tty1\0" \
	"bootargs_mmc2a=video=mxcfb0:dev=hdmi,1920x1080M@60,if=RGB24,bpp=32 " \
			"video=mxcfb1:off video=mxcfb2:off fbmem=28M\0" \
	"bootargs_mmc2b=video=mxcfb0:dev=ldb,1024x600M@60,if=RGB666,bpp=32 " \
			"video=mxcfb1:off video=mxcfb2:off fbmem=28M\0" \
	"bootargs_mmc3a=root=/dev/mmcblk0p1 rootwait consoleblank=0 \0" \
	"bootargs_mmc3b=root=/dev/mmcblk0p2 rootwait consoleblank=0 \0" \
	"bootargs_hdmi=setenv bootargs ${bootargs_mmc1} ${bootargs_mmc2a} \0" \
	"bootargs_ldb=setenv bootargs ${bootargs_mmc1} ${bootargs_mmc2b} \0" \
	"detect_hdmi=" \
			"if hdmidet; then " \
				"run bootargs_hdmi; " \
			"else " \
				"run bootargs_ldb; " \
			"fi\0" \
	"bootcmd_mmc=run detect_hdmi; " \
			"setenv bootargs ${bootargs} ${bootargs_mmc3a}; " \
			"mmc dev 0; mmc read 0x10800000 0x800 0x2000; bootm\0" \
	"mmcargs=run detect_hdmi; " \
			"setenv bootargs ${bootargs} ${bootargs_mmc3b};\0" \
	"loadimage=load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadfdt=load mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if itest.s \"x6SOLO\" == \"x${cpu}\" || " \
			"itest.s \"x6DL\" == \"x${cpu}\"; then " \
			"setenv fdt_file imx6dl-ar6mx.dtb ;" \
		"else " \
			"setenv fdt_file imx6q-ar6mx.dtb ;" \
		"fi; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootm ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootm; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootm; " \
		"fi;\0" \
	"loadbootscript=for dtype in ${bootdevs}" \
		"; do " \
			"if itest.s \"xusb\" == \"x${dtype}\" ; then " \
				"usb start ;" \
			"fi; " \
			"for disk in 0 1 ; do ${dtype} dev ${disk} ;" \
				"load " \
					"${dtype} ${disk}:1 " \
					"10008000 " \
					"/${script}" \
					"&& source 10008000 ; " \
			"done ; " \
		"done;\0" \

#define CONFIG_BOOTCOMMAND \
	"if run loadbootscript; then; " \
		"else " \
		"if run loadimage; then; " \
			"run mmcboot; " \
		"else " \
			"if run bootcmd_mmc; then; " \
			"fi; " \
		"fi;" \
	"fi"

/* SPI */
#define CONFIG_SPI
#define CONFIG_MXC_SPI
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_ATMEL
#define CONFIG_SPI_FLASH_EON
#define CONFIG_SPI_FLASH_GIGADEVICE
#define CONFIG_SPI_FLASH_MACRONIX
#define CONFIG_SPI_FLASH_SPANSION
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_SPI_FLASH_SST
#define CONFIG_SPI_FLASH_WINBOND

/* Ethernet */
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_FEC_MXC_PHYADDR		3
#define CONFIG_FEC_XCV_TYPE		RGMII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS
#define CONFIG_MII
#define CONFIG_ETHPRIME			"FEC0"
#define CONFIG_ARP_TIMEOUT		200UL
#define CONFIG_NET_RETRY_COUNT		5
#define CONFIG_PHY_MICREL
#define CONFIG_PHY_MICREL_KSZ9031

/* USB */
#define CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET	/* For OTG port */

/* I2C */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000

/* SATA */
#define CONFIG_CMD_SATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_LIBATA
#define CONFIG_LBA48
#define CONFIG_DWC_AHSATA
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR

/* GPIO */
#define CONFIG_MXC_GPIO

/* Framebuffer and LCD */
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_IPUV3_CLK          260000000
#define CONFIG_CMD_HDMIDETECT
#define CONFIG_CONSOLE_MUX
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP

/* Boot */
#define CONFIG_ZERO_BOOTDELAY_CHECK
#define CONFIG_LOADADDR			0x10800000
#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_CMDLINE_TAG		/* enable passing of ATAGs */
#define CONFIG_SYS_BOOTMAPSZ	        (8 << 20)
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

/* misc */
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_CMD_BMODE
#define CONFIG_CMD_BOOTZ
#define CONFIG_CMD_SETEXPR
#define CONFIG_SYS_GENERIC_BOARD
#define CONFIG_STACKSIZE			(128 * 1024)
#define CONFIG_SYS_MALLOC_LEN			(10 * 1024 * 1024)
#define CONFIG_SYS_U_BOOT_MAX_SIZE_SECTORS	800 /* 400 KB */

/* Device Tree Support */
#define CONFIG_OF_LIBFDT

/* SPL */
#define CONFIG_SPL_MMC_SUPPORT
#include "imx6_spl.h"
#define CONFIG_SPL_SPI_SUPPORT
#define CONFIG_SPL_SPI_FLASH_SUPPORT
#define CONFIG_SYS_SPI_U_BOOT_OFFS	(69 * 1024)
#define CONFIG_SPL_SPI_LOAD

#endif	/* __CONFIG_AR6MX_H */
