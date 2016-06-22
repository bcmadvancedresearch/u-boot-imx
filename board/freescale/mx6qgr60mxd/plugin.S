/*
 * Copyright (C) 2012-2015 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <config.h>

/* DDR script */
.macro imx6dqgr60mxd_ddr_setting
	ldr r0, =IOMUXC_BASE_ADDR
	ldr r1, =0x000c0000
	str r1, [r0, #0x798]
	ldr r1, =0x00000000
	str r1, [r0, #0x758]

	ldr r1, =0x00000030
	str r1, [r0, #0x588]
	str r1, [r0, #0x594]
	str r1, [r0, #0x56c]
	str r1, [r0, #0x578]
	str r1, [r0, #0x74c]
	str r1, [r0, #0x57c]

	ldr r1, =0x00000000
	str r1, [r0, #0x58c]

	ldr r1, =0x00000030
	str r1, [r0, #0x59c]
	str r1, [r0, #0x5a0]
	str r1, [r0, #0x78c]

	ldr r1, =0x00020000
	str r1, [r0, #0x750]

	ldr r1, =0x00000028
	str r1, [r0, #0x5a8]
	str r1, [r0, #0x5b0]
	str r1, [r0, #0x524]
	str r1, [r0, #0x51c]
	str r1, [r0, #0x518]
	str r1, [r0, #0x50c]
	str r1, [r0, #0x5b8]
	str r1, [r0, #0x5c0]

	ldr r1, =0x00020000
	str r1, [r0, #0x774]

	ldr r1, =0x00000028
	str r1, [r0, #0x784]
	str r1, [r0, #0x788]
	str r1, [r0, #0x794]
	str r1, [r0, #0x79c]
	str r1, [r0, #0x7a0]
	str r1, [r0, #0x7a4]
	str r1, [r0, #0x7a8]
	str r1, [r0, #0x748]
	str r1, [r0, #0x5ac]
	str r1, [r0, #0x5b4]
	str r1, [r0, #0x528]
	str r1, [r0, #0x520]
	str r1, [r0, #0x514]
	str r1, [r0, #0x510]
	str r1, [r0, #0x5bc]
	str r1, [r0, #0x5c4]

	ldr r0, =MMDC_P0_BASE_ADDR
	ldr r2, =0xa1390003
	str r2, [r0, #0x800]

	ldr r2, =0x001F001F
	str r2, [r0, #0x80c]
	str r2, [r0, #0x810]
	ldr r1, =MMDC_P1_BASE_ADDR
	str r2, [r1, #0x80c]
	str r2, [r1, #0x810]

	ldr r2, =0x43260335
	str r2, [r0, #0x83c]
	ldr r2, =0x031A030B
	str r2, [r0, #0x840]

	ldr r2, =0x4323033B
	str r2, [r1, #0x83c]
	ldr r2, =0x0323026F
	str r2, [r1, #0x840]

	ldr r2, =0x483D4545
	str r2, [r0, #0x848]
	ldr r2, =0x44433E48
	str r2, [r1, #0x848]

	ldr r2, =0x41444840
	str r2, [r0, #0x850]
	ldr r2, =0x4835483E
	str r2, [r1, #0x850]

	ldr r2, =0x33333333
	str r2, [r0, #0x81c]
	str r2, [r0, #0x820]
	str r2, [r0, #0x824]
	str r2, [r0, #0x828]
	str r2, [r1, #0x81c]
	str r2, [r1, #0x820]
	str r2, [r1, #0x824]
	str r2, [r1, #0x828]

	ldr r2, =0x00000800
	str r2, [r0, #0x8b8]
	str r2, [r1, #0x8b8]

	ldr r2, =0x00020036
	str r2, [r0, #0x004]
	ldr r2, =0x09444040
	str r2, [r0, #0x008]

	ldr r2, =0x8A8F7955
	str r2, [r0, #0x00c]
	ldr r2, =0xFF328F64
	str r2, [r0, #0x010]

	ldr r2, =0x01FF00DB
	str r2, [r0, #0x014]
	ldr r2, =0x00001740
	str r2, [r0, #0x018]

	ldr r2, =0x00008000
	str r2, [r0, #0x01c]
	ldr r2, =0x000026d2
	str r2, [r0, #0x02c]
	ldr r2, =0x008F1023
	str r2, [r0, #0x030]
	ldr r2, =0x00000047
	str r2, [r0, #0x040]

	ldr r2, =0x841A0000
	str r2, [r0, #0x000]

	ldr r2, =0x04088032
	str r2, [r0, #0x01c]
	ldr r2, =0x00008033
	str r2, [r0, #0x01c]
	ldr r2, =0x00048031
	str r2, [r0, #0x01c]
	ldr r2, =0x09408030
	str r2, [r0, #0x01c]
	ldr r2, =0x04008040
	str r2, [r0, #0x01c]

	ldr r2, =0x00005800
	str r2, [r0, #0x020]
	ldr r2, =0x00011117
	str r2, [r0, #0x818]
	str r2, [r1, #0x818]
	ldr r2, =0x00025576
	str r2, [r0, #0x004]
	ldr r2, =0x00011006
	str r2, [r0, #0x404]
	ldr r2, =0x00000000
	str r2, [r0, #0x01c]
.endm

.macro imx6_clock_gating
	ldr r0, =CCM_BASE_ADDR
	ldr r1, =0x00C03F3F
	str r1, [r0, #0x068]
	ldr r1, =0x0030FC03
	str r1, [r0, #0x06c]
	ldr r1, =0x0FFFC000
	str r1, [r0, #0x070]
	ldr r1, =0x3FF00000
	str r1, [r0, #0x074]
	ldr r1, =0xFFFFF300
	str r1, [r0, #0x078]
	ldr r1, =0x0F0000C3
	str r1, [r0, #0x07c]
	ldr r1, =0x00000FFF
	str r1, [r0, #0x080]
.endm

.macro imx6_qos_setting
	ldr r0, =IOMUXC_BASE_ADDR
	ldr r1, =0xF00000CF
	str r1, [r0, #0x10]

	ldr r1, =0x007F007F
	str r1, [r0, #0x18]
	str r1, [r0, #0x1c]
.endm

.macro imx6_ddr_setting
#if defined (CONFIG_MX6Q)
	imx6dqgr60mxd_ddr_setting
#else
	#error "SOC not configured"
#endif
.endm

/* include the common plugin code here */
#include <asm/arch/mx6_plugin.S>
