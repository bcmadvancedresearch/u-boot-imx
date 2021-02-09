/*
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 *
 * Generated code from MX8M_DDR_tool
 * Align with uboot-imx_v2017.03_4.9.51_imx8m_ga
 */

#include <common.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/arch/ddr.h>
#include <asm/arch/clock.h>
#include "../ddr.h"

#ifdef CONFIG_ENABLE_DDR_TRAINING_DEBUG
#define ddr_printf(args...) printf(args)
#else
#define ddr_printf(args...)
#endif

#include "../wait_ddrphy_training_complete.c"
#ifndef SRC_DDRC_RCR_ADDR
#define SRC_DDRC_RCR_ADDR SRC_IPS_BASE_ADDR +0x1000
#endif
#ifndef DDR_CSD1_BASE_ADDR
#define DDR_CSD1_BASE_ADDR 0x40000000
#endif
#define SILICON_TRAIN

void ddr_cfg_phy(void);
volatile unsigned int tmp, tmp_t, i;
void ddr_init(void)
{
	/** Initialize DDR clock and DDRC registers **/
	reg32_write(0x30330234,0x0);
	reg32_write(0x30330238,0x0);
	reg32_write(0x3033049c,0xe);
	reg32_write(0x303304a0,0xe);
	reg32_write(0x303304f4,0x0);
	reg32_write(0x3038a088,0x7070000);
	reg32_write(0x3038a084,0x4030000);
	reg32_write(0x303a00ec,0xffff);
	tmp=reg32_read(0x303a00f8);
	tmp |= 0x20;
	reg32_write(0x303a00f8,tmp);
	reg32_write(0x30391000,0x8f000000);
	reg32_write(0x30391004,0x8f000000);
	reg32_write(0x30360068,0xec6480);
	tmp=reg32_read(0x30360060);
	tmp &= ~0x80;
	reg32_write(0x30360060,tmp);
	tmp=reg32_read(0x30360060);
	tmp |= 0x200;
	reg32_write(0x30360060,tmp);
	tmp=reg32_read(0x30360060);
	tmp &= ~0x20;
	reg32_write(0x30360060,tmp);
	tmp=reg32_read(0x30360060);
	tmp &= ~0x10;
	reg32_write(0x30360060,tmp);
	do{
		tmp=reg32_read(0x30360060);
		if(tmp&0x80000000) break;
	}while(1);
	reg32_write(0x30391000,0x8f000006);
	reg32_write(0x3d400304,0x1);
	reg32_write(0x3d400030,0x1);
	reg32_write(0x3d400000,0x41040010);
	reg32_write(0x3d400028,0x1);
	reg32_write(0x3d400030,0xaa);
	reg32_write(0x3d400034,0x221306);
	reg32_write(0x3d400038,0x840000);
	reg32_write(0x3d40003c,0x30);
	reg32_write(0x3d400050,0x210070);
	reg32_write(0x3d400054,0x10008);
	reg32_write(0x3d400060,0x10);
	reg32_write(0x3d400064,0x49009c);
	reg32_write(0x3d4000c0,0x0);
	reg32_write(0x3d4000c4,0x1000);
	reg32_write(0x3d4000d0,0xc0030126);
	reg32_write(0x3d4000d4,0x770000);
	reg32_write(0x3d4000dc,0x8340001);
	reg32_write(0x3d4000e0,0x180240);
	reg32_write(0x3d4000e4,0x110000);
	reg32_write(0x3d4000e8,0x2000640);
	reg32_write(0x3d4000ec,0x816);
	reg32_write(0x3d4000f0,0x22);
	reg32_write(0x3d4000f4,0x527);
	reg32_write(0x3d400100,0x110d2914);
	reg32_write(0x3d400104,0x4051c);
	reg32_write(0x3d400108,0x608050d);
	reg32_write(0x3d40010c,0x400c);
	reg32_write(0x3d400110,0x8030309);
	reg32_write(0x3d400114,0x6060403);
	reg32_write(0x3d40011c,0x606);
	reg32_write(0x3d400120,0x5050d08);
	reg32_write(0x3d400124,0x2020a);
	reg32_write(0x3d40012c,0x1409010e);
	reg32_write(0x3d400130,0x8);
	reg32_write(0x3d40013c,0x0);
	reg32_write(0x3d400180,0x1000040);
	reg32_write(0x3d400184,0x493e);
	reg32_write(0x3d400190,0x38b8207);
	reg32_write(0x3d400194,0x2020303);
	reg32_write(0x3d400198,0x7f04011);
	reg32_write(0x3d40019c,0xb0);
	reg32_write(0x3d4001a0,0xe0400018);
	reg32_write(0x3d4001a4,0x48005a);
	reg32_write(0x3d4001a8,0x80000000);
	reg32_write(0x3d4001b0,0x1);
	reg32_write(0x3d4001b4,0xb07);
	reg32_write(0x3d4001b8,0x4);
	reg32_write(0x3d4001c0,0x1);
	reg32_write(0x3d4001c4,0x0);
	reg32_write(0x3d400200,0x3f1f);
	reg32_write(0x3d400204,0x3f0a0a);
	reg32_write(0x3d400208,0x2020200);
	reg32_write(0x3d40020c,0x2020202);
	reg32_write(0x3d400210,0x1f1f);
	reg32_write(0x3d400214,0x8080808);
	reg32_write(0x3d400218,0x8080808);
	reg32_write(0x3d40021c,0xf0f);
	reg32_write(0x3d400220,0x101);
	reg32_write(0x3d400240,0x6000610);
	reg32_write(0x3d400244,0x1323);
	reg32_write(0x3d400250,0x317d1a07);
	reg32_write(0x3d400254,0xf);
	reg32_write(0x3d40025c,0x2a001b76);
	reg32_write(0x3d400264,0x9);
	reg32_write(0x3d40026c,0x30000e06);
	reg32_write(0x3d400300,0x14);
	reg32_write(0x3d400304,0x0);
	reg32_write(0x3d40030c,0x0);
	reg32_write(0x3d400320,0x1);
	reg32_write(0x3d40036c,0x10);
	reg32_write(0x3d400400,0x11);
	reg32_write(0x3d400404,0x13193);
	reg32_write(0x3d400408,0x6096);
	reg32_write(0x3d400490,0x1);
	reg32_write(0x3d400494,0x2000c00);
	reg32_write(0x3d400498,0x3c00db);
	reg32_write(0x3d40049c,0x100001);
	reg32_write(0x3d4004a0,0x41f);
	reg32_write(0x3d402050,0x210070);
	reg32_write(0x3d402064,0x300068);
	reg32_write(0x3d4020dc,0x2100001);
	reg32_write(0x3d4020e0,0x40);
	reg32_write(0x3d4020e8,0x2000640);
	reg32_write(0x3d4020ec,0x416);
	reg32_write(0x3d402100,0xd091b0d);
	reg32_write(0x3d402104,0x30313);
	reg32_write(0x3d402108,0x506040a);
	reg32_write(0x3d40210c,0x400c);
	reg32_write(0x3d402110,0x6030206);
	reg32_write(0x3d402114,0x4040302);
	reg32_write(0x3d40211c,0x404);
	reg32_write(0x3d402120,0x4040d06);
	reg32_write(0x3d402124,0x20208);
	reg32_write(0x3d40212c,0x1206010e);
	reg32_write(0x3d402130,0x8);
	reg32_write(0x3d40213c,0x0);
	reg32_write(0x3d402180,0x1000040);
	reg32_write(0x3d402190,0x3868204);
	reg32_write(0x3d402194,0x2020303);
	reg32_write(0x3d4021b4,0x604);
	reg32_write(0x3d4021b8,0x4);
	reg32_write(0x3d402240,0x6000608);
	reg32_write(0x3d403050,0x210070);
	reg32_write(0x3d403064,0x200045);
	reg32_write(0x3d4030dc,0x1);
	reg32_write(0x3d4030e0,0x40);
	reg32_write(0x3d4030e8,0x2000640);
	reg32_write(0x3d4030ec,0x16);
	reg32_write(0x3d403100,0xb061209);
	reg32_write(0x3d403104,0x2020d);
	reg32_write(0x3d403108,0x5050309);
	reg32_write(0x3d40310c,0x400c);
	reg32_write(0x3d403110,0x4030205);
	reg32_write(0x3d403114,0x3030202);
	reg32_write(0x3d40311c,0x303);
	reg32_write(0x3d403120,0x3040d04);
	reg32_write(0x3d403124,0x20208);
	reg32_write(0x3d40312c,0x1005010e);
	reg32_write(0x3d403130,0x8);
	reg32_write(0x3d40313c,0x0);
	reg32_write(0x3d403180,0x1000040);
	reg32_write(0x3d403190,0x3848204);
	reg32_write(0x3d403194,0x2020303);
	reg32_write(0x3d4031b4,0x404);
	reg32_write(0x3d4031b8,0x4);
	reg32_write(0x3d403240,0x6000600);
	reg32_write(0x3d400060,0x11);
	reg32_write(0x30391000,0x8f000004);
	reg32_write(0x30391000,0x8f000000);
	reg32_write(0x3d400304,0x0);
	reg32_write(0x3d400030,0xaa);
	reg32_write(0x3d400320,0x0);
	reg32_write(0x3d4001b0,0x0);

	/* Configure DDR PHY's registers */
	ddr_cfg_phy();

	reg32_write(DDRC_RFSHCTL3(0), 0x00000000);
	reg32_write(DDRC_SWCTL(0), 0x0000);
	/*
	 * ------------------- 9 -------------------
	 * Set DFIMISC.dfi_init_start to 1 
	 *  -----------------------------------------
	 */
	reg32_write(DDRC_DFIMISC(0), 0x00000030);
	reg32_write(DDRC_SWCTL(0), 0x0001);

	/* wait DFISTAT.dfi_init_complete to 1 */
	tmp_t = 0;
	while(tmp_t==0){
		tmp  = reg32_read(DDRC_DFISTAT(0));
		tmp_t = tmp & 0x01;
		tmp  = reg32_read(DDRC_MRSTAT(0));
	}

	reg32_write(DDRC_SWCTL(0), 0x0000);

	/* clear DFIMISC.dfi_init_complete_en */
	reg32_write(DDRC_DFIMISC(0), 0x00000010);
	reg32_write(DDRC_DFIMISC(0), 0x00000011);
	reg32_write(DDRC_PWRCTL(0), 0x00000088);

	tmp = reg32_read(DDRC_CRCPARSTAT(0));
	/*
	 * set SWCTL.sw_done to enable quasi-dynamic register
	 * programming outside reset.
	 */
	reg32_write(DDRC_SWCTL(0), 0x00000001);

	/* wait SWSTAT.sw_done_ack to 1 */
	while((reg32_read(DDRC_SWSTAT(0)) & 0x1) == 0)
		;

	/* wait STAT.operating_mode([1:0] for ddr3) to normal state */
	while ((reg32_read(DDRC_STAT(0)) & 0x3) != 0x1)
		;

	reg32_write(DDRC_PWRCTL(0), 0x00000088);
	/* reg32_write(DDRC_PWRCTL(0), 0x018a); */
	tmp = reg32_read(DDRC_CRCPARSTAT(0));

	/* enable port 0 */
	reg32_write(DDRC_PCTRL_0(0), 0x00000001);
	/* enable DDR auto-refresh mode */
	tmp = reg32_read(DDRC_RFSHCTL3(0)) & ~0x1;
	reg32_write(DDRC_RFSHCTL3(0), tmp);
}