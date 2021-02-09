/*
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 *
 * Generated code from MX8M_DDR_tool
 * Align with uboot-imx_v2017.03_4.9.51_imx8m_ga
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/ddr.h>
#include "../ddr.h"

extern void wait_ddrphy_training_complete(void);
void ddr_cfg_phy(void) {
	unsigned int tmp, tmp_t;

	//Init DDRPHY register...
	reg32_write(0x3c080440,0x2);
	reg32_write(0x3c080444,0x3);
	reg32_write(0x3c080448,0x4);
	reg32_write(0x3c08044c,0x5);
	reg32_write(0x3c080450,0x0);
	reg32_write(0x3c080454,0x1);
	reg32_write(0x3c04017c,0x2ff);
	reg32_write(0x3c04057c,0x2ff);
	reg32_write(0x3c04417c,0x2ff);
	reg32_write(0x3c04457c,0x2ff);
	reg32_write(0x3c04817c,0x2ff);
	reg32_write(0x3c04857c,0x2ff);
	reg32_write(0x3c04c17c,0x2ff);
	reg32_write(0x3c04c57c,0x2ff);
	reg32_write(0x3c44017c,0x2ff);
	reg32_write(0x3c44057c,0x2ff);
	reg32_write(0x3c44417c,0x2ff);
	reg32_write(0x3c44457c,0x2ff);
	reg32_write(0x3c44817c,0x2ff);
	reg32_write(0x3c44857c,0x2ff);
	reg32_write(0x3c44c17c,0x2ff);
	reg32_write(0x3c44c57c,0x2ff);
	reg32_write(0x3c84017c,0x2ff);
	reg32_write(0x3c84057c,0x2ff);
	reg32_write(0x3c84417c,0x2ff);
	reg32_write(0x3c84457c,0x2ff);
	reg32_write(0x3c84817c,0x2ff);
	reg32_write(0x3c84857c,0x2ff);
	reg32_write(0x3c84c17c,0x2ff);
	reg32_write(0x3c84c57c,0x2ff);
	reg32_write(0x3c000154,0x3ff);
	reg32_write(0x3c004154,0x3ff);
	reg32_write(0x3c008154,0x3ff);
	reg32_write(0x3c00c154,0x3ff);
	reg32_write(0x3c010154,0xff);
	reg32_write(0x3c014154,0xff);
	reg32_write(0x3c018154,0x3ff);
	reg32_write(0x3c01c154,0x3ff);
	reg32_write(0x3c020154,0x3ff);
	reg32_write(0x3c024154,0x3ff);
	reg32_write(0x3c080314,0xa);
	reg32_write(0x3c480314,0xb);
	reg32_write(0x3c880314,0x6);
	reg32_write(0x3c0800b8,0x2);
	reg32_write(0x3c4800b8,0x1);
	reg32_write(0x3c8800b8,0x1);
	reg32_write(0x3c080090,0x8);
	reg32_write(0x3c0800e8,0x2);
	reg32_write(0x3c480090,0x8);
	reg32_write(0x3c0800e8,0x2);
	reg32_write(0x3c880090,0x8);
	reg32_write(0x3c0800e8,0x2);
	reg32_write(0x3c080158,0x6);
	reg32_write(0x3c480158,0xa);
	reg32_write(0x3c880158,0xa);
	reg32_write(0x3c040134,0x38);
	reg32_write(0x3c040534,0x38);
	reg32_write(0x3c044134,0x38);
	reg32_write(0x3c044534,0x38);
	reg32_write(0x3c048134,0x38);
	reg32_write(0x3c048534,0x38);
	reg32_write(0x3c04c134,0x38);
	reg32_write(0x3c04c534,0x38);
	reg32_write(0x3c440134,0x38);
	reg32_write(0x3c440534,0x38);
	reg32_write(0x3c444134,0x38);
	reg32_write(0x3c444534,0x38);
	reg32_write(0x3c448134,0x38);
	reg32_write(0x3c448534,0x38);
	reg32_write(0x3c44c134,0x38);
	reg32_write(0x3c44c534,0x38);
	reg32_write(0x3c840134,0x38);
	reg32_write(0x3c840534,0x38);
	reg32_write(0x3c844134,0x38);
	reg32_write(0x3c844534,0x38);
	reg32_write(0x3c848134,0x38);
	reg32_write(0x3c848534,0x38);
	reg32_write(0x3c84c134,0x38);
	reg32_write(0x3c84c534,0x38);
	reg32_write(0x3c040124,0xeba);
	reg32_write(0x3c040524,0xeba);
	reg32_write(0x3c044124,0xeba);
	reg32_write(0x3c044524,0xeba);
	reg32_write(0x3c048124,0xeba);
	reg32_write(0x3c048524,0xeba);
	reg32_write(0x3c04c124,0xeba);
	reg32_write(0x3c04c524,0xeba);
	reg32_write(0x3c440124,0xeba);
	reg32_write(0x3c440524,0xeba);
	reg32_write(0x3c444124,0xeba);
	reg32_write(0x3c444524,0xeba);
	reg32_write(0x3c448124,0xeba);
	reg32_write(0x3c448524,0xeba);
	reg32_write(0x3c44c124,0xeba);
	reg32_write(0x3c44c524,0xeba);
	reg32_write(0x3c840124,0xeba);
	reg32_write(0x3c840524,0xeba);
	reg32_write(0x3c844124,0xeba);
	reg32_write(0x3c844524,0xeba);
	reg32_write(0x3c848124,0xeba);
	reg32_write(0x3c848524,0xeba);
	reg32_write(0x3c84c124,0xeba);
	reg32_write(0x3c84c524,0xeba);
	reg32_write(0x3c00010c,0x63);
	reg32_write(0x3c00410c,0x63);
	reg32_write(0x3c00810c,0x63);
	reg32_write(0x3c00c10c,0x63);
	reg32_write(0x3c01010c,0x63);
	reg32_write(0x3c01410c,0x63);
	reg32_write(0x3c01810c,0x63);
	reg32_write(0x3c01c10c,0x63);
	reg32_write(0x3c02010c,0x63);
	reg32_write(0x3c02410c,0x63);
	reg32_write(0x3c080060,0x5);
	reg32_write(0x3c0801d4,0x2);
	reg32_write(0x3c080140,0x0);
	reg32_write(0x3c080020,0x258);
	reg32_write(0x3c480020,0x190);
	reg32_write(0x3c880020,0x10a);
	reg32_write(0x3c080220,0x9);
	reg32_write(0x3c0802c8,0x78);
	reg32_write(0x3c04010c,0x5b1);
	reg32_write(0x3c04050c,0x5b1);
	reg32_write(0x3c04410c,0x5b1);
	reg32_write(0x3c04450c,0x5b1);
	reg32_write(0x3c04810c,0x5b1);
	reg32_write(0x3c04850c,0x5b1);
	reg32_write(0x3c04c10c,0x5b1);
	reg32_write(0x3c04c50c,0x5b1);
	reg32_write(0x3c4802c8,0x78);
	reg32_write(0x3c44010c,0x5b1);
	reg32_write(0x3c44050c,0x5b1);
	reg32_write(0x3c44410c,0x5b1);
	reg32_write(0x3c44450c,0x5b1);
	reg32_write(0x3c44810c,0x5b1);
	reg32_write(0x3c44850c,0x5b1);
	reg32_write(0x3c44c10c,0x5b1);
	reg32_write(0x3c44c50c,0x5b1);
	reg32_write(0x3c8802c8,0x78);
	reg32_write(0x3c84010c,0x5b1);
	reg32_write(0x3c84050c,0x5b1);
	reg32_write(0x3c84410c,0x5b1);
	reg32_write(0x3c84450c,0x5b1);
	reg32_write(0x3c84810c,0x5b1);
	reg32_write(0x3c84850c,0x5b1);
	reg32_write(0x3c84c10c,0x5b1);
	reg32_write(0x3c84c50c,0x5b1);
	reg32_write(0x3c0803e8,0x1);
	reg32_write(0x3c4803e8,0x1);
	reg32_write(0x3c8803e8,0x1);
	reg32_write(0x3c080064,0x5);
	reg32_write(0x3c480064,0x5);
	reg32_write(0x3c880064,0x5);
	reg32_write(0x3c0803c0,0x5555);
	reg32_write(0x3c0803c4,0x5555);
	reg32_write(0x3c0803c8,0x5555);
	reg32_write(0x3c0803cc,0x5555);
	reg32_write(0x3c0803d0,0x5555);
	reg32_write(0x3c0803d4,0x5555);
	reg32_write(0x3c0803d8,0x5555);
	reg32_write(0x3c0803dc,0xf000);
	reg32_write(0x3c080094,0x0);
	reg32_write(0x3c0800b4,0x0);
	reg32_write(0x3c4800b4,0x0);
	reg32_write(0x3c8800b4,0x0);
	reg32_write(0x3c08031c,0x80);
	reg32_write(0x3c48031c,0x80);
	reg32_write(0x3c88031c,0x80);
	reg32_write(0x3c080328,0x106);
	reg32_write(0x3c480328,0x106);
	reg32_write(0x3c880328,0x106);

	//enable APB bus to access DDRPHY RAM
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0000, 0x0);
	//load the 1D training image
	ddr_load_train_code(FW_1D_IMAGE);

	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54000,0x80);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54003,0x960);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54004,0x2);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54005,0x2228);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54006,0x236);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54007,0x2000);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54008,0x101);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54009,0x200);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x5400b,0x31f);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x5400c,0xc8);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54012,0x1);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x5402f,0x834);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54030,0x1);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54031,0x18);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54032,0x240);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54033,0x200);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54034,0x640);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54035,0x816);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54036,0x101);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x5403f,0x1323);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x541fc,0x100);

	//disable APB bus to access DDRPHY RAM
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0000, 0x1);
	//Reset MPU and run
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0099, 0x9);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0099, 0x1);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0099, 0x0);
	wait_ddrphy_training_complete();

	//configure DDRPHY-FW DMEM structure @clock1...
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0099, 0x1);

	//set the PHY input clock to the desired frequency for pstate 1
	reg32_write(0x3038a088,0x7070000);
	reg32_write(0x3038a084,0x4030000);
	reg32_write(0x303a00ec,0xffff);
	tmp=reg32_read(0x303a00f8);
	tmp |= 0x20;
	reg32_write(0x303a00f8,tmp);
	reg32_write(0x30360068,0xec6984);
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
	reg32_write(0x30389808,0x1000000);

	//enable APB bus to access DDRPHY RAM
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0000, 0x0);
	//load the 1D training image
	ddr_load_train_code(FW_1D_IMAGE);

	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54000,0x80);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54002,0x1);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54003,0x640);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54004,0x2);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54005,0x2228);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54006,0x236);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54007,0x2000);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54008,0x101);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54009,0x200);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x5400b,0x21f);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x5400c,0xc8);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54012,0x1);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x5402f,0x210);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54030,0x1);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54032,0x40);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54033,0x200);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54034,0x640);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54035,0x416);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54036,0x101);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x5403f,0x1323);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x541fc,0x100);

	//disable APB bus to access DDRPHY RAM
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0000, 0x1);
	//Reset MPU and run
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0099, 0x9);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0099, 0x1);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0099, 0x0);
	wait_ddrphy_training_complete();

	//configure DDRPHY-FW DMEM structure @clock2...
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0099, 0x1);

	//set the PHY input clock to the desired frequency for pstate 2
	reg32_write(0x3038a088,0x7070000);
	reg32_write(0x3038a084,0x4030000);
	reg32_write(0x303a00ec,0xffff);
	tmp=reg32_read(0x303a00f8);
	tmp |= 0x20;
	reg32_write(0x303a00f8,tmp);
	reg32_write(0x30360068,0x11d0684);
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
	reg32_write(0x30389808,0x1000000);

	//enable APB bus to access DDRPHY RAM
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0000, 0x0);
	//load the 1D training image
	ddr_load_train_code(FW_1D_IMAGE);

	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54000,0x80);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54002,0x2);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54003,0x428);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54004,0x2);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54005,0x2228);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54006,0x236);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54007,0x2000);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54008,0x101);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54009,0x200);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x5400b,0x21f);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x5400c,0xc8);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54012,0x1);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54030,0x1);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54032,0x40);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54033,0x200);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54034,0x640);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54035,0x16);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54036,0x101);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x5403f,0x1323);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x541fc,0x100);

	//disable APB bus to access DDRPHY RAM
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0000, 0x1);
	//Reset MPU and run
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0099, 0x9);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0099, 0x1);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0099, 0x0);
	wait_ddrphy_training_complete();

	//set the PHY input clock to the desired frequency for pstate 0
	reg32_write(0x3038a088,0x7070000);
	reg32_write(0x3038a084,0x4030000);
	reg32_write(0x303a00ec,0xffff);
	tmp=reg32_read(0x303a00f8);
	tmp |= 0x20;
	reg32_write(0x303a00f8,tmp);
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
	reg32_write(0x30389808,0x1000000);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0099, 0x1);


	//enable APB bus to access DDRPHY RAM
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0000, 0x0);
	//load the 2D training image
	ddr_load_train_code(FW_2D_IMAGE);

	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54000,0x80);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54003,0x960);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54004,0x2);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54005,0x2228);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54006,0x236);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54007,0x2000);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54008,0x101);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54009,0x200);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x5400b,0x61);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x5400c,0x1c8);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x5400d,0x101);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x5400e,0x1f7f);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54012,0x1);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x5402f,0x834);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54030,0x1);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54031,0x18);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54032,0x240);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54033,0x200);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54034,0x640);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54035,0x816);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x54036,0x101);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x5403f,0x1323);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x541fc,0x100);

	//disable APB bus to access DDRPHY RAM
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0000, 0x1);
	//Reset MPU and run
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0099, 0x9);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0099, 0x1);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0099, 0x0);
	wait_ddrphy_training_complete();

	//Halt MPU
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0099, 0x1);
	//enable APB bus to access DDRPHY RAM
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0000, 0x0);

	//Load firmware PIE image
	reg32_write(0x3c240000,0x10);
	reg32_write(0x3c240004,0x400);
	reg32_write(0x3c240008,0x10e);
	reg32_write(0x3c24000c,0x0);
	reg32_write(0x3c240010,0x0);
	reg32_write(0x3c240014,0x8);
	reg32_write(0x3c2400a4,0xb);
	reg32_write(0x3c2400a8,0x480);
	reg32_write(0x3c2400ac,0x109);
	reg32_write(0x3c2400b0,0x8);
	reg32_write(0x3c2400b4,0x448);
	reg32_write(0x3c2400b8,0x139);
	reg32_write(0x3c2400bc,0x8);
	reg32_write(0x3c2400c0,0x478);
	reg32_write(0x3c2400c4,0x109);
	reg32_write(0x3c2400c8,0x2);
	reg32_write(0x3c2400cc,0x10);
	reg32_write(0x3c2400d0,0x139);
	reg32_write(0x3c2400d4,0xf);
	reg32_write(0x3c2400d8,0x7c0);
	reg32_write(0x3c2400dc,0x139);
	reg32_write(0x3c2400e0,0x44);
	reg32_write(0x3c2400e4,0x630);
	reg32_write(0x3c2400e8,0x159);
	reg32_write(0x3c2400ec,0x14f);
	reg32_write(0x3c2400f0,0x630);
	reg32_write(0x3c2400f4,0x159);
	reg32_write(0x3c2400f8,0x47);
	reg32_write(0x3c2400fc,0x630);
	reg32_write(0x3c240100,0x149);
	reg32_write(0x3c240104,0x4f);
	reg32_write(0x3c240108,0x630);
	reg32_write(0x3c24010c,0x179);
	reg32_write(0x3c240110,0x8);
	reg32_write(0x3c240114,0xe0);
	reg32_write(0x3c240118,0x109);
	reg32_write(0x3c24011c,0x0);
	reg32_write(0x3c240120,0x7c8);
	reg32_write(0x3c240124,0x109);
	reg32_write(0x3c240128,0x0);
	reg32_write(0x3c24012c,0x1);
	reg32_write(0x3c240130,0x8);
	reg32_write(0x3c240134,0x0);
	reg32_write(0x3c240138,0x45a);
	reg32_write(0x3c24013c,0x9);
	reg32_write(0x3c240140,0x0);
	reg32_write(0x3c240144,0x448);
	reg32_write(0x3c240148,0x109);
	reg32_write(0x3c24014c,0x40);
	reg32_write(0x3c240150,0x630);
	reg32_write(0x3c240154,0x179);
	reg32_write(0x3c240158,0x1);
	reg32_write(0x3c24015c,0x618);
	reg32_write(0x3c240160,0x109);
	reg32_write(0x3c240164,0x40c0);
	reg32_write(0x3c240168,0x630);
	reg32_write(0x3c24016c,0x149);
	reg32_write(0x3c240170,0x8);
	reg32_write(0x3c240174,0x4);
	reg32_write(0x3c240178,0x48);
	reg32_write(0x3c24017c,0x4040);
	reg32_write(0x3c240180,0x630);
	reg32_write(0x3c240184,0x149);
	reg32_write(0x3c240188,0x0);
	reg32_write(0x3c24018c,0x4);
	reg32_write(0x3c240190,0x48);
	reg32_write(0x3c240194,0x40);
	reg32_write(0x3c240198,0x630);
	reg32_write(0x3c24019c,0x149);
	reg32_write(0x3c2401a0,0x10);
	reg32_write(0x3c2401a4,0x4);
	reg32_write(0x3c2401a8,0x18);
	reg32_write(0x3c2401ac,0x0);
	reg32_write(0x3c2401b0,0x4);
	reg32_write(0x3c2401b4,0x78);
	reg32_write(0x3c2401b8,0x549);
	reg32_write(0x3c2401bc,0x630);
	reg32_write(0x3c2401c0,0x159);
	reg32_write(0x3c2401c4,0xd49);
	reg32_write(0x3c2401c8,0x630);
	reg32_write(0x3c2401cc,0x159);
	reg32_write(0x3c2401d0,0x94a);
	reg32_write(0x3c2401d4,0x630);
	reg32_write(0x3c2401d8,0x159);
	reg32_write(0x3c2401dc,0x441);
	reg32_write(0x3c2401e0,0x630);
	reg32_write(0x3c2401e4,0x149);
	reg32_write(0x3c2401e8,0x42);
	reg32_write(0x3c2401ec,0x630);
	reg32_write(0x3c2401f0,0x149);
	reg32_write(0x3c2401f4,0x1);
	reg32_write(0x3c2401f8,0x630);
	reg32_write(0x3c2401fc,0x149);
	reg32_write(0x3c240200,0x0);
	reg32_write(0x3c240204,0xe0);
	reg32_write(0x3c240208,0x109);
	reg32_write(0x3c24020c,0xa);
	reg32_write(0x3c240210,0x10);
	reg32_write(0x3c240214,0x109);
	reg32_write(0x3c240218,0x9);
	reg32_write(0x3c24021c,0x3c0);
	reg32_write(0x3c240220,0x149);
	reg32_write(0x3c240224,0x9);
	reg32_write(0x3c240228,0x3c0);
	reg32_write(0x3c24022c,0x159);
	reg32_write(0x3c240230,0x18);
	reg32_write(0x3c240234,0x10);
	reg32_write(0x3c240238,0x109);
	reg32_write(0x3c24023c,0x0);
	reg32_write(0x3c240240,0x3c0);
	reg32_write(0x3c240244,0x109);
	reg32_write(0x3c240248,0x18);
	reg32_write(0x3c24024c,0x4);
	reg32_write(0x3c240250,0x48);
	reg32_write(0x3c240254,0x18);
	reg32_write(0x3c240258,0x4);
	reg32_write(0x3c24025c,0x58);
	reg32_write(0x3c240260,0xa);
	reg32_write(0x3c240264,0x10);
	reg32_write(0x3c240268,0x109);
	reg32_write(0x3c24026c,0x2);
	reg32_write(0x3c240270,0x10);
	reg32_write(0x3c240274,0x109);
	reg32_write(0x3c240278,0x7);
	reg32_write(0x3c24027c,0x7c0);
	reg32_write(0x3c240280,0x109);
	reg32_write(0x3c240284,0x10);
	reg32_write(0x3c240288,0x10);
	reg32_write(0x3c24028c,0x109);
	reg32_write(0x3c240290,0x0);
	reg32_write(0x3c240294,0x8140);
	reg32_write(0x3c240298,0x10c);
	reg32_write(0x3c24029c,0x10);
	reg32_write(0x3c2402a0,0x8138);
	reg32_write(0x3c2402a4,0x10c);
	reg32_write(0x3c2402a8,0x8);
	reg32_write(0x3c2402ac,0x7c8);
	reg32_write(0x3c2402b0,0x101);
	reg32_write(0x3c2402b4,0x8);
	reg32_write(0x3c2402b8,0x0);
	reg32_write(0x3c2402bc,0x8);
	reg32_write(0x3c2402c0,0x8);
	reg32_write(0x3c2402c4,0x448);
	reg32_write(0x3c2402c8,0x109);
	reg32_write(0x3c2402cc,0xf);
	reg32_write(0x3c2402d0,0x7c0);
	reg32_write(0x3c2402d4,0x109);
	reg32_write(0x3c2402d8,0x47);
	reg32_write(0x3c2402dc,0x630);
	reg32_write(0x3c2402e0,0x109);
	reg32_write(0x3c2402e4,0x8);
	reg32_write(0x3c2402e8,0x618);
	reg32_write(0x3c2402ec,0x109);
	reg32_write(0x3c2402f0,0x8);
	reg32_write(0x3c2402f4,0xe0);
	reg32_write(0x3c2402f8,0x109);
	reg32_write(0x3c2402fc,0x0);
	reg32_write(0x3c240300,0x7c8);
	reg32_write(0x3c240304,0x109);
	reg32_write(0x3c240308,0x8);
	reg32_write(0x3c24030c,0x8140);
	reg32_write(0x3c240310,0x10c);
	reg32_write(0x3c240314,0x0);
	reg32_write(0x3c240318,0x1);
	reg32_write(0x3c24031c,0x8);
	reg32_write(0x3c240320,0x8);
	reg32_write(0x3c240324,0x4);
	reg32_write(0x3c240328,0x8);
	reg32_write(0x3c24032c,0x8);
	reg32_write(0x3c240330,0x7c8);
	reg32_write(0x3c240334,0x101);
	reg32_write(0x3c240018,0x0);
	reg32_write(0x3c24001c,0x0);
	reg32_write(0x3c240020,0x8);
	reg32_write(0x3c240024,0x0);
	reg32_write(0x3c240028,0x0);
	reg32_write(0x3c24002c,0x0);
	reg32_write(0x3c34039c,0x400);
	reg32_write(0x3c24005c,0x0);
	reg32_write(0x3c240098,0x2c);
	reg32_write(0x3c08002c,0x4b);
	reg32_write(0x3c080030,0x96);
	reg32_write(0x3c080034,0x5dc);
	reg32_write(0x3c080038,0x2c);
	reg32_write(0x3c48002c,0x32);
	reg32_write(0x3c480030,0x64);
	reg32_write(0x3c480034,0x3e8);
	reg32_write(0x3c480038,0x2c);
	reg32_write(0x3c88002c,0x21);
	reg32_write(0x3c880030,0x42);
	reg32_write(0x3c880034,0x299);
	reg32_write(0x3c880038,0x21);
	reg32_write(0x3c240030,0x0);
	reg32_write(0x3c240034,0x173);
	reg32_write(0x3c240038,0x60);
	reg32_write(0x3c24003c,0x6110);
	reg32_write(0x3c240040,0x2152);
	reg32_write(0x3c240044,0xdfbd);
	reg32_write(0x3c240048,0xffff);
	reg32_write(0x3c24004c,0x6152);
	reg32_write(0x3c300200, 0x00000000);
	/*
	 * CalBusy.0 =1, indicates the calibrator is actively calibrating.
	 * Wait Calibrating done.
	 */
	tmp_t = 1;
	while(tmp_t) {
		tmp = reg32_read(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x20097);
		tmp_t = tmp & 0x01;
	}
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0000, 0x0);
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0x2006e, 0x0);
	//disable APB bus to access DDRPHY RAM
	reg32_write(IP2APB_DDRPHY_IPS_BASE_ADDR(0) + 4 * 0xd0000, 0x1);
}