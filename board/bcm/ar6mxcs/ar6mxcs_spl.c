/*
 * SPL specific code for BCM AR6MXCS board
 *
 * Copyright (C) 2014 BCM Advanced Research
 *
 * Author: Frodo Lai <frodo_lai@bcmcom.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <common.h>
#include <spl.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/mx6-ddr.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/crm_regs.h>
#include <asm/imx-common/iomux-v3.h>
#include <fsl_esdhc.h>

DECLARE_GLOBAL_DATA_PTR;

#define AR6MXCS_GREEN_LED1 IMX_GPIO_NR(6, 9)
#define AR6MXCS_GREEN_LED2 IMX_GPIO_NR(6, 10)
#define AR6MXCS_SD3_CD IMX_GPIO_NR(7, 0)

/*
 * Below DRAM_RESET[DDR_SEL] = 0 which is incorrect according to
 * Freescale QRM, but this is exactly the value used by the automatic
 * calibration script and it works also in all our tests, so we leave
 * it as is at this point.
 */
#define AR6MX_DDR_IOMUX_CFG \
	.dram_sdqs0	= 0x00000030, \
	.dram_sdqs1	= 0x00000030, \
	.dram_sdqs2	= 0x00000030, \
	.dram_sdqs3	= 0x00000030, \
	.dram_sdqs4	= 0x00000030, \
	.dram_sdqs5	= 0x00000030, \
	.dram_sdqs6	= 0x00000030, \
	.dram_sdqs7	= 0x00000030, \
	.dram_dqm0	= 0x00000030, \
	.dram_dqm1	= 0x00000030, \
	.dram_dqm2	= 0x00000030, \
	.dram_dqm3	= 0x00000030, \
	.dram_dqm4	= 0x00000030, \
	.dram_dqm5	= 0x00000030, \
	.dram_dqm6	= 0x00000030, \
	.dram_dqm7	= 0x00000030, \
	.dram_cas	= 0x00000030, \
	.dram_ras	= 0x00000030, \
	.dram_sdclk_0	= 0x00000030, \
	.dram_sdclk_1	= 0x00000030, \
	.dram_sdcke0	= 0x00003000, \
	.dram_sdcke1	= 0x00003000, \
	.dram_reset	= 0x00000030, \
	.dram_sdba2	= 0x00000000, \
	.dram_sdodt0	= 0x00000030, \
	.dram_sdodt1	= 0x00000030,

#define AR6MX_GPR_IOMUX_CFG \
	.grp_b0ds	= 0x00000030, \
	.grp_b1ds	= 0x00000030, \
	.grp_b2ds	= 0x00000030, \
	.grp_b3ds	= 0x00000030, \
	.grp_b4ds	= 0x00000030, \
	.grp_b5ds	= 0x00000030, \
	.grp_b6ds	= 0x00000030, \
	.grp_b7ds	= 0x00000030, \
	.grp_addds	= 0x00000030, \
	.grp_ddrmode_ctl = 0x00020000, \
	.grp_ddrpke	= 0x00000000, \
	.grp_ddrmode	= 0x00020000, \
	.grp_ctlds	= 0x00000030, \
	.grp_ddr_type	= 0x000C0000,

/* configure MX6Q/DUAL mmdc DDR io registers */
struct mx6dq_iomux_ddr_regs mx6dq_ddr_ioregs = {
	/* SDCLK[0:1], CAS, RAS, Reset: Differential input, 40ohm */
	.dram_sdclk_0 = 0x00020030,
	.dram_sdclk_1 = 0x00020030,
	.dram_cas = 0x00020030,
	.dram_ras = 0x00020030,
	.dram_reset = 0x00020030,
	/* SDCKE[0:1]: 100k pull-up */
	.dram_sdcke0 = 0x00003000,
	.dram_sdcke1 = 0x00003000,
	/* SDBA2: pull-up disabled */
	.dram_sdba2 = 0x00000000,
	/* SDODT[0:1]: 100k pull-up, 40 ohm */
	.dram_sdodt0 = 0x00003030,
	.dram_sdodt1 = 0x00003030,
	/* SDQS[0:7]: Differential input, 40 ohm */
	.dram_sdqs0 = 0x00000030,
	.dram_sdqs1 = 0x00000030,
	.dram_sdqs2 = 0x00000030,
	.dram_sdqs3 = 0x00000030,
	.dram_sdqs4 = 0x00000030,
	.dram_sdqs5 = 0x00000030,
	.dram_sdqs6 = 0x00000030,
	.dram_sdqs7 = 0x00000030,

	/* DQM[0:7]: Differential input, 40 ohm */
	.dram_dqm0 = 0x00020030,
	.dram_dqm1 = 0x00020030,
	.dram_dqm2 = 0x00020030,
	.dram_dqm3 = 0x00020030,
	.dram_dqm4 = 0x00020030,
	.dram_dqm5 = 0x00020030,
	.dram_dqm6 = 0x00020030,
	.dram_dqm7 = 0x00020030,
};

/* configure MX6Q/DUAL mmdc GRP io registers */
struct mx6dq_iomux_grp_regs mx6dq_grp_ioregs = {
	/* DDR3 */
	.grp_ddr_type = 0x000c0000,
	.grp_ddrmode_ctl = 0x00020000,
	/* disable DDR pullups */
	.grp_ddrpke = 0x00000000,
	/* ADDR[00:16], SDBA[0:1]: 40 ohm */
	.grp_addds = 0x00000030,
	/* CS0/CS1/SDBA2/CKE0/CKE1/SDWE: 40 ohm */
	.grp_ctlds = 0x00000030,
	/* DATA[00:63]: Differential input, 40 ohm */
	.grp_ddrmode = 0x00020000,
	.grp_b0ds = 0x00000030,
	.grp_b1ds = 0x00000030,
	.grp_b2ds = 0x00000030,
	.grp_b3ds = 0x00000030,
	.grp_b4ds = 0x00000030,
	.grp_b5ds = 0x00000030,
	.grp_b6ds = 0x00000030,
	.grp_b7ds = 0x00000030,
};

/* configure MX6SOLO/DUALLITE mmdc DDR io registers */
struct mx6sdl_iomux_ddr_regs mx6sdl_ddr_ioregs = {
	/* SDCLK[0:1], CAS, RAS, Reset: Differential input, 40ohm */
	.dram_sdclk_0 = 0x00020030,
	.dram_sdclk_1 = 0x00020030,
	.dram_cas = 0x00020030,
	.dram_ras = 0x00020030,
	.dram_reset = 0x00020030,
	/* SDCKE[0:1]: 100k pull-up */
	.dram_sdcke0 = 0x00003000,
	.dram_sdcke1 = 0x00003000,
	/* SDBA2: pull-up disabled */
	.dram_sdba2 = 0x00000000,
	/* SDODT[0:1]: 100k pull-up, 40 ohm */
	.dram_sdodt0 = 0x00003030,
	.dram_sdodt1 = 0x00003030,
	/* SDQS[0:7]: Differential input, 40 ohm */
	.dram_sdqs0 = 0x00000030,
	.dram_sdqs1 = 0x00000030,
	.dram_sdqs2 = 0x00000030,
	.dram_sdqs3 = 0x00000030,
	.dram_sdqs4 = 0x00000030,
	.dram_sdqs5 = 0x00000030,
	.dram_sdqs6 = 0x00000030,
	.dram_sdqs7 = 0x00000030,

	/* DQM[0:7]: Differential input, 40 ohm */
	.dram_dqm0 = 0x00020030,
	.dram_dqm1 = 0x00020030,
	.dram_dqm2 = 0x00020030,
	.dram_dqm3 = 0x00020030,
	.dram_dqm4 = 0x00020030,
	.dram_dqm5 = 0x00020030,
	.dram_dqm6 = 0x00020030,
	.dram_dqm7 = 0x00020030,
};

/* configure MX6SOLO/DUALLITE mmdc GRP io registers */
struct mx6sdl_iomux_grp_regs mx6sdl_grp_ioregs = {
	/* DDR3 */
	.grp_ddr_type = 0x000c0000,
	/* SDQS[0:7]: Differential input, 40 ohm */
	.grp_ddrmode_ctl = 0x00020000,
	/* disable DDR pullups */
	.grp_ddrpke = 0x00000000,
	/* ADDR[00:16], SDBA[0:1]: 40 ohm */
	.grp_addds = 0x00000030,
	/* CS0/CS1/SDBA2/CKE0/CKE1/SDWE: 40 ohm */
	.grp_ctlds = 0x00000030,
	/* DATA[00:63]: Differential input, 40 ohm */
	.grp_ddrmode = 0x00020000,
	.grp_b0ds = 0x00000030,
	.grp_b1ds = 0x00000030,
	.grp_b2ds = 0x00000030,
	.grp_b3ds = 0x00000030,
	.grp_b4ds = 0x00000030,
	.grp_b5ds = 0x00000030,
	.grp_b6ds = 0x00000030,
	.grp_b7ds = 0x00000030,
};

/* H5TQ2G63DFR */
static struct mx6_ddr3_cfg h5tq2g63dfr = {
	.mem_speed = 1600,
	.density = 2,
	.width = 16,
	.banks = 8,
	.rowaddr = 14,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};

/* H5TQ4G63AFR */
static struct mx6_ddr3_cfg h5tq4g63afr = {
	.mem_speed = 1600,
	.density = 4,
	.width = 16,
	.banks = 8,
	.rowaddr = 15,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};

/*
 * calibration - these are the various CPU/DDR3 combinations we support
 */

static struct mx6_mmdc_calibration mx6dq_128x64_mmdc_calib = {
	/* write leveling calibration determine */
	.p0_mpwldectrl0 = 0x001A001C,
	.p0_mpwldectrl1 = 0x0027001E,
	.p1_mpwldectrl0 = 0x00130024,
	.p1_mpwldectrl1 = 0x000E001D,
	/* Read DQS Gating calibration */
	.p0_mpdgctrl0 = 0x03340348,
	.p0_mpdgctrl1 = 0x0334032C,
	.p1_mpdgctrl0 = 0x032C0340,
	.p1_mpdgctrl1 = 0x032C0268,
	/* Read Calibration: DQS delay relative to DQ read access */
	.p0_mprddlctl = 0x40343C3C,
	.p1_mprddlctl = 0x3E383246,
	/* Write Calibration: DQ/DM delay relative to DQS write access */
	.p0_mpwrdlctl = 0x34363E38,
	.p1_mpwrdlctl = 0x42303E34,
};

static struct mx6_mmdc_calibration mx6sdl_128x64_mmdc_calib = {
	/* write leveling calibration determine */
	.p0_mpwldectrl0 = 0x00430046,
	.p0_mpwldectrl1 = 0x003C0041,
	.p1_mpwldectrl0 = 0x00220027,
	.p1_mpwldectrl1 = 0x0029003B,
	/* Read DQS Gating calibration */
	.p0_mpdgctrl0 = 0x023C023C,
	.p0_mpdgctrl1 = 0x022C022C,
	.p1_mpdgctrl0 = 0x02200228,
	.p1_mpdgctrl1 = 0x020C0214,
	/* Read Calibration: DQS delay relative to DQ read access */
	.p0_mprddlctl = 0x46484C48,
	.p1_mprddlctl = 0x46484A44,
	/* Write Calibration: DQ/DM delay relative to DQS write access */
	.p0_mpwrdlctl = 0x342E282E,
	.p1_mpwrdlctl = 0x3434302C,
};

static struct mx6_mmdc_calibration mx6sdl_256x32_mmdc_calib = {
	/* write leveling calibration determine */
	.p0_mpwldectrl0 = 0x00430047,
	.p0_mpwldectrl1 = 0x0038003F,
	/* Read DQS Gating calibration */
	.p0_mpdgctrl0 = 0x02380240,
	.p0_mpdgctrl1 = 0x022C022C,
	/* Read Calibration: DQS delay relative to DQ read access */
	.p0_mprddlctl = 0x42464848,
	/* Write Calibration: DQ/DM delay relative to DQS write access */
	.p0_mpwrdlctl = 0x34342C2E,
};

static void ar6mxcs_spl_dram_init(void)
{
	int width = 64;
	struct mx6_ddr3_cfg *mem = NULL;
	struct mx6_mmdc_calibration *calib = NULL;
	struct mx6_ddr_sysinfo sysinfo = {
		/* width of data bus:0=16,1=32,2=64 */
		.dsize = width/32,
		/* config for full 4GB range so that get_mem_size() works */
		.cs_density = 32, /* 32Gb per CS */
		/* single chip select */
		.ncs = 1,
		.cs1_mirror = 0,
		.rtt_wr = 1 /*DDR3_RTT_60_OHM*/,	/* RTT_Wr = RZQ/4 */
#ifdef RTT_NOM_120OHM
		.rtt_nom = 2 /*DDR3_RTT_120_OHM*/,	/* RTT_Nom = RZQ/2 */
#else
		.rtt_nom = 1 /*DDR3_RTT_60_OHM*/,	/* RTT_Nom = RZQ/4 */
#endif
		.walat = 1,	/* Write additional latency */
		.ralat = 5,	/* Read additional latency */
		.mif3_mode = 3,	/* Command prediction working mode */
		.bi_on = 1,	/* Bank interleaving enabled */
		.sde_to_rst = 0x10,	/* 14 cycles, 200us (JEDEC default) */
		.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
	};

	/*
	 * MMDC Calibration requires the following data:
	 *   mx6_mmdc_calibration - board-specific calibration (routing delays)
	 *      these calibration values depend on board routing, SoC, and DDR
	 *   mx6_ddr_sysinfo - board-specific memory architecture (width/cs/etc)
	 *   mx6_ddr_cfg - chip specific timing/layout details
	 */
	switch (get_cpu_type()) {
	case MXC_CPU_MX6SOLO:
		width = 32;
		sysinfo.dsize = 1;
		mem = &h5tq4g63afr;
		calib = &mx6sdl_256x32_mmdc_calib;
		debug("2gB density\n");
		break;
	case MXC_CPU_MX6DL:
		mem = &h5tq2g63dfr;
		calib = &mx6sdl_128x64_mmdc_calib;
		debug("2gB density\n");
		break;
	case MXC_CPU_MX6D:
	case MXC_CPU_MX6Q:
		mem = &h5tq2g63dfr;
		calib = &mx6dq_128x64_mmdc_calib;
		debug("2gB density\n");
		break;
	}

	if (!mem) {
		puts("Error: Invalid Memory Configuration\n");
		hang();
	}
	if (!calib) {
		puts("Error: Invalid Board Calibration Configuration\n");
		hang();
	}

	if (is_cpu_type(MXC_CPU_MX6Q))
		mx6dq_dram_iocfg(width, &mx6dq_ddr_ioregs,
				 &mx6dq_grp_ioregs);
	else
		mx6sdl_dram_iocfg(width, &mx6sdl_ddr_ioregs,
				  &mx6sdl_grp_ioregs);
	mx6_dram_cfg(&sysinfo, calib, mem);
}

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP | \
			PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | \
				PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

iomux_v3_cfg_t const uart1_pads[] = {
	IOMUX_PADS(PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

iomux_v3_cfg_t const uart4_pads[] = {
	IOMUX_PADS(PAD_KEY_COL0__UART4_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_ROW0__UART4_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

static void ar6mxcs_setup_uart(void)
{
	SETUP_IOMUX_PADS(uart1_pads);
	SETUP_IOMUX_PADS(uart4_pads);
}

#ifdef CONFIG_FSL_ESDHC
#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

static iomux_v3_cfg_t const usdhc_pads[] = {
	IOMUX_PADS(PAD_SD3_CLK__SD3_CLK    | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_CMD__SD3_CMD    | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
  /* CD */
	IOMUX_PADS(PAD_SD3_DAT5__GPIO7_IO00    | MUX_PAD_CTRL(NO_PAD_CTRL)),

	IOMUX_PADS(PAD_SD4_CLK__SD4_CLK    | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_CMD__SD4_CMD    | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

void ar6mxcs_set_usdhc_iomux(void)
{
	SETUP_IOMUX_PADS(usdhc_pads);
}
#endif

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0x00C03F3F, &ccm->CCGR0);
	writel(0x0030FC03, &ccm->CCGR1);
	writel(0x0FFFC000, &ccm->CCGR2);
	writel(0x3FF00000, &ccm->CCGR3);
	writel(0x00FFF300, &ccm->CCGR4);
	writel(0x0F0000C3, &ccm->CCGR5);
	writel(0x000003FF, &ccm->CCGR6);
}

static void gpr_init(void)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	/* enable AXI cache for VDOA/VPU/IPU */
	writel(0xF00000CF, &iomux->gpr[4]);
	/* set IPU AXI-id0 Qos=0xf(bypass) AXI-id1 Qos=0x7 */
	writel(0x007F007F, &iomux->gpr[6]);
	writel(0x007F007F, &iomux->gpr[7]);
}

void board_init_f(ulong dummy)
{
	gd = &gdata;
	/*
	 * We don't use DMA in SPL, but we do need it in U-Boot. U-Boot
	 * initializes DMA very early (before all board code), so the only
	 * opportunity we have to initialize APBHDMA clocks is in SPL.
	 */
	arch_cpu_init();
	ccgr_init();
	gpr_init();
	timer_init();
	ar6mxcs_setup_uart();
	get_clocks();
	preloader_console_init();
	gpio_direction_output(AR6MXCS_GREEN_LED1, 0);
	ar6mxcs_spl_dram_init();
	gpio_direction_output(AR6MXCS_GREEN_LED2, 0);

	memset(__bss_start, 0, __bss_end - __bss_start);
	board_init_r(NULL, 0);
}

#ifdef CONFIG_SPL_MMC_SUPPORT
static struct fsl_esdhc_cfg usdhc_cfg = {
	.esdhc_base = USDHC3_BASE_ADDR,
	.max_bus_width = 4,
};

int board_mmc_getcd(struct mmc *mmc)
{
	/* Card Detect */
	return 1;
}

int board_mmc_init(bd_t *bis)
{
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	unsigned reg = readl(&psrc->sbmr1) >> 11;

	ar6mxcs_set_usdhc_iomux();

	/*
	 * Upon reading BOOT_CFG register the following map is done:
	 * Bit 11 and 12 of BOOT_CFG register can determine the current
	 * mmc port
	 * 0x2                  SD3
	 * 0x3                  SD4
	 */

	switch (reg & 0x3) {
	case 0x2:
		usdhc_cfg.esdhc_base = USDHC3_BASE_ADDR;
		usdhc_cfg.sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		break;
	case 0x3:
		usdhc_cfg.esdhc_base = USDHC4_BASE_ADDR;
		usdhc_cfg.sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
		usdhc_cfg.max_bus_width = 8;
		break;
	}


	return fsl_esdhc_initialize(bis, &usdhc_cfg);
}
#endif
