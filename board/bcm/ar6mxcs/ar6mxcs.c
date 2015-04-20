/*
 * Board functions for BCM AR6MXCS board
 *
 * Copyright (C) 2013, BCM Advanced Research
 *
 * Author: Frodo Lai <frodo_lai@bcmcom.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/video.h>
#include <i2c.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>

DECLARE_GLOBAL_DATA_PTR;

#define AR6MXCS_ENET_RST  IMX_GPIO_NR(1, 27)
#define AR6MXCS_CLK125_EN IMX_GPIO_NR(6, 24)
#define AR6MXCS_SD3_CD IMX_GPIO_NR(7, 0)
#define AR6MXCS_LVDS0_PWR     IMX_GPIO_NR(1, 11)
#define AR6MXCS_BL0_PWR     IMX_GPIO_NR(1, 15)
#define AR6MXCS_BL0_EN     IMX_GPIO_NR(1, 13)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define BUTTON_PAD_CTRL (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define WEAK_PULLUP	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_SRE_SLOW)

#define WEAK_PULLDOWN	(PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define OUTPUT_40OHM (PAD_CTL_SPEED_MED|PAD_CTL_DSE_40ohm)

/* I2C1, CSI, Audio Codec, Mini PCIe, DS1307 */
I2C_PADS(i2c0_pads,
	 PAD_CSI0_DAT9__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
	 PAD_CSI0_DAT9__GPIO5_IO27 | MUX_PAD_CTRL(I2C_PAD_CTRL),
	 IMX_GPIO_NR(5, 27),
	 PAD_CSI0_DAT8__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
	 PAD_CSI0_DAT8__GPIO5_IO26 | MUX_PAD_CTRL(I2C_PAD_CTRL),
	 IMX_GPIO_NR(5, 26));

/* I2C2, DSI, LVDS0, LVDS0_TH, HDMI */
I2C_PADS(i2c1_pads,
	 PAD_KEY_COL3__I2C2_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
	 PAD_KEY_COL3__GPIO4_IO12 | MUX_PAD_CTRL(I2C_PAD_CTRL),
	 IMX_GPIO_NR(4, 12),
	 PAD_KEY_ROW3__I2C2_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
	 PAD_KEY_ROW3__GPIO4_IO13 | MUX_PAD_CTRL(I2C_PAD_CTRL),
	 IMX_GPIO_NR(4, 13));

/* I2C3, LVDS1 & MPCIe */
I2C_PADS(i2c2_pads,
	 PAD_GPIO_3__I2C3_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
	 PAD_GPIO_3__GPIO1_IO03 | MUX_PAD_CTRL(I2C_PAD_CTRL),
	 IMX_GPIO_NR(1, 3),
	 PAD_GPIO_6__I2C3_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
	 PAD_GPIO_6__GPIO1_IO06 | MUX_PAD_CTRL(I2C_PAD_CTRL),
	 IMX_GPIO_NR(1, 6));

/* ENET */
iomux_v3_cfg_t const enet_pads1[] = {
	IOMUX_PADS(PAD_ENET_MDIO__ENET_MDIO  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_MDC__ENET_MDC    | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TXC__RGMII_TXC  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD0__RGMII_TD0  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD1__RGMII_TD1  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD2__RGMII_TD2  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD3__RGMII_TD3  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TX_CTL__RGMII_TX_CTL |
		   MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_REF_CLK__ENET_TX_CLK |
		   MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RXC__RGMII_RXC  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD0__RGMII_RD0  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD1__RGMII_RD1  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD2__RGMII_RD2  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD3__RGMII_RD3  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RX_CTL__GPIO6_IO24 |
		   MUX_PAD_CTRL(NO_PAD_CTRL)),
	/* PHY nRST */
	IOMUX_PADS(PAD_ENET_RXD0__GPIO1_IO27 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};
static iomux_v3_cfg_t enet_pads2[] = {
	IOMUX_PADS(PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static void setup_iomux_enet(void)
{
	SETUP_IOMUX_PADS(enet_pads1);
	/* phy reset: gpio1-27 */
	gpio_direction_output(AR6MXCS_ENET_RST, 0);
  /* Straping CLK125_EN */
	gpio_direction_output(AR6MXCS_CLK125_EN, 1);
	mdelay(50);

	gpio_direction_output(AR6MXCS_ENET_RST, 1);
	/* Set to final signal */
	SETUP_IOMUX_PADS(enet_pads2);
}

static void setup_iomux_uart(void)
{
}

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
  /* There is no CD on eMMC (SD4), assume always present */
	if (cfg->esdhc_base == USDHC3_BASE_ADDR)
		return !gpio_get_value(AR6MXCS_SD3_CD);
	else
		return 1;
}

int board_mmc_getwp(struct mmc *mmc)
{
  /* There is no WP on uSD (SD3) and eMMC (SD4), assume always present */
		return 0;
}

int board_mmc_init(bd_t *bis)
{
	int ret;
	u32 index = 0;
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	unsigned reg = readl(&psrc->sbmr1) >> 11;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

	usdhc_cfg[0].max_bus_width = 4;
	usdhc_cfg[1].max_bus_width = 8;

	/* SD3 card-detect */
	gpio_direction_input(AR6MXCS_SD3_CD);

	switch (reg & 0x3) {
	case 0x2:
		for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
			ret = fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
			if (ret)
				return ret;
		}
		break;
	case 0x3:
		for (index = CONFIG_SYS_FSL_USDHC_NUM - 1; index >= 0; --index) {
			ret = fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
			if (ret)
				return ret;
		}
		break;
	}

	return 0;
}
#endif

#if defined(CONFIG_VIDEO_IPUV3)

static iomux_v3_cfg_t lvds_pads[] = {
	IOMUX_PADS(PAD_SD2_CMD__GPIO1_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT0__GPIO1_IO15 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT2__GPIO1_IO13 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg = readl(&iomux->gpr[2]);

	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
		 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);

	writel(reg, &iomux->gpr[2]);
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
	disable_lvds(dev);
	imx_enable_hdmi_phy();
}

static int detect_i2c(struct display_info_t const *dev)
{
	return i2c_set_bus_num(dev->bus) == 0 &&
		i2c_probe(dev->addr) == 0;
}

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)
				IOMUXC_BASE_ADDR;

	/* set CH0 data width to 24bit (IOMUXC_GPR2:5 0=18bit, 1=24bit) */
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT;
	writel(reg, &iomux->gpr[2]);

	/* Enable LVDS0_PWR, BL0_PWR and BL0_EN */
	imx_iomux_v3_setup_multiple_pads(lvds_pads, ARRAY_SIZE(lvds_pads));
	gpio_direction_output(AR6MXCS_LVDS0_PWR, 1);
	gpio_direction_output(AR6MXCS_BL0_PWR, 1);
	gpio_direction_output(AR6MXCS_BL0_EN, 1);
}

struct display_info_t const displays[] = {{
	/* HDMI Output */
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_hdmi,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 2,
	.addr	= 0x50,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_i2c,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "wsvga-lvds",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 600,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} } };
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	enable_ipu_clock();
	imx_setup_hdmi();
	/* Turn on LDB0,IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |= MXC_CCM_CCGR3_LDB_DI0_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 |MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3<<MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      |(3<<MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<<MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     |IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
	     |IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     |IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     |IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     |IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
	     |IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~IOMUXC_GPR3_LVDS0_MUX_CTL_MASK)
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       <<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif /* CONFIG_VIDEO_IPUV3 */

int mx6_rgmii_rework(struct phy_device *phydev)
{
	/* RX Data Pad Skew Register */
	ksz9031_phy_extended_write(phydev, 0x02, MII_KSZ9031_EXT_RGMII_RX_DATA_SKEW,
			MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x7777);

  /* TX Data Pad Skew Register */
	ksz9031_phy_extended_write(phydev, 0x02, MII_KSZ9031_EXT_RGMII_TX_DATA_SKEW,
			MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x7777);

  /* Clock Pad Skew Register */
	ksz9031_phy_extended_write(phydev, 0x02, MII_KSZ9031_EXT_RGMII_CLOCK_SKEW,
			MII_KSZ9031_MOD_DATA_NO_POST_INC, 0x7FFF);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	mx6_rgmii_rework(phydev);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

int board_eth_init(bd_t *bis)
{
	uint32_t base = IMX_FEC_BASE;
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;
	int ret;

	setup_iomux_enet();

#ifdef CONFIG_FEC_MXC
	bus = fec_get_miibus(base, -1);
	if (!bus)
		return 0;
	/* scan phy 4,5,6,7 */
	phydev = phy_find_by_mask(bus, (0xf), PHY_INTERFACE_MODE_RGMII);
	if (!phydev)
		return 0;

	printf("using phy at %d\n", phydev->addr);
	ret  = fec_probe(bis, -1, base, bus, phydev);
	if (ret)
		printf("FEC MXC: %s:failed\n", __func__);

#endif
	return 0;
}

void dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[1].start = PHYS_SDRAM_2;

	switch (gd->ram_size) {
	case 0x20000000: /* DDR_32BIT_512MB */
		gd->bd->bi_dram[0].size = 0x20000000;
		gd->bd->bi_dram[1].size = 0;
		break;
	case 0x40000000:
		gd->bd->bi_dram[0].size = 0x40000000;
		gd->bd->bi_dram[1].size = 0;
		break;
	case 0x80000000: /* DDR_64BIT_2GB */
		gd->bd->bi_dram[0].size = 0x80000000;
		gd->bd->bi_dram[1].size = 0;
		break;
	case 0xEFF00000: /* DDR_64BIT_4GB */
		gd->bd->bi_dram[0].size = 0x70000000;
		gd->bd->bi_dram[1].size = 0x7FF00000;
		break;
	}
}

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	switch (gd->ram_size) {
	case 0x20000000:
	case 0x40000000:
	case 0x80000000:
		break;
	case 0xF0000000:
		gd->ram_size -= 0x100000;
		break;
	default:
		printf("ERROR: Unsupported DRAM size 0x%lx\n", gd->ram_size);
		return -1;
	}

	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_early_init_f(void)
{
	setup_iomux_uart();

#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif
	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, I2C_PADS_INFO(i2c0_pads));
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, I2C_PADS_INFO(i2c1_pads));
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, I2C_PADS_INFO(i2c2_pads));

	return 0;
}

int checkboard(void)
{
	puts("Board: AR6MXCS\n");

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{"mmc1",	MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},
	{NULL,		0},
};
#endif

int board_late_init(void)
{
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	unsigned reg = readl(&psrc->sbmr1) >> 11;

	switch (reg & 0x3) {
	case 0x2:
		setenv("mmcdev", "0");
		break;
	case 0x3:
		setenv("mmcdev", "0");
		break;
	}

	setenv("cpu", get_imx_type(get_cpu_type()));
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	return 0;
}
