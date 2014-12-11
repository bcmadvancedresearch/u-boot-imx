/*
 * Board functions for BCM AR6MX board
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

#define AR6MX_ENET_RST  IMX_GPIO_NR(1, 25)
#define AR6MX_SPI_BUS3_CS0  IMX_GPIO_NR(4, 24)
#define AR6MX_CLK125_EN IMX_GPIO_NR(6, 24)
#define AR6MX_SD3_CD IMX_GPIO_NR(7, 0)
#define AR6MX_SD3_WP IMX_GPIO_NR(7, 1)

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
	IOMUX_PADS(PAD_ENET_CRS_DV__GPIO1_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};
static iomux_v3_cfg_t enet_pads2[] = {
	IOMUX_PADS(PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static void setup_iomux_enet(void)
{
	SETUP_IOMUX_PADS(enet_pads1);
	/* phy reset: gpio1-25 */
	gpio_direction_output(AR6MX_ENET_RST, 0);
  /* Straping CLK125_EN */
	gpio_direction_output(AR6MX_CLK125_EN, 1);
	mdelay(50);

	gpio_direction_output(AR6MX_ENET_RST, 1);
	/* Set to final signal */
	SETUP_IOMUX_PADS(enet_pads2);
}

static void setup_iomux_uart(void)
{
}

#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 2 && cs == 0) ? (AR6MX_SPI_BUS3_CS0) : -1;
}
#endif

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
		return !gpio_get_value(AR6MX_SD3_CD);
	else
		return 1;
}

int board_mmc_getwp(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
  /* There is no WP on eMMC (SD4), assume always present */
	if (cfg->esdhc_base == USDHC3_BASE_ADDR)
		return gpio_get_value(AR6MX_SD3_WP);
	else
		return 1;
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	u32 index = 0;

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

	usdhc_cfg[0].max_bus_width = 4;
	usdhc_cfg[1].max_bus_width = 8;

	/* SD3 write-protect and card-detect */
	gpio_direction_input(AR6MX_SD3_WP);
	gpio_direction_input(AR6MX_SD3_CD);

	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
		if (status)
			return status;
	}

	return status;
}
#endif

int mx6_rgmii_rework(struct phy_device *phydev)
{
	/* RX Data Pad Skew Register */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x0002);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x0005);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0xc002);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x7777);

  /* TX Data Pad Skew Register */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x0002);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x0006);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0xc002);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x7777);

  /* Clock Pad Skew Register */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x0002);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x0008);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0xc002);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x7fff);

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
	puts("Board: AR6MX\n");

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
	setenv("cpu", get_imx_type(get_cpu_type()));
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	return 0;
}
