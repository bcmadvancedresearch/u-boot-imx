/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2014 Jasbir Matharu
 *
 * Support for BCM AR6MX Board
 *
 * SPDX-License-Identifier:	GPL-2.0+
 *
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <malloc.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/sata.h>
#include <asm/imx-common/boot_mode.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <spi_flash.h>

// recovery header to avoid implicit function Warning
#include <recovery.h>

// Device-tree support headers
#include <libfdt.h>
#include <fdt_support.h>

#define MACH_TYPE_AR6MX	4517
#define USB_OTG_PWR IMX_GPIO_NR(1, 7)
#define USB_V1_POWER IMX_GPIO_NR(5, 13)
#define USB_V2_POWER IMX_GPIO_NR(5, 14)

#define AR6MX_ENET_RST  IMX_GPIO_NR(1, 25)
#define AR6MX_CLK125_EN IMX_GPIO_NR(6, 24)

#define AR6MX_SD3_CD IMX_GPIO_NR(7, 0)
#define AR6MX_SD3_WP IMX_GPIO_NR(7, 1)

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL (PAD_CTL_PUS_100K_UP | \
		PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |	\
		PAD_CTL_SRE_FAST | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |                    \
        PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |                 \
        PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL (PAD_CTL_PUS_100K_UP | \
		PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS | \
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define USDHC3_CD_GPIO IMX_GPIO_NR(7, 0)

/* Declare version gpio pins n*/
static iomux_v3_cfg_t board_ver_pads[] = {
  IOMUX_PADS(PAD_DISP0_DAT4__GPIO4_IO25),
  IOMUX_PADS(PAD_DISP0_DAT5__GPIO4_IO26),
  IOMUX_PADS(PAD_DISP0_DAT6__GPIO4_IO27),
  IOMUX_PADS(PAD_DISP0_DAT7__GPIO4_IO28),
};

// PDi Transistor-Transistor Logic (TTL) Setup CN5 GPIO Pad
#define AR6MX_TTL_DI0	IMX_GPIO_NR(2, 0)
#define AR6MX_TTL_DI1	IMX_GPIO_NR(2, 1)
#define AR6MX_TTL_DI2	IMX_GPIO_NR(2, 2)
#define AR6MX_TTL_DI3	IMX_GPIO_NR(2, 3)
#define AR6MX_TTL_DI4	IMX_GPIO_NR(2, 4)
#define AR6MX_TTL_DI5	IMX_GPIO_NR(2, 5)
#define AR6MX_TTL_DO0	IMX_GPIO_NR(2, 6)
#define AR6MX_TTL_DO1	IMX_GPIO_NR(2, 7)

// PDi Human readable Names for TTL GPIO Setup
#define AR6MX_TV_POWER_REQ            AR6MX_TTL_DI0
#define AR6MX_TV_ARROW_UP             AR6MX_TTL_DI1
#define AR6MX_AIO_VOL_UP              AR6MX_TTL_DI1
#define AR6MX_TV_ARROW_DOWN           AR6MX_TTL_DI2
#define AR6MX_AIO_VOL_DOWN            AR6MX_TTL_DI2
#define AR6MX_TV_ARROW_LEFT           AR6MX_TTL_DI3 // Arrow Left Not Available on P19 AIO Front Panel
#define AR6MX_TV_OR_AIO               AR6MX_TTL_DI5 // float high for TV/Tab, pull low for all-in-one -JTS
#define AR6MX_ANDROID_PWRSTATE        AR6MX_TTL_DO0
#define AR6MX_INTERNAL_SPK_ENABLE     AR6MX_TTL_DO1

// PDi mrobbeloth Board version GPIO
#define AR6MX_VER_B0 	IMX_GPIO_NR(4, 25)
#define AR6MX_VER_B1 	IMX_GPIO_NR(4, 26)
#define AR6MX_VER_B2 	IMX_GPIO_NR(4, 27)
#define AR6MX_VER_B3 	IMX_GPIO_NR(4, 28)

static iomux_v3_cfg_t const uart1_pads[] = {
	IOMUX_PADS(PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

static iomux_v3_cfg_t const uart4_pads[] = {
	IOMUX_PADS(PAD_KEY_COL0__UART4_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_ROW0__UART4_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

static iomux_v3_cfg_t const usdhc3_pads[] = {
	IOMUX_PADS(PAD_SD3_CLK__SD3_CLK      | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_CMD__SD3_CMD      | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT0__SD3_DATA0   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT1__SD3_DATA1   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT2__SD3_DATA2   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT3__SD3_DATA3   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT5__GPIO7_IO00  | MUX_PAD_CTRL(NO_PAD_CTRL)), /* CD PIN */

};

static iomux_v3_cfg_t const usdhc4_pads[] = {
	IOMUX_PADS(PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

static iomux_v3_cfg_t const gpio_pads[] = {
	IOMUX_PADS(PAD_NANDF_D0__GPIO2_IO00 | MUX_PAD_CTRL(PAD_CTL_PUS_100K_UP | PAD_CTL_HYS | PAD_CTL_SPEED_HIGH | PAD_CTL_DSE_40ohm)),
	IOMUX_PADS(PAD_NANDF_D1__GPIO2_IO01 | MUX_PAD_CTRL(PAD_CTL_PUS_100K_UP | PAD_CTL_HYS | PAD_CTL_SPEED_HIGH | PAD_CTL_DSE_40ohm)),
	IOMUX_PADS(PAD_NANDF_D2__GPIO2_IO02 | MUX_PAD_CTRL(PAD_CTL_PUS_100K_UP | PAD_CTL_HYS | PAD_CTL_SPEED_HIGH | PAD_CTL_DSE_40ohm)),
	IOMUX_PADS(PAD_NANDF_D3__GPIO2_IO03 | MUX_PAD_CTRL(PAD_CTL_PUS_100K_UP | PAD_CTL_HYS | PAD_CTL_SPEED_HIGH | PAD_CTL_DSE_40ohm)),
	IOMUX_PADS(PAD_NANDF_D4__GPIO2_IO04 | MUX_PAD_CTRL(PAD_CTL_PUS_100K_UP | PAD_CTL_HYS | PAD_CTL_SPEED_HIGH | PAD_CTL_DSE_40ohm)),
	IOMUX_PADS(PAD_NANDF_D5__GPIO2_IO05 | MUX_PAD_CTRL(PAD_CTL_PUS_100K_UP | PAD_CTL_HYS | PAD_CTL_SPEED_HIGH | PAD_CTL_DSE_40ohm)),
	IOMUX_PADS(PAD_NANDF_D6__GPIO2_IO06 | MUX_PAD_CTRL(PAD_CTL_PUS_100K_UP | PAD_CTL_HYS | PAD_CTL_SPEED_HIGH | PAD_CTL_DSE_40ohm)),
	IOMUX_PADS(PAD_NANDF_D7__GPIO2_IO07 | MUX_PAD_CTRL(PAD_CTL_PUS_100K_UP | PAD_CTL_HYS | PAD_CTL_SPEED_HIGH | PAD_CTL_DSE_40ohm)),
};

enum MODEL_TYPE {
   NONE = -1,
   AIO = 0,
   TV = 1
};

static enum MODEL_TYPE mt = NONE;

/* string replace code from user chantra on:
   http://coding.debuntu.org/c-implementing-str_replace-replace-all-occurrences-substring */
char *
str_replace ( const char *string, const char *substr, const char *replacement ){
  char *tok = NULL;
  char *newstr = NULL;
  char *oldstr = NULL;
  char *head = NULL;

  /* if either substr or replacement is NULL, duplicate string a let caller handle it */
  if ( substr == NULL || replacement == NULL ) return strdup (string);
  newstr = strdup (string);
  head = newstr;
  while ( (tok = strstr ( head, substr ))){
    oldstr = newstr;
    newstr = malloc ( strlen ( oldstr ) - strlen ( substr ) + strlen ( replacement ) + 1 );
    /*failed to alloc mem, free old string and return NULL */
    if ( newstr == NULL ){
      free (oldstr);
      return NULL;
    }
    memcpy ( newstr, oldstr, tok - oldstr );
    memcpy ( newstr + (tok - oldstr), replacement, strlen ( replacement ) );
    memcpy ( newstr + (tok - oldstr) + strlen( replacement ), tok + strlen ( substr ), strlen ( oldstr ) - strlen ( substr ) - ( tok - oldstr ) );
    memset ( newstr + strlen ( oldstr ) - strlen ( substr ) + strlen ( replacement ) , 0, 1 );
    /* move back head right after the last replacement */
    head = newstr + (tok - oldstr) + strlen( replacement );
    free (oldstr);
  }
  return newstr;
}

int ar6mx_board_version(void) {
  int b3 = 0;
  int b2 = 0;
  int b1 = 0;
  int b0 = 0;
  int ret = 0;

  /* read board version registers connected to gpio pins and
     DISP0_DAT4 on i.MX6Q/DL - DISP; CSI */
  b3 = gpio_get_value(AR6MX_VER_B3);
  b2 = gpio_get_value(AR6MX_VER_B2);
  b1 = gpio_get_value(AR6MX_VER_B1);
  b0 = gpio_get_value(AR6MX_VER_B0);
  ret |= (b3 << 3);
  ret |= (b2 << 2);
  ret |= (b1 << 1);
  ret |= (b0 << 0);

  /* print board version in serial console */
  if (ret == 15) {
     printf("Board Version: %x%x%x%x or 0x%x (TAB Solo)\n",
            b3, b2, b1, b0, ret);
  }
  else if (ret == 1) {
     printf("Board Version: %x%x%x%x or 0x%x (TAB2 Quad)\n",
            b3, b2, b1, b0, ret);
  }
  else {
     printf("Board Version: %x%x%x%x or 0x%x (Unknown)\n",
            b3, b2, b1, b0, ret);
  }

  /* update bootargs with board version, could be done in
     kernel board file, but okay here too */
  char* cmdline = getenv("bootargs_base");
  char* cmdline_a = (char *) malloc(strlen(cmdline) + 24);
  sprintf(cmdline_a, "%s board_version=%x%x%x%x ", cmdline, b3, b2, b1, b0);
  setenv("bootargs_base", cmdline_a);
  free(cmdline_a);
  return ret;
}

void ar6mx_tv_or_aio_reporting(void) {

   /* Report the model class (type)
      P14TAB, Standard Module, SW Module for TV (high input)
      AIO (P19A), M-Series TV, or Universal Module for AIO (low input) */
   printf("PDi Model Class: ");
   if (gpio_get_value(AR6MX_TV_OR_AIO)) {
      printf("TV (TAB)\n");
      mt = TV;
   }
   else {
      printf("AIO\n");
      mt = AIO;
   }

   /* update bootargs with model type  */
   char* cmdline = getenv("bootargs_base");
   char* cmdline_a = (char *) malloc(strlen(cmdline) + 20);
   switch (mt) {
      case NONE: 
         sprintf(cmdline_a, "%s model_type=%s ", cmdline, "NONE");
         break;
      case AIO: 
         sprintf(cmdline_a, "%s model_type=%s ", cmdline, "AIO");
         break;
      case TV:
         sprintf(cmdline_a, "%s model_type=%s ", cmdline, "TV");
         break;
      default:
         sprintf(cmdline_a, "%s model_type=%s ", cmdline, "UNDEFINED");
 
   }
   setenv("bootargs_base", cmdline_a);
   free(cmdline_a);

}

int dram_init(void)
{
	/*
	gd->ram_size = (phys_size_t)CONFIG_DDR_MB * 1024 * 1024;
	return 0;
	*/
	
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

static void setup_iomux_uart(void)
{
	SETUP_IOMUX_PADS(uart1_pads);
	SETUP_IOMUX_PADS(uart4_pads);
}

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
                return 0;
}

int board_mmc_init(bd_t *bis)
{
        int ret;
        u32 index = 0;

        usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
        usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);

        usdhc_cfg[0].max_bus_width = 4;
        usdhc_cfg[1].max_bus_width = 8;

        /* SD3 write-protect and card-detect */
        gpio_direction_input(AR6MX_SD3_WP);
        gpio_direction_input(AR6MX_SD3_CD);

        for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
                ret = fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
                if (ret)
                        return ret;
        }

        return 0;
}

int mmc_get_env_devno(void)
{
        /* Note info on this register in section 60.7.2
           SRC Boot Mode Register 1 (SRC_SBMR1)
           contains bits that reflect status of Boot Mode
           Pins of the chip, reset depends on pads
           Address: 20D_8000h base + 4h offset = 20D_8004h */
	u32 soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
	u32 dev_no;
	u32 bootsel;

        /* BOOT_CFG1[7:6] Boot Device Selection
           01b to from USDHC interfaces */
	bootsel = (soc_sbmr & 0x000000FF) >> 6 ;

	/* If not boot from sd/mmc, use default value */
	if (bootsel != 1)
		return CONFIG_SYS_MMC_ENV_DEV;

	/* BOOT_CFG2[3] (bit 11) and BOOT_CFG2[4] (bit 12) */
	dev_no = (soc_sbmr & 0x00001800) >> 11;

	/* need ubstract 1 to map to the mmc device id
	 * see the comments in board_mmc_init function
	 */

	dev_no=dev_no - 2;

	return dev_no;
}

int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no + 1;
}

static iomux_v3_cfg_t const enet_pads1[] = {

	IOMUX_PADS(PAD_ENET_MDIO__ENET_MDIO  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_MDC__ENET_MDC    | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TXC__RGMII_TXC  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD0__RGMII_TD0  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD1__RGMII_TD1  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD2__RGMII_TD2  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD3__RGMII_TD3  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TX_CTL__RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_REF_CLK__ENET_TX_CLK  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RXC__RGMII_RXC  | MUX_PAD_CTRL(ENET_PAD_CTRL)),

	IOMUX_PADS(PAD_RGMII_RD0__RGMII_RD0 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD1__RGMII_RD1 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD2__RGMII_RD2 | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD3__RGMII_RD3 | MUX_PAD_CTRL(ENET_PAD_CTRL)),

	/* CLK125_EN 125Mhz clockout enabled */
	IOMUX_PADS(PAD_RGMII_RX_CTL__GPIO6_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL)),

	IOMUX_PADS(PAD_ENET_CRS_DV__GPIO1_IO25  | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static iomux_v3_cfg_t const enet_pads2[] = {
	IOMUX_PADS(PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

int mx6_rgmii_rework(struct phy_device *phydev)
{

	/* RX Data Pad Skew Register */
	ksz9031_phy_extended_write(phydev, 0x02,MII_KSZ9031_EXT_RGMII_RX_DATA_SKEW,
		MII_KSZ9031_MOD_DATA_NO_POST_INC,0x7777);
	/* TX Data Pad Skew Register */
	ksz9031_phy_extended_write(phydev, 0x02,MII_KSZ9031_EXT_RGMII_TX_DATA_SKEW,
		MII_KSZ9031_MOD_DATA_NO_POST_INC,0x7777);
	/* Clock Pad Skew Register */
	ksz9031_phy_extended_write(phydev, 0x02,MII_KSZ9031_EXT_RGMII_CLOCK_SKEW,
		MII_KSZ9031_MOD_DATA_NO_POST_INC,0x7FFF);

	return 0;
}

static void setup_iomux_enet(void)
{
	SETUP_IOMUX_PADS(enet_pads1);

	/* phy reset: gpio1-25 */
	gpio_direction_output(AR6MX_ENET_RST, 0);
  /* Straping CLK125_EN */
	gpio_direction_output(AR6MX_CLK125_EN, 1);
	mdelay(50);

	gpio_direction_output(AR6MX_ENET_RST, 1);

	SETUP_IOMUX_PADS(enet_pads2);
};


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

	/** phy address can only range from 1-7 **/
	phydev = phy_find_by_mask(bus, 0x07 << CONFIG_FEC_MXC_PHYADDR,
		PHY_INTERFACE_MODE_RGMII);
	if (!phydev) {
		free(bus);
		return 0;
	}

	ret  = fec_probe(bis, -1, base, bus, phydev);
	if (ret) {
		printf("FEC MXC: %s:failed\n", __func__);
		free(phydev);
		free(bus);
	}
#endif
	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	mx6_rgmii_rework(phydev);
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

static void setup_gpios(void)
{
	printf("enter setup_gpios\n");
	SETUP_IOMUX_PADS(gpio_pads);

        /* Configure gpio on CN5 */
        gpio_direction_input(AR6MX_TV_POWER_REQ);
        gpio_direction_input(AR6MX_TV_OR_AIO);

        // Floats high for P14, pulled low for AIO P19A
        if (gpio_get_value(AR6MX_TV_OR_AIO)) {
           printf("setup_gpios(): TV Mode detected\n");
           gpio_direction_input(AR6MX_TV_ARROW_UP);
           gpio_direction_input(AR6MX_TV_ARROW_DOWN);
           gpio_direction_input(AR6MX_TV_ARROW_LEFT);
        }    
        else {
           printf("setup_gpios(): AIO Mode detected\n");
           gpio_direction_input(AR6MX_AIO_VOL_UP);
           gpio_direction_input(AR6MX_AIO_VOL_DOWN);
        }    
	gpio_direction_output(AR6MX_ANDROID_PWRSTATE, 1);
        gpio_direction_output(AR6MX_INTERNAL_SPK_ENABLE, 0);

	SETUP_IOMUX_PADS(board_ver_pads);
}

#if defined(CONFIG_VIDEO_IPUV3)
struct display_info_t {
	int	bus;
	int	addr;
	int	pixfmt;
	int	(*detect)(struct display_info_t const *dev);
	void	(*enable)(struct display_info_t const *dev);
	struct	fb_videomode mode;
};

static int detect_hdmi(struct display_info_t const *dev)
{
	struct hdmi_regs *hdmi	= (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
	return readb(&hdmi->phy_stat0) & HDMI_DVI_STAT;
}

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

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)
				IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT |
	       IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT;
	writel(reg, &iomux->gpr[2]);
}

struct display_info_t const displays[] = {
	{
        .bus    = -1,
        .addr   = 0,
        .pixfmt = IPU_PIX_FMT_LVDS666,
        .detect = NULL,
        .enable = enable_lvds,
        .mode   = {
                .name           = "AU0_G185XW01",
                .refresh        = 60,
                .xres           = 1366,
                .yres           = 768,
                .pixclock       = 12844,
                .left_margin    = 80,
                .right_margin   = 80,
                .upper_margin   = 13,
                .lower_margin   = 13,
                .hsync_len      = 80,
                .vsync_len      = 14,
        }
	},
  {
        .bus    = -1,
        .addr   = 0,
        .pixfmt = IPU_PIX_FMT_RGB24,
        .detect = detect_hdmi,
        .enable = do_enable_hdmi,
        .mode   = {
                .name           = "HDMI",
                .refresh        = 60,
                .xres           = 1280,
                .yres           = 720,
                .pixclock       = 15385,
                .left_margin    = 220,
                .right_margin   = 40,
                .upper_margin   = 21,
                .lower_margin   = 7,
                .hsync_len      = 60,
                .vsync_len      = 10,
                .sync           = FB_SYNC_EXT,
                .vmode          = FB_VMODE_NONINTERLACED
        }
  }
};

size_t display_count = ARRAY_SIZE(displays);

int ft_board_setup(void *blob, bd_t *bd) {

   // open the device tree
   void *fdt_location = (void *)getenv_hex("fdt_addr", 0);
   int fdtHdrChk = fdt_check_header(fdt_location);

   if ((fdt_location != 0) && (fdtHdrChk == 0)) {
      printf("ft_board_setup(): passed fdt_check_header\n");

   }
   else {
      printf("ft_board_setup(): Device tree header at 0x%x unverifiable, error %d\n",
             (unsigned int)fdt_location, fdtHdrChk);
      return 1;
   }

   // traditional p14 with tv and android
   if ((mt == TV) && (detect_hdmi(&displays[1]))) {
      printf("TV style device with HDMI display used\n");

      // for hdmi, flip fb1 and fb0 since android prioritizes display on fb0
      int offset = fdt_path_offset(fdt_location, "/fb@0");
      fdt_set_name(fdt_location, offset, "fb@ldb");
      offset = fdt_path_offset(fdt_location, "/fb@1");
      fdt_set_name(fdt_location, offset, "fb@0");
      fdt_status_okay(fdt_location, offset);
      offset = fdt_path_offset(fdt_location, "/fb@ldb");
      fdt_set_name(fdt_location, offset, "fb@1");
      offset = fdt_path_offset(fdt_location, "/fb@1");

      // disable lvds display nothing is connected to it
      fdt_status_disabled(fdt_location, offset);
   }
   // p14 or p19 with tv, android, and lvds mipi-csi2 board--aka tab3 level hw
   else if ((mt == TV) && (!detect_hdmi(&displays[1]))) {
      printf("TV style device with LVDS display most likely\n");
      //find the frame buffer node
      int offset = fdt_path_offset(fdt_location, "/fb@0");

      // adjust from RGB24 to LVDS666 
      int propres = fdt_setprop_string(fdt_location, offset,
                                          "interface_pix_fmt", "LVDS666");
      // check result
      if (propres == 0)  {
         printf("ft_board_setup(): fb@1 interface_pix_fmt now LVDS666\n");
      }
      else {
         printf("ft_board_setup(): fdt_setprop_inplace returned error %d\n",
                propres);
      }

      // propres = fdt_setprop_u32(fdt_location, offset, "default_bpp", 18);
  }
  // traditional p19 or similar with android only and lvds (no tv board)
  else if (mt == AIO) {
     printf("ft_board_setup(): no change for traditional AIO mode\n");
  }
  return 0;
}
/*
int board_video_skip(void)
{
	int i;
	int ret;
	char const *panel = getenv("panel");
	if (!panel) {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			struct display_info_t const *dev = displays+i;
			if (dev->detect && dev->detect(dev)) {
				panel = dev->mode.name;
				printf("auto-detected panel %s\n", panel);
				break;
			}
		}
		if (!panel) {
			panel = displays[0].mode.name;
            int fourcc = displays[0].pixfmt;
			printf("No panel detected: default to %s with pixel format 0x%x\n", 
                    panel, fourcc);
			i = 0;
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			if (!strcmp(panel, displays[i].mode.name))
				break;
		}
	}
	if (i < ARRAY_SIZE(displays)) {
		ret = ipuv3_fb_init(&displays[i].mode, 0,
				    displays[i].pixfmt);
		if (!ret) {
			displays[i].enable(displays+i);
			printf("Display: %s (%ux%u)\n",
			       displays[i].mode.name,
			       displays[i].mode.xres,
			       displays[i].mode.yres);
		} else
			printf("LCD %s cannot be configured: %d\n",
			       displays[i].mode.name, ret);
	} else {
		printf("unsupported panel %s\n", panel);
		return -EINVAL;
	}

	return 0;
}
*/
static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	enable_ipu_clock();
	imx_setup_hdmi();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     | IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED
	     | IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS1_MUX_CTL_MASK
			| IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif /* CONFIG_VIDEO_IPUV3 */

int board_early_init_f(void)
{
	setup_iomux_uart();
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif

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

int board_init(void)
{

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif

	printf("Calling setup_gpios()\n");
	setup_gpios();
        printf("Returned setup_gpios()\n");

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd3",  MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},
	{NULL,0},
};
#endif


int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	ar6mx_board_version();
	ar6mx_tv_or_aio_reporting();
	return 0;
}

int checkboard(void)
{
	puts("Board: BCM AR6MX\n");
	return 0;
}

void ldo_mode_set(int ldo_bypass)
{

}

#ifdef CONFIG_FASTBOOT

void board_fastboot_setup(void)
{
	printf("board_fastboot_setup\n");
	switch (get_boot_device()) {
#if defined(CONFIG_FASTBOOT_STORAGE_SATA)
	case SATA_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "sata");
		if (!getenv("bootcmd"))
			setenv("bootcmd", "boota sata");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_SATA*/
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD2_BOOT:
	case MMC2_BOOT:
	    if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc0");
	    if (!getenv("bootcmd"))
			setenv("bootcmd", "boota mmc0");
	    break;
	case SD3_BOOT:
	case MMC3_BOOT:
	    if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc0");
	    if (!getenv("bootcmd"))
			setenv("bootcmd", "run bootargs_base;boota mmc0");
	    break;
	case MMC4_BOOT:
	    if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc1");
	    if (!getenv("bootcmd"))
			setenv("bootcmd", "run bootargs_base;boota mmc1");
	    break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("unsupported boot devices\n");
		break;
	}

}
#endif

#ifdef CONFIG_ANDROID_RECOVERY

#define GPIO_VOL_DN_KEY IMX_GPIO_NR(1, 5)
iomux_v3_cfg_t const recovery_key_pads[] = {
	IOMUX_PADS(PAD_GPIO_5__GPIO1_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

int check_recovery_cmd_file(void)
{
          int button_pressed = 0;
          int recovery_mode = 0;

          u32 reg;

          recovery_mode =  recovery_check_and_clean_flag();

          reg = readl(GPIO2_BASE_ADDR + GPIO_GDIR);
          reg &= ~(1<<1 | 1<<2 | 1<<3);
          writel(reg, GPIO2_BASE_ADDR + GPIO_GDIR);
          reg = readl(GPIO2_BASE_ADDR + GPIO_PSR);

          if(!(reg & (1<<1 | 1<<2 | 1<<3))) {
                button_pressed = 1;
                printf("Recovery key pressed\n");
          }


          return recovery_mode || button_pressed;
}

#endif

#ifdef CONFIG_FASTBOOT

int check_fastboot_by_arrow_left_and_down(void)
{
          int button_pressed = 0;

          u32 reg;

          reg = readl(GPIO2_BASE_ADDR + GPIO_GDIR);
          reg &= ~(1<<1 | 1<<2 | 1<<3);
          writel(reg, GPIO2_BASE_ADDR + GPIO_GDIR);

          // Select fastboot if Arrows Left and Up are selected and Arrow Down is not selected
          reg = (readl(GPIO2_BASE_ADDR + GPIO_PSR) ^ (1<<2));

          if(!(reg & (1<<1 | 1<<2 | 1<<3))) {
                button_pressed = 1;
                printf("fastboot key pressed\n");
          }


          return button_pressed;
}

#endif

void board_recovery_setup(void)
{

	int bootdev = get_boot_device();

	switch (bootdev) {
#if defined(CONFIG_FASTBOOT_STORAGE_SATA)
	case SATA_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"boota sata recovery");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_SATA*/
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD2_BOOT:
	case MMC2_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"boota mmc0 recovery");
		break;
	case SD3_BOOT:
	case MMC3_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"boota mmc1 recovery");
		break;
	case MMC4_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"boota mmc1 recovery");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("Unsupported bootup device for recovery: dev: %d\n",
			bootdev);
		return;
	}


	printf("setup env for recovery..\n");

        /* PDi mrobbeloth, need to account for those lvds displays */
	setenv("bootcmd", "run bootargs_base;run bootcmd_android_recovery");
}

int check_key_pressing(void)
{
        return 0;
}

void setup_recovery_env(void)
{
        board_recovery_setup();
}

void check_recovery_mode(void)
{
        if (check_key_pressing()) {
                puts("Fastboot: Recovery key pressing got!\n");
                setup_recovery_env();
        } else if (check_recovery_cmd_file()) {
                puts("Fastboot: Recovery command file found!\n");
                setup_recovery_env();
        } else {
                puts("Fastboot: Normal\n");
        }
}

//#endif /*CONFIG_ANDROID_RECOVERY*/

//#endif /*CONFIG_FASTBOOT*/

#ifdef CONFIG_IMX_UDC
iomux_v3_cfg_t const otg_udc_pads[] = {
	IOMUX_PADS(PAD_GPIO_1__USB_OTG_ID | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_GPIO_7__GPIO1_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT19__GPIO5_IO13 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT20__GPIO5_IO14 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};
void udc_pins_setting(void)
{
	imx_iomux_v3_setup_multiple_pads(otg_udc_pads,
			ARRAY_SIZE(otg_udc_pads));
	/* USB_OTG_PWR = 0 */
	gpio_direction_output(USB_OTG_PWR, 0);
	/* USB_V1_POWER = 1 */
	gpio_direction_output(USB_V1_POWER, 1);
	/* USB_V1_POWER = 1 */
	gpio_direction_output(USB_V1_POWER, 1);

	mxc_iomux_set_gpr_register(1, 13, 1, 1);

}
#endif
