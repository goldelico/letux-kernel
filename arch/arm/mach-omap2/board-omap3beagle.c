/*
 * linux/arch/arm/mach-omap2/board-omap3beagle.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/backlight.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>

#include <linux/regulator/machine.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/tsc2007.h>

#include <linux/i2c/bmp085.h>

#include <linux/input/tca8418_keypad.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>
#include <plat/timer-gp.h>
#include <plat/clock.h>
#include <plat/omap-pm.h>
#include <plat/mcspi.h>

#include "mux.h"
#include "mmc-twl4030.h"
#include "pm.h"
#include "omap3-opp.h"

#ifdef CONFIG_PM
static struct omap_opp * _omap35x_mpu_rate_table        = omap35x_mpu_rate_table;
static struct omap_opp * _omap37x_mpu_rate_table        = omap37x_mpu_rate_table;
static struct omap_opp * _omap35x_dsp_rate_table        = omap35x_dsp_rate_table;
static struct omap_opp * _omap37x_dsp_rate_table        = omap37x_dsp_rate_table;
static struct omap_opp * _omap35x_l3_rate_table         = omap35x_l3_rate_table;
static struct omap_opp * _omap37x_l3_rate_table         = omap37x_l3_rate_table;
#else   /* CONFIG_PM */
static struct omap_opp * _omap35x_mpu_rate_table        = NULL;
static struct omap_opp * _omap37x_mpu_rate_table        = NULL;
static struct omap_opp * _omap35x_dsp_rate_table        = NULL;
static struct omap_opp * _omap37x_dsp_rate_table        = NULL;
static struct omap_opp * _omap35x_l3_rate_table         = NULL;
static struct omap_opp * _omap37x_l3_rate_table         = NULL;
#endif  /* CONFIG_PM */


#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

#define NAND_BLOCK_SIZE		SZ_128K

char expansionboard_name[16];

/* see: https://patchwork.kernel.org/patch/120449/
 * OMAP3 Beagle revision
 * Run time detection of Beagle revision is done by reading GPIO.
 * GPIO ID -
 *	AXBX	= GPIO173, GPIO172, GPIO171: 1 1 1
 *	C1_3	= GPIO173, GPIO172, GPIO171: 1 1 0
 *	C4	= GPIO173, GPIO172, GPIO171: 1 0 1
 *	XM	= GPIO173, GPIO172, GPIO171: 0 0 0
 */
enum {
	OMAP3BEAGLE_BOARD_UNKN = 0,
	OMAP3BEAGLE_BOARD_AXBX,
	OMAP3BEAGLE_BOARD_C1_3,
	OMAP3BEAGLE_BOARD_C4,
	OMAP3BEAGLE_BOARD_XM,
};

static u8 omap3_beagle_version;

#define isXM (omap3_beagle_version == OMAP3BEAGLE_BOARD_XM)

static u8 omap3_beagle_get_rev(void)
{
	return omap3_beagle_version;
}

static void __init omap3_beagle_init_rev(void)
{
	int ret;
	u16 beagle_rev = 0;
	
	omap_mux_init_gpio(171, OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(172, OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(173, OMAP_PIN_INPUT_PULLUP);
	
	ret = gpio_request(171, "rev_id_0");
	if (ret < 0)
		goto fail0;
	
	ret = gpio_request(172, "rev_id_1");
	if (ret < 0)
		goto fail1;
	
	ret = gpio_request(173, "rev_id_2");
	if (ret < 0)
		goto fail2;
	
	gpio_direction_input(171);
	gpio_direction_input(172);
	gpio_direction_input(173);
	
	beagle_rev = gpio_get_value(171) | (gpio_get_value(172) << 1)
	| (gpio_get_value(173) << 2);
	
	switch (beagle_rev) {
		case 7:
			printk(KERN_INFO "OMAP3 Beagle Rev: Ax/Bx\n");
			omap3_beagle_version = OMAP3BEAGLE_BOARD_AXBX;
			break;
		case 6:
			printk(KERN_INFO "OMAP3 Beagle Rev: C1/C2/C3\n");
			omap3_beagle_version = OMAP3BEAGLE_BOARD_C1_3;
			break;
		case 5:
			printk(KERN_INFO "OMAP3 Beagle Rev: C4\n");
			omap3_beagle_version = OMAP3BEAGLE_BOARD_C4;
			break;
		case 0:
			printk(KERN_INFO "OMAP3 Beagle Rev: xM\n");
			omap3_beagle_version = OMAP3BEAGLE_BOARD_XM;
			break;
		default:
			printk(KERN_INFO "OMAP3 Beagle Rev: unknown %hd\n", beagle_rev);
			omap3_beagle_version = OMAP3BEAGLE_BOARD_UNKN;
	}
	
	return;
	
fail2:
	gpio_free(172);
fail1:
	gpio_free(171);
fail0:
	printk(KERN_ERR "Unable to get revision detection GPIO pins\n");
	omap3_beagle_version = OMAP3BEAGLE_BOARD_UNKN;
	
	return;
}


#if defined(CONFIG_ENC28J60) || defined(CONFIG_ENC28J60_MODULE)

#include <plat/mcspi.h>
#include <linux/spi/spi.h>

#define OMAP3BEAGLE_GPIO_ENC28J60_IRQ 157

static struct omap2_mcspi_device_config enc28j60_spi_chip_info = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_board_info omap3beagle_zippy_spi_board_info[] __initdata = {
	{
		.modalias		= "enc28j60",
		.bus_num		= 4,
		.chip_select		= 0,
		.max_speed_hz		= 20000000,
		.controller_data	= &enc28j60_spi_chip_info,
	},
};

static void __init omap3beagle_enc28j60_init(void)
{
	if ((gpio_request(OMAP3BEAGLE_GPIO_ENC28J60_IRQ, "ENC28J60_IRQ") == 0) &&
	    (gpio_direction_input(OMAP3BEAGLE_GPIO_ENC28J60_IRQ) == 0)) {
		gpio_export(OMAP3BEAGLE_GPIO_ENC28J60_IRQ, 0);
		omap3beagle_zippy_spi_board_info[0].irq	= OMAP_GPIO_IRQ(OMAP3BEAGLE_GPIO_ENC28J60_IRQ);
		set_irq_type(omap3beagle_zippy_spi_board_info[0].irq, IRQ_TYPE_EDGE_FALLING);
	} else {
		printk(KERN_ERR "could not obtain gpio for ENC28J60_IRQ\n");
		return;
	}

	spi_register_board_info(omap3beagle_zippy_spi_board_info,
			ARRAY_SIZE(omap3beagle_zippy_spi_board_info));
}

#else
static inline void __init omap3beagle_enc28j60_init(void) { return; }
#endif

#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)

#include <plat/mcspi.h>
#include <linux/spi/spi.h>

#define OMAP3BEAGLE_GPIO_KS8851_IRQ 157

static struct omap2_mcspi_device_config ks8851_spi_chip_info = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_board_info omap3beagle_zippy2_spi_board_info[] __initdata = {
	{
		.modalias		= "ks8851",
		.bus_num		= 4,
		.chip_select		= 0,
		.max_speed_hz		= 36000000,
		.controller_data	= &ks8851_spi_chip_info,
	},
};

static void __init omap3beagle_ks8851_init(void)
{
	if ((gpio_request(OMAP3BEAGLE_GPIO_KS8851_IRQ, "KS8851_IRQ") == 0) &&
	    (gpio_direction_input(OMAP3BEAGLE_GPIO_KS8851_IRQ) == 0)) {
		gpio_export(OMAP3BEAGLE_GPIO_KS8851_IRQ, 0);
		omap3beagle_zippy2_spi_board_info[0].irq	= OMAP_GPIO_IRQ(OMAP3BEAGLE_GPIO_KS8851_IRQ);
		set_irq_type(omap3beagle_zippy2_spi_board_info[0].irq, IRQ_TYPE_EDGE_FALLING);
	} else {
		printk(KERN_ERR "could not obtain gpio for KS8851_IRQ\n");
		return;
	}
	
	spi_register_board_info(omap3beagle_zippy2_spi_board_info,
							ARRAY_SIZE(omap3beagle_zippy2_spi_board_info));
}

#else
static inline void __init omap3beagle_ks8851_init(void) { return; }
#endif

static struct mtd_partition omap3beagle_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 15 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
		.size		= 1 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 32 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x680000 */
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct omap_nand_platform_data omap3beagle_nand_data = {
	.options	= NAND_BUSWIDTH_16,
	.parts		= omap3beagle_nand_partitions,
	.nr_parts	= ARRAY_SIZE(omap3beagle_nand_partitions),
	.dma_channel	= -1,		/* disable DMA in OMAP NAND driver */
	.nand_setup	= NULL,
	.dev_ready	= NULL,
};

static struct resource omap3beagle_nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device omap3beagle_nand_device = {
	.name		= "omap2-nand",
	.id		= -1,
	.dev		= {
		.platform_data	= &omap3beagle_nand_data,
	},
	.num_resources	= 1,
	.resource	= &omap3beagle_nand_resource,
};

/* DSS */

static int beagle_enable_dvi(struct omap_dss_device *dssdev)
{
	if (dssdev->reset_gpio != -1)
		gpio_set_value(dssdev->reset_gpio, 1);

	return 0;
}

static void beagle_disable_dvi(struct omap_dss_device *dssdev)
{
	if (dssdev->reset_gpio != -1)
		gpio_set_value(dssdev->reset_gpio, 0);
}

static struct omap_dss_device beagle_dvi_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "dvi",
	.driver_name = "generic_panel",
	.phy.dpi.data_lines = 24,
	.reset_gpio = 170,
	.platform_enable = beagle_enable_dvi,
	.platform_disable = beagle_disable_dvi,
};

static int beagle_enable_lcd(struct omap_dss_device *dssdev)
{
	printk("beagle_enable_lcd()\n");
	// whatever we need, e.g. enable power
	gpio_set_value(170, 0);	// DVI off
#if defined(CONFIG_PANEL_ORTUS_COM37H3M05DTC)

#elif defined(CONFIG_PANEL_TPO_TD028TTEC1)
	gpio_set_value(145, 1);	// enable backlight
	gpio_set_value(79, 0);	// disable green power led
#elif defined(CONFIG_PANEL_SHARP_LQ070Y3DG3B)

#elif defined(CONFIG_PANEL_SHARP_LQ050W1LC1B)

#endif
	return 0;
}

static void beagle_disable_lcd(struct omap_dss_device *dssdev)
{
	printk("beagle_disable_lcd()\n");
	// whatever we need, e.g. disable power
#if defined(CONFIG_PANEL_ORTUS_COM37H3M05DTC)

#elif defined(CONFIG_PANEL_TPO_TD028TTEC1)
	gpio_set_value(145, 0);	// shut down backlight
	gpio_set_value(79, 1);	// show green power led
#elif defined(CONFIG_PANEL_SHARP_LQ070Y3DG3B)

#elif defined(CONFIG_PANEL_SHARP_LQ050W1LC1B)

#endif
}

static struct omap_dss_device beagle_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "lcd",
#if defined(CONFIG_PANEL_ORTUS_COM37H3M05DTC)
	.driver_name = "com37h3m05dtc_panel",
#elif defined(CONFIG_PANEL_TPO_TD028TTEC1)
	.driver_name = "td028ttec1_panel",
#elif defined(CONFIG_PANEL_SHARP_LQ070Y3DG3B)
	.driver_name = "lq070y3dg3b_panel",
#elif defined(CONFIG_PANEL_SHARP_LQ050W1LC1B)
	.driver_name = "lq050w1lc1b_panel",
#endif
	.phy.dpi.data_lines = 24,
	.platform_enable = beagle_enable_lcd,
	.platform_disable = beagle_disable_lcd,
};

static int beagle_panel_enable_tv(struct omap_dss_device *dssdev)
{
#define ENABLE_VDAC_DEDICATED           0x03
#define ENABLE_VDAC_DEV_GRP             0x20

	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEDICATED,
			TWL4030_VDAC_DEDICATED);
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEV_GRP, TWL4030_VDAC_DEV_GRP);

	return 0;
}

static void beagle_panel_disable_tv(struct omap_dss_device *dssdev)
{
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEDICATED);
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEV_GRP);
}

static struct omap_dss_device beagle_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_SVIDEO,
	.platform_enable = beagle_panel_enable_tv,
	.platform_disable = beagle_panel_disable_tv,
};

static struct omap_dss_device *beagle_dss_devices[] = {
	&beagle_dvi_device,
	&beagle_tv_device,
	&beagle_lcd_device,
};

static struct omap_dss_board_info beagle_dss_data = {
	.num_devices = ARRAY_SIZE(beagle_dss_devices),
	.devices = beagle_dss_devices,
	.default_device = &beagle_dvi_device,
};

static struct platform_device beagle_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &beagle_dss_data,
	},
};

static void beagle_set_bl_intensity(int intensity)
{
	// control PWM_10
	// use 500 Hz pulse and intensity 0..255
}

static struct generic_bl_info beagle_bl_platform_data = {
	.name			= "bklight",
	.max_intensity		= 255,
	.default_intensity	= 200,
	.limit_mask		= 0,
	.set_bl_intensity	= beagle_set_bl_intensity,
	.kick_battery		= NULL,
};

static struct platform_device beagle_bklight_device = {
	.name		= "generic-bl",
	.id			= -1,
	.dev		= {
		.parent		= &beagle_dss_device.dev,
		.platform_data	= &beagle_bl_platform_data,
	},
};


static struct regulator_consumer_supply beagle_vdac_supply = {
	.supply		= "vdda_dac",
	.dev		= &beagle_dss_device.dev,
};

static struct regulator_consumer_supply beagle_vdvi_supply = {
	.supply		= "vdds_dsi",
	.dev		= &beagle_dss_device.dev,
};

static void __init beagle_display_init(void)
{
	int r;

	r = gpio_request(beagle_dvi_device.reset_gpio, "DVI reset");
	if (r < 0) {
		printk(KERN_ERR "Unable to get DVI reset GPIO %d\n", beagle_dvi_device.reset_gpio);
		return;
	}

	gpio_direction_output(beagle_dvi_device.reset_gpio, 0);
	gpio_set_value(beagle_dvi_device.reset_gpio, 0);
}

#include "sdram-micron-mt46h32m32lf-6.h"

static struct twl4030_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
//		.wires		= 8,
		.wires		= 4,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{
		.mmc		= 2,
		.wires		= 4,
		.transceiver	= true,
		.ocr_mask	= 0x00100000,	/* 3.3V */
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply beagle_vmmc1_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply beagle_vsim_supply = {
	.supply			= "vmmc_aux",
};

static struct gpio_led gpio_leds[];

static int beagle_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	if (!cpu_is_omap3630()) { /* only on Beagleboard A,B,C */
		if (system_rev >= 0x20 && system_rev <= 0x34301000) {
			omap_mux_init_gpio(23, OMAP_PIN_INPUT);
			mmc[0].gpio_wp = 23;
		} else {
			omap_mux_init_gpio(29, OMAP_PIN_INPUT);
			mmc[0].gpio_wp = 29;
		}
		/* gpio + 0 is "mmc0_cd" (input/IRQ) */
		mmc[0].gpio_cd = gpio + 0;		
	}
	twl4030_mmc_init(mmc);

	/* link regulators to MMC adapters */
	beagle_vmmc1_supply.dev = mmc[0].dev;
	beagle_vsim_supply.dev = mmc[0].dev;

	/* REVISIT: need ehci-omap hooks for external VBUS
	 * power switch and overcurrent detect
	 */

	if (cpu_is_omap3630()) {
		/* Power on DVI, Serial and PWR led */ 
 		gpio_request(gpio + 1, "nDVI_PWR_EN");
		gpio_direction_output(gpio + 1, 0);	

		/* Power on camera interface */
		gpio_request(gpio + 2, "CAM_EN");
		gpio_direction_output(gpio + 2, 1);

		/* TWL4030_GPIO_MAX + 0 == ledA, EHCI nEN_USB_PWR (out, active low) */
		gpio_request(gpio + TWL4030_GPIO_MAX, "nEN_USB_PWR");
		gpio_direction_output(gpio + TWL4030_GPIO_MAX, 1);
	}
	else {
		gpio_request(gpio + 1, "EHCI_nOC");
		gpio_direction_input(gpio + 1);

		/* TWL4030_GPIO_MAX + 0 == ledA, EHCI nEN_USB_PWR (out, active low) */
		gpio_request(gpio + TWL4030_GPIO_MAX, "nEN_USB_PWR");
		gpio_direction_output(gpio + TWL4030_GPIO_MAX, 0);
	}


	/* TWL4030_GPIO_MAX + 1 == ledB, PMU_STAT (out, active low LED) */
	gpio_leds[2].gpio = gpio + TWL4030_GPIO_MAX + 1;

	return 0;
}

static struct twl4030_gpio_platform_data beagle_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.pullups	= BIT(1),
	.pulldowns	= BIT(2) | BIT(6) | BIT(7) | BIT(8) | BIT(13)
				| BIT(15) | BIT(16) | BIT(17),
	.setup		= beagle_twl_gpio_setup,
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data beagle_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vmmc1_supply,
};

/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
static struct regulator_init_data beagle_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vsim_supply,
};

/* VDAC for DSS driving S-Video (8 mA unloaded, max 65 mA) */
static struct regulator_init_data beagle_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vdac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_init_data beagle_vpll2 = {
	.constraints = {
		.name			= "VDVI",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vdvi_supply,
};

static struct twl4030_usb_data beagle_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_codec_audio_data beagle_audio_data = {
	.audio_mclk = 26000000,
};

static struct twl4030_codec_data beagle_codec_data = {
	.audio_mclk = 26000000,
	.audio = &beagle_audio_data,
};

static struct twl4030_madc_platform_data beagle_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_platform_data beagle_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.usb		= &beagle_usb_data,
	.gpio		= &beagle_gpio_data,
	.codec		= &beagle_codec_data,
	.madc		= &beagle_madc_data,
	.vmmc1		= &beagle_vmmc1,
	.vsim		= &beagle_vsim,
	.vdac		= &beagle_vdac,
	.vpll2		= &beagle_vpll2,
};

static struct i2c_board_info __initdata beagle_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &beagle_twldata,
	},
};

	
#ifdef CONFIG_TOUCHSCREEN_TSC2007

// TODO: see also http://e2e.ti.com/support/arm174_microprocessors/omap_applications_processors/f/42/t/33262.aspx for an example...
// and http://www.embedded-bits.co.uk/?tag=struct-i2c_board_info for a description of how struct i2c_board_info works

/* TouchScreen */

#define TS_PENIRQ_GPIO		157	// GPIO pin

static int ts_get_pendown_state(void)
{
#if 1
	int val = 0;
	//	gpio_free(GPIO_FN_INTC_IRQ0);	// what does this change or not change on the board we have copied the code from?
//	gpio_request(TS_PENIRQ_GPIO, "tsc2007_pen_down");
//	gpio_direction_input(TS_PENIRQ_GPIO);
	
	val = gpio_get_value(TS_PENIRQ_GPIO);
	
//	gpio_free(TS_PENIRQ_GPIO);
	//	gpio_request(GPIO_FN_INTC_IRQ0, NULL);
//	printk("ts_get_pendown_state() -> %d\n", val);
	return val ? 0 : 1;
#else
	return 0;
#endif
}

static int __init tsc2007_init(void)
{
	printk("tsc2007_init()\n");
	omap_mux_init_gpio(TS_PENIRQ_GPIO, OMAP_PIN_INPUT_PULLUP);
	if (gpio_request(TS_PENIRQ_GPIO, "tsc2007_pen_down")) {
		printk(KERN_ERR "Failed to request GPIO %d for "
			   "TSC2007 pen down IRQ\n", TS_PENIRQ_GPIO);
		return  -ENODEV;
	}
	
	if (gpio_direction_input(TS_PENIRQ_GPIO)) {
		printk(KERN_WARNING "GPIO#%d cannot be configured as "
			   "input\n", TS_PENIRQ_GPIO);
		return -ENXIO;
	}
//	gpio_export(TS_PENIRQ_GPIO, 0);
	omap_set_gpio_debounce(TS_PENIRQ_GPIO, 1);
	omap_set_gpio_debounce_time(TS_PENIRQ_GPIO, 0xa);
	set_irq_type(OMAP_GPIO_IRQ(TS_PENIRQ_GPIO), IRQ_TYPE_EDGE_FALLING);
	return 0;
}

static void tsc2007_exit(void)
{
	gpio_free(TS_PENIRQ_GPIO);
}

struct tsc2007_platform_data tsc2007_info = {
	.model			= 2007,
	.x_plate_ohms		= 600,	// range: 250 .. 900 
	.get_pendown_state	= ts_get_pendown_state,
	.init_platform_hw	= tsc2007_init,
	.exit_platform_hw	= tsc2007_exit,
};

#endif


#ifdef CONFIG_BMP085

#define BMP085_EOC_IRQ_GPIO		113	/* BMP085 end of conversion GPIO */

static int __init bmp085_init(void)
{
	printk("bmp085_init()\n");
	omap_mux_init_gpio(BMP085_EOC_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP);
	if (gpio_request(BMP085_EOC_IRQ_GPIO, "bmp085_eoc_irq")) {
		printk(KERN_ERR "Failed to request GPIO %d for "
			   "BMP085 EOC IRQ\n", BMP085_EOC_IRQ_GPIO);
		return  -ENODEV;
	}
	
	if (gpio_direction_input(BMP085_EOC_IRQ_GPIO)) {
		printk(KERN_WARNING "GPIO#%d cannot be configured as "
			   "input\n", BMP085_EOC_IRQ_GPIO);
		return -ENXIO;
	}
//	gpio_export(TS_PENIRQ_GPIO, 0);
	omap_set_gpio_debounce(BMP085_EOC_IRQ_GPIO, 1);
	omap_set_gpio_debounce_time(BMP085_EOC_IRQ_GPIO, 0xa);
	set_irq_type(OMAP_GPIO_IRQ(BMP085_EOC_IRQ_GPIO), IRQ_TYPE_EDGE_FALLING);
	return 0;
}

static void bmp085_exit(void)
{
	gpio_free(BMP085_EOC_IRQ_GPIO);
}

struct bmp085_platform_data bmp085_info = {
	.init_platform_hw	= bmp085_init,
	.exit_platform_hw	= bmp085_exit,
};

#endif

#ifdef CONFIG_TRF7960

#define TRF7960_IRQ_GPIO		113	/* TRF7960 interrupt GPIO */

static int __init trf7960_init(void)
{
	printk("trf7960_init()\n");
	omap_mux_init_gpio(TRF7960_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP);
	if (gpio_request(TRF7960_IRQ_GPIO, "trf7960_eoc_irq")) {
		printk(KERN_ERR "Failed to request GPIO %d for "
			   "TRF7960 IRQ\n", TRF7960_IRQ_GPIO);
		return  -ENODEV;
	}
	
	if (gpio_direction_input(TRF7960_IRQ_GPIO)) {
		printk(KERN_WARNING "GPIO#%d cannot be configured as "
			   "input\n", TRF7960_IRQ_GPIO);
		return -ENXIO;
	}
	//	gpio_export(TS_PENIRQ_GPIO, 0);
	omap_set_gpio_debounce(TRF7960_IRQ_GPIO, 1);
	omap_set_gpio_debounce_time(TRF7960_IRQ_GPIO, 0xa);
	set_irq_type(OMAP_GPIO_IRQ(TRF7960_IRQ_GPIO), IRQ_TYPE_EDGE_RISING);
	return 0;
}

static void trf7960_init(void)
{
	gpio_free(TRF7960_IRQ_GPIO);
}

struct trf7960_platform_data trf7960_info = {
	.init_platform_hw	= trf7960_init,
	.exit_platform_hw	= trf7960_exit,
};

static struct platform_device rfid_device = {
	.name		= "trf7960",
	.id			= -1,
	.dev		= {
		.platform_data	= &trf7960_info,
	},
};

#endif

#if defined(CONFIG_KEYBOARD_TCA8418) || defined(CONFIG_KEYBOARD_TCA8418_MODULE)

#define KEYIRQ_GPIO 144

const uint32_t gta04_keymap[] = {
	/* KEY(row, col, val) - see include/linux/input.h */
	KEY(0, 0, KEY_LEFTCTRL),
	KEY(0, 1, KEY_RIGHTCTRL),
	KEY(0, 2, KEY_Y),
	KEY(0, 3, KEY_A),
	KEY(0, 4, KEY_Q),
	KEY(0, 5, KEY_1),
	//	KEY(0, 6, KEY_RESERVED),
	//	KEY(0, 7, KEY_RESERVED),
	KEY(0, 8, KEY_SPACE),
	KEY(0, 9, KEY_OK),
	
	KEY(1, 0, KEY_LEFTALT),
	KEY(1, 1, KEY_FN),
	KEY(1, 2, KEY_SPACE),
	KEY(1, 3, KEY_SPACE),
	KEY(1, 4, KEY_COMMA),
	KEY(1, 5, KEY_DOT),
	KEY(1, 6, KEY_PAGEDOWN),
	KEY(1, 7, KEY_END),
	KEY(1, 8, KEY_LEFT),
	KEY(1, 9, KEY_RIGHT),
	
	KEY(2, 0, KEY_DELETE),
	//	KEY(2, 1, KEY_RESERVED),
	KEY(2, 2, KEY_TAB),
	KEY(2, 3, KEY_BACKSPACE),
	KEY(2, 4, KEY_ENTER),
	KEY(2, 5, KEY_GRAVE),
	KEY(2, 6, KEY_PAGEUP),
	KEY(2, 7, KEY_HOME),
	KEY(2, 8, KEY_UP),
	KEY(2, 9, KEY_DOWN),
	
	KEY(3, 0, KEY_RIGHTALT),
	KEY(3, 1, KEY_CAPSLOCK),
	KEY(3, 2, KEY_ESC),
	//	KEY(3, 3, KEY_RESERVED),
	//	KEY(3, 4, KEY_RESERVED),
	//	KEY(3, 5, KEY_RESERVED),
	KEY(3, 6, KEY_LEFTBRACE),
	KEY(3, 7, KEY_RIGHTBRACE),
	KEY(3, 8, KEY_SEMICOLON),
	KEY(3, 9, KEY_APOSTROPHE),
	
	KEY(4, 0, KEY_LEFTSHIFT),
	KEY(4, 1, KEY_X),
	KEY(4, 2, KEY_C),
	KEY(4, 3, KEY_V),
	KEY(4, 4, KEY_B),
	KEY(4, 5, KEY_N),
	KEY(4, 6, KEY_M),
	KEY(4, 7, KEY_MINUS),
	KEY(4, 8, KEY_EQUAL),
	KEY(4, 9, KEY_KPASTERISK),
	
	KEY(5, 0, KEY_RIGHTSHIFT),
	KEY(5, 1, KEY_S),
	KEY(5, 2, KEY_D),
	KEY(5, 3, KEY_F),
	KEY(5, 4, KEY_G),
	KEY(5, 5, KEY_H),
	KEY(5, 6, KEY_J),
	KEY(5, 7, KEY_K),
	KEY(5, 8, KEY_L),
	KEY(5, 9, KEY_APOSTROPHE),
	
	KEY(6, 0, KEY_LEFTMETA),
	KEY(6, 1, KEY_W),
	KEY(6, 2, KEY_E),
	KEY(6, 3, KEY_R),
	KEY(6, 4, KEY_T),
	KEY(6, 5, KEY_Z),
	KEY(6, 6, KEY_U),
	KEY(6, 7, KEY_I),
	KEY(6, 8, KEY_O),
	KEY(6, 9, KEY_P),
	
	KEY(7, 0, KEY_RIGHTMETA),
	KEY(7, 1, KEY_2),
	KEY(7, 2, KEY_3),
	KEY(7, 3, KEY_4),
	KEY(7, 4, KEY_5),
	KEY(7, 5, KEY_6),
	KEY(7, 6, KEY_7),
	KEY(7, 7, KEY_8),
	KEY(7, 8, KEY_9),
	KEY(7, 9, KEY_0),
};

struct matrix_keymap_data tca8418_keymap = {
	.keymap = gta04_keymap,
	.keymap_size = ARRAY_SIZE(gta04_keymap),
};

struct tca8418_keypad_platform_data tca8418_pdata = {
	.keymap_data = &tca8418_keymap,
	.rows = 8,
	.cols = 10,
	.rep = 1,
	.irq_is_gpio = 1,
};

#endif

#if defined(CONFIG_EEPROM_AT24) || defined(CONFIG_EEPROM_AT24_MODULE)
#include <linux/i2c/at24.h>

static struct at24_platform_data m24c01 = {
	        .byte_len       = SZ_1K / 8,
	        .page_size      = 16,
};

#if defined(CONFIG_RTC_DRV_DS1307) || \
	defined(CONFIG_RTC_DRV_DS1307_MODULE)

static struct i2c_board_info __initdata beagle_zippy_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("ds1307", 0x68),
	},
	{
		I2C_BOARD_INFO("24c01", 0x50),
		.platform_data	= &m24c01,
	},
};
#else
static struct i2c_board_info __initdata beagle_zippy_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("24c01", 0x50),
		.platform_data  = &m24c01,
	},
};
#endif
#else
static struct i2c_board_info __initdata beagle_zippy_i2c2_boardinfo[] = {};
#endif

static struct i2c_board_info __initdata beagle_i2c2_boardinfo[] = {
#ifdef CONFIG_TOUCHSCREEN_TSC2007
{
	I2C_BOARD_INFO("tsc2007", 0x48),
	.type		= "tsc2007",
	.platform_data	= &tsc2007_info,
	.irq		=  OMAP_GPIO_IRQ(TS_PENIRQ_GPIO),
},
#endif
#ifdef CONFIG_BMP085
{
	I2C_BOARD_INFO("bmp085", 0x77),
	.type		= "bmp085",
	.platform_data	= &bmp085_info,
	.irq		=  OMAP_GPIO_IRQ(BMP085_EOC_IRQ_GPIO),
},
#endif
#ifdef CONFIG_LIS302
{
	I2C_BOARD_INFO("lis302top", 0x1c),
	.type		= "lis302",
	.platform_data	= &lis302_info,
	.irq		=  -EINVAL,
},
{
	I2C_BOARD_INFO("lis302bottom", 0x1d),
	.type		= "lis302",
	.platform_data	= &lis302_info,
	.irq		=  -EINVAL,
},
#endif
#if defined(CONFIG_LEDS_TCA6507)
{
	I2C_BOARD_INFO("tca6507", 0x45),
	.type		= "tca6507",
	.platform_data	= NULL,
},
#endif
#if defined(CONFIG_KEYBOARD_TCA8418) || defined(CONFIG_KEYBOARD_TCA8418_MODULE)
	{
	I2C_BOARD_INFO("tca8418", 0x34),	/* /sys/.../name */
	.type		= "tca8418_keypad",	/* driver name */
	.platform_data	= &tca8418_pdata,
	.irq		= KEYIRQ_GPIO,
	},	
#endif	
};

static int __init omap3_beagle_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, beagle_i2c1_boardinfo,
			ARRAY_SIZE(beagle_i2c1_boardinfo));
	if(!strcmp(expansionboard_name, "zippy") || !strcmp(expansionboard_name, "zippy2")) 
	{
		printk(KERN_INFO "Beagle expansionboard: registering i2c2 bus for zippy/zippy2\n");
		omap_register_i2c_bus(2, 400,  beagle_zippy_i2c2_boardinfo,
				ARRAY_SIZE(beagle_zippy_i2c2_boardinfo));
	} else
	{
		omap_register_i2c_bus(2, 400,  beagle_i2c2_boardinfo,
				ARRAY_SIZE(beagle_i2c2_boardinfo));
	}
	/* Bus 3 is attached to the DVI port where devices like the pico DLP
	 * projector don't work reliably with 400kHz */
	omap_register_i2c_bus(3, 100, NULL, 0);
	return 0;
}

#if 0

static struct spi_board_info beaglefpga_mcspi_board_info[] = {
	// spi 4.0
	{
		.modalias	= "spidev",
		.max_speed_hz	= 48000000, //48 Mbps
		.bus_num	= 4,
		.chip_select	= 0,	
		.mode = SPI_MODE_1,
	},
};

static void __init beaglefpga_init_spi(void)
{
		/* hook the spi ports to the spidev driver */
		spi_register_board_info(beaglefpga_mcspi_board_info,
			ARRAY_SIZE(beaglefpga_mcspi_board_info));
}
#endif

static struct gpio_led gpio_leds[] = {
	{
		.name			= "beagleboard::usr0",
		.default_trigger	= "heartbeat",
		.gpio			= 150,
	},
	{
		.name			= "beagleboard::usr1",
		.default_trigger	= "mmc0",
		.gpio			= 149,
	},
	{
		.name			= "beagleboard::pmu_stat",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= true,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static struct gpio_keys_button gpio_buttons[] = {
	{
		.code			= BTN_EXTRA,
		.gpio			= 7,
		.desc			= "user",
		.wakeup			= 1,
	},
};

static struct gpio_keys_platform_data gpio_key_info = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_key_info,
	},
};

static void __init omap3_beagle_init_irq(void)
{
        if (cpu_is_omap3630())
        {
                omap2_init_common_hw(mt46h32m32lf6_sdrc_params,
                                        mt46h32m32lf6_sdrc_params,
                                        _omap37x_mpu_rate_table,
                                        _omap37x_dsp_rate_table,
                                        _omap37x_l3_rate_table);
        }
        else
        {
                omap2_init_common_hw(mt46h32m32lf6_sdrc_params,
                                        mt46h32m32lf6_sdrc_params,
                                        _omap35x_mpu_rate_table,
                                        _omap35x_dsp_rate_table,
                                        _omap35x_l3_rate_table);
        }
	omap_init_irq();
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(12);
#endif
	omap_gpio_init();
}

static struct platform_device *omap3_beagle_devices[] __initdata = {
	&leds_gpio,
	&keys_gpio,
	&beagle_dss_device,
	&beagle_bklight_device,
#if defined(CONFIG_TRF7960)
	&rfid_device,
#endif
};

static void __init omap3beagle_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;

	u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;

	/* find out the chip-select on which NAND exists */
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			printk(KERN_INFO "Found NAND on CS%d\n", cs);
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}

	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				 "in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		omap3beagle_nand_data.cs = nandcs;
		omap3beagle_nand_data.gpmc_cs_baseaddr = (void *)
			(gpmc_base_add + GPMC_CS0_BASE + nandcs * GPMC_CS_SIZE);
		omap3beagle_nand_data.gpmc_baseaddr = (void *) (gpmc_base_add);

		printk(KERN_INFO "Registering NAND on CS%d\n", nandcs);
		if (platform_device_register(&omap3beagle_nand_device) < 0)
			printk(KERN_ERR "Unable to register NAND device\n");
	}
}

static struct ehci_hcd_omap_platform_data ehci_pdata __initdata = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = 147,
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#error we need CONFIG_OMAP_MUX
#define board_mux	NULL
#endif

static int __init expansionboard_setup(char *str)
{
	if (!str)
		return -EINVAL;
#if defined(CONFIG_PANEL_TPO_TD028TTEC1)
	str="omb";
#elif defined(CONFIG_PANEL_ORTUS_COM37H3M05DTC)
	str="b2";
#elif defined(CONFIG_PANEL_SHARP_LQ070Y3DG3B)
	str="b3";
#elif defined(CONFIG_PANEL_SHARP_LQ050W1LC1B)
	str="b4";
#endif
	strncpy(expansionboard_name, str, 16);
	printk(KERN_INFO "Beagle expansionboard: %s\n", expansionboard_name);
	return 0;
}

static void __init omap3_beagle_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	omap3_beagle_init_rev();
	omap3_beagle_i2c_init();
	platform_add_devices(omap3_beagle_devices,
			ARRAY_SIZE(omap3_beagle_devices));
	omap_serial_init();

#if defined(CONFIG_KEYBOARD_TCA8418) || defined(CONFIG_KEYBOARD_TCA8418_MODULE)
	omap_mux_init_gpio(KEYIRQ_GPIO, OMAP_PIN_INPUT_PULLUP);
	
	if (gpio_request(KEYIRQ_GPIO, "keyirq")) {
		printk(KERN_ERR "Failed to request GPIO %d for "
			   "KEYIRQ\n", KEYIRQ_GPIO);
	}
	
	if (gpio_direction_input(KEYIRQ_GPIO)) {
		printk(KERN_WARNING "GPIO#%d cannot be configured as "
			   "input\n", KEYIRQ_GPIO);
	}
	omap_set_gpio_debounce(KEYIRQ_GPIO, 1);
	omap_set_gpio_debounce_time(KEYIRQ_GPIO, 0xa);
	set_irq_type(OMAP_GPIO_IRQ(KEYIRQ_GPIO), IRQ_TYPE_EDGE_FALLING);
#endif

	if(!strcmp(expansionboard_name, "omb")) 
		{
		//	omap_mux_init_gpio(170, OMAP_PIN_INPUT);
		omap_mux_init_gpio(170, OMAP_PIN_OUTPUT);
		gpio_request(170, "DVI_nPD");
		gpio_direction_output(170, true);
		gpio_set_value(170, 0);	/* leave DVI powered down until it's needed ... */
		gpio_export(170, 0);	// no direction change
		
		printk(KERN_INFO "Beagle expansionboard: Openmoko Beagle Hybrid\n");
		//	omap_mux_init_gpio(156, OMAP_PIN_OUTPUT);	// inherit from U-Boot...
		gpio_request(156, "GPS_ON");
		gpio_direction_output(156, true);
		gpio_set_value(156, 0);	// off
		gpio_export(156, 0);	// no direction change
		
		// should be a backlight driver using PWM
		gpio_request(145, "LCD_BACKLIGHT");
		//	gpio_set_value(145, 1);	// on
		gpio_direction_output(145, true);
		gpio_export(145, 0);	// no direction change
		
		gpio_request(136, "AUX_BUTTON");
		gpio_direction_input(136);
		gpio_export(136, 0);	// no direction change
		
		gpio_request(137, "POWER_BUTTON");
		gpio_direction_input(137);
		gpio_export(137, 0);	// no direction change
		
		//	omap_mux_init_signal("gpio138", OMAP_PIN_INPUT);	// gpio 138 - with no pullup/pull-down
		gpio_request(138, "EXT_ANT");
		gpio_direction_input(138);
		gpio_export(138, 0);	// no direction change
		
		gpio_request(isXM?88:70, "AUX_RED");
		gpio_direction_output(isXM?88:70, true);
		gpio_set_value(isXM?88:70, 0);
		gpio_export(isXM?88:70, 0);	// no direction change
		
		gpio_request(isXM?89:71, "AUX_GREEN");
		gpio_direction_output(isXM?89:71, true);
		gpio_set_value(isXM?89:71, 0);
		gpio_export(isXM?89:71, 0);	// no direction change
		
		gpio_request(78, "POWER_RED");
		gpio_direction_output(78, true);
		gpio_set_value(78, 0);
		gpio_export(78, 0);	// no direction change
		
		gpio_request(79, "POWER_GREEN");
		gpio_direction_output(79, true);
		gpio_set_value(79, 0);
		gpio_export(79, 0);	// no direction change
		
		// for a definition of the mux names see arch/arm/mach-omap2/mux34xx.c
		// the syntax of the first paramter to omap_mux_init_signal() is "muxname" or "m0name.muxname" (for ambiguous modes)
		// note: calling omap_mux_init_signal() overwrites the parameter string...
		
		omap_mux_init_signal("mcbsp3_clkx.uart2_tx", OMAP_PIN_OUTPUT);	// gpio 142
		omap_mux_init_signal("mcbsp3_fsx.uart2_rx", OMAP_PIN_INPUT);	// gpio 143
		}
	
// FIXME: handle b2 and b4

	if(!strcmp(expansionboard_name, "zippy")) 
	{
		printk(KERN_INFO "Beagle expansionboard: initializing enc28j60\n");
		omap3beagle_enc28j60_init();
		printk(KERN_INFO "Beagle expansionboard: assigning GPIO 141 and 162 to MMC1\n");
		mmc[1].gpio_wp = 141;
		mmc[1].gpio_cd = 162;
	}
	
	if(!strcmp(expansionboard_name, "zippy2")) 
	{
		printk(KERN_INFO "Beagle expansionboard: initializing ks_8851\n");
		omap3beagle_ks8851_init();
		printk(KERN_INFO "Beagle expansionboard: assigning GPIO 141 and 162 to MMC1\n");
		mmc[1].gpio_wp = 141;
		mmc[1].gpio_cd = 162;
	}

	if(!strcmp(expansionboard_name, "trainer"))
	{
		printk(KERN_INFO "Beagle expansionboard: exporting GPIOs 130-141,162 to userspace\n");
		gpio_request(130, "sysfs");
		gpio_export(130, 1);
		gpio_request(131, "sysfs");
		gpio_export(131, 1);
		gpio_request(132, "sysfs");
		gpio_export(132, 1);
		gpio_request(133, "sysfs");
		gpio_export(133, 1);
		gpio_request(134, "sysfs");
		gpio_export(134, 1);
		gpio_request(135, "sysfs");
		gpio_export(135, 1);
		gpio_request(136, "sysfs");
		gpio_export(136, 1);
		gpio_request(137, "sysfs");
		gpio_export(137, 1);
		gpio_request(138, "sysfs");
		gpio_export(138, 1);
		gpio_request(139, "sysfs");
		gpio_export(139, 1);
		gpio_request(140, "sysfs");
		gpio_export(140, 1);
		gpio_request(141, "sysfs");
		gpio_export(141, 1);
		gpio_request(162, "sysfs");
		gpio_export(162, 1);
	}
	
	usb_musb_init();
	usb_ehci_init(&ehci_pdata);
	omap3beagle_flash_init();

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

	beagle_display_init();
}
static void __init omap3_beagle_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

early_param("buddy", expansionboard_setup);

MACHINE_START(OMAP3_BEAGLE, "OMAP3 Beagle Board")
	/* Maintainer: Syed Mohammed Khasim - http://beagleboard.org */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap3_beagle_map_io,
	.init_irq	= omap3_beagle_init_irq,
	.init_machine	= omap3_beagle_init,
	.timer		= &omap_timer,
MACHINE_END
