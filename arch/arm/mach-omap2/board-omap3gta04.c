/*
 * linux/arch/arm/mach-omap2/board-omap3gta04.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-omap3beagle.c
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
static u8 omap3_gta04_version;	/* counts 2..9 */

#define isXM (omap3_beagle_version == OMAP3BEAGLE_BOARD_XM)

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
	
	udelay(100);

	beagle_rev = gpio_get_value(171) | (gpio_get_value(172) << 1)
	| (gpio_get_value(173) << 2);

	/*
	 * on GTA04 the resistors are pull-downs (if placed)
	 * they are not placed for GTA04A2 and all placed for GTA04A9
	 * i.e. the inputs should read as "1" for GTA04A2
	 * i.e. we get a beagle_rev = 7.
	 */
	
	omap3_gta04_version = 9 - omap3_beagle_version;	// counts 2,3,4,...
	
	if (omap3_gta04_version == 2)
		omap3_beagle_version = OMAP3BEAGLE_BOARD_C4;	// GTA04A2 is like C4 board (OMAP3530)
	else
		omap3_beagle_version = OMAP3BEAGLE_BOARD_XM;	// GTA04A3++ is like XM board (DM3730)

	return;
	
fail2:
	gpio_free(172);
fail1:
	gpio_free(171);
fail0:
	printk(KERN_ERR "Unable to get revision detection GPIO pins\n");
	omap3_beagle_version = OMAP3BEAGLE_BOARD_UNKN;
	omap3_gta04_version = 0;

	return;
}


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
	// FIXME: GPIO170 has different job!
//	.reset_gpio = 170,
	.reset_gpio = -1,
	.platform_enable = beagle_enable_dvi,
	.platform_disable = beagle_disable_dvi,
};

static int beagle_enable_lcd(struct omap_dss_device *dssdev)
{
	printk("beagle_enable_lcd()\n");
	// whatever we need, e.g. enable power
//	gpio_set_value(170, 0);	// DVI off
	gpio_set_value(57, 1);	// enable backlight
//	gpio_set_value(79, 0);	// disable green power led
	return 0;
}

static void beagle_disable_lcd(struct omap_dss_device *dssdev)
{
	printk("beagle_disable_lcd()\n");
	// whatever we need, e.g. disable power
	gpio_set_value(57, 0);	// shut down backlight
//	gpio_set_value(79, 1);	// show green power led
}

static struct omap_dss_device beagle_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "lcd",
	.driver_name = "td028ttec1_panel",
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
	// control PWM_11
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
/* N/A on GTA04
	int r;
 
	r = gpio_request(beagle_dvi_device.reset_gpio, "DVI reset");
	if (r < 0) {
		printk(KERN_ERR "Unable to get DVI reset GPIO %d\n", beagle_dvi_device.reset_gpio);
		return;
	}

	gpio_direction_output(beagle_dvi_device.reset_gpio, 0);
	gpio_set_value(beagle_dvi_device.reset_gpio, 0);
 */
}

#include "sdram-micron-mt46h32m32lf-6.h"

static struct twl4030_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
//		.wires		= 8,
		.wires		= 4,	// only 4 are connected
		.gpio_cd	= -EINVAL,	// no card detect
//		.gpio_wp	= 29,
		.gpio_wp	= -EINVAL,	// no write protect
	},
	{ // this is the WiFi SDIO interface
		.mmc		= 2,
		.wires		= 4,
		.gpio_cd	= -EINVAL,	// no card detect
		.gpio_wp	= -EINVAL,	// no write protect
		.transceiver	= true,	// external transceiver
		.ocr_mask	= 0x00100000,	/* fixed 3.3V */
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
	// we should keep enabling mmc, vmmc1, nEN_USB_PWR, maybe CAM_EN
	
	// FIXME: this is a complete mess - mmc and gpio init mixed up
	
#if 0	
	if (system_rev >= 0x20 && system_rev <= 0x34301000) {
		omap_mux_init_gpio(23, OMAP_PIN_INPUT);
		mmc[0].gpio_wp = 23;
	} else {
		omap_mux_init_gpio(29, OMAP_PIN_INPUT);
	}
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;
#endif

	twl4030_mmc_init(mmc);

	/* link regulators to MMC adapters */
	beagle_vmmc1_supply.dev = mmc[0].dev;
//	beagle_vsim_supply.dev = mmc[0].dev;	/* supply for upper 4 bits */

#if 0
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
#endif
	
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

// FIXME: VSIM is used differently

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

#define TS_PENIRQ_GPIO		160	// GPIO pin

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
	.irq		=  114,
},
#endif
#if defined(CONFIG_LEDS_TCA6507)
{
	I2C_BOARD_INFO("tca6507", 0x45),
	.type		= "tca6507",
	.platform_data	= NULL,
},	
#endif
	/* FIXME: add other drivers for HMC5883, BMA180, Si472x, Camera */
};

static int __init omap3_beagle_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, beagle_i2c1_boardinfo,
			ARRAY_SIZE(beagle_i2c1_boardinfo));
	omap_register_i2c_bus(2, 400,  beagle_i2c2_boardinfo,
				ARRAY_SIZE(beagle_i2c2_boardinfo));
	/* Bus 3 is attached to the DVI port where devices like the pico DLP
	 * projector don't work reliably with 400kHz */
	omap_register_i2c_bus(3, 100, NULL, 0);
	return 0;
}

#if 0
// FIXME: initialize SPIs and McBSPs

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

// FIXME: replace by TCA6507 driver (and connect to appropriate triggers)
#if 0
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
#endif

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
	// FIXME: what is different?
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
	// FIXME: we could keep the Backlight "LED"...
//	&leds_gpio,
	&keys_gpio,
	&beagle_dss_device,
	&beagle_bklight_device,
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
//	.reset_gpio_port[1]  = 147,
	.reset_gpio_port[1]  = 174,
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static void __init omap3_beagle_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	omap3_beagle_init_rev();
	omap3_beagle_i2c_init();
	platform_add_devices(omap3_beagle_devices,
			ARRAY_SIZE(omap3_beagle_devices));
	omap_serial_init();

#if 0	// for Openmoko Beagle Hybrid - should have been done by u-boot - but is not reliable
		omap_mux_init_gpio(158, OMAP_PIN_OUTPUT);
		omap_mux_init_gpio(159, OMAP_PIN_INPUT);	// neither PUP nor PDN
		omap_mux_init_gpio(161, OMAP_PIN_OUTPUT);
		omap_mux_init_gpio(162, OMAP_PIN_OUTPUT);
		omap_mux_init_gpio(145, OMAP_PIN_OUTPUT);	// backlight
		omap_mux_init_gpio(138, OMAP_PIN_INPUT);	// ext ant sensor (GPS) - neither PUP nor PDN
#endif
		
#if 0
	//	omap_mux_init_gpio(170, OMAP_PIN_INPUT);
		omap_mux_init_gpio(170, OMAP_PIN_OUTPUT);
		gpio_request(170, "DVI_nPD");
		gpio_direction_output(170, true);
		gpio_set_value(170, 0);	/* leave DVI powered down until it's needed ... */
		gpio_export(170, 0);	// no direction change
#endif
	
		printk(KERN_INFO "Revision GTA04A%d\n", omap3_gta04_version);
		//	omap_mux_init_gpio(156, OMAP_PIN_OUTPUT);	// inherit from U-Boot...
		gpio_request(146, "GPS_ON");
		gpio_direction_output(146, true);
		gpio_set_value(146, 0);	// off
		gpio_export(146, 0);	// no direction change
		
		// should be a backlight driver using PWM
		gpio_request(57, "LCD_BACKLIGHT");
		//	gpio_set_value(145, 1);	// on
		gpio_direction_output(57, true);
		gpio_export(57, 0);	// no direction change

#if 0	// mapped to gpio-keys driver
		gpio_request(7, "AUX_BUTTON");
		gpio_direction_input(7);
		gpio_export(7, 0);	// no direction change
#endif		
#if 0	// mapped to twl-powerbutton driver
		gpio_request(137, "POWER_BUTTON");
		gpio_direction_input(137);
		gpio_export(137, 0);	// no direction change
#endif
	
		//	omap_mux_init_signal("gpio138", OMAP_PIN_INPUT);	// gpio 138 - with no pullup/pull-down
		gpio_request(144, "EXT_ANT");
		gpio_direction_input(144);
		gpio_export(144, 0);	// no direction change

#if 0	// FIXME: needs TCA6507 driver
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
#endif
	
		// for a definition of the mux names see arch/arm/mach-omap2/mux34xx.c
		// the syntax of the first paramter to omap_mux_init_signal() is "muxname" or "m0name.muxname" (for ambiguous modes)
		// note: calling omap_mux_init_signal() overwrites the parameter string...
		
		omap_mux_init_signal("mcbsp3_clkx.uart2_tx", OMAP_PIN_OUTPUT);	// gpio 142 / GPS TX
		omap_mux_init_signal("mcbsp3_fsx.uart2_rx", OMAP_PIN_INPUT);	// gpio 143 / GPS RX

	usb_musb_init();
#if !defined(CONFIG_I2C_OMAP_GTA04A2)	// we don't have the controller chip on the A2 board
	usb_ehci_init(&ehci_pdata);
#endif
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

MACHINE_START(GTA04, "GTA04")
	/* Maintainer: Nikolaus Schaller - http://www.gta04.org */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap3_beagle_map_io,
	.init_irq	= omap3_beagle_init_irq,
	.init_machine	= omap3_beagle_init,
	.timer		= &omap_timer,
MACHINE_END
