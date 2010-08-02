/*
 * linux/arch/arm/mach-s3c2442/mach-gta02.c
 *
 * S3C2442 Machine Support for Openmoko GTA02 / FreeRunner.
 *
 * Copyright (C) 2006-2009 by Openmoko, Inc.
 * Authors: Harald Welte <laforge@openmoko.org>
 *          Andy Green <andy@openmoko.org>
 *          Werner Almesberger <werner@openmoko.org>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>

#include <linux/mmc/host.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/io.h>

#include <linux/i2c.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>

#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/mbc.h>
#include <linux/mfd/pcf50633/adc.h>
#include <linux/mfd/pcf50633/gpio.h>
#include <linux/mfd/pcf50633/pmic.h>
#include <linux/mfd/pcf50633/backlight.h>

#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/lis302dl.h>

#include <linux/leds.h>
#include <linux/leds_pwm.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/irq.h>
#include <asm/mach-types.h>

#include <mach/regs-irq.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>
#include <mach/fb.h>

#include <mach/spi.h>
#include <mach/spi-gpio.h>
#include <plat/usb-control.h>
#include <mach/regs-mem.h>
#include <mach/hardware.h>

#include <mach/ts.h>

#include <mach/gta02.h>

#include <plat/regs-serial.h>
#include <plat/nand.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/pm.h>
#include <plat/udc.h>
#include <plat/gpio-cfg.h>
#include <plat/gpio-core.h>
#include <plat/iic.h>

#include <mach/gta02-pm-gps.h>
#include <mach/gta02-pm-wlan.h>

#include <mach/gta02-fiq.h>

#include <linux/hdq.h>
#include <linux/bq27000_battery.h>
#include <linux/platform_battery.h>

#include <linux/jbt6k74.h>
#include <linux/glamofb.h>
#include <linux/mfd/glamo.h>

#define S3C2410_GPIONO(bank,offset) ((bank) + (offset))

#define S3C2410_GPIO_BANKD   (32*3)
#define S3C2410_GPIO_BANKG   (32*6)

#define S3C2410_GPG5          S3C2410_GPIONO(S3C2410_GPIO_BANKG, 5)
#define S3C2410_GPG6          S3C2410_GPIONO(S3C2410_GPIO_BANKG, 6)
#define S3C2410_GPG7          S3C2410_GPIONO(S3C2410_GPIO_BANKG, 7)
#define S3C2410_GPD12         S3C2410_GPIONO(S3C2410_GPIO_BANKD, 12)
#define S3C2410_GPD13         S3C2410_GPIONO(S3C2410_GPIO_BANKD, 13)

#define        BITBANG_CS_ACTIVE       1       /* normally nCS, active low */
#define        BITBANG_CS_INACTIVE     0

#define S3C_SYSTEM_REV_ATAG GTA02v6_SYSTEM_REV

static struct pcf50633 *gta02_pcf;

/*
 * This gets called every 1ms when we paniced.
 */

static long gta02_panic_blink(long count)
{
	long delay = 0;
	static long last_blink;
	static char led;

	/* Fast blink: 200ms period. */
	if (count - last_blink < 100)
		return 0;

	led ^= 1;
	gpio_direction_output(GTA02_GPIO_AUX_LED, led);

	last_blink = count;

	return delay;
}


static struct map_desc gta02_iodesc[] __initdata = {
	{
		.virtual	= 0xe0000000,
		.pfn		= __phys_to_pfn(S3C2410_CS3 + 0x01000000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
};

#define UCON (S3C2410_UCON_DEFAULT | S3C2443_UCON_RXERR_IRQEN)
#define ULCON (S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB)
#define UFCON (S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE)

static struct s3c2410_uartcfg gta02_uartcfgs[] = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= UCON,
		.ulcon		= ULCON,
		.ufcon		= UFCON,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= UCON,
		.ulcon		= ULCON,
		.ufcon		= UFCON,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= UCON,
		.ulcon		= ULCON,
		.ufcon		= UFCON,
	},
};

static struct platform_device gta02_pm_bt_dev = {
	.name = "gta02-pm-bt",
};

static struct platform_device gta02_pm_gps_dev = {
	.name = "gta02-pm-gps",
};

static struct platform_device gta02_pm_gsm_dev = {
	.name = "gta02-pm-gsm",
};

static struct platform_device gta02_pm_usbhost_dev = {
	.name = "gta02-pm-usbhost",
};

static struct platform_device gta02_pm_wlan_dev = {
	.name = "gta02-pm-wlan",
};

static struct regulator_consumer_supply gsm_supply_consumer = {
	.dev = &gta02_pm_gsm_dev.dev,
	.supply = "GSM",
};

static struct regulator_consumer_supply usbhost_supply_consumer = {
	.dev = &gta02_pm_usbhost_dev.dev,
	.supply = "USBHOST",
};

static struct regulator_init_data gsm_supply_init_data = {
	.constraints = {
		.min_uV = 3700000,
		.max_uV = 3700000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &gsm_supply_consumer,
};

static struct regulator_init_data usbhost_supply_init_data = {
	.constraints = {
		.min_uV = 3700000,
		.max_uV = 3700000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &usbhost_supply_consumer,
};

static struct fixed_voltage_config gsm_supply_config = {
	.supply_name = "GSM",
	.microvolts = 3700000,
	.gpio = GTA02_GPIO_PCF(PCF50633_GPIO2),
	.enable_high = 1,
	.init_data = &gsm_supply_init_data,
};

static struct fixed_voltage_config usbhost_supply_config = {
	.supply_name = "USBHOST",
	.microvolts = 3700000,
	.gpio = GTA02_GPIO_PCF(PCF50633_GPO),
	.enable_high = 1,
	.init_data = &usbhost_supply_init_data,
};

static struct platform_device gta02_gsm_supply_device = {
	.name = "reg-fixed-voltage",
	.id = 1,
	.dev = {
		.platform_data = &gsm_supply_config,
	},
};

static struct platform_device gta02_usbhost_supply_device = {
	.name = "reg-fixed-voltage",
	.id = 2,
	.dev = {
		.platform_data = &usbhost_supply_config,
	},
};

/*
 * we crank down SD Card clock dynamically when GPS is powered
 */

static int gta02_glamo_mci_use_slow(void)
{
	return gta02_pm_gps_is_on();
}

static void gta02_glamo_external_reset(int level)
{
	s3c2410_gpio_setpin(GTA02_GPIO_3D_RESET, level);
	s3c2410_gpio_cfgpin(GTA02_GPIO_3D_RESET, S3C2410_GPIO_OUTPUT);
}

struct spi_gpio_platform_data spigpio_platform_data = {
	.sck = GTA02_GPIO_GLAMO(10),
	.mosi = GTA02_GPIO_GLAMO(11),
	.miso = GTA02_GPIO_GLAMO(5),
	.num_chipselect = 1,
};

static struct platform_device spigpio_device = {
	.name = "spi_gpio",
	.id   = 2,
	.dev = {
		.platform_data = &spigpio_platform_data,
	},
};

static void gta02_glamo_registered(struct device *dev)
{
	spigpio_device.dev.parent = dev;
	platform_device_register(&spigpio_device);
}

static struct fb_videomode gta02_glamo_modes[] = {
	{
		.name = "480x640",
		.xres = 480,
		.yres = 640,
		.pixclock	= 40816,
		.left_margin	= 8,
		.right_margin	= 16,
		.upper_margin	= 2,
		.lower_margin	= 16,
		.hsync_len	= 8,
		.vsync_len	= 2,
		.vmode = FB_VMODE_NONINTERLACED,
	}, {
		.name = "240x320",
		.xres = 240,
		.yres = 320,
		.pixclock	= 40816,
		.left_margin	= 8,
		.right_margin	= 16,
		.upper_margin	= 2,
		.lower_margin	= 16,
		.hsync_len	= 8,
		.vsync_len	= 2,
		.vmode = FB_VMODE_NONINTERLACED,
	}
};

static struct glamo_fb_platform_data gta02_glamo_fb_pdata = {
	.width  = 43,
	.height = 58,

	.num_modes = ARRAY_SIZE(gta02_glamo_modes),
	.modes = gta02_glamo_modes,
};

static struct glamo_mmc_platform_data gta02_glamo_mmc_pdata = {
	.glamo_mmc_use_slow = gta02_glamo_mci_use_slow,
};

static struct glamo_gpio_platform_data gta02_glamo_gpio_pdata = {
	.base = GTA02_GPIO_GLAMO_BASE,
	.registered = gta02_glamo_registered,
};

static struct glamo_platform_data gta02_glamo_pdata = {
	.fb_data    = &gta02_glamo_fb_pdata,
	.mmc_data   = &gta02_glamo_mmc_pdata,
	.gpio_data  = &gta02_glamo_gpio_pdata,

	.osci_clock_rate = 32768,

	.glamo_external_reset = gta02_glamo_external_reset,
};

/* JBT6k74 display controller */
static void gta02_jbt6k74_probe_completed(struct device *dev)
 {
	pcf50633_bl_set_brightness_limit(gta02_pcf, 0x3f);
}
 
const static struct jbt6k74_platform_data jbt6k74_pdata = {
	.gpio_reset = GTA02_GPIO_GLAMO(4),
};

/*----------- SPI: Accelerometers attached to SPI of s3c244x ----------------- */

void gta02_lis302dl_suspend_io(struct lis302dl_info *lis, int resume)
{
	struct lis302dl_platform_data *pdata = lis->pdata;

	if (!resume) {
		 /*
		 * we don't want to power them with a high level
		 * because GSENSOR_3V3 is not up during suspend
		 */
		s3c2410_gpio_setpin(pdata->pin_chip_select, 0);
		s3c2410_gpio_setpin(pdata->pin_clk, 0);
		s3c2410_gpio_setpin(pdata->pin_mosi, 0);
		/* misnomer: it is a pullDOWN in 2442 */
		s3c2410_gpio_pullup(pdata->pin_miso, 1);
		return;
	}

	/* back to normal */
	s3c2410_gpio_setpin(pdata->pin_chip_select, 1);
	s3c2410_gpio_setpin(pdata->pin_clk, 1);
	/* misnomer: it is a pullDOWN in 2442 */
	s3c2410_gpio_pullup(pdata->pin_miso, 0);

	s3c2410_gpio_cfgpin(pdata->pin_chip_select, S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_cfgpin(pdata->pin_clk, S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_cfgpin(pdata->pin_mosi, S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_cfgpin(pdata->pin_miso, S3C2410_GPIO_INPUT);

}

struct lis302dl_platform_data lis302_pdata_top = {
		.name		= "lis302-1 (top)",
		.pin_chip_select= S3C2410_GPD12,
		.pin_clk	= S3C2410_GPG7,
		.pin_mosi	= S3C2410_GPG6,
		.pin_miso	= S3C2410_GPG5,
		.interrupt	= GTA02_IRQ_GSENSOR_1,
		.open_drain	= 1, /* altered at runtime by PCB rev */
		.lis302dl_suspend_io = gta02_lis302dl_suspend_io,
};

struct lis302dl_platform_data lis302_pdata_bottom = {
		.name		= "lis302-2 (bottom)",
		.pin_chip_select= S3C2410_GPD13,
		.pin_clk	= S3C2410_GPG7,
		.pin_mosi	= S3C2410_GPG6,
		.pin_miso	= S3C2410_GPG5,
		.interrupt	= GTA02_IRQ_GSENSOR_2,
		.open_drain	= 1, /* altered at runtime by PCB rev */
		.lis302dl_suspend_io = gta02_lis302dl_suspend_io,
};

static struct spi_board_info gta02_spi_board_info[] = {
	{
		.modalias	= "jbt6k74",
		.platform_data	= &jbt6k74_pdata,
		.controller_data = (void*)GTA02_GPIO_GLAMO(12),
		/* irq */
		.max_speed_hz	= 100 * 1000,
		.bus_num	= 2,
		.chip_select = 0
	},
	{
		.modalias	= "lis302dl",
		/* platform_data */
		.platform_data	= &lis302_pdata_top,
		/* controller_data */
		/* irq */
		.max_speed_hz	= 100 * 1000,
		.bus_num	= 3,
		.chip_select	= 0,
	},
	{
		.modalias	= "lis302dl",
		/* platform_data */
		.platform_data	= &lis302_pdata_bottom,
		/* controller_data */
		/* irq */
		.max_speed_hz	= 100 * 1000,
		.bus_num	= 3,
		.chip_select	= 1,
	},
};

static void gta02_lis302_chip_select(struct s3c2410_spigpio_info *info, int csid, int cs)
{

	/*
	 * Huh... "quirk"... CS on this device is not really "CS" like you can
	 * expect.
	 *
	 * When it is 0 it selects SPI interface mode.
	 * When it is 1 it selects I2C interface mode.
	 *
	 * Because we have 2 devices on one interface we have to make sure
	 * that the "disabled" device (actually in I2C mode) don't think we're
	 * talking to it.
	 *
	 * When we talk to the "enabled" device, the "disabled" device sees
	 * the clocks as I2C clocks, creating havoc.
	 *
	 * I2C sees MOSI going LOW while CLK HIGH as a START action, thus we
	 * must ensure this is never issued.
	 */

	int cs_gpio, other_cs_gpio;

	cs_gpio = csid ? S3C2410_GPD13 : S3C2410_GPD12;
	other_cs_gpio = (1 - csid) ? S3C2410_GPD13 : S3C2410_GPD12;


	if (cs == BITBANG_CS_ACTIVE) {
		s3c2410_gpio_setpin(other_cs_gpio, 1);
		s3c2410_gpio_setpin(cs_gpio, 1);
		s3c2410_gpio_setpin(info->pin_clk, 1);
		s3c2410_gpio_setpin(cs_gpio, 0);
	} else {
		s3c2410_gpio_setpin(cs_gpio, 1);
		s3c2410_gpio_setpin(other_cs_gpio, 1);
	}
}

static struct s3c2410_spigpio_info gta02_spigpio_cfg = {
	.pin_clk	= S3C2410_GPG7,
	.pin_mosi	= S3C2410_GPG6,
	.pin_miso	= S3C2410_GPG5,
	.bus_num	= 3,
	.num_chipselect	= 2,
	.chip_select	= gta02_lis302_chip_select,
	.non_blocking_transfer = 1,
};

static struct platform_device gta02_spi_gpio_dev = {
	.name		= "spi_s3c24xx_gpio",
	.dev = {
		.platform_data = &gta02_spigpio_cfg,
	},
};
 
static struct resource gta02_glamo_resources[] = {
	[0] = {
		.start	= S3C2410_CS1,
		.end	= S3C2410_CS1 + 0x1000000 - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= GTA02_IRQ_3D,
		.end	= GTA02_IRQ_3D,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= IRQ_BOARD_START,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device gta02_glamo_dev = {
	.name		= "glamo3362",
	.num_resources	= ARRAY_SIZE(gta02_glamo_resources),
	.resource	= gta02_glamo_resources,
	.dev		= {
		.platform_data	= &gta02_glamo_pdata,
	},
};


#ifdef CONFIG_CHARGER_PCF50633
/*
 * On GTA02 the 1A charger features a 48K resistor to 0V on the ID pin.
 * We use this to recognize that we can pull 1A from the USB socket.
 *
 * These constants are the measured pcf50633 ADC levels with the 1A
 * charger / 48K resistor, and with no pulldown resistor.
 */

#define ADC_NOM_CHG_DETECT_1A 6
#define ADC_NOM_CHG_DETECT_USB 43

static int gta02_get_charger_online_status(void)
{
	struct pcf50633 *pcf = gta02_pcf;

	return pcf50633_mbc_get_status(pcf) & PCF50633_MBC_USB_ONLINE;
}

static int gta02_get_charger_active_status(void)
{
	struct pcf50633 *pcf = gta02_pcf;

	return pcf50633_mbc_get_status(pcf) & PCF50633_MBC_USB_ACTIVE;
}

static void
gta02_configure_pmu_for_charger(struct pcf50633 *pcf, void *unused, int res)
{
	int ma;

	if (res < ((ADC_NOM_CHG_DETECT_USB + ADC_NOM_CHG_DETECT_1A) / 2))
		ma = 1000;
	else
		ma = 100;

	pcf50633_mbc_usb_curlim_set(pcf, ma);
}

static struct delayed_work gta02_charger_work;
static int gta02_usb_vbus_draw;

static void gta02_charger_worker(struct work_struct *work)
{
	if (gta02_usb_vbus_draw) {
		pcf50633_mbc_usb_curlim_set(gta02_pcf, gta02_usb_vbus_draw);
		return;
	}

#ifdef CONFIG_PCF50633_ADC
	pcf50633_adc_async_read(gta02_pcf,
				PCF50633_ADCC1_MUX_ADCIN1,
				PCF50633_ADCC1_AVERAGE_16,
				gta02_configure_pmu_for_charger,
				NULL);
#else
	/*
	 * If the PCF50633 ADC is disabled we fallback to a
	 * 100mA limit for safety.
	 */
	pcf50633_mbc_usb_curlim_set(pcf, 100);
#endif
}

#define GTA02_CHARGER_CONFIGURE_TIMEOUT ((3000 * HZ) / 1000)

static void gta02_pmu_event_callback(struct pcf50633 *pcf, int irq)
{
	if (irq == PCF50633_IRQ_USBINS) {
		schedule_delayed_work(&gta02_charger_work,
				      GTA02_CHARGER_CONFIGURE_TIMEOUT);

		return;
	}

	if (irq == PCF50633_IRQ_USBREM) {
		cancel_delayed_work_sync(&gta02_charger_work);
		gta02_usb_vbus_draw = 0;
	}
}

static void gta02_udc_vbus_draw(unsigned int ma)
{
	if (!gta02_pcf)
		return;

	gta02_usb_vbus_draw = ma;

	schedule_delayed_work(&gta02_charger_work,
			      GTA02_CHARGER_CONFIGURE_TIMEOUT);
}
#else /* !CONFIG_CHARGER_PCF50633 */
#define gta02_pmu_event_callback	NULL
#define gta02_udc_vbus_draw		NULL
#define gta02_get_charger_online_status	NULL
#define gta02_get_charger_active_status	NULL
#endif

/*
 * This is called when pc50633 is probed, unfortunately quite late in the
 * day since it is an I2C bus device. Here we can belatedly define some
 * platform devices with the advantage that we can mark the pcf50633 as the
 * parent. This makes them get suspended and resumed with their parent
 * the pcf50633 still around.
 */

static void gta02_pmu_attach_child_devices(struct pcf50633 *pcf);


static char *gta02_batteries[] = {
	"battery",
};

static struct regulator_consumer_supply ldo4_consumers[] = {
	{
		.dev = &gta02_pm_bt_dev.dev,
		.supply = "BT_3V2",
	},
};

static struct regulator_consumer_supply ldo5_consumers[] = {
	{
		.dev = &gta02_pm_gps_dev.dev,
		.supply = "RF_3V",
	},
};

static struct regulator_consumer_supply ldo6_consumers[] = {
	REGULATOR_SUPPLY("VDC", "spi2.0"),
	REGULATOR_SUPPLY("VDDIO", "spi2.0"),
};

static struct regulator_consumer_supply hcldo_consumers[] = {
	{
		.dev = &gta02_glamo_dev.dev,
		.supply = "SD_3V3",
	},
};

static struct pcf50633_bl_platform_data gta02_backlight_data = {
	.default_brightness = 0x3f,
	.default_brightness_limit = 0,
	.ramp_time = 3,
};

struct pcf50633_platform_data gta02_pcf_pdata = {
	.resumers = {
		[0] =	PCF50633_INT1_USBINS |
			PCF50633_INT1_USBREM |
			PCF50633_INT1_ALARM,
		[1] =	PCF50633_INT2_ONKEYF,
		[2] =	PCF50633_INT3_ONKEY1S,
		[3] =	PCF50633_INT4_LOWSYS |
			PCF50633_INT4_LOWBAT |
			PCF50633_INT4_HIGHTMP,
	},

	.batteries = gta02_batteries,
	.num_batteries = ARRAY_SIZE(gta02_batteries),

	.charger_reference_current_ma = 1000,

	.backlight_data = &gta02_backlight_data,

	.gpio_base = GTA02_GPIO_PCF_BASE,

	.reg_init_data = {
		[PCF50633_REGULATOR_AUTO] = {
			.constraints = {
				.min_uV = 3300000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.always_on = 1,
				.apply_uV = 1,
				.state_mem = {
					.enabled = 1,
				},
			},
		},
		[PCF50633_REGULATOR_DOWN1] = {
			.constraints = {
				.min_uV = 1300000,
				.max_uV = 1600000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.always_on = 1,
				.apply_uV = 1,
			},
		},
		[PCF50633_REGULATOR_DOWN2] = {
			.constraints = {
				.min_uV = 1800000,
				.max_uV = 1800000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
				.always_on = 1,
				.state_mem = {
					.enabled = 1,
				},
			},
		},
		[PCF50633_REGULATOR_HCLDO] = {
			.constraints = {
				.min_uV = 2000000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | 
						REGULATOR_CHANGE_STATUS,
			},
			.num_consumer_supplies = ARRAY_SIZE(hcldo_consumers),
			.consumer_supplies = hcldo_consumers,
		},
		[PCF50633_REGULATOR_LDO1] = {
			.constraints = {
				.min_uV = 3300000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.valid_ops_mask = REGULATOR_CHANGE_STATUS,
				.apply_uV = 1,
				.state_mem = {
					.enabled = 0,
				},
			},
		},
		[PCF50633_REGULATOR_LDO2] = {
			.constraints = {
				.min_uV = 3300000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
		},
		[PCF50633_REGULATOR_LDO3] = {
			.constraints = {
				.min_uV = 3000000,
				.max_uV = 3000000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
		},
		[PCF50633_REGULATOR_LDO4] = {
			.constraints = {
				.min_uV = 3200000,
				.max_uV = 3200000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.valid_ops_mask = REGULATOR_CHANGE_STATUS,
				.apply_uV = 1,
			},
			.num_consumer_supplies = ARRAY_SIZE(ldo4_consumers),
			.consumer_supplies = ldo4_consumers,
		},
		[PCF50633_REGULATOR_LDO5] = {
			.constraints = {
				.min_uV = 3000000,
				.max_uV = 3000000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.valid_ops_mask = REGULATOR_CHANGE_STATUS,
				.apply_uV = 1,
				.state_mem = {
					.enabled = 1,
				},
			},
			.num_consumer_supplies = ARRAY_SIZE(ldo5_consumers),
			.consumer_supplies = ldo5_consumers,
		},
		[PCF50633_REGULATOR_LDO6] = {
			.constraints = {
				.min_uV = 3000000,
				.max_uV = 3000000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			},
			.num_consumer_supplies = ARRAY_SIZE(ldo6_consumers),
			.consumer_supplies = ldo6_consumers,
		},
		[PCF50633_REGULATOR_MEMLDO] = {
			.constraints = {
				.min_uV = 1800000,
				.max_uV = 1800000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.state_mem = {
					.enabled = 1,
				},
			},
		},

	},
	.probe_done = gta02_pmu_attach_child_devices,
	.mbc_event_callback = gta02_pmu_event_callback,
};


/* NOR Flash. */

#define GTA02_FLASH_BASE	0x18000000 /* GCS3 */
#define GTA02_FLASH_SIZE	0x200000 /* 2MBytes */

static struct physmap_flash_data gta02_nor_flash_data = {
	.width		= 2,
};

static struct resource gta02_nor_flash_resource = {
	.start		= GTA02_FLASH_BASE,
	.end		= GTA02_FLASH_BASE + GTA02_FLASH_SIZE - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device gta02_nor_flash = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
		.platform_data	= &gta02_nor_flash_data,
	},
	.resource	= &gta02_nor_flash_resource,
	.num_resources	= 1,
};

static struct i2c_board_info gta02_i2c_devs[] __initdata = {
	{
		I2C_BOARD_INFO("pcf50633", 0x73),
		.irq = GTA02_IRQ_PCF50633,
		.platform_data = &gta02_pcf_pdata,
	},
	{
		I2C_BOARD_INFO("wm8753", 0x1a),
	},
};

static struct s3c2410_nand_set __initdata gta02_nand_sets[] = {
	[0] = {
		/*
		 * This name is also hard-coded in the boot loaders, so
		 * changing it would would require all users to upgrade
		 * their boot loaders, some of which are stored in a NOR
		 * that is considered to be immutable.
		 */
		.name		= "neo1973-nand",
		.nr_chips	= 1,
		.flash_bbt	= 1,
	},
};

/*
 * Choose a set of timings derived from S3C@2442B MCP54
 * data sheet (K5D2G13ACM-D075 MCP Memory).
 */

static struct s3c2410_platform_nand __initdata gta02_nand_info = {
	.tacls		= 0,
	.twrph0		= 25,
	.twrph1		= 15,
	.nr_sets	= ARRAY_SIZE(gta02_nand_sets),
	.sets		= gta02_nand_sets,
	.software_ecc	= 1,
};


static void gta02_udc_command(enum s3c2410_udc_cmd_e cmd)
{
	switch (cmd) {
	case S3C2410_UDC_P_ENABLE:
		pr_debug("%s S3C2410_UDC_P_ENABLE\n", __func__);
		gpio_set_value(GTA02_GPIO_USB_PULLUP, 1);
		break;
	case S3C2410_UDC_P_DISABLE:
		pr_debug("%s S3C2410_UDC_P_DISABLE\n", __func__);
		gpio_set_value(GTA02_GPIO_USB_PULLUP, 0);
		break;
	case S3C2410_UDC_P_RESET:
		pr_debug("%s S3C2410_UDC_P_RESET\n", __func__);
		/* FIXME: Do something here. */
	}
}

/* Get PMU to set USB current limit accordingly. */
static struct s3c2410_udc_mach_info gta02_udc_cfg = {
	.vbus_draw	= gta02_udc_vbus_draw,
	.udc_command	= gta02_udc_command,

};

/* USB */
static struct s3c2410_hcd_info gta02_usb_info __initdata = {
	.port[0]	= {
		.flags	= S3C_HCDFLG_USED,
	},
	.port[1]	= {
		.flags	= 0,
	},
};

/* Touchscreen */
static struct s3c2410_ts_mach_info gta02_ts_info = {
       .delay = 10000,
       .presc = 0xff, /* slow as we can go */
	   .oversampling_shift = 2,
};

/* Buttons */
static struct gpio_keys_button gta02_buttons[] = {
	{
		.gpio = GTA02_GPIO_AUX_KEY,
		.code = KEY_PHONE,
		.desc = "Aux",
		.type = EV_KEY,
		.debounce_interval = 100,
	},
	{
		.gpio = GTA02_GPIO_HOLD_KEY,
		.code = KEY_PAUSE,
		.desc = "Hold",
		.type = EV_KEY,
		.debounce_interval = 100,
	},
};

static struct gpio_keys_platform_data gta02_buttons_pdata = {
	.buttons = gta02_buttons,
	.nbuttons = ARRAY_SIZE(gta02_buttons),
};

static struct platform_device gta02_buttons_device = {
	.name = "gpio-keys",
	.id = -1,
	.dev = {
		.platform_data = &gta02_buttons_pdata,
	},
};

/* LEDs */
static struct gpio_led gta02_gpio_leds[] = {
	{
		.name	= "gta02:red:aux",
		.gpio	= GTA02_GPIO_AUX_LED,
	},
};

static struct gpio_led_platform_data gta02_gpio_leds_pdata = {
	.leds = gta02_gpio_leds,
	.num_leds = ARRAY_SIZE(gta02_gpio_leds),
};

static struct platform_device gta02_leds_device = {
	.name	= "leds-gpio",
	.id		= -1,
	.dev = {
		.platform_data = &gta02_gpio_leds_pdata,
	},
};

static inline int gta02_pwm_to_gpio(int pwm_id)
{
	return S3C2410_GPB(pwm_id);
}

static int gta02_pwm_led_init(struct device *dev, struct led_pwm *led)
{
	int ret;
	int gpio = gta02_pwm_to_gpio(led->pwm_id);

	ret = gpio_request(gpio, dev_name(dev));
	if (ret)
		return ret;

	gpio_direction_output(gpio, 0);

	return 0;
}

static enum led_brightness gta02_pwm_led_notify(struct device *dev,
	struct led_pwm *led, enum led_brightness brightness)
{
	int gpio = gta02_pwm_to_gpio(led->pwm_id);

	if (brightness == led->max_brightness || brightness == 0) {
		s3c2410_gpio_cfgpin(gpio, S3C2410_GPIO_OUTPUT);
		gpio_set_value(gpio, brightness ? 1 : 0);

		brightness = 0;
	} else {
		s3c2410_gpio_cfgpin(gpio, S3C2410_GPIO_SFN2);
	}

	return brightness;
}

static void gta02_pwm_led_exit(struct device *dev, struct led_pwm *led)
{
	gpio_free(gta02_pwm_to_gpio(led->pwm_id));
}

static struct led_pwm gta02_pwm_leds[] = {
	{
		.name = "gta02:orange:power",
		.max_brightness = 0xff,
		.pwm_period_ns = 1000000,
		.pwm_id = 0,
	},
	{
		.name = "gta02:blue:power",
		.max_brightness = 0xff,
		.pwm_period_ns = 1000000,
		.pwm_id = 1,
	},
	{
		.name = "gta02::vibrator",
		.max_brightness = 0x3f,
		.pwm_period_ns = 60000000,
		.pwm_id = 3,
	}
};

static struct led_pwm_platform_data gta02_pwm_leds_pdata = {
	.num_leds = ARRAY_SIZE(gta02_pwm_leds),
	.leds = gta02_pwm_leds,

	.init = gta02_pwm_led_init,
	.notify = gta02_pwm_led_notify,
	.exit = gta02_pwm_led_exit,
};

static struct platform_device gta02_pwm_leds_device = {
	.name	= "leds_pwm",
	.id	= -1,
	.dev = {
		.platform_data = &gta02_pwm_leds_pdata,
	}
};

/* BQ27000 Battery */

struct bq27000_platform_data bq27000_pdata = {
	.name = "battery",
	.rsense_mohms = 20,
	.hdq_read = hdq_read,
	.hdq_write = hdq_write,
	.hdq_initialized = hdq_initialized,
	.get_charger_online_status = gta02_get_charger_online_status,
	.get_charger_active_status = gta02_get_charger_active_status
};

struct platform_device bq27000_battery_device = {
	.name 		= "bq27000-battery",
	.dev = {
		.platform_data = &bq27000_pdata,
	},
};

/* Platform battery */

/* Capacity of a typical BL-5C dumb battery */
#define GTA02_BAT_CHARGE_FULL	850000

static int gta02_bat_voltscale(int volt)
{
	/* This table is suggested by SpeedEvil based on analysis of
	 * experimental data */
	static const int lut[][2] = {
		{ 4120, 100 },
		{ 3900, 60 },
		{ 3740, 25 },
		{ 3600, 5 },
		{ 3000, 0 } };
	int i, res = 0;

	if (volt > lut[0][0])
		res = lut[0][1];
	else
		for (i = 0; lut[i][1]; i++) {
			if (volt <= lut[i][0] && volt >= lut[i+1][0]) {
				res = lut[i][1] - (lut[i][0]-volt)*
					(lut[i][1]-lut[i+1][1])/
					(lut[i][0]-lut[i+1][0]);
				break;
			}
		}
	return res;
}

static int gta02_bat_get_voltage(void)
{
	struct pcf50633 *pcf = gta02_pcf;
	u16 adc, mv = 0;
	adc = pcf50633_adc_sync_read(pcf,
		PCF50633_ADCC1_MUX_BATSNS_RES,
		PCF50633_ADCC1_AVERAGE_16);
	/* The formula from DS is for divide-by-two mode, current driver uses
	divide-by-three */
	mv = (adc * 6000) / 1023;
	return mv * 1000;
}

static int gta02_bat_get_present(void)
{
	/* There is no reliable way to tell if it is present or not */
	return 1;
}

static int gta02_bat_get_status(void)
{
#ifdef CONFIG_CHARGER_PCF50633
	if (gta02_get_charger_active_status())
		return POWER_SUPPLY_STATUS_CHARGING;
	else
		return POWER_SUPPLY_STATUS_DISCHARGING;
#else
	return POWER_SUPPLY_STATUS_UNKNOWN;
#endif
}

static int gta02_bat_get_capacity(void)
{
	return gta02_bat_voltscale(gta02_bat_get_voltage()/1000);
}

static int gta02_bat_get_charge_full(void)
{
	return GTA02_BAT_CHARGE_FULL;
}

static int gta02_bat_get_charge_now(void)
{
	return gta02_bat_get_capacity() * gta02_bat_get_charge_full() / 100;
}

static enum power_supply_property gta02_platform_bat_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
};

int (*gta02_platform_bat_get_property[])(void) = {
	gta02_bat_get_present,
	gta02_bat_get_status,
	gta02_bat_get_voltage,
	gta02_bat_get_capacity,
	gta02_bat_get_charge_full,
	gta02_bat_get_charge_now,
};

static struct platform_bat_platform_data gta02_platform_bat_pdata = {
	.name = "battery",
	.properties = gta02_platform_bat_properties,
	.num_properties = ARRAY_SIZE(gta02_platform_bat_properties),
	.get_property = gta02_platform_bat_get_property,
	.is_present = gta02_bat_get_present,
};

struct platform_device gta02_platform_bat = {
	.name = "platform_battery",
	.id = -1,
	.dev = {
		.platform_data = &gta02_platform_bat_pdata,
	}
};

/* HDQ */

static void gta02_hdq_gpio_direction_out(void)
{
	s3c2410_gpio_cfgpin(GTA02v5_GPIO_HDQ, S3C2410_GPIO_OUTPUT);
}

static void gta02_hdq_gpio_direction_in(void)
{
	s3c2410_gpio_cfgpin(GTA02v5_GPIO_HDQ, S3C2410_GPIO_INPUT);
}

static void gta02_hdq_gpio_set_value(int val)
{
	s3c2410_gpio_setpin(GTA02v5_GPIO_HDQ, val);
}

static int gta02_hdq_gpio_get_value(void)
{
	return s3c2410_gpio_getpin(GTA02v5_GPIO_HDQ);
}

struct hdq_platform_data gta02_hdq_platform_data = {
	.gpio_dir_out = gta02_hdq_gpio_direction_out,
	.gpio_dir_in = gta02_hdq_gpio_direction_in,
	.gpio_set = gta02_hdq_gpio_set_value,
	.gpio_get = gta02_hdq_gpio_get_value,

	.enable_fiq = gta02_fiq_enable,
	.disable_fiq = gta02_fiq_disable,
	.kick_fiq = gta02_fiq_kick,
};

struct platform_device gta02_hdq_device = {
	.name 		= "hdq",
	.id		= -1,
	.dev		= {
		.platform_data = &gta02_hdq_platform_data,
		.parent = &s3c_device_timer[2].dev,
	},
};

static void __init gta02_map_io(void)
{
	s3c24xx_init_io(gta02_iodesc, ARRAY_SIZE(gta02_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(gta02_uartcfgs, ARRAY_SIZE(gta02_uartcfgs));
}


/* These are the guys that don't need to be children of PMU. */

static struct platform_device *gta02_devices[] __initdata = {
	&s3c_device_ohci,
	&s3c_device_wdt,
	&s3c_device_sdi,
	&s3c_device_usbgadget,
	&s3c_device_nand,
	&gta02_nor_flash,
	&s3c_device_timer[0],
	&s3c_device_timer[1],
	&s3c_device_timer[2],
	&s3c_device_timer[3],
	&s3c_device_iis,
	&s3c_device_i2c0,
	&gta02_buttons_device,
	&gta02_leds_device,
	&gta02_pwm_leds_device,
	&gta02_pm_gps_dev,
	&gta02_pm_bt_dev,
	&gta02_pm_wlan_dev,
	&gta02_glamo_dev,
	&gta02_spi_gpio_dev,
	&s3c_device_adc,
	&s3c_device_ts,
};

/* These guys DO need to be children of PMU. */

static struct platform_device *gta02_devices_pmu_children[] = {
    &gta02_hdq_device,
	&gta02_platform_bat,
};

/*
 * This is called when pc50633 is probed, quite late in the day since it is an
 * I2C bus device.  Here we can define platform devices with the advantage that
 * we can mark the pcf50633 as the parent.  This makes them get suspended and
 * resumed with their parent the pcf50633 still around.  All devices whose
 * operation depends on something from pcf50633 must have this relationship
 * made explicit like this, or suspend and resume will become an unreliable
 * hellworld.
 */

static void gta02_pmu_attach_child_devices(struct pcf50633 *pcf)
{
	int n;

	/* Grab a copy of the now probed PMU pointer. */
	gta02_pcf = pcf;

	for (n = 0; n < ARRAY_SIZE(gta02_devices_pmu_children); n++)
		gta02_devices_pmu_children[n]->dev.parent = pcf->dev;

	platform_add_devices(gta02_devices_pmu_children,
			     ARRAY_SIZE(gta02_devices_pmu_children));
}

static void gta02_poweroff(void)
{
	pcf50633_reg_set_bit_mask(gta02_pcf, PCF50633_REG_OOCSHDWN, 1, 1);
}

struct gta02_device_children {
	const char *dev_name;
	size_t num_children;
	struct platform_device **children;
	void (*probed_callback)(struct device *dev);
};

static struct platform_device* gta02_pcf50633_gpio_children[] = {
	&gta02_gsm_supply_device,
	&gta02_usbhost_supply_device,
};

static struct platform_device* gta02_gsm_supply_children[] = {
	&gta02_pm_gsm_dev,
};

static struct platform_device* gta02_usbhost_supply_children[] = {
	&gta02_pm_usbhost_dev,
};

static struct platform_device* gta02_hdq_children[] = {
	&bq27000_battery_device,
};


static struct gta02_device_children gta02_device_children[] = {
	{
		.dev_name = "pcf50633-gpio.0",
		.num_children = 2,
		.children = gta02_pcf50633_gpio_children,
	},
	{
		.dev_name = "reg-fixed-voltage.1",
		.num_children = 1,
		.children = gta02_gsm_supply_children,
	},
	{
		.dev_name = "reg-fixed-voltage.2",
		.num_children = 1,
		.children = gta02_usbhost_supply_children,
	},
	{
		.dev_name = "spi2.0",
		.probed_callback = gta02_jbt6k74_probe_completed,
	},
	{
		.dev_name = "hdq",
		.num_children = 1,
		.children = gta02_hdq_children,
	},
};

static int gta02_add_child_devices(struct device *parent,
                                   struct platform_device **children,
								   size_t num_children)
{
	size_t i;

	for (i = 0; i < num_children; ++i)
		children[i]->dev.parent = parent;

	return platform_add_devices(children, num_children);
}

static int gta02_device_registered(struct notifier_block *block,
                                   unsigned long action, void *data)
{
	struct device *dev = data;
	const char *devname = dev_name(dev);
	size_t i;

	if (action != BUS_NOTIFY_BOUND_DRIVER)
		return 0;

	for (i = 0; i < ARRAY_SIZE(gta02_device_children); ++i) {
		if (strcmp(devname, gta02_device_children[i].dev_name) == 0) {
			gta02_add_child_devices(dev, gta02_device_children[i].children,
			gta02_device_children[i].num_children);

			if (gta02_device_children[i].probed_callback)
				gta02_device_children[i].probed_callback(dev);
			break;
		}
	}

	return 0;
}

static struct notifier_block gta02_device_register_notifier = {
	.notifier_call = gta02_device_registered,
	.priority = INT_MAX,
};


/* On hardware rev 5 and earlier the leds are missing a resistor and reading
 * from their gpio pins will always return 0, so we have to shadow the
 * led states software */
static unsigned long gpb_shadow;
extern struct s3c_gpio_chip s3c24xx_gpios[];

static void gta02_gpb_set(struct gpio_chip *chip,
				unsigned offset, int value)
{
	void __iomem *base = S3C24XX_GPIO_BASE(S3C2410_GPB(0));
	unsigned long flags;
	unsigned long dat;

	local_irq_save(flags);

	dat = __raw_readl(base + 0x04) | gpb_shadow;
	dat &= ~(1 << offset);
	gpb_shadow &= ~(1 << offset);
	if (value) {
		dat |= 1 << offset;
		switch (offset) {
		case 0 ... 2:
			gpb_shadow |= 1 << offset;
			break;
		default:
			break;
		}
	}
	__raw_writel(dat, base + 0x04);

	local_irq_restore(flags);
}

static int gta02_gpb_get(struct gpio_chip *chip, unsigned offset)
{
	void __iomem *base = S3C24XX_GPIO_BASE(S3C2410_GPB(0));
	unsigned long val;

	val = __raw_readl(base + 0x04) | gpb_shadow;
	val >>= offset;
	val &= 1;

	return val;
}

static void gta02_hijack_gpb(void)
{
/* Uncomment this, once support for S3C_SYSTEM_REV_ATAG has been merged
 * upstream.
	if (S3C_SYSTEM_REV_ATAG > GTA02v5_SYSTEM_REV)
		return;
*/

	s3c24xx_gpios[1].chip.set = gta02_gpb_set;
	s3c24xx_gpios[1].chip.get = gta02_gpb_get;
}

/*
 * Allow the bootloader to enable hw ecc
 * hardware_ecc=1|0
 */
static int __init hardware_ecc_setup(char *str)
{
	if (str && str[0] == '1')
		gta02_nand_info.software_ecc = 0;
	return 1;
}
__setup("hardware_ecc=", hardware_ecc_setup);

static void gta02_request_gpios(void)
{
	int ret;
	ret = gpio_request(GTA02_GPIO_USB_PULLUP, "USB pullup");
	if (ret) {
		printk(KERN_ERR "Failed to request USB pullup gpio pin: %d\n", ret);
	} else {
		ret = gpio_direction_output(GTA02_GPIO_USB_PULLUP, 0);
		if (ret)
			printk(KERN_ERR "Failed to set USB pullup gpio direction: %d\n", ret);
    }
}

static void __init gta02_machine_init(void)
{
	/* Set the panic callback to make AUX LED blink at ~5Hz. */
	panic_blink = gta02_panic_blink;

	switch (S3C_SYSTEM_REV_ATAG) {
		case GTA02v6_SYSTEM_REV:
		/* we need push-pull interrupt from motion sensors */
		lis302_pdata_top.open_drain = 0;
		lis302_pdata_bottom.open_drain = 0;
		break;
	default:
		break;
	}

	bus_register_notifier(&platform_bus_type, &gta02_device_register_notifier);
	bus_register_notifier(&spi_bus_type, &gta02_device_register_notifier);

	gta02_hijack_gpb();

	gta02_request_gpios();

	s3c_pm_init();

#ifdef CONFIG_CHARGER_PCF50633
	INIT_DELAYED_WORK(&gta02_charger_work, gta02_charger_worker);
#endif

	s3c24xx_udc_set_platdata(&gta02_udc_cfg);
	s3c24xx_ts_set_platdata(&gta02_ts_info);
	s3c_ohci_set_platdata(&gta02_usb_info);
	s3c_nand_set_platdata(&gta02_nand_info);
	s3c_i2c0_set_platdata(NULL);
	spi_register_board_info(gta02_spi_board_info,
				ARRAY_SIZE(gta02_spi_board_info));

	i2c_register_board_info(0, gta02_i2c_devs, ARRAY_SIZE(gta02_i2c_devs));

	platform_add_devices(gta02_devices, ARRAY_SIZE(gta02_devices));

	pm_power_off = gta02_poweroff;
}


MACHINE_START(NEO1973_GTA02, "GTA02")
	/* Maintainer: Nelson Castillo <arhuaco@freaks-unidos.net> */
	.phys_io	= S3C2410_PA_UART,
	.io_pg_offst	= (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C2410_SDRAM_PA + 0x100,
	.map_io		= gta02_map_io,
	.init_irq	= s3c24xx_init_irq,
	.init_machine	= gta02_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
