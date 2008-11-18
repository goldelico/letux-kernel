/*
 * linux/arch/arm/mach-s3c2440/mach-glofiish.c
 *
 * S3C2440 Machine Support for the E-TEN glofiish X800/M800
 *
 * Copyright (C) 2008 by Harald Welte <laforge@gnumonks.org>
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
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/mmc/host.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <asm/arch-s3c2410/regs-irq.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-gpioj.h>
#include <asm/arch/fb.h>
#include <asm/arch/mci.h>
#include <asm/arch/ts.h>
#include <asm/arch/spi.h>
#include <asm/arch/spi-gpio.h>
#include <asm/arch/usb-control.h>

#include <asm/arch/glofiish.h>
#include <asm/arch/gta01.h>

#include <asm/plat-s3c/regs-serial.h>
#include <asm/plat-s3c/nand.h>
#include <asm/plat-s3c24xx/devs.h>
#include <asm/plat-s3c24xx/cpu.h>
#include <asm/plat-s3c24xx/pm.h>
#include <asm/plat-s3c24xx/udc.h>

#include <linux/jbt6k74.h>

/*
 * this gets called every 1ms when we paniced.
 */

static long glofiish_panic_blink(long count)
{
	long delay = 0;
	static long last_blink;
	static char led;

	if (count - last_blink < 100) /* 200ms period, fast blink */
		return 0;

	/* FIXME */
#if 0
	led ^= 1;
	s3c2410_gpio_cfgpin(GTA02_GPIO_AUX_LED, S3C2410_GPIO_OUTPUT);
	neo1973_gpb_setpin(GTA02_GPIO_AUX_LED, led);

	last_blink = count;
#endif
	return delay;
}

struct platform_device gta02_version_device = {
	.name 		= "neo1973-version",
	.num_resources	= 0,
};

struct platform_device gta02_resume_reason_device = {
	.name 		= "neo1973-resume",
	.num_resources	= 0,
};

struct platform_device gta02_memconfig_device = {
	.name 		= "neo1973-memconfig",
	.num_resources	= 0,
};

static struct map_desc m800_iodesc[] __initdata = {
	{
		.virtual	= 0xe0000000,
		.pfn		= __phys_to_pfn(S3C2410_CS3+0x01000000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
};

#define UCON S3C2410_UCON_DEFAULT
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

static struct s3c2410_uartcfg m800_uartcfgs[] = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},

};

/* Configuration for 480x640 toppoly TD028TTEC1.
 * Do not mark this as __initdata or it will break! */
static struct s3c2410fb_display glofiish_displays[] =  {
	{
		.type		= S3C2410_LCDCON1_TFT,
		.width		= 43,
		.height		= 58,
		.xres		= 480,
		.yres		= 640,
		.bpp		= 16,

		.pixclock	= 40000,	/* HCLK/4 */
		.left_margin	= 2,
		.right_margin	= 2,
		.hsync_len	= 2,
		.upper_margin	= 2,
		.lower_margin	= 66,
		.vsync_len	= 2,
		.lcdcon5	= S3C2410_LCDCON5_FRM565 |
				  S3C2410_LCDCON5_INVVCLK |
				  S3C2410_LCDCON5_INVVLINE |
				  S3C2410_LCDCON5_INVVFRAME |
				  S3C2410_LCDCON5_PWREN |
				  S3C2410_LCDCON5_HWSWP,
	},
	{
		.type		= S3C2410_LCDCON1_TFT,
		.width		= 43,
		.height		= 58,
		.xres		= 480,
		.yres		= 640,
		.bpp		= 32,

		.pixclock	= 40000,	/* HCLK/4 */
		.left_margin	= 104,
		.right_margin	= 8,
		.hsync_len	= 8,
		.upper_margin	= 2,
		.lower_margin	= 16,
		.vsync_len	= 2,
		.lcdcon5	= S3C2410_LCDCON5_FRM565 |
				  S3C2410_LCDCON5_INVVCLK |
				  S3C2410_LCDCON5_INVVLINE |
				  S3C2410_LCDCON5_INVVFRAME |
				  S3C2410_LCDCON5_PWREN |
				  S3C2410_LCDCON5_HWSWP,
	},
	{
		.type		= S3C2410_LCDCON1_TFT,
		.width		= 43,
		.height		= 58,
		.xres		= 240,
		.yres		= 320,
		.bpp		= 16,

		.pixclock	= 40000,	/* HCLK/4 */
		.left_margin	= 104,
		.right_margin	= 8,
		.hsync_len	= 8,
		.upper_margin	= 2,
		.lower_margin	= 16,
		.vsync_len	= 2,
		.lcdcon5	= S3C2410_LCDCON5_FRM565 |
				  S3C2410_LCDCON5_INVVCLK |
				  S3C2410_LCDCON5_INVVLINE |
				  S3C2410_LCDCON5_INVVFRAME |
				  S3C2410_LCDCON5_PWREN |
				  S3C2410_LCDCON5_HWSWP,
	},
};

static struct s3c2410fb_mach_info glofiish_lcd_cfg __initdata = {
	.displays	= glofiish_displays,
	.num_displays	= ARRAY_SIZE(glofiish_displays),
	.default_display = 0,

	.lpcsel		= ((0xCE6) & ~7) | 1<<4,
};


static struct resource m800_sdio_resources[] = {
	[0] = {
		.flags	= IORESOURCE_IRQ,
		.start	= IRQ_SDI,
		.end	= IRQ_SDI,
	},
	[1] = {
		.flags = IORESOURCE_MEM,
		.start = S3C2410_PA_SDI,
		.end   = S3C2410_PA_SDI + S3C24XX_SZ_SDI - 1,
	},
	[2] = {
		.flags = IORESOURCE_DMA,
		.start = 0, /* Channel 0 for SDI */
		.end = 0,
	},
};

static struct platform_device *glofiish_devices[] __initdata = {
	&s3c_device_usb,
	&s3c_device_lcd,
	&s3c_device_wdt,
	&s3c_device_i2c,
	&s3c_device_iis,
	&s3c_device_sdi,
	&s3c_device_usbgadget,
	&s3c_device_nand,
	&s3c_device_ts,
};

static struct s3c2410_nand_set glofiish_nand_sets[] = {
	[0] = {
		.name		= "glofiish-nand",
		.nr_chips	= 1,
		.flags		= S3C2410_NAND_BBT,
	},
};

/* choose a set of timings derived from S3C@2442B MCP54 
 * data sheet (K5D2G13ACM-D075 MCP Memory)
 */

static struct s3c2410_platform_nand glofiish_nand_info = {
	.tacls		= 0,
	.twrph0		= 25,
	.twrph1		= 15,
	.nr_sets	= ARRAY_SIZE(glofiish_nand_sets),
	.sets		= glofiish_nand_sets,
	.software_ecc	= 1,
};

static struct s3c24xx_mci_pdata glofiish_mmc_cfg = {
	.gpio_detect	= M800_GPIO_nSD_DETECT,
	.set_power	= NULL,
	.ocr_avail	= MMC_VDD_32_33,
};

static void glofiish_udc_command(enum s3c2410_udc_cmd_e cmd)
{
	printk(KERN_DEBUG "%s(%d)\n", __func__, cmd);

	switch (cmd) {
	case S3C2410_UDC_P_ENABLE:
		s3c2410_gpio_setpin(M800_GPIO_USB_PULLUP, 1);
		break;
	case S3C2410_UDC_P_DISABLE:
		s3c2410_gpio_setpin(M800_GPIO_USB_PULLUP, 0);
		break;
	case S3C2410_UDC_P_RESET:
		/* FIXME! */
		break;
	default:
		break;
	}
}

/* get PMU to set USB current limit accordingly */

static void glofiish_udc_vbus_draw(unsigned int ma)
{
	//pcf50633_notify_usb_current_limit_change(pcf50633_global, ma);
}

static struct s3c2410_udc_mach_info glofiish_udc_cfg = {
	.vbus_draw	= glofiish_udc_vbus_draw,
	.udc_command	= glofiish_udc_command,

};

static struct s3c2410_ts_mach_info glofiish_ts_cfg = {
	.delay = 10000,
	.presc = 50000000 / 1000000, /* 50 MHz PCLK / 1MHz */
	/* simple averaging, 2^n samples */
	.oversampling_shift = 5,
	 /* averaging filter length, 2^n */
	.excursion_filter_len_bits = 5,
	/* flagged for beauty contest on next sample if differs from
	 * average more than this
	 */
	.reject_threshold_vs_avg = 2,
};


/* SPI: LCM control interface attached to Glamo3362 */

static void m800_jbt6k74_reset(int devidx, int level)
{
	//glamo_lcm_reset(level);
	printk(KERN_DEBUG "gta01_jbt6k74_reset\n");
}

/* finally bring up deferred backlight resume now LCM is resumed itself */

static void m800_jbt6k74_resuming(int devidx)
{
	//pcf50633_backlight_resume(pcf50633_global);
	//gta01bl_deferred_resume();
}

const struct jbt6k74_platform_data jbt6k74_pdata = {
	.reset		= m800_jbt6k74_reset,
	.resuming	= m800_jbt6k74_resuming,
};

static struct spi_board_info glofiish_spi_board_info[] = {
	{
		.modalias	= "jbt6k74",
		/* platform_data */
		.platform_data	= &jbt6k74_pdata,
		/* controller_data */
		/* irq */
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 2,
		/* chip_select */
	},
};

static void spi_gpio_cs(struct s3c2410_spigpio_info *spi, int csidx, int cs)
{
	switch (cs) {
	case BITBANG_CS_ACTIVE:
		s3c2410_gpio_setpin(S3C2410_GPG3, 0);
		break;
	case BITBANG_CS_INACTIVE:
		s3c2410_gpio_setpin(S3C2410_GPG3, 1);
		break;
	}
}

static struct s3c2410_spigpio_info spi_gpio_cfg = {
	.pin_clk	= S3C2410_GPG7,
	.pin_mosi	= S3C2410_GPG6,
	.pin_miso	= S3C2410_GPG5,
	.board_size	= ARRAY_SIZE(glofiish_spi_board_info),
	.board_info	= glofiish_spi_board_info,
	.chip_select	= &spi_gpio_cs,
	.num_chipselect = 2,   /*** Should be 1 or 2 for gta01? ***/
};

static struct resource s3c_spi_lcm_resource[] = {
	[0] = {
		.start = S3C2410_GPG3,
		.end   = S3C2410_GPG3,
	},
	[1] = {
		.start = S3C2410_GPG5,
		.end   = S3C2410_GPG5,
	},
	[2] = {
		.start = S3C2410_GPG6,
		.end   = S3C2410_GPG6,
	},
	[3] = {
		.start = S3C2410_GPG7,
		.end   = S3C2410_GPG7,
	},
};

struct platform_device s3c_device_spi_lcm = {
	.name		  = "spi_s3c24xx_gpio",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(s3c_spi_lcm_resource),
	.resource	  = s3c_spi_lcm_resource,
	.dev = {
		.platform_data = &spi_gpio_cfg,
	},
};

static struct gta01bl_machinfo backlight_machinfo = {
	.default_intensity	= 1,
	.max_intensity		= 1,
	.limit_mask		= 1,
	.defer_resume_backlight	= 1,
};

static struct resource gta01_bl_resources[] = {
	[0] = {
		.start	= M800_GPIO_BACKLIGHT,
		.end	= M800_GPIO_BACKLIGHT,
	},
};

struct platform_device gta01_bl_dev = {
	.name 		= "gta01-bl",
	.num_resources	= ARRAY_SIZE(gta01_bl_resources),
	.resource	= gta01_bl_resources,
	.dev		= {
		.platform_data = &backlight_machinfo,
	},
};


#if 0 /* currently this is not used and we use gpio spi */
static struct glamo_spi_info glamo_spi_cfg = {
	.board_size	= ARRAY_SIZE(gta02_spi_board_info),
	.board_info	= gta02_spi_board_info,
};
#endif /* 0 */

#if 0
static struct resource gta02_vibrator_resources[] = {
	[0] = {
		.start	= GTA02_GPIO_VIBRATOR_ON,
		.end	= GTA02_GPIO_VIBRATOR_ON,
	},
};

static struct platform_device gta02_vibrator_dev = {
	.name		= "neo1973-vibrator",
	.num_resources	= ARRAY_SIZE(gta02_vibrator_resources),
	.resource	= gta02_vibrator_resources,
};

static struct resource gta02_led_resources[] = {
	{
		.name	= "gta02-power:orange",
		.start	= GTA02_GPIO_PWR_LED1,
		.end	= GTA02_GPIO_PWR_LED1,
	}, {
		.name	= "gta02-power:blue",
		.start	= GTA02_GPIO_PWR_LED2,
		.end	= GTA02_GPIO_PWR_LED2,
	}, {
		.name	= "gta02-aux:red",
		.start	= GTA02_GPIO_AUX_LED,
		.end	= GTA02_GPIO_AUX_LED,
	},
};

struct platform_device gta02_led_dev = {
	.name		= "gta02-led",
	.num_resources	= ARRAY_SIZE(gta02_led_resources),
	.resource	= gta02_led_resources,
};

static struct resource gta02_button_resources[] = {
	[0] = {
		.start = GTA02_GPIO_AUX_KEY,
		.end   = GTA02_GPIO_AUX_KEY,
	},
	[1] = {
		.start = GTA02_GPIO_HOLD_KEY,
		.end   = GTA02_GPIO_HOLD_KEY,
	},
	[2] = {
		.start = GTA02_GPIO_JACK_INSERT,
		.end   = GTA02_GPIO_JACK_INSERT,
	},
};

static struct platform_device gta02_button_dev = {
	.name		= "neo1973-button",
	.num_resources	= ARRAY_SIZE(gta02_button_resources),
	.resource	= gta02_button_resources,
};
#endif

/* USB */
static struct s3c2410_hcd_info glofiish_usb_info = {
	.port[0]	= {
		.flags	= S3C_HCDFLG_USED,
	},
	.port[1]	= {
		.flags	= 0,
	},
};

static struct resource m800_button_resources[] = {
	[0] = {
		.start = M800_GPIO_nKEY_POWER,
		.end   = M800_GPIO_nKEY_POWER,
	},
	[1] = {
		.start = M800_GPIO_nKEY_CAMERA,
		.end   = M800_GPIO_nKEY_CAMERA,
	},
	[3] = {
		.start = M800_GPIO_nKEY_RECORD,
		.end   = M800_GPIO_nKEY_RECORD,
	},
	[2] = {
		.start = M800_GPIO_SLIDE,
		.end   = M800_GPIO_SLIDE,
	},
};

static struct platform_device m800_button_dev = {
	.name		= "m800-button",
	.num_resources	= ARRAY_SIZE(m800_button_resources),
	.resource	= m800_button_resources,
};

static struct platform_device m800_pm_bt_dev = {
	.name		= "neo1973-pm-bt",
};

static void __init glofiish_map_io(void)
{
	s3c24xx_init_io(m800_iodesc, ARRAY_SIZE(m800_iodesc));
	s3c24xx_init_clocks(16934400);
	s3c24xx_init_uarts(m800_uartcfgs, ARRAY_SIZE(m800_uartcfgs));
}

static irqreturn_t gta02_modem_irq(int irq, void *param)
{
	printk(KERN_DEBUG "modem wakeup interrupt\n");
	return IRQ_HANDLED;
}

static irqreturn_t ar6000_wow_irq(int irq, void *param)
{
	printk(KERN_DEBUG "ar6000_wow interrupt\n");
	return IRQ_HANDLED;
}

/*
 * hardware_ecc=1|0
 */
static char hardware_ecc_str[4] __initdata = "";

static int __init hardware_ecc_setup(char *str)
{
	if (str)
		strlcpy(hardware_ecc_str, str, sizeof(hardware_ecc_str));
	return 1;
}

__setup("hardware_ecc=", hardware_ecc_setup);

static void __init glofiish_machine_init(void)
{
	int rc;

	/* set the panic callback to make AUX blink fast */
	panic_blink = glofiish_panic_blink;

	/* do not force soft ecc if we are asked to use hardware_ecc */
	if (hardware_ecc_str[0] == '1')
		glofiish_nand_info.software_ecc = 0;

	s3c_device_usb.dev.platform_data = &glofiish_usb_info;
	s3c_device_nand.dev.platform_data = &glofiish_nand_info;
	s3c_device_sdi.dev.platform_data = &glofiish_mmc_cfg;

	s3c24xx_fb_set_platdata(&glofiish_lcd_cfg);
	s3c24xx_udc_set_platdata(&glofiish_udc_cfg);
	set_s3c2410ts_info(&glofiish_ts_cfg);

	platform_device_register(&gta01_bl_dev);
	platform_device_register(&m800_pm_bt_dev);
	platform_device_register(&m800_button_dev);
	platform_device_register(&s3c_device_spi_lcm);

	platform_add_devices(glofiish_devices, ARRAY_SIZE(glofiish_devices));

	s3c2410_pm_init();
}

MACHINE_START(M800, "Glofiish M800")
	.phys_io	= S3C2410_PA_UART,
	.io_pg_offst	= (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C2410_SDRAM_PA + 0x100,
	.map_io		= glofiish_map_io,
	.init_irq	= s3c24xx_init_irq,
	.init_machine	= glofiish_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
