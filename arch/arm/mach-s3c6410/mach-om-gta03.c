/* linux/arch/arm/mach-s3c6410/mach-om_gta03.c
 *
 * Copyright 2008 Openmoko, Inc.
 * Andy Green <andy@openmoko.org>
 *
 * based on mach_om_gta03.c which is
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/lis302dl.h>

#include <video/platform_lcd.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/map.h>
#include <mach/regs-fb.h>

#include <asm/irq.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <plat/iic.h>
#include <plat/fb.h>
#include <plat/gpio-cfg.h>
#include <plat/pm.h>

#include <plat/s3c6410.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>

/* #include <plat/udc.h> */
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/regulator/machine.h>

#include <mach/om-gta03.h>

#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/mbc.h>
#include <linux/mfd/pcf50633/adc.h>
#include <linux/mfd/pcf50633/gpio.h>
#include <linux/mfd/pcf50633/led.h>

#define UCON S3C2410_UCON_DEFAULT | S3C2410_UCON_UCLK
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

static struct s3c2410_uartcfg om_gta03_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	[2] = {
		.hwport      = 2,
		.flags       = 0,
		.ucon        = 0x3c5,
		.ulcon       = 0x03,
		.ufcon       = 0x51,
	},
	[3] = {
		.hwport      = 3,
		.flags       = 0,
		.ucon        = 0x3c5,
		.ulcon       = 0x03,
		.ufcon       = 0x51,
	},
};


/*
 * Situation is that Linux SPI can't work in an interrupt context, so we
 * implement our own bitbang here.  Arbitration is needed because not only
 * can this interrupt happen at any time even if foreground wants to use
 * the bitbang API from Linux, but multiple motion sensors can be on the
 * same SPI bus, and multiple interrupts can happen.
 *
 * Foreground / interrupt arbitration is okay because the interrupts are
 * disabled around all the foreground SPI code.
 *
 * Interrupt / Interrupt arbitration is evidently needed, otherwise we
 * lose edge-triggered service after a while due to the two sensors sharing
 * the SPI bus having irqs at the same time eventually.
 *
 * Servicing is typ 75 - 100us at 400MHz.
 */

/* #define DEBUG_SPEW_MS */
#define MG_PER_SAMPLE 18

struct lis302dl_platform_data lis302_pdata;

/*
 * generic SPI RX and TX bitbang
 * only call with interrupts off!
 */

static void __gta03_lis302dl_bitbang(struct lis302dl_info *lis, u8 *tx,
					     int tx_bytes, u8 *rx, int rx_bytes)
{
	struct lis302dl_platform_data *pdata = lis->pdata;
	int n;
	u8 shifter = 0;

	gpio_direction_output(pdata->pin_chip_select, 1);
	gpio_direction_output(pdata->pin_clk, 1);
	gpio_direction_output(pdata->pin_chip_select, 0);

	/* send the register index, r/w and autoinc bits */
	for (n = 0; n < (tx_bytes << 3); n++) {
		if (!(n & 7))
			shifter = ~tx[n >> 3];
		gpio_direction_output(pdata->pin_clk, 0);
		gpio_direction_output(pdata->pin_mosi, !(shifter & 0x80));
		gpio_direction_output(pdata->pin_clk, 1);
		shifter <<= 1;
	}

	for (n = 0; n < (rx_bytes << 3); n++) { /* 8 bits each */
		gpio_direction_output(pdata->pin_clk, 0);
		shifter <<= 1;
		if (gpio_direction_input(pdata->pin_miso))
			shifter |= 1;
		if ((n & 7) == 7)
			rx[n >> 3] = shifter;
		gpio_direction_output(pdata->pin_clk, 1);
	}
	gpio_direction_output(pdata->pin_chip_select, 1);
}


static int gta03_lis302dl_bitbang_read_reg(struct lis302dl_info *lis, u8 reg)
{
	u8 data = 0xc0 | reg; /* read, autoincrement */
	unsigned long flags;

	local_irq_save(flags);

	__gta03_lis302dl_bitbang(lis, &data, 1, &data, 1);

	local_irq_restore(flags);

	return data;
}

static void gta03_lis302dl_bitbang_write_reg(struct lis302dl_info *lis, u8 reg,
									 u8 val)
{
	u8 data[2] = { 0x00 | reg, val }; /* write, no autoincrement */
	unsigned long flags;

	local_irq_save(flags);

	__gta03_lis302dl_bitbang(lis, &data[0], 2, NULL, 0);

	local_irq_restore(flags);

}


void gta03_lis302dl_suspend_io(struct lis302dl_info *lis, int resume)
{
	struct lis302dl_platform_data *pdata = lis->pdata;

	if (!resume) {
		 /*
		 * we don't want to power them with a high level
		 * because GSENSOR_3V3 is not up during suspend
		 */
		gpio_direction_output(pdata->pin_chip_select, 0);
		gpio_direction_output(pdata->pin_clk, 0);
		gpio_direction_output(pdata->pin_mosi, 0);
		s3c_gpio_setpull(pdata->pin_miso, S3C_GPIO_PULL_DOWN);

		return;
	}

	/* back to normal */
	gpio_direction_output(pdata->pin_chip_select, 1);
	gpio_direction_output(pdata->pin_clk, 1);
	s3c_gpio_setpull(pdata->pin_miso, S3C_GPIO_PULL_NONE);

	s3c_gpio_cfgpin(pdata->pin_chip_select, S3C_GPIO_SFN(1));
	s3c_gpio_cfgpin(pdata->pin_clk, S3C_GPIO_SFN(1));
	s3c_gpio_cfgpin(pdata->pin_mosi, S3C_GPIO_SFN(1));
	s3c_gpio_cfgpin(pdata->pin_miso, S3C_GPIO_SFN(0));

}

struct lis302dl_platform_data lis302_pdata = {
		.name		= "lis302",
		.pin_chip_select= S3C64XX_GPC(3), /* NC */
		.pin_clk	= GTA03_GPIO_ACCEL_CLK,
		.pin_mosi	= GTA03_GPIO_ACCEL_MOSI,
		.pin_miso	= GTA03_GPIO_ACCEL_MISO,
		.interrupt	= GTA03_IRQ_GSENSOR_1,
		.open_drain	= 0,
		.lis302dl_bitbang = __gta03_lis302dl_bitbang,
		.lis302dl_bitbang_reg_read = gta03_lis302dl_bitbang_read_reg,
		.lis302dl_bitbang_reg_write = gta03_lis302dl_bitbang_write_reg,
		.lis302dl_suspend_io = gta03_lis302dl_suspend_io,
};

static struct platform_device s3c_device_spi_acc1 = {
	.name		  = "lis302dl",
	.id		  = 1,
	.dev = {
		.platform_data = &lis302_pdata,
	},
};



/* framebuffer and LCD setup. */

/* GPF15 = LCD backlight control
 * GPF13 => Panel power
 * GPN5 = LCD nRESET signal
 * PWM_TOUT1 => backlight brightness
 */

static void om_gta03_lcd_power_set(struct plat_lcd_data *pd,
				   unsigned int power)
{

}

static struct plat_lcd_data om_gta03_lcd_power_data = {
	.set_power	= om_gta03_lcd_power_set,
};

static struct platform_device om_gta03_lcd_powerdev = {
	.name			= "platform-lcd",
	.dev.parent		= &s3c_device_fb.dev,
	.dev.platform_data	= &om_gta03_lcd_power_data,
};

static struct s3c_fb_pd_win om_gta03_fb_win0 = {
	/* this is to ensure we use win0 */
	.win_mode	= {
		.pixclock	= 40816,
		.left_margin	= 8,
		.right_margin	= 16,
		.upper_margin	= 2,
		.lower_margin	= 16,
		.hsync_len	= 8,
		.vsync_len	= 2,
		.xres		= 640,
		.yres		= 480,
	},
	.max_bpp	= 32,
	.default_bpp	= 16,
};

static struct s3c_fb_platdata om_gta03_lcd_pdata __initdata = {
	.setup_gpio	= s3c64xx_fb_gpio_setup_24bpp,
	.win[0]		= &om_gta03_fb_win0,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
};


struct map_desc om_gta03_6410_iodesc[] = {};

static struct resource om_gta03_button_resources[] = {
	[0] = {
		.start = 0,
		.end   = 0,
	},
	[1] = {
		.start = GTA03_GPIO_HOLD,
		.end   = GTA03_GPIO_HOLD,
	},
	[2] = {
		.start = GTA03_GPIO_JACK_INSERT,
		.end   = GTA03_GPIO_JACK_INSERT,
	},
	[3] = {
		.start = GTA03_GPIO_KEY_PLUS,
		.end   = GTA03_GPIO_KEY_PLUS,
	},
	[4] = {
		.start = GTA03_GPIO_KEY_MINUS,
		.end   = GTA03_GPIO_KEY_MINUS,
	},
};

static struct platform_device om_gta03_button_dev = {
	.name		= "neo1973-button",
	.num_resources	= ARRAY_SIZE(om_gta03_button_resources),
	.resource	= om_gta03_button_resources,
};


/********************** PMU ***************************/
/*
 * GTA03 PMU Mapping info
 *
 *  name  maxcurr  default    Nom   consumers
 *
 *  AUTO   1100mA  ON  3.3V   3.3V  Main 3.3V rail
 *  DOWN1   500mA  ON  1.2V   1.2V  CPU VddARM, VddINT, VddMPLL, VddOTGI
 *  DOWN2   500mA  ON  1.8V   1.8V  CPU VddAlive via LDO, Memories, WLAN
 *  LED      25mA  OFF        18V   Backlight
 *  HCLDO   200mA  OFF        2.8V  Camera 2V8
 *  LDO1     50mA  ON  3.3V   3.3V  Accel
 *  LDO2     50mA  OFF        1.5V  Camera 1V5
 *  LDO3     50mA  OFF        3.3V  CODEC 3.3V
 *  LDO4    150mA  ON  2.8V   2.7V  uSD power
 *  LDO5    150mA  OFF        3.0V  GPS 3V
 *  LDO6     50mA  ON  3.0V   3.0V  LCM 3V
 *
 */


/* PMU driver info */


static struct regulator_consumer_supply ldo4_consumers[] = {
	{
		.dev = &s3c_device_hsmmc0.dev,
		.supply = "SD_3V",
	},
};

static struct platform_device om_gta03_features_dev = {
	.name		= "om-gta03",
};

static struct regulator_consumer_supply ldo5_consumers[] = {
	{
		.dev = &om_gta03_features_dev.dev,
		.supply = "RF_3V",
	},
};


static void om_gta03_pmu_event_callback(struct pcf50633 *pcf, int irq)
{
#if 0
	if (irq == PCF50633_IRQ_USBINS) {
		schedule_delayed_work(&gta02_charger_work,
				GTA02_CHARGER_CONFIGURE_TIMEOUT);
		return;
	} else if (irq == PCF50633_IRQ_USBREM) {
		cancel_delayed_work_sync(&gta02_charger_work);
		pcf50633_mbc_usb_curlim_set(pcf, 0);
		gta02_usb_vbus_draw = 0;
	}

	bq27000_charging_state_change(&bq27000_battery_device);
#endif
}


static void om_gta03_pcf50633_attach_child_devices(struct pcf50633 *pcf);
static void om_gta03_pmu_regulator_registered(struct pcf50633 *pcf, int id);

struct pcf50633_platform_data om_gta03_pcf_pdata = {

	.resumers = {
		[0] = PCF50633_INT1_USBINS |
		      PCF50633_INT1_USBREM |
		      PCF50633_INT1_ALARM,
		[1] = PCF50633_INT2_ONKEYF,
		[2] = PCF50633_INT3_ONKEY1S
	},

	.reg_init_data = {
		/* GTA03: Main 3.3V rail */
		[PCF50633_REGULATOR_AUTO] = {
			.constraints = {
				.min_uV = 3300000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
				.state_mem = {
					.enabled = 1,
				},
			},
			.num_consumer_supplies = 0,
		},
		/* GTA03: CPU core power */
		[PCF50633_REGULATOR_DOWN1] = {
			.constraints = {
				.min_uV = 900000,
				.max_uV = 1200000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
			.num_consumer_supplies = 0,
		},
		/* GTA03: Memories */
		[PCF50633_REGULATOR_DOWN2] = {
			.constraints = {
				.min_uV = 1800000,
				.max_uV = 1800000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
				.state_mem = {
					.enabled = 1,
				},
			},
			.num_consumer_supplies = 0,
		},
		/* GTA03: Camera 2V8 */
		[PCF50633_REGULATOR_HCLDO] = {
			.constraints = {
				.min_uV = 2800000,
				.max_uV = 2800000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
			},
			.num_consumer_supplies = 0,
/*			.consumer_supplies = hcldo_consumers, */
		},

		/* GTA03: Accel 3V3 */
		[PCF50633_REGULATOR_LDO1] = {
			.constraints = {
				.min_uV = 3300000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
			.num_consumer_supplies = 0,
		},
		/* GTA03: Camera 1V5 */
		[PCF50633_REGULATOR_LDO2] = {
			.constraints = {
				.min_uV = 1500000,
				.max_uV = 1500000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
			.num_consumer_supplies = 0,
		},
		/* GTA03: Codec 3.3V */
		[PCF50633_REGULATOR_LDO3] = {
			.constraints = {
				.min_uV = 3300000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
			.num_consumer_supplies = 0,
		},
		/* GTA03: uSD Power */
		[PCF50633_REGULATOR_LDO4] = {
			.constraints = {
				.min_uV = 3000000,
				.max_uV = 3000000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
			.num_consumer_supplies = 1,
			.consumer_supplies = ldo4_consumers,
		},
		/* GTA03: GPS 3V */
		[PCF50633_REGULATOR_LDO5] = {
			.constraints = {
				.min_uV = 3000000,
				.max_uV = 3000000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
			.num_consumer_supplies = 1,
			.consumer_supplies = ldo5_consumers,
		},
		/* GTA03: LCM 3V */
		[PCF50633_REGULATOR_LDO6] = {
			.constraints = {
				.min_uV = 3000000,
				.max_uV = 3000000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.state_mem = {
					.enabled = 1,
				},
			},
			.num_consumer_supplies = 0,
		},
		/* power for memories in suspend */
		[PCF50633_REGULATOR_MEMLDO] = {
			.constraints = {
				.min_uV = 1800000,
				.max_uV = 1800000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.state_mem = {
					.enabled = 1,
				},
			},
			.num_consumer_supplies = 0,
		},

	},
	.probe_done = om_gta03_pcf50633_attach_child_devices,
	.regulator_registered = om_gta03_pmu_regulator_registered,
	.mbc_event_callback = om_gta03_pmu_event_callback,
};


static struct i2c_board_info om_gta03_i2c_devs[] __initdata = {
	{
		I2C_BOARD_INFO("pcf50633", 0x73),
		.irq = GTA03_IRQ_PMU,
		.platform_data = &om_gta03_pcf_pdata,
	},
	{
		I2C_BOARD_INFO("pcap7200", 0x0a),
		.irq = GTA03_IRQ_TOUCH,
	},

};


static struct platform_device *om_gta03_devices[] __initdata = {
	&s3c_device_fb,
	&s3c_device_i2c0,
	&s3c_device_hsmmc1, /* SDIO to WLAN */
};


static void om_gta03_pmu_regulator_registered(struct pcf50633 *pcf, int id)
{
	struct platform_device *regulator, *pdev;

	regulator = pcf->pmic.pdev[id];

	switch(id) {
		case PCF50633_REGULATOR_LDO4:
			pdev = &s3c_device_hsmmc0;
			break;
		case PCF50633_REGULATOR_LDO5: /* GPS regulator */
			pdev = &om_gta03_features_dev;
			break;
		case PCF50633_REGULATOR_LDO6:
			pdev = &om_gta03_lcd_powerdev;
			break;
		default:
			return;
	}

	pdev->dev.parent = &regulator->dev;
	platform_device_register(pdev);
}

static struct platform_device *om_gta03_devices_pmu_children[] = {
	&om_gta03_button_dev,
	&s3c_device_spi_acc1, /* relies on PMU reg for power */
};

/* this is called when pc50633 is probed, unfortunately quite late in the
 * day since it is an I2C bus device.  Here we can belatedly define some
 * platform devices with the advantage that we can mark the pcf50633 as the
 * parent.  This makes them get suspended and resumed with their parent
 * the pcf50633 still around.
 */

static void om_gta03_pcf50633_attach_child_devices(struct pcf50633 *pcf)
{
	int n;

	for (n = 0; n < ARRAY_SIZE(om_gta03_devices_pmu_children); n++)
		om_gta03_devices_pmu_children[n]->dev.parent = pcf->dev;

	platform_add_devices(om_gta03_devices_pmu_children,
				     ARRAY_SIZE(om_gta03_devices_pmu_children));

	/* Switch on backlight. Qi does not do it for us */
	pcf50633_reg_write(pcf, PCF50633_REG_LEDENA, 0x00);
	pcf50633_reg_write(pcf, PCF50633_REG_LEDDIM, 0x01);
	pcf50633_reg_write(pcf, PCF50633_REG_LEDENA, 0x01);
	pcf50633_reg_write(pcf, PCF50633_REG_LEDOUT, 0x3f);

}



extern void s3c64xx_init_io(struct map_desc *, int);

static void __init om_gta03_map_io(void)
{
	s3c64xx_init_io(om_gta03_6410_iodesc, ARRAY_SIZE(om_gta03_6410_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(om_gta03_uartcfgs, ARRAY_SIZE(om_gta03_uartcfgs));
}

static void __init om_gta03_machine_init(void)
{
	s3c_pm_init();

	s3c_i2c0_set_platdata(NULL);
	s3c_fb_set_platdata(&om_gta03_lcd_pdata);

	s3c_gpio_setpull(S3C64XX_GPH(0), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C64XX_GPH(1), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C64XX_GPH(2), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C64XX_GPH(3), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C64XX_GPH(4), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C64XX_GPH(5), S3C_GPIO_PULL_UP);


	i2c_register_board_info(0, om_gta03_i2c_devs,
						 ARRAY_SIZE(om_gta03_i2c_devs));

	platform_add_devices(om_gta03_devices, ARRAY_SIZE(om_gta03_devices));
}

MACHINE_START(OPENMOKO_GTA03, "OM-GTA03")
	/* Maintainer: Andy Green <andy@openmoko.com> */
	.phys_io	= S3C_PA_UART & 0xfff00000,
	.io_pg_offst	= (((u32)S3C_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C64XX_PA_SDRAM + 0x100,

	.init_irq	= s3c6410_init_irq,
	.map_io		= om_gta03_map_io,
	.init_machine	= om_gta03_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END

