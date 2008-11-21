/* linux/arch/arm/mach-s3c6410/mach-smdk6410.c
 *
 * Copyright 2008 Openmoko, Inc.
 * Andy Green <andy@openmoko.org>
 *
 * based on mach_smdk6410.c which is
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

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/map.h>

#include <asm/irq.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <plat/iic.h>

#include <plat/s3c6410.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>

/* #include <plat/udc.h> */
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/regulator/machine.h>

#include <linux/pcf50633.h>

#include <mach/om-gta03.h>

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

static int om_gta03_pmu_callback(struct device *dev, unsigned int feature,
			enum pmu_event event)
{
#if 0
	switch (feature) {
	case PCF50633_FEAT_MBC:
		switch (event) {
		case PMU_EVT_CHARGER_IDLE:
			gta02_charger_active_status = 0;
			break;
		case PMU_EVT_CHARGER_ACTIVE:
			gta02_charger_active_status = 1;
			break;
		case PMU_EVT_USB_INSERT:
			gta02_charger_online_status = 1;
			break;
		case PMU_EVT_USB_REMOVE:
			gta02_charger_online_status = 0;
			break;
		case PMU_EVT_INSERT: /* adapter is unsused */
		case PMU_EVT_REMOVE: /* adapter is unused */
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	bq27000_charging_state_change(&bq27000_battery_device);
#endif
	return 0;
}

/* this is called when pc50633 is probed, unfortunately quite late in the
 * day since it is an I2C bus device.  Here we can belatedly define some
 * platform devices with the advantage that we can mark the pcf50633 as the
 * parent.  This makes them get suspended and resumed with their parent
 * the pcf50633 still around.
 */

static void om_gta03_pcf50633_attach_child_devices(struct device *parent_device)
{
#if 0
	int n;

	for (n = 0; n < ARRAY_SIZE(gta02_devices_pmu_children); n++)
		gta02_devices_pmu_children[n]->dev.parent = parent_device;

	mangle_glamo_res_by_system_rev();
	platform_add_devices(gta02_devices_pmu_children,
					ARRAY_SIZE(gta02_devices_pmu_children));
#endif
}


static struct regulator_consumer_supply ldo4_consumers[] = {
	{
		.dev = &s3c_device_hsmmc0.dev,
		.supply = "SD_3V",
	},
};
#if 0
static struct regulator_consumer_supply ldo5_consumers[] = {
	{
		.dev = &gta01_pm_gps_dev.dev,
		.supply = "RF_3V",
	},
};
#endif



struct pcf50633_platform_data om_gta03_pcf_pdata = {
	.used_features	= PCF50633_FEAT_MBC |
			  PCF50633_FEAT_BBC |
			  PCF50633_FEAT_RTC |
			  PCF50633_FEAT_CHGCUR |
			  PCF50633_FEAT_BATVOLT |
			  PCF50633_FEAT_BATTEMP |
			  PCF50633_FEAT_PWM_BL,
	.onkey_seconds_sig_init = 4,
	.onkey_seconds_shutdown = 8,
	.cb		= &om_gta03_pmu_callback,
	.r_fix_batt	= 10000,
	.r_fix_batt_par	= 10000,
	.r_sense_milli	= 220,
	.flag_use_apm_emulation = 0,
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
			.num_consumer_supplies = 0,
/*			.consumer_supplies = ldo5_consumers, */
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
	.attach_child_devices = om_gta03_pcf50633_attach_child_devices,
};


static struct i2c_board_info om_gta03_i2c_devs[] __initdata = {
	{
		I2C_BOARD_INFO("pcf50633", 0x73),
		.irq = GTA03_IRQ_PMU,
		.platform_data = &om_gta03_pcf_pdata,
	},
};


static struct platform_device *om_gta03_devices[] __initdata = {
	&s3c_device_hsmmc0,
	&s3c_device_hsmmc1,
	&s3c_device_i2c0,
	&om_gta03_button_dev,
};

extern void s3c64xx_init_io(struct map_desc *, int);

static void __init om_gta03_map_io(void)
{
	s3c64xx_init_io(om_gta03_6410_iodesc, ARRAY_SIZE(om_gta03_6410_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(om_gta03_uartcfgs, ARRAY_SIZE(om_gta03_uartcfgs));
}

static void __init om_gta03_machine_init(void)
{
	s3c_i2c0_set_platdata(NULL);

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

