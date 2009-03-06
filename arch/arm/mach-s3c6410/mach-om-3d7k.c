/* linux/arch/arm/mach-s3c6410/mach-om-3d7k.c
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
#include <linux/i2c.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/lis302dl.h>
#include <linux/lp5521.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/l1k002.h>
#include <linux/pcap7200.h>
#include <linux/bq27000_battery.h>
#include <linux/hdq.h>

#include <video/platform_lcd.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <asm/hardware/vic.h>
#include <asm/hardware/tzic-sp890.h>
#include <mach/map.h>
#include <mach/regs-fb.h>
#include <mach/spi-gpio.h>

#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/fiq.h>

#include <plat/regs-serial.h>
#include <plat/regs-timer.h>
#include <plat/regs-gpio.h>
#include <plat/iic.h>
#include <plat/fb.h>
#include <plat/gpio-cfg.h>
#include <plat/pm.h>
#include <plat/pwm.h>

#include <plat/s3c6410.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/tzic-sp890.h>

/* #include <plat/udc.h> */
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/regulator/machine.h>

#include <mach/om-3d7k.h>

#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/mbc.h>
#include <linux/mfd/pcf50633/adc.h>
#include <linux/mfd/pcf50633/gpio.h>
#include <linux/mfd/pcf50633/pmic.h>

#include <plat/regs-usb-hs-otg.h>

extern struct platform_device s3c_device_usbgadget;
extern struct platform_device s3c_device_camif; /* @@@ change plat/devs.h */


/* -------------------------------------------------------------------------------
 * OM_3D7K FIQ related
 *
 * Calls into vibrator and hdq and based on the return values
 * determines if we the FIQ source be kept alive
 */

#define DIVISOR_FROM_US(x) ((x) * 23)

#ifdef CONFIG_HDQ_GPIO_BITBANG
#define FIQ_DIVISOR_HDQ DIVISOR_FROM_US(HDQ_SAMPLE_PERIOD_US)
extern int hdq_fiq_handler(void);
#endif

/* Global data related to our fiq source */
static u32 om_3d7k_fiq_ack_mask;
static u32 om_3d7k_fiq_mod_mask;
static struct s3c2410_pwm om_3d7k_fiq_pwm_timer;
static u16 om_3d7k_fiq_timer_index;
static int om_3d7k_fiq_irq;

/* Convinience defines */
#define S3C6410_INTMSK	(S3C_VA_VIC0 + VIC_INT_ENABLE)
#define S3C6410_INTMOD	(S3C_VA_VIC0 + VIC_INT_SELECT)



static void om_3d7k_fiq_handler(void)
{
	u16 divisor = 0xffff;

	/* Vibrator servicing */

	/* disable further timer interrupts if nobody has any work
	 * or adjust rate according to who still has work
	 *
	 * CAUTION: it means forground code must disable FIQ around
	 * its own non-atomic S3C2410_INTMSK changes... not common
	 * thankfully and taken care of by the fiq-basis patch
	 */

#ifdef CONFIG_HDQ_GPIO_BITBANG
	if (hdq_fiq_handler())
		divisor = (u16)FIQ_DIVISOR_HDQ;
#endif

	if (divisor == 0xffff) /* mask the fiq irq source */
		__raw_writel((__raw_readl(S3C64XX_TINT_CSTAT) & 0x1f) & ~(1 << 3),
							    S3C64XX_TINT_CSTAT);
	else /* still working, maybe at a different rate */
		__raw_writel(divisor, S3C2410_TCNTB(om_3d7k_fiq_timer_index));

	__raw_writel((__raw_readl(S3C64XX_TINT_CSTAT) & 0x1f ) | 1 << 8 , S3C64XX_TINT_CSTAT);

}

static void om_3d7k_fiq_kick(void)
{
	unsigned long flags;
	u32 tcon;
	
	/* we have to take care about FIQ because this modification is
	 * non-atomic, FIQ could come in after the read and before the
	 * writeback and its changes to the register would be lost
	 * (platform INTMSK mod code is taken care of already)
	 */
	local_save_flags(flags);
	local_fiq_disable();
	/* allow FIQs to resume   */
	__raw_writel((__raw_readl(S3C64XX_TINT_CSTAT)  & 0x1f)| 1 << 3,
							    S3C64XX_TINT_CSTAT);

	tcon = __raw_readl(S3C2410_TCON) & ~S3C2410_TCON_T3START; 
	/* fake the timer to a count of 1 */
	__raw_writel(1, S3C2410_TCNTB(om_3d7k_fiq_timer_index));
	__raw_writel(tcon | S3C2410_TCON_T3MANUALUPD, S3C2410_TCON);
	__raw_writel(tcon | S3C2410_TCON_T3MANUALUPD | S3C2410_TCON_T3START,
		     S3C2410_TCON);
	__raw_writel(tcon | S3C2410_TCON_T3START, S3C2410_TCON);
	local_irq_restore(flags);
}

static int om_3d7k_fiq_enable(void)
{
	int irq_index_fiq = IRQ_TIMER3_VIC;
	int rc = 0;

	local_fiq_disable();

	om_3d7k_fiq_irq = irq_index_fiq;
	om_3d7k_fiq_ack_mask = 1 << 3;
	om_3d7k_fiq_mod_mask = 1 << 27;
	om_3d7k_fiq_timer_index = 3;

	/* set up the timer to operate as a pwm device */

	rc = s3c2410_pwm_init(&om_3d7k_fiq_pwm_timer);
	if (rc)
		goto bail;

	om_3d7k_fiq_pwm_timer.timerid = PWM0 + om_3d7k_fiq_timer_index;
	om_3d7k_fiq_pwm_timer.prescaler = ((6 - 1) / 2);
	om_3d7k_fiq_pwm_timer.divider = S3C64XX_TCFG1_MUX_DIV2 << S3C2410_TCFG1_SHIFT(3);
	/* default rate == ~32us */
	om_3d7k_fiq_pwm_timer.counter = om_3d7k_fiq_pwm_timer.comparer = 3000;

	rc = s3c2410_pwm_enable(&om_3d7k_fiq_pwm_timer);
	if (rc)
		goto bail;

	/* let our selected interrupt be a magic FIQ interrupt */
	__raw_writel(om_3d7k_fiq_mod_mask, S3C6410_INTMSK + 4);
	__raw_writel(om_3d7k_fiq_mod_mask, S3C6410_INTMOD);
	__raw_writel(om_3d7k_fiq_mod_mask, S3C6410_INTMSK);

	__raw_writel(SP890_TZIC_UNLOCK_MAGIC, S3C64XX_VA_TZIC0_LOCK);
	__raw_writel(om_3d7k_fiq_mod_mask, S3C64XX_VA_TZIC0_FIQENABLE);
	__raw_writel(om_3d7k_fiq_mod_mask, S3C64XX_VA_TZIC0_INTSELECT);

	s3c2410_pwm_start(&om_3d7k_fiq_pwm_timer);

	/* it's ready to go as soon as we unmask the source in S3C2410_INTMSK */
	local_fiq_enable();

	set_fiq_c_handler(om_3d7k_fiq_handler);

	if (rc < 0)
		goto bail;

	return 0;
bail:
	printk(KERN_ERR "Count not initialize FIQ for OM_3D7K %d \n", rc);
	return rc;
}

static void om_3d7k_fiq_disable(void)
{
	__raw_writel(0, S3C6410_INTMOD);
	local_fiq_disable();
	om_3d7k_fiq_irq = 0; /* no active source interrupt now either */

}
/* -------------------- /OM_3D7K FIQ Handler ------------------------------------- */

#define UCON S3C2410_UCON_DEFAULT | S3C2410_UCON_UCLK
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

static struct s3c2410_uartcfg om_3d7k_uartcfgs[] __initdata = {
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

static void __3d7k_lis302dl_bitbang(struct lis302dl_info *lis, u8 *tx,
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


static int om_3d7k_lis302dl_bitbang_read_reg(struct lis302dl_info *lis, u8 reg)
{
	u8 data = 0xc0 | reg; /* read, autoincrement */
	unsigned long flags;

	local_irq_save(flags);

	__3d7k_lis302dl_bitbang(lis, &data, 1, &data, 1);

	local_irq_restore(flags);

	return data;
}

static void om_3d7k_lis302dl_bitbang_write_reg(struct lis302dl_info *lis, u8 reg,
									 u8 val)
{
	u8 data[2] = { 0x00 | reg, val }; /* write, no autoincrement */
	unsigned long flags;

	local_irq_save(flags);

	__3d7k_lis302dl_bitbang(lis, &data[0], 2, NULL, 0);

	local_irq_restore(flags);

}


void om_3d7k_lis302dl_suspend_io(struct lis302dl_info *lis, int resume)
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
#if 0
struct lis302dl_platform_data lis302_pdata = {
		.name		= "lis302",
		.pin_chip_select= S3C64XX_GPC(3), /* NC */
		.pin_clk	= OM_3D7K_GPIO_ACCEL_CLK,
		.pin_mosi	= OM_3D7K_GPIO_ACCEL_MOSI,
		.pin_miso	= OM_3D7K_GPIO_ACCEL_MISO,
		.interrupt	= OM_3D7K_IRQ_GSENSOR_1,
		.open_drain	= 0,
		.lis302dl_bitbang = __3d7k_lis302dl_bitbang,
		.lis302dl_bitbang_reg_read = om_3d7k_lis302dl_bitbang_read_reg,
		.lis302dl_bitbang_reg_write = om_3d7k_lis302dl_bitbang_write_reg,
		.lis302dl_suspend_io = om_3d7k_lis302dl_suspend_io,
};

static struct platform_device s3c_device_spi_acc1 = {
	.name		  = "lis302dl",
	.id		  = 1,
	.dev = {
		.platform_data = &lis302_pdata,
	},
};

#endif

/* framebuffer and LCD setup. */

/* GPF15 = LCD backlight control
 * GPF13 => Panel power
 * GPN5 = LCD nRESET signal
 * PWM_TOUT1 => backlight brightness
 */

static void om_3d7k_lcd_power_set(struct plat_lcd_data *pd,
				   unsigned int power)
{

}

static struct plat_lcd_data om_3d7k_lcd_power_data = {
	.set_power	= om_3d7k_lcd_power_set,
};

static struct platform_device om_3d7k_lcd_powerdev = {
	.name			= "platform-lcd",
	.dev.parent		= &s3c_device_fb.dev,
	.dev.platform_data	= &om_3d7k_lcd_power_data,
};

static struct s3c_fb_pd_win om_3d7k_fb_win0 = {
	/* this is to ensure we use win0 */
	.win_mode	= {
		.pixclock	= 40816,
		.left_margin	= 8,
		.right_margin	= 16,
		.upper_margin	= 2,
		.lower_margin	= 16,
		.hsync_len	= 8,
		.vsync_len	= 2,
		.xres		= 480,
		.yres		= 640,
	},
	.max_bpp	= 32,
	.default_bpp	= 16,
};

static void om_3d7k_fb_gpio_setup(void)
{
	unsigned int gpio;

	/* GPI0, GPI1, GPI8 are for hardware version contrl.
	 * They should be set as input in order to prevent
	 * current leaking
	 */
	for (gpio = S3C64XX_GPI(2); gpio <= S3C64XX_GPI(15); gpio++) {
		if (gpio != S3C64XX_GPI(8)) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		}
	}

	for (gpio = S3C64XX_GPJ(0); gpio <= S3C64XX_GPJ(11); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}
}

static struct s3c_fb_platdata om_3d7k_lcd_pdata __initdata = {
	.setup_gpio	= om_3d7k_fb_gpio_setup,
	.win[0]		= &om_3d7k_fb_win0,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
};


struct map_desc om_3d7k_6410_iodesc[] = {};

static struct resource om_3d7k_button_resources[] = {
	[0] = {
		.start = 0,
		.end   = 0,
	},
	[1] = {
		.start = OM_3D7K_GPIO_HOLD,
		.end   = OM_3D7K_GPIO_HOLD,
	},
	[2] = {
		.start = OM_3D7K_GPIO_JACK_INSERT,
		.end   = OM_3D7K_GPIO_JACK_INSERT,
	},
	[3] = {
		.start = OM_3D7K_GPIO_KEY_PLUS,
		.end   = OM_3D7K_GPIO_KEY_PLUS,
	},
	[4] = {
		.start = OM_3D7K_GPIO_KEY_MINUS,
		.end   = OM_3D7K_GPIO_KEY_MINUS,
	},
};

static struct platform_device om_3d7k_button_dev = {
	.name		= "neo1973-button",
	.num_resources	= ARRAY_SIZE(om_3d7k_button_resources),
	.resource	= om_3d7k_button_resources,
};


/********************** PMU ***************************/
/*
 * OM_3D7K PMU Mapping info
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

static struct platform_device om_3d7k_features_dev = {
	.name		= "om-3d7k",
};

static struct regulator_consumer_supply ldo5_consumers[] = {
	{
		.dev = &om_3d7k_features_dev.dev,
		.supply = "RF_3V",
	},
};


static void om_3d7k_pmu_event_callback(struct pcf50633 *pcf, int irq)
{
#if 0
	if (irq == PCF50633_IRQ_USBINS) {
		schedule_delayed_work(&om_3d7k_charger_work,
				GTA02_CHARGER_CONFIGURE_TIMEOUT);
		return;
	} else if (irq == PCF50633_IRQ_USBREM) {
		cancel_delayed_work_sync(&om_3d7k_charger_work);
		pcf50633_mbc_usb_curlim_set(pcf, 0);
		om_3d7k_usb_vbus_draw = 0;
	}

	bq27000_charging_state_change(&bq27000_battery_device);
#endif
}

static void om_3d7k_pcf50633_attach_child_devices(struct pcf50633 *pcf);
static void om_3d7k_pmu_regulator_registered(struct pcf50633 *pcf, int id);

/* Global reference */
struct pcf50633 *om_3d7k_pcf;

struct pcf50633_platform_data om_3d7k_pcf_pdata = {

	.resumers = {
		[0] = PCF50633_INT1_USBINS |
		      PCF50633_INT1_USBREM |
		      PCF50633_INT1_ALARM,
		[1] = PCF50633_INT2_ONKEYF,
		[2] = PCF50633_INT3_ONKEY1S
	},
	.chg_ref_current_ma = 1000,
	.reg_init_data = {
		/* OM_3D7K: Main 3.3V rail */
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
		/* OM_3D7K: CPU core power */
		[PCF50633_REGULATOR_DOWN1] = {
			.constraints = {
				.min_uV = 900000,
				.max_uV = 1200000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
			.num_consumer_supplies = 0,
		},
		/* OM_3D7K: Memories */
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
		/* OM_3D7K: Camera 2V8 */
		[PCF50633_REGULATOR_HCLDO] = {
			.constraints = {
				.min_uV = 2800000,
				.max_uV = 2800000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
			},
			.num_consumer_supplies = 0,
/*			.consumer_supplies = hcldo_consumers, */
		},

		/* OM_3D7K: Accel 3V3 */
		[PCF50633_REGULATOR_LDO1] = {
			.constraints = {
				.min_uV = 3300000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
			.num_consumer_supplies = 0,
		},
		/* OM_3D7K: Camera 1V5 */
		[PCF50633_REGULATOR_LDO2] = {
			.constraints = {
				.min_uV = 1500000,
				.max_uV = 1500000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
			.num_consumer_supplies = 0,
		},
		/* OM_3D7K: Codec 3.3V */
		[PCF50633_REGULATOR_LDO3] = {
			.constraints = {
				.min_uV = 3300000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
				.always_on = 1,
			},
			.num_consumer_supplies = 0,
		},
		/* OM_3D7K: uSD Power */
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
		/* OM_3D7K: GPS 3V */
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
		/* OM_3D7K: LCM 3V */
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
	.probe_done = om_3d7k_pcf50633_attach_child_devices,
	.regulator_registered = om_3d7k_pmu_regulator_registered,
	.mbc_event_callback = om_3d7k_pmu_event_callback,
};

static void om_3d7k_bl_set_intensity(int intensity)
{
	struct pcf50633 *pcf = om_3d7k_pcf;
	int old_intensity = pcf50633_reg_read(pcf, PCF50633_REG_LEDOUT);
	int ret;

	intensity >>= 2;

	/*
	 * One code path that leads here is from a kernel panic. Trying to turn
	 * the backlight on just gives us a nearly endless stream of complaints
	 * and accomplishes nothing. We can't win. Just give up.
	 *
	 * In the unlikely event that there's another path leading here while
	 * we're atomic, we print at least a warning.
	 */
	if (in_atomic()) {
		printk(KERN_ERR
		    "3d7k_bl_set_intensity called while atomic\n");
		return;
	}

	old_intensity = pcf50633_reg_read(pcf, PCF50633_REG_LEDOUT);
	if (intensity == old_intensity)
		return;

	/* We can't do this anywhere else */
	pcf50633_reg_write(pcf, PCF50633_REG_LEDDIM, 5);

	if (!(pcf50633_reg_read(pcf, PCF50633_REG_LEDENA) & 3))
		old_intensity = 0;

	/*
	 * The PCF50633 cannot handle LEDOUT = 0 (datasheet p60)
	 * if seen, you have to re-enable the LED unit
	 */
	if (!intensity || !old_intensity)
		pcf50633_reg_write(pcf, PCF50633_REG_LEDENA, 0);

	if (!intensity) /* illegal to set LEDOUT to 0 */
		ret = pcf50633_reg_set_bit_mask(pcf, PCF50633_REG_LEDOUT, 0x3f,
									     2);
	else
		ret = pcf50633_reg_set_bit_mask(pcf, PCF50633_REG_LEDOUT, 0x3f,
			       intensity);

	if (intensity)
		pcf50633_reg_write(pcf, PCF50633_REG_LEDENA, 2);

}

static struct generic_bl_info om_3d7k_bl_info = {
	.name 			= "om-3d7k-bl",
	.max_intensity 		= 0xff,
	.default_intensity 	= 0x7f,
	.set_bl_intensity 	= om_3d7k_bl_set_intensity,
};

static struct platform_device om_3d7k_bl_dev = {
	.name  = "generic-bl",
	.id  = 1,
	.dev = {
		.platform_data = &om_3d7k_bl_info,
	},
};

/* BQ27000 Battery */
static int om_3d7k_get_charger_online_status(void)
{
	struct pcf50633 *pcf = om_3d7k_pcf;

	return pcf50633_mbc_get_status(pcf) & PCF50633_MBC_USB_ONLINE;
}

static int om_3d7k_get_charger_active_status(void)
{
	struct pcf50633 *pcf = om_3d7k_pcf;

	return pcf50633_mbc_get_status(pcf) & PCF50633_MBC_USB_ACTIVE;
}


struct bq27000_platform_data bq27000_pdata = {
	.name = "battery",
	.rsense_mohms = 20,
	.hdq_read = hdq_read,
	.hdq_write = hdq_write,
	.hdq_initialized = hdq_initialized,
	.get_charger_online_status = om_3d7k_get_charger_online_status,
	.get_charger_active_status = om_3d7k_get_charger_active_status
};

struct platform_device bq27000_battery_device = {
	.name 		= "bq27000-battery",
	.dev = {
		.platform_data = &bq27000_pdata,
	},
};

#ifdef CONFIG_HDQ_GPIO_BITBANG
/* HDQ */

static void om_3d7k_hdq_attach_child_devices(struct device *parent_device)
{
		bq27000_battery_device.dev.parent = parent_device;
		platform_device_register(&bq27000_battery_device);
}

static void om_3d7k_hdq_gpio_direction_out(void)
{
	unsigned long con;
	void __iomem *regcon = S3C64XX_GPH_BASE; 

	con = __raw_readl(regcon);
	con &= ~(0xf << 28);
	con |= 0x01 << 28;
	__raw_writel(con, regcon);

	/* Set pull-up enabled */
	con = __raw_readl(regcon + 0x0c);
	con |= 3 << 14;
	__raw_writel(con, regcon + 0x0c);
}

static void om_3d7k_hdq_gpio_direction_in(void)
{
	unsigned long con;
	void __iomem *regcon = S3C64XX_GPH_BASE;

	con = __raw_readl(regcon);
	con &= ~(0xf << 28);
	__raw_writel(con, regcon);
}

static void om_3d7k_hdq_gpio_set_value(int val)
{
	u32 dat;
	void __iomem *base = S3C64XX_GPH_BASE;

	dat = __raw_readl(base + 0x08);
	if (val)
		dat |= 1 << 7;
	else
		dat &= ~(1 << 7);

	__raw_writel(dat, base + 0x08);
}

static int om_3d7k_hdq_gpio_get_value(void)
{
	u32 dat;
	void *base = S3C64XX_GPH_BASE;
	
	dat = __raw_readl(base + 0x08);

	return dat & (1 << 7);
}

static struct resource om_3d7k_hdq_resources[] = {
	[0] = {
		.start	= S3C64XX_GPH(7),
		.end	= S3C64XX_GPH(7),
	},
};

struct hdq_platform_data om_3d7k_hdq_platform_data = {
	.attach_child_devices = om_3d7k_hdq_attach_child_devices,
	.gpio_dir_out = om_3d7k_hdq_gpio_direction_out,
	.gpio_dir_in = om_3d7k_hdq_gpio_direction_in,
	.gpio_set = om_3d7k_hdq_gpio_set_value,
	.gpio_get = om_3d7k_hdq_gpio_get_value,

	.enable_fiq = om_3d7k_fiq_enable,
	.disable_fiq = om_3d7k_fiq_disable,
	.kick_fiq = om_3d7k_fiq_kick,

};

struct platform_device om_3d7k_hdq_device = {
	.name 		= "hdq",
	.num_resources	= 1,
	.resource	= om_3d7k_hdq_resources,
	.dev		= {
		.platform_data = &om_3d7k_hdq_platform_data,
	},
};
#endif

static void om_3d7k_lp5521_chip_enable(int level)
{
	gpio_direction_output(OM_3D7K_GPIO_LED_EN, level);
	udelay(500);
}

static struct lp5521_platform_data om_3d7k_lp5521_pdata = {
	.ext_enable = om_3d7k_lp5521_chip_enable,
};

static void om_3d7k_pcap7200_reset(void)
{
	gpio_direction_output(OM_3D7K_GPIO_TP_RESET, 1);
	udelay(10);
	gpio_direction_output(OM_3D7K_GPIO_TP_RESET, 0);
}

static struct pcap7200_platform_data om_3d7k_pcap7200_pdata = {
	.mode = MULTI_TOUCH,
	.reset = om_3d7k_pcap7200_reset,
};

static struct i2c_board_info om_3d7k_i2c_devs[] __initdata = {
	{
		I2C_BOARD_INFO("pcf50633", 0x73),
		.irq = OM_3D7K_IRQ_PMU,
		.platform_data = &om_3d7k_pcf_pdata,
	},
	{
		I2C_BOARD_INFO("pcap7200", 0x0a),
		.irq = OM_3D7K_IRQ_TOUCH,
		.platform_data = &om_3d7k_pcap7200_pdata,
	},
	{
		I2C_BOARD_INFO("lp5521", 0x32),
		/* mark this temporarily, since LED INT is connected
		 * to EXT group6_9, the handling of EXT group1~group9
		 * is not implemented. Besides, we don't need this IRQ
		 * now
		 */
#if 0
		.irq = OM_3D7K_IRQ_LED,
#endif
		.platform_data = &om_3d7k_lp5521_pdata,
	},
	{
		I2C_BOARD_INFO("wm8753", 0x1a),
	},
};

struct platform_device s3c24xx_pwm_device = {
	.name 		= "s3c24xx_pwm",
	.num_resources	= 0,
};

struct platform_device om_3d7k_device_spi_lcm;

static struct platform_device *om_3d7k_devices[] __initdata = {
	&s3c_device_fb,
	&s3c_device_i2c0,
	&om_3d7k_device_spi_lcm,
	&s3c_device_usbgadget,
	&s3c24xx_pwm_device,
	&s3c_device_camif,
};


static void om_3d7k_pmu_regulator_registered(struct pcf50633 *pcf, int id)
{
	struct platform_device *regulator, *pdev;

	regulator = pcf->regulator_pdev[id];

	switch(id) {
		case PCF50633_REGULATOR_LDO4:
			pdev = &s3c_device_hsmmc0; /* uSD card */
			break;
		case PCF50633_REGULATOR_LDO5: /* GPS regulator */
			pdev = &om_3d7k_features_dev;
			break;
		case PCF50633_REGULATOR_LDO6:
			pdev = &om_3d7k_lcd_powerdev;
			break;
		default:
			return;
	}

	pdev->dev.parent = &regulator->dev;
	platform_device_register(pdev);
}

static struct platform_device *om_3d7k_devices_pmu_children[] = {
	&om_3d7k_button_dev,
//	&s3c_device_spi_acc1, /* relies on PMU reg for power */
};

/* this is called when pc50633 is probed, unfortunately quite late in the
 * day since it is an I2C bus device.  Here we can belatedly define some
 * platform devices with the advantage that we can mark the pcf50633 as the
 * parent.  This makes them get suspended and resumed with their parent
 * the pcf50633 still around.
 */

static void om_3d7k_pcf50633_attach_child_devices(struct pcf50633 *pcf)
{
	int n;

	om_3d7k_pcf = pcf;

	for (n = 0; n < ARRAY_SIZE(om_3d7k_devices_pmu_children); n++)
		om_3d7k_devices_pmu_children[n]->dev.parent = pcf->dev;

	platform_add_devices(om_3d7k_devices_pmu_children,
				     ARRAY_SIZE(om_3d7k_devices_pmu_children));

	/* backlight device should be registered until pcf50633 probe is done */
	om_3d7k_bl_dev.dev.parent = &om_3d7k_device_spi_lcm.dev;
	platform_device_register(&om_3d7k_bl_dev);

	/* Switch on backlight. Qi does not do it for us */
	pcf50633_reg_write(pcf, PCF50633_REG_LEDENA, 0x00);
	pcf50633_reg_write(pcf, PCF50633_REG_LEDDIM, 0x01);
	pcf50633_reg_write(pcf, PCF50633_REG_LEDENA, 0x01);

	/* @@@ do this properly later - WA */
	pcf50633_reg_write(om_3d7k_pcf, 0x30, 0x21);
	pcf50633_reg_write(om_3d7k_pcf, 0x39, 0x13);
	pcf50633_reg_write(om_3d7k_pcf, 0x3a, 0x21);
}

static void om_3d7k_l1k002_pwronoff(int level)
{
	gpio_direction_output(OM_3D7K_GPIO_LCM_SD, 1);
	udelay(15);

	gpio_direction_output(OM_3D7K_GPIO_LCM_RESET, !!level);

	if (level){
		udelay(15);
		gpio_direction_output(OM_3D7K_GPIO_LCM_SD, 0);
	}
}

const struct l1k002_platform_data om_3d7k_l1k002_pdata = {
	.pwr_onoff = om_3d7k_l1k002_pwronoff,
};

static struct spi_board_info om_3d7k_spi_board_info[] = {
	{
	.modalias	= "l1k002",
	.platform_data	= &om_3d7k_l1k002_pdata,
	/* controller_data */
	/* irq */
	.max_speed_hz	= 10 * 1000 * 1000,
	.bus_num	= 1,
	/* chip_select */
	},
};

static void spi_gpio_cs(struct s3c64xx_spigpio_info *spi, int csidx, int cs)
{
	switch (cs) {
	case BITBANG_CS_ACTIVE:
		gpio_direction_output(OM_3D7K_GPIO_LCM_CS, 0);
		break;
	case BITBANG_CS_INACTIVE:
		gpio_direction_output(OM_3D7K_GPIO_LCM_CS, 1);
		break;
	}
}

static struct s3c64xx_spigpio_info spi_gpio_cfg = {
	.pin_clk	= OM_3D7K_GPIO_LCM_CLK,
	.pin_mosi	= OM_3D7K_GPIO_LCM_MOSI,
	/* no pinout to MISO */
	.chip_select	= &spi_gpio_cs,
	.num_chipselect = 1,
	.bus_num	= 1,
};

struct platform_device om_3d7k_device_spi_lcm = {
	.name		  = "spi_s3c64xx_gpio",
	.id		  = 1,
	.dev = {
		.platform_data = &spi_gpio_cfg,
	},
};







extern void s3c64xx_init_io(struct map_desc *, int);

struct s3c_plat_otg_data s3c_hs_otg_plat_data = {
	.phyclk = 0
};


static void __init om_3d7k_map_io(void)
{
	s3c64xx_init_io(om_3d7k_6410_iodesc, ARRAY_SIZE(om_3d7k_6410_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(om_3d7k_uartcfgs, ARRAY_SIZE(om_3d7k_uartcfgs));
}

static void __init om_3d7k_machine_init(void)
{
	s3c_pm_init();

	s3c_device_usbgadget.dev.platform_data = &s3c_hs_otg_plat_data;

	s3c_i2c0_set_platdata(NULL);
	s3c_fb_set_platdata(&om_3d7k_lcd_pdata);

	i2c_register_board_info(0, om_3d7k_i2c_devs,
						 ARRAY_SIZE(om_3d7k_i2c_devs));

	spi_register_board_info(om_3d7k_spi_board_info,
		       				ARRAY_SIZE(om_3d7k_spi_board_info));

	platform_add_devices(om_3d7k_devices, ARRAY_SIZE(om_3d7k_devices));

	/* Register the HDQ and vibrator as children of pwm device */
	om_3d7k_hdq_device.dev.parent = &s3c24xx_pwm_device.dev;
	platform_device_register(&om_3d7k_hdq_device);
}

MACHINE_START(OM_3D7K, "OM-3D7K")
	/* Maintainer: Andy Green <andy@openmoko.com> */
	.phys_io	= S3C_PA_UART & 0xfff00000,
	.io_pg_offst	= (((u32)S3C_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C64XX_PA_SDRAM + 0x100,

	.init_irq	= s3c6410_init_irq,
	.map_io		= om_3d7k_map_io,
	.init_machine	= om_3d7k_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END

