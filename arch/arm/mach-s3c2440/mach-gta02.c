/*
 * linux/arch/arm/mach-s3c2440/mach-gta02.c
 *
 * S3C2440 Machine Support for the FIC GTA02 (Neo1973)
 *
 * Copyright (C) 2006-2007 by Openmoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
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
#include <linux/spi/glamo.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/mmc/host.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>

#include <linux/pcf50633.h>
#include <linux/lis302dl.h>

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

#include <asm/arch/gta02.h>

#include <asm/plat-s3c/regs-serial.h>
#include <asm/plat-s3c/nand.h>
#include <asm/plat-s3c24xx/devs.h>
#include <asm/plat-s3c24xx/cpu.h>
#include <asm/plat-s3c24xx/pm.h>
#include <asm/plat-s3c24xx/udc.h>
#include <asm/plat-s3c24xx/neo1973.h>
#include <asm/arch-s3c2410/neo1973-pm-gsm.h>

#include <linux/jbt6k74.h>

#include <linux/glamofb.h>

#include <asm/arch/fiq_ipc_gta02.h>
#include "fiq_c_isr.h"
#include <linux/gta02_hdq.h>
#include <linux/bq27000_battery.h>

#include "../plat-s3c24xx/neo1973_pm_gps.h"

/* arbitrates which sensor IRQ owns the shared SPI bus */
static spinlock_t motion_irq_lock;

/* the dependency of jbt / LCM on pcf50633 resume */
struct resume_dependency resume_dep_jbt_pcf;
/* the dependency of jbt / LCM on glamo resume */
struct resume_dependency resume_dep_jbt_glamo;
/* the dependency of Glamo MCI on pcf50633 resume (has to power SD slot) */
struct resume_dependency resume_dep_glamo_mci_pcf;


static int gta02_charger_online_status;
static int gta02_charger_active_status;

/* define FIQ IPC struct */
/*
 * contains stuff FIQ ISR modifies and normal kernel code can see and use
 * this is defined in <asm/arch/fiq_ipc_gta02.h>, you should customize
 * the definition in there and include the same definition in your kernel
 * module that wants to interoperate with your FIQ code.
 */
struct fiq_ipc fiq_ipc;
EXPORT_SYMBOL(fiq_ipc);

#define DIVISOR_FROM_US(x) ((x) << 1)

#define FIQ_DIVISOR_VIBRATOR DIVISOR_FROM_US(100)

#ifdef CONFIG_GTA02_HDQ
/* HDQ specific */
#define HDQ_SAMPLE_PERIOD_US 20
/* private HDQ FSM state -- all other info interesting for caller in fiq_ipc */
static enum hdq_bitbang_states hdq_state;
static u8 hdq_ctr;
static u8 hdq_ctr2;
static u8 hdq_bit;
static u8 hdq_shifter;
static u8 hdq_tx_data_done;

#define FIQ_DIVISOR_HDQ DIVISOR_FROM_US(HDQ_SAMPLE_PERIOD_US)
#endif
/* define FIQ ISR */

FIQ_HANDLER_START()
/* define your locals here -- no initializers though */
	u16 divisor;
FIQ_HANDLER_ENTRY(256, 512)
/* Your ISR here :-) */
	divisor = 0xffff;

	/* Vibrator servicing */

	if (fiq_ipc.vib_pwm_latched || fiq_ipc.vib_pwm) { /* not idle */
		if (((u8)_fiq_count_fiqs) == fiq_ipc.vib_pwm_latched)
			neo1973_gpb_setpin(fiq_ipc.vib_gpio_pin, 0);
		if (((u8)_fiq_count_fiqs) == 0) {
			fiq_ipc.vib_pwm_latched = fiq_ipc.vib_pwm;
			if (fiq_ipc.vib_pwm_latched)
				neo1973_gpb_setpin(fiq_ipc.vib_gpio_pin, 1);
		}
		divisor = FIQ_DIVISOR_VIBRATOR;
	}

#ifdef CONFIG_GTA02_HDQ
	/* HDQ servicing */

	switch (hdq_state) {
	case HDQB_IDLE:
		if (fiq_ipc.hdq_request_ctr == fiq_ipc.hdq_transaction_ctr)
			break;
		hdq_ctr = 210 / HDQ_SAMPLE_PERIOD_US;
		s3c2410_gpio_setpin(fiq_ipc.hdq_gpio_pin, 0);
		s3c2410_gpio_cfgpin(fiq_ipc.hdq_gpio_pin, S3C2410_GPIO_OUTPUT);
		hdq_tx_data_done = 0;
		hdq_state = HDQB_TX_BREAK;
		break;

	case HDQB_TX_BREAK: /* issue low for > 190us */
		if (--hdq_ctr == 0) {
			hdq_ctr = 60 / HDQ_SAMPLE_PERIOD_US;
			hdq_state = HDQB_TX_BREAK_RECOVERY;
			s3c2410_gpio_setpin(fiq_ipc.hdq_gpio_pin, 1);
		}
		break;

	case HDQB_TX_BREAK_RECOVERY: /* issue low for > 40us */
		if (--hdq_ctr)
			break;
		hdq_shifter = fiq_ipc.hdq_ads;
		hdq_bit = 8; /* 8 bits of ads / rw */
		hdq_tx_data_done = 0; /* doing ads */
		/* fallthru on last one */
	case HDQB_ADS_CALC:
		if (hdq_shifter & 1)
			hdq_ctr = 50 / HDQ_SAMPLE_PERIOD_US;
		else
			hdq_ctr = 120 / HDQ_SAMPLE_PERIOD_US;
		/* carefully precompute the other phase length */
		hdq_ctr2 = (210 - (hdq_ctr * HDQ_SAMPLE_PERIOD_US)) /
				HDQ_SAMPLE_PERIOD_US;
		hdq_state = HDQB_ADS_LOW;
		hdq_shifter >>= 1;
		hdq_bit--;
		s3c2410_gpio_setpin(fiq_ipc.hdq_gpio_pin, 0);
		break;

	case HDQB_ADS_LOW:
		if (--hdq_ctr)
			break;
		s3c2410_gpio_setpin(fiq_ipc.hdq_gpio_pin, 1);
		hdq_state = HDQB_ADS_HIGH;
		break;

	case HDQB_ADS_HIGH:
		if (--hdq_ctr2 > 1) /* account for HDQB_ADS_CALC */
			break;
		if (hdq_bit) { /* more bits to do */
			hdq_state = HDQB_ADS_CALC;
			break;
		}
		/* no more bits, wait it out until hdq_ctr2 exhausted */
		if (hdq_ctr2)
			break;
		/* ok no more bits and very last state */
		hdq_ctr = 60 / HDQ_SAMPLE_PERIOD_US;
		/* FIXME 0 = read */
		if (fiq_ipc.hdq_ads & 0x80) { /* write the byte out */
			 /* set delay before payload */
			hdq_ctr = 300 / HDQ_SAMPLE_PERIOD_US;
 			/* already high, no need to write */
			hdq_state = HDQB_WAIT_TX;
			break;
		}
		/* read the next byte */
		hdq_bit = 8; /* 8 bits of data */
		hdq_ctr = 3000 / HDQ_SAMPLE_PERIOD_US;
		hdq_state = HDQB_WAIT_RX;
		s3c2410_gpio_cfgpin(fiq_ipc.hdq_gpio_pin, S3C2410_GPIO_INPUT);
		break;

	case HDQB_WAIT_TX: /* issue low for > 40us */
		if (--hdq_ctr)
			break;
		if (!hdq_tx_data_done) { /* was that the data sent? */
			hdq_tx_data_done++;
			hdq_shifter = fiq_ipc.hdq_tx_data;
			hdq_bit = 8; /* 8 bits of data */
			hdq_state = HDQB_ADS_CALC; /* start sending */
			break;
		}
		fiq_ipc.hdq_error = 0;
		fiq_ipc.hdq_transaction_ctr++;
		hdq_state = HDQB_IDLE; /* all tx is done */
		/* idle in input mode, it's pulled up by 10K */
		s3c2410_gpio_cfgpin(fiq_ipc.hdq_gpio_pin, S3C2410_GPIO_INPUT);
		break;

	case HDQB_WAIT_RX: /* wait for battery to talk to us */
		if (s3c2410_gpio_getpin(fiq_ipc.hdq_gpio_pin) == 0) {
			/* it talks to us! */
			hdq_ctr2 = 1;
			hdq_bit = 8; /* 8 bits of data */
			/* timeout */
			hdq_ctr = 300 / HDQ_SAMPLE_PERIOD_US;
			hdq_state = HDQB_DATA_RX_LOW;
			break;
		}
		if (--hdq_ctr == 0) { /* timed out, error */
			fiq_ipc.hdq_error = 1;
			fiq_ipc.hdq_transaction_ctr++;
			hdq_state = HDQB_IDLE; /* abort */
		}
		break;

	/*
	 * HDQ basically works by measuring the low time of the bit cell
	 * 32-50us --> '1', 80 - 145us --> '0'
	 */

	case HDQB_DATA_RX_LOW:
		if (s3c2410_gpio_getpin(fiq_ipc.hdq_gpio_pin)) {
			fiq_ipc.hdq_rx_data >>= 1;
			if (hdq_ctr2 <= (65 / HDQ_SAMPLE_PERIOD_US))
				fiq_ipc.hdq_rx_data |= 0x80;

			if (--hdq_bit == 0) {
				fiq_ipc.hdq_error = 0;
				fiq_ipc.hdq_transaction_ctr++; /* done */
				hdq_state = HDQB_IDLE;
			} else
				hdq_state = HDQB_DATA_RX_HIGH;
			/* timeout */
			hdq_ctr = 1000 / HDQ_SAMPLE_PERIOD_US;
			hdq_ctr2 = 1;
			break;
		}
		hdq_ctr2++;
		if (--hdq_ctr)
			break;
		 /* timed out, error */
		fiq_ipc.hdq_error = 2;
		fiq_ipc.hdq_transaction_ctr++;
		hdq_state = HDQB_IDLE; /* abort */
		break;

	case HDQB_DATA_RX_HIGH:
		if (!s3c2410_gpio_getpin(fiq_ipc.hdq_gpio_pin)) {
			/* it talks to us! */
			hdq_ctr2 = 1;
			/* timeout */
			hdq_ctr = 400 / HDQ_SAMPLE_PERIOD_US;
			hdq_state = HDQB_DATA_RX_LOW;
			break;
		}
		if (--hdq_ctr)
			break;
		/* timed out, error */
		fiq_ipc.hdq_error = 3;
		fiq_ipc.hdq_transaction_ctr++;
		/* we're in input mode already */
		hdq_state = HDQB_IDLE; /* abort */
		break;
	}

	if (hdq_state != HDQB_IDLE) /* ie, not idle */
		if (divisor > FIQ_DIVISOR_HDQ)
			divisor = FIQ_DIVISOR_HDQ; /* keep us going */
#endif

	/* disable further timer interrupts if nobody has any work
	 * or adjust rate according to who still has work
	 *
	 * CAUTION: it means forground code must disable FIQ around
	 * its own non-atomic S3C2410_INTMSK changes... not common
	 * thankfully and taken care of by the fiq-basis patch
	 */
	if (divisor == 0xffff) /* mask the fiq irq source */
		__raw_writel(__raw_readl(S3C2410_INTMSK) | _fiq_ack_mask,
			     S3C2410_INTMSK);
	else /* still working, maybe at a different rate */
		__raw_writel(divisor, S3C2410_TCNTB(_fiq_timer_index));
	_fiq_timer_divisor = divisor;

FIQ_HANDLER_END()


/*
 * this gets called every 1ms when we paniced.
 */

static long gta02_panic_blink(long count)
{
	long delay = 0;
	static long last_blink;
	static char led;

	if (count - last_blink < 100) /* 200ms period, fast blink */
		return 0;

	led ^= 1;
	s3c2410_gpio_cfgpin(GTA02_GPIO_AUX_LED, S3C2410_GPIO_OUTPUT);
	neo1973_gpb_setpin(GTA02_GPIO_AUX_LED, led);

	last_blink = count;
	return delay;
}


/**
 * returns PCB revision information in b9,b8 and b2,b1,b0
 * Pre-GTA02 A6 returns 0x000
 *     GTA02 A6 returns 0x101
 *     ...
 */

int gta02_get_pcb_revision(void)
{
	int n;
	int u = 0;
	static unsigned long pinlist[] = {
		GTA02_PCB_ID1_0,
		GTA02_PCB_ID1_1,
		GTA02_PCB_ID1_2,
		GTA02_PCB_ID2_0,
		GTA02_PCB_ID2_1,
	};
	static int pin_offset[] = {
		0, 1, 2, 8, 9
	};

	for (n = 0 ; n < ARRAY_SIZE(pinlist); n++) {
		/*
		 * set the PCB version GPIO to be pulled-down input
		 * force low briefly first
		 */
		s3c2410_gpio_cfgpin(pinlist[n], S3C2410_GPIO_OUTPUT);
		s3c2410_gpio_setpin(pinlist[n], 0);
		/* misnomer: it is a pullDOWN in 2442 */
		s3c2410_gpio_pullup(pinlist[n], 1);
		s3c2410_gpio_cfgpin(pinlist[n], S3C2410_GPIO_INPUT);

		udelay(10);

		if (s3c2410_gpio_getpin(pinlist[n]))
			u |= 1 << pin_offset[n];

		/*
		* when not being interrogated, all of the revision GPIO
		* are set to output HIGH without pulldown so no current flows
		* if they are NC or pulled up.
		*/
		s3c2410_gpio_setpin(pinlist[n], 1);
		s3c2410_gpio_cfgpin(pinlist[n], S3C2410_GPIO_OUTPUT);
		/* misnomer: it is a pullDOWN in 2442 */
		s3c2410_gpio_pullup(pinlist[n], 0);
	}

	return u;
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

static struct map_desc gta02_iodesc[] __initdata = {
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

static struct s3c2410_uartcfg gta02_uartcfgs[] = {
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

/* BQ27000 Battery */

static int gta02_get_charger_online_status(void)
{
	return gta02_charger_online_status;
}

static int gta02_get_charger_active_status(void)
{
	return gta02_charger_active_status;
}


struct bq27000_platform_data bq27000_pdata = {
	.name = "bat",
	.rsense_mohms = 20,
	.hdq_read = gta02hdq_read,
	.hdq_write = gta02hdq_write,
	.hdq_initialized = gta02hdq_initialized,
	.get_charger_online_status = gta02_get_charger_online_status,
	.get_charger_active_status = gta02_get_charger_active_status
};

struct platform_device bq27000_battery_device = {
	.name 		= "bq27000-battery",
	.dev = {
		.platform_data = &bq27000_pdata,
	},
};


/* PMU driver info */

static int pmu_callback(struct device *dev, unsigned int feature,
			enum pmu_event event)
{
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
	return 0;
}

static struct platform_device gta01_pm_gps_dev = {
	.name		= "neo1973-pm-gps",
};

static struct platform_device gta01_pm_bt_dev = {
	.name		= "neo1973-pm-bt",
};

/* this is called when pc50633 is probed, unfortunately quite late in the
 * day since it is an I2C bus device.  Here we can belatedly define some
 * platform devices with the advantage that we can mark the pcf50633 as the
 * parent.  This makes them get suspended and resumed with their parent
 * the pcf50633 still around.
 */

static void gta02_pcf50633_attach_child_devices(struct device *parent_device)
{
	gta01_pm_gps_dev.dev.parent = parent_device;
	gta01_pm_bt_dev.dev.parent = parent_device;
	platform_device_register(&gta01_pm_bt_dev);
	platform_device_register(&gta01_pm_gps_dev);
}

static struct pcf50633_platform_data gta02_pcf_pdata = {
	.used_features	= PCF50633_FEAT_MBC |
			  PCF50633_FEAT_BBC |
			  PCF50633_FEAT_RTC |
			  PCF50633_FEAT_CHGCUR |
			  PCF50633_FEAT_BATVOLT |
			  PCF50633_FEAT_BATTEMP |
			  PCF50633_FEAT_PWM_BL,
	.onkey_seconds_sig_init = 4,
	.onkey_seconds_shutdown = 8,
	.cb		= &pmu_callback,
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
	/* warning: these get rewritten during machine init below
	 * depending on pcb variant
	 */
	.rails	= {
		[PCF50633_REGULATOR_AUTO] = {
			.name		= "io_3v3",
			.flags		= PMU_VRAIL_F_SUSPEND_ON,
			.voltage	= {
				.init	= 3300,
				.max	= 3300,
			},
		},
		[PCF50633_REGULATOR_DOWN1] = {
			.name		= "core_1v3",
			/* Wow, when we are going into suspend, after pcf50633
			 * runs its suspend (which happens real early since it
			 * is an i2c device) we are running out of the 22uF cap
			 * on core_1v3 rail !!!!
			 */
			.voltage	= {
				.init	= 1300,
				.max	= 1600,
			},
		},
		[PCF50633_REGULATOR_DOWN2] = {
			.name		= "core_1v8",
			.flags		= PMU_VRAIL_F_SUSPEND_ON,
			.voltage	= {
				.init	= 1800,
				.max	= 1800,
			},
		},
		[PCF50633_REGULATOR_HCLDO] = {
			.name		= "sd_3v3",
			.voltage	= {
				.init	= 2000,
				.max	= 3300,
			},
		},
		[PCF50633_REGULATOR_LDO1] = {
			.name		= "gsensor_3v3",
			.voltage	= {
				.init	= 1300,
				.max	= 1330,
			},
		},
		[PCF50633_REGULATOR_LDO2] = {
			.name		= "codec_3v3",
			.voltage	= {
				.init	= 3300,
				.max	= 3300,
			},
		},
		[PCF50633_REGULATOR_LDO3] = {
			.name		= "unused3",
			.voltage	= {
				.init	= 3000,
				.max	= 3000,
			},
		},
		[PCF50633_REGULATOR_LDO4] = {
			.name		= "bt_3v2",
			.voltage	= {
				.init	= 2500,
				.max	= 3300,
			},
		},
		[PCF50633_REGULATOR_LDO5] = {
			.name		= "rf3v",
			.voltage	= {
				.init	= 1500,
				.max	= 1500,
			},
		},
		[PCF50633_REGULATOR_LDO6] = {
			.name		= "lcm_3v",
			.flags = PMU_VRAIL_F_SUSPEND_ON,
			.voltage	= {
				.init	= 0,
				.max	= 3300,
			},
		},
		[PCF50633_REGULATOR_MEMLDO] = {
			.name		= "memldo",
			.flags = PMU_VRAIL_F_SUSPEND_ON,
			.voltage	= {
				.init	= 1800,
				.max	= 1800,
			},
		},
	},
	.defer_resume_backlight = 1,
	.resume_backlight_ramp_speed = 5,
	.attach_child_devices = gta02_pcf50633_attach_child_devices

};

#if 0 /* currently unused */
static void cfg_pmu_vrail(struct pmu_voltage_rail *vrail, char *name,
			  unsigned int flags, unsigned int init,
			  unsigned int max)
{
	vrail->name = name;
	vrail->flags = flags;
	vrail->voltage.init = init;
	vrail->voltage.max = max;
}
#endif

static void mangle_pmu_pdata_by_system_rev(void)
{
	switch (system_rev) {
	case GTA02v1_SYSTEM_REV:
		/* FIXME: this is only in v1 due to wrong PMU variant */
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_DOWN2].flags =
							PMU_VRAIL_F_SUSPEND_ON;
		break;
	case GTA02v2_SYSTEM_REV:
	case GTA02v3_SYSTEM_REV:
	case GTA02v4_SYSTEM_REV:
	case GTA02v5_SYSTEM_REV:
	case GTA02v6_SYSTEM_REV:
		/* we need to keep the 1.8V going since this is the SDRAM
		 * self-refresh voltage */
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_DOWN2].flags =
							PMU_VRAIL_F_SUSPEND_ON;
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_DOWN2].name =
							"io_1v8",
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_LDO1].name =
							"gsensor_3v3",
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_LDO1].voltage.init =
							3300;
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_LDO1].voltage.max =
							3300;
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_LDO1].flags &=
							~PMU_VRAIL_F_SUSPEND_ON;
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_LDO3].flags =
							PMU_VRAIL_F_UNUSED;
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_LDO5] = ((struct pmu_voltage_rail) {
							.name = "rf_3v",
							.voltage = {
								.init = 0,
								.max = 3000,
							}
						});
		gta02_pcf_pdata.rails[PCF50633_REGULATOR_LDO6] =
					((struct pmu_voltage_rail) {
						.name = "lcm_3v",
						.flags = PMU_VRAIL_F_SUSPEND_ON,
						.voltage = {
							.init = 3000,
							.max = 3000,
						}
					});
		break;
	default:
		break;
	}
}

static struct resource gta02_pmu_resources[] = {
	[0] = {
		.flags	= IORESOURCE_IRQ,
		.start	= GTA02_IRQ_PCF50633,
		.end	= GTA02_IRQ_PCF50633,
	},
};

struct platform_device gta02_pmu_dev = {
	.name 		= "pcf50633",
	.num_resources	= ARRAY_SIZE(gta02_pmu_resources),
	.resource	= gta02_pmu_resources,
	.dev		= {
		.platform_data = &gta02_pcf_pdata,
	},
};

/* FIQ */

static struct resource sc32440_fiq_resources[] = {
	[0] = {
		.flags	= IORESOURCE_IRQ,
		.start	= IRQ_TIMER3,
		.end	= IRQ_TIMER3,
	},
};

struct platform_device sc32440_fiq_device = {
	.name 		= "sc32440_fiq",
	.num_resources	= 1,
	.resource	= sc32440_fiq_resources,
};

#ifdef CONFIG_GTA02_HDQ
/* HDQ */

static struct resource gta02_hdq_resources[] = {
	[0] = {
		.start	= GTA02v5_GPIO_HDQ,
		.end	= GTA02v5_GPIO_HDQ,
	},
};

struct platform_device gta02_hdq_device = {
	.name 		= "gta02-hdq",
	.num_resources	= 1,
	.resource	= gta02_hdq_resources,
};
#endif


/* NOR Flash */

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



static struct resource gta02_sdio_resources[] = {
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


static struct platform_device gta02_sdio_dev = {
        .name           = "s3c24xx-sdio",
        .id             = -1,
        .dev            = {
                                .coherent_dma_mask      = 0xffffffff,
        },
        .resource       = gta02_sdio_resources,
        .num_resources  = ARRAY_SIZE(gta02_sdio_resources),
};

struct platform_device s3c24xx_pwm_device = {
	.name 		= "s3c24xx_pwm",
	.num_resources	= 0,
};


static struct platform_device *gta02_devices[] __initdata = {
	&s3c_device_usb,
	&s3c_device_wdt,
	&s3c_device_i2c,
	&s3c_device_iis,
	// &s3c_device_sdi, /* FIXME: temporary disable to avoid s3cmci bind */
	&s3c_device_usbgadget,
	&s3c_device_nand,
	&s3c_device_ts,
	&gta02_nor_flash,
	&sc32440_fiq_device,
	&gta02_version_device,
	&gta02_memconfig_device,
	&gta02_resume_reason_device,
	&s3c24xx_pwm_device,

};

static struct s3c2410_nand_set gta02_nand_sets[] = {
	[0] = {
		.name		= "neo1973-nand",
		.nr_chips	= 1,
		.flags		= S3C2410_NAND_BBT,
	},
};

/* choose a set of timings which should suit most 512Mbit
 * chips and beyond.
 */

static struct s3c2410_platform_nand gta02_nand_info = {
	.tacls		= 20,
	.twrph0		= 60,
	.twrph1		= 20,
	.nr_sets	= ARRAY_SIZE(gta02_nand_sets),
	.sets		= gta02_nand_sets,
	.software_ecc	= 1,
};

static struct s3c24xx_mci_pdata gta02_mmc_cfg = {
	.gpio_detect	= GTA02v1_GPIO_nSD_DETECT,
	.set_power	= NULL,
	.ocr_avail	= MMC_VDD_32_33,
};

static void gta02_udc_command(enum s3c2410_udc_cmd_e cmd)
{
	printk(KERN_DEBUG "%s(%d)\n", __func__, cmd);

	switch (cmd) {
	case S3C2410_UDC_P_ENABLE:
		neo1973_gpb_setpin(GTA02_GPIO_USB_PULLUP, 1);
		break;
	case S3C2410_UDC_P_DISABLE:
		neo1973_gpb_setpin(GTA02_GPIO_USB_PULLUP, 0);
		break;
	case S3C2410_UDC_P_RESET:
		/* FIXME! */
		break;
	default:
		break;
	}
}

/* get PMU to set USB current limit accordingly */

static void gta02_udc_vbus_draw(unsigned int ma)
{
        if (!pcf50633_global)
		return;

	pcf50633_notify_usb_current_limit_change(pcf50633_global, ma);
}

static struct s3c2410_udc_mach_info gta02_udc_cfg = {
	.vbus_draw	= gta02_udc_vbus_draw,
	.udc_command	= gta02_udc_command,

};

static struct s3c2410_ts_mach_info gta02_ts_cfg = {
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

static void gta02_jbt6k74_reset(int devidx, int level)
{
	glamo_lcm_reset(level);
}

/* finally bring up deferred backlight resume now LCM is resumed itself */

static void gta02_jbt6k74_resuming(int devidx)
{
	pcf50633_backlight_resume(pcf50633_global);
}

static int gta02_jbt6k74_all_dependencies_resumed(int devidx)
{
	if (!resume_dep_jbt_pcf.called_flag)
		return 0;

	if (!resume_dep_jbt_glamo.called_flag)
		return 0;

	return 1;
}

/* register jbt resume action to be dependent on pcf50633 and glamo resume */

static void gta02_jbt6k74_suspending(int devindex, struct spi_device *spi)
{
	void jbt6k74_resume(void *spi); /* little white lies about types */

	resume_dep_jbt_pcf.callback = jbt6k74_resume;
	resume_dep_jbt_pcf.context = (void *)spi;
	pcf50633_register_resume_dependency(pcf50633_global,
							   &resume_dep_jbt_pcf);
	resume_dep_jbt_glamo.callback = jbt6k74_resume;
	resume_dep_jbt_glamo.context = (void *)spi;
	glamo_register_resume_dependency(&resume_dep_jbt_glamo);
}


const struct jbt6k74_platform_data jbt6k74_pdata = {
	.reset		= gta02_jbt6k74_reset,
	.resuming	= gta02_jbt6k74_resuming,
	.suspending	= gta02_jbt6k74_suspending,
	.all_dependencies_resumed = gta02_jbt6k74_all_dependencies_resumed,
};

static struct spi_board_info gta02_spi_board_info[] = {
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

#if 0 /* currently this is not used and we use gpio spi */
static struct glamo_spi_info glamo_spi_cfg = {
	.board_size	= ARRAY_SIZE(gta02_spi_board_info),
	.board_info	= gta02_spi_board_info,
};
#endif /* 0 */

static struct glamo_spigpio_info glamo_spigpio_cfg = {
	.pin_clk	= GLAMO_GPIO10_OUTPUT,
	.pin_mosi	= GLAMO_GPIO11_OUTPUT,
	.pin_cs		= GLAMO_GPIO12_OUTPUT,
	.pin_miso	= 0,
	.board_size	= ARRAY_SIZE(gta02_spi_board_info),
	.board_info	= gta02_spi_board_info,
};

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

/* SPI: Accelerometers attached to SPI of s3c244x */

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

struct lis302dl_platform_data lis302_pdata[];

void gta02_lis302dl_bitbang_read(struct lis302dl_info *lis)
{
	struct lis302dl_platform_data *pdata = lis->pdata;
	u8 shifter = 0xc0 | LIS302DL_REG_OUT_X; /* read, autoincrement */
	int n, n1;
	unsigned long other_cs;
	unsigned long flags;
#ifdef DEBUG_SPEW_MS
	s8 x, y, z;
#endif

	local_irq_save(flags);

	/*
	 * Huh.. "quirk"... CS on this device is not really "CS" like you can
	 * expect.  Instead when 1 it selects I2C interface mode.  Because we
	 * have 2 devices on one interface, the "disabled" device when we talk
	 * to an "enabled" device sees the clocks as I2C clocks, creating
	 * havoc.
	 * I2C sees MOSI going LOW while CLK HIGH as a START action, we must
	 * ensure this is never issued.
	 */

	if (&lis302_pdata[0] == pdata)
		other_cs = lis302_pdata[1].pin_chip_select;
	else
		other_cs = lis302_pdata[0].pin_chip_select;

	s3c2410_gpio_setpin(other_cs, 1);
	s3c2410_gpio_setpin(pdata->pin_chip_select, 1);
	s3c2410_gpio_setpin(pdata->pin_clk, 1);
	s3c2410_gpio_setpin(pdata->pin_chip_select, 0);
	for (n = 0; n < 8; n++) { /* write the r/w, inc and address */
		s3c2410_gpio_setpin(pdata->pin_clk, 0);
		s3c2410_gpio_setpin(pdata->pin_mosi, (shifter >> 7) & 1);
		s3c2410_gpio_setpin(pdata->pin_clk, 0);
		s3c2410_gpio_setpin(pdata->pin_clk, 1);
		s3c2410_gpio_setpin(pdata->pin_clk, 1);
		shifter <<= 1;
	}

	for (n = 0; n < 5; n++) { /* 5 consequetive registers */
		for (n1 = 0; n1 < 8; n1++) { /* 8 bits each */
			s3c2410_gpio_setpin(pdata->pin_clk, 0);
			s3c2410_gpio_setpin(pdata->pin_clk, 0);
			shifter <<= 1;
			if (s3c2410_gpio_getpin(pdata->pin_miso))
				shifter |= 1;
			s3c2410_gpio_setpin(pdata->pin_clk, 1);
			s3c2410_gpio_setpin(pdata->pin_clk, 1);
		}
		switch (n) {
		case 0:
#ifdef DEBUG_SPEW_MS
			x = shifter;
#endif
			input_report_rel(lis->input_dev, REL_X, MG_PER_SAMPLE * (s8)shifter);
			break;
		case 2:
#ifdef DEBUG_SPEW_MS
			y = shifter;
#endif
			input_report_rel(lis->input_dev, REL_Y, MG_PER_SAMPLE * (s8)shifter);
			break;
		case 4:
#ifdef DEBUG_SPEW_MS
			z = shifter;
#endif
			input_report_rel(lis->input_dev, REL_Z, MG_PER_SAMPLE * (s8)shifter);
			break;
		}
	}
	s3c2410_gpio_setpin(pdata->pin_chip_select, 1);
	s3c2410_gpio_setpin(other_cs, 1);
	local_irq_restore(flags);

	input_sync(lis->input_dev);
#ifdef DEBUG_SPEW_MS
	printk("%s: %d %d %d\n", pdata->name, x, y, z);
#endif
}


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
}

struct lis302dl_platform_data lis302_pdata[] = {
	{
		.name		= "lis302-1 (top)",
		.pin_chip_select= S3C2410_GPD12,
		.pin_clk	= S3C2410_GPG7,
		.pin_mosi	= S3C2410_GPG6,
		.pin_miso	= S3C2410_GPG5,
		.open_drain	= 1, /* altered at runtime by PCB rev */
		.lis302dl_bitbang_read = gta02_lis302dl_bitbang_read,
		.lis302dl_suspend_io = gta02_lis302dl_suspend_io,
	}, {
		.name		= "lis302-2 (bottom)",
		.pin_chip_select= S3C2410_GPD13,
		.pin_clk	= S3C2410_GPG7,
		.pin_mosi	= S3C2410_GPG6,
		.pin_miso	= S3C2410_GPG5,
		.open_drain	= 1, /* altered at runtime by PCB rev */
		.lis302dl_bitbang_read = gta02_lis302dl_bitbang_read,
		.lis302dl_suspend_io = gta02_lis302dl_suspend_io,
	},
};

static struct spi_board_info gta02_spi_acc_bdinfo[] = {
	{
		.modalias	= "lis302dl",
		.platform_data	= &lis302_pdata[0],
		.irq		= GTA02_IRQ_GSENSOR_1,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 1,
		.chip_select	= 0,
		.mode		= SPI_MODE_3,
	},
	{
		.modalias	= "lis302dl",
		.platform_data	= &lis302_pdata[1],
		.irq		= GTA02_IRQ_GSENSOR_2,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 1,
		.chip_select	= 1,
		.mode		= SPI_MODE_3,
	},
};

static void spi_acc_cs(struct s3c2410_spigpio_info *spigpio_info,
		       int csid, int cs)
{
	struct lis302dl_platform_data * plat_data =
				(struct lis302dl_platform_data *)spigpio_info->
						     board_info->platform_data;
	switch (cs) {
	case BITBANG_CS_ACTIVE:
		s3c2410_gpio_setpin(plat_data[csid].pin_chip_select, 0);
		break;
	case BITBANG_CS_INACTIVE:
		s3c2410_gpio_setpin(plat_data[csid].pin_chip_select, 1);
		break;
	}
}

static struct s3c2410_spigpio_info spi_gpio_cfg = {
	.pin_clk	= S3C2410_GPG7,
	.pin_mosi	= S3C2410_GPG6,
	.pin_miso	= S3C2410_GPG5,
	.board_size	= ARRAY_SIZE(gta02_spi_acc_bdinfo),
	.board_info	= gta02_spi_acc_bdinfo,
	.chip_select	= &spi_acc_cs,
	.num_chipselect = 2,
};

static struct resource s3c_spi_acc_resource[] = {
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

static struct platform_device s3c_device_spi_acc = {
	.name		  = "spi_s3c24xx_gpio",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(s3c_spi_acc_resource),
	.resource	  = s3c_spi_acc_resource,
	.dev = {
		.platform_data = &spi_gpio_cfg,
	},
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

static struct platform_device gta02_pm_gsm_dev = {
	.name		= "neo1973-pm-gsm",
};

static struct platform_device gta02_pm_usbhost_dev = {
	.name		= "neo1973-pm-host",
};


/* USB */
static struct s3c2410_hcd_info gta02_usb_info = {
	.port[0]	= {
		.flags	= S3C_HCDFLG_USED,
	},
	.port[1]	= {
		.flags	= 0,
	},
};

static int glamo_irq_is_wired(void)
{
	int rc;
	int count = 0;

	/*
	* GTA02 S-Media IRQs prior to A5 are broken due to a lack of
	* a pullup on the INT# line.  Check for the bad behaviour.
	*/
	s3c2410_gpio_setpin(S3C2410_GPG4, 0);
	s3c2410_gpio_cfgpin(S3C2410_GPG4, S3C2410_GPG4_OUTP);
	s3c2410_gpio_cfgpin(S3C2410_GPG4, S3C2410_GPG4_INP);
	/*
	* we force it low ourselves for a moment and resume being input.
	* If there is a pullup, it won't stay low for long.  But if the
	* level converter is there as on < A5 revision, the weak keeper
	* on the input of the LC will hold the line low indefinitiely
	*/
	do
		rc = s3c2410_gpio_getpin(S3C2410_GPG4);
	while ((!rc) && ((count++) < 10));
	if (rc) { /* it got pulled back up, it's good */
		printk(KERN_INFO "Detected S-Media IRQ# pullup, "
		"enabling interrupt\n");
		return 0;
	} else  /* Gah we can't work with this level converter */
		printk(KERN_WARNING "** Detected bad IRQ# circuit found"
		" on pre-A5 GTA02: S-Media interrupt disabled **\n");
	return -ENODEV;
}


static void
gta02_glamo_mmc_set_power(unsigned char power_mode, unsigned short vdd)
{
	int mv = 1650;
	int timeout = 500;

	printk(KERN_DEBUG "mmc_set_power(power_mode=%u, vdd=%u\n",
	       power_mode, vdd);

	switch (system_rev) {
	case GTA02v1_SYSTEM_REV:
	case GTA02v2_SYSTEM_REV:
		break;
	case GTA02v3_SYSTEM_REV:
	case GTA02v4_SYSTEM_REV:
	case GTA02v5_SYSTEM_REV:
	case GTA02v6_SYSTEM_REV:
		switch (power_mode) {
		case MMC_POWER_ON:
		case MMC_POWER_UP:
			/* depend on pcf50633 driver init + not suspended */
			while (pcf50633_ready(pcf50633_global) && (timeout--))
				msleep(5);

			if (timeout < 0) {
				printk(KERN_ERR"gta02_glamo_mmc_set_power "
					     "BAILING on timeout\n");
				return;
			}
			/* select and set the voltage */
			if (vdd > 7)
				mv += 350 + 100 * (vdd - 8);
			printk(KERN_INFO "SD power -> %dmV\n", mv);
			pcf50633_voltage_set(pcf50633_global,
					     PCF50633_REGULATOR_HCLDO, mv);
			pcf50633_onoff_set(pcf50633_global,
					   PCF50633_REGULATOR_HCLDO, 1);
			break;
		case MMC_POWER_OFF:
			/* power off happens during suspend, when pcf50633 can
			 * be already gone and not coming back... just forget
			 * the action then because pcf50633 suspend already
			 * dealt with it, otherwise we spin forever
			 */
			if (pcf50633_ready(pcf50633_global))
				return;
			pcf50633_onoff_set(pcf50633_global,
					   PCF50633_REGULATOR_HCLDO, 0);
			break;
		}
		break;
	}
}


static int gta02_glamo_mci_all_dependencies_resumed(struct platform_device *dev)
{
	return resume_dep_glamo_mci_pcf.called_flag;
}

/* register jbt resume action to be dependent on pcf50633 and glamo resume */

static void gta02_glamo_mci_suspending(struct platform_device *dev)
{
#if defined(CONFIG_MFD_GLAMO_MCI) && defined(CONFIG_PM)
	resume_dep_glamo_mci_pcf.callback = (void *)dev->dev.driver->resume;
	resume_dep_glamo_mci_pcf.context = (void *)dev;
	pcf50633_register_resume_dependency(pcf50633_global,
						     &resume_dep_glamo_mci_pcf);
#endif
}



/* Smedia Glamo 3362 */

/*
 * we crank down SD Card clock dynamically when GPS is powered
 */

static int gta02_glamo_mci_use_slow(void)
{
	return neo1973_pm_gps_is_on();
}

static struct glamofb_platform_data gta02_glamo_pdata = {
	.width		= 43,
	.height		= 58,
	 /* 24.5MHz --> 40.816ns */
	.pixclock	= 40816,
	.left_margin	= 8,
	.right_margin	= 16,
	.upper_margin	= 2,
	.lower_margin	= 16,
	.hsync_len	= 8,
	.vsync_len	= 2,
	.fb_mem_size	= 0x400000, /* glamo has 8 megs of SRAM. we use 4 */
	.xres		= {
		.min	= 240,
		.max	= 640,
		.defval	= 480,
	},
	.yres		= {
		.min	= 320,
		.max	= 640,
		.defval	= 640,
	},
	.bpp		= {
		.min	= 16,
		.max	= 16,
		.defval	= 16,
	},
	//.spi_info	= &glamo_spi_cfg,
	.spigpio_info	= &glamo_spigpio_cfg,

	/* glamo MMC function platform data */
	.glamo_set_mci_power = gta02_glamo_mmc_set_power,
	.glamo_mci_use_slow = gta02_glamo_mci_use_slow,
	.glamo_irq_is_wired = glamo_irq_is_wired,
	.mci_suspending = gta02_glamo_mci_suspending,
	.mci_all_dependencies_resumed =
				      gta02_glamo_mci_all_dependencies_resumed,
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
		.start = GTA02v1_GPIO_3D_RESET,
		.end   = GTA02v1_GPIO_3D_RESET,
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

static void mangle_glamo_res_by_system_rev(void)
{
	switch (system_rev) {
	case GTA02v1_SYSTEM_REV:
		break;
	default:
		gta02_glamo_resources[2].start = GTA02_GPIO_3D_RESET;
		gta02_glamo_resources[2].end = GTA02_GPIO_3D_RESET;
		break;
	}

	switch (system_rev) {
	case GTA02v1_SYSTEM_REV:
	case GTA02v2_SYSTEM_REV:
	case GTA02v3_SYSTEM_REV:
	/* case GTA02v4_SYSTEM_REV: - FIXME: handle this later */
		/* The hardware is missing a pull-up resistor and thus can't
		 * support the Smedia Glamo IRQ */
		gta02_glamo_resources[1].start = 0;
		gta02_glamo_resources[1].end = 0;
		break;
	}
}

static void __init gta02_map_io(void)
{
	s3c24xx_init_io(gta02_iodesc, ARRAY_SIZE(gta02_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(gta02_uartcfgs, ARRAY_SIZE(gta02_uartcfgs));
}

static irqreturn_t gta02_modem_irq(int irq, void *param)
{
	printk(KERN_DEBUG "modem wakeup interrupt\n");
	gta_gsm_interrupts++;
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

static void __init gta02_machine_init(void)
{
	int rc;

	/* set the panic callback to make AUX blink fast */
	panic_blink = gta02_panic_blink;

	switch (system_rev) {
	case GTA02v6_SYSTEM_REV:
		/* we need push-pull interrupt from motion sensors */
		lis302_pdata[0].open_drain = 0;
		lis302_pdata[1].open_drain = 0;
		break;
	default:
		break;
	}

	spin_lock_init(&motion_irq_lock);

	/* do not force soft ecc if we are asked to use hardware_ecc */
	if (hardware_ecc_str[0] == '1')
		gta02_nand_info.software_ecc = 0;

	s3c_device_usb.dev.platform_data = &gta02_usb_info;
	s3c_device_nand.dev.platform_data = &gta02_nand_info;
	s3c_device_sdi.dev.platform_data = &gta02_mmc_cfg;

	/* Only GTA02v1 has a SD_DETECT GPIO.  Since the slot is not
	 * hot-pluggable, this is not required anyway */
	switch (system_rev) {
	case GTA02v1_SYSTEM_REV:
		break;
	default:
		gta02_mmc_cfg.gpio_detect = 0;
		break;
	}

	/* acc sensor chip selects */
	s3c2410_gpio_setpin(S3C2410_GPD12, 1);
	s3c2410_gpio_cfgpin(S3C2410_GPD12, S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_setpin(S3C2410_GPD13, 1);
	s3c2410_gpio_cfgpin(S3C2410_GPD13, S3C2410_GPIO_OUTPUT);

	s3c24xx_udc_set_platdata(&gta02_udc_cfg);
	set_s3c2410ts_info(&gta02_ts_cfg);

	/* FIXME: hardcoded WLAN module power-up */
	s3c2410_gpio_cfgpin(GTA02_CHIP_PWD, S3C2410_GPIO_OUTPUT);

	/* Power is down */
	s3c2410_gpio_setpin(GTA02_CHIP_PWD, 1);
	mdelay(100);

	switch (system_rev) {
	case GTA02v1_SYSTEM_REV:
		s3c2410_gpio_setpin(GTA02_CHIP_PWD, 0);
		break;
	default:
		/* Chip is in reset state */
		s3c2410_gpio_setpin(GTA02_GPIO_nWLAN_RESET, 0);
		s3c2410_gpio_cfgpin(GTA02_GPIO_nWLAN_RESET, S3C2410_GPIO_OUTPUT);
		mdelay(100);
		/* Power is up */
		s3c2410_gpio_setpin(GTA02_CHIP_PWD, 0);
		mdelay(100);
		/* Chip is out of reset */
		s3c2410_gpio_setpin(GTA02_GPIO_nWLAN_RESET, 1);
		break;
	}
	mangle_glamo_res_by_system_rev();
	platform_device_register(&gta02_glamo_dev);

	platform_device_register(&s3c_device_spi_acc);
	platform_device_register(&gta02_button_dev);
	platform_device_register(&gta02_pm_gsm_dev);
	platform_device_register(&gta02_pm_usbhost_dev);

	mangle_pmu_pdata_by_system_rev();
	platform_device_register(&gta02_pmu_dev);
	platform_device_register(&gta02_vibrator_dev);
	platform_device_register(&gta02_led_dev);


	platform_device_register(&gta02_sdio_dev);

	platform_add_devices(gta02_devices, ARRAY_SIZE(gta02_devices));

#ifdef CONFIG_GTA02_HDQ
	switch (system_rev) {
	case GTA02v5_SYSTEM_REV:
	case GTA02v6_SYSTEM_REV:
		platform_device_register(&gta02_hdq_device);
		platform_device_register(&bq27000_battery_device);
		break;
	default:
		break;
	}
#endif
	s3c2410_pm_init();

	/* Make sure the modem can wake us up */
	set_irq_type(GTA02_IRQ_MODEM, IRQT_RISING);
	rc = request_irq(GTA02_IRQ_MODEM, gta02_modem_irq, IRQF_DISABLED,
			 "modem", NULL);
	if (rc < 0)
		printk(KERN_ERR "GTA02: can't request GSM modem wakeup IRQ\n");
	enable_irq_wake(GTA02_IRQ_MODEM);

	/* Make sure the wifi module can wake us up*/
	set_irq_type(GTA02_IRQ_WLAN_GPIO1, IRQT_RISING);
	rc = request_irq(GTA02_IRQ_WLAN_GPIO1, ar6000_wow_irq, IRQF_DISABLED,
			"ar6000", NULL);

	if (rc < 0)
		printk(KERN_ERR "GTA02: can't request ar6k wakeup IRQ\n");
	enable_irq_wake(GTA02_IRQ_WLAN_GPIO1);
}

MACHINE_START(NEO1973_GTA02, "GTA02")
	.phys_io	= S3C2410_PA_UART,
	.io_pg_offst	= (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C2410_SDRAM_PA + 0x100,
	.map_io		= gta02_map_io,
	.init_irq	= s3c24xx_init_irq,
	.init_machine	= gta02_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
