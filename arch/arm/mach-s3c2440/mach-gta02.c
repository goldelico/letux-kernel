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

#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/regulator/machine.h>

#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/mbc.h>
#include <linux/mfd/pcf50633/adc.h>
#include <linux/mfd/pcf50633/gpio.h>
#include <linux/mfd/pcf50633/led.h>

#include <linux/lis302dl.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <mach/regs-irq.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>
#include <mach/fb.h>
#include <mach/mci.h>
#include <mach/ts.h>
#include <mach/spi.h>
#include <mach/spi-gpio.h>
#include <mach/usb-control.h>
#include <mach/regs-mem.h>

#include <mach/gta02.h>

#include <plat/regs-serial.h>
#include <plat/nand.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/pm.h>
#include <plat/udc.h>
#include <plat/iic.h>
#include <asm/plat-s3c24xx/neo1973.h>
#include <mach/neo1973-pm-gsm.h>
#include <mach/gta02-pm-wlan.h>

#include <linux/jbt6k74.h>

#include <linux/glamofb.h>

#include <mach/fiq_ipc_gta02.h>
#include "fiq_c_isr.h"
#include <linux/gta02_hdq.h>
#include <linux/bq27000_battery.h>

#include <linux/i2c.h>

#include "../plat-s3c24xx/neo1973_pm_gps.h"

#include <linux/ts_filter_linear.h>
#include <linux/ts_filter_mean.h>
#include <linux/ts_filter_median.h>
#include <linux/ts_filter_group.h>

/* arbitrates which sensor IRQ owns the shared SPI bus */
static spinlock_t motion_irq_lock;

/* define FIQ IPC struct */
/*
 * contains stuff FIQ ISR modifies and normal kernel code can see and use
 * this is defined in <arch/arm/mach-s3c2410/include/mach/fiq_ipc_gta02.h>, you should customize
 * the definition in there and include the same definition in your kernel
 * module that wants to interoperate with your FIQ code.
 */
struct fiq_ipc fiq_ipc;
EXPORT_SYMBOL(fiq_ipc);

#define DIVISOR_FROM_US(x) ((x) << 3)

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
FIQ_HANDLER_ENTRY(64, 64)
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
		fiq_ipc.hdq_transaction_ctr = fiq_ipc.hdq_request_ctr;
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
			fiq_ipc.hdq_transaction_ctr = fiq_ipc.hdq_request_ctr;
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
				fiq_ipc.hdq_transaction_ctr =
							fiq_ipc.hdq_request_ctr;

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
		fiq_ipc.hdq_transaction_ctr = fiq_ipc.hdq_request_ctr;
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
		fiq_ipc.hdq_transaction_ctr = fiq_ipc.hdq_request_ctr;

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
	struct pcf50633 *pcf = gta02_pcf_pdata.pcf;

	return pcf->mbc.usb_online;
}

static int gta02_get_charger_active_status(void)
{
	struct pcf50633 *pcf = gta02_pcf_pdata.pcf;

	return pcf->mbc.usb_active;
}


struct bq27000_platform_data bq27000_pdata = {
	.name = "battery",
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

#define ADC_NOM_CHG_DETECT_1A 6
#define ADC_NOM_CHG_DETECT_USB 43

static void
gta02_configure_pmu_for_charger(struct pcf50633 *pcf, void *unused, int res)
{
	int  ma;
	       
	/* Interpret charger type */
	if (res < ((ADC_NOM_CHG_DETECT_USB + ADC_NOM_CHG_DETECT_1A) / 2)) {

		/* Stop GPO driving out now that we have a IA charger */
		pcf50633_gpio_set(pcf, PCF50633_GPO, 0);
	
		ma = 1000;	
	} else
		ma = 100;

	pcf50633_mbc_usb_curlim_set(pcf, ma);
}

static struct delayed_work gta02_charger_work;
static int gta02_usb_vbus_draw;

static void gta02_charger_worker(struct work_struct *work)
{
	struct pcf50633 *pcf = gta02_pcf_pdata.pcf;

	if (gta02_usb_vbus_draw) {
		pcf50633_mbc_usb_curlim_set(pcf, gta02_usb_vbus_draw);
		return;
	} else {
		pcf50633_adc_async_read(pcf,
			PCF50633_ADCC1_MUX_ADCIN1,
			PCF50633_ADCC1_AVERAGE_16,
			gta02_configure_pmu_for_charger, NULL);
		return;
	}
}

#define GTA02_CHARGER_CONFIGURE_TIMEOUT ((3000 * HZ) / 1000)
static void gta02_pmu_event_callback(struct pcf50633 *pcf, int irq)
{
	if (irq == PCF50633_IRQ_USBINS) {
		schedule_delayed_work(&gta02_charger_work,
				GTA02_CHARGER_CONFIGURE_TIMEOUT);
		return;
	} else if (irq == PCF50633_IRQ_USBREM) {
		cancel_delayed_work_sync(&gta02_charger_work);
		gta02_usb_vbus_draw = 0;
	}
}

static struct platform_device gta01_pm_gps_dev = {
	.name		= "neo1973-pm-gps",
};

static struct platform_device gta01_pm_bt_dev = {
	.name		= "neo1973-pm-bt",
};

static struct platform_device gta02_pm_gsm_dev = {
	.name		= "neo1973-pm-gsm",
};

/* this is called when pc50633 is probed, unfortunately quite late in the
 * day since it is an I2C bus device.  Here we can belatedly define some
 * platform devices with the advantage that we can mark the pcf50633 as the
 * parent.  This makes them get suspended and resumed with their parent
 * the pcf50633 still around.
 */

static struct platform_device gta02_glamo_dev;
static void mangle_glamo_res_by_system_rev(void);

static void gta02_pmu_attach_child_devices(struct pcf50633 *pcf);
static void gta02_pmu_regulator_registered(struct pcf50633 *pcf, int id);

static struct platform_device gta02_pm_wlan_dev = {
	.name		= "gta02-pm-wlan",
};

static struct regulator_consumer_supply ldo4_consumers[] = {
	{
		.dev = &gta01_pm_bt_dev.dev,
		.supply = "BT_3V2",
	},
};

static struct regulator_consumer_supply ldo5_consumers[] = {
	{
		.dev = &gta01_pm_gps_dev.dev,
		.supply = "RF_3V",
	},
};

/*
 * We need this dummy thing to fill the regulator consumers
 */
static struct platform_device gta02_mmc_dev = {
	/* details filled in by glamo core */
};

static struct regulator_consumer_supply hcldo_consumers[] = {
	{
		.dev = &gta02_mmc_dev.dev,
		.supply = "SD_3V3",
	},
};

static char *gta02_batteries[] = {
	"battery",
};

struct pcf50633_platform_data gta02_pcf_pdata = {
	.resumers = {
		[0] = 	PCF50633_INT1_USBINS |
			PCF50633_INT1_USBREM |
			PCF50633_INT1_ALARM,
		[1] = 	PCF50633_INT2_ONKEYF,
		[2] = 	PCF50633_INT3_ONKEY1S,
		[3] = 	PCF50633_INT4_LOWSYS |
			PCF50633_INT4_LOWBAT |
			PCF50633_INT4_HIGHTMP,
	},

	.batteries = gta02_batteries,
	.num_batteries = ARRAY_SIZE(gta02_batteries),

	.reg_init_data = {
		[PCF50633_REGULATOR_AUTO] = {
			.constraints = {
				.min_uV = 3300000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.boot_on = 1,
				.apply_uV = 1,
				.state_mem = {
					.enabled = 1,
				},
			},
			.num_consumer_supplies = 0,
		},
		[PCF50633_REGULATOR_DOWN1] = {
			.constraints = {
				.min_uV = 1300000,
				.max_uV = 1600000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.boot_on = 1,
				.apply_uV = 1,
			},
			.num_consumer_supplies = 0,
		},
		[PCF50633_REGULATOR_DOWN2] = {
			.constraints = {
				.min_uV = 1800000,
				.max_uV = 1800000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
				.boot_on = 1,
				.state_mem = {
					.enabled = 1,
				},
			},
			.num_consumer_supplies = 0,
		},
		[PCF50633_REGULATOR_HCLDO] = {
			.constraints = {
				.min_uV = 2000000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
				.boot_on = 1,
			},
			.num_consumer_supplies = 1,
			.consumer_supplies = hcldo_consumers,
		},
		[PCF50633_REGULATOR_LDO1] = {
			.constraints = {
				.min_uV = 1300000,
				.max_uV = 1300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
			.num_consumer_supplies = 0,
		},
		[PCF50633_REGULATOR_LDO2] = {
			.constraints = {
				.min_uV = 3300000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
			.num_consumer_supplies = 0,
		},
		[PCF50633_REGULATOR_LDO3] = {
			.constraints = {
				.min_uV = 3000000,
				.max_uV = 3000000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
			.num_consumer_supplies = 0,
		},
		[PCF50633_REGULATOR_LDO4] = {
			.constraints = {
				.min_uV = 3200000,
				.max_uV = 3200000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
			.num_consumer_supplies = 1,
			.consumer_supplies = ldo4_consumers,
		},
		[PCF50633_REGULATOR_LDO5] = {
			.constraints = {
				.min_uV = 1500000,
				.max_uV = 1500000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
			.num_consumer_supplies = 1,
			.consumer_supplies = ldo5_consumers,
		},
		[PCF50633_REGULATOR_LDO6] = {
			.constraints = {
				.min_uV = 0,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
			},
			.num_consumer_supplies = 0,
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
			.num_consumer_supplies = 0,
		},

	},
	.probe_done = gta02_pmu_attach_child_devices,
	.regulator_registered = gta02_pmu_regulator_registered,
	.mbc_event_callback = gta02_pmu_event_callback,
};

static void mangle_pmu_pdata_by_system_rev(void)
{
	struct regulator_init_data *reg_init_data;

	reg_init_data = gta02_pcf_pdata.reg_init_data;

	switch (system_rev) {
	case GTA02v1_SYSTEM_REV:
		/* FIXME: this is only in v1 due to wrong PMU variant */
		reg_init_data[PCF50633_REGULATOR_DOWN2]
					.constraints.state_mem.enabled = 1;
		break;
	case GTA02v2_SYSTEM_REV:
	case GTA02v3_SYSTEM_REV:
	case GTA02v4_SYSTEM_REV:
	case GTA02v5_SYSTEM_REV:
	case GTA02v6_SYSTEM_REV:
		reg_init_data[PCF50633_REGULATOR_LDO1]
					.constraints.min_uV = 3300000;
		reg_init_data[PCF50633_REGULATOR_LDO1]
					.constraints.min_uV = 3300000;
		reg_init_data[PCF50633_REGULATOR_LDO1]
					.constraints.state_mem.enabled = 0;

		reg_init_data[PCF50633_REGULATOR_LDO5]
					.constraints.min_uV = 3000000;
		reg_init_data[PCF50633_REGULATOR_LDO5]
					.constraints.max_uV = 3000000;
		
		reg_init_data[PCF50633_REGULATOR_LDO6]
					.constraints.min_uV = 3000000;
		reg_init_data[PCF50633_REGULATOR_LDO6]
					.constraints.max_uV = 3000000;
		reg_init_data[PCF50633_REGULATOR_LDO6]
					.constraints.apply_uV = 1;
		break;
	default:
		break;
	}
}

#ifdef CONFIG_GTA02_HDQ
/* HDQ */

static void gta02_hdq_attach_child_devices(struct device *parent_device)
{
	switch (system_rev) {
	case GTA02v5_SYSTEM_REV:
	case GTA02v6_SYSTEM_REV:
		bq27000_battery_device.dev.parent = parent_device;
		platform_device_register(&bq27000_battery_device);
		break;
	default:
		break;
	}
}

static struct resource gta02_hdq_resources[] = {
	[0] = {
		.start	= GTA02v5_GPIO_HDQ,
		.end	= GTA02v5_GPIO_HDQ,
	},
};

struct gta02_hdq_platform_data gta02_hdq_platform_data = {
	.attach_child_devices = gta02_hdq_attach_child_devices
};

struct platform_device gta02_hdq_device = {
	.name 		= "gta02-hdq",
	.num_resources	= 1,
	.resource	= gta02_hdq_resources,
	.dev		= {
		.platform_data = &gta02_hdq_platform_data,
	},
};
#endif

/* vibrator (child of FIQ) */

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

/* FIQ, used PWM regs, so not child of PWM */

static void gta02_fiq_attach_child_devices(struct device *parent_device)
{
#ifdef CONFIG_GTA02_HDQ
	switch (system_rev) {
	case GTA02v5_SYSTEM_REV:
	case GTA02v6_SYSTEM_REV:
		gta02_hdq_device.dev.parent = parent_device;
		platform_device_register(&gta02_hdq_device);
		gta02_vibrator_dev.dev.parent = parent_device;
		platform_device_register(&gta02_vibrator_dev);
		break;
	default:
		break;
	}
#endif
}


static struct resource sc32440_fiq_resources[] = {
	[0] = {
		.flags	= IORESOURCE_IRQ,
		.start	= IRQ_TIMER3,
		.end	= IRQ_TIMER3,
	},
};

struct sc32440_fiq_platform_data gta02_sc32440_fiq_platform_data = {
	.attach_child_devices = gta02_fiq_attach_child_devices
};

struct platform_device sc32440_fiq_device = {
	.name 		= "sc32440_fiq",
	.num_resources	= 1,
	.resource	= sc32440_fiq_resources,
	.dev		= {
		.platform_data = &gta02_sc32440_fiq_platform_data,
	},
};

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


struct platform_device s3c24xx_pwm_device = {
	.name 		= "s3c24xx_pwm",
	.num_resources	= 0,
};

static struct i2c_board_info gta02_i2c_devs[] __initdata = {
	{
		I2C_BOARD_INFO("pcf50633", 0x73),
		.irq = GTA02_IRQ_PCF50633,
		.platform_data = &gta02_pcf_pdata,
	},
};

static struct s3c2410_nand_set gta02_nand_sets[] = {
	[0] = {
		.name		= "neo1973-nand",
		.nr_chips	= 1,
		.flags		= S3C2410_NAND_BBT,
	},
};

/* choose a set of timings derived from S3C@2442B MCP54 
 * data sheet (K5D2G13ACM-D075 MCP Memory)
 */

static struct s3c2410_platform_nand gta02_nand_info = {
	.tacls		= 0,
	.twrph0		= 25,
	.twrph1		= 15,
	.nr_sets	= ARRAY_SIZE(gta02_nand_sets),
	.sets		= gta02_nand_sets,
	.software_ecc	= 1,
};


static void gta02_s3c_mmc_set_power(unsigned char power_mode,
    unsigned short vdd)
{
	gta02_wlan_power(
	    power_mode == MMC_POWER_ON ||
	    power_mode == MMC_POWER_UP);
}


static struct s3c24xx_mci_pdata gta02_s3c_mmc_cfg = {
	.set_power	= gta02_s3c_mmc_set_power,
};

static void gta02_udc_command(enum s3c2410_udc_cmd_e cmd)
{
	switch (cmd) {
	case S3C2410_UDC_P_ENABLE:
		printk(KERN_DEBUG "%s S3C2410_UDC_P_ENABLE\n", __func__);
		neo1973_gpb_setpin(GTA02_GPIO_USB_PULLUP, 1);
		break;
	case S3C2410_UDC_P_DISABLE:
		printk(KERN_DEBUG "%s S3C2410_UDC_P_DISABLE\n", __func__);
		neo1973_gpb_setpin(GTA02_GPIO_USB_PULLUP, 0);
		break;
	case S3C2410_UDC_P_RESET:
		printk(KERN_DEBUG "%s S3C2410_UDC_P_RESET\n", __func__);
		/* FIXME! */
		break;
	default:
		break;
	}
}

/* get PMU to set USB current limit accordingly */

static void gta02_udc_vbus_draw(unsigned int ma)
{
        if (!gta02_pcf_pdata.pcf) {
		printk(KERN_ERR "********** NULL gta02_pcf_pdata.pcf *****\n");
		return;
	}

	gta02_usb_vbus_draw = ma;

	schedule_delayed_work(&gta02_charger_work,
				GTA02_CHARGER_CONFIGURE_TIMEOUT);
}

static struct s3c2410_udc_mach_info gta02_udc_cfg = {
	.vbus_draw	= gta02_udc_vbus_draw,
	.udc_command	= gta02_udc_command,

};


/* touchscreen configuration */

static struct ts_filter_linear_configuration gta02_ts_linear_config = {
	.constants = {1, 0, 0, 0, 1, 0, 1},	/* don't modify coords */
	.coord0 = 0,
	.coord1 = 1,
};

static struct ts_filter_group_configuration gta02_ts_group_config = {
	.extent = 12,
	.close_enough = 10,
	.threshold = 6,		/* at least half of the points in a group */
	.attempts = 10,
};

static struct ts_filter_median_configuration gta02_ts_median_config = {
	.extent = 20,
	.decimation_below = 3,
	.decimation_threshold = 8 * 3,
	.decimation_above = 4,
};

static struct ts_filter_mean_configuration gta02_ts_mean_config = {
	.bits_filter_length = 2, /* 4 points */
};

static struct s3c2410_ts_mach_info gta02_ts_cfg = {
	.delay = 10000,
	.presc = 0xff, /* slow as we can go */
	.filter_sequence = {
		[0] = &ts_filter_group_api,
		[1] = &ts_filter_median_api,
		[2] = &ts_filter_mean_api,
		[3] = &ts_filter_linear_api,
	},
	.filter_config = {
		[0] = &gta02_ts_group_config,
		[1] = &gta02_ts_median_config,
		[2] = &gta02_ts_mean_config,
		[3] = &gta02_ts_linear_config,
	},
};


static void gta02_bl_set_intensity(int intensity)
{
	struct pcf50633 *pcf = gta02_pcf_pdata.pcf;
	int old_intensity = pcf50633_reg_read(pcf, PCF50633_REG_LEDOUT);
	int ret;

	intensity >>= 2;

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

static struct generic_bl_info gta02_bl_info = {
	.name 			= "gta02-bl",
	.max_intensity 		= 0xff,
	.default_intensity 	= 0xff,
	.set_bl_intensity 	= gta02_bl_set_intensity,
};

static struct platform_device gta02_bl_dev = {
	.name		  = "generic-bl",
	.id		  = 1,
	.dev = {
		.platform_data = &gta02_bl_info,
	},
};

/* SPI: LCM control interface attached to Glamo3362 */

static void gta02_jbt6k74_reset(int devidx, int level)
{
	glamo_lcm_reset(level);
}	

static void gta02_jbt6k74_probe_completed(struct device *dev)
{
	struct pcf50633 *pcf = gta02_pcf_pdata.pcf;

	/* Switch on backlight. Qi does not do it for us */
	pcf50633_reg_write(pcf, PCF50633_REG_LEDOUT, 0x01);
	pcf50633_reg_write(pcf, PCF50633_REG_LEDENA, 0x00);
	pcf50633_reg_write(pcf, PCF50633_REG_LEDDIM, 0x01);
	pcf50633_reg_write(pcf, PCF50633_REG_LEDENA, 0x01);

	gta02_bl_dev.dev.parent = dev;
	platform_device_register(&gta02_bl_dev);
}

const struct jbt6k74_platform_data jbt6k74_pdata = {
	.reset		= gta02_jbt6k74_reset,
	.probe_completed = gta02_jbt6k74_probe_completed,
};

static struct spi_board_info gta02_spi_board_info[] = {
	{
		.modalias	= "jbt6k74",
		/* platform_data */
		.platform_data	= &jbt6k74_pdata,
		/* controller_data */
		/* irq */
		.max_speed_hz	= 100 * 1000,
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

struct lis302dl_platform_data lis302_pdata_top;
struct lis302dl_platform_data lis302_pdata_bottom;

/*
 * generic SPI RX and TX bitbang
 * only call with interrupts off!
 */

static void __gta02_lis302dl_bitbang(struct lis302dl_info *lis, u8 *tx,
					     int tx_bytes, u8 *rx, int rx_bytes)
{
	struct lis302dl_platform_data *pdata = lis->pdata;
	int n;
	u8 shifter = 0;
	unsigned long other_cs;

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

	if (&lis302_pdata_top == pdata)
		other_cs = lis302_pdata_bottom.pin_chip_select;
	else
		other_cs = lis302_pdata_top.pin_chip_select;

	s3c2410_gpio_setpin(other_cs, 1);
	s3c2410_gpio_setpin(pdata->pin_chip_select, 1);
	s3c2410_gpio_setpin(pdata->pin_clk, 1);
	s3c2410_gpio_setpin(pdata->pin_chip_select, 0);

	/* send the register index, r/w and autoinc bits */
	for (n = 0; n < (tx_bytes << 3); n++) {
		if (!(n & 7))
			shifter = ~tx[n >> 3];
		s3c2410_gpio_setpin(pdata->pin_clk, 0);
		s3c2410_gpio_setpin(pdata->pin_mosi, !(shifter & 0x80));
		s3c2410_gpio_setpin(pdata->pin_clk, 1);
		shifter <<= 1;
	}

	for (n = 0; n < (rx_bytes << 3); n++) { /* 8 bits each */
		s3c2410_gpio_setpin(pdata->pin_clk, 0);
		shifter <<= 1;
		if (s3c2410_gpio_getpin(pdata->pin_miso))
			shifter |= 1;
		if ((n & 7) == 7)
			rx[n >> 3] = shifter;
		s3c2410_gpio_setpin(pdata->pin_clk, 1);
	}
	s3c2410_gpio_setpin(pdata->pin_chip_select, 1);
	s3c2410_gpio_setpin(other_cs, 1);
}


static int gta02_lis302dl_bitbang_read_reg(struct lis302dl_info *lis, u8 reg)
{
	u8 data = 0xc0 | reg; /* read, autoincrement */
	unsigned long flags;

	local_irq_save(flags);

	__gta02_lis302dl_bitbang(lis, &data, 1, &data, 1);

	local_irq_restore(flags);

	return data;
}

static void gta02_lis302dl_bitbang_write_reg(struct lis302dl_info *lis, u8 reg,
									 u8 val)
{
	u8 data[2] = { 0x00 | reg, val }; /* write, no autoincrement */
	unsigned long flags;

	local_irq_save(flags);

	__gta02_lis302dl_bitbang(lis, &data[0], 2, NULL, 0);

	local_irq_restore(flags);

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
		.lis302dl_bitbang = __gta02_lis302dl_bitbang,
		.lis302dl_bitbang_reg_read = gta02_lis302dl_bitbang_read_reg,
		.lis302dl_bitbang_reg_write = gta02_lis302dl_bitbang_write_reg,
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
		.lis302dl_bitbang = __gta02_lis302dl_bitbang,
		.lis302dl_bitbang_reg_read = gta02_lis302dl_bitbang_read_reg,
		.lis302dl_bitbang_reg_write = gta02_lis302dl_bitbang_write_reg,
		.lis302dl_suspend_io = gta02_lis302dl_suspend_io,
};


static struct platform_device s3c_device_spi_acc1 = {
	.name		  = "lis302dl",
	.id		  = 1,
	.dev = {
		.platform_data = &lis302_pdata_top,
	},
};

static struct platform_device s3c_device_spi_acc2 = {
	.name		  = "lis302dl",
	.id		  = 2,
	.dev = {
		.platform_data = &lis302_pdata_bottom,
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
	[3] = {
		.start = 0,
		.end   = 0,
	},
	[4] = {
		.start = 0,
		.end   = 0,
	},
};

static struct platform_device gta02_button_dev = {
	.name		= "neo1973-button",
	.num_resources	= ARRAY_SIZE(gta02_button_resources),
	.resource	= gta02_button_resources,
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

static int gta02_glamo_can_set_mmc_power(void)
{
	switch (system_rev) {
		case GTA02v3_SYSTEM_REV:
		case GTA02v4_SYSTEM_REV:
		case GTA02v5_SYSTEM_REV:
		case GTA02v6_SYSTEM_REV:
			return 1;
	}

	return 0;
}

/* Smedia Glamo 3362 */

/*
 * we crank down SD Card clock dynamically when GPS is powered
 */

static int gta02_glamo_mci_use_slow(void)
{
	return neo1973_pm_gps_is_on();
}

static void gta02_glamo_external_reset(int level)
{
	s3c2410_gpio_setpin(GTA02_GPIO_3D_RESET, level);
	s3c2410_gpio_cfgpin(GTA02_GPIO_3D_RESET, S3C2410_GPIO_OUTPUT);
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
	.mmc_dev = &gta02_mmc_dev,
	.glamo_can_set_mci_power = gta02_glamo_can_set_mmc_power,
	.glamo_mci_use_slow = gta02_glamo_mci_use_slow,
	.glamo_irq_is_wired = glamo_irq_is_wired,
	.glamo_external_reset = gta02_glamo_external_reset
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

/* these are the guys that don't need to be children of PMU */

static struct platform_device *gta02_devices[] __initdata = {
	&gta02_version_device,
	&s3c_device_usb,
	&s3c_device_wdt,
	&gta02_memconfig_device,
	&s3c_device_sdi,
	&s3c_device_usbgadget,
	&s3c_device_nand,
	&gta02_nor_flash,

	&sc32440_fiq_device,
	&s3c24xx_pwm_device,
	&gta02_led_dev,
	&gta02_pm_wlan_dev, /* not dependent on PMU */

	&s3c_device_iis,
	&s3c_device_i2c0,
};

/* these guys DO need to be children of PMU */

static struct platform_device *gta02_devices_pmu_children[] = {
	&s3c_device_ts, /* input 1 */
	&gta02_pm_gsm_dev,
	&gta02_pm_usbhost_dev,
	&s3c_device_spi_acc1, /* input 2 */
	&s3c_device_spi_acc2, /* input 3 */
	&gta02_button_dev, /* input 4 */
	&gta02_resume_reason_device,
};

static void gta02_pmu_regulator_registered(struct pcf50633 *pcf, int id)
{
	struct platform_device *regulator, *pdev;

	regulator = pcf->pmic.pdev[id];

	switch(id) {
		case PCF50633_REGULATOR_LDO4:
			pdev = &gta01_pm_bt_dev;
			break;
		case PCF50633_REGULATOR_LDO5:
			pdev = &gta01_pm_gps_dev;
			break;
		case PCF50633_REGULATOR_HCLDO:
			pdev = &gta02_glamo_dev;
			break;
		default:
			return;	
	}
	
	pdev->dev.parent = &regulator->dev;
	platform_device_register(pdev);
}

/* this is called when pc50633 is probed, unfortunately quite late in the
 * day since it is an I2C bus device.  Here we can belatedly define some
 * platform devices with the advantage that we can mark the pcf50633 as the
 * parent.  This makes them get suspended and resumed with their parent
 * the pcf50633 still around.
 */

static void gta02_pmu_attach_child_devices(struct pcf50633 *pcf)
{
	int n;

	for (n = 0; n < ARRAY_SIZE(gta02_devices_pmu_children); n++)
		gta02_devices_pmu_children[n]->dev.parent = pcf->dev;

	mangle_glamo_res_by_system_rev();
	platform_add_devices(gta02_devices_pmu_children,
					ARRAY_SIZE(gta02_devices_pmu_children));
}

static void gta02_poweroff(void)
{
	pcf50633_reg_set_bit_mask(gta02_pcf_pdata.pcf, PCF50633_REG_OOCSHDWN,
		  PCF50633_OOCSHDWN_GOSTDBY, PCF50633_OOCSHDWN_GOSTDBY);
}

static void __init gta02_machine_init(void)
{
	int rc;

	/* set the panic callback to make AUX blink fast */
	panic_blink = gta02_panic_blink;

	switch (system_rev) {
	case GTA02v6_SYSTEM_REV:
		/* we need push-pull interrupt from motion sensors */
		lis302_pdata_top.open_drain = 0;
		lis302_pdata_bottom.open_drain = 0;
		break;
	default:
		break;
	}

	spin_lock_init(&motion_irq_lock);
	INIT_DELAYED_WORK(&gta02_charger_work, gta02_charger_worker);

	/* Glamo chip select optimization */
/*	 *((u32 *)(S3C2410_MEMREG(((1 + 1) << 2)))) = 0x1280; */

	/* do not force soft ecc if we are asked to use hardware_ecc */
	if (hardware_ecc_str[0] == '1')
		gta02_nand_info.software_ecc = 0;

	s3c_device_usb.dev.platform_data = &gta02_usb_info;
	s3c_device_nand.dev.platform_data = &gta02_nand_info;
	s3c_device_sdi.dev.platform_data = &gta02_s3c_mmc_cfg;

	/* acc sensor chip selects */
	s3c2410_gpio_setpin(S3C2410_GPD12, 1);
	s3c2410_gpio_cfgpin(S3C2410_GPD12, S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_setpin(S3C2410_GPD13, 1);
	s3c2410_gpio_cfgpin(S3C2410_GPD13, S3C2410_GPIO_OUTPUT);

	s3c24xx_udc_set_platdata(&gta02_udc_cfg);
	s3c_i2c0_set_platdata(NULL);
	set_s3c2410ts_info(&gta02_ts_cfg);
	
	mangle_glamo_res_by_system_rev();

	i2c_register_board_info(0, gta02_i2c_devs, ARRAY_SIZE(gta02_i2c_devs));

	mangle_pmu_pdata_by_system_rev();

	platform_add_devices(gta02_devices, ARRAY_SIZE(gta02_devices));

	s3c_pm_init();

	/* Make sure the modem can wake us up */
	set_irq_type(GTA02_IRQ_MODEM, IRQ_TYPE_EDGE_RISING);
	rc = request_irq(GTA02_IRQ_MODEM, gta02_modem_irq, IRQF_DISABLED,
			 "modem", NULL);
	if (rc < 0)
		printk(KERN_ERR "GTA02: can't request GSM modem wakeup IRQ\n");
	enable_irq_wake(GTA02_IRQ_MODEM);

	/* Make sure the wifi module can wake us up*/
	set_irq_type(GTA02_IRQ_WLAN_GPIO1, IRQ_TYPE_EDGE_RISING);
	rc = request_irq(GTA02_IRQ_WLAN_GPIO1, ar6000_wow_irq, IRQF_DISABLED,
			"ar6000", NULL);

	if (rc < 0)
		printk(KERN_ERR "GTA02: can't request ar6k wakeup IRQ\n");
	enable_irq_wake(GTA02_IRQ_WLAN_GPIO1);

	pm_power_off = gta02_poweroff;
}

void DEBUG_LED(int n)
{
//	int *p = NULL;
	switch (n) {
	case 0:
		neo1973_gpb_setpin(GTA02_GPIO_PWR_LED1, 1);
		break;
	case 1:
		neo1973_gpb_setpin(GTA02_GPIO_PWR_LED2, 1);
		break;
	default:
		neo1973_gpb_setpin(GTA02_GPIO_AUX_LED, 1);
		break;
	}
//	printk(KERN_ERR"die %d\n", *p);
}
EXPORT_SYMBOL_GPL(DEBUG_LED);

MACHINE_START(NEO1973_GTA02, "GTA02")
	.phys_io	= S3C2410_PA_UART,
	.io_pg_offst	= (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C2410_SDRAM_PA + 0x100,
	.map_io		= gta02_map_io,
	.init_irq	= s3c24xx_init_irq,
	.init_machine	= gta02_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
