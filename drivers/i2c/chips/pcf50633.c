/* Philips PCF50633 Power Management Unit (PMU) driver
 *
 * (C) 2006-2007 by Openmoko, Inc.
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
 * This driver is a monster ;) It provides the following features
 * - voltage control for a dozen different voltage domains
 * - charging control for main and backup battery
 * - rtc / alarm
 * - adc driver (hw_sensors like)
 * - backlight
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/watchdog.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/pcf50633.h>
#include <linux/apm-emulation.h>
#include <linux/jiffies.h>

#include <asm/mach-types.h>

#include "pcf50633.h"
#include <linux/resume-dependency.h>

#if 0
#define DEBUGP(x, args ...) printk("%s: " x, __FUNCTION__, ## args)
#define DEBUGPC(x, args ...) printk(x, ## args)
#else
#define DEBUGP(x, args ...)
#define DEBUGPC(x, args ...)
#endif

/***********************************************************************
 * Static data / structures
 ***********************************************************************/

static unsigned short normal_i2c[] = { 0x73, I2C_CLIENT_END };

I2C_CLIENT_INSMOD_1(pcf50633);

#define PCF50633_FIDX_CHG_ENABLED	0	/* Charger enabled */
#define PCF50633_FIDX_CHG_PRESENT	1	/* Charger present */
#define PCF50633_FIDX_CHG_ERR		3	/* Charger Error */
#define PCF50633_FIDX_CHG_PROT		4	/* Charger Protection */
#define PCF50633_FIDX_CHG_READY		5	/* Charging completed */
#define PCF50633_FIDX_PWR_PRESSED	8
#define PCF50633_FIDX_RTC_SECOND	9
#define PCF50633_FIDX_USB_PRESENT	10

#define PCF50633_F_CHG_ENABLED	(1 << PCF50633_FIDX_CHG_ENABLED)
#define PCF50633_F_CHG_PRESENT	(1 << PCF50633_FIDX_CHG_PRESENT)
#define PCF50633_F_CHG_ERR	(1 << PCF50633_FIDX_CHG_ERR)
#define PCF50633_F_CHG_PROT	(1 << PCF50633_FIDX_CHG_PROT)
#define PCF50633_F_CHG_READY	(1 << PCF50633_FIDX_CHG_READY)

#define PCF50633_F_CHG_MASK	0x000000fc

#define PCF50633_F_PWR_PRESSED	(1 << PCF50633_FIDX_PWR_PRESSED)
#define PCF50633_F_RTC_SECOND	(1 << PCF50633_FIDX_RTC_SECOND)
#define PCF50633_F_USB_PRESENT	(1 << PCF50633_FIDX_USB_PRESENT)

enum close_state {
	CLOSE_STATE_NOT,
	CLOSE_STATE_ALLOW = 0x2342,
};

enum charger_type {
	CHARGER_TYPE_NONE = 0,
	CHARGER_TYPE_HOSTUSB,
	CHARGER_TYPE_1A
};

#define ADC_NOM_CHG_DETECT_1A 6
#define ADC_NOM_CHG_DETECT_NONE 43

#define MAX_ADC_FIFO_DEPTH 8

enum pcf50633_suspend_states {
	PCF50633_SS_RUNNING,
	PCF50633_SS_STARTING_SUSPEND,
	PCF50633_SS_COMPLETED_SUSPEND,
	PCF50633_SS_RESUMING_BUT_NOT_US_YET,
	PCF50633_SS_STARTING_RESUME,
	PCF50633_SS_COMPLETED_RESUME,
};


struct pcf50633_data {
	struct i2c_client client;
	struct pcf50633_platform_data *pdata;
	struct backlight_device *backlight;
	struct mutex lock;
	unsigned int flags;
	unsigned int working;
	struct mutex working_lock;
	struct work_struct work;
	struct rtc_device *rtc;
	struct input_dev *input_dev;
	int allow_close;
	int onkey_seconds;
	int irq;
	enum pcf50633_suspend_states suspend_state;
	int usb_removal_count;
	u8 pcfirq_resume[5];
	int probe_completed;

	/* if he pulls battery while charging, we notice that and correctly
	 * report that the charger is idle.  But there is no interrupt that
	 * fires if he puts a battery back in and charging resumes.  So when
	 * the battery is pulled, we run this work function looking for
	 * either charger resumption or USB cable pull
	 */
	struct mutex working_lock_nobat;
	struct work_struct work_nobat;
	int working_nobat;
	int usb_removal_count_nobat;
	int jiffies_last_bat_ins;

	/* current limit notification handler stuff */
	struct mutex working_lock_usb_curlimit;
	struct work_struct work_usb_curlimit;
	int pending_curlimit;
	int usb_removal_count_usb_curlimit;

	int last_curlim_set;

	int coldplug_done; /* cleared by probe, set by first work service */
	int flag_bat_voltage_read; /* ipc to /sys batt voltage read func */

	int charger_adc_result_raw;
	enum charger_type charger_type;

	/* we have a FIFO of ADC measurement requests that are used only by
	 * the workqueue service code after the ADC completion interrupt
	 */
	int adc_queue_mux[MAX_ADC_FIFO_DEPTH]; /* which ADC input to use */
	int adc_queue_avg[MAX_ADC_FIFO_DEPTH]; /* amount of averaging */
	int adc_queue_head; /* head owned by foreground code */
	int adc_queue_tail; /* tail owned by service code */

#ifdef CONFIG_PM
	struct {
		u_int8_t ooctim2;
		/* enables are always [1] below
		 * I2C has limit of 32 sequential regs, so done in two lumps
		 * because it covers 33 register extent otherwise
		 */
		u_int8_t misc[PCF50633_REG_LEDDIM - PCF50633_REG_AUTOOUT + 1];
		/*  skip 1 reserved reg here */
		u_int8_t ldo[PCF50633_REG_HCLDOENA - PCF50633_REG_LDO1OUT + 1];
	} standby_regs;

	struct resume_dependency resume_dependency;
	int is_suspended;

#endif
};

static struct i2c_driver pcf50633_driver;

struct pcf50633_data *pcf50633_global;
EXPORT_SYMBOL_GPL(pcf50633_global);

static struct platform_device *pcf50633_pdev;

static void pcf50633_usb_curlim_set(struct pcf50633_data *pcf, int ma);
static void pcf50633_charge_enable(struct pcf50633_data *pcf, int on);


/***********************************************************************
 * Low-Level routines
 ***********************************************************************/

static int __reg_write(struct pcf50633_data *pcf, u_int8_t reg, u_int8_t val)
{
	if (pcf->suspend_state == PCF50633_SS_COMPLETED_SUSPEND) {
		dev_err(&pcf->client.dev, "__reg_write while suspended\n");
		dump_stack();
	}
	return i2c_smbus_write_byte_data(&pcf->client, reg, val);
}

static int reg_write(struct pcf50633_data *pcf, u_int8_t reg, u_int8_t val)
{
	int ret;

	mutex_lock(&pcf->lock);
	ret = __reg_write(pcf, reg, val);
	mutex_unlock(&pcf->lock);

	return ret;
}

static int32_t __reg_read(struct pcf50633_data *pcf, u_int8_t reg)
{
	int32_t ret;

	if (pcf->suspend_state == PCF50633_SS_COMPLETED_SUSPEND) {
		dev_err(&pcf->client.dev, "__reg_read while suspended\n");
		dump_stack();
	}
	ret = i2c_smbus_read_byte_data(&pcf->client, reg);

	return ret;
}

static u_int8_t reg_read(struct pcf50633_data *pcf, u_int8_t reg)
{
	int32_t ret;

	mutex_lock(&pcf->lock);
	ret = __reg_read(pcf, reg);
	mutex_unlock(&pcf->lock);

	return ret & 0xff;
}

static int reg_set_bit_mask(struct pcf50633_data *pcf,
			    u_int8_t reg, u_int8_t mask, u_int8_t val)
{
	int ret;
	u_int8_t tmp;

	val &= mask;

	mutex_lock(&pcf->lock);

	tmp = __reg_read(pcf, reg);
	tmp &= ~mask;
	tmp |= val;
	ret = __reg_write(pcf, reg, tmp);

	mutex_unlock(&pcf->lock);

	return ret;
}

static int reg_clear_bits(struct pcf50633_data *pcf, u_int8_t reg, u_int8_t val)
{
	int ret;
	u_int8_t tmp;

	mutex_lock(&pcf->lock);

	tmp = __reg_read(pcf, reg);
	tmp &= ~val;
	ret = __reg_write(pcf, reg, tmp);

	mutex_unlock(&pcf->lock);

	return ret;
}

/* asynchronously setup reading one ADC channel */
static void async_adc_read_setup(struct pcf50633_data *pcf,
				 int channel, int avg)
{
	channel &= PCF50633_ADCC1_ADCMUX_MASK;

	/* kill ratiometric, but enable ACCSW biasing */
	__reg_write(pcf, PCF50633_REG_ADCC2, 0x00);
	__reg_write(pcf, PCF50633_REG_ADCC3, 0x01);

	/* start ADC conversion of selected channel */
	__reg_write(pcf, PCF50633_REG_ADCC1, channel | avg |
		    PCF50633_ADCC1_ADCSTART | PCF50633_ADCC1_RES_10BIT);

}

static u_int16_t async_adc_complete(struct pcf50633_data *pcf)
{
	u_int16_t ret = (__reg_read(pcf, PCF50633_REG_ADCS1) << 2) |
			(__reg_read(pcf, PCF50633_REG_ADCS3) &
						  PCF50633_ADCS3_ADCDAT1L_MASK);

	DEBUGPC("adc result = %d\n", ret);

	return ret;
}




/***********************************************************************
 * Voltage / ADC
 ***********************************************************************/

static u_int8_t auto_voltage(unsigned int millivolts)
{
	if (millivolts < 1800)
		return 0;
	if (millivolts > 3800)
		return 0xff;

	millivolts -= 625;
	return millivolts/25;
}

static unsigned int auto_2voltage(u_int8_t bits)
{
	if (bits < 0x2f)
		return 0;
	return 625 + (bits * 25);
}

static u_int8_t down_voltage(unsigned int millivolts)
{
	if (millivolts < 625)
		return 0;
	else if (millivolts > 3000)
		return 0xff;

	millivolts -= 625;
	return millivolts/25;
}

static unsigned int down_2voltage(u_int8_t bits)
{
	return 625 + (bits*25);
}

static u_int8_t ldo_voltage(unsigned int millivolts)
{
	if (millivolts < 900)
		return 0;
	else if (millivolts > 3600)
		return 0x1f;

	millivolts -= 900;
	return millivolts/100;
}

static unsigned int ldo_2voltage(u_int8_t bits)
{
	bits &= 0x1f;
	return 900 + (bits * 100);
}

static const u_int8_t regulator_registers[__NUM_PCF50633_REGULATORS] = {
	[PCF50633_REGULATOR_AUTO]	= PCF50633_REG_AUTOOUT,
	[PCF50633_REGULATOR_DOWN1]	= PCF50633_REG_DOWN1OUT,
	[PCF50633_REGULATOR_DOWN2]	= PCF50633_REG_DOWN2OUT,
	[PCF50633_REGULATOR_MEMLDO]	= PCF50633_REG_MEMLDOOUT,
	[PCF50633_REGULATOR_LDO1]	= PCF50633_REG_LDO1OUT,
	[PCF50633_REGULATOR_LDO2]	= PCF50633_REG_LDO2OUT,
	[PCF50633_REGULATOR_LDO3]	= PCF50633_REG_LDO3OUT,
	[PCF50633_REGULATOR_LDO4]	= PCF50633_REG_LDO4OUT,
	[PCF50633_REGULATOR_LDO5]	= PCF50633_REG_LDO5OUT,
	[PCF50633_REGULATOR_LDO6]	= PCF50633_REG_LDO6OUT,
	[PCF50633_REGULATOR_HCLDO]	= PCF50633_REG_HCLDOOUT,
};

int pcf50633_onoff_set(struct pcf50633_data *pcf,
		       enum pcf50633_regulator_id reg, int on)
{
	u_int8_t addr;

	if (reg >= __NUM_PCF50633_REGULATORS)
		return -EINVAL;

	/* the *ENA register is always one after the *OUT register */
	addr = regulator_registers[reg] + 1;

	if (on == 0)
		reg_set_bit_mask(pcf, addr, PCF50633_REGULATOR_ON, 0);
	else
		reg_set_bit_mask(pcf, addr, PCF50633_REGULATOR_ON,
				 PCF50633_REGULATOR_ON);

	return 0;
}
EXPORT_SYMBOL_GPL(pcf50633_onoff_set);

int pcf50633_onoff_get(struct pcf50633_data *pcf,
		       enum pcf50633_regulator_id reg)
{
	u_int8_t val, addr;

	if (reg >= __NUM_PCF50633_REGULATORS)
		return -EINVAL;

	/* the *ENA register is always one after the *OUT register */
	addr = regulator_registers[reg] + 1;
	val = reg_read(pcf, addr) & PCF50633_REGULATOR_ON;

	return val;
}
EXPORT_SYMBOL_GPL(pcf50633_onoff_get);

int pcf50633_voltage_set(struct pcf50633_data *pcf,
			 enum pcf50633_regulator_id reg,
			 unsigned int millivolts)
{
	u_int8_t volt_bits;
	u_int8_t regnr;

	DEBUGP("pcf=%p, reg=%d, mvolts=%d\n", pcf, reg, millivolts);

	if (reg >= __NUM_PCF50633_REGULATORS)
		return -EINVAL;

	regnr = regulator_registers[reg];

	if (millivolts > pcf->pdata->rails[reg].voltage.max)
		return -EINVAL;

	switch (reg) {
	case PCF50633_REGULATOR_AUTO:
		volt_bits = auto_voltage(millivolts);
		break;
	case PCF50633_REGULATOR_DOWN1:
		volt_bits = down_voltage(millivolts);
		break;
	case PCF50633_REGULATOR_DOWN2:
		volt_bits = down_voltage(millivolts);
		break;
	case PCF50633_REGULATOR_LDO1:
	case PCF50633_REGULATOR_LDO2:
	case PCF50633_REGULATOR_LDO3:
	case PCF50633_REGULATOR_LDO4:
	case PCF50633_REGULATOR_LDO5:
	case PCF50633_REGULATOR_LDO6:
	case PCF50633_REGULATOR_HCLDO:
		volt_bits = ldo_voltage(millivolts);
		DEBUGP("ldo_voltage(0x%x)=%u\n", millivolts, volt_bits);
		break;
	default:
		return -EINVAL;
	}

	return reg_write(pcf, regnr, volt_bits);
}
EXPORT_SYMBOL_GPL(pcf50633_voltage_set);

unsigned int pcf50633_voltage_get(struct pcf50633_data *pcf,
			 enum pcf50633_regulator_id reg)
{
	u_int8_t volt_bits;
	u_int8_t regnr;
	unsigned int rc = 0;

	if (reg >= __NUM_PCF50633_REGULATORS)
		return -EINVAL;

	regnr = regulator_registers[reg];
	volt_bits = reg_read(pcf, regnr);

	switch (reg) {
	case PCF50633_REGULATOR_AUTO:
		rc = auto_2voltage(volt_bits);
		break;
	case PCF50633_REGULATOR_DOWN1:
		rc = down_2voltage(volt_bits);
		break;
	case PCF50633_REGULATOR_DOWN2:
		rc = down_2voltage(volt_bits);
		break;
	case PCF50633_REGULATOR_LDO1:
	case PCF50633_REGULATOR_LDO2:
	case PCF50633_REGULATOR_LDO3:
	case PCF50633_REGULATOR_LDO4:
	case PCF50633_REGULATOR_LDO5:
	case PCF50633_REGULATOR_LDO6:
	case PCF50633_REGULATOR_HCLDO:
		rc = ldo_2voltage(volt_bits);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL_GPL(pcf50633_voltage_get);

/* go into 'STANDBY' mode, i.e. power off the main CPU and peripherals */
void pcf50633_go_standby(void)
{
	reg_set_bit_mask(pcf50633_global, PCF50633_REG_OOCSHDWN,
		  PCF50633_OOCSHDWN_GOSTDBY, PCF50633_OOCSHDWN_GOSTDBY);
}
EXPORT_SYMBOL_GPL(pcf50633_go_standby);

void pcf50633_gpio_set(struct pcf50633_data *pcf, enum pcf50633_gpio gpio,
			int on)
{
	u_int8_t reg = gpio - PCF50633_GPIO1 + PCF50633_REG_GPIO1CFG;

	if (on)
		reg_set_bit_mask(pcf, reg, 0x0f, 0x07);
	else
		reg_set_bit_mask(pcf, reg, 0x0f, 0x00);
}
EXPORT_SYMBOL_GPL(pcf50633_gpio_set);

int pcf50633_gpio_get(struct pcf50633_data *pcf, enum pcf50633_gpio gpio)
{
	u_int8_t reg = gpio - PCF50633_GPIO1 + PCF50633_REG_GPIO1CFG;
	u_int8_t val = reg_read(pcf, reg) & 0x0f;

	if (val == PCF50633_GPOCFG_GPOSEL_1 ||
	    val == (PCF50633_GPOCFG_GPOSEL_0|PCF50633_GPOCFG_GPOSEL_INVERSE))
		return 1;

	return 0;
}
EXPORT_SYMBOL_GPL(pcf50633_gpio_get);

static int interpret_charger_type_from_adc(struct pcf50633_data *pcf,
					   int sample)
{
	/* 1A capable charger? */

	if (sample < ((ADC_NOM_CHG_DETECT_NONE + ADC_NOM_CHG_DETECT_1A) / 2))
		return CHARGER_TYPE_1A;

	/* well then, nothing in the USB hole, or USB host / unk adapter */

	if (pcf->flags & PCF50633_F_USB_PRESENT) /* ooh power is in there */
		return CHARGER_TYPE_HOSTUSB; /* HOSTUSB is the catchall */

	return CHARGER_TYPE_NONE; /* no really -- nothing in there */
}



static void configure_pmu_for_charger(struct pcf50633_data *pcf,
				      enum charger_type type)
{
	switch (type) {
	case CHARGER_TYPE_NONE:
		pcf50633_usb_curlim_set(pcf, 0);
		break;
	/*
	 * the PCF50633 has a feature that it will supply only excess current
	 * from the charger that is not used to power the device.  So this
	 * 500mA setting is "up to 500mA" according to that.
	 */
	case CHARGER_TYPE_HOSTUSB:
		/* USB subsystem should call pcf50633_usb_curlim_set to set
		 * what was negotiated with the host when it is enumerated
		 * successfully.  If we get called again after a good
		 * negotiation, we keep what was negotiated.  (Removal of
		 * USB plug destroys pcf->last_curlim_set to 0)
		 */
		if (pcf->last_curlim_set > 100)
			pcf50633_usb_curlim_set(pcf, pcf->last_curlim_set);
		else
			pcf50633_usb_curlim_set(pcf, 100);
		break;
	case CHARGER_TYPE_1A:
		pcf50633_usb_curlim_set(pcf, 1000);
		/*
		 * stop GPO / EN_HOSTUSB power driving out on the same
		 * USB power pins we have a 1A charger on right now!
		 */
		dev_dbg(&pcf->client.dev, "Charger -> CHARGER_TYPE_1A\n");
		__reg_write(pcf, PCF50633_GPO - PCF50633_GPIO1 +
				 PCF50633_REG_GPIO1CFG,
				 __reg_read(pcf, PCF50633_GPO - PCF50633_GPIO1 +
						 PCF50633_REG_GPIO1CFG) & 0xf0);
		break;
	}

	/* max out USB fast charge current -- actual current drawn is
	 * additionally limited by USB limit so no worries
	 */
	__reg_write(pcf, PCF50633_REG_MBCC5, 0xff);

}

static void trigger_next_adc_job_if_any(struct pcf50633_data *pcf)
{
	if (pcf->adc_queue_head == pcf->adc_queue_tail)
		return;
	async_adc_read_setup(pcf,
			     pcf->adc_queue_mux[pcf->adc_queue_tail],
			     pcf->adc_queue_avg[pcf->adc_queue_tail]);
}

static void add_request_to_adc_queue(struct pcf50633_data *pcf,
				     int mux, int avg)
{
	int old_head = pcf->adc_queue_head;
	pcf->adc_queue_mux[pcf->adc_queue_head] = mux;
	pcf->adc_queue_avg[pcf->adc_queue_head] = avg;

	pcf->adc_queue_head = (pcf->adc_queue_head + 1) &
			      (MAX_ADC_FIFO_DEPTH - 1);

	/* it was idle before we just added this?  we need to kick it then */
	if (old_head == pcf->adc_queue_tail)
		trigger_next_adc_job_if_any(pcf);
}

/*
 * we get run to handle servicing the async notification from USB stack that
 * we got enumerated and allowed to draw a particular amount of current
 */

static void pcf50633_work_usbcurlim(struct work_struct *work)
{
	struct pcf50633_data *pcf =
		    container_of(work, struct pcf50633_data, work_usb_curlimit);

	mutex_lock(&pcf->working_lock_usb_curlimit);

	/* just can't cope with it if we are suspending, don't reschedule */
	if ((pcf->suspend_state == PCF50633_SS_STARTING_SUSPEND) ||
	    (pcf->suspend_state == PCF50633_SS_COMPLETED_SUSPEND))
		goto bail;

	dev_dbg(&pcf->client.dev, "pcf50633_work_usbcurlim\n");

	if (!pcf->probe_completed)
		goto reschedule;

	/* we got a notification from USB stack before we completed resume...
	 * that can only make trouble, reschedule for a retry
	 */
	if (pcf->suspend_state &&
		     (pcf->suspend_state < PCF50633_SS_COMPLETED_RESUME))
		goto reschedule;

	/*
	 * did he pull USB before we managed to set the limit?
	 */
	if (pcf->usb_removal_count_usb_curlimit != pcf->usb_removal_count)
		goto bail;

	/* OK let's set the requested limit and finish */

	dev_dbg(&pcf->client.dev, "pcf50633_work_usbcurlim setting %dmA\n",
							 pcf->pending_curlimit);
	pcf50633_usb_curlim_set(pcf, pcf->pending_curlimit);

bail:
	mutex_unlock(&pcf->working_lock_usb_curlimit);
	return;

reschedule:
	dev_dbg(&pcf->client.dev, "pcf50633_work_usbcurlim rescheduling\n");
	if (!schedule_work(&pcf->work_usb_curlimit))
		dev_err(&pcf->client.dev, "curlim reschedule work "
							    "already queued\n");

	mutex_unlock(&pcf->working_lock_usb_curlimit);
	/* don't spew, delaying whatever else is happening */
	msleep(1);
}


/* this is an export to allow machine to set USB current limit according to
 * notifications of USB stack about enumeration state.  We spawn a work
 * function to handle the actual setting, because suspend / resume and such
 * can be in a bad state since this gets called externally asychronous to
 * anything else going on in pcf50633.
 */

int pcf50633_notify_usb_current_limit_change(struct pcf50633_data *pcf,
								unsigned int ma)
{
	/* can happen if he calls with pcf50633_global before probe
	 * have to bail with error since we can't even schedule the work
	 */
	if (!pcf) {
		printk(KERN_ERR "pcf50633_notify_usb_current_limit called with NULL pcf\n");
		return -EBUSY;
	}

	dev_dbg(&pcf->client.dev,
		 "pcf50633_notify_usb_current_limit_change %dmA\n", ma);

	/* prepare to detect USB power removal before we complete */
	pcf->usb_removal_count_usb_curlimit = pcf->usb_removal_count;

	pcf->pending_curlimit = ma;

	if (!schedule_work(&pcf->work_usb_curlimit))
		dev_err(&pcf->client.dev, "curlim work item already queued\n");

	return 0;
}
EXPORT_SYMBOL_GPL(pcf50633_notify_usb_current_limit_change);


/* we are run when we see a NOBAT situation, because there is no interrupt
 * source in pcf50633 that triggers on resuming charging.  It watches to see
 * if charging resumes, it reassesses the charging source if it does.  If the
 * USB power disappears, it is also a sign there must be a battery and it is
 * NOT being charged, so it exits since the next move must be USB insertion for
 * change of charger state
 */

static void pcf50633_work_nobat(struct work_struct *work)
{
	struct pcf50633_data *pcf =
			container_of(work, struct pcf50633_data, work_nobat);

	mutex_lock(&pcf->working_lock_nobat);
	pcf->working_nobat = 1;
	mutex_unlock(&pcf->working_lock_nobat);

	while (1) {
		msleep(1000);

		if (pcf->suspend_state != PCF50633_SS_RUNNING)
			continue;

		/* there's a battery in there now? */
		if (reg_read(pcf, PCF50633_REG_MBCS3) & 0x40) {

			pcf->jiffies_last_bat_ins = jiffies;

			/* figure out our charging stance */
			add_request_to_adc_queue(pcf, PCF50633_ADCC1_MUX_ADCIN1,
						     PCF50633_ADCC1_AVERAGE_16);
			goto bail;
		}

		/* he pulled USB cable since we were started?  exit then */
		if (pcf->usb_removal_count_nobat != pcf->usb_removal_count)
			goto bail;
	}

bail:
	mutex_lock(&pcf->working_lock_nobat);
	pcf->working_nobat = 0;
	mutex_unlock(&pcf->working_lock_nobat);
}


static void pcf50633_work(struct work_struct *work)
{
	struct pcf50633_data *pcf =
			container_of(work, struct pcf50633_data, work);
	u_int8_t pcfirq[5];
	int ret;
	int tail;

	mutex_lock(&pcf->working_lock);
	pcf->working = 1;

	/* sanity */
	if (!&pcf->client.dev)
		goto bail;

	/*
	 * if we are presently suspending, we are not in a position to deal
	 * with pcf50633 interrupts at all.
	 *
	 * Because we didn't clear the int pending registers, there will be
	 * no edge / interrupt waiting for us when we wake.  But it is OK
	 * because at the end of our resume, we call this workqueue function
	 * gratuitously, clearing the pending register and re-enabling
	 * servicing this interrupt.
	 */

	if ((pcf->suspend_state == PCF50633_SS_STARTING_SUSPEND) ||
	    (pcf->suspend_state == PCF50633_SS_COMPLETED_SUSPEND))
		goto bail;

	/*
	 * If we are inside suspend -> resume completion time we don't attempt
	 * service until we have fully resumed.  Although we could talk to the
	 * device as soon as I2C is up, the regs in the device which we might
	 * choose to modify as part of the service action have not been
	 * reloaded with their pre-suspend states yet.  Therefore we will
	 * defer our service if we are called like that until our resume has
	 * completed.
	 *
	 * This shouldn't happen any more because we disable servicing this
	 * interrupt in suspend and don't re-enable it until resume is
	 * completed.
	 */

	if (pcf->suspend_state &&
		(pcf->suspend_state != PCF50633_SS_COMPLETED_RESUME))
		goto reschedule;

	/* this is the case early in resume! Sanity check! */
	if (i2c_get_clientdata(&pcf->client) == NULL)
		goto reschedule;

	/*
	* datasheet says we have to read the five IRQ
	* status regs in one transaction
	*/
	ret = i2c_smbus_read_i2c_block_data(&pcf->client,
						PCF50633_REG_INT1,
						sizeof(pcfirq),
						pcfirq);
	if (ret != sizeof(pcfirq)) {
		dev_dbg(&pcf->client.dev,
			"Oh crap PMU IRQ register read failed -- "
			"retrying later %d\n", ret);
		/*
		 * it shouldn't fail, we no longer attempt to use
		 * I2C while it can be suspended.  But we don't have
		 * much option but to retry if if it ever did fail,
		 * because if we don't service the interrupt to clear
		 * it, we will never see another PMU interrupt edge.
		 */
		goto reschedule;
	}

	/* hey did we just resume? (because we don't get here unless we are
	 * running normally or the first call after resumption)
	 */

	if (pcf->suspend_state != PCF50633_SS_RUNNING) {
		/*
		* grab a copy of resume interrupt reasons
		* from pcf50633 POV
		*/
		memcpy(pcf->pcfirq_resume, pcfirq, sizeof(pcf->pcfirq_resume));

		/* pcf50633 resume is really really over now then */
		pcf->suspend_state = PCF50633_SS_RUNNING;
	}

	if (!pcf->coldplug_done) {
		DEBUGP("PMU Coldplug init\n");

		/* we used SECOND to kick ourselves started -- turn it off */
		pcfirq[0] &= ~PCF50633_INT1_SECOND;
		reg_set_bit_mask(pcf, PCF50633_REG_INT1M,
					PCF50633_INT1_SECOND,
					PCF50633_INT1_SECOND);

		/* coldplug the USB if present */
		if ((__reg_read(pcf, PCF50633_REG_MBCS1) &
		    (PCF50633_MBCS1_USBPRES | PCF50633_MBCS1_USBOK)) ==
		    (PCF50633_MBCS1_USBPRES | PCF50633_MBCS1_USBOK)) {
			DEBUGPC("COLD USBINS\n");
			input_report_key(pcf->input_dev, KEY_POWER2, 1);
			apm_queue_event(APM_POWER_STATUS_CHANGE);
			pcf->flags |= PCF50633_F_USB_PRESENT;
			if (pcf->pdata->cb)
				pcf->pdata->cb(&pcf->client.dev,
					PCF50633_FEAT_MBC, PMU_EVT_USB_INSERT);
		}

		/* figure out our initial charging stance */
		add_request_to_adc_queue(pcf, PCF50633_ADCC1_MUX_ADCIN1,
					      PCF50633_ADCC1_AVERAGE_16);

		pcf->coldplug_done = 1;
	}

	DEBUGP("INT1=0x%02x INT2=0x%02x INT3=0x%02x INT4=0x%02x INT5=0x%02x\n",
		pcfirq[0], pcfirq[1], pcfirq[2], pcfirq[3], pcfirq[4]);

	if (pcfirq[0] & PCF50633_INT1_ADPINS) {
		/* Charger inserted */
		DEBUGPC("ADPINS ");
		input_report_key(pcf->input_dev, KEY_BATTERY, 1);
		apm_queue_event(APM_POWER_STATUS_CHANGE);
		pcf->flags |= PCF50633_F_CHG_PRESENT;
		if (pcf->pdata->cb)
			pcf->pdata->cb(&pcf->client.dev,
				       PCF50633_FEAT_MBC, PMU_EVT_INSERT);
	}
	if (pcfirq[0] & PCF50633_INT1_ADPREM) {
		/* Charger removed */
		DEBUGPC("ADPREM ");
		input_report_key(pcf->input_dev, KEY_BATTERY, 0);
		apm_queue_event(APM_POWER_STATUS_CHANGE);
		pcf->flags &= ~PCF50633_F_CHG_PRESENT;
		if (pcf->pdata->cb)
			pcf->pdata->cb(&pcf->client.dev,
				       PCF50633_FEAT_MBC, PMU_EVT_REMOVE);
	}
	if (pcfirq[0] & PCF50633_INT1_USBINS) {
		DEBUGPC("USBINS ");
		input_report_key(pcf->input_dev, KEY_POWER2, 1);
		apm_queue_event(APM_POWER_STATUS_CHANGE);
		pcf->flags |= PCF50633_F_USB_PRESENT;
		if (pcf->pdata->cb)
			pcf->pdata->cb(&pcf->client.dev,
				       PCF50633_FEAT_MBC, PMU_EVT_USB_INSERT);
		msleep(500); /* debounce, allow to see any ID resistor */
		/* completion irq will figure out our charging stance */
		add_request_to_adc_queue(pcf, PCF50633_ADCC1_MUX_ADCIN1,
				     PCF50633_ADCC1_AVERAGE_16);
	}
	if (pcfirq[0] & PCF50633_INT1_USBREM &&
				!(pcfirq[0] & PCF50633_INT1_USBINS)) {
		/* the occurrence of USBINS and USBREM
		 * should be exclusive in one schedule work
		 */
		DEBUGPC("USBREM ");

		pcf->usb_removal_count++;

		/* only deal if we had understood it was in */
		if (pcf->flags & PCF50633_F_USB_PRESENT) {
			input_report_key(pcf->input_dev, KEY_POWER2, 0);
			apm_queue_event(APM_POWER_STATUS_CHANGE);
			pcf->flags &= ~PCF50633_F_USB_PRESENT;

			if (pcf->pdata->cb)
				pcf->pdata->cb(&pcf->client.dev,
					PCF50633_FEAT_MBC, PMU_EVT_USB_REMOVE);

			/* destroy any memory of grant of power from host */
			pcf->last_curlim_set = 0;

			/* completion irq will figure out our charging stance */
			add_request_to_adc_queue(pcf, PCF50633_ADCC1_MUX_ADCIN1,
					PCF50633_ADCC1_AVERAGE_16);
		}
	}
	if (pcfirq[0] & PCF50633_INT1_ALARM) {
		DEBUGPC("ALARM ");
		if (pcf->pdata->used_features & PCF50633_FEAT_RTC)
			rtc_update_irq(pcf->rtc, 1, RTC_AF | RTC_IRQF);
	}
	if (pcfirq[0] & PCF50633_INT1_SECOND) {
		if (pcf->flags & PCF50633_F_RTC_SECOND) {
			DEBUGPC("SECOND ");
			rtc_update_irq(pcf->rtc, 1, RTC_PF | RTC_IRQF);
		}

		if (pcf->onkey_seconds >= 0 &&
		    pcf->flags & PCF50633_F_PWR_PRESSED) {
			DEBUGP("ONKEY_SECONDS(%u, OOCSTAT=0x%02x) ",
				pcf->onkey_seconds,
				reg_read(pcf, PCF50633_REG_OOCSTAT));
			pcf->onkey_seconds++;
			if (pcf->onkey_seconds >=
			    pcf->pdata->onkey_seconds_sig_init) {
				/* Ask init to do 'ctrlaltdel' */
				/*
				 * currently Linux reacts badly to issuing a
				 * signal to PID #1 before init is started.
				 * What happens is that the next kernel thread
				 * to start, which is the JFFS2 Garbage
				 * collector in our case, gets the signal
				 * instead and proceeds to fail to fork --
				 * which is very bad.  Therefore we confirm
				 * PID #1 exists before issuing the signal
				 */
				if (find_task_by_pid(1)) {
					DEBUGPC("SIGINT(init) ");
					kill_proc(1, SIGINT, 1);
				}
				/* FIXME: what if userspace doesn't shut down? */
			}
			if (pcf->onkey_seconds >=
				pcf->pdata->onkey_seconds_shutdown) {
				DEBUGPC("Power Off ");
				pcf50633_go_standby();
			}
		}
	}

	if (pcfirq[1] & PCF50633_INT2_ONKEYF) {
		/* ONKEY falling edge (start of button press) */
		DEBUGPC("ONKEYF ");
		pcf->flags |= PCF50633_F_PWR_PRESSED;
		input_report_key(pcf->input_dev, KEY_POWER, 1);
	}
	if (pcfirq[1] & PCF50633_INT2_ONKEYR) {
		/* ONKEY rising edge (end of button press) */
		DEBUGPC("ONKEYR ");
		pcf->flags &= ~PCF50633_F_PWR_PRESSED;
		pcf->onkey_seconds = -1;
		input_report_key(pcf->input_dev, KEY_POWER, 0);
		/* disable SECOND interrupt in case RTC didn't
		 * request it */
		if (!(pcf->flags & PCF50633_F_RTC_SECOND))
			reg_set_bit_mask(pcf, PCF50633_REG_INT1M,
					 PCF50633_INT1_SECOND,
					 PCF50633_INT1_SECOND);
	}
	/* FIXME: we don't use EXTON1/2/3. thats why we skip it */

	if (pcfirq[2] & PCF50633_INT3_BATFULL) {
		DEBUGPC("BATFULL ");

		/* the problem is, we get a false BATFULL if we inserted battery
		 * while USB powered.  Defeat BATFULL if we recently inserted
		 * battery
		 */

		if ((jiffies - pcf->jiffies_last_bat_ins) < (HZ * 2)) {

			DEBUGPC("*** Ignoring BATFULL ***\n");

			ret = reg_read(pcf, PCF50633_REG_MBCC7) &
					PCF56033_MBCC7_USB_MASK;


			reg_set_bit_mask(pcf, PCF50633_REG_MBCC7,
					 PCF56033_MBCC7_USB_MASK,
					 PCF50633_MBCC7_USB_SUSPEND);

			reg_set_bit_mask(pcf, PCF50633_REG_MBCC7,
					 PCF56033_MBCC7_USB_MASK,
					 ret);
		} else {
			if (pcf->pdata->cb)
				pcf->pdata->cb(&pcf->client.dev,
				       PCF50633_FEAT_MBC, PMU_EVT_CHARGER_IDLE);
		}

		/* FIXME: signal this to userspace */
	}
	if (pcfirq[2] & PCF50633_INT3_CHGHALT) {
		DEBUGPC("CHGHALT ");
		/*
		 * this is really "battery not pulling current" -- it can
		 * appear with no battery attached
		 */
		if (pcf->pdata->cb)
			pcf->pdata->cb(&pcf->client.dev,
				       PCF50633_FEAT_MBC, PMU_EVT_CHARGER_CHANGE);
	}
	if (pcfirq[2] & PCF50633_INT3_THLIMON) {
		DEBUGPC("THLIMON ");
		pcf->flags |= PCF50633_F_CHG_PROT;
		if (pcf->pdata->cb)
			pcf->pdata->cb(&pcf->client.dev,
				       PCF50633_FEAT_MBC, PMU_EVT_CHARGER_CHANGE);
	}
	if (pcfirq[2] & PCF50633_INT3_THLIMOFF) {
		DEBUGPC("THLIMOFF ");
		pcf->flags &= ~PCF50633_F_CHG_PROT;
		if (pcf->pdata->cb)
			pcf->pdata->cb(&pcf->client.dev,
				       PCF50633_FEAT_MBC, PMU_EVT_CHARGER_CHANGE);
	}
	if (pcfirq[2] & PCF50633_INT3_USBLIMON) {
		DEBUGPC("USBLIMON ");
		if (pcf->pdata->cb)
			pcf->pdata->cb(&pcf->client.dev,
				       PCF50633_FEAT_MBC, PMU_EVT_CHARGER_CHANGE);
	}
	if (pcfirq[2] & PCF50633_INT3_USBLIMOFF) {
		DEBUGPC("USBLIMOFF ");
		if (pcf->pdata->cb)
			pcf->pdata->cb(&pcf->client.dev,
				       PCF50633_FEAT_MBC, PMU_EVT_CHARGER_CHANGE);
	}
	if (pcfirq[2] & PCF50633_INT3_ADCRDY) {
		/* ADC result ready */
		DEBUGPC("ADCRDY ");
		tail = pcf->adc_queue_tail;
		pcf->adc_queue_tail = (pcf->adc_queue_tail + 1) &
				      (MAX_ADC_FIFO_DEPTH - 1);

		switch (pcf->adc_queue_mux[tail]) {
		case PCF50633_ADCC1_MUX_BATSNS_RES: /* battery voltage */
			pcf->flag_bat_voltage_read = async_adc_complete(pcf);
			break;
		case PCF50633_ADCC1_MUX_ADCIN1: /* charger type */
			pcf->charger_adc_result_raw = async_adc_complete(pcf);
			pcf->charger_type = interpret_charger_type_from_adc(
					     pcf, pcf->charger_adc_result_raw);
			configure_pmu_for_charger(pcf, pcf->charger_type);
			break;
		default:
			async_adc_complete(pcf);
			break;
		}
		trigger_next_adc_job_if_any(pcf);
	}
	if (pcfirq[2] & PCF50633_INT3_ONKEY1S) {
		/* ONKEY pressed for more than 1 second */
		pcf->onkey_seconds = 0;
		DEBUGPC("ONKEY1S ");
		/* Tell PMU we are taking care of this */
		reg_set_bit_mask(pcf, PCF50633_REG_OOCSHDWN,
				 PCF50633_OOCSHDWN_TOTRST,
				 PCF50633_OOCSHDWN_TOTRST);
		/* enable SECOND interrupt (hz tick) */
		reg_clear_bits(pcf, PCF50633_REG_INT1M, PCF50633_INT1_SECOND);
	}

	if (pcfirq[3] & (PCF50633_INT4_LOWBAT|PCF50633_INT4_LOWSYS)) {
		if ((__reg_read(pcf, PCF50633_REG_MBCS1) &
		    (PCF50633_MBCS1_USBPRES | PCF50633_MBCS1_USBOK)) ==
		    (PCF50633_MBCS1_USBPRES | PCF50633_MBCS1_USBOK)) {
			/*
			 * hey no need to freak out, we have some kind of
			 * valid charger power to keep us going -- but note that
			 * we are not actually charging anything
			 */
			if (pcf->pdata->cb)
				pcf->pdata->cb(&pcf->client.dev,
				       PCF50633_FEAT_MBC, PMU_EVT_CHARGER_IDLE);

			reg_set_bit_mask(pcf, PCF50633_REG_MBCC1,
					PCF50633_MBCC1_RESUME,
					PCF50633_MBCC1_RESUME);
	
			/*
			 * Well, we are not charging anything right this second
			 * ... however in the next ~30s before we get the next
			 * NOBAT, he might insert a battery.  So we schedule a
			 * work function checking to see if
			 * we started charging something during that time.
			 * USB removal as well as charging terminates the work
			 * function so we can't get terminally confused
			 */
			mutex_lock(&pcf->working_lock_nobat);
			if (!pcf->working_nobat) {
				pcf->usb_removal_count_nobat =
							pcf->usb_removal_count;

				if (!schedule_work(&pcf->work_nobat))
					DEBUGPC("failed to schedule nobat\n");
			}
			mutex_unlock(&pcf->working_lock_nobat);


			DEBUGPC("(NO)BAT ");
		} else {
			/* Really low battery voltage, we have 8 seconds left */
			DEBUGPC("LOWBAT ");
			/*
			 * currently Linux reacts badly to issuing a signal to
			 * PID #1 before init is started.  What happens is that
			 * the next kernel thread to start, which is the JFFS2
			 * Garbage collector in our case, gets the signal
			 * instead and proceeds to fail to fork -- which is
			 * very bad.  Therefore we confirm PID #1 exists
			 * before issuing SPIGPWR
			 */
			if (find_task_by_pid(1)) {
				apm_queue_event(APM_LOW_BATTERY);
				DEBUGPC("SIGPWR(init) ");
				kill_proc(1, SIGPWR, 1);
			} else
				/*
				 * well, our situation is like this:  we do not
				 * have any external power, we have a low
				 * battery and since PID #1 doesn't exist yet,
				 * we are early in the boot, likely before
				 * rootfs mount.  We should just call it a day
				 */
				apm_queue_event(APM_CRITICAL_SUSPEND);
		}

		/* Tell PMU we are taking care of this */
		reg_set_bit_mask(pcf, PCF50633_REG_OOCSHDWN,
				 PCF50633_OOCSHDWN_TOTRST,
				 PCF50633_OOCSHDWN_TOTRST);
	}
	if (pcfirq[3] & PCF50633_INT4_HIGHTMP) {
		/* High temperature */
		DEBUGPC("HIGHTMP ");
		apm_queue_event(APM_CRITICAL_SUSPEND);
	}
	if (pcfirq[3] & PCF50633_INT4_AUTOPWRFAIL) {
		DEBUGPC("PCF50633_INT4_AUTOPWRFAIL ");
		/* FIXME: deal with this */
	}
	if (pcfirq[3] & PCF50633_INT4_DWN1PWRFAIL) {
		DEBUGPC("PCF50633_INT4_DWN1PWRFAIL ");
		/* FIXME: deal with this */
	}
	if (pcfirq[3] & PCF50633_INT4_DWN2PWRFAIL) {
		DEBUGPC("PCF50633_INT4_DWN2PWRFAIL ");
		/* FIXME: deal with this */
	}
	if (pcfirq[3] & PCF50633_INT4_LEDPWRFAIL) {
		DEBUGPC("PCF50633_INT4_LEDPWRFAIL ");
		/* FIXME: deal with this */
	}
	if (pcfirq[3] & PCF50633_INT4_LEDOVP) {
		DEBUGPC("PCF50633_INT4_LEDOVP ");
		/* FIXME: deal with this */
	}

	DEBUGPC("\n");

bail:
	pcf->working = 0;
	input_sync(pcf->input_dev);
	put_device(&pcf->client.dev);
	mutex_unlock(&pcf->working_lock);

	return;

reschedule:
	/* don't spew, delaying whatever else is happening */
	/* EXCEPTION: if we are in the middle of suspending, we don't have
	 * time to hang around since we may be turned off core 1V3 already
	 */
	if ((pcf->suspend_state != PCF50633_SS_STARTING_SUSPEND) &&
	    (pcf->suspend_state != PCF50633_SS_COMPLETED_SUSPEND)) {
		msleep(10);
		dev_dbg(&pcf->client.dev, "rescheduling interrupt service\n");
	}
	if (!schedule_work(&pcf->work))
		dev_err(&pcf->client.dev, "int service reschedule failed\n");

	/* we don't put the device here, hold it for next time */
	mutex_unlock(&pcf->working_lock);
}

static irqreturn_t pcf50633_irq(int irq, void *_pcf)
{
	struct pcf50633_data *pcf = _pcf;

	DEBUGP("entering(irq=%u, pcf=%p): scheduling work\n", irq, _pcf);
	dev_dbg(&pcf->client.dev, "pcf50633_irq scheduling work\n");

	get_device(&pcf->client.dev);
	if (!schedule_work(&pcf->work) && !pcf->working)
		dev_err(&pcf->client.dev, "pcf irq work already queued\n");

	return IRQ_HANDLED;
}

static u_int16_t adc_to_batt_millivolts(u_int16_t adc)
{
	u_int16_t mvolts;

	mvolts = (adc * 6000) / 1024;

	return mvolts;
}

#define BATTVOLT_SCALE_START 2800
#define BATTVOLT_SCALE_END 4200
#define BATTVOLT_SCALE_DIVIDER ((BATTVOLT_SCALE_END - BATTVOLT_SCALE_START)/100)

static u_int8_t battvolt_scale(u_int16_t battvolt)
{
	/* FIXME: this linear scale is completely bogus */
	u_int16_t battvolt_relative = battvolt - BATTVOLT_SCALE_START;
	unsigned int percent = battvolt_relative / BATTVOLT_SCALE_DIVIDER;

	return percent;
}

u_int16_t pcf50633_battvolt(struct pcf50633_data *pcf)
{
	int count = 10;

	pcf->flag_bat_voltage_read = -1;
	add_request_to_adc_queue(pcf, PCF50633_ADCC1_MUX_BATSNS_RES,
				      PCF50633_ADCC1_AVERAGE_16);

	while ((count--) && (pcf->flag_bat_voltage_read < 0))
		msleep(1);

	if (count < 0) { /* timeout somehow */
		DEBUGPC("pcf50633_battvolt timeout :-(\n");
		return -1;
	}

	return adc_to_batt_millivolts(pcf->flag_bat_voltage_read);
}
EXPORT_SYMBOL_GPL(pcf50633_battvolt);

static ssize_t show_battvolt(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);

	return sprintf(buf, "%u\n", pcf50633_battvolt(pcf));
}
static DEVICE_ATTR(battvolt, S_IRUGO | S_IWUSR, show_battvolt, NULL);

static int reg_id_by_name(const char *name)
{
	int reg_id;

	if (!strcmp(name, "voltage_auto"))
		reg_id = PCF50633_REGULATOR_AUTO;
	else if (!strcmp(name, "voltage_down1"))
		reg_id = PCF50633_REGULATOR_DOWN1;
	else if (!strcmp(name, "voltage_down2"))
		reg_id = PCF50633_REGULATOR_DOWN2;
	else if (!strcmp(name, "voltage_memldo"))
		reg_id = PCF50633_REGULATOR_MEMLDO;
	else if (!strcmp(name, "voltage_ldo1"))
		reg_id = PCF50633_REGULATOR_LDO1;
	else if (!strcmp(name, "voltage_ldo2"))
		reg_id = PCF50633_REGULATOR_LDO2;
	else if (!strcmp(name, "voltage_ldo3"))
		reg_id = PCF50633_REGULATOR_LDO3;
	else if (!strcmp(name, "voltage_ldo4"))
		reg_id = PCF50633_REGULATOR_LDO4;
	else if (!strcmp(name, "voltage_ldo5"))
		reg_id = PCF50633_REGULATOR_LDO5;
	else if (!strcmp(name, "voltage_ldo6"))
		reg_id = PCF50633_REGULATOR_LDO6;
	else if (!strcmp(name, "voltage_hcldo"))
		reg_id = PCF50633_REGULATOR_HCLDO;
	else
		reg_id = -1;

	return reg_id;
}

static ssize_t show_vreg(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);
	unsigned int reg_id;

	reg_id = reg_id_by_name(attr->attr.name);
	if (reg_id < 0)
		return 0;

	if (pcf50633_onoff_get(pcf, reg_id) > 0)
		return sprintf(buf, "%u\n", pcf50633_voltage_get(pcf, reg_id));
	else
		return strlcpy(buf, "0\n", PAGE_SIZE);
}

static ssize_t set_vreg(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);
	unsigned long mvolts = simple_strtoul(buf, NULL, 10);
	unsigned int reg_id;

	reg_id = reg_id_by_name(attr->attr.name);
	if (reg_id < 0)
		return -EIO;

	DEBUGP("attempting to set %s(%d) to %lu mvolts\n", attr->attr.name,
		reg_id, mvolts);

	if (mvolts == 0) {
		pcf50633_onoff_set(pcf, reg_id, 0);
	} else {
		if (pcf50633_voltage_set(pcf, reg_id, mvolts) < 0) {
			dev_warn(dev, "refusing to set %s(%d) to %lu mvolts "
				 "(max=%u)\n", attr->attr.name, reg_id, mvolts,
				 pcf->pdata->rails[reg_id].voltage.max);
			return -EINVAL;
		}
		pcf50633_onoff_set(pcf, reg_id, 1);
	}

	return count;
}

static DEVICE_ATTR(voltage_auto, S_IRUGO | S_IWUSR, show_vreg, set_vreg);
static DEVICE_ATTR(voltage_down1, S_IRUGO | S_IWUSR, show_vreg, set_vreg);
static DEVICE_ATTR(voltage_down2, S_IRUGO | S_IWUSR, show_vreg, set_vreg);
static DEVICE_ATTR(voltage_memldo, S_IRUGO | S_IWUSR, show_vreg, set_vreg);
static DEVICE_ATTR(voltage_ldo1, S_IRUGO | S_IWUSR, show_vreg, set_vreg);
static DEVICE_ATTR(voltage_ldo2, S_IRUGO | S_IWUSR, show_vreg, set_vreg);
static DEVICE_ATTR(voltage_ldo3, S_IRUGO | S_IWUSR, show_vreg, set_vreg);
static DEVICE_ATTR(voltage_ldo4, S_IRUGO | S_IWUSR, show_vreg, set_vreg);
static DEVICE_ATTR(voltage_ldo5, S_IRUGO | S_IWUSR, show_vreg, set_vreg);
static DEVICE_ATTR(voltage_ldo6, S_IRUGO | S_IWUSR, show_vreg, set_vreg);
static DEVICE_ATTR(voltage_hcldo, S_IRUGO | S_IWUSR, show_vreg, set_vreg);

/***********************************************************************
 * Charger Control
 ***********************************************************************/

/* Set maximum USB current limit */
static void pcf50633_usb_curlim_set(struct pcf50633_data *pcf, int ma)
{
	u_int8_t bits;
	int active = 0;

	pcf->last_curlim_set = ma;

	dev_dbg(&pcf->client.dev, "setting usb current limit to %d ma", ma);

	if (ma >= 1000) {
		bits = PCF50633_MBCC7_USB_1000mA;
	}
	else if (ma >= 500)
		bits = PCF50633_MBCC7_USB_500mA;
	else if (ma >= 100)
		bits = PCF50633_MBCC7_USB_100mA;
	else
		bits = PCF50633_MBCC7_USB_SUSPEND;

	/* set the nearest charging limit */
	reg_set_bit_mask(pcf, PCF50633_REG_MBCC7, PCF56033_MBCC7_USB_MASK,
			 bits);

	/* with this charging limit, is charging actually meaningful? */
	switch (bits) {
	case PCF50633_MBCC7_USB_500mA:
	case PCF50633_MBCC7_USB_1000mA:
		/* yes with this charging limit, we can do real charging */
		active = 1;
		break;
	default:
		/* no charging is gonna be happening */
		break;
	}
	/*
	 * enable or disable charging according to current limit -- this will
	 * also throw a platform notification callback about it
	 */
	pcf50633_charge_enable(pcf50633_global, active);

	/* clear batfull */
	reg_set_bit_mask(pcf, PCF50633_REG_MBCC1,
				PCF50633_MBCC1_AUTORES,
				0);
	reg_set_bit_mask(pcf, PCF50633_REG_MBCC1,
				PCF50633_MBCC1_RESUME,
				PCF50633_MBCC1_RESUME);
	reg_set_bit_mask(pcf, PCF50633_REG_MBCC1,
				PCF50633_MBCC1_AUTORES,
				PCF50633_MBCC1_AUTORES);

}

static ssize_t show_usblim(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);
	u_int8_t usblim = reg_read(pcf, PCF50633_REG_MBCC7) &
						PCF56033_MBCC7_USB_MASK;
	unsigned int ma;

	if (usblim == PCF50633_MBCC7_USB_1000mA)
		ma = 1000;
	else if (usblim == PCF50633_MBCC7_USB_500mA)
		ma = 500;
	else if (usblim == PCF50633_MBCC7_USB_100mA)
		ma = 100;
	else
		ma = 0;

	return sprintf(buf, "%u\n", ma);
}
static DEVICE_ATTR(usb_curlim, S_IRUGO | S_IWUSR, show_usblim, NULL);

/* Enable/disable charging */
static void pcf50633_charge_enable(struct pcf50633_data *pcf, int on)
{
	u_int8_t bits;
	u_int8_t usblim;

	if (!(pcf->pdata->used_features & PCF50633_FEAT_MBC))
		return;

	DEBUGPC("pcf50633_charge_enable %d\n", on);

	if (on) {
		pcf->flags |= PCF50633_F_CHG_ENABLED;
		bits = PCF50633_MBCC1_CHGENA;
		usblim = reg_read(pcf, PCF50633_REG_MBCC7) &
							PCF56033_MBCC7_USB_MASK;
		switch (usblim) {
		case PCF50633_MBCC7_USB_1000mA:
		case PCF50633_MBCC7_USB_500mA:
			if (pcf->flags & PCF50633_F_USB_PRESENT)
				if (pcf->pdata->cb)
					pcf->pdata->cb(&pcf->client.dev,
						       PCF50633_FEAT_MBC,
						       PMU_EVT_CHARGER_ACTIVE);
			break;
		default:
			break;
		}
	} else {
		pcf->flags &= ~PCF50633_F_CHG_ENABLED;
		bits = 0;
		if (pcf->pdata->cb)
			pcf->pdata->cb(&pcf->client.dev,
				       PCF50633_FEAT_MBC, PMU_EVT_CHARGER_IDLE);
	}
	reg_set_bit_mask(pcf, PCF50633_REG_MBCC1, PCF50633_MBCC1_CHGENA,
			 bits);
}

#if 0
#define ONE			1000000
static u_int16_t adc_to_rntc(struct pcf50633_data *pcf, u_int16_t adc)
{
	u_int32_t r_batt = (adc * pcf->pdata->r_fix_batt) / (1023 - adc);
	u_int16_t r_ntc;

	/* The battery NTC has a parallell 10kOhms resistor */
	r_ntc = ONE / ((ONE/r_batt) - (ONE/pcf->pdata->r_fix_batt_par));

	return r_ntc;
}
#endif
static ssize_t show_battemp(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "\n");
}
static DEVICE_ATTR(battemp, S_IRUGO | S_IWUSR, show_battemp, NULL);
#if 0
static u_int16_t adc_to_chg_milliamps(struct pcf50633_data *pcf,
					     u_int16_t adc_adcin1,
					     u_int16_t adc_batvolt)
{
	u_int32_t res = ((adc_adcin1 - adc_batvolt) * 6000);
	return res / (pcf->pdata->r_sense_milli * 1024 / 1000);
}
#endif
static ssize_t show_chgcur(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "\n");
}
static DEVICE_ATTR(chgcur, S_IRUGO | S_IWUSR, show_chgcur, NULL);

static const char *chgmode_names[] = {
	[PCF50633_MBCS2_MBC_PLAY]		= "play-only",
	[PCF50633_MBCS2_MBC_USB_PRE]		= "pre",
	[PCF50633_MBCS2_MBC_ADP_PRE]		= "pre",
	[PCF50633_MBCS2_MBC_USB_PRE_WAIT]	= "pre-wait",
	[PCF50633_MBCS2_MBC_ADP_PRE_WAIT]	= "pre-wait",
	[PCF50633_MBCS2_MBC_USB_FAST]		= "fast",
	[PCF50633_MBCS2_MBC_ADP_FAST]		= "fast",
	[PCF50633_MBCS2_MBC_USB_FAST_WAIT]	= "fast-wait",
	[PCF50633_MBCS2_MBC_ADP_FAST_WAIT]	= "fast-wait",
	[PCF50633_MBCS2_MBC_ADP_FAST_WAIT]	= "bat-full",
};

static ssize_t show_chgmode(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);
	u_int8_t mbcs2 = reg_read(pcf, PCF50633_REG_MBCS2);
	u_int8_t chgmod = (mbcs2 & PCF50633_MBCS2_MBC_MASK);

	return sprintf(buf, "%s\n", chgmode_names[chgmod]);
}

static ssize_t set_chgmode(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);

	/* As opposed to the PCF50606, we can only enable or disable
	 * charging and not directly jump into a certain mode! */

	if (!strcmp(buf, "0\n"))
		pcf50633_charge_enable(pcf, 0);
	else
		pcf50633_charge_enable(pcf, 1);

	return count;
}

static DEVICE_ATTR(chgmode, S_IRUGO | S_IWUSR, show_chgmode, set_chgmode);

static const char *chgstate_names[] = {
	[PCF50633_FIDX_CHG_ENABLED]		= "enabled",
	[PCF50633_FIDX_CHG_PRESENT] 		= "charger_present",
	[PCF50633_FIDX_USB_PRESENT] 		= "usb_present",
	[PCF50633_FIDX_CHG_ERR]			= "error",
	[PCF50633_FIDX_CHG_PROT]		= "protection",
	[PCF50633_FIDX_CHG_READY]		= "ready",
};

static ssize_t show_chgstate(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);

	char *b = buf;
	int i;

	for (i = 0; i < 32; i++)
		if (pcf->flags & (1 << i) && i < ARRAY_SIZE(chgstate_names))
			b += sprintf(b, "%s ", chgstate_names[i]);

	if (b > buf)
		b += sprintf(b, "\n");

	return b - buf;
}
static DEVICE_ATTR(chgstate, S_IRUGO | S_IWUSR, show_chgstate, NULL);

/***********************************************************************
 * APM emulation
 ***********************************************************************/

extern void (*apm_get_power_status)(struct apm_power_info *);

static void pcf50633_get_power_status(struct apm_power_info *info)
{
        struct pcf50633_data *pcf = pcf50633_global;
	u_int8_t chgmod = reg_read(pcf, PCF50633_REG_MBCS2) &
				   PCF50633_MBCS2_MBC_MASK;

	u_int16_t battvolt = pcf50633_battvolt(pcf);

	if (reg_read(pcf, PCF50633_REG_MBCS1) &
			(PCF50633_MBCS1_USBPRES|PCF50633_MBCS1_ADAPTPRES))
		info->ac_line_status = APM_AC_ONLINE;
	else
		info->ac_line_status = APM_AC_OFFLINE;

	switch (chgmod) {
	case PCF50633_MBCS2_MBC_PLAY:
	case PCF50633_MBCS2_MBC_USB_PRE:
	case PCF50633_MBCS2_MBC_USB_PRE_WAIT:
	case PCF50633_MBCS2_MBC_USB_FAST_WAIT:
	case PCF50633_MBCS2_MBC_ADP_PRE:
	case PCF50633_MBCS2_MBC_ADP_PRE_WAIT:
	case PCF50633_MBCS2_MBC_ADP_FAST_WAIT:
	case PCF50633_MBCS2_MBC_BAT_FULL:
	case PCF50633_MBCS2_MBC_HALT:
		info->battery_life = battvolt_scale(battvolt);
		break;
	case PCF50633_MBCS2_MBC_USB_FAST:
	case PCF50633_MBCS2_MBC_ADP_FAST:
		info->battery_status = APM_BATTERY_STATUS_CHARGING;
		info->battery_flag = APM_BATTERY_FLAG_CHARGING;
	default:
		break;
	}
}

/***********************************************************************
 * RTC
 ***********************************************************************/
enum pcf50633_time_indexes {
	PCF50633_TI_SEC = 0,
	PCF50633_TI_MIN,
	PCF50633_TI_HOUR,
	PCF50633_TI_WKDAY,
	PCF50633_TI_DAY,
	PCF50633_TI_MONTH,
	PCF50633_TI_YEAR,
	PCF50633_TI_EXTENT /* always last */
};


struct pcf50633_time {
	u_int8_t time[PCF50633_TI_EXTENT];
};

static void pcf2rtc_time(struct rtc_time *rtc, struct pcf50633_time *pcf)
{
	rtc->tm_sec = BCD2BIN(pcf->time[PCF50633_TI_SEC]);
	rtc->tm_min = BCD2BIN(pcf->time[PCF50633_TI_MIN]);
	rtc->tm_hour = BCD2BIN(pcf->time[PCF50633_TI_HOUR]);
	rtc->tm_wday = BCD2BIN(pcf->time[PCF50633_TI_WKDAY]);
	rtc->tm_mday = BCD2BIN(pcf->time[PCF50633_TI_DAY]);
	rtc->tm_mon = BCD2BIN(pcf->time[PCF50633_TI_MONTH]);
	rtc->tm_year = BCD2BIN(pcf->time[PCF50633_TI_YEAR]) + 100;
}

static void rtc2pcf_time(struct pcf50633_time *pcf, struct rtc_time *rtc)
{
	pcf->time[PCF50633_TI_SEC] = BIN2BCD(rtc->tm_sec);
	pcf->time[PCF50633_TI_MIN] = BIN2BCD(rtc->tm_min);
	pcf->time[PCF50633_TI_HOUR] = BIN2BCD(rtc->tm_hour);
	pcf->time[PCF50633_TI_WKDAY] = BIN2BCD(rtc->tm_wday);
	pcf->time[PCF50633_TI_DAY] = BIN2BCD(rtc->tm_mday);
	pcf->time[PCF50633_TI_MONTH] = BIN2BCD(rtc->tm_mon);
	pcf->time[PCF50633_TI_YEAR] = BIN2BCD(rtc->tm_year - 100);
}

static int pcf50633_rtc_ioctl(struct device *dev, unsigned int cmd,
			      unsigned long arg)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);

	switch (cmd) {
	case RTC_AIE_OFF:
		/* disable the alarm interrupt */
		reg_set_bit_mask(pcf, PCF50633_REG_INT1M,
				 PCF50633_INT1_ALARM, PCF50633_INT1_ALARM);
		return 0;
	case RTC_AIE_ON:
		/* enable the alarm interrupt */
		reg_clear_bits(pcf, PCF50633_REG_INT1M, PCF50633_INT1_ALARM);
		return 0;
	case RTC_PIE_OFF:
		/* disable periodic interrupt (hz tick) */
		pcf->flags &= ~PCF50633_F_RTC_SECOND;
		reg_set_bit_mask(pcf, PCF50633_REG_INT1M,
				 PCF50633_INT1_SECOND, PCF50633_INT1_SECOND);
		return 0;
	case RTC_PIE_ON:
		/* ensable periodic interrupt (hz tick) */
		pcf->flags |= PCF50633_F_RTC_SECOND;
		reg_clear_bits(pcf, PCF50633_REG_INT1M, PCF50633_INT1_SECOND);
		return 0;
	}
	return -ENOIOCTLCMD;
}

static int pcf50633_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);
	struct pcf50633_time pcf_tm;
	int ret;

	mutex_lock(&pcf->lock);

	ret = i2c_smbus_read_i2c_block_data(&pcf->client,
					    PCF50633_REG_RTCSC,
					    PCF50633_TI_EXTENT,
					    &pcf_tm.time[0]);
	if (ret != PCF50633_TI_EXTENT)
		dev_err(dev, "Failed to read time :-(\n");

	mutex_unlock(&pcf->lock);

	dev_dbg(dev, "PCF_TIME: %02x.%02x.%02x %02x:%02x:%02x\n",
		pcf_tm.time[PCF50633_TI_DAY],
		pcf_tm.time[PCF50633_TI_MONTH],
		pcf_tm.time[PCF50633_TI_YEAR],
		pcf_tm.time[PCF50633_TI_HOUR],
		pcf_tm.time[PCF50633_TI_MIN],
		pcf_tm.time[PCF50633_TI_SEC]);

	pcf2rtc_time(tm, &pcf_tm);

	dev_dbg(dev, "RTC_TIME: %u.%u.%u %u:%u:%u\n",
		tm->tm_mday, tm->tm_mon, tm->tm_year,
		tm->tm_hour, tm->tm_min, tm->tm_sec);

	return 0;
}

static int pcf50633_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);
	struct pcf50633_time pcf_tm;
	int ret;

	dev_dbg(dev, "RTC_TIME: %u.%u.%u %u:%u:%u\n",
		tm->tm_mday, tm->tm_mon, tm->tm_year,
		tm->tm_hour, tm->tm_min, tm->tm_sec);
	rtc2pcf_time(&pcf_tm, tm);
	dev_dbg(dev, "PCF_TIME: %02x.%02x.%02x %02x:%02x:%02x\n",
		pcf_tm.time[PCF50633_TI_DAY],
		pcf_tm.time[PCF50633_TI_MONTH],
		pcf_tm.time[PCF50633_TI_YEAR],
		pcf_tm.time[PCF50633_TI_HOUR],
		pcf_tm.time[PCF50633_TI_MIN],
		pcf_tm.time[PCF50633_TI_SEC]);

	mutex_lock(&pcf->lock);
	/* FIXME: disable second interrupt */

	ret = i2c_smbus_write_i2c_block_data(&pcf->client,
					     PCF50633_REG_RTCSC,
					     PCF50633_TI_EXTENT,
					     &pcf_tm.time[0]);
	if (ret)
		dev_err(dev, "Failed to set time %d\n", ret);

	/* FIXME: re-enable second interrupt */
	mutex_unlock(&pcf->lock);

	return 0;
}

static int pcf50633_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);
	struct pcf50633_time pcf_tm;
	int ret;

	mutex_lock(&pcf->lock);

	alrm->enabled =
	     __reg_read(pcf, PCF50633_REG_INT1M) & PCF50633_INT1_ALARM ? 0 : 1;

	ret = i2c_smbus_read_i2c_block_data(&pcf->client,
					    PCF50633_REG_RTCSCA,
					    PCF50633_TI_EXTENT,
					    &pcf_tm.time[0]);
	if (ret != PCF50633_TI_EXTENT)
		dev_err(dev, "Failed to read Alarm time :-(\n");

	mutex_unlock(&pcf->lock);

	pcf2rtc_time(&alrm->time, &pcf_tm);

	return 0;
}

static int pcf50633_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);
	struct pcf50633_time pcf_tm;
	u_int8_t irqmask;
	int ret;

	rtc2pcf_time(&pcf_tm, &alrm->time);

	mutex_lock(&pcf->lock);

	/* disable alarm interrupt */
	irqmask = __reg_read(pcf, PCF50633_REG_INT1M);
	irqmask |= PCF50633_INT1_ALARM;
	__reg_write(pcf, PCF50633_REG_INT1M, irqmask);

	ret = i2c_smbus_write_i2c_block_data(&pcf->client,
					     PCF50633_REG_RTCSCA,
					     PCF50633_TI_EXTENT,
					     &pcf_tm.time[0]);
	if (ret)
		dev_err(dev, "Failed to write alarm time :-( %d\n", ret);

	if (alrm->enabled) {
		/* (re-)enaable alarm interrupt */
		irqmask = __reg_read(pcf, PCF50633_REG_INT1M);
		irqmask &= ~PCF50633_INT1_ALARM;
		__reg_write(pcf, PCF50633_REG_INT1M, irqmask);
	}

	mutex_unlock(&pcf->lock);

	/* FIXME */
	return 0;
}

static struct rtc_class_ops pcf50633_rtc_ops = {
	.ioctl		= pcf50633_rtc_ioctl,
	.read_time	= pcf50633_rtc_read_time,
	.set_time	= pcf50633_rtc_set_time,
	.read_alarm	= pcf50633_rtc_read_alarm,
	.set_alarm	= pcf50633_rtc_set_alarm,
};

/***********************************************************************
 * Backlight device
 ***********************************************************************/

static int pcf50633bl_get_intensity(struct backlight_device *bd)
{
	struct pcf50633_data *pcf = bl_get_data(bd);
	int intensity = reg_read(pcf, PCF50633_REG_LEDOUT);

	if (!(reg_read(pcf, PCF50633_REG_LEDENA) & 1))
		intensity = 0;

	return intensity & 0x3f;
}

static int __pcf50633bl_set_intensity(struct pcf50633_data *pcf, int intensity)
{
	int old_intensity = reg_read(pcf, PCF50633_REG_LEDOUT);
	int ret;

	if (!(reg_read(pcf, PCF50633_REG_LEDENA) & 1))
		old_intensity = 0;

	/*
	 * The PCF50633 cannot handle LEDOUT = 0 (datasheet p60)
	 * if seen, you have to re-enable the LED unit
	 * we treat intensity 0 as disable
	 */

	if (intensity && !old_intensity) {
		ret = reg_set_bit_mask(pcf, PCF50633_REG_LEDENA, 0x01, 0x00);
		if (ret)
			return ret;
	}

	if (!intensity) /* illegal to set LEDOUT to 0 */
		ret = reg_set_bit_mask(pcf, PCF50633_REG_LEDENA, 0x01, 0x00);
	else {
		ret = reg_set_bit_mask(pcf, PCF50633_REG_LEDOUT, 0x3f,
			       intensity);
		if (ret)
			return ret;
		ret = reg_set_bit_mask(pcf, PCF50633_REG_LEDENA, 0x01, 0x01);
	}

	return ret;
}

static int pcf50633bl_set_intensity(struct backlight_device *bd)
{
	struct pcf50633_data *pcf = bl_get_data(bd);
	int intensity = bd->props.brightness;

	if ((bd->props.power != FB_BLANK_UNBLANK) ||
	    (bd->props.fb_blank != FB_BLANK_UNBLANK))
		intensity = 0;

	return __pcf50633bl_set_intensity(pcf, intensity);
}

static struct backlight_ops pcf50633bl_ops = {
	.get_brightness	= pcf50633bl_get_intensity,
	.update_status	= pcf50633bl_set_intensity,
};

/*
 * Charger type
 */

static ssize_t show_charger_type(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);
	static const char *names_charger_type[] = {
		[CHARGER_TYPE_NONE] 	= "none",
		[CHARGER_TYPE_HOSTUSB] 	= "host/500mA usb",
		[CHARGER_TYPE_1A] 	= "charger 1A",
	};
	static const char *names_charger_modes[] = {
		[PCF50633_MBCC7_USB_1000mA] 	= "1A",
		[PCF50633_MBCC7_USB_500mA] 	= "500mA",
		[PCF50633_MBCC7_USB_100mA] 	= "100mA",
		[PCF50633_MBCC7_USB_SUSPEND] 	= "suspend",
	};
	int mode = reg_read(pcf, PCF50633_REG_MBCC7) & PCF56033_MBCC7_USB_MASK;

	return sprintf(buf, "%s mode %s\n",
			    names_charger_type[pcf->charger_type],
			    names_charger_modes[mode]);
}

static DEVICE_ATTR(charger_type, 0444, show_charger_type, NULL);

static ssize_t force_usb_limit_dangerous(struct device *dev,
		   struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);
	int ma = simple_strtoul(buf, NULL, 10);

	pcf50633_usb_curlim_set(pcf, ma);
	return count;
}

static DEVICE_ATTR(force_usb_limit_dangerous, 0600,
					       NULL, force_usb_limit_dangerous);

/*
 * Charger adc
 */

static ssize_t show_charger_adc(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", pcf->charger_adc_result_raw);
}

static DEVICE_ATTR(charger_adc, 0444, show_charger_adc, NULL);

/*
 * Dump regs
 */

static ssize_t show_dump_regs(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);
	u8 dump[16];
	int n, n1, idx = 0;
	char *buf1 = buf;
	static u8 address_no_read[] = { /* must be ascending */
		PCF50633_REG_INT1,
		PCF50633_REG_INT2,
		PCF50633_REG_INT3,
		PCF50633_REG_INT4,
		PCF50633_REG_INT5,
		0 /* terminator */
	};

	for (n = 0; n < 256; n += sizeof(dump)) {

		for (n1 = 0; n1 < sizeof(dump); n1++)
			if (n == address_no_read[idx]) {
				idx++;
				dump[n1] = 0x00;
			} else
				dump[n1] = reg_read(pcf, n + n1);

		hex_dump_to_buffer(dump, sizeof(dump), 16, 1, buf1, 128, 0);
		buf1 += strlen(buf1);
		*buf1++ = '\n';
		*buf1 = '\0';
	}

	return buf1 - buf;
}

static DEVICE_ATTR(dump_regs, 0400, show_dump_regs, NULL);


/***********************************************************************
 * Driver initialization
 ***********************************************************************/

/*
 * CARE!  This table is modified at runtime!
 */
static struct attribute *pcf_sysfs_entries[] = {
	&dev_attr_voltage_auto.attr,
	&dev_attr_voltage_down1.attr,
	&dev_attr_voltage_down2.attr,
	&dev_attr_voltage_memldo.attr,
	&dev_attr_voltage_ldo1.attr,
	&dev_attr_voltage_ldo2.attr,
	&dev_attr_voltage_ldo3.attr,
	&dev_attr_voltage_ldo4.attr,
	&dev_attr_voltage_ldo5.attr,
	&dev_attr_voltage_ldo6.attr,
	&dev_attr_voltage_hcldo.attr,
	&dev_attr_charger_type.attr,
	&dev_attr_force_usb_limit_dangerous.attr,
	&dev_attr_charger_adc.attr,
	&dev_attr_dump_regs.attr,
	NULL, /* going to add things at this point! */
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
};

static struct attribute_group pcf_attr_group = {
	.name	= NULL,			/* put in device directory */
	.attrs	= pcf_sysfs_entries,
};

static void populate_sysfs_group(struct pcf50633_data *pcf)
{
	int i = 0;
	struct attribute **attr;

	for (attr = pcf_sysfs_entries; *attr; attr++)
		i++;

	if (pcf->pdata->used_features & PCF50633_FEAT_MBC) {
		pcf_sysfs_entries[i++] = &dev_attr_chgstate.attr;
		pcf_sysfs_entries[i++] = &dev_attr_chgmode.attr;
		pcf_sysfs_entries[i++] = &dev_attr_usb_curlim.attr;
	}

	if (pcf->pdata->used_features & PCF50633_FEAT_CHGCUR)
		pcf_sysfs_entries[i++] = &dev_attr_chgcur.attr;

	if (pcf->pdata->used_features & PCF50633_FEAT_BATVOLT)
		pcf_sysfs_entries[i++] = &dev_attr_battvolt.attr;

	if (pcf->pdata->used_features & PCF50633_FEAT_BATTEMP)
		pcf_sysfs_entries[i++] = &dev_attr_battemp.attr;

}

static int pcf50633_detect(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *new_client;
	struct pcf50633_data *pcf;
	int err = 0;
	int irq;

	DEBUGP("entering\n");
	if (!pcf50633_pdev) {
		printk(KERN_ERR "pcf50633: driver needs a platform_device!\n");
		return -EIO;
	}

	irq = platform_get_irq(pcf50633_pdev, 0);
	if (irq < 0) {
		dev_err(&pcf50633_pdev->dev, "no irq in platform resources!\n");
		return -EIO;
	}

	/* At the moment, we only support one PCF50633 in a system */
	if (pcf50633_global) {
		dev_err(&pcf50633_pdev->dev,
			"currently only one chip supported\n");
		return -EBUSY;
	}

	pcf = kzalloc(sizeof(*pcf), GFP_KERNEL);
	if (!pcf)
		return -ENOMEM;

	mutex_init(&pcf->lock);
	mutex_init(&pcf->working_lock);
	mutex_init(&pcf->working_lock_nobat);
	mutex_init(&pcf->working_lock_usb_curlimit);
	INIT_WORK(&pcf->work, pcf50633_work);
	INIT_WORK(&pcf->work_nobat, pcf50633_work_nobat);
	INIT_WORK(&pcf->work_usb_curlimit, pcf50633_work_usbcurlim);
	pcf->irq = irq;
	pcf->working = 0;
	pcf->onkey_seconds = -1;
	pcf->pdata = pcf50633_pdev->dev.platform_data;

	new_client = &pcf->client;
	i2c_set_clientdata(new_client, pcf);
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &pcf50633_driver;
	new_client->flags = 0;
	strlcpy(new_client->name, "pcf50633", I2C_NAME_SIZE);

	/* now we try to detect the chip */

	/* register with i2c core */
	if ((err = i2c_attach_client(new_client))) {
		dev_err(&new_client->dev,
			"error during i2c_attach_client()\n");
		goto exit_free;
	}

	init_resume_dependency_list(&pcf->resume_dependency);

	populate_sysfs_group(pcf);

	err = sysfs_create_group(&new_client->dev.kobj, &pcf_attr_group);
	if (err) {
		dev_err(&new_client->dev, "error creating sysfs group\n");
		goto exit_detach;
	}

	/* create virtual charger 'device' */

	/* register power off handler with core power management */
	pm_power_off = &pcf50633_go_standby;

	pcf->input_dev = input_allocate_device();
	if (!pcf->input_dev)
		goto exit_sysfs;

	pcf->input_dev->name = "GTA02 PMU events";
	pcf->input_dev->phys = "FIXME";
	pcf->input_dev->id.bustype = BUS_I2C;
	pcf->input_dev->cdev.dev = &new_client->dev;

	pcf->input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_PWR);
	set_bit(KEY_POWER, pcf->input_dev->keybit);
	set_bit(KEY_POWER2, pcf->input_dev->keybit);
	set_bit(KEY_BATTERY, pcf->input_dev->keybit);

	err = input_register_device(pcf->input_dev);
	if (err)
		goto exit_sysfs;

	/* configure interrupt mask */
	/* we want SECOND to kick for the coldplug initialisation */
	reg_write(pcf, PCF50633_REG_INT1M, 0x00);
	reg_write(pcf, PCF50633_REG_INT2M, 0x00);
	reg_write(pcf, PCF50633_REG_INT3M, 0x00);
	reg_write(pcf, PCF50633_REG_INT4M, 0x00);
	reg_write(pcf, PCF50633_REG_INT5M, 0x00);

	err = request_irq(irq, pcf50633_irq, IRQF_TRIGGER_FALLING,
			  "pcf50633", pcf);
	if (err < 0)
		goto exit_input;

	if (enable_irq_wake(irq) < 0)
		dev_err(&new_client->dev, "IRQ %u cannot be enabled as wake-up"
		        "source in this hardware revision!", irq);

	if (pcf->pdata->used_features & PCF50633_FEAT_RTC) {
		pcf->rtc = rtc_device_register("pcf50633", &new_client->dev,
						&pcf50633_rtc_ops, THIS_MODULE);
		if (IS_ERR(pcf->rtc)) {
			err = PTR_ERR(pcf->rtc);
			goto exit_irq;
		}
	}

	if (pcf->pdata->used_features & PCF50633_FEAT_PWM_BL) {
		pcf->backlight = backlight_device_register("pcf50633-bl",
							    &new_client->dev,
							    pcf,
							    &pcf50633bl_ops);
		if (!pcf->backlight)
			goto exit_rtc;
		/* FIXME: are we sure we want default == off? */
		pcf->backlight->props.max_brightness = 0x3f;
		pcf->backlight->props.power = FB_BLANK_UNBLANK;
		pcf->backlight->props.fb_blank = FB_BLANK_UNBLANK;
		pcf->backlight->props.brightness =
					pcf->backlight->props.max_brightness;
		backlight_update_status(pcf->backlight);
	}

	if (pcf->pdata->flag_use_apm_emulation)
		apm_get_power_status = pcf50633_get_power_status;

	pcf->probe_completed = 1;
	pcf50633_global = pcf;
	dev_info(&new_client->dev, "probe completed\n");

	/* if platform was interested, give him a chance to register
	 * platform devices that switch power with us as the parent
	 * at registration time -- ensures suspend / resume ordering
	 */
	if (pcf->pdata->attach_child_devices)
		(pcf->pdata->attach_child_devices)(&new_client->dev);

	return 0;
exit_rtc:
	if (pcf->pdata->used_features & PCF50633_FEAT_RTC)
		rtc_device_unregister(pcf50633_global->rtc);
exit_irq:
	free_irq(pcf50633_global->irq, pcf50633_global);
exit_input:
	input_unregister_device(pcf->input_dev);
exit_sysfs:
	pm_power_off = NULL;
	sysfs_remove_group(&new_client->dev.kobj, &pcf_attr_group);
exit_detach:
	i2c_detach_client(new_client);
exit_free:
	kfree(pcf);
	pcf50633_global = NULL;
	return err;
}

static int pcf50633_attach_adapter(struct i2c_adapter *adapter)
{
	DEBUGP("entering, calling i2c_probe\n");
	return i2c_probe(adapter, &addr_data, &pcf50633_detect);
}

static int pcf50633_detach_client(struct i2c_client *client)
{
	struct pcf50633_data *pcf = i2c_get_clientdata(client);

	DEBUGP("entering\n");

	apm_get_power_status = NULL;

	free_irq(pcf->irq, pcf);

	input_unregister_device(pcf->input_dev);

	if (pcf->pdata->used_features & PCF50633_FEAT_PWM_BL)
		backlight_device_unregister(pcf->backlight);

	if (pcf->pdata->used_features & PCF50633_FEAT_RTC)
		rtc_device_unregister(pcf->rtc);

	sysfs_remove_group(&client->dev.kobj, &pcf_attr_group);

	pm_power_off = NULL;

	kfree(pcf);

	return 0;
}

/* you're going to need >300 bytes in buf */

int pcf50633_report_resumers(struct pcf50633_data *pcf, char *buf)
{
	static char *int_names[] = {
		"adpins",
		"adprem",
		"usbins",
		"usbrem",
		NULL,
		NULL,
		"rtcalarm",
		"second",

		"onkeyr",
		"onkeyf",
		"exton1r",
		"exton1f",
		"exton2r",
		"exton2f",
		"exton3r",
		"exton3f",

		"batfull",
		"chghalt",
		"thlimon",
		"thlimoff",
		"usblimon",
		"usblimoff",
		"adcrdy",
		"onkey1s",

		"lowsys",
		"lowbat",
		"hightmp",
		"autopwrfail",
		"dwn1pwrfail",
		"dwn2pwrfail",
		"ledpwrfail",
		"ledovp",

		"ldo1pwrfail",
		"ldo2pwrfail",
		"ldo3pwrfail",
		"ldo4pwrfail",
		"ldo5pwrfail",
		"ldo6pwrfail",
		"hcidopwrfail",
		"hcidoovl"
	};
	char *end = buf;
	int n;

	for (n = 0; n < ARRAY_SIZE(int_names); n++)
		if (int_names[n]) {
			if (pcf->pcfirq_resume[n >> 3] & (1 >> (n & 7)))
				end += sprintf(end, "  * %s\n", int_names[n]);
			else
				end += sprintf(end, "    %s\n", int_names[n]);
		}
	
	return end - buf;
}


#ifdef CONFIG_PM

/*
 * we need to export this because pcf50633_data is kept opaque
 */

void pcf50633_register_resume_dependency(struct pcf50633_data *pcf,
					struct resume_dependency *dep)
{
	register_resume_dependency(&pcf->resume_dependency, dep);
	if (pcf->is_suspended)
		activate_all_resume_dependencies(&pcf->resume_dependency);
}
EXPORT_SYMBOL_GPL(pcf50633_register_resume_dependency);


static int pcf50633_suspend(struct device *dev, pm_message_t state)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);
	int i;
	int ret;
	u_int8_t tmp;
	u_int8_t res[5];

	dev_err(dev, "pcf50633_suspend\n");

	/* we suspend once (!) as late as possible in the suspend sequencing */

	if ((state.event != PM_EVENT_SUSPEND) ||
	    (pcf->suspend_state != PCF50633_SS_RUNNING))
		return -EBUSY;

	/* The general idea is to power down all unused power supplies,
	 * and then mask all PCF50633 interrupt sources but EXTONR, ONKEYF
	 * and ALARM */

	mutex_lock(&pcf->lock);

	pcf->suspend_state = PCF50633_SS_STARTING_SUSPEND;

	/* we are not going to service any further interrupts until we
	 * resume.  If the IRQ workqueue is still pending in the background,
	 * it will bail when it sees we set suspend state above
	 */

	disable_irq(pcf->irq);

	/* Save all registers that don't "survive" standby state */
	pcf->standby_regs.ooctim2 = __reg_read(pcf, PCF50633_REG_OOCTIM2);

	ret = i2c_smbus_read_i2c_block_data(&pcf->client,
					    PCF50633_REG_AUTOOUT,
					    sizeof(pcf->standby_regs.misc),
					    &pcf->standby_regs.misc[0]);
	if (ret != sizeof(pcf->standby_regs.misc))
		dev_err(dev, "Failed to save misc levels and enables :-(\n");

	/* regulator voltages and enable states */
	ret = i2c_smbus_read_i2c_block_data(&pcf->client,
					    PCF50633_REG_LDO1OUT,
					    sizeof(pcf->standby_regs.ldo),
					    &pcf->standby_regs.ldo[0]);
	if (ret != sizeof(pcf->standby_regs.ldo))
		dev_err(dev, "Failed to save LDO levels and enables :-(\n");

	/* switch off power supplies that are not needed during suspend */
	for (i = 0; i < __NUM_PCF50633_REGULATORS; i++) {
		if ((pcf->pdata->rails[i].flags & PMU_VRAIL_F_SUSPEND_ON))
			continue;

		/* we can save ourselves the read part of a read-modify-write
		 * here because we captured all these already
		 */
		if (i < 4)
			tmp = pcf->standby_regs.misc[i * 4 + 1];
		else
			tmp = pcf->standby_regs.ldo[(i - 4) * 2 + 1];

		dev_dbg(dev, "disabling reg %s by setting ENA %d to 0x%02X\n",
			      pcf->pdata->rails[i].name,
			      regulator_registers[i] + 1, tmp & 0xfe);

		/* associated enable is always +1 from OUT reg */
		__reg_write(pcf, regulator_registers[i] + 1, tmp & 0xfe);
	}

	/* turn off the backlight */
	__reg_write(pcf, PCF50633_REG_LEDDIM, 0);
	__reg_write(pcf, PCF50633_REG_LEDOUT, 2);
	__reg_write(pcf, PCF50633_REG_LEDENA, 0x00);

	/* set interrupt masks so only those sources we want to wake
	 * us are able to
	 */
	for (i = 0; i < 5; i++)
		res[i] = ~pcf->pdata->resumers[i];

	ret = i2c_smbus_write_i2c_block_data(&pcf->client,
					     PCF50633_REG_INT1M,
					     5, &res[0]);
	if (ret)
		dev_err(dev, "Failed to set wake masks :-( %d\n", ret);

	pcf->suspend_state = PCF50633_SS_COMPLETED_SUSPEND;

	mutex_unlock(&pcf->lock);

	pcf->is_suspended = 1;
	activate_all_resume_dependencies(&pcf->resume_dependency);
	return 0;
}


int pcf50633_ready(struct pcf50633_data *pcf)
{
	if (!pcf)
		return -EACCES;

	/* this was seen during boot with Qi, mmc_rescan racing us */
	if (!pcf->probe_completed)
		return -EACCES;

	if ((pcf->suspend_state != PCF50633_SS_RUNNING) &&
	    (pcf->suspend_state < PCF50633_SS_COMPLETED_RESUME))
		return -EBUSY;

	return 0;
}
EXPORT_SYMBOL_GPL(pcf50633_ready);

int pcf50633_wait_for_ready(struct pcf50633_data *pcf, int timeout_ms,
								char *name)
{
	/* so we always go once */
	timeout_ms += 5;

	while ((timeout_ms >= 5) && (pcf50633_ready(pcf))) {
		timeout_ms -= 5; /* well, it isn't very accurate, but OK */
		msleep(5);
	}

	if (timeout_ms < 5) {
		printk(KERN_ERR"pcf50633_wait_for_ready: "
					"%s BAILING on timeout\n", name);
		return -EBUSY;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(pcf50633_wait_for_ready);

/*
 * if backlight resume is selected to be deferred by platform, then it
 * can call this to finally reset backlight status (after LCM is resumed
 * for example
 */

void pcf50633_backlight_resume(struct pcf50633_data *pcf)
{
	dev_dbg(&pcf->client.dev, "pcf50633_backlight_resume\n");

	/* platform defines resume ramp speed */
	reg_write(pcf, PCF50633_REG_LEDDIM,
				       pcf->pdata->resume_backlight_ramp_speed);

	__pcf50633bl_set_intensity(pcf, pcf->backlight->props.brightness);
}
EXPORT_SYMBOL_GPL(pcf50633_backlight_resume);


static int pcf50633_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);
	int ret;
	u8 res[5];
	u8 misc[PCF50633_REG_LEDDIM - PCF50633_REG_AUTOOUT + 1];

	dev_dbg(dev, "pcf50633_resume suspended on entry = %d\n",
						 (int)pcf->suspend_state);
	mutex_lock(&pcf->lock);

	pcf->suspend_state = PCF50633_SS_STARTING_RESUME;

	/* these guys get reset while pcf50633 is suspend state, refresh */

	__reg_write(pcf, PCF50633_REG_OOCTIM2, pcf->standby_regs.ooctim2);

	memcpy(misc, pcf->standby_regs.misc, sizeof(pcf->standby_regs.misc));

	if (pcf->pdata->defer_resume_backlight) {
		misc[PCF50633_REG_LEDOUT - PCF50633_REG_AUTOOUT] = 1;
		misc[PCF50633_REG_LEDENA - PCF50633_REG_AUTOOUT] = 0x20;
		misc[PCF50633_REG_LEDCTL - PCF50633_REG_AUTOOUT] = 1;
		misc[PCF50633_REG_LEDDIM - PCF50633_REG_AUTOOUT] = 1;
	}

	/* regulator voltages and enable states */
	ret = i2c_smbus_write_i2c_block_data(&pcf->client,
					     PCF50633_REG_AUTOOUT,
					     sizeof(misc),
					     &misc[0]);
	if (ret)
		dev_err(dev, "Failed to restore misc :-( %d\n", ret);

	/* platform can choose to defer backlight bringup */
	if (!pcf->pdata->defer_resume_backlight)
		pcf50633_backlight_resume(pcf);

	/* regulator voltages and enable states */
	ret = i2c_smbus_write_i2c_block_data(&pcf->client,
					     PCF50633_REG_LDO1OUT,
					     sizeof(pcf->standby_regs.ldo),
					     &pcf->standby_regs.ldo[0]);
	if (ret)
		dev_err(dev, "Failed to restore LDOs :-( %d\n", ret);

	memset(res, 0, sizeof(res));
	/* not interested in second on resume */
	res[0] = PCF50633_INT1_SECOND;
	ret = i2c_smbus_write_i2c_block_data(&pcf->client,
					     PCF50633_REG_INT1M,
					     5, &res[0]);
	if (ret)
		dev_err(dev, "Failed to set int masks :-( %d\n", ret);

	pcf->suspend_state = PCF50633_SS_COMPLETED_RESUME;

	enable_irq(pcf->irq);

	mutex_unlock(&pcf->lock);

	/* gratuitous call to PCF work function, in the case that the PCF
	 * interrupt edge was missed during resume, this forces the pending
	 * register clear and lifts the interrupt back high again.  In the
	 * case nothing is waiting for service, no harm done.
	 */

	get_device(&pcf->client.dev);
	pcf50633_work(&pcf->work);

	pcf->is_suspended = 0;
	callback_all_resume_dependencies(&pcf->resume_dependency);

	return 0;
}
#else
#define pcf50633_suspend NULL
#define pcf50633_resume NULL
#endif

static struct i2c_driver pcf50633_driver = {
	.driver = {
		.name	= "pcf50633",
		.suspend= pcf50633_suspend,
		.resume	= pcf50633_resume,
	},
	.id		= I2C_DRIVERID_PCF50633,
	.attach_adapter	= pcf50633_attach_adapter,
	.detach_client	= pcf50633_detach_client,
};

/* we have this purely to capture an early indication that we are coming out
 * of suspend, before our device resume got called; async interrupt service is
 * interested in this
 */

static int pcf50633_plat_resume(struct platform_device *pdev)
{
	/* i2c_get_clientdata(to_i2c_client(&pdev->dev)) returns NULL at this
	 * early resume time so we have to use pcf50633_global
	 */
	pcf50633_global->suspend_state = PCF50633_SS_RESUMING_BUT_NOT_US_YET;

	return 0;
}

/* platform driver, since i2c devices don't have platform_data */
static int __init pcf50633_plat_probe(struct platform_device *pdev)
{
	struct pcf50633_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata)
		return -ENODEV;

	pcf50633_pdev = pdev;

	return 0;
}

static int pcf50633_plat_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver pcf50633_plat_driver = {
	.probe	= pcf50633_plat_probe,
	.remove	= pcf50633_plat_remove,
	.resume_early = pcf50633_plat_resume,
	.driver = {
		.owner	= THIS_MODULE,
		.name 	= "pcf50633",
	},
};

static int __init pcf50633_init(void)
{
	int rc;

	if (!(rc = platform_driver_register(&pcf50633_plat_driver)))
		rc = i2c_add_driver(&pcf50633_driver);

	return rc;
}

static void pcf50633_exit(void)
{
	i2c_del_driver(&pcf50633_driver);
	platform_driver_unregister(&pcf50633_plat_driver);
}

MODULE_DESCRIPTION("I2C chip driver for NXP PCF50633 power management unit");
MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_LICENSE("GPL");

module_init(pcf50633_init);
module_exit(pcf50633_exit);
