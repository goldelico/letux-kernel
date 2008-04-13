/* Philips PCF50633 Power Management Unit (PMU) driver
 *
 * (C) 2006-2007 by OpenMoko, Inc.
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

#include <asm/mach-types.h>
#include <asm/arch/gta02.h>

#include "pcf50633.h"

#if 1
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
		u_int8_t int1m, int2m, int3m, int4m, int5m;
		u_int8_t ooctim2;
		u_int8_t autoout, autoena, automxc;
		u_int8_t down1out, down1mxc;
		u_int8_t down2out, down2ena;
		u_int8_t memldoout, memldoena;
		u_int8_t ledout, ledena, leddim;
		struct {
			u_int8_t out;
			u_int8_t ena;
		} ldo[__NUM_PCF50633_REGS];
	} standby_regs;
#endif
};

static struct i2c_driver pcf50633_driver;

struct pcf50633_data *pcf50633_global;
EXPORT_SYMBOL_GPL(pcf50633_global);

static struct platform_device *pcf50633_pdev;

/***********************************************************************
 * Low-Level routines
 ***********************************************************************/

static int __reg_write(struct pcf50633_data *pcf, u_int8_t reg, u_int8_t val)
{
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
		__reg_write(pcf, PCF50633_REG_MBCC7,
						    PCF50633_MBCC7_USB_SUSPEND);
		break;
	/*
	 * the PCF50633 has a feature that it will supply only excess current
	 * from the charger that is not used to power the device.  So this
	 * 500mA setting is "up to 500mA" according to that.
	 */
	case CHARGER_TYPE_HOSTUSB:
		__reg_write(pcf, PCF50633_REG_MBCC7, PCF50633_MBCC7_USB_500mA);
		break;
	case CHARGER_TYPE_1A:
		__reg_write(pcf, PCF50633_REG_MBCC7, PCF50633_MBCC7_USB_1000mA);
		break;
	}
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

static void pcf50633_work(struct work_struct *work)
{
	struct pcf50633_data *pcf =
			container_of(work, struct pcf50633_data, work);
	u_int8_t pcfirq[5];
	int ret;
	int tail;

	mutex_lock(&pcf->working_lock);
	pcf->working = 1;
	/*
	 * datasheet says we have to read the five IRQ
	 * status regs in one transaction
	 */
	ret = i2c_smbus_read_i2c_block_data(&pcf->client, PCF50633_REG_INT1, 5,
					    pcfirq);
	if (ret != 5) {
		DEBUGP("Oh crap PMU IRQ register read failed -- "
		       "retrying later %d\n", ret);
		/*
		 * this situation can happen during resume, just defer
		 * handling the interrupt until enough I2C is up we can
		 * actually talk to the PMU.  We can't just ignore this
		 * because we are on a falling edge interrupt and our
		 * PMU interrupt source does not clear until we read these
		 * interrupt source registers.
		 */
		if (!schedule_work(&pcf->work) && !pcf->working)
			dev_dbg(&pcf->client.dev, "work item may be lost\n");

		/* we don't put the device here, hold it for next time */
		mutex_unlock(&pcf->working_lock);
		/* don't spew, delaying whatever else is happening */
		msleep(1);
		return;
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
		/* FIXME: signal this to userspace */
		//kobject_uevent( ,KOBJ_ADD);
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
		/* FIXME: signal this to userspace */
		//kobject_uevent( ,KOBJ_ADD);
	}
	if (pcfirq[0] & PCF50633_INT1_USBINS) {
		DEBUGPC("USBINS ");
		input_report_key(pcf->input_dev, KEY_POWER2, 1);
		apm_queue_event(APM_POWER_STATUS_CHANGE);
		pcf->flags |= PCF50633_F_USB_PRESENT;
		if (pcf->pdata->cb)
			pcf->pdata->cb(&pcf->client.dev,
				       PCF50633_FEAT_MBC, PMU_EVT_USB_INSERT);
		/* completion irq will figure out our charging stance */
		add_request_to_adc_queue(pcf, PCF50633_ADCC1_MUX_ADCIN1,
				     PCF50633_ADCC1_AVERAGE_16);
	}
	if (pcfirq[0] & PCF50633_INT1_USBREM) {
		DEBUGPC("USBREM ");
		/* only deal if we had understood it was in */
		if (pcf->flags & PCF50633_F_USB_PRESENT) {
			input_report_key(pcf->input_dev, KEY_POWER2, 0);
			apm_queue_event(APM_POWER_STATUS_CHANGE);
			pcf->flags &= ~PCF50633_F_USB_PRESENT;
			if (pcf->pdata->cb)
				pcf->pdata->cb(&pcf->client.dev,
					PCF50633_FEAT_MBC, PMU_EVT_USB_REMOVE);
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
		DEBUGPC("SECOND ");
		if (pcf->flags & PCF50633_F_RTC_SECOND)
			rtc_update_irq(pcf->rtc, 1, RTC_PF | RTC_IRQF);

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
		/* FIXME: signal this to userspace */
	}
	if (pcfirq[2] & PCF50633_INT3_CHGHALT) {
		DEBUGPC("CHGHALT ");
		/*
		 * this is really "battery not pulling current" -- it can
		 * appear with no battery attached
		 */
		/* FIXME: signal this to userspace */
	}
	if (pcfirq[2] & PCF50633_INT3_THLIMON) {
		DEBUGPC("THLIMON ");
		pcf->flags |= PCF50633_F_CHG_PROT;
		/* FIXME: signal this to userspace */
	}
	if (pcfirq[2] & PCF50633_INT3_THLIMOFF) {
		DEBUGPC("THLIMOFF ");
		pcf->flags &= ~PCF50633_F_CHG_PROT;
		/* FIXME: signal this to userspace */
	}
	if (pcfirq[2] & PCF50633_INT3_USBLIMON) {
		DEBUGPC("USBLIMON ");
		/* FIXME: signal this to userspace */
	}
	if (pcfirq[2] & PCF50633_INT3_USBLIMOFF) {
		DEBUGPC("USBLIMOFF ");
		/* FIXME: signal this to userspace */
	}
	if (pcfirq[2] & PCF50633_INT3_ADCRDY) {
		/* ADC result ready */
		DEBUGPC("ADCRDY ");
		tail = pcf->adc_queue_tail;
		pcf->adc_queue_tail = (pcf->adc_queue_tail + 1) &
				      (MAX_ADC_FIFO_DEPTH - 1);

		switch (pcf->adc_queue_mux[tail]) {
		case PCF50633_ADCC1_MUX_BATSNS_RES: /* battery voltage */
			pcf->flag_bat_voltage_read =
						  async_adc_complete(pcf);
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
			 * valid charger power
			 */
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

	pcf->working = 0;
	input_sync(pcf->input_dev);
	put_device(&pcf->client.dev);
	mutex_unlock(&pcf->working_lock);
}

static irqreturn_t pcf50633_irq(int irq, void *_pcf)
{
	struct pcf50633_data *pcf = _pcf;

	DEBUGP("entering(irq=%u, pcf=%p): scheduling work\n", irq, _pcf);

	get_device(&pcf->client.dev);
	if (!schedule_work(&pcf->work) && !pcf->working)
		dev_dbg(&pcf->client.dev, "work item may be lost\n");

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
void pcf50633_usb_curlim_set(struct pcf50633_data *pcf, int ma)
{
	u_int8_t bits;

	dev_dbg(&pcf->client.dev, "setting usb current limit to %d ma", ma);

	if (ma >= 1000)
		bits = PCF50633_MBCC7_USB_1000mA;
	else if (ma >= 500)
		bits = PCF50633_MBCC7_USB_500mA;
	else if (ma >= 100)
		bits = PCF50633_MBCC7_USB_100mA;
	else
		bits = PCF50633_MBCC7_USB_SUSPEND;

	reg_set_bit_mask(pcf, PCF50633_REG_MBCC7, PCF56033_MBCC7_USB_MASK,
			 bits);
}
EXPORT_SYMBOL_GPL(pcf50633_usb_curlim_set);

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
void pcf50633_charge_enable(struct pcf50633_data *pcf, int on)
{
	u_int8_t bits;

	if (!(pcf->pdata->used_features & PCF50633_FEAT_MBC))
		return;

	if (on) {
		pcf->flags |= PCF50633_F_CHG_ENABLED;
		bits = PCF50633_MBCC1_CHGENA;
	} else {
		pcf->flags &= ~PCF50633_F_CHG_ENABLED;
		bits = 0;
	}
	reg_set_bit_mask(pcf, PCF50633_REG_MBCC1, PCF50633_MBCC1_CHGENA,
			 bits);
}
EXPORT_SYMBOL_GPL(pcf50633_charge_enable);

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

struct pcf50633_time {
	u_int8_t sec;
	u_int8_t min;
	u_int8_t hour;
	u_int8_t wkday;
	u_int8_t day;
	u_int8_t month;
	u_int8_t year;
};

static void pcf2rtc_time(struct rtc_time *rtc, struct pcf50633_time *pcf)
{
	rtc->tm_sec = BCD2BIN(pcf->sec);
	rtc->tm_min = BCD2BIN(pcf->min);
	rtc->tm_hour = BCD2BIN(pcf->hour);
	rtc->tm_wday = BCD2BIN(pcf->wkday);
	rtc->tm_mday = BCD2BIN(pcf->day);
	rtc->tm_mon = BCD2BIN(pcf->month);
	rtc->tm_year = BCD2BIN(pcf->year) + 100;
}

static void rtc2pcf_time(struct pcf50633_time *pcf, struct rtc_time *rtc)
{
	pcf->sec = BIN2BCD(rtc->tm_sec);
	pcf->min = BIN2BCD(rtc->tm_min);
	pcf->hour = BIN2BCD(rtc->tm_hour);
	pcf->wkday = BIN2BCD(rtc->tm_wday);
	pcf->day = BIN2BCD(rtc->tm_mday);
	pcf->month = BIN2BCD(rtc->tm_mon);
	pcf->year = BIN2BCD(rtc->tm_year - 100);
}

static int pcf50633_rtc_ioctl(struct device *dev, unsigned int cmd,
			      unsigned long arg)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);
	switch (cmd) {
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

	mutex_lock(&pcf->lock);
	pcf_tm.sec = __reg_read(pcf, PCF50633_REG_RTCSC);
	pcf_tm.min = __reg_read(pcf, PCF50633_REG_RTCMN);
	pcf_tm.hour = __reg_read(pcf, PCF50633_REG_RTCHR);
	pcf_tm.wkday = __reg_read(pcf, PCF50633_REG_RTCWD);
	pcf_tm.day = __reg_read(pcf, PCF50633_REG_RTCDT);
	pcf_tm.month = __reg_read(pcf, PCF50633_REG_RTCMT);
	pcf_tm.year = __reg_read(pcf, PCF50633_REG_RTCYR);
	mutex_unlock(&pcf->lock);

	DEBUGP("PCF_TIME: %02x.%02x.%02x %02x:%02x:%02x\n",
		pcf_tm.day, pcf_tm.month, pcf_tm.year,
		pcf_tm.hour, pcf_tm.min, pcf_tm.sec);

	pcf2rtc_time(tm, &pcf_tm);

	DEBUGP("RTC_TIME: %u.%u.%u %u:%u:%u\n",
		tm->tm_mday, tm->tm_mon, tm->tm_year,
		tm->tm_hour, tm->tm_min, tm->tm_sec);

	return 0;
}

static int pcf50633_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);
	struct pcf50633_time pcf_tm;

	DEBUGP("RTC_TIME: %u.%u.%u %u:%u:%u\n",
		tm->tm_mday, tm->tm_mon, tm->tm_year,
		tm->tm_hour, tm->tm_min, tm->tm_sec);
	rtc2pcf_time(&pcf_tm, tm);
	DEBUGP("PCF_TIME: %02x.%02x.%02x %02x:%02x:%02x\n",
		pcf_tm.day, pcf_tm.month, pcf_tm.year,
		pcf_tm.hour, pcf_tm.min, pcf_tm.sec);

	mutex_lock(&pcf->lock);
	/* FIXME: disable second interrupt */
	__reg_write(pcf, PCF50633_REG_RTCSC, pcf_tm.sec);
	__reg_write(pcf, PCF50633_REG_RTCMN, pcf_tm.min);
	__reg_write(pcf, PCF50633_REG_RTCHR, pcf_tm.hour);
	__reg_write(pcf, PCF50633_REG_RTCWD, pcf_tm.wkday);
	__reg_write(pcf, PCF50633_REG_RTCDT, pcf_tm.day);
	__reg_write(pcf, PCF50633_REG_RTCMT, pcf_tm.month);
	__reg_write(pcf, PCF50633_REG_RTCYR, pcf_tm.year);
	/* FIXME: re-enable second interrupt */
	mutex_unlock(&pcf->lock);

	return 0;
}

static int pcf50633_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);
	struct pcf50633_time pcf_tm;

	mutex_lock(&pcf->lock);
	alrm->enabled =
	     __reg_read(pcf, PCF50633_REG_INT1M) & PCF50633_INT1_ALARM ? 0 : 1;
	pcf_tm.sec = __reg_read(pcf, PCF50633_REG_RTCSCA);
	pcf_tm.min = __reg_read(pcf, PCF50633_REG_RTCMNA);
	pcf_tm.hour = __reg_read(pcf, PCF50633_REG_RTCHRA);
	pcf_tm.wkday = __reg_read(pcf, PCF50633_REG_RTCWDA);
	pcf_tm.day = __reg_read(pcf, PCF50633_REG_RTCDTA);
	pcf_tm.month = __reg_read(pcf, PCF50633_REG_RTCMTA);
	pcf_tm.year = __reg_read(pcf, PCF50633_REG_RTCYRA);
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

	rtc2pcf_time(&pcf_tm, &alrm->time);

	mutex_lock(&pcf->lock);

	/* disable alarm interrupt */
	irqmask = __reg_read(pcf, PCF50633_REG_INT1M);
	irqmask |= PCF50633_INT1_ALARM;
	__reg_write(pcf, PCF50633_REG_INT1M, irqmask);

	__reg_write(pcf, PCF50633_REG_RTCSCA, pcf_tm.sec);
	__reg_write(pcf, PCF50633_REG_RTCMNA, pcf_tm.min);
	__reg_write(pcf, PCF50633_REG_RTCHRA, pcf_tm.hour);
	__reg_write(pcf, PCF50633_REG_RTCWDA, pcf_tm.wkday);
	__reg_write(pcf, PCF50633_REG_RTCDTA, pcf_tm.day);
	__reg_write(pcf, PCF50633_REG_RTCMTA, pcf_tm.month);
	__reg_write(pcf, PCF50633_REG_RTCYRA, pcf_tm.year);

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

	return intensity & 0x3f;
}

static int pcf50633bl_set_intensity(struct backlight_device *bd)
{
	struct pcf50633_data *pcf = bl_get_data(bd);
	int intensity = bd->props.brightness;
	int old_intensity = reg_read(pcf, PCF50633_REG_LEDOUT);
	u_int8_t ledena;
	int ret;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;

	/* The PCF50633 seems to have some kind of oddity (bug?) when
	 * the intensity was 0, you need to completely switch it off
	 * and re-enable it, before it produces any output voltage again */

	if (intensity != 0 && old_intensity == 0) {
		ledena = reg_read(pcf, PCF50633_REG_LEDENA);
		reg_write(pcf, PCF50633_REG_LEDENA, 0x00);
	}

	ret = reg_set_bit_mask(pcf, PCF50633_REG_LEDOUT, 0x3f,
			       intensity);

	if (intensity != 0 && old_intensity == 0)
		reg_write(pcf, PCF50633_REG_LEDENA, ledena);

	return ret;
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

#ifdef CONFIG_MACH_NEO1973_GTA02
/* We currently place those platform devices here to make sure the device
 * suspend/resume order is correct */
static struct platform_device gta01_pm_gps_dev = {
	.name		= "neo1973-pm-gps",
};

static struct platform_device gta01_pm_bt_dev = {
	.name		= "neo1973-pm-bt",
};
#endif

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
	struct pcf50633_data *data;
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

	if (!(data = kzalloc(sizeof(*data), GFP_KERNEL)))
		return -ENOMEM;

	mutex_init(&data->lock);
	mutex_init(&data->working_lock);
	INIT_WORK(&data->work, pcf50633_work);
	data->irq = irq;
	data->working = 0;
	data->onkey_seconds = -1;
	data->pdata = pcf50633_pdev->dev.platform_data;

	new_client = &data->client;
	i2c_set_clientdata(new_client, data);
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

	pcf50633_global = data;

	populate_sysfs_group(data);

	err = sysfs_create_group(&new_client->dev.kobj, &pcf_attr_group);
	if (err) {
		dev_err(&new_client->dev, "error creating sysfs group\n");
		goto exit_detach;
	}

	/* create virtual charger 'device' */

	/* register power off handler with core power management */
	pm_power_off = &pcf50633_go_standby;

	data->input_dev = input_allocate_device();
	if (!data->input_dev)
		goto exit_sysfs;

	data->input_dev->name = "GTA02 PMU events";
	data->input_dev->phys = "FIXME";
	data->input_dev->id.bustype = BUS_I2C;
	data->input_dev->cdev.dev = &new_client->dev;

	data->input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_PWR);
	set_bit(KEY_POWER, data->input_dev->keybit);
	set_bit(KEY_POWER2, data->input_dev->keybit);
	set_bit(KEY_BATTERY, data->input_dev->keybit);

	err = input_register_device(data->input_dev);
	if (err)
		goto exit_sysfs;

	/* configure interrupt mask */
	reg_write(data, PCF50633_REG_INT1M, 0x00); /* we want SECOND to kick */
	reg_write(data, PCF50633_REG_INT2M, 0x00);
	reg_write(data, PCF50633_REG_INT3M, 0x00);
	reg_write(data, PCF50633_REG_INT4M, 0x00);
	reg_write(data, PCF50633_REG_INT5M, 0x00);

	err = request_irq(irq, pcf50633_irq, IRQF_TRIGGER_FALLING,
			  "pcf50633", data);
	if (err < 0)
		goto exit_input;

	if (enable_irq_wake(irq) < 0)
		dev_err(&new_client->dev, "IRQ %u cannot be enabled as wake-up"
		        "source in this hardware revision!", irq);

	if (data->pdata->used_features & PCF50633_FEAT_RTC) {
		data->rtc = rtc_device_register("pcf50633", &new_client->dev,
						&pcf50633_rtc_ops, THIS_MODULE);
		if (IS_ERR(data->rtc)) {
			err = PTR_ERR(data->rtc);
			goto exit_irq;
		}
	}

	if (data->pdata->used_features & PCF50633_FEAT_PWM_BL) {
		data->backlight = backlight_device_register("pcf50633-bl",
							    &new_client->dev,
							    data,
							    &pcf50633bl_ops);
		if (!data->backlight)
			goto exit_rtc;
		/* FIXME: are we sure we want default == off? */
		data->backlight->props.max_brightness = 0x3f;
		data->backlight->props.power = FB_BLANK_UNBLANK;
		data->backlight->props.fb_blank = FB_BLANK_UNBLANK;
		data->backlight->props.brightness =
					data->backlight->props.max_brightness;
		backlight_update_status(data->backlight);
	}

	apm_get_power_status = pcf50633_get_power_status;

#ifdef CONFIG_MACH_NEO1973_GTA02
	if (machine_is_neo1973_gta02()) {
		gta01_pm_gps_dev.dev.parent = &new_client->dev;
		gta01_pm_bt_dev.dev.parent = &new_client->dev;
		platform_device_register(&gta01_pm_bt_dev);
		platform_device_register(&gta01_pm_gps_dev);
	}
#endif

	return 0;
exit_rtc:
	if (data->pdata->used_features & PCF50633_FEAT_RTC)
		rtc_device_unregister(pcf50633_global->rtc);
exit_irq:
	free_irq(pcf50633_global->irq, pcf50633_global);
exit_input:
	input_unregister_device(data->input_dev);
exit_sysfs:
	pm_power_off = NULL;
	sysfs_remove_group(&new_client->dev.kobj, &pcf_attr_group);
exit_detach:
	i2c_detach_client(new_client);
exit_free:
	kfree(data);
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

#ifdef CONFIG_MACH_NEO1973_GTA02
	if (machine_is_neo1973_gta02()) {
		platform_device_unregister(&gta01_pm_bt_dev);
		platform_device_unregister(&gta01_pm_gps_dev);
	}
#endif

	sysfs_remove_group(&client->dev.kobj, &pcf_attr_group);

	pm_power_off = NULL;

	kfree(pcf);

	return 0;
}

#ifdef CONFIG_PM
#define INT1M_RESUMERS	(PCF50633_INT1_ADPINS		| \
			 PCF50633_INT1_ADPREM		| \
			 PCF50633_INT1_USBINS		| \
			 PCF50633_INT1_USBREM		| \
			 PCF50633_INT1_ALARM)
#define INT2M_RESUMERS	(PCF50633_INT2_ONKEYF)
#define INT3M_RESUMERS	(PCF50633_INT3_BATFULL		| \
			 PCF50633_INT3_CHGHALT		| \
			 PCF50633_INT3_THLIMON		| \
			 PCF50633_INT3_THLIMOFF		| \
			 PCF50633_INT3_USBLIMON		| \
			 PCF50633_INT3_USBLIMOFF	| \
			 PCF50633_INT3_ONKEY1S)
#define INT4M_RESUMERS	(PCF50633_INT4_LOWSYS		| \
			 PCF50633_INT4_LOWBAT		| \
			 PCF50633_INT4_HIGHTMP)
#define INT5M_RESUMERS	(0)

static int pcf50633_suspend(struct device *dev, pm_message_t state)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);
	int i;

	/* The general idea is to power down all unused power supplies,
	 * and then mask all PCF50606 interrup sources but EXTONR, ONKEYF
	 * and ALARM */

	mutex_lock(&pcf->lock);

	/* Save all registers that don't "survive" standby state */
	pcf->standby_regs.ooctim2 = __reg_read(pcf, PCF50633_REG_OOCTIM2);
	pcf->standby_regs.autoout = __reg_read(pcf, PCF50633_REG_AUTOOUT);
	pcf->standby_regs.autoena = __reg_read(pcf, PCF50633_REG_AUTOENA);
	pcf->standby_regs.automxc = __reg_read(pcf, PCF50633_REG_AUTOMXC);
	pcf->standby_regs.down1out = __reg_read(pcf, PCF50633_REG_DOWN1OUT);
	pcf->standby_regs.down1mxc = __reg_read(pcf, PCF50633_REG_DOWN1MXC);
	pcf->standby_regs.down2out = __reg_read(pcf, PCF50633_REG_DOWN2OUT);
	pcf->standby_regs.down2ena = __reg_read(pcf, PCF50633_REG_DOWN2ENA);
	pcf->standby_regs.memldoout = __reg_read(pcf, PCF50633_REG_MEMLDOOUT);
	pcf->standby_regs.memldoena = __reg_read(pcf, PCF50633_REG_MEMLDOENA);
	pcf->standby_regs.ledout = __reg_read(pcf, PCF50633_REG_LEDOUT);
	pcf->standby_regs.ledena = __reg_read(pcf, PCF50633_REG_LEDENA);
	pcf->standby_regs.leddim = __reg_read(pcf, PCF50633_REG_LEDDIM);
	/* FIXME: one big read? */
	for (i = 0; i < 7; i++) {
		u_int8_t reg_out = PCF50633_REG_LDO1OUT + 2*i;
		pcf->standby_regs.ldo[i].out = __reg_read(pcf, reg_out);
		pcf->standby_regs.ldo[i].ena = __reg_read(pcf, reg_out+1);
	}

	/* switch off power supplies that are not needed during suspend */
	for (i = 0; i < __NUM_PCF50633_REGULATORS; i++) {
		if (!(pcf->pdata->rails[i].flags & PMU_VRAIL_F_SUSPEND_ON)) {
			u_int8_t tmp;

			DEBUGP("disabling pcf50633 regulator %u\n", i);
			/* we cannot use pcf50633_onoff_set() because we're
			 * already under the mutex */
			tmp = __reg_read(pcf, regulator_registers[i]+1);
			tmp &= 0xfe;
			__reg_write(pcf, regulator_registers[i]+1, tmp);
		}
	}

	/* turn off the backlight */
	__reg_write(pcf, PCF50633_REG_LEDENA, 0x00);

	pcf->standby_regs.int1m = __reg_read(pcf, PCF50633_REG_INT1M);
	pcf->standby_regs.int2m = __reg_read(pcf, PCF50633_REG_INT2M);
	pcf->standby_regs.int3m = __reg_read(pcf, PCF50633_REG_INT3M);
	pcf->standby_regs.int4m = __reg_read(pcf, PCF50633_REG_INT4M);
	pcf->standby_regs.int5m = __reg_read(pcf, PCF50633_REG_INT5M);
	__reg_write(pcf, PCF50633_REG_INT1M, ~INT1M_RESUMERS & 0xff);
	__reg_write(pcf, PCF50633_REG_INT2M, ~INT2M_RESUMERS & 0xff);
	__reg_write(pcf, PCF50633_REG_INT3M, ~INT3M_RESUMERS & 0xff);
	__reg_write(pcf, PCF50633_REG_INT4M, ~INT4M_RESUMERS & 0xff);
	__reg_write(pcf, PCF50633_REG_INT5M, ~INT5M_RESUMERS & 0xff);

	mutex_unlock(&pcf->lock);

	return 0;
}

static int pcf50633_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50633_data *pcf = i2c_get_clientdata(client);
	int i;

	printk(KERN_INFO"a\n");
	/* mutex_lock(&pcf->lock); */  /* resume in atomic context */

	__reg_write(pcf, PCF50633_REG_LEDENA, 0x01);
	printk(KERN_INFO"b\n");

	/* Resume all saved registers that don't "survive" standby state */
	__reg_write(pcf, PCF50633_REG_INT1M, pcf->standby_regs.int1m);
	__reg_write(pcf, PCF50633_REG_INT2M, pcf->standby_regs.int2m);
	__reg_write(pcf, PCF50633_REG_INT3M, pcf->standby_regs.int3m);
	__reg_write(pcf, PCF50633_REG_INT4M, pcf->standby_regs.int4m);
	__reg_write(pcf, PCF50633_REG_INT5M, pcf->standby_regs.int5m);
	printk(KERN_INFO"c\n");

	__reg_write(pcf, PCF50633_REG_OOCTIM2, pcf->standby_regs.ooctim2);
	__reg_write(pcf, PCF50633_REG_AUTOOUT, pcf->standby_regs.autoout);
	__reg_write(pcf, PCF50633_REG_AUTOMXC, pcf->standby_regs.automxc);
	__reg_write(pcf, PCF50633_REG_DOWN1OUT, pcf->standby_regs.down1out);
	__reg_write(pcf, PCF50633_REG_DOWN1MXC, pcf->standby_regs.down1mxc);
	__reg_write(pcf, PCF50633_REG_DOWN2OUT, pcf->standby_regs.down2out);
	__reg_write(pcf, PCF50633_REG_DOWN2ENA, pcf->standby_regs.down2ena);
	__reg_write(pcf, PCF50633_REG_MEMLDOOUT, pcf->standby_regs.memldoout);
	__reg_write(pcf, PCF50633_REG_MEMLDOENA, pcf->standby_regs.memldoena);
	__reg_write(pcf, PCF50633_REG_LEDOUT, pcf->standby_regs.ledout);
	__reg_write(pcf, PCF50633_REG_LEDENA, pcf->standby_regs.ledena);
	__reg_write(pcf, PCF50633_REG_LEDDIM, pcf->standby_regs.leddim);
	printk(KERN_INFO"d\n");
	/* FIXME: one big read? */
	for (i = 0; i < 7; i++) {
		u_int8_t reg_out = PCF50633_REG_LDO1OUT + 2*i;
		__reg_write(pcf, reg_out, pcf->standby_regs.ldo[i].out);
		__reg_write(pcf, reg_out+1, pcf->standby_regs.ldo[i].ena);
	}
	printk(KERN_INFO"e\n");

	/* mutex_unlock(&pcf->lock); */ /* resume in atomic context */

	pcf50633_irq(pcf->irq, pcf);

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
