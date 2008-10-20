/* Philips/NXP PCF50606 Power Management Unit (PMU) driver
 *
 * (C) 2006-2007 by Openmoko, Inc.
 * Authors: Harald Welte <laforge@openmoko.org>,
 *	    Matt Hsu <matt@openmoko.org>
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
 * - watchdog
 * - adc driver (hw_sensors like)
 * - pwm driver
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
#include <linux/pcf50606.h>
#include <linux/apm-emulation.h>

#include <asm/mach-types.h>
#include <asm/arch/gta01.h>

#include "pcf50606.h"

/* we use dev_dbg() throughout the code, but sometimes don't want to
 * write an entire line of debug related information.  This DEBUGPC
 * macro is a continuation for dev_dbg() */
#ifdef DEBUG
#define DEBUGPC(x, args ...) printk(x, ## args)
#else
#define DEBUGPC(x, args ...)
#endif

/***********************************************************************
 * Static data / structures
 ***********************************************************************/

static unsigned short normal_i2c[] = { 0x08, I2C_CLIENT_END };

I2C_CLIENT_INSMOD_1(pcf50606);

#define PCF50606_B_CHG_FAST	0	/* Charger Fast allowed */
#define PCF50606_B_CHG_PRESENT	1	/* Charger present */
#define PCF50606_B_CHG_FOK	2	/* Fast OK for battery */
#define PCF50606_B_CHG_ERR	3	/* Charger Error */
#define PCF50606_B_CHG_PROT	4	/* Charger Protection */
#define PCF50606_B_CHG_READY	5	/* Charging completed */

#define PCF50606_F_CHG_FAST	(1<<PCF50606_B_CHG_FAST)
#define PCF50606_F_CHG_PRESENT	(1<<PCF50606_B_CHG_PRESENT)
#define PCF50606_F_CHG_FOK	(1<<PCF50606_B_CHG_FOK)
#define PCF50606_F_CHG_ERR	(1<<PCF50606_B_CHG_ERR)
#define PCF50606_F_CHG_PROT	(1<<PCF50606_B_CHG_PROT)
#define PCF50606_F_CHG_READY	(1<<PCF50606_B_CHG_READY)
#define PCF50606_F_CHG_MASK	0x000000fc

#define PCF50606_F_PWR_PRESSED	0x00000100
#define PCF50606_F_RTC_SECOND	0x00000200

enum close_state {
	CLOSE_STATE_NOT,
	CLOSE_STATE_ALLOW = 0x2342,
};

enum pcf50606_suspend_states {
	PCF50606_SS_RUNNING,
	PCF50606_SS_STARTING_SUSPEND,
	PCF50606_SS_COMPLETED_SUSPEND,
	PCF50606_SS_RESUMING_BUT_NOT_US_YET,
	PCF50606_SS_STARTING_RESUME,
	PCF50606_SS_COMPLETED_RESUME,
};

struct pcf50606_data {
	struct i2c_client client;
	struct pcf50606_platform_data *pdata;
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
	int coldplug_done;
	enum pcf50606_suspend_states suspend_state;
#ifdef CONFIG_PM
	struct {
		u_int8_t dcdc1, dcdc2;
		u_int8_t dcdec1;
		u_int8_t dcudc1;
		u_int8_t ioregc;
		u_int8_t d1regc1;
		u_int8_t d2regc1;
		u_int8_t d3regc1;
		u_int8_t lpregc1;
		u_int8_t adcc1, adcc2;
		u_int8_t pwmc1;
		u_int8_t int1m, int2m, int3m;
	} standby_regs;
#endif
};

static struct i2c_driver pcf50606_driver;

/* This is an ugly construct on how to access the (currently single/global)
 * pcf50606 handle from other code in the kernel.  I didn't really come up with
 * a more decent method of dynamically resolving this */
struct pcf50606_data *pcf50606_global;
EXPORT_SYMBOL_GPL(pcf50606_global);

static struct platform_device *pcf50606_pdev;

/* This is a 10k, B=3370 NTC Thermistor -10..79 centigrade */
/* Table entries are offset by +0.5C so a properly rounded value is generated */
static const u_int16_t ntc_table_10k_3370B[] = {
	/* -10 */
	43888, 41819, 39862, 38010, 36257, 34596, 33024, 31534, 30121, 28781,
	27510, 26304, 25159, 24071, 23038, 22056, 21122, 20234, 19390, 18586,
	17821, 17093, 16399, 15738, 15107, 14506, 13933, 13387, 12865, 12367,
	11891, 11437, 11003, 10588, 10192, 9813, 9450, 9103, 8771, 8453,
	8149, 7857, 7578, 7310, 7054, 6808, 6572, 6346, 6129, 5920,
	5720, 5528, 5344, 5167, 4996, 4833, 4675, 4524, 4379, 4239,
	4104, 3975, 3850, 3730, 3614, 3503, 3396, 3292, 3193, 3097,
	3004, 2915, 2829, 2745, 2665, 2588, 2513, 2441, 2371, 2304,
	2239, 2176, 2116, 2057, 2000, 1945, 1892, 1841, 1791, 1743,
};


/***********************************************************************
 * Low-Level routines
 ***********************************************************************/

static inline int __reg_write(struct pcf50606_data *pcf, u_int8_t reg,
			      u_int8_t val)
{
	if (pcf->suspend_state == PCF50606_SS_COMPLETED_SUSPEND) {
		dev_err(&pcf->client.dev, "__reg_write while suspended.\n");
		dump_stack();
	}
	return i2c_smbus_write_byte_data(&pcf->client, reg, val);
}

static int reg_write(struct pcf50606_data *pcf, u_int8_t reg, u_int8_t val)
{
	int ret;

	mutex_lock(&pcf->lock);
	ret = __reg_write(pcf, reg, val);
	mutex_unlock(&pcf->lock);

	return ret;
}

static inline int32_t __reg_read(struct pcf50606_data *pcf, u_int8_t reg)
{
	int32_t ret;

	if (pcf->suspend_state == PCF50606_SS_COMPLETED_SUSPEND) {
		dev_err(&pcf->client.dev, "__reg_read while suspended.\n");
		dump_stack();
	}
	ret = i2c_smbus_read_byte_data(&pcf->client, reg);

	return ret;
}

static u_int8_t reg_read(struct pcf50606_data *pcf, u_int8_t reg)
{
	int32_t ret;

	mutex_lock(&pcf->lock);
	ret = __reg_read(pcf, reg);
	mutex_unlock(&pcf->lock);

	return ret & 0xff;
}

static int reg_set_bit_mask(struct pcf50606_data *pcf,
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

static int reg_clear_bits(struct pcf50606_data *pcf, u_int8_t reg, u_int8_t val)
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

/* synchronously read one ADC channel (busy-wait for result to be complete) */
static u_int16_t adc_read(struct pcf50606_data *pcf,  int channel,
			  u_int16_t *data2)
{
	u_int8_t adcs2, adcs1;
	u_int16_t ret;

	dev_dbg(&pcf->client.dev, "entering (pcf=%p, channel=%u, data2=%p)\n",
		pcf, channel, data2);

	channel &= PCF50606_ADCC2_ADCMUX_MASK;

	mutex_lock(&pcf->lock);

	/* start ADC conversion of selected channel */
	__reg_write(pcf, PCF50606_REG_ADCC2, channel |
		    PCF50606_ADCC2_ADCSTART | PCF50606_ADCC2_RES_10BIT);

	do {
		adcs2 = __reg_read(pcf, PCF50606_REG_ADCS2);
	} while (!(adcs2 & PCF50606_ADCS2_ADCRDY));

	adcs1 = __reg_read(pcf, PCF50606_REG_ADCS1);
	ret = (adcs1 << 2) | (adcs2 & 0x03);

	if (data2) {
		adcs1 = __reg_read(pcf, PCF50606_REG_ADCS3);
		*data2 = (adcs1 << 2) | ((adcs2 & 0x0c) >> 2);
	}

	mutex_unlock(&pcf->lock);

	dev_dbg(&pcf->client.dev, "returning %u %u\n", ret,
		data2 ? *data2 : 0);

	return ret;
}

/***********************************************************************
 * Voltage / ADC
 ***********************************************************************/

static u_int8_t dcudc_voltage(unsigned int millivolts)
{
	if (millivolts < 900)
		return 0;
	if (millivolts > 5500)
		return 0x1f;
	if (millivolts <= 3300) {
		millivolts -= 900;
		return millivolts/300;
	}
	if (millivolts < 4000)
		return 0x0f;
	else {
		millivolts -= 4000;
		return millivolts/100;
	}
}

static unsigned int dcudc_2voltage(u_int8_t bits)
{
	bits &= 0x1f;
	if (bits < 0x08)
		return 900 + bits * 300;
	else if (bits < 0x10)
		return 3300;
	else
		return 4000 + bits * 100;
}

static u_int8_t dcdec_voltage(unsigned int millivolts)
{
	if (millivolts < 900)
		return 0;
	else if (millivolts > 3300)
		return 0x0f;

	millivolts -= 900;
	return millivolts/300;
}

static unsigned int dcdec_2voltage(u_int8_t bits)
{
	bits &= 0x0f;
	return 900 + bits*300;
}

static u_int8_t dcdc_voltage(unsigned int millivolts)
{
	if (millivolts < 900)
		return 0;
	else if (millivolts > 3600)
		return 0x1f;

	if (millivolts < 1500) {
		millivolts -= 900;
		return millivolts/25;
	} else {
		millivolts -= 1500;
		return 0x18 + millivolts/300;
	}
}

static unsigned int dcdc_2voltage(u_int8_t bits)
{
	bits &= 0x1f;
	if ((bits & 0x18) == 0x18)
		return 1500 + ((bits & 0x7) * 300);
	else
		return 900 + (bits * 25);
}

static u_int8_t dx_voltage(unsigned int millivolts)
{
	if (millivolts < 900)
		return 0;
	else if (millivolts > 3300)
		return 0x18;

	millivolts -= 900;
	return millivolts/100;
}

static unsigned int dx_2voltage(u_int8_t bits)
{
	bits &= 0x1f;
	return 900 + (bits * 100);
}

static const u_int8_t regulator_registers[__NUM_PCF50606_REGULATORS] = {
	[PCF50606_REGULATOR_DCD]	= PCF50606_REG_DCDC1,
	[PCF50606_REGULATOR_DCDE]	= PCF50606_REG_DCDEC1,
	[PCF50606_REGULATOR_DCUD]	= PCF50606_REG_DCUDC1,
	[PCF50606_REGULATOR_D1REG]	= PCF50606_REG_D1REGC1,
	[PCF50606_REGULATOR_D2REG]	= PCF50606_REG_D2REGC1,
	[PCF50606_REGULATOR_D3REG]	= PCF50606_REG_D3REGC1,
	[PCF50606_REGULATOR_LPREG]	= PCF50606_REG_LPREGC1,
	[PCF50606_REGULATOR_IOREG]	= PCF50606_REG_IOREGC,
};

int pcf50606_onoff_set(struct pcf50606_data *pcf,
		       enum pcf50606_regulator_id reg, int on)
{
	u_int8_t addr;

	if (reg >= __NUM_PCF50606_REGULATORS)
		return -EINVAL;

	/* IOREG cannot be powered off since it powers the PMU I2C */
	if (reg == PCF50606_REGULATOR_IOREG)
		return -EIO;

	addr = regulator_registers[reg];

	if (on == 0)
		reg_set_bit_mask(pcf, addr, 0xe0, 0x00);
	else
		reg_set_bit_mask(pcf, addr, 0xe0, 0xe0);

	return 0;
}
EXPORT_SYMBOL_GPL(pcf50606_onoff_set);

int pcf50606_onoff_get(struct pcf50606_data *pcf,
		       enum pcf50606_regulator_id reg)
{
	u_int8_t val, addr;

	if (reg >= __NUM_PCF50606_REGULATORS)
		return -EINVAL;

	addr = regulator_registers[reg];
	val = (reg_read(pcf, addr) & 0xe0) >> 5;

	/* PWREN1 = 1, PWREN2 = 1, see table 16 of datasheet */
	switch (val) {
	case 0:
	case 5:
		return 0;
	default:
		return 1;
	}
}
EXPORT_SYMBOL_GPL(pcf50606_onoff_get);

int pcf50606_voltage_set(struct pcf50606_data *pcf,
			 enum pcf50606_regulator_id reg,
			 unsigned int millivolts)
{
	u_int8_t volt_bits;
	u_int8_t regnr;
	int rc;

	dev_dbg(&pcf->client.dev, "pcf=%p, reg=%d, mvolts=%d\n", pcf, reg,
		millivolts);

	if (reg >= __NUM_PCF50606_REGULATORS)
		return -EINVAL;

	if (millivolts > pcf->pdata->rails[reg].voltage.max)
		return -EINVAL;

	switch (reg) {
	case PCF50606_REGULATOR_DCD:
		volt_bits = dcdc_voltage(millivolts);
		rc = reg_set_bit_mask(pcf, PCF50606_REG_DCDC1, 0x1f,
				      volt_bits);
		break;
	case PCF50606_REGULATOR_DCDE:
		volt_bits = dcdec_voltage(millivolts);
		rc = reg_set_bit_mask(pcf, PCF50606_REG_DCDEC1, 0x0f,
				      volt_bits);
		break;
	case PCF50606_REGULATOR_DCUD:
		volt_bits = dcudc_voltage(millivolts);
		rc = reg_set_bit_mask(pcf, PCF50606_REG_DCUDC1, 0x1f,
				      volt_bits);
		break;
	case PCF50606_REGULATOR_D1REG:
	case PCF50606_REGULATOR_D2REG:
	case PCF50606_REGULATOR_D3REG:
		regnr = PCF50606_REG_D1REGC1 + (reg - PCF50606_REGULATOR_D1REG);
		volt_bits = dx_voltage(millivolts);
		rc = reg_set_bit_mask(pcf, regnr, 0x1f, volt_bits);
		break;
	case PCF50606_REGULATOR_LPREG:
		volt_bits = dx_voltage(millivolts);
		rc = reg_set_bit_mask(pcf, PCF50606_REG_LPREGC1, 0x1f,
				      volt_bits);
		break;
	case PCF50606_REGULATOR_IOREG:
		if (millivolts < 1800)
			return -EINVAL;
		volt_bits = dx_voltage(millivolts);
		rc = reg_set_bit_mask(pcf, PCF50606_REG_IOREGC, 0x1f,
				      volt_bits);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL_GPL(pcf50606_voltage_set);

unsigned int pcf50606_voltage_get(struct pcf50606_data *pcf,
			 enum pcf50606_regulator_id reg)
{
	u_int8_t volt_bits;
	u_int8_t regnr;
	unsigned int rc = 0;

	if (reg >= __NUM_PCF50606_REGULATORS)
		return -EINVAL;

	switch (reg) {
	case PCF50606_REGULATOR_DCD:
		volt_bits = reg_read(pcf, PCF50606_REG_DCDC1) & 0x1f;
		rc = dcdc_2voltage(volt_bits);
		break;
	case PCF50606_REGULATOR_DCDE:
		volt_bits = reg_read(pcf, PCF50606_REG_DCDEC1) & 0x0f;
		rc = dcdec_2voltage(volt_bits);
		break;
	case PCF50606_REGULATOR_DCUD:
		volt_bits = reg_read(pcf, PCF50606_REG_DCUDC1) & 0x1f;
		rc = dcudc_2voltage(volt_bits);
		break;
	case PCF50606_REGULATOR_D1REG:
	case PCF50606_REGULATOR_D2REG:
	case PCF50606_REGULATOR_D3REG:
		regnr = PCF50606_REG_D1REGC1 + (reg - PCF50606_REGULATOR_D1REG);
		volt_bits = reg_read(pcf, regnr) & 0x1f;
		if (volt_bits > 0x18)
			volt_bits = 0x18;
		rc = dx_2voltage(volt_bits);
		break;
	case PCF50606_REGULATOR_LPREG:
		volt_bits = reg_read(pcf, PCF50606_REG_LPREGC1) & 0x1f;
		if (volt_bits > 0x18)
			volt_bits = 0x18;
		rc = dx_2voltage(volt_bits);
		break;
	case PCF50606_REGULATOR_IOREG:
		volt_bits = reg_read(pcf, PCF50606_REG_IOREGC) & 0x1f;
		if (volt_bits > 0x18)
			volt_bits = 0x18;
		rc = dx_2voltage(volt_bits);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL_GPL(pcf50606_voltage_get);

/* go into 'STANDBY' mode, i.e. power off the main CPU and peripherals */
void pcf50606_go_standby(void)
{
	reg_write(pcf50606_global, PCF50606_REG_OOCC1,
		  PCF50606_OOCC1_GOSTDBY);
}
EXPORT_SYMBOL_GPL(pcf50606_go_standby);

void pcf50606_gpo0_set(struct pcf50606_data *pcf, int on)
{
	u_int8_t val;

	if (on)
		val = 0x07;
	else
		val = 0x0f;

	reg_set_bit_mask(pcf, PCF50606_REG_GPOC1, 0x0f, val);
}
EXPORT_SYMBOL_GPL(pcf50606_gpo0_set);

int pcf50606_gpo0_get(struct pcf50606_data *pcf)
{
	u_int8_t reg = reg_read(pcf, PCF50606_REG_GPOC1) & 0x0f;

	if (reg == 0x07 || reg == 0x08)
		return 1;

	return 0;
}
EXPORT_SYMBOL_GPL(pcf50606_gpo0_get);

static void pcf50606_work(struct work_struct *work)
{
	struct pcf50606_data *pcf =
			container_of(work, struct pcf50606_data, work);
	u_int8_t pcfirq[3];
	int ret;

	mutex_lock(&pcf->working_lock);
	pcf->working = 1;

	/* sanity */
	if (!&pcf->client.dev)
		goto bail;

	/*
	 * if we are presently suspending, we are not in a position to deal
	 * with pcf50606 interrupts at all.
	 *
	 * Because we didn't clear the int pending registers, there will be
	 * no edge / interrupt waiting for us when we wake.  But it is OK
	 * because at the end of our resume, we call this workqueue function
	 * gratuitously, clearing the pending register and re-enabling
	 * servicing this interrupt.
	 */

	if ((pcf->suspend_state == PCF50606_SS_STARTING_SUSPEND) ||
	    (pcf->suspend_state == PCF50606_SS_COMPLETED_SUSPEND))
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
		(pcf->suspend_state != PCF50606_SS_COMPLETED_RESUME))
		goto reschedule;

	/* this is the case early in resume! Sanity check! */
	if (i2c_get_clientdata(&pcf->client) == NULL)
		goto reschedule;

	/*
	 * p35 pcf50606 datasheet rev 2.2:
	 * ''The system controller shall read all interrupt registers in
	 *   one I2C read action''
	 * because if you don't INT# gets stuck asserted forever after a
	 * while
	 */
	ret = i2c_smbus_read_i2c_block_data(&pcf->client, PCF50606_REG_INT1,
					    sizeof(pcfirq), pcfirq);
	if (ret != sizeof(pcfirq)) {
		DEBUGPC("Oh crap PMU IRQ register read failed %d\n", ret);
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
	 *
	 * pcf50606 resume is really really over now then.
	 */
	if (pcf->suspend_state != PCF50606_SS_RUNNING)
		pcf->suspend_state = PCF50606_SS_RUNNING;

	if (!pcf->coldplug_done) {
		DEBUGPC("PMU Coldplug init\n");

		/* we used SECOND to kick ourselves started -- turn it off */
		pcfirq[0] &= ~PCF50606_INT1_SECOND;
		reg_set_bit_mask(pcf, PCF50606_REG_INT1M, PCF50606_INT1_SECOND,
				 PCF50606_INT1_SECOND);

		/* coldplug the USB if present */
		if (__reg_read(pcf, PCF50606_REG_OOCS) & PCF50606_OOCS_EXTON) {
			/* Charger inserted */
			DEBUGPC("COLD CHGINS ");
			input_report_key(pcf->input_dev, KEY_BATTERY, 1);
			apm_queue_event(APM_POWER_STATUS_CHANGE);
			pcf->flags |= PCF50606_F_CHG_PRESENT;
			if (pcf->pdata->cb)
				pcf->pdata->cb(&pcf->client.dev,
						PCF50606_FEAT_MBC,
						PMU_EVT_INSERT);
		}

		pcf->coldplug_done = 1;
	}


	dev_dbg(&pcf->client.dev, "INT1=0x%02x INT2=0x%02x INT3=0x%02x:",
		pcfirq[0], pcfirq[1], pcfirq[2]);

	if (pcfirq[0] & PCF50606_INT1_ONKEYF) {
		/* ONKEY falling edge (start of button press) */
		DEBUGPC("ONKEYF ");
		pcf->flags |= PCF50606_F_PWR_PRESSED;
		input_report_key(pcf->input_dev, KEY_POWER, 1);
	}
	if (pcfirq[0] & PCF50606_INT1_ONKEY1S) {
		/* ONKEY pressed for more than 1 second */
		pcf->onkey_seconds = 0;
		DEBUGPC("ONKEY1S ");
		/* Tell PMU we are taking care of this */
		reg_set_bit_mask(pcf, PCF50606_REG_OOCC1,
				 PCF50606_OOCC1_TOTRST,
				 PCF50606_OOCC1_TOTRST);
		/* enable SECOND interrupt (hz tick) */
		reg_clear_bits(pcf, PCF50606_REG_INT1M, PCF50606_INT1_SECOND);
	}
	if (pcfirq[0] & PCF50606_INT1_ONKEYR) {
		/* ONKEY rising edge (end of button press) */
		DEBUGPC("ONKEYR ");
		pcf->flags &= ~PCF50606_F_PWR_PRESSED;
		pcf->onkey_seconds = -1;
		input_report_key(pcf->input_dev, KEY_POWER, 0);
		/* disable SECOND interrupt in case RTC didn't
		 * request it */
		if (!(pcf->flags & PCF50606_F_RTC_SECOND))
			reg_set_bit_mask(pcf, PCF50606_REG_INT1M,
					 PCF50606_INT1_SECOND,
					 PCF50606_INT1_SECOND);
	}
	if (pcfirq[0] & PCF50606_INT1_EXTONR) {
		DEBUGPC("EXTONR ");
		input_report_key(pcf->input_dev, KEY_POWER2, 1);
	}
	if (pcfirq[0] & PCF50606_INT1_EXTONF) {
		DEBUGPC("EXTONF ");
		input_report_key(pcf->input_dev, KEY_POWER2, 0);
	}
	if (pcfirq[0] & PCF50606_INT1_SECOND) {
		DEBUGPC("SECOND ");
		if (pcf->flags & PCF50606_F_RTC_SECOND)
			rtc_update_irq(pcf->rtc, 1,
				       RTC_PF | RTC_IRQF);

		if (pcf->onkey_seconds >= 0 &&
		    pcf->flags & PCF50606_F_PWR_PRESSED) {
			DEBUGPC("ONKEY_SECONDS(%u, OOCC1=0x%02x) ",
				pcf->onkey_seconds,
				reg_read(pcf, PCF50606_REG_OOCC1));
			pcf->onkey_seconds++;
			if (pcf->onkey_seconds >=
			    pcf->pdata->onkey_seconds_required) {
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
					kill_proc(1, SIGINT, 1);
					DEBUGPC("SIGINT(init) ");
				}
				/* FIXME: what to do if userspace doesn't
				 * shut down? Do we want to force it? */
			}
		}
	}
	if (pcfirq[0] & PCF50606_INT1_ALARM) {
		DEBUGPC("ALARM ");
		if (pcf->pdata->used_features & PCF50606_FEAT_RTC)
			rtc_update_irq(pcf->rtc, 1,
				       RTC_AF | RTC_IRQF);
	}

	if (pcfirq[1] & PCF50606_INT2_CHGINS) {
		/* Charger inserted */
		DEBUGPC("CHGINS ");
		input_report_key(pcf->input_dev, KEY_BATTERY, 1);
		apm_queue_event(APM_POWER_STATUS_CHANGE);
		pcf->flags |= PCF50606_F_CHG_PRESENT;
		if (pcf->pdata->cb)
			pcf->pdata->cb(&pcf->client.dev,
				       PCF50606_FEAT_MBC, PMU_EVT_INSERT);
		/* FIXME: how to signal this to userspace */
	}
	if (pcfirq[1] & PCF50606_INT2_CHGRM) {
		/* Charger removed */
		DEBUGPC("CHGRM ");
		input_report_key(pcf->input_dev, KEY_BATTERY, 0);
		apm_queue_event(APM_POWER_STATUS_CHANGE);
		pcf->flags &= ~(PCF50606_F_CHG_MASK|PCF50606_F_CHG_PRESENT);
		if (pcf->pdata->cb)
			pcf->pdata->cb(&pcf->client.dev,
				       PCF50606_FEAT_MBC, PMU_EVT_INSERT);
		/* FIXME: how signal this to userspace */
	}
	if (pcfirq[1] & PCF50606_INT2_CHGFOK) {
		/* Battery ready for fast charging */
		DEBUGPC("CHGFOK ");
		pcf->flags |= PCF50606_F_CHG_FOK;
		/* FIXME: how to signal this to userspace */
	}
	if (pcfirq[1] & PCF50606_INT2_CHGERR) {
		/* Error in charge mode */
		DEBUGPC("CHGERR ");
		pcf->flags |= PCF50606_F_CHG_ERR;
		pcf->flags &= ~(PCF50606_F_CHG_FOK|PCF50606_F_CHG_READY);
		/* FIXME: how to signal this to userspace */
	}
	if (pcfirq[1] & PCF50606_INT2_CHGFRDY) {
		/* Fast charge completed */
		DEBUGPC("CHGFRDY ");
		pcf->flags |= PCF50606_F_CHG_READY;
		pcf->flags &= ~PCF50606_F_CHG_FOK;
		/* FIXME: how to signal this to userspace */
	}
	if (pcfirq[1] & PCF50606_INT2_CHGPROT) {
		/* Charging protection interrupt */
		DEBUGPC("CHGPROT ");
		pcf->flags &= ~(PCF50606_F_CHG_FOK|PCF50606_F_CHG_READY);
		/* FIXME: signal this to userspace */
	}
	if (pcfirq[1] & PCF50606_INT2_CHGWD10S) {
		/* Charger watchdog will expire in 10 seconds */
		DEBUGPC("CHGWD10S ");
		reg_set_bit_mask(pcf, PCF50606_REG_OOCC1,
				 PCF50606_OOCC1_WDTRST,
				 PCF50606_OOCC1_WDTRST);
	}
	if (pcfirq[1] & PCF50606_INT2_CHGWDEXP) {
		/* Charger watchdog expires */
		DEBUGPC("CHGWDEXP ");
		/* FIXME: how to signal this to userspace */
	}

	if (pcfirq[2] & PCF50606_INT3_ADCRDY) {
		/* ADC result ready */
		DEBUGPC("ADCRDY ");
	}
	if (pcfirq[2] & PCF50606_INT3_ACDINS) {
		/* Accessory insertion detected */
		DEBUGPC("ACDINS ");
		if (pcf->pdata->cb)
			pcf->pdata->cb(&pcf->client.dev,
				       PCF50606_FEAT_ACD, PMU_EVT_INSERT);
	}
	if (pcfirq[2] & PCF50606_INT3_ACDREM) {
		/* Accessory removal detected */
		DEBUGPC("ACDREM ");
		if (pcf->pdata->cb)
			pcf->pdata->cb(&pcf->client.dev,
				       PCF50606_FEAT_ACD, PMU_EVT_REMOVE);
	}
	/* FIXME: TSCPRES */
	if (pcfirq[2] & PCF50606_INT3_LOWBAT) {
		if (__reg_read(pcf, PCF50606_REG_OOCS) & PCF50606_OOCS_EXTON) {
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
		reg_set_bit_mask(pcf, PCF50606_REG_OOCC1,
				 PCF50606_OOCC1_TOTRST,
				 PCF50606_OOCC1_TOTRST);
	}
	if (pcfirq[2] & PCF50606_INT3_HIGHTMP) {
		/* High temperature */
		DEBUGPC("HIGHTMP ");
		apm_queue_event(APM_CRITICAL_SUSPEND);
	}

	DEBUGPC("\n");

bail:
	pcf->working = 0;
	input_sync(pcf->input_dev);
	put_device(&pcf->client.dev);
	mutex_unlock(&pcf->working_lock);

	return;

reschedule:

	if ((pcf->suspend_state != PCF50606_SS_STARTING_SUSPEND) &&
	    (pcf->suspend_state != PCF50606_SS_COMPLETED_SUSPEND)) {
		msleep(10);
		dev_info(&pcf->client.dev, "rescheduling interrupt service\n");
	}
	if (!schedule_work(&pcf->work))
		dev_err(&pcf->client.dev, "int service reschedule failed\n");

	/* we don't put the device here, hold it for next time */
	mutex_unlock(&pcf->working_lock);
}

static irqreturn_t pcf50606_irq(int irq, void *_pcf)
{
	struct pcf50606_data *pcf = _pcf;

	dev_dbg(&pcf->client.dev, "entering(irq=%u, pcf=%p): scheduling work\n",
		irq, _pcf);
	get_device(&pcf->client.dev);
	if (!schedule_work(&pcf->work) && !pcf->working)
		dev_err(&pcf->client.dev, "pcf irq work already queued.\n");

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

u_int16_t pcf50606_battvolt(struct pcf50606_data *pcf)
{
	u_int16_t adc;
	adc = adc_read(pcf, PCF50606_ADCMUX_BATVOLT_RES, NULL);

	return adc_to_batt_millivolts(adc);
}
EXPORT_SYMBOL_GPL(pcf50606_battvolt);

static ssize_t show_battvolt(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50606_data *pcf = i2c_get_clientdata(client);

	return sprintf(buf, "%u\n", pcf50606_battvolt(pcf));
}
static DEVICE_ATTR(battvolt, S_IRUGO | S_IWUSR, show_battvolt, NULL);

static int reg_id_by_name(const char *name)
{
	int reg_id;

	if (!strcmp(name, "voltage_dcd"))
		reg_id = PCF50606_REGULATOR_DCD;
	else if (!strcmp(name, "voltage_dcde"))
		reg_id = PCF50606_REGULATOR_DCDE;
	else if (!strcmp(name, "voltage_dcud"))
		reg_id = PCF50606_REGULATOR_DCUD;
	else if (!strcmp(name, "voltage_d1reg"))
		reg_id = PCF50606_REGULATOR_D1REG;
	else if (!strcmp(name, "voltage_d2reg"))
		reg_id = PCF50606_REGULATOR_D2REG;
	else if (!strcmp(name, "voltage_d3reg"))
		reg_id = PCF50606_REGULATOR_D3REG;
	else if (!strcmp(name, "voltage_lpreg"))
		reg_id = PCF50606_REGULATOR_LPREG;
	else if (!strcmp(name, "voltage_ioreg"))
		reg_id = PCF50606_REGULATOR_IOREG;
	else
		reg_id = -1;

	return reg_id;
}

static ssize_t show_vreg(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50606_data *pcf = i2c_get_clientdata(client);
	unsigned int reg_id;

	reg_id = reg_id_by_name(attr->attr.name);
	if (reg_id < 0)
		return 0;

	if (pcf50606_onoff_get(pcf, reg_id) > 0)
		return sprintf(buf, "%u\n", pcf50606_voltage_get(pcf, reg_id));
	else
		return strlcpy(buf, "0\n", PAGE_SIZE);
}

static ssize_t set_vreg(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50606_data *pcf = i2c_get_clientdata(client);
	unsigned long mvolts = simple_strtoul(buf, NULL, 10);
	unsigned int reg_id;

	reg_id = reg_id_by_name(attr->attr.name);
	if (reg_id < 0)
		return -EIO;

	dev_dbg(dev, "attempting to set %s(%d) to %lu mvolts\n",
		attr->attr.name, reg_id, mvolts);

	if (mvolts == 0) {
		pcf50606_onoff_set(pcf, reg_id, 0);
	} else {
		if (pcf50606_voltage_set(pcf, reg_id, mvolts) < 0) {
			dev_warn(dev, "refusing to set %s(%d) to %lu mvolts "
				 "(max=%u)\n", attr->attr.name, reg_id, mvolts,
				 pcf->pdata->rails[reg_id].voltage.max);
			return -EINVAL;
		}
		pcf50606_onoff_set(pcf, reg_id, 1);
	}

	return count;
}

static DEVICE_ATTR(voltage_dcd, S_IRUGO | S_IWUSR, show_vreg, set_vreg);
static DEVICE_ATTR(voltage_dcde, S_IRUGO | S_IWUSR, show_vreg, set_vreg);
static DEVICE_ATTR(voltage_dcud, S_IRUGO | S_IWUSR, show_vreg, set_vreg);
static DEVICE_ATTR(voltage_d1reg, S_IRUGO | S_IWUSR, show_vreg, set_vreg);
static DEVICE_ATTR(voltage_d2reg, S_IRUGO | S_IWUSR, show_vreg, set_vreg);
static DEVICE_ATTR(voltage_d3reg, S_IRUGO | S_IWUSR, show_vreg, set_vreg);
static DEVICE_ATTR(voltage_lpreg, S_IRUGO | S_IWUSR, show_vreg, set_vreg);
static DEVICE_ATTR(voltage_ioreg, S_IRUGO | S_IWUSR, show_vreg, set_vreg);

/***********************************************************************
 * Charger Control
 ***********************************************************************/

/* Enable/disable fast charging (500mA in the GTA01) */
void pcf50606_charge_fast(struct pcf50606_data *pcf, int on)
{
	if (!(pcf->pdata->used_features & PCF50606_FEAT_MBC))
		return;

	if (on) {
		/* We can allow PCF to automatically charge
		 * using Ifast */
		pcf->flags |= PCF50606_F_CHG_FAST;
		reg_set_bit_mask(pcf, PCF50606_REG_MBCC1,
				 PCF50606_MBCC1_AUTOFST,
				 PCF50606_MBCC1_AUTOFST);
	} else {
		pcf->flags &= ~PCF50606_F_CHG_FAST;
		/* disable automatic fast-charge */
		reg_clear_bits(pcf, PCF50606_REG_MBCC1,
				PCF50606_MBCC1_AUTOFST);
		/* switch to idle mode to abort existing charge
		 * process */
		reg_set_bit_mask(pcf, PCF50606_REG_MBCC1,
				 PCF50606_MBCC1_CHGMOD_MASK,
				 PCF50606_MBCC1_CHGMOD_IDLE);
	}
}
EXPORT_SYMBOL_GPL(pcf50606_charge_fast);

static inline u_int16_t adc_to_rntc(struct pcf50606_data *pcf, u_int16_t adc)
{
	u_int32_t r_ntc = (adc * (u_int32_t)pcf->pdata->r_fix_batt)
		/ (1023 - adc);

	return r_ntc;
}

static inline int16_t rntc_to_temp(u_int16_t rntc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ntc_table_10k_3370B); i++) {
		if (rntc > ntc_table_10k_3370B[i])
			return i - 10;	/* First element is -10 */
	}
	return -99;	/* Below our range */
}

static ssize_t show_battemp(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50606_data *pcf = i2c_get_clientdata(client);
	u_int16_t adc;

	adc = adc_read(pcf, PCF50606_ADCMUX_BATTEMP, NULL);

	return sprintf(buf, "%d\n", rntc_to_temp(adc_to_rntc(pcf, adc)));
}
static DEVICE_ATTR(battemp, S_IRUGO | S_IWUSR, show_battemp, NULL);

static inline int16_t adc_to_chg_milliamps(struct pcf50606_data *pcf,
					     u_int16_t adc_adcin1,
					     u_int16_t adc_batvolt)
{
	int32_t res = (adc_adcin1 - adc_batvolt) * 2400;
	return (res * 1000) / (pcf->pdata->r_sense_milli * 1024);
}

static ssize_t show_chgcur(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50606_data *pcf = i2c_get_clientdata(client);
	u_int16_t adc_batvolt, adc_adcin1;
	int16_t ma;

	adc_batvolt = adc_read(pcf, PCF50606_ADCMUX_BATVOLT_ADCIN1,
			       &adc_adcin1);
	ma = adc_to_chg_milliamps(pcf, adc_adcin1, adc_batvolt);

	return sprintf(buf, "%d\n", ma);
}
static DEVICE_ATTR(chgcur, S_IRUGO | S_IWUSR, show_chgcur, NULL);

static const char *chgmode_names[] = {
	[PCF50606_MBCC1_CHGMOD_QUAL]		= "qualification",
	[PCF50606_MBCC1_CHGMOD_PRE]		= "pre",
	[PCF50606_MBCC1_CHGMOD_TRICKLE]		= "trickle",
	[PCF50606_MBCC1_CHGMOD_FAST_CCCV]	= "fast_cccv",
	[PCF50606_MBCC1_CHGMOD_FAST_NOCC]	= "fast_nocc",
	[PCF50606_MBCC1_CHGMOD_FAST_NOCV]	= "fast_nocv",
	[PCF50606_MBCC1_CHGMOD_FAST_SW]		= "fast_switch",
	[PCF50606_MBCC1_CHGMOD_IDLE]		= "idle",
};

static ssize_t show_chgmode(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50606_data *pcf = i2c_get_clientdata(client);
	u_int8_t mbcc1 = reg_read(pcf, PCF50606_REG_MBCC1);
	u_int8_t chgmod = (mbcc1 & PCF50606_MBCC1_CHGMOD_MASK);

	return sprintf(buf, "%s\n", chgmode_names[chgmod]);
}

static ssize_t set_chgmode(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50606_data *pcf = i2c_get_clientdata(client);
	u_int8_t mbcc1 = reg_read(pcf, PCF50606_REG_MBCC1);

	mbcc1 &= ~PCF50606_MBCC1_CHGMOD_MASK;

	if (!strcmp(buf, "qualification"))
		mbcc1 |= PCF50606_MBCC1_CHGMOD_QUAL;
	else if (!strcmp(buf, "pre"))
		mbcc1 |= PCF50606_MBCC1_CHGMOD_PRE;
	else if (!strcmp(buf, "trickle"))
		mbcc1 |= PCF50606_MBCC1_CHGMOD_TRICKLE;
	else if (!strcmp(buf, "fast_cccv"))
		mbcc1 |= PCF50606_MBCC1_CHGMOD_FAST_CCCV;
	/* We don't allow the other fast modes for security reasons */
	else if (!strcmp(buf, "idle"))
		mbcc1 |= PCF50606_MBCC1_CHGMOD_IDLE;
	else
		return -EINVAL;

	reg_write(pcf, PCF50606_REG_MBCC1, mbcc1);

	return count;
}

static DEVICE_ATTR(chgmode, S_IRUGO | S_IWUSR, show_chgmode, set_chgmode);

static const char *chgstate_names[] = {
	[PCF50606_B_CHG_FAST]			= "fast_enabled",
	[PCF50606_B_CHG_PRESENT] 		= "present",
	[PCF50606_B_CHG_FOK]			= "fast_ok",
	[PCF50606_B_CHG_ERR]			= "error",
	[PCF50606_B_CHG_PROT]			= "protection",
	[PCF50606_B_CHG_READY]			= "ready",
};

static ssize_t show_chgstate(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50606_data *pcf = i2c_get_clientdata(client);
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

static void pcf50606_get_power_status(struct apm_power_info *info)
{
	struct pcf50606_data *pcf = pcf50606_global;
	u_int8_t mbcc1 = reg_read(pcf, PCF50606_REG_MBCC1);
	u_int8_t chgmod = mbcc1 & PCF50606_MBCC1_CHGMOD_MASK;
	u_int16_t battvolt = pcf50606_battvolt(pcf);

	if (reg_read(pcf, PCF50606_REG_OOCS) & PCF50606_OOCS_EXTON)
		info->ac_line_status = APM_AC_ONLINE;
	else
		info->ac_line_status = APM_AC_OFFLINE;

	switch (chgmod) {
	case PCF50606_MBCC1_CHGMOD_QUAL:
	case PCF50606_MBCC1_CHGMOD_PRE:
	case PCF50606_MBCC1_CHGMOD_IDLE:
		info->battery_life = battvolt_scale(battvolt);
		break;
	default:
		info->battery_status = APM_BATTERY_STATUS_CHARGING;
		info->battery_flag = APM_BATTERY_FLAG_CHARGING;
		break;
	}
}

/***********************************************************************
 * RTC
 ***********************************************************************/

struct pcf50606_time {
	u_int8_t sec;
	u_int8_t min;
	u_int8_t hour;
	u_int8_t wkday;
	u_int8_t day;
	u_int8_t month;
	u_int8_t year;
};

static void pcf2rtc_time(struct rtc_time *rtc, struct pcf50606_time *pcf)
{
	rtc->tm_sec = BCD2BIN(pcf->sec);
	rtc->tm_min = BCD2BIN(pcf->min);
	rtc->tm_hour = BCD2BIN(pcf->hour);
	rtc->tm_wday = BCD2BIN(pcf->wkday);
	rtc->tm_mday = BCD2BIN(pcf->day);
	rtc->tm_mon = BCD2BIN(pcf->month);
	rtc->tm_year = BCD2BIN(pcf->year) + 100;
}

static void rtc2pcf_time(struct pcf50606_time *pcf, struct rtc_time *rtc)
{
	pcf->sec = BIN2BCD(rtc->tm_sec);
	pcf->min = BIN2BCD(rtc->tm_min);
	pcf->hour = BIN2BCD(rtc->tm_hour);
	pcf->wkday = BIN2BCD(rtc->tm_wday);
	pcf->day = BIN2BCD(rtc->tm_mday);
	pcf->month = BIN2BCD(rtc->tm_mon);
	pcf->year = BIN2BCD(rtc->tm_year - 100);
}

static int pcf50606_rtc_ioctl(struct device *dev, unsigned int cmd,
			      unsigned long arg)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50606_data *pcf = i2c_get_clientdata(client);

	switch (cmd) {
	case RTC_AIE_OFF:
		/* disable the alarm interrupt */
		reg_set_bit_mask(pcf, PCF50606_REG_INT1M,
				 PCF50606_INT1_ALARM, PCF50606_INT1_ALARM);
		return 0;
	case RTC_AIE_ON:
		/* enable the alarm interrupt */
		reg_clear_bits(pcf, PCF50606_REG_INT1M, PCF50606_INT1_ALARM);
		return 0;
	case RTC_PIE_OFF:
		/* disable periodic interrupt (hz tick) */
		pcf->flags &= ~PCF50606_F_RTC_SECOND;
		reg_set_bit_mask(pcf, PCF50606_REG_INT1M,
				 PCF50606_INT1_SECOND, PCF50606_INT1_SECOND);
		return 0;
	case RTC_PIE_ON:
		/* ensable periodic interrupt (hz tick) */
		pcf->flags |= PCF50606_F_RTC_SECOND;
		reg_clear_bits(pcf, PCF50606_REG_INT1M, PCF50606_INT1_SECOND);
		return 0;
	}
	return -ENOIOCTLCMD;
}

static int pcf50606_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50606_data *pcf = i2c_get_clientdata(client);
	struct pcf50606_time pcf_tm;

	mutex_lock(&pcf->lock);
	pcf_tm.sec = __reg_read(pcf, PCF50606_REG_RTCSC);
	pcf_tm.min = __reg_read(pcf, PCF50606_REG_RTCMN);
	pcf_tm.hour = __reg_read(pcf, PCF50606_REG_RTCHR);
	pcf_tm.wkday = __reg_read(pcf, PCF50606_REG_RTCWD);
	pcf_tm.day = __reg_read(pcf, PCF50606_REG_RTCDT);
	pcf_tm.month = __reg_read(pcf, PCF50606_REG_RTCMT);
	pcf_tm.year = __reg_read(pcf, PCF50606_REG_RTCYR);
	mutex_unlock(&pcf->lock);

	dev_dbg(dev, "PCF_TIME: %02x.%02x.%02x %02x:%02x:%02x\n",
		pcf_tm.day, pcf_tm.month, pcf_tm.year,
		pcf_tm.hour, pcf_tm.min, pcf_tm.sec);

	pcf2rtc_time(tm, &pcf_tm);

	dev_dbg(dev, "RTC_TIME: %u.%u.%u %u:%u:%u\n",
		tm->tm_mday, tm->tm_mon, tm->tm_year,
		tm->tm_hour, tm->tm_min, tm->tm_sec);

	return 0;
}

static int pcf50606_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50606_data *pcf = i2c_get_clientdata(client);
	struct pcf50606_time pcf_tm;
	u_int8_t int1m;

	dev_dbg(dev, "RTC_TIME: %u.%u.%u %u:%u:%u\n",
		tm->tm_mday, tm->tm_mon, tm->tm_year,
		tm->tm_hour, tm->tm_min, tm->tm_sec);
	rtc2pcf_time(&pcf_tm, tm);
	dev_dbg(dev, "PCF_TIME: %02x.%02x.%02x %02x:%02x:%02x\n",
		pcf_tm.day, pcf_tm.month, pcf_tm.year,
		pcf_tm.hour, pcf_tm.min, pcf_tm.sec);

	mutex_lock(&pcf->lock);

	/* disable SECOND interrupt */
	int1m = __reg_read(pcf, PCF50606_REG_INT1M);
	__reg_write(pcf, PCF50606_REG_INT1M, int1m | PCF50606_INT1_SECOND);

	__reg_write(pcf, PCF50606_REG_RTCSC, pcf_tm.sec);
	__reg_write(pcf, PCF50606_REG_RTCMN, pcf_tm.min);
	__reg_write(pcf, PCF50606_REG_RTCHR, pcf_tm.hour);
	__reg_write(pcf, PCF50606_REG_RTCWD, pcf_tm.wkday);
	__reg_write(pcf, PCF50606_REG_RTCDT, pcf_tm.day);
	__reg_write(pcf, PCF50606_REG_RTCMT, pcf_tm.month);
	__reg_write(pcf, PCF50606_REG_RTCYR, pcf_tm.year);

	/* restore INT1M, potentially re-enable SECOND interrupt */
	__reg_write(pcf, PCF50606_REG_INT1M, int1m);

	mutex_unlock(&pcf->lock);

	return 0;
}

static int pcf50606_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50606_data *pcf = i2c_get_clientdata(client);
	struct pcf50606_time pcf_tm;

	mutex_lock(&pcf->lock);
	alrm->enabled =
		__reg_read(pcf, PCF50606_REG_INT1M) & PCF50606_INT1_ALARM
		? 0 : 1;
	pcf_tm.sec = __reg_read(pcf, PCF50606_REG_RTCSCA);
	pcf_tm.min = __reg_read(pcf, PCF50606_REG_RTCMNA);
	pcf_tm.hour = __reg_read(pcf, PCF50606_REG_RTCHRA);
	pcf_tm.wkday = __reg_read(pcf, PCF50606_REG_RTCWDA);
	pcf_tm.day = __reg_read(pcf, PCF50606_REG_RTCDTA);
	pcf_tm.month = __reg_read(pcf, PCF50606_REG_RTCMTA);
	pcf_tm.year = __reg_read(pcf, PCF50606_REG_RTCYRA);
	mutex_unlock(&pcf->lock);

	pcf2rtc_time(&alrm->time, &pcf_tm);

	return 0;
}

static int pcf50606_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50606_data *pcf = i2c_get_clientdata(client);
	struct pcf50606_time pcf_tm;
	u_int8_t irqmask;

	rtc2pcf_time(&pcf_tm, &alrm->time);

	mutex_lock(&pcf->lock);

	/* disable alarm interrupt */
	irqmask = __reg_read(pcf, PCF50606_REG_INT1M);
	irqmask |= PCF50606_INT1_ALARM;
	__reg_write(pcf, PCF50606_REG_INT1M, irqmask);

	__reg_write(pcf, PCF50606_REG_RTCSCA, pcf_tm.sec);
	__reg_write(pcf, PCF50606_REG_RTCMNA, pcf_tm.min);
	__reg_write(pcf, PCF50606_REG_RTCHRA, pcf_tm.hour);
	__reg_write(pcf, PCF50606_REG_RTCWDA, pcf_tm.wkday);
	__reg_write(pcf, PCF50606_REG_RTCDTA, pcf_tm.day);
	__reg_write(pcf, PCF50606_REG_RTCMTA, pcf_tm.month);
	__reg_write(pcf, PCF50606_REG_RTCYRA, pcf_tm.year);

	if (alrm->enabled) {
		/* (re-)enaable alarm interrupt */
		irqmask = __reg_read(pcf, PCF50606_REG_INT1M);
		irqmask &= ~PCF50606_INT1_ALARM;
		__reg_write(pcf, PCF50606_REG_INT1M, irqmask);
	}

	mutex_unlock(&pcf->lock);

	/* FIXME */
	return 0;
}

static struct rtc_class_ops pcf50606_rtc_ops = {
	.ioctl		= pcf50606_rtc_ioctl,
	.read_time	= pcf50606_rtc_read_time,
	.set_time	= pcf50606_rtc_set_time,
	.read_alarm	= pcf50606_rtc_read_alarm,
	.set_alarm	= pcf50606_rtc_set_alarm,
};

/***********************************************************************
 * Watchdog
 ***********************************************************************/

static void pcf50606_wdt_start(struct pcf50606_data *pcf)
{
	reg_set_bit_mask(pcf, PCF50606_REG_OOCC1, PCF50606_OOCC1_WDTRST,
			 PCF50606_OOCC1_WDTRST);
}

static void pcf50606_wdt_stop(struct pcf50606_data *pcf)
{
	reg_clear_bits(pcf, PCF50606_REG_OOCS, PCF50606_OOCS_WDTEXP);
}

static void pcf50606_wdt_keepalive(struct pcf50606_data *pcf)
{
	pcf50606_wdt_start(pcf);
}

static int pcf50606_wdt_open(struct inode *inode, struct file *file)
{
	struct pcf50606_data *pcf = pcf50606_global;

	file->private_data = pcf;

	/* start the timer */
	pcf50606_wdt_start(pcf);

	return nonseekable_open(inode, file);
}

static int pcf50606_wdt_release(struct inode *inode, struct file *file)
{
	struct pcf50606_data *pcf = file->private_data;

	if (pcf->allow_close == CLOSE_STATE_ALLOW)
		pcf50606_wdt_stop(pcf);
	else {
		printk(KERN_CRIT "Unexpected close, not stopping watchdog!\n");
		pcf50606_wdt_keepalive(pcf);
	}

	pcf->allow_close = CLOSE_STATE_NOT;

	return 0;
}

static ssize_t pcf50606_wdt_write(struct file *file, const char __user *data,
				  size_t len, loff_t *ppos)
{
	struct pcf50606_data *pcf = file->private_data;
	if (len) {
		size_t i;

		for (i = 0; i != len; i++) {
			char c;
			if (get_user(c, data + i))
				return -EFAULT;
			if (c == 'V')
				pcf->allow_close = CLOSE_STATE_ALLOW;
		}
		pcf50606_wdt_keepalive(pcf);
	}

	return len;
}

static struct watchdog_info pcf50606_wdt_ident = {
	.options	= WDIOF_MAGICCLOSE,
	.firmware_version = 0,
	.identity	= "PCF50606 Watchdog",
};

static int pcf50606_wdt_ioctl(struct inode *inode, struct file *file,
			      unsigned int cmd, unsigned long arg)
{
	struct pcf50606_data *pcf = file->private_data;
	void __user *argp = (void __user *)arg;
	int __user *p = argp;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &pcf50606_wdt_ident,
				    sizeof(pcf50606_wdt_ident)) ? -EFAULT : 0;
		break;
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);
	case WDIOC_KEEPALIVE:
		pcf50606_wdt_keepalive(pcf);
		return 0;
	case WDIOC_GETTIMEOUT:
		return put_user(8, p);
	default:
		return -ENOIOCTLCMD;
	}
}

static struct file_operations pcf50606_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= &pcf50606_wdt_write,
	.ioctl		= &pcf50606_wdt_ioctl,
	.open		= &pcf50606_wdt_open,
	.release	= &pcf50606_wdt_release,
};

static struct miscdevice pcf50606_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &pcf50606_wdt_fops,
};

/***********************************************************************
 * PWM
 ***********************************************************************/

static const char *pwm_dc_table[] = {
	"0/16", "1/16", "2/16", "3/16",
	"4/16", "5/16", "6/16", "7/16",
	"8/16", "9/16", "10/16", "11/16",
	"12/16", "13/16", "14/16", "15/16",
};

static ssize_t show_pwm_dc(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50606_data *pcf = i2c_get_clientdata(client);
	u_int8_t val;

	val = reg_read(pcf, PCF50606_REG_PWMC1) >> PCF50606_PWMC1_DC_SHIFT;
	val &= 0xf;

	return sprintf(buf, "%s\n", pwm_dc_table[val]);
}

static ssize_t set_pwm_dc(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50606_data *pcf = i2c_get_clientdata(client);
	u_int8_t i;

	for (i = 0; i < ARRAY_SIZE(pwm_dc_table); i++) {
		if (!strncmp(buf, pwm_dc_table[i], strlen(pwm_dc_table[i]))) {
			dev_dbg(dev, "setting pwm dc %s\n\r", pwm_dc_table[i]);
			reg_set_bit_mask(pcf, PCF50606_REG_PWMC1, 0x1e,
					 (i << PCF50606_PWMC1_DC_SHIFT));
		}
	}
	return count;
}

static DEVICE_ATTR(pwm_dc, S_IRUGO | S_IWUSR, show_pwm_dc, set_pwm_dc);

static const char *pwm_clk_table[] = {
	"512", "256", "128", "64",
	"56300", "28100", "14100", "7000",
};

static ssize_t show_pwm_clk(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50606_data *pcf = i2c_get_clientdata(client);
	u_int8_t val;

	val = reg_read(pcf, PCF50606_REG_PWMC1) >> PCF50606_PWMC1_CLK_SHIFT;
	val &= 0x7;

	return sprintf(buf, "%s\n", pwm_clk_table[val]);
}

static ssize_t set_pwm_clk(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50606_data *pcf = i2c_get_clientdata(client);
	u_int8_t i;

	for (i = 0; i < ARRAY_SIZE(pwm_clk_table); i++) {
		if (!strncmp(buf, pwm_clk_table[i], strlen(pwm_clk_table[i]))) {
			dev_dbg(dev, "setting pwm clk %s\n\r",
				pwm_clk_table[i]);
			reg_set_bit_mask(pcf, PCF50606_REG_PWMC1, 0xe0,
					 (i << PCF50606_PWMC1_CLK_SHIFT));
		}
	}
	return count;
}

static DEVICE_ATTR(pwm_clk, S_IRUGO | S_IWUSR, show_pwm_clk, set_pwm_clk);

static int pcf50606bl_get_intensity(struct backlight_device *bd)
{
	struct pcf50606_data *pcf = bl_get_data(bd);
	int intensity = reg_read(pcf, PCF50606_REG_PWMC1);
	intensity = (intensity >> PCF50606_PWMC1_DC_SHIFT);

	return intensity & 0xf;
}

static int pcf50606bl_set_intensity(struct backlight_device *bd)
{
	struct pcf50606_data *pcf = bl_get_data(bd);
	int intensity = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;

	return reg_set_bit_mask(pcf, PCF50606_REG_PWMC1, 0x1e,
				(intensity << PCF50606_PWMC1_DC_SHIFT));
}

static struct backlight_ops pcf50606bl_ops = {
	.get_brightness	= pcf50606bl_get_intensity,
	.update_status	= pcf50606bl_set_intensity,
};

/***********************************************************************
 * Driver initialization
 ***********************************************************************/

#ifdef CONFIG_MACH_NEO1973_GTA01
/* We currently place those platform devices here to make sure the device
 * suspend/resume order is correct */
static struct platform_device gta01_pm_gps_dev = {
	.name		= "neo1973-pm-gps",
};

static struct platform_device gta01_pm_bt_dev = {
	.name		= "neo1973-pm-bt",
};
#endif

static struct attribute *pcf_sysfs_entries[16] = {
	&dev_attr_voltage_dcd.attr,
	&dev_attr_voltage_dcde.attr,
	&dev_attr_voltage_dcud.attr,
	&dev_attr_voltage_d1reg.attr,
	&dev_attr_voltage_d2reg.attr,
	&dev_attr_voltage_d3reg.attr,
	&dev_attr_voltage_lpreg.attr,
	&dev_attr_voltage_ioreg.attr,
	NULL
};

static struct attribute_group pcf_attr_group = {
	.name	= NULL,			/* put in device directory */
	.attrs	= pcf_sysfs_entries,
};

static void populate_sysfs_group(struct pcf50606_data *pcf)
{
	int i = 0;
	struct attribute **attr;

	for (attr = pcf_sysfs_entries; *attr; attr++)
		i++;

	if (pcf->pdata->used_features & PCF50606_FEAT_MBC) {
		pcf_sysfs_entries[i++] = &dev_attr_chgstate.attr;
		pcf_sysfs_entries[i++] = &dev_attr_chgmode.attr;
	}

	if (pcf->pdata->used_features & PCF50606_FEAT_CHGCUR)
		pcf_sysfs_entries[i++] = &dev_attr_chgcur.attr;

	if (pcf->pdata->used_features & PCF50606_FEAT_BATVOLT)
		pcf_sysfs_entries[i++] = &dev_attr_battvolt.attr;

	if (pcf->pdata->used_features & PCF50606_FEAT_BATTEMP)
		pcf_sysfs_entries[i++] = &dev_attr_battemp.attr;

	if (pcf->pdata->used_features & PCF50606_FEAT_PWM) {
		pcf_sysfs_entries[i++] = &dev_attr_pwm_dc.attr;
		pcf_sysfs_entries[i++] = &dev_attr_pwm_clk.attr;
	}
}

static int pcf50606_detect(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *new_client;
	struct pcf50606_data *data;
	int err = 0;
	int irq;

	if (!pcf50606_pdev) {
		printk(KERN_ERR "pcf50606: driver needs a platform_device!\n");
		return -EIO;
	}

	irq = platform_get_irq(pcf50606_pdev, 0);
	if (irq < 0) {
		dev_err(&pcf50606_pdev->dev, "no irq in platform resources!\n");
		return -EIO;
	}

	/* At the moment, we only support one PCF50606 in a system */
	if (pcf50606_global) {
		dev_err(&pcf50606_pdev->dev,
			"currently only one chip supported\n");
		return -EBUSY;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	mutex_init(&data->lock);
	mutex_init(&data->working_lock);
	INIT_WORK(&data->work, pcf50606_work);
	data->irq = irq;
	data->working = 0;
	data->onkey_seconds = -1;
	data->pdata = pcf50606_pdev->dev.platform_data;

	new_client = &data->client;
	i2c_set_clientdata(new_client, data);
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &pcf50606_driver;
	new_client->flags = 0;
	strlcpy(new_client->name, "pcf50606", I2C_NAME_SIZE);

	/* now we try to detect the chip */

	/* register with i2c core */
	err = i2c_attach_client(new_client);
	if (err) {
		dev_err(&new_client->dev,
			"error during i2c_attach_client()\n");
		goto exit_free;
	}

	populate_sysfs_group(data);

	err = sysfs_create_group(&new_client->dev.kobj, &pcf_attr_group);
	if (err) {
		dev_err(&new_client->dev, "error creating sysfs group\n");
		goto exit_detach;
	}

	/* create virtual charger 'device' */

	/* input device registration */
	data->input_dev = input_allocate_device();
	if (!data->input_dev)
		goto exit_sysfs;

	data->input_dev->name = "FIC Neo1973 PMU events";
	data->input_dev->phys = "I2C";
	data->input_dev->id.bustype = BUS_I2C;
	data->input_dev->cdev.dev = &new_client->dev;

	data->input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_PWR);
	set_bit(KEY_POWER, data->input_dev->keybit);
	set_bit(KEY_POWER2, data->input_dev->keybit);
	set_bit(KEY_BATTERY, data->input_dev->keybit);

	err = input_register_device(data->input_dev);
	if (err)
		goto exit_sysfs;

	/* register power off handler with core power management */
	pm_power_off = &pcf50606_go_standby;

	/* configure interrupt mask */
	/* we don't mask SECOND here, because we want one to do coldplug with */
	reg_write(data, PCF50606_REG_INT1M, 0x00);
	reg_write(data, PCF50606_REG_INT2M, 0x00);
	reg_write(data, PCF50606_REG_INT3M, PCF50606_INT3_TSCPRES);

	err = request_irq(irq, pcf50606_irq, IRQF_TRIGGER_FALLING,
			  "pcf50606", data);
	if (err < 0)
		goto exit_input;

	if (enable_irq_wake(irq) < 0)
		dev_err(&new_client->dev, "IRQ %u cannot be enabled as wake-up"
			"source in this hardware revision!", irq);

	pcf50606_global = data;

	if (data->pdata->used_features & PCF50606_FEAT_RTC) {
		data->rtc = rtc_device_register("pcf50606", &new_client->dev,
						&pcf50606_rtc_ops, THIS_MODULE);
		if (IS_ERR(data->rtc)) {
			err = PTR_ERR(data->rtc);
			goto exit_irq;
		}
	}

	if (data->pdata->used_features & PCF50606_FEAT_WDT) {
		err = misc_register(&pcf50606_wdt_miscdev);
		if (err) {
			dev_err(&new_client->dev, "cannot register miscdev on "
			       "minor=%d (%d)\n", WATCHDOG_MINOR, err);
			goto exit_rtc;
		}
	}

	if (data->pdata->used_features & PCF50606_FEAT_PWM) {
		/* enable PWM controller */
		reg_set_bit_mask(data, PCF50606_REG_PWMC1,
				 PCF50606_PWMC1_ACTSET,
				 PCF50606_PWMC1_ACTSET);
	}

	if (data->pdata->used_features & PCF50606_FEAT_PWM_BL) {
		data->backlight = backlight_device_register("pcf50606-bl",
							    &new_client->dev,
							    data,
							    &pcf50606bl_ops);
		if (!data->backlight)
			goto exit_misc;
		data->backlight->props.max_brightness = 16;
		data->backlight->props.power = FB_BLANK_UNBLANK;
		data->backlight->props.brightness =
					data->pdata->init_brightness;
		backlight_update_status(data->backlight);
	}

	apm_get_power_status = pcf50606_get_power_status;

#ifdef CONFIG_MACH_NEO1973_GTA01
	if (machine_is_neo1973_gta01()) {
		gta01_pm_gps_dev.dev.parent = &new_client->dev;
		switch (system_rev) {
		case GTA01Bv2_SYSTEM_REV:
		case GTA01Bv3_SYSTEM_REV:
		case GTA01Bv4_SYSTEM_REV:
			gta01_pm_bt_dev.dev.parent = &new_client->dev;
			platform_device_register(&gta01_pm_bt_dev);
			break;
		}
		platform_device_register(&gta01_pm_gps_dev);
		/* a link for gllin compatibility */
		err = sysfs_create_link(&platform_bus_type.devices.kobj,
		    &gta01_pm_gps_dev.dev.kobj, "gta01-pm-gps.0");
		if (err)
			printk(KERN_ERR
			    "sysfs_create_link (gta01-pm-gps.0): %d\n", err);
	}
#endif

	if (data->pdata->used_features & PCF50606_FEAT_ACD)
		reg_set_bit_mask(data, PCF50606_REG_ACDC1,
				 PCF50606_ACDC1_ACDAPE, PCF50606_ACDC1_ACDAPE);
	else
		reg_clear_bits(data, PCF50606_REG_ACDC1,
			       PCF50606_ACDC1_ACDAPE);

	return 0;

exit_misc:
	if (data->pdata->used_features & PCF50606_FEAT_WDT)
		misc_deregister(&pcf50606_wdt_miscdev);
exit_rtc:
	if (data->pdata->used_features & PCF50606_FEAT_RTC)
		rtc_device_unregister(pcf50606_global->rtc);
exit_irq:
	free_irq(pcf50606_global->irq, pcf50606_global);
	pcf50606_global = NULL;
exit_input:
	pm_power_off = NULL;
	input_unregister_device(data->input_dev);
exit_sysfs:
	sysfs_remove_group(&new_client->dev.kobj, &pcf_attr_group);
exit_detach:
	i2c_detach_client(new_client);
exit_free:
	kfree(data);
	return err;
}

static int pcf50606_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, &pcf50606_detect);
}

static int pcf50606_detach_client(struct i2c_client *client)
{
	struct pcf50606_data *pcf = i2c_get_clientdata(client);

	apm_get_power_status = NULL;
	input_unregister_device(pcf->input_dev);

	if (pcf->pdata->used_features & PCF50606_FEAT_PWM_BL)
		backlight_device_unregister(pcf->backlight);

	if (pcf->pdata->used_features & PCF50606_FEAT_WDT)
		misc_deregister(&pcf50606_wdt_miscdev);

	if (pcf->pdata->used_features & PCF50606_FEAT_RTC)
		rtc_device_unregister(pcf->rtc);

	free_irq(pcf->irq, pcf);

	sysfs_remove_group(&client->dev.kobj, &pcf_attr_group);

	pm_power_off = NULL;

	kfree(pcf);

	return 0;
}

#ifdef CONFIG_PM
#define INT1M_RESUMERS	(PCF50606_INT1_ALARM | \
			 PCF50606_INT1_ONKEYF | \
			 PCF50606_INT1_EXTONR)
#define INT2M_RESUMERS	(PCF50606_INT2_CHGWD10S | \
			 PCF50606_INT2_CHGPROT | \
			 PCF50606_INT2_CHGERR)
#define INT3M_RESUMERS	(PCF50606_INT3_LOWBAT | \
			 PCF50606_INT3_HIGHTMP | \
			 PCF50606_INT3_ACDINS)
static int pcf50606_suspend(struct device *dev, pm_message_t state)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50606_data *pcf = i2c_get_clientdata(client);
	int i;

	/* we suspend once (!) as late as possible in the suspend sequencing */

	if ((state.event != PM_EVENT_SUSPEND) ||
	    (pcf->suspend_state != PCF50606_SS_RUNNING))
		return -EBUSY;

	/* The general idea is to power down all unused power supplies,
	 * and then mask all PCF50606 interrup sources but EXTONR, ONKEYF
	 * and ALARM */

	mutex_lock(&pcf->lock);

	pcf->suspend_state = PCF50606_SS_STARTING_SUSPEND;

	/* we are not going to service any further interrupts until we
	 * resume.  If the IRQ workqueue is still pending in the background,
	 * it will bail when it sees we set suspend state above.
	 */

	disable_irq(pcf->irq);

	/* Save all registers that don't "survive" standby state */
	pcf->standby_regs.dcdc1 = __reg_read(pcf, PCF50606_REG_DCDC1);
	pcf->standby_regs.dcdc2 = __reg_read(pcf, PCF50606_REG_DCDC2);
	pcf->standby_regs.dcdec1 = __reg_read(pcf, PCF50606_REG_DCDEC1);
	pcf->standby_regs.dcudc1 = __reg_read(pcf, PCF50606_REG_DCUDC1);
	pcf->standby_regs.ioregc = __reg_read(pcf, PCF50606_REG_IOREGC);
	pcf->standby_regs.d1regc1 = __reg_read(pcf, PCF50606_REG_D1REGC1);
	pcf->standby_regs.d2regc1 = __reg_read(pcf, PCF50606_REG_D2REGC1);
	pcf->standby_regs.d3regc1 = __reg_read(pcf, PCF50606_REG_D3REGC1);
	pcf->standby_regs.lpregc1 = __reg_read(pcf, PCF50606_REG_LPREGC1);
	pcf->standby_regs.adcc1 = __reg_read(pcf, PCF50606_REG_ADCC1);
	pcf->standby_regs.adcc2 = __reg_read(pcf, PCF50606_REG_ADCC2);
	pcf->standby_regs.pwmc1 = __reg_read(pcf, PCF50606_REG_PWMC1);

	/* switch off power supplies that are not needed during suspend */
	for (i = 0; i < __NUM_PCF50606_REGULATORS; i++) {
		if (!(pcf->pdata->rails[i].flags & PMU_VRAIL_F_SUSPEND_ON)) {
			u_int8_t tmp;

			/* IOREG powers the I@C interface so we cannot switch
			 * it off */
			if (i == PCF50606_REGULATOR_IOREG)
				continue;

			dev_dbg(dev, "disabling pcf50606 regulator %u\n", i);
			/* we cannot use pcf50606_onoff_set() because we're
			 * already under the mutex */
			tmp = __reg_read(pcf, regulator_registers[i]);
			tmp &= 0x1f;
			__reg_write(pcf, regulator_registers[i], tmp);
		}
	}

	pcf->standby_regs.int1m = __reg_read(pcf, PCF50606_REG_INT1M);
	pcf->standby_regs.int2m = __reg_read(pcf, PCF50606_REG_INT2M);
	pcf->standby_regs.int3m = __reg_read(pcf, PCF50606_REG_INT3M);
	__reg_write(pcf, PCF50606_REG_INT1M, ~INT1M_RESUMERS & 0xff);
	__reg_write(pcf, PCF50606_REG_INT2M, ~INT2M_RESUMERS & 0xff);
	__reg_write(pcf, PCF50606_REG_INT3M, ~INT3M_RESUMERS & 0xff);

	pcf->suspend_state = PCF50606_SS_COMPLETED_SUSPEND;

	mutex_unlock(&pcf->lock);

	return 0;
}

static int pcf50606_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf50606_data *pcf = i2c_get_clientdata(client);

	mutex_lock(&pcf->lock);

	pcf->suspend_state = PCF50606_SS_STARTING_RESUME;

	/* Resume all saved registers that don't "survive" standby state */
	__reg_write(pcf, PCF50606_REG_INT1M, pcf->standby_regs.int1m);
	__reg_write(pcf, PCF50606_REG_INT2M, pcf->standby_regs.int2m);
	__reg_write(pcf, PCF50606_REG_INT3M, pcf->standby_regs.int3m);

	__reg_write(pcf, PCF50606_REG_DCDC1, pcf->standby_regs.dcdc1);
	__reg_write(pcf, PCF50606_REG_DCDC2, pcf->standby_regs.dcdc2);
	__reg_write(pcf, PCF50606_REG_DCDEC1, pcf->standby_regs.dcdec1);
	__reg_write(pcf, PCF50606_REG_DCUDC1, pcf->standby_regs.dcudc1);
	__reg_write(pcf, PCF50606_REG_IOREGC, pcf->standby_regs.ioregc);
	__reg_write(pcf, PCF50606_REG_D1REGC1, pcf->standby_regs.d1regc1);
	__reg_write(pcf, PCF50606_REG_D2REGC1, pcf->standby_regs.d2regc1);
	__reg_write(pcf, PCF50606_REG_D3REGC1, pcf->standby_regs.d3regc1);
	__reg_write(pcf, PCF50606_REG_LPREGC1, pcf->standby_regs.lpregc1);
	__reg_write(pcf, PCF50606_REG_ADCC1, pcf->standby_regs.adcc1);
	__reg_write(pcf, PCF50606_REG_ADCC2, pcf->standby_regs.adcc2);
	__reg_write(pcf, PCF50606_REG_PWMC1, pcf->standby_regs.pwmc1);

	pcf->suspend_state = PCF50606_SS_COMPLETED_RESUME;

	enable_irq(pcf->irq);

	mutex_unlock(&pcf->lock);

	/* Call PCF work function; this fixes an issue on the gta01 where
	 * the power button "goes away" if it is used to wake the device.
	 */
	get_device(&pcf->client.dev);
	pcf50606_work(&pcf->work);

	return 0;
}
#else
#define pcf50606_suspend NULL
#define pcf50606_resume NULL
#endif

static struct i2c_driver pcf50606_driver = {
	.driver = {
		.name	 = "pcf50606",
		.suspend = pcf50606_suspend,
		.resume	 = pcf50606_resume,
	},
	.id		= I2C_DRIVERID_PCF50606,
	.attach_adapter	= pcf50606_attach_adapter,
	.detach_client	= pcf50606_detach_client,
};

/* platform driver, since i2c devices don't have platform_data */
static int __init pcf50606_plat_probe(struct platform_device *pdev)
{
	struct pcf50606_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata)
		return -ENODEV;

	pcf50606_pdev = pdev;

	return 0;
}

static int pcf50606_plat_remove(struct platform_device *pdev)
{
	return 0;
}

/* We have this purely to capture an early indication that we are coming out
 * of suspend, before our device resume got called; async interrupt service is
 * interested in this.
 */

static int pcf50606_plat_resume(struct platform_device *pdev)
{
	/* i2c_get_clientdata(to_i2c_client(&pdev->dev)) returns NULL at this
	 * early resume time so we have to use pcf50606_global
	 */
	pcf50606_global->suspend_state = PCF50606_SS_RESUMING_BUT_NOT_US_YET;

	return 0;
}

static struct platform_driver pcf50606_plat_driver = {
	.probe	= pcf50606_plat_probe,
	.remove	= pcf50606_plat_remove,
	.resume_early = pcf50606_plat_resume,
	.driver = {
		.owner	= THIS_MODULE,
		.name 	= "pcf50606",
	},
};

static int __init pcf50606_init(void)
{
	int rc;

	rc = platform_driver_register(&pcf50606_plat_driver);
	if (!rc)
		rc = i2c_add_driver(&pcf50606_driver);

	return rc;
}

static void pcf50606_exit(void)
{
	i2c_del_driver(&pcf50606_driver);
	platform_driver_unregister(&pcf50606_plat_driver);
}

MODULE_DESCRIPTION("I2C chip driver for NXP PCF50606 power management unit");
MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_LICENSE("GPL");

module_init(pcf50606_init);
module_exit(pcf50606_exit);
