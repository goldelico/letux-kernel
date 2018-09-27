/*
 * BQ24296/7 battery charger and OTG regulator
 *
 * found in some Android driver and hacked by <hns@goldelico.com>
 * to become useable for the Pyra
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#define UNUSED 0
#define FIXME 0

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#define VSYS_REGULATOR	0
#define OTG_REGULATOR	1
#define NUM_REGULATORS	2

/* I2C register define */
#define INPUT_SOURCE_CONTROL_REGISTER		0x00
#define POWER_ON_CONFIGURATION_REGISTER		0x01
#define CHARGE_CURRENT_CONTROL_REGISTER		0x02
#define PRE_CHARGE_TERMINATION_CURRENT_CONTROL_REGISTER		0x03
#define CHARGE_VOLTAGE_CONTROL_REGISTER		0x04
#define TERMINATION_TIMER_CONTROL_REGISTER		0x05
#define THERMAL_REGULATION_CONTROL_REGISTER		0x06
#define MISC_OPERATION_CONTROL_REGISTER		0x07
#define SYSTEM_STATS_REGISTER		0x08
#define FAULT_STATS_REGISTER		0x09
#define VENDOR_STATS_REGISTER		0x0A

/* REG00 input source control register value */

#define EN_HIZ_ENABLE	 1
#define EN_HIZ_DISABLE	 0
#define EN_HIZ_OFFSET	 7
#define EN_HIZ_MASK	 1

#define VINDPM_OFFSET		3
#define VINDPM_MASK		0xf

#define IINLIM_100MA		0
#define IINLIM_150MA		1
#define IINLIM_500MA		2
#define IINLIM_900MA		3
#define IINLIM_1200MA		4
#define IINLIM_1500MA		5
#define IINLIM_2000MA		6
#define IINLIM_3000MA		7
#define IINLIM_OFFSET		0
#define IINLIM_MASK		7

/* REG01 power-on configuration register value */
#define WATCHDOG_RESET	0x40

#define REGISTER_RESET_ENABLE	 1
#define REGISTER_RESET_DISABLE	 0
#define REGISTER_RESET_OFFSET	 7
#define REGISTER_RESET_MASK	 1

/* Charge Mode Config */
#define CHARGE_MODE_CONFIG_CHARGE_DISABLE	0x00
#define CHARGE_MODE_CONFIG_CHARGE_BATTERY	0x01
#define CHARGE_MODE_CONFIG_OTG_OUTPUT		0x02
#define CHARGE_MODE_CONFIG_OFFSET		4
#define CHARGE_MODE_CONFIG_MASK		0x03

#define SYS_MIN_OFFSET		1
#define SYS_MIN_MASK		0x7

/* OTG Mode Current Config */
#define OTG_MODE_CURRENT_CONFIG_500MA		0x00
#define OTG_MODE_CURRENT_CONFIG_1300MA	0x01
#define OTG_MODE_CURRENT_CONFIG_OFFSET	0
#define OTG_MODE_CURRENT_CONFIG_MASK		0x01

/* REG02 charge current limit register value */
#define CHARGE_CURRENT_64MA		0x01
#define CHARGE_CURRENT_128MA		0x02
#define CHARGE_CURRENT_256MA		0x04
#define CHARGE_CURRENT_512MA		0x08
#define CHARGE_CURRENT_1024MA		0x10
#define CHARGE_CURRENT_1536MA		0x18
#define CHARGE_CURRENT_2048MA		0x20
#define CHARGE_CURRENT_OFFSET		2
#define CHARGE_CURRENT_MASK		0x3f

/* REG03 Pre-Charge/Termination Current Control Register value */
/* Pre-Charge Current Limit */
#define PRE_CHARGE_CURRENT_LIMIT_128MA		0x00
#define PRE_CHARGE_CURRENT_LIMIT_256MA		0x01
#define PRE_CHARGE_CURRENT_LIMIT_OFFSET		4
#define PRE_CHARGE_CURRENT_LIMIT_MASK		0x0f
/* Termination Current Limit */
#define TERMINATION_CURRENT_LIMIT_128MA		0x00
#define TERMINATION_CURRENT_LIMIT_256MA		0x01
#define TERMINATION_CURRENT_LIMIT_OFFSET		0
#define TERMINATION_CURRENT_LIMIT_MASK		0x0f

/* REG04 Charge Voltage Register */
#define VREG_MASK	0x3f
#define VREG_OFFSET	2

/* REG05 Charge Termination/Timer control register value */
#define WATCHDOG_DISABLE		0
#define WATCHDOG_40S		1
#define WATCHDOG_80S		2
#define WATCHDOG_160S		3
#define WATCHDOG_OFFSET		4
#define WATCHDOG_MASK		3

/* REG06 boost voltage/thermal regulation register */
#define BOOSTV_OFFSET	4
#define BOOSTV_MASK	0xf

/* REG07 misc operation control register value */
#define DPDM_ENABLE	 1
#define DPDM_DISABLE	 0
#define DPDM_OFFSET	 7
#define DPDM_MASK	 1

/* REG08 system status register value */
#define VBUS_UNKNOWN		0
#define VBUS_USB_HOST		1
#define VBUS_ADAPTER_PORT		2
#define VBUS_OTG		3
#define VBUS_OFFSET		6
#define VBUS_MASK		3

#define CHRG_NO_CHARGING		0
#define CHRG_PRE_CHARGE		1
#define CHRG_FAST_CHARGE		2
#define CHRG_CHRGE_DONE		3
#define CHRG_OFFSET		4
#define CHRG_MASK		3

#define DPM_STAT	0x08
#define PG_STAT		0x04
#define THERM_STAT	0x02
#define VSYS_STAT	0x01

/* REG09 fault status register value */

#define WATCHDOG_FAULT	0x80
#define OTG_FAULT	0x40
#define CHRG_FAULT_OFFSET	4
#define CHRG_FAULT_MASK	0x3
#define BAT_FAULT	0x08
#define NTC_FAULT_OFFSET	0
#define NTC_FAULT_MASK	0x3

/* REG0a vendor status register value */
/* #define CHIP_BQ24190		0
#define CHIP_BQ24191		1
#define CHIP_BQ24192		2
#define CHIP_BQ24192I		3
#define CHIP_BQ24190_DEBUG		4
#define CHIP_BQ24192_DEBUG		5 */
#define CHIP_BQ24296		1
#define CHIP_BQ24297		3
#define CHIP_MP2624		0
#define CHIP_OFFSET		2
#define CHIP_MASK		7

#define BQ24296_CHG_COMPELET       0x03
#define BQ24296_NO_CHG             0x00

#define BQ24296_DC_CHG             0x02
#define BQ24296_USB_CHG            0x01

/* FIXME: consolidate and use pointers instead of global variables */

struct bq24296_board {
	struct gpio_desc *otg_usb_pin;
	struct gpio_desc *dc_det_pin;
	struct gpio_desc *psel_pin;
	unsigned int chg_current[3];
};

struct bq24296_board *bq24296_pdata;

struct bq24296_device_info {
	struct device 		*dev;
	struct delayed_work usb_detect_work;
	struct i2c_client	*client;
	unsigned int interval;
	struct mutex	var_lock;
	struct workqueue_struct	*freezable_work;
	struct work_struct	irq_work;	/* for Charging & VUSB/VADP */
	struct regulator_desc desc[NUM_REGULATORS];
	struct device_node *of_node[NUM_REGULATORS];
	struct regulator_dev *rdev[NUM_REGULATORS];
	struct regulator_init_data *pmic_init_data;

	struct workqueue_struct *workqueue;
	u8 chg_current;
	u8 usb_input_current;
	u8 adp_input_current;
	//struct timer_list timer;
	struct power_supply *usb;
};

struct bq24296_device_info *bq24296_di;

/* should be read from DT properties! */
static unsigned int battery_voltage_max_design_uV = 4200000;	// default
static unsigned int max_VSYS_uV = 5000000;	// "unlimited"

#if 0
#define DBG(x...) printk(KERN_INFO x)
#define DEBUG 1
#else
#define DBG(x...) do { } while (0)
#endif

/*
 * Common code for BQ24296 devices read
 */
static int bq24296_i2c_reg8_read(const struct i2c_client *client, const char reg, char *buf, int count)
{
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msgs[2];
	int ret;
	char reg_buf = reg;

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags;
	msgs[0].len = 1;
	msgs[0].buf = &reg_buf;

	msgs[1].addr = client->addr;
	msgs[1].flags = client->flags | I2C_M_RD;
	msgs[1].len = count;
	msgs[1].buf = (char *)buf;

	ret = i2c_transfer(adap, msgs, 2);

	return (ret == 2)? count : ret;
}

static int bq24296_i2c_reg8_write(const struct i2c_client *client, const char reg, const char *buf, int count)
{
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;
	int ret;
	char *tx_buf = (char *)kmalloc(count + 1, GFP_KERNEL);

	if(!tx_buf)
		return -ENOMEM;
	tx_buf[0] = reg;
	memcpy(tx_buf+1, buf, count);

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = count + 1;
	msg.buf = (char *)tx_buf;

	ret = i2c_transfer(adap, &msg, 1);
	kfree(tx_buf);
	return (ret == 1) ? count : ret;
}

static inline int bq24296_read(struct i2c_client *client, u8 reg, u8 buf[], unsigned len)
{
	int ret;

	ret = bq24296_i2c_reg8_read(client, reg, buf, len);
	return ret;
}

static inline int bq24296_write(struct i2c_client *client, u8 reg, u8 const buf[], unsigned len)
{
	int ret;

#if 0	// debug and disable write commands
	printk("bq24296_write %02x: %02x\n", reg, *buf);
	return 0;
#endif
	ret = bq24296_i2c_reg8_write(client, reg, buf, (int)len);
	return ret;
}

static int bq24296_update_reg(struct i2c_client *client, int reg, u8 value, u8 mask )
{
	int ret =0;
	u8 retval = 0;

	ret = bq24296_read(client, reg, &retval, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}
	ret = 0;

#if 0
	printk("bq24296_update_reg %02x: ( %02x & %02x ) | %02x -> %02x\n", reg, retval, (u8) ~mask, value, (u8) ((retval & ~mask) | value));
#endif

	if ((retval & mask) != value) {
		retval = (retval & ~mask) | value;
		ret = bq24296_write(client, reg, &retval, 1);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
		}
		ret = 0;
	}
#if 0	// DEBUG
{
	int i;
	u8 buffer;
	for(i=0;i<11;i++)
		{
		bq24296_read(bq24296_di->client, i, &buffer, 1);
		printk("  reg %02x value %02x\n", i, buffer);
		}
}
#endif

	return ret;
}

/* sysfs tool to show all register values */

static ssize_t show_registers(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i;
	u8 buffer;
	struct bq24296_device_info *di=bq24296_di;
	int len = 0;

	for(i=0;i<11;i++)
		{
		int n;
		bq24296_read(di->client,i,&buffer,1);
		n=scnprintf(buf, 256, "reg %02x value %02x\n", i, buffer);
		buf += n;
		len += n;
		}
	return len;
}

DEVICE_ATTR(registers, 0444, show_registers,NULL);

// FIXME: review very critical what we need to initialize
// and why it is constant and not defined by device tree properties!

static int bq24296_init_registers(void)
{
	int ret = 0;
	int max_uV, bits;

#if 0	// NO: don't do that because we are powered through this chip - u-boot must have initialized properly
	/* reset the configuration register */
	 ret = bq24296_update_reg(bq24296_di->client,
				 POWER_ON_CONFIGURATION_REGISTER,
				 REGISTER_RESET_ENABLE << REGISTER_RESET_OFFSET,
				 REGISTER_RESET_MASK << REGISTER_RESET_OFFSET);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s(): Failed to reset the register \n",
		__func__);
	goto final;
	}

	mdelay(5);	/* maybe we should poll */
#endif

/*
 * FIXME: why do we disable the watchdog?
 *
 * U-Boot has it already disabled because it can't poll the chip if waiting for commands
 * on the command-line and while Linux starts
 *
 * But in Linux we should better re-enable it and reset the watchdog by our polling function
 * Make it a DT property!
 *
 * Probably U-Boot should re-enable it before launching Linux so it becomes active
 * even if Linux does not properly boot
 *
 * So we should only restart if every now and then
 */

#if 0
	/* Disable the watchdog */
	ret = bq24296_update_reg(bq24296_di->client,
				  TERMINATION_TIMER_CONTROL_REGISTER,
				  WATCHDOG_DISABLE << WATCHDOG_OFFSET,
				  WATCHDOG_MASK << WATCHDOG_OFFSET);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s(): Failed to disable the watchdog \n",
				__func__);
		goto final;
	}
#endif

	/* Set Pre-Charge Current Limit as 128mA */
	ret = bq24296_update_reg(bq24296_di->client,
				  PRE_CHARGE_TERMINATION_CURRENT_CONTROL_REGISTER,
				  PRE_CHARGE_CURRENT_LIMIT_128MA << PRE_CHARGE_CURRENT_LIMIT_OFFSET,
				  PRE_CHARGE_CURRENT_LIMIT_MASK << PRE_CHARGE_CURRENT_LIMIT_OFFSET);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s(): Failed to set pre-charge limit 128mA\n",
				__func__);
		goto final;
	}

	/* Set Termination Current Limit as 128mA */
	ret = bq24296_update_reg(bq24296_di->client,
				  PRE_CHARGE_TERMINATION_CURRENT_CONTROL_REGISTER,
				  TERMINATION_CURRENT_LIMIT_128MA << TERMINATION_CURRENT_LIMIT_OFFSET,
				  TERMINATION_CURRENT_LIMIT_MASK << TERMINATION_CURRENT_LIMIT_OFFSET);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s(): Failed to set termination limit 128mA\n",
				__func__);
		goto final;
	}

	// VSYS may be 150mV above fully charged battery voltage
	// so to effectively limit VSYS we may have to lower the max. battery voltage
	max_uV = min(max_VSYS_uV - 150000, battery_voltage_max_design_uV);

	bits = (max_uV - 3504000) / 16000;
	bits = max(bits, 0);
	bits = min(bits, 63);

	dev_info(&bq24296_di->client->dev, "%s(): translated vbatt_max=%u and VSYS_max=%u to VREG=%u (%02x)\n",
		__func__,
		battery_voltage_max_design_uV, max_VSYS_uV, max_uV,
		bits);

	ret = bq24296_update_reg(bq24296_di->client,
				  CHARGE_VOLTAGE_CONTROL_REGISTER,
				  bits << VREG_OFFSET,
				  VREG_MASK << VREG_OFFSET);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s(): Failed to set max. battery voltage\n",
				__func__);
		goto final;
	}

#if 0
	/* Set System Voltage Limit as 3.2V */
	ret = bq24296_update_reg(bq24296_di->client,
				  POWER_ON_CONFIGURATION_REGISTER,
				  0x04,	/* 3.0V + 0.2V */
				  SYS_MIN_MASK << SYS_MIN_OFFSET);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s(): Failed to set voltage limit 3.2V \n",
				__func__);
		goto final;
	}
#endif

#if 0
	/* disable boost temperature protection (for debugging) */
	ret = bq24296_update_reg(bq24296_di->client,
							 THERMAL_REGULATION_CONTROL_REGISTER,
							 0x0c,	/* BHOT[1:0]=11 */
							 0x0c);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s(): Failed to disable boost temperature monitor \n",
				__func__);
		goto final;
	}
#endif

final:
	return ret;
}

/* helper functions */

static int bq24296_limit_current_mA_to_bits(int mA)
{
	u8 data;
	if (mA < 120)
		data = 0;
	else if(mA < 400)
		data = 1;
	else if(mA < 700)
		data = 2;
	else if(mA < 1000)
		data = 3;
	else if(mA < 1200)
		data = 4;
	else if(mA < 1800)
		data = 5;
	else if(mA < 2200)
		data = 6;
	else
		data = 7;
	return data;
}

static int bq24296_chg_current_mA_to_bits(int mA)
{
	u8 data;

	data = (mA)/64;
	data &= 0xff;
	return data;
}

/* getter and setter functions - review critically which ones we still need */

static int bq24296_get_vindpm(void)
{
	int ret;
	u8 retval = 0;

	ret = bq24296_read(bq24296_di->client, INPUT_SOURCE_CONTROL_REGISTER, &retval, 1);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return 3880 + 80*((retval >> VINDPM_OFFSET) & VINDPM_MASK);
}

static const unsigned int iinlim_table[] = {
	100000,
	150000,
	500000,
	900000,
	1000000,
	1500000,
	2000000,
	3000000,
};

static int bq24296_input_current_limit_uA(void)
{
	int cur;	/* in uA */
	int ret;
	u8 retval = 0;

	ret = bq24296_read(bq24296_di->client, INPUT_SOURCE_CONTROL_REGISTER, &retval, 1);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	if(((retval >> EN_HIZ_OFFSET) & EN_HIZ_MASK) == EN_HIZ_ENABLE)
		return 0;	// High-Z state

	return iinlim_table[(retval >> IINLIM_OFFSET) & IINLIM_MASK];
}

static int bq24296_update_input_current_limit(int value)
{
	int ret = 0;
	u8 hiz = (value < 0 ? EN_HIZ_ENABLE : EN_HIZ_DISABLE);

printk("bq24296_update_input_current_limit(%d)\n", value);

	ret = bq24296_update_reg(bq24296_di->client,
				  INPUT_SOURCE_CONTROL_REGISTER,
				  (((value & IINLIM_MASK) << IINLIM_OFFSET) | (hiz << EN_HIZ_OFFSET)),
				  ((IINLIM_MASK << IINLIM_OFFSET) | (EN_HIZ_MASK << EN_HIZ_OFFSET)));
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s(): Failed to set input current limit (0x%x) \n",
				__func__, value);
	}

	return ret;
}

static int bq24296_set_charge_current(u8 value)
{
	int ret = 0;

	ret = bq24296_update_reg(bq24296_di->client,
				  CHARGE_CURRENT_CONTROL_REGISTER,
				  (value << CHARGE_CURRENT_OFFSET) ,(CHARGE_CURRENT_MASK <<CHARGE_CURRENT_OFFSET ));
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s(): Failed to set charge current limit (0x%x) \n",
				__func__, value);
	}
	return ret;
}

static int bq24296_update_en_hiz_disable(void)
{
	int ret = 0;

	ret = bq24296_update_reg(bq24296_di->client,
				  INPUT_SOURCE_CONTROL_REGISTER,
				  EN_HIZ_DISABLE << EN_HIZ_OFFSET,
				  EN_HIZ_MASK << EN_HIZ_OFFSET);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s(): Failed to set en_hiz_disable\n",
				__func__);
	}
	return ret;
}

int bq24296_set_input_current(int on)
{
	if(on) {
		bq24296_update_input_current_limit(IINLIM_3000MA);
	} else {
		bq24296_update_input_current_limit(IINLIM_500MA);
	}
	DBG("bq24296_set_input_current %s\n", on ? "3000mA" : "500mA");

	return 0;
}

static int bq24296_update_charge_mode(u8 value)
{
	int ret = 0;

	ret = bq24296_update_reg(bq24296_di->client,
				  POWER_ON_CONFIGURATION_REGISTER,
				  value << CHARGE_MODE_CONFIG_OFFSET,
				  CHARGE_MODE_CONFIG_MASK << CHARGE_MODE_CONFIG_OFFSET);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s(): Failed to set charge mode(0x%x) \n",
				__func__, value);
	}

	return ret;
}

static int bq24296_update_otg_mode_current(u8 value)
{
	int ret = 0;

	ret = bq24296_update_reg(bq24296_di->client,
				  POWER_ON_CONFIGURATION_REGISTER,
				  value << OTG_MODE_CURRENT_CONFIG_OFFSET,
				  OTG_MODE_CURRENT_CONFIG_MASK << OTG_MODE_CURRENT_CONFIG_OFFSET);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s(): Failed to set otg current mode(0x%x) \n",
				__func__, value);
	}
	return ret;
}

static int bq24296_charge_mode_config(int on)
{
	if (on) {
		bq24296_update_en_hiz_disable();
		mdelay(5);
		bq24296_update_charge_mode(CHARGE_MODE_CONFIG_OTG_OUTPUT);
		mdelay(10);
		bq24296_update_otg_mode_current(OTG_MODE_CURRENT_CONFIG_1300MA);
	}
	else {
		bq24296_update_charge_mode(CHARGE_MODE_CONFIG_CHARGE_BATTERY);
	}

	DBG("bq24296_charge_mode_config is %s\n", on ? "OTG Mode" : "Charge Mode");

	return 0;
}

#if UNUSED
static int bq24296_charge_otg_en(int chg_en,int otg_en)
{ /* control charge/otg mode */
	int ret = 0;

	if ((chg_en ==0) && (otg_en ==0)){
		ret = bq24296_update_reg(bq24296_di->client,POWER_ON_CONFIGURATION_REGISTER,0x00 << 4,0x03 << 4);
	}
	else if ((chg_en ==0) && (otg_en ==1))
		bq24296_charge_mode_config(1);
	else
		bq24296_charge_mode_config(0);
	return ret;
}
#endif

/* this polls the bq2429x to handle VBUS events -- should become interrupt driven */

u8 r8;	// should be read in interrupt handler
u8 r9;	// should be read in interrupt handler because it autoresets

u8 previous_r8 = 0xff;

bool bq24296_battery_present;
bool bq24296_input_present;	// VBUS available

static void usb_detect_work_func(struct work_struct *work)
{
	struct delayed_work *delayed_work = (struct delayed_work *)container_of(work, struct delayed_work, work);
	struct bq24296_device_info *pi = (struct bq24296_device_info *)container_of(delayed_work, struct bq24296_device_info, usb_detect_work);
	int ret ;

	previous_r8 = r8;

//	printk("%s, line=%d\n", __func__,__LINE__);

	ret = bq24296_read(bq24296_di->client, SYSTEM_STATS_REGISTER, &r8, 1);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
		return;	/* don't schedule again */
	}

	ret = bq24296_read(bq24296_di->client, FAULT_STATS_REGISTER, &r9, 1);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
		return;	/* don't schedule again */
	}

	bq24296_battery_present = ((r9 >> NTC_FAULT_OFFSET) & NTC_FAULT_MASK) == 0;	/* if no fault */
	bq24296_input_present = (r8 & PG_STAT) != 0;

	if (r8 != previous_r8 || bq24296_input_present != ((previous_r8 & PG_STAT) != 0))
		DBG("%s: r8 = %02x bq24296_input_present = %d\n", __func__,r8,bq24296_input_present);

#if FIXME

/*
 * we should apply the same logic as U-Boot
 * if no battery available, increase the IINLIM to 2A if below
 * this makes it possible to run from a sufficiently strong USB power supply
 * in all other cases, leave the IINLIM untouched and updated only by udev events
 * or user contro writing to /sys/class/power/bq24297/
 * There is some issue even with this approach:
 * if battery is inserted and system connected to a 2A power supply
 * and then the battery is removed (might happen even during suspend)
 * it may take too long until IINLIM is increased to 2A by code so
 * that the Host stops before it can do anything good. It will then
 * probably reboot and then increase to 2A by U-Boot.
 */

	if (!bq24296_battery_present && bq24296_input_current_limit_uA() < 2000000)
		bq24296_update_input_current_limit(bq24296_limit_current_mA_to_bits(2000000));

#endif

#if FIXME

/*
 * we should also reset the watchdog every now and then
 * at least if we run from battery
 * If watchdog operation is with battery only, we should
 * enable/disable it on demand
 */

#endif

	mutex_lock(&pi->var_lock);

	if (bq24296_pdata->dc_det_pin){
		/* detect charging request */
		ret = gpiod_get_value(bq24296_pdata->dc_det_pin);
		if (ret ==0){
			bq24296_update_input_current_limit(bq24296_di->adp_input_current);
			bq24296_set_charge_current(CHARGE_CURRENT_2048MA);
			bq24296_charge_mode_config(0);
		}
		else {
			bq24296_update_input_current_limit(IINLIM_500MA);
			bq24296_set_charge_current(CHARGE_CURRENT_512MA);
		}
		DBG("%s: bq24296_di->dc_det_pin=%x\n", __func__, ret);
	}
	else {
#ifdef OLD
		DBG("%s: dwc_otg_check_dpdm %d\n", __func__, dwc_otg_check_dpdm(0));
		switch(dwc_otg_check_dpdm(0)) {
			case 2: // USB Wall charger
				bq24296_update_input_current_limit(bq24296_di->usb_input_current);
				bq24296_set_charge_current(CHARGE_CURRENT_2048MA);
				bq24296_charge_mode_config(0);
				DBG("bq24296: detect usb wall charger\n");
				break;
			case 1: //normal USB
				if (0 == get_gadget_connect_flag()){  // non-standard AC charger
					bq24296_update_input_current_limit(bq24296_di->usb_input_current);
					bq24296_set_charge_current(CHARGE_CURRENT_2048MA);
					bq24296_charge_mode_config(0);;
				}else
					{
					// connect to pc
					bq24296_update_input_current_limit(bq24296_di->usb_input_current);
					bq24296_set_charge_current(CHARGE_CURRENT_512MA);
					bq24296_charge_mode_config(0);
					DBG("bq24296: detect normal usb charger\n");
					}
				break;
			default:
				bq24296_update_input_current_limit(IINLIM_500MA);
				bq24296_set_charge_current(CHARGE_CURRENT_512MA);
				DBG("bq24296: detect no usb\n");
				break;
#else

/* FIXME/CHECKME:
   do we really have to actively switch to charging or does the charger start automatically?
   Then, we might not even need this scheduled worker function
*/

		/* detect VBUS availability changes */
		if(ret && bq24296_input_present && !(previous_r8 & PG_STAT)) { /* VBUS became available */
			DBG("bq24296: VBUS became available\n");
			printk("bq24296: VBUS became available\n");
			// this should have been queried/provided by the USB stack...
			bq24296_update_input_current_limit(bq24296_di->usb_input_current);
		/* could trigger another DPDM detection... */
		/* start charging here */
//			bq24296_update_charge_mode(CHARGE_MODE_CONFIG_CHARGE_BATTERY);
		}
		else if(ret && !bq24296_input_present && (previous_r8 & PG_STAT)) { /* VBUS became unavailable */
			DBG("bq24296: VBUS became unavailable\n");
			printk("bq24296: VBUS became unavailable\n");
		}
#endif
	}

	mutex_unlock(&pi->var_lock);

	schedule_delayed_work(&pi->usb_detect_work, 1*HZ);
}

static void bq2729x_irq_work_func(struct work_struct *work)
{
//	struct bq24296_device_info *info= container_of(work, struct bq24296_device_info, irq_work);
//	printk("%s\n", __func__);

	bq24296_read(bq24296_di->client, SYSTEM_STATS_REGISTER, &r8, 1);
	bq24296_read(bq24296_di->client, FAULT_STATS_REGISTER, &r9, 1);

	printk("%s r8=%02x r9=%02x\n", __func__, r8, r9);
}

static irqreturn_t bq2729x_chg_irq_func(int irq, void *dev_id)
{
	struct bq24296_device_info *info = dev_id;
	DBG("%s\n", __func__);
	printk("%s\n", __func__);

	queue_work(info->workqueue, &info->irq_work);

	return IRQ_HANDLED;
}

/* regulator framework integration for VSYS and OTG */

static const unsigned int vsys_VSEL_table[] = {
	3000000,
	3100000,
	3200000,
	3300000,
	3400000,
	3500000,
	3600000,
	3700000,
};

static const unsigned int otg_VSEL_table[] = {
	4550000,
	4614000,
	4678000,
	4742000,
	4806000,
	4870000,
	4934000,
	4998000,
	5062000,
	5126000,
	5190000,
	5254000,
	5318000,
	5382000,
	5446000,
	5510000,
};

static int bq24296_get_vsys_voltage(struct regulator_dev *dev)
{
	struct bq24296_device_info *di = rdev_get_drvdata(dev);
	int idx = dev->desc->id;
	int ret;
	u8 retval;

	printk("bq24296_get_vsys_voltage(%d)\n", idx);

	ret = bq24296_read(bq24296_di->client, POWER_ON_CONFIGURATION_REGISTER, &retval, 1);
	if (ret < 0)
		return ret;
	printk(" => %d uV\n", vsys_VSEL_table[(retval >> SYS_MIN_OFFSET) & SYS_MIN_MASK]);
	return vsys_VSEL_table[(retval >> SYS_MIN_OFFSET) & SYS_MIN_MASK];
}

static int bq24296_set_vsys_voltage(struct regulator_dev *dev, int min_uV, int max_uV,
			       unsigned *selector)
{
	struct bq24296_device_info *di = rdev_get_drvdata(dev);
	int idx = dev->desc->id;

	printk("bq24296_set_vsys_voltage(%d, %d, %d, %u)\n", idx, min_uV, max_uV, *selector);
// The driver should select the voltage closest to min_uV by scanning vsys_VSEL_table

	return 0;

	/* set system voltage */

	return bq24296_update_reg(bq24296_di->client,
				  POWER_ON_CONFIGURATION_REGISTER,
				  *selector,	/* 3.0V + 0.2V */
				  SYS_MIN_MASK << SYS_MIN_OFFSET);
}

// should we be able to get/set the input current limit - which is the USB input current?

static int bq24296_get_otg_voltage(struct regulator_dev *dev)
{
	struct bq24296_device_info *di = rdev_get_drvdata(dev);
	int idx = dev->desc->id;
	int ret;
	u8 retval;

	printk("bq24296_get_otg_voltage(%d)\n", idx);

	ret = bq24296_read(bq24296_di->client, THERMAL_REGULATION_CONTROL_REGISTER, &retval, 1);
	if (ret < 0)
		return ret;
	printk(" => %d uV\n", otg_VSEL_table[(retval >> BOOSTV_OFFSET) & BOOSTV_MASK]);
	return otg_VSEL_table[(retval >> BOOSTV_OFFSET) & BOOSTV_MASK];
}

static int bq24296_set_otg_voltage(struct regulator_dev *dev, int min_uV, int max_uV,
			       unsigned *selector)
{
	struct bq24296_device_info *di = rdev_get_drvdata(dev);
	int idx = dev->desc->id;

	printk("bq24296_set_otg_voltage(%d, %d, %d, %u)\n", idx, min_uV, max_uV, *selector);
// The driver should select the voltage closest to min_uV by scanning otg_VSEL_table

	return 0;

	/* set OTG step up converter voltage */

	return bq24296_update_reg(bq24296_di->client,
				  THERMAL_REGULATION_CONTROL_REGISTER,
				  *selector,
				  BOOSTV_MASK << BOOSTV_OFFSET);
}

static int bq24296_get_otg_current_limit(struct regulator_dev *dev)
{
	struct bq24296_device_info *di = rdev_get_drvdata(dev);
	int idx = dev->desc->id;
	int ret;
	u8 retval;

	printk("bq24296_get_otg_current_limit(%d)\n", idx);

	ret = bq24296_read(bq24296_di->client, POWER_ON_CONFIGURATION_REGISTER, &retval, 1);
	if (ret < 0)
		return ret;

	return ((retval >> OTG_MODE_CURRENT_CONFIG_OFFSET) & OTG_MODE_CURRENT_CONFIG_MASK) ? 1000000 : 1500000;	/* 1.0A or 1.5A */
}

static int bq24296_set_otg_current_limit(struct regulator_dev *dev,
				     int min_uA, int max_uA)
{
	struct bq24296_device_info *di = rdev_get_drvdata(dev);
	int idx = dev->desc->id;

	printk("bq24296_set_otg_current_limit(%d, %d, %d)\n", idx, min_uA, max_uA);

	/* set OTG current limit in bit 0 of POWER_ON_CONFIGURATION_REGISTER */

	if(max_uA < 1250000)
		return bq24296_update_reg(bq24296_di->client,POWER_ON_CONFIGURATION_REGISTER, OTG_MODE_CURRENT_CONFIG_500MA,0x01);	// choose 1A
	else
		return bq24296_update_reg(bq24296_di->client,POWER_ON_CONFIGURATION_REGISTER, OTG_MODE_CURRENT_CONFIG_1300MA,0x01);	// choose 1.5A
}

static int bq24296_otg_enable(struct regulator_dev *dev)
{ /* enable OTG step up converter */
	struct bq24296_device_info *di = rdev_get_drvdata(dev);
	int idx = dev->desc->id;

	printk("%s(%d)\n", __func__, idx);

	/* check if battery is present and reject if no battery */
	if (!bq24296_battery_present) {
		dev_warn(&di->client->dev, "can enable otg only with installed battery\n");
		return -EBUSY;
	}

	gpiod_set_value_cansleep(bq24296_pdata->otg_usb_pin, 1);	// enable OTG pin

	/* enable bit 5 of POWER_ON_CONFIGURATION_REGISTER */

	return bq24296_update_reg(bq24296_di->client, POWER_ON_CONFIGURATION_REGISTER, 0x01 << 5, 0x01 << 5);	// enable OTG
}

static int bq24296_otg_disable(struct regulator_dev *dev)
{ /* disable OTG step up converter */
	struct bq24296_device_info *di = rdev_get_drvdata(dev);
	int idx = dev->desc->id;

	printk("%s(%d)\n", __func__, idx);

	gpiod_set_value_cansleep(bq24296_pdata->otg_usb_pin, 0);	// disable OTG pin

	/* disable bit 5 of POWER_ON_CONFIGURATION_REGISTER */

	return bq24296_update_reg(bq24296_di->client, POWER_ON_CONFIGURATION_REGISTER, 0 << 5, 0x01 << 5);	// disable OTG
}

static int bq24296_otg_is_enabled(struct regulator_dev *dev)
{ /* check if OTG converter is enabled */
	struct bq24296_device_info *di = rdev_get_drvdata(dev);
	int idx = dev->desc->id;
	int ret;
	u8 retval;

	printk("%s(%d)\n", __func__, idx);

	ret = bq24296_read(bq24296_di->client, POWER_ON_CONFIGURATION_REGISTER, &retval, 1);
	if (ret < 0)
		return 0;	/* assume disabled */

	/* is bit 5 of POWER_ON_CONFIGURATION_REGISTER set? */
	return (retval & (0x01 << 5)) != 0;
}

static struct regulator_ops vsys_ops = {
	.get_voltage = bq24296_get_vsys_voltage,
	.set_voltage = bq24296_set_vsys_voltage,	/* change vsys voltage */
};

static struct regulator_ops otg_ops = {
	// .get_voltage
	.get_voltage = bq24296_get_otg_voltage,
	.set_voltage = bq24296_set_otg_voltage,	/* change OTG voltage */
	.get_current_limit = bq24296_get_otg_current_limit,	/* get OTG current limit */
	.set_current_limit = bq24296_set_otg_current_limit,	/* set OTG current limit */
	.enable = bq24296_otg_enable,	/* turn on OTG mode */
	.disable = bq24296_otg_disable,	/* turn off OTG mode */
	.is_enabled = bq24296_otg_is_enabled,
};

static struct of_regulator_match bq24296_regulator_matches[] = {
	[VSYS_REGULATOR] = { .name = "bq2429x-vsys"},
	[OTG_REGULATOR] ={  .name = "bq2429x-otg"},
};

/* device tree support */

#ifdef CONFIG_OF
static struct bq24296_board *bq24296_parse_dt(struct bq24296_device_info *di)
{
	struct bq24296_board *pdata;
	struct device_node *bq24296_np;
	struct device_node *regulators;
	struct of_regulator_match *matches;
	static struct regulator_init_data *reg_data;
	int idx = 0, count, ret;
	u32 val;

	DBG("%s,line=%d\n", __func__,__LINE__);

	bq24296_np = of_node_get(di->dev->of_node);
	if (!bq24296_np) {
		dev_err(&di->client->dev, "could not find bq2429x DT node\n");
		return NULL;
	}
	pdata = devm_kzalloc(di->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;
	if (of_property_read_u32_array(bq24296_np, "ti,chg_current", pdata->chg_current, 3)) {
		dev_err(&di->client->dev, "charge current not specified\n");
		return NULL;
	}

	// dc_det_pin - if 0, charger is switched by driver to 2048mA, otherwise 512mA
	pdata->dc_det_pin = devm_gpiod_get_index(&di->client->dev, "dc-det", 0, GPIOD_IN);

// FIXME: check for bq24296 (297 has no det gpio)

	if (IS_ERR(pdata->dc_det_pin)) {
		if (PTR_ERR(pdata->dc_det_pin) == -EPROBE_DEFER)
			return NULL;
		dev_err(&di->client->dev, "invalid det gpio: %ld\n", PTR_ERR(pdata->dc_det_pin));
		pdata->dc_det_pin = NULL;
	}

	pdata->otg_usb_pin = devm_gpiod_get_index(&di->client->dev, "otg", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(pdata->otg_usb_pin)) {
		if (PTR_ERR(pdata->otg_usb_pin) == -EPROBE_DEFER)
			return NULL;
		dev_err(&di->client->dev, "invalid otg gpio: %ld\n", PTR_ERR(pdata->otg_usb_pin));
		pdata->otg_usb_pin = NULL;
	}

	of_node_get(bq24296_np);
	regulators = of_get_child_by_name(bq24296_np, "regulators");
	if (!regulators) {
		dev_err(&di->client->dev, "regulator node not found\n");
		return NULL;
	}

	count = ARRAY_SIZE(bq24296_regulator_matches);
	matches = bq24296_regulator_matches;

	ret = of_regulator_match(&di->client->dev, regulators, matches, count);
// printk("%d matches\n", ret);
	of_node_put(regulators);
	if (ret < 0) {
		dev_err(&di->client->dev, "Error parsing regulator init data: %d\n",
			ret);
		return NULL;
	}

	if (ret != count) {
		dev_err(&di->client->dev, "Found %d of expected %d regulators\n",
			ret, count);
		return NULL;
	}

	regulators = of_get_next_child(regulators, NULL);	// get first regulator (vsys)
	if (!of_property_read_u32(regulators, "regulator-max-microvolt", &val)) {
		dev_err(&di->client->dev, "found regulator-max-microvolt = %u\n", val);
		max_VSYS_uV = val;	// limited by device tree
	}

	reg_data = devm_kzalloc(&di->client->dev, (sizeof(struct regulator_init_data)
					* NUM_REGULATORS), GFP_KERNEL);
	if (!reg_data)
		return NULL;

	di->pmic_init_data = reg_data;

	for (idx = 0; idx < ret; idx++) {
// printk("matches[%d].of_node = %p\n", idx, matches[idx].of_node);
		if (!matches[idx].init_data || !matches[idx].of_node)
			continue;

		memcpy(&reg_data[idx], matches[idx].init_data,
				sizeof(struct regulator_init_data));

	}

	return pdata;
}

#else
static struct bq24296_board *bq24296_parse_dt(struct bq24296_device_info *di)
{
	return NULL;
}
#endif

#ifdef CONFIG_OF
static struct of_device_id bq24296_charger_of_match[] = {
	{ .compatible = "ti,bq24296"},
	{ .compatible = "ti,bq24297"},
	{ .compatible = "mps,mp2624"},	// can control VSYS-VBATT level but not OTG max power
	{ },
};
MODULE_DEVICE_TABLE(of, bq24296_charger_of_match);
#endif

static int bq24296_charger_suspend(struct device *device, pm_message_t state)
{
	// turn off otg?
	cancel_delayed_work_sync(&bq24296_di->usb_detect_work);
	return 0;
}

static int bq24296_charger_resume(struct device *device)
{
	schedule_delayed_work(&bq24296_di->usb_detect_work, msecs_to_jiffies(50));
	return 0;
}

static void bq24296_charger_shutdown(struct i2c_client *client)
{ /* make sure we turn off OTG mode on power down */
	struct bq24296_device_info *di = i2c_get_clientdata(client);

	bq24296_otg_disable(di->rdev[1]);	/* turn off otg regulator */
}

/* SYSFS interface */

/*
 * sysfs max_current store
 * set the max current drawn from USB
 */
static ssize_t
bq24296_input_current_limit_uA_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t n)
{
	int cur = 0;
	int status = 0;
	status = kstrtoint(buf, 10, &cur);
	if (status)
		return status;
	if (cur < 0)
		return -EINVAL;
	printk("bq24296_input_current_limit_uA_store: set input max current to %u uA -> %02x\n", cur, bq24296_limit_current_mA_to_bits(cur/1000));
	if (cur < 80000)
		status = bq24296_update_input_current_limit(-1);	/* High-Z */
	else
		status = bq24296_update_input_current_limit(bq24296_limit_current_mA_to_bits(cur/1000));
	return (status == 0) ? n : status;
}

/*
 * sysfs max_current show
 * reports current drawn from VBUS
 * note: actual input current limit is the lower of I2C register and ILIM resistor
 */

static ssize_t bq24296_input_current_limit_uA_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int cur = bq24296_input_current_limit_uA();
	if (cur < 0)
		return cur;

	return scnprintf(buf, PAGE_SIZE, "%u\n", cur);
}

// checkme: how does this relate to the OTG regulator getters/setters?

/*
 * sysfs otg store
 */
static ssize_t
bq24296_otg_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t n)
{
	int cur = 0;
	int status = 0;
	status = kstrtoint(buf, 10, &cur);
	if (status)
		return status;
	if (cur < 0)
		return -EINVAL;
	printk("bq24296_otg_store: set OTG max current %u uA\n", cur);
	bq24296_update_en_hiz_disable();
	mdelay(5);
	if(cur < 500000)
		status = bq24296_update_reg(bq24296_di->client,POWER_ON_CONFIGURATION_REGISTER,0 << 5,0x01 << 5);	// disable OTG
	else if(cur < 1250000)
		status = bq24296_update_reg(bq24296_di->client,POWER_ON_CONFIGURATION_REGISTER,((1 << 5)|OTG_MODE_CURRENT_CONFIG_500MA),((0x01 << 5)|0x01));	// enable 1A
	else
		status = bq24296_update_reg(bq24296_di->client,POWER_ON_CONFIGURATION_REGISTER,((1 << 5)|OTG_MODE_CURRENT_CONFIG_1300MA),((0x01 << 5)|0x01));	// enable 1.5A
#if 1
	{
	u8 retval = 0xff;
	bq24296_read(bq24296_di->client, POWER_ON_CONFIGURATION_REGISTER, &retval, 1);
	printk("bq24296_otg_store: POWER_ON_CONFIGURATION_REGISTER = %02x\n", retval);
	}
#endif
	return (status < 0) ? status : n;
}

/*
 * sysfs otg show
 */
static ssize_t bq24296_otg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	u8 retval = 0;
	int cur = 0;
	ret = bq24296_read(bq24296_di->client, POWER_ON_CONFIGURATION_REGISTER, &retval, 1);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
	}
	// check status register if it is really on: REG08[7:6] is set to 11
	if(retval & 0x20)	// OTG CONFIG
		{
		if(retval & 0x01)
			cur=1500000;
		else
			cur=1000000;
		}
	return scnprintf(buf, PAGE_SIZE, "%u\n", cur);
}

// can be removed if handled as property
static DEVICE_ATTR(max_current, 0644, bq24296_input_current_limit_uA_show,
			bq24296_input_current_limit_uA_store);

// do we need that? Only if there is no mechanism to set the regulator from user-space
static DEVICE_ATTR(otg, 0644, bq24296_otg_show, bq24296_otg_store);

/* power_supply interface */

static int bq24296_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	int ret;
	u8 retval = 0;

	DBG("%s,line=%d prop=%d\n", __func__,__LINE__, psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = bq24296_read(bq24296_di->client, SYSTEM_STATS_REGISTER, &retval, 1);
		if (ret < 0) {
			dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
		}
		switch((retval >> CHRG_OFFSET) & CHRG_MASK) {
			case CHRG_NO_CHARGING:	val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING; break;
			case CHRG_PRE_CHARGE:	val->intval = POWER_SUPPLY_STATUS_CHARGING; break;
			case CHRG_FAST_CHARGE:	val->intval = POWER_SUPPLY_STATUS_CHARGING; break;
			case CHRG_CHRGE_DONE:	val->intval = POWER_SUPPLY_STATUS_FULL; break;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = bq24296_read(bq24296_di->client, SYSTEM_STATS_REGISTER, &retval, 1);
		if (ret < 0) {
			dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
		}
		switch((retval >> CHRG_OFFSET) & CHRG_MASK) {
			case CHRG_NO_CHARGING:	val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE; break;
			case CHRG_PRE_CHARGE:	val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE; break;
			case CHRG_FAST_CHARGE:	val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST; break;
			case CHRG_CHRGE_DONE:	val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST; break;
		}
		break;
#if FIXME
	case POWER_SUPPLY_PROP_HEALTH:
		// to get this information we must read the interrupt register
		// but it reports the interrupt situation only once (auto-reset)
		// so it is only available through an interrupt handler
		switch((retval >> CHRG_OFFSET) & CHRG_MASK) {
			case CHRG_NO_CHARGING:	val->intval = POWER_SUPPLY_HEALTH_GOOD; break;
			case CHRG_PRE_CHARGE:	val->intval = POWER_SUPPLY_HEALTH_OVERHEAT; break;
			case CHRG_FAST_CHARGE:	val->intval = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE; break;
			case CHRG_CHRGE_DONE:	val->intval = POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE; break;
		}
		break;
#endif
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = bq24296_read(bq24296_di->client, SYSTEM_STATS_REGISTER, &retval, 1);
		if (ret < 0) {
			dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
		}
		if(retval & PG_STAT) {
			if(retval & DPM_STAT)
				val->intval = bq24296_get_vindpm();
			else
				val->intval = 5000000;	/* power good: assume VBUS 5V */
		}
		else
			val->intval = 0;	/* power not good: assume 0V */
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = bq24296_input_current_limit_uA();
//		printk("bq24296 CURRENT_MAX: %u mA\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = bq24296_read(bq24296_di->client, SYSTEM_STATS_REGISTER, &retval, 1);
		if (ret < 0) {
			dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
		}
		switch((retval >> CHRG_OFFSET) & CHRG_MASK) {
			case CHRG_NO_CHARGING:
			case CHRG_CHRGE_DONE:
				val->intval = 0;	// assume not charging current
//				printk("bq24296 CURRENT_NOW: %u mA\n", val->intval = ret);
				break;
			case CHRG_PRE_CHARGE:
				ret = bq24296_read(bq24296_di->client, PRE_CHARGE_TERMINATION_CURRENT_CONTROL_REGISTER, &retval, 1);
//				printk("bq24296: PRE_CHARGE_TERMINATION_CURRENT_CONTROL_REGISTER %02x\n", retval);
				if (ret < 0) {
					dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
				}
				ret = 128000 * ((retval >> PRE_CHARGE_CURRENT_LIMIT_OFFSET) & PRE_CHARGE_CURRENT_LIMIT_MASK) + 128000;	// return precharge limit
				val->intval = bq24296_input_current_limit_uA();
				if (ret < val->intval)
					val->intval = ret;
//				printk("bq24296 CURRENT_NOW: %u mA\n", val->intval);
				break;
			case CHRG_FAST_CHARGE:
				ret = bq24296_read(bq24296_di->client, CHARGE_CURRENT_CONTROL_REGISTER, &retval, 1);
//				printk("bq24296: FAST_CHARGE CHARGE_CURRENT_CONTROL_REGISTER %02x\n", retval);
				if (ret < 0) {
					dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
				}
				ret = 64000 * ((retval >> CHARGE_CURRENT_OFFSET) & CHARGE_CURRENT_MASK) + 512000;											val->intval = bq24296_input_current_limit_uA();
				if (ret < val->intval)
					val->intval = ret;
//				printk("bq24296 CURRENT_NOW: %u mA\n", val->intval);
				break;
		}
		break;
	case POWER_SUPPLY_PROP_TEMP:
		/* read twice to clear any flag */
		ret = bq24296_read(bq24296_di->client, FAULT_STATS_REGISTER, &retval, 1);
		if (ret < 0) {
			dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
		}
		ret = bq24296_read(bq24296_di->client, FAULT_STATS_REGISTER, &retval, 1);
		if (ret < 0) {
			dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
		}
		// FIXME: deduce values from BHOT and BCOLD settings if boost mode is active
		// otherwise we report the defaults from the chip spec
		if (retval & 0x02)
			val->intval = -100;	// too cold (-10C)
		else if (retval & 0x01)
			val->intval = 600;	// too hot (60C)
		else
			val->intval = 225;	// ok (22.5C)
		break;
	case POWER_SUPPLY_PROP_ONLINE:	/* charger online, i.e. VBUS */
		ret = bq24296_read(bq24296_di->client, SYSTEM_STATS_REGISTER, &retval, 1);
		if (ret < 0) {
			dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
		}
		val->intval = (retval & PG_STAT) != 0;	/* power is good */
		break;
	case POWER_SUPPLY_PROP_PRESENT:
#if 0	/* this would indicate a low battery! */
		ret = bq24296_read(bq24296_di->client, SYSTEM_STATS_REGISTER, &retval, 1);
		if (ret < 0) {
			dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
		}
		val->intval = !(retval & 0x1);	// VBAT > VSYSMIN
#else
		val->intval = bq24296_battery_present;
#endif
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int bq24296_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	int ret;
	u8 retval = 0;
	DBG("%s,line=%d prop=%d\n", __func__,__LINE__, psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (val->intval < 80000)
			return bq24296_update_input_current_limit(-1);	/* High-Z mode */
		return bq24296_update_input_current_limit(bq24296_limit_current_mA_to_bits(val->intval/1000));
	default:
		return -EPERM;
	}
	return 0;
}

static int bq24296_writeable_property(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return 1;
	default:
		break;
	}
	return 0;
}

static enum power_supply_property bq24296_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_PRESENT,
};

static const struct power_supply_desc bq24296_power_supply_desc[] = {
	{
	.name			= "bq24296",
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= bq24296_charger_props,
	.num_properties		= ARRAY_SIZE(bq24296_charger_props),
	.get_property		= bq24296_get_property,
	.set_property		= bq24296_set_property,
	.property_is_writeable	= bq24296_writeable_property,
	},
	{
	.name			= "bq24297",
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= bq24296_charger_props,
	.num_properties		= ARRAY_SIZE(bq24296_charger_props),
	.get_property		= bq24296_get_property,
	.set_property		= bq24296_set_property,
	.property_is_writeable	= bq24296_writeable_property,
	},
};

/* PROBE */

static int bq24296_charger_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct bq24296_device_info *di;
	u8 retval = 0;
	struct bq24296_board *pdev;
	struct device_node *bq24296_node;
	struct power_supply_config psy_cfg = { };
	struct regulator_config config = { };
	struct regulator_init_data *init_data;
	struct regulator_dev *rdev;
	int i;
	int ret = -EINVAL;

	DBG("%s,line=%d\n", __func__,__LINE__);

	bq24296_node = of_node_get(client->dev.of_node);
	if (!bq24296_node) {
		dev_err(&client->dev, "could not find bq24296 DT node\n");
	}

	di = devm_kzalloc(&client->dev, sizeof(*di), GFP_KERNEL);
	if (di == NULL) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		ret = -ENOMEM;
		goto fail_probe;
	}

	di->dev = &client->dev;
	bq24296_di = di;
	i2c_set_clientdata(client, di);
	di->client = client;

	if (bq24296_node)
		pdev = bq24296_parse_dt(di);
	else
		pdev = dev_get_platdata(di->dev);

	if (!pdev) {
		dev_err(&client->dev, "failed to get platform data\n");
		ret = -EPROBE_DEFER;
		goto fail_probe;
	}
	bq24296_pdata = pdev;

	DBG("%s,line=%d chg_current =%d usb_input_current = %d adp_input_current =%d \n", __func__,__LINE__,
		pdev->chg_current[0],pdev->chg_current[1],pdev->chg_current[2]);

	/******************get set current******/
	if (pdev->chg_current[0] && pdev->chg_current[1] && pdev->chg_current[2]){
		di->chg_current = bq24296_chg_current_mA_to_bits(pdev->chg_current[0] );
		di->usb_input_current  = bq24296_limit_current_mA_to_bits(pdev->chg_current[1]);
		di->adp_input_current  = bq24296_limit_current_mA_to_bits(pdev->chg_current[2]);
	}
	else {
		di->chg_current = bq24296_chg_current_mA_to_bits(1000);
		di->usb_input_current  = bq24296_limit_current_mA_to_bits(500);
		di->adp_input_current  = bq24296_limit_current_mA_to_bits(2000);
	}

	DBG("%s,line=%d chg_current =%d usb_input_current = %d adp_input_current =%d \n", __func__,__LINE__,
		di->chg_current,di->usb_input_current,di->adp_input_current);

	/****************************************/
	/* get the vendor id */
	ret = bq24296_read(di->client, VENDOR_STATS_REGISTER, &retval, 1);
	if (ret < 0) {
		dev_err(&di->client->dev, "%s(): Failed in reading register "
				"0x%02x\n", __func__, VENDOR_STATS_REGISTER);
		ret = -EPROBE_DEFER;	// tray again later
		goto fail_probe;
	}

	if ((retval & 0xa7) != 0x20) {
		dev_err(&client->dev, "not a bq24296/97: %02x\n", retval);
		ret = -ENODEV;
		goto fail_probe;
	}

/* we can also read and save the IINLIM value inherited from the boot process here! */

	init_data = di->pmic_init_data;
	if (!init_data)
		return -EINVAL;

	mutex_init(&di->var_lock);
	di->workqueue = create_singlethread_workqueue("bq24296_irq");
	INIT_WORK(&di->irq_work, bq2729x_irq_work_func);
	INIT_DELAYED_WORK(&di->usb_detect_work, usb_detect_work_func);

	// di->usb_nb.notifier_call = bq24296_bci_usb_ncb;

	psy_cfg.drv_data = bq24296_di;
	bq24296_di->usb = devm_power_supply_register(&client->dev,
						&bq24296_power_supply_desc[id->driver_data],
						&psy_cfg);
	if (IS_ERR(bq24296_di->usb)) {
		ret = PTR_ERR(bq24296_di->usb);
		dev_err(&client->dev, "failed to register as USB power_supply: %d\n", ret);
		goto fail_probe;
	}

	for (i = 0; i < NUM_REGULATORS; i++, init_data++) {
		/* Register the regulators */

		di->desc[i].id = i;
		di->desc[i].name = bq24296_regulator_matches[i].name;
// printk("%d: %s %s\n", i, di->desc[i].name, di->desc[i].of_match);
		di->desc[i].type = REGULATOR_VOLTAGE;
		di->desc[i].owner = THIS_MODULE;

		switch(i) {
			case VSYS_REGULATOR:
				di->desc[i].ops = &vsys_ops;
				di->desc[i].n_voltages = ARRAY_SIZE(vsys_VSEL_table);
				di->desc[i].volt_table = vsys_VSEL_table;
				break;
			case OTG_REGULATOR:
				di->desc[i].ops = &otg_ops;
				di->desc[i].n_voltages = ARRAY_SIZE(otg_VSEL_table);
				di->desc[i].volt_table = otg_VSEL_table;
				break;
		}

		config.dev = di->dev;
		config.init_data = init_data;
		config.driver_data = di;
		config.of_node = bq24296_regulator_matches[i].of_node;
// printk("%d: %s\n", i, config.of_node?config.of_node->name:"?");

		rdev = devm_regulator_register(&client->dev, &di->desc[i],
					       &config);
		if (IS_ERR(rdev)) {
			dev_err(di->dev,
				"failed to register %s regulator %d %s\n",
				client->name, i, di->desc[i].name);
			return PTR_ERR(rdev);
		}

		/* Save regulator for cleanup */
		di->rdev[i] = rdev;
	}

	ret = bq24296_init_registers();
	if (ret < 0) {
		dev_err(&client->dev, "failed to initialize registers: %d\n", ret);
		goto fail_probe;
	}

	ret = devm_request_threaded_irq(&client->dev, client->irq,
				NULL, bq2729x_chg_irq_func,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				client->name,
				di);
	if (ret < 0) {
		dev_warn(&client->dev, "failed to request chg_irq: %d\n", ret);
		// run with polling
//		goto fail_probe;
	}

	if (device_create_file(&client->dev, &dev_attr_max_current))
		dev_warn(&client->dev, "could not create sysfs file max_current\n");

	if (device_create_file(&client->dev, &dev_attr_otg))
		dev_warn(&client->dev, "could not create sysfs file otg\n");

	if (device_create_file(&client->dev, &dev_attr_registers))
		dev_warn(&client->dev, "could not create sysfs file registers\n");

	schedule_delayed_work(&di->usb_detect_work, 0);

	DBG("%s ok", __func__);

	return 0;

fail_probe:
	DBG("%s failed %d", __func__, ret);
	return ret;
}

static int bq24296_charger_remove(struct i2c_client *client)
{
	struct bq24296_device_info *di = i2c_get_clientdata(client);

	device_remove_file(di->dev, &dev_attr_max_current);
	device_remove_file(di->dev, &dev_attr_otg);
	device_remove_file(di->dev, &dev_attr_registers);

	return 0;
}

static const struct i2c_device_id bq24296_charger_id[] = {
	{ "bq24296", 0 },
	{ "bq24297", 1 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, bq24296_charger_id);

static struct i2c_driver bq24296_charger_driver = {
	.probe = bq24296_charger_probe,
	.remove = bq24296_charger_remove,
	.shutdown = bq24296_charger_shutdown,
	.id_table = bq24296_charger_id,
	.driver = {
		.name = "bq2429x_charger",
	//	.pm = &bq2429x_pm_ops,
		.of_match_table =of_match_ptr(bq24296_charger_of_match),
		.suspend = bq24296_charger_suspend,
		.resume = bq24296_charger_resume,
	},
};

module_i2c_driver(bq24296_charger_driver);

MODULE_AUTHOR("Rockchip");
MODULE_DESCRIPTION("TI BQ24296/7 charger driver");
MODULE_LICENSE("GPL");
