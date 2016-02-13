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
#include <asm/unaligned.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/power/bq2429x_charger.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

struct bq24296_device_info *bq24296_di;
struct bq24296_board *bq24296_pdata;
static int bq24296_int = 0;
int bq24296_mode = 0;
#if 1
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

static int bq24296_read(struct i2c_client *client, u8 reg, u8 buf[], unsigned len)
{
	int ret;
	ret = bq24296_i2c_reg8_read(client, reg, buf, len);
	return ret;
}

static int bq24296_write(struct i2c_client *client, u8 reg, u8 const buf[], unsigned len)
{
	ret = bq24296_i2c_reg8_write(client, reg, buf, (int)len);
	return ret;
}

static ssize_t bat_param_read(struct device *dev,struct device_attribute *attr, char *buf)
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

static int bq24296_update_reg(struct i2c_client *client, int reg, u8 value, u8 mask )
{
	int ret =0;
	u8 retval = 0;

	ret = bq24296_read(client, reg, &retval, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

printk("bq24296_update_reg %02x: ( %02x & %02x ) | %02x -> %02x\n", reg, retval, (u8) ~mask, value, (u8) ((retval & ~mask) | value));

	if ((retval & mask) != value) {
		retval = (retval & ~mask) | value;
		ret = bq24296_write(client, reg, &retval, 1);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
		}
	}
{
	int i;
	u8 buffer;
	for(i=0;i<11;i++)
		{
		bq24296_read(bq24296_di->client, i, &buffer, 1);
		printk("  reg %02x value %02x\n", i, buffer);
		}
}

	return ret;
}

static int bq24296_init_registers(void)
{
	int ret = 0;

	/* reset the register */
	/*
	 ret = bq24296_update_reg(bq24296_di->client,
				 POWER_ON_CONFIGURATION_REGISTER,
				 REGISTER_RESET_ENABLE << REGISTER_RESET_OFFSET,
				 REGISTER_RESET_MASK << REGISTER_RESET_OFFSET);
	 if (ret < 0) {
		 dev_err(&bq24296_di->client->dev, "%s(): Failed to reset the register \n",
				 __func__);
		 goto final;
	 }

	 mdelay(5);
	 */
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

	/* Set Pre-Charge Current Limit as 128mA */
	ret = bq24296_update_reg(bq24296_di->client,
				PRE_CHARGE_TERMINATION_CURRENT_CONTROL_REGISTER,
				PRE_CHARGE_CURRENT_LIMIT_128MA << PRE_CHARGE_CURRENT_LIMIT_OFFSET,
				PRE_CHARGE_CURRENT_LIMIT_MASK << PRE_CHARGE_CURRENT_LIMIT_OFFSET);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s(): Failed to set pre-charge limit 128mA \n",
				__func__);
		goto final;
	}

	/* Set Termination Current Limit as 128mA */
	ret = bq24296_update_reg(bq24296_di->client,
				PRE_CHARGE_TERMINATION_CURRENT_CONTROL_REGISTER,
				TERMINATION_CURRENT_LIMIT_128MA << TERMINATION_CURRENT_LIMIT_OFFSET,
				TERMINATION_CURRENT_LIMIT_MASK << TERMINATION_CURRENT_LIMIT_OFFSET);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s(): Failed to set termination limit 128mA \n",
				__func__);
		goto final;
	}

	/* Set System Voltage Limit as 3.2V */
	ret = bq24296_update_reg(bq24296_di->client,
							 POWER_ON_CONFIGURATION_REGISTER,
							 0x04,	/* 3.0V + 0.2V */
							 0x0e);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s(): Failed to set voltage limit 3.2V \n",
				__func__);
		goto final;
	}

#if 1
	/* disable boost temperature protection (for debugging) */
	ret = bq24296_update_reg(bq24296_di->client,
							 THERMAIL_REGULATOION_CONTROL_REGISTER,
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

static int bq24296_get_limit_current(int value)
{
	u8 data;
	if (value < 120)
		data = 0;
	else if(value < 400)
		data = 1;
	else if(value < 700)
		data = 2;
	else if(value < 1000)
		data = 3;
	else if(value < 1200)
		data = 4;
	else if(value < 1800)
		data = 5;
	else if(value < 2200)
		data = 6;
	else
		data = 7;
	return data;
}

static int bq24296_get_chg_current(int value)
{
	u8 data;

	data = (value)/64;
	data &= 0xff;
	return data;
}

static int bq24296_update_input_current_limit(u8 value)
{
	int ret = 0;

printk("bq24296_update_input_current_limit(%u)\n", value);

	ret = bq24296_update_reg(bq24296_di->client,
				 INPUT_SOURCE_CONTROL_REGISTER,
				 ((value << IINLIM_OFFSET) | (EN_HIZ_DISABLE << EN_HIZ_OFFSET)),
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
	if(!bq24296_int)
		return 0;

	if(1 == on){
#ifdef CONFIG_BATTERY_RK30_USB_AND_CHARGE
		bq24296_update_input_current_limit(IINLIM_3000MA);
#else
		bq24296_update_input_current_limit(IINLIM_3000MA);
#endif
	}else{
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
	if(!bq24296_int)
		return 0;

	if(1 == on) {
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

int bq24296_charge_otg_en(int chg_en,int otg_en)
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

static int bq24296_read_sys_stats(u8 *retval)
{ /* return 0 if not charging, 1 if online */
	int ret;
	ret = bq24296_read(bq24296_di->client, SYSTEM_STATS_REGISTER, retval, 1);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
		return -1;
	}
	switch((*retval >> VBUS_OFFSET) & VBUS_MASK) {
		case VBUS_UNKNOWN:
		case VBUS_OTG:
			return 0;
		// NOTE: on the Pyra this also indicates if power comes from the main USB or the charging bypass
		case VBUS_USB_HOST:
		case VBUS_ADAPTER_PORT:
			return 1;
	}
	return 0;
}

int previous_online = 0;	// was the USB power already online last time we looked?

u8 lastretval = 0xff;
u8 lastchag_down = 0;

static void usb_detect_work_func(struct work_struct *work)
{
	struct delayed_work *delayed_work = (struct delayed_work *)container_of(work, struct delayed_work, work);
	struct bq24296_device_info *pi = (struct bq24296_device_info *)container_of(delayed_work, struct bq24296_device_info, usb_detect_work);
	u8 retval = 0;
	int ret ;

	ret = bq24296_read_sys_stats(&retval);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
		return;	/* don't schedule again */
	}

	if (((retval >> VBUS_OFFSET) & VBUS_MASK) == 3){ /* charge termination */
		bq24296_chag_down =1;
	}else
		bq24296_chag_down =0;

	if (retval != lastretval || bq24296_chag_down != lastchag_down)
		DBG("%s: retval = %02x bq24296_chag_down = %d\n", __func__,retval,bq24296_chag_down);
	lastretval = retval;
	lastchag_down = bq24296_chag_down;

	mutex_lock(&pi->var_lock);

	if (gpio_is_valid(bq24296_pdata->dc_det_pin)){
		/* detect charging request */
		ret = gpio_request(bq24296_pdata->dc_det_pin, "bq24296_dc_det");
		if (ret < 0) {
			DBG("Failed to request gpio %d with ret:""%d\n",bq24296_pdata->dc_det_pin, ret);
		}
		gpio_direction_input(bq24296_pdata->dc_det_pin);
		ret = gpio_get_value(bq24296_pdata->dc_det_pin);
		if (ret ==0){
			bq24296_update_input_current_limit(bq24296_di->adp_input_current);
			bq24296_set_charge_current(CHARGE_CURRENT_2048MA);
			bq24296_charge_mode_config(0);
		}
		else {
			bq24296_update_input_current_limit(IINLIM_500MA);
			bq24296_set_charge_current(CHARGE_CURRENT_512MA);
		}
		gpio_free(bq24296_pdata->dc_det_pin);
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

		/* detect VBUS being available */
		if(ret && !previous_online) { /* VBUS became available */
			DBG("bq24296: VBUS became available\n");
			bq24296_update_input_current_limit(bq24296_di->usb_input_current);
//			bq24296_update_charge_mode(CHARGE_MODE_CONFIG_CHARGE_BATTERY);
		}
		previous_online = ret;
#endif
	}

	mutex_unlock(&pi->var_lock);

	schedule_delayed_work(&pi->usb_detect_work, 1*HZ);
}

static void irq_work_func(struct work_struct *work)
{
//	struct bq24296_device_info *info= container_of(work, struct bq24296_device_info, irq_work);
}

static irqreturn_t chg_irq_func(int irq, void *dev_id)
{
//	struct bq24296_device_info *info = dev_id;
	DBG("%s\n", __func__);

//	queue_work(info->workqueue, &info->irq_work);

	return IRQ_HANDLED;
}

/* to be defined */
static int bq24296_set_otg_voltage(struct regulator_dev *dev, int min_uV, int max_uV,
			       unsigned *selector)
{
	printk("bq24296_set_voltage(..., %d, %d, %u)\n", min_uV, max_uV, *selector);
	/* enable/disable OTG step up converter */
	return 0;
}

static int bq24296_set_otg_current_limit(struct regulator_dev *dev,
				     int min_uA, int max_uA)
{
	printk("bq24296_set_current_limit(..., %d, %d)\n", min_uA, max_uA);
	/* set OTG current limit */
	// bq24296_update_otg_mode_current(OTG_MODE_CURRENT_CONFIG_1300MA);
	return 0;
}

static int bq24296_otg_enable(struct regulator_dev *dev)
{ /* enable OTG step up converter */
	printk("bq24296_enable(...)\n");
	/* check if battery is present and don't do w/o battery */
	bq24296_charge_mode_config(1);
	return 0;
}

static int bq24296_otg_disable(struct regulator_dev *dev)
{ /* disable OTG step up converter and enable charger */
	printk("bq24296_otg_disable(...)\n");
	bq24296_charge_mode_config(0);
	return 0;
}

static struct regulator_ops otg_ops = {
	.set_voltage = bq24296_set_otg_voltage,	/* enable/disable OTG */
	.set_current_limit = bq24296_set_otg_current_limit,	/* set OTG current limit */
	.enable = bq24296_otg_enable,	/* turn on OTG mode */
	.disable = bq24296_otg_disable,	/* turn off OTG mode */
};

#ifdef CONFIG_OF
static struct bq24296_board *bq24296_parse_dt(struct bq24296_device_info *di)
{
	struct bq24296_board *pdata;
	struct device_node *bq24296_np;

	DBG("%s,line=%d\n", __func__,__LINE__);
	bq24296_np = of_node_get(di->dev->of_node);
	if (!bq24296_np) {
		printk("could not find bq24296-node\n");
		return NULL;
	}
	pdata = devm_kzalloc(di->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;
	if (of_property_read_u32_array(bq24296_np,"ti,chg_current",pdata->chg_current, 3)) {
		printk("bq24296_parse_dt: charge current not specified\n");
		return NULL;
	}

	pdata->chg_irq_pin = of_get_named_gpio(bq24296_np,"gpios",0);
	if (!gpio_is_valid(pdata->chg_irq_pin)) {
		printk("bq24296_parse_dt: invalid gpio: %d\n",  pdata->chg_irq_pin);
	}

	pdata->dc_det_pin = of_get_named_gpio(bq24296_np,"gpios",1);
	if (!gpio_is_valid(pdata->dc_det_pin)) {
		printk("bq24296_parse_dt: invalid gpio: %d\n",  pdata->dc_det_pin);
	}

#if 0
	/* get child node(s) */
	struct device_node *np;
	struct regulator_desc desc;
	struct regulator_config cfg;

	np = of_find_node_by_name(bq24296_np, "vsys_regulator");
	// error handling
	of_get_regulator_init_data(di->dev, np, &desc);
	// error handling

	cfg.dev = di->dev;
	cfg.init_data = NULL;
	cfg.driver_data = NULL;
	cfg.of_node = np;
	regulator_register(&desc, &cfg);
	// error handling

	np = of_find_node_by_name(bq24296_np, "otg_regulator");
	// error handling
	of_get_regulator_init_data(di->dev, np, &desc);
	// error handling

	desc.name = "otg";
	desc.type = REGULATOR_VOLTAGE;
	desc.owner = THIS_MODULE;
	desc.ops = &otg_ops;
	desc.enable_time = 100000;	/* 100 ms */
	cfg.dev = di->dev;
	cfg.init_data = NULL;
	cfg.driver_data = NULL;
	cfg.of_node = np;
	regulator_register(&desc, &cfg);
	// error handling

#endif

	return pdata;
}

#else
static struct bq24296_board *bq24296_parse_dt(struct bq24296_device_info *di)
{
	return NULL;
}
#endif

#ifdef CONFIG_OF
static struct of_device_id bq24296_battery_of_match[] = {
	{ .compatible = "ti,bq24296"},
	{ .compatible = "ti,bq24297"},
	{ },
};
MODULE_DEVICE_TABLE(of, bq24296_battery_of_match);
#endif

static int bq24296_battery_suspend(struct i2c_client *client, pm_message_t mesg)
{
	cancel_delayed_work_sync(&bq24296_di->usb_detect_work);
	return 0;
}

static int bq24296_battery_resume(struct i2c_client *client)
{
	schedule_delayed_work(&bq24296_di->usb_detect_work, msecs_to_jiffies(50));
	return 0;
}

/*
 * sysfs max_current store
 * set the max current drawn from USB
 */
static ssize_t
bq24296_max_current_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t n)
{
	int cur = 0;
	int status = 0;
	status = kstrtoint(buf, 10, &cur);
	if (status)
		return status;
	if (cur < 0)
		return -EINVAL;
	printk("bq24296_max_current_store: set input max current to %u uA -> %02x\n", cur, bq24296_get_limit_current(cur));
	status = bq24296_update_input_current_limit(bq24296_get_limit_current(cur));
	return (status == 0) ? n : status;
}

/*
 * sysfs max_current show
 * reports current drawn from VBUS
 * note: actual input current limit is the lower of I2C register and ILIM resistor
 */
static ssize_t bq24296_max_current_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int cur;	/* in uA */
	int ret;
	u8 retval = 0;
	ret = bq24296_read(bq24296_di->client, POWER_ON_CONFIGURATION_REGISTER, &retval, 1);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}
	if(((retval >> EN_HIZ_OFFSET) & EN_HIZ_MASK) == EN_HIZ_ENABLE)
		cur=0;	// High-Z state
	else switch((retval >> IINLIM_OFFSET) & IINLIM_MASK) {
		case IINLIM_100MA: cur=100000; break;
		case IINLIM_150MA: cur=150000; break;
		case IINLIM_500MA: cur=500000; break;
		case IINLIM_900MA: cur=900000; break;
		case IINLIM_1200MA: cur=1200000; break;
		case IINLIM_1500MA: cur=1500000; break;
		case IINLIM_2000MA: cur=2000000; break;
		case IINLIM_3000MA: cur=3000000; break;
	}
	return scnprintf(buf, PAGE_SIZE, "%u\n", cur);
}

/*
 * sysfs id show
 */
static ssize_t bq24296_id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
#if FIXME
	// get ID pin status string "floating", 0R
	cur = ...;
	if (cur < 0)
		return cur;	// read error
#endif
	return scnprintf(buf, PAGE_SIZE, "floating\n");
}

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

static DEVICE_ATTR(max_current, 0644, bq24296_max_current_show,
			bq24296_max_current_store);
static DEVICE_ATTR(id, 0444, bq24296_id_show, NULL);
static DEVICE_ATTR(otg, 0644, bq24296_otg_show, bq24296_otg_store);


static int bq24296_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	int ret;
	u8 retval = 0;
	ret = bq24296_read_sys_stats(&retval);
	if (ret < 0) {
		dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
	}
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		switch((retval >> CHRG_OFFSET) & CHRG_MASK) {
			case CHRG_NO_CHARGING:	val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING; break;
			case CHRG_PRE_CHARGE:	val->intval = POWER_SUPPLY_STATUS_CHARGING; break;
			case CHRG_FAST_CHARGE:	val->intval = POWER_SUPPLY_STATUS_CHARGING; break;
			case CHRG_CHRGE_DONE:	val->intval = POWER_SUPPLY_STATUS_FULL; break;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		switch((retval >> CHRG_OFFSET) & CHRG_MASK) {
			case CHRG_NO_CHARGING:	val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE; break;
			case CHRG_PRE_CHARGE:	val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE; break;
			case CHRG_FAST_CHARGE:	val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST; break;
			case CHRG_CHRGE_DONE:	val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST; break;
		}
		break;
#if FIXME
	case POWER_SUPPLY_PROP_HEALTH:
		// we must read the interrupt register
		// but it reports the interrupt situation only once (auto-reset)
		switch((retval >> CHRG_OFFSET) & CHRG_MASK) {
			case CHRG_NO_CHARGING:	val->intval = POWER_SUPPLY_HEALTH_GOOD; break;
			case CHRG_PRE_CHARGE:	val->intval = POWER_SUPPLY_HEALTH_OVERHEAT; break;
			case CHRG_FAST_CHARGE:	val->intval = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE; break;
			case CHRG_CHRGE_DONE:	val->intval = POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE; break;
		}
		break;
#endif
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if((retval >> 2) & 0x01)
			val->intval = 5000000;	/* power good: assume 5V */
		else
			val->intval = 0;	/* power not good: assume 0V */
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		switch((retval >> CHRG_OFFSET) & CHRG_MASK) {
			case CHRG_NO_CHARGING:
			case CHRG_CHRGE_DONE:
				val->intval = 0;	// assume not charging current
				break;
			case CHRG_PRE_CHARGE:
				ret = bq24296_read(bq24296_di->client, PRE_CHARGE_TERMINATION_CURRENT_CONTROL_REGISTER, &retval, 1);
				if (ret < 0) {
					dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
				}
				val->intval = 128000 * ((retval >> 4) & 0x0f) + 128000;	// return precharge limit
				break;
			case CHRG_FAST_CHARGE:
				ret = bq24296_read(bq24296_di->client, CHARGE_CURRENT_CONTROL_REGISTER, &retval, 1);
				if (ret < 0) {
					dev_err(&bq24296_di->client->dev, "%s: err %d\n", __func__, ret);
				}
				val->intval = 64000 * ((retval >> CHARGE_CURRENT_OFFSET) & CHARGE_CURRENT_MASK) + 512000;	// return charge current limit (we do not know if real current is below!)
				break;
		}
		break;
	case POWER_SUPPLY_PROP_ONLINE:	/* charger online, i.e. VBUS */
		val->intval = ret;
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
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = !(retval & 0x1);	// VBAT > VSYSMIN
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property bq24296_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
//	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_PRESENT,
};

static const struct power_supply_desc bq24296_madc_bat_desc[] = {
	{
	.name			= "bq24296",
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= bq24296_charger_props,
	.num_properties		= ARRAY_SIZE(bq24296_charger_props),
	.get_property		= bq24296_get_property,
	},
	{
	.name			= "bq24297",
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= bq24296_charger_props,
	.num_properties		= ARRAY_SIZE(bq24296_charger_props),
	.get_property		= bq24296_get_property,
	},
};

static int bq24296_battery_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct bq24296_device_info *di;
	u8 retval = 0;
	struct bq24296_board *pdev;
	struct device_node *bq24296_node;
	struct power_supply_config psy_cfg = {};
	int ret = -EINVAL;

	DBG("%s,line=%d\n", __func__,__LINE__);

msleep(50);

	bq24296_node = of_node_get(client->dev.of_node);
	if (!bq24296_node) {
		printk("could not find bq24296-node\n");
	}

	di = devm_kzalloc(&client->dev, sizeof(*di), GFP_KERNEL);
	if (di == NULL) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		ret = -ENOMEM;
		goto batt_failed_2;
	}

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
		di->chg_current = bq24296_get_chg_current(pdev->chg_current[0] );
		di->usb_input_current  = bq24296_get_limit_current(pdev->chg_current[1]);
		di->adp_input_current  = bq24296_get_limit_current(pdev->chg_current[2]);
	}
	else {
		di->chg_current = bq24296_get_chg_current(1000);
		di->usb_input_current  = bq24296_get_limit_current(500);
		di->adp_input_current  = bq24296_get_limit_current(2000);
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
		printk("not a bq24296/97: %02x\n", retval);
		ret = -EINVAL;
		goto batt_failed_2;
	}

	psy_cfg.drv_data = bq24296_di;
	bq24296_di = di;
	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->client = client;

	di->workqueue = create_singlethread_workqueue("bq24296_irq");
	INIT_WORK(&di->irq_work, irq_work_func);
	mutex_init(&di->var_lock);
	INIT_DELAYED_WORK(&di->usb_detect_work, usb_detect_work_func);

	// di->usb_nb.notifier_call = bq24296_bci_usb_ncb;

	bq24296_di->usb = devm_power_supply_register(&client->dev,
						&bq24296_madc_bat_desc[id->driver_data],
						&psy_cfg);
	if (IS_ERR(bq24296_di->usb)) {
		ret = PTR_ERR(bq24296_di->usb);
		dev_err(&client->dev, "failed to register as USB power_supply: %d\n", ret);
		goto batt_failed_2;
	}

	schedule_delayed_work(&di->usb_detect_work, 0);
	bq24296_init_registers();

	if (gpio_is_valid(pdev->chg_irq_pin)){
		pdev->chg_irq = gpio_to_irq(pdev->chg_irq_pin);
		ret = request_threaded_irq(pdev->chg_irq, NULL, chg_irq_func, IRQF_TRIGGER_FALLING| IRQF_ONESHOT, "bq24296_chg_irq", di);
		if (ret) {
			ret = -EINVAL;
			printk("failed to request bq24296_chg_irq\n");
			goto err_chgirq_failed;
		}
	}

	if (device_create_file(&client->dev, &dev_attr_max_current))
		dev_warn(&client->dev, "could not create sysfs file\n");

	if (device_create_file(&client->dev, &dev_attr_id))
		dev_warn(&client->dev, "could not create sysfs file\n");

	if (device_create_file(&client->dev, &dev_attr_otg))
		dev_warn(&client->dev, "could not create sysfs file\n");

	if (device_create_file(&client->dev, &dev_attr_battparam))
		dev_warn(&client->dev, "could not create sysfs file\n");

	bq24296_int =1;

msleep(50);
	DBG("bq24296_battery_probe ok");

	return 0;

err_chgirq_failed:
	free_irq(gpio_to_irq(pdev->chg_irq_pin), NULL);
fail_probe:
	return ret;
}

static void bq24296_battery_shutdown(struct i2c_client *client)
{
	struct bq24296_device_info *di = i2c_get_clientdata(client);

	if (bq24296_pdata->chg_irq)
		free_irq(bq24296_pdata->chg_irq, di);
}

static int bq24296_battery_remove(struct i2c_client *client)
{
	struct bq24296_device_info *di = i2c_get_clientdata(client);

	device_remove_file(di->dev, &dev_attr_max_current);
	device_remove_file(di->dev, &dev_attr_id);
	device_remove_file(di->dev, &dev_attr_otg);
	device_remove_file(di->dev, &dev_attr_battparam);

	return 0;
}

static const struct i2c_device_id bq24296_id[] = {
	{ "bq24296", 0 },
	{ "bq24297", 1 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, bq24296_id);

static struct i2c_driver bq24296_battery_driver = {
	.probe = bq24296_battery_probe,
	.remove = bq24296_battery_remove,
	.suspend = bq24296_battery_suspend,
	.resume = bq24296_battery_resume,
	.id_table = bq24296_id,
	.shutdown = bq24296_battery_shutdown,
	.driver = {
		.name = "bq2429x_charger",
	//	.pm = &bq2429x_pm_ops,
		.of_match_table =of_match_ptr(bq24296_battery_of_match),
	},
	.id_table = bq24296_id,
};

module_i2c_driver(bq24296_battery_driver);

MODULE_AUTHOR("Rockchip");
MODULE_DESCRIPTION("TI BQ24296 battery monitor driver");
MODULE_LICENSE("GPL");
