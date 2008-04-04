/*
 * tsl256x.c  --  TSL256x Light Sensor driver
 *
 * Copyright 2007 by Fiwin.
 * Author: Alec Tsai <alec_tsai@fiwin.com.tw>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This I2C client driver refers to pcf50606.c.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/input.h>

#include "tsl256x.h"

static unsigned short normal_i2c[] = { 0x39, I2C_CLIENT_END };
/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

struct tsl256x_data {
	struct i2c_client	client;
	struct mutex		lock;
	struct input_dev	*input_dev;
};

static struct i2c_driver tsl256x_driver;

/******************************************************************************
 * Low-Level routines
 *****************************************************************************/
static inline int __reg_write(struct tsl256x_data *tsl, u_int8_t reg,
								u_int8_t val)
{
	return i2c_smbus_write_byte_data(&tsl->client, reg, val);
}

static int reg_write(struct tsl256x_data *tsl, u_int8_t reg, u_int8_t val)
{
	int ret;

	mutex_lock(&tsl->lock);
	ret = __reg_write(tsl, reg, val);
	mutex_unlock(&tsl->lock);

	return ret;
}

static inline int32_t __reg_read(struct tsl256x_data *tsl, u_int8_t reg)
{
	int32_t ret;

	ret = i2c_smbus_read_byte_data(&tsl->client, reg);

	return ret;
}

static u_int8_t reg_read(struct tsl256x_data *tsl, u_int8_t reg)
{
	int32_t ret;

	mutex_lock(&tsl->lock);
	ret = __reg_read(tsl, reg);
	mutex_unlock(&tsl->lock);

	return ret & 0xff;
}

u_int32_t calculate_lux(u_int32_t iGain, u_int32_t iType, u_int32_t ch0,
						u_int32_t ch1)
{
	u_int32_t channel0 = ch0 * 636 / 10;
	u_int32_t channel1 = ch1 * 636 / 10;
	u_int32_t lux_value = 0;
	u_int32_t ratio = (channel1 * (2^RATIO_SCALE)) / channel0;
	u_int32_t b = 0, m = 0;

	if (0 == ch0)
		return 0;
	else {
		if (ratio > (13 * (2^RATIO_SCALE) / 10))
			return 0;
	}

	switch (iType) {
		case 0: // T package
			if ((ratio >= 0) && (ratio <= K1T)) {
				b = B1T;
				m = M1T;
			} else if (ratio <= K2T) {
				b = B2T;
				m = M2T;
			} else if (ratio <= K3T) {
				b = B3T;
				m = M3T;
			} else if (ratio <= K4T) {
				b = B4T;
				m = M4T;
			} else if (ratio <= K5T) {
				b = B5T;
				m = M5T;
			} else if (ratio <= K6T) {
				b = B6T;
				m = M6T;
			} else if (ratio <= K7T) {
				b = B7T;
				m = M7T;
			} else if (ratio > K8T) {
				b = B8T;
				m = M8T;
			}
		break;
		case 1:// CS package
			if ((ratio >= 0) && (ratio <= K1C)) {
				b = B1C;
				m = M1C;
			} else if (ratio <= K2C) {
				b = B2C;
				m = M2C;
			} else if (ratio <= K3C) {
				b = B3C;
				m = M3C;
			} else if (ratio <= K4C) {
				b = B4C;
				m = M4C;
			} else if (ratio <= K5C) {
				b = B5C;
				m = M5C;
			} else if (ratio <= K6C) {
				b = B6C;
				m = M6C;
			} else if (ratio <= K7C) {
				b = B7C;
				m = M7C;
			} else if (ratio > K8C) {
				b = B8C;
				m = M8C;
			}
		break;
		default:
			return 0;
		break;
	}

	lux_value = ((channel0 * b) - (channel1 * m)) / 16384;
	return(lux_value);
}

static ssize_t tsl256x_show_light_lux(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tsl256x_data *tsl = i2c_get_clientdata(client);
	u_int8_t low_byte_of_ch0 = 0, high_byte_of_ch0 = 0;
	u_int8_t low_byte_of_ch1 = 0, high_byte_of_ch1 = 0;
	u_int32_t adc_value_ch0, adc_value_ch1, adc_value;

	low_byte_of_ch0 = reg_read(tsl, TSL256X_REG_DATA0LOW);
	high_byte_of_ch0 = reg_read(tsl, TSL256X_REG_DATA0HIGH);
	low_byte_of_ch1 = reg_read(tsl, TSL256X_REG_DATA1LOW);
	high_byte_of_ch1 = reg_read(tsl, TSL256X_REG_DATA1HIGH);

	adc_value_ch0 = (high_byte_of_ch0 * 256 + low_byte_of_ch0) * 16;
	adc_value_ch1 = (high_byte_of_ch1 * 256 + low_byte_of_ch1) * 16;

	adc_value = calculate_lux(0, 0, adc_value_ch0, adc_value_ch1);
	return sprintf(buf, "%d\n", adc_value);
}

static DEVICE_ATTR(light_lux, S_IRUGO, tsl256x_show_light_lux, NULL);

static int tsl256x_detect(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *new_client = NULL;
	struct tsl256x_data *tsl256x = NULL;
	u_int8_t id = 0;
	int res = 0;

	if (!(tsl256x = kzalloc(sizeof(*tsl256x), GFP_KERNEL)))
		return -ENOMEM;

	mutex_init(&tsl256x->lock);
	new_client = &tsl256x->client;
	i2c_set_clientdata(new_client, tsl256x);
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &tsl256x_driver;
	new_client->flags = 0;
	strlcpy(new_client->name, "tsl256x", I2C_NAME_SIZE);

	/* now we try to detect the chip */
	/* register with i2c core */
	res = i2c_attach_client(new_client);
	if (res) {
		printk(KERN_DEBUG "[%s]Error: during i2c_attach_client()\n",
			new_client->name);
		goto exit_free;
	} else {
		printk(KERN_INFO "TSL256X is attached to I2C bus.\n");
	}

	/* Configure TSL256X. */
	{
		/* Power up TSL256X. */
		reg_write(tsl256x, TSL256X_REG_CONTROL, 0x03);

		/* Check TSL256X ID. */
		id = reg_read(tsl256x, TSL256X_REG_ID);
		if (TSL2561_ID == (id & 0xF0)) {
			/* Configuring the Timing Register.
				High Gain (16x), integration time of 101ms. */
			reg_write(tsl256x, TSL256X_REG_TIMING, 0x11);
		} else {
			goto exit_free;
		}
	}

	res = device_create_file(&new_client->dev, &dev_attr_light_lux);
	if (res)
		goto exit_detach;

	return 0;

exit_free:
	kfree(tsl256x);
	return res;
exit_detach:
	i2c_detach_client(new_client);
	return res;
}

static int tsl256x_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, &tsl256x_detect);
}

static int tsl256x_detach_client(struct i2c_client *client)
{
	struct tsl256x_data *tsl256x = i2c_get_clientdata(client);

	printk(KERN_INFO "Detach TSL256X from I2C bus.\n");

	/* Power down TSL256X. */
	reg_write(tsl256x, TSL256X_REG_CONTROL, 0x00);

	device_remove_file(&client->dev, &dev_attr_light_lux);
	kfree(tsl256x);

	return 0;
}

#ifdef CONFIG_PM
static int tsl256x_suspend(struct device *dev, pm_message_t state)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tsl256x_data *tsl256x = i2c_get_clientdata(client);

	/* Power down TSL256X. */
	reg_write(tsl256x, TSL256X_REG_CONTROL, 0x00);

	return 0;
}

static int tsl256x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tsl256x_data *tsl256x = i2c_get_clientdata(client);

	/* Power up TSL256X. */
	reg_write(tsl256x, TSL256X_REG_CONTROL, 0x03);

	return 0;
}
#endif

static struct i2c_driver tsl256x_driver = {
	.driver = {
		.name		= "tsl256x",
		.owner		= THIS_MODULE,
#ifdef CONFIG_PM
		.suspend	= tsl256x_suspend,
		.resume		= tsl256x_resume,
#endif
	},
	.id				= I2C_DRIVERID_TSL256X,
	.attach_adapter	= tsl256x_attach_adapter,
	.detach_client	= tsl256x_detach_client,
};

static int __init tsl256x_init(void)
{
	return i2c_add_driver(&tsl256x_driver);
}

static void __exit tsl256x_exit(void)
{
	i2c_del_driver(&tsl256x_driver);
}

MODULE_AUTHOR("Alec Tsai <alec_tsai@fiwin.com.tw>");
MODULE_LICENSE("GPL");

module_init(tsl256x_init);
module_exit(tsl256x_exit);

