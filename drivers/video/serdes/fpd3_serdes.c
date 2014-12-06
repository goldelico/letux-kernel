/*
 * fpd3_serdes.c
 *
 * lvds based serial link interface for onboard communication.
 * Copyright (C) 2014-2015 Texas Instruments Incorporated - http://www.ti.com/
 * Authors: Nikhil Devshatwar <nikhil.nd@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Datasheets:-
 * DS90UB913aq/DS90UB914aq http://www.ti.com/lit/ds/snls443a/snls443a.pdf
 * DS90UH925Q http://www.ti.com/lit/ds/symlink/ds90uh925q-q1.pdf
 * DS90UH928Q http://www.ti.com/lit/ds/snls440a/snls440a.pdf
 *
 * Documentation:-
 * -> Documentation/video-serdes.txt
 * -> Documentation/devicetree/bindings/video/fpd3-serdes.txt
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include "fpd3_serdes.h"

static const unsigned int fpd3_12bit_ser_init[] = {
	/* auto volt ctrl en, 3.3V, digital reset0, digital reset1 */
	FPD3_SER_RESET,		0x33,
	/* RX CRC check, TX parity en, i2c passthrough, rising edge pclk */
	FPD3_SER_CONFIG1,	0xc5,
	/* GPIO0 - o/p LOW, GPIO0 - o/p HIGH */
	FPD3_SER_GPIO_01,	0x55,
};

static const unsigned int fpd3_12bit_des_init[] = {
	/* back channel en */
	FPD3_DES_RESET,		0x04,
	/* RX parity check, TX CRC check, auto volt ctrl en
	 * i2c passthrough, auto ack WR, rising edge pclk */
	FPD3_DES_CONFIG1,	0xec,
};

static const unsigned int fpd3_24bit_ser_init[] = {
	/* digital reset1 */
	FPD3_SER_RESET,		0x02,
	/* back chan en, auto ack WR, i2c passthrough, rising edge pclk */
	FPD3_SER_CONFIG1,	0xab,
	/* failsafe low, LFMODE override, high freq*/
	FPD3_SER_CONFIG2,	0x8a,
	/* RGB data, 24bit, i2s data forwarding */
	FPD3_SER_DATA_CTRL,	0x8a,
};

static const unsigned int fpd3_24bit_des_init[] = {
	/* digital reset1 */
	FPD3_DES_RESET,		0x02,
	/* back channel en */
	FPD3_DES_RESET,		0x04,
	/* auto clk en, LFMODE override, high freq */
	FPD3_DES_CONFIG0,	0x2a,
	/* failsafe pull up, i2c passthrough, auto ack */
	FPD3_DES_CONFIG1,	0x4c,
};

static struct fpd3_serdes_platform_data fpd3_serdes_pdata[] = {
	{
		.name = "fpd3_12b_ser",
		.dev_type = FPD3_SER_DEV,
		.ngpio = 4,
		.nslaves = 1,
		.device_id = 0xb0,
		.gpio_2reg = 2 * FPD3_SER_GPIO_01,
		.init_seq = fpd3_12bit_ser_init,
		.init_len = ARRAY_SIZE(fpd3_12bit_ser_init),
	},
	{
		.name = "fpd3_12b_des",
		.dev_type = FPD3_DES_DEV,
		.ngpio = 4,
		.nslaves = 8,
		.device_id = 0xc0,
		.gpio_2reg = 2 * FPD3_DES_GPIO_01,
		.init_seq = fpd3_12bit_des_init,
		.init_len = ARRAY_SIZE(fpd3_12bit_des_init),
	},
	{
		.name = "fpd3_24b_ser",
		.dev_type = FPD3_SER_DEV,
		.ngpio = 4,
		.nslaves = 1,
		.device_id = 0x36,
		.gpio_2reg = 2 * FPD3_SER_GPIO_01 + 1,
		.init_seq = fpd3_24bit_ser_init,
		.init_len = ARRAY_SIZE(fpd3_24bit_ser_init),
	},
	{
		.name = "fpd3_24b_des",
		.dev_type = FPD3_DES_DEV,
		.ngpio = 4,
		.nslaves = 8,
		.device_id = 0x58,
		.gpio_2reg = 2 * FPD3_DES_GPIO_01 + 1,
		.init_seq = fpd3_24bit_des_init,
		.init_len = ARRAY_SIZE(fpd3_24bit_des_init),
	},
};

/* GPIO operations */
static int __fpd3_serdes_dirval(struct gpio_chip *chip,
			unsigned offset, int value, int shift)
{
	struct fpd3_serdes_data *data =
			container_of(chip, struct fpd3_serdes_data, chip);
	const struct fpd3_serdes_platform_data *pdata = data->pdata;
	struct i2c_client *client = data->client;
	int reg, val = 0x00;

	reg = (pdata->gpio_2reg + offset) / 2;
	if ((pdata->gpio_2reg + offset) % 2)
		shift += 4;

	val = i2c_read_le8(client, reg);

	if (value)
		val |= 1 << shift;
	else
		val &= ~(1 << shift);

	return i2c_write_le8(client, reg, val);
}

static int fpd3_serdes_input(struct gpio_chip *chip, unsigned offset)
{
	return __fpd3_serdes_dirval(chip, offset,
			GPIO_DIR_INPUT, GPIO_SHIFT_DIR);
}
static int fpd3_serdes_output(struct gpio_chip *chip,
		unsigned offset, int value)
{
	int ret;

	ret = __fpd3_serdes_dirval(chip, offset,
			GPIO_DIR_OUTPUT, GPIO_SHIFT_DIR);

	if (ret)
		ret = __fpd3_serdes_dirval(chip, offset,
			value, GPIO_SHIFT_VAL);

	return ret;
}

static void fpd3_serdes_set(struct gpio_chip *chip, unsigned offset, int value)
{
	__fpd3_serdes_dirval(chip, offset, value, GPIO_SHIFT_VAL);
}

static int fpd3_serdes_get(struct gpio_chip *chip, unsigned offset)
{
	struct fpd3_serdes_data *data =
			container_of(chip, struct fpd3_serdes_data, chip);
	const struct fpd3_serdes_platform_data *pdata = data->pdata;
	struct i2c_client *client = data->client;
	int reg;

	int shift = GPIO_SHIFT_VAL;
	reg = (pdata->gpio_2reg + offset) / 2;
	if ((pdata->gpio_2reg + offset) % 2)
		shift += 4;

	return (i2c_read_le8(client, reg) | 1 << shift) ? 1 : 0;
}

static struct gpio_chip fpd3_serdes_gpiochip = {
	.owner			= THIS_MODULE,
	.base			= -1,
	.ngpio			= 4,
	.can_sleep		= false,

	.get			= fpd3_serdes_get,
	.set			= fpd3_serdes_set,
	.direction_input	= fpd3_serdes_input,
	.direction_output	= fpd3_serdes_output,
};

int fpd3_serdes_initialize(struct i2c_client *client)
{
	struct fpd3_serdes_data *data = i2c_get_clientdata(client);
	const struct fpd3_serdes_platform_data *pdata = data->pdata;
	const unsigned int *seq;
	int i, len, reg, ret = 0, count = 0;

	if (data->slave_mode == false) {
		/* Wait for the device to initialize and show up device ID */
		reg = pdata->dev_type == FPD3_SER_DEV ?
			FPD3_SER_DEV_ID : FPD3_SER_DEV_ID;
		while (ret != pdata->device_id && count < FPD3_MAX_POLL_COUNT) {
			ret = i2c_read_le8(client, reg);
			count++;
			mdelay(3);
		}
		if (count == FPD3_MAX_POLL_COUNT) {
			dev_err(&client->dev, "Not a %s chip", pdata->name);
			return -ENODEV;
		}
	}

	len = pdata->init_len;
	seq = pdata->init_seq;
	for (i = 0; i < len; i += 2) {
		ret = i2c_write_le8(client, seq[i], seq[i+1]);
		if (ret)
			break;
		if (seq[i] == 0x01)
			udelay(500);
		else
			udelay(10);
	}

	return ret;
}
EXPORT_SYMBOL(fpd3_serdes_initialize);

/* On Vision app board (up to Rev B), the deserializer I2C addresses
 * conflicts with the HDMI receiver chip, making deserializer unaccessible.
 * The workaround is to change the I2C address of the HDMI receiver at runtime.
 * Following code removes the conflict between HDMI SIL9127 and deserializers
 * 0x31 is default I2C address of SIL9127
 */
static int fpd3_serdes_remove_conflict(struct i2c_client *client)
{
	struct regval {
		u8	reg;
	u8	val;
	} conflict_regval[] = {
		{	0x14,	0xf4,	},
		{	0x15,	0xf6,	},
		{	0x16,	0xf8,	},
		{	0x17,	0xfa,	},
		{	0x18,	0xfc,	},
		{	0x19,	0xfe,	},
	};
	union i2c_smbus_data data;
	int i, ret;
	u16 addr = 0x31;
	static bool conflict_exist = true;

	if (!conflict_exist)
		return 0;

	conflict_exist = false;
	for (i = 0; i < 6; i++) {
		data.byte = conflict_regval[i].val;
		ret = i2c_smbus_xfer(client->adapter, addr, client->flags,
			I2C_SMBUS_WRITE, conflict_regval[i].reg,
			I2C_SMBUS_BYTE_DATA, &data);
		if (ret)
			return ret;
	}

	dev_err(&client->dev, "Removed conflict b/w HDMI SIL");
	return 0;
}

static int fpd3_serdes_of_probe(struct i2c_client *client,
			struct device_node *node)
{
	struct fpd3_serdes_data *data = i2c_get_clientdata(client);
	int gpio, ret;

	gpio = of_get_gpio(node, 0);
	if (gpio_is_valid(gpio)) {
		/* Toggle the pdb bit from LOW to HIGH */
		ret = devm_gpio_request_one(&client->dev, gpio,
				GPIOF_INIT_HIGH, "des_pdb");
	} else if (gpio == -ENOENT)
		/* Entry not present - no need to toggle pdb */
		ret = 0;
	else
		ret = gpio;

	if (ret)
		return ret;

	data->gpio_export = of_find_property(node, "gpio-controller", NULL) ?
			true : false;
	data->slave_mode = of_find_property(node, "slave-mode", NULL) ?
			true : false;
	return 0;
}

static const struct i2c_device_id fpd3_serdes_i2c_ids[] = {
	{ "ds90ub913aq", 8 },
	{ "ds90ub914aq", 8 },
	{ "ds90uh925q", 8 },
	{ "ds90uh928q", 8 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fpd3_serdes_i2c_ids);

static const struct of_device_id fpd3_serdes_dt_ids[] = {
	{.compatible = "ti,ds90ub913aq", &fpd3_serdes_pdata[0], },
	{.compatible = "ti,ds90ub914aq", &fpd3_serdes_pdata[1], },
	{.compatible = "ti,ds90uh925q", &fpd3_serdes_pdata[2], },
	{.compatible = "ti,ds90uh928q", &fpd3_serdes_pdata[3], },
	{ }
};

MODULE_DEVICE_TABLE(of, fpd3_serdes_dt_ids);

static int fpd3_serdes_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	const struct of_device_id *of_dev_id;
	const struct fpd3_serdes_platform_data *pdata;
	struct fpd3_serdes_data *data;
	int status;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);

	fpd3_serdes_remove_conflict(client);

	of_dev_id = of_match_device(fpd3_serdes_dt_ids, &client->dev);
	if (!of_dev_id) {
		dev_err(&client->dev, "Unable to match device");
		return -EINVAL;
	}
	pdata = of_dev_id->data;
	data->pdata = pdata;

	status = fpd3_serdes_of_probe(client, client->dev.of_node);
	if (status)
		goto fail;

	if (data->slave_mode == false) {
		status = fpd3_register_i2c_adapter(client);
		if (status) {
			dev_err(&client->dev, "Cannot register i2c adapter");
			goto fail;
		}
	} else {
		status = fpd3_serdes_initialize(client);
		if (status)
			goto fail;
	}

	if (data->gpio_export == true) {
		/* Register local gpios as a separate gpiochip */
		data->chip = fpd3_serdes_gpiochip;
		data->chip.ngpio = pdata->ngpio;
		data->chip.dev = &client->dev;
		data->chip.label = client->name;
		status = gpiochip_add(&data->chip);
		if (status)
			goto fail;
	}

	if (status)
		goto unreg_gpio;

	dev_err(&client->dev, "%s %s ready", pdata->dev_type == FPD3_SER_DEV ?
		"Serializer" : "Deserializer", data->pdata->name);

	return 0;

unreg_gpio:
	if (data->gpio_export == true)
		return gpiochip_remove(&data->chip);
fail:
	return status;
}

static int fpd3_serdes_remove(struct i2c_client *client)
{
	struct fpd3_serdes_data	*data = i2c_get_clientdata(client);
	kfree(data);

	return 0;
}

static struct i2c_driver fpd3_serdes_driver = {
	.driver = {
		.name = "fpd3_serdes",
		.owner = THIS_MODULE,
		.of_match_table	= fpd3_serdes_dt_ids,
	},
	.probe = fpd3_serdes_probe,
	.remove = fpd3_serdes_remove,
	.id_table = fpd3_serdes_i2c_ids
};
module_i2c_driver(fpd3_serdes_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikhil Devshatwar");
