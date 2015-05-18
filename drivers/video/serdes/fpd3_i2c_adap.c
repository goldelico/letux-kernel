/*
 * fpd3_i2c_adap.c
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
 * This file has helper functions to implement an i2c bus adapter driver for
 * fpd3_serdes devices. Only the master device can communicate with the
 * serializer and any other slaves connected at the remote end.
 *
 * Every i2c slave at the remote bus, can be accessed from linux i2c
 * using aliases. Master device registers some aliases and maps them to the
 * actual device addresses at remote bus.
 *
 * Any transactions addressed for these alias are detected by the master device
 * and then recreated at the remote i2c bus with the real slave address.
 * So in a way, master device acts as an i2c adapter driver for all the
 * remote clients.
 *
 * This file implements an i2c adapter which just reroutes the transactions
 * on the parent adapter with alias for the slave.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include "fpd3_serdes.h"

static int fpd3_serdes_setup_aliases(struct i2c_client *client)
{
	struct fpd3_serdes_data *data = i2c_get_clientdata(client);
	const struct fpd3_serdes_platform_data *pdata = data->pdata;
	int i = 0, ret, count = 0;
	u8 reg;

	/*
	 * First entry is for the remote serdes and any further entries
	 * are for the slaves connected at remote bus
	 */
	for (i = 0; i < data->num_slaves; i++) {

		if (data->slave_addr[i] == 0)
			continue;
		/* Setup the real slave address */
		if (i == 0)
			reg = pdata->dev_type == FPD3_SER_DEV ?
				FPD3_SER_DES_ID : FPD3_DES_SER_ID;
		else  {
			reg = pdata->dev_type == FPD3_SER_DEV ?
				FPD3_SER_SLAVE_ID0 : FPD3_DES_SLAVE_ID0;
			reg += i - 1;
		}
		ret = i2c_write_le8(client, reg, data->slave_addr[i] << 1);
		if (ret < 0)
			goto error_setup;

		/* Setup the alias for the slave */
		if (i == 0)
			reg = pdata->dev_type == FPD3_SER_DEV ?
				FPD3_SER_DES_AL : FPD3_DES_SER_AL;
		else  {
			reg = pdata->dev_type == FPD3_SER_DEV ?
				FPD3_SER_SLAVE_AL0 : FPD3_DES_SLAVE_AL0;
			reg += + i - 1;
		}
		ret = i2c_write_le8(client, reg, data->slave_alias[i] << 1);
		if (ret < 0)
			goto error_setup;

		dev_dbg(&client->dev, "Registered alias 0x%2x for 0x%02x",
			data->slave_alias[i], data->slave_addr[i]);
	}

	while (count < FPD3_MAX_POLL_COUNT) {

		reg = pdata->dev_type == FPD3_SER_DEV ?
			FPD3_SER_GN_STS : FPD3_DES_GN_STS;
		ret = i2c_read_le8(client, reg);
		if (ret < 0)
			return ret;
		if (ret & SIGNAL_DETECT)
			break;
		udelay(10);
		count++;
	}

	if (count >= FPD3_MAX_POLL_COUNT)
		return -EIO;

	dev_dbg(&client->dev, "Remote signal detected");
	return 0;
error_setup:
	dev_err(&client->dev, "Failed to setup alias for %2x",
				data->slave_addr[i]);
	return ret;
}

static int setup_link(struct i2c_adapter *adap)
{
	struct i2c_client *client = i2c_get_adapdata(adap);
	struct fpd3_serdes_data *data = i2c_get_clientdata(client);
	int ret;

	if (data->link_ok)
		return 0;
	else if (data->link_ok < 0)
		return data->link_ok;

	ret = fpd3_serdes_initialize(client);
	if (ret == 0)
		ret = fpd3_serdes_setup_aliases(client);
	if (ret == 0)
		data->link_ok = 1;
	else
		data->link_ok = -EIO;
	return ret;
}

static u8 fpd3_alloc_addr(struct i2c_adapter *adap, u8 addr)
{
	struct i2c_client *client = i2c_get_adapdata(adap);
	struct fpd3_serdes_data *data = i2c_get_clientdata(client);
	const struct fpd3_serdes_platform_data *pdata = data->pdata;
	u8 reg, i;

	for (i = 0; i < data->num_slaves; i++) {
		if (data->slave_addr[i] == 0x0)
			break;
	}
	if (i < data->num_slaves) {

		/* Program new mapping for remote address and alias */
		reg = pdata->dev_type == FPD3_SER_DEV ?
			FPD3_SER_SLAVE_ID0 : FPD3_DES_SLAVE_ID0;
		i2c_write_le8(client, reg, addr << 1);

		reg = pdata->dev_type == FPD3_SER_DEV ?
			FPD3_SER_SLAVE_AL0 : FPD3_DES_SLAVE_AL0;
		i2c_write_le8(client, reg, data->slave_alias[i] << 1);

		return data->slave_alias[i];
	} else
		return 0;
}

static u8 __get_alias_addr(struct i2c_adapter *adap, u8 addr)
{
	struct i2c_client *client = i2c_get_adapdata(adap);
	struct fpd3_serdes_data *data = i2c_get_clientdata(client);
	u8 new_alias;
	int i;

	/* master device should have the mapping between the
	 * slave address on remote bus to alias for which master device
	 * is monitoring for transactions */
	for (i = 0; i < data->num_slaves; i++) {
		if (data->slave_addr[i] == addr)
			break;
	}

	if (data->slave_addr[i] == addr)
		return data->slave_alias[i];

	new_alias = fpd3_alloc_addr(adap, addr);
	if (new_alias == 0)
		dev_err(&adap->dev, "Cannot translate chip addr 0x%02x", addr);

	return new_alias;
}

static struct i2c_adapter *__get_alias_adapter(struct i2c_adapter *adap)
{
	struct i2c_client *client = i2c_get_adapdata(adap);

	/* Same i2c adapter used for the master device should be
	 * used for communicating with the remote device via alias */
	return client->adapter;
}

static int fpd3_master_xfer(struct i2c_adapter *adap,
			struct i2c_msg msgs[], int num)
{
	struct i2c_adapter *alias_adap;
	struct i2c_msg new_msg;
	u8 alias_addr;
	int i, ret = 0;

	if (setup_link(adap))
		return -EIO;

	alias_adap = __get_alias_adapter(adap);

	for (i = 0; i < num; i++) {
		alias_addr = __get_alias_addr(adap, msgs[i].addr);
		if (alias_addr == 0) {
			ret = -EINVAL;
			break;
		}

		/* Copy the i2c_msgs into temporary buffer; xlate address */
		new_msg = msgs[i];
		new_msg.addr = alias_addr;
		/* Issue the messages on the alias adapter */
		ret = i2c_transfer(alias_adap, &new_msg, 1);
		if (ret)
			break;
	}

	return ret;
}

static int fpd3_smbus_xfer(struct i2c_adapter *adap, u16 addr,
		unsigned short flags, char read_write,
		u8 command, int size, union i2c_smbus_data *data)
{
	struct i2c_adapter *alias_adap;
	u8 alias_addr;

	if (setup_link(adap))
		return -EIO;

	alias_adap = __get_alias_adapter(adap);
	alias_addr = __get_alias_addr(adap, addr);
	if (alias_addr == 0)
		return -EINVAL;

	return i2c_smbus_xfer(alias_adap, alias_addr, flags, read_write,
			command, size, data);
}

static u32 fpd3_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK) |
		I2C_FUNC_PROTOCOL_MANGLING;
}

static const struct i2c_algorithm fpd3_i2c_algo = {
	.master_xfer	= fpd3_master_xfer,
	.smbus_xfer	= fpd3_smbus_xfer,
	.functionality	= fpd3_i2c_func,
};

static int parse_address_translation(struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	struct fpd3_serdes_data *data = i2c_get_clientdata(client);
	const struct fpd3_serdes_platform_data *pdata = data->pdata;
	unsigned int slave_bus_addr, master_bus_addr;
	int ret, i;

	for (i = 0; i < pdata->nslaves + 1; i++) {

		ret = of_property_read_u32_index(node, "ranges", 2 * i,
				&slave_bus_addr);
		if (ret)
			break;
		ret = of_property_read_u32_index(node, "ranges", 2 * i + 1,
				&master_bus_addr);
		if (ret)
			break;

		data->slave_addr[i] = slave_bus_addr;
		data->slave_alias[i] = master_bus_addr;
	}
	data->num_slaves = i;
	ret = 0;

	if (i == 0) {
		dev_err(&client->dev, "Master mode device does not have slave address translation information");
		return -ENODEV;
	}
	return 0;
}

int fpd3_register_i2c_adapter(struct i2c_client *client)
{
	struct fpd3_serdes_data *data = i2c_get_clientdata(client);
	struct i2c_adapter *adap;
	int ret = 0;

	dev_dbg(&client->dev, "Registering i2c adapter");

	adap = kzalloc(sizeof(*adap), GFP_KERNEL);
	if (adap == NULL)
		return -ENOMEM;

	data->adap = adap;
	adap->nr = -1;
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	adap->algo = &fpd3_i2c_algo;
	adap->dev.parent = &client->dev;
	adap->dev.of_node = client->dev.of_node;
	strlcpy(adap->name, "LVDS I2C adapter", sizeof(adap->name));

	ret = parse_address_translation(client);
	if (ret)
		return ret;

	i2c_set_adapdata(adap, client);
	return i2c_add_numbered_adapter(adap);
}

