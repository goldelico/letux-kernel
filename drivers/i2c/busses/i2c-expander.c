/*
 * i2c-expander.c
 *
 * i2c expander driver.
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
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

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_i2c.h>
#include <linux/delay.h>

static const struct i2c_device_id i2c_expander_id[] = {
	{ "expander", 8 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, i2c_expander_id);

/*-------------------------------------------------------------------------*/

/* Talk to 8-bit link */

static int i2c_write_le8(struct i2c_client *client, unsigned addr,
							unsigned data)
{
	int ret;

	dev_err(&client->dev, "Writing addr = %x, data = %x", addr, data);
	ret = i2c_smbus_write_byte_data(client, addr, data);
	if (ret)
		dev_err(&client->dev, "Failed to write addr = %x, data = %x",
			addr, data);
	return ret;
}

static int i2c_expander_command(struct i2c_client *client,
					unsigned int cmd, void *arg)
{
	return 0;
}

static int i2c_expander_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device_node *node = NULL;
	struct i2c_client *hdmirec_i2c_client = NULL;
	int ret;

	node = of_parse_phandle(client->dev.of_node, "hdmirec", 0);
	if (node)
		hdmirec_i2c_client = of_find_i2c_device_by_node(node);
	else
		return dev_err(&client->dev, "HDMI receiver node not found");

	/* Write HDMI SIL 9127 registers to remove conflict with i2c expander */
	ret = i2c_write_le8(hdmirec_i2c_client, 0x14, 0xf4);
	if (ret)
		return ret;
	ret = i2c_write_le8(hdmirec_i2c_client, 0x15, 0xf6);
	if (ret)
		return ret;
	ret = i2c_write_le8(hdmirec_i2c_client, 0x16, 0xf8);
	if (ret)
		return ret;
	ret = i2c_write_le8(hdmirec_i2c_client, 0x17, 0xfa);
	if (ret)
		return ret;
	ret = i2c_write_le8(hdmirec_i2c_client, 0x18, 0xfc);
	if (ret)
		return ret;
	ret = i2c_write_le8(hdmirec_i2c_client, 0x19, 0xfe);
	if (ret)
		return ret;
	mdelay(1);

	/* Write to i2c expander */
	ret = i2c_write_le8(client, 0x00, 0x00);
	if (ret)
		return ret;
	ret = i2c_write_le8(client, 0x0c, 0x00);
	if (ret)
		return ret;
	ret = i2c_write_le8(client, 0x12, 0x63);
	if (ret)
		return ret;
	ret = i2c_write_le8(client, 0x12, 0xe3);
	if (ret)
		return ret;

	ret = i2c_write_le8(client, 0x01, 0x00);
	if (ret)
		return ret;
	ret = i2c_write_le8(client, 0x0d, 0x00);
	if (ret)
		return ret;
	ret = i2c_write_le8(client, 0x13, 0x63);
	if (ret)
		return ret;
	ret = i2c_write_le8(client, 0x13, 0xe3);
	if (ret)
		return ret;
	mdelay(1);

	dev_err(&client->dev, "i2c expander initialized");
	return 0;
}

static int i2c_expander_remove(struct i2c_client *client)
{
	return 0;
}


static const struct of_device_id i2c_expander_dt_ids[] = {
	{.compatible = "ti,i2cexp", },
	{ }
};

MODULE_DEVICE_TABLE(of, i2c_expander_dt_ids);

static struct i2c_driver i2c_expander_driver = {
	.driver = {
		.name = "i2c_expander",
		.owner = THIS_MODULE,
		.of_match_table	= i2c_expander_dt_ids,
	},
	.probe = i2c_expander_probe,
	.remove = i2c_expander_remove,
	.command = i2c_expander_command,
	.id_table = i2c_expander_id,
};

static int __init i2c_expander_init(void)
{
	return i2c_add_driver(&i2c_expander_driver);
}
module_init(i2c_expander_init);

static void __exit i2c_expander_exit(void)
{
	i2c_del_driver(&i2c_expander_driver);
}
module_exit(i2c_expander_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikhil Devshatwar");
