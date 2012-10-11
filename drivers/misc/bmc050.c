/*  Copyright (c) 2010  Christoph Mair <christoph.mair@gmail.com>
	Copyright (c) 2012  Nikolaus Schaller <hns@goldelico.com>

	This driver supports the BMC050 digital magnetometer from Bosch Sensortec.
 
	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/


#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/delay.h>

#define BMC050_I2C_ADDRESS			0x10

struct bmc050_meas_data {
	s16 data_x;
	s16 data_y;
	s16 data_z;
	s16 r_hall;
	u8 int_status;
};


static const unsigned short normal_i2c[] = { BMC050_I2C_ADDRESS, I2C_CLIENT_END };

static int bmc050_read_chip_id(struct i2c_client *client)
{
	u8 chip_id;
	s32 result = i2c_smbus_read_i2c_block_data(client, 0x40, 1, &chip_id);
	printk("bytes: %d id: %02x\n", result, chip_id);
	if (result < 0)
		return result;
	return chip_id;
}


static s32 bmc050_read_data(struct i2c_client *client, struct bmc050_meas_data *data)
{
	s32 result = i2c_smbus_read_i2c_block_data(client, 0x42, sizeof(struct bmc050_meas_data), (u8*)data);
	printk("bytes: %d\n", result);
	/*
	 * here we should apply a compensation formula
	 */
	return result;
}


static ssize_t show_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	s32 result;
	struct bmc050_meas_data data;
	
	result = bmc050_read_data(client, &data);
	if (result > 0) {
		return sprintf(buf, "%d,%d,%d,%d\n", data.data_x>>2, data.data_y>>2, data.data_z>>2, data.r_hall>>2);
	}
	return result;
}
static DEVICE_ATTR(coord, S_IRUGO, show_data, NULL);


static struct attribute *bmc050_attributes[] = {
	&dev_attr_coord.attr,
	NULL
};

static const struct attribute_group bmc050_attr_group = {
	.attrs = bmc050_attributes,
};

static int bmc050_detect(struct i2c_client *client,
						 struct i2c_board_info *info)
{
	if (bmc050_read_chip_id(client) != 0x32)
		return -ENODEV;
	strlcpy(info->type, "bmc050", I2C_NAME_SIZE);
	return 0;
}


static int bmc050_suspend(struct i2c_client *client, pm_message_t mesg)
{
	u8 power_mode;
	s32 result = i2c_smbus_read_i2c_block_data(client, 0x4b, 1, &power_mode);
	printk("bytes: %d power_mode: %02x\n", result, power_mode);
	if (result < 0)
		return result;
	power_mode &= ~0x01;
	result = i2c_smbus_write_i2c_block_data(client, 0x4b, 1, &power_mode);
	if (result < 0)
		return result;
	printk(KERN_INFO "BMC050 suspended\n");
	return 0;
}

static int bmc050_resume(struct i2c_client *client)
{
	u8 power_mode;
	s32 result = i2c_smbus_read_i2c_block_data(client, 0x4b, 1, &power_mode);
	printk("bytes: %d power_mode: %02x\n", result, power_mode);
	if (result < 0)
		return result;
	power_mode |= 0x01;
	result = i2c_smbus_write_i2c_block_data(client, 0x4b, 1, &power_mode);
	if (result < 0)
		return result;
	mdelay(3);	/* wait to enter sleep mode */
	printk(KERN_INFO "BMC050 resumed\n");
	return 0;
}

static int bmc050_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	int err = 0;
	u8 op_mode;
	printk("probe for BMC050\n");
	bmc050_resume(client);	// wake up
	if (bmc050_read_chip_id(client) != 0x32)
		return -ENODEV;
	err = i2c_smbus_read_i2c_block_data(client, 0x4c, 1, &op_mode);
	printk("bytes: %d op_mode: %02x\n", err, op_mode);
	if (err < 0)
		goto exit;
	op_mode &= ~0x06;	/* set "normal" opmode */
	err = i2c_smbus_write_i2c_block_data(client, 0x4c, 1, &op_mode);
	if (err < 0)
		goto exit;
	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &bmc050_attr_group);
	if (err) {
		dev_err(&client->dev, "registering with sysfs failed!\n");
		goto exit;
	}
	
	dev_info(&client->dev, "probe succeeded!\n");
	
exit:
	return err;
}

static int bmc050_remove(struct i2c_client *client)
{
	pm_message_t mesg;
	bmc050_suspend(client, mesg);
	sysfs_remove_group(&client->dev.kobj, &bmc050_attr_group);
	return 0;
}

static const struct i2c_device_id bmc050_id[] = {
	{ "bmc050", 0 },
	{ }
};

//MODULE_DEVICE_TABLE(i2c, bmc050_id);

static struct i2c_driver bmc050_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name	= "bmc050",
	},
	.id_table	= bmc050_id,
	.probe		= bmc050_probe,
	.remove		= bmc050_remove,
	
	.detect		= bmc050_detect,
	.address_list	= normal_i2c,
	
	.suspend	= bmc050_suspend,
	.resume     = bmc050_resume,
};

static int __init bmc050_init(void)
{
	//	return printk(KERN_ERR "hust!\n");
	return i2c_add_driver(&bmc050_driver);
}

static void __exit bmc050_exit(void)
{
	//	printk(KERN_ERR "blupp\n");
	i2c_del_driver(&bmc050_driver);
}


MODULE_AUTHOR("<Nikolaus Schaller> hns@goldelico.com");
MODULE_DESCRIPTION("BMC050 magnetometer driver");
MODULE_LICENSE("GPL");

module_init(bmc050_init);
module_exit(bmc050_exit);
