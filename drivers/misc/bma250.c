/*  Copyright (c) 2011  Christoph Mair <christoph.mair@gmail.com>
	Copyright (c) 2012  Nikolaus Schaller <hns@goldelico.com>
 
    This driver supports the BMA250 digital accelerometer from Bosch Sensortec.
	It is also one component in the BMC050 accelerometer/magnetometer.
    
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

#define BMA250_I2C_ADDRESS			0x18

struct bma250_meas_data {
	s16 accel_x;
	s16 accel_y;
	s16 accel_z;
	s8 temp;
};


static const unsigned short normal_i2c[] = { BMA250_I2C_ADDRESS, I2C_CLIENT_END };

static int bma250_read_chip_id(struct i2c_client *client)
{
	u8 chip_id;
	s32 result = i2c_smbus_read_i2c_block_data(client, 0x00, 1, &chip_id);
	printk("bytes: %d id: %02x\n", result, chip_id);
	if (result < 0)
		return result;
	return chip_id;
}


static s32 bma250_read_data(struct i2c_client *client, struct bma250_meas_data *data)
{
	s32 result = i2c_smbus_read_i2c_block_data(client, 0x02, sizeof(struct bma250_meas_data), (u8*)data);

	return result;
}


static ssize_t show_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	s32 result;
	struct bma250_meas_data data;

	result = bma250_read_data(client, &data);
	if (result > 0) {
	  return sprintf(buf, "%d,%d,%d\n", data.accel_x>>2, data.accel_y>>2, data.accel_z>>2);
	}
	return result;
}
static DEVICE_ATTR(coord, S_IRUGO, show_data, NULL);


static struct attribute *bma250_attributes[] = {
	&dev_attr_coord.attr,
	NULL
};

static const struct attribute_group bma250_attr_group = {
	.attrs = bma250_attributes,
};

static int bma250_detect(struct i2c_client *client,
			  struct i2c_board_info *info)
{
	if (bma250_read_chip_id(client) != 0x03)
		return -ENODEV;
	strlcpy(info->type, "bma250", I2C_NAME_SIZE);
	return 0;
}


static int bma250_suspend(struct i2c_client *client, pm_message_t mesg)
{
	u8 power_mode;
	s32 result = i2c_smbus_read_i2c_block_data(client, 0x11, 1, &power_mode);
	printk("bytes: %d power_mode: %02x\n", result, power_mode);
	if (result < 0)
		return result;
	power_mode |= 0x80;
	result = i2c_smbus_write_i2c_block_data(client, 0x11, 1, &power_mode);
	if (result < 0)
		return result;
	printk(KERN_INFO "BMC050 suspended\n");
	return 0;
}

static int bma250_resume(struct i2c_client *client)
{
	u8 power_mode;
	s32 result = i2c_smbus_read_i2c_block_data(client, 0x11, 1, &power_mode);
	printk("bytes: %d power_mode: %02x\n", result, power_mode);
	if (result < 0)
		return result;
	power_mode &= ~0x80;
	result = i2c_smbus_write_i2c_block_data(client, 0x11, 1, &power_mode);
	if (result < 0)
		return result;
	printk(KERN_INFO "BMC050 resumed\n");
	return 0;
}

static int bma250_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err = 0;
	printk("probe for BMA250/BMC050\n");
	if (bma250_read_chip_id(client) != 0x03)
		return -ENODEV;
	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &bma250_attr_group);
	if (err) {
		dev_err(&client->dev, "registering with sysfs failed!\n");
		goto exit;
	}

	dev_info(&client->dev, "probe succeeded!\n");

  exit:
	return err;
}

static int bma250_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &bma250_attr_group);
	return 0;
}

static const struct i2c_device_id bma250_id[] = {
	{ "bma250", 0 },
	{ }
};

//MODULE_DEVICE_TABLE(i2c, bma250_id);

static struct i2c_driver bma250_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name	= "bma250",
	},
	.id_table	= bma250_id,
	.probe		= bma250_probe,
	.remove		= bma250_remove,

	.detect		= bma250_detect,
	.address_list	= normal_i2c,

	.suspend	= bma250_suspend,
	.resume     = bma250_resume,
};

static int __init bma250_init(void)
{
//	return printk(KERN_ERR "hust!\n");
	return i2c_add_driver(&bma250_driver);
}

static void __exit bma250_exit(void)
{
//	printk(KERN_ERR "blupp\n");
	i2c_del_driver(&bma250_driver);
}


MODULE_AUTHOR("<Nikolaus Schaller> hns@goldelico.com");
MODULE_DESCRIPTION("BMA250 accelerometer driver");
MODULE_LICENSE("GPL");

module_init(bma250_init);
module_exit(bma250_exit);
