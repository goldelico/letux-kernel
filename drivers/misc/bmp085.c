/*  Copyright (c) 2010  Christoph Mair <christoph.mair@gmail.com>

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
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/interrupt.h>

#include <linux/i2c/bmp085.h>


#define BMP085_I2C_ADDRESS		0x77

#define BMP085_CALIBRATION_DATA_START	0xAA
#define BMP085_CALIBRATION_DATA_LENGTH	22

#define BMP085_CHIP_ID_REG		0xD0
#define BMP085_VERSION_REG		0xD1

#define BMP085_CHIP_ID			0x55 /* 85 */
#define BMP085_CLIENT_NAME		"bmp085"

#define BMP085_CONVERTING_NOTHING	0
#define BMP085_CONVERTING_PRESSURE	1
#define BMP085_CONVERTING_TEMPERATURE	2


/* Addresses to scan: 0x77 */
static const unsigned short normal_i2c[] = { BMP085_I2C_ADDRESS,
							I2C_CLIENT_END };

struct bmp085_calibration_data {
	s16 AC1, AC2, AC3;
	u16 AC4, AC5, AC6;
	s16 B1, B2;
	s16 MB, MC, MD;
};


/* Each client has this additional data */
struct bmp085_data {
	struct i2c_client *client;
	int irq;
	struct work_struct work;

	struct semaphore conversion_mutex;
	int status;

	unsigned char version;
	struct bmp085_calibration_data calibration;
	unsigned long raw_temperature;
	unsigned long raw_pressure;
	unsigned char oversampling_setting;
	unsigned long next_temp_measurement;
	long b6; /* calculated temperature correction coefficient */
};


static void bmp085_init_client(struct i2c_client *client);


static s32 bmp085_read_calibration_data(struct i2c_client *client)
{
	u8 tmp[BMP085_CALIBRATION_DATA_LENGTH];
	struct bmp085_data *data = i2c_get_clientdata(client);
	struct bmp085_calibration_data *cali = &(data->calibration);
	s32 status = i2c_smbus_read_i2c_block_data(client,
					BMP085_CALIBRATION_DATA_START,
					BMP085_CALIBRATION_DATA_LENGTH, tmp);

	cali->AC1 =  (tmp[0] << 8) | tmp[1];
	cali->AC2 =  (tmp[2] << 8) | tmp[3];
	cali->AC3 =  (tmp[4] << 8) | tmp[5];
	cali->AC4 =  (tmp[6] << 8) | tmp[7];
	cali->AC5 =  (tmp[8] << 8) | tmp[9];
	cali->AC6 = (tmp[10] << 8) | tmp[11];
	cali->B1 =  (tmp[12] << 8) | tmp[13];
	cali->B2 =  (tmp[14] << 8) | tmp[15];
	cali->MB =  (tmp[16] << 8) | tmp[17];
	cali->MC =  (tmp[18] << 8) | tmp[19];
	cali->MD =  (tmp[20] << 8) | tmp[21];
	return status;
}

static s32 bmp085_start_temperature_measurement(struct bmp085_data *data)
{
	s32 status;

	if (down_interruptible(&data->conversion_mutex))
		return -ERESTARTSYS;

	data->status = BMP085_CONVERTING_TEMPERATURE;
	status = i2c_smbus_write_byte_data(data->client, 0xF4, 0x2E);
	if (status != 0) {
		dev_err(&data->client->dev, "Error while requesting"
						" temperature measurement.\n");
	}
	return status;
}

static s32 bmp085_update_raw_temperature(struct bmp085_data *data)
{
	u8 tmp[2];
//	msleep(5);

	s32 status = i2c_smbus_read_i2c_block_data(data->client, 0xF6,
							sizeof(tmp), tmp);
	if (status != sizeof(tmp)) {
		dev_err(&data->client->dev, "Error while requesting"
				" temperature measurement (II): %d\n", status);
		return status;
	}
	data->raw_temperature = (tmp[0] << 8) + tmp[1];
	data->next_temp_measurement = jiffies+1*HZ;

	up(&data->conversion_mutex);

	return status;
}


static s32 bmp085_start_pressure_measurement(struct bmp085_data *data)
{
	s32 status = 0;

	/* lock chip access during the conversion */
	if (down_interruptible(&data->conversion_mutex))
		return -ERESTARTSYS;

	data->status = BMP085_CONVERTING_PRESSURE;
	status = i2c_smbus_write_byte_data(data->client, 0xF4, 0x34 +
					(data->oversampling_setting<<6));
	if (status != 0) {
		dev_err(&data->client->dev, "Error while requesting"
						"pressure measurement.\n");
	}
	return status;
}


static s32 bmp085_update_raw_pressure(struct bmp085_data *data)
{
	u8 tmp[3];

	/* wait for the end of conversion */
//	msleep(2+(3 << data->oversampling_setting<<1));

	s32 status = i2c_smbus_read_i2c_block_data(data->client, 0xF6,
							sizeof(tmp), tmp);
	if (status != sizeof(tmp)) {
		dev_err(&data->client->dev, "Error while reading"
				"pressure measurement results: %d\n", status);
		return status;
	}
	/* swap positions to correct the MSB/LSB positions */
	data->raw_pressure = (tmp[0] << 16) | (tmp[1] << 8) | tmp[2];
	data->raw_pressure >>= (8-data->oversampling_setting);
	
	up(&data->conversion_mutex);

	return status;
}


static s32 bmp085_get_temperature(struct bmp085_data *data)
{
	struct bmp085_calibration_data *cali = &data->calibration;
	long x1, x2;

	if (data->next_temp_measurement < jiffies) {
		bmp085_start_temperature_measurement(data);
		bmp085_update_raw_temperature(data);
	}

	/* make sure the data stays consistent during the calculations */
	if (down_interruptible(&data->conversion_mutex))
	      return -ERESTARTSYS;

	x1 = ((data->raw_temperature - cali->AC6) * cali->AC5) >> 15;
	x2 = (cali->MC << 11) / (x1 + cali->MD);
	data->b6 = x1 + x2 - 4000;
	up(&data->conversion_mutex);

	return (x1+x2+8) >> 4;
}


static s32 bmp085_get_pressure(struct bmp085_data *data)
{
	struct bmp085_calibration_data *cali = &data->calibration;
	long x1, x2, x3, b3;
	unsigned long b4, b7;
	long p;

	if (data->next_temp_measurement < jiffies)
		bmp085_get_temperature(data);

	bmp085_start_pressure_measurement(data);
	bmp085_update_raw_pressure(data);

	if (down_interruptible(&data->conversion_mutex))
		return -ERESTARTSYS;
	
	x1 = (data->b6 * data->b6) >> 12;
	x1 *= cali->B2;
	x1 >>= 11;

	x2 = cali->AC2 * data->b6;
	x2 >>= 11;

	x3 = x1 + x2;

	b3 = (((((long)cali->AC1) * 4 + x3) << data->oversampling_setting) + 2);
	b3 >>= 2;

	x1 = (cali->AC3 * data->b6) >> 13;
	x2 = (cali->B1 * ((data->b6 * data->b6) >> 12)) >> 16;
	x3 = (x1 + x2 + 2) >> 2;
	b4 = (cali->AC4 * (unsigned long)(x3 + 32768)) >> 15;

	b7 = ((unsigned long)data->raw_pressure - b3) *
					(50000 >> data->oversampling_setting);

	up(&data->conversion_mutex);

	p = ((b7 < 0x80000000) ? ((b7 << 1) / b4) : ((b7 / b4) * 2));

	x1 = p >> 8;
	x1 *= x1;
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	p += (x1 + x2 + 3791) >> 4;
	return p;
}


static void bmp085_set_oversampling(struct bmp085_data *data,
						unsigned char oversampling)
{
	if (oversampling > 3)
		oversampling = 3;
	data->oversampling_setting = oversampling;
}


static unsigned char bmp085_get_oversampling(struct bmp085_data *data)
{
	return data->oversampling_setting;
}



static void bmp085_work(struct work_struct *work)
{
	struct bmp085_data *data =
		container_of(work, struct bmp085_data, work);

	printk(KERN_INFO "bmp085_work");
	switch(data->status) {
	case BMP085_CONVERTING_TEMPERATURE:
		bmp085_update_raw_pressure(data);
		break;
	case BMP085_CONVERTING_PRESSURE:
		bmp085_update_raw_pressure(data);
		break;
	default:
		dev_err(&data->client->dev, "Unknown operating state!\n");
	}
	
	enable_irq(data->irq);
}

static irqreturn_t bmp085_irq(int irq, void *handle)
{
	struct bmp085_data *data = handle;

	printk(KERN_INFO "bmp085_irq");
	disable_irq_nosync(data->irq);
	schedule_work(&data->work);

	return IRQ_HANDLED;
}

static void bmp085_free_irq(struct bmp085_data *data)
{
	printk(KERN_INFO "bmp085_free_irq");
	free_irq(data->irq, data);
	if (cancel_work_sync(&data->work)) {
		/*
		 * Work was pending, therefore we need to enable
		 * IRQ here to balance the disable_irq() done in the
		 * interrupt handler.
		 */
		enable_irq(data->irq);
	}
}


/* sysfs callbacks */
static ssize_t set_oversampling(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bmp085_data *data = i2c_get_clientdata(client);
	unsigned long oversampling;
	strict_strtoul(buf, 10, &oversampling);
	bmp085_set_oversampling(data, oversampling);
	return count;
}

static ssize_t show_oversampling(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bmp085_data *data = i2c_get_clientdata(client);
	return sprintf(buf, "%u\n", bmp085_get_oversampling(data));
}
static DEVICE_ATTR(oversampling, S_IWUSR | S_IRUGO,
					show_oversampling, set_oversampling);


static ssize_t show_temperature(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bmp085_data *data = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", bmp085_get_temperature(data));
}
static DEVICE_ATTR(temperature, S_IRUGO, show_temperature, NULL);


static ssize_t show_pressure(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bmp085_data *data = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", bmp085_get_pressure(data));
}
static DEVICE_ATTR(pressure, S_IRUGO, show_pressure, NULL);


static struct attribute *bmp085_attributes[] = {
	&dev_attr_temperature.attr,
	&dev_attr_pressure.attr,
	&dev_attr_oversampling.attr,
	NULL
};

static const struct attribute_group bmp085_attr_group = {
	.attrs = bmp085_attributes,
};

static int bmp085_detect(struct i2c_client *client,
                          struct i2c_board_info *info)
{
	if (client->addr != BMP085_I2C_ADDRESS)
		return -ENODEV;

	if (i2c_smbus_read_byte_data(client, BMP085_CHIP_ID_REG) != BMP085_CHIP_ID)
		return -ENODEV;

	return 0;
}

static int bmp085_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct bmp085_data *data;
	struct bmp085_platform_data *pdata = pdata = client->dev.platform_data;

	int err = 0;

/*	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_BYTE_DATA |
				     I2C_FUNC_SMBUS_READ_BLOCK_DATA))
		return -EIO;*/

	data = kzalloc(sizeof(struct bmp085_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	if (pdata) {
		data->irq = client->irq;
		if (pdata->init_platform_hw)
			pdata->init_platform_hw();

		err = request_irq(data->irq, bmp085_irq, 0,
				client->dev.driver->name, data);
		if (err < 0) {
			dev_err(&client->dev, "irq %d busy?\n", data->irq);
			goto exit_free;
		}
	}
	
	init_MUTEX(&data->conversion_mutex);
	data->status = BMP085_CONVERTING_NOTHING;

	
	/* default settings after POR */
	data->oversampling_setting = 0x00;

	i2c_set_clientdata(client, data);

	/* Initialize the BMP085 chip */
	bmp085_init_client(client);

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &bmp085_attr_group);
	if (err)
		goto exit_free;

	INIT_WORK(&data->work, bmp085_work);
	
	dev_info(&data->client->dev, "succesfully initialized bmp085!\n");
	return 0;

exit_free:
	kfree(data);
exit:
	return err;
}

static int bmp085_remove(struct i2c_client *client)
{
	struct bmp085_data *data = i2c_get_clientdata(client);
	bmp085_free_irq(data);
	sysfs_remove_group(&client->dev.kobj, &bmp085_attr_group);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static void bmp085_init_client(struct i2c_client *client)
{
	struct bmp085_data *data = i2c_get_clientdata(client);
	data->client = client;
	bmp085_read_calibration_data(client);
	data->version = i2c_smbus_read_byte_data(client, BMP085_VERSION_REG);
	data->next_temp_measurement = 0;
	data->oversampling_setting = 3;
	dev_info(&data->client->dev, "BMP085 ver. %d.%d initialized\n",
			(data->version & 0x0F), (data->version & 0xF0) >> 4);
}

static const struct i2c_device_id bmp085_id[] = {
	{ "bmp085", 0 },
	{ }
};

static struct i2c_driver bmp085_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name	= "bmp085"
	},
	.id_table	= bmp085_id,
	.probe		= bmp085_probe,
	.remove		= bmp085_remove,

	.class		= I2C_CLASS_HWMON,
	.detect		= bmp085_detect,
	.address_list	= normal_i2c
};

static int __init bmp085_init(void)
{
	return i2c_add_driver(&bmp085_driver);
}

static void __exit bmp085_exit(void)
{
	i2c_del_driver(&bmp085_driver);
}


MODULE_AUTHOR("Christoph Mair <christoph.mair@gmail.com");
MODULE_DESCRIPTION("BMP085 driver");
MODULE_LICENSE("GPL");

module_init(bmp085_init);
module_exit(bmp085_exit);
