/*
 * LDC3001 Touch Screen driver
 *
 * Copyright (C) 2013-2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>


#define TSC_READ_LEN		1
#define DEFAULT_READ_LEN	2 /* corresponds to the ID bit-field */
#define TP_READ_SIZE		3 /* co-ordinates size in bytes */
#define MAX_PAYLOAD_SIZE(mt_points)	((mt_points * TP_READ_SIZE) \
						+ DEFAULT_READ_LEN)

/* MACROS to decode data from the i2c messages */
#define	GET_X_POS(msg)		(((((msg)[0] >> 4) & 0x0f) << 8) \
				| ((msg)[1]))

#define GET_Y_POS(msg)		((((msg)[0] & 0x0f)) << 8 | ((msg)[2]))

#define GET_INDICES(msg)	((((msg)[0] & 0x00ff) \
				| (((msg)[1]) & 0x30) << 4))

/* i2c offsets for the tsc */
#define TOUCH_DATA_ADDR		0xFA
#define TOUCH_DATA_COUNT_ADDR	0xF5

/* defines a touch */
struct touch_point {
	u8 touch_id;
	u8 state;
	int x_pos;
	int y_pos;
};

/* platform data for the LGPhilips LDC3001 TSC */
struct ldc3001_platform_data {
	unsigned int x_size;
	unsigned int y_size;
	unsigned int num_mt_slots;
};

/* each client has this additional data */
struct ldc3001_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	char phys[64];		/* device physical location */
	const struct ldc3001_platform_data *pdata;
	unsigned int irq;
	u8 *message;
	struct touch_point *tps;
	u8 cur_index;
	u16 tp_fields;
};

static ssize_t ldc3001_fw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf) {
	/* FIXME: read TLC to get TS version */
	return scnprintf(buf, PAGE_SIZE, "1.04\n");

}
static DEVICE_ATTR(fw_version, S_IRUGO, ldc3001_fw_version_show, NULL);

static struct attribute *ldc3001_attrs[] = {
	&dev_attr_fw_version.attr,
	NULL
};

static const struct attribute_group ldc3001_attr_group = {
	.attrs = ldc3001_attrs,
};

#ifdef CONFIG_PM_SLEEP
static int ldc3001_i2c_ts_suspend(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);
	return 0;
}

static int ldc3001_i2c_ts_resume(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);

	return 0;
}

static SIMPLE_DEV_PM_OPS(ldc3001_dev_pm_ops,
			 ldc3001_i2c_ts_suspend, ldc3001_i2c_ts_resume);

#endif

#ifdef DEBUG
static void dump_touch_data(u8 *message, int len)
{

	print_hex_dump(KERN_ERR, " \t", DUMP_PREFIX_NONE, 16, 1,
			message, len, false);
}

static u16 get_touch_fields(struct ldc3001_data *data)
{
	u8 *message = data->message;

	pr_info("Bit field is %u\n", GET_INDICES(message));
	return GET_INDICES(message);
}

/*
 * index	- index to the read message
 * touch_idx	- touch_idx tracking id
 */

static void  get_tpos(struct ldc3001_data *data, u8 index, u8 touch_idx)
{
	u16 x, y;
	u8 *message = &data->message[index*TP_READ_SIZE + DEFAULT_READ_LEN];

	data->tps[index].x_pos = GET_X_POS(msg);
	data->tps[index].y_pos = GET_Y_POS(msg);
	data->tps[index].touch_id = touch_idx;

	pr_info("x %x\n", data->tps[index].x_pos);
	pr_info("y %x\n", data->tps[index].y_pos);

}

static int ldc3001_write_reg(struct i2c_client *client, u16 reg, u16 len,
		const void *val)
{
	u8 *buf;
	size_t count;
	int ret;

	count = len + 2;
	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	/* pack the 16 bit reg value to byte data */
	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	memcpy(&buf[2], val, len);

	ret = i2c_master_send(client, buf, count);
	if (ret == count) {
		ret = 0;
	} else {
		if (ret >= 0)
			ret = -EIO;
		dev_err(&client->dev, "%s: i2c send failed (%d)\n",
			__func__, ret);
	}

	kfree(buf);
	return ret;
}
#endif

static int ldc3001_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];
	int ret;

	/* pack the 16 bit reg value to byte data */
	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

	ret = i2c_transfer(client->adapter, xfer, 2);
	if (ret == 2) {
		ret = 0;
	} else {
		if (ret >= 0)
			ret = -EIO;
		dev_err(&client->dev, "%s: i2c transfer failed (%d)\n",
			__func__, ret);
	}

	return ret;
}


static int ldc3001_ts_read_data(struct ldc3001_data *data, int len)
{
	int error;
	struct i2c_client *client = data->client;

	error = ldc3001_read_reg(client, TOUCH_DATA_ADDR, len, data->message);
	if (error)
		dev_err(&client->dev, "i2c_read failed %x\n", error);

	return error;
}

static int ldc3001_ts_read_len(struct ldc3001_data *data)
{
	int error;
	u8 len = 0;

	struct i2c_client *client = data->client;

	error = ldc3001_read_reg(client, TOUCH_DATA_COUNT_ADDR, TSC_READ_LEN,
		&len);
	if (error)
		dev_err(&client->dev, "i2c_read len failed %x\n", error);

	return len;
}



static void ldc3001_input_touchevent(struct ldc3001_data *data)
{
	u16 touch_index = 0;
	u8 i, msg_index = 0;
	int num_mt_slots = data->pdata->num_mt_slots;
	struct input_dev *input_dev = data->input_dev;
	u8 *message = data->message;

	/* get valid touch ids */
	touch_index = GET_INDICES(message);

	/* loop through all tps to process changes */
	for (i = 0; i < num_mt_slots; i++) {

		/* tracking id has a touch event */
		if (touch_index & (1 << i)) {
			message =
		&data->message[msg_index * TP_READ_SIZE + DEFAULT_READ_LEN];

			/* decode the x and y co-ordinates */
			data->tps[i].x_pos = GET_X_POS(message);
			data->tps[i].y_pos = GET_Y_POS(message);

			/* update state and move to the next message */
			data->tps[i].state = 1;
			msg_index++;

			/* report the touch event */
			input_mt_slot(input_dev, i);
			input_mt_report_slot_state(input_dev,
					MT_TOOL_FINGER, data->tps[i].state);
			input_report_abs(input_dev, ABS_MT_POSITION_X,
					data->tps[i].x_pos);
			input_report_abs(input_dev, ABS_MT_POSITION_Y,
					data->tps[i].y_pos);

		} else if (data->tps[i].state == 1) {
			/* this is a pen release event */
			data->tps[i].state = 0;
			input_mt_slot(input_dev, i);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER,
							data->tps[i].state);
		}
	}

	/* send sync event */
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(input_dev);
}


static irqreturn_t ldc3001_ts_interrupt(int irq, void *dev_id)
{
	struct ldc3001_data *data = (struct ldc3001_data *)dev_id;
	int read_len;

	/* FIXME: need to figure out why this is required */
	disable_irq_nosync(irq);

	read_len = ldc3001_ts_read_len(data);

	/* reset touchid fields */
	data->message[0] = 0x0;
	data->message[1] = 0x0;

	if (read_len > DEFAULT_READ_LEN)
		/* fill the message array with data read from tsc */
		ldc3001_ts_read_data(data, read_len);

	ldc3001_input_touchevent(data);
	enable_irq(irq);

	return IRQ_HANDLED;
}

int ldc3001_of_populate(struct i2c_client *client,
					struct ldc3001_platform_data *pdata)
{
	struct device_node *node = client->dev.of_node;
	unsigned int val;
	int ret = -EINVAL;

	ret = of_property_read_u32(node, "res-x", &val);
	if (ret)
		goto error;
	pdata->x_size = val;

	ret = of_property_read_u32(node, "res-y", &val);
	if (ret)
		goto error;

	pdata->y_size = val;

	ret = of_property_read_u32(node, "max-touch-points", &val);
	if (ret)
		goto error;

	pdata->num_mt_slots = val;

error:
	return ret;
}

static const struct of_device_id ldc3001_dt_ids[] = {
	{ .compatible = "lgphilips,ldc3001"},
	{ /* sentinel */ }
};

static int ldc3001_probe(struct i2c_client *client,
					const struct i2c_device_id *idev_id)
{
	struct device_node *node = client->dev.of_node;
	struct ldc3001_platform_data *pdata;
	struct ldc3001_data *data;
	struct input_dev *input_dev;
	const struct of_device_id *match;
	int error;

	match = of_match_device(of_match_ptr(ldc3001_dt_ids), &client->dev);
	if (match) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct ldc3001_platform_data), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		/* get dt values to populate platform data */
		error = ldc3001_of_populate(client, pdata);
		if (error)
			return -EINVAL;

	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			dev_err(&client->dev, "platform data not populated\n");
			return -EINVAL;
		}
	}

	data = devm_kzalloc(&client->dev,
			sizeof(struct ldc3001_data), GFP_KERNEL);

	if (!data) {
		error = -ENOMEM;
		goto err_exit;
	}

	data->tps = devm_kzalloc(&client->dev,
		sizeof(struct touch_point) * pdata->num_mt_slots, GFP_KERNEL);

	if (!data->tps) {
		error = -ENOMEM;
		goto err_exit;
	}

	data->message = devm_kzalloc(&client->dev,
		(sizeof(u8) * MAX_PAYLOAD_SIZE(pdata->num_mt_slots)),
		GFP_KERNEL);
	if (!data->message) {
		error = -ENOMEM;
		goto err_exit;
	}

	input_dev = devm_input_allocate_device(&client->dev);

	if (!input_dev) {
		error = -ENOMEM;
		goto err_exit;
	}

	input_dev->name = "LDC 3001 TouchScreen Controller";

	snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
		 client->adapter->nr, client->addr);

	input_dev->phys = data->phys;

	input_dev->dev.parent = &client->dev;
	data->client = client;
	data->input_dev = input_dev;
	data->pdata = pdata;

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	/*TODO: Need to poke the tlc chip to read revision of the TSC chip */

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			0, pdata->x_size, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			0, pdata->y_size, 0, 0);

	/* For multi touch */
	error = input_mt_init_slots(input_dev, pdata->num_mt_slots, 0);

	if (error)
		goto err_exit;

	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			0, pdata->x_size, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			0, pdata->y_size, 0, 0);

	data->irq = irq_of_parse_and_map(node, 0);
	if (data->irq) {
		error = devm_request_threaded_irq(&client->dev, data->irq,
				NULL, ldc3001_ts_interrupt,
				IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
				dev_name(&client->dev), data);

		if (error) {
			dev_err(&client->dev, "failed to register interrupt\n");
			goto cleanup_exit;
		}
	} else {
		dev_err(&client->dev,
				"failed registering irq = %d\n", data->irq);
		error = -EINVAL;
		goto cleanup_exit;
	}

	error = input_register_device(input_dev);
	if (error)
		input_free_device(input_dev);
	else {
		error = sysfs_create_group(&client->dev.kobj,
							&ldc3001_attr_group);
		return error;
	}

cleanup_exit:
	input_mt_destroy_slots(input_dev);
err_exit:
	return error;
}

static int ldc3001_remove(struct i2c_client *client)
{

	struct ldc3001_data *data =
			(struct ldc3001_data *)i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	sysfs_remove_group(&client->dev.kobj, &ldc3001_attr_group);
	input_unregister_device(input_dev);

	return 0;
}

static const struct i2c_device_id ldc3001_i2c_id[] = {
	{"ldc3001", 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, ldc3001_i2c_id);

static struct i2c_driver ldc3001_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ldc3001",
		.pm	= &ldc3001_dev_pm_ops,
		.of_match_table = ldc3001_dt_ids,
	},
	.probe		= ldc3001_probe,
	.remove		= ldc3001_remove,
	.id_table	= ldc3001_i2c_id,
};

module_i2c_driver(ldc3001_i2c_driver);

MODULE_AUTHOR("Subramaniam Chanderashekarapuram <subramaniam.ca@ti.com>");
MODULE_DESCRIPTION("LDC3001 I2C Touchscreen driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:ldc3001");
