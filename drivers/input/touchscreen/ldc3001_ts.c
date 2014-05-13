/*
 * TI Touch Screen driver
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
#include <linux/of_i2c.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>


#undef DEBUG

/* polling mode set comment this line to enable interrupt mode */

/* #define TS_POLL */

#define MAX_TOUCH_POINTS	10 /* max touch points handled by the TSC */
#define X_SIZE			1028 /* X size of the TSC */
#define Y_SIZE			800 /* Y size of the TSC */
#define TP_READ_SIZE		3 /* co-ordinates size in bytes */
#define DEFAULT_READ_LEN	2 /* corresponds to the ID bit-field */


#define TOUCH_DATA_ADDR 0xFA
#define TOUCH_DATA_COUNT_ADDR 0xF5

#ifdef TS_POLL
#include <linux/delay.h>
#endif

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
	unsigned long irqflags; /* TODO: needed? */
};

/* each client has this additional data */
struct ldc3001_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	char phys[64];		/* device physical location. TODO: needed? */
	const struct ldc3001_platform_data *pdata;
	unsigned int irq;
	u8 message[MAX_TOUCH_POINTS * TP_READ_SIZE + DEFAULT_READ_LEN];
	struct touch_point tps[MAX_TOUCH_POINTS];
	u8 cur_index;
	u16 tp_fields;
#ifdef TS_POLL
	struct task_struct *task;
#endif
};

static ssize_t ldc3001_fw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf) {
	/* FIXME: read TLC to get TS version */
	return scnprintf(buf, PAGE_SIZE, "1.04\n");

}
static DEVICE_ATTR(fw_version, S_IRUGO, ldc3001_fw_version_show, NULL);

static struct attribute *mxt_attrs[] = {
	&dev_attr_fw_version.attr,
	NULL
};

static const struct attribute_group mxt_attr_group = {
	.attrs = mxt_attrs,
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

#if DEBUG
static void dump_touch_data(u8 *message, int len)
{

	print_hex_dump(KERN_DEBUG, " \t", DUMP_PREFIX_NONE, 16, 1,
			message, len, false);
}

static u16 get_touch_fields(struct ldc3001_data *data)
{
	u8 *message = data->message;
	/*FIXME: remove magic numbers */
	data->tp_fields = data->message[0] & 0x00ff;
	data->tp_fields |= ((message[1] & 0x30)  << 4);

	pr_err("Bit field is %u\n", data->tp_fields);
	return data->tp_fields;
}

/*
 * index	- index to the read message
 * touch_idx	- touch_idx tracking id
 */

static void  get_tpos(struct ldc3001_data *data, u8 index, u8 touch_idx)
{
	u16 x, y;
	u8 *message = &data->message[index*3 + 2];

	x	  = (((message[0] >> 4) & 0x0f) << 8);
	pr_err("x-hi %x\n", x);
	data->tps[index].x_pos = (x | message[1]);
	pr_err("x %x\n", data->tps[index].x_pos);

	y	  = (message[0] & 0x0f) << 8;
	pr_err("y-hi %x\n", y);
	data->tps[index].y_pos = (y | message[2]);
	pr_err("y %x\n", data->tps[index].y_pos);

	data->tps[index].touch_id = touch_idx;
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

	error = ldc3001_read_reg(client, TOUCH_DATA_ADDR, len, &data->message);
	if (error)
		dev_err(&client->dev, "i2c_read failed %x\n", error);

	return error;
}

static int ldc3001_ts_read_len(struct ldc3001_data *data)
{
	int error;
	u8 len = 0;

	struct i2c_client *client = data->client;

	/*FIXME: remove magic numbers */
	error = ldc3001_read_reg(client, TOUCH_DATA_COUNT_ADDR, 1, &len);
	if (error)
		dev_err(&client->dev, "i2c_read failed %x\n", error);
	else
		dev_dbg(&client->dev, "length is %x\n", len);

	return len;
}



static void ldc3001_input_touchevent(struct ldc3001_data *data)
{
	u16 x, y, touch_index = 0;
	u8 i, msg_index = 0;
	struct input_dev *input_dev = data->input_dev;

	u8 *message = &data->message[0];


	/* get valid touch ids */
	/* FIXME: remove all magic numbers */
	touch_index = message[0] & 0x00ff;
	touch_index |= ((message[1] & 0x30) << 4);

	/* loop through all tps to process changes */
	for (i = 0; i < MAX_TOUCH_POINTS; i++) {

		/* tracking id has a touch event */
		if (touch_index & (1 << i)) {
			message = &data->message[msg_index*3 + 2];

			/* decode the x and y co-ordinates */
			x = (((message[0] >> 4) & 0x0f) << 8);
			data->tps[i].x_pos = (x | message[1]);

			y = (message[0] & 0x0f) << 8;
			data->tps[i].y_pos = (y | message[2]);

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

			pr_debug("x_pos = %d for %d\n", data->tps[i].x_pos, i);
			pr_debug("y_pos = %d for %d\n", data->tps[i].y_pos, i);

		} else if (data->tps[i].state == 1) {
			/* FIXME: remove all magic numbers */
			/* this is a pen release event */
			data->tps[i].state = 0;
			input_mt_slot(input_dev, i);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER,
							data->tps[i].state);
			pr_debug("pen release event for %d\n", i);
		}
	}

	pr_debug("Reported %d events to userspace\n", msg_index);

	/* send sync event */
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(input_dev);
}


#ifdef TS_POLL
static int ldc3001_ts_interrupt(void *dev_id)
#else /* INTERRPUT */
static irqreturn_t ldc3001_ts_interrupt(int irq, void *dev_id)
#endif
{
	struct ldc3001_data *data = (struct ldc3001_data *)dev_id;
	int read_len;
#ifdef TS_POLL
	do {
		u16 touch_index = 0, i = 0, j = 0;

#else
		disable_irq_nosync(irq);
#endif
		/* Read and print the len value
		   Read i2c messages for the size */
		read_len = ldc3001_ts_read_len(data);

		/* reset touchid fields */
		data->message[0] = 0x0;
		data->message[1] = 0x0;

		if (read_len > DEFAULT_READ_LEN) {
			/* fill the message array with data read from tsc */
			ldc3001_ts_read_data(data, read_len);
		}
		ldc3001_input_touchevent(data);
#ifdef TS_POLL
		schedule();
		mdelay(50);
	} while (!kthread_should_stop());
#else
	enable_irq(irq);
	return IRQ_HANDLED;
#endif
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

	dev_dbg(&client->dev, "%s\n", __func__);
	match = of_match_device(of_match_ptr(ldc3001_dt_ids), &client->dev);
	if (match) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct ldc3001_platform_data), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;
		/* TODO: do of populate here
		   error = ldc3001_of_populate(client, pdata);
		   if (error)
		   return -EINVAL;
		 */
		/* TODO: REMOVE THIS ONCE OF POPULATE IS IMPLEMENTED */
		pdata->x_size = X_SIZE;
		pdata->y_size = Y_SIZE;
		pdata->num_mt_slots = MAX_TOUCH_POINTS;
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			dev_err(&client->dev, "Platform data not populated\n");
			return -EINVAL;
		}
	}

	data = devm_kzalloc(&client->dev,
			sizeof(struct ldc3001_data), GFP_KERNEL);

	if (data)
		input_dev = devm_input_allocate_device(&client->dev);

	if (!data || !input_dev) {
		dev_err(&client->dev, "failed to allocate memory\n");
		error = -ENOMEM;
		goto err_exit;
	}

	input_dev->name = "LDC 3001 TouchScreen Controller";

	snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
			client->adapter->nr, client->addr);

	input_dev->id.bustype = BUS_I2C;
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
	/* TODO:get resolution from dt */
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

#ifdef TS_POLL

	data->task = kthread_run(ldc3001_ts_interrupt, (void *)data,
							"ts_poll_thread");
#else
	/* get irq from dt. NOTE: Only dt supported for now */
	data->irq = irq_of_parse_and_map(node, 0);

	if (data->irq) {
		error = devm_request_threaded_irq(&client->dev, data->irq,
				NULL, ldc3001_ts_interrupt,
				IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
				/* IRQF_ONESHOT, */
				dev_name(&client->dev), data);

		if (error) {
			dev_err(&client->dev, "Failed to register interrupt\n");
			goto err_exit;
		}
	} else {

		pr_err("failed registering irq = %d\n", data->irq);
		error = -EINVAL;
		goto err_exit;
	}
#endif

	error = input_register_device(input_dev);
	if (!error)
		error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);

err_exit:

	return error;
}

static int ldc3001_remove(struct i2c_client *client)
{

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

