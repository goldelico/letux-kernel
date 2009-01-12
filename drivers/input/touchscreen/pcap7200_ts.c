 /* Projected capacitive touchscreen controller driver.
 *
 * Copyright(c) 2008 Openmoko Inc.
 *
 * Author: Matt Hsu <matt_hsu@openmoko.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * TODO
 * - apply ts_filter
 * - add support for gesture event
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/input.h>

#include <linux/pcap7200.h>
#include <linux/ts_filter.h>

#include <mach/om-gta03.h>

#define PCAP7200_OP_MODE_REG	0x07
#define RPT_PKT_SIZE 	5
#define EVENT_UP 	0x80
#define EVENT_DOWN 	0x81


#define coord_interpret(msb_byte, lsb_byte) \
			(msb_byte << 7 | lsb_byte)

struct pcap7200_data{
	struct i2c_client *client;
	struct input_dev *dev;
	struct ts_filter *tsf[MAX_TS_FILTER_CHAIN];
	struct mutex lock;
	int irq;
	struct work_struct work;
};

static void pcap7200_work(struct work_struct *work)
{
	struct pcap7200_data *pcap =
		container_of(work, struct pcap7200_data, work);
	uint8_t rpt_pkt[RPT_PKT_SIZE], event;
	int coords[2];
	int ret;

	mutex_lock(&pcap->lock);

	memset(rpt_pkt, 0, sizeof(rpt_pkt));

	BUG_ON(pcap == NULL);

	ret = i2c_master_recv(pcap->client, rpt_pkt, RPT_PKT_SIZE);

	event = rpt_pkt[0];

	/*
	 * this is annonying. system would receive two consecutive interrupts if we set INT
	 * type as LOW_LEVEL. the data we pull from pcap7200 are invalid which are all
	 * zero in the second interrupt.
	 */
	if (event != 0) {
		dev_dbg(&pcap->client->dev, "[%2x][%2x][%2x][%2x][%2x]: %d bytes return \n",
					rpt_pkt[0], rpt_pkt[1], rpt_pkt[2],
					rpt_pkt[3], rpt_pkt[4], ret);

		coords[0] = coord_interpret(rpt_pkt[1], rpt_pkt[2]);
		coords[1] = coord_interpret(rpt_pkt[3], rpt_pkt[4]);

		if (event == EVENT_DOWN) {
			input_report_abs(pcap->dev, ABS_X, coords[0]);
			input_report_abs(pcap->dev, ABS_Y, coords[1]);
			input_report_key(pcap->dev, BTN_TOUCH, 1);
			input_report_abs(pcap->dev, ABS_PRESSURE, 1);
		} else if (event == EVENT_UP) {
			input_report_key(pcap->dev, BTN_TOUCH, 0);
			input_report_abs(pcap->dev, ABS_PRESSURE, 0);
		} else {
			/* FIMXE: gesture events should be reported here. */
		}

		input_sync(pcap->dev);
	}

	mutex_unlock(&pcap->lock);
	enable_irq(pcap->irq);
}

static const char *op_mode_name[] = {
	[SLEEP] 		= "sleep",
	[WAKEUP] 		= "wakeup",
	[SINGLE_TOUCH] 		= "single_touch",
	[MULTI_TOUCH] 		= "multi_touch",
};

static struct i2c_driver pcap7200_driver;

static int __set_op_mode(struct pcap7200_data *pcap, u_int8_t val)
{
	u8 buf[] = { PCAP7200_OP_MODE_REG, val};
	int ret, i;

	mutex_lock(&pcap->lock);

	/* this chip has an issue.
	 * you need to give wakeup call for 3 times if it's
	 * in sleep mode.
	 */
	if (val == WAKEUP) {
		for (i = 0; i < 3; i++)
			ret = i2c_master_send(pcap->client, &buf, sizeof(buf));
	} else
		ret = i2c_master_send(pcap->client, &buf, sizeof(buf));

	mutex_unlock(&pcap->lock);
	return ret;
}

static ssize_t set_op_mode(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcap7200_data *pcap = i2c_get_clientdata(client);
	u_int8_t i;

	for (i = 0; i < ARRAY_SIZE(op_mode_name); i++) {
		if (!strncmp(buf, op_mode_name[i], strlen(op_mode_name[i])))
			__set_op_mode(pcap, i);
	}

	return count;
}

static DEVICE_ATTR(op_mode, S_IRUGO | S_IWUSR, NULL, set_op_mode);

static irqreturn_t pcap7200_irq(int irq, void *_pcap)
{
	struct pcap7200_data *pcap = _pcap;

	disable_irq(pcap->irq);
	schedule_work(&pcap->work);

	return IRQ_HANDLED;
}

static int
pcap7200_probe(struct i2c_client *client, const struct i2c_device_id *ids)
{
	struct pcap7200_data *pcap;
	struct input_dev *input_dev;
	int err;

	/* allocat pcap7200 data */
	pcap = kzalloc(sizeof(struct pcap7200_data), GFP_KERNEL);
	if (!pcap)
		return -ENOMEM;

	i2c_set_clientdata(client, pcap);
	pcap->client = client;

	mutex_init(&pcap->lock);

	/* initialize input device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "Unable to allocate the input device\n");
		err = -ENOMEM;
		goto exit_kfree;
	}

	pcap->dev = input_dev;
	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) |
					BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	/* configurable resolution 2048x2048 */
	input_set_abs_params(pcap->dev, ABS_X, 0, 0x7FF, 0, 0);
	input_set_abs_params(pcap->dev, ABS_Y, 0, 0x7FF, 0, 0);
	input_set_abs_params(pcap->dev, ABS_PRESSURE, 0, 1, 0, 0);

	input_dev->name = client->name;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	err = input_register_device(input_dev);

	if (err)
		goto exit_unreg;

	INIT_WORK(&pcap->work, pcap7200_work);

	sysfs_create_file(&client->dev.kobj, &dev_attr_op_mode.attr);

	/* setup IRQ */
	if (client->irq < 0) {
		dev_err(&client->dev,
			"No irq allocated in client resources!\n");
		return -EIO;
	}

	pcap->irq = client->irq;
	err = request_irq(pcap->irq, pcap7200_irq, IRQF_TRIGGER_LOW, "pcap7200", pcap);

	if (err < 0)
		goto exit_unreg;

	return 0;

exit_unreg:
	input_unregister_device(input_dev);
exit_kfree:
	kfree(pcap);
	return err;
}

static int pcap7200_remove(struct i2c_client *client)
{
	struct pcap7200_data *pcap = i2c_get_clientdata(client);

	free_irq(pcap->irq, pcap);
	input_unregister_device(pcap->dev);
	kfree(pcap);
	return 0;
}

#ifdef CONFIG_PM
static int pcap7200_suspend(struct device *dev, pm_message_t state)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcap7200_data *pcap = i2c_get_clientdata(client);

	__set_op_mode(pcap, SLEEP);
	return 0;
}

static int pcap7200_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcap7200_data *pcap = i2c_get_clientdata(client);

	__set_op_mode(pcap, WAKEUP);
	return 0;
}
#else
#define pcap7200_suspend NULL
#define pcap7200_resume NULL
#endif

static struct i2c_device_id pcap7200_id_table[] = {
	{"pcap7200", 0x0a},
};

static struct i2c_driver pcap7200_driver = {
	.driver = {
		.name	 = "pcap7200",
		.suspend = pcap7200_suspend,
		.resume	 = pcap7200_resume,
	},
	.id_table	= pcap7200_id_table,
	.probe 		= pcap7200_probe,
	.remove 	= pcap7200_remove,
};

static int __init pcap7200_init(void)
{
	return i2c_add_driver(&pcap7200_driver);
}

static void pcap7200_exit(void)
{
	i2c_del_driver(&pcap7200_driver);
}
module_init(pcap7200_init);
module_exit(pcap7200_exit);

MODULE_AUTHOR("Matt Hsu <matt_hsu@openmoko.org>");
MODULE_LICENSE("GPLv2");
