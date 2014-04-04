/*
 * drivers/input/touchscreen/tsc2007-gta04.c
 *
 * Copyright (c) 2008 MtekVision Co., Ltd.
 *	Kwangwoo Lee <kwlee@mtekvision.com>
 *
 * Using code from:
 *  - ads7846.c
 *	Copyright (c) 2005 David Brownell
 *	Copyright (c) 2006 Nokia Corporation
 *  - corgi_ts.c
 *	Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *	Copyright (C) 2002 MontaVista Software
 *	Copyright (C) 2004 Texas Instruments
 *	Copyright (C) 2005 Dirk Behme
 *	Copyright (C) 2011 Nikolaus Schaller - gta04 extensions (/sys entry, sample AUX & TEMP even if touch is not down)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c/tsc2007.h>

#define TS_POLL_DELAY			1 /* ms delay between IRQ and first sample */
#define TS_POLL_PERIOD			1 /* ms delay between samples */
#define TS_AUX_POLL_PERIOD		100 /* ms delay between samples of AUX, TEMP1, TEMP2 */
#define TS_VREF					1800 /* in mV - should have been defined in the board file! */

#define JITTER_HISTO			5 /* number of historic values to keep */

#define TSC2007_MEASURE_TEMP0		(0x0 << 4)
#define TSC2007_MEASURE_AUX		(0x2 << 4)
#define TSC2007_MEASURE_TEMP1		(0x4 << 4)
#define TSC2007_ACTIVATE_XN		(0x8 << 4)
#define TSC2007_ACTIVATE_YN		(0x9 << 4)
#define TSC2007_ACTIVATE_YP_XN		(0xa << 4)
#define TSC2007_SETUP			(0xb << 4)
#define TSC2007_SETUP_PULLUP_90K	0x01
#define TSC2007_SETUP_BYPASS_MAV	0x02
#define TSC2007_MEASURE_TEMP0	(0x0 << 4)
#define TSC2007_MEASURE_AUX		(0x2 << 4)
#define TSC2007_MEASURE_TEMP1	(0x4 << 4)
#define TSC2007_MEASURE_X		(0xc << 4)
#define TSC2007_MEASURE_Y		(0xd << 4)
#define TSC2007_MEASURE_Z1		(0xe << 4)
#define TSC2007_MEASURE_Z2		(0xf << 4)

#define TSC2007_POWER_OFF_IRQ_EN	(0x0 << 2)
#define TSC2007_ADC_ON_IRQ_DIS0		(0x1 << 2)
#define TSC2007_ADC_OFF_IRQ_EN		(0x2 << 2)
#define TSC2007_ADC_ON_IRQ_DIS1		(0x3 << 2)

#define TSC2007_12BIT			(0x0 << 1)
#define TSC2007_8BIT			(0x1 << 1)

#define	MAX_12BIT			((1 << 12) - 1)

#define ADC_ON_12BIT	(TSC2007_12BIT | TSC2007_ADC_ON_IRQ_DIS0)

#define READ_Y		(ADC_ON_12BIT | TSC2007_MEASURE_Y)
#define READ_Z1		(ADC_ON_12BIT | TSC2007_MEASURE_Z1)
#define READ_Z2		(ADC_ON_12BIT | TSC2007_MEASURE_Z2)
#define READ_X		(ADC_ON_12BIT | TSC2007_MEASURE_X)
#define READ_TEMP0	(ADC_ON_12BIT | TSC2007_MEASURE_TEMP0)
#define READ_TEMP1	(ADC_ON_12BIT | TSC2007_MEASURE_TEMP1)
#define READ_AUX	(ADC_ON_12BIT | TSC2007_MEASURE_AUX)
#define PWRDOWN		(TSC2007_12BIT | TSC2007_POWER_OFF_IRQ_EN)

struct ts_event {
	u16	x;
	u16	y;
	u16	z1, z2;
};

struct jitterbug {
	s32 val;	/* history of input values */
	s32 out;	/* history of output values */
};

struct tsc2007 {
	struct input_dev	*input;
	char			phys[32];
	struct delayed_work	work;
	struct delayed_work	aux_work;

	struct i2c_client	*client;

	u16			model;
	u16			x_plate_ohms;

	u16			min_x;
	u16			min_y;
	u16			min_z;
	u16			max_x;
	u16			max_y;
	u16			max_z;
	bool		flip_x;
	bool		flip_y;
	bool		swap_xy;

	bool		pendown;
	int			irq;

	int			(*get_pendown_state)(void);
	void		(*clear_penirq)(void);

	struct ts_event tc;

	int			aux_counter;

	s16			temperature;	/* in degrees C */

	u16			temp0;
	u16			temp1;
	u16			aux;
	u16			pressure;

	struct jitterbug jitterbufx[JITTER_HISTO];
	struct jitterbug jitterbufy[JITTER_HISTO];

};

/* /sys extension to access TEMP and AUX and many other values */

static ssize_t show_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tsc2007	*ts = i2c_get_clientdata(client);
	return sprintf(buf, "%u,%u,%u,%d,%d,%u,%u,%u,%u,%u,%u\n",
				   ts->tc.x,
				   ts->tc.y,
				   ts->pressure,
				   ts->pendown,
				   ts->temperature,
				   ts->tc.z1,
				   ts->tc.z2,
				   ts->temp0,
				   ts->temp1,
				   ts->aux,
				   ts->pressure > 0 ? ((4096/16) * (u32)ts->x_plate_ohms) / ts->pressure : 65535);
}

static DEVICE_ATTR(values, S_IRUGO, show_data, NULL);

static struct attribute *tsc2007_attributes[] = {
	&dev_attr_values.attr,
	NULL
};

static const struct attribute_group tsc2007_attr_group = {
	.attrs = tsc2007_attributes,
};

/* i2c/smbus access */
static inline int tsc2007_xfer(struct tsc2007 *ts, u8 cmd)
{
	s32 data;
	u16 val;

	data = i2c_smbus_read_word_data(ts->client, cmd);
	if (data < 0) {
		dev_err(&ts->client->dev, "i2c io error: %d\n", data);
		return data;
	}

	/* The protocol and raw data format from i2c interface:
	 * S Addr Wr [A] Comm [A] S Addr Rd [A] [DataLow] A [DataHigh] NA P
	 * Where DataLow has [D11-D4], DataHigh has [D3-D0 << 4 | Dummy 4bit].
	 */
	val = swab16(data) >> 4;

	dev_dbg(&ts->client->dev, "data: 0x%x, val: 0x%x\n", data, val);

	return val;
}

static void tsc2007_read_temp(struct tsc2007 *ts) {
	u32 v1, v2;	/* voltage in mV */
	ts->temp0 = tsc2007_xfer(ts, READ_TEMP0);
	ts->temp1 = tsc2007_xfer(ts, READ_TEMP1);
	v1 = (TS_VREF * (u32)ts->temp0) / 4096;
	v2 = (TS_VREF * (u32)ts->temp1) / 4096;
	ts->temperature = (2573 * (v2-v1)) / 100 - 2730; /* in tenths of degrees C */
	ts->aux = tsc2007_xfer(ts, READ_AUX);
	ts->aux_counter=0;
}

static void tsc2007_read_values(struct tsc2007 *ts)
{
	/* y- still on; turn on only y+ (and ADC) */
	ts->tc.y = tsc2007_xfer(ts, READ_Y);

	/* turn y- off, x+ on, then leave in lowpower */
	ts->tc.x = tsc2007_xfer(ts, READ_X);

	/* turn y+ off, x- on; we'll use formula #1 */
	ts->tc.z1 = tsc2007_xfer(ts, READ_Z1);
	ts->tc.z2 = tsc2007_xfer(ts, READ_Z2);

	/* keep slowly updating while we don't have scheduled the aux_work */
	if(ts->aux_counter++ >= (TS_AUX_POLL_PERIOD/TS_POLL_PERIOD))
		tsc2007_read_temp(ts);

	/* Prepare for next touch reading - power down ADC, enable PENIRQ */
	tsc2007_xfer(ts, PWRDOWN);
}

static u16 tsc2007_calculate_pressure(struct tsc2007 *ts)
{
	u32 rt = 0;

	/* range filtering */
	if (ts->tc.x >= MAX_12BIT)
		return rt;

	if (likely(ts->tc.x && ts->tc.z1 && ts->tc.z2 > ts->tc.z1)) {
		/* compute touch pressure resistance using equation #1 */
		/* and translate into increasing pressure for decreasing resistance */

		rt = ((u32) ts->tc.z1) << (32 - 12);	/* shift to maximum precision */
		rt /= (ts->tc.x * (u32)(ts->tc.z2 - ts->tc.z1));
#if 0
		printk("z1=%u z2=%u x=%u rt=%u\n", ts->tc.z1, ts->tc.z2, ts->tc.x, rt);
#endif
	}

	return rt;
}

static s32 dejitter(s32 value, struct jitterbug *history, int histlen, bool first)
{ /* dejitter a single coordinate */
	int i;
	s32 out;

#define CONFIG_TSC2007_GTA04_DEJITTER_FIR_RESET

	/* dejitter algorithm based on new value and history of old values */

#if defined(CONFIG_TSC2007_GTA04_DEJITTER_NONE)
	out = value;	/* no dejitter */
#elif defined(CONFIG_TSC2007_GTA04_DEJITTER_FIR)
	out = (value + history[0].val) / 2;	/* average with previous input value (FIR filter) */
#elif defined(CONFIG_TSC2007_GTA04_DEJITTER_IIR)
	out = (value + history[0].out) / 2;	/* average with previous output value (IIR filter) */
#elif defined(CONFIG_TSC2007_GTA04_DEJITTER_FIR_RESET)
	if(first)
		out = history[0].val = history[1].val = history[2].val = value;	/* overwrite history */
	else
		out = (8*value + 4*history[0].val + 3*history[1].val + 2*history[2].val + 1*history[3].val) / (8+4+3+2+1);
#elif defined(CONFIG_TSC2007_GTA04_DEJITTER_NEW)
	/* more algorithms can be tested */
#endif

	/* update history */
	for(i=histlen-1; i>0; i--)
		history[i]=history[i-1];	/* move */
	history[0].val = value;
	history[0].out = out;
	return out;
}

static void tsc2007_dejitter(struct tsc2007 *ts, bool first)
{ /* dejitter all relevant channels */
	ts->tc.x = dejitter(ts->tc.x, ts->jitterbufx, JITTER_HISTO, first);
	ts->tc.y = dejitter(ts->tc.y, ts->jitterbufy, JITTER_HISTO, first);
	/* dejitter pressure ? */
}

static void tsc2007_shape_values(struct tsc2007 *ts)
{
	/* Get the read values in the correct calibrated range. */

	if(ts->swap_xy) { /* swap before applying the range limits */
		u16 h = ts->tc.x;
		ts->tc.x = ts->tc.y;
		ts->tc.y = h;
	}

	/* X */
	if(ts->tc.x > ts->max_x)
		ts->tc.x = ts->max_x;
	else if(ts->tc.x < ts->min_x)
		ts->tc.x = ts->min_x;

	if(ts->flip_x)
		ts->tc.x = (ts->max_x - ts->tc.x) + ts->min_x;

	/* Y */
	if(ts->tc.y > ts->max_y)
		ts->tc.y = ts->max_y;
	else if(ts->tc.y < ts->min_y)
		ts->tc.y = ts->min_y;

	if(ts->flip_y)
		ts->tc.y = (ts->max_y - ts->tc.y) + ts->min_y;

	/* Z */
	if(ts->pressure > ts->max_z)
		ts->pressure = ts->max_z;
	else if(ts->pressure < ts->min_z)
		ts->pressure = ts->min_z;

}

static void tsc2007_send_up_event(struct tsc2007 *ts)
{
	struct input_dev *input = ts->input;

	dev_dbg(&ts->client->dev, "UP\n");

	input_report_key(input, BTN_TOUCH, 0);
	input_report_abs(input, ABS_PRESSURE, 0);
	input_sync(input);

	ts->pressure = 0;
}

static void tsc2007_aux_work(struct work_struct *aux_work)
{ /* read TEMP0, TEMP1, AUX */
	struct tsc2007 *ts =
	container_of(to_delayed_work(aux_work), struct tsc2007, aux_work);
/*	printk("tsc2007_aux_work\n"); */

	tsc2007_read_temp(ts);

	/* Prepare for next touch reading - power down ADC, enable PENIRQ */
	tsc2007_xfer(ts, PWRDOWN);

	schedule_delayed_work(&ts->aux_work,
						  msecs_to_jiffies(TS_AUX_POLL_PERIOD));
/*	printk("tsc2007_aux_work done\n"); */
}

static void tsc2007_work(struct work_struct *work)
{
	struct tsc2007 *ts =
		container_of(to_delayed_work(work), struct tsc2007, work);

/*	printk("tsc2007_work\n"); */

	/*
	 * NOTE: We can't rely on the pressure to determine the pen down
	 * state, even though this controller has a pressure sensor.
	 * The pressure value can fluctuate for quite a while after
	 * lifting the pen and in some cases may not even settle at the
	 * expected value.
	 *
	 * The only safe way to check for the pen up condition is in the
	 * work function by reading the pen signal state (it's a GPIO
	 * and IRQ). Unfortunately such callback is not always available,
	 * in that case we have rely on the pressure anyway.
	 */
	if (ts->get_pendown_state) {
		if (unlikely(!ts->get_pendown_state())) {
			tsc2007_send_up_event(ts);
			ts->pendown = false;
			goto out;
		}

		dev_dbg(&ts->client->dev, "pen is still down\n");
	}

	tsc2007_read_values(ts);

	ts->pressure = tsc2007_calculate_pressure(ts);

	if (ts->pressure > 0) {
		struct input_dev *input = ts->input;

		if (!ts->pendown) {
			dev_dbg(&ts->client->dev, "DOWN\n");

			input_report_key(input, BTN_TOUCH, 1);
			ts->pendown = true;
			tsc2007_dejitter(ts, true);
		} else {
			tsc2007_dejitter(ts, false);
		}

		tsc2007_shape_values(ts);

		input_report_abs(input, ABS_X, ts->tc.x);
		input_report_abs(input, ABS_Y, ts->tc.y);
		input_report_abs(input, ABS_PRESSURE, ts->pressure);

		input_sync(input);

		dev_dbg(&ts->client->dev, "point(%4d,%4d), pressure (%4u)\n",
					ts->tc.x, ts->tc.y, ts->pressure);

	} else if (!ts->get_pendown_state && ts->pendown) {
		/*
		 * We don't have callback to check pendown state, so we
		 * have to assume that since pressure reported is 0 the
		 * pen was lifted up.
		 */
		tsc2007_send_up_event(ts);
		ts->pendown = false;
	}

 out:
	if (ts->pendown)
		schedule_delayed_work(&ts->work,
				      msecs_to_jiffies(TS_POLL_PERIOD));
	else {
		schedule_delayed_work(&ts->aux_work,
							  msecs_to_jiffies(TS_AUX_POLL_PERIOD));	/* restart slow updates */
		ts->aux_counter = 0;
		enable_irq(ts->irq);
	}
/*	printk("tsc2007_work done\n"); */
}

static irqreturn_t tsc2007_irq(int irq, void *handle)
{
	struct tsc2007 *ts = handle;
/*	printk("tsc2007_irq\n"); */
	if (!ts->get_pendown_state || likely(ts->get_pendown_state())) {
		disable_irq_nosync(ts->irq);
		cancel_delayed_work(&ts->aux_work);
		schedule_delayed_work(&ts->work,
				      msecs_to_jiffies(TS_POLL_DELAY));
	}

	if (ts->clear_penirq)
		ts->clear_penirq();

	return IRQ_HANDLED;
}

static void tsc2007_free_irq(struct tsc2007 *ts)
{
	free_irq(ts->irq, ts);
	if (cancel_delayed_work_sync(&ts->work)) {
		/*
		 * Work was pending, therefore we need to enable
		 * IRQ here to balance the disable_irq() done in the
		 * interrupt handler.
		 */
		enable_irq(ts->irq);
	}
}

static int tsc2007_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct tsc2007 *ts;
	struct tsc2007_platform_data *pdata = pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	int err;

	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &tsc2007_attr_group);
	if (err) {
		dev_err(&client->dev, "registering with sysfs failed!\n");
		return err;
	}

	ts = kzalloc(sizeof(struct tsc2007), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	ts->client = client;
	ts->irq = client->irq;
	ts->input = input_dev;
	INIT_DELAYED_WORK(&ts->work, tsc2007_work);
	INIT_DELAYED_WORK(&ts->aux_work, tsc2007_aux_work);

	ts->model             = pdata->model;
	ts->x_plate_ohms      = pdata->x_plate_ohms;
	ts->min_x             = pdata->min_x ? : 0;
	ts->min_y             = pdata->min_y ? : 0;
	ts->min_z             = pdata->min_z ? : 0;
	ts->max_x             = pdata->max_x ? : MAX_12BIT;
	ts->max_y             = pdata->max_y ? : MAX_12BIT;
	ts->max_z             = pdata->max_z ? : MAX_12BIT;
	ts->flip_x            = pdata->flip_x;
	ts->flip_y            = pdata->flip_y;
	ts->swap_xy           = pdata->swap_xy;
	ts->get_pendown_state = pdata->get_pendown_state;
	ts->clear_penirq      = pdata->clear_penirq;

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input0", dev_name(&client->dev));

	input_dev->name = "TSC2007 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input_dev, ABS_X, pdata->min_x, pdata->max_x, pdata->fuzz_x, 0);
	input_set_abs_params(input_dev, ABS_Y, pdata->min_y, pdata->max_y, pdata->fuzz_y, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, pdata->min_z, pdata->max_z, pdata->fuzz_z, 0);

	if (pdata->init_platform_hw)
		pdata->init_platform_hw();

	err = request_irq(ts->irq, tsc2007_irq, 0,
			client->dev.driver->name, ts);
	if (err < 0) {
		dev_err(&client->dev, "irq %d busy?\n", ts->irq);
		goto err_free_mem;
	}

	/* Prepare for touch readings - power down ADC and enable PENIRQ */
	err = tsc2007_xfer(ts, PWRDOWN);
	if (err < 0)
		goto err_free_irq;

	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

	i2c_set_clientdata(client, ts);

	/* schedule AUX and TEMP readings */
	schedule_delayed_work(&ts->aux_work,
						  msecs_to_jiffies(TS_AUX_POLL_PERIOD));

	return 0;

 err_free_irq:
	tsc2007_free_irq(ts);
	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();
 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

static int tsc2007_remove(struct i2c_client *client)
{
	struct tsc2007	*ts = i2c_get_clientdata(client);
	struct tsc2007_platform_data *pdata = client->dev.platform_data;

	tsc2007_free_irq(ts);

	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();

	sysfs_remove_group(&client->dev.kobj, &tsc2007_attr_group);

	input_unregister_device(ts->input);
	kfree(ts);

	return 0;
}

static struct i2c_device_id tsc2007_idtable[] = {
	{ "tsc2007", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tsc2007_idtable);

static struct i2c_driver tsc2007_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tsc2007"
	},
	.id_table	= tsc2007_idtable,
	.probe		= tsc2007_probe,
	.remove		= tsc2007_remove,
};

static int __init tsc2007_init(void)
{
	return i2c_add_driver(&tsc2007_driver);
}

static void __exit tsc2007_exit(void)
{
	i2c_del_driver(&tsc2007_driver);
}

module_init(tsc2007_init);
module_exit(tsc2007_exit);

MODULE_AUTHOR("Kwangwoo Lee <kwlee@mtekvision.com>");
MODULE_DESCRIPTION("TSC2007 TouchScreen Driver");
MODULE_LICENSE("GPL");
