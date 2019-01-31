/*
 * The Industrial I/O core, bridge to input devices
 *
 * Copyright (c) 2018 Golden Delicious Computers GmbH&Co. KG
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "industrialio-inputbridge.h"

/* currently, only polling is implemented */
#define CONFIG_POLLING 1

/* handle up to this number of input devices */
#define MAX_INPUT_DEVICES	5

static struct iio_input_map {
	struct iio_dev *indio_dev;	/* the iio device */
	struct input_dev *input;	/* the input device */
	struct iio_channel channels[3];	/* x, y, z channels */
	int values[3];		/* values while processing */
	int m11, m12, m13;	/* translated and scaled mount-matrix */
	int m21, m22, m23;
	int m31, m32, m33;
} devices[MAX_INPUT_DEVICES];

static inline struct iio_input_map *to_iio_input_map(struct iio_channel *channel)
{
	return (struct iio_input_map *) channel->data;
}

/* we must protect against races in channel allocation */

static DEFINE_MUTEX(inputbridge_channel_mutex);

#if CONFIG_POLLING
static struct delayed_work input_work;

/* we can start/stop the worker by open("/dev/input/event") */
static int open_count = 0;

/* we must protect the open counter */
static DEFINE_MUTEX(inputbridge_open_mutex);
#endif

/* minimum and maximum range we want to report */
#define ABSMAX_ACC_VAL		(512 - 1)
#define ABSMIN_ACC_VAL		-(ABSMAX_ACC_VAL)

/* scale processed iio values so that 1g maps to ABSMAX_ACC_VAL / 2 */
#define SCALE			((100 * ABSMAX_ACC_VAL) / (2 * 981))

static int32_t atofix(const char *str)
{ /* convert float string to scaled fixed point format, e.g. 1.23 -> 1230; 0.1234 -> 123 */
	int32_t mant = 0;
	bool sign = false;
	bool decimal = false;
	int32_t divisor = 1;

	if (*str == '-')
		sign = true, str++;
	while (*str && divisor < 1000) {
		if (*str >= '0' && *str <= '9') {
			mant = 10 * mant + (*str - '0');
			if (decimal)
				divisor *= 10;
		} else if(*str == '.')
			decimal = true;
		else
			return 0;	/* error */
		str++;
	}

	mant = (1000 * mant) / divisor;

	return (sign ? -mant : mant);
}

static void iio_apply_matrix(struct iio_channel *channel, int *in, int *out)
{
// FIXME: search and conversion from string-float to int should be done only once!

	/* assume all channels of a device share the same matrix */

	const struct iio_chan_spec_ext_info *ext_info;

	printk("in: %d, %d, %d\n", in[0], in[1], in[2]);

	for (ext_info = channel->channel->ext_info; ext_info->name; ext_info++) {
		printk("ext_info: %s\n", ext_info->name);
		if (strcmp(ext_info->name, "mount_matrix") == 0)
			break;
	}

	if (ext_info->name) {
		uintptr_t priv = ext_info->private;
		const struct iio_mount_matrix *mtx;

		int m11 = 1000, m12 = 0, m13 = 0;
		int m21 = 0, m22 = 1000, m23 = 0;
		int m31 = 0, m32 = 0, m33 = 1000;

		mtx = ((iio_get_mount_matrix_t *) priv)
			(channel->indio_dev, channel->channel);

/*
		printk("%s, %s, %s; %s, %s, %s; %s, %s, %s\n",
			mtx->rotation[0], mtx->rotation[1], mtx->rotation[2],
			mtx->rotation[3], mtx->rotation[4], mtx->rotation[5],
			mtx->rotation[6], mtx->rotation[7], mtx->rotation[8]);
*/

		m11 = atofix(mtx->rotation[0]);
		m12 = atofix(mtx->rotation[1]);
		m13 = atofix(mtx->rotation[2]);
		m21 = atofix(mtx->rotation[3]);
		m22 = atofix(mtx->rotation[4]);
		m23 = atofix(mtx->rotation[5]);
		m31 = atofix(mtx->rotation[6]);
		m32 = atofix(mtx->rotation[7]);
		m33 = atofix(mtx->rotation[8]);

/*
		printk("%d, %d, %d; %d, %d, %d; %d, %d, %d\n",
			m11, m12, m13,
			m21, m22, m23,
			m31, m32, m33);
*/

		/* apply mount matrix */
		out[0] = (m11 * in[0] + m12 * in[1] + m13 * in[2]) / 1000;
		out[1] = (m21 * in[0] + m22 * in[1] + m23 * in[2]) / 1000;
		out[2] = (m31 * in[0] + m32 * in[1] + m33 * in[2]) / 1000;

	} else {

		/* if no matrix found, apply identity */
		out[0] = in[0];
		out[1] = in[1];
		out[2] = in[2];

	}

	printk("out: %d, %d, %d\n", out[0], out[1], out[2]);
}

static void accel_report_channels(void)
{
	int dindex;

	for (dindex = 0; dindex < ARRAY_SIZE(devices); dindex++) {
		struct iio_input_map *map = &devices[dindex];

		/* device might become closed while we are still processing */
		mutex_lock(&inputbridge_channel_mutex);

		if (map->input) {
			int oriented_values[3];
			int cindex;

			for (cindex = 0; cindex < ARRAY_SIZE(map->channels); cindex++) {
				struct iio_channel *channel = &map->channels[cindex];
				int val;
				int ret;

				if (!channel->indio_dev)
					continue;

				ret = iio_read_channel_raw(channel, &val);

#if 0
printk("accel_report_channel %d -> %d ret=%d\n", cindex, val, ret);
#endif

				if (ret < 0) {
					pr_err("accel channel read error %d\n", cindex);
					return;
				}

				ret = iio_convert_raw_to_processed(channel, val, &map->values[cindex], SCALE);

				if (ret < 0) {
					pr_err("accel channel processing error\n");
					return;
				}
			}

			iio_apply_matrix(&map->channels[0], map->values, oriented_values);

			input_report_abs(map->input, ABS_X, oriented_values[0]);
			input_report_abs(map->input, ABS_Y, oriented_values[1]);
			input_report_abs(map->input, ABS_Z, oriented_values[2]);
			input_sync(map->input);

			mutex_unlock(&inputbridge_channel_mutex);
		}
	}

}

#if CONFIG_POLLING
static void inputbridge_work(struct work_struct *work)
{
	struct delayed_work *delayed_work;

#if 0
	printk("inputbridge_work\n");
#endif

	delayed_work = to_delayed_work(work);
//	something = container_of(delayed_work, struct something, something_work);

	accel_report_channels();

	schedule_delayed_work(&input_work,
		msecs_to_jiffies(100));	// poll with 10 Hz
}
#endif

static int accel_open(struct input_dev *input)
{
	struct iio_dev *iiodev = input_get_drvdata(input);

#if 0
printk("accel_open()\n");
#endif

	// someone has opened an input device
	// make us start the associated iio_dev

#if CONFIG_POLLING
	mutex_lock(&inputbridge_open_mutex);
	if (open_count++ == 0)
		schedule_delayed_work(&input_work,
			msecs_to_jiffies(0));	// start now on first open
	mutex_unlock(&inputbridge_open_mutex);
#else
	int iio_channel_start_all_cb(struct iio_cb_buffer *cb_buff);
#endif

	return 0;
}

static void accel_close(struct input_dev *input)
{
	struct iio_dev *iiodev = input_get_drvdata(input);

#if 0
	printk("accel_close()\n");
#endif

#if CONFIG_POLLING
	mutex_lock(&inputbridge_open_mutex);
	if (open_count > 0) {
		cancel_delayed_work(&input_work);
		open_count--;
	}
	mutex_unlock(&inputbridge_open_mutex);
#else
	int iio_channel_stop_all_cb(struct iio_cb_buffer *cb_buff);
#endif
}

static int iio_input_register_accel_channel(struct iio_dev *indio_dev, const struct iio_chan_spec *chan)
{ // we found some accelerometer channel!
	int error;
	struct input_dev *input = NULL;

	int dindex, cindex;

#if 0
	printk("iio_device_register_inputbridge(): found an accelerometer\n");
#endif

	mutex_lock(&inputbridge_channel_mutex);

	/* look for existing input device */

	for (dindex = 0; dindex < ARRAY_SIZE(devices); dindex++) {
		if (devices[dindex].indio_dev == indio_dev) {
			break;
		}
	}

	if (dindex == ARRAY_SIZE(devices)) {
		/* not found, look for a free slot for a new input device */

		for (dindex = 0; dindex < ARRAY_SIZE(devices); dindex ++) {
			if (devices[dindex].indio_dev == NULL)
				break;
		}

		if (dindex == ARRAY_SIZE(devices)) {
			mutex_unlock(&inputbridge_channel_mutex);
			return -ENOMEM;
		}

#if 0
	printk("iio_device_register_inputbridge(): allocate the input dev\n");
#endif

		input = input_allocate_device();

#if 0
	printk("iio_device_register_inputbridge(): => %p\n", input);
#endif

		if (!input) {
			mutex_unlock(&inputbridge_channel_mutex);
			return -ENOMEM;
		}

// FIXME: ask some DT aliases for mapping?

		input->name = kasprintf(GFP_KERNEL, "iio-bridge: %s", indio_dev->name);
		input->phys = kasprintf(GFP_KERNEL, "accel/input%d", dindex);
//		input->id.bustype = BUS_I2C;

//		input->dev.parent = &indio_dev->client->dev;

		input->evbit[0] = BIT_MASK(EV_ABS);
		input->open = accel_open;
		input->close = accel_close;

		// FIXME: what happens if we unregister the first device?
		if (dindex == 0 ) { // first input
#if CONFIG_POLLING
			INIT_DELAYED_WORK(&input_work, inputbridge_work);
#else
			struct iio_cb_buffer *iio_channel_get_all_cb(struct device *dev,
					int (*cb)(const void *data,
						void *private),
						void *private);
#endif
		}

//		indio_dev->input = input;

		input_set_drvdata(input, indio_dev);

		input_alloc_absinfo(input);
		error = input_register_device(input);

#if 0
	printk("iio_device_register_inputbridge(): input_register_device => %d\n", error);
#endif

		if (error < 0) {
			kfree(input->name);
			kfree(input->phys);
			input_free_device(input);
			mutex_unlock(&inputbridge_channel_mutex);
			return error;
		}

		devices[dindex].input = input;
		devices[dindex].indio_dev = indio_dev;
	}

	/* find free channel within this device block */
	for (cindex = 0; cindex < ARRAY_SIZE(devices[dindex].channels); cindex++) {
		if (!devices[dindex].channels[cindex].indio_dev) {
			break;
		}
	}

	if (cindex == ARRAY_SIZE(devices[dindex].channels)) { /* we already have collected 3 channels */
		mutex_unlock(&inputbridge_channel_mutex);
		return 0;	/* silently ignore */
	}

#if 0
	printk("iio_device_register_inputbridge(): process channel %d of device %d\n", cindex, dindex);
#endif

	devices[dindex].channels[cindex].indio_dev = indio_dev;
	devices[dindex].channels[cindex].channel = chan;
	devices[dindex].channels[cindex].data = (void *) &devices[dindex];

	switch (cindex) {
		case 0:
			input_set_abs_params(input, ABS_X, ABSMIN_ACC_VAL, ABSMAX_ACC_VAL, 0, 0);
			break;
		case 1:
			input_set_abs_params(input, ABS_Y, ABSMIN_ACC_VAL, ABSMAX_ACC_VAL, 0, 0);
			break;
		case 2:
			input_set_abs_params(input, ABS_Z, ABSMIN_ACC_VAL, ABSMAX_ACC_VAL, 0, 0);
			break;
	}
	mutex_unlock(&inputbridge_channel_mutex);

	return 0;
}

int iio_device_register_inputbridge(struct iio_dev *indio_dev)
{
	int i;

#if 0
	printk("iio_device_register_inputbridge()\n");
#endif

	for (i = 0; i < indio_dev->num_channels; i++) {
		const struct iio_chan_spec *chan =
				&indio_dev->channels[i];

#if 0
		printk("iio_device_register_inputbridge(): %d %d %s %s %d\n",
			i, chan->channel,
			chan->extend_name, chan->datasheet_name,
			chan->type);
#endif

		if (chan->type == IIO_ACCEL) {
			int r = iio_input_register_accel_channel(indio_dev, chan);
			if (r < 0)
				return r;
		}
	}

	return 0;
}

void iio_device_unregister_inputbridge(struct iio_dev *indio_dev)
{
	struct input_dev *input = NULL;

	int dindex, cindex;

	mutex_lock(&inputbridge_channel_mutex);

	for (dindex = 0; dindex < ARRAY_SIZE(devices); dindex++) {
		for (cindex = 0; cindex < ARRAY_SIZE(devices[dindex].channels); cindex++) {
			struct iio_channel *channel = &devices[dindex].channels[cindex];

			if (channel->indio_dev == indio_dev) {
				channel->indio_dev = NULL;	/* mark slot as empty */
			}
		}
		input_unregister_device(input);
		kfree(input->name);
		kfree(input->phys);
		input_free_device(input);

		devices[dindex].indio_dev = NULL;
		devices[dindex].input = NULL;
	}

	mutex_unlock(&inputbridge_channel_mutex);
}

MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("Bridge to present Industrial I/O accelerometers as properly oriented Input devices");
MODULE_LICENSE("GPL v2");
