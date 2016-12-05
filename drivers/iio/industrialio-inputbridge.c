// SPDX-License-Identifier: GPL-2.0
/*
 * The Industrial I/O core, bridge to input devices
 *
 * Copyright (c) 2016-2019 Golden Delicious Computers GmbH&Co. KG
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
#define POLLING_MSEC	100

/* handle up to this number of input devices */
#define MAX_INPUT_DEVICES	5

#define FIXED_POINT_UNIT	1000

static struct iio_input_map {
	struct iio_dev *indio_dev;	/* the iio device */
	struct input_dev *input;	/* the input device */
	struct iio_channel channels[3];	/* x, y, z channels */
	int values[3];		/* values while processing */
	struct matrix {
		int mxx, myx, mzx;	/* translated and scaled mount-matrix */
		int mxy, myy, mzy;
		int mxz, myz, mzz;
	} matrix;
} devices[MAX_INPUT_DEVICES];

static inline struct iio_input_map *to_iio_input_map(
		struct iio_channel *channel)
{
	return (struct iio_input_map *) channel->data;
}

/* we must protect against races in channel allocation */

static DEFINE_MUTEX(inputbridge_device_mutex);

static struct delayed_work input_work;

/* we can start/stop the worker by open("/dev/input/event") */
static int open_count;

/* we must protect the open counter */
static DEFINE_MUTEX(inputbridge_open_mutex);

/* minimum and maximum range we want to report */
#define ABSMAX_ACC_VAL		(512 - 1)
#define ABSMIN_ACC_VAL		-(ABSMAX_ACC_VAL)

/* scale processed iio values so that 1g maps to ABSMAX_ACC_VAL / 2 */
#define SCALE			((100 * ABSMAX_ACC_VAL) / (2 * 981))

/*
 * convert float string to scaled fixed point format, e.g.
 *   1.23	-> 1230
 *   0.1234	->  123
 *   .01234	->   12
 */

static int32_t atofix(const char *str)
{
	int32_t mant = 0;
	bool sign = false;
	bool decimal = false;
	int32_t divisor = 1;

	if (*str == '-')
		sign = true, str++;
	while (*str && divisor < FIXED_POINT_UNIT) {
		if (*str >= '0' && *str <= '9') {
			mant = 10 * mant + (*str - '0');
			if (decimal)
				divisor *= 10;
		} else if (*str == '.')
			decimal = true;
		else
			return 0;	/* error */
		str++;
	}

	mant = (FIXED_POINT_UNIT * mant) / divisor;
	if (sign)
		mant = -mant;

	return mant;
}

static void iio_apply_matrix(struct matrix *m, int *in, int *out)
{
	/* apply mount matrix */
	out[0] = (m->mxx * in[0] + m->myx * in[1] + m->mzx * in[2])
			/ FIXED_POINT_UNIT;
	out[1] = (m->mxy * in[0] + m->myy * in[1] + m->mzy * in[2])
			/ FIXED_POINT_UNIT;
	out[2] = (m->mxz * in[0] + m->myz * in[1] + m->mzz * in[2])
			/ FIXED_POINT_UNIT;
}

static void iio_accel_report_channels(void)
{
	int dindex;

	for (dindex = 0; dindex < ARRAY_SIZE(devices); dindex++) {
		struct iio_input_map *map = &devices[dindex];

		/* device might become closed while we are still processing */
		mutex_lock(&inputbridge_device_mutex);

		if (map->input) {
			int aligned_values[3];
			int cindex = 0;

			while (cindex < ARRAY_SIZE(map->channels)) {
				struct iio_channel *channel =
					&map->channels[cindex];
				int val;
				int ret;

				if (!channel->indio_dev)
					continue;

				ret = iio_read_channel_raw(channel, &val);

				if (ret < 0) {
					pr_err("%s(): channel read error %d\n",
						__func__, cindex);
					return;
				}

				ret = iio_convert_raw_to_processed(channel, val,
						 &map->values[cindex], SCALE);

				if (ret < 0) {
					pr_err("%s(): channel processing error\n",
						__func__);
					return;
				}

				cindex++;
			}

			iio_apply_matrix(&map->matrix, map->values,
					 aligned_values);

			input_report_abs(map->input, ABS_X, aligned_values[0]);
			input_report_abs(map->input, ABS_Y, aligned_values[1]);
			input_report_abs(map->input, ABS_Z, aligned_values[2]);
			input_sync(map->input);
		}

		mutex_unlock(&inputbridge_device_mutex);
	}
}

static void iio_inputbridge_work(struct work_struct *work)
{
	struct delayed_work *delayed_work;

	delayed_work = to_delayed_work(work);

	mutex_lock(&inputbridge_open_mutex);

	iio_accel_report_channels();

	schedule_delayed_work(delayed_work,
		msecs_to_jiffies(POLLING_MSEC));

	mutex_unlock(&inputbridge_open_mutex);
}

static int iio_accel_open(struct input_dev *input)
{
	struct iio_dev *iiodev = input_get_drvdata(input);

	mutex_lock(&inputbridge_open_mutex);

	/* start on first open */
	if (open_count++ == 0)
		schedule_delayed_work(&input_work,
			msecs_to_jiffies(0));

	mutex_unlock(&inputbridge_open_mutex);

	return 0;
}

static void iio_accel_close(struct input_dev *input)
{
	struct iio_dev *iiodev = input_get_drvdata(input);

	mutex_lock(&inputbridge_open_mutex);

	/* stop after last close */
	if (--open_count == 0)
		cancel_delayed_work(&input_work);

	mutex_unlock(&inputbridge_open_mutex);
}

static int iio_input_register_accel_channel(struct iio_dev *indio_dev,
		 const struct iio_chan_spec *chan)
{ /* we found some accelerometer channel */
	int ret;

	int dindex, cindex;
	struct iio_input_map *map;

	mutex_lock(&inputbridge_device_mutex);

	/* look for existing input device for this iio device */

	for (dindex = 0; dindex < ARRAY_SIZE(devices); dindex++) {
		if (devices[dindex].indio_dev == indio_dev)
			break;
	}

	if (dindex == ARRAY_SIZE(devices)) {
		struct input_dev *input;
		const struct iio_chan_spec_ext_info *ext_info;

		/* not found, look for a free slot for a new input device */

		for (dindex = 0; dindex < ARRAY_SIZE(devices); dindex++) {
			if (!devices[dindex].input)
				break;
		}

		if (dindex == ARRAY_SIZE(devices)) {
			mutex_unlock(&inputbridge_device_mutex);
			return -ENOMEM;
		}

		input = input_allocate_device();

		if (!input) {
			mutex_unlock(&inputbridge_device_mutex);
			return -ENOMEM;
		}

		input->name = kasprintf(GFP_KERNEL, "iio-bridge: %s",
						    indio_dev->name);
		input->phys = kasprintf(GFP_KERNEL, "accel/input%d",
						    dindex);

//		input->id.bustype = BUS_I2C;
//		input->dev.parent = &indio_dev->client->dev;

		set_bit(INPUT_PROP_ACCELEROMETER, input->propbit);

		input->evbit[0] = BIT_MASK(EV_ABS);
		input->open = iio_accel_open;
		input->close = iio_accel_close;

// FIXME: what happens if we unregister the first device?
// and then register another one?

		if (dindex == 0) // first input
			INIT_DELAYED_WORK(&input_work, iio_inputbridge_work);

		input_set_drvdata(input, indio_dev);

		input_alloc_absinfo(input);
		ret = input_register_device(input);

		if (ret < 0) {
			kfree(input->name);
			kfree(input->phys);
			input_free_device(input);
			mutex_unlock(&inputbridge_device_mutex);
			return ret;
		}

		map = &devices[dindex];

		map->input = input;
		map->indio_dev = indio_dev;

		/* assume all channels of a device share the same matrix */

		ext_info = chan->ext_info;
		for (; ext_info && ext_info->name; ext_info++) {
			if (strcmp(ext_info->name, "mount_matrix") == 0)
				break;
		}

		if (ext_info && ext_info->name) {
			/* matrix found */
			uintptr_t priv = ext_info->private;
			const struct iio_mount_matrix *mtx;

			mtx = ((iio_get_mount_matrix_t *) priv)(indio_dev,
								chan);

			map->matrix.mxx = atofix(mtx->rotation[0]);
			map->matrix.myx = atofix(mtx->rotation[1]);
			map->matrix.mzx = atofix(mtx->rotation[2]);
			map->matrix.mxy = atofix(mtx->rotation[3]);
			map->matrix.myy = atofix(mtx->rotation[4]);
			map->matrix.mzy = atofix(mtx->rotation[5]);
			map->matrix.mxz = atofix(mtx->rotation[6]);
			map->matrix.myz = atofix(mtx->rotation[7]);
			map->matrix.mzz = atofix(mtx->rotation[8]);
		} else {
			map->matrix.mxx = FIXED_POINT_UNIT;
			map->matrix.myx = 0;
			map->matrix.mzx = 0;
			map->matrix.mxy = 0;
			map->matrix.myy = FIXED_POINT_UNIT;
			map->matrix.mzy = 0;
			map->matrix.mxz = 0;
			map->matrix.myz = 0;
			map->matrix.mzz = FIXED_POINT_UNIT;
		}
	}

	/* find free channel within this device block */
	map = &devices[dindex];

	for (cindex = 0; cindex < ARRAY_SIZE(map->channels); cindex++) {
		if (!map->channels[cindex].indio_dev)
			break;
	}

	/* check if we already have collected enough channels */
	if (cindex == ARRAY_SIZE(map->channels)) {
		mutex_unlock(&inputbridge_device_mutex);
		return 0;	/* silently ignore */
	}

	map->channels[cindex].indio_dev = indio_dev;
	map->channels[cindex].channel = chan;
	map->channels[cindex].data = (void *) &devices[dindex];

	switch (cindex) {
	case 0:
		input_set_abs_params(map->input, ABS_X, ABSMIN_ACC_VAL,
					ABSMAX_ACC_VAL, 0, 0);
		break;
	case 1:
		input_set_abs_params(map->input, ABS_Y, ABSMIN_ACC_VAL,
					ABSMAX_ACC_VAL, 0, 0);
		break;
	case 2:
		input_set_abs_params(map->input, ABS_Z, ABSMIN_ACC_VAL,
					ABSMAX_ACC_VAL, 0, 0);
		break;
	}
	mutex_unlock(&inputbridge_device_mutex);

	return 0;
}

int iio_device_register_inputbridge(struct iio_dev *indio_dev)
{
	int i;

	for (i = 0; i < indio_dev->num_channels; i++) {
		const struct iio_chan_spec *chan =
				&indio_dev->channels[i];

		if (chan->type == IIO_ACCEL) {
			int r = iio_input_register_accel_channel(indio_dev,
								 chan);

			if (r < 0)
				return r;
		}
	}

	return 0;
}

void iio_device_unregister_inputbridge(struct iio_dev *indio_dev)
{
	struct input_dev *input = NULL;

	int dindex = 0;

	mutex_lock(&inputbridge_device_mutex);

	for (; dindex < ARRAY_SIZE(devices); dindex++) {
		int cindex = 0;

		while (cindex < ARRAY_SIZE(devices[dindex].channels)) {
			struct iio_channel *channel =
				&devices[dindex].channels[cindex];

			/* mark slot as empty */
			if (channel->indio_dev == indio_dev)
				channel->indio_dev = NULL;
			cindex++;
		}
		input_unregister_device(input);
		kfree(input->name);
		kfree(input->phys);
		input_free_device(input);

		devices[dindex].indio_dev = NULL;
		devices[dindex].input = NULL;
	}

	mutex_unlock(&inputbridge_device_mutex);
}

MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("Bridge to present Industrial I/O accelerometers as properly oriented Input devices");
MODULE_LICENSE("GPL v2");
