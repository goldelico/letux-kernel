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
#include <linux/input-polldev.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "industrialio-inputbridge.h"

/* currently, only polling is implemented */
#define POLLING_MSEC	100

struct iio_input_map {
	struct input_polled_dev *poll_dev;	/* the input device */
	struct iio_channel channels[3];		/* x, y, z channels */
	struct matrix {
		int mxx, myx, mzx;	/* fixed point mount-matrix */
		int mxy, myy, mzy;
		int mxz, myz, mzz;
	} matrix;
};

static inline struct iio_input_map *to_iio_input_map(
		struct iio_channel *channel)
{
	return (struct iio_input_map *) channel->data;
}

/* minimum and maximum range we want to report */
#define ABSMAX_ACC_VAL		(512 - 1)
#define ABSMIN_ACC_VAL		-(ABSMAX_ACC_VAL)

/* scale processed iio values so that 1g maps to ABSMAX_ACC_VAL / 2 */
#define SCALE			((100 * ABSMAX_ACC_VAL) / (2 * 981))

/*
 * convert float string to scaled fixed point format, e.g.
 *   1		-> 1000		(value passed as unit)
 *   1.23	-> 1230
 *   0.1234	->  123
 *   -.01234	->  -12
 */

static int32_t atofix(const char *str, uint32_t unit)
{
	int32_t mantissa = 0;
	bool sign = false;
	bool decimal = false;
	int32_t divisor = 1;

	if (*str == '-')
		sign = true, str++;
	while (*str && divisor < unit) {
		if (*str >= '0' && *str <= '9') {
			mantissa = 10 * mantissa + (*str - '0');
			if (decimal)
				divisor *= 10;
		} else if (*str == '.')
			decimal = true;
		else
			return 0;	/* error */
		str++;
	}

	mantissa = (mantissa * unit) / divisor;
	if (sign)
		mantissa = -mantissa;

	return mantissa;
}

static void iio_apply_matrix(struct matrix *m, int *in, int *out, uint32_t unit)
{
	/* apply mount matrix */
	out[0] = (m->mxx * in[0] + m->myx * in[1] + m->mzx * in[2]) / unit;
	out[1] = (m->mxy * in[0] + m->myy * in[1] + m->mzy * in[2]) / unit;
	out[2] = (m->mxz * in[0] + m->myz * in[1] + m->mzz * in[2]) / unit;
}

#define FIXED_POINT_UNIT	1000	/* seems reasonable for accelerometer input */

static void iio_accel_poll(struct input_polled_dev *dev)
{
	struct iio_input_map *map = dev->private;
	struct input_dev *input = dev->input;

	int values[3];		/* values while processing */
	int aligned_values[3];	/* mount matrix applied */

	int cindex = 0;

printk("%s: map=%px input=%px\n", __func__, map, input);

	while (cindex < ARRAY_SIZE(values)) {
		struct iio_channel *channel =
			&map->channels[cindex];
		int val;
		int ret;

		if (!channel->indio_dev) {
			values[cindex] = 0;
			continue;
		}

		ret = iio_read_channel_raw(channel, &val);

		if (ret < 0) {
			pr_err("%s(): channel read error %d\n",
				__func__, cindex);
			return;
		}

		ret = iio_convert_raw_to_processed(channel, val,
				 &values[cindex], SCALE);

		if (ret < 0) {
			pr_err("%s(): channel processing error\n",
				__func__);
			return;
		}

		cindex++;
	}

	iio_apply_matrix(&map->matrix, values, aligned_values, FIXED_POINT_UNIT);

	input_report_abs(input, ABS_X, aligned_values[0]);
	input_report_abs(input, ABS_Y, aligned_values[1]);
	input_report_abs(input, ABS_Z, aligned_values[2]);
	input_sync(input);
}

static int dindex=0;	/* assign unique names to accel/input devices */

static int iio_input_register_accel_channel(struct iio_dev *indio_dev,
		 const struct iio_chan_spec *chan)
{ /* we found some accelerometer channel */
	int ret;
	int cindex;
	struct iio_input_map *map = iio_device_get_drvdata(indio_dev);

printk("%s: map=%px\n", __func__, map);

	if (!map) {
		struct input_polled_dev *poll_dev;
		const struct iio_chan_spec_ext_info *ext_info;

		map = devm_kzalloc(&indio_dev->dev, sizeof(struct iio_input_map), GFP_KERNEL);
		if (!map)
			return -ENOMEM;

		iio_device_set_drvdata(indio_dev, map);

		poll_dev = devm_input_allocate_polled_device(&indio_dev->dev);
		if (!poll_dev)
			return -ENOMEM;

		poll_dev->private = map;
		poll_dev->poll = iio_accel_poll;
		poll_dev->poll_interval = POLLING_MSEC;

		poll_dev->input->name = kasprintf(GFP_KERNEL, "iio-bridge: %s",
						    indio_dev->name);
		poll_dev->input->phys = kasprintf(GFP_KERNEL, "accel/input%d",
						    dindex++);

// do we need something like this?
//		poll_dev->input->id.bustype = BUS_IIO;
//		poll_dev->input->id.vendor = 0x0001;
//		poll_dev->input->id.product = 0x0001;
//		poll_dev->input->id.version = 0x0001;

		set_bit(INPUT_PROP_ACCELEROMETER, poll_dev->input->propbit);
		poll_dev->input->evbit[0] = BIT_MASK(EV_ABS);
		input_alloc_absinfo(poll_dev->input);
		input_set_abs_params(poll_dev->input, ABS_X, ABSMIN_ACC_VAL,
					ABSMAX_ACC_VAL, 0, 0);
		input_set_abs_params(poll_dev->input, ABS_Y, ABSMIN_ACC_VAL,
					ABSMAX_ACC_VAL, 0, 0);
		input_set_abs_params(poll_dev->input, ABS_Z, ABSMIN_ACC_VAL,
					ABSMAX_ACC_VAL, 0, 0);

		map->poll_dev = poll_dev;

		ret = input_register_polled_device(poll_dev);

		if (ret < 0) {
			kfree(poll_dev->input->name);
			kfree(poll_dev->input->phys);
			return ret;
		}

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

			map->matrix.mxx = atofix(mtx->rotation[0], FIXED_POINT_UNIT);
			map->matrix.myx = atofix(mtx->rotation[1], FIXED_POINT_UNIT);
			map->matrix.mzx = atofix(mtx->rotation[2], FIXED_POINT_UNIT);
			map->matrix.mxy = atofix(mtx->rotation[3], FIXED_POINT_UNIT);
			map->matrix.myy = atofix(mtx->rotation[4], FIXED_POINT_UNIT);
			map->matrix.mzy = atofix(mtx->rotation[5], FIXED_POINT_UNIT);
			map->matrix.mxz = atofix(mtx->rotation[6], FIXED_POINT_UNIT);
			map->matrix.myz = atofix(mtx->rotation[7], FIXED_POINT_UNIT);
			map->matrix.mzz = atofix(mtx->rotation[8], FIXED_POINT_UNIT);
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

// brauchen wir das noch? Oder nehmen wir einfach an dass es 3 Kanäle gibt?

	/* find free channel within this device */

	for (cindex = 0; cindex < ARRAY_SIZE(map->channels); cindex++) {
		if (!map->channels[cindex].indio_dev)
			break;
	}

	/* check if we already have collected enough channels */
	if (cindex == ARRAY_SIZE(map->channels))
		return 0;	/* silently ignore */

	map->channels[cindex].indio_dev = indio_dev;
	map->channels[cindex].channel = chan;
	map->channels[cindex].data = map;

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
	struct iio_input_map *map = iio_device_get_drvdata(indio_dev);
	struct input_dev *input = map->poll_dev->input;

	kfree(input->name);
	kfree(input->phys);
	input_unregister_polled_device(map->poll_dev);
}

MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("Bridge to present Industrial I/O accelerometers as properly oriented Input devices");
MODULE_LICENSE("GPL v2");
