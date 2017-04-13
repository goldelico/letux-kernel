#include <linux/iio/iio.h>
#include <linux/input.h>
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>

#include "industrialio-inputbridge.h"

/* only polling is implemented*/
#define POLLING 1

struct input_dev *idev = NULL;
static struct iio_channel channels[3];
static int channel = 0;

#if POLLING
static struct delayed_work input_work;
#endif

#define ABSMAX_ACC_VAL		((1<<9)-1) /* 10 bit */
#define ABSMIN_ACC_VAL		-(ABSMAX_ACC_VAL)

/* scale processed values so that 1g maps to ABSMAX_ACC_VAL / 2 */
#define SCALE			(100 * ABSMAX_ACC_VAL) / (2 * 981)

static void accel_report_channel(struct input_dev *input, int cindex, int iindex)
{
	int val;
	int ret;

	if (!channels[cindex].indio_dev)
		return;

	ret = iio_read_channel_raw(&channels[cindex], &val);

#if 0
printk("accel_report_channel %d -> %d ret=%d\n", cindex, val, ret);
#endif

	if (ret < 0) {
		pr_err("accel channel read error\n");
		return;
	}

	/*
	 * NOTE: SCALE is ignored by iio_convert_raw_to_processed()
	 * for IIO_VAL_INT so we get wrong values.
	 */

	ret = iio_convert_raw_to_processed(&channels[cindex], val, &val, SCALE);

	if (ret < 0) {
		pr_err("accel channel processing error\n");
		return;
	}

	input_report_abs(input, iindex, val);
}

static void accel_report_xyz(struct input_dev *input)
{
	if (!input)
		return;
	accel_report_channel(input, 0, ABS_X);
	accel_report_channel(input, 1, ABS_Y);
	accel_report_channel(input, 2, ABS_Z);
	input_sync(input);
}

#if POLLING
static void inputbridge_work(struct work_struct *work)
{
	struct delayed_work *delayed_work;

#if 0
	printk("inputbridge_work\n");
#endif

	delayed_work = to_delayed_work(work);
//	adc_bat = container_of(delayed_work, struct gab, bat_work);

	accel_report_xyz((struct input_dev *) channels[0].data);

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

	// someone has opened the input device
	// make us start the iio_dev

#if POLLING
	schedule_delayed_work(&input_work,
		msecs_to_jiffies(0));	// start now
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

#if POLLING
	cancel_delayed_work(&input_work);
#else
	int iio_channel_stop_all_cb(struct iio_cb_buffer *cb_buff);
#endif
}

static int iio_input_register_accel_channel(struct iio_dev *indio_dev, const struct iio_chan_spec *chan)
{
	// we found some accelerometer channel!

#if 0
	printk("iio_device_register_inputbridge(): found an accelerometer\n");
#endif

	if (channel >= 3)
		return 0;	// we already have collected 3 channels

	if (!idev) { // first call
		int error;

#if 0
	printk("iio_device_register_inputbridge(): allocate the input dev\n");
#endif

		idev = input_allocate_device();

#if 0
	printk("iio_device_register_inputbridge(): => %p\n", idev);
#endif

		if (!idev)
			return -ENOMEM;

		idev->name = "accelerometer-iio-input-bridge";
		idev->phys = "accel/input0";
//		idev->id.bustype = BUS_I2C;

//		idev->dev.parent = &indio_dev->client->dev;

		idev->evbit[0] = BIT_MASK(EV_ABS);
		idev->open = accel_open;
		idev->close = accel_close;

		input_set_drvdata(idev, indio_dev);

		input_alloc_absinfo(idev);
		error = input_register_device(idev);

#if 0
	printk("iio_device_register_inputbridge(): input_register_device => %d\n", error);
#endif

		if (error) {
			input_free_device(idev);
			return error;
		}

//		indio_dev->input = idev;

#if POLLING
		INIT_DELAYED_WORK(&input_work, inputbridge_work);
#else
		struct iio_cb_buffer *iio_channel_get_all_cb(struct device *dev,
				int (*cb)(const void *data,
					void *private),
					void *private);
#endif
	}

#if 0
	printk("iio_device_register_inputbridge(): process channel %d\n", channel);
#endif

	channels[channel].indio_dev = indio_dev;
	channels[channel].channel = chan;
	channels[channel].data = (void *) idev;

	switch (channel++) {
		case 0:
			input_set_abs_params(idev, ABS_X, ABSMIN_ACC_VAL, ABSMAX_ACC_VAL, 0, 0);
			break;
		case 1:
			input_set_abs_params(idev, ABS_Y, ABSMIN_ACC_VAL, ABSMAX_ACC_VAL, 0, 0);
			break;
		case 2:
			input_set_abs_params(idev, ABS_Z, ABSMIN_ACC_VAL, ABSMAX_ACC_VAL, 0, 0);
			break;
	}

	return 0;
}

int iio_device_register_inputbridge(struct iio_dev *indio_dev)
{
	int i;
// check if we have any ACCEL channels
// then assign/connect to input device

#if 0
	printk("iio_device_register_inputbridge()\n");
#endif

	if (!indio_dev->channels)
		return 0;

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
	if (channels[0].data)
		input_unregister_device((struct input_dev *) channels[0].data);
}
