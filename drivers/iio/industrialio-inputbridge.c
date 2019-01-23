#include <linux/iio/iio.h>
#include <linux/input.h>
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>

#include "industrialio-inputbridge.h"

/* only polling is implemented*/
#define POLLING 1

// we need a better mapping for more than 3 channels
// in fact we must group the channels by sharing the same iio device and input device...
// which means that we could simply create a list of all channels
// and report them all
// a simple solution would be to reserve 3*n slots and process all those where an indio_dev is != NULL
// and synchronize after processing index == 2

#define DEVICES		5	/* handle up to this number of input devices */
static struct iio_channel channels[DEVICES][3];	// 3 channels per device
static DEFINE_MUTEX(inputbridge_channel_mutex);	// we must protect the channel allocation

#if POLLING
static struct delayed_work input_work;
static int open_count = 0;
static DEFINE_MUTEX(inputbridge_open_mutex);	// we must protect the open counter
#endif

#define ABSMAX_ACC_VAL		((1<<9)-1) /* 10 bit */
#define ABSMIN_ACC_VAL		-(ABSMAX_ACC_VAL)

/* scale processed values so that 1g maps to ABSMAX_ACC_VAL / 2 */
#define SCALE			(100 * ABSMAX_ACC_VAL) / (2 * 981)

static void accel_report_channels(void)
{
	int val;
	int ret;

	int dindex, cindex;

	for (dindex = 0; dindex < ARRAY_SIZE(channels); dindex++) {
		struct input_dev *input = NULL;

	for (cindex = 0; cindex < ARRAY_SIZE(channels[0]); cindex++) {
		struct iio_channel *channel = &channels[cindex][dindex];

		mutex_lock(&inputbridge_channel_mutex);

		if (channel->indio_dev) {
			ret = iio_read_channel_raw(channel, &val);

#if 0
printk("accel_report_channel %d -> %d ret=%d\n", cindex, val, ret);
#endif

			if (ret < 0) {
				pr_err("accel channel read error %d\n", cindex);
				return;
			}

			/*
			 * NOTE: SCALE is ignored by iio_convert_raw_to_processed()
			 * for IIO_VAL_INT so we get wrong values.
			 */

			ret = iio_convert_raw_to_processed(channel, val, &val, SCALE);

			if (ret < 0) {
				pr_err("accel channel processing error\n");
				return;
			}

			input = channel->data;
			switch (dindex) {
				case 0:
					input_report_abs(input, ABS_X, val);
					break;
				case 1:
					input_report_abs(input, ABS_Y, val);
					break;
				case 2:
					input_report_abs(input, ABS_Z, val);
					break;
			}
		}
		mutex_unlock(&inputbridge_channel_mutex);
	}

		if (input)
			input_sync(input);

	}
}

#if POLLING
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

#if POLLING
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

#if POLLING
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
	struct input_dev *idev = NULL;

	int dindex, cindex;

		struct iio_channel *channel = &channels[cindex][dindex];
#if 0
	printk("iio_device_register_inputbridge(): found an accelerometer\n");
#endif

	mutex_lock(&inputbridge_channel_mutex);

	/* look for existing input device */

	for (dindex = 0; dindex < ARRAY_SIZE(channels); dindex++) {
		if (channels[dindex][0].indio_dev == indio_dev) {
			idev = channels[dindex][0].data;
			break;
		}
	}

	if (!idev) {
		/* look for a free slot for a new input device */

		for (dindex = 0; dindex < ARRAY_SIZE(channels); dindex ++) {
			if (channels[dindex][0].indio_dev == NULL)
				break;
		}

		if (dindex == ARRAY_SIZE(channels)) {
			mutex_unlock(&inputbridge_channel_mutex);
			return -ENOMEM;
		}

#if 0
	printk("iio_device_register_inputbridge(): allocate the input dev\n");
#endif

		idev = input_allocate_device();

#if 0
	printk("iio_device_register_inputbridge(): => %p\n", idev);
#endif

		if (!idev) {
			mutex_unlock(&inputbridge_channel_mutex);
			return -ENOMEM;
		}

		idev->name = "accelerometer-iio-input-bridge";
		// FIXME: when does this need kfree?
		idev->phys = kasprintf(GFP_KERNEL, "accel/input%d", dindex);
//		idev->id.bustype = BUS_I2C;

//		idev->dev.parent = &indio_dev->client->dev;

		idev->evbit[0] = BIT_MASK(EV_ABS);
		idev->open = accel_open;
		idev->close = accel_close;

		// FIXME: what happens if we unregister the first device?
		if (dindex == 0 ) { // first input
#if POLLING
			INIT_DELAYED_WORK(&input_work, inputbridge_work);
#else
			struct iio_cb_buffer *iio_channel_get_all_cb(struct device *dev,
					int (*cb)(const void *data,
						void *private),
						void *private);
#endif
		}

//		indio_dev->input = idev;

		input_set_drvdata(idev, indio_dev);

		input_alloc_absinfo(idev);
		error = input_register_device(idev);

#if 0
	printk("iio_device_register_inputbridge(): input_register_device => %d\n", error);
#endif

		if (error < 0) {
			input_free_device(idev);
			mutex_unlock(&inputbridge_channel_mutex);
			return error;
		}

	}

	/* find free channel within this device block */
	for (cindex = 0;  cindex < ARRAY_SIZE(channels[0]); cindex++) {
		if (!channels[dindex][cindex].indio_dev) {
			break;
		}
	}

	if (cindex == ARRAY_SIZE(channels[0])) { /* we already have collected 3 channels */
		mutex_unlock(&inputbridge_channel_mutex);
		return 0;	/* silently ignore */
	}

#if 0
	printk("iio_device_register_inputbridge(): process channel %d of device %d\n", cindex, dindex);
#endif

	channels[dindex][cindex].indio_dev = indio_dev;
	channels[dindex][cindex].channel = chan;
	channels[dindex][cindex].data = (void *) idev;

	switch (cindex) {
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

	for (dindex = 0; dindex < ARRAY_SIZE(channels); dindex++) {
		for (cindex = 0; cindex < ARRAY_SIZE(channels[0]); cindex++) {
			struct iio_channel *channel = &channels[cindex][dindex];

			if (channel->indio_dev == indio_dev) {
				channel->indio_dev = NULL;	/* mark as empty */
				input = channel->data;
			}
		}
	}

	if (input)
		input_unregister_device(input);
}
