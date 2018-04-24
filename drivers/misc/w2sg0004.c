// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for power controlling the w2sg0004/w2sg0084 GPS receiver.
 *
 * Copyright (C) 2013 Neil Brown <neilb@suse.de>
 * Copyright (C) 2015-2017 H. Nikolaus Schaller <hns@goldelico.com>,
 *						Golden Delicious Computers
 *
 * This receiver has an ON/OFF pin which must be toggled to
 * turn the device 'on' of 'off'.  A high->low->high toggle
 * will switch the device on if it is off, and off if it is on.
 *
 * To enable receiving on/off requests we register with the
 * UART power management notifications.
 *
 * It is not possible to directly detect the state of the device.
 * However when it is on it will send characters on a UART line
 * regularly.
 *
 * To detect that the power state is out of sync (e.g. if GPS
 * was enabled before a reboot), we register for UART data received
 * notifications.
 *
 * In addition we register as a rfkill client so that we can
 * control the LNA power.
 *
 */

#ifdef CONFIG_W2SG0004_DEBUG
#define DEBUG 1
#endif

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/rfkill.h>
#include <linux/serdev.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "gps-core.h"


/*
 * There seems to be restrictions on how quickly we can toggle the
 * on/off line.  data sheets says "two rtc ticks", whatever that means.
 * If we do it too soon it doesn't work.
 * So we have a state machine which uses the common work queue to ensure
 * clean transitions.
 * When a change is requested we record that request and only act on it
 * once the previous change has completed.
 * A change involves a 10ms low pulse, and a 990ms raised level, so only
 * one change per second.
 */

enum w2sg_state {
	W2SG_IDLE,	/* is not changing state */
	W2SG_PULSE,	/* activate on/off impulse */
	W2SG_NOPULSE	/* deactivate on/off impulse */
};

struct w2sg_data {
	struct		rfkill *rf_kill;
	struct		regulator *lna_regulator;
	int		lna_blocked;	/* rfkill block gps active */
	int		lna_is_off;	/* LNA is currently off */
	int		is_on;		/* current state (0/1) */
	unsigned long	last_toggle;
	unsigned long	backoff;	/* time to wait since last_toggle */
	int		on_off_gpio;	/* the on-off gpio number */
	struct		serdev_device *uart;	/* uart connected to the chip */
	enum		w2sg_state state;
	int		requested;	/* requested state (0/1) */
	int		suspended;
	struct delayed_work work;
	int		discard_count;
	struct gps_dev	gpsdev;
};

static int w2sg_set_lna_power(struct w2sg_data *data)
{
	int ret = 0;
	int off = data->suspended || !data->requested || data->lna_blocked;

	pr_debug("%s: %s\n", __func__, off ? "off" : "on");

	if (off != data->lna_is_off) {
		data->lna_is_off = off;
		if (!IS_ERR_OR_NULL(data->lna_regulator)) {
			if (off)
				regulator_disable(data->lna_regulator);
			else
				ret = regulator_enable(data->lna_regulator);
		}
	}

	return ret;
}

static void w2sg_set_power(void *pdata, int val)
{
	struct w2sg_data *data = (struct w2sg_data *) pdata;

	pr_debug("%s to state=%d (requested=%d)\n", __func__, val,
		 data->requested);

	if (val && !data->requested) {
		data->requested = true;
	} else if (!val && data->requested) {
		data->backoff = HZ;
		data->requested = false;
	} else
		return;

	if (!data->suspended)
		schedule_delayed_work(&data->work, 0);
}

/* called each time data is received by the UART (i.e. sent by the w2sg0004) */
static int w2sg_uart_receive_buf(struct serdev_device *serdev,
				const unsigned char *rxdata,
				size_t count)
{
	struct w2sg_data *data =
		(struct w2sg_data *) serdev_device_get_drvdata(serdev);

	if (!data->requested && !data->is_on) {
		/*
		 * we have received characters while the w2sg
		 * should have been be turned off
		 */
		data->discard_count += count;
		if ((data->state == W2SG_IDLE) &&
		    time_after(jiffies,
		    data->last_toggle + data->backoff)) {
			/* Should be off by now, time to toggle again */
			pr_debug("w2sg00x4 has sent %d characters data although it should be off!\n",
				data->discard_count);

			data->discard_count = 0;

			data->is_on = true;
			data->backoff *= 2;
			if (!data->suspended)
				schedule_delayed_work(&data->work, 0);
		}
	} else if (data->gpsdev.open_count > 0) {
		/*
		 * pass to user-space
		 */
		pr_debug("w2sg00x4: push %lu chars to user space\n",
			(unsigned long) count);

		return gps_recv_nmea_chars(&data->gpsdev, rxdata, count);
	}

	/* assume we have processed everything */
	return count;
}

/* try to toggle the power state by sending a pulse to the on-off GPIO */
static void toggle_work(struct work_struct *work)
{
	struct w2sg_data *data = container_of(work, struct w2sg_data,
					      work.work);

	switch (data->state) {
	case W2SG_IDLE:
		if (data->requested == data->is_on)
			return;

		w2sg_set_lna_power(data);	/* update LNA power state */
		gpio_set_value_cansleep(data->on_off_gpio, 0);
		data->state = W2SG_PULSE;

		pr_debug("w2sg: power gpio ON\n");

		schedule_delayed_work(&data->work,
				      msecs_to_jiffies(10));
		break;

	case W2SG_PULSE:
		gpio_set_value_cansleep(data->on_off_gpio, 1);
		data->last_toggle = jiffies;
		data->state = W2SG_NOPULSE;
		data->is_on = !data->is_on;

		pr_debug("w2sg: power gpio OFF\n");

		schedule_delayed_work(&data->work,
				      msecs_to_jiffies(10));
		break;

	case W2SG_NOPULSE:
		data->state = W2SG_IDLE;

		pr_debug("w2sg: idle\n");

		break;

	}
}

static int w2sg_rfkill_set_block(void *pdata, bool blocked)
{
	struct w2sg_data *data = pdata;

	pr_debug("%s: blocked: %d\n", __func__, blocked);

	data->lna_blocked = blocked;

	return w2sg_set_lna_power(data);
}

static struct rfkill_ops w2sg0004_rfkill_ops = {
	.set_block = w2sg_rfkill_set_block,
};

static struct serdev_device_ops serdev_ops = {
	.receive_buf = w2sg_uart_receive_buf,
#if 0
	.write_wakeup = w2sg_uart_wakeup,
#endif
};

static int w2sg_gps_open(struct gps_dev *gdev)
{ /* user-space has opened our interface */
	struct w2sg_data *data = container_of(gdev, struct w2sg_data, gpsdev);

	pr_debug("%s()\n", __func__);

	w2sg_set_power(data, true);

	return 0;
}

static int w2sg_gps_close(struct gps_dev *gdev)
{ /* user-space has finally closed our interface */
	struct w2sg_data *data = container_of(gdev, struct w2sg_data, gpsdev);

	pr_debug("%s()\n", __func__);

	w2sg_set_power(data, false);

	return 0;
}

static int w2sg_gps_send(struct gps_dev *gdev,
		const char *buffer, int count)
{ /* raw data coming from user space */
	struct w2sg_data *data = container_of(gdev, struct w2sg_data, gpsdev);

	/* simply pass down to UART */
	return serdev_device_write_buf(data->uart, buffer, count);
}

static int w2sg_probe(struct serdev_device *serdev)
{
	struct w2sg_data *data;
	struct rfkill *rf_kill;
	int err;
	enum of_gpio_flags flags;

	pr_debug("%s()\n", __func__);

	data = devm_kzalloc(&serdev->dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	pr_debug("w2sg serdev_device_set_drvdata\n");

	serdev_device_set_drvdata(serdev, data);

	data->on_off_gpio = of_get_named_gpio_flags(serdev->dev.of_node,
						     "enable-gpios", 0,
						     &flags);
	/* defer until we have all gpios */
	if (data->on_off_gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	data->lna_regulator = devm_regulator_get_optional(&serdev->dev,
							"lna");
	if (IS_ERR(data->lna_regulator)) {
		/* defer until we can get the regulator */
		if (PTR_ERR(data->lna_regulator) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		data->lna_regulator = NULL;
	}
	pr_debug("%s() lna_regulator = %p\n", __func__, data->lna_regulator);

	data->lna_blocked = true;
	data->lna_is_off = true;

	data->is_on = false;
	data->requested = false;
	data->state = W2SG_IDLE;
	data->last_toggle = jiffies;
	data->backoff = HZ;

	data->uart = serdev;

	INIT_DELAYED_WORK(&data->work, toggle_work);

	pr_debug("w2sg devm_gpio_request\n");

	err = devm_gpio_request(&serdev->dev, data->on_off_gpio,
				"w2sg0004-on-off");
	if (err < 0)
		goto out;

	gpio_direction_output(data->on_off_gpio, false);

	serdev_device_set_client_ops(data->uart, &serdev_ops);
	serdev_device_open(data->uart);

	serdev_device_set_baudrate(data->uart, 9600);
	serdev_device_set_flow_control(data->uart, false);

	pr_debug("w2sg rfkill_alloc\n");

	rf_kill = rfkill_alloc("GPS", &serdev->dev, RFKILL_TYPE_GPS,
				&w2sg0004_rfkill_ops, data);
	if (rf_kill == NULL) {
		err = -ENOMEM;
		goto err_rfkill;
	}

	pr_debug("w2sg register rfkill\n");

	err = rfkill_register(rf_kill);
	if (err) {
		dev_err(&serdev->dev, "Cannot register rfkill device\n");
		goto err_rfkill;
	}

	data->rf_kill = rf_kill;

	pr_debug("w2sg prepare for gps_register_dev\n");

	data->gpsdev.open = w2sg_gps_open;
	data->gpsdev.close = w2sg_gps_close;
	data->gpsdev.send = w2sg_gps_send;

	err = gps_register_dev(&data->gpsdev, "w2sg0004");

	if (err) {
		pr_err("%s - gps_register_dev failed(%d)\n",
			__func__, err);
		goto err_rfkill;
	}

	pr_debug("w2sg probed\n");

#ifdef CONFIG_W2SG0004_DEBUG
	pr_debug("w2sg DEBUGGING MODE enabled\n");
	/* turn on for debugging rx notifications */
	pr_debug("w2sg power gpio ON\n");
	gpio_set_value_cansleep(data->on_off_gpio, 1);
	mdelay(100);
	pr_debug("w2sg power gpio OFF\n");
	gpio_set_value_cansleep(data->on_off_gpio, 0);
	mdelay(300);
#endif

	/* keep off until user space requests the device */
	w2sg_set_power(data, false);

	return 0;

err_rfkill:
	rfkill_destroy(rf_kill);
	serdev_device_close(data->uart);
out:
	pr_debug("w2sg error %d\n", err);

	return err;
}

static void w2sg_remove(struct serdev_device *serdev)
{
	struct w2sg_data *data = serdev_device_get_drvdata(serdev);

	cancel_delayed_work_sync(&data->work);

	/* what is the right sequence to avoid problems? */
	serdev_device_close(data->uart);

	gps_unregister_dev(&data->gpsdev);
}

static int __maybe_unused w2sg_suspend(struct device *dev)
{
	struct w2sg_data *data = dev_get_drvdata(dev);

	data->suspended = true;

	cancel_delayed_work_sync(&data->work);

	w2sg_set_lna_power(data);	/* shuts down if needed */

	if (data->state == W2SG_PULSE) {
		msleep(10);
		gpio_set_value_cansleep(data->on_off_gpio, 1);
		data->last_toggle = jiffies;
		data->is_on = !data->is_on;
		data->state = W2SG_NOPULSE;
	}

	if (data->state == W2SG_NOPULSE) {
		msleep(10);
		data->state = W2SG_IDLE;
	}

	if (data->is_on) {
		pr_info("GPS off for suspend %d %d %d\n", data->requested,
			data->is_on, data->lna_is_off);

		gpio_set_value_cansleep(data->on_off_gpio, 0);
		msleep(10);
		gpio_set_value_cansleep(data->on_off_gpio, 1);
		data->is_on = 0;
	}

	return 0;
}

static int __maybe_unused w2sg_resume(struct device *dev)
{
	struct w2sg_data *data = dev_get_drvdata(dev);

	data->suspended = false;

	pr_info("GPS resuming %d %d %d\n", data->requested,
		data->is_on, data->lna_is_off);

	schedule_delayed_work(&data->work, 0);	/* enables LNA if needed */

	return 0;
}

static const struct of_device_id w2sg0004_of_match[] = {
	{ .compatible = "wi2wi,w2sg0004" },
	{ .compatible = "wi2wi,w2sg0084" },
	{},
};
MODULE_DEVICE_TABLE(of, w2sg0004_of_match);

SIMPLE_DEV_PM_OPS(w2sg_pm_ops, w2sg_suspend, w2sg_resume);

static struct serdev_device_driver w2sg_driver = {
	.probe		= w2sg_probe,
	.remove		= w2sg_remove,
	.driver = {
		.name	= "w2sg0004",
		.owner	= THIS_MODULE,
		.pm	= &w2sg_pm_ops,
		.of_match_table = of_match_ptr(w2sg0004_of_match)
	},
};

module_serdev_device_driver(w2sg_driver);

MODULE_AUTHOR("NeilBrown <neilb@suse.de>");
MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("w2sg0004 GPS power management driver");
MODULE_LICENSE("GPL v2");
