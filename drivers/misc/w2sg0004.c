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
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/workqueue.h>

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
	struct		tty_driver *tty_drv;	/* this is the user space tty */
	struct		device *dev;	/* from tty_port_register_device() */
	struct		tty_port port;
	int		open_count;	/* how often we were opened */
	enum		w2sg_state state;
	int		requested;	/* requested state (0/1) */
	int		suspended;
	struct delayed_work work;
	int		discard_count;
};

/* should become w2sg_by_minor[n] if we want to support multiple devices */
static struct w2sg_data *w2sg_by_minor;

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
	} else if (data->open_count > 0) {
		int n;

		pr_debug("w2sg00x4: push %lu chars to tty port\n",
			(unsigned long) count);

		/* pass to user-space */
		n = tty_insert_flip_string(&data->port, rxdata, count);
		if (n != count)
			pr_err("w2sg00x4: did loose %lu characters\n",
				(unsigned long) (count - n));
		tty_flip_buffer_push(&data->port);
		return n;
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
};

/*
 * we are a man-in the middle between the user-space visible tty port
 * and the serdev tty where the chip is connected.
 * This allows us to recognise when the device should be powered on
 * or off and handle the "false" state that data arrives while no
 * users-space tty client exists.
 */

static struct w2sg_data *w2sg_get_by_minor(unsigned int minor)
{
	BUG_ON(minor >= 1);
	return w2sg_by_minor;
}

static int w2sg_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct w2sg_data *data;
	int retval;

	pr_debug("%s() tty = %p\n", __func__, tty);

	data = w2sg_get_by_minor(tty->index);

	if (!data)
		return -ENODEV;

	retval = tty_standard_install(driver, tty);
	if (retval)
		goto error_init_termios;

	tty->driver_data = data;

	return 0;

error_init_termios:
	tty_port_put(&data->port);
	return retval;
}

static int w2sg_tty_open(struct tty_struct *tty, struct file *file)
{
	struct w2sg_data *data = tty->driver_data;

	pr_debug("%s() data = %p open_count = ++%d\n", __func__,
		 data, data->open_count);

	w2sg_set_power(data, ++data->open_count > 0);

	return tty_port_open(&data->port, tty, file);
}

static void w2sg_tty_close(struct tty_struct *tty, struct file *file)
{
	struct w2sg_data *data = tty->driver_data;

	pr_debug("%s()\n", __func__);

	w2sg_set_power(data, --data->open_count > 0);

	tty_port_close(&data->port, tty, file);
}

static int w2sg_tty_write(struct tty_struct *tty,
		const unsigned char *buffer, int count)
{
	struct w2sg_data *data = tty->driver_data;

	/* simply pass down to UART */
	return serdev_device_write_buf(data->uart, buffer, count);
}

static const struct tty_operations w2sg_serial_ops = {
	.install = w2sg_tty_install,
	.open = w2sg_tty_open,
	.close = w2sg_tty_close,
	.write = w2sg_tty_write,
};

static const struct tty_port_operations w2sg_port_ops = {
	/* none defined, but we need the struct */
};

static int w2sg_probe(struct serdev_device *serdev)
{
	struct w2sg_data *data;
	struct rfkill *rf_kill;
	int err;
	int minor;
	enum of_gpio_flags flags;

	pr_debug("%s()\n", __func__);

	/*
	 * For future consideration:
	 * for multiple such GPS receivers in one system
	 * we need a mechanism to define distinct minor values
	 * and search for an unused one.
	 */
	minor = 0;
	if (w2sg_get_by_minor(minor)) {
		pr_err("w2sg minor is already in use!\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&serdev->dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

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

	err = devm_gpio_request(&serdev->dev, data->on_off_gpio,
				"w2sg0004-on-off");
	if (err < 0)
		goto out;

	gpio_direction_output(data->on_off_gpio, false);

	serdev_device_set_client_ops(data->uart, &serdev_ops);
	serdev_device_open(data->uart);

	serdev_device_set_baudrate(data->uart, 9600);
	serdev_device_set_flow_control(data->uart, false);

	rf_kill = rfkill_alloc("GPS", &serdev->dev, RFKILL_TYPE_GPS,
				&w2sg0004_rfkill_ops, data);
	if (rf_kill == NULL) {
		err = -ENOMEM;
		goto err_rfkill;
	}

	err = rfkill_register(rf_kill);
	if (err) {
		dev_err(&serdev->dev, "Cannot register rfkill device\n");
		goto err_rfkill;
	}

	data->rf_kill = rf_kill;

	/* allocate the tty driver */
	data->tty_drv = alloc_tty_driver(1);
	if (!data->tty_drv)
		return -ENOMEM;

	/* initialize the tty driver */
	data->tty_drv->owner = THIS_MODULE;
	data->tty_drv->driver_name = "w2sg0004";
	data->tty_drv->name = "ttyGPS";
	data->tty_drv->major = 0;
	data->tty_drv->minor_start = 0;
	data->tty_drv->type = TTY_DRIVER_TYPE_SERIAL;
	data->tty_drv->subtype = SERIAL_TYPE_NORMAL;
	data->tty_drv->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	data->tty_drv->init_termios = tty_std_termios;
	data->tty_drv->init_termios.c_cflag = B9600 | CS8 | CREAD |
					      HUPCL | CLOCAL;
	/*
	 * optional:
	 * tty_termios_encode_baud_rate(&data->tty_drv->init_termios,
					115200, 115200);
	 * w2sg_tty_termios(&data->tty_drv->init_termios);
	 */
	tty_set_operations(data->tty_drv, &w2sg_serial_ops);

	/* register the tty driver */
	err = tty_register_driver(data->tty_drv);
	if (err) {
		pr_err("%s - tty_register_driver failed(%d)\n",
			__func__, err);
		put_tty_driver(data->tty_drv);
		goto err_rfkill;
	}

	/* minor (0) is now in use */
	w2sg_by_minor = data;

	tty_port_init(&data->port);
	data->port.ops = &w2sg_port_ops;

	data->dev = tty_port_register_device(&data->port,
			data->tty_drv, minor, &serdev->dev);

	/* keep off until user space requests the device */
	w2sg_set_power(data, false);

	return 0;

err_rfkill:
	rfkill_destroy(rf_kill);
	serdev_device_close(data->uart);
out:
	return err;
}

static void w2sg_remove(struct serdev_device *serdev)
{
	struct w2sg_data *data = serdev_device_get_drvdata(serdev);
	int minor;

	cancel_delayed_work_sync(&data->work);

	/* what is the right sequence to avoid problems? */
	serdev_device_close(data->uart);

	/* we should lookup in w2sg_by_minor */
	minor = 0;
	tty_unregister_device(data->tty_drv, minor);

	tty_unregister_driver(data->tty_drv);
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
