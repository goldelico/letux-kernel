/*
 * w2sg0004.c
 * Driver for power controlling the w2sg0004/w2sg0084 GPS receiver.
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

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/w2sg0004.h>
#include <linux/workqueue.h>
#include <linux/rfkill.h>
#include <linux/serial_core.h>

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
	W2SG_NOPULSE	/* desctivate on/off impulse */
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
	struct uart_port	*uart;		/* the drvdata of the uart or tty */
	enum w2sg_state	state;
	int		requested;	/* requested state (0/1) */
	int		suspended;
	int		rx_monitored;	/* if we are registered for rx events */
	spinlock_t	lock;
	struct delayed_work work;
};

static int w2sg_data_set_lna_power(struct w2sg_data *data)
{
	int ret = 0;
	int off = data->suspended || !data->requested || data->lna_blocked;

	pr_debug("w2sg_data_set_lna_power: %s\n", off ? "off" : "on");

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

// call this each time a data block is received by the host (i.e. sent by the w2sg0008)

static void rx_notification(void *pdata)
{
	struct w2sg_data *data = (struct w2sg_data *) pdata;
	unsigned long flags;

	pr_debug("!"); /* we have received a RX signal while GPS should be off */

	if (!data->requested && !data->is_on &&
	    (data->state == W2SG_IDLE) &&
	    time_after(jiffies,
		       data->last_toggle + data->backoff)) {
		/* Should be off by now, time to toggle again */
		data->is_on = 1;
		data->backoff *= 2;
		spin_lock_irqsave(&data->lock, flags);
		if (!data->suspended)
			schedule_delayed_work(&data->work, 0);
		spin_unlock_irqrestore(&data->lock, flags);
	}
}

static void w2sg_data_set_power(void *pdata, int val)
{
	struct w2sg_data *data = (struct w2sg_data *) pdata;
	unsigned long flags;

	pr_debug("GPS SET to %d\n", val);

	spin_lock_irqsave(&data->lock, flags);

	if (val && !data->requested) {
		if (data->rx_monitored) {
			data->rx_monitored = 0;

			if (data->uart) {
				uart_unregister_rx_notifier(data->uart, rx_notification, (void *) data);
			}
		}
		data->requested = 1;
	} else if (!val && data->requested) {
		data->backoff = HZ;
		data->requested = 0;
	} else
		goto unlock;

	if (!data->suspended)
		schedule_delayed_work(&data->work, 0);
unlock:
	spin_unlock_irqrestore(&data->lock, flags);
}

// call this by uart modem control line notifications
static void w2sg_mctrl(void *pdata, int val)
{
	val = (val & TIOCM_DTR) != 0;	/* DTR controls power on/off */
	w2sg_data_set_power(pdata, val);
}

static void toggle_work(struct work_struct *work)
{
	struct w2sg_data *data = container_of(work, struct w2sg_data, work.work);

	w2sg_data_set_lna_power(data);	/* update LNA power state */

	switch (data->state) {
	case W2SG_NOPULSE:
		data->state = W2SG_IDLE;
		pr_debug("GPS idle\n");
		break;

	case W2SG_IDLE:
		spin_lock_irq(&data->lock);
		if (data->requested == data->is_on) {
			if (!data->is_on && !data->rx_monitored) {
				/* not yet redirected in off state */
				data->rx_monitored = 1;

				if (data->uart) {
					uart_register_rx_notifier(data->uart, rx_notification, (void *) data);
				}

			}
			spin_unlock_irq(&data->lock);
			return;
		}
		spin_unlock_irq(&data->lock);
		gpio_set_value_cansleep(data->on_off_gpio, 0);
		data->state = W2SG_PULSE;

		pr_debug("GPS pulse\n");

		schedule_delayed_work(&data->work,
				      msecs_to_jiffies(10));
		break;

	case W2SG_PULSE:
		gpio_set_value_cansleep(data->on_off_gpio, 1);
		data->last_toggle = jiffies;
		data->state = W2SG_NOPULSE;

		pr_debug("GPS nopulse\n");

		data->is_on = !data->is_on;
		schedule_delayed_work(&data->work,
				      msecs_to_jiffies(10));
		break;
	}
}

static int w2sg_data_rfkill_set_block(void *pdata, bool blocked)
{
	struct w2sg_data *data = pdata;

	pr_debug("%s: blocked: %d\n", __func__, blocked);

	data->lna_blocked = blocked;

	return w2sg_data_set_lna_power(data);
}

static struct rfkill_ops w2sg_data0004_rfkill_ops = {
	.set_block = w2sg_data_rfkill_set_block,
};

static int w2sg_data_probe(struct platform_device *pdev)
{
	struct w2sg_pdata *pdata = dev_get_platdata(&pdev->dev);
	struct w2sg_data *data;
	struct rfkill *rf_kill;
	int err;

	pr_debug("w2sg_data_probe()\n");

// FIXME: with uart link, the driver only works with DT - can we remove the CONFIG_OF?

#ifdef CONFIG_OF
	if (pdev->dev.of_node) {
		struct device *dev = &pdev->dev;
		enum of_gpio_flags flags;

		pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		pdata->on_off_gpio = of_get_named_gpio_flags(dev->of_node, "on-off-gpio", 0, &flags);

		if (pdata->on_off_gpio == -EPROBE_DEFER)
			return -EPROBE_DEFER;	/* defer until we have all gpios */

		pdata->lna_regulator = devm_regulator_get_optional(dev, "lna");

		pr_debug("w2sg_data_probe() lna_regulator = %p\n", pdata->lna_regulator);

		// CHECKME: do we have to require the lna_regulator?
		if (IS_ERR(pdata->lna_regulator))
			return PTR_ERR(pdata->lna_regulator);

		pdata->gpio_base = -1;
		pdev->dev.platform_data = pdata;
	}
#endif

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	data->lna_regulator = pdata->lna_regulator;
	data->lna_blocked = true;
	data->lna_is_off = true;

	data->on_off_gpio = pdata->on_off_gpio;

	data->is_on = false;
	data->requested = true;
	data->state = W2SG_IDLE;
	data->last_toggle = jiffies;
	data->backoff = HZ;

#ifdef CONFIG_OF
	data->uart = devm_serial_get_uart_by_phandle(&pdev->dev, "uart", 0);
	if (IS_ERR(data->uart)) {
		if (PTR_ERR(data->uart) == -EPROBE_DEFER)
			return -EPROBE_DEFER;	/* we can't probe yet */
		else
			data->uart = NULL;	/* no UART */
		}
#endif

	INIT_DELAYED_WORK(&data->work, toggle_work);
	spin_lock_init(&data->lock);

	err = devm_gpio_request(&pdev->dev, data->on_off_gpio, "w2sg0004-on-off");
	if (err < 0)
		goto out;

	gpio_direction_output(data->on_off_gpio, false);

	if (data->uart) {
		uart_register_power_notifier(data->uart, w2sg_mctrl, (void *) data);
	}

	rf_kill = rfkill_alloc("GPS", &pdev->dev, RFKILL_TYPE_GPS,
				&w2sg_data0004_rfkill_ops, data);
	if (rf_kill == NULL) {
		err = -ENOMEM;
		goto err_rfkill;
	}

	err = rfkill_register(rf_kill);
	if (err) {
		dev_err(&pdev->dev, "Cannot register rfkill device\n");
		goto err_rfkill;
	}

	data->rf_kill = rf_kill;

	platform_set_drvdata(pdev, data);

	pr_debug("w2sg0004 probed\n");

	return 0;

err_rfkill:
	rfkill_destroy(rf_kill);
out:
	return err;
}

static int w2sg_data_remove(struct platform_device *pdev)
{
	struct w2sg_data *data = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&data->work);

	if (data->uart) {
		uart_unregister_power_notifier(data->uart, w2sg_mctrl, (void *) data);
		uart_unregister_rx_notifier(data->uart, rx_notification, (void *) data);
	}

	return 0;
}

static int w2sg_data_suspend(struct device *dev)
{
	struct w2sg_data *data = dev_get_drvdata(dev);

	spin_lock_irq(&data->lock);
	data->suspended = 1;
	spin_unlock_irq(&data->lock);

	cancel_delayed_work_sync(&data->work);

	w2sg_data_set_lna_power(data);	/* shuts down if needed */

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
		pr_info("GPS off for suspend %d %d %d\n", data->requested, data->is_on, data->lna_is_off);

		gpio_set_value_cansleep(data->on_off_gpio, 0);
		msleep(10);
		gpio_set_value_cansleep(data->on_off_gpio, 1);
		data->is_on = 0;
	}

	return 0;
}

static int w2sg_data_resume(struct device *dev)
{
	struct w2sg_data *data = dev_get_drvdata(dev);

	spin_lock_irq(&data->lock);
	data->suspended = 0;
	spin_unlock_irq(&data->lock);

	pr_info("GPS resuming %d %d %d\n", data->requested, data->is_on, data->lna_is_off);

	schedule_delayed_work(&data->work, 0);	/* enables LNA if needed */

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id w2sg0004_of_match[] = {
	{ .compatible = "wi2wi,w2sg0004" },
	{ .compatible = "wi2wi,w2sg0084" },
	{},
};
MODULE_DEVICE_TABLE(of, w2sg0004_of_match);
#endif

SIMPLE_DEV_PM_OPS(w2sg_pm_ops, w2sg_data_suspend, w2sg_data_resume);

static struct platform_driver w2sg_data_driver = {
	.probe		= w2sg_data_probe,
	.remove		= w2sg_data_remove,
	.driver = {
		.name	= "w2sg0004",
		.owner	= THIS_MODULE,
		.pm	= &w2sg_pm_ops,
		.of_match_table = of_match_ptr(w2sg0004_of_match)
	},
};

module_platform_driver(w2sg_data_driver);

MODULE_ALIAS("w2sg0004");

MODULE_AUTHOR("NeilBrown <neilb@suse.de>");
MODULE_DESCRIPTION("w2sg0004 GPS power management driver");
MODULE_LICENSE("GPL v2");
