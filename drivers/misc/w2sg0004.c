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

// #include something to get this declaration

struct uart {
	struct device           *dev;
	struct list_head        head;
};

extern struct uart *devm_tty_get_uart_by_phandle(struct device *dev,
		const char *phandle, u8 index);

#if 1	// move this fragment to serial driver code

static LIST_HEAD(uart_list);
static DEFINE_SPINLOCK(uart_lock);

/* same concept as __of_usb_find_phy */
static struct uart *__of_tty_find_uart(struct device_node *node)
{
        struct uart  *uart;

        if (!of_device_is_available(node))
                return ERR_PTR(-ENODEV);

        list_for_each_entry(uart, &uart_list, head) {
                if (node != uart->dev->of_node)
                        continue;

                return uart;
        }

        return ERR_PTR(-EPROBE_DEFER);
}

static void devm_tty_uart_release(struct device *dev, void *res)
{
        struct uart *uart = *(struct uart **)res;

  // FIXME:      tty_put_uart(uart);
}


/**
 * devm_tty_get_uart_by_phandle - find the uart by phandle
 * @dev - device that requests this uart
 * @phandle - name of the property holding the uart phandle value
 * @index - the index of the uart
 *
 * Returns the uart driver associated with the given phandle value,
 * after getting a refcount to it, -ENODEV if there is no such uart or
 * -EPROBE_DEFER if there is a phandle to the uart, but the device is
 * not yet loaded. While at that, it also associates the device with
 * the uart using devres. On driver detach, release function is invoked
 * on the devres data, then, devres data is freed.
 *
 * For use by tty host and peripheral drivers.
 */

/* same concept as devm_usb_get_phy_by_phandle */

struct uart *devm_tty_get_uart_by_phandle(struct device *dev,
		const char *phandle, u8 index)
{
        struct uart  *uart = ERR_PTR(-ENOMEM), **ptr;
        unsigned long   flags;
        struct device_node *node;

        if (!dev->of_node) {
                dev_dbg(dev, "device does not have a device node entry\n");
                return ERR_PTR(-EINVAL);
        }

        node = of_parse_phandle(dev->of_node, phandle, index);
        if (!node) {
                dev_dbg(dev, "failed to get %s phandle in %s node\n", phandle,
                        dev->of_node->full_name);
                return ERR_PTR(-ENODEV);
        }

        ptr = devres_alloc(devm_tty_uart_release, sizeof(*ptr), GFP_KERNEL);
        if (!ptr) {
                dev_dbg(dev, "failed to allocate memory for devres\n");
                goto err0;
        }

        spin_lock_irqsave(&uart_lock, flags);

        uart = __of_tty_find_uart(node);
        if (IS_ERR(uart)) {
                devres_free(ptr);
                goto err1;
        }

        if (!try_module_get(uart->dev->driver->owner)) {
                uart = ERR_PTR(-ENODEV);
                devres_free(ptr);
                goto err1;
        }

        *ptr = uart;
        devres_add(dev, ptr);

        get_device(uart->dev);

err1:
        spin_unlock_irqrestore(&uart_lock, flags);

err0:
        of_node_put(node);

        return uart;
}
EXPORT_SYMBOL_GPL(devm_tty_get_uart_by_phandle);

void tty_register_power_notifier(void (*function)(void *d, int state), void *data)
{
	// call function with parameter data on power state changes
}

void tty_unregister_power_notifier(void (*function)(void *d, int state), void *data)
{
	// no longer call function with parameter data on power state changes
}

void tty_register_rx_notifier(void (*function)(void *d), void *data)
{
	// call function with parameter data each time some data (block) was received
}

void tty_unregister_rx_notifier(void (*function)(void *d), void *data)
{
	// no longer call function with parameter data each time some data (block) was received
}

EXPORT_SYMBOL_GPL(tty_register_power_notifier);
EXPORT_SYMBOL_GPL(tty_unregister_power_notifier);
EXPORT_SYMBOL_GPL(tty_register_rx_notifier);
EXPORT_SYMBOL_GPL(tty_unregister_rx_notifier);

#endif

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

struct gpio_w2sg {
	struct		rfkill *rf_kill;
	struct		regulator *lna_regulator;
	int		lna_blocked;	/* rfkill block gps active */
	int		lna_is_off;	/* LNA is currently off */
	int		is_on;		/* current state (0/1) */
	unsigned long	last_toggle;
	unsigned long	backoff;	/* time to wait since last_toggle */
	int		on_off_gpio;	/* the on-off gpio number */
	struct uart	*uart;		/* the drvdata of the uart or tty */
	enum w2sg_state	state;
	int		requested;	/* requested state (0/1) */
	int		suspended;
	int		rx_monitored;	/* if we are registered for rx events */
	spinlock_t	lock;
	struct delayed_work work;
};

static int gpio_w2sg_set_lna_power(struct gpio_w2sg *gw2sg)
{
	int ret = 0;
	int off = gw2sg->suspended || !gw2sg->requested || gw2sg->lna_blocked;

	pr_debug("gpio_w2sg_set_lna_power: %s\n", off ? "off" : "on");

	if (off != gw2sg->lna_is_off) {
		gw2sg->lna_is_off = off;
		if (!IS_ERR_OR_NULL(gw2sg->lna_regulator)) {
			if (off)
				regulator_disable(gw2sg->lna_regulator);
			else
				ret = regulator_enable(gw2sg->lna_regulator);
		}
	}

	return ret;
}

// call this each time a data block is received by the host (i.e. sent by the w2sg0008)

static void rx_notification(void *data)
{
	struct gpio_w2sg *gw2sg = (struct gpio_w2sg *) data;
	unsigned long flags;

	pr_debug("!"); /* we have received a RX signal while GPS should be off */

	if (!gw2sg->requested && !gw2sg->is_on &&
	    (gw2sg->state == W2SG_IDLE) &&
	    time_after(jiffies,
		       gw2sg->last_toggle + gw2sg->backoff)) {
		/* Should be off by now, time to toggle again */
		gw2sg->is_on = 1;
		gw2sg->backoff *= 2;
		spin_lock_irqsave(&gw2sg->lock, flags);
		if (!gw2sg->suspended)
			schedule_delayed_work(&gw2sg->work, 0);
		spin_unlock_irqrestore(&gw2sg->lock, flags);
	}
}

// call this by uart power notifications

static void gpio_w2sg_set_power(void *data, int val)
{
	struct gpio_w2sg *gw2sg = (struct gpio_w2sg *) data;
	unsigned long flags;

	pr_debug("GPS SET to %d\n", val);

	spin_lock_irqsave(&gw2sg->lock, flags);

	if (val && !gw2sg->requested) {
		if (gw2sg->rx_monitored) {
			gw2sg->rx_monitored = 0;

			if (gw2sg->uart) {
				tty_unregister_rx_notifier(rx_notification, (void *) gw2sg);
			}
		}
		gw2sg->requested = 1;
	} else if (!val && gw2sg->requested) {
		gw2sg->backoff = HZ;
		gw2sg->requested = 0;
	} else
		goto unlock;

	if (!gw2sg->suspended)
		schedule_delayed_work(&gw2sg->work, 0);
unlock:
	spin_unlock_irqrestore(&gw2sg->lock, flags);
}

static void toggle_work(struct work_struct *work)
{
	struct gpio_w2sg *gw2sg = container_of(work, struct gpio_w2sg, work.work);

	gpio_w2sg_set_lna_power(gw2sg);	/* update LNA power state */

	switch (gw2sg->state) {
	case W2SG_NOPULSE:
		gw2sg->state = W2SG_IDLE;
		pr_debug("GPS idle\n");
		break;

	case W2SG_IDLE:
		spin_lock_irq(&gw2sg->lock);
		if (gw2sg->requested == gw2sg->is_on) {
			if (!gw2sg->is_on && !gw2sg->rx_monitored) {
				/* not yet redirected in off state */
				gw2sg->rx_monitored = 1;

				if (gw2sg->uart) {
					tty_register_rx_notifier(rx_notification, (void *) gw2sg);
				}

			}
			spin_unlock_irq(&gw2sg->lock);
			return;
		}
		spin_unlock_irq(&gw2sg->lock);
		gpio_set_value_cansleep(gw2sg->on_off_gpio, 0);
		gw2sg->state = W2SG_PULSE;

		pr_debug("GPS pulse\n");

		schedule_delayed_work(&gw2sg->work,
				      msecs_to_jiffies(10));
		break;

	case W2SG_PULSE:
		gpio_set_value_cansleep(gw2sg->on_off_gpio, 1);
		gw2sg->last_toggle = jiffies;
		gw2sg->state = W2SG_NOPULSE;

		pr_debug("GPS nopulse\n");

		gw2sg->is_on = !gw2sg->is_on;
		schedule_delayed_work(&gw2sg->work,
				      msecs_to_jiffies(10));
		break;
	}
}

static int gpio_w2sg_rfkill_set_block(void *data, bool blocked)
{
	struct gpio_w2sg *gw2sg = data;

	pr_debug("%s: blocked: %d\n", __func__, blocked);

	gw2sg->lna_blocked = blocked;

	return gpio_w2sg_set_lna_power(gw2sg);
}

static struct rfkill_ops gpio_w2sg0004_rfkill_ops = {
	.set_block = gpio_w2sg_rfkill_set_block,
};

static int gpio_w2sg_probe(struct platform_device *pdev)
{
	struct gpio_w2sg_data *pdata = dev_get_platdata(&pdev->dev);
	struct gpio_w2sg *gw2sg;
	struct rfkill *rf_kill;
	int err;

	pr_debug("gpio_w2sg_probe()\n");

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

		pr_debug("gpio_w2sg_probe() lna_regulator = %p\n", pdata->lna_regulator);

		// CHECKME: do we have to require the lna_regulator?
		if (IS_ERR(pdata->lna_regulator))
			return PTR_ERR(pdata->lna_regulator);

		pdata->gpio_base = -1;
		pdev->dev.platform_data = pdata;
	}
#endif

	gw2sg = devm_kzalloc(&pdev->dev, sizeof(*gw2sg), GFP_KERNEL);
	if (gw2sg == NULL)
		return -ENOMEM;

	gw2sg->lna_regulator = pdata->lna_regulator;
	gw2sg->lna_blocked = true;
	gw2sg->lna_is_off = true;

	gw2sg->on_off_gpio = pdata->on_off_gpio;

	gw2sg->is_on = false;
	gw2sg->requested = true;
	gw2sg->state = W2SG_IDLE;
	gw2sg->last_toggle = jiffies;
	gw2sg->backoff = HZ;

#ifdef CONFIG_OF
	gw2sg->uart = devm_tty_get_uart_by_phandle(&pdev->dev, "uart", 0);
	if (IS_ERR(gw2sg->uart)) {
		if (PTR_ERR(gw2sg->uart) == -EPROBE_DEFER)
			return -EPROBE_DEFER;	/* we can't probe yet */
		else
			gw2sg->uart = NULL;	/* no UART */
		}
#endif

	INIT_DELAYED_WORK(&gw2sg->work, toggle_work);
	spin_lock_init(&gw2sg->lock);

	err = devm_gpio_request(&pdev->dev, gw2sg->on_off_gpio, "w2sg0004-on-off");
	if (err < 0)
		goto out;

	gpio_direction_output(gw2sg->on_off_gpio, false);

	if (gw2sg->uart) {
		tty_register_power_notifier(gpio_w2sg_set_power, (void *) gw2sg);
	}

	rf_kill = rfkill_alloc("GPS", &pdev->dev, RFKILL_TYPE_GPS,
				&gpio_w2sg0004_rfkill_ops, gw2sg);
	if (rf_kill == NULL) {
		err = -ENOMEM;
		goto err_rfkill;
	}

	err = rfkill_register(rf_kill);
	if (err) {
		dev_err(&pdev->dev, "Cannot register rfkill device\n");
		goto err_rfkill;
	}

	gw2sg->rf_kill = rf_kill;

	platform_set_drvdata(pdev, gw2sg);

	pr_debug("w2sg0004 probed\n");

	return 0;

err_rfkill:
	rfkill_destroy(rf_kill);
out:
	return err;
}

static int gpio_w2sg_remove(struct platform_device *pdev)
{
	struct gpio_w2sg *gw2sg = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&gw2sg->work);

	if (gw2sg->uart) {
		tty_unregister_power_notifier(gpio_w2sg_set_power, (void *) gw2sg);
		tty_unregister_rx_notifier(rx_notification, (void *) gw2sg);
	}

	return 0;
}

static int gpio_w2sg_suspend(struct device *dev)
{
	struct gpio_w2sg *gw2sg = dev_get_drvdata(dev);

	spin_lock_irq(&gw2sg->lock);
	gw2sg->suspended = 1;
	spin_unlock_irq(&gw2sg->lock);

	cancel_delayed_work_sync(&gw2sg->work);

	gpio_w2sg_set_lna_power(gw2sg);	/* shuts down if needed */

	if (gw2sg->state == W2SG_PULSE) {
		msleep(10);
		gpio_set_value_cansleep(gw2sg->on_off_gpio, 1);
		gw2sg->last_toggle = jiffies;
		gw2sg->is_on = !gw2sg->is_on;
		gw2sg->state = W2SG_NOPULSE;
	}

	if (gw2sg->state == W2SG_NOPULSE) {
		msleep(10);
		gw2sg->state = W2SG_IDLE;
	}

	if (gw2sg->is_on) {
		pr_info("GPS off for suspend %d %d %d\n", gw2sg->requested, gw2sg->is_on, gw2sg->lna_is_off);

		gpio_set_value_cansleep(gw2sg->on_off_gpio, 0);
		msleep(10);
		gpio_set_value_cansleep(gw2sg->on_off_gpio, 1);
		gw2sg->is_on = 0;
	}

	return 0;
}

static int gpio_w2sg_resume(struct device *dev)
{
	struct gpio_w2sg *gw2sg = dev_get_drvdata(dev);

	spin_lock_irq(&gw2sg->lock);
	gw2sg->suspended = 0;
	spin_unlock_irq(&gw2sg->lock);

	pr_info("GPS resuming %d %d %d\n", gw2sg->requested, gw2sg->is_on, gw2sg->lna_is_off);

	schedule_delayed_work(&gw2sg->work, 0);	/* enables LNA if needed */

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

SIMPLE_DEV_PM_OPS(w2sg_pm_ops, gpio_w2sg_suspend, gpio_w2sg_resume);

static struct platform_driver gpio_w2sg_driver = {
	.probe		= gpio_w2sg_probe,
	.remove		= gpio_w2sg_remove,
	.driver = {
		.name	= "w2sg0004",
		.owner	= THIS_MODULE,
		.pm	= &w2sg_pm_ops,
		.of_match_table = of_match_ptr(w2sg0004_of_match)
	},
};

module_platform_driver(gpio_w2sg_driver);

MODULE_ALIAS("w2sg0004");

MODULE_AUTHOR("NeilBrown <neilb@suse.de>");
MODULE_DESCRIPTION("w2sg0004 GPS power management driver");
MODULE_LICENSE("GPL v2");
