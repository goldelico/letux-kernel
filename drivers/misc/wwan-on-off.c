/*
 * wwan_on_off
 * driver for controlling power states of some WWAN modules
 * like the GTM601 or the PHS8.
 *
 * Such modules usually have some ON_KEY or IGNITE input
 * that can be triggered to turn the modem power on or off
 * by giving a sufficiently long (200ms) impulse.
 *
 * Some modules have a power-is-on feedback that can be fed
 * into another GPIO so that the driver knows the real state.
 *
 * If this is not available we can monitor some USB PHY
 * port which becomes active if the modem is powered on.
 * (not implemented)
 *
 * The driver is based on the w2sg0004 driver developed
 * by Neil Brown.
 */

/// NOTE: this driver code is only a first draft and needs
/// some work (rename function, correctly handle GPIOs and
/// USB PHY

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/wwan-on-off.h>
#include <linux/workqueue.h>
#include <linux/rfkill.h>

struct wwan_on_off {
	struct rfkill *rf_kill;
	int		on_off_gpio;	/* may be invalid */
	int		feedback_gpio;	/* may be invalid */

	int		is_power_off;		/* current state */
	int		should_power_off;	/* expected state */
	spinlock_t	lock;
#ifdef CONFIG_GPIOLIB
	struct gpio_chip gpio;
	const char	*gpio_name[1];
#endif
};

/* this requires this driver to be compiled into the kernel! */

void omap_mux_set_gpio(u16 val, int gpio);

#if OLD
static void toggle_work(struct work_struct *work)
{
	struct wwan_on_off *gw2sg = container_of(
		work, struct wwan_on_off, work.work);
	switch (gw2sg->state) {
	case W2SG_UP:
		gw2sg->state = W2SG_IDLE;
		printk("WWAN idle\n");
	case W2SG_IDLE:
		spin_lock_irq(&gw2sg->lock);
		if (gw2sg->requested == gw2sg->is_on) {
			if (!gw2sg->is_on && !gw2sg->rx_redirected) {
				gw2sg->rx_redirected = 1;
				omap_mux_set_gpio(gw2sg->off_state,
						  gw2sg->feedback_gpio);
				enable_irq(gw2sg->rx_irq);
			}
			spin_unlock_irq(&gw2sg->lock);
			return;
		}
		spin_unlock_irq(&gw2sg->lock);
		gpio_set_value_cansleep(gw2sg->on_off_gpio, 0);
		printk("WWAN down\n");
		gw2sg->state = W2SG_DOWN;
		schedule_delayed_work(&gw2sg->work,
				      msecs_to_jiffies(10));
		break;
	case W2SG_DOWN:
		gpio_set_value_cansleep(gw2sg->on_off_gpio, 1);
		gw2sg->state = W2SG_UP;
		gw2sg->last_toggle = jiffies;
		printk("WWAN up\n");
		gw2sg->is_on = !gw2sg->is_on;
		schedule_delayed_work(&gw2sg->work,
				      msecs_to_jiffies(10));
		break;
	}
}

static irqreturn_t wwan_on_off_isr(int irq, void *dev_id)
{
	struct wwan_on_off *gw2sg = dev_id;
	unsigned long flags;
	printk("!");
	if (!gw2sg->requested &&
	    !gw2sg->is_on &&
	    gw2sg->state == W2SG_IDLE &&
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
	return IRQ_HANDLED;
}
#endif

static int is_powered_off(struct wwan_on_off *gw2sg)
{ /* check with physical interfaces if possible */
	if (gpio_is_valid(gw2sg->feedback_gpio))
		return !gpio_get_value(gw2sg->feedback_gpio);	/* read gpio */
	/* check with PHY if available */
	if (!gpio_is_valid(gw2sg->on_off_gpio))
		return 0;	/* we can't even control power, assume it is on */
	return gw2sg->is_power_off;	/* assume that we know the correct state */
}

static void set_power(struct wwan_on_off *gw2sg, int off)
{
	int state = is_powered_off(gw2sg);
	gw2sg->should_power_off = off;

	printk("modem: set_power %d -> %d\n", state, off);
	if (!gpio_is_valid(gw2sg->on_off_gpio))
		return;	/* we can't control power */
	if(state == off)
		return;	/* already in desired state */
	printk("modem: send impulse\n");
	gpio_set_value_cansleep(gw2sg->on_off_gpio, 1);
	msleep(200);	/* wait 200 ms */
	gpio_set_value_cansleep(gw2sg->on_off_gpio, 0);
	msleep(200);	/* wait 200 ms */
	gw2sg->is_power_off = off;
	printk("modem: done\n");
}

static void wwan_on_off_set_value(struct gpio_chip *gc,
				unsigned offset, int val)
{
	unsigned long flags;
	struct wwan_on_off *gw2sg = container_of(gc, struct wwan_on_off,
					       gpio);
	printk("WWAN SET to %d\n", val);
	set_power(gw2sg, !val);	/* 1 = enable, 0 = disable */
#if OLD
	spin_lock_irqsave(&gw2sg->lock, flags);
	if (val && !gw2sg->requested) {
		if (gw2sg->rx_redirected) {
			gw2sg->rx_redirected = 0;
			disable_irq(gw2sg->rx_irq);
			omap_mux_set_gpio(gw2sg->on_state, gw2sg->feedback_gpio);
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
#endif
}

static int wwan_on_off_direction_output(struct gpio_chip *gc,
				     unsigned offset, int val)
{
	wwan_on_off_set_value(gc, offset, val);
	return 0;
}

static int wwan_on_off_rfkill_set_block(void *data, bool blocked)
{
	struct wwan_on_off *gw2sg = data;
	int ret = 0;
	printk("%s: blocked: %d\n", __func__, blocked);

	pr_debug("%s: blocked: %d\n", __func__, blocked);
	if (!gpio_is_valid(gw2sg->on_off_gpio))
		return -EIO;	/* can't block if we have no control */

	set_power(gw2sg, blocked);
	return ret;
}

static struct rfkill_ops wwan_on_off_rfkill_ops = {
	// get status to read feedback gpio as HARD block (?)
	.set_block = wwan_on_off_rfkill_set_block,
};

static int wwan_on_off_probe(struct platform_device *pdev)
{
	struct wwan_on_off_data *pdata = dev_get_platdata(&pdev->dev);
	struct wwan_on_off *gw2sg;
	struct rfkill *rf_kill;
	int err;
	printk("wwan_on_off_probe()\n");

#ifdef CONFIG_OF

	if (pdev->dev.of_node) {
		struct device *dev = &pdev->dev;
		enum of_gpio_flags flags;
		pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;
		pdata->on_off_gpio = of_get_named_gpio_flags(dev->of_node, "on-off-gpio", 0, &flags);
		pdata->feedback_gpio = of_get_named_gpio_flags(dev->of_node, "on-indicator-gpio", 0, &flags);
		if (pdata->on_off_gpio == -EPROBE_DEFER ||
			pdata->feedback_gpio == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		// get optional reference to USB PHY (through "usb-port")
		pdata->gpio_base = -1;
		pdev->dev.platform_data = pdata;
		printk("wwan_on_off_probe() pdata=%p\n", pdata);
	}
#endif

	gw2sg = kzalloc(sizeof(*gw2sg), GFP_KERNEL);
	if (gw2sg == NULL)
		return -ENOMEM;
	gw2sg->on_off_gpio = pdata->on_off_gpio;
	gw2sg->feedback_gpio = pdata->feedback_gpio;

	gw2sg->gpio_name[0] = "enable";	/* label of controlling GPIO */

	gw2sg->gpio.label = "gpio-wwan-on-off";
	gw2sg->gpio.names = gw2sg->gpio_name;
	gw2sg->gpio.ngpio = 1;
	gw2sg->gpio.base = pdata->gpio_base;
	gw2sg->gpio.owner = THIS_MODULE;
	gw2sg->gpio.direction_output = wwan_on_off_direction_output;
	gw2sg->gpio.set = wwan_on_off_set_value;
	gw2sg->gpio.can_sleep = 0;

#ifdef CONFIG_OF_GPIO
	gw2sg->gpio.of_node = pdev->dev.of_node;
#endif

	gw2sg->is_power_off = 1;	/* assume initial power is off */

	if (!gpio_is_valid(gw2sg->on_off_gpio))
		pr_warn("wwan-on-off: I have no control over modem\n");

#if OLD
	INIT_DELAYED_WORK(&gw2sg->work, toggle_work);
	spin_lock_init(&gw2sg->lock);
#endif
	if (gpio_is_valid(gw2sg->feedback_gpio)) {
		err = gpio_request(gw2sg->on_off_gpio, "on-off-gpio");
		if (err < 0)
			goto out;
		gpio_direction_output(gw2sg->on_off_gpio, false);
	}
	if (gpio_is_valid(gw2sg->feedback_gpio)) {
		err = gpio_request(gw2sg->feedback_gpio, "on-indicator-gpio");
		if (err < 0)
			goto out1;
		gpio_direction_input(gw2sg->feedback_gpio);
	}

#if OLD
	gw2sg->rx_irq = gpio_to_irq(gw2sg->feedback_gpio);
	if (gw2sg->rx_irq < 0)
		goto out2;

	err = request_threaded_irq(gw2sg->rx_irq, NULL, wwan_on_off_isr,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "gpio-w2sg0004-rx",
				   gw2sg);
	if (err < 0) {
		dev_err(&pdev->dev, "Unable to claim irq %d; error %d\n",
			gw2sg->rx_irq, err);
		goto out2;
	}
	disable_irq(gw2sg->rx_irq);
#endif

	err = gpiochip_add(&gw2sg->gpio);
	if (err)
		goto out3;

	rf_kill = rfkill_alloc("WWAN", &pdev->dev,
						   RFKILL_TYPE_WWAN,
						   &wwan_on_off_rfkill_ops, gw2sg);
	if (rf_kill == NULL) {
		err = -ENOMEM;
		goto out4;
	}

	rfkill_init_sw_state(rf_kill, is_powered_off(gw2sg));	/* check initial state */

	err = rfkill_register(rf_kill);
	if (err) {
		dev_err(&pdev->dev, "Cannot register rfkill device\n");
		goto out5;
	}

	gw2sg->rf_kill = rf_kill;

	platform_set_drvdata(pdev, gw2sg);
	printk("wwan-on-off probed\n");
	return 0;

out5:
	rfkill_destroy(rf_kill);
out4:
	err = gpiochip_remove(&gw2sg->gpio);
out3:
#if OLD
	free_irq(gw2sg->rx_irq, gw2sg);
#endif
out2:
	if (gpio_is_valid(gw2sg->feedback_gpio))
		gpio_free(gw2sg->feedback_gpio);
out1:
	if (gpio_is_valid(gw2sg->on_off_gpio))
		gpio_free(gw2sg->on_off_gpio);
out:
	kfree(gw2sg);
	return err;
}

static int wwan_on_off_remove(struct platform_device *pdev)
{
	struct wwan_on_off *gw2sg = platform_get_drvdata(pdev);
	int ret;
#if OLD
	cancel_delayed_work_sync(&gw2sg->work);
#endif
	ret = gpiochip_remove(&gw2sg->gpio);
	if (ret)
		return ret;
#if OLD
	free_irq(gw2sg->rx_irq, gw2sg);
#endif
	if (gpio_is_valid(gw2sg->feedback_gpio))
		gpio_free(gw2sg->feedback_gpio);
	if (gpio_is_valid(gw2sg->on_off_gpio))
		gpio_free(gw2sg->on_off_gpio);
	kfree(gw2sg);
	return 0;
}

static int wwan_on_off_suspend(struct device *dev)
{
	/* we only suspend the driver (i.e. set the gpio in a state
	 * that it does not harm)
	 * the reason is that the modem must continue to be powered
	 * on to receive SMS and incoming calls that wake up the CPU
	 * through a wakeup GPIO
	 */
#if OLD
	/* Ignore the GPIO and just turn device off.
	 * we cannot really wait for a separate thread to
	 * do things, so we disable that and do it all
	 * here
	 */
	struct wwan_on_off *gw2sg = dev_get_drvdata(dev);

	spin_lock_irq(&gw2sg->lock);
	gw2sg->suspended = 1;
	spin_unlock_irq(&gw2sg->lock);

	cancel_delayed_work_sync(&gw2sg->work);
	if (gw2sg->state == W2SG_DOWN) {
		msleep(10);
		gpio_set_value_cansleep(gw2sg->on_off_gpio, 1);
		gw2sg->last_toggle = jiffies;
		gw2sg->is_on = !gw2sg->is_on;
		gw2sg->state = W2SG_UP;
	}
	if (gw2sg->state == W2SG_UP) {
		msleep(10);
		gw2sg->state = W2SG_IDLE;
	}
	if (gw2sg->is_on) {
		printk("WWAN off for suspend %d %d\n", gw2sg->requested, gw2sg->is_on);
		gpio_set_value_cansleep(gw2sg->on_off_gpio, 0);
		msleep(10);
		gpio_set_value_cansleep(gw2sg->on_off_gpio, 1);
		gw2sg->is_on = 0;
	}
#endif
	return 0;
}

static int wwan_on_off_resume(struct device *dev)
{
	struct wwan_on_off *gw2sg = dev_get_drvdata(dev);

	spin_lock_irq(&gw2sg->lock);
#if OLD
	gw2sg->suspended = 0;
#endif
	spin_unlock_irq(&gw2sg->lock);
#if OLD
	printk("WWAN resuming %d %d\n", gw2sg->requested, gw2sg->is_on);
	schedule_delayed_work(&gw2sg->work, 0);
#endif
	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id wwan_of_match[] = {
	{ .compatible = "option,gtm601" },
	{ .compatible = "gemalto,phs8" },
	{ .compatible = "gemalto,pls8" },
	{},
};
MODULE_DEVICE_TABLE(of, wwan_of_match);
#endif

SIMPLE_DEV_PM_OPS(wwan_on_off_pm_ops, wwan_on_off_suspend, wwan_on_off_resume);

static struct platform_driver wwan_on_off_driver = {
	.driver.name	= "wwan-on-off",
	.driver.owner	= THIS_MODULE,
	.driver.pm	= &wwan_on_off_pm_ops,
	.driver.of_match_table = of_match_ptr(wwan_of_match),
	.probe		= wwan_on_off_probe,
	.remove		= wwan_on_off_remove,
};

static int __init wwan_on_off_init(void)
{
	printk("wwan_on_off_init\n");
	return platform_driver_register(&wwan_on_off_driver);
}
module_init(wwan_on_off_init);

static void __exit wwan_on_off_exit(void)
{
	platform_driver_unregister(&wwan_on_off_driver);
}
module_exit(wwan_on_off_exit);


MODULE_ALIAS("wwan_on_off");

MODULE_AUTHOR("Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("3G Modem virtual GPIO and rfkill driver");
MODULE_LICENSE("GPL v2");
