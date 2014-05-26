/*
 * GTM601
 * driver for controlling some WWAN module like the GTM601
 * or the PHS8.
 *
 * Such modules usually have some ON_KEY or IGNITE input
 * that can be triggered to turn the modem power on or off
 * by giving an sufficiently long (200ms) impulse.
 *
 * Some modules have a power on feedback that can be fed
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
#include <linux/gpio-w2sg0004.h>
#include <linux/workqueue.h>
#include <linux/rfkill.h>

/*
 * There seems to restrictions on how quickly we can toggle the
 * on/off line.  data sheets says "two rtc ticks", whatever that means.
 * If we do it too soon it doesn't work.
 * So we have a state machine which uses the common work queue to ensure
 * clean transitions.
 * When a change is requested we record that request and only act on it
 * once the previous change has completed.
 * A change involves a 10ms low pulse, and a 990ms raised level, so only
 * one change per second.
 */

struct gpio_gtm601 {
	struct rfkill *rf_kill;
	int		lna_gpio;
	int		lna_blocked;
	int		is_on;
	unsigned long	last_toggle;
	unsigned long	backoff;	/* time to wait since last_toggle */
	int		on_off_gpio;
	int		rx_gpio;
	int		rx_irq;

	u16		on_state;  /* Mux state when GPS is on */
	u16		off_state; /* Mux state when GPS is off */

	enum {W2SG_IDLE, W2SG_DOWN, W2SG_UP} state;
	int		requested;
	int		suspended;
	int		rx_redirected;
	spinlock_t	lock;
	struct gpio_chip gpio;
	struct delayed_work work;
};

/* this requires this driver to be compiled into the kernel! */

void omap_mux_set_gpio(u16 val, int gpio);

static void toggle_work(struct work_struct *work)
{
	struct gpio_gtm601 *gw2sg = container_of(
		work, struct gpio_gtm601, work.work);
	switch (gw2sg->state) {
	case W2SG_UP:
		gw2sg->state = W2SG_IDLE;
		printk("GPS idle\n");
	case W2SG_IDLE:
		spin_lock_irq(&gw2sg->lock);
		if (gw2sg->requested == gw2sg->is_on) {
			if (!gw2sg->is_on && !gw2sg->rx_redirected) {
				gw2sg->rx_redirected = 1;
				omap_mux_set_gpio(gw2sg->off_state,
						  gw2sg->rx_gpio);
				enable_irq(gw2sg->rx_irq);
			}
			spin_unlock_irq(&gw2sg->lock);
			return;
		}
		spin_unlock_irq(&gw2sg->lock);
		gpio_set_value_cansleep(gw2sg->on_off_gpio, 0);
		printk("GPS down\n");
		gw2sg->state = W2SG_DOWN;
		schedule_delayed_work(&gw2sg->work,
				      msecs_to_jiffies(10));
		break;
	case W2SG_DOWN:
		gpio_set_value_cansleep(gw2sg->on_off_gpio, 1);
		gw2sg->state = W2SG_UP;
		gw2sg->last_toggle = jiffies;
		printk("GPS up\n");
		gw2sg->is_on = !gw2sg->is_on;
		schedule_delayed_work(&gw2sg->work,
				      msecs_to_jiffies(10));
		break;
	}
}

static irqreturn_t gpio_gtm601_isr(int irq, void *dev_id)
{
	struct gpio_gtm601 *gw2sg = dev_id;
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

static void gpio_gtm601_set_value(struct gpio_chip *gc,
				unsigned offset, int val)
{
	unsigned long flags;
	struct gpio_gtm601 *gw2sg = container_of(gc, struct gpio_gtm601,
					       gpio);
	printk("GPS SET to %d\n", val);
	spin_lock_irqsave(&gw2sg->lock, flags);
	if (val && !gw2sg->requested) {
		if (gw2sg->rx_redirected) {
			gw2sg->rx_redirected = 0;
			disable_irq(gw2sg->rx_irq);
			omap_mux_set_gpio(gw2sg->on_state, gw2sg->rx_gpio);
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

static int gpio_gtm601_direction_output(struct gpio_chip *gc,
				     unsigned offset, int val)
{
	gpio_gtm601_set_value(gc, offset, val);
	return 0;
}

static int gpio_gtm601_rfkill_set_block(void *data, bool blocked)
{
	struct gpio_gtm601 *rfkill_data = data;
	int ret = 0;

	pr_debug("%s: blocked: %d\n", __func__, blocked);

	rfkill_data -> lna_blocked = blocked;

	// forward !lna_blocked && is_on to lna_gpio state

	return ret;
}

static struct rfkill_ops gpio_gtm601_rfkill_regulator_ops = {
	.set_block = gpio_gtm601_rfkill_set_block,
};

static int gpio_gtm601_probe(struct platform_device *pdev)
{
	struct gpio_w2sg_data *pdata = dev_get_platdata(&pdev->dev);
	struct gpio_gtm601 *gw2sg;
	struct rfkill *rf_kill;
	int err;
	printk("gpio_gtm601_probe()\n");

	if (pdev->dev.of_node) {
		struct device *dev = &pdev->dev;
		enum of_gpio_flags flags;
		int mux_state;
		pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;
		pdata->lna_gpio = of_get_named_gpio_flags(dev->of_node, "lna-gpio", 0, &flags);
		pdata->on_off_gpio = of_get_named_gpio_flags(dev->of_node, "on-off-gpio", 0, &flags);
		pdata->rx_gpio = of_get_named_gpio_flags(dev->of_node, "rx-gpio", 0, &flags);
		if (of_property_read_u32(dev->of_node, "rx-on-mux", &mux_state))
			pdata->on_state = mux_state;
		if (of_property_read_u32(dev->of_node, "rx-off-mux", &mux_state))
			pdata->off_state = mux_state;
		if (pdata->on_off_gpio == -EPROBE_DEFER ||
			pdata->rx_gpio == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		pdata->ctrl_gpio = -1;
		pdev->dev.platform_data = pdata;
		printk("gpio_gtm601_probe() pdata=%p\n", pdata);
	}

	gw2sg = kzalloc(sizeof(*gw2sg), GFP_KERNEL);
	if (gw2sg == NULL)
		return -ENOMEM;
	gw2sg->lna_gpio = pdata->lna_gpio;
	gw2sg->on_off_gpio = pdata->on_off_gpio;
	gw2sg->rx_gpio = pdata->rx_gpio;
	gw2sg->on_state = pdata->on_state;
	gw2sg->off_state = pdata->off_state;

	gw2sg->is_on = 0;
	gw2sg->requested = 1;
	gw2sg->state = W2SG_IDLE;
	gw2sg->last_toggle = jiffies;
	gw2sg->backoff = HZ;

	gw2sg->gpio.label = "gpio-w2sg0004";
	gw2sg->gpio.ngpio = 1;
	gw2sg->gpio.base = pdata->ctrl_gpio;
	gw2sg->gpio.owner = THIS_MODULE;
	gw2sg->gpio.direction_output = gpio_gtm601_direction_output;
	gw2sg->gpio.set = gpio_gtm601_set_value;
	gw2sg->gpio.can_sleep = 0;
	INIT_DELAYED_WORK(&gw2sg->work, toggle_work);
	spin_lock_init(&gw2sg->lock);

	err = gpio_request(gw2sg->on_off_gpio, "gpio-w2sg0004-on-off");
	if (err < 0)
		goto out;
	gpio_direction_output(gw2sg->on_off_gpio, false);

	err = gpio_request(gw2sg->rx_gpio, "gpio-w2sg0004-rx");
	if (err < 0)
		goto out1;
	gpio_direction_input(gw2sg->rx_gpio);

	gw2sg->rx_irq = gpio_to_irq(gw2sg->rx_gpio);
	if (gw2sg->rx_irq < 0)
		goto out2;

	err = request_threaded_irq(gw2sg->rx_irq, NULL, gpio_gtm601_isr,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "gpio-w2sg0004-rx",
				   gw2sg);
	if (err < 0) {
		dev_err(&pdev->dev, "Unable to claim irq %d; error %d\n",
			gw2sg->rx_irq, err);
		goto out2;
	}
	disable_irq(gw2sg->rx_irq);
	err = gpiochip_add(&gw2sg->gpio);
	if (err)
		goto out3;

	rf_kill = rfkill_alloc("GPS", &pdev->dev,
						   RFKILL_TYPE_GPS,
						   &gpio_gtm601_rfkill_regulator_ops, gw2sg);
	if (rf_kill == NULL) {
		err = -ENOMEM;
		goto out4;
	}

	err = rfkill_register(rf_kill);
	if (err) {
		dev_err(&pdev->dev, "Cannot register rfkill device\n");
		goto out5;
	}

	gw2sg->rf_kill = rf_kill;

	platform_set_drvdata(pdev, gw2sg);
	printk("w2sg0004 probed\n");
	return 0;

out5:
	rfkill_destroy(rf_kill);
out4:
	err = gpiochip_remove(&gw2sg->gpio);
out3:
	free_irq(gw2sg->rx_irq, gw2sg);
out2:
	gpio_free(gw2sg->rx_gpio);
out1:
	gpio_free(gw2sg->on_off_gpio);
out:
	kfree(gw2sg);
	return err;
}

static int gpio_gtm601_remove(struct platform_device *pdev)
{
	struct gpio_gtm601 *gw2sg = platform_get_drvdata(pdev);
	int ret;

	cancel_delayed_work_sync(&gw2sg->work);
	ret = gpiochip_remove(&gw2sg->gpio);
	if (ret)
		return ret;
	free_irq(gw2sg->rx_irq, gw2sg);
	gpio_free(gw2sg->rx_gpio);
	gpio_free(gw2sg->on_off_gpio);
	kfree(gw2sg);
	return 0;
}

static int gpio_gtm601_suspend(struct device *dev)
{
	/* Ignore the GPIO and just turn device off.
	 * we cannot really wait for a separate thread to
	 * do things, so we disable that and do it all
	 * here
	 */
	struct gpio_gtm601 *gw2sg = dev_get_drvdata(dev);

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
		printk("GPS off for suspend %d %d\n", gw2sg->requested, gw2sg->is_on);
		gpio_set_value_cansleep(gw2sg->on_off_gpio, 0);
		msleep(10);
		gpio_set_value_cansleep(gw2sg->on_off_gpio, 1);
		gw2sg->is_on = 0;
	}
	return 0;
}

static int gpio_gtm601_resume(struct device *dev)
{
	struct gpio_gtm601 *gw2sg = dev_get_drvdata(dev);

	spin_lock_irq(&gw2sg->lock);
	gw2sg->suspended = 0;
	spin_unlock_irq(&gw2sg->lock);
	printk("GPS resuming %d %d\n", gw2sg->requested, gw2sg->is_on);
	schedule_delayed_work(&gw2sg->work, 0);
	return 0;
}

SIMPLE_DEV_PM_OPS(gtm601_pm_ops, gpio_gtm601_suspend, gpio_gtm601_resume);

static struct platform_driver gpio_gtm601_driver = {
	.driver.name	= "3g-gpio",
	.driver.owner	= THIS_MODULE,
	.driver.pm	= &gtm601_pm_ops,
	.probe		= gpio_gtm601_probe,
	.remove		= gpio_gtm601_remove,
};

static int __init gpio_gtm601_init(void)
{
	return platform_driver_register(&gpio_gtm601_driver);
}
module_init(gpio_gtm601_init);

static void __exit gpio_gtm601_exit(void)
{
	platform_driver_unregister(&gpio_gtm601_driver);
}
module_exit(gpio_gtm601_exit);

#if defined(CONFIG_OF)
static const struct of_device_id w2sg0004_of_match[] = {
	{ .compatible = "option,gtm601" },
	{ .compatible = "gemalto,phs8" },
	{ .compatible = "gemalto,pls8" },
	{},
};
MODULE_DEVICE_TABLE(of, w2sg0004_of_match);
#endif

MODULE_ALIAS("gtm601");

MODULE_AUTHOR("Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("3G Modem virtual GPIO and rfkill driver");
MODULE_LICENSE("GPL v2");
