/*
 * w2sg0004.c
 * Virtual GPIO of controlling the w2sg0004 GPS receiver.
 *
 * This receiver has an ON/OFF pin which must be toggled to
 * turn the device 'on' of 'off'.  A high->low->high toggle
 * will switch the device on if it is off, and off if it is on.
 * It is not possible to directly detect the state of the device.
 * However when it is on it will send characters on a UART line
 * regularly.
 * On the OMAP3, the UART line can also be programmed as a GPIO
 * on which we can receive interrupts.
 * So when we want the device to be 'off' we can reprogram
 * the line, toggle the ON/OFF pin and hope that it is off.
 * However if an interrupt arrives we know that it is really on
 * and can toggle again.
 *
 * To enable receiving on/off requests we create a gpio_chip
 * with a single 'output' GPIO.  When it is low, the
 * GPS is turned off.  When it is high, it is turned on.
 * This can be configured as the DTR GPIO on the UART which
 * connects the GPS.  Then whenever the tty is open, the GPS
 * will be switched on, and whenever it is closed, the GPS will
 * be switched off.
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
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/w2sg0004.h>
#include <linux/workqueue.h>
#include <linux/rfkill.h>
#include <linux/pinctrl/consumer.h>

#define DEBUG

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

struct gpio_w2sg {
	struct rfkill *rf_kill;
	struct	regulator *lna_regulator;
	int		lna_blocked;	/* rfkill block gps active */
	int		lna_is_off;		/* LNA is currently off */
	int		is_on;	/* current state (0/1) */
	unsigned long	last_toggle;
	unsigned long	backoff;	/* time to wait since last_toggle */
	int		on_off_gpio;
	/* fixme: can be removed if we configure the rx_irq by DT */
	int		rx_gpio;
	int		rx_irq;

	struct pinctrl *p;
	struct pinctrl_state *default_state;	/* should be UART mode */
	struct pinctrl_state *monitor_state;	/* monitor RX as GPIO */

	enum {
		W2SG_IDLE,	/* is not changing state */
		W2SG_PULSE,	/* activate on/off impulse */
		W2SG_NOPULSE}	/* desctivate on/off impulse */
			state;
	int		requested;	/* requested state (0/1) */
	int		suspended;
	int		rx_redirected;
	spinlock_t	lock;
#ifdef CONFIG_GPIOLIB
	struct gpio_chip gpio;
	const char	*gpio_name[1];
#endif
	struct delayed_work work;
};

static int gpio_w2sg_set_lna_power(struct gpio_w2sg *gw2sg)
{
	int ret = 0;
	int off = gw2sg->suspended || !gw2sg->requested || gw2sg->lna_blocked;
#ifdef DEBUG
	printk("gpio_w2sg_set_lna_power: %s\n", off?"off":"on");
#endif
	if(off != gw2sg->lna_is_off) {
		gw2sg->lna_is_off = off;
		if(!IS_ERR_OR_NULL(gw2sg->lna_regulator)) {
			if(off)
#ifdef DEBUG
				printk("disable lna_regulator\n"),
#endif
				regulator_disable(gw2sg->lna_regulator);
			else
#ifdef DEBUG
				printk("enable lna_regulator\n"),
#endif
				ret = regulator_enable(gw2sg->lna_regulator);
		}
	}
	return ret;
}

static void toggle_work(struct work_struct *work)
{
	struct gpio_w2sg *gw2sg = container_of(
		work, struct gpio_w2sg, work.work);
	gpio_w2sg_set_lna_power(gw2sg);	/* update LNA power state */
	switch (gw2sg->state) {
	case W2SG_NOPULSE:
		gw2sg->state = W2SG_IDLE;
#ifdef DEBUG
		printk("GPS idle\n");
#endif
	case W2SG_IDLE:
		spin_lock_irq(&gw2sg->lock);
		if (gw2sg->requested == gw2sg->is_on) {
			if (!gw2sg->is_on && !gw2sg->rx_redirected) {
				/* not yet redirected in off state */
				gw2sg->rx_redirected = 1;
				(void) pinctrl_select_state(gw2sg->p, gw2sg->monitor_state);
				enable_irq(gw2sg->rx_irq);
			}
			spin_unlock_irq(&gw2sg->lock);
			return;
		}
		spin_unlock_irq(&gw2sg->lock);
		gpio_set_value_cansleep(gw2sg->on_off_gpio, 0);
		gw2sg->state = W2SG_PULSE;
#ifdef DEBUG
		printk("GPS pulse\n");
#endif
		schedule_delayed_work(&gw2sg->work,
				      msecs_to_jiffies(10));
		break;
	case W2SG_PULSE:
		gpio_set_value_cansleep(gw2sg->on_off_gpio, 1);
		gw2sg->last_toggle = jiffies;
		gw2sg->state = W2SG_NOPULSE;
#ifdef DEBUG
		printk("GPS nopulse\n");
#endif
		gw2sg->is_on = !gw2sg->is_on;
		schedule_delayed_work(&gw2sg->work,
				      msecs_to_jiffies(10));
		break;
	}
}

static irqreturn_t gpio_w2sg_isr(int irq, void *dev_id)
{
	struct gpio_w2sg *gw2sg = dev_id;
	unsigned long flags;
#ifdef DEBUG
	printk("!");	/* we have received a RX signal while GPS should be off */
#endif
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

static void gpio_w2sg_set_power(struct gpio_w2sg *gw2sg, int val)
{
	unsigned long flags;
#ifdef DEBUG
	printk("GPS SET to %d\n", val);
#endif
	spin_lock_irqsave(&gw2sg->lock, flags);
	if (val && !gw2sg->requested) {
		if (gw2sg->rx_redirected) {
			gw2sg->rx_redirected = 0;
			disable_irq(gw2sg->rx_irq);
			(void) pinctrl_select_state(gw2sg->p, gw2sg->default_state);
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

static int gpio_w2sg_get_value(struct gpio_chip *gc,
								unsigned offset)
{ /* virtual GPIO to enable GPS has been changed */
	struct gpio_w2sg *gw2sg = container_of(gc, struct gpio_w2sg,
										   gpio);
	return gw2sg->is_on;
}

static void gpio_w2sg_set_value(struct gpio_chip *gc,
				unsigned offset, int val)
{ /* virtual GPIO to enabele GPS has been changed */
	struct gpio_w2sg *gw2sg = container_of(gc, struct gpio_w2sg,
					       gpio);
	gpio_w2sg_set_power(gw2sg, val);
}

static int gpio_w2sg_direction_output(struct gpio_chip *gc,
				     unsigned offset, int val)
{
	gpio_w2sg_set_value(gc, offset, val);
	return 0;
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
#ifdef DEBUG
	printk("gpio_w2sg_probe()\n");
#endif

#ifdef CONFIG_OF

	if (pdev->dev.of_node) {
		struct device *dev = &pdev->dev;
		enum of_gpio_flags flags;
#ifdef DEBUG
		printk("gpio_w2sg_probe() found DT\n");
#endif
		pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;
		pdata->on_off_gpio = of_get_named_gpio_flags(dev->of_node, "on-off-gpio", 0, &flags);
		pdata->rx_gpio = of_get_named_gpio_flags(dev->of_node, "rx-gpio", 0, &flags);
		if (pdata->on_off_gpio == -EPROBE_DEFER ||
			pdata->rx_gpio == -EPROBE_DEFER)
			return -EPROBE_DEFER;	/* defer until we have all gpios */

		pdata->lna_regulator = devm_regulator_get_optional(dev, "lna");
#ifdef DEBUG
		printk("gpio_w2sg_probe() lna_regulator = %p\n", pdata->lna_regulator);
#endif
		if(IS_ERR(pdata->lna_regulator))
			return PTR_ERR(pdata->lna_regulator);
		pdata->gpio_base = -1;
		pdev->dev.platform_data = pdata;
#ifdef DEBUG
		printk("gpio_w2sg_probe() pdata=%p\n", pdata);
#endif
	}
#endif

	gw2sg = kzalloc(sizeof(*gw2sg), GFP_KERNEL);
	if (gw2sg == NULL)
		return -ENOMEM;
	gw2sg->lna_regulator = pdata->lna_regulator;
	gw2sg->lna_blocked = true;
	gw2sg->lna_is_off = true;

	gw2sg->on_off_gpio = pdata->on_off_gpio;
	gw2sg->rx_gpio = pdata->rx_gpio;
//	gw2sg->on_state = pdata->on_state;
//	gw2sg->off_state = pdata->off_state;

	gw2sg->is_on = false;
	gw2sg->requested = true;
	gw2sg->state = W2SG_IDLE;
	gw2sg->last_toggle = jiffies;
	gw2sg->backoff = HZ;

	gw2sg->gpio_name[0] = "gpio-w2sg0004-enable";	/* label of controlling GPIO */

	gw2sg->gpio.label = "w2sg0004";
	gw2sg->gpio.names = gw2sg->gpio_name;
	gw2sg->gpio.ngpio = 1;
	gw2sg->gpio.base = pdata->gpio_base;
	gw2sg->gpio.owner = THIS_MODULE;
	gw2sg->gpio.direction_output = gpio_w2sg_direction_output;
	gw2sg->gpio.get = gpio_w2sg_get_value;
	gw2sg->gpio.set = gpio_w2sg_set_value;
	gw2sg->gpio.can_sleep = 0;

#ifdef CONFIG_OF_GPIO
	gw2sg->gpio.of_node = pdev->dev.of_node;
#endif

#ifdef CONFIG_OF
	if (pdev->dev.of_node) {
		gw2sg->p = devm_pinctrl_get(&pdev->dev);
		if (IS_ERR(gw2sg->p)) {
			err = PTR_ERR(gw2sg->p);
			dev_err(&pdev->dev, "Cannot get pinctrl: %d\n", err);
			goto out;
		}

		gw2sg->default_state = pinctrl_lookup_state(gw2sg->p, PINCTRL_STATE_DEFAULT);
		if (IS_ERR(gw2sg->default_state)) {
			err = PTR_ERR(gw2sg->default_state);
			dev_err(&pdev->dev, "Cannot look up pinctrl state %s: %d\n", PINCTRL_STATE_DEFAULT, err);
			goto out;
		}

		gw2sg->monitor_state = pinctrl_lookup_state(gw2sg->p, "monitor");
		if (IS_ERR(gw2sg->monitor_state)) {
			err = PTR_ERR(gw2sg->monitor_state);
			dev_err(&pdev->dev, "Cannot look up pinctrl state %s: %d\n", "monitor", err);
			goto out;
		}
		/* choose UART state as default */
		err = pinctrl_select_state(gw2sg->p, gw2sg->default_state);
		if (err < 0) {
			goto out;
		}
	}
#endif

	INIT_DELAYED_WORK(&gw2sg->work, toggle_work);
	spin_lock_init(&gw2sg->lock);

	err = gpio_request(gw2sg->on_off_gpio, "w2sg0004-on-off");
	if (err < 0)
		goto out;
	gpio_direction_output(gw2sg->on_off_gpio, false);

	err = gpio_request(gw2sg->rx_gpio, "w2sg0004-rx");
	if (err < 0)
		goto out1;
	gpio_direction_input(gw2sg->rx_gpio);

	gw2sg->rx_irq = gpio_to_irq(gw2sg->rx_gpio);
	if (gw2sg->rx_irq < 0)
		goto out2;

	err = request_threaded_irq(gw2sg->rx_irq, NULL, gpio_w2sg_isr,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "w2sg0004-rx",
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
						   &gpio_w2sg0004_rfkill_ops, gw2sg);
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
#ifdef DEBUG
	printk("w2sg0004 probed\n");
#endif
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

static int gpio_w2sg_remove(struct platform_device *pdev)
{
	struct gpio_w2sg *gw2sg = platform_get_drvdata(pdev);
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

static int gpio_w2sg_suspend(struct device *dev)
{
	/* Ignore the GPIO and just turn device off.
	 * we cannot really wait for a separate thread to
	 * do things, so we disable that and do it all
	 * here
	 */
	struct gpio_w2sg *gw2sg = dev_get_drvdata(dev);

	spin_lock_irq(&gw2sg->lock);
	gw2sg->suspended = 1;
	spin_unlock_irq(&gw2sg->lock);

	cancel_delayed_work_sync(&gw2sg->work);

	gpio_w2sg_set_lna_power(gw2sg);	/* shut down if needed */

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
#ifdef DEBUG
		printk("GPS off for suspend %d %d %d\n", gw2sg->requested, gw2sg->is_on, gw2sg->lna_is_off);
#endif
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

#ifdef DEBUG
	printk("GPS resuming %d %d %d\n", gw2sg->requested, gw2sg->is_on, gw2sg->lna_is_off);
#endif
	schedule_delayed_work(&gw2sg->work, 0);	/* enables LNA if needed */

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id w2sg0004_of_match[] = {
	{ .compatible = "wi2wi,w2sg0004" },
	{},
};
MODULE_DEVICE_TABLE(of, w2sg0004_of_match);
#endif

SIMPLE_DEV_PM_OPS(w2sg_pm_ops, gpio_w2sg_suspend, gpio_w2sg_resume);

static struct platform_driver gpio_w2sg_driver = {
	.driver.name	= "w2sg0004",
	.driver.owner	= THIS_MODULE,
	.driver.pm	= &w2sg_pm_ops,
	.driver.of_match_table = of_match_ptr(w2sg0004_of_match),
	.probe		= gpio_w2sg_probe,
	.remove		= gpio_w2sg_remove,
};

static int __init gpio_w2sg_init(void)
{
#ifdef DEBUG
	printk("gpio_w2sg_init()\n");
#endif
	return platform_driver_register(&gpio_w2sg_driver);
}
module_init(gpio_w2sg_init);

static void __exit gpio_w2sg_exit(void)
{
	platform_driver_unregister(&gpio_w2sg_driver);
}
module_exit(gpio_w2sg_exit);

MODULE_ALIAS("w2sg0004");

MODULE_AUTHOR("NeilBrown <neilb@suse.de>");
MODULE_DESCRIPTION("w2sg0004 GPS virtual GPIO driver");
MODULE_LICENSE("GPL v2");
