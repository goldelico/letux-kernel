/*
 * gpio-w2sg0004
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
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/gpio-w2sg0004.h>
#include <linux/workqueue.h>

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
	int		is_on;
	unsigned long	last_toggle;
	unsigned long	backoff;	/* time to wait since last_toggle */
	int		on_off_gpio;
	int		rx_gpio;
	int		rx_irq;

	u16		on_state;  /* Mux state when GPS is on */
	u16		off_state; /* Mux state when GPS is off */

	enum {idle, down, up} state;
	int		requested;
	int		suspended;
	int		rx_redirected;
	spinlock_t	lock;
	struct gpio_chip gpio;
	struct delayed_work work;
};

void omap_mux_set_gpio(u16 val, int gpio);
static void toggle_work(struct work_struct *work)
{
	struct gpio_w2sg *gw2sg = container_of(
		work, struct gpio_w2sg, work.work);
	switch (gw2sg->state) {
	case up:
		gw2sg->state = idle;
		printk("GPS idle\n");
	case idle:
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
		gw2sg->state = down;
		schedule_delayed_work(&gw2sg->work,
				      msecs_to_jiffies(10));
		break;
	case down:
		gpio_set_value_cansleep(gw2sg->on_off_gpio, 1);
		gw2sg->state = up;
		gw2sg->last_toggle = jiffies;
		printk("GPS up\n");
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
	printk("!");
	if (!gw2sg->requested &&
	    !gw2sg->is_on &&
	    gw2sg->state == idle &&
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

static void gpio_w2sg_set_value(struct gpio_chip *gc,
				unsigned offset, int val)
{
	unsigned long flags;
	struct gpio_w2sg *gw2sg = container_of(gc, struct gpio_w2sg,
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

static int gpio_w2sg_direction_output(struct gpio_chip *gc,
				     unsigned offset, int val)
{
	gpio_w2sg_set_value(gc, offset, val);
	return 0;
}

static int gpio_w2sg_probe(struct platform_device *pdev)
{
	struct gpio_w2sg_data *pdata = pdev->dev.platform_data;
	struct gpio_w2sg *gw2sg;
	int err;

	gw2sg = kzalloc(sizeof(*gw2sg), GFP_KERNEL);
	if (gw2sg == NULL)
		return -ENOMEM;
	gw2sg->on_off_gpio = pdata->on_off_gpio;
	gw2sg->rx_gpio = pdata->rx_gpio;
	gw2sg->on_state = pdata->on_state;
	gw2sg->off_state = pdata->off_state;

	gw2sg->is_on = 0;
	gw2sg->requested = 1;
	gw2sg->state = idle;
	gw2sg->last_toggle = jiffies;
	gw2sg->backoff = HZ;

	gw2sg->gpio.label = "gpio-w2sg0004";
	gw2sg->gpio.ngpio = 1;
	gw2sg->gpio.base = pdata->ctrl_gpio;
	gw2sg->gpio.owner = THIS_MODULE;
	gw2sg->gpio.direction_output = gpio_w2sg_direction_output;
	gw2sg->gpio.set = gpio_w2sg_set_value;
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

	err = request_threaded_irq(gw2sg->rx_irq, NULL, gpio_w2sg_isr,
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
	platform_set_drvdata(pdev, gw2sg);
	return 0;

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
	if (gw2sg->state == down) {
		msleep(10);
		gpio_set_value_cansleep(gw2sg->on_off_gpio, 1);
		gw2sg->last_toggle = jiffies;
		gw2sg->is_on = !gw2sg->is_on;
		gw2sg->state = up;
	}
	if (gw2sg->state == up) {
		msleep(10);
		gw2sg->state = idle;
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

static int gpio_w2sg_resume(struct device *dev)
{
	struct gpio_w2sg *gw2sg = dev_get_drvdata(dev);

	spin_lock_irq(&gw2sg->lock);
	gw2sg->suspended = 0;
	spin_unlock_irq(&gw2sg->lock);
	printk("GPS resuming %d %d\n", gw2sg->requested, gw2sg->is_on);
	schedule_delayed_work(&gw2sg->work, 0);
	return 0;
}

SIMPLE_DEV_PM_OPS(w2sg_pm_ops, gpio_w2sg_suspend, gpio_w2sg_resume);

static struct platform_driver gpio_w2sg_driver = {
	.driver.name	= "w2s-gpio",
	.driver.owner	= THIS_MODULE,
	.driver.pm	= &w2sg_pm_ops,
	.probe		= gpio_w2sg_probe,
	.remove		= gpio_w2sg_remove,
};

static int __init gpio_w2sg_init(void)
{
	return platform_driver_register(&gpio_w2sg_driver);
}
module_init(gpio_w2sg_init);

static void __exit gpio_w2sg_exit(void)
{
	platform_driver_unregister(&gpio_w2sg_driver);
}
module_exit(gpio_w2sg_exit);

MODULE_AUTHOR("NeilBrown <neilb@suse.de>");
MODULE_DESCRIPTION("w2sg0004 GPS virtual GPIO driver");
MODULE_LICENSE("GPL v2");
