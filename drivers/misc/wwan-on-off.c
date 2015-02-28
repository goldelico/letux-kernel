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

#define DEBUG

struct wwan_on_off {
	struct rfkill *rf_kill;
	int		on_off_gpio;	/* may be invalid */
	int		feedback_gpio;	/* may be invalid */

	int		is_power_off;		/* current state */
	spinlock_t	lock;
#ifdef CONFIG_GPIOLIB
	struct gpio_chip gpio;
	const char	*gpio_name[1];
#endif
};

/* this requires this driver to be compiled into the kernel! */

void omap_mux_set_gpio(u16 val, int gpio);

static int is_powered_off(struct wwan_on_off *wwan)
{ /* check with physical interfaces if possible */
	if (gpio_is_valid(wwan->feedback_gpio))
		return !gpio_get_value(wwan->feedback_gpio);	/* read gpio */
	/* check with PHY if available */
	if (!gpio_is_valid(wwan->on_off_gpio))
		return 0;	/* we can't even control power, assume it is on */
	return wwan->is_power_off;	/* assume that we know the correct state */
}

static void set_power(struct wwan_on_off *wwan, int off)
{
	int state;
#ifdef DEBUG
	printk("modem: set_power %d\n", off);
#endif
	if (!gpio_is_valid(wwan->on_off_gpio))
		return;	/* we can't control power */

	spin_lock_irq(&wwan->lock);	/* block other processes who want to change the state */

	state = is_powered_off(wwan);

#ifdef DEBUG
	printk("  state %d\n", state);
#endif

	if(state != off) {
#ifdef DEBUG
		printk("modem: send impulse\n");
#endif
		gpio_set_value_cansleep(wwan->on_off_gpio, 1);
		// FIXME: check feedback_gpio for early end of impulse
		msleep(200);	/* wait 200 ms */
		gpio_set_value_cansleep(wwan->on_off_gpio, 0);
		msleep(500);	/* wait 500 ms */
		wwan->is_power_off = off;
	}
	spin_unlock_irq(&wwan->lock);
#ifdef DEBUG
	printk("modem: done\n");
#endif
}

static int wwan_on_off_get_value(struct gpio_chip *gc,
						   unsigned offset)
{
	struct wwan_on_off *wwan = container_of(gc, struct wwan_on_off,
											 gpio);
	int state;
	spin_lock_irq(&wwan->lock);	/* block other processes who change the state */
	state = is_powered_off(wwan);
	spin_unlock_irq(&wwan->lock);
	return !state;
}

static void wwan_on_off_set_value(struct gpio_chip *gc,
				unsigned offset, int val)
{
	struct wwan_on_off *wwan = container_of(gc, struct wwan_on_off,
					       gpio);
	if(offset > 0)
		; // error
#ifdef DEBUG
	printk("WWAN GPIO set value %d\n", val);
#endif
	set_power(wwan, !val);	/* 1 = enable, 0 = disable */
}

static int wwan_on_off_direction_output(struct gpio_chip *gc,
				     unsigned offset, int val)
{
#ifdef DEBUG
	printk("WWAN GPIO set output %d\n", val);
#endif
	if(offset > 0)
		; // error
//	wwan_on_off_set_value(gc, offset, val);
	return 0;
}

static int wwan_on_off_rfkill_set_block(void *data, bool blocked)
{
	struct wwan_on_off *wwan = data;
	int ret = 0;
#ifdef DEBUG
	printk("%s: blocked: %d\n", __func__, blocked);
#endif
	pr_debug("%s: blocked: %d\n", __func__, blocked);
	if (!gpio_is_valid(wwan->on_off_gpio))
		return -EIO;	/* can't block if we have no control */

	set_power(wwan, blocked);
	return ret;
}

static struct rfkill_ops wwan_on_off_rfkill_ops = {
	// get status to read feedback gpio as HARD block (?)
	.set_block = wwan_on_off_rfkill_set_block,
};

static int wwan_on_off_probe(struct platform_device *pdev)
{
	struct wwan_on_off_data *pdata = dev_get_platdata(&pdev->dev);
	struct wwan_on_off *wwan;
	struct rfkill *rf_kill;
	int err;
#ifdef DEBUG
	printk("wwan_on_off_probe()\n");
#endif
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
#ifdef DEBUG
		printk("wwan_on_off_probe() pdata=%p\n", pdata);
#endif
	}
#endif

	wwan = kzalloc(sizeof(*wwan), GFP_KERNEL);
	if (wwan == NULL)
		return -ENOMEM;
	wwan->on_off_gpio = pdata->on_off_gpio;
	wwan->feedback_gpio = pdata->feedback_gpio;

	wwan->gpio_name[0] = "gpio-wwan-enable";	/* label of controlling GPIO */

	wwan->gpio.label = "gpio-wwan-on-off";
	wwan->gpio.names = wwan->gpio_name;
	wwan->gpio.ngpio = 1;
	wwan->gpio.base = pdata->gpio_base;
	wwan->gpio.owner = THIS_MODULE;
	wwan->gpio.direction_output = wwan_on_off_direction_output;
	wwan->gpio.get = wwan_on_off_get_value;
	wwan->gpio.set = wwan_on_off_set_value;
	wwan->gpio.can_sleep = 0;

#ifdef CONFIG_OF_GPIO
	wwan->gpio.of_node = pdev->dev.of_node;
#endif

	wwan->is_power_off = 1;	/* assume initial power is off */

	spin_lock_init(&wwan->lock);

	if (gpio_is_valid(wwan->on_off_gpio)) {
		err = gpio_request(wwan->on_off_gpio, "on-off-gpio");
		if (err < 0)
			goto out;
		gpio_direction_output(wwan->on_off_gpio, 0);	/* initially off */
	} else
		pr_warn("wwan-on-off: I have no control over modem\n");

	if (gpio_is_valid(wwan->feedback_gpio)) {
		err = gpio_request(wwan->feedback_gpio, "on-indicator-gpio");
		if (err < 0)
			goto out1;
		gpio_direction_input(wwan->feedback_gpio);
	}

	err = gpiochip_add(&wwan->gpio);
	if (err)
		goto out3;

	rf_kill = rfkill_alloc("WWAN", &pdev->dev,
						   RFKILL_TYPE_WWAN,
						   &wwan_on_off_rfkill_ops, wwan);
	if (rf_kill == NULL) {
		err = -ENOMEM;
		goto out4;
	}

	rfkill_init_sw_state(rf_kill, is_powered_off(wwan));	/* check initial state */

	err = rfkill_register(rf_kill);
	if (err) {
		dev_err(&pdev->dev, "Cannot register rfkill device\n");
		goto out5;
	}

	wwan->rf_kill = rf_kill;

	platform_set_drvdata(pdev, wwan);
#ifdef DEBUG
	printk("wwan-on-off probed\n");
#endif
	return 0;

out5:
	rfkill_destroy(rf_kill);
out4:
	gpiochip_remove(&wwan->gpio);
out3:
	if (gpio_is_valid(wwan->feedback_gpio))
		gpio_free(wwan->feedback_gpio);
out1:
	if (gpio_is_valid(wwan->on_off_gpio))
		gpio_free(wwan->on_off_gpio);
out:
	kfree(wwan);
	return err;
}

static int wwan_on_off_remove(struct platform_device *pdev)
{
	struct wwan_on_off *wwan = platform_get_drvdata(pdev);
	gpiochip_remove(&wwan->gpio);
	if (gpio_is_valid(wwan->feedback_gpio))
		gpio_free(wwan->feedback_gpio);
	if (gpio_is_valid(wwan->on_off_gpio))
		gpio_free(wwan->on_off_gpio);
	kfree(wwan);
	return 0;
}

static int wwan_on_off_suspend(struct device *dev)
{
	struct wwan_on_off *wwan = dev_get_drvdata(dev);
#ifdef DEBUG
	printk("WWAN suspend\n");
#endif
	/* we only suspend the driver (i.e. set the gpio in a state
	 * that it does not harm)
	 * the reason is that the modem must continue to be powered
	 * on to receive SMS and incoming calls that wake up the CPU
	 * through a wakeup GPIO
	 * what we could do is to decode a system power-off
	 */
	return 0;
}

static int wwan_on_off_resume(struct device *dev)
{
	struct wwan_on_off *wwan = dev_get_drvdata(dev);
#ifdef DEBUG
	printk("WWAN resume\n");
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
