/*
 * wwan_on_off
 * driver for controlling power states of some WWAN modules
 * like the GTM601 or the PHS8 which are independently powered
 * from the APU so that they can continue to run during suspend
 * and potentially during power-off.
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
 *
 * The driver is based on the w2sg0004 driver developed
 * by Neil Brown.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/usb/phy.h>
#include <linux/workqueue.h>

struct wwan_on_off {
	struct rfkill *rf_kill;
	int		on_off_gpio;	/* may be invalid */
	int		feedback_gpio;	/* may be invalid */
	bool		feedback_gpio_inverted;	/* active low */
	struct usb_phy *usb_phy;	/* USB PHY to monitor for modem activity */
	bool		is_power_on;	/* current state */
	spinlock_t	lock;
	bool		can_turnoff;	/* can also turn off by impulse */
};

static bool is_powered_on(struct wwan_on_off *wwan)
{ /* check with physical interfaces if possible */
	if (gpio_is_valid(wwan->feedback_gpio))
		return gpio_get_value_cansleep(wwan->feedback_gpio) != wwan->feedback_gpio_inverted;	/* read gpio */
	if (wwan->usb_phy != NULL && !IS_ERR(wwan->usb_phy))
		printk("wwan-on-off: USB phy event %d\n", wwan->usb_phy->last_event);
	/* check with PHY if available */
	if (!gpio_is_valid(wwan->on_off_gpio))
		return true;	/* we can't even control power, assume it is on */
	return wwan->is_power_on;	/* assume that we know the correct state */
}

static void set_power(struct wwan_on_off *wwan, bool on)
{
	int state;
#ifdef DEBUG
	printk("wwan-on-off: set_power %d\n", on);
#endif
	if (!gpio_is_valid(wwan->on_off_gpio))
		return;	/* we can't control power */

	state = is_powered_on(wwan);

#ifdef DEBUG
	printk("wwan-on-off: state %d\n", state);
#endif

	if(state != on) {
		if (!on && !wwan->can_turnoff) {
			printk("wwan-on-off: can't turn off by impulse\n");
			return;
		}
#ifdef DEBUG
		printk("wwan-on-off: send impulse\n");
#endif
		// use gpiolib to generate impulse
		gpio_set_value_cansleep(wwan->on_off_gpio, 1);
		// FIXME: check feedback_gpio for early end of impulse
		msleep(200);	/* wait 200 ms */
		gpio_set_value_cansleep(wwan->on_off_gpio, 0);
		msleep(500);	/* wait 500 ms */
		wwan->is_power_on = on;
		if (is_powered_on(wwan) != on)
			printk("wwan-on-off: failed to change modem state\n");	/* warning only! using USB feedback might not be immediate */
	}

#ifdef DEBUG
	printk("wwan-on-off: done\n");
#endif
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

	set_power(wwan, !blocked);
	return ret;
}

static struct rfkill_ops wwan_on_off_rfkill_ops = {
	// get status to read feedback gpio as HARD block (?)
	.set_block = wwan_on_off_rfkill_set_block,
};

static int wwan_on_off_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct wwan_on_off *wwan;
	struct rfkill *rf_kill;
	int err;
	enum of_gpio_flags flags;
#ifdef DEBUG
	printk("wwan-on-off: wwan_on_off_probe()\n");
#endif

	if (!pdev->dev.of_node)
		return -EINVAL;

	wwan = devm_kzalloc(dev, sizeof(*wwan), GFP_KERNEL);
	if (wwan == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, wwan);

	wwan->on_off_gpio = of_get_named_gpio_flags(dev->of_node, "on-off-gpio", 0, &flags);
	wwan->feedback_gpio = of_get_named_gpio_flags(dev->of_node, "on-indicator-gpio", 0, &flags);
	wwan->feedback_gpio_inverted = (flags & OF_GPIO_ACTIVE_LOW) != 0;
	// handle active low feedback gpio!

	wwan->usb_phy = devm_usb_get_phy_by_phandle(dev, "usb-port", 0);
	printk("wwan-on-off: onoff = %d indicator = %d act low: %d usb_phy = %ld\n", wwan->on_off_gpio, wwan->feedback_gpio, wwan->feedback_gpio_inverted, PTR_ERR(wwan->usb_phy));
	if (wwan->on_off_gpio == -EPROBE_DEFER ||
		wwan->feedback_gpio == -EPROBE_DEFER ||
		PTR_ERR(wwan->usb_phy) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	// get optional reference to USB PHY (through "usb-port")
#ifdef DEBUG
	printk("wwan-on-off: wwan_on_off_probe() wwan=%p\n", wwan);
#endif

	// FIXME: read from of_device_id table
	wwan->can_turnoff = of_device_is_compatible(dev->of_node, "option,gtm601-power");
	wwan->is_power_on = false;	/* assume initial power is off */

	spin_lock_init(&wwan->lock);

	if (gpio_is_valid(wwan->on_off_gpio)) {
		err = devm_gpio_request(dev, wwan->on_off_gpio, "on-off-gpio");
		if (err < 0)
			return err;
		gpio_direction_output(wwan->on_off_gpio, 0);	/* initially off */
	} else
		pr_warn("wwan-on-off: I have no control over modem\n");

	if (gpio_is_valid(wwan->feedback_gpio)) {
		err = devm_gpio_request(dev, wwan->feedback_gpio, "on-indicator-gpio");
		if (err < 0)
			return err;
		gpio_direction_input(wwan->feedback_gpio);
	}

	rf_kill = rfkill_alloc("WWAN", &pdev->dev,
				RFKILL_TYPE_WWAN,
				&wwan_on_off_rfkill_ops, wwan);
	if (rf_kill == NULL) {
		return -ENOMEM;
	}

	rfkill_init_sw_state(rf_kill, !is_powered_on(wwan));	/* set initial state */

	err = rfkill_register(rf_kill);
	if (err) {
		dev_err(&pdev->dev, "Cannot register rfkill device\n");
		goto err;
	}

	wwan->rf_kill = rf_kill;

#ifdef DEBUG
	printk("wwan-on-off: successfully probed\n");
#endif
	return 0;

err:
	rfkill_destroy(rf_kill);
#ifdef DEBUG
	printk("wwan-on-off: probe failed %d\n", err);
#endif
	return err;
}

static int wwan_on_off_remove(struct platform_device *pdev)
{
	struct wwan_on_off *wwan = platform_get_drvdata(pdev);
	return 0;
}

/* we only suspend the driver (i.e. set the gpio in a state
 * that it does not harm)
 * the reason is that the modem must continue to be powered
 * on to receive SMS and incoming calls that wake up the CPU
 * through a wakeup GPIO
 */

static int wwan_on_off_suspend(struct device *dev)
{
#ifdef DEBUG
	struct wwan_on_off *wwan = dev_get_drvdata(dev);
	printk("wwan-on-off: WWAN suspend\n");
#endif
	/* set gpio to harmless mode */
	return 0;
}

static int wwan_on_off_resume(struct device *dev)
{
#ifdef DEBUG
	struct wwan_on_off *wwan = dev_get_drvdata(dev);
	printk("wwan-on-off: WWAN resume\n");
#endif
	/* restore gpio */
	return 0;
}

/* on system power off we must turn off the
 * modem (which has a separate connection to
 * the battery).
 */

static int wwan_on_off_poweroff(struct device *dev)
{
	struct wwan_on_off *wwan = dev_get_drvdata(dev);
#ifdef DEBUG
	printk("wwan-on-off: WWAN poweroff\n");
#endif
	set_power(wwan, 0);	/* turn off modem */
	printk("wwan-on-off: WWAN powered off\n");
	return 0;
}

static const struct of_device_id wwan_of_match[] = {
	{ .compatible = "option,gtm601-power" },
	{ .compatible = "gemalto,phs8-power" },
	{ .compatible = "gemalto,pls8-power" },
	{},
};
MODULE_DEVICE_TABLE(of, wwan_of_match);

const struct dev_pm_ops wwan_on_off_pm_ops = {
	.suspend = wwan_on_off_suspend,
	.resume = wwan_on_off_resume,
	.freeze = wwan_on_off_suspend,
	.thaw = wwan_on_off_resume,
	.poweroff = wwan_on_off_poweroff,
	.restore = wwan_on_off_resume,
	};

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
	printk("wwan-on-off: wwan_on_off_init\n");
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
MODULE_DESCRIPTION("3G Modem rfkill and virtual GPIO driver");
MODULE_LICENSE("GPL v2");
