/*
 * w2cbw003: create plumbing between a virtual GPIO and power regulator
 * of W2CBW003.
 *
 * This module provide a single output GPIO and uses a single regulator.
 * When the GPIO is driven high, the regulator is enabled.
 * When the GPIO is driven low, the regulator is disabled.
 *
 * We sometimes need this if other drivers assume to control a GPIO but
 * hardware needs a regulator to be controlled by that driver.
 *
 * This allows for example a serial port to be given the GPIO as a DTR signal,
 * and whenever the serial port is opened, the regulator is enabled.
 *
 * The GPIO number is passed in the platform data.
 * The regulator consumer name is "vgpio"
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/w2cbw003.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>

/* FIXME: rename to struct w2cbw */
struct w2cbw {
	struct regulator	*reg;
	int	set;	// gpio is "set"
	/* there is no chip behind, but we need an address
	 * of this wrapped struct to handle the set_value call
	 */
#ifdef CONFIG_GPIOLIB
	struct gpio_chip	gpio;
	const char	*gpio_name[1];
#endif
};

/* FIXME: rename function to w2cbw_$$$ */
static int w2cbw_get_value(struct gpio_chip *gc,
							unsigned offset)
{
	struct w2cbw *greg = container_of(gc, struct w2cbw, gpio);
	// we could ask the regulator so that multiple consumers are OR'ed
	return greg->set;
}

static void w2cbw_set_value(struct gpio_chip *gc,
			       unsigned offset, int val)
{
	struct w2cbw *greg = container_of(gc, struct w2cbw, gpio);
	if (val) {
		if (!greg->set && greg->reg)
			if (regulator_enable(greg->reg) == 0)
				greg->set = 1;
	} else {
		if (greg->set && greg->reg)
			if (regulator_disable(greg->reg) == 0)
				greg->set = 0;
	}
}

static int w2cbw_direction_output(struct gpio_chip *gc,
				     unsigned offset, int val)
{
	/* output => reg_enable, input => reg_disable */
	w2cbw_set_value(gc, offset, val);
	return 0;
}


static int w2cbw_probe(struct platform_device *pdev)
{
	struct w2cbw_data *pdata = dev_get_platdata(&pdev->dev);
	struct w2cbw *greg;
	int err;
	printk("w2cbw_probe()\n");
#ifdef CONFIG_OF
	if (pdev->dev.of_node) {
		struct device *dev = &pdev->dev;
		u32 uV;
		pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		greg = kzalloc(sizeof(*greg), GFP_KERNEL);
		if (greg == NULL)
			return -ENOMEM;

		greg->gpio.ngpio = 0;

		greg->reg = regulator_get(&pdev->dev, "vdd");
		printk("w2cbw_probe() reg=%p\n", greg->reg);

		if (IS_ERR(greg->reg)) {
			err = PTR_ERR(greg->reg);
			greg->reg = NULL;
			goto out2;
		}
		if (of_property_read_u32(pdev->dev.of_node, "microvolt", &uV) == 0)
			regulator_set_voltage(greg->reg, uV, uV);
	/* FIXME: use the same code as with board file */
		greg->set = 0;

		greg->gpio_name[0] = "gpio-w2cbw003-enable";	/* label of controlling GPIO */

		greg->gpio.label = "w2cbw003";
		greg->gpio.names = greg->gpio_name;
		greg->gpio.ngpio = 1;
		greg->gpio.base = -1;
		greg->gpio.owner = THIS_MODULE;
		greg->gpio.direction_output = w2cbw_direction_output;
		greg->gpio.get = w2cbw_get_value;
		greg->gpio.set = w2cbw_set_value;
		greg->gpio.can_sleep = 1;
        greg->gpio.of_node = of_node_get(pdev->dev.of_node);
		err = gpiochip_add(&greg->gpio);
		printk("w2cbw_probe() gpiochip_add()=%d\n", err);

	out2:
		if (err)
			regulator_put(greg->reg);
		else
			platform_set_drvdata(pdev, greg);

		goto out;
	}
#endif
	greg = kzalloc(sizeof(*greg), GFP_KERNEL);
	if (greg == NULL)
		return -ENOMEM;
	greg->reg = regulator_get(&pdev->dev, "vgpio");
	if (IS_ERR(greg->reg)) {
		err = PTR_ERR(greg->reg);
		greg->reg = NULL;
		goto out;
	}
	if (pdata->uV)
		regulator_set_voltage(greg->reg, pdata->uV, pdata->uV);
	greg->set = 0;

	greg->gpio_name[0] = "gpio-w2cbw003-enable";	/* label of controlling GPIO */

	greg->gpio.label = "w2cbw003";
	greg->gpio.names = greg->gpio_name;
	greg->gpio.ngpio = 1;
	greg->gpio.base = pdata->gpio_base;
	greg->gpio.owner = THIS_MODULE;
	greg->gpio.direction_output = w2cbw_direction_output;
	greg->gpio.set = w2cbw_set_value;
	greg->gpio.can_sleep = 1;
	err = gpiochip_add(&greg->gpio);
	if (err)
		regulator_put(greg->reg);
	else
		platform_set_drvdata(pdev, greg);
out:
	if (err)
		kfree(greg);
	return err;
}

static int w2cbw_remove(struct platform_device *pdev)
{
	struct w2cbw *greg = platform_get_drvdata(pdev);
	int ret;

	if (greg->reg) {
		regulator_put(greg->reg);
		greg->reg = NULL;
	}
	ret = gpiochip_remove(&greg->gpio);
	if (ret == 0)
		kfree(greg);
	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id w2cbw003_of_match[] = {
	{ .compatible = "wi2wi,w2cbw003" },
	{},
};
MODULE_DEVICE_TABLE(of, w2cbw003_of_match);
#endif

static struct platform_driver w2cbw_driver = {
	.driver.name	= "w2cbw003",
	.driver.owner	= THIS_MODULE,
	.driver.of_match_table = of_match_ptr(w2cbw003_of_match),
	.probe		= w2cbw_probe,
	.remove		= w2cbw_remove,
};

static int __init w2cbw_init(void)
{
	printk("gpio_w2cbw_init()\n");
	return platform_driver_register(&w2cbw_driver);
}
module_init(w2cbw_init);

static void __exit w2cbw_exit(void)
{
	platform_driver_unregister(&w2cbw_driver);
}
module_exit(w2cbw_exit);

MODULE_ALIAS("w2cbw003");	/* should we have a simplified version of this as a separate driver specific for the w2cbw003? */

MODULE_AUTHOR("NeilBrown <neilb@suse.de>");
MODULE_DESCRIPTION("Virtual GPIO driver controlling a regulator");
MODULE_LICENSE("GPL v2");
