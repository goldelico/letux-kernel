/*
 * gpio-reg: create plumbing between a virtual GPIO and a regulator.
 *
 * This module provide a single output GPIO and uses a single regulator.
 * When the GPIO is driven high, the regulator is enabled.
 * When the GPIO is driven low, the regulator is disabled.
 * This allows a serial port to be given the GPIO as a DTR signal,
 * and whenever the serial port is opened, the regulator is enabled.
 *
 * The GPIO number is passed in the platform data.
 * The regulator consumer name is "vgpio"
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/gpio-reg.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>

struct gpio_reg {
	struct regulator	*reg;
	struct gpio_chip	gpio;
	int			set;
};

static void gpio_reg_set_value(struct gpio_chip *gc,
			       unsigned offset, int val)
{
	struct gpio_reg *greg = container_of(gc, struct gpio_reg, gpio);
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

static int gpio_reg_direction_output(struct gpio_chip *gc,
				     unsigned offset, int val)
{
	gpio_reg_set_value(gc, offset, val);
	return 0;
}


static int gpio_reg_probe(struct platform_device *pdev)
{
	struct gpio_reg_data *pdata = pdev->dev.platform_data;
	struct gpio_reg *greg;
	int err;

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
	greg->gpio.label = "gpio-regulator";
	greg->gpio.ngpio = 1;
	greg->gpio.base = pdata->gpio;
	greg->gpio.owner = THIS_MODULE;
	greg->gpio.direction_output = gpio_reg_direction_output;
	greg->gpio.set = gpio_reg_set_value;
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

static int gpio_reg_remove(struct platform_device *pdev)
{
	struct gpio_reg *greg = platform_get_drvdata(pdev);
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

static struct platform_driver gpio_reg_driver = {
	.driver.name	= "regulator-gpio",
	.driver.owner	= THIS_MODULE,
	.probe		= gpio_reg_probe,
	.remove		= gpio_reg_remove,
};

static int __init gpio_reg_init(void)
{
	return platform_driver_register(&gpio_reg_driver);
}
module_init(gpio_reg_init);

static void __exit gpio_reg_exit(void)
{
	platform_driver_unregister(&gpio_reg_driver);
}
module_exit(gpio_reg_exit);

MODULE_AUTHOR("NeilBrown <neilb@suse.de>");
MODULE_DESCRIPTION("Regulator based virtual GPIO driver");
MODULE_LICENSE("GPL v2");
