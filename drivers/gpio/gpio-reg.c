/*
 * gpio-reg: create plumbing between a virtual GPIO and a regulator.
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
#include <linux/gpio-reg.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>

#define MAX_REGULATORS 1	/* FIXME: add code to really handle multiple regulators */

struct gpio_reg {
	struct regulator	*reg[MAX_REGULATORS];
	/* there is no chip behind, but we need an address
	 * of this wrapped struct to handle the set_value call
	 */
	struct gpio_chip	virtual_chip;
	char			set[MAX_REGULATORS];
};

static void gpio_reg_set_value(struct gpio_chip *gc,
			       unsigned offset, int val)
{
	struct gpio_reg *greg = container_of(gc, struct gpio_reg, virtual_chip);
	if (val) {
		if (!greg->set[offset] && greg->reg[offset])
			if (regulator_enable(greg->reg[offset]) == 0)
				greg->set[offset] = 1;
	} else {
		if (greg->set[offset] && greg->reg[offset])
			if (regulator_disable(greg->reg[offset]) == 0)
				greg->set[offset] = 0;
	}
}

static int gpio_reg_direction_output(struct gpio_chip *gc,
				     unsigned offset, int val)
{
	/* output => reg_enable, input => reg_disable */
	gpio_reg_set_value(gc, offset, val);
	return 0;
}


static int gpio_reg_probe(struct platform_device *pdev)
{
	struct gpio_reg_data *pdata = dev_get_platdata(&pdev->dev);
	struct gpio_reg *greg;
	int err;
	printk("gpio_reg_probe()\n");
#ifdef CONFIG_OF
	if (pdev->dev.of_node) {
		struct device *dev = &pdev->dev;
		enum of_gpio_flags flags;
		int i;
		pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		greg = kzalloc(sizeof(*greg), GFP_KERNEL);
		if (greg == NULL)
			return -ENOMEM;

		greg->virtual_chip.ngpio = 0;

		// FIXME: loop over multiple regulators and end if no more are found

		for (i=0; i < MAX_REGULATORS; i++) {
			u32 uV;
			greg->reg[i] = regulator_get(&pdev->dev, "vgpio");
			printk("gpio_reg_probe() reg=%p\n", greg->reg[i]);

			if (IS_ERR(greg->reg[i])) {
				err = PTR_ERR(greg->reg[i]);
				greg->reg[i] = NULL;
				goto out2;
			}
			if (of_property_read_u32(pdev->dev.of_node, "microvolt", &uV) == 0)
				regulator_set_voltage(greg->reg[0], uV, uV);
			greg->set[i] = 0;
		}
		greg->virtual_chip.ngpio = i;
		greg->virtual_chip.label = "gpio-regulator";
		greg->virtual_chip.base = pdata->gpio;
		greg->virtual_chip.owner = THIS_MODULE;
		greg->virtual_chip.direction_output = gpio_reg_direction_output;
		greg->virtual_chip.set = gpio_reg_set_value;
		greg->virtual_chip.can_sleep = 1;
        greg->virtual_chip.of_node = of_node_get(pdev->dev.of_node);
		err = gpiochip_add(&greg->virtual_chip);
		printk("gpio_reg_probe() gpiochip_add()=%d\n", err);

	out2:
		if (err) {
			while (i > 0)
				regulator_put(greg->reg[--i]);
		}
		else
			platform_set_drvdata(pdev, greg);

		goto out;
	}
#endif
	greg = kzalloc(sizeof(*greg), GFP_KERNEL);
	if (greg == NULL)
		return -ENOMEM;
	greg->reg[0] = regulator_get(&pdev->dev, "vgpio");
	if (IS_ERR(greg->reg[0])) {
		err = PTR_ERR(greg->reg[0]);
		greg->reg[0] = NULL;
		goto out;
	}
	if (pdata->uV)
		regulator_set_voltage(greg->reg[0], pdata->uV, pdata->uV);
	greg->set[0] = 0;
	greg->virtual_chip.label = "gpio-regulator";
	greg->virtual_chip.ngpio = 1;
	greg->virtual_chip.base = pdata->gpio;
	greg->virtual_chip.owner = THIS_MODULE;
	greg->virtual_chip.direction_output = gpio_reg_direction_output;
	greg->virtual_chip.set = gpio_reg_set_value;
	greg->virtual_chip.can_sleep = 1;
	err = gpiochip_add(&greg->virtual_chip);
	if (err)
		regulator_put(greg->reg[0]);
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

	if (greg->reg[0]) {
		regulator_put(greg->reg[0]);
		greg->reg[0] = NULL;
	}
	ret = gpiochip_remove(&greg->virtual_chip);
	if (ret == 0)
		kfree(greg);
	return 0;
}

// FIXME: rename this to regulator-controlling-virtual-gpio

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

#if defined(CONFIG_OF)
static const struct of_device_id gpio_reg_of_match[] = {
	{ .compatible = "wi2wi,w2cbw003" },
	{},
};
MODULE_DEVICE_TABLE(of, gpio_reg_of_match);
#endif

MODULE_ALIAS("w2cbw003");	/* should we have a simplified version of this as a separate driver specific for the w2cbw003? */

MODULE_AUTHOR("NeilBrown <neilb@suse.de>");
MODULE_DESCRIPTION("Virtual GPIO driver controlling a regulator");
MODULE_LICENSE("GPL v2");
