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
	/* there is no chip behind, but we need an address
	 * of this wrapped struct to handle the set_value call
	 */
	struct gpio_chip	virtual_chip;
	int			set;
};

static void gpio_reg_set_value(struct gpio_chip *gc,
			       unsigned offset, int val)
{
	struct gpio_reg *greg = container_of(gc, struct gpio_reg, virtual_chip);
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
	/* output => reg_enable, input => reg_disable */
	gpio_reg_set_value(gc, offset, val);
	return 0;
}

#ifdef CONFIG_OF
static struct gpio_reg_data *gpio_reg_of_pdata(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct gpio_reg_data *pdata;
	u32 uV;
	if (!np)
		return NULL;
	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;
	pdata->gpio = -1;
	if (of_property_read_u32(np, "microvolt", &uV) == 0)
		pdata->uV = uV;
	return pdata;
}
#else
static inline struct gpio_reg_data *gpio_reg_of_pdata(struct device *dev)
{
	return NULL;
}
#endif

static int gpio_reg_probe(struct platform_device *pdev)
{
	struct gpio_reg_data *pdata = pdev->dev.platform_data;
	struct gpio_reg *greg;
	int err;

	if (!pdata)
		pdata = gpio_reg_of_pdata(&pdev->dev);
	if (!pdata)
		return -ENODEV;
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
	greg->virtual_chip.label = "gpio-regulator";
	greg->virtual_chip.ngpio = 1;
	greg->virtual_chip.base = pdata->gpio;
	greg->virtual_chip.owner = THIS_MODULE;
	greg->virtual_chip.direction_output = gpio_reg_direction_output;
	greg->virtual_chip.set = gpio_reg_set_value;
	greg->virtual_chip.can_sleep = 1;
#ifdef CONFIG_OF_GPIO
	greg->virtual_chip.of_node = of_node_get(pdev->dev.of_node);
#endif
	err = gpiochip_add(&greg->virtual_chip);
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
	ret = gpiochip_remove(&greg->virtual_chip);
	if (ret == 0)
		kfree(greg);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id gpio_reg_match[] = {
	{.compatible = "gpio_regulator"},
	{}
};
MODULE_DEVICE_TABLE(of, gpio_reg_match);
#endif

static struct platform_driver gpio_reg_driver = {
	.driver.name	= "regulator-gpio",
	.driver.owner	= THIS_MODULE,
	.driver.of_match_table = of_match_ptr(gpio_reg_match),
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
