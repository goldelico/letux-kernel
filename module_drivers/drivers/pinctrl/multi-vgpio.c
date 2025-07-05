#include <linux/module.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/platform_device.h>

/*
 * multiple input lineout to signle output
 * dts sample
 charger_en:vgpa {
 compatible = "ingenic,multi-vgpio";
 status = "okay";
 gpio-controller;
 #gpio-cells = <2>;
 ingenic,real-gpio = <&gpd 2 GPIO_ACTIVE_HIGH INGENIC_GPIO_NOBIAS>;
 ingenic,count = <2>;
 ingenic,name = "vgpb";
 };

 *
 *  in0 \
 *       lineout -- out
 *  in1 /
 *
 *  gpio out LOW:
    Out: GPIO_ACTIVE_HIGH  In: GPIO_ACTIVE_LOW
	Out: GPIO_ACTIVE_LOW   In: GPIO_ACTIVE_HIGH
 *  gpio out HIGH:
    Out: GPIO_ACTIVE_HIGH  In: GPIO_ACTIVE_HIGH
	Out: GPIO_ACTIVE_LOW   In: GPIO_ACTIVE_LOW
 */


struct multi_vgpio
{
	struct gpio_chip gc;
	unsigned int value;
	int in_count;
	const char *name;
	struct gpio_desc *gpiod_real;
};

static int multi_vgpio_get_direction(struct gpio_chip *gc, unsigned offset)
{
	struct multi_vgpio *priv = container_of(gc,struct multi_vgpio,gc);
	return gpiod_get_direction(priv->gpiod_real);
}

static int multi_vgpio_dir_in(struct gpio_chip *gc, unsigned int gpio)
{
	struct multi_vgpio *priv = container_of(gc,struct multi_vgpio,gc);
	return gpiod_get_value(priv->gpiod_real);
}

static int multi_vgpio_dir_out(struct gpio_chip *gc, unsigned int gpio, int val)
{
	struct multi_vgpio *priv = container_of(gc,struct multi_vgpio,gc);
	return gpiod_direction_output(priv->gpiod_real,val);
}

static int multi_vgpio_request(struct gpio_chip *gc, unsigned int pins)
{
	struct multi_vgpio *priv = container_of(gc,struct multi_vgpio,gc);
	if(pins < priv->in_count)
		return 0;
	return -EINVAL;
}

static void multi_vgpio_set(struct gpio_chip *gc,unsigned pin, int value)
{
	struct multi_vgpio *priv = container_of(gc,struct multi_vgpio,gc);
	return gpiod_set_value(priv->gpiod_real,value);
}

static int multi_vgpio_get(struct gpio_chip *gc, unsigned pin)
{
	struct multi_vgpio *priv = container_of(gc,struct multi_vgpio,gc);
	return gpiod_get_value(priv->gpiod_real);
}

static int ingenic_multi_vgpio_probe(struct platform_device *pdev)
{
	struct multi_vgpio *priv;
	struct device_node *np = pdev->dev.of_node;
	int rc;

	priv = devm_kzalloc(&pdev->dev,sizeof(*priv),GFP_KERNEL);
	if(!priv)
		return -ENOMEM;

	rc = of_property_read_string_index(np,"ingenic,name",0,&priv->name);
	if(rc != 0)
	{
		priv->name = "vgpio";
	}

	rc = of_property_read_u32(np, "ingenic,count", &priv->in_count);
	if(rc != 0)
	{
		priv->in_count = 2;
	}

	priv->gpiod_real = devm_gpiod_get_optional(&pdev->dev,"ingenic,real", GPIOD_ASIS);
	if(IS_ERR_OR_NULL(priv->gpiod_real))
	{
		dev_err(&pdev->dev,"real gpio request failed!\n");
		return -ENODEV;
	}

	priv->gc.dev = &pdev->dev;
	priv->gc.label = priv->name;
	priv->gc.base = -1;
	priv->gc.request = multi_vgpio_request;

	priv->gc.direction_input = multi_vgpio_dir_in;
	priv->gc.direction_output = multi_vgpio_dir_out;
	priv->gc.set = multi_vgpio_set;
	priv->gc.get = multi_vgpio_get;

	priv->gc.get_direction = multi_vgpio_get_direction;
	priv->gc.ngpio = priv->in_count;
	priv->gc.owner = THIS_MODULE;
	priv->gc.can_sleep = 0;
	platform_set_drvdata(pdev, priv);
	printk("multi-vgpio register ok!\n");
	return gpiochip_add(&priv->gc);
}

static int ingenic_multi_vgpio_remove(struct platform_device *pdev)
{
	struct multi_vgpio *priv = platform_get_drvdata(pdev);
	gpiochip_remove(&priv->gc);
	return 0;
}

static const struct of_device_id mult_vgpio_ids[] = {
	{.compatible	= "ingenic,multi-vgpio"},
	{},
};

static struct platform_driver multi_vgpio_driver = {
	.probe = ingenic_multi_vgpio_probe,
	.remove = ingenic_multi_vgpio_remove,
	.driver = {
		.name = "ingenic-multi-vgpio",
		.owner = THIS_MODULE,
		.of_match_table = mult_vgpio_ids,
	},
};


static int __init multi_vgpio_drv_init(void)
{
	return platform_driver_register(&multi_vgpio_driver);
}

static void __exit multi_vgpio_drv_exit(void)
{
	platform_driver_unregister(&multi_vgpio_driver);
}

postcore_initcall(multi_vgpio_drv_init);
module_exit(multi_vgpio_drv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Stephen <stephen@ingenic.com>");
MODULE_DESCRIPTION("VIRTUAL GPIO driver");
