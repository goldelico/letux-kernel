#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/gpio.h>

void pcf50633_gpio_set(struct pcf50633 *pcf, int gpio, int on)
{
	u8 reg = gpio - PCF50633_GPIO1 + PCF50633_REG_GPIO1CFG;

	if (on)
		pcf50633_reg_set_bit_mask(pcf, reg, 0x0f, 0x07);
	else
		pcf50633_reg_set_bit_mask(pcf, reg, 0x0f, 0x00);
}
EXPORT_SYMBOL_GPL(pcf50633_gpio_set);

int pcf50633_gpio_get(struct pcf50633 *pcf, int gpio)
{
	u8 reg = gpio - PCF50633_GPIO1 + PCF50633_REG_GPIO1CFG;
	u8 val = pcf50633_reg_read(pcf, reg) & 0x0f;

	if (val == PCF50633_GPOCFG_GPOSEL_1 ||
	    val == (PCF50633_GPOCFG_GPOSEL_0|PCF50633_GPOCFG_GPOSEL_INVERSE))
		return 1;

	return 0;
}
EXPORT_SYMBOL_GPL(pcf50633_gpio_get);

int __init pcf50633_gpio_probe(struct platform_device *pdev)
{
	return 0;
}

static int __devexit pcf50633_gpio_remove(struct platform_device *pdev)
{
	return 0;
}

struct platform_driver pcf50633_gpio_driver = {
	.driver = {
		.name = "pcf50633-gpio",
	},
	.probe = pcf50633_gpio_probe,
	.remove = __devexit_p(pcf50633_gpio_remove),
};

static int __init pcf50633_gpio_init(void)
{
		return platform_driver_register(&pcf50633_gpio_driver);
}
module_init(pcf50633_gpio_init);

static void __exit pcf50633_gpio_exit(void)
{
		platform_driver_unregister(&pcf50633_gpio_driver);
}
module_exit(pcf50633_gpio_exit);

MODULE_AUTHOR("Balaji Rao <balajirrao@openmoko.org>");
MODULE_DESCRIPTION("PCF50633 gpio driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pcf50633-gpio");

