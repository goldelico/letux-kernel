/* Philips PCF50606 Input Driver
 *
 * (C) 2006-2008 by Openmoko, Inc.
 * Author: Balaji Rao <balajirrao@openmoko.org>
 * All rights reserved.
 *
 * Broken down from monstrous PCF50606 driver mainly by
 * Harald Welte, Matt Hsu, Andy Green and Werner Almesberger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/input.h>

#include <linux/mfd/pcf50606/core.h>

#define PCF50606_OOCS_ONKEY		0x01
#define PCF50606_OOCS_EXTON	 	0x02

#define PCF50606_OOCC2_ONKEYDB_NONE	0x00
#define PCF50606_OOCC2_ONKEYDB_14ms	0x01
#define PCF50606_OOCC2_ONKEYDB_62ms	0x02
#define PCF50606_OOCC2_ONKEYDB_500ms	0x03
#define PCF50606_OOCC2_EXTONDB_NONE	0x00
#define PCF50606_OOCC2_EXTONDB_14ms	0x04
#define PCF50606_OOCC2_EXTONDB_62ms	0x08
#define PCF50606_OOCC2_EXTONDB_500ms	0x0c

#define PCF50606_REG_OOCS 	0x01

struct pcf50606_input {
	struct pcf50606 *pcf;
	struct input_dev *input_dev;
};

static void
pcf50606_input_irq(int irq, void *data)
{
	struct pcf50606_input *input;
	int onkey_released;

	input = data;
	onkey_released = pcf50606_reg_read(input->pcf, PCF50606_REG_OOCS) &
						PCF50606_OOCS_ONKEY;

	if (irq == PCF50606_IRQ_ONKEYF && !onkey_released)
		input_report_key(input->input_dev, KEY_POWER, 1);
	else if (irq == PCF50606_IRQ_ONKEYR && onkey_released)
		input_report_key(input->input_dev, KEY_POWER, 0);

	input_sync(input->input_dev);
}

static int __devinit pcf50606_input_probe(struct platform_device *pdev)
{
	struct pcf50606_input *input;
	struct pcf50606_subdev_pdata *pdata = pdev->dev.platform_data;
	struct input_dev *input_dev;
	int ret;


	input = kzalloc(sizeof(*input), GFP_KERNEL);
	if (!input)
		return -ENOMEM;

	input_dev = input_allocate_device();
	if (!input_dev) {
		kfree(input);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, input);
	input->pcf = pdata->pcf;
	input->input_dev = input_dev;

	input_dev->name = "PCF50606 PMU events";
	input_dev->id.bustype = BUS_I2C;
	input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_PWR);
	set_bit(KEY_POWER, input_dev->keybit);

	ret = input_register_device(input_dev);
	if (ret) {
		input_free_device(input_dev);
		kfree(input);
		return ret;
	}
	pcf50606_register_irq(pdata->pcf, PCF50606_IRQ_ONKEYR,
				pcf50606_input_irq, input);
	pcf50606_register_irq(pdata->pcf, PCF50606_IRQ_ONKEYF,
				pcf50606_input_irq, input);

	return 0;
}

static int __devexit pcf50606_input_remove(struct platform_device *pdev)
{
	struct pcf50606_input *input  = platform_get_drvdata(pdev);

	input_unregister_device(input->input_dev);
	pcf50606_free_irq(input->pcf, PCF50606_IRQ_ONKEYR);
	pcf50606_free_irq(input->pcf, PCF50606_IRQ_ONKEYF);

	kfree(input);

	return 0;
}

static struct platform_driver pcf50606_input_driver = {
	.driver = {
		.name = "pcf50606-input",
	},
	.probe = pcf50606_input_probe,
	.remove = __devexit_p(pcf50606_input_remove),
};

static int __init pcf50606_input_init(void)
{
	return platform_driver_register(&pcf50606_input_driver);
}
module_init(pcf50606_input_init);

static void __exit pcf50606_input_exit(void)
{
	platform_driver_unregister(&pcf50606_input_driver);
}
module_exit(pcf50606_input_exit);

MODULE_AUTHOR("Balaji Rao <balajirrao@openmoko.org>");
MODULE_DESCRIPTION("PCF50606 input driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pcf50606-input");
