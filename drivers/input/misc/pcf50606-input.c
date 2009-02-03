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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 060, Boston,
 * MA 02111-1307 USA
 */

#include <linux/input.h>

#include <linux/mfd/pcf50606/core.h>
#include <linux/mfd/pcf50606/input.h>

static void
pcf50606_input_irq(struct pcf50606 *pcf, int irq, void *data)
{
	struct input_dev *input_dev = pcf->input.input_dev;
	int onkey_released;

	/* We report only one event depending on if the key status */
	onkey_released = pcf50606_reg_read(pcf, PCF50606_REG_OOCS) &
					PCF50606_OOCS_ONKEY;

	if (irq == PCF50606_IRQ_ONKEYF && !onkey_released)
		input_report_key(input_dev, KEY_POWER, 1);
	else if (irq == PCF50606_IRQ_ONKEYR && onkey_released)
		input_report_key(input_dev, KEY_POWER, 0);

	input_sync(input_dev);
}

int __init pcf50606_input_probe(struct platform_device *pdev)
{
	struct pcf50606 *pcf;
	struct input_dev *input_dev;
	int ret;

	pcf = platform_get_drvdata(pdev);

	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENODEV;

	input_dev->name = "PCF50606 PMU events";
	input_dev->id.bustype = BUS_I2C;

	input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_PWR);
	set_bit(KEY_POWER, input_dev->keybit);
	set_bit(KEY_POWER2, input_dev->keybit);

	ret = input_register_device(input_dev);
	if (ret)
		goto out;

	pcf->input.input_dev = input_dev;

	/* Currently we care only about ONKEY and USBINS/USBREM
	 *
	 * USBINS/USBREM are told to us by mbc driver as we can't setup
	 * two handlers for an IRQ
	 */
	pcf->irq_handler[PCF50606_IRQ_ONKEYR].handler = pcf50606_input_irq;

	pcf->irq_handler[PCF50606_IRQ_ONKEYF].handler = pcf50606_input_irq;

	return 0;

out:
	input_free_device(input_dev);
	return ret;
}

static int __devexit pcf50606_input_remove(struct platform_device *pdev)
{
	struct pcf50606 *pcf;

	pcf = platform_get_drvdata(pdev);
	input_unregister_device(pcf->input.input_dev);

	return 0;
}

struct platform_driver pcf50606_input_driver = {
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
