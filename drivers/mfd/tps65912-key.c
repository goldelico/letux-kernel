/*
 * Copyright (C) 2013 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/mfd/tps65912.h>
#ifdef CONFIG_WAKEUP_SOURCE_NOTIFY
#include <linux/wakeup_source_notify.h>
#endif
#ifdef CONFIG_POWER_KEY_OVERRIDE
#include <linux/charger_notify.h>
#include <linux/display_notify.h>
#endif

struct tps65912_key_data {
	struct input_dev *input_dev;
	struct tps65912 *tps65912;
};

#ifdef CONFIG_POWER_KEY_OVERRIDE
static int charger_notify(struct notifier_block *self,
		unsigned long action, void *dev)
{
	struct tps65912 *tps65912 =
		container_of(self, struct tps65912, charger_nb);
	tps65912->dockstatus = (action == EVENT_DOCKON);
	pr_info("%s: dockstatus is %d\n", __func__, tps65912->dockstatus);
	return NOTIFY_OK;
}

static int display_notify(struct notifier_block *self,
		unsigned long action, void *dev)
{
	struct tps65912 *tps65912 =
		container_of(self, struct tps65912, display_nb);

	tps65912->displaystatus = (action == DISPLAY_EVENT_DISPLAYON);
	pr_info("%s: displaystatus is %d\n", __func__, tps65912->displaystatus);
	return NOTIFY_OK;
}
#endif

static int tps65912_key_probe(struct platform_device *pdev)
{
	int err;
	struct tps65912_key_data *key;

	pr_info("tps65912_key_probe begin\n");

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "no platform_data\n");
		return -EINVAL;
	}

	key = kzalloc(sizeof(*key), GFP_KERNEL);
	if (!key)
		return -ENOMEM;

	key->tps65912 = pdev->dev.platform_data;

	key->input_dev = input_allocate_device();
	if (key->input_dev == NULL) {
		dev_err(&pdev->dev, "can't allocate input device\n");
		err = -ENOMEM;
		goto err0;
	}

	set_bit(EV_KEY, key->input_dev->evbit);
	set_bit(key->tps65912->powerkey_code, key->input_dev->keybit);
#ifdef CONFIG_POWER_KEY_OVERRIDE
	set_bit(KEY_SLEEP, key->input_dev->keybit);
#endif

	key->input_dev->name = "tps65912-key";

	err = input_register_device(key->input_dev);
	if (err < 0) {
		dev_err(&pdev->dev, "could not register input device.\n");
		goto err1;
	}

	platform_set_drvdata(pdev, key);
	tps65912_set_keydata(key->tps65912, key);

#ifdef CONFIG_POWER_KEY_OVERRIDE
	key->tps65912->charger_nb.notifier_call = charger_notify;
	charger_register_notify(&key->tps65912->charger_nb);
	key->tps65912->dockstatus = false;

	key->tps65912->display_nb.notifier_call = display_notify;
	display_register_notify(&key->tps65912->display_nb);
	key->tps65912->displaystatus = true;
	key->tps65912->lastkeyevent = -1;
#endif

	dev_info(&pdev->dev, "tps65912 key device probed\n");

	return 0;

err1:
	input_free_device(key->input_dev);
err0:
	kfree(key);
	return err;
}

static int __exit tps65912_key_remove(struct platform_device *pdev)
{
	struct tps65912_key_data *key = platform_get_drvdata(pdev);

	input_unregister_device(key->input_dev);
	input_free_device(key->input_dev);
#ifdef CONFIG_POWER_KEY_OVERRIDE
	charger_unregister_notify(&key->tps65912->charger_nb);
	display_unregister_notify(&key->tps65912->display_nb);
#endif
	kfree(key);

	return 0;
}

void tps65912_broadcast_key_event(struct tps65912 *tps65912,
			       unsigned int code, int value)
{
	struct tps65912_key_data *key = tps65912_get_keydata(tps65912);

	if (key && key->input_dev) {
		pr_info("tps65912: code %d val %d\n", code, value);
		input_report_key(key->input_dev, code, value);
		/*sync with input subsystem to solve the key cached problem*/
		input_sync(key->input_dev);
#ifdef CONFIG_WAKEUP_SOURCE_NOTIFY
		if (value == PWRKEY_PRESS)
			notify_display_wakeup(POWERKEY);
#endif /* CONFIG_WAKEUP_SOURCE_NOTIFY */
	}
}
EXPORT_SYMBOL(tps65912_broadcast_key_event);

static struct platform_driver tps65912_key_driver = {
	.probe		= tps65912_key_probe,
	.remove		= __exit_p(tps65912_key_remove),
	.driver		= {
		.name	= "tps65912_key",
		.owner	= THIS_MODULE,
	},
};

static int __init tps65912_key_init(void)
{
	return platform_driver_register(&tps65912_key_driver);
}
module_init(tps65912_key_init);

static void __exit tps65912_key_exit(void)
{
	platform_driver_unregister(&tps65912_key_driver);
}
module_exit(tps65912_key_exit);

MODULE_ALIAS("platform:tps65912_key");
MODULE_DESCRIPTION("TPS65912 KEY driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
