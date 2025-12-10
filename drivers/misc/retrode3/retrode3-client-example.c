/* SPDX-License-Identifier: GPL-2.0-only */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "retrode3.h"

struct example_priv {
	struct retrode3_bus_device *mdev;
	struct retrode3_bus_controller *controller;
};

static int example_probe(struct retrode3_bus_device *dev)
{
	struct example_priv *p;
	struct retrode3_bus_controller *controller = dev->controller;;
	int ret;
	u8 buf[4];

	printk("%s %d\n", __func__, __LINE__);
	p = devm_kzalloc(&dev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->controller = controller;
	dev_set_drvdata(&dev->dev, p);

	/* sample code during probe */
	printk("%s %d\n", __func__, __LINE__);
	/* Lock the bus, set address, read 4 bytes */
	ret = retrode3_bus_lock_bus(controller);
	if (ret) {
		dev_err(&dev->dev, "failed to lock retrode3: %d\n", ret);
		return ret;
	}

	ret = retrode3_bus_set_address(controller, 0x1000);
	if (ret) {
		dev_err(&dev->dev, "set_addr failed: %d\n", ret);
		retrode3_bus_unlock_bus(controller);
		return ret;
	}

	ret = retrode3_bus_xfer(controller, retrode3_bus_DIR_READ, buf, sizeof(buf));
	if (ret < 0) {
		dev_err(&dev->dev, "xfer read failed: %d\n", ret);
		retrode3_bus_unlock_bus(controller);
		return ret;
	}

	dev_info(&dev->dev, "read %d bytes from retrode3 addr 0x1000\n", ret);
	retrode3_bus_unlock_bus(controller);
	printk("%s %d\n", __func__, __LINE__);
	/* end sample code during probe */

	return 0;
}

static int example_remove(struct retrode3_bus_device *dev)
{
	/* cleanup */
	return 0;
}

static const struct of_device_id example_of_match[] = {
	{ .compatible = "dragonbox,retrode3-client-example" },
	{}
};
MODULE_DEVICE_TABLE(of, example_of_match);

static struct retrode3_bus_client_driver example_driver = {
	.probe = example_probe,
	.remove = example_remove,
	.driver = {
		.name = "retrode3-client-example",
		.of_match_table = example_of_match,
	},
};

static int __init example_init(void)
{
	printk("%s %d\n", __func__, __LINE__);
	return retrode3_bus_client_driver_register(&example_driver);
}
static void __exit example_exit(void)
{
	retrode3_bus_client_driver_unregister(&example_driver);
}
module_init(example_init);
module_exit(example_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Example retrode3 client");
