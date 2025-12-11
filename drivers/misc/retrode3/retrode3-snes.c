/* SPDX-License-Identifier: GPL-2.0-only */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "retrode3.h"

struct retrode3_snes_priv {
	struct retrode3_bus_device *mdev;
	struct retrode3_bus_controller *controller;
};

static int retrode3_snes_probe(struct retrode3_bus_device *dev)
{
	struct retrode3_snes_priv *p;
	struct retrode3_bus_controller *controller = dev->controller;;
	int ret;
	u8 buf[4];

	p = devm_kzalloc(&dev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->controller = controller;
	dev_set_drvdata(&dev->dev, p);

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

	return 0;
}

static int retrode3_snes_remove(struct retrode3_bus_device *dev)
{
	/* cleanup */
	return 0;
}

static const struct of_device_id retrode3_snes_of_match[] = {
	{ .compatible = "dragonbox,retrode3-snes" },
	{}
};
MODULE_DEVICE_TABLE(of, retrode3_snes_of_match);

static struct retrode3_bus_client_driver retrode3_snes_driver = {
	.probe = retrode3_snes_probe,
	.remove = retrode3_snes_remove,
	.driver = {
		.name = "retrode3-snes",
		.of_match_table = retrode3_snes_of_match,
	},
};

static int __init retrode3_snes_init(void)
{
	return retrode3_bus_client_driver_register(&retrode3_snes_driver);
}
static void __exit retrode3_snes_exit(void)
{
	retrode3_bus_client_driver_unregister(&retrode3_snes_driver);
}
module_init(retrode3_snes_init);
module_exit(retrode3_snes_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("retrode3 snes driver");
