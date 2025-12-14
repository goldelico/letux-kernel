/* SPDX-License-Identifier: GPL-2.0-only */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "retrode3.h"

struct retrode3_controller_priv {
	struct retrode3_slot *slot;
	struct retrode3_bus_controller *controller;
};

static int retrode3_controller_probe(struct retrode3_slot *dev)
{
	struct retrode3_controller_priv *p;
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
	ret = retrode3_bus_select_slot(controller, dev);
	if (ret) {
		dev_err(&dev->dev, "failed to lock retrode3: %d\n", ret);
		return ret;
	}

	ret = retrode3_bus_set_address(controller, 0x1000);
	if (ret) {
		dev_err(&dev->dev, "set_addr failed: %d\n", ret);
		retrode3_bus_select_slot(controller, NULL);
		return ret;
	}

#if FIXME
	ret = retrode3_bus_xfer(controller, retrode3_bus_DIR_READ, buf, sizeof(buf));
	if (ret < 0) {
		dev_err(&dev->dev, "xfer read failed: %d\n", ret);
		retrode3_bus_select_slot(controller, NULL);
		return ret;
	}

	dev_info(&dev->dev, "read %d bytes from retrode3 addr 0x1000\n", ret);
#endif

	retrode3_bus_select_slot(controller, NULL);
	printk("%s %d\n", __func__, __LINE__);
	/* end sample code during probe */

	return 0;
}

static int retrode3_controller_remove(struct retrode3_slot *dev)
{
	/* cleanup */
	return 0;
}

static const struct of_device_id retrode3_controller_of_match[] = {
	{ .compatible = "dragonbox,retrode3-controller" },
	{}
};
MODULE_DEVICE_TABLE(of, retrode3_controller_of_match);

static struct retrode3_slot_driver retrode3_controller_driver = {
	.probe = retrode3_controller_probe,
	.remove = retrode3_controller_remove,
	.driver = {
		.name = "retrode3-controller",
		.of_match_table = retrode3_controller_of_match,
	},
};

static int __init retrode3_controller_init(void)
{
	printk("%s %d\n", __func__, __LINE__);
	return retrode3_slot_driver_register(&retrode3_controller_driver);
}
static void __exit retrode3_controller_exit(void)
{
	retrode3_slot_driver_unregister(&retrode3_controller_driver);
}
module_init(retrode3_controller_init);
module_exit(retrode3_controller_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("retrode3 controller driver");
