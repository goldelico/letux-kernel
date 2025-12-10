/* SPDX-License-Identifier: GPL-2.0-only */

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/io.h>

#include "retrode3.h"

struct retrode3_bus {
	void __iomem *base;
	struct retrode3_bus_controller controller;
	struct platform_device *pdev;
};

static int bus_lock(struct retrode3_bus_controller *controller)
{
	printk("%s %d\n", __func__, __LINE__);
	mutex_lock(&controller->lock);
	return 0;
}

static void bus_unlock(struct retrode3_bus_controller *controller)
{
	printk("%s %d\n", __func__, __LINE__);
	mutex_unlock(&controller->lock);
}

static int bus_set_addr(struct retrode3_bus_controller *controller, u32 addr)
{
	struct retrode3_bus *m = controller->priv;
	/* Write lower/upper 32bits */
	printk("%s %d:%08x\n", __func__, __LINE__, addr);
	return 0;
}

static int bus_set_select(struct retrode3_bus_controller *controller, unsigned int sel)
{
	printk("%s %d\n", __func__, __LINE__);
	/* Implementation depends on hardware; maybe write a sel register */
	/* For example, write to REG_CTRL high bits */
	return 0;
}

static int bus_xfer(struct retrode3_bus_controller *controller, u8 dir, void *buf, size_t len)
{
	struct retrode3_bus *m = controller->priv;
	size_t i;
	u8 *b = buf;

	printk("%s %d\n", __func__, __LINE__);

	for (i = 0; i < len; ++i) {
		if (dir == retrode3_bus_DIR_WRITE) {
			printk("%s %d\n", __func__, __LINE__);
		} else {
			/* strobe to read then read data */
			printk("%s %d\n", __func__, __LINE__);
		}
	}

	return len;
}

static const struct retrode3_bus_controller_ops bus_ops = {
	.lock = bus_lock,
	.unlock = bus_unlock,
	.set_addr = bus_set_addr,
	.set_select = bus_set_select,
	.xfer = bus_xfer,
};

static int retrode3_bus_probe(struct platform_device *pdev)
{
//	struct resource *res;
	struct retrode3_bus *m;
	int ret;

	printk("%s %d\n", __func__, __LINE__);

	m = devm_kzalloc(&pdev->dev, sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	/*
	 res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	 m->base = devm_ioremap_resource(&pdev->dev, res);
	 if (IS_ERR(m->base))
	 return PTR_ERR(m->base);
	 */

	m->pdev = pdev;
	m->controller.dev = &pdev->dev;
	m->controller.ops = &bus_ops;
	m->controller.priv = m;
	/* mutex in retrode3_bus_register_controller will be initialized */

	platform_set_drvdata(pdev, m);

	/* register controller with retrode3 core */
	ret = retrode3_bus_register_controller(&m->controller);
	if (ret) {
		dev_err(&pdev->dev, "failed to register retrode3 controller: %d\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "retrode3 bus controller registered\n");
	return 0;
}

static void retrode3_bus_remove(struct platform_device *pdev)
{
	struct retrode3_bus *m = platform_get_drvdata(pdev);

	retrode3_bus_unregister_controller(&m->controller);
}

static const struct of_device_id retrode3_bus_of_match[] = {
	{ .compatible = "dragonbox,retrode3-bus", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, retrode3_bus_of_match);

static struct platform_driver retrode3_bus_driver = {
	.probe = retrode3_bus_probe,
	.remove = retrode3_bus_remove,
	.driver = {
		.name = "retrode3-bus",
		.of_match_table = retrode3_bus_of_match,
	},
};
module_platform_driver(retrode3_bus_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("retrode3 bus controller");
