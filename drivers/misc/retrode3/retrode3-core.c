/* SPDX-License-Identifier: GPL-2.0-only */

#include <linux/device.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "retrode3.h"

static LIST_HEAD(ctlr_list);
static DEFINE_MUTEX(ctlr_list_lock);

static int retrode3_bus_match(struct device *dev, const struct device_driver *drv)
{
	/* default match uses of_match if dev->of_node/driver->of_match_table set */
	return of_driver_match_device(dev, drv);
}

/* Bus type for driver core */
static struct bus_type retrode3_bus_type = {
	.name = "retrode3",
	.match = retrode3_bus_match,
};

/* Called by controller driver to register itself */
int retrode3_bus_register_controller(struct retrode3_bus_controller *ctlr)
{
	struct device_node *child;
	struct retrode3_bus_device *mdev;
	struct device *dev = ctlr->dev;

	if (!ctlr || !ctlr->ops)
		return -EINVAL;

	mutex_init(&ctlr->lock);

	mutex_lock(&ctlr_list_lock);
	list_add_tail(&ctlr->dev->kobj.entry, &ctlr_list); /* just symbolic; adapt if keep real list */
	mutex_unlock(&ctlr_list_lock);

	/* create retrode3 devices for each DT child so client drivers get probed */
	if (dev->of_node) {
		for_each_available_child_of_node(dev->of_node, child) {
			mdev = retrode3_bus_device_create_from_of(ctlr, child);
			if (IS_ERR(mdev)) {
				dev_warn(dev, "failed to create retrode3 device for %pOF\n", child);
			}
			of_node_put(child);
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(retrode3_bus_register_controller);

void retrode3_bus_unregister_controller(struct retrode3_bus_controller *ctlr)
{
	/* For simplicity: we won't remove devices here; production code must cleanup */
	mutex_lock(&ctlr_list_lock);
	/* remove from list... */
	mutex_unlock(&ctlr_list_lock);
}
EXPORT_SYMBOL_GPL(retrode3_bus_unregister_controller);

/* device_create helper: allocates a retrode3_bus_device, sets parent to controller device */
struct retrode3_bus_device *retrode3_bus_device_create_from_of(struct retrode3_bus_controller *ctlr,
												 struct device_node *np)
{
	struct retrode3_bus_device *mdev;
	int ret;

	if (!ctlr || !np)
		return ERR_PTR(-EINVAL);

	mdev = kzalloc(sizeof(*mdev), GFP_KERNEL);
	if (!mdev)
		return ERR_PTR(-ENOMEM);

	mdev->ctlr = ctlr;
	/* read optional DT properties */
	of_property_read_u32(np, "retrode3,addr-width", &mdev->addr_width);
	of_property_read_u32(np, "retrode3,data-width", &mdev->data_width);

	device_initialize(&mdev->dev);
	mdev->dev.parent = ctlr->dev;
	mdev->dev.bus = &retrode3_bus_type;
	mdev->dev.of_node = of_node_get(np);
	/* set device name: use node name or a reg property */
	ret = dev_set_name(&mdev->dev, "%pOF", np);
	if (ret) {
		put_device(&mdev->dev);
		kfree(mdev);
		return ERR_PTR(ret);
	}

	ret = device_add(&mdev->dev);
	if (ret) {
		put_device(&mdev->dev);
		kfree(mdev);
		return ERR_PTR(ret);
	}

	dev_info(ctlr->dev, "created retrode3 device %s\n", dev_name(&mdev->dev));
	return mdev;
}
EXPORT_SYMBOL_GPL(retrode3_bus_device_create_from_of);

void retrode3_bus_device_remove(struct retrode3_bus_device *mdev)
{
	if (!mdev) return;
	device_del(&mdev->dev);
	put_device(&mdev->dev);
	kfree(mdev);
}
EXPORT_SYMBOL_GPL(retrode3_bus_device_remove);

/* Client API implementations */

struct retrode3_bus_controller *retrode3_bus_get_controller_by_phandle(struct device *dev,
														 const char *propname)
{
	struct device_node *np;
	struct device_node *ctlr_np;
	struct platform_device *pdev;
	struct retrode3_bus_controller *ctlr = NULL;

	if (!dev || !propname)
		return ERR_PTR(-EINVAL);

	np = dev->of_node;
	if (!np)
		return ERR_PTR(-ENODEV);

	ctlr_np = of_parse_phandle(np, propname, 0);
	if (!ctlr_np)
		return ERR_PTR(-ENODEV);

	pdev = of_find_device_by_node(ctlr_np);
	of_node_put(ctlr_np);
	if (!pdev)
		return ERR_PTR(-ENODEV);

	ctlr = platform_get_drvdata(pdev);
	if (!ctlr)
		return ERR_PTR(-ENODEV);

	return ctlr;
}
EXPORT_SYMBOL_GPL(retrode3_bus_get_controller_by_phandle);

int retrode3_bus_lock_bus(struct retrode3_bus_controller *ctlr)
{
	if (!ctlr || !ctlr->ops || !ctlr->ops->lock)
		return -ENOSYS;
	return ctlr->ops->lock(ctlr);
}
EXPORT_SYMBOL_GPL(retrode3_bus_lock_bus);

void retrode3_bus_unlock_bus(struct retrode3_bus_controller *ctlr)
{
	if (!ctlr || !ctlr->ops || !ctlr->ops->unlock)
		return;
	ctlr->ops->unlock(ctlr);
}
EXPORT_SYMBOL_GPL(retrode3_bus_unlock_bus);

int retrode3_bus_set_address(struct retrode3_bus_controller *ctlr, u64 addr)
{
	if (!ctlr || !ctlr->ops || !ctlr->ops->set_addr)
		return -ENOSYS;
	return ctlr->ops->set_addr(ctlr, addr);
}
EXPORT_SYMBOL_GPL(retrode3_bus_set_address);

int retrode3_bus_set_select(struct retrode3_bus_controller *ctlr, unsigned int sel)
{
	if (!ctlr || !ctlr->ops || !ctlr->ops->set_select)
		return -ENOSYS;
	return ctlr->ops->set_select(ctlr, sel);
}
EXPORT_SYMBOL_GPL(retrode3_bus_set_select);

int retrode3_bus_xfer(struct retrode3_bus_controller *ctlr, u8 dir, void *buf, size_t len, size_t *retlen)
{
	if (!ctlr || !ctlr->ops || !ctlr->ops->xfer)
		return -ENOSYS;
	return ctlr->ops->xfer(ctlr, dir, buf, len, retlen);
}
EXPORT_SYMBOL_GPL(retrode3_bus_xfer);

/* init/exit of module: register bus type */
static int __init retrode3_bus_core_init(void)
{
	return bus_register(&retrode3_bus_type);
}
static void __exit retrode3_bus_core_exit(void)
{
	bus_unregister(&retrode3_bus_type);
}
module_init(retrode3_bus_core_init);
module_exit(retrode3_bus_core_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("retrode3 core");
