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

static int retrode3_bus_match(struct device *dev, const struct device_driver *drv)
{
	return of_driver_match_device(dev, drv);
}

static int retrode3_uevent(const struct device *dev, struct kobj_uevent_env *env)
{
	const char *compatible;

	if (!dev->of_node)
		return 0;

	compatible = of_get_property(dev->of_node, "compatible", NULL);
	if (!compatible)
		return 0;

	/* Build MODALIAS string: "of:N*T*C<vendor>,<name>" */
	return add_uevent_var(env, "MODALIAS=of:N*T*C%s", compatible);
}

/* Bus type for driver core */
struct bus_type retrode3_bus_type = {
	.name = "retrode3",
	.match = retrode3_bus_match,
	.uevent = retrode3_uevent,
};

/* Called by controller driver to register itself */
int retrode3_bus_register_controller(struct retrode3_bus_controller *controller)
{
	struct device_node *child;
	struct retrode3_bus_device *mdev;
	struct device *dev = controller->dev;

	if (!controller || !controller->ops)
		return -EINVAL;

	/* Initialize list of devices belonging to this controller */
	INIT_LIST_HEAD(&controller->devices);

	mutex_init(&controller->lock);

	/* create retrode3 devices for each DT child so client drivers get probed */
	if (dev->of_node) {
		for_each_available_child_of_node(dev->of_node, child) {
			mdev = retrode3_bus_client_create_from_of(controller, child);
			if (IS_ERR(mdev)) {
				dev_warn(dev, "failed to create retrode3 device for %pOF\n", child);
			}
			of_node_put(child);
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(retrode3_bus_register_controller);

void retrode3_bus_unregister_controller(struct retrode3_bus_controller *controller)
{
	struct retrode3_bus_device *mdev, *tmp;

	list_for_each_entry_safe(mdev, tmp, &controller->devices, list) {
		retrode3_bus_client_remove(mdev);
	}
}
EXPORT_SYMBOL_GPL(retrode3_bus_unregister_controller);

static void retrode3_bus_dev_release(struct device *dev)
{
	struct retrode3_bus_device *mdev = to_retrode3_bus_device(dev);

	kfree(mdev);
}

/* device_create helper: allocates a retrode3_bus_device, sets parent to controller device */
struct retrode3_bus_device *retrode3_bus_client_create_from_of(struct retrode3_bus_controller *controller,
															   struct device_node *np)
{
	struct retrode3_bus_device *mdev;
	int ret;

	if (!controller || !np)
		return ERR_PTR(-EINVAL);

	mdev = kzalloc(sizeof(*mdev), GFP_KERNEL);
	if (!mdev)
		return ERR_PTR(-ENOMEM);

	mdev->controller = controller;

	// FIXME: do we need this here?
	/* read optional DT properties */
	of_property_read_u32(np, "retrode3,addr-width", &mdev->addr_width);
	of_property_read_u32(np, "retrode3,data-width", &mdev->data_width);

	device_initialize(&mdev->dev);
	mdev->dev.parent = controller->dev;
	mdev->dev.bus = &retrode3_bus_type;
	mdev->dev.of_node = of_node_get(np);

	mdev->dev.release = retrode3_bus_dev_release;

	ret = dev_set_name(&mdev->dev, np->name);
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

	list_add_tail(&mdev->list, &controller->devices);

	dev_info(controller->dev, "created retrode3 device %s\n", dev_name(&mdev->dev));
	return mdev;
}
EXPORT_SYMBOL_GPL(retrode3_bus_client_create_from_of);

void retrode3_bus_client_remove(struct retrode3_bus_device *mdev)
{
	if (!mdev)
		return;

	list_del(&mdev->list);

	device_del(&mdev->dev);
	put_device(&mdev->dev);
}
EXPORT_SYMBOL_GPL(retrode3_bus_client_remove);

int retrode3_bus_client_driver_register(struct retrode3_bus_client_driver *drv)
{
	drv->driver.bus = &retrode3_bus_type;
	return driver_register(&drv->driver);
}
EXPORT_SYMBOL_GPL(retrode3_bus_client_driver_register);

void retrode3_bus_client_driver_unregister(struct retrode3_bus_client_driver *drv)
{
	driver_unregister(&drv->driver);
}
EXPORT_SYMBOL_GPL(retrode3_bus_client_driver_unregister);

/* Client API implementations */

#if 0
struct retrode3_bus_controller *retrode3_bus_get_controller_by_phandle(
																	   struct device *dev,
																	   const char *propname)
{
	struct device_node *np;
	struct device_node *controller_np;
	struct platform_device *pdev;
	struct retrode3_bus_controller *controller = NULL;

	if (!dev || !propname)
		return ERR_PTR(-EINVAL);

	np = dev->of_node;
	if (!np)
		return ERR_PTR(-ENODEV);

	controller_np = of_parse_phandle(np, propname, 0);
	if (!controller_np)
		return ERR_PTR(-ENODEV);

	pdev = of_find_device_by_node(controller_np);
	of_node_put(controller_np);
	if (!pdev)
		return ERR_PTR(-ENODEV);

	controller = platform_get_drvdata(pdev);
	if (!controller)
		return ERR_PTR(-ENODEV);

	return controller;
}
EXPORT_SYMBOL_GPL(retrode3_bus_get_controller_by_phandle);
#endif

int retrode3_bus_lock_bus(struct retrode3_bus_controller *controller)
{
	if (!controller || !controller->ops || !controller->ops->lock)
		return -ENOSYS;
	return controller->ops->lock(controller);
}
EXPORT_SYMBOL_GPL(retrode3_bus_lock_bus);

void retrode3_bus_unlock_bus(struct retrode3_bus_controller *controller)
{
	if (!controller || !controller->ops || !controller->ops->unlock)
		return;
	controller->ops->unlock(controller);
}
EXPORT_SYMBOL_GPL(retrode3_bus_unlock_bus);

int retrode3_bus_set_address(struct retrode3_bus_controller *controller, u32 addr)
{
	if (!controller || !controller->ops || !controller->ops->set_addr)
		return -ENOSYS;
	return controller->ops->set_addr(controller, addr);
}
EXPORT_SYMBOL_GPL(retrode3_bus_set_address);

int retrode3_bus_set_select(struct retrode3_bus_controller *controller, unsigned int sel)
{
	if (!controller || !controller->ops || !controller->ops->set_select)
		return -ENOSYS;
	return controller->ops->set_select(controller, sel);
}
EXPORT_SYMBOL_GPL(retrode3_bus_set_select);

int retrode3_bus_xfer(struct retrode3_bus_controller *controller, u8 dir, void *buf, size_t len)
{
	if (!controller || !controller->ops || !controller->ops->xfer)
		return -ENOSYS;
	return controller->ops->xfer(controller, dir, buf, len);
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
