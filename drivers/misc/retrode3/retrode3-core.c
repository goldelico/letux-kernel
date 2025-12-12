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
	printk("%s %d\n", __func__, __LINE__);
	/* default match uses of_match if dev->of_node/driver->of_match_table set */
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
	struct retrode3_bus_device *slot;
	struct device *dev = controller->dev;
	printk("%s %d\n", __func__, __LINE__);

	if (!controller || !controller->ops)
		return -EINVAL;

	/* Initialize list of devices belonging to this controller */
	INIT_LIST_HEAD(&controller->devices);

	mutex_init(&controller->lock);

	/* create retrode3 devices for each DT child so client drivers get probed */
	if (dev->of_node) {
		for_each_available_child_of_node(dev->of_node, child) {
			slot = retrode3_bus_client_create_from_of(controller, child);
			if (IS_ERR(slot)) {
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
	struct retrode3_bus_device *slot, *tmp;

	printk("%s %d\n", __func__, __LINE__);

	list_for_each_entry_safe(slot, tmp, &controller->devices, list) {
		retrode3_bus_client_remove(slot);
	}
}
EXPORT_SYMBOL_GPL(retrode3_bus_unregister_controller);

static void retrode3_bus_dev_release(struct device *dev)
{
	struct retrode3_bus_device *slot = to_retrode3_bus_device(dev);

	kfree(slot);
}

/* device_create helper: allocates a retrode3_bus_device, sets parent to controller device */
struct retrode3_bus_device *retrode3_bus_client_create_from_of(struct retrode3_bus_controller *controller,
															   struct device_node *np)
{
	struct retrode3_bus_device *slot;
	int ret;
	printk("%s %d\n", __func__, __LINE__);

	if (!controller || !np)
		return ERR_PTR(-EINVAL);

	slot = kzalloc(sizeof(*slot), GFP_KERNEL);
	if (!slot)
		return ERR_PTR(-ENOMEM);

	slot->controller = controller;

	device_initialize(&slot->dev);
	slot->dev.parent = controller->dev;
	slot->dev.bus = &retrode3_bus_type;
	slot->dev.of_node = of_node_get(np);

	slot->dev.release = retrode3_bus_dev_release;

	ret = dev_set_name(&slot->dev, np->name);
	if (ret) {
		put_device(&slot->dev);
		kfree(slot);
		return ERR_PTR(ret);
	}

	ret = device_add(&slot->dev);
	if (ret) {
		put_device(&slot->dev);
		kfree(slot);
		return ERR_PTR(ret);
	}

	list_add_tail(&slot->list, &controller->devices);

	dev_info(controller->dev, "created retrode3 device %s\n", dev_name(&slot->dev));
	return slot;
}
EXPORT_SYMBOL_GPL(retrode3_bus_client_create_from_of);

void retrode3_bus_client_remove(struct retrode3_bus_device *slot)
{
	if (!slot)
		return;

	list_del(&slot->list);

	device_del(&slot->dev);
	put_device(&slot->dev);
}
EXPORT_SYMBOL_GPL(retrode3_bus_client_remove);

int retrode3_bus_client_driver_register(struct retrode3_bus_client_driver *drv)
{
	printk("%s %d\n", __func__, __LINE__);
	drv->driver.bus = &retrode3_bus_type;
	return driver_register(&drv->driver);
}
EXPORT_SYMBOL_GPL(retrode3_bus_client_driver_register);

void retrode3_bus_client_driver_unregister(struct retrode3_bus_client_driver *drv)
{
	printk("%s %d\n", __func__, __LINE__);
	driver_unregister(&drv->driver);
}
EXPORT_SYMBOL_GPL(retrode3_bus_client_driver_unregister);

/* Client API implementations */

int retrode3_bus_lock_bus(struct retrode3_bus_controller *controller)
{
	printk("%s %d\n", __func__, __LINE__);
	if (!controller || !controller->ops || !controller->ops->lock)
		return -ENOSYS;
	return controller->ops->lock(controller);
}
EXPORT_SYMBOL_GPL(retrode3_bus_lock_bus);

void retrode3_bus_unlock_bus(struct retrode3_bus_controller *controller)
{
	printk("%s %d\n", __func__, __LINE__);
	if (!controller || !controller->ops || !controller->ops->unlock)
		return;
	controller->ops->unlock(controller);
}
EXPORT_SYMBOL_GPL(retrode3_bus_unlock_bus);

int retrode3_bus_set_address(struct retrode3_bus_controller *controller, u32 addr)
{
	printk("%s %d\n", __func__, __LINE__);
	if (!controller || !controller->ops || !controller->ops->set_addr)
		return -ENOSYS;
	return controller->ops->set_addr(controller, addr);
}
EXPORT_SYMBOL_GPL(retrode3_bus_set_address);

int retrode3_bus_xfer(struct retrode3_bus_controller *controller, u8 dir, void *buf, size_t len)
{
	printk("%s %d\n", __func__, __LINE__);
	if (!controller || !controller->ops || !controller->ops->xfer)
		return -ENOSYS;
	return controller->ops->xfer(controller, dir, buf, len);
}
EXPORT_SYMBOL_GPL(retrode3_bus_xfer);

// FIXME: das hier ist sehr verdreht...
// hier können wir verschiedene Controller behandeln, also Bus-Treiber
// diese greifen aber auf den CE von Clients zu, die nur dort bekannt sind
// dieses select_device wird aber vom Client selbst ausgelöst
// eine zentrale Funktion braucht es aber nur fürs locking oder um den bisher selektierten Slot zu deselectieren
// ist das hier also eine Eigenschaft des core - oder des speziellen bus???
// der spezielle Retrode-Bus-Treiber kennt aber keine retrode3_bus_device
// Ziel ist zu vermeiden dass ein Objekt zu viel vom anderen wissen muss!
// die retrode3_bus_device Children sind aber hier bekannt!

void retrode3_bus_select_device(struct retrode3_bus_controller *controller, struct retrode3_bus_device *slot)
{ // NOTE: this relies on all slots initially being deselected
	if (slot) {
//		if (!slot->dev->driver->is_selected || slot->dev->driver->is_selected(slot))
			return;	// already selected (and locked)
//		mutex_lock(&bus->select_lock);	// lock until slot == 0
	}

	// FIXME: das hier setzt alle CE neu!
#if MOVE_TO_DEVICE_DRIVER
	for(i=0; i<ARRAY_SIZE(bus->slots); i++) {
		if (!bus->slots[i])
			continue;	// avoid to match slot == NULL
		gpiod_set_value(bus->slots[i]->ce, (bus->slots[i] == slot) ? 1:0);
	}
#endif

//	if (!slot && mutex_is_locked(&bus->select_lock))
//		mutex_unlock(&bus->select_lock);

	if (slot != controller->selected_slot) {
#if CHECKME
		if (controller->selected_slot)
			selected_slot->select(selected_slot->select, 0);	// deselect old
		if (controller->selected_slot = slot) {
			if (selected_slot->select)
				selected_slot->select(selected_slot->select, 1);	// select new
		}
#endif
	}
}
EXPORT_SYMBOL_GPL(retrode3_bus_select_device);

/* init/exit of module: register bus type */
static int __init retrode3_bus_core_init(void)
{
	printk("%s %d\n", __func__, __LINE__);
	return bus_register(&retrode3_bus_type);
}
static void __exit retrode3_bus_core_exit(void)
{
	printk("%s %d\n", __func__, __LINE__);
	bus_unregister(&retrode3_bus_type);
}
module_init(retrode3_bus_core_init);
module_exit(retrode3_bus_core_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("retrode3 core");
