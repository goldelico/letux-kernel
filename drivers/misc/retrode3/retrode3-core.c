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
EXPORT_SYMBOL_GPL(retrode3_bus_type);

/* Called by controller driver to register itself */
int retrode3_bus_register_controller(struct retrode3_bus_controller *controller)
{
	struct device_node *child;
	struct retrode3_slot *slot;
	struct device *dev = controller->dev;
	printk("%s %d\n", __func__, __LINE__);

	if (!controller || !controller->ops)
		return -EINVAL;

	/* Initialize list of slots belonging to this controller */
	INIT_LIST_HEAD(&controller->slots);

	mutex_init(&controller->lock);

	/* create retrode3 slots for each DT child so client drivers get probed */
	if (dev->of_node) {
		for_each_available_child_of_node(dev->of_node, child) {
			slot = retrode3_slot_create_from_of(controller, child);
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
	struct retrode3_slot *slot, *tmp;

	printk("%s %d\n", __func__, __LINE__);

	list_for_each_entry_safe(slot, tmp, &controller->slots, list) {
		retrode3_bus_client_remove(slot);
	}
}
EXPORT_SYMBOL_GPL(retrode3_bus_unregister_controller);

/* Client API implementations */

int retrode3_bus_select_slot(struct retrode3_bus_controller *controller, struct retrode3_slot *slot)
{
	printk("%s %d\n", __func__, __LINE__);
	if (!controller || !controller->ops || !controller->ops->select)
		return -ENOSYS;
	controller->ops->select(controller, slot);
	return 0;
}
EXPORT_SYMBOL_GPL(retrode3_bus_select_slot);

int retrode3_is_selected(struct retrode3_slot *slot)
{
	printk("%s %d\n", __func__, __LINE__);
	return slot->controller->selected_slot == slot;
}
EXPORT_SYMBOL_GPL(retrode3_is_selected);

int retrode3_bus_set_address(struct retrode3_bus_controller *controller, u32 addr)
{
	int ret;

	printk("%s %d\n", __func__, __LINE__);
	if (!controller || !controller->ops || !controller->ops->set_addr)
		return -ENOSYS;
	ret = controller->ops->set_addr(controller, addr);
	if (ret >= 0)
		controller->current_address = addr;
	return ret;
}
EXPORT_SYMBOL_GPL(retrode3_bus_set_address);

int retrode3_bus_xfer(struct retrode3_bus_controller *controller, int mode, int arg1, int arg2)
{
	printk("%s %d\n", __func__, __LINE__);
	if (!controller || !controller->ops || !controller->ops->xfer)
		return -ENOSYS;
	return controller->ops->xfer(controller, mode, arg1, arg2);
}
EXPORT_SYMBOL_GPL(retrode3_bus_xfer);

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
