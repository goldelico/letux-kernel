/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _LINUX_RETRODE3_BUS_H
#define _LINUX_RETRODE3_BUS_H

#include <linux/device.h>
#include <linux/of.h>
#include <linux/types.h>

struct retrode3_bus_controller;
struct retrode3_bus_device;

/* Transfer direction flags */
#define retrode3_bus_DIR_READ  0
#define retrode3_bus_DIR_WRITE 1

/* controller operations implemented by controller driver */
struct retrode3_bus_controller_ops {
	int (*lock)(struct retrode3_bus_controller *controller);
	void (*unlock)(struct retrode3_bus_controller *controller);

	int (*set_addr)(struct retrode3_bus_controller *controller, u32 addr);
	int (*set_select)(struct retrode3_bus_controller *controller, unsigned int sel);

	/* Generic transfer: direction, address already set, buffer is bytes */
	int (*xfer)(struct retrode3_bus_controller *controller, u8 direction,
				void *buf, size_t len);
};

struct retrode3_bus_controller {
	struct device *dev; /* controller device (platform/device) */
	struct list_head devices;     // list of retrode3_bus_device
	const struct retrode3_bus_controller_ops *ops;
	void *priv; /* controller private pointer */
	/* internal locking: exported functions will use this */
	struct mutex lock;
};

struct retrode3_bus_device {
	struct device dev; /* embed device - clients use container_of */
	/* bus-specific fields */
	struct retrode3_bus_controller *controller;
	struct list_head list;
	u32 addr_width;
	u32 data_width;
	/* any other per-device metadata */
};

struct retrode3_bus_client_driver {
	struct device_driver driver;
	int (*probe)(struct retrode3_bus_device *dev);
	int (*remove)(struct retrode3_bus_device *dev);
};

#define to_retrode3_bus_device(d) container_of(d, struct retrode3_bus_device, dev)

/* API for controllers */
int retrode3_bus_register_controller(struct retrode3_bus_controller *controller);
void retrode3_bus_unregister_controller(struct retrode3_bus_controller *controller);
struct retrode3_bus_device *retrode3_bus_client_create_from_of(struct retrode3_bus_controller *controller,
															   struct device_node *np);
void retrode3_bus_client_remove(struct retrode3_bus_device *mdev);

/* API for creating devices from DT */
int retrode3_bus_client_driver_register(struct retrode3_bus_client_driver *drv);
void retrode3_bus_client_driver_unregister(struct retrode3_bus_client_driver *drv);

/* API for bus access */
int retrode3_bus_lock_bus(struct retrode3_bus_controller *controller);
void retrode3_bus_unlock_bus(struct retrode3_bus_controller *controller);
int retrode3_bus_set_address(struct retrode3_bus_controller *controller, u32 addr);
int retrode3_bus_set_select(struct retrode3_bus_controller *controller, unsigned int sel);
int retrode3_bus_xfer(struct retrode3_bus_controller *controller, u8 dir, void *buf, size_t len);

#endif /* _LINUX_RETRODE3_BUS_H */
