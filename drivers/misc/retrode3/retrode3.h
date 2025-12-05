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
    int (*lock)(struct retrode3_bus_controller *ctlr);
    void (*unlock)(struct retrode3_bus_controller *ctlr);

    int (*set_addr)(struct retrode3_bus_controller *ctlr, u64 addr);
    int (*set_select)(struct retrode3_bus_controller *ctlr, unsigned int sel);

    /* Generic transfer: direction, address already set, buffer is bytes */
    int (*xfer)(struct retrode3_bus_controller *ctlr, u8 direction,
                void *buf, size_t len, size_t *actual);
};

struct retrode3_bus_controller {
    struct device *dev; /* controller device (platform/device) */
    const struct retrode3_bus_controller_ops *ops;
    void *priv; /* controller private pointer */
    /* internal locking: exported functions will use this */
    struct mutex lock;
};

struct retrode3_bus_device {
    struct device dev; /* embed device - clients use container_of */
    /* bus-specific fields */
    struct retrode3_bus_controller *ctlr;
    u32 addr_width;
    u32 data_width;
    /* any other per-device metadata */
};

#define to_retrode3_bus_device(d) container_of(d, struct retrode3_bus_device, dev)

/* API for controllers */
int retrode3_bus_register_controller(struct retrode3_bus_controller *ctlr);
void retrode3_bus_unregister_controller(struct retrode3_bus_controller *ctlr);

/* API for creating devices from DT (call from controller probe) */
struct retrode3_bus_device *retrode3_bus_device_create_from_of(struct retrode3_bus_controller *ctlr,
                                                 struct device_node *np);
void retrode3_bus_device_remove(struct retrode3_bus_device *mdev);

/* Client API (exported) */
struct retrode3_bus_controller *retrode3_bus_get_controller_by_phandle(struct device *dev,
                                                         const char *propname);
int retrode3_bus_lock_bus(struct retrode3_bus_controller *ctlr);
void retrode3_bus_unlock_bus(struct retrode3_bus_controller *ctlr);
int retrode3_bus_set_address(struct retrode3_bus_controller *ctlr, u64 addr);
int retrode3_bus_set_select(struct retrode3_bus_controller *ctlr, unsigned int sel);
int retrode3_bus_xfer(struct retrode3_bus_controller *ctlr, u8 dir, void *buf, size_t len, size_t *retlen);

#endif /* _LINUX_RETRODE3_BUS_H */
