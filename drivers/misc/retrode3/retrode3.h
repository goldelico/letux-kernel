/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _LINUX_RETRODE3_BUS_H
#define _LINUX_RETRODE3_BUS_H

// FIXME: better naming scheme: only retrode3 specific drivers and structs should be named such!
// then, it may be useable for a retrode4...
// i.e. only retrode3-bus.c

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/types.h>

/* general bus controller operations implemented by specific controller driver */

struct retrode3_bus_controller;
struct retrode3_slot;
extern struct bus_type retrode3_bus_type;

struct retrode3_bus_controller_ops {
	int (*set_addr)(struct retrode3_bus_controller *controller, u32 addr);
	int (*select)(struct retrode3_bus_controller *controller, struct retrode3_slot *slot);
	int (*xfer)(struct retrode3_bus_controller *controller, int mode, int arg1, int arg2);
};

/* a general bus controller */
struct retrode3_bus_controller {
	struct device *dev;	/* specific bus controller device (platform/device) */
	struct list_head slots;	// list of retrode3_slots
	const struct retrode3_bus_controller_ops *ops;
	void *priv; /* controller private pointer */
	struct mutex lock;
	struct retrode3_slot *selected_slot;
	u32 current_address;
};

/* API for controllers */
int retrode3_bus_register_controller(struct retrode3_bus_controller *controller);
void retrode3_bus_unregister_controller(struct retrode3_bus_controller *controller);

/* a general Retrode3 client device */
struct retrode3_slot {
	struct device dev; /* embed device - clients use container_of */
	struct retrode3_bus_controller *controller;
	struct list_head list;
	struct gpio_desc *cd;	// Cart Detect
	struct gpio_desc *ce;	// Cart Enable
	struct gpio_desc *led;	// Status LED
	struct gpio_desc *power;	// Power control
#define EOF	(1L<<24)	// max 24 address lines = 16 MByte
	u32 addr_width;		// address width used
	u32 bus_width;		// data width used
	struct delayed_work work;	// cd polling worker
	int cd_state;		// last CD state
	struct cdev cdev;	// the /dev/slot character device
	int id;				// slot id
};

#define to_retrode3_slot(d) container_of(d, struct retrode3_slot, dev)

/* API for bus clients */
/* some functions address the slot directly (select, power) and some the bus controller (address, data, oe, we, time, reset) */
struct retrode3_slot *retrode3_slot_create_from_of(struct retrode3_bus_controller *controller,
												   struct device_node *np);
void retrode3_bus_client_remove(struct retrode3_slot *slot);

/* functions to access the slots */
int retrode3_bus_select_slot(struct retrode3_bus_controller *controller, struct retrode3_slot *slot);
int retrode3_is_selected(struct retrode3_slot *slot);
int retrode3_get_slot_power_mV(struct retrode3_slot *slot);
int retrode3_set_slot_power_mV(struct retrode3_slot *controller, int mV);

/* Transfer mode flags */
enum {
	/* read input */
	RETRODE3_BUS_READ_HALF,
	RETRODE3_BUS_READ_BYTE,
	RETRODE3_BUS_READ_WORD,
	/* output, write, input */
	RETRODE3_BUS_WRITE_HALF,
	RETRODE3_BUS_WRITE_BYTE,
	RETRODE3_BUS_WRITE_WORD,
	/* output and write */
	RETRODE3_BUS_DRIVE_HALF,
	RETRODE3_BUS_DRIVE_WORD,
	/* input */
	RETRODE3_BUS_END_DRIVE,
	/* control lines */
	RETRODE3_BUS_ACTIVATE_WE,
	RETRODE3_BUS_ACTIVATE_TIME,
	RETRODE3_BUS_ACTIVATE_RESET,
};

/* forwarders to the bus driver */
int retrode3_bus_set_address(struct retrode3_bus_controller *controller, u32 addr);
int retrode3_bus_xfer(struct retrode3_bus_controller *controller, int mode, int arg1, int arg2);

/* wrappers to call bus_xfer in a readable, type checked, but also extensible way - note, these calls work for the selected slot! */

static inline int retrode3_read_half(struct retrode3_bus_controller *controller, int a0)	// D0..D7 or D8..D15
{
	return retrode3_bus_xfer(controller, RETRODE3_BUS_READ_HALF, a0, 0);
}

static inline int retrode3_read_byte(struct retrode3_bus_controller *controller)	// use bit 0 of current_address
{
	return retrode3_bus_xfer(controller, RETRODE3_BUS_READ_BYTE, 0, 0);
}

static inline int retrode3_read_word(struct retrode3_bus_controller *controller)	// D0..D15
{
	return retrode3_bus_xfer(controller, RETRODE3_BUS_READ_WORD, 0, 0);
}

static inline int retrode3_write_half(struct retrode3_bus_controller *controller, uint8_t data, int a0)	// D0..D7 or D8..D15
{
	return retrode3_bus_xfer(controller, RETRODE3_BUS_WRITE_HALF, a0, data);
}

static inline int retrode3_write_byte(struct retrode3_bus_controller *controller, uint8_t data)	// use bit 0 of current_address
{
	return retrode3_bus_xfer(controller, RETRODE3_BUS_WRITE_BYTE, 0, data);
}

static inline int retrode3_write_word(struct retrode3_bus_controller *controller, uint16_t data)	// D0..D15
{
	return retrode3_bus_xfer(controller, RETRODE3_BUS_WRITE_WORD, 0, data);
}

static inline int retrode3_drive_half(struct retrode3_bus_controller *controller, uint16_t data, int a0)
{
	return retrode3_bus_xfer(controller, RETRODE3_BUS_DRIVE_HALF, a0, data);
}

static inline int retrode3_drive_word(struct retrode3_bus_controller *controller, uint16_t data)
{
	return retrode3_bus_xfer(controller, RETRODE3_BUS_DRIVE_WORD, 0, data);
}

static inline int retrode3_end_drive(struct retrode3_bus_controller *controller)
{
	return retrode3_bus_xfer(controller, RETRODE3_BUS_END_DRIVE, 0, 0);
}

static inline int retrode3_set_we(struct retrode3_bus_controller *controller, int state, int a0)
{ /* 1 = active */
	return retrode3_bus_xfer(controller, RETRODE3_BUS_ACTIVATE_WE, a0, state);
}

static inline int retrode3_set_time(struct retrode3_bus_controller *controller, int state)
{ /* 1 = active */
	return retrode3_bus_xfer(controller, RETRODE3_BUS_ACTIVATE_TIME, 0, state);
}

static inline int retrode3_set_reset(struct retrode3_bus_controller *controller, int state)
{ /* 1 = active */
	return retrode3_bus_xfer(controller, RETRODE3_BUS_ACTIVATE_RESET, 0, state);
}

/* the specific retrode3 bus client (slot) driver */
struct retrode3_slot_driver {
	struct device_driver driver;
	int (*probe)(struct retrode3_slot *dev);
	int (*remove)(struct retrode3_slot *dev);
	int (*select)(struct retrode3_slot *dev, unsigned int state);
	int (*is_selected)(struct retrode3_slot *dev);
};

/* API for creating devices from DT */
void retrode3_set_bus_type(struct retrode3_slot_driver *drv);
int retrode3_slot_driver_register(struct retrode3_slot_driver *drv);
void retrode3_slot_driver_unregister(struct retrode3_slot_driver *drv);

#endif /* _LINUX_RETRODE3_BUS_H */
