// SPDX-License-Identifier: GPL-2.0

/*
 * based on ideas and fragents from
 *  drivers/char/mem.c
 *  drivers/gnss/core.c
 *  arch/arm/common/locomo.c

to be solved
- 8 bit vs. 16 bit bus width
- add read/write for RAM
- completion of slot4/gamecontrols driver
- handle status LEDs
- handle megadrive power

Idee:
	aufsplitten in BUS openpandora,retrode3 (wie i2c-core)
	und 2 Arten Device-Treiber on demand laden
	openpandora,retrode3-slot
	openpandora,retrode3-controller
	d.h. einfach über "compatible"

dazu retrode3_device mit Pointer auf den bus


 *
 *  Copyright (C) 2022-23, H. Nikolaus Schaller
 *
 */

#include "../gpio/gpiolib.h"
#include "retrode3_bus.h"

#ifdef CONFIG_IA64
# include <linux/efi.h>
#endif

/* low level address and data bus access */

// FXIME: get_multiple / set_multiple could be much faster, if supported by device driver

static inline int get_bus_bit(struct gpio_desc *desc)
{
#if 1
	struct gpio_chip *gc = desc->gdev->chip;
	return gc->get(gc, gpio_chip_hwgpio(desc));
#else
	return gpiod_get_value(desc);
#endif

}

static inline void set_bus_bit(struct gpio_desc *desc, int value)
{
#if 1
	struct gpio_chip *gc = desc->gdev->chip;
	gc->set(gc, gpio_chip_hwgpio(desc), value);
#else
	gpiod_set_value(desc, value);
#endif

}

/* access to cart bus */

static inline int set_address(struct retrode3_bus *bus, u32 addr)
{ /* set address on all gpios */
	int a;

	if (addr >= EOF)
		return -EINVAL;

// printk("%s:\n", __func__);
	bus->a0 = addr & 1;	// save for 16 bit bus access

// NOTE: this will not use the A0 gpio

	for (a = 1; a < bus->addrs->ndescs; a++) {
		if ((addr ^ bus->prev_addr) & (1 << a))	{ // address bit has really changed
// printk("%s: %d -> %d\n", __func__, a, (addr >> a) & 1);
			set_bus_bit(bus->addrs->desc[a], (addr >> a) & 1);
		}
	}

	bus->prev_addr = addr;
	return 0;
}

// FIXME switch direction of D lines on demand?

static inline int read_byte(struct retrode3_bus *bus)
{ /* read data from data lines */
	int d;
	u8 data;
// printk("%s:\n", __func__);

	set_bus_bit(bus->oe, true);

	/* read data bits either on D0..D7 or D8..D15 */
	data = 0;
	if(bus->a0 == 0)
		for (d = 0; d < bus->datas->ndescs-8; d++) {
			int bit = get_bus_bit(bus->datas->desc[d]);
			if (bit < 0)
				return bit;
			data |= bit << d;
		}
	else
		for (d = 8; d < bus->datas->ndescs; d++) {
			int bit = get_bus_bit(bus->datas->desc[d]);
			if (bit < 0)
				return bit;
			data |= bit << (d-8);
		}
	set_bus_bit(bus->oe, false);
	return data;
}

static inline int read_word(struct retrode3_bus *bus)
{ /* read data from data lines */
	int d;
	u16 data;

// printk("%s:\n", __func__);

	set_bus_bit(bus->oe, true);
	/* read data bits */
	data = 0;
	for (d = 0; d < bus->datas->ndescs; d++) {
		int bit = get_bus_bit(bus->datas->desc[d]);

// printk("bit=%d\n", bit);

		if (bit < 0)
			return bit;
		data |= bit << d;
	}
	set_bus_bit(bus->oe, false);
	return data;
}

static inline void write_word(struct retrode3_bus *bus, u16 data, int mode)
{ /* write data to data lines */
	int d;
	/* set data bits */

// printk("%s:\n", __func__);

	data = 0;
	for (d = 0; d < bus->datas->ndescs; d++) {
		gpiod_direction_output(bus->datas->desc[d], (data>>d) & 1);
	}
	/* should single pulse on upper/lower or both bytes depending on mode! */
	gpiod_set_value(bus->we->desc[0], true);
	gpiod_set_value(bus->we->desc[0], false);
	for (d = 0; d < bus->datas->ndescs; d++) {
#if FIXME
		gpiod_direction_input(bus->datas->desc[d]);
#endif
	}
}

/* cart select */

static void set_slot_power(struct retrode3_slot *slot, int mV)
{
// printk("%s: %d %px\n", __func__, mV, slot->power);

	if (IS_ERR_OR_NULL(slot->power))
		return;

	switch(mV) {
		case 5000:
			gpiod_direction_output(slot->power, false);	// switch to output and pull down
			break;
		case 3300:
			gpiod_direction_input(slot->power);	// floating
			break;
		default:
printk("%s: unknown voltage %d\n", __func__, mV);
	}
}

static void select_slot(struct retrode3_bus *bus, struct retrode3_slot *slot)
{ /* chip select */
	int i;

// printk("%s:\n", __func__);

	if (slot)
		mutex_lock(&bus->select_lock);

	for(i=0; i<ARRAY_SIZE(bus->slots); i++) {
		if (!bus->slots[i])
			continue;
		gpiod_set_value(bus->slots[i]->ce, (bus->slots[i] == slot) ? true:false);
	}

	if (!slot && mutex_is_locked(&bus->select_lock))
		mutex_unlock(&bus->select_lock);
}

// FIXME: make this kernel modules loaded and linked at runtime by retrode3_probe

#include "retrode3_cart.c"
#include "retrode3_controller.c"

/* bus driver probe */

static int retrode3_probe(struct platform_device *pdev)
{
        struct retrode3_bus *bus;
        int i = 0;
	struct device_node *slots, *child = NULL;

        dev_dbg(&pdev->dev, "%s\n", __func__);

        if (!pdev->dev.of_node) {
                dev_err(&pdev->dev, "No device tree data\n");
                return EINVAL;
        }

        bus = devm_kzalloc(&pdev->dev, sizeof(*bus), GFP_KERNEL);
        if (bus == NULL)
                return -ENOMEM;

// printk("%s: a\n", __func__);

	bus->addrs = devm_gpiod_get_array(&pdev->dev, "addr", GPIOD_OUT_HIGH);
// if (IS_ERR(bus->addrs)) usw...
// printk("%s: addrs=%px\n", __func__, bus->addrs);
	bus->datas = devm_gpiod_get_array(&pdev->dev, "data", GPIOD_IN);
// printk("%s: datas=%px\n", __func__, bus->datas);
	bus->oe = devm_gpiod_get(&pdev->dev, "oe", GPIOD_OUT_HIGH);	// active LOW is XORed with DT definition
// printk("%s: oe=%px\n", __func__, bus->oe);
	gpiod_set_value(bus->oe, false);	// turn inactive
	bus->we = devm_gpiod_get_array(&pdev->dev, "we", GPIOD_OUT_HIGH);	// active LOW is XORed with DT definition
// printk("%s: we=%px\n", __func__, bus->we);
	gpiod_set_value(bus->we->desc[0], false);	// make both inactive
	gpiod_set_value(bus->we->desc[1], false);
	bus->time = devm_gpiod_get(&pdev->dev, "time", GPIOD_OUT_HIGH);	// active LOW is XORed with DT definition
// printk("%s: time=%px\n", __func__, bus->time);
	gpiod_set_value(bus->time, false);	// make inactive
	bus->reset = devm_gpiod_get(&pdev->dev, "reset", GPIOD_OUT_HIGH);	// active LOW is XORed with DT definition
// printk("%s: reset=%px\n", __func__, bus->reset);
	gpiod_set_value(bus->reset, false);	// make inactive

#if 0
printk("%s: 1\n", __func__);
printk("%s: bus=%px\n", __func__, bus);
printk("%s: addrs=%px\n", __func__, bus->addrs);
printk("%s: desc[0]=%px\n", __func__, bus->addrs->desc[0]);
printk("%s: gdev=%px\n", __func__, bus->addrs->desc[0]->gdev);
printk("%s: chip=%px\n", __func__, bus->addrs->desc[0]->gdev->chip);
#endif

	if (bus->addrs->ndescs != 24 ||
	    bus->datas->ndescs != 16 ||
	    bus->we->ndescs != 2) {
		dev_err(&pdev->dev, "Invalid number of gpios (addr=%d, data=%d, we=%d)\n",
			bus->addrs->ndescs, bus->datas->ndescs, bus->we->ndescs);
		return EINVAL;
	}
#if 0
{
	struct gpio_chip *gc = bus->addrs->desc[0]->gdev->chip;
	printk("%s: get %ps\n", __func__, gc->get);
	printk("%s: getmult %ps\n", __func__, gc->get_multiple);
	printk("%s: set %ps\n", __func__, gc->set);
	printk("%s: setmult %ps\n", __func__, gc->set_multiple);
	printk("%s: dirout %ps\n", __func__, gc->direction_output);	// no direction_output_multiple!
	printk("%s: dirin %ps\n", __func__, gc->direction_input);	// no direction_input_multiple!
	printk("%s: setconf %ps\n", __func__, gc->set_config);
}
#endif

	mutex_init(&bus->select_lock);

// FIXME: do we needs this grouping?

	slots = of_get_child_by_name(pdev->dev.of_node, "slots");
	while ((child = of_get_next_child(slots, child))) {
		struct retrode3_slot *slot;
		int ret;

// printk("%s: slot %d\n", __func__, i);
		if (i >= ARRAY_SIZE(bus->slots)) {
			dev_err(&slot->dev, "too many slots\n");
// FIXME: better error handling
			// FIXME: dealloc previously added devices
			kfree(bus);
			return -EINVAL;
		}

		slot = kzalloc(sizeof(*slot), GFP_KERNEL);
		if (!slot) {
			kfree(bus);
			return -ENOMEM;
		}

		bus->slots[i++] = slot;
		slot->bus = bus;
		slot->dev.parent = &pdev->dev;

// FIXME: should load bus client driver through matching .compatible
// like i2c bus is doing

		if (of_property_match_string(child, "compatible", "openpandora,retrode3-slot") >= 0)
			ret = retrode3_probe_slot(slot, child);
		else if (of_property_match_string(child, "compatible", "openpandora,retrode3-controller") >= 0)
			ret = retrode3_probe_controller(slot, child);
		else {
			dev_err(&slot->dev, "unknown child type\n");
			ret = -EINVAL;
		}

		if (ret < 0) {
			// FIXME: dealloc previously added devices
			kfree(bus);
			kfree(slot);
			return ret;
		}
		dev_dbg(&pdev->dev, "%s added\n", __func__);
	}

        platform_set_drvdata(pdev, bus);

        dev_dbg(&pdev->dev, "%s successful\n", __func__);

	select_slot(bus, NULL);	// deselect all slots

        return 0;
}

static int retrode3_remove(struct platform_device *pdev)
{
        struct retrode3_bus *bus = platform_get_drvdata(pdev);
	int i;

	select_slot(bus, NULL);	// deselect all slots

	for (i=0; i<ARRAY_SIZE(bus->slots); i++) {
		struct retrode3_slot *slot = bus->slots[i];

		cancel_delayed_work_sync(&slot->work);
		cdev_device_del(&slot->cdev, &slot->dev);
	}

	mutex_destroy(&bus->select_lock);

        return 0;
}

static const struct of_device_id retrode3_of_match[] = {
        { .compatible = "openpandora,retrode3" },
        {},
};
MODULE_DEVICE_TABLE(of, retrode3_of_match);

static struct platform_driver retrode3_driver = {
        .probe          = retrode3_probe,
        .remove         = retrode3_remove,
        .driver = {
                .name   = "retrode3",
                .owner  = THIS_MODULE,
                .of_match_table = of_match_ptr(retrode3_of_match)
        },
};

static int retrode3_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct retrode3_slot *slot = container_of(dev, struct retrode3_slot, dev);
	int ret;
	char buf[100];

        dev_info(&slot->dev, "%s\n", __func__);

	ret = add_uevent_var(env, "SLOT=%s", dev_name(dev));
	if (ret)
		return ret;

	sense_show(dev, NULL, buf);
	// strip off \n?
	ret = add_uevent_var(env, "STATE=%s", buf);
	if (ret)
		return ret;

	return 0;
}

static ssize_t sense_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct retrode3_slot *slot = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", gpiod_get_value(slot->cd)?"active":"empty");
}
static DEVICE_ATTR_RO(sense);

static struct attribute *retrode3_attrs[] = {
	&dev_attr_sense.attr,
	NULL,
};
ATTRIBUTE_GROUPS(retrode3);

static int __init retrode3_module_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&retrode3_first, 0, RETRODE3_MINORS, "retrode3");
	if (ret < 0) {
		pr_err("failed to allocate device numbers: %d\n", ret);
		return ret;
	}

	retrode3_class = class_create("retrode3");
	if (IS_ERR(retrode3_class)) {
		ret = PTR_ERR(retrode3_class);
		pr_err("failed to create class: %d\n", ret);
		goto err_unregister_chrdev;
	}

	retrode3_class->dev_groups = retrode3_groups;	// for additional /sys attributes

// triggers permanent sequence of uEvents...
//	retrode3_class->dev_uevent = retrode3_uevent;

	ret = platform_driver_register(&retrode3_driver);

	if (ret < 0)
		goto err_destroy_class;

	pr_info("retrode3 driver registered with major %d\n", MAJOR(retrode3_first));

	return ret;

err_destroy_class:
	class_destroy(retrode3_class);
err_unregister_chrdev:
	unregister_chrdev_region(retrode3_first, RETRODE3_MINORS);

	return ret;
}

static void __exit retrode3_module_exit(void)
{

	class_destroy(retrode3_class);
	unregister_chrdev_region(retrode3_first, RETRODE3_MINORS);
	ida_destroy(&retrode3_minors);

	platform_driver_unregister(&retrode3_driver);
}

module_init(retrode3_module_init);
module_exit(retrode3_module_exit);

MODULE_ALIAS("retrode3");

MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("retrode 3 bus driver");
MODULE_LICENSE("GPL v2");
