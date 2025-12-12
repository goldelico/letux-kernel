/* SPDX-License-Identifier: GPL-2.0-only */

#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/io.h>

#include "../../gpio/gpiolib.h"		// for direct access to desc->gdev->chip and gpio_chip_hwgpio()

#include "retrode3.h"

struct retrode3_bus {
	struct retrode3_bus_controller controller;
	struct platform_device *pdev;
	struct gpio_descs *addrs;	// addr-gpios
	struct gpio_descs *datas;	// data-gpios
	struct gpio_descs *we;		// we-gpios
	struct gpio_desc *oe;		// oe-gpio
	struct gpio_desc *time;		// time-gpio
	struct gpio_desc *reset;	// reset-gpio
	struct retrode3_slot *slots[4];
	struct mutex select_lock;	// used by select_slot
	uint32_t current_addr;
};

#define EOF	(1L<<24)	// 24 address lines = 16 MByte

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

static inline int set_bus_bit(struct gpio_desc *desc, int value)
{
#if 1	// speed optimized direct call
	struct gpio_chip *gc = desc->gdev->chip;
	// can we use set_multiple?
// printk("%s: %px set=%pS set=%pS\n", __func__, gc, gc->set, gc->set);
	return gc->set(gc, gpio_chip_hwgpio(desc), value);
#else
	return gpiod_set_value(desc, value);
#endif
}

/* access to cart bus */

static inline int set_address(struct retrode3_bus *bus, uint32_t addr)
{ /* set address on all gpios */
	int a;

// printk("%s: bus %px addr %08x\n", __func__, bus, addr);

	if (addr >= EOF)
		return -EINVAL;

	for (a = 0; a < bus->addrs->ndescs; a++) {
		if ((addr ^ bus->current_addr) & (1 << a))	{ // address bit has really changed
// printk("%s: %d -> %d\n", __func__, a, (addr >> a) & 1);
			set_bus_bit(bus->addrs->desc[a], (addr >> a) & 1);
		}
	}

	bus->current_addr = addr;
	return 0;
}

// FIXME: force to switch direction of D lines to input? Or rely on write turning them back?

static inline int read_half(struct retrode3_bus *bus, int a0)
{ /* read data from data lines */
	int d;
	uint8_t data;
// printk("%s: a0=%d\n", __func__, a0);

	set_bus_bit(bus->oe, 0);	// this is the pin level although we have defined "active low" in the DTS

	/* read 8 data bits either on D0..D7 or D8..D15 */
	data = 0;
	if(a0)
		for (d = 0; d < bus->datas->ndescs-8; d++) {
			int bit = get_bus_bit(bus->datas->desc[d]);
// printk("%s: bit %d=%d\n", __func__, d, bit);
			if (bit < 0)
				return bit;
			data |= bit << d;
		}
	else
		for (d = 8; d < bus->datas->ndescs; d++) {
			int bit = get_bus_bit(bus->datas->desc[d]);
// printk("%s: bit %d=%d\n", __func__, d, bit);
			if (bit < 0)
				return bit;
			data |= bit << (d-8);
		}

	set_bus_bit(bus->oe, 1);
// printk("%s: data=%02x\n", __func__, data);

	return data;
}

static inline int read_byte(struct retrode3_bus *bus)
{ /* read data from data lines */
	return read_half(bus, bus->current_addr & 1);
}

static inline int read_word(struct retrode3_bus *bus)
{ /* read data from data lines */
	int d;
	uint16_t data;

// printk("%s:\n", __func__);

	set_bus_bit(bus->oe, 0);	// this is the pin level although we have "active low"

	/* read 16 data bits */
	data = 0;
	for (d = 0; d < bus->datas->ndescs; d++) {
		int bit = get_bus_bit(bus->datas->desc[d]);

// printk("bit=%d\n", bit);

		if (bit < 0)
			return bit;
		data |= bit << d;
	}

	set_bus_bit(bus->oe, 1);
// printk("%s: data=%02x\n", __func__, data);

	return data;
}

static inline void set_half(struct retrode3_bus *bus, uint8_t data, int a0)
{ // set D0..D7 or D8..D15
	int d;
	/* set data bits */

// printk("%s:\n", __func__);

	if(a0) { // D8..D15
		for (d = 8; d < bus->datas->ndescs; d++) {
			gpiod_direction_output(bus->datas->desc[d], (data>>(d-8)) & 1);
		}
	} else { // D0..D7
		for (d = 0; d < bus->datas->ndescs-8; d++) {
			gpiod_direction_output(bus->datas->desc[d], (data>>d) & 1);
		}
	}
}

static inline void drive_half(struct retrode3_bus *bus, uint8_t data, int a0)
{ // write D0..D7 or D8..D15 with WE
	set_half(bus, data, a0);
	/* pulse write enable for a0 */
	gpiod_set_value(bus->we->desc[a0], 1);
	gpiod_set_value(bus->we->desc[a0], 0);
}

static inline void end_drive_word(struct retrode3_bus *bus)
{ /* switch data bus back to input */
	int d;
	for (d = 0; d < bus->datas->ndescs; d++) {
		gpiod_direction_input(bus->datas->desc[d]);
	}
}

static inline void write_half(struct retrode3_bus *bus, uint8_t data, int a0)
{ // write D0..D7 or D8..D15 with WE and switch data bus back to input
	drive_half(bus, data, a0);
	end_drive_word(bus);
}

static inline void write_byte(struct retrode3_bus *bus, uint8_t data)
{ // use bit 0 of current_address
	return write_half(bus, data, bus->current_addr & 1);
}

static inline void set_word(struct retrode3_bus *bus, uint16_t data)	// D0..D15
{ /* set D0..D15 */
	int d;
	/* set data bits */

// printk("%s:\n", __func__);

	for (d = 0; d < bus->datas->ndescs; d++) {
		gpiod_direction_output(bus->datas->desc[d], (data>>d) & 1);
	}
}

static inline void drive_word(struct retrode3_bus *bus, uint16_t data)	// D0..D15
{ /* write data to data lines */
	set_word(bus, data);
	/* pulse both write enables */
	gpiod_set_value(bus->we->desc[0], 1);
	gpiod_set_value(bus->we->desc[1], 1);
	gpiod_set_value(bus->we->desc[0], 0);
	gpiod_set_value(bus->we->desc[1], 0);
}

static inline void write_word(struct retrode3_bus *bus, uint16_t data)	// D0..D15
{
	drive_word(bus, data);
	end_drive_word(bus);
}

#define CONFIG_RETRODE3_MDSLOT 294

#if NOT_HERE_OR_DIFFERENT

static int get_slot_power_mV(struct retrode3_slot *slot)
{
	if(!slot || IS_ERR_OR_NULL(slot->power))
		return -ENODEV;
	return gpiod_get_direction(slot->power) ? 3300 : 5000;
}

static int set_slot_power_mV(struct retrode3_slot *slot, int mV)
{
printk("%s: %dmV %px\n", __func__, mV, slot->power);

	if (IS_ERR_OR_NULL(slot->power))
		return -ENODEV;

	switch(mV) {
		case 5000:
			gpiod_direction_output(slot->power, 0);	// switch to output 0
			break;
		case 3300:
#if CONFIG_RETRODE3_MDSLOT == 293
			return -EINVAL;	// broken
#endif
			gpiod_direction_input(slot->power);	// switch to floating
			break;
		default:
printk("%s: unknown voltage %dmV\n", __func__, mV);
			return -EINVAL;
	}
	return 0;
}

// FIXME: move this into slot driver

static int is_selected(struct retrode3_slot *slot)
{
	// errors are treated as selected...
	return gpiod_get_value(slot->ce);
}

static void select_slot(struct retrode3_bus *bus, struct retrode3_slot *slot)
{ /* control chip select */
	int i;

// printk("%s:\n", __func__);

	if (slot) {
		if (is_selected(slot))
			return;	// already selected (and locked)
		mutex_lock(&bus->select_lock);	// lock until slot == 0
	}

	for(i=0; i<ARRAY_SIZE(bus->slots); i++) {
		if (!bus->slots[i])
			continue;	// avoid to match slot == NULL
		gpiod_set_value(bus->slots[i]->ce, (bus->slots[i] == slot) ? 1:0);
	}

	if (!slot && mutex_is_locked(&bus->select_lock))
		mutex_unlock(&bus->select_lock);
}

static void retrode3_remove(struct platform_device *pdev)
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
}
#endif

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
	struct retrode3_bus *bus = controller->priv;
	/* Write lower/upper 32bits */
	printk("%s %d:%08x\n", __func__, __LINE__, addr);
	return 0;
}

static int bus_xfer(struct retrode3_bus_controller *controller, u8 dir, void *buf, size_t len)
{
	struct retrode3_bus *bus = controller->priv;
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
	.xfer = bus_xfer,
};

static int retrode3_bus_probe(struct platform_device *pdev)
{
	struct retrode3_bus *bus;
	struct device_node *np;
	int ret;

	printk("%s %d\n", __func__, __LINE__);

	bus = devm_kzalloc(&pdev->dev, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;

	// printk("%s: a bus=%px\n", __func__, bus);

	bus->addrs = devm_gpiod_get_array(&pdev->dev, "addr", GPIOD_OUT_HIGH);
	// printk("%s: addrs=%px\n", __func__, bus->addrs);
	if (IS_ERR(bus->addrs))
		return PTR_ERR(bus->addrs);
	if (bus->addrs->ndescs != 24) {
		dev_err(&pdev->dev, "Invalid number of address gpios (%d != 24)\n",
				bus->addrs->ndescs);
		return -EINVAL;
	}
	// printk("%s: addrs 1\n", __func__);
	/* bring all address gpios in a defined state */
	bus->current_addr = EOF - 1;
	// printk("%s: addrs 2\n", __func__);
	set_address(bus, 0);
	// printk("%s: addrs 3\n", __func__);

	bus->datas = devm_gpiod_get_array(&pdev->dev, "data", GPIOD_IN);
	// printk("%s: datas=%px\n", __func__, bus->datas);
	if (IS_ERR(bus->datas))
		return PTR_ERR(bus-> datas);
	if (bus->datas->ndescs != 16) {
		dev_err(&pdev->dev, "Invalid number of data gpios (%d != 16)\n",
				bus->datas->ndescs);
		return -EINVAL;
	}

	bus->oe = devm_gpiod_get(&pdev->dev, "oe", GPIOD_OUT_HIGH);	// active LOW is XORed with DT definition
																// printk("%s: oe=%px\n", __func__, bus->oe);
	if (IS_ERR(bus->oe))
		return PTR_ERR(bus->oe);
	gpiod_set_value(bus->oe, 0);	// turn inactive

	bus->we = devm_gpiod_get_array(&pdev->dev, "we", GPIOD_OUT_HIGH);	// active LOW is XORed with DT definition
																		// printk("%s: we=%px\n", __func__, bus->we);
	if (IS_ERR(bus->we))
		return PTR_ERR(bus->we);
	if (bus->we->ndescs != 2) {
		dev_err(&pdev->dev, "Invalid number of we gpios (%d != 2)\n",
				bus->we->ndescs);
		return -EINVAL;
	}
	gpiod_set_value(bus->we->desc[0], 0);	// make both inactive
	gpiod_set_value(bus->we->desc[1], 0);

	bus->time = devm_gpiod_get(&pdev->dev, "time", GPIOD_OUT_HIGH);	// active LOW is XORed with DT definition
																	// printk("%s: time=%px\n", __func__, bus->time);
	if (IS_ERR(bus->time))
		return PTR_ERR(bus->time);
	gpiod_set_value(bus->time, 0);	// make inactive

	bus->reset = devm_gpiod_get(&pdev->dev, "reset", GPIOD_OUT_HIGH);	// active LOW is XORed with DT definition
																		// printk("%s: reset=%px\n", __func__, bus->reset);
	if (IS_ERR(bus->reset))
		return PTR_ERR(bus->reset);
	gpiod_set_value_cansleep(bus->reset, 0);	// make inactive

#if 0
	printk("%s: 1\n", __func__);
	printk("%s: bus=%px\n", __func__, bus);
	printk("%s: addrs=%px\n", __func__, bus->addrs);
	printk("%s: desc[0]=%px\n", __func__, bus->addrs->desc[0]);
	printk("%s: gdev=%px\n", __func__, bus->addrs->desc[0]->gdev);
	printk("%s: chip=%px\n", __func__, bus->addrs->desc[0]->gdev->chip);
#endif

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

	retrode3_bus_select_device(&bus->controller, NULL);	// deselect all slots

	mutex_init(&bus->select_lock);

	bus->pdev = pdev;
	bus->controller.dev = &pdev->dev;
	bus->controller.ops = &bus_ops;
	bus->controller.priv = bus;

	platform_set_drvdata(pdev, bus);

	/* register controller with retrode3 core */
	ret = retrode3_bus_register_controller(&bus->controller);
	if (ret) {
		dev_err(&pdev->dev, "failed to register retrode3 controller: %d\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "retrode3 bus controller registered\n");
	return 0;
}

static void retrode3_bus_remove(struct platform_device *pdev)
{
	struct retrode3_bus *bus = platform_get_drvdata(pdev);

	retrode3_bus_unregister_controller(&bus->controller);
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
