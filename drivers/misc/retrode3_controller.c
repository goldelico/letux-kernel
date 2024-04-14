// SPDX-License-Identifier: GPL-2.0

/*
 * driver for Retrode 3 game controller driver
 *
 *  Copyright (C) 2022-23, H. Nikolaus Schaller
 *
 * FIXME: make this an independent driver module
 */

#if FIXME
#include "../gpio/gpiolib.h"
#include "retrode3_bus.h"

#ifdef CONFIG_IA64
# include <linux/efi.h>
#endif
#endif

/*
 * game controller driver
 */

#define POLL_RATE	msecs_to_jiffies(20)	/* in ms */

static void retrode3_polling_work(struct work_struct *work)
{
	struct retrode3_slot *slot = container_of(work, struct retrode3_slot, work.work);
	int i;
	int word;	// left and right controllers and two mux states combined

	select_slot(slot->bus, slot);

	set_address(slot->bus, slot->bus->current_addr & ~BIT(22));	// clear A22 (select MUX)
	word = read_word(slot->bus);	// full 16 bits (D0..D15) for both channels
	if(word < 0) { // invalid read
		select_slot(slot->bus, NULL);
		return;
	}
	set_address(slot->bus, slot->bus->current_addr | BIT(22));	// set A22 (select MUX)
	word |= read_word(slot->bus) << 16;	// no error handling
	select_slot(slot->bus, NULL);

// printk("%s: word %08x\n", __func__, word);

#define GENESIS_CD1	BIT(0)		// pin 2 / D0 (select = 0) - connect detect (0 if connected)
#define GENESIS_R	BIT(0+16)	// pin 2 / D0 (select = 1)
#define GENESIS_CD2	BIT(1)		// pin 3 / D1 (select = 0) - connect detect (0 if connected)
#define GENESIS_L	BIT(1+16)	// pin 3 / D1 (select = 1)
#define GENESIS_D	BIT(2)		// pin 4 / D2 (independent of select)
#define GENESIS_U	BIT(3)		// pin 5 / D3 (independent of select)
#define GENESIS_A	BIT(4)		// pin 9 / D4 (select = 0)
#define GENESIS_B	BIT(4+16)	// pin 9 / D4 (select = 1)
#define GENESIS_S	BIT(5)		// pin 6 / D5 (select = 0)
#define GENESIS_C	BIT(5+16)	// pin 6 / D5 (select = 1)

	for (i=0; i < 2; i++) {
		struct retrode3_controller *c = &slot->controllers[i];
		int state = (word >> c->data_offset) & 0x00ff00ff;

		if (c->state_valid) { // skip first analysis after boot
			u32 changes = state ^ c->last_state;

if (changes) printk("%s: controller %d changes %08x state %08x\n", __func__, i, changes, state);

			if (changes & GENESIS_CD1) { // controller has been (un)plugged
				char *envp[4];

				envp[0] = kasprintf(GFP_KERNEL, "SLOT=%s", dev_name(&slot->dev));
				envp[1] = kasprintf(GFP_KERNEL, "CHANNEL=%d", i);
				envp[2] = kasprintf(GFP_KERNEL, "STATE=%s", (state & GENESIS_CD1)?"disconnected":"connected");
				envp[3] = NULL;
printk("%s: %s %s %s\n", __func__, envp[0], envp[1], envp[2]);
				// check with: udevadm monitor --environment
				kobject_uevent_env(&slot->dev.kobj, KOBJ_CHANGE, envp);
			}

#define GENESIS_KEY(MASK, KEY) if (changes & (MASK)) input_report_key(c->input, (KEY), !(state & (MASK)));

			GENESIS_KEY(GENESIS_U, KEY_U);
			GENESIS_KEY(GENESIS_D, KEY_D);
			GENESIS_KEY(GENESIS_L, KEY_L);
			GENESIS_KEY(GENESIS_R, KEY_R);
			GENESIS_KEY(GENESIS_A, KEY_A);
			GENESIS_KEY(GENESIS_B, KEY_B);
			GENESIS_KEY(GENESIS_C, KEY_C);
			GENESIS_KEY(GENESIS_S, KEY_ENTER);

			input_sync(c->input);
		}

		c->last_state = state;
		c->state_valid = true;
	}

	schedule_delayed_work(&slot->work, POLL_RATE);	// start next check
}

int retrode3_probe_controller(struct retrode3_slot *slot, struct device_node*child)
{
	struct device *	dev = &slot->dev;
	struct device_node *controller = NULL;
	int ret;
	int id;

	device_initialize(dev);
	dev->class = retrode3_class;
	dev_set_name(dev, "gamecontroller");
	dev->of_node = child;

	ret = device_add(dev);
	if (ret)
		return ret;

	slot->ce = devm_gpiod_get(dev, "ce", GPIOD_OUT_HIGH);	// active LOW is XORed with DT definition
	gpiod_set_value(slot->ce, 0);	// turn inactive

	id = 0;
	while ((controller = of_get_next_child(child, controller))) {
		struct input_dev *input_dev;

		dev_info(&slot->dev, "add game controller %d\n", id);

		input_dev = devm_input_allocate_device(dev);
		if (!input_dev)
			return -ENOMEM;

		slot->controllers[id].input = input_dev;

		input_dev->name = kasprintf(GFP_KERNEL, "Retrode 3 Game Controller %d", id);
		input_dev->phys = kasprintf(GFP_KERNEL, "%s/input%d", dev_name(dev), id);

		input_dev->id.bustype = BUS_GAMEPORT;

		input_set_capability(input_dev, EV_KEY, KEY_A);
		input_set_capability(input_dev, EV_KEY, KEY_B);
		input_set_capability(input_dev, EV_KEY, KEY_C);
		input_set_capability(input_dev, EV_KEY, KEY_U);
		input_set_capability(input_dev, EV_KEY, KEY_D);
		input_set_capability(input_dev, EV_KEY, KEY_L);
		input_set_capability(input_dev, EV_KEY, KEY_R);
		input_set_capability(input_dev, EV_KEY, KEY_ENTER);

		ret = input_register_device(input_dev);
		if (ret) {
			dev_err(dev, "Failed to register input device: %d\n", ret);
			return ret;
		}

		of_property_read_u32(controller, "data-offset", &slot->controllers[id].data_offset);

		id++;
	}

#if 0
	INIT_DELAYED_WORK(&slot->work, retrode3_polling_work);
	schedule_delayed_work(&slot->work, POLL_RATE);	// start polling
#endif

	return 0;
}

#if FIXME

static const struct retrode3_device_id retrode3_controller_idtable[] = {
	{ "retrode3-controller", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, retrode3_controller_idtable);

static const struct of_device_id retrode3_controller_of_match[] = {
	{ .compatible = "openpandora,retrode3-controller" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, retrode3_controller_of_match);

static struct retrode3_driver retrode3_controller_driver = {
	.driver = {
		.name	= "retrode3-controller",
		.of_match_table = retrode3_controller_of_match,
	},
	.id_table	= retrode3_controller_idtable,
	.probe		= retrode3_probe_controller,
};

module_retrode3_driver(retrode3_controller_driver);

MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("Retrode3 Game Controller Driver");
MODULE_LICENSE("GPL");

#endif
