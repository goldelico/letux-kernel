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
#include <linux/efi.h>
#endif
#endif

#include <linux/delay.h>

/*
 * game controller driver
 */

#define POLL_RATE	msecs_to_jiffies(20)	/* in ms */

static inline void retrode3_set_select(struct retrode3_slot *slot, int state)
{
	set_address(slot->bus, state ? (slot->bus->current_addr | BIT(22)) : (slot->bus->current_addr & ~BIT(22)));	// set/clear A22 (select MUX)
// printk("slot addr = %08x\n", slot->bus->current_addr);
}

static void retrode3_polling_work(struct work_struct *work)
{
	struct retrode3_slot *slot = container_of(work, struct retrode3_slot, work.work);
	int i;

	int cycle;
	int w;
	u64 word[2] = { 0, 0 };	// left and right controllers

	select_slot(slot->bus, slot);

// if 3 button controller detected, we do not need 8 cycles
	for (cycle = 0; cycle < 8; cycle++) {
		retrode3_set_select(slot, cycle%2);
		w = read_word(slot->bus);	// D0..D5 are Controller 1 and D8..D13 Controller 2
		if(w < 0) { // invalid read
			select_slot(slot->bus, NULL);
			return;
		}
		for (i=0; i<2; i++)
			word[i] = (word[i] << 8) | ((w >> slot->controllers[i].data_offset) & 0x3f);
	}
// printk("%s: %llx %llx\n", __func__, word[0], word[1]);

	select_slot(slot->bus, NULL);


// see: https://nfggames.com/forum2/index.php?topic=2266.0
/* see https://sites.ualberta.ca/~delliott/cmpe490/appnotes/2016w/g6_genesis_controllers/genesis_controller.pdf */

/* there is a 6 button variant where TH cycles through more banks: https://huguesjohnson.com/programming/genesis/6button/ */
/* NOTE: he uses a mirrored pin numbering scheme */

	for (i=0; i < 2; i++) {
		struct retrode3_controller *c = &slot->controllers[i];
		u64 state = word[i];

		if (c->state_valid) { // skip first analysis after boot
			u64 changes = state ^ c->last_state;

if (changes) printk("%s: controller %d changes %16llx state %16llx\n", __func__, i, changes, state);
			if (changes & BIT_ULL(56)) { // controller has been (un)plugged
				char *envp[4];

				envp[0] = kasprintf(GFP_KERNEL, "SLOT=%s", dev_name(&slot->dev));
				envp[1] = kasprintf(GFP_KERNEL, "CHANNEL=%d", i);
				envp[2] = kasprintf(GFP_KERNEL, "STATE=%s", (state & BIT_ULL(56))?"disconnected":"connected");
				envp[3] = NULL;
printk("%s: %s %s %s\n", __func__, envp[0], envp[1], envp[2]);
				// check with: udevadm monitor --environment
				kobject_uevent_env(&slot->dev.kobj, KOBJ_CHANGE, envp);
			}

#define SEND_CHANGE(BIT, KEY) if (changes & BIT_ULL(BIT)) input_report_key(c->input, KEY, !(state & BIT_ULL(BIT)));
			SEND_CHANGE(59, KEY_U);
			SEND_CHANGE(58, KEY_D);
			SEND_CHANGE(48, KEY_R);
			SEND_CHANGE(49, KEY_L);
			SEND_CHANGE(60, KEY_A);
			SEND_CHANGE(52, KEY_B);
			SEND_CHANGE(53, KEY_C);
			SEND_CHANGE(61, KEY_S);
			if(state & BIT_ULL(8)) { // 6 button
				SEND_CHANGE(18, KEY_X);
				SEND_CHANGE(19, KEY_Y);
				SEND_CHANGE(20, KEY_Z);
				SEND_CHANGE(16, KEY_M);
			}
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
		input_set_capability(input_dev, EV_KEY, KEY_S);
// FIXME: add only after detecting a 6 button controller?
		input_set_capability(input_dev, EV_KEY, KEY_X);
		input_set_capability(input_dev, EV_KEY, KEY_Y);
		input_set_capability(input_dev, EV_KEY, KEY_Z);
		input_set_capability(input_dev, EV_KEY, KEY_M);

		ret = input_register_device(input_dev);
		if (ret) {
			dev_err(dev, "Failed to register input device: %d\n", ret);
			return ret;
		}

		of_property_read_u32(controller, "data-offset", &slot->controllers[id].data_offset);

		id++;
	}

#if 1
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
