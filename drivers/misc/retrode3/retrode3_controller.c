// SPDX-License-Identifier: GPL-2.0

/*
 * driver for Retrode 3 game controller driver
 *
 *  Copyright (C) 2022-25, H. Nikolaus Schaller
 *
 * FIXME: make this an independent driver module
 */

#include "retrode3_bus.h"
#include <linux/delay.h>

/*
 * game controller driver for SEGA and SNES controllers
 */

#define POLL_RATE	msecs_to_jiffies(1000/60)	/* 60 Hz should be 16.67 for SNES */

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
	u64 word[4] = { 0, 0, 0, 0 };	// left and right controllers

	select_slot(slot->bus, slot);

// FIXME: if 3 button controller detected, we do not need 8 cycles
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

	/* SEGA controller
	 * see: https://nfggames.com/forum2/index.php?topic=2266.0
	 * see https://sites.ualberta.ca/~delliott/cmpe490/appnotes/2016w/g6_genesis_controllers/genesis_controller.pdf
	 *
	 * there is a 6 button variant where TH cycles through more banks: https://huguesjohnson.com/programming/genesis/6button/
	 * NOTE: he uses a mirrored pin numbering scheme
	 *
	 * A22: 		Clock
	 * D0-D6/D8-D13:	Data
	 */

	for (i=0; i < 2; i++) {
		struct retrode3_controller *c = &slot->controllers[i];
		u64 state = word[i];
		u64 changes = state ^ c->last_state;

// if (changes) printk("%s: controller %d changes %16llx state %16llx\n", __func__, i, changes, state);

		if (!c->state_valid || (changes & BIT_ULL(56))) { // first event after boot or controller has been (un)plugged
			char *envp[4];

			envp[0] = kasprintf(GFP_KERNEL, "SLOT=%s", dev_name(&slot->dev));
			envp[1] = kasprintf(GFP_KERNEL, "CHANNEL=%d", i);
			envp[2] = kasprintf(GFP_KERNEL, "STATE=%s", (state & BIT_ULL(56))?"disconnected":"connected");
			envp[3] = NULL;
// printk("%s: %s %s %s\n", __func__, envp[0], envp[1], envp[2]);
			// check with: udevadm monitor --environment
			kobject_uevent_env(&slot->dev.kobj, KOBJ_CHANGE, envp);
		}

#define SEND_CHANGE(BIT, KEY) if (changes & BIT_ULL(BIT)) input_report_key(c->input, KEY, !(state & BIT_ULL(BIT)));

		if (c->state_valid) { // skip first analysis after boot
			/* NOTE: in fast key press actions more than a single key status may have changed */
			SEND_CHANGE(59, KEY_UP);
			SEND_CHANGE(58, KEY_DOWN);
			SEND_CHANGE(48, KEY_RIGHT);
			SEND_CHANGE(49, KEY_LEFT);
			SEND_CHANGE(60, KEY_A);
			SEND_CHANGE(52, KEY_B);
			SEND_CHANGE(53, KEY_C);
			SEND_CHANGE(61, KEY_S);
			if(state & BIT_ULL(8)) { // 6 button
				SEND_CHANGE(17, KEY_X);
				SEND_CHANGE(18, KEY_Y);
				SEND_CHANGE(19, KEY_Z);
				SEND_CHANGE(16, KEY_M);
			}
			input_sync(c->input);
		}

		c->last_state = state;
		c->state_valid = true;
	}

	/* SNES controller
	 * based on description at https://gamefaqs.gamespot.com/snes/916396-super-nintendo/faqs/5395
	 *
	 * A21:			Latch
	 * A22:			Clock
	 * D7/D15:		Data (button pressed = 0)
	 * 			unplugged makes all data read 0
	 */

	retrode3_set_select(slot, 1);	// select slot and set clock high
	set_address(slot->bus, (slot->bus->current_addr | BIT(21)));	// latch pulse
	// add usleep_range(12, 20); or rely on that setting a gpio takes "enough" time?
	set_address(slot->bus, (slot->bus->current_addr & ~BIT(21)));	// end latch pulse

	// it may be possible to merge this with the above loop for sega
	// but there are differences: 16 clocks vs. 8 cycles
	// sampling is on one edge only - but that would simply multiply the bit numbers by 2
	// but makes it more difficult (&0x5555) to detect all buttons are low

	for (cycle = 0; cycle < 16; cycle ++) { // 16 clocks
		retrode3_set_select(slot, 0);
		w = read_word(slot->bus);	// D7 and D15 are relevant
// printk("%s %d: %x %llx %llx\n", __func__, line, w, word[2], word[3]);
		if(w < 0) { // invalid read
			select_slot(slot->bus, NULL);
			return;
		}
		for (i = 2; i < 4; i++) {
			word[i] = (word[i] << 1) | ((w >> slot->controllers[i].data_offset) & 0x1);
// printk("%s %d: %d %x %d %llx\n", __func__, __LINE__, i, w, ((w >> slot->controllers[i].data_offset) & 0x1), word[i]);
		retrode3_set_select(slot, 1);
		}
	}

// printk("%s: %llx %llx\n", __func__, word[2], word[3]);

	// this may also be merged into a single loop
	// only the SEND_CHANGE calls are then controller dependent

	for (i=2; i < 4; i++) {
		struct retrode3_controller *c = &slot->controllers[i];
		u64 state = word[i];
		u64 changes;

		if (state == 0)	/* there is a pull-down resistor making all bits read as 0 if unplugged */
			state |= BIT_ULL(56) | 0xffffULL;	/* assume no button is pressed now */

		changes = state ^ c->last_state;

// if (changes) printk("%s: controller %d changes %16llx state %16llx valid=%d\n", __func__, i, changes, state, c->state_valid);

		if (!c->state_valid || (changes & BIT_ULL(56))) { // first event after boot or controller has been (un)plugged
			char *envp[4];

			envp[0] = kasprintf(GFP_KERNEL, "SLOT=%s", dev_name(&slot->dev));
			envp[1] = kasprintf(GFP_KERNEL, "CHANNEL=%d", i);
			envp[2] = kasprintf(GFP_KERNEL, "STATE=%s", (state & BIT_ULL(56))?"disconnected":"connected");
			envp[3] = NULL;
// printk("%s: %s %s %s\n", __func__, envp[0], envp[1], envp[2]);
			// check with: udevadm monitor --environment
			kobject_uevent_env(&slot->dev.kobj, KOBJ_CHANGE, envp);
		}

		if (c->state_valid) { // skip first analysis after boot or plugging
			/* NOTE: in fast key press actions more than a single key status may have changed */
			SEND_CHANGE(15, KEY_B);	// B
			SEND_CHANGE(14, KEY_Y);	// Y
			SEND_CHANGE(13, KEY_C);	// Select
			SEND_CHANGE(12, KEY_S);	// Start
			SEND_CHANGE(11, KEY_UP);	// U on joypad
			SEND_CHANGE(10, KEY_DOWN);	// D on joypad
			SEND_CHANGE(9, KEY_LEFT);	// L on joypad
			SEND_CHANGE(8, KEY_RIGHT);	// R on joypad
			SEND_CHANGE(7, KEY_A);	// A
			SEND_CHANGE(6, KEY_X);	// X
			SEND_CHANGE(5, KEY_L);	// L shoulder
			SEND_CHANGE(4, KEY_R);	// R shoulder
			// 4 more bits
			input_sync(c->input);
		}

		c->last_state = state;
		c->state_valid = true;
	}

	select_slot(slot->bus, NULL);

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
	dev->release = NULL;
	dev_set_drvdata(dev, slot);
	dev_set_name(dev, "gamecontroller");
	dev->of_node = child;

	ret = device_add(dev);
	if (ret)
		return ret;

	slot->ce = devm_gpiod_get(dev, "ce", GPIOD_OUT_HIGH);	// active LOW is XORed with DT definition
	gpiod_set_value(slot->ce, 0);	// turn inactive

	id = 0;
	while (id < 4 && (controller = of_get_next_child(child, controller))) {
		struct input_dev *input_dev;

		dev_info(&slot->dev, "add game controller %d\n", id);

		input_dev = devm_input_allocate_device(dev);
		if (!input_dev)
			return -ENOMEM;

		slot->controllers[id].input = input_dev;

		input_dev->name = kasprintf(GFP_KERNEL, "Retrode 3 Game Controller %d", id);
		input_dev->phys = kasprintf(GFP_KERNEL, "%s/input%d", dev_name(dev), id);

		input_dev->id.bustype = BUS_GAMEPORT;

		if (id < 2) { // sega
			input_set_capability(input_dev, EV_KEY, KEY_A);
			input_set_capability(input_dev, EV_KEY, KEY_B);
			input_set_capability(input_dev, EV_KEY, KEY_C);
			input_set_capability(input_dev, EV_KEY, KEY_UP);
			input_set_capability(input_dev, EV_KEY, KEY_DOWN);
			input_set_capability(input_dev, EV_KEY, KEY_LEFT);
			input_set_capability(input_dev, EV_KEY, KEY_RIGHT);
			input_set_capability(input_dev, EV_KEY, KEY_S);
// FIXME: dynamically add only after detecting a 6 button controller?
			input_set_capability(input_dev, EV_KEY, KEY_X);
			input_set_capability(input_dev, EV_KEY, KEY_Y);
			input_set_capability(input_dev, EV_KEY, KEY_Z);
			input_set_capability(input_dev, EV_KEY, KEY_M);
		} else { // snes
			input_set_capability(input_dev, EV_KEY, KEY_A);
			input_set_capability(input_dev, EV_KEY, KEY_B);
			input_set_capability(input_dev, EV_KEY, KEY_X);
			input_set_capability(input_dev, EV_KEY, KEY_Y);
			input_set_capability(input_dev, EV_KEY, KEY_S);
			input_set_capability(input_dev, EV_KEY, KEY_C);
			input_set_capability(input_dev, EV_KEY, KEY_UP);
			input_set_capability(input_dev, EV_KEY, KEY_DOWN);
			input_set_capability(input_dev, EV_KEY, KEY_LEFT);
			input_set_capability(input_dev, EV_KEY, KEY_RIGHT);
			input_set_capability(input_dev, EV_KEY, KEY_L);
			input_set_capability(input_dev, EV_KEY, KEY_R);
		}

		ret = input_register_device(input_dev);
		if (ret) {
			dev_err(dev, "Failed to register input device: %d\n", ret);
			return ret;
		}

		of_property_read_u32(controller, "data-offset", &slot->controllers[id].data_offset);

// printk("%s %d: %d %d\n", __func__, __LINE__, id, slot->controllers[id].data_offset);

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
