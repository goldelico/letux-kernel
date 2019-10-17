// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2019 Andreas Kemnade
 */
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include <linux/mfd/rn5t618.h>

static const struct regmap_irq rc5t619_irqs[] = {
	[RN5T618_IRQ_SYS] = {
		.reg_offset = 0,
		.mask = (0 << 1)
	},
	[RN5T618_IRQ_DCDC] = {
		.reg_offset = 0,
		.mask = (1 << 1)
	},
	[RN5T618_IRQ_RTC]  = {
		.reg_offset = 0,
		.mask = (1 << 2)
	},
	[RN5T618_IRQ_ADC] = {
		.reg_offset = 0,
		.mask = (1 << 3)
	},
	[RN5T618_IRQ_GPIO] = {
		.reg_offset = 0,
		.mask = (1 << 4)
	},
	[RN5T618_IRQ_CHG] = {
		.reg_offset = 0,
		.mask = (1 << 6),
	}
};

static const struct regmap_irq_chip rc5t619_irq_chip = {
	.name = "rc5t619",
	.irqs = rc5t619_irqs,
	.num_irqs = ARRAY_SIZE(rc5t619_irqs),
	.num_regs = 1,
	.status_base = RN5T618_INTMON,
	.mask_base = RN5T618_INTEN,
	.mask_invert = true,
};

int rn5t618_irq_init(struct rn5t618 *rn5t618)
{
	const struct regmap_irq_chip *irq_chip;
	int ret;

	if (!rn5t618->chip_irq)
		return 0;

	switch (rn5t618->variant) {
	case RC5T619:
		irq_chip = &rc5t619_irq_chip;
		break;

		/* TODO: check irq definitions for other variants */

	default:
		irq_chip = NULL;
		break;
	}

	if (!irq_chip) {
		dev_err(rn5t618->dev, "no IRQ definition known for variant\n");
		return -ENOENT;
	}

	ret = devm_regmap_add_irq_chip(rn5t618->dev, rn5t618->regmap,
				       rn5t618->chip_irq,
				       IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				       0, irq_chip, &rn5t618->irq_data);
	if (ret != 0) {
		dev_err(rn5t618->dev, "Failed to register IRQ chip\n");
		return ret;
	}

	return 0;
}
