/*
 * TI Divider Clock
 *
 * Copyright (C) 2013 Texas Instruments, Inc.
 *
 * Tero Kristo <t-kristo@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/clk/ti.h>

#undef pr_fmt
#define pr_fmt(fmt) "%s: " fmt, __func__

static struct clk_div_table
__init *ti_clk_get_div_table(struct device_node *node)
{
	struct clk_div_table *table;
	const __be32 *divspec;
	u32 val;
	u32 num_div;
	u32 valid_div;
	int i;

	divspec = of_get_property(node, "ti,dividers", &num_div);

	if (!divspec)
		return NULL;

	num_div /= 4;

	valid_div = 0;

	/* Determine required size for divider table */
	for (i = 0; i < num_div; i++) {
		of_property_read_u32_index(node, "ti,dividers", i, &val);
		if (val)
			valid_div++;
	}

	if (!valid_div) {
		pr_err("no valid dividers for %s table\n", node->name);
		return ERR_PTR(-EINVAL);
	}

	table = kzalloc(sizeof(*table) * (valid_div + 1), GFP_KERNEL);

	if (!table)
		return ERR_PTR(-ENOMEM);

	valid_div = 0;

	for (i = 0; i < num_div; i++) {
		of_property_read_u32_index(node, "ti,dividers", i, &val);
		if (val) {
			table[valid_div].div = val;
			table[valid_div].val = i;
			valid_div++;
		}
	}

	return table;
}

static int _get_divider_width(struct device_node *node,
			      const struct clk_div_table *table,
			      u8 flags)
{
	u32 min_div;
	u32 max_div;
	u32 val = 0;
	u32 div;

	if (!table) {
		/* Clk divider table not provided, determine min/max divs */
		if (of_property_read_u32(node, "ti,min-div", &min_div))
			min_div = 1;

		if (of_property_read_u32(node, "ti,max-div", &max_div)) {
			pr_err("no max-div for %s!\n", node->name);
			return -EINVAL;
		}

		/* Determine bit width for the field */
		if (flags & CLK_DIVIDER_ONE_BASED)
			val = 1;

		div = min_div;

		while (div < max_div) {
			if (flags & CLK_DIVIDER_POWER_OF_TWO)
				div <<= 1;
			else
				div++;
			val++;
		}
	} else {
		div = 0;

		while (table[div].div) {
			val = table[div].val;
			div++;
		}
	}

	return fls(val);
}

static int __init ti_clk_divider_populate(struct device_node *node,
	void __iomem **reg, const struct clk_div_table **table,
	unsigned long *flags, u8 *div_flags, u8 *width, u8 *shift)
{
	u32 val;

	*reg = ti_clk_get_reg_addr(node, 0);
	if (!*reg)
		return -EINVAL;

	if (!of_property_read_u32(node, "ti,bit-shift", &val))
		*shift = val;
	else
		*shift = 0;

	*flags = 0;
	*div_flags = 0;

	if (of_property_read_bool(node, "ti,index-starts-at-one"))
		*div_flags |= CLK_DIVIDER_ONE_BASED;

	if (of_property_read_bool(node, "ti,index-power-of-two"))
		*div_flags |= CLK_DIVIDER_POWER_OF_TWO;

	if (of_property_read_bool(node, "ti,set-rate-parent"))
		*flags |= CLK_SET_RATE_PARENT;

	*table = ti_clk_get_div_table(node);

	if (IS_ERR(*table))
		return PTR_ERR(*table);

	*width = _get_divider_width(node, *table, *div_flags);

	return 0;
}

/**
 * of_ti_divider_clk_setup - Setup function for simple div rate clock
 * @node: device node for this clock
 *
 * Sets up a basic divider clock.
 */
static void __init of_ti_divider_clk_setup(struct device_node *node)
{
	struct clk *clk;
	struct clk_divider_desc *desc;
	const char *parent_name;

	desc = kzalloc(sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return;

	parent_name = of_clk_get_parent_name(node, 0);
	desc->desc.parent_names = &parent_name;
	desc->desc.num_parents = 1;
	desc->desc.register_func = clk_register_divider_desc;
	desc->desc.name = node->name;
	desc->ll_ops = ti_clk_ll_ops;

	if (ti_clk_divider_populate(node, &desc->reg, &desc->table,
				    &desc->desc.flags, &desc->flags,
				    &desc->width, &desc->shift) < 0)
		goto cleanup;

	clk = clk_register_desc(NULL, &desc->desc);

	if (!IS_ERR(clk)) {
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
		of_ti_clk_autoidle_setup(node);
		kfree(desc);
		return;
	}

cleanup:
	kfree(desc->table);
	kfree(desc);
}
CLK_OF_DECLARE(divider_clk, "ti,divider-clock", of_ti_divider_clk_setup);

static void __init of_ti_composite_divider_clk_setup(struct device_node *node)
{
	struct clk_divider *div;
	unsigned long val;

	div = kzalloc(sizeof(*div), GFP_KERNEL);
	if (!div)
		return;

	div->ll_ops = ti_clk_ll_ops;

	if (ti_clk_divider_populate(node, &div->reg, &div->table, &val,
				    &div->flags, &div->width, &div->shift) < 0)
		goto cleanup;

	if (!ti_clk_add_component(node, &div->hw, CLK_COMPONENT_TYPE_DIVIDER))
		return;

cleanup:
	kfree(div->table);
	kfree(div);
}
CLK_OF_DECLARE(ti_composite_divider_clk, "ti,composite-divider-clock",
	       of_ti_composite_divider_clk_setup);
