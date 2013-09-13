/*
 * TI Multiplexer Clock
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

/**
 * of_mux_clk_setup - Setup function for simple mux rate clock
 * @node: DT node for the clock
 *
 * Sets up a basic clock multiplexer.
 */
static void of_mux_clk_setup(struct device_node *node)
{
	struct clk *clk;
	const char **parent_names;
	int i;
	struct clk_mux_desc *desc;
	u32 val;

	desc = kzalloc(sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return;

	desc->desc.num_parents = of_clk_get_parent_count(node);
	if (desc->desc.num_parents < 2) {
		pr_err("mux-clock %s must have parents\n", node->name);
		goto cleanup;
	}
	parent_names = kzalloc((sizeof(char *) * desc->desc.num_parents),
			       GFP_KERNEL);
	if (!parent_names)
		goto cleanup;

	for (i = 0; i < desc->desc.num_parents; i++)
		parent_names[i] = of_clk_get_parent_name(node, i);

	desc->desc.parent_names = parent_names;
	desc->desc.name = node->name;
	desc->desc.register_func = clk_register_mux_desc;

	desc->reg = ti_clk_get_reg_addr(node, 0);

	if (!desc->reg)
		goto cleanup;

	desc->ll_ops = ti_clk_ll_ops;

	if (!of_property_read_u32(node, "ti,bit-shift", &val))
		desc->shift = val;

	if (of_property_read_bool(node, "ti,index-starts-at-one"))
		desc->flags |= CLK_MUX_INDEX_ONE;

	if (of_property_read_bool(node, "ti,set-rate-parent"))
		desc->desc.flags |= CLK_SET_RATE_PARENT;

	/* Generate bit-mask based on parent info */
	desc->mask = desc->desc.num_parents;
	if (!(desc->flags & CLK_MUX_INDEX_ONE))
		desc->mask--;

	desc->mask = (1 << fls(desc->mask)) - 1;

	clk = clk_register_desc(NULL, &desc->desc);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);

cleanup:
	kfree(desc->desc.parent_names);
	kfree(desc);
}
CLK_OF_DECLARE(mux_clk, "ti,mux-clock", of_mux_clk_setup);

static void __init of_ti_composite_mux_clk_setup(struct device_node *node)
{
	struct clk_mux *mux;
	int num_parents;
	u32 val;

	mux = kzalloc(sizeof(*mux), GFP_KERNEL);
	if (!mux)
		return;

	mux->reg = ti_clk_get_reg_addr(node, 0);

	if (!mux->reg)
		goto cleanup;

	mux->ll_ops = ti_clk_ll_ops;

	if (!of_property_read_u32(node, "ti,bit-shift", &val))
		mux->shift = val;

	if (of_property_read_bool(node, "ti,index-starts-at-one"))
		mux->flags |= CLK_MUX_INDEX_ONE;

	num_parents = of_clk_get_parent_count(node);

	if (num_parents < 2) {
		pr_err("%s must have parents\n", node->name);
		goto cleanup;
	}

	mux->mask = num_parents - 1;
	mux->mask = (1 << fls(mux->mask)) - 1;

	if (!ti_clk_add_component(node, &mux->hw, CLK_COMPONENT_TYPE_MUX))
		return;

cleanup:
	kfree(mux);
}
CLK_OF_DECLARE(ti_composite_mux_clk_setup, "ti,composite-mux-clock",
	       of_ti_composite_mux_clk_setup);
