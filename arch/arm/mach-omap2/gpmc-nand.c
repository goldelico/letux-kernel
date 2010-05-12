/*
 * gpmc-nand.c
 *
 * Copyright (C) 2009 Texas Instruments
 * Vimal Singh <vimalsingh@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <asm/mach/flash.h>

#include <plat/nand.h>
#include <plat/board.h>
#include <plat/gpmc.h>

#define WR_RD_PIN_MONITORING	0x00600000

static struct omap_nand_platform_data *gpmc_nand_data;

static struct resource gpmc_nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device gpmc_nand_device = {
	.name		= "omap2-nand",
	.id		= 0,
	.num_resources	= 1,
	.resource	= &gpmc_nand_resource,
};

static int omap2_nand_gpmc_config(int cs, void __iomem *nand_base)
{
	struct gpmc_timings t;
	int err;

	const int cs_rd_off = 36;
	const int cs_wr_off = 36;
	const int adv_on = 6;
	const int adv_rd_off = 24;
	const int adv_wr_off = 36;
	const int oe_off = 48;
	const int we_off = 30;
	const int rd_cycle = 72;
	const int wr_cycle = 72;
	const int access = 54;
	const int wr_data_mux_bus = 0;
	const int wr_access = 30;

	memset(&t, 0, sizeof(t));
	t.sync_clk = 0;
	t.cs_on = 0;
	t.adv_on = gpmc_round_ns_to_ticks(adv_on);

	/* Read */
	t.adv_rd_off = gpmc_round_ns_to_ticks(adv_rd_off);
	t.oe_on  = t.adv_on;
	t.access = gpmc_round_ns_to_ticks(access);
	t.oe_off = gpmc_round_ns_to_ticks(oe_off);
	t.cs_rd_off = gpmc_round_ns_to_ticks(cs_rd_off);
	t.rd_cycle  = gpmc_round_ns_to_ticks(rd_cycle);

	/* Write */
	t.adv_wr_off = gpmc_round_ns_to_ticks(adv_wr_off);
	t.we_on  = t.oe_on;
	if (cpu_is_omap34xx()) {
		t.wr_data_mux_bus = gpmc_round_ns_to_ticks(wr_data_mux_bus);
		t.wr_access = gpmc_round_ns_to_ticks(wr_access);
	}
	t.we_off = gpmc_round_ns_to_ticks(we_off);
	t.cs_wr_off = gpmc_round_ns_to_ticks(cs_wr_off);
	t.wr_cycle  = gpmc_round_ns_to_ticks(wr_cycle);

	/* Configure GPMC */
	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG1,
			GPMC_CONFIG1_DEVICESIZE(gpmc_nand_data->devsize) |
			GPMC_CONFIG1_DEVICETYPE_NAND);

	err = gpmc_cs_set_timings(cs, &t);
	if (err)
		return err;

	return 0;
}

static int gpmc_nand_setup(void __iomem *nand_base)
{
	struct device *dev = &gpmc_nand_device.dev;

	/* Set timings in GPMC */
	if (omap2_nand_gpmc_config(gpmc_nand_data->cs, nand_base) < 0) {
		dev_err(dev, "Unable to set gpmc timings\n");
		return -EINVAL;
	}

	return 0;
}

int __init gpmc_nand_init(struct omap_nand_platform_data *_nand_data)
{
	unsigned int val;
	int err	= 0;
	struct device *dev = &gpmc_nand_device.dev;

	gpmc_nand_data = _nand_data;
	gpmc_nand_data->nand_setup = gpmc_nand_setup;
	gpmc_nand_device.dev.platform_data = gpmc_nand_data;

	err = gpmc_nand_setup(gpmc_nand_data->gpmc_cs_baseaddr);
	if (err < 0) {
		dev_err(dev, "NAND platform setup failed: %d\n", err);
		return err;
	}

	/* Enable RD PIN Monitoring Reg */
	if (gpmc_nand_data->dev_ready) {
		val  = gpmc_cs_read_reg(gpmc_nand_data->cs,
						 GPMC_CS_CONFIG1);
		val |= WR_RD_PIN_MONITORING;
		gpmc_cs_write_reg(gpmc_nand_data->cs,
						GPMC_CS_CONFIG1, val);
	}

	err = platform_device_register(&gpmc_nand_device);
	if (err < 0) {
		dev_err(dev, "Unable to register NAND device\n");
		return err;
	}

	return 0;
}
