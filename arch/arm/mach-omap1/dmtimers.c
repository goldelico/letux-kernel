/**
 * linux/arch/arm/mach-omap1/dmtimers.c
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Thara Gopinath <thara@ti.com>
 *
 * OMAP2 Dual-Mode Timers
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/err.h>

#include <mach/irqs.h>
#include <plat/dmtimer.h>

#define OMAP1610_GPTIMER1_BASE		0xfffb1400
#define OMAP1610_GPTIMER2_BASE          0xfffb1c00
#define OMAP1610_GPTIMER3_BASE          0xfffb2400
#define OMAP1610_GPTIMER4_BASE          0xfffb2c00
#define OMAP1610_GPTIMER5_BASE          0xfffb3400
#define OMAP1610_GPTIMER6_BASE          0xfffb3c00
#define OMAP1610_GPTIMER7_BASE          0xfffb7400
#define OMAP1610_GPTIMER8_BASE          0xfffbd400

static const int dm_timer_count = NO_DM_TIMERS;

unsigned long omap1_dm_timer_set_clk(struct platform_device *pdev, int source)
{
	unsigned long rate = 0;
	int n = (pdev->id) << 1;
	struct clk *sys_clk;
	u32 l;

	l = omap_readl(MOD_CONF_CTRL_1) & ~(0x03 << n);
	l |= source << n;
	omap_writel(l, MOD_CONF_CTRL_1);

	switch (source) {
	case OMAP_TIMER_SRC_SYS_CLK:
		sys_clk = clk_get(NULL, "ck_ref");
		rate = clk_get_rate(sys_clk);
		clk_put(sys_clk);
		break;
	case OMAP_TIMER_SRC_32_KHZ:
		rate = 32000;
		break;
	case OMAP_TIMER_SRC_EXT_CLK:
		rate = 20000000;
		break;
	}
	return rate;
}

int __init omap1_dm_timer_init(void)
{
	int i = 0;

	if (!cpu_is_omap16xx())
		return 0;

	for (i = 0; i < dm_timer_count; i++) {
		struct platform_device *pdev;
		struct omap_dm_timer_plat_info *pdata;
		unsigned long base;
		unsigned int irq;
		int ret;

		pdev = platform_device_alloc("dmtimer", i);

		switch (i) {
		case 0:
			base = OMAP1610_GPTIMER1_BASE;
			irq = INT_1610_GPTIMER1;
			break;
		case 1:
			base = OMAP1610_GPTIMER2_BASE;
			irq = INT_1610_GPTIMER2;
			break;
		case 2:
			base = OMAP1610_GPTIMER3_BASE;
			irq = INT_1610_GPTIMER3;
			break;
		case 3:
			base = OMAP1610_GPTIMER4_BASE;
			irq = INT_1610_GPTIMER4;
			break;
		case 4:
			base = OMAP1610_GPTIMER5_BASE;
			irq = INT_1610_GPTIMER5;
			break;
		case 5:
			base = OMAP1610_GPTIMER6_BASE;
			irq = INT_1610_GPTIMER6;
			break;
		case 6:
			base = OMAP1610_GPTIMER7_BASE;
			irq = INT_1610_GPTIMER7;
			break;
		case 7:
			base = OMAP1610_GPTIMER8_BASE;
			irq = INT_1610_GPTIMER8;
			break;
		default:
			/* Should never reach here */
			return  -EINVAL;
		}

			pdata = kzalloc(sizeof(struct omap_dm_timer_plat_info),
					GFP_KERNEL);
			pdata->omap_dm_set_source_clk = omap1_dm_timer_set_clk;
			pdata->io_base = ioremap(base, SZ_4K);
			pdata->irq = irq;

			ret = platform_device_add_data(pdev, pdata,
					sizeof(*pdata));
			if (ret)
				goto fail;

			ret = platform_device_add(pdev);
			if (ret)
				goto fail;
			continue;
fail:
			platform_device_put(pdev);
			return ret;
		}
	return 0;
}
arch_initcall(omap1_dm_timer_init);
