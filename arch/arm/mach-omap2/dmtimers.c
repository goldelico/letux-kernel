/**
 * linux/arch/arm/mach-omap2/dmtimers.c
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
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/err.h>

#include <mach/irqs.h>
#include <plat/dmtimer.h>
#include <plat/omap_hwmod.h>
#include <plat/omap_device.h>

#include "dmtimers.h"

#define NO_EARLY_TIMERS		2

struct omap2_dm_timer {
	struct clk *iclk, *fclk;
	int (*device_enable)(struct platform_device *pdev);
	int (*device_idle)(struct platform_device *pdev);
};

static const int dm_timer_count = NO_DM_TIMERS;
static struct omap2_dm_timer omap2_dm_timers[NO_DM_TIMERS];

#ifdef CONFIG_ARCH_OMAP2
static const char *omap2_dm_source_names[] __initdata = {
	"sys_ck",
	"func_32k_ck",
	"alt_ck",
	NULL
};

static struct clk *omap2_dm_source_clocks[3];
#elif defined(CONFIG_ARCH_OMAP3)
const char *omap2_dm_source_names[] __initdata = {
	"sys_ck",
	"omap_32k_fck",
	NULL
};
struct clk *omap2_dm_source_clocks[2];
#elif defined(CONFIG_ARCH_OMAP4)
static const char *omap2_dm_source_names[] __initdata = {
	"sys_ck",
	"omap_32k_fck",
	NULL
};
static struct clk *omap2_dm_source_clocks[2];
#endif

void omap2_dm_timer_enable(struct platform_device *pdev)
{
	if (cpu_is_omap24xx())	{
		clk_enable(omap2_dm_timers[pdev->id].iclk);
		clk_enable(omap2_dm_timers[pdev->id].fclk);
	} else
		omap2_dm_timers[pdev->id].device_enable(pdev);
}

void omap2_dm_timer_disable(struct platform_device *pdev)
{
	if (cpu_is_omap24xx())	{
		clk_disable(omap2_dm_timers[pdev->id].iclk);
		clk_disable(omap2_dm_timers[pdev->id].fclk);
	} else
		omap2_dm_timers[pdev->id].device_idle(pdev);
}

unsigned long omap2_dm_timer_set_clk(struct platform_device *pdev, int source)
{
	int ret = -EINVAL;

	if (cpu_is_omap24xx())	{
		clk_disable(omap2_dm_timers[pdev->id].fclk);
		ret = clk_set_parent(omap2_dm_timers[pdev->id].fclk,
				omap2_dm_source_clocks[source]);
		WARN(ret, " omap2 dmtimers: Not able to change the \
				fclk source\n");
		clk_enable(omap2_dm_timers[pdev->id].fclk);
		/*
		 * When the functional clock disappears, too quick writes seem
		 * to cause an abort. XXX Is this still necessary?
		 */
		__delay(150000);
		return clk_get_rate(omap2_dm_timers[pdev->id].fclk);
	} else {
		struct omap_device *odev = to_omap_device(pdev);

		clk_disable(odev->hwmods[0]->_clk);
		ret = clk_set_parent(odev->hwmods[0]->_clk,
				omap2_dm_source_clocks[source]);
		WARN(ret, " omap2 dmtimers: Not able to change the \
				fclk source\n");
		clk_enable(odev->hwmods[0]->_clk);
		/*
		 * When the functional clock disappears, too quick writes seem
		 * to cause an abort. XXX Is this still necessary?
		 */
		__delay(150000);
		return clk_get_rate(odev->hwmods[0]->_clk);
	}
}

struct clk *omap2_dm_timer_get_fclk(struct platform_device *pdev)
{
	if (cpu_is_omap24xx())	{
		return omap2_dm_timers[pdev->id].fclk;
	} else {
		struct omap_device *odev = to_omap_device(pdev);

		return odev->hwmods[0]->_clk;
	}
}

/*
 * Define gpt1 and gpt2 as early devices. Later on during normal
 * init these devices will be defined again. But then the below
 * structures will be dropped after init. This is applicable only
 * for OMAP2 as OMAP3 and OMAP4 will currently use hwmod.
 */
static struct omap_dm_timer_plat_info omap2_gpt1_pdata = {
	.omap_dm_clk_enable = omap2_dm_timer_enable,
	.omap_dm_clk_disable = omap2_dm_timer_disable,
	.omap_dm_set_source_clk = omap2_dm_timer_set_clk,
	.omap_dm_get_timer_clk = omap2_dm_timer_get_fclk,
};

static struct platform_device omap2_gpt1_early_dev __initdata = {
	.name = "dmtimer",
	.id = 0,
	.dev = {
		.platform_data = &omap2_gpt1_pdata,
	},
};

static struct omap_dm_timer_plat_info omap2_gpt2_pdata = {
	.omap_dm_clk_enable = omap2_dm_timer_enable,
	.omap_dm_clk_disable = omap2_dm_timer_disable,
	.omap_dm_set_source_clk = omap2_dm_timer_set_clk,
	.omap_dm_get_timer_clk = omap2_dm_timer_get_fclk,
};

static struct platform_device omap2_gpt2_early_dev __initdata = {
	.name = "dmtimer",
	.id = 1,
	.dev = {
		.platform_data = &omap2_gpt2_pdata,
	},
};

static struct platform_device *omap2_dmtimer_early_devices[] __initdata = {
	&omap2_gpt1_early_dev,
	&omap2_gpt2_early_dev
};

struct omap_device_pm_latency omap2_dmtimer_latency[] = {
	{
	.deactivate_func = omap_device_idle_hwmods,
	.activate_func   = omap_device_enable_hwmods,
	.flags = OMAP_DEVICE_LATENCY_AUTO_ADJUST,

	},
};

void __init omap2_dm_timer_setup_clksrc(void)
{
	static int is_initialized;
	int i;

	/*
	 * Check if clock source setup has already been
	 * done as part of early init. If yes avoid it
	 * doing it again.
	 */
	if (is_initialized)
		return;

	for (i = 0; omap2_dm_source_names[i] != NULL; i++)
			omap2_dm_source_clocks[i] =
				clk_get(NULL, omap2_dm_source_names[i]);
	is_initialized = 1;
}

void __init omap2_dm_timer_early_init(void)
{
	int i = 0;

	/*
	 * OMAP 2 is not currently adapted to hwmods
	 * So implement the old way of platform_early_register
	 */
	if (cpu_is_omap24xx())	{
		for (i = 0; i < NO_EARLY_TIMERS; i++) {
			char clk_name[16];
			struct omap_dm_timer_plat_info *pdata =
				omap2_dmtimer_early_devices[i]->
				dev.platform_data;
			unsigned long base;

			switch (i) {
			case 0:
				base = OMAP24XX_GPTIMER1_BASE;
				pdata->irq = INT_24XX_GPTIMER1;
				break;
			case 1:
				base = OMAP24XX_GPTIMER2_BASE;
				pdata->irq = INT_24XX_GPTIMER2;
				break;
			default:
				/* Error should never enter here */
				return;
			}

			pdata->io_base = ioremap(base, SZ_8K);
			sprintf(clk_name, "gpt%d_ick", i + 1);
			omap2_dm_timers[i].iclk = clk_get(NULL, clk_name);
			sprintf(clk_name, "gpt%d_fck", i + 1);
			omap2_dm_timers[i].fclk = clk_get(NULL, clk_name);
		}
		early_platform_add_devices(omap2_dmtimer_early_devices,
					NO_EARLY_TIMERS);
	} else if ((cpu_is_omap34xx()) || (cpu_is_omap44xx())) {
		char *name = "dmtimer";

		do {
			struct omap_device *od;
			struct omap_hwmod *oh;
			int hw_mod_name_len = 16;
			char oh_name[hw_mod_name_len];
			struct omap_dm_timer_plat_info *pdata;

			snprintf(oh_name, hw_mod_name_len,
					"gptimer%d", i + 1);
			oh = omap_hwmod_lookup(oh_name);
			if (!oh)
				break;

			pdata = kzalloc(sizeof(struct omap_dm_timer_plat_info),
					GFP_KERNEL);
			if(!pdata){
				WARN_ON(1);
				return -ENOMEM;
			}

			pdata->omap_dm_clk_enable = omap2_dm_timer_enable;
			pdata->omap_dm_clk_disable = omap2_dm_timer_disable;
			pdata->omap_dm_set_source_clk = omap2_dm_timer_set_clk;
			pdata->omap_dm_get_timer_clk = omap2_dm_timer_get_fclk;
			pdata->io_base = oh->_rt_va;
			pdata->irq = oh->mpu_irqs[0].irq;

			od = omap_device_build(name, i, oh,
					pdata, sizeof(*pdata),
					omap2_dmtimer_latency,
					ARRAY_SIZE(omap2_dmtimer_latency), 1);

			WARN(IS_ERR(od), "Cant build omap_device for %s:%s.\n",
					name, oh->name);
			omap2_dm_timers[i].device_enable = omap_device_enable;
			omap2_dm_timers[i].device_idle = omap_device_idle;
			i++;
		} while (i < NO_EARLY_TIMERS);
	}
	omap2_dm_timer_setup_clksrc();
	early_platform_driver_register_all("earlytimer");
	early_platform_driver_probe("earlytimer", NO_EARLY_TIMERS, 0);
	return;
}

int __init omap2_dm_timer_init(void)
{
	int i = 0;

	omap2_dm_timer_setup_clksrc();
	if (cpu_is_omap24xx()) {
		for (i = 0; i < dm_timer_count; i++) {
			struct platform_device *pdev;
			struct omap_dm_timer_plat_info *pdata;
			unsigned long base;
			unsigned int irq;
			char clk_name[16];
			int ret;

			pdev = platform_device_alloc("dmtimer", i);

			switch (i) {
			case 0:
				base = OMAP24XX_GPTIMER1_BASE;
				irq = INT_24XX_GPTIMER1;
				break;
			case 1:
				base = OMAP24XX_GPTIMER2_BASE;
				irq = INT_24XX_GPTIMER2;
				break;
			case 2:
				base = OMAP24XX_GPTIMER3_BASE;
				irq = INT_24XX_GPTIMER3;
				break;
			case 3:
				base = OMAP24XX_GPTIMER4_BASE;
				irq = INT_24XX_GPTIMER4;
				break;
			case 4:
				base = OMAP24XX_GPTIMER5_BASE;
				irq = INT_24XX_GPTIMER5;
				break;
			case 5:
				base = OMAP24XX_GPTIMER6_BASE;
				irq = INT_24XX_GPTIMER6;
				break;
			case 6:
				base = OMAP24XX_GPTIMER7_BASE;
				irq = INT_24XX_GPTIMER7;
				break;
			case 7:
				base = OMAP24XX_GPTIMER8_BASE;
				irq = INT_24XX_GPTIMER8;
				break;
			case 8:
				base = OMAP24XX_GPTIMER9_BASE;
				irq = INT_24XX_GPTIMER9;
				break;
			case 9:
				base = OMAP24XX_GPTIMER10_BASE;
				irq = INT_24XX_GPTIMER10;
				break;
			case 10:
				base = OMAP24XX_GPTIMER11_BASE;
				irq = INT_24XX_GPTIMER11;
				break;
			case 11:
				base = OMAP24XX_GPTIMER12_BASE;
				irq = INT_24XX_GPTIMER12;
				break;
			default:
				/* Should never enter here */
				return -EINVAL;
			}

			pdata = kzalloc(sizeof(struct omap_dm_timer_plat_info),
					GFP_KERNEL);
			if(!pdata){
				WARN_ON(1);
				return -ENOMEM;
			}
			pdata->omap_dm_clk_enable = omap2_dm_timer_enable;
			pdata->omap_dm_clk_disable = omap2_dm_timer_disable;
			pdata->omap_dm_set_source_clk = omap2_dm_timer_set_clk;
			pdata->omap_dm_get_timer_clk = omap2_dm_timer_get_fclk;
			pdata->io_base = ioremap(base, SZ_8K);
			pdata->irq = irq;
			ret = platform_device_add_data(pdev, pdata,
					sizeof(*pdata));
			if (ret)
				goto fail;

			ret = platform_device_add(pdev);
			if (ret)
				goto fail;
			/*
			 * Check if an early init is already done
			 * for this dmtimer.If yes do not get the
			 * clock handles again.
			 */
			if (!omap2_dm_timers[i].iclk) {
				sprintf(clk_name, "gpt%d_ick", i + 1);
				omap2_dm_timers[i].iclk =
					clk_get(NULL, clk_name);
			}
			if (!omap2_dm_timers[i].fclk) {
				sprintf(clk_name, "gpt%d_fck", i + 1);
				omap2_dm_timers[i].fclk =
					clk_get(NULL, clk_name);
			}
			continue;
fail:
			platform_device_put(pdev);
			return ret;
		}
	} else if ((cpu_is_omap34xx()) || (cpu_is_omap44xx())) {
		char *name = "dmtimer";

		do {
			struct omap_device *od;
			struct omap_hwmod *oh;
			int hw_mod_name_len = 16;
			char oh_name[hw_mod_name_len];
			struct omap_dm_timer_plat_info *pdata;

			snprintf(oh_name, hw_mod_name_len,
					"gptimer%d", i + 1);
			oh = omap_hwmod_lookup(oh_name);
			if (!oh)
				break;
			pdata = kzalloc(sizeof(struct omap_dm_timer_plat_info),
					GFP_KERNEL);
			if(!pdata){
				WARN_ON(1);
				return -ENOMEM;
			}
			pdata->omap_dm_clk_enable = omap2_dm_timer_enable;
			pdata->omap_dm_clk_disable = omap2_dm_timer_disable;
			pdata->omap_dm_set_source_clk = omap2_dm_timer_set_clk;
			pdata->omap_dm_get_timer_clk = omap2_dm_timer_get_fclk;
			pdata->io_base = oh->_rt_va;
			pdata->irq = oh->mpu_irqs[0].irq;

			od = omap_device_build(name, i, oh,
					pdata, sizeof(*pdata),
					omap2_dmtimer_latency,
					ARRAY_SIZE(omap2_dmtimer_latency), 0);
			WARN(IS_ERR(od), "Cant build omap_device for %s:%s.\n",
					name, oh->name);
			omap2_dm_timers[i].device_enable = omap_device_enable;
			omap2_dm_timers[i].device_idle = omap_device_idle;
			i++;
		} while (1);
	}
	return 0;
}
arch_initcall(omap2_dm_timer_init);
