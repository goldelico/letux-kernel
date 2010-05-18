/*
 * arch/arm/mach-omap2/ssi.c
 *
 * SSI device definition
 *
 * Copyright (C) 2007-2008 Nokia Corporation. All rights reserved.
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Author: Carlos Chinea <carlos.chinea@nokia.com>
 * Author: Sebastien JAN <s-jan@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/notifier.h>
#include <mach/clock.h>
#include <mach/omap_hsi.h>
#include <linux/hsi_driver_if.h>
#include "clock.h"
#include <asm/clkdev.h>

#define	hsi_inl(p)	inl((unsigned long) p)
#define	hsi_outl(v, p)	outl(v, (unsigned long) p)

/**
 *	struct ssi_internal_clk - Generic virtual ssi clock
 *	@clk: clock data
 *	@nb: notfier block for the DVFS notification chain
 *	@childs: Array of SSI FCK and ICK clocks
 *	@n_childs: Number of clocks in childs array
 *	@rate_change: Tracks if we are in the middle of a clock rate change
 *	@pdev: Reference to the SSI platform device associated to the clock
 *	@drv_nb: Reference to driver nb, use to propagate the DVFS notification
 */
struct ssi_internal_clk {
	struct clk clk;
	struct notifier_block nb;

	struct clk **childs;
	int n_childs;

	unsigned int rate_change:1;

	struct platform_device *pdev;
	struct notifier_block *drv_nb;
};

static void ssi_set_mode(struct platform_device *pdev, u32 mode)
{
	struct hsi_platform_data *pdata = pdev->dev.platform_data;
	void __iomem *base = OMAP2_IO_ADDRESS(pdev->resource[0].start);
	int port;

	for (port = 1; port <= pdata->num_ports; port++) {
		outl(mode, (unsigned int) base + HSI_HST_MODE_REG(port));
		outl(mode, (unsigned int) base + HSI_HSR_MODE_REG(port));
	}
}

static void ssi_save_mode(struct platform_device *pdev)
{
	struct hsi_platform_data *pdata = pdev->dev.platform_data;
	void __iomem *base = OMAP2_IO_ADDRESS(pdev->resource[0].start);
	struct port_ctx *p;
	int port;

	for (port = 1; port <= pdata->num_ports; port++) {
		p = &pdata->ctx.pctx[port - 1];
		p->hst.mode = hsi_inl(base + HSI_HST_MODE_REG(port));
		p->hsr.mode = hsi_inl(base + HSI_HSR_MODE_REG(port));
	}
}

static void ssi_restore_mode(struct platform_device *pdev)
{
	struct hsi_platform_data *pdata = pdev->dev.platform_data;
	void __iomem *base = OMAP2_IO_ADDRESS(pdev->resource[0].start);
	struct port_ctx *p;
	int port;

	for (port = 1; port <= pdata->num_ports; port++) {
		p = &pdata->ctx.pctx[port - 1];
		hsi_outl(p->hst.mode, base + HSI_HST_MODE_REG(port));
		hsi_outl(p->hsr.mode, base + HSI_HSR_MODE_REG(port));
	}
}

static int ssi_clk_event(struct notifier_block *nb, unsigned long event,
								void *data)
{
/* TODO: implement clock change support
	struct ssi_internal_clk *ssi_clk =
				container_of(nb, struct ssi_internal_clk, nb);
	switch (event) {
	case CLK_PRE_RATE_CHANGE:
		ssi_clk->drv_nb->notifier_call(ssi_clk->drv_nb, event, data);
		ssi_clk->rate_change = 1;
		if (ssi_clk->clk.usecount > 0) {
			ssi_save_mode(ssi_clk->pdev);
			ssi_set_mode(ssi_clk->pdev, HSI_MODE_SLEEP);
		}
		break;
	case CLK_ABORT_RATE_CHANGE:
	case CLK_POST_RATE_CHANGE:
		if ((ssi_clk->clk.usecount > 0) && (ssi_clk->rate_change))
			ssi_restore_mode(ssi_clk->pdev);

		ssi_clk->rate_change = 0;
		ssi_clk->drv_nb->notifier_call(ssi_clk->drv_nb, event, data);
		break;
	default:
		break;
	}
*/

	return NOTIFY_DONE;
}

static int ssi_clk_notifier_register(struct clk *clk, struct notifier_block *nb)
{
	struct ssi_internal_clk *ssi_clk;

	if (!clk || !nb)
		return -EINVAL;

	ssi_clk = container_of(clk, struct ssi_internal_clk, clk);
	ssi_clk->drv_nb = nb;
	ssi_clk->nb.priority = nb->priority;
	/* NOTE: We only want notifications from the functional clock */
/* TODO: implement clock change support
	return clk_notifier_register(ssi_clk->childs[1], &ssi_clk->nb);
*/
	pr_debug("%s called\n", __func__);
	return 0;
}

static int ssi_clk_notifier_unregister(struct clk *clk,
						struct notifier_block *nb)
{
	struct ssi_internal_clk *ssi_clk;

	if (!clk || !nb)
		return -EINVAL;

	ssi_clk = container_of(clk, struct ssi_internal_clk, clk);
	ssi_clk->drv_nb = NULL;
/* TODO: implement clock change support
	return clk_notifier_unregister(ssi_clk->childs[1], &ssi_clk->nb);
*/
	pr_debug(KERN_DEBUG "%s called", __func__);
	return 0;
}

static void ssi_save_ctx(struct platform_device *pdev)
{
	struct hsi_platform_data *pdata = pdev->dev.platform_data;
	void __iomem *base = OMAP2_IO_ADDRESS(pdev->resource[0].start);
	struct port_ctx *p;
	int port;

/* TODO: update support for omap_pm_get_dev_context_loss_count
	pdata->ctx.loss_count =
			omap_pm_get_dev_context_loss_count(&pdev->dev);
*/
	pdata->ctx.sysconfig = hsi_inl(base + HSI_SYS_SYSCONFIG_REG);
	pdata->ctx.gdd_gcr = hsi_inl(base + HSI_GDD_GCR_REG);
	for (port = 1; port <= pdata->num_ports; port++) {
		p = &pdata->ctx.pctx[port - 1];
		p->sys_mpu_enable[0] = hsi_inl(base +
					HSI_SYS_MPU_ENABLE_REG(port, 0));
		p->sys_mpu_enable[1] = hsi_inl(base +
					HSI_SYS_MPU_ENABLE_REG(port, 1));
		p->hst.frame_size = hsi_inl(base +
						HSI_HST_FRAMESIZE_REG(port));
		p->hst.divisor = hsi_inl(base + HSI_HST_DIVISOR_REG(port));
		p->hst.channels = hsi_inl(base + HSI_HST_CHANNELS_REG(port));
		p->hst.arb_mode = hsi_inl(base + HSI_HST_ARBMODE_REG(port));
		p->hsr.frame_size = hsi_inl(base +
						HSI_HSR_FRAMESIZE_REG(port));
		p->hsr.timeout = hsi_inl(base + HSI_HSR_COUNTERS_REG(port));
		p->hsr.channels = hsi_inl(base + HSI_HSR_CHANNELS_REG(port));
	}
}

static void ssi_restore_ctx(struct platform_device *pdev)
{
	struct hsi_platform_data *pdata = pdev->dev.platform_data;
	void __iomem *base = OMAP2_IO_ADDRESS(pdev->resource[0].start);
	struct port_ctx *p;
	int port;
/* TODO: update support for omap_pm_get_dev_context_loss_count
	int loss_count;

	loss_count = omap_pm_get_dev_context_loss_count(&pdev->dev);
	if (loss_count == pdata->ctx.loss_count)
		return;
*/
	hsi_outl(pdata->ctx.sysconfig, base + HSI_SYS_SYSCONFIG_REG);
	hsi_outl(pdata->ctx.gdd_gcr, base + HSI_GDD_GCR_REG);
	for (port = 1; port <= pdata->num_ports; port++) {
		p = &pdata->ctx.pctx[port - 1];
		hsi_outl(p->sys_mpu_enable[0], base +
					HSI_SYS_MPU_ENABLE_REG(port, 0));
		hsi_outl(p->sys_mpu_enable[1], base +
					HSI_SYS_MPU_ENABLE_REG(port, 1));
		hsi_outl(p->hst.frame_size, base +
						HSI_HST_FRAMESIZE_REG(port));
		hsi_outl(p->hst.divisor, base + HSI_HST_DIVISOR_REG(port));
		hsi_outl(p->hst.channels, base + HSI_HST_CHANNELS_REG(port));
		hsi_outl(p->hst.arb_mode, base + HSI_HST_ARBMODE_REG(port));
		hsi_outl(p->hsr.frame_size, base +
						HSI_HSR_FRAMESIZE_REG(port));
		hsi_outl(p->hsr.timeout, base + HSI_HSR_COUNTERS_REG(port));
		hsi_outl(p->hsr.channels, base + HSI_HSR_CHANNELS_REG(port));
	}
}

static void ssi_pdev_release(struct device *dev)
{
}

/*
 * NOTE: We abuse a little bit the struct port_ctx to use it also for
 * initialization.
 */
static struct port_ctx ssi_port_ctx[] = {
	[0] = {
		.hst.mode = HSI_MODE_FRAME,
		.hst.flow = 0, /* not supported on SSI */
		.hst.frame_size = HSI_FRAMESIZE_DEFAULT,
		.hst.divisor = 1,
		.hst.channels = HSI_CHANNELS_DEFAULT,
		.hst.arb_mode = HSI_ARBMODE_ROUNDROBIN,
		.hsr.mode = HSI_MODE_FRAME,
		.hsr.flow = 0, /* not supported on SSI */
		.hsr.frame_size = HSI_FRAMESIZE_DEFAULT,
		.hsr.channels = HSI_CHANNELS_DEFAULT,
		.hsr.divisor = 0, /* not supported  on SSI */
		.hsr.timeout = HSI_TIMEOUT_DEFAULT,
		},
};

static struct hsi_platform_data ssi_pdata = {
	.num_ports = ARRAY_SIZE(ssi_port_ctx),
	.ctx.pctx = ssi_port_ctx,
	.clk_notifier_register = ssi_clk_notifier_register,
	.clk_notifier_unregister = ssi_clk_notifier_unregister,
};

static struct resource ssi_resources[] = {
	[0] =	{
		.start = 0x48058000,
		.end = 0x4805bbff,
		.name = "omap_ssi_iomem",
		.flags = IORESOURCE_MEM,
		},
	[1] =	{
		.start = 67,
		.end = 67,
		.name = "ssi_p1_mpu_irq0",
		.flags = IORESOURCE_IRQ,
		},
	[2] =	{
		.start = 69,
		.end = 69,
		.name = "ssi_p1_mpu_irq1",
		.flags = IORESOURCE_IRQ,
		},
	[3] =	{
		.start = 68,
		.end = 68,
		.name = "ssi_p2_mpu_irq0",
		.flags = IORESOURCE_IRQ,
		},
	[4] =	{
		.start = 70,
		.end = 70,
		.name = "ssi_p2_mpu_irq1",
		.flags = IORESOURCE_IRQ,
		},
	[5] =	{
		.start = 71,
		.end = 71,
		.name = "ssi_gdd",
		.flags = IORESOURCE_IRQ,
		},
	[6] =	{
		.start = 151,
		.end = 0,
		.name = "ssi_p1_cawake_gpio",
		.flags = IORESOURCE_IRQ | IORESOURCE_UNSET,
		},
	[7] =	{
		.start = 0,
		.end = 0,
		.name = "ssi_p2_cawake_gpio",
		.flags = IORESOURCE_IRQ | IORESOURCE_UNSET,
		},
	[8] =	{
		.start = 8, /* DMA channels available */
		.end = 8,
		.name = "hsi_gdd_chan_count",
		.flags = IORESOURCE_DMA,
		},
};

static struct platform_device ssi_pdev = {
	.name = "omap_ssi",
	.id = -1,
	.num_resources = ARRAY_SIZE(ssi_resources),
	.resource = ssi_resources,
	.dev =	{
		.release = ssi_pdev_release,
		.platform_data = &ssi_pdata,
		},
};

#if 0 /* TODO: check if following code can be removed when testing with PM */
#define __HSI_CLK_FIX__
#ifdef __HSI_CLK_FIX__
/*
 * FIXME: TO BE REMOVED.
 * This hack allows us to ensure that clocks are stable before accessing
 * SSI controller registers. To be removed when PM functionalty is in place.
 */
static int check_ssi_active(void)
{
	u32 reg;
	unsigned long dl = jiffies + msecs_to_jiffies(500);
	void __iomem *cm_idlest1 = OMAP2_IO_ADDRESS(0x48004a20);

	reg = inl(cm_idlest1);
	while ((!(reg & 0x01)) && (time_before(jiffies, dl)))
		reg = inl(cm_idlest1);

	if (!(reg & 0x01)) { /* ST_SSI */
		pr_err("SSI is still in STANDBY ! (BUG !?)\n");
		return -1;
	}

	return 0;
}
#endif /* __HSI_CLK_FIX__ */
#endif

static int ssi_clk_init(struct ssi_internal_clk *ssi_clk)
{
	const char *clk_names[] = { "ssi_ssr_fck", "ssi_sst_fck", "ssi_l4_ick",
								"ssi_ick" };
	int i;
	int j;

	ssi_clk->n_childs = ARRAY_SIZE(clk_names);
	ssi_clk->childs = kzalloc(ssi_clk->n_childs * sizeof(*ssi_clk->childs),
								GFP_KERNEL);
	if (!ssi_clk->childs)
		return -ENOMEM;

	for (i = 0; i < ssi_clk->n_childs; i++) {
		ssi_clk->childs[i] = clk_get(&ssi_clk->pdev->dev, clk_names[i]);
		if (IS_ERR(ssi_clk->childs[i])) {
			pr_err("Unable to get SSI clock: %s\n", clk_names[i]);
			for (j = i - 1; j >= 0; j--)
				clk_put(ssi_clk->childs[j]);
			return -ENODEV;
		}
	}

	return 0;
}

static int ssi_clk_enable(struct clk *clk)
{
	struct ssi_internal_clk *ssi_clk =
				container_of(clk, struct ssi_internal_clk, clk);
	int err;
	int i;

	for (i = 0; i < ssi_clk->n_childs; i++) {
		err = omap2_clk_enable(ssi_clk->childs[i]);
		if (unlikely(err < 0))
			goto rollback;
	}
#ifdef __HSI_CLK_FIX__
	/*
	 * FIXME: To be removed
	 * Wait until the SSI controller has the clocks stable
	 */
	check_ssi_active();
#endif
	ssi_restore_ctx(ssi_clk->pdev);
	if (!ssi_clk->rate_change)
		ssi_restore_mode(ssi_clk->pdev);

	return 0;
rollback:
	pr_err("Error on SSI clk child %d\n", i);
	for (i = i - 1; i >= 0; i--)
		omap2_clk_disable(ssi_clk->childs[i]);

	return err;
}

static void ssi_clk_disable(struct clk *clk)
{
	struct ssi_internal_clk *ssi_clk =
				container_of(clk, struct ssi_internal_clk, clk);
	int i;

	if (!ssi_clk->rate_change) {
		ssi_save_mode(ssi_clk->pdev);
		ssi_set_mode(ssi_clk->pdev, HSI_MODE_SLEEP);
	}
	/* Save ctx in all ports */
	ssi_save_ctx(ssi_clk->pdev);

	for (i = 0; i < ssi_clk->n_childs; i++)
		omap2_clk_disable(ssi_clk->childs[i]);
}

int omap_ssi_config(struct omap_ssi_board_config *ssi_config)
{
	int port;
	int cawake_gpio;

	ssi_pdata.num_ports = ssi_config->num_ports;
	for (port = 0; port < ssi_config->num_ports; port++) {
		cawake_gpio = ssi_config->cawake_gpio[port];
		if (cawake_gpio < 0)
			continue;	/* Nothing to do */

		if (gpio_request(cawake_gpio, "CAWAKE") < 0) {
			dev_err(&ssi_pdev.dev, "FAILED to request CAWAKE"
						" GPIO %d\n", cawake_gpio);
			return -EBUSY;
		}

		gpio_direction_input(cawake_gpio);
		ssi_resources[6 + port].start = gpio_to_irq(cawake_gpio);
		ssi_resources[6 + port].flags &= ~IORESOURCE_UNSET;
		ssi_resources[6 + port].flags |= IORESOURCE_IRQ_HIGHEDGE |
							IORESOURCE_IRQ_LOWEDGE;
	}

	pr_debug("GPIO for SSI is now configured\n");
	return 0;
}

/* Mux settings for OMAP3430 */
/* FIXME: deprecated since pin muxing was reworked in .32 */
/*static const int omap34xx_ssi_pins[] = {
	AA8_3430_SSI1_DAT_TX,
	AA9_3430_SSI1_FLAG_TX,
	W8_3430_SSI1_RDY_TX,
	Y8_3430_GPIO_151,
	AE1_3430_SSI1_DAT_RX,
	AD1_3430_SSI1_FLAG_RX,
	AD2_3430_SSI1_RDY_RX,
	AC1_3430_SSI1_WAKE,
};
*/
void omap_ssi_mux_setup(void)
{
/*	int i;
	for (i = 0; i < ARRAY_SIZE(omap34xx_ssi_pins); i++)
		omap_cfg_reg(omap34xx_ssi_pins[i]);
	pr_debug("Pin muxing for SSI support (disables UART1 and McBsp4)\n");
*/}

static const struct clkops clkops_ssi = {
	.enable		= ssi_clk_enable,
	.disable	= ssi_clk_disable,
};

static struct ssi_internal_clk ssi_clock = {
	.clk = {
		.name = "hsi_clk",
		.id = -1,
		.clkdm_name = "core_l4_clkdm",
		.ops = &clkops_ssi,
	},
	.nb = {
		.notifier_call = ssi_clk_event,
		.priority = INT_MAX,
	},
	.pdev = &ssi_pdev,
};

static struct clk_lookup hsi_lk = {
	.dev_id = NULL,
	.con_id = "hsi_clk",
	.clk = &ssi_clock.clk,
};

static int __init omap_ssi_init(void)
{
	int err;
	struct clk *hsi_clk = &ssi_clock.clk;

	ssi_clk_init(&ssi_clock);
	clk_preinit(hsi_clk);
	clkdev_add(&hsi_lk);
	clk_register(hsi_clk);
	omap2_init_clk_clkdm(hsi_clk);

	err = platform_device_register(&ssi_pdev);
	if (err < 0) {
		pr_err("Unable to register SSI platform device: %d\n", err);
		return err;
	}

	omap_ssi_mux_setup();

	pr_info("SSI: device registered\n");

	return 0;
}
subsys_initcall(omap_ssi_init);
