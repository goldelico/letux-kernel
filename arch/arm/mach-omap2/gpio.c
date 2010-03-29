/*
 *  linux/arch/arm/mach-omap2/gpio.c
 *
 * Support functions for OMAP GPIO
 *
 * Copyright (C) 2003-2005 Nokia Corporation
 * Written by Juha Yrjölä <juha.yrjola@nokia.com>
 *
 * Copyright (C) 2009 Texas Instruments
 * Added OMAP4 support - Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/sysdev.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <asm/mach/irq.h>

/*
 * OMAP242X GPIO1 interface data
 */
static struct __initdata resource omap242x_gpio1_resources[] = {
	{
		.start	= OMAP242X_GPIO1_BASE,
		.end	= OMAP242X_GPIO1_BASE + OMAP2_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_24XX_GPIO_BANK1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap242x_gpio1_config = {
	.ick_name = "gpios_ick",
	.fck_name = "gpios_fck",
	.virtual_irq_start = IH_GPIO_BASE,
};

static struct __initdata platform_device omap242x_gpio1 = {
	.name           = "omap-gpio",
	.id             = 0,
	.dev            = {
		.platform_data = &omap242x_gpio1_config,
	},
	.num_resources = ARRAY_SIZE(omap242x_gpio1_resources),
	.resource = omap242x_gpio1_resources,
};

/*
 * OMAP242X GPIO2 interface data
 */
static struct __initdata resource omap242x_gpio2_resources[] = {
	{
		.start	= OMAP242X_GPIO2_BASE,
		.end	= OMAP242X_GPIO2_BASE + OMAP2_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_24XX_GPIO_BANK2,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap242x_gpio2_config = {
	.ick_name = "gpios_ick",
	.fck_name = "gpios_fck",
	.virtual_irq_start = IH_GPIO_BASE + 32,
};

static struct __initdata platform_device omap242x_gpio2 = {
	.name           = "omap-gpio",
	.id             = 1,
	.dev            = {
		.platform_data = &omap242x_gpio2_config,
	},
	.num_resources = ARRAY_SIZE(omap242x_gpio2_resources),
	.resource = omap242x_gpio2_resources,
};

/*
 * OMAP242X GPIO3 interface data
 */
static struct __initdata resource omap242x_gpio3_resources[] = {
	{
		.start	= OMAP242X_GPIO3_BASE,
		.end	= OMAP242X_GPIO3_BASE + OMAP2_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_24XX_GPIO_BANK3,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap242x_gpio3_config = {
	.ick_name = "gpios_ick",
	.fck_name = "gpios_fck",
	.virtual_irq_start = IH_GPIO_BASE + 64,
};

static struct __initdata platform_device omap242x_gpio3 = {
	.name           = "omap-gpio",
	.id             = 2,
	.dev            = {
		.platform_data = &omap242x_gpio3_config,
	},
	.num_resources = ARRAY_SIZE(omap242x_gpio3_resources),
	.resource = omap242x_gpio3_resources,
};

/*
 * OMAP242X GPIO4 interface data
 */
static struct __initdata resource omap242x_gpio4_resources[] = {
	{
		.start	= OMAP242X_GPIO4_BASE,
		.end	= OMAP242X_GPIO4_BASE + OMAP2_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_24XX_GPIO_BANK4,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap242x_gpio4_config = {
	.ick_name = "gpios_ick",
	.fck_name = "gpios_fck",
	.virtual_irq_start = IH_GPIO_BASE + 96,
};

static struct __initdata platform_device omap242x_gpio4 = {
	.name           = "omap-gpio",
	.id             = 3,
	.dev            = {
		.platform_data = &omap242x_gpio4_config,
	},
	.num_resources = ARRAY_SIZE(omap242x_gpio4_resources),
	.resource = omap242x_gpio4_resources,
};

/*
 * OMAP243X GPIO1 interface data
 */
static struct __initdata resource omap243x_gpio1_resources[] = {
	{
		.start	= OMAP243X_GPIO1_BASE,
		.end	= OMAP243X_GPIO1_BASE + OMAP2_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_24XX_GPIO_BANK1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap243x_gpio1_config = {
	.ick_name = "gpios_ick",
	.fck_name = "gpios_fck",
	.virtual_irq_start = IH_GPIO_BASE,
};

static struct __initdata platform_device omap243x_gpio1 = {
	.name           = "omap-gpio",
	.id             = 0,
	.dev            = {
		.platform_data = &omap243x_gpio1_config,
	},
	.num_resources = ARRAY_SIZE(omap243x_gpio1_resources),
	.resource = omap243x_gpio1_resources,
};

/*
 * OMAP243X GPIO2 interface data
 */
static struct __initdata resource omap243x_gpio2_resources[] = {
	{
		.start	= OMAP243X_GPIO2_BASE,
		.end	= OMAP243X_GPIO2_BASE + OMAP2_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_24XX_GPIO_BANK2,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap243x_gpio2_config = {
	.ick_name = "gpios_ick",
	.fck_name = "gpios_fck",
	.virtual_irq_start = IH_GPIO_BASE + 32,
};

static struct __initdata platform_device omap243x_gpio2 = {
	.name           = "omap-gpio",
	.id             = 1,
	.dev            = {
		.platform_data = &omap243x_gpio2_config,
	},
	.num_resources = ARRAY_SIZE(omap243x_gpio2_resources),
	.resource = omap243x_gpio2_resources,
};

/*
 * OMAP243X GPIO3 interface data
 */
static struct __initdata resource omap243x_gpio3_resources[] = {
	{
		.start	= OMAP243X_GPIO3_BASE,
		.end	= OMAP243X_GPIO3_BASE + OMAP2_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_24XX_GPIO_BANK3,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap243x_gpio3_config = {
	.ick_name = "gpios_ick",
	.fck_name = "gpios_fck",
	.virtual_irq_start = IH_GPIO_BASE + 64,
};

static struct __initdata platform_device omap243x_gpio3 = {
	.name           = "omap-gpio",
	.id             = 2,
	.dev            = {
		.platform_data = &omap243x_gpio3_config,
	},
	.num_resources = ARRAY_SIZE(omap243x_gpio3_resources),
	.resource = omap243x_gpio3_resources,
};

/*
 * OMAP243X GPIO4 interface data
 */
static struct __initdata resource omap243x_gpio4_resources[] = {
	{
		.start	= OMAP243X_GPIO4_BASE,
		.end	= OMAP243X_GPIO4_BASE + OMAP2_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_24XX_GPIO_BANK4,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap243x_gpio4_config = {
	.ick_name = "gpios_ick",
	.fck_name = "gpios_fck",
	.virtual_irq_start = IH_GPIO_BASE + 96,
};

static struct __initdata platform_device omap243x_gpio4 = {
	.name           = "omap-gpio",
	.id             = 3,
	.dev            = {
		.platform_data = &omap243x_gpio4_config,
	},
	.num_resources = ARRAY_SIZE(omap243x_gpio4_resources),
	.resource = omap243x_gpio4_resources,
};

/*
 * OMAP243X GPIO5 interface data
 */
static struct __initdata resource omap243x_gpio5_resources[] = {
	{
		.start	= OMAP243X_GPIO5_BASE,
		.end	= OMAP243X_GPIO5_BASE + OMAP2_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_24XX_GPIO_BANK5,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap243x_gpio5_config = {
	.ick_name = "gpio5_ick",
	.fck_name = "gpio5_fck",
	.virtual_irq_start = IH_GPIO_BASE + 128,
};

static struct __initdata platform_device omap243x_gpio5 = {
	.name           = "omap-gpio",
	.id             = 4,
	.dev            = {
		.platform_data = &omap243x_gpio5_config,
	},
	.num_resources = ARRAY_SIZE(omap243x_gpio5_resources),
	.resource = omap243x_gpio5_resources,
};

/*
 * OMAP3 GPIO1 interface data
 */
static struct __initdata resource omap3_gpio1_resources[] = {
	{
		.start	= OMAP34XX_GPIO1_BASE,
		.end	= OMAP34XX_GPIO1_BASE + OMAP3_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_34XX_GPIO_BANK1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap3_gpio1_config = {
	.ick_name = "gpio1_ick",
	.dbck_name = "gpio1_dbck",
	.virtual_irq_start = IH_GPIO_BASE,
};

static struct __initdata platform_device omap3_gpio1 = {
	.name           = "omap-gpio",
	.id             = 0,
	.dev            = {
		.platform_data = &omap3_gpio1_config,
	},
	.num_resources = ARRAY_SIZE(omap3_gpio1_resources),
	.resource = omap3_gpio1_resources,
};

/*
 * OMAP3 GPIO2 interface data
 */
static struct __initdata resource omap3_gpio2_resources[] = {
	{
		.start	= OMAP34XX_GPIO2_BASE,
		.end	= OMAP34XX_GPIO2_BASE + OMAP3_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_34XX_GPIO_BANK2,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap3_gpio2_config = {
	.ick_name = "gpio2_ick",
	.dbck_name = "gpio2_dbck",
	.virtual_irq_start = IH_GPIO_BASE + 32,
};

static struct __initdata platform_device omap3_gpio2 = {
	.name           = "omap-gpio",
	.id             = 1,
	.dev            = {
		.platform_data = &omap3_gpio2_config,
	},
	.num_resources = ARRAY_SIZE(omap3_gpio2_resources),
	.resource = omap3_gpio2_resources,
};

/*
 * OMAP3 GPIO3 interface data
 */
static struct __initdata resource omap3_gpio3_resources[] = {
	{
		.start	= OMAP34XX_GPIO3_BASE,
		.end	= OMAP34XX_GPIO3_BASE + OMAP3_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_34XX_GPIO_BANK3,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap3_gpio3_config = {
	.ick_name = "gpio3_ick",
	.dbck_name = "gpio3_dbck",
	.virtual_irq_start = IH_GPIO_BASE + 64,
};

static struct __initdata platform_device omap3_gpio3 = {
	.name           = "omap-gpio",
	.id             = 2,
	.dev            = {
		.platform_data = &omap3_gpio3_config,
	},
	.num_resources = ARRAY_SIZE(omap3_gpio3_resources),
	.resource = omap3_gpio3_resources,
};

/*
 * OMAP3 GPIO4 interface data
 */
static struct __initdata resource omap3_gpio4_resources[] = {
	{
		.start	= OMAP34XX_GPIO4_BASE,
		.end	= OMAP34XX_GPIO4_BASE + OMAP3_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_34XX_GPIO_BANK4,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap3_gpio4_config = {
	.ick_name = "gpio4_ick",
	.dbck_name = "gpio4_dbck",
	.virtual_irq_start = IH_GPIO_BASE + 96,
};

static struct __initdata platform_device omap3_gpio4 = {
	.name           = "omap-gpio",
	.id             = 3,
	.dev            = {
		.platform_data = &omap3_gpio4_config,
	},
	.num_resources = ARRAY_SIZE(omap3_gpio4_resources),
	.resource = omap3_gpio4_resources,
};

/*
 * OMAP3 GPIO5 interface data
 */
static struct __initdata resource omap3_gpio5_resources[] = {
	{
		.start	= OMAP34XX_GPIO5_BASE,
		.end	= OMAP34XX_GPIO5_BASE + OMAP3_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_34XX_GPIO_BANK5,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap3_gpio5_config = {
	.ick_name = "gpio5_ick",
	.dbck_name = "gpio5_dbck",
	.virtual_irq_start = IH_GPIO_BASE + 128,
};

static struct __initdata platform_device omap3_gpio5 = {
	.name           = "omap-gpio",
	.id             = 4,
	.dev            = {
		.platform_data = &omap3_gpio5_config,
	},
	.num_resources = ARRAY_SIZE(omap3_gpio5_resources),
	.resource = omap3_gpio5_resources,
};

/*
 * OMAP3 GPIO6 interface data
 */
static struct __initdata resource omap3_gpio6_resources[] = {
	{
		.start	= OMAP34XX_GPIO6_BASE,
		.end	= OMAP34XX_GPIO6_BASE + OMAP3_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_34XX_GPIO_BANK4,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap3_gpio6_config = {
	.ick_name = "gpio6_ick",
	.dbck_name = "gpio6_dbck",
	.virtual_irq_start = IH_GPIO_BASE + 160,
};

static struct __initdata platform_device omap3_gpio6 = {
	.name           = "omap-gpio",
	.id             = 5,
	.dev            = {
		.platform_data = &omap3_gpio6_config,
	},
	.num_resources = ARRAY_SIZE(omap3_gpio6_resources),
	.resource = omap3_gpio6_resources,
};

/*
 * OMAP44XX GPIO1 interface data
 */
static struct __initdata resource omap4_gpio1_resources[] = {
	{
		.start	= OMAP44XX_GPIO1_BASE,
		.end	= OMAP44XX_GPIO1_BASE + OMAP4_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_44XX_GPIO_BANK1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap4_gpio1_config = {
	.ick_name = "gpio1_ck",
	.virtual_irq_start = IH_GPIO_BASE,
};

static struct __initdata platform_device omap4_gpio1 = {
	.name           = "omap-gpio",
	.id             = 0,
	.dev            = {
		.platform_data = &omap4_gpio1_config,
	},
	.num_resources = ARRAY_SIZE(omap4_gpio1_resources),
	.resource = omap4_gpio1_resources,
};

/*
 * OMAP44XX GPIO2 interface data
 */
static struct __initdata resource omap4_gpio2_resources[] = {
	{
		.start	= OMAP44XX_GPIO2_BASE,
		.end	= OMAP44XX_GPIO2_BASE + OMAP4_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_44XX_GPIO_BANK2,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap4_gpio2_config = {
	.ick_name = "gpio2_ck",
	.virtual_irq_start = IH_GPIO_BASE + 32,
};

static struct __initdata platform_device omap4_gpio2 = {
	.name           = "omap-gpio",
	.id             = 1,
	.dev            = {
		.platform_data = &omap4_gpio2_config,
	},
	.num_resources = ARRAY_SIZE(omap4_gpio2_resources),
	.resource = omap4_gpio2_resources,
};

/*
 * OMAP44XX GPIO3 interface data
 */
static struct __initdata resource omap4_gpio3_resources[] = {
	{
		.start	= OMAP44XX_GPIO3_BASE,
		.end	= OMAP44XX_GPIO3_BASE + OMAP4_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_44XX_GPIO_BANK3,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap4_gpio3_config = {
	.ick_name = "gpio3_ck",
	.virtual_irq_start = IH_GPIO_BASE + 64,
};

static struct __initdata platform_device omap4_gpio3 = {
	.name           = "omap-gpio",
	.id             = 2,
	.dev            = {
		.platform_data = &omap4_gpio3_config,
	},
	.num_resources = ARRAY_SIZE(omap4_gpio3_resources),
	.resource = omap4_gpio3_resources,
};

/*
 * OMAP44XX GPIO4 interface data
 */
static struct __initdata resource omap4_gpio4_resources[] = {
	{
		.start	= OMAP44XX_GPIO4_BASE,
		.end	= OMAP44XX_GPIO4_BASE + OMAP4_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_44XX_GPIO_BANK4,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap4_gpio4_config = {
	.ick_name = "gpio4_ck",
	.virtual_irq_start = IH_GPIO_BASE + 96,
};

static struct __initdata platform_device omap4_gpio4 = {
	.name           = "omap-gpio",
	.id             = 3,
	.dev            = {
		.platform_data = &omap4_gpio4_config,
	},
	.num_resources = ARRAY_SIZE(omap4_gpio4_resources),
	.resource = omap4_gpio4_resources,
};

/*
 * OMAP44XX GPIO5 interface data
  */
static struct __initdata resource omap4_gpio5_resources[] = {
	{
		.start	= OMAP44XX_GPIO5_BASE,
		.end	= OMAP44XX_GPIO5_BASE + OMAP4_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_44XX_GPIO_BANK5,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap4_gpio5_config = {
	.ick_name = "gpio5_ck",
	.virtual_irq_start = IH_GPIO_BASE + 128,
};

static struct __initdata platform_device omap4_gpio5 = {
	.name           = "omap-gpio",
	.id             = 4,
	.dev            = {
		.platform_data = &omap4_gpio5_config,
	},
	.num_resources = ARRAY_SIZE(omap4_gpio5_resources),
	.resource = omap4_gpio5_resources,
};

/*
 * OMAP44XX GPIO6 interface data
  */
static struct __initdata resource omap4_gpio6_resources[] = {
	{
		.start	= OMAP44XX_GPIO6_BASE,
		.end	= OMAP44XX_GPIO6_BASE + OMAP4_GPIO_AS_LEN - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_44XX_GPIO_BANK6,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct __initdata omap_gpio_platform_data omap4_gpio6_config = {
	.ick_name = "gpio6_ck",
	.virtual_irq_start = IH_GPIO_BASE + 160,
};

static struct __initdata platform_device omap4_gpio6 = {
	.name           = "omap-gpio",
	.id             = 5,
	.dev            = {
		.platform_data = &omap4_gpio6_config,
	},
	.num_resources = ARRAY_SIZE(omap4_gpio6_resources),
	.resource = omap4_gpio6_resources,
};

static struct __initdata platform_device * omap242x_gpio_early_dev[] = {
	&omap242x_gpio1,
	&omap242x_gpio2,
	&omap242x_gpio3,
	&omap242x_gpio4
};

static struct __initdata platform_device * omap243x_gpio_early_dev[] = {
	&omap243x_gpio1,
	&omap243x_gpio2,
	&omap243x_gpio3,
	&omap243x_gpio4,
	&omap243x_gpio5
};

static struct __initdata platform_device * omap3_gpio_early_dev[] = {
	&omap3_gpio1,
	&omap3_gpio2,
	&omap3_gpio3,
	&omap3_gpio4,
	&omap3_gpio5,
	&omap3_gpio6
};

static struct __initdata platform_device * omap4_gpio_early_dev[] = {
	&omap4_gpio1,
	&omap4_gpio2,
	&omap4_gpio3,
	&omap4_gpio4,
	&omap4_gpio5,
	&omap4_gpio6
};

struct omap3_gpio_regs {
	u32 sysconfig;
	u32 irqenable1;
	u32 irqenable2;
	u32 wake_en;
	u32 ctrl;
	u32 oe;
	u32 leveldetect0;
	u32 leveldetect1;
	u32 risingdetect;
	u32 fallingdetect;
	u32 dataout;
	u32 setwkuena;
	u32 setdataout;
};

#ifdef CONFIG_ARCH_OMAP3
static struct omap3_gpio_regs gpio_context[OMAP34XX_NR_GPIOS];
#endif

static struct gpio_bank gpio_bank[OMAP_MAX_NR_GPIOS];
static int gpio_bank_count;

static inline struct gpio_bank *get_gpio_bank(int gpio)
{
	if (cpu_is_omap24xx() || cpu_is_omap34xx() || cpu_is_omap44xx())
		return &gpio_bank[gpio >> 5];
	BUG();
	return  -EINVAL;
}

static inline int get_gpio_index(int gpio)
{
	if (cpu_is_omap24xx() || cpu_is_omap34xx() || cpu_is_omap44xx())
		return gpio & 0x1f;
	BUG();
	return -EINVAL;
}

static inline int gpio_valid(int gpio)
{
	if (gpio < 0)
		return -1;
	if (cpu_is_omap24xx() && gpio < 128)
		return 0;
	if ((cpu_is_omap34xx() || cpu_is_omap44xx()) && gpio < 192)
		return 0;
	return -EINVAL;
}

static int check_gpio(int gpio)
{
	if (unlikely(gpio_valid(gpio) < 0)) {
		printk(KERN_ERR "omap-gpio: invalid GPIO %d\n", gpio);
		dump_stack();
		return -1;
	}
	return 0;
}

static void _set_gpio_direction(struct gpio_bank *bank, int gpio, int is_input)
{
	void __iomem *reg = bank->base;
	u32 l;

	if (cpu_is_omap24xx() || cpu_is_omap34xx())
		reg += OMAP24XX_GPIO_OE;
	else if (cpu_is_omap44xx())
		reg += OMAP4_GPIO_OE;
	else {
		BUG();
		return;
	}

	l = __raw_readl(reg);
	if (is_input)
		l |= 1 << gpio;
	else
		l &= ~(1 << gpio);
	__raw_writel(l, reg);
}

static void _set_gpio_dataout(struct gpio_bank *bank, int gpio, int enable)
{
	void __iomem *reg = bank->base;
	u32 l = 0;

	if (cpu_is_omap24xx() || cpu_is_omap34xx()) {
		if (enable)
			reg += OMAP24XX_GPIO_SETDATAOUT;
		else
			reg += OMAP24XX_GPIO_CLEARDATAOUT;
	} else  if (cpu_is_omap44xx()) {
		if (enable)
			reg += OMAP4_GPIO_SETDATAOUT;
		else
			reg += OMAP4_GPIO_CLEARDATAOUT;
	} else {
		BUG();
		return;
	}

	l = 1 << gpio;
	__raw_writel(l, reg);
}

static int _get_gpio_datain(struct gpio_bank *bank, int gpio)
{
	void __iomem *reg;

	if (check_gpio(gpio) < 0)
		return -EINVAL;

	reg = bank->base;
	if (cpu_is_omap24xx() || cpu_is_omap34xx())
		reg += OMAP24XX_GPIO_DATAIN;
	else if (cpu_is_omap44xx())
		reg += OMAP4_GPIO_DATAIN;
	else
		return -EINVAL;

	return (__raw_readl(reg)
			& (1 << get_gpio_index(gpio))) != 0;
}

static int _get_gpio_dataout(struct gpio_bank *bank, int gpio)
{
	void __iomem *reg;

	if (check_gpio(gpio) < 0)
		return -EINVAL;

	reg = bank->base;
	reg += OMAP24XX_GPIO_DATAOUT;
	return (__raw_readl(reg) & (1 << get_gpio_index(gpio))) != 0;
}

#define MOD_REG_BIT(reg, bit_mask, set)	\
do {	\
	int l = __raw_readl(base + reg); \
	if (set) \
		l |= bit_mask; \
	else \
		l &= ~bit_mask; \
		__raw_writel(l, base + reg); \
} while (0)

void omap_set_gpio_debounce(int gpio, int enable)
{
	struct gpio_bank *bank;
	void __iomem *reg;
	unsigned long flags;
	u32 val, l = 1 << get_gpio_index(gpio);

	bank = get_gpio_bank(gpio);
	reg = bank->base;
	if (cpu_is_omap24xx() || cpu_is_omap34xx())
		reg += OMAP24XX_GPIO_DEBOUNCE_EN;
	else if (cpu_is_omap44xx())
		reg += OMAP4_GPIO_DEBOUNCENABLE;
	else {
		BUG();
		return;
	}

	if (!(bank->mod_usage & l)) {
		printk(KERN_ERR "GPIO %d not requested\n", gpio);
		return;
	}

	spin_lock_irqsave(&bank->lock, flags);
	val = __raw_readl(reg);

	if (enable && !(val & l))
		val |= l;
	else if (!enable && (val & l))
		val &= ~l;
	else
		goto done;

	if (cpu_is_omap34xx()) {
		if (enable)
			clk_enable(bank->dbck);
		else
			clk_disable(bank->dbck);
	}

	__raw_writel(val, reg);
done:
	spin_unlock_irqrestore(&bank->lock, flags);
}
EXPORT_SYMBOL(omap_set_gpio_debounce);

void omap_set_gpio_debounce_time(int gpio, int enc_time)
{
	struct gpio_bank *bank;
	void __iomem *reg;

	bank = get_gpio_bank(gpio);
	reg = bank->base;

	if (!bank->mod_usage) {
		printk(KERN_ERR "GPIO not requested\n");
		return;
	}

	enc_time &= 0xff;
	if (cpu_is_omap24xx() || cpu_is_omap34xx())
		reg += OMAP24XX_GPIO_DEBOUNCE_VAL;
	else if (cpu_is_omap44xx())
		reg += OMAP4_GPIO_DEBOUNCINGTIME;
	else {
		BUG();
		return;
	}
	__raw_writel(enc_time, reg);
}
EXPORT_SYMBOL(omap_set_gpio_debounce_time);

static inline void set_24xx_gpio_triggering(struct gpio_bank *bank, int gpio,
						int trigger)
{
	void __iomem *base = bank->base;
	u32 gpio_bit = 1 << gpio;
	u32 val;

	if (cpu_is_omap44xx()) {
		MOD_REG_BIT(OMAP4_GPIO_LEVELDETECT0, gpio_bit,
			trigger & IRQ_TYPE_LEVEL_LOW);
		MOD_REG_BIT(OMAP4_GPIO_LEVELDETECT1, gpio_bit,
			trigger & IRQ_TYPE_LEVEL_HIGH);
		MOD_REG_BIT(OMAP4_GPIO_RISINGDETECT, gpio_bit,
			trigger & IRQ_TYPE_EDGE_RISING);
		MOD_REG_BIT(OMAP4_GPIO_FALLINGDETECT, gpio_bit,
			trigger & IRQ_TYPE_EDGE_FALLING);
	} else if (cpu_is_omap24xx() || cpu_is_omap34xx()) {
		MOD_REG_BIT(OMAP24XX_GPIO_LEVELDETECT0, gpio_bit,
			trigger & IRQ_TYPE_LEVEL_LOW);
		MOD_REG_BIT(OMAP24XX_GPIO_LEVELDETECT1, gpio_bit,
			trigger & IRQ_TYPE_LEVEL_HIGH);
		MOD_REG_BIT(OMAP24XX_GPIO_RISINGDETECT, gpio_bit,
			trigger & IRQ_TYPE_EDGE_RISING);
		MOD_REG_BIT(OMAP24XX_GPIO_FALLINGDETECT, gpio_bit,
			trigger & IRQ_TYPE_EDGE_FALLING);
	} else {
		BUG();
		return;
	}

	if (likely(!(bank->non_wakeup_gpios & gpio_bit))) {
		if (cpu_is_omap44xx()) {
			if (trigger != 0)
				__raw_writel(1 << gpio, bank->base+
						OMAP4_GPIO_IRQWAKEN0);
			else {
				val = __raw_readl(bank->base +
							OMAP4_GPIO_IRQWAKEN0);
				__raw_writel(val & (~(1 << gpio)), bank->base +
							 OMAP4_GPIO_IRQWAKEN0);
			}
		} else {
			if (trigger != 0)
				__raw_writel(1 << gpio, bank->base
					+ OMAP24XX_GPIO_SETWKUENA);
			else
				__raw_writel(1 << gpio, bank->base
					+ OMAP24XX_GPIO_CLEARWKUENA);
		}
	} else {
		if (trigger != 0)
			bank->enabled_non_wakeup_gpios |= gpio_bit;
		else
			bank->enabled_non_wakeup_gpios &= ~gpio_bit;
	}

	if (cpu_is_omap44xx()) {
		bank->level_mask =
			__raw_readl(bank->base + OMAP4_GPIO_LEVELDETECT0) |
			__raw_readl(bank->base + OMAP4_GPIO_LEVELDETECT1);
	} else {
		bank->level_mask =
			__raw_readl(bank->base + OMAP24XX_GPIO_LEVELDETECT0) |
			__raw_readl(bank->base + OMAP24XX_GPIO_LEVELDETECT1);
	}
}

static int _set_gpio_triggering(struct gpio_bank *bank, int gpio, int trigger)
{
	void __iomem *reg = bank->base;
	u32 l = 0;

	set_24xx_gpio_triggering(bank, gpio, trigger);
	__raw_writel(l, reg);
	return 0;
}

static int gpio_irq_type(unsigned irq, unsigned type)
{
	struct gpio_bank *bank;
	unsigned gpio;
	int retval;
	unsigned long flags;

	gpio = irq - IH_GPIO_BASE;

	if (check_gpio(gpio) < 0)
		return -EINVAL;

	if (type & ~IRQ_TYPE_SENSE_MASK)
		return -EINVAL;

	bank = get_irq_chip_data(irq);
	spin_lock_irqsave(&bank->lock, flags);
	retval = _set_gpio_triggering(bank, get_gpio_index(gpio), type);
	if (retval == 0) {
		irq_desc[irq].status &= ~IRQ_TYPE_SENSE_MASK;
		irq_desc[irq].status |= type;
	}
	spin_unlock_irqrestore(&bank->lock, flags);

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH))
		__set_irq_handler_unlocked(irq, handle_level_irq);
	else if (type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		__set_irq_handler_unlocked(irq, handle_edge_irq);

	return retval;
}

static void _clear_gpio_irqbank(struct gpio_bank *bank, int gpio_mask)
{
	void __iomem *reg = bank->base;

	if (cpu_is_omap24xx() || cpu_is_omap34xx())
		reg += OMAP24XX_GPIO_IRQSTATUS1;
	else if (cpu_is_omap44xx())
		reg += OMAP4_GPIO_IRQSTATUS0;
	else {
		BUG();
		return;
	}
	__raw_writel(gpio_mask, reg);

	/* Workaround for clearing DSP GPIO interrupts to allow retention */
	if (cpu_is_omap24xx() || cpu_is_omap34xx())
		reg = bank->base + OMAP24XX_GPIO_IRQSTATUS2;
	else
		reg = bank->base + OMAP4_GPIO_IRQSTATUS1;
	__raw_writel(gpio_mask, reg);

	/* Flush posted write for the irq status to avoid spurious interrupts */
	__raw_readl(reg);
}

static inline void _clear_gpio_irqstatus(struct gpio_bank *bank, int gpio)
{
	_clear_gpio_irqbank(bank, 1 << get_gpio_index(gpio));
}

static u32 _get_gpio_irqbank_mask(struct gpio_bank *bank)
{
	void __iomem *reg = bank->base;
	int inv = 0;
	u32 l;
	u32 mask;

	if (cpu_is_omap24xx() || cpu_is_omap34xx())
		reg += OMAP24XX_GPIO_IRQENABLE1;
	else if (cpu_is_omap44xx())
		reg += OMAP4_GPIO_IRQSTATUSSET0;
	else {
		BUG();
		return -EINVAL;
	}

	mask = 0xffffffff;
	l = __raw_readl(reg);
	if (inv)
		l = ~l;
	l &= mask;
	return l;
}

static void _enable_gpio_irqbank(struct gpio_bank *bank, int gpio_mask,
					int enable)
{
	void __iomem *reg = bank->base;
	u32 l;

	if (cpu_is_omap24xx() || cpu_is_omap34xx()) {
		if (enable)
			reg += OMAP24XX_GPIO_SETIRQENABLE1;
		else
			reg += OMAP24XX_GPIO_CLEARIRQENABLE1;
	} else if (cpu_is_omap44xx()) {
		if (enable)
			reg += OMAP4_GPIO_IRQSTATUSSET0;
		else
			reg += OMAP4_GPIO_IRQSTATUSCLR0;
	} else {
		BUG();
		return;
	}
	l = gpio_mask;
	__raw_writel(l, reg);
}

static inline void _set_gpio_irqenable(struct gpio_bank *bank, int gpio,
					int enable)
{
	_enable_gpio_irqbank(bank, 1 << get_gpio_index(gpio), enable);
}

/*
 * Note that ENAWAKEUP needs to be enabled in GPIO_SYSCONFIG register.
 * 1510 does not seem to have a wake-up register. If JTAG is connected
 * to the target, system will wake up always on GPIO events. While
 * system is running all registered GPIO interrupts need to have wake-up
 * enabled. When system is suspended, only selected GPIO interrupts need
 * to have wake-up enabled.
 */
static int _set_gpio_wakeup(struct gpio_bank *bank, int gpio, int enable)
{
	unsigned long flags;

	if (bank->non_wakeup_gpios & (1 << gpio)) {
		printk(KERN_ERR "Unable to modify wakeup on "
				"non-wakeup GPIO%d\n",
				(bank - gpio_bank) * 32 + gpio);
		return -EINVAL;
	}
	spin_lock_irqsave(&bank->lock, flags);
	if (enable)
		bank->suspend_wakeup |= (1 << gpio);
	else
		bank->suspend_wakeup &= ~(1 << gpio);
	spin_unlock_irqrestore(&bank->lock, flags);
	return 0;
}

static void _reset_gpio(struct gpio_bank *bank, int gpio)
{
	_set_gpio_direction(bank, get_gpio_index(gpio), 1);
	_set_gpio_irqenable(bank, gpio, 0);
	_clear_gpio_irqstatus(bank, gpio);
	_set_gpio_triggering(bank, get_gpio_index(gpio), IRQ_TYPE_NONE);
}

/* Use disable_irq_wake() and enable_irq_wake() functions from drivers */
static int gpio_wake_enable(unsigned int irq, unsigned int enable)
{
	unsigned int gpio = irq - IH_GPIO_BASE;
	struct gpio_bank *bank;
	int retval;

	if (check_gpio(gpio) < 0)
		return -ENODEV;
	bank = get_irq_chip_data(irq);
	retval = _set_gpio_wakeup(bank, get_gpio_index(gpio), enable);

	return retval;
}

static int omap_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_bank *bank = container_of(chip, struct gpio_bank, chip);
	unsigned long flags;

	spin_lock_irqsave(&bank->lock, flags);

	/* Set trigger to none. You need to enable the desired trigger with
	 * request_irq() or set_irq_type().
	 */
	_set_gpio_triggering(bank, offset, IRQ_TYPE_NONE);

	if (!bank->mod_usage) {
		u32 ctrl;
		ctrl = __raw_readl(bank->base + OMAP24XX_GPIO_CTRL);
		ctrl &= 0xFFFFFFFE;
		/* Module is enabled, clocks are not gated */
		__raw_writel(ctrl, bank->base + OMAP24XX_GPIO_CTRL);
	}
	bank->mod_usage |= 1 << offset;

	spin_unlock_irqrestore(&bank->lock, flags);

	return 0;
}

static void omap_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_bank *bank = container_of(chip, struct gpio_bank, chip);
	unsigned long flags;

	spin_lock_irqsave(&bank->lock, flags);
	{
		/* Disable wake-up during idle for dynamic tick */
		void __iomem *reg = bank->base + OMAP24XX_GPIO_CLEARWKUENA;
		__raw_writel(1 << offset, reg);
	}

	bank->mod_usage &= ~(1 << offset);
	if (!bank->mod_usage) {
		u32 ctrl;
		ctrl = __raw_readl(bank->base + OMAP24XX_GPIO_CTRL);
		/* Module is disabled, clocks are gated */
		ctrl |= 1;
		__raw_writel(ctrl, bank->base + OMAP24XX_GPIO_CTRL);
	}

	_reset_gpio(bank, bank->chip.base + offset);
	spin_unlock_irqrestore(&bank->lock, flags);
}

/*
 * We need to unmask the GPIO bank interrupt as soon as possible to
 * avoid missing GPIO interrupts for other lines in the bank.
 * Then we need to mask-read-clear-unmask the triggered GPIO lines
 * in the bank to avoid missing nested interrupts for a GPIO line.
 * If we wait to unmask individual GPIO lines in the bank after the
 * line's interrupt handler has been run, we may miss some nested
 * interrupts.
 */
static void gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	void __iomem *isr_reg = NULL;
	u32 isr;
	unsigned int gpio_irq;
	struct gpio_bank *bank;
	u32 retrigger = 0;
	int unmasked = 0;

	desc->chip->ack(irq);

	bank = get_irq_data(irq);
	if (cpu_is_omap24xx() || cpu_is_omap34xx())
		isr_reg = bank->base + OMAP24XX_GPIO_IRQSTATUS1;
	else if (cpu_is_omap44xx())
		isr_reg = bank->base + OMAP4_GPIO_IRQSTATUS0;
	else {
		BUG();
		return;
	}

	while (1) {
		u32 isr_saved, level_mask = 0;
		u32 enabled;

		enabled = _get_gpio_irqbank_mask(bank);
		isr = __raw_readl(isr_reg) & enabled;
		isr_saved = isr;
		level_mask = bank->level_mask & enabled;

		/* clear edge sensitive interrupts before handler(s) are
		called so that we don't miss any interrupt occurred while
		executing them */
		_enable_gpio_irqbank(bank, isr_saved & ~level_mask, 0);
		_clear_gpio_irqbank(bank, isr_saved & ~level_mask);
		_enable_gpio_irqbank(bank, isr_saved & ~level_mask, 1);

		/* if there is only edge sensitive GPIO pin interrupts
		configured, we could unmask GPIO bank interrupt immediately */
		if (!level_mask && !unmasked) {
			unmasked = 1;
			desc->chip->unmask(irq);
		}

		isr |= retrigger;
		retrigger = 0;
		if (!isr)
			break;

		gpio_irq = bank->virtual_irq_start;
		for (; isr != 0; isr >>= 1, gpio_irq++) {
			if (!(isr & 1))
				continue;

			generic_handle_irq(gpio_irq);
		}
	}
	/* if bank has any level sensitive GPIO pin interrupt
	configured, we must unmask the bank interrupt only after
	handler(s) are executed in order to avoid spurious bank
	interrupt */
	if (!unmasked)
		desc->chip->unmask(irq);

}

static void gpio_irq_shutdown(unsigned int irq)
{
	unsigned int gpio = irq - IH_GPIO_BASE;
	struct gpio_bank *bank = get_irq_chip_data(irq);

	_reset_gpio(bank, gpio);
}

static void gpio_ack_irq(unsigned int irq)
{
	unsigned int gpio = irq - IH_GPIO_BASE;
	struct gpio_bank *bank = get_irq_chip_data(irq);

	_clear_gpio_irqstatus(bank, gpio);
}

static void gpio_mask_irq(unsigned int irq)
{
	unsigned int gpio = irq - IH_GPIO_BASE;
	struct gpio_bank *bank = get_irq_chip_data(irq);

	_set_gpio_irqenable(bank, gpio, 0);
	_set_gpio_triggering(bank, get_gpio_index(gpio), IRQ_TYPE_NONE);
}

static void gpio_unmask_irq(unsigned int irq)
{
	unsigned int gpio = irq - IH_GPIO_BASE;
	struct gpio_bank *bank = get_irq_chip_data(irq);
	unsigned int irq_mask = 1 << get_gpio_index(gpio);
	struct irq_desc *desc = irq_to_desc(irq);
	u32 trigger = desc->status & IRQ_TYPE_SENSE_MASK;

	if (trigger)
		_set_gpio_triggering(bank, get_gpio_index(gpio), trigger);

	/* For level-triggered GPIOs, the clearing must be done after
	 * the HW source is cleared, thus after the handler has run */
	if (bank->level_mask & irq_mask) {
		_set_gpio_irqenable(bank, gpio, 0);
		_clear_gpio_irqstatus(bank, gpio);
	}

	_set_gpio_irqenable(bank, gpio, 1);
}

static struct irq_chip gpio_irq_chip = {
	.name		= "GPIO",
	.shutdown	= gpio_irq_shutdown,
	.ack		= gpio_ack_irq,
	.mask		= gpio_mask_irq,
	.unmask		= gpio_unmask_irq,
	.set_type	= gpio_irq_type,
	.set_wake	= gpio_wake_enable,
};

static int gpio_input(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_bank *bank;
	unsigned long flags;

	bank = container_of(chip, struct gpio_bank, chip);
	spin_lock_irqsave(&bank->lock, flags);
	_set_gpio_direction(bank, offset, 1);
	spin_unlock_irqrestore(&bank->lock, flags);
	return 0;
}

static int gpio_is_input(struct gpio_bank *bank, int mask)
{
	void __iomem *reg = bank->base;

	reg += OMAP24XX_GPIO_OE;
	return __raw_readl(reg) & mask;
}

static int gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_bank *bank;
	void __iomem *reg;
	int gpio;
	u32 mask;

	gpio = chip->base + offset;
	bank = get_gpio_bank(gpio);
	reg = bank->base;
	mask = 1 << get_gpio_index(gpio);

	if (gpio_is_input(bank, mask))
		return _get_gpio_datain(bank, gpio);
	else
		return _get_gpio_dataout(bank, gpio);
}

static int gpio_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct gpio_bank *bank;
	unsigned long flags;

	bank = container_of(chip, struct gpio_bank, chip);
	spin_lock_irqsave(&bank->lock, flags);
	_set_gpio_dataout(bank, offset, value);
	_set_gpio_direction(bank, offset, 0);
	spin_unlock_irqrestore(&bank->lock, flags);
	return 0;
}

static void gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct gpio_bank *bank;
	unsigned long flags;

	bank = container_of(chip, struct gpio_bank, chip);
	spin_lock_irqsave(&bank->lock, flags);
	_set_gpio_dataout(bank, offset, value);
	spin_unlock_irqrestore(&bank->lock, flags);
}

static int gpio_2irq(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_bank *bank;

	bank = container_of(chip, struct gpio_bank, chip);
	return bank->virtual_irq_start + offset;
}

static void __init omap_gpio_show_rev(void)
{
	u32 rev;

	if (cpu_is_omap24xx() || cpu_is_omap34xx())
		rev = __raw_readl(gpio_bank[0].base + OMAP24XX_GPIO_REVISION);
	else if (cpu_is_omap44xx())
		rev = __raw_readl(gpio_bank[0].base + OMAP4_GPIO_REVISION);
	else
		return;

	printk(KERN_INFO "OMAP GPIO hardware version %d.%d\n",
		(rev >> 4) & 0x0f, rev & 0x0f);
}

/* This lock class tells lockdep that GPIO irqs are in a different
 * category than their parents, so it won't report false recursion.
 */
static struct lock_class_key gpio_lock_class;

static int omap_gpio_suspend(struct sys_device *dev, pm_message_t mesg)
{
	int i;

	if (!cpu_class_is_omap2())
		return 0;

	for (i = 0; i < gpio_bank_count; i++) {
		struct gpio_bank *bank = &gpio_bank[i];
		void __iomem *wake_status;
		void __iomem *wake_clear;
		void __iomem *wake_set;
		unsigned long flags;

		if (cpu_is_omap24xx() || cpu_is_omap34xx()) {
			wake_status = bank->base + OMAP24XX_GPIO_WAKE_EN;
			wake_clear = bank->base + OMAP24XX_GPIO_CLEARWKUENA;
			wake_set = bank->base + OMAP24XX_GPIO_SETWKUENA;
		} else if (cpu_is_omap44xx()) {
			wake_status = bank->base + OMAP4_GPIO_IRQWAKEN0;
			wake_clear = bank->base + OMAP4_GPIO_IRQWAKEN0;
			wake_set = bank->base + OMAP4_GPIO_IRQWAKEN0;
		} else
			return -EINVAL;

		spin_lock_irqsave(&bank->lock, flags);
		bank->saved_wakeup = __raw_readl(wake_status);
		__raw_writel(0xffffffff, wake_clear);
		__raw_writel(bank->suspend_wakeup, wake_set);
		spin_unlock_irqrestore(&bank->lock, flags);
	}

	return 0;
}

static int omap_gpio_resume(struct sys_device *dev)
{
	int i;

	if (!cpu_class_is_omap2())
		return 0;

	for (i = 0; i < gpio_bank_count; i++) {
		struct gpio_bank *bank = &gpio_bank[i];
		void __iomem *wake_clear;
		void __iomem *wake_set;
		unsigned long flags;

		if (cpu_is_omap24xx() || cpu_is_omap34xx()) {
			wake_clear = bank->base + OMAP24XX_GPIO_CLEARWKUENA;
			wake_set = bank->base + OMAP24XX_GPIO_SETWKUENA;
		} else if (cpu_is_omap44xx()) {
			wake_clear = bank->base + OMAP4_GPIO_IRQWAKEN0;
			wake_set = bank->base + OMAP4_GPIO_IRQWAKEN0;
		} else
			return -EINVAL;

		spin_lock_irqsave(&bank->lock, flags);
		__raw_writel(0xffffffff, wake_clear);
		__raw_writel(bank->saved_wakeup, wake_set);
		spin_unlock_irqrestore(&bank->lock, flags);
	}

	return 0;
}

static struct sysdev_class omap_gpio_sysclass = {
	.name		= "gpio",
	.suspend	= omap_gpio_suspend,
	.resume		= omap_gpio_resume,
};

static struct sys_device omap_gpio_device = {
	.id		= 0,
	.cls		= &omap_gpio_sysclass,
};

static int workaround_enabled;

void omap2_gpio_prepare_for_retention(void)
{
	int i, c = 0;

	/* Remove triggering for all non-wakeup GPIOs.  Otherwise spurious
	 * IRQs will be generated.  See OMAP2420 Errata item 1.101. */
	for (i = 0; i < gpio_bank_count; i++) {
		struct gpio_bank *bank = &gpio_bank[i];
		u32 l1, l2;

		if (!(bank->enabled_non_wakeup_gpios))
			continue;
		if (cpu_is_omap24xx() || cpu_is_omap34xx()) {
			bank->saved_datain = __raw_readl(bank->base +
							OMAP24XX_GPIO_DATAIN);
			l1 = __raw_readl(bank->base +
						OMAP24XX_GPIO_FALLINGDETECT);
			l2 = __raw_readl(bank->base +
						OMAP24XX_GPIO_RISINGDETECT);
		} else if (cpu_is_omap44xx()) {
			bank->saved_datain = __raw_readl(bank->base +
							OMAP4_GPIO_DATAIN);
			l1 = __raw_readl(bank->base + OMAP4_GPIO_FALLINGDETECT);
			l2 = __raw_readl(bank->base + OMAP4_GPIO_RISINGDETECT);
		} else
			return;
		bank->saved_fallingdetect = l1;
		bank->saved_risingdetect = l2;
		l1 &= ~bank->enabled_non_wakeup_gpios;
		l2 &= ~bank->enabled_non_wakeup_gpios;
		if (cpu_is_omap24xx()) {
			__raw_writel(l1, bank->base +
						OMAP24XX_GPIO_FALLINGDETECT);
			__raw_writel(l2, bank->base +
						OMAP24XX_GPIO_RISINGDETECT);
		} else if (cpu_is_omap44xx()) {
			__raw_writel(l1, bank->base +
						OMAP4_GPIO_FALLINGDETECT);
			__raw_writel(l2, bank->base +
						OMAP4_GPIO_RISINGDETECT);
		}
		c++;
	}
	if (!c) {
		workaround_enabled = 0;
		return;
	}
	workaround_enabled = 1;
}

void omap2_gpio_resume_after_retention(void)
{
	int i;

	if (!workaround_enabled)
		return;
	for (i = 0; i < gpio_bank_count; i++) {
		struct gpio_bank *bank = &gpio_bank[i];
		u32 l, gen, gen0, gen1;

		if (!(bank->enabled_non_wakeup_gpios))
			continue;
		if (cpu_is_omap24xx() || cpu_is_omap34xx()) {
			__raw_writel(bank->saved_fallingdetect,
					bank->base +
					OMAP24XX_GPIO_FALLINGDETECT);
			__raw_writel(bank->saved_risingdetect,
					bank->base +
					OMAP24XX_GPIO_RISINGDETECT);
			l = __raw_readl(bank->base + OMAP24XX_GPIO_DATAIN);
		} else if (cpu_is_omap44xx()) {
			__raw_writel(bank->saved_fallingdetect,
					bank->base +
					OMAP4_GPIO_FALLINGDETECT);
			__raw_writel(bank->saved_risingdetect,
					bank->base +
					OMAP4_GPIO_RISINGDETECT);
			l = __raw_readl(bank->base + OMAP4_GPIO_DATAIN);
		} else
			return;
		/* Check if any of the non-wakeup interrupt GPIOs have changed
		 * state.  If so, generate an IRQ by software.  This is
		 * horribly racy, but it's the best we can do to work around
		 * this silicon bug. */
		l ^= bank->saved_datain;
		l &= bank->non_wakeup_gpios;

		/*
		 * No need to generate IRQs for the rising edge for gpio IRQs
		 * configured with falling edge only; and vice versa.
		 */
		gen0 = l & bank->saved_fallingdetect;
		gen0 &= bank->saved_datain;

		gen1 = l & bank->saved_risingdetect;
		gen1 &= ~(bank->saved_datain);

		/* FIXME: Consider GPIO IRQs with level detections properly! */
		gen = l & (~(bank->saved_fallingdetect) &
				~(bank->saved_risingdetect));
		/* Consider all GPIO IRQs needed to be updated */
		gen |= gen0 | gen1;

		if (gen) {
			u32 old0, old1;
			if (cpu_is_omap24xx() || cpu_is_omap34xx()) {
				old0 = __raw_readl(bank->base +
						OMAP24XX_GPIO_LEVELDETECT0);
				old1 = __raw_readl(bank->base +
						OMAP24XX_GPIO_LEVELDETECT1);
				__raw_writel(old0 | gen, bank->base +
						OMAP24XX_GPIO_LEVELDETECT0);
				__raw_writel(old1 | gen, bank->base +
						OMAP24XX_GPIO_LEVELDETECT1);
				__raw_writel(old0, bank->base +
						OMAP24XX_GPIO_LEVELDETECT0);
				__raw_writel(old1, bank->base +
						OMAP24XX_GPIO_LEVELDETECT1);
			} else if (cpu_is_omap44xx()) {
				old0 = __raw_readl(bank->base +
						OMAP4_GPIO_LEVELDETECT0);
				old1 = __raw_readl(bank->base +
						OMAP4_GPIO_LEVELDETECT1);
				__raw_writel(old0 | l, bank->base +
						OMAP4_GPIO_LEVELDETECT0);
				__raw_writel(old1 | l, bank->base +
						OMAP4_GPIO_LEVELDETECT1);
				__raw_writel(old0, bank->base +
						OMAP4_GPIO_LEVELDETECT0);
				__raw_writel(old1, bank->base +
						OMAP4_GPIO_LEVELDETECT1);
			}
		}
	}
}

#ifdef CONFIG_ARCH_OMAP3
/* save the registers of bank 2-6 */
void omap_gpio_save_context(void)
{
	int i;

	/* saving banks from 2-6 only since GPIO1 is in WKUP */
	for (i = 1; i < gpio_bank_count; i++) {
		struct gpio_bank *bank = &gpio_bank[i];
		gpio_context[i].sysconfig =
			__raw_readl(bank->base + OMAP24XX_GPIO_SYSCONFIG);
		gpio_context[i].irqenable1 =
			__raw_readl(bank->base + OMAP24XX_GPIO_IRQENABLE1);
		gpio_context[i].irqenable2 =
			__raw_readl(bank->base + OMAP24XX_GPIO_IRQENABLE2);
		gpio_context[i].wake_en =
			__raw_readl(bank->base + OMAP24XX_GPIO_WAKE_EN);
		gpio_context[i].ctrl =
			__raw_readl(bank->base + OMAP24XX_GPIO_CTRL);
		gpio_context[i].oe =
			__raw_readl(bank->base + OMAP24XX_GPIO_OE);
		gpio_context[i].leveldetect0 =
			__raw_readl(bank->base + OMAP24XX_GPIO_LEVELDETECT0);
		gpio_context[i].leveldetect1 =
			__raw_readl(bank->base + OMAP24XX_GPIO_LEVELDETECT1);
		gpio_context[i].risingdetect =
			__raw_readl(bank->base + OMAP24XX_GPIO_RISINGDETECT);
		gpio_context[i].fallingdetect =
			__raw_readl(bank->base + OMAP24XX_GPIO_FALLINGDETECT);
		gpio_context[i].dataout =
			__raw_readl(bank->base + OMAP24XX_GPIO_DATAOUT);
		gpio_context[i].setwkuena =
			__raw_readl(bank->base + OMAP24XX_GPIO_SETWKUENA);
		gpio_context[i].setdataout =
			__raw_readl(bank->base + OMAP24XX_GPIO_SETDATAOUT);
	}
}

/* restore the required registers of bank 2-6 */
void omap_gpio_restore_context(void)
{
	int i;

	for (i = 1; i < gpio_bank_count; i++) {
		struct gpio_bank *bank = &gpio_bank[i];
		__raw_writel(gpio_context[i].sysconfig,
				bank->base + OMAP24XX_GPIO_SYSCONFIG);
		__raw_writel(gpio_context[i].irqenable1,
				bank->base + OMAP24XX_GPIO_IRQENABLE1);
		__raw_writel(gpio_context[i].irqenable2,
				bank->base + OMAP24XX_GPIO_IRQENABLE2);
		__raw_writel(gpio_context[i].wake_en,
				bank->base + OMAP24XX_GPIO_WAKE_EN);
		__raw_writel(gpio_context[i].ctrl,
				bank->base + OMAP24XX_GPIO_CTRL);
		__raw_writel(gpio_context[i].oe,
				bank->base + OMAP24XX_GPIO_OE);
		__raw_writel(gpio_context[i].leveldetect0,
				bank->base + OMAP24XX_GPIO_LEVELDETECT0);
		__raw_writel(gpio_context[i].leveldetect1,
				bank->base + OMAP24XX_GPIO_LEVELDETECT1);
		__raw_writel(gpio_context[i].risingdetect,
				bank->base + OMAP24XX_GPIO_RISINGDETECT);
		__raw_writel(gpio_context[i].fallingdetect,
				bank->base + OMAP24XX_GPIO_FALLINGDETECT);
		__raw_writel(gpio_context[i].dataout,
				bank->base + OMAP24XX_GPIO_DATAOUT);
		__raw_writel(gpio_context[i].setwkuena,
				bank->base + OMAP24XX_GPIO_SETWKUENA);
		__raw_writel(gpio_context[i].setdataout,
				bank->base + OMAP24XX_GPIO_SETDATAOUT);
	}
}
#endif

static int __devexit omap_gpio_remove(struct platform_device *pdev)
{
	struct omap_gpio_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_bank *bank;
	int id;

	if (!pdev || !pdata)
		return 0;

	id = pdev->id;
	if (id > gpio_bank_count)
		return 0;

	bank = &gpio_bank[id];
	if (cpu_is_omap24xx()) {
		clk_disable(bank->fck);
		clk_put(bank->fck);
	}
	clk_disable(bank->ick);
	clk_put(bank->ick);

	bank->ick = NULL;
	bank->fck = NULL;
	bank->initialized = 0;
	iounmap(bank->base);

	return 0;
}

static int __devinit omap_gpio_probe(struct platform_device *pdev)
{
	static int show_rev_once;
	struct omap_gpio_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_bank *bank;
	struct resource *res;
	int id, i;

	if (!pdev || !pdata) {
		printk(KERN_ERR "GPIO device initialize without"
					"platform data\n");
		return -EINVAL;
	}

	if (cpu_is_omap242x())
		gpio_bank_count = OMAP242X_NR_GPIOS;
	else if (cpu_is_omap243x())
		gpio_bank_count = OMAP243X_NR_GPIOS;
	else if (cpu_is_omap34xx())
		gpio_bank_count = OMAP34XX_NR_GPIOS;
	else if (cpu_is_omap44xx())
		gpio_bank_count = OMAP44XX_NR_GPIOS;

	id = pdev->id;
	if (id > gpio_bank_count) {
		printk(KERN_ERR "Invalid GPIO device id (%d)\n", id);
		return -EINVAL;
	}

	bank = &gpio_bank[id];

	if (bank->initialized == 1)
		return 0;

	bank->virtual_irq_start = pdata->virtual_irq_start;

	bank->ick = clk_get(NULL, pdata->ick_name);
	if (IS_ERR(bank->ick))
		printk(KERN_ERR "Could not get %s\n", pdata->ick_name);
	else
		clk_enable(bank->ick);

	if (cpu_is_omap24xx()) {
		bank->fck = clk_get(NULL, pdata->fck_name);
		if (IS_ERR(bank->fck))
			printk(KERN_ERR "Could not get %s\n", pdata->fck_name);
		else
			clk_enable(bank->fck);
	}

	spin_lock_init(&bank->lock);

	/* Static mapping, never released */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!res)) {
		printk(KERN_ERR "GPIO Bank %i Invalid mem resource\n", id);
		return -ENODEV;
	}

	bank->base = ioremap(res->start, resource_size(res));
	if (!bank->base) {
		printk(KERN_ERR "Could not ioremap gpio bank%i\n", id);
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (unlikely(!res)) {
		printk(KERN_ERR "GPIO Bank %i Invalid irq resource\n", id);
		return -ENODEV;
	}

	if (cpu_is_omap44xx()) {
		__raw_writel(0xffffffff, bank->base +
					OMAP4_GPIO_IRQSTATUSCLR0);
		__raw_writew(0x0015, bank->base +
					OMAP4_GPIO_SYSCONFIG);
		__raw_writel(0x00000000, bank->base +
					 OMAP4_GPIO_DEBOUNCENABLE);
		/* Initialize interface clk ungated, module enabled */
		__raw_writel(0, bank->base + OMAP4_GPIO_CTRL);
	} else {
		__raw_writel(0x00000000, bank->base +
					OMAP24XX_GPIO_IRQENABLE1);
		__raw_writel(0xffffffff, bank->base +
					OMAP24XX_GPIO_IRQSTATUS1);
		__raw_writew(0x0015, bank->base +
					OMAP24XX_GPIO_SYSCONFIG);
		__raw_writel(0x00000000, bank->base +
					OMAP24XX_GPIO_DEBOUNCE_EN);

		/* Initialize interface clk ungated, module enabled */
		__raw_writel(0, bank->base + OMAP24XX_GPIO_CTRL);
	}
	{
		static const u32 non_wakeup_gpios[] = {
			0xe203ffc0, 0x08700040
		};
		if (id < ARRAY_SIZE(non_wakeup_gpios))
			bank->non_wakeup_gpios = non_wakeup_gpios[id];
	}

	bank->mod_usage = 0;
	/* REVISIT eventually switch from OMAP-specific gpio structs
	 * over to the generic ones
	 */
	bank->chip.request = omap_gpio_request;
	bank->chip.free = omap_gpio_free;
	bank->chip.direction_input = gpio_input;
	bank->chip.get = gpio_get;
	bank->chip.direction_output = gpio_output;
	bank->chip.set = gpio_set;
	bank->chip.to_irq = gpio_2irq;
	bank->chip.label = "gpio";
	bank->chip.base = id * 32;
	bank->chip.ngpio = 32;

	gpiochip_add(&bank->chip);

	for (i = bank->virtual_irq_start; i < bank->virtual_irq_start + 32;
			i++) {
		lockdep_set_class(&irq_desc[i].lock, &gpio_lock_class);
		set_irq_chip_data(i, bank);
		set_irq_chip(i, &gpio_irq_chip);
		set_irq_handler(i, handle_simple_irq);
		set_irq_flags(i, IRQF_VALID);
	}
	set_irq_chained_handler(res->start, gpio_irq_handler);
	set_irq_data(res->start, bank);

	if (cpu_is_omap34xx()) {
		bank->dbck = clk_get(NULL, pdata->dbck_name);
		if (IS_ERR(bank->dbck))
			printk(KERN_ERR "Could not get %s\n", pdata->dbck_name);
	}

	/* Enable autoidle for the OCP interface */
	if (cpu_is_omap24xx())
		omap_writel(1 << 0, 0x48019010);
	if (cpu_is_omap34xx())
		omap_writel(1 << 0, 0x48306814);

	bank->initialized = 1;

	if (!show_rev_once) {
		omap_gpio_show_rev();
		show_rev_once = 1;
	}

	return 0;
}

void __init omap_gpio_early_init(void)
{
	struct platform_device **pdev;

	if (cpu_is_omap242x()) {
		pdev = omap242x_gpio_early_dev;
		gpio_bank_count = OMAP242X_NR_GPIOS;
	} else if (cpu_is_omap243x()) {
		pdev = omap243x_gpio_early_dev;
		gpio_bank_count = OMAP243X_NR_GPIOS;
	} else if (cpu_is_omap34xx()) {
		pdev = omap3_gpio_early_dev;
		gpio_bank_count = OMAP34XX_NR_GPIOS;
	} else if (cpu_is_omap44xx()) {
		pdev = omap4_gpio_early_dev;
		gpio_bank_count = OMAP44XX_NR_GPIOS;
	}

	early_platform_add_devices(pdev, gpio_bank_count);
	early_platform_driver_register_all("earlygpio");
	early_platform_driver_probe("earlygpio", gpio_bank_count, 0);
}

int __init omap_init_gpio(void)
{
	if (cpu_is_omap242x()) {
		platform_device_register(&omap242x_gpio1);
		platform_device_register(&omap242x_gpio2);
		platform_device_register(&omap242x_gpio3);
		platform_device_register(&omap242x_gpio4);
	} else if (cpu_is_omap243x()) {
		platform_device_register(&omap243x_gpio1);
		platform_device_register(&omap243x_gpio2);
		platform_device_register(&omap243x_gpio3);
		platform_device_register(&omap243x_gpio4);
		platform_device_register(&omap243x_gpio5);
	} else if (cpu_is_omap34xx()) {
		platform_device_register(&omap3_gpio1);
		platform_device_register(&omap3_gpio2);
		platform_device_register(&omap3_gpio3);
		platform_device_register(&omap3_gpio4);
		platform_device_register(&omap3_gpio5);
		platform_device_register(&omap3_gpio6);
	} else if (cpu_is_omap44xx()) {
		platform_device_register(&omap4_gpio1);
		platform_device_register(&omap4_gpio2);
		platform_device_register(&omap4_gpio3);
		platform_device_register(&omap4_gpio4);
		platform_device_register(&omap4_gpio5);
		platform_device_register(&omap4_gpio6);
	}
	return 0;
}

static struct platform_driver omap_gpio_driver = {
	.probe		= omap_gpio_probe,
	.remove		= __devexit_p(omap_gpio_remove),
	.driver		= {
		.name	= "omap-gpio",
	},
};

/*
 */
int __init omap_gpio_init(void)
{
	return platform_driver_register(&omap_gpio_driver);
}

static int __init omap_gpio_sysinit(void)
{
	int ret = 0;

	ret = omap_gpio_init();
	if (ret == 0) {
		ret = sysdev_class_register(&omap_gpio_sysclass);
		if (ret == 0)
			ret = sysdev_register(&omap_gpio_device);
	}

	return ret;
}

static void __exit omap_gpio_exit(void)
{
	platform_driver_unregister(&omap_gpio_driver);
}
early_platform_init("earlygpio", &omap_gpio_driver);
arch_initcall(omap_init_gpio);
module_init(omap_gpio_sysinit);
module_exit(omap_gpio_exit);

#ifdef	CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int dbg_gpio_show(struct seq_file *s, void *unused)
{
	unsigned	i, j, gpio;

	for (i = 0, gpio = 0; i < gpio_bank_count; i++) {
		struct gpio_bank	*bank = gpio_bank + i;
		unsigned		bankwidth = 32;
		u32			mask = 1;

		for (j = 0; j < bankwidth; j++, gpio++, mask <<= 1) {
			unsigned	irq, value, is_in, irqstat;
			const char	*label;

			label = gpiochip_is_requested(&bank->chip, j);
			if (!label)
				continue;

			irq = bank->virtual_irq_start + j;
			value = gpio_get_value(gpio);
			is_in = gpio_is_input(bank, mask);

			seq_printf(s, "GPIO %3d ", gpio);
			seq_printf(s, "(%-20.20s): %s %s",
					label,
					is_in ? "in " : "out",
					value ? "hi"  : "lo");

			/* FIXME show pullup/pulldown state */

			irqstat = irq_desc[irq].status;
			if (is_in && ((bank->suspend_wakeup & mask)
					|| irqstat & IRQ_TYPE_SENSE_MASK)) {
				char	*trigger = NULL;

				switch (irqstat & IRQ_TYPE_SENSE_MASK) {
				case IRQ_TYPE_EDGE_FALLING:
					trigger = "falling";
					break;
				case IRQ_TYPE_EDGE_RISING:
					trigger = "rising";
					break;
				case IRQ_TYPE_EDGE_BOTH:
					trigger = "bothedge";
					break;
				case IRQ_TYPE_LEVEL_LOW:
					trigger = "low";
					break;
				case IRQ_TYPE_LEVEL_HIGH:
					trigger = "high";
					break;
				case IRQ_TYPE_NONE:
					trigger = "(?)";
					break;
				}
				seq_printf(s, ", irq-%d %-8s%s",
						irq, trigger,
						(bank->suspend_wakeup & mask)
							? " wakeup" : "");
			}
			seq_printf(s, "\n");
		}
	}
	return 0;
}

static int dbg_gpio_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_gpio_show, &inode->i_private);
}

static const struct file_operations debug_fops = {
	.open		= dbg_gpio_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init omap_gpio_debuginit(void)
{
	(void) debugfs_create_file("omap_gpio", S_IRUGO,
					NULL, NULL, &debug_fops);
	return 0;
}
late_initcall(omap_gpio_debuginit);
#endif

MODULE_DESCRIPTION("OMAP GPIO DRIVER");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc");
