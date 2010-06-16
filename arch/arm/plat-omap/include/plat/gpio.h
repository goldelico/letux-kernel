/*
 * arch/arm/plat-omap/include/mach/gpio.h
 *
 * OMAP GPIO handling defines and functions
 *
 * Copyright (C) 2003-2005 Nokia Corporation
 *
 * Written by Juha Yrjölä <juha.yrjola@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#ifndef __ASM_ARCH_OMAP_GPIO_H
#define __ASM_ARCH_OMAP_GPIO_H

#include <linux/io.h>
#include <linux/sysdev.h>
#include <mach/irqs.h>

#define OMAP1_MPUIO_BASE			0xfffb5000

#if (defined(CONFIG_ARCH_OMAP730) || defined(CONFIG_ARCH_OMAP850))

#define OMAP_MPUIO_INPUT_LATCH		0x00
#define OMAP_MPUIO_OUTPUT		0x02
#define OMAP_MPUIO_IO_CNTL		0x04
#define OMAP_MPUIO_KBR_LATCH		0x08
#define OMAP_MPUIO_KBC			0x0a
#define OMAP_MPUIO_GPIO_EVENT_MODE	0x0c
#define OMAP_MPUIO_GPIO_INT_EDGE	0x0e
#define OMAP_MPUIO_KBD_INT		0x10
#define OMAP_MPUIO_GPIO_INT		0x12
#define OMAP_MPUIO_KBD_MASKIT		0x14
#define OMAP_MPUIO_GPIO_MASKIT		0x16
#define OMAP_MPUIO_GPIO_DEBOUNCING	0x18
#define OMAP_MPUIO_LATCH		0x1a
#else
#define OMAP_MPUIO_INPUT_LATCH		0x00
#define OMAP_MPUIO_OUTPUT		0x04
#define OMAP_MPUIO_IO_CNTL		0x08
#define OMAP_MPUIO_KBR_LATCH		0x10
#define OMAP_MPUIO_KBC			0x14
#define OMAP_MPUIO_GPIO_EVENT_MODE	0x18
#define OMAP_MPUIO_GPIO_INT_EDGE	0x1c
#define OMAP_MPUIO_KBD_INT		0x20
#define OMAP_MPUIO_GPIO_INT		0x24
#define OMAP_MPUIO_KBD_MASKIT		0x28
#define OMAP_MPUIO_GPIO_MASKIT		0x2c
#define OMAP_MPUIO_GPIO_DEBOUNCING	0x30
#define OMAP_MPUIO_LATCH		0x34
#endif

#if defined(CONFIG_ARCH_OMAP2)
#define OMAP_NR_GPIOS			5
#define OMAP242X_NR_GPIOS		4
#elif defined(CONFIG_ARCH_OMAP3)
#define OMAP_NR_GPIOS			6
#elif defined(CONFIG_ARCH_OMAP4)
#define OMAP_NR_GPIOS			6
#endif

#define OMAP_MPUIO(nr)		(OMAP_MAX_GPIO_LINES + (nr))
#define OMAP_GPIO_IS_MPUIO(nr)	((nr) >= OMAP_MAX_GPIO_LINES)

#define OMAP_GPIO_IRQ(nr)	(OMAP_GPIO_IS_MPUIO(nr) ? \
				 IH_MPUIO_BASE + ((nr) & 0x0f) : \
				 IH_GPIO_BASE + (nr))

#if defined(CONFIG_ARCH_OMAP2) || defined(CONFIG_ARCH_OMAP3) || \
			defined(CONFIG_ARCH_OMAP4)
/*
 * omap24xx specific GPIO registers
 */
#define OMAP242X_GPIO1_BASE		(L4_24XX_BASE + 0x18000)
#define OMAP242X_GPIO2_BASE		(L4_24XX_BASE + 0x1a000)
#define OMAP242X_GPIO3_BASE		(L4_24XX_BASE + 0x1c000)
#define OMAP242X_GPIO4_BASE		(L4_24XX_BASE + 0x1e000)

#define OMAP243X_GPIO1_BASE		(L4_WK_243X_BASE + 0x0c000)
#define OMAP243X_GPIO2_BASE		(L4_WK_243X_BASE + 0x0e000)
#define OMAP243X_GPIO3_BASE		(L4_WK_243X_BASE + 0x10000)
#define OMAP243X_GPIO4_BASE		(L4_WK_243X_BASE + 0x12000)
#define OMAP243X_GPIO5_BASE		(L4_24XX_BASE + 0xb6000)

#define OMAP2_GPIO_AS_LEN		4096

#define OMAP24XX_GPIO_REVISION		0x0000
#define OMAP24XX_GPIO_SYSCONFIG		0x0010
#define OMAP24XX_GPIO_SYSSTATUS		0x0014
#define OMAP24XX_GPIO_IRQSTATUS1	0x0018
#define OMAP24XX_GPIO_IRQSTATUS2	0x0028
#define OMAP24XX_GPIO_IRQENABLE2	0x002c
#define OMAP24XX_GPIO_IRQENABLE1	0x001c
#define OMAP24XX_GPIO_WAKE_EN		0x0020
#define OMAP24XX_GPIO_CTRL		0x0030
#define OMAP24XX_GPIO_OE		0x0034
#define OMAP24XX_GPIO_DATAIN		0x0038
#define OMAP24XX_GPIO_DATAOUT		0x003c
#define OMAP24XX_GPIO_LEVELDETECT0	0x0040
#define OMAP24XX_GPIO_LEVELDETECT1	0x0044
#define OMAP24XX_GPIO_RISINGDETECT	0x0048
#define OMAP24XX_GPIO_FALLINGDETECT	0x004c
#define OMAP24XX_GPIO_DEBOUNCE_EN	0x0050
#define OMAP24XX_GPIO_DEBOUNCE_VAL	0x0054
#define OMAP24XX_GPIO_CLEARIRQENABLE1	0x0060
#define OMAP24XX_GPIO_SETIRQENABLE1	0x0064
#define OMAP24XX_GPIO_CLEARWKUENA	0x0080
#define OMAP24XX_GPIO_SETWKUENA		0x0084
#define OMAP24XX_GPIO_CLEARDATAOUT	0x0090
#define OMAP24XX_GPIO_SETDATAOUT	0x0094

/*
 * omap34xx specific GPIO registers
 */
#define OMAP34XX_GPIO1_BASE	(L4_WK_34XX_BASE + 0x10000)
#define OMAP34XX_GPIO2_BASE	(L4_PER_34XX_BASE + 0x50000)
#define OMAP34XX_GPIO3_BASE	(L4_PER_34XX_BASE + 0x52000)
#define OMAP34XX_GPIO4_BASE	(L4_PER_34XX_BASE + 0x54000)
#define OMAP34XX_GPIO5_BASE	(L4_PER_34XX_BASE + 0x56000)
#define OMAP34XX_GPIO6_BASE	(L4_PER_34XX_BASE + 0x58000)

#define OMAP3_GPIO_AS_LEN		4096

/*
 * OMAP44XX  specific GPIO registers
 */
#define OMAP44XX_GPIO1_BASE	(L4_WK_44XX_BASE + 0x10000)
#define OMAP44XX_GPIO2_BASE	(L4_PER_44XX_BASE + 0x55000)
#define OMAP44XX_GPIO3_BASE	(L4_PER_44XX_BASE + 0x57000)
#define OMAP44XX_GPIO4_BASE	(L4_PER_44XX_BASE + 0x59000)
#define OMAP44XX_GPIO5_BASE	(L4_PER_44XX_BASE + 0x5b000)
#define OMAP44XX_GPIO6_BASE	(L4_PER_44XX_BASE + 0x5d000)

#define OMAP4_GPIO_AS_LEN		4096
#define OMAP4_GPIO_REVISION		0x0000
#define OMAP4_GPIO_SYSCONFIG		0x0010
#define OMAP4_GPIO_EOI			0x0020
#define OMAP4_GPIO_IRQSTATUSRAW0	0x0024
#define OMAP4_GPIO_IRQSTATUSRAW1	0x0028
#define OMAP4_GPIO_IRQSTATUS0		0x002c
#define OMAP4_GPIO_IRQSTATUS1		0x0030
#define OMAP4_GPIO_IRQSTATUSSET0	0x0034
#define OMAP4_GPIO_IRQSTATUSSET1	0x0038
#define OMAP4_GPIO_IRQSTATUSCLR0	0x003c
#define OMAP4_GPIO_IRQSTATUSCLR1	0x0040
#define OMAP4_GPIO_IRQWAKEN0		0x0044
#define OMAP4_GPIO_IRQWAKEN1		0x0048
#define OMAP4_GPIO_SYSSTATUS		0x0114
#define OMAP4_GPIO_IRQENABLE1		0x011c
#define OMAP4_GPIO_WAKE_EN		0x0120
#define OMAP4_GPIO_IRQSTATUS2		0x0128
#define OMAP4_GPIO_IRQENABLE2		0x012c
#define OMAP4_GPIO_CTRL			0x0130
#define OMAP4_GPIO_OE			0x0134
#define OMAP4_GPIO_DATAIN		0x0138
#define OMAP4_GPIO_DATAOUT		0x013c
#define OMAP4_GPIO_LEVELDETECT0		0x0140
#define OMAP4_GPIO_LEVELDETECT1		0x0144
#define OMAP4_GPIO_RISINGDETECT		0x0148
#define OMAP4_GPIO_FALLINGDETECT	0x014c
#define OMAP4_GPIO_DEBOUNCENABLE	0x0150
#define OMAP4_GPIO_DEBOUNCINGTIME	0x0154
#define OMAP4_GPIO_CLEARIRQENABLE1	0x0160
#define OMAP4_GPIO_SETIRQENABLE1	0x0164
#define OMAP4_GPIO_CLEARWKUENA		0x0180
#define OMAP4_GPIO_SETWKUENA		0x0184
#define OMAP4_GPIO_CLEARDATAOUT		0x0190
#define OMAP4_GPIO_SETDATAOUT		0x0194

extern void omap_gpio_early_init(void);
extern int omap_gpio_init(void);	/* Call from board init only */
extern void omap2_gpio_prepare_for_idle(int power_state);
extern void omap2_gpio_resume_after_idle(void);
extern void omap_set_gpio_debounce(int gpio, int enable);
extern void omap_set_gpio_debounce_time(int gpio, int enable);
extern void omap_gpio_save_context(void);
extern void omap_gpio_restore_context(void);
extern void omap3_gpio_restore_pad_context(int restore_oe);
#endif
/*-------------------------------------------------------------------------*/

/* Wrappers for "new style" GPIO calls, using the new infrastructure
 * which lets us plug in FPGA, I2C, and other implementations.
 * *
 * The original OMAP-specfic calls should eventually be removed.
 */

#include <linux/errno.h>
#include <asm-generic/gpio.h>
#include <linux/platform_device.h>

struct omap_gpio_platform_data {
	void __iomem *base;
	u16 irq;
	u16 virtual_irq_start;
	int (*device_enable)(struct platform_device *pdev);
	int (*device_shutdown) (struct platform_device *pdev);
	int (*device_idle)(struct platform_device *pdev);
};

struct gpio_bank {
#if defined(CONFIG_ARCH_OMAP15XX) || defined(CONFIG_ARCH_OMAP16XX) || \
		defined(CONFIG_ARCH_OMAP730) || defined(CONFIG_ARCH_OMAP850)
	unsigned long pbase;
	void __iomem *base;
	u16 irq;
	u16 virtual_irq_start;
	int method;
#endif
	u32 suspend_wakeup;
	u32 saved_wakeup;
	u32 level_mask;
	spinlock_t lock; /* for locking in GPIO operations */
	struct gpio_chip chip;
#if defined(CONFIG_ARCH_OMAP2) || defined(CONFIG_ARCH_OMAP3) || \
			defined(CONFIG_ARCH_OMAP4)
	u32 non_wakeup_gpios;
	u32 enabled_non_wakeup_gpios;
	u32 saved_datain;
	u32 saved_fallingdetect;
	u32 saved_risingdetect;
	u32 mod_usage;
	u8 initialized;
	struct clk *dbck;
	void __iomem *base;
	u16 virtual_irq_start;
	int (*device_enable)(struct platform_device *pdev);
	int (*device_shutdown) (struct platform_device *pdev);
	int (*device_idle)(struct platform_device *pdev);
	u32 dbck_enable_mask;
#endif
};

static inline int gpio_get_value(unsigned gpio)
{
	return __gpio_get_value(gpio);
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	__gpio_set_value(gpio, value);
}

static inline int gpio_cansleep(unsigned gpio)
{
	return __gpio_cansleep(gpio);
}

static inline int gpio_to_irq(unsigned gpio)
{
	return __gpio_to_irq(gpio);
}

static inline int irq_to_gpio(unsigned irq)
{
	int tmp;

	/* omap1 SOC mpuio */
	if (cpu_class_is_omap1() && (irq < (IH_MPUIO_BASE + 16)))
		return (irq - IH_MPUIO_BASE) + OMAP_MAX_GPIO_LINES;

	/* SOC gpio */
	tmp = irq - IH_GPIO_BASE;
	if (tmp < OMAP_MAX_GPIO_LINES)
		return tmp;

	/* we don't supply reverse mappings for non-SOC gpios */
	return -EIO;
}

#endif
