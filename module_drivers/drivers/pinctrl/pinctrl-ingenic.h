/*
 * pinctrl/ingenic/pinctrl-ingenic.c
 *
 * Copyright 2015 Ingenic Semiconductor Co.,Ltd
 *
 * Author: cli <chen.li@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __PINCTRL_INGENIC_H__
#define __PINCTRL_INGENIC_H__

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/irqdomain.h>
#include <linux/gpio.h>
#include <linux/pinctrl/pinctrl.h>
#include <dt-bindings/pinctrl/ingenic-pinctrl.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include "core.h"

enum gpio_function {
	GPIO_FUNC_0	= 0x00,  //0000, GPIO as function 0 / device 0
	GPIO_FUNC_1	= 0x01,  //0001, GPIO as function 1 / device 1
	GPIO_FUNC_2	= 0x02,  //0010, GPIO as function 2 / device 2
	GPIO_FUNC_3	= 0x03,  //0011, GPIO as function 3 / device 3
	GPIO_OUTPUT0	= 0x04,  //0100, GPIO output low  level
	GPIO_OUTPUT1	= 0x05,  //0101, GPIO output high level
	GPIO_INPUT	= 0x06,  //0110, GPIO as input
	GPIO_INT_LO 	= 0x08,  //1100, Low  Level trigger interrupt
	GPIO_INT_HI 	= 0x09,  //1101, High Level trigger interrupt
	GPIO_INT_FE 	= 0x0a,  //1110, Fall Edge trigger interrupt
	GPIO_INT_RE 	= 0x0b,  //1111, Rise Edge trigger interrupt
	GPIO_INT_RE_FE 	= 0x1a,  //Fall&Rise Edge trigger interrupt
};

#define PxPIN           0x00   /* PIN Level Register */
#define PxINT           0x10   /* Port Interrupt Register */
#define PxINTS          0x14   /* Port Interrupt Set Register */
#define PxINTC          0x18   /* Port Interrupt Clear Register */
#define PxMSK           0x20   /* Port Interrupt Mask Reg */
#define PxMSKS          0x24   /* Port Interrupt Mask Set Reg */
#define PxMSKC          0x28   /* Port Interrupt Mask Clear Reg */
#define PxPAT1          0x30   /* Port Pattern 1 Set Reg. */
#define PxPAT1S         0x34   /* Port Pattern 1 Set Reg. */
#define PxPAT1C         0x38   /* Port Pattern 1 Clear Reg. */
#define PxPAT0          0x40   /* Port Pattern 0 Register */
#define PxPAT0S         0x44   /* Port Pattern 0 Set Register */
#define PxPAT0C         0x48   /* Port Pattern 0 Clear Register */
#define PxFLG           0x50   /* Port Flag Register */
#define PxFLGC          0x58   /* Port Flag clear Register */

/* TODO: move these regs to soc special. */

#if defined(CONFIG_SOC_X1520)
	#define PxPEN	0x60
	#define PxPENS	0x64
	#define PxPENC	0x68
#elif defined(CONFIG_SOC_X1600)
	#define PxPEN	0x80
	#define PxPENS	0x84
	#define PxPENC	0x88
#else
	#define PxPEN	-1
	#define PxPENS	-1
	#define PxPENC	-1
#endif

/* Note: Not All Socs have these regs. */
#if defined(CONFIG_SOC_X2000_V12) || defined(CONFIG_SOC_X2000)	\
    || defined(CONFIG_SOC_X2100) || defined(CONFIG_SOC_M300)	\
    || defined(CONFIG_SOC_X1600)
	#define PxEDG		0x70	/* PORT Dual-Edge Interrupt Register */
	#define PxEDGS		0x74	/* PORT Dual-Edge Interrupt Register Set */
	#define PxEDGC		0x78	/* PORT Dual-Edge Interrupt Register Clear */
#else
	#define PxEDG		-1
	#define PxEDGS		-1
	#define PxEDGC		-1
#endif


/* Soc Secial. X2000 s.*/
#if defined(CONFIG_SOC_X2000_V12) || defined(CONFIG_SOC_X2000)	\
    || defined(CONFIG_SOC_X2100) || defined(CONFIG_SOC_M300)	\
    || defined(CONFIG_SOC_X1600)
	#define PxPU		0x80	/* PORT PULL-UP State Register */
	#define PxPUS		0x84	/* PORT PULL-UP State Register Set */
	#define PxPUC		0x88	/* PORT PULL-UP State Register Clear */
	#define PxPD		0x90	/* PORT PULL-DOWN State Register */
	#define PxPDS		0x94	/* PORT PULL-DOWN State Register Set */
	#define PxPDC		0x98	/* PORT PULL-DOWN State Register Clear */

#elif defined(CONFIG_SOC_X2500)
/* Soc Secial. T40/X2500 s.*/
	#define PxPU		0x110	/* PORT PULL-UP State Register */
	#define PxPUS		0x114	/* PORT PULL-UP State Register Set */
	#define PxPUC		0x118	/* PORT PULL-UP State Register Clear */
	#define PxPD		0x120	/* PORT PULL-DOWN State Register */
	#define PxPDS		0x124	/* PORT PULL-DOWN State Register Set */
	#define PxPDC		0x128	/* PORT PULL-DOWN State Register Clear */
#else
	#define PxPU		-1	/* PORT PULL-UP State Register */
	#define PxPUS		-1	/* PORT PULL-UP State Register Set */
	#define PxPUC		-1	/* PORT PULL-UP State Register Clear */
	#define PxPD		-1	/* PORT PULL-DOWN State Register */
	#define PxPDS		-1	/* PORT PULL-DOWN State Register Set */
	#define PxPDC		-1	/* PORT PULL-DOWN State Register Clear */
#endif


#if defined(CONFIG_SOC_X2000_V12) || defined(CONFIG_SOC_X2000)	\
    || defined(CONFIG_SOC_X2100) || defined(CONFIG_SOC_M300)
/* Soc Special. X2000 */
	#define PXDS0		0xA0   /* PORT Drive Strength State Register0*/
	#define PXDS0S		0xA4   /* PORT Drive Strength State set Register0*/
	#define PXDS0C		0xA8   /* PORT Drive Strength State clear Register0*/
	#define PXDS1		0xB0   /* PORT Drive Strength State Register1*/
	#define PXDS1S		0xB4   /* PORT Drive Strength State set Register1*/
	#define PXDS1C		0xB8   /* PORT Drive Strength State clear Register1*/
	#define PXDS2		0xC0   /* PORT Drive Strength State Register2*/
	#define PXDS2S		0xC4   /* PORT Drive Strength State set Register2*/
	#define PXDS2C		0xC8   /* PORT Drive Strength State clear Register2*/

#elif defined(CONFIG_SOC_X2500)
/* T40/X2500. */
	#define PXDS0		0x130   /* PORT Drive Strength State Register0*/
	#define PXDS0S		0x134   /* PORT Drive Strength State set Register0*/
	#define PXDS0C		0x138   /* PORT Drive Strength State clear Register0*/
	#define PXDS1		0x140   /* PORT Drive Strength State Register1*/
	#define PXDS1S		0x144   /* PORT Drive Strength State set Register1*/
	#define PXDS1C		0x148   /* PORT Drive Strength State clear Register1*/
	#define PXDS2		0x150   /* PORT Drive Strength State Register2*/
	#define PXDS2S		0x154   /* PORT Drive Strength State set Register2*/
	#define PXDS2C		0x158   /* PORT Drive Strength State clear Register2*/
#else
	#define PXDS0		-1   /* PORT Drive Strength State Register0*/
	#define PXDS0S		-1   /* PORT Drive Strength State set Register0*/
	#define PXDS0C		-1   /* PORT Drive Strength State clear Register0*/
	#define PXDS1		-1   /* PORT Drive Strength State Register1*/
	#define PXDS1S		-1   /* PORT Drive Strength State set Register1*/
	#define PXDS1C		-1   /* PORT Drive Strength State clear Register1*/
	#define PXDS2		-1   /* PORT Drive Strength State Register2*/
	#define PXDS2S		-1   /* PORT Drive Strength State set Register2*/
	#define PXDS2C		-1   /* PORT Drive Strength State clear Register2*/
#endif

/* Soc Special. */

#if defined(CONFIG_SOC_X2000_V12) || defined(CONFIG_SOC_X2000)	\
    || defined(CONFIG_SOC_X2100) || defined(CONFIG_SOC_M300)
	#define PxSR		0xD0    /* PORT Slew Rate Register */
	#define PxSRS		0xD4	/* PORT Slew Rate Register Set */
	#define PxSRC		0xD8	/* PORT Slew Rate Register Clear */
#elif defined(CONFIG_SOC_X2500)
	#define PxSR		0x160    /* PORT Slew Rate Register */
	#define PxSRS		0x164	/* PORT Slew Rate Register Set */
	#define PxSRC		0x168	/* PORT Slew Rate Register Clear */
#else
	#define PxSR		-1    /* PORT Slew Rate Register */
	#define PxSRS		-1	/* PORT Slew Rate Register Set */
	#define PxSRC		-1	/* PORT Slew Rate Register Clear */
#endif

#if defined(CONFIG_SOC_X2000_V12) || defined(CONFIG_SOC_X2000)	\
    || defined(CONFIG_SOC_X2100) || defined(CONFIG_SOC_M300)
	#define PxSMT		0xE0	/* PORT Schmitt Trigger Register */
	#define PxSMTS		0xE4	/* PORT Schmitt Trigger Register Set */
	#define PxSMTC		0xE8	/*  PORT Schmitt Trigger Register Clear */
#elif defined(CONFIG_SOC_X2500)
	#define PxSMT		0x170    /* PORT Slew Rate Register */
	#define PxSMTS		0x174	/* PORT Slew Rate Register Set */
	#define PxSMTC		0x178	/* PORT Slew Rate Register Clear */
// TODO: X2500 has different schmit registers for GPC.

#else
	#define PxSMT		-1    /* PORT Slew Rate Register */
	#define PxSMTS		-1	/* PORT Slew Rate Register Set */
	#define PxSMTC		-1	/* PORT Slew Rate Register Clear */
#endif


#define PSHADOW_OFF(regsoffset)		(7*(regsoffset))
#define PZGIDLD         0xF0   /* GPIOZ Group ID to load */

#define PIN_NAMES_LEN		10
#define MAX_GPIOS_ON_CHIP	32
#define MAX_FUNCTIONS_ON_GPIO	4
#define PIN_ARGS_FROM_INDEX	0
#define PIN_ARGS_TO_INDEX	1
#define PIN_ARGS_CFG_INDEX	2

static inline u32 pin_bitmap(u32 from, u32 to)
{
	if (to == 31)
		return ~((1<<(from))-1);
	return ((~((1<<(from))-1))&((1 << ((to)+1))-1));
}

static inline unsigned bit_count(unsigned v)
{
	unsigned int c;
	for (c = 0; v; c++)
		v &= v - 1;
	return c;
}

typedef enum ingenic_gpio_pm_arr {
	PM_SLEEP_PULL = 0,	/*sleep state*/		/*input pull*/
	PM_SLEEP_NOPULL,				/*input no pull*/
	PM_SLEEP_PULLUP,				/*input pullup*/
	PM_SLEEP_PULLDOWN,				/*input pulldown*/
	PM_SLEEP_HIZ,					/*input hiz*/
	PM_SLEEP_HIGH,					/*output high*/
	PM_SLEEP_LOW,					/*output low*/
	PM_RESUME_INT,		/*keep register for resume*/
	PM_RESUME_MSK,
	PM_RESUME_PAT0,
	PM_RESUME_PAT1,
	PM_RESUME_PULL,
	PM_RESUME_PULLUP,
	PM_RESUME_PULLDOWN,
	PM_STATE_CNT,
} ingenic_gpio_pm_arr_t;

/*ingenic gpio chip*/
struct ingenic_gpio_chip {
	struct gpio_chip		gc;
	char				name[4];			/*name format "GPX"(X = 'A' + idx)*/
	u8				idx;				/*gpio index of this chip*/
	const struct device_node	*of_node;			/*gpio chip device node*/
	int				irq;				/*gpio chip irq*/
	struct irq_domain		*irq_domain;
	struct pinctrl_gpio_range	grange;
	struct ingenic_pinctrl		*pctl;
	spinlock_t			lock;				/*gpio func write lock*/
	u32				*mcu_gpio_reg;			/*used for gpio irq to mcu
									  mcu's pending register*/
	u32				used_pins_bitmap;			/*bitmap of pins for been had requested*/
	u32				resume_pending;			/*bitmap of pins wakeup pending when sleep*/
	u32				sleep_level;			/*bitmap of pins level when sleep*/
	u32				filter_bitmap;			/*bitmap of pins filter support*/
	u32				pull_bitmap;			/*bitmap of pins pill support*/
	u32				wakeup_bitmap;			/*bitmap of pins wakeup pins*/
	u32				pm_irq_bitmap;			/*bitmap of pins used for irq sleep&resume*/
	u32				pm_bitmap[PM_STATE_CNT];	/*bitmap of pins used for sleep&resume*/
};

#define gc_to_ingenic_gc(gpiochip) container_of(gpiochip, struct ingenic_gpio_chip, gc)

/*function information*/
struct ingenic_pinctrl_func {
	const char              *name;
	struct device_node	*of_node;	/* function device node */
	const char              **groups;	/* all sub groups name*/
	u8                      num_groups;	/* num groups of this function */
};

/*group information*/
struct ingenic_pinctrl_group {
	const char              *name;
	struct device_node	*of_node;	/* group device node */
	struct gpio_chip	*gc;		/* corresponding gpio chip*/
	unsigned		*pins;		/* Contained pins software gpio num*/
	u8                      num_pins;	/* num gpios of the set group*/
	u32			pinmux_bitmap;	/* pins bitmap of this group*/
	enum gpio_function	pinmux_func;	/* pins function select of this group*/
};


struct soc_special_regs {
	unsigned int reg; 		/* register offset */
	unsigned int reg_set;		/* register set offset */
	unsigned int reg_clear;		/* register clear offset */
};

/*soc private data*/
struct ingenic_priv {
	bool				have_shadow;		/*support shadow register or not*/
	bool				have_filter;		/*support input pin filter of nor*/
	bool				pullen_set;		/*set bit to enable pull function.*/
	bool				voltage_switchable;	/*ports support 1.8v and 3.3v switch.*/
	bool				dual_edge_interrupt ;	/*support Dual-Edge Interrupt.*/
	bool				pull_tristate;		/*support pullup/pulldown/hiz settable.*/
	bool				have_ds;		/*support drive strength settings.*/
	bool				have_slew;		/*support drive strength settings.*/
	bool				have_schmit;		/*support drive strength settings.*/
	int (*set_filter)(struct ingenic_gpio_chip *jzgc,	/*set input pin filter function*/
			unsigned pin,
			u16 filter);
	ssize_t (*dump_filter)(struct ingenic_pinctrl *pctl,
			char *buf,
			ssize_t size);
};

struct ingenic_pinctrl {
	void __iomem			*io_base;
	struct device_node		*of_node;		/*pinctrl device_node*/
	struct device			*dev;

	spinlock_t			shadow_lock;		/*shadow register access lock*/
	struct ingenic_gpio_chip	*gpio_chips;		/*gpio chips of this pinctrl*/
	unsigned			num_chips;		/*num gpio chips*/
	unsigned			total_pins;		/*total pins of this pinctrl*/
	u32				*bitmap_priv;		/*******private*******/

	struct pinctrl_desc		pctl_desc;
	struct pinctrl_dev		*pctl_dev;

	struct ingenic_pinctrl_group	*groups;
	unsigned			num_groups;

	struct ingenic_pinctrl_func	*functions;
	unsigned			num_funs;
	u32				group_regs_offset;		/* gpio group regs offset get from dts  */

	const struct ingenic_priv		*priv;			/*soc data*/
};

static inline struct ingenic_pinctrl_group *find_group_by_of_node(
		struct ingenic_pinctrl *pctl, struct device_node *np)
{
	int i;
	for (i = 0; i < pctl->num_groups; i++) {
		if (pctl->groups[i].of_node == np)
			return &pctl->groups[i];
	}
	return NULL;
}

static inline struct ingenic_pinctrl_func *find_func_by_of_node(
		struct ingenic_pinctrl *pctl, struct device_node *np)
{
	int i;
	for (i = 0; i < pctl->num_funs; i++) {
		if (pctl->functions[i].of_node == np)
			return &pctl->functions[i];
	}
	return NULL;
}

static inline u32 ingenic_gpio_readl(struct ingenic_gpio_chip *chip, int offset)
{
	struct ingenic_pinctrl *pctl = chip->pctl;
	return readl(pctl->io_base + (chip->idx * pctl->group_regs_offset) + offset);
}

static inline void ingenic_gpio_writel(struct ingenic_gpio_chip *chip, int offset, int value)
{
	struct ingenic_pinctrl *pctl = chip->pctl;
	writel(value , pctl->io_base + (chip->idx * pctl->group_regs_offset) + offset);
}

static inline void ingenic_gpio_shadow_fill(struct ingenic_gpio_chip *chip, int offset, int value)
{
	struct ingenic_pinctrl *pctl = chip->pctl;
	writel(value, pctl->io_base + PSHADOW_OFF(pctl->group_regs_offset)+ offset);
}

static inline void ingenic_gpio_shadow_writel(struct ingenic_gpio_chip *chip)
{
	struct ingenic_pinctrl *pctl = chip->pctl;
	writel(chip->idx, pctl->io_base + PSHADOW_OFF(pctl->group_regs_offset) + PZGIDLD);
}
#endif /*__PINCTRL_INGENIC_H__*/
