/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *  JZ4740 platform timer support for the JZ4730
 *  Copyright (C) 2017 Paul Boddie <paul@boddie.org.uk>
 *  JZ4730 customisations
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General	 Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __ASM_MACH_JZ4730_TIMER
#define __ASM_MACH_JZ4730_TIMER

#define JZ_REG_TIMER_ENABLE		0x00

#define JZ_REG_TIMER_RDR(x) (((x) * 0x20) + 0x10)
#define JZ_REG_TIMER_CNT(x) (((x) * 0x20) + 0x14)
#define JZ_REG_TIMER_CTRL(x) (((x) * 0x20) + 0x18)

#define JZ_TIMER_CTRL_IF		0x40
#define JZ_TIMER_CTRL_IE		0x20
#define JZ_TIMER_CTRL_SRC_MASK		0x7
#define JZ_TIMER_CTRL_SRC_PCLK_DIV_4	0
#define JZ_TIMER_CTRL_SRC_PCLK_DIV_16	1
#define JZ_TIMER_CTRL_SRC_PCLK_DIV_64	2
#define JZ_TIMER_CTRL_SRC_PCLK_DIV_256	3
#define JZ_TIMER_CTRL_SRC_RTC		4
#define JZ_TIMER_CTRL_SRC_EXT		5

/* Compensate for the lack of a EXTAL/16 clock source by multiplying the full
   level by 16 from 0xffff. */

#define JZ_TIMER_CLK_SRC                JZ_TIMER_CTRL_SRC_EXT
#define JZ_TIMER_CLK_DIVSHIFT		0
#define JZ_TIMER_CLK_FULL_LEVEL		0xfffff

extern void __iomem *jz4740_timer_base;
void __init jz4740_timer_init(void);

void jz4740_timer_enable_watchdog(void);
void jz4740_timer_disable_watchdog(void);

static inline bool jz4740_timer_is_enabled(unsigned int timer)
{
	return !!(readb(jz4740_timer_base + JZ_REG_TIMER_ENABLE) & BIT(timer));
}

static inline void jz4740_timer_enable(unsigned int timer)
{
	/* NOTE: Not atomic! */
	unsigned int val = readl(jz4740_timer_base + JZ_REG_TIMER_ENABLE);
	writeb(val | BIT(timer), jz4740_timer_base + JZ_REG_TIMER_ENABLE);
}

static inline void jz4740_timer_disable(unsigned int timer)
{
	/* NOTE: Not atomic! */
	unsigned int val = readl(jz4740_timer_base + JZ_REG_TIMER_ENABLE);
	writeb(val & ~BIT(timer), jz4740_timer_base + JZ_REG_TIMER_ENABLE);
}

static inline void jz4740_timer_set_period(unsigned int timer, uint32_t period)
{
	writel(period, jz4740_timer_base + JZ_REG_TIMER_RDR(timer));
}

static inline void jz4740_timer_set_count(unsigned int timer, uint32_t count)
{
	writel(count, jz4740_timer_base + JZ_REG_TIMER_CNT(timer));
}

static inline uint32_t jz4740_timer_get_count(unsigned int timer)
{
	return readl(jz4740_timer_base + JZ_REG_TIMER_CNT(timer));
}

static inline void jz4740_timer_ack_full(unsigned int timer)
{
	/* NOTE: Not atomic! */
	unsigned int val = readl(jz4740_timer_base + JZ_REG_TIMER_CTRL(timer));
	writew(val & ~JZ_TIMER_CTRL_IF, jz4740_timer_base + JZ_REG_TIMER_CTRL(timer));
}

static inline void jz4740_timer_irq_full_enable(unsigned int timer)
{
	/* NOTE: Not atomic! */
	unsigned int val = readl(jz4740_timer_base + JZ_REG_TIMER_CTRL(timer));
	writew(val | JZ_TIMER_CTRL_IE, jz4740_timer_base + JZ_REG_TIMER_CTRL(timer));
}

static inline void jz4740_timer_irq_full_disable(unsigned int timer)
{
	/* NOTE: Not atomic! */
	unsigned int val = readl(jz4740_timer_base + JZ_REG_TIMER_CTRL(timer));
	writew(val & ~JZ_TIMER_CTRL_IE, jz4740_timer_base + JZ_REG_TIMER_CTRL(timer));
}

static inline void jz4740_timer_set_ctrl(unsigned int timer, uint16_t ctrl)
{
	writew(ctrl, jz4740_timer_base + JZ_REG_TIMER_CTRL(timer));
}

static inline uint16_t jz4740_timer_get_ctrl(unsigned int timer)
{
	return readw(jz4740_timer_base + JZ_REG_TIMER_CTRL(timer));
}

#endif
