/*
 * Ingenic JZ4730 SoC CGU driver
 *
 * Copyright (c) 2015 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * Copyright (c) 2017 Paul Boddie <paul@boddie.org.uk>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <dt-bindings/clock/jz4730-cgu.h>
#include <asm/mach-jz4740/clock.h>
#include "cgu.h"

/* CGU register offsets */
#define CGU_REG_CFCR		0x00 /* CPCCR in jz4740 */
#define CGU_REG_LPCR		0x04 /* LCR in jz4740 */
#define CGU_REG_PLCR1		0x10 /* CPPCR in jz4740 */
#define CGU_REG_MSCR		0x20 /* CLKGR in jz4740 */
#define CGU_REG_CFCR2		0x60 /* LPCDR in jz4740 */

/* bits within a PLL control register */
#define PLLCTL_M_SHIFT		23
#define PLLCTL_M_MASK		(0x1ff << PLLCTL_M_SHIFT)
#define PLLCTL_N_SHIFT		18
#define PLLCTL_N_MASK		(0x1f << PLLCTL_N_SHIFT)
#define PLLCTL_OD_SHIFT		16
#define PLLCTL_OD_MASK		(0x3 << PLLCTL_OD_SHIFT)
#define PLLCTL_STABLE		(1 << 10)
#define PLLCTL_BYPASS		(1 << 9)
#define PLLCTL_ENABLE		(1 << 8)

/* bits within the LCR register */
#define LPCR_SLEEP		(1 << 0)

/* bits within the MSCR register */
#define MSCR_UDC		(1 << 24)

static struct ingenic_cgu *cgu;

static const s8 pll_od_encoding[4] = {
	0x0, 0x1, -1, 0x3,
};

static const s8 div_step_encoding[] = {
	0, 1, 2, 3, -1, 4, -1, 5,	/* 1...8 */
	-1, -1, -1, 6, -1, -1, -1, 7,	/* 9...16 */
	-1, -1, -1, -1, -1, -1, -1, 8,	/* 17...24 */
	-1, -1, -1, -1, -1, -1, -1, 9,	/* 25...32 */
};

static const struct ingenic_cgu_clk_info jz4730_cgu_clocks[] = {

	/* External clocks */

	[JZ4730_CLK_EXT] = { "ext", CGU_CLK_EXT },
	[JZ4730_CLK_RTC] = { "rtc", CGU_CLK_EXT },
	[JZ4730_CLK_MSC16M] = { "msc16m", CGU_CLK_EXT },
	[JZ4730_CLK_MSC24M] = { "msc24m", CGU_CLK_EXT },
	[JZ4730_CLK_USB48M] = { "usb48m", CGU_CLK_EXT },

	[JZ4730_CLK_PLL] = {
		"pll", CGU_CLK_PLL,
		.parents = { JZ4730_CLK_EXT, -1, -1, -1 },
		.pll = {
			.reg = CGU_REG_PLCR1,
			.m_shift = 23,
			.m_bits = 9,
			.m_offset = 2,
			.n_shift = 18,
			.n_bits = 5,
			.n_offset = 2,
			.od_shift = 16,
			.od_bits = 2,
			.od_max = 4,
			.od_encoding = pll_od_encoding,
			.stable_bit = 10,
			.bypass_bit = 9,
			.enable_bit = 8,
		},
	},

	/* Muxes & dividers */

	[JZ4730_CLK_PLL_HALF] = {
		"pll2", CGU_CLK_FIXDIV,
		.parents = { JZ4730_CLK_PLL, -1, -1, -1 },
		.div = { 2 },
	},

	[JZ4730_CLK_CCLK_PLL] = {
		"cclkdiv", CGU_CLK_STEPDIV,
		.parents = { JZ4730_CLK_PLL, -1, -1, -1 },
		.stepdiv = { CGU_REG_CFCR, 0, 1, 4, 20, -1, -1, div_step_encoding },
	},

	[JZ4730_CLK_CCLK] = {
		"cclk", CGU_CLK_MUX,
		.parents = { JZ4730_CLK_EXT, JZ4730_CLK_CCLK_PLL, -1, -1 },
		.mux = { CGU_REG_PLCR1, 8, 1 },
	},

	[JZ4730_CLK_HCLK_PLL] = {
		"hclkdiv", CGU_CLK_STEPDIV,
		.parents = { JZ4730_CLK_PLL, -1, -1, -1 },
		.stepdiv = { CGU_REG_CFCR, 4, 1, 4, 20, -1, -1, div_step_encoding },
	},

	[JZ4730_CLK_HCLK] = {
		"hclk", CGU_CLK_MUX,
		.parents = { JZ4730_CLK_EXT, JZ4730_CLK_HCLK_PLL, -1, -1 },
		.mux = { CGU_REG_PLCR1, 8, 1 },
	},

	[JZ4730_CLK_PCLK_PLL] = {
		"pclkdiv", CGU_CLK_STEPDIV,
		.parents = { JZ4730_CLK_PLL, -1, -1, -1 },
		.stepdiv = { CGU_REG_CFCR, 8, 1, 4, 20, -1, -1, div_step_encoding },
	},

	[JZ4730_CLK_PCLK] = {
		"pclk", CGU_CLK_MUX,
		.parents = { JZ4730_CLK_EXT, JZ4730_CLK_PCLK_PLL, -1, -1 },
		.mux = { CGU_REG_PLCR1, 8, 1 },
	},

	[JZ4730_CLK_MCLK_PLL] = {
		"mclkdiv", CGU_CLK_STEPDIV,
		.parents = { JZ4730_CLK_PLL, -1, -1, -1 },
		.stepdiv = { CGU_REG_CFCR, 16, 1, 4, 20, -1, -1, div_step_encoding },
	},

	[JZ4730_CLK_MCLK] = {
		"mclk", CGU_CLK_MUX,
		.parents = { JZ4730_CLK_EXT, JZ4730_CLK_MCLK_PLL, -1, -1 },
		.mux = { CGU_REG_PLCR1, 8, 1 },
	},

	[JZ4730_CLK_LCD_PLL] = {
		"lcddiv", CGU_CLK_STEPDIV | CGU_CLK_GATE,
		.parents = { JZ4730_CLK_PLL, -1, -1, -1 },
		.stepdiv = { CGU_REG_CFCR, 12, 1, 4, 20, -1, -1, div_step_encoding },
		.gate = { CGU_REG_MSCR, 7 },
	},

	[JZ4730_CLK_LCD] = {
		"lcd", CGU_CLK_MUX,
		.parents = { JZ4730_CLK_EXT, JZ4730_CLK_LCD_PLL, -1, -1 },
		.mux = { CGU_REG_PLCR1, 8, 1 },
	},

	[JZ4730_CLK_LCD_PCLK] = {
		"lcd_pclk", CGU_CLK_DIV,
		.parents = { JZ4730_CLK_PLL, -1, -1, -1 },
		.div = { CGU_REG_CFCR2, 0, 1, 11, -1, -1, -1 },
	},

	[JZ4730_CLK_I2S] = {
		"i2s", CGU_CLK_MUX,
		.parents = { JZ4730_CLK_PLL, JZ4730_CLK_PLL_HALF, -1, -1 },
		.mux = { CGU_REG_CFCR, 29, 1 },
	},

	[JZ4730_CLK_UHC_IN] = {
		"uhc", CGU_CLK_DIV,
		.parents = { JZ4730_CLK_PLL, -1, -1, -1 },
		.div = { CGU_REG_CFCR, 25, 1, 3, -1, -1, -1 },
	},

	[JZ4730_CLK_UHC] = {
		"uhc", CGU_CLK_MUX,
		.parents = { JZ4730_CLK_UHC_IN, JZ4730_CLK_USB48M, -1, -1 },
		.mux = { CGU_REG_CFCR, 28, 1 },
	},

	/* Gate-only clocks */

	[JZ4730_CLK_DMA] = {
		"dma", CGU_CLK_GATE,
		.parents = { JZ4730_CLK_PCLK, -1, -1, -1 },
		.gate = { CGU_REG_CFCR, 5 },
	},

	[JZ4730_CLK_MMC] = {
		"mmc", CGU_CLK_GATE,
		.parents = { JZ4730_CLK_MSC16M, JZ4730_CLK_MSC24M, -1, -1 },
		.gate = { CGU_REG_CFCR, 24 },
	},

	[JZ4730_CLK_UART0] = {
		"uart0", CGU_CLK_GATE,
		.parents = { JZ4730_CLK_EXT, -1, -1, -1 },
		.gate = { CGU_REG_MSCR, 0 },
	},

	[JZ4730_CLK_UART1] = {
		"uart1", CGU_CLK_GATE,
		.parents = { JZ4730_CLK_EXT, -1, -1, -1 },
		.gate = { CGU_REG_MSCR, 1 },
	},

	[JZ4730_CLK_UART2] = {
		"uart2", CGU_CLK_GATE,
		.parents = { JZ4730_CLK_EXT, -1, -1, -1 },
		.gate = { CGU_REG_MSCR, 2 },
	},

	[JZ4730_CLK_UART3] = {
		"uart3", CGU_CLK_GATE,
		.parents = { JZ4730_CLK_EXT, -1, -1, -1 },
		.gate = { CGU_REG_MSCR, 20 },
	},
};

static void __init jz4730_cgu_init(struct device_node *np)
{
	int retval;

	cgu = ingenic_cgu_new(jz4730_cgu_clocks,
			      ARRAY_SIZE(jz4730_cgu_clocks), np);
	if (!cgu) {
		pr_err("%s: failed to initialise CGU\n", __func__);
		return;
	}

	retval = ingenic_cgu_register_clocks(cgu);
	if (retval)
		pr_err("%s: failed to register CGU Clocks\n", __func__);
}
CLK_OF_DECLARE(jz4730_cgu, "ingenic,jz4730-cgu", jz4730_cgu_init);

void jz4740_clock_set_wait_mode(enum jz4740_wait_mode mode)
{
	uint32_t lcr = readl(cgu->base + CGU_REG_LPCR);

	switch (mode) {
	case JZ4740_WAIT_MODE_IDLE:
		lcr &= ~LPCR_SLEEP;
		break;

	case JZ4740_WAIT_MODE_SLEEP:
		lcr |= LPCR_SLEEP;
		break;
	}

	writel(lcr, cgu->base + CGU_REG_LPCR);
}

void jz4740_clock_udc_disable_auto_suspend(void)
{
	uint32_t clkgr = readl(cgu->base + CGU_REG_MSCR);

	clkgr &= ~MSCR_UDC;
	writel(clkgr, cgu->base + CGU_REG_MSCR);
}
EXPORT_SYMBOL_GPL(jz4740_clock_udc_disable_auto_suspend);

void jz4740_clock_udc_enable_auto_suspend(void)
{
	uint32_t clkgr = readl(cgu->base + CGU_REG_MSCR);

	clkgr |= MSCR_UDC;
	writel(clkgr, cgu->base + CGU_REG_MSCR);
}
EXPORT_SYMBOL_GPL(jz4740_clock_udc_enable_auto_suspend);

#define JZ_CLOCK_GATE_UART0	BIT(0)
#define JZ_CLOCK_GATE_OST	BIT(3)
#define JZ_CLOCK_GATE_DMAC	BIT(5)

void jz4740_clock_suspend(void)
{
	uint32_t clkgr, cppcr;

	clkgr = readl(cgu->base + CGU_REG_MSCR);
	clkgr |= JZ_CLOCK_GATE_OST | JZ_CLOCK_GATE_DMAC | JZ_CLOCK_GATE_UART0;
	writel(clkgr, cgu->base + CGU_REG_MSCR);

	cppcr = readl(cgu->base + CGU_REG_PLCR1);
	cppcr &= ~BIT(jz4730_cgu_clocks[JZ4730_CLK_PLL].pll.enable_bit);
	writel(cppcr, cgu->base + CGU_REG_PLCR1);
}

void jz4740_clock_resume(void)
{
	uint32_t clkgr, cppcr, stable;

	cppcr = readl(cgu->base + CGU_REG_PLCR1);
	cppcr |= BIT(jz4730_cgu_clocks[JZ4730_CLK_PLL].pll.enable_bit);
	writel(cppcr, cgu->base + CGU_REG_PLCR1);

	stable = BIT(jz4730_cgu_clocks[JZ4730_CLK_PLL].pll.stable_bit);
	do {
		cppcr = readl(cgu->base + CGU_REG_PLCR1);
	} while (!(cppcr & stable));

	clkgr = readl(cgu->base + CGU_REG_MSCR);
	clkgr &= ~JZ_CLOCK_GATE_OST;
	clkgr &= ~JZ_CLOCK_GATE_DMAC;
	clkgr &= ~JZ_CLOCK_GATE_UART0;
	writel(clkgr, cgu->base + CGU_REG_MSCR);
}
