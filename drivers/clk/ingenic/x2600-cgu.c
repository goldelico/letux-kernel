// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Ingenic X2600 SoC CGU driver
 *
 * Copyright (c) 2013-2015 Imagination Technologies
 * Author: Paul Burton <paul.burton@mips.com>
 * Copyright (c) 2020 周琰杰 (Zhou Yanjie) <zhouyanjie@wanyeetech.com>
 * Copyright (c) 2023, 2024 Paul Boddie <paul@boddie.org.uk>
 * Copyright (c) 2026 H. Nikolaus Schaller <hns@goldelico.com>
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/rational.h>
#include <linux/of.h>

#include <dt-bindings/clock/ingenic,x2600-cgu.h>

#include "cgu.h"
#include "pm.h"

/* CGU register offsets */
#define CGU_REG_CLOCKCONTROL	0x00
#define CGU_REG_LCR		0x04
#define CGU_REG_CPPCR		0x0c	// not in x1600
#define CGU_REG_APLL		0x10
#define CGU_REG_MPLL		0x14
#define CGU_REG_EPLL		0x18
#define CGU_REG_CLKGR0		0x20
#define CGU_REG_OPCR		0x24
#define CGU_REG_CLKGR1		0x28
#define CGU_REG_DDRCDR		0x2c
#define CGU_REG_G2DCDR		0x30	// not in x1600
#define CGU_REG_CPSPR		0x34
#define CGU_REG_CPSPPR		0x38
#define CGU_REG_USBPCR		0x3c
#define CGU_REG_USBRDT		0x40
#define CGU_REG_USBVBFIL	0x44
#define CGU_REG_USBPCR1		0x48
#define CGU_REG_USB1PCR		0x4c	// not in x1600
#define CGU_REG_USB1RDT		0x50	// not in x1600
#define CGU_REG_MACCDR		0x54
#define CGU_REG_USB1VBFIL	0x58	// not in x1600
#define CGU_REG_SSICDR		0x5c
#define CGU_REG_I2S0CDR		0x60
#define CGU_REG_LPCDR		0x64
#define CGU_REG_MSC0CDR		0x68
#define CGU_REG_PWMCDR		0x6c
#define CGU_REG_I2S0CDR1	0x70
#define CGU_REG_SFCCDR		0x74
#define CGU_REG_CIMCDR		0x78
#define CGU_REG_TPCCDR		0x7c	// not in x1600
#define CGU_REG_APLLFRAC	0x84
#define CGU_REG_MPLLFRAC	0x88
#define CGU_REG_EPLLFRAC	0x8c
#define CGU_REG_CAN0CDR		0xa0
#define CGU_REG_MSC1CDR		0xa4
#define CGU_REG_CAN1CDR		0xa8
#define CGU_REG_SADCCDR		0xac	// not in x1600
#define CGU_REG_CMP_INTR	0xb0
#define CGU_REG_CMP_INTRE	0xb4
#define CGU_REG_CMP_SFTINT	0xbc
#define CGU_REG_PCMCDR		0xc0	// not in x1600
#define CGU_REG_SRBC		0xc4	// not in x1600
#define CGU_REG_SLBC		0xc8	// not in x1600
#define CGU_REG_SLPC		0xcc	// not in x1600
#define CGU_REG_DRCG		0xd0
#define CGU_REG_CLOCKSTATUS	0xd4
#define CGU_REG_DMICR		0xd8	// not in x1600
#define CGU_REG_PCMCDR1		0xdc	// not in x1600
#define CGU_REG_EXCLK_DS	0xe0	// not in x1600
#define CGU_REG_MPHY0C		0xe4
#define CGU_REG_USB1PCR1	0xe8	// not in x1600
#define CGU_REG_MESTSEL		0xec	// not in x1600

/* x1600 only
#define CGU_REG_I2S1CDR		0x7c
#define CGU_REG_I2S1CDR1	0x80
#define CGU_REG_CDBUSCDR	0xac
*/

/* bits within the OPCR register */
#define OPCR_SPENDN0		BIT(7)
#define OPCR_GATEUSBPHYCLK	BIT(23)

static struct ingenic_cgu *cgu;

static int x2600_otg_phy_enable(struct clk_hw *hw)
{
	void __iomem *reg_opcr		= cgu->base + CGU_REG_OPCR;

	writel((readl(reg_opcr) & ~OPCR_GATEUSBPHYCLK) | OPCR_SPENDN0, reg_opcr);

	return 0;
}

static void x2600_otg_phy_disable(struct clk_hw *hw)
{
	void __iomem *reg_opcr		= cgu->base + CGU_REG_OPCR;

	writel((readl(reg_opcr) & ~OPCR_SPENDN0) | OPCR_GATEUSBPHYCLK, reg_opcr);
}

static int x2600_otg_phy_is_enabled(struct clk_hw *hw)
{
	void __iomem *reg_opcr		= cgu->base + CGU_REG_OPCR;

	return (readl(reg_opcr) & (OPCR_SPENDN0 | OPCR_GATEUSBPHYCLK)) == OPCR_SPENDN0;
}

static u8 x2600_otg_phy_get_parent(struct clk_hw *hw)
{
	(void) hw;
	return 0;
}

static const struct clk_ops x2600_otg_phy_ops = {
	.enable		= x2600_otg_phy_enable,
	.disable	= x2600_otg_phy_disable,
	.is_enabled	= x2600_otg_phy_is_enabled,
	.get_parent	= x2600_otg_phy_get_parent,
};

// FIXME: might need something for x2600_usb_phy

static void
x2600_pll_calc_m_n_od(const struct ingenic_cgu_pll_info *pll_info,
		      unsigned long rate, unsigned long parent_rate,
		      unsigned int *pm, unsigned int *pn, unsigned int *pod,
		      unsigned int *pod1)
{
	const unsigned int od_max = pll_info->od_max - 1;
	const unsigned int od1_max = pll_info->od1_max - 1;
	const unsigned int m_max = GENMASK(pll_info->m_bits - 1, 0);
	const unsigned int n_max = GENMASK(pll_info->n_bits - 1, 0);
	unsigned long m, n;
	unsigned int od = 1, od1 = 1;
	const unsigned long fvco_min = 600000000, fvco_max = 2400000000;
	unsigned long fvco;

	/*
	 * Combined output divider range:
	 *
	 * ceil(fvco_min / rate) - fvco_max / rate
	 */
	unsigned od_combined_min = (fvco_min + rate) / rate;
	unsigned od_combined_max = fvco_max / rate;
	unsigned od_combined;

	for (od_combined = od_combined_min; od_combined <= od_combined_max; od_combined++)
	{
		od = int_sqrt(od_combined);
		od1 = od_combined / od;

		/* od == floor(sqrt(od_combined)); od1 >= od */
		if ((od * od1 == od_combined) && (od <= od_max) && (od1 <= od1_max))
		{
			fvco = rate * od_combined;
			rational_best_approximation(fvco, parent_rate, m_max, n_max, &m, &n);
			break;
		}
	}

	*pm = m;
	*pn = n;
	*pod = od;
	*pod1 = od1;
}

static const struct ingenic_cgu_clk_info x2600_cgu_clocks[] = {

	/* External clocks */

	[X2600_CLK_EXCLK] = { "ext", CGU_CLK_EXT },
	[X2600_CLK_RTCLK] = { "rtc", CGU_CLK_EXT },

#if 0	// or can we use this instead of X2600_CLK_RTCLK and all rtc references in device tree?
	[X1600_CLK_EXCLK_DIV512] = {
		"exclk_div512", CGU_CLK_FIXDIV,
		.parents = { X1600_CLK_EXCLK },
		.fixdiv = { 512 },
	},
#endif

// FIXME: there is no CLK12M? So let's pretend it is the same as EXCLK until we renumber the ingenic,x2600-cgu.h numbers
	[X2600_CLK_12M] = { "ext" /* "clk12m" */, CGU_CLK_EXT },

	/* PLLs */

#define DEF_PLL(name) { \
	.reg = CGU_REG_ ## name, \
	.rate_multiplier = 1, \
	.m_shift = 20, \
	.m_bits = 12, \
	.m_offset = 0, \
	.n_shift = 14, \
	.n_bits = 6, \
	.n_offset = 0, \
	.od_shift = 8, \
	.od_bits = 3, \
	.od_max = 8, \
	.od1_shift = 11, \
	.od1_bits = 3, \
	.od1_max = 8, \
	.od_encoding = 0, \
	.stable_bit = 2, \
	.enable_bit = 0, \
	.bypass_bit = -1, \
	.calc_m_n_od = x2600_pll_calc_m_n_od, \
}

	[X2600_CLK_APLL] = {
		"apll", CGU_CLK_PLL,
		.parents = { X2600_CLK_EXCLK, -1, -1, -1 },
		.pll = DEF_PLL(APLL),
	},

	[X2600_CLK_MPLL] = {
		"mpll", CGU_CLK_PLL,
		.parents = { X2600_CLK_EXCLK, -1, -1, -1 },
		.pll = DEF_PLL(MPLL),
	},

	[X2600_CLK_EPLL] = {
		"epll", CGU_CLK_PLL,
		.parents = { X2600_CLK_EXCLK, -1, -1, -1 },
		.pll = DEF_PLL(EPLL),
	},

#undef DEF_PLL

	/* Muxes & dividers */

	[X2600_CLK_SCLKA] = {
		"sclk_a", CGU_CLK_MUX,
		.parents = { -1, X2600_CLK_EXCLK, X2600_CLK_APLL, -1 },
		.mux = { CGU_REG_CLOCKCONTROL, 30, 2 },
	},

	[X2600_CLK_CPUMUX] = {
		"cpumux", CGU_CLK_MUX,
		.parents = { -1, X2600_CLK_SCLKA, X2600_CLK_MPLL, -1 },
		.mux = { CGU_REG_CLOCKCONTROL, 28, 2 },
	},

	[X2600_CLK_CPU] = {
		"cpu", CGU_CLK_DIV,
		/*
		 * Disabling the CPU clock or any parent clocks will hang the
		 * system; mark it critical.
		 */
		.flags = CLK_IS_CRITICAL,
		.parents = { X2600_CLK_CPUMUX, -1, -1, -1 },
		.div = { CGU_REG_CLOCKCONTROL, 0, 1, 4, 22, -1, -1 },
	},

	[X2600_CLK_L2CACHE] = {
		"l2cache", CGU_CLK_DIV,
		/*
		 * The L2 cache clock is critical if caches are enabled and
		 * disabling it or any parent clocks will hang the system.
		 */
		.flags = CLK_IS_CRITICAL,
		.parents = { X2600_CLK_CPUMUX, -1, -1, -1 },
		.div = { CGU_REG_CLOCKCONTROL, 4, 1, 4, -1, -1, -1 },
	},

	[X2600_CLK_AHB0] = {
		"ahb0", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { -1, X2600_CLK_SCLKA, X2600_CLK_MPLL, -1 },
		.mux = { CGU_REG_CLOCKCONTROL, 26, 2 },
		.div = { CGU_REG_CLOCKCONTROL, 8, 1, 4, 21, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 29 },
	},

	[X2600_CLK_AHB2PMUX] = {
		"ahb2_apb_mux", CGU_CLK_MUX,
		.parents = { -1, X2600_CLK_SCLKA, X2600_CLK_MPLL, -1 },
		.mux = { CGU_REG_CLOCKCONTROL, 24, 2 },
	},

	[X2600_CLK_AHB2] = {
		"ahb2", CGU_CLK_DIV,
		.parents = { X2600_CLK_AHB2PMUX, -1, -1, -1 },
		.div = { CGU_REG_CLOCKCONTROL, 12, 1, 4, 20, -1, -1 },
	},

	[X2600_CLK_PCLK] = {
		"pclk", CGU_CLK_DIV,
		.parents = { X2600_CLK_AHB2PMUX, -1, -1, -1 },
		.div = { CGU_REG_CLOCKCONTROL, 16, 1, 4, 20, -1, -1 },
	},

	[X2600_CLK_DDR] = {
		"ddr", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		/*
		 * Disabling DDR clock or its parents will render DRAM
		 * inaccessible; mark it critical.
		 */
		.flags = CLK_IS_CRITICAL,
		.parents = { -1, X2600_CLK_SCLKA, X2600_CLK_MPLL, -1 },
		.mux = { CGU_REG_DDRCDR, 30, 2 },
		.div = { CGU_REG_DDRCDR, 0, 1, 4, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR0, 31 },
	},

	[X2600_CLK_I2S0] = {
		"i2s0", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X2600_CLK_SCLKA, X2600_CLK_EPLL, -1, -1 },
		.mux = { CGU_REG_I2S0CDR, 30, 1 },
		.div = { CGU_REG_I2S0CDR, 20, 1, 9, -1, -1, -1 },
		.mdiv = { CGU_REG_I2S0CDR, 0, 1, 20 },
		.nddiv = { CGU_REG_I2S0CDR1, 31, 30 },
		.gate = { CGU_REG_I2S0CDR, 29, true },
	},

#if 0	// FIXME: there is no X2600_CLK_I2S1?
	[X2600_CLK_I2S1] = {
		"i2s1", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X2600_CLK_SCLKA, X2600_CLK_EPLL, -1, -1 },
		.mux = { CGU_REG_I2S1CDR, 30, 1 },
		.div = { CGU_REG_I2S1CDR, 20, 1, 9, -1, -1, -1 },
		.mdiv = { CGU_REG_I2S1CDR, 0, 1, 20 },
		.nddiv = { CGU_REG_I2S1CDR1, 31, 30 },
		.gate = { CGU_REG_I2S1CDR, 29, true },
	},
#endif

	[X2600_CLK_LCDPIXCLK] = {
		"lcd0pixclk", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X2600_CLK_SCLKA, X2600_CLK_MPLL,
			     X2600_CLK_EPLL, -1 },
		.mux = { CGU_REG_LPCDR, 30, 2 },
		.div = { CGU_REG_LPCDR, 0, 1, 8, 28, 27, 26 },
		.gate = { CGU_REG_CLKGR0, 23 },
	},

	[X2600_CLK_MAC] = {
		"mac", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X2600_CLK_SCLKA, X2600_CLK_MPLL,
			     X2600_CLK_EPLL, -1 },
		.mux = { CGU_REG_MACCDR, 30, 2 },
		.div = { CGU_REG_MACCDR, 0, 1, 8, 28, 27, 26 },
		.gate = { CGU_REG_CLKGR1, 23 },
	},

	[X2600_CLK_MSC0] = {
		"msc0", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X2600_CLK_SCLKA, X2600_CLK_MPLL,
			     X2600_CLK_EPLL, -1 },
		.mux = { CGU_REG_MSC0CDR, 30, 2 },
		.div = { CGU_REG_MSC0CDR, 0, 2, 8, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR0, 4 },
	},

	[X2600_CLK_MSC1] = {
		"msc1", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X2600_CLK_SCLKA, X2600_CLK_MPLL,
			     X2600_CLK_EPLL, -1 },
		.mux = { CGU_REG_MSC1CDR, 30, 2 },
		.div = { CGU_REG_MSC1CDR, 0, 2, 8, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR0, 5 },
	},

	[X2600_CLK_SSI0] = {
		"ssi", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X2600_CLK_SCLKA, X2600_CLK_MPLL,
			     X2600_CLK_EPLL, -1 },
		.mux = { CGU_REG_SSICDR, 30, 2 },
		.div = { CGU_REG_SSICDR, 0, 1, 8, 29, 28, 27 },
		.gate = { CGU_REG_CLKGR0, 19 },
	},

	[X2600_CLK_CIMMCLK] = {
		"cim_mclk", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { X2600_CLK_SCLKA, X2600_CLK_MPLL,
			     X2600_CLK_EPLL, -1 },
		.mux = { CGU_REG_CIMCDR, 30, 2 },
		.div = { CGU_REG_CIMCDR, 0, 1, 8, 30, 29, 28 },
		.gate = { CGU_REG_CLKGR0, 22 },
	},

	/* Custom (SoC-specific) OTG PHY */

	[X2600_CLK_OTGPHY] = {
		"otg_phy", CGU_CLK_CUSTOM,
		.parents = { X2600_CLK_12M },
		.custom = { &x2600_otg_phy_ops },
	},

	/* Gate-only clocks */
	[X2600_CLK_NEMC] = {
		"nemc", CGU_CLK_GATE,
		.parents = { X2600_CLK_AHB2, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 0 },	// = EFUSE
	},

	[X2600_CLK_GATE_OTG] = {
		"gate_otg", CGU_CLK_GATE,
		.parents = { X2600_CLK_AHB2, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 4 },
	},

	[X2600_CLK_GATE_USB] = {
		"gate_usb", CGU_CLK_GATE,
		.parents = { X2600_CLK_AHB2, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 5 },
	},

	[X2600_CLK_SMB0] = {
		"smb0", CGU_CLK_GATE,
		.parents = { X2600_CLK_PCLK, -1, -1, -1 },
//FIXME
		.gate = { CGU_REG_CLKGR0, 7 },
	},

	[X2600_CLK_SMB1] = {
		"smb1", CGU_CLK_GATE,
		.parents = { X2600_CLK_PCLK, -1, -1, -1 },
//FIXME
		.gate = { CGU_REG_CLKGR0, 7 },
	},

	[X2600_CLK_AIC] = {
		"aic", CGU_CLK_GATE,
		.parents = { X2600_CLK_AHB2, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 24 },
	},

	[X2600_CLK_SADC] = {
		"sadc", CGU_CLK_GATE,
		.parents = { X2600_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR1, 10 },
	},

	[X2600_CLK_UART0] = {
		"uart0", CGU_CLK_GATE,
		.parents = { X2600_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 12 },
	},

	[X2600_CLK_UART1] = {
		"uart1", CGU_CLK_GATE,
		.parents = { X2600_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 13 },
	},

	[X2600_CLK_UART2] = {
		"uart2", CGU_CLK_GATE,
		.parents = { X2600_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 14 },
	},

	[X2600_CLK_UART3] = {
		"uart3", CGU_CLK_GATE,
		.parents = { X2600_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 15 },
	},

	[X2600_CLK_UART4] = {
		"uart4", CGU_CLK_GATE,
		.parents = { X2600_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 16 },
	},

	[X2600_CLK_UART5] = {
		"uart5", CGU_CLK_GATE,
		.parents = { X2600_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 17 },
	},

	[X2600_CLK_UART6] = {
		"uart6", CGU_CLK_GATE,
		.parents = { X2600_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 18 },
	},

	[X2600_CLK_UART7] = {
		"uart7", CGU_CLK_GATE,
		.parents = { X2600_CLK_EXCLK, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR0, 19 },
	},

	[X2600_CLK_PDMA] = {
		"pdma", CGU_CLK_GATE,
		.parents = { X2600_CLK_AHB2, -1, -1, -1 },
// FIXME
		.gate = { CGU_REG_CLKGR1, 0 },
	},

	[X2600_CLK_AES] = {
		"aes", CGU_CLK_GATE,
		.parents = { X2600_CLK_AHB2, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR1, 2 },
	},

	[X2600_CLK_GATE_TCU0] = {
		"gate_tcu0", CGU_CLK_GATE,
		.parents = { X2600_CLK_AHB2, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR1, 8 },
	},

	[X2600_CLK_GATE_TCU1] = {
		"gate_tcu1", CGU_CLK_GATE,
		.parents = { X2600_CLK_AHB2, -1, -1, -1 },
		.gate = { CGU_REG_CLKGR1, 9 },
	},


#if 0	// missing definitions
	[X2600_CLK_GATE_SSI0] =
	[X2600_CLK_GATE_SSI1] =
	[X2600_CLK_DIV_SSI] =
	[X2600_CLK_GATE_TCU0] =
	[X2600_CLK_GATE_TCU1] =
	[X2600_CLK_GATE_OTG] =
	[X2600_CLK_GATE_USB] =
	[X2600_CLK_DIV_CIM] =
	[X2600_CLK_GATE_CIM_MCLK] =
	X2600_CLK_UART4..7
#endif
};

static void __init x2600_cgu_init(struct device_node *np)
{
	int retval;

	cgu = ingenic_cgu_new(x2600_cgu_clocks,
			      ARRAY_SIZE(x2600_cgu_clocks), np);
	if (!cgu) {
		pr_err("%s: failed to initialise CGU\n", __func__);
		return;
	}

	retval = ingenic_cgu_register_clocks(cgu);
	if (retval) {
		pr_err("%s: failed to register CGU Clocks\n", __func__);
		return;
	}

	ingenic_cgu_register_syscore(cgu);
}
CLK_OF_DECLARE_DRIVER(x2600_cgu, "ingenic,x2600-cgu", x2600_cgu_init);
