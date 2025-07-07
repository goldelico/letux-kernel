/* SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause */
/*
 * This header provides clock numbers for the ingenic,x2000-cgu DT binding.
 *
 * They are roughly ordered as:
 *   - external clocks
 *   - PLLs
 *   - muxes/dividers in the order they appear in the x2000 programmers manual
 *   - gates in order of their bit in the CLKGR* registers
 */

#ifndef __DT_BINDINGS_CLOCK_X2000_CGU_H__
#define __DT_BINDINGS_CLOCK_X2000_CGU_H__

#define X2000_CLK_EXCLK			0
#define X2000_CLK_RTCLK			1
#define X2000_CLK_12M			2
#define X2000_CLK_APLL			3
#define X2000_CLK_MPLL			4
#define X2000_CLK_EPLL			5
#define X2000_CLK_OTGPHY		6
#define X2000_CLK_SCLKA			7
#define X2000_CLK_I2S0			8
#define X2000_CLK_I2S1			9
#define X2000_CLK_I2S2			10
#define X2000_CLK_I2S3			11
#define X2000_CLK_CPUMUX		12
#define X2000_CLK_CPU			13
#define X2000_CLK_L2CACHE		14
#define X2000_CLK_AHB0			15
#define X2000_CLK_AHB2PMUX		16
#define X2000_CLK_AHB2			17
#define X2000_CLK_PCLK			18
#define X2000_CLK_DDR			19
#define X2000_CLK_ISP			20
#define X2000_CLK_MACPTP		21
#define X2000_CLK_MACPHY		22
#define X2000_CLK_MAC0TX		23
#define X2000_CLK_MAC1TX		24
#define X2000_CLK_RSA			25
#define X2000_CLK_SSIPLL		26
#define X2000_CLK_LCD			27
#define X2000_CLK_MSC_EXCLK		28
#define X2000_CLK_MSC0			29
#define X2000_CLK_MSC1			30
#define X2000_CLK_MSC2			31
#define X2000_CLK_PWM			32
#define X2000_CLK_SFC			33
#define X2000_CLK_CIM			34
#define X2000_CLK_DMIC_EXCLK		35
#define X2000_CLK_DMIC			36
#define X2000_CLK_EXCLK_DIV512		37
#define X2000_CLK_RTC			38
#define X2000_CLK_EMC			39
#define X2000_CLK_EFUSE			40
#define X2000_CLK_OTG			41
#define X2000_CLK_SCC			42
#define X2000_CLK_I2C0			43
#define X2000_CLK_I2C1			44
#define X2000_CLK_I2C2			45
#define X2000_CLK_I2C3			46
#define X2000_CLK_SADC			47
#define X2000_CLK_UART0			48
#define X2000_CLK_UART1			49
#define X2000_CLK_UART2			50
#define X2000_CLK_DTRNG			51
#define X2000_CLK_TCU			52
#define X2000_CLK_SSI0			53
#define X2000_CLK_OST			54
#define X2000_CLK_PDMA			55
#define X2000_CLK_SSI1			56
#define X2000_CLK_I2C4			57
#define X2000_CLK_I2C5			58
#define X2000_CLK_ISP0			59
#define X2000_CLK_ISP1			60
#define X2000_CLK_HASH			61
#define X2000_CLK_UART3			62
#define X2000_CLK_UART4			63
#define X2000_CLK_UART5			64
#define X2000_CLK_UART6			65
#define X2000_CLK_UART7			66
#define X2000_CLK_UART8			67
#define X2000_CLK_UART9			68
#define X2000_CLK_MAC0			69
#define X2000_CLK_MAC1			70
#define X2000_CLK_INTC			71
#define X2000_CLK_CSI			72
#define X2000_CLK_DSI			73

#endif /* __DT_BINDINGS_CLOCK_X2000_CGU_H__ */
