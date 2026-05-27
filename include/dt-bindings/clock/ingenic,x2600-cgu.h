/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This header provides clock numbers for the ingenic,x2600-cgu DT binding.
 *
 * They are roughly ordered as:
 *   - external clocks
 *   - PLLs
 *   - muxes/dividers in the order they appear in the x2600 programmers manual
 *   - gates in order of their bit in the CLKGR* registers
 */

#ifndef __DT_BINDINGS_CLOCK_X2600_CGU_H__
#define __DT_BINDINGS_CLOCK_X2600_CGU_H__

#define X2600_CLK_EXCLK			0
#define X2600_CLK_RTCLK			1
// FIXME: there is no CLK12M?
#define X2600_CLK_12M			2
#define X2600_CLK_APLL			3
#define X2600_CLK_EPLL			4
#define X2600_CLK_MPLL			5
#define X2600_CLK_SCLKA			6
#define X2600_CLK_CPUMUX		7
#define X2600_CLK_CPU			8
#define X2600_CLK_L2CACHE		9
#define X2600_CLK_AHB0			10
#define X2600_CLK_AHB2PMUX		11
#define X2600_CLK_AHB2			12
#define X2600_CLK_PCLK			13
#define X2600_CLK_DDR			14
#define X2600_CLK_I2S0			15
// FIXME: there is no X2600_CLK_I2S1?
#define X2600_CLK_I2S1			16
#define X2600_CLK_LCDPIXCLK		17
#define X2600_CLK_MAC			18
#define X2600_CLK_MSC0			19
#define X2600_CLK_MSC1			20
#define X2600_CLK_SSI0			21
#define X2600_CLK_CIMMCLK		22
#define X2600_CLK_GATE_CIM	(0 + 3 + 3 + 19 + 21 + 6)
#define X2600_CLK_GATE_MIPI_CSI	(0 + 3 + 3 + 19 + 21 + 27)
#define X2600_CLK_EXCLK_DIV512		23
#define X2600_CLK_RTC			24
#define X2600_CLK_OTGPHY		25
#define X2600_CLK_NEMC			26
#define X2600_CLK_OTG0			27
#define X2600_CLK_SMB0			28
#define X2600_CLK_SMB1			29
#define X2600_CLK_AIC			30
#define X2600_CLK_SADC			31
#define X2600_CLK_UART0			32
#define X2600_CLK_UART1			33
#define X2600_CLK_UART2			34
#define X2600_CLK_UART3			35
#define X2600_CLK_PDMA			36
#define X2600_CLK_AES			37
#define X2600_CLK_GATE_TCU0		38
#define X2600_CLK_GATE_TCU1		39

// FIXME:
#define X2600_CLK_I2S0_RX		38
#define X2600_CLK_I2S0_TX		39
#define X2600_CLK_GATE_SSI0		100
#define X2600_CLK_GATE_SSI1		101
#define X2600_CLK_DIV_SSI		102
#define X2600_CLK_GATE_OTG		105
#define X2600_CLK_GATE_USB		106
#define X2600_CLK_DIV_CIM		107
#define X2600_CLK_GATE_CIM_MCLK		108
#define X2600_CLK_UART4			109
#define X2600_CLK_UART5			110
#define X2600_CLK_UART6			111
#define X2600_CLK_UART7			112

#endif /* __DT_BINDINGS_CLOCK_X2600_CGU_H__ */
