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
#define X2600_CLK_APLL			1
#define X2600_CLK_EPLL			2
#define X2600_CLK_MPLL			3
#define X2600_CLK_SCLKA			4
#define X2600_CLK_CPUMUX		5
#define X2600_CLK_CPU			6
#define X2600_CLK_L2CACHE		7
#define X2600_CLK_AHB0			8
#define X2600_CLK_AHB2PMUX		9
#define X2600_CLK_AHB2			10
#define X2600_CLK_PCLK			11
#define X2600_CLK_DDR			12
#define X2600_CLK_I2S0			13
#define X2600_CLK_LCDPIXCLK		14
#define X2600_CLK_MAC			15
#define X2600_CLK_MSC0			16
#define X2600_CLK_MSC1			17
#define X2600_CLK_SSI0			18
#define X2600_CLK_CIMMCLK		19
#define X2600_CLK_OTGPHY		20
#define X2600_CLK_NEMC			21
#define X2600_CLK_GATE_OTG		22
#define X2600_CLK_GATE_USB		23
#define X2600_CLK_SMB0			24
#define X2600_CLK_SMB1			25
#define X2600_CLK_AIC			26
#define X2600_CLK_SADC			27
#define X2600_CLK_UART0			28
#define X2600_CLK_UART1			29
#define X2600_CLK_UART2			30
#define X2600_CLK_UART3			31
#define X2600_CLK_UART4			32
#define X2600_CLK_UART5			33
#define X2600_CLK_UART6			34
#define X2600_CLK_UART7			35
#define X2600_CLK_PDMA			36
#define X2600_CLK_AES			37
#define X2600_CLK_GATE_TCU0		38
#define X2600_CLK_GATE_TCU1		39
#define X2600_CLK_GATE_CIM		40
#define X2600_CLK_GATE_MIPI_CSI		41

// FIXME - must be inserted or follow the last
#define X2600_CLK_I2S0_RX		98
#define X2600_CLK_I2S0_TX		99
#define X2600_CLK_GATE_SSI0		100
#define X2600_CLK_GATE_SSI1		101
#define X2600_CLK_DIV_SSI		102
#define X2600_CLK_DIV_CIM		107
#define X2600_CLK_GATE_CIM_MCLK		108

#endif /* __DT_BINDINGS_CLOCK_X2600_CGU_H__ */
