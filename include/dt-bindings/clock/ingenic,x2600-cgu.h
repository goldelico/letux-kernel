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
#define X2600_CLK_MAC			15
#define X2600_CLK_I2S			13
#define X2600_CLK_PCM			14
#define X2600_CLK_LCDPIXCLK		15
#define X2600_CLK_MSC0			16
#define X2600_CLK_MSC1			17
#define X2600_CLK_SFC			18
#define X2600_CLK_SSI			19
#if 0
#define X2600_CLK_PWM
#endif
#define X2600_CLK_TPC			20
#define X2600_CLK_CIMMCLK		21
#define X2600_CLK_G2D			22
#define X2600_CLK_CAN0			23
#define X2600_CLK_CAN1			24
#define X2600_CLK_SADC			25
#define X2600_CLK_OTGPHY		26

#define X2600_CLK_GATE_NEMC		27
#define X2600_CLK_GATE_OTG		28
#define X2600_CLK_GATE_USB		29
#define X2600_CLK_GATE_I2C0		30
#define X2600_CLK_GATE_I2C1		31
#define X2600_CLK_GATE_I2C2		32
#define X2600_CLK_GATE_I2C3		33
#define X2600_CLK_GATE_UART0		34
#define X2600_CLK_GATE_UART1		35
#define X2600_CLK_GATE_UART2		36
#define X2600_CLK_GATE_UART3		37
#define X2600_CLK_GATE_UART4		38
#define X2600_CLK_GATE_UART5		39
#define X2600_CLK_GATE_UART6		40
#define X2600_CLK_GATE_UART7		41
#define X2600_CLK_GATE_SSI0		42
#define X2600_CLK_GATE_SSI1		43
#define X2600_CLK_GATE_SSI_SLV		44
#define X2600_CLK_GATE_MIPI_DSI		45
#define X2600_CLK_GATE_AIC		46
#define X2600_CLK_GATE_DMIC		47
#define X2600_CLK_GATE_I2ST		48
#define X2600_CLK_GATE_DTRNG		49
#define X2600_CLK_GATE_OST		50
#define X2600_CLK_GATE_INTC		51
#define X2600_CLK_GATE_NFI		52
#define X2600_CLK_GATE_PDMA		53	// DMAC0
#define X2600_CLK_GATE_DMAC1		54
#define X2600_CLK_GATE_AES		55
#define X2600_CLK_GATE_HASH		56
#if 1
#define X2600_CLK_GATE_PWM		57
#endif
#define X2600_CLK_GATE_TCU0		58
#define X2600_CLK_GATE_TCU1		59
#define X2600_CLK_GATE_TCSM		60
#define X2600_CLK_GATE_ROTATE		61
#define X2600_CLK_GATE_FELIX		62
#define X2600_CLK_GATE_JPEGD		63
#define X2600_CLK_GATE_JPEGE		64
#define X2600_CLK_GATE_BMON		65
#define X2600_CLK_GATE_PCM0		66
#define X2600_CLK_GATE_PCM1		67

#define X2600_CLK_GATE_ARB		68
#define X2600_CLK_GATE_APB		69
#define X2600_CLK_GATE_AHB2		70
#define X2600_CLK_GATE_APB0		71

#endif /* __DT_BINDINGS_CLOCK_X2600_CGU_H__ */
