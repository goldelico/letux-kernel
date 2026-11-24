/* SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause */
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
#define X2600_CLK_MAC			13
#define X2600_CLK_I2S			14
#define X2600_CLK_PCM			15
#define X2600_CLK_LCDPIXCLK		16
#define X2600_CLK_MSC0			17
#define X2600_CLK_MSC1			18
#define X2600_CLK_SFC			19
#define X2600_CLK_SSI			20
#if 0
#define X2600_CLK_PWM	// gibt es den hier als MUX/DIV clock?
#endif
#define X2600_CLK_TPC			21
#define X2600_CLK_CIMMCLK		22
#define X2600_CLK_G2D			23
#define X2600_CLK_CAN0			24
#define X2600_CLK_CAN1			25
#define X2600_CLK_SADC			26

#define X2600_CLK_OTGPHY		27
#define X2600_CLK_USBPHY		28

#define X2600_CLK_GATE_NEMC		29
#define X2600_CLK_GATE_OTG		30
#define X2600_CLK_GATE_USB		31
#define X2600_CLK_GATE_I2C0		32
#define X2600_CLK_GATE_I2C1		33
#define X2600_CLK_GATE_I2C2		34
#define X2600_CLK_GATE_I2C3		35
#define X2600_CLK_GATE_UART0		36
#define X2600_CLK_GATE_UART1		37
#define X2600_CLK_GATE_UART2		38
#define X2600_CLK_GATE_UART3		39
#define X2600_CLK_GATE_UART4		40
#define X2600_CLK_GATE_UART5		41
#define X2600_CLK_GATE_UART6		42
#define X2600_CLK_GATE_UART7		43
#define X2600_CLK_GATE_SSI0		44
#define X2600_CLK_GATE_SSI1		45
#define X2600_CLK_GATE_SSI_SLV		46
#define X2600_CLK_GATE_MIPI_DSI		47
#define X2600_CLK_GATE_AIC		48
#define X2600_CLK_GATE_DMIC		49
#define X2600_CLK_GATE_I2ST		50
#define X2600_CLK_GATE_DTRNG		51
#define X2600_CLK_GATE_OST		52
#define X2600_CLK_GATE_INTC		53
#define X2600_CLK_GATE_NFI		54
#define X2600_CLK_GATE_PDMA		55	// DMAC0
#define X2600_CLK_GATE_DMAC1		56
#define X2600_CLK_GATE_AES		57
#define X2600_CLK_GATE_HASH		58
#define X2600_CLK_GATE_PWM		59
#define X2600_CLK_GATE_TCU0		60
#define X2600_CLK_GATE_TCU1		61
#define X2600_CLK_GATE_TCSM		62
#define X2600_CLK_GATE_ROTATE		63
#define X2600_CLK_GATE_FELIX		64
#define X2600_CLK_GATE_JPEGD		65
#define X2600_CLK_GATE_JPEGE		66
#define X2600_CLK_GATE_BMON		67
#define X2600_CLK_GATE_PCM0		68
#define X2600_CLK_GATE_PCM1		69

#define X2600_CLK_GATE_ARB		70
#define X2600_CLK_GATE_APB		71
#define X2600_CLK_GATE_AHB2		72
#define X2600_CLK_GATE_APB0		73

#endif /* __DT_BINDINGS_CLOCK_X2600_CGU_H__ */
