/*
 * omap_hwmod_34xx.h - hardware modules present on the OMAP34xx chips
 *
 * Copyright (C) 2009 Nokia Corporation
 * Paul Walmsley
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __ARCH_ARM_PLAT_OMAP_INCLUDE_MACH_OMAP_HWMOD34XX_H
#define __ARCH_ARM_PLAT_OMAP_INCLUDE_MACH_OMAP_HWMOD34XX_H

#ifdef CONFIG_ARCH_OMAP34XX

#include <plat/omap_hwmod.h>
#include <mach/irqs.h>
#include <plat/cpu.h>
#include <plat/dma.h>
#include <plat/serial.h>

#include "prm-regbits-34xx.h"

static struct omap_hwmod omap34xx_mpu_hwmod;
static struct omap_hwmod omap34xx_l3_hwmod;
static struct omap_hwmod omap34xx_l4_core_hwmod;
static struct omap_hwmod omap34xx_l4_per_hwmod;

/* L3 -> L4_CORE interface */
static struct omap_hwmod_ocp_if omap34xx_l3__l4_core = {
	.master	= &omap34xx_l3_hwmod,
	.slave	= &omap34xx_l4_core_hwmod,
	.user	= OCP_USER_MPU | OCP_USER_SDMA,
};

/* L3 -> L4_PER interface */
static struct omap_hwmod_ocp_if omap34xx_l3__l4_per = {
	.master = &omap34xx_l3_hwmod,
	.slave	= &omap34xx_l4_per_hwmod,
	.user	= OCP_USER_MPU | OCP_USER_SDMA,
};

/* MPU -> L3 interface */
static struct omap_hwmod_ocp_if omap34xx_mpu__l3 = {
	.master = &omap34xx_mpu_hwmod,
	.slave	= &omap34xx_l3_hwmod,
	.user	= OCP_USER_MPU,
};

/* Slave interfaces on the L3 interconnect */
/* KJH: OCP ifs which have L3 interconnect as the slave  */
static struct omap_hwmod_ocp_if *omap34xx_l3_slaves[] = {
	&omap34xx_mpu__l3,
};

/* Master interfaces on the L3 interconnect */
static struct omap_hwmod_ocp_if *omap34xx_l3_masters[] = {
	&omap34xx_l3__l4_core,
	&omap34xx_l3__l4_per,
};

/* L3 */
static struct omap_hwmod omap34xx_l3_hwmod = {
	.name		= "l3_hwmod",
	.masters	= omap34xx_l3_masters,
	.masters_cnt	= ARRAY_SIZE(omap34xx_l3_masters),
	.slaves		= omap34xx_l3_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_l3_slaves),
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430)
};

static struct omap_hwmod omap34xx_l4_wkup_hwmod;
static struct omap_hwmod omap34xx_uart1;
static struct omap_hwmod omap34xx_uart2;
static struct omap_hwmod omap34xx_uart3;

/* L4_CORE -> L4_WKUP interface */
static struct omap_hwmod_ocp_if omap34xx_l4_core__l4_wkup = {
	.master	= &omap34xx_l4_core_hwmod,
	.slave	= &omap34xx_l4_wkup_hwmod,
	.user	= OCP_USER_MPU | OCP_USER_SDMA,
};

/* L4 CORE -> UART1 interface */
static struct omap_hwmod_addr_space omap34xx_uart1_addr_space[] = {
	{
		.pa_start	= OMAP_UART1_BASE,
		.pa_end		= OMAP_UART1_BASE + SZ_8K - 1,
		.flags		= ADDR_MAP_ON_INIT | ADDR_TYPE_RT,
	},
};

static struct omap_hwmod_ocp_if omap3_l4_core__uart1 = {
	.master		= &omap34xx_l4_core_hwmod,
	.slave		= &omap34xx_uart1,
	.clkdev_dev_id	= NULL,
	.clkdev_con_id  = "uart1_ick",
	.addr		= omap34xx_uart1_addr_space,
	.addr_cnt	= ARRAY_SIZE(omap34xx_uart1_addr_space),
	.user		= OCP_USER_MPU | OCP_USER_SDMA,
};

/* L4 CORE -> UART2 interface */
static struct omap_hwmod_addr_space omap34xx_uart2_addr_space[] = {
	{
		.pa_start	= OMAP_UART2_BASE,
		.pa_end		= OMAP_UART2_BASE + SZ_1K - 1,
		.flags		= ADDR_MAP_ON_INIT |ADDR_TYPE_RT,
	},
};

static struct omap_hwmod_ocp_if omap3_l4_core__uart2 = {
	.master		= &omap34xx_l4_core_hwmod,
	.slave		= &omap34xx_uart2,
	.clkdev_dev_id	= NULL,
	.clkdev_con_id  = "uart2_ick",
	.addr		= omap34xx_uart2_addr_space,
	.addr_cnt	= ARRAY_SIZE(omap34xx_uart2_addr_space),
	.user		= OCP_USER_MPU | OCP_USER_SDMA,
};

/* L4 PER -> UART3 interface */
static struct omap_hwmod_addr_space omap34xx_uart3_addr_space[] = {
	{
		.pa_start	= OMAP_UART3_BASE,
		.pa_end		= OMAP_UART3_BASE + SZ_1K - 1,
		.flags		= ADDR_MAP_ON_INIT | ADDR_TYPE_RT,
	},
};

static struct omap_hwmod_ocp_if omap3_l4_per__uart3 = {
	.master		= &omap34xx_l4_per_hwmod,
	.slave		= &omap34xx_uart3,
	.clkdev_dev_id	= NULL,
	.clkdev_con_id  = "uart3_ick",
	.addr		= omap34xx_uart3_addr_space,
	.addr_cnt	= ARRAY_SIZE(omap34xx_uart3_addr_space),
	.user		= OCP_USER_MPU | OCP_USER_SDMA,
};

/* Slave interfaces on the L4_CORE interconnect */
/* KJH: OCP ifs where L4 CORE is the slave */
static struct omap_hwmod_ocp_if *omap34xx_l4_core_slaves[] = {
	&omap34xx_l3__l4_core,
};

/* Master interfaces on the L4_CORE interconnect */
/* KJH: OCP ifs where L4 CORE is the master */
static struct omap_hwmod_ocp_if *omap34xx_l4_core_masters[] = {
	&omap34xx_l4_core__l4_wkup,
	&omap3_l4_core__uart1,
	&omap3_l4_core__uart2,
};

/* L4 CORE */
static struct omap_hwmod omap34xx_l4_core_hwmod = {
	.name		= "l4_core_hwmod",
	.masters	= omap34xx_l4_core_masters,
	.masters_cnt	= ARRAY_SIZE(omap34xx_l4_core_masters),
	.slaves		= omap34xx_l4_core_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_l4_core_slaves),
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430)
};

/* Slave interfaces on the L4_PER interconnect */
static struct omap_hwmod_ocp_if *omap34xx_l4_per_slaves[] = {
	&omap34xx_l3__l4_per,
};

/* Master interfaces on the L4_PER interconnect */
static struct omap_hwmod_ocp_if *omap34xx_l4_per_masters[] = {
	&omap3_l4_per__uart3,
};

/* L4 PER */
static struct omap_hwmod omap34xx_l4_per_hwmod = {
	.name		= "l4_per_hwmod",
	.masters	= omap34xx_l4_per_masters,
	.masters_cnt	= ARRAY_SIZE(omap34xx_l4_per_masters),
	.slaves		= omap34xx_l4_per_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_l4_per_slaves),
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430)
};

/* Slave interfaces on the L4_WKUP interconnect */
static struct omap_hwmod_ocp_if *omap34xx_l4_wkup_slaves[] = {
	&omap34xx_l4_core__l4_wkup,
};

/* Master interfaces on the L4_WKUP interconnect */
static struct omap_hwmod_ocp_if *omap34xx_l4_wkup_masters[] = {
};

/* L4 WKUP */
static struct omap_hwmod omap34xx_l4_wkup_hwmod = {
	.name		= "l4_wkup_hwmod",
	.masters	= omap34xx_l4_wkup_masters,
	.masters_cnt	= ARRAY_SIZE(omap34xx_l4_wkup_masters),
	.slaves		= omap34xx_l4_wkup_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_l4_wkup_slaves),
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430)
};

/* Master interfaces on the MPU device */
/* KJH: OCP ifs where MPU is the master */
static struct omap_hwmod_ocp_if *omap34xx_mpu_masters[] = {
	&omap34xx_mpu__l3,
};

/* MPU */
static struct omap_hwmod omap34xx_mpu_hwmod = {
	.name		= "mpu_hwmod",
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= "arm_fck",
	.masters	= omap34xx_mpu_masters,
	.masters_cnt	= ARRAY_SIZE(omap34xx_mpu_masters),
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

/* UART common */

static struct omap_hwmod_sysconfig uart_if_ctrl = {
	.rev_offs	= 0x50,
	.sysc_offs	= 0x54,
	.syss_offs	= 0x58,
	.sysc_flags	= (SYSC_HAS_SIDLEMODE |
			   SYSC_HAS_ENAWAKEUP | SYSC_HAS_SOFTRESET |
			   SYSC_HAS_AUTOIDLE),
	.idlemodes	= (SIDLE_FORCE | SIDLE_NO | SIDLE_SMART),
	.sysc_fields    = &omap_hwmod_sysc_type1,
};

/* UART1 */

static struct omap_hwmod_irq_info uart1_mpu_irqs[] = {
	{ .irq = INT_24XX_UART1_IRQ, },
};

static struct omap_hwmod_dma_info uart1_sdma_chs[] = {
	{ .name = "tx",	.dma_ch = OMAP24XX_DMA_UART1_TX, },
	{ .name = "rx",	.dma_ch = OMAP24XX_DMA_UART1_RX, },
};

static struct omap_hwmod_ocp_if *omap34xx_uart1_slaves[] = {
	&omap3_l4_core__uart1,
};

static struct omap_hwmod omap34xx_uart1 = {
	.name		= "uart1",
	.mpu_irqs	= uart1_mpu_irqs,
	.mpu_irqs_cnt	= ARRAY_SIZE(uart1_mpu_irqs),
	.sdma_chs	= uart1_sdma_chs,
	.sdma_chs_cnt	= ARRAY_SIZE(uart1_sdma_chs),
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= "uart1_fck",
	.prcm		= {
		.omap2 = {
			.prcm_reg_id = 1,
			.module_bit = OMAP3430_EN_UART1_SHIFT,
		},
	},
	.slaves		= omap34xx_uart1_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_uart1_slaves),
	.sysconfig	= &uart_if_ctrl,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

/* UART2 */

static struct omap_hwmod_irq_info uart2_mpu_irqs[] = {
	{ .irq = INT_24XX_UART2_IRQ, },
};

static struct omap_hwmod_dma_info uart2_sdma_chs[] = {
	{ .name = "tx",	.dma_ch = OMAP24XX_DMA_UART2_TX, },
	{ .name = "rx",	.dma_ch = OMAP24XX_DMA_UART2_RX, },
};

static struct omap_hwmod_ocp_if *omap34xx_uart2_slaves[] = {
	&omap3_l4_core__uart2,
};

static struct omap_hwmod omap34xx_uart2 = {
	.name		= "uart2",
	.mpu_irqs	= uart2_mpu_irqs,
	.mpu_irqs_cnt	= ARRAY_SIZE(uart2_mpu_irqs),
	.sdma_chs	= uart2_sdma_chs,
	.sdma_chs_cnt	= ARRAY_SIZE(uart2_sdma_chs),
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= "uart2_fck",
	.prcm		= {
		.omap2 = {
			.prcm_reg_id = 1,
			.module_bit = OMAP3430_EN_UART2_SHIFT,
		},
	},
	.slaves		= omap34xx_uart2_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_uart2_slaves),
	.sysconfig	= &uart_if_ctrl,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

/* UART3 */

static struct omap_hwmod_irq_info uart3_mpu_irqs[] = {
	{ .irq = INT_24XX_UART3_IRQ, },
};

static struct omap_hwmod_dma_info uart3_sdma_chs[] = {
	{ .name = "tx",	.dma_ch = OMAP24XX_DMA_UART3_TX, },
	{ .name = "rx",	.dma_ch = OMAP24XX_DMA_UART3_RX, },
};

static struct omap_hwmod_ocp_if *omap34xx_uart3_slaves[] = {
	&omap3_l4_per__uart3,
};

static struct omap_hwmod omap34xx_uart3 = {
	.name		= "uart3",
	.mpu_irqs	= uart3_mpu_irqs,
	.mpu_irqs_cnt	= ARRAY_SIZE(uart3_mpu_irqs),
	.sdma_chs	= uart3_sdma_chs,
	.sdma_chs_cnt	= ARRAY_SIZE(uart3_sdma_chs),
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= "uart3_fck",
	.prcm		= {
		.omap2 = {
			.prcm_reg_id = 1,
			.module_bit = OMAP3430_EN_UART3_SHIFT,
		},
	},
	.slaves		= omap34xx_uart3_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_uart3_slaves),
	.sysconfig	= &uart_if_ctrl,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

static __initdata struct omap_hwmod *omap34xx_hwmods[] = {
	&omap34xx_l3_hwmod,
	&omap34xx_l4_core_hwmod,
	&omap34xx_l4_per_hwmod,
	&omap34xx_l4_wkup_hwmod,
	&omap34xx_mpu_hwmod,
	&omap34xx_uart1,
	&omap34xx_uart2,
	&omap34xx_uart3,
	NULL,
};

#else
# define omap34xx_hwmods		0
#endif

#endif


