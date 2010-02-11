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
#include <plat/l4_3xxx.h>
#include <plat/i2c.h>
#include <plat/gpio.h>

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
static struct omap_hwmod omap34xx_i2c1;
static struct omap_hwmod omap34xx_i2c2;
static struct omap_hwmod omap34xx_i2c3;
static struct omap_hwmod omap34xx_gpio1;
static struct omap_hwmod omap34xx_gpio2;
static struct omap_hwmod omap34xx_gpio3;
static struct omap_hwmod omap34xx_gpio4;
static struct omap_hwmod omap34xx_gpio5;
static struct omap_hwmod omap34xx_gpio6;

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

#define OMAP3_I2C1_BASE			(L4_34XX_BASE + 0x70000)
#define OMAP3_I2C2_BASE			(L4_34XX_BASE + 0x72000)
#define OMAP3_I2C3_BASE			(L4_34XX_BASE + 0x60000)

/* I2C IP block address space length (in bytes) */
#define OMAP2_I2C_AS_LEN		128

/* L4 CORE -> I2C1 interface */
static struct omap_hwmod_addr_space omap34xx_i2c1_addr_space[] = {
	{
		.pa_start	= OMAP3_I2C1_BASE,
		.pa_end		= OMAP3_I2C1_BASE + OMAP2_I2C_AS_LEN - 1,
		.flags		= ADDR_TYPE_RT,
	},
};

static struct omap_hwmod_ocp_if omap34xx_l4_core__i2c1 = {
	.master		= &omap34xx_l4_core_hwmod,
	.slave		= &omap34xx_i2c1,
	.clkdev_dev_id	= "i2c_omap.1",
	.clkdev_con_id  = "ick",
	.addr		= omap34xx_i2c1_addr_space,
	.addr_cnt	= ARRAY_SIZE(omap34xx_i2c1_addr_space),
	.fw = {
		.omap2 = {
			.l4_fw_region  = OMAP3_L4_CORE_FW_I2C1_REGION,
			.l4_prot_group = 7,
		}
	},
	.user		= OCP_USER_MPU | OCP_USER_SDMA,
	.flags		= OMAP_FIREWALL_L4
};

/* L4 CORE -> I2C2 interface */
static struct omap_hwmod_addr_space omap34xx_i2c2_addr_space[] = {
	{
		.pa_start	= OMAP3_I2C2_BASE,
		.pa_end		= OMAP3_I2C2_BASE + OMAP2_I2C_AS_LEN - 1,
		.flags		= ADDR_TYPE_RT,
	},
};

static struct omap_hwmod_ocp_if omap34xx_l4_core__i2c2 = {
	.master		= &omap34xx_l4_core_hwmod,
	.slave		= &omap34xx_i2c2,
	.clkdev_dev_id	= "i2c_omap.2",
	.clkdev_con_id  = "ick",
	.addr		= omap34xx_i2c2_addr_space,
	.addr_cnt	= ARRAY_SIZE(omap34xx_i2c2_addr_space),
	.fw = {
		.omap2 = {
			.l4_fw_region  = OMAP3_L4_CORE_FW_I2C2_REGION,
			.l4_prot_group = 7,
		}
	},
	.user		= OCP_USER_MPU | OCP_USER_SDMA,
	.flags		= OMAP_FIREWALL_L4
};

/* L4 CORE -> I2C3 interface */
static struct omap_hwmod_addr_space omap34xx_i2c3_addr_space[] = {
	{
		.pa_start	= OMAP3_I2C3_BASE,
		.pa_end		= OMAP3_I2C3_BASE + OMAP2_I2C_AS_LEN - 1,
		.flags		= ADDR_TYPE_RT,
	},
};

static struct omap_hwmod_ocp_if omap34xx_l4_core__i2c3 = {
	.master		= &omap34xx_l4_core_hwmod,
	.slave		= &omap34xx_i2c3,
	.clkdev_dev_id	= "i2c_omap.3",
	.clkdev_con_id  = "ick",
	.addr		= omap34xx_i2c3_addr_space,
	.addr_cnt	= ARRAY_SIZE(omap34xx_i2c3_addr_space),
	.fw = {
		.omap2 = {
			.l4_fw_region  = OMAP3_L4_CORE_FW_I2C3_REGION,
			.l4_prot_group = 7,
		}
	},
	.user		= OCP_USER_MPU | OCP_USER_SDMA,
	.flags		= OMAP_FIREWALL_L4
};

/*
 * GPIO1 interface data
 */

static struct omap_hwmod_addr_space omap34xx_gpio1_addr_space[] = {
	{
		.pa_start	= OMAP34XX_GPIO1_BASE,
		.pa_end		= OMAP34XX_GPIO1_BASE + OMAP3_GPIO_AS_LEN - 1,
		.flags		= ADDR_TYPE_RT
	},
};

/* GPIO1 <- L4_WKUP interface */
static struct omap_hwmod_ocp_if omap34xx_l4_wkup__gpio1 = {
	.master		= &omap34xx_l4_wkup_hwmod,
	.slave		= &omap34xx_gpio1,
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= "gpio1_ick",
	.addr		= omap34xx_gpio1_addr_space,
	.addr_cnt	= ARRAY_SIZE(omap34xx_gpio1_addr_space),
	.user		= OCP_USER_MPU | OCP_USER_SDMA,
};

static struct omap_hwmod_ocp_if *omap34xx_gpio1_slaves[] = {
	&omap34xx_l4_wkup__gpio1,
};

/*
 * GPIO2 interface data
 */

static struct omap_hwmod_addr_space omap34xx_gpio2_addr_space[] = {
	{
		.pa_start	= OMAP34XX_GPIO2_BASE,
		.pa_end		= OMAP34XX_GPIO2_BASE + OMAP3_GPIO_AS_LEN - 1,
		.flags		= ADDR_TYPE_RT
	},
};

/* GPIO2 <- L4_PER interface */
static struct omap_hwmod_ocp_if omap34xx_l4_per__gpio2 = {
	.master		= &omap34xx_l4_per_hwmod,
	.slave		= &omap34xx_gpio2,
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= "gpio2_ick",
	.addr		= omap34xx_gpio2_addr_space,
	.addr_cnt	= ARRAY_SIZE(omap34xx_gpio2_addr_space),
	.user		= OCP_USER_MPU | OCP_USER_SDMA,
};

static struct omap_hwmod_ocp_if *omap34xx_gpio2_slaves[] = {
	&omap34xx_l4_per__gpio2,
};

/*
 * GPIO3 interface data
 */

static struct omap_hwmod_addr_space omap34xx_gpio3_addr_space[] = {
	{
		.pa_start	= OMAP34XX_GPIO3_BASE,
		.pa_end		= OMAP34XX_GPIO3_BASE + OMAP3_GPIO_AS_LEN - 1,
		.flags		= ADDR_TYPE_RT
	},
};

/* GPIO3 <- L4_PER interface */
static struct omap_hwmod_ocp_if omap34xx_l4_per__gpio3 = {
	.master		= &omap34xx_l4_per_hwmod,
	.slave		= &omap34xx_gpio3,
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= "gpio3_ick",
	.addr		= omap34xx_gpio3_addr_space,
	.addr_cnt	= ARRAY_SIZE(omap34xx_gpio3_addr_space),
	.user		= OCP_USER_MPU | OCP_USER_SDMA,
};
static struct omap_hwmod_ocp_if *omap34xx_gpio3_slaves[] = {
	&omap34xx_l4_per__gpio3,
};

/*
 * GPIO4 interface data
 */

static struct omap_hwmod_addr_space omap34xx_gpio4_addr_space[] = {
	{
		.pa_start	= OMAP34XX_GPIO4_BASE,
		.pa_end		= OMAP34XX_GPIO4_BASE + OMAP3_GPIO_AS_LEN - 1,
		.flags		= ADDR_TYPE_RT
	},
};

/* GPIO4 <- L4_PER interface */
static struct omap_hwmod_ocp_if omap34xx_l4_per__gpio4 = {
	.master		= &omap34xx_l4_per_hwmod,
	.slave		= &omap34xx_gpio4,
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= "gpio4_ick",
	.addr		= omap34xx_gpio4_addr_space,
	.addr_cnt	= ARRAY_SIZE(omap34xx_gpio4_addr_space),
	.user		= OCP_USER_MPU | OCP_USER_SDMA,
};

static struct omap_hwmod_ocp_if *omap34xx_gpio4_slaves[] = {
	&omap34xx_l4_per__gpio4,
};

/*
 * GPIO5 interface data
 */

static struct omap_hwmod_addr_space omap34xx_gpio5_addr_space[] = {
	{
		.pa_start	= OMAP34XX_GPIO5_BASE,
		.pa_end		= OMAP34XX_GPIO5_BASE + OMAP3_GPIO_AS_LEN - 1,
		.flags		= ADDR_TYPE_RT
	},
};

/* GPIO5 <- L4_PER interface */
static struct omap_hwmod_ocp_if omap34xx_l4_per__gpio5 = {
	.master		= &omap34xx_l4_per_hwmod,
	.slave		= &omap34xx_gpio5,
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= "gpio5_ick",
	.addr		= omap34xx_gpio5_addr_space,
	.addr_cnt	= ARRAY_SIZE(omap34xx_gpio5_addr_space),
	.user		= OCP_USER_MPU | OCP_USER_SDMA,
};

static struct omap_hwmod_ocp_if *omap34xx_gpio5_slaves[] = {
	&omap34xx_l4_per__gpio5,
};

/*
 * GPIO6 interface data
 */

static struct omap_hwmod_addr_space omap34xx_gpio6_addr_space[] = {
	{
		.pa_start	= OMAP34XX_GPIO6_BASE,
		.pa_end		= OMAP34XX_GPIO6_BASE + OMAP3_GPIO_AS_LEN - 1,
		.flags		= ADDR_TYPE_RT
	},
};

/* GPIO6 <- L4_PER interface */
static struct omap_hwmod_ocp_if omap34xx_l4_per__gpio6 = {
	.master		= &omap34xx_l4_per_hwmod,
	.slave		= &omap34xx_gpio6,
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= "gpio6_ick",
	.addr		= omap34xx_gpio6_addr_space,
	.addr_cnt	= ARRAY_SIZE(omap34xx_gpio6_addr_space),
	.user		= OCP_USER_MPU | OCP_USER_SDMA,
};

static struct omap_hwmod_ocp_if *omap34xx_gpio6_slaves[] = {
	&omap34xx_l4_per__gpio6,
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
	&omap34xx_l4_core__i2c1,
	&omap34xx_l4_core__i2c2,
	&omap34xx_l4_core__i2c3,
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
	&omap34xx_l4_per__gpio2,
	&omap34xx_l4_per__gpio3,
	&omap34xx_l4_per__gpio4,
	&omap34xx_l4_per__gpio5,
	&omap34xx_l4_per__gpio6,
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
	&omap34xx_l4_wkup__gpio1,
};

/* Master interfaces on the L4_WKUP interconnect */
static struct omap_hwmod_ocp_if *omap34xx_l4_wkup_masters[] = {
	&omap34xx_l4_wkup__gpio1,
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

/* I2C common */
static struct omap_hwmod_sysconfig i2c_if_ctrl = {
	.rev_offs	= 0x00,
	.sysc_offs	= 0x20,
	.syss_offs	= 0x10,
	.sysc_flags	= (SYSC_HAS_CLOCKACTIVITY | SYSC_HAS_SIDLEMODE |
			   SYSC_HAS_ENAWAKEUP | SYSC_HAS_SOFTRESET |
			   SYSC_HAS_AUTOIDLE),
	.idlemodes	= (SIDLE_FORCE | SIDLE_NO | SIDLE_SMART),
	.sysc_fields    = &omap_hwmod_sysc_type1,
};

/* I2C1 */

static struct omap_i2c_dev_attr i2c1_dev_attr = {
	.fifo_depth	= 8, /* bytes */
};

static struct omap_hwmod_irq_info i2c1_mpu_irqs[] = {
	{ .irq = INT_24XX_I2C1_IRQ, },
};

static struct omap_hwmod_dma_info i2c1_sdma_chs[] = {
	{ .name = "tx", .dma_ch = OMAP24XX_DMA_I2C1_TX },
	{ .name = "rx", .dma_ch = OMAP24XX_DMA_I2C1_RX },
};

static struct omap_hwmod_ocp_if *omap34xx_i2c1_slaves[] = {
	&omap34xx_l4_core__i2c1,
};

static struct omap_hwmod omap34xx_i2c1 = {
	.name		= "i2c1",
	.mpu_irqs	= i2c1_mpu_irqs,
	.mpu_irqs_cnt	= ARRAY_SIZE(i2c1_mpu_irqs),
	.sdma_chs	= i2c1_sdma_chs,
	.sdma_chs_cnt	= ARRAY_SIZE(i2c1_sdma_chs),
	.clkdev_dev_id	= "i2c_omap.1",
	.clkdev_con_id	= "fck",
	.prcm		= {
		.omap2 = {
			.prcm_reg_id = 1,
			.module_bit = OMAP3430_GRPSEL_I2C1_SHIFT,
		},
	},
	.slaves		= omap34xx_i2c1_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_i2c1_slaves),
	.sysconfig	= &i2c_if_ctrl,
	.dev_attr	= &i2c1_dev_attr,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

/* I2C2 */

static struct omap_i2c_dev_attr i2c2_dev_attr = {
	.fifo_depth	= 8, /* bytes */
};

static struct omap_hwmod_irq_info i2c2_mpu_irqs[] = {
	{ .irq = INT_24XX_I2C2_IRQ, },
};

static struct omap_hwmod_dma_info i2c2_sdma_chs[] = {
	{ .name = "tx", .dma_ch = OMAP24XX_DMA_I2C2_TX },
	{ .name = "rx", .dma_ch = OMAP24XX_DMA_I2C2_RX },
};

static struct omap_hwmod_ocp_if *omap34xx_i2c2_slaves[] = {
	&omap34xx_l4_core__i2c2,
};

static struct omap_hwmod omap34xx_i2c2 = {
	.name		= "i2c2",
	.mpu_irqs	= i2c2_mpu_irqs,
	.mpu_irqs_cnt	= ARRAY_SIZE(i2c2_mpu_irqs),
	.sdma_chs	= i2c2_sdma_chs,
	.sdma_chs_cnt	= ARRAY_SIZE(i2c2_sdma_chs),
	.clkdev_dev_id	= "i2c_omap.2",
	.clkdev_con_id	= "fck",
	.prcm		= {
		.omap2 = {
			.prcm_reg_id = 1,
			.module_bit = OMAP3430_GRPSEL_I2C2_SHIFT,
		},
	},
	.slaves		= omap34xx_i2c2_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_i2c2_slaves),
	.sysconfig	= &i2c_if_ctrl,
	.dev_attr	= &i2c2_dev_attr,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

/* I2C3 */

static struct omap_i2c_dev_attr i2c3_dev_attr = {
	.fifo_depth	= 64, /* bytes */
};

static struct omap_hwmod_irq_info i2c3_mpu_irqs[] = {
	{ .irq = INT_34XX_I2C3_IRQ, },
};

static struct omap_hwmod_dma_info i2c3_sdma_chs[] = {
	{ .name = "tx", .dma_ch = OMAP34XX_DMA_I2C3_TX },
	{ .name = "rx", .dma_ch = OMAP34XX_DMA_I2C3_RX },
};

static struct omap_hwmod_ocp_if *omap34xx_i2c3_slaves[] = {
	&omap34xx_l4_core__i2c3,
};

static struct omap_hwmod omap34xx_i2c3 = {
	.name		= "i2c3",
	.mpu_irqs	= i2c3_mpu_irqs,
	.mpu_irqs_cnt	= ARRAY_SIZE(i2c3_mpu_irqs),
	.sdma_chs	= i2c3_sdma_chs,
	.sdma_chs_cnt	= ARRAY_SIZE(i2c3_sdma_chs),
	.clkdev_dev_id	= "i2c_omap.3",
	.clkdev_con_id	= "fck",
	.prcm		= {
		.omap2 = {
			.prcm_reg_id = 1,
			.module_bit = OMAP3430_GRPSEL_I2C3_SHIFT,
		},
	},
	.slaves		= omap34xx_i2c3_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_i2c3_slaves),
	.sysconfig	= &i2c_if_ctrl,
	.dev_attr	= &i2c3_dev_attr,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

/*
 * GPIO1 (GPIO1)
 */

static struct omap_hwmod_sysconfig gpio_if_ctrl = {
	.rev_offs	= 0x0000,
	.sysc_offs	= 0x0010,
	.syss_offs	= 0x0014,
	.sysc_flags	= (SYSC_HAS_SIDLEMODE |
			   SYSC_HAS_ENAWAKEUP | SYSC_HAS_SOFTRESET |
			   SYSC_HAS_AUTOIDLE),
	.idlemodes	= (SIDLE_FORCE | SIDLE_NO | SIDLE_SMART),
	.sysc_fields    = &omap_hwmod_sysc_type1,
};

static struct omap_hwmod_irq_info omap34xx_gpio1_mpu_irqs[] = {
	{ .name = "gpio_mpu_irq", .irq = INT_34XX_GPIO_BANK1 },
};

static struct omap_hwmod_opt_clk omap34xx_gpio1_opt_clk[] = {
	{
		.role = "gpio1_dbclk",
		.clkdev_dev_id	= NULL,
		.clkdev_con_id	= "gpio1_dbck",
	},
};

static struct omap_hwmod omap34xx_gpio1 = {
	.name		= "gpio1",
	.mpu_irqs	= omap34xx_gpio1_mpu_irqs,
	.mpu_irqs_cnt	= ARRAY_SIZE(omap34xx_gpio1_mpu_irqs),
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= NULL,
	.opt_clks	= omap34xx_gpio1_opt_clk,
	.opt_clks_cnt	= ARRAY_SIZE(omap34xx_gpio1_opt_clk),
	.prcm		= {
		.omap2 = {
			.prcm_reg_id = 1,
			.module_bit = OMAP3430_EN_GPIO1_SHIFT,
		},
	},
	.slaves		= omap34xx_gpio1_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_gpio1_slaves),
	.sysconfig	= &gpio_if_ctrl,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

/*
 * GPIO2 (GPIO2)
 */

static struct omap_hwmod_irq_info omap34xx_gpio2_mpu_irqs[] = {
	{ .name = "gpio_mpu_irq", .irq = INT_34XX_GPIO_BANK2 },
};

static struct omap_hwmod_opt_clk omap34xx_gpio2_opt_clk[] = {
	{
		.role = "gpio2_dbclk",
		.clkdev_dev_id	= NULL,
		.clkdev_con_id	= "gpio2_dbck",
	},
};

static struct omap_hwmod omap34xx_gpio2 = {
	.name		= "gpio2",
	.mpu_irqs	= omap34xx_gpio2_mpu_irqs,
	.mpu_irqs_cnt	= ARRAY_SIZE(omap34xx_gpio2_mpu_irqs),
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= NULL,
	.opt_clks	= omap34xx_gpio2_opt_clk,
	.opt_clks_cnt	= ARRAY_SIZE(omap34xx_gpio2_opt_clk),
	.prcm		= {
		.omap2 = {
			.prcm_reg_id = 1,
			.module_bit = OMAP3430_EN_GPIO2_SHIFT,
		},
	},
	.slaves		= omap34xx_gpio2_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_gpio2_slaves),
	.sysconfig	= &gpio_if_ctrl,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

/*
 * GPIO3 (GPIO3)
 */

static struct omap_hwmod_irq_info omap34xx_gpio3_mpu_irqs[] = {
	{ .name = "gpio_mpu_irq", .irq = INT_34XX_GPIO_BANK3 },
};

static struct omap_hwmod_opt_clk omap34xx_gpio3_opt_clk[] = {
	{
		.role = "gpio3_dbclk",
		.clkdev_dev_id	= NULL,
		.clkdev_con_id	= "gpio3_dbck",
	},
};

static struct omap_hwmod omap34xx_gpio3 = {
	.name		= "gpio3",
	.mpu_irqs	= omap34xx_gpio3_mpu_irqs,
	.mpu_irqs_cnt	= ARRAY_SIZE(omap34xx_gpio3_mpu_irqs),
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= NULL,
	.opt_clks	= omap34xx_gpio3_opt_clk,
	.opt_clks_cnt	= ARRAY_SIZE(omap34xx_gpio3_opt_clk),
	.prcm		= {
		.omap2 = {
			.prcm_reg_id = 1,
			.module_bit = OMAP3430_EN_GPIO3_SHIFT,
		},
	},
	.slaves		= omap34xx_gpio3_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_gpio3_slaves),
	.sysconfig	= &gpio_if_ctrl,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};


/*
 * GPIO4 (GPIO4)
 */

static struct omap_hwmod_irq_info omap34xx_gpio4_mpu_irqs[] = {
	{ .name = "gpio_mpu_irq", .irq = INT_34XX_GPIO_BANK4 },
};

static struct omap_hwmod_opt_clk omap34xx_gpio4_opt_clk[] = {
	{
		.role = "gpio4_dbclk",
		.clkdev_dev_id	= NULL,
		.clkdev_con_id	= "gpio4_dbck",
	},
};

static struct omap_hwmod omap34xx_gpio4 = {
	.name		= "gpio4",
	.mpu_irqs	= omap34xx_gpio4_mpu_irqs,
	.mpu_irqs_cnt	= ARRAY_SIZE(omap34xx_gpio4_mpu_irqs),
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= NULL,
	.opt_clks	= omap34xx_gpio4_opt_clk,
	.opt_clks_cnt	= ARRAY_SIZE(omap34xx_gpio4_opt_clk),
	.prcm		= {
		.omap2 = {
			.prcm_reg_id = 1,
			.module_bit = OMAP3430_EN_GPIO4_SHIFT,
		},
	},
	.slaves		= omap34xx_gpio4_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_gpio4_slaves),
	.sysconfig	= &gpio_if_ctrl,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};


/*
 * GPIO5 (GPIO5)
 */

static struct omap_hwmod_irq_info omap34xx_gpio5_mpu_irqs[] = {
	{ .name = "gpio_mpu_irq", .irq = INT_34XX_GPIO_BANK5 },
};

static struct omap_hwmod_opt_clk omap34xx_gpio5_opt_clk[] = {
	{
		.role = "gpio5_dbclk",
		.clkdev_dev_id	= NULL,
		.clkdev_con_id	= "gpio5_dbck",
	},
};
static struct omap_hwmod omap34xx_gpio5 = {
	.name		= "gpio5",
	.mpu_irqs	= omap34xx_gpio5_mpu_irqs,
	.mpu_irqs_cnt	= ARRAY_SIZE(omap34xx_gpio5_mpu_irqs),
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= NULL,
	.opt_clks	= omap34xx_gpio5_opt_clk,
	.opt_clks_cnt	= ARRAY_SIZE(omap34xx_gpio5_opt_clk),
	.prcm		= {
		.omap2 = {
			.prcm_reg_id = 1,
			.module_bit = OMAP3430_EN_GPIO5_SHIFT,
		},
	},
	.slaves		= omap34xx_gpio5_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_gpio5_slaves),
	.sysconfig	= &gpio_if_ctrl,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

/*
 * GPIO6 (GPIO6)
 */

static struct omap_hwmod_irq_info omap34xx_gpio6_mpu_irqs[] = {
	{ .name = "gpio_mpu_irq", .irq = INT_34XX_GPIO_BANK6 },
};

static struct omap_hwmod_opt_clk omap34xx_gpio6_opt_clk[] = {
	{
		.role = "gpio6_dbclk",
		.clkdev_dev_id	= NULL,
		.clkdev_con_id	= "gpio6_dbck",
	},
};

static struct omap_hwmod omap34xx_gpio6 = {
	.name		= "gpio6",
	.mpu_irqs	= omap34xx_gpio6_mpu_irqs,
	.mpu_irqs_cnt	= ARRAY_SIZE(omap34xx_gpio6_mpu_irqs),
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= NULL,
	.opt_clks	= omap34xx_gpio6_opt_clk,
	.opt_clks_cnt	= ARRAY_SIZE(omap34xx_gpio6_opt_clk),
	.prcm		= {
		.omap2 = {
			.prcm_reg_id = 1,
			.module_bit = OMAP3430_EN_GPIO6_SHIFT,
		},
	},
	.slaves		= omap34xx_gpio6_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_gpio6_slaves),
	.sysconfig	= &gpio_if_ctrl,
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
	&omap34xx_i2c1,
	&omap34xx_i2c2,
	&omap34xx_i2c3,
	&omap34xx_gpio1,
	&omap34xx_gpio2,
	&omap34xx_gpio3,
	&omap34xx_gpio4,
	&omap34xx_gpio5,
	&omap34xx_gpio6,
	NULL,
};

#else
# define omap34xx_hwmods		0
#endif

#endif


