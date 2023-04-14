/*
 * X2000 CPM register definition
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __CPM_H__
#define __CPM_H__

#define BIT0            (1 << 0)
#define BIT1            (1 << 1)
#define BIT2            (1 << 2)
#define BIT3            (1 << 3)
#define BIT4            (1 << 4)
#define BIT5            (1 << 5)
#define BIT6            (1 << 6)
#define BIT7            (1 << 7)
#define BIT8            (1 << 8)
#define BIT9            (1 << 9)
#define BIT10           (1 << 10)
#define BIT11           (1 << 11)
#define BIT12           (1 << 12)
#define BIT13           (1 << 13)
#define BIT14           (1 << 14)
#define BIT15           (1 << 15)
#define BIT16           (1 << 16)
#define BIT17           (1 << 17)
#define BIT18           (1 << 18)
#define BIT19           (1 << 19)
#define BIT20           (1 << 20)
#define BIT21           (1 << 21)
#define BIT22           (1 << 22)
#define BIT23           (1 << 23)
#define BIT24           (1 << 24)
#define BIT25           (1 << 25)
#define BIT26           (1 << 26)
#define BIT27           (1 << 27)
#define BIT28           (1 << 28)
#define BIT29           (1 << 29)
#define BIT30           (1 << 30)
#define BIT31           (1 << 31)

/*
 * Clock reset and power controller module(CPM) address definition
 */
#define	CPM_BASE		0xB0000000

/*
 * CPM registers offset address definition
 */
#define CPM_CPCCR		(0x00)
#define CPM_RSR			(0x08)
#define CPM_RSACDR		(0x50)
#define CPM_CLKGR0		(0x20)
#define CPM_OPCR		(0x24)
#define CPM_CLKGR1		(0x28)
#define CPM_DDRCDR		(0x2c)
#define CPM_USBPCR		(0x3c)
#define CPM_USBRDT		(0x40)
#define CPM_USBVBFIL		(0x44)
#define CPM_USBPCR1		(0x48)
#define CPM_MSC0CDR		(0x68)
#define CPM_MSC1CDR		(0xA4)
#define CPM_MSC4CDR		(0xA8)
#define CPM_SFCCDR		(0x74)
#define CPM_CPCSR		(0xd4)
#define CPM_SLBC		(0xc8)
#define CPM_SLPC		(0xcc)
#define CPM_CPPCR		(0x0c)
#define CPM_CPAPCR		(0x10)
#define CPM_CPMPCR		(0x14)
#define CPM_EXCLK_DS		(0xE0)

/*
 * CPM registers common define
 */
/* Reset status register(RSR) */
#define RSR_P0R			BIT2
#define RSR_WR			BIT1
#define RSR_PR			BIT0

/* Clk Gate Register (CLKGR0) */
#define CLKGR0_DDR		BIT31
#define CLKGR0_RSA		BIT25
#define CLKGR0_AES		BIT24
#define CLKGR0_PDMA		BIT21
#define CLKGR0_OST		BIT20
#define CLKGR0_TCU		BIT18
#define CLKGR0_UART2		BIT16
#define CLKGR0_UART1		BIT15
#define CLKGR0_UART0		BIT14
#define CLKGR0_MSC1		BIT5
#define CLKGR0_MSC0		BIT4
#define CLKGR0_OTG		BIT3
#define CLKGR0_SFC		BIT2
#define CLKGR0_EFUSE		BIT1

/* Clk Gate Register (CLKGR1) */
#define CLKGR1_MSC4		BIT25
#define CLKGR1_HASH		BIT6

/* Msc Device Clock Divider Register (MSCnCDR) */
#define MSCCDR_DIV_LSB		0
#define MSCCDR_DIV_MASK		BITS_H2L(7, MSCCDR_DIV_LSB)

#define MSCCDR_MPCS_LSB		30
#define MSCCDR_MPCS_MASK	BITS_H2L(31, MSCCDR_MPCS_LSB)
#define MSCCDR_MPCS_EXCLK	(0x2 << MSCCDR_MPCS_LSB)

#define MSCCDR_BUSY		BIT28
#define MSCCDR_CE		BIT29
#define MSCCDR_EXCK_E		BIT21

/* Sleep Pc Register (SLPC) */
#define SLPC_SW_MAGIC		0x425753 // SWB (software boot)
#define SLPC_SW_USB_BOOT	(0x2 << 0)

/* APLL Control Register (CPAPCR) */
#define CPAPCR_PLLFD_LSB	20
#define CPAPCR_PLLFD_MASK	BITS_H2L(28, CPAPCR_PLLFD_LSB)

#define CPAPCR_PLLRD_LSB	14
#define CPAPCR_PLLRD_MASK	BITS_H2L(19, CPAPCR_PLLRD_LSB)

#define CPAPCR_PLLOD_LSB	11
#define CPAPCR_PLLOD_MASK	BITS_H2L(13, CPAPCR_PLLOD_LSB)

#define CPAPCR_PLLRG_LSB	5
#define CPAPCR_PLLRG_MASK	BITS_H2L(7, CPAPCR_PLLRG_LSB)

#define CPAPCR_PLL_ON		BIT3
#define CPAPCR_PLL_LOCK		BIT2
#define CPAPCR_PLL_EN		BIT0

/* EXCLK_DS MSC VOL SELECT */
#define EXCLKDS_POC_MSC		BIT31


#define cpm_readl(o)		readl(CPM_BASE + (o))
#define cpm_writel(b, o)	writel(b, CPM_BASE + (o))


#define __cpm_start_sfc()				\
	do {						\
		unsigned int tmp;			\
		tmp = cpm_readl(CPM_CLKGR0);		\
		tmp &= ~CLKGR0_SFC;			\
		cpm_writel(tmp, CPM_CLKGR0);		\
	} while (0)
#define __cpm_start_msc()				\
	do {						\
		unsigned int tmp;			\
		tmp = cpm_readl(CPM_CLKGR0);		\
		tmp &= ~(CLKGR0_MSC0 | CLKGR0_MSC1);	\
		cpm_writel(tmp, CPM_CLKGR0);		\
		tmp = cpm_readl(CPM_CLKGR1);		\
		tmp &= ~CLKGR1_MSC4;			\
		cpm_writel(tmp, CPM_CLKGR1);		\
	} while (0)
#define __cpm_start_otg()				\
	do {						\
		unsigned int tmp;			\
		tmp = cpm_readl(CPM_CLKGR0);		\
		tmp &= ~CLKGR0_OTG;			\
		cpm_writel(tmp, CPM_CLKGR0);		\
	} while (0)

#define __cpm_start_uart0()				\
	do {						\
		unsigned int tmp;			\
		tmp = cpm_readl(CPM_CLKGR0);		\
		tmp &= ~CLKGR0_UART0;			\
		cpm_writel(tmp, CPM_CLKGR0);		\
	} while (0)
#define __cpm_start_uart1()				\
	do {						\
		unsigned int tmp;			\
		tmp = cpm_readl(CPM_CLKGR0);		\
		tmp &= ~CLKGR0_UART1;			\
		cpm_writel(tmp, CPM_CLKGR0);		\
	} while (0)
#define __cpm_start_uart2()				\
	do {						\
		unsigned int tmp;			\
		tmp = cpm_readl(CPM_CLKGR0);		\
		tmp &= ~CLKGR0_UART2;			\
		cpm_writel(tmp, CPM_CLKGR0);		\
	} while (0)
#define __cpm_start_ost()				\
	do {						\
		unsigned int tmp;			\
		tmp = cpm_readl(CPM_CLKGR0);		\
		tmp &= ~CLKGR0_OST;			\
		cpm_writel(tmp, CPM_CLKGR0);		\
	} while (0)

#define __cpm_msc4_vol_1_8V()				\
	do {						\
		unsigned int tmp;			\
		tmp = cpm_readl(CPM_EXCLK_DS);		\
		tmp |= EXCLKDS_POC_MSC;			\
		cpm_writel(tmp, CPM_EXCLK_DS);		\
	} while (0)

#define __cpm_msc4_vol_3_3V()				\
	do {						\
		unsigned int tmp;			\
		tmp = cpm_readl(CPM_EXCLK_DS);		\
		tmp &= ~EXCLKDS_POC_MSC;			\
		cpm_writel(tmp, CPM_EXCLK_DS);		\
	} while (0)

#ifdef FPGA_TEST
#define __cpm_msc_tuning_set(v, n)				\
	do {							\
		unsigned int tmp;				\
		unsigned int off;				\
		if(n==0)					\
			off = CPM_MSC0CDR;			\
		else if(n==4)					\
			off = CPM_MSC4CDR;			\
		else						\
			off = CPM_MSC1CDR;			\
		tmp = cpm_readl(off);				\
		tmp &= ~(1 << 20);				\
		tmp |= ((v) & 0x1) << 20;			\
		cpm_writel(tmp, off);				\
	} while (0)
#endif

#endif /* __CPM_H__ */
