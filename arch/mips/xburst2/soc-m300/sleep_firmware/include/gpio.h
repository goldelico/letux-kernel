/*
 *
 * X2000 GPIO register definition.
 *
 * Copyright (C) 2012 Ingenic Semiconductor Co., Ltd.
 */

/* check */

#ifndef __GPIO_H__
#define __GPIO_H__


/*************************************************************************
 * GPIO (General-Purpose I/O Ports)
 *************************************************************************/
#define GPIO_BASE	0xB0010000

#define GPA(x) 		(32 * 0 + (x))
#define GPB(x) 		(32 * 1 + (x))
#define GPC(x) 		(32 * 2 + (x))
#define GPD(x) 		(32 * 3 + (x))
#define GPE(x) 		(32 * 4 + (x))
#define GPF(x) 		(32 * 5 + (x))

#define GPIO_PORT_NUM   6
#define MAX_GPIO_NUM	192
#define GPIO_WAKEUP     GPF(31)  //WAKE_UP key PF31

//n = 0,1,2,3,4 (PORTA, PORTB, PORTC, PORTD, PORTE)
#define GPIO_PXPIN(n)	(0x00 + (n) * 0x100) /* PIN Level Register */

#define GPIO_PXINT(n)	(0x10 + (n) * 0x100) /* Port Interrupt Register */
#define GPIO_PXINTS(n)	(0x14 + (n) * 0x100) /* Port Interrupt Set Register */
#define GPIO_PXINTC(n)	(0x18 + (n) * 0x100) /* Port Interrupt Clear Register */

#define GPIO_PXMASK(n)	(0x20 + (n) * 0x100) /* Port Interrupt Mask Register */
#define GPIO_PXMASKS(n)	(0x24 + (n) * 0x100) /* Port Interrupt Mask Set Reg */
#define GPIO_PXMASKC(n)	(0x28 + (n) * 0x100) /* Port Interrupt Mask Clear Reg */

#define GPIO_PXPAT1(n)	(0x30 + (n) * 0x100) /* Port Pattern 1 Register */
#define GPIO_PXPAT1S(n)	(0x34 + (n) * 0x100) /* Port Pattern 1 Set Reg. */
#define GPIO_PXPAT1C(n)	(0x38 + (n) * 0x100) /* Port Pattern 1 Clear Reg. */

#define GPIO_PXPAT0(n)	(0x40 + (n) * 0x100) /* Port Pattern 0 Register */
#define GPIO_PXPAT0S(n)	(0x44 + (n) * 0x100) /* Port Pattern 0 Set Register */
#define GPIO_PXPAT0C(n)	(0x48 + (n) * 0x100) /* Port Pattern 0 Clear Register */

#define GPIO_PXFLG(n)	(0x50 + (n) * 0x100) /* Port Flag Register */
#define GPIO_PXFLGC(n)	(0x58 + (n) * 0x100) /* Port Flag clear Register */

#define GPIO_PXPU(n)	(0x80 + (n) * 0x100) /* PORT PULL-UP State Register */
#define GPIO_PXPUS(n)	(0x84 + (n) * 0x100) /* PORT PULL-UP State Set Register*/
#define GPIO_PXPUC(n)	(0x88 + (n) * 0x100) /* PORT PULL-UP State Clear Register */

#define GPIO_PXPD(n)	(0x90 + (n) * 0x100) /* PORT PULL-DOWN State Register */
#define GPIO_PXPDS(n)	(0x94 + (n) * 0x100) /* PORT PULL-DOWN State Set Register */
#define GPIO_PXPDC(n)	(0x98 + (n) * 0x100) /* PORT PULL-DOWN State Set Register */

#define gpio_readl(o)		readl(GPIO_BASE + (o))
#define gpio_writel(b, o)	writel(b, GPIO_BASE + (o))

#ifndef __MIPS_ASSEMBLER

/*----------------------------------------------------------------
 * p is the port number (0,1,2,3,4,5)
 * o is the pin offset (0-31) inside the port
 * n is the absolute number of a pin (0-127), regardless of the port
 */

//----------------------------------------------------------------
/*
 * Gpio Pull set
 */
#define __gpio_set_hiz(v, p)			\
do {						\
	unsigned long val = v;			\
	gpio_writel(val, GPIO_PXPUC(p));	\
	gpio_writel(val, GPIO_PXPDC(p));	\
} while (0)

/*
 * UART0_TxD, UART0_RxD, PD27
 */
#define __gpio_as_uart0()			\
do {						\
	gpio_writel((3 << 23), GPIO_PXINTC(3));	\
	gpio_writel((3 << 23), GPIO_PXMASKC(3));\
	gpio_writel((3 << 23), GPIO_PXPAT1S(3));\
	gpio_writel((3 << 23), GPIO_PXPAT0C(3));\
} while (0)

/*
 * UART1_TxD, UART1_RxD, PA23
 */
#define __gpio_as_uart1()			\
do {						\
	gpio_writel((3 << 23), GPIO_PXINTC(2));	\
	gpio_writel((3 << 23), GPIO_PXMASKC(2));	\
	gpio_writel((3 << 23), GPIO_PXPAT1C(2));	\
	gpio_writel((3 << 23), GPIO_PXPAT0S(2));	\
} while (0)

/*
 * UART2_TxD, UART2_RxD
 */
#define __gpio_as_uart2()			\
do {						\
	gpio_writel((3 << 30), GPIO_PXINTC(3));	\
	gpio_writel((3 << 30), GPIO_PXMASKC(3));\
	gpio_writel((3 << 30), GPIO_PXPAT1C(3));\
	gpio_writel((3 << 30), GPIO_PXPAT0C(3));\
} while (0)

/*
 * CS2#, RD#, WR#, WAIT#, A0 ~ A5
 * @n: chip select number(1 ~ 4)
 */
#define __gpio_as_nor(n)				\
do {							\
	gpio_writel((0x7 << 13), GPIO_PXINTC(1));	\
	gpio_writel((0x7 << 13), GPIO_PXMASKC(1));	\
	gpio_writel((0x7 << 13), GPIO_PXPAT1C(1));	\
	gpio_writel((0x7 << 13), GPIO_PXPAT0C(1));	\
	gpio_writel((0x3 << 23), GPIO_PXINTC(2));	\
	gpio_writel((0x3 << 23), GPIO_PXMASKC(2));	\
	gpio_writel((0x3 << 23), GPIO_PXPAT1S(2));	\
	gpio_writel((0x3 << 23), GPIO_PXPAT0S(2));	\
} while (0)
/*
 * D0 ~ D7
 */
#define __gpio_as_nor_8bit()						\
do {									\
	gpio_writel((0xffffffff), GPIO_PXINTC(1));		\
	gpio_writel((0xffffffff), GPIO_PXMASKC(1));		\
	gpio_writel((0xffffffff), GPIO_PXPAT1C(1));		\
	gpio_writel((0xffffffff), GPIO_PXPAT0C(1));		\
} while (0)

/*
 * D0 ~ D15
 */
#define __gpio_as_nor_16bit()						\
do {	               							\
	gpio_writel((0xffffffff), GPIO_PXINTC(1));		\
	gpio_writel((0xffffffff), GPIO_PXMASKC(1));		\
	gpio_writel((0xffffffff), GPIO_PXPAT1C(1));		\
	gpio_writel((0xffffffff), GPIO_PXPAT0C(1));		\
} while (0)


/* gpio as sfc disable pull */
/* PE16 - PE21 */
#define __gpio_as_3_3V_sfc()				\
do {						\
	gpio_writel((0x3f << 16), GPIO_PXINTC(4));	\
	gpio_writel((0x3f << 16), GPIO_PXMASKC(4));	\
	gpio_writel((0x3f << 16), GPIO_PXPAT1C(4));	\
	gpio_writel((0x3f << 16), GPIO_PXPAT0C(4));	\
	if(!EFUSE_SWC_SFC)				\
		__gpio_set_hiz((0x3f) << 16, 4);	\
} while (0)

/* gpio as sfc disable pull */
/* PD17 - PD22 */
#define __gpio_as_1_8V_sfc()				\
do {						\
	gpio_writel((0x3f << 17), GPIO_PXINTC(3));	\
	gpio_writel((0x3f << 17), GPIO_PXMASKC(3));	\
	gpio_writel((0x3f << 17), GPIO_PXPAT1C(3));	\
	gpio_writel((0x3f << 17), GPIO_PXPAT0S(3));	\
	if(!EFUSE_SWC_SFC)				\
		__gpio_set_hiz((0x3f) << 17, 3);	\
} while (0)

/*
 * MSC0_CMD, MSC0_CLK, MSC0_D0 ~ MSC0_D7
 */

/* pull disable */

/* PD17 - PD26, func0, 1bit*/
#define __gpio_as_msc0_1bit()			\
do {						\
	gpio_writel(0x7 << 17, GPIO_PXINTC(3));	\
	gpio_writel(0x7 << 17, GPIO_PXMASKC(3));	\
	gpio_writel(0x7 << 17, GPIO_PXPAT1C(3));	\
	gpio_writel(0x7 << 17, GPIO_PXPAT0C(3));	\
	if(!EFUSE_SWC_MSC)			\
		__gpio_set_hiz((0x7 << 17), 3);	\
} while (0)

/* PD17 - PD26, func0, 4bit*/
#define __gpio_as_msc0_4bit()			\
do {						\
	gpio_writel(0x3f << 17, GPIO_PXINTC(3));	\
	gpio_writel(0x3f << 17, GPIO_PXMASKC(3));	\
	gpio_writel(0x3f << 17, GPIO_PXPAT1C(3));	\
	gpio_writel(0x3f << 17, GPIO_PXPAT0C(3));	\
	if(!EFUSE_SWC_MSC)			\
		__gpio_set_hiz((0x3f << 17), 3);	\
} while (0)


/* PD08 - PD13, func0, 1bit*/
#define __gpio_as_msc1_1bit()			\
do {						\
	gpio_writel(0x7 << 8, GPIO_PXINTC(3));	\
	gpio_writel(0x7 << 8, GPIO_PXMASKC(3));	\
	gpio_writel(0x7 << 8, GPIO_PXPAT1C(3));	\
	gpio_writel(0x7 << 8, GPIO_PXPAT0C(3));	\
	if(!EFUSE_SWC_MSC)			\
		__gpio_set_hiz((0x7 << 8), 3);	\
} while (0)

/* PD08 - PD13, func0, 4bit*/
#define __gpio_as_msc1_4bit()			\
do {						\
	gpio_writel(0x3f << 8, GPIO_PXINTC(3));	\
	gpio_writel(0x3f << 8, GPIO_PXMASKC(3));	\
	gpio_writel(0x3f << 8, GPIO_PXPAT1C(3));	\
	gpio_writel(0x3f << 8, GPIO_PXPAT0C(3));	\
	if(!EFUSE_SWC_MSC)			\
		__gpio_set_hiz((0x3f << 8), 3);	\
} while (0)


/* PE0 - PE5, func0 */
#define __gpio_as_msc4_1bit()			\
do {						\
	gpio_writel((0x7 << 0), GPIO_PXINTC(4));	\
	gpio_writel((0x7 << 0), GPIO_PXMASKC(4));	\
	gpio_writel((0x7 << 0), GPIO_PXPAT1C(4));	\
	gpio_writel((0x7 << 0), GPIO_PXPAT0C(4));	\
	if(!EFUSE_SWC_MSC)				\
		__gpio_set_hiz((0x7) << 0, 4);	\
} while (0)

/* PE0 - PE5, func0 */
#define __gpio_as_msc4_4bit()			\
do {						\
	gpio_writel((0x3f << 0), GPIO_PXINTC(4));	\
	gpio_writel((0x3f << 0), GPIO_PXMASKC(4));	\
	gpio_writel((0x3f << 0), GPIO_PXPAT1C(4));	\
	gpio_writel((0x3f << 0), GPIO_PXPAT0C(4));	\
	if(!EFUSE_SWC_MSC)				\
		__gpio_set_hiz((0x3f) << 0, 4);	\
} while (0)

#define bootsel_gpio_init() 			\
do{						\
	gpio_writel((0x7 << 25), GPIO_PXINTC(4));	\
	gpio_writel((0x7 << 25), GPIO_PXMASKS(4));\
	gpio_writel((0x7 << 25), GPIO_PXPAT1S(4));\
	gpio_writel((0x7 << 25), GPIO_PXPAT0C(4));\
	__gpio_set_hiz((0x7) << 25, 4);	\
}while(0)

#define boot_select()	((gpio_readl(GPIO_PXPIN(4)) >> 25) & 0x7)

#define BOOT_SELECT_3_3V	(0)
#define BOOT_SELECT_1_8V	(1)
#endif /* __MIPS_ASSEMBLER */

#endif /* __GPIO_H__ */

