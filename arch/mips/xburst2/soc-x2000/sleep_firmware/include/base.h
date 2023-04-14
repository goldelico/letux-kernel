
#ifndef __JZSOC_SOC_DEV_H__
#define __JZSOC_SOC_DEV_H__

/*
 * Define the module base addresses
 */
#define DDRC_BASE	0xb34f0000
#define DDRC_APB_OFFSET (-0x4e0000 + 0x2000)
#define DDR_PHY_OFFSET	(-0x4e0000 + 0x1000)

/* AHB0 BUS Devices Base */
#define HARB0_IOBASE	0x13000000 /* 64KB, AHB Bus Aribtier */
#define DDR_PHY_IOBASE	0x13011000 /* 4KB, DDR PHY */
#define DDRC_H0_IOBASE	0x13012000 /* 4KB, DDR Controller Registers@ASHB0 */
#define DPU_IOBASE	0x13050000 /* 64KB, LCD Controller */
#define CIM_IOBASE	0x13060000 /* 64KB, Camera Interface Module */
#define ROTATE_IOBASE	0x13070000 /* 64KB, Rotate DMA */
#define HELIX_IOBASE	0x13200000 /* 64KB VPU encoder */
#define FELIX_IOBASE	0x13300000 /* 64KB VPU decoder */
#define ISP0_IOBASE	0x13700000 /* 64KB Tiziano ISP0 */
#define ISP1_IOBASE	0x13800000 /* 64KB Tiziano ISP1 */

/* AHB2 Bus Devices Base */
#define HARB2_IOBASE	0x13400000 /* 64KB, AHB Bus Arbiter */
#define NEMC_IOBASE	0x13410000 /* 64KB, External Normal Memory/Boot ROM/OTP Controller */
#define PDMA_IOBASE	0x13420000 /* 64KB, Programmable DMA Controller */
#define AES_IOBASE	0x13430000 /* 64KB, AES */
#define SFC_IOBASE	0x13440000 /* 64KB, SPI Flash Controller */
#define MSC0_IOBASE	0x13450000 /* 64KB, MMC SD Controller0 */
#define MSC1_IOBASE	0x13460000 /* 64KB, MMC SD Controller1 */
#define MSC2_IOBASE	0x13490000 /* 64KB, MMC SD Controller2 */
#define HASH_IOBASE	0x13470000 /* 64KB, HASH */
#define RSA_IOBASE	0x13480000 /* 64KB, RSA */
#define MAC0_IOBASE	0x134b0000 /* 64KB, GMAC/MAC */
#define MAC1_IOBASE	0x134a0000 /* 64KB, GMAC/MAC */
#define PWM_IOBASE	0x134c0000 /* 64KB, PWM */
#define DDRC_H2_IOBASE	0x134f0000 /* 4KB, DDR Controller Register @AHB2 */
#define AUDIO_IOBASE	0x134d0000 /* 64KB, Aduio System */
#define OTG_IOBASE	0x13500000 /* 256KB, OTG2.0 Controller */
#define EFUSE_IOBASE	0x13540000 /* 64KB, EFUSE */
//#define INTC_IOBASE	0x13600000 /* Interrupt Controller */


/* APB BUS Devices Base */
#define CPM_IOBASE	0x10000000 /* 4KB, Clocks and Power Manager */
#define TCU_IOBASE	0x10002000 /* 4KB, Timer/Count Unit,OperatingSystemTimer,Watchdog Timer */
#define WDT_IOBASE	TCU_IOBASE /* 4KB, Timer/Count Unit,OperatingSystemTimer,Watchdog Timer */
#define RTC_IOBASE	0x10003000 /* 4KB, Real-Time Clock */
#define GPIO_IOBASE	0x10010000 /* 4KB, General-Purpose I/O */
#define CODEC_IOBASE	0x10020000 /* 4KB, icodec I/O */
#define	UART0_IOBASE	0x10030000 /* 4KB, UART0 Controller */
#define	UART1_IOBASE	0x10031000 /* 4KB, UART1 Controller */
#define	UART2_IOBASE	0x10032000 /* 4KB, UART2 Controller */
#define	UART3_IOBASE	0x10033000 /* 4KB, UART3 Controller */
#define	UART4_IOBASE	0x10034000 /* 4KB, UART4 Controller */
#define	UART5_IOBASE	0x10035000 /* 4KB, UART5 Controller */
#define	UART6_IOBASE	0x10036000 /* 4KB, UART6 Controller */
#define	UART7_IOBASE	0x10037000 /* 4KB, UART7 Controller */
#define	UART8_IOBASE	0x10038000 /* 4KB, UART8 Controller */
#define	UART9_IOBASE	0x10039000 /* 4KB, UART9 Controller */
#define SCC_IOBASE	0x10040000 /* 4KB, Smart Card Controller */
#define SSI0_IOBASE	0x10043000 /* 4KB, Synchronous Serial Interface */
#define SSI1_IOBASE	0x10044000 /* 4KB, Synchronous Serial Interface */
#define I2C0_IOBASE	0x10050000 /* 4KB, I2C 0 Bus Interface */
#define I2C1_IOBASE	0x10051000 /* 4KB, I2C 1 Bus Interface */
#define I2C2_IOBASE	0x10052000 /* 4KB, I2C 2 Bus Interface */
#define I2C3_IOBASE	0x10053000 /* 4KB, I2C 3 Bus Interface */
#define I2C4_IOBASE	0x10054000 /* 4KB, I2C 4 Bus Interface */
#define I2C5_IOBASE	0x10055000 /* 4KB, I2C 5 Bus Interface */
#define SADC_IOBASE	0x10070000 /* 4KB, SADC */
#define DTRNG_IOBASE	0x10072000 /* 4KB, DTRNG */
#define MIPI_RX_2_IOBASE	0x10073000 /* 4KB, MIPI RX 2 Lane ctrl */
#define MIPI_RX_4_IOBASE	0x10074000 /* 4KB, MIPI RX 4 lane ctrl */
#define MIPI_TX_2_IOBASE	0x10075000 /* 4KB, MIPI TX 2 lane ctrl */
#define MIPI_RX_PHY_IOBASE	0x10076000 /* 4KB, MIPI RX PHY  */
#define MIPI_TX_PHY_IOBASE	0x10077000 /* 4KB, MIPI TX PHY  */
#define USB_PHY_IOBASE		0x10078000 /* 4KB, USB PHY */

/* TODO: Others To Be add */
/* NAND CHIP Base Address */
#define NEMC_CS1_IOBASE 0X1b000000
#define NEMC_CS2_IOBASE 0X1a000000
#define NEMC_CS3_IOBASE 0X19000000
#define NEMC_CS4_IOBASE 0X18000000
#define NEMC_CS5_IOBASE 0X17000000
#define NEMC_CS6_IOBASE 0X16000000

#define	OST_IOBASE	0x12000000

#endif
