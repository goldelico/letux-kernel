/*
 *
 * X2000 UART register definition.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __UART_H__
#define __UART_H__

#define readb(addr)     (*(volatile unsigned char *)(addr))
#define readw(addr)     (*(volatile unsigned short *)(addr))
#define readl(addr)     (*(volatile unsigned int *)(addr))
#define writeb(b, addr) (*(volatile unsigned char *)(addr)) = (b)
#define writew(b, addr) (*(volatile unsigned short *)(addr)) = (b)
#define writel(b, addr) (*(volatile unsigned int *)(addr)) = (b)

void serial_init(int);
void serial_puts(int, const char *);
void serial_put_hex(int, unsigned int);


#define	UART0_BASE	0xB0030000

/*************************************************************************
 * UART
 *************************************************************************/

#define UART_BASE	UART0_BASE
#define UART_OFF	0x1000

/* Register Offset */
#define OFF_RDR(n)		((n) * UART_OFF + 0x00)	/* R  8b H'xx */
#define OFF_TDR(n)		((n) * UART_OFF + 0x00)	/* W  8b H'xx */
#define OFF_DLLR(n)		((n) * UART_OFF + 0x00)	/* RW 8b H'00 */
#define OFF_DLHR(n)		((n) * UART_OFF + 0x04)	/* RW 8b H'00 */
#define OFF_IER(n)		((n) * UART_OFF + 0x04)	/* RW 8b H'00 */
#define OFF_ISR(n)		((n) * UART_OFF + 0x08)	/* R  8b H'01 */
#define OFF_FCR(n)		((n) * UART_OFF + 0x08)	/* W  8b H'00 */
#define OFF_LCR(n)		((n) * UART_OFF + 0x0C)	/* RW 8b H'00 */
#define OFF_MCR(n)		((n) * UART_OFF + 0x10)	/* RW 8b H'00 */
#define OFF_LSR(n)		((n) * UART_OFF + 0x14)	/* R  8b H'00 */
#define OFF_MSR(n)		((n) * UART_OFF + 0x18)	/* R  8b H'00 */
#define OFF_SPR(n)		((n) * UART_OFF + 0x1C)	/* RW 8b H'00 */
#define OFF_SIRCR(n)	((n) * UART_OFF + 0x20)	/* RW 8b H'00, UART0 */
#define OFF_UMR(n)		((n) * UART_OFF + 0x24)	/* RW 8b H'00, UART M Register */
#define OFF_UACR(n)		((n) * UART_OFF + 0x28)	/* RW 8b H'00, UART Add Cycle Register */
#define OFF_URCR(n)		((n) * UART_OFF + 0x40)	/* R  8b   00, UART RXFIFO Counter Register */
#define OFF_UTCR(n)		((n) * UART_OFF + 0x44)	/* R  8b   00, UART TXFIFO Counter Register */

/*
 * Define macros for UARTFCR
 * UART FIFO Control Register
 */
#define UARTFCR_FE	(1 << 0)	/* 0: non-FIFO mode  1: FIFO mode */
#define UARTFCR_RFLS	(1 << 1)	/* write 1 to flush receive FIFO */
#define UARTFCR_TFLS	(1 << 2)	/* write 1 to flush transmit FIFO */
#define UARTFCR_DMS	(1 << 3)	/* 0: disable DMA mode */
#define UARTFCR_UUE	(1 << 4)	/* 0: disable UART */
#define UARTFCR_RTRG	(3 << 6)	/* Receive FIFO Data Trigger */
#define UARTFCR_RTRG_1	(0 << 6)
#define UARTFCR_RTRG_16	(1 << 6)
#define UARTFCR_RTRG_32	(2 << 6)
#define UARTFCR_RTRG_60	(3 << 6)

/*
 * Define macros for UARTLCR
 * UART Line Control Register
 */
#define UARTLCR_WLEN	(3 << 0)	/* word length */
#define UARTLCR_WLEN_5	(0 << 0)
#define UARTLCR_WLEN_6	(1 << 0)
#define UARTLCR_WLEN_7	(2 << 0)
#define UARTLCR_WLEN_8	(3 << 0)
#define UARTLCR_STOP	(1 << 2)	/* 0: 1 stop bit when word length is 5,6,7,8
					   1: 1.5 stop bits when 5; 2 stop bits when 6,7,8 */
#define UARTLCR_STOP1	(0 << 2)
#define UARTLCR_STOP2	(1 << 2)
#define UARTLCR_PE	(1 << 3)	/* 0: parity disable */
#define UARTLCR_PROE	(1 << 4)	/* 0: even parity  1: odd parity */
#define UARTLCR_SPAR	(1 << 5)	/* 0: sticky parity disable */
#define UARTLCR_SBRK	(1 << 6)	/* write 0 normal, write 1 send break */
#define UARTLCR_DLAB	(1 << 7)	/* 0: access UARTRDR/TDR/IER  1: access UARTDLLR/DLHR */

/*
 * Define macros for SIRCR
 * Slow IrDA Control Register
 */
#define SIRCR_TSIRE	(1 << 0)  /* 0: transmitter is in UART mode  1: SIR mode */
#define SIRCR_RSIRE	(1 << 1)  /* 0: receiver is in UART mode  1: SIR mode */
#define SIRCR_TPWS	(1 << 2)  /* 0: transmit 0 pulse width is 3/16 of bit length
					   1: 0 pulse width is 1.6us for 115.2Kbps */
#define SIRCR_TDPL	(1 << 3)  /* 0: encoder generates a positive pulse for 0 */
#define SIRCR_RDPL	(1 << 4)  /* 0: decoder interprets positive pulse as 0 */

/*
 * Define macros for UART_LSR
 * UART Line Status Register
 */
#define UART_LSR_DR	(1 << 0)	/* 0: receive FIFO is empty  1: receive data is ready */
#define UART_LSR_ORER	(1 << 1)	/* 0: no overrun error */
#define UART_LSR_PER	(1 << 2)	/* 0: no parity error */
#define UART_LSR_FER	(1 << 3)	/* 0; no framing error */
#define UART_LSR_BRK	(1 << 4)	/* 0: no break detected  1: receive a break signal */
#define UART_LSR_TDRQ	(1 << 5)	/* 1: transmit FIFO half "empty" */
#define UART_LSR_TEMT	(1 << 6)	/* 1: transmit FIFO and shift registers empty */
#define UART_LSR_RFER	(1 << 7)	/* 0: no receive error  1: receive error in FIFO mode */

#define uart_readl(o)		readl(UART_BASE + (o))
#define uart_writel(b, o)	writel(b, UART_BASE + (o))

#endif /* __UART_H__ */

