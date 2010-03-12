/*
 * linux/include/asm-arm/arch-omap/omap-serial.h
 *
 * register definitions for omap24xx/omap34xx uart controller.
 *
 * Copyright (C) 2008-2009 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * (at your option) any later version.
 */

#ifndef __OMAP_SERIAL_H__
#define __OMAP_SERIAL_H__

#include <linux/serial_core.h>
#include <linux/platform_device.h>

#include <plat/control.h>
#include <plat/mux.h>

#define DRIVER_NAME	"omap-hsuart"

/*
 * Use tty device name as ttyO, [O -> OMAP]
 * in bootargs we specify as console=ttyO0 if uart1
 * is used as console uart.
 * Use Major 204 and minor 213.
 * This is necessary in order to coexist with the 8250 driver,
 * if we have an external TI-16C750 UART. Ex.ZOOM2/3 Boards.
 */

#define OMAP_SERIAL_NAME	"ttyO"
#define OMAP_SERIAL_MAJOR	204
#define OMAP_SERIAL_MINOR	213

#define OMAP_MDR1_DISABLE	0x07
#define OMAP_MDR1_MODE13X	0x03
#define OMAP_MDR1_MODE16X	0x00
#define OMAP_MODE13X_SPEED	230400

#define RX_TIMEOUT		(3 * HZ)
#define OMAP_MAX_HSUART_PORTS	4

#define MSR_SAVE_FLAGS		UART_MSR_ANY_DELTA

struct uart_port_info {
	bool			dma_enabled;
	unsigned int		uartclk;
	};

struct uart_omap_dma {
	u8			uart_dma_tx;
	u8			uart_dma_rx;
	int			rx_dma_channel;
	int			tx_dma_channel;
	dma_addr_t		rx_buf_dma_phys;
	dma_addr_t		tx_buf_dma_phys;
	/*
	 * Buffer for rx dma.It is not required for tx because the buffer
	 * comes from port structure
	 */
	unsigned int		uart_base;
	unsigned char		*rx_buf;
	unsigned int		prev_rx_dma_pos;
	int			tx_buf_size;
	int			tx_dma_used;
	int			rx_dma_used;
	spinlock_t		tx_lock;
	spinlock_t		rx_lock;
	/* timer to poll activity on rx dma */
	struct timer_list	rx_timer;
	int			rx_buf_size;
	int			rx_timeout;
};

struct uart_omap_port {
	struct uart_port	port;
	struct uart_omap_dma	uart_dma;
	struct platform_device	*pdev;

	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	unsigned char		fcr;
	unsigned char		efr;

	int			use_dma;
	/*
	 * Some bits in registers are cleared on a read, so they must
	 * be saved whenever the register is read but the bits will not
	 * be immediately processed.
	 */
	unsigned int		lsr_break_flag;
	unsigned char		msr_saved_flags;
	char			name[20];
	spinlock_t		uart_lock;
	unsigned long		port_activity;
};

extern char *saved_command_line;
#endif /* __OMAP_SERIAL_H__ */
