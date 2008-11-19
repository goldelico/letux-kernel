/* linux/drivers/serial/s3c2410.c
 *
 * Driver for Samsung S3C2410 SoC onboard UARTs.
 *
 * Ben Dooks, Copyright (c) 2003-2005,2008 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/serial.h>

#include <asm/irq.h>
#include <mach/hardware.h>

#include <plat/regs-serial.h>
#include <mach/regs-gpio.h>

#include "samsung.h"

static int s3c2410_serial_setsource(struct uart_port *port,
				    struct s3c24xx_uart_clksrc *clk)
{
	unsigned long ucon = rd_regl(port, S3C2410_UCON);

	if (strcmp(clk->name, "uclk") == 0)
		ucon |= S3C2410_UCON_UCLK;
	else
		ucon &= ~S3C2410_UCON_UCLK;

	wr_regl(port, S3C2410_UCON, ucon);
	return 0;
}

static int s3c2410_serial_getsource(struct uart_port *port,
				    struct s3c24xx_uart_clksrc *clk)
{
	unsigned long ucon = rd_regl(port, S3C2410_UCON);

	clk->divisor = 1;
	clk->name = (ucon & S3C2410_UCON_UCLK) ? "uclk" : "pclk";

	return 0;
}

static int s3c2410_serial_resetport(struct uart_port *port,
				    struct s3c2410_uartcfg *cfg)
{
	dbg("s3c2410_serial_resetport: port=%p (%08lx), cfg=%p\n",
	    port, port->mapbase, cfg);

	wr_regl(port, S3C2410_UCON,  cfg->ucon);
	wr_regl(port, S3C2410_ULCON, cfg->ulcon);

	/* reset both fifos */

	wr_regl(port, S3C2410_UFCON, cfg->ufcon | S3C2410_UFCON_RESETBOTH);
	wr_regl(port, S3C2410_UFCON, cfg->ufcon);

	return 0;
}

static struct s3c24xx_uart_info s3c2410_uart_inf = {
	.name		= "Samsung S3C2410 UART",
	.type		= PORT_S3C2410,
	.fifosize	= 16,
	.rx_fifomask	= S3C2410_UFSTAT_RXMASK,
	.rx_fifoshift	= S3C2410_UFSTAT_RXSHIFT,
	.rx_fifofull	= S3C2410_UFSTAT_RXFULL,
	.tx_fifofull	= S3C2410_UFSTAT_TXFULL,
	.tx_fifomask	= S3C2410_UFSTAT_TXMASK,
	.tx_fifoshift	= S3C2410_UFSTAT_TXSHIFT,
	.get_clksrc	= s3c2410_serial_getsource,
	.set_clksrc	= s3c2410_serial_setsource,
	.reset_port	= s3c2410_serial_resetport,
};

static int s3c2410_serial_probe(struct platform_device *dev)
{
	return s3c24xx_serial_probe(dev, &s3c2410_uart_inf);
}

static struct platform_driver s3c2410_serial_drv = {
	.probe		= s3c2410_serial_probe,
	.remove		= s3c24xx_serial_remove,
	.driver		= {
		.name	= "s3c2410-uart",
		.owner	= THIS_MODULE,
	},
};

s3c24xx_console_init(&s3c2410_serial_drv, &s3c2410_uart_inf);

static int __init s3c2410_serial_init(void)
{
	return s3c24xx_serial_init(&s3c2410_serial_drv, &s3c2410_uart_inf);
}

static void __exit s3c2410_serial_exit(void)
{
	platform_driver_unregister(&s3c2410_serial_drv);
}

module_init(s3c2410_serial_init);
module_exit(s3c2410_serial_exit);

#define s3c2410_uart_inf_at &s3c2410_uart_inf
#else

static inline int s3c2410_serial_init(void)
{
	return 0;
}

static inline void s3c2410_serial_exit(void)
{
}

#define s3c2410_uart_inf_at NULL

#endif /* CONFIG_CPU_S3C2410 */

#if defined(CONFIG_CPU_S3C2440) || defined(CONFIG_CPU_S3C2442)

static int s3c2440_serial_setsource(struct uart_port *port,
				     struct s3c24xx_uart_clksrc *clk)
{
	unsigned long ucon = rd_regl(port, S3C2410_UCON);

	// todo - proper fclk<>nonfclk switch //

	ucon &= ~S3C2440_UCON_CLKMASK;

	if (strcmp(clk->name, "uclk") == 0)
		ucon |= S3C2440_UCON_UCLK;
	else if (strcmp(clk->name, "pclk") == 0)
		ucon |= S3C2440_UCON_PCLK;
	else if (strcmp(clk->name, "fclk") == 0)
		ucon |= S3C2440_UCON_FCLK;
	else {
		printk(KERN_ERR "unknown clock source %s\n", clk->name);
		return -EINVAL;
	}

	wr_regl(port, S3C2410_UCON, ucon);
	return 0;
}


static int s3c2440_serial_getsource(struct uart_port *port,
				    struct s3c24xx_uart_clksrc *clk)
{
	unsigned long ucon = rd_regl(port, S3C2410_UCON);
	unsigned long ucon0, ucon1, ucon2;

	switch (ucon & S3C2440_UCON_CLKMASK) {
	case S3C2440_UCON_UCLK:
		clk->divisor = 1;
		clk->name = "uclk";
		break;

	case S3C2440_UCON_PCLK:
	case S3C2440_UCON_PCLK2:
		clk->divisor = 1;
		clk->name = "pclk";
		break;

	case S3C2440_UCON_FCLK:
		/* the fun of calculating the uart divisors on
		 * the s3c2440 */

		ucon0 = __raw_readl(S3C24XX_VA_UART0 + S3C2410_UCON);
		ucon1 = __raw_readl(S3C24XX_VA_UART1 + S3C2410_UCON);
		ucon2 = __raw_readl(S3C24XX_VA_UART2 + S3C2410_UCON);

		printk("ucons: %08lx, %08lx, %08lx\n", ucon0, ucon1, ucon2);

		ucon0 &= S3C2440_UCON0_DIVMASK;
		ucon1 &= S3C2440_UCON1_DIVMASK;
		ucon2 &= S3C2440_UCON2_DIVMASK;

		if (ucon0 != 0) {
			clk->divisor = ucon0 >> S3C2440_UCON_DIVSHIFT;
			clk->divisor += 6;
		} else if (ucon1 != 0) {
			clk->divisor = ucon1 >> S3C2440_UCON_DIVSHIFT;
			clk->divisor += 21;
		} else if (ucon2 != 0) {
			clk->divisor = ucon2 >> S3C2440_UCON_DIVSHIFT;
			clk->divisor += 36;
		} else {
			/* manual calims 44, seems to be 9 */
			clk->divisor = 9;
		}

		clk->name = "fclk";
		break;
	}

	return 0;
}

static int s3c2440_serial_resetport(struct uart_port *port,
				    struct s3c2410_uartcfg *cfg)
{
	unsigned long ucon = rd_regl(port, S3C2410_UCON);

	dbg("s3c2440_serial_resetport: port=%p (%08lx), cfg=%p\n",
	    port, port->mapbase, cfg);

	/* ensure we don't change the clock settings... */

	ucon &= (S3C2440_UCON0_DIVMASK | (3<<10));

	wr_regl(port, S3C2410_UCON,  ucon | cfg->ucon);
	wr_regl(port, S3C2410_ULCON, cfg->ulcon);

	/* reset both fifos */

	wr_regl(port, S3C2410_UFCON, cfg->ufcon | S3C2410_UFCON_RESETBOTH);
	wr_regl(port, S3C2410_UFCON, cfg->ufcon);

	return 0;
}

static struct s3c24xx_uart_info s3c2440_uart_inf = {
	.name		= "Samsung S3C2440 UART",
	.type		= PORT_S3C2440,
	.fifosize	= 64,
	.rx_fifomask	= S3C2440_UFSTAT_RXMASK,
	.rx_fifoshift	= S3C2440_UFSTAT_RXSHIFT,
	.rx_fifofull	= S3C2440_UFSTAT_RXFULL,
	.tx_fifofull	= S3C2440_UFSTAT_TXFULL,
	.tx_fifomask	= S3C2440_UFSTAT_TXMASK,
	.tx_fifoshift	= S3C2440_UFSTAT_TXSHIFT,
	.get_clksrc	= s3c2440_serial_getsource,
	.set_clksrc	= s3c2440_serial_setsource,
	.reset_port	= s3c2440_serial_resetport,
};

/* device management */

static int s3c2440_serial_probe(struct platform_device *dev)
{
	dbg("s3c2440_serial_probe: dev=%p\n", dev);
	return s3c24xx_serial_probe(dev, &s3c2440_uart_inf);
}

static struct platform_driver s3c2440_serial_drv = {
	.probe		= s3c2440_serial_probe,
	.remove		= s3c24xx_serial_remove,
	.suspend	= s3c24xx_serial_suspend,
	.resume		= s3c24xx_serial_resume,
	.driver		= {
		.name	= "s3c2440-uart",
		.owner	= THIS_MODULE,
	},
};


static inline int s3c2440_serial_init(void)
{
	return s3c24xx_serial_init(&s3c2440_serial_drv, &s3c2440_uart_inf);
}

static inline void s3c2440_serial_exit(void)
{
	platform_driver_unregister(&s3c2440_serial_drv);
}

#define s3c2440_uart_inf_at &s3c2440_uart_inf
#else

static inline int s3c2440_serial_init(void)
{
	return 0;
}

static inline void s3c2440_serial_exit(void)
{
}

#define s3c2440_uart_inf_at NULL
#endif /* CONFIG_CPU_S3C2440 */

#if defined(CONFIG_CPU_S3C2412)

static int s3c2412_serial_setsource(struct uart_port *port,
				     struct s3c24xx_uart_clksrc *clk)
{
	unsigned long ucon = rd_regl(port, S3C2410_UCON);

	ucon &= ~S3C2412_UCON_CLKMASK;

	if (strcmp(clk->name, "uclk") == 0)
		ucon |= S3C2440_UCON_UCLK;
	else if (strcmp(clk->name, "pclk") == 0)
		ucon |= S3C2440_UCON_PCLK;
	else if (strcmp(clk->name, "usysclk") == 0)
		ucon |= S3C2412_UCON_USYSCLK;
	else {
		printk(KERN_ERR "unknown clock source %s\n", clk->name);
		return -EINVAL;
	}

	wr_regl(port, S3C2410_UCON, ucon);
	return 0;
}


static int s3c2412_serial_getsource(struct uart_port *port,
				    struct s3c24xx_uart_clksrc *clk)
{
	unsigned long ucon = rd_regl(port, S3C2410_UCON);

	switch (ucon & S3C2412_UCON_CLKMASK) {
	case S3C2412_UCON_UCLK:
		clk->divisor = 1;
		clk->name = "uclk";
		break;

	case S3C2412_UCON_PCLK:
	case S3C2412_UCON_PCLK2:
		clk->divisor = 1;
		clk->name = "pclk";
		break;

	case S3C2412_UCON_USYSCLK:
		clk->divisor = 1;
		clk->name = "usysclk";
		break;
	}

	return 0;
}

static int s3c2412_serial_resetport(struct uart_port *port,
				    struct s3c2410_uartcfg *cfg)
{
	unsigned long ucon = rd_regl(port, S3C2410_UCON);

	dbg("%s: port=%p (%08lx), cfg=%p\n",
	    __func__, port, port->mapbase, cfg);

	/* ensure we don't change the clock settings... */

	ucon &= S3C2412_UCON_CLKMASK;

	wr_regl(port, S3C2410_UCON,  ucon | cfg->ucon);
	wr_regl(port, S3C2410_ULCON, cfg->ulcon);

	/* reset both fifos */

	wr_regl(port, S3C2410_UFCON, cfg->ufcon | S3C2410_UFCON_RESETBOTH);
	wr_regl(port, S3C2410_UFCON, cfg->ufcon);

	return 0;
}

static struct s3c24xx_uart_info s3c2412_uart_inf = {
	.name		= "Samsung S3C2412 UART",
	.type		= PORT_S3C2412,
	.fifosize	= 64,
	.rx_fifomask	= S3C2440_UFSTAT_RXMASK,
	.rx_fifoshift	= S3C2440_UFSTAT_RXSHIFT,
	.rx_fifofull	= S3C2440_UFSTAT_RXFULL,
	.tx_fifofull	= S3C2440_UFSTAT_TXFULL,
	.tx_fifomask	= S3C2440_UFSTAT_TXMASK,
	.tx_fifoshift	= S3C2440_UFSTAT_TXSHIFT,
	.get_clksrc	= s3c2412_serial_getsource,
	.set_clksrc	= s3c2412_serial_setsource,
	.reset_port	= s3c2412_serial_resetport,
};

/* device management */

static int s3c2412_serial_probe(struct platform_device *dev)
{
	dbg("s3c2440_serial_probe: dev=%p\n", dev);
	return s3c24xx_serial_probe(dev, &s3c2412_uart_inf);
}

static struct platform_driver s3c2412_serial_drv = {
	.probe		= s3c2412_serial_probe,
	.remove		= s3c24xx_serial_remove,
	.suspend	= s3c24xx_serial_suspend,
	.resume		= s3c24xx_serial_resume,
	.driver		= {
		.name	= "s3c2412-uart",
		.owner	= THIS_MODULE,
	},
};


static inline int s3c2412_serial_init(void)
{
	return s3c24xx_serial_init(&s3c2412_serial_drv, &s3c2412_uart_inf);
}

static inline void s3c2412_serial_exit(void)
{
	platform_driver_unregister(&s3c2412_serial_drv);
}

#define s3c2412_uart_inf_at &s3c2412_uart_inf
#else

static inline int s3c2412_serial_init(void)
{
	return 0;
}

static inline void s3c2412_serial_exit(void)
{
}

#define s3c2412_uart_inf_at NULL
#endif /* CONFIG_CPU_S3C2440 */


/* module initialisation code */

static int __init s3c24xx_serial_modinit(void)
{
	int ret;

	ret = uart_register_driver(&s3c24xx_uart_drv);
	if (ret < 0) {
		printk(KERN_ERR "failed to register UART driver\n");
		return -1;
	}

	s3c2400_serial_init();
	s3c2410_serial_init();
	s3c2412_serial_init();
	s3c2440_serial_init();

	return 0;
}

static void __exit s3c24xx_serial_modexit(void)
{
	s3c2400_serial_exit();
	s3c2410_serial_exit();
	s3c2412_serial_exit();
	s3c2440_serial_exit();

	uart_unregister_driver(&s3c24xx_uart_drv);
}


module_init(s3c24xx_serial_modinit);
module_exit(s3c24xx_serial_modexit);

/* Console code */

#ifdef CONFIG_SERIAL_S3C2410_CONSOLE

static struct uart_port *cons_uart;

static int
s3c24xx_serial_console_txrdy(struct uart_port *port, unsigned int ufcon)
{
	struct s3c24xx_uart_info *info = s3c24xx_port_to_info(port);
	unsigned long ufstat, utrstat;

	if (ufcon & S3C2410_UFCON_FIFOMODE) {
		/* fifo mode - check ammount of data in fifo registers... */

		ufstat = rd_regl(port, S3C2410_UFSTAT);
		return (ufstat & info->tx_fifofull) ? 0 : 1;
	}

	/* in non-fifo mode, we go and use the tx buffer empty */

	utrstat = rd_regl(port, S3C2410_UTRSTAT);
	return (utrstat & S3C2410_UTRSTAT_TXE) ? 1 : 0;
}

static void
s3c24xx_serial_console_putchar(struct uart_port *port, int ch)
{
	unsigned int ufcon = rd_regl(cons_uart, S3C2410_UFCON);
	unsigned int umcon = rd_regl(cons_uart, S3C2410_UMCON);

	/* If auto HW flow control enabled, temporarily turn it off */
	if (umcon & S3C2410_UMCOM_AFC)
		wr_regl(port, S3C2410_UMCON, (umcon & !S3C2410_UMCOM_AFC));

	while (!s3c24xx_serial_console_txrdy(port, ufcon))
		barrier();
	wr_regb(cons_uart, S3C2410_UTXH, ch);

	if (umcon & S3C2410_UMCOM_AFC)
		wr_regl(port, S3C2410_UMCON, umcon);
}

static void
s3c24xx_serial_console_write(struct console *co, const char *s,
			     unsigned int count)
{
	uart_console_write(cons_uart, s, count, s3c24xx_serial_console_putchar);
}

static void __init
s3c24xx_serial_get_options(struct uart_port *port, int *baud,
			   int *parity, int *bits)
{
	struct s3c24xx_uart_clksrc clksrc;
	struct clk *clk;
	unsigned int ulcon;
	unsigned int ucon;
	unsigned int ubrdiv;
	unsigned long rate;

	ulcon  = rd_regl(port, S3C2410_ULCON);
	ucon   = rd_regl(port, S3C2410_UCON);
	ubrdiv = rd_regl(port, S3C2410_UBRDIV);

	dbg("s3c24xx_serial_get_options: port=%p\n"
	    "registers: ulcon=%08x, ucon=%08x, ubdriv=%08x\n",
	    port, ulcon, ucon, ubrdiv);

	if ((ucon & 0xf) != 0) {
		/* consider the serial port configured if the tx/rx mode set */

		switch (ulcon & S3C2410_LCON_CSMASK) {
		case S3C2410_LCON_CS5:
			*bits = 5;
			break;
		case S3C2410_LCON_CS6:
			*bits = 6;
			break;
		case S3C2410_LCON_CS7:
			*bits = 7;
			break;
		default:
		case S3C2410_LCON_CS8:
			*bits = 8;
			break;
		}

		switch (ulcon & S3C2410_LCON_PMASK) {
		case S3C2410_LCON_PEVEN:
			*parity = 'e';
			break;

		case S3C2410_LCON_PODD:
			*parity = 'o';
			break;

		case S3C2410_LCON_PNONE:
		default:
			*parity = 'n';
		}

		/* now calculate the baud rate */

		s3c24xx_serial_getsource(port, &clksrc);

		clk = clk_get(port->dev, clksrc.name);
		if (!IS_ERR(clk) && clk != NULL)
			rate = clk_get_rate(clk) / clksrc.divisor;
		else
			rate = 1;


		*baud = rate / ( 16 * (ubrdiv + 1));
		dbg("calculated baud %d\n", *baud);
	}

}

/* s3c24xx_serial_init_ports
 *
 * initialise the serial ports from the machine provided initialisation
 * data.
*/

static int s3c24xx_serial_init_ports(struct s3c24xx_uart_info *info)
{
	struct s3c24xx_uart_port *ptr = s3c24xx_serial_ports;
	struct platform_device **platdev_ptr;
	int i;

	dbg("s3c24xx_serial_init_ports: initialising ports...\n");

	platdev_ptr = s3c24xx_uart_devs;

	for (i = 0; i < NR_PORTS; i++, ptr++, platdev_ptr++) {
		s3c24xx_serial_init_port(ptr, info, *platdev_ptr);
	}

	return 0;
}

static int __init
s3c24xx_serial_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	dbg("s3c24xx_serial_console_setup: co=%p (%d), %s\n",
	    co, co->index, options);

	/* is this a valid port */

	if (co->index == -1 || co->index >= NR_PORTS)
		co->index = 0;

	port = &s3c24xx_serial_ports[co->index].port;

	/* is the port configured? */

	if (port->mapbase == 0x0) {
		co->index = 0;
		port = &s3c24xx_serial_ports[co->index].port;
	}

	cons_uart = port;

	dbg("s3c24xx_serial_console_setup: port=%p (%d)\n", port, co->index);

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		s3c24xx_serial_get_options(port, &baud, &parity, &bits);

	dbg("s3c24xx_serial_console_setup: baud %d\n", baud);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

/* s3c24xx_serial_initconsole
 *
 * initialise the console from one of the uart drivers
*/

static struct console s3c24xx_serial_console =
{
	.name		= S3C24XX_SERIAL_NAME,
	.device		= uart_console_device,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.write		= s3c24xx_serial_console_write,
	.setup		= s3c24xx_serial_console_setup
};

static int s3c24xx_serial_initconsole(void)
{
	struct s3c24xx_uart_info *info;
	struct platform_device *dev = s3c24xx_uart_devs[0];

	dbg("s3c24xx_serial_initconsole\n");

	/* select driver based on the cpu */

	if (dev == NULL) {
		printk(KERN_ERR "s3c24xx: no devices for console init\n");
		return 0;
	}

	if (strcmp(dev->name, "s3c2400-uart") == 0) {
		info = s3c2400_uart_inf_at;
	} else if (strcmp(dev->name, "s3c2410-uart") == 0) {
		info = s3c2410_uart_inf_at;
	} else if (strcmp(dev->name, "s3c2440-uart") == 0) {
		info = s3c2440_uart_inf_at;
	} else if (strcmp(dev->name, "s3c2412-uart") == 0) {
		info = s3c2412_uart_inf_at;
	} else {
		printk(KERN_ERR "s3c24xx: no driver for %s\n", dev->name);
		return 0;
	}

	if (info == NULL) {
		printk(KERN_ERR "s3c24xx: no driver for console\n");
		return 0;
	}

	s3c24xx_serial_console.data = &s3c24xx_uart_drv;
	s3c24xx_serial_init_ports(info);

	register_console(&s3c24xx_serial_console);
	return 0;
}

console_initcall(s3c24xx_serial_initconsole);

#endif /* CONFIG_SERIAL_S3C2410_CONSOLE */

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Ben Dooks <ben@simtec.co.uk>");
MODULE_DESCRIPTION("Samsung S3C2410 SoC Serial port driver");
MODULE_ALIAS("platform:s3c2410-uart");
