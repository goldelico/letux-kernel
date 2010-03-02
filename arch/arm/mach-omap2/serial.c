/*
 * arch/arm/mach-omap2/serial.c
 *
 * OMAP2 serial support.
 *
 * Copyright (C) 2005-2008 Nokia Corporation
 * Author: Paul Mundt <paul.mundt@nokia.com>
 *
 * Major rework for PM support by Kevin Hilman
 *
 * Based off of arch/arm/mach-omap/omap1/serial.c
 *
 * Copyright (C) 2009 Texas Instruments
 * Added OMAP4 support - Santosh Shilimkar <santosh.shilimkar@ti.com
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/serial_8250.h>
#include <linux/serial_reg.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <plat/common.h>
#include <plat/board.h>
#include <plat/clock.h>
#include <plat/control.h>

#include <plat/dma.h>
#include <plat/omap-serial.h>

#include "prm.h"
#include "pm.h"
#include "prm-regbits-34xx.h"

#define UART_OMAP_NO_EMPTY_FIFO_READ_IP_REV	0x52
#define UART_OMAP_WER		0x17	/* Wake-up enable register */

#define DEFAULT_TIMEOUT (5 * HZ)

struct omap_uart_state {
	int num;
	int can_sleep;
	struct timer_list timer;
	u32 timeout;

	void __iomem *wk_st;
	void __iomem *wk_en;
	u32 wk_mask;
	u32 padconf;

	struct clk *ick;
	struct clk *fck;
	int clocked;

	int irq;
	int regshift;
	int irqflags;
	void __iomem *membase;
	resource_size_t mapbase;

	struct list_head node;
	struct platform_device pdev;

#if defined(CONFIG_ARCH_OMAP3) && defined(CONFIG_PM)
	int context_valid;

	/* Registers to be saved/restored for OFF-mode */
	u16 dll;
	u16 dlh;
	u16 ier;
	u16 sysc;
	u16 scr;
	u16 wer;
#endif
};

static LIST_HEAD(uart_list);

#ifndef CONFIG_SERIAL_OMAP
static struct plat_serial8250_port serial_platform_data0[] = {
	{
		.mapbase	= OMAP_UART1_BASE,
		.irq		= 72,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= OMAP24XX_BASE_BAUD * 16,
	}, {
		.flags		= 0
	}
};

static struct plat_serial8250_port serial_platform_data1[] = {
	{
		.mapbase	= OMAP_UART2_BASE,
		.irq		= 73,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= OMAP24XX_BASE_BAUD * 16,
	}, {
		.flags		= 0
	}
};

static struct plat_serial8250_port serial_platform_data2[] = {
	{
		.mapbase	= OMAP_UART3_BASE,
		.irq		= 74,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= OMAP24XX_BASE_BAUD * 16,
	}, {
		.flags		= 0
	}
};

#ifdef CONFIG_ARCH_OMAP4
static struct plat_serial8250_port serial_platform_data3[] = {
	{
		.mapbase	= OMAP_UART4_BASE,
		.irq		= 70,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= OMAP24XX_BASE_BAUD * 16,
	}, {
		.flags		= 0
	}
};
#endif

static struct omap_uart_state omap_uart[] = {
	{
		.pdev = {
			.name			= "serial8250",
			.id			= PLAT8250_DEV_PLATFORM,
			.dev			= {
				.platform_data	= serial_platform_data0,
			},
		},
	}, {
		.pdev = {
			.name			= "serial8250",
			.id			= PLAT8250_DEV_PLATFORM1,
			.dev			= {
				.platform_data	= serial_platform_data1,
			},
		},
	}, {
		.pdev = {
			.name			= "serial8250",
			.id			= PLAT8250_DEV_PLATFORM2,
			.dev			= {
				.platform_data	= serial_platform_data2,
			},
		},
	},
#ifdef CONFIG_ARCH_OMAP4
	{
		.pdev = {
			.name			= "serial8250",
			.id			= 3,
			.dev			= {
				.platform_data	= serial_platform_data3,
			},
		},
	},
#endif
};

#else
static struct uart_port_info uart1_port_info = {
		.dma_enabled	= 0,
		.uartclk	= OMAP24XX_BASE_BAUD * 16
};

static struct uart_port_info uart2_port_info = {
		.dma_enabled	= 0,
		.uartclk	= OMAP24XX_BASE_BAUD * 16
};

static struct uart_port_info uart3_port_info = {
		.dma_enabled	= 0,
		.uartclk	= OMAP24XX_BASE_BAUD * 16
};

#ifdef CONFIG_ARCH_OMAP4
static struct uart_port_info uart4_port_info = {
		.dma_enabled	= 0,
		.uartclk	= OMAP24XX_BASE_BAUD * 16
};
#endif

static struct resource omap_uart1_resources[] = {
	{
		.start          = OMAP_UART1_BASE,
		.end            = OMAP_UART1_BASE + 0x3ff,
		.flags          = IORESOURCE_MEM,
	}, {
		/* UART1 IRQ - 72*/
		.start          = INT_24XX_UART1_IRQ,
		.flags          = IORESOURCE_IRQ,
	}, {
		/* UART1 TX DMA CHANNEL -S_DMA_48- */
		.start		= OMAP24XX_DMA_UART1_TX,
		.flags		= IORESOURCE_DMA,
	}, {
		/* UART1 RX DMA CHANNEL -S_DMA_49- */
		.start		= OMAP24XX_DMA_UART1_RX,
		.flags		= IORESOURCE_DMA,
	}
};

static struct resource omap_uart2_resources[] = {
	{
		.start          = OMAP_UART2_BASE,
		.end            = OMAP_UART2_BASE + 0x3ff,
		.flags          = IORESOURCE_MEM,
	}, {
		/* UART2 IRQ - 73*/
		.start          = INT_24XX_UART2_IRQ,
		.flags          = IORESOURCE_IRQ,
	}, {
		/* UART2 TX DMA CHANNEL -S_DMA_50- */
		.start		= OMAP24XX_DMA_UART2_TX,
		.flags		= IORESOURCE_DMA,
	}, {
		/* UART2 RX DMA CHANNEL -S_DMA_51- */
		.start		= OMAP24XX_DMA_UART2_RX,
		.flags		= IORESOURCE_DMA,
	}
};

static struct resource omap_uart3_resources[] = {
	{
		.start          = OMAP_UART3_BASE,
		.end            = OMAP_UART3_BASE + 0x3ff,
		.flags          = IORESOURCE_MEM,
	}, {
		/* UART3 IRQ - 74*/
		.start          = INT_24XX_UART3_IRQ,
		.flags          = IORESOURCE_IRQ,
	}, {
		/* UART3 TX DMA CHANNEL -S_DMA_52- */
		.start		= OMAP24XX_DMA_UART3_TX,
		.flags		= IORESOURCE_DMA,
	}, {
		/* UART3 RX DMA CHANNEL -S_DMA_53- */
		.start		= OMAP24XX_DMA_UART3_RX,
		.flags		= IORESOURCE_DMA,
	}
};

#ifdef CONFIG_ARCH_OMAP4
static struct resource omap_uart4_resources[] = {
	{
		.start          = OMAP_UART4_BASE,
		.end            = OMAP_UART4_BASE + 0x3ff,
		.flags          = IORESOURCE_MEM,
	}, {
		/* UART4 IRQ - 70*/
		.start          = 70,
		.flags          = IORESOURCE_IRQ,
	}, {
		/* UART3 TX DMA CHANNEL -S_DMA_54- */
		.start		= OMAP44XX_DMA_UART4_TX,
		.flags		= IORESOURCE_DMA,
	}, {
		/* UART3 RX DMA CHANNEL -S_DMA_55- */
		.start		= OMAP44XX_DMA_UART4_RX,
		.flags		= IORESOURCE_DMA,
	}
};
#endif

static struct omap_uart_state omap_uart[] = {
	{
		.pdev = {
			.name		= DRIVER_NAME,
			.id		= 0,
			.num_resources	= ARRAY_SIZE(omap_uart1_resources),
			.resource	= omap_uart1_resources,
			.dev		= {
				.platform_data  = &uart1_port_info
			},
		}
	}, {
		.pdev = {
			.name		= DRIVER_NAME,
			.id		= 1,
			.num_resources	= ARRAY_SIZE(omap_uart2_resources),
			.resource	= omap_uart2_resources,
			.dev		= {
				.platform_data  = &uart2_port_info,
			},
		}
	}, {
		.pdev = {
			.name		= DRIVER_NAME,
			.id		= 2,
			.num_resources	= ARRAY_SIZE(omap_uart3_resources),
			.resource	= omap_uart3_resources,
			.dev		= {
				.platform_data  = &uart3_port_info,
			},
		}
	},
#ifdef CONFIG_ARCH_OMAP4
	{
		.pdev = {
			.name		= DRIVER_NAME,
			.id		= 3,
			.num_resources	= ARRAY_SIZE(omap_uart4_resources),
			.resource	= omap_uart4_resources,
			.dev		= {
				.platform_data  = &uart4_port_info,
			},
		}
	},
#endif
};
#endif

static inline unsigned int serial_read_reg(struct omap_uart_state *up,
					   int offset)
{
	offset <<= up->regshift;
	return (unsigned int)__raw_readb(up->membase + offset);
}

static inline void serial_write_reg(struct omap_uart_state *up, int offset,
				    int value)
{
	offset <<= up->regshift;
	__raw_writeb(value, up->membase + offset);
}

/*
 * Internal UARTs need to be initialized for the 8250 autoconfig to work
 * properly. Note that the TX watermark initialization may not be needed
 * once the 8250.c watermark handling code is merged.
 */
static inline void __init omap_uart_reset(struct omap_uart_state *p)
{
	serial_write_reg(p, UART_OMAP_MDR1, 0x07);
	serial_write_reg(p, UART_OMAP_SCR, 0x08);
	serial_write_reg(p, UART_OMAP_MDR1, 0x00);
	serial_write_reg(p, UART_OMAP_SYSC, (0x02 << 3) | (1 << 2) | (1 << 0));
}

#if defined(CONFIG_PM) && defined(CONFIG_ARCH_OMAP3)

static void omap_uart_save_context(struct omap_uart_state *uart)
{
	u16 lcr = 0;

	if (!enable_off_mode)
		return;

	lcr = serial_read_reg(uart, UART_LCR);
	serial_write_reg(uart, UART_LCR, 0xBF);
	uart->dll = serial_read_reg(uart, UART_DLL);
	uart->dlh = serial_read_reg(uart, UART_DLM);
	serial_write_reg(uart, UART_LCR, lcr);
	uart->ier = serial_read_reg(uart, UART_IER);
	uart->sysc = serial_read_reg(uart, UART_OMAP_SYSC);
	uart->scr = serial_read_reg(uart, UART_OMAP_SCR);
	uart->wer = serial_read_reg(uart, UART_OMAP_WER);

	uart->context_valid = 1;
}

static void omap_uart_restore_context(struct omap_uart_state *uart)
{
	u16 efr = 0;

	if (!uart->context_valid)
		return;

	uart->context_valid = 0;

	serial_write_reg(uart, UART_OMAP_MDR1, 0x7);
	serial_write_reg(uart, UART_LCR, 0xBF); /* Config B mode */
	efr = serial_read_reg(uart, UART_EFR);
	serial_write_reg(uart, UART_EFR, UART_EFR_ECB);
	serial_write_reg(uart, UART_LCR, 0x0); /* Operational mode */
	serial_write_reg(uart, UART_IER, 0x0);
	serial_write_reg(uart, UART_LCR, 0xBF); /* Config B mode */
	serial_write_reg(uart, UART_DLL, uart->dll);
	serial_write_reg(uart, UART_DLM, uart->dlh);
	serial_write_reg(uart, UART_LCR, 0x0); /* Operational mode */
	serial_write_reg(uart, UART_IER, uart->ier);
	serial_write_reg(uart, UART_FCR, 0xA1);
	serial_write_reg(uart, UART_LCR, 0xBF); /* Config B mode */
	serial_write_reg(uart, UART_EFR, efr);
	serial_write_reg(uart, UART_LCR, UART_LCR_WLEN8);
	serial_write_reg(uart, UART_OMAP_SCR, uart->scr);
	serial_write_reg(uart, UART_OMAP_WER, uart->wer);
	serial_write_reg(uart, UART_OMAP_SYSC, uart->sysc);
	serial_write_reg(uart, UART_OMAP_MDR1, 0x00); /* UART 16x mode */
}
#else
static inline void omap_uart_save_context(struct omap_uart_state *uart) {}
static inline void omap_uart_restore_context(struct omap_uart_state *uart) {}
#endif /* CONFIG_PM && CONFIG_ARCH_OMAP3 */

static inline void omap_uart_enable_clocks(struct omap_uart_state *uart)
{
	if (uart->clocked)
		return;

	clk_enable(uart->ick);
	if (!cpu_is_omap44xx())
		clk_enable(uart->fck);
	uart->clocked = 1;
	omap_uart_restore_context(uart);
}

#ifdef CONFIG_PM

static inline void omap_uart_disable_clocks(struct omap_uart_state *uart)
{
	if (!uart->clocked)
		return;

	omap_uart_save_context(uart);
	uart->clocked = 0;
	clk_disable(uart->ick);
	if (!cpu_is_omap44xx())
		clk_disable(uart->fck);
}

static void omap_uart_enable_wakeup(struct omap_uart_state *uart)
{
	/* Set wake-enable bit */
	if (uart->wk_en && uart->wk_mask) {
		u32 v = __raw_readl(uart->wk_en);
		v |= uart->wk_mask;
		__raw_writel(v, uart->wk_en);
	}

	/* Ensure IOPAD wake-enables are set */
	if (cpu_is_omap34xx() && uart->padconf) {
		u16 v = omap_ctrl_readw(uart->padconf);
		v |= OMAP3_PADCONF_WAKEUPENABLE0;
		omap_ctrl_writew(v, uart->padconf);
	}
}

static void omap_uart_disable_wakeup(struct omap_uart_state *uart)
{
	/* Clear wake-enable bit */
	if (uart->wk_en && uart->wk_mask) {
		u32 v = __raw_readl(uart->wk_en);
		v &= ~uart->wk_mask;
		__raw_writel(v, uart->wk_en);
	}

	/* Ensure IOPAD wake-enables are cleared */
	if (cpu_is_omap34xx() && uart->padconf) {
		u16 v = omap_ctrl_readw(uart->padconf);
		v &= ~OMAP3_PADCONF_WAKEUPENABLE0;
		omap_ctrl_writew(v, uart->padconf);
	}
}

static void omap_uart_smart_idle_enable(struct omap_uart_state *p,
					  int enable)
{
	u16 sysc;

	sysc = serial_read_reg(p, UART_OMAP_SYSC) & 0x7;
	if (enable)
		sysc |= 0x2 << 3;
	else
		sysc |= 0x1 << 3;

	serial_write_reg(p, UART_OMAP_SYSC, sysc);
}

static void omap_uart_block_sleep(struct omap_uart_state *uart)
{
	omap_uart_enable_clocks(uart);

	omap_uart_smart_idle_enable(uart, 0);
	uart->can_sleep = 0;
	if (uart->timeout)
		mod_timer(&uart->timer, jiffies + uart->timeout);
	else
		del_timer(&uart->timer);
}

static void omap_uart_allow_sleep(struct omap_uart_state *uart)
{
	if (device_may_wakeup(&uart->pdev.dev))
		omap_uart_enable_wakeup(uart);
	else
		omap_uart_disable_wakeup(uart);

	if (!uart->clocked)
		return;

	omap_uart_smart_idle_enable(uart, 1);
	uart->can_sleep = 1;
	del_timer(&uart->timer);
}

static void omap_uart_idle_timer(unsigned long data)
{
	struct omap_uart_state *uart = (struct omap_uart_state *)data;

	omap_uart_allow_sleep(uart);
}

void omap_uart_prepare_idle(int num)
{
	struct omap_uart_state *uart;

	list_for_each_entry(uart, &uart_list, node) {
		if (num == uart->num && uart->can_sleep) {
			omap_uart_disable_clocks(uart);
			return;
		}
	}
}

void omap_uart_resume_idle(int num)
{
	struct omap_uart_state *uart;

	list_for_each_entry(uart, &uart_list, node) {
		if (num == uart->num) {
			omap_uart_enable_clocks(uart);

			/* Check for IO pad wakeup */
			if (cpu_is_omap34xx() && uart->padconf) {
				u16 p = omap_ctrl_readw(uart->padconf);

				if (p & OMAP3_PADCONF_WAKEUPEVENT0)
					omap_uart_block_sleep(uart);
			}

			/* Check for normal UART wakeup */
			if (uart->wk_st && uart->wk_mask)
				if(__raw_readl(uart->wk_st) & uart->wk_mask)
					omap_uart_block_sleep(uart);
			return;
		}
	}
}

void omap_uart_prepare_suspend(void)
{
	struct omap_uart_state *uart;

	list_for_each_entry(uart, &uart_list, node) {
		omap_uart_allow_sleep(uart);
	}
}

int omap_uart_can_sleep(void)
{
	struct omap_uart_state *uart;
	int can_sleep = 1;

	list_for_each_entry(uart, &uart_list, node) {
		if (!uart->clocked)
			continue;

		if (!uart->can_sleep) {
			can_sleep = 0;
			continue;
		}

		/* This UART can now safely sleep. */
		omap_uart_allow_sleep(uart);
	}

	return can_sleep;
}

/**
 * omap_uart_interrupt()
 *
 * This handler is used only to detect that *any* UART interrupt has
 * occurred.  It does _nothing_ to handle the interrupt.  Rather,
 * any UART interrupt will trigger the inactivity timer so the
 * UART will not idle or sleep for its timeout period.
 *
 **/
static irqreturn_t omap_uart_interrupt(int irq, void *dev_id)
{
	struct omap_uart_state *uart = dev_id;

	omap_uart_block_sleep(uart);

	return IRQ_NONE;
}

static void omap_uart_idle_init(struct omap_uart_state *uart)
{
	int ret;

	uart->can_sleep = 0;
	uart->timeout = DEFAULT_TIMEOUT;
	setup_timer(&uart->timer, omap_uart_idle_timer,
		    (unsigned long) uart);
	mod_timer(&uart->timer, jiffies + uart->timeout);
	omap_uart_smart_idle_enable(uart, 0);

	if (cpu_is_omap34xx()) {
		u32 mod = (uart->num == 2) ? OMAP3430_PER_MOD : CORE_MOD;
		u32 wk_mask = 0;
		u32 padconf = 0;

		uart->wk_en = OMAP34XX_PRM_REGADDR(mod, PM_WKEN1);
		uart->wk_st = OMAP34XX_PRM_REGADDR(mod, PM_WKST1);
		switch (uart->num) {
		case 0:
			wk_mask = OMAP3430_ST_UART1_MASK;
			padconf = 0x182;
			break;
		case 1:
			wk_mask = OMAP3430_ST_UART2_MASK;
			padconf = 0x17a;
			break;
		case 2:
			wk_mask = OMAP3430_ST_UART3_MASK;
			padconf = 0x19e;
			break;
		}
		uart->wk_mask = wk_mask;
		uart->padconf = padconf;
	} else if (cpu_is_omap24xx()) {
		u32 wk_mask = 0;

		if (cpu_is_omap2430()) {
			uart->wk_en = OMAP2430_PRM_REGADDR(CORE_MOD, PM_WKEN1);
			uart->wk_st = OMAP2430_PRM_REGADDR(CORE_MOD, PM_WKST1);
		} else if (cpu_is_omap2420()) {
			uart->wk_en = OMAP2420_PRM_REGADDR(CORE_MOD, PM_WKEN1);
			uart->wk_st = OMAP2420_PRM_REGADDR(CORE_MOD, PM_WKST1);
		}
		switch (uart->num) {
		case 0:
			wk_mask = OMAP24XX_ST_UART1_MASK;
			break;
		case 1:
			wk_mask = OMAP24XX_ST_UART2_MASK;
			break;
		case 2:
			wk_mask = OMAP24XX_ST_UART3_MASK;
			break;
		}
		uart->wk_mask = wk_mask;

	} else {
		uart->wk_en = 0;
		uart->wk_st = 0;
		uart->wk_mask = 0;
		uart->padconf = 0;
	}

	uart->irqflags |= IRQF_SHARED;
	ret = request_irq(uart->irq, omap_uart_interrupt, IRQF_SHARED,
			  "serial idle", (void *)uart);
	WARN_ON(ret);
}

void omap_uart_enable_irqs(int enable)
{
	int ret;
	struct omap_uart_state *uart;

	list_for_each_entry(uart, &uart_list, node) {
		if (enable)
			ret = request_irq(uart->irq, omap_uart_interrupt,
				IRQF_SHARED, "serial idle", (void *)uart);
		else
			free_irq(uart->irq, (void *)uart);
	}
}

static ssize_t sleep_timeout_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct platform_device *pdev = container_of(dev,
					struct platform_device, dev);
	struct omap_uart_state *uart = container_of(pdev,
					struct omap_uart_state, pdev);

	return sprintf(buf, "%u\n", uart->timeout / HZ);
}

static ssize_t sleep_timeout_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t n)
{
	struct platform_device *pdev = container_of(dev,
					struct platform_device, dev);
	struct omap_uart_state *uart = container_of(pdev,
					struct omap_uart_state, pdev);
	unsigned int value;

	if (sscanf(buf, "%u", &value) != 1) {
		printk(KERN_ERR "sleep_timeout_store: Invalid value\n");
		return -EINVAL;
	}

	uart->timeout = value * HZ;
	if (uart->timeout)
		mod_timer(&uart->timer, jiffies + uart->timeout);
	else
		/* A zero value means disable timeout feature */
		omap_uart_block_sleep(uart);

	return n;
}

DEVICE_ATTR(sleep_timeout, 0644, sleep_timeout_show, sleep_timeout_store);
#define DEV_CREATE_FILE(dev, attr) WARN_ON(device_create_file(dev, attr))
#else
static inline void omap_uart_idle_init(struct omap_uart_state *uart) {}
#define DEV_CREATE_FILE(dev, attr)
#endif /* CONFIG_PM */

#ifndef CONFIG_SERIAL_OMAP
/*
 * Override the default 8250 read handler: mem_serial_in()
 * Empty RX fifo read causes an abort on omap3630 and omap4
 * This function makes sure that an empty rx fifo is not read on these silicons
 * (OMAP1/2/3430 are not affected)
 */
static unsigned int serial_in_override(struct uart_port *up, int offset)
{
	if (UART_RX == offset) {
		unsigned int lsr;
		lsr = serial_read_reg(&omap_uart[up->line], UART_LSR);
		if (!(lsr & UART_LSR_DR))
			return -EPERM;
	}
	return serial_read_reg(&omap_uart[up->line], offset);
}
#endif

static void omap_uart_early_port_init(struct omap_uart_state *uart)
{
	switch (uart->num) {
	case 0:
		uart->irq	= INT_24XX_UART1_IRQ;
		uart->mapbase	= OMAP_UART1_BASE;
		break;
	case 1:
		uart->irq	= INT_24XX_UART2_IRQ;
		uart->mapbase	= OMAP_UART2_BASE;
		break;
	case 2:
		uart->irq	= INT_24XX_UART3_IRQ;
		uart->mapbase	= OMAP_UART3_BASE;
		break;
#ifdef CONFIG_ARCH_OMAP4
	case 3:
		uart->irq	= 70;
		uart->mapbase	= OMAP_UART4_BASE;
		break;
#endif
	}
	uart->regshift = 2;
	uart->irqflags = IRQF_SHARED;
}

void __init omap_serial_early_init(void)
{
	int i;
	char name[16];

	/*
	 * Make sure the serial ports are muxed on at this point.
	 * You have to mux them off in device drivers later on
	 * if not needed.
	 */

	for (i = 0; i < ARRAY_SIZE(omap_uart); i++) {
		struct omap_uart_state *uart = &omap_uart[i];

		uart->num = i;
		omap_uart_early_port_init(uart);

		/*
		 * Module 4KB + L4 interconnect 4KB
		 * Static mapping, never released
		 */
		uart->membase = ioremap(uart->mapbase, SZ_8K);
		if (!uart->membase) {
			printk(KERN_ERR "ioremap failed for uart%i\n", i + 1);
			continue;
		}

		if (!cpu_is_omap44xx())
			sprintf(name, "uart%d_ick", i+1);
		else
			sprintf(name, "uart%d_ck", i+1);
		uart->ick = clk_get(NULL, name);
		if (IS_ERR(uart->ick)) {
			printk(KERN_ERR "Could not get uart%d_ick\n", i+1);
			uart->ick = NULL;
		}

		if (!cpu_is_omap44xx()) {
			sprintf(name, "uart%d_fck", i+1);
			uart->fck = clk_get(NULL, name);
			if (IS_ERR(uart->fck)) {
				printk(KERN_ERR "Could not get uart%d_fck\n",
									 i+1);
				uart->fck = NULL;
			}
		}

		/* FIXME: Remove this once the clkdev is ready */
		if (!cpu_is_omap44xx()) {
			if (!uart->ick || !uart->fck)
				continue;
		}

		if (cpu_is_omap44xx())
			uart->irq += 32;
	}
}

/**
 * omap_serial_init_port() - initialize single serial port
 * @port: serial port number (0-3)
 *
 * This function initialies serial driver for given @port only.
 * Platforms can call this function instead of omap_serial_init()
 * if they don't plan to use all available UARTs as serial ports.
 *
 * Don't mix calls to omap_serial_init_port() and omap_serial_init(),
 * use only one of the two.
 */
void __init omap_serial_init_port(int port)
{
	struct omap_uart_state *uart;
	struct platform_device *pdev;
	struct device *dev;
#ifndef CONFIG_SERIAL_OMAP
	struct plat_serial8250_port *p;
#endif

	BUG_ON(port < 0);
#if 1
	BUG_ON(port >= ARRAY_SIZE(omap_uart));

	uart = &omap_uart[port];
	pdev = &uart->pdev;
	dev = &pdev->dev;

#ifndef CONFIG_SERIAL_OMAP
	p = dev->platform_data;
	p->membase = uart->membase;
	p->private_data = uart;
	if (cpu_is_omap44xx())
		p->irq += 32;
#endif
	omap_uart_enable_clocks(uart);
	omap_uart_reset(uart);
	omap_uart_idle_init(uart);
	list_add_tail(&uart->node, &uart_list);

#ifndef CONFIG_SERIAL_OMAP
	/* omap44xx: Never read empty UART fifo
	 * omap3xxx: Never read empty UART fifo on UARTs
	 * with IP rev >=0x52
	 */
	if (cpu_is_omap44xx()) {
		p->serial_in = serial_in_override;
	} else if ((serial_read_reg(uart, UART_OMAP_MVER) & 0xFF)
			>= UART_OMAP_NO_EMPTY_FIFO_READ_IP_REV) {
		p->serial_in = serial_in_override;
	}
#else
	BUG_ON(port >= num_uarts);

	list_for_each_entry(uart, &uart_list, node)
		if (port == uart->num)
			break;
	{

		struct omap_hwmod *oh = uart->oh;
		struct omap_device *od;
		void *pdata = NULL;
		u32 pdata_size = 0;

		struct plat_serial8250_port ports[2] = {
			{},
			{.flags = 0},
		};
		struct plat_serial8250_port *p = &ports[0];

		name = "serial8250";
		uart->dma_enabled = 0;

		/*
		 * !! 8250 driver does not use standard IORESOURCE* It
		 * has it's own custom pdata that can be taken from
		 * the hwmod resource data.  But, this needs to be
		 * done after the build.
		 *
		 * ?? does it have to be done before the register ??
		 * YES, because platform_device_data_add() copies
		 * pdata, it does not use a pointer.
		 */
		p->flags = UPF_BOOT_AUTOCONF;
		p->iotype = UPIO_MEM;
		p->regshift = 2;
		p->uartclk = OMAP24XX_BASE_BAUD * 16;
		p->irq = oh->mpu_irqs[0].irq;
		p->mapbase = oh->slaves[0]->addr->pa_start;
		p->membase = oh->_rt_va;
		p->irqflags = IRQF_SHARED;
		p->private_data = uart;

		/* omap44xx: Never read empty UART fifo
		 * omap3xxx: Never read empty UART fifo on UARTs
		 * with IP rev >=0x52
		 */
		if (cpu_is_omap44xx()) {
			p->serial_in = serial_in_override;
			p->serial_out = serial_out_override;
		}

		pdata = &ports[0];
		pdata_size = 2 * sizeof(struct plat_serial8250_port);

		if (WARN_ON(!oh))
			return;

		od = omap_device_build(name, uart->num, oh, pdata, pdata_size,
				       omap_uart_latency,
				       ARRAY_SIZE(omap_uart_latency), 0);
		WARN(IS_ERR(od), "Could not build omap_device for %s: %s.\n",
		     name, oh->name);

		uart->irq = oh->mpu_irqs[0].irq;
		uart->regshift = 2;
		uart->mapbase = oh->slaves[0]->addr->pa_start;
		uart->membase = oh->_rt_va;
		uart->pdev = &od->pdev;

		oh->dev_attr = uart;

#ifdef CONFIG_DEBUG_LL
		/*
		 * Because of earlyprintk output, UART did not get idled
		 * on init.  Now that omap_device is ready, ensure full idle
		 * before doing omap_device_enable().
		 */
		omap_hwmod_idle(uart->oh);
#endif
#endif

	if (WARN_ON(platform_device_register(pdev)))
		return;

	if ((cpu_is_omap34xx() && uart->padconf) ||
	    (uart->wk_en && uart->wk_mask)) {
		device_init_wakeup(dev, true);
		DEV_CREATE_FILE(dev, &dev_attr_sleep_timeout);
	}
}

/**
 * omap_serial_init() - intialize all supported serial ports
 *
 * Initializes all available UARTs as serial ports. Platforms
 * can call this function when they want to have default behaviour
 * for serial ports (e.g initialize them all as serial ports).
 */
void __init omap_serial_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(omap_uart); i++)
		omap_serial_init_port(i);
}
