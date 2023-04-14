#include <uart.h>
#include <cpm.h>
#include <gpio.h>

static void serial_putc (int num, const char c)
{
	if (c == '\n')
		serial_putc (num, '\r');

	/* Wait for fifo to shift out some bytes */
	while ( !((uart_readl(OFF_LSR(num)) & (UART_LSR_TDRQ | UART_LSR_TEMT)) == 0x60) );

	uart_writel((unsigned char)c, OFF_TDR(num));
}

static void serial_setbrg(int num)
{
	unsigned int baud_div, tmp;

	uart_writel(16, OFF_UMR(num));
	uart_writel(0, OFF_UACR(num));
	baud_div = 13;

	tmp = uart_readl(OFF_LCR(num));
	tmp |= UARTLCR_DLAB;
	uart_writel(tmp, OFF_LCR(num));

	uart_writel((baud_div >> 8) & 0xff, OFF_DLHR(num));
	uart_writel(baud_div & 0xff, OFF_DLLR(num));

	tmp &= ~UARTLCR_DLAB;
	uart_writel(tmp, OFF_LCR(num));
}

void serial_init(int num)
{
	switch(num){
		case 0:
			__gpio_as_uart0();
			__cpm_start_uart0();
			break;
		case 2:
			__gpio_as_uart2();
			__cpm_start_uart2();
			break;
		case 1:
		default:
			__gpio_as_uart1();
			__cpm_start_uart1();
			break;
	}

	uart_writel(0, OFF_SPR(num));
	uart_writel(0, OFF_MCR(num));

	/* Disable port interrupts while changing hardware */
	uart_writel(0, OFF_IER(num));

	/* Disable UART unit function */
	uart_writel(~UARTFCR_UUE, OFF_FCR(num));

	/* Set both receiver and transmitter in UART mode (not SIR) */
	uart_writel(~(SIRCR_RSIRE | SIRCR_TSIRE), OFF_SIRCR(num));

	/* Set databits, stopbits and parity. (8-bit data, 1 stopbit, no parity) */
	uart_writel(UARTLCR_WLEN_8 & ~(UARTLCR_STOP2), OFF_LCR(num));

	serial_setbrg(num);

	/* Enable UART unit, enable and clear FIFO */
	uart_writel(UARTFCR_UUE | UARTFCR_TFLS | UARTFCR_RFLS | UARTFCR_FE, OFF_FCR(num));

}

void serial_puts (int num,  const char *s)
{
	while (*s) {
		serial_putc (num, *s++);
	}
}

void serial_put_hex(int num, unsigned int  d)
{
	char c[12];
	unsigned char i;
	for(i = 0; i < 8; i++)
	{
		c[i] = (d >> ((7 - i) * 4)) & 0xf;
		if(c[i] < 10)
			c[i] += 0x30;
		else
			c[i] += (0x41 - 10);
	}
	c[8] = '\n';
	c[9] = 0;
	serial_puts(num, c);
}
