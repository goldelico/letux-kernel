/*
 *  Copyright (C) 2009, Lars-Peter Clausen <lars@metafoo.de>
 *  Copyright (C) 2015, Paul Boddie <paul@boddie.org.uk>
 *  JZ4730-specific GPIO pin definitions
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef _JZ_JZ4730_GPIO_H
#define _JZ_JZ4730_GPIO_H

#include <linux/types.h>

enum jz_gpio_function {
    JZ_GPIO_FUNC_NONE,
    JZ_GPIO_FUNC1,
    JZ_GPIO_FUNC2,
    JZ_GPIO_FUNC3,
};

/*
 Usually a driver for a SoC component has to request several gpio pins and
 configure them as funcion pins.
 jz_gpio_bulk_request can be used to ease this process.
 Usually one would do something like:

 static const struct jz_gpio_bulk_request i2c_pins[] = {
	JZ_GPIO_BULK_PIN(I2C_SDA),
	JZ_GPIO_BULK_PIN(I2C_SCK),
 };

 inside the probe function:

    ret = jz_gpio_bulk_request(i2c_pins, ARRAY_SIZE(i2c_pins));
    if (ret) {
	...

 inside the remove function:

    jz_gpio_bulk_free(i2c_pins, ARRAY_SIZE(i2c_pins));

*/

struct jz_gpio_bulk_request {
	int gpio;
	const char *name;
	enum jz_gpio_function function;
};

#define JZ_GPIO_BULK_PIN(pin) { \
    .gpio = JZ_GPIO_ ## pin, \
    .name = #pin, \
    .function = JZ_GPIO_FUNC_ ## pin \
}

int jz_gpio_bulk_request(const struct jz_gpio_bulk_request *request, size_t num);
void jz_gpio_bulk_free(const struct jz_gpio_bulk_request *request, size_t num);
void jz_gpio_bulk_suspend(const struct jz_gpio_bulk_request *request, size_t num);
void jz_gpio_bulk_resume(const struct jz_gpio_bulk_request *request, size_t num);
void jz_gpio_enable_pullup(unsigned gpio);
void jz_gpio_disable_pullup(unsigned gpio);
int jz_gpio_set_function(int gpio, enum jz_gpio_function function);

int jz_gpio_port_direction_input(int port, uint32_t mask);
int jz_gpio_port_direction_output(int port, uint32_t mask);
void jz_gpio_port_set_value(int port, uint32_t value, uint32_t mask);
uint32_t jz_gpio_port_get_value(int port, uint32_t mask);

#include <asm/mach-generic/gpio.h>

#define JZ_GPIO_PORTA(x) ((x) + 32 * 0)
#define JZ_GPIO_PORTB(x) ((x) + 32 * 1)
#define JZ_GPIO_PORTC(x) ((x) + 32 * 2)
#define JZ_GPIO_PORTD(x) ((x) + 32 * 3)

/* Port A function pins */
// DMA pins disappear in later datasheets...
#define JZ_GPIO_DMA_DREQ0		JZ_GPIO_PORTA(12)
#define JZ_GPIO_DMA_DACK0		JZ_GPIO_PORTA(13)
// RD4730_PMP-HW 2.2_EN.pdf claims GPIO 125...
#define JZ_GPIO_CHARGING		JZ_GPIO_PORTA(17)
#define JZ_GPIO_DMA_AEN			JZ_GPIO_PORTA(26)
#define JZ_GPIO_DMA_EOP			JZ_GPIO_PORTA(27)
#define JZ_GPIO_UHC_CLK			JZ_GPIO_PORTA(28)
#define JZ_GPIO_UHC_PPWR0		JZ_GPIO_PORTA(29)

#define JZ_GPIO_FUNC_DMA_DREQ0		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_DMA_DACK0		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_DMA_CHARGING	JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_DMA_AEN		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_DMA_EOP		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_UHC_CLK		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_UHC_PPWR0		JZ_GPIO_FUNC1

/* Port B function pins */
#define JZ_GPIO_MSC_CMD			JZ_GPIO_PORTB(2)
#define JZ_GPIO_MSC_CLK			JZ_GPIO_PORTB(3)
#define JZ_GPIO_MSC_DATA0		JZ_GPIO_PORTB(4)
#define JZ_GPIO_MSC_DATA1		JZ_GPIO_PORTB(5)
#define JZ_GPIO_MSC_DATA2		JZ_GPIO_PORTB(6)
#define JZ_GPIO_MSC_DATA3		JZ_GPIO_PORTB(7)
#define JZ_GPIO_LCD_DATA0		JZ_GPIO_PORTB(8)
#define JZ_GPIO_LCD_DATA1		JZ_GPIO_PORTB(9)
#define JZ_GPIO_LCD_DATA2		JZ_GPIO_PORTB(10)
#define JZ_GPIO_LCD_DATA3		JZ_GPIO_PORTB(11)
#define JZ_GPIO_LCD_DATA4		JZ_GPIO_PORTB(12)
#define JZ_GPIO_LCD_DATA5		JZ_GPIO_PORTB(13)
#define JZ_GPIO_LCD_DATA6		JZ_GPIO_PORTB(14)
#define JZ_GPIO_LCD_DATA7		JZ_GPIO_PORTB(15)
#define JZ_GPIO_LCD_DATA8		JZ_GPIO_PORTB(16)
#define JZ_GPIO_LCD_DATA9		JZ_GPIO_PORTB(17)
#define JZ_GPIO_LCD_DATA10		JZ_GPIO_PORTB(18)
#define JZ_GPIO_LCD_DATA11		JZ_GPIO_PORTB(19)
#define JZ_GPIO_LCD_DATA12		JZ_GPIO_PORTB(20)
#define JZ_GPIO_LCD_DATA13		JZ_GPIO_PORTB(21)
#define JZ_GPIO_LCD_DATA14		JZ_GPIO_PORTB(22)
#define JZ_GPIO_LCD_DATA15		JZ_GPIO_PORTB(23)
#define JZ_GPIO_LCD_VSYNC		JZ_GPIO_PORTB(24)
#define JZ_GPIO_LCD_HSYNC		JZ_GPIO_PORTB(25)
#define JZ_GPIO_LCD_PCLK		JZ_GPIO_PORTB(26)
#define JZ_GPIO_LCD_DE			JZ_GPIO_PORTB(27)
#define JZ_GPIO_LCD_SPL			JZ_GPIO_PORTB(28)
#define JZ_GPIO_LCD_CLS			JZ_GPIO_PORTB(29)
#define JZ_GPIO_LCD_PS			JZ_GPIO_PORTB(30)
#define JZ_GPIO_LCD_REV			JZ_GPIO_PORTB(31)

#define JZ_GPIO_FUNC_MSC_CMD		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_MSC_CLK		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_MSC_DATA		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_MSC_DATA0		JZ_GPIO_FUNC_MSC_DATA
#define JZ_GPIO_FUNC_MSC_DATA1		JZ_GPIO_FUNC_MSC_DATA
#define JZ_GPIO_FUNC_MSC_DATA2		JZ_GPIO_FUNC_MSC_DATA
#define JZ_GPIO_FUNC_MSC_DATA3		JZ_GPIO_FUNC_MSC_DATA
#define JZ_GPIO_FUNC_LCD_DATA0		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_DATA1		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_DATA2		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_DATA3		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_DATA4		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_DATA5		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_DATA6		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_DATA7		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_DATA8		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_DATA9		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_DATA10		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_DATA11		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_DATA12		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_DATA13		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_DATA14		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_DATA15		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_VSYNC		JZ_GPIO_FUNC2
#define JZ_GPIO_FUNC_LCD_HSYNC		JZ_GPIO_FUNC2
#define JZ_GPIO_FUNC_LCD_PCLK		JZ_GPIO_FUNC2
#define JZ_GPIO_FUNC_LCD_DE		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_SPL		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_CLS		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_PS		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_LCD_REV		JZ_GPIO_FUNC1

/* Port C function pins */
#define JZ_GPIO_AIC_SYSCLK		JZ_GPIO_PORTC(4)
#define JZ_GPIO_AIC_SDATA_OUT		JZ_GPIO_PORTC(6)
#define JZ_GPIO_AIC_SDATA_IN		JZ_GPIO_PORTC(7)
#define JZ_GPIO_SPI_CLK			JZ_GPIO_PORTC(8)
#define JZ_GPIO_SPI_CE1			JZ_GPIO_PORTC(9)
#define JZ_GPIO_SPI_DT			JZ_GPIO_PORTC(10)
#define JZ_GPIO_SPI_DR			JZ_GPIO_PORTC(11)
#define JZ_GPIO_SPI_CE2			JZ_GPIO_PORTC(12)
#define JZ_GPIO_AIC_BITCLK		JZ_GPIO_PORTC(13)
#define JZ_GPIO_AIC_SYNC		JZ_GPIO_PORTC(14)
#define JZ_GPIO_PWM0			JZ_GPIO_PORTC(30)
#define JZ_GPIO_PWM1			JZ_GPIO_PORTC(31)

#define JZ_GPIO_FUNC_AIC_SYSCLK		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_AIC_SDATA_OUT	JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_AIC_SDATA_IN	JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_SPI_CLK		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_SPI_CE1		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_SPI_DT		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_SPI_DR		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_SPI_CE2		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_AIC_BITCLK		JZ_GPIO_FUNC2
#define JZ_GPIO_FUNC_AIC_SYNC		JZ_GPIO_FUNC2
#define JZ_GPIO_FUNC_PWM		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_PWM0		JZ_GPIO_FUNC_PWM
#define JZ_GPIO_FUNC_PWM1		JZ_GPIO_FUNC_PWM

/* Port D function pins */

#define JZ_GPIO_UART0_RXD		JZ_GPIO_PORTD(30)
#define JZ_GPIO_UART0_TXD		JZ_GPIO_PORTD(31)

#define JZ_GPIO_FUNC_UART0_RXD		JZ_GPIO_FUNC1
#define JZ_GPIO_FUNC_UART0_TXD		JZ_GPIO_FUNC1

#endif
