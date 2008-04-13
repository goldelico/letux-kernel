/* linux/arch/arm/plat-s3c24xx/gpio.c
 *
 * Copyright (c) 2004-2005 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C24XX GPIO support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-gpioj.h>

void s3c2410_gpio_cfgpin(unsigned int pin, unsigned int function)
{
	void __iomem *base = S3C24XX_GPIO_BASE(pin);
	unsigned long mask;
	unsigned long con;
	unsigned long flags;

	if (pin < S3C2410_GPIO_BANKB) {
		mask = 1 << S3C2410_GPIO_OFFSET(pin);
	} else {
		mask = 3 << S3C2410_GPIO_OFFSET(pin)*2;
	}

	switch (function) {
	case S3C2410_GPIO_LEAVE:
		mask = 0;
		function = 0;
		break;

	case S3C2410_GPIO_INPUT:
	case S3C2410_GPIO_OUTPUT:
	case S3C2410_GPIO_SFN2:
	case S3C2410_GPIO_SFN3:
		if (pin < S3C2410_GPIO_BANKB) {
			function -= 1;
			function &= 1;
			function <<= S3C2410_GPIO_OFFSET(pin);
		} else {
			function &= 3;
			function <<= S3C2410_GPIO_OFFSET(pin)*2;
		}
	}

	/* modify the specified register wwith IRQs off */

	local_irq_save(flags);

	con  = __raw_readl(base + 0x00);
	con &= ~mask;
	con |= function;

	__raw_writel(con, base + 0x00);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(s3c2410_gpio_cfgpin);

unsigned int s3c2410_gpio_getcfg(unsigned int pin)
{
	void __iomem *base = S3C24XX_GPIO_BASE(pin);
	unsigned long val = __raw_readl(base);

	if (pin < S3C2410_GPIO_BANKB) {
		val >>= S3C2410_GPIO_OFFSET(pin);
		val &= 1;
		val += 1;
	} else {
		val >>= S3C2410_GPIO_OFFSET(pin)*2;
		val &= 3;
	}

	return val | S3C2410_GPIO_INPUT;
}

EXPORT_SYMBOL(s3c2410_gpio_getcfg);

void s3c2410_gpio_pullup(unsigned int pin, unsigned int to)
{
	void __iomem *base = S3C24XX_GPIO_BASE(pin);
	unsigned long offs = S3C2410_GPIO_OFFSET(pin);
	unsigned long flags;
	unsigned long up;

	if (pin < S3C2410_GPIO_BANKB)
		return;

	local_irq_save(flags);

	up = __raw_readl(base + 0x08);
	up &= ~(1L << offs);
	up |= to << offs;
	__raw_writel(up, base + 0x08);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(s3c2410_gpio_pullup);

void s3c2410_gpio_setpin(unsigned int pin, unsigned int to)
{
	void __iomem *base = S3C24XX_GPIO_BASE(pin);
	unsigned long offs = S3C2410_GPIO_OFFSET(pin);
	unsigned long flags;
	unsigned long dat;

	local_irq_save(flags);

	dat = __raw_readl(base + 0x04);
	dat &= ~(1 << offs);
	dat |= to << offs;
	__raw_writel(dat, base + 0x04);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(s3c2410_gpio_setpin);

unsigned int s3c2410_gpio_getpin(unsigned int pin)
{
	void __iomem *base = S3C24XX_GPIO_BASE(pin);
	unsigned long offs = S3C2410_GPIO_OFFSET(pin);

	return __raw_readl(base + 0x04) & (1<< offs);
}

EXPORT_SYMBOL(s3c2410_gpio_getpin);

unsigned int s3c2410_modify_misccr(unsigned int clear, unsigned int change)
{
	unsigned long flags;
	unsigned long misccr;

	local_irq_save(flags);
	misccr = __raw_readl(S3C24XX_MISCCR);
	misccr &= ~clear;
	misccr ^= change;
	__raw_writel(misccr, S3C24XX_MISCCR);
	local_irq_restore(flags);

	return misccr;
}

EXPORT_SYMBOL(s3c2410_modify_misccr);

int s3c2410_gpio_getirq(unsigned int pin)
{
	if (pin < S3C2410_GPF0 || pin > S3C2410_GPG15)
		return -1;	/* not valid interrupts */

	if (pin < S3C2410_GPG0 && pin > S3C2410_GPF7)
		return -1;	/* not valid pin */

	if (pin < S3C2410_GPF4)
		return (pin - S3C2410_GPF0) + IRQ_EINT0;

	if (pin < S3C2410_GPG0)
		return (pin - S3C2410_GPF4) + IRQ_EINT4;

	return (pin - S3C2410_GPG0) + IRQ_EINT8;
}

EXPORT_SYMBOL(s3c2410_gpio_getirq);

int s3c2410_irq_to_gpio(unsigned int irq)
{
	/* not valid interrupts */
	if (irq > 15 + IRQ_EINT8)
		return -1;

	if (irq < IRQ_EINT4)
		return (irq - IRQ_EINT0) + S3C2410_GPF0;

	if (irq < IRQ_EINT8)
		return (irq - IRQ_EINT4) + S3C2410_GPF4;

	return (irq - IRQ_EINT8) + S3C2410_GPG0;
}

EXPORT_SYMBOL(s3c2410_irq_to_gpio);

static void pretty_dump(u32 cfg, u32 state, u32 pull,
			const char ** function_names_2,
			const char ** function_names_3,
			const char * prefix,
			int count)
{
	int n;
	const char *tag_type = NULL,
		   *tag_state = NULL,
		   *tag_pulldown = NULL,
		   * level0 = "0",
		   * level1 = "1";

	for (n = 0; n < count; n++) {
		switch ((cfg >> (2 * n)) & 3) {
		case 0:
			tag_type = "input      ";
			break;
		case 1:
			tag_type = "OUTPUT     ";
			break;
		case 2:
			if (function_names_2) {
				if (function_names_2[n])
					tag_type = function_names_2[n];
				else
					tag_type = "*** ILLEGAL CFG (2) *** ";
			} else
				tag_type = "(function) ";
			break;
		default:
			if (function_names_3) {
				if (function_names_3[n])
					tag_type = function_names_3[n];
				else
					tag_type = "*** ILLEGAL CFG (3) *** ";
			} else
				tag_type = "(function) ";
			break;
		}
		if ((state >> n) & 1)
			tag_state = level1;
		else
			tag_state = level0;

		if (((pull >> n) & 1))
			tag_pulldown = "";
		else
			tag_pulldown = "(pulldown)";

		printk(KERN_INFO"%s%02d: %s %s %s\n", prefix, n, tag_type,
						      tag_state, tag_pulldown);
	}
	printk(KERN_INFO"\n");
}

static void pretty_dump_a(u32 cfg, u32 state,
			  const char ** function_names,
			  const char * prefix,
			  int count)
{
	int n;
	const char *tag_type = NULL,
		   *tag_state = NULL,
		   * level0 = "0",
		   * level1 = "1";

	for (n = 0; n < count; n++) {
		switch ((cfg >> n) & 1) {
		case 0:
			tag_type = "OUTPUT     ";
			break;
		default:
			if (function_names) {
				if (function_names[n])
					tag_type = function_names[n];
				else
					tag_type = "*** ILLEGAL CFG *** ";
			} else
				tag_type = "(function) ";
			break;
		}
		if ((state >> n) & 1)
			tag_state = level1;
		else
			tag_state = level0;

		printk(KERN_INFO"%s%02d: %s %s\n", prefix, n, tag_type,
						   tag_state);
	}
	printk(KERN_INFO"\n");
}

static const char * funcs_a[] = {
	"ADDR0      ",
	"ADDR16     ",
	"ADDR17     ",
	"ADDR18     ",
	"ADDR19     ",
	"ADDR20     ",
	"ADDR21     ",
	"ADDR22     ",
	"ADDR23     ",
	"ADDR24     ",
	"ADDR25     ",
	"ADDR26     ",
	"nGCS[1]    ",
	"nGCS[2]    ",
	"nGCS[3]    ",
	"nGCS[4]    ",
	"nGCS[5]    ",
	"CLE        ",
	"ALE        ",
	"nFWE       ",
	"nFRE       ",
	"nRSTOUT    ",
	"nFCE       ",
	NULL,
	NULL
};


static const char * funcs_b2[] = {
	"TOUT0      ",
	"TOUT1      ",
	"TOUT2      ",
	"TOUT3      ",
	"TCLK[0]    ",
	"nXBACK     ",
	"nXBREQ     ",
	"nXDACK1    ",
	"nXDREQ1    ",
	"nXDACK0    ",
	"nXDREQ0    ",
};
static const char * funcs_b3[] = {
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
};

static const char * funcs_c2[] = {
	"LEND       ",
	"VCLK       ",
	"VLINE      ",
	"VFRAME     ",
	"VM         ",
	"LCD_LPCOE  ",
	"LCD_LPCREV ",
	"LCD_LPCREVB",
	"VD[0]      ",
	"VD[1]      ",
	"VD[2]      ",
	"VD[3]      ",
	"VD[4]      ",
	"VD[5]      ",
	"VD[6]      ",
	"VD[7]      ",
};
static const char * funcs_c3[] = {
	NULL,
	NULL,
	NULL,
	NULL,
	"I2SSDI     ",
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
};

static const char * funcs_d2[] = {
	"VD[8]      ",
	"VD[9]      ",
	"VD[10]     ",
	"VD[11]     ",
	"VD[12]     ",
	"VD[13]     ",
	"VD[14]     ",
	"VD[15]     ",
	"VD[16]     ",
	"VD[17]     ",
	"VD[18]     ",
	"VD[19]     ",
	"VD[20]     ",
	"VD[21]     ",
	"VD[22]     ",
	"VD[23]     ",
};
static const char * funcs_d3[] = {
	"nSPICS1    ",
	"SPICLK1    ",
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	"SPIMISO1   ",
	"SPIMOSI1   ",
	"SPICLK1    ",
	NULL,
	NULL,
	NULL,
	"nSS1       ",
	"nSS0       ",
};

static const char * funcs_e2[] = {
	"I2SLRCK    ",
	"I2SSCLK    ",
	"CDCLK      ",
	"I2SDI      ",
	"I2SDO      ",
	"SDCLK      ",
	"SDCMD      ",
	"SDDAT0     ",
	"SDDAT1     ",
	"SDDAT2     ",
	"SDDAT3     ",
	"SPIMISO0   ",
	"SPIMOSI0   ",
	"SPICLK0    ",
	"IICSCL     ",
	"IICSDA     ",
};
static const char * funcs_e3[] = {
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
};

static const char * funcs_f2[] = {
	"EINT[0]    ",
	"EINT[1]    ",
	"EINT[2]    ",
	"EINT[3]    ",
	"EINT[4]    ",
	"EINT[5]    ",
	"EINT[6]    ",
	"EINT[7]    ",
};
static const char * funcs_f3[] = {
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
};


static const char * funcs_g2[] = {
	"EINT[8]    ",
	"EINT[9]    ",
	"EINT[10]   ",
	"EINT[11]   ",
	"EINT[12]   ",
	"EINT[13]   ",
	"EINT[14]   ",
	"EINT[15]   ",
	"EINT[16]   ",
	"EINT[17]   ",
	"EINT[18]   ",
	"EINT[19]   ",
	"EINT[20]   ",
	"EINT[21]   ",
	"EINT[22]   ",
	"EINT[23]   ",
};
static const char * funcs_g3[] = {
	NULL,
	NULL,
	"nSS0       ",
	"nSS1       ",
	"LCD_PWRDN  ",
	"SPIMISO1   ",
	"SPIMOSI1   ",
	"SPICLK1    ",
	NULL,
	"nRTS1      ",
	"nCTS1      ",
	"TCLK[1]    ",
	"nSPICS0    ",
	NULL,
	NULL,
	NULL,
};

static const char * funcs_h2[] = {
	"nCTS0      ",
	"nRTS0      ",
	"TXD[0]     ",
	"RXD[0]     ",
	"TXD[1]     ",
	"RXD[1]     ",
	"TXD[2]     ",
	"RXD[2]     ",
	"UEXTCLK    ",
	"CLKOUT0    ",
	"CLKOUT1    ",
};
static const char * funcs_h3[] = {
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	"nRTS1      ",
	"nCTS1      ",
	NULL,
	"nSPICS0    ",
	NULL,
};

static const char * funcs_j2[] = {
	"CAMDATA[0] ",
	"CAMDATA[1] ",
	"CAMDATA[2] ",
	"CAMDATA[3] ",
	"CAMDATA[4] ",
	"CAMDATA[5] ",
	"CAMDATA[6] ",
	"CAMDATA[7] ",
	"CAMPCLK    ",
	"CAMVSYNC   ",
	"CAMHREF    ",
	"CAMCLKOUT  ",
	"CAMRESET   ",
};
static const char * funcs_j3[] = {
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
};

/* used to dump GPIO states at suspend */
void s3c24xx_dump_gpio_states(void)
{
	pretty_dump_a(__raw_readl(S3C2410_GPACON),
		      __raw_readl(S3C2410_GPADAT),
		      funcs_a, "GPA", 25);
	pretty_dump(__raw_readl(S3C2410_GPBCON),
		    __raw_readl(S3C2410_GPBDAT),
		    __raw_readl(S3C2410_GPBUP),
		    funcs_b2, funcs_b3, "GPB", 11);
	pretty_dump(__raw_readl(S3C2410_GPCCON),
		    __raw_readl(S3C2410_GPCDAT),
		    __raw_readl(S3C2410_GPCUP),
		    funcs_c2, funcs_c3, "GPC", 16);
	pretty_dump(__raw_readl(S3C2410_GPDCON),
		    __raw_readl(S3C2410_GPDDAT),
		    __raw_readl(S3C2410_GPDUP),
		    funcs_d2, funcs_d3, "GPD", 16);
	pretty_dump(__raw_readl(S3C2410_GPECON),
		    __raw_readl(S3C2410_GPEDAT),
		    __raw_readl(S3C2410_GPEUP),
		    funcs_e2, funcs_e3, "GPE", 16);
	pretty_dump(__raw_readl(S3C2410_GPFCON),
		    __raw_readl(S3C2410_GPFDAT),
		    __raw_readl(S3C2410_GPFUP),
		    funcs_f2, funcs_f3, "GPF", 8);
	pretty_dump(__raw_readl(S3C2410_GPGCON),
		    __raw_readl(S3C2410_GPGDAT),
		    __raw_readl(S3C2410_GPGUP),
		    funcs_g2, funcs_g3, "GPG", 16);
	pretty_dump(__raw_readl(S3C2410_GPHCON),
		    __raw_readl(S3C2410_GPHDAT),
		    __raw_readl(S3C2410_GPHUP),
		    funcs_h2, funcs_h3, "GPH", 11);
	pretty_dump(__raw_readl(S3C2440_GPJCON),
		    __raw_readl(S3C2440_GPJDAT),
		    __raw_readl(S3C2440_GPJUP),
		    funcs_j2, funcs_j3, "GPJ", 13);

}
EXPORT_SYMBOL(s3c24xx_dump_gpio_states);
