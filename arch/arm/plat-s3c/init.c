/* linux/arch/arm/plat-s3c/init.c
 *
 * Copyright (c) 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * S3C series CPU initialisation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/cpu.h>
#include <plat/devs.h>
#include <plat/clock.h>

#include <plat/regs-serial.h>

static struct cpu_table *cpu;

static void __init set_system_rev(unsigned int idcode)
{
	/*
	 * system_rev encoding is as follows
	 * system_rev & 0xff000000 -> S3C Class (24xx/64xx)
	 * system_rev & 0xfff00000 -> S3C Sub Class (241x/244x)
	 * system_rev & 0xffff0000 -> S3C Type (2410/2440/6400/6410)
	 *
	 * Remaining[15:0] are preserved from the value set by ATAG
	 *
	 * Exception:
	 *  Store Revision A to 1 such as
	 *  s3c2410A to s3c2411
	 *  s3c2440A to s3c2441
	 */

	system_rev &= 0xffff;
	system_rev |= (idcode & 0x0ffff000) << 4;

	if (idcode == 0x32410002 || idcode == 0x32440001)
		system_rev |= (0x1 << 16);
	if (idcode == 0x32440aaa	/* s3c2442 */
	    || idcode == 0x32440aab)	/* s3c2442b */
		system_rev |= (0x2 << 16);
	if (idcode == 0x0)		/* s3c2400 */
		system_rev |= (0x2400 << 16);
}

static struct cpu_table * __init s3c_lookup_cpu(unsigned long idcode,
						struct cpu_table *tab,
						unsigned int count)
{
	for (; count != 0; count--, tab++) {
		if ((idcode & tab->idmask) == tab->idcode)
			return tab;
	}

	return NULL;
}

void __init s3c_init_cpu(unsigned long idcode,
			 struct cpu_table *cputab, unsigned int cputab_size)
{
	cpu = s3c_lookup_cpu(idcode, cputab, cputab_size);

	if (cpu == NULL) {
		printk(KERN_ERR "Unknown CPU type 0x%08lx\n", idcode);
		panic("Unknown S3C24XX CPU");
	}

	set_system_rev(idcode);

	printk("CPU %s (id 0x%08lx)\n", cpu->name, idcode);

	if (cpu->map_io == NULL || cpu->init == NULL) {
		printk(KERN_ERR "CPU %s support not enabled\n", cpu->name);
		panic("Unsupported Samsung CPU");
	}

	cpu->map_io();
}

/* s3c24xx_init_clocks
 *
 * Initialise the clock subsystem and associated information from the
 * given master crystal value.
 *
 * xtal  = 0 -> use default PLL crystal value (normally 12MHz)
 *      != 0 -> PLL crystal value in Hz
*/

void __init s3c24xx_init_clocks(int xtal)
{
	if (xtal == 0)
		xtal = 12*1000*1000;

	if (cpu == NULL)
		panic("s3c24xx_init_clocks: no cpu setup?\n");

	if (cpu->init_clocks == NULL)
		panic("s3c24xx_init_clocks: cpu has no clock init\n");
	else
		(cpu->init_clocks)(xtal);
}

/* uart management */

static int nr_uarts __initdata = 0;

static struct s3c2410_uartcfg uart_cfgs[CONFIG_SERIAL_SAMSUNG_UARTS];

/* s3c24xx_init_uartdevs
 *
 * copy the specified platform data and configuration into our central
 * set of devices, before the data is thrown away after the init process.
 *
 * This also fills in the array passed to the serial driver for the
 * early initialisation of the console.
*/

void __init s3c24xx_init_uartdevs(char *name,
				  struct s3c24xx_uart_resources *res,
				  struct s3c2410_uartcfg *cfg, int no)
{
	struct platform_device *platdev;
	struct s3c2410_uartcfg *cfgptr = uart_cfgs;
	struct s3c24xx_uart_resources *resp;
	int uart;

	memcpy(cfgptr, cfg, sizeof(struct s3c2410_uartcfg) * no);

	for (uart = 0; uart < no; uart++, cfg++, cfgptr++) {
		platdev = s3c24xx_uart_src[cfgptr->hwport];

		resp = res + cfgptr->hwport;

		s3c24xx_uart_devs[uart] = platdev;

		platdev->name = name;
		platdev->resource = resp->resources;
		platdev->num_resources = resp->nr_resources;

		platdev->dev.platform_data = cfgptr;
	}

	nr_uarts = no;
}

void __init s3c24xx_init_uarts(struct s3c2410_uartcfg *cfg, int no)
{
	if (cpu == NULL)
		return;

	if (cpu->init_uarts == NULL) {
		printk(KERN_ERR "s3c24xx_init_uarts: cpu has no uart init\n");
	} else
		(cpu->init_uarts)(cfg, no);
}

static int __init s3c_arch_init(void)
{
	int ret;

	// do the correct init for cpu

	if (cpu == NULL)
		panic("s3c_arch_init: NULL cpu\n");

	ret = (cpu->init)();
	if (ret != 0)
		return ret;

	ret = platform_add_devices(s3c24xx_uart_devs, nr_uarts);
	return ret;
}

arch_initcall(s3c_arch_init);
