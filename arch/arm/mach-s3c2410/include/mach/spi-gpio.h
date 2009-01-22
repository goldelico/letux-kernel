/* arch/arm/mach-s3c2410/include/mach/spi-gpio.h
 *
 * Copyright (c) 2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C2410 - SPI Controller platfrom_device info
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_SPIGPIO_H
#define __ASM_ARCH_SPIGPIO_H __FILE__

struct s3c2410_spigpio_info {
	unsigned long		 pin_clk;
	unsigned long		 pin_mosi;
	unsigned long		 pin_miso;

	int			 num_chipselect;
	int			 bus_num;
	int			 num_chipselect;

	/*
 	 * FIXME: board_size and board_info DO NOT belong here.
 	 * These were already removed upstream... but we still rely on them
 	 * so leave for now and revisit this.
 	 */
	unsigned long            board_size;
	struct spi_board_info   *board_info;

	void (*chip_select)(struct s3c2410_spigpio_info *spi, int csid, int cs);
};


#endif /* __ASM_ARCH_SPIGPIO_H */
