/* linux/drivers/spi/spi_s3c64xx_gpio.c
 *
 * Copyright (c) 2009 Openmoko Inc.
 * Author: Matt Hsu <matt_hsu@openmoko.org>
 *
 * S3C64XX GPIO-SPI driver.
 * This driver is based on spi_s3c24xx_gpio.c
 *
 * Copyright (c) 2006 Ben Dooks
 * Copyright (c) 2006 Simtec Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <plat/gpio-cfg.h>
#include <mach/spi-gpio.h>

struct s3c64xx_spigpio {
	struct spi_bitbang 		bitbang;
	struct s3c64xx_spigpio_info 	*info;
	struct platform_device 		*dev;
};

static inline struct s3c64xx_spigpio *spidev_to_sg(struct spi_device *spi)
{
	return spi->controller_data;
}

static inline void setsck(struct spi_device *dev, int on)
{
	struct s3c64xx_spigpio *sg = spidev_to_sg(dev);
	gpio_direction_output(sg->info->pin_clk, on ? 1 : 0);
}

static inline void setmosi(struct spi_device *dev, int on)
{
	struct s3c64xx_spigpio *sg = spidev_to_sg(dev);
	gpio_direction_output(sg->info->pin_mosi, on ? 1 : 0);
}

static inline u32 getmiso(struct spi_device *dev)
{
	struct s3c64xx_spigpio *sg = spidev_to_sg(dev);
	return gpio_direction_input(sg->info->pin_miso) ? 1 : 0;
}

#define spidelay(x) ndelay(x)

#define	EXPAND_BITBANG_TXRX
#include <linux/spi/spi_bitbang.h>

static u32 s3c64xx_spigpio_txrx_mode0(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 0, word, bits);
}

static u32 s3c64xx_spigpio_txrx_mode1(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 0, word, bits);
}

static u32 s3c64xx_spigpio_txrx_mode2(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 1, word, bits);
}

static u32 s3c64xx_spigpio_txrx_mode3(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 1, word, bits);
}
static void s3c64xx_spigpio_chipselect(struct spi_device *dev, int value)
{
	struct s3c64xx_spigpio *sg = spidev_to_sg(dev);

	if (sg->info && sg->info->chip_select)
		(sg->info->chip_select)(sg->info, dev->chip_select, value);
}

static int s3c64xx_spigpio_probe(struct platform_device *dev)
{
	struct s3c64xx_spigpio_info *info;
	struct spi_master	*master;
	struct s3c64xx_spigpio 	*spi;

	int ret;
	int i;

	master = spi_alloc_master(&dev->dev, sizeof(struct s3c64xx_spigpio));
	if (master == NULL) {
		dev_err(&dev->dev, "failed to allocate spi master\n");
		ret = -ENOMEM;
		goto err;
	}

	spi = spi_master_get_devdata(master);

	/* copy in the platform data */
	info = spi->info = dev->dev.platform_data;

	master->num_chipselect = info->num_chipselect;

	/* setup spi bitbang adaptor */
	spi->bitbang.master = spi_master_get(master);
	spi->bitbang.master->bus_num = info->bus_num;

	spi->bitbang.chipselect = s3c64xx_spigpio_chipselect;

	spi->bitbang.txrx_word[SPI_MODE_0] = s3c64xx_spigpio_txrx_mode0;
	spi->bitbang.txrx_word[SPI_MODE_1] = s3c64xx_spigpio_txrx_mode1;
	spi->bitbang.txrx_word[SPI_MODE_2] = s3c64xx_spigpio_txrx_mode2;
	spi->bitbang.txrx_word[SPI_MODE_3] = s3c64xx_spigpio_txrx_mode3;

	/* set state of spi pins. */
	gpio_direction_output(info->pin_clk, 0);
	s3c_gpio_cfgpin(info->pin_clk, S3C_GPIO_OUTPUT);

	ret = spi_bitbang_start(&spi->bitbang);
	if (ret)
		goto err_no_bitbang;

	/* register the chips to go with the board */
	for (i = 0; i < spi->info->board_size; i++) {
		struct spi_device *spidev;

		dev_info(&dev->dev, "registering %p: %s\n",
			 &spi->info->board_info[i],
			 spi->info->board_info[i].modalias);

		spi->info->board_info[i].controller_data = spi;
		spidev = spi_new_device(master, spi->info->board_info + i);
		if (spidev)
			spidev->max_speed_hz =
					  spi->info->board_info[i].max_speed_hz;
	}

	return 0;

 err_no_bitbang:
	spi_master_put(spi->bitbang.master);
 err:
	return ret;
}

static int s3c64xx_spigpio_remove(struct platform_device *dev)
{
	struct s3c64xx_spigpio *sp = platform_get_drvdata(dev);

	spi_bitbang_stop(&sp->bitbang);
	spi_master_put(sp->bitbang.master);

	return 0;
}

#define s3c64xx_spigpio_suspend NULL
#define s3c64xx_spigpio_resume NULL

static struct platform_driver s3c64xx_spigpio_drv = {
	.probe		= s3c64xx_spigpio_probe,
	.remove		= s3c64xx_spigpio_remove,
	.suspend	= s3c64xx_spigpio_suspend,
	.resume		= s3c64xx_spigpio_resume,
	.driver		= {
		.name	= "spi_s3c64xx_gpio",
		.owner	= THIS_MODULE,
	},
};

static int __init s3c64xx_spigpio_init(void)
{
	return platform_driver_register(&s3c64xx_spigpio_drv);
}

static void __exit s3c64xx_spigpio_exit(void)
{
	platform_driver_unregister(&s3c64xx_spigpio_drv);
}

module_init(s3c64xx_spigpio_init);
module_exit(s3c64xx_spigpio_exit);

MODULE_DESCRIPTION("S3C64XX GPIO-SPI Driver");
MODULE_AUTHOR("Matt Hsu, <matt_hsu@openmoko.org>");
MODULE_LICENSE("GPLv2");
