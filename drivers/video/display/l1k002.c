/*
 *  Copyright (C) 2009 Openmoko, Inc.
 *
 *  Author: Matt Hsu <matt_hsu@openmoko.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or (at
 *  your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
*/

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#include <linux/l1k002.h>

struct l1k002_data {
	struct spi_device *spi;
	struct mutex lock;
	u8 mosi_buf[2];
	u8 reg_cache[0x40];
};

enum l1k002_regs_table {

	L1K002_REG_SYNCP_SEL 		= 0x02,
	L1K002_REG_VSTS 		= 0x03,
	L1K002_REG_HSTS 		= 0x04,
	L1K002_REG_MISC 		= 0x07,
	L1K002_REG_CMDR 		= 0x08,
	L1K002_REG_IN_DATA_TIMING 	= 0x09,
	L1K002_REG_ENGR_OTP 		= 0x0b,
	L1K002_REG_VGLS 		= 0x0c,
	L1K002_REG_DISP_8_9		= 0x0d,
	L1K002_REG_DISP_0_7		= 0x0e,
	L1K002_REG_HTOTAL_8_10		= 0x0f,
	L1K002_REG_HTOTAL_0_7 		= 0x10,
	L1K002_REG_WCKH 		= 0x20,
	L1K002_REG_GCKH 		= 0x21,
	L1K002_REG_DCKH 		= 0x22,
	L1K002_REG_WENBV 		= 0x23,
	L1K002_REG_DCKV 		= 0x25,
	L1K002_REG_WCKV 		= 0x27,
	L1K002_REG_DA_VCOM 		= 0x2a,
	L1K002_REG_PVH 			= 0x2b,
	L1K002_REG_NVH_NVL 		= 0x2c,
	L1K002_REG_GC1 			= 0x2d,
	L1K002_REG_GC2  		= 0x2e,
	L1K002_REG_GC3  		= 0x2f,
	L1K002_REG_GC4  		= 0x30,
	L1K002_REG_GC5  		= 0x31,
	L1K002_REG_GC6  		= 0x32,
	L1K002_REG_GC7  		= 0x33,
	L1K002_REG_GC8 			= 0x34,
	L1K002_REG_GC9  		= 0x35,
	L1K002_REG_GC10 		= 0x36,
	L1K002_REG_GC11	 		= 0x37,
	L1K002_REG_GC12 		= 0x38,
	L1K002_REG_GC13 		= 0x39,
	L1K002_REG_GC14 		= 0x3a,
	L1K002_REG_GC15 		= 0x3b,
};

static int l1k002_reg_write(struct l1k002_data *l1k002, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&l1k002->lock);

	l1k002->mosi_buf[0] = reg;
	l1k002->mosi_buf[1] = data;

	ret = spi_write(l1k002->spi, (u8 *)l1k002->mosi_buf, 2*sizeof(u8));
	if (ret == 0)
		l1k002->reg_cache[reg] = data;
	else
		dev_err(&l1k002->spi->dev, "reg spi_write ret: %d\n", ret);

	mutex_unlock(&l1k002->lock);
	return ret;
}

static int l1k002_init_reg(struct l1k002_data *l1k002)
{
	int ret;

	/* software reset */
	ret = l1k002_reg_write(l1k002, L1K002_REG_CMDR, 0x01);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_CMDR, 0x00);

	/* setup color mode and direction */
	ret |= l1k002_reg_write(l1k002, L1K002_REG_MISC, 0xd9);

	/* dclk initial */
	ret |= l1k002_reg_write(l1k002, L1K002_REG_SYNCP_SEL, 0x00);

	/* start vertical data */
	ret |= l1k002_reg_write(l1k002, L1K002_REG_VSTS, 0x04);

	/* start horizonal data */
	ret |= l1k002_reg_write(l1k002, L1K002_REG_HSTS, 0x14);

	/* setup hsnc, vsnc, data_enable */
	ret |= l1k002_reg_write(l1k002, L1K002_REG_IN_DATA_TIMING, 0x03);
	/* enable engineering mode and OTP */
	ret |= l1k002_reg_write(l1k002, L1K002_REG_ENGR_OTP, 0x18);

	/* display area */
	ret |= l1k002_reg_write(l1k002, L1K002_REG_VGLS, 0x41);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_DISP_8_9, 0x02);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_DISP_0_7, 0x80);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_HTOTAL_8_10, 0x02);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_HTOTAL_0_7, 0x08);

	/* CKH pulse config */
	ret |= l1k002_reg_write(l1k002, L1K002_REG_WCKH, 0x3c);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_GCKH, 0x0c);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_DCKH, 0x10);

	/* ENBV config */
	ret |= l1k002_reg_write(l1k002, L1K002_REG_WENBV, 0x38);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_DCKV, 0x3c);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_WCKV, 0xdb);

	/* driving voltage */
	ret |= l1k002_reg_write(l1k002, L1K002_REG_DA_VCOM, 0x66);

	/* gamma output voltage level */
	ret |= l1k002_reg_write(l1k002, L1K002_REG_PVH, 0x70);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_NVH_NVL, 0x70);

	/* gamma correction */
	ret |= l1k002_reg_write(l1k002, L1K002_REG_GC1, 0x15);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_GC2, 0xaa);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_GC3, 0xbf);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_GC4, 0x86);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_GC5, 0x11);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_GC6, 0x5e);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_GC7, 0xb6);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_GC8, 0x16);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_GC9, 0x4e);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_GC10, 0x78);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_GC11, 0xbf);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_GC12, 0xec);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_GC13, 0x10);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_GC14, 0x30);
	ret |= l1k002_reg_write(l1k002, L1K002_REG_GC15, 0xff);

	ret |= l1k002_reg_write(l1k002, 0x07, 0xc9);

	if (ret == 0)
		dev_info(&l1k002->spi->dev, "initialize OK \n");
	else
		dev_err(&l1k002->spi->dev, "initialize failed ret: %d\n", ret);
	return ret;
}

static int l1k002_probe(struct spi_device *spi)
{
	int ret;
	struct l1k002_data *l1k002;
	struct l1k002_platform_data *l1k002_pdata = spi->dev.platform_data;

	if (l1k002_pdata == NULL) {
		dev_err(&spi->dev,
			"no platform data available \n");
		return -EINVAL;
	}

	spi->mode = SPI_CPOL | SPI_CPHA;
	spi->bits_per_word = 8;

	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev,
			"error during spi_setup of l1k002 driver\n");
		return ret;
	}

	l1k002 = kzalloc(sizeof(*l1k002), GFP_KERNEL);
	if (!l1k002)
		return -ENOMEM;

	l1k002->spi = spi;
	dev_set_drvdata(&spi->dev, l1k002);

	mutex_init(&l1k002->lock);

	/* hard reset l1k002 */
	(l1k002_pdata->pwr_onoff)(1);

	ret = l1k002_init_reg(l1k002);
	if (ret)
		goto err_free;

	/* FIXME: sysfs should be added here */

	return 0;

err_free:
	kfree(l1k002);
	return ret;
}

static int __devexit l1k002_remove(struct spi_device *spi)
{
	struct l1k002 *l1k002 = dev_get_drvdata(&spi->dev);

	dev_set_drvdata(&spi->dev, NULL);
	kfree(l1k002);
	return 0;
}

#ifdef CONFIG_PM
static int l1k002_suspend(struct spi_device *spi, pm_message_t state)
{
	struct l1k002_platform_data *l1k002_pdata = spi->dev.platform_data;

	/* l1k002 doesn't have sleep mode
	 * it should be powered down as entering suspend state
	 */
	(l1k002_pdata->pwr_onoff)(0);
	return 0;
}

static int l1k002_resume(struct spi_device *spi)
{
	struct l1k002_platform_data *l1k002_pdata = spi->dev.platform_data;
	struct l1k002 *l1k002 = dev_get_drvdata(&spi->dev);

	(l1k002_pdata->pwr_onoff)(1);
	return l1k002_init_reg(l1k002);
}
#else
#define l1k002_suspend 	NULL
#define l1k002_resume 	NULL
#endif

static struct spi_driver l1k002_driver = {
	.driver = {
		.name	= "l1k002",
		.owner	= THIS_MODULE,
	},

	.probe	 = l1k002_probe,
	.remove	 = __devexit_p(l1k002_remove),
	.suspend = l1k002_suspend,
	.resume	 = l1k002_resume,
};

static int __init l1k002_init(void)
{
	return spi_register_driver(&l1k002_driver);
}

static void __exit l1k002_exit(void)
{
	spi_unregister_driver(&l1k002_driver);
}

MODULE_AUTHOR("Matt Hsu <matt_hsu@openmoko.org>");
MODULE_LICENSE("GPL v2");

module_init(l1k002_init);
module_exit(l1k002_exit);
