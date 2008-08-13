/* Linux kernel driver for the ST LIS302D 3-axis accelerometer
 *
 * Copyright (C) 2007 by Openmoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 * TODO
 * 	* statistics for overflow events
 * 	* configuration interface (sysfs) for
 * 		* enable/disable x/y/z axis data ready
 * 		* enable/disable resume from freee fall / click
 * 		* free fall / click parameters
 * 		* high pass filter parameters
 */
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>

#include <linux/lis302dl.h>

/* lowlevel register access functions */

#define READ_BIT		0x80
#define READ_BIT_INC_ADS	0xc0
#define	ADDR_MASK		0x3f

static u_int8_t __reg_read(struct lis302dl_info *lis, u_int8_t reg)
{
	int rc;
	u_int8_t cmd;

	BUG_ON(reg & ~ADDR_MASK);

	cmd = reg | READ_BIT;

	rc = spi_w8r8(lis->spi_dev, cmd);

	return rc;
}

static u_int8_t reg_read(struct lis302dl_info *lis, u_int8_t reg)
{
	u_int8_t ret;

	mutex_lock(&lis->lock);
	ret = __reg_read(lis, reg);
	mutex_unlock(&lis->lock);

	return ret;
}

static int __reg_write(struct lis302dl_info *lis, u_int8_t reg, u_int8_t val)
{
	u_int8_t buf[2];

	BUG_ON(reg & ~ADDR_MASK);

	buf[0] = reg;
	buf[1] = val;

	return spi_write(lis->spi_dev, buf, sizeof(buf));
}

static int reg_write(struct lis302dl_info *lis, u_int8_t reg, u_int8_t val)
{
	int ret;

	mutex_lock(&lis->lock);
	ret = __reg_write(lis, reg, val);
	mutex_unlock(&lis->lock);

	return ret;
}

static int reg_set_bit_mask(struct lis302dl_info *lis,
			    u_int8_t reg, u_int8_t mask, u_int8_t val)
{
	int ret;
	u_int8_t tmp;

	val &= mask;

	mutex_lock(&lis->lock);

	tmp = __reg_read(lis, reg);
	tmp &= ~mask;
	tmp |= val;
	ret = __reg_write(lis, reg, tmp);

	mutex_unlock(&lis->lock);

	return ret;
}

/* interrupt handling related */

enum lis302dl_intmode {
	LIS302DL_INTMODE_GND		= 0x00,
	LIS302DL_INTMODE_FF_WU_1	= 0x01,
	LIS302DL_INTMODE_FF_WU_2	= 0x02,
	LIS302DL_INTMODE_FF_WU_12	= 0x03,
	LIS302DL_INTMODE_DATA_READY	= 0x04,
	LIS302DL_INTMODE_CLICK		= 0x07,
};

static void lis302dl_int_mode(struct spi_device *spi, int int_pin,
			      enum lis302dl_intmode mode)
{
	struct lis302dl_info *lis = dev_get_drvdata(&spi->dev);

	switch (int_pin) {
	case 1:
		reg_set_bit_mask(lis, LIS302DL_REG_CTRL3, 0x07, mode);
		break;
	case 2:
		reg_set_bit_mask(lis, LIS302DL_REG_CTRL3, 0x38, mode << 3);
		break;
	default:
		BUG();
	}
}
#if 0
static void _report_btn_single(struct input_dev *inp, int btn)
{
	input_report_key(inp, btn, 1);
	input_sync(inp);
	input_report_key(inp, btn, 0);
}

static void _report_btn_double(struct input_dev *inp, int btn)
{
	input_report_key(inp, btn, 1);
	input_sync(inp);
	input_report_key(inp, btn, 0);
	input_sync(inp);
	input_report_key(inp, btn, 1);
	input_sync(inp);
	input_report_key(inp, btn, 0);
}
#endif


static irqreturn_t lis302dl_interrupt(int irq, void *_lis)
{
	struct lis302dl_info *lis = _lis;

	(lis->pdata->lis302dl_bitbang_read)(lis);
	return IRQ_HANDLED;
}

/* sysfs */

static ssize_t show_rate(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct lis302dl_info *lis = dev_get_drvdata(dev);
	u_int8_t ctrl1 = reg_read(lis, LIS302DL_REG_CTRL1);

	return sprintf(buf, "%d\n", ctrl1 & LIS302DL_CTRL1_DR ? 400 : 100);
}

static ssize_t set_rate(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct lis302dl_info *lis = dev_get_drvdata(dev);

	if (!strcmp(buf, "400\n"))
		reg_set_bit_mask(lis, LIS302DL_REG_CTRL1, LIS302DL_CTRL1_DR,
				 LIS302DL_CTRL1_DR);
	else
		reg_set_bit_mask(lis, LIS302DL_REG_CTRL1, LIS302DL_CTRL1_DR, 0);

	return count;
}

static DEVICE_ATTR(sample_rate, S_IRUGO | S_IWUSR, show_rate, set_rate);

static ssize_t show_scale(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct lis302dl_info *lis = dev_get_drvdata(dev);
	u_int8_t ctrl1 = reg_read(lis, LIS302DL_REG_CTRL1);

	return sprintf(buf, "%s\n", ctrl1 & LIS302DL_CTRL1_FS ? "9.2" : "2.3");
}

static ssize_t set_scale(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct lis302dl_info *lis = dev_get_drvdata(dev);

	if (!strcmp(buf, "9.2\n"))
		reg_set_bit_mask(lis, LIS302DL_REG_CTRL1, LIS302DL_CTRL1_FS,
				 LIS302DL_CTRL1_FS);
	else
		reg_set_bit_mask(lis, LIS302DL_REG_CTRL1, LIS302DL_CTRL1_FS, 0);

	return count;
}

static DEVICE_ATTR(full_scale, S_IRUGO | S_IWUSR, show_scale, set_scale);

static ssize_t lis302dl_dump(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct lis302dl_info *lis = dev_get_drvdata(dev);
	int n = 0;
	u8 reg[0x40];
	char *end = buf;
	unsigned long flags;

	local_save_flags(flags);

	for (n = 0; n < sizeof(reg); n++)
		reg[n] = reg_read(lis, n);

	local_irq_restore(flags);

	for (n = 0; n < sizeof(reg); n += 16) {
		hex_dump_to_buffer(reg + n, 16, 16, 1, end, 128, 0);
		end += strlen(end);
		*end++ = '\n';
		*end++ = '\0';
	}

	return end - buf;
}
static DEVICE_ATTR(dump, S_IRUGO, lis302dl_dump, NULL);

static struct attribute *lis302dl_sysfs_entries[] = {
	&dev_attr_sample_rate.attr,
	&dev_attr_full_scale.attr,
	&dev_attr_dump.attr,
	NULL
};

static struct attribute_group lis302dl_attr_group = {
	.name	= NULL,
	.attrs	= lis302dl_sysfs_entries,
};

/* input device handling and driver core interaction */

static int lis302dl_input_open(struct input_dev *inp)
{
	struct lis302dl_info *lis = inp->private;
	u_int8_t ctrl1 = LIS302DL_CTRL1_PD | LIS302DL_CTRL1_Xen |
			 LIS302DL_CTRL1_Yen | LIS302DL_CTRL1_Zen;
	unsigned long flags;

	local_save_flags(flags);
	/* make sure we're powered up and generate data ready */
	reg_set_bit_mask(lis, LIS302DL_REG_CTRL1, ctrl1, ctrl1);
	local_irq_restore(flags);

	/* kick it off -- since we are edge triggered, if we missed the edge
	 * permanent low interrupt is death for us */
	(lis->pdata->lis302dl_bitbang_read)(lis);

	return 0;
}

static void lis302dl_input_close(struct input_dev *inp)
{
	struct lis302dl_info *lis = inp->private;
	u_int8_t ctrl1 = LIS302DL_CTRL1_Xen | LIS302DL_CTRL1_Yen |
			 LIS302DL_CTRL1_Zen;
	unsigned long flags;

	local_save_flags(flags);

	/* since the input core already serializes access and makes sure we
	 * only see close() for the close of the last user, we can safely
	 * disable the data ready events */
	reg_set_bit_mask(lis, LIS302DL_REG_CTRL1, ctrl1, 0x00);

	/* however, don't power down the whole device if still needed */
	if (!(lis->flags & LIS302DL_F_WUP_FF ||
	      lis->flags & LIS302DL_F_WUP_CLICK)) {
		reg_set_bit_mask(lis, LIS302DL_REG_CTRL1, LIS302DL_CTRL1_PD,
				 0x00);
	}
	local_irq_restore(flags);
}

/* get the device to reload its coefficients from EEPROM and wait for it
 * to complete
 */

static int __lis302dl_reset_device(struct lis302dl_info *lis)
{
	int timeout = 10;

	reg_write(lis, LIS302DL_REG_CTRL2, LIS302DL_CTRL2_BOOT |
							    LIS302DL_CTRL2_FDS);

	while ((reg_read(lis, LIS302DL_REG_CTRL2) & LIS302DL_CTRL2_BOOT) &&
								    (timeout--))
		mdelay(1);

	return !!(timeout < 0);
}

static int __devinit lis302dl_probe(struct spi_device *spi)
{
	int rc;
	struct lis302dl_info *lis;
	u_int8_t wai;
	unsigned long flags;
	struct lis302dl_platform_data *pdata;

	lis = kzalloc(sizeof(*lis), GFP_KERNEL);
	if (!lis)
		return -ENOMEM;

	local_save_flags(flags);

	mutex_init(&lis->lock);
	lis->spi_dev = spi;

	spi_set_drvdata(spi, lis);

	pdata = spi->dev.platform_data;

	rc = spi_setup(spi);
	if (rc < 0) {
		dev_err(&spi->dev, "error during spi_setup\n");
		dev_set_drvdata(&spi->dev, NULL);
		goto bail_free_lis;
	}

	wai = reg_read(lis, LIS302DL_REG_WHO_AM_I);
	if (wai != LIS302DL_WHO_AM_I_MAGIC) {
		dev_err(&spi->dev, "unknown who_am_i signature 0x%02x\n", wai);
		dev_set_drvdata(&spi->dev, NULL);
		rc = -ENODEV;
		goto bail_free_lis;
	}

	rc = sysfs_create_group(&spi->dev.kobj, &lis302dl_attr_group);
	if (rc) {
		dev_err(&spi->dev, "error creating sysfs group\n");
		goto bail_free_lis;
	}

	/* initialize input layer details */
	lis->input_dev = input_allocate_device();
	if (!lis->input_dev) {
		dev_err(&spi->dev, "Unable to allocate input device\n");
		goto bail_sysfs;
	}

	set_bit(EV_REL, lis->input_dev->evbit);
	set_bit(REL_X, lis->input_dev->relbit);
	set_bit(REL_Y, lis->input_dev->relbit);
	set_bit(REL_Z, lis->input_dev->relbit);
/*	set_bit(EV_KEY, lis->input_dev->evbit);
	set_bit(BTN_X, lis->input_dev->keybit);
	set_bit(BTN_Y, lis->input_dev->keybit);
	set_bit(BTN_Z, lis->input_dev->keybit);
*/
	lis->input_dev->private = lis;
	lis->input_dev->name = pdata->name;
	 /* SPI Bus not defined as a valid bus for input subsystem*/
	lis->input_dev->id.bustype = BUS_I2C; /* lie about it */
	lis->input_dev->open = lis302dl_input_open;
	lis->input_dev->close = lis302dl_input_close;

	rc = input_register_device(lis->input_dev);
	if (rc) {
		dev_err(&spi->dev, "error %d registering input device\n", rc);
		goto bail_inp_dev;
	}

	if (__lis302dl_reset_device(lis))
		dev_err(&spi->dev, "device BOOT reload failed\n");

	/* force us powered */
	reg_write(lis, LIS302DL_REG_CTRL1, LIS302DL_CTRL1_PD |
					   LIS302DL_CTRL1_Xen |
					   LIS302DL_CTRL1_Yen |
					   LIS302DL_CTRL1_Zen);
	mdelay(1);

	reg_write(lis, LIS302DL_REG_CTRL2, 0);
	reg_write(lis, LIS302DL_REG_CTRL3, LIS302DL_CTRL3_PP_OD |
							    LIS302DL_CTRL3_IHL);
	reg_write(lis, LIS302DL_REG_FF_WU_THS_1, 0x14);
	reg_write(lis, LIS302DL_REG_FF_WU_DURATION_1, 0x00);
	reg_write(lis, LIS302DL_REG_FF_WU_CFG_1, 0x95);

	/* start off in powered down mode; we power up when someone opens us */
	reg_write(lis, LIS302DL_REG_CTRL1, LIS302DL_CTRL1_Xen |
					   LIS302DL_CTRL1_Yen |
			 		   LIS302DL_CTRL1_Zen);

	if (pdata->open_drain)
		/* switch interrupt to open collector, active-low */
		reg_write(lis, LIS302DL_REG_CTRL3, LIS302DL_CTRL3_PP_OD |
						   LIS302DL_CTRL3_IHL);
	else
		/* push-pull, active-low */
		reg_write(lis, LIS302DL_REG_CTRL3, LIS302DL_CTRL3_IHL);

	lis302dl_int_mode(spi, 1, LIS302DL_INTMODE_DATA_READY);
	lis302dl_int_mode(spi, 2, LIS302DL_INTMODE_DATA_READY);

	reg_read(lis, LIS302DL_REG_STATUS);
	reg_read(lis, LIS302DL_REG_FF_WU_SRC_1);
	reg_read(lis, LIS302DL_REG_FF_WU_SRC_2);
	reg_read(lis, LIS302DL_REG_CLICK_SRC);

	dev_info(&spi->dev, "Found %s\n", pdata->name);

	lis->pdata = pdata;

	rc = request_irq(lis->spi_dev->irq, lis302dl_interrupt,
			 IRQF_TRIGGER_FALLING, "lis302dl", lis);
	if (rc < 0) {
		dev_err(&spi->dev, "error requesting IRQ %d\n",
			lis->spi_dev->irq);
		goto bail_inp_reg;
	}
	local_irq_restore(flags);
	return 0;

bail_inp_reg:
	input_unregister_device(lis->input_dev);
bail_inp_dev:
	input_free_device(lis->input_dev);
bail_sysfs:
	sysfs_remove_group(&spi->dev.kobj, &lis302dl_attr_group);
bail_free_lis:
	kfree(lis);
	local_irq_restore(flags);
	return rc;
}

static int __devexit lis302dl_remove(struct spi_device *spi)
{
	struct lis302dl_info *lis = dev_get_drvdata(&spi->dev);
	unsigned long flags;

	/* Reset and power down the device */
	local_save_flags(flags);
	reg_write(lis, LIS302DL_REG_CTRL3, 0x00);
	reg_write(lis, LIS302DL_REG_CTRL2, 0x00);
	reg_write(lis, LIS302DL_REG_CTRL1, 0x00);
	local_irq_restore(flags);

	/* Cleanup resources */
	free_irq(lis->spi_dev->irq, lis);
	sysfs_remove_group(&spi->dev.kobj, &lis302dl_attr_group);
	input_unregister_device(lis->input_dev);
	if (lis->input_dev)
		input_free_device(lis->input_dev);
	dev_set_drvdata(&spi->dev, NULL);
	kfree(lis);

	return 0;
}

#ifdef CONFIG_PM
static int lis302dl_suspend(struct spi_device *spi, pm_message_t state)
{
	struct lis302dl_info *lis = dev_get_drvdata(&spi->dev);
	unsigned long flags;

	disable_irq(lis->spi_dev->irq);
	local_save_flags(flags);

	/*
	 * When we share SPI over multiple sensors, there is a race here
	 * that one or more sensors will lose.  In that case, the shared
	 * SPI bus GPIO will be in sleep mode and partially pulled down.  So
	 * we explicitly put our IO into "wake" mode here before the final
	 * traffic to the sensor.
	 */
	(lis->pdata->lis302dl_suspend_io)(lis, 1);

	/* save registers */
	lis->regs[LIS302DL_REG_CTRL1] = reg_read(lis, LIS302DL_REG_CTRL1);
	lis->regs[LIS302DL_REG_CTRL2] = reg_read(lis, LIS302DL_REG_CTRL2);
	lis->regs[LIS302DL_REG_CTRL3] = reg_read(lis, LIS302DL_REG_CTRL3);
	lis->regs[LIS302DL_REG_FF_WU_CFG_1] =
				reg_read(lis, LIS302DL_REG_FF_WU_CFG_1);
	lis->regs[LIS302DL_REG_FF_WU_THS_1] =
				reg_read(lis, LIS302DL_REG_FF_WU_THS_1);
	lis->regs[LIS302DL_REG_FF_WU_DURATION_1] =
				reg_read(lis, LIS302DL_REG_FF_WU_DURATION_1);
	lis->regs[LIS302DL_REG_FF_WU_CFG_2] =
				reg_read(lis, LIS302DL_REG_FF_WU_CFG_2);
	lis->regs[LIS302DL_REG_FF_WU_THS_2] =
				reg_read(lis, LIS302DL_REG_FF_WU_THS_2);
	lis->regs[LIS302DL_REG_FF_WU_DURATION_2] =
				reg_read(lis, LIS302DL_REG_FF_WU_DURATION_2);
	lis->regs[LIS302DL_REG_CLICK_CFG] =
				reg_read(lis, LIS302DL_REG_CLICK_CFG);
	lis->regs[LIS302DL_REG_CLICK_THSY_X] =
				reg_read(lis, LIS302DL_REG_CLICK_THSY_X);
	lis->regs[LIS302DL_REG_CLICK_THSZ] =
				reg_read(lis, LIS302DL_REG_CLICK_THSZ);
	lis->regs[LIS302DL_REG_CLICK_TIME_LIMIT] =
				reg_read(lis, LIS302DL_REG_CLICK_TIME_LIMIT);
	lis->regs[LIS302DL_REG_CLICK_LATENCY] =
				reg_read(lis, LIS302DL_REG_CLICK_LATENCY);
	lis->regs[LIS302DL_REG_CLICK_WINDOW] =
				reg_read(lis, LIS302DL_REG_CLICK_WINDOW);

	/* determine if we want to wake up from the accel. */
	if (!(lis->flags & LIS302DL_F_WUP_FF ||
	      lis->flags & LIS302DL_F_WUP_CLICK)) {
		/* power down */
		u_int8_t tmp;
		tmp = reg_read(lis, LIS302DL_REG_CTRL1);
		tmp &= ~LIS302DL_CTRL1_PD;
		reg_write(lis, LIS302DL_REG_CTRL1, tmp);
	}

	/* place our IO to the device in sleep-compatible states */
	(lis->pdata->lis302dl_suspend_io)(lis, 0);

	local_irq_restore(flags);

	return 0;
}

static int lis302dl_resume(struct spi_device *spi)
{
	struct lis302dl_info *lis = dev_get_drvdata(&spi->dev);
	unsigned long flags;

	local_save_flags(flags);

	/* get our IO to the device back in operational states */
	(lis->pdata->lis302dl_suspend_io)(lis, 1);

	/* resume from powerdown first! */
	reg_write(lis, LIS302DL_REG_CTRL1, LIS302DL_CTRL1_PD |
					   LIS302DL_CTRL1_Xen |
					   LIS302DL_CTRL1_Yen |
					   LIS302DL_CTRL1_Zen);
	mdelay(1);

	if (__lis302dl_reset_device(lis))
		dev_err(&spi->dev, "device BOOT reload failed\n");

	/* restore registers after resume */
	reg_write(lis, LIS302DL_REG_CTRL1, lis->regs[LIS302DL_REG_CTRL1] |
						LIS302DL_CTRL1_PD |
						LIS302DL_CTRL1_Xen |
						LIS302DL_CTRL1_Yen |
						LIS302DL_CTRL1_Zen);
	reg_write(lis, LIS302DL_REG_CTRL2, lis->regs[LIS302DL_REG_CTRL2]);
	reg_write(lis, LIS302DL_REG_CTRL3, lis->regs[LIS302DL_REG_CTRL3]);
	reg_write(lis, LIS302DL_REG_FF_WU_CFG_1,
		  lis->regs[LIS302DL_REG_FF_WU_CFG_1]);
	reg_write(lis, LIS302DL_REG_FF_WU_THS_1,
		  lis->regs[LIS302DL_REG_FF_WU_THS_1]);
	reg_write(lis, LIS302DL_REG_FF_WU_DURATION_1,
		  lis->regs[LIS302DL_REG_FF_WU_DURATION_1]);
	reg_write(lis, LIS302DL_REG_FF_WU_CFG_2,
		  lis->regs[LIS302DL_REG_FF_WU_CFG_2]);
	reg_write(lis, LIS302DL_REG_FF_WU_THS_2,
		  lis->regs[LIS302DL_REG_FF_WU_THS_2]);
	reg_write(lis, LIS302DL_REG_FF_WU_DURATION_2,
		  lis->regs[LIS302DL_REG_FF_WU_DURATION_2]);
	reg_write(lis, LIS302DL_REG_CLICK_CFG,
		  lis->regs[LIS302DL_REG_CLICK_CFG]);
	reg_write(lis, LIS302DL_REG_CLICK_THSY_X,
		  lis->regs[LIS302DL_REG_CLICK_THSY_X]);
	reg_write(lis, LIS302DL_REG_CLICK_THSZ,
		  lis->regs[LIS302DL_REG_CLICK_THSZ]);
	reg_write(lis, LIS302DL_REG_CLICK_TIME_LIMIT,
		  lis->regs[LIS302DL_REG_CLICK_TIME_LIMIT]);
	reg_write(lis, LIS302DL_REG_CLICK_LATENCY,
		  lis->regs[LIS302DL_REG_CLICK_LATENCY]);
	reg_write(lis, LIS302DL_REG_CLICK_WINDOW,
		  lis->regs[LIS302DL_REG_CLICK_WINDOW]);

	local_irq_restore(flags);
	enable_irq(lis->spi_dev->irq);

	return 0;
}
#else
#define lis302dl_suspend	NULL
#define lis302dl_resume		NULL
#endif

static struct spi_driver lis302dl_driver = {
	.driver = {
		.name	= "lis302dl",
		.owner	= THIS_MODULE,
	},

	.probe	 = lis302dl_probe,
	.remove	 = __devexit_p(lis302dl_remove),
	.suspend = lis302dl_suspend,
	.resume	 = lis302dl_resume,
};

static int __init lis302dl_init(void)
{
	return spi_register_driver(&lis302dl_driver);
}

static void __exit lis302dl_exit(void)
{
	spi_unregister_driver(&lis302dl_driver);
}

MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_LICENSE("GPL");

module_init(lis302dl_init);
module_exit(lis302dl_exit);
