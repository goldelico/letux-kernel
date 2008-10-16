/* Linux kernel driver for the ST LIS302D 3-axis accelerometer
 *
 * Copyright (C) 2007-2008 by Openmoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 *         converted to private bitbang by:
 *         Andy Green <andy@openmoko.com>
 *         ability to set acceleration threshold added by:
 *         Simon Kagstrom <simon.kagstrom@gmail.com>
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

/* Utility functions */
static u8 __reg_read(struct lis302dl_info *lis, u8 reg)
{
	return (lis->pdata->lis302dl_bitbang_reg_read)(lis, reg);
}

static void __reg_write(struct lis302dl_info *lis, u8 reg, u8 val)
{
	(lis->pdata->lis302dl_bitbang_reg_write)(lis, reg, val);
}

static void __reg_set_bit_mask(struct lis302dl_info *lis, u8 reg, u8 mask,
		u8 val)
{
	u_int8_t tmp;

	val &= mask;

	tmp = __reg_read(lis, reg);
	tmp &= ~mask;
	tmp |= val;
	__reg_write(lis, reg, tmp);
}

static int __ms_to_duration(struct lis302dl_info *lis, int ms)
{
	/* If we have 400 ms sampling rate, the stepping is 2.5 ms,
	 * on 100 ms the stepping is 10ms */
	if (lis->flags & LIS302DL_F_DR)
		return min((ms * 10) / 25, 637);

	return min(ms / 10, 2550);
}

static int __duration_to_ms(struct lis302dl_info *lis, int duration)
{
	if (lis->flags & LIS302DL_F_DR)
		return (duration * 25) / 10;

	return duration * 10;
}

static u8 __mg_to_threshold(struct lis302dl_info *lis, int mg)
{
	/* If FS is set each bit is 71mg, otherwise 18mg. The THS register
	 * has 7 bits for the threshold value */
	if (lis->flags & LIS302DL_F_FS)
		return min(mg / 71, 127);

	return min(mg / 18, 127);
}

static int __threshold_to_mg(struct lis302dl_info *lis, u8 threshold)
{
	if (lis->flags & LIS302DL_F_FS)
		return threshold * 71;

	return threshold * 18;
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


static void __lis302dl_int_mode(struct device *dev, int int_pin,
			      enum lis302dl_intmode mode)
{
	struct lis302dl_info *lis = dev_get_drvdata(dev);

	switch (int_pin) {
	case 1:
		__reg_set_bit_mask(lis, LIS302DL_REG_CTRL3, 0x07, mode);
		break;
	case 2:
		__reg_set_bit_mask(lis, LIS302DL_REG_CTRL3, 0x38, mode << 3);
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


static void lis302dl_bitbang_read_sample(struct lis302dl_info *lis)
{
	u8 data = 0xc0 | LIS302DL_REG_OUT_X; /* read, autoincrement */
	u8 read[5];
	unsigned long flags;
	int mg_per_sample;

	local_irq_save(flags);
	mg_per_sample = __threshold_to_mg(lis, 1);

	(lis->pdata->lis302dl_bitbang)(lis, &data, 1, &read[0], 5);

	local_irq_restore(flags);

	input_report_rel(lis->input_dev, REL_X, mg_per_sample * (s8)read[0]);
	input_report_rel(lis->input_dev, REL_Y, mg_per_sample * (s8)read[2]);
	input_report_rel(lis->input_dev, REL_Z, mg_per_sample * (s8)read[4]);

	input_sync(lis->input_dev);

	/* Reset the HP filter */
	__reg_read(lis,	LIS302DL_REG_HP_FILTER_RESET);
}

static irqreturn_t lis302dl_interrupt(int irq, void *_lis)
{
	struct lis302dl_info *lis = _lis;

	lis302dl_bitbang_read_sample(lis);
	return IRQ_HANDLED;
}

/* sysfs */

static ssize_t show_rate(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct lis302dl_info *lis = dev_get_drvdata(dev);
	u8 ctrl1;
	unsigned long flags;

	local_irq_save(flags);
	ctrl1 = __reg_read(lis, LIS302DL_REG_CTRL1);
	local_irq_restore(flags);

	return sprintf(buf, "%d\n", ctrl1 & LIS302DL_CTRL1_DR ? 400 : 100);
}

static ssize_t set_rate(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct lis302dl_info *lis = dev_get_drvdata(dev);
	unsigned long flags;
	int duration_ms = __duration_to_ms(lis, lis->duration);

	local_irq_save(flags);

	if (!strcmp(buf, "400\n")) {
		__reg_set_bit_mask(lis, LIS302DL_REG_CTRL1, LIS302DL_CTRL1_DR,
				 LIS302DL_CTRL1_DR);
		lis->flags |= LIS302DL_F_DR;
	} else {
		__reg_set_bit_mask(lis, LIS302DL_REG_CTRL1, LIS302DL_CTRL1_DR,
				0);
		lis->flags &= ~LIS302DL_F_DR;
	}
	lis->duration = __ms_to_duration(lis, duration_ms);
	local_irq_restore(flags);

	return count;
}

static DEVICE_ATTR(sample_rate, S_IRUGO | S_IWUSR, show_rate, set_rate);

static ssize_t show_scale(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct lis302dl_info *lis = dev_get_drvdata(dev);
	u_int8_t ctrl1;
	unsigned long flags;

	local_irq_save(flags);
	ctrl1 = __reg_read(lis, LIS302DL_REG_CTRL1);
	local_irq_restore(flags);

	return sprintf(buf, "%s\n", ctrl1 & LIS302DL_CTRL1_FS ? "9.2" : "2.3");
}

static ssize_t set_scale(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct lis302dl_info *lis = dev_get_drvdata(dev);
	unsigned long flags;
	int threshold_mg = __threshold_to_mg(lis, lis->threshold);

	local_irq_save(flags);

	if (!strcmp(buf, "9.2\n")) {
		__reg_set_bit_mask(lis, LIS302DL_REG_CTRL1, LIS302DL_CTRL1_FS,
				 LIS302DL_CTRL1_FS);
		lis->flags |= LIS302DL_F_FS;
	} else {
		__reg_set_bit_mask(lis, LIS302DL_REG_CTRL1, LIS302DL_CTRL1_FS,
				0);
		lis->flags &= ~LIS302DL_F_FS;
	}

	/* Adjust the threshold */
	lis->threshold = __mg_to_threshold(lis, threshold_mg);
	if (lis->flags & LIS302DL_F_INPUT_OPEN)
		__reg_write(lis, LIS302DL_REG_FF_WU_THS_1, lis->threshold);

	local_irq_restore(flags);

	return count;
}

static DEVICE_ATTR(full_scale, S_IRUGO | S_IWUSR, show_scale, set_scale);

static ssize_t show_threshold(struct device *dev, struct device_attribute *attr,
		 char *buf)
{
	struct lis302dl_info *lis = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", __threshold_to_mg(lis, lis->threshold));
}

static ssize_t set_threshold(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	struct lis302dl_info *lis = dev_get_drvdata(dev);
	u32 val;

	if (sscanf(buf, "%d\n", &val) != 1)
		return -EINVAL;
	/* 8g is the maximum if FS is 1 */
	if (val < 0 || val > 8000)
		return -ERANGE;

	/* Set the threshold and write it out if the device is used */
	lis->threshold = __mg_to_threshold(lis, val);
	if (lis->flags & LIS302DL_F_INPUT_OPEN)
		__reg_write(lis, LIS302DL_REG_FF_WU_THS_1, lis->threshold);

	return count;
}

static DEVICE_ATTR(threshold, S_IRUGO | S_IWUSR, show_threshold, set_threshold);

static ssize_t show_duration(struct device *dev, struct device_attribute *attr,
		 char *buf)
{
	struct lis302dl_info *lis = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", __duration_to_ms(lis, lis->duration));
}

static ssize_t set_duration(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	struct lis302dl_info *lis = dev_get_drvdata(dev);
	u32 val;

	if (sscanf(buf, "%d\n", &val) != 1)
		return -EINVAL;
	if (val < 0 || val > 2550)
		return -ERANGE;

	lis->duration = __ms_to_duration(lis, val);
	if (lis->flags & LIS302DL_F_INPUT_OPEN)
		__reg_write(lis, LIS302DL_REG_FF_WU_DURATION_1, lis->duration);

	return count;
}

static DEVICE_ATTR(duration, S_IRUGO | S_IWUSR, show_duration, set_duration);

static ssize_t lis302dl_dump(struct device *dev, struct device_attribute *attr,
								      char *buf)
{
	struct lis302dl_info *lis = dev_get_drvdata(dev);
	int n = 0;
	u8 reg[0x40];
	char *end = buf;
	unsigned long flags;

	local_irq_save(flags);

	for (n = 0; n < sizeof(reg); n++)
		reg[n] = __reg_read(lis, n);

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

/* Configure freefall/wakeup interrupts */
static ssize_t set_freefall_common(int which, struct device *dev,
		   struct device_attribute *attr, const char *buf, size_t count)
{
	struct lis302dl_info *lis = dev_get_drvdata(dev);
	u_int8_t x_lo, y_lo, z_lo;
	u_int8_t x_hi, y_hi, z_hi;
	int duration;
	int threshold;
	int and_events;
	int r_ths = LIS302DL_REG_FF_WU_THS_1; /* registers, assume first pin */
	int r_duration = LIS302DL_REG_FF_WU_DURATION_1;
	int r_cfg = LIS302DL_REG_FF_WU_CFG_1;
	int flag_mask = LIS302DL_F_WUP_FF_1;
	int intmode = LIS302DL_INTMODE_FF_WU_1;
	int x, y, z;
	int ms;
	unsigned long flags;

	/* Configure for second freefall/wakeup pin */
	if (which == 2) {
		r_ths = LIS302DL_REG_FF_WU_THS_2;
		r_duration = LIS302DL_REG_FF_WU_DURATION_2;
		r_cfg = LIS302DL_REG_FF_WU_CFG_2;
		flag_mask = LIS302DL_F_WUP_FF_2;
		intmode = LIS302DL_INTMODE_FF_WU_2;

		printk(KERN_WARNING
			    "Configuring second freefall / wakeup interrupt\n");
	}

	/* Parse the input */
	if (strcmp(buf, "0\n") == 0) {
		/* Turn off the interrupt */
		local_irq_save(flags);
		if (lis->flags & LIS302DL_F_IRQ_WAKE)
			disable_irq_wake(lis->pdata->interrupt);
		__lis302dl_int_mode(lis->dev, which,
						   LIS302DL_INTMODE_DATA_READY);
		lis->flags &= ~(flag_mask | LIS302DL_F_IRQ_WAKE);

		__reg_write(lis, r_cfg, 0);
		__reg_write(lis, r_ths, 0);
		__reg_write(lis, r_duration, 0);

		/* Power off unless the input subsystem is using the device */
		if (!(lis->flags & LIS302DL_F_INPUT_OPEN))
			__reg_set_bit_mask(lis, LIS302DL_REG_CTRL1,
							  LIS302DL_CTRL1_PD, 0);

		local_irq_restore(flags);

		return count;
	}

	if (sscanf(buf, "%d %d %d %d %d %d", &x, &y, &z, &threshold, &ms,
							      &and_events) != 6)
		return -EINVAL;

	local_irq_save(flags);
	duration = __ms_to_duration(lis, ms);
	local_irq_save(flags);

	if (duration < 0)
		return -ERANGE;

	/* 7 bits */
	if (threshold < 0 || threshold > 127)
		return -ERANGE;

	/* Interrupt flags */
	x_lo = x < 0 ? LIS302DL_FFWUCFG_XLIE : 0;
	y_lo = y < 0 ? LIS302DL_FFWUCFG_YLIE : 0;
	z_lo = z < 0 ? LIS302DL_FFWUCFG_ZLIE : 0;
	x_hi = x > 0 ? LIS302DL_FFWUCFG_XHIE : 0;
	y_hi = y > 0 ? LIS302DL_FFWUCFG_YHIE : 0;
	z_hi = z > 0 ? LIS302DL_FFWUCFG_ZHIE : 0;

	/* Setup the configuration registers */
	local_irq_save(flags);
	/* First zero to get to a known state */
	__reg_write(lis, r_cfg, 0);
	__reg_write(lis, r_cfg,
		(and_events ? LIS302DL_FFWUCFG_AOI : 0) |
		x_lo | x_hi | y_lo | y_hi | z_lo | z_hi);
	__reg_write(lis, r_ths, threshold & ~LIS302DL_FFWUTHS_DCRM);
	__reg_write(lis, r_duration, duration);

	/* Route the interrupt for wakeup */
	__lis302dl_int_mode(lis->dev, which, intmode);

	/* Power up the device and note that we want to wake up from
	 * this interrupt */
	if (!(lis->flags & LIS302DL_F_IRQ_WAKE))
		enable_irq_wake(lis->pdata->interrupt);

	lis->flags |= flag_mask | LIS302DL_F_IRQ_WAKE;
	__reg_set_bit_mask(lis, LIS302DL_REG_CTRL1, LIS302DL_CTRL1_PD,
			LIS302DL_CTRL1_PD);
	local_irq_restore(flags);

	return count;
}

static ssize_t set_freefall_1(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	return set_freefall_common(1, dev, attr, buf, count);
}
static ssize_t set_freefall_2(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	return set_freefall_common(2, dev, attr, buf, count);
}


static ssize_t show_freefall_common(int which, struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lis302dl_info *lis = dev_get_drvdata(dev);
	u_int8_t duration;
	u_int8_t threshold;
	u_int8_t config;
	u_int8_t r4;
	u_int8_t r5;
	int r_ths = LIS302DL_REG_FF_WU_THS_1; /* registers, assume first pin */
	int r_duration = LIS302DL_REG_FF_WU_DURATION_1;
	int r_cfg = LIS302DL_REG_FF_WU_CFG_1;
	int r_src = LIS302DL_REG_FF_WU_SRC_1;
	unsigned long flags;
	int ms;

	/* Configure second freefall/wakeup pin */
	if (which == 2) {
		r_ths = LIS302DL_REG_FF_WU_THS_2;
		r_duration = LIS302DL_REG_FF_WU_DURATION_2;
		r_cfg = LIS302DL_REG_FF_WU_CFG_2;
		r_src = LIS302DL_REG_FF_WU_SRC_2;
	}

	local_irq_save(flags);
	config = __reg_read(lis, r_cfg);
	threshold = __reg_read(lis, r_ths);
	duration = __reg_read(lis, r_duration);
	r4 = __reg_read(lis, r_src);
	r5 = __reg_read(lis, LIS302DL_REG_CTRL3);
	ms = __duration_to_ms(lis, duration);
	local_irq_restore(flags);

	/* All events off? */
	if ((config & (LIS302DL_FFWUCFG_XLIE | LIS302DL_FFWUCFG_XHIE |
			LIS302DL_FFWUCFG_YLIE | LIS302DL_FFWUCFG_YHIE |
			LIS302DL_FFWUCFG_ZLIE | LIS302DL_FFWUCFG_ZHIE)) == 0)
		return sprintf(buf, "off\n");


	return sprintf(buf,
			"%s events, %s interrupt, duration %d, threshold %d, "
			"enabled: %s %s %s %s %s %s\n",
			(config & LIS302DL_FFWUCFG_AOI) == 0 ? "or" : "and",
			(config & LIS302DL_FFWUCFG_LIR) == 0 ?
							"don't latch" : "latch",
			ms, threshold,
			(config & LIS302DL_FFWUCFG_XLIE) == 0 ? "---" : "xlo",
			(config & LIS302DL_FFWUCFG_XHIE) == 0 ? "---" : "xhi",
			(config & LIS302DL_FFWUCFG_YLIE) == 0 ? "---" : "ylo",
			(config & LIS302DL_FFWUCFG_YHIE) == 0 ? "---" : "yhi",
			(config & LIS302DL_FFWUCFG_ZLIE) == 0 ? "---" : "zlo",
			(config & LIS302DL_FFWUCFG_ZHIE) == 0 ? "---" : "zhi");
}

static ssize_t show_freefall_1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return show_freefall_common(1, dev, attr, buf);
}

static ssize_t show_freefall_2(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return show_freefall_common(2, dev, attr, buf);
}

static DEVICE_ATTR(freefall_wakeup_1, S_IRUGO | S_IWUSR, show_freefall_1,
								set_freefall_1);
static DEVICE_ATTR(freefall_wakeup_2, S_IRUGO | S_IWUSR, show_freefall_2,
								set_freefall_2);

static struct attribute *lis302dl_sysfs_entries[] = {
	&dev_attr_sample_rate.attr,
	&dev_attr_full_scale.attr,
	&dev_attr_threshold.attr,
	&dev_attr_duration.attr,
	&dev_attr_dump.attr,
	&dev_attr_freefall_wakeup_1.attr,
	&dev_attr_freefall_wakeup_2.attr,
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
	u_int8_t ctrl2 = LIS302DL_CTRL2_HPFF1;
	unsigned long flags;

	local_irq_save(flags);
	/* make sure we're powered up and generate data ready */
	__reg_set_bit_mask(lis, LIS302DL_REG_CTRL1, ctrl1, ctrl1);

	__reg_write(lis, LIS302DL_REG_CTRL2,
			ctrl2);
	__reg_write(lis, LIS302DL_REG_FF_WU_THS_1, lis->threshold);
	__reg_write(lis, LIS302DL_REG_FF_WU_DURATION_1, lis->duration);

	/* Clear the HP filter "starting point" */
	__reg_read(lis, LIS302DL_REG_HP_FILTER_RESET);
	__reg_write(lis, LIS302DL_REG_FF_WU_CFG_1, LIS302DL_FFWUCFG_XHIE |
			LIS302DL_FFWUCFG_YHIE |	LIS302DL_FFWUCFG_ZHIE);

	lis->flags |= LIS302DL_F_INPUT_OPEN;

	/* kick it off -- since we are edge triggered, if we missed the edge
	 * permanent low interrupt is death for us */
	lis302dl_bitbang_read_sample(lis);

	local_irq_restore(flags);

	return 0;
}

static void lis302dl_input_close(struct input_dev *inp)
{
	struct lis302dl_info *lis = inp->private;
	u_int8_t ctrl1 = LIS302DL_CTRL1_Xen | LIS302DL_CTRL1_Yen |
			 LIS302DL_CTRL1_Zen;
	unsigned long flags;

	local_irq_save(flags);

	/* since the input core already serializes access and makes sure we
	 * only see close() for the close of the last user, we can safely
	 * disable the data ready events */
	__reg_set_bit_mask(lis, LIS302DL_REG_CTRL1, ctrl1, 0x00);
	lis->flags &= ~LIS302DL_F_INPUT_OPEN;

	/* however, don't power down the whole device if still needed */
	if (!(lis->flags & LIS302DL_F_WUP_FF ||
	      lis->flags & LIS302DL_F_WUP_CLICK)) {
		__reg_set_bit_mask(lis, LIS302DL_REG_CTRL1, LIS302DL_CTRL1_PD,
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

	__reg_write(lis, LIS302DL_REG_CTRL2,
			LIS302DL_CTRL2_BOOT | LIS302DL_CTRL2_FDS);

	while ((__reg_read(lis, LIS302DL_REG_CTRL2)
			& LIS302DL_CTRL2_BOOT) && (timeout--))
		mdelay(1);

	return !!(timeout < 0);
}

static int __devinit lis302dl_probe(struct platform_device *pdev)
{
	int rc;
	struct lis302dl_info *lis;
	u_int8_t wai;
	unsigned long flags;
	struct lis302dl_platform_data *pdata = pdev->dev.platform_data;

	lis = kzalloc(sizeof(*lis), GFP_KERNEL);
	if (!lis)
		return -ENOMEM;

	local_irq_save(flags);

	lis->dev = &pdev->dev;

	dev_set_drvdata(lis->dev, lis);

	lis->pdata = pdata;

	/* Configure our IO */
	(lis->pdata->lis302dl_suspend_io)(lis, 1);

	wai = __reg_read(lis, LIS302DL_REG_WHO_AM_I);
	if (wai != LIS302DL_WHO_AM_I_MAGIC) {
		dev_err(lis->dev, "unknown who_am_i signature 0x%02x\n", wai);
		dev_set_drvdata(lis->dev, NULL);
		rc = -ENODEV;
		goto bail_free_lis;
	}

	rc = sysfs_create_group(&lis->dev->kobj, &lis302dl_attr_group);
	if (rc) {
		dev_err(lis->dev, "error creating sysfs group\n");
		goto bail_free_lis;
	}

	/* initialize input layer details */
	lis->input_dev = input_allocate_device();
	if (!lis->input_dev) {
		dev_err(lis->dev, "Unable to allocate input device\n");
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
	lis->threshold = 1;
	lis->duration = 0;

	lis->input_dev->private = lis;
	lis->input_dev->name = pdata->name;
	 /* SPI Bus not defined as a valid bus for input subsystem*/
	lis->input_dev->id.bustype = BUS_I2C; /* lie about it */
	lis->input_dev->open = lis302dl_input_open;
	lis->input_dev->close = lis302dl_input_close;

	rc = input_register_device(lis->input_dev);
	if (rc) {
		dev_err(lis->dev, "error %d registering input device\n", rc);
		goto bail_inp_dev;
	}

	if (__lis302dl_reset_device(lis))
		dev_err(lis->dev, "device BOOT reload failed\n");

	/* force us powered */
	__reg_write(lis, LIS302DL_REG_CTRL1, LIS302DL_CTRL1_PD |
			LIS302DL_CTRL1_Xen |
			LIS302DL_CTRL1_Yen |
			LIS302DL_CTRL1_Zen);
	mdelay(1);

	__reg_write(lis, LIS302DL_REG_CTRL2, 0);
	__reg_write(lis, LIS302DL_REG_CTRL3,
			LIS302DL_CTRL3_PP_OD | LIS302DL_CTRL3_IHL);
	__reg_write(lis, LIS302DL_REG_FF_WU_THS_1, 0x0);
	__reg_write(lis, LIS302DL_REG_FF_WU_DURATION_1, 0x00);
	__reg_write(lis, LIS302DL_REG_FF_WU_CFG_1, 0x0);

	/* start off in powered down mode; we power up when someone opens us */
	__reg_write(lis, LIS302DL_REG_CTRL1, LIS302DL_CTRL1_Xen |
			LIS302DL_CTRL1_Yen | LIS302DL_CTRL1_Zen);

	if (pdata->open_drain)
		/* switch interrupt to open collector, active-low */
		__reg_write(lis, LIS302DL_REG_CTRL3,
				LIS302DL_CTRL3_PP_OD | LIS302DL_CTRL3_IHL);
	else
		/* push-pull, active-low */
		__reg_write(lis, LIS302DL_REG_CTRL3, LIS302DL_CTRL3_IHL);

	__lis302dl_int_mode(lis->dev, 1, LIS302DL_INTMODE_FF_WU_12);
	__lis302dl_int_mode(lis->dev, 2, LIS302DL_INTMODE_FF_WU_12);

	__reg_read(lis, LIS302DL_REG_STATUS);
	__reg_read(lis, LIS302DL_REG_FF_WU_SRC_1);
	__reg_read(lis, LIS302DL_REG_FF_WU_SRC_2);
	__reg_read(lis, LIS302DL_REG_CLICK_SRC);

	dev_info(lis->dev, "Found %s\n", pdata->name);

	lis->pdata = pdata;

	rc = request_irq(pdata->interrupt, lis302dl_interrupt,
			 IRQF_TRIGGER_FALLING, "lis302dl", lis);
	if (rc < 0) {
		dev_err(lis->dev, "error requesting IRQ %d\n",
			lis->pdata->interrupt);
		goto bail_inp_reg;
	}
	local_irq_restore(flags);
	return 0;

bail_inp_reg:
	input_unregister_device(lis->input_dev);
bail_inp_dev:
	input_free_device(lis->input_dev);
bail_sysfs:
	sysfs_remove_group(&lis->dev->kobj, &lis302dl_attr_group);
bail_free_lis:
	kfree(lis);
	local_irq_restore(flags);
	return rc;
}

static int __devexit lis302dl_remove(struct platform_device *pdev)
{
	struct lis302dl_info *lis = dev_get_drvdata(&pdev->dev);
	unsigned long flags;

	/* Reset and power down the device */
	local_irq_save(flags);
	__reg_write(lis, LIS302DL_REG_CTRL3, 0x00);
	__reg_write(lis, LIS302DL_REG_CTRL2, 0x00);
	__reg_write(lis, LIS302DL_REG_CTRL1, 0x00);
	local_irq_restore(flags);

	/* Cleanup resources */
	free_irq(lis->pdata->interrupt, lis);
	sysfs_remove_group(&pdev->dev.kobj, &lis302dl_attr_group);
	input_unregister_device(lis->input_dev);
	if (lis->input_dev)
		input_free_device(lis->input_dev);
	dev_set_drvdata(lis->dev, NULL);
	kfree(lis);

	return 0;
}

#ifdef CONFIG_PM

static u8 regs_to_save[] = {
	LIS302DL_REG_CTRL1,
	LIS302DL_REG_CTRL2,
	LIS302DL_REG_CTRL3,
	LIS302DL_REG_FF_WU_CFG_1,
	LIS302DL_REG_FF_WU_THS_1,
	LIS302DL_REG_FF_WU_DURATION_1,
	LIS302DL_REG_FF_WU_CFG_2,
	LIS302DL_REG_FF_WU_THS_2,
	LIS302DL_REG_FF_WU_DURATION_2,
	LIS302DL_REG_CLICK_CFG,
	LIS302DL_REG_CLICK_THSY_X,
	LIS302DL_REG_CLICK_THSZ,
	LIS302DL_REG_CLICK_TIME_LIMIT,
	LIS302DL_REG_CLICK_LATENCY,
	LIS302DL_REG_CLICK_WINDOW,

};

static int lis302dl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct lis302dl_info *lis = dev_get_drvdata(&pdev->dev);
	unsigned long flags;
	u_int8_t tmp;
	int n;

	/* determine if we want to wake up from the accel. */
	if (lis->flags & LIS302DL_F_WUP_FF ||
		lis->flags & LIS302DL_F_WUP_CLICK)
		return 0;

	disable_irq(lis->pdata->interrupt);
	local_irq_save(flags);

	/*
	 * When we share SPI over multiple sensors, there is a race here
	 * that one or more sensors will lose.  In that case, the shared
	 * SPI bus GPIO will be in sleep mode and partially pulled down.  So
	 * we explicitly put our IO into "wake" mode here before the final
	 * traffic to the sensor.
	 */
	(lis->pdata->lis302dl_suspend_io)(lis, 1);

	/* save registers */
	for (n = 0; n < ARRAY_SIZE(regs_to_save); n++)
		lis->regs[regs_to_save[n]] =
			__reg_read(lis, regs_to_save[n]);

	/* power down */
	tmp = __reg_read(lis, LIS302DL_REG_CTRL1);
	tmp &= ~LIS302DL_CTRL1_PD;
	__reg_write(lis, LIS302DL_REG_CTRL1, tmp);

	/* place our IO to the device in sleep-compatible states */
	(lis->pdata->lis302dl_suspend_io)(lis, 0);

	local_irq_restore(flags);

	return 0;
}

static int lis302dl_resume(struct platform_device *pdev)
{
	struct lis302dl_info *lis = dev_get_drvdata(&pdev->dev);
	unsigned long flags;
	int n;

	if (lis->flags & LIS302DL_F_WUP_FF ||
		lis->flags & LIS302DL_F_WUP_CLICK)
		return 0;

	local_irq_save(flags);

	/* get our IO to the device back in operational states */
	(lis->pdata->lis302dl_suspend_io)(lis, 1);

	/* resume from powerdown first! */
	__reg_write(lis, LIS302DL_REG_CTRL1,
			LIS302DL_CTRL1_PD |
			LIS302DL_CTRL1_Xen |
			LIS302DL_CTRL1_Yen |
			LIS302DL_CTRL1_Zen);
	mdelay(1);

	if (__lis302dl_reset_device(lis))
		dev_err(&pdev->dev, "device BOOT reload failed\n");

	lis->regs[LIS302DL_REG_CTRL1] |=	LIS302DL_CTRL1_PD |
						LIS302DL_CTRL1_Xen |
						LIS302DL_CTRL1_Yen |
						LIS302DL_CTRL1_Zen;

	/* restore registers after resume */
	for (n = 0; n < ARRAY_SIZE(regs_to_save); n++)
		__reg_write(lis, regs_to_save[n], lis->regs[regs_to_save[n]]);

	local_irq_restore(flags);
	enable_irq(lis->pdata->interrupt);

	return 0;
}
#else
#define lis302dl_suspend	NULL
#define lis302dl_resume		NULL
#endif

static struct platform_driver lis302dl_driver = {
	.driver = {
		.name	= "lis302dl",
		.owner	= THIS_MODULE,
	},

	.probe	 = lis302dl_probe,
	.remove	 = __devexit_p(lis302dl_remove),
	.suspend = lis302dl_suspend,
	.resume	 = lis302dl_resume,
};

static int __devinit lis302dl_init(void)
{
	return platform_driver_register(&lis302dl_driver);
}

static void __exit lis302dl_exit(void)
{
	platform_driver_unregister(&lis302dl_driver);
}

MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_LICENSE("GPL");

module_init(lis302dl_init);
module_exit(lis302dl_exit);
