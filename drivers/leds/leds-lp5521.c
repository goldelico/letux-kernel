/* NS LP5521 Programmable LED driver.
 *
 * (C) 2009 by Openmoko, Inc.
 * Author: Matt Hsu <matt_hsu@openmoko.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <linux/lp5521.h>

#define LP5521_DRIVER_NAME              "lp5521"

static int __lp5521_reg_write(struct lp5521 *lp, u8 reg, u8 value)
{
	return i2c_smbus_write_byte_data(lp->client, reg, value);
}

static int reg_write(struct lp5521 *lp, u_int8_t reg, u_int8_t val)
{
	int ret;

	mutex_lock(&lp->lock);
	ret = __lp5521_reg_write(lp, reg, val);
	mutex_unlock(&lp->lock);

	return ret;
}

static int __lp5521_reg_read(struct lp5521 *lp, u8 reg)
{
	int32_t ret;

	ret = i2c_smbus_read_byte_data(lp->client, reg);

	return ret;
}

static u_int8_t reg_read(struct lp5521 *lp, u_int8_t reg)
{
	int32_t ret;

	mutex_lock(&lp->lock);
	ret = __lp5521_reg_read(lp, reg);
	mutex_unlock(&lp->lock);

	return ret & 0xff;
}

static int reg_set_bit_mask(struct lp5521 *lp,
			    u_int8_t reg, u_int8_t mask, u_int8_t val)
{
	int ret;
	u_int8_t tmp;

	val &= mask;

	mutex_lock(&lp->lock);

	tmp = __lp5521_reg_read(lp, reg);
	tmp &= ~mask;
	tmp |= val;
	ret = __lp5521_reg_write(lp, reg, tmp);

	mutex_unlock(&lp->lock);

	return ret;
}

static const char *lp5521_ch_name[] = {
	"blue", "green", "red",
};

static inline int channel_id_by_name(const char *name)
{
	int channel_id = -1;

	if (!strncmp(name, lp5521_ch_name[LP5521_BLUE],
				strlen(lp5521_ch_name[LP5521_BLUE])))
		channel_id = LP5521_BLUE;
	else if (!strncmp(name, lp5521_ch_name[LP5521_GREEN],
				strlen(lp5521_ch_name[LP5521_GREEN])))
		channel_id = LP5521_GREEN;
	else if (!strncmp(name, lp5521_ch_name[LP5521_RED],
				strlen(lp5521_ch_name[LP5521_RED])))
		channel_id = LP5521_RED;

	return channel_id;
}

static const char *lp5521_ch_mode[] = {
	"disable", "load", "run",
	"direct",
};

/*
 * Individual mode control
 */
static ssize_t show_mode(struct device *dev, struct device_attribute
					*attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lp5521 *lp = i2c_get_clientdata(client);
	int id;
	uint8_t val;

	id = channel_id_by_name(attr->attr.name);
	val = reg_read(lp, LP5521_REG_OP_MODE);

	val = val >> (id * 2);
	val &= 0x3;

	return sprintf(buf, "%s\n", lp5521_ch_mode[val]);
}

static ssize_t set_mode(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lp5521 *lp = i2c_get_clientdata(client);
	int id;
	uint8_t mask, i;

	id = channel_id_by_name(attr->attr.name);

	mask = (0x3 << (id * 2));

	for (i = LP5521_REG_OP_MODE; i <= LP5521_MODE_DIRECT; i++) {
		if (!strncmp(buf, lp5521_ch_mode[i], strlen(lp5521_ch_mode[i]))) {
			reg_set_bit_mask(lp,
				LP5521_REG_OP_MODE, mask, (i << (id * 2)));
		}
	}

	return count;
}

static DEVICE_ATTR(red_mode, S_IRUGO | S_IWUSR, show_mode, set_mode);
static DEVICE_ATTR(green_mode, S_IRUGO | S_IWUSR, show_mode, set_mode);
static DEVICE_ATTR(blue_mode, S_IRUGO | S_IWUSR, show_mode, set_mode);

/*
 * Individual pwm control
 */
static ssize_t show_pwm(struct device *dev, struct device_attribute
					*attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lp5521 *lp = i2c_get_clientdata(client);
	int id;
	uint8_t val;

	id = channel_id_by_name(attr->attr.name);
	val = reg_read(lp, LP5521_REG_B_PWM-id);

	return sprintf(buf, "%d\n", val);
}

static ssize_t set_pwm(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int id;
	struct i2c_client *client = to_i2c_client(dev);
	struct lp5521 *lp = i2c_get_clientdata(client);
	unsigned int pwm = simple_strtoul(buf, NULL, 10);

	id = channel_id_by_name(attr->attr.name);
	reg_write(lp, LP5521_REG_B_PWM-id, pwm);

	return count;
}

static DEVICE_ATTR(red_pwm, S_IRUGO | S_IWUSR, show_pwm, set_pwm);
static DEVICE_ATTR(green_pwm, S_IRUGO | S_IWUSR, show_pwm, set_pwm);
static DEVICE_ATTR(blue_pwm, S_IRUGO | S_IWUSR, show_pwm, set_pwm);

/*
 * Individual current control
 */
static ssize_t show_cur(struct device *dev, struct device_attribute
					*attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lp5521 *lp = i2c_get_clientdata(client);
	int id;
	uint8_t val;

	id = channel_id_by_name(attr->attr.name);
	val = reg_read(lp, LP5521_REG_B_CUR-id);

	return sprintf(buf, "%d (100uA)\n", val);
}

static ssize_t set_cur(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int id;
	struct i2c_client *client = to_i2c_client(dev);
	struct lp5521 *lp = i2c_get_clientdata(client);
	unsigned int cur = simple_strtoul(buf, NULL, 10);

	id = channel_id_by_name(attr->attr.name);
	reg_write(lp, LP5521_REG_B_CUR-id, cur);

	return count;
}

static DEVICE_ATTR(red_cur, S_IRUGO | S_IWUSR, show_cur, set_cur);
static DEVICE_ATTR(green_cur, S_IRUGO | S_IWUSR, show_cur, set_cur);
static DEVICE_ATTR(blue_cur, S_IRUGO | S_IWUSR, show_cur, set_cur);

static struct attribute *lp_sysfs_entries[16];

static struct attribute_group lp_attr_group = {
	.name   = NULL,
	.attrs  = lp_sysfs_entries,
};

static void populate_sysfs_group(struct lp5521 *lp)
{
	int i = 0;

	if (lp->pdata->channels[LP5521_RED] & LP5521_CONNECTED) {
		lp_sysfs_entries[i++] = &dev_attr_red_mode.attr;
		lp_sysfs_entries[i++] = &dev_attr_red_pwm.attr;
		lp_sysfs_entries[i++] = &dev_attr_red_cur.attr;
	}

	if (lp->pdata->channels[LP5521_GREEN] & LP5521_CONNECTED) {
		lp_sysfs_entries[i++] = &dev_attr_green_mode.attr;
		lp_sysfs_entries[i++] = &dev_attr_green_pwm.attr;
		lp_sysfs_entries[i++] = &dev_attr_green_cur.attr;
	}

	if (lp->pdata->channels[LP5521_BLUE] & LP5521_CONNECTED) {
		lp_sysfs_entries[i++] = &dev_attr_blue_mode.attr;
		lp_sysfs_entries[i++] = &dev_attr_blue_pwm.attr;
		lp_sysfs_entries[i++] = &dev_attr_blue_cur.attr;
	}
}

static struct i2c_driver lp5521_driver;

#ifdef CONFIG_PM
static int lp5521_suspend(struct device *dev, pm_message_t state)
{
	/* FIXME: Not implemented
	 * Here we could upload firmware to perform
	 * any scenarios we want and save registers.
	 */
	return 0;
}

static int lp5521_resume(struct device *dev)
{
	/* FIXME: Not implemented */
	return 0;
}
#else
#define lp5521_suspend NULL
#define lp5521_resume NULL
#endif

static irqreturn_t lp5521_irq(int irq, void *_lp)
{
	struct lp5521 *lp = _lp;
	dev_info(lp->dev, "lp5521 interrupt\n");

	return IRQ_HANDLED;
}

static int __devinit
lp5521_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lp5521 *lp;
	int ret = 0;
	struct lp5521_platform_data *pdata = client->dev.platform_data;

	lp = kzalloc(sizeof(*lp), GFP_KERNEL);
	if (!lp)
		return -ENOMEM;

	lp->client = client;
	lp->irq = client->irq;
	lp->dev = &client->dev;
	i2c_set_clientdata(client, lp);

	lp->pdata = pdata;
	mutex_init(&lp->lock);

	/* enter start-up mode */
	if (pdata->ext_enable)
		(pdata->ext_enable)(1);

	reg_write(lp, LP5521_REG_ENABLE, 0x40);

	/* charge pump mode and clk src selection */
	reg_write(lp, LP5521_REG_CONFIG, 0x11);

	/* allocate IRQ resource */
	if (lp->irq) {
		ret = request_irq(client->irq, lp5521_irq,
				IRQF_TRIGGER_LOW, LP5521_DRIVER_NAME, lp);
		if (ret) {
			dev_err(lp->dev, "request IRQ failed\n");
			goto fail;
		}
	} else {
		dev_err(lp->dev, "No IRQ allocated \n");
	}

	populate_sysfs_group(lp);

	ret = sysfs_create_group(&client->dev.kobj, &lp_attr_group);

	if (ret) {
		dev_err(lp->dev, "error creating sysfs group\n");
		goto fail;
	}

	return ret;

fail:
	kfree(lp);
	return ret;
}

static int __devexit lp5521_remove(struct i2c_client *client)
{
	struct lp5521 *lp = i2c_get_clientdata(client);

	kfree(lp);

	return 0;
}

static struct i2c_device_id lp5521_id[] = {
	{LP5521_DRIVER_NAME, },
};

static struct i2c_driver lp5521_driver = {
	.driver = {
		.name	= LP5521_DRIVER_NAME,
		.suspend = lp5521_suspend,
		.resume	= lp5521_resume,
	},
	.id_table 	= lp5521_id,
	.probe		= lp5521_probe,
	.remove		= __exit_p(lp5521_remove),
};

static int __init lp5521_init(void)
{
	return i2c_add_driver(&lp5521_driver);
}

static void __exit lp5521_exit(void)
{
	i2c_del_driver(&lp5521_driver);
}

MODULE_AUTHOR("Matt Hsu <matt_hsu@openmoko.org>");
MODULE_DESCRIPTION("NS lp5521 LED driver");
MODULE_LICENSE("GPLv2");

module_init(lp5521_init);
module_exit(lp5521_exit);
