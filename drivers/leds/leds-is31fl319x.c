/*
 * Copyright 2015-16 Golden Delicious Computers
 *
 * Author: Nikolaus Schaller <hns@goldelico.com>
 *
 * Based on leds-tca6507.c
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * LED driver for the IS31FL3191/3/6/99 to drive 1, 3, 6 or 9 light
 * effect LEDs.
 *
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/slab.h>

/* register numbers */
#define IS31FL319X_SHUTDOWN	0x00
#define IS31FL319X_CTRL1	0x01
#define IS31FL319X_CTRL2	0x02
#define IS31FL319X_CONFIG1	0x03
#define IS31FL319X_CONFIG2	0x04
#define IS31FL319X_RAMP_MODE	0x05
#define IS31FL319X_BREATH_MASK	0x06
#define IS31FL319X_PWM1		0x07
#define IS31FL319X_PWM2		0x08
#define IS31FL319X_PWM3		0x09
#define IS31FL319X_PWM4		0x0a
#define IS31FL319X_PWM5		0x0b
#define IS31FL319X_PWM6		0x0c
#define IS31FL319X_PWM7		0x0d
#define IS31FL319X_PWM8		0x0e
#define IS31FL319X_PWM9		0x0f
#define IS31FL319X_DATA_UPDATE	0x10
#define IS31FL319X_T0_1		0x11
#define IS31FL319X_T0_2		0x12
#define IS31FL319X_T0_3		0x13
#define IS31FL319X_T0_4		0x14
#define IS31FL319X_T0_5		0x15
#define IS31FL319X_T0_6		0x16
#define IS31FL319X_T0_7		0x17
#define IS31FL319X_T0_8		0x18
#define IS31FL319X_T0_9		0x19
#define IS31FL319X_T123_1	0x1a
#define IS31FL319X_T123_2	0x1b
#define IS31FL319X_T123_3	0x1c
#define IS31FL319X_T4_1		0x1d
#define IS31FL319X_T4_2		0x1e
#define IS31FL319X_T4_3		0x1f
#define IS31FL319X_T4_4		0x20
#define IS31FL319X_T4_5		0x21
#define IS31FL319X_T4_6		0x22
#define IS31FL319X_T4_7		0x23
#define IS31FL319X_T4_8		0x24
#define IS31FL319X_T4_9		0x25
#define IS31FL319X_TIME_UPDATE	0x26
#define IS31FL319X_RESET	0xff

#define IS31FL319X_REG_CNT	(IS31FL319X_RESET + 1)

#define NUM_LEDS 9	/* max for 3199 chip */

struct is31fl319x_chip {
	struct i2c_client	*client;
	struct regmap		*regmap;

	struct is31fl319x_led {
		struct is31fl319x_chip	*chip;
		struct led_classdev	led_cdev;
	} leds[NUM_LEDS];
};

static const struct i2c_device_id is31fl319x_id[] = {
	{ "is31fl3190", 1 },
	{ "is31fl3191", 1 },
	{ "is31fl3193", 3 },
	{ "is31fl3196", 6 },
	{ "is31fl3199", 9 },
	{ "sn3199", 9 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, is31fl319x_id);

static int is31fl319x_brightness_set(struct led_classdev *led_cdev,
				   enum led_brightness brightness)
{
	struct is31fl319x_led *led = container_of(led_cdev,
						  struct is31fl319x_led,
						  led_cdev);
	struct is31fl319x_chip *is31 = led->chip;
	int ret;

	int i;
	u8 ctrl1, ctrl2;

	dev_dbg(&is31->client->dev, "%s %d: %d\n", __func__, (led - is31->leds),
		brightness);

	/* update PWM register */
	ret = regmap_write(is31->regmap, IS31FL319X_PWM1 + (led - is31->leds),
			   brightness);
	if (ret < 0)
		return ret;

	ctrl1 = 0;
	ctrl2 = 0;

	/* read current brightness of all PWM channels */
	for (i = 0; i < NUM_LEDS; i++) {
		unsigned int pwm_value;
		bool on;

		/*
		 * since neither led_cdev nor the chip can provide
		 * the current setting, we read from the regmap cache
		 */

		ret = regmap_read(is31->regmap, IS31FL319X_PWM1 + i,
				  &pwm_value);
		dev_dbg(&is31->client->dev, "%s read %d: ret=%d: %d\n",
			__func__, i, ret, pwm_value);
		on = ret >= 0 && pwm_value > LED_OFF;

		if (i < 3)
			ctrl1 |= on << i;	/* 0..2 => bit 0..2 */
		else if (i < 6)
			ctrl1 |= on << (i+1);	/* 3..5 => bit 4..6 */
		else
			ctrl2 |= on << (i-6);	/* 6..8 => bit 0..2 */
	}

	if (ctrl1 > 0 || ctrl2 > 0) {
		dev_dbg(&is31->client->dev, "power up %02x %02x\n",
			ctrl1, ctrl2);
		regmap_write(is31->regmap, IS31FL319X_CTRL1, ctrl1);
		regmap_write(is31->regmap, IS31FL319X_CTRL2, ctrl2);
		/* update PWMs */
		regmap_write(is31->regmap, IS31FL319X_DATA_UPDATE, 0x00);
		/* enable chip from shut down */
		ret = regmap_write(is31->regmap, IS31FL319X_SHUTDOWN, 0x01);
	} else {
		dev_dbg(&is31->client->dev, "power down\n");
		/* shut down (no need to clear CTRL1/2) */
		ret = regmap_write(is31->regmap, IS31FL319X_SHUTDOWN, 0x00);
	}

	return ret;
}

static struct led_info *
is31fl319x_parse_dt(struct i2c_client *client, int num_leds)
{
	struct device_node *np = client->dev.of_node, *child;
	struct led_info *is31_leds;
	int count;

	if (!np)
		return ERR_PTR(-ENODEV);

	count = of_get_child_count(np);
	dev_dbg(&client->dev, "child count %d\n", count);
	if (!count || count > NUM_LEDS)
		return ERR_PTR(-ENODEV);

	is31_leds = devm_kzalloc(&client->dev,
			sizeof(struct led_info) * NUM_LEDS, GFP_KERNEL);
	if (!is31_leds)
		return ERR_PTR(-ENOMEM);

	for_each_child_of_node(np, child) {
		struct led_info led;
		u32 reg;
		int ret;

		led.name = NULL;
		ret = of_property_read_string(child, "label", &led.name);
		if (ret < 0 && ret != -EINVAL)	/* is optional */
			return ERR_PTR(ret);
		led.default_trigger = NULL;
		ret = of_property_read_string(child, "linux,default-trigger",
			&led.default_trigger);
		if (ret < 0 && ret != -EINVAL)	/* is optional */
			return ERR_PTR(ret);
		led.flags = 0;
		ret = of_property_read_u32(child, "reg", &reg);
		dev_dbg(&client->dev, "name = %s reg = %d\n", led.name, reg);
		reg -= 1;	/* index 0 is at reg = 1 */
		if (ret != 0 || reg < 0 || reg >= num_leds)
			continue;

		if (is31_leds[reg].name)
			dev_err(&client->dev, "duplicate led line %d for %s -> %s\n",
				reg, is31_leds[reg].name, led.name);
		is31_leds[reg] = led;
	}

	return is31_leds;
}

static const struct of_device_id of_is31fl319x_leds_match[] = {
	{ .compatible = "issi,is31fl3190", (void *) 1 },
	{ .compatible = "issi,is31fl3191", (void *) 1 },
	{ .compatible = "issi,is31fl3193", (void *) 3 },
	{ .compatible = "issi,is31fl3196", (void *) 6 },
	{ .compatible = "issi,is31fl3199", (void *) 9 },
	{ .compatible = "si-en,sn3199", (void *) 9 },
	{},
};
MODULE_DEVICE_TABLE(of, of_is31fl319x_leds_match);

static bool is31fl319x_readable_reg(struct device *dev, unsigned int reg)
{ /* we have no readable registers */
	return false;
}

static bool is31fl319x_volatile_reg(struct device *dev, unsigned int reg)
{ /* volatile registers are not cached */
	switch (reg) {
	case IS31FL319X_DATA_UPDATE:
	case IS31FL319X_TIME_UPDATE:
	case IS31FL319X_RESET:
		return true;	/* always write-through */
	default:
		return false;
	}
}

struct regmap_config regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = IS31FL319X_REG_CNT,
	.cache_type = REGCACHE_FLAT,
	.readable_reg = is31fl319x_readable_reg,
	.volatile_reg = is31fl319x_volatile_reg,
};

static int is31fl319x_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct is31fl319x_chip *is31;
	struct i2c_adapter *adapter;
	struct led_info *leds;
	int err;
	int i = 0;

	adapter = to_i2c_adapter(client->dev.parent);

	dev_dbg(&client->dev, "probe IS31FL319x for num_leds = %d\n",
		(int) id->driver_data);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	leds = is31fl319x_parse_dt(client, (int) id->driver_data);
	if (IS_ERR(leds)) {
		dev_err(&client->dev, "DT parsing error %d\n",
			(int) PTR_ERR(leds));
		return PTR_ERR(leds);
	}

	is31 = devm_kzalloc(&client->dev, sizeof(*is31), GFP_KERNEL);
	if (!is31)
		return -ENOMEM;

	is31->client = client;
	is31->regmap = devm_regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(is31->regmap)) {
		dev_err(&client->dev, "failed to allocate register map\n");
		return PTR_ERR(is31->regmap);
	}

	i2c_set_clientdata(client, is31);

	/* check for write-reply from chip (we can't read any registers) */
	err = regmap_write(is31->regmap, IS31FL319X_RESET, 0x00);
	if (err < 0) {
		dev_err(&client->dev, "no response from chip write: err = %d\n",
			err);
		return -EIO;	/* does not answer */
	}

	/* initialize chip and regmap so that we never try to read from i2c */
	regmap_write(is31->regmap, IS31FL319X_CTRL1, 0x00);
	regmap_write(is31->regmap, IS31FL319X_CTRL2, 0x00);
	for (i = 0; i < NUM_LEDS; i++)
		regmap_write(is31->regmap, IS31FL319X_PWM1 + i, 0x00);

	if (client->dev.of_node) {
		u32 val;
		u8 config2 = 0;

		if (of_property_read_u32(client->dev.of_node,
					 "led-max-microamp", &val)) {
			if (val > 40000)
				val = 40000;
			if (val < 5000)
				val = 5000;
			config2 |= (((64000 - val) / 5000) & 0x7) << 4; /* CS */
		}
		if (of_property_read_u32(client->dev.of_node, "audio-gain-db",
					 &val)) {
			if (val > 21)
				val = 21;
			config2 |= val / 3; /* AGS */
		}
		regmap_write(is31->regmap, IS31FL319X_CONFIG2, config2);
	}

	for (i = 0; i < NUM_LEDS; i++) {
		struct is31fl319x_led *l = is31->leds + i;

		l->chip = is31;
		if (leds[i].name && !leds[i].flags) {
			l->led_cdev.name = leds[i].name;
			l->led_cdev.default_trigger
				= leds[i].default_trigger;
			l->led_cdev.brightness_set_blocking
				= is31fl319x_brightness_set;
			/* NOTE: is31fl319x_brightness_set will be called
			 * immediately after register() before we return
			 */
			err = devm_led_classdev_register(&client->dev,
						    &l->led_cdev);
			if (err < 0)
				return err;
		}
	}

	dev_dbg(&client->dev, "probed\n");
	return 0;
}

static struct i2c_driver is31fl319x_driver = {
	.driver   = {
		.name    = "leds-is31fl319x",
		.of_match_table = of_match_ptr(of_is31fl319x_leds_match),
	},
	.probe    = is31fl319x_probe,
	.id_table = is31fl319x_id,
};

module_i2c_driver(is31fl319x_driver);

MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("IS31FL319X LED driver");
MODULE_LICENSE("GPL v2");
