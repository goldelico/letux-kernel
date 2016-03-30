/*
 * Copyright 2015 Golden Delcious Computers
 *
 * Author: Nikolaus Schaller <hns@goldelico.com>
 *
 * Based on leds-tca6507.c
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * LED driver for the IS31FL3196/99 to drive 6 or 9 light effect LEDs.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/leds-is31fl319x.h>
#include <linux/of.h>

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

#define	IS31FL319X_REG_CNT	(IS31FL319X_TIME_UPDATE + 1)

#define NUM_LEDS 9	/* max for 3199 chip */

struct is31fl319x_chip {
	struct i2c_client	*client;
	struct work_struct	work;
	bool			work_scheduled;
	spinlock_t		lock;
	u8			reg_file[IS31FL319X_REG_CNT];

	struct is31fl319x_led {
		struct is31fl319x_chip	*chip;
		struct led_classdev	led_cdev;
	} leds[NUM_LEDS];
};

static const struct i2c_device_id is31fl319x_id[] = {
	{ "is31fl3196", 6 },
	{ "is31fl3196", 9 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, is31fl319x_id);


static int is31fl319x_write(struct is31fl319x_chip *is31, u8 reg, u8 byte)
{
	struct i2c_client *cl = is31->client;
	if (reg >= IS31FL319X_REG_CNT)
		return -EIO;
	is31->reg_file[reg] = byte;	/* save in cache */
	dev_dbg(&is31->client->dev, "write %02x to reg %02x\n", byte, reg);
	return i2c_smbus_write_byte_data(cl, reg, byte);
}

static int is31fl319x_read(struct is31fl319x_chip *is31, u8 reg)
{
	if (reg >= IS31FL319X_REG_CNT)
		return -EIO;
	return is31->reg_file[reg];	/* read crom cache (can't read chip) */
}

#ifdef UNUSED
static int is31fl319x_clear_set(struct is31fl319x_chip *is31, u8 reg, u8 clear, u8 set)
{
	struct i2c_client *cl = is31->client;
	u8 byte;
	if (reg >= IS31FL319X_REG_CNT)
		return -EIO;
	byte = (is31->reg_file[reg] & (~clear)) | set;	/* new bit pattern */
	if (byte == is31->reg_file[reg])
		return 0;	/* no change */
	is31->reg_file[reg] = byte;	/* save in cache */
	dev_dbg(&is31->client->dev, "write %02x to reg %02x\n", byte, reg);
	return i2c_smbus_write_byte_data(cl, reg, byte);
}
#endif

static void is31fl319x_work(struct work_struct *work)
{
	struct is31fl319x_chip *is31 = container_of(work, struct is31fl319x_chip,
						work);
	unsigned long flags;
	int i;
	u8 ctrl1, ctrl2;

	dev_dbg(&is31->client->dev, "work called\n");

	spin_lock_irqsave(&is31->lock, flags);
	is31->work_scheduled = false;	/* subsequent changes run another schedule_work */
	spin_unlock_irqrestore(&is31->lock, flags);

	dev_dbg(&is31->client->dev, "write to chip\n");

	ctrl1 = 0;
	ctrl2 = 0;

	for (i = 0; i < NUM_LEDS; i++) {
		struct led_classdev *led = &is31->leds[i].led_cdev;
		bool on;
		
		if (!is31->leds[i].led_cdev.name)
			continue;

		dev_dbg(&is31->client->dev, "set brightness %u for led %u\n", led->brightness, i);

		/* update brightness register */
		is31fl319x_write(is31, IS31FL319X_PWM1 + i, led->brightness);

		/* update output enable bits */
		on = led->brightness > LED_OFF;
		if (i < 3)
			ctrl1 |= on << i;	/* 0..2 => bit 0..2 */
		else if (i < 6)
			ctrl1 |= on << (i+1);	/* 3..5 => bit 4..6 */
		else
			ctrl2 |= on << (i-6);	/* 6..8 => bit 0..2 */
	}

	/* check if any PWM is enabled or all outputs are now off */
	if (ctrl1 > 0 || ctrl2 > 0) {
		dev_dbg(&is31->client->dev, "power up\n");
		is31fl319x_write(is31, IS31FL319X_CTRL1, ctrl1);
		is31fl319x_write(is31, IS31FL319X_CTRL2, ctrl2);
		is31fl319x_write(is31, IS31FL319X_DATA_UPDATE, 0x00);	/* update */
		is31fl319x_write(is31, IS31FL319X_SHUTDOWN, 0x01);	/* enable */
	} else {
		dev_dbg(&is31->client->dev, "power down\n");
		is31fl319x_write(is31, IS31FL319X_SHUTDOWN, 0x00);	/* shut down */
	}

	dev_dbg(&is31->client->dev, "work done\n");

}

/* NOTE: this may be called from within irq context, e.g. led_trigger_event */

static int is31fl319x_brightness_get(struct led_classdev *led_cdev)
{
	struct is31fl319x_led *led = container_of(led_cdev, struct is31fl319x_led,
					       led_cdev);
	struct is31fl319x_chip *is31 = led->chip;

	/* read PWM register */
	return is31fl319x_read(is31, IS31FL319X_PWM1 + (led - is31->leds));
}

static void is31fl319x_brightness_set(struct led_classdev *led_cdev,
				   enum led_brightness brightness)
{
	struct is31fl319x_led *led = container_of(led_cdev, struct is31fl319x_led,
					       led_cdev);
	struct is31fl319x_chip *is31 = led->chip;
	unsigned long flags;
// printk("is31fl319x_brightness_set %u: %u->%u (sch: %u)\n", led-is31->leds, led->led_cdev.brightness, brightness, is31->work_scheduled);

	spin_lock_irqsave(&is31->lock, flags);

	if (brightness != is31fl319x_brightness_get(led_cdev)) {
		if (!is31->work_scheduled) {
			schedule_work(&is31->work);
			is31->work_scheduled = true;
// printk("scheduled for brightness %u\n", brightness);
		}
	}

	spin_unlock_irqrestore(&is31->lock, flags);
}

static int is31fl319x_blink_set(struct led_classdev *led_cdev,
			     unsigned long *delay_on,
			     unsigned long *delay_off)
{
	struct is31fl319x_led *led = container_of(led_cdev, struct is31fl319x_led,
					       led_cdev);

	struct is31fl319x_chip *is31 = led->chip;
	unsigned long flags;
	printk("is31fl319x_blink_set %u %lu %lu\n", led-is31->leds, *delay_on, *delay_off);

	// check for real change

	return 1;	// software blink

	spin_lock_irqsave(&is31->lock, flags);

	// store new delays in led->...

	if (!is31->work_scheduled) {
		schedule_work(&is31->work);
		is31->work_scheduled = true;
		printk("scheduled for blink %lu %lu\n", *delay_on, *delay_off);
	}

	spin_unlock_irqrestore(&is31->lock, flags);

	return 0;
}

#ifdef CONFIG_OF
static struct is31fl319x_platform_data *
is31fl319x_led_dt_init(struct i2c_client *client, int num_leds)
{
	struct device_node *np = client->dev.of_node, *child;
	struct is31fl319x_platform_data *pdata;
	struct led_info *is31_leds;
	int count;

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

		led.name =
			of_get_property(child, "label", NULL) ? : child->name;
		led.default_trigger =
			of_get_property(child, "linux,default-trigger", NULL);
		led.flags = 0;
		ret = of_property_read_u32(child, "reg", &reg);
		dev_dbg(&client->dev, "name = %s reg = %d\n", led.name, reg);
		if (ret != 0 || reg < 0 || reg >= num_leds)
			continue;

		if (is31_leds[reg].name)
			dev_err(&client->dev, "duplicate led line %d for %s -> %s\n", reg, is31_leds[reg].name, led.name);
		is31_leds[reg] = led;
	}
	pdata = devm_kzalloc(&client->dev,
			sizeof(struct is31fl319x_platform_data), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->leds.leds = is31_leds;
	return pdata;
}

static const struct of_device_id of_is31fl319x_leds_match[] = {
	{ .compatible = "issi,is31fl3196", (void *) 6 },
	{ .compatible = "issi,is31fl3199", (void *) 9 },
	{},
};
MODULE_DEVICE_TABLE(of, of_is31fl319x_leds_match);

#else
static struct is31fl319x_platform_data *
is31fl319x_led_dt_init(struct i2c_client *client)
{
	return ERR_PTR(-ENODEV);
}

#endif

static int is31fl319x_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct is31fl319x_chip *is31;
	struct i2c_adapter *adapter;
	struct is31fl319x_platform_data *pdata;
	int err;
	int i = 0;

	adapter = to_i2c_adapter(client->dev.parent);
	pdata = dev_get_platdata(&client->dev);

	dev_dbg(&client->dev, "probe\n");

	dev_dbg(&client->dev, "NUM_LEDS = %d num_leds = %d\n", NUM_LEDS, (int) id->driver_data);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	if (!pdata) {
		pdata = is31fl319x_led_dt_init(client, (int) id->driver_data);
		if (IS_ERR(pdata)) {
			dev_err(&client->dev, "DT led error %d\n", (int) PTR_ERR(pdata));
			return PTR_ERR(pdata);
		}
	}
	is31 = devm_kzalloc(&client->dev, sizeof(*is31), GFP_KERNEL);
	if (!is31)
		return -ENOMEM;

	is31->client = client;
	INIT_WORK(&is31->work, is31fl319x_work);
	spin_lock_init(&is31->lock);
	i2c_set_clientdata(client, is31);

	err = is31fl319x_write(is31, IS31FL319X_SHUTDOWN, 0x01);	/* check for reply */
	if (err < 0) {
		dev_err(&client->dev, "no response from chip write: err = %d\n", err);
		return -EPROBE_DEFER;	/* does not answer (yet) */
	}

	/* set PWM modes
	 * handle LED current and AGC values from DT etc.
	 * and write to registers as needed
	 * CS = (3 - ((mA + 4) / 5) & 0x7) << 4	-- require between 5 mA and 40 mA
	 * AGS = (dB / 3)
	 */

	for (i = 0; i < NUM_LEDS; i++) {
		struct is31fl319x_led *l = is31->leds + i;

		l->chip = is31;
		if (pdata->leds.leds[i].name && !pdata->leds.leds[i].flags) {
			l->led_cdev.name = pdata->leds.leds[i].name;
			l->led_cdev.default_trigger
				= pdata->leds.leds[i].default_trigger;
			l->led_cdev.brightness_set = is31fl319x_brightness_set;
			l->led_cdev.blink_set = is31fl319x_blink_set;
			err = led_classdev_register(&client->dev,
						    &l->led_cdev);
			if (err < 0)
				goto exit;
		}
	}

	schedule_work(&is31->work);	/* first update */

	dev_dbg(&client->dev, "probed\n");
	return 0;
exit:
	dev_err(&client->dev, "led error %d\n", err);

	while (i--) {
		if (is31->leds[i].led_cdev.name)
			led_classdev_unregister(&is31->leds[i].led_cdev);
	}
	return err;
}

static int is31fl319x_remove(struct i2c_client *client)
{
	int i;
	struct is31fl319x_chip *is31 = i2c_get_clientdata(client);
	struct is31fl319x_led *is31_leds = is31->leds;

	for (i = 0; i < NUM_LEDS; i++) {
		if (is31_leds[i].led_cdev.name)
			led_classdev_unregister(&is31_leds[i].led_cdev);
	}

	cancel_work_sync(&is31->work);

	return 0;
}

static struct i2c_driver is31fl319x_driver = {
	.driver   = {
		.name    = "leds-is31fl319x",
		.of_match_table = of_match_ptr(of_is31fl319x_leds_match),
	},
	.probe    = is31fl319x_probe,
	.remove   = is31fl319x_remove,
	.id_table = is31fl319x_id,
};

module_i2c_driver(is31fl319x_driver);

MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("IS31FL319X LED driver");
MODULE_LICENSE("GPL v2");
