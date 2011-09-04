/*
 * Copyright 2007-2008 Extreme Engineering Solutions, Inc.
 *
 * Author: Nate Case <ncase@xes-inc.com>
 *
 * derived from leds-pca955x.c for TCA6507 by: Nikolaus Schaller <hns@goldelico.com>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * LED driver for various tca6507 I2C LED drivers
 *
 * Supported devices:
 *
 *	Device		Description		7-bit slave address
 *	------		-----------		-------------------
 *	TCA6507		7-bit driver		0x45
 *
 * TI TCA6507 LED driver chips follow a register map as shown below:
 *
 *	Control Register		Description
 *	----------------		-----------
 *	0x00					Select0
 *	0x01					Select1
 *  0x02					Select2
 *	0x03					Fade on time
 *	0x04					Fully on time
 *	0x05					Fade off time
 *	0x06					First fully off time
 *	0x07					Second fully off time
 *	0x08					Maximum intensity
 *	0x09					One Shot / Master Intensity
 *	0x0a					Initialize
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>

/* LED select registers determine the source that drives LED outputs */
#define TCA6507_LS_LED_OFF	0x0	/* Output HI-Z (off) */
#define TCA6507_LS_LED_PWM0	0x2	/* Output LOW with Bank0 rate */
#define TCA6507_LS_LED_PWM1	0x3	/* Output LOW with Bank1 rate */
#define TCA6507_LS_LED_ON	0x4	/* Output LOW (on) */
#define TCA6507_LS_LED_MIR	0x5	/* Output LOW with Master Intensity */
#define TCA6507_LS_BLINK0	0x6	/* Blink at Bank0 rate */
#define TCA6507_LS_BLINK1	0x7	/* Blink at Bank1 rate */

/* PWM registers */
#define TCA6507_MAX_INTENSITY		0x08
#define TCA6507_MASTER_INTENSITY	0x09

enum tca6507_type {
	tca6507,
};

struct tca6507_chipdef {
	int			bits;
	u8			slv_addr;	/* 7-bit slave address mask */
	int			slv_addr_shift;	/* Number of bits to ignore */
};

static struct tca6507_chipdef tca6507_chipdefs[] = {
	[tca6507] = {
		.bits		= 7,
		.slv_addr	= /* 1000101x */ 0x45,
		.slv_addr_shift	= 0,
	},
};

static const struct i2c_device_id tca6507_id[] = {
	{ "tca6507", tca6507 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tca6507);

struct tca6507_led {
	struct tca6507_chipdef	*chipdef;
	struct i2c_client	*client;
	struct work_struct	work;
	spinlock_t		lock;
	enum led_brightness	brightness;
	struct led_classdev	led_cdev;
	int				led_num;	/* 0 .. 15 potentially */
	char			name[32];
};

/*
 * Return an LED selector registers value based on an existing one, with
 * the appropriate 3-bit state value set for the given LED number (0-6).
 */
static inline u32 tca6507_ledsel(u32 oldval, int led_num, int state)
{
	oldval &= ~(0x010101 << led_num);
	oldval |= (state & 0x01) << (0+led_num);
	oldval |= (state & 0x02) << (8-1+led_num);
	oldval |= (state & 0x04) << (16-2+led_num);
	return oldval;
}

/*
 * Write to frequency prescaler register, used to program the
 * period of the PWM output.  period = (PSCx + 1) / 38
 */
static void tca6507_write_psc(struct i2c_client *client, int n, u8 val)
{
//	struct tca6507_led *tca6507 = i2c_get_clientdata(client);
	/* N/A on TCA6507 */
}

/*
 * Write to PWM register, which determines the duty cycle of the
 * output.  LED is OFF when the count is less than the value of this
 * register, and ON when it is greater.  If PWMx == 0, LED is always OFF.
 *
 * Duty cycle is (256 - PWMx) / 256
 */
static void tca6507_write_pwm(struct i2c_client *client, int reg, u8 val)
{
//	struct tca6507_led *tca6507 = i2c_get_clientdata(client);
	/* the 8 bits are split into two 4 bit values for Bank1 and Bank0 */
	i2c_smbus_write_byte_data(client, reg, val);
}

/*
 * Write to LED selector registers, which determine the source that
 * drives the LED output.
 */
static void tca6507_write_ls(struct i2c_client *client, int n, u32 val)
{
//	struct tca6507_led *tca6507 = i2c_get_clientdata(client);

	i2c_smbus_write_byte_data(client,
							  0x00,
							  (u8) (val>>0));
	i2c_smbus_write_byte_data(client,
							  0x01,
							  (u8) (val>>8));
	i2c_smbus_write_byte_data(client,
							  0x02,
							  (u8) (val>>16));
}

/*
 * Read the LED selector registers, which determine the source that
 * drives the LED output.
 */
static u32 tca6507_read_ls(struct i2c_client *client, int n)
{
//	struct tca6507_led *tca6507 = i2c_get_clientdata(client);

	return (u32) ( i2c_smbus_read_byte_data(client, 0x00)
				  | ( i2c_smbus_read_byte_data(client, 0x01) << 8)
				  | ( i2c_smbus_read_byte_data(client, 0x02) << 16)
				  );
}

static void tca6507_led_work(struct work_struct *work)
{
	struct tca6507_led *tca6507;
	u32 ls;
	int ls_led;	/* which set of bits within the Select registers to use (0-2) */

	tca6507 = container_of(work, struct tca6507_led, work);
	ls_led = tca6507->led_num % 4;

	ls = tca6507_read_ls(tca6507->client, ls_led);

	
	switch (tca6507->brightness) {
	case LED_FULL:	/* 255 */
		ls = tca6507_ledsel(ls, ls_led, TCA6507_LS_LED_ON);
		break;
	case LED_OFF:	/* 0 */
		ls = tca6507_ledsel(ls, ls_led, TCA6507_LS_LED_OFF);
		break;
	case LED_HALF:	/* 127 */
		ls = tca6507_ledsel(ls, ls_led, TCA6507_LS_LED_PWM0);
		break;
	default:
		/*
		 * Use PWM0 for all other values.  This has the unwanted
		 * side effect of making all LEDs on the chip share the
		 * same brightness level if set to a value other than
		 * OFF, HALF, or FULL.  But, this is probably better than
		 * just turning off for all other values.
		 */
		tca6507_write_pwm(tca6507->client, TCA6507_MASTER_INTENSITY, 0x30 | (tca6507->brightness & 0xf));	/* write master intensity and use in BANK0/1 */
		if(tca6507->brightness <= 15)
			ls = tca6507_ledsel(ls, ls_led, TCA6507_LS_LED_MIR);	// just dimmed
		else if(tca6507->brightness <= 31)
			ls = tca6507_ledsel(ls, ls_led, TCA6507_LS_BLINK0);
		else 
			ls = tca6507_ledsel(ls, ls_led, TCA6507_LS_LED_PWM0);
		break;
	}

	tca6507_write_ls(tca6507->client, ls_led, ls);
}

static void tca6507_led_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct tca6507_led *tca6507;

	tca6507 = container_of(led_cdev, struct tca6507_led, led_cdev);

	spin_lock(&tca6507->lock);
	tca6507->brightness = value;

	/*
	 * Must use workqueue for the actual I/O since I2C operations
	 * can sleep.
	 */
	schedule_work(&tca6507->work);

	spin_unlock(&tca6507->lock);
}

static int __devinit tca6507_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct tca6507_led *tca6507;
	struct tca6507_chipdef *chip;
	struct i2c_adapter *adapter;
	struct led_platform_data *pdata;
	int i, err;

	chip = &tca6507_chipdefs[id->driver_data];
	adapter = to_i2c_adapter(client->dev.parent);
	pdata = client->dev.platform_data;

	/* Make sure the slave address / chip type combo given is possible */
	if ((client->addr & ~((1 << chip->slv_addr_shift) - 1)) !=
	    chip->slv_addr) {
		dev_err(&client->dev, "invalid slave address %02x\n",
				client->addr);
		return -ENODEV;
	}

	printk(KERN_INFO "leds-tca6507: Using %s %d-bit LED driver at "
			"slave address 0x%02x\n",
			id->name, chip->bits, client->addr);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	if (pdata) {
		if (pdata->num_leds != chip->bits) {
			dev_err(&client->dev, "board info claims %d LEDs"
					" on a %d-bit chip\n",
					pdata->num_leds, chip->bits);
			return -ENODEV;
		}
	}

	tca6507 = kzalloc(sizeof(*tca6507) * chip->bits, GFP_KERNEL);
	if (!tca6507)
		return -ENOMEM;

	i2c_set_clientdata(client, tca6507);

	for (i = 0; i < chip->bits; i++) {
		tca6507[i].chipdef = chip;
		tca6507[i].client = client;
		tca6507[i].led_num = i;

		/* Platform data can specify LED names and default triggers */
		if (pdata) {
			if (pdata->leds[i].name)
				snprintf(tca6507[i].name,
					 sizeof(tca6507[i].name), "tca6507:%s",
					 pdata->leds[i].name);
			if (pdata->leds[i].default_trigger)
				tca6507[i].led_cdev.default_trigger =
					pdata->leds[i].default_trigger;
		} else {
			snprintf(tca6507[i].name, sizeof(tca6507[i].name),
				 "tca6507:%d", i);
		}

		spin_lock_init(&tca6507[i].lock);

		tca6507[i].led_cdev.name = tca6507[i].name;
		tca6507[i].led_cdev.brightness_set = tca6507_led_set;

		INIT_WORK(&tca6507[i].work, tca6507_led_work);

		err = led_classdev_register(&client->dev, &tca6507[i].led_cdev);
		if (err < 0)
			goto exit;
	}

	/* PWM0 is used for half brightness or 50% duty cycle */
	tca6507_write_pwm(client, 0, 255-LED_HALF);

	/* PWM1 is used for variable brightness, default to OFF */
	tca6507_write_pwm(client, 1, 0);

	/* Set to fast frequency so we do not see flashing */
	tca6507_write_psc(client, 0, 0);
	tca6507_write_psc(client, 1, 0);

	return 0;

exit:
	while (i--) {
		led_classdev_unregister(&tca6507[i].led_cdev);
		cancel_work_sync(&tca6507[i].work);
	}

	kfree(tca6507);
	i2c_set_clientdata(client, NULL);

	return err;
}

static int __devexit tca6507_remove(struct i2c_client *client)
{
	struct tca6507_led *tca6507 = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < tca6507->chipdef->bits; i++) {
		led_classdev_unregister(&tca6507[i].led_cdev);
		cancel_work_sync(&tca6507[i].work);
	}

	kfree(tca6507);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static struct i2c_driver tca6507_driver = {
	.driver = {
		.name	= "leds-tca6507",
		.owner	= THIS_MODULE,
	},
	.probe	= tca6507_probe,
	.remove	= __devexit_p(tca6507_remove),
	.id_table = tca6507_id,
};

static int __init tca6507_leds_init(void)
{
	return i2c_add_driver(&tca6507_driver);
}

static void __exit tca6507_leds_exit(void)
{
	i2c_del_driver(&tca6507_driver);
}

module_init(tca6507_leds_init);
module_exit(tca6507_leds_exit);

MODULE_AUTHOR("Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("TCA6507 LED driver");
MODULE_LICENSE("GPL v2");
