/*
 *  Copyright (C) 2004 Samsung Electronics 
 *             SW.LEE <hitchcar@samsung.com>
 *
 *  Copyright (C) 2000 Russell King : pcf8583.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Driver for FIMC20 Camera Decoder 
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/delay.h>


#ifdef CONFIG_ARCH_S3C24A0A
#else
//#include <asm/arch/S3C2440.h>
#endif

#define SW_DEBUG
#define CONFIG_VIDEO_V4L1_COMPAT
#include <linux/videodev.h>
#include "camif.h"
#include "sensor.h"

#ifndef SAMSUNG_SXGA_CAM
#include "s5x532_rev36.h"
#else
#include "sxga.h"
#endif

static struct i2c_driver s5x532_driver;
static camif_gc_t data = {
	itu_fmt:      CAMIF_ITU601,
	order422:     CAMIF_YCBYCR,
	camclk:       24000000,
#ifndef SAMSUNG_SXGA_CAM
	source_x:     640,
	source_y:     480,
	win_hor_ofst: 112,
	win_ver_ofst: 20,
#else
	source_x:     1280,
	source_y:     1024,
	win_hor_ofst: 0,
	win_ver_ofst: 0,
#endif
	polarity_pclk:1,
	polarity_href:0,
#ifdef CONFIG_ARCH_S3C24A0A
	reset_type:CAMIF_EX_RESET_AL, /* Active Low */
#else
	reset_type:CAMIF_EX_RESET_AH, /* Ref board has inverted signal */
#endif
	reset_udelay:2000,
};

#define CAM_ID 0x5a

static unsigned short ignore = I2C_CLIENT_END;
static unsigned short normal_addr[] = { (CAM_ID>>1), I2C_CLIENT_END };
static struct i2c_client_address_data addr_data = {
	normal_i2c:		normal_addr,
	probe:			&ignore,
	ignore:			&ignore,
};

s5x532_t s5x532_regs_mirror[S5X532_REGS];

unsigned char 
s5x532_read(struct i2c_client *client, unsigned char subaddr)
{
	int ret;
	unsigned char buf[1];
	struct i2c_msg msg ={ client->addr, 0, 1, buf};
	buf[0] = subaddr;

	ret = i2c_transfer(client->adapter,&msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) {
	  printk(" I2C write Error \n");
	  return -EIO;
	}
	
	msg.flags = I2C_M_RD;
	ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;

	return buf[0];
}


static int
s5x532_write(struct i2c_client *client,
		  unsigned char subaddr, unsigned char val)
{
       unsigned char buf[2];
       struct i2c_msg msg = { client->addr, 0, 2, buf};

       buf[0]= subaddr;
       buf[1]= val;

       return i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}

void inline s5x532_init(struct i2c_client *sam_client)
{
	int i;

	printk(KERN_ERR "s5x532_init  \n");
	for (i = 0; i < S5X532_INIT_REGS; i++) {
		s5x532_write(sam_client,
			      s5x532_reg[i].subaddr, s5x532_reg[i].value );
	}

#ifdef YOU_WANT_TO_CHECK_IMG_SENSOR 
	for (i = 0; i < S5X532_INIT_REGS;i++) {
		if ( s5x532_reg[i].subaddr == PAGE_ADDRESS ) { 
			s5x532_write(sam_client,
			      s5x532_reg[i].subaddr, s5x532_reg[i].value);

			printk(KERN_ERR "Page: Subaddr %02x = 0x%02x\n", 
			       s5x532_reg[i].subaddr, s5x532_regs_mirror[i].value);
	

		} else 
		{      
			s5x532_regs_mirror[i].subaddr = s5x532_reg[i].subaddr; 
			s5x532_regs_mirror[i].value = 
			s5x532_read(sam_client,s5x532_reg[i].subaddr);
			printk(KERN_ERR "Subaddr %02x = 0x%02x\n", 
			       s5x532_reg[i].subaddr, s5x532_regs_mirror[i].value);
		}
	}
#endif
       
}

static int
s5x532_attach(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *c;

	c = kmalloc(sizeof(*c), GFP_KERNEL);
	if (!c)	return -ENOMEM;

	strcpy(c->name, "S5X532");
//	c->id		= s5x532_driver.id;
	c->flags	= 0 /* I2C_CLIENT_ALLOW_USE */;
	c->addr		= addr;
	c->adapter	= adap;
	c->driver	= &s5x532_driver;
	data.sensor     = c;
	i2c_set_clientdata(c, &data);

	camif_register_decoder(c);
	return i2c_attach_client(c);
}

static int s5x532_probe(struct i2c_adapter *adap)
{ 
	return i2c_probe(adap, &addr_data, s5x532_attach);
}

static int s5x532_detach(struct i2c_client *client)
{
	i2c_detach_client(client);
	camif_unregister_decoder(client);
	return 0;
}

static int
s5x532_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
        switch (cmd) {
        case SENSOR_INIT:
                 s5x532_init(client);
		 printk(KERN_INFO "CAMERA: S5X532 Sensor initialized\n");
		 break;
	case USER_ADD:
		/* MOD_INC_USE_COUNT; uh.. 2.6 deals with this, old-timer */
		break;
	case USER_EXIT:
		/* MOD_DEC_USE_COUNT; */
		break;
/* Todo
	case SENSOR_BRIGHTNESS:
		change_sensor();
		break;
*/
	default:	
		panic("Unexpect Sensor Command \n");
		break;
        }
	return 0;
}

static struct i2c_driver s5x532_driver = {
	driver:		{ name:		"S5X532" },
	id:		0, /* optional in i2c-id.h I2C_ALGO_S3C, */
	attach_adapter:	s5x532_probe,
	detach_client:	s5x532_detach,
	command:	s5x532_command
};

static void iic_gpio_port(void) 
{
/*    FIXME: no gpio config for i2c !!!
#ifdef CONFIG_ARCH_S3C24A0A
#else
	GPECON &=  ~(0xf <<28);
	GPECON |=    0xa <<28;
#endif
*/
}

static __init int camif_sensor_init(void)
{
        iic_gpio_port();
	return i2c_add_driver(&s5x532_driver);
}


static __init void camif_sensor_exit(void)
{
	i2c_del_driver(&s5x532_driver);
}

module_init(camif_sensor_init)
module_exit(camif_sensor_exit)

MODULE_AUTHOR("SW.LEE <hitchcar@sec.samsung.com>");
MODULE_DESCRIPTION("I2C Client Driver For Fimc2.0 MISC Driver");
MODULE_LICENSE("GPL");



/*
 * Local variables:
 * c-basic-offset: 8
 * End:
 */
