/*
 * i2c_jz47xx.c for the INGENIC I2C bus access.
 *
 * Copyright (C) 2006 - 2009 Ingenic Semiconductor Inc.
 * Author: <cwjia@ingenic.cn>
 * The first Modified :<zhzhao@ingenic.cn>
 * Date:20091027 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <linux/module.h>
#include <asm/addrspace.h>

#include <asm/jzsoc.h>
#include "i2c-jz47xx.h"

#ifndef EEPROM_DEVICE_NUMBER
#define EEPROM_DEVICE_NUMBER   0x50   /*eeprom device number.20091027*/
#endif

/* I2C protocol */
#define I2C_READ	1
#define I2C_WRITE	0

#define TIMEOUT         1000
unsigned long sub_addr = 0;

struct jz_i2c {
	spinlock_t		lock;
	wait_queue_head_t	wait;
	struct i2c_msg		*msg;
	unsigned int		msg_num;
	unsigned int		slave_addr;
	struct i2c_adapter	adap;
	struct clk		*clk;
};

/*
 * I2C bus protocol basic routines
 */
static int i2c_put_data(unsigned char data)
{
	unsigned int timeout = TIMEOUT*10;
	__i2c_write(data);
	__i2c_set_drf();
	while (__i2c_check_drf() != 0 && timeout)
		timeout--;
	while (!__i2c_transmit_ended());
	
	timeout = TIMEOUT*10;	
	while (!__i2c_received_ack() && timeout)
		timeout--;
	if (timeout){
		return 0;
	}
	else{
		return -ETIMEDOUT;
	}
}


static int i2c_get_data(unsigned char *data, int ack)
{
	int timeout = TIMEOUT*10;
	if (!ack)
		__i2c_send_nack();
	else
		__i2c_send_ack();
	
	while (__i2c_check_drf() == 0 && timeout)
		timeout--;
	if (timeout) {
		if (!ack)
			__i2c_send_stop();
		*data = __i2c_read();
		__i2c_clear_drf();
		return 0;
	} else{
		
		return -ETIMEDOUT;
	}
}

/*
 * I2C interface
 */
void i2c_jz_setclk(unsigned int i2cclk)
{
#ifdef CONFIG_SOC_JZ4730
	printk(KERN_EMERG "i2c_jz_setclk()\n");
	__i2c_set_clk(jz_clocks.devclk, i2cclk);
#else
	__i2c_set_clk(jz_clocks.extalclk, i2cclk);
#endif
}


static int xfer_read(unsigned char device, unsigned char *buf, int length)
{
	int cnt = length;
	int timeout = 5;
	
	/*eeprom device address transfer*/
	if(EEPROM_DEVICE_NUMBER == (device & 0xf0)){
		device = device | ((sub_addr & 0x0700) >> 8);
		sub_addr = sub_addr & 0xff;
	}
	
L_try_again:
	
	if (timeout < 0)
		goto L_timeout;
	
	__i2c_send_nack();	/* Master does not send ACK, slave sends it */
	
	__i2c_send_start();
	
	
	if (i2c_put_data( (device << 1) | I2C_WRITE ) < 0)
		goto device_werr;
	
	if (i2c_put_data(sub_addr) < 0)
		goto address_err;
	
	__i2c_send_start();
	
	if (i2c_put_data((device << 1) | I2C_READ ) < 0)
		goto device_rerr;
	
	
	__i2c_send_ack();	/* Master sends ACK for continue reading */
	
	
	while (cnt) {
		
		if (cnt == 1) {
			
			if (i2c_get_data(buf, 0) < 0)
				break;
		} else {
			
			if (i2c_get_data(buf, 1) < 0){
				break;
			}
		}
		cnt--;
		buf++;
	}
	__i2c_send_stop();
	
	return length - cnt;
device_rerr:
device_werr:
address_err:
	
	timeout --;
	__i2c_send_stop();
	goto L_try_again;
	
L_timeout:
	__i2c_send_stop();
	printk("Read I2C device 0x%2x failed.\n", device);
	return -ENODEV;
}


static int xfer_write(unsigned char device, unsigned char *buf, int length)
{
	int cnt = length;
	int cnt_in_pg;
	int timeout = 5;
	unsigned char *tmpbuf;
	
	/*eeprom device address transfer*/
	if(EEPROM_DEVICE_NUMBER == (device & 0xf0)){
		device = device | ((sub_addr & 0x0700) >> 8);
		sub_addr = sub_addr & 0xff; 
	}
	__i2c_send_nack();	/* Master does not send ACK, slave sends it */
	
W_try_again:
	if (timeout < 0)
		goto W_timeout;
	
	cnt = length;
	tmpbuf = (unsigned char *)buf;
	
start_write_page:
	cnt_in_pg = 0;
	__i2c_send_start();
	if (i2c_put_data( (device << 1) | I2C_WRITE ) < 0)
		goto device_err;
	
	if (i2c_put_data(sub_addr) < 0)
		goto address_err;
	
	
	
	while (cnt) {
		if (++cnt_in_pg > 8) {
			__i2c_send_stop();
			mdelay(1);
			sub_addr += 8;
			mdelay(2);// add 20091027
			goto start_write_page;
			
		}
		
		
		if (i2c_put_data(*tmpbuf) < 0) 
			break;
		cnt--;
		tmpbuf++;
	}
	__i2c_send_stop();
	return length - cnt;
device_err:
address_err:
	timeout--;
	__i2c_send_stop();
	goto W_try_again;
	
W_timeout:
	printk( "Write I2C device 0x%2x failed.\n", device);
	__i2c_send_stop();
	return -ENODEV;
}

static int i2c_jz_xfer(struct i2c_adapter *adap, struct i2c_msg *pmsg, int num)
{
	int ret, i;
	
	dev_dbg(&adap->dev, "jz47xx_xfer: processing %d messages:\n", num);
	for (i = 0; i < num; i++) {
		dev_dbg(&adap->dev, " #%d: %sing %d byte%s %s 0x%02x\n", i,
				pmsg->flags & I2C_M_RD ? "read" : "writ",
				pmsg->len, pmsg->len > 1 ? "s" : "",
				pmsg->flags & I2C_M_RD ? "from" : "to",	pmsg->addr);
		if (pmsg->len && pmsg->buf) {	/* sanity check */
			if (pmsg->flags & I2C_M_RD){
				
				ret = xfer_read(pmsg->addr,  pmsg->buf, pmsg->len);
			}else{
				
				ret = xfer_write(pmsg->addr,  pmsg->buf, pmsg->len);
			}
			if (ret)
				return ret;
			/* Wait until transfer is finished */
		}
		dev_dbg(&adap->dev, "transfer complete\n");
		pmsg++;		/* next message */
	}
	return i;
}

static u32 i2c_jz_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm i2c_jz_algorithm = {
.master_xfer	= i2c_jz_xfer,
.functionality	= i2c_jz_functionality,
};

static int i2c_jz_probe(struct platform_device *dev)
{
	
	struct jz_i2c *i2c;
	struct i2c_jz_platform_data *plat = dev->dev.platform_data;
	int ret;
#ifdef CONFIG_SOC_JZ4730
	printk(KERN_EMERG "i2c_jz_probe()\n");
	/*
	 * I2C_SCK, I2C_SDA
	 */
/*#define __gpio_as_i2c()				\
do {						\
REG_GPIO_PXFUNS(3) = 0x01800000;	\
REG_GPIO_PXSELS(3) = 0x01800000;	\
REG_GPIO_PXPES(3) = 0x01800000;		\
} while (0)
*/	
	__i2c_set_clk(jz_clocks.devclk, 10000); /* default 10 KHz */
#else
	__gpio_as_i2c(); // open i2c 20091027
	__i2c_set_clk(jz_clocks.extalclk, 10000); /* default 10 KHz */
#endif
	__i2c_enable();
	
	i2c = kzalloc(sizeof(struct jz_i2c), GFP_KERNEL);
	if (!i2c) {
		printk("There is no enough memory\n");
		ret = -ENOMEM;
		goto emalloc;
	}
	
	i2c->adap.owner   = THIS_MODULE;
	i2c->adap.algo    = &i2c_jz_algorithm;
	i2c->adap.retries = 5;
	spin_lock_init(&i2c->lock);
	init_waitqueue_head(&i2c->wait);
	sprintf(i2c->adap.name, "jz_i2c-i2c.%u", dev->id);
	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &dev->dev;
	
	if (plat) {
		i2c->adap.class = plat->class;
	}
	
	/*
	 * If "dev->id" is negative we consider it as zero.
	 * The reason to do so is to avoid sysfs names that only make
	 * sense when there are multiple adapters.
	 */
	i2c->adap.nr = dev->id != -1 ? dev->id : 0;
	/* ret = i2c_add_adapter(&i2c->adap); */
	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {
		printk(KERN_INFO "I2C: Failed to add bus\n");
		goto eadapt;
	}
	
	platform_set_drvdata(dev, i2c);
	dev_info(&dev->dev, "JZ47xx i2c bus driver.\n");
	return 0;
eadapt:
	__i2c_disable();
emalloc:
	return ret;
}

static int i2c_jz_remove(struct platform_device *dev)
{
	struct i2c_adapter *adapter = platform_get_drvdata(dev);
	int rc;
	
	rc = i2c_del_adapter(adapter);
	platform_set_drvdata(dev, NULL);
	return rc;
}

static struct platform_driver i2c_jz_driver = {
.probe		= i2c_jz_probe,
.remove		= i2c_jz_remove,
.driver		= {
.name	= "jz_i2c",
},
};

static int __init i2c_adap_jz_init(void)
{
	return platform_driver_register(&i2c_jz_driver);
}

static void __exit i2c_adap_jz_exit(void)
{
	return platform_driver_unregister(&i2c_jz_driver);
}

MODULE_LICENSE("GPL");

module_init(i2c_adap_jz_init);
module_exit(i2c_adap_jz_exit);
