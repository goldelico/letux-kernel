/*
 * i2c_jz47xx.c for the INGENIC I2C bus access.
 *
 * Copyright (C) 2006 - 2009 Ingenic Semiconductor Inc.
 * Author: <cwjia@ingenic.cn>
 * The first Modified :<zhzhao@ingenic.cn>
 * fixed to really work: hns@goldelico.com
 * Date:20100504
 * fixed to handle http://www.i2c-bus.org/fileadmin/ftp/i2c_bus_specification_1995.pdf
 * Date:20100906
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


/* I2C protocol */
#define I2C_READ	1
#define I2C_WRITE	0

#define TIMEOUT         1000

struct jz_i2c {
	spinlock_t		lock;
	wait_queue_head_t	wait;
	struct i2c_msg		*msg;
	unsigned int		msg_num;
	unsigned int		slave_addr;
	struct i2c_adapter	adap;
	struct clk		*clk;
};


#if OLD

/*
 * I2C bus protocol basic routines
 */

static int i2c_put_data(unsigned char data)
{
	unsigned int timeout;
	printk(KERN_EMERG "i2c_put_data(%02x)\n", data);
	__i2c_send_nack();
	__i2c_write(data);	// set data to be written
	__i2c_set_drf();	// data is ready flag
	timeout = TIMEOUT*10;	
	while (!__i2c_transmit_ended() && timeout)
		timeout--;	// wait until transmit done by controller
	if (timeout) {
		if (!__i2c_received_ack())
			return -EIO;
		return 0;	// ok
	}
	printk(KERN_EMERG "i2c_put_data(): timeout\n");
	return -ETIMEDOUT;
}


static int i2c_get_data(unsigned char *data)
{
	unsigned int timeout;
	__i2c_send_ack();
	while (__i2c_check_drf() == 0 && timeout)
		timeout--;	// wait until data received by controller
	if (timeout) {
		*data = __i2c_read();
		printk(KERN_EMERG "i2c_get_data() -> %02x\n", *data);
		if (!__i2c_received_ack())
			return -EIO;	// no ACK
		__i2c_clear_drf();	// received
		return 0;	// ok
	}
	printk(KERN_EMERG "i2c_get_data(): timeout\n");		
	return -ETIMEDOUT;
}

/*
 * I2C interface
 */

static int xfer_read(unsigned char device, unsigned char *buf, int length, int flags)
{
	int cnt;
	int timeout = 5;
	unsigned char *tmpbuf;
	
L_try_again:
	
	if (timeout < 0)
		goto L_timeout;
	
	cnt = length;
	tmpbuf = (unsigned char *)buf;

	__i2c_send_nack();	/* Master does not send ACK, slave sends it */
	
//	__i2c_send_start();
		
//	if (i2c_put_data( (device << 1) | I2C_WRITE ) < 0)
//		goto device_werr;
	
//	if (i2c_put_data(sub_addr) < 0)
//		goto address_err;
	
	// should skip this if I2C_M_NOSTART so that we can follow a read on a write (which sets the subaddress)
	// but we should then send a repeated-start condition
	
	__i2c_send_start();
	
	if (i2c_put_data((device << 1) | I2C_READ ) < 0)
		goto device_rerr;
	
	
	__i2c_send_ack();	/* Master sends ACK for continue reading */
	
//	printk(KERN_EMERG "xfer_read: read %d bytes\n", cnt);
	
	while (cnt) {
		
		if (cnt == 1) {
			
			if (i2c_get_data(tmpbuf, 0) < 0)	// last read - does __i2c_send_stop() 
				break;
		} else {
			
			if (i2c_get_data(tmpbuf, 1) < 0)
				break;
		}
		cnt--;
		tmpbuf++;
	}
//	printk(KERN_EMERG "xfer_read: cnt is %d\n", cnt);
	if(flags & I2C_M_STOP)
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


static int xfer_write(unsigned char device, unsigned char *buf, int length, int flags)
{
	int cnt;
	int timeout = 5;
	unsigned char *tmpbuf;
	
W_try_again:
	if (timeout < 0)
		goto W_timeout;
	
	cnt = length;
	tmpbuf = (unsigned char *)buf;
	
	__i2c_send_start();
	if (i2c_put_data( (device << 1) | I2C_WRITE ) < 0)
		goto device_err;
	
	while (cnt) {
		
		if (i2c_put_data(*tmpbuf) < 0) 
			break;	// NAK before all sent
		cnt--;
		tmpbuf++;
	}
	if(flags & I2C_M_STOP)
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

#endif

void i2c_jz_setclk(unsigned int i2cclk)
{
#ifdef CONFIG_SOC_JZ4730
	//	printk(KERN_EMERG "i2c_jz_setclk()\n");
	__i2c_set_clk(jz_clocks.devclk, i2cclk);
#else
	__i2c_set_clk(jz_clocks.extalclk, i2cclk);
#endif
}

static int i2c_jz_xfer(struct i2c_adapter *adap, struct i2c_msg *pmsg, int num)
{
	int ret, i;
	int cnt;
	unsigned char *tmpbuf;
	
	printk(KERN_INFO "i2c_jz_xfer %d messages\n", num);
	dev_dbg(&adap->dev, "jz47xx_xfer: processing %d messages:\n", num);
	for (i = 0; i < num; i++, pmsg++) {
		ret = num;	// assume ok
		dev_dbg(&adap->dev, " #%d: %s %d byte%s %s 0x%02x flags %04x\n", i,
				pmsg->flags & I2C_M_RD ? "reading" : "writing",
				pmsg->len, pmsg->len > 1 ? "s" : "",
				pmsg->flags & I2C_M_RD ? "from" : "to",	pmsg->addr,
				pmsg->flags);
		if (pmsg->flags & (/*I2C_M_TEN|I2C_M_NOSTART|*/I2C_M_REV_DIR_ADDR|/*I2C_M_IGNORE_NAK|*/I2C_M_NO_RD_ACK|I2C_M_RECV_LEN)) {
			dev_dbg(&adap->dev, "jz47xx_xfer: flags=%04x not supported\n", pmsg->flags);
			return -EIO;
		}
		
		if (!pmsg->buf)
			continue;	/* sanity check */
		printk(KERN_INFO "addr=%x flags=%04x\n", pmsg->addr, pmsg->flags);
		if (!(pmsg->flags & I2C_M_NOSTART))
			__i2c_send_start();
		__i2c_send_ack();	// default (only last byte during receive gets nack)
/*
		if ((pmsg->flags & I2C_M_TEN)) { // 10 bit address
			__i2c_write(((pmsg->addr >> 7) &0x06) | ((pmsg->flags & I2C_M_RD) ? (0xf0 | I2C_READ) : (0xf0 | I2C_WRITE)) ); 	// first 2 bits
			__i2c_set_drf();	// data is ready flag
			while(__i2c_check_drf())
				if(!(pmsg->flags & I2C_M_IGNORE_NAK) && !__i2c_received_ack())
					ret = -EIO;
			__i2c_write(pmsg->addr);	// final 8 bits
			__i2c_set_drf();	// data is ready flag
			while(__i2c_check_drf())
				if(!(pmsg->flags & I2C_M_IGNORE_NAK) && !__i2c_received_ack())
					ret = -EIO;
		}
		else { // 7 bit address
			__i2c_write((pmsg->addr << 1) | ((pmsg->flags & I2C_M_RD) ? I2C_READ : I2C_WRITE));	// set data to be written
			__i2c_set_drf();	// data is ready flag
			while(__i2c_check_drf())
				if(!(pmsg->flags & I2C_M_IGNORE_NAK) && !__i2c_received_ack())
					ret = -EIO;
		}
		
		if (pmsg->flags & I2C_M_RD) {
			while(!__i2c_transmit_ended())	// wait for final ack before starting to read
				if(!(pmsg->flags & I2C_M_IGNORE_NAK) && !__i2c_received_ack())
					ret = -EIO;			
			printk(KERN_INFO "address transmit finished - starting read\n");
		}
		
		if(ret < 0)
			goto error;

		printk(KERN_INFO "address %x sent\n", pmsg->addr);
*/
		tmpbuf = pmsg->buf;
		cnt = (pmsg->flags & I2C_M_TEN) ? -2:-1;	// prepare for sending address
		
		for(; cnt < pmsg->len; cnt++) {
			printk(KERN_INFO "%d (%d)", cnt, pmsg->len);
#if 0
			if(cnt == pmsg->len-1 && i == num-1) {
				printk("  send stop\n");
				__i2c_send_stop();	// last byte of last message				
			}
#endif
			if (cnt >= 0 && pmsg->flags & I2C_M_RD) { // read data
				int timeout = TIMEOUT*10;
				while(!__i2c_check_drf() && timeout--)	// wait for data to arrive
					;
				if (timeout < 0)
					ret = -ETIMEDOUT;
				else {
					if(cnt == pmsg->len-2)
						__i2c_send_nack();	// nack last byte
					*tmpbuf++ = __i2c_read();	// read data byte
					printk("  r: %02x\n", tmpbuf[-1]);
					__i2c_clear_drf();					
				}
			}
			else {
				if(cnt == -2) { // send first byte of 10-bit address
					printk("  w: %02x\n", ((pmsg->addr >> 7) & 0x06) | ((pmsg->flags & I2C_M_RD) ? (0xf0 | I2C_READ) : (0xf0 | I2C_WRITE)));
					__i2c_write(((pmsg->addr >> 7) & 0x06) | ((pmsg->flags & I2C_M_RD) ? (0xf0 | I2C_READ) : (0xf0 | I2C_WRITE)) ); 	// first 2 bits					
				}
				else if(cnt == -1) { // send 7 bit address or second byte
					if ((pmsg->flags & I2C_M_TEN)) {
						printk("  w: %02x\n", pmsg->addr & 0xff);
						__i2c_write(pmsg->addr);	// final 8 bits
					}
					else {
						printk("  w: %02x\n", (pmsg->addr << 1) | ((pmsg->flags & I2C_M_RD) ? I2C_READ : I2C_WRITE));
						__i2c_write((pmsg->addr << 1) | ((pmsg->flags & I2C_M_RD) ? I2C_READ : I2C_WRITE));	// set data to be written				
					}
				}
				else { // send data
					printk("  w: %02x\n", *tmpbuf);
					__i2c_write(*tmpbuf++);
					
				}
				__i2c_set_drf();	// data is ready flag
				int timeout = TIMEOUT*10;
				if(0 || cnt == -1 || cnt == pmsg->len-1) { // last byte, wait for ACK
					printk("  wait transmit_ended\n");
#if 0
					while(__i2c_check_drf())
						;
#endif
					while(!__i2c_transmit_ended() && timeout--)
						if(/*!(pmsg->flags & I2C_M_IGNORE_NAK) &&*/ !__i2c_received_ack()) {
							ret = -EIO;
							break;
						}
				}
				else { // wait until we can push the next byte
					while(__i2c_check_drf() && timeout--)
						if(!(pmsg->flags & I2C_M_IGNORE_NAK) && !__i2c_received_ack()) {
							ret = -EIO;
							break;
						}
					}
				if (timeout < 0)
					ret = -ETIMEDOUT;
			}
			if(ret < 0)
				break;
		}
		dev_dbg(&adap->dev, "transfer complete\n");
	}
	__i2c_send_stop();	// finally, send a stop to release the bus
	if(ret < 0) {
		dev_dbg(&adap->dev, "jz47xx_xfer failed: ret=%d\n", ret);
#if 1
		if(ret == -EIO)
			printk("  EIO\n");
		else if(ret == -ETIMEDOUT)
			printk("  ETIMEDOUT\n");
#endif
	}
	return ret;
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
	printk(KERN_INFO "i2c_jz_probe()\n");
	i2c_jz_setclk(10000); /* default 10 KHz */

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
	printk(KERN_INFO "  adapter id=%d\n", i2c->adap.nr);
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
