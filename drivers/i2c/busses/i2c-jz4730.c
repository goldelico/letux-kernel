/*
 * Ingenic JZ4730 I2C bus driver
 *
 * Copyright (C) 2006 - 2009 Ingenic Semiconductor Inc.
 * Copyright (C) 2015 Imagination Technologies
 * Copyright (C) 2017 Paul Boddie <paul@boddie.org.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/time.h>

#define JZ4730_REG_I2C_DR	0x00
#define JZ4730_REG_I2C_CR	0x04
#define JZ4730_REG_I2C_SR	0x08
#define JZ4730_REG_I2C_GR	0x0C

#define JZ4730_I2C_CR_IEN	BIT(4)
#define JZ4730_I2C_CR_STA	BIT(3)
#define JZ4730_I2C_CR_STO	BIT(2)
#define JZ4730_I2C_CR_AC	BIT(1)
#define JZ4730_I2C_CR_I2CE	BIT(0)

#define JZ4730_I2C_SR_STX	BIT(4)
#define JZ4730_I2C_SR_BUSY	BIT(3)
#define JZ4730_I2C_SR_TEND	BIT(2)
#define JZ4730_I2C_SR_DRF	BIT(1)
#define JZ4730_I2C_SR_ACKF	BIT(0)

#define JZ4730_I2C_TIMEOUT	300	/* ms */

struct jz4730_i2c {
	void __iomem		*iomem;
	int			 irq;
	struct clk		*clk;
	struct i2c_adapter	 adap;
	int			speed;

	struct completion	trans_waitq;
	spinlock_t		lock;

	/* locked members */
	struct i2c_msg		*msg;
	int			data_read;
	unsigned char		*rbuf;
	int			ret;
};

static inline unsigned char jz4730_i2c_readb(struct jz4730_i2c *i2c,
					      unsigned long offset)
{
	return readb(i2c->iomem + offset);
}

static inline void jz4730_i2c_writeb(struct jz4730_i2c *i2c,
				     unsigned long offset, unsigned char val)
{
	writeb(val, i2c->iomem + offset);
}

static inline void jz4730_i2c_writew(struct jz4730_i2c *i2c,
				     unsigned long offset, unsigned short val)
{
	writew(val, i2c->iomem + offset);
}

static void jz4730_i2c_updateb(struct jz4730_i2c *i2c,
				      unsigned long offset,
				      unsigned char affected,
				      unsigned char val)
{
	unsigned long flags;

	spin_lock_irqsave(&i2c->lock, flags);
	writeb((readb(i2c->iomem + offset) & ~affected) | val,
		i2c->iomem + offset);
	spin_unlock_irqrestore(&i2c->lock, flags);
}

static int jz4730_i2c_disable(struct jz4730_i2c *i2c)
{
	jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_CR, JZ4730_I2C_CR_I2CE, 0);
	return 0;
}

static int jz4730_i2c_enable(struct jz4730_i2c *i2c)
{
	jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_CR, JZ4730_I2C_CR_I2CE,
			   JZ4730_I2C_CR_I2CE);
	return 0;
}

static int jz4730_i2c_set_speed(struct jz4730_i2c *i2c)
{
	int dev_clk_khz = clk_get_rate(i2c->clk) / 1000;
	int i2c_clk = i2c->speed;

	if (jz4730_i2c_disable(i2c))
		dev_dbg(&i2c->adap.dev, "i2c not disabled\n");

	/* Set the I2C clock divider. */

	jz4730_i2c_writew(i2c, JZ4730_REG_I2C_GR, dev_clk_khz / (16 * i2c_clk) - 1);

	return 0;
}

static int jz4730_i2c_recv(struct jz4730_i2c *i2c, unsigned char *data)
{
	int timeout = JZ4730_I2C_TIMEOUT;

	/* Wait for data ready. */

	while (timeout-- && !(jz4730_i2c_readb(i2c, JZ4730_REG_I2C_SR) & JZ4730_I2C_SR_DRF))
		udelay(1000);

	if (!timeout)
		return -ETIMEDOUT;

	*data = jz4730_i2c_readb(i2c, JZ4730_REG_I2C_DR);
	return 0;
}

static int jz4730_i2c_send(struct jz4730_i2c *i2c, unsigned char data)
{
	int timeout = JZ4730_I2C_TIMEOUT;

	/* Write data, set data ready. */

	jz4730_i2c_writeb(i2c, JZ4730_REG_I2C_DR, data);

	jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_SR, JZ4730_I2C_SR_DRF,
			   JZ4730_I2C_SR_DRF);

	/* Wait for data to be sent and the data register to clear. */

	while (timeout-- && (jz4730_i2c_readb(i2c, JZ4730_REG_I2C_SR) & JZ4730_I2C_SR_DRF))
		udelay(1000);

	if (!timeout)
		return -ETIMEDOUT;
	else
		return 0;
}

static int jz4730_i2c_wait_ack(struct jz4730_i2c *i2c, struct i2c_msg *msg)
{
	int timeout = JZ4730_I2C_TIMEOUT;

	while (timeout-- && !(jz4730_i2c_readb(i2c, JZ4730_REG_I2C_SR) & JZ4730_I2C_SR_TEND))
		udelay(1000);

	if (!timeout)
		return -ETIMEDOUT;

	/* Test for negative acknowledgement condition. */

	if (!(msg->flags & I2C_M_IGNORE_NAK) &&
	    (jz4730_i2c_readb(i2c, JZ4730_REG_I2C_SR) & JZ4730_I2C_SR_ACKF))
		return -EIO;

	return 0;
}

static int jz4730_i2c_set_target(struct jz4730_i2c *i2c, struct i2c_msg *msg)
{
	int read = msg->flags & I2C_M_RD;
	int ret;

	/* Send 10-bit address or 7-bit address. */

	if (msg->flags & I2C_M_TEN) {

		/* 11110aad where aa = address[9:8] */

		ret = jz4730_i2c_send(i2c, (read ? BIT(0) : 0) | ((msg->addr >> 7) & 0x06) | 0xf0);
		if (ret)
			return ret;

		/* aaaaaaaa = address[7:0] */

		ret = jz4730_i2c_send(i2c, msg->addr & 0xff);

	} else {
		/* aaaaaaad */

		ret = jz4730_i2c_send(i2c, (read ? BIT(0) : 0) | (msg->addr << 1));
	}

	if (ret)
		return ret;

	/* Wait for acknowledgement if reading. */

	if (read)
		return jz4730_i2c_wait_ack(i2c, msg);

	return 0;
}

static void jz4730_i2c_stop(struct jz4730_i2c *i2c)
{
	jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_CR,
			JZ4730_I2C_CR_STO, JZ4730_I2C_CR_STO);
}

static int jz4730_i2c_cleanup(struct jz4730_i2c *i2c)
{
	/* Disable interrupt and device. */

	jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_CR, JZ4730_I2C_CR_IEN, 0);
	return jz4730_i2c_disable(i2c);
}

static void jz4730_i2c_trans_done(struct jz4730_i2c *i2c)
{
	/* Reset interrupt enable and record completion. */

	jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_CR, JZ4730_I2C_CR_IEN, 0);
	complete(&i2c->trans_waitq);
}

static irqreturn_t jz4730_i2c_irq(int irqno, void *dev_id)
{
	int ret;
	unsigned char data;
	struct jz4730_i2c *i2c = dev_id;
	unsigned long flags;

	spin_lock_irqsave(&i2c->lock, flags);

	/* Test for earlier failure. */

	if (i2c->ret)
		goto done;

	/* Test for superfluous data. */

	if (i2c->data_read >= i2c->msg->len)
		goto done;

	/* Test for incoming read. */

	if (!(i2c->msg->flags & I2C_M_RD) &&
	    (jz4730_i2c_readb(i2c, JZ4730_REG_I2C_SR) & JZ4730_I2C_SR_DRF)) {

		/* Read data and check status. */

		ret = jz4730_i2c_recv(i2c, &data);
		if (ret) {
			i2c->ret = ret;
			goto done;
		}

		/* Store and count data. */

		*(i2c->rbuf++) = data;
		i2c->data_read++;

		/* Assert non-acknowledgement condition before final byte. */

		if (i2c->data_read == i2c->msg->len - 1) {
			jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_CR,
					JZ4730_I2C_CR_AC, JZ4730_I2C_CR_AC);
		}

		/* Clear data ready condition. */

		jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_SR, JZ4730_I2C_SR_DRF, 0);

		if (i2c->data_read == i2c->msg->len) {
			jz4730_i2c_trans_done(i2c);
			goto done;
		}
	}

done:
	spin_unlock_irqrestore(&i2c->lock, flags);
	return IRQ_HANDLED;
}

static int jz4730_i2c_xfer_read(struct jz4730_i2c *i2c,
				       struct i2c_msg *msg)
{
	unsigned long flags;
	unsigned char *buf = msg->buf;
	unsigned int len = msg->len;
	int wait_time = JZ4730_I2C_TIMEOUT * (len + 5);
	long timeout;

	memset(buf, 0, len);

	/* Initialise transfer state. */

	spin_lock_irqsave(&i2c->lock, flags);

	i2c->msg = msg;
	i2c->data_read = 0;
	i2c->rbuf = buf;
	i2c->ret = 0;

	spin_unlock_irqrestore(&i2c->lock, flags);

	/* Wait for the transfer to occur in the background. */

	timeout = wait_for_completion_timeout(&i2c->trans_waitq,
					      msecs_to_jiffies(wait_time));

	if (!timeout) {
		dev_err(&i2c->adap.dev, "irq read timeout\n");
		return -EIO;
	}

	return i2c->ret;
}

static int jz4730_i2c_xfer_write(struct jz4730_i2c *i2c,
				        struct i2c_msg *msg)
{
	int ret = 0;
	unsigned char *buf = msg->buf;
	unsigned int len = msg->len;
	unsigned int i;

	/* Transfer written data synchronously. */

	for (i = 0; i < len; i++) {

		ret = jz4730_i2c_send(i2c, *(buf++));
		if (ret) {
			jz4730_i2c_stop(i2c);
			return -EIO;
		}

		/* Terminate the transfer upon non-acknowledgement. */

		if (jz4730_i2c_readb(i2c, JZ4730_REG_I2C_SR) & JZ4730_I2C_SR_ACKF) {
			jz4730_i2c_stop(i2c);
			return 0;
		}
	}

	/* Send stop condition. */

	jz4730_i2c_stop(i2c);

	/* Wait for final acknowledgement for the final byte. */

	return jz4730_i2c_wait_ack(i2c, msg);
}

static int jz4730_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msg,
			   int count)
{
	struct jz4730_i2c *i2c = adap->algo_data;
	int i;
	int ret = 0;

	/* Enable interrupt handling. */

	jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_CR, JZ4730_I2C_CR_IEN,
			   JZ4730_I2C_CR_IEN);

	/* Send messages. */

	for (i = 0; i < count; i++, msg++) {

		/* Send start. */

		if (!(msg->flags & I2C_M_NOSTART))
			jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_CR,
					JZ4730_I2C_CR_STO, JZ4730_I2C_CR_STO);

		/* Set acknowledge level. */

		jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_CR, JZ4730_I2C_CR_AC, 0);

		/* Send address and direction. */

		ret = jz4730_i2c_set_target(i2c, msg);
		if (ret) goto out;

		/* Schedule/perform each read/write. */

		if (msg->flags & I2C_M_RD)
			ret = jz4730_i2c_xfer_read(i2c, msg);
		else
			ret = jz4730_i2c_xfer_write(i2c, msg);

		if (ret) goto out;
	}

	/* Set the number of transferred messages. */

	ret = i;

out:
	jz4730_i2c_cleanup(i2c);
	return ret;
}

static u32 jz4730_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm jz4730_i2c_algorithm = {
	.master_xfer	= jz4730_i2c_xfer,
	.functionality	= jz4730_i2c_functionality,
};

static const struct of_device_id jz4730_i2c_of_matches[] = {
	{ .compatible = "ingenic,jz4730-i2c", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, jz4730_i2c_of_matches);

static int jz4730_i2c_probe(struct platform_device *pdev)
{
	int ret;
	unsigned int clk_freq;
	struct resource *r;
	struct jz4730_i2c *i2c;

	i2c = devm_kzalloc(&pdev->dev, sizeof(struct jz4730_i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->adap.owner		= THIS_MODULE;
	i2c->adap.algo		= &jz4730_i2c_algorithm;
	i2c->adap.algo_data	= i2c;
	i2c->adap.retries	= 5;
	i2c->adap.dev.parent	= &pdev->dev;
	i2c->adap.dev.of_node	= pdev->dev.of_node;
	sprintf(i2c->adap.name, "%s", pdev->name);

	init_completion(&i2c->trans_waitq);
	spin_lock_init(&i2c->lock);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i2c->iomem = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(i2c->iomem))
		return PTR_ERR(i2c->iomem);

	platform_set_drvdata(pdev, i2c);

	i2c->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(i2c->clk))
		return PTR_ERR(i2c->clk);

	ret = clk_prepare_enable(i2c->clk);
	if (ret)
		return ret;

	ret = of_property_read_u32(pdev->dev.of_node, "clock-frequency",
				   &clk_freq);
	if (ret) {
		dev_err(&pdev->dev, "clock-frequency not specified in DT\n");
		goto err;
	}

	i2c->speed = clk_freq / 1000;
	if (i2c->speed == 0) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "clock-frequency minimum is 1000\n");
		goto err;
	}
	dev_info(&pdev->dev, "Bus frequency is %d kHz\n", i2c->speed);

	/* Configure and enable the peripheral. */

	jz4730_i2c_set_speed(i2c);
	jz4730_i2c_enable(i2c);

	i2c->irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, i2c->irq, jz4730_i2c_irq, 0,
			       dev_name(&pdev->dev), i2c);
	if (ret)
		goto err;

	ret = i2c_add_adapter(&i2c->adap);
	if (ret < 0)
		goto err;

	return 0;

err:
	clk_disable_unprepare(i2c->clk);
	return ret;
}

static int jz4730_i2c_remove(struct platform_device *pdev)
{
	struct jz4730_i2c *i2c = platform_get_drvdata(pdev);

	clk_disable_unprepare(i2c->clk);
	i2c_del_adapter(&i2c->adap);
	return 0;
}

static struct platform_driver jz4730_i2c_driver = {
	.probe		= jz4730_i2c_probe,
	.remove		= jz4730_i2c_remove,
	.driver		= {
		.name	= "jz4730-i2c",
		.of_match_table = of_match_ptr(jz4730_i2c_of_matches),
	},
};

module_platform_driver(jz4730_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Paul Boddie <paul@boddie.org.uk>");
MODULE_DESCRIPTION("I2C driver for JZ4730 SoC");
