/*
 * Ingenic JZ4730 I2C bus driver
 *
 * Copyright (C) 2006 - 2009 Ingenic Semiconductor Inc.
 * Copyright (C) 2015 Imagination Technologies
 * Copyright (C) 2017 Paul Boddie <paul@boddie.org.uk>
 * Copyright (C) 2020 H. Nikolaus Schaller <hns@goldelico.com>
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

enum {
	STATE_IDLE,
	STATE_SEND_ADDR,
	STATE_SEND_ADDR10,
	STATE_WRITE,
	STATE_READ,
	STATE_DONE
};

struct jz4730_i2c {
	void __iomem		*iomem;
	struct clk		*clk;
	struct i2c_adapter	adap;
	int			speed;

	struct completion	trans_waitq;
	spinlock_t		lock;

	/* locked members */
	struct i2c_msg		*msg;
	int			state;
	int			pos;
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

static int jz4730_i2c_set_speed(struct jz4730_i2c *i2c)
{
	int dev_clk_khz = clk_get_rate(i2c->clk) / 1000;
	int i2c_clk = i2c->speed;

	/* Set the I2C clock divider. */

	// FIXME: check for overflow - we only have 16 bit divider resolution?
	jz4730_i2c_writew(i2c, JZ4730_REG_I2C_GR, dev_clk_khz / (16 * i2c_clk) - 1);

	return 0;
}

static irqreturn_t jz4730_i2c_irq(int irqno, void *dev_id)
{
	struct jz4730_i2c *i2c = dev_id;
	struct i2c_msg *msg;
	unsigned long flags;
	u8 status;

	spin_lock_irqsave(&i2c->lock, flags);

	msg = i2c->msg;

	status = jz4730_i2c_readb(i2c, JZ4730_REG_I2C_SR);

	switch (i2c->state) {
		case STATE_IDLE:
			dev_err(&i2c->adap.dev, "spurious interrupt during idle state\n");
			/* stop interrupt flooding */
			jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_CR, JZ4730_I2C_CR_IEN, 0);
			break;

		case STATE_DONE:

			if (status & JZ4730_I2C_SR_ACKF)
				i2c->ret = -EIO;	/* there was no positive ACK */

			/* clear data ready status if we received an extra byte */
			jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_SR, JZ4730_I2C_SR_DRF, 0);

			i2c->state = STATE_IDLE;

			complete(&i2c->trans_waitq);

			break;

		case STATE_SEND_ADDR:
			if (msg->flags & I2C_M_TEN) {
				/* 11110aad where aa = address[9:8] abd d = direction */
				jz4730_i2c_writeb(i2c, JZ4730_REG_I2C_DR,
					(msg->flags & I2C_M_RD ? BIT(0) : 0) |
					((msg->addr >> 7) & 0x06) | 0xf0);
				i2c->state = STATE_SEND_ADDR10;
			}
			else {
				/* aaaaaaad */
				jz4730_i2c_writeb(i2c, JZ4730_REG_I2C_DR,
					(msg->flags & I2C_M_RD ? BIT(0) : 0) |
					(msg->addr << 1));
				if (msg->flags & I2C_M_RD)
					i2c->state = STATE_READ;
				else
					i2c->state = STATE_WRITE;
			}
			jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_SR, JZ4730_I2C_SR_DRF,
					    JZ4730_I2C_SR_DRF);
			break;

		case STATE_SEND_ADDR10:
			/* aaaaaaaa */
			jz4730_i2c_writeb(i2c, JZ4730_REG_I2C_DR, msg->addr & 0xff);
			jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_SR, JZ4730_I2C_SR_DRF,
					    JZ4730_I2C_SR_DRF);

			if (msg->flags & I2C_M_RD)
				i2c->state = STATE_READ;
			else
				i2c->state = STATE_WRITE;
			break;

		case STATE_WRITE:
			if (i2c->pos < msg->len) {
				jz4730_i2c_writeb(i2c, JZ4730_REG_I2C_DR, msg->buf[i2c->pos++]);
				jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_SR, JZ4730_I2C_SR_DRF,
				   JZ4730_I2C_SR_DRF);
			}
			else {
				// send stop bit unless we have another message
				jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_CR,
					JZ4730_I2C_CR_STO, JZ4730_I2C_CR_STO);
				/* FIXME: how can we make SR: BUSY + TEND go away? */
				i2c->state = STATE_DONE;
			}
			break;

		case STATE_READ:
			if (i2c->pos == msg->len - 1) {
				/* Assert non-acknowledgement condition before final byte. */
				jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_CR,
						JZ4730_I2C_CR_AC, JZ4730_I2C_CR_AC);
			}
			if (i2c->pos < msg->len) {
				msg->buf[i2c->pos++] = jz4730_i2c_readb(i2c, JZ4730_REG_I2C_DR);
				jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_SR, JZ4730_I2C_SR_DRF, 0);
			}
			else {
				i2c->state = STATE_DONE;
			}
			break;
	}

	spin_unlock_irqrestore(&i2c->lock, flags);
	return IRQ_HANDLED;
}

static int jz4730_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msg,
			   int count)
{
	struct jz4730_i2c *i2c = adap->algo_data;
	int i;

	/* first message must request a START */
	if (count > 0 && msg->flags & I2C_M_NOSTART)
		return -EINVAL;

	/* Send/Receive messages. */

	for (i = 0; i < count; i++, msg++) {

		int wait_time = 8 * (msg->len + 5) / i2c->speed + 1;
		long timeout;

// SPINLOCK? (not if we disable interrupts after each operation

		/* start with sending address */
		i2c->state = STATE_SEND_ADDR;
		i2c->msg = msg;
		i2c->pos = 0;
		i2c->ret = 0;

		/* Set acknowledge level. */

		jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_CR, JZ4730_I2C_CR_AC, 0);

		/* Send start. */

		if (!(msg->flags & I2C_M_NOSTART))
			jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_CR,
					JZ4730_I2C_CR_STA, JZ4730_I2C_CR_STA);

		/* Wait for the transfer to occur in the background. */

		timeout = wait_for_completion_timeout(&i2c->trans_waitq,
					      msecs_to_jiffies(wait_time));

		if (!timeout) {
			dev_err(&i2c->adap.dev, "irq timeout %ld\n", timeout);
			return -ETIMEDOUT;
		}

		if (i2c->ret)
			return i2c->ret;
	}

	return i;
}

static u32 jz4730_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_10BIT_ADDR /* | I2C_FUNC_NOSTART */;
}

static const struct i2c_algorithm jz4730_i2c_algorithm = {
	.master_xfer	= jz4730_i2c_xfer,
	.functionality	= jz4730_i2c_functionality,
};

static const struct of_device_id jz4730_i2c_of_matches[] = {
	{ .compatible = "ingenic,jz4730-i2c", },
	/* JZ4740 has the same I2C controller so we do not need to differentiate */
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, jz4730_i2c_of_matches);

static int jz4730_i2c_probe(struct platform_device *pdev)
{
	int ret;
	int irq;
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

	irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, irq, jz4730_i2c_irq,
			       IRQF_TRIGGER_NONE, dev_name(&pdev->dev), i2c);
	if (ret)
		goto err;

	ret = i2c_add_adapter(&i2c->adap);
	if (ret < 0)
		goto err;

	jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_CR, JZ4730_I2C_CR_IEN | JZ4730_I2C_CR_I2CE,
			    JZ4730_I2C_CR_IEN | JZ4730_I2C_CR_I2CE);

	return 0;

err:
	clk_disable_unprepare(i2c->clk);
	return ret;
}

static int jz4730_i2c_remove(struct platform_device *pdev)
{
	struct jz4730_i2c *i2c = platform_get_drvdata(pdev);

	jz4730_i2c_updateb(i2c, JZ4730_REG_I2C_CR, JZ4730_I2C_CR_IEN | JZ4730_I2C_CR_I2CE, 0);
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
