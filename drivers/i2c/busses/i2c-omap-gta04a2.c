/*
 * Bitbanging I2C bus driver using the GPIO API
 *
 * Copyright (C) 2007 Atmel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <asm/gpio.h>

#if !defined(CONFIG_I2C_OMAP_GTA04A2)

#include <linux/i2c-gpio.h>

/* Toggle SDA by changing the direction of the pin */
static void i2c_gpio_setsda_dir(void *data, int state)
{
	struct i2c_gpio_platform_data *pdata = data;

	if (state)
		gpio_direction_input(pdata->sda_pin);
	else
		gpio_direction_output(pdata->sda_pin, 0);
}

/*
 * Toggle SDA by changing the output value of the pin. This is only
 * valid for pins configured as open drain (i.e. setting the value
 * high effectively turns off the output driver.)
 */
static void i2c_gpio_setsda_val(void *data, int state)
{
	struct i2c_gpio_platform_data *pdata = data;

	gpio_set_value(pdata->sda_pin, state);
}

/* Toggle SCL by changing the direction of the pin. */
static void i2c_gpio_setscl_dir(void *data, int state)
{
	struct i2c_gpio_platform_data *pdata = data;

	if (state)
		gpio_direction_input(pdata->scl_pin);
	else
		gpio_direction_output(pdata->scl_pin, 0);
}

/*
 * Toggle SCL by changing the output value of the pin. This is used
 * for pins that are configured as open drain and for output-only
 * pins. The latter case will break the i2c protocol, but it will
 * often work in practice.
 */
static void i2c_gpio_setscl_val(void *data, int state)
{
	struct i2c_gpio_platform_data *pdata = data;

	gpio_set_value(pdata->scl_pin, state);
}

static int i2c_gpio_getsda(void *data)
{
	struct i2c_gpio_platform_data *pdata = data;

	return gpio_get_value(pdata->sda_pin);
}

static int i2c_gpio_getscl(void *data)
{
	struct i2c_gpio_platform_data *pdata = data;

	return gpio_get_value(pdata->scl_pin);
}
#else

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/io.h>
#include <linux/i2c-omap.h>

#define DEBUG

/* use I2C register in test mode */
/* for GTA04A2 we have to swap I2C1 */
/* for Beagle Hybrid we don't swap and can test I2C1 and I2C2 */

/* I2C controller revisions */
#define OMAP_I2C_REV_2			0x20

/* I2C controller revisions present on specific hardware */
#define OMAP_I2C_REV_ON_2430		0x36
#define OMAP_I2C_REV_ON_3430		0x3C

/* timeout waiting for the controller to respond */
#define OMAP_I2C_TIMEOUT (msecs_to_jiffies(1000))

#define OMAP_I2C_REV_REG		0x00
#define OMAP_I2C_IE_REG			0x04
#define OMAP_I2C_STAT_REG		0x08
#define OMAP_I2C_IV_REG			0x0c
/* For OMAP3 I2C_IV has changed to I2C_WE (wakeup enable) */
#define OMAP_I2C_WE_REG			0x0c
#define OMAP_I2C_SYSS_REG		0x10
#define OMAP_I2C_BUF_REG		0x14
#define OMAP_I2C_CNT_REG		0x18
#define OMAP_I2C_DATA_REG		0x1c
#define OMAP_I2C_SYSC_REG		0x20
#define OMAP_I2C_CON_REG		0x24
#define OMAP_I2C_OA_REG			0x28
#define OMAP_I2C_SA_REG			0x2c
#define OMAP_I2C_PSC_REG		0x30
#define OMAP_I2C_SCLL_REG		0x34
#define OMAP_I2C_SCLH_REG		0x38
#define OMAP_I2C_SYSTEST_REG		0x3c
#define OMAP_I2C_BUFSTAT_REG		0x40

/* I2C Interrupt Enable Register (OMAP_I2C_IE): */
#define OMAP_I2C_IE_XDR		(1 << 14)	/* TX Buffer drain int enable */
#define OMAP_I2C_IE_RDR		(1 << 13)	/* RX Buffer drain int enable */
#define OMAP_I2C_IE_XRDY	(1 << 4)	/* TX data ready int enable */
#define OMAP_I2C_IE_RRDY	(1 << 3)	/* RX data ready int enable */
#define OMAP_I2C_IE_ARDY	(1 << 2)	/* Access ready int enable */
#define OMAP_I2C_IE_NACK	(1 << 1)	/* No ack interrupt enable */
#define OMAP_I2C_IE_AL		(1 << 0)	/* Arbitration lost int ena */

/* I2C Status Register (OMAP_I2C_STAT): */
#define OMAP_I2C_STAT_XDR	(1 << 14)	/* TX Buffer draining */
#define OMAP_I2C_STAT_RDR	(1 << 13)	/* RX Buffer draining */
#define OMAP_I2C_STAT_BB	(1 << 12)	/* Bus busy */
#define OMAP_I2C_STAT_ROVR	(1 << 11)	/* Receive overrun */
#define OMAP_I2C_STAT_XUDF	(1 << 10)	/* Transmit underflow */
#define OMAP_I2C_STAT_AAS	(1 << 9)	/* Address as slave */
#define OMAP_I2C_STAT_AD0	(1 << 8)	/* Address zero */
#define OMAP_I2C_STAT_XRDY	(1 << 4)	/* Transmit data ready */
#define OMAP_I2C_STAT_RRDY	(1 << 3)	/* Receive data ready */
#define OMAP_I2C_STAT_ARDY	(1 << 2)	/* Register access ready */
#define OMAP_I2C_STAT_NACK	(1 << 1)	/* No ack interrupt enable */
#define OMAP_I2C_STAT_AL	(1 << 0)	/* Arbitration lost int ena */

/* I2C WE wakeup enable register */
#define OMAP_I2C_WE_XDR_WE	(1 << 14)	/* TX drain wakup */
#define OMAP_I2C_WE_RDR_WE	(1 << 13)	/* RX drain wakeup */
#define OMAP_I2C_WE_AAS_WE	(1 << 9)	/* Address as slave wakeup*/
#define OMAP_I2C_WE_BF_WE	(1 << 8)	/* Bus free wakeup */
#define OMAP_I2C_WE_STC_WE	(1 << 6)	/* Start condition wakeup */
#define OMAP_I2C_WE_GC_WE	(1 << 5)	/* General call wakeup */
#define OMAP_I2C_WE_DRDY_WE	(1 << 3)	/* TX/RX data ready wakeup */
#define OMAP_I2C_WE_ARDY_WE	(1 << 2)	/* Reg access ready wakeup */
#define OMAP_I2C_WE_NACK_WE	(1 << 1)	/* No acknowledgment wakeup */
#define OMAP_I2C_WE_AL_WE	(1 << 0)	/* Arbitration lost wakeup */

#define OMAP_I2C_WE_ALL		(OMAP_I2C_WE_XDR_WE | OMAP_I2C_WE_RDR_WE | \
OMAP_I2C_WE_AAS_WE | OMAP_I2C_WE_BF_WE | \
OMAP_I2C_WE_STC_WE | OMAP_I2C_WE_GC_WE | \
OMAP_I2C_WE_DRDY_WE | OMAP_I2C_WE_ARDY_WE | \
OMAP_I2C_WE_NACK_WE | OMAP_I2C_WE_AL_WE)

/* I2C Buffer Configuration Register (OMAP_I2C_BUF): */
#define OMAP_I2C_BUF_RDMA_EN	(1 << 15)	/* RX DMA channel enable */
#define OMAP_I2C_BUF_RXFIF_CLR	(1 << 14)	/* RX FIFO Clear */
#define OMAP_I2C_BUF_XDMA_EN	(1 << 7)	/* TX DMA channel enable */
#define OMAP_I2C_BUF_TXFIF_CLR	(1 << 6)	/* TX FIFO Clear */

/* I2C Configuration Register (OMAP_I2C_CON): */
#define OMAP_I2C_CON_EN		(1 << 15)	/* I2C module enable */
#define OMAP_I2C_CON_BE		(1 << 14)	/* Big endian mode */
#define OMAP_I2C_CON_OPMODE_HS	(1 << 12)	/* High Speed support */
#define OMAP_I2C_CON_STB	(1 << 11)	/* Start byte mode (master) */
#define OMAP_I2C_CON_MST	(1 << 10)	/* Master/slave mode */
#define OMAP_I2C_CON_TRX	(1 << 9)	/* TX/RX mode (master only) */
#define OMAP_I2C_CON_XA		(1 << 8)	/* Expand address */
#define OMAP_I2C_CON_RM		(1 << 2)	/* Repeat mode (master only) */
#define OMAP_I2C_CON_STP	(1 << 1)	/* Stop cond (master only) */
#define OMAP_I2C_CON_STT	(1 << 0)	/* Start condition (master) */

/* I2C SCL time value when Master */
#define OMAP_I2C_SCLL_HSSCLL	8
#define OMAP_I2C_SCLH_HSSCLH	8

/* I2C System Test Register (OMAP_I2C_SYSTEST): */
#ifdef DEBUG
#define OMAP_I2C_SYSTEST_ST_EN		(1 << 15)	/* System test enable */
#define OMAP_I2C_SYSTEST_FREE		(1 << 14)	/* Free running mode */
#define OMAP_I2C_SYSTEST_TMODE_MASK	(3 << 12)	/* Test mode select */
#define OMAP_I2C_SYSTEST_TMODE_SHIFT	(12)		/* Test mode select */
#define OMAP_I2C_SYSTEST_SCL_I		(1 << 3)	/* SCL line sense in */
#define OMAP_I2C_SYSTEST_SCL_O		(1 << 2)	/* SCL line drive out */
#define OMAP_I2C_SYSTEST_SDA_I		(1 << 1)	/* SDA line sense in */
#define OMAP_I2C_SYSTEST_SDA_O		(1 << 0)	/* SDA line drive out */
#endif

/* OCP_SYSSTATUS bit definitions */
#define SYSS_RESETDONE_MASK		(1 << 0)

/* OCP_SYSCONFIG bit definitions */
#define SYSC_CLOCKACTIVITY_MASK		(0x3 << 8)
#define SYSC_SIDLEMODE_MASK		(0x3 << 3)
#define SYSC_ENAWAKEUP_MASK		(1 << 2)
#define SYSC_SOFTRESET_MASK		(1 << 1)
#define SYSC_AUTOIDLE_MASK		(1 << 0)

#define SYSC_IDLEMODE_SMART		0x2
#define SYSC_CLOCKACTIVITY_FCLK		0x2


struct omap_i2c_dev {
	struct device		*dev;
	void __iomem		*base;		/* virtual */
	int			irq;
	struct clk		*iclk;		/* Interface clock */
	struct clk		*fclk;		/* Functional clock */
	struct completion	cmd_complete;
	struct resource		*ioarea;
	u32			latency;	/* maximum mpu wkup latency */
	void			(*set_mpu_wkup_lat)(struct device *dev,
										int latency);
	u32			speed;		/* Speed of bus in Khz */
	u16			cmd_err;
	u8			*buf;
	size_t			buf_len;
	struct i2c_adapter	adapter;
	u8			fifo_size;	/* use as flag and value
							 * fifo_size==0 implies no fifo
							 * if set, should be trsh+1
							 */
	u8			rev;
	unsigned		b_hw:1;		/* bad h/w fixes */
	unsigned		idle:1;
	u16			iestate;	/* Saved interrupt register */
	u16			pscstate;
	u16			scllstate;
	u16			sclhstate;
	u16			bufstate;
	u16			syscstate;
	u16			westate;
};

static inline void omap_i2c_write_reg(struct omap_i2c_dev *i2c_dev,
									  int reg, u16 val)
{
	__raw_writew(val, i2c_dev->base + reg);
}

static inline u16 omap_i2c_read_reg(struct omap_i2c_dev *i2c_dev, int reg)
{
	return __raw_readw(i2c_dev->base + reg);
}

static int __init omap_i2c_get_clocks(struct omap_i2c_dev *dev)
{
	int ret;
	
	dev->iclk = clk_get(dev->dev, "ick");
	if (IS_ERR(dev->iclk)) {
		ret = PTR_ERR(dev->iclk);
		dev->iclk = NULL;
		printk("I2C bitbang fails to get ick\n");
		printk("dev=%p\n", dev);
		printk("dev->dev=%p\n", dev->dev);
		printk("dev->dev->init_name=%s\n", dev->dev->init_name);
		return ret;
	}
	
	dev->fclk = clk_get(dev->dev, "fck");
	if (IS_ERR(dev->fclk)) {
		ret = PTR_ERR(dev->fclk);
		if (dev->iclk != NULL) {
			clk_put(dev->iclk);
			dev->iclk = NULL;
		}
		printk("I2C bitbang fails to get fck\n");
		dev->fclk = NULL;
		return ret;
	}
	
	return 0;
}

static void omap_i2c_put_clocks(struct omap_i2c_dev *dev)
{
	clk_put(dev->fclk);
	dev->fclk = NULL;
	clk_put(dev->iclk);
	dev->iclk = NULL;
}


static void omap_i2c_unidle(struct omap_i2c_dev *dev)
{
	printk("omap_i2c_unidle\n");	// convert into device data pointer
	WARN_ON(!dev->idle);
	
	clk_enable(dev->iclk);
	clk_enable(dev->fclk);
	if (cpu_is_omap34xx()) {
		omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, 0);
		omap_i2c_write_reg(dev, OMAP_I2C_PSC_REG, dev->pscstate);
		omap_i2c_write_reg(dev, OMAP_I2C_SCLL_REG, dev->scllstate);
		omap_i2c_write_reg(dev, OMAP_I2C_SCLH_REG, dev->sclhstate);
		omap_i2c_write_reg(dev, OMAP_I2C_BUF_REG, dev->bufstate);
		omap_i2c_write_reg(dev, OMAP_I2C_SYSC_REG, dev->syscstate);
		omap_i2c_write_reg(dev, OMAP_I2C_WE_REG, dev->westate);
		omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, OMAP_I2C_CON_EN);
	}
	dev->idle = 0;
	omap_i2c_write_reg(dev, OMAP_I2C_IE_REG, dev->iestate);
}

static int omap_i2c_init(struct omap_i2c_dev *dev)
{
	u16 psc = 0, scll = 0, sclh = 0, buf = 0;
	u16 fsscll = 0, fssclh = 0, hsscll = 0, hssclh = 0;
	unsigned long fclk_rate = 12000000;
	unsigned long timeout;
	unsigned long internal_clk = 0;
	
	if (dev->rev >= OMAP_I2C_REV_2) {
		omap_i2c_write_reg(dev, OMAP_I2C_SYSC_REG, SYSC_SOFTRESET_MASK);
		/* For some reason we need to set the EN bit before the
		 * reset done bit gets set. */
		timeout = jiffies + OMAP_I2C_TIMEOUT;
		omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, OMAP_I2C_CON_EN);
		while (!(omap_i2c_read_reg(dev, OMAP_I2C_SYSS_REG) &
				 SYSS_RESETDONE_MASK)) {
			if (time_after(jiffies, timeout)) {
				dev_warn(dev->dev, "timeout waiting "
						 "for controller reset\n");
				return -ETIMEDOUT;
			}
			msleep(1);
		}
		
		/* SYSC register is cleared by the reset; rewrite it */
		if (dev->rev == OMAP_I2C_REV_ON_2430) {
			
			omap_i2c_write_reg(dev, OMAP_I2C_SYSC_REG,
							   SYSC_AUTOIDLE_MASK);
			
		} else if (dev->rev >= OMAP_I2C_REV_ON_3430) {
			dev->syscstate = SYSC_AUTOIDLE_MASK;
			dev->syscstate |= SYSC_ENAWAKEUP_MASK;
			dev->syscstate |= (SYSC_IDLEMODE_SMART <<
							   __ffs(SYSC_SIDLEMODE_MASK));
			dev->syscstate |= (SYSC_CLOCKACTIVITY_FCLK <<
							   __ffs(SYSC_CLOCKACTIVITY_MASK));
			
			omap_i2c_write_reg(dev, OMAP_I2C_SYSC_REG,
							   dev->syscstate);
			/*
			 * Enabling all wakup sources to stop I2C freezing on
			 * WFI instruction.
			 * REVISIT: Some wkup sources might not be needed.
			 */
			dev->westate = OMAP_I2C_WE_ALL;
			omap_i2c_write_reg(dev, OMAP_I2C_WE_REG, dev->westate);
		}
	}
	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, 0);
	
	if (cpu_class_is_omap1()) {
		/*
		 * The I2C functional clock is the armxor_ck, so there's
		 * no need to get "armxor_ck" separately.  Now, if OMAP2420
		 * always returns 12MHz for the functional clock, we can
		 * do this bit unconditionally.
		 */
		fclk_rate = clk_get_rate(dev->fclk);
		
		/* TRM for 5912 says the I2C clock must be prescaled to be
		 * between 7 - 12 MHz. The XOR input clock is typically
		 * 12, 13 or 19.2 MHz. So we should have code that produces:
		 *
		 * XOR MHz	Divider		Prescaler
		 * 12		1		0
		 * 13		2		1
		 * 19.2		2		1
		 */
		if (fclk_rate > 12000000)
			psc = fclk_rate / 12000000;
	}
	
	if (cpu_is_omap2430() || cpu_is_omap34xx()) {
		
		/*
		 * HSI2C controller internal clk rate should be 19.2 Mhz for
		 * HS and for all modes on 2430. On 34xx we can use lower rate
		 * to get longer filter period for better noise suppression.
		 * The filter is iclk (fclk for HS) period.
		 */
		if (dev->speed > 400 || cpu_is_omap2430())
			internal_clk = 19200;
		else if (dev->speed > 100)
			internal_clk = 9600;
		else
			internal_clk = 4000;
		fclk_rate = clk_get_rate(dev->fclk) / 1000;
		
		/* Compute prescaler divisor */
		psc = fclk_rate / internal_clk;
		psc = psc - 1;
		
		/* If configured for High Speed */
		if (dev->speed > 400) {
			unsigned long scl;
			
			/* For first phase of HS mode */
			scl = internal_clk / 400;
			fsscll = scl - (scl / 3) - 7;
			fssclh = (scl / 3) - 5;
			
			/* For second phase of HS mode */
			scl = fclk_rate / dev->speed;
			hsscll = scl - (scl / 3) - 7;
			hssclh = (scl / 3) - 5;
		} else if (dev->speed > 100) {
			unsigned long scl;
			
			/* Fast mode */
			scl = internal_clk / dev->speed;
			fsscll = scl - (scl / 3) - 7;
			fssclh = (scl / 3) - 5;
		} else {
			/* Standard mode */
			fsscll = internal_clk / (dev->speed * 2) - 7;
			fssclh = internal_clk / (dev->speed * 2) - 5;
		}
		scll = (hsscll << OMAP_I2C_SCLL_HSSCLL) | fsscll;
		sclh = (hssclh << OMAP_I2C_SCLH_HSSCLH) | fssclh;
	} else {
		/* Program desired operating rate */
		fclk_rate /= (psc + 1) * 1000;
		if (psc > 2)
			psc = 2;
		scll = fclk_rate / (dev->speed * 2) - 7 + psc;
		sclh = fclk_rate / (dev->speed * 2) - 7 + psc;
	}
	
	/* Setup clock prescaler to obtain approx 12MHz I2C module clock: */
	omap_i2c_write_reg(dev, OMAP_I2C_PSC_REG, psc);
	
	/* SCL low and high time values */
	omap_i2c_write_reg(dev, OMAP_I2C_SCLL_REG, scll);
	omap_i2c_write_reg(dev, OMAP_I2C_SCLH_REG, sclh);
	
	if (dev->fifo_size) {
		/* Note: setup required fifo size - 1. RTRSH and XTRSH */
		buf = (dev->fifo_size - 1) << 8 | OMAP_I2C_BUF_RXFIF_CLR |
		(dev->fifo_size - 1) | OMAP_I2C_BUF_TXFIF_CLR;
		omap_i2c_write_reg(dev, OMAP_I2C_BUF_REG, buf);
	}
	
	/* Take the I2C module out of reset: */
	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, OMAP_I2C_CON_EN);
	
	/* Enable interrupts */
	dev->iestate = (OMAP_I2C_IE_XRDY | OMAP_I2C_IE_RRDY |
					OMAP_I2C_IE_ARDY | OMAP_I2C_IE_NACK |
					OMAP_I2C_IE_AL)  | ((dev->fifo_size) ?
										(OMAP_I2C_IE_RDR | OMAP_I2C_IE_XDR) : 0);
	omap_i2c_write_reg(dev, OMAP_I2C_IE_REG, dev->iestate);
	if (cpu_is_omap34xx()) {
		dev->pscstate = psc;
		dev->scllstate = scll;
		dev->sclhstate = sclh;
		dev->bufstate = buf;
	}
	
	omap_i2c_write_reg(dev, OMAP_I2C_SYSTEST_REG, (OMAP_I2C_SYSTEST_ST_EN | OMAP_I2C_SYSTEST_FREE | (3 << OMAP_I2C_SYSTEST_TMODE_SHIFT)) | OMAP_I2C_SYSTEST_SDA_O | OMAP_I2C_SYSTEST_SCL_O);	// initialize systest mode

	return 0;
}


static int __init
omap_i2c_probe(struct platform_device *pdev)	// should be called from i2c_gpio_probe to initialize the controller
{
	struct omap_i2c_dev	*dev;
//	struct i2c_adapter	*adap;
	struct resource		*mem, *irq, *ioarea;
	struct omap_i2c_bus_platform_data *pdata = pdev->dev.platform_data;
//	irq_handler_t isr;
	int r;
	u32 speed = 0;
	
	/* NOTE: driver uses the static register mapping */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return -ENODEV;
	}
	
	ioarea = request_mem_region(mem->start, resource_size(mem),
								pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "I2C region already claimed\n");
		return -EBUSY;
	}
	
	dev = kzalloc(sizeof(struct omap_i2c_dev), GFP_KERNEL);
	if (!dev) {
		r = -ENOMEM;
		goto err_release_region;
	}
	
	if (pdata != NULL) {
		speed = pdata->clkrate;
		dev->set_mpu_wkup_lat = pdata->set_mpu_wkup_lat;
	} else {
		speed = 100;	/* Default speed */
		dev->set_mpu_wkup_lat = NULL;
	}
	
	dev->speed = speed;
	dev->idle = 1;
	dev->dev = &pdev->dev;	// i.e. platform data
	dev->irq = irq->start;
	dev->base = ioremap(mem->start, resource_size(mem));
	if (!dev->base) {
		r = -ENOMEM;
		goto err_free_mem;
	}
	
	platform_set_drvdata(pdev, dev);	// set device pointer

	printk("dev=%p / %p\n", dev, platform_get_drvdata((struct platform_device *) pdev));	// convert into device data pointer
	printk("dev->base=%p\n", dev->base);
	printk("pdata=%p pdev->dev.platform_data=%p\n", pdata, pdev->dev.platform_data);
	printk("pdata->base=%p\n", dev->base);

	if ((r = omap_i2c_get_clocks(dev)) != 0) {
		dev_err(dev->dev, "failure getting clocks\n");
		goto err_iounmap;
	}
	
	omap_i2c_unidle(dev);
	
	dev->rev = omap_i2c_read_reg(dev, OMAP_I2C_REV_REG) & 0xff;
	
	if (cpu_is_omap2430() || cpu_is_omap34xx()) {
		u16 s;
		
		/* Set up the fifo size - Get total size */
		s = (omap_i2c_read_reg(dev, OMAP_I2C_BUFSTAT_REG) >> 14) & 0x3;
		dev->fifo_size = 0x8 << s;
		
		/*
		 * Set up notification threshold as half the total available
		 * size. This is to ensure that we can handle the status on int
		 * call back latencies.
		 */
		dev->fifo_size = (dev->fifo_size / 2);
		dev->b_hw = 1; /* Enable hardware fixes */
		
		/* calculate wakeup latency constraint for MPU */
		if (dev->set_mpu_wkup_lat != NULL)
			dev->latency = (1000000 * dev->fifo_size) /
			(1000 * speed / 8);
	}
	
	/* reset ASAP, clearing any IRQs */
	omap_i2c_init(dev);

#ifdef OLD	
	isr = (dev->rev < OMAP_I2C_REV_2) ? omap_i2c_rev1_isr : omap_i2c_isr;
	r = request_irq(dev->irq, isr, 0, pdev->name, dev);
	
	if (r) {
		dev_err(dev->dev, "failure requesting irq %i\n", dev->irq);
		goto err_unuse_clocks;
	}
	dev_info(dev->dev, "bus %d rev%d.%d at %d kHz\n",
			 pdev->id, dev->rev >> 4, dev->rev & 0xf, dev->speed);
	
	omap_i2c_idle(dev);
	
	adap = &dev->adapter;
	i2c_set_adapdata(adap, dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	strlcpy(adap->name, "OMAP I2C adapter", sizeof(adap->name));
	adap->algo = &omap_i2c_algo;
	adap->dev.parent = &pdev->dev;
	
	/* i2c device drivers may be active on return from add_adapter() */
	adap->nr = pdev->id;
	r = i2c_add_numbered_adapter(adap);
	if (r) {
		dev_err(dev->dev, "failure adding adapter\n");
		goto err_free_irq;
	}
#endif
	return 0;

#ifdef OLD
err_free_irq:
	free_irq(dev->irq, dev);
err_unuse_clocks:
	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, 0);
	omap_i2c_idle(dev);
	omap_i2c_put_clocks(dev);
#endif
err_iounmap:
	iounmap(dev->base);
err_free_mem:
	platform_set_drvdata(pdev, NULL);
	kfree(dev);
err_release_region:
	release_mem_region(mem->start, resource_size(mem));
	printk("I2C bitbang test mode driver returns error %d\n", r);

	return r;
}

static void i2c_gpio_setsda_val(void *data, int state)
{
	struct omap_i2c_dev *dev = data;	// convert into i2c device data pointer
#if 1
	u16 val = omap_i2c_read_reg(dev, OMAP_I2C_SYSTEST_REG);
	if(state)
		val |= OMAP_I2C_SYSTEST_SDA_O;
	else
		val &= ~OMAP_I2C_SYSTEST_SDA_O;
	omap_i2c_write_reg(dev, OMAP_I2C_SYSTEST_REG, val);
#if 0
	{
	static int first=0;
	if(first < 10)
		{
			if(first == 0)
				printk(first="i2c_gpio_setsda_val data=%p dev=%p dev->base=%p\n", data, dev, dev->base);
			printk(first="sda:=%d val=%04x\n", state, val);
			first++;
		}
	}
#endif
#endif
}

static void i2c_gpio_setscl_val(void *data, int state)
{
	struct omap_i2c_dev *dev = data;	// convert into i2c device data pointer
#if 1
	u16 val = omap_i2c_read_reg(dev, OMAP_I2C_SYSTEST_REG);
	if(state)
		val |= OMAP_I2C_SYSTEST_SCL_O;
	else
		val &= ~OMAP_I2C_SYSTEST_SCL_O;
	omap_i2c_write_reg(dev, OMAP_I2C_SYSTEST_REG, val);
#if 0
	{
	static int first=0;
	if(first < 10)
		{
		printk(first="scl:=%d val=%04x\n", state, val);
		first++;
		}
	}
#endif
#endif
}

static int i2c_gpio_getsda(void *data)
{
	struct omap_i2c_dev *dev = data;	// convert into i2c device data pointer
#if 1
	return (omap_i2c_read_reg(dev, OMAP_I2C_SYSTEST_REG) & OMAP_I2C_SYSTEST_SDA_I) != 0;
#else
	return 1;
#endif
}

static int i2c_gpio_getscl(void *data)
{
	struct omap_i2c_dev *dev = data;	// convert into i2c device data pointer
#if 1
	return (omap_i2c_read_reg(dev, OMAP_I2C_SYSTEST_REG) & OMAP_I2C_SYSTEST_SCL_I) != 0;
#else
	return 1;
#endif
}

#endif

// FIXME: platform_device is a omap-i2c!

static int __devinit i2c_gpio_probe(struct platform_device *pdev)
{
	struct omap_i2c_bus_platform_data *pdata;
	struct i2c_algo_bit_data *bit_data;
	struct i2c_adapter *adap;
	struct resource *res;

	int ret;

	pdata = pdev->dev.platform_data;
	printk("pdev=%p\n", pdev);
	printk("pdata=%p\n", pdata);
	if (!pdata)
		return -ENXIO;
	printk("pdev->name=%s\n", pdev->name);
	res = pdev->resource;
	printk("pdev->res[0] start=%p end=%p\n", res[0].start, res[0].end);

	ret = omap_i2c_probe(pdev);	// initialize what we need to acces the I2C controller
	if (ret)
		goto err_alloc_adap;

	ret = -ENOMEM;
	adap = kzalloc(sizeof(struct i2c_adapter), GFP_KERNEL);
	if (!adap)
		goto err_alloc_adap;
	bit_data = kzalloc(sizeof(struct i2c_algo_bit_data), GFP_KERNEL);
	if (!bit_data)
		goto err_alloc_bit_data;

#if !defined(CONFIG_I2C_OMAP_GTA04A2)
	ret = gpio_request(pdata->sda_pin, "sda");
	if (ret)
		goto err_request_sda;
	ret = gpio_request(pdata->scl_pin, "scl");
	if (ret)
		goto err_request_scl;

	if (pdata->sda_is_open_drain) {
		gpio_direction_output(pdata->sda_pin, 1);
		bit_data->setsda = i2c_gpio_setsda_val;
	} else {
		gpio_direction_input(pdata->sda_pin);
		bit_data->setsda = i2c_gpio_setsda_dir;
	}

	if (pdata->scl_is_open_drain || pdata->scl_is_output_only) {
		gpio_direction_output(pdata->scl_pin, 1);
		bit_data->setscl = i2c_gpio_setscl_val;
	} else {
		gpio_direction_input(pdata->scl_pin);
		bit_data->setscl = i2c_gpio_setscl_dir;
	}

	if (!pdata->scl_is_output_only)
		bit_data->getscl = i2c_gpio_getscl;
	bit_data->getsda = i2c_gpio_getsda;
	
#else

#if defined(CONFIG_I2C_OMAP_GTA04A2_SWPAPPED)

	bit_data->setsda = i2c_gpio_setscl_val;
	bit_data->setscl = i2c_gpio_setsda_val;
	bit_data->getsda = i2c_gpio_getscl;
	bit_data->getscl = i2c_gpio_getsda;
	printk("I2C bitbang swaps SCL and SDA on bus %d\n", pdev->id);

#else
	
	bit_data->setsda = i2c_gpio_setsda_val;
	bit_data->setscl = i2c_gpio_setscl_val;
	bit_data->getsda = i2c_gpio_getsda;
	bit_data->getscl = i2c_gpio_getscl;
	
#endif
#endif

	bit_data->udelay = 50;			/* 10 kHz */

	bit_data->timeout = HZ / 10;		/* 100 ms */
	
	bit_data->data = platform_get_drvdata((struct platform_device *) pdev);	// callback data (dev pointer)
	
	printk("drvdata=%p\n", platform_get_drvdata((struct platform_device *) pdev));
	printk("bit_data=%p\n", bit_data);
	printk("adap=%p\n", adap);

	adap->owner = THIS_MODULE;
	snprintf(adap->name, sizeof(adap->name), "i2c-test%d", pdev->id);
	adap->algo_data = bit_data;
	adap->class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	adap->dev.parent = &pdev->dev;

	/*
	 * If "dev->id" is negative we consider it as zero.
	 * The reason to do so is to avoid sysfs names that only make
	 * sense when there are multiple adapters.
	 */
	adap->nr = (pdev->id != -1) ? pdev->id : 0;
	ret = i2c_bit_add_numbered_bus(adap);
	if (ret)
		goto err_add_bus;

//	platform_set_drvdata(pdev, adap);

/*	dev_info(&pdev->dev, "using pins %u (SDA) and %u (SCL%s)\n",
		 pdata->sda_pin, pdata->scl_pin,
		 pdata->scl_is_output_only
		 ? ", no clock stretching" : "");
*/


	printk("I2C bitbang test mode driver nr %d installed\n", adap->nr);

	return 0;

err_add_bus:
//	gpio_free(pdata->scl_pin);
//err_request_scl:
//	gpio_free(pdata->sda_pin);
//err_request_sda:
	kfree(bit_data);
err_alloc_bit_data:
	kfree(adap);
err_alloc_adap:
	printk("I2C bitbang test mode driver returns error %d\n", ret);

	return ret;
}

static int __devexit i2c_gpio_remove(struct platform_device *pdev)
{
	struct i2c_gpio_platform_data *pdata;
	struct i2c_adapter *adap;

	adap = platform_get_drvdata(pdev);
	pdata = pdev->dev.platform_data;

	i2c_del_adapter(adap);
/*	gpio_free(pdata->scl_pin);
	gpio_free(pdata->sda_pin);
 */
	kfree(adap->algo_data);
	kfree(adap);

	return 0;
}

static struct platform_driver i2c_gpio_driver = {
	.driver		= {
		.name	= "i2c-gta04",
		.owner	= THIS_MODULE,
	},
	.probe		= i2c_gpio_probe,
	.remove		= __devexit_p(i2c_gpio_remove),
};

static int __init i2c_gpio_init(void)
{
	int ret;

	ret = platform_driver_register(&i2c_gpio_driver);
	if (ret)
		printk(KERN_ERR "i2c-gta04: i2c_gpio_init failed: %d\n", ret);

	return ret;
}
subsys_initcall(i2c_gpio_init);

static void __exit i2c_gpio_exit(void)
{
	platform_driver_unregister(&i2c_gpio_driver);
}
module_exit(i2c_gpio_exit);

MODULE_AUTHOR("Haavard Skinnemoen <hskinnemoen@atmel.com>, Nikolaus Schaller <hns@goldelico.com>");
MODULE_DESCRIPTION("bitbanging I2C driver using OMAP I2C controller in test mode");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:i2c-gta04");
