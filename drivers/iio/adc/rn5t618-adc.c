// SPDX-License-Identifier: GPL-2.0+
/*
 * ADC driver for the RICOH RN5T618 power management chip family
 *
 * Copyright (C) 2019 Andreas Kemnade
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mfd/rn5t618.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/slab.h>
#include <linux/irqdomain.h>

#define RN5T618_ADC_CONVERSION_TIMEOUT   (msecs_to_jiffies(500))
#define REFERENCE_VOLT 2500

/* mask for selecting channels for single conversion */
#define ADCCNT3_CHANNEL_MASK 0x7
/* average 4-time conversion mode */
#define ADCCNT3_AVG BIT(3)
/* set for starting a single conversion, gets cleared by hw when done */
#define ADCCNT3_GODONE BIT(4)
/* automatic conversion, period is in ADCCNT2, selected channels are
 * in ADCCNT1
 */
#define ADCCNT3_AUTO BIT(5)
#define ADCEND_IRQ BIT(0)

struct rn5t618_adc_data {
	struct device *dev;
	struct rn5t618 *rn5t618;
	struct completion conv_completion;
	int irq;
};

struct rn5t618_channel_ratios {
	u16 numerator;
	u16 denominator;
};

static const struct rn5t618_channel_ratios rn5t618_ratios[8] = {
	{50, 32}, /* LIMMON measured across 20mOhm, amplified by 32 */
	{2, 1}, /* VBAT */
	{3, 1}, /* VADP */
	{3, 1}, /* VUSB */
	{3, 1}, /* VSYS */
	{1, 1}, /* VTHM */
	{1, 1}, /* AIN1 */
	{1, 1}, /* AIN0 */
};

static int rn5t618_read_adc_reg(struct rn5t618 *rn5t618, int reg, u16 *val)
{
	unsigned int h;
	unsigned int l;
	int ret;

	ret = regmap_read(rn5t618->regmap, reg, &h);
	if (ret < 0)
		return ret;

	ret = regmap_read(rn5t618->regmap, reg + 1, &l);
	if (ret < 0)
		return ret;

	h <<= 4;
	h |= (l & 0xF);
	h &= 0xFFF;
	*val = h;

	return 0;
}

static irqreturn_t rn5t618_adc_irq(int irq, void *data)
{
	struct rn5t618_adc_data *adc = data;
	unsigned int r = 0;
	int ret;

	/* clear low & high threshold irqs */
	regmap_write(adc->rn5t618->regmap, RN5T618_IR_ADC1, 0);
	regmap_write(adc->rn5t618->regmap, RN5T618_IR_ADC2, 0);

	ret = regmap_read(adc->rn5t618->regmap, RN5T618_IR_ADC3, &r);
	if (ret < 0)
		dev_err(adc->dev, "failed to read IRQ status: %d\n", ret);

	regmap_write(adc->rn5t618->regmap, RN5T618_IR_ADC3, 0);

	if (r & ADCEND_IRQ)
		complete(&adc->conv_completion);

	return IRQ_HANDLED;
}

static int rn5t618_adc_read(struct iio_dev *iio_dev,
			    const struct iio_chan_spec *chan,
			    int *val, int *val2, long mask)
{
	struct rn5t618_adc_data *adc = iio_priv(iio_dev);
	u16 raw;
	int ret;

	/* select channel */
	ret = regmap_update_bits(adc->rn5t618->regmap, RN5T618_ADCCNT3,
				 ADCCNT3_CHANNEL_MASK,
				 chan->channel);
	if (ret < 0)
		return ret;

	ret = regmap_write(adc->rn5t618->regmap, RN5T618_EN_ADCIR3, ADCEND_IRQ);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(adc->rn5t618->regmap, RN5T618_ADCCNT3,
				 ADCCNT3_AVG,
				 mask == IIO_CHAN_INFO_AVERAGE_RAW ?
				 ADCCNT3_AVG : 0);
	if (ret < 0)
		return ret;

	init_completion(&adc->conv_completion);
	/* single conversion */
	ret = regmap_update_bits(adc->rn5t618->regmap, RN5T618_ADCCNT3,
				 ADCCNT3_GODONE, ADCCNT3_GODONE);
	if (ret < 0)
		return ret;

	ret = wait_for_completion_timeout(&adc->conv_completion,
					  RN5T618_ADC_CONVERSION_TIMEOUT);
	if (ret == 0) {
		dev_warn(adc->dev, "timeout waiting for adc result\n");
		return -ETIMEDOUT;
	}

	ret = rn5t618_read_adc_reg(adc->rn5t618,
				   RN5T618_ILIMDATAH + 2 * chan->channel,
				   &raw);
	if (ret < 0)
		return ret;

	*val = raw;
	if (mask == IIO_CHAN_INFO_PROCESSED)
		*val = *val * REFERENCE_VOLT *
		       rn5t618_ratios[chan->channel].numerator /
		       rn5t618_ratios[chan->channel].denominator / 4095;

	return IIO_VAL_INT;
}

static const struct iio_info rn5t618_adc_iio_info = {
	.read_raw = &rn5t618_adc_read,
};

#define RN5T618_ADC_CHANNEL(_channel, _type, _name) { \
	.type = _type, \
	.channel = _channel, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			      BIT(IIO_CHAN_INFO_AVERAGE_RAW) | \
			      BIT(IIO_CHAN_INFO_PROCESSED), \
	.datasheet_name = _name, \
	.indexed = 1. \
}

static const struct iio_chan_spec rn5t618_adc_iio_channels[] = {
	RN5T618_ADC_CHANNEL(0, IIO_CURRENT, "LIMMON"),
	RN5T618_ADC_CHANNEL(1, IIO_VOLTAGE, "VBAT"),
	RN5T618_ADC_CHANNEL(2, IIO_VOLTAGE, "VADP"),
	RN5T618_ADC_CHANNEL(3, IIO_VOLTAGE, "VUSB"),
	RN5T618_ADC_CHANNEL(4, IIO_VOLTAGE, "VSYS"),
	RN5T618_ADC_CHANNEL(5, IIO_VOLTAGE, "VTHM"),
	RN5T618_ADC_CHANNEL(6, IIO_VOLTAGE, "AIN1"),
	RN5T618_ADC_CHANNEL(7, IIO_VOLTAGE, "AIN0")
};

static int rn5t618_adc_probe(struct platform_device *pdev)
{
	int ret;
	struct iio_dev *iio_dev;
	struct rn5t618_adc_data *adc;
	struct rn5t618 *rn5t618 = dev_get_drvdata(pdev->dev.parent);

	iio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*adc));
	if (!iio_dev) {
		dev_err(&pdev->dev, "failed allocating iio device\n");
		return -ENOMEM;
	}

	adc = iio_priv(iio_dev);
	adc->dev = &pdev->dev;
	adc->rn5t618 = rn5t618;
	adc->irq = -ENOENT;

	if (rn5t618->irq_data)
		adc->irq = regmap_irq_get_virq(rn5t618->irq_data,
					       RN5T618_IRQ_ADC);

	if (adc->irq  < 0) {
		dev_err(&pdev->dev, "get virq failed\n");
		return adc->irq;
	}

	init_completion(&adc->conv_completion);

	iio_dev->name = dev_name(&pdev->dev);
	iio_dev->dev.parent = &pdev->dev;
	iio_dev->info = &rn5t618_adc_iio_info;
	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->channels = rn5t618_adc_iio_channels;
	iio_dev->num_channels = ARRAY_SIZE(rn5t618_adc_iio_channels);

	/* stop any auto-conversion */
	ret = regmap_write(rn5t618->regmap, RN5T618_ADCCNT3, 0);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, iio_dev);

	ret = request_threaded_irq(adc->irq, NULL,
				   rn5t618_adc_irq,
				   IRQF_ONESHOT, dev_name(adc->dev),
				   adc);
	if (ret < 0) {
		dev_err(adc->dev, "request irq %d failed: %d\n", adc->irq, ret);
		return ret;
	}

	ret = iio_device_register(iio_dev);
	if (ret < 0)
		free_irq(adc->irq, adc);

	return ret;
}

static int rn5t618_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *iio_dev = platform_get_drvdata(pdev);
	struct rn5t618_adc_data *adc = iio_priv(iio_dev);

	iio_device_unregister(iio_dev);
	free_irq(adc->irq, adc);

	return 0;
}

static struct platform_driver rn5t618_adc_driver = {
	.driver = {
		.name   = "rn5t618-adc",
	},
	.probe = rn5t618_adc_probe,
	.remove = rn5t618_adc_remove,
};

module_platform_driver(rn5t618_adc_driver);
MODULE_ALIAS("platform:rn5t618-adc");
MODULE_DESCRIPTION("RICOH RN5T618 ADC driver");
MODULE_LICENSE("GPL");

