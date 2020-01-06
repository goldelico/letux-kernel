// SPDX-License-Identifier: GPL-2.0+
/*
 * ADC driver for the RICOH RN5T618 power management chip family
 *
 * Copyright (C) 2019 Andreas Kemnade
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mfd/rn5t618.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/slab.h>
#include <linux/irqdomain.h>

#define REFERENCE_VOLT 2500

struct rn5t618_adc_data {
	struct device *dev;
	struct rn5t618 *rn5t618;
};

struct rn5t618_channel_ratios {
	u16 numerator;
	u16 denominator;
};

static const struct rn5t618_channel_ratios rn5t618_ratios[8] =
{
	{1, 1}, /* LIMMON, tbd */
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

static int rn5t618_adc_read(struct iio_dev *iio_dev,
			    const struct iio_chan_spec *chan,
			    int *val, int *val2, long mask)
{
	struct rn5t618_adc_data *adc = iio_priv(iio_dev);
	u16 raw;
	int ret;

	ret = rn5t618_read_adc_reg(adc->rn5t618, RN5T618_ILIMDATAH + 2 * chan->channel, &raw);
	if (ret < 0)
		return ret;

	*val = raw;
	if (mask == IIO_CHAN_INFO_PROCESSED) {
		*val = *val * REFERENCE_VOLT * rn5t618_ratios[chan->channel].numerator / rn5t618_ratios[chan->channel].denominator / 4095; 
	}

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

	iio_dev->name = dev_name(&pdev->dev);
	iio_dev->dev.parent = &pdev->dev;
	iio_dev->info = &rn5t618_adc_iio_info;
	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->channels = rn5t618_adc_iio_channels;
	iio_dev->num_channels = ARRAY_SIZE(rn5t618_adc_iio_channels);

	/* stop */
	ret = regmap_write(rn5t618->regmap, RN5T618_ADCCNT3, 0);
	if (ret < 0)
		return ret;

	ret = regmap_write(rn5t618->regmap, RN5T618_ADCCNT2, 0);
	if (ret < 0)
		return ret;

	/* select all channels */
	ret = regmap_write(rn5t618->regmap, RN5T618_ADCCNT1, 0xff);
	if (ret < 0)
		return ret;

	/* periodic start */
	ret = regmap_write(rn5t618->regmap, RN5T618_ADCCNT3, 0x28);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, iio_dev);

	ret = iio_device_register(iio_dev);
	return ret;
}

static int rn5t618_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *iio_dev = platform_get_drvdata(pdev);
	struct rn5t618_adc_data *adc = iio_priv(iio_dev);

	iio_device_unregister(iio_dev);

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

