/*
 * Regulators driver for X-Powers AXP
 *
 * Copyright (C) 2013 X-Powers, Ltd.
 *  Zhang Donglu <zhangdonglu@x-powers.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/module.h>
#include <linux/version.h>


#include "axp-cfg.h"
#include "axp-regu.h"


static inline struct device *to_axp_dev(struct regulator_dev *rdev)
{
	return rdev_get_dev(rdev)->parent;
}

static inline int check_range(struct axp_regulator_info *info,
				int min_uV, int max_uV)
{
	if (min_uV < info->min_uV || min_uV > info->max_uV)
		return -EINVAL;

	return 0;
}


/* AXP common operations */
static int axp_set_voltage(struct regulator_dev *rdev,
				  int min_uV, int max_uV,unsigned *selector)
{
	struct axp_regulator_info *info = rdev_get_drvdata(rdev);
	struct device *axp_dev = to_axp_dev(rdev);
	uint8_t val, mask;

	if (check_range(info, min_uV, max_uV)) {
		pr_err("invalid voltage range (%d, %d) uV\n", min_uV, max_uV);
		return -EINVAL;
	}
	if ((info->switch_uV != 0) && (info->step2_uV!= 0) &&
	(info->new_level_uV != 0) && (min_uV > info->switch_uV)) {
		val = (info->switch_uV- info->min_uV + info->step1_uV - 1) / info->step1_uV;
		if (min_uV <= info->new_level_uV){
			val += 1;
		} else {
			val += (min_uV - info->new_level_uV) / info->step2_uV;
			val += 1;
		}
		mask = ((1 << info->vol_nbits) - 1)  << info->vol_shift;
	} else if ((info->switch_uV != 0) && (info->step2_uV!= 0) &&
	(min_uV > info->switch_uV) && (info->new_level_uV == 0)) {
		val = (info->switch_uV- info->min_uV + info->step1_uV - 1) / info->step1_uV;
		val += (min_uV - info->switch_uV) / info->step2_uV;
		mask = ((1 << info->vol_nbits) - 1)  << info->vol_shift;
	} else {
		val = (min_uV - info->min_uV + info->step1_uV - 1) / info->step1_uV;
		val <<= info->vol_shift;
		mask = ((1 << info->vol_nbits) - 1)  << info->vol_shift;
	}
	//if(info->vol_reg==0x25 || info->vol_reg==0x10){
		dev_dbg(axp_dev, "[AXP216-reg] axp_regulator_set_voltage reg=0x%x,val=0x%x!\n",info->vol_reg,val);
	//}
	return axp_update(axp_dev, info->vol_reg, val, mask);
}

static int axp_get_voltage(struct regulator_dev *rdev)
{
	struct axp_regulator_info *info = rdev_get_drvdata(rdev);
	struct device *axp_dev = to_axp_dev(rdev);
	uint8_t val, mask;
	int ret, switch_val, vol;

	ret = axp_read(axp_dev, rdev->desc->vsel_reg, &val);
	if (ret)
		return ret;

	mask = ((1 << info->vol_nbits) - 1)  << info->vol_shift;
	switch_val = ((info->switch_uV- info->min_uV + info->step1_uV - 1) / info->step1_uV);
	val = (val & mask) >> info->vol_shift;

	if ((info->switch_uV != 0) && (info->step2_uV!= 0) &&
	(val > switch_val) && (info->new_level_uV != 0)) {
		val -= switch_val;
		vol = info->new_level_uV + info->step2_uV * val;
	} else if ((info->switch_uV != 0) && (info->step2_uV!= 0) &&
	(val > switch_val) && (info->new_level_uV == 0)) {
		val -= switch_val;
		vol = info->switch_uV + info->step2_uV * val;
	} else {
		vol = info->min_uV + info->step1_uV * val;
	}

	return vol;

}

static int axp_enable(struct regulator_dev *rdev)
{
	struct axp_regulator_info *info = rdev_get_drvdata(rdev);
	struct device *axp_dev = to_axp_dev(rdev);
	return axp_set_bits(axp_dev, info->enable_reg,
					1 << info->enable_bit);
}

static int axp_disable(struct regulator_dev *rdev)
{
	struct axp_regulator_info *info = rdev_get_drvdata(rdev);
	struct device *axp_dev = to_axp_dev(rdev);
	return axp_clr_bits(axp_dev, info->enable_reg,
					1 << info->enable_bit);
}

static int axp_is_enabled(struct regulator_dev *rdev)
{
	struct axp_regulator_info *info = rdev_get_drvdata(rdev);
	struct device *axp_dev = to_axp_dev(rdev);
	uint8_t reg_val;
	int ret;
	ret = axp_read(axp_dev, info->enable_reg, &reg_val);
	if (ret)
		return ret;

	return !!(reg_val & (1 << info->enable_bit));
}

static int axp_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	struct axp_regulator_info *info = rdev_get_drvdata(rdev);
	int ret;

	ret = info->min_uV + info->step1_uV * selector;
	if ((info->switch_uV != 0) && (info->step2_uV != 0) &&
	(ret > info->switch_uV) && (info->new_level_uV != 0)) {
		selector -= ((info->switch_uV-info->min_uV)/info->step1_uV);
		ret = info->new_level_uV + info->step2_uV * selector;
	} else if ((info->switch_uV != 0) && (info->step2_uV != 0) &&
	(ret > info->switch_uV) && (info->new_level_uV == 0)) {
		selector -= ((info->switch_uV-info->min_uV)/info->step1_uV);
		ret = info->switch_uV + info->step2_uV * selector;
	}
	if (ret > info->max_uV)
		return -EINVAL;
	return ret;
}

static int axp_set_suspend_voltage(struct regulator_dev *rdev, int uV)
{
	return axp_set_voltage(rdev, uV, uV,NULL);
}


static struct regulator_ops axp_ops = {
	.set_voltage	= axp_set_voltage,
	.get_voltage	= axp_get_voltage,
	.list_voltage	= axp_list_voltage,
	.enable		= axp_enable,
	.disable	= axp_disable,
	.is_enabled	= axp_is_enabled,
//	.set_suspend_enable		= axp_enable,
//	.set_suspend_disable	= axp_disable,
	.set_suspend_voltage	= axp_set_suspend_voltage,
};


static int axp_ldoio01_enable(struct regulator_dev *rdev)
{
	struct axp_regulator_info *info = rdev_get_drvdata(rdev);
	struct device *axp_dev = to_axp_dev(rdev);

	 axp_set_bits(axp_dev, info->enable_reg,0x03);
	 return axp_clr_bits(axp_dev, info->enable_reg,0x04);
}

static int axp_ldoio01_disable(struct regulator_dev *rdev)
{
	struct axp_regulator_info *info = rdev_get_drvdata(rdev);
	struct device *axp_dev = to_axp_dev(rdev);

	return axp_clr_bits(axp_dev, info->enable_reg,0x07);
}

static int axp_ldoio01_is_enabled(struct regulator_dev *rdev)
{
	struct axp_regulator_info *info = rdev_get_drvdata(rdev);
	struct device *axp_dev = to_axp_dev(rdev);
	uint8_t reg_val;
	int ret;

	ret = axp_read(axp_dev, info->enable_reg, &reg_val);
	if (ret)
		return ret;

	return (((reg_val &= 0x07)== 0x03)?1:0);
}

static struct regulator_ops axp_ldoio01_ops = {
	.set_voltage	= axp_set_voltage,
	.get_voltage	= axp_get_voltage,
	.list_voltage	= axp_list_voltage,
	.enable		= axp_ldoio01_enable,
	.disable	= axp_ldoio01_disable,
	.is_enabled	= axp_ldoio01_is_enabled,
	.set_suspend_enable		= axp_ldoio01_enable,
	.set_suspend_disable	= axp_ldoio01_disable,
	.set_suspend_voltage	= axp_set_suspend_voltage,
};


#define AXP216_LDO(_id, min, max, step1, vreg, shift, nbits, ereg, ebit, switch_vol, step2, new_level)	\
	AXP_LDO(AXP, _id, min, max, step1, vreg, shift, nbits, ereg, ebit, switch_vol, step2, new_level)

#define AXP216_DCDC(_id, min, max, step1, vreg, shift, nbits, ereg, ebit, switch_vol, step2, new_level)	\
	AXP_DCDC(AXP, _id, min, max, step1, vreg, shift, nbits, ereg, ebit, switch_vol, step2, new_level)



static struct axp_regulator_info axp_regulator_info[12] = {
	AXP216_LDO(RTC,	3000,	3000,	0,	LDO1,	0,	0,	LDO1EN,	0, 0, 0, 0),//ldo1 for rtc
	AXP216_LDO(ALDO1,	700,	3300,	100,	LDO2,	0,	5,	LDO2EN,	6, 0, 0, 0),//ldo2 for aldo1
	AXP216_LDO(ALDO2,	700,	3300,	100,	LDO3,	0,	5,	LDO3EN,	7, 0, 0, 0),//ldo3 for aldo2
	AXP216_LDO(ALDO3,	700,	3300,	100,	LDO4,	0,	5,	LDO4EN,	5, 0, 0, 0),//ldo3 for aldo3

	AXP216_LDO(ELDO1,	700,	3300,	100,	LDO9,	0,	5,	LDO9EN,	0, 0, 0, 0),//ldo9 for eldo1
	AXP216_LDO(ELDO2,	700,	3300,	100,	LDO10,	0,	5,	LDO10EN,1, 0, 0, 0),//ldo10 for eldo2

	AXP216_DCDC(1,	1600,	3400,	100,	DCDC1,	0,	5,	DCDC1EN,1, 0, 0, 0),//buck1 for io
	AXP216_DCDC(2,	600,	1540,	20,	DCDC2,	0,	6,	DCDC2EN,2, 0, 0, 0),//buck2 for cpu
	AXP216_DCDC(3,	600,	1860,	20,	DCDC3,	0,	6,	DCDC3EN,3, 0, 0, 0),//buck3 for gpu
	AXP216_DCDC(4,	600,	2600,	20,	DCDC4,	0,	6,	DCDC4EN,4, 1540, 100, 1800),//buck4 for core
	AXP216_DCDC(5,	1000,	2550,	50,	DCDC5,	0,	5,	DCDC5EN,5, 0, 0, 0),//buck5 for ddr

	AXP216_LDO(LDOIO1,	700,	3300,	100,	LDOIO1,	0,	5,	LDOIO1EN,0, 0, 0, 0),//ldoio1
};

#define REGULATOR_NODE_NAME	"regulators"

static ssize_t workmode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct regulator_dev *rdev = dev_get_drvdata(dev);
	struct axp_regulator_info *info = rdev_get_drvdata(rdev);
	struct device *axp_dev = to_axp_dev(rdev);
	int ret;
	uint8_t val;
	ret = axp_read(axp_dev, AXP_BUCKMODE, &val);
	if (ret)
		return sprintf(buf, "IO ERROR\n");

	if(info->desc.id == AXP_ID_DCDC1){
		switch (val & 0x04) {
			case 0:return sprintf(buf, "AUTO\n");
			case 4:return sprintf(buf, "PWM\n");
			default:return sprintf(buf, "UNKNOWN\n");
		}
	}
	else if(info->desc.id == AXP_ID_DCDC2){
		switch (val & 0x02) {
			case 0:return sprintf(buf, "AUTO\n");
			case 2:return sprintf(buf, "PWM\n");
			default:return sprintf(buf, "UNKNOWN\n");
		}
	}
	else if(info->desc.id == AXP_ID_DCDC3){
		switch (val & 0x02) {
			case 0:return sprintf(buf, "AUTO\n");
			case 2:return sprintf(buf, "PWM\n");
			default:return sprintf(buf, "UNKNOWN\n");
		}
	}
	else if(info->desc.id == AXP_ID_DCDC4){
		switch (val & 0x02) {
			case 0:return sprintf(buf, "AUTO\n");
			case 2:return sprintf(buf, "PWM\n");
			default:return sprintf(buf, "UNKNOWN\n");
		}
	}
	else if(info->desc.id == AXP_ID_DCDC5){
		switch (val & 0x02) {
			case 0:return sprintf(buf, "AUTO\n");
			case 2:return sprintf(buf, "PWM\n");
			default:return sprintf(buf, "UNKNOWN\n");
		}
	}
	else
		return sprintf(buf, "IO ID ERROR\n");
}

static ssize_t workmode_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct regulator_dev *rdev = dev_get_drvdata(dev);
	struct axp_regulator_info *info = rdev_get_drvdata(rdev);
	struct device *axp_dev = to_axp_dev(rdev);
	char mode;
	uint8_t val;
	if(  buf[0] > '0' && buf[0] < '9' )// 1/AUTO: auto mode; 2/PWM: pwm mode;
		mode = buf[0];
	else
		mode = buf[1];

	switch(mode){
	 case 'U':
	 case 'u':
	 case '1':
		val = 0;break;
	 case 'W':
	 case 'w':
	 case '2':
	 	val = 1;break;
	 default:
	    val =0;
	}

	if(info->desc.id == AXP_ID_DCDC1){
		if(val)
			axp_set_bits(axp_dev, AXP_BUCKMODE,0x01);
		else
			axp_clr_bits(axp_dev, AXP_BUCKMODE,0x01);
	}
	else if(info->desc.id == AXP_ID_DCDC2){
		if(val)
			axp_set_bits(axp_dev, AXP_BUCKMODE,0x02);
		else
			axp_clr_bits(axp_dev, AXP_BUCKMODE,0x02);
	}
	else if(info->desc.id == AXP_ID_DCDC3){
		if(val)
			axp_set_bits(axp_dev, AXP_BUCKMODE,0x04);
		else
			axp_clr_bits(axp_dev, AXP_BUCKMODE,0x04);
	}
	else if(info->desc.id == AXP_ID_DCDC4){
		if(val)
			axp_set_bits(axp_dev, AXP_BUCKMODE,0x08);
		else
			axp_clr_bits(axp_dev, AXP_BUCKMODE,0x08);
	}
	else if(info->desc.id == AXP_ID_DCDC5){
		if(val)
			axp_set_bits(axp_dev, AXP_BUCKMODE,0x10);
		else
			axp_clr_bits(axp_dev, AXP_BUCKMODE,0x10);
	}
	return count;
}

static ssize_t frequency_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct regulator_dev *rdev = dev_get_drvdata(dev);
	struct device *axp_dev = to_axp_dev(rdev);
	int ret;
	uint8_t val;
	ret = axp_read(axp_dev, AXP_BUCKFREQ, &val);
	if (ret)
		return ret;
	ret = val & 0x0F;
	return sprintf(buf, "%d\n",(ret*75 + 750));
}

static ssize_t frequency_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct regulator_dev *rdev = dev_get_drvdata(dev);
	struct device *axp_dev = to_axp_dev(rdev);
	uint8_t val,tmp;
	int var;
	var = simple_strtoul(buf, NULL, 10);
	if(var < 750)
		var = 750;
	if(var > 1875)
		var = 1875;

	val = (var -750)/75;
	val &= 0x0F;

	axp_read(axp_dev, AXP_BUCKFREQ, &tmp);
	tmp &= 0xF0;
	val |= tmp;
	axp_write(axp_dev, AXP_BUCKFREQ, val);
	return count;
}


static struct device_attribute axp_regu_attrs[] = {
	AXP_REGU_ATTR(workmode),
	AXP_REGU_ATTR(frequency),
};

int axp_regu_create_attrs(struct platform_device *pdev)
{
	int j,ret;
	for (j = 0; j < ARRAY_SIZE(axp_regu_attrs); j++) {
		ret = device_create_file(&pdev->dev,&axp_regu_attrs[j]);
		if (ret)
			goto sysfs_failed;
	}
    goto succeed;

sysfs_failed:
	while (j--)
		device_remove_file(&pdev->dev,&axp_regu_attrs[j]);
succeed:
	return ret;
}

static int  axp_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;
# if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	struct regulator_config config = { };
#endif
	struct  axp_reg_init * platform_data = (struct  axp_reg_init *)(pdev->dev.platform_data);
	//struct axp_regulator_info *info = platform_data->info;
	struct axp_regulator_info *ri = NULL;
	int i;

	dev_dbg(&pdev->dev, "pdev->name = %s\n",pdev->name);
	dev_dbg(&pdev->dev, "pdev->dev.parent->driver->name %s\n",pdev->dev.parent->driver->name);
	for (i = 0; i < 12; i++) {

		ri = &axp_regulator_info[i];
		if (ri->desc.id == AXP_ID_RTC || ri->desc.id == AXP_ID_ALDO1 \
				|| ri->desc.id == AXP_ID_ALDO2 || ri->desc.id == AXP_ID_ALDO3 \
				|| ri->desc.id == AXP_ID_ELDO1 || ri->desc.id == AXP_ID_ELDO2 \
				|| ri->desc.id == AXP_ID_DCDC1 || ri->desc.id == AXP_ID_DCDC2 \
				|| ri->desc.id == AXP_ID_DCDC3 || ri->desc.id == AXP_ID_DCDC4 \
				|| ri->desc.id == AXP_ID_DCDC5 )
			ri->desc.ops = &axp_ops;
		if (ri->desc.id == AXP_ID_LDOIO1 )
			ri->desc.ops = &axp_ldoio01_ops;

		ri->desc.of_match	= of_match_ptr(ri->desc.name);
		ri->desc.regulators_node = of_match_ptr(REGULATOR_NODE_NAME);

		config.dev = pdev->dev.parent;
		config.init_data = &(platform_data->axp_reg_init_data);
		config.driver_data = ri;

		rdev = devm_regulator_register(&pdev->dev, &ri->desc, &config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev, "failed to register %s regulator (errno %ld\n)\n",
					ri->desc.name, PTR_ERR(rdev));
			return PTR_ERR(rdev);
		}
	}
	return 0;
}

static int  axp_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);
	return 0;
}

static struct platform_driver axp_regulator_driver = {
	.driver	= {
		.name	= "axp216-regulator",
		.owner	= THIS_MODULE,
	},
	.probe		= axp_regulator_probe,
	.remove		= axp_regulator_remove,
};

static int __init axp_regulator_init(void)
{
	return platform_driver_register(&axp_regulator_driver);
}
subsys_initcall(axp_regulator_init);

static void __exit axp_regulator_exit(void)
{
	platform_driver_unregister(&axp_regulator_driver);
}
module_exit(axp_regulator_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Criss");
MODULE_DESCRIPTION("Regulator Driver for X-powers AXP216 PMIC");
MODULE_ALIAS("platform:axp-regulator");
