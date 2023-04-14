/*
 * Base driver for X-Powers AXP
 *
 * Copyright (C) 2013 X-Powers, Ltd.
 *  Zhang Donglu <zhangdonglu@x-powers.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/version.h>
//#include <mach/system.h>

//#include "axp-cfg.h"
#include "axp216-mfd.h"

#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/mfd/core.h>
#include <linux/platform_device.h>
static const struct mfd_cell axp216_cells[] = {
	{ .name = "axp216-regulator" },
	{ .name = "axp216-supplyer" },
};

static void axp_mfd_irq_work(struct work_struct *work)
{
	struct axp_mfd_chip *chip = container_of(work, struct axp_mfd_chip, irq_work);
	uint64_t irqs = 0;
	while (1) {
		if (chip->ops->read_irqs(chip, &irqs)){
			printk("read irq fail\n");
			break;
		}
		irqs &= chip->irqs_enabled;
		if (irqs == 0){
			break;
		}

		if(irqs > 0xffffffff){
			blocking_notifier_call_chain(&chip->notifier_list, (uint32_t)(irqs>>32), (void *)1);
		}
		else{
			blocking_notifier_call_chain(&chip->notifier_list, (uint32_t)irqs, (void *)0);
		}
	}
	enable_irq(chip->client->irq);
}

static irqreturn_t axp_mfd_irq_handler(int irq, void *data)
{
	struct axp_mfd_chip *chip = data;
	disable_irq_nosync(irq);
	(void)schedule_work(&chip->irq_work);

	return IRQ_HANDLED;
}

static struct axp_mfd_chip_ops axp_mfd_ops[] = {
	[0] = {
		.init_chip    = axp_init_chip,
		.enable_irqs  = axp_enable_irqs,
		.disable_irqs = axp_disable_irqs,
		.read_irqs    = axp_read_irqs,
	},
};

int axp_mfd_create_attrs(struct axp_mfd_chip *chip)
{
	int j,ret;
	if (chip->type ==  AXP216){
		for (j = 0; j < ARRAY_SIZE(axp_mfd_attrs); j++) {
			ret = device_create_file(chip->dev,&axp_mfd_attrs[j]);
			if (ret)
			goto sysfs_failed;
		}
	}
	else
		ret = 0;
	goto succeed;

sysfs_failed:
	while (j--)
		device_remove_file(chip->dev,&axp_mfd_attrs[j]);
succeed:
	return ret;
}

static int __remove_subdev(struct device *dev, void *unused)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}

static int axp_mfd_remove_subdevs(struct axp_mfd_chip *chip)
{
	return device_for_each_child(chip->dev, NULL, __remove_subdev);
}

void axp_power_off(void)
{
	uint8_t val;

    axp_clr_bits(&axp->dev, AXP_GPIO1_CTL, 0x07);//out low
#if defined (CONFIG_KP_AXP)
	if(SHUTDOWNVOL >= 2600 && SHUTDOWNVOL <= 3300){
		if (SHUTDOWNVOL > 3200){
			val = 0x7;
		}
		else if (SHUTDOWNVOL > 3100){
			val = 0x6;
		}
		else if (SHUTDOWNVOL > 3000){
			val = 0x5;
		}
		else if (SHUTDOWNVOL > 2900){
			val = 0x4;
		}
		else if (SHUTDOWNVOL > 2800){
			val = 0x3;
		}
		else if (SHUTDOWNVOL > 2700){
			val = 0x2;
		}
		else if (SHUTDOWNVOL > 2600){
			val = 0x1;
		}
		else
			val = 0x0;

		axp_update(&axp->dev, AXP_VOFF_SET, val, 0x7);
	}
	val = 0xff;
    mdelay(20);
    if(POWER_START != 1){
		axp_read(&axp->dev, AXP_STATUS, &val);
		if(val & 0xF0){
	    	axp_read(&axp->dev, AXP_MODE_CHGSTATUS, &val);
	    	if(val & 0x20){
	        	axp_write(&axp->dev, AXP_BUFFERC, 0x0f);
            	mdelay(20);
		//# if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
		  //  	arch_reset(0,NULL);
		//#endif
		dev_err(&axp->dev,"[axp] warning!!! arch can't ,reboot, maybe some error happend!\n");
	    	}
		}
	}
    axp_write(&axp->dev, AXP_BUFFERC, 0x00);
    mdelay(20);
#if RESET_WHEN_ACIN
	axp_read(&axp->dev, AXP_STATUS, &val);
	if(val & 0xF0){
		printk("[axp] reset!\n");
		axp_set_bits(&axp->dev, AXP_VOFF_SET, 0x40);
	}else
#endif
    axp_set_bits(&axp->dev, AXP_OFF_CTL, 0x80);
    mdelay(20);
    dev_dbg(&axp->dev, "[axp] warning!!! axp can't power-off, maybe some error happend!\n");

#endif
}
EXPORT_SYMBOL_GPL(axp_power_off);
static int  axp_mfd_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct axp_mfd_chip *chip;
	struct device_node *np = client->dev.of_node;
	//enum of_gpio_flags flags;
	enum of_gpio_flags flags;
	int ret;
	unsigned int axp216_irq;

	chip = kzalloc(sizeof(struct axp_mfd_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	axp = client;

	chip->client = client;
	chip->dev = &client->dev;
	chip->ops = &axp_mfd_ops[id->driver_data];

	mutex_init(&chip->lock);
	INIT_WORK(&chip->irq_work, axp_mfd_irq_work);
	BLOCKING_INIT_NOTIFIER_HEAD(&chip->notifier_list);

	axp216_irq = of_get_named_gpio_flags(np, "ingenic,axp216-gpio", 0, &flags);
	if(!gpio_is_valid(axp216_irq)) {
		dev_err(&client->dev,"couldn't find pmu irq gpio, axp216_irq = %d\n", axp216_irq);
		goto out_free_chip;
	}

	i2c_set_clientdata(client, chip);

	ret = chip->ops->init_chip(chip);
	if (ret)
		goto out_free_chip;

	client->irq=gpio_to_irq(axp216_irq); //chan
	if (client->irq < 0) {
		dev_err(&client->dev,"couldn't find pmu irq\n");
		goto out_free_chip;
	}
	dev_dbg(&client->dev, "[axp-mfd] client->irq=%d!\n",client->irq);

	ret = request_irq(/*gpio_to_irq(4)*/client->irq, axp_mfd_irq_handler,
		flags, "axp_mfd", chip);
  	if (ret) {
  		dev_err(&client->dev, "failed to request irq %d\n",
  				client->irq);
  		goto out_free_chip;
  	}
       //enable_irq(gpio_to_irq(225));
	ret = mfd_add_devices(&client->dev, -1, axp216_cells,
			ARRAY_SIZE(axp216_cells), NULL, 0, NULL);
	if (ret) {
		dev_err(&client->dev, "failed to add sub-devices: %d\n", ret);
		goto out_free_irq;
	}

	/* PM hookup */
	//if(!pm_power_off)
	pm_power_off = axp_power_off;

	ret = axp_mfd_create_attrs(chip);
	if(ret){
		return ret;
	}
	return 0;

out_free_irq:
	free_irq(client->irq, chip);

out_free_chip:
	i2c_set_clientdata(client, NULL);
	kfree(chip);

	return ret;
}

static int  axp_mfd_remove(struct i2c_client *client)
{
	struct axp_mfd_chip *chip = i2c_get_clientdata(client);

	pm_power_off = NULL;
	axp = NULL;

	axp_mfd_remove_subdevs(chip);
	kfree(chip);
	return 0;
}
static const struct of_device_id axp216_match[] = {
	{.compatible = "x-power,axp216",},
	{},
};

MODULE_DEVICE_TABLE(of, axp216_match);
static const struct i2c_device_id axp_mfd_id_table[] = {
	{"axp216", 0},
	{},
};
static struct i2c_driver axp_mfd_driver = {
	.probe		= axp_mfd_probe,
	.remove		= axp_mfd_remove,
	.driver	= {
		.name	= "axp216",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(axp216_match),
	},
	.id_table	= axp_mfd_id_table,
};

static int __init axp_mfd_init(void)
{
	return i2c_add_driver(&axp_mfd_driver);
}
subsys_initcall(axp_mfd_init);

static void __exit axp_mfd_exit(void)
{
	i2c_del_driver(&axp_mfd_driver);
}
module_exit(axp_mfd_exit);

MODULE_DESCRIPTION("MFD Driver for X-Powers AXP PMIC");
MODULE_AUTHOR("Weijin Zhong X-POWERS");
MODULE_LICENSE("GPL");
