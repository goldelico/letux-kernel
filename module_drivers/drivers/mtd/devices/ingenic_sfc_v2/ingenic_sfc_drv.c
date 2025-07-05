/*
 * SFC controller for SPI protocol, use FIFO and DMA;
 *
 * Copyright (c) 2015 Ingenic
 * Author: <xiaoyang.fu@ingenic.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include "ingenic_sfc_drv.h"
#include "sfc.h"
#include "sfc_flash.h"
#include "ingenic_sfc_common.h"


#define is_nor_type(a)          ((a[0]==0x55) && (a[1]==0xaa) && (a[2]==0x55) && (a[3]==0xaa))
#define is_nand_type(a)         ((a[0]==0x55) && (a[1]==0xaa) && (a[2]==0x55) && (a[3]==0x00))

unsigned int flash_type = -1;
static int __init flash_type_get(char *str)
{
	if(!strcmp(str,"nand"))
		flash_type = NAND;
	if(!strcmp(str,"nor"))
		flash_type = NOR;
	return 0;
}
early_param("flashtype", flash_type_get);

static const struct of_device_id ingenic_sfc_match[];

static int __init ingenic_sfc_probe(struct platform_device *pdev)
{
	struct sfc_flash *flash;
	int ret = 0;
	const struct of_device_id *of_match;
	struct sfc_data *data = NULL;

	flash = kzalloc(sizeof(struct sfc_flash), GFP_KERNEL);
	if (IS_ERR_OR_NULL(flash))
		return -ENOMEM;

	platform_set_drvdata(pdev, flash);
	flash->dev = &pdev->dev;

	of_match = of_match_node( ingenic_sfc_match, pdev->dev.of_node);
	if(IS_ERR_OR_NULL(of_match))
	{
		kfree(flash);
		return -ENODEV;
	}

	data = (struct sfc_data *)of_match->data;

	ret = of_property_read_u32(pdev->dev.of_node, "ingenic,sfc-init-frequency", (unsigned int *)&flash->sfc_init_frequency);
	if (ret < 0) {
		dev_err(flash->dev, "Cannot get sfc init frequency\n");
	}

	ret = of_property_read_u32(pdev->dev.of_node, "ingenic,sfc-max-frequency", (unsigned int *)&flash->sfc_max_frequency);
	if (ret < 0) {
		dev_err(flash->dev, "Cannot get sfc max frequency\n");
		kfree(flash);
		return -ENOENT;
	}

	flash->sfc = sfc_res_init(pdev);
	if(IS_ERR(flash->sfc)) {
		dev_err(flash->dev, "sfc control init error!\n");
		kfree(flash);
		return PTR_ERR(flash->sfc);
	}

	flash->pdata_params = pdev->dev.platform_data;

	mutex_init(&flash->lock);

	if(flash_type == -1){						/*flash type is not declared in bootargs */
		flash_type = data->flash_type_auto_detect(pdev);
	}

	switch(flash_type)
	{
		case NAND:
			ret = ingenic_sfc_nand_probe(flash);
			break;
		case NOR:
			ret = ingenic_sfc_nor_probe(flash);
			break;
		default:
			dev_err(&pdev->dev, "unknown flash type");
			ret = -EINVAL;
	}
	if(ret){
		kfree(flash);
		return ret;
	}

	return 0;
}

static int __exit ingenic_sfc_remove(struct platform_device *pdev)
{
	struct sfc_flash *flash = platform_get_drvdata(pdev);
	struct sfc *sfc = flash->sfc;

	clk_disable_unprepare(sfc->clk_gate);
	clk_put(sfc->clk_gate);
	clk_disable_unprepare(sfc->clk);
	clk_put(sfc->clk);
	free_irq(sfc->irq, flash);
	iounmap(sfc->iomem);
	release_mem_region(sfc->ioarea->start, resource_size(sfc->ioarea));
	platform_set_drvdata(pdev, NULL);
	free_sfc_desc(sfc);

	if(flash_type == NAND){
		dma_free_coherent(flash->dev, flash->mtd.writesize, flash->sfc->tmp_buffer, flash->sfc->tbuff_pyaddr);
#ifdef CONFIG_INGENIC_SFCNAND_FMW
		sysfs_remove_group(&pdev->dev.kobj, flash->attr_group);
#endif
	} else if(flash_type == NOR)
		sysfs_remove_group(&pdev->dev.kobj, flash->attr_group);
	else{
		dev_err(&pdev->dev, "unknown flash type!\n");
		return -EINVAL;
	}
	return 0;
}


static int ingenic_sfc_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct sfc_flash *flash = platform_get_drvdata(pdev);
	struct sfc *sfc = flash->sfc;

	/* 1.Memory power OFF */
	/*(*(volatile unsigned int *)0xb00000f8) |= (1 << 26);*/

	/* 2.Irq OFF */
	disable_irq(sfc->irq);

	/* 3.Clk OFF */
	clk_disable_unprepare(sfc->clk_gate);
	clk_disable_unprepare(sfc->clk);

	return 0;
}

static int ingenic_sfc_resume(struct platform_device *pdev)
{
	struct sfc_flash *flash = platform_get_drvdata(pdev);
	struct sfc *sfc = flash->sfc;

	/* 1.Clk ON */
	clk_prepare_enable(sfc->clk);
	clk_prepare_enable(sfc->clk_gate);

	/* 2.Irq ON */
	enable_irq(sfc->irq);

	/* 3.Memory power ON */
	/*(*(volatile unsigned int *)0xb00000f8) &= ~(1 << 26);*/

	flash->create_cdt_table(flash->sfc, flash->flash_info, DEFAULT_CDT | UPDATE_CDT);

	return 0;
}

void ingenic_sfc_shutdown(struct platform_device *pdev)
{
	struct sfc_flash *flash = platform_get_drvdata(pdev);
	struct sfc *sfc = flash->sfc;

	disable_irq(sfc->irq);
	clk_disable_unprepare(sfc->clk_gate);
	clk_disable_unprepare(sfc->clk);
	return ;
}

int flash_type_auto_detect_from_tcsm(struct platform_device *pdev)
{
	unsigned char *t;
	int ret;

	t = (volatile unsigned char *)(0xb2401005);

	if (is_nand_type(t))
		ret = NAND;
	else if (is_nor_type(t))
		ret = NOR;
	else
		ret = -EINVAL;

	return ret;
}

int flash_type_auto_detect_from_ddr(struct platform_device *pdev)
{
	unsigned char *t;
	int ret;

	t = (volatile unsigned char *)(0x80001005);

	if (is_nand_type(t))
		ret = NAND;
	else if (is_nor_type(t))
		ret = NOR;
	else
		ret = -EINVAL;

	return ret;
}

struct sfc_data x2000_sfc_priv = {
	.flash_type_auto_detect = flash_type_auto_detect_from_tcsm,
};

struct sfc_data m300_sfc_priv = {
	.flash_type_auto_detect = flash_type_auto_detect_from_tcsm,
};

struct sfc_data x2500_sfc_priv = {
	.flash_type_auto_detect = flash_type_auto_detect_from_ddr,
};

struct sfc_data x1600_sfc_priv = {
	.flash_type_auto_detect = flash_type_auto_detect_from_ddr,
};

static const struct of_device_id ingenic_sfc_match[] = {
	{ .compatible = "ingenic,x2000-sfc",
	  .data = &x2000_sfc_priv, },
	{ .compatible = "ingenic,m300-sfc",
	  .data = &m300_sfc_priv, },
	{ .compatible = "ingenic,x2500-sfc",
	  .data = &x2500_sfc_priv, },
	{ .compatible = "ingenic,x1600-sfc",
	  .data = &x1600_sfc_priv, },
	{},
};
MODULE_DEVICE_TABLE(of, ingenic_sfc_match);

static struct platform_driver ingenic_sfc_drv = {
	.driver		= {
		.name	= "ingenic-sfc",
		.owner	= THIS_MODULE,
		.of_match_table = ingenic_sfc_match,
	},
	.remove		= __exit_p(ingenic_sfc_remove),
	.suspend	= ingenic_sfc_suspend,
	.resume		= ingenic_sfc_resume,
	.shutdown	= ingenic_sfc_shutdown,
};
module_platform_driver_probe(ingenic_sfc_drv, ingenic_sfc_probe);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("INGENIC SFC Driver");
