/* drivers/mfd/ingenic_adc.c
 *
 * Copyright (C) 2012 Ingenic Semiconductor Co., Ltd.
 *      http://www.ingenic.com
 *      Sun Jiwei<jwsun@ingenic.cn>
 * ingenic4780 SOC ADC device core
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This driver is designed to control the usage of the ADC block between
 * the touchscreen and any other drivers that may need to use it, such as
 * the hwmon driver.
 */

#include <linux/err.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/mfd/core.h>

#include <linux/mfd/ingenic_adc.h>
#include <irq.h>
#include <linux/of_irq.h>

#define INGENIC_REG_ADC_ENABLE       0x00
#define INGENIC_REG_ADC_CTRL         0x08
#define INGENIC_REG_ADC_STATUS       0x0c

#define INGENIC_REG_ADC_AUX_BASE     0x10
#define INGENIC_REG_ADC_CLKDIV	     0x20
/*
 *the following registeres is for touchscreen<junyang@ingenic.cn>
 */
#define CLKDIV		120
#define CLKDIV_US       2
#define CLKDIV_MS       200

#define INGENIC_ADC_IRQ_NUM	8

static const struct of_device_id sadc_match[];

struct ingenic_adc_priv {
	unsigned int aux_channels;
};

struct ingenic_adc {
	struct resource *mem;
	void __iomem *base;

	int irq;
	int irq_base;
	struct irq_domain *irq_domain;
	struct device_node *np;
	struct resource *aux_res_ptr;
	struct mfd_cell *adc_cells;

	struct clk *clk;
	atomic_t clk_ref;

	spinlock_t lock;
	const struct ingenic_adc_priv *priv;/*soc data*/
};

static inline void ingenic_adc_irq_set_masked(struct ingenic_adc *adc, int irq,
		bool masked)
{
	unsigned long flags;
	uint8_t val;

	irq -= adc->irq_base;

	spin_lock_irqsave(&adc->lock, flags);

	val = readb(adc->base + INGENIC_REG_ADC_CTRL);
	if (masked) {
		val |= BIT(irq);
	}
	else {
		val &= ~BIT(irq);
	}
	writeb(val, adc->base + INGENIC_REG_ADC_CTRL);

	spin_unlock_irqrestore(&adc->lock, flags);
}

static void ingenic_adc_irq_mask(struct irq_data *data)
{
	struct ingenic_adc *adc = irq_data_get_irq_chip_data(data);
	ingenic_adc_irq_set_masked(adc, data->irq, true);
}

static void ingenic_adc_irq_unmask(struct irq_data *data)
{
	struct ingenic_adc *adc = irq_data_get_irq_chip_data(data);
	ingenic_adc_irq_set_masked(adc, data->irq, false);
}

static void ingenic_adc_irq_ack(struct irq_data *data)
{
	struct ingenic_adc *adc = irq_data_get_irq_chip_data(data);
	unsigned int irq = data->irq - adc->irq_base;
	writeb(BIT(irq), adc->base + INGENIC_REG_ADC_STATUS);
}

static struct irq_chip ingenic_adc_irq_chip = {
	.name = "ingenic-adc",
	.irq_mask = ingenic_adc_irq_mask,
	.irq_disable = ingenic_adc_irq_mask,
	.irq_unmask = ingenic_adc_irq_unmask,
	.irq_ack = ingenic_adc_irq_ack,
};

static void ingenic_adc_irq_demux(struct irq_desc *desc)
{
	struct ingenic_adc *adc = irq_desc_get_handler_data(desc);
	uint8_t status;
	unsigned int i;

	status = readb(adc->base + INGENIC_REG_ADC_STATUS);

	for (i = 0; i < adc->priv->aux_channels; i++) {
		if (status & BIT(i)) {
			generic_handle_irq(irq_find_mapping(adc->irq_domain, i));
		}
	}
}


static inline void ingenic_adc_enable(struct ingenic_adc *adc)
{
	uint16_t val;

	if (atomic_inc_return(&adc->clk_ref) == 1) {
		val = readw(adc->base + INGENIC_REG_ADC_ENABLE);
		val &= ~BIT(15);
		writew(val, adc->base + INGENIC_REG_ADC_ENABLE);
		msleep(5);
	}
}

static inline void ingenic_adc_disable(struct ingenic_adc *adc)
{
	uint16_t val;

	if (atomic_dec_return(&adc->clk_ref) == 0) {
		val = readw(adc->base + INGENIC_REG_ADC_ENABLE);
		val |= BIT(15);
		writew(val, adc->base + INGENIC_REG_ADC_ENABLE);
	}
}

static inline void ingenic_adc_set_enabled(struct ingenic_adc *adc, int engine,
		bool enabled)
{
	unsigned long flags;
	uint16_t val;

	spin_lock_irqsave(&adc->lock, flags);

	val = readw(adc->base + INGENIC_REG_ADC_ENABLE);
	if (enabled) {
		val |= BIT(engine);
	}
	else {
		val &= ~BIT(engine);
	}
	writew(val, adc->base + INGENIC_REG_ADC_ENABLE);

	spin_unlock_irqrestore(&adc->lock, flags);
}

static int ingenic_adc_cell_enable(struct platform_device *pdev)
{
	struct ingenic_adc *adc = dev_get_drvdata(pdev->dev.parent);

	ingenic_adc_enable(adc);
	ingenic_adc_set_enabled(adc, pdev->id, true);

	return 0;
}

static int ingenic_adc_cell_disable(struct platform_device *pdev)
{
	struct ingenic_adc *adc = dev_get_drvdata(pdev->dev.parent);

	ingenic_adc_set_enabled(adc, pdev->id, false);
	ingenic_adc_disable(adc);

	return 0;
}


int adc_write_reg(struct device *dev ,uint8_t addr_offset,uint32_t mask,uint32_t val)
{
	struct ingenic_adc *adc = dev_get_drvdata(dev);
	unsigned long flags;
	uint32_t value;
	if(!adc)
		return -ENODEV;
	spin_lock_irqsave(&adc->lock,flags);

	value = readl(adc->base + addr_offset);
	value &= ~mask;
	value |= val;
	writel(value,adc->base + addr_offset);
	spin_unlock_irqrestore(&adc->lock,flags);
	return 0;
}
uint32_t adc_read_reg(struct device *dev,uint8_t addr_offset)
{
	struct ingenic_adc *adc = dev_get_drvdata(dev);
	unsigned long flags;
	uint32_t ret;
	if(!adc)
		return -ENODEV;
	spin_lock_irqsave(&adc->lock,flags);
	ret = readl(adc->base + addr_offset);
	spin_unlock_irqrestore(&adc->lock,flags);
	return ret;
}


static void ingenic_adc_clk_div(struct ingenic_adc *adc, const unsigned char clkdiv,
		const unsigned char clkdiv_us, const unsigned short clkdiv_ms)
{
	unsigned int val;

	val = clkdiv | (clkdiv_us << 8) | (clkdiv_ms << 16);
	writel(val, adc->base + INGENIC_REG_ADC_CLKDIV);
}

static int ingenic_alloc_aux_resources(struct ingenic_adc *adc)
{
	struct resource *r = NULL;
	unsigned int i = 0;

	adc->aux_res_ptr = kzalloc(sizeof(struct resource) * adc->priv->aux_channels * 2, GFP_KERNEL);
	if(!adc->aux_res_ptr) {
		return -ENOMEM;
	}

	for(i = 0; i < adc->priv->aux_channels; i++) {
		r = &adc->aux_res_ptr[i * 2];
		r[0].start = i;
		r[0].flags = IORESOURCE_IRQ;
		r[1].start = INGENIC_REG_ADC_AUX_BASE + 2 * i;
		r[1].end   = INGENIC_REG_ADC_AUX_BASE + 2 * i + 1;
		r[1].flags = IORESOURCE_MEM;
	}
	return 0;
}

static int ingenic_alloc_adc_cells(struct ingenic_adc *adc)
{
	unsigned int i;

	adc->adc_cells = kzalloc(sizeof(struct mfd_cell) * adc->priv->aux_channels, GFP_KERNEL);
	if(!adc->adc_cells) {
		return -ENOMEM;
	}

	for(i = 0; i < adc->priv->aux_channels; i++) {
		adc->adc_cells[i].id = i;
		adc->adc_cells[i].name = "ingenic-aux";
		adc->adc_cells[i].num_resources = 2;
		adc->adc_cells[i].resources = &adc->aux_res_ptr[i * 2];
		adc->adc_cells[i].enable = ingenic_adc_cell_enable;
		adc->adc_cells[i].disable = ingenic_adc_cell_disable;
	}
	return 0;
}

static int adc_irq_domain_map(struct irq_domain *d, unsigned int virq, irq_hw_number_t hw)
{
	struct ingenic_adc *adc = d->host_data;
	if(hw == 0) {
		adc->irq_base = virq;
	}
	irq_set_chip_data(virq, adc);
	irq_set_chip_and_handler(virq, &ingenic_adc_irq_chip, handle_level_irq);
	return 0;
}

static const struct irq_domain_ops ingenic_adc_irq_domain_ops = {
	.map = adc_irq_domain_map,
	.xlate = irq_domain_xlate_onetwocell,
};

static int setup_adc_irq(struct ingenic_adc *adc, struct device_node *np)
{
	int i, ret;

	adc->irq = irq_of_parse_and_map(np, 0);
	if (!adc->irq)
		return -EINVAL;

	adc->irq_domain = irq_domain_add_linear(np, adc->priv->aux_channels, &ingenic_adc_irq_domain_ops, (void *)adc);
	if (!adc->irq_domain)
		return -ENOMEM;

	for(i = 0; i < adc->priv->aux_channels; i++) {
		ret = irq_create_mapping(adc->irq_domain, i);
		if(ret < 0) {
			return -1;
		}
	}
	irq_set_handler_data(adc->irq, adc);
	irq_set_chained_handler(adc->irq, ingenic_adc_irq_demux);

	return 0;
}


static int ingenic_adc_probe(struct platform_device *pdev)
{
	int ret;
	struct ingenic_adc *adc;
	struct device_node *np;
	struct resource *mem_base;
	const struct of_device_id *match;
	unsigned char clkdiv, clkdiv_us;
	unsigned short clkdiv_ms;

	np = pdev->dev.of_node;

	adc = kmalloc(sizeof(*adc), GFP_KERNEL);
	if (!adc) {
		dev_err(&pdev->dev, "Failed to allocate driver structre\n");
		return -ENOMEM;
	}

	match = of_match_node(sadc_match, pdev->dev.of_node);
	if (!match)
		return -ENODEV;

	adc->priv = match->data;

	adc->irq = platform_get_irq(pdev, 0);
	if (adc->irq < 0) {
		ret = adc->irq;
		dev_err(&pdev->dev, "Failed to get platform irq: %d\n", ret);
		goto err_free;
	}

	mem_base = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_base) {
		ret = -ENOENT;
		dev_err(&pdev->dev, "Failed to get platform mmio resource");
		goto err_free;
	}

	adc->mem = request_mem_region(mem_base->start, INGENIC_REG_ADC_STATUS, pdev->name);
	if (!adc->mem) {
		ret = -EBUSY;
		dev_err(&pdev->dev, "Failed to request mmio memory region\n");
		goto err_free;
	}

	adc->base = ioremap(adc->mem->start, resource_size(adc->mem));
	if (!adc->base) {
		ret = -EBUSY;
		dev_err(&pdev->dev, "Failed to ioremap mmio memory\n");
		goto err_release_mem_region;
	}

	adc->clk = clk_get(&pdev->dev, "gate_sadc");
	if (IS_ERR(adc->clk)) {
		ret = PTR_ERR(adc->clk);
		dev_err(&pdev->dev, "Failed to get clock: %d\n", ret);
		goto err_iounmap;
	}

	ret = ingenic_alloc_aux_resources(adc);
	if(ret < 0) {
		dev_err(&pdev->dev, "Failed to allocate aux resources: %d\n", ret);
		goto err_free;
	}

	ret = ingenic_alloc_adc_cells(adc);
	if(ret < 0) {
		dev_err(&pdev->dev, "Failed to allocate adc cells: %d\n", ret);
		goto err_free;
	}

	ret = setup_adc_irq(adc, np);
	if(ret < 0) {
		dev_err(&pdev->dev, "Failed to setup adc irq: %d\n", ret);
		goto err_free;
	}

	spin_lock_init(&adc->lock);
	atomic_set(&adc->clk_ref, 0);

	platform_set_drvdata(pdev, adc);

	clk_prepare_enable(adc->clk);

	writew(0x8000, adc->base + INGENIC_REG_ADC_ENABLE);
	writew(0xffff, adc->base + INGENIC_REG_ADC_CTRL);

	clkdiv = CLKDIV - 1;
	clkdiv_us = CLKDIV_US - 1;
	clkdiv_ms = CLKDIV_MS - 1;

	ingenic_adc_clk_div(adc, clkdiv, clkdiv_us, clkdiv_ms);

	ret = mfd_add_devices(&pdev->dev, 0, adc->adc_cells, adc->priv->aux_channels, mem_base, adc->irq_base, NULL);
	if (ret < 0) {
		goto err_clk_put;
	}

	printk("ingenic SADC driver registeres over!\n");

	return 0;

err_clk_put:
	clk_put(adc->clk);
err_iounmap:
	platform_set_drvdata(pdev, NULL);
	iounmap(adc->base);
err_release_mem_region:
	release_mem_region(adc->mem->start, resource_size(adc->mem));
err_free:
	kfree(adc->aux_res_ptr);
	kfree(adc->adc_cells);
	kfree(adc);

	return ret;
}

static int ingenic_adc_remove(struct platform_device *pdev)
{
	struct ingenic_adc *adc = platform_get_drvdata(pdev);

	clk_disable_unprepare(adc->clk);
	mfd_remove_devices(&pdev->dev);

	irq_set_handler_data(adc->irq, NULL);
	irq_set_chained_handler(adc->irq, NULL);

	iounmap(adc->base);
	release_mem_region(adc->mem->start, resource_size(adc->mem));

	clk_put(adc->clk);

	platform_set_drvdata(pdev, NULL);

	kfree(adc->aux_res_ptr);
	kfree(adc->adc_cells);
	kfree(adc);

	return 0;
}


static const struct ingenic_adc_priv x1600_adc_priv[] = {
	{.aux_channels = 4,},
};
static const struct ingenic_adc_priv x2000_adc_priv[] = {
	{.aux_channels = 6,},
};
static const struct ingenic_adc_priv x2500_adc_priv[] = {
	{.aux_channels = 4,},
};
static const struct ingenic_adc_priv x2100_adc_priv[] = {
	{.aux_channels = 6,},
};
static const struct ingenic_adc_priv m300_adc_priv[] = {
	{.aux_channels = 6,},
};

static const struct of_device_id sadc_match[] = {
	{ .compatible = "ingenic,x2000-sadc",.data = (void*)&x2000_adc_priv},
	{ .compatible = "ingenic,x2500-sadc",.data = (void*)&x2500_adc_priv},
	{ .compatible = "ingenic,x1600-sadc",.data = (void*)&x1600_adc_priv},
	{ .compatible = "ingenic,x2100-sadc",.data = (void*)&x2100_adc_priv},
	{ .compatible = "ingenic,m300-sadc",.data = (void*)&m300_adc_priv},
	{},
};
MODULE_DEVICE_TABLE(of, sadc_match);

struct platform_driver ingenic_adc_driver = {
	.probe	= ingenic_adc_probe,
	.remove	= ingenic_adc_remove,
	.driver = {
		.name	= "ingenic-adc",
		.owner	= THIS_MODULE,
		.of_match_table = sadc_match,
	},
};

static int __init ingenic_adc_init(void)
{
	return platform_driver_register(&ingenic_adc_driver);
}
module_init(ingenic_adc_init);

static void __exit ingenic_adc_exit(void)
{
	platform_driver_unregister(&ingenic_adc_driver);
}
module_exit(ingenic_adc_exit);

MODULE_DESCRIPTION("ingenic SOC ADC driver");
MODULE_AUTHOR("Guo Xu<xu.guo@ingenic.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:T15-adc");
