/*
 * omap iommu: omap4 device registration
 *
 * Copyright (C) 2009-2010 Nokia Corporation
 *
 * Written by Hari Kanigeri <h-kanigeri2@ti.com>
 *
 * Added support for OMAP4. This is based on original file
 * omap3-iommu.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>

#include <plat/iommu.h>
#include <plat/irqs.h>

#define OMAP4_MMU1_BASE	0x55082000
#define OMAP4_MMU2_BASE	0x4A066000

#define OMAP4_MMU1_IRQ	INT_44XX_DUCATI_MMU_IRQ
#define OMAP4_MMU2_IRQ	INT_44XX_DSP_MMU



static unsigned long iommu_base[] __initdata = {
	OMAP4_MMU1_BASE,
	OMAP4_MMU2_BASE,
};

static int iommu_irq[] __initdata = {
	OMAP4_MMU1_IRQ,
	OMAP4_MMU2_IRQ,
};

static const struct iommu_platform_data omap4_iommu_pdata[] __initconst = {
	{
		.name = "ducati",
		.nr_tlb_entries = 32,
	},
#if defined(CONFIG_MPU_TESLA_IOMMU)
	{
		.name = "tesla",
		.nr_tlb_entries = 32,
	},
#endif
};
#define NR_IOMMU_DEVICES ARRAY_SIZE(omap4_iommu_pdata)

static struct platform_device *omap4_iommu_pdev[NR_IOMMU_DEVICES];

static int __init omap4_iommu_init(void)
{
	int i, err;

	for (i = 0; i < NR_IOMMU_DEVICES; i++) {
		struct platform_device *pdev;
		struct resource res[2];

		pdev = platform_device_alloc("omap-iommu", i);
		if (!pdev) {
			err = -ENOMEM;
			goto err_out;
		}

		memset(res, 0,  sizeof(res));
		res[0].start = iommu_base[i];
		res[0].end = iommu_base[i] + MMU_REG_SIZE - 1;
		res[0].flags = IORESOURCE_MEM;
		res[1].start = res[1].end = iommu_irq[i];
		res[1].flags = IORESOURCE_IRQ;

		err = platform_device_add_resources(pdev, res,
						    ARRAY_SIZE(res));
		if (err)
			goto err_out;
		err = platform_device_add_data(pdev, &omap4_iommu_pdata[i],
					       sizeof(omap4_iommu_pdata[0]));
		if (err)
			goto err_out;
		err = platform_device_add(pdev);
		if (err)
			goto err_out;
		omap4_iommu_pdev[i] = pdev;
	}
	return 0;

err_out:
	while (i--)
		platform_device_put(omap4_iommu_pdev[i]);
	return err;
}
module_init(omap4_iommu_init);

static void __exit omap4_iommu_exit(void)
{
	int i;

	for (i = 0; i < NR_IOMMU_DEVICES; i++)
		platform_device_unregister(omap4_iommu_pdev[i]);
}
module_exit(omap4_iommu_exit);

MODULE_AUTHOR("Hiroshi DOYU, Hari Kanigeri");
MODULE_DESCRIPTION("omap iommu: omap4 device registration");
MODULE_LICENSE("GPL v2");

