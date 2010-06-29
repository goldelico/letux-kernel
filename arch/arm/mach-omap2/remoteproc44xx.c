/*
 * Remote Processor machine-specific module for OMAP3
 *
 * Copyright (C) 2010 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <plat/remoteproc.h>
#include <mach/irqs.h>
#include <plat/omap_device.h>

#include "cm.h"
#include "prm.h"

#define RM_M3_RST1ST				0x1
#define RM_M3_RST2ST				0x2
#define RM_M3_RST3ST				0x4
#define RM_M3_REL_RST1_MASK			0x2
#define RM_M3_REL_RST2_MASK			0x0
#define RM_M3_AST_RST1_MASK			0x3
#define RM_M3_AST_RST2_MASK			0x2

#define M3_CLK_MOD_MODE_HW_AUTO			0x1
#define M3_CLKTRCTRL_SW_WKUP			0x2
#define M3_CLKTRCTRL_SW_SLEEP			0x1
#define M3_CLKACTIVITY_MPU_M3_CLK		0x100

#define OMAP4_CM_DUCATI_DUCATI_CLKCTRL_OFFSET	0x0220
#define OMAP4_CM_DUCATI_CLKSTCTRL_OFFSET	0x0200
#define OMAP4_RM_DUCATI_RSTCTRL_OFFSET		0x0210
#define OMAP4_RM_DUCATI_RSTST_OFFSET		0x0214
#define OMAP4430_CM2_CORE_MOD_1			0x4700

static inline int proc44x_sysm3_start(struct omap_rproc *rproc)
{
	u32 reg;
	int counter = 10;
	struct device *dev = rproc->dev;

	/* Module is managed automatically by HW */
	cm_write_mod_reg(M3_CLK_MOD_MODE_HW_AUTO, OMAP4430_CM2_CORE_MOD_1,
					OMAP4_CM_DUCATI_DUCATI_CLKCTRL_OFFSET);

	/* Enable the M3 clock */
	cm_write_mod_reg(M3_CLKTRCTRL_SW_WKUP, OMAP4430_CM2_CORE_MOD_1,
					OMAP4_CM_DUCATI_CLKSTCTRL_OFFSET);
	do {
		reg = cm_read_mod_reg(OMAP4430_CM2_CORE_MOD_1,
					OMAP4_CM_DUCATI_CLKSTCTRL_OFFSET);
		if (reg & M3_CLKACTIVITY_MPU_M3_CLK) {
			dev_info(dev, "M3 clock enabled:"
			"OMAP4430_CM_DUCATI_CLKSTCTRL = 0x%x\n", reg);
			break;
		}
		msleep(1);
	} while (--counter);
	if (counter == 0) {
		dev_info(dev, "FAILED TO ENABLE DUCATI M3 CLOCK !%x\n", reg);
		return -EFAULT;
	}

	/* De-assert RST1, and clear the Reset status */
	dev_info(dev, "De-assert RST1\n");
	prm_write_mod_reg(RM_M3_REL_RST1_MASK, OMAP4430_PRM_CORE_MOD,
				OMAP4_RM_DUCATI_RSTCTRL_OFFSET);
	while (!(prm_read_mod_reg(OMAP4430_PRM_CORE_MOD,
		OMAP4_RM_DUCATI_RSTST_OFFSET) & RM_M3_RST1ST))
		;
	dev_info(dev, "RST1 released!");

	prm_write_mod_reg(RM_M3_RST1ST, OMAP4430_PRM_CORE_MOD,
				OMAP4_RM_DUCATI_RSTST_OFFSET);

	return 0;
}

static inline int proc44x_appm3_start(struct omap_rproc *rproc)
{
	struct device *dev = rproc->dev;

	/* De-assert RST2, and clear the Reset status */
	dev_info(dev, "De-assert RST2\n");
	prm_write_mod_reg(RM_M3_REL_RST2_MASK, OMAP4430_PRM_CORE_MOD,
				OMAP4_RM_DUCATI_RSTCTRL_OFFSET);

	while (!(prm_read_mod_reg(OMAP4430_PRM_CORE_MOD,
		OMAP4_RM_DUCATI_RSTST_OFFSET) & RM_M3_RST2ST))
		;
	dev_info(dev, "RST2 released!");

	prm_write_mod_reg(RM_M3_RST2ST, OMAP4430_PRM_CORE_MOD,
				OMAP4_RM_DUCATI_RSTST_OFFSET);

	return 0;
}

static inline int proc44x_sysm3_stop(struct omap_rproc *rproc)
{
	struct device *dev = rproc->dev;
	u32 reg;

	reg = prm_read_mod_reg(OMAP4430_PRM_CORE_MOD,
					OMAP4_RM_DUCATI_RSTCTRL_OFFSET);

	dev_info(dev, "assert RST1 reg = 0x%x\n", reg);
	prm_write_mod_reg((reg | RM_M3_AST_RST1_MASK), OMAP4430_PRM_CORE_MOD,
				OMAP4_RM_DUCATI_RSTCTRL_OFFSET);

	return 0;
}

static inline int proc44x_appm3_stop(struct omap_rproc *rproc)
{
	struct device *dev = rproc->dev;
	u32 reg;

	reg = prm_read_mod_reg(OMAP4430_PRM_CORE_MOD,
					OMAP4_RM_DUCATI_RSTCTRL_OFFSET);

	dev_info(dev, "assert RST2 reg = 0x%x\n", reg);
	prm_write_mod_reg((reg | RM_M3_AST_RST2_MASK), OMAP4430_PRM_CORE_MOD,
				OMAP4_RM_DUCATI_RSTCTRL_OFFSET);
	return 0;
}

static inline int omap4_rproc_get_state(struct omap_rproc *rproc)
{
	return rproc->state;
}

static struct omap_rproc_ops omap4_ducati0_ops = {
	.start = proc44x_sysm3_start,
	.stop = proc44x_sysm3_stop,
	.get_state = omap4_rproc_get_state,
};

static struct omap_rproc_ops omap4_ducati1_ops = {
	.start = proc44x_appm3_start,
	.stop = proc44x_appm3_stop,
	.get_state = omap4_rproc_get_state,
};

static struct omap_rproc_ops omap4_tesla_ops = {
	.start = NULL,
	.stop = NULL,
};

static struct omap_rproc_platform_data omap4_rproc_data[] = {
	{
		.name = "tesla",
		.ops = &omap4_tesla_ops,
		.oh_name = "tesla_hwmod",
	},
	{
		.name = "ducati-proc0",
		.ops = &omap4_ducati0_ops,
		.oh_name = "ducati_hwmod0",
	},
	{
		.name = "ducati-proc1",
		.ops = &omap4_ducati1_ops,
		.oh_name = "ducati_hwmod1",
	},
};

struct omap_rproc_platform_data *remoteproc_get_plat_data(void)
{
	return omap4_rproc_data;
}

int remoteproc_get_plat_data_size(void)
{
	return ARRAY_SIZE(omap4_rproc_data);
}
EXPORT_SYMBOL(remoteproc_get_plat_data_size);


#define NR_RPROC_DEVICES ARRAY_SIZE(omap4_rproc_data)

static struct platform_device *omap4_rproc_pdev[NR_RPROC_DEVICES];

static int __init omap4_rproc_init(void)
{
	int i, err;

	for (i = 0; i < NR_RPROC_DEVICES; i++) {
		struct platform_device *pdev;

		pdev = platform_device_alloc("omap-remoteproc", i);
		if (!pdev) {
			err = -ENOMEM;
			goto err_out;
		}

		err = platform_device_add_data(pdev, &omap4_rproc_data[i],
					       sizeof(omap4_rproc_data[0]));
		err = platform_device_add(pdev);
		if (err)
			goto err_out;
		omap4_rproc_pdev[i] = pdev;
	}
	return 0;

err_out:
	while (i--)
		platform_device_put(omap4_rproc_pdev[i]);
	return err;
}
module_init(omap4_rproc_init);

static void __exit omap4_rproc_exit(void)
{
	int i;

	for (i = 0; i < NR_RPROC_DEVICES; i++)
		platform_device_unregister(omap4_rproc_pdev[i]);
}
module_exit(omap4_rproc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("OMAP4 Remote Processor module");
MODULE_AUTHOR("Ohad Ben-Cohen <ohad@wizery.com>");
MODULE_AUTHOR("Hari Kanigeri <h-kanigeri2@ti.com>");
