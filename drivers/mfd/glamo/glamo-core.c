/* Smedia Glamo 336x/337x driver
 *
 * (C) 2007 by Openmoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/kernel_stat.h>
#include <linux/spinlock.h>
#include <linux/glamofb.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#include "glamo-regs.h"
#include "glamo-core.h"

#define RESSIZE(ressource) (((ressource)->end - (ressource)->start)+1)

#define GLAMO_MEM_REFRESH_COUNT 0x100

static struct glamo_core *glamo_handle;

static inline void __reg_write(struct glamo_core *glamo,
				u_int16_t reg, u_int16_t val)
{
	writew(val, glamo->base + reg);
}

static inline u_int16_t __reg_read(struct glamo_core *glamo,
				   u_int16_t reg)
{
	return readw(glamo->base + reg);
}

static void __reg_set_bit_mask(struct glamo_core *glamo,
				u_int16_t reg, u_int16_t mask,
				u_int16_t val)
{
	u_int16_t tmp;

	val &= mask;

	tmp = __reg_read(glamo, reg);
	tmp &= ~mask;
	tmp |= val;
	__reg_write(glamo, reg, tmp);
}

static void reg_set_bit_mask(struct glamo_core *glamo,
				u_int16_t reg, u_int16_t mask,
				u_int16_t val)
{
	spin_lock(&glamo->lock);
	__reg_set_bit_mask(glamo, reg, mask, val);
	spin_unlock(&glamo->lock);
}

static inline void __reg_set_bit(struct glamo_core *glamo,
				 u_int16_t reg, u_int16_t bit)
{
	__reg_set_bit_mask(glamo, reg, bit, 0xffff);
}

static inline void __reg_clear_bit(struct glamo_core *glamo,
				   u_int16_t reg, u_int16_t bit)
{
	__reg_set_bit_mask(glamo, reg, bit, 0);
}

static inline void glamo_vmem_write(struct glamo_core *glamo, u_int32_t addr,
				    u_int16_t *src, int len)
{
	if (addr & 0x0001 || (unsigned long)src & 0x0001 || len & 0x0001) {
		dev_err(&glamo->pdev->dev, "unaligned write(0x%08x, 0x%p, "
			"0x%x)!!\n", addr, src, len);
	}

}

static inline void glamo_vmem_read(struct glamo_core *glamo, u_int16_t *buf,
				   u_int32_t addr, int len)
{
	if (addr & 0x0001 || (unsigned long) buf & 0x0001 || len & 0x0001) {
		dev_err(&glamo->pdev->dev, "unaligned read(0x%p, 0x08%x, "
			"0x%x)!!\n", buf, addr, len);
	}


}

/***********************************************************************
 * resources of sibling devices
 ***********************************************************************/

#if 0
static struct resource glamo_core_resources[] = {
	{
		.start	= GLAMO_REGOFS_GENERIC,
		.end	= GLAMO_REGOFS_GENERIC + 0x400,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device glamo_core_dev = {
	.name		= "glamo-core",
	.resource	= &glamo_core_resources,
	.num_resources	= ARRAY_SIZE(glamo_core_resources),
};
#endif

static struct resource glamo_jpeg_resources[] = {
	{
		.start	= GLAMO_REGOFS_JPEG,
		.end	= GLAMO_REGOFS_MPEG - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_GLAMO_JPEG,
		.end	= IRQ_GLAMO_JPEG,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device glamo_jpeg_dev = {
	.name		= "glamo-jpeg",
	.resource	= glamo_jpeg_resources,
	.num_resources	= ARRAY_SIZE(glamo_jpeg_resources),
};

static struct resource glamo_mpeg_resources[] = {
	{
		.start	= GLAMO_REGOFS_MPEG,
		.end	= GLAMO_REGOFS_LCD - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_GLAMO_MPEG,
		.end	= IRQ_GLAMO_MPEG,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device glamo_mpeg_dev = {
	.name		= "glamo-mpeg",
	.resource	= glamo_mpeg_resources,
	.num_resources	= ARRAY_SIZE(glamo_mpeg_resources),
};

static struct resource glamo_2d_resources[] = {
	{
		.start	= GLAMO_REGOFS_2D,
		.end	= GLAMO_REGOFS_3D - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_GLAMO_2D,
		.end	= IRQ_GLAMO_2D,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device glamo_2d_dev = {
	.name		= "glamo-2d",
	.resource	= glamo_2d_resources,
	.num_resources	= ARRAY_SIZE(glamo_2d_resources),
};

static struct resource glamo_3d_resources[] = {
	{
		.start	= GLAMO_REGOFS_3D,
		.end	= GLAMO_REGOFS_END - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device glamo_3d_dev = {
	.name		= "glamo-3d",
	.resource	= glamo_3d_resources,
	.num_resources	= ARRAY_SIZE(glamo_3d_resources),
};

static struct platform_device glamo_spigpio_dev = {
	.name		= "glamo-spi-gpio",
};

static struct resource glamo_fb_resources[] = {
	/* FIXME: those need to be incremented by parent base */
	{
		.name	= "glamo-fb-regs",
		.start	= GLAMO_REGOFS_LCD,
		.end	= GLAMO_REGOFS_MMC - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.name	= "glamo-fb-mem",
		.start	= GLAMO_OFFSET_FB,
		.end	= GLAMO_OFFSET_FB + GLAMO_FB_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device glamo_fb_dev = {
	.name		= "glamo-fb",
	.resource	= glamo_fb_resources,
	.num_resources	= ARRAY_SIZE(glamo_fb_resources),
};

static struct resource glamo_mmc_resources[] = {
	{
		/* FIXME: those need to be incremented by parent base */
		.start	= GLAMO_REGOFS_MMC,
		.end	= GLAMO_REGOFS_MPROC0 - 1,
		.flags	= IORESOURCE_MEM
	}, {
		.start	= IRQ_GLAMO_MMC,
		.end	= IRQ_GLAMO_MMC,
		.flags	= IORESOURCE_IRQ,
	}, { /* our data buffer for MMC transfers */
		.start	= GLAMO_OFFSET_FB + GLAMO_FB_SIZE,
		.end	= GLAMO_OFFSET_FB + GLAMO_FB_SIZE +
				  GLAMO_MMC_BUFFER_SIZE - 1,
		.flags	= IORESOURCE_MEM
	},
};

static struct platform_device glamo_mmc_dev = {
	.name		= "glamo-mci",
	.resource	= glamo_mmc_resources,
	.num_resources	= ARRAY_SIZE(glamo_mmc_resources),
};

struct glamo_mci_pdata glamo_mci_def_pdata = {
	.gpio_detect		= 0,
	.glamo_set_mci_power	= NULL, /* filled in from MFD platform data */
	.ocr_avail	= MMC_VDD_20_21 |
			  MMC_VDD_21_22 |
			  MMC_VDD_22_23 |
			  MMC_VDD_23_24 |
			  MMC_VDD_24_25 |
			  MMC_VDD_25_26 |
			  MMC_VDD_26_27 |
			  MMC_VDD_27_28 |
			  MMC_VDD_28_29 |
			  MMC_VDD_29_30 |
			  MMC_VDD_30_31 |
			  MMC_VDD_32_33,
	.glamo_irq_is_wired	= NULL, /* filled in from MFD platform data */
	.mci_suspending = NULL, /* filled in from MFD platform data */
	.mci_all_dependencies_resumed = NULL, /* filled in from MFD plat data */
};
EXPORT_SYMBOL_GPL(glamo_mci_def_pdata);



static void mangle_mem_resources(struct resource *res, int num_res,
				 struct resource *parent)
{
	int i;

	for (i = 0; i < num_res; i++) {
		if (res[i].flags != IORESOURCE_MEM)
			continue;
		res[i].start += parent->start;
		res[i].end += parent->start;
		res[i].parent = parent;
	}
}

/***********************************************************************
 * IRQ demultiplexer
 ***********************************************************************/
#define irq2glamo(x)	(x - IRQ_GLAMO(0))

static void glamo_ack_irq(unsigned int irq)
{
	/* clear interrupt source */
	__reg_write(glamo_handle, GLAMO_REG_IRQ_CLEAR,
		    1 << irq2glamo(irq));
}

static void glamo_mask_irq(unsigned int irq)
{
	u_int16_t tmp;

	/* clear bit in enable register */
	tmp = __reg_read(glamo_handle, GLAMO_REG_IRQ_ENABLE);
	tmp &= ~(1 << irq2glamo(irq));
	__reg_write(glamo_handle, GLAMO_REG_IRQ_ENABLE, tmp);
}

static void glamo_unmask_irq(unsigned int irq)
{
	u_int16_t tmp;

	/* set bit in enable register */
	tmp = __reg_read(glamo_handle, GLAMO_REG_IRQ_ENABLE);
	tmp |= (1 << irq2glamo(irq));
	__reg_write(glamo_handle, GLAMO_REG_IRQ_ENABLE, tmp);
}

static struct irq_chip glamo_irq_chip = {
	.ack	= glamo_ack_irq,
	.mask	= glamo_mask_irq,
	.unmask	= glamo_unmask_irq,
};

static void glamo_irq_demux_handler(unsigned int irq, struct irq_desc *desc)
{
	const unsigned int cpu = smp_processor_id();

	spin_lock(&desc->lock);

	desc->status &= ~(IRQ_REPLAY | IRQ_WAITING);

	if (unlikely(desc->status & IRQ_INPROGRESS)) {
		desc->status |= (IRQ_PENDING | IRQ_MASKED);
		desc->chip->mask(irq);
		desc->chip->ack(irq);
		goto out_unlock;
	}

	kstat_cpu(cpu).irqs[irq]++;
	desc->chip->ack(irq);
	desc->status |= IRQ_INPROGRESS;

	do {
		u_int16_t irqstatus;
		int i;

		if (unlikely((desc->status &
				(IRQ_PENDING | IRQ_MASKED | IRQ_DISABLED)) ==
				(IRQ_PENDING | IRQ_MASKED))) {
			/* dealing with pending IRQ, unmasking */
			desc->chip->unmask(irq);
			desc->status &= ~IRQ_MASKED;
		}

		desc->status &= ~IRQ_PENDING;

		/* read IRQ status register */
		irqstatus = __reg_read(glamo_handle, GLAMO_REG_IRQ_STATUS);
		for (i = 0; i < 9; i++)
			if (irqstatus & (1 << i))
				desc_handle_irq(IRQ_GLAMO(i),
				    irq_desc+IRQ_GLAMO(i));

	} while ((desc->status & (IRQ_PENDING | IRQ_DISABLED)) == IRQ_PENDING);

	desc->status &= ~IRQ_INPROGRESS;

out_unlock:
	spin_unlock(&desc->lock);
}

/***********************************************************************
 * 'engine' support
 ***********************************************************************/

int __glamo_engine_enable(struct glamo_core *glamo, enum glamo_engine engine)
{
	switch (engine) {
	case GLAMO_ENGINE_LCD:
		__reg_set_bit_mask(glamo, GLAMO_REG_CLOCK_LCD,
			    GLAMO_CLOCK_LCD_EN_M5CLK |
			    GLAMO_CLOCK_LCD_EN_DHCLK |
			    GLAMO_CLOCK_LCD_EN_DMCLK |
			    GLAMO_CLOCK_LCD_EN_DCLK |
			    GLAMO_CLOCK_LCD_DG_M5CLK |
			    GLAMO_CLOCK_LCD_DG_DMCLK, 0xffff);
		__reg_set_bit_mask(glamo, GLAMO_REG_CLOCK_GEN5_1,
			    GLAMO_CLOCK_GEN51_EN_DIV_DHCLK |
			    GLAMO_CLOCK_GEN51_EN_DIV_DMCLK |
			    GLAMO_CLOCK_GEN51_EN_DIV_DCLK, 0xffff);
		__reg_set_bit_mask(glamo, GLAMO_REG_HOSTBUS(2),
			    GLAMO_HOSTBUS2_MMIO_EN_LCD,
			    0xffff);
		break;
	case GLAMO_ENGINE_MMC:
		__reg_set_bit_mask(glamo, GLAMO_REG_CLOCK_MMC,
				   GLAMO_CLOCK_MMC_EN_M9CLK |
				   GLAMO_CLOCK_MMC_EN_TCLK |
				   GLAMO_CLOCK_MMC_DG_M9CLK |
				   GLAMO_CLOCK_MMC_DG_TCLK, 0xffff);
		__reg_set_bit_mask(glamo, GLAMO_REG_HOSTBUS(2),
				   GLAMO_HOSTBUS2_MMIO_EN_MMC,
				   GLAMO_HOSTBUS2_MMIO_EN_MMC);
		break;
	case GLAMO_ENGINE_2D:
		__reg_set_bit_mask(glamo, GLAMO_REG_CLOCK_2D,
				   GLAMO_CLOCK_2D_EN_M7CLK |
				   GLAMO_CLOCK_2D_EN_GCLK |
				   GLAMO_CLOCK_2D_DG_M7CLK |
				   GLAMO_CLOCK_2D_DG_GCLK, 0xffff);
		__reg_set_bit_mask(glamo, GLAMO_REG_HOSTBUS(2),
				   GLAMO_HOSTBUS2_MMIO_EN_2D,
				   GLAMO_HOSTBUS2_MMIO_EN_2D);
		break;
	case GLAMO_ENGINE_CMDQ:
		__reg_set_bit_mask(glamo, GLAMO_REG_CLOCK_2D,
				   GLAMO_CLOCK_2D_EN_M6CLK, 0xffff);
		__reg_set_bit_mask(glamo, GLAMO_REG_HOSTBUS(2),
				   GLAMO_HOSTBUS2_MMIO_EN_CQ,
				   GLAMO_HOSTBUS2_MMIO_EN_CQ);
		break;
	/* FIXME: Implementation */
	default:
		break;
	}

	glamo->engine_enabled_bitfield |= 1 << engine;

	return 0;
}

int glamo_engine_enable(struct glamo_core *glamo, enum glamo_engine engine)
{
	int ret;

	spin_lock(&glamo->lock);

	ret = __glamo_engine_enable(glamo, engine);

	spin_unlock(&glamo->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(glamo_engine_enable);

int __glamo_engine_disable(struct glamo_core *glamo, enum glamo_engine engine)
{
	switch (engine) {
	case GLAMO_ENGINE_LCD:
		/* remove pixel clock to LCM */
		__reg_set_bit_mask(glamo, GLAMO_REG_CLOCK_LCD,
			    GLAMO_CLOCK_LCD_EN_DCLK, 0);
		__reg_set_bit_mask(glamo, GLAMO_REG_CLOCK_LCD,
			    GLAMO_CLOCK_LCD_EN_DHCLK |
			    GLAMO_CLOCK_LCD_EN_DMCLK, 0);
		/* kill memory clock */
		__reg_set_bit_mask(glamo, GLAMO_REG_CLOCK_LCD,
			    GLAMO_CLOCK_LCD_EN_M5CLK, 0);
		/* stop dividing the clocks */
		__reg_set_bit_mask(glamo, GLAMO_REG_CLOCK_GEN5_1,
			    GLAMO_CLOCK_GEN51_EN_DIV_DHCLK |
			    GLAMO_CLOCK_GEN51_EN_DIV_DMCLK |
			    GLAMO_CLOCK_GEN51_EN_DIV_DCLK, 0);
		__reg_set_bit_mask(glamo, GLAMO_REG_HOSTBUS(2),
			    GLAMO_HOSTBUS2_MMIO_EN_LCD, 0);
		break;

	case GLAMO_ENGINE_MMC:
		__reg_set_bit_mask(glamo, GLAMO_REG_CLOCK_MMC, 0,
						   GLAMO_CLOCK_MMC_EN_M9CLK |
						   GLAMO_CLOCK_MMC_EN_TCLK |
						   GLAMO_CLOCK_MMC_DG_M9CLK |
						   GLAMO_CLOCK_MMC_DG_TCLK);
		/* disable the TCLK divider clk input */
		__reg_set_bit_mask(glamo, GLAMO_REG_CLOCK_GEN5_1, 0,
						GLAMO_CLOCK_GEN51_EN_DIV_TCLK);
		__reg_set_bit_mask(glamo, GLAMO_REG_HOSTBUS(2), 0,
						   GLAMO_HOSTBUS2_MMIO_EN_MMC);
		/* good idea to hold the thing in reset when we power it off? */
/*		writew(readw(glamo->base + GLAMO_REG_CLOCK_MMC) |
		      GLAMO_CLOCK_MMC_RESET, glamo->base + GLAMO_REG_CLOCK_MMC);
*/
		break;
	default:
		break;
	}

	glamo->engine_enabled_bitfield &= ~(1 << engine);

	return 0;
}
int glamo_engine_disable(struct glamo_core *glamo, enum glamo_engine engine)
{
	int ret;

	spin_lock(&glamo->lock);

	ret = __glamo_engine_disable(glamo, engine);

	spin_unlock(&glamo->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(glamo_engine_disable);

static const u_int16_t engine_clock_regs[__NUM_GLAMO_ENGINES] = {
	[GLAMO_ENGINE_LCD]	= GLAMO_REG_CLOCK_LCD,
	[GLAMO_ENGINE_MMC]	= GLAMO_REG_CLOCK_MMC,
	[GLAMO_ENGINE_ISP]	= GLAMO_REG_CLOCK_ISP,
	[GLAMO_ENGINE_JPEG]	= GLAMO_REG_CLOCK_JPEG,
	[GLAMO_ENGINE_3D]	= GLAMO_REG_CLOCK_3D,
	[GLAMO_ENGINE_2D]	= GLAMO_REG_CLOCK_2D,
	[GLAMO_ENGINE_MPEG_ENC] = GLAMO_REG_CLOCK_MPEG,
	[GLAMO_ENGINE_MPEG_DEC] = GLAMO_REG_CLOCK_MPEG,
};

void glamo_engine_clkreg_set(struct glamo_core *glamo,
			     enum glamo_engine engine,
			     u_int16_t mask, u_int16_t val)
{
	reg_set_bit_mask(glamo, engine_clock_regs[engine], mask, val);
}
EXPORT_SYMBOL_GPL(glamo_engine_clkreg_set);

u_int16_t glamo_engine_clkreg_get(struct glamo_core *glamo,
				  enum glamo_engine engine)
{
	u_int16_t val;

	spin_lock(&glamo->lock);
	val = __reg_read(glamo, engine_clock_regs[engine]);
	spin_unlock(&glamo->lock);

	return val;
}
EXPORT_SYMBOL_GPL(glamo_engine_clkreg_get);

struct glamo_script reset_regs[] = {
	[GLAMO_ENGINE_LCD] = {
		GLAMO_REG_CLOCK_LCD, GLAMO_CLOCK_LCD_RESET
	},
#if 0
	[GLAMO_ENGINE_HOST] = {
		GLAMO_REG_CLOCK_HOST, GLAMO_CLOCK_HOST_RESET
	},
	[GLAMO_ENGINE_MEM] = {
		GLAMO_REG_CLOCK_MEM, GLAMO_CLOCK_MEM_RESET
	},
#endif
	[GLAMO_ENGINE_MMC] = {
		GLAMO_REG_CLOCK_MMC, GLAMO_CLOCK_MMC_RESET
	},
	[GLAMO_ENGINE_2D] = {
		GLAMO_REG_CLOCK_2D, GLAMO_CLOCK_2D_RESET
	},
	[GLAMO_ENGINE_JPEG] = {
		GLAMO_REG_CLOCK_JPEG, GLAMO_CLOCK_JPEG_RESET
	},
};

void glamo_engine_reset(struct glamo_core *glamo, enum glamo_engine engine)
{
	struct glamo_script *rst;

	if (engine >= ARRAY_SIZE(reset_regs)) {
		dev_warn(&glamo->pdev->dev, "unknown engine %u ", engine);
		return;
	}

	rst = &reset_regs[engine];

	spin_lock(&glamo->lock);
	__reg_set_bit(glamo, rst->reg, rst->val);
	spin_unlock(&glamo->lock);

	msleep(1);

	spin_lock(&glamo->lock);
	__reg_clear_bit(glamo, rst->reg, rst->val);
	spin_unlock(&glamo->lock);

	msleep(1);
}
EXPORT_SYMBOL_GPL(glamo_engine_reset);

void glamo_lcm_reset(int level)
{
	if (!glamo_handle)
		return;

	glamo_gpio_setpin(glamo_handle, GLAMO_GPIO4, level);
	glamo_gpio_cfgpin(glamo_handle, GLAMO_GPIO4_OUTPUT);

}
EXPORT_SYMBOL_GPL(glamo_lcm_reset);

enum glamo_pll {
	GLAMO_PLL1,
	GLAMO_PLL2,
};

static int glamo_pll_rate(struct glamo_core *glamo,
			  enum glamo_pll pll)
{
	u_int16_t reg;
	unsigned int div = 512;
	/* FIXME: move osci into platform_data */
	unsigned int osci = 32768;

	if (osci == 32768)
		div = 1;

	switch (pll) {
	case GLAMO_PLL1:
		reg = __reg_read(glamo, GLAMO_REG_PLL_GEN1);
		break;
	case GLAMO_PLL2:
		reg = __reg_read(glamo, GLAMO_REG_PLL_GEN3);
		break;
	default:
		return -EINVAL;
	}
	return (osci/div)*reg;
}

int glamo_engine_reclock(struct glamo_core *glamo,
			 enum glamo_engine engine,
			 int ps)
{
	int pll, khz;
	u_int16_t reg, mask, val = 0;

	if (!ps)
		return 0;

	switch (engine) {
	case GLAMO_ENGINE_LCD:
		pll = GLAMO_PLL1;
		reg = GLAMO_REG_CLOCK_GEN7;
		mask = 0xff;
		break;
	default:
		dev_warn(&glamo->pdev->dev,
			 "reclock of engine 0x%x not supported\n", engine);
		return -EINVAL;
		break;
	}

	pll = glamo_pll_rate(glamo, pll);
	khz = 1000000000UL / ps;

	if (khz)
		val = (pll / khz) / 1000;

	dev_dbg(&glamo->pdev->dev,
			"PLL %d, kHZ %d, div %d\n", pll, khz, val);

	if (val) {
		val--;
		reg_set_bit_mask(glamo, reg, mask, val);
		mdelay(5); /* wait some time to stabilize */

		return 0;
	} else {
		return -EINVAL;
	}
}
EXPORT_SYMBOL_GPL(glamo_engine_reclock);

/***********************************************************************
 * script support
 ***********************************************************************/

int glamo_run_script(struct glamo_core *glamo, struct glamo_script *script,
		     int len, int may_sleep)
{
	int i;

	for (i = 0; i < len; i++) {
		struct glamo_script *line = &script[i];

		switch (line->reg) {
		case 0xffff:
			return 0;
		case 0xfffe:
			if (may_sleep)
				msleep(line->val);
			else
				mdelay(line->val * 4);
			break;
		case 0xfffd:
			/* spin until PLLs lock */
			while ((__reg_read(glamo, GLAMO_REG_PLL_GEN5) & 3) != 3)
				;
			break;
		default:
			__reg_write(glamo, script[i].reg, script[i].val);
			break;
		}
	}

	return 0;
}
EXPORT_SYMBOL(glamo_run_script);

static struct glamo_script glamo_init_script[] = {
	{ GLAMO_REG_CLOCK_HOST,		0x1000 },
		{ 0xfffe, 2 },
	{ GLAMO_REG_CLOCK_MEMORY, 	0x1000 },
	{ GLAMO_REG_CLOCK_MEMORY,	0x2000 },
	{ GLAMO_REG_CLOCK_LCD,		0x1000 },
	{ GLAMO_REG_CLOCK_MMC,		0x1000 },
	{ GLAMO_REG_CLOCK_ISP,		0x1000 },
	{ GLAMO_REG_CLOCK_ISP,		0x3000 },
	{ GLAMO_REG_CLOCK_JPEG,		0x1000 },
	{ GLAMO_REG_CLOCK_3D,		0x1000 },
	{ GLAMO_REG_CLOCK_3D,		0x3000 },
	{ GLAMO_REG_CLOCK_2D,		0x1000 },
	{ GLAMO_REG_CLOCK_2D,		0x3000 },
	{ GLAMO_REG_CLOCK_RISC1,	0x1000 },
	{ GLAMO_REG_CLOCK_MPEG,		0x3000 },
	{ GLAMO_REG_CLOCK_MPEG,		0x3000 },
	{ GLAMO_REG_CLOCK_MPROC,	0x1000 /*0x100f*/ },
		{ 0xfffe, 2 },
	{ GLAMO_REG_CLOCK_HOST,		0x0000 },
	{ GLAMO_REG_CLOCK_MEMORY,	0x0000 },
	{ GLAMO_REG_CLOCK_LCD,		0x0000 },
	{ GLAMO_REG_CLOCK_MMC,		0x0000 },
#if 0
/* unused engines must be left in reset to stop MMC block read "blackouts" */
	{ GLAMO_REG_CLOCK_ISP,		0x0000 },
	{ GLAMO_REG_CLOCK_ISP,		0x0000 },
	{ GLAMO_REG_CLOCK_JPEG,		0x0000 },
	{ GLAMO_REG_CLOCK_3D,		0x0000 },
	{ GLAMO_REG_CLOCK_3D,		0x0000 },
	{ GLAMO_REG_CLOCK_2D,		0x0000 },
	{ GLAMO_REG_CLOCK_2D,		0x0000 },
	{ GLAMO_REG_CLOCK_RISC1,	0x0000 },
	{ GLAMO_REG_CLOCK_MPEG,		0x0000 },
	{ GLAMO_REG_CLOCK_MPEG,		0x0000 },
#endif
	{ GLAMO_REG_PLL_GEN1,		0x05db },	/* 48MHz */
	{ GLAMO_REG_PLL_GEN3,		0x0aba },	/* 90MHz */
	{ 0xfffd, 0 },
	/*
	 * b9 of this register MUST be zero to get any interrupts on INT#
	 * the other set bits enable all the engine interrupt sources
	 */
	{ GLAMO_REG_IRQ_ENABLE,		0x01ff },
	{ GLAMO_REG_CLOCK_GEN6,		0x2000 },
	{ GLAMO_REG_CLOCK_GEN7,		0x0101 },
	{ GLAMO_REG_CLOCK_GEN8,		0x0100 },
	{ GLAMO_REG_CLOCK_HOST,		0x000d },
	{ 0x200,	0x0ef0 },
	{ 0x202, 	0x07ff },
	{ 0x212,	0x0000 },
	{ 0x214,	0x4000 },
	{ 0x216,	0xf00e },

	/* S-Media recommended "set tiling mode to 512 mode for memory access
	 * more efficiency when 640x480" */
	{ GLAMO_REG_MEM_TYPE,		0x0c74 }, /* 8MB, 16 word pg wr+rd */
	{ GLAMO_REG_MEM_GEN,		0xafaf }, /* 63 grants min + max */
	/*
	 * the register below originally 0x0108 makes unreliable Glamo MMC
	 * write operations.  Cranked to 0x05ad to add a wait state, the
	 * unreliability is not seen after 4GB of write / read testing
	 */
	{ GLAMO_REG_MEM_TIMING1,	0x0108 },
	{ GLAMO_REG_MEM_TIMING2,	0x0010 }, /* Taa = 3 MCLK */
	{ GLAMO_REG_MEM_TIMING3,	0x0000 },
	{ GLAMO_REG_MEM_TIMING4,	0x0000 }, /* CE1# delay fall/rise */
	{ GLAMO_REG_MEM_TIMING5,	0x0000 }, /* UB# LB# */
	{ GLAMO_REG_MEM_TIMING6,	0x0000 }, /* OE# */
	{ GLAMO_REG_MEM_TIMING7,	0x0000 }, /* WE# */
	{ GLAMO_REG_MEM_TIMING8,	0x1002 }, /* MCLK delay, was 0x1000 */
	{ GLAMO_REG_MEM_TIMING9,	0x6006 },
	{ GLAMO_REG_MEM_TIMING10,	0x00ff },
	{ GLAMO_REG_MEM_TIMING11,	0x0001 },
	{ GLAMO_REG_MEM_POWER1,		0x0020 },
	{ GLAMO_REG_MEM_POWER2,		0x0000 },
	{ GLAMO_REG_MEM_DRAM1,		0x0000 },
		{ 0xfffe, 1 },
	{ GLAMO_REG_MEM_DRAM1,		0xc100 },
		{ 0xfffe, 1 },
	{ GLAMO_REG_MEM_DRAM1,		0xe100 },
	{ GLAMO_REG_MEM_DRAM2,		0x01d6 },
	{ GLAMO_REG_CLOCK_MEMORY,	0x000b },
};


enum glamo_power {
	GLAMO_POWER_ON,
	GLAMO_POWER_SUSPEND,
};

static void glamo_power(struct glamo_core *glamo,
			enum glamo_power new_state)
{
	int n;

	spin_lock(&glamo->lock);

	dev_dbg(&glamo->pdev->dev, "***** glamo_power -> %d\n", new_state);

	switch (new_state) {
	case GLAMO_POWER_ON:
		/* power up PLL1 and PLL2 */
		__reg_set_bit_mask(glamo, GLAMO_REG_DFT_GEN6, 0x0001, 0xffff);
		__reg_set_bit_mask(glamo, GLAMO_REG_PLL_GEN3, 0x2000, 0x0000);

		/* spin until PLL1 and PLL2 lock */
		while ((__reg_read(glamo, GLAMO_REG_PLL_GEN5) & 3) != 3)
			;

		/* Get memory out of deep powerdown */

		__reg_write(glamo, GLAMO_REG_MEM_DRAM2,
					(7 << 6) | /* tRC */
					(1 << 4) | /* tRP */
					(1 << 2) | /* tRCD */
					2); /* CAS latency */

		/* Stop self-refresh */

		__reg_write(glamo, GLAMO_REG_MEM_DRAM1,
					GLAMO_MEM_DRAM1_EN_DRAM_REFRESH |
					GLAMO_MEM_DRAM1_EN_GATE_CKE |
					GLAMO_MEM_REFRESH_COUNT);
		__reg_write(glamo, GLAMO_REG_MEM_DRAM1,
					GLAMO_MEM_DRAM1_EN_MODEREG_SET |
					GLAMO_MEM_DRAM1_EN_DRAM_REFRESH |
					GLAMO_MEM_DRAM1_EN_GATE_CKE |
					GLAMO_MEM_REFRESH_COUNT);

		/* re-enable clocks to memory */

		__reg_write(glamo, GLAMO_REG_CLOCK_MEMORY,
					GLAMO_CLOCK_MEM_EN_MOCACLK |
					GLAMO_CLOCK_MEM_EN_M1CLK |
					GLAMO_CLOCK_MEM_DG_M1CLK);

		/* restore each engine that was up before suspend */
		for (n = 0; n < __NUM_GLAMO_ENGINES; n++)
			if (glamo->engine_enabled_bitfield_suspend & (1 << n))
				__glamo_engine_enable(glamo, n);
		break;

	case GLAMO_POWER_SUSPEND:
		/* stash a copy of which engines were running */
		glamo->engine_enabled_bitfield_suspend =
						 glamo->engine_enabled_bitfield;

		/* take down each engine before we kill mem and pll */
		for (n = 0; n < __NUM_GLAMO_ENGINES; n++)
			if (glamo->engine_enabled_bitfield & (1 << n))
				__glamo_engine_disable(glamo, n);

		/* enable self-refresh */

		__reg_write(glamo, GLAMO_REG_MEM_DRAM1,
					GLAMO_MEM_DRAM1_EN_DRAM_REFRESH |
					GLAMO_MEM_DRAM1_EN_GATE_CKE |
					GLAMO_MEM_DRAM1_SELF_REFRESH |
					GLAMO_MEM_REFRESH_COUNT);
		__reg_write(glamo, GLAMO_REG_MEM_DRAM1,
					GLAMO_MEM_DRAM1_EN_MODEREG_SET |
					GLAMO_MEM_DRAM1_EN_DRAM_REFRESH |
					GLAMO_MEM_DRAM1_EN_GATE_CKE |
					GLAMO_MEM_DRAM1_SELF_REFRESH |
					GLAMO_MEM_REFRESH_COUNT);

		/* force RAM into deep powerdown */

		__reg_write(glamo, GLAMO_REG_MEM_DRAM2,
					GLAMO_MEM_DRAM2_DEEP_PWRDOWN |
					(7 << 6) | /* tRC */
					(1 << 4) | /* tRP */
					(1 << 2) | /* tRCD */
					2); /* CAS latency */

		/* kill clocks to memory */

		__reg_write(glamo, GLAMO_REG_CLOCK_MEMORY, 0);

		/* power down PLL2 and then PLL1 */
		__reg_set_bit_mask(glamo, GLAMO_REG_PLL_GEN3, 0x2000, 0xffff);
		__reg_set_bit_mask(glamo, GLAMO_REG_DFT_GEN5, 0x0001, 0xffff);
		break;
	}

	spin_unlock(&glamo->lock);
}

#if 0
#define MEMDETECT_RETRY	6
static unsigned int detect_memsize(struct glamo_core *glamo)
{
	int i;

	/*static const u_int16_t pattern[] = {
		0x1111, 0x8a8a, 0x2222, 0x7a7a,
		0x3333, 0x6a6a, 0x4444, 0x5a5a,
		0x5555, 0x4a4a, 0x6666, 0x3a3a,
		0x7777, 0x2a2a, 0x8888, 0x1a1a
	}; */

	for (i = 0; i < MEMDETECT_RETRY; i++) {
		switch (glamo->type) {
		case 3600:
			__reg_write(glamo, GLAMO_REG_MEM_TYPE, 0x0072);
			__reg_write(glamo, GLAMO_REG_MEM_DRAM1, 0xc100);
			break;
		case 3650:
			switch (glamo->revision) {
			case GLAMO_CORE_REV_A0:
				if (i & 1)
					__reg_write(glamo, GLAMO_REG_MEM_TYPE,
						    0x097a);
				else
					__reg_write(glamo, GLAMO_REG_MEM_TYPE,
						    0x0173);

				__reg_write(glamo, GLAMO_REG_MEM_DRAM1, 0x0000);
				msleep(1);
				__reg_write(glamo, GLAMO_REG_MEM_DRAM1, 0xc100);
				break;
			default:
				if (i & 1)
					__reg_write(glamo, GLAMO_REG_MEM_TYPE,
						    0x0972);
				else
					__reg_write(glamo, GLAMO_REG_MEM_TYPE,
						    0x0872);

				__reg_write(glamo, GLAMO_REG_MEM_DRAM1, 0x0000);
				msleep(1);
				__reg_write(glamo, GLAMO_REG_MEM_DRAM1, 0xe100);
				break;
			}
			break;
		case 3700:
			/* FIXME */
		default:
			break;
		}

#if 0
		/* FIXME: finish implementation */
		for (j = 0; j < 8; j++) {
			__
#endif
	}

	return 0;
}
#endif

/* Find out if we can support this version of the Glamo chip */
static int glamo_supported(struct glamo_core *glamo)
{
	u_int16_t dev_id, rev_id; /*, memsize; */

	dev_id = __reg_read(glamo, GLAMO_REG_DEVICE_ID);
	rev_id = __reg_read(glamo, GLAMO_REG_REVISION_ID);

	switch (dev_id) {
	case 0x3650:
		switch (rev_id) {
		case GLAMO_CORE_REV_A2:
			break;
		case GLAMO_CORE_REV_A0:
		case GLAMO_CORE_REV_A1:
		case GLAMO_CORE_REV_A3:
			dev_warn(&glamo->pdev->dev, "untested core revision "
				 "%04x, your mileage may vary\n", rev_id);
			break;
		default:
			dev_warn(&glamo->pdev->dev, "unknown glamo revision "
				 "%04x, your mileage may vary\n", rev_id);
			/* maybe should abort ? */
		}
		break;
	case 0x3600:
	case 0x3700:
	default:
		dev_err(&glamo->pdev->dev, "unsupported Glamo device %04x\n",
			dev_id);
		return 0;
	}

	dev_info(&glamo->pdev->dev, "Detected Glamo core %04x Revision %04x "
		 "(%uHz CPU / %uHz Memory)\n", dev_id, rev_id,
		 glamo_pll_rate(glamo, GLAMO_PLL1),
		 glamo_pll_rate(glamo, GLAMO_PLL2));

	return 1;
}

static ssize_t regs_write(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	unsigned long reg = simple_strtoul(buf, NULL, 10);
	struct glamo_core *glamo = dev_get_drvdata(dev);

	while (*buf && (*buf != ' '))
		buf++;
	if (*buf != ' ')
		return -EINVAL;
	while (*buf && (*buf == ' '))
		buf++;
	if (!*buf)
		return -EINVAL;

	printk(KERN_INFO"reg 0x%02lX <-- 0x%04lX\n",
	       reg, simple_strtoul(buf, NULL, 10));

	__reg_write(glamo, reg, simple_strtoul(buf, NULL, 10));

	return count;
}

static ssize_t regs_read(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct glamo_core *glamo = dev_get_drvdata(dev);
	int n, n1 = 0, r;
	char * end = buf;
	struct reg_range {
		int start;
		int count;
		char * name;
	};
	struct reg_range reg_range[] = {
		{ 0x0000, 0x76, "General" },
		{ 0x0200, 0x100, "Host Bus" },
		{ 0x0300, 0x38, "Memory" },
/*		{ 0x0400, 0x100, "Sensor" },
		{ 0x0500, 0x300, "ISP" },
		{ 0x0800, 0x400, "JPEG" },
		{ 0x0c00, 0x500, "MPEG" },
		{ 0x1100, 0x400, "LCD" },
		{ 0x1500, 0x080, "MPU 0" },
		{ 0x1580, 0x080, "MPU 1" },
		{ 0x1600, 0x080, "Command Queue" },
		{ 0x1680, 0x080, "RISC CPU" },
		{ 0x1700, 0x400, "2D Unit" },
		{ 0x1b00, 0x900, "3D Unit" },
*/
	};

	spin_lock(&glamo->lock);

	for (r = 0; r < ARRAY_SIZE(reg_range); r++) {
		n1 = 0;
		end += sprintf(end, "\n%s\n", reg_range[r].name);
		for (n = reg_range[r].start;
		     n < reg_range[r].start + reg_range[r].count; n += 2) {
			if (((n1++) & 7) == 0)
				end += sprintf(end, "\n%04X:  ", n);
			end += sprintf(end, "%04x ", __reg_read(glamo, n));
		}
		end += sprintf(end, "\n");
	}
	spin_unlock(&glamo->lock);

	return end - buf;
}

static DEVICE_ATTR(regs, 0644, regs_read, regs_write);
static struct attribute *glamo_sysfs_entries[] = {
	&dev_attr_regs.attr,
	NULL
};
static struct attribute_group glamo_attr_group = {
	.name	= NULL,
	.attrs	= glamo_sysfs_entries,
};



static int __init glamo_probe(struct platform_device *pdev)
{
	int rc = 0, irq;
	struct glamo_core *glamo;

	if (glamo_handle) {
		dev_err(&pdev->dev,
			"This driver supports only one instance\n");
		return -EBUSY;
	}

	glamo = kmalloc(GFP_KERNEL, sizeof(*glamo));
	if (!glamo)
		return -ENOMEM;

	spin_lock_init(&glamo->lock);
	glamo_handle = glamo;
	glamo->pdev = pdev;
	glamo->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	glamo->irq = platform_get_irq(pdev, 0);
	glamo->pdata = pdev->dev.platform_data;
	if (!glamo->mem || !glamo->pdata) {
		dev_err(&pdev->dev, "platform device with no MEM/PDATA ?\n");
		rc = -ENOENT;
		goto out_free;
	}

	init_resume_dependency_list(&glamo->resume_dependency);

	/* register a number of sibling devices whoise IOMEM resources
	 * are siblings of pdev's IOMEM resource */
#if 0
	glamo_core_dev.dev.parent = &pdev.dev;
	mangle_mem_resources(glamo_core_dev.resources,
			     glamo_core_dev.num_resources, glamo->mem);
	glamo_core_dev.resources[1].start = glamo->irq;
	glamo_core_dev.resources[1].end = glamo->irq;
	platform_device_register(&glamo_core_dev);
#endif
	/* only remap the generic, hostbus and memory controller registers */
	glamo->base = ioremap(glamo->mem->start, GLAMO_REGOFS_VIDCAP);
	if (!glamo->base) {
		dev_err(&pdev->dev, "failed to ioremap() memory region\n");
		goto out_free;
	}

	/* bring MCI specific stuff over from our MFD platform data */
	glamo_mci_def_pdata.glamo_set_mci_power =
					glamo->pdata->glamo_set_mci_power;
	glamo_mci_def_pdata.glamo_mci_use_slow =
					glamo->pdata->glamo_mci_use_slow;
	glamo_mci_def_pdata.glamo_irq_is_wired =
					glamo->pdata->glamo_irq_is_wired;
	glamo_mci_def_pdata.mci_suspending =
					glamo->pdata->mci_suspending;
	glamo_mci_def_pdata.mci_all_dependencies_resumed =
				glamo->pdata->mci_all_dependencies_resumed;

	glamo_2d_dev.dev.parent = &pdev->dev;
	mangle_mem_resources(glamo_2d_dev.resource,
			     glamo_2d_dev.num_resources, glamo->mem);
	platform_device_register(&glamo_2d_dev);

	glamo_3d_dev.dev.parent = &pdev->dev;
	mangle_mem_resources(glamo_3d_dev.resource,
			     glamo_3d_dev.num_resources, glamo->mem);
	platform_device_register(&glamo_3d_dev);

	glamo_jpeg_dev.dev.parent = &pdev->dev;
	mangle_mem_resources(glamo_jpeg_dev.resource,
			     glamo_jpeg_dev.num_resources, glamo->mem);
	platform_device_register(&glamo_jpeg_dev);

	glamo_mpeg_dev.dev.parent = &pdev->dev;
	mangle_mem_resources(glamo_mpeg_dev.resource,
			     glamo_mpeg_dev.num_resources, glamo->mem);
	platform_device_register(&glamo_mpeg_dev);

	glamo->pdata->glamo = glamo;
	glamo_fb_dev.dev.parent = &pdev->dev;
	glamo_fb_dev.dev.platform_data = glamo->pdata;
	mangle_mem_resources(glamo_fb_dev.resource,
			     glamo_fb_dev.num_resources, glamo->mem);
	platform_device_register(&glamo_fb_dev);

	glamo->pdata->spigpio_info->glamo = glamo;
	glamo_spigpio_dev.dev.parent = &pdev->dev;
	glamo_spigpio_dev.dev.platform_data = glamo->pdata->spigpio_info;
	platform_device_register(&glamo_spigpio_dev);

	glamo_mmc_dev.dev.parent = &pdev->dev;
	/* we need it later to give to the engine enable and disable */
	glamo_mci_def_pdata.pglamo = glamo;
	mangle_mem_resources(glamo_mmc_dev.resource,
			     glamo_mmc_dev.num_resources, glamo->mem);
	platform_device_register(&glamo_mmc_dev);

	/* only request the generic, hostbus and memory controller MMIO */
	glamo->mem = request_mem_region(glamo->mem->start,
					GLAMO_REGOFS_VIDCAP, "glamo-core");
	if (!glamo->mem) {
		dev_err(&pdev->dev, "failed to request memory region\n");
		goto out_free;
	}

	if (!glamo_supported(glamo)) {
		dev_err(&pdev->dev, "This Glamo is not supported\n");
		goto out_free;
	}

	rc = sysfs_create_group(&pdev->dev.kobj, &glamo_attr_group);
	if (rc < 0) {
		dev_err(&pdev->dev, "cannot create sysfs group\n");
		goto out_free;
	}

	platform_set_drvdata(pdev, glamo);

	dev_dbg(&glamo->pdev->dev, "running init script\n");
	glamo_run_script(glamo, glamo_init_script,
			 ARRAY_SIZE(glamo_init_script), 1);

	dev_info(&glamo->pdev->dev, "Glamo core now %uHz CPU / %uHz Memory)\n",
		 glamo_pll_rate(glamo, GLAMO_PLL1),
		 glamo_pll_rate(glamo, GLAMO_PLL2));

	glamo_lcm_reset(1);

	for (irq = IRQ_GLAMO(0); irq <= IRQ_GLAMO(8); irq++) {
		set_irq_chip(irq, &glamo_irq_chip);
		set_irq_handler(irq, handle_level_irq);
		set_irq_flags(irq, IRQF_VALID);
	}

	if (glamo->pdata->glamo_irq_is_wired &&
	    !glamo->pdata->glamo_irq_is_wired()) {
		set_irq_chained_handler(glamo->irq, glamo_irq_demux_handler);
		set_irq_type(glamo->irq, IRQT_FALLING);
		glamo->irq_works = 1;
	} else
		glamo->irq_works = 0;

	return 0;

out_free:
	glamo_handle = NULL;
	kfree(glamo);
	return rc;
}

static int glamo_remove(struct platform_device *pdev)
{
	struct glamo_core *glamo = platform_get_drvdata(pdev);
	int irq;

	disable_irq(glamo->irq);
	set_irq_chained_handler(glamo->irq, NULL);

	for (irq = IRQ_GLAMO(0); irq <= IRQ_GLAMO(8); irq++) {
		set_irq_flags(irq, 0);
		set_irq_chip(irq, NULL);
	}

	platform_set_drvdata(pdev, NULL);
	platform_device_unregister(&glamo_fb_dev);
	platform_device_unregister(&glamo_mmc_dev);
	iounmap(glamo->base);
	release_mem_region(glamo->mem->start, GLAMO_REGOFS_VIDCAP);
	glamo_handle = NULL;
	kfree(glamo);

	return 0;
}

#ifdef CONFIG_PM

/* have to export this because struct glamo_core is opaque */

void glamo_register_resume_dependency(struct resume_dependency *
							      resume_dependency)
{
	register_resume_dependency(&glamo_handle->resume_dependency,
							     resume_dependency);
	if (glamo_handle->is_suspended)
		activate_all_resume_dependencies(
					      &glamo_handle->resume_dependency);
}
EXPORT_SYMBOL_GPL(glamo_register_resume_dependency);


static int glamo_suspend(struct platform_device *pdev, pm_message_t state)
{
	glamo_power(glamo_handle, GLAMO_POWER_SUSPEND);
	glamo_handle->is_suspended = 1;
	activate_all_resume_dependencies(&glamo_handle->resume_dependency);
	return 0;
}

static int glamo_resume(struct platform_device *pdev)
{
	glamo_power(glamo_handle, GLAMO_POWER_ON);
	glamo_handle->is_suspended = 0;
	callback_all_resume_dependencies(&glamo_handle->resume_dependency);

	return 0;
}
#else
#define glamo_suspend NULL
#define glamo_resume  NULL
#endif

static struct platform_driver glamo_driver = {
	.probe		= glamo_probe,
	.remove		= glamo_remove,
	.suspend_late	= glamo_suspend,
	.resume_early	= glamo_resume,
	.driver		= {
		.name	= "glamo3362",
		.owner	= THIS_MODULE,
	},
};

static int __devinit glamo_init(void)
{
	return platform_driver_register(&glamo_driver);
}

static void __exit glamo_cleanup(void)
{
	platform_driver_unregister(&glamo_driver);
}

module_init(glamo_init);
module_exit(glamo_cleanup);

MODULE_AUTHOR("Harald Welte <laforge@openmoko.org>");
MODULE_DESCRIPTION("Smedia Glamo 336x/337x core/resource driver");
MODULE_LICENSE("GPL");
