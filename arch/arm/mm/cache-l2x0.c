/*
 * arch/arm/mm/cache-l2x0.c - L210/L220 cache controller support
 *
 * Copyright (C) 2007 ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/io.h>

#include <asm/cacheflush.h>
#include <asm/hardware/cache-l2x0.h>

#define CACHE_LINE_SIZE		32

static void __iomem *l2x0_base;
static DEFINE_SPINLOCK(l2x0_lock);
bool l2x0_disabled;

#ifdef CONFIG_PL310_ERRATA_588369
static void debug_writel(unsigned long val)
{
	register unsigned long r0 asm("r0") = val;
	/*
	 * Texas Instrument secure monitor api to modify the PL310
	 * Debug Control Register.
	 */
	__asm__ __volatile__(
		__asmeq("%0", "r0")
		"ldr r12, =0x100\n"
		"dsb\n"
		"smc\n"
		: : "r" (r0)
		: "r4", "r5", "r6", "r7", "r8");
}
#endif

static inline void cache_wait(void __iomem *reg, unsigned long mask)
{
	/* wait for the operation to complete */
	while (readl(reg) & mask)
		;
}

static inline void cache_sync(void)
{
	void __iomem *base = l2x0_base;
	writel(0, base + L2X0_CACHE_SYNC);
	cache_wait(base + L2X0_CACHE_SYNC, 1);
}

static inline void l2x0_inv_all(void)
{
	unsigned long flags;

	/* invalidate all ways */
	spin_lock_irqsave(&l2x0_lock, flags);
	writel(0xff, l2x0_base + L2X0_INV_WAY);
	cache_wait(l2x0_base + L2X0_INV_WAY, 0xff);
	cache_sync();
	spin_unlock_irqrestore(&l2x0_lock, flags);
}

static void l2x0_inv_range(unsigned long start, unsigned long end)
{
	void __iomem *base = l2x0_base;
	unsigned long flags;

	spin_lock_irqsave(&l2x0_lock, flags);
	if (start & (CACHE_LINE_SIZE - 1)) {
		start &= ~(CACHE_LINE_SIZE - 1);
#ifdef CONFIG_PL310_ERRATA_588369
		debug_writel(0x03);
		cache_wait(base + L2X0_CLEAN_LINE_PA, 1);
		writel(start, base + L2X0_CLEAN_LINE_PA);
		cache_wait(base + L2X0_INV_LINE_PA, 1);
		writel(start, base + L2X0_INV_LINE_PA);
		debug_writel(0x00);
#else
		cache_wait(base + L2X0_CLEAN_INV_LINE_PA, 1);
		writel(start, base + L2X0_CLEAN_INV_LINE_PA);
#endif
		start += CACHE_LINE_SIZE;
	}

	if (end & (CACHE_LINE_SIZE - 1)) {
		end &= ~(CACHE_LINE_SIZE - 1);
#ifdef CONFIG_PL310_ERRATA_588369
		debug_writel(0x03);
		cache_wait(base + L2X0_CLEAN_LINE_PA, 1);
		writel(end, base + L2X0_CLEAN_LINE_PA);
		cache_wait(base + L2X0_INV_LINE_PA, 1);
		writel(end, base + L2X0_INV_LINE_PA);
			debug_writel(0x00);
#else
		cache_wait(base + L2X0_CLEAN_INV_LINE_PA, 1);
		writel(end, base + L2X0_CLEAN_INV_LINE_PA);
#endif
	}

	while (start < end) {
		unsigned long blk_end = start + min(end - start, 4096UL);

		while (start < blk_end) {
			cache_wait(base + L2X0_INV_LINE_PA, 1);
			writel(start, base + L2X0_INV_LINE_PA);
			start += CACHE_LINE_SIZE;
		}

		if (blk_end < end) {
			spin_unlock_irqrestore(&l2x0_lock, flags);
			spin_lock_irqsave(&l2x0_lock, flags);
		}
	}
	cache_wait(base + L2X0_INV_LINE_PA, 1);
	cache_sync();
	spin_unlock_irqrestore(&l2x0_lock, flags);
}

static void l2x0_clean_range(unsigned long start, unsigned long end)
{
	void __iomem *base = l2x0_base;
	unsigned long flags;

	spin_lock_irqsave(&l2x0_lock, flags);
	start &= ~(CACHE_LINE_SIZE - 1);
	while (start < end) {
		unsigned long blk_end = start + min(end - start, 4096UL);

		while (start < blk_end) {
			cache_wait(base + L2X0_CLEAN_LINE_PA, 1);
			writel(start, base + L2X0_CLEAN_LINE_PA);
			start += CACHE_LINE_SIZE;
		}

		if (blk_end < end) {
			spin_unlock_irqrestore(&l2x0_lock, flags);
			spin_lock_irqsave(&l2x0_lock, flags);
		}
	}
	cache_wait(base + L2X0_CLEAN_LINE_PA, 1);
	cache_sync();
	spin_unlock_irqrestore(&l2x0_lock, flags);
}

static void l2x0_flush_range(unsigned long start, unsigned long end)
{
	void __iomem *base = l2x0_base;
	unsigned long flags;

	spin_lock_irqsave(&l2x0_lock, flags);
	start &= ~(CACHE_LINE_SIZE - 1);
	while (start < end) {
		unsigned long blk_end = start + min(end - start, 4096UL);

		while (start < blk_end) {
#ifdef CONFIG_PL310_ERRATA_588369
			debug_writel(0x03);
			/* Clean by PA followed by Invalidate by PA */
			cache_wait(base + L2X0_CLEAN_LINE_PA, 1);
			writel(start, base + L2X0_CLEAN_LINE_PA);
			cache_wait(base + L2X0_INV_LINE_PA, 1);
			writel(start, base + L2X0_INV_LINE_PA);
#else
			cache_wait(base + L2X0_CLEAN_INV_LINE_PA, 1);
			writel(start, base + L2X0_CLEAN_INV_LINE_PA);
#endif
			start += CACHE_LINE_SIZE;
		}

		if (blk_end < end) {
			spin_unlock_irqrestore(&l2x0_lock, flags);
			spin_lock_irqsave(&l2x0_lock, flags);
		}
	}
#ifdef CONFIG_PL310_ERRATA_588369
	cache_wait(base + L2X0_CLEAN_LINE_PA, 1);
	cache_wait(base + L2X0_INV_LINE_PA, 1);
#else
	cache_wait(base + L2X0_CLEAN_INV_LINE_PA, 1);
#endif
	cache_sync();
	spin_unlock_irqrestore(&l2x0_lock, flags);
}

void __init l2x0_init(void __iomem *base, __u32 aux_val, __u32 aux_mask)
{
	__u32 aux;

	if (l2x0_disabled) {
		printk(KERN_INFO "L2X0 cache controller disabled\n");
		return;
	}

	l2x0_base = base;

	/* disable L2X0 */
	writel(0, l2x0_base + L2X0_CTRL);

	aux = readl(l2x0_base + L2X0_AUX_CTRL);
	aux &= aux_mask;
	aux |= aux_val;
	writel(aux, l2x0_base + L2X0_AUX_CTRL);

	l2x0_inv_all();

	/* enable L2X0 */
	writel(1, l2x0_base + L2X0_CTRL);

	outer_cache.inv_range = l2x0_inv_range;
	outer_cache.clean_range = l2x0_clean_range;
	outer_cache.flush_range = l2x0_flush_range;

	printk(KERN_INFO "L2X0 cache controller enabled\n");
}

static int __init l2x0_disable(char *unused)
{
	l2x0_disabled = 1;
	return 0;
}
early_param("nol2x0", l2x0_disable);
