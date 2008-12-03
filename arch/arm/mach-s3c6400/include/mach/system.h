/* linux/arch/arm/mach-s3c6400/include/mach/system.h
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * S3C6400 - system implementation
 */

#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H __FILE__

#include <linux/io.h>
#include <mach/map.h>

#include <plat/regs-sys.h>
#include <plat/regs-syscon-power.h>

static void arch_idle(void)
{
	unsigned long flags;
	u32 mode;

	/* ensure that if we execute the cpu idle sequence that we
	 * go into idle mode instead of powering off. */

	local_irq_save(flags);
	mode = __raw_readl(S3C64XX_PWR_CFG);
	mode &= ~S3C64XX_PWRCFG_CFG_WFI_MASK;
	mode |= S3C64XX_PWRCFG_CFG_WFI_IDLE;
	__raw_writel(mode, S3C64XX_PWR_CFG);

	local_irq_restore(flags);

	cpu_do_idle();
}

static void arch_reset(char mode)
{
	/* nothing here yet */
}

#endif /* __ASM_ARCH_IRQ_H */
