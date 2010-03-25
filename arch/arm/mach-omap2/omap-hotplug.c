/*
 * OMAP4 SMP cpu-hotplug support
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 * Author:
 *      Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Platform file needed for the OMAP4 SMP. This file is based on arm
 * realview smp platform.
 * Copyright (c) 2002 ARM Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/smp.h>
#include <linux/completion.h>

#include <asm/cacheflush.h>
#include <plat/io.h>

static DECLARE_COMPLETION(cpu_killed);

static inline void cpu_enter_lowpower(void)
{
	unsigned int v;

	flush_cache_all();
	/* FIXME: check L2 state and see if the l2 flush is necessary */

	asm volatile(
		"	mcr	p15, 0, %1, c7, c5, 0\n"
		"	mcr	p15, 0, %1, c7, c10, 4\n"
		/* FIXME: Need to use secure API for AUX control */

		"	mrc	p15, 0, %0, c1, c0, 0\n"
		"	bic	%0, %0, #0x04\n"
		"	mcr	p15, 0, %0, c1, c0, 0\n"
		  : "=&r" (v)
		  : "r" (0)
		  : "cc");
}

static inline void cpu_leave_lowpower(void)
{
	unsigned int v;

	asm volatile(
		"mrc	p15, 0, %0, c1, c0, 0\n"
		"	orr	%0, %0, #0x04\n"
		"	mcr	p15, 0, %0, c1, c0, 0\n"
		/* FIXME: Need to use secure API for AUX control */

		  : "=&r" (v)
		  :
		  : "cc");
}

static inline void omap_do_lowpower(unsigned int cpu)
{


	if(omap_modify_auxcoreboot0(0x0, 0x200) != 0x0) {
		printk(KERN_CRIT "Secure clear status failed\n");
		BUG();
	}

	/*
	 * FIXME: Hook up the omap low power here.
	 */
	for (;;) {
		/* Program the possible low power state here */

		/* 1. SCP power status register to dormant mode */
		omap_writel(0x02, 0x48240008);
		dsb();
		asm volatile("wfi\n"
			:
			:
			: "memory", "cc");

		if (omap_read_auxcoreboot0() == cpu) {
			/*
			 * OK, proper wakeup, we're done
			 */
			local_irq_enable();
			break;
		}
#ifdef DEBUG
		printk("CPU%u: spurious wakeup call\n", cpu);
#endif
	}
}


int platform_cpu_kill(unsigned int cpu)
{
	return wait_for_completion_timeout(&cpu_killed, 5000);
}

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */
void platform_cpu_die(unsigned int cpu)
{
#ifdef DEBUG
	unsigned int this_cpu = hard_smp_processor_id();

	if (cpu != this_cpu) {
		printk(KERN_CRIT "Eek! platform_cpu_die running on %u, should be %u\n",
			   this_cpu, cpu);
		BUG();
	}
#endif
	printk(KERN_NOTICE "CPU%u: shutdown\n", cpu);
	complete(&cpu_killed);

	/*
	 * we're ready for shutdown now, so do it
	 */
	cpu_enter_lowpower();
	omap_do_lowpower(cpu);

	/*
	 * bring this CPU back into the world of cache
	 * coherency, and then restore interrupts
	 */
	cpu_leave_lowpower();
}

int mach_cpu_disable(unsigned int cpu)
{
	/*
	 * we don't allow CPU 0 to be shutdown (it is still too special
	 * e.g. clock tick interrupts)
	 */
	return cpu == 0 ? -EPERM : 0;
}
