/*
 * linux/arch/arm/mach-omap2/sleep44xx.S
 *
 * (C) Copyright 2010
 * Texas Instruments
 * Rajendra Nayak <rnayak@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR /PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/linkage.h>
#include <asm/assembler.h>

/*
 * Forces OMAP into idle state
 *
 * omap44xx_suspend() - This bit of code just executes the WFI
 * for normal idles.
 *
 * Note: This code get's copied to internal SRAM at boot. When the OMAP
 *	 wakes up it continues execution at the point it went to sleep.
 */
ENTRY(omap44xx_cpu_suspend)
	stmfd	sp!, {r0-r12, lr}		@ save registers on stack
loop:
	/*b	loop*/	@Enable to debug by stepping through code
	/* Data memory barrier and Data sync barrier */
	/* FIXME: check whther we need this code executed from SRAM ? */
	dsb

	wfi				@ wait for interrupt

	ldmfd	sp!, {r0-r12, pc}		@ restore regs and return
ENTRY(omap44xx_cpu_suspend_sz)
	.word   . - omap44xx_cpu_suspend
