#ifndef _LINUX_FIQ_C_ISR_H
#define _LINUX_FIQ_C_ISR_H

#include <asm/arch-s3c2410/regs-irq.h>

extern unsigned long _fiq_count_fiqs;
extern u32 _fiq_ack_mask;
extern int _fiq_timer_index;
extern u16 _fiq_timer_divisor;

/* This CANNOT be implemented in a module -- it has to be used in code
 * included in the monolithic kernel
 */

#define FIQ_HANDLER_START() \
void __attribute__ ((naked)) s3c2440_fiq_isr(void) \
{\
	/*\
	 * you can declare local vars here, take care to set the frame size\
	 *  below accordingly if there are more than a few dozen bytes of them\
	 */\

/* stick your locals here :-)
 * Do NOT initialize them here!  define them and initialize them after
 * FIQ_HANDLER_ENTRY() is done.
 */

#define FIQ_HANDLER_ENTRY(LOCALS, FRAME) \
	const int _FIQ_FRAME_SIZE = FRAME; \
	/* entry takes care to store registers we will be treading on here */\
	asm __volatile__ (\
		"mov     ip, sp ;"\
		/* stash FIQ and r0-r8 normal regs */\
		"stmdb	sp!, {r0-r12, lr};"\
		/* allow SP to get some space */\
		"sub     sp, sp, %1 ;"\
		/* !! THIS SETS THE FRAME, adjust to > sizeof locals */\
		"sub     fp, sp, %0 ;"\
		:\
		: "rI" (LOCALS), "rI" (FRAME)\
		:"r9"\
	);

/* stick your ISR code here and then end with... */

#define FIQ_HANDLER_END() \
	_fiq_count_fiqs++;\
	__raw_writel(_fiq_ack_mask, S3C2410_SRCPND);\
\
	/* exit back to normal mode restoring everything */\
	asm __volatile__ (\
		/* pop our allocation */\
		"add     sp, sp, %0 ;"\
		/* return FIQ regs back to pristine state\
		 * and get normal regs back\
		 */\
		"ldmia	sp!, {r0-r12, lr};"\
\
		/* return */\
		"subs	pc, lr, #4;"\
		: \
		: "rI" (_FIQ_FRAME_SIZE) \
	);\
}

#endif /* _LINUX_FIQ_C_ISR_H */
