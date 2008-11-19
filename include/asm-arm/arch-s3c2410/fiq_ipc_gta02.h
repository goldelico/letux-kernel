#ifndef _LINUX_FIQ_IPC_H
#define _LINUX_FIQ_IPC_H

/*
 * this defines the struct which is used to communicate between the FIQ
 * world and the normal linux kernel world.  One of these structs is
 * statically defined for you in the monolithic kernel so the FIQ ISR code
 * can safely touch it any any time.
 *
 * You also want to include this file in your kernel module that wants to
 * communicate with your FIQ code.  Add any kinds of vars that are used by
 * the FIQ ISR and the module in here.
 *
 * To get you started there is just an int that is incremented every FIQ
 * you can remove this when you are ready to customize, but it is useful
 * for testing
 */

struct fiq_ipc {
	u8 u8a[0]; /* placeholder */
};

/* actual definition lives in arch/arm/mach-s3c2440/fiq_c_isr.c */
extern struct fiq_ipc fiq_ipc;
extern unsigned long _fiq_count_fiqs;
extern void fiq_kick(void);  /* provoke a FIQ "immediately" */

#endif /* _LINUX_FIQ_IPC_H */
