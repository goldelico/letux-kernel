/*
 * dpc.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Deferred Procedure Call(DPC) Services.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef DPC_
#define DPC_

/*
 *  ======== DPC_PROC ========
 *  Purpose:
 *      Deferred processing routine.  Typically scheduled from an ISR to
 *      complete I/O processing.
 *  Parameters:
 *      pRefData:   Ptr to user data: passed in via ISR_ScheduleDPC.
 *  Returns:
 *  Requires:
 *      The DPC should not block, or otherwise acquire resources.
 *      Interrupts to the processor are enabled.
 *      DPC_PROC executes in a critical section.
 *  Ensures:
 *      This DPC will not be reenterred on the same thread.
 *      However, the DPC may take hardware interrupts during execution.
 *      Interrupts to the processor are enabled.
 */
       typedef void(*DPC_PROC) (void *pRefData);

/* The DPC object, passed to our priority event callback routine: */
struct DPC_OBJECT {
	u32 dwSignature;	/* Used for object validation.   */
	void *pRefData;		/* Argument for client's DPC.    */
	DPC_PROC pfnDPC;	/* Client's DPC.                 */
	u32 numRequested;	/* Number of requested DPC's.      */
	u32 numScheduled;	/* Number of executed DPC's.      */
	struct tasklet_struct dpc_tasklet;

#ifdef DEBUG
	u32 cEntryCount;	/* Number of times DPC reentered. */
	u32 numRequestedMax;	/* Keep track of max pending DPC's. */
#endif

	spinlock_t dpc_lock;
};

/*
 *  ======== DPC_Exit ========
 *  Purpose:
 *      Discontinue usage of the DPC module.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      DPC_Init(void) was previously called.
 *  Ensures:
 *      Resources acquired in DPC_Init(void) are freed.
 */
       extern void DPC_Exit(void);

/*
 *  ======== DPC_Init ========
 *  Purpose:
 *      Initialize the DPC module's private state.
 *  Parameters:
 *  Returns:
 *      TRUE if initialized; FALSE if error occured.
 *  Requires:
 *  Ensures:
 *      A requirement for each of the other public DPC functions.
 */
       extern bool DPC_Init(void);

/*  ----------------------------------- Function Prototypes */
 void DPC_DeferredProcedure(IN unsigned long pDeferredContext);

#endif				/* DPC_ */
