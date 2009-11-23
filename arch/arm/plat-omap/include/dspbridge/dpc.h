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

/* The DPC object, passed to our priority event callback routine: */
struct DPC_OBJECT {
	u32 dwSignature;	/* Used for object validation.   */
	void *pRefData;		/* Argument for client's DPC.    */
	u32 numRequested;	/* Number of requested DPC's.      */
	u32 numScheduled;	/* Number of executed DPC's.      */
	struct tasklet_struct dpc_tasklet;

#ifdef DEBUG
	u32 cEntryCount;	/* Number of times DPC reentered. */
	u32 numRequestedMax;	/* Keep track of max pending DPC's. */
#endif

	spinlock_t dpc_lock;
};

#endif				/* DPC_ */
