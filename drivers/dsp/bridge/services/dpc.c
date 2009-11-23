/*
 * dpc.c
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

/*  ----------------------------------- Host OS */
#include <dspbridge/host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <dspbridge/std.h>
#include <dspbridge/dbdefs.h>
#include <dspbridge/errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <dspbridge/dbc.h>
#include <dspbridge/gt.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/mem.h>

/*  ----------------------------------- This */
#include <dspbridge/dpc.h>

/*  ----------------------------------- Defines, Data Structures, Typedefs */
#define SIGNATURE       0x5f435044	/* "DPC_" (in reverse). */

/*  ----------------------------------- Globals */
#if GT_TRACE
static struct GT_Mask DPC_DebugMask = { NULL, NULL };	/* DPC Debug Mask */
#endif

/*
 *  ======== DPC_Exit ========
 *  Purpose:
 *      Discontinue usage of the DPC module.
 */
void DPC_Exit(void)
{
	GT_0trace(DPC_DebugMask, GT_5CLASS, "Entered DPC_Exit\n");
}

/*
 *  ======== DPC_Init ========
 *  Purpose:
 *      Initialize the DPC module's private state.
 */
bool DPC_Init(void)
{
	GT_create(&DPC_DebugMask, "DP");

	GT_0trace(DPC_DebugMask, GT_5CLASS, "Entered DPC_Init\n");

	return true;
}

