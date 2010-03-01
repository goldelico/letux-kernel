/*
 * gt.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 * There are two definitions that affect which portions of trace
 * are acutally compiled into the client: GT_TRACE and GT_ASSERT. If
 * GT_TRACE is set to 0 then all trace statements (except for assertions)
 * will be compiled out of the client. If GT_ASSERT is set to 0 then
 * assertions will be compiled out of the client. GT_ASSERT can not be
 * set to 0 unless GT_TRACE is also set to 0 (i.e. GT_TRACE == 1 implies
 * GT_ASSERT == 1).
 */

#include <linux/types.h>
#ifndef GT_
#define GT_

#ifndef GT_TRACE
#define GT_TRACE 0	    /* 0 = "trace compiled out"; 1 = "trace active" */
#endif


#endif				/* GTCE_ */
