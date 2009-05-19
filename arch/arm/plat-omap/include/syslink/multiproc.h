/*
* multiproc.h
*
* Many multi-processor modules have the concept of processor id. MultiProc
* centeralizes the processor id management.
*
* Copyright (C) 2008-2009 Texas Instruments, Inc.
*
* This package is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
* WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR
* PURPOSE.
*/

#ifndef _MULTIPROC_H_
#define _MULTIPROC_H_

#include <linux/types.h>

/*
 * Macro to define invalid processor id
*/
#define MULTIPROC_INVALIDID ((u16)0xFFFF)

/*
 *  Maximum number of processors in the system
 *  OMAP4 has 4 processors in single core.
 */
#define MULTIPROC_MAXPROCESSORS 4

bool multiproc_set_local_id(u16 proc_id);
u16  multiproc_get_id(const char *proc_name);
char *multiproc_get_name(u16 proc_id);
u16 multiproc_get_max_processors(void);

#endif	/* _MULTIPROC_H_ */

