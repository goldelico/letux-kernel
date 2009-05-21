/*
*  multiproc.c
*
*  Many multi-processor modules have the concept of processor id. MultiProc
*  centeralizes the processor id management.
*
*  Copyright (C) 2008-2009 Texas Instruments, Inc.
*
*  This package is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License version 2 as
*  published by the Free Software Foundation.
*
*  THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
*  IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
*  WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR
*  PURPOSE.
*/

/*
 *  ======== multiproc.c ========
 *  Notes:
 *  The processor id start at 0 and ascend without skipping values till maximum_
 *  no_of_processors - 1
 */

#include <linux/types.h>
#include <linux/string.h>
#include <linux/errno.h>

#include <multiproc.h>

#define MULTIPROC_MAXPROCESSORS   4
/*
 *  Local processor's id. It has to be set before any module init.
 */

struct multiproc_module_object {
	u16 local_id;
};

/*  TDO:Add these back to the StateObject and configure them during Driver
  *  bootup using a Module_Init function.
 */
static char modena[32] = "MPU";
static char tesla[32] = "Tesla";
static char sysm3[32] = "SysM3";
static char appm3[32] = "AppM3";

static struct multiproc_module_object multiproc_local_state
				= { MULTIPROC_INVALIDID };
static struct multiproc_module_object *module = &multiproc_local_state;
static char *multiproc_namelist[] = { modena, tesla, sysm3, appm3 };



/*
 * ======== multiProc_set_local_id ========
 *  Purpose:
 *  This will set the processor id of local processor on run time
 */
int multiproc_set_local_id(u16 proc_id)
{
	int status = 0;

	if (proc_id >= MULTIPROC_MAXPROCESSORS)
		status = -EINVAL;
	else
		module->local_id = proc_id;

	return status;
}

/*
 * ======== multiProc_set_local_id ========
 *  Purpose:
 *  This will get the processor id from proccessor name
 */
u16 multiproc_get_id(const char *proc_name)
{
	s32 i;
	u16  proc_id = MULTIPROC_INVALIDID;

	/* If the name is NULL, just return the local id */
	if (proc_name == NULL) {
		proc_id = module->local_id;
	} else {
		for (i = 0; i < MULTIPROC_MAXPROCESSORS; i++) {
			if ((multiproc_namelist[i] != NULL) &&
					(strcmp(proc_name,
					multiproc_namelist[i]) == 0)) {
				proc_id = i;
				break;
			}
		}
	}
	return proc_id;
}

/*
 * ======== multiProc_set_local_id ========
 *  Purpose:
 *  This will get the processor name from proccessor id
 */
char *multiproc_get_name(u16 proc_id)
{
	char *proc_name = NULL;

	if (proc_id >= MULTIPROC_MAXPROCESSORS)
		goto end;

	proc_name = multiproc_namelist[proc_id];

end:
	return proc_name;
}

/*
 * ======== multiProc_set_local_id ========
 *  Purpose:
 *  This will get the maximum proccessor id in the system
 */
u16 multiproc_get_max_processors(void)
{
	u16 proc_id = MULTIPROC_MAXPROCESSORS;
	return proc_id;
}

