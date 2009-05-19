/*
 *  gatepeterson.h
 *
 *  The Gate Peterson Algorithm for mutual exclusion of shared memory.
 *  Current implementation works for 2 processors.
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

#ifndef _GATEPETERSON_H_
#define _GATEPETERSON_H_

#include <linux/types.h>

/*
 *  A set of context protection levels that each correspond to
 *  single processor gates used for local protection
 */
enum gatepeterson_protect {
	GATEPETERSON_PROTECT_DEFAULT,
	GATEPETERSON_PROTECT_INTERRUPT,
	GATEPETERSON_PROTECT_TASKLET,
	GATEPETERSON_PROTECT_THREAD,
	GATEPETERSON_PROTECT_PROCESS,
	GATEPETERSON_PROTECT_NONE
};

/*
 *  Structure defining config parameters for the Gate Peterson
 *  module
 */
struct gatepeterson_config {
	u32 max_name_len; /* GP name len */
	enum gatepeterson_protect default_protection;
	bool use_nameserver; /* Need a nameserver or not */
	u32 max_runtime_entries; /* No of dynamic gps */
	void *name_table_gate; /* for nameserver */
};

/*
 *  Structure defining config parameters for the Gate Peterson
 *  instances
 */
struct gatepeterson_params {
	void *shared_addr;
	u32 shared_addr_size;
	char *name;
	u16 opener_proc_id;
	enum gatepeterson_protect local_protection;
};

/*
 *  Function to initialize the parameter structure
 */
int gatepeterson_get_config(struct gatepeterson_config *config);

/*
 *  Function to initialize GP module
 */
int gatepeterson_setup(const struct gatepeterson_config *config);

/*
 *  Function to destroy the GP module
 */
int gatepeterson_destroy(void);

/*
 *  Function to initialize the parameter structure
 */
int gatepeterson_params_init(struct gatepeterson_params *params);

/*
 *  Function to create an instance of GatePeterson
 */
void *gatepeterson_create(const struct gatepeterson_params *params);

/*
 *  Function to delete an instance of GatePeterson
 */
int gatepeterson_delete(void **gphandle);

/*
 *  Function to open a previously created instance
 */
int gatepeterson_open(void **gphandle,
			const struct gatepeterson_params *params);

/*
 *  Function to close a previously opened instance
 */
int gatepeterson_close(void **gphandle);

/*
 * Function to enter the gate peterson
 */
u32 gatepeterson_enter(void *gphandle);

/*
 *Function to leave the gate peterson
 */
void gatepeterson_leave(void *gphandle, u32 flag);


/*
 *  Returns the gatepeterson kernel object pointer
 */
void *gatepeterson_get_knl_handle(void **gpHandle);

/*
 * Function to return the shared memory requirement
 */
u32 gatepeterson_shared_memreq(const struct gatepeterson_params *params);

#endif /* _GATEPETERSON_H_ */

