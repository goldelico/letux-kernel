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
 *  GATEPETERSON_MODULEID
 *  Unique module ID
 */
#define GATEPETERSON_MODULEID       (0xF415)

/*
 *  A set of context protection levels that each correspond to
 *  single processor gates used for local protection
 */
enum gatepeterson_protect {
	GATEPETERSON_PROTECT_DEFAULT    = 0,
	GATEPETERSON_PROTECT_NONE       = 1,
	GATEPETERSON_PROTECT_INTERRUPT  = 2,
	GATEPETERSON_PROTECT_TASKLET    = 3,
	GATEPETERSON_PROTECT_THREAD     = 4,
	GATEPETERSON_PROTECT_PROCESS    = 5,
	GATEPETERSON_PROTECT_END_VALUE	= 6
};

/*
 *  Structure defining config parameters for the Gate Peterson
 *  module
 */
struct gatepeterson_config {
	enum gatepeterson_protect default_protection;
	/*!< Default module-wide local context protection level. The level of
	* protection specified here determines which local gate is created per
	* GatePeterson instance for local protection during create. The instance
	* configuration parameter may be usedto override this module setting per
	* instance.  The configuration used here should reflect both the context
	* in which enter and leave are to be called,as well as the maximum level
	* of protection needed locally.
	*/
	u32 max_name_len; /* GP name len */
	bool use_nameserver;
	/*!< Whether to have this module use the NameServer or not. If the
	*   NameServer is not needed, set this configuration parameter to false.
	*   This informs GatePeterson not to pull in the NameServer module.
	*   In this case, all names passed into create and open are ignored.
	*/
};

/*
 *  Structure defining config parameters for the Gate Peterson
 *  instances
 */
struct gatepeterson_params {
	void *shared_addr;
	/* Address of the shared memory. The creator must supply a cache-aligned
	*   address in shared memory that will be used to store shared state
	*   information.
	*/

	u32 shared_addr_size;
    /* Size of the shared memory region. Can use gatepeterson_shared_memreq
	*   call to determine the required size.
	*/

	char *name;
    /* If using nameserver, name of this instance. The name (if not NULL) must
	*   be unique among all gatepeterson instances in the entire system.
	*/

	enum gatepeterson_protect local_protection;
	/* Local gate protection level. The default value, (Protect_DEFAULT)
	*   results in inheritance from module-level defaultProtection. This
	*   instance setting should be set to an alternative only if a different
	*   local protection level is needed for the instance.
	*/
	bool use_nameserver;
	/* Whether to have this module use the nameserver or not. If the
	*  nameserver is not needed, set this configuration parameter to
	*  false.This informs gatepeterson not to pull in the nameaerver
	*  module. In this case, all names passed into create and open are
	*  ignored.
	*/
};

/*
 *  Function to initialize the parameter structure
 */
void gatepeterson_get_config(struct gatepeterson_config *config);

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
void gatepeterson_params_init(void *handle,
				struct gatepeterson_params *params);

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
					struct gatepeterson_params *params);

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

