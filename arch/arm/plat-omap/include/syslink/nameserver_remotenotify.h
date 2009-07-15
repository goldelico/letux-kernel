/*
 *  nameserver_remotenotify.h
 *
 *  The nameserver_remotenotify module provides functionality to get name
 *  value pair from a remote nameserver.
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

#ifndef _NAMESERVER_REMOTENOTIFY_H_
#define _NAMESERVER_REMOTENOTIFY_H_

#include <linux/types.h>

/*
 *  NAMESERVERREMOTENOTIFY_MODULEID
 *  Unique module ID
 */
#define NAMESERVERREMOTENOTIFY_MODULEID      (0x08FD)

/*
 *  Module configuration structure
 */
struct nameserver_remotenotify_config {
	u32 reserved;
	/* Reserved value (not currently used) */
};

/*
  * Module configuration structure
 */
struct nameserver_remotenotify_params {
	u32 notify_event_no; /* Notify event number */
	void *notify_driver; /* Notify Driver handle */
	void *shared_addr; /* Address of the shared memory */
	u32 shared_addr_size; /* Size of the shared memory */
	void *gate; /* Handle to the gate used for protecting
			nameserver add and delete */
};

/*
 *  Function to get the default configuration for the nameserver_remotenotify
 *  module
 */
void nameserver_remotenotify_get_config(
				struct nameserver_remotenotify_config *cfg);

/*
 *  Function to setup the nameserver_remotenotify module
 */
int nameserver_remotenotify_setup(struct nameserver_remotenotify_config *cfg);

/*
 *  Function to destroy the nameserver_remotenotify module
 */
int nameserver_remotenotify_destroy(void);

/*
 *  Function to get the current configuration values
 */
void nameserver_remotenotify_params_init(void *handle,
			struct nameserver_remotenotify_params *params);

/*
 *  Function to setup the Name Server remote notify
 */
void *nameserver_remotenotify_create(u16 proc_id,
			const struct nameserver_remotenotify_params *params);

/*
 *  Function to destroy the Name Server remote notify
 */
int nameserver_remotenotify_delete(void **handle);


/*
 *  Function to get a name/value from remote nameserver
 */
int nameserver_remotenotify_get(void *handle,
				const char *instance_name, const char *name,
				void *value, u32 value_len, void *reserved);

/*
 *  Get the shared memory requirements for the nameserver_remotenotify
 */
u32 nameserver_remotenotify_shared_memreq(
			const struct nameserver_remotenotify_params *params);


#endif /* _NAMESERVER_REMOTENOTIFY_H_ */

