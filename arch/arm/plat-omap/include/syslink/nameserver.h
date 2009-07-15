/*
 *  nameserver.h
 *
 *  The nameserver module manages local name/value pairs that
 *  enables an application and other modules to store and retrieve
 *  values based on a name.
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

#ifndef _NAMESERVER_H_
#define _NAMESERVER_H_

#include <linux/types.h>
#include <linux/list.h>

/*
 *  NAMESERVER_MODULEID
 *  Unique module ID
 */
#define NAMESERVER_MODULEID      (0xF414)

/*
 *  Instance config-params object.
 */
struct nameserver_params {
	bool check_existing; /* Prevents duplicate entry add in to the table */
	void *gate_handle; /* Lock used for critical region of the table */
	u16 max_name_len; /* Length, in MAUs, of name field */
	u32 max_runtime_entries;
	u32 max_value_len; /* Length, in MAUs, of the value field */
	void *table_heap; /* Table is placed into a section on dyn creates */
};

/*
 *  Function to setup the nameserver module
 */
int nameserver_setup(void);

/*
 *  Function to destroy the nameserver module
 */
int nameserver_destroy(void);

/*
 *  Function to initialize the parameter structure
 */
int nameserver_params_init(struct nameserver_params *params);

/*
 *  Function to initialize the parameter structure
 */
int nameserver_get_params(void *handle,
			struct nameserver_params *params);

/*
 *  Function to create a name server
 */
void *nameserver_create(const char *name,
			const struct nameserver_params *params);

/*
 *  Function to delete a name server
 */
int nameserver_delete(void **handle);

/*
 *  Function to add a variable length value into the local table
 */
void *nameserver_add(void *handle, const char *name,
			void  *buffer, u32 length);

/*
 *  Function to add a 32 bit value into the local table
 */
void *nameserver_add_uint32(void *handle,
			const char *name, u32 value);

/*
 *  Function to Retrieve the value portion of a name/value pair
 */
int nameserver_get(void *handle, const char *name,
		void *buffer, u32 length, u16 procId[]);

/*
 *  Function to get the value portion of a name/value pair from local table
 */
int nameserver_get_local(void *handle, const char *name,
			void *buffer, u32 length);

/*
 *  Function to removes a value/pair
 */
int nameserver_remove(void *handle, const char *name);

/*
 *  Function to Remove an entry from the table
 */
int nameserver_remove_entry(void *nshandle, void *nsentry);

/*
 *  Function to handle for  a name
 */
void *nameserver_get_handle(const char *name);

/*
 *  Function to Match the name
 */
int nameserver_match(void *handle, const char *name, u32 *value);

/*
 *  Function to register a remote driver
 */
int nameserver_register_remote_driver(void *handle, u16 proc_id);

/*
 *  Function to unregister a remote driver
 */
int nameserver_unregister_remote_driver(u16 proc_id);

#endif /* _NAMESERVER_H_ */

