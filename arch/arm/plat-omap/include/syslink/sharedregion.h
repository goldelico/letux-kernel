/*
 *  sharedregion.h
 *
 *  The SharedRegion module is designed to be used in a
 *  multi-processor environment where there are memory regions
 *  that are shared and accessed across different processors
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

#ifndef _SHAREDREGION_H_
#define _SHAREDREGION_H_

#include <linux/types.h>

/*
 *  SHAREDREGION_MODULEID
 *  Module ID for Shared region manager
 */
#define SHAREDREGION_MODULEID          (0x5D8A)

/*
 *  Name of the reserved nameserver used for application
 */
#define SHAREDREGION_NAMESERVER        "SHAREDREGION"

/*
 *  Name of the reserved nameserver used for application
 */
#define SHAREDREGION_INVALIDSRPTR      ((u32 *)0xFFFFFFFF)


struct sharedregion_info {
	bool   is_valid; /* table entry is valid or not? */
	void *base; /* Ptr to the base address of a table entry */
	u32 len; /* The length of a table entry */
};

/*
 * Module configuration structure
 */
struct sharedregion_config {
	void *gate_handle;
	void *heap_handle;
	u32 max_regions;
};

/*
 *  Function to get the configuration
 */
int sharedregion_get_config(struct sharedregion_config *config);

/*
 *  Function to setup the SharedRegion module
 */
int sharedregion_setup(const struct sharedregion_config *config);

/*
 *  Function to destroy the SharedRegion module
 */
int sharedregion_destroy(void);

/* Fucntion to Add a memory segment to the lookup table during
 *  runtime by base and length
 */
int sharedregion_add(u32 index, void *base, u32 len);

/* Removes the memory segment at the specified index from the lookup
 *  table at runtime
 */
int sharedregion_remove(u32 index);

/*
 *  Returns the index for the specified address pointer
 */
int sharedregion_get_index(void *addr);

/*
 *  Returns the address pointer associated with the shared region pointer
 */
void *sharedregion_get_ptr(u32 *srptr);

/*
 *  Returns the shared region pointer
 */
u32 *sharedregion_get_srptr(void *addr, int index);

/*
 *  Gets the table entry information for the specified index and processor id
 */
int sharedregion_get_table_info(u32 index, u16 proc_id,
				struct sharedregion_info *info);

/*
 *  Sets the base address of the entry in the table
 */
int sharedregion_set_table_info(u32 index, u16  proc_id,
					struct sharedregion_info *info);

#endif /* _SHAREDREGION_H */

