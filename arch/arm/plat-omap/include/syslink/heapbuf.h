/*
 *  heapbuf.h
 *
 *  Heap module manages fixed size buffers that can be used
 *  in a multiprocessor system with shared memory.
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

#ifndef _HEAPBUF_H_
#define _HEAPBUF_H_

#include <linux/types.h>
#include <heap.h>
#include <listmp.h>

/*!
 *  @def	LISTMP_MODULEID
 *  @brief  Unique module ID.
 */
#define HEAPBUF_MODULEID	(0x4cd5)

/*
 *  Creation of Heap Buf succesful.
*/
#define HEAPBUF_CREATED            (0x05251995)

/*
 *  Version.
 */
#define HEAPBUF_VERSION            (1)

/*
 *  Structure defining config parameters for the HeapBuf module.
 */
struct heapbuf_config {
    u32 max_name_len; /* Maximum length of name */
    bool use_nameserver; /* To have this module use the NameServer or not */
    bool track_max_allocs; /* Track the maximum number of allocated blocks */
};

/*
 *  Structure defining parameters for the HeapBuf module
 */
struct heapbuf_params {
	void *gate;
	bool exact; /* Only allocate on exact match of rquested size */
	char *name; /* Name when using nameserver */
	int resource_id; /* Resource id of the hardware linked list */
	bool cache_flag; /* Whether to perform cache coherency calls */
	u32 align; /* Alignment (in MAUs, power of 2) of each block */
	u32 num_blocks; /* Number of fixed-size blocks */
	u32 block_size; /* Size (in MAUs) of each block*/
	void *shared_addr; /* Physical address of the shared memory */
	u32 shared_addr_size; /* Size of shareAddr  */
	void *shared_buf; /* Physical address of the shared buffers */
	u32 shared_buf_size; /* Size of sharedBuf */
};

/*
 *  Stats structure for the getExtendedStats API.
 */
struct heapbuf_extended_stats {
    u32 max_allocated_blocks;
    /* maximum number of blocks allocated from this heap instance */
    u32 num_allocated_blocks;
    /* total number of blocks currently allocated from this heap instance*/
};


/*
 *  Function to get default configuration for the heapbuf module
 */
int heapbuf_get_config(struct heapbuf_config *cfgparams);

/*
 *  Function to setup the heapbuf module
 */
int heapbuf_setup(const struct heapbuf_config *cfg);

/*
 *  Function to destroy the heapbuf module
 */
int heapbuf_destroy(void);

/* Initialize this config-params structure with supplier-specified
 *  defaults before instance creation
 */
void heapbuf_params_init(void *handle, struct heapbuf_params *params);

/*
 *  Creates a new instance of heapbuf module
 */
void *heapbuf_create(const struct heapbuf_params *params);

/*
 * Deletes a instance of heapbuf module
 */
int heapbuf_delete(void **handle_ptr);

/*
 *  Opens a created instance of heapbuf module
 */
int heapbuf_open(void **handle_ptr, struct heapbuf_params *params);

/*
 *  Closes previously opened/created instance of heapbuf module
 */
int heapbuf_close(void *handle_ptr);

/*
 *  Returns the amount of shared memory required for creation
 *  of each instance
 */
int heapbuf_shared_memreq(const struct heapbuf_params *params, u32 *buf_size);

/*
 *  Allocate a block
 */
void *heapbuf_alloc(void *hphandle, u32 size, u32 align);

/*
 *  Frees the block to this heapbuf
 */
int heapbuf_free(void *hphandle, void *block, u32 size);

/*
 *  Get memory statistics
 */
int heapbuf_get_stats(void *hphandle, struct memory_stats *stats);

/*
 *  Indicate whether the heap may block during an alloc or free call
 */
bool heapbuf_isblocking(void *handle);

/*
 *  Get extended statistics
 */
int heapbuf_get_extended_stats(void *hphandle,
			       struct heapbuf_extended_stats *stats);

#endif /* _HEAPBUF_H_ */
