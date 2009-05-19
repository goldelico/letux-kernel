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

/*
 *  Creation of Heap Buf succesful.
*/
#define HEAPBUF_CREATED            (0x05251995)

/*
 *  Version.
 */
#define HEAPBUF_VERSION            (1)

/*
 *  Structure defining parameters for the Heap Buf module
 */
struct heapbuf_params {
	void *lock_handle;
	bool exact; /* Only allocate on exact match of rquested size */
	char *name; /* Name when using nameserver */
	u32 align; /* Alignment (in MAUs, power of 2) of each block */
	u32 num_blocks; /* Number of fixed-size blocks */
	u32 block_size; /* Size (in MAUs) of each block*/
	void *shared_addr; /* Physical address of the shared memory */
	u32 shared_addr_size; /* Size of shareAddr  */
};

/*
 *  Function to get default configuration for the heapbuf module
 */
int heapbuf_get_config(struct heap_config *cfgparams);

/*
 *  Function to setup the heapbuf module
 */
int heapbuf_setup(const struct heap_config *config);

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
int heapbuf_delete(void **handle);

/*
 *  Opens a created instance of heapbuf module
 */
int heapbuf_open(void **handle, const struct heapbuf_params *params);

/*
 *  Closes previously opened/created instance of heapbuf module
 */
int heapbuf_close(void *handle);

/*
 *  Returns the amount of shared memory required for creation
 *  of each instance
 */
int heapbuf_shared_memreq(const struct heapbuf_params *params);

/*
 *  Allocate a block
 */
void *heapbuf_alloc(void *handle, u32 size, u32 align);

/*
 *  Frees the block to this heapbuf
 */
int heapbuf_free(void *handle, void *block, u32 size);

/*
 *  Get memory statistics
 */
int heapbuf_get_stats(void *handle, struct memory_stats *stats);

/*
 *  Get extended statistics
 */
int heapbuf_get_extended_stats(void *handle, struct heap_extended_stats *stats);
#endif /* _HEAPBUF_H_ */

