/*
 *  heap.c
 *
 *  Heap module manages fixed size buffers that can be used
 *  in a multiprocessor system with shared memory
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
#include <linux/types.h>
#include <linux/bug.h>


#include <heap.h>


/*
 * ======== heap_alloc ========
 *  Purpose:
 *  This will allocate a block of memory of specified
 *  size
 */
void *heap_alloc(void *hphandle, u32 size, u32 align)
{
	char *block = NULL;
	struct heap_object *obj = NULL;

	BUG_ON(hphandle == NULL);

	obj = (struct heap_object *)hphandle;
	BUG_ON(obj->alloc == NULL);
	block = obj->alloc(hphandle, size, align);
	return block;
}

/*
 * ======== heap_free ========
 *  Purpose:
 *  This will frees a block of memory allocated
 *  rom heap
 */
int heap_free(void *hphandle, void *block, u32 size)
{
	struct heap_object *obj = NULL;
	s32 retval  = 0;

	BUG_ON(hphandle == NULL);

	obj = (struct heap_object *)hphandle;
	BUG_ON(obj->free == NULL);
	retval = obj->free(hphandle, block, size);
	return retval;
}

/*
 * ======== heap_get_stats ========
 *  Purpose:
 *  This will get the heap memory statistics
 */
int heap_get_stats(void *hphandle, struct memory_stats *stats)
{
	struct heap_object *obj = NULL;
	s32 retval  = 0;

	BUG_ON(hphandle == NULL);
	BUG_ON(stats == NULL);

	obj = (struct heap_object *)hphandle;
	BUG_ON(obj->get_stats == NULL);
	retval = obj->get_stats(hphandle, stats);
	return retval;
}

/*
 * ======== heap_get_extended_stats ========
 *  Purpose:
 *  This will get the heap memory extended statistics
 */
int heap_get_extended_stats(void *hphandle,
				struct heap_extended_stats *stats)
{
	struct heap_object *obj = NULL;
	s32 retval  = 0;

	BUG_ON(hphandle == NULL);
	BUG_ON(stats == NULL);

	obj = (struct heap_object *)hphandle;
	BUG_ON(obj->get_extended_stats == NULL);
	retval = obj->get_extended_stats(hphandle, stats);
	return retval;
}

