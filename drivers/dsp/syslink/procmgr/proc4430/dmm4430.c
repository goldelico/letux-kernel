/*
 * dmm4430.c
 *
 * Syslink support functions for TI OMAP processors.
 *
 * Copyright (C) 2009-2010 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 *  ======== dmm.c ========
 *  Purpose:
 *The Dynamic Memory Manager (DMM) module manages the DSP Virtual address
 *space that can be directly mapped to any MPU buffer or memory region
 *
 *  Public Functions:
 *dmm_create_tables
 *dmm_create
 *dmm_destroy
 *dmm_exit
 *dmm_init
 *dmm_map_memory
 *DMM_Reset
 *dmm_reserve_memory
 *dmm_unmap_memory
 *dmm_unreserve_memory
 *
 *  Private Functions:
 *add_region
 *create_region
 *get_region
 *	get_free_region
 *	get_mapped_region
 *
 *  Notes:
 *Region: Generic memory entitiy having a start address and a size
 *Chunk:  Reserved region
 *
 *!
 */

#include <linux/types.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/vmalloc.h>
#include <asm/page.h>
#include "dmm4430.h"


#define DMM_ADDR_VIRTUAL(x, a)						\
	do {								\
		x = (((struct map_page *)(a) - p_virt_mapping_tbl) * PAGE_SIZE \
						+ dyn_mem_map_begin);\
	} while (0)

#define DMM_ADDR_TO_INDEX(i, a)						\
	do {								\
		i = (((a) - dyn_mem_map_begin) / PAGE_SIZE);		\
	} while (0)

struct map_page {
	u32 region_size:31;
	u32 b_reserved:1;
};

/*  Create the free list */
static struct map_page *p_virt_mapping_tbl;
static u32 i_freeregion;	/* The index of free region */
static u32 i_freesize;
static u32 table_size;/* The size of virtual and physical pages tables */
static u32 dyn_mem_map_begin;
struct mutex *dmm_lock;

static struct map_page *get_free_region(u32 size);
static struct map_page *get_mapped_region(u32 addr);

/*  ======== dmm_create_tables ========
 *  Purpose:
 *Create table to hold information of the virtual memory that is reserved
 *for DSP.
 */
int dmm_create_tables(u32 addr, u32 size)
{
	int status = 0;

	dmm_delete_tables();
	if (WARN_ON(mutex_lock_interruptible(dmm_lock)) < 0) {
		status = -EFAULT;
		goto func_exit;
	}
	dyn_mem_map_begin = addr;
	table_size = (size/PAGE_SIZE) + 1;
	/*  Create the free list */
	p_virt_mapping_tbl = (struct map_page *)vmalloc(
					table_size*sizeof(struct map_page));
	if (WARN_ON(p_virt_mapping_tbl == NULL))
		status = -ENOMEM;
	/* On successful allocation,
	* all entries are zero ('free') */
	i_freeregion = 0;
	i_freesize = table_size*PAGE_SIZE;
	p_virt_mapping_tbl[0].region_size = table_size;
	mutex_unlock(dmm_lock);

func_exit:
	return status;
}

/*
 *  ======== dmm_create ========
 *  Purpose:
 *Create a dynamic memory manager object.
 */
int dmm_create(void)
{
	int status = 0;
	dmm_lock = kmalloc(sizeof(struct mutex), GFP_KERNEL);
	if (WARN_ON(dmm_lock == NULL)) {
		status = -EFAULT;
		goto func_exit;
	}
	mutex_init(dmm_lock);
func_exit:
	return status;
}

/*
 *  ======== dmm_destroy ========
 *  Purpose:
 *Release the communication memory manager resources.
 */
void dmm_destroy(void)
{
	dmm_delete_tables();
	kfree(dmm_lock);
}


/*
 *  ======== dmm_delete_tables ========
 *  Purpose:
 *Delete DMM Tables.
 */
void dmm_delete_tables(void)
{
	/* Delete all DMM tables */
	WARN_ON(mutex_lock_interruptible(dmm_lock));
	if (p_virt_mapping_tbl != NULL) {
		vfree(p_virt_mapping_tbl);
		p_virt_mapping_tbl = NULL;
	}
	mutex_unlock(dmm_lock);
}

/*
 *  ======== dmm_init ========
 *  Purpose:
 *Initializes private state of DMM module.
 */
void dmm_init(void)
{
	p_virt_mapping_tbl = NULL ;
	table_size = 0;
	return;
}

/*
 *  ======== dmm_reserve_memory ========
 *  Purpose:
 *Reserve a chunk of virtually contiguous DSP/IVA address space.
 */
int dmm_reserve_memory(u32 size, u32 *p_rsv_addr)
{
	int status = 0;
	struct map_page *node;
	u32 rsv_addr = 0;
	u32 rsv_size = 0;

	if (WARN_ON(mutex_lock_interruptible(dmm_lock)) < 0) {
		status = -EFAULT;
		goto func_exit;
	}

	/* Try to get a DSP chunk from the free list */
	node = get_free_region(size);
	if (node != NULL) {
		/*  DSP chunk of given size is available. */
		DMM_ADDR_VIRTUAL(rsv_addr, node);
		/* Calculate the number entries to use */
		rsv_size = size/PAGE_SIZE;
		if (rsv_size < node->region_size) {
			/* Mark remainder of free region */
			node[rsv_size].b_reserved = false;
			node[rsv_size].region_size =
				node->region_size - rsv_size;
		}
		/*  get_region will return first fit chunk. But we only use what
			is requested. */
		node->b_reserved = true;
		node->region_size = rsv_size;
		/* Return the chunk's starting address */
		*p_rsv_addr = rsv_addr;
	} else
		/*dSP chunk of given size is not available */
		status = -ENOMEM;

	mutex_unlock(dmm_lock);
func_exit:
	return status;
}


/*
 *  ======== dmm_unreserve_memory ========
 *  Purpose:
 *Free a chunk of reserved DSP/IVA address space.
 */
int dmm_unreserve_memory(u32 rsv_addr, u32 *psize)
{
	struct map_page *chunk;
	int status = 0;

	WARN_ON(mutex_lock_interruptible(dmm_lock));

	/* Find the chunk containing the reserved address */
	chunk = get_mapped_region(rsv_addr);
	if (chunk == NULL)
		status = -ENXIO;
	WARN_ON(status < 0);
	if (status == 0) {
		chunk->b_reserved = false;
		*psize = chunk->region_size * PAGE_SIZE;
		/* NOTE: We do NOT coalesce free regions here.
		 * Free regions are coalesced in get_region(), as it traverses
		 *the whole mapping table
		 */
	}
	mutex_unlock(dmm_lock);
	return status;
}

/*
 *  ======== get_free_region ========
 *  Purpose:
 *  Returns the requested free region
 */
static struct map_page *get_free_region(u32 size)
{
	struct map_page *curr_region = NULL;
	u32 i = 0;
	u32 region_size = 0;
	u32 next_i = 0;

	if (p_virt_mapping_tbl == NULL)
		return curr_region;
	if (size > i_freesize) {
		/* Find the largest free region
		* (coalesce during the traversal) */
		while (i < table_size) {
			region_size = p_virt_mapping_tbl[i].region_size;
			next_i = i+region_size;
			if (p_virt_mapping_tbl[i].b_reserved == false) {
				/* Coalesce, if possible */
				if (next_i < table_size &&
				p_virt_mapping_tbl[next_i].b_reserved
							== false) {
					p_virt_mapping_tbl[i].region_size +=
					p_virt_mapping_tbl[next_i].region_size;
					continue;
				}
				region_size *= PAGE_SIZE;
				if (region_size > i_freesize) 	{
					i_freeregion = i;
					i_freesize = region_size;
				}
			}
			i = next_i;
		}
	}
	if (size <= i_freesize) {
		curr_region = p_virt_mapping_tbl + i_freeregion;
		i_freeregion += (size / PAGE_SIZE);
		i_freesize -= size;
	}
	return curr_region;
}

/*
 *  ======== get_mapped_region ========
 *  Purpose:
 *  Returns the requestedmapped region
 */
static struct map_page *get_mapped_region(u32 addr)
{
	u32 i = 0;
	struct map_page *curr_region = NULL;

	if (p_virt_mapping_tbl == NULL)
		return curr_region;

	DMM_ADDR_TO_INDEX(i, addr);
	if (i < table_size && (p_virt_mapping_tbl[i].b_reserved))
		curr_region = p_virt_mapping_tbl + i;
	return curr_region;
}

#ifdef DSP_DMM_DEBUG
int dmm_mem_map_dump(void)
{
	struct map_page *curNode = NULL;
	u32 i;
	u32 freemem = 0;
	u32 bigsize = 0;

	WARN_ON(mutex_lock_interruptible(dmm_lock));

	if (p_virt_mapping_tbl != NULL) {
		for (i = 0; i < table_size; i +=
				p_virt_mapping_tbl[i].region_size) {
			curNode = p_virt_mapping_tbl + i;
			if (curNode->b_reserved == true)	{
				/*printk("RESERVED size = 0x%x, "
					"Map size = 0x%x\n",
					(curNode->region_size * PAGE_SIZE),
					(curNode->b_mapped == false) ? 0 :
					(curNode->mapped_size * PAGE_SIZE));
*/
			} else {
/*				printk("UNRESERVED size = 0x%x\n",
					(curNode->region_size * PAGE_SIZE));
*/
				freemem += (curNode->region_size * PAGE_SIZE);
				if (curNode->region_size > bigsize)
					bigsize = curNode->region_size;
			}
		}
	}
	printk(KERN_INFO "Total DSP VA FREE memory = %d Mbytes\n",
			freemem/(1024*1024));
	printk(KERN_INFO "Total DSP VA USED memory= %d Mbytes \n",
			(((table_size * PAGE_SIZE)-freemem))/(1024*1024));
	printk(KERN_INFO "DSP VA - Biggest FREE block = %d Mbytes \n\n",
			(bigsize*PAGE_SIZE/(1024*1024)));
	mutex_unlock(dmm_lock);

	return 0;
}
#endif
