/*
 *  platform_mem.c
 *
 * Target memory management interface implementation.
 *
 *  This abstracts the Memory management interface in the kernel
 *  code. Allocation, Freeing-up, copy and address translate are
 *  supported for the kernel memory management.
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

/* Linux specific header files */
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include <platform_mem.h>
#include <atomic_linux.h>

/* Macro to make a correct module magic number with ref_count */
#define PLATFORM_MEM_MAKE_MAGICSTAMP(x) ((PLATFORM_MEM_MODULEID << 12u) | (x))

/*
 *  Structure for containing
 */
struct platform_mem_map_table_info {
	struct list_head mem_entry; /* Pointer to mem_entry entry */
	u32	physical_address; /* Actual address */
	u32	knl_virtual_address; /* Mapped address */
	u32	size; /* Size of the region mapped */
};

/*
 *  Structure defining state object of system memory manager
 */
struct platform_mem_module_object {
	atomic_t ref_count; /* Reference count */
	struct list_head map_table; /* Head of map table */
	struct mutex *gate; /* Pointer to lock */
};


/*
 *  Object containing state of the platform mem module
 */
static struct platform_mem_module_object platform_mem_state;

/*
 * ======== platform_mem_setup ========
 *  Purpose:
 *  This will initialize the platform mem module.
 */
int platform_mem_setup(void)
{
	s32 retval = 0;

	atomic_cmpmask_and_set(&platform_mem_state.ref_count,
				PLATFORM_MEM_MAKE_MAGICSTAMP(0),
				PLATFORM_MEM_MAKE_MAGICSTAMP(0));
	if (atomic_inc_return(&platform_mem_state.ref_count)
				!= PLATFORM_MEM_MAKE_MAGICSTAMP(1)) {
		return 1;
	}

	/* Create the Gate handle */
	platform_mem_state.gate =
			kmalloc(sizeof(struct mutex), GFP_KERNEL);
	if (platform_mem_state.gate == NULL) {
			retval = -ENOMEM;
		goto gate_create_fail;
	}

	/* Construct the map table */
	INIT_LIST_HEAD(&platform_mem_state.map_table);
	mutex_init(platform_mem_state.gate);
	goto exit;

gate_create_fail:
	atomic_set(&platform_mem_state.ref_count,
			PLATFORM_MEM_MAKE_MAGICSTAMP(0));
exit:
	return retval;
}
EXPORT_SYMBOL(platform_mem_setup);

/*
 * ======== platform_mem_destroy ========
 *  Purpose:
 *  This will finalize the platform mem module.
 */
int platform_mem_destroy(void)
{
	s32 retval = 0;

	if (atomic_cmpmask_and_lt(&(platform_mem_state.ref_count),
				  PLATFORM_MEM_MAKE_MAGICSTAMP(0),
				  PLATFORM_MEM_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto exit;
	}

	if (atomic_dec_return(&platform_mem_state.ref_count)
			== PLATFORM_MEM_MAKE_MAGICSTAMP(0)) {
		list_del(&platform_mem_state.map_table);
		/* Delete the gate handle */
		kfree(platform_mem_state.gate);
	}

exit:
	return  retval;
}
EXPORT_SYMBOL(platform_mem_destroy);

/*
 * ======== platform_mem_map ========
 *  Purpose:
 *  This will  maps a memory area into virtual space.
 */
int platform_mem_map(memory_map_info *map_info)
{
	int retval = 0;
	struct platform_mem_map_table_info *info = NULL;

	if (atomic_cmpmask_and_lt(&(platform_mem_state.ref_count),
				PLATFORM_MEM_MAKE_MAGICSTAMP(0),
				PLATFORM_MEM_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto exit;
	}

	if (WARN_ON(map_info == NULL)) {
		retval = -EINVAL;
		goto exit;

	}

	if (map_info->src == (u32) NULL) {
		retval = -EINVAL;
		goto exit;
	}

	info = kmalloc(sizeof(struct platform_mem_map_table_info),
					GFP_KERNEL);
	if (info == NULL) {
		retval = -ENOMEM;
		goto exit;
	}

	retval = mutex_lock_interruptible(platform_mem_state.gate);
	if (retval)
		goto lock_fail;

	map_info->dst = 0;
	if (map_info->is_cached == true)
		map_info->dst = (u32) ioremap((dma_addr_t)
					(map_info->src), map_info->size);
	else
		map_info->dst = (u32) ioremap_nocache((dma_addr_t)
					(map_info->src), map_info->size);

	if (map_info->dst == 0) {
			retval = -EFAULT;
			goto ioremap_fail;
	}

	/* Populate the info */
	info->physical_address = map_info->src;
	info->knl_virtual_address = map_info->dst;
	info->size = map_info->size;
	/* Put the info into the list */
	list_add(&info->mem_entry, &platform_mem_state.map_table);
	mutex_unlock(platform_mem_state.gate);
	goto exit;

ioremap_fail:
	mutex_unlock(platform_mem_state.gate);
lock_fail:
	kfree(info);
exit:
	return retval;
}
EXPORT_SYMBOL(platform_mem_map);

/*
 * ======== platform_mem_unmap ========
 *  Purpose:
 *  This will  unmaps a memory area into virtual space.
 */
int platform_mem_unmap(memory_unmap_info *unmap_info)
{
	s32 retval = 0;
	struct platform_mem_map_table_info *info = NULL;


	if (atomic_cmpmask_and_lt(&(platform_mem_state.ref_count),
			  PLATFORM_MEM_MAKE_MAGICSTAMP(0),
			  PLATFORM_MEM_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto exit;
	}

	if (unmap_info == NULL) {
		retval = -EINVAL;
		goto exit;
	}

	if (unmap_info->addr == (u32) NULL) {
		retval = -EINVAL;
		goto exit;
	}

	retval = mutex_lock_interruptible(platform_mem_state.gate);
	if (retval)
		goto exit;

	iounmap((unsigned int *) unmap_info->addr);
	/* Delete the node in the map table */
	list_for_each_entry(info, &platform_mem_state.map_table, mem_entry) {
		if (info->knl_virtual_address == unmap_info->addr) {
			list_del(&info->mem_entry);
			kfree(info);
			break;
		}
	}
	mutex_unlock(platform_mem_state.gate);

exit:
	return retval;
}
EXPORT_SYMBOL(platform_mem_unmap);

/*
 * ======== platform_mem_map ========
 *  Purpose:
 *  This will translate an address.
 */
void *platform_mem_translate(void *src_addr, enum memory_xlt_flags flags)
{
	void *buf = NULL;
	struct platform_mem_map_table_info *tinfo  = NULL;
	u32 frm_addr;
	u32 to_addr;
	s32 retval = 0;

	if (atomic_cmpmask_and_lt(&(platform_mem_state.ref_count),
				PLATFORM_MEM_MAKE_MAGICSTAMP(0),
				PLATFORM_MEM_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto exit;
	}

	retval = mutex_lock_interruptible(platform_mem_state.gate);
	if (retval)
		goto exit;

	/* Traverse to the node in the map table */
	list_for_each_entry(tinfo, &platform_mem_state.map_table, mem_entry) {
		frm_addr = (flags == PLATFORM_MEM_XLT_FLAGS_VIRT2PHYS) ?
			tinfo->knl_virtual_address : tinfo->physical_address;
		to_addr = (flags == PLATFORM_MEM_XLT_FLAGS_VIRT2PHYS) ?
			tinfo->physical_address : tinfo->knl_virtual_address;
		if ((((u32) src_addr) >= frm_addr)
			&& (((u32) src_addr) < (frm_addr + tinfo->size))) {
			buf = (void *) (to_addr + ((u32)src_addr - frm_addr));
			break;
		}
	}
	mutex_unlock(platform_mem_state.gate);

exit:
	return buf;
}
EXPORT_SYMBOL(platform_mem_translate);

