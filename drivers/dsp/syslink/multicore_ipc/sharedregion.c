/*
 *  sharedregion.c
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
#include <linux/module.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <syslink/atomic_linux.h>

#include <multiproc.h>
#include <nameserver.h>
#include <sharedregion.h>

/* Macro to make a correct module magic number with refCount */
#define SHAREDREGION_MAKE_MAGICSTAMP(x)   ((SHAREDREGION_MODULEID << 16u) | (x))

#define SHAREDREGION_MAX_REGIONS_DEFAULT  4

/*
 *  Module state object
 */
struct sharedregion_module_object {
	atomic_t ref_count;   /* Reference count */
	struct mutex *gate_handle;
	struct sharedregion_info *table; /* Ptr to the table */
	u32 bitOffset;  /* Index bit offset */
	u32 region_size; /* Max size of each region */
	struct sharedregion_config cfg;	/* Current config values */
	u32 *ref_count_table; /* The number of times each
							entry has been added */
};

/*
 *  Shared region state object variable with default settings
 */
static struct sharedregion_module_object sharedregion_state = {
	.cfg.heap_handle = NULL,
	.cfg.gate_handle = NULL,
	.cfg.max_regions = SHAREDREGION_MAX_REGIONS_DEFAULT
};

/*
 * ======== sharedregion_get_config ========
 *  Purpose:
 *  This will get  sharedregion module configiguration
 */
int sharedregion_get_config(struct sharedregion_config *config)
{
	BUG_ON((config == NULL));
	memcpy(config, &sharedregion_state.cfg,
				sizeof(struct sharedregion_config));
	return 0;
}
EXPORT_SYMBOL(sharedregion_get_config);


/*
 * ======== sharedregion_get_bitoffset  ========
 *  Purpose:
 *  This will get get the bit offset
 */
static u32 sharedregion_get_bitoffset(u32 max_regions)
{
	u32 i;
	u32 bitoffset = 0;
	for (i = ((sizeof(void *) * 8) - 1); i >= 0; i--) {
		if (max_regions > (1 << i))
			break;
	}

	bitoffset = (((sizeof(void *) * 8) - 1) - i);
	return bitoffset;
}

/*
 * ======== sharedregion_setup ========
 *  Purpose:
 *  This will get setup the sharedregion module
 */
int sharedregion_setup(const struct sharedregion_config *config)
{
	struct sharedregion_config *tmpcfg = &sharedregion_state.cfg;
	struct sharedregion_info *table = NULL;
	u32 i;
	u32 j;
	s32 retval = 0;
	u16 proc_count;

	/* This sets the refCount variable is not initialized, upper 16 bits is
	* written with module Id to ensure correctness of refCount variable
	*/
	atomic_cmpmask_and_set(&sharedregion_state.ref_count,
				SHAREDREGION_MAKE_MAGICSTAMP(0),
				SHAREDREGION_MAKE_MAGICSTAMP(0));

	if (atomic_inc_return(&sharedregion_state.ref_count)
				!= SHAREDREGION_MAKE_MAGICSTAMP(1)) {
		return 1;
	}

	if (config != NULL) {
		if (WARN_ON(config->max_regions == 0)) {
			retval = -EINVAL;
			goto error;
		}
		memcpy(&sharedregion_state.cfg, config,
				sizeof(struct sharedregion_config));
	}

	sharedregion_state.gate_handle = kmalloc(sizeof(struct mutex),
							GFP_KERNEL);
	if (sharedregion_state.gate_handle == NULL)
		goto gate_create_fail;

	sharedregion_state.bitOffset =
			sharedregion_get_bitoffset(tmpcfg->max_regions);
	sharedregion_state.region_size = (1 << sharedregion_state.bitOffset);
	proc_count = multiproc_get_max_processors();
	 /* TODO check heap usage & + 1 ? */
	sharedregion_state.table = kmalloc(sizeof(struct sharedregion_info) *
					tmpcfg->max_regions * (proc_count + 1),
					GFP_KERNEL);
	if (sharedregion_state.table == NULL) {
		retval = -ENOMEM;
		goto table_alloc_fail;
	}

	sharedregion_state.ref_count_table = kmalloc(sizeof(u32) *
					tmpcfg->max_regions * (proc_count + 1),
					GFP_KERNEL);
	if (sharedregion_state.ref_count_table == NULL) {
		retval = -ENOMEM;
		goto table_alloc_fail;
	}

	table = sharedregion_state.table;
	for (i = 0; i < tmpcfg->max_regions; i++) {
		for (j = 0; j < (proc_count + 1); j++) {
			(table + (j * tmpcfg->max_regions) + i)->is_valid =
									false;
			(table + (j * tmpcfg->max_regions) + i)->base = 0;
			(table + (j * tmpcfg->max_regions) + i)->len = 0;
			sharedregion_state.ref_count_table[(j *
						tmpcfg->max_regions) + i] = 0;
		}
	}

	mutex_init(sharedregion_state.gate_handle);
	return 0;

table_alloc_fail:
	kfree(sharedregion_state.gate_handle);

gate_create_fail:
	memset(&sharedregion_state, 0,
		sizeof(struct sharedregion_module_object));
	sharedregion_state.cfg.max_regions = SHAREDREGION_MAX_REGIONS_DEFAULT;

error:
	printk(KERN_ERR "sharedregion_setup failed status:%x\n", retval);
	return retval;
}
EXPORT_SYMBOL(sharedregion_setup);

/*
 * ======== sharedregion_destroy ========
 *  Purpose:
 *  This will get destroy the sharedregion module
 */
int sharedregion_destroy(void)
{
	s32 retval = 0;
	void *gate_handle = NULL;

	if (atomic_cmpmask_and_lt(&(sharedregion_state.ref_count),
				SHAREDREGION_MAKE_MAGICSTAMP(0),
				SHAREDREGION_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto error;
	}

	if (!(atomic_dec_return(&sharedregion_state.ref_count)
					== SHAREDREGION_MAKE_MAGICSTAMP(0))) {
		retval = 1; /* Syslink is not handling this on 2.0.0.06 */
		goto error;
	}

	retval = mutex_lock_interruptible(sharedregion_state.gate_handle);
	if (retval)
		goto error;

	kfree(sharedregion_state.ref_count_table);
	kfree(sharedregion_state.table);
	gate_handle = sharedregion_state.gate_handle; /* backup gate handle */
	memset(&sharedregion_state, 0,
		sizeof(struct sharedregion_module_object));
	sharedregion_state.cfg.max_regions = SHAREDREGION_MAX_REGIONS_DEFAULT;
	mutex_unlock(gate_handle);
	kfree(gate_handle);
	return 0;

error:
	if (retval < 0) {
		printk(KERN_ERR "sharedregion_destroy failed status:%x\n",
			retval);
	}
	return retval;
}
EXPORT_SYMBOL(sharedregion_destroy);

/*
 * ======== sharedregion_add ========
 *  Purpose:
 *  This will add a memory segment to the lookup table
 *  during runtime by base and length
 */
int sharedregion_add(u32 index, void *base, u32 len)
{
	struct sharedregion_info *entry = NULL;
	struct sharedregion_info *table = NULL;
	s32 retval = 0;
	u32 i;
	u16 myproc_id;
	bool overlap = false;
	bool same = false;

	if (atomic_cmpmask_and_lt(&(sharedregion_state.ref_count),
				SHAREDREGION_MAKE_MAGICSTAMP(0),
				SHAREDREGION_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto error;
	}

	if (index >= sharedregion_state.cfg.max_regions ||
			sharedregion_state.region_size < len) {
		retval = -EINVAL;
		goto error;
	}

	myproc_id = multiproc_get_id(NULL);
	retval = mutex_lock_interruptible(sharedregion_state.gate_handle);
	if (retval)
		goto error;


	table = sharedregion_state.table;
	/* Check for overlap */
	for (i = 0; i < sharedregion_state.cfg.max_regions; i++) {
		entry = (table
			+ (myproc_id * sharedregion_state.cfg.max_regions)
			+ i);
		if (entry->is_valid) {
			/* Handle duplicate entry */
			if ((base == entry->base) && (len == entry->len)) {
				same = true;
				break;
			}

			if ((base >= entry->base) &&
			(base < (void *)((u32)entry->base + entry->len))) {
				overlap = true;
				break;
			}

			if ((base < entry->base) &&
			(void *)((u32)base + len) >= entry->base) {
				overlap = true;
				break;
			}
		}
	}

	if (same) {
		retval = 1;
		goto success;
	}

	if (overlap) {
		/* FHACK: FIX ME */
		retval = 1;
		goto mem_overlap_error;
	}

	entry = (table
		 + (myproc_id * sharedregion_state.cfg.max_regions)
		 + index);
	if (entry->is_valid == false) {
		entry->base = base;
		entry->len = len;
		entry->is_valid = true;

	} else {
		/* FHACK: FIX ME */
		sharedregion_state.ref_count_table[(myproc_id *
				sharedregion_state.cfg.max_regions)
				+ index] += 1;
		retval = 1;
		goto dup_entry_error;
	}

success:
	mutex_unlock(sharedregion_state.gate_handle);
	return 0;

dup_entry_error: /* Fall through */
mem_overlap_error:
	printk(KERN_WARNING "sharedregion_add entry exists status: %x\n",
		retval);
	mutex_unlock(sharedregion_state.gate_handle);

error:
	if (retval < 0)
		printk(KERN_ERR "sharedregion_add failed status:%x\n", retval);
	return retval;
}
EXPORT_SYMBOL(sharedregion_add);

/*
 * ======== sharedregion_remove ========
 *  Purpose:
 *  This will removes a memory segment to the lookup table
 *  during runtime by base and length
 */
int sharedregion_remove(u32 index)
{
	struct sharedregion_info *entry = NULL;
	struct sharedregion_info *table = NULL;
	u16 myproc_id;
	s32 retval = 0;

	if (atomic_cmpmask_and_lt(&(sharedregion_state.ref_count),
				SHAREDREGION_MAKE_MAGICSTAMP(0),
				SHAREDREGION_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto error;
	}

	if (index >= sharedregion_state.cfg.max_regions) {
		retval = -EINVAL;
		goto error;
	}

	retval = mutex_lock_interruptible(sharedregion_state.gate_handle);
	if (retval)
		goto error;

	myproc_id = multiproc_get_id(NULL);
	table = sharedregion_state.table;
	entry = (table
		 + (myproc_id * sharedregion_state.cfg.max_regions)
		 + index);

	if (sharedregion_state.ref_count_table[(myproc_id *
				sharedregion_state.cfg.max_regions)
				+ index] > 0)
		sharedregion_state.ref_count_table[(myproc_id *
				sharedregion_state.cfg.max_regions)
				+ index] -= 1;
	else {
		entry->is_valid = false;
		entry->base = NULL;
		entry->len = 0;
	}
	mutex_unlock(sharedregion_state.gate_handle);
	return 0;

error:
	printk(KERN_ERR "sharedregion_remove failed status:%x\n", retval);
	return retval;
}
EXPORT_SYMBOL(sharedregion_remove);

/*
 * ======== sharedregion_get_index ========
 *  Purpose:
 *  This will return the index for the specified address pointer.
 */
int sharedregion_get_index(void *addr)
{
	struct sharedregion_info *entry = NULL;
	struct sharedregion_info *table = NULL;
	bool found = false;
	u32 i;
	u16 myproc_id;
	s32 retval = 0;

	if (WARN_ON(atomic_cmpmask_and_lt(&(sharedregion_state.ref_count),
				SHAREDREGION_MAKE_MAGICSTAMP(0),
				SHAREDREGION_MAKE_MAGICSTAMP(1)) == true)) {
		retval = -ENODEV;
		goto exit;
	}

	myproc_id = multiproc_get_id(NULL);
	retval = mutex_lock_interruptible(sharedregion_state.gate_handle);
	if (retval) {
		retval = -ENODEV;
		goto exit;
	}

	table = sharedregion_state.table;
	for (i = 0; i < sharedregion_state.cfg.max_regions; i++) {
		entry = (table
			+ (myproc_id * sharedregion_state.cfg.max_regions)
			+ i);
		if ((addr >= entry->base) &&
		(addr < (void *)((u32)entry->base + (entry->len)))) {
			found = true;
			break;
		}
	}

	if (found)
		retval = i;
	else
		retval = -ENOENT; /* No entry found in the table */

	mutex_unlock(sharedregion_state.gate_handle);
	return retval;

exit:
	printk(KERN_ERR "sharedregion_get_index failed index:%x\n", retval);
	return retval;
}
EXPORT_SYMBOL(sharedregion_get_index);

/*
 * ======== sharedregion_get_ptr ========
 *  Purpose:
 *  This will return the address pointer associated with the
 *  shared region pointer
 */
void *sharedregion_get_ptr(u32 *srptr)
{
	struct sharedregion_info *entry = NULL;
	void *ptr = NULL;
	u16 myproc_id;
	s32 retval = 0;

	if (atomic_cmpmask_and_lt(&(sharedregion_state.ref_count),
				SHAREDREGION_MAKE_MAGICSTAMP(0),
				SHAREDREGION_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto error;
	}

	if (srptr == SHAREDREGION_INVALIDSRPTR)
		goto error;

	myproc_id = multiproc_get_id(NULL);
	retval = mutex_lock_interruptible(sharedregion_state.gate_handle);
	if (WARN_ON(retval != 0))
		goto error;

	entry = (sharedregion_state.table
		 + (myproc_id * sharedregion_state.cfg.max_regions)
		 + ((u32)srptr >> sharedregion_state.bitOffset));
	/* TO DO check:: is this correct ? */
	ptr = ((void *)(((u32)srptr &
		((1 << sharedregion_state.bitOffset) - 1)) + (u32)entry->base));
	mutex_unlock(sharedregion_state.gate_handle);
	return ptr;

error:
	printk(KERN_ERR "sharedregion_get_ptr failed \n");
	return (void *)NULL;

}
EXPORT_SYMBOL(sharedregion_get_ptr);

/*
 * ======== sharedregion_get_srptr ========
 *  Purpose:
 *  This will return sharedregion pointer associated with the
 *  an address in a shared region area registered with the
 *  sharedregion module
 */
u32 *sharedregion_get_srptr(void *addr, s32 index)
{
	struct sharedregion_info *entry = NULL;
	u32 *ptr = SHAREDREGION_INVALIDSRPTR ;
	u32 myproc_id;
	s32 retval = 0;

	if (atomic_cmpmask_and_lt(&(sharedregion_state.ref_count),
				SHAREDREGION_MAKE_MAGICSTAMP(0),
				SHAREDREGION_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto error;
	}

	if (WARN_ON(addr == NULL))
		goto error;

	if (WARN_ON(index >= sharedregion_state.cfg.max_regions))
		goto error;

	retval = mutex_lock_interruptible(sharedregion_state.gate_handle);
	if (WARN_ON(retval != 0))
		goto error;

	myproc_id = multiproc_get_id(NULL);
	entry = (sharedregion_state.table
		+ (myproc_id * sharedregion_state.cfg.max_regions)
		+ index);
	ptr = (u32 *) ((index << sharedregion_state.bitOffset)
				| ((u32)addr - (u32)entry->base));
	mutex_unlock(sharedregion_state.gate_handle);
	return ptr;

error:
	printk(KERN_ERR	"sharedregion_get_srptr failed\n");
	return (u32 *)NULL;
}
EXPORT_SYMBOL(sharedregion_get_srptr);

/*
 * ======== sharedregion_get_table_info ========
 *  Purpose:
 *  This will get the table entry information for the
 *  specified index and id
 */
int sharedregion_get_table_info(u32 index, u16 proc_id,
				struct sharedregion_info *info)
{
	struct sharedregion_info *entry = NULL;
	struct sharedregion_info *table = NULL;
	u16 proc_count;
	s32 retval = 0;

	BUG_ON(info == NULL);
	if (atomic_cmpmask_and_lt(&(sharedregion_state.ref_count),
				SHAREDREGION_MAKE_MAGICSTAMP(0),
				SHAREDREGION_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto error;
	}

	proc_count = multiproc_get_max_processors();
	if (index >= sharedregion_state.cfg.max_regions ||
						proc_id >= proc_count) {
		retval = -EINVAL;
		goto error;
	}

	retval = mutex_lock_interruptible(sharedregion_state.gate_handle);
	if (retval)
		goto error;

	table = sharedregion_state.table;
	entry = (table
		+ (proc_id * sharedregion_state.cfg.max_regions)
		+ index);
	memcpy((void *) info, (void *) entry, sizeof(struct sharedregion_info));
	mutex_unlock(sharedregion_state.gate_handle);
	return 0;

error:
	printk(KERN_ERR "sharedregion_get_table_info failed status:%x\n",
			retval);
	return retval;
}
EXPORT_SYMBOL(sharedregion_get_table_info);

/*
 * ======== sharedregion_set_table_info ========
 *  Purpose:
 *  This will set the table entry information for the
 *  specified index and id
 */
int sharedregion_set_table_info(u32 index, u16 proc_id,
				struct sharedregion_info *info)
{
	struct sharedregion_info *entry = NULL;
	struct sharedregion_info *table = NULL;
	u16 proc_count;
	s32 retval = 0;

	BUG_ON(info != NULL);
	if (atomic_cmpmask_and_lt(&(sharedregion_state.ref_count),
				SHAREDREGION_MAKE_MAGICSTAMP(0),
				SHAREDREGION_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto error;
	}

	proc_count = multiproc_get_max_processors();
	if (index >= sharedregion_state.cfg.max_regions ||
						proc_id >= proc_count) {
		retval = -EINVAL;
		goto error;
	}

	retval = mutex_lock_interruptible(sharedregion_state.gate_handle);
	if (retval)
		goto error;

	table = sharedregion_state.table;
	entry = (table
		+ (proc_id * sharedregion_state.cfg.max_regions)
		+ index);
	memcpy((void *) entry, (void *) info, sizeof(struct sharedregion_info));
	mutex_unlock(sharedregion_state.gate_handle);
	return 0;

error:
	printk(KERN_ERR "sharedregion_set_table_info failed status:%x\n",
			retval);
	return retval;
}
EXPORT_SYMBOL(sharedregion_set_table_info);

/*
 * ======== Sharedregion_attach ========
 *  Purpose:
 *  This will attachs the shared region with an proc_id
 *
 *  Application should call this function from the callback
 *  function registered for device attach to the processor
 *  manager. All modules which requires some logic setup
 *  to be done when a device gets attach to the system,
 *  should export API like this. Please see the below psuedo
 *  code for example:
 *  Example
 *  code
 *      void function (proc_id, config) {
 *          NotifyDriver_attach (proc_id, config->ndParams);
 *           SysMemMgr_attach (proc_id);
 *           SMM_attach (proc_id);
 *            NameServerRemoteTransport_attach(proc_id, config->nsrtParams);
 *            SharedRegion_attach (proc_id);
 *            SharedMemory_getConfig (&cfg);
 *             for (i = 0u; i < cfg->maxRegions; i++) {
 *                  SharedRegion_getTableInfo(i, &myinfo,   myProcId);
 *                  SharedRegion_getTableInfo(i, &peerinfo, proc_id);
 *                   DMM_map (proc_id,
 *                           PA(myinfo->vaddr),
 *                            peerinfo->vaddr,
 *                            myinfo->len);
 *               }
 *               ...
 *          }
 *
 *          main () {
 *               # attach callback for device attach only
 *                ProcMgr_register (function, proc_id, DEV_ATTACH);
 *          }
 *
 */
void sharedregion_attach(u16 proc_id)
{
	struct sharedregion_info *entry = NULL;
	struct sharedregion_info *table = NULL;
	char *hexstr = "0123456789ABCDEF";
	char tname[80];
	u16 proc_id_list[2];
	u32 addr = 0;
	u32 len;
	u16 proc_count;
	void *nshandle;
	s32 retval = 0;
	s32 i;

	if (WARN_ON(sharedregion_state.table == NULL))
		goto error;

	proc_count = multiproc_get_max_processors();
	if (WARN_ON(proc_id >= proc_count))
		goto error;

	proc_id_list[0] = proc_id;
	proc_id_list[1] = MULTIPROC_INVALIDID;
	nshandle = nameserver_get_handle(SHAREDREGION_NAMESERVER);
	if (nshandle == NULL)
		goto error;

	/* Get Shared region entries from the remote shared region nameserver */
	for (i = 0u; i < sharedregion_state.cfg.max_regions; i++) {
		memset(tname, 0, 80);
		strcpy(tname, "SHAREDREGION:SRENTRY_ADDR_");
		tname[strlen(tname)] = hexstr[(proc_id >> 4u) & 0xF];
		tname[strlen(tname)] = hexstr[proc_id & 0xF];
		tname[strlen(tname)] = '_';
		tname[strlen(tname)] = hexstr[(i >> 28u) & 0xF];
		tname[strlen(tname)] = hexstr[(i >> 24u) & 0xF];
		tname[strlen(tname)] = hexstr[(i >> 20u) & 0xF];
		tname[strlen(tname)] = hexstr[(i >> 16u) & 0xF];
		tname[strlen(tname)] = hexstr[(i >> 12u) & 0xF];
		tname[strlen(tname)] = hexstr[(i >>  8u) & 0xF];
		tname[strlen(tname)] = hexstr[(i >>  4u) & 0xF];
		tname[strlen(tname)] = hexstr[i & 0xF];
		retval = nameserver_get(nshandle, tname,
					&len, sizeof(u32), &proc_id_list[0]);
		if (WARN_ON(retval))
			;

		memset(tname, 0, 80u);
		strcpy(tname, "SHAREDREGION:SRENTRY_LEN_");
		tname[strlen(tname)] = hexstr[(proc_id >> 4u) & 0xF];
		tname[strlen(tname)] = hexstr[proc_id & 0xF];
		tname[strlen(tname)] = '_';
		tname[strlen(tname)] = hexstr[(i >> 28u) & 0xF];
		tname[strlen(tname)] = hexstr[(i >> 24u) & 0xF];
		tname[strlen(tname)] = hexstr[(i >> 20u) & 0xF];
		tname[strlen(tname)] = hexstr[(i >> 16u) & 0xF];
		tname[strlen(tname)] = hexstr[(i >> 12u) & 0xF];
		tname[strlen(tname)] = hexstr[(i >>  8u) & 0xF];
		tname[strlen(tname)] = hexstr[(i >>  4u) & 0xF];
		tname[strlen(tname)] = hexstr[i & 0xF];

		/* TO DO : check this */
		retval = nameserver_get(nshandle, tname,
					&len, sizeof(u32), &proc_id_list[0]);
		if (WARN_ON(retval))
			;

		/* Found an entry in the remote nameserver */
		/* Add it into the shared region table */
		if (retval == 0) {
			retval = mutex_lock_interruptible(
					sharedregion_state.gate_handle);
			if (WARN_ON(retval))
				break;

			table = sharedregion_state.table;
			/* mark entry invalid */
			entry = (table
				+ (proc_id * sharedregion_state.cfg.max_regions)
				+ i);
			entry->base    = (void *)addr;
			entry->len     = len;
			entry->is_valid = false;
			mutex_unlock(sharedregion_state.gate_handle);
		}

	}

error:
	return;
}
EXPORT_SYMBOL(sharedregion_attach);

/*
 * ======== Sharedregion_detach ========
 *  Purpose:
 *  This will detachs the shared region for an proc_id
 *
 *  Application should call this function from the callback
 *  function registered for device detach to the processorl
 *  manager. All modules which requires some logic setup
 *  to be done when a device gets detach from the system,
 *  should export API like this.
 *  Please see the below psuedo code for example:
 *  @Example
 *  @code
 *    void function (proc_id) {
 *    	SharedRegion_detach (proc_id);
 *    	SysMemMgr_detach (proc_id);
 *     	...
 *   	# Name server must be detached last
 *    	nameserver_detach (proc_id);
 *     }
 *
 *    main () {
 *    	# attach callback for device detach only
 *	Processor_register (function, proc_id, DEV_DETACH);
 *    }
 *
 */
void sharedregion_detach(u16 proc_id)
{
	u16 proc_count;

	if (WARN_ON(sharedregion_state.table == NULL))
		goto error;

	proc_count = multiproc_get_max_processors();
	if (WARN_ON(proc_id >= proc_count))
		goto error;

error:
	return;
}
EXPORT_SYMBOL(sharedregion_detach);

