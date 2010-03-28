/*
 *  gatehwspinlock.c
 *
 *  Hardware-based spinlock gate for mutual exclusion of shared memory.
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
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include <syslink/atomic_linux.h>
#include <multiproc.h>
#include <sharedregion.h>
#include <gatemp.h>
#include <igatempsupport.h>
#include <igateprovider.h>
#include <iobject.h>
#include <gatehwspinlock.h>


/* =============================================================================
 * Macros
 * =============================================================================
 */

/* Macro to make a correct module magic number with refCount */
#define GATEHWSPINLOCK_MAKE_MAGICSTAMP(x) ((GATEHWSPINLOCK_MODULEID << 12u)  \
					| (x))
/*
 *  structure for gatehwspinlock module state
 */
struct gatehwspinlock_module_object {
	atomic_t ref_count; /* Reference count */
	struct gatehwspinlock_config cfg;
	struct gatehwspinlock_config default_cfg;
	struct gatehwspinlock_params def_inst_params; /* default instance
							paramters */
	u32 *base_addr; /* Base address of lock registers */
	u32 num_locks; /* Maximum number of locks */
};

/*
 *  Structure defining object for the Gate Spinlock
 */
struct gatehwspinlock_object {
	IGATEPROVIDER_SUPEROBJECT; /* For inheritance from IGateProvider */
	IOBJECT_SUPEROBJECT;       /* For inheritance for IObject */
	u32 lock_num;
	u32 nested;
	void *local_gate;
};

/*
 *  Variable for holding state of the gatehwspinlock module
 */
struct gatehwspinlock_module_object gatehwspinlock_state = {
	.default_cfg.default_protection	= \
					gatehwspinlock_LOCALPROTECT_INTERRUPT,
	.default_cfg.num_locks		= 32u,
	.def_inst_params.shared_addr	= NULL,
	.def_inst_params.resource_id	= 0x0,
	.def_inst_params.region_id	= 0x0,
	.num_locks			= 32u
};

static struct gatehwspinlock_module_object *gatehwspinlock_module =
						&gatehwspinlock_state;

/* =============================================================================
 * Internal functions
 * =============================================================================
 */

/* TODO: figure these out */
#define gate_enter_system() 0
#define gate_leave_system(key) {}

/* =============================================================================
 * APIS
 * =============================================================================
 */
/*
 * ======== gatehwspinlock_get_config ========
 *  Purpose:
 *  This will get the default configuration parameters for gatehwspinlock
 *  module
 */
void gatehwspinlock_get_config(struct gatehwspinlock_config *config)
{
	if (WARN_ON(config == NULL))
		goto exit;

	if (atomic_cmpmask_and_lt(&(gatehwspinlock_module->ref_count),
				GATEHWSPINLOCK_MAKE_MAGICSTAMP(0),
				GATEHWSPINLOCK_MAKE_MAGICSTAMP(1)) == true)
		memcpy(config, &gatehwspinlock_module->default_cfg,
					sizeof(struct gatehwspinlock_config));
	else
		memcpy(config, &gatehwspinlock_module->cfg,
					sizeof(struct gatehwspinlock_config));

exit:
	return;
}
EXPORT_SYMBOL(gatehwspinlock_get_config);

/*
 * ======== gatehwspinlock_setup ========
 *  Purpose:
 *  This will setup the gatehwspinlock module
 */
int gatehwspinlock_setup(const struct gatehwspinlock_config *config)
{
	struct gatehwspinlock_config tmp_cfg;
	int *key = 0;

	key = gate_enter_system();

	/* This sets the ref_count variable  not initialized, upper 16 bits is
	* written with module _id to ensure correctness of ref_count variable
	*/
	atomic_cmpmask_and_set(&gatehwspinlock_module->ref_count,
					GATEHWSPINLOCK_MAKE_MAGICSTAMP(0),
					GATEHWSPINLOCK_MAKE_MAGICSTAMP(0));

	if (atomic_inc_return(&gatehwspinlock_module->ref_count)
					!= GATEHWSPINLOCK_MAKE_MAGICSTAMP(1)) {
		gate_leave_system(key);
		return 1;
	}

	if (config == NULL) {
		gatehwspinlock_get_config(&tmp_cfg);
		config = &tmp_cfg;
	}
	gate_leave_system(key);

	memcpy(&gatehwspinlock_module->cfg, config,
					sizeof(struct gatehwspinlock_config));

	gatehwspinlock_module->base_addr = (void *)config->base_addr;
	gatehwspinlock_module->num_locks = config->num_locks;

	return 0;

}
EXPORT_SYMBOL(gatehwspinlock_setup);

/*
 * ======== gatehwspinlock_destroy ========
 *  Purpose:
 *  This will destroy the gatehwspinlock module
 */
int gatehwspinlock_destroy(void)
{
	s32 retval = 0;
	int *key = 0;

	key = gate_enter_system();

	if (atomic_cmpmask_and_lt(&(gatehwspinlock_module->ref_count),
				GATEHWSPINLOCK_MAKE_MAGICSTAMP(0),
				GATEHWSPINLOCK_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto exit;
	}

	if (!(atomic_dec_return(&gatehwspinlock_module->ref_count)
			== GATEHWSPINLOCK_MAKE_MAGICSTAMP(0))) {
		gate_leave_system(key);
		retval = 1;
		goto exit;
	}
	gate_leave_system(key);

	memset(&gatehwspinlock_module->cfg, 0,
				sizeof(struct gatehwspinlock_config));
	return 0;

exit:
	if (retval < 0) {
		printk(KERN_ERR "gatehwspinlock_destroy failed status:%x\n",
			retval);
	}
	return retval;

}
EXPORT_SYMBOL(gatehwspinlock_destroy);

/*
 * ======== gatehwspinlock_get_num_instances ========
 *  Purpose:
 *  Function to return the number of instances configured in the module.
 */
u32 gatehwspinlock_get_num_instances(void)
{
	return gatehwspinlock_module->num_locks;
}
EXPORT_SYMBOL(gatehwspinlock_get_num_instances);

/*
 * ======== gatepeterson_locks_init ========
 *  Purpose:
 *  Function to initialize the locks.
 */
void gatehwspinlock_locks_init(void)
{
	u32 i;

	for (i = 0; i < gatehwspinlock_module->num_locks; i++)
		gatehwspinlock_module->base_addr[i] = 0;
}
EXPORT_SYMBOL(gatehwspinlock_locks_init);

/*
 * ======== gatehwspinlock_params_init ========
 *  Purpose:
 *  This will  Initialize this config-params structure with
 *  supplier-specified defaults before instance creation
 */
void gatehwspinlock_params_init(struct gatehwspinlock_params *params)
{
	int *key = 0;

	key = gate_enter_system();

	if (WARN_ON(atomic_cmpmask_and_lt(&(gatehwspinlock_module->ref_count),
				GATEHWSPINLOCK_MAKE_MAGICSTAMP(0),
				GATEHWSPINLOCK_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	if (WARN_ON(params == NULL))
		goto exit;

	memcpy(params, &(gatehwspinlock_module->def_inst_params),
				sizeof(struct gatehwspinlock_params));

exit:
	gate_leave_system(key);
	return;
}
EXPORT_SYMBOL(gatehwspinlock_params_init);

/*
 * ======== gatehwspinlock_create ========
 *  Purpose:
 *  This will creates a new instance of gatehwspinlock module
 */
void *gatehwspinlock_create(enum igatempsupport_local_protect local_protect,
				const struct gatehwspinlock_params *params)
{
	void *handle = NULL;
	struct gatehwspinlock_object *obj = NULL;
	s32 retval = 0;

	BUG_ON(params == NULL);
	if (atomic_cmpmask_and_lt(&(gatehwspinlock_module->ref_count),
					GATEHWSPINLOCK_MAKE_MAGICSTAMP(0),
				GATEHWSPINLOCK_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto exit;
	}

	if (WARN_ON(params->shared_addr == NULL)) {
		retval = -EINVAL;
		goto exit;
	}

	obj = kmalloc(sizeof(struct gatehwspinlock_object), GFP_KERNEL);
	if (obj == NULL) {
		retval = -ENOMEM;
		goto exit;
	}

	/* Create the local gate */
	obj->local_gate = gatemp_create_local(local_protect);
	if (obj->local_gate == NULL) {
		retval = GATEHWSPINLOCK_E_FAIL;
		goto exit;
	}

	obj->lock_num = params->resource_id;
	obj->nested   = 0;

	handle = obj;
	return handle;

exit:
	printk(KERN_ERR "gatehwspinlock_create failed status: %x\n", retval);
	return NULL;
}
EXPORT_SYMBOL(gatehwspinlock_create);

/*
 * ======== gatehwspinlock_delete ========
 *  Purpose:
 *  This will deletes an instance of gatehwspinlock module
 */
int gatehwspinlock_delete(void **gphandle)

{
	struct gatehwspinlock_object *obj = NULL;
	s32 retval;

	BUG_ON(gphandle == NULL);
	BUG_ON(*gphandle == NULL);
	if (atomic_cmpmask_and_lt(&(gatehwspinlock_module->ref_count),
				GATEHWSPINLOCK_MAKE_MAGICSTAMP(0),
				GATEHWSPINLOCK_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto exit;
	}

	obj = (struct gatehwspinlock_object *)(*gphandle);
	if (unlikely(obj == NULL)) {
		retval = -EINVAL;
		goto exit;
	}

	retval = gatemp_delete(&obj->local_gate);

	if (unlikely(retval < 0)) {
		retval = GATEHWSPINLOCK_E_FAIL;
		goto exit;
	}

	kfree(obj);
	*gphandle = 0;

	return 0;

exit:
	printk(KERN_ERR "gatehwspinlock_delete failed status: %x\n", retval);
	return retval;
}
EXPORT_SYMBOL(gatehwspinlock_delete);


/*
 * ======== gatehwspinlock_enter ========
 *  Purpose:
 *  This will enters the gatehwspinlock instance
 */
int *gatehwspinlock_enter(void *gphandle)
{
	struct gatehwspinlock_object *obj = NULL;
	s32 retval = 0;
	int *key = 0;
	VOLATILE u32 *base_addr = (VOLATILE u32 *)
					gatehwspinlock_module->base_addr;

	BUG_ON(gphandle == NULL);
	if (WARN_ON(atomic_cmpmask_and_lt(&(gatehwspinlock_module->ref_count),
				GATEHWSPINLOCK_MAKE_MAGICSTAMP(0),
				GATEHWSPINLOCK_MAKE_MAGICSTAMP(1)) == true)) {
		retval = -ENODEV;
		goto exit;
	}

	obj = (struct gatehwspinlock_object *)gphandle;

	/* Enter local gate */
	key = igateprovider_enter(obj->local_gate);

	/* If the gate object has already been entered, return the
							nested value */
	obj->nested++;
	if (obj->nested > 1)
		return key;

	/* Enter the spinlock */
	while (1) {
		if (base_addr[obj->lock_num] == 0)
			break;
	}

	return key;

exit:
	return 0;
}
EXPORT_SYMBOL(gatehwspinlock_enter);

/*
 * ======== gatehwspinlock_leave ========
 *  Purpose:
 *  This will leaves the gatehwspinlock instance
 */
void gatehwspinlock_leave(void *gphandle, int *key)
{
	struct gatehwspinlock_object *obj = NULL;
	VOLATILE u32 *base_addr = (VOLATILE u32 *)
					gatehwspinlock_module->base_addr;

	if (WARN_ON(atomic_cmpmask_and_lt(&(gatehwspinlock_module->ref_count),
				GATEHWSPINLOCK_MAKE_MAGICSTAMP(0),
				GATEHWSPINLOCK_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	BUG_ON(gphandle == NULL);

	obj = (struct gatehwspinlock_object *)gphandle;

	obj->nested--;

	/* Leave the spinlock if the leave() is not nested */
	if (obj->nested == 0)
		base_addr[obj->lock_num] = 0;

	igateprovider_leave(obj->local_gate, key);
exit:
	return;
}
EXPORT_SYMBOL(gatehwspinlock_leave);

/*
 *  ======== gatehwspinlock_get_resource_id ========
 */
u32 gatehwspinlock_get_resource_id(void *handle)
{
	struct gatehwspinlock_object *obj = NULL;
	s32 retval = 0;

	if (WARN_ON(atomic_cmpmask_and_lt(&(gatehwspinlock_module->ref_count),
				GATEHWSPINLOCK_MAKE_MAGICSTAMP(0),
				GATEHWSPINLOCK_MAKE_MAGICSTAMP(1)) == true)) {
		retval = -ENODEV;
		goto exit;
	}

	BUG_ON(handle == NULL);

	if (handle == NULL) {
		retval = -EINVAL;
		goto exit;
	}

	obj = (struct gatehwspinlock_object *)handle;

	return obj->lock_num;
exit:
	printk(KERN_ERR "gatehwspinlock_delete failed status: %x\n", retval);
	return (u32)-1;
}
/*
 * ======== gatehwspinlock_shared_memreq ========
 *  Purpose:
 *  This will give the amount of shared memory required
 *  for creation of each instance
 */
u32 gatehwspinlock_shared_mem_req(const struct igatempsupport_params *params)
{
	return 0;
}
EXPORT_SYMBOL(gatehwspinlock_shared_mem_req);
