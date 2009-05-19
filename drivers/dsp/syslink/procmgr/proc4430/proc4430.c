/*
 * proc4430.c
 *
 * Syslink driver support functions for TI OMAP processors.
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

#include <linux/types.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/vmalloc.h>

/* Module level headers */
#include "../procdefs.h"
#include "../processor.h"
#include "../procmgr.h"
#include "../procmgr_drvdefs.h"
#include "proc4430.h"

#include <syslink/multiproc.h>
#include <syslink/ducatienabler.h>

/*OMAP4430 Module state object */
struct proc4430_module_object {
	u32 config_size;
	/* Size of configuration structure */
	struct proc4430_config cfg;
	/* OMAP4430 configuration structure */
	struct proc4430_config def_cfg;
	/* Default module configuration */
	struct proc4430_params def_inst_params;
	/* Default parameters for the OMAP4430 instances */
	bool is_setup;
	/* Indicates whether the OMAP4430 module is setup. */
	void *proc_handles[MULTIPROC_MAXPROCESSORS];
	/* Processor handle array. */
	struct mutex *gate_handle;
	/* void * of gate to be used for local thread safety */
};

/*
  OMAP4430 instance object.
 */
struct proc4430_object {
	struct proc4430_params params;
	/* Instance parameters (configuration values) */
};


/* =================================
 *  Globals
 * =================================
 */
/*
  OMAP4430 state object variable
 */

static struct proc4430_module_object proc4430_state = {
	.is_setup = false,
	.config_size = sizeof(struct proc4430_config),
	.def_cfg.gate_handle = NULL,
	.gate_handle = NULL,
	.def_inst_params.num_mem_entries = 0u,
	.def_inst_params.mem_entries = NULL,
	.def_inst_params.reset_vector_mem_entry = 0
};


/* =================================
 * APIs directly called by applications
 * =================================
 */
/*
 * Function to get the default configuration for the OMAP4430
 * module.
 *
 * This function can be called by the application to get their
 * configuration parameter to proc4430_setup filled in by the
 * OMAP4430 module with the default parameters. If the user
 * does not wish to make any change in the default parameters, this
 * API is not required to be called.
 */
void proc4430_get_config(struct proc4430_config *cfg)
{
	BUG_ON(cfg == NULL);
	memcpy(cfg, &(proc4430_state.def_cfg),
			sizeof(struct proc4430_config));
}
EXPORT_SYMBOL(proc4430_get_config);

/*
 * Function to setup the OMAP4430 module.
 *
 * This function sets up the OMAP4430 module. This function
 * must be called before any other instance-level APIs can be
 * invoked.
 * Module-level configuration needs to be provided to this
 * function. If the user wishes to change some specific config
 * parameters, then proc4430_get_config can be called to get the
 * configuration filled with the default values. After this, only
 * the required configuration values can be changed. If the user
 * does not wish to make any change in the default parameters, the
 * application can simply call proc4430_setup with NULL
 * parameters. The default parameters would get automatically used.
 */
int proc4430_setup(struct proc4430_config *cfg)
{
	int retval = 0;
	struct proc4430_config tmp_cfg;

	if (cfg == NULL) {
		proc4430_get_config(&tmp_cfg);
		cfg = &tmp_cfg;
	}

	if (cfg->gate_handle != NULL) {
		proc4430_state.gate_handle = cfg->gate_handle;
	} else {
		/* User has not provided any gate handle, so create a
		* default handle. */
		proc4430_state.gate_handle = kmalloc(sizeof(struct mutex),
						GFP_KERNEL);
		mutex_init(proc4430_state.gate_handle);
		ducati_setup();
	}
	/* Copy the user provided values into the state object. */
	memcpy(&proc4430_state.cfg, cfg,
				sizeof(struct proc4430_config));
	/* Initialize the name to handles mapping array. */
	memset(&proc4430_state.proc_handles, 0,
			(sizeof(void *) * MULTIPROC_MAXPROCESSORS));
	return retval;
}
EXPORT_SYMBOL(proc4430_setup);

/*
 * Function to destroy the OMAP4430 module.
 *
 * Once this function is called, other OMAP4430 module APIs,
 * except for the proc4430_get_config API cannot be called
 * anymore.
 */
int proc4430_destroy(void)
{
	int retval = 0;
	u16 i;

	/* Check if any OMAP4430 instances have not been deleted so far. If not,
	 * delete them.
	 */
	for (i = 0; i < MULTIPROC_MAXPROCESSORS; i++) {
		BUG_ON(proc4430_state.proc_handles[i] == NULL);
		if (proc4430_state.proc_handles[i] != NULL)
			proc4430_delete(&(proc4430_state.proc_handles[i]));
	}

	/* Check if the gate_handle was created internally. */
	if (proc4430_state.cfg.gate_handle == NULL) {
		if (proc4430_state.gate_handle != NULL) {
			mutex_destroy(proc4430_state.gate_handle);
			kfree(proc4430_state.gate_handle);
		}
	}
	ducati_destroy();
	proc4430_state.is_setup = false;
	return retval;
}
EXPORT_SYMBOL(proc4430_destroy);

/*=================================================
 * Function to initialize the parameters for this Processor
 * instance.
 */
void proc4430_params_init(void *handle, struct proc4430_params *params)
{
	struct proc4430_object *proc_object = (struct proc4430_object *)handle;

	BUG_ON(params == NULL);
	if (handle == NULL)
		memcpy(params, &(proc4430_state.def_inst_params),
				sizeof(struct proc4430_params));
	else
		memcpy(params, &(proc_object->params),
				sizeof(struct proc4430_params));
}
EXPORT_SYMBOL(proc4430_params_init);

/*===================================================
 *Function to create an instance of this Processor.
 *
 */
void *proc4430_create(u16 proc_id, const struct proc4430_params *params)
{
	struct processor_object *handle = NULL;
	struct proc4430_object *object = NULL;


	BUG_ON(!IS_VALID_PROCID(proc_id));
	BUG_ON(params == NULL);

	/* Enter critical section protection. */
	WARN_ON(mutex_lock_interruptible(proc4430_state.gate_handle));
	if (proc4430_state.proc_handles[proc_id] != NULL) {
		printk(KERN_WARNING "Processor already exists for specified"
			"%d  proc_id\n", proc_id);
		WARN_ON(1);
		goto func_end;
	} else {
		handle = (struct processor_object *)
			vmalloc(sizeof(struct processor_object));
		if (handle == NULL) {
			handle->proc_fxn_table.attach = &proc4430_attach;
			handle->proc_fxn_table.detach = &proc4430_detach;
			handle->proc_fxn_table.start = &proc4430_start;
			handle->proc_fxn_table.stop = &proc4430_stop;
			handle->proc_fxn_table.read = &proc4430_read;
			handle->proc_fxn_table.write = &proc4430_write;
			handle->proc_fxn_table.control = &proc4430_control;
			handle->proc_fxn_table.translateAddr =
						 &proc4430_translate_addr;
			handle->proc_fxn_table.map = &proc4430_map;
			handle->state = PROC_MGR_STATE_UNKNOWN;
			handle->object = vmalloc
					(sizeof(struct proc4430_object));

			handle->proc_id = proc_id;
			object = (struct proc4430_object *)handle->object;
			/* Copy params into instance object. */
			memcpy(&(object->params), (void *)params,
				sizeof(struct proc4430_object));

			/* Allocate memory for, and copy memEntries table*/
			object->params.mem_entries = vmalloc(
			sizeof(struct proc4430_mem_entry) *
			params->num_mem_entries);
			memcpy(object->params.mem_entries,
				params->mem_entries,
				(sizeof(struct proc4430_mem_entry)
				* params->num_mem_entries));
			/* Set the handle in the state object. */
			proc4430_state.proc_handles[proc_id] = handle;
		}
	}
func_end:
	mutex_unlock(proc4430_state.gate_handle);
	return handle;
}
EXPORT_SYMBOL(proc4430_create);

/*=================================================
 * Function to delete an instance of this Processor.
 *
 * The user provided pointer to the handle is reset after
 * successful completion of this function.
 *
 */
int proc4430_delete(void **handle_ptr)
{
	int retval = 0;
	struct proc4430_object *object = NULL;
	struct processor_object *handle;

	BUG_ON(handle_ptr == NULL);
	BUG_ON(*handle_ptr == NULL);

	handle = (struct processor_object *)(*handle_ptr);
	BUG_ON(!IS_VALID_PROCID(handle->proc_id));
	/* Enter critical section protection. */
	WARN_ON(mutex_lock_interruptible(proc4430_state.gate_handle));
	/* Reset handle in PwrMgr handle array. */
	proc4430_state.proc_handles[handle->proc_id] = NULL;
	/* Free memory used for the OMAP4430 object. */
	if (handle->object != NULL) {
		object = (struct proc4430_object *)handle->object;
		if (object->params.mem_entries != NULL) {
			vfree(object->params.mem_entries);
			object->params.mem_entries = NULL;
		}
		vfree(handle->object);
		handle->object = NULL;
	}
	/* Free memory used for the Processor object. */
	vfree(handle);
	*handle_ptr = NULL;
	/* Leave critical section protection. */
	mutex_unlock(proc4430_state.gate_handle);
	return retval;
}
EXPORT_SYMBOL(proc4430_delete);

/*===================================================
 * Function to open a handle to an instance of this Processor. This
 * function is called when access to the Processor is required from
 * a different process.
 */
int proc4430_open(void **handle_ptr, u16 proc_id)
{
	int retval = 0;

	BUG_ON(handle_ptr == NULL);
	BUG_ON(!IS_VALID_PROCID(proc_id));

	/* Initialize return parameter handle. */
	*handle_ptr = NULL;

	/* Check if the PwrMgr exists and return the handle if found. */
	if (proc4430_state.proc_handles[proc_id] == NULL) {
		retval = -ENODEV;
		goto func_exit;
	} else
		*handle_ptr = proc4430_state.proc_handles[proc_id];
func_exit:
	return retval;
}
EXPORT_SYMBOL(proc4430_open);

/*===============================================
 * Function to close a handle to an instance of this Processor.
 *
 */
int proc4430_close(void *handle)
{
	int retval = 0;

	BUG_ON(handle == NULL);
	/* nothing to be done for now */
	return retval;
}
EXPORT_SYMBOL(proc4430_close);

/* =================================
 * APIs called by Processor module (part of function table interface)
 * =================================
 */
/*================================
 * Function to initialize the slave processor
 *
 */
int proc4430_attach(void *handle, struct processor_attach_params *params)
{
	int retval = 0;
	/* TODO */
	return retval;
}


/*==========================================
 * Function to detach from the Processor.
 *
 */
int proc4430_detach(void *handle)
{
	int retval = 0;

	/* TODO */
	return retval;
}


/*==========================================
 * Function to start the slave processor
 *
 * Start the slave processor running from its entry point.
 * Depending on the boot mode, this involves configuring the boot
 * address and releasing the slave from reset.
 *
 */
int proc4430_start(void *handle, u32 entry_pt,
			struct processor_start_params *params)
{
	int retval = 0;
	/*TODO */
	return retval;
}


/*
 * Function to stop the slave processor
 *
 * Stop the execution of the slave processor. Depending on the boot
 * mode, this may result in placing the slave processor in reset.
 *
 *  @param	  handle	void * to the Processor instance
 *
 *  @sa		 proc4430_start, OMAP3530_halResetCtrl
 */
int
proc4430_stop(void *handle)
{
	int retval = 0;

	BUG_ON(handle == NULL);

	/* TODO*/
	return retval;
}


/*==============================================
 *	 Function to read from the slave processor's memory.
 *
 * Read from the slave processor's memory and copy into the
 * provided buffer.
 */
int proc4430_read(void *handle, u32 proc_addr, u32 *num_bytes,
						void *buffer)
{
	int retval = 0;
	/* TODO */
	return retval;
}


/*==============================================
 * Function to write into the slave processor's memory.
 *
 * Read from the provided buffer and copy into the slave
 * processor's memory.
 *
 */
int proc4430_write(void *handle, u32 proc_addr, u32 *num_bytes,
						void *buffer)
{
	int retval = 0;

	/* TODO */
	return retval;
}


/*=========================================================
 * Function to perform device-dependent operations.
 *
 * Performs device-dependent control operations as exposed by this
 * implementation of the Processor module.
 */
int proc4430_control(void *handle, int cmd, void *arg)
{
	int retval = 0;


	return retval;
}


/*=====================================================
 * Function to translate between two types of address spaces.
 *
 * Translate between the specified address spaces.
 */
int proc4430_translate_addr(void *handle,
		void **dst_addr, enum proc_mgr_addr_type dst_addr_type,
		void *src_addr, enum proc_mgr_addr_type src_addr_type)
{
	int retval = 0;
	/* TODO */
	return retval;
}


/*=================================================
 * Function to map slave address to host address space
 *
 * Map the provided slave address to master address space. This
 * function also maps the specified address to slave MMU space.
 */
int proc4430_map(void *handle, u32 proc_addr,
	u32 size, u32 *mapped_addr, u32 *mapped_size)
{
	int retval = 0;
	/* TODO */
	return retval;
}
