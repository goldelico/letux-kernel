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
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/delay.h>

/* Module level headers */
#include "../procdefs.h"
#include "../processor.h"
#include <procmgr.h>
#include "../procmgr_drvdefs.h"
#include "proc4430.h"
#include "dmm4430.h"
#include <syslink/multiproc.h>
#include <syslink/ducatienabler.h>
#include <syslink/platform_mem.h>
#include <syslink/atomic_linux.h>

#define DUCATI_DMM_START_ADDR			0xa0000000
#define DUCATI_DMM_POOL_SIZE			0x6000000

#define SYS_M3					2
#define APP_M3					3
#define CORE_PRM_BASE				OMAP2_L4_IO_ADDRESS(0x4a306700)
#define CORE_CM2_DUCATI_CLKSTCTRL               OMAP2_L4_IO_ADDRESS(0x4A008900)
#define CORE_CM2_DUCATI_CLKCTRL			OMAP2_L4_IO_ADDRESS(0x4A008920)
#define RM_MPU_M3_RSTCTRL_OFFSET		0x210
#define RM_MPU_M3_RSTST_OFFSET			0x214
#define RM_MPU_M3_RST1				0x1
#define RM_MPU_M3_RST2				0x2
#define RM_MPU_M3_RST3				0x4

#define OMAP4430PROC_MODULEID          (u16) 0xbbec

/* Macro to make a correct module magic number with refCount */
#define OMAP4430PROC_MAKE_MAGICSTAMP(x) ((OMAP4430PROC_MODULEID << 12u) | (x))

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
	void *proc_handles[MULTIPROC_MAXPROCESSORS];
	/* Processor handle array. */
	struct mutex *gate_handle;
	/* void * of gate to be used for local thread safety */
	atomic_t ref_count;
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
	.config_size = sizeof(struct proc4430_config),
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
	atomic_cmpmask_and_set(&proc4430_state.ref_count,
			OMAP4430PROC_MAKE_MAGICSTAMP(0),
				 OMAP4430PROC_MAKE_MAGICSTAMP(0));

	if (atomic_inc_return(&proc4430_state.ref_count) !=
				OMAP4430PROC_MAKE_MAGICSTAMP(1)) {
		return 1;
	}

	if (cfg == NULL) {
		proc4430_get_config(&tmp_cfg);
		cfg = &tmp_cfg;
	}

	dmm_create();
	dmm_create_tables(DUCATI_DMM_START_ADDR, DUCATI_DMM_POOL_SIZE);

	/* Create a default gate handle for local module protection. */
	proc4430_state.gate_handle =
		kmalloc(sizeof(struct mutex), GFP_KERNEL);
	if (proc4430_state.gate_handle == NULL) {
		retval = -ENOMEM;
		goto error;
	}

	mutex_init(proc4430_state.gate_handle);

	/* Initialize the name to handles mapping array. */
	memset(&proc4430_state.proc_handles, 0,
		(sizeof(void *) * MULTIPROC_MAXPROCESSORS));

	/* Copy the user provided values into the state object. */
	memcpy(&proc4430_state.cfg, cfg,
				sizeof(struct proc4430_config));

	return 0;

error:
	atomic_dec_return(&proc4430_state.ref_count);
	dmm_delete_tables();
	dmm_destroy();

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

	if (atomic_cmpmask_and_lt(&proc4430_state.ref_count,
					OMAP4430PROC_MAKE_MAGICSTAMP(0),
					OMAP4430PROC_MAKE_MAGICSTAMP(1))
					== true) {
		retval = -ENODEV;
		goto exit;
	}
	if (!(atomic_dec_return(&proc4430_state.ref_count)
			== OMAP4430PROC_MAKE_MAGICSTAMP(0))) {

		retval = 1;
		goto exit;
	}

	/* Check if any OMAP4430 instances have not been
	 * deleted so far. If not,delete them.
	 */

	for (i = 0; i < MULTIPROC_MAXPROCESSORS; i++) {
		if (proc4430_state.proc_handles[i] == NULL)
			continue;
		proc4430_delete(&(proc4430_state.proc_handles[i]));
	}

	/* Check if the gate_handle was created internally. */
	if (proc4430_state.gate_handle != NULL) {
		mutex_destroy(proc4430_state.gate_handle);
		kfree(proc4430_state.gate_handle);
	}

exit:
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

	if (atomic_cmpmask_and_lt(&proc4430_state.ref_count,
					OMAP4430PROC_MAKE_MAGICSTAMP(0),
					OMAP4430PROC_MAKE_MAGICSTAMP(1))
					== true) {
		printk(KERN_ERR "proc4430_params_init failed "
			"Module not initialized");
		return;
	}

	if (WARN_ON(params == NULL)) {
		printk(KERN_ERR "proc4430_params_init failed "
			"Argument of type proc4430_params * "
			"is NULL");
		return;
	}

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

	if (atomic_cmpmask_and_lt(&proc4430_state.ref_count,
					OMAP4430PROC_MAKE_MAGICSTAMP(0),
					OMAP4430PROC_MAKE_MAGICSTAMP(1))
					== true) {
		printk(KERN_ERR "proc4430_create failed "
			"Module not initialized");
		goto error;
	}

	/* Enter critical section protection. */
	WARN_ON(mutex_lock_interruptible(proc4430_state.gate_handle));
	if (proc4430_state.proc_handles[proc_id] != NULL) {
		handle = proc4430_state.proc_handles[proc_id];
		goto func_end;
	} else {
		handle = (struct processor_object *)
			vmalloc(sizeof(struct processor_object));
		if (WARN_ON(handle == NULL))
			goto func_end;

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
		handle->proc_fxn_table.unmap = &proc4430_unmap;
		handle->proc_fxn_table.procinfo = &proc4430_proc_info;
		handle->proc_fxn_table.virt_to_phys = &proc4430_virt_to_phys;
		handle->state = PROC_MGR_STATE_UNKNOWN;
		handle->object = vmalloc(sizeof(struct proc4430_object));
		handle->proc_id = proc_id;
		object = (struct proc4430_object *)handle->object;
		if (params != NULL) {
			/* Copy params into instance object. */
			memcpy(&(object->params), (void *)params,
					sizeof(struct proc4430_params));
		}
		if ((params != NULL) && (params->mem_entries != NULL)
					&& (params->num_mem_entries > 0)) {
			/* Allocate memory for, and copy mem_entries table*/
			object->params.mem_entries = vmalloc(sizeof(struct
						proc4430_mem_entry) *
						params->num_mem_entries);
			memcpy(object->params.mem_entries,
				params->mem_entries,
				(sizeof(struct proc4430_mem_entry) *
				 params->num_mem_entries));
		}
		handle->boot_mode = PROC_MGR_BOOTMODE_NOLOAD;
		/* Set the handle in the state object. */
		proc4430_state.proc_handles[proc_id] = handle;
	}

func_end:
	mutex_unlock(proc4430_state.gate_handle);
error:
	return (void *)handle;
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

	if (atomic_cmpmask_and_lt(&proc4430_state.ref_count,
					OMAP4430PROC_MAKE_MAGICSTAMP(0),
					OMAP4430PROC_MAKE_MAGICSTAMP(1))
					== true) {
		printk(KERN_ERR "proc4430_delete failed "
			"Module not initialized");
		return -ENODEV;
	}

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

	if (atomic_cmpmask_and_lt(&proc4430_state.ref_count,
					OMAP4430PROC_MAKE_MAGICSTAMP(0),
					OMAP4430PROC_MAKE_MAGICSTAMP(1))
					== true) {
		printk(KERN_ERR "proc4430_open failed "
			"Module not initialized");
		return -ENODEV;
	}

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

	if (atomic_cmpmask_and_lt(&proc4430_state.ref_count,
					OMAP4430PROC_MAKE_MAGICSTAMP(0),
					OMAP4430PROC_MAKE_MAGICSTAMP(1))
					== true) {
		printk(KERN_ERR "proc4430_close failed "
			"Module not initialized");
		return -ENODEV;
	}

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

	struct processor_object *proc_handle = NULL;
	struct proc4430_object *object = NULL;
	u32 map_count = 0;
	u32 i;
	memory_map_info map_info;

	if (atomic_cmpmask_and_lt(&proc4430_state.ref_count,
					OMAP4430PROC_MAKE_MAGICSTAMP(0),
					OMAP4430PROC_MAKE_MAGICSTAMP(1))
					== true) {
		printk(KERN_ERR "proc4430_attach failed"
			"Module not initialized");
		return -ENODEV;
	}

	if (WARN_ON(handle == NULL)) {
		printk(KERN_ERR "proc4430_attach failed"
			"Driver handle is NULL");
		return -EINVAL;
	}

	if (WARN_ON(params == NULL)) {
		printk(KERN_ERR "proc4430_attach failed"
			"Argument processor_attach_params * is NULL");
		return -EINVAL;
	}

	proc_handle = (struct processor_object *)handle;

	object = (struct proc4430_object *)proc_handle->object;
	/* Return memory information in params. */
	for (i = 0; (i < object->params.num_mem_entries); i++) {
		/* If the configured master virtual address is invalid, get the
		* actual address by mapping the physical address into master
		* kernel memory space.
		*/
		if ((object->params.mem_entries[i].master_virt_addr == (u32)-1)
		&& (object->params.mem_entries[i].shared == true)) {
			map_info.src = object->params.mem_entries[i].phys_addr;
			map_info.size = object->params.mem_entries[i].size;
			map_info.is_cached = false;
			retval = platform_mem_map(&map_info);
			if (retval != 0) {
				printk(KERN_ERR "proc4430_attach failed\n");
				return -EFAULT;
			}
			map_count++;
			object->params.mem_entries[i].master_virt_addr =
								map_info.dst;
			params->mem_entries[i].addr
				[PROC_MGR_ADDRTYPE_MASTERKNLVIRT] =
								map_info.dst;
			params->mem_entries[i].addr
				[PROC_MGR_ADDRTYPE_SLAVEVIRT] =
			(object->params.mem_entries[i].slave_virt_addr);
			/* User virtual will be filled by user side. For now,
			fill in the physical address so that it can be used
			by mmap to remap this region into user-space */
			params->mem_entries[i].addr
				[PROC_MGR_ADDRTYPE_MASTERUSRVIRT] = \
				object->params.mem_entries[i].phys_addr;
			params->mem_entries[i].size =
				object->params.mem_entries[i].size;
		}
	}
	params->num_mem_entries = map_count;
	return retval;
}


/*==========================================
 * Function to detach from the Processor.
 *
 */
int proc4430_detach(void *handle)
{
	struct processor_object *proc_handle = NULL;
	struct proc4430_object *object = NULL;
	u32 i;
	memory_unmap_info unmap_info;

	if (atomic_cmpmask_and_lt(&proc4430_state.ref_count,
					OMAP4430PROC_MAKE_MAGICSTAMP(0),
					OMAP4430PROC_MAKE_MAGICSTAMP(1))
					== true) {

		printk(KERN_ERR "proc4430_detach failed "
			"Module not initialized");
		return -ENODEV;
	}

	if (WARN_ON(handle == NULL)) {
		printk(KERN_ERR "proc4430_detach failed "
			"Argument Driverhandle is NULL");
		return -EINVAL;
	}

	proc_handle = (struct processor_object *)handle;
	object = (struct proc4430_object *)proc_handle->object;
	for (i = 0; (i < object->params.num_mem_entries); i++) {
		if ((object->params.mem_entries[i].master_virt_addr > 0)
		    && (object->params.mem_entries[i].shared == true)) {
			unmap_info.addr =
				object->params.mem_entries[i].master_virt_addr;
			unmap_info.size = object->params.mem_entries[i].size;
			platform_mem_unmap(&unmap_info);
			object->params.mem_entries[i].master_virt_addr =
				(u32)-1;
		}
	}
	return 0;
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
			struct processor_start_params *start_params)
{
	u32 reg;
	int counter = 10;
	if (atomic_cmpmask_and_lt(&proc4430_state.ref_count,
					OMAP4430PROC_MAKE_MAGICSTAMP(0),
					OMAP4430PROC_MAKE_MAGICSTAMP(1))
					== true) {

		printk(KERN_ERR "proc4430_start failed "
			"Module not initialized");
		return -ENODEV;
	}

	/*FIXME: Remove handle and entry_pt if not used */
	if (WARN_ON(start_params == NULL)) {
		printk(KERN_ERR "proc4430_start failed "
			"Argument processor_start_params * is NULL");
		return -EINVAL;
	}

	reg = __raw_readl(CORE_PRM_BASE + RM_MPU_M3_RSTST_OFFSET);
	printk(KERN_INFO "proc4430_start: Reset Status [0x%x]", reg);
	reg = __raw_readl(CORE_PRM_BASE + RM_MPU_M3_RSTCTRL_OFFSET);
	printk(KERN_INFO "proc4430_start: Reset Control [0x%x]", reg);

	switch (start_params->params->proc_id) {
	case SYS_M3:
		/* Module is managed automatically by HW */
		__raw_writel(0x01, CORE_CM2_DUCATI_CLKCTRL);
		/* Enable the M3 clock */
		__raw_writel(0x02, CORE_CM2_DUCATI_CLKSTCTRL);
		do {
			reg = __raw_readl(CORE_CM2_DUCATI_CLKSTCTRL);
			if (reg & 0x100) {
				printk(KERN_INFO "M3 clock enabled:"
				"CORE_CM2_DUCATI_CLKSTCTRL = 0x%x\n", reg);
				break;
			}
			msleep(1);
		} while (--counter);
		if (counter == 0) {
			printk(KERN_ERR "FAILED TO ENABLE DUCATI M3 CLOCK !\n");
			return -EFAULT;
		}
		/* Check that releasing resets would indeed be effective */
		reg = __raw_readl(CORE_PRM_BASE + RM_MPU_M3_RSTCTRL_OFFSET);
		if (reg != 7) {
			printk(KERN_ERR "proc4430_start: Resets in not proper state!\n");
			__raw_writel(0x7,
				CORE_PRM_BASE + RM_MPU_M3_RSTCTRL_OFFSET);
		}

		/* De-assert RST3, and clear the Reset status */
		printk(KERN_INFO "De-assert RST3\n");
		__raw_writel(0x3, CORE_PRM_BASE + RM_MPU_M3_RSTCTRL_OFFSET);
		while (!(__raw_readl(CORE_PRM_BASE + RM_MPU_M3_RSTST_OFFSET)
			& 0x4))
			;
		printk(KERN_INFO "RST3 released!");
		__raw_writel(0x4, CORE_PRM_BASE + RM_MPU_M3_RSTST_OFFSET);
		ducati_setup();

		/* De-assert RST1, and clear the Reset status */
		printk(KERN_INFO "De-assert RST1\n");
		__raw_writel(0x2, CORE_PRM_BASE + RM_MPU_M3_RSTCTRL_OFFSET);
		while (!(__raw_readl(CORE_PRM_BASE + RM_MPU_M3_RSTST_OFFSET)
			& 0x1))
			;
		printk(KERN_INFO "RST1 released!");
		__raw_writel(0x1, CORE_PRM_BASE + RM_MPU_M3_RSTST_OFFSET);
		break;
	case APP_M3:
		/* De-assert RST2, and clear the Reset status */
		printk(KERN_INFO "De-assert RST2\n");
		__raw_writel(0x0, CORE_PRM_BASE + RM_MPU_M3_RSTCTRL_OFFSET);
		while (!(__raw_readl(CORE_PRM_BASE + RM_MPU_M3_RSTST_OFFSET)
			& 0x2))
			;
		printk(KERN_INFO "RST2 released!");
		__raw_writel(0x2, CORE_PRM_BASE + RM_MPU_M3_RSTST_OFFSET);
		break;
	default:
		printk(KERN_ERR "proc4430_start: ERROR input\n");
		break;
	}
	return 0;
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
proc4430_stop(void *handle, struct processor_stop_params *stop_params)
{
	if (atomic_cmpmask_and_lt(&proc4430_state.ref_count,
					OMAP4430PROC_MAKE_MAGICSTAMP(0),
					OMAP4430PROC_MAKE_MAGICSTAMP(1))
					== true) {
		printk(KERN_ERR "proc4430_stop failed "
			"Module not initialized");
		return -ENODEV;
	}
	switch (stop_params->params->proc_id) {
	case SYS_M3:
		ducati_destroy();
		printk(KERN_INFO "Assert RST1 and RST2 and RST3\n");
		__raw_writel(0x7, CORE_PRM_BASE + RM_MPU_M3_RSTCTRL_OFFSET);
		/* Disable the M3 clock */
		__raw_writel(0x01, CORE_CM2_DUCATI_CLKSTCTRL);
		break;
	case APP_M3:
		printk(KERN_INFO "Assert RST2\n");
		__raw_writel(0x2, CORE_PRM_BASE + RM_MPU_M3_RSTCTRL_OFFSET);
		break;
	default:
		printk(KERN_ERR "proc4430_stop: ERROR input\n");
		break;
	}
	return 0;
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
	if (atomic_cmpmask_and_lt(&proc4430_state.ref_count,
					OMAP4430PROC_MAKE_MAGICSTAMP(0),
					OMAP4430PROC_MAKE_MAGICSTAMP(1))
					== true) {
		printk(KERN_ERR "proc4430_read failed "
			"Module not initialized");
		return -ENODEV;
	}

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

	if (atomic_cmpmask_and_lt(&proc4430_state.ref_count,
					OMAP4430PROC_MAKE_MAGICSTAMP(0),
					OMAP4430PROC_MAKE_MAGICSTAMP(1))
					== true) {
		printk(KERN_ERR "proc4430_write failed "
			"Module not initialized");
		return -ENODEV;
	}

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

	/*FIXME: Remove handle,etc if not used */

	if (atomic_cmpmask_and_lt(&proc4430_state.ref_count,
					OMAP4430PROC_MAKE_MAGICSTAMP(0),
					OMAP4430PROC_MAKE_MAGICSTAMP(1))
					== true) {
		printk(KERN_ERR "proc4430_control failed "
			"Module not initialized");
		return -ENODEV;
	}

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
	struct processor_object *proc_handle = NULL;
	struct proc4430_object *object = NULL;
	struct proc4430_mem_entry *entry = NULL;
	bool found = false;
	u32 fm_addr_base = (u32)NULL;
	u32 to_addr_base = (u32)NULL;
	u32 i;

	if (atomic_cmpmask_and_lt(&proc4430_state.ref_count,
					OMAP4430PROC_MAKE_MAGICSTAMP(0),
					OMAP4430PROC_MAKE_MAGICSTAMP(1))
					== true) {
		printk(KERN_ERR "proc4430_translate_addr failed "
			"Module not initialized");
		retval = -ENODEV;
		goto error_exit;
	}

	if (WARN_ON(handle == NULL)) {
		retval = -EINVAL;
		goto error_exit;
	}
	if (WARN_ON(dst_addr == NULL)) {
		retval = -EINVAL;
		goto error_exit;
	}
	if (WARN_ON(dst_addr_type > PROC_MGR_ADDRTYPE_ENDVALUE)) {
		retval = -EINVAL;
		goto error_exit;
	}
	if (WARN_ON(src_addr == NULL)) {
		retval = -EINVAL;
		goto error_exit;
	}
	if (WARN_ON(src_addr_type > PROC_MGR_ADDRTYPE_ENDVALUE)) {
		retval = -EINVAL;
		goto error_exit;
	}

	proc_handle = (struct processor_object *)handle;
	object = (struct proc4430_object *)proc_handle->object;
	*dst_addr = NULL;
	for (i = 0 ; i < object->params.num_mem_entries ; i++) {
		entry = &(object->params.mem_entries[i]);
		fm_addr_base =
			(src_addr_type == PROC_MGR_ADDRTYPE_MASTERKNLVIRT) ?
			entry->master_virt_addr : entry->slave_virt_addr;
		to_addr_base =
			(dst_addr_type == PROC_MGR_ADDRTYPE_MASTERKNLVIRT) ?
			entry->master_virt_addr : entry->slave_virt_addr;
		/* Determine whether which way to convert */
		if (((u32)src_addr < (fm_addr_base + entry->size)) &&
			((u32)src_addr >= fm_addr_base)) {
			found = true;
			*dst_addr = (void *)(((u32)src_addr - fm_addr_base)
				+ to_addr_base);
			break;
		}
	}

	/* This check must not be removed even with build optimize. */
	if (WARN_ON(found == false)) {
		/*Failed to translate address. */
		retval = -ENXIO;
		goto error_exit;
	}
	return 0;

error_exit:
	return retval;
}


/*=================================================
 * Function to map slave address to host address space
 *
 * Map the provided slave address to master address space. This
 * function also maps the specified address to slave MMU space.
 */
int proc4430_map(void *handle, u32 proc_addr,
	u32 size, u32 *mapped_addr, u32 *mapped_size, u32 map_attribs)
{
	int retval = 0;
	u32 da_align;
	u32 da;
	u32 va_align;
	u32 size_align;

	if (atomic_cmpmask_and_lt(&proc4430_state.ref_count,
					OMAP4430PROC_MAKE_MAGICSTAMP(0),
					OMAP4430PROC_MAKE_MAGICSTAMP(1))
					== true) {
		printk(KERN_ERR "proc4430_map failed "
			"Module not initialized");
		retval = -ENODEV;
		goto error_exit;
	}

	/*FIXME: Remove handle,etc if not used */

	/* FIX ME: Temporary work around until the dynamic memory mapping
	  * for Tiler address space is available
	  */
	if ((map_attribs & DSP_MAPTILERADDR)) {
		da_align = user_va2pa(current->mm, proc_addr);
		*mapped_addr = (da_align | (proc_addr & (PAGE_SIZE - 1)));
		printk(KERN_INFO "proc4430_map -tiler: user_va2pa: mapped_addr"
			"= 0x%x\n", *mapped_addr);
		return 0;
	}

	/* Calculate the page-aligned PA, VA and size */
	va_align = PG_ALIGN_LOW(proc_addr, PAGE_SIZE);
	size_align = PG_ALIGN_HIGH(size + (u32)proc_addr - va_align, PAGE_SIZE);

	dmm_reserve_memory(size_align, &da);
	da_align = PG_ALIGN_LOW((u32)da, PAGE_SIZE);
	retval = ducati_mem_map(va_align, da_align, size_align, map_attribs);

	/* Mapped address = MSB of DA | LSB of VA */
	*mapped_addr = (da_align | (proc_addr & (PAGE_SIZE - 1)));

error_exit:
	return retval;
}

/*=================================================
 * Function to unmap slave address to host address space
 *
 * UnMap the provided slave address to master address space. This
 * function also unmaps the specified address to slave MMU space.
 */
int proc4430_unmap(void *handle, u32 mapped_addr)
{
	int da_align;
	int ret_val = 0;
	int size_align;

	if (atomic_cmpmask_and_lt(&proc4430_state.ref_count,
					OMAP4430PROC_MAKE_MAGICSTAMP(0),
					OMAP4430PROC_MAKE_MAGICSTAMP(1))
					== true) {
		printk(KERN_ERR "proc4430_map failed "
			"Module not initialized");
		ret_val = -1;
		goto error_exit;
	}

	/*FIXME: Remove handle,etc if not used */

	da_align = PG_ALIGN_LOW((u32)mapped_addr, PAGE_SIZE);
	ret_val = dmm_unreserve_memory(da_align, &size_align);
	if (WARN_ON(ret_val < 0))
		goto error_exit;
	ret_val = ducati_mem_unmap(da_align, size_align);
	if (WARN_ON(ret_val < 0))
		goto error_exit;
	return 0;

error_exit:
	printk(KERN_WARNING "proc4430_unmap failed !!!!\n");
	return ret_val;
}

/*=================================================
 * Function to return list of translated mem entries
 *
 * This function takes the remote processor address as
 * an input and returns the mapped Page entries in the
 * buffer passed
 */
int proc4430_virt_to_phys(void *handle, u32 da, u32 *mapped_entries,
						u32 num_of_entries)
{
	int da_align;
	int i;
	int ret_val = 0;

	if (atomic_cmpmask_and_lt(&proc4430_state.ref_count,
					OMAP4430PROC_MAKE_MAGICSTAMP(0),
					OMAP4430PROC_MAKE_MAGICSTAMP(1))
					== true) {
		printk(KERN_ERR "proc4430_virt_to_phys failed "
			"Module not initialized");
		ret_val = -EFAULT;
		goto error_exit;
	}

	if (handle == NULL || mapped_entries == NULL || num_of_entries == 0) {
		ret_val = -EFAULT;
		goto error_exit;
	}
	da_align = PG_ALIGN_LOW((u32)da, PAGE_SIZE);
	for (i = 0; i < num_of_entries; i++) {
		mapped_entries[i] = ducati_mem_virtToPhys(da_align);
		da_align += PAGE_SIZE;
	}
	return 0;

error_exit:
	printk(KERN_WARNING "proc4430_virtToPhys failed !!!!\n");
	return ret_val;
}


/*=================================================
 * Function to return PROC4430 mem_entries info
 *
 */
int proc4430_proc_info(void *handle, struct proc_mgr_proc_info *procinfo)
{
	struct processor_object *proc_handle = NULL;
	struct proc4430_object *object = NULL;
	struct proc4430_mem_entry *entry = NULL;
	int i;

	if (atomic_cmpmask_and_lt(&proc4430_state.ref_count,
					OMAP4430PROC_MAKE_MAGICSTAMP(0),
					OMAP4430PROC_MAKE_MAGICSTAMP(1))
					== true) {
		printk(KERN_ERR "proc4430_proc_info failed "
			"Module not initialized");
		goto error_exit;
	}

	if (WARN_ON(handle == NULL))
		goto error_exit;
	if (WARN_ON(procinfo == NULL))
		goto error_exit;

	proc_handle = (struct processor_object *)handle;

	object = (struct proc4430_object *)proc_handle->object;

	for (i = 0 ; i < object->params.num_mem_entries ; i++) {
		entry = &(object->params.mem_entries[i]);
		procinfo->mem_entries[i].addr[PROC_MGR_ADDRTYPE_MASTERKNLVIRT]
						= entry->master_virt_addr;
		procinfo->mem_entries[i].addr[PROC_MGR_ADDRTYPE_SLAVEVIRT]
						= entry->slave_virt_addr;
		procinfo->mem_entries[i].size = entry->size;
	}
	procinfo->num_mem_entries = object->params.num_mem_entries;
	procinfo->boot_mode = proc_handle->boot_mode;
	return 0;

error_exit:
	printk(KERN_WARNING "proc4430_proc_info failed !!!!\n");
	return -EFAULT;
}
