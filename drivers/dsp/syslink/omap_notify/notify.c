/*
 * notify.c
 *
 * Syslink driver support for OMAP Processors.
 *
 * Copyright (C) 2008-2009 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <asm/pgtable.h>

#include <syslink/notify.h>
#include <syslink/notify_driver.h>
#include <syslink/GlobalTypes.h>
#include <syslink/gt.h>
#include <syslink/multiproc.h>
#include <syslink/atomic_linux.h>

/*
 * func   _notify_is_support_proc
 *
 *desc   Check if specified processor ID is supported by the Notify driver.
 *
 */
static bool _notify_is_support_proc(struct notify_driver_object *drv_handle,
							int proc_id);

struct notify_module_object notify_state = {
	.def_cfg.maxDrivers = 2,
};
EXPORT_SYMBOL(notify_state);


/*
 * Get the default configuration for the Notify module.
 *
 *  This function can be called by the application to get their
 *  configuration parameter to Notify_setup filled in by the
 *  Notify module with the default parameters. If the user
 *  does not wish to make any change in the default parameters, this
 *  API is not required to be called.
 *
 * param-cfg  :Pointer to the Notify module configuration
 * structure in which the default config is to be returned.
 */
void notify_get_config(struct notify_config *cfg)
{
	BUG_ON(cfg == NULL);

	if (atomic_cmpmask_and_lt(&(notify_state.ref_count),
			NOTIFY_MAKE_MAGICSTAMP(0),
			NOTIFY_MAKE_MAGICSTAMP(1)) == true)
		memcpy(cfg, &notify_state.def_cfg,
			sizeof(struct notify_config));
	else
		memcpy(cfg, &notify_state.cfg, sizeof(struct notify_config));
}
EXPORT_SYMBOL(notify_get_config);

/*
 *  Setup the Notify module.
 *
 * This function sets up the Notify module. This function
 * must be called before any other instance-level APIs can be
 * invoked.
 * Module-level configuration needs to be provided to this
 * function. If the user wishes to change some specific config
 * parameters, then Notify_getConfig can be called to get the
 * configuration filled with the default values. After this, only
 * the required configuration values can be changed. If the user
 * does not wish to make any change in the default parameters, the
 * application can simply call Notify_setup with NULL
 * parameters. The default parameters would get automatically used.
 *
 * param -cfg   Optional Notify module configuration. If provided as
 *       NULL, default configuration is used.
 */
int notify_setup(struct notify_config *cfg)
{
	int status = NOTIFY_SUCCESS;
	struct notify_config tmpCfg;

	atomic_cmpmask_and_set(&notify_state.ref_count,
				    NOTIFY_MAKE_MAGICSTAMP(0),
				    NOTIFY_MAKE_MAGICSTAMP(0));

	if (atomic_inc_return(&notify_state.ref_count)
				!= NOTIFY_MAKE_MAGICSTAMP(1u)) {
		status = NOTIFY_S_ALREADYSETUP;
	} else {
		if (cfg == NULL) {
			notify_get_config(&tmpCfg);
			cfg = &tmpCfg;
		}

		notify_state.gate_handle = kmalloc(sizeof(struct mutex),
							GFP_ATOMIC);
		/*User has not provided any gate handle,
		so create a default handle.*/
		mutex_init(notify_state.gate_handle);

		if (WARN_ON(cfg->maxDrivers > NOTIFY_MAX_DRIVERS)) {
			status = NOTIFY_E_CONFIG;
			kfree(notify_state.gate_handle);
			atomic_set(&notify_state.ref_count,
				NOTIFY_MAKE_MAGICSTAMP(0));
			goto func_end;
		}
		memcpy(&notify_state.cfg, cfg, sizeof(struct notify_config));
		memset(&notify_state.drivers, 0,
			(sizeof(struct notify_driver_object) *
						NOTIFY_MAX_DRIVERS));

		notify_state.disable_depth = 0;

	}
func_end:
	return status;
}
EXPORT_SYMBOL(notify_setup);

/*
 * Destroy the Notify module.
 *
 * Once this function is called, other Notify module APIs,
 *  except for the Notify_getConfig API cannot be called
 *  anymore.
 */
int notify_destroy(void)
{
	int i;
	int status = NOTIFY_SUCCESS;

	if (atomic_cmpmask_and_lt(&(notify_state.ref_count),
			NOTIFY_MAKE_MAGICSTAMP(0),
			NOTIFY_MAKE_MAGICSTAMP(1)) == true) {
			status = NOTIFY_E_INVALIDSTATE;
	} else {
		if (atomic_dec_return(&(notify_state.ref_count)) ==
					NOTIFY_MAKE_MAGICSTAMP(0)) {

			/* Check if any Notify driver instances have
			 * not been deleted so far. If not, assert.
			 */
			for (i = 0; i < NOTIFY_MAX_DRIVERS; i++)
				WARN_ON(notify_state.drivers[i].is_init
								!= false);

			if (notify_state.gate_handle != NULL)
				kfree(notify_state.gate_handle);

			atomic_set(&notify_state.ref_count,
				NOTIFY_MAKE_MAGICSTAMP(0));
		}
	}
	return status;
}
EXPORT_SYMBOL(notify_destroy);

/*
 * func   notify_register_event
 *
 * desc   This function registers a callback for a specific event with the
 * Notify module.
 */
int notify_register_event(void *notify_driver_handle, u16 proc_id,
	u32 event_no, notify_callback_fxn notify_callback_fxn, void *cbck_arg)
{
	int status = NOTIFY_SUCCESS;

	struct notify_driver_object *drv_handle =
		(struct notify_driver_object *)notify_driver_handle;

	BUG_ON(drv_handle == NULL);

	if (atomic_cmpmask_and_lt(&(notify_state.ref_count),
			NOTIFY_MAKE_MAGICSTAMP(0),
			NOTIFY_MAKE_MAGICSTAMP(1)) == true) {
		status = NOTIFY_E_INVALIDSTATE;
		goto func_end;
	}
	if (WARN_ON(drv_handle->is_init == false)) {
		status = NOTIFY_E_INVALIDSTATE;
		goto func_end;
	}
	if (WARN_ON(_notify_is_support_proc(drv_handle, proc_id) != true)) {
		status = NOTIFY_E_INVALIDARG;
		goto func_end;
	}
	if (WARN_ON(((event_no & NOTIFY_EVENT_MASK)
		>= drv_handle->attrs.proc_info[proc_id].max_events))) {
		status = NOTIFY_E_INVALIDEVENT;
		goto func_end;
	}
	if (WARN_ON(((event_no & NOTIFY_EVENT_MASK) <
		drv_handle->attrs.proc_info[proc_id].reserved_events) &&
		((event_no & NOTIFY_SYSTEM_KEY_MASK) >> sizeof(u16)) !=
		NOTIFY_SYSTEM_KEY)) {
		status = NOTIFY_E_RESERVEDEVENT;
		goto func_end;
	}
	if (mutex_lock_interruptible(notify_state.gate_handle) != 0)
		WARN_ON(1);

	status = drv_handle->fn_table.register_event(drv_handle, proc_id,
			(event_no & NOTIFY_EVENT_MASK), notify_callback_fxn,
			cbck_arg);
	mutex_unlock(notify_state.gate_handle);
	if (WARN_ON(status < 0))
		status = NOTIFY_E_FAIL;
	else
		status = NOTIFY_SUCCESS;
func_end:
	return status;
}
EXPORT_SYMBOL(notify_register_event);

/*
 * func   notify_unregister_event
 *
 * desc   This function un-registers the callback for the specific event with
 * the Notify module.
 */

int notify_unregister_event(void *notify_driver_handle, u16 proc_id,
	u32 event_no, notify_callback_fxn notify_callback_fxn, void *cbck_arg)
{
	int status = NOTIFY_SUCCESS;
	struct notify_driver_object *drv_handle =
		(struct notify_driver_object *)notify_driver_handle;
	BUG_ON(drv_handle == NULL);

	if (atomic_cmpmask_and_lt(&(notify_state.ref_count),
			NOTIFY_MAKE_MAGICSTAMP(0),
			NOTIFY_MAKE_MAGICSTAMP(1)) == true) {
		status = NOTIFY_E_INVALIDSTATE;
		goto func_end;
	}
	if (WARN_ON(drv_handle->is_init == false)) {
		status = NOTIFY_E_INVALIDSTATE;
		goto func_end;
	}
	if (WARN_ON(_notify_is_support_proc(drv_handle, proc_id) != true)) {
		status = NOTIFY_E_INVALIDARG;
		goto func_end;
	}
	if (WARN_ON(((event_no & NOTIFY_EVENT_MASK)
		>= drv_handle->attrs.proc_info[proc_id].max_events))) {
		status = NOTIFY_E_INVALIDEVENT;
		goto func_end;
	}
	if (WARN_ON(((event_no & NOTIFY_EVENT_MASK) <
		drv_handle->attrs.proc_info[proc_id].reserved_events) &&
		((event_no & NOTIFY_SYSTEM_KEY_MASK) >> sizeof(u16)) !=
		NOTIFY_SYSTEM_KEY)) {
		status = NOTIFY_E_RESERVEDEVENT;
		goto func_end;
	}
	if (mutex_lock_interruptible(notify_state.gate_handle) != 0)
		WARN_ON(1);
	status = drv_handle->fn_table.unregister_event(drv_handle,
			proc_id, (event_no & NOTIFY_EVENT_MASK),
			notify_callback_fxn, cbck_arg);
	mutex_unlock(notify_state.gate_handle);
	if (WARN_ON(status < 0))
		status = NOTIFY_E_FAIL;
	else
		status = NOTIFY_SUCCESS;

func_end:
	return status;
}
EXPORT_SYMBOL(notify_unregister_event);

/*
 * func   notify_sendevent
 *
 * desc   This function sends a notification to the specified event.
 *
 *
 */
int notify_sendevent(void *notify_driver_handle, u16 proc_id,
				u32 event_no, u32 payload, bool wait_clear)
{
	int status = NOTIFY_SUCCESS;
	struct notify_driver_object *drv_handle =
		(struct notify_driver_object *)notify_driver_handle;
	BUG_ON(drv_handle == NULL);

	if (atomic_cmpmask_and_lt(&(notify_state.ref_count),
			NOTIFY_MAKE_MAGICSTAMP(0),
			NOTIFY_MAKE_MAGICSTAMP(1)) == true) {
		status = NOTIFY_E_INVALIDSTATE;
		goto func_end;
	}
	if (WARN_ON(drv_handle->is_init == false)) {
		status = NOTIFY_E_INVALIDSTATE;
		goto func_end;
	}
	if (WARN_ON(_notify_is_support_proc(drv_handle, proc_id) != true)) {
		status = NOTIFY_E_INVALIDARG;
		goto func_end;
	}
	if (WARN_ON(((event_no & NOTIFY_EVENT_MASK)
		>= drv_handle->attrs.proc_info[proc_id].max_events))) {
		status = NOTIFY_E_INVALIDEVENT;
		goto func_end;
	}
	if (WARN_ON(((event_no & NOTIFY_EVENT_MASK) <
		drv_handle->attrs.proc_info[proc_id].reserved_events) &&
		((event_no & NOTIFY_SYSTEM_KEY_MASK) >> sizeof(u16)) !=
		NOTIFY_SYSTEM_KEY)) {
		status = NOTIFY_E_RESERVEDEVENT;
		goto func_end;
	}
	if (mutex_lock_interruptible(notify_state.gate_handle) != 0)
			WARN_ON(1);
	status = drv_handle->fn_table.send_event
		(drv_handle, proc_id, (event_no & NOTIFY_EVENT_MASK),
		payload, wait_clear);
	mutex_unlock(notify_state.gate_handle);
	if (status < 0)
		status = NOTIFY_E_FAIL;
	else
		status = NOTIFY_SUCCESS;
func_end:
	return status;
}
EXPORT_SYMBOL(notify_sendevent);

/*
 * func   notify_disable
 *
 * desc   This function disables all events. This is equivalent to global
 *	interrupt disable, however restricted within interrupts handled by
 *	the Notify module. All callbacks registered for all events are
 *	disabled with this API. It is not possible to disable a specific
 *	callback.
 *
 */
u32 notify_disable(u16 proc_id)
{
	struct notify_driver_object *drv_handle;
	int i;

	if (atomic_cmpmask_and_lt(&(notify_state.ref_count),
			NOTIFY_MAKE_MAGICSTAMP(0),
			NOTIFY_MAKE_MAGICSTAMP(1)) == true)
			WARN_ON(1);


	if (mutex_lock_interruptible(notify_state.gate_handle) != 0)
		WARN_ON(1);

	for (i = 0; i < notify_state.cfg.maxDrivers; i++) {
		drv_handle = &(notify_state.drivers[i]);
		WARN_ON(_notify_is_support_proc(drv_handle, proc_id)
							== false);
		if (drv_handle->is_init ==
			NOTIFY_DRIVERINITSTATUS_NOTDONE) {
				if (drv_handle->fn_table.disable) {
					drv_handle->disable_flag[notify_state.
						disable_depth] =
						(u32 *)drv_handle->fn_table.
							disable(drv_handle,
								proc_id);
				}
		}
	}
	notify_state.disable_depth++;
	mutex_unlock(notify_state.gate_handle);

	return notify_state.disable_depth;
}
EXPORT_SYMBOL(notify_disable);

/*
 * notify_restore
 *
 * desc   This function restores the Notify module to the state before the
 *	last notify_disable() was called. This is equivalent to global
 *	interrupt restore, however restricted within interrupts handled by
 *	the Notify module. All callbacks registered for all events as
 *	specified in the flags are enabled with this API. It is not possible
 *	to enable a specific callback.
 *
 *
 */
void notify_restore(u32 key, u16 proc_id)
{
	struct notify_driver_object *drv_handle;
	int  i;

	if (atomic_cmpmask_and_lt(&(notify_state.ref_count),
			NOTIFY_MAKE_MAGICSTAMP(0),
			NOTIFY_MAKE_MAGICSTAMP(1)) == true)
			WARN_ON(1);

	if (mutex_lock_interruptible(notify_state.gate_handle) != 0)
		WARN_ON(1);
	notify_state.disable_depth--;
	for (i = 0; i < notify_state.cfg.maxDrivers; i++) {
		drv_handle = &(notify_state.drivers[i]);
			if (drv_handle->fn_table.restore)
				drv_handle->fn_table.restore(drv_handle,
							key, proc_id);
	}
	mutex_unlock(notify_state.gate_handle);
	return;
}
EXPORT_SYMBOL(notify_restore);

/*
 *func   notify_disable_event
 *
 * desc   This function disables a specific event. All callbacks registered
 * for the specific event are disabled with this API. It is not
 * possible to disable a specific callback.
 *
 */

void notify_disable_event(void *notify_driver_handle, u16 proc_id, u32 event_no)
{
	int status = 0;
	struct notify_driver_object *drv_handle =
			(struct notify_driver_object *)notify_driver_handle;
	BUG_ON(drv_handle == NULL);

	if (atomic_cmpmask_and_lt(&(notify_state.ref_count),
			NOTIFY_MAKE_MAGICSTAMP(0),
			NOTIFY_MAKE_MAGICSTAMP(1)) == true) {
		status = NOTIFY_E_INVALIDSTATE;
		goto func_end;
	}

	if (WARN_ON(drv_handle->is_init == false)) {
		status = NOTIFY_E_INVALIDSTATE;
		goto func_end;
	}
	if (WARN_ON(_notify_is_support_proc(drv_handle, proc_id) != true)) {
		status = NOTIFY_E_INVALIDARG;
		goto func_end;
	}
	if (WARN_ON(((event_no & NOTIFY_EVENT_MASK)
		>= drv_handle->attrs.proc_info[proc_id].max_events))) {
		status = NOTIFY_E_INVALIDEVENT;
		goto func_end;
	}
	if (WARN_ON(((event_no & NOTIFY_EVENT_MASK) <
		drv_handle->attrs.proc_info[proc_id].reserved_events) &&
		((event_no & NOTIFY_SYSTEM_KEY_MASK) >> sizeof(u16)) !=
		NOTIFY_SYSTEM_KEY)) {
		status = NOTIFY_E_RESERVEDEVENT;
		goto func_end;
	}
	if (mutex_lock_interruptible(notify_state.gate_handle) != 0)
		WARN_ON(1);
	drv_handle->fn_table.disable_event(drv_handle,
			proc_id, (event_no & NOTIFY_EVENT_MASK));
	mutex_unlock(notify_state.gate_handle);
func_end:
	return;
}
EXPORT_SYMBOL(notify_disable_event);

/*
 * notify_enable_event
 *
 * This function enables a specific event. All callbacks registered for
 * this specific event are enabled with this API. It is not possible to
 * enable a specific callback.
 *
 */
void notify_enable_event(void *notify_driver_handle, u16 proc_id, u32 event_no)
{

	struct notify_driver_object *drv_handle =
			(struct notify_driver_object *) notify_driver_handle;

	BUG_ON(drv_handle == NULL);

	if (atomic_cmpmask_and_lt(&(notify_state.ref_count),
			NOTIFY_MAKE_MAGICSTAMP(0),
			NOTIFY_MAKE_MAGICSTAMP(1)) == true) {
		goto func_end;
	}

	if (WARN_ON(drv_handle->is_init == false))
		goto func_end;
	if (WARN_ON(_notify_is_support_proc(drv_handle, proc_id) != true))
		goto func_end;
	if (WARN_ON(((event_no & NOTIFY_EVENT_MASK)
		>= drv_handle->attrs.proc_info[proc_id].max_events)))
		goto func_end;
	if (WARN_ON(((event_no & NOTIFY_EVENT_MASK) <
		drv_handle->attrs.proc_info[proc_id].reserved_events) &&
		((event_no & NOTIFY_SYSTEM_KEY_MASK) >> sizeof(u16)) !=
		NOTIFY_SYSTEM_KEY))
		goto func_end;

	if (mutex_lock_interruptible(notify_state.gate_handle) != 0)
		WARN_ON(1);
		if (drv_handle->fn_table.enable_event) {
			drv_handle->fn_table.enable_event(drv_handle,
				proc_id, (event_no & NOTIFY_EVENT_MASK));
		}
	mutex_unlock(notify_state.gate_handle);
func_end:
	return;
}
EXPORT_SYMBOL(notify_enable_event);

/*
 *_notify_is_support_proc
 *
 * Check if specified processor ID is supported by the Notify driver.
 *
 */
static bool _notify_is_support_proc(struct notify_driver_object *drv_handle,
							int proc_id)
{
	bool  found = false;
	struct notify_driver_attrs *attrs;
	int i;

	BUG_ON(drv_handle == NULL);
	attrs = &(drv_handle->attrs);
	for (i = 0; i < MULTIPROC_MAXPROCESSORS; i++) {
		if (attrs->proc_info[i].proc_id == proc_id) {
			found = true;
			break;
		}
	}
	return found;
}
