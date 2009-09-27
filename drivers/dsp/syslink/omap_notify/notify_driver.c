/*
 * notify_driver.c
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
#include <syslink/gt.h>

#include <syslink/notify.h>
#include <syslink/notify_driver.h>
#include <syslink/atomic_linux.h>

/*
 *func   notify_register_driver
 *
 *desc   This function registers a Notify driver with the Notify module.
 */

int notify_register_driver(char *driver_name,
			struct notify_interface *fn_table,
			struct notify_driver_attrs *drv_attrs,
			struct notify_driver_object **driver_handle)
{
	int status = NOTIFY_SUCCESS;
	struct notify_driver_object *drv_handle = NULL;
	int i;

	BUG_ON(driver_name == NULL);
	BUG_ON(fn_table == NULL);
	BUG_ON(drv_attrs == NULL);
	BUG_ON(driver_handle == NULL);

	if (atomic_cmpmask_and_lt(&(notify_state.ref_count),
			NOTIFY_MAKE_MAGICSTAMP(0),
			NOTIFY_MAKE_MAGICSTAMP(1)) == true) {
		status = NOTIFY_E_INVALIDSTATE;
		goto func_end;
	}

	/*Initialize to status that indicates that an empty slot was not
	  *found for the driver.
	 */
	if (mutex_lock_interruptible(notify_state.gate_handle) != 0)
		WARN_ON(1);
	status = NOTIFY_E_MAXDRIVERS;
	for (i = 0; i < notify_state.cfg.maxDrivers; i++) {
		drv_handle = &(notify_state.drivers[i]);

		if (drv_handle->is_init == NOTIFY_DRIVERINITSTATUS_DONE) {
			if (strncmp(driver_name, drv_handle->name,
				NOTIFY_MAX_NAMELEN) == 0) {
				status = NOTIFY_E_ALREADYEXISTS;
				goto return_existing_handle;
			}
		}
		 if (drv_handle->is_init == NOTIFY_DRIVERINITSTATUS_NOTDONE) {
			/* Found an empty slot, so block it. */
			drv_handle->is_init =
					NOTIFY_DRIVERINITSTATUS_INPROGRESS;
			status = NOTIFY_SUCCESS;
			break;
		}
	}
	mutex_unlock(notify_state.gate_handle);
	WARN_ON(status < 0);
	/*Complete registration of the driver. */
	strncpy(drv_handle->name,
			driver_name, NOTIFY_MAX_NAMELEN);
	memcpy(&(drv_handle->attrs), drv_attrs,
				sizeof(struct notify_driver_attrs));
	memcpy(&(drv_handle->fn_table), fn_table,
				sizeof(struct notify_interface));
	drv_handle->driver_object = NULL;

return_existing_handle:
	/*is_setup is set when driverInit is called. */
	*driver_handle = drv_handle;

func_end:
	return status;
}
EXPORT_SYMBOL(notify_register_driver);

/*========================================
 *func   notify_unregister_driver
 *
 *desc   This function un-registers a Notify driver with the Notify module.
 */
int notify_unregister_driver(struct notify_driver_object  *drv_handle)
{
	int status = NOTIFY_SUCCESS;

	BUG_ON(drv_handle == NULL);


	if (atomic_cmpmask_and_lt(&(notify_state.ref_count),
			NOTIFY_MAKE_MAGICSTAMP(0),
			NOTIFY_MAKE_MAGICSTAMP(1)) == true) {
		status = NOTIFY_E_INVALIDSTATE;
	} else {
		/* Unregister the driver. */
		drv_handle->is_init = NOTIFY_DRIVERINITSTATUS_NOTDONE;

	}
	return status;
}
EXPORT_SYMBOL(notify_unregister_driver);


/*==================================
 * Function to find and return the driver handle maintained within
 * the Notify module.
 *
 * driver_name       Name of the driver to be searched.
 * handle       Return parameter: Handle to the driver.
 *
 */
int notify_get_driver_handle(char *driver_name,
				struct notify_driver_object **handle)
{
	int status = NOTIFY_E_NOTFOUND;
	struct notify_driver_object *drv_handle;
	int i;

	BUG_ON(handle == NULL);

	if (atomic_cmpmask_and_lt(&(notify_state.ref_count),
			NOTIFY_MAKE_MAGICSTAMP(0),
			NOTIFY_MAKE_MAGICSTAMP(1)) == true) {
		status = NOTIFY_E_INVALIDSTATE;
	} else if (WARN_ON(driver_name == NULL))
		status = NOTIFY_E_INVALIDARG;
	else {
		if (mutex_lock_interruptible(notify_state.gate_handle) != 0)
			WARN_ON(1);

		for (i = 0; i < notify_state.cfg.maxDrivers; i++) {
			drv_handle = &(notify_state.drivers[i]);
			/* Check whether the driver handle slot is occupied. */
			if (drv_handle->is_init == true) {
				if (strncmp(driver_name, drv_handle->name,
					NOTIFY_MAX_NAMELEN) == 0) {
					status = NOTIFY_SUCCESS;
					*handle = drv_handle;
					break;
				}
			}
		}
		mutex_unlock(notify_state.gate_handle);
	}
	return status;
}
EXPORT_SYMBOL(notify_get_driver_handle);
