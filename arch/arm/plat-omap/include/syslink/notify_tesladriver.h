/*
 * notify_tesladriver.h
 *
 * Notify driver support for OMAP Processors.
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




/* Notify*/
#include <syslink/GlobalTypes.h>
#include <syslink/notifyerr.h>
#include <syslink/notify_driverdefs.h>
#include <syslink/notifydefs.h>

/*
 *  const  NOTIFYSHMDRV_DRIVERNAME
 *
 *  desc   Name of the Notify Shared Memory Mailbox driver.
 *
 */
#define NOTIFYMBXDRV_DRIVERNAME   "NOTIFYMBXDRV"

/*
*  struct    notify_tesladrv_params
*
*  desc   driver.params
*
*/


struct notify_tesladrv_params{
	int shared_addr;
	int shared_addr_size;
	int num_events;
	int recv_int_id;
	int send_int_id;
	int remote_proc_id;
	int num_reserved_events;
	int send_event_poll_count;
} ;

/*
*  struct    notify_tesladrv_config
*
*  desc   driver.configuration
*
*/


struct notify_tesladrv_config {
	void *gate_handle;
};



/*
*  func   notify_mbxdrv_register_event
*
*  desc   Register a callback for an event with the Notify driver.
*
*
*/

int notify_tesladrv_register_event(
				struct notify_driver_object *handle,
				short int        proc_id,
				int              event_no,
				fn_notify_cbck        fn_notify_cbck,
				void *cbck_arg) ;

/*
*  func   notify_mbxdrv_unregevent
*
*  desc   Unregister a callback for an event with the Notify driver.
*
*
*/

int notify_tesladrv_unregister_event(
				struct notify_driver_object *handle,
				short int        proc_id,
				int        event_no,
				fn_notify_cbck        fn_notify_cbck,
				void *cbck_arg) ;

/*
*  func   notify_mbxdrv_sendevent
*
*  desc   Send a notification event to the registered users for this
*          notification on the specified processor.
*
*
*/

int notify_tesladrv_sendevent(struct notify_driver_object *handle,
			short int        proc_id,
			int              event_no,
			int              payload,
			short int        wait_clear) ;

/*
*  func   notify_mbxdrv_disable
*
*  desc   Disable all events for this Notify driver.
*
*
*/

void * notify_tesladrv_disable(struct notify_driver_object *handle, u16 proc_Id) ;

/*
*  func   notify_mbxdrv_restore
*
*  desc   Restore the Notify driver to the state before the last disable was
*          called.
*
*
*/

int notify_tesladrv_restore(struct notify_driver_object *handle,
					u32 key, u16 proc_id) ;

/*
*  func   notify_mbxdrv_disable_event
*
*  desc   Disable a specific event for this Notify driver.
*
*
*/

int notify_tesladrv_disable_event(
			struct notify_driver_object *handle,
			short int       proc_id,
			int   event_no) ;

/*
*  func   notify_mbxdrv_enable_event
*
*  desc   Enable a specific event for this Notify driver.
*
*
*/

int notify_tesladrv_enable_event(
			struct notify_driver_object *handle,
			short int    proc_id,
			int    event_no) ;






/*
*  func   notify_tesladrv_debug
*
*  desc   Print debug information for the Notify driver.
*
*
*/

int notify_tesladrv_debug(struct notify_driver_object *handle) ;

/*
*  func   notify_tesladrv_create
*
*  desc   creates driver handle.
*
*
*/

struct notify_driver_object * notify_tesladrv_create(char * driver_name,
	const struct notify_tesladrv_params* params);

/*
*  func   notify_tesladrv_delete
*
*  desc   deletes driver handle.
*/


int notify_tesladrv_delete (struct notify_driver_object** handlePtr);

/*
*  func   notify_tesladrv_getconfig
*
*  desc   Get the default configuration for driver.
*/
void notify_tesladrv_getconfig (struct notify_tesladrv_config* cfg);


/*
*  func   notify_tesladrv_setup
*
*  desc   setup the driver with the given config.
*/


int notify_tesladrv_setup (struct notify_tesladrv_config * cfg);

/*
*  func   notify_tesladrv_params_init
*
*  desc   initializes parameters for driver.
*/

void notify_tesladrv_params_init (struct notify_driver_object *handle,
	struct notify_tesladrv_params * params);

/*
*  func   notify_tesladrv_destroy
*
*  desc   destroys the driver
*/

int notify_tesladrv_destroy (void);



