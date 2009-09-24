/*
 * notify_ducatidriver.h
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

#ifndef NOTIFY_DUCATIDRIVER_H_
#define NOTIFY_DUCATIDRIVER_H_



/* Notify*/
#include <syslink/GlobalTypes.h>
#include <syslink/notifyerr.h>
#include <syslink/notify_driverdefs.h>

/*
 *  const  NOTIFYDUCATI_DRIVERNAME
 *
 *  desc   Name of the ducati driver.
 *
 */

#define IPC_BUF_ALIGN     128
#define IPC_ALIGN(x, y) (unsigned long int)\
((unsigned long int)((x + y - 1) / y) * y)


#define NOTIFYDUCATI_DRIVERNAME   "NOTIFY_DUCATIDRV"

#define REG volatile


extern u32 get_ducati_virt_mem();
extern void unmap_ducati_virt_mem(u32 shm_virt_addr);

/*
*  func   notify_mbxdrv_register_event
*
*  desc   Register a callback for an event with the Notify driver.
*
*
*/

int notify_ducatidrv_register_event(
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

int notify_ducatidrv_unregister_event(
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

int notify_ducatidrv_sendevent(struct notify_driver_object *handle,
			       short int   proc_id,
			       int   event_no,
			       int   payload,
			       short int           wait_clear) ;

/*
*  func   notify_mbxdrv_disable
*
*  desc   Disable all events for this Notify driver.
*
*
*/

void *notify_ducatidrv_disable(struct notify_driver_object *handle);

/*
*  func   notify_mbxdrv_restore
*
*  desc   Restore the Notify driver to the state before the last disable was
*          called.
*
*
*/

int notify_ducatidrv_restore(struct notify_driver_object *handle,
			     void *flags) ;

/*
*  func   notify_mbxdrv_disable_event
*
*  desc   Disable a specific event for this Notify driver.
*
*
*/

int notify_ducatidrv_disable_event(
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

int notify_ducatidrv_enable_event(
	struct notify_driver_object *handle,
	short int    proc_id,
	int    event_no) ;


/*
*  func   notify_mbxdrv_debug
*
*  desc   Print debug information for the Notify driver.
*
*
*/

int notify_ducatidrv_debug(struct notify_driver_object *handle) ;

struct notify_ducatidrv_params {
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
 *  struct   notify_ducatidrv_config
 *
 */

struct notify_ducatidrv_config {
	u32 reserved;
};

/* Function to get the default configuration for the Notify module. */
void notify_ducatidrv_getconfig(struct notify_ducatidrv_config *cfg);

/* Function to setup the notify ducati driver with the given configuration*/
int notify_ducatidrv_setup(struct notify_ducatidrv_config *cfg);

/* Function to destroy the notify ducati driver*/
int notify_ducatidrv_destroy(void);

/* Function to create the ducati driver handle and performs initialization. */

struct notify_driver_object *notify_ducatidrv_create(char *driver_name,
		const struct notify_ducatidrv_params *params);

/* Function to delete the ducati driver handle and performs de initialization.*/
int notify_ducatidrv_delete(struct notify_driver_object **handle_ptr);

/*Function to open the ducati driver  */
int notify_ducatidrv_open(char *driver_name,
			  struct notify_driver_object **handle_ptr);

/*Function to close the ducati driver  */
int notify_ducatidrv_close(struct notify_driver_object **handle_ptr);

/*Function to initialize the given parameters  */
void notify_ducatidrv_params_init(struct notify_driver_object *handle,
				  struct notify_ducatidrv_params *params);

#endif  /* !defined  NOTIFY_SHMDRIVER_H_ */

