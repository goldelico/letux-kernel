/*
 * notify_driver.h
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


#if !defined NOTIFYDRIVER_H
#define NOTIFYDRIVER_H

#include<linux/list.h>

/*  ----------------------------------- Notify */
#include <syslink/notifyerr.h>

/*  ----------------------------------- Notify driver */
#include <syslink/notify_driverdefs.h>

/* Function to register notify driver */
int notify_register_driver(char *driver_name,
		struct notify_interface *fn_table,
		struct notify_driver_attrs *drv_attrs,
		struct notify_driver_object **driver_handle);


/* Function to unregister notify driver */
int notify_unregister_driver(struct notify_driver_object *drv_handle);

/* Function to find the driver in the list of drivers */
int notify_get_driver_handle(char *driver_name,
				struct notify_driver_object **handle);

#endif  /* !defined (NOTIFYDRIVER_H) */

