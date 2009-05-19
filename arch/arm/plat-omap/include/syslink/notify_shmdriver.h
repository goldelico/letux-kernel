
/*
 * notify_shmdriver.h
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


#if !defined NOTIFY_SHMDRIVER_H_
#define NOTIFY_SHMDRIVER_H_

/*
 *  const  NOTIFYSHMDRV_DRIVERNAME
 *
 *  desc   Name of the Notify Shared Memory Mailbox driver.
 *
 */
#define NOTIFYSHMDRV_DRIVERNAME   "NOTIFYSHMDRV"

/*
 *  const  NOTIFYSHMDRV_RESERVED_EVENTS
 *
 *  desc   Maximum number of events marked as reserved events by the
 *          NotiyShmDrv driver.
 *          If required, this value can be changed by the system integrator.
 *
 */

#define NOTIFYSHMDRV_RESERVED_EVENTS  3


/*
 *  name   notify_shmdrv_attrs
 *
 *  desc   This structure defines the attributes for Notify Shared Memory
 *          Mailbox driver.
 *          These attributes are passed to the driver when notify_driver_init ()
 *          is called for this driver.
 *
 *  field  shm_base_addr
 *              Shared memory address base for the NotifyShmDrv driver. This
 *              must be the start of shared memory as used by both connected
 *              processors, and the same must be specified on both sides when
 *              initializing the NotifyShmDrv driver.
 *  field  shm_size
 *              Size of shared memory provided to the NotifyShmDrv driver. This
 *              must be the start of shared memory as used by both connected
 *              processors, and the same must be specified on both sides when
 *              initializing the NotifyShmDrv driver.
 *  field  num_events
 *              Number of events required to be supported. Must be greater than
 *              or equal to reserved events supported by the driver.
 *  field  send_event_pollcount
 *              Poll count to be used when sending event. If the count is
 *              specified as -1, the wait will be infinite. NOTIFY_sendEvent
 *              will return with timeout error if the poll count expires before
 *              the other processor acknowledges the received event.
 *
 *  see    None.
 *
 */
struct notify_shmdrv_attrs {
	unsigned long int    shm_base_addr ;
	unsigned long int    shm_size ;
	unsigned long int    num_events ;
	unsigned long int    send_event_pollcount ;
};

#endif  /* !defined  NOTIFY_SHMDRIVER_H_ */


