
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
 */
struct notify_shmdrv_attrs {
	unsigned long int shm_base_addr;
	unsigned long int shm_size;
	unsigned long int num_events;
	unsigned long int send_event_pollcount;
};


/*
*  name	notify_shmdrv_event_entry
*/
struct notify_shmdrv_event_entry {
	REG unsigned long int flag;
	REG unsigned long int payload;
	REG unsigned long int reserved;
	unsigned long int padding[29];
};

/*
*  name	notify_shmdrv_eventreg_mask
*
*/
struct notify_shmdrv_eventreg_mask {
	REG unsigned long int mask;
	REG unsigned long int enable_mask;
	unsigned long int padding[30];
};

/*
*  name	notify_shmdrv_eventreg
*
*/
struct notify_shmdrv_eventreg {
	unsigned long int reg_event_no;
	unsigned long int reserved;
};

/*
*  name	notify_shmdrv_proc_ctrl
*
*/
struct notify_shmdrv_proc_ctrl {
	struct notify_shmdrv_event_entry *self_event_chart;
	struct notify_shmdrv_event_entry *other_event_chart;
	unsigned long int recv_init_status;
	unsigned long int send_init_status;
	unsigned long int padding[28];
	struct notify_shmdrv_eventreg_mask reg_mask;
	struct notify_shmdrv_eventreg	*reg_chart;
};

/*
 *  brief  Defines the  notify_shmdrv_ctrl structure, which contains all
 *          information shared between the two connected processors
 *          This structure is shared between the two processors.
 */
struct notify_shmdrv_ctrl {
	struct notify_shmdrv_proc_ctrl proc_ctrl[2];
};

#endif  /* !defined  NOTIFY_SHMDRIVER_H_ */


