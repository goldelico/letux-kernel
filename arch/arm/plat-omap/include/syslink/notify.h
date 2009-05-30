/*
 * notify.h
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


#if !defined NOTIFY_H
#define NOTIFY_H

#include <syslink/host_os.h>

#define NOTIFY_MAX_DRIVERS		16

/*
 * desc  Maximum length of the name of Notify drivers, inclusive of NULL
 * string terminator.
 *
 */
#define NOTIFY_MAX_NAMELEN		32

#define NOTIFY_MODULEID		0x5f84

/*
 *Status code base for Notify module.
 */
#define NOTIFY_STATUSCODEBASE		(NOTIFY_MODULEID << 12u)

/*
 * Macro to make error code.
 */
#define NOTIFY_MAKE_FAILURE(x)		((int)(0x80000000\
					| (NOTIFY_STATUSCODEBASE + (x))))

/*
 * Macro to make success code.
 */
#define NOTIFY_MAKE_SUCCESS(x)		(NOTIFY_STATUSCODEBASE + (x))

/*
 * Generic failure.
 */
#define NOTIFY_E_FAIL			NOTIFY_MAKE_FAILURE(1)

/*
 * A timeout occurred while performing the specified operation.
 */
#define NOTIFY_E_TIMEOUT		NOTIFY_MAKE_FAILURE(2)

/*
 *Configuration failure.
 */
#define NOTIFY_E_CONFIG		NOTIFY_MAKE_FAILURE(3)

/*
 *  The module is already initialized
 */
#define NOTIFY_E_ALREADYINIT		NOTIFY_MAKE_FAILURE(4)

/*
 *  Unable to find the specified entity (e.g. registered event, driver).
 */
#define NOTIFY_E_NOTFOUND 		NOTIFY_MAKE_FAILURE(5)

/*
 * The specified operation is not supported.
 */
#define NOTIFY_E_NOTSUPPORTED		NOTIFY_MAKE_FAILURE(6)

/*
* Invalid event number specified to the Notify operation.
 */
#define NOTIFY_E_INVALIDEVENT		NOTIFY_MAKE_FAILURE(7)

/*
 * Invalid pointer provided.
 */
#define NOTIFY_E_POINTER 		NOTIFY_MAKE_FAILURE(8)
/*
 *  The specified value is out of valid range.
 */
#define NOTIFY_E_RANGE			NOTIFY_MAKE_FAILURE(9)

/* An invalid handle was provided.
 */
#define NOTIFY_E_HANDLE		NOTIFY_MAKE_FAILURE(10)

/*
 * An invalid argument was provided to the API.
 */
#define NOTIFY_E_INVALIDARG		NOTIFY_MAKE_FAILURE(11)

/*
 * A memory allocation failure occurred.
 */
#define NOTIFY_E_MEMORY		NOTIFY_MAKE_FAILURE(12)

/*
 * The module has not been setup.
 */
#define NOTIFY_E_SETUP			NOTIFY_MAKE_FAILURE(13)

/*
 *  Maximum number of supported drivers have already been registered.
 */
#define NOTIFY_E_MAXDRIVERS		NOTIFY_MAKE_FAILURE(14)

/*
 *  Invalid attempt to use a reserved event number.
 */
#define NOTIFY_E_RESERVEDEVENT	NOTIFY_MAKE_FAILURE(15)

/*
 * The specified entity (e.g. driver) already exists.
 */
#define NOTIFY_E_ALREADYEXISTS	NOTIFY_MAKE_FAILURE(16)

/*
 * brief The Notify driver has not been initialized.
 */
#define NOTIFY_E_DRIVERINIT		NOTIFY_MAKE_FAILURE(17)

/*
* The remote processor is not ready to receive the event.
 */
#define NOTIFY_E_NOTREADY		NOTIFY_MAKE_FAILURE(18)

/*
 * @brief Failed to register driver with Notify module.
 */
#define NOTIFY_E_REGDRVFAILED		NOTIFY_MAKE_FAILURE(19)

/*
* Failed to unregister driver with Notify module.
 */
#define NOTIFY_E_UNREGDRVFAILED	 NOTIFY_MAKE_FAILURE(20)

/*
* Failure in an OS-specific operation.
 */
#define NOTIFY_E_OSFAILURE		NOTIFY_MAKE_FAILURE(21)

/*
 *Maximum number of supported events have already been registered.
 */
#define NOTIFY_E_MAXEVENTS		NOTIFY_MAKE_FAILURE(22)

/* Maximum number of supported user clients have already been
 * registered.
 */
#define NOTIFY_E_MAXCLIENTS		NOTIFY_MAKE_FAILURE(23)

/* Operation is successful.
 */
#define NOTIFY_SUCCESS			NOTIFY_MAKE_SUCCESS(0)

/* The ProcMgr module has already been setup in this process.
 */
#define NOTIFY_S_ALREADYSETUP		NOTIFY_MAKE_SUCCESS(1)

/* Other ProcMgr clients have still setup the ProcMgr module.
 */
#define NOTIFY_S_SETUP			NOTIFY_MAKE_SUCCESS(2)

/* Other ProcMgr handles are still open in this process.
 */
#define NOTIFY_S_OPENHANDLE		NOTIFY_MAKE_SUCCESS(3)

/* The ProcMgr instance has already been created/opened in this process
 */
#define NOTIFY_S_ALREADYEXISTS	NOTIFY_MAKE_SUCCESS(4)

/* Maximum depth for nesting Notify_disable / Notify_restore calls.
 */
#define NOTIFY_MAXNESTDEPTH		2

/*
 *  const  NOTIFYSHMDRV_DRIVERNAME
 *
 *  desc   Name of the Notify Shared Memory Mailbox driver.
 *
 */
#define NOTIFYMBXDRV_DRIVERNAME   "NOTIFYMBXDRV"

#define REG volatile
/*
 *  const  NOTIFYSHMDRV_RESERVED_EVENTS
 *
 *  desc   Maximum number of events marked as reserved events by the
 *          notify_shmdrv driver.
 *          If required, this value can be changed by the system integrator.
 *
 */
#define NOTIFYSHMDRV_RESERVED_EVENTS  3

/*
* This key must be provided as the upper 16 bits of the eventNo when
 * registering for an event, if any reserved event numbers are to be
 * used.
 */
#define NOTIFY_SYSTEM_KEY		0xC1D2

struct notify_config {
	u32 maxDrivers;
	/* Maximum number of drivers that can be created for Notify at a time */
	struct mutex *gate_handle;
	/* Handle of gate to be used for local thread safety */
};

typedef void (*notify_callback_fxn)(u16 proc_id, u32 eventNo, void *arg,
					u32 payload);

extern struct notify_module_object notify_state;

/*
*  name   struct notify_shmdrv_event_entry
*
*  desc   Defines the structure of event entry within the event chart.
*          Each entry contains occured event-specific information.
*          This is shared between GPP and DSP.
*
*  field  flag
*              Indicating event is present or not.
*  field  payload
*              Variable containing data associated with each occured event.
*  field  reserved
*              Reserved field to contain additional information about the
*              event entry.
*  field  padding
*              Padding.
*
*/
struct notify_shmdrv_event_entry {
	REG unsigned long int flag;
	REG unsigned long int payload;
	REG unsigned long int reserved;
	/*ADD_PADDING(padding, NOTIFYSHMDRV_EVENT_ENTRY_PADDING)*/
};

/*
*  name   struct notify_shmdrv_eventreg_mask
*
*  desc   Defines the mask indicating registered events on the processor.
*          This is shared between GPP and DSP.
*
*  field  mask
*              Indicating event is registered.
*  field  enable_mask
*              Indicates event is enabled.
*  field  padding
*              Padding.
*
*/
struct notify_shmdrv_eventreg_mask {
	REG unsigned long int mask;
	REG unsigned long int enable_mask;
	/*ADD_PADDING (padding, IPC_64BIT_PADDING)*/
};

/*
*  name   notify_shmdrv_eventreg
*
*  desc   Defines the structure of event registration entry within the Event
*          Registration Chart.
*          Each entry contains registered event-specific information.
*
*  field  reg_event_no
*              Index into the event chart, indicating the registered event.
*  field  reserved
*              Reserved field to contain additional information about the
*              registered event.
*
*/
struct notify_shmdrv_eventreg {
	unsigned long int reg_event_no;
	unsigned long int reserved;
};

/*
*  name   struct NotifyShmDrv_ProcCtrl
*
*  desc   Defines the NotifyShmDrv control structure, which contains all
*          information for one processor.
*          This structure is shared between the two processors.
*
*  field  otherEventChart
*              Address of event chart for the other processor.
*  field  selfEventChart
*              Address of event chart for this processor.
*  field  recv_init_status
*              Indicates whether the driver has been initialized, and is ready
*              to receive events on this processor. If the driver does not
*              support events from other processor to this processor, this flag
*              will always indicate not-initialized status.
*  field  send_init_status
*              Indicates whether the driver has been initialized, and is ready
*              to send events on this processor. If the driver does not
*              support events from this processor to other processor, this flag
*              will always indicate not-initialized status.
*  field  padding
*              Padding for alignment.
*  field  selfRegMask
*              Registration mask.
*
*/
struct notify_shmdrv_proc_ctrl {
	struct notify_shmdrv_event_entry *self_event_chart;
	struct notify_shmdrv_event_entry *other_event_chart;
	unsigned long int recv_init_status;
	unsigned long int send_init_status;
	/*ADD_PADDING(padding, NOTIFYSHMDRV_CTRL_PADDING)*/
	struct notify_shmdrv_eventreg_mask reg_mask;
};

/*!
 *  @brief  Defines the NotifyDriverShm control structure, which contains all
 *          information shared between the two connected processors
 *          This structure is shared between the two processors.
 */
struct notify_shmdrv_ctrl {
	struct notify_shmdrv_proc_ctrl proc_ctrl[2];
};

/* Function to get the default configuration for the Notify module. */
void notify_get_config(struct notify_config *cfg);

/* Function to setup the Notify Module */
int notify_setup(struct notify_config *cfg);

/* Function to destroy the Notify module */
int notify_destroy(void);

/* Function to register an event */
int notify_register_event(void *notify_driver_handle, u16 proc_id,
			u32 event_no,
			notify_callback_fxn notify_callback_fxn,
			void *cbck_arg);

/* Function to unregister an event */
int notify_unregister_event(void *notify_driver_handle, u16 proc_id,
			u32 event_no,
			notify_callback_fxn notify_callback_fxn,
			void *cbck_arg);

/* Function to send an event to other processor */
int notify_sendevent(void *notify_driver_handle, u16 proc_id,
		     u32 event_no, u32 payload, bool wait_clear);

/* Function to disable Notify module */
u32 notify_disable(u16 procId);

/* Function to restore Notify module state */
void notify_restore(u32 key, u16 proc_id);

/* Function to disable particular event */
void notify_disable_event(void *notify_driver_handle, u16 proc_id,
			u32 event_no);

/* Function to enable particular event */
void notify_enable_event(void *notify_driver_handle, u16 proc_id, u32 event_no);

#endif /* !defined NOTIFY_H */

