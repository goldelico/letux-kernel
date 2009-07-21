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
#define NOTIFY_E_INVALIDSTATE	NOTIFY_MAKE_FAILURE(13)

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
 * brief Failed to register driver with Notify module.
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


/* brief Macro to make a correct module magic number with refCount */
#define NOTIFY_MAKE_MAGICSTAMP(x) ((NOTIFY_MODULEID << 12u) | (x))


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
};

typedef void (*notify_callback_fxn)(u16 proc_id, u32 eventNo, void *arg,
					u32 payload);

extern struct notify_module_object notify_state;

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

