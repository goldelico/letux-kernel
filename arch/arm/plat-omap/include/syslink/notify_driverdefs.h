/*
 * notify_driverdefs.h
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


#if !defined NOTIFYDRIVERDEFS_H
#define NOTIFYDRIVERDEFS_H


#include <syslink/host_os.h>

/*  ----------------------------------- Notify */
#include <syslink/notify.h>
#include <syslink/notify_shmdriver.h>
#include <syslink/notifydefs.h>
#include <syslink/multiproc.h>

#define NOTIFY_BASE_CMD		(0x100)

/*
 *  Command for Notify_getConfig
 */
#define CMD_NOTIFY_GETCONFIG		(NOTIFY_BASE_CMD + 1u)

/*
 *  Command for Notify_setup
 */
#define CMD_NOTIFY_SETUP		(NOTIFY_BASE_CMD + 2u)

/*
 *  Command for Notify_destroy
 */
#define CMD_NOTIFY_DESTROY		(NOTIFY_BASE_CMD + 3u)

/*
 *  Command for Notify_registerEvent
 */
#define CMD_NOTIFY_REGISTEREVENT	 (NOTIFY_BASE_CMD + 4u)

/*
 *  Command for Notify_unregisterEvent
 */
#define CMD_NOTIFY_UNREGISTEREVENT	 (NOTIFY_BASE_CMD + 5u)

/*
 *  Command for Notify_sendEvent
 */
#define CMD_NOTIFY_SENDEVENT		(NOTIFY_BASE_CMD + 6u)

/*
 *  Command for Notify_disable
 */
#define CMD_NOTIFY_DISABLE		(NOTIFY_BASE_CMD + 7u)

/*
 *  Command for Notify_restore
 */
#define CMD_NOTIFY_RESTORE		(NOTIFY_BASE_CMD + 8u)

/*
 *  Command for Notify_disableEvent
 */
#define CMD_NOTIFY_DISABLEEVENT	(NOTIFY_BASE_CMD + 9u)

/*
 *  Command for Notify_enableEvent
 */
#define CMD_NOTIFY_ENABLEEVENT	(NOTIFY_BASE_CMD + 10u)

/*!
 *  @brief  Command for Notify_attach
 */
#define CMD_NOTIFY_ATTACH                    (NOTIFY_BASE_CMD + 11u)

/*!
 *  @brief  Command for Notify_detach
 */
#define CMD_NOTIFY_DETACH                    (NOTIFY_BASE_CMD + 12u)

/*
 *  const  NOTIFY_SYSTEM_KEY_MASK
 *
 *  desc   Mask to check for system key.
 *
 */

#define NOTIFY_SYSTEM_KEY_MASK	(unsigned short int) 0xFFFF0000

/*
 *  const  NOTIFY_EVENT_MASK
 *
 *  desc   Mask to check for event ID.
 *
 */

#define NOTIFY_EVENT_MASK	(unsigned short int) 0x0000FFFF

struct notify_cmd_args {
	int apiStatus;
	/* Status of the API being called. */
};

/*
 * Command arguments for Notify_getConfig
 */
struct notify_cmd_args_get_config {
	struct notify_cmd_args commonArgs;
	struct notify_config *cfg;
};

/*
 * Command arguments for Notify_setup
 */
struct notify_cmd_args_setup {
	struct notify_cmd_args commonArgs;
	struct notify_config *cfg;
};

/*
 * Command arguments for Notify_destroy
 */
struct notify_cmd_args_destroy {
	struct notify_cmd_args commonArgs;
};

/*
 * Command arguments for Notify_registerEvent
 */
struct notify_cmd_args_register_event {
	struct notify_cmd_args commonArgs;
	struct notify_driver_object *handle;
	u16 procId;
	u32 eventNo;
	notify_callback_fxn fnNotifyCbck;
	void *cbckArg;
	u32 pid;
};

/*
 * Command arguments for Notify_unregisterEvent
 */
struct notify_cmd_args_unregister_event {
	struct notify_cmd_args		commonArgs;
	struct notify_driver_object *handle;
	u16 procId;
	u32 eventNo;
	notify_callback_fxn fnNotifyCbck;
	void *cbckArg;
	u32 pid;
};

/*
 * Command arguments for Notify_sendEvent
 */
struct notify_cmd_args_send_event {
	struct notify_cmd_args commonArgs;
	struct notify_driver_object *handle;
	u16 procId;
	u32 eventNo;
	u32 payload;
	bool waitClear;
};

/*
 * Command arguments for Notify_disable
 */
struct notify_cmd_args_disable {
	struct notify_cmd_args commonArgs;
	u16 procId;
	u32 flags;
};

/*
 * Command arguments for Notify_restore
 */
struct notify_cmd_args_restore {
	struct notify_cmd_args commonArgs;
	u32 key;
	u16 procId;
};

/*
 * Command arguments for Notify_disableEvent
 */
struct notify_cmd_args_disable_event {
	struct notify_cmd_args commonArgs;
	struct notify_driver_object *handle;
	u16 procId;
	u32 eventNo;
};

/*
 * Command arguments for Notify_enableEvent
 */
struct notify_cmd_args_enable_event {
	struct notify_cmd_args commonArgs;
	void *notify_driver_handle;
	u16 procId;
	u32 eventNo;
};

/*
 * Command arguments for Notify_exit
 */
struct notify_cmd_args_exit {
	struct notify_cmd_args commonArgs;
};


enum {
	NOTIFY_DRIVERINITSTATUS_NOTDONE = 0,
	/* Driver initialization is not done. */
	NOTIFY_DRIVERINITSTATUS_DONE = 1,
	/* Driver initialization is complete. */
	NOTIFY_DRIVERINITSTATUS_INPROGRESS = 2,
	/* Driver initialization is in progress. */
	NOTIFY_DRIVERINITSTATUS_ENDVALUE = 3
	/* End delimiter indicating start of invalid values for this enum */
};

/*
 *This structure defines information for all processors supported by
 *the Notify driver.
 *An instance of this object is provided for each processor handled by
 *the Notify driver, when registering itself with the Notify module.
 *
 */
struct notify_driver_proc_info {
	u32 max_events;
	u32 reserved_events;
	bool event_priority;
	u32 payload_size;
	u16 proc_id;
};

/*
 * This structure defines the structure for specifying Notify driver
 * attributes to the Notify module.
 * This structure provides information about the Notify driver to the
 * Notify module. The information is used by the Notify module mainly
 * for parameter validation. It may also be used by the Notify module
 * to take appropriate action if required, based on the characteristics
 * of the Notify driver.
 */
struct notify_driver_attrs {
	u32 numProc;
	struct notify_driver_proc_info
				proc_info[MULTIPROC_MAXPROCESSORS];
};


/* ========================================
 *  Function pointer types
 * ========================================
 */
/*
 * This type defines the function to register a callback for an event
 *  with the Notify driver.
 *  This function gets called internally from the Notify_registerEvent
 *  API. The Notify_registerEvent () function passes on the
 *  request into the Notify driver identified by the Notify Handle.
 *
 */
typedef int(*NotifyDriver_RegisterEvent)(struct notify_driver_object *handle,
		u16 procId, u32 eventNo, notify_callback_fxn cbckFxn,
		void *cbckArg);
/*
 * This type defines the function to unregister a callback for an event
 *  with the Notify driver.
 *  This function gets called internally from the Notify_unregisterEvent
 *  API. The Notify_unregisterEvent () function passes on the
 *  request into the Notify driver identified by the Notify Handle.
 *
 */
typedef int(*NotifyDriver_UnregisterEvent) (struct notify_driver_object *handle,
		u16 procId, u32 eventNo, notify_callback_fxn  cbckFxn,
		void *cbckArg);

/*
 * This type defines the function to send a notification event to the
 *  registered users for this notification on the specified processor.
 *  This function gets called internally from the Notify_sendEvent ()
 *  API. The Notify_sendEvent () function passes on the initialization
 *  request into the Notify driver identified by the Notify Handle.
 */
typedef int(*NotifyDriver_SendEvent) (struct notify_driver_object *handle,
		u16 procId, u32 eventNo, u32 payload, bool waitClear);

/*
 * This type defines the function to disable all events for the
 *  specified processor ID.
 *  This function gets called internally from the Notify_disable ()
 *  API. The Notify_disable () function passes on the request into the
 *  Notify driver identified by the Notify Handle.
 */
typedef u32(*NotifyDriver_Disable) (struct notify_driver_object *handle,
							u16 procId);

/*
 * This type defines the function to restore all events for the
 *  specified processor ID.
 *  This function gets called internally from the Notify_restore ()
 *  API. The Notify_restore () function passes on the request into the
 *  Notify driver identified by the Notify Handle.
 */
typedef void (*NotifyDriver_Restore) (struct notify_driver_object *handle,
				u32 key, u16 procId);

/*
 * This type defines the function to disable specified event for the
 *  specified processor ID.
 *  This function gets called internally from the Notify_disableEvent ()
 *  API. The Notify_disableEvent () function passes on the request into
 *  the Notify driver identified by the Notify Handle.
 */
typedef void (*NotifyDriver_DisableEvent) (struct notify_driver_object *handle,
			u16 procId, u32 eventNo);

/*
 * This type defines the function to enable specified event for the
 *  specified processor ID.
 *  This function gets called internally from the Notify_enableEvent ()
 *  API. The Notify_enableEvent () function passes on the request into
 *  the Notify driver identified by the Notify Handle.
 *
 */
typedef void (*NotifyDriver_EnableEvent) (struct notify_driver_object *handle,
		u16 procId, u32 eventNo);


/*
 * This structure defines the function table interface for the Notify
 * driver.
 * This function table interface must be implemented by each Notify
 * driver and registered with the Notify module.
 *
 */
struct notify_interface {
	NotifyDriver_RegisterEvent register_event;
	/* interface function registerEvent  */
	NotifyDriver_UnregisterEvent unregister_event;
	/* interface function unregisterEvent  */
	NotifyDriver_SendEvent send_event;
	/* interface function sendEvent  */
	NotifyDriver_Disable disable;
	/* interface function disable  */
	NotifyDriver_Restore restore;
	/* interface function restore  */
	NotifyDriver_DisableEvent disable_event;
	/* interface function disableEvent  */
	NotifyDriver_EnableEvent enable_event;
};


union notify_drv_procevents{
	struct {
		struct notify_shmdrv_attrs  attrs;
		struct notify_shmdrv_ctrl *ctrl_ptr;
       } shm_events;

	struct {
		/*Attributes */
		unsigned long int  num_events;
		unsigned long int  send_event_pollcount;
		/*Control Paramters */
		unsigned long int send_init_status ;
		struct notify_shmdrv_eventreg_mask  reg_mask ;
	} non_shm_events;
};


/*
 * This structure defines the Notify driver object and handle used
 * internally to contain all information required for the Notify driver
 * This object contains all information for the Notify module to be
 * able to identify and interact with the Notify driver.
 */
struct notify_driver_object {
	int is_init;
	struct notify_interface fn_table;
	char name[NOTIFY_MAX_NAMELEN];
	struct notify_driver_attrs attrs;
	u32 *disable_flag[NOTIFY_MAXNESTDEPTH];
	void *driver_object;
};


struct notify_drv_eventlist {
       unsigned long int event_handler_count;
       struct list_head listeners;
};



struct notify_drv_eventlistner{
	struct list_head element;
	fn_notify_cbck fn_notify_cbck;
	void *cbck_arg;
};


struct notify_drv_proc_module {

       unsigned long int proc_id;
       struct notify_drv_eventlist *event_list;
       struct notify_shmdrv_eventreg *reg_chart;
       union notify_drv_procevents events_obj;
};

/*
 *  Defines the Notify state object, which contains all the module
 *  specific information.
 */
struct notify_module_object {
	atomic_t ref_count;
	struct notify_config cfg;
	/* Notify configuration structure */
	struct notify_config def_cfg;
	/* Default module configuration */
	struct mutex *gate_handle;
	/* Handle of gate to be used for local thread safety */
	struct notify_driver_object drivers[NOTIFY_MAX_DRIVERS];
	/* Array of configured drivers. */
	u32 disable_depth;
	/* Current disable depth for Notify module. */
};
#endif  /* !defined (NOTIFYDRIVERDEFS_H) */

