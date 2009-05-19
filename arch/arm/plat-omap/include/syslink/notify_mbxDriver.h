/*
 * notify_mbxDriver.h
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

#ifndef NOTIFY_SHMDRIVER_H_
#define NOTIFY_SHMDRIVER_H_



/* Notify*/
#include <syslink/GlobalTypes.h>
#include <syslink/notifyerr.h>
#include <syslink/notify_driverdefs.h>

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
 *  name   notify_shmdrv_attrs
 *
 *  desc   This structure defines the attributes for Notify Shared Memory
 *          Mailbox driver.
 *          These attributes are passed to the driver when notify_driver_init ()
 *          is called for this driver.
 *
 *  field  shmBaseAddr
 *              Shared memory address base for the NotifyShmDrv driver. This
 *              must be the start of shared memory as used by both connected
 *              processors, and the same must be specified on both sides when
 *              initializing the NotifyShmDrv driver.
 *  field  shmSize
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
	unsigned long int    shmBaseAddr ;
	unsigned long int    shmSize ;
	unsigned long int    num_events ;
	unsigned long int    send_event_pollcount ;
};


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
	REG unsigned long int  flag     ;
	REG unsigned long int  payload  ;
	REG unsigned long int  reserved ;
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
	REG unsigned long int mask ;
	REG unsigned long int enable_mask ;
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
	unsigned long int     reg_event_no ;
	unsigned long int     reserved ;
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
*  field  recvInitStatus
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
	struct notify_shmdrv_event_entry *selfEventChart ;
	struct notify_shmdrv_event_entry *otherEventChart ;
	unsigned long int                        recvInitStatus ;
	unsigned long int                        send_init_status ;
	/*ADD_PADDING(padding, NOTIFYSHMDRV_CTRL_PADDING)*/
	struct notify_shmdrv_eventreg_mask     reg_mask ;
};

/*
*  name   notify_shmdrv_ctrl
*
*  desc   Defines the NotifyShmDrv control structure, which contains all
*          information shared between the two connected processors
*          This structure is shared between the two processors.
*
*  field  otherProcCtrl
*              Control structure for other processor
*  field  selfProcCtrl
*              Control structure for self processor
*
*/
struct notify_shmdrv_ctrl {
	struct NotifyShmDrv_ProcCtrl otherProcCtrl ;
	struct NotifyShmDrv_ProcCtrl selfProcCtrl ;
};


/*
 *  name   notify_mbxdrv_init
 *
 *  desc   Top-level initialization function for the Notify shared memory
 *          mailbox driver.
 *          This can be plugged in as the user init function.
 *
 *  arg    None.
 *
 *  ret    None.
 *
 *  enter  Notify module must have been initialized before this call
 *
 *  leave  On success, the driver is registered with the Notify module.
 *
 *  see    notify_mbxdrv_exit ()
 *
 */

void notify_mbxdrv_init(void) ;

/*
 *  name   notify_mbxdrv_exit
 *
 *  desc   Top-level finalization function for the Notify shared memory
 *          mailbox driver.
 *
 *  arg    None.
 *
 *  ret    None.
 *
 *  enter  Notify module must have been initialized before this call
 *
 *  leave  On success, the driver is unregistered with the Notify module.
 *
 *  see    notify_mbxdrv_init ()
 *
 */

void notify_mbxdrv_exit(void) ;



/*
*  name   notify_mbxdrv_driver_init
*
*  desc   Initialization function for the Notify shared memory mailbox driver.
*
*  arg    driver_name
*              Name of the Notify driver to be initialized.
*  arg    config
*              Configuration information for the Notify driver. This contains
*              generic information as well as information specific to the type
*              of Notify driver, as defined by the driver provider.
*  arg    driver_object
*              Location to receive the pointer to the Notify-driver specific
*              object.
*
*  ret    NOTIFY_SOK
*              The Notify driver has been successfully initialized
*          NOTIFY_EMEMORY
*              Operation failed due to a memory error
*          NOTIFY_EPOINTER
*              Invalid pointer passed
*          NOTIFY_ECONFIG
*              Invalid configuration information passed
*          NOTIFY_EINVALIDARG
*              Invalid arguments
*          NOTIFY_EFAIL
*              General failure
*
*  enter  Notify module must have been initialized before this call
*          driver_name must be a valid pointer
*          config must be valid
*          driver_object must be a valid pointer.
*
*  leave  On success, the driver must be initialized.
*
*  see    notify_interface, notify_driver_init ()
*
*/

signed long int notify_mbxdrv_driver_init(char *driver_name,
				struct notify_config *config,
				void **driver_object) ;

/*
*  name   notify_mbxdrv_driver_exit
*
*  desc   Finalization function for the Notify driver.
*
*  arg    handle
*              Handle to the Notify driver
*
*  ret    NOTIFY_SOK
*              The Notify driver has been successfully initialized
*          NOTIFY_EMEMORY
*              Operation failed due to a memory error
*          NOTIFY_EHANDLE
*              Invalid Notify handle specified
*          NOTIFY_EINVALIDARG
*              Invalid arguments
*          NOTIFY_EFAIL
*              General failure
*
*  enter  The Notify module and driver must be initialized before calling
*          this function.
*          handle must be a valid Notify driver handle
*
*  leave  On success, the driver must be initialized.
*
*  see    notify_interface, notify_driver_exit ()
*
*/

signed long int notify_mbxdrv_driver_exit(
				struct notify_driver_handle *handle) ;

/*
*  func   notify_mbxdrv_register_event
*
*  desc   Register a callback for an event with the Notify driver.
*
*  arg    handle
*              Handle to the Notify driver
*  arg    proc_id
*              ID of the processor from which notifications can be received on
*              this event.
*  arg    event_no
*              Event number to be registered.
*  arg    fn_notify_cbck
*              Callback function to be registered for the specified event.
*  arg    cbck_arg
*              Optional argument to the callback function to be registered for
*              the specified event. This argument shall be passed to each
*              invocation of the callback function.
*
*  ret    NOTIFY_SOK
*              Operation successfully completed
*          NOTIFY_EINVALIDARG
*              Invalid argument
*          NOTIFY_EMEMORY
*              Operation failed due to a memory error
*          NOTIFY_EHANDLE
*              Invalid Notify handle specified
*          NOTIFY_EFAIL
*              General failure
*
*  enter  The Notify module and driver must be initialized before calling
*          this function.
*          handle must be a valid Notify driver handle
*          fn_notify_cbck must be a valid pointer.
*          The event must be supported by the Notify driver.
*
*  leave  On success, the event must be registered with the Notify driver
*
*  see    notify_interface, notify_register_event ()
*
*/

signed long int notify_mbxdrv_register_event(
				struct notify_driver_handle *handle,
				unsigned long int        proc_id,
				unsigned long int              event_no,
				fn_notify_cbck        fn_notify_cbck,
				void *cbck_arg) ;

/*
*  func   notify_mbxdrv_unregevent
*
*  desc   Unregister a callback for an event with the Notify driver.
*
*  arg    handle
*              Handle to the Notify driver
*  arg    proc_id
*              ID of the processor from which notifications can be received on
*              this event.
*  arg    event_no
*              Event number to be un-registered.
*  arg    fn_notify_cbck
*              Callback function to be un-registered for the specified event.
*  arg    cbck_arg
*              Optional argument to the callback function to be un-registered
*              for the specified event.
*
*  ret    NOTIFY_SOK
*              Operation successfully completed
*          NOTIFY_EINVALIDARG
*              Invalid argument
*          NOTIFY_EMEMORY
*              Operation failed due to a memory error
*          NOTIFY_ENOTFOUND
*              Specified event registration was not found
*          NOTIFY_EHANDLE
*              Invalid Notify handle specified
*          NOTIFY_EFAIL
*              General failure
*
*  enter  The Notify module and driver must be initialized before calling this
*          function.
*          handle must be a valid Notify driver handle
*          fn_notify_cbck must be a valid pointer.
*          The event must be supported by the Notify driver.
*          The event must have been registered with the Notify driver earlier.
*
*  leave  On success, the event must be unregistered with the Notify driver.
*
*  see    notify_interface, notify_unregister_event ()
*
*/

signed long int notify_mbxdrv_unregevent(
				struct notify_driver_handle *handle,
				unsigned long int        proc_id,
				unsigned long int        event_no,
				fn_notify_cbck        fn_notify_cbck,
				void *cbck_arg) ;

/*
*  func   notify_mbxdrv_sendevent
*
*  desc   Send a notification event to the registered users for this
*          notification on the specified processor.
*
*  arg    handle
*              Handle to the Notify driver
*  arg    proc_id
*              ID of the processor to which the notification is to be sent.
*  arg    event_no
*              Event number to be used.
*  arg    payload
*              Data to be sent along with the event.
*  arg    wait_clear
*              Indicates whether the function should wait for previous event
*              to be cleared, or sending single event is sufficient without
*              payload.
*
*  ret    NOTIFY_SOK
*              Operation successfully completed
*          NOTIFY_EDRIVERINIT
*              Remote Notify driver is not setup to receive events.
*          NOTIFY_ENOTREADY
*              Other side is not ready to receive an event. This can be due to
*              one of two reasons:
*              1. No client is registered for this event on the other side
*              2. Other side has disabled the event
*          NOTIFY_EINVALIDARG
*              Invalid argument
*          NOTIFY_EHANDLE
*              Invalid Notify handle specified
*          NOTIFY_EFAIL
*              General failure
*
*  enter  The Notify module and driver must be initialized before calling this
*          function.
*          handle must be a valid Notify driver handle
*          The event must be supported by the Notify driver.
*
*  leave  On success, the event must be sent to the specified processor.
*
*  see    notify_interface, notify_sendevent ()
*
*/

signed long int notify_mbxdrv_sendevent(struct notify_driver_handle *handle,
			unsigned long int        proc_id,
			unsigned long int              event_no,
			unsigned long int              payload,
			short int                wait_clear) ;

/*
*  func   notify_mbxdrv_disable
*
*  desc   Disable all events for this Notify driver.
*
*  arg    handle
*              Handle to the Notify driver
*
*  ret    flags
*              Flags to be provided when notify_mbxdrv_restore is called.
*
*  enter  The Notify module and driver must be initialized before calling this
*          function.
*          handle must be a valid Notify driver handle
*
*  leave  On success, all events for the Notify driver must be disabled
*
*  see    notify_interfacej, notify_disable ()
*
*/

void * notify_mbxdrv_disable(struct notify_driver_handle *handle) ;

/*
*  func   notify_mbxdrv_restore
*
*  desc   Restore the Notify driver to the state before the last disable was
*          called.
*
*  arg    handle
*              Handle to the Notify driver
*  arg    flags
*            Flags returned from the call to last notify_mbxdrv_disable in order
*            to restore the Notify driver to the state before the last
*            disable call.
*
*  ret    NOTIFY_SOK
*              Operation successfully completed
*          NOTIFY_EHANDLE
*              Invalid Notify handle specified
*          NOTIFY_EPOINTER
*              Invalid pointer passed
*          NOTIFY_EFAIL
*              General failure
*
*  enter  The Notify module and driver must be initialized before calling this
*          function.
*          handle must be a valid Notify driver handle
*          flags must be the same as what was returned from the previous
*          notify_mbxdrv_disable call.
*
*  leave  On success, all events for the Notify driver must be restored to
*          the state as indicated by the provided flags.
*
*  see    notify_interface, notify_restore ()
*
*/

signed long int notify_mbxdrv_restore(struct notify_driver_handle *handle,
					void *flags) ;

/*
*  func   notify_mbxdrv_disable_event
*
*  desc   Disable a specific event for this Notify driver.
*
*  arg    handle
*              Handle to the Notify driver
*  arg    proc_id
*              ID of the processor.
*  arg    event_no
*              Event number to be disabled.
*
*  ret    NOTIFY_SOK
*              Operation successfully completed
*          NOTIFY_EINVALIDARG
*              Invalid argument
*          NOTIFY_EHANDLE
*              Invalid Notify handle specified
*          NOTIFY_EFAIL
*              General failure
*
*  enter  The Notify module and driver must be initialized before calling this
*          function.
*          handle must be a valid Notify driver handle
*          The event must be supported by the Notify driver.
*
*  leave  On success, the event must be disabled.
*
*  see    notify_interface, notify_disable_event ()
*
*/

signed long int notify_mbxdrv_disable_event(
			struct notify_driver_handle *handle,
			unsigned long int       proc_id,
			unsigned long int   event_no) ;

/*
*  func   notify_mbxdrv_enable_event
*
*  desc   Enable a specific event for this Notify driver.
*
*  arg    handle
*              Handle to the Notify driver
*  arg    proc_id
*              ID of the processor.
*  arg    event_no
*              Event number to be enabled.
*
*  ret    NOTIFY_SOK
*              Operation successfully completed
*          NOTIFY_EINVALIDARG
*              Invalid argument
*          NOTIFY_EHANDLE
*              Invalid Notify handle specified
*          NOTIFY_EFAIL
*              General failure
*
*  enter  The Notify module and driver must be initialized before calling this
*          function.
*          handle must be a valid Notify driver handle
*          The event must be supported by the Notify driver.
*
*  leave  On success, the event must be enabled.
*
*  see    notify_interface, notify_enable_event ()
*
*/

signed long int notify_mbxdrv_enable_event(
			struct notify_driver_handle *handle,
			unsigned long int    proc_id,
			unsigned long int    event_no) ;


/*
*  func   notify_mbxdrv_debug
*
*  desc   Print debug information for the Notify driver.
*
*  arg    handle
*              Handle to the Notify driver
*
*  ret    NOTIFY_SOK
*              Operation successfully completed
*          NOTIFY_EHANDLE
*              Invalid Notify handle specified
*          NOTIFY_EFAIL
*              General failure
*
*  enter  The Notify module and driver must be initialized before calling this
*          function.
*          handle must be a valid Notify driver handle
*
*  leave  None.
*
*  see    notify_interface, notify_debug ()
*
*/

signed long int notify_mbxdrv_debug(struct notify_driver_handle *handle) ;





#endif  /* !defined  NOTIFY_SHMDRIVER_H_ */

