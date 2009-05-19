/*
 *  messageq_transportshm.h
 *
 *  MessageQ shared memory based physical transport for
 *  communication with the remote processor.
 *
 *  This file contains the declarations of types and APIs as part
 *  of interface of the MessageQ shared memory transport.
 *
 *  Copyright (C) 2008-2009 Texas Instruments, Inc.
 *
 *  This package is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 *  IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 *  WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE.
 */

#ifndef _MESSAGEQ_TRANSPORTSHM_H_
#define _MESSAGEQ_TRANSPORTSHM_H_

/* Standard headers */
#include <linux/types.h>

/* Utilities headers */
#include <linux/list.h>

/* =============================================================================
 *  All success and failure codes for the module
 * =============================================================================
 */
/*!
 *  @def	MESSAGEQ_TRANSPORTSHM_MODULEID
 *  @brief  Unique module ID.
 */
#define MESSAGEQ_TRANSPORTSHM_MODULEID		(0x0a7a)

/* =============================================================================
 *  All success and failure codes for the module
 * =============================================================================
 */
/*!
 *  @def	MESSAGEQ_TRANSPORTSHM_STATUSCODEBASE
 *  @brief  Error code base for MessageQ.
 */
#define MESSAGEQ_TRANSPORTSHM_STATUSCODEBASE \
					(MESSAGEQ_TRANSPORTSHM_MODULEID << 12)

/*!
 *  @def	MESSAGEQ_TRANSPORTSHM_MAKE_FAILURE
 *  @brief  Macro to make error code.
 */
#define MESSAGEQ_TRANSPORTSHM_MAKE_FAILURE(x)	((int)  (0x80000000 \
				+ (MESSAGEQ_TRANSPORTSHM_STATUSCODEBASE \
				+ (x))))

/*!
 *  @def	MESSAGEQ_TRANSPORTSHM_MAKE_SUCCESS
 *  @brief  Macro to make success code.
 */
#define MESSAGEQ_TRANSPORTSHM_MAKE_SUCCESS(x) \
				(MESSAGEQ_TRANSPORTSHM_STATUSCODEBASE + (x))

/*!
 *  @def	MESSAGEQ_TRANSPORTSHM_E_INVALIDARG
 *  @brief  Argument passed to a function is invalid.
 */
#define MESSAGEQ_TRANSPORTSHM_E_INVALIDARG \
				MESSAGEQ_TRANSPORTSHM_MAKE_FAILURE(1)

/*!
 *  @def	MESSAGEQ_TRANSPORTSHM_E_INVALIDSIZE
 *  @brief  Invalid shared address size
 */
#define MESSAGEQ_TRANSPORTSHM_E_INVALIDSIZE \
				MESSAGEQ_TRANSPORTSHM_MAKE_FAILURE(2)

/*!
 *  @def	MESSAGEQ_TRANSPORTSHM_E_INVALIDSTATE
 *  @brief  Module is not initialized.
 */
#define MESSAGEQ_TRANSPORTSHM_E_INVALIDSTATE \
				MESSAGEQ_TRANSPORTSHM_MAKE_FAILURE(3)

/*!
 *  @def	MESSAGEQ_TRANSPORTSHM_E_BADVERSION
 *  @brief  Versions don't match
 */
#define MESSAGEQ_TRANSPORTSHM_E_BADVERSION \
				MESSAGEQ_TRANSPORTSHM_MAKE_FAILURE(4)

/*!
 *  @def	MESSAGEQ_TRANSPORTSHM_E_FAIL
 *  @brief  General Failure
*/
#define MESSAGEQ_TRANSPORTSHM_E_FAIL \
				MESSAGEQ_TRANSPORTSHM_MAKE_FAILURE(5)

/*!
 *  @def	MESSAGEQ_TRANSPORTSHM_E_MEMORY
 *  @brief  Memory allocation failed
 */
#define MESSAGEQ_TRANSPORTSHM_E_MEMORY \
				MESSAGEQ_TRANSPORTSHM_MAKE_FAILURE(6)

/*!
 *  @def	MESSAGEQ_TRANSPORTSHM_E_OSFAILURE
 *  @brief  Failure in OS call.
 */
#define MESSAGEQ_TRANSPORTSHM_E_OSFAILURE \
				MESSAGEQ_TRANSPORTSHM_MAKE_FAILURE(7)

/*!
 *  @def	MESSAGEQ_TRANSPORTSHM_E_HANDLE
 *  @brief  Invalid handle specified.
 */
#define MESSAGEQ_TRANSPORTSHM_E_HANDLE \
				MESSAGEQ_TRANSPORTSHM_MAKE_FAILURE(8)

/*!
 *  @def	MESSAGEQ_TRANSPORTSHM_SUCCESS
 *  @brief  Operation successful.
 */
#define MESSAGEQ_TRANSPORTSHM_SUCCESS \
				MESSAGEQ_TRANSPORTSHM_MAKE_SUCCESS(0)

/*!
 *  @def	MESSAGETRANSPORTSHM_S_ALREADYSETUP
 *  @brief  The MESSAGETRANSPORTSHM module has
 *	  already been setup in this process.
 */
#define MESSAGEQ_TRANSPORTSHM_S_ALREADYSETUP \
				MESSAGEQ_TRANSPORTSHM_MAKE_SUCCESS(1)


/* =============================================================================
 * Structures & Enums
 * =============================================================================
 */
/*!
 *  @brief  Module configuration structure.
 */
struct  messageq_transportshm_config {
	void *gate_handle;
	/*!< Handle of gate to be used for local thread safety. If provided as
	 NULL, gate handle is created internally. */
};

/*!
 *  @brief  Structure defining config parameters for the MessageQ transport
 *  instances.
 */
struct messageq_transportshm_params {
	void *gate;
	/*!< Gate used for critical region management of the shared memory */
	void *shared_addr;
	/*!<  Address of the shared memory. The creator must supply the shared
	*    memory that this will use for maintain shared state information.
	*/
	u32 shared_addr_size;
	/*!<  Size of shared region provided. */
	u32 notify_event_no;
	/*!<  Notify event number to be used by the transport */
	void *notify_driver;
	/*!<  Notify driver to be used by the transport */
	u32 priority;
	/*!<  Priority of messages supported by this transport */
};

/*!
 *  @brief  Structure defining Transport status values
 */
enum messageq_transportshm_status {
	messageq_transportshm_status_INIT,
	/*!< MessageQ transport Shm instance has not not completed
	* initialization. */
	messageq_transportshm_status_UP,
	/*!< MessageQ transport Shm instance is up and functional. */
	messageq_transportshm_status_DOWN,
	/*!<  MessageQ transport Shm instance is down and not functional. */
	messageq_transportshm_status_RESETTING
	/*!<  MessageQ transport Shm instance was up at one point and is in
	* process of resetting.
	*/
};

/* =============================================================================
 *  APIs called by applications
 * =============================================================================
 */
/* Function to get the default configuration for the MessageQTransportShm
 * module. */
void messageq_transportshm_get_config(struct messageq_transportshm_config *cfg);

/* Function to setup the MessageQTransportShm module. */
int messageq_transportshm_setup(struct messageq_transportshm_config *cfg);

/* Function to destroy the MessageQTransportShm module. */
int messageq_transportshm_destroy(void);

/* Get the default parameters for the NotifyShmDriver. */
void messageq_transportshm_params_init(void *mqtshm_handle,
				struct messageq_transportshm_params *params);

/* Create an instance of the MessageQTransportShm. */
void *messageq_transportshm_create(u16 proc_id,
			const struct messageq_transportshm_params *params);

/* Delete an instance of the MessageQTransportShm. */
int messageq_transportshm_delete(void **mqtshm_handleptr);

/* Get the shared memory requirements for the MessageQTransportShm. */
u32 messageq_transportshm_shared_mem_req(const
				struct messageq_transportshm_params *params);

/* =============================================================================
 *  APIs called internally by MessageQ module.
 * =============================================================================
 */
/* Put msg to remote list */
int messageq_transportshm_put(void *mqtshm_handle, void *msg);

/* Control Function */
bool messageq_transportshm_control(void *mqtshm_handle, u32 cmd,
					u32 *cmd_arg);

/* Get current status of the MessageQTransportShm */
enum messageq_transportshm_status messageq_transportshm_get_status(
						void *mqtshm_handle);

#endif /* _MESSAGEQ_TRANSPORTSHM_H_ */
