/*
 *  messageq.h
 *
 *  The MessageQ module supports the structured sending and receiving of
 *  variable length messages. This module can be used for homogeneous or
 *  heterogeneous multi-processor messaging.
 *
 *  Copyright (C) 2009 Texas Instruments, Inc.
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

#ifndef _MESSAGEQ_H_
#define _MESSAGEQ_H_

/* Standard headers */
#include <linux/types.h>

/* Utilities headers */
#include <linux/list.h>

/* Syslink headers */
#include <listmp.h>
#include <messageq_transportshm.h>


/*!
 *  @def	MESSAGEQ_MODULEID
 *  @brief	Unique module ID.
 */
#define MESSAGEQ_MODULEID			(0xded2)

/* =============================================================================
 *  All success and failure codes for the module
 * =============================================================================
 */

/*!
 *  @def	MESSAGEQ_STATUSCODEBASE
 *  @brief	Error code base for MessageQ.
 */
#define MESSAGEQ_STATUSCODEBASE			(MESSAGEQ_MODULEID << 12)

/*!
 *  @def	MESSAGEQ_MAKE_FAILURE
 *  @brief	Macro to make error code.
 */
#define MESSAGEQ_MAKE_FAILURE(x)		((int) (0x80000000 + \
						(MESSAGEQ_STATUSCODEBASE + \
						(x))))

/*!
 *  @def	MESSAGEQ_MAKE_SUCCESS
 *  @brief	Macro to make success code.
 */
#define MESSAGEQ_MAKE_SUCCESS(x)		(MESSAGEQ_STATUSCODEBASE + (x))

/*!
 *  @def	MESSAGEQ_E_INVALIDARG
 *  @brief	Argument passed to a function is invalid.
 */
#define MESSAGEQ_E_INVALIDARG			MESSAGEQ_MAKE_FAILURE(1)

/*!
 *  @def	MESSAGEQ_E_MEMORY
 *  @brief	Memory allocation failed.
 */
#define MESSAGEQ_E_MEMORY			MESSAGEQ_MAKE_FAILURE(2)

/*!
 *  @def	MESSAGEQ_E_BUSY
 *  @brief	the name is already registered or not.
 */
#define MESSAGEQ_E_BUSY				MESSAGEQ_MAKE_FAILURE(3)

/*!
 *  @def	MESSAGEQ_E_FAIL
 *  @brief	Generic failure.
 */
#define MESSAGEQ_E_FAIL				MESSAGEQ_MAKE_FAILURE(4)

/*!
 *  @def	MESSAGEQ_E_NOTFOUND
 *  @brief	name not found in the nameserver.
 */
#define MESSAGEQ_E_NOTFOUND			MESSAGEQ_MAKE_FAILURE(5)

/*!
 *  @def	MESSAGEQ_E_INVALIDSTATE
 *  @brief	Module is not initialized.
 */
#define MESSAGEQ_E_INVALIDSTATE			MESSAGEQ_MAKE_FAILURE(6)

/*!
 *  @def	MESSAGEQ_E_NOTONWER
 *  @brief	Instance is not created on this processor.
 */
#define MESSAGEQ_E_NOTONWER			MESSAGEQ_MAKE_FAILURE(7)

/*!
 *  @def	MESSAGEQ_E_REMOTEACTIVE
 *  @brief	Remote opener of the instance has not closed the instance.
 */
#define MESSAGEQ_E_REMOTEACTIVE			MESSAGEQ_MAKE_FAILURE(8)

/*!
 *  @def	MESSAGEQ_E_INUSE
 *  @brief	Indicates that the instance is in use..
 */
#define MESSAGEQ_E_INUSE			MESSAGEQ_MAKE_FAILURE(9)

/*!
 *  @def	MESSAGEQ_E_INVALIDCONTEXT
 *  @brief	Indicates that the api is called with wrong handle
 */
#define MESSAGEQ_E_INVALIDCONTEXT		MESSAGEQ_MAKE_FAILURE(10)

/*!
 *  @def	MESSAGEQ_E_INVALIDMSG
 *  @brief	Indicates that an invalid msg has been specified
 *
 */
#define MESSAGEQ_E_INVALIDMSG			MESSAGEQ_MAKE_FAILURE(11)

/*!
 *  @def	MESSAGEQ_E_INVALIDHEAPID
 *  @brief	Indicates that an invalid heap has been specified
 */
#define MESSAGEQ_E_INVALIDHEAPID		MESSAGEQ_MAKE_FAILURE(12)

/*!
 *  @def	MESSAGEQ_E_INVALIDPROCID
 *  @brief	Indicates that an invalid proc id has been specified
 */
#define MESSAGEQ_E_INVALIDPROCID		MESSAGEQ_MAKE_FAILURE(13)

/*!
 *  @def	MESSAGEQ_E_MAXREACHED
 *  @brief	Indicates that all message queues are taken
 */
#define MESSAGEQ_E_MAXREACHED			MESSAGEQ_MAKE_FAILURE(14)

/*!
 *  @def	MESSAGEQ_E_UNREGISTERHEAPID
 *  @brief	Indicates that heap id has not been registered
 */
#define MESSAGEQ_E_UNREGISTERHEAPID		MESSAGEQ_MAKE_FAILURE(15)

/*!
 *  @def	MESSAGEQ_E_CANNOTFREESTATICMSG
 *  @brief	Indicates that static msg cannot be freed
 */
#define MESSAGEQ_E_CANNOTFREESTATICMSG		MESSAGEQ_MAKE_FAILURE(16)

/*!
 *  @def	MESSAGEQ_E_HEAPIDINVALID
 *  @brief	Indicates that the heap id is invalid
 */
#define MESSAGEQ_E_HEAPIDINVALID		MESSAGEQ_MAKE_FAILURE(17)

/*!
 *  @def	MESSAGEQ_E_PROCIDINVALID
 *  @brief	Indicates that the proc id is invalid
 */
#define MESSAGEQ_E_PROCIDINVALID		MESSAGEQ_MAKE_FAILURE(18)

/*!
 *  @def    MESSAGEQ_E_OSFAILURE
 *  @brief  Failure in OS call.
 */
#define MESSAGEQ_E_OSFAILURE			MESSAGEQ_MAKE_FAILURE(19)

/*!
 *  @def    MESSAGEQ_E_ALREADYEXISTS
 *  @brief  Specified entity already exists
 */
#define MESSAGEQ_E_ALREADYEXISTS		MESSAGEQ_MAKE_FAILURE(20)

/*!
 *  @def    MESSAGEQ_E_TIMEOUT
 *  @brief  Timeout while attempting to get a message
 */
#define MESSAGEQ_E_TIMEOUT			MESSAGEQ_MAKE_FAILURE(21)

/*!
 *  @def	MESSAGEQ_SUCCESS
 *  @brief	Operation successful.
 */
#define MESSAGEQ_SUCCESS			MESSAGEQ_MAKE_SUCCESS(0)

/*!
 *  @def    MESSAGEQ_S_ALREADYSETUP
 *  @brief  The MESSAGEQ module has already been setup in this process.
 */
#define MESSAGEQ_S_ALREADYSETUP			MESSAGEQ_MAKE_SUCCESS(1)


/* =============================================================================
 * Macros and types
 * =============================================================================
 */
/*!
 *	@brief	Mask to extract version setting
 */
#define MESSAGEQ_HEADERVERSION			0x2000u

/*!
 *	@brief	Mask to extract priority setting
 */
#define MESSAGEQ_PRIORITYMASK			0x3u

/*!
 *	@brief	Mask to extract priority setting
 */
#define MESSAGEQ_TRANSPORTPRIORITYMASK		0x01u

/*!
 *  Mask to extract version setting
 */
#define MESSAGEQ_VERSIONMASK			0xE000;

/*!
 *  Used as the timeout value to specify wait forever
 */
#define MESSAGEQ_FOREVER			(~((u32) 0))

/*!
 *  Invalid message id
 */
#define MESSAGEQ_INVALIDMSGID			0xFFFF

/*!
 * Invalid message queue
 */
#define MESSAGEQ_INVALIDMESSAGEQ		0xFFFF

/*!
 * Indicates that if maximum number of message queues are already created,
 * should allow growth to create additional Message Queue.
 */
#define MESSAGEQ_ALLOWGROWTH			(~((u32) 0))

/*!
 * Number of types of priority queues for each transport
 */
#define MESSAGEQ_NUM_PRIORITY_QUEUES		2


/* =============================================================================
 * Structures & Enums
 * =============================================================================
 */
/*!
 * Message priority
 */
enum messageq_priority {
	MESSAGEQ_NORMALPRI = 0,
	/*!< Normal priority message */
	MESSAGEQ_HIGHPRI = 1,
	/*!< High priority message */
	MESSAGEQ_RESERVEDPRI = 2,
	/*!< Reserved value for message priority */
	MESSAGEQ_URGENTPRI = 3
	/*!< Urgent priority message */
};

/*! Structure which defines the first field in every message */
struct msgheader {
	u32 reserved0;
	/*!< Reserved field */
	u32 reserved1;
	/*!< Reserved field */
	u32 msg_size;
	/*!< Size of the message (including header) */
	u16 flags;
	/*!< Flags */
	u16 msg_id;
	/*!< Maximum length for Message queue names */
	u16 dst_id;
	/*!< Maximum length for Message queue names */
	u16 dst_proc;
	/*!< Maximum length for Message queue names */
	u16 reply_id;
	/*!< Maximum length for Message queue names */
	u16 reply_proc;
	/*!< Maximum length for Message queue names */
	u16 src_proc;
	/*!< Maximum length for Message queue names */
	u16 heap_id;
	/*!< Maximum length for Message queue names */
	u32       reserved;
	/*!< Reserved field */
};
/*! Structure which defines the first field in every message */
#define messageq_msg	struct msgheader *
/*typedef struct msgheader *messageq_msg;*/


/*!
 *  @brief	Structure defining config parameters for the MessageQ Buf module.
 */
struct messageq_config {
	u16 num_heaps;
	/*!
	*  Number of heapIds in the system
	*
	*  This allows MessageQ to pre-allocate the heaps table.
	*  The heaps table is used when registering heaps.
	*
	*  The default is 1 since generally all systems need at least
	*  one heap.
	*
	*  There is no default heap, so unless the system is only using
	*  staticMsgInit, the application must register a heap.
	*/

	u32 max_runtime_entries;
	/*!
	*  Maximum number of MessageQs that can be dynamically created
	*/

	struct mutex *name_table_gate;
	/*!
	*  Gate used to make the name table thread safe. If NULL is passed, gate
	*  will be created internally.
	*/

	u32 max_name_len;
	/*!
	*  Maximum length for Message queue names
	*/
};

struct messageq_params {
	u32 reserved;
	/*!< No parameters required currently. Reserved field. */
	u32 max_name_len;
	/*!< Maximum length for Message queue names */
};

/* =============================================================================
 *  APIs
 * =============================================================================
 */
/* Functions to get the configuration for messageq setup */
void messageq_get_config(struct messageq_config *cfg);

/* Function to setup the MessageQ module. */
int messageq_setup(const struct messageq_config *cfg);

/* Function to destroy the MessageQ module. */
int messageq_destroy(void);

/* Initialize this config-params structure with supplier-specified
 * defaults before instance creation.
 */
void messageq_params_init(void *messageq_handle,
				struct messageq_params *params);

/* Create a message queue */
void *messageq_create(char *name, const struct messageq_params *params);

/* Deletes a instance of MessageQ module. */
int messageq_delete(void **messageq_handleptr);

/* Allocates a message from the heap */
messageq_msg messageq_alloc(u16 heapId, u32 size);

/* Frees a message back to the heap */
int messageq_free(messageq_msg msg);

/* Open a message queue */
int messageq_open(char *name, u32 *queue_id);

/* Close an opened message queue handle */
void messageq_close(u32 *queue_id);

/* Initializes a message not obtained from MessageQ_alloc */
void messageq_static_msg_init(messageq_msg msg, u32 size);

/* Place a message onto a message queue */
int messageq_put(u32 queueId, messageq_msg msg);

/* Gets a message for a message queue and blocks if the queue is empty */
int messageq_get(void *messageq_handle, messageq_msg *msg,
							u32 timeout);

/* Register a heap with MessageQ */
int messageq_register_heap(void *heap_handle, u16 heap_id);

/* Unregister a heap with MessageQ */
int messageq_unregister_heap(u16 heapId);

/* Returns the number of messages in a message queue */
int messageq_count(void *messageq_handle);

/* Set the destination queue of the message. */
void messageq_set_reply_queue(void *messageq_handle, messageq_msg msg);

/* Get the queue Id of the message. */
u32 messageq_get_queue_id(void *messageq_handle);

/* Get the proc Id of the message. */
u16 messageq_get_proc_id(void *messageq_handle);

/*
 *  Functions to set Message properties
 */
/*!
 *  @brief   Returns the MessageQ_Queue handle of the destination
 *           message queue for the specified message.
 */
u32 messageq_get_dst_queue(messageq_msg msg);

/*!
 *  @brief   Returns the message ID of the specified message.
 */
u16 messageq_get_msg_id(messageq_msg msg);

/*!
 *  @brief   Returns the size of the specified message.
 */
u32 messageq_get_msg_size(messageq_msg msg);

/*!
 *  @brief	Gets the message priority of a message
 */
u32 messageq_get_msg_pri(messageq_msg msg);

/*!
 *  @brief   Returns the MessageQ_Queue handle of the destination
 *           message queue for the specified message.
 */
u32 messageq_get_reply_queue(messageq_msg msg);

/*!
 *  @brief   Sets the message ID in the specified message.
 */
void messageq_set_msg_id(messageq_msg msg, u16 msg_id);
/*!
 *  @brief   Sets the message priority in the specified message.
 */
void messageq_set_msg_pri(messageq_msg msg, u32 priority);

/* =============================================================================
 *  APIs called internally by MessageQ transports
 * =============================================================================
 */
/* Register a transport with MessageQ */
int messageq_register_transport(void *messageq_transportshm_handle,
					u16 proc_id, u32 priority);

/* Unregister a transport with MessageQ */
int messageq_unregister_transport(u16 proc_id, u32 priority);


#endif /* _MESSAGEQ_H_ */
