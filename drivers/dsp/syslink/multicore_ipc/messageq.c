/*
 *  messageq.c
 *
 *  The messageQ module supports the structured sending and receiving of
 *  variable length messages. This module can be used for homogeneous or
 *  heterogeneous multi-processor messaging.
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

/*!
 *  MessageQ provides more sophisticated messaging than other modules. It is
 *  typically used for complex situations such as multi-processor messaging.
 *
 *  The following are key features of the MessageQ module:
 *  -Writers and readers can be relocated to another processor with no
 *   runtime code changes.
 *  -Timeouts are allowed when receiving messages.
 *  -Readers can determine the writer and reply back.
 *  -Receiving a message is deterministic when the timeout is zero.
 *  -Messages can reside on any message queue.
 *  -Supports zero-copy transfers.
 *  -Can send and receive from any type of thread.
 *  -Notification mechanism is specified by application.
 *  -Allows QoS (quality of service) on message buffer pools. For example,
 *   using specific buffer pools for specific message queues.
 *
 *  Messages are sent and received via a message queue. A reader is a thread
 *  that gets (reads) messages from a message queue. A writer is a thread that
 *  puts (writes) a message to a message queue. Each message queue has one
 *  reader and can have many writers. A thread may read from or write to
 *  multiple message queues.
 *
 *  Conceptually, the reader thread owns a message queue. The reader thread
 *  creates a message queue. Writer threads  a created message queues to
 *  get access to them.
 *
 *  Message queues are identified by a system-wide unique name. Internally,
 *  MessageQ uses the NameServer module for managing
 *  these names. The names are used for opening a message queue. Using
 *  names is not required.
 *
 *  Messages must be allocated from the MessageQ module. Once a message is
 *  allocated, it can be sent on any message queue. Once a message is sent, the
 *  writer loses ownership of the message and should not attempt to modify the
 *  message. Once the reader receives the message, it owns the message. It
 *  may either free the message or re-use the message.
 *
 *  Messages in a message queue can be of variable length. The only
 *  requirement is that the first field in the definition of a message must be a
 *  MsgHeader structure. For example:
 *  typedef struct MyMsg {
 *	  messageq_MsgHeader header;
 *	  ...
 *  } MyMsg;
 *
 *  The MessageQ API uses the messageq_MsgHeader internally. Your application
 *  should not modify or directly access the fields in the messageq_MsgHeader.
 *
 *  All messages sent via the MessageQ module must be allocated from a
 *  Heap implementation. The heap can be used for
 *  other memory allocation not related to MessageQ.
 *
 *  An application can use multiple heaps. The purpose of having multiple
 *  heaps is to allow an application to regulate its message usage. For
 *  example, an application can allocate critical messages from one heap of fast
 *  on-chip memory and non-critical messages from another heap of slower
 *  external memory
 *
 *  MessageQ does support the usage of messages that allocated via the
 *  alloc function. Please refer to the static_msg_init
 *  function description for more details.
 *
 *  In a multiple processor system, MessageQ communications to other
 *  processors via MessageQ_transport} instances. There must be one and
 *  only one IMessageQ_transport instance for each processor where communication
 *  is desired.
 *  So on a four processor system, each processor must have three
 *  IMessageQ_transport instance.
 *
 *  The user only needs to create the IMessageQ_transport instances. The
 *  instances are responsible for registering themselves with MessageQ.
 *  This is accomplished via the register_transport function.
 */



/* Standard headers */
#include <linux/types.h>
#include <linux/module.h>

/* Utilities headers */
#include <linux/string.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/semaphore.h>

/* Syslink headers */
#include <syslink/atomic_linux.h>

/* Module level headers */
#include <nameserver.h>
#include <multiproc.h>
#include <messageq_transportshm.h>
#include <heap.h>
#include <messageq.h>


/*! @brief Macro to make a correct module magic number with refCount */
#define MESSAGEQ_MAKE_MAGICSTAMP(x) ((MESSAGEQ_MODULEID << 12u) | (x))

/* =============================================================================
 * Globals
 * =============================================================================
 */
/*!
 *  @brief  Name of the reserved NameServer used for MessageQ.
 */
#define MESSAGEQ_NAMESERVER  "MessageQ"

/*!
 *  @brief  Number of types of priority queues for each transport
 */
#define MESSAGEQ_NUM_PRIORITY_QUEUES  2


/* =============================================================================
 * Structures & Enums
 * =============================================================================
 */
/* structure for MessageQ module state */
struct messageq_module_object {
	atomic_t ref_count;
	/*!< Reference count */
	void *ns_handle;
	/*!< Handle to the local NameServer used for storing GP objects */
	struct mutex *gate_handle;
	/*!< Handle of gate to be used for local thread safety */
	struct messageq_config cfg;
	/*!< Current config values */
	struct messageq_config default_cfg;
	/*!< Default config values */
	struct messageq_params default_inst_params;
	/*!< Default instance creation parameters */
	void *transports[MULTIPROC_MAXPROCESSORS][MESSAGEQ_NUM_PRIORITY_QUEUES];
	/*!< Transport to be set in messageq_registerTransport */
	void **queues; /*messageq_handle *queues;*/
	/*!< Grow option */
	void **heaps; /*Heap_Handle *heaps; */
	/*!< Heap to be set in messageq_registerHeap */
	u16 num_queues;
	/*!< Heap to be set in messageq_registerHeap */
	u16 num_heaps;
	/*!< Number of Heaps */
	bool can_free_queues;
	/*!< Grow option */
};

/*!
 *  @brief	Structure for the Handle for the MessageQ.
 */
struct messageq_object {
	struct messageq_params params;
	/*! Instance specific creation parameters */
	char *name;
	/*! MessageQ name */
	u32 queue;
	/* Unique id */
	struct list_head normal_list;
	/* Embedded List objects */
	struct list_head high_list;
	/* Embedded List objects */
	/*OsalSemaphore_Handle synchronizer;*/
	struct semaphore *synchronizer;
	/* Semaphore used for synchronizing message events */
};


static struct messageq_module_object messageq_state = {
				.ns_handle = NULL,
				.gate_handle = NULL,
				.queues = NULL,
				.heaps = NULL,
				.num_queues = 1,
				.num_heaps = 1,
				.can_free_queues = false,
				.default_cfg.num_heaps = 1,
				.default_cfg.max_runtime_entries = 32,
				.default_cfg.name_table_gate = NULL,
				.default_cfg.max_name_len = 32,
				.default_inst_params.reserved = 0
};


/* =============================================================================
 * Constants
 * =============================================================================
 */
/*
 *  Used to denote a message that was initialized
 *  with the messageq_static_msg_init function.
 */
#define MESSAGEQ_STATICMSG 0xFFFF


/* =============================================================================
 * Forward declarations of internal functions
 * =============================================================================
 */
/*
 *  @brief   Grow the MessageQ table
 *
 *  @sa	  messageq_create
 */
static u16 _messageq_grow(struct messageq_object *obj);

/* =============================================================================
 * APIS
 * =============================================================================
 */
/*
 * ======== messageq_get_config ========
 *  Purpose:
 *  Function to get the default configuration for the MessageQ
 *  module.
 *
 *  This function can be called by the application to get their
 *  configuration parameter to MessageQ_setup filled in by the
 *  MessageQ module with the default parameters. If the user does
 *  not wish to make any change in the default parameters, this API
 *  is not required to be called.
 *  the listmp_sharedmemory module.
 */
void messageq_get_config(struct messageq_config *cfg)
{
	if (WARN_ON(cfg == NULL))
		goto exit;

	if (atomic_cmpmask_and_lt(&(messageq_state.ref_count),
				MESSAGEQ_MAKE_MAGICSTAMP(0),
				MESSAGEQ_MAKE_MAGICSTAMP(1)) == true) {
		/* (If setup has not yet been called) */
		memcpy(cfg, &messageq_state.default_cfg,
			sizeof(struct messageq_config));
	} else {
		memcpy(cfg, &messageq_state.cfg,
			sizeof(struct messageq_config));
	}
	return;

exit:
	printk(KERN_ERR "messageq_get_config: Argument of type "
		"(struct messageq_config *) passed is null!\n");
}
EXPORT_SYMBOL(messageq_get_config);

/*
 * ======== messageq_setup ========
 *  Purpose:
 *  Function to setup the MessageQ module.
 *
 *  This function sets up the MessageQ module. This function must
 *  be called before any other instance-level APIs can be invoked.
 *  Module-level configuration needs to be provided to this
 *  function. If the user wishes to change some specific config
 *  parameters, then MessageQ_getConfig can be called to get the
 *  configuration filled with the default values. After this, only
 *  the required configuration values can be changed. If the user
 *  does not wish to make any change in the default parameters, the
 *  application can simply call MessageQ with NULL parameters.
 *  The default parameters would get automatically used.
 */
int messageq_setup(const struct messageq_config *cfg)
{
	int status = MESSAGEQ_SUCCESS;
	struct nameserver_params params;
	struct messageq_config tmpcfg;

	/* This sets the ref_count variable is not initialized, upper 16 bits is
	* written with module Id to ensure correctness of refCount variable.
	*/
	atomic_cmpmask_and_set(&messageq_state.ref_count,
				MESSAGEQ_MAKE_MAGICSTAMP(0),
				MESSAGEQ_MAKE_MAGICSTAMP(0));
	if (atomic_inc_return(&messageq_state.ref_count)
				!= MESSAGEQ_MAKE_MAGICSTAMP(1)) {
		return 1;
	}

	if (cfg == NULL) {
		messageq_get_config(&tmpcfg);
		cfg = &tmpcfg;
	}

	if (WARN_ON(cfg->max_name_len == 0)) {
		status = -EINVAL;
		goto exit;
	}
	if (WARN_ON(cfg->max_name_len == 0)) {
		status = -EINVAL;
		goto exit;
	}

	if (cfg->name_table_gate != NULL) {
		messageq_state.gate_handle = cfg->name_table_gate;
	} else {
		/* User has not provided any gate handle, so create a default
		* handle for protecting list object */
		messageq_state.gate_handle = kmalloc(sizeof(struct mutex),
					GFP_KERNEL);
		if (messageq_state.gate_handle == NULL) {
			/*! @retval MESSAGEQ_E_FAIL Failed to create lock! */
			status = MESSAGEQ_E_FAIL;
			printk(KERN_ERR "messageq_setup: Failed to create a "
				"mutex.\n");
			status = -ENOMEM;
			goto exit;
		}
		mutex_init(messageq_state.gate_handle);
	}

	/* Initialize the parameters */
	nameserver_params_init(&params);
	params.max_value_len = 4;
	params.max_name_len = cfg->max_name_len;

	/* Create the nameserver for modules */
	messageq_state.ns_handle = nameserver_create(MESSAGEQ_NAMESERVER,
								&params);
	if (messageq_state.ns_handle == NULL) {
		/*! @retval MESSAGEQ_E_FAIL Failed to create the
		 * MessageQ nameserver*/
		status = MESSAGEQ_E_FAIL;
		printk(KERN_ERR "messageq_setup: Failed to create the messageq"
			"nameserver!\n");
		goto nameserver_create_fail;
	}

	messageq_state.num_heaps = cfg->num_heaps;
	messageq_state.heaps = kzalloc(sizeof(void *) * \
				messageq_state.num_heaps, GFP_KERNEL);
	if (messageq_state.heaps == NULL) {
		status = -ENOMEM;
		goto heaps_alloc_fail;
	}

	messageq_state.num_queues = cfg->max_runtime_entries;
	messageq_state.queues = kzalloc(sizeof(struct messageq_object *) * \
					messageq_state.num_queues, GFP_KERNEL);
	if (messageq_state.queues == NULL) {
		status = -ENOMEM;
		goto queues_alloc_fail;
	}

	memset(&(messageq_state.transports), 0, (sizeof(void *) * \
				MULTIPROC_MAXPROCESSORS * \
				MESSAGEQ_NUM_PRIORITY_QUEUES));

	BUG_ON(status < 0);
	return status;

queues_alloc_fail:
	if (messageq_state.queues != NULL)
		kfree(messageq_state.queues);
heaps_alloc_fail:
	if (messageq_state.heaps != NULL)
		kfree(messageq_state.heaps);
	if (messageq_state.ns_handle != NULL)
		nameserver_delete(&messageq_state.ns_handle);
nameserver_create_fail:
	if (cfg->name_table_gate != NULL) {
		if (messageq_state.gate_handle != NULL) {
			kfree(messageq_state.gate_handle);
			messageq_state.gate_handle = NULL;
		}
	}

	memset(&messageq_state.cfg, 0, sizeof(struct messageq_config));
	messageq_state.queues = NULL;
	messageq_state.heaps = NULL;
	messageq_state.num_queues = 0;
	messageq_state.num_heaps = 1;
	messageq_state.can_free_queues = true;

exit:
	if (status < 0) {
		atomic_set(&messageq_state.ref_count,
					MESSAGEQ_MAKE_MAGICSTAMP(0));
		printk(KERN_ERR "messageq_setup failed! status = 0x%x\n",
			status);
	}
	return status;
}
EXPORT_SYMBOL(messageq_setup);

/*
 * ======== messageq_destroy ========
 *  Purpose:
 *  Function to destroy the MessageQ module.
 */
int messageq_destroy(void)
{
	int status = MESSAGEQ_SUCCESS;
	u32 i;

	if (atomic_cmpmask_and_lt(&(messageq_state.ref_count),
				MESSAGEQ_MAKE_MAGICSTAMP(0),
				MESSAGEQ_MAKE_MAGICSTAMP(1)) == true) {
		status = -ENODEV;
		goto exit;
	}

	if (!(atomic_dec_return(&messageq_state.ref_count)
					== MESSAGEQ_MAKE_MAGICSTAMP(0))) {
		status = 1;
		goto exit;
	}

	if (WARN_ON(messageq_state.ns_handle == NULL)) {
		status = -ENODEV;
		goto exit;
	}

	/* Delete any Message Queues that have not been deleted so far. */
	for (i = 0; i < messageq_state.num_queues; i++) {
		if (messageq_state.queues[i] != NULL)
			messageq_delete(&(messageq_state.queues[i]));
	}

	/* Delete the nameserver for modules */
	status = nameserver_delete(&messageq_state.ns_handle);
	BUG_ON(status < 0);

	/* Delete the gate if created internally */
	if (messageq_state.cfg.name_table_gate == NULL) {
		kfree(messageq_state.gate_handle);
		messageq_state.gate_handle = NULL;
		BUG_ON(status < 0);
	}

	memset(&(messageq_state.transports), 0, (sizeof(void *) * \
		MULTIPROC_MAXPROCESSORS * MESSAGEQ_NUM_PRIORITY_QUEUES));
	if (messageq_state.heaps != NULL) {
		kfree(messageq_state.heaps);
		messageq_state.heaps = NULL;
	}
	if (messageq_state.queues != NULL) {
		kfree(messageq_state.queues);
		messageq_state.queues = NULL;
	}

	memset(&messageq_state.cfg, 0, sizeof(struct messageq_config));
	messageq_state.num_queues = 0;
	messageq_state.num_heaps = 1;
	messageq_state.can_free_queues = true;
	atomic_set(&messageq_state.ref_count,
				MESSAGEQ_MAKE_MAGICSTAMP(0));

exit:
	if (status < 0) {
		printk(KERN_ERR "messageq_destroy failed! status = 0x%x\n",
			status);
	}
	return status;
}
EXPORT_SYMBOL(messageq_destroy);

/*
 * ======== messageq_params_init ========
 *  Purpose:
 *  Initialize this config-params structure with supplier-specified
 *  defaults before instance creation.
 */
void messageq_params_init(void *messageq_handle,
			struct messageq_params *params)
{
	struct messageq_object *object = NULL;

	if (atomic_cmpmask_and_lt(&(messageq_state.ref_count),
				MESSAGEQ_MAKE_MAGICSTAMP(0),
				MESSAGEQ_MAKE_MAGICSTAMP(1)) == true)
		goto exit;

	if (WARN_ON(params == NULL)) {
		printk(KERN_ERR "messageq_params_init failed:Argument of "
			"type(messageq_params *) is NULL!\n");
		goto exit;
	}

	if (messageq_handle == NULL) {
		memcpy(params, &(messageq_state.default_inst_params),
				sizeof(struct messageq_params));
	} else {
		object = (struct messageq_object *) messageq_handle;
		memcpy((void *)params, (void *)&object->params,
				sizeof(struct messageq_params));
	}

exit:
	return;
}
EXPORT_SYMBOL(messageq_params_init);

/*
 * ======== messageq_create ========
 *  Purpose:
 *  Creates a new instance of MessageQ module.
 */
void *messageq_create(char *name, const struct messageq_params *params)
{
	int status = 0;
	struct messageq_object *handle = NULL;
	bool found = false;
	u16 count = 0;
	int  i;
	u16 start;
	int key;
	u16 queueIndex = 0;

	if (atomic_cmpmask_and_lt(&(messageq_state.ref_count),
				MESSAGEQ_MAKE_MAGICSTAMP(0),
				MESSAGEQ_MAKE_MAGICSTAMP(1)) == true) {
		status = -ENODEV;
		goto exit;
	}
	if (WARN_ON(params == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	/* Create the generic handle */
	handle = kzalloc(sizeof(struct messageq_object), 0);
	if (handle == NULL) {
		status = -ENOMEM;
		goto exit;
	}

	key = mutex_lock_interruptible(messageq_state.gate_handle);
	start = 0; /* Statically allocated objects not supported */
	count = messageq_state.num_queues;
	/* Search the dynamic array for any holes */
	for (i = start; i < count ; i++) {
		if (messageq_state.queues[i] == NULL) {
			messageq_state.queues[i] = (void *) handle;
			queueIndex = i;
			found = true;
			break;
		}
	}
	/*
	 *  If no free slot was found:
	 *  - if no growth allowed, raise an error
	 *  - if growth is allowed, grow the array
	 */
	if (found == false) {
		if (messageq_state.cfg.max_runtime_entries
			!= MESSAGEQ_ALLOWGROWTH) {
			mutex_unlock(messageq_state.gate_handle);
			status = MESSAGEQ_E_MAXREACHED;
			printk(KERN_ERR "messageq_create: All message queues "
				"are full!\n");
			goto free_slot_fail;
		} else {
			queueIndex = _messageq_grow(handle);
			if (queueIndex == MESSAGEQ_INVALIDMESSAGEQ) {
				mutex_unlock(messageq_state.gate_handle);
				status = MESSAGEQ_E_MAXREACHED;
				printk(KERN_ERR "messageq_create: All message "
					"queues are full!\n");
				goto free_slot_fail;
			}
		}
	}

	BUG_ON(status < 0);
	mutex_unlock(messageq_state.gate_handle);

	/* Construct the list object */
	INIT_LIST_HEAD(&handle->normal_list);
	INIT_LIST_HEAD(&handle->high_list);

	/* Copy the name */
	if (name != NULL) {
		handle->name = kmalloc((strlen(name) + 1), GFP_KERNEL);
		if (handle->name == NULL) {
			status = -ENOMEM;
			goto handle_name_alloc_fail;
		}
		strncpy(handle->name, name, strlen(name) + 1);
	}

	/* Update processor information */
	handle->queue = ((u32)(multiproc_get_id(NULL)) << 16) | queueIndex;
	/*handle->synchronizer = OsalSemaphore_create(OsalSemaphore_Type_Binary
				| OsalSemaphore_IntType_Interruptible);*/
	handle->synchronizer = kzalloc(sizeof(struct semaphore), GFP_KERNEL);
	if (handle->synchronizer == NULL) {
		status = MESSAGEQ_E_FAIL;
		printk(KERN_ERR "messageq_create: Failed to create "
			"synchronizer semaphore!\n");
		goto semaphore_create_fail;
	} else {
		sema_init(handle->synchronizer, 0);
	}

	if (name != NULL) {
		nameserver_add_uint32(messageq_state.ns_handle, name,
						handle->queue);
	}
	goto exit;

semaphore_create_fail:
	if (handle->name != NULL)
		kfree(handle->name);

handle_name_alloc_fail:
	list_del(&handle->high_list);
	list_del(&handle->normal_list);

free_slot_fail:
	/* Now free the handle */
	if (handle != NULL) {
		kfree(handle);
		handle = NULL;
	}

exit:
	if (status < 0) {
		printk(KERN_ERR "messageq_create failed! status = 0x%x\n",
			status);
	}
	return (void *) handle;
}
EXPORT_SYMBOL(messageq_create);

/*
 * ======== messageq_delete ========
 *  Purpose:
 *  Deletes a instance of MessageQ module.
 */
int messageq_delete(void **msg_handleptr)
{
	int status = 0;
	struct messageq_object *handle = NULL;
	int key;

	if (atomic_cmpmask_and_lt(&(messageq_state.ref_count),
				MESSAGEQ_MAKE_MAGICSTAMP(0),
				MESSAGEQ_MAKE_MAGICSTAMP(1)) == true) {
		status = -ENODEV;
		goto exit;
	}
	if (WARN_ON(msg_handleptr == NULL)) {
		status = -EINVAL;
		goto exit;
	}
	if (WARN_ON(*msg_handleptr == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	handle = (struct messageq_object *) (*msg_handleptr);

	/* Take the local lock */
	key = mutex_lock_interruptible(messageq_state.gate_handle);

	if (handle->name != NULL) {
		/* remove from the name serve */
		nameserver_remove(messageq_state.ns_handle, handle->name);
		/* Free memory for the name */
		kfree(handle->name);
	}

	/* Release the local lock */
	mutex_unlock(messageq_state.gate_handle);

	/* Free the list */
	list_del(&handle->high_list);
	list_del(&handle->normal_list);

	/*if (handle->synchronizer != NULL)
		status = OsalSemaphore_delete(&handle->synchronizer);*/
	if (handle->synchronizer != NULL) {
		kfree(handle->synchronizer);
		handle->synchronizer = NULL;
	}
	/* Clear the MessageQ handle from array. */
	messageq_state.queues[handle->queue] = NULL;

	/* Now free the handle */
	kfree(handle);
	*msg_handleptr = NULL;

exit:
	if (status < 0) {
		printk(KERN_ERR "messageq_delete failed! status = 0x%x\n",
			status);;
	}
	return status;
}
EXPORT_SYMBOL(messageq_delete);

/*
 * ======== messageq_open ========
 *  Purpose:
 *  Opens a created instance of MessageQ module.
 */
int messageq_open(char *name, u32 *queue_id)
{
	int status = 0;
	int len = 0;

	if (atomic_cmpmask_and_lt(&(messageq_state.ref_count),
				MESSAGEQ_MAKE_MAGICSTAMP(0),
				MESSAGEQ_MAKE_MAGICSTAMP(1)) == true) {
		status = -ENODEV;
		goto exit;
	}
	if (WARN_ON(queue_id == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	/* Initialize return queue ID to invalid. */
	*queue_id = MESSAGEQ_INVALIDMESSAGEQ;
	len = nameserver_get(messageq_state.ns_handle, name, queue_id,
					sizeof(u32), NULL);
	if (len < 0) {
		if (len == -ENOENT) {
			/* Name not found */
			status = -ENOENT;
		} else {
			/* Any other error from nameserver */
			status = len;
		}
	}

exit:
	if (status < 0) {
		printk(KERN_ERR "messageq_open failed! status = 0x%x\n",
			status);
	}
	return status;
}
EXPORT_SYMBOL(messageq_open);

/*
 * ======== messageq_close ========
 *  Purpose:
 *  Closes previously opened/created instance of MessageQ module.
 */
void messageq_close(u32 *queue_id)
{
	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
				MESSAGEQ_MAKE_MAGICSTAMP(0),
				MESSAGEQ_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	if (WARN_ON(queue_id == NULL)) {
		printk(KERN_ERR "messageq_close: queue_id passed is NULL!\n");
		goto exit;
	}

	*queue_id = MESSAGEQ_INVALIDMESSAGEQ;

exit:
	return;
}
EXPORT_SYMBOL(messageq_close);

/*
 *  ======== messageq_get ========
 */
int messageq_get(void *messageq_handle, messageq_msg *msg,
							u32 timeout)
{
	int status = 0;
	struct messageq_object *obj = (struct messageq_object *)messageq_handle;
	int key;

	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
					MESSAGEQ_MAKE_MAGICSTAMP(0),
					MESSAGEQ_MAKE_MAGICSTAMP(1)) == true)) {
		status = -ENODEV;
		goto exit;
	}

	if (WARN_ON(msg == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	if (WARN_ON(obj == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	/* Keep looping while there is no element in the list */
	/* Take the local lock */
	key = mutex_lock_interruptible(messageq_state.gate_handle);
	if (!list_empty(&obj->high_list)) {
		*msg = (messageq_msg) (obj->high_list.next);
		list_del_init(obj->high_list.next);
	}
	/* Leave the local lock */
	mutex_unlock(messageq_state.gate_handle);
	while (*msg == NULL) {
		key = mutex_lock_interruptible(messageq_state.gate_handle);
		if (!list_empty(&obj->normal_list)) {
			*msg = (messageq_msg) (obj->normal_list.next);
			list_del_init(obj->normal_list.next);
		}
		mutex_unlock(messageq_state.gate_handle);

		if (*msg == NULL) {
			/*
			 *  Block until notified.  If pend times-out, no message
			 *  should be returned to the caller
			 */
			/*! @retval NULL timeout has occurred */
			if (obj->synchronizer != NULL) {
				/* TODO: cater to different timeout values */
				/*status = OsalSemaphore_pend(
					obj->synchronizer, timeout); */
				status = down_timeout(obj->synchronizer,
						msecs_to_jiffies(timeout));
				if (status < 0) {
					*msg = NULL;
					break;
				}
			}
			key = mutex_lock_interruptible(
					messageq_state.gate_handle);
			if (!list_empty(&obj->high_list)) {
				*msg = (messageq_msg) (obj->high_list.next);
				list_del_init(obj->high_list.next);
			}
			mutex_unlock(messageq_state.gate_handle);
		}
	}

exit:
	if (status < 0)
		printk(KERN_ERR "messageq_get failed! status = 0x%x\n", status);
	return status;
}
EXPORT_SYMBOL(messageq_get);

/*
 * ======== messageq_count ========
 *  Purpose:
 *  Count the number of messages in the queue
 */
int messageq_count(void *messageq_handle)
{
	struct messageq_object *obj = (struct messageq_object *)messageq_handle;
	int count = 0;
	struct list_head *elem;
	int key;

	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
					MESSAGEQ_MAKE_MAGICSTAMP(0),
					MESSAGEQ_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	if (WARN_ON(obj == NULL)) {
		printk(KERN_ERR "messageq_count: obj passed is NULL!\n");
		goto exit;
	}

	key = mutex_lock_interruptible(messageq_state.gate_handle) ;
	list_for_each(elem, &obj->high_list) {
		count++;
	}
	list_for_each(elem, &obj->normal_list) {
		count++;
	}
	mutex_unlock(messageq_state.gate_handle);

exit:
	return count;
}
EXPORT_SYMBOL(messageq_count);

/*
 * ======== messageq_static_msg_init ========
 *  Purpose:
 *  Initialize a static message
 */
void messageq_static_msg_init(messageq_msg msg, u32 size)
{
	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
					MESSAGEQ_MAKE_MAGICSTAMP(0),
					MESSAGEQ_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	if (WARN_ON(msg == NULL)) {
		printk(KERN_ERR "messageq_static_msg_init: msg is invalid!\n");
		goto exit;
	}

	/* Fill in the fields of the message */
	msg->heap_id = MESSAGEQ_STATICMSG;
	msg->msg_size = size;
	msg->reply_id = (u16)MESSAGEQ_INVALIDMESSAGEQ;
	msg->msg_id = MESSAGEQ_INVALIDMSGID;
	msg->dst_id = (u16)MESSAGEQ_INVALIDMESSAGEQ;
	msg->flags = MESSAGEQ_HEADERVERSION | MESSAGEQ_NORMALPRI;

exit:
	return;
}
EXPORT_SYMBOL(messageq_static_msg_init);

/*
 * ======== messageq_alloc ========
 *  Purpose:
 *  Allocate a message and initial the needed fields (note some
 *  of the fields in the header at set via other APIs or in the
 *  messageq_put function.
 */
messageq_msg messageq_alloc(u16 heap_id, u32 size)
{
	messageq_msg msg = NULL;

	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
					MESSAGEQ_MAKE_MAGICSTAMP(0),
					MESSAGEQ_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	if (WARN_ON(heap_id >= messageq_state.num_heaps)) {
		printk(KERN_ERR "messageq_alloc: heap_id is invalid!\n");
		goto exit;
	}

	if (messageq_state.heaps[heap_id] != NULL) {
		/* Allocate the message. No alignment requested */
		msg = heap_alloc(messageq_state.heaps[heap_id], size, 0);
		if (msg == NULL) {
			printk(KERN_ERR "messageq_alloc: message allocation "
				"failed!\n");
			goto exit;
		}

		/* Fill in the fields of the message */
		msg->msg_size = size;
		msg->heap_id = heap_id;
		msg->reply_id = (u16)MESSAGEQ_INVALIDMESSAGEQ;
		msg->dst_id = (u16)MESSAGEQ_INVALIDMESSAGEQ;
		msg->msg_id = MESSAGEQ_INVALIDMSGID;
		msg->flags = MESSAGEQ_HEADERVERSION | MESSAGEQ_NORMALPRI;
	} else {
		printk(KERN_ERR "messageq_alloc: heap_id was not "
			"registered!\n");
	}

exit:
	return msg;
}
EXPORT_SYMBOL(messageq_alloc);

/*
 * ======== messageq_free ========
 *  Purpose:
 *  Frees the message.
 */
int messageq_free(messageq_msg msg)
{
	u32 status = 0;
	void *heap = NULL;

	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
					MESSAGEQ_MAKE_MAGICSTAMP(0),
					MESSAGEQ_MAKE_MAGICSTAMP(1)) == true)) {
		status = -ENODEV;
		goto exit;
	}

	if (WARN_ON(msg == NULL)) {
		status = -EINVAL;
		goto exit;
	}
	if (msg->heap_id >= messageq_state.num_heaps) {
		status = MESSAGEQ_E_INVALIDHEAPID;
		goto exit;
	}
	if (msg->heap_id == MESSAGEQ_STATICMSG) {
		status = MESSAGEQ_E_CANNOTFREESTATICMSG;
		goto exit;
	}

	heap = messageq_state.heaps[msg->heap_id];
	heap_free(heap, msg, msg->msg_size);

exit:
	if (status < 0) {
		printk(KERN_ERR "messageq_free failed! status = 0x%x\n",
			status);
	}
	return status;
}
EXPORT_SYMBOL(messageq_free);

/*
 * ======== messageq_put ========
 *  Purpose:
 *  Put a message in the queue
 */
int messageq_put(u32 queue_id, messageq_msg msg)
{
	int status = 0;
	u16 dst_proc_id = (u16)(queue_id >> 16);
	struct messageq_object *obj = NULL;
	void *transport = NULL;
	u32 priority;
	int key;

	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
					MESSAGEQ_MAKE_MAGICSTAMP(0),
					MESSAGEQ_MAKE_MAGICSTAMP(1)) == true)) {
		status = -ENODEV;
		goto exit;
	}
	if (WARN_ON(msg == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	msg->dst_id = (u16)(queue_id);
	msg->dst_proc = (u16)(queue_id >> 16);
	if (dst_proc_id != multiproc_get_id(NULL)) {
		if (dst_proc_id >= multiproc_get_max_processors()) {
			/* Invalid destination processor id */
			status = MESSAGEQ_E_INVALIDPROCID;
			goto exit;
		}

		priority = (u32)((msg->flags) & MESSAGEQ_TRANSPORTPRIORITYMASK);
		/* Call the transport associated with this message queue */
		transport = messageq_state.transports[dst_proc_id][priority];
		if (transport == NULL) {
			/* Try the other transport */
			priority = !priority;
			transport =
			messageq_state.transports[dst_proc_id][priority];
		}

		if (transport != NULL)
			status = messageq_transportshm_put(transport, msg);
		else {
			status = -ENODEV;
			goto exit;
		}
	} else {
		/* It is a local MessageQ */
		obj = (struct messageq_object *)
				(messageq_state.queues[(u16)(queue_id)]);
		key = mutex_lock_interruptible(messageq_state.gate_handle);
		if ((msg->flags & MESSAGEQ_PRIORITYMASK) == \
			MESSAGEQ_URGENTPRI) {
			list_add((struct list_head *) msg, &obj->high_list);
		} else {
			if ((msg->flags & MESSAGEQ_PRIORITYMASK) == \
				MESSAGEQ_NORMALPRI) {
				list_add_tail((struct list_head *) msg,
					&obj->normal_list);
			} else {
				list_add_tail((struct list_head *) msg,
					&obj->high_list);
			}
		}
		mutex_unlock(messageq_state.gate_handle);

		/* Notify the reader. */
		if (obj->synchronizer != NULL) {
			up(obj->synchronizer);
			/*OsalSemaphore_post(obj->synchronizer);*/
		}
	}

exit:
	if (status < 0)
		printk(KERN_ERR "messageq_put failed! status = 0x%x\n", status);
	return status;
}
EXPORT_SYMBOL(messageq_put);

/*
 * ======== messageq_register_heap ========
 *  Purpose:
 *  register a heap
 */
int messageq_register_heap(void *heap_handle, u16 heap_id)
{
	int  status = 0;
	int key;

	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
				MESSAGEQ_MAKE_MAGICSTAMP(0),
				MESSAGEQ_MAKE_MAGICSTAMP(1)) == true)) {
		status = -ENODEV;
		goto exit;
	}
	/* Make sure the heap_id is valid */
	if (WARN_ON(heap_id >= messageq_state.num_heaps)) {
		/*! @retval MESSAGEQ_E_HEAPIDINVALID Invalid heap_id */
		status = MESSAGEQ_E_HEAPIDINVALID;
		goto exit;
	}

	key = mutex_lock_interruptible(messageq_state.gate_handle);
	if (messageq_state.heaps[heap_id] == NULL)
		messageq_state.heaps[heap_id] = heap_handle;
	else {
		/*! @retval MESSAGEQ_E_ALREADYEXISTS Specified heap is
		already registered. */
		status = MESSAGEQ_E_ALREADYEXISTS;
	}
	mutex_unlock(messageq_state.gate_handle);

exit:
	if (status < 0) {
		printk(KERN_ERR "messageq_register_heap failed! "
			"status = 0x%x\n", status);
	}
	return status;
}
EXPORT_SYMBOL(messageq_register_heap);

/*
 * ======== messageq_unregister_heap ========
 *  Purpose:
 *  Unregister a heap
 */
int messageq_unregister_heap(u16 heap_id)
{
	int  status = 0;
	int key;

	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
				MESSAGEQ_MAKE_MAGICSTAMP(0),
				MESSAGEQ_MAKE_MAGICSTAMP(1)) == true)) {
		status = -ENODEV;
		goto exit;
	}

	/* Make sure the heap_id is valid */
	if (WARN_ON(heap_id > messageq_state.num_heaps)) {
		/*! @retval MESSAGEQ_E_HEAPIDINVALID Invalid heap_id */
		status = MESSAGEQ_E_HEAPIDINVALID;
		goto exit;
	}

	key = mutex_lock_interruptible(messageq_state.gate_handle);
	if (messageq_state.heaps != NULL)
		messageq_state.heaps[heap_id] = NULL;
	mutex_unlock(messageq_state.gate_handle);

exit:
	if (status < 0) {
		printk(KERN_ERR "messageq_unregister_heap failed! "
			"status = 0x%x\n", status);
	}
	return status;
}
EXPORT_SYMBOL(messageq_unregister_heap);

/*
 * ======== messageq_register_transport ========
 *  Purpose:
 *  register a transport
 */
int messageq_register_transport(void *messageq_transportshm_handle,
				 u16 proc_id, u32 priority)
{
	int  status = 0;
	int key;

	BUG_ON(messageq_transportshm_handle == NULL);
	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
				MESSAGEQ_MAKE_MAGICSTAMP(0),
				MESSAGEQ_MAKE_MAGICSTAMP(1)) == true)) {
		status = -ENODEV;
		goto exit;
	}

	/* Make sure the proc_id is valid */
	if (WARN_ON(proc_id >= multiproc_get_max_processors())) {
		/*! @retval MESSAGEQ_E_PROCIDINVALID Invalid proc_id */
		status = MESSAGEQ_E_PROCIDINVALID;
		goto exit;
	}

	key = mutex_lock_interruptible(messageq_state.gate_handle);
	if (messageq_state.transports[proc_id][priority] == NULL) {
		messageq_state.transports[proc_id][priority] = \
			messageq_transportshm_handle;
	} else {
		/*! @retval MESSAGEQ_E_ALREADYEXISTS Specified transport is
		already registered. */
		status = MESSAGEQ_E_ALREADYEXISTS;
	}
	mutex_unlock(messageq_state.gate_handle);

exit:
	if (status < 0) {
		printk(KERN_ERR "messageq_register_transport failed! "
			"status = 0x%x\n", status);
	}
	return status;
}
EXPORT_SYMBOL(messageq_register_transport);

/*
 * ======== messageq_unregister_transport ========
 *  Purpose:
 *  Unregister a transport
 */
int messageq_unregister_transport(u16 proc_id, u32 priority)
{
	int  status = 0;
	int key;

	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
					MESSAGEQ_MAKE_MAGICSTAMP(0),
					MESSAGEQ_MAKE_MAGICSTAMP(1)) == true)) {
		status = -ENODEV;
		goto exit;
	}

	/* Make sure the proc_id is valid */
	if (WARN_ON(proc_id >= multiproc_get_max_processors())) {
		/*! @retval MESSAGEQ_E_PROCIDINVALID Invalid proc_id */
		status = MESSAGEQ_E_PROCIDINVALID;
		goto exit;
	}

	key = mutex_lock_interruptible(messageq_state.gate_handle);
	if (messageq_state.transports[proc_id][priority] == NULL)
		messageq_state.transports[proc_id][priority] = NULL;
	mutex_unlock(messageq_state.gate_handle);

exit:
	if (status < 0) {
		printk(KERN_ERR "messageq_unregister_transport failed! "
			"status = 0x%x\n", status);
	}
	return status;
}
EXPORT_SYMBOL(messageq_unregister_transport);

/*
 * ======== messageq_set_reply_queue ========
 *  Purpose:
 *  Set the destination queue of the message.
 */
void messageq_set_reply_queue(void *messageq_handle, messageq_msg msg)
{
	struct messageq_object *obj = \
			(struct messageq_object *) messageq_handle;

	BUG_ON(messageq_handle == NULL);
	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
					MESSAGEQ_MAKE_MAGICSTAMP(0),
					MESSAGEQ_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	if (WARN_ON(msg == NULL)) {
		printk(KERN_ERR "messageq_set_reply_queue: msg passed is "
			"NULL!\n");
		goto exit;
	}

	msg->reply_id = (u16)(obj->queue);
	msg->reply_proc = (u16)(obj->queue >> 16);

exit:
	return;
}
EXPORT_SYMBOL(messageq_set_reply_queue);

/*
 * ======== messageq_get_queue_id ========
 *  Purpose:
 *  Get the queue _id of the message.
 */
u32 messageq_get_queue_id(void *messageq_handle)
{
	struct messageq_object *obj = \
			(struct messageq_object *) messageq_handle;
	u32 queue_id = MESSAGEQ_INVALIDMESSAGEQ;

	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
					MESSAGEQ_MAKE_MAGICSTAMP(0),
					MESSAGEQ_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	if (WARN_ON(obj == NULL)) {
		printk(KERN_ERR "messageq_get_queue_id: obj passed is NULL!\n");
		goto exit;
	}

	queue_id = (obj->queue);

exit:
	return queue_id;
}
EXPORT_SYMBOL(messageq_get_queue_id);

/*
 * ======== messageq_get_proc_id ========
 *  Purpose:
 *  Get the proc _id of the message.
 */
u16 messageq_get_proc_id(void *messageq_handle)
{
	struct messageq_object *obj = \
			(struct messageq_object *) messageq_handle;
	u16 proc_id = MULTIPROC_INVALIDID;

	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
				MESSAGEQ_MAKE_MAGICSTAMP(0),
				MESSAGEQ_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	if (WARN_ON(obj == NULL)) {
		printk(KERN_ERR "messageq_get_proc_id: obj passed is NULL!\n");
		goto exit;
	}

	proc_id = (u16)(obj->queue >> 16);

exit:
	return proc_id;
}
EXPORT_SYMBOL(messageq_get_proc_id);

/*
 * ======== messageq_get_dst_queue ========
 *  Purpose:
 *  Get the destination queue of the message.
 */
u32 messageq_get_dst_queue(messageq_msg msg)
{
	u32 queue_id = MESSAGEQ_INVALIDMESSAGEQ;

	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
				MESSAGEQ_MAKE_MAGICSTAMP(0),
				MESSAGEQ_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	if (WARN_ON(msg == NULL)) {
		printk(KERN_ERR "messageq_get_dst_queue: msg passed is "
			"NULL!\n");
		goto exit;
	}

	/*construct queue value */
	if (msg->dst_id != (u32)MESSAGEQ_INVALIDMESSAGEQ)
		queue_id = ((u32) multiproc_get_id(NULL) << 16) | msg->dst_id;

exit:
	return queue_id;
}
EXPORT_SYMBOL(messageq_get_dst_queue);

/*
 * ======== messageq_get_msg_id ========
 *  Purpose:
 *  Get the message id of the message.
 */
u16 messageq_get_msg_id(messageq_msg msg)
{
	u16 id = MESSAGEQ_INVALIDMSGID;

	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
				MESSAGEQ_MAKE_MAGICSTAMP(0),
				MESSAGEQ_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	if (WARN_ON(msg == NULL)) {
		printk(KERN_ERR "messageq_get_msg_id: msg passed is NULL!\n");
		goto exit;
	}

	id = msg->msg_id;

exit:
	return id;
}
EXPORT_SYMBOL(messageq_get_msg_id);

/*
 * ======== messageq_get_msg_size ========
 *  Purpose:
 *  Get the message size of the message.
 */
u32 messageq_get_msg_size(messageq_msg msg)
{
	u32 size = 0;

	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
				MESSAGEQ_MAKE_MAGICSTAMP(0),
				MESSAGEQ_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	if (WARN_ON(msg == NULL)) {
		printk(KERN_ERR "messageq_get_msg_size: msg passed is NULL!\n");
		goto exit;
	}

	size = msg->msg_size;

exit:
	return size;
}
EXPORT_SYMBOL(messageq_get_msg_size);

/*
 * ======== messageq_get_msg_pri ========
 *  Purpose:
 *  Get the message priority of the message.
 */
u32 messageq_get_msg_pri(messageq_msg msg)
{
	u32 priority = MESSAGEQ_NORMALPRI;

	BUG_ON(msg == NULL);
	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
				MESSAGEQ_MAKE_MAGICSTAMP(0),
				MESSAGEQ_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	if (WARN_ON(msg == NULL)) {
		printk(KERN_ERR "messageq_get_msg_pri: msg passed is NULL!\n");
		goto exit;
	}

	priority = ((u32)(msg->flags & MESSAGEQ_PRIORITYMASK));

exit:
	return priority;
}
EXPORT_SYMBOL(messageq_get_msg_pri);

/*
 * ======== messageq_get_reply_queue ========
 *  Purpose:
 *  Get the embedded source message queue out of the message.
 */
u32 messageq_get_reply_queue(messageq_msg msg)
{
	u32 queue = MESSAGEQ_INVALIDMESSAGEQ;

	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
				MESSAGEQ_MAKE_MAGICSTAMP(0),
				MESSAGEQ_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	if (WARN_ON(msg == NULL)) {
		printk(KERN_ERR "messageq_get_reply_queue: msg passed is "
			"NULL!\n");
		goto exit;
	}

	if (msg->reply_id != (u16)MESSAGEQ_INVALIDMESSAGEQ)
		queue = ((u32)(msg->reply_proc) << 16) | msg->reply_id;

exit:
	return queue;
}
EXPORT_SYMBOL(messageq_get_reply_queue);

/*
 * ======== messageq_set_msg_id ========
 *  Purpose:
 *  Set the message id of the message.
 */
void messageq_set_msg_id(messageq_msg msg, u16 msg_id)
{
	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
				MESSAGEQ_MAKE_MAGICSTAMP(0),
				MESSAGEQ_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	if (WARN_ON(msg == NULL)) {
		printk(KERN_ERR "messageq_set_msg_id: msg passed is NULL!\n");
		goto exit;
	}

	msg->msg_id = msg_id;

exit:
	return;
}
EXPORT_SYMBOL(messageq_set_msg_id);

/*
 * ======== messageq_set_msg_pri ========
 *  Purpose:
 *  Set the priority of the message.
 */
void messageq_set_msg_pri(messageq_msg msg, u32 priority)
{
	if (WARN_ON(atomic_cmpmask_and_lt(&(messageq_state.ref_count),
				MESSAGEQ_MAKE_MAGICSTAMP(0),
				MESSAGEQ_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	if (WARN_ON(msg == NULL)) {
		printk(KERN_ERR "messageq_set_msg_pri: msg passed is NULL!\n");
		goto exit;
	}

	msg->flags = priority & MESSAGEQ_PRIORITYMASK;

exit:
	return;
}
EXPORT_SYMBOL(messageq_set_msg_pri);

/* =============================================================================
 * Internal functions
 * =============================================================================
 */
/*
 * ======== _messageq_grow ========
 *  Purpose:
 *  Grow the MessageQ table
 */
u16 _messageq_grow(struct messageq_object *obj)
{
	u16 queue_index = messageq_state.num_queues;
	int oldSize;
	void **queues;
	void **oldqueues;

	if (WARN_ON(obj == NULL)) {
		printk(KERN_ERR "_messageq_grow: obj passed is NULL!\n");
		goto exit;
	}

	oldSize = (messageq_state.num_queues) * \
			sizeof(struct messageq_object *);
	queues = kmalloc(oldSize + sizeof(struct messageq_object *),
				GFP_KERNEL);
	if (queues == NULL) {
		printk(KERN_ERR "_messageq_grow: Growing the messageq "
			"failed!\n");
		goto exit;
	}

	/* Copy contents into new table */
	memcpy(queues, messageq_state.queues, oldSize);
	/* Fill in the new entry */
	queues[queue_index] = (void *)obj;
	/* Hook-up new table */
	oldqueues = messageq_state.queues;
	messageq_state.queues = queues;
	messageq_state.num_queues++;

	/* Delete old table if not statically defined*/
	if (messageq_state.can_free_queues == true)
		kfree(oldqueues);
	else
		messageq_state.can_free_queues = true;

exit:
	return queue_index;
}
