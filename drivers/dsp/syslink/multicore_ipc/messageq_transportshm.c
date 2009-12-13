/*
 *  messageq_transportshm.c
 *
 *  MessageQ Transport module
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

/* Standard headers */
#include <linux/types.h>

/* Utilities headers */
#include <linux/string.h>
#include <linux/slab.h>

/* Syslink headers */
#include <syslink/atomic_linux.h>
/* Module level headers */
#include <multiproc.h>
#include <nameserver.h>
#include <gatepeterson.h>
#include <notify.h>
#include <messageq.h>
#include <listmp_sharedmemory.h>
#include <messageq_transportshm.h>

/* =============================================================================
 * Globals
 * =============================================================================
 */
/* Cache line size */
#define MESSAGEQ_TRANSPORTSHM_CACHESIZE	128

/* Indicates that the transport is up. */
#define MESSAGEQ_TRANSPORTSHM_UP	0xBADC0FFE

/* messageq_transportshm Version. */
#define MESSAGEQ_TRANSPORTSHM_VERSION	1

/*!
 *  @brief  Macro to make a correct module magic number with refCount
 */
#define MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(x)                                \
				((MESSAGEQ_TRANSPORTSHM_MODULEID << 12u) | (x))

/* =============================================================================
 * Structures & Enums
 * =============================================================================
 */
/*
 *  Defines the messageq_transportshm state object, which contains all the
 *           module specific information.
 */
struct messageq_transportshm_moduleobject {
	atomic_t ref_count;
	struct messageq_transportshm_config cfg;
	/*< messageq_transportshm configuration structure */
	struct messageq_transportshm_config def_cfg;
	/*< Default module configuration */
	struct messageq_transportshm_params def_inst_params;
	/*< Default instance parameters */
	void *gate_handle;
	/*< Handle to the gate for local thread safety */
	void *transports[MULTIPROC_MAXPROCESSORS][MESSAGEQ_NUM_PRIORITY_QUEUES];
	/*!< Transport to be set in messageq_register_transport */

};

/*
 * Structure of attributes in shared memory
 */
struct messageq_transportshm_attrs {
	VOLATILE u32 version;
	VOLATILE u32 flag;
};

/*
 * Structure defining config parameters for the MessageQ transport
 *  instances.
 */
struct messageq_transportshm_object {
	VOLATILE struct messageq_transportshm_attrs *attrs[2];
	/* Attributes for both processors */
	void *my_listmp_handle;
	/* List for this processor	*/
	void *remote_listmp_handle;
	/* List for remote processor	*/
	VOLATILE int status;
	/* Current status		 */
	int my_index;
	/* 0 | 1			  */
	int remote_index;
	/* 1 | 0			 */
	int notify_event_no;
	/* Notify event to be used	*/
	void *notify_driver;
	/* Notify driver to be used	*/
	u16 proc_id;
	/* Dest proc id		  */
	void *gate;
	/* Gate for critical regions	*/
	struct messageq_transportshm_params params;
	/* Instance specific parameters  */
	u32 priority;
	/*!<  Priority of messages supported by this transport */
};

/* =============================================================================
 *  Globals
 * =============================================================================
 */
/*
 *  @var    messageq_transportshm_state
 *
 * messageq_transportshm state object variable
 */
static struct messageq_transportshm_moduleobject messageq_transportshm_state = {
	.gate_handle = NULL,
	.def_cfg.err_fxn = 0,
	.def_inst_params.gate = NULL,
	.def_inst_params.shared_addr = 0x0,
	.def_inst_params.shared_addr_size = 0x0,
	.def_inst_params.notify_event_no = (u32)(-1),
	.def_inst_params.notify_driver = NULL,
	.def_inst_params.priority = MESSAGEQ_NORMALPRI
};


/* =============================================================================
 * Forward declarations of internal functions
 * =============================================================================
 */
/* Callback function registered with the Notify module. */
static void _messageq_transportshm_notify_fxn(u16 proc_id,
					u32 event_no, void *arg, u32 payload);

/* =============================================================================
 * APIs called directly by applications
 * =============================================================================
 */
/*
 * ======== messageq_transportshm_get_config ========
 *  Purpose:
 *  Get the default configuration for the messageq_transportshm
 *  module.
 *
 *  This function can be called by the application to get their
 *  configuration parameter to messageq_transportshm_setup filled in
 *  by the messageq_transportshm module with the default parameters.
 *  If the user does not wish to make any change in the default
 *  parameters, this API is not required to be called.
 */
void messageq_transportshm_get_config(
				struct messageq_transportshm_config *cfg)
{
	if (WARN_ON(cfg == NULL))
		goto exit;

	if (atomic_cmpmask_and_lt(&(messageq_transportshm_state.ref_count),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(1)) == true) {
		memcpy(cfg, &(messageq_transportshm_state.def_cfg),
			sizeof(struct messageq_transportshm_config));
	} else {
		memcpy(cfg, &(messageq_transportshm_state.cfg),
			sizeof(struct messageq_transportshm_config));
	}
	return;

exit:
	printk(KERN_ERR "messageq_transportshm_get_config: Argument of type"
		"(struct messageq_transportshm_config *) passed is null!\n");
}


/*
 * ======== messageq_transportshm_setup ========
 *  Purpose:
 *  Setup the messageq_transportshm module.
 *
 *  This function sets up the messageq_transportshm module. This
 *  function must be called before any other instance-level APIs can
 *  be invoked.
 *  Module-level configuration needs to be provided to this
 *  function. If the user wishes to change some specific config
 *  parameters, then messageq_transportshm_getConfig can be called
 *  to get the configuration filled with the default values. After
 *  this, only the required configuration values can be changed. If
 *  the user does not wish to make any change in the default
 *  parameters, the application can simply call
 *  messageq_transportshm_setup with NULL parameters. The default
 *  parameters would get automatically used.
 */
int messageq_transportshm_setup(const struct messageq_transportshm_config *cfg)
{
	int status = MESSAGEQ_TRANSPORTSHM_SUCCESS;
	struct messageq_transportshm_config tmpCfg;

	/* This sets the refCount variable is not initialized, upper 16 bits is
	* written with module Id to ensure correctness of refCount variable.
	*/
	atomic_cmpmask_and_set(&messageq_transportshm_state.ref_count,
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0));

	if (atomic_inc_return(&messageq_transportshm_state.ref_count)
		!= MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(1u)) {
		return 1;
	}

	if (cfg == NULL) {
		messageq_transportshm_get_config(&tmpCfg);
		cfg = &tmpCfg;
	}

	messageq_transportshm_state.gate_handle = \
				kmalloc(sizeof(struct mutex), GFP_KERNEL);
	if (messageq_transportshm_state.gate_handle == NULL) {
		/* @retval MESSAGEQTRANSPORTSHM_E_FAIL Failed to create
		GateMutex! */
		status = MESSAGEQ_TRANSPORTSHM_E_FAIL;
		printk(KERN_ERR "messageq_transportshm_setup: Failed to create "
			"mutex!\n");
		atomic_set(&messageq_transportshm_state.ref_count,
				MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0));
		goto exit;
	}
	mutex_init(messageq_transportshm_state.gate_handle);

	/* Copy the user provided values into the state object. */
	memcpy(&messageq_transportshm_state.cfg, cfg,
			sizeof(struct messageq_transportshm_config));
	memset(&(messageq_transportshm_state.transports), 0, (sizeof(void *) * \
		MULTIPROC_MAXPROCESSORS * MESSAGEQ_NUM_PRIORITY_QUEUES));
	return status;

exit:
	printk(KERN_ERR "messageq_transportshm_setup failed: status = 0x%x",
		status);
	return status;
}


/*
 * ======== messageq_transportshm_destroy ========
 *  Purpose:
 *  Destroy the messageq_transportshm module.
 *
 *  Once this function is called, other messageq_transportshm module
 *  APIs, except for the messageq_transportshm_getConfig API cannot
 *  be called anymore.
 */
int messageq_transportshm_destroy(void)
{
	int status = 0;
	u16 i;
	u16 j;

	if (WARN_ON(atomic_cmpmask_and_lt(
			&(messageq_transportshm_state.ref_count),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(1)) == true)) {
		status = -ENODEV;
		goto exit;
	}

	if (!(atomic_dec_return(&messageq_transportshm_state.ref_count)
				== MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0))) {
		status = 1;
		goto exit;
	}

	/* Temporarily increment ref_count here. */
	atomic_set(&messageq_transportshm_state.ref_count,
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(1));

	/* Delete any Transports that have not been deleted so far. */
	for (i = 0; i < MULTIPROC_MAXPROCESSORS; i++) {
		for (j = 0 ; j < MESSAGEQ_NUM_PRIORITY_QUEUES; j++) {
			if (messageq_transportshm_state.transports[i][j] != \
				NULL) {
				messageq_transportshm_delete(&
				(messageq_transportshm_state.transports[i][j]));
			}
		}
	}
	if (messageq_transportshm_state.gate_handle != NULL) {
		kfree(messageq_transportshm_state.gate_handle);
		messageq_transportshm_state.gate_handle = NULL;
	}

	/* Decrease the ref_count */
	atomic_set(&messageq_transportshm_state.ref_count,
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0));
	return 0;

exit:
	if (status < 0) {
		printk(KERN_ERR "messageq_transportshm_destroy failed: "
			"status = 0x%x\n", status);
	}
	return status;
}


/*
 * ======== messageq_transportshm_params_init ========
 *  Purpose:
 *  Get Instance parameters
 */
void messageq_transportshm_params_init(void *mqtshm_handle,
				struct messageq_transportshm_params *params)
{
	struct messageq_transportshm_object *object = NULL;

	if (WARN_ON(atomic_cmpmask_and_lt(
			&(messageq_transportshm_state.ref_count),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(1)) == true)) {
		printk(KERN_ERR "messageq_transportshm_params_init: Module was "
				" not initialized\n");
		goto exit;
	}

	if (WARN_ON(params == NULL)) {
		printk(KERN_ERR "messageq_transportshm_params_init: Argument of"
				" type (struct messageq_transportshm_params *) "
				"is NULL!");
		goto exit;
	}

	if (mqtshm_handle == NULL) {
		memcpy(params, 	&(messageq_transportshm_state.def_inst_params),
				sizeof(struct messageq_transportshm_params));
	} else {
		/* Return updated messageq_transportshm instance
		specific parameters. */
		object = (struct messageq_transportshm_object *) mqtshm_handle;
		memcpy(params, &(object->params),
			sizeof(struct messageq_transportshm_params));
	}

exit:
	return;
}

/*
 * ======== messageq_transportshm_create ========
 *  Purpose:
 *  Create a transport instance. This function waits for the remote
 *  processor to complete its transport creation. Hence it must be
 *  called only after the remote processor is running.
 */
void *messageq_transportshm_create(u16 proc_id,
			const struct messageq_transportshm_params *params)
{
	struct messageq_transportshm_object *handle = NULL;
	int status = 0;
	int my_index;
	int remote_index;
	listmp_sharedmemory_params listmp_params[2];
	VOLATILE u32 *otherflag;

	if (WARN_ON(atomic_cmpmask_and_lt(
			&(messageq_transportshm_state.ref_count),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(1)) == true)) {
		status = -ENODEV;
		goto exit;
	}

	if (WARN_ON(params == NULL)) {
		status = -EINVAL;
		goto exit;
	}
	if (WARN_ON(params->shared_addr_size < \
		messageq_transportshm_shared_mem_req(params))) {
		status = -EINVAL;
		goto exit;
	}
	if (messageq_transportshm_state.transports[proc_id][params->priority] \
		!= NULL) {
		/* Specified transport is already registered. */
		status = MESSAGEQ_E_ALREADYEXISTS;
		goto exit;
	}

	/*
	 *  Determine who gets the '0' slot and who gets the '1' slot
	 *  The '0' slot is given to the lower multiproc id.
	 */
	if (multiproc_get_id(NULL) < proc_id) {
		my_index = 0;
		remote_index = 1;
	} else {
		my_index = 1;
		remote_index = 0;
	}

	handle = kzalloc(sizeof(struct messageq_transportshm_object),
					GFP_KERNEL);
	if (handle == NULL) {
		status = -ENOMEM;
		goto exit;
	}

	handle->attrs[0] = (struct messageq_transportshm_attrs *)
				params->shared_addr;
	handle->attrs[1] = (struct messageq_transportshm_attrs *)
				((u32)(handle->attrs[0]) + \
				MESSAGEQ_TRANSPORTSHM_CACHESIZE);
	handle->status = messageq_transportshm_status_INIT;
	handle->gate = params->gate;
	memcpy(&(handle->params), (void *)params,
		sizeof(struct messageq_transportshm_params));

	status = notify_register_event(params->notify_driver, proc_id,
					params->notify_event_no,
					_messageq_transportshm_notify_fxn,
					(void *)handle);
	if (status < 0) {
		/* @retval NULL Notify register failed */
		printk(KERN_ERR "messageq_transportshm_create: "
				"notify_register_event failed!\n");
		goto notify_register_fail;
	}

	handle->notify_driver = params->notify_driver;
	handle->notify_event_no = params->notify_event_no;
	handle->priority	  = params->priority;
	handle->proc_id = proc_id;
	handle->my_index = my_index;
	handle->remote_index = remote_index;

	/* Create the shared lists for the transport. */
	listmp_sharedmemory_params_init(NULL, &(listmp_params[0]));
	listmp_params[0].shared_addr = (u32 *)((u32)(params->shared_addr) + \
			(2 * MESSAGEQ_TRANSPORTSHM_CACHESIZE));
	listmp_params[0].shared_addr_size = \
			listmp_sharedmemory_shared_memreq(&(listmp_params[0]));
	listmp_params[0].gate = params->gate;
	listmp_params[0].name = NULL;
	listmp_params[0].list_type = listmp_type_SHARED;

	listmp_sharedmemory_params_init(NULL, &(listmp_params[1]));
	listmp_params[1].shared_addr = \
				(u32 *)((u32)(listmp_params[0].shared_addr) + \
				listmp_params[0].shared_addr_size);
	listmp_params[1].shared_addr_size = \
		listmp_sharedmemory_shared_memreq(&(listmp_params[1]));
	listmp_params[1].name = NULL;
	listmp_params[1].list_type = listmp_type_SHARED;
	listmp_params[1].gate = params->gate;

	handle->my_listmp_handle = listmp_sharedmemory_create
					(&(listmp_params[my_index]));
	handle->attrs[my_index]->version = MESSAGEQ_TRANSPORTSHM_VERSION;
	handle->attrs[my_index]->flag = MESSAGEQ_TRANSPORTSHM_UP;

	/* Store in VOLATILE to make sure it is not compiled out... */
	otherflag = &(handle->attrs[remote_index]->flag);

	/* Loop until the other side is up */
	while (*otherflag != MESSAGEQ_TRANSPORTSHM_UP)
		;

	if (handle->attrs[remote_index]->version
		!= MESSAGEQ_TRANSPORTSHM_VERSION) {
		/* @retval NULL Versions do not match */
		printk(KERN_ERR "messageq_transportshm_create: "
				"Incorrect version of remote transport!\n");
		goto exit;
	}

	status = listmp_sharedmemory_open
		((listmp_sharedmemory_handle *) &(handle->remote_listmp_handle),
		&listmp_params[remote_index]);
	if (status < 0) {
		/* @retval NULL List creation failed */
		goto listmp_open_fail;
	}

	/* Register the transport with MessageQ */
	status = messageq_register_transport((void *)handle, proc_id,
			(u32)params->priority);
	if (status >= 0) {
		messageq_transportshm_state.transports
			[proc_id][params->priority] = (void *)handle;
		handle->status = messageq_transportshm_status_UP;
	}
	return handle;

listmp_open_fail:
	printk(KERN_ERR "messageq_transportshm_create: "
			"listmp_sharedmemory_open failed!\n");
notify_register_fail:
	if (status < 0) {
		if (handle != NULL)
			messageq_transportshm_delete((void **)(&handle));
	}

exit:
	printk(KERN_ERR "messageq_transportshm_create failed: status = 0x%x\n",
		status);
	return handle;
}

/*
 * ======== messageq_transportshm_delete ========
 *  Purpose:
 *  Delete instance
 */
int messageq_transportshm_delete(void **mqtshm_handleptr)
{
	int status = 0;
	int tmpstatus = 0;
	struct messageq_transportshm_object *obj;

	if (WARN_ON(atomic_cmpmask_and_lt(
			&(messageq_transportshm_state.ref_count),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(1)) == true)) {
		status = -ENODEV;
		goto exit;
	}

	if (WARN_ON(mqtshm_handleptr == NULL)) {
		status = -EINVAL;
		goto exit;
	}
	if (WARN_ON(*mqtshm_handleptr == NULL)) {
		/* @retval MESSAGEQTRANSPORTSHM_E_HANDLE Invalid NULL handle
		specified */
		status = MESSAGEQ_TRANSPORTSHM_E_HANDLE;
		printk(KERN_WARNING "messageq_transportshm_delete: Invalid NULL"
			" mqtshm_handle specified! status = 0x%x\n", status);
		goto exit;
	}

	obj = (struct messageq_transportshm_object *) (*mqtshm_handleptr);
	/* Clear handle in the local array */
	messageq_transportshm_state.transports[obj->proc_id][obj->priority] = \
		NULL;
	obj->attrs[obj->my_index]->flag = 0;
	status = listmp_sharedmemory_delete(
			(listmp_sharedmemory_handle *)&obj->my_listmp_handle);
	if (status < 0) {
		printk(KERN_WARNING "messageq_transportshm_delete: Failed to "
			"delete listmp_sharedmemory instance!\n");
	}

	tmpstatus = listmp_sharedmemory_close(
			(listmp_sharedmemory_handle) obj->remote_listmp_handle);
	if ((tmpstatus < 0) && (status >= 0)) {
		status = tmpstatus;
		printk(KERN_WARNING "messageq_transportshm_delete: Failed to "
			"close listmp_sharedmemory instance!\n");
	}

	tmpstatus = messageq_unregister_transport(obj->proc_id,
							obj->params.priority);
	if ((tmpstatus < 0) && (status >= 0)) {
		status = tmpstatus;
		printk(KERN_WARNING "messageq_transportshm_delete: Failed to "
			"unregister transport!\n");
	}

	tmpstatus = notify_unregister_event(obj->notify_driver, obj->proc_id,
					obj->notify_event_no,
					_messageq_transportshm_notify_fxn,
					(void *)obj);
	if ((tmpstatus < 0) && (status >= 0)) {
		status = tmpstatus;
		printk(KERN_WARNING "messageq_transportshm_delete: Failed to "
			"unregister notify event!\n");
	}

	kfree(obj);
	*mqtshm_handleptr = NULL;

exit:
	if (status < 0)
		printk(KERN_ERR "messageq_transportshm_delete failed: "
			"status = 0x%x\n", status);
	return status;
}

/*
 * ======== messageq_transportshm_put ========
 *  Purpose:
 *  Put msg to remote list
*/
int  messageq_transportshm_put(void *mqtshm_handle,
				void *msg)
{
	int status = 0;
	struct messageq_transportshm_object *obj = \
			(struct messageq_transportshm_object *) mqtshm_handle;

	if (WARN_ON(atomic_cmpmask_and_lt(
			&(messageq_transportshm_state.ref_count),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(1)) == true)) {
		status = -ENODEV;
		goto exit;
	}

	BUG_ON(mqtshm_handle == NULL);
	if (WARN_ON(msg == NULL)) {
		status = -EINVAL;
		goto exit;
	}
	if (WARN_ON(obj == NULL)) {
		status = -EINVAL;
		goto exit;
	}

	status = listmp_put_tail(obj->remote_listmp_handle,
					(struct listmp_elem *) msg);
	if (status < 0) {
		/* @retval MESSAGEQ_TRANSPORTSHM_E_FAIL
		*	  Notification to remote processor failed!
		*/
		status = MESSAGEQ_TRANSPORTSHM_E_FAIL;
		printk(KERN_ERR "messageq_transportshm_put: Failed to put "
			"message in the shared list! status = 0x%x\n", status);
		goto exit;
	}

	status = notify_sendevent(obj->notify_driver, obj->proc_id,
					obj->notify_event_no, 0, true);
	if (status < 0)
		goto notify_send_fail;
	else
		goto exit;

notify_send_fail:
	printk(KERN_ERR "messageq_transportshm_put: Notification to remote "
		"processor failed, status = 0x%x\n", status);
	/* If sending the event failed, then remove the element from the list.*/
	/* Ignore the status of remove. */
	listmp_remove(obj->remote_listmp_handle, (struct listmp_elem *) msg);

exit:
	if (status < 0)
		printk(KERN_ERR "messageq_transportshm_put failed: "
			"status = 0x%x\n", status);
	return status;
}

/*
 * ======== messageq_transportshm_control ========
 *  Purpose:
 *  Control Function
*/
int messageq_transportshm_control(void *mqtshm_handle, u32 cmd, u32 *cmdArg)
{
	BUG_ON(mqtshm_handle == NULL);

	printk(KERN_ALERT "messageq_transportshm_control not supported!\n");
	return MESSAGEQTRANSPORTSHM_E_NOTSUPPORTED;
}

/*
 * ======== messageq_transportshm_get_status ========
 *  Purpose:
 *  Get status
 */
enum messageq_transportshm_status messageq_transportshm_get_status(
					void *mqtshm_handle)
{
	struct messageq_transportshm_object *obj = \
			(struct messageq_transportshm_object *) mqtshm_handle;

	BUG_ON(obj == NULL);

	return obj->status;
}

/*
 * ======== messageq_transportshm_put ========
 *  Purpose:
 *  Get shared memory requirements.
 */
u32 messageq_transportshm_shared_mem_req(const
				struct messageq_transportshm_params *params)
{
	u32 totalSize;
	listmp_sharedmemory_params listmp_params;
	u32 listmp_size;

	/* There are two transport flags in shared memory */
	totalSize = 2 * MESSAGEQ_TRANSPORTSHM_CACHESIZE;

	listmp_sharedmemory_params_init(NULL, &listmp_params);
	listmp_size = listmp_sharedmemory_shared_memreq(&listmp_params);

	/* MyList */
	totalSize += listmp_size;

	/* RemoteList */
	totalSize += listmp_size;

	return totalSize;
}


/* =============================================================================
 * internal functions
 * =============================================================================
 */
/*
 * ======== _messageq_transportshm_notify_fxn ========
 *  Purpose:
 *  Callback function registered with the Notify module.
 */
void _messageq_transportshm_notify_fxn(u16 proc_id, u32 event_no,
					void *arg, u32 payload)
{
	struct messageq_transportshm_object *obj = NULL;
	messageq_msg msg = NULL;
	u32 queue_id;

	if (WARN_ON(arg == NULL))
		goto exit;

	obj = (struct messageq_transportshm_object *)arg;
	/*  While there is are messages, get them out and send them to
	 *  their final destination. */
	while ((msg = (messageq_msg) listmp_get_head(obj->my_listmp_handle))
		!= NULL) {
		/* Get the destination message queue Id */
		queue_id = messageq_get_dst_queue(msg);
		messageq_put(queue_id, msg);
	}
	return;

exit:
	printk(KERN_ERR "messageq_transportshm_notify_fxn: argument passed is "
		"NULL!\n");
}


/*
 * ======== messageq_transportshm_delete ========
 *  Purpose:
 *  This will set the asynchronous error function for the transport module
 */
void messageq_transportshm_set_err_fxn(
				void (*err_fxn)(
				enum MessageQTransportShm_Reason reason,
				void *handle,
				void *msg,
				u32 info))
{
	u32 key;

	key = mutex_lock_interruptible(messageq_transportshm_state.gate_handle);
	if (key < 0)
		goto exit;

	messageq_transportshm_state.cfg.err_fxn = err_fxn;
	mutex_unlock(messageq_transportshm_state.gate_handle);

exit:
	return;
}


