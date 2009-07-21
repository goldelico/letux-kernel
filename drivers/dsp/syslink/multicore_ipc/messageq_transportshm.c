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
#include <gt.h>
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
};

/*
 * Structure of attributes in shared memory
 */
struct messageq_transportshm_attrs {
	volatile u32 version;
	volatile u32 flag;
};

/*
 * Structure defining config parameters for the MessageQ transport
 *  instances.
 */
struct messageq_transportshm_object {
	volatile struct messageq_transportshm_attrs *attrs[2];
	/* Attributes for both processors */
	void *my_listmp_handle;
	/* List for this processor	*/
	void *remote_listmp_handle;
	/* List for remote processor	*/
	volatile int status;
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

#if GT_TRACE
static struct GT_Mask mqtshm_debugmask = { NULL, NULL }; /* GT trace variable */
EXPORT_SYMBOL(mqtshm_debugmask);
#endif

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
	gt_1trace(mqtshm_debugmask, GT_ENTER,
			"messageq_transportshm_getConfig", cfg);

	BUG_ON(cfg == NULL);
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

exit:
	gt_0trace(mqtshm_debugmask, GT_LEAVE,
			"messageq_transportshm_getConfig");
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

	gt_1trace(mqtshm_debugmask, GT_ENTER,
			"messageq_transportshm_setup", cfg);

	/* This sets the refCount variable is not initialized, upper 16 bits is
	* written with module Id to ensure correctness of refCount variable.
	*/
    atomic_cmpmask_and_set(&messageq_transportshm_state.ref_count,
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0));

	if (atomic_inc_return(&messageq_transportshm_state.ref_count)
		!= MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(1u)) {
		status = -EEXIST;;
		goto exit;
	}

	if (cfg == NULL) {
		messageq_transportshm_get_config(&tmpCfg);
		cfg = &tmpCfg;
	}

	messageq_transportshm_state.gate_handle = \
				kmalloc(sizeof(struct mutex), GFP_KERNEL);
	mutex_init(messageq_transportshm_state.gate_handle);

	if (messageq_transportshm_state.gate_handle == NULL) {
		/* @retval MESSAGEQTRANSPORTSHM_E_FAIL Failed to create
		GateMutex! */
		status = MESSAGEQ_TRANSPORTSHM_E_FAIL;
		gt_2trace(mqtshm_debugmask,
				GT_4CLASS,
				"messageq_transportshm_setup",
				status,
				"Failed to create GateMutex!");
		atomic_set(&messageq_transportshm_state.ref_count,
				MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0));
		goto exit;
	}

	/* Copy the user provided values into the state object. */
	memcpy(&messageq_transportshm_state.cfg, cfg,
			sizeof(struct messageq_transportshm_config));

exit:
	gt_1trace(mqtshm_debugmask, GT_LEAVE,
			"messageq_transportshm_setup", status);
	/* @retval MESSAGEQTRANSPORTSHM_SUCCESS Operation successful */
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

	gt_0trace(mqtshm_debugmask, GT_ENTER, "messageq_transportshm_destroy");

	if (WARN_ON(atomic_cmpmask_and_lt(
			&(messageq_transportshm_state.ref_count),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(1)) == true)) {
		status = -ENODEV;
		goto exit;
	}

	if (atomic_dec_return(&messageq_transportshm_state.ref_count)
				== MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0)) {
		if (messageq_transportshm_state.gate_handle != NULL) {
			kfree(messageq_transportshm_state.gate_handle);
			messageq_transportshm_state.gate_handle = NULL;
		}
		/* Decrease the ref_count */
		atomic_set(&messageq_transportshm_state.ref_count,
				MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0));
	}

exit:
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

	gt_2trace(mqtshm_debugmask, GT_ENTER,
			"messageq_transportshm_params_init", mqtshm_handle,
			params);

	if (WARN_ON(atomic_cmpmask_and_lt(
			&(messageq_transportshm_state.ref_count),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	BUG_ON(params == NULL);
	if (WARN_ON(params == NULL)) {
		/* @retval None */
		gt_2trace(mqtshm_debugmask,
				GT_4CLASS,
				"messageq_transportshm_params_init",
				MESSAGEQ_TRANSPORTSHM_E_INVALIDARG,
				"Argument of type "
				"(messageq_transportshm_params *) is "
				"NULL!");
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
	gt_0trace(mqtshm_debugmask, GT_LEAVE,
		"messageq_transportshm_params_init");
	/* @retval None */
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
	volatile u32 *otherflag;

	gt_2trace(mqtshm_debugmask, GT_ENTER, "messageq_transportshm_create",
			proc_id, params);

	if (WARN_ON(atomic_cmpmask_and_lt(
			&(messageq_transportshm_state.ref_count),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(1)) == true))
		goto exit;

	BUG_ON(params == NULL);
	if (WARN_ON(params == NULL)) {
		status = -EINVAL;
		goto exit;
	}
	if (WARN_ON(params->shared_addr_size < \
		messageq_transportshm_shared_mem_req(params))) {
		status = -EINVAL;
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
		gt_2trace(mqtshm_debugmask,
					GT_4CLASS,
					"messageq_transportshm_create",
					MESSAGEQ_TRANSPORTSHM_E_FAIL,
					"Notify register failed!");
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

	/* Store in volatile to make sure it is not compiled out... */
	otherflag = &(handle->attrs[remote_index]->flag);
	gt_1trace(mqtshm_debugmask, GT_1CLASS, "messageq_transportshm_create\n"
		"Synchronization flag addr [0x%x]", otherflag);

	/* Loop until the other side is up */
	while (*otherflag != MESSAGEQ_TRANSPORTSHM_UP)
		;

	if (handle->attrs[remote_index]->version
		!= MESSAGEQ_TRANSPORTSHM_VERSION) {
		/* @retval NULL Versions do not match */
		gt_2trace(mqtshm_debugmask, GT_4CLASS,
				"messageq_transportshm_create",
				MESSAGEQ_TRANSPORTSHM_E_BADVERSION,
				"Incorrect version of remote transport!");
		goto exit;
	}

	status = listmp_sharedmemory_open
		((listmp_sharedmemory_handle *) &(handle->remote_listmp_handle),
		&listmp_params[remote_index]);
	if (status < 0) {
		/* @retval NULL List creation failed */
		gt_2trace(mqtshm_debugmask,
			GT_4CLASS,
			"messageq_transportshm_create",
			status,
			"List creation failed!");
		goto listmp_open_fail;
	}

	/* Register the transport with MessageQ */
	status = messageq_register_transport((void *)handle, proc_id,
			(u32)params->priority);
	if (status >= 0)
		handle->status = messageq_transportshm_status_UP;

listmp_open_fail:
notify_register_fail:
	if (status < 0) {
		if (handle != NULL) {
			kfree(handle);
			handle = NULL;
		}
	}

exit:
	gt_1trace(mqtshm_debugmask, GT_LEAVE, "messageq_transportshm_create",
		handle);
	/* @retval Valid handle of type messageq_transportshm_handle
	 Operation successful */
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

	gt_1trace(mqtshm_debugmask, GT_ENTER, "messageq_transportshm_delete",
			mqtshm_handleptr);

	if (WARN_ON(atomic_cmpmask_and_lt(
			&(messageq_transportshm_state.ref_count),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(1)) == true)) {
		status = -ENODEV;
		goto exit;
	}

	BUG_ON(mqtshm_handleptr == NULL);
	if (WARN_ON(mqtshm_handleptr == NULL)) {
		status = -EINVAL;
		goto exit;
	}
	if (WARN_ON(*mqtshm_handleptr == NULL)) {
		/* @retval MESSAGEQTRANSPORTSHM_E_HANDLE Invalid NULL handle
		specified */
		status = MESSAGEQ_TRANSPORTSHM_E_HANDLE;
		gt_2trace(mqtshm_debugmask,
				GT_4CLASS,
				"messageq_transportshm_delete",
				MESSAGEQ_TRANSPORTSHM_E_HANDLE,
				"Invalid NULL mqtshm_handle specified");
		goto exit;
	}

	obj = (struct messageq_transportshm_object *) (*mqtshm_handleptr);
	obj->attrs[obj->my_index]->flag = 0;
	status = listmp_sharedmemory_delete(
			(listmp_sharedmemory_handle *)&obj->my_listmp_handle);
	if (status < 0) {
		gt_2trace(mqtshm_debugmask, GT_4CLASS,
				"MessageQTransportShm_delete", status,
				"Failed to delete ListMPSharedMemory"
				" instance!");
	}

	tmpstatus = listmp_sharedmemory_close(
			(listmp_sharedmemory_handle) obj->remote_listmp_handle);
	if ((tmpstatus < 0) && (status >= 0)) {
		status = tmpstatus;
		gt_2trace(mqtshm_debugmask, GT_4CLASS,
			"MessageQTransportShm_delete",
			status, "Failed to close ListMPSharedMemory instance!");
	}

	tmpstatus = messageq_unregister_transport(obj->proc_id,
							obj->params.priority);
	if ((tmpstatus < 0) && (status >= 0)) {
		status = tmpstatus;
		gt_2trace(mqtshm_debugmask, GT_4CLASS,
				"MessageQTransportShm_delete", status,
				"Failed to unregister transport");
	}

	tmpstatus = notify_unregister_event(obj->notify_driver, obj->proc_id,
					obj->notify_event_no,
					_messageq_transportshm_notify_fxn,
					(void *)obj);
	if ((tmpstatus < 0) && (status >= 0)) {
		status = tmpstatus;
		gt_2trace(mqtshm_debugmask, GT_4CLASS,
				"MessageQTransportShm_delete",
				status, "Failed to unregister Notify event");
	}

	kfree(obj);
	*mqtshm_handleptr = NULL;

exit:
	gt_0trace(mqtshm_debugmask, GT_LEAVE, "messageq_transportshm_delete");
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

	gt_2trace(mqtshm_debugmask, GT_ENTER, "messageq_transportshm_put",
		obj, msg);
	if (WARN_ON(atomic_cmpmask_and_lt(
			&(messageq_transportshm_state.ref_count),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(0),
			MESSAGEQTRANSPORTSHM_MAKE_MAGICSTAMP(1)) == true)) {
		status = -ENODEV;
		goto exit;
	}

	BUG_ON(mqtshm_handle == NULL);
	BUG_ON(msg == NULL);

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
		gt_2trace(mqtshm_debugmask,
				GT_4CLASS,
				"messageq_transportshm_put",
				status,
				"Failed to put message in the shared list!");
		goto exit;
	}

	status = notify_sendevent(obj->notify_driver, obj->proc_id,
					obj->notify_event_no, 0, false);
	if (status < 0)
		goto notify_send_fail;
	else
		goto exit;

notify_send_fail:
	gt_2trace(mqtshm_debugmask, GT_4CLASS, "messageq_transportshm_put",
		status, "Notification to remote processor failed!");
	/* If sending the event failed, then remove the element from the list.*/
	/* Ignore the status of remove. */
	listmp_remove(obj->remote_listmp_handle, (struct listmp_elem *) msg);

exit:
	gt_1trace(mqtshm_debugmask, GT_LEAVE, "messageq_transportshm_put",
			status);
	return status;
}

/*
 * ======== messageq_transportshm_control ========
 *  Purpose:
 *  Control Function
*/
int messageq_transportshm_control(void *mqtshm_handle, u32 cmd, u32 *cmdArg)
{
	gt_3trace(mqtshm_debugmask, GT_ENTER, "messageq_transportshm_control",
			mqtshm_handle, cmd, cmdArg);

	BUG_ON(mqtshm_handle == NULL);

	gt_1trace(mqtshm_debugmask, GT_LEAVE, "messageq_transportshm_control",
			MESSAGEQTRANSPORTSHM_E_NOTSUPPORTED);
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

	gt_1trace(mqtshm_debugmask, GT_ENTER,
		"messageq_transportshm_getStatus", obj);

	BUG_ON(obj == NULL);

	gt_1trace(mqtshm_debugmask, GT_LEAVE, "messageq_transportshm_getStatus",
			obj->status);
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

	gt_1trace(mqtshm_debugmask, GT_ENTER,
		"messageq_transportshm_sharedMemReq", params);

	/* There are two transport flags in shared memory */
	totalSize = 2 * MESSAGEQ_TRANSPORTSHM_CACHESIZE;

	listmp_sharedmemory_params_init(NULL, &listmp_params);
	listmp_size = listmp_sharedmemory_shared_memreq(&listmp_params);

	/* MyList */
	totalSize += listmp_size;

	/* RemoteList */
	totalSize += listmp_size;

	gt_1trace(mqtshm_debugmask, GT_LEAVE,
		"messageq_transportshm_sharedMemReq", totalSize);
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

	gt_4trace(mqtshm_debugmask, GT_ENTER,
			"_messageq_transportshm_notify_fxn",
			proc_id, event_no, arg, payload);

	BUG_ON(arg == NULL);
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

exit:
	gt_0trace(mqtshm_debugmask, GT_LEAVE,
		"messageq_transportshm_notifyFxn");
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


