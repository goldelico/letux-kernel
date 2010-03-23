/*
 *  nameserver_remotenotify.c
 *
 *  The nameserver_remotenotify module provides functionality to get name
 *  value pair from a remote nameserver.
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

/* Standard headers */
#include <linux/types.h>

/* Utilities headers */
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/semaphore.h>

/* Syslink headers */
#include <syslink/atomic_linux.h>

/* Module level headers */
#include <gate_remote.h>
#include <gatepeterson.h>
#include <nameserver.h>
#include <multiproc.h>
#include <nameserver_remotenotify.h>
#include <notify.h>


/*
 *  Macro to make a correct module magic number with refCount
 */
#define NAMESERVERREMOTENOTIFY_MAKE_MAGICSTAMP(x)	\
			((NAMESERVERREMOTENOTIFY_MODULEID << 12u) | (x))

/*
 *  Cache line length
 *  TODO: Short-term hack. Make parameter or figure out some other way!
 */
#define NAMESERVERREMOTENOTIFY_CACHESIZE	128

/*
 *  Maximum length of value buffer that can be stored in the NameServer
 *  managed by this NameServerRemoteNotify instance.
 */
#define NAMESERVERREMOTENOTIFY_MAXVALUEBUFLEN	300

/* Defines the nameserver_remotenotify state object, which contains all the
 * module specific information
 */
struct nameserver_remotenotify_module_object {
	struct nameserver_remotenotify_config cfg;
	struct nameserver_remotenotify_config def_cfg;
	struct nameserver_remotenotify_params def_inst_params;
	bool is_setup;
	void *gate_handle;
	atomic_t ref_count;
};

/*
 *  NameServer remote transport state attributes
 */
struct nameserver_remotenotify_attrs {
	u32 version;
	u32 status;
	u32 shared_addr_size;
};

/*
 *  NameServer remote transport packet definition
 */
struct nameserver_remotenotify_message {
	u32 request;
	u32 response;
	u32 request_status;
	u32 value;
	u32 value_len;
	char instance_name[32];
	char name[32];
	char value_buf[NAMESERVERREMOTENOTIFY_MAXVALUEBUFLEN];
};

/*
 *  NameServer remote transport state object definition
 */
struct nameserver_remotenotify_obj {
	struct nameserver_remotenotify_attrs *attrs;
	struct nameserver_remotenotify_message *msg[2];
	struct nameserver_remotenotify_params params;
	u16 remote_proc_id;
	struct mutex *local_gate;
	struct semaphore *sem_handle; /* Binary semaphore */
};

/*
 *  NameServer remote transport state object definition
 */
struct nameserver_remotenotify_object {
	int (*get)(void *,
		const char *instance_name, const char *name,
		void *value, u32 value_len, void *reserved);
	void *obj; /* Implementation specific object */
};

/*
 *  nameserver_remotenotify state object variable
 */
static struct nameserver_remotenotify_module_object
				nameserver_remotenotify_state = {
	.is_setup = false,
	.gate_handle = NULL,
	.def_cfg.reserved = 0,
	.def_inst_params.gate = NULL,
	.def_inst_params.shared_addr = 0x0,
	.def_inst_params.shared_addr_size = 0x0,
	.def_inst_params.notify_event_no = (u32) -1,
	.def_inst_params.notify_driver = NULL,
};

/*
 * ======== nameserver_remotenotify_get_config ========
 *  Purpose:
 *  This will get the default configuration for the  nameserver remote
 *  module
 *  This function can be called by the application to get their
 *  configuration parameter to nameserver_remotenotify_setup filled
 *  in by the nameserver_remotenotify module with the default
 *  parameters. If the user does not wish to make any change in the
 *  default parameters, this API is not required to be called
 */
void nameserver_remotenotify_get_config(
			struct nameserver_remotenotify_config *cfg)
{
	BUG_ON(cfg == NULL);
	if (nameserver_remotenotify_state.is_setup == false)
		memcpy(cfg, &(nameserver_remotenotify_state.def_cfg),
			sizeof(struct nameserver_remotenotify_config));
	else
		memcpy(cfg, &(nameserver_remotenotify_state.cfg),
			sizeof(struct nameserver_remotenotify_config));
}


/*
 * ======== nameserver_remotenotify_setup ========
 *  Purpose:
 *  This will setup the nameserver_remotenotify module
 *  This function sets up the nameserver_remotenotify module. This
 *  function must be called before any other instance-level APIs can
 *  be invoked
 *  Module-level configuration needs to be provided to this
 *  function. If the user wishes to change some specific config
 *  parameters, then nameserver_remotenotify_get_config can be called
 *  to get the configuration filled with the default values. After
 *  this, only the required configuration values can be changed. If
 *  the user does not wish to make any change in the default
 *  parameters, the application can simply call
 *  nameserver_remotenotify_setup with NULL parameters. The default
 *  parameters would get automatically used
 */
int nameserver_remotenotify_setup(
				struct nameserver_remotenotify_config *cfg)
{
	struct nameserver_remotenotify_config tmp_cfg;
	s32 retval = 0;
	struct mutex *lock = NULL;
	bool user_cfg = true;

	/* This sets the ref_count variable is not initialized, upper 16 bits is
	* written with module Id to ensure correctness of refCount variable.
	*/
	atomic_cmpmask_and_set(&nameserver_remotenotify_state.ref_count,
				NAMESERVERREMOTENOTIFY_MAKE_MAGICSTAMP(0),
				NAMESERVERREMOTENOTIFY_MAKE_MAGICSTAMP(0));
	if (atomic_inc_return(&nameserver_remotenotify_state.ref_count)
				!= NAMESERVERREMOTENOTIFY_MAKE_MAGICSTAMP(1)) {
		return 1;
	}

	if (cfg == NULL) {
		nameserver_remotenotify_get_config(&tmp_cfg);
		cfg = &tmp_cfg;
		user_cfg = false;
	}

	/* Create a default gate handle for local module protection */
	lock = kmalloc(sizeof(struct mutex), GFP_KERNEL);
	if (lock == NULL) {
		retval = -ENOMEM;
		goto exit;
	}

	mutex_init(lock);
	nameserver_remotenotify_state.gate_handle = lock;
	if (user_cfg)
		memcpy(&nameserver_remotenotify_state.cfg,  cfg,
			sizeof(struct nameserver_remotenotify_config));

	nameserver_remotenotify_state.is_setup = true;

exit:
	return retval;
}

/*
 * ======== nameserver_remotenotify_destroy ========
 *  Purpose:
 *  This will destroy the nameserver_remotenotify module.
 *  Once this function is called, other nameserver_remotenotify
 *  module APIs, except for the nameserver_remotenotify_get_config
 *  API  cannot be called anymore.
 */
int nameserver_remotenotify_destroy(void)
{
	s32 retval = 0;

	if (WARN_ON(atomic_cmpmask_and_lt(
			&(nameserver_remotenotify_state.ref_count),
			NAMESERVERREMOTENOTIFY_MAKE_MAGICSTAMP(0),
			NAMESERVERREMOTENOTIFY_MAKE_MAGICSTAMP(1)) == true)) {
		retval = -ENODEV;
		goto exit;
	}

	if (!(atomic_dec_return(&nameserver_remotenotify_state.ref_count)
			== NAMESERVERREMOTENOTIFY_MAKE_MAGICSTAMP(0))) {
		retval = 1;
		goto exit;
	}

	if (nameserver_remotenotify_state.gate_handle != NULL)
		kfree(nameserver_remotenotify_state.gate_handle);

	nameserver_remotenotify_state.is_setup = false;
	return 0;

exit:
	return retval;
}

/*
 * ======== ameserver_remotenotify_params_init ========
 *  Purpose:
 *  This will get the current configuration values
 */
void nameserver_remotenotify_params_init(void *handle,
			struct nameserver_remotenotify_params *params)
{
	struct nameserver_remotenotify_object *object = NULL;
	struct nameserver_remotenotify_obj *obj = NULL;

	if (atomic_cmpmask_and_lt(&(nameserver_remotenotify_state.ref_count),
			NAMESERVERREMOTENOTIFY_MAKE_MAGICSTAMP(0),
			NAMESERVERREMOTENOTIFY_MAKE_MAGICSTAMP(1)) == true) {
		printk(KERN_ERR "nameserver_remotenotify_params_init failed: "
			"Module is not initialized!\n");
		return;
	}

	if (WARN_ON(params == NULL)) {
		printk(KERN_ERR "nameserver_remotenotify_params_init failed: "
			"Argument of type(nameserver_remotenotify_params *) "
			"is NULL!\n");
		return;
	}

	object = (struct nameserver_remotenotify_object *)handle;
	if (handle == NULL)
		memcpy(params,
			&(nameserver_remotenotify_state.def_inst_params),
			sizeof(struct nameserver_remotenotify_params));
	else {
		obj = (struct nameserver_remotenotify_obj *)object->obj;
		/* Return updated nameserver_remotenotify instance specific
		 * parameters.
		 */
		 memcpy(params, &(obj->params),
			 sizeof(struct nameserver_remotenotify_params));
	}
}


/*
 * ======== nameserver_remotenotify_callback ========
 *  Purpose:
 *  This will be called when a notify event is received
 */
void nameserver_remotenotify_callback(u16 proc_id, u32 event_no,
						void *arg, u32 payload)
{
	struct nameserver_remotenotify_obj *handle = NULL;
	u32 proc_count;
	u16  offset = 0;
	void *nshandle = NULL;
	u32 value_len;
	u32 key;
	s32 retval = 0;
	s32 count = -ENOENT;

	if (atomic_cmpmask_and_lt(&(nameserver_remotenotify_state.ref_count),
			NAMESERVERREMOTENOTIFY_MAKE_MAGICSTAMP(0),
			NAMESERVERREMOTENOTIFY_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto exit;
	}

	if (WARN_ON(arg == NULL)) {
		retval = -EINVAL;
		goto exit;
	}

	proc_count = multiproc_get_max_processors();
	if (WARN_ON(proc_id >= proc_count)) {
		retval = -EINVAL;
		goto exit;
	}

	handle = (struct nameserver_remotenotify_obj *)arg;
	if ((multiproc_get_id(NULL) > proc_id))
		offset = 1;

	if (handle->msg[1 - offset]->request != true)
		goto signal_response;

	/* This is a request */
	value_len = handle->msg[1 - offset]->value_len;
	nshandle = nameserver_get_handle(
				handle->msg[1 - offset]->instance_name);
	if (nshandle != NULL) {
		/* Search for the NameServer entry */
		if (value_len == sizeof(u32)) {
			count = nameserver_get_local(nshandle,
					handle->msg[1 - offset]->name,
					&handle->msg[1 - offset]->value,
					value_len);
		} else {
			count = nameserver_get_local(nshandle,
					handle->msg[1 - offset]->name,
					&handle->msg[1 - offset]->value_buf,
					value_len);
		}
	}

	key = gatepeterson_enter(handle->params.gate);
	/* If retval > 0 then an entry found */
	if (count != -ENOENT)
		handle->msg[1 - offset]->request_status = true;

	/* Send a response back */
	handle->msg[1 - offset]->response = true;
	handle->msg[1 - offset]->request = false;
	/* now we can leave the gate */
	gatepeterson_leave(handle->params.gate, key);

	/*
	*  The NotifyDriver handle must exists at this point,
	*  otherwise the notify_sendEvent should have failed
	*/
	retval = notify_sendevent(handle->params.notify_driver,
				proc_id, event_no, 0, true);

signal_response:
	if (handle->msg[offset]->response == true)
		up(handle->sem_handle);
exit:
	if (retval < 0) {
		printk(KERN_ERR "nameserver_remotenotify_callback failed! "
			"status = 0x%x\n", retval);
	}
	return;
}

/*
 * ======== nameserver_remotenotify_get ========
 *  Purpose:
 *  This will get a remote name value pair
 */
int nameserver_remotenotify_get(void *rhandle,
				const char *instance_name, const char *name,
				void *value, u32 value_len, void *reserved)
{
	struct nameserver_remotenotify_object *handle = NULL;
	struct nameserver_remotenotify_obj *obj = NULL;
	s32 offset = 0;
	s32 len;
	u32 key;
	s32 retval = 0;

	BUG_ON(instance_name == NULL);
	BUG_ON(name == NULL);
	BUG_ON(value == NULL);
	BUG_ON((value_len <= 0) || \
		(value_len > NAMESERVERREMOTENOTIFY_MAXVALUEBUFLEN));

	if (atomic_cmpmask_and_lt(&(nameserver_remotenotify_state.ref_count),
			NAMESERVERREMOTENOTIFY_MAKE_MAGICSTAMP(0),
			NAMESERVERREMOTENOTIFY_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto exit;
	}

	if (WARN_ON(rhandle == NULL)) {
		retval = -EINVAL;
		goto exit;
	}

	if ((value_len == 0) || \
		(value_len > NAMESERVERREMOTENOTIFY_MAXVALUEBUFLEN)) {
		retval = -EINVAL;
		goto exit;
	}

	handle = (struct nameserver_remotenotify_object *)rhandle;
	obj = (struct nameserver_remotenotify_obj *)handle->obj;
	if (multiproc_get_id(NULL) > obj->remote_proc_id)
		offset = 1;

	key = gatepeterson_enter(obj->params.gate);
	/* This is a request message */
	obj->msg[offset]->request = 1;
	obj->msg[offset]->response = 0;
	obj->msg[offset]->request_status = 0;
	obj->msg[offset]->value_len = value_len;
	len = strlen(instance_name) + 1; /* Take termination null char */
	if (len >= 32) {
		retval = -EINVAL;
		goto inval_len_error;
	}
	strncpy(obj->msg[offset]->instance_name, instance_name, len);
	len = strlen(name);
	if (len >= 32) {
		retval = -EINVAL;
		goto inval_len_error;
	}
	strncpy(obj->msg[offset]->name, name, len);

	/* Send the notification to remote processor */
	retval = notify_sendevent(obj->params.notify_driver,
			obj->remote_proc_id,
			obj->params.notify_event_no,
			0, /* Payload */
			false); /* Not sending a payload */
	if (retval < 0) {
		/* Undo previous operations */
		obj->msg[offset]->request = 0;
		obj->msg[offset]->value_len = 0;
		goto notify_error;
	}

	gatepeterson_leave(obj->params.gate, key);

	/* Pend on the semaphore */
	retval = down_interruptible(obj->sem_handle);
	if (retval) {
		goto exit;
	}

	key = gatepeterson_enter(obj->params.gate);
	if (obj->msg[offset]->request_status != true) {
		retval = -ENOENT;
		goto request_error;
	}

	if (!value_len) {
		retval = -ENOENT;
		goto request_error;
	}

	if (value_len == sizeof(u32))
		memcpy((void *)value, (void *) &(obj->msg[offset]->value),
			sizeof(u32));
	else
		memcpy((void *)value, (void *)&(obj->msg[offset]->value_buf),
			value_len);

	obj->msg[offset]->request_status = false;
	obj->msg[offset]->request = 0;
	obj->msg[offset]->response = 0;
	retval = value_len;

inval_len_error:
notify_error:
request_error:
	gatepeterson_leave(obj->params.gate, key);
exit:
	return retval;
}

/*
 * ======== nameServer_remote_notify_params_init ========
 *  Purpose:
 *  This will get the current configuration values
 */
int nameServer_remote_notify_params_init(
			struct nameserver_remotenotify_params *params)
{
	BUG_ON(params == NULL);

	params->notify_event_no = 0;
	params->notify_driver = NULL;
	params->shared_addr = NULL;
	params->shared_addr_size = 0;
	params->gate = NULL;
	return 0;
}

/*
 * ======== nameserver_remotenotify_create ========
 *  Purpose:
 *  This will setup the nameserver remote module
 */
void *nameserver_remotenotify_create(u16 proc_id,
			const struct nameserver_remotenotify_params *params)
{
	struct nameserver_remotenotify_object *handle = NULL;
	struct nameserver_remotenotify_obj *obj = NULL;
	u16 proc_count;
	s32 retval = 0;
	s32 retval1 = 0;
	u32 offset = 0;

	if (atomic_cmpmask_and_lt(&(nameserver_remotenotify_state.ref_count),
			NAMESERVERREMOTENOTIFY_MAKE_MAGICSTAMP(0),
			NAMESERVERREMOTENOTIFY_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto exit;
	}

	if (WARN_ON(params == NULL)) {
		retval = -EINVAL;
		goto exit;
	}

	if (WARN_ON(params->notify_driver == NULL ||
		params->shared_addr == NULL ||
		params->shared_addr_size == 0)) {
		retval = -EINVAL;
		goto exit;
	}

	proc_count = multiproc_get_max_processors();
	if (proc_id >= proc_count) {
		retval = -EINVAL;
		goto exit;
	}

	obj = kmalloc(sizeof(struct nameserver_remotenotify_obj), GFP_KERNEL);
	handle = kmalloc(sizeof(struct nameserver_remotenotify_object),
								GFP_KERNEL);
	if (obj == NULL || handle == NULL) {
		retval = -ENOMEM;
		goto mem_error;
	}

	handle->get =  nameserver_remotenotify_get;
	handle->obj = (void *)obj;
	obj->local_gate = kmalloc(sizeof(struct mutex), GFP_KERNEL);
	if (obj->local_gate == NULL) {
		retval = -ENOMEM;
		goto mem_error;
	}

	obj->remote_proc_id = proc_id;
	if (multiproc_get_id(NULL) > proc_id)
		offset = 1;

	obj->attrs = (struct nameserver_remotenotify_attrs *)
							params->shared_addr;
	obj->msg[0] = (struct nameserver_remotenotify_message *)
					((u32)obj->attrs  +
					NAMESERVERREMOTENOTIFY_CACHESIZE);
	obj->msg[1] = (struct nameserver_remotenotify_message *)
					((u32)obj->msg[0] +
					sizeof(struct
					nameserver_remotenotify_message));
	/* Clear out self shared structures */
	memset(obj->msg[offset], 0,
			sizeof(struct nameserver_remotenotify_message));
	memcpy(&obj->params, params,
				sizeof(struct nameserver_remotenotify_params));
	retval = notify_register_event(params->notify_driver, proc_id,
					params->notify_event_no,
					nameserver_remotenotify_callback,
					(void *)obj);
	if (retval < 0)
		goto notify_error;

	retval = nameserver_register_remote_driver((void *)handle, proc_id);
	obj->sem_handle = kmalloc(sizeof(struct semaphore), GFP_KERNEL);
	if (obj->sem_handle == NULL) {
		retval = -ENOMEM;
		goto sem_alloc_error;
	}

	sema_init(obj->sem_handle, 0);
	/* its is at the end since its init state = unlocked? */
	mutex_init(obj->local_gate);
	return (void *)handle;

sem_alloc_error:
	nameserver_unregister_remote_driver(proc_id);
	/* Do we want to check the staus ? */
	retval1 = notify_unregister_event(obj->params.notify_driver,
				obj->remote_proc_id,
				obj->params.notify_event_no,
				nameserver_remotenotify_callback,
				(void *)obj);

notify_error:
	kfree(obj->local_gate);

mem_error:
	kfree(obj);
	kfree(handle);

exit:
	printk(KERN_ERR "nameserver_remotenotify_create failed! "
		"status = 0x%x\n", retval);
	return NULL;
}


/*
 * ======== nameserver_remotenotify_create ========
 *  Purpose:
 *  This will delete the nameserver remote transport instance
 */
int nameserver_remotenotify_delete(void **rhandle)
{
	struct nameserver_remotenotify_object *handle = NULL;
	struct nameserver_remotenotify_obj *obj = NULL;
	s32 retval = 0;
	struct mutex *gate = NULL;

	if (atomic_cmpmask_and_lt(&(nameserver_remotenotify_state.ref_count),
			NAMESERVERREMOTENOTIFY_MAKE_MAGICSTAMP(0),
			NAMESERVERREMOTENOTIFY_MAKE_MAGICSTAMP(1)) == true) {
		retval = -ENODEV;
		goto exit;
	}

	if (WARN_ON((rhandle == NULL) || (*rhandle == NULL))) {
		retval = -EINVAL;
		goto exit;
	}

	handle = (struct nameserver_remotenotify_object *)(*rhandle);
	obj = (struct nameserver_remotenotify_obj *)handle->obj;
	if (obj == NULL) {
		retval = -EINVAL;
		goto exit;
	}

	retval = mutex_lock_interruptible(obj->local_gate);
	if (retval)
		goto exit;

	retval = nameserver_unregister_remote_driver(obj->remote_proc_id);
	/* Do we have to bug_on/warn_on oops here intead of exit ?*/
	if (retval < 0)
		goto exit;

	kfree(obj->sem_handle);
	obj->sem_handle = NULL;
	/* Unregister the event from Notify */
	retval = notify_unregister_event(obj->params.notify_driver,
				obj->remote_proc_id,
				obj->params.notify_event_no,
				nameserver_remotenotify_callback,
				(void *)obj);
	if (retval == NOTIFY_SUCCESS)
		retval = 0;
	gate = obj->local_gate;
	kfree(obj);
	kfree(handle);
	*rhandle = NULL;
	mutex_unlock(gate);
	kfree(gate);

exit:
	if (retval < 0) {
		printk(KERN_ERR "nameserver_remotenotify_delete failed! "
			"status = 0x%x\n", retval);
	}
	return retval;
}


/*
 * ======== nameserver_remotenotify_create ========
 *  Purpose:
 *  This will give shared memory requirements for the
 *  nameserver remote transport instance
 */
u32 nameserver_remotenotify_shared_memreq(const
			struct nameserver_remotenotify_params *params)
{
	u32 total_size;
	/* params is not used- to remove warning. */
	(void)params;

	BUG_ON(params == NULL);
	/*
	 *  The attrs takes a Ipc_cacheSize plus 2 Message structs are required.
	 *  One for sending request and one for sending response.
	 */
	total_size =	NAMESERVERREMOTENOTIFY_CACHESIZE +
			(2 * sizeof(struct nameserver_remotenotify_message));
	return total_size;
}

