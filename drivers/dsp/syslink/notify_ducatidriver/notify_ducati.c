/*
 * notify_ducati.c
 *
 * Syslink driver support functions for TI OMAP processors.
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


#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/module.h>
#include <plat/mailbox.h>

#include <syslink/notify_driver.h>
#include <syslink/notifydefs.h>
#include <syslink/notify_driverdefs.h>
#include <syslink/notify_ducatidriver.h>
#include <syslink/multiproc.h>
#include <syslink/atomic_linux.h>



#define NOTIFYSHMDRV_MEM_ALIGN	0

#define NOTIFYSHMDRV_MAX_EVENTS	32

#define NOTIFYSHMDRV_INIT_STAMP	0xA9C8B7D6

#define NOTIFYNONSHMDRV_MAX_EVENTS	1

#define NOTIFYNONSHMDRV_RESERVED_EVENTS	1

#define NOTIFYDRV_DUCATI_RECV_MBX	2

#define NOTIFYDRV_DUCATI_SEND_MBX	3

/*FIX ME: Make use of Multi Proc module */
#define SELF_ID	0

#define OTHER_ID	1

#define UP	1

#define DOWN	0

#define PROC_TESLA	0
#define PROC_DUCATI	1
#define PROC_GPP	2
#define PROCSYSM3	2
#define PROCAPPM3	3
#define MAX_SUBPROC_EVENTS	15

/*FIXME MOVE THIS TO  A SUITABLE HEADER */
#define NOTIFYDRIVERSHM_MODULEID           (u32) 0xb9d4

/* Macro to make a correct module magic number with refCount */
#define NOTIFYDRIVERSHM_MAKE_MAGICSTAMP(x) \
				((NOTIFYDRIVERSHM_MODULEID << 12u) | (x))

static struct omap_mbox *ducati_mbox;
static void notify_ducatidrv_isr(void *ntfy_msg);
static void notify_ducatidrv_isr_callback(void *ref_data, void* ntfy_msg);

struct notify_driver_object_list {
	struct list_head elem;
	struct notify_driver_object *drv_handle;
};


/*
 *	brief	Notify ducati driver instance object.
 */
struct notify_ducatidrv_object {
	struct notify_ducatidrv_params params;
	short int  proc_id;
	struct notify_drv_eventlist *event_list;
	VOLATILE struct notify_shmdrv_ctrl *ctrl_ptr;
	struct notify_shmdrv_eventreg *reg_chart;
	struct notify_driver_object *drv_handle;
	short int  self_id;
	short int other_id;
};


/*
 *	brief	Defines the notify_ducatidrv state object, which contains all
 * 		the module specific information.
 */
struct notify_ducatidrv_module {
	atomic_t ref_count;
	struct notify_ducatidrv_config cfg;
	struct notify_ducatidrv_config def_cfg;
	struct notify_ducatidrv_params def_inst_params;
	struct mutex *gate_handle;
	struct list_head drv_handle_list;
} ;


static struct notify_ducatidrv_module notify_ducatidriver_state = {
	.gate_handle = NULL,
	.def_inst_params.shared_addr = 0x0,
	.def_inst_params.shared_addr_size = 0x0,
	.def_inst_params.num_events = NOTIFYSHMDRV_MAX_EVENTS,
	.def_inst_params.num_reserved_events = 3,
	.def_inst_params.send_event_poll_count = (int) -1,
	.def_inst_params.remote_proc_id = -1,
	.def_inst_params.recv_int_id = (int) -1,
	.def_inst_params.send_int_id = (int) -1
};

/*
 *	This function searchs for a element the List.
 */
static void notify_ducatidrv_qsearch_elem(struct list_head *list,
		struct notify_drv_eventlistner *check_obj,
		struct notify_drv_eventlistner **listener);


/*
 *	brief	Get the default configuration for the notify_ducatidrv module.
 *
 *	This function can be called by the application to get their
 *	configuration parameter to notify_ducatidrv_setup filled in by
 *	the notify_ducatidrv module with the default parameters. If the
 *	user does not wish to make any change in the default parameters,
 *	this API is not required to be called.
 *
 */
void notify_ducatidrv_getconfig(struct notify_ducatidrv_config *cfg)
{
	BUG_ON(cfg == NULL);

	if (atomic_cmpmask_and_lt(&(notify_ducatidriver_state.ref_count),
					NOTIFYDRIVERSHM_MAKE_MAGICSTAMP(0),
					NOTIFYDRIVERSHM_MAKE_MAGICSTAMP(1))
					== true)
		memcpy(cfg,
			&(notify_ducatidriver_state.def_cfg),
			sizeof(struct notify_ducatidrv_config));
	else
		memcpy(cfg,
			&(notify_ducatidriver_state.cfg),
			sizeof(struct notify_ducatidrv_config));
}
EXPORT_SYMBOL(notify_ducatidrv_getconfig);

/*
 *	brief	Function to open a handle to an existing notify_ducatidrv_object
 *		handling the procId.
 *
 *	This function returns a handle to an existing notify_ducatidrv
 *	instance created for this procId. It enables other entities to
 *	access and use this notify_ducatidrv instance.
 */
int notify_ducatidrv_open(char *driver_name,
			  struct notify_driver_object **handle_ptr)
{
	int status = 0;
	BUG_ON(driver_name == NULL);
	BUG_ON(handle_ptr == NULL);
	/* Enter critical section protection. */
	WARN_ON(mutex_lock_interruptible(notify_ducatidriver_state.
			gate_handle) != 0);
	/* Get the handle from Notify module. */
	status = notify_get_driver_handle(driver_name, handle_ptr);
	WARN_ON(status < 0);
	mutex_unlock(notify_ducatidriver_state.gate_handle);
	return status;
}




/*
 *	brief	Function to close this handle to the notify_ducatidrv instance.
 *
 *	This function closes the handle to the notify_ducatidrv instance
 *	obtained through notify_ducatidrv_open call made earlier.
 */
int notify_ducatidrv_close(struct notify_driver_object **handle_ptr)
{
	int status = 0;
	BUG_ON(handle_ptr == NULL);
	BUG_ON(*handle_ptr == NULL);
	*handle_ptr = NULL;
	return status;
}




/*
 *	brief	Function to initialize the parameters for this notify_ducatidrv
 *		instance.
 */
void notify_ducatidrv_params_init(struct notify_driver_object *handle,
				struct notify_ducatidrv_params *params)
{
	struct notify_ducatidrv_object *driver_obj;
	BUG_ON(params == NULL);

	if (atomic_cmpmask_and_lt(&(notify_ducatidriver_state.ref_count),
					NOTIFYDRIVERSHM_MAKE_MAGICSTAMP(0),
					NOTIFYDRIVERSHM_MAKE_MAGICSTAMP(1))
								== true) {
		/*FIXME not intialized to be returned */
		BUG_ON(1);
	}

	if (handle == NULL) {
		memcpy(params,
		&(notify_ducatidriver_state.def_inst_params),
		sizeof(struct notify_ducatidrv_params));
	} else {
		/*Return updated notify_ducatidrv instance specific parameters*/
		driver_obj = (struct notify_ducatidrv_object *)
				handle->driver_object;
		memcpy(params, &(driver_obj->params),
			sizeof(struct notify_ducatidrv_params));
	}
}
EXPORT_SYMBOL(notify_ducatidrv_params_init);



/*
 *	brief	Function to create an instance of this Notify ducati driver.
 *
 */
struct notify_driver_object *notify_ducatidrv_create(char *driver_name,
		const struct notify_ducatidrv_params *params)
{

	int status = 0;
	struct notify_ducatidrv_object *driver_obj = NULL;
	struct notify_driver_object *drv_handle = NULL;
	struct notify_drv_eventlist *event_list = NULL;
	VOLATILE struct notify_shmdrv_proc_ctrl *ctrl_ptr = NULL;
	struct notify_driver_attrs drv_attrs;
	struct notify_interface fxn_table;
	struct notify_driver_object_list *drv_handle_inst = NULL;
	int proc_id;
	int i;
	u32 shm_va;
	int tmp_status = NOTIFY_SUCCESS;

	BUG_ON(driver_name == NULL);
	BUG_ON(params == NULL);

	if (atomic_cmpmask_and_lt(&(notify_ducatidriver_state.ref_count),
					NOTIFYDRIVERSHM_MAKE_MAGICSTAMP(0),
					NOTIFYDRIVERSHM_MAKE_MAGICSTAMP(1))
					== true) {
		printk(KERN_ERR "Module not initialized\n");
		goto func_end;
	}

	if (params->num_events > NOTIFYSHMDRV_MAX_EVENTS) {
		printk(KERN_ERR "More than max number of events passed\n");
		goto func_end;
	}
	WARN_ON(mutex_lock_interruptible(notify_ducatidriver_state.
				gate_handle) != 0);
	proc_id = PROC_DUCATI;

	tmp_status = notify_get_driver_handle(driver_name, &drv_handle);

	if (tmp_status != NOTIFY_E_NOTFOUND) {
		printk(KERN_ERR "Driver handle not found\n");
		goto error_unlock_and_return;
	}

	/* Fill in information about driver attributes. */
	/* This driver supports interaction with one other remote
	* processor.*/

	for (i = 0 ; i < MULTIPROC_MAXPROCESSORS; i++) {
		/* Initialize all to invalid. */
		drv_attrs.proc_info[i].proc_id = (u16)0xFFFF;
	}

	/*FIXME: Hack to allow SYSM3 and APPM3 events. Re-visit later */
	if (params->remote_proc_id >= PROCSYSM3) {
		for (i = PROCSYSM3; i <= PROCAPPM3; i++) {
			drv_attrs.numProc = 1;
			drv_attrs.proc_info[i].max_events = params->num_events;
			drv_attrs.proc_info[i].reserved_events =
				params->num_reserved_events;
			/* Events are prioritized. */
			drv_attrs.proc_info[i].event_priority = true;
			/* 32-bit payload supported. */
			drv_attrs.proc_info[i].payload_size = sizeof(int);
			drv_attrs.proc_info[i].proc_id = i;
		}
	}

	/* Function table information */
	fxn_table.register_event = (void *)&notify_ducatidrv_register_event;
	fxn_table.unregister_event = (void *)&notify_ducatidrv_unregister_event;
	fxn_table.send_event = (void *)&notify_ducatidrv_sendevent;
	fxn_table.disable = (void *)&notify_ducatidrv_disable;
	fxn_table.restore = (void *)&notify_ducatidrv_restore;
	fxn_table.disable_event = (void *)&notify_ducatidrv_disable_event;
	fxn_table.enable_event = (void *)&notify_ducatidrv_enable_event;

	/* Register driver with the Notify module. */
	status = notify_register_driver(driver_name,
				&fxn_table,
				&drv_attrs,
				&drv_handle);
	/*FIXME: To take care of already exists case */
	if ((status != NOTIFY_SUCCESS) && (status != NOTIFY_E_ALREADYEXISTS)) {
		printk(KERN_ERR "Notify register failed\n");
		goto error_clean_and_exit;
	}
	/* Allocate memory for the notify_ducatidrv_object object. */
	drv_handle->driver_object = driver_obj =
			kmalloc(sizeof(struct notify_ducatidrv_object),
				GFP_ATOMIC);

	if (driver_obj == NULL) {
		status = -ENOMEM;
		goto error_clean_and_exit;
	} else {
		memcpy(&(driver_obj->params), (void *) params,
		sizeof(struct notify_ducatidrv_params));
	}

	if (params->remote_proc_id > multiproc_get_id(NULL)) {
		driver_obj->self_id  = SELF_ID;
		driver_obj->other_id = OTHER_ID;
	} else {
		driver_obj->self_id  = OTHER_ID;
		driver_obj->other_id = SELF_ID;
	}

	shm_va = get_ducati_virt_mem();
	driver_obj->ctrl_ptr = (struct notify_shmdrv_ctrl *) shm_va;
	ctrl_ptr = &(driver_obj->ctrl_ptr->
		proc_ctrl[driver_obj->self_id]);
	ctrl_ptr->self_event_chart =
				(struct notify_shmdrv_event_entry *)
				((int)(driver_obj->ctrl_ptr)
				+ sizeof(struct notify_shmdrv_ctrl)+
				(sizeof(struct
					notify_shmdrv_event_entry)
					* params->num_events
					* driver_obj->other_id));

	ctrl_ptr->other_event_chart =
				(struct notify_shmdrv_event_entry *)
				((int)(driver_obj->ctrl_ptr)
				+ sizeof(struct notify_shmdrv_ctrl) +
				(sizeof(struct
					notify_shmdrv_event_entry)
					* params->num_events
					* driver_obj->self_id));
	driver_obj->proc_id = params->remote_proc_id;
	driver_obj->event_list = kmalloc(
				(sizeof(struct notify_drv_eventlist)
				* params->num_events), GFP_ATOMIC);
	if (driver_obj->event_list == NULL) {
		status = -ENOMEM;
		goto error_clean_and_exit;
	} else {
		memset(driver_obj->event_list, 0,
		sizeof(struct notify_drv_eventlist)*params->
						num_events);
	}

	driver_obj->reg_chart = kmalloc(sizeof(
					struct notify_shmdrv_eventreg)
					*params->num_events,
					GFP_ATOMIC);
	if (driver_obj->reg_chart == NULL) {
		status = -ENOMEM;
		goto error_clean_and_exit;
	} else {
		memset(driver_obj->reg_chart, 0,
			sizeof(struct notify_shmdrv_eventreg)
			*params->num_events);
	}

	event_list = driver_obj->event_list;

	for (i = 0 ; (i < params->num_events) ; i++) {
		ctrl_ptr->self_event_chart[i].flag = 0;
		driver_obj->reg_chart[i].reg_event_no = (int) -1;
		event_list[i].event_handler_count = 0;
		INIT_LIST_HEAD(&event_list[i].listeners);
	}

	/*Set up the ISR on the Modena-ducati FIFO */
	/* Add the driver handle to list */
	drv_handle_inst = kmalloc(sizeof
		(struct notify_driver_object_list), GFP_ATOMIC);
	if (drv_handle_inst == NULL) {
		status = -ENOMEM;
		goto error_clean_and_exit;
	}

	drv_handle_inst->drv_handle = drv_handle;
	list_add_tail(&(drv_handle_inst->elem),
		&(notify_ducatidriver_state.drv_handle_list));

	driver_obj = drv_handle->driver_object;
	ctrl_ptr->reg_mask.mask = 0x0;
	ctrl_ptr->reg_mask.enable_mask = 0xFFFFFFFF;
	ctrl_ptr->recv_init_status = NOTIFYSHMDRV_INIT_STAMP;
	ctrl_ptr->send_init_status = NOTIFYSHMDRV_INIT_STAMP;
	drv_handle->is_init = NOTIFY_DRIVERINITSTATUS_DONE;
	mutex_unlock(notify_ducatidriver_state.gate_handle);
	omap_mbox_enable_irq(ducati_mbox, IRQ_RX);

	/* Done with initialization. goto function end */
	goto func_end;

error_clean_and_exit:
	if (drv_handle != NULL) {
		/* Unregister driver from the Notify module*/
		notify_unregister_driver(drv_handle);
		if (ctrl_ptr != NULL) {
			/* Clear initialization status in
			shared memory. */
			ctrl_ptr->recv_init_status = 0x0;
			ctrl_ptr->send_init_status = 0x0;
			ctrl_ptr = NULL;
		}
		/* Check if driverObj was allocated. */
		if (driver_obj != NULL) {
			/* Check if event List was allocated. */
			if (driver_obj->event_list != NULL) {
				/* Check if lists were
				created. */
				for (i = 0 ;
				i < params->num_events ; i++) {
					list_del(
					(struct list_head *)
					&driver_obj->
					event_list[i].
					listeners);
				}
				kfree(driver_obj->event_list);
				driver_obj->event_list = NULL;
			}
			/* Check if regChart was allocated. */
			if (driver_obj->reg_chart != NULL) {
				kfree(driver_obj->reg_chart);
					driver_obj->reg_chart
						= NULL;
			}
			kfree(driver_obj);
		}
		drv_handle->is_init =
			NOTIFY_DRIVERINITSTATUS_NOTDONE;
		drv_handle = NULL;
	}

error_unlock_and_return:
	/* Leave critical section protection. */
	mutex_unlock(notify_ducatidriver_state.gate_handle);
func_end:
	return drv_handle;
}
EXPORT_SYMBOL(notify_ducatidrv_create);

/*
 *	brief	Function to delete the instance of shared memory driver
 *
 */
int notify_ducatidrv_delete(struct notify_driver_object **handle_ptr)
{
	int status    = 0;
	struct notify_driver_object  *drv_handle = NULL;
	struct notify_ducatidrv_object *driver_obj = NULL;
	struct notify_drv_eventlist *event_list;
	short int i = 0;
	struct list_head *elem = NULL;
	struct notify_driver_object_list *drv_list_entry  = NULL;
	int proc_id;

	if (atomic_cmpmask_and_lt(&(notify_ducatidriver_state.ref_count),
					NOTIFYDRIVERSHM_MAKE_MAGICSTAMP(0),
					NOTIFYDRIVERSHM_MAKE_MAGICSTAMP(1))
					== true) {
		/*FIXME not intialized to be returned */
		return -1;
	}

	WARN_ON(handle_ptr == NULL);
	if (handle_ptr == NULL)
		return -1;

	drv_handle = (*handle_ptr);
	WARN_ON(drv_handle == NULL);
	if (drv_handle == NULL)
		return -1;

	driver_obj = (struct notify_ducatidrv_object *)
			(*handle_ptr)->driver_object;
	WARN_ON((*handle_ptr)->driver_object == NULL);

	/*Uninstall the ISRs & Disable the Mailbox interrupt.*/

	if (drv_handle != NULL) {
		list_for_each(elem,
			&notify_ducatidriver_state.drv_handle_list) {
			drv_list_entry = container_of(elem,
				struct notify_driver_object_list, elem);
			if (drv_list_entry->drv_handle == drv_handle) {
				list_del(elem);
				kfree(drv_list_entry);
				status = notify_unregister_driver(drv_handle);
				drv_handle = NULL;
				break;
			}
		}
	}

	if (status != NOTIFY_SUCCESS)
		printk(KERN_WARNING "driver is not registerd\n");

	if (driver_obj != NULL) {
		if (driver_obj->ctrl_ptr != NULL) {
			/* Clear initialization status in shared memory. */
			driver_obj->ctrl_ptr->proc_ctrl[driver_obj->self_id].
				recv_init_status = 0x0;
			driver_obj->ctrl_ptr->proc_ctrl[driver_obj->self_id].
				send_init_status = 0x0;
			unmap_ducati_virt_mem((u32)(driver_obj->ctrl_ptr));
			driver_obj->ctrl_ptr = NULL;
		}

		event_list = driver_obj->event_list;
		if (event_list != NULL) {
			/* Check if lists were created. */
			for (i = 0 ; i < driver_obj->params.num_events ; i++) {
				WARN_ON(event_list[i].event_handler_count != 0);
				event_list[i].event_handler_count = 0;
				list_del((struct list_head *)
					&event_list[i].listeners);
			}

			kfree(event_list);
			driver_obj->event_list = NULL;
		}

		/* Check if regChart was allocated. */
		if (driver_obj->reg_chart != NULL) {
			kfree(driver_obj->reg_chart);
			driver_obj->reg_chart = NULL;
		}

		/* Disable the interrupt, Uninstall the ISR and delete it. */
		/* Check if ISR was created. */
		/*Remove the ISR on the Modena-ducati FIFO */
		proc_id = PROC_DUCATI;

		omap_mbox_disable_irq(ducati_mbox, IRQ_RX);

		kfree(driver_obj);
		driver_obj = NULL;
	}
	return status;
}
EXPORT_SYMBOL(notify_ducatidrv_delete);


/*
 *	brief	Destroy the notify_ducatidrv module.
 *
 */
int notify_ducatidrv_destroy(void)
{
	int status = 0;
	struct list_head *handle_list = NULL;
	struct notify_driver_object_list *entry_list = NULL;
	struct notify_driver_object *drv_handle = NULL;
	struct list_head *entry;

	if (atomic_cmpmask_and_lt(&(notify_ducatidriver_state.ref_count),
					NOTIFYDRIVERSHM_MAKE_MAGICSTAMP(0),
					NOTIFYDRIVERSHM_MAKE_MAGICSTAMP(1))
								== true)
		/* FIXME Invalid state to be reuurned. */
		return -1;

	if (atomic_dec_return(&(notify_ducatidriver_state.ref_count)) ==
					NOTIFYDRIVERSHM_MAKE_MAGICSTAMP(0)) {

		/* Temprarily increment the refcount */
		atomic_set(&(notify_ducatidriver_state.ref_count),
			NOTIFYDRIVERSHM_MAKE_MAGICSTAMP(1));
		handle_list = &(notify_ducatidriver_state.drv_handle_list);

		list_for_each(entry, handle_list) {
			entry_list = (struct notify_driver_object_list *)
				container_of(entry,
				struct notify_driver_object_list, elem);
			drv_handle = entry_list->drv_handle;
			notify_ducatidrv_delete(&drv_handle);
		}

		/* Check if the gate_handle was created internally. */

		if (notify_ducatidriver_state.gate_handle != NULL)
			kfree(notify_ducatidriver_state.gate_handle);

		atomic_set(&(notify_ducatidriver_state.ref_count),
			NOTIFYDRIVERSHM_MAKE_MAGICSTAMP(0));

		omap_mbox_put(ducati_mbox);
		ducati_mbox = NULL;
	}


	return status;
}
EXPORT_SYMBOL(notify_ducatidrv_destroy);



/*
 *	brief	Setup the notify_ducatidrv module.
 *
 *	This function sets up the notify_ducatidrv module. This function
 *	must be called before any other instance-level APIs can be
 *	invoked.
 *	Module-level configuration needs to be provided to this
 *	function. If the user wishes to change some specific config
 *	parameters, then notify_ducatidrv_getconfig can be called to get
 *	the configuration filled with the default values. After this,
 *	only the required configuration values can be changed. If the
 *	user does not wish to make any change in the default parameters,
 *	the application can simply call notify_ducatidrv_setup with NULL
 *	parameters. The default parameters would get automatically used.
 */
int notify_ducatidrv_setup(struct notify_ducatidrv_config *cfg)
{
	int status = 0;
	struct notify_ducatidrv_config tmp_cfg;

	if (cfg == NULL) {
		notify_ducatidrv_getconfig(&tmp_cfg);
		cfg = &tmp_cfg;
	}

	/* Init the ref_count to 0 */
	atomic_cmpmask_and_set(&(notify_ducatidriver_state.ref_count),
					NOTIFYDRIVERSHM_MAKE_MAGICSTAMP(0),
					NOTIFYDRIVERSHM_MAKE_MAGICSTAMP(0));
	if (atomic_inc_return(&(notify_ducatidriver_state.ref_count)) !=
		NOTIFYDRIVERSHM_MAKE_MAGICSTAMP(1u)) {
		return 1;
	}


	/* Create a default gate handle here */
	notify_ducatidriver_state.gate_handle =
			kmalloc(sizeof(struct mutex), GFP_KERNEL);
	mutex_init(notify_ducatidriver_state.gate_handle);

	if (notify_ducatidriver_state.gate_handle == NULL) {
		atomic_set(&(notify_ducatidriver_state.ref_count),
				NOTIFYDRIVERSHM_MAKE_MAGICSTAMP(0));
		status = -ENOMEM;
		goto error_exit;
	} else {
		memcpy(&notify_ducatidriver_state.cfg,
		cfg, sizeof(struct notify_ducatidrv_config));
	}

	INIT_LIST_HEAD(&(notify_ducatidriver_state.
		drv_handle_list));
	/* Initialize the maibox modulde for ducati */
	if (ducati_mbox == NULL) {
		ducati_mbox = omap_mbox_get("mailbox-2");
		if (ducati_mbox == NULL) {
			printk(KERN_ERR "Failed in omap_mbox_get()\n");
			status = -ENODEV;
			goto error_mailbox_get_failed;
		}
		ducati_mbox->rxq->callback =
				(int (*)(void *))notify_ducatidrv_isr;
	}

	return 0;

error_mailbox_get_failed:
	kfree(notify_ducatidriver_state.gate_handle);
error_exit:
	return status;
}
EXPORT_SYMBOL(notify_ducatidrv_setup);


/*
*	brief	register a callback for an event with the Notify driver.
*
*/
int notify_ducatidrv_register_event(
	struct notify_driver_object *handle,
	short int  proc_id,
	int  event_no,
	fn_notify_cbck     fn_notify_cbck,
	void *cbck_arg)
{
	int status = 0;
	int first_reg = false;
	bool done = true;
	struct notify_drv_eventlistner *event_listener;
	struct notify_drv_eventlist *event_list;
	struct notify_ducatidrv_object *driver_object;
	struct notify_shmdrv_eventreg *reg_chart;
	VOLATILE struct notify_shmdrv_ctrl *ctrl_ptr;
	VOLATILE struct notify_shmdrv_event_entry *self_event_chart;
	int i;
	int j;
	BUG_ON(handle == NULL);
	BUG_ON(handle->is_init != NOTIFY_DRIVERINITSTATUS_DONE);
	BUG_ON(handle->driver_object == NULL);
	BUG_ON(fn_notify_cbck == NULL);


	driver_object = (struct notify_ducatidrv_object *)
				handle->driver_object;

	ctrl_ptr = driver_object->ctrl_ptr;
	/* Allocate memory for event listener. */
	event_listener = kmalloc(sizeof(struct notify_drv_eventlistner),
					GFP_ATOMIC);

	if (event_listener == NULL) {
		status = -ENOMEM;
		goto func_end;
	} else {
		memset(event_listener, 0,
			sizeof(struct notify_drv_eventlistner));
	}

	if (mutex_lock_interruptible(notify_ducatidriver_state.gate_handle)
					!= 0)
		WARN_ON(1);

	event_list = driver_object->event_list;
	WARN_ON(event_list == NULL);
	event_listener->fn_notify_cbck = fn_notify_cbck;
	event_listener->cbck_arg = cbck_arg;
	/* Check if this is the first registration for this event. */

	if (list_empty((struct list_head *)
			&event_list[event_no].listeners)) {
		first_reg = true;
		self_event_chart = ctrl_ptr->proc_ctrl[driver_object->self_id].
				 self_event_chart;
		/* Clear any pending unserviced event as there are no listeners
		* for the pending event
		*/
		self_event_chart[event_no].flag = DOWN;
	}

	list_add_tail((struct list_head *)
		&(event_listener->element),
		(struct list_head *)
		&event_list[event_no].listeners);
	event_list[event_no].event_handler_count++;


	if (first_reg == true) {
		reg_chart = driver_object->reg_chart;
		for (i = 0 ; i < driver_object->params.num_events ; i++) {
			/* Find the correct slot in the registration array. */
			if (reg_chart[i].reg_event_no == (int) -1) {
				for (j = (i - 1); j >= 0; j--) {
					if (event_no < reg_chart[j].
						reg_event_no) {
						reg_chart[j + 1].reg_event_no =
							reg_chart[j].
							reg_event_no;
						reg_chart[j + 1].reserved =
							reg_chart[j].reserved;
						i = j;
					} else {
						/* End the loop, slot found. */
						j = -1;
					}
				}
				reg_chart[i].reg_event_no = event_no;
				done = true;
				break;
			}
		}

		if (done) {
			set_bit(event_no, (unsigned long *)
				&ctrl_ptr->proc_ctrl[driver_object->self_id].
				reg_mask.mask);
		} else {
			/*retval NOTIFY_E_MAXEVENTS Maximum number of
			supported events have already been registered. */
			status = -EINVAL;
			kfree(event_listener);

		}
	}
func_end:
	mutex_unlock(notify_ducatidriver_state.gate_handle);
	return status;
}


/*
*
*	brief	Unregister a callback for an event with the Notify driver.
*
*/

int notify_ducatidrv_unregister_event(
	struct notify_driver_object *handle,
	short int  proc_id,
	int  event_no,
	fn_notify_cbck	fn_notify_cbck,
	void *cbck_arg)
{
	int status = 0;
	struct notify_drv_eventlistner *listener  = NULL;
	int num_events;
	struct notify_ducatidrv_object *driver_object;
	struct notify_drv_eventlist *event_list;
	struct notify_shmdrv_eventreg *reg_chart;
	VOLATILE struct notify_shmdrv_ctrl *ctrl_ptr = NULL;
	struct notify_drv_eventlistner   unreg_info;
	VOLATILE struct notify_shmdrv_event_entry *self_event_chart;
	int i;
	int j;

	BUG_ON(fn_notify_cbck ==  NULL);
	BUG_ON(handle == NULL);
	BUG_ON(handle->driver_object == NULL);

	driver_object = (struct notify_ducatidrv_object *)
				handle->driver_object;
	num_events = driver_object->params.num_events;

	ctrl_ptr = driver_object->ctrl_ptr;

	/* Enter critical section protection. */
	if (mutex_lock_interruptible(notify_ducatidriver_state.gate_handle)
						!= 0)
		WARN_ON(1);

	event_list = driver_object->event_list;


	unreg_info.fn_notify_cbck = fn_notify_cbck;
	unreg_info.cbck_arg = cbck_arg;
	notify_ducatidrv_qsearch_elem(&event_list[event_no].listeners,
				&unreg_info,
				&listener);
	if (listener == NULL) {
		status = -EFAULT;
		goto func_end;
	}
	list_del((struct list_head *)&(listener->element));
	kfree(listener);
	event_list[event_no].event_handler_count--;


	if (list_empty((struct list_head *)
			&event_list[event_no].listeners) == true) {
		clear_bit(event_no, (unsigned long *)
			&ctrl_ptr->proc_ctrl[driver_object->self_id].reg_mask.
			  mask);
		self_event_chart = ctrl_ptr->proc_ctrl[driver_object->self_id].
				   self_event_chart;
		/* Clear any pending unserviced event as there are no
		* listeners for the pending event
		*/
		self_event_chart[event_no].flag = DOWN;
		reg_chart = driver_object->reg_chart;
		for (i = 0; i < num_events; i++) {
			/* Find the correct slot in the registration array. */
			if (event_no == reg_chart[i].reg_event_no) {
				reg_chart[i].reg_event_no = (int) -1;
				for (j = (i + 1);
					(reg_chart[j].reg_event_no != (int) -1)
						&& (j != num_events); j++) {
					reg_chart[j - 1].reg_event_no =
						reg_chart[j].reg_event_no;
					reg_chart[j - 1].reserved =
						reg_chart[j].reserved;
				}

				if (j == num_events) {
					reg_chart[j - 1].reg_event_no =
								(int) -1;
				}
				break;
			}
		}
	}



func_end:
	mutex_unlock(notify_ducatidriver_state.gate_handle);
	return status;
}

/*
*	brief	Send a notification event to the registered users for this
*		notification on the specified processor.
*
*/
int notify_ducatidrv_sendevent(struct notify_driver_object *handle,
		short int proc_id, int event_no,
		int payload, short int wait_clear)
{
	int status = 0;
	struct notify_ducatidrv_object *driver_object;
	VOLATILE struct notify_shmdrv_ctrl *ctrl_ptr;
	int max_poll_count;
	VOLATILE struct notify_shmdrv_event_entry *other_event_chart;

	int i = 0;

	BUG_ON(handle ==  NULL);
	BUG_ON(handle->driver_object == NULL);

	dsb();
	driver_object = (struct notify_ducatidrv_object *)
				handle->driver_object;

	BUG_ON(event_no > driver_object->params.num_events);

	ctrl_ptr = driver_object->ctrl_ptr;
	other_event_chart = ctrl_ptr->proc_ctrl[driver_object->self_id].
							other_event_chart;
	max_poll_count = driver_object->params.send_event_poll_count;

	/* Check whether driver supports interrupts from this processor to the
	other processor, and if it is initialized
	*/
	if (ctrl_ptr->proc_ctrl[driver_object->other_id].recv_init_status
		!= NOTIFYSHMDRV_INIT_STAMP) {
		status = -ENODEV;
		/* This may be used for polling till other-side
		driver is ready, sodo not set failure reason.
		*/
	} else {
		/* Check if other side is ready to receive this event. */
		if ((test_bit(event_no, (unsigned long *)
			&ctrl_ptr->proc_ctrl[driver_object->other_id].
			reg_mask.mask) != 1)
			|| (test_bit(event_no, &ctrl_ptr->
				proc_ctrl[driver_object->other_id].reg_mask.
				enable_mask) != 1)) {
			status = -ENODEV;
			printk(KERN_ERR "NOTIFY DRV: OTHER SIDE NOT READY TO"
				"RECEIVE. %d\n", event_no);
			/* This may be used for polling till other-side
			is ready, so do not set failure reason.*/
		} else {
			dsb();
			/* Enter critical section protection. */
			if (mutex_lock_interruptible(notify_ducatidriver_state.
							gate_handle) != 0)
				WARN_ON(1);
			if (wait_clear == true) {
				/*Wait for completion of prev
				event from other side*/
				while ((other_event_chart[event_no].flag
					!= DOWN)
					&& status == 0) {
					/* Leave critical section protection
					Create a window of opportunity
					for other interrupts to be handled.
					*/
					mutex_unlock(notify_ducatidriver_state.
							gate_handle);
					i++;
					if ((max_poll_count != (int) -1)
					&&	(i == max_poll_count)) {
						status = -EBUSY;
					}
					/* Enter critical section protection. */
					if (mutex_lock_interruptible(
						notify_ducatidriver_state.
						gate_handle) != 0)
						WARN_ON(1);
				}
			}
			if (status >= 0) {
				/* Set the event bit field and payload. */
				other_event_chart[event_no].payload = payload;
				other_event_chart[event_no].flag = UP;
				/* Send an interrupt with the event
				information to theremote processor */
				status = omap_mbox_msg_send(ducati_mbox,
								payload);
			}
			/* Leave critical section protection. */
			mutex_unlock(notify_ducatidriver_state.gate_handle);
		}
	}

	return status;
}

/*
*	brief	Disable all events for this Notify driver.
*
*/
void *notify_ducatidrv_disable(struct notify_driver_object *handle)
{

	omap_mbox_disable_irq(ducati_mbox, IRQ_RX);

	return NULL; /*No flags to be returned. */
}

/*
*	brief	Restore the notify_ducatidrv to the state before the
*		last disable was called.
*
*/
int notify_ducatidrv_restore(struct notify_driver_object *handle,
		void *flags)
{
	(void) handle;
	(void) flags;
	/*Enable the receive interrupt for ducati */
	omap_mbox_enable_irq(ducati_mbox, IRQ_RX);
	return 0;
}

/*
*	brief	Disable a specific event for this Notify ducati driver
*
*/
int notify_ducatidrv_disable_event(
	struct notify_driver_object *handle,
	short int  proc_id, int  event_no)
{
	static int access_count ;
	signed long int status = 0;
	struct notify_ducatidrv_object *driver_object;
	BUG_ON(handle == NULL);
	BUG_ON(handle->driver_object == NULL);
	access_count++;
	driver_object = (struct notify_ducatidrv_object *)
				handle->driver_object;
	/* Enter critical section protection. */

	if (mutex_lock_interruptible(notify_ducatidriver_state.
					gate_handle) != 0)
		WARN_ON(1);

	clear_bit(event_no, (unsigned long *)
		&driver_object->ctrl_ptr->proc_ctrl[driver_object->self_id].
			reg_mask.enable_mask);
	/* Leave critical section protection. */
	mutex_unlock(notify_ducatidriver_state.gate_handle);
	return status;
}

/*
*	brief	Enable a specific event for this Notify ducati driver
*
*/
int notify_ducatidrv_enable_event(struct notify_driver_object *handle,
		short int proc_id, int event_no)
{
	int status = 0;
	struct notify_ducatidrv_object *driver_object;
	BUG_ON(handle == NULL);
	BUG_ON(handle->driver_object == NULL);

	driver_object = (struct notify_ducatidrv_object *)
				handle->driver_object;
	/* Enter critical section protection. */

	if (mutex_lock_interruptible(notify_ducatidriver_state.
					gate_handle) != 0)
		WARN_ON(1);

	set_bit(event_no, (unsigned long *)
		&driver_object->ctrl_ptr->proc_ctrl[driver_object->self_id].
		reg_mask.enable_mask);

	mutex_unlock(notify_ducatidriver_state.gate_handle);
	return status;
}

/*
*	brief	Print debug information for the Notify ducati driver
*
*/
int notify_ducatidrv_debug(struct notify_driver_object *handle)
{
	int status = 0;
	printk(KERN_WARNING "ducati Debug: Nothing being printed currently\n");
	return status;
}


/*
 *
 *	brief	This function implements the interrupt service routine for the
 *		interrupt received from the Ducati processor.
 *
 */
static void notify_ducatidrv_isr(void *ntfy_msg)
{
	struct notify_driver_object_list *obj_list;
	struct list_head *entry = NULL;
	list_for_each(entry, &(notify_ducatidriver_state.drv_handle_list)) {
		obj_list = (struct notify_driver_object_list *)
				container_of(entry,
				struct notify_driver_object_list, elem);
		notify_ducatidrv_isr_callback(
			obj_list->drv_handle->driver_object, ntfy_msg);
	}
}
EXPORT_SYMBOL(notify_ducatidrv_isr);


static void notify_ducatidrv_isr_callback(void *ref_data, void* ntfy_msg)
{
	int payload   = 0;
	int i = 0;
	struct list_head *temp;
	int j = 0;
	VOLATILE struct notify_shmdrv_event_entry  *self_event_chart;
	struct notify_ducatidrv_object *driver_obj;
	struct notify_shmdrv_eventreg *reg_chart;
	VOLATILE struct notify_shmdrv_proc_ctrl *proc_ctrl_ptr;
	int event_no;
	dsb();
	/* Enter critical section protection. */

	driver_obj = (struct notify_ducatidrv_object *) ref_data;
	proc_ctrl_ptr = &(driver_obj->ctrl_ptr->proc_ctrl[driver_obj->self_id]);
	reg_chart = driver_obj->reg_chart;
	self_event_chart = proc_ctrl_ptr->self_event_chart;
	dsb();
	/* Execute the loop till no asserted event
	is found for one complete loop
	through all registered events
	*/
	do {
		/* Check if the entry is a valid registered event.*/
		event_no = reg_chart[i].reg_event_no;
		/* Determine the current high priority event.*/
		/* Check if the event is set and enabled.*/
		if (self_event_chart[event_no].flag == UP &&
			test_bit(event_no,
			(unsigned long *) &proc_ctrl_ptr->reg_mask.enable_mask)
			&& (event_no != (int) -1)) {

			payload = self_event_chart[event_no].
						payload;
			dsb();
			/* Acknowledge the event. */
			payload = (int)ntfy_msg;
			self_event_chart[event_no].flag = DOWN;
			dsb();
			/*Call the callbacks associated with the event*/
			temp = driver_obj->
				event_list[event_no].
				listeners.next;
			if (temp != NULL) {
				for (j = 0; j < driver_obj->
					event_list[event_no].
					event_handler_count;
							j++) {
					/* Check for empty list. */
					if (temp == NULL)
						continue;
					/*FIXME: Hack to support SYSM3 and
						 APPM3 */
					if ((driver_obj->proc_id == PROCAPPM3)
					&& (event_no <= MAX_SUBPROC_EVENTS)) {
						driver_obj->proc_id = PROCSYSM3;
					}
					if ((driver_obj->proc_id == PROCSYSM3)
					&& (event_no > MAX_SUBPROC_EVENTS)) {
						driver_obj->proc_id = PROCAPPM3;
					}
					((struct notify_drv_eventlistner *)
						temp)->fn_notify_cbck(
					driver_obj->proc_id,
					event_no,
					((struct notify_drv_eventlistner *)
					temp)->cbck_arg,
					payload);
					temp = temp->next;
				}
				/* reinitialize the event check counter. */
				i = 0;
			}
		} else {
		/* check for next event. */
		i++;
		}
	} while ((event_no != (int) -1)
	&& (i < driver_obj->params.num_events));

}

/*
*	brief	This function searchs for a element the List.
*
*/
static void notify_ducatidrv_qsearch_elem(struct list_head *list,
		struct notify_drv_eventlistner *check_obj,
		struct notify_drv_eventlistner **listener)
{
	struct list_head    *temp = NULL ;
	struct notify_drv_eventlistner *l_temp = NULL ;
	short int found = false;

	BUG_ON(list ==  NULL);
	BUG_ON(check_obj == NULL);
	BUG_ON(listener == NULL);

	*listener = NULL;
	if ((list !=  NULL) && (check_obj != NULL)) {
		if (list_empty((struct list_head *)list) == false) {
			temp = list->next;
			while ((found == false) && (temp != NULL)) {
				l_temp =
					(struct notify_drv_eventlistner *)
					(temp);
				if ((l_temp->fn_notify_cbck ==
					check_obj->fn_notify_cbck) &&
					(l_temp->cbck_arg ==
					 check_obj->cbck_arg)) {
					found = true;
				} else
					temp = temp->next;
			}
			if (found == true)
				*listener = l_temp;
		}
	}
	return;
}
