/*
 * notify_mbxDriver.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
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

#include  <syslink/_bitops.h>
#include <syslink/notify_driver.h>
#include <syslink/notifydefs.h>
#include <syslink/notify_driverdefs.h>
#include <syslink/notify_tesladriver.h>
#include <syslink/gt.h>
#include <syslink/notify_dispatcher.h>



#define NOTIFYSHMDRV_MEM_ALIGN       0

#define NOTIFYSHMDRV_MAX_EVENTS      32

#define NOTIFYSHMDRV_INIT_STAMP      0xA9C8B7D6

#define NOTIFYNONSHMDRV_MAX_EVENTS      1

#define NOTIFYNONSHMDRV_RESERVED_EVENTS      1

#define NOTIFYDRV_TESLA_RECV_MBX     1

#define NOTIFYDRV_TESLA_SEND_MBX     0

#define SELF_ID    0

#define OTHER_ID   1

#define UP    1

#define DOWN  0

#define PROC_TESLA  0
#define PROC_DUCATI 1
#define PROC_GPP    2

irqreturn_t(*irq_handler)(int, void *, struct pt_regs *);
EXPORT_SYMBOL(irq_handler);



/*
 *  brief  NotifyDriverShm instance object.
 */
struct notify_tesladrv_object {
	struct notify_tesladrv_params params;
	short int  proc_id;
	struct notify_drv_eventlist *event_list;
	struct notify_shmdrv_ctrl *ctrl_ptr;
	struct notify_shmdrv_eventreg *reg_chart;
	struct notify_driver_object *drv_handle;
	short int  self_id;
	short int other_id;
};


/*
 *  brief   Defines the NotifyDriverShm state object, which contains all the
 *           module specific information.
 */
struct notify_tesladrv_module {
	struct notify_tesladrv_config cfg;
	struct notify_tesladrv_config def_cfg;
	struct notify_tesladrv_params def_inst_params;
	bool is_setup;
	struct mutex *gate_handle;
} ;


static struct notify_tesladrv_module notify_tesladriver_state = {
	.is_setup = false,
	.gate_handle = NULL,
	.def_cfg.gate_handle = NULL,
	.def_inst_params.shared_addr = 0x0,
	.def_inst_params.shared_addr_size = 0x0,
	.def_inst_params.num_events = NOTIFYSHMDRV_MAX_EVENTS,
	.def_inst_params.num_reserved_events = 3,
	.def_inst_params.send_event_poll_count = (int) -1,
	.def_inst_params.remote_proc_id = -1,
	.def_inst_params.recv_int_id = (int) -1,
	.def_inst_params.send_int_id = (int) -1
};



void notify_tesladrv_isr(void *ref_data);



/*
 *This function searchs for a element the List.
 */
static void notify_tesladrv_qsearch_elem(struct list_head *list,
		struct notify_drv_eventlistner *check_obj,
		struct notify_drv_eventlistner **listener);


/*!
 *  brief      Get the default configuration for the NotifyDriverShm module.
 *
 *              This function can be called by the application to get their
 *              configuration parameter to NotifyDriverShm_setup filled in by
 *              the NotifyDriverShm module with the default parameters. If the
 *              user does not wish to make any change in the default parameters,
 *              this API is not required to be called.
 */
void notify_tesladrv_getconfig(struct notify_tesladrv_config *cfg)
{
	BUG_ON(cfg == NULL);

	memcpy(cfg,
		&(notify_tesladriver_state.def_cfg),
		sizeof(struct notify_tesladrv_config));
}
EXPORT_SYMBOL(notify_tesladrv_getconfig);




/*
 *  brief      Function to open a handle to an existing NotifyDriverShm object
 *              handling the procId.
 *
 *              This function returns a handle to an existing NotifyDriverShm
 *              instance created for this procId. It enables other entities to
 *              access and use this NotifyDriverShm instance.
 *
 */
int notify_tesladrv_open(char *driver_name,
			 struct notify_driver_object **handle_ptr)
{
	int status = 0;
	BUG_ON(driver_name == NULL);
	BUG_ON(handle_ptr == NULL);
	/* Enter critical section protection. */
	WARN_ON(mutex_lock_interruptible(notify_tesladriver_state.gate_handle) != 0);
	/* Get the handle from Notify module. */
	status = notify_get_driver_handle(driver_name, handle_ptr);
	if (status < 0)
		status = -EINVAL;
	mutex_unlock(notify_tesladriver_state.gate_handle);
	return status;
}

/*
 *  brief      Function to close this handle to the NotifyDriverShm instance.
 *
 *              This function closes the handle to the NotifyDriverShm instance
 *              obtained through NotifyDriverShm_open call made earlier.
 */
int notify_tesladrv_close(struct notify_driver_object **handle_ptr)
{
	int status = 0;
	BUG_ON(handle_ptr == NULL);
	BUG_ON(*handle_ptr == NULL);
	*handle_ptr = NULL;
	return status;
}




/*
 *  brief      Function to initialize the parameters for this NotifyDriver
 *              instance.
 *
 */
void notify_tesladrv_params_init(struct notify_driver_object *handle,
				 struct notify_tesladrv_params *params)
{
	struct notify_tesladrv_object *driver_obj;
	BUG_ON(params == NULL);
	if (handle == NULL) {
		memcpy(params,
			&(notify_tesladriver_state.def_inst_params),
			sizeof(struct notify_tesladrv_params));
	} else {
		/* Return updated NotifyDriverShm instance specific parameters. */
		driver_obj = (struct notify_tesladrv_object *)
				handle->driver_object;
		memcpy(params, &(driver_obj->params),
			sizeof(struct notify_tesladrv_params));
	}
}
EXPORT_SYMBOL(notify_tesladrv_params_init);



/*
 *  brief      Function to create an instance of this NotifyDriver.
 *
 */
struct notify_driver_object *notify_tesladrv_create(char *driver_name,
		const struct notify_tesladrv_params *params) {

	int status = 0;
	struct notify_tesladrv_object *driver_obj = NULL;
	struct notify_driver_object *drv_handle = NULL;
	struct notify_drv_eventlist *event_list = NULL;
	struct notify_shmdrv_proc_ctrl *ctrl_ptr = NULL;
	struct notify_driver_attrs drv_attrs;
	struct notify_interface fxn_table;
	int proc_id;
	int i;
	unsigned long int num_events = NOTIFYNONSHMDRV_MAX_EVENTS;

	struct mbox_config *mbox_hw_config;
	int mbox_module_no;
	int interrupt_no;
	int mbx_ret_val;

	BUG_ON(driver_name == NULL);
	BUG_ON(params == NULL);
	BUG_ON(notify_tesladriver_state.is_setup == false);

	if (params->num_events > NOTIFYSHMDRV_MAX_EVENTS) {
		status = -EINVAL;
		goto func_end;
	} else {

		if (mutex_lock_interruptible(notify_tesladriver_state.
					gate_handle) != 0)
			WARN_ON(1);
		proc_id = PROC_TESLA;
		if (proc_id == PROC_TESLA) {
			printk(KERN_WARNING " --Tesla currently"
					" not using driverAttrs--\n");
		}

		drv_attrs.numProc = 1;
		drv_attrs.proc_info[PROC_TESLA].max_events = num_events;

		drv_attrs.proc_info[PROC_TESLA].reserved_events =
			params->num_reserved_events;
		/* 32-bit payload supported. */
		drv_attrs.proc_info[PROC_TESLA].payload_size = sizeof(int);
		drv_attrs.proc_info[PROC_TESLA].proc_id =
			params->remote_proc_id;

		/* Function table information */
		fxn_table.register_event = (void*)&notify_tesladrv_register_event;
		fxn_table.unregister_event = (void*)&notify_tesladrv_unregister_event;
		fxn_table.send_event = (void*)&notify_tesladrv_sendevent;
		fxn_table.disable = (void*)&notify_tesladrv_disable;
		fxn_table.restore = (void*)&notify_tesladrv_restore;
		fxn_table.disable_event = (void*)&notify_tesladrv_disable_event;
		fxn_table.enable_event = (void*)&notify_tesladrv_enable_event;

		status = notify_register_driver(driver_name,
						&fxn_table, &drv_attrs, &drv_handle);

		if (status != NOTIFY_SUCCESS) {
			/*! @retval NULL Failed to register driver with Notify module!*/
			pr_err("notify_register_driver failed and status = %d\n", status);
			status = -EINVAL;
			goto func_end;
		} else {

			/* Allocate memory for the NotifyDriverShm_Object object. */
			drv_handle->driver_object = driver_obj =
				kmalloc(sizeof(struct notify_tesladrv_object),
						GFP_ATOMIC);

			if (driver_obj == NULL) {
				status = -ENOMEM;
				goto func_end;
			} else {
				memcpy(&(driver_obj->params), (void *) params,
					sizeof(struct notify_tesladrv_params));

			}
		}

	}
	/*FIXME: Check with MultiProc_getId */
	driver_obj->self_id  = 1;
	driver_obj->other_id = 0;
	driver_obj->ctrl_ptr = (struct notify_shmdrv_ctrl *) params->shared_addr;


	ctrl_ptr = &(driver_obj->ctrl_ptr->proc_ctrl[driver_obj->self_id]);


	ctrl_ptr->self_event_chart = (struct notify_shmdrv_event_entry *)
					((int)(driver_obj->ctrl_ptr)
					+ sizeof(struct notify_shmdrv_ctrl)
					+(sizeof(struct notify_shmdrv_event_entry)
					* params->num_events
					* driver_obj->other_id));


	ctrl_ptr->other_event_chart = (struct notify_shmdrv_event_entry *)
					((int)(driver_obj->ctrl_ptr)
					+ sizeof(struct notify_shmdrv_ctrl)
					+(sizeof(struct notify_shmdrv_event_entry)
					* params->num_events
					* driver_obj->self_id));

	driver_obj->proc_id = params->remote_proc_id;


	driver_obj->event_list = kmalloc((sizeof(struct notify_drv_eventlist)
					 * params->num_events), GFP_ATOMIC);

	if (driver_obj->event_list == NULL) {

		status = -ENOMEM;
		goto func_end;
	} else {
		memset(driver_obj->event_list, 0,
			sizeof(struct notify_drv_eventlist)*params->num_events);
	}


	driver_obj->reg_chart = kmalloc(sizeof(struct notify_shmdrv_eventreg)
				       *params->num_events, GFP_ATOMIC);


	if (driver_obj->reg_chart == NULL) {

		status = -ENOMEM;
		goto func_end;
	} else {
		memset(driver_obj->reg_chart, 0, sizeof(struct notify_shmdrv_eventreg)
					*params->num_events);
	}


	event_list = driver_obj->event_list;
	for (i = 0 ; (i < params->num_events) ; i++) {

		ctrl_ptr->self_event_chart[i].flag = 0;

		driver_obj->reg_chart[i].reg_event_no = (int) -1;

		event_list[i].event_handler_count = 0;
		INIT_LIST_HEAD(&event_list[i].listeners);
	}


	ctrl_ptr->reg_mask.mask = 0x0;

	/* Enable all events initially.*/
	ctrl_ptr->reg_mask.enable_mask = 0xFFFFFFFF;


	mbox_hw_config = ntfy_disp_get_config();
	mbox_module_no = mbox_hw_config->mbox_modules;
	interrupt_no = mbox_hw_config->interrupt_lines[mbox_module_no-1];


	mbx_ret_val = ntfy_disp_bind_interrupt(interrupt_no,
			       (void *)notify_mailbx0_user0_isr, NULL);
	/*Set up the ISR on the Modena-Tesla FIFO */
	if (mbx_ret_val == 0) {
		proc_id = PROC_TESLA;
		mbx_ret_val = ntfy_disp_register(mbox_module_no,
						 (NOTIFYDRV_TESLA_RECV_MBX * 2),
						 (void *)notify_tesladrv_isr,
						 (void *)driver_obj);

		if (mbx_ret_val == 0) {

			mbx_ret_val = ntfy_disp_interrupt_enable(
			      mbox_module_no, (NOTIFYDRV_TESLA_RECV_MBX * 2));
		}
	}


	/*Set up the ISR on the Modena-Ducati FIFO */
	if (mbx_ret_val != 0) {
		status = -ENODEV;
		WARN_ON(1);
		goto func_end;
	} else
		status = 0;


	if (status == 0) {
		driver_obj = drv_handle->driver_object;
		ctrl_ptr->reg_mask.mask = 0x0;
		ctrl_ptr->reg_mask.enable_mask = 0xFFFFFFFF;
		ctrl_ptr->recv_init_status = NOTIFYSHMDRV_INIT_STAMP;
		ctrl_ptr->send_init_status = NOTIFYSHMDRV_INIT_STAMP;
		drv_handle->is_init = NOTIFY_DRIVERINITSTATUS_DONE;
	}



func_end:
	mutex_unlock(notify_tesladriver_state.gate_handle);
	return drv_handle;
}
EXPORT_SYMBOL(notify_tesladrv_create);



/*
 *  brief      Function to delete the instance of tesla driver
 *
 */
int notify_tesladrv_delete(struct notify_driver_object **handlePtr)
{
	int status    = 0;
	struct notify_driver_object  *drv_handle = NULL;
	struct notify_tesladrv_object *driver_obj = NULL;
	struct notify_drv_eventlist *event_list;
	short int i;
	int proc_id;
	struct mbox_config *mbox_hw_config;
	int mbox_module_no;
	int interrupt_no;
	int mbx_ret_val = 0;


	WARN_ON(handlePtr == NULL);
	if (handlePtr == NULL)
		return -1;
	driver_obj = (struct notify_tesladrv_object *)(*handlePtr)->driver_object;
	drv_handle = (*handlePtr);
	WARN_ON((*handlePtr)->driver_object == NULL);

	/*Uninstall the ISRs & Disable the Mailbox interrupt.*/
	mbox_hw_config = ntfy_disp_get_config();
	mbox_module_no = mbox_hw_config->mbox_modules;
	interrupt_no = mbox_hw_config->interrupt_lines[mbox_module_no-1];

	if (drv_handle != NULL) {
		status = notify_unregister_driver(drv_handle);
		driver_obj = drv_handle->driver_object;
	}
	if (status != NOTIFY_SUCCESS)
		printk(KERN_WARNING "driver is not registerd\n");

	if (driver_obj != NULL) {
		if (driver_obj->ctrl_ptr != NULL) {
			/* Clear initialization status in shared memory. */
			driver_obj->ctrl_ptr->proc_ctrl[driver_obj->self_id].recv_init_status =
				0x0;
			driver_obj->ctrl_ptr->proc_ctrl[driver_obj->self_id].send_init_status =
				0x0;
			driver_obj->ctrl_ptr = NULL;
		}

		event_list = driver_obj->event_list;
		if (event_list != NULL) {
			/* Check if lists were created. */
			for (i = 0 ; i < driver_obj->params.num_events ; i++) {
				WARN_ON(event_list[i].event_handler_count != 0);
				event_list[i].event_handler_count = 0;
				list_del((struct list_head *)&event_list[i].listeners);
			}

			kfree(event_list);
			driver_obj->event_list = NULL;
		}

		/* Check if reg_chart was allocated. */
		if (driver_obj->reg_chart != NULL) {
			kfree(driver_obj->reg_chart);
			driver_obj->reg_chart = NULL;
		}

		/* Disable the interrupt, Uninstall the ISR and delete it. */
		/* Check if ISR was created. */
		/*Remove the ISR on the Modena-Tesla FIFO */
		proc_id = PROC_TESLA;
		ntfy_disp_interrupt_disable(mbox_module_no,
					    (NOTIFYDRV_TESLA_RECV_MBX * 2));
		ntfy_disp_unregister(mbox_module_no,
				     (NOTIFYDRV_TESLA_RECV_MBX * 2));

		/*Remove the generic ISR */
		mbx_ret_val = ntfy_disp_unbind_interrupt(interrupt_no);

		if (mbx_ret_val != 0)
			WARN_ON(1);

		kfree(driver_obj);
		driver_obj = NULL;
	}
	return status;
}

EXPORT_SYMBOL(notify_tesladrv_delete);


/*
 *  brief      Destroy the notify tesla module.
 *
 */
int notify_tesladrv_destroy(void)
{

	int status = 0;
	WARN_ON(notify_tesladriver_state.is_setup != true);

	/* Check if the gate_handle was created internally. */
	if (notify_tesladriver_state.cfg.gate_handle == NULL) {
		if (notify_tesladriver_state.gate_handle != NULL) {
			kfree(notify_tesladriver_state.gate_handle);
		}
	}
	notify_tesladriver_state.is_setup = false;
	return status;
}
EXPORT_SYMBOL(notify_tesladrv_destroy);



/*
 *  brief      Setup the notify tesla module.
 *
 *              This function sets up the notify tesla module. This function
 *              must be called before any other instance-level APIs can be
 *              invoked.
 *              Module-level configuration needs to be provided to this
 *              function. If the user wishes to change some specific config
 *              parameters, then notify_tesla_get_config can be called to get
 *              the configuration filled with the default values. After this,
 *              only the required configuration values can be changed. If the
 *              user does not wish to make any change in the default parameters,
 *              the application can simply call notify_tesla_setup with NULL
 *              parameters. The default parameters would get automatically used.
 *
 */
int notify_tesladrv_setup(struct notify_tesladrv_config *cfg)
{
	int status = 0;
	struct notify_tesladrv_config tmpCfg;

	if (cfg == NULL) {
		notify_tesladrv_getconfig(&tmpCfg);
		cfg = &tmpCfg;
	}

	if (cfg->gate_handle != NULL)
		notify_tesladriver_state.gate_handle = cfg->gate_handle;
	else {
		notify_tesladriver_state.gate_handle = kmalloc(sizeof(struct mutex),
						      GFP_KERNEL);
		mutex_init(notify_tesladriver_state.gate_handle);
	}

	if (notify_tesladriver_state.gate_handle == NULL) {
		status = -ENOMEM;
	} else {
		memcpy(&notify_tesladriver_state.cfg,
		       cfg, sizeof(struct notify_tesladrv_config));
		notify_tesladriver_state.is_setup = true;

	}

	return status;
}
EXPORT_SYMBOL(notify_tesladrv_setup);


/*
* brief     Register a callback for an event with the Notify driver.
*
*/
int  notify_tesladrv_register_event(
	struct notify_driver_object *handle,
	short int  proc_id,
	int  event_no,
	fn_notify_cbck     fn_notify_cbck,
	void *cbck_arg)
{
	int status = 0;
	int firstReg = false;
	bool done = true;
	struct notify_drv_eventlistner *event_listener;
	struct notify_drv_eventlist *event_list;
	struct notify_tesladrv_object *driver_object;
	struct notify_shmdrv_eventreg *reg_chart;
	struct notify_shmdrv_ctrl *ctrl_ptr;
	struct notify_shmdrv_event_entry *self_event_chart;
	int i;
	int j;
	BUG_ON(handle == NULL);
	BUG_ON(handle->is_init != NOTIFY_DRIVERINITSTATUS_DONE);
	BUG_ON(handle->driver_object == NULL);
	BUG_ON(fn_notify_cbck == NULL);


	driver_object = (struct notify_tesladrv_object *) handle->driver_object;

	ctrl_ptr = driver_object->ctrl_ptr;

	/* Allocate memory for event listener. */
	event_listener = kmalloc(sizeof(struct notify_drv_eventlistner), GFP_ATOMIC);

	if (event_listener == NULL) {
		status = -ENOMEM;
		goto func_end;
	} else {
		memset(event_listener, 0, sizeof(struct notify_drv_eventlistner));
	}

	if (mutex_lock_interruptible(notify_tesladriver_state.gate_handle) != 0)
		WARN_ON(1);

	event_list = driver_object->event_list;
	WARN_ON(event_list == NULL);
	event_listener->fn_notify_cbck = fn_notify_cbck;
	event_listener->cbck_arg = cbck_arg;
	/* Check if this is the first registration for this event. */

	if (list_empty((struct list_head *)
			&event_list[event_no].listeners)) {
		firstReg = true;
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


	if (firstReg == true) {
		reg_chart = driver_object->reg_chart;
		for (i = 0 ; i < driver_object->params.num_events ; i++) {
			/* Find the correct slot in the registration array. */
			if (reg_chart[i].reg_event_no == (int) -1) {
				for (j = (i - 1); j >= 0; j--) {
					if (event_no < reg_chart[j].reg_event_no) {
						reg_chart[j + 1].reg_event_no =
							reg_chart[j].reg_event_no;
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
			SET_BIT(ctrl_ptr->proc_ctrl[driver_object->self_id].
				reg_mask.mask,
				event_no);
		} else {
			/*! @retval NOTIFY_E_MAXEVENTS Maximum number of
			supported events have already been registered. */
			status = -EINVAL;
			list_del((struct list_head *)&
					(event_listener->element));
			kfree(event_listener);

		}
	}

func_end:
	mutex_unlock(notify_tesladriver_state.gate_handle);
	return status;
}


/*
*
* brief   unregister a callback for an event with the Notify driver.
*
*/

int notify_tesladrv_unregister_event(
		struct notify_driver_object *handle,
		short int  proc_id,
		int  event_no,
		fn_notify_cbck     fn_notify_cbck,
		void *cbck_arg)
{
	int status = 0;
	struct notify_drv_eventlistner *listener  = NULL;
	int num_events;
	struct notify_tesladrv_object *driver_object;
	struct notify_drv_eventlist *event_list;
	struct notify_shmdrv_eventreg *reg_chart;
	struct notify_shmdrv_ctrl *ctrl_ptr = NULL;
	struct notify_drv_eventlistner   unreg_info;
	struct notify_shmdrv_event_entry *self_event_chart;
	int i;
	int j;

	BUG_ON(fn_notify_cbck ==  NULL);
	BUG_ON(handle == NULL);
	BUG_ON(handle->driver_object == NULL);

	driver_object = (struct notify_tesladrv_object *) handle->driver_object;
	num_events = driver_object->params.num_events;

	ctrl_ptr = driver_object->ctrl_ptr;

	/* Enter critical section protection. */
	if (mutex_lock_interruptible(notify_tesladriver_state.gate_handle) != 0)
		WARN_ON(1);

	event_list = driver_object->event_list;


	unreg_info.fn_notify_cbck = fn_notify_cbck;
	unreg_info.cbck_arg = cbck_arg;
	notify_tesladrv_qsearch_elem(&event_list[event_no].listeners,
				&unreg_info,
				&listener);
	if (listener == NULL) {
		status = -EFAULT;
		goto func_end;
	}
	list_del((struct list_head *)&(listener->element));
	event_list[event_no].event_handler_count--;
	kfree(listener);


	if (list_empty((struct list_head *)
			&event_list[event_no].listeners) == true) {
		CLEAR_BIT(ctrl_ptr->proc_ctrl[driver_object->self_id].reg_mask.
				mask, event_no);
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
					reg_chart[j - 1].reg_event_no = (int) -1;
				}

				break;
			}
		}
	}



func_end:
	mutex_unlock(notify_tesladriver_state.gate_handle);
	return status;
}

/*
*brief      Send a notification event to the registered users for this
*            notification on the specified processor.
*
*/
int notify_tesladrv_sendevent(struct notify_driver_object *handle,
		short int proc_id, int event_no,
		int payload, short int wait_clear)
{
	int status = 0;
	struct notify_tesladrv_object*driver_object;
	struct notify_shmdrv_ctrl *ctrl_ptr;
	int max_poll_count;

	struct mbox_config *mbox_hw_config = ntfy_disp_get_config();
	int mbox_module_no = mbox_hw_config->mbox_modules;
	int mbx_ret_val = 0;
	BUG_ON(handle ==  NULL);
	BUG_ON(handle->driver_object == NULL);

	driver_object = (struct notify_tesladrv_object *) handle->driver_object;
	BUG_ON(event_no > driver_object->params.num_events);

	ctrl_ptr = driver_object->ctrl_ptr;
	max_poll_count = driver_object->params.send_event_poll_count;

	mbx_ret_val = ntfy_disp_send(mbox_module_no,
				     NOTIFYDRV_TESLA_SEND_MBX, payload);
	return status;
}

/*
* brief    Disable all events for this Notify driver.
*
*/
void *notify_tesladrv_disable(struct notify_driver_object *handle, u16 proc_id)
{
	int mbx_ret_val = KErrNone;
	struct mbox_config *mbox_hw_config = ntfy_disp_get_config();
	int mbox_module_no = mbox_hw_config->mbox_modules;
	mbx_ret_val = ntfy_disp_interrupt_disable(mbox_module_no,
			(NOTIFYDRV_TESLA_RECV_MBX * 2));
	return NULL; /*No flags to be returned. */
}

/*
* brief    Restore the Notify driver to the state before the last disable was
*           called.
*
*/
int notify_tesladrv_restore(struct notify_driver_object *handle,
					u32 key, u16 proc_id)
{
	int mbx_ret_val = 0;
	struct mbox_config *mbox_hw_config = ntfy_disp_get_config();
	int mbox_module_no = mbox_hw_config->mbox_modules;

	(void) handle;
	(void) key;
	(void) proc_id;
	/*Enable the receive interrupt for Tesla */
	mbx_ret_val = ntfy_disp_interrupt_enable(mbox_module_no,
			(NOTIFYDRV_TESLA_RECV_MBX * 2));
	return mbx_ret_val;
}

/*
* brief    Disable a specific event for this Notify driver.
*
*/
int notify_tesladrv_disable_event(
		struct notify_driver_object *handle,
		short int  proc_id, int  event_no)
{
	static int access_count ;
	signed long int status = 0;
	struct notify_tesladrv_object *driver_object;
	BUG_ON(handle == NULL);
	BUG_ON(handle->driver_object == NULL);
	access_count++;
	driver_object = (struct notify_tesladrv_object *) handle->driver_object;
	/* Enter critical section protection. */

	WARN_ON(mutex_lock_interruptible(notify_tesladriver_state.gate_handle) != 0);
	CLEAR_BIT(driver_object->ctrl_ptr->proc_ctrl[driver_object->self_id].
		  reg_mask.enable_mask, event_no);
	/* Leave critical section protection. */
	mutex_unlock(notify_tesladriver_state.gate_handle);
	return status;
}

/*
* brief         Enable a specific event for this Notify driver.
*
*/
int notify_tesladrv_enable_event(struct notify_driver_object *handle,
	short int proc_id, int event_no)
{
	int status = 0;
	struct notify_tesladrv_object *driver_object;
	BUG_ON(handle == NULL);
	BUG_ON(handle->driver_object == NULL);

	driver_object = (struct notify_tesladrv_object *) handle->driver_object;
	/* Enter critical section protection. */

	WARN_ON(mutex_lock_interruptible(notify_tesladriver_state.gate_handle) != 0);
	SET_BIT(driver_object->ctrl_ptr->proc_ctrl[driver_object->self_id].
		reg_mask.enable_mask, event_no);

	mutex_unlock(notify_tesladriver_state.gate_handle);
	return status;
}

/*
* brief     Print debug information for the Notify driver.
*
*/
int notify_tesladrv_debug(struct notify_driver_object *handle)
{
	int status = 0;
	printk(KERN_WARNING "Tesla Debug: Nothing being printed currently.\n");
	return status;
}

/*
* brief     This function implements the interrupt service routine for the
*            interrupt received from the DSP.
*
*/
void notify_tesladrv_isr(void *ref_data)
{
	int payload = 0;
	int i = 0;
	struct list_head *temp;
	int j = 0;
	struct notify_shmdrv_eventreg *reg_chart;
	int event_no;
	struct notify_tesladrv_object *drv_object;
	struct notify_shmdrv_proc_ctrl *ctrl_ptr;
	struct mbox_config *mbox_hw_config = ntfy_disp_get_config();
	unsigned long int mbox_module_no = mbox_hw_config->mbox_modules;
	signed long int mbx_ret_val = 0;
	int num_messages = 0;
	int num_events = 0;

	drv_object = (struct notify_tesladrv_object *) ref_data;
	ctrl_ptr = &(drv_object->ctrl_ptr->proc_ctrl[drv_object->self_id]);
	reg_chart = drv_object->reg_chart;
	num_events = drv_object->params.num_events;


	/* Commented out, since this function will be called in an ISR */

	BUG_ON(drv_object == NULL);


	do {
		event_no = reg_chart[i].reg_event_no;
		if (event_no != (unsigned long int) -1) {
			if (TEST_BIT(ctrl_ptr->reg_mask.enable_mask, event_no) == 1) {
				mbx_ret_val = ntfy_disp_read(mbox_module_no,
						NOTIFYDRV_TESLA_RECV_MBX,
						&payload,
						&num_messages, false);
				BUG_ON(mbx_ret_val != 0);
				temp  = drv_object->event_list[event_no].
					listeners.next;

				for (j = 0; j <
						drv_object->event_list[event_no].
						event_handler_count; j++) {
					if (temp > (struct list_head *)0) {
						((struct
						notify_drv_eventlistner
						*)temp)->fn_notify_cbck(
							drv_object->
							proc_id,
							event_no,
							((struct
							notify_drv_eventlistner
							*)
							temp)->cbck_arg,
							payload);
						temp = temp->next;
					}
				}
			}
			i++;
		}
	} while ((event_no != (unsigned long int) -1) && (i < num_events));
}
EXPORT_SYMBOL(notify_tesladrv_isr);

/*
* brief     This function searchs for a element the List.
*
*/
static void notify_tesladrv_qsearch_elem(struct list_head *list,
		struct notify_drv_eventlistner *check_obj,
		struct notify_drv_eventlistner **listener)
{
	struct list_head    *temp = NULL ;
	struct notify_drv_eventlistner *l_temp = NULL ;
	short int found = false;


	BUG_ON(list ==  NULL);
	BUG_ON(check_obj == NULL);
	WARN_ON(listener == NULL);

	if (listener != NULL)
		return;
	*listener = NULL;
	if ((check_obj != NULL)) {
		if (list_empty((struct list_head *)list) == false) {
			temp = list->next;
			while ((found == false) && (temp != NULL)) {
				l_temp =
				(struct notify_drv_eventlistner *)(temp);
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

