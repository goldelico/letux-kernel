/*
 * drv_notify.c
 *
 * Syslink support functions for TI OMAP processors.
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

#include <linux/autoconf.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <asm/pgtable.h>
#include <linux/types.h>
#include <linux/cdev.h>

#include <syslink/gt.h>
#include <syslink/notify_driver.h>
#include <syslink/notify.h>
#include <syslink/GlobalTypes.h>


/** ============================================================================
 *  Macros and types
 *  ============================================================================
 */
#define IPCNOTIFY_NAME "ipcnotify"

static char *driver_name =  IPCNOTIFY_NAME;

static s32 driver_major;

static s32 driver_minor;

struct ipcnotify_dev {
	struct cdev cdev;
};

static struct ipcnotify_dev *ipcnotify_device;

static struct class *ipcnotify_class;


/*
 * Maximum number of user supported.
 */

#define MAX_PROCESSES 32

/*Structure of Event Packet read from notify kernel-side..*/
struct notify_drv_event_packet {
	struct list_head element;
	u32 pid;
	u32 proc_id;
	u32 event_no;
	u32 data;
	notify_callback_fxn func;
	void *param;
	bool is_exit;
};

/*Structure of Event callback argument passed to register fucntion*/
struct notify_drv_event_cbck {
	struct list_head element;
	u32 proc_id;
	notify_callback_fxn func;
	void *param;
	u32 pid;
};

/*Keeps the information related to Event.*/
struct notifydrv_event_state {
	struct list_head buf_list;
	u32 pid;
	u32 ref_count;
	/*Reference count, used when multiple Notify_registerEvent are called
	from same process space(multi threads/processes). */
	struct semaphore *semhandle;
	/* Semphore for waiting on event. */
	struct semaphore *tersemhandle;
	/* Termination synchronization semaphore. */
};

struct notifydrv_moduleobject{
	bool is_setup;
	/*Indicates whether the module has been already setup */
	int open_refcount;
	/* Open reference count. */
	struct mutex *gatehandle;
	/*Handle of gate to be used for local thread safety */
	struct list_head evt_cbck_list;
	/*List containg callback arguments for all registered handlers from
	 user mode. */
	struct notifydrv_event_state event_state[MAX_PROCESSES];
	/* List for all user processes registered. */
};

struct notifydrv_moduleobject notifydrv_state = {
	.is_setup = false,
	.open_refcount = 0,
	.gatehandle = NULL,
};

/*Major number of driver.*/
int major = 232;

static void notify_drv_setup(void);

static int notify_drv_add_buf_by_pid(u16 procId, u32 pid,
		u32 eventNo, u32 data, notify_callback_fxn cbFxn, void *param);

/* open the Notify driver object..*/
static int notify_drv_open(struct inode *inode, struct file *filp) ;

/* close the Notify driver object..*/
static int notify_drv_close(struct inode *inode, struct file *filp);

/* Linux driver function to map memory regions to user space. */
static int notify_drv_mmap(struct file *filp, struct vm_area_struct *vma);

/* read function for of Notify driver.*/
static int notify_drv_read(struct file *filp, char *dst,
				size_t size, loff_t *offset);

/* ioctl function for of Linux Notify driver.*/
static int notify_drv_ioctl(struct inode *inode, struct file *filp, u32 cmd,
						unsigned long args);

/* Module initialization function for Linux driver.*/
static int __init notify_drv_init_module(void);

/* Module finalization function for Linux driver.*/
static void __exit notify_drv_finalize_module(void) ;

static void notify_drv_destroy(void);

static int notify_drv_register_driver(void);

static int notify_drv_unregister_driver(void);

/* Attach a process to notify user support framework. */
static int notify_drv_attach(u32 pid);

/* Detach a process from notify user support framework. */
static int notify_drv_detach(u32 pid);


/* Function to invoke the APIs through ioctl.*/
static const struct file_operations driver_ops = {
	.open = notify_drv_open,
	.ioctl = notify_drv_ioctl,
	.release = notify_drv_close,
	.read = notify_drv_read,
	.mmap = notify_drv_mmap,
};

static int notify_drv_register_driver(void)
{
	notify_drv_setup();
	return 0;
}

static int notify_drv_unregister_driver(void)
{
	notify_drv_destroy();
	return 0;
}


/*
* This function implements the callback registered with IPS. Here
* to pass event no. back to user function(so that it can do another
* level of demultiplexing of callbacks)
*/
static void notify_drv_cbck(u16 proc_id, u32 event_no,
				void *arg, u32 payload)
{
	struct notify_drv_event_cbck *cbck;

	if (WARN_ON(notifydrv_state.is_setup == false))
		goto func_end;
	BUG_ON(arg == NULL);
	cbck = (struct notify_drv_event_cbck *)arg;
	notify_drv_add_buf_by_pid(proc_id, cbck->pid, event_no, payload,
					cbck->func, cbck->param);
func_end:
	return;
}

/*
 * Linux specific function to open the driver.
 */
static int notify_drv_open(struct inode *inode, struct file *filp)
{
	return 0;
}

/*
 * close the driver
 */
static int notify_drv_close(struct inode *inode, struct file *filp)
{
	return 0 ;
}

/*
 * read data from the driver
 */
static int notify_drv_read(struct file *filp, char *dst, size_t size,
		loff_t *offset)
{

	bool flag = false;
	struct notify_drv_event_packet *u_buf = NULL;
	int ret_val = 0;
	u32 i;
	struct list_head *elem;
	struct notify_drv_event_packet t_buf;

	if (WARN_ON(notifydrv_state.is_setup == false)) {
		ret_val = -EFAULT;
		goto func_end;
	}

	ret_val = copy_from_user((void *)&t_buf,
				(void *)dst,
				sizeof(struct notify_drv_event_packet));
	WARN_ON(ret_val != 0);


	for (i = 0 ; i < MAX_PROCESSES ; i++) {
		if (notifydrv_state.event_state[i].pid == t_buf.pid) {
			flag = true;
			break;
		}
	}
	if (flag == false) {
		ret_val = -EFAULT;
		goto func_end;
	}
	if (list_empty(&(notifydrv_state.event_state[i].buf_list)) == false) {
		/* Wait for the event */
		ret_val = down_interruptible(
			notifydrv_state.event_state[i].semhandle);
		if (ret_val < 0) {
			ret_val = -ERESTARTSYS;
			goto func_end;
		}
		WARN_ON(mutex_lock_interruptible(notifydrv_state.gatehandle));
		elem = ((struct list_head *)&(notifydrv_state.event_state[i]. \
							buf_list))->next;
		u_buf = container_of(elem, struct notify_drv_event_packet,
				    element);
		list_del(elem);
		mutex_unlock(notifydrv_state.gatehandle);
		if (u_buf == NULL) {
			ret_val = -EFAULT;
			goto func_end;
		}
		ret_val = copy_to_user((void *)dst, u_buf,
			sizeof(struct notify_drv_event_packet));
		kfree(u_buf);

		if (WARN_ON(ret_val != 0))
			ret_val = -EFAULT;
		if (u_buf->is_exit == true)
			up(notifydrv_state.event_state[i].tersemhandle);
	}
func_end:
	return ret_val ;
}

static int notify_drv_mmap(struct file *filp, struct vm_area_struct *vma)
{
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
		vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		return -EAGAIN;
	}
	return 0;
}

/*
 * name notify_drv_ioctl
 *
 * ioctl function for of Linux Notify driver.
 *
*/
static int notify_drv_ioctl(struct inode *inode, struct file *filp, u32 cmd,
					unsigned long args)
{
	int retval = 0;
	int status = NOTIFY_SUCCESS;
	struct notify_cmd_args *cmdArgs = (struct notify_cmd_args *)args;
	struct notify_cmd_args commonArgs;

	switch (cmd) {
	case CMD_NOTIFY_GETCONFIG:
	{
		struct notify_cmd_args_get_config *src_args =
				(struct notify_cmd_args_get_config *)args;
		struct notify_config cfg;
		notify_get_config(&cfg);
		retval = copy_to_user((void *) (src_args->cfg),
			(const void *) &cfg, sizeof(struct notify_config));
	}
	break;

	case CMD_NOTIFY_SETUP:
	{
		struct notify_cmd_args_setup *src_args =
			(struct notify_cmd_args_setup *) args;
		struct notify_config cfg;

		retval = copy_from_user((void *) &cfg,
					 (const void *) (src_args->cfg),
					 sizeof(struct notify_config));
		if (WARN_ON(retval != 0))
			goto func_end;
		notify_setup(&cfg);
	}
	break;

	case CMD_NOTIFY_DESTROY:
	{
		/* copy_from_user is not needed for Notify_getConfig, since the
		 * user's config is not used.
		 */
		status = notify_destroy();
	}
	break;

	case CMD_NOTIFY_REGISTEREVENT:
	{
		struct notify_cmd_args_register_event src_args;
		struct notify_drv_event_cbck *cbck = NULL;

		/* Copy the full args from user-side. */
		retval = copy_from_user((void *) &src_args,
				(const void *) (args),
				sizeof(struct notify_cmd_args_register_event));

		if (WARN_ON(retval != 0))
			goto func_end;
		cbck = kmalloc(sizeof(struct notify_drv_event_cbck),
					GFP_ATOMIC);
		WARN_ON(cbck == NULL);
		cbck->proc_id = src_args.procId;
		cbck->func = src_args.fnNotifyCbck;
		cbck->param = src_args.cbckArg;
		cbck->pid = src_args.pid;
		status = notify_register_event(src_args.handle, src_args.procId,
			src_args.eventNo, notify_drv_cbck, (void *)cbck);
		if (status < 0) {
			/* This does not impact return status of this function,
			 * so retval comment is not used.
			 */
			kfree(cbck);
		} else {
			WARN_ON(mutex_lock_interruptible
					(notifydrv_state.gatehandle));
			INIT_LIST_HEAD((struct list_head *)&(cbck->element));
			list_add_tail(&(cbck->element),
				&(notifydrv_state.evt_cbck_list));
			mutex_unlock(notifydrv_state.gatehandle);
		}
	}
	break;

	case CMD_NOTIFY_UNREGISTEREVENT:
	{
		bool found = false;
		u32 pid;
		struct notify_drv_event_cbck *cbck = NULL;
		struct list_head *entry = NULL;
		struct notify_cmd_args_unregister_event src_args;

		/* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args, (const void *)(args),
			sizeof(struct notify_cmd_args_unregister_event));

		if (WARN_ON(retval != 0))
			goto func_end;

		pid = src_args.pid;
		WARN_ON(mutex_lock_interruptible(notifydrv_state.gatehandle));
		list_for_each(entry,
			(struct list_head *)&(notifydrv_state.evt_cbck_list)) {
			cbck = (struct notify_drv_event_cbck *)(entry);
			if ((cbck->func == src_args.fnNotifyCbck) &&
				(cbck->param == src_args.cbckArg) &&
				(cbck->pid == pid) &&
				(cbck->proc_id == src_args.procId)) {
					found = true;
					break;
			}
		}
		mutex_unlock(notifydrv_state.gatehandle);
		if (found == false) {
			status = NOTIFY_E_NOTFOUND;
			goto func_end;
		}
		status = notify_unregister_event(src_args.handle,
				src_args.procId,
				src_args.eventNo,
				notify_drv_cbck, (void *) cbck);
		/* This check is needed at run-time also to propagate the
		 * status to user-side. This must not be optimized out.
		 */
		if (status < 0)
			printk(KERN_ERR " notify_unregister_event failed \n");
		else {
			WARN_ON(mutex_lock_interruptible
						(notifydrv_state.gatehandle));
			list_del((struct list_head *)cbck);
			mutex_unlock(notifydrv_state.gatehandle);
			kfree(cbck);
		}
	}
	break;

	case CMD_NOTIFY_SENDEVENT:
	{
		struct notify_cmd_args_send_event src_args;

		/* Copy the full args from user-side. */
		retval = copy_from_user((void *) &src_args,
				(const void *) (args),
				sizeof(struct notify_cmd_args_send_event));
		if (WARN_ON(retval != 0)) {
			retval = -EFAULT;
			goto func_end;
		}
		status = notify_sendevent(src_args.handle, src_args.procId,
				src_args.eventNo, src_args.payload,
				src_args.waitClear);
	}
	break;

	case CMD_NOTIFY_DISABLE:
	{
		struct notify_cmd_args_disable src_args;

		/* Copy the full args from user-side. */
		retval = copy_from_user((void *) &src_args,
				(const void *) (args),
				sizeof(struct notify_cmd_args_disable));

		/* This check is needed at run-time also since it depends on
		 * run environment. It must not be optimized out.
		 */
		if (WARN_ON(retval != 0)) {
			retval = -EFAULT;
			goto func_end;
		}
		src_args.flags = notify_disable(src_args.procId);

		/* Copy the full args to user-side */
		retval = copy_to_user((void *) (args),
					(const void *) &src_args,
					sizeof(struct notify_cmd_args_disable));
		/* This check is needed at run-time also since it depends on
		 * run environment. It must not be optimized out.
		 */
		if (WARN_ON(retval != 0))
			retval = -EFAULT;
	}
	break;

	case CMD_NOTIFY_RESTORE:
	{
		struct notify_cmd_args_restore src_args;

		/* Copy the full args from user-side. */
		retval = copy_from_user((void *) &src_args,
				(const void *)(args),
				sizeof(struct notify_cmd_args_restore));
		if (WARN_ON(retval != 0)) {
			retval = -EFAULT;
			goto func_end;
		}
		notify_restore(src_args.key, src_args.procId);
	}
	break;

	case CMD_NOTIFY_DISABLEEVENT:
	{
		struct notify_cmd_args_disable_event src_args;

		/* Copy the full args from user-side. */
		retval = copy_from_user((void *) &src_args,
				(const void *)(args),
				sizeof(struct notify_cmd_args_disable_event));

		/* This check is needed at run-time also since it depends on
		 * run environment. It must not be optimized out.
		 */
		if (WARN_ON(retval != 0)) {
			retval = -EFAULT;
			goto func_end;
		}
		notify_disable_event(src_args.handle, src_args.procId,
						src_args.eventNo);
	}
	break;

	case CMD_NOTIFY_ENABLEEVENT:
	{
		struct notify_cmd_args_enable_event src_args;

		/* Copy the full args from user-side. */
		retval = copy_from_user((void *)&src_args,
				(const void *)(args),
				sizeof(struct notify_cmd_args_enable_event));
		if (WARN_ON(retval != 0)) {
			retval = -EFAULT;
			goto func_end;
		}
		notify_enable_event(src_args.notify_driver_handle,
				src_args.procId, src_args.eventNo);
	}
	break;

	case CMD_NOTIFY_ATTACH:
	{
		/* FIXME: User copy_from_user */
		u32 pid = *((u32 *)args);
		status = notify_drv_attach(pid);

		if (status < 0)
			printk(KERN_ERR "NOTIFY_ATTACH FAILED\n");
	}
	break;

	case CMD_NOTIFY_DETACH:
	{
		/* FIXME: User copy_from_user */
		u32 pid = *((u32 *)args);
		status = notify_drv_detach(pid);

		if (status < 0)
			printk(KERN_ERR "NOTIFY_DETACH FAILED\n");
	}
	break;

	default:
	{
		/* This does not impact return status of this function,so retval
		 * comment is not used.
		 */
		status = NOTIFY_E_INVALIDARG;
		printk(KERN_ERR "not valid command\n");
	}
	break;
	}
func_end:
	/* Set the status and copy the common args to user-side. */
	commonArgs.apiStatus = status;
	status = copy_to_user((void *) cmdArgs,
			(const void *) &commonArgs,
			sizeof(struct notify_cmd_args));
	if (status < 0)
		retval = -EFAULT;
	return retval;
}

/*====================
 * notify_drv_add_buf_by_pid
 *
 */
static int notify_drv_add_buf_by_pid(u16 proc_id, u32 pid,
		u32 event_no, u32 data, notify_callback_fxn cbFxn, void *param)
{
	s32 status = 0;
	bool flag = false;
	bool is_exit = false;
	struct notify_drv_event_packet *u_buf = NULL;
	u32 i;

	for (i = 0 ; (i < MAX_PROCESSES) && (flag != true) ; i++) {
		if (notifydrv_state.event_state[i].pid == pid) {
			flag = true ;
			break ;
		}
	}
	if (WARN_ON(flag == false)) {
		status = -EFAULT;
		goto func_end;
	}
	u_buf = kmalloc(sizeof(struct notify_drv_event_packet), GFP_ATOMIC);
	if (u_buf != NULL) {
		INIT_LIST_HEAD((struct list_head *)&u_buf->element);
		u_buf->proc_id = proc_id;
		u_buf->data = data ;
		u_buf->event_no = event_no ;
		u_buf->func = cbFxn;
		u_buf->param = param;
		if (u_buf->event_no == (u32) -1) {
			u_buf->is_exit = true;
			is_exit = true;
		}
		if (mutex_lock_interruptible(notifydrv_state.gatehandle))
			return NOTIFY_E_OSFAILURE;
		list_add_tail((struct list_head *)&(u_buf->element),
			(struct list_head *)
			&(notifydrv_state.event_state[i].buf_list));
		mutex_unlock(notifydrv_state.gatehandle);

		up(notifydrv_state.event_state[i].semhandle);
		/* Termination packet */
		if (is_exit == true) {
			if (down_interruptible(notifydrv_state.
				event_state[i].tersemhandle))
				status = NOTIFY_E_OSFAILURE;
		}
	}
func_end:
	return status;
}

/*
 * Module setup function.
 *
 */
static void notify_drv_setup(void)
{
	int i;

	INIT_LIST_HEAD((struct list_head *)&(notifydrv_state.evt_cbck_list));
	notifydrv_state.gatehandle = kmalloc(sizeof(struct mutex),
						GFP_KERNEL);
	mutex_init(notifydrv_state.gatehandle);
	for (i = 0; i < MAX_PROCESSES ; i++) {
		notifydrv_state.event_state[i].pid = -1;
		notifydrv_state.event_state[i].ref_count = 0;
		INIT_LIST_HEAD((struct list_head *)
			&(notifydrv_state.event_state[i].buf_list));
	}
	notifydrv_state.is_setup = true;
}


/*
* brief Module destroy function.
*/
static void notify_drv_destroy(void)
{
	int i;
	struct notify_drv_event_packet *packet;
	struct list_head *entry;
	struct notify_drv_event_cbck *cbck;

	for (i = 0; i < MAX_PROCESSES ; i++) {
		list_for_each(entry, (struct list_head *)
				&(notifydrv_state.event_state[i].buf_list)) {
			packet = (struct notify_drv_event_packet *)entry;
			if (packet != NULL)
				kfree(packet);
		}
		INIT_LIST_HEAD(&notifydrv_state.event_state[i].buf_list);
	}
	list_for_each(entry,
			(struct list_head *)&(notifydrv_state.evt_cbck_list)) {
		cbck = (struct notify_drv_event_cbck *)(entry);
		if (cbck != NULL)
			kfree(cbck) ;
	}
	INIT_LIST_HEAD(&notifydrv_state.evt_cbck_list);
	mutex_destroy(notifydrv_state.gatehandle);
	kfree(notifydrv_state.gatehandle);
	notifydrv_state.is_setup = false;
	return;
}




/*
 *  brief      Attach a process to notify user support framework.
 */
static int notify_drv_attach(u32 pid)
{
	s32 status = NOTIFY_SUCCESS;
	bool flag = false;
	bool is_init = false;
	u32 i;
	struct semaphore *sem_handle;
	struct semaphore *ter_sem_handle;
	int ret_val = 0;

	if (notifydrv_state.is_setup == false) {
		status = NOTIFY_E_FAIL;
	} else {
		WARN_ON(mutex_lock_interruptible(notifydrv_state.gatehandle));

		for (i = 0 ; (i < MAX_PROCESSES) ; i++) {
			if (notifydrv_state.event_state[i].pid == pid) {
				notifydrv_state.event_state[i].ref_count++;
				is_init = true;
				break;
			}
		}
		mutex_unlock(notifydrv_state.gatehandle);

		if (is_init == true)
			goto func_end;

		sem_handle = kmalloc(sizeof(struct semaphore), GFP_ATOMIC);
		ter_sem_handle = kmalloc(sizeof(struct semaphore), GFP_ATOMIC);

		sema_init(sem_handle, 0);
		/* Create the termination semaphore */
		sema_init(ter_sem_handle, 0);

		WARN_ON(mutex_lock_interruptible(notifydrv_state.gatehandle));
		/* Search for an available slot for user process. */
		for (i = 0 ; i < MAX_PROCESSES ; i++) {
			if (notifydrv_state.event_state[i].pid == -1) {
				notifydrv_state.event_state[i].semhandle =
							sem_handle;
				notifydrv_state.event_state[i].tersemhandle =
							ter_sem_handle;
				notifydrv_state.event_state[i].pid = pid;
				notifydrv_state.event_state[i].ref_count
								= 1;
				INIT_LIST_HEAD(&(notifydrv_state.event_state[i].
								buf_list));
				flag = true;
				break;
			}
		}
		mutex_unlock(notifydrv_state.gatehandle);

		if (WARN_ON(flag != true)) {
			/* Max users have registered. No more clients
			 * can be supported */
			status = NOTIFY_E_MAXCLIENTS;
		}

		if (status == NOTIFY_SUCCESS)
			ret_val = 0;
		else {
			kfree(ter_sem_handle);
			kfree(sem_handle);
			ret_val = -EINVAL;
		}

	}
func_end:
    return ret_val;
}


/*
 *  brief      Detach a process from notify user support framework.
 */
static int notify_drv_detach(u32 pid)
{
	s32 status = NOTIFY_SUCCESS;
	bool flag = false;
	u32 i;
	struct semaphore *sem_handle;
	struct semaphore *ter_sem_handle;

	if (notifydrv_state.is_setup == false) {
		status = NOTIFY_E_FAIL;
		goto func_end;
	}

	/* Send the termination packet to notify thread */
	status = notify_drv_add_buf_by_pid(0, pid, (u32)-1, (u32)0,
					NULL, NULL);

	if (status < 0)
		goto func_end;

	if (mutex_lock_interruptible(notifydrv_state.gatehandle)) {
		status = NOTIFY_E_OSFAILURE;
		goto func_end;
	}
	for (i = 0; i < MAX_PROCESSES; i++) {
		if (notifydrv_state.event_state[i].pid == pid) {
			if (notifydrv_state.event_state[i].ref_count == 1) {
				/* Last client being unregistered for this
				* process*/
				notifydrv_state.event_state[i].pid = -1;
				notifydrv_state.event_state[i].ref_count = 0;
				sem_handle =
				notifydrv_state.event_state[i].semhandle;
				ter_sem_handle =
				notifydrv_state.event_state[i].tersemhandle;
				INIT_LIST_HEAD((struct list_head *)
				&(notifydrv_state.event_state[i].buf_list));
				notifydrv_state.event_state[i].semhandle =
								NULL;
				notifydrv_state.event_state[i].tersemhandle =
								NULL;
				flag = true;
				break;
			} else
				notifydrv_state.event_state[i].ref_count--;
		}
	}
	mutex_unlock(notifydrv_state.gatehandle);

	if ((flag == false) && (i == MAX_PROCESSES)) {
		/*retval NOTIFY_E_NOTFOUND The specified user process was
		 * not found registered with Notify Driver module. */
	    status = NOTIFY_E_NOTFOUND;
	} else {
	    kfree(sem_handle);
	    kfree(ter_sem_handle);
	}
func_end:
	    return status;

    /*! @retval NOTIFY_SUCCESS Operation successfully completed */
    return status;
}


/* Module initialization function for Notify driver.*/
static int __init notify_drv_init_module(void)
{
	int result = 0 ;
	dev_t dev;

	if (driver_major) {
		dev = MKDEV(driver_major, driver_minor);
		result = register_chrdev_region(dev, 1, driver_name);
	} else {
		result = alloc_chrdev_region(&dev, driver_minor, 1,
				 driver_name);
		driver_major = MAJOR(dev);
	}

	ipcnotify_device = kmalloc(sizeof(struct ipcnotify_dev), GFP_KERNEL);
	if (!ipcnotify_device) {
		result = -ENOMEM;
		unregister_chrdev_region(dev, 1);
		goto func_end;
	}
	memset(ipcnotify_device, 0, sizeof(struct ipcnotify_dev));
	cdev_init(&ipcnotify_device->cdev, &driver_ops);
	ipcnotify_device->cdev.owner = THIS_MODULE;
	ipcnotify_device->cdev.ops = &driver_ops;

	result = cdev_add(&ipcnotify_device->cdev, dev, 1);

	if (result) {
		printk(KERN_ERR "Failed to add the syslink ipcnotify device \n");
		goto func_end;
	}

	/* udev support */
	ipcnotify_class = class_create(THIS_MODULE, "syslink-ipcnotify");

	if (IS_ERR(ipcnotify_class)) {
		printk(KERN_ERR "Error creating ipcnotify class \n");
		goto func_end;
	}
	device_create(ipcnotify_class, NULL, MKDEV(driver_major, driver_minor),
			NULL, IPCNOTIFY_NAME);
	result = notify_drv_register_driver();
func_end:
	return result ;
}

/* Module finalization function for Notify driver.*/
static void __exit notify_drv_finalize_module(void)
{
	dev_t dev_no;

	notify_drv_unregister_driver();

	dev_no = MKDEV(driver_major, driver_minor);
	if (ipcnotify_device) {
		cdev_del(&ipcnotify_device->cdev);
		kfree(ipcnotify_device);
	}
	unregister_chrdev_region(dev_no, 1);
	if (ipcnotify_class) {
		/* remove the device from sysfs */
		device_destroy(ipcnotify_class, MKDEV(driver_major,
						driver_minor));
		class_destroy(ipcnotify_class);
	}
	return;
}


/*
 *name module_init/module_exit
 *
 *desc Macro calls that indicate initialization and finalization functions
 *to the kernel.
 *
 */
module_init(notify_drv_init_module) ;
module_exit(notify_drv_finalize_module) ;
MODULE_LICENSE("GPL");

