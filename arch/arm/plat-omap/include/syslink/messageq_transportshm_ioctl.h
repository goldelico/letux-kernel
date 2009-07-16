/*
 *  messageq_transportshm_ioctl.h
 *
 *  Definitions of messageq_transportshm driver types and structures.
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

#ifndef _MESSAGEQ_TRANSPORTSHM_IOCTL_H_
#define _MESSAGEQ_TRANSPORTSHM_IOCTL_H_

/* Standard headers */
#include <linux/types.h>

/* Syslink headers */
#include <ipc_ioctl.h>
#include <messageq_transportshm.h>
#include <sharedregion.h>


/* =============================================================================
 *  Macros and types
 * =============================================================================
 */
/* Base command ID for messageq_transportshm */
#define MESSAGEQ_TRANSPORTSHM_IOC_MAGIC		IPC_IOC_MAGIC
enum messageq_transportshm_drv_cmd {
	MESSAGEQ_TRANSPORTSHM_GETCONFIG = MESSAGEQ_TRANSPORTSHM_BASE_CMD,
	MESSAGEQ_TRANSPORTSHM_SETUP,
	MESSAGEQ_TRANSPORTSHM_DESTROY,
	MESSAGEQ_TRANSPORTSHM_PARAMS_INIT,
	MESSAGEQ_TRANSPORTSHM_CREATE,
	MESSAGEQ_TRANSPORTSHM_DELETE,
	MESSAGEQ_TRANSPORTSHM_PUT,
	MESSAGEQ_TRANSPORTSHM_SHAREDMEMREQ,
	MESSAGEQ_TRANSPORTSHM_GETSTATUS
};

/*  ----------------------------------------------------------------------------
 *  IOCTL command IDs for messageq_transportshm
 *  ----------------------------------------------------------------------------
 */
/* Base command ID for messageq_transportshm */
#define MESSAGEQ_TRANSPORTSHM_BASE_CMD		0x0

/* Command for messageq_transportshm_get_config */
#define CMD_MESSAGEQ_TRANSPORTSHM_GETCONFIG \
	_IOWR(MESSAGEQ_TRANSPORTSHM_IOC_MAGIC, \
	MESSAGEQ_TRANSPORTSHM_GETCONFIG, struct messageq_transportshm_cmd_args)

/* Command for messageq_transportshm_setup */
#define CMD_MESSAGEQ_TRANSPORTSHM_SETUP \
	_IOWR(MESSAGEQ_TRANSPORTSHM_IOC_MAGIC, MESSAGEQ_TRANSPORTSHM_SETUP, \
	struct messageq_transportshm_cmd_args)

/* Command for messageq_transportshm_setup */
#define CMD_MESSAGEQ_TRANSPORTSHM_DESTROY \
	_IOWR(MESSAGEQ_TRANSPORTSHM_IOC_MAGIC, MESSAGEQ_TRANSPORTSHM_DESTROY, \
	struct messageq_transportshm_cmd_args)

/* Command for messageq_transportshm_destroy */
#define CMD_MESSAGEQ_TRANSPORTSHM_PARAMS_INIT \
	_IOWR(MESSAGEQ_TRANSPORTSHM_IOC_MAGIC, \
	MESSAGEQ_TRANSPORTSHM_PARAMS_INIT, \
	struct messageq_transportshm_cmd_args)

/* Command for messageq_transportshm_create */
#define CMD_MESSAGEQ_TRANSPORTSHM_CREATE \
	_IOWR(MESSAGEQ_TRANSPORTSHM_IOC_MAGIC, MESSAGEQ_TRANSPORTSHM_CREATE, \
	struct messageq_transportshm_cmd_args)

/* Command for messageq_transportshm_delete */
#define CMD_MESSAGEQ_TRANSPORTSHM_DELETE \
	_IOWR(MESSAGEQ_TRANSPORTSHM_IOC_MAGIC, MESSAGEQ_TRANSPORTSHM_DELETE, \
	struct messageq_transportshm_cmd_args)

/* Command for messageq_transportshm_put */
#define CMD_MESSAGEQ_TRANSPORTSHM_PUT \
	_IOWR(MESSAGEQ_TRANSPORTSHM_IOC_MAGIC, MESSAGEQ_TRANSPORTSHM_PUT, \
	struct messageq_transportshm_cmd_args)

/* Command for messageq_transportshm_shared_memreq */
#define CMD_MESSAGEQ_TRANSPORTSHM_SHAREDMEMREQ \
	_IOWR(MESSAGEQ_TRANSPORTSHM_IOC_MAGIC, \
	MESSAGEQ_TRANSPORTSHM_SHAREDMEMREQ, \
	struct messageq_transportshm_cmd_args)

/* Command for messageq_transportshm_get_status */
#define CMD_MESSAGEQ_TRANSPORTSHM_GETSTATUS \
	_IOWR(MESSAGEQ_TRANSPORTSHM_IOC_MAGIC, \
	MESSAGEQ_TRANSPORTSHM_GETSTATUS, struct messageq_transportshm_cmd_args)

/* Command arguments for messageq_transportshm */
struct messageq_transportshm_cmd_args {
	union {
		struct {
			struct messageq_transportshm_config *config;
		} get_config;

		struct {
			struct messageq_transportshm_config *config;
		} setup;

		struct {
			void *messageq_transportshm_handle;
			struct messageq_transportshm_params *params;
		} params_init;

		struct {
			void *messageq_transportshm_handle;
			u16 proc_id;
			struct messageq_transportshm_params *params;
			u32 shared_addr_srptr;
			void *knl_lock_handle;
			void *knl_notify_driver;
		} create;

		struct {
			void *messageq_transportshm_handle;
		} delete_transport;

		struct {
			void *messageq_transportshm_handle;
			u32 *msg_srptr;
		} put;

		struct {
			void *messageq_transportshm_handle;
			enum messageq_transportshm_status status;
		} get_status;

		struct {
			struct messageq_transportshm_params *params;
			u32 bytes;
		} shared_memreq;
	} args;

	int api_status;
};

/*  ----------------------------------------------------------------------------
 *  IOCTL functions for messageq_transportshm module
 *  ----------------------------------------------------------------------------
 */
/*
 *  ioctl interface function for messageq_transportshm
 */
int messageq_transportshm_ioctl(struct inode *inode, struct file *filp,
				unsigned int cmd, unsigned long args);

#endif /* _MESSAGEQ_TRANSPORTSHM_IOCTL_H_ */
