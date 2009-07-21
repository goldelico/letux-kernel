/*
 *  gatepeterson_ioctl.h
 *
 *  The Gate Peterson Algorithm for mutual exclusion of shared memory.
 *  Current implementation works for 2 processors.
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

#ifndef _GATEPETERSON_IOCTL_
#define _GATEPETERSON_IOCTL_

#include <linux/ioctl.h>
#include <linux/types.h>

#include <ipc_ioctl.h>
#include <gatepeterson.h>

enum CMD_GATEPETERSON {
	GATEPETERSON_GETCONFIG = GATEPETERSON_BASE_CMD,
	GATEPETERSON_SETUP,
	GATEPETERSON_DESTROY,
	GATEPETERSON_PARAMS_INIT,
	GATEPETERSON_CREATE,
	GATEPETERSON_DELETE,
	GATEPETERSON_OPEN,
	GATEPETERSON_CLOSE,
	GATEPETERSON_ENTER,
	GATEPETERSON_LEAVE,
	GATEPETERSON_SHAREDMEMREQ
};

/*
 *  IOCTL command IDs for gatepeterson
 */

/*
 *  Command for gatepeterson_get_config
 */
#define CMD_GATEPETERSON_GETCONFIG	_IOWR(IPC_IOC_MAGIC,                   \
					GATEPETERSON_GETCONFIG,                \
					struct gatepeterson_cmd_args)

/*
 *  Command for gatepeterson_setup
 */
#define CMD_GATEPETERSON_SETUP		_IOWR(IPC_IOC_MAGIC,                   \
					GATEPETERSON_SETUP,                    \
					struct gatepeterson_cmd_args)

/*
 *  Command for gatepeterson_setup
 */
#define CMD_GATEPETERSON_DESTROY	_IOWR(IPC_IOC_MAGIC,                   \
					GATEPETERSON_DESTROY,                  \
					struct gatepeterson_cmd_args)

/*
 *  Command for gatepeterson_destroy
 */
#define CMD_GATEPETERSON_PARAMS_INIT	_IOWR(IPC_IOC_MAGIC,                   \
					GATEPETERSON_PARAMS_INIT,              \
					struct gatepeterson_cmd_args)

/*
 *  Command for gatepeterson_create
 */
#define CMD_GATEPETERSON_CREATE		_IOWR(IPC_IOC_MAGIC,                   \
					GATEPETERSON_CREATE,                   \
					struct gatepeterson_cmd_args)

/*
 *  Command for gatepeterson_delete
 */
#define CMD_GATEPETERSON_DELETE		_IOWR(IPC_IOC_MAGIC,                   \
					GATEPETERSON_DELETE,                   \
					struct gatepeterson_cmd_args)
/*
 *  Command for gatepeterson_open
 */
#define CMD_GATEPETERSON_OPEN		_IOWR(IPC_IOC_MAGIC,                   \
					GATEPETERSON_OPEN,                     \
					struct gatepeterson_cmd_args)

/*
 *  Command for gatepeterson_close
 */
#define CMD_GATEPETERSON_CLOSE		_IOWR(IPC_IOC_MAGIC,                   \
					GATEPETERSON_CLOSE,                    \
					struct gatepeterson_cmd_args)
/*
 *  Command for gatepeterson_enter
 */
#define CMD_GATEPETERSON_ENTER		_IOWR(IPC_IOC_MAGIC,                   \
					GATEPETERSON_ENTER,                    \
					struct gatepeterson_cmd_args)

/*
 *  Command for gatepeterson_leave
 */
#define CMD_GATEPETERSON_LEAVE		_IOWR(IPC_IOC_MAGIC,                   \
					GATEPETERSON_LEAVE,                    \
					struct gatepeterson_cmd_args)

/*
 *  Command for gatepeterson_shared_memreq
 */
#define CMD_GATEPETERSON_SHAREDMEMREQ	_IOWR(IPC_IOC_MAGIC,                   \
					GATEPETERSON_SHAREDMEMREQ,             \
					struct gatepeterson_cmd_args)

/*
 *  Command arguments for gatepeterson
 */
union gatepeterson_arg {
	struct {
		void *handle;
		struct gatepeterson_params *params;
	} params_init;

	struct {
		struct gatepeterson_config *config;
	} get_config;

	struct {
		struct gatepeterson_config *config;
	} setup;

	struct {
		void *handle;
		struct gatepeterson_params *params;
		u32 name_len;
		u32 shared_addr_srptr;
	} create;

	struct {
		void *handle;
	} delete;

	struct {
		void *handle;
		struct gatepeterson_params *params;
		u32 name_len;
		u32 shared_addr_srptr;
	} open;

	struct {
		void *handle;
	} close;

	struct {
		void *handle;
		u32 flags;
	} enter;

	struct {
		void *handle;
		u32 flags;
	} leave;

	struct {
		void *handle;
		struct gatepeterson_params *params;
		u32 bytes;
	} shared_memreq;

};

/*
 *  Command arguments for gatepeterson
 */
struct gatepeterson_cmd_args {
	union gatepeterson_arg args;
	s32 api_status;
};

/*
 *  This ioctl interface for gatepeterson module
 */
int gatepeterson_ioctl(struct inode *inode, struct file *filp,
			unsigned int cmd, unsigned long args);

#endif /* _GATEPETERSON_IOCTL_ */

