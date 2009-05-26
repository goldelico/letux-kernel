/*
 *  nameserver_remotenotify_ioctl.h
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

#ifndef _NAMESERVERREMOTENOTIFY_DRVDEFS_H
#define _NAMESERVERREMOTENOTIFY_DRVDEFS_H


#include <linux/ioctl.h>
#include <linux/types.h>

#include <ipc_ioctl.h>
#include <nameserver_remotenotify.h>

enum CMD_NAMESERVERREMOTENOTIFY {
	NAMESERVERREMOTENOTIFY_GETCONFIG = NAMESERVERREMOTENOTIFY_BASE_CMD,
	NAMESERVERREMOTENOTIFY_SETUP,
	NAMESERVERREMOTENOTIFY_DESTROY,
	NAMESERVERREMOTENOTIFY_PARAMS_INIT,
	NAMESERVERREMOTENOTIFY_CREATE,
	NAMESERVERREMOTENOTIFY_DELETE,
	NAMESERVERREMOTENOTIFY_GET,
	NAMESERVERREMOTENOTIFY_SHAREDMEMREQ
};


/*
 *  IOCTL command IDs for nameserver_remotenotify
 *
 */

/*
 *  Command for nameserver_remotenotify_get_config
 */
#define CMD_NAMESERVERREMOTENOTIFY_GETCONFIG 	_IOWR(IPC_IOC_MAGIC,	       \
					NAMESERVERREMOTENOTIFY_GETCONFIG,      \
					struct nameserver_remotenotify_cmd_args)
/*
 *  Command for nameserver_remotenotify_setup
 */
#define CMD_NAMESERVERREMOTENOTIFY_SETUP	_IOWR(IPC_IOC_MAGIC,	       \
					NAMESERVERREMOTENOTIFY_SETUP,	       \
					struct nameserver_remotenotify_cmd_args)

/*
 *  Command for nameserver_remotenotify_setup
 */
#define CMD_NAMESERVERREMOTENOTIFY_DESTROY	_IOWR(IPC_IOC_MAGIC,	       \
					NAMESERVERREMOTENOTIFY_DESTROY,	       \
					struct nameserver_remotenotify_cmd_args)

/*
 *  Command for nameserver_remotenotify_destroy
 */
#define CMD_NAMESERVERREMOTENOTIFY_PARAMS_INIT	_IOWR(IPC_IOC_MAGIC,	       \
					NAMESERVERREMOTENOTIFY_PARAMS_INIT,    \
					struct nameserver_remotenotify_cmd_args)

/*
 * Command for nameserver_remotenotify_create
 */
#define CMD_NAMESERVERREMOTENOTIFY_CREATE	_IOWR(IPC_IOC_MAGIC,	       \
					NAMESERVERREMOTENOTIFY_CREATE,	       \
					struct nameserver_remotenotify_cmd_args)

/*
 *  Command for nameserver_remotenotify_delete
 */
#define CMD_NAMESERVERREMOTENOTIFY_DELETE	_IOWR(IPC_IOC_MAGIC,	       \
					NAMESERVERREMOTENOTIFY_DELETE,	       \
					struct nameserver_remotenotify_cmd_args)

/*
 *  Command for nameserver_remotenotify_get
 */
#define CMD_NAMESERVERREMOTENOTIFY_GET 	_IOWR(IPC_IOC_MAGIC,		       \
					NAMESERVERREMOTENOTIFY_GET,	       \
					struct nameserver_remotenotify_cmd_args)

/*
 *  Command for nameserver_remotenotify_shared_memreq
 */
#define CMD_NAMESERVERREMOTENOTIFY_SHAREDMEMREQ	_IOWR(IPC_IOC_MAGIC,	       \
					NAMESERVERREMOTENOTIFY_SHAREDMEMREQ,   \
					struct nameserver_remotenotify_cmd_args)

/*
 *  Command arguments for nameserver_remotenotify
 */
union nameserver_remotenotify_arg {
	struct {
		struct nameserver_remotenotify_config *config;
	} get_config;

	struct {
		struct nameserver_remotenotify_config *config;
	} setup;

	struct {
		void *handle;
		struct nameserver_remotenotify_params *params;
	} params_init;

	struct {
		void *handle;
		u16 proc_id;
		struct nameserver_remotenotify_params *params;
	} create;

	struct {
		void *handle;
	} delete_instance;

	struct {
		void *handle;
		char *instance_name;
		u32 instance_name_len;
		char *name;
		u32 name_len;
		u8 *value;
		s32 value_len;
		void *reserved;
		s32 len;
	} get;

	struct {
		void *handle;
		struct nameserver_remotenotify_params *params;
		u32 shared_mem_size;
	} shared_memreq;
};

/*
 *  Command arguments for nameserver_remotenotify
 */
struct nameserver_remotenotify_cmd_args {
	union nameserver_remotenotify_arg args;
	s32 api_status;
};

/*
 *  This ioctl interface for nameserver_remotenotify module
 */
int nameserver_remotenotify_ioctl(struct inode *inode, struct file *filp,
			unsigned int cmd, unsigned long args);


#endif /* _NAMESERVERREMOTENOTIFY_DRVDEFS_H */

