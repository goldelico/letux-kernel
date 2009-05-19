/*
*  multiproc_ioctl.h
*
*  This provides the ioctl interface for multiproc module
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

#ifndef _MULTIPROC_IOCTL_H_
#define _MULTIPROC_IOCTL_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#include <ipc_ioctl.h>
#include <multiproc.h>

enum CMD_MULTIPROC {
	MULTIPROC_GETID = MULTIPROC_BASE_CMD,
	MULTIPROC_GETNAME,
	MULTIPROC_GETMAXID,
};

/*
 *  Command for multiproc_get_id
 */
#define CMD_MULTIPROC_GETID	_IOWR(IPC_IOC_MAGIC, MULTIPROC_GETID,          \
				struct multiproc_cmd_args)

/*
 *  Command for multiproc_get_name
 */
#define CMD_MULTIPROC_GETNAME	_IOWR(IPC_IOC_MAGIC, MULTIPROC_GETNAME,        \
				struct multiproc_cmd_args)

/*
 *  Command for multiproc_get_max_processors
 */
#define CMD_MULTIPROC_GETMAXID _IOWR(IPC_IOC_MAGIC, MULTIPROC_GETMAXID,        \
				struct multiproc_cmd_args)


/*
 *  Command arguments for multiproc
 */
union multiproc_arg {

	struct {
		u16 *proc_id;
		char *name;
		u32 name_len;
	} get_id;

	struct {
		u16 proc_id;
		char *name;
	} get_name;

	struct {
		u16 *max_id;
	} get_max_id;
};

/*
 *  Command arguments for multiproc
 */
struct multiproc_cmd_args {
	union multiproc_arg cmd_arg;
	s32 api_status;
};

/*
 *  This ioctl interface for multiproc module
 */
int multiproc_ioctl(struct inode *inode, struct file *filp,
			unsigned int cmd, unsigned long args);

#endif	/* _MULTIPROC_IOCTL_H_ */
