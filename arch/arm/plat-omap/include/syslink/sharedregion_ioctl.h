/*
 *  sharedregion_ioctl.h
 *
 *  The sharedregion module is designed to be used in a
 *  multi-processor environment where there are memory regions
 *  that are shared and accessed across different processors
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

#ifndef _SHAREDREGION_IOCTL_H
#define _SHAREDREGION_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>

#include <ipc_ioctl.h>
#include <sharedregion.h>

enum CMD_SHAREDREGION {
	SHAREDREGION_GETCONFIG = SHAREDREGION_BASE_CMD,
	SHAREDREGION_SETUP,
	SHAREDREGION_DESTROY,
	SHAREDREGION_ADD,
	SHAREDREGION_GETPTR,
	SHAREDREGION_GETSRPTR,
	SHAREDREGION_GETTABLEINFO,
	SHAREDREGION_REMOVE,
	SHAREDREGION_SETTABLEINFO,
	SHAREDREGION_GETINDEX,
};

/*
 *  IOCTL command IDs for sharedregion
 *
 */

/*
 *  Command for sharedregion_get_config
 */
#define CMD_SHAREDREGION_GETCONFIG	_IOWR(IPC_IOC_MAGIC,                   \
					SHAREDREGION_GETCONFIG,		       \
					struct sharedregion_cmd_args)
/*
 *  Command for sharedregion_setup
 */
#define CMD_SHAREDREGION_SETUP		_IOWR(IPC_IOC_MAGIC,                   \
					SHAREDREGION_SETUP,		       \
					struct sharedregion_cmd_args)
/*
 *  Command for sharedregion_setup
 */
#define CMD_SHAREDREGION_DESTROY	_IOWR(IPC_IOC_MAGIC,                   \
					SHAREDREGION_DESTROY, 		       \
					struct sharedregion_cmd_args)
/*
 *  Command for sharedregion_ADD
 */
#define CMD_SHAREDREGION_ADD		_IOWR(IPC_IOC_MAGIC,                   \
					SHAREDREGION_ADD,		       \
					struct sharedregion_cmd_args)
/*
 *  Command for sharedregion_get_ptr
 */
#define CMD_SHAREDREGION_GETPTR		_IOWR(IPC_IOC_MAGIC,                   \
					SHAREDREGION_GETPTR,		       \
					struct sharedregion_cmd_args)

/*
 *  Command for sharedregion_get_srptr
 */
#define CMD_SHAREDREGION_GETSRPTR	_IOWR(IPC_IOC_MAGIC,                   \
					SHAREDREGION_GETSRPTR,		       \
					struct sharedregion_cmd_args)

/*
 *  Command for sharedregion_get_table_info
 */
#define CMD_SHAREDREGION_GETTABLEINFO	_IOWR(IPC_IOC_MAGIC,                   \
					SHAREDREGION_GETTABLEINFO,	       \
					struct sharedregion_cmd_args)

/*
 *  Command for sharedregion_remove
 */
#define CMD_SHAREDREGION_REMOVE		_IOWR(IPC_IOC_MAGIC,                   \
					SHAREDREGION_REMOVE,		       \
					struct sharedregion_cmd_args)
/*
 *  Command for sharedregion_set_table_info
 */
#define CMD_SHAREDREGION_SETTABLEINFO	_IOWR(IPC_IOC_MAGIC,                   \
					SHAREDREGION_SETTABLEINFO,	       \
					struct sharedregion_cmd_args)

/*
 *  Command for sharedregion_get_index
 */
#define CMD_SHAREDREGION_GETINDEX	_IOWR(IPC_IOC_MAGIC,                   \
					SHAREDREGION_GETINDEX,		       \
					struct sharedregion_cmd_args)

/*
 *  Command arguments for sharedregion
 */
union sharedregion_arg {
	struct {
	    struct sharedregion_config *config;
	} get_config;

	struct {
	    struct sharedregion_config *config;
	    struct sharedregion_config *default_cfg;
	    struct sharedregion_info *table;
	} setup;

	struct {
	    u32 index;
	    void *base;
	    u32	len;
	} add;

	struct {
	    void *addr;
	    s32 index;
	} get_index;

	struct {
	    u32 *srptr;
	    void *addr;
	} get_ptr;

	struct {
	    u32 *srptr;
	    void *addr;
	    s32 index;
	} get_srptr;

	struct {
	    u32 index;
	    u16 proc_id;
	    struct sharedregion_info *info;
	} get_table_info;

	struct {
	    u32 index;
	} remove;

	struct {
	    u32	index;
	    u16 proc_id;
	    struct sharedregion_info *info;
	} set_table_info;
} ;

/*
 *  Command arguments for sharedregion
 */
struct sharedregion_cmd_args {
	union sharedregion_arg args;
	s32 api_status;
};

/*
 *  This ioctl interface for sharedregion module
 */
int sharedregion_ioctl(struct inode *inode, struct file *filp,
			unsigned int cmd, unsigned long args);

#endif /* _SHAREDREGION_IOCTL_H */


