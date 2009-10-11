/*
 *  sysmgr_ioctl.h
 *
 *  Definitions of sysmgr driver types and structures..
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

#ifndef _SYSMGR_IOCTL_H_
#define _SYSMGR_IOCTL_H_

/* Standard headers */
#include <linux/types.h>

/* Syslink headers */
#include <ipc_ioctl.h>
#include <sysmgr.h>


/* =============================================================================
 *  Macros and types
 * =============================================================================
 */
/*  ----------------------------------------------------------------------------
 *  IOCTL command IDs for sysmgr
 *  ----------------------------------------------------------------------------
 */
/* IOC Magic Number for sysmgr */
#define SYSMGR_IOC_MAGIC		IPC_IOC_MAGIC

/* IOCTL command numbers for sysmgr */
enum sysmgr_drv_cmd {
	SYSMGR_SETUP = SYSMGR_BASE_CMD,
	SYSMGR_DESTROY,
	SYSMGR_LOADCALLBACK,
	SYSMGR_STARTCALLBACK,
	SYSMGR_STOPCALLBACK
};

/* Command for sysmgr_setup */
#define CMD_SYSMGR_SETUP \
			_IOWR(SYSMGR_IOC_MAGIC, SYSMGR_SETUP, \
			struct sysmgr_cmd_args)

/* Command for sysmgr_destroy */
#define CMD_SYSMGR_DESTROY \
			_IOWR(SYSMGR_IOC_MAGIC, SYSMGR_DESTROY, \
			struct sysmgr_cmd_args)

/* Command for load callback */
#define CMD_SYSMGR_LOADCALLBACK \
			_IOWR(SYSMGR_IOC_MAGIC, SYSMGR_LOADCALLBACK, \
			struct sysmgr_cmd_args)

/* Command for load callback */
#define CMD_SYSMGR_STARTCALLBACK \
			_IOWR(SYSMGR_IOC_MAGIC, SYSMGR_STARTCALLBACK, \
			struct sysmgr_cmd_args)

/* Command for stop callback */
#define CMD_SYSMGR_STOPCALLBACK \
			_IOWR(SYSMGR_IOC_MAGIC, SYSMGR_STOPCALLBACK, \
			struct sysmgr_cmd_args)


/*  ----------------------------------------------------------------------------
 *  Command arguments for sysmgr
 *  ----------------------------------------------------------------------------
 */
/* Command arguments for sysmgr */
struct sysmgr_cmd_args {
	union {
		struct {
			struct sysmgr_config *config;
		} setup;

		int proc_id;
	} args;

	s32 api_status;
};

/*  ----------------------------------------------------------------------------
 *  IOCTL functions for sysmgr module
 *  ----------------------------------------------------------------------------
 */
/* ioctl interface function for sysmgr */
int sysmgr_ioctl(struct inode *inode, struct file *filp,
				unsigned int cmd, unsigned long args);

#endif /* _SYSMGR_IOCTL_H_ */
