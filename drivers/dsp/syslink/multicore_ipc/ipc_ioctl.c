/*
 *  ipc_ioctl.c
 *
 *  This is the collection of ioctl functions that will invoke various ipc
 *  module level functions based on user comands
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

#include <linux/uaccess.h>

#include <ipc_ioctl.h>
#include <multiproc_ioctl.h>
#include <nameserver_ioctl.h>
#include <heapbufmp_ioctl.h>
#include <sharedregion_ioctl.h>
#include <gatemp_ioctl.h>
#include <listmp_ioctl.h>
#include <messageq_ioctl.h>
#include <sysipc_ioctl.h>
/*#include <sysmemmgr_ioctl.h>*/
#include <heapmemmp_ioctl.h>
#include <drv_notify.h>

/*
 *  This will route the ioctl commands to proper modules
 */
int ipc_ioc_router(u32 cmd, ulong arg)
{
	s32 retval = 0;
	u32 ioc_nr = _IOC_NR(cmd);

	if (ioc_nr >= MULTIPROC_BASE_CMD && ioc_nr <= MULTIPROC_END_CMD)
		retval = multiproc_ioctl(NULL, NULL, cmd, arg);
	else if (ioc_nr >= NAMESERVER_BASE_CMD && ioc_nr <= NAMESERVER_END_CMD)
		retval = nameserver_ioctl(NULL, NULL, cmd, arg);
	else if (ioc_nr >= HEAPBUFMP_BASE_CMD && ioc_nr <= HEAPBUFMP_END_CMD)
		retval = heapbufmp_ioctl(NULL, NULL, cmd, arg);
	else if (ioc_nr >= SHAREDREGION_BASE_CMD &&
					ioc_nr <= SHAREDREGION_END_CMD)
		retval = sharedregion_ioctl(NULL, NULL, cmd, arg);
	else if (ioc_nr >= GATEMP_BASE_CMD && ioc_nr <= GATEMP_END_CMD)
		retval = gatemp_ioctl(NULL, NULL, cmd, arg);
	else if (ioc_nr >= LISTMP_BASE_CMD && ioc_nr <= LISTMP_END_CMD)
		retval = listmp_ioctl(NULL, NULL, cmd, arg);
	else if (ioc_nr >= MESSAGEQ_BASE_CMD && ioc_nr <= MESSAGEQ_END_CMD)
		retval = messageq_ioctl(NULL, NULL, cmd, arg);
	else if (ioc_nr >= IPC_BASE_CMD && ioc_nr <= IPC_END_CMD)
		retval = sysipc_ioctl(NULL, NULL, cmd, arg);
/*	else if (ioc_nr >= SYSMEMMGR_BASE_CMD && ioc_nr <= SYSMEMMGR_END_CMD)
		retval = sysmemmgr_ioctl(NULL, NULL, cmd, arg);*/
	else if (ioc_nr >= HEAPMEMMP_BASE_CMD && ioc_nr <= HEAPMEMMP_END_CMD)
		retval = heapmemmp_ioctl(NULL, NULL, cmd, arg);
	else if (ioc_nr >= NOTIFY_BASE_CMD && ioc_nr <= NOTIFY_END_CMD)
		retval = notify_drv_ioctl(NULL, NULL, cmd, arg);
	else
		retval = -ENOTTY;

	return retval;
}
