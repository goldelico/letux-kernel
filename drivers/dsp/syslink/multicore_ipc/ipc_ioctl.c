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
#include <heapbuf_ioctl.h>
#include <sharedregion_ioctl.h>
#include <gatepeterson_ioctl.h>
#include <listmp_sharedmemory_ioctl.h>
#include <messageq_ioctl.h>
#include <messageq_transportshm_ioctl.h>
#include <nameserver_remotenotify_ioctl.h>
#include <sysmgr_ioctl.h>
#include <sysmemmgr_ioctl.h>

/*
 * ======== ipc_ioctl_router ========
 *  Purpose:
 *  This will route the ioctl commands to proper
 *  modules
 */
int ipc_ioc_router(u32 cmd, ulong arg)
{
	s32 retval = 0;
	u32 ioc_nr = _IOC_NR(cmd);

	if (ioc_nr >= MULTIPROC_BASE_CMD && ioc_nr <= MULTIPROC_END_CMD)
		retval = multiproc_ioctl(NULL, NULL, cmd, arg);
	else if (ioc_nr >= NAMESERVER_BASE_CMD &&
					ioc_nr <= NAMESERVER_END_CMD)
		retval = nameserver_ioctl(NULL, NULL, cmd, arg);
	else if (ioc_nr >= HEAPBUF_BASE_CMD && ioc_nr <= HEAPBUF_END_CMD)
		retval = heapbuf_ioctl(NULL, NULL, cmd, arg);
	else if (ioc_nr >= SHAREDREGION_BASE_CMD &&
					ioc_nr <= SHAREDREGION_END_CMD)
		retval = sharedregion_ioctl(NULL, NULL, cmd, arg);
	else if (ioc_nr >= GATEPETERSON_BASE_CMD &&
					ioc_nr <= GATEPETERSON_END_CMD)
		retval = gatepeterson_ioctl(NULL, NULL, cmd, arg);
	else if (ioc_nr >= LISTMP_SHAREDMEMORY_BASE_CMD &&
					ioc_nr <= LISTMP_SHAREDMEMORY_END_CMD)
		retval = listmp_sharedmemory_ioctl(NULL, NULL, cmd, arg);
	else if (ioc_nr >= MESSAGEQ_BASE_CMD &&
					ioc_nr <= MESSAGEQ_END_CMD)
		retval = messageq_ioctl(NULL, NULL, cmd, arg);
	else if (ioc_nr >= MESSAGEQ_TRANSPORTSHM_BASE_CMD &&
					ioc_nr <= MESSAGEQ_TRANSPORTSHM_END_CMD)
		retval = messageq_transportshm_ioctl(NULL, NULL, cmd, arg);
	else if (ioc_nr >= NAMESERVERREMOTENOTIFY_BASE_CMD &&
				ioc_nr <= NAMESERVERREMOTENOTIFY_END_CMD)
		retval = nameserver_remotenotify_ioctl(NULL, NULL, cmd, arg);
	else if (ioc_nr >= SYSMGR_BASE_CMD &&
				ioc_nr <= SYSMGR_END_CMD)
		retval = sysmgr_ioctl(NULL, NULL, cmd, arg);
	else if (ioc_nr >= SYSMEMMGR_BASE_CMD &&
				ioc_nr <= SYSMEMMGR_END_CMD)
		retval = sysmemmgr_ioctl(NULL, NULL, cmd, arg);
	else
		retval = -ENOTTY;

	return retval;
}
