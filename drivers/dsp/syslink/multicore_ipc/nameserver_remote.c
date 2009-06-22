/*
 *  nameserver_remote.c
 *
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

#include <linux/types.h>
#include <linux/slab.h>

#include <gt.h>
#include <nameserver_remote.h>

/*
 * ======== nameserver_remote_get ========
 *  Purpose:
 *  This will get data from remote name server
 */
int nameserver_remote_get(const struct nameserver_remote_object *handle,
				const char *instance_name, const char *name,
				void *value, u32 value_len)
{
	s32 retval = 0;


	gt_0trace(ns_debugmask, GT_ENTER, "nameserver_remote_get\n");
	if (handle == NULL || instance_name == NULL ||
				name == NULL || value == NULL) {
		gt_5trace(ns_debugmask, GT_6CLASS,
			"nameserver_remote_get: invalid argument!\n"
			"instance_name: %s,\n name: %s,\n"
			"handle: %x, value: %x, value_len: %x\n",
			instance_name, name, handle, value, value_len);
		retval = -EINVAL;
		goto exit;
	}

	retval = handle->get(handle, instance_name, name, value, value_len);

exit:
	return retval;
}
