/*
 *  platformcfg.c
 *
 *  Implementation of platform specific configuration for Syslink.
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


/* Standard headers */
#include <linux/types.h>
#include <linux/module.h>

/* Utilities headers */
#include <linux/string.h>


/* Module headers */
#include <sysmgr.h>

/* =============================================================================
 * APIS
 * =============================================================================
 */
/*
 * ======== platform_override_config ========
 *  Purpose:
 *  Function to override the default confiuration values.
 */
int platform_override_config(struct sysmgr_config *config)
{
	int status = 0;

	if (WARN_ON(config == NULL)) {
		status = -EINVAL;
		goto failure;
	}

	/* Override the multiproc default config */
	config->multiproc_cfg.max_processors = 4;
	config->multiproc_cfg.id = 0;
	strcpy(config->multiproc_cfg.name_list[0], "MPU");
	strcpy(config->multiproc_cfg.name_list[1], "Tesla");
	strcpy(config->multiproc_cfg.name_list[2], "SysM3");
	strcpy(config->multiproc_cfg.name_list[3], "AppM3");

	/* Override the gatepeterson default config */

	/* Override the sharedregion default config */
	config->sharedregion_cfg.gate_handle = NULL;
	config->sharedregion_cfg.heap_handle = NULL;
	config->sharedregion_cfg.max_regions = 4;

	/* Override the listmp default config */

	/* Override the messageq default config */
	/* We use 2 heaps, 1 for APPM3 and 1 for SYSM3 */
	/* FIXME: Temporary Fix - Add one more for the SW DMM heap */
	if (config->messageq_cfg.num_heaps < 3)
		config->messageq_cfg.num_heaps = 3;

	/* Override the notify default config */
	config->notify_cfg.maxDrivers = 2;

	/* Override the procmgr default config */

	/* Override the heapbuf default config */

	/* Override the listmp_sharedmemory default config */

	/* Override the messageq_transportshm default config */

	/* Override the notify ducati driver default config */

	/* Override the nameserver remotenotify default config */
	goto success;

failure:
	printk(KERN_ERR "platform_override_config failed [0x%x]", status);
success:
	return status;
}
