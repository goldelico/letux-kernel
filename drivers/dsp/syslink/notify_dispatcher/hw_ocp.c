/*
 * hw_ocp.c
 *
 * Syslink dispatcher driver support for OMAP Processors.
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

#include <syslink/GlobalTypes.h>
#include <syslink/hw_ocp.h>
#include <syslink/hw_defs.h>
#include <syslink/MBXRegAcM.h>
#include <syslink/MBXAccInt.h>
#include <linux/module.h>


long hw_ocp_soft_reset(const unsigned long base_address)
{
	long status = RET_OK;
	MLBMAILBOX_SYSCONFIGSoftResetWrite32(base_address, HW_SET);
	return status;
}

long hw_ocp_soft_reset_isdone(const unsigned long base_address,
					unsigned long *reset_is_done)
{
	long status = RET_OK;

	*reset_is_done = MLBMAILBOX_SYSSTATUSResetDoneRead32(base_address);

	return status;
}

long hw_ocp_idle_modeset(const unsigned long base_address,
			enum hal_ocp_idlemode_t idle_mode)
{
	long status = RET_OK;

	MLBMAILBOX_SYSCONFIGSIdleModeWrite32(base_address, idle_mode);

	return status;
}


long hw_ocp_idlemode_get(const unsigned long base_address,
			enum hal_ocp_idlemode_t *idle_mode)
{
	long status = RET_OK;

	*idle_mode = (enum hal_ocp_idlemode_t)
			MLBMAILBOX_SYSCONFIGSIdleModeRead32(base_address);

	return status;
}

long hw_ocp_autoidle_set(const unsigned long base_address,
				enum hw_set_clear_t auto_idle)
{
	long status = RET_OK;

	MLBMAILBOX_SYSCONFIGAutoIdleWrite32(base_address, auto_idle);

	return status;
}

long hw_ocp_autoidle_get(const unsigned long base_address,
				enum hw_set_clear_t *auto_idle)
{
	long status = RET_OK;

	*auto_idle = (enum hw_set_clear_t)
			MLBMAILBOX_SYSCONFIGAutoIdleRead32(base_address);

	return status;
}

