/*
 * hw_ocp.h
 *
 * Syslink driver support for OMAP Processors.
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


#ifndef __HW_OCP_H
#define __HW_OCP_H

#include <syslink/GlobalTypes.h>
#include <syslink/hw_ocp.h>
#include <syslink/hw_defs.h>
#include <syslink/MBXRegAcM.h>
#include <syslink/MBXAccInt.h>


/*
* TYPE:         HW_IdleMode_t
*
* DESCRIPTION:  Enumerated Type for idle modes in OCP SYSCONFIG register
*
*
*/
enum hal_ocp_idlemode_t {
	HW_OCP_FORCE_IDLE,
	HW_OCP_NO_IDLE,
	HW_OCP_SMART_IDLE
};

extern long hw_ocp_soft_reset(const unsigned long base_address);

extern long hw_ocp_soft_reset_isdone(const unsigned long base_address,
					     unsigned long *reset_is_done);

extern long hw_ocp_idle_modeset(const unsigned long base_address,
					enum hal_ocp_idlemode_t idle_mode);

extern long hw_ocp_idlemode_get(const unsigned long base_address,
					enum hal_ocp_idlemode_t *idle_mode);

extern long hw_ocp_autoidle_set(const unsigned long base_address,
					enum hw_set_clear_t auto_idle);

extern long hw_ocp_autoidle_get(const unsigned long base_address,
					enum hw_set_clear_t *auto_idle);

#endif  /* __HW_OCP_H */

