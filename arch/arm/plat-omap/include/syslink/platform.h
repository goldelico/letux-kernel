/*
 *  platform.h
 *
 *  Defines the platform functions to be used by SysMgr module.
 *
 *  Copyright (C) 2009 Texas Instruments, Inc.
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

#ifndef _PLATFORM_H_
#define _PLATFORM_H_

/* Module headers */
#include <sysmgr.h>

/* =============================================================================
 * APIs
 * =============================================================================
 */
/* Function to setup the platform */
s32 platform_setup(struct sysmgr_config *config);

/* Function to destroy the platform */
s32 platform_destroy(void);

/* Function called when slave is loaded with executable */
void platform_load_callback(void *arg);

/* Function called when slave is in started state*/
void platform_start_callback(void *arg);

/* Function called when slave is stopped state */
void platform_stop_callback(void *arg);

s32 platform_override_config(struct sysmgr_config *config);

#endif /* ifndef _PLATFORM_H_ */
