/*
 * dmm.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */


/*
 *  ======== dmm.h ========
 *  Purpose:
 *The Dynamic Memory Mapping(DMM) module manages the DSP Virtual address
 *space that can be directly mapped to any MPU buffer or memory region
 *
 *  Public Functions:
 *
 */

#ifndef DMM_4430_
#define DMM_4430_

#include <linux/types.h>

int dmm_reserve_memory(u32 size, u32 *p_rsv_addr);

int dmm_unreserve_memory(u32 rsv_addr, u32 *psize);

void dmm_destroy(void);

void dmm_delete_tables(void);

int dmm_create(void);

void dmm_init(void);

int dmm_create_tables(u32 addr, u32 size);

#ifdef DSP_DMM_DEBUG
int dmm_mem_map_dump(void);
#endif
#endif/* DMM_4430_ */
