/*
 * hw_defs.h
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

#ifndef __HW_DEFS_H
#define __HW_DEFS_H

#include <syslink/GlobalTypes.h>

/* Page size */
#define HW_PAGE_SIZE_4KB   0x1000
#define HW_PAGE_SIZE_64KB  0x10000
#define HW_PAGE_SIZE_1MB   0x100000
#define HW_PAGE_SIZE_16MB  0x1000000

/* hw_status:  return type for HW API */
typedef long hw_status;

/* hw_set_clear_t:  Enumerated Type used to set and clear any bit */
enum hw_set_clear_t {
	HW_CLEAR,
	HW_SET
} ;

/* hw_endianism_t:  Enumerated Type used to specify the endianism
 *		Do NOT change these values. They are used as bit fields. */
enum hw_endianism_t {
	HW_LITTLE_ENDIAN,
	HW_BIG_ENDIAN

} ;

/* hw_elemnt_siz_t:  Enumerated Type used to specify the element size
 *		Do NOT change these values. They are used as bit fields. */
enum hw_elemnt_siz_t {
	HW_ELEM_SIZE_8BIT,
	HW_ELEM_SIZE_16BIT,
	HW_ELEM_SIZE_32BIT,
	HW_ELEM_SIZE_64BIT

} ;

/* HW_IdleMode_t:  Enumerated Type used to specify Idle modes */
enum HW_IdleMode_t {
	HW_FORCE_IDLE,
	HW_NO_IDLE,
	HW_SMART_IDLE
} ;

#endif  /* __HW_DEFS_H */
