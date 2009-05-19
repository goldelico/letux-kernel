
/*
 * list.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
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
 *  ======== list.h ========
 *  Purpose:
 *      Declarations of list management control structures and definitions
 *      of inline list management functions.
 *
 *  Public Functions:
 *
 *  Notes:
 *
 *! Revision History
 *! ================
 */

#ifndef NOTIFY_LIST
#define NOTIFY_LIST
#include<linux/list.h>
#include <syslink/host_os.h>


struct lst_list {
	struct list_head head;
} ;


/** ==========================================================================
 *  @name   ListMatchFunc
 *
 *  @desc   Signature of the Matching function to be used by search algo.
 *
 *  @arg    elem
 *              Element to be compared.
 *  @arg    data
 *              Comparing key data.
 *
 *  @ret    TRUE or FALSE.
 *
 *  @enter  None.
 *
 *  @leave  None.
 *
 *  @see    None.
 *  ==========================================================================*/

typedef bool (*ListMatchFunc)(struct list_head *elem, void  *data);



extern signed long int omap_list_search(struct lst_list *list,
					void  *data,
					struct list_head **elem,
					 ListMatchFunc  matchFunc);

#endif				/* NOTIFY_LIST */

