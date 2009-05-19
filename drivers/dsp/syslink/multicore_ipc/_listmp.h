/*
 *  _listmp.h
 *
 *  Internal definitions  for Internal Defines for shared memory
 *              doubly linked list.
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

#ifndef __LISTMP_H_
#define __LISTMP_H_

/* Standard headers */
#include <linux/types.h>

/*!
 *  @brief  Structure defining attribute parameters for the
 *          ListMP module.
 */
struct listmp_attrs {
	u32 version;
	/*!< Version of module */
	u32 status;
	/*!< Status of module */
	u32 shared_addr_size;
	/*!< Shared address size of module */
};

/*!
 *  @brief  Structure defining processor related information for the
 *          ListMP module.
 */
struct listmp_proc_attrs {
	bool creator;   /*!< Creator or opener */
	u16 proc_id;    /*!< Processor Identifier */
	u32 open_count; /*!< How many times it is opened on a processor */
};

#endif /* __LISTMP_H_ */
