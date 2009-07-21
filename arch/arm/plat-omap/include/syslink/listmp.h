/*
 *  listmp.h
 *
 *  The listmp module defines the shared memory doubly linked list.
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

#ifndef _LISTMP_H_
#define _LISTMP_H_

/* Standard headers */
#include <linux/types.h>

/* Utilities headers */
#include <linux/list.h>
/*#include <heap.h>*/

/* =============================================================================
 *  All success and failure codes for the module
 * =============================================================================
 */
/*!
 *  @def	LISTMP_MODULEID
 *  @brief  Unique module ID.
 */
#define LISTMP_MODULEID	(0xa413)

/*!
 *  @def	LISTMP_ERRORCODEBASE
 *  @brief  Error code base for ListMP.
 */
#define LISTMP_ERRORCODEBASE	(LISTMP_MODULEID << 12)

/*!
 *  @def	LISTMP_MAKE_FAILURE
 *  @brief  Macro to make error code.
 */
#define LISTMP_MAKE_FAILURE(x)	((int)(0x80000000 \
				 + (LISTMP_ERRORCODEBASE + (x))))

/*!
 *  @def	LISTMP_MAKE_SUCCESS
 *  @brief  Macro to make success code.
 */
#define LISTMP_MAKE_SUCCESS(x)	(LISTMP_ERRORCODEBASE + (x))

/*!
 *  @def	LISTMP_E_INVALIDARG
 *  @brief  Argument passed to a function is invalid.
 */
#define LISTMP_E_INVALIDARG	LISTMP_MAKE_FAILURE(1)

/*!
 *  @def	LISTMP_E_MEMORY
 *  @brief  Memory allocation failed.
 */
#define LISTMP_E_MEMORY	LISTMP_MAKE_FAILURE(2)

/*!
 *  @def	LISTMP_E_BUSY
 *  @brief  The name is already registered or not.
 */
#define LISTMP_E_BUSY		LISTMP_MAKE_FAILURE(3)

/*!
 *  @def	LISTMP_E_FAIL
 *  @brief  Generic failure.
 */
#define LISTMP_E_FAIL		LISTMP_MAKE_FAILURE(4)

/*!
 *  @def	LISTMP_E_NOTFOUND
 *  @brief  Name not found in the nameserver.
 */
#define LISTMP_E_NOTFOUND	LISTMP_MAKE_FAILURE(5)

/*!
 *  @def	LISTMP_E_INVALIDSTATE
 *  @brief  Module is not initialized.
 */
#define LISTMP_E_INVALIDSTATE	LISTMP_MAKE_FAILURE(6)

/*!
 *  @def	LISTMP_E_OSFAILURE
 *  @brief  Failure in OS call.
 */
#define LISTMP_E_OSFAILURE	LISTMP_MAKE_FAILURE(7)

/*!
 *  @def	LISTMP_SUCCESS
 *  @brief  Operation successful.
 */
#define LISTMP_SUCCESS		LISTMP_MAKE_SUCCESS(0)

/*!
 *  @def	LISTMP_S_ALREADYSETUP
 *  @brief  The LISTMP module has already been setup in this process.
 */
#define LISTMP_S_ALREADYSETUP	LISTMP_MAKE_SUCCESS(1)

/* =============================================================================
 *  Macros and types
 * =============================================================================
 */
/*!
 *  @brief  Enum defining types of list for the ListMP module.
 */
enum listmp_type {
	listmp_type_SHARED = 0,
	/*!< List in shared memory */
	listmp_type_FAST = 1
	/*!< Hardware Queue */
};

/*!
 *  @brief  Structure defining config parameters for the ListMP module.
 */
struct listmp_config {
	u32 max_name_len;
	/*!< Maximum length of name */
	bool use_name_server;
	/*!< Whether to have this module use the NameServer or not. If the
	*   NameServer is not needed, set this configuration parameter to false.
	*   This informs ListMPSharedMemory not to pull in the NameServer module
	*   In this case, all names passed into create and open are ignored.
	*/
};

/*!
 *  @brief  Structure defining list element for the ListMP.
 */
struct listmp_elem {
	volatile struct listmp_elem *next;
	volatile struct listmp_elem *prev;
};

/*!
 *  @brief  Structure defining config parameters for the ListMP instances.
 */
struct listmp_params {
	bool cache_flag;
	/*!< Set to 1 by the open() call. No one else should touch this!   */
	struct mutex *gate;
	/*!< Lock used for critical region management of the list */
	void *shared_addr;
	/*!< shared memory address */
	u32 shared_addr_size;
	/*!< shared memory size */
	char *name;
	/*!< Name of the object	*/
	int resource_id;
	/*!<
	*  resourceId  Specifies the resource id number.
	*  Parameter is used only when type is set to Fast List
	*/
	enum listmp_type list_type ;
	/*!< Type of list */
};


/* =============================================================================
 *  Forward declarations
 * =============================================================================
 */
/*!
 *  @brief  Structure defining config parameters for the ListMPSharedMemory.
 */
struct listmp_object {
	bool (*empty)(void *listmp_handle);
	/* Function to check if list is empty */
	void *(*get_head)(void *listmp_handle);
	/* Function to get head element from list */
	void *(*get_tail)(void *listmp_handle);
	/* Function to get tail element from list */
	int (*put_head)(void *listmp_handle, struct listmp_elem *elem);
	/* Function to put head element into list */
	int (*put_tail)(void *listmp_handle, struct listmp_elem *elem);
	/* Function to put tail element into list */
	int (*insert)(void *listmp_handle, struct listmp_elem *elem,
					struct listmp_elem *curElem);
	/* Function to insert element into list */
	int (*remove)(void *listmp_handle, struct listmp_elem *elem);
	/* Function to remove element from list */
	void *(*next)(void *listmp_handle, struct listmp_elem *elem);
	/* Function to traverse to next element in list */
	void *(*prev)(void *listmp_handle, struct listmp_elem *elem);
	/* Function to traverse to prev element in list */
	void *obj;
	/*!< Handle to ListMP */
	enum listmp_type list_type;
	/* Type of list */
};

/*
 *  Function initializes listmp parameters
 */
void listmp_params_init(void *listmp_handle,
						struct listmp_params *params);

/*
 *  Function to get shared memory requirement for the module
 */
int listmp_shared_memreq(struct listmp_params *params);

/* =============================================================================
 *  Functions to create instance of a list
 * =============================================================================
 */
/* Function to create an instance of ListMP */
void *listmp_create(struct listmp_params *params);

/* Function to delete an instance of ListMP */
int listmp_delete(void **listmp_handle_ptr);

/* =============================================================================
 *  Functions to open/close handle to list instance
 * =============================================================================
 */
/* Function to open a previously created instance */
int listmp_open(void **listmp_handle_ptr, struct listmp_params *params);

/* Function to close a previously opened instance */
int listmp_close(void *listmp_handle);

/* =============================================================================
 *  Function pointer types for list operations
 * =============================================================================
 */
/* Function to check if list is empty */
bool listmp_empty(void *listmp_handle);

/* Function to get head element from list */
void *listmp_get_head(void *listmp_handle);

/* Function to get tail element from list */
void *listmp_get_tail(void *listmp_handle);

/* Function to put head element into list */
int listmp_put_head(void *listmp_handle, struct listmp_elem *elem);

/* Function to put tail element into list */
int listmp_put_tail(void *listmp_handle, struct listmp_elem *elem);

/* Function to insert element into list */
int listmp_insert(void *listmp_handle, struct listmp_elem *elem,
			struct listmp_elem *curElem);

/* Function to traverse to remove element from list */
int listmp_remove(void *listmp_handle, struct listmp_elem *elem);

/* Function to traverse to next element in list */
void *listmp_next(void *listmp_handle, struct listmp_elem *elem);

/* Function to traverse to prev element in list */
void *listmp_prev(void *listmp_handle, struct listmp_elem *elem);

#endif /* _LISTMP_H_ */
