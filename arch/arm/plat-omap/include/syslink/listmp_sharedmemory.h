/*
 *  listmp_sharedmemory.c
 *
 *  The listmp_sharedmemory is a double  linked-list based module designed to be
 *  used in a multi-processor environment.  It is designed to provide a means
 *  of communication between different processors.
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

#ifndef _LISTMP_SHAREDMEMORY_H_
#define _LISTMP_SHAREDMEMORY_H_

/* Standard headers */
#include <linux/types.h>

/* Utilities headers */

/* Other headers */
#include <listmp.h>

/* =============================================================================
 *  All success and failure codes for the module
 * =============================================================================
 */
/*!
 *  @def	LISTMPSHAREDMEMORY_MODULEID
 *  @brief  Unique module ID.
 */
#define LISTMPSHAREDMEMORY_MODULEID			(0xDD3C)

/*!
 *  @def	LISTMPSHAREDMEMORY_ERRORCODEBASE
 *  @brief  Error code base for ListMPSharedMemory.
 */
#define LISTMPSHAREDMEMORY_ERRORCODEBASE	\
	(LISTMPSHAREDMEMORY_MODULEID << 12)

/*!
 *  @def	LISTMPSHAREDMEMORY_MAKE_FAILURE
 *  @brief  Macro to make error code.
 */
#define LISTMPSHAREDMEMORY_MAKE_FAILURE(x)	\
	((int)  (0x80000000			\
	+ (LISTMPSHAREDMEMORY_ERRORCODEBASE	\
	 + (x))))

/*!
 *  @def	LISTMPSHAREDMEMORY_MAKE_SUCCESS
 *  @brief  Macro to make success code.
 */
#define LISTMPSHAREDMEMORY_MAKE_SUCCESS(x)	\
	(LISTMPSHAREDMEMORY_ERRORCODEBASE + (x))

/*!
 *  @def	LISTMPSHAREDMEMORY_E_INVALIDARG
 *  @brief  Argument passed to a function is invalid.
 */
#define LISTMPSHAREDMEMORY_E_INVALIDARG \
				LISTMPSHAREDMEMORY_MAKE_FAILURE(1)

/*!
 *  @def	LISTMPSHAREDMEMORY_E_MEMORY
 *  @brief  Memory allocation failed.
 */
#define LISTMPSHAREDMEMORY_E_MEMORY \
				LISTMPSHAREDMEMORY_MAKE_FAILURE(2)

/*!
 *  @def	LISTMPSHAREDMEMORY_E_FAIL
 *  @brief  Generic failure.
 */
#define LISTMPSHAREDMEMORY_E_FAIL \
				LISTMPSHAREDMEMORY_MAKE_FAILURE(3)

/*!
 *  @def	LISTMPSHAREDMEMORY_E_INVALIDSTATE
 *  @brief  Module is not initialized.
 */
#define LISTMPSHAREDMEMORY_E_INVALIDSTATE \
				LISTMPSHAREDMEMORY_MAKE_FAILURE(4)

/*!
 *  @def	LISTMPSHAREDMEMORY_E_OSFAILURE
 *  @brief  Failure in OS call.
 */
#define LISTMPSHAREDMEMORY_E_OSFAILURE \
				LISTMPSHAREDMEMORY_MAKE_FAILURE(5)

/*!
 *  @def	LISTMPSHAREDMEMORY_E_NOTONWER
 *  @brief  Instance is not created on this processor.
 */
#define LISTMPSHAREDMEMORY_E_NOTOWNER \
				LISTMPSHAREDMEMORY_MAKE_FAILURE(6)

/*!
 *  @def	LISTMPSHAREDMEMORY_E_REMOTEACTIVE
 *  @brief  Remote opener of the instance has not closed the instance.
 */
#define LISTMPSHAREDMEMORY_E_REMOTEACTIVE \
				LISTMPSHAREDMEMORY_MAKE_FAILURE(7)

/*!
 *  @def	LISTMPSHAREDMEMORY_E_INUSE
 *  @brief  Indicates that the instance is in use..
 */
#define LISTMPSHAREDMEMORY_E_INUSE \
				LISTMPSHAREDMEMORY_MAKE_FAILURE(8)

/*!
 *  @def	LISTMPSHAREDMEMORY_E_NOTFOUND
 *  @brief  name not found in the nameserver.
 */
#define LISTMPSHAREDMEMORY_E_NOTFOUND \
				LISTMPSHAREDMEMORY_MAKE_FAILURE(9)

/*!
 *  @def    LISTMPSHAREDMEMORY_E_NOTCREATED
 *  @brief  Instance is not created yet
 */
#define LISTMPSHAREDMEMORY_E_NOTCREATED \
				LISTMPSHAREDMEMORY_MAKE_FAILURE(10)

/*!
 *  @def    LISTMPSHAREDMEMORY_E_VERSION
 *  @brief  Version mismatch error.
 */
#define LISTMPSHAREDMEMORY_E_VERSION \
				LISTMPSHAREDMEMORY_MAKE_FAILURE(11)

/*!
 *  @def	LISTMPSHAREDMEMORY_E_BUSY
 *  @brief  the name is already registered or not.
 */
#define LISTMPSHAREDMEMORY_E_BUSY \
				LISTMPSHAREDMEMORY_MAKE_FAILURE(12)


/*!
 *  @def	LISTMPSHAREDMEMORY_SUCCESS
 *  @brief  Operation successful.
 */
#define LISTMPSHAREDMEMORY_SUCCESS \
				LISTMPSHAREDMEMORY_MAKE_SUCCESS(0)

/*!
 *  @def	LISTMPSHAREDMEMORY_S_ALREADYSETUP
 *  @brief  The LISTMPSHAREDMEMORY module has already been setup in this
 *	  process.
 */
#define LISTMPSHAREDMEMORY_S_ALREADYSETUP \
				LISTMPSHAREDMEMORY_MAKE_SUCCESS(1)

/*!
 *  @def	listmp_sharedmemory_CREATED
 *  @brief  Creation of list succesful.
*/
#define LISTMP_SHAREDMEMORY_CREATED	(0x12181964)

/*!
 *  @def	LISTMP_SHAREDMEMORY_VERSION
 *  @brief  Version.
 */
#define LISTMP_SHAREDMEMORY_VERSION	(1)

/* =============================================================================
 *  Structure definitions
 * =============================================================================
 */
/*!
 *  @brief  Structure defining config parameters for the ListMP instances.
 */
#define listmp_sharedmemory_params	struct listmp_params


/*! @brief Forward declaration of structure defining object for the
 *      ListMPSharedMemory.
 */
/*!
 *  @brief  Object for the ListMPSharedMemory Handle
 */
#define listmp_sharedmemory_object	struct listmp_object

/*!
 *  @brief  Handle for the ListMPSharedMemory
 */
#define listmp_sharedmemory_handle	void *

/* =============================================================================
 *  Functions to create the module
 * =============================================================================
 */
/*  Function to get configuration parameters to setup
 *  the ListMPSharedMemory module.
 */
int listmp_sharedmemory_get_config(struct listmp_config *cfgParams);

/* Function to setup the ListMPSharedMemory module.  */
int listmp_sharedmemory_setup(struct listmp_config *config) ;

/* Function to destroy the ListMPSharedMemory module. */
int listmp_sharedmemory_destroy(void);

/* =============================================================================
 *  Functions to create instance of a list
 * =============================================================================
 */
/* Function to create an instance of ListMP */
listmp_sharedmemory_handle listmp_sharedmemory_create
			(listmp_sharedmemory_params *params);

/* Function to delete an instance of ListMP */
int listmp_sharedmemory_delete(listmp_sharedmemory_handle *listMPHandlePtr);

/* =============================================================================
 *  Functions to open/close handle to list instance
 * =============================================================================
 */
/*
 *  Initialize this config-params structure with supplier-specified
 *  defaults before instance creation.
 */
void listmp_sharedmemory_params_init(listmp_sharedmemory_handle handle,
				listmp_sharedmemory_params *params);

/* Function to open a previously created instance */
int listmp_sharedmemory_open(listmp_sharedmemory_handle *listMpHandlePtr,
				listmp_sharedmemory_params *params);

/* Function to close a previously opened instance */
int listmp_sharedmemory_close(listmp_sharedmemory_handle listMPHandle);

/* =============================================================================
 *  Functions for list operations
 * =============================================================================
 */
/* Function to check if list is empty */
bool listmp_sharedmemory_empty(listmp_sharedmemory_handle listMPHandle);

/* Function to get head element from list */
void *listmp_sharedmemory_get_head(listmp_sharedmemory_handle listMPHandle);

/* Function to get tail element from list */
void *listmp_sharedmemory_get_tail(listmp_sharedmemory_handle listMPHandle);

/* Function to put head element into list */
int listmp_sharedmemory_put_head(listmp_sharedmemory_handle listMPHandle,
				struct listmp_elem *elem);

/* Function to put tail element into list */
int listmp_sharedmemory_put_tail(listmp_sharedmemory_handle listMPHandle,
				struct listmp_elem *elem);

/* Function to insert element into list */
int listmp_sharedmemory_insert(listmp_sharedmemory_handle listMPHandle,
				struct listmp_elem *elem,
				struct listmp_elem *curElem);

/* Function to traverse to remove element from list */
int listmp_sharedmemory_remove(listmp_sharedmemory_handle listMPHandle,
				struct listmp_elem *elem);

/* Function to traverse to next element in list */
void *listmp_sharedmemory_next(listmp_sharedmemory_handle listMPHandle,
				struct listmp_elem *elem);

/* Function to traverse to prev element in list */
void *listmp_sharedmemory_prev(listmp_sharedmemory_handle listMPHandle,
				struct listmp_elem *elem);

/* =============================================================================
 *  Functions for shared memory requirements
 * =============================================================================
 */
/* Amount of shared memory required for creation of each instance. */
int listmp_sharedmemory_shared_memreq(
				listmp_sharedmemory_params *params);

#endif /* _LISTMP_SHAREDMEMORY_H_ */
