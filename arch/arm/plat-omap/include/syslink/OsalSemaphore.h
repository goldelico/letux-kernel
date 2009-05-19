/*!
 *  @file	OsalSemaphore.h
 *
 *  @brief	Kernel utils Semaphore interface definitions.
 *
 *		This abstracts the Semaphore interface in Kernel code and
 *		is implemented using the wait queues. It has interfaces
 *		for creating, destroying, waiting and triggering the Semaphores.
 *
 *  @date	04-02-2009
 *
 *  @internal   04-02-2009, Harshit Srivastava, Revision 0001:
 *		[1] Original version.
 *		Apr 18, 2009, Mugdha Kamoolkar, Revision 0002:
 *		[1] Minor updates.
 */


#ifndef OSALSEMAPHORE_H_0xF6D6
#define OSALSEMAPHORE_H_0xF6D6


/* Standard headers */
#include <linux/types.h>

/* OSAL and utils */

/*!
 *  @def	OSALSEMAPHORE_MODULEID
 *  @brief  Module ID for OsalSemaphore OSAL module.
 */
#define OSALSEMAPHORE_MODULEID		(u16) 0xF6D6

/* =============================================================================
 *  All success and failure codes for the module
 * =============================================================================
 */
/*!
* @def   OSALSEMAPHORE_STATUSCODEBASE
* @brief Stauts code base for MEMORY module.
*/
#define OSALSEMAPHORE_STATUSCODEBASE	(OSALSEMAPHORE_MODULEID << 12)

/*!
* @def   OSALSEMAPHORE_MAKE_FAILURE
* @brief Convert to failure code.
*/
#define OSALSEMAPHORE_MAKE_FAILURE(x)	((int) (0x80000000 \
					+ (OSALSEMAPHORE_STATUSCODEBASE + (x))))
/*!
* @def   OSALSEMAPHORE_MAKE_SUCCESS
* @brief Convert to success code.
*/
#define OSALSEMAPHORE_MAKE_SUCCESS(x)	(OSALSEMAPHORE_STATUSCODEBASE + (x))

/*!
* @def   OSALSEMAPHORE_E_MEMORY
* @brief Indicates OsalSemaphore alloc/free failure.
*/
#define OSALSEMAPHORE_E_MEMORY		OSALSEMAPHORE_MAKE_FAILURE(1)

/*!
* @def   OSALSEMAPHORE_E_INVALIDARG
* @brief Invalid argument provided
*/
#define OSALSEMAPHORE_E_INVALIDARG	OSALSEMAPHORE_MAKE_FAILURE(2)

/*!
* @def   OSALSEMAPHORE_E_FAIL
* @brief Generic failure
*/
#define OSALSEMAPHORE_E_FAIL		OSALSEMAPHORE_MAKE_FAILURE(3)

/*!
* @def   OSALSEMAPHORE_E_TIMEOUT
* @brief A timeout occurred
*/
#define OSALSEMAPHORE_E_TIMEOUT		OSALSEMAPHORE_MAKE_FAILURE(4)

/*!
 *  @def	OSALSEMAPHORE_E_HANDLE
 *  @brief  Invalid handle provided
 */
#define OSALSEMAPHORE_E_HANDLE		OSALSEMAPHORE_MAKE_FAILURE(5)

/*!
 *  @def	OSALSEMAPHORE_E_WAITNONE
 *  @brief  WAIT_NONE timeout value was provided, but semaphore was not
 *	  available.
 */
#define OSALSEMAPHORE_E_WAITNONE	OSALSEMAPHORE_MAKE_FAILURE(6)

/*!
* @def   OSALSEMAPHORE_SUCCESS
* @brief Operation successfully completed
*/
#define OSALSEMAPHORE_SUCCESS		OSALSEMAPHORE_MAKE_SUCCESS(0)


/* =============================================================================
 *  Macros and types
 * =============================================================================
 */
/*!
 *  @def	OSALSEMAPHORE_WAIT_FOREVER
 *  @brief  Indicates forever wait for APIs that can wait.
 */
#define OSALSEMAPHORE_WAIT_FOREVER	(~((u32) 0))

/*!
 *  @def	OSALSEMAPHORE_WAIT_NONE
 *  @brief  Indicates zero wait for APIs that can wait.
 */
#define OSALSEMAPHORE_WAIT_NONE		((u32) 0)

/*!
 *  @def	OSALSEMAPHORE_TYPE_VALUE
 *  @brief  Returns the value of semaphore type (binary/counting)
 */
#define OSALSEMAPHORE_TYPE_VALUE(type)	(type & 0x0000FFFF)

/*!
 *  @def	OSALSEMAPHORE_INTTYPE_VALUE
 *  @brief  Returns the value of semaphore interruptability type
 */
#define OSALSEMAPHORE_INTTYPE_VALUE(type)	(type & 0xFFFF0000)

/*!
 *  @brief  Declaration for the OsalSemaphore object handle.
 *	  Definition of OsalSemaphore_Object is not exposed.
 */
typedef struct OsalSemaphore_Object *OsalSemaphore_Handle;

/*!
 *  @brief   Enumerates the types of semaphores
 */
typedef enum {
	OsalSemaphore_Type_Binary = 0x00000000,
	/*!< Binary semaphore */
	OsalSemaphore_Type_Counting = 0x00000001,
	/*!< Counting semaphore */
	OsalSemaphore_Type_EndValue = 0x00000002
	/*!< End delimiter indicating start of invalid values for this enum */
} OsalSemaphore_Type;

/*!
 *  @brief   Enumerates the interruptible/non-interruptible types.
 */
typedef enum {
	OsalSemaphore_IntType_Interruptible = 0x00000000,
	/*!< Waits on this mutex are interruptible */
	OsalSemaphore_IntType_Noninterruptible = 0x00010000,
	/*!< Waits on this mutex are non-interruptible */
	OsalSemaphore_IntType_EndValue = 0x00020000
	/*!< End delimiter indicating start of invalid values for this enum */
} OsalSemaphore_IntType;


/* =============================================================================
 *  APIs
 * =============================================================================
 */
/* Creates the semaphore object. */
OsalSemaphore_Handle OsalSemaphore_create(u32 semType);

/* Deletes the semaphore object */
int OsalSemaphore_delete(OsalSemaphore_Handle *semHandle);

/* Wait on the said Semaphore in the kernel thread context */
int OsalSemaphore_pend(OsalSemaphore_Handle semHandle, u32 timeout);

/* Signal the semaphore and make it available for other threads. */
int OsalSemaphore_post(OsalSemaphore_Handle semHandle);

#endif /* ifndef OSALSEMAPHORE_H_0xF6D6 */
