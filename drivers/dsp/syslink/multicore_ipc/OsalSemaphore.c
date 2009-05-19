/*!
 *  @file       OsalSemaphore.c
 *
 *  @brief      Linux kernel Semaphore interface implementation.
 *
 *              This abstracts the Semaphore interface in Kernel code and
 *              is implemented using the wait queues. It has interfaces
 *              for creating, destroying, waiting and triggering the Semaphores.
 *
 *  @date       04 Feb, 2009
 *
 *  @internal   04 Feb, 2009, Harshit Srivastava, Revision 0001:
 *              [1] Original version.
 *              17 Apr, 2009, Mugdha Kamoolkar, Revision 0002:
 *              [1] Corrections and updates for coding guidelines etc.
 */


/* Standard headers */
#include <linux/types.h>
#include <linux/module.h>

/* OSAL and utils headers */
#include <OsalSemaphore.h>
#include <gt.h>
#include <linux/slab.h>

//#include <Memory.h>

/* Linux specific header files */
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/sched.h>


#define gt_setFailureReason(x, y, z, p, q)
#define gt_assert(x, y)

/* =============================================================================
 *  Macros and types
 * =============================================================================
 */
/*!
 *  @brief   Defines object to encapsulate the Semaphore.
 *           The definition is OS/platform specific.
 */
typedef struct OsalSemaphore_Object_tag {
    OsalSemaphore_Type  semType;
    /*!< Indicates the type of the semaphore (binary or counting). */
    wait_queue_head_t   list;
    /*!< List of waiting processes. */
    u32              value;
    /*!< Current status of semaphore (0,1) - binary and (0-n) counting. */
    struct mutex        lock;
    /*!< lock on which this Semaphore is based. */
} OsalSemaphore_Object;


/* =============================================================================
 * APIs
 * =============================================================================
 */
/*!
 *  @brief      Creates an instance of Mutex object.
 *
 *  @param      semType Type of semaphore. This parameter is a mask of semaphore
 *                      type and interruptability type.
 *
 *  @sa         OsalSemaphore_delete
 */
OsalSemaphore_Handle
OsalSemaphore_create (u32 semType)
{
    OsalSemaphore_Object * semObj = NULL;

    gt_1trace (curTrace, GT_ENTER, "OsalSemaphore_create", semType);

    /* Check for semaphore type (binary/counting) */
    gt_assert (curTrace,
               (    (OSALSEMAPHORE_TYPE_VALUE(semType))
                <   OsalSemaphore_Type_EndValue));
    /* Check for semaphore interruptability */
    gt_assert (curTrace,
               (    (OSALSEMAPHORE_INTTYPE_VALUE(semType))
                <   OsalSemaphore_IntType_EndValue));

    if (OSALSEMAPHORE_TYPE_VALUE(semType) >= OsalSemaphore_Type_EndValue) {
        /*! @retVal NULL Invalid semaphore type (OsalSemaphore_Type) provided */
        gt_setFailureReason (curTrace,
                        GT_4CLASS,
                        "OsalSemaphore_create",
                        OSALSEMAPHORE_E_INVALIDARG,
                        "Invalid semaphore type (OsalSemaphore_Type) provided");
    }
    else if (   OSALSEMAPHORE_INTTYPE_VALUE(semType)
             >= OsalSemaphore_IntType_EndValue) {
        /*! @retVal NULL Invalid semaphore interruptability type
                         (OsalSemaphore_IntType) provided */
        gt_setFailureReason (curTrace,
                        GT_4CLASS,
                        "OsalSemaphore_create",
                        OSALSEMAPHORE_E_INVALIDARG,
                        "Invalid semaphore interruptability type "
                        "(OsalSemaphore_IntType) provided");
    }
    else {
        semObj = kzalloc (sizeof (OsalSemaphore_Object), GFP_KERNEL);
        if (semObj == NULL) {
            /*! @retVal NULL Failed to allocate memory for semaphore object. */
            gt_setFailureReason (curTrace,
                             GT_4CLASS,
                             "OsalSemaphore_create",
                             OSALSEMAPHORE_E_MEMORY,
                             "Failed to allocate memory for semaphore object.");
        }
        else {
            semObj->semType = semType;
            semObj->value = 0u;
            mutex_init (&(semObj->lock));
            init_waitqueue_head (&semObj->list);
#if !defined(SYSLINK_BUILD_OPTIMIZE)
        }
    }
#endif /* #if !defined(SYSLINK_BUILD_OPTIMIZE) */

    gt_1trace (curTrace, GT_LEAVE, "OsalSemaphore_create", semObj);

    /*! @retVal Semaphore-handle Operation successfully completed. */
    return (OsalSemaphore_Handle) semObj;
}
EXPORT_SYMBOL(OsalSemaphore_create);

/*!
 *  @brief      Deletes an instance of Semaphore object.
 *
 *  @param      mutexHandle   Semaphore object handle which needs to be deleted.
 *
 *  @sa         OsalSemaphore_create
 */
int
OsalSemaphore_delete (OsalSemaphore_Handle * semHandle)
{
    int status = OSALSEMAPHORE_SUCCESS;

    gt_1trace (curTrace, GT_ENTER, "OsalSemaphore_delete", semHandle);

    gt_assert (curTrace, (semHandle != NULL));

    if (semHandle == NULL) {
        /*! @retVal OSALSEMAPHORE_E_INVALIDARG NULL provided for argument
                                           semHandle.*/
        status = OSALSEMAPHORE_E_INVALIDARG;
        gt_setFailureReason (curTrace,
                             GT_4CLASS,
                             "OsalSemaphore_delete",
                             status,
                             "NULL provided for argument semHandle");
    }
    else if (*semHandle == NULL) {
        /*! @retVal OSALSEMAPHORE_E_HANDLE NULL Semaphore handle provided. */
        status = OSALSEMAPHORE_E_HANDLE;
        gt_setFailureReason (curTrace,
                             GT_4CLASS,
                             "OsalSemaphore_delete",
                             status,
                             "NULL Semaphore handle provided.");
    }
    else {
        kfree (*semHandle);
        *semHandle = NULL;
    }

    gt_1trace (curTrace, GT_LEAVE, "OsalSemaphore_delete", status);

    /*! @retVal OSALSEMAPHORE_SUCCESS Operation successfully completed. */
    return status;
}
EXPORT_SYMBOL(OsalSemaphore_delete);

/*!
 *  @brief      Wait on the Semaphore in the kernel thread context
 *
 *  @param      semHandle   Semaphore object handle
 *  @param      timeout     Timeout (in msec). Special values are provided for
 *                          no-wait and infinite-wait.
 *
 *  @sa         OsalSemaphore_post
 */
int
OsalSemaphore_pend (OsalSemaphore_Handle semHandle, u32 timeout)
{
    int                     status      = OSALSEMAPHORE_SUCCESS;
    OsalSemaphore_Object *  semObj      = (OsalSemaphore_Object *) semHandle;
    int                     osStatus    = 0;
    u32                  timeoutVal  = 0u;

    gt_2trace (curTrace, GT_ENTER, "OsalSemaphore_pend", semHandle, timeout);

    BUG_ON (semHandle == NULL);

    if (semHandle == NULL) {
        /*! @retVal OSALSEMAPHORE_E_HANDLE NULL Semaphore handle provided. */
        status = OSALSEMAPHORE_E_HANDLE;
        gt_setFailureReason (curTrace,
                             GT_4CLASS,
                             "OsalSemaphore_pend",
                             status,
                             "NULL Semaphore handle provided.");
    }
    else {
        /* Different handling for no-timeout case. */
        if (timeout == OSALSEMAPHORE_WAIT_NONE) {
            osStatus = mutex_lock_interruptible(&(semObj->lock));
            gt_assert (curTrace, (osStatus == 0));
            /* TBD: Check if preempt_enable has to be used instead of mutex. */
            /* preempt_disable (); */
            if (semObj->value == 0u) {
                /*! @retVal OSALSEMAPHORE_E_WAITNONE WAIT_NONE timeout value was
                            provided, but semaphore was not available. */
                status = OSALSEMAPHORE_E_WAITNONE ;
                gt_setFailureReason (curTrace,
                                     GT_4CLASS,
                                     "OsalSemaphore_pend",
                                     status,
                                     "WAIT_NONE timeout value was provided, but"
                                     " semaphore was not available.");
            }
            else {
                if (    OSALSEMAPHORE_TYPE_VALUE(semObj->semType)
                    ==  OsalSemaphore_Type_Binary) {
                    semObj->value = 0u;
                }
                else {
                    semObj->value--;
                }
            }
            mutex_unlock (&semObj->lock);
            /* TBD: Check if preempt_enable has to be used instead of mutex. */
            /* preempt_enable (); */
        }
        /* Finite and infinite timeout cases */
        else {
            DECLARE_WAITQUEUE (wait, current);
            /* Get timeout value in OS-recognizable format. */
            if (timeout == OSALSEMAPHORE_WAIT_FOREVER) {
                timeoutVal = MAX_SCHEDULE_TIMEOUT;
            }
            else {
                timeoutVal = msecs_to_jiffies (timeout);
            }
            /* Add the current process to wait queue */
            add_wait_queue_exclusive (&semObj->list, &wait);
            /* Set the current task status as interruptible */
            do {
                set_current_state (TASK_INTERRUPTIBLE);
                /* TBD: Check if preempt_enable has to be used instead of
                        mutex. */
                osStatus = mutex_lock_interruptible(&(semObj->lock));
                gt_assert (curTrace, (osStatus == 0));
                /* preempt_disable (); */
                if (semObj->value != 0) {
                    if (    OSALSEMAPHORE_TYPE_VALUE (semObj->semType)
                        ==  OsalSemaphore_Type_Binary) {
                        semObj->value = 0u;
                    }
                    else {
                        semObj->value--;
                    }

                    /* Release the lock */
                    mutex_unlock (&semObj->lock);
                    /* TBD: Check if preempt_enable has to be used instead of
                     * mutex.
                     */
                    /* preempt_enable (); */
                    break;
                }

                /* Release the lock */
                mutex_unlock (&semObj->lock);
                /* TBD: Check if preempt_enable has to be used instead of
                 * mutex.
                 */
                /* preempt_enable (); */
                osStatus = schedule_timeout (timeoutVal);
                /* Check for status. This run-time check must remain in code
                 * and must not be optimized out.
                 */
                if (osStatus == 0) { /* Timedout? */
                    /*! @retVal OSALSEMAPHORE_E_TIMEOUT Timeout occurred on
                                                        semaphore pend */
                    status = OSALSEMAPHORE_E_TIMEOUT ;
                    gt_setFailureReason (curTrace,
                                         GT_4CLASS,
                                         "OsalSemaphore_pend",
                                         status,
                                         "Timeout occurred on semaphore pend");
                    break ;
                }

                if (osStatus == -ERESTARTSYS) { /* Interrupted? */
                    /* Not a failure. Execution can be resumed. */
                    status = -ERESTARTSYS;
                    gt_setFailureReason (curTrace,
                               GT_1CLASS,
                               "    OsalSemaphore_pend: Semaphore pend",
                               " interrupted\n Handle [0x%x]\n",
                               semObj);
                    break ;
                }

                if (signal_pending(current)) {
                    /* Restart the operation, we don't quit as
                     * application must do a clean exit
                     */
                    flush_signals (current);
                }
            } while (1);
            /* Remove from wait list */
            remove_wait_queue (&semObj->list, &wait);
            /* Set the current task status as running */
            set_current_state (TASK_RUNNING);
        }
    }

    gt_1trace (curTrace, GT_LEAVE, "OsalSemaphore_pend", status);

    /*! @retVal OSALSEMAPHORE_SUCCESS Operation successfully completed. */
    return status;
}
EXPORT_SYMBOL(OsalSemaphore_pend);

/*!
 *  @brief      Signals the semaphore and makes it available for other
 *              threads.
 *
 *  @param      semHandle Semaphore object handle
 *
 *  @sa         OsalSemaphore_pend
 */
int
OsalSemaphore_post (OsalSemaphore_Handle semHandle)
{
    int                     status      = OSALSEMAPHORE_SUCCESS;
    OsalSemaphore_Object *  semObj      = (OsalSemaphore_Object *) semHandle;
    int                     osStatus    = 0;

    gt_1trace (curTrace, GT_ENTER, "OsalSemaphore_post", semHandle);

    gt_assert (curTrace, (semHandle != NULL));

    if (semHandle == NULL) {
        /*! @retVal OSALSEMAPHORE_E_HANDLE NULL Semaphore handle provided. */
        status = OSALSEMAPHORE_E_HANDLE;
        gt_setFailureReason (curTrace,
                             GT_4CLASS,
                             "OsalSemaphore_post",
                             status,
                             "NULL Semaphore handle provided.");
    }
    else {
        if (    OSALSEMAPHORE_TYPE_VALUE (semObj->semType)
            ==  OsalSemaphore_Type_Binary) {
            osStatus = mutex_lock_interruptible (&(semObj->lock));
            gt_assert (curTrace, (osStatus == 0));
            /* TBD: Check if preempt_disable has to be used instead of mutex. */
            /* preempt_disable (); */
            semObj->value = 1u;
            wake_up_interruptible (&(semObj->list));
            mutex_unlock(&(semObj->lock));
            /* TBD: Check if preempt_enable has to be used instead of mutex. */
            /* preempt_enable (); */
        }
        else {
            osStatus = mutex_lock_interruptible (&(semObj->lock));
            gt_assert (curTrace, (osStatus == 0));
            /* TBD: Check if preempt_disable has to be used instead of mutex. */
            /* preempt_disable (); */
            semObj->value++;
            wake_up_interruptible (&semObj->list);
            mutex_unlock (&semObj->lock);
            /* TBD: Check if preempt_enable has to be used instead of mutex. */
            /* preempt_enable (); */
        }
    }

    gt_1trace (curTrace, GT_LEAVE, "OsalSemaphore_post", status);

    /*! @retVal OSALSEMAPHORE_SUCCESS Operation successfully completed. */
    return status;
}
EXPORT_SYMBOL(OsalSemaphore_post);
