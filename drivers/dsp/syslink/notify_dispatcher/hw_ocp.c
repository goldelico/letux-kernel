/*===========================================================================
  File    hal_ocp.c

  Path    $(PROJROOT)\driver\mailbox

  Desc    API definitions to setup OCP Socket system registers

  Rev     0.1.0

  This computer program is copyright to Texas Instruments Incorporated.
  The program may not be used without the written permission of
  Texas Instruments Incorporated or against the terms and conditions
  stipulated in the agreement under which this program has been supplied.

 (c) Texas Instruments Incorporated 2008

  ===========================================================================
*/


/*============================================================================
*STANDARD INCLUDE FILES
*=============================================================================
*/

/*============================================================================
*PROJECT SPECIFIC INCLUDE FILES
*=============================================================================
*/
#include <syslink/GlobalTypes.h>
#include <syslink/hw_ocp.h>
#include <syslink/hw_defs.h>
#include <syslink/MBXRegAcM.h>
#include <syslink/MBXAccInt.h>
#include <linux/module.h>
MODULE_LICENSE("GPL");


/*============================================================================
*GLOBAL VARIABLES DECLARATIONS
*=============================================================================
*/

/*============================================================================
*LOCAL TYPES AND DEFINITIONS
*=============================================================================
*/


/*============================================================================
*LOCAL VARIABLES DECLARATIONS
*=============================================================================
*/

/*============================================================================
*LOCAL FUNCTIONS PROTOTYPES
*=============================================================================
*/

/*============================================================================
*EXPORTED FUNCTIONS
*=============================================================================
*/

long hw_ocp_soft_reset(const unsigned long base_address)
{
    long status = RET_OK;
    MLBMAILBOX_SYSCONFIGSoftResetWrite32(base_address, HW_SET);
    return status;
}

long hw_ocp_soft_reset_isdone(const unsigned long base_address,
					unsigned long *reset_is_done)
{
    long status = RET_OK;

    *reset_is_done = MLBMAILBOX_SYSSTATUSResetDoneRead32(base_address);

    return status;
}

long hw_ocp_idle_modeset(const unsigned long base_address,
			enum hal_ocp_idlemode_t idle_mode)
{
    long status = RET_OK;

    MLBMAILBOX_SYSCONFIGSIdleModeWrite32(base_address, idle_mode);

    return status;
}


long hw_ocp_idlemode_get(const unsigned long base_address,
			enum hal_ocp_idlemode_t *idle_mode)
{
    long status = RET_OK;

    *idle_mode = (enum hal_ocp_idlemode_t)
			MLBMAILBOX_SYSCONFIGSIdleModeRead32(base_address);

    return status;
}

long hw_ocp_autoidle_set(const unsigned long base_address,
				enum hw_set_clear_t auto_idle)
{
    long status = RET_OK;

    MLBMAILBOX_SYSCONFIGAutoIdleWrite32(base_address, auto_idle);

    return status;
}

long hw_ocp_autoidle_get(const unsigned long base_address,
				enum hw_set_clear_t *auto_idle)
{
    long status = RET_OK;

    *auto_idle = (enum hw_set_clear_t)
			MLBMAILBOX_SYSCONFIGAutoIdleRead32(base_address);

    return status;
}

/*============================================================================
*LOCAL FUNCTIONS
*=============================================================================
*/

