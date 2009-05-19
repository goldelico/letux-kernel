
/*  =================================================================
  File hw_ocp.h

  Path $ (PROJROOT)\driver\mailbox

  Desc API declarations for generic OCP Socket system registers for Mailbox

  Rev  0.1.0

  This computer program is copyright to Texas Instruments Incorporated.
  The program may not be used without the written permission of
  Texas Instruments Incorporated or against the terms and conditions
  stipulated in the agreement under which this program has been supplied.

  (c) Texas Instruments Incorporated 2008

  =====================================================================
*/
#ifndef __HW_OCP_H
#define __HW_OCP_H

/* ========================================================================
* INCLUDE FILES (only if necessary)
* =========================================================================
*/

#ifdef __cplusplus
extern "C"
{

#endif

#include <syslink/GlobalTypes.h>
#include <syslink/hw_ocp.h>
#include <syslink/hw_defs.h>
#include <syslink/MBXRegAcM.h>
#include <syslink/MBXAccInt.h>


/* =======================================================================
* EXPORTED DEFINITIONS
* ========================================================================
*/


/* =======================================================================
* EXPORTED TYPES
* ========================================================================
*/

/* -----------------------------------------------------------------------
* TYPE:         HW_IdleMode_t
*
* DESCRIPTION:  Enumerated Type for idle modes in OCP SYSCONFIG register
*
* -------------------------------------------------------------------------
*/
enum hal_ocp_idlemode_t {
HW_OCP_FORCE_IDLE,
HW_OCP_NO_IDLE,
HW_OCP_SMART_IDLE
};

/* =======================================================================
* EXPORTED VARIABLES
* ========================================================================
*/


/* =======================================================================
* EXPORTED FUNCTIONS
* ========================================================================
*/

extern long hw_ocp_soft_reset(const unsigned long base_address);

extern long hw_ocp_soft_reset_isdone(const unsigned long base_address,
				unsigned long *reset_is_done);

extern long hw_ocp_idle_modeset(const unsigned long base_address,
				enum hal_ocp_idlemode_t idle_mode);

extern long hw_ocp_idlemode_get(const unsigned long base_address,
				enum hal_ocp_idlemode_t *idle_mode);

extern long hw_ocp_autoidle_set(const unsigned long base_address,
				enum hw_set_clear_t auto_idle);

extern long hw_ocp_autoidle_get(const unsigned long base_address,
				enum hw_set_clear_t *auto_idle);

#ifdef __cplusplus
}
#endif

#endif  /* __HW_OCP_H */

