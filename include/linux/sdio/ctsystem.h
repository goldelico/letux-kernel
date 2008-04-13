/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
@file: cpsystem.h

@abstract: common system include file.

@notice: Copyright (c), 2004-2006 Atheros Communications, Inc.


 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation;
 *
 *  Software distributed under the License is distributed on an "AS
 *  IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
 *  implied. See the License for the specific language governing
 *  rights and limitations under the License.
 *
 *  Portions of this code were developed with information supplied from the
 *  SD Card Association Simplified Specifications. The following conditions and disclaimers may apply:
 *
 *   The following conditions apply to the release of the SD simplified specification (�Simplified
 *   Specification�) by the SD Card Association. The Simplified Specification is a subset of the complete
 *   SD Specification which is owned by the SD Card Association. This Simplified Specification is provided
 *   on a non-confidential basis subject to the disclaimers below. Any implementation of the Simplified
 *   Specification may require a license from the SD Card Association or other third parties.
 *   Disclaimers:
 *   The information contained in the Simplified Specification is presented only as a standard
 *   specification for SD Cards and SD Host/Ancillary products and is provided "AS-IS" without any
 *   representations or warranties of any kind. No responsibility is assumed by the SD Card Association for
 *   any damages, any infringements of patents or other right of the SD Card Association or any third
 *   parties, which may result from its use. No license is granted by implication, estoppel or otherwise
 *   under any patent or other rights of the SD Card Association or any third party. Nothing herein shall
 *   be construed as an obligation by the SD Card Association to disclose or distribute any technical
 *   information, know-how or other confidential information to any third party.
 *
 *
 *  The initial developers of the original code are Seung Yi and Paul Lever
 *
 *  sdio@atheros.com
 *
 *

+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#ifndef __CPSYSTEM_H___
#define __CPSYSTEM_H___

/* SDIO stack status defines */
/* < 0 error, >0 warning, 0 success */
#define SDIO_IS_WARNING(status) ((status) > 0)
#define SDIO_IS_ERROR(status) ((status) < 0)
#define SDIO_SUCCESS(status) ((SDIO_STATUS)(status) >= 0)
#define SDIO_STATUS_SUCCESS             0
#define SDIO_STATUS_ERROR              -1
#define SDIO_STATUS_INVALID_PARAMETER  -2
#define SDIO_STATUS_PENDING             3
#define SDIO_STATUS_DEVICE_NOT_FOUND   -4
#define SDIO_STATUS_DEVICE_ERROR       -5
#define SDIO_STATUS_INTERRUPTED        -6
#define SDIO_STATUS_NO_RESOURCES       -7
#define SDIO_STATUS_CANCELED           -8
#define SDIO_STATUS_BUFFER_TOO_SMALL   -9
#define SDIO_STATUS_NO_MORE_MESSAGES   -10
#define SDIO_STATUS_BUS_RESP_TIMEOUT   -20    /* response timed-out */
#define SDIO_STATUS_BUS_READ_TIMEOUT   -21    /* read data timed-out */
#define SDIO_STATUS_BUS_READ_CRC_ERR   -22   /* data CRC failed */
#define SDIO_STATUS_BUS_WRITE_ERROR    -23   /* write failed */
#define SDIO_STATUS_BUS_RESP_CRC_ERR   -24   /* response received with a CRC error */
#define SDIO_STATUS_INVALID_TUPLE_LENGTH -25 /* tuple length was invalid */
#define SDIO_STATUS_TUPLE_NOT_FOUND      -26 /* tuple could not be found */
#define SDIO_STATUS_CIS_OUT_OF_RANGE     -27 /* CIS is out of range in the tuple scan */
#define SDIO_STATUS_FUNC_ENABLE_TIMEOUT  -28 /* card timed out enabling or disabling */
#define SDIO_STATUS_DATA_STATE_INVALID   -29 /* card is in an invalid state for data */
#define SDIO_STATUS_DATA_ERROR_UNKNOWN   -30 /* card cannot process data transfer */
#define SDIO_STATUS_INVALID_FUNC         -31 /* sdio request is not valid for the function */
#define SDIO_STATUS_FUNC_ARG_ERROR       -32 /* sdio request argument is invalid or out of range */
#define SDIO_STATUS_INVALID_COMMAND      -33 /* SD COMMAND is invalid for the card state */
#define SDIO_STATUS_SDREQ_QUEUE_FAILED   -34 /* request failed to insert into queue */
#define SDIO_STATUS_BUS_RESP_TIMEOUT_SHIFTABLE -35  /* response timed-out, possibily shiftable to correct  */
#define SDIO_STATUS_UNSUPPORTED          -36  /* not supported  */
#define SDIO_STATUS_PROGRAM_TIMEOUT      -37  /* memory card programming timeout  */
#define SDIO_STATUS_PROGRAM_STATUS_ERROR -38  /* memory card programming errors  */

#include <linux/sdio/ctsystem_linux.h>

/* get structure from contained field */
#define CONTAINING_STRUCT(address, struct_type, field_name)\
            ((struct_type *)((ULONG_PTR)(address) - (ULONG_PTR)(&((struct_type *)0)->field_name)))

#define ZERO_OBJECT(obj) memset(&(obj),0,sizeof(obj))
#define ZERO_POBJECT(pObj) memset((pObj),0,sizeof(*(pObj)))


/* bit field support functions */
static INLINE void SetBit(PULONG pField, UINT position) {
    *pField |= 1 << position;
}
static INLINE void ClearBit(PULONG pField, UINT position) {
    *pField &= ~(1 << position);
}
static INLINE BOOL IsBitSet(PULONG pField, UINT position) {
    return (*pField & (1 << position));
}
static INLINE INT FirstClearBit(PULONG pField) {
    UINT ii;
    for(ii = 0; ii < sizeof(ULONG)*8; ii++) {
        if (!IsBitSet(pField, ii)) {
            return ii;
        }
    }
    /* no clear bits found */
    return -1;
}

#endif /* __CPSYSTEM_H___ */
