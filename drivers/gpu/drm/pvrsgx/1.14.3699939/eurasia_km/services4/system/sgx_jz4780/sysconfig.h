/*************************************************************************/ /*!
@Title          System Description Header
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    This header provides system-specific declarations and macros
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#if !defined(__SOCCONFIG_H__)
#define __SOCCONFIG_H__

#if defined(SUPPORT_EXTERNAL_SYSTEM_CACHE)
#include "extsyscache.h"
#endif

#define VS_PRODUCT_NAME	"JZ4780 SGX"

#define SYS_SGX_USSE_COUNT					(1)

#if defined(NO_HARDWARE)
#if defined(SGX_FEATURE_MP)
/* One block for each core plus one for the all-cores bank and one for the master bank.*/
#define SGX_REG_SIZE 	(0x4000 * (SGX_FEATURE_MP_CORE_COUNT_3D + 2))
#else
#define SGX_REG_SIZE 	(0x4000)
#endif /* SGX_FEATURE_MP */
#endif /* NO_HARDWARE */

#if defined(SGX_FEATURE_HOST_PORT)
	/* INTEGRATION_POINT: change these dummy values if host port is needed */
	#define SYS_SGX_HP_SIZE		0x0
	/* device virtual address of host port base */
	#define SYS_SGX_HOSTPORT_BASE_DEVVADDR 0x0
#endif

#define SYS_SGX_ACTIVE_POWER_LATENCY_MS				2
#define SYS_SGX_PDS_TIMER_FREQ			(1000) // 1ms (1000hz)
#define SYS_SGX_HWRECOVERY_TIMEOUT_FREQ		(100)	// 10ms (100hz)

#define DEVICE_SGX_INTERRUPT		(1<<0)

#ifndef SYS_SGX_DEV_NAME
#define	SYS_SGX_DEV_NAME	"ingenic,jz4780-sgx"
#endif

/*****************************************************************************
 * system specific data structures
 *****************************************************************************/
 
#endif	/* __SOCCONFIG_H__ */
