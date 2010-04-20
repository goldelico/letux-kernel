/**********************************************************************
 *
 * Copyright(c) 2008 Imagination Technologies Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful but, except
 * as otherwise stated in writing, without any warranty; without even the
 * implied warranty of merchantability or fitness for a particular purpose.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK
 *
 ******************************************************************************/

#include <linux/version.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/hardirq.h>
#include <linux/spinlock.h>
#include <asm/bug.h>

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,26))
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22))
#include <asm/arch/resource.h>
#endif
#endif

#include "sgxdefs.h"
#include "services_headers.h"
#include "sysinfo.h"
#include "sgxapi_km.h"
#include "sysconfig.h"
#include "sgxinfokm.h"
#include "syslocal.h"

#define	ONE_MHZ	1000000
#define	HZ_TO_MHZ(m) ((m) / ONE_MHZ)

#if defined(SUPPORT_OMAP3430_SGXFCLK_96M)
#define SGX_PARENT_CLOCK "cm_96m_fck"
#else
#define SGX_PARENT_CLOCK "core_ck"
#endif

static IMG_BOOL PowerLockWrappedOnCPU(SYS_SPECIFIC_DATA unref__ *psSysSpecData)
{
	return IMG_FALSE;
}

static IMG_VOID PowerLockWrap(SYS_SPECIFIC_DATA unref__ *psSysSpecData)
{
}

static IMG_VOID PowerLockUnwrap(SYS_SPECIFIC_DATA unref__ *psSysSpecData)
{
}

PVRSRV_ERROR SysPowerLockWrap(SYS_DATA unref__ *psSysData)
{
	return PVRSRV_OK;
}

IMG_VOID SysPowerLockUnwrap(SYS_DATA unref__ *psSysData)
{
}

IMG_BOOL WrapSystemPowerChange(SYS_SPECIFIC_DATA *psSysSpecData)
{
	IMG_BOOL bPowerLock = PowerLockWrappedOnCPU(psSysSpecData);

	if (bPowerLock)
	{
		PowerLockUnwrap(psSysSpecData);
	}

	return bPowerLock;
}

IMG_VOID UnwrapSystemPowerChange(SYS_SPECIFIC_DATA *psSysSpecData)
{
	PowerLockWrap(psSysSpecData);
}

static inline IMG_UINT32 scale_by_rate(IMG_UINT32 val, IMG_UINT32 rate1, IMG_UINT32 rate2)
{
	if (rate1 >= rate2)
	{
		return val * (rate1 / rate2);
	}

	return val / (rate2 / rate1);
}

static inline IMG_UINT32 scale_prop_to_SGX_clock(IMG_UINT32 val, IMG_UINT32 rate)
{
	return scale_by_rate(val, rate, SYS_SGX_CLOCK_SPEED);
}

static inline IMG_UINT32 scale_inv_prop_to_SGX_clock(IMG_UINT32 val, IMG_UINT32 rate)
{
	return scale_by_rate(val, SYS_SGX_CLOCK_SPEED, rate);
}

IMG_VOID SysGetSGXTimingInformation(SGX_TIMING_INFORMATION *psTimingInfo)
{
	IMG_UINT32 rate;

#if defined(NO_HARDWARE)
	rate = SYS_SGX_CLOCK_SPEED;
#else
	PVR_ASSERT(atomic_read(&gpsSysSpecificData->sSGXClocksEnabled) != 0);

       rate = SYS_SGX_CLOCK_SPEED;
	PVR_ASSERT(rate != 0);
#endif
	psTimingInfo->ui32CoreClockSpeed = rate;
	psTimingInfo->ui32HWRecoveryFreq = scale_prop_to_SGX_clock(SYS_SGX_HWRECOVERY_TIMEOUT_FREQ, rate);
	psTimingInfo->ui32uKernelFreq = scale_prop_to_SGX_clock(SYS_SGX_PDS_TIMER_FREQ, rate);
#if defined(SUPPORT_ACTIVE_POWER_MANAGEMENT)
	psTimingInfo->bEnableActivePM = IMG_TRUE;
#else
	psTimingInfo->bEnableActivePM = IMG_FALSE;
#endif
	psTimingInfo->ui32ActivePowManLatencyms = SYS_SGX_ACTIVE_POWER_LATENCY_MS;
}

PVRSRV_ERROR EnableSGXClocks(SYS_DATA *psSysData)
{
#if !defined(NO_HARDWARE)
	SYS_SPECIFIC_DATA *psSysSpecData = (SYS_SPECIFIC_DATA *) psSysData->pvSysSpecificData;


	if (atomic_read(&psSysSpecData->sSGXClocksEnabled) != 0)
	{
		return PVRSRV_OK;
	}

	PVR_DPF((PVR_DBG_MESSAGE, "EnableSGXClocks: Enabling SGX Clocks"));




	atomic_set(&psSysSpecData->sSGXClocksEnabled, 1);

#else
	PVR_UNREFERENCED_PARAMETER(psSysData);
#endif
	return PVRSRV_OK;
}


IMG_VOID DisableSGXClocks(SYS_DATA *psSysData)
{
#if !defined(NO_HARDWARE)
	SYS_SPECIFIC_DATA *psSysSpecData = (SYS_SPECIFIC_DATA *) psSysData->pvSysSpecificData;


	if (atomic_read(&psSysSpecData->sSGXClocksEnabled) == 0)
	{
		return;
	}

	PVR_DPF((PVR_DBG_MESSAGE, "DisableSGXClocks: Disabling SGX Clocks"));


	atomic_set(&psSysSpecData->sSGXClocksEnabled, 0);

#else
	PVR_UNREFERENCED_PARAMETER(psSysData);
#endif
}

PVRSRV_ERROR EnableSystemClocks(SYS_DATA *psSysData)
{
	SYS_SPECIFIC_DATA *psSysSpecData = (SYS_SPECIFIC_DATA *) psSysData->pvSysSpecificData;
	PVRSRV_ERROR eError;
	IMG_BOOL bPowerLock;

#if !defined(NO_OMAP_TIMER)
#if defined(DEBUG) || defined(TIMING)
	IMG_INT rate;
	struct clk *sys_ck;
	IMG_CPU_PHYADDR     TimerRegPhysBase;
	IMG_HANDLE hTimerEnable;
	IMG_UINT32 *pui32TimerEnable;

#endif
#endif

	PVR_TRACE(("EnableSystemClocks: Enabling System Clocks"));

	if (!psSysSpecData->bSysClocksOneTimeInit)
	{
		bPowerLock = IMG_FALSE;

		spin_lock_init(&psSysSpecData->sPowerLock);
		atomic_set(&psSysSpecData->sPowerLockCPU, -1);
		spin_lock_init(&psSysSpecData->sNotifyLock);
		atomic_set(&psSysSpecData->sNotifyLockCPU, -1);

		atomic_set(&psSysSpecData->sSGXClocksEnabled, 0);
		psSysSpecData->bSysClocksOneTimeInit = IMG_TRUE;
	}

	eError = PVRSRV_OK;
	goto Exit;

#if !defined(NO_OMAP_TIMER)
#if defined(DEBUG) || defined(TIMING)
ExitDisableGPT11ICK:
ExitDisableGPT11FCK:
ExitUnRegisterConstraintNotifications:
#endif
#endif
	eError = PVRSRV_ERROR_GENERIC;
Exit:
	if (bPowerLock)
	{
		PowerLockWrap(psSysSpecData);
	}

	return eError;
}

IMG_VOID DisableSystemClocks(SYS_DATA *psSysData)
{
	SYS_SPECIFIC_DATA *psSysSpecData = (SYS_SPECIFIC_DATA *) psSysData->pvSysSpecificData;
	IMG_BOOL bPowerLock;
#if !defined(NO_OMAP_TIMER)
#if defined(DEBUG) || defined(TIMING)
	IMG_CPU_PHYADDR TimerRegPhysBase;
	IMG_HANDLE hTimerDisable;
	IMG_UINT32 *pui32TimerDisable;
#endif
#endif

	PVR_TRACE(("DisableSystemClocks: Disabling System Clocks"));


	DisableSGXClocks(psSysData);

	bPowerLock = PowerLockWrappedOnCPU(psSysSpecData);
	if (bPowerLock)
	{

		PowerLockUnwrap(psSysSpecData);
	}

#if !defined(NO_OMAP_TIMER)
#if defined(DEBUG) || defined(TIMING)

	TimerRegPhysBase.uiAddr = SYS_OMAP3430_GP11TIMER_ENABLE_SYS_PHYS_BASE;
	pui32TimerDisable = OSMapPhysToLin(TimerRegPhysBase,
				4,
				PVRSRV_HAP_KERNEL_ONLY|PVRSRV_HAP_UNCACHED,
				&hTimerDisable);

	if (pui32TimerDisable == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "DisableSystemClocks: OSMapPhysToLin failed"));
	}
	else
	{
		*pui32TimerDisable = 0;

		OSUnMapPhysToLin(pui32TimerDisable,
				4,
				PVRSRV_HAP_KERNEL_ONLY|PVRSRV_HAP_UNCACHED,
				hTimerDisable);
	}


#endif
#endif
	if (bPowerLock)
	{
		PowerLockWrap(psSysSpecData);
	}
}
