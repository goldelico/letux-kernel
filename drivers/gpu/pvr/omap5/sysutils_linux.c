/**********************************************************************
 *
 * Copyright (C) Imagination Technologies Ltd. All rights reserved.
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
#include <linux/mutex.h>
#include <linux/slab.h>

#include "sgxdefs.h"
#include "services_headers.h"
#include "sysinfo.h"
#include "sgxapi_km.h"
#include "sysconfig.h"
#include "sgxinfokm.h"
#include "syslocal.h"

#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/opp.h>

#if defined(SUPPORT_DRI_DRM_PLUGIN)
#include <drm/drmP.h>
#include <drm/drm.h>

#include <linux/omap_gpu.h>

#include "pvr_drm.h"
#endif

#ifdef CONFIG_OMAP_GPU_GOVERNOR
#include <linux/thermal_framework.h>
#endif

#define	ONE_MHZ	1000000
#define	HZ_TO_MHZ(m) ((m) / ONE_MHZ)

#if defined(SUPPORT_OMAP3430_SGXFCLK_96M)
#define SGX_PARENT_CLOCK "cm_96m_fck"
#else
#define SGX_PARENT_CLOCK "core_ck"
#endif

extern uint sgx_apm_timeout;

#if defined(LDM_PLATFORM) && !defined(PVR_DRI_DRM_NOT_PCI)
extern struct platform_device *gpsPVRLDMDev;
#endif

static PVRSRV_ERROR PowerLockWrap(SYS_SPECIFIC_DATA *psSysSpecData, IMG_BOOL bTryLock)
{
	if (!in_interrupt())
	{
		if (bTryLock)
		{
			int locked = mutex_trylock(&psSysSpecData->sPowerLock);
			if (locked == 0)
			{
				return PVRSRV_ERROR_RETRY;
			}
		}
		else
		{
			mutex_lock(&psSysSpecData->sPowerLock);
		}
	}

	return PVRSRV_OK;
}

static IMG_VOID PowerLockUnwrap(SYS_SPECIFIC_DATA *psSysSpecData)
{
	if (!in_interrupt())
	{
		mutex_unlock(&psSysSpecData->sPowerLock);
	}
}

PVRSRV_ERROR SysPowerLockWrap(IMG_BOOL bTryLock)
{
	SYS_DATA	*psSysData;

	SysAcquireData(&psSysData);

	return PowerLockWrap(psSysData->pvSysSpecificData, bTryLock);
}

IMG_VOID SysPowerLockUnwrap(IMG_VOID)
{
	SYS_DATA	*psSysData;

	SysAcquireData(&psSysData);

	PowerLockUnwrap(psSysData->pvSysSpecificData);
}

IMG_BOOL WrapSystemPowerChange(SYS_SPECIFIC_DATA *psSysSpecData)
{
	return IMG_TRUE;
}

IMG_VOID UnwrapSystemPowerChange(SYS_SPECIFIC_DATA *psSysSpecData)
{
}

IMG_VOID SysGetSGXTimingInformation(SGX_TIMING_INFORMATION *psTimingInfo)
{
#if !defined(NO_HARDWARE)
	PVR_ASSERT(atomic_read(&gpsSysSpecificData->sSGXClocksEnabled) != 0);
#endif
	/*
	 * Always report highest possible core clock speed for the purpose
	 * of initializing SGX timers. This allows us the freedom to scale
	 * core clock asynchronously without shortening timer period in
	 * wall-clock time domain (potentially more problematic than
	 * lengthening).
	*/
	psTimingInfo->ui32CoreClockSpeed =
		gpsSysSpecificData->pui32SGXFreqList[gpsSysSpecificData->ui32SGXFreqListSize - 2];

	psTimingInfo->ui32HWRecoveryFreq = SYS_SGX_HWRECOVERY_TIMEOUT_FREQ;
	psTimingInfo->ui32uKernelFreq = SYS_SGX_PDS_TIMER_FREQ;
#if defined(SUPPORT_ACTIVE_POWER_MANAGEMENT)
	psTimingInfo->bEnableActivePM = IMG_TRUE;
#else
	psTimingInfo->bEnableActivePM = IMG_FALSE;
#endif
	psTimingInfo->ui32ActivePowManLatencyms = sgx_apm_timeout;
}

IMG_VOID RequestSGXFreq(SYS_DATA *psSysData, IMG_UINT32 freq_index)
{
	SYS_SPECIFIC_DATA *psSysSpecData = (SYS_SPECIFIC_DATA *) psSysData->pvSysSpecificData;
	PVRSRV_SGXDEV_INFO *psDevInfo = (PVRSRV_SGXDEV_INFO *)psSysSpecData->psSGXDevNode->pvDevice;
	struct gpu_platform_data *pdata;
	int res;

	pdata = (struct gpu_platform_data *)gpsPVRLDMDev->dev.platform_data;

	if (psSysSpecData->ui32SGXFreqListIndex != freq_index)
	{
		PVR_ASSERT(pdata->device_scale != IMG_NULL);
		res = pdata->device_scale(&gpsPVRLDMDev->dev,
					  &gpsPVRLDMDev->dev,
					  psSysSpecData->pui32SGXFreqList[freq_index]);

		if (res == 0)
			psSysSpecData->ui32SGXFreqListIndex = freq_index;
		else if (res == -EBUSY)
		{
			PVR_DPF((PVR_DBG_WARNING, "RequestSGXFreq: Unable to scale SGX frequency (EBUSY)"));
			psSysSpecData->ui32SGXFreqListIndex = psSysSpecData->ui32SGXFreqListSize - 1;
		}
		else if (res < 0)
		{
			PVR_DPF((PVR_DBG_ERROR, "RequestSGXFreq: Unable to scale SGX frequency (%d)", res));
			psSysSpecData->ui32SGXFreqListIndex = psSysSpecData->ui32SGXFreqListSize - 1;
		}
	}
	psDevInfo->ui32CoreClockSpeed =
		psSysSpecData->pui32SGXFreqList[psSysSpecData->ui32SGXFreqListIndex];
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

#if defined(LDM_PLATFORM) && !defined(PVR_DRI_DRM_NOT_PCI)
	{

		int res = pm_runtime_get_sync(&gpsPVRLDMDev->dev);
		if (res < 0)
		{
			PVR_DPF((PVR_DBG_ERROR, "EnableSGXClocks: pm_runtime_get_sync failed (%d)", -res));
			return PVRSRV_ERROR_UNABLE_TO_ENABLE_CLOCK;
		}
	}
#endif

	SysEnableSGXInterrupts(psSysData);


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

	SysDisableSGXInterrupts(psSysData);

#if defined(LDM_PLATFORM) && !defined(PVR_DRI_DRM_NOT_PCI)
	{
		int res = pm_runtime_put_sync(&gpsPVRLDMDev->dev);
		if (res < 0)
		{
			PVR_DPF((PVR_DBG_ERROR, "DisableSGXClocks: pm_runtime_put_sync failed (%d)", -res));
		}
	}
#endif


	atomic_set(&psSysSpecData->sSGXClocksEnabled, 0);

#else
	PVR_UNREFERENCED_PARAMETER(psSysData);
#endif
}

#if (defined(DEBUG) || defined(TIMING)) && !defined(PVR_NO_OMAP_TIMER)
#if defined(PVR_OMAP_USE_DM_TIMER_API)
#define	GPTIMER_TO_USE 11
static PVRSRV_ERROR AcquireGPTimer(SYS_SPECIFIC_DATA *psSysSpecData)
{
	PVR_ASSERT(psSysSpecData->psGPTimer == NULL);


	psSysSpecData->psGPTimer = omap_dm_timer_request_specific(GPTIMER_TO_USE);
	if (psSysSpecData->psGPTimer == NULL)
	{

		PVR_DPF((PVR_DBG_WARNING, "%s: omap_dm_timer_request_specific failed", __FUNCTION__));
		return PVRSRV_ERROR_CLOCK_REQUEST_FAILED;
	}


	omap_dm_timer_set_source(psSysSpecData->psGPTimer, OMAP_TIMER_SRC_SYS_CLK);
	omap_dm_timer_enable(psSysSpecData->psGPTimer);


	omap_dm_timer_set_load_start(psSysSpecData->psGPTimer, 1, 0);

	omap_dm_timer_start(psSysSpecData->psGPTimer);


	psSysSpecData->sTimerRegPhysBase.uiAddr = SYS_OMAP5430_GP11TIMER_REGS_SYS_PHYS_BASE;

	return PVRSRV_OK;
}

static void ReleaseGPTimer(SYS_SPECIFIC_DATA *psSysSpecData)
{
	if (psSysSpecData->psGPTimer != NULL)
	{

		(void) omap_dm_timer_stop(psSysSpecData->psGPTimer);

		omap_dm_timer_disable(psSysSpecData->psGPTimer);

		omap_dm_timer_free(psSysSpecData->psGPTimer);

		psSysSpecData->sTimerRegPhysBase.uiAddr = 0;

		psSysSpecData->psGPTimer = NULL;
	}

}
#else
static PVRSRV_ERROR AcquireGPTimer(SYS_SPECIFIC_DATA *psSysSpecData)
{
#if defined(PVR_OMAP5_TIMING_PRCM)
	struct clk *psCLK;
	IMG_INT res;
	struct clk *sys_ck;
	IMG_INT rate;
#endif
	PVRSRV_ERROR eError;

	IMG_CPU_PHYADDR sTimerRegPhysBase;
	IMG_HANDLE hTimerEnable;
	IMG_UINT32 *pui32TimerEnable;

	PVR_ASSERT(psSysSpecData->sTimerRegPhysBase.uiAddr == 0);

#if defined(PVR_OMAP5_TIMING_PRCM)

	psCLK = clk_get(NULL, "gpt11_fck");
	if (IS_ERR(psCLK))
	{
		PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: Couldn't get GPTIMER11 functional clock"));
		goto ExitError;
	}
	psSysSpecData->psGPT11_FCK = psCLK;

	psCLK = clk_get(NULL, "gpt11_ick");
	if (IS_ERR(psCLK))
	{
		PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: Couldn't get GPTIMER11 interface clock"));
		goto ExitError;
	}
	psSysSpecData->psGPT11_ICK = psCLK;

	sys_ck = clk_get(NULL, "sys_clkin_ck");
	if (IS_ERR(sys_ck))
	{
		PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: Couldn't get System clock"));
		goto ExitError;
	}

	if(clk_get_parent(psSysSpecData->psGPT11_FCK) != sys_ck)
	{
		PVR_TRACE(("Setting GPTIMER11 parent to System Clock"));
		res = clk_set_parent(psSysSpecData->psGPT11_FCK, sys_ck);
		if (res < 0)
		{
			PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: Couldn't set GPTIMER11 parent clock (%d)", res));
		goto ExitError;
		}
	}

	rate = clk_get_rate(psSysSpecData->psGPT11_FCK);
	PVR_TRACE(("GPTIMER11 clock is %dMHz", HZ_TO_MHZ(rate)));

	res = clk_enable(psSysSpecData->psGPT11_FCK);
	if (res < 0)
	{
		PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: Couldn't enable GPTIMER11 functional clock (%d)", res));
		goto ExitError;
	}

	res = clk_enable(psSysSpecData->psGPT11_ICK);
	if (res < 0)
	{
		PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: Couldn't enable GPTIMER11 interface clock (%d)", res));
		goto ExitDisableGPT11FCK;
	}
#endif


	sTimerRegPhysBase.uiAddr = SYS_OMAP5430_GP11TIMER_TSICR_SYS_PHYS_BASE;
	pui32TimerEnable = OSMapPhysToLin(sTimerRegPhysBase,
	              4,
	              PVRSRV_HAP_KERNEL_ONLY|PVRSRV_HAP_UNCACHED,
	              &hTimerEnable);

	if (pui32TimerEnable == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: OSMapPhysToLin failed"));
		goto ExitDisableGPT11ICK;
	}

	if(!(*pui32TimerEnable & 4))
	{
		PVR_TRACE(("Setting GPTIMER11 mode to posted (currently is non-posted)"));


		*pui32TimerEnable |= 4;
	}

	OSUnMapPhysToLin(pui32TimerEnable,
		    4,
		    PVRSRV_HAP_KERNEL_ONLY|PVRSRV_HAP_UNCACHED,
		    hTimerEnable);


	sTimerRegPhysBase.uiAddr = SYS_OMAP5430_GP11TIMER_ENABLE_SYS_PHYS_BASE;
	pui32TimerEnable = OSMapPhysToLin(sTimerRegPhysBase,
	              4,
	              PVRSRV_HAP_KERNEL_ONLY|PVRSRV_HAP_UNCACHED,
	              &hTimerEnable);

	if (pui32TimerEnable == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "EnableSystemClocks: OSMapPhysToLin failed"));
		goto ExitDisableGPT11ICK;
	}


	*pui32TimerEnable = 3;

	OSUnMapPhysToLin(pui32TimerEnable,
		    4,
		    PVRSRV_HAP_KERNEL_ONLY|PVRSRV_HAP_UNCACHED,
		    hTimerEnable);

	psSysSpecData->sTimerRegPhysBase = sTimerRegPhysBase;

	eError = PVRSRV_OK;

	goto Exit;

ExitDisableGPT11ICK:
#if defined(PVR_OMAP5_TIMING_PRCM)
	clk_disable(psSysSpecData->psGPT11_ICK);
ExitDisableGPT11FCK:
	clk_disable(psSysSpecData->psGPT11_FCK);
ExitError:
#endif
	eError = PVRSRV_ERROR_CLOCK_REQUEST_FAILED;
Exit:
	return eError;
}

static void ReleaseGPTimer(SYS_SPECIFIC_DATA *psSysSpecData)
{
	IMG_HANDLE hTimerDisable;
	IMG_UINT32 *pui32TimerDisable;

	if (psSysSpecData->sTimerRegPhysBase.uiAddr == 0)
	{
		return;
	}


	pui32TimerDisable = OSMapPhysToLin(psSysSpecData->sTimerRegPhysBase,
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

	psSysSpecData->sTimerRegPhysBase.uiAddr = 0;

#if defined(PVR_OMAP5_TIMING_PRCM)
	clk_disable(psSysSpecData->psGPT11_ICK);

	clk_disable(psSysSpecData->psGPT11_FCK);
#endif
}
#endif
#else
static PVRSRV_ERROR AcquireGPTimer(SYS_SPECIFIC_DATA *psSysSpecData)
{
	PVR_UNREFERENCED_PARAMETER(psSysSpecData);

	return PVRSRV_OK;
}
static void ReleaseGPTimer(SYS_SPECIFIC_DATA *psSysSpecData)
{
	PVR_UNREFERENCED_PARAMETER(psSysSpecData);
}
#endif

PVRSRV_ERROR EnableSystemClocks(SYS_DATA *psSysData)
{
	SYS_SPECIFIC_DATA *psSysSpecData = (SYS_SPECIFIC_DATA *) psSysData->pvSysSpecificData;

	PVR_TRACE(("EnableSystemClocks: Enabling System Clocks"));

	if (!psSysSpecData->bSysClocksOneTimeInit)
	{
		mutex_init(&psSysSpecData->sPowerLock);

		atomic_set(&psSysSpecData->sSGXClocksEnabled, 0);

		psSysSpecData->bSysClocksOneTimeInit = IMG_TRUE;
	}

	return AcquireGPTimer(psSysSpecData);
}

IMG_VOID DisableSystemClocks(SYS_DATA *psSysData)
{
	SYS_SPECIFIC_DATA *psSysSpecData = (SYS_SPECIFIC_DATA *) psSysData->pvSysSpecificData;

	PVR_TRACE(("DisableSystemClocks: Disabling System Clocks"));


	DisableSGXClocks(psSysData);

	ReleaseGPTimer(psSysSpecData);
}

PVRSRV_ERROR SysPMRuntimeRegister(void)
{
#if defined(LDM_PLATFORM) && !defined(PVR_DRI_DRM_NOT_PCI)
	pm_runtime_enable(&gpsPVRLDMDev->dev);
#endif
	return PVRSRV_OK;
}

PVRSRV_ERROR SysPMRuntimeUnregister(void)
{
#if defined(LDM_PLATFORM) && !defined(PVR_DRI_DRM_NOT_PCI)
	pm_runtime_disable(&gpsPVRLDMDev->dev);
#endif
	return PVRSRV_OK;
}

static enum hrtimer_restart sgx_dvfs_idle_timer_func(struct hrtimer *timer)
{
	queue_work(system_unbound_wq, &gpsSysSpecificData->sgx_dvfs_idle_work);
	return HRTIMER_NORESTART;
}

static void sgx_dvfs_idle_work_func(struct work_struct *work)
{
	mutex_lock(&gpsSysSpecificData->sgx_dvfs_lock);

	RequestSGXFreq(gpsSysData, 0);

	mutex_unlock(&gpsSysSpecificData->sgx_dvfs_lock);
}

static enum hrtimer_restart sgx_dvfs_active_timer_func(struct hrtimer *timer)
{
	queue_work(system_unbound_wq, &gpsSysSpecificData->sgx_dvfs_active_work);
	return HRTIMER_NORESTART;
}

static void sgx_dvfs_active_work_func(struct work_struct *work)
{
	mutex_lock(&gpsSysSpecificData->sgx_dvfs_lock);

	/* Reset active DVFS counter if min(active, limit) is changing. */
	if(gpsSysSpecificData->ui32SGXFreqListIndexActive <
		gpsSysSpecificData->ui32SGXFreqListIndexLimit)
	{
		gpsSysSpecificData->counter = 0;
	}

	gpsSysSpecificData->ui32SGXFreqListIndexActive =
		gpsSysSpecificData->ui32SGXFreqListSize - 2;

	/* Increase frequency now only if active. */
	if(!gpsSysSpecificData->sgx_is_idle)
	{
		RequestSGXFreq(gpsSysData,
			gpsSysSpecificData->ui32SGXFreqListIndexLimit);
	}

	mutex_unlock(&gpsSysSpecificData->sgx_dvfs_lock);
}

#ifdef CONFIG_OMAP_GPU_GOVERNOR
static int sgx_tmfw_cool_dev(struct thermal_dev *dev, int cooling_level)
{
	mutex_lock(&gpsSysSpecificData->sgx_dvfs_lock);

	if((cooling_level > gpsSysSpecificData->cooling_level) &&
		(gpsSysSpecificData->ui32SGXFreqListIndexLimit > 0))
	{
		gpsSysSpecificData->ui32SGXFreqListIndexLimit--;

		/* Reduce frequency if needed */
		if(gpsSysSpecificData->ui32SGXFreqListIndexLimit <
			gpsSysSpecificData->ui32SGXFreqListIndex)
		{
			RequestSGXFreq(gpsSysData,
				gpsSysSpecificData->ui32SGXFreqListIndexLimit);
		}

		/* Reset active DVFS counter if min(active, limit) changed. */
		if(gpsSysSpecificData->ui32SGXFreqListIndexActive >
			gpsSysSpecificData->ui32SGXFreqListIndexLimit)
		{
			gpsSysSpecificData->counter = 0;
		}
	}
	else if((cooling_level < gpsSysSpecificData->cooling_level) &&
		(gpsSysSpecificData->ui32SGXFreqListIndexLimit <
		gpsSysSpecificData->ui32SGXFreqListSize - 2))
	{
		gpsSysSpecificData->ui32SGXFreqListIndexLimit++;

		/* Increase frequency now only if active. */
		if(!gpsSysSpecificData->sgx_is_idle)
		{
			RequestSGXFreq(gpsSysData,
				min(gpsSysSpecificData->ui32SGXFreqListIndexActive,
				gpsSysSpecificData->ui32SGXFreqListIndexLimit));
		}
		/* Reset active DVFS counter if min(active, limit) changed. */
		if(gpsSysSpecificData->ui32SGXFreqListIndexActive >=
			gpsSysSpecificData->ui32SGXFreqListIndexLimit)
		{
			gpsSysSpecificData->counter = 0;
		}
	}
	gpsSysSpecificData->cooling_level = cooling_level;

	mutex_unlock(&gpsSysSpecificData->sgx_dvfs_lock);

	PVR_DPF((PVR_DBG_THERMAL, "cooling level = %d, limit freq = %d ",
		cooling_level, gpsSysSpecificData->pui32SGXFreqList[gpsSysSpecificData->
		ui32SGXFreqListIndexLimit]));

	return 0;
}

static struct thermal_dev_ops sgx_tmfw_dev_ops = {
	.cool_device = sgx_tmfw_cool_dev,
};

static struct thermal_dev sgx_tmfw_dev = {
	.name = "gpu_cooling",
	.domain_name = "gpu",
	.dev_ops = &sgx_tmfw_dev_ops,
};
#endif

PVRSRV_ERROR SysDvfsInitialize(SYS_SPECIFIC_DATA *psSysSpecificData)
{
	IMG_UINT32 i, *freq_list;
	IMG_INT32 opp_count;
	unsigned long freq;
	struct opp *opp;

	/**
	 * We query and store the list of SGX frequencies just this once under the
	 * assumption that they are unchanging, e.g. no disabling of high frequency
	 * option for thermal management. This is currently valid for 4430 and 4460.
	 */
	rcu_read_lock();
	opp_count = opp_get_opp_count(&gpsPVRLDMDev->dev);
	if (opp_count < 1)
	{
		rcu_read_unlock();
		PVR_DPF((PVR_DBG_ERROR, "SysDvfsInitialize: Could not retrieve opp count"));
		return PVRSRV_ERROR_NOT_SUPPORTED;
	}

	/**
	 * Allocate the frequency list with a slot for each available frequency plus
	 * one additional slot to hold a designated frequency value to assume when in
	 * an unknown frequency state.
	 */
	freq_list = kmalloc((opp_count + 1) * sizeof(IMG_UINT32), GFP_ATOMIC);
	if (!freq_list)
	{
		rcu_read_unlock();
		PVR_DPF((PVR_DBG_ERROR, "SysDvfsInitialize: Could not allocate frequency list"));
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	/**
	 * Fill in frequency list from lowest to highest then finally the "unknown"
	 * frequency value. We use the highest available frequency as our assumed value
	 * when in an unknown state, because it is safer for APM and hardware recovery
	 * timers to be longer than intended rather than shorter.
	 */
	freq = 0;
	for (i = 0; i < opp_count; i++)
	{
		opp = opp_find_freq_ceil(&gpsPVRLDMDev->dev, &freq);
		if (IS_ERR_OR_NULL(opp))
		{
			rcu_read_unlock();
			PVR_DPF((PVR_DBG_ERROR, "SysDvfsInitialize: Could not retrieve opp level %d", i));
			kfree(freq_list);
			return PVRSRV_ERROR_NOT_SUPPORTED;
		}
		freq_list[i] = (IMG_UINT32)freq;
		freq++;
	}
	rcu_read_unlock();
	freq_list[opp_count] = freq_list[opp_count - 1];

	psSysSpecificData->ui32SGXFreqListSize = opp_count + 1;
	psSysSpecificData->pui32SGXFreqList = freq_list;
	/* Start in unknown state - no frequency request to DVFS yet made */
	psSysSpecificData->ui32SGXFreqListIndex = opp_count;
	/* Initialize target active frequency and limit frequency to max */
	psSysSpecificData->ui32SGXFreqListIndexActive = opp_count - 1;
	psSysSpecificData->ui32SGXFreqListIndexLimit = opp_count - 1;

	hrtimer_init(&psSysSpecificData->sgx_dvfs_idle_timer, HRTIMER_BASE_MONOTONIC,
			HRTIMER_MODE_REL);
	psSysSpecificData->sgx_dvfs_idle_timer.function = sgx_dvfs_idle_timer_func;
	INIT_WORK(&psSysSpecificData->sgx_dvfs_idle_work, sgx_dvfs_idle_work_func);

	hrtimer_init(&psSysSpecificData->sgx_dvfs_active_timer, HRTIMER_BASE_MONOTONIC,
			HRTIMER_MODE_REL);
	psSysSpecificData->sgx_dvfs_active_timer.function = sgx_dvfs_active_timer_func;
	INIT_WORK(&psSysSpecificData->sgx_dvfs_active_work, sgx_dvfs_active_work_func);

	mutex_init(&psSysSpecificData->sgx_dvfs_lock);

	psSysSpecificData->sgx_idle_stamp = ktime_set(0, 0);
	psSysSpecificData->sgx_active_stamp = ktime_set(0, 0);
	psSysSpecificData->sgx_work_stamp = ktime_set(0, 0);
	psSysSpecificData->dss_return_stamp = ktime_set(0, 0);
	psSysSpecificData->sgx_is_idle = true;
	psSysSpecificData->dss_kick_is_pending = false;
	psSysSpecificData->sgx_active_kickcmd = SGXMKIF_CMD_MAX;
	psSysSpecificData->counter = 0;

#ifdef CONFIG_OMAP_GPU_GOVERNOR
	if(thermal_cooling_dev_register(&sgx_tmfw_dev))
	{
		PVR_DPF((PVR_DBG_ERROR, "SysDvfsInitialize: Could not register with thermal framework"));
		kfree(psSysSpecificData->pui32SGXFreqList);
		psSysSpecificData->pui32SGXFreqList = 0;
		psSysSpecificData->ui32SGXFreqListSize = 0;
		psSysSpecificData->ui32SGXFreqListIndexActive = 0;
		psSysSpecificData->ui32SGXFreqListIndexLimit = 0;
		return PVRSRV_ERROR_NOT_SUPPORTED;
	}
#endif

	return PVRSRV_OK;
}

PVRSRV_ERROR SysDvfsDeinitialize(SYS_SPECIFIC_DATA *psSysSpecificData)
{
#ifdef CONFIG_OMAP_GPU_GOVERNOR
	thermal_cooling_dev_unregister(&sgx_tmfw_dev);
#endif

	hrtimer_cancel(&gpsSysSpecificData->sgx_dvfs_idle_timer);
	cancel_work_sync(&gpsSysSpecificData->sgx_dvfs_idle_work);
	hrtimer_cancel(&gpsSysSpecificData->sgx_dvfs_active_timer);
	cancel_work_sync(&gpsSysSpecificData->sgx_dvfs_active_work);

	mutex_lock(&gpsSysSpecificData->sgx_dvfs_lock);

	RequestSGXFreq(gpsSysData, 0);

	mutex_unlock(&gpsSysSpecificData->sgx_dvfs_lock);

	kfree(psSysSpecificData->pui32SGXFreqList);
	psSysSpecificData->pui32SGXFreqList = 0;
	psSysSpecificData->ui32SGXFreqListSize = 0;
	psSysSpecificData->ui32SGXFreqListIndexActive = 0;
	psSysSpecificData->ui32SGXFreqListIndexLimit = 0;

	return PVRSRV_OK;
}

#if defined(SUPPORT_DRI_DRM_PLUGIN)
static struct omap_gpu_plugin sOMAPGPUPlugin;

#define	SYS_DRM_SET_PLUGIN_FIELD(d, s, f) (d)->f = (s)->f
int
SysDRMRegisterPlugin(PVRSRV_DRM_PLUGIN *psDRMPlugin)
{
	int iRes;

	SYS_DRM_SET_PLUGIN_FIELD(&sOMAPGPUPlugin, psDRMPlugin, name);
	SYS_DRM_SET_PLUGIN_FIELD(&sOMAPGPUPlugin, psDRMPlugin, open);
	SYS_DRM_SET_PLUGIN_FIELD(&sOMAPGPUPlugin, psDRMPlugin, load);
	SYS_DRM_SET_PLUGIN_FIELD(&sOMAPGPUPlugin, psDRMPlugin, unload);
	SYS_DRM_SET_PLUGIN_FIELD(&sOMAPGPUPlugin, psDRMPlugin, release);
	SYS_DRM_SET_PLUGIN_FIELD(&sOMAPGPUPlugin, psDRMPlugin, mmap);
	SYS_DRM_SET_PLUGIN_FIELD(&sOMAPGPUPlugin, psDRMPlugin, ioctls);
	SYS_DRM_SET_PLUGIN_FIELD(&sOMAPGPUPlugin, psDRMPlugin, num_ioctls);
	SYS_DRM_SET_PLUGIN_FIELD(&sOMAPGPUPlugin, psDRMPlugin, ioctl_start);

	iRes = omap_gpu_register_plugin(&sOMAPGPUPlugin);
	if (iRes != 0)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: omap_gpu_register_plugin failed (%d)", __FUNCTION__, iRes));
	}

	return iRes;
}

void
SysDRMUnregisterPlugin(PVRSRV_DRM_PLUGIN *psDRMPlugin)
{
	int iRes = omap_gpu_unregister_plugin(&sOMAPGPUPlugin);
	if (iRes != 0)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: omap_gpu_unregister_plugin failed (%d)", __FUNCTION__, iRes));
	}
}
#endif
