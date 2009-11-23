/*
 * mmu_fault.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Implements DSP MMU fault handling functions.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/*  ----------------------------------- DSP/BIOS Bridge */
#include <dspbridge/std.h>
#include <dspbridge/dbdefs.h>
#include <dspbridge/errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <dspbridge/host_os.h>
#include <dspbridge/dbc.h>
#include <dspbridge/dbg.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/dpc.h>
#include <dspbridge/mem.h>
#include <dspbridge/drv.h>

/*  ----------------------------------- Link Driver */
#include <dspbridge/wmddeh.h>

/* ------------------------------------ Hardware Abstraction Layer */
#include <hw_defs.h>
#include <hw_mmu.h>

/*  ----------------------------------- This */
#include "_deh.h"
#include <dspbridge/cfg.h>
#include "_tiomap_mmu.h"
#include "_tiomap.h"
#include "mmu_fault.h"

static u32 dmmuEventMask;
u32 faultAddr;

static bool MMU_CheckIfFault(struct WMD_DEV_CONTEXT *pDevContext);

/*
 *  ======== MMU_FaultDpc ========
 *      Deferred procedure call to handle DSP MMU fault.
 */
void MMU_FaultDpc(IN unsigned long pRefData)
{
	struct DEH_MGR *hDehMgr = (struct DEH_MGR *)pRefData;
	struct DEH_MGR *pDehMgr = (struct DEH_MGR *)hDehMgr;

	DBG_Trace(DBG_LEVEL1, "MMU_FaultDpc Enter: 0x%x\n", pRefData);

	if (pDehMgr)
		WMD_DEH_Notify(hDehMgr, DSP_MMUFAULT, 0L);

	DBG_Trace(DBG_LEVEL1, "MMU_FaultDpc Exit: 0x%x\n", pRefData);
}

/*
 *  ======== MMU_FaultIsr ========
 *      ISR to be triggered by a DSP MMU fault interrupt.
 */
irqreturn_t  MMU_FaultIsr(int irq, IN void *pRefData)
{
	struct DEH_MGR *pDehMgr = (struct DEH_MGR *)pRefData;
	struct WMD_DEV_CONTEXT *pDevContext;
	DSP_STATUS status = DSP_SOK;
	unsigned long flags;

	DBG_Trace(DBG_LEVEL1, "Entering DEH_DspMmuIsr: 0x%x\n", pRefData);
       DBC_Require(irq == INT_DSP_MMU_IRQ);
	DBC_Require(MEM_IsValidHandle(pDehMgr, SIGNATURE));

	if (MEM_IsValidHandle(pDehMgr, SIGNATURE)) {

		pDevContext = (struct WMD_DEV_CONTEXT *)pDehMgr->hWmdContext;
		if (DSP_FAILED(status))
			DBG_Trace(DBG_LEVEL7,
				 "**Failed to get Host Resources "
				 "in MMU ISR **\n");
		if (MMU_CheckIfFault(pDevContext)) {
			printk(KERN_INFO "***** DSPMMU FAULT ***** IRQStatus "
				"0x%x\n", dmmuEventMask);
			printk(KERN_INFO "***** DSPMMU FAULT ***** faultAddr "
				"0x%x\n", faultAddr);
			/*
			 * Schedule a DPC directly. In the future, it may be
			 * necessary to check if DSP MMU fault is intended for
			 * Bridge.
			 */
			/* Increment count of DPC's pending. */
			spin_lock_irqsave(&pDehMgr->hMmuFaultDpc->dpc_lock,
						flags);
			pDehMgr->hMmuFaultDpc->numRequested++;
			spin_unlock_irqrestore(&pDehMgr->hMmuFaultDpc->dpc_lock,
						flags);

			/* Schedule DPC */
			tasklet_schedule(&pDehMgr->hMmuFaultDpc->dpc_tasklet);
#ifdef DEBUG
			if (pDehMgr->hMmuFaultDpc->numRequested >
			   pDehMgr->hMmuFaultDpc->numScheduled +
			   pDehMgr->hMmuFaultDpc->numRequestedMax) {
				pDehMgr->hMmuFaultDpc->numRequestedMax =
					pDehMgr->hMmuFaultDpc->numRequested -
					pDehMgr->hMmuFaultDpc->numScheduled;
			}
#endif

			/* Reset errInfo structure before use. */
			pDehMgr->errInfo.dwErrMask = DSP_MMUFAULT;
			pDehMgr->errInfo.dwVal1 = faultAddr >> 16;
			pDehMgr->errInfo.dwVal2 = faultAddr & 0xFFFF;
			pDehMgr->errInfo.dwVal3 = 0L;
			/* Disable the MMU events, else once we clear it will
			 * start to raise INTs again */
			HW_MMU_EventDisable(pDevContext->dwDSPMmuBase,
					    HW_MMU_TRANSLATION_FAULT);
		} else {
			DBG_Trace(DBG_LEVEL7,
				 "***** MMU FAULT ***** faultcode 0x%x\n",
				 dmmuEventMask);
			HW_MMU_EventDisable(pDevContext->dwDSPMmuBase,
					    HW_MMU_ALL_INTERRUPTS);
		}
	}
       return IRQ_HANDLED;
}


/*
 *  ======== MMU_CheckIfFault ========
 *      Check to see if MMU Fault is valid TLB miss from DSP
 *  Note: This function is called from an ISR
 */
static bool MMU_CheckIfFault(struct WMD_DEV_CONTEXT *pDevContext)
{


	bool retVal = false;
	DSP_STATUS status = DSP_SOK;
	HW_STATUS hwStatus;

	if (DSP_FAILED(status))
		DBG_Trace(DBG_LEVEL7, "**Failed to get Host Resources in "
			 "MMU_CheckIfFault **\n");

	hwStatus = HW_MMU_EventStatus(pDevContext->dwDSPMmuBase,
					&dmmuEventMask);
	if (dmmuEventMask  ==  HW_MMU_TRANSLATION_FAULT) {
		HW_MMU_FaultAddrRead(pDevContext->dwDSPMmuBase, &faultAddr);
		DBG_Trace(DBG_LEVEL1, "WMD_DEH_Notify: DSP_MMUFAULT, fault "
			 "address = 0x%x\n", faultAddr);
		retVal = true;
	}
	return retVal;
}
