/*
 * tiomap_sm.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
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

#include <dspbridge/dbdefs.h>
#include <dspbridge/errbase.h>

#include <dspbridge/cfg.h>
#include <dspbridge/drv.h>
#include <dspbridge/dev.h>

#include <dspbridge/dbg.h>

#include "_tiomap.h"
#include "_tiomap_pwr.h"


DSP_STATUS CHNLSM_InterruptDSP2(struct WMD_DEV_CONTEXT *pDevContext,
				u16 wMbVal)
{
	DSP_STATUS status = DSP_SOK;
	u32 temp;

	if (!pDevContext->mbox)
		return DSP_SOK;

#ifdef CONFIG_BRIDGE_DVFS
	if (pDevContext->dwBrdState == BRD_DSP_HIBERNATION ||
	    pDevContext->dwBrdState == BRD_HIBERNATION) {
		if (!in_atomic())
			tiomap3430_bump_dsp_opp_level();
	}
#endif

	if (pDevContext->dwBrdState == BRD_DSP_HIBERNATION ||
	    pDevContext->dwBrdState == BRD_HIBERNATION) {
		/* Restart the peripheral clocks */
		DSP_PeripheralClocks_Enable(pDevContext, NULL);

		/* Restore mailbox settings */
		/* Enabling Dpll in lock mode*/
		temp = (u32) *((REG_UWORD32 *)
				((u32) (pDevContext->cmbase) + 0x34));
		temp = (temp & 0xFFFFFFFE) | 0x1;
		*((REG_UWORD32 *) ((u32) (pDevContext->cmbase) + 0x34)) =
			(u32) temp;
		temp = (u32) *((REG_UWORD32 *)
				((u32) (pDevContext->cmbase) + 0x4));
		temp = (temp & 0xFFFFFC8) | 0x37;

		*((REG_UWORD32 *) ((u32) (pDevContext->cmbase) + 0x4)) =
			(u32) temp;
		omap_mbox_restore_ctx(pDevContext->mbox);

		/*  Access MMU SYS CONFIG register to generate a short wakeup */
		temp = (u32) *((REG_UWORD32 *) ((u32)
						(pDevContext->dwDSPMmuBase) + 0x10));

		pDevContext->dwBrdState = BRD_RUNNING;
	} else if (pDevContext->dwBrdState == BRD_RETENTION)
		/* Restart the peripheral clocks */
		DSP_PeripheralClocks_Enable(pDevContext, NULL);

	status = omap_mbox_msg_send(pDevContext->mbox, wMbVal, NULL);

	if (status) {
		pr_err("omap_mbox_msg_send Fail and status = %d\n", status);
		status = DSP_EFAIL;
	}
	DBG_Trace(DBG_LEVEL3, "writing %x to Mailbox\n", wMbVal);

	return DSP_SOK;
}

