/*
 * cfg.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Implementation of platform specific config services.
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
#include <dspbridge/dbc.h>
#include <dspbridge/gt.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/reg.h>

/*  ----------------------------------- This */
#include <dspbridge/cfg.h>
#include <dspbridge/list.h>

struct DRV_EXT {
	struct LST_ELEM link;
	char szString[MAXREGPATHLENGTH];
};

/*  ----------------------------------- Globals */
#if GT_TRACE
static struct GT_Mask CFG_debugMask = { NULL, NULL };	/* CFG debug Mask */
#endif

/*
 *  ======== CFG_Exit ========
 *  Purpose:
 *      Discontinue usage of the CFG module.
 */
void CFG_Exit(void)
{
	GT_0trace(CFG_debugMask, GT_5CLASS, "Entered CFG_Exit\n");
}

/*
 *  ======== CFG_GetAutoStart ========
 *  Purpose:
 *      Retreive the autostart mask, if any, for this board.
 */
DSP_STATUS CFG_GetAutoStart(struct CFG_DEVNODE *hDevNode,
			    OUT u32 *pdwAutoStart)
{
	DSP_STATUS status = DSP_SOK;
	u32 dwBufSize;
	GT_2trace(CFG_debugMask, GT_ENTER,
		  "Entered CFG_GetAutoStart: \n\thDevNode:"
		  "0x%x\n\tpdwAutoStart: 0x%x\n", hDevNode, pdwAutoStart);
	dwBufSize = sizeof(*pdwAutoStart);
	if (!hDevNode)
		status = CFG_E_INVALIDHDEVNODE;
	if (!pdwAutoStart)
		status = CFG_E_INVALIDPOINTER;
	if (DSP_SUCCEEDED(status)) {
		status = REG_GetValue(AUTOSTART, (u8 *)pdwAutoStart,
					&dwBufSize);
		if (DSP_FAILED(status))
			status = CFG_E_RESOURCENOTAVAIL;
	}
#ifdef CONFIG_BRIDGE_DEBUG
	if (DSP_SUCCEEDED(status)) {
		GT_0trace(CFG_debugMask, GT_1CLASS,
			 "CFG_GetAutoStart SUCCESS \n");
	} else {
		GT_0trace(CFG_debugMask, GT_6CLASS,
		"CFG_GetAutoStart Failed \n");
	}
#endif
	DBC_Ensure((status == DSP_SOK &&
		(*pdwAutoStart == 0 || *pdwAutoStart == 1))
		|| status != DSP_SOK);
	return status;
}

/*
 *  ======== CFG_GetDevObject ========
 *  Purpose:
 *      Retrieve the Device Object handle for a given devnode.
 */
DSP_STATUS CFG_GetDevObject(struct CFG_DEVNODE *hDevNode, OUT u32 *pdwValue)
{
	DSP_STATUS status = DSP_SOK;
	u32 dwBufSize;
	GT_2trace(CFG_debugMask, GT_ENTER, "Entered CFG_GetDevObject, args: "
		 "\n\thDevNode: 0x%x\n\tpdwValue: 0x%x\n", hDevNode,
		 *pdwValue);
	if (!hDevNode)
		status = CFG_E_INVALIDHDEVNODE;

	if (!pdwValue)
		status = CFG_E_INVALIDHDEVNODE;

	dwBufSize = sizeof(pdwValue);
	if (DSP_SUCCEEDED(status)) {

		/* check the device string and then call the REG_SetValue*/
		if (!(strcmp((char *)((struct DRV_EXT *)hDevNode)->szString,
							"TIOMAP1510"))) {
			GT_0trace(CFG_debugMask, GT_1CLASS,
				  "Fetching DSP Device from "
				  "Registry \n");
			status = REG_GetValue("DEVICE_DSP", (u8 *)pdwValue,
						&dwBufSize);
		} else {
			GT_0trace(CFG_debugMask, GT_6CLASS,
				  "Failed to Identify the Device to Fetch \n");
		}
	}
#ifdef CONFIG_BRIDGE_DEBUG
	if (DSP_SUCCEEDED(status)) {
		GT_1trace(CFG_debugMask, GT_1CLASS,
			  "CFG_GetDevObject SUCCESS DevObject"
			  ": 0x%x\n ", *pdwValue);
	} else {
		GT_0trace(CFG_debugMask, GT_6CLASS,
			  "CFG_GetDevObject Failed \n");
	}
#endif
	return status;
}

/*
 *  ======== CFG_GetDSPResources ========
 *  Purpose:
 *      Get the DSP resources available to a given device.
 */
DSP_STATUS CFG_GetDSPResources(struct CFG_DEVNODE *hDevNode,
			       OUT struct CFG_DSPRES *pDSPResTable)
{
	DSP_STATUS status = DSP_SOK;	/* return value */
	u32 dwResSize;
	GT_2trace(CFG_debugMask, GT_ENTER,
		  "Entered CFG_GetDSPResources, args: "
		  "\n\thDevNode:  0x%x\n\tpDSPResTable:  0x%x\n",
		  hDevNode, pDSPResTable);
	if (!hDevNode)
		status = CFG_E_INVALIDHDEVNODE;
	else if (!pDSPResTable)
		status = CFG_E_INVALIDPOINTER;
	else
		status = REG_GetValue(DSPRESOURCES, (u8 *)pDSPResTable,
					&dwResSize);
	if (DSP_SUCCEEDED(status)) {
		GT_0trace(CFG_debugMask, GT_1CLASS,
			  "CFG_GetDSPResources SUCCESS\n");
	} else {
		status = CFG_E_RESOURCENOTAVAIL;
		GT_0trace(CFG_debugMask, GT_6CLASS,
			  "CFG_GetDSPResources Failed \n");
	}
#ifdef CONFIG_BRIDGE_DEBUG
	/* assert that resource values are reasonable */
	DBC_Assert(pDSPResTable->uChipType < 256);
	DBC_Assert(pDSPResTable->uWordSize > 0);
	DBC_Assert(pDSPResTable->uWordSize < 32);
	DBC_Assert(pDSPResTable->cChips > 0);
	DBC_Assert(pDSPResTable->cChips < 256);
#endif
	return status;
}

/*
 *  ======== CFG_GetExecFile ========
 *  Purpose:
 *      Retreive the default executable, if any, for this board.
 */
DSP_STATUS CFG_GetExecFile(struct CFG_DEVNODE *hDevNode, u32 ulBufSize,
			   OUT char *pstrExecFile)
{
	DSP_STATUS status = DSP_SOK;
	u32 cExecSize = ulBufSize;
	GT_3trace(CFG_debugMask, GT_ENTER,
		  "Entered CFG_GetExecFile:\n\tthDevNode: "
		  "0x%x\n\tulBufSize: 0x%x\n\tpstrExecFile: 0x%x\n", hDevNode,
		  ulBufSize, pstrExecFile);
	if (!hDevNode)
		status = CFG_E_INVALIDHDEVNODE;
	else if (!pstrExecFile)
		status = CFG_E_INVALIDPOINTER;

	if (DSP_SUCCEEDED(status)) {
		status = REG_GetValue(DEFEXEC, (u8 *)pstrExecFile, &cExecSize);
		if (DSP_FAILED(status))
			status = CFG_E_RESOURCENOTAVAIL;
		else if (cExecSize > ulBufSize)
			status = DSP_ESIZE;

	}
#ifdef CONFIG_BRIDGE_DEBUG
	if (DSP_SUCCEEDED(status)) {
		GT_1trace(CFG_debugMask, GT_1CLASS,
			  "CFG_GetExecFile SUCCESS Exec File"
			  "name : %s\n ", pstrExecFile);
	} else {
		GT_0trace(CFG_debugMask, GT_6CLASS,
			  "CFG_GetExecFile Failed \n");
	}
#endif
	DBC_Ensure(((status == DSP_SOK) &&
		(strlen(pstrExecFile) <= ulBufSize)) || (status != DSP_SOK));
	return status;
}

/*
 *  ======== CFG_GetHostResources ========
 *  Purpose:
 *      Get the Host allocated resources assigned to a given device.
 */
DSP_STATUS CFG_GetHostResources(struct CFG_DEVNODE *hDevNode,
				OUT struct CFG_HOSTRES *pHostResTable)
{
	DSP_STATUS status = DSP_SOK;
	u32 dwBufSize;
	GT_2trace(CFG_debugMask, GT_ENTER,
		  "Entered CFG_GetHostResources, args:\n\t"
		  "pHostResTable:  0x%x\n\thDevNode:  0x%x\n",
		  pHostResTable, hDevNode);
	if (!hDevNode)
		status = CFG_E_INVALIDHDEVNODE;

	if (!pHostResTable)
		status = CFG_E_INVALIDPOINTER;

	if (DSP_SUCCEEDED(status)) {
		dwBufSize = sizeof(struct CFG_HOSTRES);
		if (DSP_FAILED(REG_GetValue(CURRENTCONFIG, (u8 *)pHostResTable,
						&dwBufSize))) {
			status = CFG_E_RESOURCENOTAVAIL;
		}
	}
#ifdef CONFIG_BRIDGE_DEBUG
	if (DSP_SUCCEEDED(status)) {
		GT_0trace(CFG_debugMask, GT_1CLASS,
			  "CFG_GetHostResources SUCCESS \n");
	} else {
		GT_0trace(CFG_debugMask, GT_6CLASS,
			  "CFG_GetHostResources Failed \n");
	}
#endif
	return status;
}

/*
 *  ======== CFG_GetObject ========
 *  Purpose:
 *      Retrieve the Object handle from the Registry
 */
DSP_STATUS CFG_GetObject(OUT u32 *pdwValue, u32 dwType)
{
	DSP_STATUS status = DSP_EINVALIDARG;
	u32 dwBufSize;
	DBC_Require(pdwValue != NULL);
	GT_1trace(CFG_debugMask, GT_ENTER,
		 "Entered CFG_GetObject, args:pdwValue: "
		 "0x%x\n", *pdwValue);
	dwBufSize = sizeof(pdwValue);
	switch (dwType) {
	case (REG_DRV_OBJECT):
		status = REG_GetValue(DRVOBJECT, (u8 *)pdwValue, &dwBufSize);
		if (DSP_FAILED(status))
			status = CFG_E_RESOURCENOTAVAIL;
		break;
	case (REG_MGR_OBJECT):
		status = REG_GetValue(MGROBJECT, (u8 *)pdwValue, &dwBufSize);
		if (DSP_FAILED(status))
			status = CFG_E_RESOURCENOTAVAIL;
		break;
	default:
		break;
	}
	if (DSP_SUCCEEDED(status)) {
		GT_1trace(CFG_debugMask, GT_1CLASS,
			  "CFG_GetObject SUCCESS DrvObject: "
			  "0x%x\n ", *pdwValue);
	} else {
		*pdwValue = 0;
		GT_0trace(CFG_debugMask, GT_6CLASS, "CFG_GetObject Failed \n");
	}
	DBC_Ensure((DSP_SUCCEEDED(status) && *pdwValue != 0) ||
		   (DSP_FAILED(status) && *pdwValue == 0));
	return status;
}

/*
 *  ======== CFG_Init ========
 *  Purpose:
 *      Initialize the CFG module's private state.
 */
bool CFG_Init(void)
{
	struct CFG_DSPRES dspResources;
	GT_create(&CFG_debugMask, "CF");	/* CF for ConFig */
	GT_0trace(CFG_debugMask, GT_5CLASS, "Entered CFG_Init\n");
	GT_0trace(CFG_debugMask, GT_5CLASS, "Intializing DSP Registry Info \n");

	dspResources.uChipType = DSPTYPE_64;
	dspResources.cChips = 1;
	dspResources.uWordSize = DSPWORDSIZE;
	dspResources.cMemTypes = 0;
	dspResources.aMemDesc[0].uMemType = 0;
	dspResources.aMemDesc[0].ulMin = 0;
	dspResources.aMemDesc[0].ulMax = 0;
	if (DSP_SUCCEEDED(REG_SetValue(DSPRESOURCES, (u8 *)&dspResources,
				 sizeof(struct CFG_DSPRES)))) {
		GT_0trace(CFG_debugMask, GT_5CLASS,
			  "Initialized DSP resources in "
			  "Registry \n");
	} else
		GT_0trace(CFG_debugMask, GT_5CLASS,
			  "Failed to Initialize DSP resources"
			  " in Registry \n");
	return true;
}

/*
 *  ======== CFG_SetDevObject ========
 *  Purpose:
 *      Store the Device Object handle and devNode pointer for a given devnode.
 */
DSP_STATUS CFG_SetDevObject(struct CFG_DEVNODE *hDevNode, u32 dwValue)
{
	DSP_STATUS status = DSP_SOK;
	u32 dwBuffSize;
	GT_2trace(CFG_debugMask, GT_ENTER,
		  "Entered CFG_SetDevObject, args: \n\t"
		  "hDevNode: 0x%x\n\tdwValue: 0x%x\n", hDevNode, dwValue);
	if (!hDevNode)
		status = CFG_E_INVALIDHDEVNODE;

	dwBuffSize = sizeof(dwValue);
	if (DSP_SUCCEEDED(status)) {
		/* Store the WCD device object in the Registry */

		if (!(strcmp((char *)hDevNode, "TIOMAP1510"))) {
			GT_0trace(CFG_debugMask, GT_1CLASS,
				  "Registering the DSP Device \n");
			status = REG_SetValue("DEVICE_DSP", (u8 *)&dwValue,
						dwBuffSize);
		} else {
			GT_0trace(CFG_debugMask, GT_6CLASS,
				  "Failed to Register Device \n");
		}
	}
#ifdef CONFIG_BRIDGE_DEBUG
	if (DSP_SUCCEEDED(status)) {
		GT_0trace(CFG_debugMask, GT_1CLASS,
			  "CFG_SetDevObject SUCCESS \n");
	} else {
		GT_0trace(CFG_debugMask, GT_6CLASS,
			  "CFG_SetDevObject Failed \n");
	}
#endif
	return status;
}

/*
 *  ======== CFG_SetObject ========
 *  Purpose:
 *      Store the Driver Object handle
 */
DSP_STATUS CFG_SetObject(u32 dwValue, u32 dwType)
{
	DSP_STATUS status = DSP_EINVALIDARG;
	u32 dwBuffSize;
	GT_1trace(CFG_debugMask, GT_ENTER,
		  "Entered CFG_SetObject, args: dwValue: "
		  "0x%x\n", dwValue);
	dwBuffSize = sizeof(dwValue);
	switch (dwType) {
	case (REG_DRV_OBJECT):
		status = REG_SetValue(DRVOBJECT, (u8 *)&dwValue, dwBuffSize);
		break;
	case (REG_MGR_OBJECT):
		status = REG_SetValue(MGROBJECT, (u8 *) &dwValue, dwBuffSize);
		break;
	default:
		break;
	}
#ifdef CONFIG_BRIDGE_DEBUG
	if (DSP_SUCCEEDED(status))
		GT_0trace(CFG_debugMask, GT_1CLASS, "CFG_SetObject SUCCESS \n");
	else
		GT_0trace(CFG_debugMask, GT_6CLASS, "CFG_SetObject Failed \n");

#endif
	return status;
}
