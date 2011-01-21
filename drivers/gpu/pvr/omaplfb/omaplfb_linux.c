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

#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif

#include <linux/version.h>
#include <linux/module.h>
#include <linux/fb.h>
#include <asm/io.h>
#include <plat/vrfb.h>
#include <plat/display.h>

#ifdef RELEASE
#include <../drivers/video/omap2/omapfb/omapfb.h>
#undef DEBUG
#else
#undef DEBUG
#include <../drivers/video/omap2/omapfb/omapfb.h>
#endif

#if defined(LDM_PLATFORM)
#include <linux/platform_device.h>
#if defined(SGX_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#endif

#include "img_defs.h"
#include "servicesext.h"
#include "kerneldisplay.h"
#include "omaplfb.h"
#include "pvrmodule.h"

MODULE_SUPPORTED_DEVICE(DEVNAME);

/*
 * Kernel malloc
 * in: ui32ByteSize
 */
void *OMAPLFBAllocKernelMem(unsigned long ui32ByteSize)
{
	void *p;

	p = kmalloc(ui32ByteSize, GFP_KERNEL);
	if(!p)
		return 0;
	return p;
}

/*
 * Kernel free
 * in: pvMem
 */
void OMAPLFBFreeKernelMem(void *pvMem)
{
	kfree(pvMem);
}

/*
 * Here we get the function pointer to get jump table from
 * services using an external function.
 * in: szFunctionName
 * out: ppfnFuncTable
 */
OMAP_ERROR OMAPLFBGetLibFuncAddr (char *szFunctionName,
	PFN_DC_GET_PVRJTABLE *ppfnFuncTable)
{
	if(strcmp("PVRGetDisplayClassJTable", szFunctionName) != 0)
	{
		ERROR_PRINTK("Unable to get function pointer for %s"
			" from services", szFunctionName);
		return OMAP_ERROR_INVALID_PARAMS;
	}
	*ppfnFuncTable = PVRGetDisplayClassJTable;

	return OMAP_OK;
}


#if defined(FLIP_TECHNIQUE_FRAMEBUFFER)
/*
 * Presents the flip in the display with the framebuffer API
 * in: psSwapChain, aPhyAddr
 */
static void OMAPLFBFlipNoLock(OMAPLFB_SWAPCHAIN *psSwapChain,
	unsigned long aPhyAddr)
{
	OMAPLFB_DEVINFO	*psDevInfo = (OMAPLFB_DEVINFO *)psSwapChain->pvDevInfo;
	struct fb_info * framebuffer = psDevInfo->psLINFBInfo;

	/* Get the framebuffer physical address base */
	unsigned long fb_base_phyaddr =
		psDevInfo->sSystemBuffer.sSysAddr.uiAddr;

	/* Calculate the virtual Y to move in the framebuffer */
	framebuffer->var.yoffset =
		(aPhyAddr - fb_base_phyaddr) / framebuffer->fix.line_length;
	framebuffer->var.activate = FB_ACTIVATE_FORCE;
	fb_set_var(framebuffer, &framebuffer->var);
}

#elif defined(FLIP_TECHNIQUE_OVERLAY)
/*
 * Presents the flip in the display with the DSS2 overlay API
 * in: psSwapChain, aPhyAddr
 */
static void OMAPLFBFlipNoLock(OMAPLFB_SWAPCHAIN *psSwapChain,
	unsigned long aPhyAddr)
{
	OMAPLFB_DEVINFO	*psDevInfo = (OMAPLFB_DEVINFO *)psSwapChain->pvDevInfo;
	struct fb_info * framebuffer = psDevInfo->psLINFBInfo;
	struct omapfb_info *ofbi = FB2OFB(framebuffer);
	unsigned long fb_offset;
	int i;

	fb_offset = aPhyAddr - psDevInfo->sSystemBuffer.sSysAddr.uiAddr;

	for(i = 0; i < ofbi->num_overlays ; i++)
	{
		struct omap_dss_device *display = NULL;
		struct omap_overlay_manager *manager;
		struct omap_overlay *overlay;
		struct omap_overlay_info overlay_info;

		overlay = ofbi->overlays[i];
		manager = overlay->manager;
		overlay->get_overlay_info( overlay, &overlay_info );

		overlay_info.paddr = framebuffer->fix.smem_start + fb_offset;
		overlay_info.vaddr = framebuffer->screen_base + fb_offset;
		overlay->set_overlay_info(overlay, &overlay_info);

		if (manager) {
			display = manager->device;
			/* No display attached to this overlay, don't update */
			if (!display)
				continue;
			manager->apply(manager);
		}

		if (display && display->update &&
			display->get_update_mode(display) ==
			OMAP_DSS_UPDATE_MANUAL)
			display->update(display, 0, 0, overlay_info.width,
				overlay_info.height);

	}
}

#else
#error No flipping technique selected, please define \
	FLIP_TECHNIQUE_FRAMEBUFFER or FLIP_TECHNIQUE_OVERLAY
#endif

void OMAPLFBFlip(OMAPLFB_SWAPCHAIN *psSwapChain, unsigned long aPhyAddr)
{
	OMAPLFB_DEVINFO *psDevInfo = (OMAPLFB_DEVINFO *)psSwapChain->pvDevInfo;
	struct fb_info *framebuffer = psDevInfo->psLINFBInfo;
	struct omapfb_info *ofbi = FB2OFB(framebuffer);
	struct omapfb2_device *fbdev = ofbi->fbdev;

	omapfb_lock(fbdev);
	OMAPLFBFlipNoLock(psSwapChain, aPhyAddr);
	omapfb_unlock(fbdev);
}

/*
 * Synchronize with the display to prevent tearing
 * On DSI panels the display->sync function is used to handle FRAMEDONE IRQ
 * On DPI panels the display->wait_vsync is used to handle VSYNC IRQ
 * in: psDevInfo
 */
void OMAPLFBPresentSync(OMAPLFB_DEVINFO *psDevInfo,
	OMAPLFB_FLIP_ITEM *psFlipItem)
{
	struct fb_info * framebuffer = psDevInfo->psLINFBInfo;
	struct omapfb_info *ofbi = FB2OFB(framebuffer);
	struct omapfb2_device *fbdev = ofbi->fbdev;
	struct omap_dss_device *display;
	int err = 1;

	omapfb_lock(fbdev);

	display = fb2display(framebuffer);

	/* The framebuffer doesn't have a display attached, just bail out */
	if (!display) {
		omapfb_unlock(fbdev);
		return;
	}

	if (display->sync &&
		display->get_update_mode(display) == OMAP_DSS_UPDATE_MANUAL) {
		/* Wait first for the DSI bus to be released then update */
		err = display->sync(display);
		OMAPLFBFlipNoLock(psDevInfo->psSwapChain,
			(unsigned long)psFlipItem->sSysAddr->uiAddr);
	} else if (display->wait_vsync) {
		/*
		 * Update the video pipelines registers then wait until the
		 * frame is shown with a VSYNC
		 */
		OMAPLFBFlipNoLock(psDevInfo->psSwapChain,
			(unsigned long)psFlipItem->sSysAddr->uiAddr);
		err = display->wait_vsync(display);
	}

	if (err)
		WARNING_PRINTK("Unable to sync with display %u!",
			psDevInfo->uDeviceID);

	omapfb_unlock(fbdev);
}

#if defined(LDM_PLATFORM)

static volatile OMAP_BOOL bDeviceSuspended;

/*
 * Common suspend driver function
 * in: psSwapChain, aPhyAddr
 */
static void OMAPLFBCommonSuspend(void)
{
	if (bDeviceSuspended)
	{
		DEBUG_PRINTK("Driver is already suspended");
		return;
	}

	OMAPLFBDriverSuspend();
	bDeviceSuspended = OMAP_TRUE;
}

#if defined(SGX_EARLYSUSPEND)

static struct early_suspend omaplfb_early_suspend;

/*
 * Android specific, driver is requested to be suspended
 * in: ea_event
 */
static void OMAPLFBDriverSuspend_Entry(struct early_suspend *ea_event)
{
	DEBUG_PRINTK("Requested driver suspend");
	OMAPLFBCommonSuspend();
}

/*
 * Android specific, driver is requested to be suspended
 * in: ea_event
 */
static void OMAPLFBDriverResume_Entry(struct early_suspend *ea_event)
{
	DEBUG_PRINTK("Requested driver resume");
	OMAPLFBDriverResume();
	bDeviceSuspended = OMAP_FALSE;
}

static struct platform_driver omaplfb_driver = {
	.driver = {
		.name = DRVNAME,
	}
};

#else /* defined(SGX_EARLYSUSPEND) */

/*
 * Function called when the driver is requested to be suspended
 * in: pDevice, state
 */
static int OMAPLFBDriverSuspend_Entry(struct platform_device unref__ *pDevice,
	pm_message_t unref__ state)
{
	DEBUG_PRINTK("Requested driver suspend");
	OMAPLFBCommonSuspend();
	return 0;
}

/*
 * Function called when the driver is requested to resume
 * in: pDevice
 */
static int OMAPLFBDriverResume_Entry(struct platform_device unref__ *pDevice)
{
	DEBUG_PRINTK("Requested driver resume");
	OMAPLFBDriverResume();
	bDeviceSuspended = OMAP_FALSE;
	return 0;
}

/*
 * Function called when the driver is requested to shutdown
 * in: pDevice
 */
static IMG_VOID OMAPLFBDriverShutdown_Entry(
	struct platform_device unref__ *pDevice)
{
	DEBUG_PRINTK("Requested driver shutdown");
	OMAPLFBCommonSuspend();
}

static struct platform_driver omaplfb_driver = {
	.driver = {
		.name = DRVNAME,
	},
	.suspend = OMAPLFBDriverSuspend_Entry,
	.resume	= OMAPLFBDriverResume_Entry,
	.shutdown = OMAPLFBDriverShutdown_Entry,
};

#endif /* defined(SGX_EARLYSUSPEND) */

#endif /* defined(LDM_PLATFORM) */

/*
 * Driver init function
 */
static int __init OMAPLFB_Init(void)
{
	if(OMAPLFBInit() != OMAP_OK)
	{
		WARNING_PRINTK("Driver init failed");
		return -ENODEV;
	}

#if defined(LDM_PLATFORM)
	DEBUG_PRINTK("Registering platform driver");
	if (platform_driver_register(&omaplfb_driver))
	{
		WARNING_PRINTK("Unable to register platform driver");
		if(OMAPLFBDeinit() != OMAP_OK)
			WARNING_PRINTK("Driver cleanup failed\n");
		return -ENODEV;
	}

#if defined(SGX_EARLYSUSPEND)
	omaplfb_early_suspend.suspend = OMAPLFBDriverSuspend_Entry;
        omaplfb_early_suspend.resume = OMAPLFBDriverResume_Entry;
        omaplfb_early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
        register_early_suspend(&omaplfb_early_suspend);
	DEBUG_PRINTK("Registered early suspend support");
#endif

#endif
	return 0;
}

/*
 * Driver exit function
 */
static IMG_VOID __exit OMAPLFB_Cleanup(IMG_VOID)
{    
#if defined(LDM_PLATFORM)
	DEBUG_PRINTK("Removing platform driver");
	platform_driver_unregister(&omaplfb_driver);
#if defined(SGX_EARLYSUSPEND)
        unregister_early_suspend(&omaplfb_early_suspend);
#endif
#endif
	if(OMAPLFBDeinit() != OMAP_OK)
		WARNING_PRINTK("Driver cleanup failed");
}

late_initcall(OMAPLFB_Init);
module_exit(OMAPLFB_Cleanup);

