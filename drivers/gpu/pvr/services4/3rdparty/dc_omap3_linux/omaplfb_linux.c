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
#include <linux/omapfb.h>

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29))
#include <plat/vrfb.h>
#else
#include <mach/vrfb.h>
#endif
#include <../drivers/video/omap2/omapfb/omapfb.h>

#if defined(LDM_PLATFORM)
#include <linux/platform_device.h>
#endif 

#include <asm/io.h>


#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29))
#include <plat/display.h>
#elif (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,26))
#include <mach/display.h>
#else 
#include <asm/arch-omap/display.h>
#endif 

#include "img_defs.h"
#include "servicesext.h"
#include "kerneldisplay.h"
#include "omaplfb.h"
#include "pvrmodule.h"

MODULE_SUPPORTED_DEVICE(DEVNAME);

#define unref__ __attribute__ ((unused))

static struct omap_overlay_manager* lcd_mgr = 0;

void *OMAPLFBAllocKernelMem(unsigned long ulSize)
{
	return kmalloc(ulSize, GFP_KERNEL);
}

void OMAPLFBFreeKernelMem(void *pvMem)
{
	kfree(pvMem);
}


OMAP_ERROR OMAPLFBGetLibFuncAddr (char *szFunctionName, PFN_DC_GET_PVRJTABLE *ppfnFuncTable)
{
	if(strcmp("PVRGetDisplayClassJTable", szFunctionName) != 0)
	{
		return (OMAP_ERROR_INVALID_PARAMS);
	}

	
	*ppfnFuncTable = PVRGetDisplayClassJTable;

	return (OMAP_OK);
}

static void GetLcdManager(void){
	struct omap_overlay *ovl;
	ovl = omap_dss_get_overlay(0);
	lcd_mgr = omap_dss_get_overlay_manager(ovl->manager->id);
    if(!lcd_mgr)
    {
    	DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": GetLcdManager couldn't find lcd overlay manager\n"));
    }

}


void OMAPLFBFlip(OMAPLFB_SWAPCHAIN *psSwapChain, unsigned long aPhyAddr)
{
struct omap_overlay* overlay;
	struct omap_overlay_info overlay_info;
	struct fb_info * framebuffer;
	OMAPLFB_DEVINFO	*psDevInfo;
	struct omapfb_info *ofbi;
	struct omapfb2_device *fbdev;
	int i;
	unsigned long fb_offset;

	psDevInfo = (OMAPLFB_DEVINFO *) psSwapChain->pvDevInfo;
	framebuffer = psDevInfo->psLINFBInfo;
	ofbi = FB2OFB(framebuffer);
	fb_offset = aPhyAddr - psDevInfo->sSystemBuffer.sSysAddr.uiAddr;
	fbdev = ofbi->fbdev;

	omapfb_lock(fbdev);

	for(i = 0; i < ofbi->num_overlays ; i++)
	{
		overlay = ofbi->overlays[i];
		overlay->get_overlay_info( overlay, &overlay_info );
		/* If the overlay is not enabled don't update it */
		if(!overlay_info.enabled)
			continue;

		overlay_info.paddr = framebuffer->fix.smem_start + fb_offset;
		overlay_info.vaddr = framebuffer->screen_base + fb_offset;
		overlay->set_overlay_info(overlay, &overlay_info);
		overlay->manager->apply(overlay->manager);

		if(overlay->manager->device->update)
		{
			overlay->manager->device->update(
				overlay->manager->device, 0, 0,
				overlay_info.width,
				overlay_info.height);
		}
	}

	omapfb_unlock(fbdev);

}

void OMAPLFBWaitForVSync(void)
{
	if (lcd_mgr && lcd_mgr->device)	
		lcd_mgr->device->wait_vsync(lcd_mgr->device);
}


#if defined(LDM_PLATFORM)

static volatile OMAP_BOOL bDeviceSuspended;

#if !defined(SGX_EARLYSUSPEND)

static void OMAPLFBCommonSuspend(void)
{
	if (bDeviceSuspended)
	{
		return;
	}

	OMAPLFBDriverSuspend();

	bDeviceSuspended = OMAP_TRUE;
}

static int OMAPLFBDriverSuspend_Entry(struct platform_device unref__ *pDevice, pm_message_t unref__ state)
{
	DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBDriverSuspend_Entry\n"));

	OMAPLFBCommonSuspend();

	return 0;
}

static int OMAPLFBDriverResume_Entry(struct platform_device unref__ *pDevice)
{
	DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBDriverResume_Entry\n"));

	OMAPLFBDriverResume();

	bDeviceSuspended = OMAP_FALSE;

	return 0;
}

static IMG_VOID OMAPLFBDriverShutdown_Entry(struct platform_device unref__ *pDevice)
{
	DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBDriverShutdown_Entry\n"));

	OMAPLFBCommonSuspend();
}

#else /* !defined(SGX_EARLYSUSPEND) */
static void OMAPLFBCommonSuspend(void)
{

	if (bDeviceSuspended)
	{
		return;
	}

	OMAPLFBDriverSuspend();

	bDeviceSuspended = OMAP_TRUE;

}

static void OMAPLFBDriverSuspend_Entry(struct early_suspend *h)
{
	DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBDriverSuspend_Entry\n"));

	printk(KERN_WARNING DRIVER_PREFIX ": **** SUSPEND\n");
	
	OMAPLFBCommonSuspend();

}

static void OMAPLFBDriverResume_Entry(struct early_suspend *h)
{
	DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBDriverResume_Entry\n"));

	printk(KERN_WARNING DRIVER_PREFIX ": **** RESUME\n");	

	OMAPLFBDriverResume();	

	bDeviceSuspended = OMAP_FALSE;

}
#endif /* !defined(SGX_EARLYSUSPEND) */

static struct platform_driver omaplfb_driver = {
	.driver = {
		.name		= DRVNAME,
	},
#if !defined(SGX_EARLYSUSPEND)
	.suspend	= OMAPLFBDriverSuspend_Entry,
	.resume		= OMAPLFBDriverResume_Entry,
	.shutdown	= OMAPLFBDriverShutdown_Entry,
#endif
};

#if defined(MODULE)

static void OMAPLFBDeviceRelease_Entry(struct device unref__ *pDevice)
{
	DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBDriverRelease_Entry\n"));

	OMAPLFBCommonSuspend();
}

static struct platform_device omaplfb_device = {
	.name			= DEVNAME,
	.id				= -1,
	.dev 			= {
		.release		= OMAPLFBDeviceRelease_Entry
	}
};

#endif /* defined(MODULE) */  

#endif	/* defined(LDM_PLATFORM) */

#if defined(SGX_EARLYSUSPEND)
	OMAPLFB_DEVINFO *psDevInfo;
#endif	

static int __init OMAPLFB_Init(void)
{
#if defined(LDM_PLATFORM)
	int error;
#endif

	if(OMAPLFBInit() != OMAP_OK)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": OMAPLFB_Init: OMAPLFBInit failed\n");
		return -ENODEV;
	}

	/* Get the LCD manager which corresponds to the primary display*/
	GetLcdManager();

#if defined(LDM_PLATFORM)
	if ((error = platform_driver_register(&omaplfb_driver)) != 0)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": OMAPLFB_Init: Unable to register platform driver (%d)\n", error);

		goto ExitDeinit;
	}

#if defined(MODULE)
	if ((error = platform_device_register(&omaplfb_device)) != 0)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": OMAPLFB_Init: Unable to register platform device (%d)\n", error);

		goto ExitDeinit;
	}
#endif /* defined(MODULE) */

#if defined(SGX_EARLYSUSPEND)
	psDevInfo = NULL;
	psDevInfo = GetAnchorPtr();

	if (psDevInfo == NULL)
	{
		DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFB_Init: Unable to get DevInfo anchor pointer\n"));
		goto ExitDeinit;
	}

	psDevInfo->sFBInfo.early_suspend.suspend = OMAPLFBDriverSuspend_Entry;
        psDevInfo->sFBInfo.early_suspend.resume = OMAPLFBDriverResume_Entry;
        psDevInfo->sFBInfo.early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
        register_early_suspend(&psDevInfo->sFBInfo.early_suspend);
#endif /* defined(SGX_EARLYSUSPEND) */

#endif /* defined(LDM_PLATFORM) */

	return 0;

#if defined(LDM_PLATFORM)
ExitDeinit:
	platform_driver_unregister(&omaplfb_driver);

#if defined(SGX_EARLYSUSPEND)
        unregister_early_suspend(&psDevInfo->sFBInfo.early_suspend);
#endif
	if(OMAPLFBDeinit() != OMAP_OK)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": OMAPLFB_Init: OMAPLFBDeinit failed\n");
	}

	return -ENODEV;
#endif /* defined(LDM_PLATFORM) */ 
}

static IMG_VOID __exit OMAPLFB_Cleanup(IMG_VOID)
{    
#if defined (LDM_PLATFORM)
#if defined (MODULE)
	platform_device_unregister(&omaplfb_device);
#endif
	platform_driver_unregister(&omaplfb_driver);
#endif /* defined (LDM_PLATFORM) */

#if defined(SGX_EARLYSUSPEND)
	unregister_early_suspend(&psDevInfo->sFBInfo.early_suspend);
#endif

	if(OMAPLFBDeinit() != OMAP_OK)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": OMAPLFB_Cleanup: OMAPLFBDeinit failed\n");
	}
}

module_init(OMAPLFB_Init);
module_exit(OMAPLFB_Cleanup);

