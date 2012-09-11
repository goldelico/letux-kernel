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

#include <linux/pci.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/interrupt.h>

#if defined(LDM_PLATFORM)
#include <linux/platform_device.h>
#endif 

#if defined (SUPPORT_TI_DSS_FW)
#include <asm/io.h>

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,26))
#include <plat/display.h>
#else 
#include <asm/arch-omap/display.h>
#endif 
#else
#if !defined (CONFIG_OMAP2_DSS)
#define DISPC_IRQ_VSYNC 0x0002
extern int omap_dispc_request_irq(unsigned long, void (*)(void *), void *);
extern void omap_dispc_free_irq(unsigned long, void (*)(void *), void *);
extern void omap_dispc_set_plane_base(int plane, IMG_UINT32 phys_addr);
#else
#include <plat/display.h>
#include <linux/console.h>
#include <linux/fb.h>
static omap_dispc_isr_t *pOMAPLFBVSyncISRHandle = NULL;
#endif
#endif



#include "img_defs.h"
#include "servicesext.h"
#include "kerneldisplay.h"
#include "omaplfb.h"
#include "pvrmodule.h"

MODULE_SUPPORTED_DEVICE(DEVNAME);

#define unref__ __attribute__ ((unused))

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
#if !defined (SUPPORT_TI_DSS_FW) 
IMG_VOID OMAPLFBEnableVSyncInterrupt(OMAPLFB_SWAPCHAIN *psSwapChain)
{
	if (pOMAPLFBVSyncISRHandle == NULL)
		OMAPLFBInstallVSyncISR (psSwapChain);
}

IMG_VOID OMAPLFBDisableVSyncInterrupt(OMAPLFB_SWAPCHAIN *psSwapChain)
{
	if (pOMAPLFBVSyncISRHandle != NULL)
		OMAPLFBUninstallVSyncISR (psSwapChain);
}
#else
static void OMAPLFBVSyncWriteReg(OMAPLFB_SWAPCHAIN *psSwapChain, unsigned long ulOffset, unsigned long ulValue)
{
	void *pvRegAddr = (void *)((char *)psSwapChain->pvRegs + ulOffset);

	
	writel(ulValue, pvRegAddr);
}

static unsigned long OMAPLFBVSyncReadReg(OMAPLFB_SWAPCHAIN *psSwapChain, unsigned long ulOffset)
{
	return readl((char *)psSwapChain->pvRegs + ulOffset);
}

void OMAPLFBEnableVSyncInterrupt(OMAPLFB_SWAPCHAIN *psSwapChain)
{
#if defined(SYS_USING_INTERRUPTS)
	
	unsigned long ulInterruptEnable  = OMAPLFBVSyncReadReg(psSwapChain, OMAPLCD_IRQENABLE);
	ulInterruptEnable |= OMAPLCD_INTMASK_VSYNC;
	OMAPLFBVSyncWriteReg(psSwapChain, OMAPLCD_IRQENABLE, ulInterruptEnable );
#endif
}

void OMAPLFBDisableVSyncInterrupt(OMAPLFB_SWAPCHAIN *psSwapChain)
{
#if defined(SYS_USING_INTERRUPTS)
	
	unsigned long ulInterruptEnable = OMAPLFBVSyncReadReg(psSwapChain, OMAPLCD_IRQENABLE);
	ulInterruptEnable &= ~(OMAPLCD_INTMASK_VSYNC);
	OMAPLFBVSyncWriteReg(psSwapChain, OMAPLCD_IRQENABLE, ulInterruptEnable);
#endif
}
#endif
#if defined(SYS_USING_INTERRUPTS)
static void
#if defined (SUPPORT_TI_DSS_FW)
OMAPLFBVSyncISR(void *arg, struct pt_regs unref__ *regs)
#else
#if defined (CONFIG_OMAP2_DSS)
OMAPLFBVSyncISR(void *arg, u32 mask)
#else
OMAPLFBVSyncISR(void *arg)
#endif
#endif
{
	OMAPLFB_SWAPCHAIN *psSwapChain= (OMAPLFB_SWAPCHAIN *)arg;
	
	(void) OMAPLFBVSyncIHandler(psSwapChain);
}
#endif
#if !defined (SUPPORT_TI_DSS_FW)
OMAP_ERROR OMAPLFBInstallVSyncISR(OMAPLFB_SWAPCHAIN *psSwapChain)
{
#if !defined (CONFIG_OMAP2_DSS)
	if (omap_dispc_request_irq(DISPC_IRQ_VSYNC, OMAPLFBVSyncISR, psSwapChain) != 0)
#else
	 pOMAPLFBVSyncISRHandle = omap_dispc_register_isr(
			(omap_dispc_isr_t)OMAPLFBVSyncISR, psSwapChain, DISPC_IRQ_VSYNC);

	if (pOMAPLFBVSyncISRHandle != NULL)
#endif
		return PVRSRV_ERROR_OUT_OF_MEMORY; /* not worth a proper mapping */
	return OMAP_OK;
}


OMAP_ERROR OMAPLFBUninstallVSyncISR (OMAPLFB_SWAPCHAIN *psSwapChain)
{
#if !defined (CONFIG_OMAP2_DSS)
	omap_dispc_free_irq(DISPC_IRQ_VSYNC, OMAPLFBVSyncISR, psSwapChain);
#else
	omap_dispc_unregister_isr (OMAPLFBVSyncISR, psSwapChain, DISPC_IRQ_VSYNC);
#endif
	return OMAP_OK;		
} 


IMG_VOID OMAPLFBFlip(OMAPLFB_SWAPCHAIN *psSwapChain,
						  IMG_UINT32 aPhyAddr)
{
#if !defined (CONFIG_OMAP2_DSS)
	omap_dispc_set_plane_base(0, aPhyAddr);
#else
	OMAPLFBFlipDSS2 (psSwapChain, aPhyAddr);
#endif
}
#else

OMAP_ERROR OMAPLFBInstallVSyncISR(OMAPLFB_SWAPCHAIN *psSwapChain)
{
#if defined(SYS_USING_INTERRUPTS)
	OMAPLFBDisableVSyncInterrupt(psSwapChain);

	if (omap2_disp_register_isr(OMAPLFBVSyncISR, psSwapChain,
				    DISPC_IRQSTATUS_VSYNC))
	{
		printk(KERN_INFO DRIVER_PREFIX ": OMAPLFBInstallVSyncISR: Request OMAPLCD IRQ failed\n");
		return (OMAP_ERROR_INIT_FAILURE);
	}

#endif
	return (OMAP_OK);
}


OMAP_ERROR OMAPLFBUninstallVSyncISR (OMAPLFB_SWAPCHAIN *psSwapChain)
{
#if defined(SYS_USING_INTERRUPTS)
	OMAPLFBDisableVSyncInterrupt(psSwapChain);

	omap2_disp_unregister_isr(OMAPLFBVSyncISR);

#endif
	return (OMAP_OK);
}

void OMAPLFBEnableDisplayRegisterAccess(void)
{
	omap2_disp_get_dss();
}

void OMAPLFBDisableDisplayRegisterAccess(void)
{
	omap2_disp_put_dss();
}

void OMAPLFBFlip(OMAPLFB_SWAPCHAIN *psSwapChain, unsigned long aPhyAddr)
{
	unsigned long control;

	
	OMAPLFBVSyncWriteReg(psSwapChain, OMAPLCD_GFX_BA0, aPhyAddr);
	OMAPLFBVSyncWriteReg(psSwapChain, OMAPLCD_GFX_BA1, aPhyAddr);

	control = OMAPLFBVSyncReadReg(psSwapChain, OMAPLCD_CONTROL);
	control |= OMAP_CONTROL_GOLCD;
	OMAPLFBVSyncWriteReg(psSwapChain, OMAPLCD_CONTROL, control);
}
#endif

#if defined(LDM_PLATFORM)

static OMAP_BOOL bDeviceSuspended;

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

static void OMAPLFBDeviceRelease_Entry(struct device unref__ *pDevice)
{
	DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBDriverRelease_Entry\n"));

	OMAPLFBCommonSuspend();
}

static struct platform_driver omaplfb_driver = {
	.driver = {
		.name		= DRVNAME,
	},
	.suspend	= OMAPLFBDriverSuspend_Entry,
	.resume		= OMAPLFBDriverResume_Entry,
	.shutdown	= OMAPLFBDriverShutdown_Entry,
};

static struct platform_device omaplfb_device = {
	.name			= DEVNAME,
	.id				= -1,
	.dev 			= {
		.release		= OMAPLFBDeviceRelease_Entry
	}
};
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

#if defined(LDM_PLATFORM)
	if ((error = platform_driver_register(&omaplfb_driver)) != 0)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": OMAPLFB_Init: Unable to register platform driver (%d)\n", error);

		goto ExitDeinit;
	}

	if ((error = platform_device_register(&omaplfb_device)) != 0)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": OMAPLFB_Init:  Unable to register platform device (%d)\n", error);

		goto ExitDriverUnregister;
	}
#endif 

	return 0;

#if defined(LDM_PLATFORM)
ExitDriverUnregister:
	platform_driver_unregister(&omaplfb_driver);

ExitDeinit:
	if(OMAPLFBDeinit() != OMAP_OK)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": OMAPLFB_Init: OMAPLFBDeinit failed\n");
	}

	return -ENODEV;
#endif 
}

static IMG_VOID __exit OMAPLFB_Cleanup(IMG_VOID)
{    
#if defined (LDM_PLATFORM)
	platform_device_unregister(&omaplfb_device);
	platform_driver_unregister(&omaplfb_driver);
#endif

	if(OMAPLFBDeinit() != OMAP_OK)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": OMAPLFB_Cleanup: OMAPLFBDeinit failed\n");
	}
}

module_init(OMAPLFB_Init);
module_exit(OMAPLFB_Cleanup);

