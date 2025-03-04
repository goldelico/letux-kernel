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

#if !defined(__PVR_DRM_H__)
#define __PVR_DRM_H__

#include <drm/drm_device.h>
#include <linux/platform_device.h>

#if defined (PDUMP)
#include "linuxsrv.h"
#endif

#include "pvr_drm_shared.h"

#if defined(SUPPORT_DRI_DRM)

#if defined(PVR_DISPLAY_CONTROLLER_DRM_IOCTL)
#include "3rdparty_dc_drm_shared.h"
#endif

#define	PVR_DRM_MAKENAME_HELPER(x, y) x ## y
#define	PVR_DRM_MAKENAME(x, y) PVR_DRM_MAKENAME_HELPER(x, y)

#if defined(PVR_DRI_DRM_PLATFORM_DEV)
#define	LDM_DEV	struct platform_device
#endif

int PVRCore_Init(void);
void PVRCore_Cleanup(void);
int PVRSRVOpen(struct drm_device *dev, struct drm_file *pFile);
void PVRSRVRelease(void *pvPrivData);

#if !defined(SUPPORT_DRI_DRM_PLUGIN)
#if defined(PVR_DRI_DRM_PLATFORM_DEV)
void PVRSRVDriverShutdown(LDM_DEV *pDevice);
int PVRSRVDriverSuspend(LDM_DEV *pDevice, pm_message_t state);
int PVRSRVDriverResume(LDM_DEV *pDevice);
#else
#if defined(SUPPORT_DRM_MODESET)
int PVRSRVDriverSuspend(struct pci_dev *pDevice, pm_message_t state);
int PVRSRVDriverResume(struct pci_dev *pDevice);
#else
int PVRSRVDriverSuspend(struct drm_device *pDevice, pm_message_t state);
int PVRSRVDriverResume(struct drm_device *pDevice);
#endif /* defined(SUPPORT_DRM_MODESET) */
#endif /* defined(PVR_DRI_DRM_PLATFORM_DEV) */
#endif /* !defined(SUPPORT_DRI_DRM_PLUGIN) */

int PVRSRV_BridgeDispatchKM(struct drm_device *dev, void *arg, struct drm_file *pFile);
int PVRSRV_BridgeCompatDispatchKM(struct drm_device *dev, void *arg, struct drm_file *pFile);

#if defined(SUPPORT_DRI_DRM_EXT)
#define	DRI_DRM_STATIC
/*Exported functions to common drm layer*/
int PVRSRVDrmLoad(struct drm_device *dev, unsigned long flags);
int PVRSRVDrmUnload(struct drm_device *dev);
int PVRSRVDrmOpen(struct drm_device *dev, struct drm_file *file);
#if defined(PVR_LINUX_USING_WORKQUEUES)
DRI_DRM_STATIC int PVRSRVDrmRelease(struct inode *inode, struct file *filp);
#else
void PVRSRVDrmPostClose(struct drm_device *dev, struct drm_file *file);
#endif
int PVRDRMIsMaster(struct drm_device *dev, IMG_VOID *arg, struct drm_file *pFile);
int PVRDRMUnprivCmd(struct drm_device *dev, IMG_VOID *arg, struct drm_file *pFile);
int PVRDRM_Dummy_ioctl(struct drm_device *dev, IMG_VOID *arg, struct drm_file *pFile);
#else
#define	DRI_DRM_STATIC	static
#endif	/* defined(SUPPORT_DRI_DRM_EXT) */

#if defined(DISPLAY_CONTROLLER)
extern int PVR_DRM_MAKENAME(DISPLAY_CONTROLLER, _Init)(struct drm_device *);
extern void PVR_DRM_MAKENAME(DISPLAY_CONTROLLER, _Cleanup)(struct drm_device *);
extern int PVR_DRM_MAKENAME(DISPLAY_CONTROLLER, _Suspend)(struct drm_device *);
extern int PVR_DRM_MAKENAME(DISPLAY_CONTROLLER, _Resume)(struct drm_device *);
#if defined(PVR_DISPLAY_CONTROLLER_DRM_IOCTL)
extern int PVR_DRM_MAKENAME(DISPLAY_CONTROLLER, _Ioctl)(struct drm_device *dev, void *arg, struct drm_file *pFile);
#endif
#endif

#if defined(PDUMP)
int dbgdrv_init(void);
void dbgdrv_cleanup(void);
IMG_INT dbgdrv_ioctl(struct drm_device *dev, IMG_VOID *arg, struct drm_file *pFile);
#endif

#if !defined(SUPPORT_DRI_DRM_EXT)
/*
 * We need the command number names to begin with "DRM_" for newer versions
 * of the macro used to fill out the DRM ioctl table.  Similarly, the
 * ioctl number names must begin with "DRM_IOCTL_". The suffixes for the
 * two sets of strings must match (e.g. end with "PVR_SRVKM" in both
 * cases).
 */
#define	DRM_PVR_SRVKM		PVR_DRM_SRVKM_CMD
#define	DRM_PVR_IS_MASTER	PVR_DRM_IS_MASTER_CMD
#define	DRM_PVR_UNPRIV		PVR_DRM_UNPRIV_CMD
#define	DRM_PVR_DBGDRV		PVR_DRM_DBGDRV_CMD
#define	DRM_PVR_DISP		PVR_DRM_DISP_CMD

/*
 * Some versions of the kernel will dereference a NULL pointer if data is
 * is passed to an ioctl that doesn't expect any, so avoid using the _IO
 * macro, and use _IOW instead, specifying a dummy argument.
*/
typedef struct {
	char dummy[4];
} drm_pvr_dummy_arg;

/* IOCTL numbers */
#define	DRM_IOCTL_PVR_SRVKM	DRM_IOWR(DRM_COMMAND_BASE + DRM_PVR_SRVKM, PVRSRV_BRIDGE_PACKAGE)
#define	DRM_IOCTL_PVR_IS_MASTER DRM_IOW(DRM_COMMAND_BASE + DRM_PVR_IS_MASTER, drm_pvr_dummy_arg)
#define	DRM_IOCTL_PVR_UNPRIV	DRM_IOWR(DRM_COMMAND_BASE + DRM_PVR_UNPRIV, drm_pvr_unpriv_cmd)

#if defined(PDUMP)
#define	DRM_IOCTL_PVR_DBGDRV	DRM_IOWR(DRM_COMMAND_BASE + DRM_PVR_DBGDRV, IOCTL_PACKAGE)
#endif

#if defined(PVR_DISPLAY_CONTROLLER_DRM_IOCTL)
#define	DRM_IOCTL_PVR_DISP	DRM_IOWR(DRM_COMMAND_BASE + DRM_PVR_DISP, drm_pvr_display_cmd)
#endif
#endif	/* !defined(SUPPORT_DRI_DRM_EXT) */

#if defined(SUPPORT_DRI_DRM_PLUGIN)
typedef	struct {
	char *name;

	int (*load)(struct drm_device *dev, unsigned long flags);
	int (*unload)(struct drm_device *dev);

	int (*open)(struct drm_device *dev, struct drm_file *file);
	int (*release)(struct drm_device *dev, struct drm_file *file);

	int (*mmap)(struct file* pFile, struct vm_area_struct* ps_vma);

	struct drm_ioctl_desc *ioctls;
	int num_ioctls;
	int ioctl_start;
} PVRSRV_DRM_PLUGIN;

int SysDRMRegisterPlugin(PVRSRV_DRM_PLUGIN *psDRMPlugin);
void SysDRMUnregisterPlugin(PVRSRV_DRM_PLUGIN *psDRMPlugin);
#endif	/* defined(SUPPORT_DRI_DRM_PLUGIN) */

#endif	/* defined(SUPPORT_DRI_DRM) */

#endif /* defined(__PVR_DRM_H__) */
