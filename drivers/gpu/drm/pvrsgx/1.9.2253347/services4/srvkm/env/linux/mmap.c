/*************************************************************************/ /*!
@Title          Linux mmap interface
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
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

#include <linux/version.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38))
#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#endif

#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pfn_t.h>
#include <linux/vmalloc.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
#include <linux/wrapper.h>
#endif
#include <linux/slab.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26))
#include <linux/highmem.h>
#endif
#include <asm/io.h>
#include <asm/page.h>
#include <asm/shmparam.h>
#include <asm/pgtable.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22))
#include <linux/sched.h>
#include <asm/current.h>
#endif
#if defined(SUPPORT_DRI_DRM)
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,5,0))
#include <drm/drmP.h>
#else
#include <drm/drm_file.h>
#endif
#endif

#include "services_headers.h"

#include "pvrmmap.h"
#include "mutils.h"
#include "mmap.h"
#include "mm.h"
#include "proc.h"
#include "mutex.h"
#include "handle.h"
#include "perproc.h"
#include "env_perproc.h"
#include "bridged_support.h"
#if defined(SUPPORT_DRI_DRM)
#include "pvr_drm.h"
#endif

#if !defined(PVR_SECURE_HANDLES) && !defined (SUPPORT_SID_INTERFACE)
#error "The mmap code requires PVR_SECURE_HANDLES"
#endif

/* WARNING:
 * The mmap code has its own mutex, to prevent a possible deadlock,
 * when using gPVRSRVLock.
 * The Linux kernel takes the mm->mmap_sem before calling the mmap
 * entry points (PVRMMap, MMapVOpen, MMapVClose), but the ioctl
 * entry point may take mm->mmap_sem during fault handling, or 
 * before calling get_user_pages.  If gPVRSRVLock was used in the
 * mmap entry points, a deadlock could result, due to the ioctl
 * and mmap code taking the two locks in different orders.
 * As a corollary to this, the mmap entry points must not call
 * any driver code that relies on gPVRSRVLock is held.
 */
PVRSRV_LINUX_MUTEX g_sMMapMutex;

static LinuxKMemCache *g_psMemmapCache = NULL;
static LIST_HEAD(g_sMMapAreaList);
static LIST_HEAD(g_sMMapOffsetStructList);
#if defined(DEBUG_LINUX_MMAP_AREAS)
static IMG_UINT32 g_ui32RegisteredAreas = 0;
static IMG_UINT32 g_ui32TotalByteSize = 0;
#endif

static inline PKV_OFFSET_STRUCT FindOffsetStructByPID(LinuxMemArea *psLinuxMemArea, IMG_UINT32 ui32PID);

#if defined(DEBUG_LINUX_MMAP_AREAS)
static struct proc_dir_entry *g_ProcMMap;
#endif /* defined(DEBUG_LINUX_MMAP_AREAS) */

#if !defined(PVR_MAKE_ALL_PFNS_SPECIAL)
/*
 * Now that we are using mmap2 in srvclient, almost (*) the full 32
 * bit offset is available.  The range of values is divided into two.
 * The first part of the range, from FIRST_PHYSICAL_PFN to
 * LAST_PHYSICAL_PFN, is for raw page mappings (VM_PFNMAP).  The
 * resulting 43 bit (*) physical address range should be enough for
 * the current range of processors we support.
 *
 * NB: (*) -- the above figures assume 4KB page size.  The offset
 * argument to mmap2() is in units of 4,096 bytes regardless of page
 * size.  Thus, we lose (PAGE_SHIFT-12) bits of resolution on other
 * architectures.
 *
 * The second part of the range, from FIRST_SPECIAL_PFN to LAST_SPECIAL_PFN,
 * is used for all other mappings.  These other mappings will always
 * consist of pages with associated page structures, and need not
 * represent a contiguous range of physical addresses.
 *
 */
#define MMAP2_PGOFF_RESOLUTION (32-PAGE_SHIFT+12)
#define RESERVED_PGOFF_BITS 1
#define	MAX_MMAP_HANDLE		((1UL<<(MMAP2_PGOFF_RESOLUTION-RESERVED_PGOFF_BITS))-1)

#define	FIRST_PHYSICAL_PFN	0
#define	LAST_PHYSICAL_PFN	(FIRST_PHYSICAL_PFN + MAX_MMAP_HANDLE)
#define	FIRST_SPECIAL_PFN	(LAST_PHYSICAL_PFN + 1)
#define	LAST_SPECIAL_PFN	(FIRST_SPECIAL_PFN + MAX_MMAP_HANDLE)

#else	/* !defined(PVR_MAKE_ALL_PFNS_SPECIAL) */

#if PAGE_SHIFT != 12
#error This build variant has not yet been made non-4KB page-size aware
#endif

/*
 * Since we no longer have to worry about clashes with the mmap
 * offsets used for pure PFN mappings (VM_PFNMAP), there is greater
 * freedom in choosing the mmap handles.  This is useful if the
 * mmap offset space has to be shared with another driver component.
 */

#if defined(PVR_MMAP_OFFSET_BASE)
#define	FIRST_SPECIAL_PFN 	PVR_MMAP_OFFSET_BASE
#else
#define	FIRST_SPECIAL_PFN	0x80000000UL
#endif

#if defined(PVR_NUM_MMAP_HANDLES)
#define	MAX_MMAP_HANDLE		PVR_NUM_MMAP_HANDLES
#else
#define	MAX_MMAP_HANDLE		0x7fffffffUL
#endif

#endif	/* !defined(PVR_MAKE_ALL_PFNS_SPECIAL) */

#if !defined(PVR_MAKE_ALL_PFNS_SPECIAL)
static inline IMG_BOOL
PFNIsPhysical(IMG_UINT32 pfn)
{
	/* Unsigned, no need to compare >=0 */
	return (/*(pfn >= FIRST_PHYSICAL_PFN) &&*/ (pfn <= LAST_PHYSICAL_PFN)) ? IMG_TRUE : IMG_FALSE;
}

static inline IMG_BOOL
PFNIsSpecial(IMG_UINT32 pfn)
{
	/* Unsigned, no need to compare <=MAX_UINT */
	return ((pfn >= FIRST_SPECIAL_PFN) /*&& (pfn <= LAST_SPECIAL_PFN)*/) ? IMG_TRUE : IMG_FALSE;
}
#endif

#if !defined(PVR_MAKE_ALL_PFNS_SPECIAL)
static inline IMG_HANDLE
MMapOffsetToHandle(IMG_UINT32 pfn)
{
	if (PFNIsPhysical(pfn))
	{
		PVR_ASSERT(PFNIsPhysical(pfn));
		return IMG_NULL;
	}
	return (IMG_HANDLE)(pfn - FIRST_SPECIAL_PFN);
}
#endif

static inline IMG_UINT32
#if defined (SUPPORT_SID_INTERFACE)
HandleToMMapOffset(IMG_SID hHandle)
#else
HandleToMMapOffset(IMG_HANDLE hHandle)
#endif
{
	IMG_UINT32 ulHandle = (IMG_UINT32)hHandle;

#if !defined(PVR_MAKE_ALL_PFNS_SPECIAL)
	if (PFNIsSpecial(ulHandle))
	{
		PVR_ASSERT(PFNIsSpecial(ulHandle));
		return 0;
	}
#endif
	return ulHandle + FIRST_SPECIAL_PFN;
}

#if !defined(PVR_MAKE_ALL_PFNS_SPECIAL)
/*
 * Determine whether physical or special mappings will be used for
 * a given memory area.  At present, this decision is made on
 * whether the mapping represents a contiguous range of physical
 * addresses, which is a requirement for raw page mappings (VM_PFNMAP).
 * In the VMA structure for such a mapping, vm_pgoff is the PFN
 * (page frame number, the physical address divided by the page size)
 * of the first page in the VMA.  The second page is assumed to have
 * PFN (vm_pgoff + 1), the third (vm_pgoff + 2) and so on.
 */
static inline IMG_BOOL
LinuxMemAreaUsesPhysicalMap(LinuxMemArea *psLinuxMemArea)
{
    return LinuxMemAreaPhysIsContig(psLinuxMemArea);
}
#endif

#if !defined(PVR_MAKE_ALL_PFNS_SPECIAL)
static inline IMG_UINT32
GetCurrentThreadID(IMG_VOID)
{
	/*
 	 * The PID is the thread ID, as each thread is a
 	 * seperate process.
 	 */
	return (IMG_UINT32)current->pid;
}
#endif

/*
 * Create an offset structure, which is used to hold per-process
 * mmap data.
 */
static PKV_OFFSET_STRUCT
CreateOffsetStruct(LinuxMemArea *psLinuxMemArea, IMG_UINT32 ui32Offset, IMG_UINT32 ui32RealByteSize)
{
    PKV_OFFSET_STRUCT psOffsetStruct;
#if defined(DEBUG) || defined(DEBUG_LINUX_MMAP_AREAS)
    const IMG_CHAR *pszName = LinuxMemAreaTypeToString(LinuxMemAreaRootType(psLinuxMemArea));
#endif

#if defined(DEBUG) || defined(DEBUG_LINUX_MMAP_AREAS)
    PVR_DPF((PVR_DBG_MESSAGE,
             "%s(%s, psLinuxMemArea: 0x%p, ui32AllocFlags: 0x%8x)",
             __FUNCTION__, pszName, psLinuxMemArea, psLinuxMemArea->ui32AreaFlags));
#endif

    PVR_ASSERT(psLinuxMemArea->eAreaType != LINUX_MEM_AREA_SUB_ALLOC || LinuxMemAreaRoot(psLinuxMemArea)->eAreaType != LINUX_MEM_AREA_SUB_ALLOC);

    PVR_ASSERT(psLinuxMemArea->bMMapRegistered);

    psOffsetStruct = KMemCacheAllocWrapper(g_psMemmapCache, GFP_KERNEL);
    if(psOffsetStruct == IMG_NULL)
    {
        PVR_DPF((PVR_DBG_ERROR,"PVRMMapRegisterArea: Couldn't alloc another mapping record from cache"));
        return IMG_NULL;
    }
    
    psOffsetStruct->ui32MMapOffset = ui32Offset;

    psOffsetStruct->psLinuxMemArea = psLinuxMemArea;

    psOffsetStruct->ui32RealByteSize = ui32RealByteSize;

    /*
     * We store the TID in case two threads within a process
     * generate the same offset structure, and both end up on the
     * list of structures waiting to be mapped, at the same time.
     * This could happen if two sub areas within the same page are
     * being mapped at the same time.
     * The TID allows the mmap entry point to distinguish which
     * mapping is being done by which thread.
     */
#if !defined(PVR_MAKE_ALL_PFNS_SPECIAL)
    psOffsetStruct->ui32TID = GetCurrentThreadID();
#endif
    psOffsetStruct->ui32PID = OSGetCurrentProcessIDKM();

#if defined(DEBUG_LINUX_MMAP_AREAS)
    /* Extra entries to support proc filesystem debug info */
    psOffsetStruct->pszName = pszName;
#endif

    list_add_tail(&psOffsetStruct->sAreaItem, &psLinuxMemArea->sMMapOffsetStructList);

    return psOffsetStruct;
}


static IMG_VOID
DestroyOffsetStruct(PKV_OFFSET_STRUCT psOffsetStruct)
{
#ifdef DEBUG
    IMG_CPU_PHYADDR CpuPAddr;
    CpuPAddr = LinuxMemAreaToCpuPAddr(psOffsetStruct->psLinuxMemArea, 0);
#endif

    list_del(&psOffsetStruct->sAreaItem);

    if (psOffsetStruct->bOnMMapList)
    {
        list_del(&psOffsetStruct->sMMapItem);
    }

#ifdef DEBUG
    PVR_DPF((PVR_DBG_MESSAGE, "%s: Table entry: "
             "psLinuxMemArea=%p, CpuPAddr=0x%08X", __FUNCTION__,
             psOffsetStruct->psLinuxMemArea,
             CpuPAddr.uiAddr));
#endif
    
    KMemCacheFreeWrapper(g_psMemmapCache, psOffsetStruct);
}


/*
 * There are no alignment constraints for mapping requests made by user
 * mode Services.  For this, and potentially other reasons, the
 * mapping created for a users request may look different to the
 * original request in terms of size and alignment.
 *
 * This function determines an offset that the user can add to the mapping
 * that is _actually_ created which will point to the memory they are
 * _really_ interested in.
 *
 */
static inline IMG_VOID
DetermineUsersSizeAndByteOffset(LinuxMemArea *psLinuxMemArea,
                               IMG_UINT32 *pui32RealByteSize,
                               IMG_UINT32 *pui32ByteOffset)
{
    IMG_UINT32 ui32PageAlignmentOffset;
    IMG_CPU_PHYADDR CpuPAddr;
    
    CpuPAddr = LinuxMemAreaToCpuPAddr(psLinuxMemArea, 0);
    ui32PageAlignmentOffset = ADDR_TO_PAGE_OFFSET(CpuPAddr.uiAddr);
    
    *pui32ByteOffset = ui32PageAlignmentOffset;

    *pui32RealByteSize = PAGE_ALIGN(psLinuxMemArea->ui32ByteSize + ui32PageAlignmentOffset);
}

#if defined(SUPPORT_DRI_DRM_EXTERNAL)
#include <drm/omap_drm.h>
#include "syscommon.h"
static struct omap_gem_vm_ops gem_ops;
static struct drm_gem_object *
create_gem_wrapper(struct drm_device *dev, struct drm_file *file,
		IMG_HANDLE handle, LinuxMemArea *psLinuxMemArea,
		IMG_UINT32 ui32ByteOffset, IMG_UINT32 ui32ByteSize)
{
	/* create a new GEM buffer wrapping this mem-area.. */
	union omap_gem_size gsize;
	uint32_t flags;
	struct page **pages = NULL;
	unsigned long paddr = 0;
	int i, npages = PAGE_ALIGN(ui32ByteSize) / PAGE_SIZE;


	/* from GEM buffer object point of view, we are either mapping
	 * in terms of an array of struct pages for (potentially)
	 * discontiguous RAM, or in terms of a physically contiguous
	 * area (which could be RAM or IO) specified in terms of a
	 * base physical address.  So try and convert the mem-area
	 * to one of these two types.
	 *
	 * TODO double check this.. I don't see any evidence that PVR
	 * is doing anything like remapping discontiguous IO regions
	 * into one contiguous virtual user mapping..
	 */
	switch(psLinuxMemArea->eAreaType) {
	case LINUX_MEM_AREA_IOREMAP:
		paddr = psLinuxMemArea->uData.sIORemap.CPUPhysAddr.uiAddr;
		paddr += ui32ByteOffset;
		break;
	case LINUX_MEM_AREA_EXTERNAL_KV:
		if (psLinuxMemArea->uData.sExternalKV.bPhysContig) {
			paddr = SysSysPAddrToCpuPAddr(
					psLinuxMemArea->uData.sExternalKV.uPhysAddr.SysPhysAddr).uiAddr;
			paddr += ui32ByteOffset;
		} else {
			IMG_UINT32 ui32PageIndex = PHYS_TO_PFN(ui32ByteOffset);
			pages = kmalloc(sizeof(pages) * npages, GFP_KERNEL);
			for (i = 0; i < npages; i++) {
				IMG_SYS_PHYADDR SysPAddr =
						psLinuxMemArea->uData.sExternalKV.uPhysAddr.pSysPhysAddr[ui32PageIndex];
				pages[i] = phys_to_page(SysSysPAddrToCpuPAddr(SysPAddr).uiAddr);
			}
		}
		break;
	case LINUX_MEM_AREA_IO:
		paddr = psLinuxMemArea->uData.sIO.CPUPhysAddr.uiAddr;
		paddr += ui32ByteOffset;
		break;
	case LINUX_MEM_AREA_VMALLOC:
		pages = kmalloc(sizeof(pages) * npages, GFP_KERNEL);
		for (i = 0; i < npages; i++) {
			char *vaddr = ((char *)psLinuxMemArea->uData.sVmalloc.pvVmallocAddress) +
					(i * PAGE_SIZE) + ui32ByteOffset;
			pages[i] = vmalloc_to_page(vaddr);
		}
		break;
	case LINUX_MEM_AREA_ALLOC_PAGES:
		pages = kmalloc(sizeof(pages) * npages, GFP_KERNEL);
		for (i = 0; i < npages; i++) {
			pages[i] = psLinuxMemArea->uData.sPageList.ppsPageList[i + PHYS_TO_PFN(ui32ByteOffset)];
		}
		break;
	case LINUX_MEM_AREA_SUB_ALLOC:
		return create_gem_wrapper(dev, file, handle,
				psLinuxMemArea->uData.sSubAlloc.psParentLinuxMemArea,
				psLinuxMemArea->uData.sSubAlloc.ui32ByteOffset + ui32ByteOffset,
				ui32ByteSize);
		break;
	default:
		PVR_DPF((PVR_DBG_ERROR, "%s: Unknown LinuxMemArea type (%d)\n",
				__FUNCTION__, psLinuxMemArea->eAreaType));
		return NULL;
	}

	/* map PVR cache type flags to GEM.. */
	switch(psLinuxMemArea->ui32AreaFlags & PVRSRV_HAP_CACHETYPE_MASK) {
	case PVRSRV_HAP_CACHED:
		flags = OMAP_BO_CACHED;
		break;
	case PVRSRV_HAP_WRITECOMBINE:
		flags = OMAP_BO_WC;
		break;
	case PVRSRV_HAP_UNCACHED:
		flags = OMAP_BO_UNCACHED;
		break;
	default:
		PVR_DPF((PVR_DBG_ERROR, "%s: unknown cache type", __FUNCTION__));
		return NULL;
	}

	/* we need to give GEM page aligned address, because that is what will
	 * be mapped.. userspace will figure out the offset..
	 */
	paddr &= ~(PAGE_SIZE-1);

	gsize.bytes = ui32ByteSize;
	return omap_gem_new_ext(dev, file, handle, gsize, flags, paddr, pages, &gem_ops);
}
#endif /* SUPPORT_DRI_DRM_EXTERNAL */


/*!
 *******************************************************************************

 @Function  PVRMMapOSMemHandleToMMapData

 @Description

 Determine various parameters needed to mmap a memory area, and to
 locate the memory within the mapped area.

 @input psPerProc : Per-process data.
 @input hMHandle : Memory handle.
 @input pui32MMapOffset : pointer to location for returned mmap offset.
 @input pui32ByteOffset : pointer to location for returned byte offset.
 @input pui32RealByteSize : pointer to location for returned real byte size.
 @input pui32UserVaddr : pointer to location for returned user mode address.

 @output pui32MMapOffset : points to mmap offset to be used in mmap2 sys call.
 @output pui32ByteOffset : points to byte offset of start of memory
			   within mapped area returned by mmap2.
 @output pui32RealByteSize : points to size of area to be mapped.
 @output pui32UserVAddr : points to user mode address of start of
			  mapping, or 0 if it hasn't been mapped yet.

 @Return PVRSRV_ERROR : PVRSRV_OK, or error code.

 ******************************************************************************/
PVRSRV_ERROR
PVRMMapOSMemHandleToMMapData(PVRSRV_PER_PROCESS_DATA *psPerProc,
#if defined (SUPPORT_SID_INTERFACE)
                             IMG_SID     hMHandle,
#else
                             IMG_HANDLE hMHandle,
#endif
                             IMG_UINT32 *pui32MMapOffset,
                             IMG_UINT32 *pui32ByteOffset,
                             IMG_UINT32 *pui32RealByteSize,
                             IMG_UINT32 *pui32UserVAddr)
{
    LinuxMemArea *psLinuxMemArea;
    PKV_OFFSET_STRUCT psOffsetStruct;
    IMG_HANDLE hOSMemHandle;
    PVRSRV_ERROR eError;
#if defined(SUPPORT_DRI_DRM_EXTERNAL)
	PVRSRV_ENV_PER_PROCESS_DATA *psEnvPerProc =
			(PVRSRV_ENV_PER_PROCESS_DATA *)PVRSRVProcessPrivateData(psPerProc);
	struct drm_gem_object *buf = NULL;
        IMG_UINT32 *puiHandle;
	IMG_UINT32 ret;
#endif /* SUPPORT_DRI_DRM_EXTERNAL */

    LinuxLockMutexNested(&g_sMMapMutex, PVRSRV_LOCK_CLASS_MMAP);

    PVR_ASSERT(PVRSRVGetMaxHandle(psPerProc->psHandleBase) <= MAX_MMAP_HANDLE);

    eError = PVRSRVLookupOSMemHandle(psPerProc->psHandleBase, &hOSMemHandle, hMHandle);
    if (eError != PVRSRV_OK)
    {
#if defined (SUPPORT_SID_INTERFACE)
        PVR_DPF((PVR_DBG_ERROR, "%s: Lookup of handle %x failed", __FUNCTION__, hMHandle));
#else
        PVR_DPF((PVR_DBG_ERROR, "%s: Lookup of handle %p failed", __FUNCTION__, hMHandle));
#endif

        goto exit_unlock;
    }

    psLinuxMemArea = (LinuxMemArea *)hOSMemHandle;
    puiHandle = &psLinuxMemArea->uiHandle;

        /* Sparse mappings have to ask the BM for the virtual size */
	if (psLinuxMemArea->hBMHandle)
	{
		*pui32RealByteSize = BM_GetVirtualSize(psLinuxMemArea->hBMHandle);
		*pui32ByteOffset = 0;
	}
	else
	{
		DetermineUsersSizeAndByteOffset(psLinuxMemArea,
										pui32RealByteSize,
										pui32ByteOffset);
	}

#if defined(SUPPORT_DRI_DRM_EXTERNAL)
    /* if we are using DRM/GEM, then let GEM generate the buffer offset..
     * this is done by creating a wrapper object.
     */
    if (psEnvPerProc->dev && psEnvPerProc->file)
    {
        buf = psLinuxMemArea->buf;
        if (!buf)
        {
            buf = create_gem_wrapper(psEnvPerProc->dev, psEnvPerProc->file, hMHandle,
                    psLinuxMemArea, 0, *pui32RealByteSize);
	    if(buf){
		ret = drm_gem_handle_create(psEnvPerProc->file, (struct drm_gem_object *)buf, puiHandle);
		if(ret) {
			/*This means we are royaly screwed up. Go home. */
			/*Please don't worry about not freeing the GEM. */
			/*DRM core will take care of it, eventually.    */
			buf = NULL;
		}
	    }
            if (!buf)
            {
                PVR_DPF((PVR_DBG_ERROR, "%s: Screw you guys, I'm going home..", __FUNCTION__));
                eError = PVRSRV_ERROR_OUT_OF_MEMORY;
                goto exit_unlock;
            }

            psLinuxMemArea->buf = buf;
        } else {
		ret = drm_gem_handle_create(psEnvPerProc->file, (struct drm_gem_object *)buf, puiHandle);
		if(ret) {
			PVR_DPF((PVR_DBG_ERROR, "%s: Screw you guys, I'm going home..", __FUNCTION__));
			eError = PVRSRV_ERROR_OUT_OF_MEMORY;
			goto exit_unlock;
		}
	}
    }
#endif /* SUPPORT_DRI_DRM_EXTERNAL */

    psOffsetStruct = FindOffsetStructByPID(psLinuxMemArea, psPerProc->ui32PID);
    if (psOffsetStruct)
    {
			if (!psLinuxMemArea->hBMHandle)
			{
				PVR_ASSERT(*pui32RealByteSize == psOffsetStruct->ui32RealByteSize);
			}
	   /*
	    * User mode locking is required to stop two threads racing to
	    * map the same memory area.  The lock should prevent a
	    * second thread retrieving mmap data for a given handle,
	    * before the first thread has done the mmap.
	    * Without locking, both threads may attempt the mmap,
	    * and one of them will fail.
	    */
	   *pui32MMapOffset = psOffsetStruct->ui32MMapOffset;
	   *pui32UserVAddr = psOffsetStruct->ui32UserVAddr;
	   PVRSRVOffsetStructIncRef(psOffsetStruct);

	   eError = PVRSRV_OK;
	   goto exit_unlock;
    }

    /* Memory area won't have been mapped yet */
    *pui32UserVAddr = 0;

#if !defined(PVR_MAKE_ALL_PFNS_SPECIAL)
    if (LinuxMemAreaUsesPhysicalMap(psLinuxMemArea))
    {
        *pui32MMapOffset = LinuxMemAreaToCpuPFN(psLinuxMemArea, 0);
        PVR_ASSERT(PFNIsPhysical(*pui32MMapOffset));
    }
    else
#endif
    {
        *pui32MMapOffset = HandleToMMapOffset(hMHandle);
#if !defined(PVR_MAKE_ALL_PFNS_SPECIAL)
        PVR_ASSERT(PFNIsSpecial(*pui32MMapOffset));
#endif
    }

#if defined(SUPPORT_DRI_DRM_EXTERNAL)
	if (buf)
	{
        /* note: omap_gem_mmap_offset() gives us a byte offset that is multiple
         * of pages, which can be passed to mmap().. but PVR wants a page offset
         * that can be passed to mmap2().  So convert it back to pages:
         */
	    *pui32MMapOffset = omap_gem_mmap_offset(buf) >> PAGE_SHIFT;
	}
#endif /* SUPPORT_DRI_DRM_EXTERNAL */

    psOffsetStruct = CreateOffsetStruct(psLinuxMemArea, *pui32MMapOffset, *pui32RealByteSize);
    if (psOffsetStruct == IMG_NULL)
    {
        eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	goto exit_unlock;
    }

    /*
    * Offset structures representing physical mappings are added to
    * a list, so that they can be located when the memory area is mapped.
    */
    list_add_tail(&psOffsetStruct->sMMapItem, &g_sMMapOffsetStructList);

    psOffsetStruct->bOnMMapList = IMG_TRUE;

    PVRSRVOffsetStructIncRef(psOffsetStruct);

    eError = PVRSRV_OK;

	/* Need to scale up the offset to counter the shifting that
	   is done in the mmap2() syscall, as it expects the pgoff
	   argument to be in units of 4,096 bytes irrespective of
	   page size */
	*pui32MMapOffset = *pui32MMapOffset << (PAGE_SHIFT - 12);

exit_unlock:

    LinuxUnLockMutex(&g_sMMapMutex);

    return eError;
}


/*!
 *******************************************************************************

 @Function  PVRMMapReleaseMMapData

 @Description

 Release mmap data.

 @input psPerProc : Per-process data.
 @input hMHandle : Memory handle.
 @input pbMUnmap : pointer to location for munmap flag.
 @input pui32UserVAddr : pointer to location for user mode address of mapping.
 @input pui32ByteSize : pointer to location for size of mapping.

 @Output pbMUnmap : points to flag that indicates whether an munmap is
		    required.
 @output pui32UserVAddr : points to user mode address to munmap.

 @Return PVRSRV_ERROR : PVRSRV_OK, or error code.

 ******************************************************************************/
PVRSRV_ERROR
PVRMMapReleaseMMapData(PVRSRV_PER_PROCESS_DATA *psPerProc,
#if defined (SUPPORT_SID_INTERFACE)
				IMG_SID   hMHandle,
#else
				IMG_HANDLE hMHandle,
#endif
				IMG_BOOL *pbMUnmap,
				IMG_UINT32 *pui32RealByteSize,
                                IMG_UINT32 *pui32UserVAddr)
{
    LinuxMemArea *psLinuxMemArea;
    PKV_OFFSET_STRUCT psOffsetStruct;
    IMG_HANDLE hOSMemHandle;
    PVRSRV_ERROR eError;
    IMG_UINT32 ui32PID = OSGetCurrentProcessIDKM();
#if defined(SUPPORT_DRI_DRM_EXTERNAL)
	PVRSRV_ENV_PER_PROCESS_DATA *psEnvPerProc =
			(PVRSRV_ENV_PER_PROCESS_DATA *)PVRSRVProcessPrivateData(psPerProc);
#endif /* SUPPORT_DRI_DRM_EXTERNAL */

    LinuxLockMutexNested(&g_sMMapMutex, PVRSRV_LOCK_CLASS_MMAP);

    PVR_ASSERT(PVRSRVGetMaxHandle(psPerProc->psHandleBase) <= MAX_MMAP_HANDLE);

    eError = PVRSRVLookupOSMemHandle(psPerProc->psHandleBase, &hOSMemHandle, hMHandle);
    if (eError != PVRSRV_OK)
    {
#if defined (SUPPORT_SID_INTERFACE)
	PVR_DPF((PVR_DBG_ERROR, "%s: Lookup of handle %x failed", __FUNCTION__, hMHandle));
#else
	PVR_DPF((PVR_DBG_ERROR, "%s: Lookup of handle %p failed", __FUNCTION__, hMHandle));
#endif

	goto exit_unlock;
    }

    psLinuxMemArea = (LinuxMemArea *)hOSMemHandle;

#if defined(SUPPORT_DRI_DRM_EXTERNAL)
    if (psLinuxMemArea->uiHandle) {
	drm_gem_handle_delete(psEnvPerProc->file, psLinuxMemArea->uiHandle);
	psLinuxMemArea->uiHandle = 0;
    }
#endif

    psOffsetStruct = FindOffsetStructByPID(psLinuxMemArea, ui32PID);
    if (psOffsetStruct)
    {
	    if (psOffsetStruct->ui32RefCount == 0)
	    {
		PVR_DPF((PVR_DBG_ERROR, "%s: Attempt to release mmap data with zero reference count for offset struct 0x%p, memory area %p", __FUNCTION__, psOffsetStruct, psLinuxMemArea));
		eError = PVRSRV_ERROR_STILL_MAPPED;
		goto exit_unlock;
	    }

	    PVRSRVOffsetStructDecRef(psOffsetStruct);

	    *pbMUnmap = (IMG_BOOL)((psOffsetStruct->ui32RefCount == 0) && (psOffsetStruct->ui32UserVAddr != 0));

	    *pui32UserVAddr = (*pbMUnmap) ? psOffsetStruct->ui32UserVAddr : 0;
	    *pui32RealByteSize = (*pbMUnmap) ? psOffsetStruct->ui32RealByteSize : 0;

	    eError = PVRSRV_OK;
	    goto exit_unlock;
    }

    /* MMap data not found */
#if defined (SUPPORT_SID_INTERFACE)
    PVR_DPF((PVR_DBG_ERROR, "%s: Mapping data not found for handle %x (memory area %p)", __FUNCTION__, hMHandle, psLinuxMemArea));
#else
    PVR_DPF((PVR_DBG_ERROR, "%s: Mapping data not found for handle %p (memory area %p)", __FUNCTION__, hMHandle, psLinuxMemArea));
#endif

    eError =  PVRSRV_ERROR_MAPPING_NOT_FOUND;

exit_unlock:
    LinuxUnLockMutex(&g_sMMapMutex);

    return eError;
}

static inline PKV_OFFSET_STRUCT
FindOffsetStructByOffset(IMG_UINT32 ui32Offset, IMG_UINT32 ui32RealByteSize)
{
    PKV_OFFSET_STRUCT psOffsetStruct;
#if !defined(PVR_MAKE_ALL_PFNS_SPECIAL)
    IMG_UINT32 ui32TID = GetCurrentThreadID();
#endif
    IMG_UINT32 ui32PID = OSGetCurrentProcessIDKM();

    list_for_each_entry(psOffsetStruct, &g_sMMapOffsetStructList, sMMapItem)
    {
        if (ui32Offset == psOffsetStruct->ui32MMapOffset && ui32RealByteSize == psOffsetStruct->ui32RealByteSize && psOffsetStruct->ui32PID == ui32PID)
        {
#if !defined(PVR_MAKE_ALL_PFNS_SPECIAL)
	    /*
	     * If the offset is physical, make sure the thread IDs match,
	     * as different threads may be mapping different memory areas
	     * with the same offset.
	     */
	    if (!PFNIsPhysical(ui32Offset) || psOffsetStruct->ui32TID == ui32TID)
#endif
	    {
	        return psOffsetStruct;
	    }
        }
    }

    return IMG_NULL;
}

static inline PKV_OFFSET_STRUCT
FindOffsetStructByPID(LinuxMemArea *psLinuxMemArea, IMG_UINT32 ui32PID)
{
	PKV_OFFSET_STRUCT psOffsetStruct;
	list_for_each_entry(psOffsetStruct, &psLinuxMemArea->sMMapOffsetStructList, sAreaItem)
	{
		if (psOffsetStruct->ui32PID == ui32PID)
		{
			return psOffsetStruct;
		}
	}
	return NULL;
}
/*
 * Map a memory area into user space.
 * Note, the ui32ByteOffset is _not_ implicitly page aligned since
 * LINUX_MEM_AREA_SUB_ALLOC LinuxMemAreas have no alignment constraints.
 */
static IMG_BOOL
DoMapToUser(LinuxMemArea *psLinuxMemArea,
            struct vm_area_struct* ps_vma,
            IMG_UINT32 ui32ByteOffset)
{
    IMG_UINT32 ui32ByteSize;

	if ((psLinuxMemArea->hBMHandle) && (ui32ByteOffset != 0))
	{
		/* Partial mapping of sparse allocations should never happen */
		return IMG_FALSE;
	}

    if (psLinuxMemArea->eAreaType == LINUX_MEM_AREA_SUB_ALLOC)
    {
        return DoMapToUser(LinuxMemAreaRoot(psLinuxMemArea),		/* PRQA S 3670 */ /* allow recursion */
                    ps_vma,
                    psLinuxMemArea->uData.sSubAlloc.ui32ByteOffset + ui32ByteOffset);
    }

    /*
     * Note that ui32ByteSize may be larger than the size of the memory
     * area being mapped, as the former is a multiple of the page size.
     */
    ui32ByteSize = ps_vma->vm_end - ps_vma->vm_start;
    PVR_ASSERT(ADDR_TO_PAGE_OFFSET(ui32ByteSize) == 0);

#if defined (__sparc__)
    /*
     * For LINUX_MEM_AREA_EXTERNAL_KV, we don't know where the address range
     * we are being asked to map has come from, that is, whether it is memory
     * or I/O.  For all architectures other than SPARC, there is no distinction.
     * Since we don't currently support SPARC, we won't worry about it.
     */
#error "SPARC not supported"
#endif

#if !defined(PVR_MAKE_ALL_PFNS_SPECIAL)
    if (PFNIsPhysical(ps_vma->vm_pgoff))
    {
	IMG_INT result;

	PVR_ASSERT(LinuxMemAreaPhysIsContig(psLinuxMemArea));
	PVR_ASSERT(LinuxMemAreaToCpuPFN(psLinuxMemArea, ui32ByteOffset) == ps_vma->vm_pgoff);
        /*
	 * Since the memory is contiguous, we can map the whole range in one
	 * go .
	 */

	PVR_ASSERT(psLinuxMemArea->hBMHandle == IMG_NULL);

	result = IO_REMAP_PFN_RANGE(ps_vma, ps_vma->vm_start, ps_vma->vm_pgoff, ui32ByteSize, ps_vma->vm_page_prot);

        if(result == 0)
        {
            return IMG_TRUE;
        }

        PVR_DPF((PVR_DBG_MESSAGE, "%s: Failed to map contiguous physical address range (%d), trying non-contiguous path", __FUNCTION__, result));
    }
#endif

    {
        /*
         * Memory may be non-contiguous, so we map the range page,
	 * by page.  Since VM_PFNMAP mappings are assumed to be physically
	 * contiguous, we can't legally use REMAP_PFN_RANGE (that is, we
	 * could, but the resulting VMA may confuse other bits of the kernel
	 * that attempt to interpret it).
	 * The only alternative is to use VM_INSERT_PAGE, which requires
	 * finding the page structure corresponding to each page, or
	 * if mixed maps are supported (VM_MIXEDMAP), vm_insert_mixed.
	 */
        IMG_UINT32 ulVMAPos;
	IMG_UINT32 ui32ByteEnd = ui32ByteOffset + ui32ByteSize;
	IMG_UINT32 ui32PA;
	IMG_UINT32 ui32AdjustedPA = ui32ByteOffset;
#if defined(PVR_MAKE_ALL_PFNS_SPECIAL)
	IMG_BOOL bMixedMap = IMG_FALSE;
#endif
	/* First pass, validate the page frame numbers */
	for(ui32PA = ui32ByteOffset; ui32PA < ui32ByteEnd; ui32PA += PAGE_SIZE)
	{
		IMG_UINT32 pfn;
	    IMG_BOOL bMapPage = IMG_TRUE;

		if (psLinuxMemArea->hBMHandle)
		{
			if (!BM_MapPageAtOffset(psLinuxMemArea->hBMHandle, ui32PA))
			{
				bMapPage = IMG_FALSE;
			}
		}

		if (bMapPage)
		{
			pfn =  LinuxMemAreaToCpuPFN(psLinuxMemArea, ui32AdjustedPA);
			if (!pfn_valid(pfn))
			{
#if !defined(PVR_MAKE_ALL_PFNS_SPECIAL)
					PVR_DPF((PVR_DBG_ERROR,"%s: Error - PFN invalid: 0x%x", __FUNCTION__, pfn));
					return IMG_FALSE;
#else
			bMixedMap = IMG_TRUE;
#endif
			}
			ui32AdjustedPA += PAGE_SIZE;
		}
	}

#if defined(PVR_MAKE_ALL_PFNS_SPECIAL)
	if (bMixedMap)
	{
            ps_vma->vm_flags |= VM_MIXEDMAP;
	}
#endif
	/* Second pass, get the page structures and insert the pages */
        ulVMAPos = ps_vma->vm_start;
        ui32AdjustedPA = ui32ByteOffset;
	for(ui32PA = ui32ByteOffset; ui32PA < ui32ByteEnd; ui32PA += PAGE_SIZE)
	{
	    IMG_UINT32 pfn;
	    IMG_INT result;
	    IMG_BOOL bMapPage = IMG_TRUE;

		if (psLinuxMemArea->hBMHandle)
		{
			/* We have a sparse allocation, check if this page should be mapped */
			if (!BM_MapPageAtOffset(psLinuxMemArea->hBMHandle, ui32PA))
			{
				bMapPage = IMG_FALSE;
			}
		}

		if (bMapPage)
		{
			pfn =  LinuxMemAreaToCpuPFN(psLinuxMemArea, ui32AdjustedPA);

#if defined(PVR_MAKE_ALL_PFNS_SPECIAL)
		    if (bMixedMap)
		    {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,20,0))
			result = vmf_insert_mixed(ps_vma, ulVMAPos, pfn_to_pfn_t(pfn));
			if(result != 0)
			{
			    PVR_DPF((PVR_DBG_ERROR,"%s: Error - vmf_insert_mixed failed (%x)", __FUNCTION__, result));
			    return IMG_FALSE;
			}
#else
			result = vm_insert_mixed(ps_vma, ulVMAPos, pfn);
	                if(result != 0)
	                {
	                    PVR_DPF((PVR_DBG_ERROR,"%s: Error - vm_insert_mixed failed (%d)", __FUNCTION__, result));
	                    return IMG_FALSE;
	                }
#endif
		    }
		    else
#endif
		    {
			struct page *psPage;
	
		        PVR_ASSERT(pfn_valid(pfn));
	
		        psPage = pfn_to_page(pfn);
	
		        result = VM_INSERT_PAGE(ps_vma,  ulVMAPos, psPage);
	                if(result != 0)
	                {
	                    PVR_DPF((PVR_DBG_ERROR,"%s: Error - VM_INSERT_PAGE failed (%d)", __FUNCTION__, result));
	                    return IMG_FALSE;
	                }
		    }
		    ui32AdjustedPA += PAGE_SIZE;
		}
        ulVMAPos += PAGE_SIZE;
    }
    }

    return IMG_TRUE;
}


static IMG_VOID
MMapVOpenNoLock(struct vm_area_struct* ps_vma, PKV_OFFSET_STRUCT psOffsetStruct)
{
    PVR_ASSERT(psOffsetStruct != IMG_NULL);
    PVR_ASSERT(!psOffsetStruct->bOnMMapList);

    PVRSRVOffsetStructIncMapped(psOffsetStruct);

    if (psOffsetStruct->ui32Mapped > 1)
    {
	PVR_DPF((PVR_DBG_WARNING, "%s: Offset structure 0x%p is being shared across processes (psOffsetStruct->ui32Mapped: %u)", __FUNCTION__, psOffsetStruct, psOffsetStruct->ui32Mapped));
        PVR_ASSERT((ps_vma->vm_flags & VM_DONTCOPY) == 0);
    }

#if defined(DEBUG_LINUX_MMAP_AREAS)

    PVR_DPF((PVR_DBG_MESSAGE,
             "%s: psLinuxMemArea 0x%p, KVAddress 0x%p MMapOffset %d, ui32Mapped %d",
             __FUNCTION__,
             psOffsetStruct->psLinuxMemArea,
             LinuxMemAreaToCpuVAddr(psOffsetStruct->psLinuxMemArea),
             psOffsetStruct->ui32MMapOffset,
             psOffsetStruct->ui32Mapped));
#endif
}


/*
 * Linux mmap open entry point.
 */
static void
MMapVOpen(struct vm_area_struct* ps_vma)
{
    LinuxLockMutexNested(&g_sMMapMutex, PVRSRV_LOCK_CLASS_MMAP);

    MMapVOpenNoLock(ps_vma, ps_vma->vm_private_data);

    LinuxUnLockMutex(&g_sMMapMutex);
}


static IMG_VOID
MMapVCloseNoLock(struct vm_area_struct* ps_vma, PKV_OFFSET_STRUCT psOffsetStruct)
{
    struct drm_gem_object *obj;

    WARN_ON(!psOffsetStruct);
    if (!psOffsetStruct) {
        return;
    }

#if defined(DEBUG_LINUX_MMAP_AREAS)
    PVR_DPF((PVR_DBG_MESSAGE,
             "%s: psLinuxMemArea %p, CpuVAddr %p ui32MMapOffset %d, ui32Mapped %d",
             __FUNCTION__,
             psOffsetStruct->psLinuxMemArea,
             LinuxMemAreaToCpuVAddr(psOffsetStruct->psLinuxMemArea),
             psOffsetStruct->ui32MMapOffset,
             psOffsetStruct->ui32Mapped));
#endif

    PVR_ASSERT(!psOffsetStruct->bOnMMapList);
    PVRSRVOffsetStructDecMapped(psOffsetStruct);
    if (psOffsetStruct->ui32Mapped == 0)
    {
	if (psOffsetStruct->ui32RefCount != 0)
	{
	        PVR_DPF((PVR_DBG_MESSAGE, "%s: psOffsetStruct %p has non-zero reference count (ui32RefCount = %u). User mode address of start of mapping: 0x%x", __FUNCTION__, psOffsetStruct, psOffsetStruct->ui32RefCount, psOffsetStruct->ui32UserVAddr));
	}

	DestroyOffsetStruct(psOffsetStruct);
    }

    obj = ps_vma->vm_private_data;
    drm_gem_object_put_unlocked(obj);

    ps_vma->vm_private_data = NULL;
}

/*
 * Linux mmap close entry point.
 */
static void
MMapVClose(struct vm_area_struct* ps_vma)
{
    LinuxLockMutexNested(&g_sMMapMutex, PVRSRV_LOCK_CLASS_MMAP);

    MMapVCloseNoLock(ps_vma, ps_vma->vm_private_data);

    LinuxUnLockMutex(&g_sMMapMutex);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26))
/*
 * This vma operation is used to read data from mmap regions. It is called
 * by access_process_vm, which is called to handle PTRACE_PEEKDATA ptrace
 * requests and reads from /proc/<pid>/mem.
 */
static int MMapVAccess(struct vm_area_struct *ps_vma, unsigned long addr,
					   void *buf, int len, int write)
{
    PKV_OFFSET_STRUCT psOffsetStruct;
    LinuxMemArea *psLinuxMemArea;
    unsigned long ulOffset;
	int iRetVal = -EINVAL;
	IMG_VOID *pvKernelAddr;

	LinuxLockMutexNested(&g_sMMapMutex, PVRSRV_LOCK_CLASS_MMAP);

	psOffsetStruct = (PKV_OFFSET_STRUCT)ps_vma->vm_private_data;
	psLinuxMemArea = psOffsetStruct->psLinuxMemArea;
	ulOffset = addr - ps_vma->vm_start;

    if (ulOffset+len > psLinuxMemArea->ui32ByteSize)
		/* Out of range. We shouldn't get here, because the kernel will do
		   the necessary checks before calling access_process_vm. */
		goto exit_unlock;

	pvKernelAddr = LinuxMemAreaToCpuVAddr(psLinuxMemArea);

	if (pvKernelAddr)
	{
		memcpy(buf, pvKernelAddr+ulOffset, len);
		iRetVal = len;
	}
	else
	{
		IMG_UINT32 pfn, ui32OffsetInPage;
		struct page *page;

		pfn = LinuxMemAreaToCpuPFN(psLinuxMemArea, ulOffset);

		if (!pfn_valid(pfn))
			goto exit_unlock;

		page = pfn_to_page(pfn);
		ui32OffsetInPage = ADDR_TO_PAGE_OFFSET(ulOffset);

		if (ui32OffsetInPage+len > PAGE_SIZE)
			/* The region crosses a page boundary */
			goto exit_unlock;

		pvKernelAddr = kmap(page);
		memcpy(buf, pvKernelAddr+ui32OffsetInPage, len);
		kunmap(page);

		iRetVal = len;
	}

exit_unlock:
	LinuxUnLockMutex(&g_sMMapMutex);
    return iRetVal;
}
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26) */

static struct vm_operations_struct MMapIOOps =
{
	.open=MMapVOpen,
	.close=MMapVClose,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26))
	.access=MMapVAccess,
#endif
};


/*!
 *******************************************************************************

 @Function  PVRMMap

 @Description

 Driver mmap entry point.

 @input pFile : unused.
 @input ps_vma : pointer to linux memory area descriptor.

 @Return 0, or Linux error code.

 ******************************************************************************/
int
PVRMMap(struct file* pFile, struct vm_area_struct* ps_vma)
{
    LinuxMemArea *psFlushMemArea = IMG_NULL;
    PKV_OFFSET_STRUCT psOffsetStruct;
    IMG_UINT32 ui32ByteSize;
    IMG_VOID *pvBase = IMG_NULL;
    int iRetVal = 0;
    IMG_UINT32 ui32ByteOffset = 0;	/* Keep compiler happy */
    IMG_UINT32 ui32FlushSize = 0;

    PVR_UNREFERENCED_PARAMETER(pFile);

    LinuxLockMutexNested(&g_sMMapMutex, PVRSRV_LOCK_CLASS_MMAP);
    
    ui32ByteSize = ps_vma->vm_end - ps_vma->vm_start;
    
    PVR_DPF((PVR_DBG_MESSAGE, "%s: Received mmap(2) request with ui32MMapOffset 0x%08lx,"
                              " and ui32ByteSize %d(0x%08x)",
            __FUNCTION__,
            ps_vma->vm_pgoff,
            ui32ByteSize, ui32ByteSize));
   
    psOffsetStruct = FindOffsetStructByOffset(ps_vma->vm_pgoff, ui32ByteSize);

    if (psOffsetStruct == IMG_NULL)
    {
#if defined(SUPPORT_DRI_DRM)
        LinuxUnLockMutex(&g_sMMapMutex);

#if !defined(SUPPORT_DRI_DRM_EXT)
        /* Pass unknown requests onto the DRM module */
	return drm_gem_mmap(pFile, ps_vma);
#else
        /*
         * Indicate to caller that the request is not for us.
         * Do not return this error elsewhere in this function, as the
         * caller may use it as a clue as to whether the mmap request
         * should be passed on to another component (e.g. drm_mmap).
         */
        return -ENOENT;
#endif
#else
        PVR_UNREFERENCED_PARAMETER(pFile);

        PVR_DPF((PVR_DBG_ERROR,
             "%s: Attempted to mmap unregistered area at vm_pgoff 0x%lx",
             __FUNCTION__, ps_vma->vm_pgoff));
        iRetVal = -EINVAL;
#endif
        goto unlock_and_return;
    }

    list_del(&psOffsetStruct->sMMapItem);
    psOffsetStruct->bOnMMapList = IMG_FALSE;

    /* Only support shared writeable mappings */
    if (((ps_vma->vm_flags & VM_WRITE) != 0) &&
        ((ps_vma->vm_flags & VM_SHARED) == 0))
    {
        PVR_DPF((PVR_DBG_ERROR, "%s: Cannot mmap non-shareable writable areas", __FUNCTION__));
        iRetVal = -EINVAL;
        goto unlock_and_return;
    }
   
    PVR_DPF((PVR_DBG_MESSAGE, "%s: Mapped psLinuxMemArea 0x%p\n",
         __FUNCTION__, psOffsetStruct->psLinuxMemArea));

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 6, 0))
    ps_vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;
#else
    ps_vma->vm_flags |= VM_RESERVED;
#endif
    ps_vma->vm_flags |= VM_IO;

    /*
     * Disable mremap because our nopage handler assumes all
     * page requests have already been validated.
     */
    ps_vma->vm_flags |= VM_DONTEXPAND;
    
    /* Don't allow mapping to be inherited across a process fork */
    ps_vma->vm_flags |= VM_DONTCOPY;

    ps_vma->vm_private_data = (void *)psOffsetStruct;
    
    switch(psOffsetStruct->psLinuxMemArea->ui32AreaFlags & PVRSRV_HAP_CACHETYPE_MASK)
    {
        case PVRSRV_HAP_CACHED:
            /* This is the default, do nothing. */
            break;
        case PVRSRV_HAP_WRITECOMBINE:
            ps_vma->vm_page_prot = PGPROT_WC(ps_vma->vm_page_prot);
            break;
        case PVRSRV_HAP_UNCACHED:
            ps_vma->vm_page_prot = PGPROT_UC(ps_vma->vm_page_prot);
            break;
        default:
            PVR_DPF((PVR_DBG_ERROR, "%s: unknown cache type", __FUNCTION__));
            iRetVal = -EINVAL;
	    goto unlock_and_return;
    }
    
    /* Install open and close handlers for ref-counting */
    ps_vma->vm_ops = &MMapIOOps;
    
    if(!DoMapToUser(psOffsetStruct->psLinuxMemArea, ps_vma, 0))
    {
        iRetVal = -EAGAIN;
        goto unlock_and_return;
    }
    
    PVR_ASSERT(psOffsetStruct->ui32UserVAddr == 0);

    psOffsetStruct->ui32UserVAddr = ps_vma->vm_start;

    /* Compute the flush region (if necessary) inside the mmap mutex */
    if(psOffsetStruct->psLinuxMemArea->bNeedsCacheInvalidate)
    {
        psFlushMemArea = psOffsetStruct->psLinuxMemArea;

		/* Sparse mappings have to ask the BM for the virtual size */
		if (psFlushMemArea->hBMHandle)
		{
			pvBase = (IMG_VOID *)ps_vma->vm_start;
			ui32ByteOffset = 0;
			ui32FlushSize = BM_GetVirtualSize(psFlushMemArea->hBMHandle);
		}
		else
		{
	        IMG_UINT32 ui32DummyByteSize;

	        DetermineUsersSizeAndByteOffset(psFlushMemArea,
	                                        &ui32DummyByteSize,
	                                        &ui32ByteOffset);
	
	        pvBase = (IMG_VOID *)ps_vma->vm_start + ui32ByteOffset;
	        ui32FlushSize = psFlushMemArea->ui32ByteSize;
		}

        psFlushMemArea->bNeedsCacheInvalidate = IMG_FALSE;
    }


    /* Call the open routine to increment the usage count */
    MMapVOpenNoLock(ps_vma, ps_vma->vm_private_data);
    
    PVR_DPF((PVR_DBG_MESSAGE, "%s: Mapped area at offset 0x%08lx\n",
             __FUNCTION__, ps_vma->vm_pgoff));

unlock_and_return:
    if (iRetVal != 0 && psOffsetStruct != IMG_NULL)
    {
        DestroyOffsetStruct(psOffsetStruct);
    }

    LinuxUnLockMutex(&g_sMMapMutex);

    if(psFlushMemArea)
    {
        OSInvalidateCPUCacheRangeKM(psFlushMemArea, ui32ByteOffset, pvBase,
									ui32FlushSize);
    }

    return iRetVal;
}

#if defined(SUPPORT_DRI_DRM_EXTERNAL)
/* note: currently we stuff memarea struct in bo->private_data.. but there
 * must be a better way.  Currently PVR's mapper-private data is the
 * PVRSRV_KERNEL_SYNC_INFO..  I'm not sure if there is a way to go back
 * and forth between this an the offset struct?
 */

static void
MMapVOpenExt(struct vm_area_struct* ps_vma)
{
	struct drm_gem_object *obj = ps_vma->vm_private_data;
	void *priv = omap_gem_priv(obj);
	PKV_OFFSET_STRUCT psOffsetStruct =
			FindOffsetStructByPID(priv, OSGetCurrentProcessIDKM());
	if (WARN_ON(!psOffsetStruct))
		return;
	LinuxLockMutexNested(&g_sMMapMutex, PVRSRV_LOCK_CLASS_MMAP);
	MMapVOpenNoLock(ps_vma, psOffsetStruct);
	LinuxUnLockMutex(&g_sMMapMutex);
}

static void
MMapVCloseExt(struct vm_area_struct* ps_vma)
{
	struct drm_gem_object *obj = ps_vma->vm_private_data;
	void *priv = omap_gem_priv(obj);
	PKV_OFFSET_STRUCT psOffsetStruct =
			FindOffsetStructByPID(priv, OSGetCurrentProcessIDKM());
	if (WARN_ON(!psOffsetStruct))
		return;
	LinuxLockMutexNested(&g_sMMapMutex, PVRSRV_LOCK_CLASS_MMAP);
	MMapVCloseNoLock(ps_vma, psOffsetStruct);
	LinuxUnLockMutex(&g_sMMapMutex);
}

/* this function doesn't actually handle user mapping.. but does update some
 * internal data structures that would otherwise not get updated if we didn't
 * have a callback to notify of the user mapping.  It is used when something
 * outside of the PVR driver (ie. DRM) is handling the userspace mapping
 */
static void
PVRMMapExt(struct file* pFile, struct vm_area_struct* ps_vma)
{
	struct drm_gem_object *obj = ps_vma->vm_private_data;
    IMG_UINT32 ui32ByteSize;
    PKV_OFFSET_STRUCT psOffsetStruct;

    /* for cache maintenance: */
    LinuxMemArea *psFlushMemArea = IMG_NULL;
    IMG_VOID *pvBase = IMG_NULL;
    IMG_UINT32 ui32ByteOffset = 0;	/* Keep compiler happy */

    PVR_UNREFERENCED_PARAMETER(pFile);

    LinuxLockMutexNested(&g_sMMapMutex, PVRSRV_LOCK_CLASS_MMAP);

    ui32ByteSize = ps_vma->vm_end - ps_vma->vm_start;

    ps_vma->vm_flags |= VM_DONTCOPY;

	psOffsetStruct = FindOffsetStructByOffset(ps_vma->vm_pgoff, ui32ByteSize);
	if (psOffsetStruct == IMG_NULL)
	{
	    PVR_DPF((PVR_DBG_WARNING, "%s: No mapped area at offset 0x%08lx, size=%d\n",
	             __FUNCTION__, ps_vma->vm_pgoff, ui32ByteSize));
		goto unlock_and_return;
	}
	list_del(&psOffsetStruct->sMMapItem);
	psOffsetStruct->bOnMMapList = IMG_FALSE;

    if(!DoMapToUser(psOffsetStruct->psLinuxMemArea, ps_vma, 0))
    {
        goto unlock_and_return;
    }

	PVR_ASSERT(psOffsetStruct->ui32UserVAddr == 0);

	psOffsetStruct->ui32UserVAddr = ps_vma->vm_start;

	omap_gem_set_priv(obj, psOffsetStruct->psLinuxMemArea);


    /* Compute the flush region (if necessary) inside the mmap mutex */
    if(psOffsetStruct->psLinuxMemArea->bNeedsCacheInvalidate)
    {
        IMG_UINT32 ui32DummyByteSize;

        DetermineUsersSizeAndByteOffset(psOffsetStruct->psLinuxMemArea,
                                        &ui32DummyByteSize,
                                        &ui32ByteOffset);

        pvBase = (IMG_VOID *)ps_vma->vm_start + ui32ByteOffset;
        psFlushMemArea = psOffsetStruct->psLinuxMemArea;

        psOffsetStruct->psLinuxMemArea->bNeedsCacheInvalidate = IMG_FALSE;
    }

    /* Call the open routine to increment the usage count */
	MMapVOpenNoLock(ps_vma, psOffsetStruct);

unlock_and_return:
	LinuxUnLockMutex(&g_sMMapMutex);

    if(psFlushMemArea)
    {
        OSInvalidateCPUCacheRangeKM(psFlushMemArea, ui32ByteOffset, pvBase,
									psFlushMemArea->ui32ByteSize);
    }
}

static struct omap_gem_vm_ops gem_ops = {
		.open  = MMapVOpenExt,
		.close = MMapVCloseExt,
		.mmap  = PVRMMapExt,
};
#endif

#if defined(DEBUG_LINUX_MMAP_AREAS)

/*
 * Lock MMap regions list (called on page start/stop while reading /proc/mmap)

 * sfile : seq_file that handles /proc file
 * start : TRUE if it's start, FALSE if it's stop
 *  
*/
static void ProcSeqStartstopMMapRegistations(struct seq_file *sfile,IMG_BOOL start) 
{
	if(start) 
	{
	    LinuxLockMutexNested(&g_sMMapMutex, PVRSRV_LOCK_CLASS_MMAP);		
	}
	else
	{
	    LinuxUnLockMutex(&g_sMMapMutex);
	}
}


/*
 * Convert offset (index from KVOffsetTable) to element 
 * (called when reading /proc/mmap file)

 * sfile : seq_file that handles /proc file
 * off : index into the KVOffsetTable from which to print
 *  
 * returns void* : Pointer to element that will be dumped
 *  
*/
static void* ProcSeqOff2ElementMMapRegistrations(struct seq_file *sfile, loff_t off)
{
    LinuxMemArea *psLinuxMemArea;
	if(!off) 
	{
		return PVR_PROC_SEQ_START_TOKEN;
	}

    list_for_each_entry(psLinuxMemArea, &g_sMMapAreaList, sMMapItem)
    {
        PKV_OFFSET_STRUCT psOffsetStruct;

	 	list_for_each_entry(psOffsetStruct, &psLinuxMemArea->sMMapOffsetStructList, sAreaItem)
        {
	    	off--;
	    	if (off == 0)
	    	{				
				PVR_ASSERT(psOffsetStruct->psLinuxMemArea == psLinuxMemArea);
				return (void*)psOffsetStruct;
		    }
        }
    }
	return (void*)0;
}

/*
 * Gets next MMap element to show. (called when reading /proc/mmap file)

 * sfile : seq_file that handles /proc file
 * el : actual element
 * off : index into the KVOffsetTable from which to print
 *  
 * returns void* : Pointer to element to show (0 ends iteration)
*/
static void* ProcSeqNextMMapRegistrations(struct seq_file *sfile,void* el,loff_t off)
{
	return ProcSeqOff2ElementMMapRegistrations(sfile,off);
}


/*
 * Show MMap element (called when reading /proc/mmap file)

 * sfile : seq_file that handles /proc file
 * el : actual element
 *  
*/
static void ProcSeqShowMMapRegistrations(struct seq_file *sfile, void *el)
{
	KV_OFFSET_STRUCT *psOffsetStruct = (KV_OFFSET_STRUCT*)el;
    LinuxMemArea *psLinuxMemArea;
	IMG_UINT32 ui32RealByteSize;
	IMG_UINT32 ui32ByteOffset;

	if(el == PVR_PROC_SEQ_START_TOKEN) 
	{
        seq_printf( sfile,
#if !defined(DEBUG_LINUX_XML_PROC_FILES)
						  "Allocations registered for mmap: %u\n"
                          "In total these areas correspond to %u bytes\n"
                          "psLinuxMemArea "
						  "UserVAddr "
						  "KernelVAddr "
						  "CpuPAddr "
                          "MMapOffset "
                          "ByteLength "
                          "LinuxMemType             "
						  "Pid   Name     Flags\n",
#else
                          "<mmap_header>\n"
                          "\t<count>%u</count>\n"
                          "\t<bytes>%u</bytes>\n"
                          "</mmap_header>\n",
#endif
						  g_ui32RegisteredAreas,
                          g_ui32TotalByteSize
                          );
		return;
	}

   	psLinuxMemArea = psOffsetStruct->psLinuxMemArea;

	DetermineUsersSizeAndByteOffset(psLinuxMemArea,
									&ui32RealByteSize,
									&ui32ByteOffset);

	seq_printf( sfile,
#if !defined(DEBUG_LINUX_XML_PROC_FILES)
						"%-8p       %08x %-8p %08x %08x   %-8d   %-24s %-5u %-8s %08x(%s)\n",
#else
                        "<mmap_record>\n"
						"\t<pointer>%-8p</pointer>\n"
                        "\t<user_virtual>%-8x</user_virtual>\n"
                        "\t<kernel_virtual>%-8p</kernel_virtual>\n"
                        "\t<cpu_physical>%08x</cpu_physical>\n"
                        "\t<mmap_offset>%08x</mmap_offset>\n"
                        "\t<bytes>%-8d</bytes>\n"
                        "\t<linux_mem_area_type>%-24s</linux_mem_area_type>\n"
                        "\t<pid>%-5u</pid>\n"
                        "\t<name>%-8s</name>\n"
                        "\t<flags>%08x</flags>\n"
                        "\t<flags_string>%s</flags_string>\n"
                        "</mmap_record>\n",
#endif
                        psLinuxMemArea,
						psOffsetStruct->ui32UserVAddr + ui32ByteOffset,
						LinuxMemAreaToCpuVAddr(psLinuxMemArea),
                        LinuxMemAreaToCpuPAddr(psLinuxMemArea,0).uiAddr,
						psOffsetStruct->ui32MMapOffset,
						psLinuxMemArea->ui32ByteSize,
                        LinuxMemAreaTypeToString(psLinuxMemArea->eAreaType),
						psOffsetStruct->ui32PID,
						psOffsetStruct->pszName,
						psLinuxMemArea->ui32AreaFlags,
                        HAPFlagsToString(psLinuxMemArea->ui32AreaFlags));
}

#endif


/*!
 *******************************************************************************

 @Function  PVRMMapRegisterArea

 @Description

 Register a memory area with the mmap code.

 @input psLinuxMemArea : pointer to memory area.

 @Return PVRSRV_OK, or PVRSRV_ERROR.

 ******************************************************************************/
PVRSRV_ERROR
PVRMMapRegisterArea(LinuxMemArea *psLinuxMemArea)
{
    PVRSRV_ERROR eError;
#if defined(DEBUG) || defined(DEBUG_LINUX_MMAP_AREAS)
    const IMG_CHAR *pszName = LinuxMemAreaTypeToString(LinuxMemAreaRootType(psLinuxMemArea));
    (void)pszName; /* Ignore fatal warning. */
#endif

    LinuxLockMutexNested(&g_sMMapMutex, PVRSRV_LOCK_CLASS_MMAP);

#if defined(DEBUG) || defined(DEBUG_LINUX_MMAP_AREAS)
    PVR_DPF((PVR_DBG_MESSAGE,
             "%s(%s, psLinuxMemArea 0x%p, ui32AllocFlags 0x%8x)",
             __FUNCTION__, pszName, psLinuxMemArea,  psLinuxMemArea->ui32AreaFlags));
#endif

    PVR_ASSERT(psLinuxMemArea->eAreaType != LINUX_MEM_AREA_SUB_ALLOC || LinuxMemAreaRoot(psLinuxMemArea)->eAreaType != LINUX_MEM_AREA_SUB_ALLOC);

    /* Check this mem area hasn't already been registered */
    if(psLinuxMemArea->bMMapRegistered)
    {
        PVR_DPF((PVR_DBG_ERROR, "%s: psLinuxMemArea 0x%p is already registered",
                __FUNCTION__, psLinuxMemArea));
        eError = PVRSRV_ERROR_INVALID_PARAMS;
	goto exit_unlock;
    }

    list_add_tail(&psLinuxMemArea->sMMapItem, &g_sMMapAreaList);

    psLinuxMemArea->bMMapRegistered = IMG_TRUE;

#if defined(DEBUG_LINUX_MMAP_AREAS)
    g_ui32RegisteredAreas++;
    /*
     * Sub memory areas are excluded from g_ui32TotalByteSize so that we
     * don't count memory twice, once for the parent and again for sub
     * allocationis.
     */
    if (psLinuxMemArea->eAreaType != LINUX_MEM_AREA_SUB_ALLOC)
    {
        g_ui32TotalByteSize += psLinuxMemArea->ui32ByteSize;
    }
#endif

    eError = PVRSRV_OK;

exit_unlock:
    LinuxUnLockMutex(&g_sMMapMutex);

    return eError;
}


/*!
 *******************************************************************************

 @Function  PVRMMapRemoveRegisterArea

 @Description

 Unregister a memory area with the mmap code.

 @input psLinuxMemArea : pointer to memory area.

 @Return PVRSRV_OK, or PVRSRV_ERROR.

 ******************************************************************************/
PVRSRV_ERROR
PVRMMapRemoveRegisteredArea(LinuxMemArea *psLinuxMemArea)
{
    PVRSRV_ERROR eError;
    PKV_OFFSET_STRUCT psOffsetStruct, psTmpOffsetStruct;

    LinuxLockMutexNested(&g_sMMapMutex, PVRSRV_LOCK_CLASS_MMAP);

    PVR_ASSERT(psLinuxMemArea->bMMapRegistered);

    list_for_each_entry_safe(psOffsetStruct, psTmpOffsetStruct, &psLinuxMemArea->sMMapOffsetStructList, sAreaItem)
    {
	if (psOffsetStruct->ui32Mapped != 0)
	{
	     PVR_DPF((PVR_DBG_ERROR, "%s: psOffsetStruct 0x%p for memory area 0x0x%p is still mapped; psOffsetStruct->ui32Mapped %u",  __FUNCTION__, psOffsetStruct, psLinuxMemArea, psOffsetStruct->ui32Mapped));
		dump_stack();
		PVRSRVDumpRefCountCCB();
		eError = PVRSRV_ERROR_STILL_MAPPED;
		goto exit_unlock;
	}
	else
	{
	      /*
	      * An offset structure is created when a call is made to get
	      * the mmap data for a physical mapping.  If the data is never
	      * used for mmap, we will be left with an umapped offset
	      * structure.
	      */
	     PVR_DPF((PVR_DBG_WARNING, "%s: psOffsetStruct 0x%p was never mapped",  __FUNCTION__, psOffsetStruct));
	}

	PVR_ASSERT((psOffsetStruct->ui32Mapped == 0) && psOffsetStruct->bOnMMapList);

	DestroyOffsetStruct(psOffsetStruct);
    }

    list_del(&psLinuxMemArea->sMMapItem);

    psLinuxMemArea->bMMapRegistered = IMG_FALSE;

#if defined(DEBUG_LINUX_MMAP_AREAS)
    g_ui32RegisteredAreas--;
    if (psLinuxMemArea->eAreaType != LINUX_MEM_AREA_SUB_ALLOC)
    {
        g_ui32TotalByteSize -= psLinuxMemArea->ui32ByteSize;
    }
#endif

    eError = PVRSRV_OK;

exit_unlock:
    LinuxUnLockMutex(&g_sMMapMutex);
    return eError;
}


/*!
 *******************************************************************************

 @Function  LinuxMMapPerProcessConnect

 @Description

 Per-process mmap initialisation code.

 @input psEnvPerProc : pointer to OS specific per-process data.

 @Return PVRSRV_OK, or PVRSRV_ERROR.

 ******************************************************************************/
PVRSRV_ERROR
LinuxMMapPerProcessConnect(PVRSRV_ENV_PER_PROCESS_DATA *psEnvPerProc)
{
    PVR_UNREFERENCED_PARAMETER(psEnvPerProc);

    return PVRSRV_OK;
}

/*!
 *******************************************************************************

 @Function  LinuxMMapPerProcessDisconnect

 @Description

 Per-process mmap deinitialisation code.

 @input psEnvPerProc : pointer to OS specific per-process data.

 ******************************************************************************/
IMG_VOID
LinuxMMapPerProcessDisconnect(PVRSRV_ENV_PER_PROCESS_DATA *psEnvPerProc)
{
    PKV_OFFSET_STRUCT psOffsetStruct, psTmpOffsetStruct;
    IMG_BOOL bWarn = IMG_FALSE;
    IMG_UINT32 ui32PID = OSGetCurrentProcessIDKM();

    PVR_UNREFERENCED_PARAMETER(psEnvPerProc);

    LinuxLockMutexNested(&g_sMMapMutex, PVRSRV_LOCK_CLASS_MMAP);

    list_for_each_entry_safe(psOffsetStruct, psTmpOffsetStruct, &g_sMMapOffsetStructList, sMMapItem)
    {
	if (psOffsetStruct->ui32PID == ui32PID)
	{
	    if (!bWarn)
	    {
		PVR_DPF((PVR_DBG_WARNING, "%s: process has unmapped offset structures. Removing them", __FUNCTION__));
		bWarn = IMG_TRUE;
	    }
	    PVR_ASSERT(psOffsetStruct->ui32Mapped == 0);
	    PVR_ASSERT(psOffsetStruct->bOnMMapList);

	    DestroyOffsetStruct(psOffsetStruct);
	}
    }

    LinuxUnLockMutex(&g_sMMapMutex);
}


/*!
 *******************************************************************************

 @Function  LinuxMMapPerProcessHandleOptions

 @Description

 Set secure handle options required by mmap code.

 @input psHandleBase : pointer to handle base.

 @Return PVRSRV_OK, or PVRSRV_ERROR.

 ******************************************************************************/
PVRSRV_ERROR LinuxMMapPerProcessHandleOptions(PVRSRV_HANDLE_BASE *psHandleBase)
{
    PVRSRV_ERROR eError;

    eError = PVRSRVSetMaxHandle(psHandleBase, MAX_MMAP_HANDLE);
    if (eError != PVRSRV_OK)
    {
	PVR_DPF((PVR_DBG_ERROR,"%s: failed to set handle limit (%d)", __FUNCTION__, eError));
	return eError;
    }

    return eError;
}


/*!
 *******************************************************************************

 @Function  PVRMMapInit

 @Description

 MMap initialisation code

 ******************************************************************************/
IMG_VOID
PVRMMapInit(IMG_VOID)
{
    LinuxInitMutex(&g_sMMapMutex);

    g_psMemmapCache = KMemCacheCreateWrapper("img-mmap", sizeof(KV_OFFSET_STRUCT), 0, 0);
    if (!g_psMemmapCache)
    {
        PVR_DPF((PVR_DBG_ERROR,"%s: failed to allocate kmem_cache", __FUNCTION__));
	goto error;
    }

#if defined(DEBUG_LINUX_MMAP_AREAS)
	g_ProcMMap = CreateProcReadEntrySeq("mmap", NULL, 
						  ProcSeqNextMMapRegistrations,
						  ProcSeqShowMMapRegistrations,
						  ProcSeqOff2ElementMMapRegistrations,
						  ProcSeqStartstopMMapRegistations
						 );
#endif  /* defined(DEBUG_LINUX_MMAP_AREAS) */
    return;

error:
    PVRMMapCleanup();
    return;
}


/*!
 *******************************************************************************

 @Function  PVRMMapCleanup

 @Description

 Mmap deinitialisation code

 ******************************************************************************/
IMG_VOID
PVRMMapCleanup(IMG_VOID)
{
    PVRSRV_ERROR eError;

    if (!list_empty(&g_sMMapAreaList))
    {
	LinuxMemArea *psLinuxMemArea, *psTmpMemArea;

	PVR_DPF((PVR_DBG_ERROR, "%s: Memory areas are still registered with MMap", __FUNCTION__));
	
	PVR_TRACE(("%s: Unregistering memory areas", __FUNCTION__));
 	list_for_each_entry_safe(psLinuxMemArea, psTmpMemArea, &g_sMMapAreaList, sMMapItem)
	{
		eError = PVRMMapRemoveRegisteredArea(psLinuxMemArea);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: PVRMMapRemoveRegisteredArea failed (%d)", __FUNCTION__, eError));
		}
		PVR_ASSERT(eError == PVRSRV_OK);

		LinuxMemAreaDeepFree(psLinuxMemArea);
	}
    }
    PVR_ASSERT(list_empty((&g_sMMapAreaList)));

#if defined(DEBUG_LINUX_MMAP_AREAS)
    RemoveProcEntrySeq(g_ProcMMap);
#endif /* defined(DEBUG_LINUX_MMAP_AREAS) */

    if(g_psMemmapCache)
    {
        KMemCacheDestroyWrapper(g_psMemmapCache);
        g_psMemmapCache = NULL;
    }
}
