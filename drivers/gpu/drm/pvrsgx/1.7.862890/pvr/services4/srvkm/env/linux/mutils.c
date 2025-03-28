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

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38))
#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#endif

#include <linux/spinlock.h>
#include <linux/mm.h>
#include <asm/page.h>
#include <asm/pgtable.h>

#include "img_defs.h"
#include "pvr_debug.h"
#include "mutils.h"

#if defined(SUPPORT_LINUX_X86_PAT)
#define	PAT_LINUX_X86_WC	1

#define	PAT_X86_ENTRY_BITS	8

#define	PAT_X86_BIT_PWT		1U
#define	PAT_X86_BIT_PCD		2U
#define	PAT_X86_BIT_PAT		4U
#define	PAT_X86_BIT_MASK	(PAT_X86_BIT_PAT | PAT_X86_BIT_PCD | PAT_X86_BIT_PWT)

static IMG_BOOL g_write_combining_available = IMG_FALSE;

#define	PROT_TO_PAT_INDEX(v, B) ((v & _PAGE_ ## B) ? PAT_X86_BIT_ ## B : 0)

static inline IMG_UINT
pvr_pat_index(pgprotval_t prot_val)
{
	IMG_UINT ret = 0;
	pgprotval_t val = prot_val & _PAGE_CACHE_MASK;

	ret |= PROT_TO_PAT_INDEX(val, PAT);
	ret |= PROT_TO_PAT_INDEX(val, PCD);
	ret |= PROT_TO_PAT_INDEX(val, PWT);

	return ret;
}

static inline IMG_UINT
pvr_pat_entry(u64 pat, IMG_UINT index)
{
	return (IMG_UINT)(pat >> (index * PAT_X86_ENTRY_BITS)) & PAT_X86_BIT_MASK;
}

static IMG_VOID
PVRLinuxX86PATProbe(IMG_VOID)
{
        /*
	 * cpu_has_pat indicates whether PAT support is available on the CPU,
	 * but doesn't indicate if it has been enabled.
	 */
	if (boot_cpu_has(X86_FEATURE_PAT))/* PRQA S 3335 */ /* ignore 'no function declared' */
	{
		u64 pat;
		IMG_UINT pat_index;
		IMG_UINT pat_entry;

		PVR_TRACE(("%s: PAT available", __FUNCTION__));
		/*
		 * There is no Linux API for finding out if write combining
		 * is avaialable through the PAT, so we take the direct
		 * approach, and see if the PAT MSR contains a write combining
		 * entry.
		 */
		rdmsrl(MSR_IA32_CR_PAT, pat);
		PVR_TRACE(("%s: Top 32 bits of PAT: 0x%.8x", __FUNCTION__, (IMG_UINT)(pat >> 32)));
		PVR_TRACE(("%s: Bottom 32 bits of PAT: 0x%.8x", __FUNCTION__, (IMG_UINT)(pat)));

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0))
                pat_index = pvr_pat_index(_PAGE_CACHE_MODE_WC);
#else
		pat_index = pvr_pat_index(_PAGE_CACHE_WC);
#endif
		PVR_TRACE(("%s: PAT index for write combining: %u", __FUNCTION__, pat_index));

		pat_entry = pvr_pat_entry(pat, pat_index);
		PVR_TRACE(("%s: PAT entry for write combining: 0x%.2x (should be 0x%.2x)", __FUNCTION__, pat_entry, PAT_LINUX_X86_WC));

#if defined(SUPPORT_LINUX_X86_WRITECOMBINE)
		g_write_combining_available = (IMG_BOOL)(pat_entry == PAT_LINUX_X86_WC);
#endif
	}
#if defined(DEBUG)
#if defined(SUPPORT_LINUX_X86_WRITECOMBINE)
	if (g_write_combining_available)
	{
		PVR_TRACE(("%s: Write combining available via PAT", __FUNCTION__));
	}
	else
	{
		PVR_TRACE(("%s: Write combining not available", __FUNCTION__));
	}
#else	
	PVR_TRACE(("%s: Write combining disabled in driver build", __FUNCTION__));
#endif	
#endif	
}

pgprot_t
pvr_pgprot_writecombine(pgprot_t prot)
{
    /*
     * It would be worth checking from time to time to see if a
     * pgprot_writecombine function (or similar) is introduced on Linux for
     * x86 processors.  If so, this function, and PVRLinuxX86PATProbe can be
     * removed, and a macro used to select between pgprot_writecombine and
     * pgprot_noncached, dpending on the value for of
     * SUPPORT_LINUX_X86_WRITECOMBINE.
     */
    /* PRQA S 0481,0482 2 */ /* scalar expressions */
    return (g_write_combining_available) ?
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0))
		__pgprot((pgprot_val(prot) & ~_PAGE_CACHE_MASK) | _PAGE_CACHE_MODE_WC) : pgprot_noncached(prot);
#else
		__pgprot((pgprot_val(prot) & ~_PAGE_CACHE_MASK) | _PAGE_CACHE_WC) : pgprot_noncached(prot);
#endif
}
#endif	/* defined(SUPPORT_LINUX_X86_PAT) */

IMG_VOID
PVRLinuxMUtilsInit(IMG_VOID)
{
#if defined(SUPPORT_LINUX_X86_PAT)
	PVRLinuxX86PATProbe();
#endif
}

