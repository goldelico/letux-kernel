/*
 * ducatienabler.c
 *
 * Syslink driver support for TI OMAP processors.
 *
 * Copyright (C) 2008-2009 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */



#include <linux/types.h>
#include <linux/mm.h>
#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/module.h>
#include <asm/page.h>
#include <linux/kernel.h>
#include <linux/pagemap.h>


#include <linux/autoconf.h>
#include <asm/system.h>
#include <asm/atomic.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <linux/syscalls.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/stddef.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/vmalloc.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/pagemap.h>
#include <asm/cacheflush.h>
#include <linux/dma-mapping.h>
#include <mach/iommu.h>
#include "../../../arch/arm/plat-omap/iopgtable.h"
#include <syslink/ducatienabler.h>


#ifdef DEBUG_DUCATI_IPC
#define DPRINTK(fmt, args...) printk(KERN_INFO "%s: " fmt, __func__, ## args)
#else
#define DPRINTK(fmt, args...)
#endif

/*
 * Macro to define the physical memory address for the
 * Ducati Base image. The 74Mb memory is preallocated
 * during the make menuconfig.
 *
 */
#define DUCATI_BASEIMAGE_PHYSICAL_ADDRESS	0x87200000

#define phys_to_page(phys)      pfn_to_page((phys) >> PAGE_SHIFT)



enum pagetype {
	SECTION = 0,
	LARGE_PAGE = 1,
	SMALL_PAGE = 2,
	SUPER_SECTION  = 3
};


static u32 shm_phys_addr;
static u32 shm_virt_addr;

struct iommu *ducati_iommu_ptr;

static void bad_page_dump(u32 pa, struct page *pg)
{
	pr_emerg("DSPBRIDGE: MAP function: COUNT 0 FOR PA 0x%x\n", pa);
	pr_emerg("Bad page state in process '%s'\n", current->comm);
	BUG();
}

/*============================================
 * This function calculates PTE address (MPU virtual) to be updated
 *  It also manages the L2 page tables
 */
static int pte_set(u32 pa, u32 va, u32 size, struct hw_mmu_map_attrs_t *attrs)
{
	struct iotlb_entry tlb_entry;
	switch (size) {
	case HW_PAGE_SIZE_16MB:
		tlb_entry.pgsz = MMU_CAM_PGSZ_16M;
		break;
	case HW_PAGE_SIZE_1MB:
		tlb_entry.pgsz = MMU_CAM_PGSZ_1M;
		break;
	case HW_PAGE_SIZE_64KB:
		tlb_entry.pgsz = MMU_CAM_PGSZ_64K;
		break;
	case HW_PAGE_SIZE_4KB:
		tlb_entry.pgsz = MMU_CAM_PGSZ_4K;
		break;
	}
	tlb_entry.prsvd = MMU_CAM_P;
	tlb_entry.valid = MMU_CAM_V;
	switch (attrs->element_size) {
	case HW_ELEM_SIZE_8BIT:
		tlb_entry.elsz = MMU_RAM_ELSZ_8;
		break;
	case HW_ELEM_SIZE_16BIT:
		tlb_entry.elsz = MMU_RAM_ELSZ_16;
		break;
	case HW_ELEM_SIZE_32BIT:
		tlb_entry.elsz = MMU_RAM_ELSZ_32;
		break;
	case HW_ELEM_SIZE_64BIT:
		tlb_entry.elsz = 0x3; /* No translation */
		break;
	}
	switch (attrs->endianism) {
	case HW_LITTLE_ENDIAN:
		tlb_entry.endian = MMU_RAM_ENDIAN_LITTLE;
		break;
	case HW_BIG_ENDIAN:
		tlb_entry.endian = MMU_RAM_ENDIAN_BIG;
		break;
	}
	switch (attrs->mixedSize) {
	case HW_MMU_TLBES:
		tlb_entry.mixed = 0;
		break;
	case HW_MMU_CPUES:
		tlb_entry.mixed = MMU_RAM_MIXED;
		break;
	}
	tlb_entry.da = va;
	tlb_entry.pa = pa;
	DPRINTK("pte set ducati_iommu_ptr = 0x%x, tlb_entry = 0x%x \n",
					ducati_iommu_ptr, tlb_entry);
	if (iopgtable_store_entry(ducati_iommu_ptr, &tlb_entry))
		goto error_exit;
	return 0;
error_exit:
	printk(KERN_ERR "pte set failure \n");
	return -EFAULT;
}


/*=============================================
 * This function calculates the optimum page-aligned addresses and sizes
 * Caller must pass page-aligned values
 */
static int pte_update(u32 pa, u32 va, u32 size,
			struct hw_mmu_map_attrs_t *map_attrs)
{
	u32 i;
	u32 all_bits;
	u32 pa_curr = pa;
	u32 va_curr = va;
	u32 num_bytes = size;
	int status = 0;
	u32 pg_size[] = {HW_PAGE_SIZE_16MB, HW_PAGE_SIZE_1MB,
			   HW_PAGE_SIZE_64KB, HW_PAGE_SIZE_4KB};
	DPRINTK("> pte_update  pa %x, va %x, "
		 "size %x, map_attrs %x\n", pa, va, size, (u32)map_attrs);
	while (num_bytes && (status == 0)) {
		/* To find the max. page size with which both PA & VA are
		 * aligned */
		all_bits = pa_curr | va_curr;
		DPRINTK("all_bits %x, pa_curr %x, va_curr %x, "
			 "num_bytes %x\n ",
			all_bits, pa_curr, va_curr, num_bytes);

		for (i = 0; i < 4; i++) {
			if ((num_bytes >= pg_size[i]) && ((all_bits &
			   (pg_size[i] - 1)) == 0)) {
				DPRINTK("pg_size %x\n", pg_size[i]);
				status = pte_set(pa_curr,
					va_curr, pg_size[i], map_attrs);
				pa_curr += pg_size[i];
				va_curr += pg_size[i];
				num_bytes -= pg_size[i];
				 /* Don't try smaller sizes. Hopefully we have
				 * reached an address aligned to a bigger page
				 * size */
				break;
			}
		}
	}
	DPRINTK("< pte_update status %x num_bytes %x\n", status, num_bytes);
	return status;
}

/*
 *  ======== ducati_mem_unmap ========
 *      Invalidate the PTEs for the DSP VA block to be unmapped.
 *
 *      PTEs of a mapped memory block are contiguous in any page table
 *      So, instead of looking up the PTE address for every 4K block,
 *      we clear consecutive PTEs until we unmap all the bytes
 */
int ducati_mem_unmap(u32 da, u32 num_bytes)
{
	u32 bytes;
	struct page *pg = NULL;
	int temp = 0;
	u32 nent;
	u32 phyaddress;
	s32 numofBytes = num_bytes;

	 while (num_bytes > 0) {
		u32 *iopgd = iopgd_offset(ducati_iommu_ptr, da);
		if (*iopgd & IOPGD_TABLE) {
			u32 *iopte = iopte_offset(iopgd, da);
			if (*iopte & IOPTE_LARGE) {
				nent = 16;
				/* rewind to the 1st entry */
				phyaddress = *iopte & IOLARGE_MASK;
			} else {
				nent = 1;
				phyaddress = (*iopte) & IOPAGE_MASK;
			}
		} else {
			if ((*iopgd & IOPGD_SUPER) == IOPGD_SUPER) {
				nent = 4096;
				/* rewind to the 1st entry */
				iopgd = (u32 *)((u32)iopgd & IOSUPER_MASK);
			} else
				nent = 256;
			phyaddress = (*iopgd) & IOPGD_MASK;
		}
		for (temp = 0; temp < nent; temp++) {
			if (pfn_valid(__phys_to_pfn(phyaddress))) {
				pg = phys_to_page(phyaddress);
				if (page_count(pg) < 1) {
					pr_info("DSPBRIDGE:UNMAP function: "
						"COUNT 0 FOR PA 0x%x,"
						" size = 0x%x\n",
						phyaddress, numofBytes);
					bad_page_dump(phyaddress, pg);
				}
				SetPageDirty(pg);
				page_cache_release(pg);
			}
			phyaddress += HW_PAGE_SIZE_4KB;
		}
		bytes = iopgtable_clear_entry(ducati_iommu_ptr, da);
		num_bytes -= bytes;
		da += bytes;
	}
	return 0;


}

/*
 *  ======== ducati_mem_virtToPhys ========
 *  This funciton provides the translation from
 *  Remote virtual address to Physical address
 */

inline u32 ducati_mem_virtToPhys(u32 da)
{
	u32 *iopgd = iopgd_offset(ducati_iommu_ptr, da);
	u32 phyaddress;

	if (*iopgd & IOPGD_TABLE) {
		u32 *iopte = iopte_offset(iopgd, da);
		if (*iopte & IOPTE_LARGE) {
			phyaddress = *iopte & IOLARGE_MASK;
			phyaddress |= (da & (IOLARGE_SIZE - 1));
		} else
			phyaddress = (*iopte) & IOPAGE_MASK;
	} else {
		if ((*iopgd & IOPGD_SUPER) == IOPGD_SUPER) {
			phyaddress = *iopgd & IOSUPER_MASK;
			phyaddress |= (da & (IOSUPER_SIZE - 1));
		} else {
			phyaddress = (*iopgd) & IOPGD_MASK;
			phyaddress |= (da & (IOPGD_SIZE - 1));
		}
	}
	return phyaddress;
}


/*
 *  ======== user_va2pa ========
 *  Purpose:
 *      This function walks through the Linux page tables to convert a userland
 *      virtual address to physical address
 */
u32 user_va2pa(struct mm_struct *mm, u32 address)
{
	pgd_t *pgd;
	pmd_t *pmd;
	pte_t *ptep, pte;

	pgd = pgd_offset(mm, address);
	if (!(pgd_none(*pgd) || pgd_bad(*pgd))) {
		pmd = pmd_offset(pgd, address);
		if (!(pmd_none(*pmd) || pmd_bad(*pmd))) {
			ptep = pte_offset_map(pmd, address);
			if (ptep) {
				pte = *ptep;
				if (pte_present(pte))
					return pte & PAGE_MASK;
			}
		}
	}

	return 0;
}
/*============================================
 * This function maps MPU buffer to the DSP address space. It performs
* linear to physical address translation if required. It translates each
* page since linear addresses can be physically non-contiguous
* All address & size arguments are assumed to be page aligned (in proc.c)
 *
 */
int ducati_mem_map(u32 mpu_addr, u32 ul_virt_addr,
				u32 num_bytes, u32 map_attr)
{
	u32 attrs;
	int status = 0;
	struct hw_mmu_map_attrs_t hw_attrs;
	struct vm_area_struct *vma;
	struct mm_struct *mm = current->mm;
	struct task_struct *curr_task = current;
	u32 write = 0;
	u32 da = ul_virt_addr;
	u32 pa = 0;
	int pg_i = 0;
	int pg_num = 0;
	struct page *mappedPage, *pg;
	int num_usr_pages = 0;

	DPRINTK("> WMD_BRD_MemMap  pa %x, va %x, "
		 "size %x, map_attr %x\n", mpu_addr, ul_virt_addr,
		 num_bytes, map_attr);
	if (num_bytes == 0)
		return -EINVAL;
	if (map_attr != 0) {
		attrs = map_attr;
		attrs |= DSP_MAPELEMSIZE32;
	} else {
		/* Assign default attributes */
		attrs = DSP_MAPVIRTUALADDR | DSP_MAPELEMSIZE32;
	}
	/* Take mapping properties */
	if (attrs & DSP_MAPBIGENDIAN)
		hw_attrs.endianism = HW_BIG_ENDIAN;
	else
		hw_attrs.endianism = HW_LITTLE_ENDIAN;

	hw_attrs.mixedSize = (enum hw_mmu_mixed_size_t)
			     ((attrs & DSP_MAPMIXEDELEMSIZE) >> 2);
	/* Ignore element_size if mixedSize is enabled */
	if (hw_attrs.mixedSize == 0) {
		if (attrs & DSP_MAPELEMSIZE8) {
			/* Size is 8 bit */
			hw_attrs.element_size = HW_ELEM_SIZE_8BIT;
		} else if (attrs & DSP_MAPELEMSIZE16) {
			/* Size is 16 bit */
			hw_attrs.element_size = HW_ELEM_SIZE_16BIT;
		} else if (attrs & DSP_MAPELEMSIZE32) {
			/* Size is 32 bit */
			hw_attrs.element_size = HW_ELEM_SIZE_32BIT;
		} else if (attrs & DSP_MAPELEMSIZE64) {
			/* Size is 64 bit */
			hw_attrs.element_size = HW_ELEM_SIZE_64BIT;
		} else {
			/* Mixedsize isn't enabled, so size can't be
			 * zero here */
			DPRINTK("WMD_BRD_MemMap: MMU element size is zero\n");
			return -EINVAL;
		}
	}
	/*
	 * Do OS-specific user-va to pa translation.
	 * Combine physically contiguous regions to reduce TLBs.
	 * Pass the translated pa to PteUpdate.
	 */
	if ((attrs & DSP_MAPPHYSICALADDR)) {
		status = pte_update(mpu_addr, ul_virt_addr, num_bytes,
								&hw_attrs);
		goto func_cont;
	}
	/*
	 * Important Note: mpu_addr is mapped from user application process
	 * to current process - it must lie completely within the current
	 * virtual memory address space in order to be of use to us here!
	 */
	down_read(&mm->mmap_sem);
	vma = find_vma(mm, mpu_addr);
	/*
	 * It is observed that under some circumstances, the user buffer is
	 * spread across several VMAs. So loop through and check if the entire
	 * user buffer is covered
	 */
	while ((vma) && (mpu_addr + num_bytes > vma->vm_end)) {
		/* jump to the next VMA region */
		vma = find_vma(mm, vma->vm_end + 1);
	}
	if (!vma) {
		status = -EINVAL;
		up_read(&mm->mmap_sem);
		goto func_cont;
	}
	if (vma->vm_flags & VM_IO) {
		num_usr_pages = num_bytes / PAGE_SIZE;
		/* Get the physical addresses for user buffer */
		for (pg_i = 0; pg_i < num_usr_pages; pg_i++) {
			pa = user_va2pa(mm, mpu_addr);
			if (!pa) {
				status = -EFAULT;
				pr_err("DSPBRIDGE: VM_IO mapping physical"
						"address is invalid\n");
				break;
			}
			if (pfn_valid(__phys_to_pfn(pa))) {
				pg = phys_to_page(pa);
				get_page(pg);
				if (page_count(pg) < 1) {
					pr_err("Bad page in VM_IO buffer\n");
					bad_page_dump(pa, pg);
				}
			}
			status = pte_set(pa, da, HW_PAGE_SIZE_4KB, &hw_attrs);
			if (WARN_ON(status < 0))
				break;
			mpu_addr += HW_PAGE_SIZE_4KB;
			da += HW_PAGE_SIZE_4KB;
		}
	} else {
		num_usr_pages =  num_bytes / PAGE_SIZE;
		if (vma->vm_flags & (VM_WRITE | VM_MAYWRITE))
			write = 1;

		for (pg_i = 0; pg_i < num_usr_pages; pg_i++) {
			pg_num = get_user_pages(curr_task, mm, mpu_addr, 1,
						write, 1, &mappedPage, NULL);
			if (pg_num > 0) {
				if (page_count(mappedPage) < 1) {
					pr_err("Bad page count after doing"
							"get_user_pages on"
							"user buffer\n");
					bad_page_dump(page_to_phys(mappedPage),
								mappedPage);
				}
				status = pte_set(page_to_phys(mappedPage), da,
					HW_PAGE_SIZE_4KB, &hw_attrs);
				if (WARN_ON(status < 0))
					break;
				da += HW_PAGE_SIZE_4KB;
				mpu_addr += HW_PAGE_SIZE_4KB;
			} else {
				pr_err("DSPBRIDGE: get_user_pages FAILED,"
						"MPU addr = 0x%x,"
						"vma->vm_flags = 0x%lx,"
						"get_user_pages Err"
						"Value = %d, Buffer"
						"size=0x%x\n", mpu_addr,
						vma->vm_flags, pg_num,
						num_bytes);
				status = -EFAULT;
				break;
			}
		}
	}
	up_read(&mm->mmap_sem);
func_cont:
	/* Don't propogate Linux or HW status to upper layers */
	if (status < 0) {
		/*
		 * Roll out the mapped pages incase it failed in middle of
		 * mapping
		 */
		if (pg_i)
			ducati_mem_unmap(ul_virt_addr, (pg_i * PAGE_SIZE));
	}
	WARN_ON(status < 0);
	DPRINTK("< WMD_BRD_MemMap status %x\n", status);
	return status;
}

 /*=========================================
 * Decides a TLB entry size
 *
 */
static int get_mmu_entry_size(u32  phys_addr, u32 size, enum pagetype *size_tlb,
								u32 *entry_size)
{
	int status = 0;
	bool  page_align_4kb  = false;
	bool  page_align_64kb = false;
	bool  page_align_1mb = false;
	bool  page_align_16mb = false;

	/*  First check the page alignment*/
	if ((phys_addr % PAGE_SIZE_4KB)  == 0)
		page_align_4kb  = true;
	if ((phys_addr % PAGE_SIZE_64KB) == 0)
		page_align_64kb = true;
	if ((phys_addr % PAGE_SIZE_1MB)  == 0)
		page_align_1mb  = true;
	if ((phys_addr % PAGE_SIZE_16MB)  == 0)
		page_align_16mb  = true;

	if ((!page_align_64kb) && (!page_align_1mb)  && (!page_align_4kb)) {
		status = -EINVAL;
		goto error_exit;
	}
	if (status == 0) {
		/*  Now decide the entry size */
		if (size >= PAGE_SIZE_16MB) {
			if (page_align_16mb) {
				*size_tlb   = SUPER_SECTION;
				*entry_size = PAGE_SIZE_16MB;
			} else if (page_align_1mb) {
				*size_tlb   = SECTION;
				*entry_size = PAGE_SIZE_1MB;
			} else if (page_align_64kb) {
				*size_tlb   = LARGE_PAGE;
				*entry_size = PAGE_SIZE_64KB;
			} else if (page_align_4kb) {
				*size_tlb   = SMALL_PAGE;
				*entry_size = PAGE_SIZE_4KB;
			} else {
				status = -EINVAL;
				goto error_exit;
			}
		} else if (size >= PAGE_SIZE_1MB && size < PAGE_SIZE_16MB) {
			if (page_align_1mb) {
				*size_tlb   = SECTION;
				*entry_size = PAGE_SIZE_1MB;
			} else if (page_align_64kb) {
				*size_tlb   = LARGE_PAGE;
				*entry_size = PAGE_SIZE_64KB;
			} else if (page_align_4kb) {
				*size_tlb   = SMALL_PAGE;
				*entry_size = PAGE_SIZE_4KB;
			} else {
				status = -EINVAL;
				goto error_exit;
			}
		} else if (size > PAGE_SIZE_4KB &&
				size < PAGE_SIZE_1MB) {
			if (page_align_64kb) {
				*size_tlb   = LARGE_PAGE;
				*entry_size = PAGE_SIZE_64KB;
			} else if (page_align_4kb) {
				*size_tlb   = SMALL_PAGE;
				*entry_size = PAGE_SIZE_4KB;
			} else {
				status = -EINVAL;
				goto error_exit;
			}
		} else if (size == PAGE_SIZE_4KB) {
				if (page_align_4kb) {
					*size_tlb   = SMALL_PAGE;
					*entry_size = PAGE_SIZE_4KB;
				} else {
					status = -EINVAL;
					goto error_exit;
				}
			} else {
				status = -EINVAL;
				goto error_exit;
			}
	}
	DPRINTK("< GetMMUEntrySize status %x\n", status);
	return 0;
error_exit:
	DPRINTK("< GetMMUEntrySize FAILED !!!!!!\n");
	return status;
}

/*=========================================
 * Add DSP MMU entries corresponding to given MPU-Physical address
 * and DSP-virtual address
 */
static int add_dsp_mmu_entry(u32  *phys_addr, u32 *dsp_addr,
						u32 size)
{
	u32 mapped_size = 0;
	enum pagetype size_tlb = SECTION;
	u32 entry_size = 0;
	int status = 0;
	u32 page_size   = HW_PAGE_SIZE_1MB;
	u32 flags = 0;

	flags = (DSP_MAPELEMSIZE32 | DSP_MAPLITTLEENDIAN |
					DSP_MAPPHYSICALADDR);

	DPRINTK("Entered add_dsp_mmu_entry phys_addr = "
		 "0x%x, dsp_addr = 0x%x,size = 0x%x\n",
		*phys_addr, *dsp_addr, size);

	while ((mapped_size < size) && (status == 0)) {
		status = get_mmu_entry_size(*phys_addr,
			(size - mapped_size), &size_tlb, &entry_size);

		if (size_tlb == SECTION)
			page_size = HW_PAGE_SIZE_1MB;

		else if (size_tlb == LARGE_PAGE)
			page_size = HW_PAGE_SIZE_64KB;

		else if (size_tlb == SMALL_PAGE)
			page_size = HW_PAGE_SIZE_4KB;
		if (status == 0) {
			ducati_mem_map(*phys_addr,
			*dsp_addr, page_size, flags);
			mapped_size  += entry_size;
			*phys_addr   += entry_size;
			*dsp_addr   += entry_size;
		}
	}
	DPRINTK("Exited add_dsp_mmu_entryphys_addr = "
		"0x%x, dsp_addr = 0x%x\n",
		*phys_addr, *dsp_addr);
	return status;
 }


/*=============================================
 * Add DSP MMU entries corresponding to given MPU-Physical address
 * and DSP-virtual address
 *
 */
static int add_entry_ext(u32 *phys_addr, u32 *dsp_addr,
					u32 size)
{
	u32 mapped_size = 0;
	enum pagetype     size_tlb = SECTION;
	u32 entry_size = 0;
	int status = 0;
	u32 page_size = HW_PAGE_SIZE_1MB;
	u32 flags = 0;

	flags = (DSP_MAPELEMSIZE32 | DSP_MAPLITTLEENDIAN |
					DSP_MAPPHYSICALADDR);
	while ((mapped_size < size) && (status == 0)) {

		/*  get_mmu_entry_size fills the size_tlb and entry_size
		based on alignment and size of memory to map
		to DSP - size */
		status = get_mmu_entry_size(*phys_addr,
				(size - mapped_size),
				&size_tlb,
				&entry_size);

		if (size_tlb == SUPER_SECTION)
			page_size = HW_PAGE_SIZE_16MB;
		else if (size_tlb == SECTION)
			page_size = HW_PAGE_SIZE_1MB;
		else if (size_tlb == LARGE_PAGE)
			page_size = HW_PAGE_SIZE_64KB;
		else if (size_tlb == SMALL_PAGE)
			page_size = HW_PAGE_SIZE_4KB;

		if (status == 0) {

			ducati_mem_map(*phys_addr,
			*dsp_addr, page_size, flags);
			mapped_size  += entry_size;
			*phys_addr   += entry_size;
			*dsp_addr   += entry_size;
		}
	}
	return status;
}

/*================================
 * Initialize the Ducati MMU.
 */
int  ducati_mmu_init(u32 a_phy_addr)
{
	int ret_val = 0;
	u32 phys_addr = 0;
	u32 num_l4_entries;
	u32 i = 0;
	u32 map_attrs;
	u32 ducati_boot_addr = 0;
	u32 num_l3_mem_entries = 0;
	u32 tiler_mapbeg = 0;
	u32 tiler_totalsize = 0;

	num_l4_entries = (sizeof(l4_map) / sizeof(struct mmu_entry));
	num_l3_mem_entries = sizeof(l3_memory_regions) /
					  sizeof(struct memory_entry);

	phys_addr = a_phy_addr;
	DPRINTK("Value before calling add_dsp_mmu_entry phys_addr = 0x%x,"
		"ducati_boot_addr = 0x%x\n",
		phys_addr, ducati_boot_addr);
	ret_val = add_dsp_mmu_entry(&phys_addr,
		&ducati_boot_addr, PAGE_SIZE_4KB);
	if (WARN_ON(ret_val < 0))
		goto error_exit;

	DPRINTK("Value after calling add_dsp_mmu_entry phys_addr = 0x%x,"
		"ducati_boot_addr = 0x%x\n",
		phys_addr, ducati_boot_addr);

	printk(KERN_ALERT "  Programming Ducati memory regions\n");
	printk(KERN_ALERT "=========================================\n");
	for (i = 0; i < num_l3_mem_entries; i++) {
		printk(KERN_ALERT "VA = [0x%x] of size [0x%x] at PA = [0x%x]\n",
				l3_memory_regions[i].ul_virt_addr,
				l3_memory_regions[i].ul_size, phys_addr);
		if (l3_memory_regions[i].ul_virt_addr == DUCATI_SHARED_IPC_ADDR)
			shm_phys_addr = phys_addr;
		ret_val = add_entry_ext(&phys_addr,
				(u32 *)(&(l3_memory_regions[i].ul_virt_addr)),
				(l3_memory_regions[i].ul_size));
		if (WARN_ON(ret_val < 0))
			goto error_exit;
	}

	tiler_mapbeg = L3_TILER_VIEW0_ADDR;
	tiler_totalsize = DUCATIVA_TILER_VIEW0_LEN;
	phys_addr = L3_TILER_VIEW0_ADDR;

	printk(KERN_ALERT " Programming TILER memory region at "
			"[VA = 0x%x] of size [0x%x] at [PA = 0x%x]\n",
			tiler_mapbeg, tiler_totalsize, phys_addr);
	ret_val = add_entry_ext(&phys_addr, &tiler_mapbeg, tiler_totalsize);
	if (WARN_ON(ret_val < 0))
		goto error_exit;

	map_attrs = 0x00000000;
	map_attrs |= DSP_MAPLITTLEENDIAN;
	map_attrs |= DSP_MAPPHYSICALADDR;
	map_attrs |= DSP_MAPELEMSIZE32;
	printk(KERN_ALERT "  Programming Ducati L4 peripherals\n");
	printk(KERN_ALERT "=========================================\n");
	for (i = 0; i < num_l4_entries; i++) {
		printk(KERN_INFO "PA [0x%x] VA [0x%x] size [0x%x]\n",
				l4_map[i].ul_phy_addr, l4_map[i].ul_virt_addr,
				l4_map[i].ul_size);
		ret_val = ducati_mem_map(l4_map[i].ul_phy_addr,
			l4_map[i].ul_virt_addr, l4_map[i].ul_size, map_attrs);
		if (WARN_ON(ret_val < 0)) {

			DPRINTK("**** Failed to map Peripheral ****");
			DPRINTK("Phys addr [0x%x] Virt addr [0x%x] size [0x%x]",
				l4_map[i].ul_phy_addr, l4_map[i].ul_virt_addr,
				l4_map[i].ul_size);
			DPRINTK(" Status [0x%x]", ret_val);
			goto error_exit;
		}
	}

	DPRINTK("  Programmed Ducati BootVectors 0x0 to first page at [0x%x]",
							a_phy_addr);

	DPRINTK("  Leaving DDucati_MMUManager::ducati_mmu_init [0x%x]",
								ret_val);
	return 0;
error_exit:
	return ret_val;
}

/*========================================
 * This sets up the Ducati processor
 *
 */
int ducati_setup(void)
{
	int ret_val = 0;
	ducati_iommu_ptr = iommu_get("ducati");
	if (IS_ERR(ducati_iommu_ptr)) {
		pr_err("Error iommu_get\n");
		return -EFAULT;
	}

	if (WARN_ON(ret_val < 0))
		goto error_exit;
	ret_val = ducati_mmu_init(DUCATI_BASEIMAGE_PHYSICAL_ADDRESS);
	if (WARN_ON(ret_val < 0))
		goto error_exit;
	return 0;
error_exit:
	WARN_ON(1);
	printk(KERN_ERR "DUCATI SETUP FAILED !!!!!\n");
	return ret_val;
}
EXPORT_SYMBOL(ducati_setup);

/*============================================
 * De-Initialize the Ducati MMU and free the
 * memory allocation for L1 and L2 pages
 *
 */
void ducati_destroy(void)
{
	DPRINTK(" Release IOMMU pointer\n");
	iommu_put(ducati_iommu_ptr);
	ducati_iommu_ptr = NULL;
	return;
}
EXPORT_SYMBOL(ducati_destroy);

/*============================================
 * Returns the ducati virtual address for IPC shared memory
 *
 */
u32 get_ducati_virt_mem()
{
	shm_virt_addr = (u32)ioremap(shm_phys_addr, DUCATI_SHARED_IPC_LEN);
	return shm_virt_addr;
}
EXPORT_SYMBOL(get_ducati_virt_mem);

/*============================================
 * Unmaps the ducati virtual address for IPC shared memory
 *
 */
void unmap_ducati_virt_mem(u32 shm_virt_addr)
{
	iounmap((unsigned int *) shm_virt_addr);
	return;
}
EXPORT_SYMBOL(unmap_ducati_virt_mem);
