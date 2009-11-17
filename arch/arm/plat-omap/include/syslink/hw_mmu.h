/*
 * hw_mbox.h
 *
 * Syslink driver support for OMAP Processors.
 *
 *  Copyright (C) 2008-2009 Texas Instruments, Inc.
 *
 *  This package is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 *  IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 *  WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE.
 */

#ifndef __HW_MMU_H
#define __HW_MMU_H

#include <linux/types.h>

/* Bitmasks for interrupt sources */
#define HW_MMU_TRANSLATION_FAULT   0x2
#define HW_MMU_ALL_INTERRUPTS      0x1F

#define HW_MMU_COARSE_PAGE_SIZE 0x400

/* hw_mmu_mixed_size_t:  Enumerated Type used to specify whether to follow
			CPU/TLB Element size */
enum hw_mmu_mixed_size_t {
	HW_MMU_TLBES,
	HW_MMU_CPUES

} ;

/* hw_mmu_map_attrs_t:  Struct containing MMU mapping attributes */
struct hw_mmu_map_attrs_t {
	enum hw_endianism_t     endianism;
	enum hw_elemnt_siz_t   element_size;
	enum hw_mmu_mixed_size_t  mixedSize;
} ;

extern hw_status hw_mmu_enable(const u32 base_address);

extern hw_status hw_mmu_disable(const u32 base_address);

extern hw_status hw_mmu_numlocked_set(const u32 base_address,
				      u32 num_lcked_entries);

extern hw_status hw_mmu_victim_numset(const u32 base_address,
				      u32 vctm_entry_num);

/* For MMU faults */
extern hw_status hw_mmu_eventack(const u32 base_address,
				 u32 irq_mask);

extern hw_status hw_mmu_event_disable(const u32 base_address,
				      u32 irq_mask);

extern hw_status hw_mmu_event_enable(const u32 base_address,
				     u32 irq_mask);

extern hw_status hw_mmu_event_status(const u32 base_address,
				     u32 *irq_mask);

extern hw_status hw_mmu_flt_adr_rd(const u32 base_address,
				   u32 *addr);

/* Set the TT base address */
extern hw_status hw_mmu_ttbset(const u32 base_address,
			       u32 ttb_phys_addr);

extern hw_status hw_mmu_twl_enable(const u32 base_address);

extern hw_status hw_mmu_twl_disable(const u32 base_address);

extern hw_status hw_mmu_tlb_flush(const u32 base_address,
				  u32 virtual_addr,
				  u32 page_size);

extern hw_status hw_mmu_tlb_flushAll(const u32 base_address);

extern hw_status hw_mmu_tlb_add(const u32     base_address,
				u32	   physical_addr,
				u32	   virtual_addr,
				u32	   page_size,
				u32	    entryNum,
				struct hw_mmu_map_attrs_t *map_attrs,
				enum hw_set_clear_t    preserve_bit,
				enum hw_set_clear_t    valid_bit);


/* For PTEs */
extern hw_status hw_mmu_pte_set(const u32     pg_tbl_va,
				u32	   physical_addr,
				u32	   virtual_addr,
				u32	   page_size,
				struct hw_mmu_map_attrs_t *map_attrs);

extern hw_status hw_mmu_pte_clear(const u32   pg_tbl_va,
				  u32	 pg_size,
				  u32	 virtual_addr);

static inline u32 hw_mmu_pte_addr_l1(u32 l1_base, u32 va)
{
	u32 pte_addr;
	u32 VA_31_to_20;

	VA_31_to_20  = va >> (20 - 2); /* Left-shift by 2 here itself */
	VA_31_to_20 &= 0xFFFFFFFCUL;
	pte_addr = l1_base + VA_31_to_20;

	return pte_addr;
}

static inline u32 hw_mmu_pte_addr_l2(u32 l2_base, u32 va)
{
	u32 pte_addr;

	pte_addr = (l2_base & 0xFFFFFC00) | ((va >> 10) & 0x3FC);

	return pte_addr;
}

static inline u32 hw_mmu_pte_coarsel1(u32 pte_val)
{
	u32 pteCoarse;

	pteCoarse = pte_val & 0xFFFFFC00;

	return pteCoarse;
}

static inline u32 hw_mmu_pte_sizel1(u32 pte_val)
{
	u32 pte_size = 0;

	if ((pte_val & 0x3) == 0x1) {
		/* Points to L2 PT */
		pte_size = HW_MMU_COARSE_PAGE_SIZE;
	}

	if ((pte_val & 0x3) == 0x2) {
		if (pte_val & (1 << 18))
			pte_size = HW_PAGE_SIZE_16MB;
		else
			pte_size = HW_PAGE_SIZE_1MB;
	}

	return pte_size;
}

static inline u32 hw_mmu_pte_sizel2(u32 pte_val)
{
	u32 pte_size = 0;

	if (pte_val & 0x2)
		pte_size = HW_PAGE_SIZE_4KB;
	else if (pte_val & 0x1)
		pte_size = HW_PAGE_SIZE_64KB;

	return pte_size;
}
extern hw_status hw_mmu_tlb_dump(u32 base_address, bool shw_inv_entries);

extern u32 hw_mmu_pte_phyaddr(u32 pte_val, u32 pte_size);

extern u32 hw_mmu_fault_dump(const u32 base_address);

#endif  /* __HW_MMU_H */
