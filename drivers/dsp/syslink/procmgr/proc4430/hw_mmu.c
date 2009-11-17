/*
 * hw_mbox.c
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

#include<linux/kernel.h>
#include<linux/module.h>

#include <syslink/GlobalTypes.h>
#include <syslink/MMURegAcM.h>
#include <syslink/hw_defs.h>
#include <syslink/hw_mmu.h>

#define MMU_BASE_VAL_MASK        0xFC00
#define MMU_PAGE_MAX             3
#define MMU_ELEMENTSIZE_MAX      3
#define MMU_ADDR_MASK            0xFFFFF000
#define MMU_TTB_MASK             0xFFFFC000
#define MMU_SECTION_ADDR_MASK    0xFFF00000
#define MMU_SSECTION_ADDR_MASK   0xFF000000
#define MMU_PAGE_TABLE_MASK      0xFFFFFC00
#define MMU_LARGE_PAGE_MASK      0xFFFF0000
#define MMU_SMALL_PAGE_MASK      0xFFFFF000

#define MMU_LOAD_TLB        0x00000001
#define NUM_TLB_ENTRIES 32



/*
* type: hw_mmu_pgsiz_t
*
* desc:  Enumerated Type used to specify the MMU Page Size(SLSS)
*
*
*/
enum hw_mmu_pgsiz_t {
	HW_MMU_SECTION,
	HW_MMU_LARGE_PAGE,
	HW_MMU_SMALL_PAGE,
	HW_MMU_SUPERSECTION

};

/*
* function : mmu_flsh_entry
*/

static hw_status mmu_flsh_entry(const u32   base_address);

 /*
* function : mme_set_cam_entry
*
*/

static hw_status mme_set_cam_entry(const u32    base_address,
	const u32    page_size,
	const u32    preserve_bit,
	const u32    valid_bit,
	const u32    virt_addr_tag);

/*
* function  : mmu_set_ram_entry
*/
static hw_status mmu_set_ram_entry(const u32        base_address,
	const u32        physical_addr,
	enum hw_endianism_t      endianism,
	enum hw_elemnt_siz_t    element_size,
	enum hw_mmu_mixed_size_t   mixedSize);

/*
* hw functions
*
*/

hw_status hw_mmu_enable(const u32 base_address)
{
	hw_status status = RET_OK;

	MMUMMU_CNTLMMUEnableWrite32(base_address, HW_SET);

	return status;
}
EXPORT_SYMBOL(hw_mmu_enable);

hw_status hw_mmu_disable(const u32 base_address)
{
	hw_status status = RET_OK;

	MMUMMU_CNTLMMUEnableWrite32(base_address, HW_CLEAR);

	return status;
}
EXPORT_SYMBOL(hw_mmu_disable);

hw_status hw_mmu_autoidle_en(const u32 base_address)
{
	hw_status status;

	status = mmu_sisconf_auto_idle_set32(base_address, HW_SET);
	status = RET_OK;
	return status;
}
EXPORT_SYMBOL(hw_mmu_autoidle_en);

hw_status hw_mmu_nulck_set(const u32 base_address, u32 *num_lcked_entries)
{
	hw_status status = RET_OK;

	*num_lcked_entries = MMUMMU_LOCKBaseValueRead32(base_address);

	return status;
}
EXPORT_SYMBOL(hw_mmu_nulck_set);


hw_status hw_mmu_numlocked_set(const u32 base_address, u32 num_lcked_entries)
{
	hw_status status = RET_OK;

	MMUMMU_LOCKBaseValueWrite32(base_address, num_lcked_entries);

	return status;
}
EXPORT_SYMBOL(hw_mmu_numlocked_set);


hw_status hw_mmu_vctm_numget(const u32 base_address, u32 *vctm_entry_num)
{
	hw_status status = RET_OK;

	*vctm_entry_num = MMUMMU_LOCKCurrentVictimRead32(base_address);

	return status;
}
EXPORT_SYMBOL(hw_mmu_vctm_numget);


hw_status hw_mmu_victim_numset(const u32 base_address, u32 vctm_entry_num)
{
	hw_status status = RET_OK;

	mmu_lck_crnt_vctmwite32(base_address, vctm_entry_num);

	return status;
}
EXPORT_SYMBOL(hw_mmu_victim_numset);

hw_status hw_mmu_tlb_flushAll(const u32 base_address)
{
	hw_status status = RET_OK;

	MMUMMU_GFLUSHGlobalFlushWrite32(base_address, HW_SET);

	return status;
}
EXPORT_SYMBOL(hw_mmu_tlb_flushAll);

hw_status hw_mmu_eventack(const u32 base_address, u32 irq_mask)
{
	hw_status status = RET_OK;

	MMUMMU_IRQSTATUSWriteRegister32(base_address, irq_mask);

	return status;
}
EXPORT_SYMBOL(hw_mmu_eventack);

hw_status hw_mmu_event_disable(const u32 base_address, u32 irq_mask)
{
	hw_status status = RET_OK;
	u32 irqReg;
	irqReg = MMUMMU_IRQENABLEReadRegister32(base_address);

	MMUMMU_IRQENABLEWriteRegister32(base_address, irqReg & ~irq_mask);

	return status;
}
EXPORT_SYMBOL(hw_mmu_event_disable);

hw_status hw_mmu_event_enable(const u32 base_address, u32 irq_mask)
{
	hw_status status = RET_OK;
	u32 irqReg;

	irqReg = MMUMMU_IRQENABLEReadRegister32(base_address);

	MMUMMU_IRQENABLEWriteRegister32(base_address, irqReg | irq_mask);

	return status;
}
EXPORT_SYMBOL(hw_mmu_event_enable);

hw_status hw_mmu_event_status(const u32 base_address, u32 *irq_mask)
{
	hw_status status = RET_OK;

	*irq_mask = MMUMMU_IRQSTATUSReadRegister32(base_address);

	return status;
}
EXPORT_SYMBOL(hw_mmu_event_status);

hw_status hw_mmu_flt_adr_rd(const u32 base_address, u32 *addr)
{
	hw_status status = RET_OK;

	/*Check the input Parameters*/
	CHECK_INPUT_PARAM(base_address, 0, RET_BAD_NULL_PARAM,
					RES_MMU_BASE + RES_INVALID_INPUT_PARAM);
	/* read values from register */
	*addr = MMUMMU_FAULT_ADReadRegister32(base_address);

	return status;
}
EXPORT_SYMBOL(hw_mmu_flt_adr_rd);


hw_status hw_mmu_ttbset(const u32 base_address, u32 ttb_phys_addr)
{
	hw_status status = RET_OK;
	u32 loadTTB;

	/*Check the input Parameters*/
	CHECK_INPUT_PARAM(base_address, 0, RET_BAD_NULL_PARAM,
					RES_MMU_BASE + RES_INVALID_INPUT_PARAM);

	loadTTB = ttb_phys_addr & ~0x7FUL;
	/* write values to register */
	MMUMMU_TTBWriteRegister32(base_address, loadTTB);

	return status;
}
EXPORT_SYMBOL(hw_mmu_ttbset);

hw_status hw_mmu_twl_enable(const u32 base_address)
{
	hw_status status = RET_OK;

	MMUMMU_CNTLTWLEnableWrite32(base_address, HW_SET);

	return status;
}
EXPORT_SYMBOL(hw_mmu_twl_enable);

hw_status hw_mmu_twl_disable(const u32 base_address)
{
	hw_status status = RET_OK;

	MMUMMU_CNTLTWLEnableWrite32(base_address, HW_CLEAR);

	return status;
}
EXPORT_SYMBOL(hw_mmu_twl_disable);


hw_status hw_mmu_tlb_flush(const u32 base_address,
	u32 virtual_addr,
	u32 page_size)
{
	hw_status status = RET_OK;
	u32 virt_addr_tag;
	enum hw_mmu_pgsiz_t pg_sizeBits;

	switch (page_size) {
	case HW_PAGE_SIZE_4KB:
	pg_sizeBits = HW_MMU_SMALL_PAGE;
	break;

	case HW_PAGE_SIZE_64KB:
	pg_sizeBits = HW_MMU_LARGE_PAGE;
	break;

	case HW_PAGE_SIZE_1MB:
	pg_sizeBits = HW_MMU_SECTION;
	break;

	case HW_PAGE_SIZE_16MB:
	pg_sizeBits = HW_MMU_SUPERSECTION;
	break;

	default:
	return RET_FAIL;
	}

	/* Generate the 20-bit tag from virtual address */
	virt_addr_tag = ((virtual_addr & MMU_ADDR_MASK) >> 12);

	mme_set_cam_entry(base_address, pg_sizeBits, 0, 0, virt_addr_tag);

	mmu_flsh_entry(base_address);

	return status;
}
EXPORT_SYMBOL(hw_mmu_tlb_flush);


hw_status hw_mmu_tlb_add(const u32        base_address,
	u32              physical_addr,
	u32              virtual_addr,
	u32              page_size,
	u32              entryNum,
	struct hw_mmu_map_attrs_t    *map_attrs,
	enum hw_set_clear_t       preserve_bit,
	enum hw_set_clear_t       valid_bit)
{
	hw_status  status = RET_OK;
	u32 lockReg;
	u32 virt_addr_tag;
	enum hw_mmu_pgsiz_t mmu_pg_size;

	/*Check the input Parameters*/
	CHECK_INPUT_PARAM(base_address, 0, RET_BAD_NULL_PARAM,
				RES_MMU_BASE + RES_INVALID_INPUT_PARAM);
	CHECK_INPUT_RANGE_MIN0(page_size, MMU_PAGE_MAX, RET_PARAM_OUT_OF_RANGE,
				RES_MMU_BASE + RES_INVALID_INPUT_PARAM);
	CHECK_INPUT_RANGE_MIN0(map_attrs->element_size,
			MMU_ELEMENTSIZE_MAX, RET_PARAM_OUT_OF_RANGE,
				RES_MMU_BASE + RES_INVALID_INPUT_PARAM);

		switch (page_size) {
		case HW_PAGE_SIZE_4KB:
		mmu_pg_size = HW_MMU_SMALL_PAGE;
		break;

		case HW_PAGE_SIZE_64KB:
		mmu_pg_size = HW_MMU_LARGE_PAGE;
		break;

		case HW_PAGE_SIZE_1MB:
		mmu_pg_size = HW_MMU_SECTION;
		break;

		case HW_PAGE_SIZE_16MB:
		mmu_pg_size = HW_MMU_SUPERSECTION;
		break;

		default:
		return RET_FAIL;
		}

	lockReg = mmu_lckread_reg_32(base_address);

	/* Generate the 20-bit tag from virtual address */
	virt_addr_tag = ((virtual_addr & MMU_ADDR_MASK) >> 12);

	/* Write the fields in the CAM Entry Register */
	mme_set_cam_entry(base_address,  mmu_pg_size, preserve_bit, valid_bit,
	virt_addr_tag);

	/* Write the different fields of the RAM Entry Register */
	/* endianism of the page,Element Size of the page (8, 16, 32, 64 bit) */
	mmu_set_ram_entry(base_address, physical_addr,
	map_attrs->endianism, map_attrs->element_size, map_attrs->mixedSize);

	/* Update the MMU Lock Register */
	/* currentVictim between lockedBaseValue and (MMU_Entries_Number - 1) */
	mmu_lck_crnt_vctmwite32(base_address, entryNum);

	/* Enable loading of an entry in TLB by writing 1 into LD_TLB_REG
	register */
	mmu_ld_tlbwrt_reg32(base_address, MMU_LOAD_TLB);


	mmu_lck_write_reg32(base_address, lockReg);

	return status;
}
EXPORT_SYMBOL(hw_mmu_tlb_add);



hw_status hw_mmu_pte_set(const u32        pg_tbl_va,
	u32              physical_addr,
	u32              virtual_addr,
	u32              page_size,
	struct hw_mmu_map_attrs_t    *map_attrs)
{
	hw_status status = RET_OK;
	u32 pte_addr, pte_val;
	long int num_entries = 1;

	switch (page_size) {

	case HW_PAGE_SIZE_4KB:
	pte_addr = hw_mmu_pte_addr_l2(pg_tbl_va, virtual_addr &
		MMU_SMALL_PAGE_MASK);
	pte_val = ((physical_addr & MMU_SMALL_PAGE_MASK) |
	(map_attrs->endianism << 9) |
	(map_attrs->element_size << 4) |
	(map_attrs->mixedSize << 11) | 2
	);
	break;

	case HW_PAGE_SIZE_64KB:
	num_entries = 16;
	pte_addr = hw_mmu_pte_addr_l2(pg_tbl_va, virtual_addr &
		MMU_LARGE_PAGE_MASK);
	pte_val = ((physical_addr & MMU_LARGE_PAGE_MASK) |
	(map_attrs->endianism << 9) |
	(map_attrs->element_size << 4) |
	(map_attrs->mixedSize << 11) | 1
	);
	break;

	case HW_PAGE_SIZE_1MB:
	pte_addr = hw_mmu_pte_addr_l1(pg_tbl_va, virtual_addr &
		MMU_SECTION_ADDR_MASK);
	pte_val = ((((physical_addr & MMU_SECTION_ADDR_MASK) |
	(map_attrs->endianism << 15) |
	(map_attrs->element_size << 10) |
	(map_attrs->mixedSize << 17)) &
	~0x40000) | 0x2
	);
	break;

	case HW_PAGE_SIZE_16MB:
	num_entries = 16;
	pte_addr = hw_mmu_pte_addr_l1(pg_tbl_va, virtual_addr &
		MMU_SSECTION_ADDR_MASK);
	pte_val = (((physical_addr & MMU_SSECTION_ADDR_MASK) |
	(map_attrs->endianism << 15) |
	(map_attrs->element_size << 10) |
	(map_attrs->mixedSize << 17)
	) | 0x40000 | 0x2
	);
	break;

	case HW_MMU_COARSE_PAGE_SIZE:
	pte_addr = hw_mmu_pte_addr_l1(pg_tbl_va, virtual_addr &
		MMU_SECTION_ADDR_MASK);
	pte_val = (physical_addr & MMU_PAGE_TABLE_MASK) | 1;
	break;

	default:
	return RET_FAIL;
	}

	while (--num_entries >= 0)
		((u32 *)pte_addr)[num_entries] = pte_val;


	return status;
}
EXPORT_SYMBOL(hw_mmu_pte_set);

hw_status hw_mmu_pte_clear(const u32  pg_tbl_va,
	u32        virtual_addr,
	u32        pg_size)
{
	hw_status status = RET_OK;
	u32 pte_addr;
	long int num_entries = 1;

	switch (pg_size) {
	case HW_PAGE_SIZE_4KB:
	pte_addr = hw_mmu_pte_addr_l2(pg_tbl_va,
			virtual_addr & MMU_SMALL_PAGE_MASK);
	break;

	case HW_PAGE_SIZE_64KB:
	num_entries = 16;
	pte_addr = hw_mmu_pte_addr_l2(pg_tbl_va,
			virtual_addr & MMU_LARGE_PAGE_MASK);
	break;

	case HW_PAGE_SIZE_1MB:
	case HW_MMU_COARSE_PAGE_SIZE:
	pte_addr = hw_mmu_pte_addr_l1(pg_tbl_va,
			virtual_addr & MMU_SECTION_ADDR_MASK);
	break;

	case HW_PAGE_SIZE_16MB:
	num_entries = 16;
	pte_addr = hw_mmu_pte_addr_l1(pg_tbl_va,
			virtual_addr & MMU_SSECTION_ADDR_MASK);
	break;

	default:
	return RET_FAIL;
	}

	while (--num_entries >= 0)
		((u32 *)pte_addr)[num_entries] = 0;

	return status;
}
EXPORT_SYMBOL(hw_mmu_pte_clear);

/*
* function: mmu_flsh_entry
*/
static hw_status mmu_flsh_entry(const u32 base_address)
{
	hw_status status = RET_OK;
	u32 flushEntryData = 0x1;

	/*Check the input Parameters*/
	CHECK_INPUT_PARAM(base_address, 0, RET_BAD_NULL_PARAM,
				RES_MMU_BASE + RES_INVALID_INPUT_PARAM);

	/* write values to register */
	MMUMMU_FLUSH_ENTRYWriteRegister32(base_address, flushEntryData);

	return status;
}
EXPORT_SYMBOL(mmu_flsh_entry);
/*
* function : mme_set_cam_entry
*/
static hw_status mme_set_cam_entry(const u32    base_address,
	const u32    page_size,
	const u32    preserve_bit,
	const u32    valid_bit,
	const u32    virt_addr_tag)
{
	hw_status status = RET_OK;
	u32 mmuCamReg;

	/*Check the input Parameters*/
	CHECK_INPUT_PARAM(base_address, 0, RET_BAD_NULL_PARAM,
				RES_MMU_BASE + RES_INVALID_INPUT_PARAM);

	mmuCamReg = (virt_addr_tag << 12);
	mmuCamReg = (mmuCamReg) | (page_size) |  (valid_bit << 2)
						| (preserve_bit << 3);

	/* write values to register */
	MMUMMU_CAMWriteRegister32(base_address, mmuCamReg);

	return status;
}
EXPORT_SYMBOL(mme_set_cam_entry);
/*
* function: mmu_set_ram_entry
*/
static hw_status mmu_set_ram_entry(const u32       base_address,
	const u32       physical_addr,
	enum hw_endianism_t     endianism,
	enum hw_elemnt_siz_t   element_size,
	enum hw_mmu_mixed_size_t  mixedSize)
{
	hw_status status = RET_OK;
	u32 mmuRamReg;

	/*Check the input Parameters*/
	CHECK_INPUT_PARAM(base_address, 0, RET_BAD_NULL_PARAM,
					RES_MMU_BASE + RES_INVALID_INPUT_PARAM);
	CHECK_INPUT_RANGE_MIN0(element_size, MMU_ELEMENTSIZE_MAX,
					RET_PARAM_OUT_OF_RANGE,
					RES_MMU_BASE + RES_INVALID_INPUT_PARAM);


	mmuRamReg = (physical_addr & MMU_ADDR_MASK);
	mmuRamReg = (mmuRamReg) | ((endianism << 9) |  (element_size << 7)
					| (mixedSize << 6));

	/* write values to register */
	MMUMMU_RAMWriteRegister32(base_address, mmuRamReg);

	return status;

}
EXPORT_SYMBOL(mmu_set_ram_entry);

u32 hw_mmu_fault_dump(const u32 base_address)
{
	u32 reg;

	reg = MMUMMU_FAULT_ADReadRegister32(base_address);
	printk(KERN_INFO "Fault Address Address = 0x%x\n", reg);
	reg = MMUMMU_FAULT_PCReadRegister32(base_address);
	printk(KERN_INFO "Fault PC Register Address = 0x%x\n", reg);
	reg = MMUMMU_FAULT_STATUSReadRegister32(base_address);
	printk(KERN_INFO "Fault PC address doesn't show right value in DUCATI"
				"because of HW limitation\n");
	printk(KERN_INFO "Fault Status Register  = 0x%x\n", reg);
	reg = MMUMMU_FAULT_EMUAddressReadRegister32(base_address);
	printk(KERN_INFO "Fault EMU  Address = 0x%x\n", reg);
	return 0;
}
EXPORT_SYMBOL(hw_mmu_fault_dump);

long hw_mmu_tlb_dump(const u32 base_address, bool shw_inv_entries)
{
	u32 i;
	u32 lockSave;
	u32 cam;
	u32 ram;


	/*  Save off the lock register contents,
	we'll restore it when we are done  */

	lockSave = mmu_lckread_reg_32(base_address);

	printk(KERN_INFO "TLB locked entries = %u, current victim = %u\n",
		((lockSave & MMU_MMU_LOCK_BaseValue_MASK)
		>> MMU_MMU_LOCK_BaseValue_OFFSET),
		((lockSave & MMU_MMU_LOCK_CurrentVictim_MASK)
		>> MMU_MMU_LOCK_CurrentVictim_OFFSET));
	printk(KERN_INFO "=============================================\n");
	for (i = 0; i < NUM_TLB_ENTRIES; i++) {
		mmu_lck_crnt_vctmwite32(base_address, i);
		cam = MMUMMU_CAMReadRegister32(base_address);
		ram = MMUMMU_RAMReadRegister32(base_address);

		if ((cam & 0x4) != 0) {
			printk(KERN_INFO "TLB Entry [0x%2x]: VA = 0x%8x "
				"PA = 0x%8x Protected = 0x%1x\n",
				i, (cam & MMU_ADDR_MASK), (ram & MMU_ADDR_MASK),
				(cam & 0x8) ? 1 : 0);

		} else if (shw_inv_entries != false)
			printk(KERN_ALERT "TLB Entry [0x%x]: <INVALID>\n", i);
	}
	mmu_lck_write_reg32(base_address, lockSave);
	return RET_OK;
}
EXPORT_SYMBOL(hw_mmu_tlb_dump);

u32 hw_mmu_pte_phyaddr(u32 pte_val, u32 pte_size)
{
	u32	ret_val = 0;

	switch (pte_size) {

	case HW_PAGE_SIZE_4KB:
		ret_val = pte_val & MMU_SMALL_PAGE_MASK;
		break;
	case HW_PAGE_SIZE_64KB:
		ret_val = pte_val & MMU_LARGE_PAGE_MASK;
		break;

	case HW_PAGE_SIZE_1MB:
		ret_val = pte_val & MMU_SECTION_ADDR_MASK;
		break;
	case HW_PAGE_SIZE_16MB:
		ret_val = pte_val & MMU_SSECTION_ADDR_MASK;
		break;
	default:
		/*  Invalid  */
		break;

	}

	return ret_val;
}
EXPORT_SYMBOL(hw_mmu_pte_phyaddr);
