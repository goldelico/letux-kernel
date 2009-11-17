/*
 * MMUAccInt.h
 *
 * Syslink ducati driver support for OMAP Processors.
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

#ifndef _MMU_ACC_INT_H
#define _MMU_ACC_INT_H


/* Register offset address definitions */

#define MMU_MMU_REVISION_OFFSET 0x0
#define MMU_MMU_SYSCONFIG_OFFSET 0x10
#define MMU_MMU_SYSSTATUS_OFFSET 014
#define MMU_MMU_IRQSTATUS_OFFSET 0x18
#define MMU_MMU_IRQENABLE_OFFSET 0x1c
#define MMU_MMU_WALKING_ST_OFFSET 0x40
#define MMU_MMU_CNTL_OFFSET     0x44
#define MMU_MMU_FAULT_AD_OFFSET 0x48
#define MMU_MMU_TTB_OFFSET      0x4c
#define MMU_MMU_LOCK_OFFSET     0x50
#define MMU_MMU_LD_TLB_OFFSET   0x54
#define MMU_MMU_CAM_OFFSET      0x58
#define MMU_MMU_RAM_OFFSET      0x5c
#define MMU_MMU_GFLUSH_OFFSET   0x60
#define MMU_MMU_FLUSH_ENTRY_OFFSET 0x64
#define MMU_MMU_READ_CAM_OFFSET	0x68
#define MMU_MMU_READ_RAM_OFFSET	0x6c
#define MMU_MMU_EMU_FAULT_AD_OFFSET	0x70
#define MMU_MMU_FAULT_PC_OFFSET  0x80
#define MMU_MMU_FAULT_STATUS_OFFSET  0x84

/* Bitfield mask and offset declarations */

#define MMU_MMU_REVISION_Rev_MASK	0xff
#define MMU_MMU_REVISION_Rev_OFFSET	0

#define MMU_MMU_SYSCONFIG_ClockActivity_MASK	0x300
#define MMU_MMU_SYSCONFIG_ClockActivity_OFFSET	8

#define MMU_MMU_SYSCONFIG_IdleMode_MASK	0x18
#define MMU_MMU_SYSCONFIG_IdleMode_OFFSET	3

#define MMU_MMU_SYSCONFIG_SoftReset_MASK	0x2
#define MMU_MMU_SYSCONFIG_SoftReset_OFFSET	1

#define MMU_MMU_SYSCONFIG_AutoIdle_MASK	0x1
#define MMU_MMU_SYSCONFIG_AutoIdle_OFFSET	0

#define MMU_MMU_SYSSTATUS_ResetDone_MASK	0x1
#define MMU_MMU_SYSSTATUS_ResetDone_OFFSET	0

#define MMU_MMU_IRQSTATUS_MultiHitFault_MASK	0x10
#define MMU_MMU_IRQSTATUS_MultiHitFault_OFFSET	4

#define MMU_MMU_IRQSTATUS_TableWalkFault_MASK	0x8
#define MMU_MMU_IRQSTATUS_TableWalkFault_OFFSET	3

#define MMU_MMU_IRQSTATUS_EMUMiss_MASK	0x4
#define MMU_MMU_IRQSTATUS_EMUMiss_OFFSET	2

#define MMU_MMU_IRQSTATUS_TranslationFault_MASK	0x2
#define MMU_MMU_IRQSTATUS_TranslationFault_OFFSET	1

#define MMU_MMU_IRQSTATUS_TLBMiss_MASK	0x1
#define MMU_MMU_IRQSTATUS_TLBMiss_OFFSET	0

#define MMU_MMU_IRQENABLE_MultiHitFault_MASK	0x10
#define MMU_MMU_IRQENABLE_MultiHitFault_OFFSET	4

#define MMU_MMU_IRQENABLE_TableWalkFault_MASK	0x8
#define MMU_MMU_IRQENABLE_TableWalkFault_OFFSET	3

#define MMU_MMU_IRQENABLE_EMUMiss_MASK	0x4
#define MMU_MMU_IRQENABLE_EMUMiss_OFFSET	2

#define MMU_MMU_IRQENABLE_TranslationFault_MASK	0x2
#define MMU_MMU_IRQENABLE_TranslationFault_OFFSET	1

#define MMU_MMU_IRQENABLE_TLBMiss_MASK	0x1
#define MMU_MMU_IRQENABLE_TLBMiss_OFFSET	0

#define MMU_MMU_WALKING_ST_TWLRunning_MASK	0x1
#define MMU_MMU_WALKING_ST_TWLRunning_OFFSET	0

#define MMU_MMU_CNTL_EmuTLBUpdate_MASK	0x8
#define MMU_MMU_CNTL_EmuTLBUpdate_OFFSET	3

#define MMU_MMU_CNTL_TWLEnable_MASK	0x4
#define MMU_MMU_CNTL_TWLEnable_OFFSET	2

#define MMU_MMU_CNTL_MMUEnable_MASK	0x2
#define MMU_MMU_CNTL_MMUEnable_OFFSET	1

#define MMU_MMU_FAULT_AD_FaultAddress_MASK	0xffffffff
#define MMU_MMU_FAULT_AD_FaultAddress_OFFSET	0

#define MMU_MMU_TTB_TTBAddress_MASK	0xffffff00
#define MMU_MMU_TTB_TTBAddress_OFFSET	8

#define MMU_MMU_LOCK_BaseValue_MASK	0xfc00
#define MMU_MMU_LOCK_BaseValue_OFFSET	10

#define MMU_MMU_LOCK_CurrentVictim_MASK	0x3f0
#define MMU_MMU_LOCK_CurrentVictim_OFFSET	4

#define MMU_MMU_LD_TLB_LdTLBItem_MASK	0x1
#define MMU_MMU_LD_TLB_LdTLBItem_OFFSET	0

#define MMU_MMU_CAM_VATag_MASK	0xfffff000
#define MMU_MMU_CAM_VATag_OFFSET	12

#define MMU_MMU_CAM_P_MASK	0x8
#define MMU_MMU_CAM_P_OFFSET	3

#define MMU_MMU_CAM_V_MASK	0x4
#define MMU_MMU_CAM_V_OFFSET	2

#define MMU_MMU_CAM_PageSize_MASK	0x3
#define MMU_MMU_CAM_PageSize_OFFSET	0

#define MMU_MMU_RAM_PhysicalAddress_MASK	0xfffff000
#define MMU_MMU_RAM_PhysicalAddress_OFFSET	12

#define MMU_MMU_RAM_Endianness_MASK	0x200
#define MMU_MMU_RAM_Endianness_OFFSET	9

#define MMU_MMU_RAM_ElementSize_MASK	0x180
#define MMU_MMU_RAM_ElementSize_OFFSET	7

#define MMU_MMU_RAM_Mixed_MASK	0x40
#define MMU_MMU_RAM_Mixed_OFFSET	6

#define MMU_MMU_GFLUSH_GlobalFlush_MASK	0x1
#define MMU_MMU_GFLUSH_GlobalFlush_OFFSET	0

#define MMU_MMU_FLUSH_ENTRY_FlushEntry_MASK	0x1
#define MMU_MMU_FLUSH_ENTRY_FlushEntry_OFFSET	0

#define MMU_MMU_READ_CAM_VATag_MASK	0xfffff000
#define MMU_MMU_READ_CAM_VATag_OFFSET	12

#define MMU_MMU_READ_CAM_P_MASK	0x8
#define MMU_MMU_READ_CAM_P_OFFSET	3

#define MMU_MMU_READ_CAM_V_MASK	0x4
#define MMU_MMU_READ_CAM_V_OFFSET	2

#define MMU_MMU_READ_CAM_PageSize_MASK	0x3
#define MMU_MMU_READ_CAM_PageSize_OFFSET	0

#define MMU_MMU_READ_RAM_PhysicalAddress_MASK	0xfffff000
#define MMU_MMU_READ_RAM_PhysicalAddress_OFFSET	12

#define MMU_MMU_READ_RAM_Endianness_MASK	0x200
#define MMU_MMU_READ_RAM_Endianness_OFFSET	9

#define MMU_MMU_READ_RAM_ElementSize_MASK	0x180
#define MMU_MMU_READ_RAM_ElementSize_OFFSET	7

#define MMU_MMU_READ_RAM_Mixed_MASK	0x40
#define MMU_MMU_READ_RAM_Mixed_OFFSET	6

#define MMU_MMU_EMU_FAULT_AD_EmuFaultAddress_MASK	0xffffffff
#define MMU_MMU_EMU_FAULT_AD_EmuFaultAddress_OFFSET	0

#endif /* _MMU_ACC_INT_H */
/* EOF */

