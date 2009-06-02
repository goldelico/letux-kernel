/*
 * MBXRegAcM.h
 *
 * Notify mailbox driver support for OMAP Processors.
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

#ifndef _MBX_REG_ACM_H
#define _MBX_REG_ACM_H


#include <syslink/GlobalTypes.h>
#include <syslink/MBXAccInt.h>


/*
 * EXPORTED DEFINITIONS
 *
 */

#if defined(USE_LEVEL_1_MACROS)

#define MLBMAILBOX_REVISIONReadRegister32(base_address)\
(RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
MLB_MAILBOX_REVISION_OFFSET))



#define MLBMAILBOX_REVISIONRevRead32(base_address)\
((((RD_MEM_32_VOLATILE((((unsigned long)(base_address))+ \
(MLB_MAILBOX_REVISION_OFFSET)))) &\
MLB_MAILBOX_REVISION_Rev_MASK) >>\
MLB_MAILBOX_REVISION_Rev_OFFSET))



#define MLBMAILBOX_REVISIONRevGet32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_REVISION_Rev_MASK) \
>> MLB_MAILBOX_REVISION_Rev_OFFSET))



#define MLBMAILBOX_SYSCONFIGReadRegister32(base_address)\
(RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
MLB_MAILBOX_SYSCONFIG_OFFSET))


#define MLBMAILBOX_SYSCONFIGWriteRegister32(base_address, value)\
do {\
	const unsigned long offset = MLB_MAILBOX_SYSCONFIG_OFFSET;\
	register unsigned long newValue = ((unsigned long)(value));\
	WR_MEM_32_VOLATILE(((unsigned long)(base_address))+ \
	offset, newValue);\
} while (0)


#define MLBMAILBOX_SYSCONFIGClockActivityRead32(base_address)\
((((RD_MEM_32_VOLATILE((((unsigned long)(base_address))+\
(MLB_MAILBOX_SYSCONFIG_OFFSET)))) &\
MLB_MAILBOX_SYSCONFIG_ClockActivity_MASK) >>\
MLB_MAILBOX_SYSCONFIG_ClockActivity_OFFSET))


#define MLBMAILBOX_SYSCONFIGClockActivityGet32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_SYSCONFIG_ClockActivity_MASK) \
>> MLB_MAILBOX_SYSCONFIG_ClockActivity_OFFSET))


#define MLBMAILBOX_SYSCONFIGSIdleModeRead32(base_address)\
((((RD_MEM_32_VOLATILE((((unsigned long)(base_address))+\
(MLB_MAILBOX_SYSCONFIG_OFFSET)))) &\
MLB_MAILBOX_SYSCONFIG_SIdleMode_MASK) >>\
MLB_MAILBOX_SYSCONFIG_SIdleMode_OFFSET))


#define MLBMAILBOX_SYSCONFIGSIdleModeReadIsb0032(base_address)\
((MLBMAILBOX_SYSCONFIGSIdleModeb00 == (MLBMAILBOX_SYSCONFIGSIdleModeE)\
(((RD_MEM_32_VOLATILE((((unsigned long)(base_address))+\
(MLB_MAILBOX_SYSCONFIG_OFFSET)))) &\
MLB_MAILBOX_SYSCONFIG_SIdleMode_MASK) >>\
MLB_MAILBOX_SYSCONFIG_SIdleMode_OFFSET)))


#define MLBMAILBOX_SYSCONFIGSIdleModeReadIsb0132(base_address)\
((MLBMAILBOX_SYSCONFIGSIdleModeb01 == (MLBMAILBOX_SYSCONFIGSIdleModeE)\
(((RD_MEM_32_VOLATILE((((unsigned long)(base_address))+\
(MLB_MAILBOX_SYSCONFIG_OFFSET)))) &\
MLB_MAILBOX_SYSCONFIG_SIdleMode_MASK) >>\
MLB_MAILBOX_SYSCONFIG_SIdleMode_OFFSET)))


#define MLBMAILBOX_SYSCONFIGSIdleModeReadIsb1032(base_address)\
((MLBMAILBOX_SYSCONFIGSIdleModeb10 == (MLBMAILBOX_SYSCONFIGSIdleModeE)\
(((RD_MEM_32_VOLATILE((((unsigned long)(base_address))+\
(MLB_MAILBOX_SYSCONFIG_OFFSET)))) &\
MLB_MAILBOX_SYSCONFIG_SIdleMode_MASK) >>\
MLB_MAILBOX_SYSCONFIG_SIdleMode_OFFSET)))

#define MLBMAILBOX_SYSCONFIGSIdleModeReadIsb1132(base_address)\
((MLBMAILBOX_SYSCONFIGSIdleModeb11 == (MLBMAILBOX_SYSCONFIGSIdleModeE)\
(((RD_MEM_32_VOLATILE((((unsigned long)(base_address))+\
(MLB_MAILBOX_SYSCONFIG_OFFSET)))) &\
MLB_MAILBOX_SYSCONFIG_SIdleMode_MASK) >>\
MLB_MAILBOX_SYSCONFIG_SIdleMode_OFFSET)))

#define MLBMAILBOX_SYSCONFIGSIdleModeGet32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_SYSCONFIG_SIdleMode_MASK) >> \
MLB_MAILBOX_SYSCONFIG_SIdleMode_OFFSET))

#define MLBMAILBOX_SYSCONFIGSIdleModeIsb0032(var)\
((MLBMAILBOX_SYSCONFIGSIdleModeb00  ==  \
(MLBMAILBOX_SYSCONFIGSIdleModeE)\
((((unsigned long)(var)) & MLB_MAILBOX_SYSCONFIG_SIdleMode_MASK) >>\
MLB_MAILBOX_SYSCONFIG_SIdleMode_OFFSET)))

#define MLBMAILBOX_SYSCONFIGSIdleModeIsb0132(var)\
((MLBMAILBOX_SYSCONFIGSIdleModeb01 == (MLBMAILBOX_SYSCONFIGSIdleModeE)\
((((unsigned long)(var)) & MLB_MAILBOX_SYSCONFIG_SIdleMode_MASK) >>\
MLB_MAILBOX_SYSCONFIG_SIdleMode_OFFSET)))

#define MLBMAILBOX_SYSCONFIGSIdleModeIsb1032(var)\
((MLBMAILBOX_SYSCONFIGSIdleModeb10 == (MLBMAILBOX_SYSCONFIGSIdleModeE)\
((((unsigned long)(var)) & MLB_MAILBOX_SYSCONFIG_SIdleMode_MASK) >>\
MLB_MAILBOX_SYSCONFIG_SIdleMode_OFFSET)))

#define MLBMAILBOX_SYSCONFIGSIdleModeIsb1132(var)\
((MLBMAILBOX_SYSCONFIGSIdleModeb11 == (MLBMAILBOX_SYSCONFIGSIdleModeE)\
((((unsigned long)(var)) & MLB_MAILBOX_SYSCONFIG_SIdleMode_MASK) >>\
MLB_MAILBOX_SYSCONFIG_SIdleMode_OFFSET)))

#define MLBMAILBOX_SYSCONFIGSIdleModeWrite32(base_address, value)\
do {\
	const unsigned long offset = MLB_MAILBOX_SYSCONFIG_OFFSET;\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_SYSCONFIG_SIdleMode_MASK);\
	newValue <<= MLB_MAILBOX_SYSCONFIG_SIdleMode_OFFSET;\
	newValue &= MLB_MAILBOX_SYSCONFIG_SIdleMode_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+ \
	offset, newValue);\
} while (0)

#define MLBMAILBOX_SYSCONFIGSIdleModeWriteb0032(base_address)\
do {\
	const unsigned long offset = MLB_MAILBOX_SYSCONFIG_OFFSET;\
	const unsigned long newValue = \
	(unsigned long)MLBMAILBOX_SYSCONFIGSIdleModeb00 <<\
	MLB_MAILBOX_SYSCONFIG_SIdleMode_OFFSET;\
	register unsigned long data = \
	RD_MEM_32_VOLATILE((unsigned long)(base_address)+offset);\
	data &= ~(MLB_MAILBOX_SYSCONFIG_SIdleMode_MASK);\
	data |= newValue;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, data);\
} while (0)

#define MLBMAILBOX_SYSCONFIGSIdleModeWriteb0132(base_address)\
do {\
	const unsigned long offset = MLB_MAILBOX_SYSCONFIG_OFFSET;\
	const unsigned long newValue = \
	(unsigned long)MLBMAILBOX_SYSCONFIGSIdleModeb01 <<\
	MLB_MAILBOX_SYSCONFIG_SIdleMode_OFFSET;\
	register unsigned long data = \
	RD_MEM_32_VOLATILE((unsigned long)(base_address)+offset);\
	data &= ~(MLB_MAILBOX_SYSCONFIG_SIdleMode_MASK);\
	data |= newValue;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, data);\
} while (0)

#define MLBMAILBOX_SYSCONFIGSIdleModeWriteb1032(base_address)\
do {\
	const unsigned long offset = MLB_MAILBOX_SYSCONFIG_OFFSET;\
	const unsigned long newValue = \
	(unsigned long)MLBMAILBOX_SYSCONFIGSIdleModeb10 <<\
	MLB_MAILBOX_SYSCONFIG_SIdleMode_OFFSET;\
	register unsigned long data = \
	RD_MEM_32_VOLATILE((unsigned long)(base_address)+offset);\
	data &= ~(MLB_MAILBOX_SYSCONFIG_SIdleMode_MASK);\
	data |= newValue;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, data);\
} while (0)

#define MLBMAILBOX_SYSCONFIGSIdleModeWriteb1132(base_address)\
do {\
	const unsigned long offset = MLB_MAILBOX_SYSCONFIG_OFFSET;\
	const unsigned long newValue = \
	(unsigned long)MLBMAILBOX_SYSCONFIGSIdleModeb11 <<\
	MLB_MAILBOX_SYSCONFIG_SIdleMode_OFFSET;\
	register unsigned long data = \
	RD_MEM_32_VOLATILE((unsigned long)(base_address)+offset);\
	data &= ~(MLB_MAILBOX_SYSCONFIG_SIdleMode_MASK);\
	data |= newValue;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, data);\
} while (0)

#define MLBMAILBOX_SYSCONFIGSIdleModeSet32(var, value)\
(((((unsigned long)(var)) & ~(MLB_MAILBOX_SYSCONFIG_SIdleMode_MASK)) |\
((((unsigned long)(value)) << MLB_MAILBOX_SYSCONFIG_SIdleMode_OFFSET) &\
MLB_MAILBOX_SYSCONFIG_SIdleMode_MASK)))

#define MLBMAILBOX_SYSCONFIGSoftResetRead32(base_address)\
((((RD_MEM_32_VOLATILE((((unsigned long)(base_address))+\
(MLB_MAILBOX_SYSCONFIG_OFFSET)))) &\
MLB_MAILBOX_SYSCONFIG_SoftReset_MASK) >>\
MLB_MAILBOX_SYSCONFIG_SoftReset_OFFSET))

#define MLBMAILBOX_SYSCONFIGSoftResetReadIsb032(base_address)\
((MLBMAILBOX_SYSCONFIGSoftResetb0  ==  (MLBMAILBOX_SYSCONFIGSoftResetE)\
(((RD_MEM_32_VOLATILE((((unsigned long)(base_address))+\
(MLB_MAILBOX_SYSCONFIG_OFFSET)))) &\
MLB_MAILBOX_SYSCONFIG_SoftReset_MASK) >>\
MLB_MAILBOX_SYSCONFIG_SoftReset_OFFSET)))

#define MLBMAILBOX_SYSCONFIGSoftResetReadIsb132(base_address)\
((MLBMAILBOX_SYSCONFIGSoftResetb1 == (MLBMAILBOX_SYSCONFIGSoftResetE)\
(((RD_MEM_32_VOLATILE((((unsigned long)(base_address))+\
(MLB_MAILBOX_SYSCONFIG_OFFSET)))) &\
MLB_MAILBOX_SYSCONFIG_SoftReset_MASK) >>\
MLB_MAILBOX_SYSCONFIG_SoftReset_OFFSET)))

#define MLBMAILBOX_SYSCONFIGSoftResetGet32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_SYSCONFIG_SoftReset_MASK) >> \
MLB_MAILBOX_SYSCONFIG_SoftReset_OFFSET))

#define MLBMAILBOX_SYSCONFIGSoftResetIsb032(var)\
((MLBMAILBOX_SYSCONFIGSoftResetb0  ==  (MLBMAILBOX_SYSCONFIGSoftResetE)\
((((unsigned long)(var)) & MLB_MAILBOX_SYSCONFIG_SoftReset_MASK) >>\
MLB_MAILBOX_SYSCONFIG_SoftReset_OFFSET)))

#define MLBMAILBOX_SYSCONFIGSoftResetIsb132(var)\
((MLBMAILBOX_SYSCONFIGSoftResetb1 == (MLBMAILBOX_SYSCONFIGSoftResetE)\
((((unsigned long)(var)) & MLB_MAILBOX_SYSCONFIG_SoftReset_MASK) >>\
MLB_MAILBOX_SYSCONFIG_SoftReset_OFFSET)))

#define MLBMAILBOX_SYSCONFIGSoftResetWrite32(base_address, value)\
do {\
	const unsigned long offset = MLB_MAILBOX_SYSCONFIG_OFFSET;\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_SYSCONFIG_SoftReset_MASK);\
	newValue <<= MLB_MAILBOX_SYSCONFIG_SoftReset_OFFSET;\
	newValue &= MLB_MAILBOX_SYSCONFIG_SoftReset_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_SYSCONFIGSoftResetWriteb032(base_address)\
do {\
	const unsigned long offset = MLB_MAILBOX_SYSCONFIG_OFFSET;\
	const unsigned long newValue = \
	(unsigned long)MLBMAILBOX_SYSCONFIGSoftResetb0 <<\
	MLB_MAILBOX_SYSCONFIG_SoftReset_OFFSET;\
	register unsigned long data = \
	RD_MEM_32_VOLATILE((unsigned long)(base_address)+offset);\
	data &= ~(MLB_MAILBOX_SYSCONFIG_SoftReset_MASK);\
	data |= newValue;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, data);\
} while (0)

#define MLBMAILBOX_SYSCONFIGSoftResetWriteb132(base_address)\
do {\
	const unsigned long offset = MLB_MAILBOX_SYSCONFIG_OFFSET;\
	const unsigned long newValue = \
	(unsigned long)MLBMAILBOX_SYSCONFIGSoftResetb1 <<\
	MLB_MAILBOX_SYSCONFIG_SoftReset_OFFSET;\
	register unsigned long data = \
	RD_MEM_32_VOLATILE((unsigned long)(base_address)+offset);\
	data &= ~(MLB_MAILBOX_SYSCONFIG_SoftReset_MASK);\
	data |= newValue;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, data);\
} while (0)

#define MLBMAILBOX_SYSCONFIGSoftResetSet32(var, value)\
(((((unsigned long)(var)) & ~(MLB_MAILBOX_SYSCONFIG_SoftReset_MASK)) |\
((((unsigned long)(value)) << MLB_MAILBOX_SYSCONFIG_SoftReset_OFFSET) &\
MLB_MAILBOX_SYSCONFIG_SoftReset_MASK)))

#define MLBMAILBOX_SYSCONFIGAutoIdleRead32(base_address)\
((((RD_MEM_32_VOLATILE((((unsigned long)(base_address))+\
(MLB_MAILBOX_SYSCONFIG_OFFSET)))) &\
MLB_MAILBOX_SYSCONFIG_AutoIdle_MASK) >>\
MLB_MAILBOX_SYSCONFIG_AutoIdle_OFFSET))

#define MLBMAILBOX_SYSCONFIGAutoIdleReadIsb032(base_address)\
((MLBMAILBOX_SYSCONFIGAutoIdleb0 == (MLBMAILBOX_SYSCONFIGAutoIdleE)\
(((RD_MEM_32_VOLATILE((((unsigned long)(base_address))+\
(MLB_MAILBOX_SYSCONFIG_OFFSET)))) &\
MLB_MAILBOX_SYSCONFIG_AutoIdle_MASK) >>\
MLB_MAILBOX_SYSCONFIG_AutoIdle_OFFSET)))

#define MLBMAILBOX_SYSCONFIGAutoIdleReadIsb132(base_address)\
((MLBMAILBOX_SYSCONFIGAutoIdleb1 == (MLBMAILBOX_SYSCONFIGAutoIdleE)\
(((RD_MEM_32_VOLATILE((((unsigned long)(base_address))+\
(MLB_MAILBOX_SYSCONFIG_OFFSET)))) &\
MLB_MAILBOX_SYSCONFIG_AutoIdle_MASK) >>\
MLB_MAILBOX_SYSCONFIG_AutoIdle_OFFSET)))

#define MLBMAILBOX_SYSCONFIGAutoIdleGet32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_SYSCONFIG_AutoIdle_MASK) >> MLB_MAILBOX_SYSCONFIG_AutoIdle_OFFSET))

#define MLBMAILBOX_SYSCONFIGAutoIdleIsb032(var)\
((MLBMAILBOX_SYSCONFIGAutoIdleb0 == (MLBMAILBOX_SYSCONFIGAutoIdleE)\
((((unsigned long)(var)) & MLB_MAILBOX_SYSCONFIG_AutoIdle_MASK) >>\
MLB_MAILBOX_SYSCONFIG_AutoIdle_OFFSET)))

#define MLBMAILBOX_SYSCONFIGAutoIdleIsb132(var)\
((MLBMAILBOX_SYSCONFIGAutoIdleb1 == (MLBMAILBOX_SYSCONFIGAutoIdleE)\
((((unsigned long)(var)) & MLB_MAILBOX_SYSCONFIG_AutoIdle_MASK) >>\
MLB_MAILBOX_SYSCONFIG_AutoIdle_OFFSET)))

#define MLBMAILBOX_SYSCONFIGAutoIdleWrite32(base_address, value)\
do {\
	const unsigned long offset = MLB_MAILBOX_SYSCONFIG_OFFSET;\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_SYSCONFIG_AutoIdle_MASK);\
	newValue <<= MLB_MAILBOX_SYSCONFIG_AutoIdle_OFFSET;\
	newValue &= MLB_MAILBOX_SYSCONFIG_AutoIdle_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_SYSCONFIGAutoIdleWriteb032(base_address)\
do {\
	const unsigned long offset = MLB_MAILBOX_SYSCONFIG_OFFSET;\
	const unsigned long newValue = \
	(unsigned long)MLBMAILBOX_SYSCONFIGAutoIdleb0 <<\
	MLB_MAILBOX_SYSCONFIG_AutoIdle_OFFSET;\
	register unsigned long data = \
	RD_MEM_32_VOLATILE((unsigned long)(base_address)+offset);\
	data &= ~(MLB_MAILBOX_SYSCONFIG_AutoIdle_MASK);\
	data |= newValue;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, data);\
} while (0)

#define MLBMAILBOX_SYSCONFIGAutoIdleWriteb132(base_address)\
do {\
	const unsigned long offset = MLB_MAILBOX_SYSCONFIG_OFFSET;\
	const unsigned long newValue = \
	(unsigned long)MLBMAILBOX_SYSCONFIGAutoIdleb1 <<\
	MLB_MAILBOX_SYSCONFIG_AutoIdle_OFFSET;\
	register unsigned long data = \
	RD_MEM_32_VOLATILE((unsigned long)(base_address)+offset);\
	data &= ~(MLB_MAILBOX_SYSCONFIG_AutoIdle_MASK);\
	data |= newValue;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, data);\
} while (0)

#define MLBMAILBOX_SYSCONFIGAutoIdleSet32(var, value)\
(((((unsigned long)(var)) & ~(MLB_MAILBOX_SYSCONFIG_AutoIdle_MASK)) |\
((((unsigned long)(value)) << MLB_MAILBOX_SYSCONFIG_AutoIdle_OFFSET) &\
MLB_MAILBOX_SYSCONFIG_AutoIdle_MASK)))

#define MLBMAILBOX_SYSSTATUSReadRegister32(base_address)\
(RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
MLB_MAILBOX_SYSSTATUS_OFFSET))

#define MLBMAILBOX_SYSSTATUSResetDoneRead32(base_address)\
((((RD_MEM_32_VOLATILE((((unsigned long)(base_address))+\
(MLB_MAILBOX_SYSSTATUS_OFFSET)))) &\
MLB_MAILBOX_SYSSTATUS_ResetDone_MASK) >>\
MLB_MAILBOX_SYSSTATUS_ResetDone_OFFSET))

#define MLBMAILBOX_SYSSTATUSResetDoneReadisrstongoing32(base_address)\
((MLBMAILBOX_SYSSTATUSResetDonerstongoing == (MLBMAILBOX_SYSSTATUSResetDoneE)\
(((RD_MEM_32_VOLATILE((((unsigned long)(base_address))+\
(MLB_MAILBOX_SYSSTATUS_OFFSET)))) &\
MLB_MAILBOX_SYSSTATUS_ResetDone_MASK) >>\
MLB_MAILBOX_SYSSTATUS_ResetDone_OFFSET)))

#define MLBMAILBOX_SYSSTATUSResetDoneReadisrstcomp32(base_address)\
((MLBMAILBOX_SYSSTATUSResetDonerstcomp == (MLBMAILBOX_SYSSTATUSResetDoneE)\
(((RD_MEM_32_VOLATILE((((unsigned long)(base_address))+\
(MLB_MAILBOX_SYSSTATUS_OFFSET)))) &\
MLB_MAILBOX_SYSSTATUS_ResetDone_MASK) >>\
MLB_MAILBOX_SYSSTATUS_ResetDone_OFFSET)))

#define MLBMAILBOX_SYSSTATUSResetDoneGet32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_SYSSTATUS_ResetDone_MASK) >> \
MLB_MAILBOX_SYSSTATUS_ResetDone_OFFSET))

#define MLBMAILBOX_SYSSTATUSResetDoneisrstongoing32(var)\
((MLBMAILBOX_SYSSTATUSResetDonerstongoing  == (MLBMAILBOX_SYSSTATUSResetDoneE)\
((((unsigned long)(var)) & MLB_MAILBOX_SYSSTATUS_ResetDone_MASK) >>\
MLB_MAILBOX_SYSSTATUS_ResetDone_OFFSET)))

#define MLBMAILBOX_SYSSTATUSResetDoneisrstcomp32(var)\
((MLBMAILBOX_SYSSTATUSResetDonerstcomp == (MLBMAILBOX_SYSSTATUSResetDoneE)\
((((unsigned long)(var)) & MLB_MAILBOX_SYSSTATUS_ResetDone_MASK) >>\
MLB_MAILBOX_SYSSTATUS_ResetDone_OFFSET)))

#define MLBMAILBOX_MESSAGE___0_15ReadRegister32(base_address, bank)\
(RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_MESSAGE___REGSET_0_15_OFFSET +\
MLB_MAILBOX_MESSAGE___0_15_OFFSET+((bank)*\
MLB_MAILBOX_MESSAGE___REGSET_0_15_STEP))))

#define MLBMAILBOX_MESSAGE___0_15WriteRegister32(base_address, bank, value)\
do {\
	const unsigned long offset = MLB_MAILBOX_MESSAGE___REGSET_0_15_OFFSET +\
	MLB_MAILBOX_MESSAGE___0_15_OFFSET +\
	((bank) * MLB_MAILBOX_MESSAGE___REGSET_0_15_STEP);\
	register unsigned long newValue = ((unsigned long)(value));\
	WR_MEM_32_VOLATILE(((unsigned long)(base_address))+offset, newValue);\
} while (0)

#define MLBMAILBOX_MESSAGE___0_15MessageValueMBmRead32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_MESSAGE___REGSET_0_15_OFFSET +\
MLB_MAILBOX_MESSAGE___0_15_OFFSET+((bank)*\
MLB_MAILBOX_MESSAGE___REGSET_0_15_STEP)))) &\
MLB_MAILBOX_MESSAGE___0_15_MessageValueMBm_MASK) >>\
MLB_MAILBOX_MESSAGE___0_15_MessageValueMBm_OFFSET))

#define MLBMAILBOX_MESSAGE___0_15MessageValueMBmGet32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_MESSAGE___0_15_MessageValueMBm_MASK) >> \
MLB_MAILBOX_MESSAGE___0_15_MessageValueMBm_OFFSET))

#define MLBMAILBOX_MESSAGE___0_15MessageValueMBmWrite32\
(base_address, bank, value) do {\
	const unsigned long offset = \
	MLB_MAILBOX_MESSAGE___REGSET_0_15_OFFSET +\
	MLB_MAILBOX_MESSAGE___0_15_OFFSET +\
	((bank) * MLB_MAILBOX_MESSAGE___REGSET_0_15_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_MESSAGE___0_15_MessageValueMBm_MASK);\
	newValue <<= MLB_MAILBOX_MESSAGE___0_15_MessageValueMBm_OFFSET;\
	newValue &= MLB_MAILBOX_MESSAGE___0_15_MessageValueMBm_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_MESSAGE___0_15MessageValueMBmSet32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_MESSAGE___0_15_MessageValueMBm_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_MESSAGE___0_15_MessageValueMBm_OFFSET) &\
MLB_MAILBOX_MESSAGE___0_15_MessageValueMBm_MASK)))

#define MLBMAILBOX_FIFOSTATUS___0_15ReadRegister32(base_address, bank)\
(RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_FIFOSTATUS___REGSET_0_15_OFFSET +\
MLB_MAILBOX_FIFOSTATUS___0_15_OFFSET+((bank)*\
MLB_MAILBOX_FIFOSTATUS___REGSET_0_15_STEP))))

#define MLBMAILBOX_FIFOSTATUS___0_15FifoFullMBmRead32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_FIFOSTATUS___REGSET_0_15_OFFSET +\
MLB_MAILBOX_FIFOSTATUS___0_15_OFFSET+((bank)*\
MLB_MAILBOX_FIFOSTATUS___REGSET_0_15_STEP)))) &\
MLB_MAILBOX_FIFOSTATUS___0_15_FifoFullMBm_MASK) >>\
MLB_MAILBOX_FIFOSTATUS___0_15_FifoFullMBm_OFFSET))

#define MLBMAILBOX_FIFOSTATUS___0_15FifoFullMBmGet32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_FIFOSTATUS___0_15_FifoFullMBm_MASK) >> \
MLB_MAILBOX_FIFOSTATUS___0_15_FifoFullMBm_OFFSET))

#define MLBMAILBOX_MSGSTATUS___0_15ReadRegister32(base_address, bank)\
(RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_MSGSTATUS___REGSET_0_15_OFFSET +\
MLB_MAILBOX_MSGSTATUS___0_15_OFFSET+((bank)*\
MLB_MAILBOX_MSGSTATUS___REGSET_0_15_STEP))))

#define MLBMAILBOX_MSGSTATUS___0_15NbOfMsgMBmRead32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_MSGSTATUS___REGSET_0_15_OFFSET +\
MLB_MAILBOX_MSGSTATUS___0_15_OFFSET+((bank)*\
MLB_MAILBOX_MSGSTATUS___REGSET_0_15_STEP)))) &\
MLB_MAILBOX_MSGSTATUS___0_15_NbOfMsgMBm_MASK) >>\
MLB_MAILBOX_MSGSTATUS___0_15_NbOfMsgMBm_OFFSET))

#define MLBMAILBOX_MSGSTATUS___0_15NbOfMsgMBmGet32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_MSGSTATUS___0_15_NbOfMsgMBm_MASK) \
>> MLB_MAILBOX_MSGSTATUS___0_15_NbOfMsgMBm_OFFSET))

#if defined(OMAP44XX) || defined(VPOM4430_1_06)

#define MLBMAILBOX_IRQSTATUS_CLR___0_3ReadRegister32(base_address, bank)\
(RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS_CLR___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS_CLR___REGSET_0_3_STEP))))

#else

#define MLBMAILBOX_IRQSTATUS___0_3ReadRegister32(base_address, bank)\
(RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP))))

#endif

#if defined(OMAP44XX) || defined(VPOM4430_1_06)

#define MLBMAILBOX_IRQSTATUS_CLR___0_3WriteRegister32\
(base_address, bank, value) do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS_CLR___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * \
	MLB_MAILBOX_IRQSTATUS_CLR___REGSET_0_3_STEP);\
	register unsigned long newValue = ((unsigned long)(value));\
	WR_MEM_32_VOLATILE(((unsigned long)(base_address))+ \
	offset, newValue);\
} while (0)

#else

#define MLBMAILBOX_IRQSTATUS___0_3WriteRegister32(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long newValue = ((unsigned long)(value));\
	WR_MEM_32_VOLATILE(((unsigned long)(base_address))+offset, newValue);\
} while (0)

#endif

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB15Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB15_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB15_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB15Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB15_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB15_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB15Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB15_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB15_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB15_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB15Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB15_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB15_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB15_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB15Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB15_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB15_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB15Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB15_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB15_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB15Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB15_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB15_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB15_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB15Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB15_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB15_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB15_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB14Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+\
((bank)*MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB14_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB14_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB14Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB14_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB14_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB14Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB14_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB14_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB14_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB14Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB14_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB14_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB14_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB14Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB14_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB14_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB14Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB14_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB14_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB14Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB14_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB14_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB14_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB14Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB14_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB14_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB14_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB13Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB13_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB13_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB13Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB13_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB13_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB13Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB13_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB13_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB13_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB13Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB13_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB13_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB13_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB13Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB13_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB13_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB13Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB13_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB13_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB13Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB13_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB13_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB13_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB13Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB13_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB13_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB13_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB12Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB12_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB12_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB12Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB12_MASK) >> \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB12_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB12Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB12_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB12_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB12_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB12Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB12_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB12_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB12_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB12Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB12_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB12_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB12Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB12_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB12_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB12Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB12_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB12_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB12_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB12Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB12_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB12_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB12_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB11Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB11_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB11_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB11Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB11_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB11_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB11Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB11_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB11_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB11_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB11Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB11_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB11_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB11_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB11Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB11_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB11_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB11Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB11_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB11_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB11Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB11_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB11_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB11_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB11Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB11_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB11_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB11_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB10Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB10_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB10_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB10Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB10_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB10_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB10Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB10_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB10_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB10_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB10Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB10_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB10_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB10_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB10Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB10_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB10_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB10Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB10_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB10_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB10Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB10_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB10_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB10_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB10Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB10_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB10_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB10_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB9Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB9_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB9_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB9Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB9_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB9_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB9Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB9_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB9_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB9_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB9Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB9_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB9_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB9_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB9Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB9_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB9_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB9Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB9_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB9_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB9Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB9_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB9_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB9_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB9Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB9_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB9_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB9_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB8Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB8_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB8_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB8Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB8_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB8_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB8Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB8_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB8_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB8_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB8Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB8_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB8_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB8_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB8Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB8_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB8_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB8Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB8_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB8_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB8Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB8_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB8_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB8_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB8Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB8_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB8_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB8_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB7Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB7_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB7_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB7Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB7_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB7_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB7Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB7_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB7_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB7_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB7Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB7_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB7_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB7_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB7Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB7_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB7_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB7Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB7_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB7_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB7Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB7_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB7_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB7_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB7Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB7_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB7_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB7_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB6Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB6_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB6_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB6Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB6_MASK) >> \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB6_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB6Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB6_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB6_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB6_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB6Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB6_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB6_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB6_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB6Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB6_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB6_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB6Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB6_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB6_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB6Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB6_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB6_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB6_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB6Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB6_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB6_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB6_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB5Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+\
((bank)*MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB5_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB5_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB5Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB5_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB5_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB5Write32\
(base_address, bank, value)\
do {\
	onst unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB5_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB5_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB5_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB5Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB5_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB5_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB5_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB5Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB5_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB5_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB5Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB5_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB5_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB5Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB5_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB5_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB5_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB5Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB5_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB5_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB5_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB4Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB4_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB4_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB4Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB4_MASK) >> \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB4_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB4Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB4_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB4_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB4_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB4Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB4_MASK)) |\
((((unsigned long)(value)) << \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB4_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB4_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB4Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB4_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB4_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB4Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB4_MASK) >> \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB4_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB4Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB4_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB4_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB4_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB4Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB4_MASK)) |\
((((unsigned long)(value)) << \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB4_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB4_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB3Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))\
+(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)\
*MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB3_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB3_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB3Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB3_MASK) >> \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB3_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB3Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB3_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB3_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB3_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB3Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB3_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB3_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB3_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB3Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB3_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB3_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB3Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB3_MASK) >> \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB3_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB3Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB3_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB3_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB3_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB3Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB3_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB3_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB3_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB2Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB2_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB2_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB2Get32(var)\
((unsigned long)((((unsigned long)(var)) \
& MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB2_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB2_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB2Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB2_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB2_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB2_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB2Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB2_MASK)) |\
((((unsigned long)(value)) << \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB2_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB2_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB2Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB2_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB2_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB2Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB2_MASK) \
>> MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB2_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB2Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB2_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB2_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB2_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB2Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB2_MASK)) |\
((((unsigned long)(value)) << \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB2_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB2_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB1Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+\
((bank)*MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB1_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB1_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB1Get32(var)\
((unsigned long)((((unsigned long)(var)) \
& MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB1_MASK) >> \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB1_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB1Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = RD_\
	MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB1_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB1_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB1_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB1Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB1_MASK)) |\
((((unsigned long)(value)) << \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB1_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB1_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB1Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB1_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB1_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB1Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB1_MASK) >> \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB1_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB1Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB1_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB1_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB1_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB1Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB1_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB1_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB1_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB0Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB0_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB0_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB0Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB0_MASK) >> \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB0_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB0Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB0_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB0_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB0_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NotFullStatusUuMB0Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB0_MASK)) |\
((((unsigned long)(value)) << \
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB0_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB0_MASK)))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB0Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQSTATUS___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB0_MASK) >>\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB0_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB0Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB0_MASK) >> \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB0_OFFSET))

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB0Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQSTATUS___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB0_MASK);\
	newValue <<= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB0_OFFSET;\
	newValue &= MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB0_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQSTATUS___0_3NewMsgStatusUuMB0Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB0_MASK)) |\
((((unsigned long)(value)) << \
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB0_OFFSET) &\
MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB0_MASK)))

#if defined(OMAP44XX) || defined(VPOM4430_1_06)

#define MLBMAILBOX_IRQENABLE_SET___0_3ReadRegister32(base_address, bank)\
(RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE_SET___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE_SET___REGSET_0_3_STEP))))

#else

#define MLBMAILBOX_IRQENABLE___0_3ReadRegister32(base_address, bank)\
(RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP))))

#endif

#if defined(OMAP44XX) || defined(VPOM4430_1_06)

#define MLBMAILBOX_IRQENABLE_SET___0_3WriteRegister32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE_SET___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * \
	MLB_MAILBOX_IRQENABLE_SET___REGSET_0_3_STEP);\
	register unsigned long newValue = ((unsigned long)(value));\
	WR_MEM_32_VOLATILE(((unsigned long)(base_address))+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE_CLR___0_3WriteRegister32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE_CLR___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * \
	MLB_MAILBOX_IRQENABLE_CLR___REGSET_0_3_STEP);\
	register unsigned long newValue = ((unsigned long)(value));\
	WR_MEM_32_VOLATILE(((unsigned long)(base_address))+offset, newValue);\
} while (0)

#else

#define MLBMAILBOX_IRQENABLE___0_3WriteRegister32(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long newValue = ((unsigned long)(value));\
	WR_MEM_32_VOLATILE(((unsigned long)(base_address))+offset, newValue);\
} while (0)

#endif

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB15Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB15_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB15_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB15Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB15_MASK) >> \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB15_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB15Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB15_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB15_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB15_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB15Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB15_MASK)) |\
((((unsigned long)(value)) << \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB15_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB15_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB15Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB15_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB15_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB15Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB15_MASK) >> \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB15_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB15Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB15_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB15_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB15_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB15Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB15_MASK)) |\
((((unsigned long)(value)) << \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB15_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB15_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB14Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB14_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB14_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB14Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB14_MASK) \
>> MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB14_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB14Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB14_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB14_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB14_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB14Set32\
(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB14_MASK)) |\
((((unsigned long)(value)) << \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB14_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB14_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB14Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB14_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB14_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB14Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB14_MASK) \
>> MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB14_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB14Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB14_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB14_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB14_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB14Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB14_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB14_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB14_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB13Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB13_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB13_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB13Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB13_MASK) >> \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB13_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB13Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB13_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB13_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB13_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB13Set32\
(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB13_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB13_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB13_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB13Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB13_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB13_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB13Get32(var)\
((unsigned long)((((unsigned long)(var)) \
& MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB13_MASK) \
>> MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB13_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB13Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB13_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB13_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB13_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB13Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB13_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB13_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB13_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB12Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB12_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB12_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB12Get32(var)\
((unsigned long)((((unsigned long)(var)) \
& MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB12_MASK) \
>> MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB12_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB12Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB12_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB12_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB12_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB12Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB12_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB12_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB12_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB12Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB12_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB12_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB12Get32(var)\
((unsigned long)((((unsigned long)(var)) \
& MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB12_MASK) \
>> MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB12_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB12Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB12_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB12_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB12_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB12Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB12_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB12_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB12_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB11Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB11_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB11_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB11Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB11_MASK) \
>> MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB11_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB11Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB11_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB11_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB11_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB11Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB11_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB11_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB11_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB11Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB11_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB11_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB11Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB11_MASK) \
>> MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB11_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB11Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB11_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB11_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB11_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB11Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB11_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB11_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB11_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB10Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB10_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB10_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB10Get32(var)\
((unsigned long)((((unsigned long)(var)) \
& MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB10_MASK) \
>> MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB10_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB10Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = \
	((unsigned long)(value));\
	data &= \
	~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB10_MASK);\
	newValue <<= \
	MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB10_OFFSET;\
	newValue \
	&= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB10_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB10Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB10_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB10_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB10_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB10Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+\
((bank)*MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB10_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB10_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB10Get32(var)\
((unsigned long)((((unsigned long)(var)) \
& MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB10_MASK) \
>> MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB10_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB10Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = \
	((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB10_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB10_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB10_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB10Set32\
(var, value)\
(((((unsigned long)(var)) &\
~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB10_MASK)) |\
((((unsigned long)(value)) << \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB10_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB10_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB9Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB9_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB9_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB9Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB9_MASK) >> \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB9_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB9Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB9_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB9_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB9_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB9Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB9_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB9_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB9_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB9Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB9_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB9_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB9Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB9_MASK) >> \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB9_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB9Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB9_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB9_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB9_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB9Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB9_MASK)) |\
((((unsigned long)(value)) << \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB9_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB9_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB8Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))\
+(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB8_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB8_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB8Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB8_MASK) \
>> MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB8_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB8Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB8_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB8_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB8_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB8Set32(var, value)\
(((((unsigned long)(var)) & \
~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB8_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB8_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB8_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB8Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB8_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB8_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB8Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB8_MASK) >> \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB8_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB8Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB8_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB8_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB8_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB8Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB8_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB8_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB8_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB7Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB7_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB7_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB7Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB7_MASK) \
>> MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB7_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB7Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB7_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB7_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB7_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB7Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB7_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB7_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB7_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB7Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB7_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB7_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB7Get32(var)\
((unsigned long)((((unsigned long)(var)) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB7_MASK) >> \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB7_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB7Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB7_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB7_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB7_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB7Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB7_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB7_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB7_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB6Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB6_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB6_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB6Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB6_MASK) >> \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB6_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB6Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB6_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB6_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB6_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB6Set32\
(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB6_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB6_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB6_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB6Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB6_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB6_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB6Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB6_MASK) >> \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB6_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB6Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB6_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB6_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB6_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB6Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB6_MASK)) |\
((((unsigned long)(value)) << \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB6_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB6_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB5Read32(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB5_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB5_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB5Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB5_MASK) >> \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB5_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB5Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB5_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB5_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB5_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB5Set32\
(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB5_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB5_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB5_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB5Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))\
+(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB5_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB5_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB5Get32(var)\
((unsigned long)((((unsigned long)(var)) \
& MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB5_MASK) >> \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB5_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB5Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB5_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB5_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB5_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB5Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB5_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB5_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB5_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB4Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))\
+(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB4_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB4_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB4Get32\
(var)\
((unsigned long)((((unsigned long)(var)) \
& MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB4_MASK) >> \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB4_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB4Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB4_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB4_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB4_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB4Set32\
(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB4_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB4_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB4_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB4Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB4_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB4_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB4Get32(var)\
((unsigned long)((((unsigned long)(var)) \
& MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB4_MASK) \
>> MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB4_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB4Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB4_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB4_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB4_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB4Set32\
(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB4_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB4_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB4_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB3Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))\
+(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)\
*MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB3_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB3_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB3Get32\
(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB3_MASK) >> \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB3_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB3Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB3_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB3_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB3_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB3Set32\
(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB3_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB3_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB3_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB3Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB3_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB3_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB3Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB3_MASK) \
>> MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB3_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB3Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB3_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB3_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB3_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB3Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB3_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB3_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB3_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB2Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB2_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB2_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB2Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB2_MASK) >> \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB2_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB2Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB2_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB2_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB2_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB2Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB2_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB2_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB2_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB2Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB2_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB2_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB2Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB2_MASK) >> \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB2_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB2Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB2_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB2_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB2_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB2Set32\
(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB2_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB2_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB2_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB1Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))\
+(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)\
*MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB1_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB1_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB1Get32(var)\
((unsigned long)((((unsigned long)(var)) \
& MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB1_MASK) \
>> MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB1_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB1Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB1_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB1_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB1_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB1Set32\
(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB1_MASK)) |\
((((unsigned long)(value)) << \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB1_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB1_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB1Read32\
(base_address, bank) do {\
	((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
	(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
	MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB1_MASK) >>\
	MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB1_OFFSET))\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB1Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB1_MASK) >> \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB1_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB1Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB1_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB1_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB1_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB1Set32(var, value)\
(((((unsigned long)(var)) &\
~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB1_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB1_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB1_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB0Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB0_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB0_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB0Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB0_MASK) \
>> MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB0_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB0Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB0_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB0_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB0_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NotFullEnableUuMB0Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB0_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB0_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB0_MASK)))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB0Read32\
(base_address, bank)\
((((RD_MEM_32_VOLATILE(((unsigned long)(base_address))+\
(MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
MLB_MAILBOX_IRQENABLE___0_3_OFFSET+((bank)*\
MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP)))) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB0_MASK) >>\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB0_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB0Get32(var)\
((unsigned long)((((unsigned long)(var)) & \
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB0_MASK) \
>> MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB0_OFFSET))

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB0Write32\
(base_address, bank, value)\
do {\
	const unsigned long offset = \
	MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET +\
	MLB_MAILBOX_IRQENABLE___0_3_OFFSET +\
	((bank) * MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP);\
	register unsigned long data = \
	RD_MEM_32_VOLATILE(((unsigned long)(base_address))+offset);\
	register unsigned long newValue = ((unsigned long)(value));\
	data &= ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB0_MASK);\
	newValue <<= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB0_OFFSET;\
	newValue &= MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB0_MASK;\
	newValue |= data;\
	WR_MEM_32_VOLATILE((unsigned long)(base_address)+offset, newValue);\
} while (0)

#define MLBMAILBOX_IRQENABLE___0_3NewMsgEnableUuMB0Set32(var, value)\
(((((unsigned long)(var)) \
& ~(MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB0_MASK)) |\
((((unsigned long)(value)) \
<< MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB0_OFFSET) &\
MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB0_MASK)))

#endif    /* USE_LEVEL_1_MACROS */


#endif /* _MBX_REG_ACM_H */
