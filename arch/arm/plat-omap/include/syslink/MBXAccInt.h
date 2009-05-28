/*
 * MBXAccInt.h
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


#ifndef _MLB_ACC_INT_H
#define _MLB_ACC_INT_H


/*
 * EXPORTED DEFINITIONS
 *
 */


#define MLB_MAILBOX_MESSAGE___REGSET_0_15_OFFSET (unsigned long)(0x0040)
#define MLB_MAILBOX_MESSAGE___REGSET_0_15_STEP   (unsigned long)(0x0004)
#define MLB_MAILBOX_MESSAGE___REGSET_0_15_BANKS  (unsigned long)(16)

/* Register offset address definitions relative
to register set MAILBOX_MESSAGE___REGSET_0_15 */

#define MLB_MAILBOX_MESSAGE___0_15_OFFSET (unsigned long)(0x0)


/* Register set MAILBOX_FIFOSTATUS___REGSET_0_15
address offset, bank address increment and number of banks */

#define MLB_MAILBOX_FIFOSTATUS___REGSET_0_15_OFFSET (unsigned long)(0x0080)
#define MLB_MAILBOX_FIFOSTATUS___REGSET_0_15_STEP   (unsigned long)(0x0004)
#define MLB_MAILBOX_FIFOSTATUS___REGSET_0_15_BANKS (unsigned long)(16)

/* Register offset address definitions relative to
register set MAILBOX_FIFOSTATUS___REGSET_0_15 */

#define MLB_MAILBOX_FIFOSTATUS___0_15_OFFSET  (unsigned long)(0x0)


/* Register set MAILBOX_MSGSTATUS___REGSET_0_15
address offset, bank address increment and number of banks */

#define MLB_MAILBOX_MSGSTATUS___REGSET_0_15_OFFSET (unsigned long)(0x00c0)
#define MLB_MAILBOX_MSGSTATUS___REGSET_0_15_STEP   (unsigned long)(0x0004)
#define MLB_MAILBOX_MSGSTATUS___REGSET_0_15_BANKS  (unsigned long)(16)

/* Register offset address definitions relative to
register set MAILBOX_MSGSTATUS___REGSET_0_15 */

#define MLB_MAILBOX_MSGSTATUS___0_15_OFFSET  (unsigned long)(0x0)


/*Register set MAILBOX_IRQSTATUS___REGSET_0_3 address
offset, bank address increment and number of banks */

#define MLB_MAILBOX_IRQSTATUS___REGSET_0_3_OFFSET (unsigned long)(0x0100)
#define MLB_MAILBOX_IRQSTATUS___REGSET_0_3_STEP (unsigned long)(0x0008)
#define MLB_MAILBOX_IRQSTATUS___REGSET_0_3_BANKS (unsigned long)(4)

#define MLB_MAILBOX_IRQSTATUS_CLR___REGSET_0_3_OFFSET  (unsigned long)(0x0104)
#define MLB_MAILBOX_IRQSTATUS_CLR___REGSET_0_3_STEP    (unsigned long)(0x0010)
#define MLB_MAILBOX_IRQSTATUS_CLR___REGSET_0_3_BANKS   (unsigned long)(4)

/* Register offset address definitions relative to
register set MAILBOX_IRQSTATUS___REGSET_0_3 */

#define MLB_MAILBOX_IRQSTATUS___0_3_OFFSET (unsigned long)(0x0)


/* Register set MAILBOX_IRQENABLE___REGSET_0_3
address offset, bank address increment and number of banks */

#define MLB_MAILBOX_IRQENABLE___REGSET_0_3_OFFSET (unsigned long)(0x0104)
#define MLB_MAILBOX_IRQENABLE___REGSET_0_3_STEP (unsigned long)(0x0008)
#define MLB_MAILBOX_IRQENABLE___REGSET_0_3_BANKS (unsigned long)(4)

#define MLB_MAILBOX_IRQENABLE_SET___REGSET_0_3_OFFSET  (unsigned long)(0x0108)
#define MLB_MAILBOX_IRQENABLE_SET___REGSET_0_3_STEP    (unsigned long)(0x0010)
#define MLB_MAILBOX_IRQENABLE_SET___REGSET_0_3_BANKS   (unsigned long)(4)

#define MLB_MAILBOX_IRQENABLE_CLR___REGSET_0_3_OFFSET   (unsigned long)(0x010C)
#define MLB_MAILBOX_IRQENABLE_CLR___REGSET_0_3_STEP     (unsigned long)(0x0010)
#define MLB_MAILBOX_IRQENABLE_CLR___REGSET_0_3_BANKS    (unsigned long)(4)

/* Register offset address definitions relative to register
set MAILBOX_IRQENABLE___REGSET_0_3 */

#define MLB_MAILBOX_IRQENABLE___0_3_OFFSET (unsigned long)(0x0)


/* Register offset address definitions */

#define MLB_MAILBOX_REVISION_OFFSET  (unsigned long)(0x0)
#define MLB_MAILBOX_SYSCONFIG_OFFSET (unsigned long)(0x10)
#define MLB_MAILBOX_SYSSTATUS_OFFSET (unsigned long)(0x14)


/* Bitfield mask and offset declarations */

#define MLB_MAILBOX_REVISION_Rev_MASK  (unsigned long)(0xff)
#define MLB_MAILBOX_REVISION_Rev_OFFSET  (unsigned long)(0)

#define MLB_MAILBOX_SYSCONFIG_ClockActivity_MASK  (unsigned long)(0x100)
#define MLB_MAILBOX_SYSCONFIG_ClockActivity_OFFSET (unsigned long)(8)

#define MLB_MAILBOX_SYSCONFIG_SIdleMode_MASK  (unsigned long)(0x18)
#define MLB_MAILBOX_SYSCONFIG_SIdleMode_OFFSET (unsigned long)(3)

#define MLB_MAILBOX_SYSCONFIG_SoftReset_MASK   (unsigned long)(0x2)
#define MLB_MAILBOX_SYSCONFIG_SoftReset_OFFSET (unsigned long)(1)

#define MLB_MAILBOX_SYSCONFIG_AutoIdle_MASK    (unsigned long)(0x1)
#define MLB_MAILBOX_SYSCONFIG_AutoIdle_OFFSET  (unsigned long)(0)

#define MLB_MAILBOX_SYSSTATUS_ResetDone_MASK   (unsigned long)(0x1)
#define MLB_MAILBOX_SYSSTATUS_ResetDone_OFFSET (unsigned long)(0)

#define MLB_MAILBOX_MESSAGE___0_15_MessageValueMBm_MASK  \
(unsigned long)(0xffffffff)

#define MLB_MAILBOX_MESSAGE___0_15_MessageValueMBm_OFFSET (unsigned long)(0)

#define MLB_MAILBOX_FIFOSTATUS___0_15_FifoFullMBm_MASK  (unsigned long)(0x1)
#define MLB_MAILBOX_FIFOSTATUS___0_15_FifoFullMBm_OFFSET (unsigned long)(0)

#define MLB_MAILBOX_MSGSTATUS___0_15_NbOfMsgMBm_MASK   (unsigned long)(0x7f)
#define MLB_MAILBOX_MSGSTATUS___0_15_NbOfMsgMBm_OFFSET (unsigned long)(0)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB15_MASK \
(unsigned long)(0x80000000)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB15_OFFSET \
(unsigned long)(31)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB15_MASK  \
(unsigned long)(0x40000000)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB15_OFFSET \
(unsigned long)(30)


#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB14_MASK \
(unsigned long)(0x20000000)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB14_OFFSET \
(unsigned long)(29)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB14_MASK  \
(unsigned long)(0x10000000)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB14_OFFSET \
(unsigned long)(28)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB13_MASK  \
(unsigned long)(0x8000000)
#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB13_OFFSET \
(unsigned long)(27)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB13_MASK   \
(unsigned long)(0x4000000)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB13_OFFSET  \
(unsigned long)(26)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB12_MASK   \
(unsigned long)(0x2000000)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB12_OFFSET \
(unsigned long)(25)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB12_MASK \
(unsigned long)(0x1000000)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB12_OFFSET \
(unsigned long)(24)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB11_MASK  \
(unsigned long)(0x800000)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB11_OFFSET \
(unsigned long)(23)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB11_MASK    \
(unsigned long)(0x400000)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB11_OFFSET  \
(unsigned long)(22)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB10_MASK    \
(unsigned long)(0x200000)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB10_OFFSET  \
(unsigned long)(21)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB10_MASK \
(unsigned long)(0x100000)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB10_OFFSET \
(unsigned long)(20)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB9_MASK \
(unsigned long)(0x80000)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB9_OFFSET \
(unsigned long)(19)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB9_MASK \
(unsigned long)(0x40000)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB9_OFFSET \
(unsigned long)(18)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB8_MASK \
(unsigned long)(0x20000)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB8_OFFSET \
(unsigned long)(17)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB8_MASK  \
(unsigned long)(0x10000)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB8_OFFSET  \
(unsigned long)(16)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB7_MASK   \
(unsigned long)(0x8000)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB7_OFFSET  \
(unsigned long)(15)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB7_MASK     \
(unsigned long)(0x4000)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB7_OFFSET \
(unsigned long)(14)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB6_MASK  \
(unsigned long)(0x2000)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB6_OFFSET \
(unsigned long)(13)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB6_MASK \
(unsigned long)(0x1000)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB6_OFFSET \
(unsigned long)(12)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB5_MASK  \
(unsigned long)(0x800)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB5_OFFSET \
(unsigned long)(11)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB5_MASK   \
(unsigned long)(0x400)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB5_OFFSET \
(unsigned long)(10)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB4_MASK  \
(unsigned long)(0x200)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB4_OFFSET  \
(unsigned long)(9)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB4_MASK \
(unsigned long)(0x100)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB4_OFFSET \
(unsigned long)(8)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB3_MASK  \
(unsigned long)(0x80)
#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB3_OFFSET \
(unsigned long)(7)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB3_MASK  \
(unsigned long)(0x40)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB3_OFFSET \
(unsigned long)(6)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB2_MASK  \
(unsigned long)(0x20)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB2_OFFSET  \
(unsigned long)(5)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB2_MASK \
(unsigned long)(0x10)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB2_OFFSET \
(unsigned long)(4)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB1_MASK  \
(unsigned long)(0x8)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB1_OFFSET  \
(unsigned long)(3)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB1_MASK  \
(unsigned long)(0x4)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB1_OFFSET \
(unsigned long)(2)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB0_MASK  \
(unsigned long)(0x2)

#define MLB_MAILBOX_IRQSTATUS___0_3_NotFullStatusUuMB0_OFFSET \
(unsigned long)(1)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB0_MASK    \
(unsigned long)(0x1)

#define MLB_MAILBOX_IRQSTATUS___0_3_NewMsgStatusUuMB0_OFFSET  \
(unsigned long)(0)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB15_MASK  \
(unsigned long)(0x80000000)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB15_OFFSET  \
(unsigned long)(31)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB15_MASK \
(unsigned long)(0x40000000)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB15_OFFSET \
(unsigned long)(30)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB14_MASK \
(unsigned long)(0x20000000)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB14_OFFSET \
(unsigned long)(29)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB14_MASK \
(unsigned long)(0x10000000)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB14_OFFSET \
(unsigned long)(28)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB13_MASK \
(unsigned long)(0x8000000)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB13_OFFSET \
(unsigned long)(27)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB13_MASK \
(unsigned long)(0x4000000)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB13_OFFSET \
(unsigned long)(26)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB12_MASK  \
(unsigned long)(0x2000000)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB12_OFFSET \
(unsigned long)(25)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB12_MASK  \
(unsigned long)(0x1000000)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB12_OFFSET \
(unsigned long)(24)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB11_MASK  \
(unsigned long)(0x800000)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB11_OFFSET \
(unsigned long)(23)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB11_MASK \
(unsigned long)(0x400000)
#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB11_OFFSET \
(unsigned long)(22)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB10_MASK \
(unsigned long)(0x200000)
#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB10_OFFSET \
(unsigned long)(21)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB10_MASK  \
(unsigned long)(0x100000)
#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB10_OFFSET \
(unsigned long)(20)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB9_MASK  \
(unsigned long)(0x80000)
#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB9_OFFSET \
(unsigned long)(19)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB9_MASK \
(unsigned long)(0x40000)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB9_OFFSET \
(unsigned long)(18)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB8_MASK \
(unsigned long)(0x20000)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB8_OFFSET \
(unsigned long)(17)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB8_MASK \
(unsigned long)(0x10000)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB8_OFFSET \
(unsigned long)(16)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB7_MASK \
(unsigned long)(0x8000)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB7_OFFSET \
(unsigned long)(15)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB7_MASK \
(unsigned long)(0x4000)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB7_OFFSET \
(unsigned long)(14)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB6_MASK \
(unsigned long)(0x2000)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB6_OFFSET \
(unsigned long)(13)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB6_MASK \
(unsigned long)(0x1000)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB6_OFFSET \
(unsigned long)(12)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB5_MASK \
(unsigned long)(0x800)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB5_OFFSET \
(unsigned long)(11)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB5_MASK \
(unsigned long)(0x400)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB5_OFFSET \
(unsigned long)(10)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB4_MASK  \
(unsigned long)(0x200)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB4_OFFSET  \
(unsigned long)(9)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB4_MASK   \
(unsigned long)(0x100)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB4_OFFSET  \
(unsigned long)(8)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB3_MASK  \
(unsigned long)(0x80)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB3_OFFSET \
(unsigned long)(7)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB3_MASK  \
(unsigned long)(0x40)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB3_OFFSET  \
(unsigned long)(6)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB2_MASK  \
(unsigned long)(0x20)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB2_OFFSET \
(unsigned long)(5)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB2_MASK  \
(unsigned long)(0x10)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB2_OFFSET \
(unsigned long)(4)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB1_MASK \
(unsigned long)(0x8)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB1_OFFSET \
(unsigned long)(3)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB1_MASK \
(unsigned long)(0x4)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB1_OFFSET \
(unsigned long)(2)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB0_MASK \
(unsigned long)(0x2)

#define MLB_MAILBOX_IRQENABLE___0_3_NotFullEnableUuMB0_OFFSET \
(unsigned long)(1)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB0_MASK \
(unsigned long)(0x1)

#define MLB_MAILBOX_IRQENABLE___0_3_NewMsgEnableUuMB0_OFFSET \
(unsigned long)(0)


/*
 * EXPORTED TYPES
 *
 */

/* The following type definitions
represent the enumerated values for each bitfield */

enum MLBMAILBOX_SYSCONFIGSIdleModeE {
	MLBMAILBOX_SYSCONFIGSIdleModeb00 = 0x0000,
	MLBMAILBOX_SYSCONFIGSIdleModeb01 = 0x0001,
	MLBMAILBOX_SYSCONFIGSIdleModeb10 = 0x0002,
	MLBMAILBOX_SYSCONFIGSIdleModeb11 = 0x0003
};

enum MLBMAILBOX_SYSCONFIGSoftResetE {
	MLBMAILBOX_SYSCONFIGSoftResetb0 = 0x0000,
	MLBMAILBOX_SYSCONFIGSoftResetb1 = 0x0001
};

enum MLBMAILBOX_SYSCONFIGAutoIdleE {
	MLBMAILBOX_SYSCONFIGAutoIdleb0 = 0x0000,
	MLBMAILBOX_SYSCONFIGAutoIdleb1 = 0x0001
};

enum MLBMAILBOX_SYSSTATUSResetDoneE {
	MLBMAILBOX_SYSSTATUSResetDonerstongoing = 0x0000,
	MLBMAILBOX_SYSSTATUSResetDonerstcomp = 0x0001
};

#endif /* _MLB_ACC_INT_H */
