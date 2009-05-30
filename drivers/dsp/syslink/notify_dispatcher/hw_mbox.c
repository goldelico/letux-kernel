/*
 * hw_mbox.c
 *
 * Syslink dispatcher driver support for OMAP Processors.
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


#include <syslink/GlobalTypes.h>
#include <syslink/MBXRegAcM.h>
#include <syslink/MBXAccInt.h>
#include <syslink/hw_defs.h>
#include <syslink/hw_mbox.h>
#include<linux/module.h>

#if defined(OMAP3430)
struct mailbox_context mboxsetting = {0, 0, 0};
/*
 * func:  HAL_MBOX_saveSettings
 * purpose:
 * Saves the mailbox context
 */
long hw_mbox_save_settings(unsigned long    base_address)
{
	long status = RET_OK;

	mboxsetting.sysconfig =
	MLBMAILBOX_SYSCONFIGReadRegister32(base_address);

	/* Get current enable status */
	mboxsetting.irqEnable0 = MLBMAILBOX_IRQENABLE___0_3ReadRegister32
		(base_address, HW_MBOX_U0_ARM11);
	mboxsetting.irqEnable1 = MLBMAILBOX_IRQENABLE___0_3ReadRegister32
		(base_address, HW_MBOX_U1_UMA);

	return status;
}

/*
 * func:HAL_MBOX_restoreSettings
 * purpose:
 * Restores the mailbox context
 */
long hw_mbox_restore_settings(unsigned long    base_address)
{
	long status = RET_OK;

	/* Restore IRQ enable status */
	MLBMAILBOX_IRQENABLE___0_3WriteRegister32
	(base_address, HW_MBOX_U0_ARM11, mboxsetting.irqEnable0);
	MLBMAILBOX_IRQENABLE___0_3WriteRegister32(base_address, HW_MBOX_U1_UMA,
			mboxsetting.irqEnable1);

	/* Restore Sysconfig register */
	MLBMAILBOX_SYSCONFIGWriteRegister32(base_address,
				mboxsetting.sysconfig);

	return status;
}
EXPORT_SYMBOL(hw_mbox_restore_settings);
#endif

long hw_mbox_msg_read(
		const unsigned long base_address,
		const enum hw_mbox_id_t mail_box_id,
		unsigned long *const p_read_value
		)
{
	long status = RET_OK;
	/* Check input parameters */
	CHECK_INPUT_PARAM(base_address, 0,  RET_BAD_NULL_PARAM,
		RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

	CHECK_INPUT_PARAM(p_read_value,  NULL, RET_BAD_NULL_PARAM,
		RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

	CHECK_INPUT_RANGE_MIN0(mail_box_id,
			HAL_MBOX_ID_MAX,
			RET_INVALID_ID,
			RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

	/* Read 32-bit message in mail box */
	*p_read_value = MLBMAILBOX_MESSAGE___0_15ReadRegister32
		(base_address, (unsigned long)mail_box_id);

	return status;
}
EXPORT_SYMBOL(hw_mbox_msg_read);

long hw_mbox_msg_write(const unsigned long   base_address,
			 const enum hw_mbox_id_t  mail_box_id,
			 const unsigned long write_value)
{

	long status = RET_OK;
	/* Check input parameters */
	CHECK_INPUT_PARAM(base_address, 0, RET_BAD_NULL_PARAM,
		RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
	CHECK_INPUT_RANGE_MIN0(mail_box_id,
			HAL_MBOX_ID_MAX,
			RET_INVALID_ID,
			RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

	/* Write 32-bit value to mailbox */
	MLBMAILBOX_MESSAGE___0_15WriteRegister32(base_address,
		(unsigned long)mail_box_id, (unsigned long)write_value);

	return status;
}

long hw_mbox_is_full(
		const unsigned long base_address,
		const enum hw_mbox_id_t mail_box_id,
		unsigned long  *const p_is_full
		)
{
	long status = RET_OK;
	unsigned long fullStatus;
	/* Check input parameters */
	CHECK_INPUT_PARAM(base_address, 0, RET_BAD_NULL_PARAM,
		RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

	CHECK_INPUT_PARAM(p_is_full,  NULL, RET_BAD_NULL_PARAM,
		RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

	CHECK_INPUT_RANGE_MIN0(mail_box_id,
			HAL_MBOX_ID_MAX,
			RET_INVALID_ID,
			RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

	/* read the is full status parameter for Mailbox */
	fullStatus = MLBMAILBOX_FIFOSTATUS___0_15FifoFullMBmRead32
		(base_address, (unsigned long)mail_box_id);

	/* fill in return parameter */
	*p_is_full = (fullStatus & 0xFF);

	return status;
}

long hw_mbox_nomsg_get(
		const unsigned long base_address,
		const enum hw_mbox_id_t mail_box_id,
		unsigned long *const p_num_msg
		)
{
	long status = RET_OK;
	/* Check input parameters */
	CHECK_INPUT_PARAM(base_address, 0, RET_BAD_NULL_PARAM,
		RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

	CHECK_INPUT_PARAM(p_num_msg,  NULL, RET_BAD_NULL_PARAM,
		RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

	CHECK_INPUT_RANGE_MIN0(mail_box_id,
			HAL_MBOX_ID_MAX,
			RET_INVALID_ID,
			RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

	/* Get number of messages available for MailBox */
	*p_num_msg = MLBMAILBOX_MSGSTATUS___0_15NbOfMsgMBmRead32
		(base_address, (unsigned long)mail_box_id);

	return status;
}
EXPORT_SYMBOL(hw_mbox_nomsg_get);

long hw_mbox_event_enable(
		const unsigned long base_address,
		const enum hw_mbox_id_t mail_box_id,
		const enum hw_mbox_userid_t user_id,
		const unsigned long events
		)
{
	long status = RET_OK;
	unsigned long      irqEnableReg;
	/* Check input parameters */
	CHECK_INPUT_PARAM(base_address, 0, RET_BAD_NULL_PARAM,
		RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

	CHECK_INPUT_RANGE_MIN0(
			mail_box_id,
			HAL_MBOX_ID_MAX,
			RET_INVALID_ID,
			RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
	CHECK_INPUT_RANGE_MIN0(
			enableIrq,
			HAL_MBOX_INT_MAX,
			RET_INVALID_ID,
			RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
	CHECK_INPUT_RANGE_MIN0(
			user_id,
			HAL_MBOX_USER_MAX,
			RET_INVALID_ID,
			RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
#if  defined(OMAP44XX) || defined(VPOM4430_1_06)
	/* update enable value */
	irqEnableReg = (((unsigned long)(events)) <<
		(((unsigned long)(mail_box_id))*HW_MBOX_ID_WIDTH));

	/* write new enable status */
	MLBMAILBOX_IRQENABLE_SET___0_3WriteRegister32(base_address,
		(unsigned long)user_id, (unsigned long)irqEnableReg);


#else
	/* Get current enable status */
	irqEnableReg = MLBMAILBOX_IRQENABLE___0_3ReadRegister32
		(base_address, (unsigned long)user_id);

	/* update enable value */
	irqEnableReg |= ((unsigned long)(events)) <<
		(((unsigned long)(mail_box_id))*HW_MBOX_ID_WIDTH);

	/* write new enable status */
	MLBMAILBOX_IRQENABLE___0_3WriteRegister32
(base_address, (unsigned long)user_id, (unsigned long)irqEnableReg);
#endif
	return status;
}
EXPORT_SYMBOL(hw_mbox_event_enable);

long hw_mbox_event_disable(
		const unsigned long base_address,
		const enum hw_mbox_id_t mail_box_id,
		const enum hw_mbox_userid_t user_id,
		const unsigned long events
		)
{
	long status = RET_OK;
	unsigned long irqDisableReg;
	/* Check input parameters */
	CHECK_INPUT_PARAM(base_address, 0, RET_BAD_NULL_PARAM,
		RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

	CHECK_INPUT_RANGE_MIN0(
			mail_box_id,
			HAL_MBOX_ID_MAX,
			RET_INVALID_ID,
			RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
	CHECK_INPUT_RANGE_MIN0(
			disableIrq,
			HAL_MBOX_INT_MAX,
			RET_INVALID_ID,
			RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
	CHECK_INPUT_RANGE_MIN0(
			user_id,
			HAL_MBOX_USER_MAX,
			RET_INVALID_ID,
			RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

#if defined(OMAP44XX) || defined(VPOM4430_1_06)
	irqDisableReg = (((unsigned long)(events)) <<
		(((unsigned long)(mail_box_id))*HW_MBOX_ID_WIDTH));

	/* write new enable status */
	MLBMAILBOX_IRQENABLE_CLR___0_3WriteRegister32(base_address,
	(unsigned long)user_id, (unsigned long)irqDisableReg);

#else
	/* Get current enable status */
	irqDisableReg = MLBMAILBOX_IRQENABLE___0_3ReadRegister32
		(base_address, (unsigned long)user_id);

	/* update enable value */
	irqDisableReg &= ~(((unsigned long)(events)) <<
		(((unsigned long)(mail_box_id))*HW_MBOX_ID_WIDTH));

	/* write new enable status */
	MLBMAILBOX_IRQENABLE___0_3WriteRegister32(base_address,
		(unsigned long)user_id, (unsigned long)irqDisableReg);
#endif
	return status;
}
EXPORT_SYMBOL(hw_mbox_event_disable);

long hw_mbox_event_status(
		const unsigned long base_address,
		const enum hw_mbox_id_t mail_box_id,
		const enum hw_mbox_userid_t user_id,
		unsigned long *const p_eventStatus)
{
	long status = RET_OK;
	unsigned long      irq_status_reg;
	/* Check input parameters */
	CHECK_INPUT_PARAM(base_address, 0, RET_BAD_NULL_PARAM,
		RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

	CHECK_INPUT_PARAM(pIrqStatus, NULL, RET_BAD_NULL_PARAM,
		RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

	CHECK_INPUT_RANGE_MIN0(
			mail_box_id,
			HAL_MBOX_ID_MAX,
			RET_INVALID_ID,
			RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
	CHECK_INPUT_RANGE_MIN0(
			user_id,
			HAL_MBOX_USER_MAX,
			RET_INVALID_ID,
			RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);

#if defined(OMAP44XX) || defined(VPOM4430_1_06)
	irq_status_reg = MLBMAILBOX_IRQSTATUS_CLR___0_3ReadRegister32(
					base_address, (unsigned long)user_id);
#else
	/* Get Irq Status for specified mailbox/User Id */
	irq_status_reg = MLBMAILBOX_IRQSTATUS___0_3ReadRegister32
		(base_address, (unsigned long)user_id);
#endif

	/* update status value */
	*p_eventStatus = (unsigned long)((((unsigned long)(irq_status_reg)) >>
		(((unsigned long)(mail_box_id))*HW_MBOX_ID_WIDTH)) &
		((unsigned long)(HW_MBOX_INT_ALL)));

	return status;
}

long hw_mbox_event_ack(
		const unsigned long base_address,
		const enum hw_mbox_id_t mail_box_id,
		const enum hw_mbox_userid_t    user_id,
		const unsigned long event
		)
{
	long status = RET_OK;
	unsigned long irq_status_reg;
	/* Check input parameters */
	CHECK_INPUT_PARAM(base_address, 0, RET_BAD_NULL_PARAM,
		RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
	CHECK_INPUT_RANGE_MIN0(
			irqStatus,
			HAL_MBOX_INT_MAX,
			RET_INVALID_ID,
			RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
	CHECK_INPUT_RANGE_MIN0(
			mail_box_id,
			HAL_MBOX_ID_MAX,
			RET_INVALID_ID,
			RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
	CHECK_INPUT_RANGE_MIN0(
			user_id,
			HAL_MBOX_USER_MAX,
			RET_INVALID_ID,
			RES_MBOX_BASE + RES_INVALID_INPUT_PARAM);
	/* calculate status to write */
	irq_status_reg = ((unsigned long)event) <<
		(((unsigned long)(mail_box_id))*HW_MBOX_ID_WIDTH);

#if defined(OMAP44XX) || defined(VPOM4430_1_06)
	/* clear Irq Status for specified mailbox/User Id */
	MLBMAILBOX_IRQSTATUS_CLR___0_3WriteRegister32(base_address,
			(unsigned long)user_id, (unsigned long)irq_status_reg);

#else
	/* clear Irq Status for specified mailbox/User Id */
	MLBMAILBOX_IRQSTATUS___0_3WriteRegister32(base_address,
		(unsigned long)user_id, (unsigned long)irq_status_reg);
#endif

	return status;
}
EXPORT_SYMBOL(hw_mbox_event_ack);
