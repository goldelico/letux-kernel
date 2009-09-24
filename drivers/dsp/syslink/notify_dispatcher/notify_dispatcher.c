/*
 * notify_dispatcher.c
 *
 * Syslnk IPC support functions for TI OMAP processors.
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
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <syslink/GlobalTypes.h>
#include <syslink/notify_dispatcher.h>
#include <syslink/hw_mbox.h>
#include <syslink/hw_ocp.h>
#include <linux/delay.h>
#include <mach/irqs.h>
#include <linux/io.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Suman Anna");
MODULE_AUTHOR("Hari Kanigeri");
MODULE_DESCRIPTION("Dispatchers mailbox events");

#define OMAP_MBOX_BASE 0x4A0F4000
#define OMAP_MBOX_SIZE 0x2000

struct mbox_isrs mailbx_swisrs;
EXPORT_SYMBOL(mailbx_swisrs);

struct mbox_config mailbx_hw_config;
EXPORT_SYMBOL(mailbx_hw_config);


unsigned long int i_mbox_module_no;
unsigned long int i_a_irq_bit;
const unsigned long int kmpu_mailboxes = 4;


/* notify dispatcher module initialization function. */
static int __init ntfy_disp_init_module(void) ;

/* notify dispatcher module cleanup function. */
static void  __exit ntfy_disp_finalize_module(void) ;

/*
* Bind an ISR to the HW interrupt line coming into the processor
*/
irqreturn_t notify_mailbx0_user0_isr(int temp, void *anArg, struct pt_regs *p)
{

	REG int p_eventStatus = 0;
	int mbox_index = mailbx_hw_config.mbox_modules - 1;
	int i;

	for (i = 0; i <  mailbx_hw_config.mailboxes[mbox_index]; i++) {
		/*Read the Event Status */
		hw_mbox_event_status(mailbx_hw_config.mbox_linear_addr,
		(enum hw_mbox_id_t)(i), HW_MBOX_U0_ARM11,
		(unsigned long *) &p_eventStatus);
		if (p_eventStatus & HW_MBOX_INT_ALL) {
			if ((mailbx_swisrs.isrs
			[mbox_index][i*HW_MBOX_ID_WIDTH]) > 0) {
				(*mailbx_swisrs.isrs[mbox_index][i
				*HW_MBOX_ID_WIDTH])
				(mailbx_swisrs.isr_params[mbox_index]
				[i*HW_MBOX_ID_WIDTH]);

				hw_mbox_event_ack(mailbx_hw_config.
				mbox_linear_addr,
				(enum hw_mbox_id_t)i,
				HW_MBOX_U0_ARM11, HW_MBOX_INT_NEW_MSG);
			}
		}
	}
	return IRQ_HANDLED;
}
EXPORT_SYMBOL(notify_mailbx0_user0_isr);

/*
*
* Bind an ISR to the HW interrupt line coming into the processor
*/
int ntfy_disp_bind_interrupt(int interrupt_no,
		isr_call_back hw_isr, void *isr_arg)
{
	int status = 0;
	bool valid_interrupt = false;
	int i;

	/*Validate the arguments*/
	for (i = 0; i < mailbx_hw_config.mbox_modules; i++) {
		if (interrupt_no == mailbx_hw_config.
				interrupt_lines[i]) {
			valid_interrupt = true;
			break;
		}
	}
	if (valid_interrupt != true)
		status = -EINVAL;
	if ((status == 0) && (hw_isr == NULL))
		status = -EINVAL;
	status =
	request_irq(interrupt_no, (void *)hw_isr,
	IRQF_SHARED, "mbox", notify_mailbx0_user0_isr);
	if (status != 0)
		printk(KERN_ALERT "REQUEST_IRQ FAILED\n");
	return status;
}
EXPORT_SYMBOL(ntfy_disp_bind_interrupt);
/*
*
* Print the mailbox registers and other useful debug information
*/
void ntfy_disp_Debug(void)
{
	unsigned long p_eventStatus;
	hw_mbox_event_status(mailbx_hw_config.mbox_linear_addr,
		(enum hw_mbox_id_t)1, HW_MBOX_U0_ARM11, &p_eventStatus);
}

/*
*
* Uninitialize the Notify disptacher Manager module
*/
int ntfy_disp_deinit(void)
{
	unsigned long int temp;
	int i;

	/*Reset the Mailbox module */
	hw_ocp_soft_reset(mailbx_hw_config.mbox_linear_addr);
	do {
		hw_ocp_soft_reset_isdone(mailbx_hw_config.mbox_linear_addr,
								&temp);
	} while (temp == 0);

	/*Reset the Configuration for the Mailbox modules on MPU */
	for (i = 0; i < MAX_MBOX_MODULES; i++) {
		if ((i < mailbx_hw_config.mbox_modules) &&
			(mailbx_hw_config.interrupt_lines[i] != (-1)))
			disable_irq((int)(mailbx_hw_config.interrupt_lines[i]));

		mailbx_hw_config.interrupt_lines[i] = (-1);
		mailbx_hw_config.mailboxes[i] = (-1);
	}

	if (mailbx_hw_config.mbox_linear_addr != NULL)
		iounmap((unsigned int *) mailbx_hw_config.mbox_linear_addr);
	mailbx_hw_config.mbox_modules = 0;
	mailbx_hw_config.mbox_linear_addr = (unsigned long int) (-1);
	return 0;
}
EXPORT_SYMBOL(ntfy_disp_deinit);
/*
*
* Return the pointer to the Mailbox Manager's configuration object
*/
struct mbox_config *ntfy_disp_get_config(void)
{
	return &(mailbx_hw_config);
}
EXPORT_SYMBOL(ntfy_disp_get_config);
/*
*
*  Initialize the Mailbox Manager module and the mailbox hardware
*/
int ntfy_disp_init(void)
{
	int status = 0;
	int i, j;

	/*Initialize the configuration parameters for the Mailbox modules on
	* MPU */
	for (i = 0; i < MAX_MBOX_MODULES; i++) {
		mailbx_hw_config.interrupt_lines[i] = (-1);
		mailbx_hw_config.mailboxes[i] = (-1);
	}
	for (i = 0; i < MAX_MBOX_MODULES; i++) {
		mailbx_swisrs.isrNo[i] = (-1);
		for (j = 0; j < MAX_MBOX_ISRS; j++) {
				mailbx_swisrs.isrs[i][j] = NULL;
				mailbx_swisrs.isr_params[i][j] = NULL;
		}
	}
	/*Setup the configuration parameters for the Mailbox modules on MPU */
	mailbx_hw_config.mbox_linear_addr =
		(u32)ioremap(OMAP_MBOX_BASE, OMAP_MBOX_SIZE);
	mailbx_hw_config.mbox_modules = 1;
	mailbx_hw_config.interrupt_lines[(mailbx_hw_config.mbox_modules-1)]
			= INT_44XX_MAIL_U0_MPU;
	mailbx_hw_config.mailboxes[(mailbx_hw_config.mbox_modules-1)]
			= kmpu_mailboxes;

	return status;
}
EXPORT_SYMBOL(ntfy_disp_init);
/*
*
* Disable a particular IRQ bit on a Mailbox IRQ Enable Register
*/
int ntfy_disp_interrupt_disable(unsigned long int mbox_module_no,
					int a_irq_bit)
{
	int status = 0;

	/*Validate the parameters */
	if (mbox_module_no > mailbx_hw_config.mbox_modules) {
		status = -EINVAL;
		goto func_end;
	}
	if (a_irq_bit > (HW_MBOX_ID_WIDTH *
			mailbx_hw_config.mailboxes[mbox_module_no-1])) {
		status = -EINVAL;
		goto func_end;
	}
	/*Interrupts on transmission not supported currently */
	if ((a_irq_bit % HW_MBOX_ID_WIDTH)) {
		status = -EACCES;
		goto func_end;
	}
	hw_mbox_event_disable(mailbx_hw_config.mbox_linear_addr ,
		(enum hw_mbox_id_t)(a_irq_bit / HW_MBOX_ID_WIDTH),
		HW_MBOX_U0_ARM11, HW_MBOX_INT_NEW_MSG);
func_end:
	return status;
}
EXPORT_SYMBOL(ntfy_disp_interrupt_disable);
/*
*
*  Enable a particular IRQ bit on a Mailbox IRQ Enable Register
*/
int ntfy_disp_interrupt_enable(unsigned long int mbox_module_no,
					int a_irq_bit)
{
	int status = 0;
	/*Validate the parameters */
	if (mbox_module_no > mailbx_hw_config.mbox_modules) {
		status = -EINVAL;
		goto func_end;
	}
	if (a_irq_bit > (HW_MBOX_ID_WIDTH *
			mailbx_hw_config.mailboxes[mbox_module_no-1])) {
		status = -EINVAL;
		goto func_end;
	}
	if (mailbx_swisrs.isrs[mbox_module_no-1][a_irq_bit] == NULL) {
		status = -EFAULT;
		goto func_end;
	}
	/*Interrupts on transmission not supported currently */
	if (a_irq_bit % HW_MBOX_ID_WIDTH) {
		status = -EACCES;
		goto func_end;
	}
	hw_mbox_event_enable(mailbx_hw_config.mbox_linear_addr,
	(enum hw_mbox_id_t)(a_irq_bit / HW_MBOX_ID_WIDTH),
	HW_MBOX_U0_ARM11, HW_MBOX_INT_NEW_MSG);
func_end:
	return status;
}
EXPORT_SYMBOL(ntfy_disp_interrupt_enable);
/*
*
* Read a message on a Mailbox FIFO queue
*/
int ntfy_disp_read(unsigned long int mbox_module_no,
		int a_mbox_no, int *messages,
		int *num_messages, short int read_all)
{
	int status = 0;

	if (mbox_module_no > mailbx_hw_config.mbox_modules) {
		status = -EINVAL;
		goto func_end;
	}
	if (a_mbox_no >= mailbx_hw_config.mailboxes[mbox_module_no-1]) {
		status = -EINVAL;
		goto func_end;
	}
	if (messages == NULL || num_messages == NULL) {
		status = -EFAULT;
		goto func_end;
	}
	/*Read a single message */
	hw_mbox_nomsg_get(mailbx_hw_config.mbox_linear_addr ,
	(enum hw_mbox_id_t)a_mbox_no, (unsigned long *const)num_messages);
	if (*num_messages > 0) {
		hw_mbox_msg_read(mailbx_hw_config.mbox_linear_addr,
		(enum hw_mbox_id_t)a_mbox_no, (unsigned long *const)messages);
	} else
		status = -EBUSY;
func_end:
	return status;
}
EXPORT_SYMBOL(ntfy_disp_read);
/*
*
* Register a ISR callback associated with a particular IRQ bit on a
* Mailbox IRQ Enable Register and Also Reigisters a Interrupt Handler
*/
int ntfy_disp_register(unsigned long int mbox_module_no,
		int a_irq_bit, isr_call_back isr_cbck_fn,
		void *isrCallbackArgs)

{
	int status = 0;

	i_mbox_module_no =  mbox_module_no;
	i_a_irq_bit = a_irq_bit;

/*Validate the parameters */
	if (mbox_module_no > mailbx_hw_config.mbox_modules) {
		status = -EINVAL;
		goto func_end;
	}
	if (a_irq_bit > (HW_MBOX_ID_WIDTH *
			mailbx_hw_config.mailboxes[mbox_module_no-1])) {
		status = -EINVAL;
		goto func_end;
	}
	if ((isr_cbck_fn == NULL)) {
		status = -EINVAL;
		goto func_end;
	}
	if (a_irq_bit % HW_MBOX_ID_WIDTH) {
		status = -EINVAL;
		goto func_end;
	}

	hw_mbox_event_disable(mailbx_hw_config.mbox_linear_addr,
	(enum hw_mbox_id_t)(a_irq_bit / HW_MBOX_ID_WIDTH),
	HW_MBOX_U0_ARM11, HW_MBOX_INT_NEW_MSG);

	mailbx_swisrs.isrs[mbox_module_no-1][a_irq_bit] = isr_cbck_fn;
	mailbx_swisrs.isr_params[mbox_module_no-1][a_irq_bit] = isrCallbackArgs;
func_end:
	return status;
}
EXPORT_SYMBOL(ntfy_disp_register);
/*
*
* Send a message on a Mailbox FIFO queue
*/
int ntfy_disp_send(unsigned long int mbox_module_no,
		int a_mbox_no, int message)
{
	int status = 0;

	/*Validate the arguments */
	if (mbox_module_no > mailbx_hw_config.mbox_modules) {
		status = -EINVAL;
		goto func_end;
	}
	if (a_mbox_no > mailbx_hw_config.mailboxes[mbox_module_no-1]) {
		status = -EINVAL;
		goto func_end;
	}

	/*Send the message */
	hw_mbox_msg_write(mailbx_hw_config.mbox_linear_addr,
		(enum hw_mbox_id_t)a_mbox_no, message);

func_end:
	return status;
}
EXPORT_SYMBOL(ntfy_disp_send);
/*
*
* Remove the ISR to the HW interrupt line coming into the processor
*/
int ntfy_disp_unbind_interrupt(int interrupt_no)
{
	int i;
	short int valid_interrupt = false;

	/*Validate the arguments*/
	for (i = 0; i < mailbx_hw_config.mbox_modules; i++) {
		if (interrupt_no == mailbx_hw_config.
				interrupt_lines[i]) {
			valid_interrupt = true;
			break;
		}
	}
	if (valid_interrupt != true)
		return -EFAULT;
	/*Unbind the HW Interrupt */
	disable_irq(interrupt_no);
	return 0;
}
EXPORT_SYMBOL(ntfy_disp_unbind_interrupt);
/*
*
* Unregister a ISR callback associated with a particular IRQ bit on a
* Mailbox IRQ Enable Register
*/
int ntfy_disp_unregister(unsigned long int mbox_module_no,
					int a_irq_bit)
{
	int status = 0;

	/*Validate the arguments */
	if (mbox_module_no > mailbx_hw_config.mbox_modules) {
		status = -EINVAL;
		goto func_end;
	}

	if (a_irq_bit > (HW_MBOX_ID_WIDTH *
			mailbx_hw_config.mailboxes[mbox_module_no-1])) {
		status = -EINVAL;
		goto func_end;
	}

	/*Interrupts on transmission not supported currently */
	if (a_irq_bit % HW_MBOX_ID_WIDTH) {
		status = -EINVAL;
		goto func_end;
	}
	/*Remove the ISR plugin */
	mailbx_swisrs.isrs[mbox_module_no-1][a_irq_bit] = NULL;
	mailbx_swisrs.isr_params[mbox_module_no-1][a_irq_bit] = NULL;
func_end:
	return status;
}
EXPORT_SYMBOL(ntfy_disp_unregister);

/* Initialization function */
static int __init ntfy_disp_init_module(void)
{
	ntfy_disp_init();
	return 0;
}

/* Finalization function */
static void __exit ntfy_disp_finalize_module(void)
{
	ntfy_disp_deinit();
}

module_init(ntfy_disp_init_module);
module_exit(ntfy_disp_finalize_module);
