/*
 * ==========================================================================
 *               Texas Instruments OMAP(TM) Platform Firmware
 * (c) Copyright 2009, Texas Instruments Incorporated.  All Rights Reserved.
 *
 *  Use of this firmware is controlled by the terms and conditions found
 *  in the license agreement under which this firmware has been supplied.
 * ==========================================================================
 */

#include "abe_main.h"

/*
 *  initialize the default values for call-backs to subroutines
 *      - FIFO IRQ call-backs for sequenced tasks
 *      - FIFO IRQ call-backs for audio player/recorders (ping-pong protocols)
 *      - Remote debugger interface
 *      - Error monitoring
 *      - Activity Tracing
 */

/*
 *	ABE_IRQ_PING_PONG
 *
 *	Parameter  :
 *		No parameter
 *
 *	Operations :
 *      Check for ping-pong subroutines (low-power players)
 *
 *	Return value :
 *		None.
 */
void abe_irq_ping_pong(void)
{
	abe_call_subroutine(abe_irq_pingpong_player_id, NOPARAMETER, NOPARAMETER, NOPARAMETER, NOPARAMETER);
}

/*
 *	ABE_IRQ_CHECK_FOR_SEQUENCES
 *
 *	Parameter  :
 *		No parameter
 *
 *	Operations :
 *      check the active sequence list
 *
 *	Return value :
 *		None.
 */
void abe_irq_check_for_sequences(void)
{
}
