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
 *  ABE_DBG_LOG
 *
 *  Parameter  :
 *      x : data to be logged
 *
 *      abe_dbg_activity_log : global circular buffer holding the data
 *      abe_dbg_activity_log_write_pointer : circular write pointer
 *
 *  Operations :
 *      saves data in the log file
 *
 *  Return value :
 *      none
 */
void abe_dbg_log_copy(abe_uint32 x)
{
	abe_dbg_activity_log[abe_dbg_activity_log_write_pointer] = x;

	if (abe_dbg_activity_log_write_pointer == (DBG_LOG_SIZE - 1))
		abe_dbg_activity_log_write_pointer = 0;
	else
		abe_dbg_activity_log_write_pointer ++;
}

void abe_dbg_log(abe_uint32 x)
{
	abe_time_stamp_t t;
	abe_millis_t m;
	abe_micros_t time;

	abe_read_global_counter(&t, &m);/* extract AE timer */
	abe_read_sys_clock(&time);	/* extract system timer */

	abe_dbg_log_copy(x);		/* dump data */
	abe_dbg_log_copy(time);
	abe_dbg_log_copy(t);
	abe_dbg_log_copy(m);
}

/*
 *  ABE_DEBUG_OUTPUT_PINS
 *
 *  Parameter  :
 *      x : d
 *
 *  Operations :
 *      set the debug output pins of AESS
 *
 *  Return value :
 *
 */
void abe_debug_output_pins(abe_uint32 x)
{
	just_to_avoid_the_many_warnings = x;
}


/*
 *  ABE_DBG_ERROR_LOG
 *
 *  Parameter  :
 *      x : d
 *
 *  Operations :
 *      log the error codes
 *
 *  Return value :
 *
 */
void abe_dbg_error_log(abe_uint32 x)
{
	just_to_avoid_the_many_warnings = x;
}

/*
 *  ABE_DEBUGGER
 *
 *  Parameter  :
 *      x : d
 *
 *  Operations :
 *
 *
 *  Return value :
 *
 */
void abe_debugger(abe_uint32 x)
{
	just_to_avoid_the_many_warnings = x;
}
