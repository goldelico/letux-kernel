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
	abe_time_stamp_t t = 0;
	abe_millis_t m = 0;
	abe_micros_t time;

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
/*
 *	S = power (2, 31) * 0.25;
 *	N =  4; B = 2; F=[1/N 1/N]; gen_and_save('dbg_8k_2.txt',	B, F, N, S);
 * 	N =  8; B = 2; F=[1/N 2/N]; gen_and_save('dbg_16k_2.txt', B, F, N, S);
 * 	N = 12; B = 2; F=[1/N 2/N]; gen_and_save('dbg_48k_2.txt', B, F, N, S);
 * 	N = 60; B = 2; F=[4/N 8/N]; gen_and_save('dbg_amic.txt', B, F, N, S);
 * 	N = 10; B = 6; F=[1/N 2/N 3/N 1/N 2/N 3/N]; gen_and_save('dbg_dmic.txt', B, F, N, S);
 */
void abe_load_embeddded_patterns (void)
{
	abe_uint32 i;
#if 0
#define patterns_dmic_len 60
const long patterns_dmic[patterns_dmic_len] = {	// 9.6kHZ
	315564800, 510594560, 510594560, 315564800, 510594560, 510594560,
	510594560, 315564800, -315565056, 510594560, 315564800, -315565056,
	510594560, -315565056, -315565056, 510594560, -315565056, -315565056,
	315564800, -510594816, 510594560, 315564800, -510594816, 510594560,
	0,	-256, 0, 0, -256, 0,
	-315565056, 510594560, -510594816, -315565056, 510594560, -510594816,
	-510594816, 315564800, 315564800, -510594816, 315564800, 315564800,
	-510594816, -315565056, 315564800, -510594816, -315565056, 315564800,
	-315565056, -510594816, -510594816, -315565056, -510594816, -510594816,
	-256, -256, -256, -256, -256, -256,
};
#endif
#define patterns_mcpdm_len (6*12)
const long patterns_mcpdm[patterns_mcpdm_len] = {
	268435200, 464943616, 536870912, 536870912, 464943616, 268435200,
	464943616, 464943616,0, 0, 464943616, 464943616,
	536870912, 	0, -536870912, -536870912, 0,536870912,
	464943616, -464943872, -256, -256, -464943872, 464943616,
	268435456, -464943872, 536870912, 536870912, -464943872, 268435456,
		0, -256, 0, 0, -256, 0,
	-268435456, 464943616, -536870912, -536870912, 464943616, -268435456,
	-464943872, 464943616, -256, -256, 464943616, -464943872,
	-536870912, 0, 536870912, 536870912, 0, -536870912,
	-464943872, -464943872, 0, 0, -464943872, -464943872,
	-268435712, -464943872, -536870912, -536870912, -464943872, -268435712,
	-256, -256, -256, -256, -256, -256,
};
#if 0
#define patterns_amic_len 120
const long patterns_amic[patterns_amic_len] = { // 6 / 12kHz
	218364928,	398972672,
	398972672,	533929728,
	510594560,	315564800,
	533929728,	-111621888,
	464943616,	-464943872,
	315564800,	-510594816,
	111621632,	-218365184,
	-111621888,	218364928,
	-315565056,	510594560,
	-464943872,	464943616,
	-533929984,	111621632,
	-510594816,	-315565056,
	-398972928,	-533929984,
	-218365184,	-398972928,
	-256,			-256,
	218364928,	398972672,
	398972672,	533929728,
	510594560,	315564800,
	533929728,	-111621888,
	464943616,	-464943872,
	315564800,	-510594816,
	111621632,	-218365184,
	-111621888,	218364928,
	-315565056,	510594560,
	-464943872,	464943616,
	-533929984,	111621632,
	-510594816,	-315565056,
	-398972928,	-533929984,
	-218365184,	-398972928,
	-256,			-256,
	218364928,	398972672,
	398972672,	533929728,
	510594560,	315564800,
	533929728,	-111621888,
	464943616,	-464943872,
	315564800,	-510594816,
	111621632,	-218365184,
	-111621888,	218364928,
	-315565056,	510594560,
	-464943872,	464943616,
	-533929984,	111621632,
	-510594816,	-315565056,
	-398972928,	-533929984,
	-218365184,	-398972928,
	-256,			-256,
	218364928,	398972672,
	398972672,	533929728,
	510594560,	315564800,
	533929728,	-111621888,
	464943616,	-464943872,
	315564800,	-510594816,
	111621632,	-218365184,
	-111621888,	218364928,
	-315565056,	510594560,
	-464943872,	464943616,
	-533929984,	111621632,
	-510594816,	-315565056,
	-398972928,	-533929984,
	-218365184,	-398972928,
	-256,			-256,
};
#endif
#define patterns_48k_len 24
const long patterns_48k[patterns_48k_len] = {	// 4kHz 8kHZ
	268435200,	464943616,
	464943616,	464943616,
	536870912,		0,
	464943616,	-464943872,
	268435456,	-464943872,
		0,	-256,
	-268435456,	464943616,
	-464943872,	464943616,
	-536870912,		0,
	-464943872,	-464943872,
	-268435712,	-464943872,
	-256,			-256,
};
#define patterns_16k_len 16
const long patterns_16k[patterns_16k_len] = {	// 2kHz / 4kHz
	379624960,	536870912,
	536870912, 		0,
	379624960,	-536870912,
		0,		-256,
	-379625216,	536870912,
	-536870912,		0,
	-379625216,	-536870912,
	-256,			-256,
};
#define patterns_8k_len 8
const long patterns_8k[patterns_8k_len] = { // 2kHz
	536870912,	536870912,
		0,		0,
	-536870912,	-536870912,
	-256,			-256,
};

	for(i = 0; i < patterns_mcpdm_len; i++)
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_SMEM,
				S_DBG_MCPDM_PATTERN_ADDR *8,
					(abe_uint32 *)(&(patterns_mcpdm[i])), 4);
	for(i = 0; i < patterns_16k_len; i++)
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_SMEM,
				S_DBG_16K_PATTERN_ADDR *8,
					(abe_uint32 *)(&(patterns_16k[i])), 4);
	for(i = 0; i < patterns_8k_len; i++)
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_SMEM,
				S_DBG_8K_PATTERN_ADDR *8,
					(abe_uint32 *)(&(patterns_8k[i])), 4);
	for (i = 0; i < patterns_48k_len; i++)
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_SMEM,
				S_DBG_48K_PATTERN_ADDR *8,
					(abe_uint32 *)(&(patterns_48k[i])), 4);
}
