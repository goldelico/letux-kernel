/* =============================================================================
*	     Texas Instruments OMAP(TM) Platform Software
*  (c) Copyright 2009 Texas Instruments Incorporated.  All Rights Reserved.
*
*  Use of this software is controlled by the terms and conditions found
*  in the license agreement under which this software has been supplied.
* =========================================================================== */
/**
 * @file  ABE_API.C
 *
 * All the visible API for the audio drivers
 *
 * @path
 * @rev     01.00
 */
/* ----------------------------------------------------------------------------
*!
*! Revision History
*! ===================================
*! 27-Nov-2008 Original
*! 05-Jun-2009 V05 release
* =========================================================================== */

#if !PC_SIMULATION

#include "abe_main.h"

#include <math.h>

/* ========================================================================== */
/**
* @fn ABE_AUTO_CHECK_DATA_FORMAT_TRANSLATION()
*/
/* ========================================================================= */

void abe_auto_check_data_format_translation (void)
{
	abe_float test, f_decibel, f_linear, max_error;
	abe_uint32 data_mem;
#define C_20LOG2 ((abe_float)6.020599913)

	for (max_error = 0, f_decibel = -70; f_decibel < 30; f_decibel += 0.1)
	{
		f_linear = 0;
		abe_translate_gain_format (DECIBELS_TO_LINABE, f_decibel, &f_linear);
		abe_translate_to_xmem_format (ABE_CMEM, f_linear, &data_mem);
		abe_translate_to_xmem_format (ABE_SMEM, f_linear, &data_mem);
		abe_translate_to_xmem_format (ABE_DMEM, f_linear, &data_mem);

		test = (abe_float) pow (2, f_decibel/C_20LOG2);
		if ( (absolute(f_linear - test) / 1) > max_error)
			max_error = (absolute(f_linear - test) / 1);

		abe_translate_gain_format (LINABE_TO_DECIBELS, f_linear, &f_decibel);
		test = (abe_float) (C_20LOG2 * log(f_linear) / log(2));

		if ( (absolute(f_decibel - test) / 1) > max_error)
			max_error = (absolute(f_decibel - test) / 1);

		/* the observed max gain error is 0.0001 decibel  (0.002%) */
	}

}
/* ========================================================================== */
/**
* @fn ABE_AUTO_CHECK_DATA_FORMAT_TRANSLATION()
*/
/* ========================================================================= */

void abe_check_opp (void)
{
	abe_use_case_id UC[] = {ABE_AUDIO_PLAYER_ON_HEADSET_OR_EARPHONE, ABE_RINGER_TONES, 0};
	abe_opp_t OPP;
	abe_hw_config_init_t CONFIG;

	abe_read_hardware_configuration (UC, &OPP,  &CONFIG); /* check HW config and OPP config */
}

/* ========================================================================== */
/**
* @fn ABE_AUTO_CHECK_DATA_FORMAT_TRANSLATION()
*/
/* ========================================================================= */

void abe_check_dma (void)
{
	abe_port_id id;
	abe_data_format_t format;
	abe_uint32 d;
	abe_dma_t dma_sink;

	/* check abe_read_port_address () */
	id = VX_UL_PORT;
	abe_read_port_address (id, &dma_sink);

	format.f = 8000; format.samp_format = STEREO_MSB;
	d = abe_dma_port_iter_factor (&format);
	d = abe_dma_port_iteration (&format);
	abe_connect_cbpr_dmareq_port (VX_DL_PORT, &format, ABE_CBPR1_IDX, &dma_sink);
	abe_read_port_address (id, &dma_sink);

}

#endif /* #if !PC_SIMULATION */
