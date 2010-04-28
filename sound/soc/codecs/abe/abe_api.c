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

#include "C_ABE_FW_SIZE.h"	      /* used to tune the FW size for faster RTL simulations */

static abe_uint32 ABE_FW_PM[ABE_PMEM_SIZE / 4]  = {
#include "C_ABE_FW.PM"
};
static abe_uint32 ABE_FW_CM[ABE_CMEM_SIZE_OPTIMIZED / 4]  = {
#include "C_ABE_FW.CM"
};
static abe_uint32 ABE_FW_DM[ABE_DMEM_SIZE_OPTIMIZED / 4]  = {
#include "C_ABE_FW.lDM"
};
static abe_uint32 ABE_FW_SM[ABE_SMEM_SIZE_OPTIMIZED / 4]  = {
#include "C_ABE_FW.SM32"
};

/**
* @fn abe_reset_hal()
*
*  Operations : reset the HAL by reloading the static variables and default AESS registers.
*	Called after a PRCM cold-start reset of ABE
*
* @see	ABE_API.h
*/
void abe_reset_hal(void)
{
	abe_dma_t dma_sink;
	abe_data_format_t format;

	abe_dbg_output = TERMINAL_OUTPUT;

	/* load firmware */
	abe_load_fw ();

	/* load default port values */
	abe_reset_all_ports();

	/* load default sequence list */
	abe_reset_all_sequence();

	/* build the scheduler tables */
	abe_build_scheduler_table();

	/* build the default uplink router configurations */
	abe_set_router_configuration(UPROUTE, UPROUTE_CONFIG_AMIC, (abe_router_t *)abe_router_ul_table_preset[UPROUTE_CONFIG_AMIC]);
#if 0
	abe_set_router_configuration(UPROUTE, UPROUTE_CONFIG_DMIC1, (abe_router_t *)abe_router_ul_table_preset[UPROUTE_CONFIG_DMIC1]);
	abe_set_router_configuration(UPROUTE, UPROUTE_CONFIG_DMIC2, (abe_router_t *)abe_router_ul_table_preset[UPROUTE_CONFIG_DMIC2]);
	abe_set_router_configuration(UPROUTE, UPROUTE_CONFIG_DMIC3, (abe_router_t *)abe_router_ul_table_preset[UPROUTE_CONFIG_DMIC3]);
#endif
	/* meaningful other microphone configuration can be added here */

	/* init hardware components */
	abe_hw_configuration();

	/* enable the VX_UL path with Analog microphones from Phoenix */

	/* MM_DL INIT connect a DMA channel to MM_DL port (ATC FIFO) */
	format.f = 48000;
	format.samp_format = STEREO_MSB;
	abe_connect_cbpr_dmareq_port(MM_DL_PORT, &format, ABE_CBPR0_IDX, &dma_sink);

	/* VX_DL INIT connect a DMA channel to VX_DL port (ATC FIFO) */
	format.f = 8000;
	format.samp_format = STEREO_MSB;
	abe_connect_cbpr_dmareq_port(VX_DL_PORT, &format, ABE_CBPR1_IDX, &dma_sink);
#if PC_SIMULATION
	{	int dVirtAudioVoiceMode, dVirtAudioVoiceSampleFrequency;
		/* dVirtAudioVoiceMode = 1;		 MONO for Voice */
		dVirtAudioVoiceMode = 2;		/* MONO for Voice */
		dVirtAudioVoiceSampleFrequency = 1;	/* 8kHz for Voice */
		target_server_set_voice_sampling(dVirtAudioVoiceMode, dVirtAudioVoiceSampleFrequency);
	}
#endif

	/* VX_UL INIT connect a DMA channel to VX_UL port (ATC FIFO) */
	format.f = 8000;
	format.samp_format = STEREO_MSB;
	abe_connect_cbpr_dmareq_port(VX_UL_PORT, &format, ABE_CBPR2_IDX, &dma_sink);

	/* MM_UL2 INIT connect a DMA channel to MM_UL2 port (ATC FIFO) */
	format.f = 48000;
	format.samp_format = STEREO_MSB;
	abe_connect_cbpr_dmareq_port(MM_UL2_PORT, &format, ABE_CBPR4_IDX, &dma_sink);

	/* TONES INIT connect a DMA channel to TONES port (ATC FIFO) */
	format.f = 48000;
	format.samp_format = STEREO_MSB;
	abe_connect_cbpr_dmareq_port(TONES_DL_PORT, &format, ABE_CBPR5_IDX, &dma_sink);

	/* VIBRA/HAPTICS INIT connect a DMA channel to VIBRA/HAPTICS port (ATC FIFO) */
	format.f = 24000;
	format.samp_format = STEREO_MSB;
	abe_connect_cbpr_dmareq_port(VIB_DL_PORT, &format, ABE_CBPR6_IDX, &dma_sink);

	/* mixers' default configuration = voice on earphone + music on hands-free path */
	abe_write_mixer(MIXDL1, MUTE_GAIN, RAMP_0MS, MIX_DL1_INPUT_TONES);
	abe_write_mixer(MIXDL1, MUTE_GAIN, RAMP_1MS, MIX_DL1_INPUT_VX_DL);
	abe_write_mixer(MIXDL1, MUTE_GAIN, RAMP_2MS, MIX_DL1_INPUT_MM_DL);
	abe_write_mixer(MIXDL1, MUTE_GAIN, RAMP_5MS, MIX_DL1_INPUT_MM_UL2);

	abe_write_mixer(MIXDL2, MUTE_GAIN, RAMP_10MS, MIX_DL2_INPUT_TONES);
	abe_write_mixer(MIXDL2, MUTE_GAIN, RAMP_20MS, MIX_DL2_INPUT_VX_DL);
	abe_write_mixer(MIXDL2, MUTE_GAIN, RAMP_50MS, MIX_DL2_INPUT_MM_DL);
	abe_write_mixer(MIXDL2, MUTE_GAIN, RAMP_100MS, MIX_DL2_INPUT_MM_UL2);

	abe_write_mixer(MIXSDT, MUTE_GAIN, RAMP_0MS, MIX_SDT_INPUT_UP_MIXER);
	abe_write_mixer(MIXSDT, GAIN_0dB, RAMP_0MS, MIX_SDT_INPUT_DL1_MIXER);
}

/**
* @fn abe_load_fwl()
*
*  Operations :
*      loads the Audio Engine firmware, generate a single pulse on the Event generator
*      to let execution start, read the version number returned from this execution.
*
* @see	ABE_API.h
*/
void abe_load_fw_param(abe_uint32 *PMEM, abe_uint32 PMEM_SIZE,
			abe_uint32 *CMEM, abe_uint32 CMEM_SIZE,
				abe_uint32 *SMEM, abe_uint32 SMEM_SIZE,
					abe_uint32 *DMEM, abe_uint32 DMEM_SIZE)
{
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_PMEM, 0, PMEM, PMEM_SIZE);
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_CMEM, 0, CMEM, CMEM_SIZE);
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_SMEM, 0, SMEM, SMEM_SIZE);
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, 0, DMEM, DMEM_SIZE);
}

void abe_load_fw(void)
{
	abe_load_fw_param(ABE_FW_PM, sizeof (ABE_FW_PM),
		ABE_FW_CM, sizeof (ABE_FW_CM), ABE_FW_SM, sizeof (ABE_FW_SM),
						ABE_FW_DM, sizeof (ABE_FW_DM));
}

/*
 *  ABE_HARDWARE_CONFIGURATION
 *
 *  Parameter  :
 *  U : use-case description list (pointer)
 *  H : pointer to the output structure
 *
 *  Operations :
 *      return a structure with the HW thresholds compatible with the HAL/FW/AESS_ATC
 *      will be upgraded in FW06
 *
 *  Return value :
 *      None.
 */
void abe_read_hardware_configuration(abe_use_case_id *u, abe_opp_t *o, abe_hw_config_init_t *hw)
{
	abe_read_use_case_opp(u, o);

	/* @@@ keep OPP 100 for now */
	(*o) = ABE_OPP100;

	hw->MCPDM_CTRL__DIV_SEL = 0;		/* 0: 96kHz   1:192kHz */
	hw->MCPDM_CTRL__CMD_INT = 1;		/* 0: no command in the FIFO,  1: 6 data on each lines (with commands) */
	hw->MCPDM_CTRL__PDMOUTFORMAT = 0;	/* 0:MSB aligned  1:LSB aligned */
	hw->MCPDM_CTRL__PDM_DN5_EN = 1;
	hw->MCPDM_CTRL__PDM_DN4_EN = 1;
	hw->MCPDM_CTRL__PDM_DN3_EN = 1;
	hw->MCPDM_CTRL__PDM_DN2_EN = 1;
	hw->MCPDM_CTRL__PDM_DN1_EN = 1;
	hw->MCPDM_CTRL__PDM_UP3_EN = 0;
	hw->MCPDM_CTRL__PDM_UP2_EN = 1;
	hw->MCPDM_CTRL__PDM_UP1_EN = 1;
	hw->MCPDM_FIFO_CTRL_DN__DN_TRESH = MCPDM_DL_ITER/6; /* All the McPDM_DL FIFOs are enabled simultaneously */
	hw->MCPDM_FIFO_CTRL_UP__UP_TRESH = MCPDM_UL_ITER/2; /* number of ATC access upon AMIC DMArequests, 2 the FIFOs channels are enabled */

	hw->DMIC_CTRL__DMIC_CLK_DIV = 0;	/* 0:2.4MHz  1:3.84MHz */
	hw->DMIC_CTRL__DMICOUTFORMAT = 0;	/* 0:MSB aligned  1:LSB aligned */
	hw->DMIC_CTRL__DMIC_UP3_EN = 1;
	hw->DMIC_CTRL__DMIC_UP2_EN = 1;
	hw->DMIC_CTRL__DMIC_UP1_EN = 1;
	hw->DMIC_FIFO_CTRL__DMIC_TRESH = DMIC_ITER/6; /* 1*(DMIC_UP1_EN+ 2+ 3)*2 OCP read access every 96/88.1 KHz. */

	hw->MCBSP_SPCR1_REG__RJUST = 1;		/* 1:MSB  2:LSB aligned */
	hw->MCBSP_THRSH2_REG_REG__XTHRESHOLD = 1;
	hw->MCBSP_THRSH1_REG_REG__RTHRESHOLD = 1;

	hw->AESS_EVENT_GENERATOR_COUNTER__COUNTER_VALUE = EVENT_GENERATOR_COUNTER_DEFAULT; /* 2050 gives about 96kHz */
	hw->AESS_EVENT_SOURCE_SELECTION__SELECTION = 0;	/* 0: DMAreq, 1:Counter */
	hw->AESS_AUDIO_ENGINE_SCHEDULER__DMA_REQ_SELECTION = ABE_ATC_MCPDMDL_DMA_REQ; /* 5bits DMAreq selection */

	hw->HAL_EVENT_SELECTION = EVENT_MCPDM;	/* to be used before calling "abe_event_generator_switch (EVENT_MCPDM)" */
}

/*
 *  ABE_DEFAULT_CONFIGURATION
 *
 *  Parameter  :
 *      use-case-ID : "LP player", "voice-call" use-cases as defined in the paragraph
 *      "programming use-case sequences"
 *      Param 1, 2, 3, 4 used for non regression tests
 *
 *  Operations :
 *      private API used during development. Loads all the necessary parameters and data
 *      patterns to allow a stand-alone functional test without the need of.
 *
 *  Return value :
 *      None.
 */
void abe_default_configuration(abe_uint32 use_case)
{
	abe_data_format_t format;
	abe_dma_t dma_sink;
	abe_uint32 data_sink;
	abe_use_case_id UC2[] = {ABE_AUDIO_PLAYER_ON_HEADSET_OR_EARPHONE,
					ABE_RINGER_TONES, (abe_use_case_id)0};
	abe_use_case_id UC5[] = {ABE_AUDIO_PLAYER_ON_HEADSET_OR_EARPHONE,
							(abe_use_case_id)0};
        abe_opp_t OPP;
	abe_hw_config_init_t CONFIG;

	switch (use_case) {
	/* voice ul/dl on earpiece + MM_DL on IHF */
	case UC2_VOICE_CALL_AND_IHF_MMDL:
		/* enable one of the preloaded and programmable routing
		 * configuration for the uplink paths
		 * Here enable the VX_UL path with Digital microphones:
		 * abe_enable_router_configuration(UPROUTE, UPROUTE_CONFIG_DMIC1);
		 * To be added here:
		 * - Device driver initialization following
		 *   abe_read_hardware_configuration() returned data
		 * McPDM_DL : 6 slots activated (5 + Commands)
		 * DMIC : 6 microphones activated
		 * McPDM_UL : 2 microphones activated (No status)
		 */
		abe_read_hardware_configuration(UC2, &OPP, &CONFIG);	/* check hw config and opp config */
		abe_set_opp_processing(OPP);				/* sets the OPP100 on FW05.xx */
		abe_write_event_generator(CONFIG.HAL_EVENT_SELECTION);	/* "tick" of the audio engine */
		/* mixers' configuration = voice on earphone + music on hands-free path */
		abe_write_mixer(MIXDL1, GAIN_0dB, RAMP_1MS, MIX_DL1_INPUT_VX_DL);
		abe_write_mixer(MIXDL2, GAIN_0dB, RAMP_50MS, MIX_DL2_INPUT_MM_DL);

		abe_enable_data_transfer(MM_UL2_PORT);

		abe_enable_data_transfer(MM_DL_PORT);			/* enable all the data paths */
		abe_enable_data_transfer(VX_DL_PORT);
		abe_enable_data_transfer(VX_UL_PORT);
		abe_enable_data_transfer(PDM_UL_PORT);
		abe_enable_data_transfer(DMIC_PORT1);			/* DMIC ATC can be enabled even if the DMIC */
		abe_enable_data_transfer(DMIC_PORT2);			/*  IP is not enabled. */
		abe_enable_data_transfer(DMIC_PORT3);
		abe_enable_data_transfer(PDM_DL1_PORT);
		break;
	case UC5_PINGPONG_MMDL:
		/* Ping-Pong access through MM_DL using Left/Right
		 * 16bits/16bits data format
		 * To be added here:
		 * - Device driver initialization following abe_read_hardware_configuration() returned data
		 * McPDM_DL : 6 slots activated (5 + Commands)
		 * DMIC : 6 microphones activated
		 * McPDM_UL : 2 microphones activated (No status)
		 */
		abe_read_hardware_configuration(UC5, &OPP, &CONFIG);	/* check hw config and opp config */
		abe_set_opp_processing(OPP);				/* sets the OPP100 on FW05.xx */
		abe_write_event_generator(CONFIG.HAL_EVENT_SELECTION);	/* "tick" of the audio engine */

		/* MM_DL init: overwrite the previous default initialization made above */
		format.f = 48000;
		format.samp_format = MONO_MSB;

		/* connect a Ping-Pong SDMA protocol to MM_DL port
		 * with Ping-Pong 576 mono samples
		 * (12x4 bytes for each ping & pong size)
		 */
		abe_connect_dmareq_ping_pong_port(MM_DL_PORT, &format, ABE_CBPR0_IDX, (12 * 4), &dma_sink);

		/* mixers' configuration = voice on earphone + music on hands-free path */
		abe_write_mixer(MIXDL1, GAIN_0dB, RAMP_2MS, MIX_DL1_INPUT_MM_DL);
		abe_write_mixer(MIXDL2, GAIN_0dB, RAMP_50MS, MIX_DL2_INPUT_MM_DL);

		/* Here: connect the sDMA to "dma_sink" content */
		abe_enable_data_transfer(MM_DL_PORT);			/* enable all the data paths */
		abe_enable_data_transfer(PDM_DL1_PORT);
		break;
	case UC6_PINGPONG_MMDL_WITH_IRQ:
		/* Ping-Pong using the IRQ instead of the sDMA  */
		abe_read_hardware_configuration(UC5, &OPP, &CONFIG);	/* check hw config and opp config */
		abe_set_opp_processing(OPP);				/* sets the OPP100 on FW05.xx */
		abe_write_event_generator(CONFIG.HAL_EVENT_SELECTION);	/* "tick" of the audio engine */

		/* MM_DL init: overwrite the previous default initialization made above */
		format.f = 48000;
		format.samp_format = STEREO_16_16;

		/* connect a Ping-Pong cache-flush protocol to MM_DL port
		 * with 50Hz (20ms) rate
		 */
		abe_add_subroutine(&abe_irq_pingpong_player_id,
			(abe_subroutine2) abe_default_irq_pingpong_player,
								SUB_0_PARAM, (abe_uint32*)0);
		#define N_SAMPLES_BYTES (12 *4)
		abe_connect_irq_ping_pong_port(MM_DL_PORT, &format,
			abe_irq_pingpong_player_id, N_SAMPLES_BYTES, &data_sink,
							PING_PONG_WITH_MCU_IRQ);

		/* mixers' configuration = voice on earphone + music on hands-free path */
		abe_write_mixer(MIXDL1, GAIN_0dB, RAMP_2MS, MIX_DL1_INPUT_MM_DL);
		abe_write_mixer(MIXDL2, GAIN_0dB, RAMP_50MS, MIX_DL2_INPUT_MM_DL);

		abe_enable_data_transfer(MM_DL_PORT);			/* enable all the data paths */
		abe_enable_data_transfer(PDM_DL1_PORT);
		break;
	case UC7_STOP_ALL:
		abe_disable_data_transfer(MM_UL2_PORT);
		abe_disable_data_transfer(MM_DL_PORT);
		abe_disable_data_transfer(VX_DL_PORT);
		abe_disable_data_transfer(VX_UL_PORT);
		abe_disable_data_transfer(PDM_UL_PORT);
		abe_disable_data_transfer(DMIC_PORT1);
		abe_disable_data_transfer(DMIC_PORT2);
		abe_disable_data_transfer(DMIC_PORT3);
		abe_disable_data_transfer(PDM_DL1_PORT);
		abe_disable_data_transfer(PDM_DL2_PORT);
		break;
	case UC81_ROUTE_AMIC:
		abe_set_router_configuration(UPROUTE, UPROUTE_CONFIG_AMIC,
			(abe_router_t *) abe_router_ul_table_preset[UPROUTE_CONFIG_AMIC]);
		break;
	case UC82_ROUTE_DMIC01:
		abe_set_router_configuration(UPROUTE, UPROUTE_CONFIG_DMIC1,
			(abe_router_t *) abe_router_ul_table_preset[UPROUTE_CONFIG_DMIC1]);
		break;
	case UC83_ROUTE_DMIC23:
		abe_set_router_configuration(UPROUTE, UPROUTE_CONFIG_DMIC2,
			(abe_router_t *) abe_router_ul_table_preset[UPROUTE_CONFIG_DMIC2]);
		break;
	case UC84_ROUTE_DMIC45:
		abe_set_router_configuration(UPROUTE, UPROUTE_CONFIG_DMIC3,
			(abe_router_t *) abe_router_ul_table_preset[UPROUTE_CONFIG_DMIC3]);
		break;
	default:
		break;
	}
}

/*
 *	ABE_IRQ_PROCESSING
 *
 *	Parameter  :
 *		No parameter
 *
 *	Operations :
 *		This subroutine will check the IRQ_FIFO from the AE and act accordingly.
 *		Some IRQ source are originated for the delivery of "end of time sequenced tasks"
 *		notifications, some are originated from the Ping-Pong protocols, some are generated from
 *		the embedded debugger when the firmware stops on programmable break-points, etc …
 *
 *	Return value :
 *		None.
 */
void abe_irq_processing(void)
{
	abe_uint32 clear_abe_irq;
	abe_monitoring();
	abe_irq_ping_pong();
	abe_irq_check_for_sequences();
	clear_abe_irq = 1;
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_ATC, ABE_MCU_IRQSTATUS, &clear_abe_irq, 4);
}

/*
 *  ABE_WRITE_EVENT_GENERATOR
 *
 *  Parameter  :
 *      e: Event Generation Counter, McPDM, DMIC or default.
 *
 *  Operations :
 *      load the AESS event generator hardware source. Loads the firmware parameters
 *      accordingly. Indicates to the FW which data stream is the most important to preserve
 *      in case all the streams are asynchronous. If the parameter is "default", let the HAL
 *      decide which Event source is the best appropriate based on the opened ports.
 *
 *      When neither the DMIC and the McPDM are activated the AE will have its EVENT generator programmed
 *      with the EVENT_COUNTER. The event counter will be tuned in order to deliver a pulse frequency higher
 *      than 96 kHz. The DPLL output at 100% OPP is MCLK = (32768kHz x6000) = 196.608kHz
 *      The ratio is (MCLK/96000)+(1<<1) = 2050
 *      (1<<1) in order to have the same speed at 50% and 100% OPP (only 15 MSB bits are used at OPP50%)
 *
 *  Return value :
 *      None.
 */
void abe_write_event_generator(abe_event_id e)
{
	abe_uint32 event, selection, counter, start;

	counter = EVENT_GENERATOR_COUNTER_DEFAULT;
	abe_current_event_id = e;

	switch (e) {
	case EVENT_MCPDM:
		selection = EVENT_SOURCE_DMA;
		start = EVENT_GENERATOR_ON;
		event = ABE_ATC_MCPDMDL_DMA_REQ;
		break;
	case EVENT_DMIC:
		selection = EVENT_SOURCE_DMA;
		start = EVENT_GENERATOR_ON;
		event = ABE_ATC_DMIC_DMA_REQ;
		break;
	case EVENT_TIMER:
		selection = EVENT_SOURCE_COUNTER;
		start = EVENT_GENERATOR_ON;
		event = 0;
		break;
	case EVENT_McBSP:
		selection = EVENT_SOURCE_COUNTER;
		start = EVENT_GENERATOR_ON;
		event = 0;
		break;
	case EVENT_McASP:
		selection = EVENT_SOURCE_COUNTER;
		start = EVENT_GENERATOR_ON;
		event = 0;
		break;
	case EVENT_SLIMBUS:
		selection = EVENT_SOURCE_COUNTER;
		start = EVENT_GENERATOR_ON;
		event = 0;
		break;
	case EVENT_DEFAULT:
		selection = EVENT_SOURCE_COUNTER;
		start = EVENT_GENERATOR_ON;
		event = 0;
		break;
	default:
		abe_dbg_param |= ERR_API;
		abe_dbg_error_log(ABE_BLOCK_COPY_ERR);
	}

	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_ATC, EVENT_GENERATOR_COUNTER, &counter, 4);
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_ATC, EVENT_SOURCE_SELECTION, &selection, 4);
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_ATC, EVENT_GENERATOR_START, &start, 4);
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_ATC, AUDIO_ENGINE_SCHEDULER, &event, 4);
}

/**
*  abe_read_use_case_opp() description for void abe_read_use_case_opp().
*
*  Operations : returns the expected min OPP for a given use_case list
*
*  Parameter  : No parameter
* @param
*
* @pre	  no pre-condition
*
* @post
*
* @return       error code
*
* @see
*/
void abe_read_use_case_opp(abe_use_case_id *u, abe_opp_t *o)
{
	abe_uint32 opp, i;
	abe_use_case_id *ptr = u;

	#define MAX_READ_USE_CASE_OPP 10	/* there is no reason to have more use_cases */
	#define OPP_25	1
	#define OPP_50	2
	#define OPP_100 4

	opp = i = 0;
	do {
		/* check for pointer errors */
		if (i > MAX_READ_USE_CASE_OPP) {
			abe_dbg_param |= ERR_API;
			abe_dbg_error_log(ABE_READ_USE_CASE_OPP_ERR);
			break;
		}

		/* check for end_of_list */
		if (*ptr <= 0)
			break;

		/* check for end_of_list */
		if (*ptr > ABE_LAST_USE_CASE)
			break;

		/* OPP selection based on current firmware implementation */
		switch (*ptr) {
		case ABE_AUDIO_PLAYER_ON_HEADSET_OR_EARPHONE:
			opp |= OPP_25;
			break;
		case ABE_DRIFT_MANAGEMENT_FOR_AUDIO_PLAYER:
			opp |= OPP_100;
			break;
		case ABE_VOICE_CALL_ON_HEADSET_OR_EARPHONE_OR_BT:
			opp |= OPP_50;
			break;
		case ABE_MULTIMEDIA_AUDIO_RECORDER:
			opp |= OPP_50;
			break;
		case ABE_VIBRATOR_OR_HAPTICS:
			opp |= OPP_100;
			break;
		case ABE_VOICE_CALL_ON_HANDS_FREE_SPEAKER:
			opp |= OPP_100;
			break;
		case ABE_RINGER_TONES:
			opp |= OPP_100;
			break;
		case ABE_VOICE_CALL_WITH_EARPHONE_ACTIVE_NOISE_CANCELLER:
			opp |= OPP_100;
			break;
		default:
			break;
		}

		i++;
		ptr++;
	} while (*ptr <= 0 || *ptr > ABE_LAST_USE_CASE);

	if (opp & OPP_100)
		*o = ABE_OPP100;
	else if (opp & OPP_50)
		*o = ABE_OPP50;
	else
		*o = ABE_OPP25;
}

/*
 *  ABE_READ_LOWEST_OPP
 *
 *  Parameter  :
 *      Data pointer : returned data
 *
 *  Operations :
 *      Returns the lowest possible OPP based on the current active ports
 *
 *  Return value :
 *      None.
 */
void abe_read_lowest_opp(abe_opp_t *o)
{
	*o = ABE_OPP100;	    /* @@@ RELEASE 5 allows only 100% OPP */

#if 0
	for (i = 0; i < LAST_PORT_ID; i++)

    abe_port_t abe_port [MAXNBABEPORTS];		    /* list of ABE ports */
const   abe_port_t abe_port_init [MAXNBABEPORTS] = {

			/* Status    Data Format       Drift     Call-Back			       Protocol+selector	    desc_addr;	     buf_addr;  buf_size;		iter;		  thr_low;       irq_addr	  irq_data      DMA_T				     #Features reseted at start	      Port Name for the debug trace */
/*      DMIC1   */      { IDLE_P, {96000, STEREO_MSB}, NODRIFT, NOCALLBACK, smem_dmic1, 0,	       {SNK_P, DMIC_PORT_PROT,			    {dmem_dmic, dmem_dmic_size, DMIC_ITER}},								   {0, 0},				   {EQDMIC1, 0},		   "DMIC_UL" },
/*      DMIC2   */      { IDLE_P, {96000, STEREO_MSB}, NODRIFT, NOCALLBACK, smem_dmic2, 0,	       {SNK_P, DMIC_PORT_PROT,			    {dmem_dmic, dmem_dmic_size, DMIC_ITER}},								   {0, 0},				   {EQDMIC2, 0},		   "DMIC_UL" },
/*      DMIC3   */      { IDLE_P, {96000, STEREO_MSB}, NODRIFT, NOCALLBACK, smem_dmic3, 0,	       {SNK_P, DMIC_PORT_PROT,			    {dmem_dmic, dmem_dmic_size, DMIC_ITER}},								   {0, 0},				   {EQDMIC3, 0},		   "DMIC_UL" },
/*      PDM_UL  */      { IDLE_P, {96000, STEREO_MSB}, NODRIFT, NOCALLBACK, smem_amic,  0,	       {SNK_P, MCPDMUL_PORT_PROT,			 {dmem_amic, dmem_amic_size, MCPDM_UL_ITER}},							       {0, 0},				   {EQAMIC, 0},		    "AMIC_UL" },
/*      BT_VX_UL*/      { IDLE_P, { 8000, STEREO_MSB}, NODRIFT, NOCALLBACK, smem_bt_vx_ul, 0,	    {SNK_P, SERIAL_PORT_PROT,   {MCBSP1_DMA_TX*ATC_SIZE,dmem_bt_vx_ul,dmem_bt_vx_ul_size,   1 * SCHED_LOOP_8kHz,  DEFAULT_THR_WRITE}},			    {0, 0},				   {0},			    "BT_VX_UL" },
/*      MM_UL   */      { IDLE_P, {48000, STEREO_MSB}, NODRIFT, NOCALLBACK, smem_mm_ul, 0,	       {SRC_P, DMAREQ_PORT_PROT,   {CBPr_DMA_RTX3*ATC_SIZE,dmem_mm_ul,dmem_mm_ul_size,	10 * SCHED_LOOP_48kHz, DEFAULT_THR_WRITE,ABE_DMASTATUS_RAW,(1<<3)}},   {CIRCULAR_BUFFER_PERIPHERAL_R__3, 120},   {UPROUTE, 0},		   "MM_UL" },
/*      MM_UL2  */      { IDLE_P, {48000, STEREO_MSB}, NODRIFT, NOCALLBACK, smem_mm_ul2,0,	       {SRC_P, DMAREQ_PORT_PROT,   {CBPr_DMA_RTX4*ATC_SIZE,dmem_mm_ul2,dmem_mm_ul2_size,       2 * SCHED_LOOP_48kHz, DEFAULT_THR_WRITE,ABE_DMASTATUS_RAW,(1<<4)}},   {CIRCULAR_BUFFER_PERIPHERAL_R__4,  24},   {UPROUTE, 0},		   "MM_UL2" },
/*      VX_UL   */      { IDLE_P, { 8000, MONO_MSB},   NODRIFT, NOCALLBACK, smem_vx_ul, 0,	       {SRC_P, DMAREQ_PORT_PROT,   {CBPr_DMA_RTX2*ATC_SIZE,dmem_vx_ul,(dmem_vx_ul_size/2),     1 * SCHED_LOOP_8kHz,  DEFAULT_THR_WRITE,ABE_DMASTATUS_RAW,(1<<2)}},   {CIRCULAR_BUFFER_PERIPHERAL_R__2,  4},    {ASRC2, 0},		     "VX_UL" },
/*      MM_DL   */      { IDLE_P, {48000, STEREO_MSB}, NODRIFT, NOCALLBACK, smem_mm_dl, 0,	       {SNK_P, DMAREQ_PORT_PROT,   {CBPr_DMA_RTX0*ATC_SIZE,dmem_mm_dl,dmem_mm_dl_size,	 2 * SCHED_LOOP_48kHz, DEFAULT_THR_READ ,ABE_DMASTATUS_RAW,(1<<0)}},   {CIRCULAR_BUFFER_PERIPHERAL_R__0, 24},    {ASRC3, 0},		     "MM_DL" },
/*      VX_DL   */      { IDLE_P, { 8000, MONO_MSB},   NODRIFT, NOCALLBACK, smem_vx_dl, 0,	       {SNK_P, DMAREQ_PORT_PROT,   {CBPr_DMA_RTX1*ATC_SIZE,dmem_vx_dl,dmem_vx_dl_size,	 1 * SCHED_LOOP_8kHz,  DEFAULT_THR_READ ,ABE_DMASTATUS_RAW,(1<<1)}},   {CIRCULAR_BUFFER_PERIPHERAL_R__1, 4},     {ASRC1, 0},		     "VX_DL" },
/*      TONES_DL*/      { IDLE_P, {48000, STEREO_MSB}, NODRIFT, NOCALLBACK, smem_tones_dl, 0,	    {SNK_P, DMAREQ_PORT_PROT,   {CBPr_DMA_RTX5*ATC_SIZE,dmem_tones_dl,dmem_tones_dl_size,   2 * SCHED_LOOP_48kHz, DEFAULT_THR_READ ,ABE_DMASTATUS_RAW,(1<<5)}},   {CIRCULAR_BUFFER_PERIPHERAL_R__5, 24},    {0},			    "TONES_DL" },
/*      VIB_DL  */      { IDLE_P, {24000, STEREO_MSB}, NODRIFT, NOCALLBACK, smem_vib, 0,		 {SNK_P, DMAREQ_PORT_PROT,   {CBPr_DMA_RTX6*ATC_SIZE,dmem_vib_dl,dmem_vib_dl_size,       2 * SCHED_LOOP_24kHz, DEFAULT_THR_READ ,ABE_DMASTATUS_RAW,(1<<6)}},   {CIRCULAR_BUFFER_PERIPHERAL_R__6, 12},    {0},			    "VIB_DL" },
/*      BT_VX_DL*/      { IDLE_P, { 8000, MONO_MSB},   NODRIFT, NOCALLBACK, smem_bt_vx_dl, 0,	    {SRC_P, SERIAL_PORT_PROT,   {MCBSP1_DMA_RX*ATC_SIZE,dmem_bt_vx_dl,dmem_bt_vx_dl_size,   1 * SCHED_LOOP_8kHz,  DEFAULT_THR_WRITE}},			    {0, 0},				   {0},			    "BT_VX_DL" },
/*      PDM_DL1 */      { IDLE_P, {96000, STEREO_MSB}, NODRIFT, NOCALLBACK, 0,0,			 {SRC_P, MCPDMDL_PORT_PROT,			 {dmem_mcpdm, dmem_mcpdm_size}},									    {0, 0},				   {MIXDL1, EQ1, APS1, 0},	 "PDM_DL1" },
/*      MM_EXT_OUT*/    { IDLE_P, {48000, STEREO_MSB}, NODRIFT, NOCALLBACK, smem_mm_ext_out, 0,	  {SRC_P, SERIAL_PORT_PROT,   {MCBSP1_DMA_RX*ATC_SIZE,dmem_mm_ext_out,dmem_mm_ext_out_size, 2 * SCHED_LOOP_48kHz, DEFAULT_THR_WRITE}},			  {0, 0},				   {0},			    "MM_EXT_OUT" },
/*      PDM_DL2 */      { IDLE_P, {96000, STEREO_MSB}, NODRIFT, NOCALLBACK, 0, 0,			{SRC_P, MCPDMDL_PORT_PROT,			 {0,0,0,0,0,0,0}},											  {0, 0},				   {MIXDL2, EQ2L, APS2L, EQ2R, APS2R, 0},"PDM_DL2" },
/*      PDM_VIB */      { IDLE_P, {24000, STEREO_MSB}, NODRIFT, NOCALLBACK, 0, 0,			{SRC_P, MCPDMDL_PORT_PROT,			 {0,0,0,0,0,0,0}},											  {0, 0},				   {0},			    "PDM_VIB" },
/*      SCHD_DBG_PORT*/ { RUN_P,  { 8000, MONO_MSB},   NODRIFT, NOCALLBACK, 0, 0,			{0, 0,					     {0,0,0,0,0,0,0}},											  {0, 0},				   {SEQUENCE, CONTROL, GAINS, 0},  "SCHD_DBG" },
			};

		DMIC_PORT1,	     /* digital microphones pairs */
		DMIC_PORT2,
		DMIC_PORT3,
		PDM_UL_PORT,	    /* analog MICs */
		BT_VX_UL_PORT,	  /* BT uplink */

					/* AE source ports - Uplink */
		MM_UL_PORT,	     /* up to 5 stereo channels */
		MM_UL2_PORT,	    /* stereo FM record path */
		VX_UL_PORT,	     /* stereo FM record path */

					/* AE sink ports - Downlink */
		MM_DL_PORT,	     /* multimedia player audio path */
		VX_DL_PORT,
		TONES_DL_PORT,
		VIB_DL_PORT,

		BT_VX_DL_PORT,	  /* AE source ports - Downlink */
		PDM_DL1_PORT,
		MM_EXT_OUT_PORT,
		PDM_DL2_PORT,
		PDM_VIB_PORT,

		SCHD_DBG_PORT,	  /* dummy port used to declare the other tasks of the scheduler */

		LAST_PORT_ID


    abe_uint32 opp = i = 0;
    abe_use_case_id *ptr = u;
    #define MAX_READ_USE_CASE_OPP 10	    /* there is no reason to have more use_cases */
    #define OPP_25  1
    #define OPP_50  2
    #define OPP_100 4

    while (1)
    {
	if (i > MAX_READ_USE_CASE_OPP)	  /* check for pointer errors */
	{
	    abe_dbg_param |= ERR_API;
	    abe_dbg_error_log(ABE_READ_USE_CASE_OPP_ERR)
	    break;
	}
	if (*ptr <= 0) break;		   /* check for end_of_list */
	if (*ptr > ABE_LAST_USE_CASE) break;    /* check for end_of_list */

	switch (*ptr)			   /* OPP selection based on current Firmware implementation */
	{
	    case ABE_AUDIO_PLAYER_ON_HEADSET_OR_EARPHONE :	      opp |= OPP_25; break;
	    case ABE_DRIFT_MANAGEMENT_FOR_AUDIO_PLAYER :		opp |= OPP_100; break;
	    case ABE_VOICE_CALL_ON_HEADSET_OR_EARPHONE_OR_BT :	  opp |= OPP_50; break;
	    case ABE_MULTIMEDIA_AUDIO_RECORDER :			opp |= OPP_50; break;
	    case ABE_VIBRATOR_OR_HAPTICS :			      opp |= OPP_100; break;
	    case ABE_VOICE_CALL_ON_HANDS_FREE_SPEAKER :		 opp |= OPP_100; break;
	    case ABE_RINGER_TONES :				     opp |= OPP_100; break;
	    case ABE_VOICE_CALL_WITH_EARPHONE_ACTIVE_NOISE_CANCELLER :  opp |= OPP_100; break;
	}
	i++;
	ptr++;
    }

    if (opp | OPP_100) *o = ABE_OPP100;
	else if (opp | OPP_50) *o = ABE_OPP50;
	    else *o = ABE_OPP25;

#endif
}

/*
 *  ABE_SET_OPP_PROCESSING
 *
 *  Parameter  :
 *      New processing network and OPP:
 *	  0: Ultra Lowest power consumption audio player (no post-processing, no mixer)
 *	  1: OPP 25% (simple multimedia features, including low-power player)
 *	  2: OPP 50% (multimedia and voice calls)
 *	  3: OPP100% (EANC, multimedia complex use-cases)
 *
 *  Operations :
 *      Rearranges the FW task network to the corresponding OPP list of features.
 *      The corresponding AE ports are supposed to be set/reset accordingly before this switch.
 *
 *  Return value :
 *      error code when the new OPP do not corresponds the list of activated features
 */
void abe_set_opp_processing(abe_opp_t opp)
{
	abe_uint32 scheduler_task_steps;

	switch (opp) {
	case ABE_OPP25:
		scheduler_task_steps = SCHED_TASK_STEP_OPP_25;
		break;
	case ABE_OPP50:
		scheduler_task_steps = SCHED_TASK_STEP_OPP_50;
		break;
	case ABE_OPP100:
		scheduler_task_steps = SCHED_TASK_STEP_OPP_100;
		break;
	case ABE_OPP0:
		/* default = OPP25% processing */
	default: scheduler_task_steps = SCHED_TASK_STEP_OPP_25;
		abe_dbg_param |= ERR_API;
		abe_dbg_error_log(ABE_BLOCK_COPY_ERR);
	}

	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, D_taskStep_ADDR, &scheduler_task_steps, sizeof(scheduler_task_steps));
}

/*
 *  ABE_SET_PING_PONG_BUFFER
 *
 *  Parameter  :
 *      Port_ID :
 *      New data
 *
 *  Operations :
 *      Updates the next ping-pong buffer with "size" samples copied from the
 *      host processor. This API notifies the FW that the data transfer is done.
 *      The size is added on top of the previous size.
 */
void abe_set_ping_pong_buffer(abe_port_id port, abe_uint32 n)
{
	abe_uint32 sio_pp_desc_address, struct_offset, *src, dst;
	ABE_SPingPongDescriptor desc_pp;

	/* ping_pong is only supported on MM_DL */
	if (port != MM_DL_PORT) {
	    abe_dbg_param |= ERR_API;
	    abe_dbg_error_log(ABE_PARAMETER_ERROR);
	}

	/* read the port SIO descriptor and extract the current pointer address  after reading the counter */
	sio_pp_desc_address = D_PingPongDesc_ADDR;
	abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM, sio_pp_desc_address, (abe_uint32*)&desc_pp, sizeof(ABE_SPingPongDescriptor));

	if ((desc_pp.counter & 0x1) == 0)
		desc_pp.nextbuff0_Samples = (abe_uint16) (n >> 2);     /* FW uses words numbers instead of bytes */
	else
		desc_pp.nextbuff1_Samples = (abe_uint16) (n >> 2);

	struct_offset = (abe_uint32)&(desc_pp.nextbuff0_BaseAddr) - (abe_uint32)&(desc_pp);
	src = (abe_uint32 *) &(desc_pp.nextbuff0_BaseAddr);
	dst = sio_pp_desc_address + struct_offset;
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, dst, src,
							sizeof(desc_pp) - struct_offset);
}

/*
 *  ABE_READ_NEXT_PING_PONG_BUFFER
 *
 *  Parameter  :
 *      Port_ID :
 *      Returned address to the next buffer (byte offset from DMEM start)
 *
 *  Operations :
 *      Tell the next base address of the next ping_pong Buffer and reset its size
 *
 *
 */
void abe_read_next_ping_pong_buffer(abe_port_id port, abe_uint32 *p, abe_uint32 *n)
{
	abe_uint32 sio_pp_desc_address;
	ABE_SPingPongDescriptor desc_pp;

	/* ping_pong is only supported on MM_DL */
	if (port != MM_DL_PORT) {
		abe_dbg_param |= ERR_API;
		abe_dbg_error_log(ABE_PARAMETER_ERROR);
	}

	/* read the port SIO descriptor and extract the current pointer address  after reading the counter */
	sio_pp_desc_address = D_PingPongDesc_ADDR;
	abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM, sio_pp_desc_address, (abe_uint32*)&desc_pp, sizeof(ABE_SPingPongDescriptor));

	if ((desc_pp.counter & 0x1) == 0) {
		(*p) = desc_pp.nextbuff0_BaseAddr;
		(*n) = desc_pp.nextbuff0_Samples;
	} else {
		(*p) = desc_pp.nextbuff1_BaseAddr;
		(*n) = desc_pp.nextbuff1_Samples;
	}
}

/*
 *  ABE_INIT_PING_PONG_BUFFER
 *
 *  Parameter  :
 *      size of the ping pong
 *      number of buffers (2 = ping/pong)
 *      returned address of the ping-pong list of base address (byte offset from DMEM start)
 *
 *  Operations :
 *      Computes the base address of the ping_pong buffers
 *
 */
void abe_init_ping_pong_buffer(abe_port_id id, abe_uint32 s, abe_uint32 n, abe_uint32 *p)
{
	abe_uint32 i, dmem_addr;

	abe_size_pingpong = s;
	abe_nb_pingpong = n;

	/* ping_pong is supported in 2 buffers configuration right now but FW is ready for ping/pong/pung/pang... */
	if (id != MM_DL_PORT || n > MAX_PINGPONG_BUFFERS) {
		abe_dbg_param |= ERR_API;
		abe_dbg_error_log(ABE_PARAMETER_ERROR);
	}

	for (i = 0; i < abe_nb_pingpong; i++) {
	    dmem_addr = dmem_ping_pong_buffer + (i * abe_size_pingpong);
	    abe_base_address_pingpong [i] = dmem_addr;   /* base addresses of the ping pong buffers in U8 unit */
	}

	*p = (abe_uint32)dmem_ping_pong_buffer;
}

/*
 *  ABE_PLUG_SUBROUTINE
 *
 *  Parameter :
 *  id: returned sequence index after plugging a new subroutine
 *  f : subroutine address to be inserted
 *  n : number of parameters of this subroutine
 *
 *  Returned value : error code
 *
 *  Operations : register a list of subroutines for call-back purpose
 *
 */
void abe_plug_subroutine(abe_uint32 *id, abe_subroutine2 f, abe_uint32 n, abe_uint32* params)
{
	/* debug trace */
	abe_add_subroutine (id, f, n, params);
}

/*
 *  ABE_PLUG_SEQUENCE
 *
 *  Parameter  :
 *       Id: returned sequence index after pluging a new sequence (index in the tables)
 *       s : sequence to be inserted
 *
 *  Operations :
 *      Load a time-sequenced operations.
 *
 *  Return value :
 *      None.
 */
void abe_plug_sequence(abe_uint32 *id, abe_sequence_t *s)
{
	just_to_avoid_the_many_warnings = *id;
	just_to_avoid_the_many_warnings_abe_sequence_t = *s;
}

/*
 *  ABE_SET_SEQUENCE_TIME_ACCURACY
 *
 *  Parameter  :
 *      patch bit field used to guarantee the code compatibility without conditionnal compilation
 *      Sequence index
 *
 *  Operations : two counters are implemented in the firmware:
 *  -	one "fast" counter, generating an IRQ to the HAL for sequences scheduling, the rate is in the range 1ms .. 100ms
 *  -	one "slow" counter, generating an IRQ to the HAL for the management of ASRC drift, the rate is in the range 1s .. 100s
 *
 *  Return value :
 *      None.
 */
void abe_set_sequence_time_accuracy(abe_micros_t fast, abe_micros_t slow)
{
	just_to_avoid_the_many_warnings_abe_micros_t = fast;
	just_to_avoid_the_many_warnings_abe_micros_t = slow;
}

/*
 *  ABE_LAUNCH_SEQUENCE
 *
 *  Parameter  :
 *      patch bit field used to guarantee the code compatibility without conditionnal compilation
 *      Sequence index
 *
 *  Operations :
 *      Launch a list a time-sequenced operations.
 *
 *  Return value :
 *      None.
 */
void abe_launch_sequence(abe_patch_rev patch, abe_uint32 n)
{
	just_to_avoid_the_many_warnings_abe_patch_rev = patch;
	just_to_avoid_the_many_warnings = n;
}

/*
 *  ABE_LAUNCH_SEQUENCE_PARAM
 *
 *  Parameter  :
 *      patch bit field used to guarantee the code compatibility without conditionnal compilation
 *      Sequence index
 *      Parameters to the programmable sequence
 *
 *  Operations :
 *      Launch a list a time-sequenced operations.
 *
 *  Return value :
 *      None.
 */
void abe_launch_sequence_param(abe_patch_rev patch, abe_uint32 n, abe_int32 *param1, abe_int32 *param2, abe_int32 *param3, abe_int32 *param4)
{
	just_to_avoid_the_many_warnings_abe_patch_rev = patch;
	just_to_avoid_the_many_warnings = *param1;
	just_to_avoid_the_many_warnings = *param2;
	just_to_avoid_the_many_warnings = *param3;
	just_to_avoid_the_many_warnings = *param4;
	just_to_avoid_the_many_warnings = n;
}

/*
 *  ABE_READ_ANALOG_GAIN_DL
 *
 *  Parameter  :
 *      gain value pointer
 *
 *  Operations :
 *      returns to the data pointer the gain value of the phoenix headset in case the
 *      dynamic extension is activated.
 *
 *  Return value :
 *      None.
 */
void abe_read_analog_gain_dl(abe_gain_t *a)
{
	just_to_avoid_the_many_warnings_abe_gain_t = *a;
}

/*
 *  ABE_READ_ANALOG_GAIN_UL
 *
 *  Parameter  :
 *      gain value pointer
 *
 *  Operations :
 *      returns to the data pointer the gain value of the phoenix headset in case the
 *	dynamic extension is activated.
 *
 *  Return value :
 *      None.
 */
void abe_read_analog_gain_ul(abe_gain_t *a)
{
	just_to_avoid_the_many_warnings_abe_gain_t = *a;
}

/*
 *  ABE_ENABLE_DYN_UL_GAIN
 *
 *  Parameter  :
 *      None.
 *
 *  Operations :
 *      enables the audio engine capability to change dynamically the analog microphone
 *      amplitude based on the information of analog gain changes.
 *
 *  Return value :
 *      None.
 */
void abe_enable_dyn_ul_gain(void)
{
}

/*
 *  ABE_DISABLE_DYN_UL_GAIN
 *
 *  Parameter  :
 *
 *
 *  Operations :
 *      disables the audio engine capability to change dynamically the analog microphone
 *      amplitude based on the information of analog gain changes.
 *
 *  Return value :
 *      None.
 */
void abe_disable_dyn_ul_gain(void)
{
}

/*
 *  ABE_ENABLE_DYN_EXTENSION
 *
 *  Parameter  :
 *      None
 *
 *  Operations :
 *      enables the audio engine capability to change the analog gain of Phoenix headset port.
 *      This feature enables dynamic range extension by increasing the digital gains and lowering
 *      the analog gains. This API is preferably called at reset time.
 *
 *  Return value :
 *      None.
 */
void abe_enable_dyn_extension(void)
{
}

/*
 *  ABE_DISABLE_DYN_EXTENSION
 *
 *  Parameter  :
 *      None
 *
 *  Operations :
 *      disables the audio engine capability to change the analog gain of Phoenix headset port.
 *
 *  Return value :
 *      None.
 */
void abe_disable_dyn_extension(void)
{
}

/*
 *  ABE_NOTIFY_ANALOG_GAIN_CHANGED
 *
 *  Parameter  :
 *      Id: name of the Phoenix (or other device) port for which a gain was changed
 *      G: pointer to the notified gain value
 *
 *  Operations :
 *      The upper layer tells the HAL a new gain was programmed in the analog renderer.
 *      This will help the tuning of the APS parameters.
 *
 *  Return value :
 *      None.
 */
void abe_notify_analog_gain_changed(abe_ana_port_id Id, abe_gain_t *G)
{
	just_to_avoid_the_many_warnings_abe_gain_t = *G;
	just_to_avoid_the_many_warnings_abe_ana_port_id = Id;
}

/*
 *  ABE_RESET_PORT
 *
 *  Parameters :
 *  id: port name
 *
 *  Returned value : error code
 *
 *  Operations : stop the port activity and reload default parameters on the associated processing features.
 *       Clears the internal AE buffers.
 *
 */
void abe_reset_port(abe_port_id id)
{
	abe_port [id] = ((abe_port_t *) abe_port_init) [id];
}

/*
 *  ABE_READ_REMAINING_DATA
 *
 *  Parameter  :
 *      Port_ID :
 *      size : pointer to the remaining number of 32bits words
 *
 *  Operations :
 *      computes the remaining amount of data in the buffer.
 *
 *  Return value :
 *      error code
 */
void abe_read_remaining_data(abe_port_id port, abe_uint32 *n)
{
	just_to_avoid_the_many_warnings = *n;
	just_to_avoid_the_many_warnings_abe_port_id = port;
}

/*
 *  ABE_DISABLE_DATA_TRANSFER
 *
 *  Parameter  :
 *      p: port indentifier
 *
 *  Operations :
 *      disables the ATC descriptor and stop IO/port activities
 *
 *  Return value :
 *      None.
 */
void abe_disable_data_transfer(abe_port_id id)
{
	/* local host variable status= "port is running" */
	abe_port[id].status = IDLE_P;
	/* disable DMA requests */
	abe_disable_dma_request (id);
	/* disable ATC transfers */
	abe_disable_atc (id);
}

/*
 *  ABE_ENABLE_DATA_TRANSFER
 *
 *  Parameter  :
 *      p: port indentifier
 *
 *  Operations :
 *      enables the ATC descriptor
 *
 *  Return value :
 *      None.
 */
void abe_enable_data_transfer(abe_port_id id)
{
	/* enable DMA requests */
	abe_enable_dma_request(id);
	/* enable ATC data path */
	abe_enable_atc(id);
	/* local host variable status= "port is running" */
	abe_port[id].status = RUN_P;

#if PC_SIMULATION
	if (id == DMIC_PORT1  || id == DMIC_PORT2 || id == DMIC_PORT3)
		target_server_activate_dmic();
	if (id == PDM_DL1_PORT || id == PDM_DL2_PORT || id == PDM_VIB_PORT)
		target_server_activate_mcpdm_dl();
	if (id == PDM_UL_PORT)
		target_server_activate_mcpdm_ul();
#endif
}

/*
 *  ABE_WRITE_CLOCK_MODE
 *
 *  Parameter  :
 *
 *  Operations :
 *      information from the upper layers of the clock configuration in Phoenix in
 *      order to correctly translate the global counter values in time.
 *
 *  Return value :
 *      None.
 */
void abe_write_clock_mode(abe_uint32 mode)
{
	just_to_avoid_the_many_warnings = mode;
}

/*
 *  ABE_READ_GLOBAL_COUNTER
 *
 *  Parameter  :
 *      data pointer to the counter value
 *      data pointer to the translated milliseconds
 *
 *  Operations :
 *      returns the value of the 32bits counter incremented on each firmware scheduling task
 *      loops (250us / 272us with respectively 48kHz / 44.1kHz on Phoenix). Translates this data
 *      in milli seconds format.
 *
 *  Return value :
 *      None.
 */
void abe_read_global_counter(abe_time_stamp_t *t, abe_millis_t *m)
{
	just_to_avoid_the_many_warnings_abe_time_stamp_t = *t;
	just_to_avoid_the_many_warnings_abe_millis_t = *m;
}

/*
 *  ABE_SET_DMIC_FILTER
 *
 *  Parameter  :
 *      DMIC decimation ratio : 16/25/32/40
 *
 *  Operations :
 *      Loads in CMEM a specific list of coefficients depending on the DMIC sampling
 *      frequency (2.4MHz or 3.84MHz). This table compensates the DMIC decimator roll-off at 20kHz.
 *      The default table is loaded with the DMIC 2.4MHz recommended configuration.
 *
 *  Return value :
 *      None.
 */
void abe_set_dmic_filter(abe_dmic_ratio_t d)
{
	just_to_avoid_the_many_warnings = (abe_uint32)d;
}

/**
* @fn abe_connect_cbpr_dmareq_port()
*
*  Operations : enables the data echange between a DMA and the ABE through the
*       CBPr registers of AESS.
*
*   Parameters :
*   id: port name
*   f : desired data format
*   d : desired dma_request line (0..7)
*   a : returned pointer to the base address of the CBPr register and number of
*       samples to exchange during a DMA_request.
*
* @see	ABE_API.h
*/
void abe_connect_cbpr_dmareq_port(abe_port_id id, abe_data_format_t *f, abe_uint32 d, abe_dma_t *returned_dma_t)
{
	abe_port[id] = ((abe_port_t *)abe_port_init)[id];

	abe_port[id].format = *f;
	abe_port[id].protocol.protocol_switch = DMAREQ_PORT_PROT;
	abe_port[id].protocol.p.prot_dmareq.iter = abe_dma_port_iteration(f);
	abe_port[id].protocol.p.prot_dmareq.dma_addr = ABE_DMASTATUS_RAW;
	abe_port[id].protocol.p.prot_dmareq.dma_data = (1 << d);

	abe_port[id].status = RUN_P;

	/* load the micro-task parameters */
	abe_init_io_tasks(id, &((abe_port [id]).format), &((abe_port [id]).protocol));

	/* load the dma_t with physical information from AE memory mapping */
	abe_init_dma_t(id, &((abe_port [id]).protocol));

	/* load the ATC descriptors - disabled */
	abe_init_atc(id);

	/* return the dma pointer address */
	abe_read_port_address(id, returned_dma_t);
}

/**
* @fn abe_connect_dmareq_port()
*
*  Operations : enables the data echanges between a DMA and a direct access to
*   the DMEM memory of ABE. On each dma_request activation the DMA will exchange
*   "iter" bytes and rewind its pointer to the base address "l3" waiting for the
*   next activation. The scheme is used for the MM_UL and debug port
*
*   Parameters :
*   id: port name
*   f : desired data format
*   d : desired dma_request line (0..7)
*   a : returned pointer to the base address of the ping-pong buffer and number
*       of samples to exchange during a DMA_request..
*
* @see	ABE_API.h
*/
void abe_connect_dmareq_port(abe_port_id id, abe_data_format_t *f, abe_uint32 d, abe_dma_t *a)
{
}

/**
* @fn abe_connect_dmareq_ping_pong_port()
*
*  Operations : enables the data echanges between a DMA and a direct access to
*   the DMEM memory of ABE. On each dma_request activation the DMA will exchange
*   "s" bytes and switch to the "pong" buffer for a new buffer exchange.
*
*    Parameters :
*    id: port name
*    f : desired data format
*    d : desired dma_request line (0..7)
*    s : half-buffer (ping) size
*
*    a : returned pointer to the base address of the ping-pong buffer and number of samples to exchange during a DMA_request.
*
* @see	ABE_API.h
*/
void abe_connect_dmareq_ping_pong_port(abe_port_id id, abe_data_format_t *f, abe_uint32 d, abe_uint32 s, abe_dma_t *returned_dma_t)
{
	abe_dma_t dma1;

	/* ping_pong is only supported on MM_DL */
	if (id != MM_DL_PORT) {
		abe_dbg_param |= ERR_API;
		abe_dbg_error_log(ABE_PARAMETER_ERROR);
	}

	/* declare PP buffer and prepare the returned dma_t */
	abe_init_ping_pong_buffer(MM_DL_PORT, s, 2, (abe_uint32 *)&(returned_dma_t->data));

	abe_port [id]				   = ((abe_port_t *) abe_port_init) [id];

	(abe_port [id]).format			    = (*f);
	(abe_port [id]).protocol.protocol_switch	  = PINGPONG_PORT_PROT;
	(abe_port [id]).protocol.p.prot_pingpong.buf_addr = dmem_ping_pong_buffer;
	(abe_port [id]).protocol.p.prot_pingpong.buf_size = s;
	(abe_port [id]).protocol.p.prot_pingpong.irq_addr = ABE_DMASTATUS_RAW;
	(abe_port [id]).protocol.p.prot_pingpong.irq_data = (1 << d);

	abe_port [id].status = RUN_P;

	/* load the micro-task parameters DESC_IO_PP */
	abe_init_io_tasks(id, &((abe_port [id]).format), &((abe_port [id]).protocol));

	/* load the dma_t with physical information from AE memory mapping */
	abe_init_dma_t(id, &((abe_port [id]).protocol));

	dma1.data = (abe_uint32 *)(abe_port [id].dma.data + ABE_DMEM_BASE_ADDRESS_L3);
	dma1.iter = abe_port [id].dma.iter;
	(*returned_dma_t) = dma1;
}

/**
* @fn abe_connect_irq_ping_pong_port()
*
*  Operations : enables the data echanges between a direct access to the DMEM
*       memory of ABE using cache flush. On each IRQ activation a subroutine
*       registered with "abe_plug_subroutine" will be called. This subroutine
*       will generate an amount of samples, send them to DMEM memory and call
*       "abe_set_ping_pong_buffer" to notify the new amount of samples in the
*       pong buffer.
*
*    Parameters :
*       id: port name
*       f : desired data format
*       I : index of the call-back subroutine to call
*       s : half-buffer (ping) size
*
*       p: returned base address of the first (ping) buffer)
*
* @see	ABE_API.h
*/
void abe_connect_irq_ping_pong_port(abe_port_id id, abe_data_format_t *f,
			abe_uint32 d, abe_uint32 s, abe_uint32 *p, abe_uint32 dsp_mcu_flag)
{
	/* ping_pong is only supported on MM_DL */
	if (id != MM_DL_PORT) {
	    abe_dbg_param |= ERR_API;
	    abe_dbg_error_log(ABE_PARAMETER_ERROR);
	}

	/* declare PP buffer for mono 20ms = 960 samples */
	abe_init_ping_pong_buffer(MM_DL_PORT, s, 2, p);

	abe_port [id]				   = ((abe_port_t *) abe_port_init) [id];
	(abe_port [id]).format			    = (*f);
	(abe_port [id]).protocol.protocol_switch	  = PINGPONG_PORT_PROT;
	(abe_port [id]).protocol.p.prot_pingpong.buf_addr = dmem_ping_pong_buffer;
	(abe_port [id]).protocol.p.prot_pingpong.buf_size = s;
	(abe_port [id]).protocol.p.prot_pingpong.irq_data = (1);

	if (dsp_mcu_flag == PING_PONG_WITH_MCU_IRQ)
		(abe_port [id]).protocol.p.prot_pingpong.irq_addr = ABE_MCU_IRQSTATUS_RAW;

	if (dsp_mcu_flag == PING_PONG_WITH_DSP_IRQ)
		(abe_port [id]).protocol.p.prot_pingpong.irq_addr = ABE_DSP_IRQSTATUS_RAW;

	abe_port [id].status = RUN_P;

	/* load the micro-task parameters */
	abe_init_io_tasks(id, &((abe_port [id]).format), &((abe_port [id]).protocol));

	/* load the ATC descriptors - disabled */
	abe_init_atc(id);

	(*p)= (abe_port [id]).protocol.p.prot_pingpong.buf_addr;
}

/**
* @fn abe_connect_serial_port()
*
*  Operations : enables the data echanges between a McBSP and an ATC buffer in
*   DMEM. This API is used connect 48kHz McBSP streams to MM_DL and 8/16kHz
*   voice streams to VX_UL, VX_DL, BT_VX_UL, BT_VX_DL. It abstracts the
*   abe_write_port API.
*
*   Parameters :
*    id: port name
*    f : data format
*    i : peripheral ID (McBSP #1, #2, #3)
*
* @see	ABE_API.h
*/
void abe_connect_serial_port(abe_port_id id, abe_data_format_t *f, abe_uint32 i)
{
#if 0
	abe_port [id] = ((abe_port_t *) abe_port_init) [id];
	(abe_port [id]).format = (*f);
	(abe_port [id]).protocol.protocol_switch = SERIAL_PORT_PROT;

	abe_port [id].status = RUN_P;

	/* load the micro-task parameters */
	abe_init_io_tasks(id, &((abe_port [id]).format), &((abe_port [id]).protocol));

	/* load the ATC descriptors - disabled */
	abe_init_atc(id);
#endif
}

/*
 *  ABE_WRITE_PORT_DESCRIPTOR
 *
 *  Parameter  :
 *      id: port name
 *      f : input pointer to the data format
 *      p : input pointer to the protocol description
 *      dma : output pointer to the DMA iteration and data destination pointer :
 *
 *  Operations :
 *      writes the corresponding port descriptor in the AE memory spaces. The ATC DMEM descriptors
 *      are initialized.
 *      - translates the data format to AE I/O task format
 *      - copy to DMEM
 *      - load ATC descriptor - disabled
 *
 *  Return value :
 *      None.
 */
void abe_write_port_descriptor(abe_port_id id, abe_data_format_t *format, abe_port_protocol_t *prot, abe_dma_t *dma)
{
#if 0
	abe_dbg_log(ID_WRITE_PORT_DESCRIPTOR);

	if (abe_port [id].status)
		abe_dbg_log(ABE_PORT_REPROGRAMMING);


	// load internal HAL data

	abe_port [id].status = RUN_P;			   // running status
	abe_port [id].format = (*format);		       // format = input parameter
	abe_port [id].drift = 0;				// no drift yet
	abe_port [id].callback = NOCALLBACK;
	abe_port [id].protocol = (*prot);		       // protocol = input parameter


	// load AE memory space

	abe_init_io_tasks(id, format, prot);		   // load the micro-task parameters
	abe_init_dma_t(id, prot);			      // load the dma_t with physical information from AE memory mapping
	abe_init_atc(id);				      // load the ATC descriptors - disabled
	abe_read_port_address(id, dma);			// return the dma pointer address

#endif
}

/*
 *  ABE_READ_PORT_DESCRIPTOR
 *
 *  Parameter  :
 *      id: port name
 *      f : input pointer to the data format
 *      p : input pointer to the protocol description
 *      dma : output pointer to the DMA iteration and data destination pointer :
 *
 *  Operations :
 *      returns the port parameters from the HAL internal buffer.
 *
 *  Return value :
 *      error code in case the Port_id is not compatible with the current OPP value
 */
void abe_read_port_descriptor(abe_port_id port, abe_data_format_t *f, abe_port_protocol_t *p)
{
	(*f)    = (abe_port[port]).format;
	(*p)    = (abe_port[port]).protocol;
}

/*
 *  ABE_READ_APS_ENERGY
 *
 *  Parameter  :
 *      Port_ID : port ID supporting APS
 *      APS data struct pointer
 *
 *  Operations :
 *      Returns the estimated amount of energy
 *
 *  Return value :
 *      error code when the Port is not activated.
 */
void abe_read_aps_energy(abe_port_id *p, abe_gain_t *a)
{
	just_to_avoid_the_many_warnings_abe_port_id = *p;
	just_to_avoid_the_many_warnings_abe_gain_t = *a;
}

/*
 *  ABE_READ_PORT_ADDRESS
 *
 *  Parameter  :
 *      dma : output pointer to the DMA iteration and data destination pointer
 *
 *  Operations :
 *      This API returns the address of the DMA register used on this audio port.
 *      Depending on the protocol being used, adds the base address offset L3 (DMA) or MPU (ARM)
 *
 *  Return value :
 */
void abe_read_port_address(abe_port_id port, abe_dma_t *dma2)
{
	abe_dma_t_offset dma1;
	abe_uint32 protocol_switch;

	dma1  = (abe_port[port]).dma;
	protocol_switch = abe_port[port].protocol.protocol_switch;

	switch (protocol_switch) {
	case PINGPONG_PORT_PROT:
		dma1.data =  dma1.data + ABE_DMEM_BASE_ADDRESS_L3;
		break;
	case DMAREQ_PORT_PROT:
//		if (MM_UL_PORT == port)				     /* special behavior for the MM_UL port */
//		    dma1.data =  dma1.data + ABE_DMEM_BASE_ADDRESS_L3;      /*   DMA points directly to the DMEM instead of CBPr register */
//		else
		dma1.data =  dma1.data + ABE_ATC_BASE_ADDRESS_L3;
		break;
	case CIRCULAR_PORT_PROT:
		dma1.data =  dma1.data + ABE_ATC_BASE_ADDRESS_L3;
		break;
	case SLIMBUS_PORT_PROT:
	case SERIAL_PORT_PROT:
	case DMIC_PORT_PROT:
	case MCPDMDL_PORT_PROT:
	case MCPDMUL_PORT_PROT:
	default:
		break;
	}

	(*dma2).data  = (void *)(dma1.data);
	(*dma2).iter  = (dma1.iter);
}

/*
 *  ABE_WRITE_PORT_GAIN
 *
 *  Parameter  :
 *      port : name of the port (VX_DL_PORT, MM_DL_PORT, MM_EXT_DL_PORT, TONES_DL_PORT, …)
 *      dig_gain_port pointer to returned port gain and time constant
 *
 *  Operations :
 *      saves the gain data in the local HAL-L0 table of gains in native format.
 *      Translate the gain to the AE-FW format and load it in CMEM
 *
 *  Return value :
 *      error code in case the gain_id is not compatible with the current OPP value.
 */
void abe_write_port_gain(abe_port_id port, abe_gain_t g, abe_ramp_t ramp)
{
#if 0
	abe_uint32 port_coef, lin_g, iir_ramp1, iir_ramp1MA;
	abe_float f_g, f_lin_g, f_ramp, f_iir_ramp1;

	switch (id) {
	case PDM_UL_PORT:
		port_coef = cmem_port_pdmul;
		break;
	case BT_VX_UL_PORT:
		port_coef = cmem_port_btul;
		break;
	case MM_UL_PORT:
		port_coef = cmem_port_mmul;
		break;
	case MM_UL2_PORT:
		port_coef = cmem_port_mmul2;
		break;
	case VX_UL_PORT:
		port_coef = cmem_port_vxul;
		break;
	case MM_DL_PORT:
		port_coef = cmem_port_pdmdl;
		break;
	case VX_DL_PORT:
		port_coef = cmem_port_vxdl;
		break;
	case TONES_DL_PORT:
		port_coef = cmem_port_tones;
		break;
	case VIB_DL_PORT:
		port_coef = cmem_port_vibdl;
		break;
	case BT_VX_DL_PORT:
		port_coef = cmem_port_btdl;
		break;
	case PDM_DL1_PORT:
		port_coef = cmem_port_pdmdl1;
		break;
	case MM_EXT_OUT_PORT:
		port_coef = cmem_port_mmext;
		break;
	case PDM_DL2_PORT:
		port_coef = cmem_port_pdmdl2;
		break;
	case PDM_VIB_PORT:
		port_coef = cmem_port_pdmvib;
		break;
	case DMIC_PORT2:
		port_coef = cmem_port_dmic2;
		break;
	case DMIC_PORT3:
		port_coef = cmem_port_dmic3;
		break;
	case DMIC_PORT1:
	default:
		port_coef = cmem_port_dmic1;
		break;
	}

	f_g = (abe_float)g;
	f_ramp = (abe_float)ramp;

	abe_translate_gain_format(MILLIBELS_TO_LINABE, f_g, &f_lin_g);
	abe_translate_to_xmem_format(ABE_CMEM, f_lin_g, &lin_g);

	abe_translate_ramp_format(f_ramp, &f_iir_ramp1);
	abe_translate_to_xmem_format(ABE_CMEM, f_iir_ramp1, &iir_ramp1);
	abe_translate_to_xmem_format(ABE_CMEM, (1 - f_iir_ramp1), &iir_ramp1MA);

	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_CMEM, 4*port_coef, (abe_uint32*)&lin_g, sizeof(lin_g));
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_CMEM, 4*cmem_ramp_alpha, (abe_uint32*)&iir_ramp1, sizeof(iir_ramp1));
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_CMEM, 4*cmem_ramp_1_M_alpha, (abe_uint32*)&iir_ramp1MA, sizeof(iir_ramp1MA));
#endif
}

/*
 *  ABE_READ_GAIN
 *
 *  Parameter  :
 *      port : name of the port (VX_DL_PORT, MM_DL_PORT, MM_EXT_DL_PORT, TONES_DL_PORT, …)
 *      dig_gain_port pointer to returned port gain and time constant
 *
 *  Operations :
 *      returns the real-time value of the gain from CMEM.
 *
 *  Return value :
 *      error code in case the gain_id is not compatible with the current OPP value.
 */
void abe_read_port_gain(abe_port_id port, abe_gain_t *gain, abe_ramp_t *ramp)
{
	just_to_avoid_the_many_warnings_abe_port_id = port;
	just_to_avoid_the_many_warnings_abe_gain_t = *gain;
	just_to_avoid_the_many_warnings_abe_ramp_t = *ramp;
}

/*
 *  ABE_READ_GAIN_RANGES
 *
 *  Parameter  :
 *      Id  : name of the AE port
 *      Gain data pointer to the minimum, maximum gain,
 *	gain steps for the overall digital and/or analog hardware.
 *
 *  Operations :
 *      returns the gain steps accuracy and range. If the API abe_enable_dyn_extension
 *      was called, the ports connected to Phoenix McPDM will also incorporate the gains
 *      of the analog part, else only the digital part is taken into account.
 *
 *  Return value :
 *      None.
 */
void abe_read_gain_range(abe_port_id id, abe_gain_t *min, abe_gain_t *max, abe_gain_t *step)
{
	just_to_avoid_the_many_warnings_abe_port_id = id;
	just_to_avoid_the_many_warnings_abe_gain_t = *min;
	just_to_avoid_the_many_warnings_abe_gain_t = *max;
	just_to_avoid_the_many_warnings_abe_gain_t = *step;
}

/*
 *  ABE_WRITE_EQUALIZER
 *
 *  Parameter  :
 *      Id  : name of the equalizer
 *      Param : equalizer coefficients
 *
 *  Operations :
 *      Load the coefficients in CMEM. This API can be called when the corresponding equalizer
 *      is not activated. After reloading the firmware the default coefficients corresponds to
 *      "no equalizer feature". Loading all coefficients with zeroes disables the feature.
 *
 *  Return value :
 *      None.
 */
void abe_write_equalizer(abe_equ_id id, abe_equ_t *param)
{
#if 0
	setCM(C_DMIC1_EQ_Coefs_ADDR, C_DMIC1_EQ_Coefs_ADDR+16, pGain, Sth);
	setCM(C_DMIC2_EQ_Coefs_ADDR, C_DMIC2_EQ_Coefs_ADDR+16, pGain, Sth);
	setCM(C_DMIC3_EQ_Coefs_ADDR, C_DMIC3_EQ_Coefs_ADDR+16, pGain, Sth);
	setCM(C_SDT_Coefs_ADDR, C_SDT_Coefs_ADDR+16, pGain, Sth);
	setCM(C_DL1_Coefs_ADDR, C_DL1_Coefs_ADDR+16, pGain, Sth);

	setCM(C_DL2_R_Coefs_ADDR, C_DL2_R_Coefs_ADDR+16, pGain, Sth);
	setCM(C_DL2_L_Coefs_ADDR, C_DL2_L_Coefs_ADDR+16, pGain, Sth)
#endif
}

/*
 * ABE_SET_ASRC_DRIFT_CONTROL
 *
 *  Parameter  :
 *      Id  : name of the asrc
 *      f: flag which enables (1) the automatic computation of drift parameters
 *
 *  Operations :
 *      When an audio port is connected to an hardware peripheral (MM_DL connected to a McBSP for
 *      example), the drift compensation can be managed in "forced mode" (f=0) or "adaptive mode"
 *      (f=1). In the first case the drift is managed with the usage of the API "abe_write_asrc".
 *      In the second case the firmware will generate on periodic basis an information about the
 *      observed drift, the HAL will reload the drift parameter based on those observations.
 *
 *  Return value :
 *      None.
 */
void abe_set_asrc_drift_control(abe_asrc_id id, abe_uint32 f)
{
}

/*
 * ABE_WRITE_ASRC
 *
 *  Parameter  :
 *      Id  : name of the asrc
 *      param : drift value t compensate
 *
 *  Operations :
 *      Load the drift coefficients in FW memory. This API can be called when the corresponding
 *      ASRC is not activated. After reloading the firmware the default coefficients corresponds
 *      to "no ASRC activated". Loading the drift value with zero disables the feature.
 *
 *  Return value :
 *      None.
 */
void abe_write_asrc(abe_asrc_id id, abe_drift_t *param)
{
#if 0
    int       dTempValue;
    int       aTempValue32[8];

    /*
     * x = ppm
     *
     *   - 1000000/x must be multiple of 16
     *   - DeltaAlpha = round(2^20*x*16/1000000)=round(2^18/5^6*x) on 22 bits. Then shifted by 2
     *   - MinusDeltaAlpha
     *   - OneMinusEpsilon = 1-DeltaAlpha/2.
     *
     *   PPM = 250
     *   - 1000000/250=4000
     *   - DeltaAlpha = 4194.3 ~ 4195 => 0x00418C
     */

    if (dPpm != 0)
    {
	/* Default value for -6250 ppm */
	aTempValue32[0] = 4;	  /* D_ConstAlmost0 */
	aTempValue32[1] = -1;	  /* D_DriftSign */
	aTempValue32[2] = 15;	 /* D_Subblock */
	aTempValue32[3] = 0x00066668; /* D_DeltaAlpha */
	aTempValue32[4] = 0xFFF99998; /* D_MinusDeltaAlpha */
	aTempValue32[5] = 0x003CCCCC; /* D_OneMinusEpsilon */
	aTempValue32[6] = 0x00000000; /* D_AlphaZero */
	aTempValue32[7] = 0x00400000; /* D_BetaOne */

	/* compute new value for the PPM */
	if (dPpm > 0){
	    aTempValue32[1] = 1;	   /* D_DriftSign */
	} else {
	    aTempValue32[1] = -1;	  /* D_DriftSign */
	}

	dTempValue = (int)(ceil(262144.0/15625.0*abs(dPpm)));
	aTempValue32[3] = dTempValue<<2;
	aTempValue32[4] = (-dTempValue)<<2;
	aTempValue32[5] = (0x00100000-(dTempValue/2))<<2;

	switch (dPath) {
	case 0x0000:
	    /* Voice Uplink */
	    /* Init CMem */
	    dTempValue = 0x00000000;
	    setCM(C_AlphaCurrent_UL_VX_ADDR, C_AlphaCurrent_UL_VX_ADDR, &dTempValue, Sth);
	    dTempValue = 0x00400000;
	    setCM(C_BetaCurrent_UL_VX_ADDR,  C_BetaCurrent_UL_VX_ADDR, &dTempValue, Sth);

	    /* Init DMem */
	    setlDM(D_AsrcVars_UL_VX_ADDR,  D_AsrcVars_UL_VX_ADDR+28, &aTempValue32[0], Sth);
	    break;
	case 0x0001:
	    /* Voice Downlink */
	    /* Init CMem */
	    dTempValue = 0x00000000;
	    setCM(C_AlphaCurrent_DL_VX_ADDR, C_AlphaCurrent_DL_VX_ADDR, &dTempValue, Sth);
	    dTempValue = 0x00400000;
	    setCM(C_BetaCurrent_DL_VX_ADDR,  C_BetaCurrent_DL_VX_ADDR, &dTempValue, Sth);

	    /* Init DMem */
	    setlDM(D_AsrcVars_DL_VX_ADDR,  D_AsrcVars_DL_VX_ADDR+28, &aTempValue32[0], Sth);
	    break;
	case 0x0002:
	    /* Multimedia Downlink */
	    /* Init CMem */
	    dTempValue = 0x00000000;
	    setCM(C_AlphaCurrent_DL_MM_ADDR, C_AlphaCurrent_DL_MM_ADDR, &dTempValue, Sth);
	    dTempValue = 0x00400000;
	    setCM(C_BetaCurrent_DL_MM_ADDR,  C_BetaCurrent_DL_MM_ADDR, &dTempValue, Sth);

	    /* Init DMem */
	    setlDM(D_AsrcVars_DL_MM_ADDR,  D_AsrcVars_DL_MM_ADDR+28, &aTempValue32[0], Sth);
	    break;
	}
#endif
}

/*
 *  ABE_WRITE_APS
 *
 *  Parameter  :
 *      Id  : name of the aps filter
 *      param : table of filter coefficients
 *
 *  Operations :
 *      Load the filters and thresholds coefficients in FW memory. This API can be called when
 *      the corresponding APS is not activated. After reloading the firmware the default coefficients
 *      corresponds to "no APS activated". Loading all the coefficients value with zero disables
 *      the feature.
 *
 *  Return value :
 *      None.
 */
void abe_write_aps(abe_aps_id id, abe_aps_t *param)
{
}

/*
 *  ABE_WRITE_MIXER
 *
 *  Parameter  :
 *  Id  : name of the mixer
 *  param : list of input gains of the mixer
 *  p : list of port corresponding to the above gains
 *
 *  Operations :
 *      Load the gain coefficients in FW memory. This API can be called when the corresponding
 *      MIXER is not activated. After reloading the firmware the default coefficients corresponds
 *      to "all input and output mixer's gain in mute state". A mixer is disabled with a network
 *      reconfiguration corresponding to an OPP value.
 *
 *  Return value :
 *      None.
 */
void abe_write_mixer(abe_mixer_id id, abe_gain_t f_g, abe_ramp_t f_ramp, abe_port_id p)
{
        abe_uint32 lin_g, mixer_coef;
	abe_int32 gain_index;

	gain_index = ((f_g - min_mdb) / 100);
	gain_index = maximum (gain_index, 0);
	gain_index = minimum (gain_index, sizeof_db2lin_table);

	lin_g = abe_db2lin_table [gain_index];

	switch (id) {
	default:
	case MIXDL1:
		mixer_coef = cmem_mixer_dl1;
		break;
	case MIXDL2:
		mixer_coef = cmem_mixer_dl2;
		break;
	case MIXSDT:
		mixer_coef = cmem_mixer_sdt;
		break;
	case MIXECHO:
		mixer_coef = cmem_mixer_echo;
		break;
	case MIXAUDUL:
		mixer_coef = cmem_mixer_audul;
		break;
	case MIXVXREC:
		mixer_coef = cmem_mixer_vxrec;
		break;
	}

	/* to be changed when all the gains are managed with Ramps */
#if 0
	abe_translate_gain_format(DECIBELS_TO_LINABE, f_g, &f_lin_g);
	abe_translate_to_xmem_format(ABE_CMEM, f_lin_g, &lin_g);

	/* there are no RAMPS on Mixers' gain in FW05.xx */
	abe_translate_ramp_format(f_ramp, &f_iir_ramp1);
	abe_translate_to_xmem_format(ABE_CMEM, f_iir_ramp1, &iir_ramp1);
	abe_translate_to_xmem_format(ABE_CMEM, (1 - f_iir_ramp1), &iir_ramp1MA);
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_CMEM, (cmem_ramp_alpha + p) * 4, (abe_uint32*)&iir_ramp1, sizeof(iir_ramp1));
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_CMEM, (cmem_ramp_1_M_alpha + p) * 4, (abe_uint32*)&iir_ramp1MA, sizeof(iir_ramp1MA));
#endif

	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_CMEM, (mixer_coef + p) * 4, (abe_uint32*)&lin_g, sizeof(lin_g));
}

/*
 *  ABE_WRITE_EANC
 *
 *  Parameter  :
 *      param : table of coefficients
 *
 *  Operations :
 *      Load the EANC coefficients in FW memory. This API can be called when EANC is not
 *      activated. After reloading the firmware the default coefficients corresponds to
 *      "no EANC activated". Loading the coefficients value with zero disables the feature.
 *
 *  Return value :
 *      None.
 */
void abe_write_eanc(abe_eanc_t *param)
{
}

/*
 *  ABE_ENABLE_ROUTER_CONFIGURATION
 *
 *  Parameter  :
 *      Id  : name of the router
 *      param : list of output index of the route
 *
 *  Operations : load a pre-computed router configuration
 *
 *  Return value :
 *      None.
 */
void abe_enable_router_configuration(abe_router_id id, abe_uint32 configuration)
{
}

/*
 *  ABE_SET_ROUTER_CONFIGURATION
 *
 *  Parameter  :
 *      Id  : name of the router
 *      Conf : id of the configuration
 *      param : list of output index of the route
 *
 *  Operations :
 *      The uplink router takes its input from DMIC (6 samples), AMIC (2 samples) and
 *      PORT1/2 (2 stereo ports). Each sample will be individually stored in an intermediate
 *      table of 10 elements. The intermediate table is used to route the samples to
 *      three directions : REC1 mixer, 2 EANC DMIC source of filtering and MM recording audio path.
 *      For example, a use case consisting in AMIC used for uplink voice communication, DMIC 0,1,2,3
 *      used for multimedia recording, , DMIC 5 used for EANC filter, DMIC 4 used for the feedback channel,
 *      will be implemented with the following routing table index list :
 *      [3, 2 , 1, 0, 0, 0 (two dummy indexes to data that will not be on MM_UL), 4, 5, 7, 6]
 *
 *  Return value :
 *      None.
 */
void abe_set_router_configuration(abe_router_id id, abe_uint32 configuration, abe_router_t *param)
{
	abe_uint8 aUplinkMuxing[16], n, i;

	n = D_aUplinkRouting_ADDR_END - D_aUplinkRouting_ADDR + 1;

	for(i=0; i < n; i++)
		aUplinkMuxing[i] = (abe_uint8) (param [i]);

	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, D_aUplinkRouting_ADDR, (abe_uint32 *)aUplinkMuxing, sizeof (aUplinkMuxing));
}

/*
 *  ABE_READ_ASRC
 *
 *  Parameter  :
 *      Id  : name of the asrc
 *      param : drift value to compensate
 *
 *
 *  Operations :
 *      read drift ppm value
 *
 *  Return value :
 *      None.
 */
void abe_read_asrc(abe_asrc_id id, abe_drift_t *param)
{
}

/*
 *  ABE_READ_APS
 *
 *  Parameter  :
 *      Id  : name of the aps filter
 *      param : table of filter coefficients
 *
 *  Operations :
 *      Read APS coefficients
 *
 *  Return value :
 *      None.
 */
void abe_read_aps(abe_aps_id id, abe_aps_t *param)
{
}

/*
 *  ABE_READ_MIXER
 *
 *  Parameter  :
 *      Id  : name of the mixer
 *      param : gain
 *      port corresponding to the above gains
 *
 *
 *  Return value :
 *      None.
 */
void abe_read_mixer(abe_mixer_id id, abe_gain_t *gain, abe_ramp_t *f_ramp, abe_port_id p)
{
	abe_uint32 offset; //, lin_g, iir_ramp;

	switch (id) {
	case MIXSDT:
		offset = cmem_mixer_sdt;
		break;
	case MIXECHO:
		offset = cmem_mixer_echo;
		break;
	case MIXAUDUL:
		offset = cmem_mixer_audul;
		break;
	case MIXVXREC:
		offset = cmem_mixer_vxrec;
		break;
	case MIXDL2:
		offset = cmem_mixer_dl2;
		break;
	case MIXDL1:
	default:
		offset = cmem_mixer_dl1;
		break;
	}

//	abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM, 4 * (cmem_gain + offsset + p), (abe_uint32*)&lin_g, sizeof(lin_g));
//	abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM, 4 * (cmem_ramp + offsset + p), (abe_uint32*)&iir_ramp, sizeof(iir_ramp));
//
//	abe_translate_gain_format(LINABE_TO_MILLIBELS, &lin_g, &g);
//	abe_translate_gain_format(IIRABE_TO_MICROS, &iir_ramp, &ramp);
}

/*
 *  ABE_READ_EANC
 *
 *  Parameter  :
 *      param : table of coefficients
 *
 *  Operations :
 *      read eanc coefficients
 *
 *  Return value :
 *      None.
 */
void abe_read_eanc(abe_eanc_t *param)
{
}

/*
 *  ABE_READ_ROUTER
 *
 *  Parameter  :
 *      Id  : name of the router
 *      param : list of output index of the route
 *
 *  Operations :
 *
 *
 *  Return value :
 *      None.
 */
void abe_read_router(abe_router_id id, abe_router_t *param)
{
	just_to_avoid_the_many_warnings_abe_router_t = *param;
	just_to_avoid_the_many_warnings_abe_router_id = id;
}

/*
 *  ABE_READ_DEBUG_TRACE
 *
 *  Parameter  :
 *      data destination pointer
 *      max number of data read
 *
 *  Operations :
 *      reads the AE circular data pointer holding pairs of debug data+timestamps, and store
 *      the pairs in linear addressing to the parameter pointer. Stops the copy when the max
 *	parameter is reached or when the FIFO is empty.
 *
 *  Return value :
 *      None.
 */
void abe_read_debug_trace(abe_uint32 *data, abe_uint32 *n)
{
	just_to_avoid_the_many_warnings = (*data);
	just_to_avoid_the_many_warnings = (*n);
}

/*
 *  ABE_SET_DEBUG_TRACE
 *
 *  Parameter  :
 *      debug ID from a list to be defined
 *
 *  Operations :
 *      load a mask which filters the debug trace to dedicated types of data
 *
 *  Return value :
 *      None.
 */
void abe_set_debug_trace(abe_dbg_t debug)
{
}

/*
 *  ABE_SET_DEBUG_PINS
 *
 *  Parameter  :
 *      debugger commands to the AESS register in charge of the debug pins
 *
 *  Operations :
 *      interpret the debugger commands: activate the AESS debug pins and
 *      allocate those pins to the AE outputs.
 *
 *  Return value :
 *      None.
 */
void abe_set_debug_pins(abe_uint32 debug_pins)
{
	just_to_avoid_the_many_warnings = debug_pins;
}

/*
 *  ABE_REMOTE_DEBUGGER_INTERFACE
 *
 *  Parameter  :
 *
 *  Operations :
 *      interpretation of the UART stream from the remote debugger commands.
 *      The commands consist in setting break points, loading parameter
 *
 *  Return value :
 *      None.
 */
void abe_remote_debugger_interface(abe_uint32 n, abe_uint8 *p)
{
	just_to_avoid_the_many_warnings = n;
	just_to_avoid_the_many_warnings = (abe_uint32) (*p);
}
