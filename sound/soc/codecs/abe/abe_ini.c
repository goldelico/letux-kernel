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
#include "abe_dat.h" /* data declaration */

/*
 *  initialize the default values for call-backs to subroutines
 *      - FIFO IRQ call-backs for sequenced tasks
 *      - FIFO IRQ call-backs for audio player/recorders (ping-pong protocols)
 *      - Remote debugger interface
 *      - Error monitoring
 *      - Activity Tracing
 */

/*
 *  ABE_HW_CONFIGURATION
 *
 *  Parameter  :
 *
 *  Operations :
 *
 *
 *  Return value :
 *
 */
void abe_hw_configuration()
{
	abe_uint32 atc_reg;
	abe_port_protocol_t *protocol;
	abe_data_format_t format;

	/* initializes the ABE ATC descriptors in DMEM - MCPDM_UL */
	protocol = &(abe_port[PDM_UL_PORT].protocol);
	format = abe_port[PDM_UL_PORT].format;
	abe_init_atc(PDM_UL_PORT);
	abe_init_io_tasks(PDM_UL_PORT, &format, protocol);

	/* initializes the ABE ATC descriptors in DMEM - MCPDM_DL */
	protocol = &(abe_port[PDM_DL_PORT].protocol);
	format = abe_port[PDM_DL_PORT].format;
	abe_init_atc(PDM_DL_PORT);
	abe_init_io_tasks(PDM_DL_PORT, &format, protocol);

	/* one DMIC port enabled = all DMICs enabled, since there is a single DMIC path for all DMICs */
	protocol = &(abe_port[DMIC_PORT].protocol);
	format = abe_port[DMIC_PORT].format;
	abe_init_atc(DMIC_PORT);
	abe_init_io_tasks(DMIC_PORT, &format, protocol);

	/* enables the DMAreq from AESS  AESS_DMAENABLE_SET = 255 */
	atc_reg = 0xFF;
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_ATC, 0x60, &atc_reg, 4);
}

/*
 *  ABE_BUILD_SCHEDULER_TABLE
 *
 *  Parameter  :
 *
 *  Operations :
 *
 *
 *  Return value :
 *
 */
void abe_build_scheduler_table()
{
	short VirtAudio_aMultiFrame[PROCESSING_SLOTS][TASKS_IN_SLOT];
	abe_uint16 i, n;
	abe_uint8 *ptr;
	char aUplinkMuxing[16];

	/* LOAD OF THE TASKS' MULTIFRAME */
	/* WARNING ON THE LOCATION OF IO_MM_DL WHICH IS PATCHED IN "abe_init_io_tasks" */

	for (ptr = (abe_uint8 *)&(VirtAudio_aMultiFrame[0][0]), i=0; i < sizeof (VirtAudio_aMultiFrame); i++)
		*ptr++ = 0;

	//VirtAudio_aMultiFrame[0][0] = 0;
	//VirtAudio_aMultiFrame[0][1] = 0;
	VirtAudio_aMultiFrame[0][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_VX_DL;
	//VirtAudio_aMultiFrame[0][3] = 0;
	//VirtAudio_aMultiFrame[0][4] = 0;
	//VirtAudio_aMultiFrame[0][5] = 0;
	//@@@ VirtAudio_aMultiFrame[0][6] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_ASRC_MM_DL; No ASRC
	//VirtAudio_aMultiFrame[0][7] = 0;
	//VirtAudio_aMultiFrame[1][0] = 0;
	//VirtAudio_aMultiFrame[1][1] = 0;
#define TASK_ASRC_VX_DL_SLT 1
#define TASK_ASRC_VX_DL_IDX 2
	//VirtAudio_aMultiFrame[1][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_ASRC_VX_DL_16;
#define TASK_VX_DL_SLT 1
#define TASK_VX_DL_IDX 3
	VirtAudio_aMultiFrame[1][3] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VX_DL_16_48;
	//VirtAudio_aMultiFrame[1][4] = 0;
	//VirtAudio_aMultiFrame[1][5] = 0;
	VirtAudio_aMultiFrame[1][6] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL2Mixer;
	VirtAudio_aMultiFrame[1][7] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_VIB_DL;
	VirtAudio_aMultiFrame[2][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL1Mixer;
	//VirtAudio_aMultiFrame[2][1] = 0;
	VirtAudio_aMultiFrame[2][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_SideTone;
	//VirtAudio_aMultiFrame[2][3] = 0;
	VirtAudio_aMultiFrame[2][4] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_SDTMixer;
	VirtAudio_aMultiFrame[2][5] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_DMIC;
	//VirtAudio_aMultiFrame[2][6] = 0;
	//VirtAudio_aMultiFrame[2][7] = 0;

	VirtAudio_aMultiFrame[3][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL1_EQ;
	//VirtAudio_aMultiFrame[3][1] = 0;
	VirtAudio_aMultiFrame[3][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_EchoMixer;
	//VirtAudio_aMultiFrame[3][3] = 0;
	//VirtAudio_aMultiFrame[3][4] = 0;
	//VirtAudio_aMultiFrame[3][5] = 0;
	VirtAudio_aMultiFrame[3][6] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL2_EQ;
	VirtAudio_aMultiFrame[3][7] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VIBRA_SPLIT;
	VirtAudio_aMultiFrame[4][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL1_APS_EQ;
	VirtAudio_aMultiFrame[4][1] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL1_GAIN;
	VirtAudio_aMultiFrame[4][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VXRECMixer;
	VirtAudio_aMultiFrame[4][3] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VXREC_SPLIT;
	//VirtAudio_aMultiFrame[4][4] = 0;
	//VirtAudio_aMultiFrame[4][5] = 0;
	VirtAudio_aMultiFrame[4][6] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VIBRA1;
	VirtAudio_aMultiFrame[4][7] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VIBRA2;

	VirtAudio_aMultiFrame[5][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_EARP_48_96_0SR;
	VirtAudio_aMultiFrame[5][1] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_EARP_48_96_LP;
	VirtAudio_aMultiFrame[5][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_PDM_UL;
	//VirtAudio_aMultiFrame[5][3] = 0;
	//VirtAudio_aMultiFrame[5][4] = 0;
	//VirtAudio_aMultiFrame[5][5] = 0;
	VirtAudio_aMultiFrame[5][6] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL2_APS_EQ;
	VirtAudio_aMultiFrame[5][7] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL2_GAIN;

	VirtAudio_aMultiFrame[6][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_EARP_48_96_LP;
	VirtAudio_aMultiFrame[6][1] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_PDM_DL;
	//VirtAudio_aMultiFrame[6][2] = 0;
	//VirtAudio_aMultiFrame[6][3] = 0;
	//VirtAudio_aMultiFrame[6][4] = 0;
	//VirtAudio_aMultiFrame[6][5] = 0;
	VirtAudio_aMultiFrame[6][6] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL2_APS_SPLIT;
	//VirtAudio_aMultiFrame[6][7] = 0;

	//VirtAudio_aMultiFrame[7][0] = 0;
	//VirtAudio_aMultiFrame[7][1] = 0;
	VirtAudio_aMultiFrame[7][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_BT_UL_SPLIT;
	VirtAudio_aMultiFrame[7][3] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DBG_SYNC;
	//VirtAudio_aMultiFrame[7][4] = 0;
	//VirtAudio_aMultiFrame[7][5] = 0;
	VirtAudio_aMultiFrame[7][6] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL2_R_APS_CORE;
	VirtAudio_aMultiFrame[7][7] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL2_L_APS_CORE;

	//VirtAudio_aMultiFrame[8][0] = 0;
	//VirtAudio_aMultiFrame[8][1] = 0;
	VirtAudio_aMultiFrame[8][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DMIC1_96_48_LP;
	//VirtAudio_aMultiFrame[8][3] = 0;
	VirtAudio_aMultiFrame[8][4] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DMIC1_SPLIT;
	//VirtAudio_aMultiFrame[8][5] = 0;
	//VirtAudio_aMultiFrame[8][6] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_EANC_FBK_96_48;
	//VirtAudio_aMultiFrame[8][7] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_EANC_FBK_SPLIT;

	//VirtAudio_aMultiFrame[9][0] = 0;
	//VirtAudio_aMultiFrame[9][1] = 0;
	VirtAudio_aMultiFrame[9][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DMIC2_96_48_LP;
	//VirtAudio_aMultiFrame[9][3] = 0;
	VirtAudio_aMultiFrame[9][4] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DMIC2_SPLIT;
	//VirtAudio_aMultiFrame[9][5] = 0;
	VirtAudio_aMultiFrame[9][6] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IHF_48_96_0SR;
	VirtAudio_aMultiFrame[9][7] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IHF_48_96_LP;

	//VirtAudio_aMultiFrame[10][0] = 0;
	//VirtAudio_aMultiFrame[10][1] = 0;
	VirtAudio_aMultiFrame[10][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DMIC3_96_48_LP;
	//VirtAudio_aMultiFrame[10][3] = 0;
	VirtAudio_aMultiFrame[10][4] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DMIC3_SPLIT;
	//VirtAudio_aMultiFrame[10][5] = 0;
	//VirtAudio_aMultiFrame[10][6] = 0; // D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_EANC_COPY; // NEW: copy EANC coefs to working CMEM areas
	VirtAudio_aMultiFrame[10][7] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IHF_48_96_LP;

	//VirtAudio_aMultiFrame[11][0] = 0;
	//VirtAudio_aMultiFrame[11][1] = 0;
	VirtAudio_aMultiFrame[11][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_AMIC_96_48_LP;
	//VirtAudio_aMultiFrame[11][3] = 0;
	VirtAudio_aMultiFrame[11][4] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_AMIC_SPLIT;
	//VirtAudio_aMultiFrame[11][5] = 0;
	//VirtAudio_aMultiFrame[11][6] = 0;
	VirtAudio_aMultiFrame[11][7] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VIBRA_PACK;

	//VirtAudio_aMultiFrame[12][0] = 0;
	//VirtAudio_aMultiFrame[12][1] = 0;
	VirtAudio_aMultiFrame[12][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_BT_VX_DL;
	VirtAudio_aMultiFrame[12][3] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VX_UL_ROUTING;
	VirtAudio_aMultiFrame[12][4] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_ULMixer;
#define TASK_VX_UL_SLT 12
#define TASK_VX_UL_IDX 5
	VirtAudio_aMultiFrame[12][5] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VX_UL_48_16;
	//VirtAudio_aMultiFrame[12][6] = 0;
	//VirtAudio_aMultiFrame[12][7] = 0;
	//VirtAudio_aMultiFrame[13][0] = 0;
	//VirtAudio_aMultiFrame[13][1] = 0;
	VirtAudio_aMultiFrame[13][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_MM_UL2_ROUTING;
	//VirtAudio_aMultiFrame[13][3] = 0;
	VirtAudio_aMultiFrame[13][4] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_MM_UL;
	//VirtAudio_aMultiFrame[13][5] = 0;
	//VirtAudio_aMultiFrame[13][6] = 0;
	//VirtAudio_aMultiFrame[13][7] = 0;

	//VirtAudio_aMultiFrame[14][0] = 0;
	//VirtAudio_aMultiFrame[14][1] = 0;
	//VirtAudio_aMultiFrame[14][2] = 0;
	VirtAudio_aMultiFrame[14][3] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_DMIC;
#define TASK_BT_DL_48_8_SLT	14
#define	TASK_BT_DL_48_8_IDX	4
	VirtAudio_aMultiFrame[14][4] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_BT_DL_48_8;
	//VirtAudio_aMultiFrame[14][5] = 0;
#define TASK_ECHO_SLT 14
#define TASK_ECHO_IDX 6
	VirtAudio_aMultiFrame[14][6] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_ECHO_REF_48_16;
	//VirtAudio_aMultiFrame[14][7] = 0;

	VirtAudio_aMultiFrame[15][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL1_APS_IIR;
	VirtAudio_aMultiFrame[15][1] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DL1_APS_CORE;
	//VirtAudio_aMultiFrame[15][2] = 0;
	VirtAudio_aMultiFrame[15][3] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_BT_VX_UL;
	//VirtAudio_aMultiFrame[15][4] = 0;
	//VirtAudio_aMultiFrame[15][5] = 0;
	//VirtAudio_aMultiFrame[15][6] = 0;
#define TASK_ASRC_ECHO_SLT 15
#define TASK_ASRC_ECHO_IDX 7
	//@@@	VirtAudio_aMultiFrame[15][7] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_ASRC_ECHO_REF_16;

	//VirtAudio_aMultiFrame[16][0] = 0;
	//VirtAudio_aMultiFrame[16][1] = 0;
#define TASK_ASRC_VX_UL_SLT 16
#define TASK_ASRC_VX_UL_IDX 2
	//@@@  VirtAudio_aMultiFrame[16][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_ASRC_VX_UL_16;
	VirtAudio_aMultiFrame[16][3] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_VX_UL; // USING ECHO REF MERGED
	//VirtAudio_aMultiFrame[16][4] = 0;
	//VirtAudio_aMultiFrame[16][5] = 0;
	//VirtAudio_aMultiFrame[16][6] = 0;
	//VirtAudio_aMultiFrame[16][7] = 0;

	//VirtAudio_aMultiFrame[17][0] = 0;
	//VirtAudio_aMultiFrame[17][1] = 0;
#define TASK_BT_UL_8_48_SLT	17
#define TASK_BT_UL_8_48_IDX	2
	VirtAudio_aMultiFrame[17][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_BT_UL_8_48;
	VirtAudio_aMultiFrame[17][3] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_MM_UL2;
	//VirtAudio_aMultiFrame[17][4] = 0;
	//VirtAudio_aMultiFrame[17][5] = 0;
	//VirtAudio_aMultiFrame[17][6] = 0;
	//VirtAudio_aMultiFrame[17][7] = 0;

	VirtAudio_aMultiFrame[18][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_PDM_DL;
	//VirtAudio_aMultiFrame[18][1] = 0;
	//VirtAudio_aMultiFrame[18][2] = 0;
	//VirtAudio_aMultiFrame[18][3] = 0;
	//VirtAudio_aMultiFrame[18][4] = 0;
	//VirtAudio_aMultiFrame[18][5] = 0;
	//VirtAudio_aMultiFrame[18][6] = 0;
	//VirtAudio_aMultiFrame[18][7] = 0;

#define TASK_IO_MM_DL_SLT 19
#define TASK_IO_MM_DL_IDX 0
	VirtAudio_aMultiFrame[19][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_MM_DL;
	//VirtAudio_aMultiFrame[19][1] = 0
	//VirtAudio_aMultiFrame[19][2] = 0;
	//VirtAudio_aMultiFrame[19][3] = 0;
	//VirtAudio_aMultiFrame[19][4] = 0;
	//VirtAudio_aMultiFrame[19][5] = 0;
	//VirtAudio_aMultiFrame[19][6] = 0;
	//VirtAudio_aMultiFrame[19][7] = 0;

	VirtAudio_aMultiFrame[20][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_TONES_DL;
	VirtAudio_aMultiFrame[20][1] = 0;
	VirtAudio_aMultiFrame[20][2] = 0;
	//VirtAudio_aMultiFrame[20][3] = 0;
	//VirtAudio_aMultiFrame[20][4] = 0;
	//VirtAudio_aMultiFrame[20][5] = 0;
	//VirtAudio_aMultiFrame[20][6] = 0;
	//VirtAudio_aMultiFrame[20][7] = 0;

	//VirtAudio_aMultiFrame[21][0] = 0;
	//VirtAudio_aMultiFrame[21][1] = 0;
	//VirtAudio_aMultiFrame[21][2] = 0;
	//VirtAudio_aMultiFrame[21][3] = 0;
	//VirtAudio_aMultiFrame[21][4] = 0;
	//VirtAudio_aMultiFrame[21][5] = 0;
	//VirtAudio_aMultiFrame[21][6] = 0;
	//VirtAudio_aMultiFrame[21][7] = 0;

	VirtAudio_aMultiFrame[22][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_DEBUG_IRQFIFO;	// MUST STAY ON SLOT 22
	//VirtAudio_aMultiFrame[22][1] = 0;
	VirtAudio_aMultiFrame[22][2] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_MM_EXT_OUT;
	VirtAudio_aMultiFrame[22][3] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_MM_EXT_IN;
	//VirtAudio_aMultiFrame[22][4] = 0;
	//VirtAudio_aMultiFrame[22][5] = 0;
	//VirtAudio_aMultiFrame[22][6] = 0;
	//VirtAudio_aMultiFrame[22][7] = 0;

	VirtAudio_aMultiFrame[23][0] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_GAIN_UPDATE;
	//VirtAudio_aMultiFrame[23][1] = 0;
	//VirtAudio_aMultiFrame[23][2] = 0;
	//VirtAudio_aMultiFrame[23][3] = 0;
	//VirtAudio_aMultiFrame[23][4] = 0;
	//VirtAudio_aMultiFrame[23][5] = 0;
	//VirtAudio_aMultiFrame[23][6] = 0;
	//VirtAudio_aMultiFrame[23][7] = 0;

	//VirtAudio_aMultiFrame[24][0] = 0;
	//VirtAudio_aMultiFrame[24][1] = 0;
	//VirtAudio_aMultiFrame[24][2] = 0;
	//VirtAudio_aMultiFrame[24][3] = 0;
	//VirtAudio_aMultiFrame[24][4] = 0;
	//VirtAudio_aMultiFrame[24][5] = 0;
	//VirtAudio_aMultiFrame[24][6] = 0;
	//VirtAudio_aMultiFrame[24][7] = 0;

	abe_block_copy (COPY_FROM_HOST_TO_ABE, ABE_DMEM, D_multiFrame_ADDR, (abe_uint32*)VirtAudio_aMultiFrame, sizeof (VirtAudio_aMultiFrame));

	// EANC Fast Loopback
	// dFastLoopback = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_EANC_WRAP2;
	// abe_block_copy (COPY_FROM_HOST_TO_ABE, ABE_DMEM, D_pFastLoopBack_ADDR, (abe_uint32*)&dFastLoopback, sizeof (dFastLoopback));

	/* reset the uplink router */
	n = D_aUplinkRouting_ADDR_END - D_aUplinkRouting_ADDR + 1;
	for(i = 0; i < n; i++)
		aUplinkMuxing[i] = ZERO_labelID;

	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, D_aUplinkRouting_ADDR, (abe_uint32 *)aUplinkMuxing, sizeof(aUplinkMuxing));
}

/*
 *  ABE_INIT_ATC
 *
 *  Parameter  :
 *      prot : protocol being used
 *
 *  Operations :
 *      load the DMEM ATC/AESS descriptors
 *
 *  Return value :
 *
 */
void abe_init_atc(abe_port_id id)
{
	abe_satcdescriptor_aess desc;
	abe_uint8 iter;
	abe_int32 datasize;

	// load default values of the descriptor
	desc.rdpt = desc.wrpt = desc.irqdest = desc.cberr = desc.desen = desc.nw =0;
	desc.reserved0 = desc.reserved1 = desc.reserved2 = 0;
	desc.srcid = desc.destid = desc.badd = desc.iter = desc.cbsize = 0;

	datasize = abe_dma_port_iter_factor (&((abe_port[id]).format));
	iter = (abe_uint8) abe_dma_port_iteration (&((abe_port[id]).format));

	if (abe_port[id].protocol.direction == ABE_ATC_DIRECTION_IN) // IN from AESS point of view
		if (iter + 2*datasize > 126)
			desc.wrpt = (iter >>1) + (2*datasize);
		else
			desc.wrpt = iter + 2*datasize;
	else
		desc.wrpt = 0 + 2*datasize;

	switch ((abe_port[id]).protocol.protocol_switch) {
	case SLIMBUS_PORT_PROT:
		desc.cbdir = (abe_port[id]).protocol.direction;
		desc.cbsize = (abe_port[id]).protocol.p.prot_slimbus.buf_size;
		desc.badd = ((abe_port[id]).protocol.p.prot_slimbus.buf_addr1) >> 4;
		desc.iter = (abe_port[id]).protocol.p.prot_slimbus.iter;
		desc.srcid = abe_atc_srcid [(abe_port[id]).protocol.p.prot_slimbus.desc_addr1 >> 3];

		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
				(abe_port[id]).protocol.p.prot_slimbus.desc_addr1,
							(abe_uint32*)&desc, sizeof(desc));

		desc.badd = (abe_port[id]).protocol.p.prot_slimbus.buf_addr2;
		desc.srcid = abe_atc_srcid [(abe_port[id]).protocol.p.prot_slimbus.desc_addr2 >> 3];

		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
				(abe_port[id]).protocol.p.prot_slimbus.desc_addr2,
							(abe_uint32*)&desc, sizeof(desc));
		break;
	case SERIAL_PORT_PROT:
		desc.cbdir = (abe_port[id]).protocol.direction;
		desc.cbsize = (abe_port[id]).protocol.p.prot_serial.buf_size;
		desc.badd = ((abe_port[id]).protocol.p.prot_serial.buf_addr) >> 4;
		desc.iter = (abe_port[id]).protocol.p.prot_serial.iter;
		desc.srcid = abe_atc_srcid[(abe_port[id]).protocol.p.prot_serial.desc_addr >> 3];
		desc.destid = abe_atc_dstid[(abe_port[id]).protocol.p.prot_serial.desc_addr >> 3];

		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
				(abe_port[id]).protocol.p.prot_serial.desc_addr,
							(abe_uint32*)&desc, sizeof(desc));
		break;
	case DMIC_PORT_PROT:
		desc.cbdir = ABE_ATC_DIRECTION_IN;
		desc.cbsize = (abe_port[id]).protocol.p.prot_dmic.buf_size;
		desc.badd = ((abe_port[id]).protocol.p.prot_dmic.buf_addr) >> 4;
		desc.iter = DMIC_ITER;
		desc.srcid = abe_atc_srcid[ABE_ATC_DMIC_DMA_REQ];

		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
				ABE_ATC_DMIC_DMA_REQ * ATC_SIZE, (abe_uint32*)&desc, sizeof(desc));
		break;
	case MCPDMDL_PORT_PROT:
		abe_global_mcpdm_control = abe_port[id].protocol.p.prot_mcpdmdl.control;	/* Control allowed on McPDM DL */
		desc.cbdir = ABE_ATC_DIRECTION_OUT;
		desc.cbsize = (abe_port[id]).protocol.p.prot_mcpdmdl.buf_size;
		desc.badd = ((abe_port[id]).protocol.p.prot_mcpdmdl.buf_addr) >> 4;
		desc.iter = MCPDM_DL_ITER;
		desc.destid = abe_atc_dstid[ABE_ATC_MCPDMDL_DMA_REQ];

		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
				ABE_ATC_MCPDMDL_DMA_REQ * ATC_SIZE, (abe_uint32*)&desc, sizeof(desc));
		break;
	case MCPDMUL_PORT_PROT:
		desc.cbdir = ABE_ATC_DIRECTION_IN;
		desc.cbsize = (abe_port[id]).protocol.p.prot_mcpdmul.buf_size;
		desc.badd = ((abe_port[id]).protocol.p.prot_mcpdmul.buf_addr) >> 4;
		desc.iter = MCPDM_UL_ITER;
		desc.srcid = abe_atc_srcid[ABE_ATC_MCPDMUL_DMA_REQ];

		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
				ABE_ATC_MCPDMUL_DMA_REQ * ATC_SIZE, (abe_uint32*)&desc, sizeof(desc));
		break;
	case PINGPONG_PORT_PROT:
		/* software protocol, nothing to do on ATC */
		break;
	case DMAREQ_PORT_PROT:
		desc.cbdir = (abe_port[id]).protocol.direction;
		desc.cbsize = (abe_port[id]).protocol.p.prot_dmareq.buf_size;
		desc.badd = ((abe_port[id]).protocol.p.prot_dmareq.buf_addr) >> 4;
		desc.iter = 1; /* CBPr needs ITER=1. this is the eDMA job to do the iterations */
		/* input from ABE point of view */
		if (abe_port[id].protocol.direction == ABE_ATC_DIRECTION_IN) {
			desc.rdpt = 127;
			desc.wrpt = 0;
			desc.srcid = abe_atc_srcid[(abe_port[id]).protocol.p.prot_dmareq.desc_addr >> 3];
		} else {
			desc.rdpt = 0;
			desc.wrpt = 127;
			desc.destid = abe_atc_dstid[(abe_port[id]).protocol.p.prot_dmareq.desc_addr >> 3];
		}

		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
			(abe_port[id]).protocol.p.prot_dmareq.desc_addr, (abe_uint32*)&desc, sizeof (desc));
		break;
	default:
		break;
	}
}

/*
 *
 * ABE_INIT_DMA_T
 *  Parameter  :
 *      prot : protocol being used
 *
 *  Operations :
 *      load the dma_t with physical information from AE memory mapping
 *
 *  Return value :
 *
 */
void abe_init_dma_t(abe_port_id id, abe_port_protocol_t *prot)
{
	abe_dma_t_offset dma;
	abe_uint32 idx;

	dma.data = 0;	/* default dma_t points to address 0000... */
	dma.iter = 0;

	switch (prot->protocol_switch) {
	case PINGPONG_PORT_PROT:
		for (idx = 0; idx < 32; idx++) {
			if (((prot->p).prot_pingpong.irq_data) == (abe_uint32)(1 << idx))
				break;
		}
		(prot->p).prot_dmareq.desc_addr = (CBPr_DMA_RTX0+idx)*ATC_SIZE;
		dma.data = (prot->p).prot_pingpong.buf_addr;
		dma.iter = (prot->p).prot_pingpong.buf_size;
		break;
	case DMAREQ_PORT_PROT:
		for (idx = 0; idx < 32; idx++) {
			if (((prot->p).prot_dmareq.dma_data) == (abe_uint32)(1 << idx))
				break;
		}
		dma.data = (CIRCULAR_BUFFER_PERIPHERAL_R__0 + idx*4);
		dma.iter = (prot->p).prot_dmareq.iter;
		(prot->p).prot_dmareq.desc_addr = (CBPr_DMA_RTX0+idx)*ATC_SIZE;
		break;
	case SLIMBUS_PORT_PROT:
	case SERIAL_PORT_PROT:
	case DMIC_PORT_PROT:
	case MCPDMDL_PORT_PROT:
	case MCPDMUL_PORT_PROT:
	default:
		break;
	}

	/* upload the dma type */
	abe_port [id].dma = dma;
}

/*
 * ABE_DISENABLE_DMA_REQUEST
 * Parameter:
 * Operations:
 * Return value:
 */
void abe_disable_enable_dma_request(abe_port_id id, abe_uint32 on_off)
{
	ABE_SIODescriptor desc;
	abe_uint8 desc_second_word[4], irq_dmareq_field;
	abe_uint32 sio_desc_address;

	if (abe_port[id].protocol.protocol_switch == DMAREQ_PORT_PROT) {
		sio_desc_address = dmem_port_descriptors + (id * sizeof(ABE_SIODescriptor));
		abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM, sio_desc_address, (abe_uint32*)&desc, sizeof (desc));
		if (on_off) {
			desc.atc_irq_data = (abe_uint8) abe_port[id].protocol.p.prot_dmareq.dma_data;
				desc.on_off = 0x80;
		} else {
			desc.atc_irq_data = 0;
			desc.on_off = 0;
		}
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, sio_desc_address, (abe_uint32*)&desc, sizeof (desc));
	}

	if (abe_port[id].protocol.protocol_switch == PINGPONG_PORT_PROT) {
		irq_dmareq_field = (abe_uint8)(on_off * abe_port[id].protocol.p.prot_pingpong.irq_data);
		sio_desc_address = D_PingPongDesc_ADDR;

		abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM, sio_desc_address + 4, (abe_uint32 *)desc_second_word, 4);
		desc_second_word[2] = irq_dmareq_field;
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, sio_desc_address + 4, (abe_uint32 *)desc_second_word, 4);
	}
}

void abe_enable_dma_request(abe_port_id id)
{
	abe_disable_enable_dma_request(id, 1);
}

/*
 * ABE_DISABLE_DMA_REQUEST
 *
 * Parameter:
 * Operations:
 * Return value:
 *
 */
void abe_disable_dma_request(abe_port_id id)
{
	abe_disable_enable_dma_request(id, 0);
}


/*
 * ABE_ENABLE_ATC
 * Parameter:
 * Operations:
 * Return value:
 */
void abe_enable_atc(abe_port_id id)
{
	just_to_avoid_the_many_warnings = (abe_port_id)id;
#if 0
	abe_satcdescriptor_aess desc;
	abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM,
		(abe_port[id]).protocol.p.prot_dmareq.desc_addr,
				(abe_uint32*)&desc, sizeof (desc));
	desc.desen = 1;
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
		(abe_port[id]).protocol.p.prot_dmareq.desc_addr,
				(abe_uint32*)&desc, sizeof (desc));
#endif
}


/*
 * ABE_DISABLE_ATC
 * Parameter:
 * Operations:
 * Return value:
 */
void abe_disable_atc(abe_port_id id)
{
	abe_satcdescriptor_aess desc;

	abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM,
		(abe_port[id]).protocol.p.prot_dmareq.desc_addr,
				(abe_uint32*)&desc, sizeof (desc));
	desc.desen = 0;
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
		(abe_port[id]).protocol.p.prot_dmareq.desc_addr,
				(abe_uint32*)&desc, sizeof (desc));
}

/*
 *   ABE_INIT_IO_TASKS
 *
 *  Parameter  :
 *  prot : protocol being used
 *
 *  Operations :
 *      load the micro-task parameters doing to DMEM <==> SMEM data moves
 *
 *       I/O descriptors input parameters :
 *	 For Read from DMEM usually THR1/THR2 = X+1/X-1
 *	 For Write to DMEM usually  THR1/THR2 = 2/0
 *	 UP_1/2 =X+1/X-1
 *
 *  Return value :
 *
 */
void abe_init_io_tasks(abe_port_id id, abe_data_format_t *format, abe_port_protocol_t *prot)
{
	ABE_SIODescriptor desc;
	ABE_SPingPongDescriptor desc_pp;
	abe_uint32 x_io, direction, iter_samples, smem1, smem2, smem3, io_sub_id;
	abe_uint32 copy_func_index, before_func_index, after_func_index;
	abe_uint32 dmareq_addr, dmareq_field;
	abe_uint32 sio_desc_address, datasize, iter, nsamp, datasize2, dOppMode32;
	abe_uint32 atc_ptr_saved, atc_ptr_saved2, copy_func_index1;
	abe_uint32 copy_func_index2, atc_desc_address1, atc_desc_address2;
	short VirtAudio_aMultiFrame[PROCESSING_SLOTS][TASKS_IN_SLOT];

	if (prot->protocol_switch == PINGPONG_PORT_PROT) {
		if (MM_DL_PORT == id) {
			// @@@@  reset local memory
			abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM,
				D_multiFrame_ADDR,
					(abe_uint32*)VirtAudio_aMultiFrame,
						sizeof (VirtAudio_aMultiFrame));
			VirtAudio_aMultiFrame[TASK_IO_MM_DL_SLT][TASK_IO_MM_DL_IDX] =
				D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO_PING_PONG;
			abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
				D_multiFrame_ADDR, (abe_uint32*)VirtAudio_aMultiFrame,
						sizeof (VirtAudio_aMultiFrame));
		} else {
			abe_dbg_param |= ERR_API;
			abe_dbg_error_log (ABE_PARAMETER_ERROR);
		}

		smem1 = (abe_uint8) abe_port[id].smem_buffer1;
		copy_func_index = (abe_uint8) abe_dma_port_copy_subroutine_id (id);
		dmareq_addr  = abe_port[id].protocol.p.prot_pingpong.irq_addr;
		dmareq_field = abe_port[id].protocol.p.prot_pingpong.irq_data;
		datasize = abe_dma_port_iter_factor (format);
		iter = abe_dma_port_iteration (format);
		iter_samples = (iter / datasize);	  /* number of "samples" either mono or stereo */

		/* load the IO descriptor */
		desc_pp.drift_ASRC = 0; /* no drift  */
		desc_pp.drift_io = 0; /* no drift  */
		desc_pp.hw_ctrl_addr = (abe_uint16) dmareq_addr;
		desc_pp.copy_func_index = (abe_uint8) copy_func_index;
		desc_pp.smem_addr = (abe_uint8) smem1;
		desc_pp.atc_irq_data = (abe_uint8) dmareq_field; /* DMA req 0 is used for CBPr0 */
		desc_pp.x_io = (abe_uint8) iter_samples; /* size of block transfer */
		desc_pp.data_size = (abe_uint8) datasize;
		desc_pp.workbuff_BaseAddr = (abe_uint16) (abe_base_address_pingpong [0]);			 /* address comunicated in Bytes */
		desc_pp.workbuff_Samples = (abe_uint16) ((abe_size_pingpong/datasize) >> 2);	 /* size comunicated in XIO sample */
		desc_pp.nextbuff0_BaseAddr = (abe_uint16) (abe_base_address_pingpong [0]);
		desc_pp.nextbuff0_Samples = (abe_uint16) ((abe_size_pingpong/datasize) >> 2);
		desc_pp.nextbuff1_BaseAddr = (abe_uint16) (abe_base_address_pingpong [1]);
		desc_pp.nextbuff1_Samples = (abe_uint16) ((abe_size_pingpong/datasize) >> 2);
		desc_pp.counter = 1;

		/* send a DMA req to fill B0 with N samples
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_ATC, ABE_DMASTATUS_RAW, &(abe_port[id].protocol.p.prot_pingpong.irq_data), 4); */

		sio_desc_address = D_PingPongDesc_ADDR;
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, sio_desc_address,
				(abe_uint32*)&desc_pp, sizeof (desc_pp));
	} else {
		io_sub_id = dmareq_addr = ABE_DMASTATUS_RAW;
		dmareq_field = 0;
		atc_desc_address1 = atc_desc_address2 = 0;

		datasize2=datasize = abe_dma_port_iter_factor(format);
		x_io = (abe_uint8) abe_dma_port_iteration(format);
		nsamp = (x_io / datasize);

		atc_ptr_saved2=atc_ptr_saved = DMIC_ATC_PTR_labelID + id;

		smem1 = abe_port[id].smem_buffer1;
		smem3 = smem2 = abe_port[id].smem_buffer2;

		copy_func_index1 = (abe_uint8) abe_dma_port_copy_subroutine_id(id);
		before_func_index = after_func_index = copy_func_index2 = NULL_COPY_CFPID;

		switch (prot->protocol_switch) {
		case DMIC_PORT_PROT:
			/* DMIC port is read in two steps */
			x_io = x_io >> 1;
			nsamp = nsamp >> 1;
			atc_desc_address1 = ABE_ATC_DMIC_DMA_REQ*ATC_SIZE;
			io_sub_id = IO_IP_CFPID;
			break;
		case MCPDMDL_PORT_PROT:
			/* PDMDL port is written to  in two steps */
			x_io = x_io >> 1;
			atc_desc_address1 = ABE_ATC_MCPDMDL_DMA_REQ*ATC_SIZE;
			io_sub_id = IO_IP_CFPID;
			break;
		case MCPDMUL_PORT_PROT:
			atc_desc_address1 = ABE_ATC_MCPDMUL_DMA_REQ*ATC_SIZE;
			io_sub_id = IO_IP_CFPID;
			break;
		case SLIMBUS_PORT_PROT:
			atc_desc_address1 = abe_port[id].protocol.p.prot_slimbus.desc_addr1;
			atc_desc_address2 = abe_port[id].protocol.p.prot_slimbus.desc_addr2;
			copy_func_index2 = NULL_COPY_CFPID;
/* @@@@@@
#define SPLIT_SMEM_CFPID	9
#define MERGE_SMEM_CFPID	10
#define SPLIT_TDM_12_CFPID	11
#define MERGE_TDM_12_CFPID	12
*/
				io_sub_id = IO_IP_CFPID;
		case SERIAL_PORT_PROT: /* McBSP/McASP */
			atc_desc_address1  = (abe_int16) abe_port[id].protocol.p.prot_serial.desc_addr;
				io_sub_id = IO_IP_CFPID;
			break;
		case DMAREQ_PORT_PROT: /* DMA w/wo CBPr */
			dmareq_addr = abe_port[id].protocol.p.prot_dmareq.dma_addr;
			dmareq_field = 0;
			atc_desc_address1 = abe_port[id].protocol.p.prot_dmareq.desc_addr;
			io_sub_id = IO_DMAREQ_CFPID;
			break;
		default:
			break;
		}

		/* special situation of the PING_PONG protocol which has its own SIO descriptor format */
		/* Sequence of operations on ping-pong buffers B0/B1
		 * ---------- time --------------------------------------------->>>>
		 * Host Application  is ready to send data from DDR to B0
		 * SDMA is initialized from "abe_connect_irq_ping_pong_port" to B0
		 * ABE HAL init FW to B0
		 * send DMAreq to fill B0
		 * FIRMWARE starts sending B1 data, sends DMAreq v
		 * continue with B0, sends DMAreq v   continue with B1
		 * DMAreq v (direct access from HAL to AESS regs)
		 * v (from ABE_FW) v (from ABE_FW)
		 * SDMA | fills B0 | fills B1...| fills B0...
		 */

		if (MM_UL_PORT == id) {
			copy_func_index1 = sub_copy_mm_ul;
			before_func_index = ROUTE_MM_UL_CFPID;
		}

		/* check for 8kHz/16kHz */
		if (VX_DL_PORT == id) {
			abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM, D_multiFrame_ADDR,
				(abe_uint32*)VirtAudio_aMultiFrame, sizeof (VirtAudio_aMultiFrame));
			if (abe_port[id].format.f == 8000) {
				//@@@ VirtAudio_aMultiFrame[TASK_ASRC_VX_DL_SLT][TASK_ASRC_VX_DL_IDX]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_ASRC_VX_DL_8;
				VirtAudio_aMultiFrame[TASK_VX_DL_SLT][TASK_VX_DL_IDX] =
						D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VX_DL_8_48;
				smem1 = Voice_8k_DL_labelID; //@@@ IO_VX_DL_ASRC_labelID
			} else {
				//@@@ VirtAudio_aMultiFrame[TASK_ASRC_VX_DL_SLT][TASK_ASRC_VX_DL_IDX]  = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_ASRC_VX_DL_16;
				VirtAudio_aMultiFrame[TASK_VX_DL_SLT][TASK_VX_DL_IDX] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VX_DL_16_48;
				smem1 = Voice_16k_DL_labelID; //@@@ IO_VX_DL_ASRC_labelID
			}
			abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, D_multiFrame_ADDR,
				(abe_uint32*)VirtAudio_aMultiFrame, sizeof (VirtAudio_aMultiFrame));
		}
		/* check for 8kHz/16kHz */
		if (VX_UL_PORT == id) {
			abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM, D_multiFrame_ADDR, (abe_uint32*)VirtAudio_aMultiFrame, sizeof(VirtAudio_aMultiFrame));
			if (abe_port[id].format.f == 8000) {
				//@@@ VirtAudio_aMultiFrame[TASK_ASRC_VX_UL_SLT][TASK_ASRC_VX_UL_IDX] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_ASRC_VX_UL_8;
				VirtAudio_aMultiFrame[TASK_VX_UL_SLT][TASK_VX_UL_IDX] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VX_UL_48_8;
				//@@@ VirtAudio_aMultiFrame[TASK_ECHO_SLT][TASK_ECHO_IDX] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_ECHO_REF_48_8;
				//@@@ VirtAudio_aMultiFrame[TASK_ASRC_ECHO_SLT][TASK_ASRC_ECHO_IDX] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_ASRC_ECHO_REF_8;
				smem1 = Voice_8k_UL_labelID; //@@@ XinASRC_UL_VX_labelID
			} else {
				//@@@ VirtAudio_aMultiFrame[TASK_ASRC_VX_UL_SLT][TASK_ASRC_VX_UL_IDX] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_ASRC_VX_UL_16;
				VirtAudio_aMultiFrame[TASK_VX_UL_SLT][TASK_VX_UL_IDX] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_VX_UL_48_16;
				//@@@ VirtAudio_aMultiFrame[TASK_ECHO_SLT][TASK_ECHO_IDX] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_ECHO_REF_48_16;
				//@@@ VirtAudio_aMultiFrame[TASK_ASRC_ECHO_SLT][TASK_ASRC_ECHO_IDX] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_ASRC_ECHO_REF_16;
				smem1 = Voice_16k_UL_labelID; //@@@ XinASRC_UL_VX_labelID
			}
			abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, D_multiFrame_ADDR, (abe_uint32*)VirtAudio_aMultiFrame, sizeof(VirtAudio_aMultiFrame));
		}

		/* check for 8kHz/16kHz */
		if (BT_VX_DL_PORT == id) {
			abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM,
					D_multiFrame_ADDR,
						(abe_uint32*)VirtAudio_aMultiFrame,
							sizeof (VirtAudio_aMultiFrame));
			if (abe_port[id].format.f == 8000) {
				VirtAudio_aMultiFrame[TASK_BT_DL_48_8_SLT][TASK_BT_DL_48_8_IDX] =
					D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_BT_DL_48_8;
				smem1 = BT_DL_8k_labelID;
			} else {
				VirtAudio_aMultiFrame[TASK_BT_DL_48_8_SLT][TASK_BT_DL_48_8_IDX] =
					D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_BT_DL_48_16;
				smem1 = BT_DL_16k_labelID;
			}
			abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
				D_multiFrame_ADDR, (abe_uint32*)VirtAudio_aMultiFrame,
							sizeof (VirtAudio_aMultiFrame));
		}

		/* check for 8kHz/16kHz */
		if (BT_VX_UL_PORT == id) {
			abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM, D_multiFrame_ADDR,
				(abe_uint32*)VirtAudio_aMultiFrame, sizeof (VirtAudio_aMultiFrame));
			if (abe_port[id].format.f == 8000) {
				VirtAudio_aMultiFrame[TASK_BT_UL_8_48_SLT][TASK_BT_UL_8_48_IDX] =
					D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_BT_UL_8_48;
				smem1 = BT_UL_8k_labelID;
			} else {
				VirtAudio_aMultiFrame[TASK_BT_UL_8_48_SLT][TASK_BT_UL_8_48_IDX] =
					D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_BT_UL_16_48;
				smem1 = BT_UL_16k_labelID;
			}
			abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
				D_multiFrame_ADDR, (abe_uint32*)VirtAudio_aMultiFrame,
							sizeof (VirtAudio_aMultiFrame));
		}

		if (MM_DL_PORT == id) {
			//@@@ abe_block_copy (COPY_FROM_ABE_TO_HOST, ABE_DMEM, D_multiFrame_ADDR, (abe_uint32*)VirtAudio_aMultiFrame, sizeof (VirtAudio_aMultiFrame));
			//@@@ VirtAudio_aMultiFrame[TASK_IO_MM_DL_SLT][TASK_IO_MM_DL_IDX] = D_tasksList_ADDR + sizeof(ABE_STask)*C_ABE_FW_TASK_IO2_MM_DL;
			//@@@ abe_block_copy (COPY_FROM_HOST_TO_ABE, ABE_DMEM, D_multiFrame_ADDR, (abe_uint32*)VirtAudio_aMultiFrame, sizeof (VirtAudio_aMultiFrame));
			/* set the SMEM buffer @@@@@ programming sequence : OPP must be set before channel is defined */
			abe_block_copy (COPY_FROM_ABE_TO_HOST, ABE_DMEM, D_maxTaskBytesInSlot_ADDR, &dOppMode32, sizeof(abe_uint32));
			if (dOppMode32 == DOPPMODE32_OPP100)
				smem1 = smem_mm_dl_opp100; /* ASRC input buffer, size 40 */
			else
				smem1 = smem_mm_dl_opp25; /* at OPP 25/50 or without ASRC  */
		}
		/* MM_EXT_IN takes the PDM_UL path */
		if (MM_EXT_IN_PORT == id) {
			/* set the PDM_UL SMEM buffer to /nul */
			sio_desc_address = dmem_port_descriptors + (PDM_UL_PORT * sizeof(ABE_SIODescriptor));
			abe_block_copy (COPY_FROM_ABE_TO_HOST, ABE_DMEM, sio_desc_address, (abe_uint32*)&desc, sizeof(desc));
			desc.smem_addr1 = (abe_uint16) Dummy_AM_labelID;
			abe_block_copy (COPY_FROM_HOST_TO_ABE, ABE_DMEM, sio_desc_address, (abe_uint32*)&desc, sizeof(desc));
		}

		if (abe_port[id].protocol.direction == ABE_ATC_DIRECTION_IN)
			direction = 0;
		else
			direction = 3; /* offset of the write pointer in the ATC descriptor */

		desc.drift_ASRC = 0;
		desc.drift_io = 0;
		desc.io_type_idx = (abe_uint8) io_sub_id;
		desc.samp_size = (abe_uint8) datasize;
		//desc.unused1 = (abe_uint8)0;
		//desc.unused2 = (abe_uint8)0;

		desc.hw_ctrl_addr = (abe_uint16) (dmareq_addr << 2);
		desc.atc_irq_data = (abe_uint8) dmareq_field;
		desc.flow_counter = (abe_uint16) 0;

		desc.direction_rw = (abe_uint8) direction;
		desc.nsamp = (abe_uint8) nsamp;
		desc.x_io = (abe_uint8) x_io;
		desc.on_off = 0x80; // set ATC ON

		desc.split_addr1 = (abe_uint16) smem1;
		desc.split_addr2 = (abe_uint16) smem2;
		desc.split_addr3 = (abe_uint16) smem3;
		desc.before_f_index = (abe_uint8) before_func_index;
		desc.after_f_index = (abe_uint8) after_func_index;

		desc.smem_addr1 = (abe_uint16) smem1;
		desc.atc_address1 = (abe_uint16) atc_desc_address1;
		desc.atc_pointer_saved1 = (abe_uint16) atc_ptr_saved;
		desc.data_size1 = (abe_uint8) datasize;
		desc.copy_f_index1 = (abe_uint8) copy_func_index1;

		desc.smem_addr2 = (abe_uint16) smem2;
		desc.atc_address2 = (abe_uint16) atc_desc_address2;
		desc.atc_pointer_saved2 = (abe_uint16) atc_ptr_saved2;
		desc.data_size2 = (abe_uint8) datasize2;
		desc.copy_f_index2 = (abe_uint8) copy_func_index2;

		sio_desc_address = dmem_port_descriptors + (id * sizeof(ABE_SIODescriptor));
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
			sio_desc_address, (abe_uint32*)&desc, sizeof(desc));
	}
}

/*
 *  ABE_INIT_DMIC
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
void abe_init_dmic(abe_uint32 x)
{
	just_to_avoid_the_many_warnings = x;
}

/*
 *  ABE_INIT_MCPDM
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
void abe_init_mcpdm(abe_uint32 x)
{
	just_to_avoid_the_many_warnings = x;
}

/*
 *  ABE_RESET_FEATURE
 *
 *  Parameter  :
 *      x : index of the feature to be initialized
 *
 *  Operations :
 *      reload the configuration
 *
 *  Return value :
 *
 */
void abe_reset_one_feature(abe_uint32 x)
{
	all_feature[x] = all_feature_init[x];	/* load default fields */
	/* abe_call_subroutine ((all_feature[x]).disable_feature, NOPARAMETER, NOPARAMETER, NOPARAMETER, NOPARAMETER);  */
}

/*
 *  ABE_RESET_ALL_FEATURE
 *
 *  Parameter  :
 *      none
 *
 *  Operations :
 *      load default configuration for all features
 *	 struct {
 *		uint16 load_default_data;
 *		uint16 read_parameter;
 *		uint16 write_parameter;
 *		uint16 running_status;
 *		uint16 fw_input_buffer_address;
 *		uint16 fw_output_buffer_address;
 *		uint16 fw_scheduler_slot_position;
 *		uint16 fw_scheduler_subslot_position;
 *		uint16 min_opp;
 *		char name[NBCHARFEATURENAME];
 *	} abe_feature_t;
 *
 *  Return value :
 *
 */
void abe_reset_all_features(void)
{
	abe_uint16 i;

	for (i = 0; i < FEAT_GAINS_DMIC1; i++)
		abe_reset_one_feature(i);
}

/*  ABE_RESET_ONE_PORT
 *
 *  Parameter  :
 *      none
 *
 *  Operations :
 *      load default configuration for one port
 *
 *  Return value :
 *
 */
void abe_reset_one_port(abe_uint32 x)
{
	abe_port[x] = abe_port_init[x];
}

/*
 *  ABE_RESET_ALL_PORTS
 *
 *  Parameter  :
 *      none
 *
 *  Operations :
 *      load default configuration for all features
 *
 *  Return value :
 *
 */
void abe_reset_all_ports(void)
{
	abe_uint16 i;

	for (i = 0; i < MAXNBABEPORTS; i++)
		abe_reset_one_port(i);

	/* mixers' configuration */
	abe_write_mixer(MIXDL1, GAIN_M6dB, RAMP_100MS, MIX_DL1_INPUT_MM_DL);
	abe_write_mixer(MIXDL1, MUTE_GAIN, RAMP_100MS, MIX_DL1_INPUT_MM_UL2);
	abe_write_mixer(MIXDL1, GAIN_M6dB, RAMP_100MS, MIX_DL1_INPUT_VX_DL);
	abe_write_mixer(MIXDL1, GAIN_M6dB, RAMP_100MS, MIX_DL1_INPUT_TONES);

	abe_write_mixer(MIXDL2, GAIN_0dB , RAMP_100MS, MIX_DL2_INPUT_TONES);
	abe_write_mixer(MIXDL2, GAIN_0dB , RAMP_100MS, MIX_DL2_INPUT_VX_DL);
	abe_write_mixer(MIXDL2, GAIN_0dB , RAMP_100MS, MIX_DL2_INPUT_MM_DL);
	abe_write_mixer(MIXDL2, MUTE_GAIN, RAMP_100MS, MIX_DL2_INPUT_MM_UL2);

	abe_write_mixer(MIXSDT, MUTE_GAIN, RAMP_100MS, MIX_SDT_INPUT_UP_MIXER);
	abe_write_mixer(MIXSDT, GAIN_0dB , RAMP_100MS, MIX_SDT_INPUT_DL1_MIXER);

	abe_write_mixer(MIXECHO, GAIN_0dB, RAMP_100MS, GAIN_LEFT_OFFSET);
	abe_write_mixer(MIXECHO, GAIN_0dB, RAMP_100MS, GAIN_RIGHT_OFFSET);

	abe_write_mixer(MIXAUDUL, MUTE_GAIN, RAMP_100MS, MIX_AUDUL_INPUT_MM_DL);
	abe_write_mixer(MIXAUDUL, MUTE_GAIN, RAMP_100MS, MIX_AUDUL_INPUT_TONES);
	abe_write_mixer(MIXAUDUL, GAIN_0dB, RAMP_100MS, MIX_AUDUL_INPUT_UPLINK);
	abe_write_mixer(MIXAUDUL, MUTE_GAIN, RAMP_100MS, MIX_AUDUL_INPUT_VX_DL);

	abe_write_mixer(MIXVXREC, GAIN_0dB, RAMP_100MS, MIX_VXREC_INPUT_TONES);
	abe_write_mixer(MIXVXREC, GAIN_0dB, RAMP_100MS, MIX_VXREC_INPUT_VX_DL);
	abe_write_mixer(MIXVXREC, GAIN_0dB, RAMP_100MS, MIX_VXREC_INPUT_MM_DL);
	abe_write_mixer(MIXVXREC, GAIN_0dB, RAMP_100MS, MIX_VXREC_INPUT_VX_UL);

	abe_write_gain(GAINS_DMIC1,GAIN_0dB, RAMP_100MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_DMIC1,GAIN_0dB, RAMP_100MS, GAIN_RIGHT_OFFSET);
	abe_write_gain(GAINS_DMIC2,GAIN_0dB, RAMP_100MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_DMIC2,GAIN_0dB, RAMP_100MS, GAIN_RIGHT_OFFSET);
	abe_write_gain(GAINS_DMIC3,GAIN_0dB, RAMP_100MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_DMIC3,GAIN_0dB, RAMP_100MS, GAIN_RIGHT_OFFSET);
	abe_write_gain(GAINS_AMIC,GAIN_0dB, RAMP_100MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_AMIC, GAIN_0dB, RAMP_100MS, GAIN_RIGHT_OFFSET);

	abe_write_gain(GAINS_SPLIT, GAIN_0dB, RAMP_100MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_SPLIT, GAIN_0dB, RAMP_100MS, GAIN_RIGHT_OFFSET);
	//abe_write_gain(GAINS_EANC ,GAIN_0dB , RAMP_100MS, GAIN_LEFT_OFFSET);
	//abe_write_gain(GAINS_EANC, GAIN_0dB , RAMP_100MS, GAIN_RIGHT_OFFSET);

	abe_write_gain(GAINS_DL1, GAIN_0dB, RAMP_100MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_DL1, GAIN_0dB, RAMP_100MS, GAIN_RIGHT_OFFSET);
	abe_write_gain(GAINS_DL2, GAIN_0dB, RAMP_100MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_DL2, GAIN_0dB, RAMP_100MS, GAIN_RIGHT_OFFSET);
}

/*
 *	ABE_CLEAN_TEMPORARY_BUFFERS
 *
 *	Parameter  :
 *	none
 *
 *	Operations :
 *	clear temporary buffers
 *
 *	Return value :
 *
 */
void abe_clean_temporary_buffers(abe_port_id id)
{
	switch (id) {
	case DMIC_PORT:
		abe_reset_mem(ABE_DMEM, D_DMIC_UL_FIFO_ADDR,D_DMIC_UL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_DMIC0_96_48_data_ADDR << 3, S_DMIC0_96_48_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_DMIC1_96_48_data_ADDR << 3, S_DMIC1_96_48_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_DMIC2_96_48_data_ADDR << 3, S_DMIC1_96_48_data_sizeof << 3);
		break;
	case PDM_UL_PORT:
		abe_reset_mem(ABE_DMEM, D_McPDM_UL_FIFO_ADDR, D_McPDM_UL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_BT_UL_ADDR << 3, S_BT_UL_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_AMIC_96_48_data_ADDR << 3, S_AMIC_96_48_data_sizeof << 3);
		break;
	case BT_VX_UL_PORT:	// ABE <-- BT (8/16kHz)
		abe_reset_mem(ABE_DMEM, D_BT_UL_FIFO_ADDR, D_BT_UL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_BT_UL_ADDR << 3, S_BT_UL_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_BT_UL_ADDR << 3, S_BT_UL_sizeof << 3);
		break;
	case MM_UL_PORT:
		abe_reset_mem(ABE_DMEM, D_MM_UL_FIFO_ADDR, D_MM_UL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_MM_UL_ADDR << 3, S_MM_UL_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_MM_UL2_ADDR << 3, D_MM_UL2_FIFO_sizeof << 3);
		break;
	case MM_UL2_PORT:
		abe_reset_mem(ABE_DMEM, D_MM_UL2_FIFO_ADDR, D_MM_UL2_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_MM_UL2_ADDR << 3, S_MM_UL2_sizeof << 3);
		break;
	case VX_UL_PORT:
		abe_reset_mem(ABE_DMEM, D_VX_UL_FIFO_ADDR, D_VX_UL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_VX_UL_ADDR << 3, S_VX_UL_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_VX_UL_48_8_BP_data_ADDR << 3, S_VX_UL_48_8_BP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_VX_UL_48_8_LP_data_ADDR << 3, S_VX_UL_48_8_LP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_VX_UL_48_16_HP_data_ADDR << 3, S_VX_UL_48_16_HP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_VX_UL_48_16_LP_data_ADDR << 3, S_VX_UL_48_16_LP_data_sizeof << 3);
		break;
	case MM_DL_PORT:
		abe_reset_mem(ABE_DMEM, D_MM_DL_FIFO_ADDR, D_MM_DL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_MM_DL_ADDR << 3,  S_MM_DL_sizeof << 3);
		break;
	case VX_DL_PORT:
		abe_reset_mem(ABE_DMEM, D_VX_DL_FIFO_ADDR, D_VX_DL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_VX_DL_ADDR << 3, S_VX_DL_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_VX_DL_8_48_BP_data_ADDR << 3, S_VX_DL_8_48_BP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_VX_DL_8_48_LP_data_ADDR << 3, S_VX_DL_8_48_LP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_VX_DL_16_48_HP_data_ADDR << 3, S_VX_DL_16_48_HP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_VX_DL_16_48_LP_data_ADDR << 3, S_VX_DL_16_48_LP_data_sizeof << 3);
		break;
	case TONES_DL_PORT:
		abe_reset_mem(ABE_DMEM, D_TONES_DL_FIFO_ADDR, D_TONES_DL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_Tones_ADDR << 3, S_Tones_sizeof << 3);
		break;
	case VIB_DL_PORT:
		abe_reset_mem(ABE_DMEM, D_VIB_DL_FIFO_ADDR, D_VIB_DL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_VIBRA_ADDR << 3, S_VIBRA_sizeof << 3);
		break;
	case BT_VX_DL_PORT:// ABE --> BT (8/16kHz)
		abe_reset_mem(ABE_DMEM, D_BT_DL_FIFO_ADDR, D_BT_DL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_BT_DL_ADDR << 3, S_BT_DL_sizeof << 3);
		break;
	case PDM_DL_PORT:
		abe_reset_mem(ABE_DMEM, D_McPDM_DL_FIFO_ADDR, D_McPDM_DL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_DMIC2_96_48_data_ADDR << 3, S_DMIC1_96_48_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_DL2_M_LR_EQ_data_ADDR << 3, S_DL2_M_LR_EQ_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_DL1_M_EQ_data_ADDR << 3, S_DL1_M_EQ_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_EARP_48_96_LP_data_ADDR << 3, S_EARP_48_96_LP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_IHF_48_96_LP_data_ADDR << 3, S_IHF_48_96_LP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_APS_DL1_EQ_data_ADDR << 3, S_APS_DL1_EQ_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_APS_DL2_EQ_data_ADDR << 3, S_APS_DL2_EQ_data_sizeof << 3);
		break;
	case MM_EXT_OUT_PORT:
		abe_reset_mem(ABE_DMEM, D_MM_EXT_OUT_FIFO_ADDR, D_MM_EXT_OUT_FIFO_sizeof);
		break;
	case MM_EXT_IN_PORT:
		abe_reset_mem(ABE_DMEM, D_MM_EXT_IN_FIFO_ADDR, D_MM_EXT_IN_FIFO_sizeof);
		break;
	default:
		break;
	}
}
