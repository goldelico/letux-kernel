/*
 * ==========================================================================
 *               Texas Instruments OMAP(TM) Platform Firmware
 * (c) Copyright 2009, Texas Instruments Incorporated.  All Rights Reserved.
 *
 *  Use of this firmware is controlled by the terms and conditions found
 *  in the license agreement under which this firmware has been supplied.
 * ==========================================================================
 */

#ifndef _ABE_FUNCTIONSID_H_
#define _ABE_FUNCTIONSID_H_

/*
 * TASK function ID definitions
 */
#define C_ABE_FW_FUNCTION_IIR					0
#define C_ABE_FW_FUNCTION_monoToStereoPack			1
#define C_ABE_FW_FUNCTION_stereoToMonoSplit			2
#define C_ABE_FW_FUNCTION_decimator				3
#define C_ABE_FW_FUNCTION_OS0Fill				4
#define C_ABE_FW_FUNCTION_mixer2				5
#define C_ABE_FW_FUNCTION_mixer4				6
#define C_ABE_FW_FUNCTION_inplaceGain				7
#define C_ABE_FW_FUNCTION_EANC					8
#define C_ABE_FW_FUNCTION_StreamRouting				9
#define C_ABE_FW_FUNCTION_VIBRA2				10
#define C_ABE_FW_FUNCTION_VIBRA1				11
#define C_ABE_FW_FUNCTION_APS_core				12
#define C_ABE_FW_FUNCTION_ASRC_DL_wrapper			13
#define C_ABE_FW_FUNCTION_ASRC_UL_wrapper			14
#define C_ABE_FW_FUNCTION_gainConverge				15
#define C_ABE_FW_FUNCTION_dualIir				16
#define C_ABE_FW_FUNCTION_EANC_wrapper				17
#define C_ABE_FW_FUNCTION_DCoffset				18
#define C_ABE_FW_FUNCTION_DCoffset2				19
#define C_ABE_FW_FUNCTION_IO_DL_pp				20
#define C_ABE_FW_FUNCTION_EANCUpdateOutSample			21
#define C_ABE_FW_FUNCTION_VX_DL_8_48_wrapper			22
#define C_ABE_FW_FUNCTION_VX_UL_48_8_wrapper			23
#define C_ABE_FW_FUNCTION_VX_DL_16_48_wrapper			24
#define C_ABE_FW_FUNCTION_VX_UL_48_16_wrapper			25
#define C_ABE_FW_FUNCTION_BT_UL_8_48_wrapper			26
#define C_ABE_FW_FUNCTION_BT_DL_48_8_wrapper			27
#define C_ABE_FW_FUNCTION_BT_UL_16_48_wrapper			28
#define C_ABE_FW_FUNCTION_BT_DL_48_16_wrapper			29
#define C_ABE_FW_FUNCTION_ECHO_REF_48_8_wrapper			30
#define C_ABE_FW_FUNCTION_ECHO_REF_48_16_wrapper		31
#define C_ABE_FW_FUNCTION_IO_generic2				32
#define C_ABE_FW_FUNCTION_irq_fifo_debug			33
#define C_ABE_FW_FUNCTION_synchronize_pointers			34
#define C_ABE_FW_FUNCTION_IIR_SRC_MIC				35
#define C_ABE_FW_FUNCTION_APS_FEEDBACK_DL1_wrapper		36
#define C_ABE_FW_FUNCTION_APS_FEEDBACK_DL2_L_wrapper		37
#define C_ABE_FW_FUNCTION_APS_FEEDBACK_DL2_R_wrapper		38

/*
 * COPY function ID definitions
 */
#define NULL_COPY_CFPID			0
#define COPY_D2S_LR_CFPID		1
#define COPY_D2S_2_CFPID		2
#define COPY_D2S_MONO_CFPID		3
#define COPY_S1D_MONO_CFPID		4
#define COPY_S2D_MONO_CFPID		5
#define COPY_S2D_2_CFPID		6
#define COPY_DMIC_CFPID			7
#define COPY_MCPDM_DL_CFPID		8
#define COPY_MM_UL_CFPID		9
#define SPLIT_SMEM_CFPID		10
#define MERGE_SMEM_CFPID		11
#define SPLIT_TDM_12_CFPID		12
#define MERGE_TDM_12_CFPID		13
#define ROUTE_MM_UL_CFPID		14
#define IO_DMAREQ_CFPID			15
#define IO_IP_CFPID			16

#endif	/* _ABE_FUNCTIONSID_H_ */
