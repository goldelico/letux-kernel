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
#define C_ABE_FW_FUNCTION_nullCall		0
#define C_ABE_FW_FUNCTION_test			1
#define C_ABE_FW_FUNCTION_IIR			2
#define C_ABE_FW_FUNCTION_monoToStereoPack	3
#define C_ABE_FW_FUNCTION_stereoToMonoSplit	4
#define C_ABE_FW_FUNCTION_decimator		5
#define C_ABE_FW_FUNCTION_OS0Fill		6
#define C_ABE_FW_FUNCTION_mixer2		7
#define C_ABE_FW_FUNCTION_mixer4		8
#define C_ABE_FW_FUNCTION_taskCluster		9
#define C_ABE_FW_FUNCTION_treatDelayedTasksFifo	10
#define C_ABE_FW_FUNCTION_treatPostedIrq	11
#define C_ABE_FW_FUNCTION_inplaceGain		12
#define C_ABE_FW_FUNCTION_EANC			13
#define C_ABE_FW_FUNCTION_IO_MM_UL_old		14
#define C_ABE_FW_FUNCTION_fillMcPDMOutFifo	15
#define C_ABE_FW_FUNCTION_IO_generic		16
#define C_ABE_FW_FUNCTION_IO_DMIC_UL		17
#define C_ABE_FW_FUNCTION_IO_AMIC_UL		18
#define C_ABE_FW_FUNCTION_StreamRouting		19
#define C_ABE_FW_FUNCTION_StreamRoutingLoop	20
#define C_ABE_FW_FUNCTION_VIBRA2		21
#define C_ABE_FW_FUNCTION_VIBRA1		22
#define C_ABE_FW_FUNCTION_APS_core		23
#define C_ABE_FW_FUNCTION_ASRC_DL_wrapper	24
#define C_ABE_FW_FUNCTION_ASRC_UL_wrapper	25
#define C_ABE_FW_FUNCTION_IO_MM_UL		26
#define C_ABE_FW_FUNCTION_gainConverge		27
#define C_ABE_FW_FUNCTION_dualIir		28
#define C_ABE_FW_FUNCTION_EANC_wrapper		29
#define C_ABE_FW_FUNCTION_DCoffset		30
#define C_ABE_FW_FUNCTION_IO_DL_pp		31

/*
 * COMMAND function ID definitions
 */
#define nullCommand_CMDID			0
#define cmdMemcpyDM2SM_CMDID			1
#define cmdMemcpyDM2CM_CMDID			2
#define cmdMemcpySM2SM_CMDID			3
#define cmdMemcpyCM2CM_CMDID			4
#define cmdMemcpySM2DM_CMDID			5
#define cmdMemcpyCM2DM_CMDID			6
#define cmdClearCM_CMDID			7
#define cmdClearSM_CMDID			8
#define cmdSendIRQ_CMDID			9
#define cmdMemcpyDM2DM_CMDID			10

/*
 * COPY function ID definitions
 */
#define COPY_D2S_LR_CFPID			0
#define COPY_D2S_2_CFPID			1
#define COPY_D2S_MONO_CFPID			2
#define COPY_S1D_MONO_CFPID			3
#define COPY_S2D_MONO_CFPID			4
#define COPY_S2D_2_CFPID			5

#endif	/* _ABE_FUNCTIONSID_H_ */
