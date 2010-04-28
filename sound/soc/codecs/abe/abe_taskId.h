/*
 * ==========================================================================
 *               Texas Instruments OMAP(TM) Platform Firmware
 * (c) Copyright 2009, Texas Instruments Incorporated.  All Rights Reserved.
 *
 *  Use of this firmware is controlled by the terms and conditions found
 *  in the license agreement under which this firmware has been supplied.
 * ==========================================================================
 */

#ifndef _ABE_TASKID_H_
#define _ABE_TASKID_H_

#define C_ABE_FW_TASK_test			0
#define C_ABE_FW_TASK_void			1
#define C_ABE_FW_TASK_cluster			2
#define C_ABE_FW_TASK_DL1Mixer			3
#define C_ABE_FW_TASK_DL2Mixer			4
#define C_ABE_FW_TASK_EchoMixer			5
#define C_ABE_FW_TASK_SideTone			6
#define C_ABE_FW_TASK_SDTMixer			7
#define C_ABE_FW_TASK_IO_DMIC			8
#define C_ABE_FW_TASK_IO_DMIC2			9
#define C_ABE_FW_TASK_AMIC_96_48_LP		10
#define C_ABE_FW_TASK_AMIC_96_48_DEC		11
#define C_ABE_FW_TASK_AMIC_EQ			12
#define C_ABE_FW_TASK_DMIC1_96_48_LP		13
#define C_ABE_FW_TASK_DMIC1_96_48_DEC		14
#define C_ABE_FW_TASK_DMIC1_EQ			15
#define C_ABE_FW_TASK_DMIC2_96_48_LP		16
#define C_ABE_FW_TASK_DMIC2_96_48_DEC		17
#define C_ABE_FW_TASK_DMIC2_EQ			18
#define C_ABE_FW_TASK_DMIC3_96_48_LP		19
#define C_ABE_FW_TASK_DMIC3_96_48_DEC		20
#define C_ABE_FW_TASK_DMIC3_EQ			21
#define C_ABE_FW_TASK_DMIC1_SPLIT		22
#define C_ABE_FW_TASK_DMIC2_SPLIT		23
#define C_ABE_FW_TASK_DMIC3_SPLIT		24
#define C_ABE_FW_TASK_AMIC_SPLIT		25
#define C_ABE_FW_TASK_IO_MM_UL_test		26
#define C_ABE_FW_TASK_IO_MM_UL2_test		27
#define C_ABE_FW_TASK_IO_VX_UL			28
#define C_ABE_FW_TASK_IO_VX_DL			29
#define C_ABE_FW_TASK_IO_TONES_DL		30
#define C_ABE_FW_TASK_IO_MM_DL			31
#define C_ABE_FW_TASK_IO_VIB_DL_test		32
#define C_ABE_FW_TASK_IO_AMIC			33
#define C_ABE_FW_TASK_McPDM_DL			34
#define C_ABE_FW_TASK_VX_UL_ROUTING		35
#define C_ABE_FW_TASK_MM_UL2_ROUTING		36
#define C_ABE_FW_TASK_MM_UL_ROUTING		37
#define C_ABE_FW_TASK_IO_MM_UL			38
#define C_ABE_FW_TASK_IO_MM_UL2			39
#define C_ABE_FW_TASK_ASRC_VX_DL_8		40
#define C_ABE_FW_TASK_ASRC_VX_UL_8		41
#define C_ABE_FW_TASK_ASRC_VX_DL_16		42
#define C_ABE_FW_TASK_ASRC_VX_UL_16		43
#define C_ABE_FW_TASK_ASRC_MM_DL		44
#define C_ABE_FW_TASK_VXRECMixer		45
#define C_ABE_FW_TASK_VXREC_SPLIT		46
#define C_ABE_FW_TASK_ULMixer			47
#define C_ABE_FW_TASK_MM_SPLIT			48
#define C_ABE_FW_TASK_VX_DL_8_48_BP		49
#define C_ABE_FW_TASK_VX_DL_8_48_0SR		50
#define C_ABE_FW_TASK_VX_DL_8_48_LP		51
#define C_ABE_FW_TASK_VX_DL_16_48_HP		52
#define C_ABE_FW_TASK_VX_DL_16_48_0SR		53
#define C_ABE_FW_TASK_VX_DL_16_48_LP		54
#define C_ABE_FW_TASK_DL1_EQ			55
#define C_ABE_FW_TASK_DL1_GAIN			56
#define C_ABE_FW_TASK_EARP_48_96_0SR		57
#define C_ABE_FW_TASK_EARP_48_96_LP		58
#define C_ABE_FW_TASK_DL2_EQ			59
#define C_ABE_FW_TASK_DL2_GAIN			60
#define C_ABE_FW_TASK_IHF_48_96_0SR		61
#define C_ABE_FW_TASK_IHF_48_96_LP		62
#define C_ABE_FW_TASK_VX_UL_48_8_LP		63
#define C_ABE_FW_TASK_VX_UL_48_8_DEC1		64
#define C_ABE_FW_TASK_VX_UL_48_8_BP		65
#define C_ABE_FW_TASK_VX_UL_48_16_LP		66
#define C_ABE_FW_TASK_VX_UL_48_16_DEC1		67
#define C_ABE_FW_TASK_VX_UL_48_16_HP		68
#define C_ABE_FW_TASK_GAIN_UPDATE		69
#define C_ABE_FW_TASK_DELAY_CMD			70
#define C_ABE_FW_TASK_ASRC_ECHO_REF_8		71
#define C_ABE_FW_TASK_ASRC_ECHO_REF_16		72
#define C_ABE_FW_TASK_DL1_APS_IIR		73
#define C_ABE_FW_TASK_DL1_APS_CORE		74
#define C_ABE_FW_TASK_DL2_APS_IIR		75
#define C_ABE_FW_TASK_DL2_APS_SPLIT		76
#define C_ABE_FW_TASK_DL2_L_APS_CORE		77
#define C_ABE_FW_TASK_DL2_R_APS_CORE		78
#define C_ABE_FW_TASK_ECHO_REF_48_8_LP		79
#define C_ABE_FW_TASK_ECHO_REF_48_8_DEC1	80
#define C_ABE_FW_TASK_ECHO_REF_48_8_BP		81
#define C_ABE_FW_TASK_ECHO_REF_48_16_LP		82
#define C_ABE_FW_TASK_ECHO_REF_48_16_DEC1	83
#define C_ABE_FW_TASK_ECHO_REF_48_16_HP		84
#define C_ABE_FW_TASK_EANC			85
#define C_ABE_FW_TASK_EANC_WRAP			86
#define C_ABE_FW_TASK_DL1_APS_EQ		87
#define C_ABE_FW_TASK_DL2_APS_EQ		88
#define C_ABE_FW_TASK_DC_REMOVAL		89
#define C_ABE_FW_TASK_IO_VIBRA_DL		90
#define C_ABE_FW_TASK_VIBRA1			91
#define C_ABE_FW_TASK_VIBRA2			92
#define C_ABE_FW_TASK_VIBRA_SPLIT		93
#define C_ABE_FW_TASK_VIBRA_PACK		94
#define C_ABE_FW_TASK_IO_PING_PONG		95

#endif	/* _ABE_TASKID_H_ */
