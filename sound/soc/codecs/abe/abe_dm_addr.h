/*
 * ==========================================================================
 *		 Texas Instruments OMAP(TM) Platform Firmware
 * (c) Copyright 2009, Texas Instruments Incorporated.	All Rights Reserved.
 *
 *  Use of this firmware is controlled by the terms and conditions found
 *  in the license agreement under which this firmware has been supplied.
 * ==========================================================================
 */

#ifndef _ABE_DM_ADDR_H_
#define _ABE_DM_ADDR_H_

#define D_atcDescriptors_ADDR				0
#define D_atcDescriptors_ADDR_END			511
#define D_atcDescriptors_sizeof				512

#define stack_ADDR					512
#define stack_ADDR_END					623
#define stack_sizeof					112

#define D_version_ADDR					624
#define D_version_ADDR_END				627
#define D_version_sizeof				4

#define D_BT_DL_FIFO_ADDR				1024
#define D_BT_DL_FIFO_ADDR_END				1503
#define D_BT_DL_FIFO_sizeof				480

#define D_BT_UL_FIFO_ADDR				1536
#define D_BT_UL_FIFO_ADDR_END				2015
#define D_BT_UL_FIFO_sizeof				480

#define D_MM_EXT_OUT_FIFO_ADDR				2048
#define D_MM_EXT_OUT_FIFO_ADDR_END			2527
#define D_MM_EXT_OUT_FIFO_sizeof			480

#define D_MM_EXT_IN_FIFO_ADDR				2560
#define D_MM_EXT_IN_FIFO_ADDR_END			3039
#define D_MM_EXT_IN_FIFO_sizeof				480

#define D_MM_UL2_FIFO_ADDR				3072
#define D_MM_UL2_FIFO_ADDR_END				3551
#define D_MM_UL2_FIFO_sizeof				480

#define D_VX_UL_FIFO_ADDR				3584
#define D_VX_UL_FIFO_ADDR_END				4063
#define D_VX_UL_FIFO_sizeof				480

#define D_VX_DL_FIFO_ADDR				4096
#define D_VX_DL_FIFO_ADDR_END				4575
#define D_VX_DL_FIFO_sizeof				480

#define D_DMIC_UL_FIFO_ADDR				4608
#define D_DMIC_UL_FIFO_ADDR_END				5087
#define D_DMIC_UL_FIFO_sizeof				480

#define D_MM_UL_FIFO_ADDR				5120
#define D_MM_UL_FIFO_ADDR_END				5599
#define D_MM_UL_FIFO_sizeof				480

#define D_MM_DL_FIFO_ADDR				5632
#define D_MM_DL_FIFO_ADDR_END				6111
#define D_MM_DL_FIFO_sizeof				480

#define D_TONES_DL_FIFO_ADDR				6144
#define D_TONES_DL_FIFO_ADDR_END			6623
#define D_TONES_DL_FIFO_sizeof				480

#define D_VIB_DL_FIFO_ADDR				6656
#define D_VIB_DL_FIFO_ADDR_END				7135
#define D_VIB_DL_FIFO_sizeof				480

#define D_McPDM_DL_FIFO_ADDR				7168
#define D_McPDM_DL_FIFO_ADDR_END			7647
#define D_McPDM_DL_FIFO_sizeof				480

#define D_McPDM_UL_FIFO_ADDR				7680
#define D_McPDM_UL_FIFO_ADDR_END			8159
#define D_McPDM_UL_FIFO_sizeof				480

#define D_IOdescr_ADDR					8160
#define D_IOdescr_ADDR_END				8719
#define D_IOdescr_sizeof				560

#define D_debugATCptrs_ADDR				8720
#define D_debugATCptrs_ADDR_END				8783
#define D_debugATCptrs_sizeof				64

#define d_zero_ADDR					8784
#define d_zero_ADDR_END					8784
#define d_zero_sizeof					1

#define dbg_trace1_ADDR					8785
#define dbg_trace1_ADDR_END				8785
#define dbg_trace1_sizeof				1

#define dbg_trace2_ADDR					8786
#define dbg_trace2_ADDR_END				8786
#define dbg_trace2_sizeof				1

#define dbg_trace3_ADDR					8787
#define dbg_trace3_ADDR_END				8787
#define dbg_trace3_sizeof				1

#define D_multiFrame_ADDR				8788
#define D_multiFrame_ADDR_END				9187
#define D_multiFrame_sizeof				400

#define D_tasksList_ADDR				9188
#define D_tasksList_ADDR_END				11235
#define D_tasksList_sizeof				2048

#define D_idleTask_ADDR					11236
#define D_idleTask_ADDR_END				11237
#define D_idleTask_sizeof				2

#define D_typeLengthCheck_ADDR				11238
#define D_typeLengthCheck_ADDR_END			11239
#define D_typeLengthCheck_sizeof			2

#define D_maxTaskBytesInSlot_ADDR			11240
#define D_maxTaskBytesInSlot_ADDR_END			11241
#define D_maxTaskBytesInSlot_sizeof			2

#define D_rewindTaskBytes_ADDR				11242
#define D_rewindTaskBytes_ADDR_END			11243
#define D_rewindTaskBytes_sizeof			2

#define D_pCurrentTask_ADDR				11244
#define D_pCurrentTask_ADDR_END				11245
#define D_pCurrentTask_sizeof				2

#define D_pFastLoopBack_ADDR				11246
#define D_pFastLoopBack_ADDR_END			11247
#define D_pFastLoopBack_sizeof				2

#define D_pNextFastLoopBack_ADDR			11248
#define D_pNextFastLoopBack_ADDR_END			11251
#define D_pNextFastLoopBack_sizeof			4

#define D_ppCurrentTask_ADDR				11252
#define D_ppCurrentTask_ADDR_END			11253
#define D_ppCurrentTask_sizeof				2

#define D_slotCounter_ADDR				11256
#define D_slotCounter_ADDR_END				11257
#define D_slotCounter_sizeof				2

#define D_loopCounter_ADDR				11260
#define D_loopCounter_ADDR_END				11261
#define D_loopCounter_sizeof				2

#define D_RewindFlag_ADDR				11262
#define D_RewindFlag_ADDR_END				11263
#define D_RewindFlag_sizeof				2

#define D_Slot23_ctrl_ADDR				11264
#define D_Slot23_ctrl_ADDR_END				11267
#define D_Slot23_ctrl_sizeof				4

#define D_McuIrqFifo_ADDR				11268
#define D_McuIrqFifo_ADDR_END				11331
#define D_McuIrqFifo_sizeof				64

#define D_PingPongDesc_ADDR				11332
#define D_PingPongDesc_ADDR_END				11379
#define D_PingPongDesc_sizeof				48

#define D_PP_MCU_IRQ_ADDR				11380
#define D_PP_MCU_IRQ_ADDR_END				11381
#define D_PP_MCU_IRQ_sizeof				2

#define D_ctrlPortFifo_ADDR				11392
#define D_ctrlPortFifo_ADDR_END				11407
#define D_ctrlPortFifo_sizeof				16

#define D_Idle_State_ADDR				11408
#define D_Idle_State_ADDR_END				11411
#define D_Idle_State_sizeof				4

#define D_Stop_Request_ADDR				11412
#define D_Stop_Request_ADDR_END				11415
#define D_Stop_Request_sizeof				4

#define D_Ref0_ADDR					11416
#define D_Ref0_ADDR_END					11417
#define D_Ref0_sizeof					2

#define D_DebugRegister_ADDR				11420
#define D_DebugRegister_ADDR_END			11559
#define D_DebugRegister_sizeof				140

#define D_Gcount_ADDR					11560
#define D_Gcount_ADDR_END				11561
#define D_Gcount_sizeof					2

#define D_DCcounter_ADDR				11564
#define D_DCcounter_ADDR_END				11567
#define D_DCcounter_sizeof				4

#define D_DCsum_ADDR					11568
#define D_DCsum_ADDR_END				11575
#define D_DCsum_sizeof					8

#define D_fastCounter_ADDR				11576
#define D_fastCounter_ADDR_END				11579
#define D_fastCounter_sizeof				4

#define D_slowCounter_ADDR				11580
#define D_slowCounter_ADDR_END				11583
#define D_slowCounter_sizeof				4

#define D_aUplinkRouting_ADDR				11584
#define D_aUplinkRouting_ADDR_END			11599
#define D_aUplinkRouting_sizeof				16

#define D_VirtAudioLoop_ADDR				11600
#define D_VirtAudioLoop_ADDR_END			11603
#define D_VirtAudioLoop_sizeof				4

#define D_AsrcVars_DL_VX_ADDR				11604
#define D_AsrcVars_DL_VX_ADDR_END			11635
#define D_AsrcVars_DL_VX_sizeof				32

#define D_AsrcVars_UL_VX_ADDR				11636
#define D_AsrcVars_UL_VX_ADDR_END			11667
#define D_AsrcVars_UL_VX_sizeof				32

#define D_CoefAddresses_VX_ADDR				11668
#define D_CoefAddresses_VX_ADDR_END			11699
#define D_CoefAddresses_VX_sizeof			32

#define D_AsrcVars_DL_MM_ADDR				11700
#define D_AsrcVars_DL_MM_ADDR_END			11731
#define D_AsrcVars_DL_MM_sizeof				32

#define D_CoefAddresses_DL_MM_ADDR			11732
#define D_CoefAddresses_DL_MM_ADDR_END			11763
#define D_CoefAddresses_DL_MM_sizeof			32

#define D_APS_DL1_M_thresholds_ADDR			11764
#define D_APS_DL1_M_thresholds_ADDR_END			11771
#define D_APS_DL1_M_thresholds_sizeof			8

#define D_APS_DL1_M_IRQ_ADDR				11772
#define D_APS_DL1_M_IRQ_ADDR_END			11773
#define D_APS_DL1_M_IRQ_sizeof				2

#define D_APS_DL1_C_IRQ_ADDR				11774
#define D_APS_DL1_C_IRQ_ADDR_END			11775
#define D_APS_DL1_C_IRQ_sizeof				2

#define D_TraceBufAdr_ADDR				11776
#define D_TraceBufAdr_ADDR_END				11777
#define D_TraceBufAdr_sizeof				2

#define D_TraceBufOffset_ADDR				11778
#define D_TraceBufOffset_ADDR_END			11779
#define D_TraceBufOffset_sizeof				2

#define D_TraceBufLength_ADDR				11780
#define D_TraceBufLength_ADDR_END			11781
#define D_TraceBufLength_sizeof				2

#define D_AsrcVars_ECHO_REF_ADDR			11784
#define D_AsrcVars_ECHO_REF_ADDR_END			11815
#define D_AsrcVars_ECHO_REF_sizeof			32

#define D_Pempty_ADDR					11816
#define D_Pempty_ADDR_END				11819
#define D_Pempty_sizeof					4

#define D_APS_DL2_L_M_IRQ_ADDR				11820
#define D_APS_DL2_L_M_IRQ_ADDR_END			11821
#define D_APS_DL2_L_M_IRQ_sizeof			2

#define D_APS_DL2_L_C_IRQ_ADDR				11822
#define D_APS_DL2_L_C_IRQ_ADDR_END			11823
#define D_APS_DL2_L_C_IRQ_sizeof			2

#define D_APS_DL2_R_M_IRQ_ADDR				11824
#define D_APS_DL2_R_M_IRQ_ADDR_END			11825
#define D_APS_DL2_R_M_IRQ_sizeof			2

#define D_APS_DL2_R_C_IRQ_ADDR				11826
#define D_APS_DL2_R_C_IRQ_ADDR_END			11827
#define D_APS_DL2_R_C_IRQ_sizeof			2

#define D_APS_DL1_C_thresholds_ADDR			11828
#define D_APS_DL1_C_thresholds_ADDR_END			11835
#define D_APS_DL1_C_thresholds_sizeof			8

#define D_APS_DL2_L_M_thresholds_ADDR			11836
#define D_APS_DL2_L_M_thresholds_ADDR_END		11843
#define D_APS_DL2_L_M_thresholds_sizeof			8

#define D_APS_DL2_L_C_thresholds_ADDR			11844
#define D_APS_DL2_L_C_thresholds_ADDR_END		11851
#define D_APS_DL2_L_C_thresholds_sizeof			8

#define D_APS_DL2_R_M_thresholds_ADDR			11852
#define D_APS_DL2_R_M_thresholds_ADDR_END		11859
#define D_APS_DL2_R_M_thresholds_sizeof			8

#define D_APS_DL2_R_C_thresholds_ADDR			11860
#define D_APS_DL2_R_C_thresholds_ADDR_END		11867
#define D_APS_DL2_R_C_thresholds_sizeof			8

#define D_nextMultiFrame_ADDR				11868
#define D_nextMultiFrame_ADDR_END			11875
#define D_nextMultiFrame_sizeof				8

#define D_PING_ADDR					16384
#define D_PING_ADDR_END					40959
#define D_PING_sizeof					24576

#define D_PONG_ADDR					40960
#define D_PONG_ADDR_END					65535
#define D_PONG_sizeof					24576

#endif	/* _ABE_DM_ADDR_H_ */
