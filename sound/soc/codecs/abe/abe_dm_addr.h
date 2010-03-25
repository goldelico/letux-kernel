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

#define D_atcDescriptors_ADDR			0
#define D_atcDescriptors_ADDR_END		511
#define D_atcDescriptors_sizeof			512

#define stack_ADDR				512
#define stack_ADDR_END				623
#define stack_sizeof				112

#define D_version_ADDR				624
#define D_version_ADDR_END			627
#define D_version_sizeof			4

#define D_BT_DL_FIFO_ADDR			768
#define D_BT_DL_FIFO_ADDR_END			1087
#define D_BT_DL_FIFO_sizeof			320

#define D_BT_UL_FIFO_ADDR			1280
#define D_BT_UL_FIFO_ADDR_END			1599
#define D_BT_UL_FIFO_sizeof			320

#define D_MM_EXT_OUT_FIFO_ADDR			1792
#define D_MM_EXT_OUT_FIFO_ADDR_END		2111
#define D_MM_EXT_OUT_FIFO_sizeof		320

#define D_MM_EXT_IN_FIFO_ADDR			2304
#define D_MM_EXT_IN_FIFO_ADDR_END		2623
#define D_MM_EXT_IN_FIFO_sizeof			320

#define D_MM_UL2_FIFO_ADDR			2816
#define D_MM_UL2_FIFO_ADDR_END			2975
#define D_MM_UL2_FIFO_sizeof			160

#define D_VX_UL_FIFO_ADDR			3072
#define D_VX_UL_FIFO_ADDR_END			3391
#define D_VX_UL_FIFO_sizeof			320

#define D_VX_DL_FIFO_ADDR			3584
#define D_VX_DL_FIFO_ADDR_END			3743
#define D_VX_DL_FIFO_sizeof			160

#define D_DMIC_UL_FIFO_ADDR			3840
#define D_DMIC_UL_FIFO_ADDR_END			4319
#define D_DMIC_UL_FIFO_sizeof			480

#define D_MM_UL_FIFO_ADDR			4352
#define D_MM_UL_FIFO_ADDR_END			4847
#define D_MM_UL_FIFO_sizeof			496

#define D_MM_DL_FIFO_ADDR			4864
#define D_MM_DL_FIFO_ADDR_END			5023
#define D_MM_DL_FIFO_sizeof			160

#define D_TONES_DL_FIFO_ADDR			5120
#define D_TONES_DL_FIFO_ADDR_END		5279
#define D_TONES_DL_FIFO_sizeof			160

#define D_VIB_DL_FIFO_ADDR			5376
#define D_VIB_DL_FIFO_ADDR_END			5535
#define D_VIB_DL_FIFO_sizeof			160

#define D_McPDM_DL_FIFO_ADDR			5632
#define D_McPDM_DL_FIFO_ADDR_END		6111
#define D_McPDM_DL_FIFO_sizeof			480

#define D_McPDM_UL_FIFO_ADDR			6144
#define D_McPDM_UL_FIFO_ADDR_END		6623
#define D_McPDM_UL_FIFO_sizeof			480

#define D_IOdescr_ADDR				6624
#define D_IOdescr_ADDR_END			7183
#define D_IOdescr_sizeof			560

#define D_debugATCptrs_ADDR			7184
#define D_debugATCptrs_ADDR_END			7247
#define D_debugATCptrs_sizeof			64

#define d_zero_ADDR				7248
#define d_zero_ADDR_END				7248
#define d_zero_sizeof				1

#define dbg_trace1_ADDR				7249
#define dbg_trace1_ADDR_END			7249
#define dbg_trace1_sizeof			1

#define dbg_trace2_ADDR				7250
#define dbg_trace2_ADDR_END			7250
#define dbg_trace2_sizeof			1

#define dbg_trace3_ADDR				7251
#define dbg_trace3_ADDR_END			7251
#define dbg_trace3_sizeof			1

#define D_multiFrame_ADDR			7252
#define D_multiFrame_ADDR_END			7651
#define D_multiFrame_sizeof			400

#define D_tasksList_ADDR			7652
#define D_tasksList_ADDR_END			9699
#define D_tasksList_sizeof			2048

#define D_idleTask_ADDR				9700
#define D_idleTask_ADDR_END			9701
#define D_idleTask_sizeof			2

#define D_typeLengthCheck_ADDR			9702
#define D_typeLengthCheck_ADDR_END		9703
#define D_typeLengthCheck_sizeof		2

#define D_maxTaskBytesInSlot_ADDR		9704
#define D_maxTaskBytesInSlot_ADDR_END		9705
#define D_maxTaskBytesInSlot_sizeof		2

#define D_rewindTaskBytes_ADDR			9706
#define D_rewindTaskBytes_ADDR_END		9707
#define D_rewindTaskBytes_sizeof		2

#define D_pCurrentTask_ADDR			9708
#define D_pCurrentTask_ADDR_END			9709
#define D_pCurrentTask_sizeof			2

#define D_pFastLoopBack_ADDR			9710
#define D_pFastLoopBack_ADDR_END		9711
#define D_pFastLoopBack_sizeof			2

#define D_pNextFastLoopBack_ADDR		9712
#define D_pNextFastLoopBack_ADDR_END		9715
#define D_pNextFastLoopBack_sizeof		4

#define D_ppCurrentTask_ADDR			9716
#define D_ppCurrentTask_ADDR_END		9717
#define D_ppCurrentTask_sizeof			2

#define D_slotCounter_ADDR			9720
#define D_slotCounter_ADDR_END			9721
#define D_slotCounter_sizeof			2

#define D_loopCounter_ADDR			9724
#define D_loopCounter_ADDR_END			9725
#define D_loopCounter_sizeof			2

#define D_RewindFlag_ADDR			9726
#define D_RewindFlag_ADDR_END			9727
#define D_RewindFlag_sizeof			2

#define D_Slot23_ctrl_ADDR			9728
#define D_Slot23_ctrl_ADDR_END			9731
#define D_Slot23_ctrl_sizeof			4

#define D_McuIrqFifo_ADDR			9732
#define D_McuIrqFifo_ADDR_END			9795
#define D_McuIrqFifo_sizeof			64

#define D_PingPongDesc_ADDR			9796
#define D_PingPongDesc_ADDR_END			9843
#define D_PingPongDesc_sizeof			48

#define D_PP_MCU_IRQ_ADDR			9844
#define D_PP_MCU_IRQ_ADDR_END			9845
#define D_PP_MCU_IRQ_sizeof			2

#define D_ctrlPortFifo_ADDR			9856
#define D_ctrlPortFifo_ADDR_END			9871
#define D_ctrlPortFifo_sizeof			16

#define D_Idle_State_ADDR			9872
#define D_Idle_State_ADDR_END			9875
#define D_Idle_State_sizeof			4

#define D_Stop_Request_ADDR			9876
#define D_Stop_Request_ADDR_END			9879
#define D_Stop_Request_sizeof			4

#define D_Ref0_ADDR				9880
#define D_Ref0_ADDR_END				9881
#define D_Ref0_sizeof				2

#define D_DebugRegister_ADDR			9884
#define D_DebugRegister_ADDR_END		10023
#define D_DebugRegister_sizeof			140

#define D_Gcount_ADDR				10024
#define D_Gcount_ADDR_END			10025
#define D_Gcount_sizeof				2

#define D_DCcounter_ADDR			10028
#define D_DCcounter_ADDR_END			10031
#define D_DCcounter_sizeof			4

#define D_DCsum_ADDR				10032
#define D_DCsum_ADDR_END			10039
#define D_DCsum_sizeof				8

#define D_fastCounter_ADDR			10040
#define D_fastCounter_ADDR_END			10043
#define D_fastCounter_sizeof			4

#define D_slowCounter_ADDR			10044
#define D_slowCounter_ADDR_END			10047
#define D_slowCounter_sizeof			4

#define D_aUplinkRouting_ADDR			10048
#define D_aUplinkRouting_ADDR_END		10063
#define D_aUplinkRouting_sizeof			16

#define D_VirtAudioLoop_ADDR			10064
#define D_VirtAudioLoop_ADDR_END		10067
#define D_VirtAudioLoop_sizeof			4

#define D_AsrcVars_DL_VX_ADDR			10068
#define D_AsrcVars_DL_VX_ADDR_END		10099
#define D_AsrcVars_DL_VX_sizeof			32

#define D_AsrcVars_UL_VX_ADDR			10100
#define D_AsrcVars_UL_VX_ADDR_END		10131
#define D_AsrcVars_UL_VX_sizeof			32

#define D_CoefAddresses_VX_ADDR			10132
#define D_CoefAddresses_VX_ADDR_END		10163
#define D_CoefAddresses_VX_sizeof		32

#define D_AsrcVars_DL_MM_ADDR			10164
#define D_AsrcVars_DL_MM_ADDR_END		10195
#define D_AsrcVars_DL_MM_sizeof			32

#define D_CoefAddresses_DL_MM_ADDR		10196
#define D_CoefAddresses_DL_MM_ADDR_END		10227
#define D_CoefAddresses_DL_MM_sizeof		32

#define D_APS_DL1_M_thresholds_ADDR		10228
#define D_APS_DL1_M_thresholds_ADDR_END		10235
#define D_APS_DL1_M_thresholds_sizeof		8

#define D_APS_DL1_M_IRQ_ADDR			10236
#define D_APS_DL1_M_IRQ_ADDR_END		10237
#define D_APS_DL1_M_IRQ_sizeof			2

#define D_APS_DL1_C_IRQ_ADDR			10238
#define D_APS_DL1_C_IRQ_ADDR_END		10239
#define D_APS_DL1_C_IRQ_sizeof			2

#define D_TraceBufAdr_ADDR			10240
#define D_TraceBufAdr_ADDR_END			10241
#define D_TraceBufAdr_sizeof			2

#define D_TraceBufOffset_ADDR			10242
#define D_TraceBufOffset_ADDR_END		10243
#define D_TraceBufOffset_sizeof			2

#define D_TraceBufLength_ADDR			10244
#define D_TraceBufLength_ADDR_END		10245
#define D_TraceBufLength_sizeof			2

#define D_AsrcVars_ECHO_REF_ADDR		10248
#define D_AsrcVars_ECHO_REF_ADDR_END		10279
#define D_AsrcVars_ECHO_REF_sizeof		32

#define D_Pempty_ADDR				10280
#define D_Pempty_ADDR_END			10283
#define D_Pempty_sizeof				4

#define D_APS_DL2_L_M_IRQ_ADDR			10284
#define D_APS_DL2_L_M_IRQ_ADDR_END		10285
#define D_APS_DL2_L_M_IRQ_sizeof		2

#define D_APS_DL2_L_C_IRQ_ADDR			10286
#define D_APS_DL2_L_C_IRQ_ADDR_END		10287
#define D_APS_DL2_L_C_IRQ_sizeof		2

#define D_APS_DL2_R_M_IRQ_ADDR			10288
#define D_APS_DL2_R_M_IRQ_ADDR_END		10289
#define D_APS_DL2_R_M_IRQ_sizeof		2

#define D_APS_DL2_R_C_IRQ_ADDR			10290
#define D_APS_DL2_R_C_IRQ_ADDR_END		10291
#define D_APS_DL2_R_C_IRQ_sizeof		2

#define D_APS_DL1_C_thresholds_ADDR		10292
#define D_APS_DL1_C_thresholds_ADDR_END		10299
#define D_APS_DL1_C_thresholds_sizeof		8

#define D_APS_DL2_L_M_thresholds_ADDR		10300
#define D_APS_DL2_L_M_thresholds_ADDR_END	10307
#define D_APS_DL2_L_M_thresholds_sizeof		8

#define D_APS_DL2_L_C_thresholds_ADDR		10308
#define D_APS_DL2_L_C_thresholds_ADDR_END	10315
#define D_APS_DL2_L_C_thresholds_sizeof		8

#define D_APS_DL2_R_M_thresholds_ADDR		10316
#define D_APS_DL2_R_M_thresholds_ADDR_END	10323
#define D_APS_DL2_R_M_thresholds_sizeof		8

#define D_APS_DL2_R_C_thresholds_ADDR		10324
#define D_APS_DL2_R_C_thresholds_ADDR_END	10331
#define D_APS_DL2_R_C_thresholds_sizeof		8

#define D_nextMultiFrame_ADDR			10332
#define D_nextMultiFrame_ADDR_END		10339
#define D_nextMultiFrame_sizeof			8

#define D_PING_ADDR				16384
#define D_PING_ADDR_END				40959
#define D_PING_sizeof				24576

#define D_PONG_ADDR				40960
#define D_PONG_ADDR_END				65535
#define D_PONG_sizeof				24576

#endif	/* _ABE_DM_ADDR_H_ */
