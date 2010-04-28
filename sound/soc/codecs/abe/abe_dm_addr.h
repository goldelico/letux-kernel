/*
 * ==========================================================================
 *               Texas Instruments OMAP(TM) Platform Firmware
 * (c) Copyright 2009, Texas Instruments Incorporated.  All Rights Reserved.
 *
 *  Use of this firmware is controlled by the terms and conditions found
 *  in the license agreement under which this firmware has been supplied.
 * ==========================================================================
 */

#ifndef _ABE_DM_ADDR_H_
#define _ABE_DM_ADDR_H_

#define D_atcDescriptors_ADDR			0
#define D_atcDescriptors_ADDR_END		511
#define D_atcDescriptors_sizeof		512

#define D_McPDM_UL_Fifo_ADDR			512
#define D_McPDM_UL_Fifo_ADDR_END		1019
#define D_McPDM_UL_Fifo_sizeof			508

#define D_McPDM_DL_Fifo_ADDR			1024
#define D_McPDM_DL_Fifo_ADDR_END		1531
#define D_McPDM_DL_Fifo_sizeof			508

#define D_DMIC_UL_Fifo_ADDR			1536
#define D_DMIC_UL_Fifo_ADDR_END		2043
#define D_DMIC_UL_Fifo_sizeof			508

#define D_MM_UL_FIFO_ADDR			2048
#define D_MM_UL_FIFO_ADDR_END			2535
#define D_MM_UL_FIFO_sizeof			488

#define D_MM_DL_ADDR				2560
#define D_MM_DL_ADDR_END			2847
#define D_MM_DL_sizeof				288

#define stack_ADDR				2848
#define stack_ADDR_END				2975
#define stack_sizeof				128

#define D_version_ADDR				2976
#define D_version_ADDR_END			2979
#define D_version_sizeof			4

#define D_tasksList_ADDR			2980
#define D_tasksList_ADDR_END			4003
#define D_tasksList_sizeof			1024

#define D_multiFrame_ADDR			4004
#define D_multiFrame_ADDR_END			4195
#define D_multiFrame_sizeof			192

#define D_nextMultiFrame_ADDR			4196
#define D_nextMultiFrame_ADDR_END		4387
#define D_nextMultiFrame_sizeof		192

#define D_taskClusters_ADDR			4388
#define D_taskClusters_ADDR_END		4451
#define D_taskClusters_sizeof			64

#define D_idleTask_ADDR			4452
#define D_idleTask_ADDR_END			4453
#define D_idleTask_sizeof			2

#define D_typeLengthCheck_ADDR			4454
#define D_typeLengthCheck_ADDR_END		4455
#define D_typeLengthCheck_sizeof		2

#define D_taskStep_ADDR			4456
#define D_taskStep_ADDR_END			4459
#define D_taskStep_sizeof			4

#define D_pCurrentTask_ADDR			4460
#define D_pCurrentTask_ADDR_END		4461
#define D_pCurrentTask_sizeof			2

#define D_pFastLoopBack_ADDR			4462
#define D_pFastLoopBack_ADDR_END		4463
#define D_pFastLoopBack_sizeof			2

#define D_pNextFastLoopBack_ADDR		4464
#define D_pNextFastLoopBack_ADDR_END		4467
#define D_pNextFastLoopBack_sizeof		4

#define D_ppCurrentTask_ADDR			4468
#define D_ppCurrentTask_ADDR_END		4469
#define D_ppCurrentTask_sizeof			2

#define D_loopCounter_ADDR			4470
#define D_loopCounter_ADDR_END			4471
#define D_loopCounter_sizeof			2

#define D_idleSlotsReq_ADDR			4472
#define D_idleSlotsReq_ADDR_END		4473
#define D_idleSlotsReq_sizeof			2

#define D_PostMcuIrq_ADDR			4476
#define D_PostMcuIrq_ADDR_END			4479
#define D_PostMcuIrq_sizeof			4

#define D_PostDspIrq_ADDR			4480
#define D_PostDspIrq_ADDR_END			4483
#define D_PostDspIrq_sizeof			4

#define D_McuIrqFifo_ADDR			4484
#define D_McuIrqFifo_ADDR_END			4507
#define D_McuIrqFifo_sizeof			24

#define D_DspIrqFifo_ADDR			4508
#define D_DspIrqFifo_ADDR_END			4531
#define D_DspIrqFifo_sizeof			24

#define D_McuIrqFifoDesc_ADDR			4532
#define D_McuIrqFifoDesc_ADDR_END		4543
#define D_McuIrqFifoDesc_sizeof		12

#define D_DspIrqFifoDesc_ADDR			4544
#define D_DspIrqFifoDesc_ADDR_END		4555
#define D_DspIrqFifoDesc_sizeof		12

#define D_debugFifo_ADDR			4556
#define D_debugFifo_ADDR_END			4619
#define D_debugFifo_sizeof			64

#define D_debugFifoDesc_ADDR			4620
#define D_debugFifoDesc_ADDR_END		4631
#define D_debugFifoDesc_sizeof			12

#define D_DCFifoDesc_ADDR			4632
#define D_DCFifoDesc_ADDR_END			4655
#define D_DCFifoDesc_sizeof			24

#define D_DCFifo_ADDR				4656
#define D_DCFifo_ADDR_END			4847
#define D_DCFifo_sizeof			192

#define D_IODescr_ADDR				4848
#define D_IODescr_ADDR_END			5227
#define D_IODescr_sizeof			380

#define D_IODescrList_ADDR			5228
#define D_IODescrList_ADDR_END			5275
#define D_IODescrList_sizeof			48

#define D_PingPongDesc_ADDR			5276
#define D_PingPongDesc_ADDR_END		5323
#define D_PingPongDesc_sizeof			48

#define D_IRQMask_ADDR				5324
#define D_IRQMask_ADDR_END			5355
#define D_IRQMask_sizeof			32

#define D_DebugAbe2hal_ADDR			5356
#define D_DebugAbe2hal_ADDR_END		5483
#define D_DebugAbe2hal_sizeof			128

#define D_Debug_hal2abe_ADDR			5484
#define D_Debug_hal2abe_ADDR_END		5611
#define D_Debug_hal2abe_sizeof			128

#define D_Cmd2PhenixFifoDesc_ADDR		5612
#define D_Cmd2PhenixFifoDesc_ADDR_END		5623
#define D_Cmd2PhenixFifoDesc_sizeof		12

#define D_ctrlPortFifo_ADDR			5632
#define D_ctrlPortFifo_ADDR_END		5647
#define D_ctrlPortFifo_sizeof			16

#define D_StatusFromPhoenixFifoDesc_ADDR	5648
#define D_StatusFromPhoenixFifoDesc_ADDR_END	5659
#define D_StatusFromPhoenixFifoDesc_sizeof	12

#define D_Cmd2PhoenixFifo_ADDR			5660
#define D_Cmd2PhoenixFifo_ADDR_END		5691
#define D_Cmd2PhoenixFifo_sizeof		32

#define D_StatusFromPhoenixFifo_ADDR		5692
#define D_StatusFromPhoenixFifo_ADDR_END	5723
#define D_StatusFromPhoenixFifo_sizeof		32

#define D_Data_LSB_2_ADDR			5724
#define D_Data_LSB_2_ADDR_END			5727
#define D_Data_LSB_2_sizeof			4

#define D_Ref0_ADDR				5728
#define D_Ref0_ADDR_END			5737
#define D_Ref0_sizeof				10

#define D_VIB_DL_ADDR				5740
#define D_VIB_DL_ADDR_END			5931
#define D_VIB_DL_sizeof			192

#define D_DebugRegister_ADDR			5932
#define D_DebugRegister_ADDR_END		6071
#define D_DebugRegister_sizeof			140

#define D_Gcount_ADDR				6072
#define D_Gcount_ADDR_END			6073
#define D_Gcount_sizeof			2

#define D_VX_UL_ADDR				6144
#define D_VX_UL_ADDR_END			6239
#define D_VX_UL_sizeof				96

#define D_VX_DL_ADDR				6272
#define D_VX_DL_ADDR_END			6367
#define D_VX_DL_sizeof				96

#define D_DCcounter_ADDR			6368
#define D_DCcounter_ADDR_END			6371
#define D_DCcounter_sizeof			4

#define D_DCsum_ADDR				6372
#define D_DCsum_ADDR_END			6379
#define D_DCsum_sizeof				8

#define D_MM_UL2_FIFO_ADDR			6400
#define D_MM_UL2_FIFO_ADDR_END			6511
#define D_MM_UL2_FIFO_sizeof			112

#define D_TONES_DL_ADDR			6528
#define D_TONES_DL_ADDR_END			6639
#define D_TONES_DL_sizeof			112

#define D_aUplinkRouting_ADDR			6640
#define D_aUplinkRouting_ADDR_END		6655
#define D_aUplinkRouting_sizeof		16

#define D_VirtAudioLoop_ADDR			6656
#define D_VirtAudioLoop_ADDR_END		6659
#define D_VirtAudioLoop_sizeof			4

#define D_VIBRA_DL_ADDR			6784
#define D_VIBRA_DL_ADDR_END			6839
#define D_VIBRA_DL_sizeof			56

#define D_AsrcVars_DL_VX_ADDR			6840
#define D_AsrcVars_DL_VX_ADDR_END		6871
#define D_AsrcVars_DL_VX_sizeof		32

#define D_CoefAddresses_DL_VX_ADDR		6872
#define D_CoefAddresses_DL_VX_ADDR_END		6903
#define D_CoefAddresses_DL_VX_sizeof		32

#define D_AsrcVars_UL_VX_ADDR			6904
#define D_AsrcVars_UL_VX_ADDR_END		6935
#define D_AsrcVars_UL_VX_sizeof		32

#define D_CoefAddresses_UL_VX_ADDR		6936
#define D_CoefAddresses_UL_VX_ADDR_END		6967
#define D_CoefAddresses_UL_VX_sizeof		32

#define D_AsrcVars_DL_MM_ADDR			6968
#define D_AsrcVars_DL_MM_ADDR_END		6999
#define D_AsrcVars_DL_MM_sizeof		32

#define D_CoefAddresses_DL_MM_ADDR		7000
#define D_CoefAddresses_DL_MM_ADDR_END		7031
#define D_CoefAddresses_DL_MM_sizeof		32

#define D_APS_thresholds_ADDR			7032
#define D_APS_thresholds_ADDR_END		7039
#define D_APS_thresholds_sizeof		8

#define D_APS_IRQ_ADDR				7040
#define D_APS_IRQ_ADDR_END			7047
#define D_APS_IRQ_sizeof			8

#define D_TraceBufAdr_ADDR			7048
#define D_TraceBufAdr_ADDR_END			7049
#define D_TraceBufAdr_sizeof			2

#define D_TraceBufOffset_ADDR			7050
#define D_TraceBufOffset_ADDR_END		7051
#define D_TraceBufOffset_sizeof		2

#define D_TraceBufLength_ADDR			7052
#define D_TraceBufLength_ADDR_END		7053
#define D_TraceBufLength_sizeof		2

#define D_AsrcVars_ECHO_REF_ADDR		7056
#define D_AsrcVars_ECHO_REF_ADDR_END		7087
#define D_AsrcVars_ECHO_REF_sizeof		32

#define D_Pempty_ADDR				7088
#define D_Pempty_ADDR_END			7091
#define D_Pempty_sizeof			4

#define D_Trace_buffer_ADDR			8192
#define D_Trace_buffer_ADDR_END		10239
#define D_Trace_buffer_sizeof			2048

#define D_PING_ADDR				16384
#define D_PING_ADDR_END			40959
#define D_PING_sizeof				24576

#define D_PONG_ADDR				40960
#define D_PONG_ADDR_END			65535
#define D_PONG_sizeof				24576


#endif /* _ABEDM_ADDR_H_ */
