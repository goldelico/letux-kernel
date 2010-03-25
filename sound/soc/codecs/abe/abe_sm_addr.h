/*
 * ==========================================================================
 *               Texas Instruments OMAP(TM) Platform Firmware
 * (c) Copyright 2009, Texas Instruments Incorporated.  All Rights Reserved.
 *
 *  Use of this firmware is controlled by the terms and conditions found
 *  in the license agreement under which this firmware has been supplied.
 * ==========================================================================
 */

#ifndef _ABE_SM_ADDR_H_
#define _ABE_SM_ADDR_H_

#define init_SM_ADDR				0
#define init_SM_ADDR_END			284
#define init_SM_sizeof				285

#define S_Data0_ADDR				285
#define S_Data0_ADDR_END			285
#define S_Data0_sizeof				1

#define S_Temp_ADDR				286
#define S_Temp_ADDR_END				286
#define S_Temp_sizeof				1

#define S_PhoenixOffset_ADDR			287
#define S_PhoenixOffset_ADDR_END		287
#define S_PhoenixOffset_sizeof			1

#define S_GTarget1_ADDR				288
#define S_GTarget1_ADDR_END			294
#define S_GTarget1_sizeof			7

#define S_Gtarget_DL1_ADDR			295
#define S_Gtarget_DL1_ADDR_END			296
#define S_Gtarget_DL1_sizeof			2

#define S_Gtarget_DL2_ADDR			297
#define S_Gtarget_DL2_ADDR_END			298
#define S_Gtarget_DL2_sizeof			2

#define S_Gtarget_Echo_ADDR			299
#define S_Gtarget_Echo_ADDR_END			299
#define S_Gtarget_Echo_sizeof			1

#define S_Gtarget_SDT_ADDR			300
#define S_Gtarget_SDT_ADDR_END			300
#define S_Gtarget_SDT_sizeof			1

#define S_Gtarget_VxRec_ADDR			301
#define S_Gtarget_VxRec_ADDR_END		302
#define S_Gtarget_VxRec_sizeof			2

#define S_Gtarget_UL_ADDR			303
#define S_Gtarget_UL_ADDR_END			304
#define S_Gtarget_UL_sizeof			2

#define S_Gtarget_unused_ADDR			305
#define S_Gtarget_unused_ADDR_END		305
#define S_Gtarget_unused_sizeof			1

#define S_GCurrent_ADDR				306
#define S_GCurrent_ADDR_END			323
#define S_GCurrent_sizeof			18

#define S_GAIN_ONE_ADDR				324
#define S_GAIN_ONE_ADDR_END			324
#define S_GAIN_ONE_sizeof			1

#define S_Tones_ADDR				325
#define S_Tones_ADDR_END			336
#define S_Tones_sizeof				12

#define S_VX_DL_ADDR				337
#define S_VX_DL_ADDR_END			348
#define S_VX_DL_sizeof				12

#define S_MM_UL2_ADDR				349
#define S_MM_UL2_ADDR_END			360
#define S_MM_UL2_sizeof				12

#define S_MM_DL_ADDR				361
#define S_MM_DL_ADDR_END			372
#define S_MM_DL_sizeof				12

#define S_DL1_M_Out_ADDR			373
#define S_DL1_M_Out_ADDR_END			384
#define S_DL1_M_Out_sizeof			12

#define S_DL2_M_Out_ADDR			385
#define S_DL2_M_Out_ADDR_END			396
#define S_DL2_M_Out_sizeof			12

#define S_Echo_M_Out_ADDR			397
#define S_Echo_M_Out_ADDR_END			408
#define S_Echo_M_Out_sizeof			12

#define S_SDT_M_Out_ADDR			409
#define S_SDT_M_Out_ADDR_END			420
#define S_SDT_M_Out_sizeof			12

#define S_VX_UL_ADDR				421
#define S_VX_UL_ADDR_END			432
#define S_VX_UL_sizeof				12

#define S_VX_UL_M_ADDR				433
#define S_VX_UL_M_ADDR_END			444
#define S_VX_UL_M_sizeof			12

#define S_BT_DL_ADDR				445
#define S_BT_DL_ADDR_END			456
#define S_BT_DL_sizeof				12

#define S_BT_UL_ADDR				457
#define S_BT_UL_ADDR_END			468
#define S_BT_UL_sizeof				12

#define S_BT_DL_8k_ADDR				469
#define S_BT_DL_8k_ADDR_END			470
#define S_BT_DL_8k_sizeof			2

#define S_BT_DL_16k_ADDR			471
#define S_BT_DL_16k_ADDR_END			474
#define S_BT_DL_16k_sizeof			4

#define S_BT_UL_8k_ADDR				475
#define S_BT_UL_8k_ADDR_END			476
#define S_BT_UL_8k_sizeof			2

#define S_BT_UL_16k_ADDR			477
#define S_BT_UL_16k_ADDR_END			480
#define S_BT_UL_16k_sizeof			4

#define S_BT_UL_8_48_BP_data_ADDR		481
#define S_BT_UL_8_48_BP_data_ADDR_END		493
#define S_BT_UL_8_48_BP_data_sizeof		13

#define S_BT_UL_8_48_LP_data_ADDR		494
#define S_BT_UL_8_48_LP_data_ADDR_END		506
#define S_BT_UL_8_48_LP_data_sizeof		13

#define S_BT_UL_16_48_HP_data_ADDR		507
#define S_BT_UL_16_48_HP_data_ADDR_END		513
#define S_BT_UL_16_48_HP_data_sizeof		7

#define S_BT_UL_16_48_LP_data_ADDR		514
#define S_BT_UL_16_48_LP_data_ADDR_END		526
#define S_BT_UL_16_48_LP_data_sizeof		13

#define S_BT_DL_48_8_BP_data_ADDR		527
#define S_BT_DL_48_8_BP_data_ADDR_END		539
#define S_BT_DL_48_8_BP_data_sizeof		13

#define S_BT_DL_48_8_LP_data_ADDR		540
#define S_BT_DL_48_8_LP_data_ADDR_END		552
#define S_BT_DL_48_8_LP_data_sizeof		13

#define S_BT_DL_48_16_HP_data_ADDR		553
#define S_BT_DL_48_16_HP_data_ADDR_END		559
#define S_BT_DL_48_16_HP_data_sizeof		7

#define S_BT_DL_48_16_LP_data_ADDR		560
#define S_BT_DL_48_16_LP_data_ADDR_END		572
#define S_BT_DL_48_16_LP_data_sizeof		13

#define S_SDT_F_ADDR				573
#define S_SDT_F_ADDR_END			584
#define S_SDT_F_sizeof				12

#define S_SDT_F_data_ADDR			585
#define S_SDT_F_data_ADDR_END			593
#define S_SDT_F_data_sizeof			9

#define S_MM_DL_OSR_ADDR			594
#define S_MM_DL_OSR_ADDR_END			617
#define S_MM_DL_OSR_sizeof			24

#define S_24_zeros_ADDR				618
#define S_24_zeros_ADDR_END			641
#define S_24_zeros_sizeof			24

#define S_DMIC1_ADDR				642
#define S_DMIC1_ADDR_END			653
#define S_DMIC1_sizeof				12

#define S_DMIC2_ADDR				654
#define S_DMIC2_ADDR_END			665
#define S_DMIC2_sizeof				12

#define S_DMIC3_ADDR				666
#define S_DMIC3_ADDR_END			677
#define S_DMIC3_sizeof				12

#define S_AMIC_ADDR				678
#define S_AMIC_ADDR_END				689
#define S_AMIC_sizeof				12

#define S_EANC_FBK_in_ADDR			690
#define S_EANC_FBK_in_ADDR_END			713
#define S_EANC_FBK_in_sizeof			24

#define S_EANC_FBK_out_ADDR			714
#define S_EANC_FBK_out_ADDR_END			725
#define S_EANC_FBK_out_sizeof			12

#define S_DMIC1_L_ADDR				726
#define S_DMIC1_L_ADDR_END			737
#define S_DMIC1_L_sizeof			12

#define S_DMIC1_R_ADDR				738
#define S_DMIC1_R_ADDR_END			749
#define S_DMIC1_R_sizeof			12

#define S_DMIC2_L_ADDR				750
#define S_DMIC2_L_ADDR_END			761
#define S_DMIC2_L_sizeof			12

#define S_DMIC2_R_ADDR				762
#define S_DMIC2_R_ADDR_END			773
#define S_DMIC2_R_sizeof			12

#define S_DMIC3_L_ADDR				774
#define S_DMIC3_L_ADDR_END			785
#define S_DMIC3_L_sizeof			12

#define S_DMIC3_R_ADDR				786
#define S_DMIC3_R_ADDR_END			797
#define S_DMIC3_R_sizeof			12

#define S_BT_UL_L_ADDR				798
#define S_BT_UL_L_ADDR_END			809
#define S_BT_UL_L_sizeof			12

#define S_BT_UL_R_ADDR				810
#define S_BT_UL_R_ADDR_END			821
#define S_BT_UL_R_sizeof			12

#define S_AMIC_L_ADDR				822
#define S_AMIC_L_ADDR_END			833
#define S_AMIC_L_sizeof				12

#define S_AMIC_R_ADDR				834
#define S_AMIC_R_ADDR_END			845
#define S_AMIC_R_sizeof				12

#define S_EANC_FBK_L_ADDR			846
#define S_EANC_FBK_L_ADDR_END			857
#define S_EANC_FBK_L_sizeof			12

#define S_EANC_FBK_R_ADDR			858
#define S_EANC_FBK_R_ADDR_END			869
#define S_EANC_FBK_R_sizeof			12

#define S_EchoRef_L_ADDR			870
#define S_EchoRef_L_ADDR_END			881
#define S_EchoRef_L_sizeof			12

#define S_EchoRef_R_ADDR			882
#define S_EchoRef_R_ADDR_END			893
#define S_EchoRef_R_sizeof			12

#define S_MM_DL_L_ADDR				894
#define S_MM_DL_L_ADDR_END			905
#define S_MM_DL_L_sizeof			12

#define S_MM_DL_R_ADDR				906
#define S_MM_DL_R_ADDR_END			917
#define S_MM_DL_R_sizeof			12

#define S_MM_UL_ADDR				918
#define S_MM_UL_ADDR_END			1041
#define S_MM_UL_sizeof				124

#define S_AMIC_96k_ADDR				1042
#define S_AMIC_96k_ADDR_END			1065
#define S_AMIC_96k_sizeof			24

#define S_DMIC0_96k_ADDR			1066
#define S_DMIC0_96k_ADDR_END			1089
#define S_DMIC0_96k_sizeof			24

#define S_DMIC1_96k_ADDR			1090
#define S_DMIC1_96k_ADDR_END			1113
#define S_DMIC1_96k_sizeof			24

#define S_DMIC2_96k_ADDR			1114
#define S_DMIC2_96k_ADDR_END			1137
#define S_DMIC2_96k_sizeof			24

#define S_UL_VX_UL_48_8K_ADDR			1138
#define S_UL_VX_UL_48_8K_ADDR_END		1149
#define S_UL_VX_UL_48_8K_sizeof			12

#define S_UL_VX_UL_48_16K_ADDR			1150
#define S_UL_VX_UL_48_16K_ADDR_END		1161
#define S_UL_VX_UL_48_16K_sizeof		12

#define S_UL_MIC_48K_ADDR			1162
#define S_UL_MIC_48K_ADDR_END			1173
#define S_UL_MIC_48K_sizeof			12

#define S_Voice_8k_UL_ADDR			1174
#define S_Voice_8k_UL_ADDR_END			1176
#define S_Voice_8k_UL_sizeof			3

#define S_Voice_8k_DL_ADDR			1177
#define S_Voice_8k_DL_ADDR_END			1178
#define S_Voice_8k_DL_sizeof			2

#define S_McPDM_Out1_ADDR			1179
#define S_McPDM_Out1_ADDR_END			1202
#define S_McPDM_Out1_sizeof			24

#define S_McPDM_Out2_ADDR			1203
#define S_McPDM_Out2_ADDR_END			1226
#define S_McPDM_Out2_sizeof			24

#define S_McPDM_Out3_ADDR			1227
#define S_McPDM_Out3_ADDR_END			1250
#define S_McPDM_Out3_sizeof			24

#define S_Voice_16k_UL_ADDR			1251
#define S_Voice_16k_UL_ADDR_END			1255
#define S_Voice_16k_UL_sizeof			5

#define S_Voice_16k_DL_ADDR			1256
#define S_Voice_16k_DL_ADDR_END			1259
#define S_Voice_16k_DL_sizeof			4

#define S_XinASRC_DL_VX_ADDR			1260
#define S_XinASRC_DL_VX_ADDR_END		1299
#define S_XinASRC_DL_VX_sizeof			40

#define S_XinASRC_UL_VX_ADDR			1300
#define S_XinASRC_UL_VX_ADDR_END		1339
#define S_XinASRC_UL_VX_sizeof			40

#define S_XinASRC_DL_MM_ADDR			1340
#define S_XinASRC_DL_MM_ADDR_END		1379
#define S_XinASRC_DL_MM_sizeof			40

#define S_VX_REC_ADDR				1380
#define S_VX_REC_ADDR_END			1391
#define S_VX_REC_sizeof				12

#define S_VX_REC_L_ADDR				1392
#define S_VX_REC_L_ADDR_END			1403
#define S_VX_REC_L_sizeof			12

#define S_VX_REC_R_ADDR				1404
#define S_VX_REC_R_ADDR_END			1415
#define S_VX_REC_R_sizeof			12

#define S_DL2_M_L_ADDR				1416
#define S_DL2_M_L_ADDR_END			1427
#define S_DL2_M_L_sizeof			12

#define S_DL2_M_R_ADDR				1428
#define S_DL2_M_R_ADDR_END			1439
#define S_DL2_M_R_sizeof			12

#define S_DL2_M_LR_EQ_data_ADDR			1440
#define S_DL2_M_LR_EQ_data_ADDR_END		1464
#define S_DL2_M_LR_EQ_data_sizeof		25

#define S_DL1_M_EQ_data_ADDR			1465
#define S_DL1_M_EQ_data_ADDR_END		1489
#define S_DL1_M_EQ_data_sizeof			25

#define S_VX_DL_8_48_BP_data_ADDR		1490
#define S_VX_DL_8_48_BP_data_ADDR_END		1502
#define S_VX_DL_8_48_BP_data_sizeof		13

#define S_VX_DL_8_48_LP_data_ADDR		1503
#define S_VX_DL_8_48_LP_data_ADDR_END		1515
#define S_VX_DL_8_48_LP_data_sizeof		13

#define S_EARP_48_96_LP_data_ADDR		1516
#define S_EARP_48_96_LP_data_ADDR_END		1530
#define S_EARP_48_96_LP_data_sizeof		15

#define S_IHF_48_96_LP_data_ADDR		1531
#define S_IHF_48_96_LP_data_ADDR_END		1545
#define S_IHF_48_96_LP_data_sizeof		15

#define S_VX_DL_16_48_HP_data_ADDR		1546
#define S_VX_DL_16_48_HP_data_ADDR_END		1552
#define S_VX_DL_16_48_HP_data_sizeof		7

#define S_VX_DL_16_48_LP_data_ADDR		1553
#define S_VX_DL_16_48_LP_data_ADDR_END		1565
#define S_VX_DL_16_48_LP_data_sizeof		13

#define S_VX_UL_48_8_BP_data_ADDR		1566
#define S_VX_UL_48_8_BP_data_ADDR_END		1578
#define S_VX_UL_48_8_BP_data_sizeof		13

#define S_VX_UL_48_8_LP_data_ADDR		1579
#define S_VX_UL_48_8_LP_data_ADDR_END		1591
#define S_VX_UL_48_8_LP_data_sizeof		13

#define S_VX_UL_8_TEMP_ADDR			1592
#define S_VX_UL_8_TEMP_ADDR_END			1593
#define S_VX_UL_8_TEMP_sizeof			2

#define S_VX_UL_48_16_HP_data_ADDR		1594
#define S_VX_UL_48_16_HP_data_ADDR_END		1600
#define S_VX_UL_48_16_HP_data_sizeof		7

#define S_VX_UL_48_16_LP_data_ADDR		1601
#define S_VX_UL_48_16_LP_data_ADDR_END		1613
#define S_VX_UL_48_16_LP_data_sizeof		13

#define S_VX_UL_16_TEMP_ADDR			1614
#define S_VX_UL_16_TEMP_ADDR_END		1617
#define S_VX_UL_16_TEMP_sizeof			4

#define S_EANC_IIR_data_ADDR			1618
#define S_EANC_IIR_data_ADDR_END		1634
#define S_EANC_IIR_data_sizeof			17

#define S_EANC_SignalTemp_ADDR			1635
#define S_EANC_SignalTemp_ADDR_END		1655
#define S_EANC_SignalTemp_sizeof		21

#define S_EANC_Input_ADDR			1656
#define S_EANC_Input_ADDR_END			1656
#define S_EANC_Input_sizeof			1

#define S_EANC_Output_ADDR			1657
#define S_EANC_Output_ADDR_END			1657
#define S_EANC_Output_sizeof			1

#define S_APS_IIRmem1_ADDR			1658
#define S_APS_IIRmem1_ADDR_END			1666
#define S_APS_IIRmem1_sizeof			9

#define S_APS_M_IIRmem2_ADDR			1667
#define S_APS_M_IIRmem2_ADDR_END		1669
#define S_APS_M_IIRmem2_sizeof			3

#define S_APS_C_IIRmem2_ADDR			1670
#define S_APS_C_IIRmem2_ADDR_END		1672
#define S_APS_C_IIRmem2_sizeof			3

#define S_APS_DL1_OutSamples_ADDR		1673
#define S_APS_DL1_OutSamples_ADDR_END		1674
#define S_APS_DL1_OutSamples_sizeof		2

#define S_APS_DL1_COIL_OutSamples_ADDR		1675
#define S_APS_DL1_COIL_OutSamples_ADDR_END	1676
#define S_APS_DL1_COIL_OutSamples_sizeof	2

#define S_APS_DL2_L_OutSamples_ADDR		1677
#define S_APS_DL2_L_OutSamples_ADDR_END		1678
#define S_APS_DL2_L_OutSamples_sizeof		2

#define S_APS_DL2_L_COIL_OutSamples_ADDR	1679
#define S_APS_DL2_L_COIL_OutSamples_ADDR_END	1680
#define S_APS_DL2_L_COIL_OutSamples_sizeof	2

#define S_APS_DL2_R_OutSamples_ADDR		1681
#define S_APS_DL2_R_OutSamples_ADDR_END		1682
#define S_APS_DL2_R_OutSamples_sizeof		2

#define S_APS_DL2_R_COIL_OutSamples_ADDR	1683
#define S_APS_DL2_R_COIL_OutSamples_ADDR_END	1684
#define S_APS_DL2_R_COIL_OutSamples_sizeof	2

#define S_XinASRC_ECHO_REF_ADDR			1685
#define S_XinASRC_ECHO_REF_ADDR_END		1724
#define S_XinASRC_ECHO_REF_sizeof		40

#define S_ECHO_REF_16K_ADDR			1725
#define S_ECHO_REF_16K_ADDR_END			1729
#define S_ECHO_REF_16K_sizeof			5

#define S_ECHO_REF_8K_ADDR			1730
#define S_ECHO_REF_8K_ADDR_END			1732
#define S_ECHO_REF_8K_sizeof			3

#define S_DL1_ADDR				1733
#define S_DL1_ADDR_END				1744
#define S_DL1_sizeof				12

#define S_APS_DL2_L_IIRmem1_ADDR		1745
#define S_APS_DL2_L_IIRmem1_ADDR_END		1753
#define S_APS_DL2_L_IIRmem1_sizeof		9

#define S_APS_DL2_R_IIRmem1_ADDR		1754
#define S_APS_DL2_R_IIRmem1_ADDR_END		1762
#define S_APS_DL2_R_IIRmem1_sizeof		9

#define S_APS_DL2_L_M_IIRmem2_ADDR		1763
#define S_APS_DL2_L_M_IIRmem2_ADDR_END		1765
#define S_APS_DL2_L_M_IIRmem2_sizeof		3

#define S_APS_DL2_R_M_IIRmem2_ADDR		1766
#define S_APS_DL2_R_M_IIRmem2_ADDR_END		1768
#define S_APS_DL2_R_M_IIRmem2_sizeof		3

#define S_APS_DL2_L_C_IIRmem2_ADDR		1769
#define S_APS_DL2_L_C_IIRmem2_ADDR_END		1771
#define S_APS_DL2_L_C_IIRmem2_sizeof		3

#define S_APS_DL2_R_C_IIRmem2_ADDR		1772
#define S_APS_DL2_R_C_IIRmem2_ADDR_END		1774
#define S_APS_DL2_R_C_IIRmem2_sizeof		3

#define S_DL1_APS_ADDR				1775
#define S_DL1_APS_ADDR_END			1786
#define S_DL1_APS_sizeof			12

#define S_DL2_L_APS_ADDR			1787
#define S_DL2_L_APS_ADDR_END			1798
#define S_DL2_L_APS_sizeof			12

#define S_DL2_R_APS_ADDR			1799
#define S_DL2_R_APS_ADDR_END			1810
#define S_DL2_R_APS_sizeof			12

#define S_ECHO_REF_48_8_BP_data_ADDR		1811
#define S_ECHO_REF_48_8_BP_data_ADDR_END	1823
#define S_ECHO_REF_48_8_BP_data_sizeof		13

#define S_ECHO_REF_48_8_LP_data_ADDR		1824
#define S_ECHO_REF_48_8_LP_data_ADDR_END	1836
#define S_ECHO_REF_48_8_LP_data_sizeof		13

#define S_ECHO_REF_48_16_HP_data_ADDR		1837
#define S_ECHO_REF_48_16_HP_data_ADDR_END	1843
#define S_ECHO_REF_48_16_HP_data_sizeof		7

#define S_ECHO_REF_48_16_LP_data_ADDR		1844
#define S_ECHO_REF_48_16_LP_data_ADDR_END	1856
#define S_ECHO_REF_48_16_LP_data_sizeof		13

#define S_APS_DL1_EQ_data_ADDR			1857
#define S_APS_DL1_EQ_data_ADDR_END		1865
#define S_APS_DL1_EQ_data_sizeof		9

#define S_APS_DL2_EQ_data_ADDR			1866
#define S_APS_DL2_EQ_data_ADDR_END		1874
#define S_APS_DL2_EQ_data_sizeof		9

#define S_DC_DCvalue_ADDR			1875
#define S_DC_DCvalue_ADDR_END			1875
#define S_DC_DCvalue_sizeof			1

#define S_VIBRA_ADDR				1876
#define S_VIBRA_ADDR_END			1881
#define S_VIBRA_sizeof				6

#define S_Vibra2_in_ADDR			1882
#define S_Vibra2_in_ADDR_END			1887
#define S_Vibra2_in_sizeof			6

#define S_Vibra2_addr_ADDR			1888
#define S_Vibra2_addr_ADDR_END			1888
#define S_Vibra2_addr_sizeof			1

#define S_VibraCtrl_forRightSM_ADDR		1889
#define S_VibraCtrl_forRightSM_ADDR_END		1912
#define S_VibraCtrl_forRightSM_sizeof		24

#define S_Rnoise_mem_ADDR			1913
#define S_Rnoise_mem_ADDR_END			1913
#define S_Rnoise_mem_sizeof			1

#define S_Ctrl_ADDR				1914
#define S_Ctrl_ADDR_END				1931
#define S_Ctrl_sizeof				18

#define S_Vibra1_in_ADDR			1932
#define S_Vibra1_in_ADDR_END			1937
#define S_Vibra1_in_sizeof			6

#define S_Vibra1_temp_ADDR			1938
#define S_Vibra1_temp_ADDR_END			1961
#define S_Vibra1_temp_sizeof			24

#define S_VibraCtrl_forLeftSM_ADDR		1962
#define S_VibraCtrl_forLeftSM_ADDR_END		1985
#define S_VibraCtrl_forLeftSM_sizeof		24

#define S_Vibra1_mem_ADDR			1986
#define S_Vibra1_mem_ADDR_END			1996
#define S_Vibra1_mem_sizeof			11

#define S_VibraCtrl_Stereo_ADDR			1997
#define S_VibraCtrl_Stereo_ADDR_END		2020
#define S_VibraCtrl_Stereo_sizeof		24

#define S_AMIC_96_48_data_ADDR			2021
#define S_AMIC_96_48_data_ADDR_END		2035
#define S_AMIC_96_48_data_sizeof		15

#define S_DMIC0_96_48_data_ADDR			2036
#define S_DMIC0_96_48_data_ADDR_END		2050
#define S_DMIC0_96_48_data_sizeof		15

#define S_DMIC1_96_48_data_ADDR			2051
#define S_DMIC1_96_48_data_ADDR_END		2065
#define S_DMIC1_96_48_data_sizeof		15

#define S_DMIC2_96_48_data_ADDR			2066
#define S_DMIC2_96_48_data_ADDR_END		2080
#define S_DMIC2_96_48_data_sizeof		15

#define S_EANC_FBK_96_48_data_ADDR		2081
#define S_EANC_FBK_96_48_data_ADDR_END		2095
#define S_EANC_FBK_96_48_data_sizeof		15

#define S_DBG_8K_PATTERN_ADDR			2096
#define S_DBG_8K_PATTERN_ADDR_END		2103
#define S_DBG_8K_PATTERN_sizeof			8

#define S_DBG_16K_PATTERN_ADDR			2104
#define S_DBG_16K_PATTERN_ADDR_END		2119
#define S_DBG_16K_PATTERN_sizeof		16

#define S_DBG_48K_PATTERN_ADDR			2120
#define S_DBG_48K_PATTERN_ADDR_END		2143
#define S_DBG_48K_PATTERN_sizeof		24

#define S_DBG_MCPDM_PATTERN_ADDR		2144
#define S_DBG_MCPDM_PATTERN_ADDR_END		2215
#define S_DBG_MCPDM_PATTERN_sizeof		72

#endif /* _ABESM_ADDR_H_ */
