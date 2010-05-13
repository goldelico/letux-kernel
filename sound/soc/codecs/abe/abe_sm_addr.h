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

#define init_SM_ADDR						0
#define init_SM_ADDR_END					284
#define init_SM_sizeof						285

#define S_Data0_ADDR						285
#define S_Data0_ADDR_END					285
#define S_Data0_sizeof						1

#define S_Temp_ADDR						286
#define S_Temp_ADDR_END						286
#define S_Temp_sizeof						1

#define S_PhoenixOffset_ADDR					287
#define S_PhoenixOffset_ADDR_END				287
#define S_PhoenixOffset_sizeof					1

#define S_GTarget1_ADDR						288
#define S_GTarget1_ADDR_END					294
#define S_GTarget1_sizeof					7

#define S_Gtarget_DL1_ADDR					295
#define S_Gtarget_DL1_ADDR_END					296
#define S_Gtarget_DL1_sizeof					2

#define S_Gtarget_DL2_ADDR					297
#define S_Gtarget_DL2_ADDR_END					298
#define S_Gtarget_DL2_sizeof					2

#define S_Gtarget_Echo_ADDR					299
#define S_Gtarget_Echo_ADDR_END					299
#define S_Gtarget_Echo_sizeof					1

#define S_Gtarget_SDT_ADDR					300
#define S_Gtarget_SDT_ADDR_END					300
#define S_Gtarget_SDT_sizeof					1

#define S_Gtarget_VxRec_ADDR					301
#define S_Gtarget_VxRec_ADDR_END				302
#define S_Gtarget_VxRec_sizeof					2

#define S_Gtarget_UL_ADDR					303
#define S_Gtarget_UL_ADDR_END					304
#define S_Gtarget_UL_sizeof					2

#define S_Gtarget_unused_ADDR					305
#define S_Gtarget_unused_ADDR_END				305
#define S_Gtarget_unused_sizeof					1

#define S_GCurrent_ADDR						306
#define S_GCurrent_ADDR_END					323
#define S_GCurrent_sizeof					18

#define S_GAIN_ONE_ADDR						324
#define S_GAIN_ONE_ADDR_END					324
#define S_GAIN_ONE_sizeof					1

#define S_Tones_ADDR						325
#define S_Tones_ADDR_END					336
#define S_Tones_sizeof						12

#define S_VX_DL_ADDR						337
#define S_VX_DL_ADDR_END					348
#define S_VX_DL_sizeof						12

#define S_MM_UL2_ADDR						349
#define S_MM_UL2_ADDR_END					360
#define S_MM_UL2_sizeof						12

#define S_MM_DL_ADDR						361
#define S_MM_DL_ADDR_END					372
#define S_MM_DL_sizeof						12

#define S_DL1_M_Out_ADDR					373
#define S_DL1_M_Out_ADDR_END					384
#define S_DL1_M_Out_sizeof					12

#define S_DL2_M_Out_ADDR					385
#define S_DL2_M_Out_ADDR_END					396
#define S_DL2_M_Out_sizeof					12

#define S_Echo_M_Out_ADDR					397
#define S_Echo_M_Out_ADDR_END					408
#define S_Echo_M_Out_sizeof					12

#define S_SDT_M_Out_ADDR					409
#define S_SDT_M_Out_ADDR_END					420
#define S_SDT_M_Out_sizeof					12

#define S_VX_UL_ADDR						421
#define S_VX_UL_ADDR_END					432
#define S_VX_UL_sizeof						12

#define S_VX_UL_M_ADDR						433
#define S_VX_UL_M_ADDR_END					444
#define S_VX_UL_M_sizeof					12

#define S_BT_DL_ADDR						445
#define S_BT_DL_ADDR_END					456
#define S_BT_DL_sizeof						12

#define S_BT_UL_ADDR						457
#define S_BT_UL_ADDR_END					468
#define S_BT_UL_sizeof						12

#define S_BT_DL_8k_ADDR						469
#define S_BT_DL_8k_ADDR_END					470
#define S_BT_DL_8k_sizeof					2

#define S_BT_DL_16k_ADDR					471
#define S_BT_DL_16k_ADDR_END					474
#define S_BT_DL_16k_sizeof					4

#define S_BT_UL_8k_ADDR						475
#define S_BT_UL_8k_ADDR_END					476
#define S_BT_UL_8k_sizeof					2

#define S_BT_UL_16k_ADDR					477
#define S_BT_UL_16k_ADDR_END					480
#define S_BT_UL_16k_sizeof					4

#define S_BT_UL_8_48_BP_data_ADDR				481
#define S_BT_UL_8_48_BP_data_ADDR_END				493
#define S_BT_UL_8_48_BP_data_sizeof				13

#define S_BT_UL_8_48_LP_data_ADDR				494
#define S_BT_UL_8_48_LP_data_ADDR_END				506
#define S_BT_UL_8_48_LP_data_sizeof				13

#define S_BT_UL_16_48_HP_data_ADDR				507
#define S_BT_UL_16_48_HP_data_ADDR_END				513
#define S_BT_UL_16_48_HP_data_sizeof				7

#define S_BT_UL_16_48_LP_data_ADDR				514
#define S_BT_UL_16_48_LP_data_ADDR_END				526
#define S_BT_UL_16_48_LP_data_sizeof				13

#define S_BT_DL_48_8_BP_data_ADDR				527
#define S_BT_DL_48_8_BP_data_ADDR_END				539
#define S_BT_DL_48_8_BP_data_sizeof				13

#define S_BT_DL_48_8_LP_data_ADDR				540
#define S_BT_DL_48_8_LP_data_ADDR_END				552
#define S_BT_DL_48_8_LP_data_sizeof				13

#define S_BT_DL_48_16_HP_data_ADDR				553
#define S_BT_DL_48_16_HP_data_ADDR_END				559
#define S_BT_DL_48_16_HP_data_sizeof				7

#define S_BT_DL_48_16_LP_data_ADDR				560
#define S_BT_DL_48_16_LP_data_ADDR_END				572
#define S_BT_DL_48_16_LP_data_sizeof				13

#define S_SDT_F_ADDR						573
#define S_SDT_F_ADDR_END					584
#define S_SDT_F_sizeof						12

#define S_SDT_F_data_ADDR					585
#define S_SDT_F_data_ADDR_END					593
#define S_SDT_F_data_sizeof					9

#define S_MM_DL_OSR_ADDR					594
#define S_MM_DL_OSR_ADDR_END					617
#define S_MM_DL_OSR_sizeof					24

#define S_24_zeros_ADDR						618
#define S_24_zeros_ADDR_END					641
#define S_24_zeros_sizeof					24

#define S_DMIC1_ADDR						642
#define S_DMIC1_ADDR_END					653
#define S_DMIC1_sizeof						12

#define S_DMIC2_ADDR						654
#define S_DMIC2_ADDR_END					665
#define S_DMIC2_sizeof						12

#define S_DMIC3_ADDR						666
#define S_DMIC3_ADDR_END					677
#define S_DMIC3_sizeof						12

#define S_AMIC_ADDR						678
#define S_AMIC_ADDR_END						689
#define S_AMIC_sizeof						12

#define S_EANC_FBK_in_ADDR					690
#define S_EANC_FBK_in_ADDR_END					713
#define S_EANC_FBK_in_sizeof					24

#define S_EANC_FBK_out_ADDR					714
#define S_EANC_FBK_out_ADDR_END					725
#define S_EANC_FBK_out_sizeof					12

#define S_DMIC1_L_ADDR						726
#define S_DMIC1_L_ADDR_END					737
#define S_DMIC1_L_sizeof					12

#define S_DMIC1_R_ADDR						738
#define S_DMIC1_R_ADDR_END					749
#define S_DMIC1_R_sizeof					12

#define S_DMIC2_L_ADDR						750
#define S_DMIC2_L_ADDR_END					761
#define S_DMIC2_L_sizeof					12

#define S_DMIC2_R_ADDR						762
#define S_DMIC2_R_ADDR_END					773
#define S_DMIC2_R_sizeof					12

#define S_DMIC3_L_ADDR						774
#define S_DMIC3_L_ADDR_END					785
#define S_DMIC3_L_sizeof					12

#define S_DMIC3_R_ADDR						786
#define S_DMIC3_R_ADDR_END					797
#define S_DMIC3_R_sizeof					12

#define S_BT_UL_L_ADDR						798
#define S_BT_UL_L_ADDR_END					809
#define S_BT_UL_L_sizeof					12

#define S_BT_UL_R_ADDR						810
#define S_BT_UL_R_ADDR_END					821
#define S_BT_UL_R_sizeof					12

#define S_AMIC_L_ADDR						822
#define S_AMIC_L_ADDR_END					833
#define S_AMIC_L_sizeof						12

#define S_AMIC_R_ADDR						834
#define S_AMIC_R_ADDR_END					845
#define S_AMIC_R_sizeof						12

#define S_EANC_FBK_L_ADDR					846
#define S_EANC_FBK_L_ADDR_END					857
#define S_EANC_FBK_L_sizeof					12

#define S_EANC_FBK_R_ADDR					858
#define S_EANC_FBK_R_ADDR_END					869
#define S_EANC_FBK_R_sizeof					12

#define S_EchoRef_L_ADDR					870
#define S_EchoRef_L_ADDR_END					881
#define S_EchoRef_L_sizeof					12

#define S_EchoRef_R_ADDR					882
#define S_EchoRef_R_ADDR_END					893
#define S_EchoRef_R_sizeof					12

#define S_MM_DL_L_ADDR						894
#define S_MM_DL_L_ADDR_END					905
#define S_MM_DL_L_sizeof					12

#define S_MM_DL_R_ADDR						906
#define S_MM_DL_R_ADDR_END					917
#define S_MM_DL_R_sizeof					12

#define S_MM_UL_ADDR						918
#define S_MM_UL_ADDR_END					1037
#define S_MM_UL_sizeof						120

#define S_AMIC_96k_ADDR						1038
#define S_AMIC_96k_ADDR_END					1061
#define S_AMIC_96k_sizeof					24

#define S_DMIC0_96k_ADDR					1062
#define S_DMIC0_96k_ADDR_END					1085
#define S_DMIC0_96k_sizeof					24

#define S_DMIC1_96k_ADDR					1086
#define S_DMIC1_96k_ADDR_END					1109
#define S_DMIC1_96k_sizeof					24

#define S_DMIC2_96k_ADDR					1110
#define S_DMIC2_96k_ADDR_END					1133
#define S_DMIC2_96k_sizeof					24

#define S_UL_VX_UL_48_8K_ADDR					1134
#define S_UL_VX_UL_48_8K_ADDR_END				1145
#define S_UL_VX_UL_48_8K_sizeof					12

#define S_UL_VX_UL_48_16K_ADDR					1146
#define S_UL_VX_UL_48_16K_ADDR_END				1157
#define S_UL_VX_UL_48_16K_sizeof				12

#define S_UL_MIC_48K_ADDR					1158
#define S_UL_MIC_48K_ADDR_END					1169
#define S_UL_MIC_48K_sizeof					12

#define S_Voice_8k_UL_ADDR					1170
#define S_Voice_8k_UL_ADDR_END					1172
#define S_Voice_8k_UL_sizeof					3

#define S_Voice_8k_DL_ADDR					1173
#define S_Voice_8k_DL_ADDR_END					1174
#define S_Voice_8k_DL_sizeof					2

#define S_McPDM_Out1_ADDR					1175
#define S_McPDM_Out1_ADDR_END					1198
#define S_McPDM_Out1_sizeof					24

#define S_McPDM_Out2_ADDR					1199
#define S_McPDM_Out2_ADDR_END					1222
#define S_McPDM_Out2_sizeof					24

#define S_McPDM_Out3_ADDR					1223
#define S_McPDM_Out3_ADDR_END					1246
#define S_McPDM_Out3_sizeof					24

#define S_Voice_16k_UL_ADDR					1247
#define S_Voice_16k_UL_ADDR_END					1251
#define S_Voice_16k_UL_sizeof					5

#define S_Voice_16k_DL_ADDR					1252
#define S_Voice_16k_DL_ADDR_END					1255
#define S_Voice_16k_DL_sizeof					4

#define S_XinASRC_DL_VX_ADDR					1256
#define S_XinASRC_DL_VX_ADDR_END				1295
#define S_XinASRC_DL_VX_sizeof					40

#define S_XinASRC_UL_VX_ADDR					1296
#define S_XinASRC_UL_VX_ADDR_END				1335
#define S_XinASRC_UL_VX_sizeof					40

#define S_XinASRC_DL_MM_ADDR					1336
#define S_XinASRC_DL_MM_ADDR_END				1375
#define S_XinASRC_DL_MM_sizeof					40

#define S_VX_REC_ADDR						1376
#define S_VX_REC_ADDR_END					1387
#define S_VX_REC_sizeof						12

#define S_VX_REC_L_ADDR						1388
#define S_VX_REC_L_ADDR_END					1399
#define S_VX_REC_L_sizeof					12

#define S_VX_REC_R_ADDR						1400
#define S_VX_REC_R_ADDR_END					1411
#define S_VX_REC_R_sizeof					12

#define S_DL2_M_L_ADDR						1412
#define S_DL2_M_L_ADDR_END					1423
#define S_DL2_M_L_sizeof					12

#define S_DL2_M_R_ADDR						1424
#define S_DL2_M_R_ADDR_END					1435
#define S_DL2_M_R_sizeof					12

#define S_DL2_M_LR_EQ_data_ADDR					1436
#define S_DL2_M_LR_EQ_data_ADDR_END				1460
#define S_DL2_M_LR_EQ_data_sizeof				25

#define S_DL1_M_EQ_data_ADDR					1461
#define S_DL1_M_EQ_data_ADDR_END				1485
#define S_DL1_M_EQ_data_sizeof					25

#define S_VX_DL_8_48_BP_data_ADDR				1486
#define S_VX_DL_8_48_BP_data_ADDR_END				1498
#define S_VX_DL_8_48_BP_data_sizeof				13

#define S_VX_DL_8_48_LP_data_ADDR				1499
#define S_VX_DL_8_48_LP_data_ADDR_END				1511
#define S_VX_DL_8_48_LP_data_sizeof				13

#define S_EARP_48_96_LP_data_ADDR				1512
#define S_EARP_48_96_LP_data_ADDR_END				1526
#define S_EARP_48_96_LP_data_sizeof				15

#define S_IHF_48_96_LP_data_ADDR				1527
#define S_IHF_48_96_LP_data_ADDR_END				1541
#define S_IHF_48_96_LP_data_sizeof				15

#define S_VX_DL_16_48_HP_data_ADDR				1542
#define S_VX_DL_16_48_HP_data_ADDR_END				1548
#define S_VX_DL_16_48_HP_data_sizeof				7

#define S_VX_DL_16_48_LP_data_ADDR				1549
#define S_VX_DL_16_48_LP_data_ADDR_END				1561
#define S_VX_DL_16_48_LP_data_sizeof				13

#define S_VX_UL_48_8_BP_data_ADDR				1562
#define S_VX_UL_48_8_BP_data_ADDR_END				1574
#define S_VX_UL_48_8_BP_data_sizeof				13

#define S_VX_UL_48_8_LP_data_ADDR				1575
#define S_VX_UL_48_8_LP_data_ADDR_END				1587
#define S_VX_UL_48_8_LP_data_sizeof				13

#define S_VX_UL_8_TEMP_ADDR					1588
#define S_VX_UL_8_TEMP_ADDR_END					1589
#define S_VX_UL_8_TEMP_sizeof					2

#define S_VX_UL_48_16_HP_data_ADDR				1590
#define S_VX_UL_48_16_HP_data_ADDR_END				1596
#define S_VX_UL_48_16_HP_data_sizeof				7

#define S_VX_UL_48_16_LP_data_ADDR				1597
#define S_VX_UL_48_16_LP_data_ADDR_END				1609
#define S_VX_UL_48_16_LP_data_sizeof				13

#define S_VX_UL_16_TEMP_ADDR					1610
#define S_VX_UL_16_TEMP_ADDR_END				1613
#define S_VX_UL_16_TEMP_sizeof					4

#define S_EANC_IIR_data_ADDR					1614
#define S_EANC_IIR_data_ADDR_END				1630
#define S_EANC_IIR_data_sizeof					17

#define S_EANC_SignalTemp_ADDR					1631
#define S_EANC_SignalTemp_ADDR_END				1651
#define S_EANC_SignalTemp_sizeof				21

#define S_EANC_Input_ADDR					1652
#define S_EANC_Input_ADDR_END					1652
#define S_EANC_Input_sizeof					1

#define S_EANC_Output_ADDR					1653
#define S_EANC_Output_ADDR_END					1653
#define S_EANC_Output_sizeof					1

#define S_APS_IIRmem1_ADDR					1654
#define S_APS_IIRmem1_ADDR_END					1662
#define S_APS_IIRmem1_sizeof					9

#define S_APS_M_IIRmem2_ADDR					1663
#define S_APS_M_IIRmem2_ADDR_END				1665
#define S_APS_M_IIRmem2_sizeof					3

#define S_APS_C_IIRmem2_ADDR					1666
#define S_APS_C_IIRmem2_ADDR_END				1668
#define S_APS_C_IIRmem2_sizeof					3

#define S_APS_DL1_OutSamples_ADDR				1669
#define S_APS_DL1_OutSamples_ADDR_END				1670
#define S_APS_DL1_OutSamples_sizeof				2

#define S_APS_DL1_COIL_OutSamples_ADDR				1671
#define S_APS_DL1_COIL_OutSamples_ADDR_END			1672
#define S_APS_DL1_COIL_OutSamples_sizeof			2

#define S_APS_DL2_L_OutSamples_ADDR				1673
#define S_APS_DL2_L_OutSamples_ADDR_END				1674
#define S_APS_DL2_L_OutSamples_sizeof				2

#define S_APS_DL2_L_COIL_OutSamples_ADDR			1675
#define S_APS_DL2_L_COIL_OutSamples_ADDR_END			1676
#define S_APS_DL2_L_COIL_OutSamples_sizeof			2

#define S_APS_DL2_R_OutSamples_ADDR				1677
#define S_APS_DL2_R_OutSamples_ADDR_END				1678
#define S_APS_DL2_R_OutSamples_sizeof				2

#define S_APS_DL2_R_COIL_OutSamples_ADDR			1679
#define S_APS_DL2_R_COIL_OutSamples_ADDR_END			1680
#define S_APS_DL2_R_COIL_OutSamples_sizeof			2

#define S_XinASRC_ECHO_REF_ADDR					1681
#define S_XinASRC_ECHO_REF_ADDR_END				1720
#define S_XinASRC_ECHO_REF_sizeof				40

#define S_ECHO_REF_16K_ADDR					1721
#define S_ECHO_REF_16K_ADDR_END					1725
#define S_ECHO_REF_16K_sizeof					5

#define S_ECHO_REF_8K_ADDR					1726
#define S_ECHO_REF_8K_ADDR_END					1728
#define S_ECHO_REF_8K_sizeof					3

#define S_DL1_ADDR						1729
#define S_DL1_ADDR_END						1740
#define S_DL1_sizeof						12

#define S_APS_DL2_L_IIRmem1_ADDR				1741
#define S_APS_DL2_L_IIRmem1_ADDR_END				1749
#define S_APS_DL2_L_IIRmem1_sizeof				9

#define S_APS_DL2_R_IIRmem1_ADDR				1750
#define S_APS_DL2_R_IIRmem1_ADDR_END				1758
#define S_APS_DL2_R_IIRmem1_sizeof				9

#define S_APS_DL2_L_M_IIRmem2_ADDR				1759
#define S_APS_DL2_L_M_IIRmem2_ADDR_END				1761
#define S_APS_DL2_L_M_IIRmem2_sizeof				3

#define S_APS_DL2_R_M_IIRmem2_ADDR				1762
#define S_APS_DL2_R_M_IIRmem2_ADDR_END				1764
#define S_APS_DL2_R_M_IIRmem2_sizeof				3

#define S_APS_DL2_L_C_IIRmem2_ADDR				1765
#define S_APS_DL2_L_C_IIRmem2_ADDR_END				1767
#define S_APS_DL2_L_C_IIRmem2_sizeof				3

#define S_APS_DL2_R_C_IIRmem2_ADDR				1768
#define S_APS_DL2_R_C_IIRmem2_ADDR_END				1770
#define S_APS_DL2_R_C_IIRmem2_sizeof				3

#define S_DL1_APS_ADDR						1771
#define S_DL1_APS_ADDR_END					1782
#define S_DL1_APS_sizeof					12

#define S_DL2_L_APS_ADDR					1783
#define S_DL2_L_APS_ADDR_END					1794
#define S_DL2_L_APS_sizeof					12

#define S_DL2_R_APS_ADDR					1795
#define S_DL2_R_APS_ADDR_END					1806
#define S_DL2_R_APS_sizeof					12

#define S_ECHO_REF_48_8_BP_data_ADDR				1807
#define S_ECHO_REF_48_8_BP_data_ADDR_END			1819
#define S_ECHO_REF_48_8_BP_data_sizeof				13

#define S_ECHO_REF_48_8_LP_data_ADDR				1820
#define S_ECHO_REF_48_8_LP_data_ADDR_END			1832
#define S_ECHO_REF_48_8_LP_data_sizeof				13

#define S_ECHO_REF_48_16_HP_data_ADDR				1833
#define S_ECHO_REF_48_16_HP_data_ADDR_END			1839
#define S_ECHO_REF_48_16_HP_data_sizeof				7

#define S_ECHO_REF_48_16_LP_data_ADDR				1840
#define S_ECHO_REF_48_16_LP_data_ADDR_END			1852
#define S_ECHO_REF_48_16_LP_data_sizeof				13

#define S_APS_DL1_EQ_data_ADDR					1853
#define S_APS_DL1_EQ_data_ADDR_END				1861
#define S_APS_DL1_EQ_data_sizeof				9

#define S_APS_DL2_EQ_data_ADDR					1862
#define S_APS_DL2_EQ_data_ADDR_END				1870
#define S_APS_DL2_EQ_data_sizeof				9

#define S_DC_DCvalue_ADDR					1871
#define S_DC_DCvalue_ADDR_END					1871
#define S_DC_DCvalue_sizeof					1

#define S_VIBRA_ADDR						1872
#define S_VIBRA_ADDR_END					1877
#define S_VIBRA_sizeof						6

#define S_Vibra2_in_ADDR					1878
#define S_Vibra2_in_ADDR_END					1883
#define S_Vibra2_in_sizeof					6

#define S_Vibra2_addr_ADDR					1884
#define S_Vibra2_addr_ADDR_END					1884
#define S_Vibra2_addr_sizeof					1

#define S_VibraCtrl_forRightSM_ADDR				1885
#define S_VibraCtrl_forRightSM_ADDR_END				1908
#define S_VibraCtrl_forRightSM_sizeof				24

#define S_Rnoise_mem_ADDR					1909
#define S_Rnoise_mem_ADDR_END					1909
#define S_Rnoise_mem_sizeof					1

#define S_Ctrl_ADDR						1910
#define S_Ctrl_ADDR_END						1927
#define S_Ctrl_sizeof						18

#define S_Vibra1_in_ADDR					1928
#define S_Vibra1_in_ADDR_END					1933
#define S_Vibra1_in_sizeof					6

#define S_Vibra1_temp_ADDR					1934
#define S_Vibra1_temp_ADDR_END					1957
#define S_Vibra1_temp_sizeof					24

#define S_VibraCtrl_forLeftSM_ADDR				1958
#define S_VibraCtrl_forLeftSM_ADDR_END				1981
#define S_VibraCtrl_forLeftSM_sizeof				24

#define S_Vibra1_mem_ADDR					1982
#define S_Vibra1_mem_ADDR_END					1992
#define S_Vibra1_mem_sizeof					11

#define S_VibraCtrl_Stereo_ADDR					1993
#define S_VibraCtrl_Stereo_ADDR_END				2016
#define S_VibraCtrl_Stereo_sizeof				24

#define S_AMIC_96_48_data_ADDR					2017
#define S_AMIC_96_48_data_ADDR_END				2035
#define S_AMIC_96_48_data_sizeof				19

#define S_DMIC0_96_48_data_ADDR					2036
#define S_DMIC0_96_48_data_ADDR_END				2054
#define S_DMIC0_96_48_data_sizeof				19

#define S_DMIC1_96_48_data_ADDR					2055
#define S_DMIC1_96_48_data_ADDR_END				2073
#define S_DMIC1_96_48_data_sizeof				19

#define S_DMIC2_96_48_data_ADDR					2074
#define S_DMIC2_96_48_data_ADDR_END				2092
#define S_DMIC2_96_48_data_sizeof				19

#define S_EANC_FBK_96_48_data_ADDR				2093
#define S_EANC_FBK_96_48_data_ADDR_END				2111
#define S_EANC_FBK_96_48_data_sizeof				19

#define S_DBG_8K_PATTERN_ADDR					2112
#define S_DBG_8K_PATTERN_ADDR_END				2119
#define S_DBG_8K_PATTERN_sizeof					8

#define S_DBG_16K_PATTERN_ADDR					2120
#define S_DBG_16K_PATTERN_ADDR_END				2135
#define S_DBG_16K_PATTERN_sizeof				16

#define S_DBG_48K_PATTERN_ADDR					2136
#define S_DBG_48K_PATTERN_ADDR_END				2159
#define S_DBG_48K_PATTERN_sizeof				24

#define S_DBG_MCPDM_PATTERN_ADDR				2160
#define S_DBG_MCPDM_PATTERN_ADDR_END				2231
#define S_DBG_MCPDM_PATTERN_sizeof				72

#endif /* _ABESM_ADDR_H_ */
