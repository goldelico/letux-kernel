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

/* Samples memory sizeof */
#define abe_sm_sizeof(x)	(S_##x##_ADDR_END - S_##x##_ADDR + 1)

#define S_init_ADDR				0
#define S_init_ADDR_END				249

#define S_Data0_ADDR				250
#define S_Data0_ADDR_END			250

#define S_Temp_ADDR				251
#define S_Temp_ADDR_END				251

#define S_PhoenixOffset_ADDR			252
#define S_PhoenixOffset_ADDR_END		252

#define S_GTarget_ADDR				253
#define S_GTarget_ADDR_END			261

#define S_GCurrent_ADDR				262
#define S_GCurrent_ADDR_END			270

#define S_Tones_ADDR				271
#define S_Tones_ADDR_END			282

#define S_VX_DL_ADDR				283
#define S_VX_DL_ADDR_END			294

#define S_MM_UL2_ADDR				295
#define S_MM_UL2_ADDR_END			306

#define S_MM_DL_ADDR				307
#define S_MM_DL_ADDR_END			318

#define S_DL1_M_Out_ADDR			319
#define S_DL1_M_Out_ADDR_END			330

#define S_DL2_M_Out_ADDR			331
#define S_DL2_M_Out_ADDR_END			342

#define S_Echo_M_Out_ADDR			343
#define S_Echo_M_Out_ADDR_END			354

#define S_SDT_M_Out_ADDR			355
#define S_SDT_M_Out_ADDR_END			366

#define S_VX_UL_ADDR				367
#define S_VX_UL_ADDR_END			378

#define S_SDT_F_ADDR				379
#define S_SDT_F_ADDR_END			390

#define S_SDT_F_data_ADDR			391
#define S_SDT_F_data_ADDR_END			407

#define S_MM_DL_OSR_ADDR			408
#define S_MM_DL_OSR_ADDR_END			431

#define S_MM_96_F_ADDR				432
#define S_MM_96_F_ADDR_END			455

#define S_MM96_F_data_ADDR			456
#define S_MM96_F_data_ADDR_END			472

#define S_24_zeros_ADDR				473
#define S_24_zeros_ADDR_END			496

#define S_DMIC1_ADDR				497
#define S_DMIC1_ADDR_END			508

#define S_DMIC2_ADDR				509
#define S_DMIC2_ADDR_END			520

#define S_DMIC3_ADDR				521
#define S_DMIC3_ADDR_END			532

#define S_BT_UL_ADDR				533
#define S_BT_UL_ADDR_END			544

#define S_AMIC_ADDR				545
#define S_AMIC_ADDR_END				556

#define S_EANC_FBK_ADDR				557
#define S_EANC_FBK_ADDR_END			568

#define S_DMIC1_L_ADDR				569
#define S_DMIC1_L_ADDR_END			580

#define S_DMIC1_R_ADDR				581
#define S_DMIC1_R_ADDR_END			592

#define S_DMIC2_L_ADDR				593
#define S_DMIC2_L_ADDR_END			604

#define S_DMIC2_R_ADDR				605
#define S_DMIC2_R_ADDR_END			616

#define S_DMIC3_L_ADDR				617
#define S_DMIC3_L_ADDR_END			628

#define S_DMIC3_R_ADDR				629
#define S_DMIC3_R_ADDR_END			640

#define S_BT_L_ADDR				641
#define S_BT_L_ADDR_END				652

#define S_BT_R_ADDR				653
#define S_BT_R_ADDR_END				664

#define S_AMIC_L_ADDR				665
#define S_AMIC_L_ADDR_END			676

#define S_AMIC_R_ADDR				677
#define S_AMIC_R_ADDR_END			688

#define S_EANC_FBK_L_ADDR			689
#define S_EANC_FBK_L_ADDR_END			700

#define S_EANC_FBK_R_ADDR			701
#define S_EANC_FBK_R_ADDR_END			712

#define S_EchoRef_L_ADDR			713
#define S_EchoRef_L_ADDR_END			724

#define S_EchoRef_R_ADDR			725
#define S_EchoRef_R_ADDR_END			736

#define S_MM_DL_L_ADDR				737
#define S_MM_DL_L_ADDR_END			748

#define S_MM_DL_R_ADDR				749
#define S_MM_DL_R_ADDR_END			760

#define S_MM_UL_ADDR				761
#define S_MM_UL_ADDR_END			820

#define S_AMIC_96k_ADDR				821
#define S_AMIC_96k_ADDR_END			844

#define S_DMIC0_96k_ADDR			845
#define S_DMIC0_96k_ADDR_END			868

#define S_DMIC1_96k_ADDR			869
#define S_DMIC1_96k_ADDR_END			892

#define S_DMIC2_96k_ADDR			893
#define S_DMIC2_96k_ADDR_END			916

#define S_DMIC4EANC_ADDR			917
#define S_DMIC4EANC_ADDR_END			922

#define S_AMIC_96_F_ADDR			923
#define S_AMIC_96_F_ADDR_END			946

#define S_DMIC0_96_F_ADDR			947
#define S_DMIC0_96_F_ADDR_END			970

#define S_DMIC1_96_F_ADDR			971
#define S_DMIC1_96_F_ADDR_END			994

#define S_DMIC2_96_F_ADDR			995
#define S_DMIC2_96_F_ADDR_END			1018

#define S_UL_MIC_48K_ADDR			1019
#define S_UL_MIC_48K_ADDR_END			1030

#define S_AMIC_EQ_data_ADDR			1031
#define S_AMIC_EQ_data_ADDR_END			1047

#define S_DMIC1_EQ_data_ADDR			1048
#define S_DMIC1_EQ_data_ADDR_END		1064

#define S_DMIC2_EQ_data_ADDR			1065
#define S_DMIC2_EQ_data_ADDR_END		1081

#define S_DMIC3_EQ_data_ADDR			1082
#define S_DMIC3_EQ_data_ADDR_END		1098

#define S_Voice_8k_UL_ADDR			1099
#define S_Voice_8k_UL_ADDR_END			1101

#define S_Voice_8k_DL_ADDR			1102
#define S_Voice_8k_DL_ADDR_END			1103

#define S_VX_DL_data_ADDR			1104
#define S_VX_DL_data_ADDR_END			1120

#define S_McPDM_Out1_ADDR			1121
#define S_McPDM_Out1_ADDR_END			1144

#define S_McPDM_Out2_ADDR			1145
#define S_McPDM_Out2_ADDR_END			1168

#define S_McPDM_Out3_ADDR			1169
#define S_McPDM_Out3_ADDR_END			1192

#define S_Voice_16k_UL_ADDR			1193
#define S_Voice_16k_UL_ADDR_END			1197

#define S_Voice_16k_DL_ADDR			1198
#define S_Voice_16k_DL_ADDR_END			1201

#define S_XinASRC_DL_VX_ADDR			1202
#define S_XinASRC_DL_VX_ADDR_END		1241

#define S_XinASRC_UL_VX_ADDR			1242
#define S_XinASRC_UL_VX_ADDR_END		1281

#define S_XinASRC_DL_MM_ADDR			1282
#define S_XinASRC_DL_MM_ADDR_END		1321

#define S_VX_REC_ADDR				1322
#define S_VX_REC_ADDR_END			1333

#define S_VX_REC_L_ADDR				1334
#define S_VX_REC_L_ADDR_END			1345

#define S_VX_REC_R_ADDR				1346
#define S_VX_REC_R_ADDR_END			1357

#define S_DL2_M_L_ADDR				1358
#define S_DL2_M_L_ADDR_END			1369

#define S_DL2_M_R_ADDR				1370
#define S_DL2_M_R_ADDR_END			1381

#define S_DL2_M_LR_EQ_data_ADDR			1382
#define S_DL2_M_LR_EQ_data_ADDR_END		1406

#define S_DL1_M_EQ_data_ADDR			1407
#define S_DL1_M_EQ_data_ADDR_END		1431

#define S_VX_DL_8_48_BP_data_ADDR		1432
#define S_VX_DL_8_48_BP_data_ADDR_END		1444

#define S_VX_DL_8_48_LP_data_ADDR		1445
#define S_VX_DL_8_48_LP_data_ADDR_END		1457

#define S_EARP_48_96_LP_data_ADDR		1458
#define S_EARP_48_96_LP_data_ADDR_END		1472

#define S_IHF_48_96_LP_data_ADDR		1473
#define S_IHF_48_96_LP_data_ADDR_END		1487

#define S_VX_DL_16_48_HP_data_ADDR		1488
#define S_VX_DL_16_48_HP_data_ADDR_END		1494

#define S_VX_DL_16_48_LP_data_ADDR		1495
#define S_VX_DL_16_48_LP_data_ADDR_END		1507

#define S_VX_UL_48_8_BP_data_ADDR		1508
#define S_VX_UL_48_8_BP_data_ADDR_END		1520

#define S_VX_UL_48_8_LP_data_ADDR		1521
#define S_VX_UL_48_8_LP_data_ADDR_END		1533

#define S_VX_UL_8_TEMP_ADDR			1534
#define S_VX_UL_8_TEMP_ADDR_END			1535

#define S_VX_UL_48_16_HP_data_ADDR		1536
#define S_VX_UL_48_16_HP_data_ADDR_END		1542

#define S_VX_UL_48_16_LP_data_ADDR		1543
#define S_VX_UL_48_16_LP_data_ADDR_END		1555

#define S_VX_UL_16_TEMP_ADDR			1556
#define S_VX_UL_16_TEMP_ADDR_END		1559

#define S_EANC_IIR_data_ADDR			1560
#define S_EANC_IIR_data_ADDR_END		1576

#define S_EANC_SignalTemp_ADDR			1577
#define S_EANC_SignalTemp_ADDR_END		1597

#define S_EANC_Input_ADDR			1598
#define S_EANC_Input_ADDR_END			1598

#define S_EANC_Output_ADDR			1599
#define S_EANC_Output_ADDR_END			1599

#define S_APS_IIRmem1_ADDR			1600
#define S_APS_IIRmem1_ADDR_END			1608

#define S_APS_IIRmem2_ADDR			1609
#define S_APS_IIRmem2_ADDR_END			1611

#define S_APS_OutSamples_ADDR			1612
#define S_APS_OutSamples_ADDR_END		1623

#define S_XinASRC_ECHO_REF_ADDR			1624
#define S_XinASRC_ECHO_REF_ADDR_END		1663

#define S_ECHO_REF_16K_ADDR			1664
#define S_ECHO_REF_16K_ADDR_END			1668

#define S_ECHO_REF_8K_ADDR			1669
#define S_ECHO_REF_8K_ADDR_END			1671

#define S_DL1_ADDR				1672
#define S_DL1_ADDR_END				1683

#define S_APS_DL2_IIRmem1_ADDR			1684
#define S_APS_DL2_IIRmem1_ADDR_END		1692

#define S_APS_DL2_L_IIRmem2_ADDR		1693
#define S_APS_DL2_L_IIRmem2_ADDR_END		1695

#define S_APS_DL2_R_IIRmem2_ADDR		1696
#define S_APS_DL2_R_IIRmem2_ADDR_END		1698

#define S_DL1_APS_ADDR				1699
#define S_DL1_APS_ADDR_END			1710

#define S_DL2_L_APS_ADDR			1711
#define S_DL2_L_APS_ADDR_END			1722

#define S_DL2_R_APS_ADDR			1723
#define S_DL2_R_APS_ADDR_END			1734

#define S_ECHO_REF_48_8_BP_data_ADDR		1735
#define S_ECHO_REF_48_8_BP_data_ADDR_END	1747

#define S_ECHO_REF_48_8_LP_data_ADDR		1748
#define S_ECHO_REF_48_8_LP_data_ADDR_END	1760

#define S_ECHO_REF_48_16_HP_data_ADDR		1761
#define S_ECHO_REF_48_16_HP_data_ADDR_END	1767

#define S_ECHO_REF_48_16_LP_data_ADDR		1768
#define S_ECHO_REF_48_16_LP_data_ADDR_END	1780

#define S_APS_DL1_EQ_data_ADDR			1781
#define S_APS_DL1_EQ_data_ADDR_END		1789

#define S_APS_DL2_EQ_data_ADDR			1790
#define S_APS_DL2_EQ_data_ADDR_END		1798

#define S_DC_DCvalue_ADDR			1799
#define S_DC_DCvalue_ADDR_END			1799

#define S_VIBRA_ADDR				1800
#define S_VIBRA_ADDR_END			1805

#define S_Vibra2_in_ADDR			1806
#define S_Vibra2_in_ADDR_END			1811

#define S_Vibra2_addr_ADDR			1812
#define S_Vibra2_addr_ADDR_END			1812

#define S_VibraCtrl_forRightSM_ADDR		1813
#define S_VibraCtrl_forRightSM_ADDR_END		1836

#define S_Rnoise_mem_ADDR			1837
#define S_Rnoise_mem_ADDR_END			1837

#define S_Ctrl_ADDR				1838
#define S_Ctrl_ADDR_END				1855

#define S_Vibra1_in_ADDR			1856
#define S_Vibra1_in_ADDR_END			1861

#define S_Vibra1_temp_ADDR			1862
#define S_Vibra1_temp_ADDR_END			1885

#define S_VibraCtrl_forLeftSM_ADDR		1886
#define S_VibraCtrl_forLeftSM_ADDR_END		1909

#define S_Vibra1_mem_ADDR			1910
#define S_Vibra1_mem_ADDR_END			1920

#define S_VibraCtrl_Stereo_ADDR			1921
#define S_VibraCtrl_Stereo_ADDR_END		1944

#define S_AMIC_96_48_data_ADDR			1945
#define S_AMIC_96_48_data_ADDR_END		1959

#define S_DMIC0_96_48_data_ADDR			1960
#define S_DMIC0_96_48_data_ADDR_END		1974

#define S_DMIC1_96_48_data_ADDR			1975
#define S_DMIC1_96_48_data_ADDR_END		1989

#define S_DMIC2_96_48_data_ADDR			1990
#define S_DMIC2_96_48_data_ADDR_END		2004

#endif /* _ABESM_ADDR_H_ */
