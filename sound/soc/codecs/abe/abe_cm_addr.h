/*
 * ==========================================================================
 *               Texas Instruments OMAP(TM) Platform Firmware
 * (c) Copyright 2009, Texas Instruments Incorporated.  All Rights Reserved.
 *
 *  Use of this firmware is controlled by the terms and conditions found
 *  in the license agreement under which this firmware has been supplied.
 * ==========================================================================
 */

#ifndef _ABE_CM_ADDR_H_
#define _ABE_CM_ADDR_H_

/* Coefficients memory sizeof */
#define abe_cm_sizeof(x)	(C_##x##_ADDR_END - C_##x##_ADDR + 1)

#define C_init_ADDR				0
#define C_init_ADDR_END				249

#define C_Coef1_ADDR				250
#define C_Coef1_ADDR_END			250

#define C_CoefM1_ADDR				251
#define C_CoefM1_ADDR_END			251

#define C_1_Alpha_ADDR				252
#define C_1_Alpha_ADDR_END			260

#define C_Alpha_ADDR				261
#define C_Alpha_ADDR_END			269

#define C_GainsWRamp_ADDR			270
#define C_GainsWRamp_ADDR_END			287

#define C_Gains_ADDR				288
#define C_Gains_ADDR_END			299

#define C_Gains_DL1M_ADDR			300
#define C_Gains_DL1M_ADDR_END			303

#define C_Gains_DL2M_ADDR			304
#define C_Gains_DL2M_ADDR_END			307

#define C_Gains_EchoM_ADDR			308
#define C_Gains_EchoM_ADDR_END			309

#define C_Gains_SDTM_ADDR			310
#define C_Gains_SDTM_ADDR_END			311

#define C_Gains_VxRecM_ADDR			312
#define C_Gains_VxRecM_ADDR_END			315

#define C_Gains_ULM_ADDR			316
#define C_Gains_ULM_ADDR_END			319

#define C_SDT_Coefs_ADDR			320
#define C_SDT_Coefs_ADDR_END			336

#define C_MM96_F_Coefs_ADDR			337
#define C_MM96_F_Coefs_ADDR_END			353

#define C_DMIC0_96F_Coefs_ADDR			354
#define C_DMIC0_96F_Coefs_ADDR_END		370

#define C_AMIC_EQ_Coefs_ADDR			371
#define C_AMIC_EQ_Coefs_ADDR_END		387

#define C_DMIC1_EQ_Coefs_ADDR			388
#define C_DMIC1_EQ_Coefs_ADDR_END		404

#define C_DMIC2_EQ_Coefs_ADDR			405
#define C_DMIC2_EQ_Coefs_ADDR_END		421

#define C_DMIC3_EQ_Coefs_ADDR			422
#define C_DMIC3_EQ_Coefs_ADDR_END		438

#define C_CoefASRC1_UL_VX_ADDR			439
#define C_CoefASRC1_UL_VX_ADDR_END		457

#define C_CoefASRC2_UL_VX_ADDR			458
#define C_CoefASRC2_UL_VX_ADDR_END		476

#define C_CoefASRC3_UL_VX_ADDR			477
#define C_CoefASRC3_UL_VX_ADDR_END		495

#define C_CoefASRC4_UL_VX_ADDR			496
#define C_CoefASRC4_UL_VX_ADDR_END		514

#define C_CoefASRC5_UL_VX_ADDR			515
#define C_CoefASRC5_UL_VX_ADDR_END		533

#define C_CoefASRC6_UL_VX_ADDR			534
#define C_CoefASRC6_UL_VX_ADDR_END		552

#define C_CoefASRC7_UL_VX_ADDR			553
#define C_CoefASRC7_UL_VX_ADDR_END		571

#define C_CoefASRC8_UL_VX_ADDR			572
#define C_CoefASRC8_UL_VX_ADDR_END		590

#define C_CoefASRC9_UL_VX_ADDR			591
#define C_CoefASRC9_UL_VX_ADDR_END		609

#define C_CoefASRC10_UL_VX_ADDR			610
#define C_CoefASRC10_UL_VX_ADDR_END		628

#define C_CoefASRC11_UL_VX_ADDR			629
#define C_CoefASRC11_UL_VX_ADDR_END		647

#define C_CoefASRC12_UL_VX_ADDR			648
#define C_CoefASRC12_UL_VX_ADDR_END		666

#define C_CoefASRC13_UL_VX_ADDR			667
#define C_CoefASRC13_UL_VX_ADDR_END		685

#define C_CoefASRC14_UL_VX_ADDR			686
#define C_CoefASRC14_UL_VX_ADDR_END		704

#define C_CoefASRC15_UL_VX_ADDR			705
#define C_CoefASRC15_UL_VX_ADDR_END		723

#define C_CoefASRC16_UL_VX_ADDR			724
#define C_CoefASRC16_UL_VX_ADDR_END		742

#define C_AlphaCurrent_UL_VX_ADDR		743
#define C_AlphaCurrent_UL_VX_ADDR_END		743

#define C_BetaCurrent_UL_VX_ADDR		744
#define C_BetaCurrent_UL_VX_ADDR_END		744

#define C_CoefASRC1_DL_VX_ADDR			745
#define C_CoefASRC1_DL_VX_ADDR_END		763

#define C_CoefASRC2_DL_VX_ADDR			764
#define C_CoefASRC2_DL_VX_ADDR_END		782

#define C_CoefASRC3_DL_VX_ADDR			783
#define C_CoefASRC3_DL_VX_ADDR_END		801

#define C_CoefASRC4_DL_VX_ADDR			802
#define C_CoefASRC4_DL_VX_ADDR_END		820

#define C_CoefASRC5_DL_VX_ADDR			821
#define C_CoefASRC5_DL_VX_ADDR_END		839

#define C_CoefASRC6_DL_VX_ADDR			840
#define C_CoefASRC6_DL_VX_ADDR_END		858

#define C_CoefASRC7_DL_VX_ADDR			859
#define C_CoefASRC7_DL_VX_ADDR_END		877

#define C_CoefASRC8_DL_VX_ADDR			878
#define C_CoefASRC8_DL_VX_ADDR_END		896

#define C_CoefASRC9_DL_VX_ADDR			897
#define C_CoefASRC9_DL_VX_ADDR_END		915

#define C_CoefASRC10_DL_VX_ADDR			916
#define C_CoefASRC10_DL_VX_ADDR_END		934

#define C_CoefASRC11_DL_VX_ADDR			935
#define C_CoefASRC11_DL_VX_ADDR_END		953

#define C_CoefASRC12_DL_VX_ADDR			954
#define C_CoefASRC12_DL_VX_ADDR_END		972

#define C_CoefASRC13_DL_VX_ADDR			973
#define C_CoefASRC13_DL_VX_ADDR_END		991

#define C_CoefASRC14_DL_VX_ADDR			992
#define C_CoefASRC14_DL_VX_ADDR_END		1010

#define C_CoefASRC15_DL_VX_ADDR			1011
#define C_CoefASRC15_DL_VX_ADDR_END		1029

#define C_CoefASRC16_DL_VX_ADDR			1030
#define C_CoefASRC16_DL_VX_ADDR_END		1048

#define C_AlphaCurrent_DL_VX_ADDR		1049
#define C_AlphaCurrent_DL_VX_ADDR_END		1049

#define C_BetaCurrent_DL_VX_ADDR		1050
#define C_BetaCurrent_DL_VX_ADDR_END		1050

#define C_CoefASRC1_DL_MM_ADDR			1051
#define C_CoefASRC1_DL_MM_ADDR_END		1068

#define C_CoefASRC2_DL_MM_ADDR			1069
#define C_CoefASRC2_DL_MM_ADDR_END		1086

#define C_CoefASRC3_DL_MM_ADDR			1087
#define C_CoefASRC3_DL_MM_ADDR_END		1104

#define C_CoefASRC4_DL_MM_ADDR			1105
#define C_CoefASRC4_DL_MM_ADDR_END		1122

#define C_CoefASRC5_DL_MM_ADDR			1123
#define C_CoefASRC5_DL_MM_ADDR_END		1140

#define C_CoefASRC6_DL_MM_ADDR			1141
#define C_CoefASRC6_DL_MM_ADDR_END		1158

#define C_CoefASRC7_DL_MM_ADDR			1159
#define C_CoefASRC7_DL_MM_ADDR_END		1176

#define C_CoefASRC8_DL_MM_ADDR			1177
#define C_CoefASRC8_DL_MM_ADDR_END		1194

#define C_CoefASRC9_DL_MM_ADDR			1195
#define C_CoefASRC9_DL_MM_ADDR_END		1212

#define C_CoefASRC10_DL_MM_ADDR			1213
#define C_CoefASRC10_DL_MM_ADDR_END		1230

#define C_CoefASRC11_DL_MM_ADDR			1231
#define C_CoefASRC11_DL_MM_ADDR_END		1248

#define C_CoefASRC12_DL_MM_ADDR			1249
#define C_CoefASRC12_DL_MM_ADDR_END		1266

#define C_CoefASRC13_DL_MM_ADDR			1267
#define C_CoefASRC13_DL_MM_ADDR_END		1284

#define C_CoefASRC14_DL_MM_ADDR			1285
#define C_CoefASRC14_DL_MM_ADDR_END		1302

#define C_CoefASRC15_DL_MM_ADDR			1303
#define C_CoefASRC15_DL_MM_ADDR_END		1320

#define C_CoefASRC16_DL_MM_ADDR			1321
#define C_CoefASRC16_DL_MM_ADDR_END		1338

#define C_AlphaCurrent_DL_MM_ADDR		1339
#define C_AlphaCurrent_DL_MM_ADDR_END		1339

#define C_BetaCurrent_DL_MM_ADDR		1340
#define C_BetaCurrent_DL_MM_ADDR_END		1340

#define C_DL2_L_Coefs_ADDR			1341
#define C_DL2_L_Coefs_ADDR_END			1365

#define C_DL2_R_Coefs_ADDR			1366
#define C_DL2_R_Coefs_ADDR_END			1390

#define C_DL1_Coefs_ADDR			1391
#define C_DL1_Coefs_ADDR_END			1415

#define C_VX_8_48_BP_Coefs_ADDR			1416
#define C_VX_8_48_BP_Coefs_ADDR_END		1428

#define C_VX_8_48_LP_Coefs_ADDR			1429
#define C_VX_8_48_LP_Coefs_ADDR_END		1441

#define C_VX_48_8_LP_Coefs_ADDR			1442
#define C_VX_48_8_LP_Coefs_ADDR_END		1454

#define C_VX_16_48_HP_Coefs_ADDR		1455
#define C_VX_16_48_HP_Coefs_ADDR_END		1461

#define C_VX_16_48_LP_Coefs_ADDR		1462
#define C_VX_16_48_LP_Coefs_ADDR_END		1474

#define C_VX_48_16_LP_Coefs_ADDR		1475
#define C_VX_48_16_LP_Coefs_ADDR_END		1487

#define C_EANC_WarpCoeffs_ADDR			1488
#define C_EANC_WarpCoeffs_ADDR_END		1489

#define C_EANC_FIRcoeffs_ADDR			1490
#define C_EANC_FIRcoeffs_ADDR_END		1510

#define C_EANC_IIRcoeffs_ADDR			1511
#define C_EANC_IIRcoeffs_ADDR_END		1527

#define C_APS_coeffs1_ADDR			1528
#define C_APS_coeffs1_ADDR_END			1536

#define C_APS_coeffs2_ADDR			1537
#define C_APS_coeffs2_ADDR_END			1539

#define C_APS_DL2_L_coeffs1_ADDR		1540
#define C_APS_DL2_L_coeffs1_ADDR_END		1548

#define C_APS_DL2_R_coeffs1_ADDR		1549
#define C_APS_DL2_R_coeffs1_ADDR_END		1557

#define C_APS_DL2_L_coeffs2_ADDR		1558
#define C_APS_DL2_L_coeffs2_ADDR_END		1560

#define C_APS_DL2_R_coeffs2_ADDR		1561
#define C_APS_DL2_R_coeffs2_ADDR_END		1563

#define C_AlphaCurrent_ECHO_REF_ADDR		1564
#define C_AlphaCurrent_ECHO_REF_ADDR_END	1564

#define C_BetaCurrent_ECHO_REF_ADDR		1565
#define C_BetaCurrent_ECHO_REF_ADDR_END		1565

#define C_APS_DL1_EQ_ADDR			1566
#define C_APS_DL1_EQ_ADDR_END			1574

#define C_APS_DL2_L_EQ_ADDR			1575
#define C_APS_DL2_L_EQ_ADDR_END			1583

#define C_APS_DL2_R_EQ_ADDR			1584
#define C_APS_DL2_R_EQ_ADDR_END			1592

#define C_Vibra2_consts_ADDR			1593
#define C_Vibra2_consts_ADDR_END		1596

#define C_Vibra1_coeffs_ADDR			1597
#define C_Vibra1_coeffs_ADDR_END		1607

#define C_48_96_LP_Coefs_ADDR			1608
#define C_48_96_LP_Coefs_ADDR_END		1622

#define C_98_48_LP_Coefs_ADDR			1623
#define C_98_48_LP_Coefs_ADDR_END		1637

#endif	/* _ABECM_ADDR_H_ */
