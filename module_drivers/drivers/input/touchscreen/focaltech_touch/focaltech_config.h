/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2019, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/************************************************************************
*
* File Name: focaltech_config.h
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract: global configurations
*
* Version: v1.0
*
************************************************************************/
#ifndef _LINUX_FOCLATECH_CONFIG_H_
#define _LINUX_FOCLATECH_CONFIG_H_

/**************************************************/
/****** G: A, I: B, S: C, U: D  ******************/
/****** chip type defines, do not modify *********/

#define _FT7511             0x75110402
#define _FT7661             0x76610402
/*************************************************/

/*
 * choose your ic chip type of focaltech
 */
//#define FTS_CHIP_TYPE   _FT7511
#define FTS_CHIP_TYPE   0x0402

/******************* Enables *********************/
/*********** 1 to enable, 0 to disable ***********/

/*
 * show debug log info
 * enable it for debug, disable it for release
 */
#define FTS_DEBUG_EN                            0

/*
 * Linux MultiTouch Protocol
 * 1: Protocol B(default), 0: Protocol A
 */
#if defined (CONFIG_STAGE_TL040WVS03CT) || defined (CONFIG_STAGE_ST7701S_RGB666)
#define FTS_MT_PROTOCOL_B_EN                    0
#else
#define FTS_MT_PROTOCOL_B_EN                    1
#endif /*CONFIG_PANEL_V12_TL040WVS03CT*/


/*
 * Report Pressure in multitouch
 * 1:enable(default),0:disable
*/
#define FTS_REPORT_PRESSURE_EN                  1

/*
 * Gesture function enable
 * default: disable
 */
#define FTS_GESTURE_EN                          0

/*
 * ESD check & protection
 * default: disable
 */
#define FTS_ESDCHECK_EN                         0


/*
 * Pinctrl enable
 * default: disable
 */
#define FTS_PINCTRL_EN                          0


/*
 * Customer power enable
 * enable it when customer need control TP power
 * default: disable
 */
#define FTS_POWER_SOURCE_CUST_EN                0

#define FTS_EX_MODE_EN				0
#define FTS_EX_FUNC_EN				0

/****************************************************/

/********************** Upgrade ****************************/
/*
 * auto upgrade
 */
#define FTS_AUTO_UPGRADE_EN                     0

/*
 * auto upgrade for lcd cfg
 */
#define FTS_AUTO_LIC_UPGRADE_EN                 0

/*
 * Numbers of modules support
 */
#define FTS_GET_MODULE_NUM                      0

/*
 * module_id: mean vendor_id generally, also maybe gpio or lcm_id...
 * If means vendor_id, the FTS_MODULE_ID = PANEL_ID << 8 + VENDOR_ID
 * FTS_GET_MODULE_NUM == 0/1, no check module id, you may ignore them
 * FTS_GET_MODULE_NUM >= 2, compatible with FTS_MODULE2_ID
 * FTS_GET_MODULE_NUM >= 3, compatible with FTS_MODULE3_ID
 */
#define FTS_MODULE_ID                          0x0000
#define FTS_MODULE2_ID                         0x0000
#define FTS_MODULE3_ID                         0x0000

/*
 * Need set the following when get firmware via firmware_request()
 * For example: if module'vendor is tianma,
 * #define FTS_MODULE_NAME                        "tianma"
 * then file_name will be "focaltech_ts_fw_tianma"
 * You should rename fw to "focaltech_ts_fw_tianma", and push it into
 * etc/firmware or by customers
 */
#define FTS_MODULE_NAME                        ""
#define FTS_MODULE2_NAME                       ""
#define FTS_MODULE3_NAME                       ""

/*
 * FW.i file for auto upgrade, you must replace it with your own
 * define your own fw_file, the sample one to be replaced is invalid
 * NOTE: if FTS_GET_MODULE_NUM > 1, it's the fw corresponding with FTS_VENDOR_ID
 */
#define FTS_UPGRADE_FW_FILE                      "include/firmware/fw_sample.i"

/*
 * if FTS_GET_MODULE_NUM >= 2, fw corrsponding with FTS_VENDOR_ID2
 * define your own fw_file, the sample one is invalid
 */
#define FTS_UPGRADE_FW2_FILE                     "include/firmware/fw_sample.i"

/*
 * if FTS_GET_MODULE_NUM >= 3, fw corrsponding with FTS_VENDOR_ID3
 * define your own fw_file, the sample one is invalid
 */
#define FTS_UPGRADE_FW3_FILE                     "include/firmware/fw_sample.i"

/*********************************************************/

#endif /* _LINUX_FOCLATECH_CONFIG_H_ */
