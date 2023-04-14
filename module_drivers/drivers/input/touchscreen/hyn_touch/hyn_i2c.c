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
* File Name: focaltech_i2c.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-04
*
* Abstract: i2c communication with TP
*
* Version: v1.0
*
* Revision History:
*
************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
//#include "hyn_core.h"
#include "hyn_cst3xx.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define I2C_RETRY_NUMBER                    1
#define I2C_BUF_LENGTH                      256

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/

/*****************************************************************************
* Static variables
*****************************************************************************/

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/

/*****************************************************************************
* functions body
*****************************************************************************/
extern struct cst3xx_ts_data *ts_data;

int cst3xx_bus_init(struct cst3xx_ts_data *ts_data)
{
    HYN_FUNC_ENTER();
    ts_data->bus_tx_buf = kzalloc(I2C_BUF_LENGTH, GFP_KERNEL);
    if (NULL == ts_data->bus_tx_buf) {
        HYN_ERROR("failed to allocate memory for bus_tx_buf");
        return -ENOMEM;
    }

    ts_data->bus_rx_buf = kzalloc(I2C_BUF_LENGTH, GFP_KERNEL);
    if (NULL == ts_data->bus_rx_buf) {
        HYN_ERROR("failed to allocate memory for bus_rx_buf");
        return -ENOMEM;
    }
    HYN_FUNC_EXIT();
    return 0;
}

int cst3xx_bus_exit(struct cst3xx_ts_data *ts_data)
{
    HYN_FUNC_ENTER();
    if (ts_data && ts_data->bus_tx_buf) {
        kfree(ts_data->bus_tx_buf);
        ts_data->bus_tx_buf = NULL;
    }

    if (ts_data && ts_data->bus_rx_buf) {
        kfree(ts_data->bus_rx_buf);
        ts_data->bus_rx_buf = NULL;
    }
    HYN_FUNC_EXIT();
    return 0;
}
