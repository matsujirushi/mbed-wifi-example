/**
 ******************************************************************************
 * @file    bt_rfcomm_server.h
 * @author  Jian Zhang
 * @version V1.0.0
 * @date    2-April-2016
 * @brief   RFCOMM Server Sample application header file
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2014 MXCHIP Inc.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is furnished
 *  to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 *  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ******************************************************************************
 **/

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
 * Constants
 ****************************************************************************/


/*****************************************************************************
 * Globals
 *****************************************************************************/

extern       mico_bt_cfg_settings_t mico_bt_cfg_settings;
extern const mico_bt_cfg_buf_pool_t mico_bt_cfg_buf_pools[];

/*****************************************************************************
 * Function Prototypes
 *****************************************************************************/

extern void ble_proximity_reporter_init(void);

#ifdef __cplusplus
}
#endif
