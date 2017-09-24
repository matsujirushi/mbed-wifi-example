/**
 ******************************************************************************
 * @file    ble_hello_center.cpp
 * @author  Jian Zhang
 * @version V1.0.0
 * @date    12-Sep-2017
 * @file    Demonstrate a BLE center device function
 * ******************************************************************************
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

#include <string.h>

#include "mico.h"
#include "mico_bt_ble.h"
#include "mico_bt_gatt.h"
#include "mico_bt_cfg.h"
#include "mico_bt_stack.h"

#include "StringUtils.h"

#include "oled.h"

#include "ble_hello_center.h"

/******************************************************************************
 *                                Constants
******************************************************************************/

#define HANDLE_HSENS_SERVICE_CHAR_COLOR_VAL 0x2d

#define HELLO_PERIPHERAL_LOG(fmt, ...) custom_log("HELLO", fmt, ##__VA_ARGS__)

/******************************************************************************
 *                                Structures
 ******************************************************************************/

/**
 *  Bluetooth Smart device
 */
typedef struct {
    mico_bt_device_address_t    address;      /**< Bluetooth device address */
    uint8_t                     addr_type;    /**< Bluetooth device address type */
    char                        name[31];     /**< User-friendly name       */
    uint16_t                    conn_id;      /**< Connection ID */
} hello_center_device_t;

/******************************************************************************
 *                           Function Prototypes
 ******************************************************************************/

static void hello_center_application_init();

static void hello_center_scan_result_callback(mico_bt_ble_scan_results_t *p_adv_result, uint8_t *p_adv_data);
static mico_bt_result_t hello_center_management_callback(mico_bt_management_evt_t event, mico_bt_management_evt_data_t *p_event_data);
static mico_bt_gatt_status_t hello_center_gatts_connection_status_handler(mico_bt_gatt_connection_status_t *p_status);
static mico_bt_gatt_status_t hello_center_gatts_connection_up(mico_bt_gatt_connection_status_t *p_status);
static mico_bt_gatt_status_t hello_center_gatts_connection_down(mico_bt_gatt_connection_status_t *p_status);
static mico_bt_gatt_status_t hello_center_gatts_callback(mico_bt_gatt_evt_t event, mico_bt_gatt_event_data_t *p_data);

static OSStatus hello_center_handle_async_scanning_stop_event(void *arg);
static OSStatus hello_center_handle_async_connect_request(void *arg);
static OSStatus hello_center_handle_async_conn_up(void *arg);
static OSStatus hello_center_handle_async_conn_down(void *arg);
static OSStatus hello_center_handle_async_timer_event(void *arg);

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/

static char peer_device[] = DEFAULT_NAME;
static char oled_show_line[OLED_DISPLAY_MAX_CHAR_PER_ROW + 1] = { '\0' };   // max char each line

static mico_worker_thread_t     hello_center_worker_thread;
static mico_worker_thread_t     hello_center_timer_worker_thread;
static mico_timed_event_t       hello_center_time_event;

static hello_center_device_t   *hello_center_current_device = NULL;
static mico_semaphore_t         hello_center_write_op_sem;

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */

void app_ble_hello_center()
{
    OSStatus         err;
    mico_bt_result_t result;

    mico_system_context_init(0);

    OLED_Init();

    HELLO_PERIPHERAL_LOG("Hello Center Start");
    snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "BLE starting ...");
    OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_1, oled_show_line);

    /* Register call back and configuration with stack */
    result = mico_bt_stack_init(hello_center_management_callback,
                                &mico_bt_cfg_settings_peripheral,
                                mico_bt_cfg_buf_pools_peripheral);
    if (result != MICO_BT_SUCCESS) {
        HELLO_PERIPHERAL_LOG("Bluetooth Stack initialised failed");
        goto exit1;
    }

    /* Initialise other section */
    err = mico_rtos_create_worker_thread(&hello_center_worker_thread, MICO_APPLICATION_PRIORITY, 2048, 10);
    require_noerr_string(err, exit1, "Create worker thread failed");

    err = mico_rtos_create_worker_thread(&hello_center_timer_worker_thread, MICO_APPLICATION_PRIORITY, 2048, 1);
    require_noerr_string(err, exit2, "Create worker thread for timer failed");

    return;

exit2:
    mico_rtos_delete_worker_thread(&hello_center_worker_thread);
exit1:
    memset(oled_show_line, 0, sizeof(oled_show_line));
    snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "BLE init failed");
    OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_1, oled_show_line);
}

/*
 * This function is executed in the BTM_ENABLED_EVT management callback.
 */
static void hello_center_application_init()
{
    mico_bt_gatt_status_t gatt_status;
    mico_bt_result_t      result;

    HELLO_PERIPHERAL_LOG("hello_center_application_init");
    /* Started */
    memset(oled_show_line, 0, sizeof(oled_show_line));
    snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "BLE Hello Center");
    OLED_Clear();
    OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_1, oled_show_line);

    /* Register with stack to receive GATT callback */
    gatt_status = mico_bt_gatt_register(GATT_IF_CLIENT, hello_center_gatts_callback);
    HELLO_PERIPHERAL_LOG("mico_bt_gatt_register: %d", gatt_status);

    /* Start scanning procedure. */
    result = mico_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY, MICO_TRUE, hello_center_scan_result_callback);
    if (MICO_BT_PENDING != result) {
        HELLO_PERIPHERAL_LOG("mico_bt_ble_scan failed");
        return;
    }
    HELLO_PERIPHERAL_LOG("Start to scan %d", result);

    /* Advertising */
    memset(oled_show_line, 0, sizeof(oled_show_line));
    snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "    SCANNING    ");
    OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, oled_show_line);
}

/*
 * hello_center scanning report callback
 */
static void
hello_center_scan_result_callback(mico_bt_ble_scan_results_t *p_adv_result, uint8_t *p_adv_data)
{
#if 0
    if (p_adv_result != NULL) {
        HELLO_PERIPHERAL_LOG("ADDR: %s  RSSI: %d",
                             DataToHexStringWithColons(p_adv_result->remote_bd_addr, 6),
                             p_adv_result->rssi);
    }
#endif

    if (!hello_center_current_device && p_adv_result != NULL && p_adv_result->length > 0) {
        uint8_t data_length = 0;
        uint8_t *name = mico_bt_ble_check_advertising_data(p_adv_data,
                                                           BTM_BLE_ADVERT_TYPE_NAME_COMPLETE,
                                                           &data_length);
        if (name != NULL && data_length != 0) {
            if (memcmp(name, peer_device, strlen(peer_device)) == 0) {
                hello_center_current_device = (hello_center_device_t *)malloc(sizeof(hello_center_device_t));
                if (hello_center_current_device != NULL) {
                    /* Prepare to connect it. */
                    memset(hello_center_current_device, 0, sizeof(hello_center_device_t));
                    hello_center_current_device->addr_type = p_adv_result->ble_evt_type;
                    memcpy(hello_center_current_device->address, p_adv_result->remote_bd_addr, 6);
                    memcpy(hello_center_current_device->name, name, MIN(sizeof(hello_center_current_device->name), data_length));
                    OSStatus err = mico_rtos_send_asynchronous_event(&hello_center_worker_thread,
                                                                     hello_center_handle_async_connect_request,
                                                                     hello_center_current_device);
                    if (err != kNoErr) {
                        free(hello_center_current_device);
                        hello_center_current_device = NULL;
                    }
                } else {
                    HELLO_PERIPHERAL_LOG("%s: malloc(hello_center_device_t) failed", __FUNCTION__);
                }
            }
        }
    } else if (p_adv_result == NULL) {
        mico_rtos_send_asynchronous_event(&hello_center_worker_thread,
                                          hello_center_handle_async_scanning_stop_event,
                                          NULL);
    }
}

/*
 * hello_sensor bt/ble link management callback
 */
static mico_bt_result_t
hello_center_management_callback(mico_bt_management_evt_t event,
                                 mico_bt_management_evt_data_t *p_event_data)
{
    mico_bt_result_t result = MICO_BT_SUCCESS;
    mico_bt_dev_ble_pairing_info_t *p_info;
    mico_bt_ble_scan_type_t scan_type;

    HELLO_PERIPHERAL_LOG("hello_center_management_callback: %x", event);

    switch (event) {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            hello_center_application_init();
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_BOND | BTM_LE_AUTH_REQ_MITM;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_info = &p_event_data->pairing_complete.pairing_complete_info.ble;
            HELLO_PERIPHERAL_LOG("Pairing Complete: %d", p_info->reason);
            break;

        case BTM_SECURITY_REQUEST_EVT:
            mico_bt_ble_security_grant(p_event_data->security_request.bd_addr,
                                       MICO_BT_SUCCESS);
            break;


        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            scan_type = p_event_data->ble_scan_state_changed;
            HELLO_PERIPHERAL_LOG("Scanning State change: %d", scan_type);
            break;

        default:
            break;
    }

    return result;
}

/* This function is invoked when connection is established */
static mico_bt_gatt_status_t
hello_center_gatts_connection_up(mico_bt_gatt_connection_status_t *p_status)
{
    /* update connection status */
    HELLO_PERIPHERAL_LOG("Connection Up :0x%04x", p_status->conn_id);
    hello_center_current_device->conn_id = p_status->conn_id;

    mico_rtos_send_asynchronous_event(&hello_center_worker_thread,
                                      hello_center_handle_async_conn_up,
                                      NULL);

    return MICO_BT_GATT_SUCCESS;
}

/*
 * This function is invoked when connection is lost
 */
static mico_bt_gatt_status_t
hello_center_gatts_connection_down(mico_bt_gatt_connection_status_t *p_status)
{
    /* update connection status */
    HELLO_PERIPHERAL_LOG("Connection down :0x%04x reason:%d",
                         p_status->conn_id, p_status->reason);

    mico_rtos_send_asynchronous_event(&hello_center_worker_thread,
                                      hello_center_handle_async_conn_down,
                                      NULL);

    return MICO_BT_SUCCESS;
}

/*
 * Connection up/down event
 */
static mico_bt_gatt_status_t
hello_center_gatts_connection_status_handler(mico_bt_gatt_connection_status_t *p_status)
{
    if (p_status->connected) {
        return hello_center_gatts_connection_up(p_status);
    }

    return hello_center_gatts_connection_down(p_status);
}

/*
 * Callback for various GATT events.  As this application performs only as a
 * GATT server, some of
 * the events are ommitted.
 */
static mico_bt_gatt_status_t
hello_center_gatts_callback(mico_bt_gatt_evt_t event,
                            mico_bt_gatt_event_data_t *p_data)
{
    mico_bt_gatt_status_t result = MICO_BT_GATT_INVALID_PDU;

    switch (event) {
        case GATT_CONNECTION_STATUS_EVT:
            result = hello_center_gatts_connection_status_handler(&p_data->connection_status);
            break;

        case GATT_OPERATION_CPLT_EVT:
            if (p_data->operation_complete.op == GATTC_OPTYPE_WRITE) {
#if 0
                HELLO_PERIPHERAL_LOG("Write-callback event for handle: 0x%02x status: %d",
                                     p_data->operation_complete.response_data.handle,
                                     p_data->operation_complete.status);
#endif
                mico_rtos_set_semaphore(&hello_center_write_op_sem);
            }
            break;

        case GATT_DISCOVERY_RESULT_EVT:
            break;

        case GATT_DISCOVERY_CPLT_EVT:
            break;

        default:
            break;
    }

    return result;
}

static OSStatus hello_center_handle_async_scanning_stop_event(void *arg)
{
    UNUSED_PARAMETER(arg);

    mico_bt_result_t result = mico_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY,
                                               MICO_TRUE,
                                               hello_center_scan_result_callback);
    HELLO_PERIPHERAL_LOG("restart to scann: %d", result);

    return kNoErr;
}

static OSStatus hello_center_handle_async_connect_request(void *arg)
{
    if (arg == NULL) return kGeneralErr;

    /* Connection */
    if (mico_bt_gatt_le_connect(hello_center_current_device->address,
                                hello_center_current_device->addr_type,
                                BTM_BLE_SCAN_TYPE_HIGH_DUTY,
                                MICO_TRUE)) {
        HELLO_PERIPHERAL_LOG("start to connect device: %s", hello_center_current_device->name);
    } else {
        free(hello_center_current_device);
        hello_center_current_device = NULL;
        HELLO_PERIPHERAL_LOG("connect to device failed");
    }

    return kNoErr;
}

static OSStatus hello_center_handle_async_conn_up(void *arg)
{
    OSStatus err;

    UNUSED_PARAMETER(arg);

    /* Todo discovery specified Service by UUID */

    /* Now we used HANDLE_HSENS_SERVICE_CHAR_COLOR_VAL to control ATT Server */
    mico_rtos_init_semaphore(&hello_center_write_op_sem, 1);
    err = mico_rtos_register_timed_event(&hello_center_time_event, &hello_center_timer_worker_thread,
                                         hello_center_handle_async_timer_event, 1000, NULL);
    require_noerr_string(err, exit, "Register Timer Event failed");

    exit:
    return kNoErr;
}

static OSStatus hello_center_handle_async_conn_down(void *arg)
{
    UNUSED_PARAMETER(arg);

    if (hello_center_current_device) {
        free(hello_center_current_device);
        hello_center_current_device = NULL;
    }

    /* De-init resource */
    mico_rtos_deregister_timed_event(&hello_center_time_event);
    mico_rtos_deinit_semaphore(&hello_center_write_op_sem);

    /* Clear OLED */
    memset(oled_show_line, 0, sizeof(oled_show_line));
    snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "BLE Hello Center");
    OLED_Clear();
    OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_1, oled_show_line);

    memset(oled_show_line, 0, sizeof(oled_show_line));
    snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "    SCANNING    ");
    OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, oled_show_line);

    return kNoErr;
}

static OSStatus hello_center_handle_async_timer_event(void *arg)
{
    static uint8_t color_idx = 0;

    OSStatus err;

    uint8_t                 buffer[50];
    mico_bt_gatt_value_t    *value = (mico_bt_gatt_value_t *)buffer;

    UNUSED_PARAMETER(arg);

    mico_rtos_get_semaphore(&hello_center_write_op_sem, 0);

    value->handle   = HANDLE_HSENS_SERVICE_CHAR_COLOR_VAL;
    value->offset   = 0;
    value->len      = sizeof(color_idx);
    value->auth_req = GATT_AUTH_REQ_NONE;
    memcpy(value->value, &color_idx, value->len);

    err = mico_bt_gatt_send_write(hello_center_current_device->conn_id, GATT_WRITE, value);
    if (err != kNoErr) {
        HELLO_PERIPHERAL_LOG("mico_bt_gatt_send_write failed: %d", err);
        goto exit;
    }

    err = mico_rtos_get_semaphore(&hello_center_write_op_sem, 1000 * 3);
    if (err != kNoErr) {
        HELLO_PERIPHERAL_LOG("mico_bt_gatt_send_write tiemout");
    }

    memset(oled_show_line, 0, sizeof(oled_show_line));
    snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "Tx Color: 0x%02x", color_idx++);
    OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_3, oled_show_line);

exit:
    return err;
}
