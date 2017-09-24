/**
 ******************************************************************************
 * @file    ble_hello_sensor.cpp
 * @author  Jian Zhang
 * @version V1.0.0
 * @date    12-Sep-2017
 * @file    Demonstrate a BLE peripheral device function
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

#include "mbed.h"
#include "mico.h"
#include "mico_bt_dev.h"
#include "mico_bt_ble.h"
#include "mico_bt_cfg.h"
#include "mico_bt_stack.h"
#include "sdpdefs.h"

#include "oled.h"

#include "ble_hello_sensor.h"

/******************************************************************************
 *                                Constants
******************************************************************************/


/******************************************************************************
 *                           Function Prototypes
 ******************************************************************************/

#define HELLO_PERIPHERAL_LOG(fmt, ...) custom_log("HELLO", fmt, ##__VA_ARGS__)

/* HSB convert to RGB */

void hsb2rgb_led_init();
void hsb2rgb_led_open(float hues, float saturation, float brightness);
void hsb2rgb_led_close();

static void hello_sensor_application_init();
static mico_bt_gatt_status_t hello_sensor_gatts_connection_status_handler(mico_bt_gatt_connection_status_t *p_status);
static mico_bt_gatt_status_t hello_sensor_gatts_connection_up(mico_bt_gatt_connection_status_t *p_status);
static mico_bt_gatt_status_t hello_sensor_gatts_connection_down(mico_bt_gatt_connection_status_t *p_status);
static mico_bt_result_t      hello_sensor_management_callback(mico_bt_management_evt_t event, mico_bt_management_evt_data_t *p_event_data);
static mico_bt_gatt_status_t hello_sensor_gatts_callback(mico_bt_gatt_evt_t event, mico_bt_gatt_event_data_t *p_data);
static mico_bt_gatt_status_t hello_sensor_gatt_server_read_request_handler(uint16_t conn_id, mico_bt_gatt_read_t *p_read_data);
static mico_bt_gatt_status_t hello_sensor_gatt_server_write_request_handler(uint16_t conn_id, mico_bt_gatt_write_t *p_data);
static mico_bt_gatt_status_t hello_sensor_gatt_server_write_and_execute_request_handler(uint16_t conn_id, mico_bt_gatt_exec_flag_t exec_flag);
static void hello_sensor_rssi_callback(void *arg);
static void hello_sensor_set_advertisement_data();

/******************************************************************************
 *                                Structures
 ******************************************************************************/

typedef struct {
    BD_ADDR     remote_addr;          /* remote peer device address */
    uint32_t    timer_count;          /* timer count */
    uint32_t    fine_timer_count;     /* fine timer count */
    uint16_t    conn_id;              /* connection ID referenced by the stack */
    uint16_t    peer_mtu;             /* peer MTU */
    uint8_t     flag_indication_sent; /* indicates waiting for confirmation */
    uint8_t     flag_stay_connected;  /* stay connected or disconnect after all messages are sent */
    uint8_t     battery_level;        /* dummy battery level */
} hello_sensor_state_t;

typedef PACKED struct {
    BD_ADDR     bdaddr;                               /* BD address of the bonded host */
    uint16_t    characteristic_client_configuration;  /* Current value of the client configuration descriptor */
    uint8_t     color_idx;                            /* Sensor config, number of times to blink the LEd when button is pushed. */
} host_info_t;

typedef struct {
    uint16_t handle;
    uint16_t attr_len;
    void    *p_attr;
} attribute_t;

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/

PwmOut LED_R(RGB_R);
PwmOut LED_G(RGB_G);
PwmOut LED_B(RGB_B);

static char oled_show_line[OLED_DISPLAY_MAX_CHAR_PER_ROW + 1] = { '\0' };   // max char each line

static uint8_t hello_sensor_device_name[] = "Hello";

static uint8_t hello_sensor_appearance_name[2] = {
        BIT16_TO_8(APPEARANCE_GENERIC_TAG)
};

static char hello_sensor_char_notify_value[] = {
        'H', 'E', 'L', 'L', 'O', '0'
};

static char hello_sensor_char_mfr_name_value[] = {
        'B', 'r', 'o', 'a', 'd', 'c', 'o', 'm', 0,
};

static char hello_sensor_char_model_num_value[] = {
        '1', '2', '3', '4',
         0,   0,   0,   0
};

static uint8_t hello_sensor_char_system_id_value[] = {
        0xbb, 0xb8, 0xa1, 0x80,
        0x5f, 0x9f, 0x91, 0x71
};

static hello_sensor_state_t hello_sensor_state;
static host_info_t          hello_sensor_hostinfo;
static mico_bool_t          is_connected = FALSE;
static uint8_t              hello_sensor_write;

attribute_t gatt_user_attributes[] = {
    { HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,        sizeof(hello_sensor_device_name),           hello_sensor_device_name },
    { HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,  sizeof(hello_sensor_appearance_name),       hello_sensor_appearance_name },
    { HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL,              sizeof(hello_sensor_char_notify_value),     hello_sensor_char_notify_value },
    { HANDLE_HSENS_SERVICE_CHAR_CFG_DESC,                2,                                          (void *)&hello_sensor_hostinfo.characteristic_client_configuration},
    { HANDLE_HSENS_SERVICE_CHAR_COLOR_VAL,               1,                                          &hello_sensor_hostinfo.color_idx },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,   sizeof(hello_sensor_char_mfr_name_value),   hello_sensor_char_mfr_name_value },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,  sizeof(hello_sensor_char_model_num_value),  hello_sensor_char_model_num_value },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,  sizeof(hello_sensor_char_system_id_value),  hello_sensor_char_system_id_value },
    { HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,       1,                                          &hello_sensor_state.battery_level },
};

/******************************************************************************
 *                                GATT DATABASE
 ******************************************************************************/
/*
 * This is the GATT database for the Hello Sensor application.  It defines
 * services, characteristics and descriptors supported by the sensor.  Each
 * attribute in the database has a handle, (characteristic has two, one for
 * characteristic itself, another for the value).  The handles are used by
 * the peer to access attributes, and can be used locally by application for
 * example to retrieve data written by the peer.  Definition of characteristics
 * and descriptors has GATT Properties (read, write, notify...) but also has
 * permissions which identify if and how peer is allowed to read or write
 * into it.  All handles do not need to be sequential, but need to be in
 * ascending order.
 */
const uint8_t hello_sensor_gatt_database[] = {
    /* Declare mandatory GATT service */
    PRIMARY_SERVICE_UUID16(HANDLE_HSENS_GATT_SERVICE,
                           UUID_SERVCLASS_GATT_SERVER),

    /* Declare mandatory GAP service. Device Name and Appearance are
     * mandatory
     * characteristics of GAP service */
    PRIMARY_SERVICE_UUID16(HANDLE_HSENS_GAP_SERVICE,
                           UUID_SERVCLASS_GAP_SERVER),

    /* Declare mandatory GAP service characteristic: Dev Name */
    CHARACTERISTIC_UUID16(
            HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME,
            HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,
            GATT_UUID_GAP_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ,
            LEGATTDB_PERM_READABLE),

    /* Declare mandatory GAP service characteristic: Appearance */
    CHARACTERISTIC_UUID16(
            HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE,
            HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
            GATT_UUID_GAP_ICON, LEGATTDB_CHAR_PROP_READ,
            LEGATTDB_PERM_READABLE),

    /* Declare proprietary Hello Service with 128 byte UUID */
    PRIMARY_SERVICE_UUID128(HANDLE_HSENS_SERVICE,
                            UUID_HELLO_SERVICE),

    /* Declare characteristic used to notify/indicate change */
    CHARACTERISTIC_UUID128(HANDLE_HSENS_SERVICE_CHAR_NOTIFY,
                           HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL,
                           UUID_HELLO_CHARACTERISTIC_NOTIFY,
                           LEGATTDB_CHAR_PROP_READ |
                           LEGATTDB_CHAR_PROP_NOTIFY |
                           LEGATTDB_CHAR_PROP_INDICATE,
                           LEGATTDB_PERM_READABLE),

    /* Declare client characteristic configuration descriptor
     * Value of the descriptor can be modified by the client
     * Value modified shall be retained during connection and across
     * connection
     * for bonded devices.  Setting value to 1 tells this
     * application to send notification
     * when value of the characteristic changes.  Value 2 is to
     * allow indications. */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(
            HANDLE_HSENS_SERVICE_CHAR_CFG_DESC,
            GATT_UUID_CHAR_CLIENT_CONFIG,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),

    /* Declare characteristic Hello Configuration */
    CHARACTERISTIC_UUID128_WRITABLE(
            HANDLE_HSENS_SERVICE_CHAR_COLOR,
            HANDLE_HSENS_SERVICE_CHAR_COLOR_VAL,
            UUID_HELLO_CHARACTERISTIC_CONFIG,
            LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD |
            LEGATTDB_PERM_WRITE_REQ),

    /* Declare Device info service */
    PRIMARY_SERVICE_UUID16(HANDLE_HSENS_DEV_INFO_SERVICE,
                           UUID_SERVCLASS_DEVICE_INFO),

    /* Handle 0x4e: characteristic Manufacturer Name, handle 0x4f
       characteristic value */
    CHARACTERISTIC_UUID16(
            HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME,
            HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
            GATT_UUID_MANU_NAME, LEGATTDB_CHAR_PROP_READ,
            LEGATTDB_PERM_READABLE),

    /* Handle 0x50: characteristic Model Number, handle 0x51
       characteristic value */
    CHARACTERISTIC_UUID16(
            HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM,
            HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,
            GATT_UUID_MODEL_NUMBER_STR, LEGATTDB_CHAR_PROP_READ,
            LEGATTDB_PERM_READABLE),

    /* Handle 0x52: characteristic System ID, handle 0x53
       characteristic value */
    CHARACTERISTIC_UUID16(
            HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID,
            HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,
            GATT_UUID_SYSTEM_ID, LEGATTDB_CHAR_PROP_READ,
            LEGATTDB_PERM_READABLE),

    /* Declare Battery service */
    PRIMARY_SERVICE_UUID16(HANDLE_HSENS_BATTERY_SERVICE,
                           UUID_SERVCLASS_BATTERY),

    /* Handle 0x62: characteristic Battery Level, handle 0x63
       characteristic value */
    CHARACTERISTIC_UUID16(
            HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL,
            HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,
            GATT_UUID_BATTERY_LEVEL, LEGATTDB_CHAR_PROP_READ,
            LEGATTDB_PERM_READABLE),
};

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */

void app_ble_hello_sensor()
{
    mico_system_context_init(0);

    OLED_Init();
    /* LED Initialize */
    hsb2rgb_led_init();

    HELLO_PERIPHERAL_LOG("Hello Sensor Start");
    snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "BLE starting ...");
    OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_1, oled_show_line);
    hsb2rgb_led_open(0, 100, 50);

    /* Register call back and configuration with stack */
    mico_bt_stack_init(hello_sensor_management_callback,
                       &mico_bt_cfg_settings_peripheral,
                       mico_bt_cfg_buf_pools_peripheral);
}

/*
 * This function is executed in the BTM_ENABLED_EVT management callback.
 */
static void hello_sensor_application_init()
{
    mico_bt_gatt_status_t gatt_status;
    mico_bt_result_t      result;

    HELLO_PERIPHERAL_LOG("hello_sensor_application_init");
    /* Started */
    memset(oled_show_line, 0, sizeof(oled_show_line));
    snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "BLE Hello Sensor");
    OLED_Clear();
    OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_1, oled_show_line);

    /* Register with stack to receive GATT callback */
    gatt_status = mico_bt_gatt_register(GATT_IF_FIXED_DB_APP, hello_sensor_gatts_callback);

    HELLO_PERIPHERAL_LOG("mico_bt_gatt_register: %d", gatt_status);

    /*  Tell stack to use our GATT database */
    gatt_status = mico_bt_gatt_db_init(hello_sensor_gatt_database,
                                       sizeof(hello_sensor_gatt_database));

    HELLO_PERIPHERAL_LOG("mico_bt_gatt_db_init %d", gatt_status);

    /* Set the advertising parameters and make the device discoverable */
    hello_sensor_set_advertisement_data();

    result = mico_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);

    HELLO_PERIPHERAL_LOG("mico_bt_start_advertisements %d", result);

    /* Advertising */
    memset(oled_show_line, 0, sizeof(oled_show_line));
    snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "   ADVERTISING  ");
    OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, oled_show_line);

    /*
     * Set flag_stay_connected to remain connected after all messages are sent
     * Reset flag to 0, to disconnect
     */
    hello_sensor_state.flag_stay_connected = 1;
}

/*
 * Setup advertisement data with 16 byte UUID and device name
 */
void hello_sensor_set_advertisement_data()
{
    OSStatus err;

    mico_bt_ble_128service_t adver_services_128 = {
            .list_cmpl = FALSE,
            .uuid128 = {UUID_HELLO_SERVICE}
    };

    mico_bt_ble_advert_data_t adv_data;

    adv_data.flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    adv_data.p_services_128b = &adver_services_128;

    err = mico_bt_ble_set_advertisement_data(BTM_BLE_ADVERT_BIT_DEV_NAME |
                                             BTM_BLE_ADVERT_BIT_SERVICE_128 |
                                             BTM_BLE_ADVERT_BIT_FLAGS,
                                             &adv_data);

    HELLO_PERIPHERAL_LOG("mico_bt_ble_set_advertisement_data %d", err);
}

/*
 * This function is invoked when advertisements stop.  If we are configured to
 * stay connected,
 * disconnection was caused by the peer, start low advertisements, so that peer
 * can connect
 * when it wakes up
 */
void hello_sensor_advertisement_stopped()
{
    mico_bt_result_t result;

    if (hello_sensor_state.flag_stay_connected && !hello_sensor_state.conn_id) {
        result = mico_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);
        HELLO_PERIPHERAL_LOG("mico_bt_start_advertisements: %d", result);
    } else {
        HELLO_PERIPHERAL_LOG("ADV stop");

        memset(oled_show_line, 0, sizeof(oled_show_line));
        snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "BLE Hello Sensor");
        OLED_Clear();
        OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_1, oled_show_line);

        /* ADV ed */
        memset(oled_show_line, 0, sizeof(oled_show_line));
        snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "   ADVERTISED   ");
        OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, oled_show_line);
    }
}

/*
 * hello_sensor bt/ble link management callback
 */
static mico_bt_result_t
hello_sensor_management_callback(mico_bt_management_evt_t event,
                                 mico_bt_management_evt_data_t *p_event_data)
{
    mico_bt_result_t result = MICO_BT_SUCCESS;
    mico_bt_dev_ble_pairing_info_t *p_info;
    mico_bt_ble_advert_mode_t *p_mode;

    HELLO_PERIPHERAL_LOG("hello_sensor_management_callback: %x", event);

    switch (event) {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            hello_sensor_application_init();
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

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            p_mode = &p_event_data->ble_advert_state_changed;
            HELLO_PERIPHERAL_LOG("Advertisement State Change: %d", *p_mode);
            if (*p_mode == BTM_BLE_ADVERT_OFF) {
                hello_sensor_advertisement_stopped();
            }
            break;

        default:
            break;
    }

    return result;
}


/*
 * Find attribute description by handle
 */
static attribute_t *hello_sensor_get_attribute(uint16_t handle)
{
    size_t i;
    for (i = 0; i < sizeof(gatt_user_attributes) / sizeof(gatt_user_attributes[0]); i++) {
        if (gatt_user_attributes[i].handle == handle) {
            return (&gatt_user_attributes[i]);
        }
    }
    HELLO_PERIPHERAL_LOG("attribute not found:%x", handle);
    return NULL;
}

/*
 * Process Read request or command from peer device
 */
static mico_bt_gatt_status_t
hello_sensor_gatt_server_read_request_handler(uint16_t              conn_id,
                                              mico_bt_gatt_read_t  *p_read_data)
{
    attribute_t *puAttribute;
    int attr_len_to_copy;

    if ((puAttribute = hello_sensor_get_attribute(p_read_data->handle)) == NULL) {
        HELLO_PERIPHERAL_LOG("read_hndlr attr not found hdl:%x", p_read_data->handle);
        return MICO_BT_GATT_INVALID_HANDLE;
    }

    /* Dummy battery value read increment */
    if (p_read_data->handle == HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL) {
        if (hello_sensor_state.battery_level++ > 99) {
            hello_sensor_state.battery_level = 0;
        }
    }

    if (p_read_data->handle == HANDLE_HSENS_SERVICE_CHAR_COLOR_VAL) {
        puAttribute->p_attr = &hello_sensor_write;
        puAttribute->attr_len = sizeof(hello_sensor_write);
    }
    attr_len_to_copy = puAttribute->attr_len;

    HELLO_PERIPHERAL_LOG("read_hndlr conn_id:%d hdl:%x offset:%d len:%d",
                         conn_id, p_read_data->handle, p_read_data->offset,
                         attr_len_to_copy);

    if (p_read_data->offset >= puAttribute->attr_len) {
        attr_len_to_copy = 0;
    }

    if (attr_len_to_copy != 0) {
        uint8_t *from;
        int to_copy = attr_len_to_copy - p_read_data->offset;

        if (to_copy > *p_read_data->p_val_len) {
            to_copy = *p_read_data->p_val_len;
        }

        from = ((uint8_t *)puAttribute->p_attr) + p_read_data->offset;
        *p_read_data->p_val_len = (uint16_t)to_copy;

        memcpy(p_read_data->p_val, from, (size_t)to_copy);
    }

    return MICO_BT_GATT_SUCCESS;
}

/*
 * Process write request or write command from peer device
 */
static mico_bt_gatt_status_t
hello_sensor_gatt_server_write_request_handler(uint16_t conn_id,
                                               mico_bt_gatt_write_t *p_data)
{
    mico_bt_gatt_status_t   result = MICO_BT_GATT_SUCCESS;
    uint8_t                 *p_attr = p_data->p_val;
    uint8_t                 attribute_value = *(uint8_t *)p_data->p_val;

    HELLO_PERIPHERAL_LOG("write_handler: conn_id:%d hdl:0x%x prep:%d offset:%d len:%d ", conn_id,
                         p_data->handle, p_data->is_prep, p_data->offset, p_data->val_len);

    switch (p_data->handle) {
        /* By writing into Characteristic Client Configuration descriptor
         * peer can enable or disable notification or indication
         */
        case HANDLE_HSENS_SERVICE_CHAR_CFG_DESC:
            if (p_data->val_len != 2) {
                return MICO_BT_GATT_INVALID_ATTR_LEN;
            }
            hello_sensor_hostinfo.characteristic_client_configuration = p_attr[0] | (p_attr[1] << 8);
            break;

        case HANDLE_HSENS_SERVICE_CHAR_COLOR_VAL:
            if (p_data->val_len != 1) {
                return MICO_BT_GATT_INVALID_ATTR_LEN;
            }
            hello_sensor_hostinfo.color_idx = p_attr[0];
            if (hello_sensor_hostinfo.color_idx != 0) {
                HELLO_PERIPHERAL_LOG("hello_sensor_write_handler:LED Color: %d",
                                     hello_sensor_hostinfo.color_idx);
            }

            /* Read RSSI */
            mico_bt_dev_read_rssi(hello_sensor_hostinfo.bdaddr, BT_TRANSPORT_LE, hello_sensor_rssi_callback);

            /* Change LED Color */
            hsb2rgb_led_open((hello_sensor_hostinfo.color_idx * 60) % 360, 100, 50);
            /* State Changed */
            memset(oled_show_line, 0, sizeof(oled_show_line));
            snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "Color: 0x%02x",
                     hello_sensor_hostinfo.color_idx);
            OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_4, oled_show_line);

            hello_sensor_write = attribute_value;
            HELLO_PERIPHERAL_LOG("The value written is: %i", attribute_value);
            break;

        default:
            result = MICO_BT_GATT_INVALID_HANDLE;
            break;
    }
    return result;
}

/*
 * Write Execute Procedure
 */
static mico_bt_gatt_status_t
hello_sensor_gatt_server_write_and_execute_request_handler(uint16_t conn_id,
                                                           mico_bt_gatt_exec_flag_t exec_flag)
{
    HELLO_PERIPHERAL_LOG("write exec: flag:%d", exec_flag);
    return MICO_BT_GATT_SUCCESS;
}

/*
 * Process MTU request from the peer
 */
static mico_bt_gatt_status_t
hello_sensor_gatt_server_mtu_request_handler(uint16_t conn_id, uint16_t mtu)
{
    HELLO_PERIPHERAL_LOG("req_mtu: %d", mtu);
    return MICO_BT_GATT_SUCCESS;
}

static mico_bt_gatt_status_t
hello_sensor_gatt_server_confirmation_handler(uint16_t conn_id,
                                              uint16_t handle)
{
    HELLO_PERIPHERAL_LOG("hello_sensor_indication_confirmation, conn %d hdl %d", conn_id, handle);

    if (!hello_sensor_state.flag_indication_sent) {
        HELLO_PERIPHERAL_LOG("Hello: Wrong Confirmation!");
        return MICO_BT_GATT_SUCCESS;
    }
    hello_sensor_state.flag_indication_sent = 0;

    return MICO_BT_GATT_SUCCESS;
}

/* This function is invoked when connection is established */
static mico_bt_gatt_status_t
hello_sensor_gatts_connection_up(mico_bt_gatt_connection_status_t *p_status)
{
    mico_bt_result_t result;

    HELLO_PERIPHERAL_LOG("hello_sensor_conn_up  id:%d:", p_status->conn_id);

    /* Update the connection handler.  Save address of the connected device. */
    hello_sensor_state.conn_id = p_status->conn_id;
    memcpy(hello_sensor_state.remote_addr, p_status->bd_addr, sizeof(BD_ADDR));

    /* Stop advertising */
    result = mico_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);

    HELLO_PERIPHERAL_LOG("Stopping Advertisements%d", result);

    memcpy(hello_sensor_hostinfo.bdaddr, p_status->bd_addr, sizeof(BD_ADDR));
    hello_sensor_hostinfo.characteristic_client_configuration = 0;
    hello_sensor_hostinfo.color_idx = 0;

    /* Connected */
    memset(oled_show_line, 0, sizeof(oled_show_line));
    snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "   CONNECTED    ");
    OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, oled_show_line);

    return MICO_BT_GATT_SUCCESS;
}

/*
 * This function is invoked when connection is lost
 */
static mico_bt_gatt_status_t
hello_sensor_gatts_connection_down(mico_bt_gatt_connection_status_t *p_status)
{
    mico_bt_result_t result;

    HELLO_PERIPHERAL_LOG("connection_down  conn_id:%d reason:%d",
                         p_status->conn_id, p_status->reason);

    /* Resetting the device info */
    memset(hello_sensor_state.remote_addr, 0, 6);
    hello_sensor_state.conn_id = 0;

    memset(oled_show_line, 0, sizeof(oled_show_line));
    snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "BLE Hello Sensor");
    OLED_Clear();
    OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_1, oled_show_line);

    memset(oled_show_line, 0, sizeof(oled_show_line));

    /*
     * If we are configured to stay connected, disconnection was
     * caused by the peer, start low advertisements, so that peer
     * can connect when it wakes up
     */
    if (hello_sensor_state.flag_stay_connected) {
        result = mico_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);
        HELLO_PERIPHERAL_LOG("mico_bt_start_advertisements %d", result);
        snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "  ADVERTISING   ");
    } else {
        snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "  DISCONNECTED  ");
    }

    OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, oled_show_line);

    return MICO_BT_SUCCESS;
}

/*
 * Connection up/down event
 */
static mico_bt_gatt_status_t
hello_sensor_gatts_connection_status_handler(mico_bt_gatt_connection_status_t *p_status)
{
    is_connected = p_status->connected;
    if (p_status->connected) {
        return hello_sensor_gatts_connection_up(p_status);
    }

    return hello_sensor_gatts_connection_down(p_status);
}

/*
 * Process GATT request from the peer
 */
static mico_bt_gatt_status_t
hello_sensor_gatt_server_request_handler(mico_bt_gatt_attribute_request_t *p_data)
{
    mico_bt_gatt_status_t result = MICO_BT_GATT_INVALID_PDU;

    HELLO_PERIPHERAL_LOG("hello_sensor_gatt_server_request_handler. conn %d, type %d",
                         p_data->conn_id, p_data->request_type);

    switch (p_data->request_type) {
        case GATTS_REQ_TYPE_READ:
            result = hello_sensor_gatt_server_read_request_handler(
                    p_data->conn_id, &(p_data->data.read_req));
            break;

        case GATTS_REQ_TYPE_WRITE:
            result = hello_sensor_gatt_server_write_request_handler(
                    p_data->conn_id, &(p_data->data.write_req));
            break;

        case GATTS_REQ_TYPE_WRITE_EXEC:
            result = hello_sensor_gatt_server_write_and_execute_request_handler(
                    p_data->conn_id, p_data->data.exec_write);
            break;

        case GATTS_REQ_TYPE_MTU:
            result = hello_sensor_gatt_server_mtu_request_handler(p_data->conn_id,
                                                                  p_data->data.mtu);
            break;

        case GATTS_REQ_TYPE_CONF:
            result = hello_sensor_gatt_server_confirmation_handler(p_data->conn_id,
                                                                   p_data->data.handle);
            break;

        default:
            break;
    }
    return result;
}

/*
 * Callback for various GATT events.  As this application performs only as a
 * GATT server, some of
 * the events are ommitted.
 */
static mico_bt_gatt_status_t
hello_sensor_gatts_callback(mico_bt_gatt_evt_t event,
                            mico_bt_gatt_event_data_t *p_data)
{
    mico_bt_gatt_status_t result = MICO_BT_GATT_INVALID_PDU;

    switch (event) {
        case GATT_CONNECTION_STATUS_EVT:
            result = hello_sensor_gatts_connection_status_handler(&p_data->connection_status);
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            result = hello_sensor_gatt_server_request_handler(&p_data->attribute_request);
            break;

        default:
            break;
    }

    return result;
}

/**
 * RSSI Value
 */
static void hello_sensor_rssi_callback(void *arg)
{
    mico_bt_dev_rssi_result_t *p_rssi = (mico_bt_dev_rssi_result_t *)arg;

    if (p_rssi->status == MICO_BT_SUCCESS && p_rssi->hci_status == HCI_SUCCESS) {
        HELLO_PERIPHERAL_LOG("RSSI: %d", p_rssi->rssi);

        memset(oled_show_line, 0, sizeof(oled_show_line));
        snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "RSSI : %4d", p_rssi->rssi);
        OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_3, oled_show_line);
    }
}

/**
 * RGB LED
 */
#define H2R_MAX_RGB_val 255.0f

static float constrain(float value, float min, float max)
{
    if (value >= max)
        return max;
    if (value <= min)
        return min;
    return value;
}

static void H2R_HSBtoRGB(float hue, float sat, float bright, float *color)
{
    // constrain all input variables to expected range
    hue = constrain(hue, 0, 360);
    sat = constrain(sat, 0, 100);
    bright = constrain(bright, 0, 100);
    // define maximum value for RGB array elements
    float max_rgb_val = H2R_MAX_RGB_val;
    // convert saturation and brightness value to decimals and init r, g, b variables
    float sat_f = (float) sat / 100.0f;
    float bright_f = (float) bright / 100.0f;
    float r = 0, g = 0, b = 0;
    // If brightness is 0 then color is black (achromatic)
    // therefore, R, G and B values will all equal to 0
    if (bright <= 0) {
        color[0] = 0;
        color[1] = 0;
        color[2] = 0;
    }
    // If saturation is 0 then color is gray (achromatic)
    // therefore, R, G and B values will all equal the current brightness
    if (sat <= 0) {
        color[0] = bright_f * max_rgb_val;
        color[1] = bright_f * max_rgb_val;
        color[2] = bright_f * max_rgb_val;
    }
        // if saturation and brightness are greater than 0 then calculate
        // R, G and B values based on the current hue and brightness
    else {
        if (hue >= 0 && hue < 120) {
            float hue_primary = 1.0f - ((float) hue / 120.0f);
            float hue_secondary = (float) hue / 120.0f;
            float sat_primary = (1.0f - hue_primary) * (1.0f - sat_f);
            float sat_secondary = (1.0f - hue_secondary) * (1.0f - sat_f);
            float sat_tertiary = 1.0f - sat_f;
            r = (bright_f * max_rgb_val) * (hue_primary + sat_primary);
            g = (bright_f * max_rgb_val) * (hue_secondary + sat_secondary);
            b = (bright_f * max_rgb_val) * sat_tertiary;
        } else if (hue >= 120 && hue < 240) {
            float hue_primary = 1.0f - (((float) hue - 120.0f) / 120.0f);
            float hue_secondary = ((float) hue - 120.0f) / 120.0f;
            float sat_primary = (1.0f - hue_primary) * (1.0f - sat_f);
            float sat_secondary = (1.0f - hue_secondary) * (1.0f - sat_f);
            float sat_tertiary = 1.0f - sat_f;
            r = (bright_f * max_rgb_val) * sat_tertiary;
            g = (bright_f * max_rgb_val) * (hue_primary + sat_primary);
            b = (bright_f * max_rgb_val) * (hue_secondary + sat_secondary);
        } else if (hue >= 240 && hue <= 360) {
            float hue_primary = 1.0f - (((float) hue - 240.0f) / 120.0f);
            float hue_secondary = ((float) hue - 240.0f) / 120.0f;
            float sat_primary = (1.0f - hue_primary) * (1.0f - sat_f);
            float sat_secondary = (1.0f - hue_secondary) * (1.0f - sat_f);
            float sat_tertiary = 1.0f - sat_f;
            r = (bright_f * max_rgb_val) * (hue_secondary + sat_secondary);
            g = (bright_f * max_rgb_val) * sat_tertiary;
            b = (bright_f * max_rgb_val) * (hue_primary + sat_primary);
        }
        color[0] = r;
        color[1] = g;
        color[2] = b;
    }
}

/*----------------------------------------------------- INTERNAL FUNCTION  ---------------------------------------*/

// call RGB LED driver to control LED
static void OpenLED_RGB(float *color)
{
    uint8_t blue = (uint8_t) (color[2]);
    uint8_t green = (uint8_t) (color[1]);
    uint8_t red = (uint8_t) (color[0]);

    LED_R = red / H2R_MAX_RGB_val;
    LED_G = green / H2R_MAX_RGB_val;
    LED_B = blue / H2R_MAX_RGB_val;
}

static void CloseLED_RGB()
{
    LED_R = 0.0;
    LED_G = 0.0;
    LED_B = 0.0;
}


/*----------------------------------------------------- USER INTERFACES ---------------------------------------*/

void hsb2rgb_led_init()
{
    LED_R.period(0.001);
    LED_G.period(0.001);
    LED_B.period(0.001);
}

void hsb2rgb_led_open(float hues, float saturation, float brightness)
{
    float color[3] = {0};
    H2R_HSBtoRGB(hues, saturation, brightness, color);
    OpenLED_RGB(color);
}

void hsb2rgb_led_close()
{
    CloseLED_RGB();
}
