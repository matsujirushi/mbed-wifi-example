/**
 ******************************************************************************
 * @file    ble_hello_sensor.c
 * @author  William Xu
 * @version V1.0.0
 * @date    17-Feb-2016
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
 * BLE Vendor Specific Device
 *
 * Features demonstrated
 *  - GATT database and Device configuration initialization
 *  - Registration with LE stack for various events
 *  - Sending data to the client
 *  - Processing write requests from the client
 *
 * To demonstrate the app, work through the following steps.
 * 1. Plug the MiCOKit board into your computer
 * 2. Build and download the application (to the MiCOKit board)
 * 3. On application start the device acts as a GATT server and advertises itself as Hello Sensor
 * 4. Connect to GATT server using one of the LE clients (BlueDeng(android)) or (LightBlue(Apple Store))
 * 5. Once connected the client can read Hello sensor characteristics
 * 6. Write the hello sensor characteristic configuration value from client
 * 7. on the console terminal you can see the value entered in the client
 * 8. To receive notification the user has to register for notification after LE connection on the client
 * 9. To test notifications user can enter the command "notify" on the console
 * 10. Each time when user enters the notify command a new value of notification can be seen on the LE client.
 *
 * Important Notice
 * SPP service is to deliver data between APP_UART and BLE interface.
 * EMW3238 module has only one user UART and configured for debug info in default, to use this UART as
 * APP_UART and acquire its data properly, two macro should be changed
 *
 ******************************************************************************
 **/

#include "mico.h"
#include "ble_hello_sensor.h"
#include "sdpdefs.h"
#include "mico_bt_cfg.h"
#include "mico_bt.h"
#include "mico_bt_peripheral.h"

#include "oled.h"

#define hello_sensor_log(M, ...) custom_log("LE Sensor", M, ##__VA_ARGS__)


/******************************************************************************
 *                                Constants
******************************************************************************/

/******************************************************************************
 *                           Function Prototypes
 ******************************************************************************/

static void hello_sensor_send_notification(CLI_ARGS);

static void hello_sensor_send_indication(CLI_ARGS);

/******************************************************************************
 *                                Structures
 ******************************************************************************/

typedef struct {
    uint8_t flag_stay_connected;    /* stay connected or disconnect after all messages are sent */
    uint8_t battery_level;          /* dummy battery level */
    uint8_t led_color_idx;          /* RGB led color index */
} hello_sensor_state_t;

typedef struct {
    mico_bt_device_address_t bdaddr;
    uint16_t service_changed;                        /* BD address of the bonded host */
    uint16_t characteristic_client_configuration;    /* Current value of the client configuration descriptor */
} host_info_t;

/* Peripheral auto advertising settings */
static mico_bt_smart_advertising_settings_t advertising_settings = {
    .type                     =  BT_SMART_UNDIRECTED_ADVERTISING,                 /**< Advertising type                                               */
    .use_high_duty            =  MICO_TRUE,                                       /**< Using high duty to start advertising                           */
    .high_duty_interval       =  MICO_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MIN_INTERVAL,  /**< High duty advertising interval                                 */
    .high_duty_duration       =  5,                                               /**< High duty advertising duration in seconds (0 for infinite)     */
    .low_duty_interval        =  MICO_BT_CFG_DEFAULT_LOW_DUTY_ADV_MIN_INTERVAL,   /**< Low duty advertising interval                                  */
    .low_duty_duration        =  60,                                              /**< Low duty advertising duration in seconds (0 for infinite)      */
};

/* Peripheral security settings */
static const mico_bt_smart_security_settings_t security_settings = {
    .timeout_second              = 15,
    .io_capabilities             = BT_SMART_IO_NO_INPUT_NO_OUTPUT,
    .authentication_requirements = BT_SMART_AUTH_REQ_BONDING,
    .oob_authentication          = BT_SMART_OOB_AUTH_NONE,
    .max_encryption_key_size     = 16,
    .master_key_distribution     = BT_SMART_DISTRIBUTE_ALL_KEYS,
    .slave_key_distribution      = BT_SMART_DISTRIBUTE_ALL_KEYS,
};

struct cli_command hello_sensor_send_message_cmd[] = {
    {"notify",   "Send Notification", hello_sensor_send_notification},
    {"indicate", "Send Indication",   hello_sensor_send_indication},
};


/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/

/* Initialized attribute value */
static uint8_t hello_sensor_appearance_name[2]      = { BIT16_TO_8(APPEARANCE_GENERIC_TAG) };
static uint8_t hello_sensor_char_system_id_value[]  = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };
static char    hello_sensor_char_disable_value[]    = { 'D', 'I', 'S', 'A', 'B', 'L', 'E', 'D' };
static char    hello_sensor_char_notify_value[]     = { 'H', 'E', 'L', 'L', 'O', '0' };
static char    hello_sensor_char_indicate_value[]   = { 'I', 'N', 'D', 'I', 'C', 'A', 'T', 'E', '0' };

static hello_sensor_state_t hello_sensor_state;
static host_info_t hello_sensor_hostinfo;

/* Attributes used to send notify or indicate to le client */
static mico_bt_ext_attribute_value_t *hello_notify_indicate_attribute = NULL;

/* MiCO BT smart peripheral connection controller */
static mico_bt_peripheral_socket_t peripheral_socket;

static char oled_show_line[OLED_DISPLAY_MAX_CHAR_PER_ROW + 1] = { '\0' };   // max char each line

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
    PRIMARY_SERVICE_UUID16(HDLS_GENERIC_ATTRIBUTE, UUID_SERVCLASS_GATT_SERVER),

    CHARACTERISTIC_UUID16(HDLC_GENERIC_ATTRIBUTE_SERVICE_CHANGED,
                          HDLC_GENERIC_ATTRIBUTE_SERVICE_CHANGED_VALUE,
                          GATT_UUID_GATT_SRV_CHGD,
                          LEGATTDB_CHAR_PROP_INDICATE,
                          LEGATTDB_PERM_NONE),

    /* Declare mandatory GAP service. Device Name and Appearance are mandatory
     * characteristics of GAP service                                        */
    PRIMARY_SERVICE_UUID16(HDLS_GENERIC_ACCESS, UUID_SERVCLASS_GAP_SERVER),

    /* Declare mandatory GAP service characteristic: Dev Name */
    CHARACTERISTIC_UUID16(HDLC_GENERIC_ACCESS_DEVICE_NAME, HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE,
                          GATT_UUID_GAP_DEVICE_NAME,
                          LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),

    /* Declare mandatory GAP service characteristic: Appearance */
    CHARACTERISTIC_UUID16(HDLC_GENERIC_ACCESS_APPEARANCE, HDLC_GENERIC_ACCESS_APPEARANCE_VALUE,
                          GATT_UUID_GAP_ICON,
                          LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),

    /* Declare proprietary Hello Service with 128 byte UUID */
    PRIMARY_SERVICE_UUID128(HDLS_HELLO_SENSOR, UUID_HELLO_SERVICE),

    /* Declare characteristic used to notify/indicate change */
    CHARACTERISTIC_UUID128(HDLC_HELLO_SENSOR_NOTIFY, HDLC_HELLO_SENSOR_NOTIFY_VALUE,
                           UUID_HELLO_CHARACTERISTIC_NOTIFY,
                           LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY |
                           LEGATTDB_CHAR_PROP_INDICATE,
                           LEGATTDB_PERM_READABLE),

    /* Declare client characteristic configuration descriptor
     * Value of the descriptor can be modified by the client
     * Value modified shall be retained during connection and across connection
     * for bonded devices.  Setting value to 1 tells this application to send notification
     * when value of the characteristic changes.  Value 2 is to allow indications. */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLC_HELLO_SENSOR_NOTIFY_CFG_DESC, GATT_UUID_CHAR_CLIENT_CONFIG,
                                    LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),

    /* Declare characteristic Hello Configuration */
    CHARACTERISTIC_UUID128_WRITABLE(HDLC_HELLO_SENSOR_COLOR, HDLC_HELLO_SENSOR_COLOR_VALUE,
                                    UUID_HELLO_CHARACTERISTIC_COLOR,
                                    LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
                                    LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD |
                                    LEGATTDB_PERM_WRITE_REQ),

    /* Declare Device info service */
    PRIMARY_SERVICE_UUID16(HDLS_DEV_INFO, UUID_SERVCLASS_DEVICE_INFO),

    /* Handle 0x4e: characteristic Manufacturer Name */
    CHARACTERISTIC_UUID16(HDLC_DEV_INFO_MFR_NAME, HDLC_DEV_INFO_MFR_NAME_VALUE,
                          GATT_UUID_MANU_NAME,
                          LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),

    /* Handle 0x50: characteristic Model Number */
    CHARACTERISTIC_UUID16(HDLC_DEV_INFO_MODEL_NUM, HDLC_DEV_INFO_MODEL_NUM_VALUE,
                          GATT_UUID_MODEL_NUMBER_STR,
                          LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),

    /* Handle 0x52: characteristic System ID */
    CHARACTERISTIC_UUID16(HDLC_DEV_INFO_SYSTEM_ID, HDLC_DEV_INFO_SYSTEM_ID_VALUE,
                          GATT_UUID_SYSTEM_ID,
                          LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),

    /* Declare Battery service */
    PRIMARY_SERVICE_UUID16(HDLS_BAT, UUID_SERVCLASS_BATTERY),

    /* Handle 0x62: characteristic Battery Level, handle 0x63 characteristic value */
    CHARACTERISTIC_UUID16(HDLC_BAT_LEVEL, HDLC_BAT_LEVEL_VALUE,
                          GATT_UUID_BATTERY_LEVEL,
                          LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),
};

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

static void hello_sensor_create_attribute_db();

/* Peripheral connection handlers */
static OSStatus connection_handler(mico_bt_peripheral_socket_t *socket);

static OSStatus disconnection_handler(mico_bt_peripheral_socket_t *socket);

static OSStatus hello_sensor_set_advertisement_data();

static OSStatus advertisement_complete_handle(void *arg);

/* Peripheral attribute operation handlers */
static mico_bt_gatt_status_t
battery_level_callback(mico_bt_ext_attribute_value_t *attribute, mico_bt_gatt_request_type_t op);

static mico_bt_gatt_status_t
color_val_callback(mico_bt_ext_attribute_value_t *attribute, mico_bt_gatt_request_type_t op);

static mico_bt_gatt_status_t
notification_char_callback(mico_bt_ext_attribute_value_t *attribute, mico_bt_gatt_request_type_t op);

static mico_bt_gatt_status_t
char_cfg_callback(mico_bt_ext_attribute_value_t *attribute, mico_bt_gatt_request_type_t op);

/*
 *  Entry point to the application.
 */

int app_ble_hello_sensor()
{
    OSStatus err = kNoErr;

    OLED_Init();

    /* mico system initialize */
    mico_system_context_init(0);

    /* Starting ... */
    hello_sensor_log("ble_hello_sensor initialising...");
    snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "BLE starting ...");
    OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_1, oled_show_line);

    /* Initialise MiCO Bluetooth Framework */
    err = mico_bt_init(MICO_BT_HCI_MODE, DEFAULT_NAME "_SENSOR", 0, 1);  //Client + server connections
    require_noerr_string(err, exit, "Error initialising MiCO Bluetooth Framework");

    /* Initialise MiCO Bluetooth Peripheral interface */
    mico_bt_peripheral_init(&peripheral_socket, &security_settings, connection_handler, disconnection_handler, NULL);

    /* Build BT stack layer GATT database (handle, uuid, permission, properity)*/
    mico_bt_gatt_db_init(hello_sensor_gatt_database, sizeof(hello_sensor_gatt_database));

    /* Build BT application layer GATT database ( extenal value, callback functions )*/
    hello_sensor_create_attribute_db();

    hello_sensor_log("ble_hello_sensor initialised");

    /* Started */
    memset(oled_show_line, 0, sizeof(oled_show_line));
    snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "BLE Hello Sensor");
    OLED_Clear();
    OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_1, oled_show_line);

    /* Set the advertising parameters and make the device discoverable */
    hello_sensor_set_advertisement_data();
    mico_bt_peripheral_start_advertisements(&advertising_settings, advertisement_complete_handle);

    /* Advertising */
    memset(oled_show_line, 0, sizeof(oled_show_line));
    snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s",   "   ADVERTISING  ");
    OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, oled_show_line);

    /*
     * Set flag_stay_connected to remain connected after all messages are sent
     * Reset flag to 0, to disconnect
     */
    hello_sensor_state.flag_stay_connected = 1;
    hello_sensor_state.battery_level = 0;
    hello_sensor_state.led_color_idx = 0;

#ifdef USE_MiCOKit_EXT
    hsb2rgb_led_open((hello_sensor_state.led_color_idx * 60) % 360 , 100, 5);
#endif

#ifdef MICO_CLI_ENABLE
    cli_register_commands(hello_sensor_send_message_cmd, 2);
#endif

exit:
    return err;
}


OSStatus hello_sensor_set_advertisement_data()
{
    OSStatus err = kNoErr;

    uint16_t uuid[1] = { UUID_SERVCLASS_BATTERY };

    mico_bt_ble_service_t adver_services_16 = {
            .num_service = 1,
            .list_cmpl = false,
            .p_uuid = uuid
    };

    mico_bt_ble_128service_t adver_services_128 = {
            .list_cmpl = false,
            .uuid128 = { UUID_HELLO_SERVICE }
    };

    mico_bt_ble_advert_data_t adv_data;

    adv_data.flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    adv_data.p_services = &adver_services_16;
    adv_data.p_services_128b = &adver_services_128;

    mico_bt_ble_set_advertisement_data(BTM_BLE_ADVERT_BIT_DEV_NAME |
                                       BTM_BLE_ADVERT_BIT_SERVICE |
                                       BTM_BLE_ADVERT_BIT_FLAGS, 
                                       &adv_data);

    mico_bt_ble_set_scan_response_data(BTM_BLE_ADVERT_BIT_SERVICE_128, &adv_data);

    return err;
}

/*
 * This function is invoked when advertisements changed.  If we are configured to stay connected,
 * disconnection was caused by the peer, start low advertisements, so that peer can connect
 * when it wakes up
 */

OSStatus advertisement_complete_handle(void *arg)
{
    UNUSED_PARAMETER(arg);
    OSStatus result = kNoErr;
    mico_bt_peripheral_socket_status_t status;

    mico_bt_peripheral_get_socket_status(&peripheral_socket, &status);
    if (hello_sensor_state.flag_stay_connected && status == PERIPHERAL_SOCKET_DISCONNECTED) {
        advertising_settings.use_high_duty = MICO_FALSE;
        result = mico_bt_peripheral_start_advertisements(&advertising_settings, advertisement_complete_handle);
        hello_sensor_log("mico_bt_start_advertisements: %d", result);
    } else {
        hello_sensor_log("ADV stop");

        memset(oled_show_line, 0, sizeof(oled_show_line));
        snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "BLE Sensor Demo");
        OLED_Clear();
        OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_1, oled_show_line);

        /* ADV ed */
        memset(oled_show_line, 0, sizeof(oled_show_line));
        snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "   ADVERTISED   ");
        OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, oled_show_line);
    }
    return result;
}

void hello_sensor_create_attribute_db()
{
    extern mico_bt_cfg_settings_t mico_bt_cfg_settings;

    /* Create BLE GATT value database */
    // ***** Primary service 'Generic Attribute'
    mico_bt_peripheral_ext_attribute_add(HDLC_GENERIC_ATTRIBUTE_SERVICE_CHANGED_VALUE, 0, NULL, NULL);

    // ***** Primary service 'Generic Access'
    mico_bt_peripheral_ext_attribute_add(HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE,
                                         (uint16_t)strlen((char *) mico_bt_cfg_settings.device_name),
                                         mico_bt_cfg_settings.device_name, NULL);
    mico_bt_peripheral_ext_attribute_add(HDLC_GENERIC_ACCESS_APPEARANCE_VALUE, sizeof(hello_sensor_appearance_name),
                                         hello_sensor_appearance_name, NULL);

    // ***** Primary service 'Device info'
    mico_bt_peripheral_ext_attribute_add(HDLC_DEV_INFO_MFR_NAME_VALUE, 5, (uint8_t *) "hello", NULL);
    mico_bt_peripheral_ext_attribute_add(HDLC_DEV_INFO_MODEL_NUM_VALUE, (uint16_t)strlen((char *) MODEL),
                                         (uint8_t *) MODEL, NULL);
    mico_bt_peripheral_ext_attribute_add(HDLC_DEV_INFO_SYSTEM_ID_VALUE, sizeof(hello_sensor_char_system_id_value),
                                         hello_sensor_char_system_id_value, NULL);

    // ***** Primary service 'Battery'
    mico_bt_peripheral_ext_attribute_add(HDLC_BAT_LEVEL_VALUE, 0, NULL, battery_level_callback);

    // ***** Primary service 'Hello' (Vender specific)
    hello_notify_indicate_attribute = mico_bt_peripheral_ext_attribute_add(HDLC_HELLO_SENSOR_NOTIFY_VALUE, 0, NULL,
                                                                           notification_char_callback);
    mico_bt_peripheral_ext_attribute_add(HDLC_HELLO_SENSOR_NOTIFY_CFG_DESC, 2,
                                         (uint8_t *) &hello_sensor_hostinfo.characteristic_client_configuration,
                                         char_cfg_callback);

    mico_bt_peripheral_ext_attribute_add(HDLC_HELLO_SENSOR_COLOR_VALUE, 0, NULL, color_val_callback);

    mico_bt_peripheral_ext_attribute_find_by_handle(HDLC_HELLO_SENSOR_NOTIFY_VALUE, &hello_notify_indicate_attribute);
}

/* TX Power report handler */
static void hello_sensor_tx_power_callback(mico_bt_tx_power_result_t *p_tx_power)
{
    if ((p_tx_power->status == MICO_BT_SUCCESS) && (p_tx_power->hci_status == HCI_SUCCESS)) {
        hello_sensor_log("Local TX: %d", p_tx_power->tx_power);
    } else {
        hello_sensor_log("Unable to read Local TX power. (btm_status=0x%x, hci_status=0x%x)",
                         p_tx_power->status, p_tx_power->hci_status);
    }
}

/* RSSI report handler */
static void hello_sensor_rssi_callback(mico_bt_dev_rssi_result_t *p_rssi)
{
    if ((p_rssi->status == MICO_BT_SUCCESS) && (p_rssi->hci_status == HCI_SUCCESS)) {
        hello_sensor_log("RSSI: %d", p_rssi->rssi);
        memset(oled_show_line, 0, sizeof(oled_show_line));
        snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "    RSSI: %4d  ", p_rssi->rssi);
        OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_3, oled_show_line);
    } else {
        hello_sensor_log("Unable to read RSSI. (btm_status=0x%x, hci_status=0x%x)",
                         p_rssi->status, p_rssi->hci_status);
    }
}


OSStatus connection_handler(mico_bt_peripheral_socket_t *socket)
{
    OSStatus result;

    hello_sensor_log("hello_sensor_conn_up id:0x%4x:", socket->connection_handle);

    /* Stop advertising */
    result = mico_bt_peripheral_stop_advertisements();
    hello_sensor_log("Stopping Advertisements%d", result);

    /* Connected */
    memset(oled_show_line, 0, sizeof(oled_show_line));
    snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "   CONNECTED    ");
    OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, oled_show_line);

    mico_bt_dev_read_tx_power(socket->remote_device.address,
                              BT_TRANSPORT_LE,
                              (mico_bt_dev_cmpl_cback_t *) hello_sensor_tx_power_callback);

    hello_sensor_hostinfo.characteristic_client_configuration = GATT_CLIENT_CONFIG_NONE;
    hello_sensor_hostinfo.service_changed = 1;

    return kNoErr;
}

OSStatus disconnection_handler(mico_bt_peripheral_socket_t *socket)
{
    OSStatus result;

    UNUSED_PARAMETER(socket);

    hello_sensor_log("hello_sensor_conn_down id:0x%4x:", socket->connection_handle);

    memset(oled_show_line, 0, sizeof(oled_show_line));
    snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "BLE Sensor Demo");
    OLED_Clear();
    OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_1, oled_show_line);

    memset(oled_show_line, 0, sizeof(oled_show_line));

    /*
     * If we are configured to stay connected, disconnection was
     * caused by the peer, start low advertisements, so that peer
     * can connect when it wakes up
     */
    if (hello_sensor_state.flag_stay_connected) {
        advertising_settings.use_high_duty = TRUE;
        result = mico_bt_peripheral_start_advertisements(&advertising_settings, advertisement_complete_handle);
        hello_sensor_log("mico_bt_start_advertisements %d", result);

        snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "  ADVERTISING   ");
    } else {
        snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "%s", "  DISCONNECTED  ");
    }

    OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_2, oled_show_line);

    return kNoErr;
}

/*
 * Hello Sensor functions
 */

/*
 * This function will be triggered by command "notify" on command line 
 */
void hello_sensor_send_notification(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    uint8_t *p_attr = (uint8_t *) hello_sensor_char_notify_value;
    mico_bt_peripheral_socket_status_t status;

    UNUSED_PARAMETER(argc);
    UNUSED_PARAMETER(argv);

    mico_bt_peripheral_get_socket_status(&peripheral_socket, &status);
    if (status == PERIPHERAL_SOCKET_CONNECTED) {
        if (hello_sensor_hostinfo.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION) {
            /*incrementing the value field , to ensure that notification values are changing*/
            hello_sensor_char_notify_value[5]++;
            mico_bt_peripheral_ext_attribute_value_write(hello_notify_indicate_attribute,
                                                         sizeof(hello_sensor_char_notify_value), 0, p_attr);
            mico_bt_peripheral_gatt_notify_attribute_value(&peripheral_socket, hello_notify_indicate_attribute);
            snprintf(pcWriteBuffer, xWriteBufferLen, "Notify is sent");
        } else {
            snprintf(pcWriteBuffer, xWriteBufferLen, "Client configuration not allow");
        }
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Cannot send notification, hello sensor not connected");
    }
}

/*
 * This function will be triggered by command "inticate" on command line 
 */
void hello_sensor_send_indication(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    uint8_t *p_attr = (uint8_t *) hello_sensor_char_indicate_value;
    mico_bt_peripheral_socket_status_t status;

    UNUSED_PARAMETER(argc);
    UNUSED_PARAMETER(argv);

    mico_bt_peripheral_get_socket_status(&peripheral_socket, &status);
    if (status == PERIPHERAL_SOCKET_CONNECTED) {
        if (hello_sensor_hostinfo.characteristic_client_configuration & GATT_CLIENT_CONFIG_INDICATION) {
            /*incrementing the value field , to ensure that indication values are changing*/
            hello_sensor_char_indicate_value[8]++;
            mico_bt_peripheral_ext_attribute_value_write(hello_notify_indicate_attribute,
                                                         sizeof(hello_sensor_char_indicate_value), 0, p_attr);
            mico_bt_peripheral_gatt_indicate_attribute_value(&peripheral_socket, hello_notify_indicate_attribute);
            snprintf(pcWriteBuffer, xWriteBufferLen, "Indicate is sent");
        } else {
            snprintf(pcWriteBuffer, xWriteBufferLen, "client configuration not allow");
        }
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Cannot send indication, hello sensor not connected");
    }
}


mico_bt_gatt_status_t color_val_callback(mico_bt_ext_attribute_value_t *attribute, mico_bt_gatt_request_type_t op)
{
    if (op == GATTS_REQ_TYPE_READ) {
        hello_sensor_log("hello_sensor_read_handler: led_color_idx: %d", hello_sensor_state.led_color_idx);
        mico_bt_peripheral_ext_attribute_value_write(attribute, 1, 0, &hello_sensor_state.led_color_idx);
        return MICO_BT_GATT_SUCCESS;
    } else if (op == GATTS_REQ_TYPE_WRITE) {
        if (attribute->value_length != 1) {
            return MICO_BT_GATT_INVALID_ATTR_LEN;
        }
        hello_sensor_state.led_color_idx = attribute->p_value[0];
        hello_sensor_log("hello_sensor_write_handler: led_color_idx: %d", hello_sensor_state.led_color_idx);
        mico_bt_dev_read_rssi(peripheral_socket.remote_device.address, BT_TRANSPORT_LE,
                              (mico_bt_dev_cmpl_cback_t *) hello_sensor_rssi_callback);

#ifdef USE_MiCOKit_EXT
        hsb2rgb_led_open( (hello_sensor_state.led_color_idx * 60)%360 , 100, 5);
#endif
        memset(oled_show_line, 0, sizeof(oled_show_line));
        snprintf(oled_show_line, OLED_DISPLAY_MAX_CHAR_PER_ROW + 1, "RX Value: 0x%02x", hello_sensor_state.led_color_idx);
        OLED_ShowString(OLED_DISPLAY_COLUMN_START, OLED_DISPLAY_ROW_4, oled_show_line);

        return MICO_BT_GATT_SUCCESS;
    } else {
        return MICO_BT_GATT_ERROR;
    }
}

static mico_bt_gatt_status_t
notification_char_callback(mico_bt_ext_attribute_value_t *attribute, mico_bt_gatt_request_type_t op)
{
    uint8_t *p_attr;

    if (op == GATTS_REQ_TYPE_READ) {
        /* If client has not registered for indication or notification, no action */
        if (hello_sensor_hostinfo.characteristic_client_configuration == 0) {
            p_attr = (uint8_t *) hello_sensor_char_disable_value;
            mico_bt_peripheral_ext_attribute_value_write(attribute, sizeof(hello_sensor_char_disable_value), 0, p_attr);
        } else if (hello_sensor_hostinfo.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION) {
            p_attr = (uint8_t *) hello_sensor_char_notify_value;
            mico_bt_peripheral_ext_attribute_value_write(attribute, sizeof(hello_sensor_char_notify_value), 0, p_attr);
        } else {
            p_attr = (uint8_t *) hello_sensor_char_indicate_value;
            mico_bt_peripheral_ext_attribute_value_write(attribute, sizeof(hello_sensor_char_indicate_value), 0,
                                                         p_attr);
        }
        return MICO_BT_GATT_SUCCESS;
    } else
        return MICO_BT_GATT_ERROR;
}

mico_bt_gatt_status_t char_cfg_callback(mico_bt_ext_attribute_value_t *attribute, mico_bt_gatt_request_type_t op)
{

    if (op == GATTS_REQ_TYPE_READ) {
        return MICO_BT_GATT_SUCCESS;
    } else if (op == GATTS_REQ_TYPE_WRITE) {
        if (attribute->value_length != 2) {
            return MICO_BT_GATT_INVALID_ATTR_LEN;
        }
        hello_sensor_hostinfo.characteristic_client_configuration =
                attribute->p_value[0] | (attribute->p_value[1] << 8);
        return MICO_BT_GATT_SUCCESS;
    } else {
        return MICO_BT_GATT_ERROR;
    }
}

/*
 * Demo battery functions
 */
mico_bt_gatt_status_t battery_level_callback(mico_bt_ext_attribute_value_t *attribute, mico_bt_gatt_request_type_t op)
{
    if (op == GATTS_REQ_TYPE_READ) {
        hello_sensor_state.battery_level = (hello_sensor_state.battery_level + 1) % 100;
        hello_sensor_log("Read battery level %d", hello_sensor_state.battery_level);
        mico_bt_peripheral_ext_attribute_value_write(attribute, 1, 0, &hello_sensor_state.battery_level);
        return MICO_BT_GATT_SUCCESS;
    }
    return MICO_BT_GATT_ERROR;
}
