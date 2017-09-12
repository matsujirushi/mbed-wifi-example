/*
 * Broadcom Proprietary and Confidential. Copyright 2016 Broadcom
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */

/** @file
 *
 * BLE Proximity Reporter Sample Application
 *
 */

#include "mico_bt_cfg.h"
#include "mico_bt_gatt_db.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define ble_proximity_log(fmt, ...) custom_log("PROXMITY", fmt, ##__VA_ARGS__);

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
static void ble_proximity_tx_power_callback(mico_bt_tx_power_result_t *p_tx_power);

static mico_bt_gatt_status_t ble_proximity_gatt_write_request(mico_bt_gatt_write_t *p_write_request);

static mico_bt_gatt_status_t ble_proximity_gatt_read_request(mico_bt_gatt_read_t *p_read_request);

static mico_bt_gatt_status_t
ble_proximity_gatt_cback(mico_bt_gatt_evt_t event, mico_bt_gatt_event_data_t *p_event_data);

/******************************************************
 *               Variable Definitions
 ******************************************************/
extern mico_bt_cfg_settings_t mico_bt_cfg_settings;

/* Proximity reporter attribute values */
static uint8_t proximity_immediate_alert_level;
static uint8_t proximity_link_loss_alert_level;
static int8_t proximity_tx_power_level;

/* GATT attribute values */
static uint32_t proximity_gatt_attribute_service_changed = 0;
static uint16_t proximity_gatt_generic_access_appearance = 0;

/******************************************************
 *               Function Definitions
 ******************************************************/

/* TX Power report handler */
static void ble_proximity_tx_power_callback(mico_bt_tx_power_result_t *p_tx_power)
{
    if ((p_tx_power->status == MICO_BT_SUCCESS) && (p_tx_power->hci_status == HCI_SUCCESS)) { ble_proximity_log(
                "Local TX power: %i\n", p_tx_power->tx_power);
        proximity_tx_power_level = p_tx_power->tx_power;
    } else { ble_proximity_log("Unable to read Local TX power. (btm_status=0x%x, hci_status=0x%x)\n",
                               p_tx_power->status, p_tx_power->hci_status);
        proximity_tx_power_level = 0;
    }
}

/* Handler for attribute write requests */
static mico_bt_gatt_status_t ble_proximity_gatt_write_request(mico_bt_gatt_write_t *p_write_request)
{
    uint8_t attribute_value = *(uint8_t *) p_write_request->p_val;
    mico_bt_gatt_status_t status = MICO_BT_GATT_SUCCESS;

    switch (p_write_request->handle) {
        case HDLC_LINK_LOSS_ALERT_LEVEL_VALUE:
            proximity_link_loss_alert_level = attribute_value;ble_proximity_log(
            "Link loss alert level changed to: %i\n", attribute_value);
            break;

        case HDLC_IMMEDIATE_ALERT_LEVEL_VALUE:
            proximity_immediate_alert_level = attribute_value;ble_proximity_log("Proximity alert (level: %i)\n",
                                                                                attribute_value);
            break;

        default:ble_proximity_log("Write request to invalid handle: 0x%x\n", p_write_request->handle);
            status = MICO_BT_GATT_WRITE_NOT_PERMIT;
            break;
    }
    return (status);
}

/* Handler for attribute read requests */
static mico_bt_gatt_status_t ble_proximity_gatt_read_request(mico_bt_gatt_read_t *p_read_request)
{
    mico_bt_gatt_status_t status = MICO_BT_GATT_SUCCESS;
    void *p_attribute_value_source;
    uint16_t attribute_value_length = 0;

    switch (p_read_request->handle) {
        case HDLC_GENERIC_ATTRIBUTE_SERVICE_CHANGED_VALUE:
            p_attribute_value_source = &proximity_gatt_attribute_service_changed;
            attribute_value_length = sizeof(proximity_gatt_attribute_service_changed);
            break;

        case HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE:
            p_attribute_value_source = (void *) mico_bt_cfg_settings.device_name;
            attribute_value_length = (uint16_t)strlen((char *) mico_bt_cfg_settings.device_name);
            ble_proximity_log("BT NAME: %s \r\n", ((char *) p_attribute_value_source));
            break;

        case HDLC_GENERIC_ACCESS_APPEARANCE_VALUE:
            p_attribute_value_source = &proximity_gatt_generic_access_appearance;
            attribute_value_length = sizeof(proximity_gatt_generic_access_appearance);
            break;

        case HDLC_TX_POWER_LEVEL_VALUE:
            p_attribute_value_source = &proximity_tx_power_level;
            attribute_value_length = sizeof(proximity_tx_power_level);
            break;

        case HDLC_LINK_LOSS_ALERT_LEVEL_VALUE:
            p_attribute_value_source = &proximity_link_loss_alert_level;
            attribute_value_length = sizeof(proximity_link_loss_alert_level);
            break;

        default:
            status = MICO_BT_GATT_READ_NOT_PERMIT;ble_proximity_log("Read request to invalid handle: 0x%x\n",
                                                                    p_read_request->handle);
            break;
    }
    /* Validate destination buffer size */
    if (attribute_value_length > *p_read_request->p_val_len) {
        *p_read_request->p_val_len = attribute_value_length;
    }
    /* Copy the attribute value */
    if (attribute_value_length) {
        memcpy(p_read_request->p_val, p_attribute_value_source, attribute_value_length);
    }
    /* Indicate number of bytes copied */
    *p_read_request->p_val_len = attribute_value_length;
    return (status);
}

/* GATT event handler */
static mico_bt_gatt_status_t ble_proximity_gatt_cback(mico_bt_gatt_evt_t event, mico_bt_gatt_event_data_t *p_event_data)
{
    mico_bt_gatt_status_t status = MICO_BT_GATT_SUCCESS;
    uint8_t *bda;

    switch (event) {
        case GATT_CONNECTION_STATUS_EVT:
            /* GATT connection status change */
            bda = p_event_data->connection_status.bd_addr;ble_proximity_log(
            "GATT connection to [%02X:%02X:%02X:%02X:%02X:%02X] %s.\n",
            bda[0], bda[1], bda[2], bda[3], bda[4], bda[5],
            (p_event_data->connection_status.connected ? "established" : "released"));

            if (p_event_data->connection_status.connected) {
                /* Connection established. Get current TX power  (required for setting TX power attribute in GATT database) */
                mico_bt_dev_read_tx_power(p_event_data->connection_status.bd_addr,
                                          p_event_data->connection_status.transport,
                                          (mico_bt_dev_cmpl_cback_t *) ble_proximity_tx_power_callback);

                /* Disable connectability. */
                mico_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
            } else {
                /* Connection released. Re-enable BLE connectability. */
                mico_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
                /* test code for directed adv */
                /* mico_bt_start_advertisements (BTM_BLE_ADVERT_DIRECTED_HIGH, 0, p_event_data->connection_status.bd_addr); */
                ble_proximity_log("Waiting for proximity monitor to connect...\n");
            }
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            /* GATT attribute read/write request */
            if (p_event_data->attribute_request.request_type == GATTS_REQ_TYPE_WRITE) {
                status = ble_proximity_gatt_write_request(&p_event_data->attribute_request.data.write_req);
            } else if (p_event_data->attribute_request.request_type == GATTS_REQ_TYPE_READ) {
                status = ble_proximity_gatt_read_request(&p_event_data->attribute_request.data.read_req);

            }
            break;

        default:
            break;
    }

    return (status);
}

/* Initialize Proximity Reporter */
void ble_proximity_reporter_init(void)
{
    mico_bt_ble_advert_data_t adv_elem;

    adv_elem.flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    mico_bt_ble_set_advertisement_data(BTM_BLE_ADVERT_BIT_FLAGS | BTM_BLE_ADVERT_BIT_DEV_NAME,
                                       &adv_elem);

    /* Register for gatt event notifications */
    mico_bt_gatt_register(GATT_IF_FIXED_DB_APP, &ble_proximity_gatt_cback);

    /* Initialize GATT database */
    mico_bt_gatt_db_init((uint8_t *) gatt_db, gatt_db_size);

    /* Enable Bluetooth LE advertising and connectability */

    /* start LE advertising */
    mico_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);ble_proximity_log(
            "Waiting for proximity monitor to connect...\n");
}
