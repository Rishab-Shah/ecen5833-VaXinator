/*
 * ble.c
 *
 *  Created on: Sep 27, 2021
 *      Author: vishn
 */

#include "ble.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"


ble_data_struct_t ble_data = { 0 };


void handle_ble_event(sl_bt_msg_t* event) {
    switch(SL_BT_MSG_ID(event->header)) {
        case sl_bt_evt_system_boot_id:
            BLE_HandleBootEvent();
            break;

        case sl_bt_evt_connection_opened_id:
            BLE_HandleConnectionOpenedEvent(event);
            break;

        case sl_bt_evt_connection_closed_id:
            BLE_HandleConnectionClosedEvent();
            break;

        case sl_bt_evt_connection_parameters_id:
            BLE_HandleConnectionParametersEvent(event);
            break;

        case sl_bt_evt_system_external_signal_id:
            BLE_HandleExternalSignalEvent();
            break;

        case sl_bt_evt_gatt_server_characteristic_status_id:
            BLE_HandleCharacteristicStatusEvent(event);
            break;

        case sl_bt_evt_gatt_server_indication_timeout_id:
            BLE_HandleIndicationTimeoutEvent();
            break;

    }
}


ble_data_struct_t* BLE_GetDataStruct(void) {
    return (&ble_data);
}


void BLE_HandleBootEvent(void) {
    sl_status_t ble_status;

    // Get unique id
    ble_status = sl_bt_system_get_identity_address(&(ble_data.address), &(ble_data.myAddressType));
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_system_get_identity_address: %d\r\n", ble_status);
    }

    // Pad and reverse unique ID to get System ID.
    ble_data.systemId[0] = ble_data.address.addr[5];
    ble_data.systemId[1] = ble_data.address.addr[4];
    ble_data.systemId[2] = ble_data.address.addr[3];
    ble_data.systemId[3] = 0xFF;
    ble_data.systemId[4] = 0xFE;
    ble_data.systemId[5] = ble_data.address.addr[2];
    ble_data.systemId[6] = ble_data.address.addr[1];
    ble_data.systemId[7] = ble_data.address.addr[0];

    ble_status = sl_bt_gatt_server_write_attribute_value(gattdb_system_id, 0, sizeof(ble_data.systemId), ble_data.systemId);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_gatt_server_write_attribute_value: %d\r\n", ble_status);
    }

    // Create advertising set
    ble_status = sl_bt_advertiser_create_set(&(ble_data.advertisingSetHandle));
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_advertiser_create_set: %d\r\n", ble_status);
    }

    ble_status = sl_bt_advertiser_set_timing(ble_data.advertisingSetHandle, 400, 400, 0, 0);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_advertiser_set_timing: %d\r\n", ble_status);
    }

    // Start advertising
    ble_status = sl_bt_advertiser_start(ble_data.advertisingSetHandle, sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_advertiser_start: %d\r\n", ble_status);
    }
}


void BLE_HandleConnectionOpenedEvent(sl_bt_msg_t* event) {
    sl_status_t ble_status;

    // Modify ble_data variables
    ble_data.connectionHandle = event->data.evt_connection_opened.connection;
    ble_data.connected = true;

    // Stop advertising
    ble_status = sl_bt_advertiser_stop(ble_data.advertisingSetHandle);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_advertiser_stop: %d\r\n", ble_status);
    }

    // Set connection parameters
    ble_status = sl_bt_connection_set_parameters(ble_data.connectionHandle, 120, 120, 4, 1500, 0, 0xFFFF);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_connection_set_parameters: %d\r\n", ble_status);
    }
}


void BLE_HandleConnectionClosedEvent(void) {
    sl_status_t ble_status;

    // Modify ble_data variables
    ble_data.connected = false;
    ble_data.indicating = false;
    ble_data.notFirstConnection = false;

    ble_status = sl_bt_advertiser_start(ble_data.advertisingSetHandle, sl_bt_advertiser_general_discoverable,
            sl_bt_advertiser_connectable_scannable);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_advertiser_start: %d\r\n", ble_status);
    }
}


void BLE_HandleConnectionParametersEvent(sl_bt_msg_t* event) {
    LOG_INFO("handle: %d, interval, %d, latency: %d, timeout: %d\r\n",
             event->data.evt_connection_parameters.connection,
             event->data.evt_connection_parameters.interval,
             event->data.evt_connection_parameters.latency,
             event->data.evt_connection_parameters.timeout);
}


void BLE_HandleExternalSignalEvent(void) {
    return;
}


void BLE_HandleCharacteristicStatusEvent(sl_bt_msg_t* event) {
    uint8_t status_flags;
    uint16_t client_flags;

    ble_data.tempCharacteristicHandle = event->data.evt_gatt_server_characteristic_status.characteristic;

    status_flags = event->data.evt_gatt_server_characteristic_status.status_flags;
    // Indication received successfully
    if (status_flags == 0x2) {
        ble_data.indicationInFlight = false;
        return;
    }

    client_flags = event->data.evt_gatt_server_characteristic_status.client_config_flags;
    if (client_flags == 0x0) {
        ble_data.indicating = false;
    }
    else if (client_flags == 0x2 && ble_data.notFirstConnection) {
        ble_data.indicating = true;
    }
    else if (!(ble_data.notFirstConnection)) {
        ble_data.notFirstConnection = true; //stack returns this event immediately
                                            //after connection and indicates even
                                            //when it's not supposed to
    }
}


void BLE_HandleIndicationTimeoutEvent(void) {
    LOG_ERROR("Timeout occurred on indication\r\n");
}
