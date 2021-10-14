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


// Original code from Dan Walkes. I (Sluiter) fixed a sign extension bug with the mantissa.
// convert IEEE-11073 32-bit float to integer
int32_t gattFloat32ToInt(const uint8_t *value_start_little_endian);


void BLE_Init(void) {
    bd_addr server_addr    = SERVER_BT_ADDRESS;
    ble_data.serverAddress = server_addr;
    ble_data.serverAddressType = 0;
}


void handle_ble_event(sl_bt_msg_t* event) {
    switch(SL_BT_MSG_ID(event->header)) {
#if DEVICE_IS_BLE_SERVER
        case sl_bt_evt_system_boot_id:
            BleServer_HandleBootEvent();
            break;

        case sl_bt_evt_connection_opened_id:
            BleServer_HandleConnectionOpenedEvent(event);
            break;

        case sl_bt_evt_connection_closed_id:
            BleServer_HandleConnectionClosedEvent();
            break;

        case sl_bt_evt_connection_parameters_id:
            BleServer_HandleConnectionParametersEvent(event);
            break;

        case sl_bt_evt_system_external_signal_id:
            BleServer_HandleExternalSignalEvent();
            break;

        case sl_bt_evt_gatt_server_characteristic_status_id:
            BleServer_HandleCharacteristicStatusEvent(event);
            break;

        case sl_bt_evt_gatt_server_indication_timeout_id:
            BleServer_HandleIndicationTimeoutEvent();
            break;

        case sl_bt_evt_system_soft_timer_id:
            BleServer_HandleSoftTimerEvent();
            break;

#else
        case sl_bt_evt_system_boot_id:
            BleClient_HandleBootEvent();
            break;

        case sl_bt_evt_scanner_scan_report_id:
            BleClient_HandleScanReportEvent(event);
            break;

        case sl_bt_evt_connection_opened_id:
            BleClient_HandleConnectionOpenedEvent(event);
            break;

        case sl_bt_evt_gatt_procedure_completed_id:
            break;

        case sl_bt_evt_gatt_service_id:
            BleClient_HandleGattServiceEvent(event);
            break;

        case sl_bt_evt_gatt_characteristic_id:
            BleClient_HandleGattCharacteristicEvent(event);
            break;

        case sl_bt_evt_gatt_characteristic_value_id:
            BleClient_HandleGattCharacteristicValueEvent(event);
            break;

        case sl_bt_evt_connection_closed_id:
            BleClient_HandleConnectionClosedEvent();
            break;

        case sl_bt_evt_system_soft_timer_id:
            BleClient_HandleSoftTimerEvent();
            break;
#endif
    }
}


ble_data_struct_t* BLE_GetDataStruct(void) {
    return (&ble_data);
}


/************************************************/
/***************Server Functions*****************/
/************************************************/


void BleServer_HandleBootEvent(void) {
    sl_status_t ble_status;

    // Get unique id
    /*ble_status = sl_bt_system_get_identity_address(&(ble_data.deviceAddress), &(ble_data.myAddressType));
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_system_get_identity_address: %d\r\n", ble_status);
    }*/

    // Pad and reverse unique ID to get System ID.
    ble_data.systemId[0] = ble_data.serverAddress.addr[5];
    ble_data.systemId[1] = ble_data.serverAddress.addr[4];
    ble_data.systemId[2] = ble_data.serverAddress.addr[3];
    ble_data.systemId[3] = 0xFF;
    ble_data.systemId[4] = 0xFE;
    ble_data.systemId[5] = ble_data.serverAddress.addr[2];
    ble_data.systemId[6] = ble_data.serverAddress.addr[1];
    ble_data.systemId[7] = ble_data.serverAddress.addr[0];

    ble_status = sl_bt_gatt_server_write_attribute_value(gattdb_system_id, 0, sizeof(ble_data.systemId), ble_data.systemId);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_gatt_server_write_attribute_value: %d\r\n", ble_status);
    }

    // Create advertising set
    ble_status = sl_bt_advertiser_create_set(&(ble_data.s_AdvertisingHandle));
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_advertiser_create_set: %d\r\n", ble_status);
    }

    ble_status = sl_bt_advertiser_set_timing(ble_data.s_AdvertisingHandle, 400, 400, 0, 0);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_advertiser_set_timing: %d\r\n", ble_status);
    }

    // Start advertising
    ble_status = sl_bt_advertiser_start(ble_data.s_AdvertisingHandle, sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_advertiser_start: %d\r\n", ble_status);
    }

    displayInit();

    displayPrintf(DISPLAY_ROW_NAME, "Server");
    displayPrintf(DISPLAY_ROW_BTADDR, "%x:%x:%x:%x:%x:%x",
                  ble_data.serverAddress.addr[0], ble_data.serverAddress.addr[1],
                  ble_data.serverAddress.addr[2], ble_data.serverAddress.addr[3],
                  ble_data.serverAddress.addr[4], ble_data.serverAddress.addr[5]);
    displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising");
    displayPrintf(DISPLAY_ROW_ASSIGNMENT, "A7");
}


void BleServer_HandleConnectionOpenedEvent(sl_bt_msg_t* event) {
    sl_status_t ble_status;

    // Modify ble_data variables
    ble_data.s_ConnectionHandle = event->data.evt_connection_opened.connection;
    ble_data.s_Connected = true;

    // Stop advertising
    ble_status = sl_bt_advertiser_stop(ble_data.s_AdvertisingHandle);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_advertiser_stop: %d\r\n", ble_status);
    }

    // Set connection parameters
    ble_status = sl_bt_connection_set_parameters(ble_data.s_ConnectionHandle, 120, 120, 4, 1500, 0, 0xFFFF);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_connection_set_parameters: %d\r\n", ble_status);
    }

    displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");
}


void BleServer_HandleConnectionClosedEvent(void) {
    sl_status_t ble_status;

    // Modify ble_data variables
    ble_data.s_Connected = false;
    ble_data.s_Indicating = false;
    //ble_data.s_NotFirstConnection = false;

    ble_status = sl_bt_advertiser_start(ble_data.s_AdvertisingHandle, sl_bt_advertiser_general_discoverable,
            sl_bt_advertiser_connectable_scannable);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_advertiser_start: %d\r\n", ble_status);
    }

    displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising");
}


void BleServer_HandleConnectionParametersEvent(sl_bt_msg_t* event) {
    LOG_INFO("handle: %d, interval, %d, latency: %d, timeout: %d\r\n",
             event->data.evt_connection_parameters.connection,
             event->data.evt_connection_parameters.interval,
             event->data.evt_connection_parameters.latency,
             event->data.evt_connection_parameters.timeout);
}


void BleServer_HandleExternalSignalEvent(void) {
    ble_data.s_ReadingTemp = 1;
    return;
}


void BleServer_HandleCharacteristicStatusEvent(sl_bt_msg_t* event) {
    uint8_t status_flags;
    uint16_t client_flags;

    ble_data.s_CharacteristicHandle = event->data.evt_gatt_server_characteristic_status.characteristic;

    status_flags = event->data.evt_gatt_server_characteristic_status.status_flags;
    // Indication received successfully
    if (status_flags == 0x2) {
        ble_data.s_IndicationInFlight = false;
        return;
    }

    client_flags = event->data.evt_gatt_server_characteristic_status.client_config_flags;
    if (client_flags == 0x0) {
        ble_data.s_Indicating = false;
    }
    else if (client_flags == 0x2) { //&& ble_data.s_NotFirstConnection) {
        ble_data.s_Indicating = true;
    }
    /*else if (!(ble_data.s_NotFirstConnection)) {
        ble_data.s_NotFirstConnection = true; //stack returns this event immediately
                                            //after connection and indicates even
                                            //when it's not supposed to
    }*/
}


void BleServer_HandleIndicationTimeoutEvent(void) {
    LOG_ERROR("Timeout occurred on indication\r\n");
}


void BleServer_HandleSoftTimerEvent(void) {
    displayUpdate();
}


/************************************************/
/***************Client Functions*****************/
/************************************************/


void BleClient_HandleBootEvent(void) {
    sl_status_t ble_status;

    ble_status = sl_bt_system_get_identity_address(&(ble_data.c_DeviceAddress), &(ble_data.c_DeviceAddressType));
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_system_get_identity_address: %d\r\n", ble_status);
    }

    // Set passive scanning on 1Mb PHY
    ble_status = sl_bt_scanner_set_mode(sl_bt_gap_1m_phy, 0);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_scanner_set_mode: %d\r\n", ble_status);
    }

    // Set scan interval and scan window
    ble_status = sl_bt_scanner_set_timing(sl_bt_gap_1m_phy, 80, 40);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_scanner_set_timing: %d\r\n", ble_status);
    }

    // Set the default connection parameters for subsequent connections
    /*ble_status = sl_bt_connection_set_default_parameters(120, 120, 4, 1500, 0, 0xFFFF);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_connection_set_default_parameters: %d\r\n", ble_status);
    }*/

    ble_status = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_scanner_start: %d\r\n", ble_status);
    }

    displayInit();

    displayPrintf(DISPLAY_ROW_NAME, "Client");
    displayPrintf(DISPLAY_ROW_BTADDR, "%x:%x:%x:%x:%x:%x",
                  ble_data.c_DeviceAddress.addr[0], ble_data.c_DeviceAddress.addr[1],
                  ble_data.c_DeviceAddress.addr[2], ble_data.c_DeviceAddress.addr[3],
                  ble_data.c_DeviceAddress.addr[4], ble_data.c_DeviceAddress.addr[5]);
    displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering");
    displayPrintf(DISPLAY_ROW_ASSIGNMENT, "A7");
}


void BleClient_HandleScanReportEvent(sl_bt_msg_t* event) {
    sl_status_t ble_status;

    // Check for packet type: Connectable Scannable Undirected Advertising
    if (event->data.evt_scanner_scan_report.packet_type == 0) {
        // Check for address type: Public Address
        if (event->data.evt_scanner_scan_report.address_type == 0) {
            // Check for addresses matching
            if (!memcmp(&(ble_data.serverAddress.addr[0]), &(event->data.evt_scanner_scan_report.address.addr[0]), 6)) {
                ble_status = sl_bt_scanner_stop();
                if (ble_status != SL_STATUS_OK) {
                    LOG_ERROR("sl_bt_scanner_stop: %d\r\n", ble_status);
                }

                ble_status = sl_bt_connection_open(ble_data.serverAddress,
                                                   ble_data.serverAddressType,
                                                   sl_bt_gap_1m_phy, NULL);
                if (ble_status != SL_STATUS_OK) {
                    LOG_ERROR("sl_bt_connection_open: %d\r\n", ble_status);
                }
            }
        }
    }
}


void BleClient_HandleConnectionOpenedEvent(sl_bt_msg_t* event) {
    ble_data.c_ConnectionHandle = event->data.evt_connection_opened.connection;
    ble_data.c_Connected = true;

    displayPrintf(DISPLAY_ROW_BTADDR2, "%x:%x:%x:%x:%x:%x",
                  ble_data.serverAddress.addr[0], ble_data.serverAddress.addr[1],
                  ble_data.serverAddress.addr[2], ble_data.serverAddress.addr[3],
                  ble_data.serverAddress.addr[4], ble_data.serverAddress.addr[5]);
    displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");
}


void BleClient_HandleGattServiceEvent(sl_bt_msg_t* event) {
    ble_data.c_ServiceHandle = event->data.evt_gatt_service.service;
}


void BleClient_HandleGattCharacteristicEvent(sl_bt_msg_t* event) {
    ble_data.c_CharacteristicHandle = event->data.evt_gatt_characteristic.characteristic;
    ble_data.c_CharacteristicProperties = event->data.evt_gatt_characteristic.properties;
}


void BleClient_HandleGattCharacteristicValueEvent(sl_bt_msg_t* event) {
    sl_status_t ble_status;
    uint8array value_array;
    int32_t temp_value;

    if (ble_data.c_Connected && ble_data.c_Indicating) {
        value_array = event->data.evt_gatt_characteristic_value.value;

        temp_value = gattFloat32ToInt(value_array.data);

        displayPrintf(DISPLAY_ROW_TEMPVALUE, "%d", temp_value);

        ble_status = sl_bt_gatt_send_characteristic_confirmation(ble_data.c_ConnectionHandle);
        if (ble_status != SL_STATUS_OK) {
            LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation: %d\r\n", ble_status);
        }
    }
}


void BleClient_HandleConnectionClosedEvent(void) {
    ble_data.c_Connected = false;
    ble_data.c_Indicating = false;
}


void BleClient_HandleSoftTimerEvent(void) {
    displayUpdate();
}


// Original code from Dan Walkes. I (Sluiter) fixed a sign extension bug with the mantissa.
// convert IEEE-11073 32-bit float to integer
int32_t gattFloat32ToInt(const uint8_t *value_start_little_endian)
{
    uint8_t signByte = 0;
    int32_t mantissa;
    // data format pointed at by value_start_little_endian is:
    // [0] = contains the flags byte
    // [3][2][1] = mantissa (24-bit 2’s complement)
    // [4] = exponent (8-bit 2’s complement)
    int8_t exponent = (int8_t)value_start_little_endian[4];
    // sign extend the mantissa value if the mantissa is negative
    if (value_start_little_endian[3] & 0x80) { // msb of [3] is the sign of the mantissa
    signByte = 0xFF;
    }
    mantissa = (int32_t) (value_start_little_endian[1] << 0) |
    (value_start_little_endian[2] << 8) |
    (value_start_little_endian[3] << 16) |
    (signByte << 24) ;
    // value = 10^exponent * mantissa, pow() returns a double type
    return (int32_t) (pow(10, exponent) * mantissa);
} // gattFloat32ToInt
