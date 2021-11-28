/*
 * ble.c
 *
 *  Created on: Sep 27, 2021
 *      Author: vishn
 */

#include "ble.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

//macros for client and server
#define CONNECTION_PARAMETER_DEBUG_PRINTS           (0)

//macros for server
#define ADVERTISE_VALUE                             (0x190)//400
#define CONNECTION_TIME                             (0x3C) //60
//Security related macros
#define BONDING_FLAGS                               (0x0F)

//Macros for client
#define SCAN_PASSIVE                                (0)
#define SCAN_INTERVAL                               (80)// value = 50 msec /0.625 msec
#define SCAN_WINDOW                                 (40)// value = 25 msec /0.625 msec

uint8_t health_service[HEALTH_SIZE] = {
    0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87,
    0x3e, 0x43, 0xc8,0x38, 0x03, 0x00, 0x00, 0x00
};

uint8_t health_char[HEALTH_SIZE] = {
    0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87,
    0x3e, 0x43, 0xc8,0x38, 0x04, 0x00, 0x00, 0x00
};

ble_data_struct_t ble_data = { 0 };


// Original code from Dan Walkes. I (Sluiter) fixed a sign extension bug with the mantissa.
// convert IEEE-11073 32-bit float to integer
int32_t gattFloat32ToInt(const uint8_t *value_start_little_endian);


static indication_struct_t indication_q[INDICATION_QUEUE_SIZE];
static uint8_t rd_ptr = 0;
static uint8_t wr_ptr = 0;
static uint8_t q_size = 0;



/************************************************/
/****************Queue Functions*****************/
/************************************************/


void IndicationQ_Enqueue(indication_struct_t indication) {
    if (q_size == INDICATION_QUEUE_SIZE) {
        return;
    }
    indication_q[(wr_ptr++) & INDICATION_QUEUE_SIZE_MASK] = indication;
    q_size++;
}


indication_struct_t IndicationQ_Dequeue(void) {
    indication_struct_t current_indication;
    if (q_size == 0) {
        current_indication = *(indication_struct_t *)0;
    }
    else {
        current_indication = indication_q[(rd_ptr++) & INDICATION_QUEUE_SIZE_MASK];
        q_size--;
    }
    return current_indication;
}


uint8_t IndicationQ_IsIndicationPending(void) {
    if (q_size == 0) {
        return 0;
    }

    return 1;
}


void IndicationQ_Reset(void) {
    q_size = 0;
    rd_ptr = 0;
    wr_ptr = 0;
}


/************************************************/
/*************Common BLE Functions***************/
/************************************************/


void BLE_Init(void) {
    bd_addr server_addr    = SERVER_BT_ADDRESS;
    ble_data.serverAddress = server_addr;
    ble_data.serverAddressType = 0;

    //Rishab
    ble_data.s_IndicationInFlight = false;
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
            BleServer_HandleExternalSignalEvent(event);
            break;

        case sl_bt_evt_gatt_server_characteristic_status_id:
            BleServer_HandleCharacteristicStatusEvent(event);
            break;

        case sl_bt_evt_gatt_server_indication_timeout_id:
            BleServer_HandleIndicationTimeoutEvent();
            break;

        case sl_bt_evt_sm_confirm_bonding_id:
            BleServer_HandleBondingConfirmEvent();
            break;

        case sl_bt_evt_sm_confirm_passkey_id:
            BleServer_HandlePasskeyConfirmEvent(event);
            break;

        case sl_bt_evt_sm_bonded_id:
            BleServer_HandleBondedEvent();
            break;

        case sl_bt_evt_sm_bonding_failed_id:
            BleServer_HandleBondingFailedEvent(event);
            break;

        case sl_bt_evt_system_soft_timer_id:
            BleServer_HandleSoftTimerEvent(event);
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

        case sl_bt_evt_connection_closed_id:
            BleClient_HandleConnectionClosedEvent();
            break;

        case sl_bt_evt_connection_parameters_id:
            BleClient_HandleConnectionParametersEvent(event);
            break;

        case sl_bt_evt_gatt_procedure_completed_id:
            BleClient_HandleGattProcedureCompleted(event);
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

        case sl_bt_evt_system_external_signal_id:
            BleClient_HandleExternalSignalEvent(event);
            break;

        case sl_bt_evt_sm_confirm_passkey_id:
            BleClient_HandlePasskeyConfirmEvent(event);
            break;

        case sl_bt_evt_sm_bonded_id:
            BleClient_HandleBondedEvent();
            break;

        case sl_bt_evt_sm_bonding_failed_id:
            BleClient_HandleBondingFailedEvent(event);
            break;

        case sl_bt_evt_system_soft_timer_id:
            BleClient_HandleSoftTimerEvent(event);
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

    ble_status = sl_bt_sm_delete_bondings();
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_sm_delete_bondings: %x\r", ble_status);
    }

    ble_status = sl_bt_sm_configure(BONDING_FLAGS, sl_bt_sm_io_capability_displayyesno);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_sm_configure: %x\r", ble_status);
    }

    ble_status = sl_bt_gatt_server_write_attribute_value(gattdb_system_id, 0, sizeof(ble_data.serverAddress.addr), ble_data.serverAddress.addr);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_gatt_server_write_attribute_value: %x\r", ble_status);
    }

    // Create advertising set
    ble_status = sl_bt_advertiser_create_set(&(ble_data.s_AdvertisingHandle));
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_advertiser_create_set: %x\r", ble_status);
    }

    ble_status = sl_bt_advertiser_set_timing(ble_data.s_AdvertisingHandle, ADVERTISE_VALUE, ADVERTISE_VALUE, 0, 0);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_advertiser_set_timing: %x\r", ble_status);
    }

    // Start advertising
    ble_status = sl_bt_advertiser_start(ble_data.s_AdvertisingHandle, sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_advertiser_start: %x\r", ble_status);
    }

    ble_status = sl_bt_system_set_soft_timer(INDICATION_QUEUE_TIMER_INTERVAL, INDICATION_QUEUE_TIMER_HANDLE, false);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_system_set_soft_timer: %x\r", ble_status);
    }

    displayInit();

    displayPrintf(DISPLAY_ROW_NAME, BLE_DEVICE_TYPE_STRING);
    displayPrintf(DISPLAY_ROW_BTADDR, "%02X:%02X:%02X:%02X:%02X:%02X",
                  ble_data.serverAddress.addr[0], ble_data.serverAddress.addr[1],
                  ble_data.serverAddress.addr[2], ble_data.serverAddress.addr[3],
                  ble_data.serverAddress.addr[4], ble_data.serverAddress.addr[5]);
    displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising");
    displayPrintf(DISPLAY_ROW_ASSIGNMENT, "IoT Project");
}


void BleServer_HandleConnectionOpenedEvent(sl_bt_msg_t* event) {
    sl_status_t ble_status;

    // Modify ble_data variables
    ble_data.s_ConnectionHandle = event->data.evt_connection_opened.connection;
    ble_data.s_ClientConnected = true;
    ble_data.s_IndicationInFlight = false;

    // Stop advertising
    ble_status = sl_bt_advertiser_stop(ble_data.s_AdvertisingHandle);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_advertiser_stop: %x\r", ble_status);
    }

    // Set connection parameters
    ble_status = sl_bt_connection_set_parameters( ble_data.s_ConnectionHandle,
                                                  CONNECTION_TIME,
                                                  CONNECTION_TIME,
                                                  0x03, //(75 * 3 =  At 4th, connection will happen)
                                                  0x50, //80 - (1+3)*(75*2) = 600 -> 600/10 = 60 (took 80) -> 0x50
                                                  0,
                                                  0xffff);
    //sl_bt_connection_set_parameters(ble_data.s_ConnectionHandle, 120, 120, 4, 1500, 0, 0xFFFF);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_connection_set_parameters: %x\r", ble_status);
    }

    displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");
}


void BleServer_HandleConnectionClosedEvent(void) {
    sl_status_t ble_status;

    // Modify ble_data variables
    ble_data.s_ClientConnected = false;
    ble_data.s_ConnectionHandle = 0;
    ble_data.s_TemperatureIndicating = false;
    ble_data.s_ButtonIndicating = false;
    ble_data.s_IndicationInFlight = false;
    ble_data.s_Bonded = false;
    gpioLed0SetOff();
    gpioLed1SetOff();

    IndicationQ_Reset();

    ble_status = sl_bt_sm_delete_bondings();
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_sm_delete_bondings: %x\r", ble_status);
    }


    ble_status = sl_bt_advertiser_start(ble_data.s_AdvertisingHandle, sl_bt_advertiser_general_discoverable,
            sl_bt_advertiser_connectable_scannable);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_advertiser_start: %x\r", ble_status);
    }

    displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising");
}


void BleServer_HandleConnectionParametersEvent(sl_bt_msg_t* event) {
    LOG_INFO("handle: %d, interval, %d, latency: %d, timeout: %x\r",
             event->data.evt_connection_parameters.connection,
             event->data.evt_connection_parameters.interval,
             event->data.evt_connection_parameters.latency,
             event->data.evt_connection_parameters.timeout);
}


void BleServer_HandleExternalSignalEvent(sl_bt_msg_t* event) {
    sl_status_t ble_status;
    indication_struct_t indication;

    if (event->data.evt_system_external_signal.extsignals == ev_PB0_PRESSED) {
        displayPrintf(DISPLAY_ROW_9, "Button Pressed");

        indication.characteristicHandle = gattdb_heartbeat_state;
        indication.buff[0] = BUTTON_STATE_PRESSED;
        indication.buff[1] = 0;
        indication.bufferLen = BUTTON_BUFF_LEN;

        ble_status = sl_bt_gatt_server_write_attribute_value(gattdb_heartbeat_state,
                                                             0,
                                                             1,
                                                             &(indication.buff[0]));
        if (ble_status != SL_STATUS_OK) {
            LOG_ERROR("sl_bt_gatt_server_write_attribute_value: %x\r", ble_status);
        }
        if ((ble_data.s_IndicationInFlight) && (ble_data.s_ButtonIndicating) && (ble_data.s_Bonded)) {
            IndicationQ_Enqueue(indication);
        }
        else {
            if (ble_data.s_BondingPending) {
                ble_status = sl_bt_sm_passkey_confirm(ble_data.s_ConnectionHandle, true);
                if (ble_status != SL_STATUS_OK) {
                    LOG_ERROR("sl_bt_sm_passkey_confirm: %x\r", ble_status);
                }
                ble_data.s_BondingPending = false;
            }
            else if (ble_data.s_ButtonIndicating && ble_data.s_Bonded) {
                ble_status = sl_bt_gatt_server_send_indication(
                    ble_data.s_ConnectionHandle,
                    indication.characteristicHandle,
                    indication.bufferLen,
                    indication.buff);
                if (ble_status != SL_STATUS_OK) {
                    LOG_ERROR("sl_bt_gatt_server_send_indication: %x\r", ble_status);
                }
                ble_data.s_IndicationInFlight = true;
            }
        }
    }
    else if (event->data.evt_system_external_signal.extsignals == ev_PB0_RELEASED) {
        displayPrintf(DISPLAY_ROW_9, "Button Released");
        indication.characteristicHandle = gattdb_heartbeat_state;
        indication.buff[0] = BUTTON_STATE_RELEASED;
        indication.buff[1] = 0;
        indication.bufferLen = BUTTON_BUFF_LEN;

        ble_status = sl_bt_gatt_server_write_attribute_value(gattdb_heartbeat_state,
                                                             0,
                                                             1,
                                                             &(indication.buff[0]));
        if (ble_status != SL_STATUS_OK) {
            LOG_ERROR("sl_bt_gatt_server_write_attribute_value: %x\r", ble_status);
        }

        if ((ble_data.s_IndicationInFlight) && (ble_data.s_ButtonIndicating) && (ble_data.s_Bonded)) {
            IndicationQ_Enqueue(indication);
        }
        else {
            if (ble_data.s_ButtonIndicating && ble_data.s_Bonded) {
                ble_status = sl_bt_gatt_server_send_indication(
                    ble_data.s_ConnectionHandle,
                    indication.characteristicHandle,
                    indication.bufferLen,
                    indication.buff);
                if (ble_status != SL_STATUS_OK) {
                    LOG_ERROR("sl_bt_gatt_server_send_indication: %x\r", ble_status);
                }
                ble_data.s_IndicationInFlight = true;
            }
        }
    }
    // Temp state machine event
    else {
        ble_data.s_ReadingTemp = 1;
    }
}


void BleServer_HandleCharacteristicStatusEvent(sl_bt_msg_t* event) {
    uint8_t status_flags;
    uint16_t client_flags;
    uint16_t characteristic_flags;

#if 0
    if (event->data.evt_gatt_server_characteristic_status.characteristic == gattdb_temperature_measurement) {
        ble_data.s_TemperatureCharacteristicHandle = gattdb_temperature_measurement;

        status_flags = event->data.evt_gatt_server_characteristic_status.status_flags;
        // Indication received successfully
        if (status_flags == 0x2) {
            ble_data.s_IndicationInFlight = false;
            return;
        }

        client_flags = event->data.evt_gatt_server_characteristic_status.client_config_flags;
        if (client_flags == 0x0) {
            gpioLed0SetOff();
            ble_data.s_TemperatureIndicating = false;
        }
        else if (client_flags == 0x2) {
            gpioLed0SetOn();
            ble_data.s_TemperatureIndicating = true;
        }
    }
#endif
#if 0
    else if (event->data.evt_gatt_server_characteristic_status.characteristic == gattdb_heartbeat_state) {
        ble_data.s_ButtonCharacteristicHandle = gattdb_heartbeat_state;

        status_flags = event->data.evt_gatt_server_characteristic_status.status_flags;
        // Indication received successfully
        if (status_flags == 0x2) {
            ble_data.s_IndicationInFlight = false;
            return;
        }

        client_flags = event->data.evt_gatt_server_characteristic_status.client_config_flags;
        if (client_flags == 0x0) {
            gpioLed1SetOff();
            ble_data.s_ButtonIndicating = false;
        }
        else if ((client_flags == 0x2)) {
            gpioLed1SetOn();
            ble_data.s_ButtonIndicating = true;
        }
    }
#endif

    characteristic_flags = event->data.evt_gatt_server_characteristic_status.characteristic;
    status_flags = event->data.evt_gatt_server_characteristic_status.status_flags;
    client_flags = event->data.evt_gatt_server_characteristic_status.client_config_flags;

    if(characteristic_flags == gattdb_heartbeat_state)
    {
        if(status_flags == sl_bt_gatt_server_client_config)
        {
            if(client_flags == gatt_disable)
            {
                /* app gave disable indication */
                ble_data.s_health_indications_client = false;
                LOG_INFO("health::GATT disable\r");
                gpioLed1SetOff();
            }
            if(client_flags == gatt_indication)
            {
                /* app gave enable indication */
                ble_data.s_health_indications_client = true;
                LOG_INFO("health::GATT Enable\r");
                gpioLed1SetOn();
            }
        }
        if(status_flags == sl_bt_gatt_server_confirmation)
        {
            ble_data.s_IndicationInFlight = false;
        }
    }
}


void BleServer_HandleIndicationTimeoutEvent(void) {
    LOG_ERROR("Timeout occurred on indication\r");
}


void BleServer_HandleBondingConfirmEvent(void) {
    sl_status_t ble_status;

    ble_status = sl_bt_sm_bonding_confirm(ble_data.s_ConnectionHandle, 1);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_sm_bonding_confirm: %x\r", ble_status);
    }
}


void BleServer_HandlePasskeyConfirmEvent(sl_bt_msg_t* event) {
    displayPrintf(DISPLAY_ROW_PASSKEY, "%d", event->data.evt_sm_confirm_passkey.passkey);
    displayPrintf(DISPLAY_ROW_ACTION, "Confirm with PB0");
    ble_data.s_BondingPending = true;
}


void BleServer_HandleBondedEvent(void) {
    displayPrintf(DISPLAY_ROW_CONNECTION, "Bonded");
    displayPrintf(DISPLAY_ROW_PASSKEY, "");
    displayPrintf(DISPLAY_ROW_ACTION, "");
    ble_data.s_Bonded = true;
}


void BleServer_HandleBondingFailedEvent(sl_bt_msg_t* event) {
    ble_data.s_Bonded = false;
    LOG_ERROR("Bonding failed: %x\r", event->data.evt_sm_bonding_failed.reason);
}


void BleServer_HandleSoftTimerEvent(sl_bt_msg_t* event) {
    indication_struct_t indication;
    sl_status_t ble_status;

    if (event->data.evt_system_soft_timer.handle == LCD_TIMER_HANDLE) {
        displayUpdate();
    }
    else if (event->data.evt_system_soft_timer.handle == INDICATION_QUEUE_TIMER_HANDLE) {
        if (IndicationQ_IsIndicationPending() && !(ble_data.s_IndicationInFlight)) {
            indication = IndicationQ_Dequeue();
            ble_status = sl_bt_gatt_server_send_indication(
                ble_data.s_ConnectionHandle,
                indication.characteristicHandle,
                indication.bufferLen,
                indication.buff);
            if (ble_status != SL_STATUS_OK) {
                LOG_ERROR("sl_bt_gatt_server_send_indication: %x\r", ble_status);
            }
            ble_data.s_IndicationInFlight = true;
        }
    }
}


/************************************************/
/***************Client Functions*****************/
/************************************************/


void BleClient_HandleBootEvent(void) {
    sl_status_t ble_status;

    ble_status = sl_bt_sm_delete_bondings();
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_sm_delete_bondings: %x\r", ble_status);
    }

    ble_status = sl_bt_sm_configure(BONDING_FLAGS, sl_bt_sm_io_capability_displayyesno);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_sm_configure: %x\r", ble_status);
    }

    ble_status = sl_bt_system_get_identity_address(&(ble_data.c_DeviceAddress), &(ble_data.c_DeviceAddressType));
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_system_get_identity_address: %x\r", ble_status);
    }

    // Set passive scanning on 1Mb PHY
    ble_status = sl_bt_scanner_set_mode(sl_bt_gap_1m_phy,SCAN_PASSIVE);
    //sl_bt_scanner_set_mode(sl_bt_gap_1m_phy, 0);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_scanner_set_mode: %x\r", ble_status);
    }

    // Set scan interval and scan window
    ble_status = sl_bt_scanner_set_timing(sl_bt_gap_1m_phy,SCAN_INTERVAL,SCAN_WINDOW);
    //sl_bt_scanner_set_timing(sl_bt_gap_1m_phy, 80, 40);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_scanner_set_timing: %x\r", ble_status);
    }

    // Set the default connection parameters for subsequent connections
    ble_status = sl_bt_connection_set_default_parameters(CONNECTION_TIME, //75
                                                         CONNECTION_TIME,
                                                         0x03, //(75 * 3 =  At 4th, connection will happen) //0
                                                         0x50, //80 - (1+3)*(75*2) = 600 -> 675/10 = 60 (took 80) -> 0x50 //
                                                         0,
                                                         0xffff);
        //sl_bt_connection_set_default_parameters(120, 120, 4, 1500, 0, 0xFFFF);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_connection_set_default_parameters: %x\r", ble_status);
    }

    ble_status = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_scanner_start: %x\r", ble_status);
    }

    displayInit();

    displayPrintf(DISPLAY_ROW_NAME, BLE_DEVICE_TYPE_STRING);
    displayPrintf(DISPLAY_ROW_BTADDR, "%02X:%02X:%02X:%02X:%02X:%02X",
                  ble_data.c_DeviceAddress.addr[0], ble_data.c_DeviceAddress.addr[1],
                  ble_data.c_DeviceAddress.addr[2], ble_data.c_DeviceAddress.addr[3],
                  ble_data.c_DeviceAddress.addr[4], ble_data.c_DeviceAddress.addr[5]);
    displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering");
    displayPrintf(DISPLAY_ROW_ASSIGNMENT, "IoT Project");

    //Rishab
    //health service
    memcpy(ble_data.health_service,health_service,HEALTH_SIZE*sizeof(ble_data.health_service[0]));
    memcpy(ble_data.health_char,health_char,HEALTH_SIZE*sizeof(ble_data.health_char[0]));

}


void BleClient_HandleScanReportEvent(sl_bt_msg_t* event) {
    sl_status_t ble_status;

    // Check for packet type: Connectable Scannable Undirected Advertising
    if ((event->data.evt_scanner_scan_report.packet_type & 0x7) == 0) {
        // Check for address type: Public Address
        if (event->data.evt_scanner_scan_report.address_type == sl_bt_gap_public_address) {
            // Check for addresses matching
            if (!memcmp(&(ble_data.serverAddress.addr[0]), &(event->data.evt_scanner_scan_report.address.addr[0]), sizeof(bd_addr))) {
                ble_status = sl_bt_scanner_stop();
                if (ble_status != SL_STATUS_OK) {
                    LOG_ERROR("sl_bt_scanner_stop: %x\r", ble_status);
                }

                ble_status = sl_bt_connection_open(ble_data.serverAddress,
                                                   ble_data.serverAddressType,
                                                   sl_bt_gap_1m_phy, NULL);
                if (ble_status != SL_STATUS_OK) {
                    LOG_ERROR("sl_bt_connection_open: %x\r", ble_status);
                }
            }
        }
    }
}


void BleClient_HandleConnectionOpenedEvent(sl_bt_msg_t* event) {
    ble_data.c_ConnectionHandle = event->data.evt_connection_opened.connection;
    ble_data.c_Connected = true;

    displayPrintf(DISPLAY_ROW_BTADDR2, "%02X:%02X:%02X:%02X:%02X:%02X",
                  ble_data.serverAddress.addr[0], ble_data.serverAddress.addr[1],
                  ble_data.serverAddress.addr[2], ble_data.serverAddress.addr[3],
                  ble_data.serverAddress.addr[4], ble_data.serverAddress.addr[5]);
    displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");
}


void BleClient_HandleConnectionClosedEvent(void) {
    sl_status_t ble_status;
    ble_data.c_Connected = false;
    ble_data.c_TemperatureIndicating = false;
    ble_data.c_ButtonIndicating = false;
    ble_data.c_Bonded = false;

    ble_status = sl_bt_sm_delete_bondings();
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_sm_delete_bondings: %x\r", ble_status);
    }

    displayPrintf(DISPLAY_ROW_BTADDR2, "");
    displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering");
    displayPrintf(DISPLAY_ROW_TEMPVALUE, "");

    //Rishab

}


void BleClient_HandleConnectionParametersEvent(sl_bt_msg_t* event) {
    LOG_INFO("handle: %d, interval, %d, latency: %d, timeout: %x\r",
             event->data.evt_connection_parameters.connection,
             event->data.evt_connection_parameters.interval,
             event->data.evt_connection_parameters.latency,
             event->data.evt_connection_parameters.timeout);
}


void BleClient_HandleGattProcedureCompleted(sl_bt_msg_t* event) {
    sl_status_t ble_status;
    if (event->data.evt_gatt_procedure_completed.result == SL_STATUS_BT_ATT_INSUFFICIENT_ENCRYPTION) {
        ble_status = sl_bt_sm_increase_security(ble_data.c_ConnectionHandle);
        if (ble_status != SL_STATUS_OK) {
            LOG_ERROR("sl_bt_sm_increase_security: %x\r", ble_status);
        }
        ble_data.c_BondingPending = true;
    }
    else if (event->data.evt_gatt_procedure_completed.result == 0)
    {

    }
}


void BleClient_HandleGattServiceEvent(sl_bt_msg_t* event) {
    if (event->data.evt_gatt_service.uuid.len == sizeof(thermo_service)) {
        ble_data.c_TemperatureServiceHandle = event->data.evt_gatt_service.service;
    }
#if 0
    else if (event->data.evt_gatt_service.uuid.len == sizeof(button_service)) {
        ble_data.c_ButtonServiceHandle = event->data.evt_gatt_service.service;
    }
#endif

    if(0 == (memcmp(event->data.evt_gatt_service.uuid.data,ble_data.health_service,sizeof(ble_data.health_service))))
    {
        ble_data.health_service_status = true;
        ble_data.c_health_service_handle = event->data.evt_gatt_service.service;
    }
}


void BleClient_HandleGattCharacteristicEvent(sl_bt_msg_t* event) {
    if (event->data.evt_gatt_characteristic.uuid.len == sizeof(thermo_char)) {
        ble_data.c_TemperatureCharacteristicHandle = event->data.evt_gatt_characteristic.characteristic;
        ble_data.c_TemperatureCharacteristicProperties = event->data.evt_gatt_characteristic.properties;
    }
#if 0
    else if (event->data.evt_gatt_characteristic.uuid.len == sizeof(button_char)) {
        ble_data.c_ButtonCharacteristicHandle = event->data.evt_gatt_characteristic.characteristic;
        ble_data.c_ButtonCharacteristicProperties = event->data.evt_gatt_characteristic.properties;
    }
#endif

    // Identify the correct characteristic //multiple firing
    if(0 == (memcmp(event->data.evt_gatt_characteristic.uuid.data,ble_data.health_char,sizeof(ble_data.health_char))))
    {
        ble_data.health_characteristic_status = true;
        ble_data.c_health_characteristic_handle = event->data.evt_gatt_characteristic.characteristic;
    }
}


void BleClient_HandleGattCharacteristicValueEvent(sl_bt_msg_t* event) {
    sl_status_t ble_status;
    uint8_t* value_array;
    int32_t temp_value;
    //uint8_t button_state;

    if (event->data.evt_gatt_characteristic_value.characteristic == ble_data.c_TemperatureCharacteristicHandle) {
        if (ble_data.c_Connected && ble_data.c_TemperatureIndicating) {
            value_array = event->data.evt_gatt_characteristic_value.value.data;

            temp_value = gattFloat32ToInt(&value_array[0]);

            displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temp=%d", temp_value);

            ble_status = sl_bt_gatt_send_characteristic_confirmation(ble_data.c_ConnectionHandle);
            if (ble_status != SL_STATUS_OK) {
                LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation: %x\r", ble_status);
            }
        }
    }
#if 0
    else if (event->data.evt_gatt_characteristic_value.characteristic == ble_data.c_ButtonCharacteristicHandle) {
        if (event->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication) {
            if (ble_data.c_Connected && ble_data.c_Bonded && ble_data.c_ButtonIndicating) {
                button_state = event->data.evt_gatt_characteristic_value.value.data[0];

                if (button_state == 1) {
                    displayPrintf(DISPLAY_ROW_9, "Button Pressed");
                }
                else if (button_state == 0) {
                    displayPrintf(DISPLAY_ROW_9, "Button Released");
                }

                ble_status = sl_bt_gatt_send_characteristic_confirmation(ble_data.c_ConnectionHandle);
                if (ble_status != SL_STATUS_OK) {
                    LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation: %x\r", ble_status);
                }
            }
        }
        else if (event->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_read_response) {
            if (ble_data.c_Connected && ble_data.c_Bonded) {
                button_state = event->data.evt_gatt_characteristic_value.value.data[0];

                if (button_state == 1) {
                    displayPrintf(DISPLAY_ROW_9, "Button Pressed");
                }
                else if (button_state == 0) {
                    displayPrintf(DISPLAY_ROW_9, "Button Released");
                }
            }
        }
    }
#endif

    //Rishab
    sl_status_t sc = 0;
    if( (event->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication)
      && (event->data.evt_gatt_characteristic_value.characteristic == ble_data.c_health_characteristic_handle) )
    {
        ble_data.health_char_value = &(event->data.evt_gatt_characteristic_value.value.data[0]);
        uint8_t heartbeat_value = ble_data.health_char_value[0];
        LOG_INFO("heartbeat_value = %d\r",heartbeat_value);
        displayPrintf(DISPLAY_ROW_HEARTBEAT, "HeartBeat = %d\r",heartbeat_value);

        sc = sl_bt_gatt_send_characteristic_confirmation(event->data.evt_gatt_characteristic_value.connection);
        if(sc != SL_STATUS_OK)
        {
            LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation:: status=0x%04x\r",(unsigned int)sc);
        }
    }
}


void BleClient_HandleExternalSignalEvent(sl_bt_msg_t* event) {
    sl_status_t ble_status;
    ble_ext_signal_event_t ev = event->data.evt_system_external_signal.extsignals;

    if (ev == ev_PB0_PRESSED) {
        if (ble_data.c_BondingPending) {
            sl_bt_sm_passkey_confirm(ble_data.c_ConnectionHandle, true);
            ble_data.c_BondingPending = false;
        }
        ble_data.c_ButtonIndicationStatus = IND_SEQ_PB0_PRESSED;
    }
    else if (ev == ev_PB0_RELEASED) {
        if (ble_data.c_ButtonIndicationStatus == (IND_SEQ_PB0_PRESSED | IND_SEQ_PB1_PRESSED | IND_SEQ_PB1_RELEASED)) {
            if (ble_data.c_ButtonIndicating) {
                ble_status = sl_bt_gatt_set_characteristic_notification(ble_data.c_ConnectionHandle,
                                                                        ble_data.c_ButtonCharacteristicHandle, 0x00);
                if (ble_status != SL_STATUS_OK) {
                    LOG_ERROR("sl_bt_gatt_set_characteristic_notification: %x\r", ble_status);
                }
                ble_data.c_ButtonIndicating = false;
            }
            else {
                ble_status = sl_bt_gatt_set_characteristic_notification(ble_data.c_ConnectionHandle,
                                                                        ble_data.c_ButtonCharacteristicHandle, 0x02);
                if (ble_status != SL_STATUS_OK) {
                    LOG_ERROR("sl_bt_gatt_set_characteristic_notification: %x\r", ble_status);
                }
                ble_data.c_ButtonIndicating = true;
            }
        }
        else {
            ble_data.c_ButtonIndicationStatus = 0;
        }
    }
    else if (ev == ev_PB1_PRESSED) {
        if (ble_data.c_ButtonIndicationStatus == IND_SEQ_PB0_PRESSED) {
            ble_data.c_ButtonIndicationStatus |= IND_SEQ_PB1_PRESSED;
            return;
        }
        else {
            ble_data.c_ButtonIndicationStatus = 0;
        }

        ble_status = sl_bt_gatt_read_characteristic_value(ble_data.c_ConnectionHandle, ble_data.c_ButtonCharacteristicHandle);
        if ((ble_status != SL_STATUS_OK) && (ble_status != SL_STATUS_BT_ATT_INSUFFICIENT_ENCRYPTION)) {
            LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation: %x\r", ble_status);
        }
    }
    else if (ev == ev_PB1_RELEASED) {
        if (ble_data.c_ButtonIndicationStatus == (IND_SEQ_PB0_PRESSED | IND_SEQ_PB1_PRESSED)) {
            ble_data.c_ButtonIndicationStatus |= IND_SEQ_PB1_RELEASED;
        }
        else {
            ble_data.c_ButtonIndicationStatus = 0;
        }
    }
}


void BleClient_HandlePasskeyConfirmEvent(sl_bt_msg_t* event) {
    displayPrintf(DISPLAY_ROW_PASSKEY, "%d", event->data.evt_sm_confirm_passkey.passkey);
    displayPrintf(DISPLAY_ROW_ACTION, "Confirm with PB0");
    ble_data.c_BondingPending = true;
}


void BleClient_HandleBondedEvent(void) {
    displayPrintf(DISPLAY_ROW_CONNECTION, "Bonded");
    displayPrintf(DISPLAY_ROW_PASSKEY, "");
    displayPrintf(DISPLAY_ROW_ACTION, "");
    ble_data.c_Bonded = true;
}


void BleClient_HandleBondingFailedEvent(sl_bt_msg_t* event) {
    ble_data.c_Bonded = false;
    LOG_ERROR("Bonding failed: %x\r", event->data.evt_sm_bonding_failed.reason);
}


void BleClient_HandleSoftTimerEvent(sl_bt_msg_t* event) {
    if (event->data.evt_system_soft_timer.handle == LCD_TIMER_HANDLE) {
        displayUpdate();
    }
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
               (signByte << 24);
    // value = 10^exponent * mantissa, pow() returns a double type
    return (int32_t) (pow(10, exponent) * mantissa);
} // gattFloat32ToInt

void send_health_data_over_bluetooth(uint8_t heartbeat_value)
{
    sl_status_t sc = 0;
    uint8_t heartbeat_buffer[2] = {0};
    uint8_t *p = &heartbeat_buffer[0];

    //read is again to set only the current status of the button in the local database
    UINT8_TO_BITSTREAM(p, heartbeat_value);

    if((ble_data.s_IndicationInFlight == false))
    {
      sc = sl_bt_gatt_server_send_indication(ble_data.s_ConnectionHandle,gattdb_heartbeat_state,
                                             1,&heartbeat_buffer[0]); //slcp

      if(sc != SL_STATUS_OK)
      {
          LOG_ERROR("sl_bt_gatt_server_send_indication returned != 0 status=0x%04x\r", (unsigned int) sc);
      }
      else
      {
          ble_data.s_IndicationInFlight = true;
      }
    }
    else
    {
        LOG_INFO("button:In flight\r");
        //write_to_buffer(gattdb_button_state, 2, button_buffer); //slcp
    }
}
