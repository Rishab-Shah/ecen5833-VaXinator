/*
 * ble.c
 *
 *  Created on: Sep 27, 2021
 *      Author: vishnu
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

// Health service UUID defined by Bluetooth SIG
static const uint8_t health_service[HEALTH_SIZE] = {
    0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87,
    0x3e, 0x43, 0xc8,0x38, 0x03, 0x00, 0x00, 0x00
};

// Health characteristic UUID defined by Bluetooth SIG
static const uint8_t health_char[HEALTH_SIZE] = {
    0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87,
    0x3e, 0x43, 0xc8,0x38, 0x04, 0x00, 0x00, 0x00
};

// Accel service UUID defined by Bluetooth SIG
static const uint8_t accel_service[16] = {
    0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87,
    0x3e, 0x43, 0xc8, 0x38, 0x01, 0x00, 0x00, 0x00
};

// Accel characteristic UUID defined by Bluetooth SIG
static const uint8_t accel_char[16] = {
    0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87,
    0x3e, 0x43, 0xc8, 0x38, 0x02, 0x00, 0x00, 0x00
};

ble_data_struct_t ble_data = { 0 };

static indication_struct_t indication_q[INDICATION_QUEUE_SIZE];
static uint8_t rd_ptr = 0;
static uint8_t wr_ptr = 0;
static uint8_t q_size = 0;


/************************************************/
/***************Server Functions*****************/
/************************************************/

/*
 * Handles BLE Server boot event
 *
 * @param None
 *
 * @return None
 *
 * Attribution: The sl_bt_evt_system_boot_id code was sourced
 *    from the soc_thermometer example project.
 */
void BleServer_HandleBootEvent(void);


/*
 * Handles BLE Server connection opened event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BleServer_HandleConnectionOpenedEvent(sl_bt_msg_t* event);


/*
 * Handles BLE Server connection closed event
 *
 * @param None
 *
 * @return None
 */
void BleServer_HandleConnectionClosedEvent(void);


/*
 * Handles BLE Server connection parameters event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BleServer_HandleConnectionParametersEvent(sl_bt_msg_t* event);


/*
 * Handles BLE Server external signal event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BleServer_HandleExternalSignalEvent(sl_bt_msg_t* event);


/*
 * Handles BLE Server characteristic status event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BleServer_HandleCharacteristicStatusEvent(sl_bt_msg_t* event);


/*
 * Handles BLE Server indication timeout event
 *
 * @param None
 *
 * @return None
 */
void BleServer_HandleIndicationTimeoutEvent(void);


/*
 * Handles BLE Client bonding confirm event
 *
 * @param None
 *
 * @return None
 */
void BleClient_HandleBondingConfirmEvent(void);

/*
 * Handles BLE Server bonding confirm event
 *
 * @param None
 *
 * @return None
 */
void BleServer_HandleBondingConfirmEvent(void);


/*
 * Handles BLE Server passkey confirm event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BleServer_HandlePasskeyConfirmEvent(sl_bt_msg_t* event);


/*
 * Handles BLE Server bonded event
 *
 * @param None
 *
 * @return None
 */
void BleServer_HandleBondedEvent(void);


/*
 * Handles BLE Server bonding failed event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BleServer_HandleBondingFailedEvent(sl_bt_msg_t* event);


/*
 * Handles BLE Server soft timer event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BleServer_HandleSoftTimerEvent(sl_bt_msg_t* event);


/************************************************/
/***************Client Functions*****************/
/************************************************/


/*
 * Handles BLE Client boot event
 *
 * @param None
 *
 * @return None
 *
 * Attribution: The sl_bt_evt_system_boot_id code was sourced
 *    from the soc_thermometer_client example project.
 */
void BleClient_HandleBootEvent(void);


/*
 * Handles BLE Client scan report event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BleClient_HandleScanReportEvent(sl_bt_msg_t* event);


/*
 * Handles BLE Client connection opened event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BleClient_HandleConnectionOpenedEvent(sl_bt_msg_t* event);


/*
 * Handles BLE Client connection closed event
 *
 * @param None
 *
 * @return None
 */
void BleClient_HandleConnectionClosedEvent(void);


/*
 * Handles BLE Client connection parameters event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BleClient_HandleConnectionParametersEvent(sl_bt_msg_t* event);


/*
 * Handles BLE Client gatt procedure completed event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BleClient_HandleGattProcedureCompleted(sl_bt_msg_t* event);


/*
 * Handles BLE Client gatt service event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BleClient_HandleGattServiceEvent(sl_bt_msg_t* event);


/*
 * Handles BLE Client gatt characteristic event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BleClient_HandleGattCharacteristicEvent(sl_bt_msg_t* event);


/*
 * Handles BLE Client gatt characteristic value event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BleClient_HandleGattCharacteristicValueEvent(sl_bt_msg_t* event);


/*
 * Handles BLE Client external signal event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BleClient_HandleExternalSignalEvent(sl_bt_msg_t* event);


/*
 * Handles BLE Client passkey confirm event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BleClient_HandlePasskeyConfirmEvent(sl_bt_msg_t* event);


/*
 * Handles BLE Client bonded event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BleClient_HandleBondedEvent(void);


/*
 * Handles BLE Client bonding failed event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BleClient_HandleBondingFailedEvent(sl_bt_msg_t* event);


/*
 * Handles BLE Client soft timer event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BleClient_HandleSoftTimerEvent(sl_bt_msg_t* event);

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
    bd_addr server_addr           = SERVER_BT_ADDRESS;
    ble_data.serverAddress        = server_addr;
    ble_data.serverAddressType    = 0;
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
#if CONNECTION_PARAMETER_DEBUG_PRINTS
            BleServer_HandleConnectionParametersEvent(event);
#endif
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
#if CONNECTION_PARAMETER_DEBUG_PRINTS
            BleClient_HandleConnectionParametersEvent(event);
#endif
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
    ble_data.s_HealthIndicating = false;
    ble_data.s_AccelIndication = false;

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

    gpioLed0SetOff();
    gpioLed1SetOff();

}

void BleServer_HandleConnectionParametersEvent(sl_bt_msg_t* event) {
    LOG_INFO("handle: %d, interval, %d, latency: %d, timeout: %x\r",
             event->data.evt_connection_parameters.connection,
             event->data.evt_connection_parameters.interval,
             event->data.evt_connection_parameters.latency,
             event->data.evt_connection_parameters.timeout);
}

void BleServer_HandleExternalSignalEvent(sl_bt_msg_t* event) {
    //sl_status_t ble_status;
    //indication_struct_t indication;
    sl_status_t sc;

    if(event->data.evt_system_external_signal.extsignals == ev_PB0_PRESSED)
    {
        if((ble_data.s_Bonded == false) && (ble_data.s_ClientConnected == true))
        {
            sc = sl_bt_sm_passkey_confirm(ble_data.s_ConnectionHandle,1);
            if(sc != SL_STATUS_OK)
            {
                LOG_ERROR("sl_bt_sm_passkey_confirm:: status=0x%04x\r",(unsigned int)sc);
            }
        }
    }
}

void BleServer_HandleCharacteristicStatusEvent(sl_bt_msg_t* event) {
    uint8_t status_flags;
    uint16_t client_flags;
    uint16_t characteristic_flags;

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
                ble_data.s_HealthIndicating = false;
                gpioLed1SetOff();
            }
            if(client_flags == gatt_indication)
            {
                /* app gave enable indication */
                ble_data.s_HealthIndicating = true;
                gpioLed1SetOn();
            }
        }
        if(status_flags == sl_bt_gatt_server_confirmation)
        {
            ble_data.s_IndicationInFlight = false;
        }
    }
    else if (characteristic_flags == gattdb_xyz_accel_state) {
        if(status_flags == sl_bt_gatt_server_client_config)
        {
            if(client_flags == gatt_disable)
            {
                /* app gave disable indication */
                ble_data.s_AccelIndication = false;
                gpioLed1SetOff();
            }
            if(client_flags == gatt_indication)
            {
                /* app gave enable indication */
                ble_data.s_AccelIndication = true;
                gpioLed0SetOn();
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

void BleClient_HandleBondingConfirmEvent(void) {
    sl_status_t ble_status;

    ble_status = sl_bt_sm_bonding_confirm(ble_data.c_ConnectionHandle, 1);
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
    gpioLed0SetOff();
    gpioLed1SetOff();
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

    //health service
    memcpy(ble_data.s_HealthService,health_service,HEALTH_SIZE*sizeof(ble_data.s_HealthService[0]));
    memcpy(ble_data.s_HealthChar,health_char,HEALTH_SIZE*sizeof(ble_data.s_HealthChar[0]));

    memcpy(ble_data.s_AccelService, accel_service, ACCEL_SIZE);
    memcpy(ble_data.s_AccelChar, accel_char, ACCEL_SIZE);
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
    displayPrintf(DISPLAY_ROW_X, "");
    displayPrintf(DISPLAY_ROW_Y, "");
    displayPrintf(DISPLAY_ROW_Z, "");
    displayPrintf(DISPLAY_ROW_HEARTBEAT, "");
    displayPrintf(DISPLAY_ROW_ACTIVITY_STATE, "");

    gpioLed0SetOff();
    gpioLed1SetOff();
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
}

void BleClient_HandleGattServiceEvent(sl_bt_msg_t* event) {
    if(0 == (memcmp(event->data.evt_gatt_service.uuid.data,ble_data.s_HealthService,sizeof(ble_data.s_HealthService))))
    {
        ble_data.c_HealthServiceHandle = event->data.evt_gatt_service.service;
    }
    else if(!(memcmp(event->data.evt_gatt_service.uuid.data, ble_data.s_AccelService, sizeof(ble_data.s_AccelService))))
    {
        ble_data.c_AccelServiceHandle = event->data.evt_gatt_service.service;
    }
}

void BleClient_HandleGattCharacteristicEvent(sl_bt_msg_t* event) {
    // Identify the correct characteristic //multiple firing
    if(0 == (memcmp(event->data.evt_gatt_characteristic.uuid.data, ble_data.s_HealthChar, sizeof(ble_data.s_HealthChar))))
    {
        ble_data.c_HealthCharacteristicHandle = event->data.evt_gatt_characteristic.characteristic;
    }
    else if(!(memcmp(event->data.evt_gatt_characteristic.uuid.data, ble_data.s_AccelChar, sizeof(ble_data.s_AccelChar))))
    {
        ble_data.c_AccelCharacteristicHandle = event->data.evt_gatt_characteristic.characteristic;
    }
}

void BleClient_HandleGattCharacteristicValueEvent(sl_bt_msg_t* event) {
    sl_status_t sc = 0;
    int16_t x, y, z;
    if((event->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication)
      && (event->data.evt_gatt_characteristic_value.characteristic == ble_data.c_HealthCharacteristicHandle) )
    {
        ble_data.c_HealthCharValue = &(event->data.evt_gatt_characteristic_value.value.data[0]);
        ble_data.c_HealthValue = ble_data.c_HealthCharValue[0];
        LOG_INFO("heartbeat_value = %d\r",ble_data.c_HealthValue);
        displayPrintf(DISPLAY_ROW_HEARTBEAT, "HeartBeat = %d",ble_data.c_HealthValue);

        sc = sl_bt_gatt_send_characteristic_confirmation(event->data.evt_gatt_characteristic_value.connection);
        if(sc != SL_STATUS_OK)
        {
            LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation:: status=0x%04x\r",(unsigned int)sc);
        }
    }
    else if ((event->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication)
      && (event->data.evt_gatt_characteristic_value.characteristic == ble_data.c_AccelCharacteristicHandle)) {
        memcpy(&(ble_data.c_AccelBuffer[0]), &(event->data.evt_gatt_characteristic_value.value.data[0]), 6);

        sc = sl_bt_gatt_send_characteristic_confirmation(event->data.evt_gatt_characteristic_value.connection);
        if(sc != SL_STATUS_OK)
        {
            LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation:: status=0x%04x\r",(unsigned int)sc);
        }

        x = (ble_data.c_AccelBuffer[0] << 8) | ble_data.c_AccelBuffer[1];
        y = (ble_data.c_AccelBuffer[2] << 8) | ble_data.c_AccelBuffer[3];
        z = (ble_data.c_AccelBuffer[4] << 8) | ble_data.c_AccelBuffer[5];

        x = x >> 4;
        y = y >> 4;
        z = z >> 4;

        displayPrintf(DISPLAY_ROW_X, "X = %d mg", x);
        displayPrintf(DISPLAY_ROW_Y, "Y = %d mg", y);
        displayPrintf(DISPLAY_ROW_Z, "Z = %d mg", z);

        if (ble_data.c_HealthValue == 0) {
            displayPrintf(DISPLAY_ROW_ACTIVITY_STATE, "Reading Heartbeat...");
        }
        else if (ble_data.c_HealthValue < 80) {
            if ((ble_data.c_AccelX == 0) && (ble_data.c_AccelY == 0) && (ble_data.c_AccelZ == 0)) {
                displayPrintf(DISPLAY_ROW_ACTIVITY_STATE, "Relaxed State");
            }
            else if (((x > ble_data.c_AccelX - 10) && (x < ble_data.c_AccelX + 10)) && ((y > ble_data.c_AccelY - 10) && (y < ble_data.c_AccelY + 10)) && ((z > ble_data.c_AccelZ - 10) && (z < ble_data.c_AccelZ + 10))) {
                displayPrintf(DISPLAY_ROW_ACTIVITY_STATE, "Relaxed State");
            }
            else {
                displayPrintf(DISPLAY_ROW_ACTIVITY_STATE, "Low Activity State");
            }
        }
        else if (ble_data.c_HealthValue >= 80 && ble_data.c_HealthValue < 90) {
            if ((ble_data.c_AccelX == 0) && (ble_data.c_AccelY == 0) && (ble_data.c_AccelZ == 0)) {
                displayPrintf(DISPLAY_ROW_ACTIVITY_STATE, "Low Activity State");
            }
            else if (((x > ble_data.c_AccelX - 10) && (x < ble_data.c_AccelX + 10)) && ((y > ble_data.c_AccelY - 10) && (y < ble_data.c_AccelY + 10)) && ((z > ble_data.c_AccelZ - 10) && (z < ble_data.c_AccelZ + 10))) {
                displayPrintf(DISPLAY_ROW_ACTIVITY_STATE, "Low Activity State");
            }
            else {
                displayPrintf(DISPLAY_ROW_ACTIVITY_STATE, "Mid Activity State");
            }
        }
        else if (ble_data.c_HealthValue >= 90 && ble_data.c_HealthValue < 100) {
            if ((ble_data.c_AccelX == 0) && (ble_data.c_AccelY == 0) && (ble_data.c_AccelZ == 0)) {
                displayPrintf(DISPLAY_ROW_ACTIVITY_STATE, "Mid Activity State");
            }
            else if (((x > ble_data.c_AccelX - 10) && (x < ble_data.c_AccelX + 10)) && ((y > ble_data.c_AccelY - 10) && (y < ble_data.c_AccelY + 10)) && ((z > ble_data.c_AccelZ - 10) && (z < ble_data.c_AccelZ + 10))) {
                displayPrintf(DISPLAY_ROW_ACTIVITY_STATE, "Mid Activity State");
            }
            else {
                displayPrintf(DISPLAY_ROW_ACTIVITY_STATE, "High Activity State");
            }
        }
        else if (ble_data.c_HealthValue >= 100) {
            displayPrintf(DISPLAY_ROW_ACTIVITY_STATE, "High Activity State");
        }

        ble_data.c_AccelX = x;
        ble_data.c_AccelY = y;
        ble_data.c_AccelZ = z;
    }
}

void BleClient_HandleExternalSignalEvent(sl_bt_msg_t* event) {
    sl_status_t sc;
    if(event->data.evt_system_external_signal.extsignals == ev_PB0_PRESSED)
    {
        if((ble_data.c_Bonded == false) && (ble_data.c_Connected == true))
        {
            sc = sl_bt_sm_passkey_confirm(ble_data.c_ConnectionHandle,1);
            if(sc != SL_STATUS_OK)
            {
                LOG_ERROR("sl_bt_sm_passkey_confirm:: status=0x%04x\r",(unsigned int)sc);
            }
        }
    }
    else if(event->data.evt_system_external_signal.extsignals == ev_PB1_PRESSED)
    {
        sc = sl_bt_gatt_read_characteristic_value(ble_data.c_ConnectionHandle,
                                                  ble_data.c_HealthCharacteristicHandle);
        if(sc != SL_STATUS_OK)
        {
            LOG_ERROR("sl_bt_gatt_read_characteristic_value:: status=0x%04x\r",(unsigned int)sc);
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
    gpioLed0SetOff();
    gpioLed1SetOff();
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

void BleServer_SendHearbeatDataToClient(uint8_t heartbeat_value)
{
    sl_status_t sc = 0;
    uint8_t heartbeat_buffer[2] = {0};
    uint8_t *p = &heartbeat_buffer[0];

    indication_struct_t indication;

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
        indication.characteristicHandle = gattdb_heartbeat_state;
        memcpy(&indication.buff[0], &heartbeat_buffer[0], 1);
        indication.bufferLen = 1;
        IndicationQ_Enqueue(indication);
    }
}

void BleServer_SendAccelDataToClient(uint8_t* accel_buff) {
    sl_status_t ble_status;
    indication_struct_t indication;

    if (!(ble_data.s_IndicationInFlight)) {
        ble_status = sl_bt_gatt_server_send_indication(ble_data.s_ConnectionHandle, gattdb_xyz_accel_state,
                                                       6, accel_buff);
        if (ble_status != SL_STATUS_OK) {
            LOG_ERROR("sl_bt_gatt_server_send_indication returned != 0 status=0x%04x\r", (unsigned int)ble_status);
        }
        else {
            ble_data.s_IndicationInFlight = true;
        }
    }
    else {
        indication.characteristicHandle = gattdb_xyz_accel_state;
        memcpy(&indication.buff[0], &accel_buff[0], 6);
        indication.bufferLen = 6;
        IndicationQ_Enqueue(indication);
    }
}
