/*
 * scheduler.c - Scheduler functions
 *
 *  Created on: Sep 12, 2021
 *      Author: vishn
 */

#include "scheduler.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"


/************************************************/
/****************Event Handlers******************/
/************************************************/


void Scheduler_SetEvent_LETIMER0_UF(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    //EventQ_EnqueueEvent(ev_LETIMER0_UF);
    SysTick_IncrementCounter();
    sl_bt_external_signal(ev_LETIMER0_UF);
    CORE_EXIT_CRITICAL();
}


void Scheduler_SetEvent_LETIMER0_COMP1(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    //EventQ_EnqueueEvent(ev_LETIMER0_COMP1);
    sl_bt_external_signal(ev_LETIMER0_COMP1);
    CORE_EXIT_CRITICAL();
}


void Scheduler_SetEvent_I2C0_TRANSFER_DONE(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    //EventQ_EnqueueEvent(ev_I2C0_TRANSFER_DONE);
    sl_bt_external_signal(ev_I2C0_TRANSFER_DONE);
    CORE_EXIT_CRITICAL();
}


void Scheduler_SetEvent_PB0_PRESSED(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    //EventQ_EnqueueEvent(ev_PB0_PRESSED);
    sl_bt_external_signal(ev_PB0_PRESSED);
    CORE_EXIT_CRITICAL();
}


void Scheduler_SetEvent_PB0_RELEASED(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    //EventQ_EnqueueEvent(ev_PB0_RELEASED);
    sl_bt_external_signal(ev_PB0_RELEASED);
    CORE_EXIT_CRITICAL();
}


void Scheduler_SetEvent_PB1_PRESSED(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    //EventQ_EnqueueEvent(ev_PB0_PRESSED);
    sl_bt_external_signal(ev_PB1_PRESSED);
    CORE_EXIT_CRITICAL();
}


void Scheduler_SetEvent_PB1_RELEASED(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    //EventQ_EnqueueEvent(ev_PB0_RELEASED);
    sl_bt_external_signal(ev_PB1_RELEASED);
    CORE_EXIT_CRITICAL();
}


/************************************************/
/***************Server Functions*****************/
/************************************************/

void ActivityMonitoringSystem_StateMachine(sl_bt_msg_t* event) {
    activity_monitoring_state_t current_state;
    //static activity_monitoring_state_t next_state = HEARTBEAT_INIT;
    static activity_monitoring_state_t next_state = ACCEL_INIT;
    if (SL_BT_MSG_ID(event->header) != sl_bt_evt_system_external_signal_id) {
        return;
    }

    current_state = next_state;

    switch (current_state) {
        case ACCEL_INIT:
            next_state = MMA8452Q_InitStateMachine(event);
            //LOG_INFO("ACCEL_INIT\r");
            break;

        case HEARTBEAT_INIT:
            next_state = init_heartbeat_machine(event);
            //LOG_INFO("HEARTBEAT_INIT\r");
            break;

        case HEARTBEAT_CONFIGURE:
            next_state = config_heartbeat_machine(event);
            //LOG_INFO("HEARTBEAT_CONFIGURE\r");
            break;

        case HEARTBEAT_READ:
            next_state = heartbeat_machine_running(event);
            //LOG_INFO("HEARTBEAT_READ\r");
            break;

        case ACCEL_READ:
            next_state = MMA8452Q_ReadStateMachine(event);
            //LOG_INFO("ACCEL_READ\r");
            break;
    }
}


/************************************************/
/***************Client Functions*****************/
/************************************************/

void BleClient_DiscoveryStateMachine(sl_bt_msg_t* event) {
    disc_fsm_state_t current_state;
    static disc_fsm_state_t next_state = SCANNING;
    ble_data_struct_t* ble_data = BLE_GetDataStruct();

    current_state = next_state;

    if (SL_BT_MSG_ID(event->header) == sl_bt_evt_connection_closed_id) {
        BleClient_RestartScanning();
        next_state = SCANNING;
        return;
    }

    switch (current_state) {
        case SCANNING:
            if (SL_BT_MSG_ID(event->header) == sl_bt_evt_connection_opened_id) {
                LOG_INFO("SCANNING\r");
                BleClient_RequestHealthServiceInfo(ble_data);
                next_state = RECEIVING_HEALTH_SERVICE_INFO;
            }
            break;

        case RECEIVING_HEALTH_SERVICE_INFO:
            if (SL_BT_MSG_ID(event->header) == sl_bt_evt_gatt_procedure_completed_id) {
                LOG_INFO("RECEIVING_HEALTH_SERVICE_INFO\r");
                BleClient_RequestHealthCharacteristicInfo(ble_data);
                next_state = RECEIVING_HEALTH_CHARACTERISTIC_INFO;
            }
            break;

        case RECEIVING_HEALTH_CHARACTERISTIC_INFO:
            if (SL_BT_MSG_ID(event->header) == sl_bt_evt_gatt_procedure_completed_id) {
                LOG_INFO("RECEIVING_HEALTH_CHARACTERISTIC_INFO\r");
                BleClient_EnableHealthIndications(ble_data);
                next_state = ENABLING_HEALTH_INDICATIONS;
            }
            break;

        case ENABLING_HEALTH_INDICATIONS:
            if (SL_BT_MSG_ID(event->header) == sl_bt_evt_gatt_procedure_completed_id) {
                LOG_INFO("ENABLING_HEALTH_INDICATIONS\r");
                //BleClient_EnableButtonIndications(ble_data);
                next_state = DISCOVERED;
            }
            break;

        case DISCOVERED:
            //LOG_INFO("DISCOVERED\r");
            break;

#if 0
        case RECEIVING_TEMP_SERVICE_INFO:
            if (SL_BT_MSG_ID(event->header) == sl_bt_evt_gatt_procedure_completed_id) {
                //BleClient_RequestTemperatureCharacteristicInfo(ble_data);
                BleClient_RequestHealthCharacteristicInfo(ble_data);
                next_state = RECEIVING_TEMP_CHARACTERISTIC_INFO;
            }
            break;


        case RECEIVING_TEMP_CHARACTERISTIC_INFO:
            if (SL_BT_MSG_ID(event->header) == sl_bt_evt_gatt_procedure_completed_id) {
                BleClient_RequestButtonServiceInfo(ble_data);
                next_state = RECEIVING_BUTTON_SERVICE_INFO;
            }
            break;

        case RECEIVING_BUTTON_SERVICE_INFO:
            if (SL_BT_MSG_ID(event->header) == sl_bt_evt_gatt_procedure_completed_id) {
                BleClient_RequestButtonCharacteristicInfo(ble_data);
                next_state = RECEIVING_BUTTON_CHARACTERISTIC_INFO;
            }
            break;

        case RECEIVING_BUTTON_CHARACTERISTIC_INFO:
            if (SL_BT_MSG_ID(event->header) == sl_bt_evt_gatt_procedure_completed_id) {
                BleClient_EnableTemperatureIndications(ble_data);
                next_state = ENABLING_TEMP_INDICATIONS;
            }
            break;

        case ENABLING_TEMP_INDICATIONS:
            if (SL_BT_MSG_ID(event->header) == sl_bt_evt_gatt_procedure_completed_id) {
                BleClient_EnableButtonIndications(ble_data);
                next_state = ENABLING_BUTTON_INDICATIONS;
            }
            break;

        case ENABLING_BUTTON_INDICATIONS:
            if (SL_BT_MSG_ID(event->header) == sl_bt_evt_gatt_procedure_completed_id) {
                BleClient_SetDisplayingOfIndications(ble_data);
                next_state = DISCOVERED;
            }
            break;
#endif

        default:
          break;
    }
}


void BleClient_RequestHealthServiceInfo(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_status = sl_bt_gatt_discover_primary_services_by_uuid(ble_data->c_ConnectionHandle,
                                                                  sizeof(ble_data->s_health_service),
                                                                  ble_data->s_health_service);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("s_health_service:: sl_bt_gatt_discover_primary_services_by_uuid: %x\r", ble_status);
    }
}

void BleClient_RequestHealthCharacteristicInfo(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_status = sl_bt_gatt_discover_characteristics_by_uuid(ble_data->c_ConnectionHandle,
                                                             ble_data->c_health_service_handle,
                                                             sizeof(ble_data->s_health_char),
                                                             ble_data->s_health_char);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("health_char:: sl_bt_gatt_discover_characteristics_by_uuid: %x\r", ble_status);
    }
}

void BleClient_EnableHealthIndications(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_status = sl_bt_gatt_set_characteristic_notification(ble_data->c_ConnectionHandle,
                                                            ble_data->c_health_characteristic_handle,
                                                            sl_bt_gatt_indication);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_gatt_set_characteristic_notification: %x\r", ble_status);
    }
}



void BleClient_RequestTemperatureServiceInfo(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_status = sl_bt_gatt_discover_primary_services_by_uuid(ble_data->c_ConnectionHandle,
                                                                  sizeof(thermo_service),
                                                                  thermo_service);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_gatt_discover_primary_services_by_uuid: %x\r", ble_status);
    }
}


void BleClient_RequestTemperatureCharacteristicInfo(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_status = sl_bt_gatt_discover_characteristics_by_uuid(ble_data->c_ConnectionHandle,
                                                             ble_data->c_TemperatureServiceHandle,
                                                             sizeof(thermo_char),
                                                             thermo_char);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_gatt_discover_characteristics_by_uuid: %x\r", ble_status);
    }
}


void BleClient_RequestButtonServiceInfo(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_status = sl_bt_gatt_discover_primary_services_by_uuid(ble_data->c_ConnectionHandle,
                                                                  sizeof(button_service),
                                                                  button_service);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_gatt_discover_primary_services_by_uuid: %x\r", ble_status);
    }
}


void BleClient_RequestButtonCharacteristicInfo(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_status = sl_bt_gatt_discover_characteristics_by_uuid(ble_data->c_ConnectionHandle,
                                                             ble_data->c_ButtonServiceHandle,
                                                             sizeof(button_char),
                                                             button_char);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_gatt_discover_characteristics_by_uuid: %x\r", ble_status);
    }
}


void BleClient_EnableTemperatureIndications(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_status = sl_bt_gatt_set_characteristic_notification(ble_data->c_ConnectionHandle,
                                                            ble_data->c_TemperatureCharacteristicHandle, 0x02);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_gatt_set_characteristic_notification: %x\r", ble_status);
    }
}


void BleClient_EnableButtonIndications(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_data->c_TemperatureIndicating = true;
    ble_status = sl_bt_gatt_set_characteristic_notification(ble_data->c_ConnectionHandle,
                                                            ble_data->c_ButtonCharacteristicHandle, 0x02);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_gatt_set_characteristic_notification: %x\r", ble_status);
    }
}


void BleClient_SetDisplayingOfIndications(ble_data_struct_t* ble_data) {
    ble_data->c_ButtonIndicating = true;
    displayPrintf(DISPLAY_ROW_CONNECTION, "Handling Indications");
}


void BleClient_RestartScanning(void) {
    sl_status_t ble_status;

    ble_status = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_scanner_start: %x\r", ble_status);
    }
}
