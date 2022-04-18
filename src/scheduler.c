/*
 * scheduler.c - Scheduler functions
 *
 *  Created on: Sep 12, 2021
 *      Author: vishnu and rishab
 */

#include "scheduler.h"

#define INCLUDE_LOG_DEBUG 0
#include "src/log.h"
uint32_t event_requested;
/************************************************/
/****************Event Handlers******************/
/************************************************/
uint32_t Scheduler_GetNextEvent()
{
  uint32_t theEvent = 0;

  //Exit critical section
  CORE_DECLARE_IRQ_STATE;

  //Enter critical section
  CORE_ENTER_CRITICAL();

  theEvent = (uint32_t)event_requested;
  event_requested = ev_NONE;

  //Exit critical section
  CORE_EXIT_CRITICAL();

  return (theEvent);
}

void Scheduler_SetEvent_LETIMER0_UF(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    //EventQ_EnqueueEvent(ev_LETIMER0_UF);
    SysTick_IncrementCounter();
#if NO_BL
    sl_bt_external_signal(ev_LETIMER0_UF);
#else
    event_requested = ev_LETIMER0_UF;
#endif
    CORE_EXIT_CRITICAL();
}


void Scheduler_SetEvent_LETIMER0_COMP1(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    //EventQ_EnqueueEvent(ev_LETIMER0_COMP1);
#if NO_BL
    sl_bt_external_signal(ev_LETIMER0_COMP1);
#else
    event_requested = ev_LETIMER0_COMP1;
#endif
    CORE_EXIT_CRITICAL();
}


void Scheduler_SetEvent_I2C0_TRANSFER_DONE(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    //EventQ_EnqueueEvent(ev_I2C0_TRANSFER_DONE);
#if NO_BL
    sl_bt_external_signal(ev_I2C0_TRANSFER_DONE);
#else
    event_requested = ev_I2C0_TRANSFER_DONE;
#endif
    CORE_EXIT_CRITICAL();
}


void Scheduler_SetEvent_PB0_PRESSED(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
#if NO_BL
    //EventQ_EnqueueEvent(ev_PB0_PRESSED);
    sl_bt_external_signal(ev_PB0_PRESSED);
#else
    event_requested = ev_PB0_PRESSED;
#endif
    CORE_EXIT_CRITICAL();
}


void Scheduler_SetEvent_PB0_RELEASED(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    //EventQ_EnqueueEvent(ev_PB0_RELEASED);
#if NO_BL
    sl_bt_external_signal(ev_PB0_RELEASED);
#else
    event_requested = ev_PB0_RELEASED;
#endif
    CORE_EXIT_CRITICAL();
}


void Scheduler_SetEvent_PB1_PRESSED(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    //EventQ_EnqueueEvent(ev_PB0_PRESSED);
#if NO_BL
    sl_bt_external_signal(ev_PB1_PRESSED);
#else
    event_requested = ev_PB1_PRESSED;
#endif

    CORE_EXIT_CRITICAL();
}


void Scheduler_SetEvent_PB1_RELEASED(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    //EventQ_EnqueueEvent(ev_PB0_RELEASED);
#if NO_BL
    sl_bt_external_signal(ev_PB1_RELEASED);
#else
    event_requested = ev_PB1_RELEASED;
#endif
    CORE_EXIT_CRITICAL();
}

void Scheduler_SetEvent_SPI_TX(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
#if NO_BL
    sl_bt_external_signal(ev_SPI_TX);
#else
    event_requested = ev_SPI_TX;
#endif

    CORE_EXIT_CRITICAL();
}

void Scheduler_SetEvent_SPI_RX(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
#if NO_BL
    sl_bt_external_signal(ev_SPI_RX);
#else
    event_requested = ev_SPI_RX;
#endif

    CORE_EXIT_CRITICAL();
}

/************************************************/
/***************Client Functions*****************/
/************************************************/
void AssetMonitoringSystem_StateMachine(sl_bt_msg_t* event) {
    asset_monitoring_state_t current_state;
    //static activity_monitoring_state_t next_state = HEARTBEAT_INIT;
    static asset_monitoring_state_t next_state = BME280_INIT_CONFIG;
    if (SL_BT_MSG_ID(event->header) != sl_bt_evt_system_external_signal_id) {
        return;
    }

    current_state = next_state;

    switch (current_state) {
        case BME280_INIT_CONFIG:
            next_state = init_bme280_machine(event);
            LOG_INFO("BME280_INIT_CONFIG\r");
            break;

        case BNO055_INIT_CONFIG:
            next_state = init_bno055_machine(event);
            LOG_INFO("BNO055_INIT_CONFIG\r");
            break;

        case BME280_READ:
            next_state = bme280_read_machine(event);
            LOG_INFO("BME280_READ\r");
            break;

        case BNO055_READ:
            next_state = bno055_read_machine(event);
            LOG_INFO("BNO055_READ\r");
            break;
    }
}

#if 0
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
                BleClient_RequestAccelServiceInfo(ble_data);
                displayPrintf(DISPLAY_ROW_CONNECTION, "Handling Indications");
                next_state = RECEIVING_ACCEL_SERVICE_INFO;
            }
            break;

        case RECEIVING_ACCEL_SERVICE_INFO:
            if (SL_BT_MSG_ID(event->header) == sl_bt_evt_gatt_procedure_completed_id) {
                LOG_INFO("RECEIVING_HEALTH_SERVICE_INFO\r");
                BleClient_RequestAccelCharacteristicInfo(ble_data);
                next_state = RECEIVING_ACCEL_CHARACTERISTIC_INFO;
            }
            break;

        case RECEIVING_ACCEL_CHARACTERISTIC_INFO:
            if (SL_BT_MSG_ID(event->header) == sl_bt_evt_gatt_procedure_completed_id) {
                LOG_INFO("RECEIVING_HEALTH_CHARACTERISTIC_INFO\r");
                BleClient_EnableAccelIndications(ble_data);
                next_state = ENABLING_ACCEL_INDICATIONS;
            }
            break;

        case ENABLING_ACCEL_INDICATIONS:
            if (SL_BT_MSG_ID(event->header) == sl_bt_evt_gatt_procedure_completed_id) {
                LOG_INFO("ENABLING_HEALTH_INDICATIONS\r");
                displayPrintf(DISPLAY_ROW_CONNECTION, "Handling Indications");
                next_state = DISCOVERED;
            }
            break;

        case DISCOVERED:
            LOG_INFO("DISCOVERED\r");
            break;

        default:
          break;
    }
}

void BleClient_RequestHealthServiceInfo(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_status = sl_bt_gatt_discover_primary_services_by_uuid(ble_data->c_ConnectionHandle,
                                                                  sizeof(ble_data->s_HealthService),
                                                                  ble_data->s_HealthService);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("s_health_service:: sl_bt_gatt_discover_primary_services_by_uuid: %x\r", ble_status);
    }
}

void BleClient_RequestHealthCharacteristicInfo(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_status = sl_bt_gatt_discover_characteristics_by_uuid(ble_data->c_ConnectionHandle,
                                                             ble_data->c_HealthServiceHandle,
                                                             sizeof(ble_data->s_HealthChar),
                                                             ble_data->s_HealthChar);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("health_char:: sl_bt_gatt_discover_characteristics_by_uuid: %x\r", ble_status);
    }
}

void BleClient_EnableHealthIndications(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_status = sl_bt_gatt_set_characteristic_notification(ble_data->c_ConnectionHandle,
                                                            ble_data->c_HealthCharacteristicHandle,
                                                            sl_bt_gatt_indication);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_gatt_set_characteristic_notification: %x\r", ble_status);
    }
}

void BleClient_RequestAccelServiceInfo(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_status = sl_bt_gatt_discover_primary_services_by_uuid(ble_data->c_ConnectionHandle,
                                                                  sizeof(ble_data->s_AccelService),
                                                                  ble_data->s_AccelService);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("s_health_service:: sl_bt_gatt_discover_primary_services_by_uuid: %x\r", ble_status);
    }
}

void BleClient_RequestAccelCharacteristicInfo(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_status = sl_bt_gatt_discover_characteristics_by_uuid(ble_data->c_ConnectionHandle,
                                                             ble_data->c_AccelServiceHandle,
                                                             sizeof(ble_data->s_AccelChar),
                                                             ble_data->s_AccelChar);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("health_char:: sl_bt_gatt_discover_characteristics_by_uuid: %x\r", ble_status);
    }
}

void BleClient_EnableAccelIndications(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_status = sl_bt_gatt_set_characteristic_notification(ble_data->c_ConnectionHandle,
                                                            ble_data->c_AccelCharacteristicHandle,
                                                            sl_bt_gatt_indication);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_gatt_set_characteristic_notification: %x\r", ble_status);
    }
    gpioDebugLEDSetOn();
}

void BleClient_RestartScanning(void) {
    sl_status_t ble_status;

    ble_status = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_scanner_start: %x\r", ble_status);
    }
}
#endif

