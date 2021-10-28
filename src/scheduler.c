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


void BleServer_TemperatureStateMachine(sl_bt_msg_t* event) {
    temp_fsm_state_t current_state;
    static temp_fsm_state_t next_state = PERIOD_WAIT;
    ble_data_struct_t* ble_data = BLE_GetDataStruct();
    ble_ext_signal_event_t ev;

    if (!(ble_data->s_ClientConnected) || !(ble_data->s_TemperatureIndicating)) {
        if (ble_data->s_ReadingTemp) {
            LETIMER_IntDisable(LETIMER0, LETIMER_IEN_COMP1);
            NVIC_DisableIRQ(I2C0_IRQn);
            I2C0_Teardown();
            displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
            ev = ev_SHUTDOWN;
        }
        ble_data->s_ReadingTemp = 0;
    }
    else if (!ble_data->s_ReadingTemp){
        return;
    }
    else {
        ev = event->data.evt_system_external_signal.extsignals;
    }

    current_state = next_state;

    switch (current_state) {
        case PERIOD_WAIT:
            if (ev == ev_LETIMER0_UF) {
                PowerUp();
                next_state = POWERING_UP;
            }
            else if (ev == ev_SHUTDOWN) {
                next_state = PERIOD_WAIT;
            }
            break;

        case POWERING_UP:
            if (ev == ev_LETIMER0_COMP1) {
                SendReadTempCommand();
                next_state = REQUEST_TEMP;
            }
            else if (ev == ev_SHUTDOWN) {
                next_state = PERIOD_WAIT;
            }
            break;

        case REQUEST_TEMP:
            if (ev == ev_I2C0_TRANSFER_DONE) {
                WaitForTempSensorReading();
                next_state = READING_TEMP;
            }
            else if (ev == ev_SHUTDOWN) {
                next_state = PERIOD_WAIT;
            }
            break;

        case READING_TEMP:
            if (ev == ev_LETIMER0_COMP1) {
                RequestTempSensorReading();
                next_state = RECEIVED_TEMP;
            }
            else if (ev == ev_SHUTDOWN) {
                next_state = PERIOD_WAIT;
            }
            break;

        case RECEIVED_TEMP:
            if (ev == ev_I2C0_TRANSFER_DONE) {
                ReadOutTempSensorReading(ble_data);
                ble_data->s_ReadingTemp = 0;
                next_state = PERIOD_WAIT;
            }
            else if (ev == ev_SHUTDOWN) {
                next_state = PERIOD_WAIT;
            }
            break;
    }
}


void PowerUp(void) {
    I2C0_Init();
    //I2C0_Enable(true);
    timerWaitUs_irq(TEMP_SENSOR_POWER_ON_WAIT_US);
}


void SendReadTempCommand(void) {
    LETIMER_IntDisable(LETIMER0, LETIMER_IEN_COMP1);
    sl_power_manager_add_em_requirement(EM1);

    I2C0_SendCommand(CMD_READ_TEMP);
}


void WaitForTempSensorReading(void) {
    sl_power_manager_remove_em_requirement(EM1);
    I2C0_DisableIntForTransfer();

    timerWaitUs_irq(TEMP_SENSOR_READ_TEMP_WAIT_US);
}


void RequestTempSensorReading(void) {
    LETIMER_IntDisable(LETIMER0, LETIMER_IEN_COMP1);
    sl_power_manager_add_em_requirement(EM1);

    I2C0_RequestRead();
}


void ReadOutTempSensorReading(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;
    uint8_t read_buff[2];
    int16_t temperature_code;
    int16_t temperature_C;
    indication_struct_t indication;
    uint32_t temp_C_float;
    uint8_t* p = indication.buff;

    sl_power_manager_remove_em_requirement(EM1);
    I2C0_DisableIntForTransfer();

    I2C0_ReadBytes(read_buff, 2);
    //I2C0_Enable(false);

    temperature_code = (read_buff[0] << 8) | read_buff[1];
    temperature_C = (175.25 * temperature_code) / 65536.0 - 46.85; //calculation taken from data sheet


    temp_C_float = UINT32_TO_FLOAT(temperature_C * 1000, -3);

    UINT8_TO_BITSTREAM(p, 0); //Celcius
    UINT32_TO_BITSTREAM(p, temp_C_float);

    indication.characteristicHandle = gattdb_temperature_measurement;
    indication.bufferLen = TEMP_BUFF_LEN;

    if (!(ble_data->s_IndicationInFlight)) {
        ble_status = sl_bt_gatt_server_send_indication(
            ble_data->s_ConnectionHandle,
            indication.characteristicHandle,
            indication.bufferLen,
            indication.buff);
        if (ble_status != SL_STATUS_OK) {
            LOG_ERROR("sl_bt_gatt_server_send_indication: %x\r\n", ble_status);
        }
        ble_data->s_IndicationInFlight = true;
    }
    else {
        IndicationQ_Enqueue(indication);
    }
    displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temp=%d", temperature_C);
    LOG_INFO("%d\r\n", temperature_C);

    I2C0_Teardown();
}

/*
void TemperatureStateMachine(event_t event) {
    temp_fsm_state_t current_state;
    static temp_fsm_state_t next_state = PERIOD_WAIT;

    current_state = next_state;

    switch (current_state) {
        case PERIOD_WAIT:
            if (event == ev_LETIMER0_UF) {
                PowerUp();
                next_state = POWERING_UP;
            }
            break;

        case POWERING_UP:
            if (event == ev_LETIMER0_COMP1) {
                SendReadTempCommand();
                next_state = REQUEST_TEMP;
            }
            break;

        case REQUEST_TEMP:
            if (event == ev_I2C0_TRANSFER_DONE) {
                WaitForTempSensorReading();
                next_state = READING_TEMP;
            }
            break;

        case READING_TEMP:
            if (event == ev_LETIMER0_COMP1) {
                RequestTempSensorReading();
                next_state = RECEIVED_TEMP;
            }
            break;

        case RECEIVED_TEMP:
            if (event == ev_I2C0_TRANSFER_DONE) {
                ReadOutTempSensorReading();
                next_state = PERIOD_WAIT;
            }
            break;
    }
}
*/


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
                BleClient_RequestTemperatureServiceInfo(ble_data);
                next_state = RECEIVING_TEMP_SERVICE_INFO;
            }
            break;

        case RECEIVING_TEMP_SERVICE_INFO:
            if (SL_BT_MSG_ID(event->header) == sl_bt_evt_gatt_procedure_completed_id) {
                BleClient_RequestTemperatureCharacteristicInfo(ble_data);
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

        case DISCOVERED:
            break;

    }
}


void BleClient_RequestTemperatureServiceInfo(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_status = sl_bt_gatt_discover_primary_services_by_uuid(ble_data->c_ConnectionHandle,
                                                                  sizeof(thermo_service),
                                                                  thermo_service);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_gatt_discover_primary_services_by_uuid: %x\r\n", ble_status);
    }
}


void BleClient_RequestTemperatureCharacteristicInfo(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_status = sl_bt_gatt_discover_characteristics_by_uuid(ble_data->c_ConnectionHandle,
                                                             ble_data->c_TemperatureServiceHandle,
                                                             sizeof(thermo_char),
                                                             thermo_char);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_gatt_discover_characteristics_by_uuid: %x\r\n", ble_status);
    }
}


void BleClient_RequestButtonServiceInfo(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_status = sl_bt_gatt_discover_primary_services_by_uuid(ble_data->c_ConnectionHandle,
                                                                  sizeof(button_service),
                                                                  button_service);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_gatt_discover_primary_services_by_uuid: %x\r\n", ble_status);
    }
}


void BleClient_RequestButtonCharacteristicInfo(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_status = sl_bt_gatt_discover_characteristics_by_uuid(ble_data->c_ConnectionHandle,
                                                             ble_data->c_ButtonServiceHandle,
                                                             sizeof(button_char),
                                                             button_char);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_gatt_discover_characteristics_by_uuid: %x\r\n", ble_status);
    }
}


void BleClient_EnableTemperatureIndications(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_status = sl_bt_gatt_set_characteristic_notification(ble_data->c_ConnectionHandle,
                                                            ble_data->c_TemperatureCharacteristicHandle, 0x02);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_gatt_set_characteristic_notification: %x\r\n", ble_status);
    }
}


void BleClient_EnableButtonIndications(ble_data_struct_t* ble_data) {
    sl_status_t ble_status;

    ble_data->c_TemperatureIndicating = true;
    ble_status = sl_bt_gatt_set_characteristic_notification(ble_data->c_ConnectionHandle,
                                                            ble_data->c_ButtonCharacteristicHandle, 0x02);
    if (ble_status != SL_STATUS_OK) {
        LOG_ERROR("sl_bt_gatt_set_characteristic_notification: %x\r\n", ble_status);
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
        LOG_ERROR("sl_bt_scanner_start: %x\r\n", ble_status);
    }
}
