/*
 * scheduler.h - Scheduler functions
 *
 *  Created on: Sep 12, 2021
 *      Author: vishn
 */

#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_

#include <stdlib.h>
#include "em_core.h"
#include "ble.h"
#include "i2c.h"
#include "lcd.h"
#include "timers.h"
#include "app.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"


/*
 * Server Temperature FSM states enum
 */
typedef enum {
    PERIOD_WAIT,
    POWERING_UP,
    REQUEST_TEMP,
    READING_TEMP,
    RECEIVED_TEMP
} temp_fsm_state_t;


/*
 * Client Discovery FSM states enum
 */
typedef enum {
    SCANNING,
    RECEIVING_SERVICE_INFO,
    RECEIVING_CHARACTERISTIC_INFO,
    ENABLING_INDICATIONS,
    DISCOVERED
} disc_fsm_state_t;




/************************************************/
/****************Event Handlers******************/
/************************************************/


/*
 * Sets LETIMER0_UF event
 *
 * @param None
 *
 * @return None
 */
void Scheduler_SetEvent_LETIMER0_UF(void);


/*
 * Sets LETIMER0_COMP1 event
 *
 * @param None
 *
 * @return None
 */
void Scheduler_SetEvent_LETIMER0_COMP1(void);


/*
 * Sets I2C0_TRANSFER_DONE event
 *
 * @param None
 *
 * @return None
 */
void Scheduler_SetEvent_I2C0_TRANSFER_DONE(void);


/*
 * Sets PB0_PRESSED event
 *
 * @param None
 *
 * @return None
 */
void Scheduler_SetEvent_PB0_PRESSED(void);


/*
 * Sets PB0_RELEASED event
 *
 * @param None
 *
 * @return None
 */
void Scheduler_SetEvent_PB0_RELEASED(void);


/************************************************/
/***************Server Functions*****************/
/************************************************/


/*
 * Temperature state machine driven by BLE Server events
 *
 * @param event - BLE event to pass into state machine
 *
 * @return None
 */
void BleServer_TemperatureStateMachine(sl_bt_msg_t* event);


/*
 * Powers up I2C0 module and sets up wait time for power-up
 *
 * @param None
 *
 * @return None
 */
void PowerUp(void);


/*
 * Sends read temperature command to temperature sensor
 *
 * @param None
 *
 * @return None
 */
void SendReadTempCommand(void);


/*
 * Sets wait time for temperature reading to complete
 *
 * @param None
 *
 * @return None
 */
void WaitForTempSensorReading(void);


/*
 * Kicks off read from temperature sensor
 *
 * @param None
 *
 * @return None
 */
void RequestTempSensorReading(void);


/*
 * Reads out temperature sensor reading and completes teardown of I2C0
 *
 * @param ble_data - BLE data struct with connection info
 *
 * @return None
 */
void ReadOutTempSensorReading(ble_data_struct_t* ble_data);


/*
 * State machine code
 *
 * @param event - Event to pass into state machine
 *
 * @return None
 */
//void TemperatureStateMachine(event_t event);


/************************************************/
/***************Client Functions*****************/
/************************************************/


/*
 * Discovery state machine driven by BLE Client events
 *
 * @param event - BLE event to pass into state machine
 *
 * @return None
 */
void BleClient_DiscoveryStateMachine(sl_bt_msg_t* event);


/*
 * Requests service info from the server
 *
 * @param ble_data - BLE data struct with connection info
 *
 * @return None
 */
void BleClient_RequestServiceInfo(ble_data_struct_t* ble_data);


/*
 * Requests characteristic info from the server
 *
 * @param ble_data - BLE data struct with connection info
 *
 * @return None
 */
void BleClient_RequestCharacteristicInfo(ble_data_struct_t* ble_data);


/*
 * Enables indications from the server
 *
 * @param ble_data - BLE data struct with connection info
 *
 * @return None
 */
void BleClient_EnableIndications(ble_data_struct_t* ble_data);


/*
 * Displays indications being handled from the server
 *
 * @param ble_data - BLE data struct with connection info
 *
 * @return None
 */
void BleClient_SetDisplayingOfIndications(ble_data_struct_t* ble_data);


/*
 * Restarts scanning upon closed connection
 *
 * @param None
 *
 * @return None
 */
void BleClient_RestartScanning(void);

#endif /* SRC_SCHEDULER_H_ */
