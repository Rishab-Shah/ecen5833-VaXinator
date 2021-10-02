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
#include "i2c.h"
#include "timers.h"
#include "app.h"
#include "sl_bluetooth.h"
#include "ble.h"
#include "gatt_db.h"

#define EVENT_QUEUE_SIZE  (128)
#define EVENT_QUEUE_SIZE_MASK  (127)


/*
 * Temperature FSM states enum
 */
typedef enum {
    PERIOD_WAIT,
    POWERING_UP,
    REQUEST_TEMP,
    READING_TEMP,
    RECEIVED_TEMP
} temp_fsm_state_t;


/*
 * Event enum
 */
typedef enum {
    ev_NONE,
    ev_LETIMER0_COMP1,
    ev_LETIMER0_UF,
    ev_I2C0_TRANSFER_DONE,
    ev_SHUTDOWN
} event_t;


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
 * Gets next event from the scheduler
 *
 * @param None
 *
 * @return current event
 */
event_t Scheduler_GetNextEvent(void);


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


/*
 * State machine for BLE events
 *
 * @param event - BLE event to pass into state machine
 *
 * @return None
 */
void state_machine(sl_bt_msg_t* event);

#endif /* SRC_SCHEDULER_H_ */
