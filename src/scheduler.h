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
    ev_I2C0_TRANSFER_DONE
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
 * Reads temperature sensor reading and completes teardown of I2C0
 *
 * @param None
 *
 * @return None
 */
void ReadTempSensorReading(void);

#endif /* SRC_SCHEDULER_H_ */
