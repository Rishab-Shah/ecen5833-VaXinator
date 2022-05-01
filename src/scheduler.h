/*
 * scheduler.h - Scheduler functions
 *
 *  Created on: Sep 12, 2021
 *      Author: vishnu and rishab
 */

#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_

#include <stdlib.h>
#include "em_core.h"
#include "ble.h"
#include "i2c.h"
#include "lcd.h"
#include "timers.h"
#include "irq.h"
#include "app.h"
#include "asset_monitoring.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "asset_monitoring.h"

/*
 * Client Discovery FSM states enum
 */
typedef enum {
    SCANNING,
    RECEIVING_HEALTH_SERVICE_INFO,
    RECEIVING_HEALTH_CHARACTERISTIC_INFO,
    ENABLING_HEALTH_INDICATIONS,
    RECEIVING_ACCEL_SERVICE_INFO,
    RECEIVING_ACCEL_CHARACTERISTIC_INFO,
    ENABLING_ACCEL_INDICATIONS,
    DISCOVERED
} disc_fsm_state_t;


typedef enum
{
  DEBUG_STATE = 0,
  DEBUG_DEFAULT,
}debug_state_t;

#define ERROR     (-1)
#define NEXT_ITERATION_DELAY                   ((1500)*(MSEC_TO_USEC))
void AssetMonitoringSystem_StateMachine(sl_bt_msg_t* event);
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


/*
 * Sets PB0_PRESSED event
 *
 * @param None
 *
 * @return None
 */
void Scheduler_SetEvent_PB1_PRESSED(void);


/*
 * Sets PB0_RELEASED event
 *
 * @param None
 *
 * @return None
 */
void Scheduler_SetEvent_PB1_RELEASED(void);


/*
 * Sets SPI_TX event
 *
 * @param None
 *
 * @return None
 */
void Scheduler_SetEvent_SPI_TX(void);


/*
 * Sets SPI_RX event
 *
 * @param None
 *
 * @return None
 */
void Scheduler_SetEvent_SPI_RX(void);

/*
 * Sets BNO055_INT event
 *
 * @param None
 *
 * @return None
 */
void Scheduler_SetEvent_BNO55_Int();

/************************************************/
/***************Server Functions*****************/
/************************************************/



/*
 * Activity monitoring system state machine driven by BLE Client events
 *
 * @param event - BLE event to pass into state machine
 *
 * @return None
 */
void ActivityMonitoringSystem_StateMachine(sl_bt_msg_t* event);


/************************************************/
/***************Client Functions*****************/
/************************************************/

#if 0
/*
 * Discovery state machine driven by BLE Client events
 *
 * @param event - BLE event to pass into state machine
 *
 * @return None
 */
void BleClient_DiscoveryStateMachine(sl_bt_msg_t* event);
#endif

/*
 * Requests health service info from the server
 *
 * @param ble_data - BLE data struct with connection info
 *
 * @return None
 */
void BleClient_RequestHealthServiceInfo(ble_data_struct_t* ble_data);


/*
 * Requests health characteristic info from the server
 *
 * @param ble_data - BLE data struct with connection info
 *
 * @return None
 */
void BleClient_RequestHealthCharacteristicInfo(ble_data_struct_t* ble_data);


/*
 * Enables health indications from the server
 *
 * @param ble_data - BLE data struct with connection info
 *
 * @return None
 */
void BleClient_EnableHealthIndications(ble_data_struct_t* ble_data);


/*
 * Requests accel service info from the server
 *
 * @param ble_data - BLE data struct with connection info
 *
 * @return None
 */
void BleClient_RequestAccelServiceInfo(ble_data_struct_t* ble_data);


/*
 * Requests accel characteristic info from the server
 *
 * @param ble_data - BLE data struct with connection info
 *
 * @return None
 */
void BleClient_RequestAccelCharacteristicInfo(ble_data_struct_t* ble_data);


/*
 * Enables accel indications from the server
 *
 * @param ble_data - BLE data struct with connection info
 *
 * @return None
 */
void BleClient_EnableAccelIndications(ble_data_struct_t* ble_data);


/*
 * Restarts scanning upon closed connection
 *
 * @param None
 *
 * @return None
 */
void BleClient_RestartScanning(void);

#endif /* SRC_SCHEDULER_H_ */
