/*
 * ble.h - BLE functions
 *
 *  Created on: Sep 27, 2021
 *      Author: vishn
 *  Attribution: The sl_bt_evt_system_boot_id code was sourced
 *      from the soc_thermometer and soc_thermometer_client example project.
 */

#ifndef SRC_BLE_H_
#define SRC_BLE_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "lcd.h"
#include "ble_device_type.h"





#define INDICATION_QUEUE_SIZE  (16)
#define INDICATION_QUEUE_SIZE_MASK  (15)

#define INDICATION_QUEUE_TIMER_HANDLE  (4)
#define INDICATION_QUEUE_TIMER_INTERVAL  (6554)


#define BUTTON_BUFF_LEN   (2)
#define TEMP_BUFF_LEN     (5)

#define BUTTON_STATE_PRESSED  (0x01)
#define BUTTON_STATE_RELEASED (0x00)


#define IND_SEQ_PB0_PRESSED  (0x01)
#define IND_SEQ_PB1_PRESSED  (0x02)
#define IND_SEQ_PB1_RELEASED (0x04)
#define IND_SEQ_PB0_RELEASED (0x08)


#define UINT8_TO_BITSTREAM(p, n) { *(p)++ = (uint8_t)(n); }

#define UINT32_TO_BITSTREAM(p, n) { *(p)++ = (uint8_t)(n); *(p)++ = (uint8_t)((n) >> 8); \
 *(p)++ = (uint8_t)((n) >> 16); *(p)++ = (uint8_t)((n) >> 24); }

#define UINT32_TO_FLOAT(m, e) (((uint32_t)(m) & 0x00FFFFFFU) | (uint32_t)((int32_t)(e) << 24))

// Health Thermometer service UUID defined by Bluetooth SIG
static const uint8_t thermo_service[2] = { 0x09, 0x18 };
// Temperature Measurement characteristic UUID defined by Bluetooth SIG
static const uint8_t thermo_char[2] = { 0x1c, 0x2a };

// Button service UUID defined by Bluetooth SIG
static const uint8_t button_service[16] = {
    0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87,
    0x3e, 0x43, 0xc8, 0x38, 0x01, 0x00, 0x00, 0x00
};
// Button characteristic UUID defined by Bluetooth SIG
static const uint8_t button_char[16] = {
    0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87,
    0x3e, 0x43, 0xc8, 0x38, 0x02, 0x00, 0x00, 0x00
};

typedef struct indiaction_struct_s {
    uint16_t characteristicHandle;
    uint8_t buff[5];
    uint8_t bufferLen;
} indication_struct_t;


// BLE Data Structure, save all of our private BT data in here.
// Modern C (circa 2021 does it this way)
// typedef ble_data_struct_t is referred to as an anonymous struct definition
typedef struct ble_data_struct_s {
    // values that are common to servers and clients, no prefixes
    bd_addr serverAddress;
    uint8_t serverAddressType;
    // Values unique for server, prefixed with "s_"
    uint8_t s_AdvertisingHandle;
    uint8_t s_ConnectionHandle;
    uint16_t s_TemperatureCharacteristicHandle;
    uint16_t s_ButtonCharacteristicHandle;
    bool s_ClientConnected;
    bool s_TemperatureIndicating;
    bool s_ButtonIndicating;
    bool s_ReadingTemp;
    bool s_IndicationInFlight;
    bool s_BondingPending;
    bool s_Bonded;
    // Values unique for client, prefixed with "c_"
    bd_addr c_DeviceAddress;
    uint8_t c_DeviceAddressType;
    uint8_t c_ConnectionHandle;
    uint32_t c_TemperatureServiceHandle;
    uint16_t c_TemperatureCharacteristicHandle;
    uint8_t c_TemperatureCharacteristicProperties;
    uint32_t c_ButtonServiceHandle;
    uint16_t c_ButtonCharacteristicHandle;
    uint8_t c_ButtonCharacteristicProperties;
    bool c_Connected;
    bool c_TemperatureIndicating;
    bool c_ButtonIndicating;
    bool c_BondingPending;
    bool c_Bonded;
    uint8_t c_ButtonIndicationStatus;
} ble_data_struct_t;


/*
 * Event enum
 */
typedef enum {
    ev_NONE = 0,
    ev_LETIMER0_COMP1 =1,
    ev_LETIMER0_UF = 2,
    ev_I2C0_TRANSFER_DONE = 3,
    ev_PB0_PRESSED = 4,
    ev_PB0_RELEASED = 5,
    ev_PB1_PRESSED = 6,
    ev_PB1_RELEASED = 7,
    ev_SHUTDOWN = 8
} ble_ext_signal_event_t;


/************************************************/
/****************Queue Functions*****************/
/************************************************/


/*
 * Enqueues indication
 *
 * @param indication - Indication to enqueue
 *
 * @return None
 */
void IndicationQ_Enqueue(indication_struct_t indication);


/*
 * Dequeues indication
 *
 * @param None
 *
 * @return indication at head of queue
 */
indication_struct_t IndicationQ_Dequeue(void);


/*
 * Checks if an indication is in the queue
 *
 * @param None
 *
 * @return 1 if queue has indication, 0 if not
 */
uint8_t IndicationQ_IsIndicationPending(void);


/*
 * Reset indication queue
 *
 * @param None
 *
 * @return None
 */
void IndicationQ_Reset(void);


/************************************************/
/**************Common BLE Functions**************/
/************************************************/


/*
 * Initialize BLE
 *
 * @param None
 *
 * @return None
 *
 * Attribution: Professor Sluiter during office hours on
 * 10/12/2021
 */
void BLE_Init(void);


/*
 * Handles Bluetooth stack event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void handle_ble_event(sl_bt_msg_t* event);


/*
 * Gets ble_data to modify
 *
 * @param None
 *
 * @return ble_data_struct_t pointer to modify
 */
ble_data_struct_t* BLE_GetDataStruct(void);


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

#endif /* SRC_BLE_H_ */
