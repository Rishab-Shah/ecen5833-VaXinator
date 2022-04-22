/*
 * ble.h - BLE functions
 *
 *  Created on: Sep 27, 2021
 *      Author: vishnu and rishab
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


#define INDICATION_QUEUE_SIZE             (16)
#define INDICATION_QUEUE_SIZE_MASK        (15)

#define INDICATION_QUEUE_TIMER_HANDLE     (4)
#define INDICATION_QUEUE_TIMER_INTERVAL   (6554)


#define BUTTON_BUFF_LEN                   (2)
#define TEMP_BUFF_LEN                     (5)

#define BUTTON_STATE_PRESSED              (0x01)
#define BUTTON_STATE_RELEASED             (0x00)


#define IND_SEQ_PB0_PRESSED               (0x01)
#define IND_SEQ_PB1_PRESSED               (0x02)
#define IND_SEQ_PB1_RELEASED              (0x04)
#define IND_SEQ_PB0_RELEASED              (0x08)

#define UINT8_TO_BITSTREAM(p,n)        { *(p)++ = (uint8_t)(n); }

#define UINT32_TO_BITSTREAM(p, n)      { *(p)++ = (uint8_t)(n);           \
                                        *(p)++ = (uint8_t)((n) >> 8);     \
                                        *(p)++ = (uint8_t)((n) >> 16);    \
                                        *(p)++ = (uint8_t)((n) >> 24); }

#define UINT32_TO_FLOAT(m, e)         (((uint32_t)(m) & 0x00FFFFFFU) | ((uint32_t)((int32_t)(e) << 24)))

#define HEALTH_SIZE                        (16)
#define ACCEL_SIZE                         (16)
#define TRH_SIZE                           (16)

typedef struct indication_struct_s {
    uint16_t characteristicHandle;
    uint8_t buff[6];
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
    bool s_ClientConnected;
    bool s_IndicationInFlight;
    bool s_Bonded;
    //Server - Services
    //bool s_HealthIndicating;
    //uint8_t s_HealthService[HEALTH_SIZE];
    //uint8_t s_HealthChar[HEALTH_SIZE];

    bool s_AccelIndication;
    uint8_t s_AccelService[ACCEL_SIZE];
    uint8_t s_AccelChar[ACCEL_SIZE];

    bool s_TRHIndication;
    uint8_t s_TRHService[ACCEL_SIZE];
    uint8_t s_TRHChar[ACCEL_SIZE];









    // Values unique for client, prefixed with "c_"
    bd_addr c_DeviceAddress;
    uint8_t c_DeviceAddressType;
    uint8_t c_ConnectionHandle;
    bool c_Connected;
    bool c_Bonded;

    //Client - Services
    uint32_t c_HealthServiceHandle;
    uint16_t c_HealthCharacteristicHandle;
    uint8_t* c_HealthCharValue;
    uint8_t c_HealthValue;
    uint32_t c_AccelServiceHandle;
    uint16_t c_AccelCharacteristicHandle;
    uint8_t c_AccelBuffer[6];
    int16_t c_AccelX;
    int16_t c_AccelY;
    int16_t c_AccelZ;

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
    ev_SHUTDOWN = 8,
    ev_SPI_TX = 9,
    ev_SPI_RX = 10,
    ev_BNO055_Int = 11,
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

void BleServer_SendTRHDataToClient(float temperature_data, float RH_data);


void BleServer_SendAccelDataToClient(uint8_t* accel_buff);


#endif /* SRC_BLE_H_ */
