/*
 * ble.h - BLE functions
 *
 *  Created on: Sep 27, 2021
 *      Author: vishn
 *  Attribution: The sl_bt_evt_system_boot_id code was sourced
 *      from the soc_thermometer example project.
 */

#ifndef SRC_BLE_H_
#define SRC_BLE_H_

#include <stdint.h>
#include <stdbool.h>
#include "sl_bluetooth.h"
#include "gatt_db.h"

#define UINT8_TO_BITSTREAM(p, n) { *(p)++ = (uint8_t)(n); }

#define UINT32_TO_BITSTREAM(p, n) { *(p)++ = (uint8_t)(n); *(p)++ = (uint8_t)((n) >> 8); \
 *(p)++ = (uint8_t)((n) >> 16); *(p)++ = (uint8_t)((n) >> 24); }

#define UINT32_TO_FLOAT(m, e) (((uint32_t)(m) & 0x00FFFFFFU) | (uint32_t)((int32_t)(e) << 24))

// BLE Data Structure, save all of our private BT data in here.
// Modern C (circa 2021 does it this way)
// typedef ble_data_struct_t is referred to as an anonymous struct definition
typedef struct {
    // values that are common to servers and clients
    bd_addr address;
    uint8_t myAddressType;
    uint8_t systemId[8];
    // values unique for server
    // The advertising set handle allocated from Bluetooth stack.
    uint8_t advertisingSetHandle;
    uint8_t connectionHandle;
    uint16_t tempCharacteristicHandle;
    bool connected;
    bool indicating;
    bool notFirstConnection;
    bool readingTemp;
    bool indicationInFlight;
    // values unique for client
} ble_data_struct_t;


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


/*
 * Handles BLE boot event
 *
 * @param None
 *
 * @return None
 *
 * Attribution: The sl_bt_evt_system_boot_id code was sourced
 *    from the soc_thermometer example project.
 */
void BLE_HandleBootEvent(void);


/*
 * Handles BLE connection opened event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BLE_HandleConnectionOpenedEvent(sl_bt_msg_t* event);


/*
 * Handles BLE connection closed event
 *
 * @param None
 *
 * @return None
 */
void BLE_HandleConnectionClosedEvent(void);


/*
 * Handles BLE connection parameters event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BLE_HandleConnectionParametersEvent(sl_bt_msg_t* event);


/*
 * Handles BLE external signal event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BLE_HandleExternalSignalEvent(void);


/*
 * Handles BLE characteristic status event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BLE_HandleCharacteristicStatusEvent(sl_bt_msg_t* event);


/*
 * Handles BLE indication timeout event
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return None
 */
void BLE_HandleIndicationTimeoutEvent(void);


#endif /* SRC_BLE_H_ */
