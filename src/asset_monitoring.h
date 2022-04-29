/*
 *  asset_monitoring.h
 *
 *  Created on: Apr 18, 2022
 *      Author: rishab */

#ifndef SRC_ASSET_MONITORING_H_
#define SRC_ASSET_MONITORING_H_

#include <stdlib.h>

/*
 * Asset Monitoring FSM states enum
 */
typedef enum {
  BME280_INIT_CONFIG,
  BNO055_INIT_CONFIG,
  BME280_READ,
  BNO055_READ,
  DEBUG_READ,
} asset_monitoring_state_t;

#endif /* SRC_ASSET_MONITORING_H_ */
