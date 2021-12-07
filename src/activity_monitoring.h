/*
 * activity_monitoring.h
 *
 *  Created on: Nov 24, 2021
 *      Author: vishnu
 */

#ifndef SRC_ACTIVITY_MONITORING_H_
#define SRC_ACTIVITY_MONITORING_H_

#include <stdlib.h>

/*
 * Activity Monitoring FSM states enum
 */
typedef enum {
    HEARTBEAT_INIT,
    HEARTBEAT_CONFIGURE,
    ACCEL_INIT,
    HEARTBEAT_READ,
    ACCEL_READ
} activity_monitoring_state_t;

#endif /* SRC_ACTIVITY_MONITORING_H_ */
