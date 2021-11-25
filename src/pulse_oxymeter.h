/*
 * pulse_oxymeter.h
 *
 *  Created on: Nov 23, 2021
 *      Author: risha
 */

#ifndef SRC_PULSE_OXYMETER_H_
#define SRC_PULSE_OXYMETER_H_

#include "activity_monitoring.h"
#include "i2c.h"
#include "timers.h"
#include "em_letimer.h"
#include "gpio.h"
#include "ble.h"

typedef enum uint32_t_pulse_oxymeter
{
  DEFAULT_STATE = 0,
  STATE_INIT = 1,
  STATE_CONFIGURATION = 2,
  STATE_RUNNING = 3,
  NUM_PULSE_CONF_STATES
}oxymeter_states;


typedef enum uint32_t_heartbeat_init_states
{
  HB_INIT_STATE_0 = 1,
  HB_INIT_STATE_1 = 2,
  HB_INIT_STATE_2 = 3,
  HB_INIT_STATE_3 = 4,
  HB_INIT_STATE_4 = 5,
  NUM_HB_INIT_STATES
}heartbeat_init_states;

typedef enum uint32_t_heartbeat_config_states
{
  HB_CONFIG_STATE_0 = 1,
  HB_CONFIG_STATE_1 = 2,
  HB_CONFIG_STATE_2 = 3,
  HB_CONFIG_STATE_3 = 4,
  HB_CONFIG_STATE_4 = 5,
  HB_CONFIG_STATE_5 = 6,
  HB_CONFIG_STATE_6 = 7,
  HB_CONFIG_STATE_7 = 8,
  HB_CONFIG_STATE_8 = 9,
  HB_CONFIG_STATE_9 = 10,
  HB_CONFIG_STATE_10 = 11,
  HB_CONFIG_STATE_11 = 12,
  HB_CONFIG_STATE_12 = 13,
  HB_CONFIG_STATE_13 = 14,

  NUM_HB_CONFIG_STATES
}heartbeat_config_states;

typedef enum uint32_t_heartbeat_run_states
{
  HB_RUN_STATE_0 = 1,
  HB_RUN_STATE_1 = 2,
  HB_RUN_STATE_2 = 3,
  HB_RUN_STATE_3 = 4,
  HB_RUN_STATE_4 = 5,
  HB_RUN_STATE_5 = 6,
  HB_RUN_STATE_6 = 7,
  HB_RUN_STATE_7 = 8,
  HB_RUN_STATE_8 = 9,
  HB_RUN_STATE_9 = 10,
  HB_RUN_STATE_10 = 11,
  HB_RUN_STATE_11 = 12,
  HB_RUN_STATE_12 = 13,
  HB_RUN_STATE_13 = 14,

  NUM_HB_RUN_STATES
}heartbeat_running_states;

//Handle MAX30101
//void pulse_oxymeter_machine(sl_bt_msg_t *evt);
activity_monitoring_state_t init_heartbeat_machine(sl_bt_msg_t *evt);
activity_monitoring_state_t config_heartbeat_machine(sl_bt_msg_t *evt);
activity_monitoring_state_t heartbeat_machine_running(sl_bt_msg_t *evt);

#endif /* SRC_PULSE_OXYMETER_H_ */
