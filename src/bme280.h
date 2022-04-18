/*
 * bme280.h
 *
 *  Created on: Feb 24, 2022
 *      Author: Mukta
 */

#ifndef SRC_BME280_H_
#define SRC_BME280_H_

#include <stdbool.h>

#include "i2c.h"
#include "timers.h"
#include "em_letimer.h"
#include "gpio.h"
#include "ble.h"
#include "scheduler.h"
#include "asset_monitoring.h"

#define BME280_ADDRESS            (0x76)
#define BME280_ID                 (0x60)
#define BME280_CHIP_ID_ADDR       (0xD0)
#define BME280_REGISTER_SOFTRESET (0xE0)
#define BME280_REGISTER_STATUS    (0XF3)
#define BME280_TEMP_PRESS_CALIB_DATA_ADDR         (0x88)
#define BME280_HUMIDITY_CALIB1_DATA_ADDR          (0xA1)
#define BME280_HUMIDITY_CALIB_DATA_ADDR           (0xE1)
#define BME280_CTRL_HUM_ADDR                      (0xF2)
#define BME280_CTRL_MEAS_ADDR                     (0xF4)
#define BME280_CONFIG_ADDR                        (0xF5)
#define BME280_DATA_ADDR                          (0xFA)

//name Macros related to size
#define BME280_TEMP_PRESS_CALIB_DATA_LEN          (6)
#define BME280_HUMIDITY_CALIB_DATA_LEN            (7)
#define BME280_T_RH_DATA_LEN                      (5)

//settings for BME280
#define BME280_SLEEP_MODE_SEL                     (0x00)
#define BME280_HUM_SAMP_ON                        (0x05)
#define BME280_TEMP_ON_PRS_OFF                    (0xA3)
#define BME280_CON_1SEC_FILT_OFF                  (0x00)
#define BME280_FORCED_MODE_SEL                    (0x21)

//Timer
#define SETMODE_DELAY               ((30)*(MSEC_TO_USEC))
#define NORMAL_MODE_DELAY           ((50)*(MSEC_TO_USEC))
#define STD_DELAY                   ((10)*(MSEC_TO_USEC))
#define POST_RESET_STARTUP_DELAY    ((500)*(MSEC_TO_USEC))

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb)             (((uint16_t)msb << 8) | (uint16_t)lsb)


typedef enum
{
  BME280_ADD_VERIFN,

  BME280_REG_SOFTRESET,
  BME280_REG_SOFTRESET_DELAY_1,

  BME280_READ_STATUS,
  BME280_READ_STATUS_RESPONSE,

  BME280_READ_CALIB_1,
  BME280_READ_CALIB_1_RESPONSE,
  BME280_READ_CALIB_2,
  BME280_READ_CALIB_2_RESPONSE,
  BME280_READ_CALIB_3,
  BME280_READ_CALIB_3_RESPONSE,

  BME280_SLEEP_MODE_SET,
  BME280_CONFIG_SET,
  BME280_HUM_CTRL_SET,
  BME280_MEAS_CTRL_SET,
  BME280_AFTER_SET_DELAY_2,
  BME280_AFTER_INIT_DONE,

  BME280_READ_TRH_DATA,
  BME280_DISP_TRH_DATA,

  BME280_DEFAULT,
}BME280_state_t;

asset_monitoring_state_t init_bme280_machine(sl_bt_msg_t *evt);
asset_monitoring_state_t bme280_read_machine(sl_bt_msg_t *evt);

#endif /* SRC_BME280_H_ */
