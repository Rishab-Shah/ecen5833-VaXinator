/*
 * bme280.h
 *
 *  Created on: Feb 24, 2022
 *      Author: rishab
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

#define BME280_ADDRESS            (0x76)
#define BME280_ID                 (0x60)
#define BME280_CHIP_ID_ADDR       (0xD0)
#define BME280_REGISTER_SOFTRESET (0xE0)
#define BME280_REGISTER_STATUS    (0XF3)

#if 0
#define OPERATION_MODE_CONFIG   (0X00)
#define BME280_OPR_MODE_ADDR    (0X3D)
#define BME280_SYS_TRIGGER_ADDR (0X3F)
#define BME280_PWR_MODE_ADDR    (0X3E)
#define POWER_MODE_NORMAL       (0X00)
#define BME280_PAGE_ID_ADDR     (0X07)
#define OPERATION_MODE_NDOF     (0X0C)

#define BME280_EULER_H_LSB_ADDR (0X1A)
#endif
//Timer
#define SETMODE_DELAY               ((30)*(MSEC_TO_USEC))
#define NORMAL_MODE_DELAY           ((50)*(MSEC_TO_USEC))
#define STD_DELAY                   ((10)*(MSEC_TO_USEC))
#define POST_RESET_STARTUP_DELAY    ((500)*(MSEC_TO_USEC))

typedef enum
{
  BME280_ADD_VERIFN,

  BME280_REG_SOFTRESET,
  BME280_REG_SOFTRESET_DELAY_1,

  BME280_READ_CALIB,
  BME280_READ_CALIB_DELAY_2,

  BME280_RESET,
  BME280_RESET_DELAY_2,

  BME280_READ_POST_RESET,
  BME280_READ_POST_RESET_DELAY_3,

  BME280_NORMAL_MODE_SET,
  BME280_NORMAL_MODE_SET_DELAY_4,

  BME280_PAGE_ADDR,
  BME280_PAGE_ADDR_DELAY_5,

  BME280_SYS_TRIGGER,
  BME280_SYS_TRIGGER_DELAY_6,

  BME280_SET_REQ_MODE,
  BME280_SET_REQ_MODE_DELAY_7,

  //READ_XYZ_DATA,
  //READ_XYZ_DATA_DELAY,

  BME280_DEFAULT,
}BME280_state_t;


BME280_state_t init_BME280_machine(sl_bt_msg_t *evt);


#endif /* SRC_BME280_H_ */
