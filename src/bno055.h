/*
 * bno055.h
 *
 *  Created on: Feb 17, 2022
 *      Author: risha
 *      Reference: Adafruit BNO055 library
 */

#ifndef SRC_BNO055_H_
#define SRC_BNO055_H_

#include <stdbool.h>

#include "i2c.h"
#include "timers.h"
#include "em_letimer.h"
#include "gpio.h"
#include "ble.h"
#include "scheduler.h"
#include "app.h"
#include "asset_monitoring.h"

#define BNO055_ADDRESS_A        (0x28)
#define BNO055_ID               (0xA0)

#define BNO055_CHIP_ID_ADDR     (0x00)
#define OPERATION_MODE_CONFIG   (0X00)
#define BNO055_OPR_MODE_ADDR    (0X3D)
#define BNO055_SYS_TRIGGER_ADDR (0X3F)
#define BNO055_PWR_MODE_ADDR    (0X3E)
#define POWER_MODE_NORMAL       (0X00)
#define POWER_MODE_LOW_POWER    (0x01)
#define BNO055_PAGE_ID_ADDR     (0X07)
#define OPERATION_MODE_NDOF     (0X0C)

#define BNO055_EULER_H_LSB_ADDR (0X1A)

//Timer
#define SETMODE_DELAY               ((30)*(MSEC_TO_USEC))
#define POWER_MODE_DELAY            ((50)*(MSEC_TO_USEC))
#define BNO055_STD_DELAY            ((10)*(MSEC_TO_USEC))
#define POST_RESET_STARTUP_DELAY    ((500)*(MSEC_TO_USEC))
#define POST_OPERATION_MODE_DELAY   ((1000)*(MSEC_TO_USEC))
#define BNO055_DATA_POLL            ((1000)*(MSEC_TO_USEC))
#define PSEUDO_TRIGGER              ((1)*(MSEC_TO_USEC))

typedef enum
{
  BNO055_ADD_VERIFN,

  BNO055_SETMODE,
  BNO055_SETMODE_DELAY_1,

  BNO055_RESET,
  BNO055_RESET_DELAY_2,

  BNO055_READ_POST_RESET,
  BNO055_READ_POST_RESET_DELAY_3,

  BNO055_POWER_MODE_SET,
  BNO055_POWER_MODE_SET_DELAY_4,

  BNO055_PAGE_ADDR,
  BNO055_PAGE_ADDR_DELAY_5,

  BNO055_SYS_TRIGGER,
  BNO055_SYS_TRIGGER_DELAY_6,

  BNO055_SET_REQ_MODE,
  BNO055_SET_REQ_MODE_DELAY_7,

  READ_XYZ_DATA,
  READ_XYZ_DATA_DELAY,

  BNO055_DEFAULT,
}BNO055_state_t;

#if NO_BL
//BNO055_state_t init_bno055_machine(sl_bt_msg_t *evt);
#else
//BNO055_state_t init_bno055_machine(ble_ext_signal_event_t evt);
#endif

asset_monitoring_state_t bno055_read_machine(sl_bt_msg_t *evt);
asset_monitoring_state_t init_bno055_machine(sl_bt_msg_t *evt);
#endif /* SRC_BNO055_H_ */
