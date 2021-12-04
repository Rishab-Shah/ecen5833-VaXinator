/*
 * MMA8452Q.h - MMA8452Q Accelerometer functions
 *
 *  Created on: Nov 16, 2021
 *      Author: vishn
 *
 * Reference: https://github.com/sparkfun/MMA8452_Accelerometer
 */

#ifndef SRC_MMA8452Q_H_
#define SRC_MMA8452Q_H_


#include <stdlib.h>
#include "activity_monitoring.h"
#include "i2c.h"
#include "ble.h"
#include "timers.h"


#define MMA8452Q_ADDR (0x1D)

#define STATUS_MMA8452Q  (0x00)
#define OUT_X_MSB  (0x01)
#define OUT_X_LSB  (0x02)
#define OUT_Y_MSB  (0x03)
#define OUT_Y_LSB  (0x04)
#define OUT_Z_MSB  (0x05)
#define OUT_Z_LSB  (0x06)
#define SYSMOD  (0x0B)
#define INT_SOURCE  (0x0C)
#define WHO_AM_I  (0x0D)
#define XYZ_DATA_CFG  (0x0E)
#define HP_FILTER_CUTOFF  (0x0F)
#define PL_STATUS  (0x10)
#define PL_CFG  (0x11)
#define PL_COUNT  (0x12)
#define PL_BF_ZCOMP  (0x13)
#define P_L_THS_REG  (0x14)
#define FF_MT_CFG  (0x15)
#define FF_MT_SRC  (0x16)
#define FF_MT_THS  (0x17)
#define FF_MT_COUNT  (0x18)
#define TRANSIENT_CFG  (0x1D)
#define TRANSIENT_SRC  (0x1E)
#define TRANSIENT_THS  (0x1F)
#define TRANSIENT_COUNT  (0x20)
#define PULSE_CFG  (0x21)
#define PULSE_SRC  (0x22)
#define PULSE_THSX  (0x23)
#define PULSE_THSY  (0x24)
#define PULSE_THSZ  (0x25)
#define PULSE_TMLT  (0x26)
#define PULSE_LTCY  (0x27)
#define PULSE_WIND  (0x28)
#define ASLP_COUNT  (0x29)
#define CTRL_REG1  (0x2A)
#define CTRL_REG2  (0x2B)
#define CTRL_REG3  (0x2C)
#define CTRL_REG4  (0x2D)
#define CTRL_REG5  (0x2E)
#define OFF_X  (0x2F)
#define OFF_Y  (0x30)
#define OFF_Z  (0x31)


typedef enum {
    VERIFY_IDENTITY,
    DELAY_1,
    READ_CTRL_REG1_1,
    DELAY_2,
    SET_STANDBY,
    DELAY_3,
    READ_CTRL_REG1_2,
    DELAY_4,
    SET_SAMPLING_RATE,
    DELAY_5,
    READ_CTRL_REG2_1,
    DELAY_6,
    SET_LOW_POWER_MODE,
    DELAY_7,
    READ_CTRL_REG1_3,
    DELAY_8,
    SET_ACTIVE,
    DELAY_9
} mma8452q_init_state_t;


typedef enum {
    READ_XYZ,
    DELAY_10,
    SEND_XYZ
} mma8452q_read_state_t;


typedef enum {
    MMA8452Q_INIT,
    MMA8452Q_READ
} mma8452q_state_t;


/*
 * FSM for initializing the MMA8452Q Accelerometer
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return Next state of MMA8452Q state machine
 */
activity_monitoring_state_t MMA8452Q_InitStateMachine(sl_bt_msg_t* event);


/*
 * FSM for reading measurements from the MMA8452Q Accelerometer
 *
 * @param event - Pointer to Bluetooth event
 *
 * @return Next state of MMA8452Q state machine
 */
activity_monitoring_state_t MMA8452Q_ReadStateMachine(sl_bt_msg_t* event);


#endif /* SRC_MMA8452Q_H_ */
