/*
 * MMA8452Q.c
 *
 *  Created on: Nov 16, 2021
 *      Author: vishn
 */


#include "MMA8452Q.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

#define CTRL_REG1_STANDBY     (0x00)
#define CTRL_REG1_ACTIVE      (0x01)
#define CTRL_REG1_ACTIVE_MASK (0x01)
#define CTRL_REG1_OBR_VAL     (0x07) // 1.56Hz Sampling Rate
#define CTRL_REG1_OBR_MASK    (0x38)
#define CTRL_REG1_OBR_SHIFT   (3)

#define CTRL_REG2_MODS0_VAL   (0x03) // Low Power Mode
#define CTRL_REG2_MODS0_MASK  (0x03)


uint8_t mma8452q_wr_buff[8] = { 0 };
uint8_t mma8452q_rd_buff[8] = { 0 };


void MMA8452Q_VerifyIdentity(uint8_t* rd_buff);
void MMA8452Q_ReadCtrlReg1(uint8_t* rd_buff);
void MMA8452Q_ReadCtrlReg2(uint8_t* rd_buff);
void MMA8452Q_SetStandby(uint8_t* wr_buff, uint8_t ctrl_reg1);
void MMA8452Q_SetSamplingRate(uint8_t* wr_buff, uint8_t ctrl_reg1);
void MMA8452Q_SetLowPowerMode(uint8_t* wr_buff, uint8_t ctrl_reg2);
void MMA8452Q_SetActive(uint8_t* wr_buff, uint8_t ctrl_reg1);
void MMA8452Q_CheckDataAvailable(uint8_t* rd_buff);
void MMA8452Q_ReadXYZ(uint8_t* rd_buff);


void MMA8452Q_StateMachine(sl_bt_msg_t* event) {

    mma8452q_init_state_t current_state;
    static mma8452q_init_state_t next_state = VERIFY_IDENTITY;
    ble_ext_signal_event_t ev;

    if (SL_BT_MSG_ID(event->header) == sl_bt_evt_system_external_signal_id) {
        ev = event->data.evt_system_external_signal.extsignals;
    }
    else {
        return;
    }

    current_state = next_state;


    switch (current_state) {
        case VERIFY_IDENTITY:
            if (ev == ev_LETIMER0_UF) {
                MMA8452Q_VerifyIdentity(&mma8452q_rd_buff[0]);
                next_state = DELAY_1;
            }
            break;

        case DELAY_1:
            if (ev == ev_I2C0_TRANSFER_DONE) {
                timerWaitUs_irq(10000);
                next_state = READ_CTRL_REG1_1;
            }
            break;

        case READ_CTRL_REG1_1:
            if (ev == ev_LETIMER0_COMP1) {
                if (mma8452q_rd_buff[0] == 0x2A) {
                    MMA8452Q_ReadCtrlReg1(&mma8452q_rd_buff[0]);
                    next_state = DELAY_2;
                }
                else {
                    LOG_ERROR("Wrong device!\r\n");
                    next_state = VERIFY_IDENTITY;
                }
            }
            break;

        case DELAY_2:
            if (ev == ev_I2C0_TRANSFER_DONE) {
                timerWaitUs_irq(10000);
                next_state = SET_STANDBY;
            }
            break;

        case SET_STANDBY:
            if (ev == ev_LETIMER0_COMP1) {
                MMA8452Q_SetStandby(&mma8452q_wr_buff[0], mma8452q_rd_buff[0]);
                next_state = DELAY_3;
            }
            break;

        case DELAY_3:
            if (ev == ev_I2C0_TRANSFER_DONE) {
                timerWaitUs_irq(10000);
                next_state = READ_CTRL_REG1_2;
            }
            break;

        case READ_CTRL_REG1_2:
            if (ev == ev_LETIMER0_COMP1) {
                MMA8452Q_ReadCtrlReg1(&mma8452q_rd_buff[0]);
                next_state = DELAY_4;
            }
            break;

        case DELAY_4:
            if (ev == ev_I2C0_TRANSFER_DONE) {
                timerWaitUs_irq(10000);
                next_state = SET_SAMPLING_RATE;
            }
            break;

        case SET_SAMPLING_RATE:
            if (ev == ev_LETIMER0_COMP1) {
                MMA8452Q_SetSamplingRate(&mma8452q_wr_buff[0], mma8452q_rd_buff[0]);
                next_state = DELAY_5;
            }
            break;

        case DELAY_5:
            if (ev == ev_I2C0_TRANSFER_DONE) {
                timerWaitUs_irq(10000);
                next_state = READ_CTRL_REG2_1;
            }
            break;

        case READ_CTRL_REG2_1:
            if (ev == ev_LETIMER0_COMP1) {
                MMA8452Q_ReadCtrlReg2(&mma8452q_rd_buff[0]);
                next_state = DELAY_6;
            }
            break;

        case DELAY_6:
            if (ev == ev_I2C0_TRANSFER_DONE) {
                timerWaitUs_irq(10000);
                next_state = SET_LOW_POWER_MODE;
            }
            break;

        case SET_LOW_POWER_MODE:
            if (ev == ev_LETIMER0_COMP1) {
                MMA8452Q_SetLowPowerMode(&mma8452q_wr_buff[0], mma8452q_rd_buff[0]);
                next_state = DELAY_7;
            }
            break;

        case DELAY_7:
            if (ev == ev_I2C0_TRANSFER_DONE) {
                timerWaitUs_irq(10000);
                next_state = READ_CTRL_REG1_3;
            }
            break;

        case READ_CTRL_REG1_3:
            if (ev == ev_LETIMER0_COMP1) {
                MMA8452Q_ReadCtrlReg1(&mma8452q_rd_buff[0]);
                next_state = DELAY_8;
            }
            break;

        case DELAY_8:
            if (ev == ev_I2C0_TRANSFER_DONE) {
                timerWaitUs_irq(10000);
                next_state = SET_ACTIVE;
            }
            break;

        case SET_ACTIVE:
            if (ev == ev_LETIMER0_COMP1) {
                MMA8452Q_SetActive(&mma8452q_wr_buff[0], mma8452q_rd_buff[0]);
                next_state = DELAY_9;
            }
            break;

        case DELAY_9:
            if (ev == ev_I2C0_TRANSFER_DONE) {
                timerWaitUs_irq(10000);
                next_state = READ_XYZ;
            }
            break;

        case ACCEL_INIT_COMPLETE:
            if (ev == ev_LETIMER0_COMP1) {
                next_state = READ_XYZ;
            }
            break;

        case READ_XYZ:
            if (ev == ev_LETIMER0_UF) {
                MMA8452Q_ReadXYZ(&mma8452q_rd_buff[0]);
                next_state = DELAY_10;
            }
            break;

        case DELAY_10:
            if (ev == ev_I2C0_TRANSFER_DONE) {
                timerWaitUs_irq(10000);
                next_state = SEND_XYZ;
            }
            break;

        case SEND_XYZ:
            if (ev == ev_LETIMER0_COMP1) {
                LOG_INFO("%x\r\n", mma8452q_rd_buff[0]);
                LOG_INFO("%x\r\n", mma8452q_rd_buff[1]);
                LOG_INFO("%x\r\n", mma8452q_rd_buff[2]);
                LOG_INFO("%x\r\n", mma8452q_rd_buff[3]);
                LOG_INFO("%x\r\n", mma8452q_rd_buff[4]);
                LOG_INFO("%x\r\n", mma8452q_rd_buff[5]);
                next_state = READ_XYZ;
            }
    }
}


void MMA8452Q_VerifyIdentity(uint8_t* rd_buff) {
    uint8_t reg = WHO_AM_I;
    I2C0_WriteRead(MMA8452Q_ADDR, &reg, 1, &rd_buff[0], 1);
}


void MMA8452Q_ReadCtrlReg1(uint8_t* rd_buff) {
    uint8_t reg = CTRL_REG1;
    I2C0_WriteRead(MMA8452Q_ADDR, &reg, 1, &rd_buff[0], 1);
}

void MMA8452Q_ReadCtrlReg2(uint8_t* rd_buff) {
    uint8_t reg = CTRL_REG2;
    I2C0_WriteRead(MMA8452Q_ADDR, &reg, 1, &rd_buff[0], 1);
}

void MMA8452Q_SetStandby(uint8_t* wr_buff, uint8_t ctrl_reg1)  {
    ctrl_reg1 &= ~CTRL_REG1_ACTIVE_MASK;
    ctrl_reg1 |= CTRL_REG1_STANDBY;

    wr_buff[0] = CTRL_REG1;
    wr_buff[1] = ctrl_reg1;
    I2C0_Write(MMA8452Q_ADDR, &wr_buff[0], 2);
}


void MMA8452Q_SetSamplingRate(uint8_t* wr_buff, uint8_t ctrl_reg1) {
    ctrl_reg1 &= ~CTRL_REG1_OBR_MASK;
    ctrl_reg1 |= (CTRL_REG1_OBR_VAL << CTRL_REG1_OBR_SHIFT);

    wr_buff[0] = CTRL_REG1;
    wr_buff[1] = ctrl_reg1;
    I2C0_Write(MMA8452Q_ADDR, &wr_buff[0], 2);
}


void MMA8452Q_SetLowPowerMode(uint8_t* wr_buff, uint8_t ctrl_reg2) {
    ctrl_reg2 &= ~CTRL_REG2_MODS0_MASK;
    ctrl_reg2 |= CTRL_REG2_MODS0_VAL;

    wr_buff[0] = CTRL_REG2;
    wr_buff[1] = ctrl_reg2 | 0x01;
    I2C0_Write(MMA8452Q_ADDR, &wr_buff[0], 2);
}


void MMA8452Q_SetActive(uint8_t* wr_buff, uint8_t ctrl_reg1) {
    ctrl_reg1 &= ~CTRL_REG1_ACTIVE_MASK;
    ctrl_reg1 |= CTRL_REG1_ACTIVE;

    wr_buff[0] = CTRL_REG1;
    wr_buff[1] = ctrl_reg1;
    I2C0_Write(MMA8452Q_ADDR, &wr_buff[0], 2);
}


void MMA8452Q_CheckDataAvailable(uint8_t* rd_buff) {
    uint8_t reg = STATUS_MMA8452Q;
    I2C0_WriteRead(MMA8452Q_ADDR, &reg, 1, &rd_buff[0], 1);
}


void MMA8452Q_ReadXYZ(uint8_t* rd_buff) {
    uint8_t reg = OUT_X_MSB;
    I2C0_WriteRead(MMA8452Q_ADDR, &reg, 1, &rd_buff[0], 6);
}
