/*
 * MMA8452Q.c
 *
 *  Created on: Nov 16, 2021
 *      Author: vishn
 */


#include "MMA8452Q.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"


uint8_t mma8452q_wr_buff[8] = { 0 };
uint8_t mma8452q_rd_buff[8] = { 0 };


void MMA8452Q_VerifyIdentity(uint8_t* rd_buff);
void MMA8452Q_ReadCtrlReg1(uint8_t* rd_buff);
void MMA8452Q_ActivateAccel(uint8_t* wr_buff, uint8_t ctrl_reg1);
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
                next_state = READ_CTRL_REG1;
            }
            break;

        case READ_CTRL_REG1:
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
                next_state = ACTIVATE_ACCEL;
            }
            break;

        case ACTIVATE_ACCEL:
            if (ev == ev_LETIMER0_COMP1) {
                MMA8452Q_ActivateAccel(&mma8452q_wr_buff[0], mma8452q_rd_buff[0]);
                next_state = DELAY_3;
            }
            break;

        case DELAY_3:
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
                next_state = DELAY_4;
            }
            break;

        case DELAY_4:
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


void MMA8452Q_ActivateAccel(uint8_t* wr_buff, uint8_t ctrl_reg1) {
    wr_buff[0] = CTRL_REG1;
    wr_buff[1] = ctrl_reg1 | 0x01;
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


//void MMA8452Q_ReadY(uint8_t* rd_buff) {
//    uint8_t reg = OUT_Y_MSB;
//    I2C0_WriteRead(MMA8452Q_ADDR, &reg, 1, &rd_buff[2], 2);
//}
//
//
//void MMA8452Q_ReadZ(uint8_t* rd_buff) {
//    uint8_t reg = OUT_Z_MSB;
//    I2C0_WriteRead(MMA8452Q_ADDR, &reg, 1, &rd_buff[4], 2);
//}
