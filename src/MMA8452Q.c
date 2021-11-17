/*
 * MMA8452Q.c
 *
 *  Created on: Nov 16, 2021
 *      Author: vishn
 */


#include "MMA8452Q.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"


//uint8_t wr_buff[8];
uint8_t rd_buff[8] = { 0 };


void MMA8452Q_VerityIdentity(uint8_t* rd_buff);


void MMA8452Q_InitStateMachine(sl_bt_msg_t* event) {
    mma8452q_init_state_t current_state;
    static mma8452q_init_state_t next_state = VERIFY_IDENTITY;

    ble_ext_signal_event_t ev = event->data.evt_system_external_signal.extsignals;

    current_state = next_state;

    switch (current_state) {
        case VERIFY_IDENTITY:
            if (ev == ev_LETIMER0_UF) {
                MMA8452Q_VerityIdentity(rd_buff);
                next_state = ACCEL_INIT_COMPLETE;
            }
            break;
        case ACCEL_INIT_COMPLETE:
            if (ev == ev_I2C0_TRANSFER_DONE) {
                if (rd_buff[0] == 0x2A) {
                    LOG_INFO("We are reading MMA8452Q!\r\n");
                }
            }
            break;
    }
}


void MMA8452Q_VerityIdentity(uint8_t* rd_buff) {
    uint8_t command = WHO_AM_I;
    I2C0_WriteRead(MMA8452Q_ADDR, &command, 1, rd_buff, 1);
}
