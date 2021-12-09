/*
 * i2c.c - I2C functions
 *
 *  Created on: Sep 13, 2021
 *      Author: vishnu and rishab
 */


#include "i2c.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

I2CSPM_Init_TypeDef i2c_init_struct = {
    .port = I2C0,
    .sclPort = gpioPortC,
    .sclPin = 10,
    .sdaPort = gpioPortC,
    .sdaPin = 11,
    .portLocationScl = 14,
    .portLocationSda = 16,
    .i2cRefFreq = 0,
    .i2cMaxFreq = I2C_FREQ_STANDARD_MAX,
    .i2cClhr = i2cClockHLRStandard
};


I2C_TransferSeq_TypeDef i2c_transfer_seq;


void I2C0_Init(void) {
    I2CSPM_Init(&i2c_init_struct);
}


I2C_TransferReturn_TypeDef I2C0_Write(uint8_t addr, uint8_t* wr_buff, uint8_t wr_buff_len) {
    I2C_TransferReturn_TypeDef i2c_transfer_ret;

    I2C0_Init();
    i2c_transfer_seq.addr = (addr << 1);
    i2c_transfer_seq.flags = I2C_FLAG_WRITE;
    i2c_transfer_seq.buf[0].data = wr_buff;
    i2c_transfer_seq.buf[0].len = wr_buff_len;

    NVIC_EnableIRQ(I2C0_IRQn);

    i2c_transfer_ret = I2C_TransferInit(I2C0, &i2c_transfer_seq);
    return i2c_transfer_ret;
}


I2C_TransferReturn_TypeDef I2C0_Read(uint8_t addr, uint8_t* rd_buff, uint8_t rd_buff_len) {
    I2C_TransferReturn_TypeDef i2c_transfer_ret;

    I2C0_Init();
    i2c_transfer_seq.addr = (addr << 1);
    i2c_transfer_seq.flags = I2C_FLAG_READ;
    i2c_transfer_seq.buf[0].data = rd_buff;
    i2c_transfer_seq.buf[0].len = rd_buff_len;

    NVIC_EnableIRQ(I2C0_IRQn);

    i2c_transfer_ret = I2C_TransferInit(I2C0, &i2c_transfer_seq);
    return i2c_transfer_ret;
}


I2C_TransferReturn_TypeDef I2C0_WriteRead(uint8_t addr, uint8_t* wr_buff, uint8_t wr_buff_len, uint8_t* rd_buff, uint8_t rd_buff_len) {
    I2C_TransferReturn_TypeDef i2c_transfer_ret;

    i2c_transfer_seq.addr = (addr << 1);
    i2c_transfer_seq.flags = I2C_FLAG_WRITE_READ;
    i2c_transfer_seq.buf[0].data = wr_buff;
    i2c_transfer_seq.buf[0].len = wr_buff_len;
    i2c_transfer_seq.buf[1].data = rd_buff;
    i2c_transfer_seq.buf[1].len = rd_buff_len;

    NVIC_EnableIRQ(I2C0_IRQn);

    i2c_transfer_ret = I2C_TransferInit(I2C0, &i2c_transfer_seq);
    return i2c_transfer_ret;
}


void I2C0_EnableIntForTransfer(void) {
    NVIC_EnableIRQ(I2C0_IRQn);
}


void I2C0_DisableIntForTransfer(void) {
    NVIC_DisableIRQ(I2C0_IRQn);
}
