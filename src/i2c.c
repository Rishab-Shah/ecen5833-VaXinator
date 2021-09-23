/*
 * i2c.c - I2C functions
 *
 *  Created on: Sep 13, 2021
 *      Author: vishn
 * Attribution: The temp_sensor_init values were sourced
 *              from sl_i2csmp_init.c.
 */


#include "i2c.h"

#define INCLUDE_LOG_DEBUG 0
#include "src/log.h"

I2CSPM_Init_TypeDef temp_sensor_init = {
    .port = SL_I2CSPM_SENSOR_PERIPHERAL,
    .sclPort = SL_I2CSPM_SENSOR_SCL_PORT,
    .sclPin = SL_I2CSPM_SENSOR_SCL_PIN,
    .sdaPort = SL_I2CSPM_SENSOR_SDA_PORT,
    .sdaPin = SL_I2CSPM_SENSOR_SDA_PIN,
    .portLocationScl = SL_I2CSPM_SENSOR_SCL_LOC,
    .portLocationSda = SL_I2CSPM_SENSOR_SDA_LOC,
    .i2cRefFreq = 0,
    .i2cMaxFreq = I2C_FREQ_STANDARD_MAX,
    .i2cClhr = i2cClockHLRStandard
};


I2C_TransferSeq_TypeDef i2c_transfer_seq;
uint8_t i2c_read_buffer[2] = { 0 };
uint8_t temp_sensor_command = CMD_READ_TEMP;


void I2C0_Init(void) {
    I2CSPM_Init(&temp_sensor_init);

    GPIO_DriveStrengthSet(TEMP_SENSOR_POWER_PORT, gpioDriveStrengthWeakAlternateWeak);
    GPIO_PinModeSet(TEMP_SENSOR_POWER_PORT, TEMP_SENSOR_POWER_PIN, gpioModePushPull, false);
}


void I2C0_Enable(bool enable) {
    if (enable) {
        GPIO_PinOutSet(TEMP_SENSOR_POWER_PORT, TEMP_SENSOR_POWER_PIN);
    }
    else {
        GPIO_PinOutClear(TEMP_SENSOR_POWER_PORT, TEMP_SENSOR_POWER_PIN);
    }
}


I2C_TransferReturn_TypeDef I2C0_SendCommand(uint8_t command) {
    I2C_TransferReturn_TypeDef i2c_transfer_ret;
    temp_sensor_command = command;

    i2c_transfer_seq.addr = (TEMP_SENSOR_ADDR << 1);
    i2c_transfer_seq.flags = I2C_FLAG_WRITE;
    i2c_transfer_seq.buf[0].data = &temp_sensor_command;
    i2c_transfer_seq.buf[0].len = 1;

    NVIC_EnableIRQ(I2C0_IRQn);

    i2c_transfer_ret = I2C_TransferInit(SL_I2CSPM_SENSOR_PERIPHERAL, &i2c_transfer_seq);
    return i2c_transfer_ret;
}


I2C_TransferReturn_TypeDef I2C0_RequestRead(void) {
    I2C_TransferReturn_TypeDef i2c_transfer_ret;

    i2c_transfer_seq.addr = (TEMP_SENSOR_ADDR << 1);
    i2c_transfer_seq.flags = I2C_FLAG_READ;
    i2c_transfer_seq.buf[0].data = i2c_read_buffer;
    i2c_transfer_seq.buf[0].len = 2;

    NVIC_EnableIRQ(I2C0_IRQn);

    i2c_transfer_ret = I2C_TransferInit(SL_I2CSPM_SENSOR_PERIPHERAL, &i2c_transfer_seq);
    return i2c_transfer_ret;
}


void I2C0_ReadBytes(uint8_t* read_buff, uint8_t read_buff_len) {
    memcpy(read_buff, i2c_read_buffer, read_buff_len);
}


void I2C0_EnableIntForTransfer(void) {
    NVIC_EnableIRQ(I2C0_IRQn);
}


void I2C0_DisableIntForTransfer(void) {
    NVIC_DisableIRQ(I2C0_IRQn);
}


void I2C0_Teardown(void) {
    GPIO_PinModeSet(TEMP_SENSOR_POWER_PORT, TEMP_SENSOR_POWER_PORT, gpioModeDisabled, 0);
    GPIO_PinModeSet(SL_I2CSPM_SENSOR_SCL_PORT, SL_I2CSPM_SENSOR_SCL_PIN, gpioModeDisabled, 0);
    GPIO_PinModeSet(SL_I2CSPM_SENSOR_SDA_PORT, SL_I2CSPM_SENSOR_SDA_PIN, gpioModeDisabled, 0);

    CMU_ClockEnable(cmuClock_I2C0, false);
}


/*int16_t I2C0_GetTempReading(void) {
    uint8_t temp_buffer[2];
    uint16_t temp_buffer_len = 2;
    I2C_TransferReturn_TypeDef i2c_ret;
    int16_t temperature_code;
    int16_t temperature_C;

    I2C0_Enable(true);
    timerWaitUs_polled(TEMP_SENSOR_POWER_ON_WAIT_US);
    i2c_ret = I2C0_SendCommand(CMD_READ_TEMP);
    if (i2c_ret != i2cTransferDone) {
        LOG_ERROR("I2C Write Error\r\n");
        return -1;
    }
    timerWaitUs_polled(TEMP_SENSOR_READ_TEMP_WAIT_US);

    i2c_ret = I2C0_RequestRead(temp_buffer, temp_buffer_len);
    if (i2c_ret != i2cTransferDone) {
        LOG_ERROR("I2C Read Error\r\n");
        return -1;
    }

    timerWaitUs_polled(TEMP_SENSOR_POWER_ON_WAIT_US);
    I2C0_Enable(false);

    temperature_code = (temp_buffer[0] << 8) | temp_buffer[1];
    temperature_C = (175.25 * temperature_code) / 65536.0 - 46.85; //calculation taken from data sheet

    return temperature_C;
}*/
