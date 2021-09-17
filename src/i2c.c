/*
 * i2c.c - I2C functions
 *
 *  Created on: Sep 13, 2021
 *      Author: vishn
 * Attribution: The temp_sensor_init values were sourced
 *              from sl_i2csmp_init.c.
 */


#include "i2c.h"

#define INCLUDE_LOG_DEBUG 1
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


I2C_TransferReturn_TypeDef temp_sensor_transfer_ret;
I2C_TransferSeq_TypeDef temp_sensor_transfer_seq;
uint8_t temp_sensor_command = CMD_READ_TEMP;


void temperature_sensor_Init(void) {
    I2CSPM_Init(&temp_sensor_init);

    GPIO_DriveStrengthSet(TEMP_SENSOR_POWER_PORT, gpioDriveStrengthWeakAlternateWeak);
    GPIO_PinModeSet(TEMP_SENSOR_POWER_PORT, TEMP_SENSOR_POWER_PIN, gpioModePushPull, false);
}


I2C_TransferReturn_TypeDef temperature_sensor_SendCommand(uint8_t command) {
    temp_sensor_command = command;

    temp_sensor_transfer_seq.addr = (TEMP_SENSOR_ADDR << 1);
    temp_sensor_transfer_seq.flags = I2C_FLAG_WRITE;
    temp_sensor_transfer_seq.buf[0].data = &temp_sensor_command;
    temp_sensor_transfer_seq.buf[0].len = 1;

    temp_sensor_transfer_ret = I2CSPM_Transfer(SL_I2CSPM_SENSOR_PERIPHERAL, &temp_sensor_transfer_seq);
    return temp_sensor_transfer_ret;
}


void temperature_sensor_Enable(bool enable) {
    if (enable) {
        GPIO_PinOutSet(TEMP_SENSOR_POWER_PORT, TEMP_SENSOR_POWER_PIN);
    }
    else {
        GPIO_PinOutClear(TEMP_SENSOR_POWER_PORT, TEMP_SENSOR_POWER_PIN);
    }
}


I2C_TransferReturn_TypeDef temperature_sensor_ReadFromSensor(uint8_t* read_buff, uint8_t read_buff_len) {
    temp_sensor_transfer_seq.addr = (TEMP_SENSOR_ADDR << 1);
    temp_sensor_transfer_seq.flags = I2C_FLAG_READ;
    temp_sensor_transfer_seq.buf[0].data = read_buff;
    temp_sensor_transfer_seq.buf[0].len = read_buff_len;

    temp_sensor_transfer_ret = I2CSPM_Transfer(SL_I2CSPM_SENSOR_PERIPHERAL, &temp_sensor_transfer_seq);
    return temp_sensor_transfer_ret;
}


int16_t temperature_sensor_GetTempReading(void) {
    uint8_t temp_buffer[2];
    uint16_t temp_buffer_len = 2;
    I2C_TransferReturn_TypeDef i2c_ret;
    int16_t temperature_code;
    int16_t temperature_C;

    temperature_sensor_Enable(true);
    timerWaitUs(TEMP_SENSOR_POWER_ON_WAIT_US);
    i2c_ret = temperature_sensor_SendCommand(CMD_READ_TEMP);
    if (i2c_ret != i2cTransferDone) {
        LOG_ERROR("I2C Write Error\r\n");
        return -1;
    }
    timerWaitUs(TEMP_SENSOR_READ_TEMP_WAIT_US);

    i2c_ret = temperature_sensor_ReadFromSensor(temp_buffer, temp_buffer_len);
    if (i2c_ret != i2cTransferDone) {
        LOG_ERROR("I2C Read Error\r\n");
        return -1;
    }

    timerWaitUs(TEMP_SENSOR_POWER_ON_WAIT_US);
    temperature_sensor_Enable(false);

    temperature_code = (temp_buffer[0] << 8) | temp_buffer[1];
    temperature_C = (175.25 * temperature_code) / 65536.0 - 46.85; //calculation taken from data sheet

    return temperature_C;
}
