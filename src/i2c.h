/*
 * i2c.h - I2C functions
 *
 *  Created on: Sep 13, 2021
 *      Author: vishn
 */

#ifndef SRC_I2C_H_
#define SRC_I2C_H_

#include <stdbool.h>
#include "em_i2c.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "sl_i2cspm.h"
#include "sl_i2cspm_sensor_config.h"
#include "src/timers.h"

#define TEMP_SENSOR_POWER_PORT  (gpioPortD)
#define TEMP_SENSOR_POWER_PIN   (15)

#define TEMP_SENSOR_ADDR        (0x40)

#define CMD_READ_TEMP           (0xF3)


#define TEMP_SENSOR_READ_TEMP_WAIT_US (10800)
#define TEMP_SENSOR_POWER_ON_WAIT_US  (80000)


/*
 * Initializes the I2C0 module
 *
 * @param None
 *
 * @return None
 */
void I2C0_Init(void);


/*
 * Enables the I2C0 module
 *
 * @param None
 *
 * @return None
 */
void I2C0_Enable(bool enable);


/*
 * Sends command to I2C0
 *
 * @param command - Command to send to temperature sensor
 *
 * @return status of write
 */
I2C_TransferReturn_TypeDef I2C0_SendCommand(uint8_t command);


/*
 * Requests an I2C0 read into i2c_read_buffer
 *
 * @param None
 *
 * @return status of read
 */
I2C_TransferReturn_TypeDef I2C0_RequestRead(void);


/*
 * Copies data stored in i2c_read_buffer into read_buff
 *
 * @param read_buff - Buffer to fill with read data
 * @param read_buff_len - Number of bytes to read
 *
 * @return None
 */
void I2C0_ReadBytes(uint8_t* read_buff, uint8_t read_buff_len);


/*
 * Enables I2C0 interrupts for transfer
 *
 * @param None
 *
 * @return None
 */
void I2C0_EnableIntForTransfer(void);


/*
 * Disables I2C0 interrupts for transfer
 *
 * @param None
 *
 * @return None
 */
void I2C0_DisableIntForTransfer(void);


/*
 * Tears down the I2C0 module
 *
 * @param None
 *
 * @return None
 */
void I2C0_Teardown(void);


/*
 * Gets temperature reading
 *
 * @param None
 *
 * @return temperature in Celsius
 */
//int16_t I2C0_GetTempReading(void);

#endif /* SRC_I2C_H_ */
