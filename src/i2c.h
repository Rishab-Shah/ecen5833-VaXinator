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
#include "sl_i2cspm.h"
#include "sl_i2cspm_sensor_config.h"

#define TEMP_SENSOR_POWER_PORT  (gpioPortD)
#define TEMP_SENSOR_POWER_PIN   (15)

#define TEMP_SENSOR_ADDR        (0x40)

#define CMD_READ_TEMP           (0xF3)


#define TEMP_SENSOR_READ_TEMP_WAIT_US (10800)
#define TEMP_SENSOR_POWER_ON_WAIT_US  (80000)


/*
 * Initializes the temperature sensor
 *
 * @param None
 *
 * @return None
 */
void temperature_sensor_Init(void);


/*
 * Enables the temperature sensor
 *
 * @param None
 *
 * @return None
 */
void temperature_sensor_Enable(bool enable);


/*
 * Reads data from temperature sensor
 *
 * @param read_buff - Buffer to fill with read data
 * @param read_buff_len - Number of bytes to read
 *
 * @return number of bytes read
 */
I2C_TransferReturn_TypeDef temperature_sensor_ReadTemp(uint8_t* read_buff, uint8_t read_buff_len);


/*
 * Sends command to temperature sensor
 *
 * @param command - Command to send to temperature sensor
 *
 * @return number of bytes written
 */
I2C_TransferReturn_TypeDef temperature_sensor_SendCommand(uint8_t command);

#endif /* SRC_I2C_H_ */
