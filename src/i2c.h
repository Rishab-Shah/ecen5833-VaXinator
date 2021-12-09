/*
 * i2c.h - I2C functions
 *
 *  Created on: Sep 13, 2021
 *      Author: vishnu and rishab
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


/*
 * Initializes the I2C0 module
 *
 * @param None
 *
 * @return None
 */
void I2C0_Init(void);


/*
 * Writes to I2C0 bus
 *
 * @param addr - Slave address
 * @param wr_buff - Buffer to write to slave
 * @param wr_buff_len - Length of wr_buff
 *
 * @return status of write
 */
I2C_TransferReturn_TypeDef I2C0_Write(uint8_t addr, uint8_t* wr_buff, uint8_t wr_buff_len);


/*
 * Reads from I2C0 bus
 *
 * @param addr - Slave address
 * @param rd_buff - Buffer to fill with bytes read from slave
 * @param rd_buff_len - Length of rd_buff
 *
 * @return status of read
 */
I2C_TransferReturn_TypeDef I2C0_Read(uint8_t addr, uint8_t* rd_buff, uint8_t rd_buff_len);


/*
 * Writes and reads (Start and restart I2C sequence) from I2C0 bus
 *
 * @param addr - Slave address
 * @param wr_buff - Buffer to write to slave
 * @param wr_buff_len - Length of wr_buff
 * @param rd_buff - Buffer to fill with bytes read from slave
 * @param rd_buff_len - Length of rd_buff
 *
 * @return None
 */
I2C_TransferReturn_TypeDef I2C0_WriteRead(uint8_t addr, uint8_t* wr_buff, uint8_t wr_buff_len, uint8_t* rd_buff, uint8_t rd_buff_len);


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


#endif /* SRC_I2C_H_ */
