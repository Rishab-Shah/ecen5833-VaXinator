/*
 * spi.h
 *
 *  Created on: Mar 4, 2022
 *      Author: risha
 */

#ifndef SRC_SPI_H_
#define SRC_SPI_H_

#include <stdbool.h>

#include "timers.h"
#include "em_letimer.h"
#include "gpio.h"
#include "ble.h"
#include "scheduler.h"


//Timer
#define SPI_SETMODE_DELAY               ((30)*(MSEC_TO_USEC))
//#define NORMAL_MODE_DELAY           ((50)*(MSEC_TO_USEC))
//#define STD_DELAY                   ((10)*(MSEC_TO_USEC))
//#define POST_RESET_STARTUP_DELAY    ((500)*(MSEC_TO_USEC))



#define FLASH_ADDRESS                   (0xAB)
#define FLASH_WR_EN_CMD                 (0x06)
#define FLASH_WRITE_DISABLE_CMD         (0x04)
#define FLASH_BYTE_PROG_CMD             (0x02)
#define FLASH_READ_20MHZ_CMD            (0x03)
#define FLASH_READ_STATUS_REG_CMD       (0x05)
#define FLASH_EN_WR_STATUS_REG_CMD      (0x50)
#define FLASH_WR_STATUS_REG_CMD         (0x01)
#define FLASH_BLOCK_ERASE_CMD           (0x52)

#define FLASH_BPL_NONE                  (0x00)
#define FLASH_BPL_0C000H_0FFFFH         (0x04)
#define FLASH_BPL_08000H_0FFFFH         (0x08)
#define FLASH_BPL_00000H_0FFFFH         (0x0C)

typedef enum
{
  FLASH_ADD_VERIFN,

  FLASH_WR_ENABLE,
  FLASH_BYTE_PROG,
  FLASH_READ_BYTE,
  FLASH_READ_STATUS_REG,
  FLASH_BLOCK_ERASE,
  FLASH_EN_WR_BP_PROG_STATUS_REG,

  FLASH_READ_STATUS_REGISTER,
  FLASH_SETMODE,
  FLASH_SETMODE_2,

  FLASH_DEFAULT,
}FLASH_state_t;




#include "sl_status.h"
#include "em_usart.h"
#include "em_cmu.h"




#define SEND_AS_DATA          (0)
#define SEND_AS_ADDRESS       (1)


//FLASH_state_t init_flash_machine(sl_bt_msg_t *evt);
FLASH_state_t init_flash_setup(sl_bt_msg_t *evt);

void flash_spi_init();
void flash_spi_usart_configuration();
void trigger_CE_high_to_low_transition();
void trigger_CE_low_to_high_transition();

void update_write_params(uint8_t command, uint16_t write_count,
                         uint8_t tempbuffer[],uint8_t data_or_address);

void update_read_params(uint16_t read_count);

#endif /* SRC_SPI_H_ */
