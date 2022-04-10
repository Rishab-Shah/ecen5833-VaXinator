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

typedef enum
{
  FLASH_ADD_VERIFN,

  FLASH_READ_STATUS_REGISTER,
  FLASH_SETMODE,
  FLASH_SETMODE_2,
#if 0
  BNO055_SETMODE_DELAY_1,

  BNO055_RESET,
  BNO055_RESET_DELAY_2,

  BNO055_READ_POST_RESET,
  BNO055_READ_POST_RESET_DELAY_3,

  BNO055_NORMAL_MODE_SET,
  BNO055_NORMAL_MODE_SET_DELAY_4,

  BNO055_PAGE_ADDR,
  BNO055_PAGE_ADDR_DELAY_5,

  BNO055_SYS_TRIGGER,
  BNO055_SYS_TRIGGER_DELAY_6,

  BNO055_SET_REQ_MODE,
  BNO055_SET_REQ_MODE_DELAY_7,

  READ_XYZ_DATA,
  READ_XYZ_DATA_DELAY,
#endif
  FLASH_DEFAULT,
}FLASH_state_t;




#include "sl_status.h"
#include "em_usart.h"
#include "em_cmu.h"

#define SL_SPIDRV_FLASH_MEM_PERIPHERAL           (USART1)
#define SL_SPIDRV_FLASH_MEM_PERIPHERAL_NO        (1)

// USART1 TX on PF5
#define SL_SPIDRV_FLASH_MEM_TX_PORT              (gpioPortF)
#define SL_SPIDRV_FLASH_MEM_TX_PIN               (5)
#define SL_SPIDRV_FLASH_MEM_TX_LOC               (29)

// USART1 RX on PF6
#define SL_SPIDRV_FLASH_MEM_RX_PORT              (gpioPortF)
#define SL_SPIDRV_FLASH_MEM_RX_PIN               (6)
#define SL_SPIDRV_FLASH_MEM_RX_LOC               (29)

// USART1 CLK on PF4
#define SL_SPIDRV_FLASH_MEM_CLK_PORT             (gpioPortF)
#define SL_SPIDRV_FLASH_MEM_CLK_PIN              (4)
#define SL_SPIDRV_FLASH_MEM_CLK_LOC              (26)

// USART1 CS on PF7
#define SL_SPIDRV_FLASH_MEM_CS_PORT              (gpioPortF)
#define SL_SPIDRV_FLASH_MEM_CS_PIN               (7)
#define SL_SPIDRV_FLASH_MEM_CS_LOC               (28)

#define RX_DMA_CHANNEL                           (0)
#define TX_DMA_CHANNEL                           (1)


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
void start_ldma_transfer(uint8_t tx_rx_channel_selection);
void stop_ldma_transfer(uint8_t tx_rx_channel_selection);
void initTransferLDMA(void);

#endif /* SRC_SPI_H_ */
