/*
 * leuart.h
 *
 *  Created on: Feb 25, 2022
 *      Author: rishab and mukta
 */

#ifndef SRC_LEUART_H_
#define SRC_LEUART_H_


#if 1
#include <em_usart.h>
#include <stdbool.h>
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"

#include "sl_status.h"
#include "src/timers.h"

#include "sl_uartdrv_leuart_vcom2_config.h"
#include "uartdrv.h"
#include "em_leuart.h"

typedef struct GNSS_data_t{
  char header[7];
  char utctime[10];
  char latitude[11];
  float flat;
  char longitude[12];
  float flon;
  char gspeed[5];
  uint16_t gspd;
} GNSS_data_t;


typedef struct {
  USART_TypeDef *usart;
  CMU_Clock_TypeDef clock;
  uint8_t tx_port;
  uint8_t tx_pin;
  uint8_t tx_loc;
  uint8_t rx_port;
  uint8_t rx_pin;
  uint8_t rx_loc;
} sli_gps_uart_handle_t;


sl_status_t sli_gps_uart_init(sli_gps_uart_handle_t *handle, int baudrate, USART_ClockMode_TypeDef mode);
sl_status_t sli_gps_uart_tx(sli_gps_uart_handle_t *handle, const void *data, unsigned len);
sl_status_t sli_gps_uart_rx(sli_gps_uart_handle_t *handle, const void *data, unsigned len);


void LEUART0_Init();
void LEUART_rx_callback(UARTDRV_Handle_t handle, Ecode_t transferStatus, uint8_t *data,
                             UARTDRV_Count_t transferCount);

void initLeuart(void);
int echo_test(void);
void LEUART_Transmit(uint8_t * buf, int size, int rx_size);
void LEUART_Receive(uint8_t * buf, int size);

#endif

#endif /* SRC_LEUART_H_ */
