/*
 * uart.c
 *
 *  Created on: Feb 25, 2022
 *      Author: rishab
 */

#include "uart.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

#if 0
void LEUART0_Init()
{
  const USART_InitAsync_TypeDef initUSART;

  initUSART.enable        = usartEnable;
  initUSART.refFreq       = 0;
  initUSART.baudrate      = 9600;
  //initUSART.oversampling  =
  initUSART.databits      = usartDatabits8;
  initUSART.parity        = usartNoParity;
  initUSART.stopbits      = usartStopbits1;
  //initUSART.mvdis
  //initUSART.prsRxEnable
  //initUSART.prsRxCh
  //initUSART.autoCsEnable
  //initUSART.autoCsHold
  //initUSART.autoCsSetup
  initUSART.hwFlowControl = usartHwFlowControlNone;

  //USART_InitAsync(USART_TypeDef *usart, &initUSART);
}
#endif

