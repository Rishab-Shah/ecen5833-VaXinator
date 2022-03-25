/*
 * uart.c
 *
 *  Created on: Feb 25, 2022
 *      Author: rishab
 */

#include <src/leuart.h>

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

#if 1
#define packetLength 66
uint8_t leuartbuffer[100];
char gnssarray[100];
UARTDRV_HandleData_t leuartHandle0; /* UART driver handle */
UARTDRV_Handle_t  gnssHandle0 = &leuartHandle0;
GNSS_data_t GNRMC_data = {
    "$GNRMC",
    "000000.00",
    "00000.0000",
    0.0,
    "00000.0000",
    0.0,
    "0.00",
    0
};

void LEUART_rx_callback(UARTDRV_Handle_t handle, Ecode_t transferStatus, uint8_t *data, UARTDRV_Count_t transferCount){

  LOG_INFO("In-here\r");
  if(transferStatus == ECODE_EMDRV_UARTDRV_OK){
    char *gnssstr;
    // check if gnss packet starts with the GNRMC (Recommended Minimum Specific GNSS Data) and UTC time stamp is populated
    //gpioGpsToggleSetOff();
    gnssstr = strstr((char *)data, "$GNRMC");
    if((gnssstr != 0) && (strlen(gnssstr) > 56) && !(strncmp(gnssstr, "$GNRMC,,", 8) == 0)){
      // split gnssstr into data we want
      strncpy(GNRMC_data.header, gnssstr, 6);
      strncpy(GNRMC_data.utctime, &gnssstr[7], 9);
      strncpy(GNRMC_data.latitude, &gnssstr[19], 10);
      if(strchr(GNRMC_data.latitude, ',') != NULL) return;  // check if gps lock is complete (in case of empty values during warmup)
      strncpy(GNRMC_data.longitude, &gnssstr[32], 11);
      strncpy(GNRMC_data.gspeed, &gnssstr[46], 4);

      // convert strings to floats for display
      GNRMC_data.flon = strtof(GNRMC_data.longitude, NULL) / 100;
      GNRMC_data.flat = strtof(GNRMC_data.latitude, NULL) / 100;
      float fgspeed = strtof(GNRMC_data.gspeed, NULL);
      GNRMC_data.gspd = (uint16_t)(FLT_TO_UINT32(fgspeed, 0) >> 1); // convert speed in knots to m/s. units is 1/100 of a m/s
      //gpioGpsToggleSetOn();
    }
  }
}

void LEUART0_Init()
{
#if 0
  USART_InitAsync_TypeDef initUSART = USART_INITASYNC_DEFAULT;
  USART_TypeDef usart_serial;

  CMU_ClockEnable(cmuClock_GPIO, true);

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

  //void USART_InitAsync(USART_TypeDef *usart, const USART_InitAsync_TypeDef *init)
  USART_InitAsync(&usart_serial, &initUSART);
#endif

  UARTDRV_InitLeuart_t initData = LEUART_INIT_DEFAULT;

  DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_RX_BUFS, rxBufferQueueI0);
  DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_TX_BUFS, txBufferQueueI0);

  initData.port                 = SL_UARTDRV_LEUART_VCOM2_PERIPHERAL;
  initData.baudRate             = 9600;
  initData.portLocationTx       = SL_UARTDRV_LEUART_VCOM2_TX_LOC;
  initData.portLocationRx       = SL_UARTDRV_LEUART_VCOM2_RX_LOC;
  initData.stopBits             = (LEUART_Stopbits_TypeDef) LEUART_CTRL_STOPBITS_ONE;
  initData.parity               = (LEUART_Parity_TypeDef) LEUART_CTRL_PARITY_NONE;
  initData.fcType               = uartdrvFlowControlNone;
  initData.rxQueue              = (UARTDRV_Buffer_FifoQueue_t *)&rxBufferQueueI0;
  initData.txQueue              = (UARTDRV_Buffer_FifoQueue_t *)&txBufferQueueI0;

  UARTDRV_InitLeuart(gnssHandle0, &initData);



  LOG_INFO("COMPLETE\r");
}
#endif

#if 0
sl_status_t sli_gps_uart_init(sli_gps_uart_handle_t *handle, int baudrate, USART_ClockMode_TypeDef mode)
{
  USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;
  USART_TypeDef *usart = handle->usart;

  CMU_ClockEnable(cmuClock_GPIO, true);//cmuClock_LEUART0
  CMU_ClockEnable(handle->clock, true);

  GPIO_PinModeSet((GPIO_Port_TypeDef)handle->clk_port, handle->clk_pin, gpioModePushPull, 0);
  GPIO_PinModeSet((GPIO_Port_TypeDef)handle->mosi_port, handle->mosi_pin, gpioModePushPull, 0);

  init.baudrate = baudrate;
  init.clockMode = mode;

  USART_InitSync(usart, &init);


  usart->ROUTELOC0 = (handle->mosi_loc << _USART_ROUTELOC0_TXLOC_SHIFT)
                     | (handle->clk_loc << _USART_ROUTELOC0_CLKLOC_SHIFT);
  usart->ROUTEPEN = USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_CLKPEN;


  return SL_STATUS_OK;
}
#endif
