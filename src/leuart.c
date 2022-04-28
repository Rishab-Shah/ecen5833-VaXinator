/*
 * uart.c
 *
 *  Created on: Feb 25, 2022
 *      Author: rishab and mukta
 */

#include <src/leuart.h>

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

#if 0
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

#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_leuart.h"
#include "em_chip.h"

#define RX_BUFFER_SIZE 200             // Software receive buffer size

static bool rxDataReady = 0, txInProcess = 0;      // Flag indicating receiver does not have data
static volatile char rxBuffer[RX_BUFFER_SIZE]; // Software receive buffer
static char txBuffer[RX_BUFFER_SIZE]; // Software transmit buffer
static uint32_t txSize = 0, rxSize = 0;

/**************************************************************************//**
 * @brief
 *    Initialize the LEUART module
 *****************************************************************************/
void initLeuart(void)
{

  // GPIO clock
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Initialize LEUART0 TX and RX pins
  GPIO_PinModeSet(gpioPortC, 11, gpioModePushPull, 1); // TX
  GPIO_PinModeSet(gpioPortC, 10, gpioModeInput, 0);    // RX

  // Enable LE (low energy) clocks
  CMU_ClockEnable(cmuClock_HFLE, true); // Necessary for accessing LE modules
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO); // Set a reference clock

  // Enable clocks for LEUART0
  CMU_ClockEnable(cmuClock_LEUART0, true);
  CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_1); // Don't prescale LEUART clock

  // Initialize the LEUART0 module
  LEUART_Init_TypeDef init = LEUART_INIT_DEFAULT;
  LEUART_Init(LEUART0, &init);

  // Enable LEUART0 RX/TX pins on PD[11:10] (see readme.txt for details)
  LEUART0->ROUTEPEN  = LEUART_ROUTEPEN_RXPEN | LEUART_ROUTEPEN_TXPEN;
  LEUART0->ROUTELOC0 = LEUART_ROUTELOC0_RXLOC_LOC14 | LEUART_ROUTELOC0_TXLOC_LOC16;

  // Enable LEUART0 RX/TX interrupts
  LEUART_IntEnable(LEUART0, LEUART_IEN_RXDATAV | LEUART_IEN_TXC);
  NVIC_EnableIRQ(LEUART0_IRQn);
}

/**************************************************************************//**
 * @brief
 *    LEUART0 interrupt service routine
 *
 * @details
 *    Keep receiving data while there is still data left in the hardware RX buffer.
 *    Store incoming data into rxBuffer and set rxDataReady when a linefeed '\n' is
 *    sent or if there is no more room in the buffer.
 *****************************************************************************/
void LEUART0_IRQHandler(void)
{
  // Note: These are static because the handler will exit/enter
  //       multiple times to fully transmit a message.
  static uint32_t rxIndex = 0;
  static uint32_t txIndex = 0;

  // Acknowledge the interrupt
  uint32_t flags = LEUART_IntGet(LEUART0);
  LEUART_IntClear(LEUART0, flags);

  // RX portion of the interrupt handler
  if (flags & LEUART_IF_RXDATAV) {
    while (LEUART0->STATUS & LEUART_STATUS_RXDATAV) { // While there is still incoming data
      char data = LEUART_Rx(LEUART0);
      if ((rxIndex < RX_BUFFER_SIZE - 2) && (rxIndex < rxSize)) { // Save two spots for '\n' and '\0'
        rxBuffer[rxIndex++] = data;
      } else { // Done receiving
        rxBuffer[rxIndex++] = '\r';
        rxBuffer[rxIndex++] = '\n';
        rxBuffer[rxIndex++] = '\0';
        rxDataReady = 1;
        //rxSize = rxIndex;
        rxIndex = 0;
        break;
      }
    }
  }

  // TX portion of the interrupt handler
  if (flags & LEUART_IF_TXC) {
    if ((txIndex < RX_BUFFER_SIZE) && (txIndex < txSize)) {
      LEUART_Tx(LEUART0, txBuffer[txIndex++]); // Send the data
    } else { // Done transmitting
      txIndex = 0;
      txInProcess = 0;
      LEUART_IntDisable(LEUART0, LEUART_IEN_TXC); // Disable interrupts
    }
  }
}

int echo_test(void)
{
  uint32_t i;
  // Print the welcome message
  char welcomeString[] = "LEUART echo code example\r\n";
  for (i = 0 ; welcomeString[i] != 0; i++) {
    txBuffer[i] = welcomeString[i];
  }
  txBuffer[i] = '\0';
  LEUART_IntSet(LEUART0, LEUART_IFS_TXC);

  while (1) {

      if(rxDataReady)
        {
          LEUART_IntDisable(LEUART0, LEUART_IEN_RXDATAV | LEUART_IEN_TXC); // Disable interrupts
          rxDataReady=0;

          LOG_INFO("RX - %s\r\n", rxBuffer);
          memset(rxBuffer, 0, RX_BUFFER_SIZE);
          LEUART_IntEnable(LEUART0, LEUART_IEN_RXDATAV | LEUART_IEN_TXC); // Re-enable interrupts
          txInProcess = 1;
          LEUART_IntSet(LEUART0, LEUART_IFS_TXC);
        }
    // Wait for incoming data in EM2 to save energy
    //EMU_EnterEM2(false);
  }
}

void LEUART_Transmit(uint8_t * buf, int size, int rx_size)
{
  int i = 0;

  while(txInProcess);

  LEUART_IntDisable(LEUART0, LEUART_IEN_RXDATAV | LEUART_IEN_TXC); // Disable interrupts

  rxDataReady=0;

  memset(txBuffer, 0, RX_BUFFER_SIZE);

  for (i = 0 ; i < size; i++) {
    txBuffer[i] = buf[i];
  }

  txSize = size;
  rxSize = rx_size;
  LEUART_IntEnable(LEUART0, LEUART_IEN_RXDATAV | LEUART_IEN_TXC); // Re-enable interrupts
  txInProcess = 1;
  LEUART_IntSet(LEUART0, LEUART_IFS_TXC);

}

void LEUART_Receive(uint8_t * buf, int size)
{
  int i = 0;

  while(!rxDataReady || txInProcess)
    {
      timerWaitUs_polled(100000);
      i++;
      if(i>20)
        break;
    }

  LEUART_IntDisable(LEUART0, LEUART_IEN_RXDATAV | LEUART_IEN_TXC); // Disable interrupts

  rxDataReady=0;

//  if ((size != rxSize) || (size != (rxSize+2)))
//    return;

  for (i = 0 ; i < rxSize; i++) {
      buf[i] = rxBuffer[i];
  }
  rxSize = 0;
  memset(rxBuffer, 0, RX_BUFFER_SIZE);
}

