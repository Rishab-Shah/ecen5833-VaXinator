/*
 * spi.c
 *
 *  Created on: Mar 4, 2022
 *      Author: rishab
 */

#include "spi.h"

#include "sl_spidrv_FLASH_MEM_config.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_ldma.h"

/*******************************************************************************
 Headers
*******************************************************************************/
//#include <src/BME280.h>
// Include logging for this file
#define INCLUDE_LOG_DEBUG       (1)
#include "src/log.h"
#include "ble.h"

/*******************************************************************************
 Macros
*******************************************************************************/
#define ERROR                       (-1)
#define DEBUG_1                     (1)
#define MHZ_1                       (1000000)
#define FLASH_OPERATING_FREQUENCY   ((4)*(MHZ_1))

#define TX_BUFFER_SIZE              (10)
#define RX_BUFFER_SIZE              (TX_BUFFER_SIZE)
#define SANITY_DELAY                ((0.5)*(MSEC_TO_USEC))

/*******************************************************************************
 Global
*******************************************************************************/
uint8_t TxBuffer[TX_BUFFER_SIZE];// = {0x03, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t RxBuffer[RX_BUFFER_SIZE];
LDMA_Descriptor_t ldmaTXDescriptor;
LDMA_TransferCfg_t ldmaTXConfig;

LDMA_Descriptor_t ldmaRXDescriptor;
LDMA_TransferCfg_t ldmaRXConfig;

uint8_t rx_dma_channel;
uint8_t tx_dma_channel;
/*******************************************************************************
 Function Prototypes
*******************************************************************************/
void initTransferLDMA(void);
void init_ldma_params();
/*******************************************************************************
 Function Definition
*******************************************************************************/
void flash_spi_init()
{
  // Configure GPIO mode
  flash_spi_usart_configuration();

  //init_ldma_params();
}

void init_ldma_params()
{
  CMU_ClockEnable(cmuClock_LDMA, true);

  rx_dma_channel = RX_DMA_CHANNEL;
  tx_dma_channel = TX_DMA_CHANNEL;

  // Initializing the DMA
  LDMA_Init_t ldmaInit = LDMA_INIT_DEFAULT;
  LDMA_Init(&ldmaInit);

  //initTransferLDMA();
}


void update_write_params(uint8_t command, uint16_t write_count, uint8_t tempbuffer[],uint8_t data_or_address)//int8_t TxBuffer,uint16_t write_count
{
  USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;

  TxBuffer[0] = command;

  uint16_t iterator = 0;
  if(data_or_address == SEND_AS_DATA)
  {
    for(iterator = 1; iterator < (write_count+1); iterator++)
    {
        TxBuffer[iterator] = tempbuffer[iterator-1];
    }
  }
#if 0
  // LDMA descriptor and config for transferring TxBuffer
  // Source: TxBuffer, Destination: USART1->TXDATA, Bytes to send: 10
  ldmaTXDescriptor = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(TxBuffer, &(USART1->TXDATA),(write_count+1));
  // Setting the transfer to trigger once there is room in the USART1_TXDATA buffer
  ldmaTXConfig = (LDMA_TransferCfg_t)LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART1_TXBL);

#endif
}

void update_read_params(uint16_t read_count)//int8_t TxBuffer,uint16_t write_count
{
  memset(RxBuffer,0,sizeof(RxBuffer));
#if 0
  // LDMA descriptor and config for receiving data from the slave
  // Source: USART1->RXDATA, Destination: RxBuffer, Bytes to receive: 10
  ldmaRXDescriptor = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_SINGLE_P2M_BYTE(&(USART1->RXDATA),RxBuffer,read_count);
  // Setting the transfer to trigger whenever data comes into USART1_RXDATAV
  ldmaRXConfig = (LDMA_TransferCfg_t)LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART1_RXDATAV);
#endif
}

void start_ldma_transfer(uint8_t tx_rx_channel_selection)
{
  if(tx_rx_channel_selection == TX_DMA_CHANNEL)
  {
     LDMA_StartTransfer(TX_DMA_CHANNEL, &ldmaTXConfig, &ldmaTXDescriptor);
  }
  else if(tx_rx_channel_selection == RX_DMA_CHANNEL)
  {
     LDMA_StartTransfer(RX_DMA_CHANNEL, &ldmaRXConfig, &ldmaRXDescriptor);
  }
  else
  {
      /* Do nothing */
  }
}

void stop_ldma_transfer(uint8_t tx_rx_channel_selection)
{
  if(tx_rx_channel_selection == TX_DMA_CHANNEL)
  {
     LDMA_StartTransfer(TX_DMA_CHANNEL, &ldmaTXConfig, &ldmaTXDescriptor);
  }
  else if(tx_rx_channel_selection == RX_DMA_CHANNEL)
  {
     LDMA_StartTransfer(RX_DMA_CHANNEL, &ldmaRXConfig, &ldmaRXDescriptor);
  }
  else
  {
      /* Do nothing */
  }
}

void initTransferLDMA(void)
{
  trigger_CE_high_to_low_transition();
#if 1
  //update_write_params(4);

  update_read_params(2);
  //LDMA_StartTransfer(rx_dma_channel, &ldmaRXConfig, &ldmaRXDescriptor);

  start_ldma_transfer(TX_DMA_CHANNEL);

#endif

#if 0
  // LDMA descriptor and config for transferring TxBuffer
  ldmaTXDescriptor = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(TxBuffer, &(USART1->TXDATA), 4); // Source: TxBuffer, Destination: USART1->TXDATA, Bytes to send: 10
  ldmaTXConfig = (LDMA_TransferCfg_t)LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART1_TXBL);                  // Setting the transfer to trigger once there is room in the USART1_TXDATA buffer

  // LDMA descriptor and config for receiving data from the slave
  ldmaRXDescriptor = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_SINGLE_P2M_BYTE(&(USART1->RXDATA), RxBuffer,5); // Source: USART1->RXDATA, Destination: RxBuffer, Bytes to receive: 10
  ldmaRXConfig = (LDMA_TransferCfg_t)LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART1_RXDATAV);         // Setting the transfer to trigger whenever data comes into USART1_RXDATAV

  // Starting both transfers
  LDMA_StartTransfer(TX_DMA_CHANNEL, &ldmaTXConfig, &ldmaTXDescriptor);
  //LDMA_StartTransfer(RX_DMA_CHANNEL, &ldmaRXConfig, &ldmaRXDescriptor);
#endif
}


FLASH_state_t init_flash_setup(sl_bt_msg_t *evt)
{
  ble_ext_signal_event_t event = evt->data.evt_system_external_signal.extsignals;
  /* return state logic */
  FLASH_state_t return_state = FLASH_DEFAULT;
  /* current machine logic */
  FLASH_state_t currentState;
  static FLASH_state_t nextState = FLASH_ADD_VERIFN;
  //bool address_verification = false;

  int ret_status = 0;
  if (SL_BT_MSG_ID(evt->header) != sl_bt_evt_system_external_signal_id) {
      return return_state;
  }
  currentState = nextState;

  switch(currentState)
  {
    case FLASH_ADD_VERIFN:
    {
      if(event == ev_LETIMER0_UF)
      {
#if DEBUG_1
        LOG_INFO("FLASH_ADD_VERIFN\r");
#endif
        flash_spi_init();
        USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;
        trigger_CE_high_to_low_transition();

        uint8_t tx_sequence[TX_BUFFER_SIZE] = {FLASH_ADDRESS,0x00,0x00,0x00};
        USART_SpiTransfer(USART1, tx_sequence[0]);
        for (uint32_t i=1; i<4; i++)
        {
          USART_SpiTransfer(USART1,tx_sequence[i]);
        }

        //Read start
        uint8_t rx_sequence[RX_BUFFER_SIZE]; memset(rx_sequence,0,sizeof(rx_sequence));
        USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;
        for (uint32_t i=0; i<2; i++)
        {
            rx_sequence[i] = USART_SpiTransfer(USART1, 0xFF);
            LOG_INFO("rx_sequence[%d] = %x\r",i,rx_sequence[i]);
        }
        trigger_CE_low_to_high_transition();

        ret_status = timerWaitUs_irq(SANITY_DELAY);
        if(ret_status == ERROR)
        {
           LOG_ERROR("The value is more than the routine can provide\r");
        }
        else
        {
           nextState =  FLASH_EN_WR_BP_PROG_STATUS_REG;
        }
        //start_ldma_transfer(TX_DMA_CHANNEL);
        //update_read_params(1);
        //LDMA_StartTransfer(rx_dma_channel, &ldmaRXConfig, &ldmaRXDescriptor);
      }
      break;





    }
    case FLASH_EN_WR_BP_PROG_STATUS_REG:
    {
      if(event == ev_LETIMER0_COMP1)
      {
        USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;
        trigger_CE_high_to_low_transition();

        uint8_t tx_sequence[TX_BUFFER_SIZE] = {FLASH_EN_WR_STATUS_REG_CMD};
        USART_SpiTransfer(USART1, tx_sequence[0]);

        trigger_CE_low_to_high_transition();

        USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;
        trigger_CE_high_to_low_transition();

        memset(tx_sequence,0,sizeof(tx_sequence));
        //tx_sequence[TX_BUFFER_SIZE] = {FLASH_WR_STATUS_REG_CMD,0x00};
        tx_sequence[0] = FLASH_WR_STATUS_REG_CMD;
        tx_sequence[1] = 0x00;

        USART_SpiTransfer(USART1, tx_sequence[0]);
        USART_SpiTransfer(USART1, tx_sequence[1]);

        trigger_CE_low_to_high_transition();

        ret_status = timerWaitUs_irq(SANITY_DELAY);
        if(ret_status == ERROR)
        {
           LOG_ERROR("The value is more than the routine can provide\r");
        }
        else
        {
           nextState = FLASH_READ_STATUS_REG;
        }
      }
      break;
    }
    case FLASH_WR_ENABLE:
    {
      if(event == ev_LETIMER0_COMP1)
      {
        USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;

        trigger_CE_high_to_low_transition();
        uint8_t tx_sequence[TX_BUFFER_SIZE] = {FLASH_WR_EN_CMD};
        USART_SpiTransfer(USART1, tx_sequence[0]);

        trigger_CE_low_to_high_transition();

        nextState = FLASH_BYTE_PROG;
#if 0
        ret_status = timerWaitUs_irq(SANITY_DELAY);
        if(ret_status == ERROR)
        {
           LOG_ERROR("The value is more than the routine can provide\r");
        }
        else
        {
           nextState = FLASH_BYTE_PROG;
        }
#endif
      }
      break;
    }
    case FLASH_BYTE_PROG:
    {
      if(event == ev_LETIMER0_COMP1)
      {
        USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;
        trigger_CE_high_to_low_transition();

        uint8_t tx_sequence[TX_BUFFER_SIZE] = {FLASH_BYTE_PROG_CMD,0xFF,0xFF,0xFE,0xAA};
        USART_SpiTransfer(USART1, tx_sequence[0]);
        for (uint32_t i=1; i<5; i++)
        {
          USART_SpiTransfer(USART1,tx_sequence[i]);
        }

        for(int i = 0;i< 200;i++)
        {

        }

        trigger_CE_low_to_high_transition();

        ret_status = timerWaitUs_irq(SANITY_DELAY);
        if(ret_status == ERROR)
        {
           LOG_ERROR("The value is more than the routine can provide\r");
        }
        else
        {
           nextState = FLASH_READ_BYTE;
        }
      }
      break;
    }
    case FLASH_READ_BYTE:
    {
      if(event == ev_LETIMER0_COMP1)
      {
        USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;
        trigger_CE_high_to_low_transition();

        uint8_t tx_sequence[TX_BUFFER_SIZE] = {FLASH_READ_20MHZ_CMD,0xFF,0xFF,0xFE};
        USART_SpiTransfer(USART1, tx_sequence[0]);
        for (uint32_t i=1; i<4; i++)
        {
          USART_SpiTransfer(USART1,tx_sequence[i]);
        }

        //Read start
        uint8_t rx_sequence[RX_BUFFER_SIZE]; memset(rx_sequence,0,sizeof(rx_sequence));
        USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;
        for (uint32_t i=0; i<1; i++)
        {
            rx_sequence[i] = USART_SpiTransfer(USART1, 0xFF);
            LOG_INFO("rx_sequence[%d] = %x\r",i,rx_sequence[i]);
        }

        trigger_CE_low_to_high_transition();

        ret_status = timerWaitUs_irq(SANITY_DELAY);
        if(ret_status == ERROR)
        {
           LOG_ERROR("The value is more than the routine can provide\r");
        }
        else
        {

           nextState = FLASH_READ_STATUS_REG;
        }
      }
      break;
    }
    case FLASH_READ_STATUS_REG:
    {
      LOG_INFO("H\r");
      if(event == ev_LETIMER0_COMP1)
      {
        USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;
        trigger_CE_high_to_low_transition();

        uint8_t tx_sequence[TX_BUFFER_SIZE] = {FLASH_READ_STATUS_REG_CMD,0x00,0x00,0x00};
        USART_SpiTransfer(USART1, tx_sequence[0]);

        //Read start
        uint8_t rx_sequence[RX_BUFFER_SIZE]; memset(rx_sequence,0,sizeof(rx_sequence));
        USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;
        for (uint32_t i=0; i<1; i++)
        {
            //rx_sequence[i] = USART_SpiTransfer(USART1, 0xFF);
           // LOG_INFO("rx_sequence[%d] = %x\r",i,rx_sequence[i]);
            LOG_INFO(" = %x\r",USART_SpiTransfer(USART1, 0xFF));
        }

        trigger_CE_low_to_high_transition();

        ret_status = timerWaitUs_irq(SANITY_DELAY);
        if(ret_status == ERROR)
        {
           LOG_ERROR("The value is more than the routine can provide\r");
        }
        else
        {
           LOG_INFO("H\r");
           nextState = FLASH_WR_ENABLE;
        }
      }
      break;
    }
    case FLASH_SETMODE:
    {
#if DEBUG_1
      LOG_INFO("FLASH_SETMODE\r");
#endif
      if(event == ev_LETIMER0_COMP1)
      {
#if DEBUG_1
        LOG_INFO("FLASH_ADD_VERIFN\r");
#endif
        //flash_spi_init();
        trigger_CE_high_to_low_transition();
        uint8_t tx_sequence[TX_BUFFER_SIZE] = {0x00, 0x00, 0x80, 0x55, 0x55};
        update_write_params(0x02,5,tx_sequence,SEND_AS_DATA);
        start_ldma_transfer(TX_DMA_CHANNEL);
      }
      else if(event == ev_SPI_TX)
      {
          trigger_CE_low_to_high_transition();
          ret_status = timerWaitUs_irq(SANITY_DELAY);
          if(ret_status == ERROR)
          {
             LOG_ERROR("The value is more than the routine can provide\r");
          }
          else
          {
             nextState = FLASH_SETMODE_2;
          }
      }
      else
      {

      }
      break;
    }
    case FLASH_SETMODE_2:
    {
      if(event == ev_LETIMER0_COMP1)
      {
        #if DEBUG_1
        LOG_INFO("FLASH_ADD_VERIFN\r");
        #endif
        //flash_spi_init();
        trigger_CE_high_to_low_transition();
        uint8_t tx_sequence[TX_BUFFER_SIZE] = {0x00, 0x00, 0x80, 0xFF};
        update_write_params(0x03,4,tx_sequence,SEND_AS_DATA);

        start_ldma_transfer(TX_DMA_CHANNEL);


      }
      if(event == ev_SPI_TX)
      {        update_read_params(1);
        start_ldma_transfer(RX_DMA_CHANNEL);
          //uint8_t tx2_sequence[TX_BUFFER_SIZE] = {0x00, 0x00, 0x80, 0xFF};
          //update_write_params(0x03,4,tx2_sequence,SEND_AS_DATA);
          //start_ldma_transfer(TX_DMA_CHANNEL);


        LOG_INFO("ev_SPI_TX\r");
      }
      if(event == ev_SPI_RX)
      {
        //LOG_ERROR("Determine the cause for this\r");
        trigger_CE_low_to_high_transition();

        for(int i = 0; i<RX_BUFFER_SIZE;i++)
        {
          //LOG_INFO("TxBuffer[%d] = %x\r",i,TxBuffer[i]);
          LOG_INFO("RxBuffer[%d] = %x\r",i,RxBuffer[i]);
        }
        nextState = FLASH_ADD_VERIFN;
      }
      break;
    }
    default:
      break;
  }
  return return_state;
}

void flash_spi_usart_configuration()
{
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_USART1, true);


  GPIO_PinModeSet(SL_SPIDRV_FLASH_MEM_CLK_PORT, SL_SPIDRV_FLASH_MEM_CLK_PIN, gpioModePushPull, false); // US1_CLK is push pull

  GPIO_DriveStrengthSet(SL_SPIDRV_FLASH_MEM_CS_PORT, gpioDriveStrengthStrongAlternateStrong);
  GPIO_PinModeSet(SL_SPIDRV_FLASH_MEM_CS_PORT, SL_SPIDRV_FLASH_MEM_CS_PIN, gpioModePushPull, true); // US1_CS is push pull
  GPIO_PinOutSet(SL_SPIDRV_FLASH_MEM_CS_PORT,SL_SPIDRV_FLASH_MEM_CS_PIN);

  GPIO_PinModeSet(SL_SPIDRV_FLASH_MEM_TX_PORT, SL_SPIDRV_FLASH_MEM_TX_PIN, gpioModePushPull, true); // US1_TX (MOSI) is push pull
  GPIO_PinModeSet(SL_SPIDRV_FLASH_MEM_RX_PORT, SL_SPIDRV_FLASH_MEM_RX_PIN, gpioModeInput, true);    // US1_RX (MISO) is input

  USART_Reset(USART1);
  // Start with default config, then modify as necessary
  USART_InitSync_TypeDef config = USART_INITSYNC_DEFAULT;
  config.master       = true;            // master mode
  config.baudrate     = FLASH_OPERATING_FREQUENCY;         // CLK freq is 1 MHz
  config.autoCsEnable = false;            // CS pin controlled by hardware, not firmware
  config.clockMode    = usartClockMode0; // clock idle low, sample on rising/first edge
  config.msbf         = true;            // send MSB first
  config.enable       = usartDisable;    // making sure USART isn't enabled until we set it up
  //LOG_INFO("Hy - here before spi_init\r");
  USART_BaudrateSyncSet(USART1, 0,FLASH_OPERATING_FREQUENCY );
  USART_InitSync(USART1, &config);

#if 0
  USART1->ROUTELOC0 = USART_ROUTELOC0_CLKLOC_LOC31  |
                      USART_ROUTELOC0_CSLOC_LOC0   |
                      USART_ROUTELOC0_TXLOC_LOC0   |
                      USART_ROUTELOC0_RXLOC_LOC1;
#endif
  USART1->ROUTELOC0 = (USART1->ROUTELOC0 & ~(_USART_ROUTELOC0_TXLOC_MASK
                        | _USART_ROUTELOC0_RXLOC_MASK | _USART_ROUTELOC0_CLKLOC_MASK))
                        | (SL_SPIDRV_FLASH_MEM_TX_LOC << _USART_ROUTELOC0_TXLOC_SHIFT)
                        | (SL_SPIDRV_FLASH_MEM_RX_LOC << _USART_ROUTELOC0_RXLOC_SHIFT)
                        | (SL_SPIDRV_FLASH_MEM_CLK_LOC << _USART_ROUTELOC0_CLKLOC_SHIFT);
  // Enable USART pins
  USART1->ROUTEPEN = USART_ROUTEPEN_CLKPEN  | USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN;
//USART_ROUTEPEN_CSPEN
  // Enable USART1
  USART_Enable(USART1, usartEnable);

}

void trigger_CE_high_to_low_transition()
{
  GPIO_PinOutClear(SL_SPIDRV_FLASH_MEM_CS_PORT,SL_SPIDRV_FLASH_MEM_CS_PIN);
}

void trigger_CE_low_to_high_transition()
{
  GPIO_PinOutSet(SL_SPIDRV_FLASH_MEM_CS_PORT,SL_SPIDRV_FLASH_MEM_CS_PIN);
}

static int accel_write(uint8_t start_register, uint8_t *tx, uint32_t nbytes)
{
  if (nbytes <= 0 || tx == NULL) return -1;
  USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;
  uint8_t cmd;
  if (nbytes > 1)
  {
    cmd = MULTI_WRITE_CMD(start_register);
  }
  else if (nbytes == 1)
  {
    cmd = SINGLE_WRITE_CMD(start_register);
  }
  accel_cs_low();
  USART_SpiTransfer(USART1, cmd);
  for (uint32_t i=0; i<nbytes; i++)
  {
    USART_SpiTransfer(USART1, tx[i]);
  }
  accel_cs_high();
  return 0;
}

static int accel_read(uint8_t start_register, uint8_t *rx, uint32_t nbytes)
{
  if (nbytes <= 0 || rx == NULL) return -1;
  USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;
  uint8_t cmd;
  if (nbytes > 1)
  {
    cmd = MULTI_READ_CMD(start_register);
  }
  else if (nbytes == 1)
  {
    cmd = SINGLE_READ_CMD(start_register);
  }
  accel_cs_low();
  USART_SpiTransfer(USART1, cmd);
  for (uint32_t i=0; i<nbytes; i++)
  {
    rx[i] = USART_SpiTransfer(USART1, 0xFF);
  }
  accel_cs_high();
  return 0;
}
