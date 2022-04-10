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
#define FLASH_OPERATING_FREQUENCY   ((20)*(MHZ_1))

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

  init_ldma_params();
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
  TxBuffer[0] = command;

  uint16_t iterator = 0;
  if(data_or_address == SEND_AS_DATA)
  {
    for(iterator = 1; iterator < (write_count+1); iterator++)
    {
        TxBuffer[iterator] = tempbuffer[iterator-1];
    }
  }
  // LDMA descriptor and config for transferring TxBuffer
  // Source: TxBuffer, Destination: USART1->TXDATA, Bytes to send: 10
  ldmaTXDescriptor = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(TxBuffer, &(USART1->TXDATA),(write_count+1));
  // Setting the transfer to trigger once there is room in the USART1_TXDATA buffer
  ldmaTXConfig = (LDMA_TransferCfg_t)LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART1_TXBL);
}

void update_read_params(uint16_t read_count)//int8_t TxBuffer,uint16_t write_count
{
  memset(RxBuffer,0,sizeof(RxBuffer));
  // LDMA descriptor and config for receiving data from the slave
  // Source: USART1->RXDATA, Destination: RxBuffer, Bytes to receive: 10
  ldmaRXDescriptor = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_SINGLE_P2M_BYTE(&(USART1->RXDATA),RxBuffer,read_count);
  // Setting the transfer to trigger whenever data comes into USART1_RXDATAV
  ldmaRXConfig = (LDMA_TransferCfg_t)LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART1_RXDATAV);
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
        trigger_CE_high_to_low_transition();
        uint8_t tx_sequence[TX_BUFFER_SIZE] = {0};
        update_write_params(0x06,0,tx_sequence,SEND_AS_DATA);
        start_ldma_transfer(TX_DMA_CHANNEL);
        //update_read_params(1);
        //LDMA_StartTransfer(rx_dma_channel, &ldmaRXConfig, &ldmaRXDescriptor);
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
             nextState = FLASH_SETMODE;
              //nextState = FLASH_READ_STATUS_REGISTER;
          }
      }
      else
      {

      }
      break;
#if 0
        trigger_CE_high_to_low_transition();
        update_write_params(4);
        update_read_params(4);
        start_ldma_transfer(TX_DMA_CHANNEL);
#endif

        //nextState = FLASH_SETMODE;
#if 0
        // start a timer for 1 msec
        ret_status = timerWaitUs_irq(SANITY_DELAY);
        if(ret_status == ERROR)
        {
           LOG_ERROR("The value is more than the routine can provide\r");
        }
        else
        {
           nextState = FLASH_SETMODE;
        }
#endif
    }
    case FLASH_READ_STATUS_REGISTER:
    {
      if(event == ev_LETIMER0_COMP1)
      {
          trigger_CE_high_to_low_transition();
          uint8_t tx_sequence[TX_BUFFER_SIZE] = {0x00, 0x00, 0x80, 0x55, 0x55};
          update_write_params(0x05,0,tx_sequence,SEND_AS_DATA);
          start_ldma_transfer(TX_DMA_CHANNEL);
      }
      else if(event == ev_SPI_TX)
      {
#if DEBUG_1
        LOG_INFO("ev_SPI_TX\r");
#endif
        update_read_params(1);
        start_ldma_transfer(RX_DMA_CHANNEL);

#if 0
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
#endif
      }
      else if(event == ev_SPI_RX)
      {
#if DEBUG_1
        LOG_INFO("ev_SPI_RX\r");
#endif
        //LOG_ERROR("Determine the cause for this\r");
        trigger_CE_low_to_high_transition();

        for(int i = 0; i<RX_BUFFER_SIZE;i++)
        {
          //LOG_INFO("TxBuffer[%d] = %x\r",i,TxBuffer[i]);
          LOG_INFO("RxBuffer[%d] = %x\r",i,RxBuffer[i]);
        }
        nextState = FLASH_ADD_VERIFN;
      }
      else
      {

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
#if 0
      if(event == ev_SPI_TX)
      {
          //LDMA_StopTransfer(tx_dma_channel);
          start_ldma_transfer(RX_DMA_CHANNEL);
          nextState = FLASH_SETMODE_2;
      }
      else if(event == ev_SPI_RX)
      {
          //
          LDMA_StopTransfer(rx_dma_channel);
          trigger_CE_low_to_high_transition();

          for(int i = 0; i<RX_BUFFER_SIZE;i++)
          {
            //LOG_INFO("TxBuffer[%d] = %x\r",i,TxBuffer[i]);
            LOG_INFO("RxBuffer[%d] = %x\r",i,RxBuffer[i]);
          }
          nextState = FLASH_ADD_VERIFN;
      }
      else
      {LOG_ERROR("Determine the cause for this\r");
          /* Do nothing */
      }
#endif
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




#if 0
    case BNO055_SETMODE_DELAY_1:
    {
      if(event == ev_I2C0_TRANSFER_DONE)
      {
        ret_status = timerWaitUs_irq(SETMODE_DELAY);
        if(ret_status == ERROR)
        {
          LOG_ERROR("The value is more than the routine can provide\r");
        }
        else
        {
          nextState = BNO055_RESET;
        }
      }
      break;
    }
    case BNO055_RESET:
    {
      if(event == ev_LETIMER0_COMP1)
      {
          LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
          LOG_INFO("BNO055_RESET\r");
#endif
          BNO055_write(BNO055_SYS_TRIGGER_ADDR,0x20);
          nextState = BNO055_RESET_DELAY_2;
      }
      break;
    }
    case BNO055_RESET_DELAY_2:
    {
      if(event == ev_I2C0_TRANSFER_DONE)
      {
         ret_status = timerWaitUs_irq(POST_RESET_STARTUP_DELAY);
         if(ret_status == ERROR)
         {
             LOG_ERROR("The value is more than the routine can provide\r");
         }
         else
         {
             nextState = BNO055_READ_POST_RESET;
         }
       }
      break;
    }
    case BNO055_READ_POST_RESET:
    {
      if(event == ev_LETIMER0_COMP1)
      {
        LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
        LOG_INFO("BNO055_READ_POST_RESET\r");
#endif
        address_verification = false;
        address_verification = BNO055_VerifyIdentity(&bno055_rd_buff[0]);
        if(address_verification == true)
        {
            ret_status = timerWaitUs_irq(STD_DELAY);
            if(ret_status == ERROR)
            {
                LOG_ERROR("The value is more than the routine can provide\r");
            }
            else
            {
                nextState = BNO055_READ_POST_RESET_DELAY_3;
            }
        }
        else
        {
           ret_status = timerWaitUs_irq(STD_DELAY);
           if(ret_status == ERROR)
           {
               LOG_ERROR("The value is more than the routine can provide\r");
           }
           else
           {
               nextState = BNO055_READ_POST_RESET;
           }
        }
      }
      break;
    }
    case BNO055_READ_POST_RESET_DELAY_3:
    {
       ret_status = timerWaitUs_irq(NORMAL_MODE_DELAY);
       if(ret_status == ERROR)
       {
           LOG_ERROR("The value is more than the routine can provide\r");
       }
       else
       {
           nextState = BNO055_NORMAL_MODE_SET;
       }

      break;
    }
    case BNO055_NORMAL_MODE_SET:
    {
      if(event == ev_LETIMER0_COMP1)
      {
          LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
          LOG_INFO("BNO055_NORMAL_MODE_SET\r");
#endif
          BNO055_write(BNO055_PWR_MODE_ADDR,POWER_MODE_NORMAL);
          nextState = BNO055_NORMAL_MODE_SET_DELAY_4;
      }
      break;
    }
    case BNO055_NORMAL_MODE_SET_DELAY_4:
    {
      if(event == ev_I2C0_TRANSFER_DONE)
      {
          ret_status = timerWaitUs_irq(STD_DELAY);
          if(ret_status == -1)
          {
              LOG_ERROR("The value is more than the routine can provide \r");
          }
          else
          {
              nextState = BNO055_PAGE_ADDR;
          }
      }
      break;
    }
    case BNO055_PAGE_ADDR:
    {
      if(event == ev_LETIMER0_COMP1)
      {
          LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
          LOG_INFO("BNO055_PAGE_ADDR\r");
#endif
          BNO055_write(BNO055_PAGE_ID_ADDR,0x00);
          nextState = BNO055_PAGE_ADDR_DELAY_5;
      }
      break;
    }
    case BNO055_PAGE_ADDR_DELAY_5:
    {
      if(event == ev_I2C0_TRANSFER_DONE)
      {
          ret_status = timerWaitUs_irq(STD_DELAY);
          if(ret_status == -1)
          {
              LOG_ERROR("The value is more than the routine can provide \r");
          }
          else
          {
              nextState = BNO055_SYS_TRIGGER;
          }
      }
      break;
    }
    case BNO055_SYS_TRIGGER:
    {
      if(event == ev_LETIMER0_COMP1)
      {
          LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
          LOG_INFO("BNO055_SYS_TRIGGER\r");
#endif
          BNO055_write(BNO055_SYS_TRIGGER_ADDR,0x00);
          nextState = BNO055_SYS_TRIGGER_DELAY_6;
      }
      break;
    }
    case BNO055_SYS_TRIGGER_DELAY_6:
    {
      if(event == ev_I2C0_TRANSFER_DONE)
      {
          ret_status = timerWaitUs_irq(STD_DELAY);
          if(ret_status == -1)
          {
              LOG_ERROR("The value is more than the routine can provide \r");
          }
          else
          {
              nextState = BNO055_SET_REQ_MODE;
          }
      }
      break;
     }
    case BNO055_SET_REQ_MODE:
    {
      if(event == ev_LETIMER0_COMP1)
      {
          LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
          LOG_INFO("BNO055_SET_REQ_MODE\r");
#endif
          setMode(OPERATION_MODE_NDOF);
          nextState = BNO055_SET_REQ_MODE_DELAY_7;
      }
      break;
    }
    case BNO055_SET_REQ_MODE_DELAY_7:
    {
      if(event == ev_I2C0_TRANSFER_DONE)
      {
          ret_status = timerWaitUs_irq((1000)*(1000));
          if(ret_status == -1)
          {
              LOG_ERROR("The value is more than the routine can provide \r");
          }
          else
          {
              nextState = READ_XYZ_DATA;
          }
      }
      break;
     }
    case READ_XYZ_DATA:
    {
      if(event == ev_LETIMER0_COMP1)
      {
          LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
          LOG_INFO("READ_XYZ_DATA\r");
#endif
          memset(bno055_rd_buff,0,sizeof(bno055_rd_buff));
          bno055_wr_buff[0] = BNO055_EULER_H_LSB_ADDR;
          I2C0_WriteRead(BNO055_ADDRESS_A, &bno055_wr_buff[0], 1, &bno055_rd_buff[0], 6);
          nextState = READ_XYZ_DATA_DELAY;
      }

      break;
    }
    case READ_XYZ_DATA_DELAY:
    {
      if(event == ev_I2C0_TRANSFER_DONE)
      {
          //processing
          int16_t x = 0,y = 0, z= 0;
          x = ((int16_t)(bno055_rd_buff[0]) | (int16_t)(bno055_rd_buff[1] << 8));
          y = ((int16_t)(bno055_rd_buff[2]) | (int16_t)(bno055_rd_buff[3] << 8));
          z = ((int16_t)(bno055_rd_buff[4]) | (int16_t)(bno055_rd_buff[5] << 8));

          LOG_INFO("X= %d::Y=%d::Z=%d\r",x,y,z);
          displayPrintf(DISPLAY_ROW_X, "X = %d mg", x);
          displayPrintf(DISPLAY_ROW_Y, "Y = %d mg", y);
          displayPrintf(DISPLAY_ROW_Z, "Z = %d mg", z);

          ret_status = timerWaitUs_irq((500)*(1000));
          if(ret_status == -1)
          {
              LOG_ERROR("The value is more than the routine can provide \r");
          }
          else
          {
              nextState = READ_XYZ_DATA;
          }
      }
      break;
    }
#endif
    default:
      break;
  }
  return return_state;
}

void flash_spi_usart_configuration()
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_USART1, true);
  GPIO_PinModeSet(SL_SPIDRV_FLASH_MEM_CLK_PORT, SL_SPIDRV_FLASH_MEM_CLK_PIN, gpioModePushPull, false); // US1_CLK is push pull

  GPIO_DriveStrengthSet(SL_SPIDRV_FLASH_MEM_CS_PORT, gpioDriveStrengthStrongAlternateStrong);
  GPIO_PinModeSet(SL_SPIDRV_FLASH_MEM_CS_PORT, SL_SPIDRV_FLASH_MEM_CS_PIN, gpioModePushPull, true); // US1_CS is push pull
  GPIO_PinOutSet(SL_SPIDRV_FLASH_MEM_CS_PORT,SL_SPIDRV_FLASH_MEM_CS_PIN);

  GPIO_PinModeSet(SL_SPIDRV_FLASH_MEM_TX_PORT, SL_SPIDRV_FLASH_MEM_TX_PIN, gpioModePushPull, true); // US1_TX (MOSI) is push pull
  GPIO_PinModeSet(SL_SPIDRV_FLASH_MEM_RX_PORT, SL_SPIDRV_FLASH_MEM_RX_PIN, gpioModeInput, true);    // US1_RX (MISO) is input

  // Start with default config, then modify as necessary
  USART_InitSync_TypeDef config = USART_INITSYNC_DEFAULT;
  config.master       = true;            // master mode
  config.baudrate     = FLASH_OPERATING_FREQUENCY;         // CLK freq is 1 MHz
  config.autoCsEnable = true;            // CS pin controlled by hardware, not firmware
  config.clockMode    = usartClockMode0; // clock idle low, sample on rising/first edge
  config.msbf         = true;            // send MSB first
  config.enable       = usartDisable;    // making sure USART isn't enabled until we set it up
  //LOG_INFO("Hy - here before spi_init\r");
  USART_InitSync(USART1, &config);
#if 0
  // Set USART pin locations
  USART1->ROUTELOC0 = (USART_ROUTELOC0_CLKLOC_LOC11) | // US1_CLK       on location 11 = PC8 per datasheet section 6.4 = EXP Header pin 8
                      (USART_ROUTELOC0_CSLOC_LOC11)  | // US1_CS        on location 11 = PC9 per datasheet section 6.4 = EXP Header pin 10
                      (USART_ROUTELOC0_TXLOC_LOC11)  | // US1_TX (MOSI) on location 11 = PC6 per datasheet section 6.4 = EXP Header pin 4
                      (USART_ROUTELOC0_RXLOC_LOC11);   // US1_RX (MISO) on location 11 = PC7 per datasheet section 6.4 = EXP Header pin 6
#endif

  USART1->ROUTELOC0 = USART_ROUTELOC0_CLKLOC_LOC26  |
                      USART_ROUTELOC0_CSLOC_LOC28   |
                      USART_ROUTELOC0_TXLOC_LOC29   |
                      USART_ROUTELOC0_RXLOC_LOC29;


  // Enable USART pins
  USART1->ROUTEPEN = USART_ROUTEPEN_CLKPEN | USART_ROUTEPEN_CSPEN | USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN;

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
