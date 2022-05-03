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
#define RX_BUFFER_SIZE              (50)
#define SANITY_DELAY                ((0.5)*(MSEC_TO_USEC))

#define BYTE_0                      (0x000000ff)
#define BYTE_1                      (0x0000ff00)
#define BYTE_2                      (0x00ff0000)

/*******************************************************************************
 Global
*******************************************************************************/
uint8_t g_rx_sequence[RX_BUFFER_SIZE];
/*******************************************************************************
 Function Prototypes
*******************************************************************************/
uint8_t poll_read_status_reg();
/*******************************************************************************
 Function Definition
*******************************************************************************/
void flash_spi_init()
{
  // Configure GPIO mode
  flash_spi_usart_configuration();

  poll_read_status_reg();

  read_chip_address();

  enable_write_to_status_register(FLASH_BPL_NONE);
}

unsigned createMask(unsigned a, unsigned b)
{
   unsigned r = 0;
   for (unsigned i=a; i<=b; i++)
       r |= 1 << i;

   return r;
}


void byte_write(uint32_t complete_address, uint8_t data_to_be_written)
{
  //a incrmented adress from teh user will come in a multi-write scenario
  write_command(FLASH_WR_EN_CMD);

  //LOG_INFO("byte_write::complete_address = 0x%.8X\r",complete_address);

  //extract address from the array - bit operations - future
  uint8_t address1 = 0,address2 = 0,address3= 0;
  get_memory_address(complete_address,&address1,&address2,&address3);

  //write a byte to the memory
  write_byte_data(address1,address2,address3,data_to_be_written);

  #if 1
  this:
  if(1 == poll_read_status_reg())
  {
    goto this;
  };
  #endif
}

void write_byte_data(uint8_t address1, uint8_t address2,uint8_t address3, uint8_t data_to_be_written)
{
  USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;
  trigger_CE_high_to_low_transition();

  uint8_t tx_sequence[TX_BUFFER_SIZE] = {FLASH_BYTE_PROG_CMD,address3,address2,address1,data_to_be_written};
  USART_SpiTransfer(USART1, tx_sequence[0]);
  for (uint32_t i=1; i<5; i++)
  {
    USART_SpiTransfer(USART1,tx_sequence[i]);
  }

  trigger_CE_low_to_high_transition();
}

void block_erase(uint32_t complete_address, uint8_t region_to_protect)
{
  //any memory to be protected - update status register






  //
  write_command(FLASH_WR_EN_CMD);

  LOG_INFO("block_erase::complete_address = 0x%.8X\r",complete_address);
  //extract address from the array - bit operations - future
  uint8_t address1 = 0,address2 = 0,address3= 0;
  get_memory_address(complete_address,&address1,&address2,&address3);

  USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;
  trigger_CE_high_to_low_transition();

  uint8_t tx_sequence[TX_BUFFER_SIZE] = {FLASH_BLOCK_ERASE_CMD,address3,address2,address1};
  USART_SpiTransfer(USART1, tx_sequence[0]);
  for (uint32_t i=1; i<4; i++)
  {
    USART_SpiTransfer(USART1,tx_sequence[i]);
  }

  trigger_CE_low_to_high_transition();

#if 1
  this:
  if(1 == poll_read_status_reg())
  {
      goto this;
  };
#endif
}

uint8_t poll_read_status_reg()
{
  USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;
  trigger_CE_high_to_low_transition();

  //write command for read status register
  uint8_t tx_sequence[TX_BUFFER_SIZE] = {FLASH_READ_STATUS_REG_CMD};
  USART_SpiTransfer(USART1, tx_sequence[0]);

  //Read start
  USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;
  uint8_t rx_sequence[RX_BUFFER_SIZE]; memset(rx_sequence,0,sizeof(rx_sequence));
  rx_sequence[0] = USART_SpiTransfer(USART1, 0xFF);

  trigger_CE_low_to_high_transition();


  uint8_t busy_bit = 0;
  busy_bit = rx_sequence[0] & (1<<0);
  //LOG_INFO("rx_sequence[0] = %d\r",rx_sequence[0]);
  //LOG_INFO("busy_bit = %d\r",busy_bit);
  return busy_bit;
}

void write_command(uint8_t cmd)
{
  USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;
  trigger_CE_high_to_low_transition();

  uint8_t cmd_sequence[TX_BUFFER_SIZE] = {0};
  cmd_sequence[0] = cmd;
  USART_SpiTransfer(USART1, cmd_sequence[0]);

  trigger_CE_low_to_high_transition();
}

void enable_write_to_status_register(uint8_t blp_setup)
{
  write_command(FLASH_EN_WR_STATUS_REG_CMD);

  USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;
  trigger_CE_high_to_low_transition();

  uint8_t tx_sequence[TX_BUFFER_SIZE] = {FLASH_WR_STATUS_REG_CMD,blp_setup};
  USART_SpiTransfer(USART1, tx_sequence[0]);
  USART_SpiTransfer(USART1, tx_sequence[1]);

  trigger_CE_low_to_high_transition();
}


void read_chip_address()
{
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
}

void get_memory_address(uint32_t complete_address,uint8_t *add1,uint8_t *add2,uint8_t *add3)
{
  *add1 = complete_address & BYTE_0;

  unsigned r =0;
  r = createMask(8,15);
  unsigned address2 = r & complete_address;
  *add2 = address2 >> 8;

  r =0;
  r = createMask(16,23);
  unsigned address3 = r & complete_address;
  *add3 = address3 >> 16;
  //LOG_INFO("address1 = 0x%.8X - address2 = 0x%.8X -------address3 = 0x%.8X\r",*add1,*add2,*add3);

}
uint8_t byte_read(uint32_t complete_address, uint32_t no_of_bytes_requested, uint8_t *buff)
{
  uint32_t i=0;
#if 1
  if(no_of_bytes_requested > RX_BUFFER_SIZE)
  {
    LOG_ERROR("Bytes requested more than buffer size\r");
    return 0;
  }
#endif
  //LOG_INFO("byte_read::complete_address = 0x%.8X\r",complete_address);

  //extract address from the array - bit operations - future
  uint8_t address1 = 0,address2 = 0,address3= 0;
  get_memory_address(complete_address,&address1,&address2,&address3);

  USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;
  trigger_CE_high_to_low_transition();

  uint8_t tx_sequence[TX_BUFFER_SIZE] = {FLASH_READ_20MHZ_CMD,address3,address2,address1};
  USART_SpiTransfer(USART1, tx_sequence[0]);
  for (uint32_t i=1; i<4; i++)
  {
    USART_SpiTransfer(USART1,tx_sequence[i]);
  }

  //Read start
  memset(g_rx_sequence,0,sizeof(g_rx_sequence));
  USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;
  for (i=0; i<no_of_bytes_requested; i++)
  {
    g_rx_sequence[i] = USART_SpiTransfer(USART1, 0xFF);
    buff[i] = g_rx_sequence[i];
    LOG_INFO("INSIDE::g_rx_sequence[%d] = %x\r",i,g_rx_sequence[i]);
    if(buff[i] == '\r' || buff[i] == 0xFF)
      {
        buff[i] = '\0';
        i++;
        break;
      }
    //LOG_INFO("%x \r",USART_SpiTransfer(USART1, 0xFF));
  }
  buff[i] = '\0';
  trigger_CE_low_to_high_transition();
#if 1
  this:
  if(1 == poll_read_status_reg())
  {
      goto this;
  };
#endif
  return (uint8_t)i;
}

FLASH_state_t init_flash_setup(sl_bt_msg_t *evt)
{
  ble_ext_signal_event_t event = evt->data.evt_system_external_signal.extsignals;
  /* return state logic */
  FLASH_state_t return_state = FLASH_DEFAULT;
  /* current machine logic */
  FLASH_state_t currentState;
  static FLASH_state_t nextState = FLASH_ADD_VERIFN;

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
#if 0
        LOG_INFO("FLASH_ADD_VERIFN\r");
#endif
        poll_read_status_reg();

        block_erase(0x00,0x00);
        char check_data[100] = {0};
        const char* temp = "1 -23 45 Y\n";
        strncpy(check_data,temp,strlen(temp));
        int z = strlen(temp);

        for(uint32_t x = 0;x < z;x++)
        {
          byte_write(x,check_data[x]);
        }

        byte_read(0x00,20, check_data);

        for(uint32_t q = 0;q < z;q++)
        {
          LOG_INFO("OUT:g_rx_sequence[%d] = %c\r",q,g_rx_sequence[q]);
        }

        LOG_INFO("COMPLETED\r");
        poll_read_status_reg();

        ret_status = timerWaitUs_irq(SANITY_DELAY);
        if(ret_status == ERROR)
        {
           LOG_ERROR("The value is more than the routine can provide\r");
        }
        else
        {
           //nextState =  FLASH_READ_BYTE;
        }
      }
      break;
    }
    case FLASH_EN_WR_BP_PROG_STATUS_REG:
    {
      if(event == ev_LETIMER0_COMP1)
      {
#if 0
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
#endif
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
            //LOG_INFO(" = %x\r",USART_SpiTransfer(USART1, 0xFF));
        }

        trigger_CE_low_to_high_transition();

        ret_status = timerWaitUs_irq(SANITY_DELAY);
        if(ret_status == ERROR)
        {
           LOG_ERROR("The value is more than the routine can provide\r");
        }
        else
        {
           //LOG_INFO("H\r");
           nextState = FLASH_WR_ENABLE;
        }
      }
      break;
    }
    case FLASH_BLOCK_ERASE:
    {
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
           // LOG_INFO(" = %x\r",USART_SpiTransfer(USART1, 0xFF));
        }

        trigger_CE_low_to_high_transition();

        ret_status = timerWaitUs_irq(SANITY_DELAY);
        if(ret_status == ERROR)
        {
           LOG_ERROR("The value is more than the routine can provide\r");
        }
        else
        {
           //LOG_INFO("H\r");
           nextState = FLASH_WR_ENABLE;
        }
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

  USART_BaudrateSyncSet(USART1, 0,FLASH_OPERATING_FREQUENCY );
  USART_InitSync(USART1, &config);

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
