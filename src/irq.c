/*
 * irq.c - Interrupt service routines
 *
 *  Created on: Sep 5, 2021
 *      Author: vishnu
 */

#include "irq.h"
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"


uint32_t g_systickCounter = 0;

extern LDMA_Descriptor_t ldmaTXDescriptor;
extern LDMA_TransferCfg_t ldmaTXConfig;

extern LDMA_Descriptor_t ldmaRXDescriptor;
extern LDMA_TransferCfg_t ldmaRXConfig;

void LETIMER0_IRQHandler(void) {
    uint32_t flags;

    //determine pending interrupts
    flags = LETIMER_IntGet(LETIMER0);

    //clear pending interrupts
    LETIMER_IntClear(LETIMER0,flags);

    if (flags & _LETIMER_IF_UF_MASK) {
        Scheduler_SetEvent_LETIMER0_UF();
    }
    else if (flags & _LETIMER_IF_COMP1_MASK) {
        Scheduler_SetEvent_LETIMER0_COMP1();
    }
}


void I2C0_IRQHandler(void) {
    I2C_TransferReturn_TypeDef transfer_status;

    transfer_status = I2C_Transfer(I2C0);

    if (transfer_status == i2cTransferDone) {
        Scheduler_SetEvent_I2C0_TRANSFER_DONE();
        NVIC_DisableIRQ(I2C0_IRQn);
    }
    else if (transfer_status < i2cTransferDone) {
        LOG_ERROR("I2C Error Code %d\r", transfer_status);
    }
}

extern uint8_t rx_dma_channel;
extern uint8_t tx_dma_channel;

uint8_t counter_rx = 0;
uint8_t counter_tx = 0;

#if 0
void LDMA_IRQHandler()
{
  uint32_t flags = LDMA_IntGet();

  if(flags & (1 << rx_dma_channel))
  {
      //LDMA_StartTransfer(RX_DMA_CHANNEL, &ldmaRXConfig, &ldmaRXDescriptor);
      Scheduler_SetEvent_SPI_RX();
      counter_rx++;
      if(counter_rx == 1)
      {
          trigger_CE_low_to_high_transition();

          LDMA_StopTransfer(tx_dma_channel);
          LDMA_StopTransfer(rx_dma_channel);
          counter_rx = 0;
      }

      LDMA_IntClear(1 << rx_dma_channel);
  }
  else
  {
      counter_tx++;
      Scheduler_SetEvent_SPI_TX();
#if 1
      if(counter_tx == 1)
      {
          //LDMA_StartTransfer(tx_dma_channel, &ldmaTXConfig, &ldmaTXDescriptor);
          //LDMA_StartTransfer(rx_dma_channel, &ldmaRXConfig, &ldmaRXDescriptor);
          counter_tx = 0;
         // trigger_CE_low_to_high_transition();
      }
      else
      {

      }
#endif
      LDMA_IntClear(1 << tx_dma_channel);
  }
}
#endif

#if 1
void LDMA_IRQHandler()
{
  uint32_t flags = LDMA_IntGet();

  if(flags & (1 << rx_dma_channel))
  {
      LDMA_IntClear(1 << rx_dma_channel);
      LDMA_StopTransfer(rx_dma_channel);
      Scheduler_SetEvent_SPI_RX();
  }
  else
  {
      LDMA_IntClear(1 << tx_dma_channel);
      LDMA_StopTransfer(tx_dma_channel);
      Scheduler_SetEvent_SPI_TX();
  }
}
#endif





#if 1
void GPIO_EVEN_IRQHandler(void) {
    uint32_t interrupt_flags = GPIO_IntGet();
    GPIO->IFC = 0xFFFF;

    if (interrupt_flags & (0x01 << PB0_pin)) {
        if(GPIO_PinInGet(PB0_port, PB0_pin) == 0) {
            Scheduler_SetEvent_PB0_RELEASED();
        }
    }
}
#endif

#if 0
void GPIO_ODD_IRQHandler(void) {
    uint32_t interrupt_flags = GPIO_IntGet();
    GPIO->IFC = 0xFFFF;
    static uint8_t pressed = 1; // Button press is first time we enter ISR

    if (interrupt_flags & (0x01 << PB1_pin)) {
        if (!pressed) {
            Scheduler_SetEvent_PB1_RELEASED();
        }
        else {
            Scheduler_SetEvent_PB1_PRESSED();
        }

        pressed ^= 1;
    }
}
#endif

uint32_t letimerMilliseconds()
{
    uint32_t value_return = 0;

    value_return = (((g_systickCounter)*(LETIMER_PERIOD_MS))
        + ((VALUE_TO_LOAD_FOR_PERIOD-LETIMER_CounterGet(LETIMER0))/CLK_FREQ_TO_PERIOD_MAPPING) );

    return (value_return);
}


void SysTick_IncrementCounter()
{
    g_systickCounter++;
}
