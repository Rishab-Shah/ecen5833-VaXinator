/*
 * irq.c
 *
 *  Created on: Sep 5, 2021
 *      Author: vishn
 */

#include "irq.h"

void LETIMER0_IRQHandler(void) {
  uint32_t interrupt_flags = LETIMER0->IF;
  //uint16_t cnt;
  LETIMER0->IFC = 0x1F;

  if (interrupt_flags & _LETIMER_IF_COMP1_MASK) {
      gpioLed0SetOn();
    //turn on led
      //cnt = LETIMER0->CNT;
      //cnt--;
  }
  else if (interrupt_flags & _LETIMER_IF_UF_MASK) {
      gpioLed0SetOff();
    //turn off led
      //cnt = LETIMER0->CNT;
      //cnt--;
  }
}
