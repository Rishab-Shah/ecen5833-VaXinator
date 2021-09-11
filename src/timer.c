/*
 * timer.c - Timer functions
 *
 *  Created on: Sep 5, 2021
 *      Author: vishn
 */

#include "timers.h"

void init_letimer0(void) {
  LETIMER_Init_TypeDef timer_settings;
  uint32_t timer_period_count, timer_on_count;

  timer_period_count = LED_PERIOD_MS * CLK_FREQ / CLK_DIV / MS_PER_SEC;
  timer_on_count = LED_ON_MS * CLK_FREQ / CLK_DIV / MS_PER_SEC;
  timer_settings = (LETIMER_Init_TypeDef){
      false, false, true, false, 0, 0, letimerUFOANone, letimerUFOANone, letimerRepeatFree, 0
  };

  LETIMER_Init(LETIMER0, &timer_settings);
  LETIMER_CompareSet(LETIMER0, 0, timer_period_count); //199 should be swapped with period of LED
  LETIMER_CompareSet(LETIMER0, 1, timer_on_count); //23 should be swapped with cnt value to toggle LED

  LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP1 | LETIMER_IEN_UF);

  NVIC_ClearPendingIRQ(LETIMER0_IRQn);
  NVIC_EnableIRQ(LETIMER0_IRQn);
}


void start_letimer0(void) {
  LETIMER_Enable(LETIMER0, true);
}
