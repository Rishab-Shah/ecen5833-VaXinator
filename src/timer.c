/*
 * timer.c - Timer functions
 *
 *  Created on: Sep 5, 2021
 *      Author: vishn
 */

#include "timers.h"

void letimer0_init(void) {
    LETIMER_Init_TypeDef timer_settings;
    uint32_t timer_period_count, timer_on_count;

    timer_period_count = LETIMER_PERIOD_MS * CLK_FREQ / CLK_DIV / MS_PER_SEC;
    timer_on_count = LED_ON_MS * CLK_FREQ / CLK_DIV / MS_PER_SEC;
    timer_settings = (LETIMER_Init_TypeDef){
        false, false, true, false, 0, 0, letimerUFOANone, letimerUFOANone, letimerRepeatFree, 0
    };

    LETIMER_Init(LETIMER0, &timer_settings);
    LETIMER_CompareSet(LETIMER0, 0, timer_period_count);
    LETIMER_CompareSet(LETIMER0, 1, timer_on_count);

    LETIMER_IntEnable(LETIMER0, LETIMER_IEN_UF);

    NVIC_ClearPendingIRQ(LETIMER0_IRQn);
    NVIC_EnableIRQ(LETIMER0_IRQn);
}


void letimer0_start(void) {
    LETIMER_Enable(LETIMER0, true);
}


void timerWaitUs(uint32_t us_wait) {
    uint32_t wait_ticks, current_ticks;

    if (us_wait > 131071) { // max value * 32768 that can be stored in uint32_t
        us_wait = 131071;
    }

    wait_ticks = us_wait * CLK_FREQ / CLK_DIV / US_PER_SEC;

    current_ticks = LETIMER_CounterGet(LETIMER0);

    while(LETIMER_CounterGet(LETIMER0) > (current_ticks - wait_ticks));

    current_ticks = LETIMER_CounterGet(LETIMER0);
}
