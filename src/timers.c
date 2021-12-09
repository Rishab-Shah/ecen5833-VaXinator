/*
 * timer.c - Timer functions
 *
 *  Created on: Sep 5, 2021
 *      Author: rishab
 */

#include "timers.h"

void LETIMER0_Init(void) {
    LETIMER_Init_TypeDef timer_settings;
    uint32_t timer_period_count;

    timer_period_count = LETIMER_PERIOD_MS * LFACLK_FREQ_HZ / LFACLK_PRESCALER_DIV_RATIO / MSEC_TO_USEC;
    timer_settings = (LETIMER_Init_TypeDef){
        false, false, true, false, 0, 0, letimerUFOANone, letimerUFOANone, letimerRepeatFree, 0
    };

    LETIMER_Init(LETIMER0, &timer_settings);
    LETIMER_CompareSet(LETIMER0, 0, timer_period_count);

    LETIMER_IntEnable(LETIMER0, LETIMER_IEN_UF);

    NVIC_ClearPendingIRQ(LETIMER0_IRQn);
    NVIC_EnableIRQ(LETIMER0_IRQn);
}

void LETIMER0_Start(void) {
    LETIMER_Enable(LETIMER0, true);
}

int timerWaitUs_irq(uint32_t us_wait) {
    uint32_t ref_tick_value = 0;
    uint32_t req_wait_time = 0;
    uint32_t cal_wait_time = 0;

    //Calculate the value - us_wait into equivalent no of ticks wait time
    us_wait = (uint32_t)(us_wait / 1000);
    req_wait_time = (((us_wait * ACTUAL_CLK_FREQ) / 1000));

    if(req_wait_time > VALUE_TO_LOAD_FOR_PERIOD)
    {
        return -1;
    }

    //Take reference value of clock (Latest)
    ref_tick_value = LETIMER_CounterGet(LETIMER0);

    if(ref_tick_value > req_wait_time)
    {
        cal_wait_time = ref_tick_value - req_wait_time;
    }
    else
    {
        cal_wait_time = VALUE_TO_LOAD_FOR_PERIOD + ref_tick_value - req_wait_time;
    }

    //clear any pending
    //Very important to add, otherwise first interupt is messed up
    LETIMER_IntClear(LETIMER0,LETIMER_IFC_COMP1);
    //Load the counter value
    LETIMER_CompareSet(LETIMER0,1,cal_wait_time);
    //Enable the Interrupt for COMP1
    LETIMER_IntEnable(LETIMER0,LETIMER_IEN_COMP1);

    return 0;
}
