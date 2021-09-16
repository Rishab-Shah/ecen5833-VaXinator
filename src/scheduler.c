/*
 * scheduler.c - Scheduler functions
 *
 *  Created on: Sep 12, 2021
 *      Author: vishn
 */

#include "scheduler.h"

static event_t event = ev_NONE;


void scheduler_set_event_LETIMER0_UF(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    event = ev_LETIMER0_UF;
    CORE_EXIT_CRITICAL();
}

event_t get_next_event(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    event_t current_event = event;
    event = ev_NONE;
    CORE_EXIT_CRITICAL();
    return current_event;
}
