/*
 * scheduler.c - Scheduler functions
 *
 *  Created on: Sep 12, 2021
 *      Author: vishn
 */

#include "scheduler.h"

static event_t event_q[EVENT_QUEUE_SIZE];
static uint8_t rd_ptr = 0;
static uint8_t wr_ptr = 0;
static uint8_t q_len = 0;



void scheduler_set_event_LETIMER0_UF(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    if (q_len == EVENT_QUEUE_SIZE - 1) {
        CORE_EXIT_CRITICAL();
        return;
    }
    q_len++;
    event_q[(wr_ptr++) & EVENT_QUEUE_SIZE_MASK] = ev_LETIMER0_UF;
    CORE_EXIT_CRITICAL();
}

event_t get_next_event(void) {
    event_t current_event;
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    if (q_len == 0) {
        current_event = ev_NONE;
    }
    else {
        current_event = event_q[(rd_ptr++) & EVENT_QUEUE_SIZE_MASK];
        q_len--;
    }
    CORE_EXIT_CRITICAL();
    return current_event;
}
