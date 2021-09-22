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



/*
 * Enqueues event onto event queue
 *
 * @param event - Event to be enqueued
 *
 * @return None
 */
static void EventQ_EnqueueEvent(event_t event);


/*
 * Dequeues event from event queue
 *
 * @param None
 *
 * @return Next event
 */
static event_t EventQ_DequeueEvent(void);


static void EventQ_EnqueueEvent(event_t event) {
    if (q_len == EVENT_QUEUE_SIZE) {
        return;
    }
    event_q[(wr_ptr++) & EVENT_QUEUE_SIZE_MASK] = event;
    q_len++;
}


static event_t EventQ_DequeueEvent(void) {
    event_t current_event;
    if (q_len == 0) {
        current_event = ev_NONE;
    }
    else {
        current_event = event_q[(rd_ptr++) & EVENT_QUEUE_SIZE_MASK];
        q_len--;
    }
    return current_event;
}


void Scheduler_SetEvent_LETIMER0_UF(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    EventQ_EnqueueEvent(ev_LETIMER0_UF);
    CORE_EXIT_CRITICAL();
}


void Scheduler_SetEvent_LETIMER0_COMP1(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    EventQ_EnqueueEvent(ev_LETIMER0_COMP1);
    CORE_EXIT_CRITICAL();
}


void Scheduler_SetEvent_I2C0_TRANSFER_DONE(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    EventQ_EnqueueEvent(ev_I2C0_TRANSFER_DONE);
    CORE_EXIT_CRITICAL();
}


event_t Scheduler_GetNextEvent(void) {
    event_t current_event;
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    current_event = EventQ_DequeueEvent();
    CORE_EXIT_CRITICAL();
    return current_event;
}
