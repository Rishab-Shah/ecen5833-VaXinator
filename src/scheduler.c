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
void enqueue_event(event_t event);


/*
 * Dequeues event from event queue
 *
 * @param None
 *
 * @return Next event
 */
event_t dequeue_event(void);


void enqueue_event(event_t event) {
    if (q_len == EVENT_QUEUE_SIZE) {
        return;
    }
    event_q[(wr_ptr++) & EVENT_QUEUE_SIZE_MASK] = event;
    q_len++;
}


event_t dequeue_event(void) {
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


void scheduler_set_event_LETIMER0_UF(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    enqueue_event(ev_LETIMER0_UF);
    CORE_EXIT_CRITICAL();
}


void scheduler_set_event_LETIMER0_COMP1(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    enqueue_event(ev_LETIMER0_COMP1);
    CORE_EXIT_CRITICAL();
}


void scheduler_set_event_I2C0_TRANSFER_DONE(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    enqueue_event(ev_I2C0_TRANSFER_DONE);
    CORE_EXIT_CRITICAL();
}


event_t get_next_event(void) {
    event_t current_event;
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    current_event = dequeue_event();
    CORE_EXIT_CRITICAL();
    return current_event;
}
