/*
 * scheduler.h - Scheduler functions
 *
 *  Created on: Sep 12, 2021
 *      Author: vishn
 */

#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_

#include "em_core.h"
#include <stdlib.h>

typedef enum {
    ev_NONE,
    ev_LETIMER0_COMP1,
    ev_LETIMER0_UF
} event_t;


void scheduler_set_event_LETIMER0_COMP1(void);

void scheduler_set_event_LETIMER0_UF(void);

event_t get_next_event(void);

#endif /* SRC_SCHEDULER_H_ */
