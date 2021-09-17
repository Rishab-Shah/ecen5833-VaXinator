/*
 * scheduler.h - Scheduler functions
 *
 *  Created on: Sep 12, 2021
 *      Author: vishn
 */

#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_

#include <stdlib.h>
#include "em_core.h"

#define EVENT_QUEUE_SIZE  (256)
#define EVENT_QUEUE_SIZE_MASK  (255)


/*
 * Event enum
 */
typedef enum {
    ev_NONE,
    ev_LETIMER0_UF
} event_t;


/*
 * Sets LETIMER0_COMP1 event
 *
 * @param None
 *
 * @return None
 */
void scheduler_set_event_LETIMER0_COMP1(void);


/*
 * Sets LETIMER0_UF event
 *
 * @param None
 *
 * @return None
 */
void scheduler_set_event_LETIMER0_UF(void);


/*
 * Gets next event from the scheduler
 *
 * @param None
 *
 * @return current event
 */
event_t get_next_event(void);

#endif /* SRC_SCHEDULER_H_ */
