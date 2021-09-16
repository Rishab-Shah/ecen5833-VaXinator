/*
 * timers.h - Timer functions
 *
 *  Created on: Sep 5, 2021
 *      Author: vishn
 */

#ifndef SRC_TIMERS_H_
#define SRC_TIMERS_H_

#include "em_letimer.h"
#include "app.h"

#define LETIMER_PERIOD_MS (3000)
#define LETIMER_PERIOD_US (3000000)
#define LED_ON_MS     (175)


/*
 * Initializes the LETIMER0
 *
 * @param None
 *
 * @return None
 */
void letimer0_init(void);


/*
 * Starts the LETIMER0
 *
 * @param None
 *
 * @return None
 */
void letimer0_start(void);


/*
 * Waits for given time in microseconds
 *
 * @param us_wait - Time to wait in microseconds
 *
 * @return None
 */
void timerWaitUs(uint32_t us_wait);

#endif /* SRC_TIMERS_H_ */
