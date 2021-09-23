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
void LETIMER0_Init(void);


/*
 * Starts the LETIMER0
 *
 * @param None
 *
 * @return None
 */
void LETIMER0_Start(void);


/*
 * Increments LETIMER0 ticker
 *
 * @param None
 *
 * @return None
 */
void LETIMER0_IncrementTicker(void);


/*
 * Waits for given time in microseconds - polling based
 *
 * @param us_wait - Time to wait in microseconds
 *
 * @return None
 */
void timerWaitUs_polled(uint32_t us_wait);


/*
 * Waits for given time in microseconds - interrupt based
 *
 * @param us_wait - Time to wait in microseconds
 *
 * @return None
 */
void timerWaitUs_irq(uint32_t us_wait);


/*
 * Gets number of milliseconds since execution began (resolution of 3000 ms)
 *
 * @param None
 *
 * @return Milliseconds since execution began (resolution of 3000 ms)
 */
uint64_t letimerMilliseconds(void);

#endif /* SRC_TIMERS_H_ */
