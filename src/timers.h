/*
 * timers.h - Timer functions
 *
 *  Created on: Sep 5, 2021
 *      Author: rishab
 */

#ifndef SRC_TIMERS_H_
#define SRC_TIMERS_H_

#include "em_letimer.h"
#include "app.h"

#define ACTUAL_CLK_FREQ             ((LFACLK_FREQ_HZ)/(LFACLK_PRESCALER_DIV_RATIO))
#define VALUE_TO_LOAD_FOR_PERIOD    (((LETIMER_PERIOD_MS)*(ACTUAL_CLK_FREQ))/(1000))
#define CLK_FREQ_TO_PERIOD_MAPPING  (VALUE_TO_LOAD_FOR_PERIOD/LETIMER_PERIOD_MS)

#define LETIMER_PERIOD_MS           (1500)
#define LED_ON_MS                   (175)


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
 * @return 1 on success, 0 on failure
 */
int timerWaitUs_irq(uint32_t us_wait);


#endif /* SRC_TIMERS_H_ */
