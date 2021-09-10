/*
 * timers.h
 *
 *  Created on: Sep 5, 2021
 *      Author: vishn
 */

#ifndef SRC_TIMERS_H_
#define SRC_TIMERS_H_

#include "em_letimer.h"
#include "app.h"

#define LED_PERIOD_MS (2250)
#define LED_ON_MS     (175)

void init_letimer0(void);
void start_letimer0(void);

#endif /* SRC_TIMERS_H_ */
