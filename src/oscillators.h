/*
 * oscillators.h - Oscillator functions
 *
 *  Created on: Sep 5, 2021
 *      Author: vishn
 */

#ifndef SRC_OSCILLATORS_H_
#define SRC_OSCILLATORS_H_

#include "em_cmu.h"
#include "app.h"


/*
 * Initializes clocks for LETIMER0
 *
 * @param None
 *
 * @return None
 */
void letimer0_clock_init(void);


/*
 * Initializes clocks for I2C0
 *
 * @param None
 *
 * @return None
 */
void i2c0_clock_init(void);

#endif /* SRC_OSCILLATORS_H_ */
