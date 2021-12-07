/*
 * oscillators.h - Oscillator functions
 *
 *  Created on: Sep 5, 2021
 *      Author: vishnu
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
void Osc_InitLETIMER0(void);

/*
 * Initializes clocks for I2C0
 *
 * @param None
 *
 * @return None
 */
void Osc_InitI2C0(void);

#endif /* SRC_OSCILLATORS_H_ */
