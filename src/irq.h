/*
 * irq.h - Interrupt service routines
 *
 *  Created on: Sep 5, 2021
 *      Author: vishn
 */

#ifndef SRC_IRQ_H_
#define SRC_IRQ_H_

#include "em_letimer.h"
#include "gpio.h"

/*
 * LETIMER0 ISR
 *
 * @param None
 *
 * @return None
 */
void LETIMER0_IRQHandler(void);

#endif /* SRC_IRQ_H_ */
