/*
 * irq.h - Interrupt service routines
 *
 *  Created on: Sep 5, 2021
 *      Author: vishnu
 */

#ifndef SRC_IRQ_H_
#define SRC_IRQ_H_

#include "em_letimer.h"
#include "em_i2c.h"
#include "em_gpio.h"
#include "timers.h"
#include "gpio.h"
#include "scheduler.h"
#include "spi.h"
/*
 * LETIMER0 ISR
 *
 * @param None
 *
 * @return None
 */
void LETIMER0_IRQHandler(void);


/*
 * I2C0 ISR
 *
 * @param None
 *
 * @return None
 */
void I2C0_IRQHandler(void);


/*
 * PB0 ISR
 *
 * @param None
 *
 * @return None
 */
void GPIO_EVEN_IRQHandler(void);


/*
 * Gets number of milliseconds since execution began
 *
 * @param None
 *
 * @return Milliseconds since execution began
 */
uint32_t letimerMilliseconds();


/*
 * Increments SysTick Counter
 *
 * @param None
 *
 * @return None
 */
void SysTick_IncrementCounter(void);

#endif /* SRC_IRQ_H_ */
