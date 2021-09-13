/*
 * irq.c - Interrupt service routines
 *
 *  Created on: Sep 5, 2021
 *      Author: vishn
 */

#include "irq.h"

void LETIMER0_IRQHandler(void) {
    uint32_t interrupt_flags = LETIMER0->IF;
    LETIMER0->IFC = 0x1F;

    if (interrupt_flags & _LETIMER_IF_COMP1_MASK) {
        scheduler_set_event_LETIMER0_COMP1();
    }
    else if (interrupt_flags & _LETIMER_IF_UF_MASK) {
        scheduler_set_event_LETIMER0_UF();
    }
    else {

    }

}
