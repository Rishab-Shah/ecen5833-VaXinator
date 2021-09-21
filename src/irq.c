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

    if (interrupt_flags & _LETIMER_IF_UF_MASK) {
        scheduler_set_event_LETIMER0_UF();
    }
    else if (interrupt_flags & _LETIMER_IF_COMP1_MASK) {
        scheduler_set_event_LETIMER0_COMP1();
    }
}


void I2C0_IRQHandler(void) {
    I2C_TransferReturn_TypeDef transfer_status;

    transfer_status = I2C_Transfer(I2C0);

    if (transfer_status == i2cTransferDone) {
        set_scheduler_event_I2C0_TRANSFER_DONE();
    }
}
