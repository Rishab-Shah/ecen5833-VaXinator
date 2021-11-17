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
        LETIMER0_IncrementTicker();
        Scheduler_SetEvent_LETIMER0_UF();
    }
    else if (interrupt_flags & _LETIMER_IF_COMP1_MASK) {
        Scheduler_SetEvent_LETIMER0_COMP1();
    }
}


void I2C0_IRQHandler(void) {
    I2C_TransferReturn_TypeDef transfer_status;

    transfer_status = I2C_Transfer(I2C0);

    if (transfer_status == i2cTransferDone) {
        Scheduler_SetEvent_I2C0_TRANSFER_DONE();
        NVIC_DisableIRQ(I2C0_IRQn);
    }
}


void GPIO_EVEN_IRQHandler(void) {
    uint32_t interrupt_flags = GPIO_IntGet();
    GPIO->IFC = 0xFFFF;
    static uint8_t pressed = 1; // Button press is first time we enter ISR

    if (interrupt_flags & (0x01 << PB0_pin)) {
        if (!pressed) {
            Scheduler_SetEvent_PB0_RELEASED();
        }
        else {
            Scheduler_SetEvent_PB0_PRESSED();
        }

        pressed ^= 1;
    }
}


void GPIO_ODD_IRQHandler(void) {
    uint32_t interrupt_flags = GPIO_IntGet();
    GPIO->IFC = 0xFFFF;
    static uint8_t pressed = 1; // Button press is first time we enter ISR

    if (interrupt_flags & (0x01 << PB1_pin)) {
        if (!pressed) {
            Scheduler_SetEvent_PB1_RELEASED();
        }
        else {
            Scheduler_SetEvent_PB1_PRESSED();
        }

        pressed ^= 1;
    }
}
