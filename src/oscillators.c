/*
 * oscillators.c - Oscillator functions
 *
 *  Created on: Sep 5, 2021
 *      Author: vishnu
 */

#include "oscillators.h"

void Osc_InitLETIMER0(void) {

#if COMMENT
#if ((LOWEST_ENERGY_MODE == EMO_MODE) || (LOWEST_ENERGY_MODE == EM1_MODE) || (LOWEST_ENERGY_MODE == EM2_MODE))
  /* Below configuration work for EM0 to EM2 modes - 32,768 Hz (before prescaler)*/
  //select LFX0,and return only when the clock has stabilized
  CMU_OscillatorEnable(cmuOsc_LFXO,true,true);

  //enables the chosen clock source, if not enabled it should
  //the relevant clock tree
  CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);

#elif (LOWEST_ENERGY_MODE == EM3_MODE)
  /* Below is configured for EM3 mode - 1000 Hz (before prescaler)*/
  //select ULFRC0,and return only when the clock has stabilized
  CMU_OscillatorEnable(cmuOsc_ULFRCO,true,true);

  //enables the chosen clock source
  CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_ULFRCO);
#endif

  //set the pre-scaler value (tentative)
  CMU_ClockDivSet(cmuClock_LETIMER0,LFACLK_PRESCALER_DIV_RATIO);

  //Enable LETIMER0 (tail end)
  CMU_ClockEnable(cmuClock_LETIMER0,true);
#endif

#if 1
    if (LOWEST_ENERGY_LEVEL == EM3) {
        CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
        CMU_ClockDivSet(cmuClock_LETIMER0, cmuClkDiv_1);
        CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);



        CMU_ClockEnable(cmuClock_LFA, true);
        CMU_ClockEnable(cmuClock_LETIMER0, true);
    }
    else {
        CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
        CMU_ClockDivSet(cmuClock_LETIMER0, cmuClkDiv_4);
        CMU_OscillatorEnable(cmuOsc_LFXO, true, true);


        CMU_ClockEnable(cmuClock_LFA, true);
        CMU_ClockEnable(cmuClock_LETIMER0, true);

    }

    CMU_ClockEnable(cmuClock_HFLE, true); // Necessary for accessing LE modules
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO); // Set a reference clock

    // Enable clocks for LEUART0
    CMU_ClockEnable(cmuClock_LEUART0, true);
    CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_1); // Don't prescale LEUART clock

    CMU_ClockEnable(cmuClock_GPIO,true);

#endif
}

void Osc_InitI2C0(void) {
    CMU_ClockSelectSet(cmuClock_HFPER, cmuSelect_HFRCO);
    CMU_ClockDivSet(cmuClock_HFPER, cmuClkDiv_4);
    CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);
}
