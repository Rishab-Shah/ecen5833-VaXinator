/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * Date:        08-07-2021
 * Author:      Rishab
 * Description: This code was created by the Silicon Labs application wizard
 *              and started as "Bluetooth - SoC Empty".
 *              It is to be used only for ECEN 5823 "IoT Embedded Firmware".
 *              The MSLA referenced above is in effect.
 *
 ******************************************************************************/


#include "app.h"

// Include logging for this file
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"


/*****************************************************************************
 * Application Power Manager callbacks
 *****************************************************************************/
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)



bool app_is_ok_to_sleep(void)
{

    return APP_IS_OK_TO_SLEEP;

} // app_is_ok_to_sleep()



sl_power_manager_on_isr_exit_t app_sleep_on_isr_exit(void)
{

    return APP_SLEEP_ON_ISR_EXIT;

} // app_sleep_on_isr_exit()



#endif // defined(SL_CATALOG_POWER_MANAGER_PRESENT)




/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{

    gpioInit(); // DOS, added this back in.

    // DOS
#if (LOWEST_ENERGY_LEVEL == EM1 || LOWEST_ENERGY_LEVEL == EM2)
            sl_power_manager_add_em_requirement(LOWEST_ENERGY_LEVEL);
#endif
    Osc_InitLETIMER0();
    LETIMER0_Init();
    LETIMER0_Start();
#if 1
    BLE_Init();
    I2C0_Init();
    //LEUART0_Init();
    //sl_uartdrv_init_instances();
#endif
    LOG_INFO("Hey print - after spi\r");
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
    // Put your application code here.
    // This is called repeatedly from the main while(1) loop
    // Notice: This function is not passed or has access to Bluetooth stack events.
    //         We will create/use a scheme that is far more energy efficient in
    //         later assignments.


#if NO_BL

#else
  ble_ext_signal_event_t event = Scheduler_GetNextEvent();
  init_bno055_machine(event);
  LOG_INFO("START OF CODE\r");
#endif
  //TemperatureStateMachine(event);
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *
 * The code here will process events from the Bluetooth stack. This is the only
 * opportunity we will get to act on an event.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{


    // Just a trick to hide a compiler warning about unused input parameter evt.
    // We will add real functionality here later.
    //if (evt->header) {
    //        printf(".\n");
    //}

    // Some events require responses from our application code,
    // and don’t necessarily advance our state machines.
    // For assignment 5 uncomment the next 2 function calls
    // put this code in ble.c/.h
    // sequence through states driven by events
#if DEVICE_IS_BLE_SERVER
#if NO_BL
    handle_ble_event(evt);
    ble_ext_signal_event_t event = evt->data.evt_system_external_signal.extsignals;

    if(event == ev_PB0_RELEASED)
    {
      gpioPMICSetOff();
    }

    //AssetMonitoringSystem_StateMachine(evt);
    //init_bme280_machine(evt);
    init_flash_setup(evt);
#else
    //nothing to write
#endif

#else
    //BleClient_DiscoveryStateMachine(evt);
#endif


#if 0
    uint8_t databuf[200] = {0};//{"ABCDEFGH"};
    //uint8_t rxbuf[10] = {"ABCDEFGH"};
    UARTDRV_ReceiveB(sl_uartdrv_get_default(), databuf, 200);
    LOG_INFO("GPS - %s\r", databuf);
//    static int i = 0;
//    while(i<8)
//      {
//          UARTDRV_TransmitB(sl_uartdrv_get_default(), &databuf[i], 1);
//          LOG_INFO("GPS TX - %c\r\n", databuf[i]);
//
//          UARTDRV_ReceiveB(sl_uartdrv_get_default(), &rxbuf[i], 1);
//          LOG_INFO("GPS RX - %c\r\n", rxbuf[i]);
//          i++;
//      }
#endif
} // sl_bt_on_event()

