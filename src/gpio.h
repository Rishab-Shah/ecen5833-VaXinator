/*
   gpio.h
  
    Created on: Dec 12, 2018
        Author: Dan Walkes

    Updated by Dave Sluiter Sept 7, 2020. moved #defines from .c to .h file.
    Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.
    Updated by Rishab Shah Nov 12, 2021. Configured GPIO for project.

 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_
#include <stdbool.h>
#include "em_gpio.h"
#include <string.h>


// Student Edit: Define these, 0's are placeholder values.
// See the radio board user guide at https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf
// and GPIO documentation at https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/group__GPIO.html
// to determine the correct values for these.

#define PMIC_port           (gpioPortA)
#define PMIC_pin            (5)

#define	LED0_port           (gpioPortF)
#define LED0_pin            (4)

#define Debug_port          (gpioPortA)
#define Debug_pin           (4)

#define GPSLoadSw_port      (gpioPortF)
#define GPSLoadSw_pin       (5)

#define PB0_port            (gpioPortF)
#define PB0_pin             (6)
#define PB1_port            (gpioPortF)
#define PB1_pin             (7)





#define LCD_ENABLE_PORT     (gpioPortD)
#define LCD_ENABLE_PIN      (15)

#define LCD_EXCOMIN_PORT    (gpioPortD)
#define LCD_EXCOMIN_PIN     (13)

//PROJECT::MAX30101 - GPIO Configurations
#define MAX30101_port        (gpioPortD)
#define MAX30101_reset_pin   (10)
#define MAX30101_mfio_pin    (11)

#define BUTTON_PRESSED       (0)
#define BUTTON_RELEASE       (1)


// Function prototypes
void gpioInit();
void gpioDebugLEDSetOn();
void gpioDebugLEDSetOff();
void gpioGPSLoadSwitchOff();
void gpioGPSLoadSwitchOn();
void gpioPMICSetOn();
void gpioPMICSetOff();

void PB0_Init(void);
void PB1_Init(void);



//PROJECT::MAX30101 - GPIO Configurations
void gpioMAX30101_InitConfigurations();
void gpioPowerOff_mfio_MAX30101();
void gpioPowerOn_mfio_MAX30101();
void gpioPowerOff_reset_MAX30101();
void gpioPowerOn_reset_MAX30101();
void gpioSensorEnSetOn(void);
void gpioSetDisplayExtcomin(int on);
#endif /* SRC_GPIO_H_ */
