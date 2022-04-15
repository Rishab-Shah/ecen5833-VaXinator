#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// $[ACMP0]
// [ACMP0]$

// $[ACMP1]
// [ACMP1]$

// $[ADC0]
// [ADC0]$

// $[CMU]
// [CMU]$

// $[DBG]
// DBG SWCLKTCK on PF0
#define DBG_SWCLKTCK_PORT                        gpioPortF
#define DBG_SWCLKTCK_PIN                         0
#define DBG_ROUTE_LOC                            0

// DBG SWDIOTMS on PF1
#define DBG_SWDIOTMS_PORT                        gpioPortF
#define DBG_SWDIOTMS_PIN                         1

// DBG TDO on PF2
#define DBG_TDO_PORT                             gpioPortF
#define DBG_TDO_PIN                              2

// [DBG]$

// $[ETM]
// [ETM]$

// $[PTI]
// [PTI]$

// $[GPIO]
// [GPIO]$

// $[I2C0]
// I2C0 SCL on PC9
#define I2C0_SCL_PORT                            gpioPortC
#define I2C0_SCL_PIN                             9
#define I2C0_SCL_LOC                             13

// I2C0 SDA on PC8
#define I2C0_SDA_PORT                            gpioPortC
#define I2C0_SDA_PIN                             8
#define I2C0_SDA_LOC                             13

// [I2C0]$

// $[I2C1]
// [I2C1]$

// $[LESENSE]
// [LESENSE]$

// $[LETIMER0]
// [LETIMER0]$

// $[LEUART0]
// LEUART0 RX on PC10
#define LEUART0_RX_PORT                          gpioPortC
#define LEUART0_RX_PIN                           10
#define LEUART0_RX_LOC                           14

// LEUART0 TX on PC11
#define LEUART0_TX_PORT                          gpioPortC
#define LEUART0_TX_PIN                           11
#define LEUART0_TX_LOC                           16

// [LEUART0]$

// $[LFXO]
// [LFXO]$

// $[MODEM]
// [MODEM]$

// $[PCNT0]
// [PCNT0]$

// $[PRS.CH0]
// [PRS.CH0]$

// $[PRS.CH1]
// [PRS.CH1]$

// $[PRS.CH2]
// [PRS.CH2]$

// $[PRS.CH3]
// [PRS.CH3]$

// $[PRS.CH4]
// [PRS.CH4]$

// $[PRS.CH5]
// [PRS.CH5]$

// $[PRS.CH6]
// [PRS.CH6]$

// $[PRS.CH7]
// [PRS.CH7]$

// $[PRS.CH8]
// [PRS.CH8]$

// $[PRS.CH9]
// [PRS.CH9]$

// $[PRS.CH10]
// [PRS.CH10]$

// $[PRS.CH11]
// [PRS.CH11]$

// $[TIMER0]
// [TIMER0]$

// $[TIMER1]
// [TIMER1]$

// $[USART0]
// USART0 RX on PF4
#define USART0_RX_PORT                           gpioPortF
#define USART0_RX_PIN                            4
#define USART0_RX_LOC                            27

// USART0 TX on PF3
#define USART0_TX_PORT                           gpioPortF
#define USART0_TX_PIN                            3
#define USART0_TX_LOC                            27

// [USART0]$

// $[USART1]
// USART1 CLK on PA1
#define USART1_CLK_PORT                          gpioPortA
#define USART1_CLK_PIN                           1
#define USART1_CLK_LOC                           31

// USART1 CS on PA3
#define USART1_CS_PORT                           gpioPortA
#define USART1_CS_PIN                            3
#define USART1_CS_LOC                            0

// USART1 RX on PA2
#define USART1_RX_PORT                           gpioPortA
#define USART1_RX_PIN                            2
#define USART1_RX_LOC                            1

// USART1 TX on PA0
#define USART1_TX_PORT                           gpioPortA
#define USART1_TX_PIN                            0
#define USART1_TX_LOC                            0

// [USART1]$

// $[USART2]
// [USART2]$

// $[VDAC0]
// [VDAC0]$

// $[WTIMER0]
// [WTIMER0]$

#endif // PIN_CONFIG_H

