#ifndef SL_MEMLCD_CONFIG_H
#define SL_MEMLCD_CONFIG_H

// <<< sl:start pin_tool >>>
// <usart signal=TX,CLK> SL_MEMLCD_SPI
// $[USART_SL_MEMLCD_SPI]
#define SL_MEMLCD_SPI_PERIPHERAL                 USART1
#define SL_MEMLCD_SPI_PERIPHERAL_NO              1

// USART1 TX on PA0
#define SL_MEMLCD_SPI_TX_PORT                    gpioPortA
#define SL_MEMLCD_SPI_TX_PIN                     0
#define SL_MEMLCD_SPI_TX_LOC                     0

// USART1 CLK on PA1
#define SL_MEMLCD_SPI_CLK_PORT                   gpioPortA
#define SL_MEMLCD_SPI_CLK_PIN                    1
#define SL_MEMLCD_SPI_CLK_LOC                    31
// [USART_SL_MEMLCD_SPI]$

// <gpio> SL_MEMLCD_SPI_CS
// $[GPIO_SL_MEMLCD_SPI_CS]
#define SL_MEMLCD_SPI_CS_PORT                    gpioPortD
#define SL_MEMLCD_SPI_CS_PIN                     14
// [GPIO_SL_MEMLCD_SPI_CS]$

// <gpio optional=true> SL_MEMLCD_EXTCOMIN
// $[GPIO_SL_MEMLCD_EXTCOMIN]
#define SL_MEMLCD_EXTCOMIN_PORT                  gpioPortD
#define SL_MEMLCD_EXTCOMIN_PIN                   13
// [GPIO_SL_MEMLCD_EXTCOMIN]$

// <<< sl:end pin_tool >>>

#endif
