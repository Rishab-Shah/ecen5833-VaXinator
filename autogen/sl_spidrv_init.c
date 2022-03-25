#include "spidrv.h"
#include "sl_spidrv_instances.h"

#include "sl_spidrv_FLASH_MEM_config.h"

SPIDRV_HandleData_t sl_spidrv_FLASH_MEM_handle_data;
SPIDRV_Handle_t sl_spidrv_FLASH_MEM_handle = &sl_spidrv_FLASH_MEM_handle_data;

SPIDRV_Init_t sl_spidrv_init_FLASH_MEM = {
  .port = SL_SPIDRV_FLASH_MEM_PERIPHERAL,
#if defined(_USART_ROUTELOC0_MASK)
  .portLocationTx = SL_SPIDRV_FLASH_MEM_TX_LOC,
  .portLocationRx = SL_SPIDRV_FLASH_MEM_RX_LOC,
  .portLocationClk = SL_SPIDRV_FLASH_MEM_CLK_LOC,
  .portLocationCs = SL_SPIDRV_FLASH_MEM_CS_LOC,
#elif defined(_GPIO_USART_ROUTEEN_MASK)
  .portTx = SL_SPIDRV_FLASH_MEM_TX_PORT,
  .portRx = SL_SPIDRV_FLASH_MEM_RX_PORT,
  .portClk = SL_SPIDRV_FLASH_MEM_CLK_PORT,
  .portCs = SL_SPIDRV_FLASH_MEM_CS_PORT,
  .pinTx = SL_SPIDRV_FLASH_MEM_TX_PIN,
  .pinRx = SL_SPIDRV_FLASH_MEM_RX_PIN,
  .pinClk = SL_SPIDRV_FLASH_MEM_CLK_PIN,
  .pinCs = SL_SPIDRV_FLASH_MEM_CS_PIN,
#else
  .portLocation = SL_SPIDRV_FLASH_MEM_ROUTE_LOC,
#endif
  .bitRate = SL_SPIDRV_FLASH_MEM_BITRATE,
  .frameLength = SL_SPIDRV_FLASH_MEM_FRAME_LENGTH,
  .dummyTxValue = 0,
  .type = SL_SPIDRV_FLASH_MEM_TYPE,
  .bitOrder = SL_SPIDRV_FLASH_MEM_BIT_ORDER,
  .clockMode = SL_SPIDRV_FLASH_MEM_CLOCK_MODE,
  .csControl = SL_SPIDRV_FLASH_MEM_CS_CONTROL,
  .slaveStartMode = SL_SPIDRV_FLASH_MEM_SLAVE_START_MODE,
};

void sl_spidrv_init_instances(void) {
  SPIDRV_Init(sl_spidrv_FLASH_MEM_handle, &sl_spidrv_init_FLASH_MEM);
}
