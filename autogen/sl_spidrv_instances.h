#ifndef SL_SPIDRV_INSTANCES_H
#define SL_SPIDRV_INSTANCES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "spidrv.h"
extern SPIDRV_Handle_t sl_spidrv_FLASH_MEM_handle;

void sl_spidrv_init_instances(void);

#ifdef __cplusplus
}
#endif

#endif // SL_SPIDRV_INSTANCES_H
