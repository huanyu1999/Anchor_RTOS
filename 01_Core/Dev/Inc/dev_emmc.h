#ifndef __DEV_EMMC_H__
#define __DEV_EMMC_H__

#include "dev_linkFatFs.h"
#include "cmsis_os.h"

extern osSemaphoreId_t emmcWriteSemaID;
extern osSemaphoreId_t emmcReadSemaID;

extern const Diskio_drvTypeDef emmc_driver;
DSTATUS dev_emmcSemaInit(void);
DSTATUS dev_emmcInitialize(BYTE lun);
void dev_emmcPrintfInfo(void);
void dev_eMMC_WriteCpltCallback(void);
void dev_eMMC_ReadCpltCallback(void);
DSTATUS dev_emmcStatus(BYTE lun);


#endif
