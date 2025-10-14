#ifndef __DEV_EMMC_H__
#define __DEV_EMMC_H__

#include "dev_linkFatFs.h"

extern const Diskio_drvTypeDef emmc_driver;
DSTATUS dev_emmcInitialize(BYTE lun);
void dev_emmcPrintfInfo(void);
void dev_eMMC_WriteCpltCallback(void);
void dev_eMMC_ReadCpltCallback(void);


#endif
