#ifndef __DEV_EMMC_H__
#define __DEV_EMMC_H__

#include "dev_linkFatFs.h"

extern const Diskio_drvTypeDef emmc_driver;

DSTATUS dev_emmcInitialize(BYTE lun);
// DSTATUS dev_emmcStatus(BYTE lun);
// DRESULT dev_emmcRead(BYTE lun, BYTE *buff, DWORD sector, UINT count);

// #if _USE_WRITE == 1
// DRESULT dev_emmcWrite(BYTE lun, const BYTE *buff, DWORD sector, UINT count);
// #endif /* _USE_WRITE == 1 */

// #if _USE_IOCTL == 1
// DRESULT dev_emmcIoCtl(BYTE lun, BYTE cmd, void *buff);
// #endif /* _USE_IOCTL == 1 */

void dev_emmcPrintfInfo(void);

#endif
