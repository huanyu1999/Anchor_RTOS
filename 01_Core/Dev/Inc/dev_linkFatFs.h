#ifndef __DEV_LINKFATFS_H__
#define __DEV_LINKFATFS_H__

#include "diskio.h"
#include "ff.h"
#include <stdint.h>

/**
  * @brief  Disk IO Driver structure definition
  */
typedef struct
{
    DSTATUS (*disk_initialize) (BYTE);                     /*!< Initialize Disk Drive                     */
    DSTATUS (*disk_status)     (BYTE);                     /*!< Get Disk Status                           */
    DRESULT (*disk_read)       (BYTE, BYTE*, DWORD, UINT);       /*!< Read Sector(s)                            */
#if _USE_WRITE == 1
    DRESULT (*disk_write)      (BYTE, const BYTE*, DWORD, UINT); /*!< Write Sector(s) when _USE_WRITE = 0       */
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
    DRESULT (*disk_ioctl)      (BYTE, BYTE, void*);              /*!< I/O control operation when _USE_IOCTL = 1 */
#endif /* _USE_IOCTL == 1 */
    
}Diskio_drvTypeDef;

/**
  * @brief  Global Disk IO Drivers structure definition
  */
typedef struct
{
    uint8_t                 is_initialized[FF_VOLUMES];
    const Diskio_drvTypeDef *drv[FF_VOLUMES];
    uint8_t                 lun[FF_VOLUMES];
    volatile uint8_t        nbr;
}Disk_drvTypeDef;


uint8_t dev_FATFS_LinkDriverEx(const Diskio_drvTypeDef *drv, char *path, uint8_t lun);
uint8_t dev_FATFS_LinkDriver(const Diskio_drvTypeDef *drv, char *path);
uint8_t dev_FATFS_UnLinkDriverEx(char *path, uint8_t lun);
uint8_t dev_FATFS_UnLinkDriver(char *path);
uint8_t dev_FATFS_GetAttachedDriversNbr(void);
#endif
