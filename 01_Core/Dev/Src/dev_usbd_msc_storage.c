/**
  ******************************************************************************
  * @file    dev_usbd_msc_storage.c
  * @author  MCD Application Team
  * @brief   Memory management layer
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* BSPDependencies

EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "board_emmc.h"
#include "dev_usbd_msc_storage.h"
#include "dev_emmc.h"
#include "drv_emmc.h"
#include "cmsis_os.h"
#include "elog.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define STORAGE_TIMEOUT 3 * 1000
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#define EMMC
// #define RAM_DISK
#define STORAGE_LUN_NBR                  1U

#ifdef EMMC
#define STORAGE_BLK_NBR                  0x1000000 
#define STORAGE_BLK_SIZ                  0x200U
/* (Small Computer System Interface)SCSI Inquiry cmd */
/* USB Mass storage Standard Inquiry Data */
int8_t STORAGE_Inquirydata[] =  /* 36 */
{
    /* LUN 0 */
    0x00,
    0x80,
    0x02,
    0x02,
    (STANDARD_INQUIRY_DATA_LEN - 5),
    0x00,
    0x00,
    0x00,
    'H', 'Z', 'D', ' ', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
    'P', 'r', 'o', 'd', 'u', 'c', 't', ' ', /* Product      : 16 Bytes */
    ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
    '0', '.', '0', '1',                     /* Version      : 4 Bytes */
};
#endif

#ifdef RAM_DISK
/* 模拟 32KB 的 RAM 盘（64 个 512 字节扇区） */
#define STORAGE_BLK_SIZ  512
#define STORAGE_BLK_NBR  64
static uint8_t RAM_Disk[STORAGE_BLK_NBR * STORAGE_BLK_SIZ];
int8_t RAM_STORAGE_Inquirydata[] = {
    0x00, 0x80, 0x02, 0x02,
    (STANDARD_INQUIRY_DATA_LEN - 5),
    0x00, 0x00, 0x00,
    'S','T','M',' ',' ',' ',' ',' ',
    'R','A','M',' ','D','I','S','K',' ',' ',' ',' ',' ',' ',' ',' ',
    '0','.','0','1'
};
#endif

int8_t STORAGE_Init(uint8_t lun);
int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t *block_num,  uint16_t *block_size);
int8_t STORAGE_IsReady(uint8_t lun);
int8_t STORAGE_IsWriteProtected(uint8_t lun);
int8_t STORAGE_Read(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
int8_t STORAGE_Write(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
int8_t STORAGE_GetMaxLun(void);

USBD_StorageTypeDef USBD_MSC_DISK_fops = {
    STORAGE_Init,
    STORAGE_GetCapacity,
    STORAGE_IsReady,
    STORAGE_IsWriteProtected,
    STORAGE_Read,
    STORAGE_Write,
    STORAGE_GetMaxLun,
#ifdef EMMC
    STORAGE_Inquirydata,
#endif
#ifdef RAM_DISK
    RAM_STORAGE_Inquirydata
#endif
};

#ifdef EMMC
/**
  * @brief  Initializes the storage unit (medium)
  * @param  lun: Logical unit number
  * @retval Status (0 : OK / -1 : Error)
  */
int8_t STORAGE_Init(uint8_t lun) {
    uint8_t Stat = STA_NOINIT;

    board_emmcInit();                       // 初始化emmc Reset引脚
    board_emmcReset();                      // 复位emmc
    if (drv_emmcInit() == EMMC_OK) {        // SDIO接口初始化成功
        Stat = dev_emmcStatus(lun);    // 获取sd卡状态
    }
    if (Stat != STA_NOINIT) {
//        emmcWriteSemaID = osSemaphoreNew(1, 0, &emmcWriteSema_attr);
//        emmcReadSemaID = osSemaphoreNew(1, 0, &emmcReadSema_attr);
//        if ((emmcWriteSemaID == NULL) || (emmcReadSemaID == NULL)) { 
//            log_e("osSemaphoreNew failed."); 
//        } else { 
//            log_d("osSemaphoreNew success."); 
//        }
        // 初始化emmc成功，打印emmc的相关信息
        dev_emmcPrintfInfo();
    }
    return (0);
}

/**
  * @brief  Returns the medium capacity.
  * @param  lun: Logical unit number
  * @param  block_num: Number of total block number
  * @param  block_size: Block size
  * @retval Status (0: OK / -1: Error)
  */
int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
    UNUSED(lun);

    HAL_MMC_CardInfoTypeDef emmcCardInfo;
    HAL_MMC_CardCIDTypeDef  emmcCardCID;

    if (drv_emmcGetState() != EMMC_TRANSFER_OK) {
        return -1;
    }

    if (drv_emmcGetInfo(&emmcCardInfo, &emmcCardCID) != EMMC_OK) {
        return -1;
    }

    *block_num  = emmcCardInfo.LogBlockNbr;   // 不需要 -1，否则主机会少识别一块
    *block_size = emmcCardInfo.LogBlockSize;

    return 0;
}


/**
  * @brief  Checks whether the medium is ready.
  * @param  lun: Logical unit number
  * @retval Status (0: OK / -1: Error)
  */
int8_t  STORAGE_IsReady(uint8_t lun) {
    UNUSED(lun);

    // int res = -1;
    // if (drv_emmcGetState() == EMMC_TRANSFER_OK) {
    //     res = 0;
    // }
    return 0;
}

/**
  * @brief  Checks whether the medium is write protected.
  * @param  lun: Logical unit number
  * @retval Status (0: write enabled / -1: otherwise)
  */
int8_t  STORAGE_IsWriteProtected(uint8_t lun) {
    UNUSED(lun);

    return  0;
}

/**
  * @brief  Reads data from the medium.
  * @param  lun: Logical unit number
  * @param  buf: data buffer
  * @param  blk_addr: Logical block address
  * @param  blk_len: Blocks number
  * @retval Status (0: OK / -1: Error)
  */
int8_t STORAGE_Read(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len) {
    UNUSED(lun);
    UNUSED(buf);
    UNUSED(blk_addr);
    UNUSED(blk_len);

    int res = -1;
    uint32_t timer;  
    osStatus status;

    if(drv_emmcReadBlocks((uint32_t*)buf, blk_addr, blk_len) == EMMC_OK) {
        /* Get the message from the queue */
        status = osSemaphoreAcquire(emmcReadSemaID ,STORAGE_TIMEOUT);
        if (status == osOK) {
            timer = osKernelGetTickCount() + STORAGE_TIMEOUT;
            while(timer > osKernelGetTickCount()) {                 /* block until SDIO IP is ready or a timeout occur */
                if (drv_emmcGetState() == EMMC_TRANSFER_OK) {
                    res = 0;
                    break;
                }
            }
        } else {
            log_e("osSemaphoreAcquire error %d", status);
        }
    }
    return res;
}

/**
  * @brief  Writes data into the medium.
  * @param  lun: Logical unit number
  * @param  buf: data buffer
  * @param  blk_addr: Logical block address
  * @param  blk_len: Blocks number
  * @retval Status (0 : OK / -1 : Error)
  */
int8_t STORAGE_Write(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len) {
    UNUSED(lun);
    UNUSED(buf);
    UNUSED(blk_addr);
    UNUSED(blk_len);

    int res = -1;
    uint32_t timer;  
    osStatus status;

    if(drv_emmcWriteBlocks((uint32_t*)buf, blk_addr, blk_len) == EMMC_OK) {
        /* Get the message from the queue */
        status = osSemaphoreAcquire(emmcWriteSemaID ,STORAGE_TIMEOUT);
        if (status == osOK) {
            timer = osKernelGetTickCount() + STORAGE_TIMEOUT;
            while(timer > osKernelGetTickCount()) {                 /* block until SDIO IP is ready or a timeout occur */
                if (drv_emmcGetState() == EMMC_TRANSFER_OK) {
                    res = 0;
                    break;
                }
            }
        } else {
            log_e("osSemaphoreAcquire error %d", status);
        }
    }
    return res;
}

/**
  * @brief  Returns the Max Supported LUNs.
  * @param  None
  * @retval Lun(s) number
  */
int8_t STORAGE_GetMaxLun(void)
{
    return (STORAGE_LUN_NBR - 1);
}
#endif

#ifdef RAM_DISK
int8_t STORAGE_Init(uint8_t lun)
{
    memset(RAM_Disk, 0, sizeof(RAM_Disk));  // 清空
//    RAM_Disk[510] = 0x55;
//    RAM_Disk[511] = 0xAA;
    return 0;
}

int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
    *block_num  = STORAGE_BLK_NBR;
    *block_size = STORAGE_BLK_SIZ;
    return 0;
}

int8_t STORAGE_IsReady(uint8_t lun)
{
    return 0;   // 永远 ready
}

int8_t STORAGE_IsWriteProtected(uint8_t lun)
{
    return 0;   // 可写
}

int8_t STORAGE_Read(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
    if ((blk_addr + blk_len) > STORAGE_BLK_NBR)
        return -1;
    memcpy(buf, &RAM_Disk[blk_addr * STORAGE_BLK_SIZ], blk_len * STORAGE_BLK_SIZ);
    return 0;
}

int8_t STORAGE_Write(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
    if ((blk_addr + blk_len) > STORAGE_BLK_NBR)
        return -1;
    memcpy(&RAM_Disk[blk_addr * STORAGE_BLK_SIZ], buf, blk_len * STORAGE_BLK_SIZ);
    return 0;
}

int8_t STORAGE_GetMaxLun(void)
{
    return (STORAGE_LUN_NBR - 1);
}
#endif


