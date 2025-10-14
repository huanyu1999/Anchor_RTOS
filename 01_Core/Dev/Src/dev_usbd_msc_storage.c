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
#include "dev_usbd_msc_storage.h"
#include "drv_emmc.h"
#include "cmsis_os.h"
#include "elog.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
extern osSemaphoreId_t emmcWriteSemaID;
extern osSemaphoreId_t emmcReadSemaID;
#define STORAGE_TIMEOUT 4 * 1000
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#define STORAGE_LUN_NBR                  1U
#define STORAGE_BLK_NBR                  0x10000U
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

int8_t STORAGE_Init(uint8_t lun);
int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t *block_num,  uint16_t *block_size);
int8_t STORAGE_IsReady(uint8_t lun);
int8_t STORAGE_IsWriteProtected(uint8_t lun);
int8_t STORAGE_Read(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
int8_t STORAGE_Write(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
int8_t STORAGE_GetMaxLun(void);

USBD_StorageTypeDef USBD_MSC_Template_fops = {
    STORAGE_Init,
    STORAGE_GetCapacity,
    STORAGE_IsReady,
    STORAGE_IsWriteProtected,
    STORAGE_Read,
    STORAGE_Write,
    STORAGE_GetMaxLun,
    STORAGE_Inquirydata,
};

/**
  * @brief  Initializes the storage unit (medium)
  * @param  lun: Logical unit number
  * @retval Status (0 : OK / -1 : Error)
  */
int8_t STORAGE_Init(uint8_t lun) {
    drv_emmcInit();
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

    int res = -1;
    HAL_MMC_CardInfoTypeDef emmcCardInfo;
    HAL_MMC_CardCIDTypeDef  emmcCardCID;

    if (drv_emmcGetState() == EMMC_TRANSFER_OK) {
        drv_emmcGetInfo(&emmcCardInfo, &emmcCardCID);
        res = 0;
    }

    *block_num  = emmcCardInfo.LogBlockNbr - 1;
    *block_size = emmcCardInfo.LogBlockSize;

    return res;
}


/**
  * @brief  Checks whether the medium is ready.
  * @param  lun: Logical unit number
  * @retval Status (0: OK / -1: Error)
  */
int8_t  STORAGE_IsReady(uint8_t lun) {
    UNUSED(lun);

    int res = -1;
    if (drv_emmcGetState() == EMMC_TRANSFER_OK) {
        res = 0;
    }
    return res;
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


