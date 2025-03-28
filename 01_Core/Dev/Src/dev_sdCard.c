/**
  ******************************************************************************
  * @file           : dev_sdCard.c
  * @brief          : SD卡设备驱动
  ******************************************************************************
  * @attention
  * 在该文件实现SD卡的初始化，读写，IO控制的功能，再通过Diskio_drvTypeDef SDCard_driver传递给diskio.c访问
  * 
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#include "dev_linkFatFs.h"
#include "dev_sdCard.h"
#include "drv_sdio.h"

#define SDQUEUE_SIZE       (uint32_t) 10
#define READ_CPLT_MSG      (uint32_t) 1
#define WRITE_CPLT_MSG     (uint32_t) 2

#define SD_TIMEOUT         30 * 1000
#define SD_DEFAULT_BLOCK_SIZE   512

/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

static osMessageQueueId_t SDQueueID;                           
uint16_t SDQueueBuffer[SDQUEUE_SIZE];           // 定义SDQueue的存储空间
StaticQueue_t SDQueueCB;                        // 定义SDQueue的控制块
const osMessageQueueAttr_t SDQueue_attr = {     // 定义SDQueue的属性
    .name    = "SDQueue",
    .cb_mem  = &SDQueueCB,
    .cb_size = sizeof(SDQueueCB),
    .mq_mem  = &SDQueueBuffer,
    .mq_size = sizeof(SDQueueBuffer)
};

// static osMessageQId SDQueueID;

/*
 * Depending on the usecase, the SD card initialization could be done at the
 * application level, if it is the case define the flag below to disable
 * the BSP_SD_Init() call in the SD_Initialize().
 */

/* #define DISABLE_SD_INIT */


static DSTATUS dev_SD_CheckStatus(BYTE lun);
DSTATUS dev_SD_initialize(BYTE lun);
DSTATUS dev_SD_status(BYTE lun);
DRESULT dev_SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
DRESULT dev_SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
DRESULT dev_SD_ioctl(BYTE lun, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

const Diskio_drvTypeDef SDCard_driver = 
{
    dev_SD_initialize,
    dev_SD_status,
    dev_SD_read,
#if _USE_WRITE == 1    
    dev_SD_write,
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
    dev_SD_ioctl,
#endif /* _USE_IOCTL == 1 */
};

static DSTATUS dev_SD_CheckStatus(BYTE lun)
{
    Stat = STA_NOINIT;

    if(drv_sdioGetCardState() == MSD_OK) // 判断SDCard 的状态
    {
        Stat &= ~STA_NOINIT;
    }

    return Stat;
}

/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS dev_SD_initialize(BYTE lun)
{
    Stat = STA_NOINIT;
    /*
    * check that the kernel has been started before continuing
    * as the osMessage API will fail otherwise
    */
    if(osKernelGetState())              // 系统目前在运行
    {
#if !defined(DISABLE_SD_INIT)
        if(drv_sdioInit() == MSD_OK)    // SDIO接口初始化成功
        {
            Stat = dev_SD_CheckStatus(lun); // 获取sd卡状态
        }
#else
        Stat = SD_CheckStatus(lun);
#endif

        if (Stat != STA_NOINIT)
        {
            // osMessageQDef(SD_Queue, QUEUE_SIZE, uint16_t);
            // SDQueueID = osMessageCreate (osMessageQ(SD_Queue), NULL);
            SDQueueID = osMessageQueueNew(SDQUEUE_SIZE, sizeof(uint16_t), &SDQueue_attr);       // if the SD is correctly initialized, create the operation queue
        }
    }

    return Stat;
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS dev_SD_status(BYTE lun)
{
    return dev_SD_CheckStatus(lun);
}

/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT dev_SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
    DRESULT res = RES_ERROR;
    // osEvent event;
    uint32_t timer;
    uint32_t msg;           // 存储接收到的队列内容
    osStatus status;

    if(drv_sdioReadBlocks_Dma((uint32_t*)buff, (uint32_t) (sector), count) == MSD_OK)
    {
        /* wait for a message from the queue or a timeout */
        // event = osMessageGet(SDQueueID, SD_TIMEOUT);

        status = osMessageQueueGet(SDQueueID, &msg, NULL, SD_TIMEOUT);

        // if (event.status == osEventMessage)
        if (status == osOK)                     // 成功接收到队列
        {
            // if (event.value.v == READ_CPLT_MSG)
            if (msg == READ_CPLT_MSG)           // 接收到的消息为读取完成
            {
                timer = osKernelSysTick() + SD_TIMEOUT;
                while(timer > osKernelSysTick())                            /* block until SDIO IP is ready or a timeout occur */
                {
                    if (drv_sdioGetCardState() == SD_TRANSFER_OK)           // 超时完成后，读取SDCard状态，状态为传输完成，直接返回
                    {
                        res = RES_OK;
                        break;
                    }
                }
            }
        }
    }

    return res;
}

/**
  * @brief  Writes Sector(s)
  * @param  lun : not used
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT dev_SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
    // osEvent event;
    DRESULT res = RES_ERROR;
    uint32_t timer;
    uint32_t msg;           // 存储接收到的队列内容
    osStatus status;

    if(drv_sdioWriteBlocks_Dma((uint32_t*)buff, (uint32_t) (sector), count) == MSD_OK)
    {
        /* Get the message from the queue */
        // event = osMessageGet(SDQueueID, SD_TIMEOUT);
        status = osMessageQueueGet(SDQueueID, &msg, NULL, SD_TIMEOUT);
        // if (event.status == osEventMessage)
        if (status == osOK)
        {
            // if (event.value.v == WRITE_CPLT_MSG)
            if (msg == WRITE_CPLT_MSG)
            {
                timer = osKernelSysTick() + SD_TIMEOUT;
                while(timer > osKernelSysTick())                        /* block until SDIO IP is ready or a timeout occur */
                {
                    if (drv_sdioGetCardState() == SD_TRANSFER_OK)
                    {
                        res = RES_OK;
                        break;
                    }
                }
            }
        }
    }

    return res;
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT dev_SD_ioctl(BYTE lun, BYTE cmd, void *buff)
{
    DRESULT res = RES_ERROR;
    SD_CardInfo CardInfo;

    if (Stat & STA_NOINIT) return RES_NOTRDY;

    switch (cmd)
    {
    /* Make sure that no pending write process */
    case CTRL_SYNC :
        res = RES_OK;
        break;

    /* Get number of sectors on the disk (DWORD) */
    case GET_SECTOR_COUNT :
        drv_sdioGetCardInfo(&CardInfo);
        *(DWORD*)buff = CardInfo.LogBlockNbr;
        res = RES_OK;
        break;

    /* Get R/W sector size (WORD) */
    case GET_SECTOR_SIZE :
        drv_sdioGetCardInfo(&CardInfo);
        *(WORD*)buff = CardInfo.LogBlockSize;
        res = RES_OK;
        break;

    /* Get erase block size in unit of sector (DWORD) */
    case GET_BLOCK_SIZE :
        drv_sdioGetCardInfo(&CardInfo);
        *(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
        res = RES_OK;
        break;

    default:
        res = RES_PARERR;
    }

    return res;
}
#endif /* _USE_IOCTL == 1 */

/**
  * @brief Tx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void dev_SD_WriteCpltCallback(void)
{
    uint32_t msg = WRITE_CPLT_MSG;
    /*
     * No need to add an "osKernelRunning()" check here, as the SD_initialize()
     * is always called before any SD_Read()/SD_Write() call
     */
    // osMessagePut(SDQueueID, WRITE_CPLT_MSG, osWaitForever);         // SD卡写操作完成，向队列发送写完成消息
    osMessageQueuePut(SDQueueID, &msg, 0U, osWaitForever);
}

/**
  * @brief Rx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void dev_SD_ReadCpltCallback(void)
{
    uint32_t msg = READ_CPLT_MSG;
    /*
     * No need to add an "osKernelRunning()" check here, as the SD_initialize()
     * is always called before any SD_Read()/SD_Write() call
    */
    // osMessagePut(SDQueueID, READ_CPLT_MSG, osWaitForever);         // SD卡读操作完成，向队列发送写完成消息
    osMessageQueuePut(SDQueueID, &msg, 0U, osWaitForever);
}
