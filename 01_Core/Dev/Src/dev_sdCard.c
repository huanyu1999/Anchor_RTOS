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
#include "cmsis_os.h"

#include "dev_linkFatFs.h"
#include "dev_sdCard.h"
#include "drv_sdio.h"

#include "elog.h"

#define SDQUEUE_SIZE       (uint32_t) 10
#define READ_CPLT_MSG      (uint32_t) 1
#define WRITE_CPLT_MSG     (uint32_t) 2

#define SD_TIMEOUT         10 * 1000
#define SD_DEFAULT_BLOCK_SIZE   512

/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

// static osMessageQueueId_t SDQueueID;
uint32_t SDQueueBuffer[SDQUEUE_SIZE];           // 定义SDQueue的存储空间
StaticQueue_t SDQueueCB;                        // 定义SDQueue的控制块
const osMessageQueueAttr_t SDQueue_attr = {     // 定义SDQueue的属性
    .name    = "SDQueue",
    .cb_mem  = &SDQueueCB,
    .cb_size = sizeof(SDQueueCB),
    .mq_mem  = &SDQueueBuffer,
    .mq_size = sizeof(SDQueueBuffer)
};

static osSemaphoreId_t SDSemaID;
StaticQueue_t SDSemaCB;                        // 定义SDQueue的控制块
const osSemaphoreAttr_t SDSema_attr = {     // 定义SDQueue的属性
    .name    = "SDSema",
    .cb_mem  = &SDSemaCB,
    .cb_size = sizeof(SDSemaCB),
};

static osSemaphoreId_t SDReadSemaID;
StaticQueue_t SDReadSemaCB;                        // 定义SDQueue的控制块
const osSemaphoreAttr_t SDReadSema_attr = {     // 定义SDQueue的属性
    .name    = "SDReadSema",
    .cb_mem  = &SDReadSemaCB,
    .cb_size = sizeof(SDReadSemaCB),
};

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
  * @
    dev_SD_ioctl,param  lun : not used
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
            // SDQueueID = osMessageQueueNew(SDQUEUE_SIZE, sizeof(uint32_t), &SDQueue_attr);       // if the SD is correctly initialized, create the operation queue
            SDSemaID = osSemaphoreNew(1, 0, &SDSema_attr);
            SDReadSemaID = osSemaphoreNew(1, 0, &SDReadSema_attr);
            if ((SDSemaID == NULL) || (SDReadSemaID == NULL)) 
            { 
                log_e("osSemaphoreNew failed."); 
            }
            else 
            { 
                log_d("osSemaphoreNew success."); 
            }
            dev_SD_printfInfo();
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
    uint32_t timer;
    // uint32_t msg;         
    osStatus status;

    if(drv_sdioReadBlocks_Dma((uint32_t*)buff, (uint32_t) (sector), count) == MSD_OK)
    {
        /* wait for a message from the queue or a timeout */
        // status = osMessageQueueGet(SDQueueID, (void*)&msg, 0, SD_TIMEOUT);
        status = osSemaphoreAcquire(SDReadSemaID, SD_TIMEOUT);
        if (status == osOK)                     // 成功接收到队列
        {
            // if (msg == READ_CPLT_MSG)           // 接收到的消息为读取完成
            // {
                timer = osKernelGetTickCount() + SD_TIMEOUT;
                while(timer > osKernelGetTickCount())                            /* block until SDIO IP is ready or a timeout occur */
                {
                    if (drv_sdioGetCardState() == SD_TRANSFER_OK)           // 超时完成后，读取SDCard状态，状态为传输完成，直接返回
                    {
                        res = RES_OK;
                        break;
                    }
                }
            // }
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
    // uint32_t msg;           // 存储接收到的队列内容
    osStatus status;

    if(drv_sdioWriteBlocks_Dma((uint32_t*)buff, (uint32_t)(sector), count) == MSD_OK)
    {
        /* Get the message from the queue */
        // status = osMessageQueueGet(SDQueueID, &msg, 0, SD_TIMEOUT);
        status = osSemaphoreAcquire(SDSemaID ,SD_TIMEOUT);
        if (status == osOK)
        {
            timer = osKernelGetTickCount() + SD_TIMEOUT;
            while(timer > osKernelGetTickCount())                        /* block until SDIO IP is ready or a timeout occur */
            {
                if (drv_sdioGetCardState() == SD_TRANSFER_OK)
                {
                    res = RES_OK;
                    break;
                }
            }
        }
        else
        {
            log_e("osSemaphoreAcquire error %d", status);
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
    SD_CardCID  CardCID;

    if (Stat & STA_NOINIT) return RES_NOTRDY;

    switch (cmd)
    {
    /* Make sure that no pending write process */
    case CTRL_SYNC :
        res = RES_OK;
        break;

    /* Get number of sectors on the disk (DWORD) */
    case GET_SECTOR_COUNT :
        drv_sdioGetCardInfo(&CardInfo, &CardCID);
        *(DWORD*)buff = CardInfo.LogBlockNbr;
        res = RES_OK;
        break;

    /* Get R/W sector size (WORD) */
    case GET_SECTOR_SIZE :
        drv_sdioGetCardInfo(&CardInfo, &CardCID);
        *(WORD*)buff = CardInfo.LogBlockSize;
        res = RES_OK;
        break;

    /* Get erase block size in unit of sector (DWORD) */
    case GET_BLOCK_SIZE :
        drv_sdioGetCardInfo(&CardInfo, &CardCID);
        *(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
        res = RES_OK;
        break;

    default:
        res = RES_PARERR;
    }

    return res;
}
#endif /* _USE_IOCTL == 1 */

void dev_SD_printfInfo(void)
{
    uint64_t card_cap;
    HAL_SD_CardInfoTypeDef sd_cardInfo;
    HAL_SD_CardCIDTypeDef sd_cardCID;

    drv_sdioGetCardInfo(&sd_cardInfo, &sd_cardCID);
    card_cap = (uint64_t)(sd_cardInfo.LogBlockNbr) * (uint64_t)(sd_cardInfo.LogBlockSize);
    switch (sd_cardInfo.CardType)
    {
        case CARD_SDSC :
            if(sd_cardInfo.CardVersion == CARD_V1_X)
            {
                log_i("Card Type:SDSC V1\r\n");
            }
            else if(sd_cardInfo.CardVersion == CARD_V2_X)
            {
                log_i("Card Type:SDSC V2\r\n");
            }
            break;
        
        case CARD_SDHC_SDXC :
            log_i("Card Type:CARD_SDHC\r\n");
            break;

        default :
            break;
    }

    log_i("Card ManufacturerID: %d \r\n",sd_cardCID.ManufacturerID);				//制造商ID	
    log_i("CardVersion:         %d \r\n",(uint32_t)(sd_cardInfo.CardVersion));		//卡版本号
    log_i("Class:               %d \r\n",(uint32_t)(sd_cardInfo.Class));		    //
    log_i("Card RCA(RelCardAdd):%d \r\n",sd_cardInfo.RelCardAdd);					//卡相对地址
    log_i("Card BlockNbr:       %d \r\n",sd_cardInfo.BlockNbr);						//块数量
    log_i("Card BlockSize:      %d \r\n",sd_cardInfo.BlockSize);					//块大小
    log_i("LogBlockNbr:         %d \r\n",(uint32_t)(sd_cardInfo.LogBlockNbr));		//逻辑块数量
    log_i("LogBlockSize:        %d \r\n",(uint32_t)(sd_cardInfo.LogBlockSize));		//逻辑块大小
    log_i("Card Capacity:       %d MB\r\n",(uint32_t)(card_cap>>20));				//卡容量
}

/**
  * @brief Tx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void dev_SD_WriteCpltCallback(void)
{
    // uint32_t msg = WRITE_CPLT_MSG;
    /*
     * No need to add an "osKernelRunning()" check here, as the SD_initialize()
     * is always called before any SD_Read()/SD_Write() call
     */
    // SD卡写操作完成，向队列发送写完成消息
    // osMessageQueuePut(SDQueueID, &msg, 0U, osWaitForever);
    // log_d("write cplt ");
    // if (osMessageQueuePut(SDQueueID, &msg, 0U, osWaitForever) == osOK)
    if (osSemaphoreRelease(SDSemaID) != osOK)
    {
        log_d("write cplt osSemaphoreRelease error.");
    }
}

/**
  * @brief Rx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void dev_SD_ReadCpltCallback(void)
{
    // uint32_t msg = READ_CPLT_MSG;
    /*
     * No need to add an "osKernelRunning()" check here, as the SD_initialize()
     * is always called before any SD_Read()/SD_Write() call
    */
    // osMessagePut(SDQueueID, READ_CPLT_MSG, osWaitForever);         // SD卡读操作完成，向队列发送写完成消息
    // log_d("read cplt ");
    // if (osMessageQueuePut(SDQueueID, &msg, 0U, osWaitForever) != osOK)
    if (osSemaphoreRelease(SDReadSemaID) != osOK)
    {
        log_d("read cplt osSemaphoreRelease error.");
    }
}
