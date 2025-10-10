#include "dev_linkFatFs.h"
#include "dev_emmc.h"
#include "board_emmc.h"
#include "drv_emmc.h"
#include "elog.h"
#include "cmsis_os.h"

// #define DISABLE_SD_INIT 0
#define SDQUEUE_SIZE       (uint32_t) 10

#define SD_TIMEOUT         4 * 1000
#define SD_DEFAULT_BLOCK_SIZE   512

/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

// static osMessageQueueId_t SDQueueID;
uint32_t SDQueueBuffer[SDQUEUE_SIZE];          // 定义SDQueue的存储空间
StaticQueue_t SDQueueCB;                     
const osMessageQueueAttr_t SDQueue_attr = {     
    .name    = "SDQueue",
    .cb_mem  = &SDQueueCB,
    .cb_size = sizeof(SDQueueCB),
    .mq_mem  = &SDQueueBuffer,
    .mq_size = sizeof(SDQueueBuffer)
};

static osSemaphoreId_t SDSemaID;
StaticQueue_t SDSemaCB;                        // 定义SDQueue的控制块
const osSemaphoreAttr_t SDSema_attr = {     
    .name    = "SDSema",
    .cb_mem  = &SDSemaCB,
    .cb_size = sizeof(SDSemaCB),
};

static osSemaphoreId_t SDReadSemaID;
StaticQueue_t SDReadSemaCB;                    // 定义SDQueue的控制块
const osSemaphoreAttr_t SDReadSema_attr = {    
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

static DSTATUS dev_emmcCheckStatus(BYTE lun);
DSTATUS dev_emmcInitialize(BYTE lun);
DSTATUS dev_emmcStatus(BYTE lun);
DRESULT dev_emmcRead(BYTE lun, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
DRESULT dev_emmcWrite(BYTE lun, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
DRESULT dev_emmcIoCtl(BYTE lun, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

const Diskio_drvTypeDef emmc_driver = 
{
    dev_emmcInitialize,
    dev_emmcStatus,
    dev_emmcRead,
#if _USE_WRITE == 1
    dev_emmcWrite,
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
    dev_emmcIoCtl
#endif /* _USE_IOCTL == 1 */
};

/**
  * @brief  Initializes a Drive
  * @
    dev_SD_ioctl,param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS dev_emmcInitialize(BYTE lun)
{
    Stat = STA_NOINIT;
    /*
    * check that the kernel has been started before continuing
    * as the osMessage API will fail otherwise
    */
    if (osKernelGetState())              // 系统目前在运行
    {
#if !defined(DISABLE_SD_INIT)
        if (drv_emmcInit() == EMMC_OK)    // SDIO接口初始化成功
        {
            Stat = dev_emmcCheckStatus(lun); // 获取sd卡状态
        }
        board_emmcInit();
#else
        Stat = dev_emmcCheckStatus(lun);
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
            dev_emmcPrintfInfo();
        }
    }
    return Stat;
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS dev_emmcStatus(BYTE lun)
{
    return dev_emmcCheckStatus(lun);
}

/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT dev_emmcRead(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
    DRESULT res = RES_ERROR;
    uint32_t timer;  
    osStatus status;

    if (drv_emmcReadBlocks((uint32_t*)buff, (uint32_t) (sector), count) == EMMC_OK)
    {
        /* wait for a message from the queue or a timeout */
        status = osSemaphoreAcquire(SDReadSemaID, SD_TIMEOUT);
        if (status == osOK)                     // 成功接收到队列
        {
            timer = osKernelGetTickCount() + SD_TIMEOUT;
            while(timer > osKernelGetTickCount())                            /* block until SDIO IP is ready or a timeout occur */
            {
                if (drv_emmcGetState() == EMMC_TRANSFER_OK)           // 超时完成后，读取SDCard状态，状态为传输完成，直接返回
                {
                    res = RES_OK;
                    break;
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
DRESULT dev_emmcWrite(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
    // osEvent event;
    DRESULT res = RES_ERROR;
    uint32_t timer;
    osStatus status;

    if(drv_emmcWriteBlocks((uint32_t*)buff, (uint32_t)(sector), count) == EMMC_OK)
    {
        /* Get the message from the queue */
        status = osSemaphoreAcquire(SDSemaID ,SD_TIMEOUT);
        if (status == osOK)
        {
            timer = osKernelGetTickCount() + SD_TIMEOUT;
            while(timer > osKernelGetTickCount())                        /* block until SDIO IP is ready or a timeout occur */
            {
                if (drv_emmcGetState() == EMMC_TRANSFER_OK)
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
DRESULT dev_emmcIoCtl(BYTE lun, BYTE cmd, void *buff)
{
    DRESULT res = RES_ERROR;
    HAL_MMC_CardInfoTypeDef emmcCardInfo;
    HAL_MMC_CardCIDTypeDef  emmcCardCID;

    if (Stat & STA_NOINIT) return RES_NOTRDY;

    switch (cmd)
    {
    /* Make sure that no pending write process */
    case CTRL_SYNC :
        res = RES_OK;
        break;

    /* Get number of sectors on the disk (DWORD) */
    case GET_SECTOR_COUNT :
        drv_emmcGetInfo(&emmcCardInfo, &emmcCardCID);
        *(DWORD*)buff = emmcCardInfo.LogBlockNbr;
        res = RES_OK;
        break;

    /* Get R/W sector size (WORD) */
    case GET_SECTOR_SIZE :
        drv_emmcGetInfo(&emmcCardInfo, &emmcCardCID);
        *(WORD*)buff = emmcCardInfo.LogBlockSize;
        res = RES_OK;
        break;

    /* Get erase block size in unit of sector (DWORD) */
    case GET_BLOCK_SIZE :
        drv_emmcGetInfo(&emmcCardInfo, &emmcCardCID);
        *(DWORD*)buff = emmcCardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
        res = RES_OK;
        break;

    default:
        res = RES_PARERR;
    }

    return res;
}
#endif /* _USE_IOCTL == 1 */

void dev_emmcPrintfInfo(void)
{
    uint64_t card_cap;
    HAL_MMC_CardInfoTypeDef emmc_cardInfo;
    HAL_MMC_CardCIDTypeDef emmc_cardCID;

    drv_emmcGetInfo(&emmc_cardInfo, &emmc_cardCID);
    card_cap = (uint64_t)(emmc_cardInfo.LogBlockNbr) * (uint64_t)(emmc_cardInfo.LogBlockSize);
    switch (emmc_cardInfo.CardType)
    {
        case MMC_LOW_CAPACITY_CARD :
            log_i("Card Type:Low capacity.\r\n");
            break;
        
        case MMC_HIGH_CAPACITY_CARD :
            log_i("Card Type:High capacity.\r\n");
            break;

        default :
            break;
    }

    log_i("Card ManufacturerID: %d \r\n",emmc_cardCID.ManufacturerID);				//制造商ID	
    log_i("Class:               %d \r\n",(uint32_t)(emmc_cardInfo.Class));		    //
    log_i("Card RCA(RelCardAdd):%d \r\n",emmc_cardInfo.RelCardAdd);					//卡相对地址
    log_i("Card BlockNbr:       %d \r\n",emmc_cardInfo.BlockNbr);						//块数量
    log_i("Card BlockSize:      %d \r\n",emmc_cardInfo.BlockSize);					//块大小
    log_i("LogBlockNbr:         %d \r\n",(uint32_t)(emmc_cardInfo.LogBlockNbr));		//逻辑块数量
    log_i("LogBlockSize:        %d \r\n",(uint32_t)(emmc_cardInfo.LogBlockSize));		//逻辑块大小
    log_i("Card Capacity:       %d MB\r\n",(uint32_t)(card_cap>>20));				//卡容量
}

/**
  * @brief Tx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void dev_eMMC_WriteCpltCallback(void)
{
    /*
     * No need to add an "osKernelRunning()" check here, as the SD_initialize()
     * is always called before any SD_Read()/SD_Write() call
     */
    // SD卡写操作完成，向队列发送写完成消息
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
void dev_eMMC_ReadCpltCallback(void)
{
    /*
     * No need to add an "osKernelRunning()" check here, as the SD_initialize()
     * is always called before any SD_Read()/SD_Write() call
    */
    if (osSemaphoreRelease(SDReadSemaID) != osOK)
    {
        log_d("read cplt osSemaphoreRelease error.");
    }
}

static DSTATUS dev_emmcCheckStatus(BYTE lun)
{
    Stat = STA_NOINIT;

    if(drv_emmcGetState() == EMMC_OK) // 判断SDCard 的状态
    {
        Stat &= ~STA_NOINIT;
    }

    return Stat;
}

