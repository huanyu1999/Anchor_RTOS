#include <stdint.h>

#include "main.h"
#include "stm32f405xx.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_sd.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_ll_sdmmc.h"
#include "drv_sdio.h"
#include "dev.h"
#include "usart.h"
#include "elog.h"

SD_HandleTypeDef sdCard_Handle;

/**
  * @brief  Initializes the SD card device.
  * @retval SDIO Status
  */
uint8_t drv_sdioInit(void)
{
    uint8_t sd_state = MSD_OK;

    sdCard_Handle.Instance = SDIO;
    sdCard_Handle.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    sdCard_Handle.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE; 
    sdCard_Handle.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    sdCard_Handle.Init.BusWide = SDIO_BUS_WIDE_1B;                              // use one bit data line
    sdCard_Handle.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    sdCard_Handle.Init.ClockDiv = SDIO_TRANSFER_CLK_DIV;                            // 初始化时钟

    if (drv_sdIsDetect() == SDIO_NOT_PRESENT)
    {
        sd_state = MSD_ERROR;
    }

    if(HAL_SD_Init(&sdCard_Handle) != HAL_OK)
    {
        sd_state = MSD_ERROR;
        Error_Handler();
    }
    
    if(sd_state == MSD_OK) 
    {
        if(HAL_SD_ConfigWideBusOperation(&sdCard_Handle, SDIO_BUS_WIDE_4B) != HAL_OK)
        {
            sd_state = MSD_ERROR;
        }
        else 
        {
            sd_state = MSD_OK;
        }
    }
    return sd_state;
}

/**
  * @brief  Configures Interrupt mode for SD detection pin.
  * @retval None
  */
void drv_sdDetectInit(void)
{
    GPIO_InitTypeDef GPIO_Init_Structure;
    
    // 对应的GPIO时钟已经在board_gpio初始化中完成，这里就不执行了

    /* Configure Interrupt mode for SD detection pin */ 
    GPIO_Init_Structure.Mode      = GPIO_MODE_IT_RISING_FALLING;
    GPIO_Init_Structure.Pull      = GPIO_PULLUP;
    GPIO_Init_Structure.Speed     = GPIO_SPEED_HIGH;
    GPIO_Init_Structure.Pin       = SDIO_DECT_PIN;
    HAL_GPIO_Init(SDIO_DECT_PORT, &GPIO_Init_Structure);

    /* NVIC configuration for SDIO interrupts */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0xE, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/**
  * @brief  Detects if SD card is correctly plugged in the memory slot or not.
  * @retval Returns if SD is detected or not
  */
uint8_t drv_sdIsDetect(void)
{
    __IO uint8_t status = SDIO_PRESENT;

    if (HAL_GPIO_ReadPin(SDIO_DECT_PORT, SDIO_DECT_PIN) != GPIO_PIN_RESET)
    {
        status = SDIO_NOT_PRESENT;
    }
    return status;
}

/**
  * @brief  Configures Interrupt mode for SD detection pin.
  * @retval None
  */
void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
{
    static DMA_HandleTypeDef rxHandle;
    static DMA_HandleTypeDef txHandle;
    GPIO_InitTypeDef GPIO_InitStructure;

    __HAL_RCC_SDIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Alternate = GPIO_AF12_SDIO;
    GPIO_InitStructure.Pin = SDIO_CLK_PIN | SDIO_D0_PIN | SDIO_D1_PIN | SDIO_D2_PIN | SDIO_D3_PIN;
    HAL_GPIO_Init(SDIO_CLK_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = SDIO_CMD_PIN;
    HAL_GPIO_Init(SDIO_CMD_PORT, &GPIO_InitStructure);
    
      /* SD Card detect pin configuration */
    GPIO_InitStructure.Mode      = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull      = GPIO_PULLUP;
    GPIO_InitStructure.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pin       = SDIO_DECT_PIN;
    HAL_GPIO_Init(SDIO_DECT_PORT, &GPIO_InitStructure);
    
    // drv_sdDetectInit();

    /* NVIC configuration for SDIO interrupts. */
    HAL_NVIC_SetPriority(SDIO_IRQn, 0x05, 0);
    HAL_NVIC_EnableIRQ(SDIO_IRQn);
    
    /* Configure DMA RX parameters. */
    rxHandle.Init.Channel = DMA_CHANNEL_4;
    rxHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    rxHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    rxHandle.Init.MemInc = DMA_MINC_ENABLE;
    rxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    rxHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    rxHandle.Init.Mode = DMA_PFCTRL;
    rxHandle.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    rxHandle.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    rxHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    rxHandle.Init.MemBurst =DMA_MBURST_INC4;
    rxHandle.Init.PeriphBurst = DMA_PBURST_INC4;

    rxHandle.Instance = DMA2_Stream6;

    __HAL_LINKDMA(hsd, hdmarx, rxHandle);
    HAL_DMA_DeInit(&rxHandle);
    HAL_DMA_Init(&rxHandle);

    /* Configure DMA TX parameters. */
    txHandle.Init.Channel = DMA_CHANNEL_4;
    txHandle.Init.Direction = DMA_MEMORY_TO_PERIPH;
    txHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    txHandle.Init.MemInc = DMA_MINC_ENABLE;
    txHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    txHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    txHandle.Init.Mode = DMA_PFCTRL;
    txHandle.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    txHandle.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    txHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    txHandle.Init.MemBurst =DMA_MBURST_INC4;
    txHandle.Init.PeriphBurst = DMA_PBURST_INC4;

    txHandle.Instance = DMA2_Stream3;

    __HAL_LINKDMA(hsd, hdmatx, txHandle);
    HAL_DMA_DeInit(&txHandle);
    HAL_DMA_Init(&txHandle);

    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

/**
  * @brief  Configures Interrupt mode for SD detection pin.
  * @retval None
  */
uint8_t drv_sdioGetCardInfo(HAL_SD_CardInfoTypeDef* cardInfo, HAL_SD_CardCIDTypedef* cardCID)
{
    HAL_SD_GetCardInfo(&sdCard_Handle, cardInfo);
    HAL_SD_GetCardCID(&sdCard_Handle, cardCID);
    return 0;
}

/**
  * @brief  Gets the current SD card data status.
  * @retval Data transfer state.
  *          This value can be one of the following values:
  *            @arg  SD_TRANSFER_OK: No data transfer is acting
  *            @arg  SD_TRANSFER_BUSY: Data transfer is acting
  */
uint8_t drv_sdioGetCardState(void)
{
    return ((HAL_SD_GetCardState(&sdCard_Handle) == HAL_SD_CARD_TRANSFER) ? SD_TRANSFER_OK : SD_TRANSFER_BUSY);
}

/**
  * @brief  Reads block(s) from a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of SD blocks to read
  * @param  Timeout: Timeout for read operation
  * @retval SD status
  */
uint8_t drv_sdioReadBlocks(uint32_t *pData, uint32_t readAddr, uint32_t numOfBlocks, uint32_t timeOut)
{
    if(HAL_SD_ReadBlocks(&sdCard_Handle, (uint8_t *)pData, readAddr, 
        numOfBlocks, timeOut) != HAL_OK)
    {
        return MSD_ERROR;
    }
    else
    {
        return MSD_OK;
    }
}

/**
  * @brief  Writes block(s) to a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  writeAddr: Address from where data is to be written
  * @param  numOfBlocks: Number of SD blocks to write 
  * @retval SD status
  */
uint8_t drv_sdioWriteBlocks(uint32_t *pData, uint32_t writeAddr, uint32_t numOfBlocks)
{
    if(HAL_SD_WriteBlocks(&sdCard_Handle, (uint8_t *)pData, writeAddr, 
        numOfBlocks, 1000) != HAL_OK )
    {
        return MSD_ERROR;
    }
    else 
    {
        return MSD_OK;
    }
}

/**
  * @brief  Reads block(s) from a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of SD blocks to read
  * @param  Timeout: Timeout for read operation
  * @retval SD status
  */
uint8_t drv_sdioReadBlocks_Dma(uint32_t *pData, uint32_t readAddr, uint32_t numOfBlocks)
{
    if(HAL_SD_ReadBlocks_DMA(&sdCard_Handle, (uint8_t *)pData, readAddr, 
        numOfBlocks) != HAL_OK)
    {
        return MSD_ERROR;
    }
    else
    {
        return MSD_OK;
    }
}

/**
  * @brief  Writes block(s) to a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  writeAddr: Address from where data is to be written
  * @param  numOfBlocks: Number of SD blocks to write 
  * @retval SD status
  */
uint8_t drv_sdioWriteBlocks_Dma(uint32_t *pData, uint32_t writeAddr, uint32_t numOfBlocks)
{
    if(HAL_SD_WriteBlocks_DMA(&sdCard_Handle, (uint8_t *)pData, writeAddr, 
        numOfBlocks) != HAL_OK )
    {
        while (__HAL_SD_GET_FLAG(&sdCard_Handle, SDIO_FLAG_TXUNDERR))
        {
            log_d("SDIO FIFO Underrun!");
        }
        return MSD_ERROR;
    }
    else 
    {
        while (__HAL_SD_GET_FLAG(&sdCard_Handle, SDIO_FLAG_TXUNDERR))
        {
            log_d("SDIO FIFO Underrun!");
        }
        return MSD_OK;
    }
}

/**
  * @brief  Erases the specified memory area of the given SD card. 
  * @param  startAddr: Start byte address
  * @param  endAddr: End byte address
  * @retval SD status
  */
uint8_t drv_sdioCardErase(uint32_t startAddr, uint32_t endAddr)
{
    if(HAL_SD_Erase(&sdCard_Handle, startAddr, endAddr) != HAL_OK)
    {
        return MSD_ERROR;
    }
    else
    {
        return MSD_OK;
    }
}

/**
  * @brief Tx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
    dev_SD_WriteCpltCallback();
}

/**
  * @brief Rx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
    dev_SD_ReadCpltCallback();
}

__weak void dev_SD_WriteCpltCallback(void)
{

}

__weak void dev_SD_ReadCpltCallback(void)
{

}


// 使用原生HAL库读取函数读取第一个扇区，结果也是一样，原因待排查
// 使用原生HAL库读取函数读取第一个扇区，结果也是一样，原因待排查，原因就是发送DMA通道配置中MemInc 参数设置为DISABLE，导致读取异常
void task_sdCardReadTest(void *arg)
{
    uint8_t buffer_write[8] = { 0x00, 0xaa, 0xa2, 0x03, 0x04, 0xd5, 0x06, 0x07, };

    uint8_t buffer_read[512];
    HAL_StatusTypeDef status;
    BYTE lun;

    dev_SD_initialize(lun);
    
    status = HAL_SD_WriteBlocks_DMA(&sdCard_Handle, buffer_write, 55, 1);
    if (status != HAL_OK)
    {
        log_e("Read Block 0 Failed! Error: %lu\r\n", HAL_SD_GetError(&sdCard_Handle));
    }
    else
    {
        for (int i = 0; i < 8; i++) 
        {
            log_d("%02X ", buffer_write[i]);
        }
    }

    // osDelay(3);

    status = HAL_SD_ReadBlocks_DMA(&sdCard_Handle, buffer_read, 0, 1);
    if (status != HAL_OK) 
    {
        log_e("Read Block 0 Failed! Error: %lu\r\n", HAL_SD_GetError(&sdCard_Handle));
    }
    else 
    {
        log_e("read success.");
    }

    // while (HAL_SD_GetCardState(&sdCard_Handle) != HAL_SD_CARD_TRANSFER);
    // 打印前 16 字节（用于检查 MBR/FAT32 结构）
    log_d("Boot Sector First 16 Bytes:\n");
    for (int i = 0; i < 16; i++) {

        log_d("%02X ", buffer_read[i]);
    }

    // 检查引导扇区签名 (0x55AA)
    if (buffer_read[510] == 0x55 && buffer_read[511] == 0xAA) 
    {
        log_d("Valid Boot Sector Found! (Signature 0x%02X%02X)", buffer_read[510], buffer_read[511]);
    }
    else 
    {
        log_d("Invalid Boot Sector (Signature 0x%02X%02X)", buffer_read[510], buffer_read[511]);
    }

    for(;;)
    {
        // osDelay(1000);
    }
}
