#include <stdint.h>

#include "main.h"
#include "stm32f405xx.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_ll_sdmmc.h"
#include "drv_sdio.h"

SD_HandleTypeDef sdCard_Handle;

uint8_t drv_sdio_init(void)
{
    uint8_t sd_state = SDIO_OK;

    sdCard_Handle.Instance = SDIO;
    sdCard_Handle.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    sdCard_Handle.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE; 
    sdCard_Handle.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    sdCard_Handle.Init.BusWide = SDIO_BUS_WIDE_4B;                              // use four bit data line
    sdCard_Handle.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    sdCard_Handle.Init.ClockDiv = SDIO_INIT_CLK_DIV;                            // 初始化时钟

    if(HAL_SD_Init(&sdCard_Handle) != HAL_OK)
    {
        sd_state = SDIO_ERROR;
        Error_Handler();
        HAL_SD_InitCard();
    }

    if(sd_state == SDIO_OK) 
    {
        if(HAL_SD_ConfigWideBusOperation(&sdCard_Handle, SDIO_BUS_WIDE_4B) != HAL_OK)
        {
            sd_state = SDIO_ERROR;
        }
        else 
        {
            sd_state = SDIO_OK;
        }
    }
    return sd_state;
}

void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
{
    static DMA_HandleTypeDef rxHandle;
    static DMA_HandleTypeDef txHandle;
    GPIO_InitTypeDef GPIO_InitStructure;

    __HAL_RCC_SDIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Alternate = GPIO_AF12_SDIO;
    GPIO_InitStructure.Pin = SDIO_CLK_PIN | SDIO_D0_PIN | SDIO_D1_PIN | SDIO_D2_PIN | SDIO_D3_PIN;
    HAL_GPIO_Init(SDIO_CLK_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = SDIO_CMD_PIN;
    HAL_GPIO_Init(SDIO_CMD_PORT, &GPIO_InitStructure);

    /* NVIC configuration for SDIO interrupts. */
    HAL_NVIC_SetPriority(SDIO_IRQn, 0x0e, 0);
    HAL_NVIC_EnableIRQ(SDIO_IRQn);
    
    /* Configure DMA RX parameters. */
    rxHandle.Init.Channel = DMA_CHANNEL_4;
    rxHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    rxHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    rxHandle.Init.MemInc = DMA_MINC_DISABLE;
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
    rxHandle.Init.Channel = DMA_CHANNEL_4;
    rxHandle.Init.Direction = DMA_MEMORY_TO_PERIPH;
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

    rxHandle.Instance = DMA2_Stream3;

    __HAL_LINKDMA(hsd, hdmatx, txHandle);
    HAL_DMA_DeInit(&txHandle);
    HAL_DMA_Init(&txHandle);

    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

uint8_t drv_sdioGetCardInfo(HAL_SD_CardInfoTypeDef* cardInfo)
{
    HAL_SD_GetCardInfo(&sdCard_Handle, cardInfo);
    return 0;
}

uint8_t drv_sdioGetCardState(void)
{
    return ((HAL_SD_GetCardState(&sdCard_Handle) == HAL_SD_CARD_TRANSFER) ? SDIO_TRANS_OK : SDIO_TRANS_BUSY);
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
        return SDIO_ERROR;
    }
    else {
        return SDIO_OK;
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
        return SDIO_ERROR;
    }
    else 
    {
        return SDIO_OK;
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
uint8_t drv_sdioReadBlocks_Dma(uint32_t *pData, uint32_t readAddr, uint32_t numOfBlocks, uint32_t timeOut)
{
    if(HAL_SD_ReadBlocks_DMA(&sdCard_Handle, (uint8_t *)pData, readAddr, 
        numOfBlocks) != HAL_OK)
    {
        return SDIO_ERROR;
    }
    else
    {
        return SDIO_OK;
    }
}

/**
  * @brief  Writes block(s) to a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  writeAddr: Address from where data is to be written
  * @param  numOfBlocks: Number of SD blocks to write 
  * @retval SD status
  */
uint8_t drv_sdioWriteBlocks_Dma(uint8_t *pData, uint32_t writeAddr, uint32_t numOfBlocks)
{
    if(HAL_SD_WriteBlocks_DMA(&sdCard_Handle, (uint8_t *)pData, writeAddr, 
        numOfBlocks) != HAL_OK )
    {
        return SDIO_ERROR;
    }
    else 
    {
        return SDIO_OK;
    };
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
        return SDIO_ERROR;
    }
    else
    {
        return SDIO_OK;
    }
}
