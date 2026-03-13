#include "drv_emmc.h"
#include "dev_emmc.h"

MMC_HandleTypeDef emmc_handle;
DMA_HandleTypeDef rxHandle;
DMA_HandleTypeDef txHandle;

uint8_t drv_emmcInit(void) 
{
    uint8_t emmc_state = EMMC_OK;
    
    emmc_handle.Instance = SDIO;
    emmc_handle.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    emmc_handle.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE; 
    emmc_handle.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    emmc_handle.Init.BusWide = SDIO_BUS_WIDE_1B;                              // use one bit data line
    emmc_handle.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    emmc_handle.Init.ClockDiv = 1;                            // 初始化时钟

    if(HAL_MMC_Init(&emmc_handle) != HAL_OK) 
    {
        emmc_state = EMMC_ERROR; 
    }

    if(emmc_state == EMMC_OK) 
    {
        if(HAL_MMC_ConfigWideBusOperation(&emmc_handle, SDIO_BUS_WIDE_8B) != HAL_OK) 
        {
            emmc_state = EMMC_ERROR;
        } 
        else 
        {
            emmc_state = EMMC_OK;
        }
    }
    return emmc_state;
}

void HAL_MMC_MspInit(MMC_HandleTypeDef *hmmc) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    __HAL_RCC_SDMMC1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Alternate = GPIO_AF12_SDIO;

    GPIO_InitStructure.Pin = EMMC_CLK_PIN | EMMC_D0_PIN | EMMC_D1_PIN | EMMC_D2_PIN | EMMC_D3_PIN | EMMC_D6_PIN | EMMC_D7_PIN;
    HAL_GPIO_Init(EMMC_CLK_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = EMMC_D4_PIN | EMMC_D5_PIN;
    HAL_GPIO_Init(EMMC_D4_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = EMMC_CMD_PIN;
    HAL_GPIO_Init(EMMC_CMD_PORT, &GPIO_InitStructure);

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
    HAL_DMA_Init(&rxHandle);
    __HAL_LINKDMA(hmmc, hdmarx, rxHandle);
    
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
    HAL_DMA_Init(&txHandle);
    __HAL_LINKDMA(hmmc, hdmatx, txHandle);

    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

uint8_t drv_emmcGetInfo(HAL_MMC_CardInfoTypeDef* emmcInfo, HAL_MMC_CardCIDTypeDef* emmcCID) 
{
    HAL_MMC_GetCardInfo(&emmc_handle, emmcInfo);
    HAL_MMC_GetCardCID(&emmc_handle, emmcCID);

    return 0;
}

uint8_t drv_emmcReadBlocks(uint32_t *pData, uint32_t readAddr, uint32_t numOfBlocks) 
{
    if(HAL_MMC_ReadBlocks_DMA(&emmc_handle, (uint8_t *)pData, readAddr, numOfBlocks) != HAL_OK) 
    {
        return EMMC_ERROR;
    }
    else
    {
        return EMMC_OK;
    }
}

uint8_t drv_emmcWriteBlocks(uint32_t *pData, uint32_t writeAddr, uint32_t numOfBlocks) 
{
    if(HAL_MMC_WriteBlocks_DMA(&emmc_handle, (uint8_t *)pData, writeAddr, numOfBlocks) != HAL_OK) 
    {
        return EMMC_ERROR;
    } 
    else 
    {
        return EMMC_OK;
    }
}

uint8_t drv_emmcErase(uint32_t startAddr, uint32_t endAddr) 
{
    if(HAL_MMC_Erase(&emmc_handle, startAddr, endAddr) != HAL_OK) 
    {
        return EMMC_ERROR;
    } 
    else 
    {
        return EMMC_OK;
    }
}

uint8_t drv_emmcGetState(void) 
{
    return ((HAL_MMC_GetCardState(&emmc_handle) == HAL_MMC_CARD_TRANSFER) ? EMMC_TRANSFER_OK : EMMC_TRANSFER_BUSY);
}

void HAL_MMC_TxCpltCallback(MMC_HandleTypeDef *hmmc) 
{
    UNUSED(hmmc);
    dev_eMMC_WriteCpltCallback();
}

void HAL_MMC_RxCpltCallback(MMC_HandleTypeDef *hmmc)
{
    UNUSED(hmmc);
    dev_eMMC_ReadCpltCallback();
}
