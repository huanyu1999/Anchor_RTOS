/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "spi.h"
#include "cmsis_os.h"

#define USE_SPI1_DMA 1

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

/* SPI1 init function */
void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }

}

/* SPI2 init function */
void MX_SPI2_Init(void)
{
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;

    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
}

void drv_spiCSCtrl(SPI_HandleTypeDef* spiHandle, GPIO_PinState level)
{
    if(spiHandle->Instance == SPI1)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, level);
    }
    else if (spiHandle->Instance == SPI2)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, level);
    }
}

void drv_spiWriteBytes(uint8_t* data, uint32_t length)
{
    HAL_SPI_Transmit(&hspi2, data, length, 0xFFFF);
}

uint8_t drv_spiReadByte(void)
{
    uint8_t data[2] = {0xFF, 0xFF};
    // HAL_SPI_Receive(&hspi2, data, length, 0xFFFF);
    HAL_SPI_TransmitReceive(&hspi2, data, data + 1, 1, 0xFFFF);
    
    return data[1];
}

void drv_spiReadBytes(uint8_t* data, uint32_t length)
{
    // uint8_t data[2] = {0xFF, 0xFF};
    HAL_SPI_Receive(&hspi2, data, length, 0xFFFF);
    // HAL_SPI_TransmitReceive(&hspi2, data, data + 1, 1, 0xFFFF);
}

/* weak function */
void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(spiHandle->Instance == SPI1)
    {
        /* SPI1 clock enable */
        __HAL_RCC_SPI1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();

        /**SPI1 GPIO Configuration
        PA4     ------> SPI1_CS
        PA5     ------> SPI1_SCK
        PA6     ------> SPI1_MISO
        PA7     ------> SPI1_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

#ifdef USE_SPI1_DMA
        __HAL_RCC_DMA2_CLK_ENABLE();
       /* USER CODE BEGIN SPI1_MspInit 1 */
       /* SPI1_TX DMA Init */
        hdma_spi1_tx.Instance                 = DMA2_Stream5;
        hdma_spi1_tx.Init.Channel             = DMA_CHANNEL_3;
        hdma_spi1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_spi1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_spi1_tx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_spi1_tx.Init.Mode                = DMA_NORMAL;
        hdma_spi1_tx.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_spi1_tx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
        hdma_spi1_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
        hdma_spi1_tx.Init.MemBurst            = DMA_MBURST_INC8;
        hdma_spi1_tx.Init.PeriphBurst         = DMA_MBURST_INC8;
        hdma_spi1_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;

        HAL_DMA_Init(&hdma_spi1_tx);
        __HAL_LINKDMA(spiHandle, hdmatx, hdma_spi1_tx);

        /* SPI1_RX DMA Init */
        hdma_spi1_rx.Instance                 = DMA2_Stream2;
        hdma_spi1_rx.Init.Channel             = DMA_CHANNEL_3;
        hdma_spi1_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_spi1_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_spi1_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi1_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_spi1_rx.Init.Mode                = DMA_NORMAL;
        hdma_spi1_rx.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_spi1_rx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;         
        hdma_spi1_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
        hdma_spi1_rx.Init.MemBurst            = DMA_MBURST_INC8;
        hdma_spi1_rx.Init.PeriphBurst         = DMA_MBURST_INC8;
        hdma_spi1_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        
        HAL_DMA_Init(&hdma_spi1_rx);
        __HAL_LINKDMA(spiHandle, hdmarx, hdma_spi1_rx);
        
        /* NVIC configuration for DMA transfer complete interrupt (SPI1_TX) */
        HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0x05, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
        
        /* NVIC configuration for DMA transfer complete interrupt (SPI1_RX) */
        HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0x05, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
#endif
        /* USER CODE END SPI1_MspInit 1 */
    }
    else if(spiHandle->Instance==SPI2)
    {
        /* SPI2 clock enable */
        __HAL_RCC_SPI2_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();

        /**SPI2 GPIO Configuration
        PB12     ------> SPI2_CS
        PB14     ------> SPI2_MISO
        PB15     ------> SPI2_MOSI
        PB13     ------> SPI2_SCK
        */
        GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        // GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/* weak function */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{
    static DMA_HandleTypeDef hdma_tx;
    static DMA_HandleTypeDef hdma_rx;

    if(spiHandle->Instance==SPI1)
    {
        /* Peripheral clock disable */
        __HAL_RCC_SPI1_CLK_DISABLE();

        /**SPI1 GPIO Configuration
        PA5     ------> SPI1_SCK
        PA6     ------> SPI1_MISO
        PA7     ------> SPI1_MOSI
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

        /* USER CODE BEGIN SPI1_MspDeInit 1 */

        /* USER CODE END SPI1_MspDeInit 1 */
    }
    else if(spiHandle->Instance==SPI2)
    {
        /* Peripheral clock disable */
        __HAL_RCC_SPI2_CLK_DISABLE();

        /**SPI2 GPIO Configuration
        PB14     ------> SPI2_MISO
        PB15     ------> SPI2_MOSI
        PB13     ------> SPI2_SCK
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_14 | GPIO_PIN_13 | GPIO_PIN_15);

        /* USER CODE BEGIN SPI2_MspDeInit 1 */

        /* De-Initialize the DMA Stream associate to transmission process */
        HAL_DMA_DeInit(&hdma_tx);
            
        /* De-Initialize the DMA Stream associate to reception process */
        HAL_DMA_DeInit(&hdma_rx);

        // HAL_NVIC_DisableIRQ(DMA2_Stream3_IRQn);
        // HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
    }
}

extern osSemaphoreId_t dw1000WriteSem;
extern osSemaphoreId_t dw1000ReadSem;

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    UNUSED(hspi);
    osSemaphoreRelease(dw1000ReadSem);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    UNUSED(hspi);
    osSemaphoreRelease(dw1000WriteSem);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    UNUSED(hspi);
}
