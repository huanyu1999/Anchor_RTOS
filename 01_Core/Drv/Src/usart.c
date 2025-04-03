/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"
#include <stdio.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_tx;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_uart1_tx;

/* UART4 init function */
void MX_UART4_Init(void)
{
    /* USER CODE BEGIN UART4_Init 0 */

    /* USER CODE END UART4_Init 0 */

    /* USER CODE BEGIN UART4_Init 1 */

    /* USER CODE END UART4_Init 1 */
    huart4.Instance        = UART4;
    huart4.Init.BaudRate   = 9600;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits   = UART_STOPBITS_1;
    huart4.Init.Parity     = UART_PARITY_NONE;
    huart4.Init.Mode       = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN UART4_Init 2 */

    /* USER CODE END UART4_Init 2 */
}

/* UART1 init function */
void MX_UART1_Init(void)
{
    /* USER CODE BEGIN UART4_Init 0 */

    /* USER CODE END UART4_Init 0 */

    /* USER CODE BEGIN UART4_Init 1 */

    /* USER CODE END UART4_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN UART4_Init 2 */

    /* USER CODE END UART4_Init 2 */
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(uartHandle->Instance == UART4)
    {
        /* USER CODE BEGIN UART4_MspInit 0 */

        /* USER CODE END UART4_MspInit 0 */
        /* UART4 clock enable */
        __HAL_RCC_UART4_CLK_ENABLE();
        
        /* DMA controller clock enable */
        __HAL_RCC_DMA1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**UART4 GPIO Configuration
        PA0-WKUP     ------> UART4_TX
        PA1     ------> UART4_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* UART4 DMA Init */
        /* UART4_TX Init */
        hdma_uart4_tx.Instance = DMA1_Stream4;
        hdma_uart4_tx.Init.Channel = DMA_CHANNEL_4;
        hdma_uart4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_uart4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_uart4_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_uart4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_uart4_tx.Init.Mode = DMA_NORMAL;
        hdma_uart4_tx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_uart4_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_uart4_tx) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(uartHandle,hdmatx,hdma_uart4_tx);

        /* UART4 interrupt Init */
        HAL_NVIC_SetPriority(UART4_IRQn, 7, 0);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
        /* DMA interrupt init */
        /* DMA1_Stream4_IRQn interrupt configuration */
        HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 8, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    }
    else if (uartHandle->Instance == USART1)
    {
        /* USART1 clock enable */
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**UART4 GPIO Configuration
        PA9      ------> UART4_TX
        PA10     ------> UART4_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* UART4 DMA Init */
        /* UART4_TX Init */
        hdma_uart1_tx.Instance       = DMA2_Stream7;
        hdma_uart1_tx.Init.Channel   = DMA_CHANNEL_4;
        hdma_uart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_uart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_uart1_tx.Init.MemInc    = DMA_MINC_ENABLE;
        hdma_uart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_uart1_tx.Init.Mode      = DMA_NORMAL;
        hdma_uart1_tx.Init.Priority  = DMA_PRIORITY_HIGH;
        hdma_uart1_tx.Init.FIFOMode  = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_uart1_tx) != HAL_OK)
        {
            Error_Handler();
        }
        __HAL_LINKDMA(uartHandle, hdmatx, hdma_uart1_tx);
        HAL_NVIC_SetPriority(USART1_IRQn, 7, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);  
        
        HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 8, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

    if(uartHandle->Instance == UART4)
    {
        /* USER CODE BEGIN UART4_MspDeInit 0 */

        /* USER CODE END UART4_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_UART4_CLK_DISABLE();

        /**UART4 GPIO Configuration
        PA0-WKUP     ------> UART4_TX
        PA1     ------> UART4_RX
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);

        /* UART4 DMA DeInit */
        HAL_DMA_DeInit(uartHandle->hdmatx);

        /* UART4 interrupt Deinit */
        HAL_NVIC_DisableIRQ(UART4_IRQn);
        /* USER CODE BEGIN UART4_MspDeInit 1 */

        /* USER CODE END UART4_MspDeInit 1 */
    }
    else if(uartHandle->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
        HAL_DMA_DeInit(uartHandle->hdmatx);
        HAL_NVIC_DisableIRQ(USART1_IRQn);
    }
}

/* debug */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);  
    return (ch);
}


/* USER CODE END 1 */
