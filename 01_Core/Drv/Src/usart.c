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
DMA_HandleTypeDef hdma_uart4_rx;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart3_tx;

/* UART4 init function */
void MX_UART4_Init(void)
{
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
}

void MX_UART3_Init(void)
{
    huart3.Instance        = USART3;
    huart3.Init.BaudRate   = 9600;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits   = UART_STOPBITS_1;
    huart3.Init.Parity     = UART_PARITY_NONE;
    huart3.Init.Mode       = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(uartHandle->Instance == UART4)      
    {
        /* UART4 clock enable */
        __HAL_RCC_UART4_CLK_ENABLE();
        
        __HAL_RCC_GPIOA_CLK_ENABLE();

        __HAL_RCC_DMA1_CLK_ENABLE();
        
        /* UART4 GPIO Configuration PA0-WKUP ------> UART4_TX PA1 ------> UART4_RX */
        GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        
        hdma_uart4_rx.Instance = DMA1_Stream2;
        hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
        hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_uart4_rx.Init.MemInc    = DMA_MINC_ENABLE;
        hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart4_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_uart4_rx.Init.Mode      = DMA_CIRCULAR;
        hdma_uart4_rx.Init.Priority  = DMA_PRIORITY_HIGH;
        hdma_uart4_rx.Init.FIFOMode  = DMA_FIFOMODE_DISABLE;
        hdma_uart4_rx.Init.FIFOThreshold  = DMA_FIFO_THRESHOLD_FULL;    // 不使用FIFO Mode的情况下，这个配置无效
        hdma_uart4_rx.Init.MemBurst       = DMA_MBURST_INC4;            // 不使用FIFO Mode的情况下，这个配置无效
        hdma_uart4_rx.Init.PeriphBurst    = DMA_PBURST_INC4;            // 不使用FIFO Mode的情况下，这个配置无效
        
        HAL_DMA_Init(&hdma_uart4_rx);
        
        __HAL_LINKDMA(uartHandle, hdmarx, hdma_uart4_rx);

        __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
        
        /* UART4 interrupt Init */
        HAL_NVIC_SetPriority(UART4_IRQn, 7, 0);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
        // HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 8, 0);
        // HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
    }
    else if (uartHandle->Instance == USART3)
    {
        /* USART3 clock enable */
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        
        /* USART3 GPIO Configuration PB10 ------> USART3_TX PB11 ------> USART3_RX */
        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART3_IRQn, 7, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);  
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
    if(uartHandle->Instance == UART4)
    {
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
    }
    else if(uartHandle->Instance == USART3)
    {
        __HAL_RCC_USART3_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11 | GPIO_PIN_10);
        HAL_NVIC_DisableIRQ(USART3_IRQn);
    }
}
/* USER CODE END 1 */
