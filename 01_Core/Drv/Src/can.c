/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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

/* USER CODE BEGIN 0 */

#include "can.h"
#include "gpio.h"
#include "elog.h"
#include <stdio.h>
#include <string.h>

static void CAN_FILTER_CONFIG(void);

/* USER CODE END 0 */

#if USE_CAN1

CAN_HandleTypeDef hcan1;

void drv_can1Init(void)
{
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 12;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = ENABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* USER CODE BEGIN CAN1_Init 2 */
    CAN_FILTER_CONFIG();

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();        /* Start Error */
    }
    /* USER CODE END CAN1_Init 2 */
}

#endif 

#if USE_CAN2
CAN_HandleTypeDef hcan2;
void drv_can2Init(void)
{
    hcan2.Instance = CAN2;
    hcan2.Init.Prescaler = 3;
    hcan2.Init.Mode = CAN_MODE_NORMAL;
    hcan2.Init.SyncJumpWidth = CAN_SJW_2TQ;
    hcan2.Init.TimeSeg1 = CAN_BS1_12TQ;
    hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
    hcan2.Init.TimeTriggeredMode = DISABLE;
    hcan2.Init.AutoBusOff = ENABLE;
    hcan2.Init.AutoWakeUp = ENABLE;
    hcan2.Init.AutoRetransmission = ENABLE;
    hcan2.Init.ReceiveFifoLocked = DISABLE;
    hcan2.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* USER CODE BEGIN CAN1_Init 2 */
    CAN_FILTER_CONFIG();

    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        Error_Handler();        /* Start Error */
    }
    
}
#endif

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(canHandle->Instance == CAN1)
    {
        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        
        /* CAN1 GPIO Configuration PA11     ------> CAN1_RX PA12     ------> CAN1_TX */
        GPIO_InitStruct.Pin   =  GPIO_PIN_12;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull  = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USER CODE BEGIN CAN1_MspInit 1 */
        GPIO_InitStruct.Pin = GPIO_PIN_11;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        
        /* NVIC configuration for CAN1 Reception complete interrupt */
        HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
        /* USER CODE END CAN1_MspInit 1 */
    }
    else if (canHandle->Instance == CAN2)
    {
        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_CAN2_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        
        /* CAN2 GPIO Configuration PB5 ------> CAN2_RX   PB6 ------> CAN2_TX */
        GPIO_InitStruct.Pin   =  GPIO_PIN_5;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull  = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USER CODE BEGIN CAN1_MspInit 1 */
        GPIO_InitStruct.Pin = GPIO_PIN_6;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull  = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        
        /* NVIC configuration for CAN1 Reception complete interrupt */
        HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
        /* USER CODE END CAN1_MspInit 1 */
    }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

    if(canHandle->Instance==CAN1)
    {

        /* Peripheral clock disable */
        __HAL_RCC_CAN1_CLK_DISABLE();

        // HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6);
    }
}

/* USER CODE BEGIN 1 */
void drv_canEnableReceiveInt(void)
{
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        log_e("rx interrupt enable failed.");
    }
}

/**
  * @brief  can filter config
  * @param  none 
  * @retval none
  */
static void CAN_FILTER_CONFIG(void)
{
    CAN_FilterTypeDef sFilterConfig = {0};

    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterBank = 14;
    sFilterConfig.SlaveStartFilterBank = 14;
    
    if(HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

}

/* USER CODE END 1 */
