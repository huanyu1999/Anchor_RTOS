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
#include "usart.h"
#include "gpio.h"
#include "can.h"
#include <stdio.h>
#include <string.h>

static void CAN_FILTER_CONFIG(void);

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

    /* USER CODE BEGIN CAN1_Init 0 */

    /* USER CODE END CAN1_Init 0 */

    /* USER CODE BEGIN CAN1_Init 1 */

    /* USER CODE END CAN1_Init 1 */
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
    
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();        /* Notification Error */
    }

    /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(canHandle->Instance==CAN1)
    {
        /* USER CODE BEGIN CAN1_MspInit 0 */

        /* USER CODE END CAN1_MspInit 0 */
        /* CAN1 clock enable */
        __HAL_RCC_CAN1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**CAN1 GPIO Configuration
            PA11     ------> CAN1_RX
            PA12     ------> CAN1_TX
        */
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
        HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
        /* USER CODE END CAN1_MspInit 1 */
        
    }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
  * @brief  
  * @param  
  *         
  * @retval 
  */
void canSendMsg(uint32_t extId, uint8_t* data, uint32_t length)
{
    CAN_TxHeaderTypeDef *txHeader;
    uint32_t TxMailbox = 0;
    
    txHeader->ExtId = extId;                // 使用扩展ID
    txHeader->IDE = CAN_ID_EXT;
    txHeader->RTR = CAN_RTR_DATA;
    txHeader->DLC = length;
    txHeader->TransmitGlobalTime = DISABLE;
    
    if(HAL_CAN_AddTxMessage(&hcan1, txHeader, data, &TxMailbox) == HAL_OK)
    {
        led_toggle(can_tx_led);
    } 
    else 
    {
        led_off(can_tx_led);
    }
}

/**
  * @brief  
  * @param  
  *         
  * @retval 
  */
void canPollingRxMsg(void)
{
    uint8_t recv_data[8];
    
    CAN_RxHeaderTypeDef *rxHeader;
    while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0)
    {
        if(__HAL_CAN_GET_FLAG(&hcan1, CAN_FLAG_FOV0) !=RESET) 
        {
            
        }
        
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, rxHeader, recv_data);

        printf_use_dma( "ExtId ID:%d\n",rxHeader->ExtId);
        printf_use_dma( "CAN IDE:0x%x\n",rxHeader->IDE);
        printf_use_dma( "CAN RTR:0x%x\n",rxHeader->RTR);
        printf_use_dma( "CAN DLC:0x%x\n",rxHeader->DLC);
        printf_use_dma( "RECV DATA:");
        for(int i = 0; i < rxHeader->DLC; i++)
        {
            printf_use_dma( "0x%x ",recv_data[i]);
        }
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
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.SlaveStartFilterBank = 14;
    
    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

}

/* USER CODE END 1 */
