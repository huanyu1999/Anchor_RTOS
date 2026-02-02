/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "board_dw1000.h"
#include "board_w5500.h"
#include "stm32f4xx_hal_pcd.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"
#include "elog.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
// extern TIM_HandleTypeDef  htimer2;
extern TIM_HandleTypeDef  htimer6;
extern CAN_HandleTypeDef  hcan1;
extern PCD_HandleTypeDef  hpcd;
extern MMC_HandleTypeDef  emmc_handle;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

void TIM2_IRQHandler(void)
{
    extern TIM_HandleTypeDef timerForInvaildDistanceClearHandle;
    HAL_TIM_IRQHandler(&timerForInvaildDistanceClearHandle);
}

void TIM3_IRQHandler(void)
{
    extern TIM_HandleTypeDef button_tickHandler;
    HAL_TIM_IRQHandler(&button_tickHandler);
}

#ifdef TASK_DEBUG_INFO
void TIM4_IRQHandler(void)
{
    extern TIM_HandleTypeDef timer50usHandle;
    HAL_TIM_IRQHandler(&timer50usHandle);
}
#endif

void TIM5_IRQHandler(void)
{
    extern TIM_HandleTypeDef dhcp_oneSecondHandle;
    HAL_TIM_IRQHandler(&dhcp_oneSecondHandle);
}

/**
  * @brief This function handles CAN1 RX0 interrupt request.
  */
void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI2_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(Dw1000_IRQ_Pin); 
}

void EXTI4_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(Dw1000_RSTn_Pin);
}

void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(W5500_INT_PIN);
}

void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{        
    HAL_UART_IRQHandler(&huart4);   
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart4);
        HAL_UART_IdleCallback(&huart4);
        // log_d("HAL_UART_IdleCallback");
    }                                 
}

void HAL_UART_IdleCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        // 释放一个信号量
        extern osSemaphoreId_t gnssReceiveSem;
        osSemaphoreRelease(gnssReceiveSem);
    }
}

void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htimer6);
}

void SDIO_IRQHandler(void)
{
    HAL_MMC_IRQHandler(&emmc_handle);
}

void DMA2_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(emmc_handle.hdmatx);
}

void DMA2_Stream6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(emmc_handle.hdmarx);
}

#if USE_SPI1_DMA
extern SPI_HandleTypeDef hspi1;
void DMA2_Stream5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(hspi1.hdmatx);
}

void DMA2_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(hspi1.hdmarx);
}
#endif 

void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd);
}
