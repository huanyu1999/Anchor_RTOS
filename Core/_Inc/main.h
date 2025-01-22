/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Dw1000_IRQ_Pin          GPIO_PIN_13
#define Dw1000_IRQ_GPIO_Port    GPIOC
#define Dw1000_IRQ_EXTI_IRQn    EXTI15_10_IRQn

#define Dw1000_RSTn_Pin         GPIO_PIN_14
#define Dw1000_RSTn_GPIO_Port   GPIOC

#define UwbOK_LED_Pin           GPIO_PIN_9
#define UwbOK_LED_GPIO_Port     GPIOB

#define S_KEY_Pin               GPIO_PIN_1
#define S_KEY_GPIO_Port         GPIOB

#define P_KEY_Pin               GPIO_PIN_0
#define P_KEY_GPIO_Port         GPIOB

#define Onside_LED_Pin          GPIO_PIN_6
#define Onside_LED_GPIO_Port    GPIOC

#define Across_LED_Pin          GPIO_PIN_7
#define Across_LED_GPIO_Port    GPIOC

#define BUZZER_Pin              GPIO_PIN_8
#define BUZZER_GPIO_Port        GPIOA

#define CAN_TX_LED_Pin          GPIO_PIN_3
#define CAN_TX_LED_GPIO_Port    GPIOC
#define CAN_RX_LED_Pin          GPIO_PIN_2
#define CAN_RX_LED_GPIO_Port    GPIOA

#define SW0_Pin                 GPIO_PIN_0
#define SW0_GPIO_Port           GPIOC
#define SW1_Pin                 GPIO_PIN_1
#define SW1_GPIO_Port           GPIOC
#define SW2_Pin                 GPIO_PIN_2
#define SW2_GPIO_Port           GPIOC

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
