/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Private defines */
#define USE_CAN1 0
#define USE_CAN2 1
/* USER CODE END Private defines */

#if USE_CAN1
extern CAN_HandleTypeDef hcan1;
void drv_can1Init(void);
#endif

#if USE_CAN2
extern CAN_HandleTypeDef hcan2;
void drv_can2Init(void);
#endif

void drv_canEnableReceiveInt(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

