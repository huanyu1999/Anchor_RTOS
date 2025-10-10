/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
//#define IO_OUTPUT_CLOSE()                                                \
//        HAL_GPIO_WritePin(UwbOK_LED_GPIO_Port, UwbOK_LED_Pin, GPIO_PIN_RESET);    \
//        HAL_GPIO_WritePin(Onside_LED_GPIO_Port, Onside_LED_Pin, GPIO_PIN_RESET);  \
//        HAL_GPIO_WritePin(Across_LED_GPIO_Port, Across_LED_Pin, GPIO_PIN_RESET);  \
//        HAL_GPIO_WritePin(CAN_TX_LED_GPIO_Port, CAN_TX_LED_Pin, GPIO_PIN_RESET);  \
//        HAL_GPIO_WritePin(CAN_RX_LED_GPIO_Port, CAN_RX_LED_Pin, GPIO_PIN_RESET);  \
//        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET)             \

//#define SW_IS_ON(sw_gpio_port, sw_pin)    return ((HAL_GPIO_ReadPin(sw_gpio_port, sw_pin)) ? (0) : (1)
/* USER CODE END Private defines */

typedef enum {
    GPIO_PORT_A,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,
    GPIO_PORT_E,
    GPIO_PORT_F,
    GPIO_PORT_G,
    GPIO_PORT_H,
    GPIO_PORT_I,
    GPIO_PORT_NUM
} gpio_clkPort_t;

typedef enum gpio_pin_e {
    PIN_0,
    PIN_1,
    PIN_2,
    PIN_3,
    PIN_4,
    PIN_5,
    PIN_6,
    PIN_7,
    PIN_8,
    PIN_9,
    PIN_10,
    PIN_11,
    PIN_12,
    PIN_13,
    PIN_14s,
    PIN_15,
    PIN_NUM
} gpio_pin_t;

typedef struct gpio_config_s {
    GPIO_TypeDef* gpio_port;
    gpio_clkPort_t clk_port;
    uint32_t      gpio_pin;
    uint32_t      gpio_mode;
    uint32_t      gpio_pull;
    uint32_t      gpio_speed;
} gpio_config_t;

typedef struct {
    IRQn_Type irq_name;
    uint32_t irq_priority;
} exti_irq_t;

void drv_gpioInit(gpio_config_t *io_cfg);
void drv_extiInit(exti_irq_t *exti_cfg);
void drv_gpioSetLevel(gpio_config_t *io_cfg, uint8_t pin_level);
uint8_t drv_gpioGetLevel(gpio_config_t *io_cfg);
void drv_gpioToggleLevel(gpio_config_t *io_cfg);
void drv_gpioClose(gpio_config_t *io_cfg);
void drv_gpioClkEn(gpio_clkPort_t io_port);

/* USER CODE BEGIN Prototypes */
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

