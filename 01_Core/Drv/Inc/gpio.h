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
#define IO_OUTPUT_CLOSE()                                                \
        HAL_GPIO_WritePin(UwbOK_LED_GPIO_Port, UwbOK_LED_Pin, GPIO_PIN_RESET);    \
        HAL_GPIO_WritePin(Onside_LED_GPIO_Port, Onside_LED_Pin, GPIO_PIN_RESET);  \
        HAL_GPIO_WritePin(Across_LED_GPIO_Port, Across_LED_Pin, GPIO_PIN_RESET);  \
        HAL_GPIO_WritePin(CAN_TX_LED_GPIO_Port, CAN_TX_LED_Pin, GPIO_PIN_RESET);  \
        HAL_GPIO_WritePin(CAN_RX_LED_GPIO_Port, CAN_RX_LED_Pin, GPIO_PIN_RESET);  \
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET)             \

#define SW_IS_ON(sw_gpio_port, sw_pin)    return ((HAL_GPIO_ReadPin(sw_gpio_port, sw_pin)) ? (0) : (1)
/* USER CODE END Private defines */

typedef enum gpio_enum_e {
    dw1000_irq,
    dw1000_rst,
    uwb_ok_led,
    switch_key,
    pause_key,
    onside_led,
    across_led,
    buzzer,
    can_tx_led,
    can_rx_led,
    sw0,
    sw1,
    sw2,    
    gpio_num
} gpio_enum_t;

typedef enum exti_irq_e {
    dw1000_interrupt,
    exti_num
} exti_irq_enum_t;

typedef struct gpio_config_s {
    GPIO_TypeDef* gpio_port;
    uint32_t      gpio_pin;
    uint32_t      gpio_mode;
    uint32_t      gpio_pull;
    uint32_t      gpio_speed;
} gpio_config_t;

typedef struct exti_irq_s{
    IRQn_Type irq_name;
    uint32_t irq_priority;
} exti_irq_t;

void MX_GPIO_Init(void);
void gpioAndExti_init(void);

/* USER CODE BEGIN Prototypes */
void pause_key_init(void);
void switch_key_init(void);

void led_toggle (gpio_enum_t led);
void led_on (gpio_enum_t led);
void led_off (gpio_enum_t led);

uint8_t read_SwitchValue(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

