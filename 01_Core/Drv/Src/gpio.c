/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"
#include "instance.h"
/* USER CODE BEGIN 0 */
#include "com_multiButton.h"
#include "usart.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*
    dw1000_irq  PC13
    dw1000_rst  PC14
    uwb_ok_led  PB9

    onside_led  PC6
    across_led  PC7
    buzzer      PA8
    can_tx_led  PA12 
    can_rx_led  PA2
    sw0         PC0
    sw1         PC1
    sw2         PC2
*/
/*----------------------------------------------------------------------------*/

void drv_gpioInit(gpio_config_t *io_cfg)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    drv_gpioClkEn(io_cfg->clk_port);

    GPIO_InitStruct.Pin   = io_cfg->gpio_pin;
    GPIO_InitStruct.Mode  = io_cfg->gpio_mode;
    GPIO_InitStruct.Pull  = io_cfg->gpio_pull;
    GPIO_InitStruct.Speed = io_cfg->gpio_speed;

    HAL_GPIO_Init(io_cfg->gpio_port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(io_cfg->gpio_port, io_cfg->gpio_pin, GPIO_PIN_RESET);
}

void drv_extiInit(exti_irq_t *exti_cfg)
{
    HAL_NVIC_SetPriority(exti_cfg->irq_name, exti_cfg->irq_priority, 0);
    HAL_NVIC_EnableIRQ(exti_cfg->irq_name);
}

void drv_gpioSetLevel(gpio_config_t *io_cfg, uint8_t pin_level)
{
    if (pin_level == GPIO_PIN_RESET)
    {
        HAL_GPIO_WritePin(io_cfg->gpio_port, io_cfg->gpio_pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(io_cfg->gpio_port, io_cfg->gpio_pin, GPIO_PIN_SET);
    }
}

uint8_t drv_gpioGetLevel(gpio_config_t *io_cfg)
{
    return HAL_GPIO_ReadPin(io_cfg->gpio_port, io_cfg->gpio_pin);
}

void drv_gpioToggleLevel(gpio_config_t *io_cfg)
{
    HAL_GPIO_TogglePin(io_cfg->gpio_port, io_cfg->gpio_pin);
}

void drv_gpioClose(gpio_config_t *io_cfg)
{
    HAL_GPIO_DeInit(io_cfg->gpio_port, io_cfg->gpio_pin);
}

void drv_gpioClkEn(gpio_clkPort_t io_port)
{
    switch (io_port)
    {
    case GPIO_PORT_A:
        __HAL_RCC_GPIOA_CLK_ENABLE();
        break;

    case GPIO_PORT_B:
        __HAL_RCC_GPIOB_CLK_ENABLE();
        break;

    case GPIO_PORT_C:
        __HAL_RCC_GPIOC_CLK_ENABLE();
        break;

    case GPIO_PORT_D:
        __HAL_RCC_GPIOD_CLK_ENABLE();
        break;

    case GPIO_PORT_E:
        __HAL_RCC_GPIOE_CLK_ENABLE();
        break;

    case GPIO_PORT_F:
        __HAL_RCC_GPIOF_CLK_ENABLE();
        break;

    case GPIO_PORT_G:
        __HAL_RCC_GPIOG_CLK_ENABLE();
        break;

    case GPIO_PORT_H:
        __HAL_RCC_GPIOH_CLK_ENABLE();
        break;

    case GPIO_PORT_I:
        __HAL_RCC_GPIOI_CLK_ENABLE();
        break;

    default:
        break;
    }

}

/************************************************Button************************************************/
void pause_key_init(void)
{
    // button_init(&pause_key_b, ReadButtonPin, 0, KEY_ID_PAUSE);
    // button_attach(&pause_key_b, PRESS_DOWN, pause_key_handler1);
    // button_attach(&pause_key_b, LONG_PRESS_START, pause_key_handler2);
    // button_start(&pause_key_b);
}

void switch_key_init(void)
{
    // button_init(&switch_key_b, ReadButtonPin, 0, KEY_ID_SWITCH);
    // button_attach(&switch_key_b, PRESS_DOWN, switch_key_left_handler);
    // button_attach(&switch_key_b, PRESS_UP, switch_key_right_handler);
    // button_start(&switch_key_b);
}

/****************************************DIP Switch************************************************/
// uint8_t read_SwitchValue(void)
// {
//     uint8_t switch_value = 0;
//     switch_value =  switch_is_on(SW2_GPIO_Port, SW2_Pin) << 2
//                     | switch_is_on(SW1_GPIO_Port, SW1_Pin) << 1
//                     | switch_is_on(SW0_GPIO_Port, SW0_Pin);
//     return switch_value;
// }

// static int switch_is_on(GPIO_TypeDef* sw_port, uint16_t sw_pin)
// {
// 	return ((HAL_GPIO_ReadPin(sw_port, sw_pin)) ? (0) : (1));
// }

/* USER CODE END 2 */
