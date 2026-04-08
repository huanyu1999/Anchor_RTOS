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
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

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

    // HAL_GPIO_WritePin(io_cfg->gpio_port, io_cfg->gpio_pin, GPIO_PIN_RESET);
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

    case GPIO_PORT_NUM:
    default:
        break;
    }

}
/* USER CODE END 2 */
