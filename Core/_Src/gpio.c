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
    warning bee PA8
    pause_key   PB0
    switch_key  PB1
    across_led  PC7
    onside_led  PC6
*/
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
#define KEY_ID_PAUSE  0
#define KEY_ID_SWITCH 1

static uint8_t ReadButtonPin(uint8_t buttonID);

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, RUN_LED_Pin|BEE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, Onside_LED_Pin|Across_LED_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = Dw1000_IRQ_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(Dw1000_IRQ_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = Dw1000_RSTn_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Dw1000_RSTn_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = RUN_LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(RUN_LED_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = P_KEY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(P_KEY_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = S_KEY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(S_KEY_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PCPin PCPin */
    GPIO_InitStruct.Pin = Onside_LED_Pin|Across_LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = BEE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(BEE_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    
}

/* USER CODE BEGIN 2 */

/************************************************Button************************************************/
struct Button pause_key;
struct Button switch_key;
    
void pause_key_init()
{
    button_init(&pause_key, ReadButtonPin, 0, KEY_ID_PAUSE);
    button_attach(&pause_key, PRESS_DOWN, pause_key_handler1);
    button_attach(&pause_key, LONG_PRESS_START, pause_key_handler2);
    button_start(&pause_key);
}

void switch_key_init(void)
{
    button_init(&switch_key, ReadButtonPin, 0, KEY_ID_SWITCH);
    button_attach(&switch_key, PRESS_DOWN, switch_key_handler1);
    button_attach(&switch_key, PRESS_UP, switch_key_handler2);
    button_start(&switch_key);

}

static uint8_t ReadButtonPin(uint8_t buttonID)
{
    switch(buttonID)
    {
        case KEY_ID_PAUSE:
            return HAL_GPIO_ReadPin(P_KEY_GPIO_Port, P_KEY_Pin);
        case KEY_ID_SWITCH:
            return HAL_GPIO_ReadPin(S_KEY_GPIO_Port, S_KEY_Pin);
        default:
            break;
    }
    return 0;
}


/************************************************Led************************************************/
void led_toggle (led_t led)
{
    switch (led)
    {
        case RUN_LED:
            HAL_GPIO_TogglePin(RUN_LED_GPIO_Port, RUN_LED_Pin);
            break;

        default:
            break;
    }
}


void led_on (led_t led)
{
    switch (led)
    {
        case RUN_LED:
            HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);
            break;

        default:
            break;
    }
}

void led_off (led_t led)
{
    switch (led)
    {
        case RUN_LED:
            HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_SET);
            break;

        default:
            break;
    }
}
/* USER CODE END 2 */
