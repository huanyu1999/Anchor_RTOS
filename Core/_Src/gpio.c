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
    switch_key  PB1
    pause_key   PB0
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
/* USER CODE BEGIN 1 */
const static gpio_config_t gpio_config[gpio_num] = {
    {.gpio_port = Dw1000_IRQ_GPIO_Port,  .gpio_pin = Dw1000_IRQ_Pin, .gpio_mode = GPIO_MODE_IT_RISING, .gpio_pull = GPIO_PULLDOWN, .gpio_speed = NULL},
    {.gpio_port = Dw1000_RSTn_GPIO_Port, .gpio_pin = Dw1000_RSTn_Pin, .gpio_mode = GPIO_MODE_ANALOG, .gpio_pull = GPIO_NOPULL, .gpio_speed = NULL},
    {.gpio_port = UwbOK_LED_GPIO_Port,   .gpio_pin = UwbOK_LED_Pin, .gpio_mode = GPIO_MODE_OUTPUT_PP, .gpio_pull = GPIO_PULLUP, .gpio_speed = GPIO_SPEED_FREQ_VERY_HIGH},
    {.gpio_port = S_KEY_GPIO_Port,       .gpio_pin = S_KEY_Pin, .gpio_mode = GPIO_MODE_INPUT, .gpio_pull = GPIO_PULLUP, .gpio_speed = NULL},
    {.gpio_port = P_KEY_GPIO_Port,       .gpio_pin = P_KEY_Pin, .gpio_mode = GPIO_MODE_INPUT, .gpio_pull = GPIO_PULLUP, .gpio_speed = NULL},
    {.gpio_port = Onside_LED_GPIO_Port,  .gpio_pin = Onside_LED_Pin, .gpio_mode = GPIO_MODE_OUTPUT_PP, .gpio_pull = GPIO_PULLUP, .gpio_speed = GPIO_SPEED_FREQ_VERY_HIGH},
    {.gpio_port = Across_LED_GPIO_Port,  .gpio_pin = Across_LED_Pin, .gpio_mode = GPIO_MODE_OUTPUT_PP, .gpio_pull = GPIO_PULLUP, .gpio_speed = GPIO_SPEED_FREQ_VERY_HIGH},
    {.gpio_port = BUZZER_GPIO_Port,      .gpio_pin = BUZZER_Pin, .gpio_mode = GPIO_MODE_OUTPUT_PP, .gpio_pull = GPIO_PULLUP, .gpio_speed = GPIO_SPEED_FREQ_VERY_HIGH},
    {.gpio_port = CAN_TX_LED_GPIO_Port,  .gpio_pin = CAN_TX_LED_Pin, .gpio_mode = GPIO_MODE_OUTPUT_PP, .gpio_pull = GPIO_PULLUP, .gpio_speed = GPIO_SPEED_FREQ_VERY_HIGH},
    {.gpio_port = CAN_RX_LED_GPIO_Port,  .gpio_pin = CAN_RX_LED_Pin, .gpio_mode = GPIO_MODE_OUTPUT_PP, .gpio_pull = GPIO_PULLUP, .gpio_speed = GPIO_SPEED_FREQ_VERY_HIGH},
    {.gpio_port = SW0_GPIO_Port,         .gpio_pin = SW0_Pin, .gpio_mode = GPIO_MODE_INPUT, .gpio_pull = GPIO_PULLUP, .gpio_speed = NULL},
    {.gpio_port = SW1_GPIO_Port,         .gpio_pin = SW1_Pin, .gpio_mode = GPIO_MODE_INPUT, .gpio_pull = GPIO_PULLUP, .gpio_speed = NULL},
    {.gpio_port = SW2_GPIO_Port,         .gpio_pin = SW2_Pin, .gpio_mode = GPIO_MODE_INPUT, .gpio_pull = GPIO_PULLUP, .gpio_speed = NULL},
};

const static exti_irq_t exti_config[exti_num] = {
    {.irq_name = Dw1000_IRQ_EXTI_IRQn, .irq_priority = 4}
};

#define KEY_ID_PAUSE  0
#define KEY_ID_SWITCH 1

static uint8_t ReadButtonPin(uint8_t buttonID);
static int switch_is_on(GPIO_TypeDef* sw_port,  uint16_t sw_pin);

/* USER CODE END 1 */

/** Configure pins as
        * Analog * Input * Output * EVENT_OUT * EXTI
*/
void MX_GPIO_Init(void)
{
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    IO_OUTPUT_CLOSE();
    gpioAndExti_init();
}

/* USER CODE BEGIN 2 */
void gpioAndExti_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    for(int i = 0; i < gpio_num; i++)
    {
        GPIO_InitStruct.Pin = gpio_config[i].gpio_pin;
        GPIO_InitStruct.Mode = gpio_config[i].gpio_mode;
        GPIO_InitStruct.Pull = gpio_config[i].gpio_pull;
        GPIO_InitStruct.Speed = gpio_config[i].gpio_speed;

        HAL_GPIO_Init(gpio_config[i].gpio_port, &GPIO_InitStruct);
        HAL_GPIO_WritePin(gpio_config[i].gpio_port, gpio_config[i].gpio_pin, GPIO_PIN_RESET);
    }
    
    for(int i = 0; i < exti_num; i++)
    {
        HAL_NVIC_SetPriority(exti_config[i].irq_name, exti_config[i].irq_priority, 0);
        HAL_NVIC_EnableIRQ(exti_config[i].irq_name);
    }
}

/************************************************Button************************************************/
struct Button pause_key_b;
struct Button switch_key_b;
    
void pause_key_init(void)
{
    button_init(&pause_key_b, ReadButtonPin, 0, KEY_ID_PAUSE);
    button_attach(&pause_key_b, PRESS_DOWN, pause_key_handler1);
    button_attach(&pause_key_b, LONG_PRESS_START, pause_key_handler2);
    button_start(&pause_key_b);
}

void switch_key_init(void)
{
    button_init(&switch_key_b, ReadButtonPin, 0, KEY_ID_SWITCH);
    button_attach(&switch_key_b, PRESS_DOWN, switch_key_left_handler);
    button_attach(&switch_key_b, PRESS_UP, switch_key_right_handler);
    button_start(&switch_key_b);

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
void led_toggle (gpio_enum_t led)
{
    switch (led)
    {
        case uwb_ok_led:
            HAL_GPIO_TogglePin(UwbOK_LED_GPIO_Port, UwbOK_LED_Pin);
            break;
        case onside_led:
            HAL_GPIO_TogglePin(Onside_LED_GPIO_Port, Onside_LED_Pin);
            break;
        case across_led:
            HAL_GPIO_TogglePin(Across_LED_GPIO_Port, Across_LED_Pin);
            break;
        case can_tx_led:
            HAL_GPIO_TogglePin(CAN_TX_LED_GPIO_Port, CAN_TX_LED_Pin);
            break;
        case can_rx_led:
            HAL_GPIO_TogglePin(CAN_RX_LED_GPIO_Port, CAN_RX_LED_Pin);
            break;
        default:
            break;
    }
}


void led_on (gpio_enum_t led)
{
    switch (led)
    {
        case uwb_ok_led:
            HAL_GPIO_WritePin(UwbOK_LED_GPIO_Port, UwbOK_LED_Pin, GPIO_PIN_RESET);
            break;
        case onside_led:
            HAL_GPIO_WritePin(Onside_LED_GPIO_Port, Onside_LED_Pin, GPIO_PIN_RESET);
            break;
        case across_led:
            HAL_GPIO_WritePin(Across_LED_GPIO_Port, Across_LED_Pin, GPIO_PIN_RESET);
            break;
        case can_tx_led:
            HAL_GPIO_WritePin(CAN_TX_LED_GPIO_Port, CAN_TX_LED_Pin, GPIO_PIN_RESET);
            break;
        case can_rx_led:
            HAL_GPIO_WritePin(CAN_RX_LED_GPIO_Port, CAN_RX_LED_Pin, GPIO_PIN_RESET);
            break;
        default:
            break;
    }
}

void led_off (gpio_enum_t led)
{
    switch (led)
    {
        case uwb_ok_led:
            HAL_GPIO_WritePin(UwbOK_LED_GPIO_Port, UwbOK_LED_Pin, GPIO_PIN_SET);
            break;
        case onside_led:
            HAL_GPIO_WritePin(Onside_LED_GPIO_Port, Onside_LED_Pin, GPIO_PIN_SET);
            break;
        case across_led:
            HAL_GPIO_WritePin(Across_LED_GPIO_Port, Across_LED_Pin, GPIO_PIN_SET);
            break;
        case can_tx_led:
            HAL_GPIO_WritePin(CAN_TX_LED_GPIO_Port, CAN_TX_LED_Pin, GPIO_PIN_SET);
            break;
        case can_rx_led:
            HAL_GPIO_WritePin(CAN_RX_LED_GPIO_Port, CAN_RX_LED_Pin, GPIO_PIN_SET);
            break;
        default:
            break;
    }
}

/****************************************DIP Switch************************************************/
uint8_t read_SwitchValue(void)
{
    uint8_t switch_value = 0;
    switch_value =  switch_is_on(SW2_GPIO_Port, SW2_Pin) << 2
                    | switch_is_on(SW1_GPIO_Port, SW1_Pin) << 1
                    | switch_is_on(SW0_GPIO_Port, SW0_Pin);
    return switch_value;
}

static int switch_is_on(GPIO_TypeDef* sw_port, uint16_t sw_pin)
{
	return ((HAL_GPIO_ReadPin(sw_port, sw_pin)) ? (0) : (1));
}

/* USER CODE END 2 */
