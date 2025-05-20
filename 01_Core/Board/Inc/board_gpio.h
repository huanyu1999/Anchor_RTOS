#ifndef __BOARD_GPIO_H__
#define __BOARD_GPIO_H__

#include "gpio.h"

#define UwbOK_LED_Pin           GPIO_PIN_2
#define UwbOK_LED_GPIO_Port     GPIOC

#define Onside_LED_Pin          GPIO_PIN_5
#define Onside_LED_GPIO_Port    GPIOC

#define Across_LED_Pin          GPIO_PIN_0
#define Across_LED_GPIO_Port    GPIOB

#define CAN_RX_LED_Pin          GPIO_PIN_1
#define CAN_RX_LED_GPIO_Port    GPIOB

#define S_KEY_Pin               GPIO_PIN_1
#define S_KEY_GPIO_Port         GPIOC

#define P_KEY_Pin               GPIO_PIN_0
#define P_KEY_GPIO_Port         GPIOC

#define BUZZER_Pin              GPIO_PIN_6
#define BUZZER_GPIO_Port        GPIOC

#define SW0_Pin                 GPIO_PIN_14
#define SW0_GPIO_Port           GPIOC
#define SW1_Pin                 GPIO_PIN_15
#define SW1_GPIO_Port           GPIOC


typedef enum {
    uwb_ok_led,
    onside_led,
    across_led,
    can_rx_led,   
    switch_key,
    pause_key,
    buzzer,
    sw0,
    sw1,
    IO_NUM
} gpioBoard_enum_t;

void board_gpioInit(gpioBoard_enum_t led_index);
void board_gpioSetLevel(gpioBoard_enum_t led_index, uint8_t level);
uint8_t board_gpioGetLevel(gpioBoard_enum_t led_index);
void board_gpioToggleLevel(gpioBoard_enum_t led_index);
void board_gpioClose(gpioBoard_enum_t io_index);
#endif
