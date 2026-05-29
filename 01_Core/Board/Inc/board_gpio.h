#ifndef __BOARD_GPIO_H__
#define __BOARD_GPIO_H__

#include "gpio.h"

#define RT9013_EN_PIN           GPIO_PIN_8
#define RT9013_GPIO_PORT        GPIOE

#define UWBOK_LED_PIN           GPIO_PIN_0
#define UWBOK_LED_GPIO_PORT     GPIOE

#define CAN_RX_LED_PIN          GPIO_PIN_1
#define CAN_RX_LED_GPIO_PORT    GPIOE

#define ONSIDE_LED_PIN          GPIO_PIN_1
#define ONSIDE_LED_GPIO_PORT    GPIOB

#define ACROSS_LED_PIN          GPIO_PIN_7
#define ACROSS_LED_GPIO_PORT    GPIOE

#define S_KEY_PIN               GPIO_PIN_0
#define S_KEY_GPIO_PORT         GPIOB

#define P_KEY_PIN               GPIO_PIN_5
#define P_KEY_GPIO_PORT         GPIOC

#define BUZZER_PIN              GPIO_PIN_10
#define BUZZER_GPIO_PORT        GPIOE

#define ANCHORID0_PIN           GPIO_PIN_2
#define ANCHORID0_GPIO_PORT     GPIOE

#define ANCHORID1_PIN           GPIO_PIN_3
#define ANCHORID1_GPIO_PORT     GPIOE

#define ANCHORID2_PIN           GPIO_PIN_4
#define ANCHORID2_GPIO_PORT     GPIOE

typedef enum {
    RT9013_EN,
    UWB_OK_LED,
    CAN_RX_LED, 
    ONSIDE_LED,
    ACROSS_LED,
    SWITCH_KEY,
    PAUSE_KEY,
    BUZZER,
    ANCHOR_ID0,
    ANCHOR_ID1,
    ANCHOR_ID2,
    IO_NUM
} gpioBoard_enum_t;

void board_gpioInit(gpioBoard_enum_t led_index);
void board_gpioSetLevel(gpioBoard_enum_t led_index, uint8_t level);
uint8_t board_gpioGetLevel(gpioBoard_enum_t led_index);
void board_gpioToggleLevel(gpioBoard_enum_t led_index);
void board_gpioClose(gpioBoard_enum_t io_index);
#endif
