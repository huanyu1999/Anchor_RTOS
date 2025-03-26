#ifndef __DEV_LED_BUZZER_DIP_H__
#define __DEV_LED_BUZZER_DIP_H__

#include "board_gpio.h"

#define io_LevelLow GPIO_PIN_RESET
#define io_LevelHigh GPIO_PIN_SET 

#define SWITCH_IS_ON(dip) ((dev_dipRead(dip)) ? (0) : (1))

void dev_ledAllInit(void);
void dev_ledInit(gpioBoard_enum_t led);
void dev_ledBlink(gpioBoard_enum_t led);
void dev_ledOn(gpioBoard_enum_t led);
void dev_ledOff(gpioBoard_enum_t led);

void dev_buzzerInit(gpioBoard_enum_t buzzer);
void dev_buzzerOpen(gpioBoard_enum_t buzzer);
void dev_buzzerClose(gpioBoard_enum_t buzzer);
void dev_dipInit(gpioBoard_enum_t dip);

uint8_t dev_dipRead(gpioBoard_enum_t dip);
uint8_t dev_getDipVal(void);
#endif
