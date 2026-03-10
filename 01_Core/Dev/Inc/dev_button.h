#ifndef __DEV_BUTTON_H__
#define __DEV_BUTTON_H__

#include "dev_led_buzzer_dip.h"
#include "board_gpio.h"
#include "com_multiButton.h"

#define BUTTON_ID_PAUSE  0
#define BUTTON_ID_SWITCH 1

#define FLAG_DIS_ALARM      (1 << 0)
#define FLAG_P_BUTTON_ALARM (1 << 1)
#define FLAG_P_BUTTON_MUTE  (1 << 2)
#define FLAG_S_BUTTON_ALARM (1 << 3)
#define FLAG_S_BUTTON_MUTE  (1 << 4)
#define FLAG_DIS_THRELOD_ALARM (1 << 5)
#define FLAG_DIS_INVAILD_ALARM  (1 << 6)

#define FLAG_S_BUTTON_TRIGGER 0x03

void dev_buttonInit(gpioBoard_enum_t button);
uint8_t dev_buttonRead(uint8_t button);
void dev_buttonMultiInit(void);

#endif
