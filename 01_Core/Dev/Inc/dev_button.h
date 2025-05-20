#ifndef __DEV_BUTTON_H__
#define __DEV_BUTTON_H__

#include "dev_led_buzzer_dip.h"
#include "board_gpio.h"
#include "com_multiButton.h"

#define BUTTON_ID_PAUSE  0
#define BUTTON_ID_SWITCH 1

void dev_buttonInit(gpioBoard_enum_t button);
void dev_buttonTaskInit(void);
uint8_t dev_buttonRead(uint8_t button);
void dev_multiButtonInit( struct Button* button_handler, uint8_t button_id);
void dev_multiButtonAddAndStart(struct Button* button_handler, PressEvent event, BtnCallback cb);
void dev_buttonTaskInit(void);
void printf_taskState(void);

#endif
