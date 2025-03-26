#ifndef __DEV_BUTTON_H__
#define __DEV_BUTTON_H__

#include "dev.h"
#include "com_multiButton.h"

#define BUTTON_ID_PAUSE  0
#define BUTTON_ID_SWITCH 1

void dev_buttonInit(gpioBoard_enum_t button);
uint8_t dev_buttonRead(uint8_t button);
void dev_multiButtonInit( struct Button* button_handler, uint8_t button_id);
void dev_multiButtonAddAndStart(struct Button* button_handler, PressEvent event, BtnCallback cb);
void dev_bottonTaskInit(void);

#endif
