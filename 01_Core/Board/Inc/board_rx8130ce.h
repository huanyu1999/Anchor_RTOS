#ifndef __BOARD_RX8130CE_H__
#define __BOARD_RX8130CE_H__

#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"

#define RX8130CE_RST_PORT       GPIOB
#define RX8130CE_RST_PIN        GPIO_PIN_8

#define RX8130CE_INT_PORT       GPIOB
#define RX8130CE_INT_PIN        GPIO_PIN_7

void board_rx8130ceIoInit(void);
void board_rx8130ceI2cInit(void);
void board_rx8130ceInit(void);
void board_rx8130ceBufWrite(uint8_t addr, uint8_t reg, uint8_t* val, uint16_t buf_size);
void board_rx8130ceBufRead(uint8_t addr, uint8_t reg, uint8_t* buf, uint16_t buf_size);
void board_rx8130ceHardRst(void);
#endif
