#ifndef __BOARD_EMMC_H__
#define __BOARD_EMMC_H__

#define EMMC_RST_PORT GPIOA
#define EMMC_RST_PIN   GPIO_PIN_15

void board_emmcInit(void);
void board_emmcReset(void);

#endif
