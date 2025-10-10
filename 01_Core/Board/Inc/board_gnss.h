#ifndef __BOARD_GNSS_H__
#define __BOARD_GNSS_H__

#define GNSS_RST_PIN  GPIO_PIN_6
#define GNSS_RST_PORT GPIOE

void board_gnssModInit(void);
void board_gnssModReset(void);

#endif
