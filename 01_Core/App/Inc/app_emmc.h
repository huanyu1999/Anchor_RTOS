#ifndef __APP_EMMC_H
#define __APP_EMMC_H

#include "main.h"

uint8_t app_emmcTest(void);
void app_logWrite(const char* log_data);
char* app_logFileNameMake(uint8_t year, uint8_t month, uint8_t day);
uint8_t app_sdCapacityCheck(void);
void app_emmcReadWriteDemo(void);

#endif
