#ifndef __APP_SDCARD_H__
#define __APP_SDCARD_H__

#include "main.h"

uint8_t app_sdFileSystemInit(void);
uint8_t app_emmcTest(void);
void app_logWrite(const char* log_data);
char* app_logFileNameMake(uint8_t year, uint8_t month, uint8_t day);
uint8_t app_sdCapacityCheck(void);
void sdCard_readWriteDemo(void);

#endif
