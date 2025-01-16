#ifndef __USART_VOICE_H__
#define __USART_VOICE_H__

#include "main.h"


void Voice(uint8_t *pVoiceBuf, int len);
void User_VoiceInit(int volume);
void User_DistanceOut(void);
void User_FinalDisTask(void *pvParameters);
void User_VoiceTask(void *pvParameters);
void Report_Dis(float distance);
#endif
