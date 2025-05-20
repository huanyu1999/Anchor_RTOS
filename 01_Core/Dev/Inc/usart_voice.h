#ifndef __USART_VOICE_H__
#define __USART_VOICE_H__

#include "main.h"

typedef enum {
    AppointTrack                    = 0x07,                 /*指定曲目播放*/
    SetCycleCount                   = 0x19,                 /*设置循环次数*/
    SetEQ                           = 0X1A,                 /*EQ设置*/
    SelectTrackNoPlay               = 0x19,                 /*选曲不播放*/
    GoToDisksign                    = 0X0B,                 /*切换指定盘符*/
    SetVolume                       = 0x13,                 /*音量设置*/
    SetLoopMode                     = 0x18,                 /*设置循环模式*/
    SetChannel                      = 0x1D,                 /*设置通道*/ 
    AppointTimeBack                 = 0x22,                 /*指定时间快退*/
    AppointTimeFast                 = 0x23,                 /*指定时间快退*/
}UartCommandData;                                           //包含多个数据的指令,起始码-指令类型-数据长度-数据1-...-校验和

void Voice(uint8_t *pVoiceBuf, int len);
void User_VoiceInit(int volume);
void User_DistanceOut(void);
void User_FinalDisTask(void *pvParameters);
void User_VoiceTask(void *pvParameters);
void Report_Dis(float distance);
void JQ8x00_Command_Data(UartCommandData Command, uint8_t DATA);
#endif
