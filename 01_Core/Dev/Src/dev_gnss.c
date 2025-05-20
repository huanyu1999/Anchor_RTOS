#include "dev_gnss.h"
#include "usart.h"
#include "cmsis_os.h"
#include "elog.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/***********************************GNSS模块控制命令***********************************/
#define GNSSMOD_FIRMWARE_VER    $PCAS06,0*1B
#define GNSSMOD_SET_BAUDRATE_19200  $PCAS01,2*1E

char outputCtrl_cmd[] = "$PCAS03,0GGA,0GLL,0GSA,0GSV,1RMC,0VTG,0ZDA,0ANT,0DHV,0LPS,res1,res2,0UTC,0GST,res3,res4,res5,0TIM*CS\r\n";

char outputCtrl_cmd1[] = "$PCAS03,0,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n";
char outputCtrl_cmd2[] = "$PCAS03,1,1,1,1,1,1,1,1,1,1,,,1,1*02\r\n";
char outputCtrl_cmd3[] = "$PCAS03,0,0,0,0,1,0,0,0,0,0,,,0,0,,,,0*02\r\n";

#define GNSS_DMA_RX_BUF_SIZE    256
uint8_t gnss_dmaRxBuf[GNSS_DMA_RX_BUF_SIZE];
uint8_t gnss_lineBuf[GNSS_DMA_RX_BUF_SIZE];
uint16_t gnssRxLen = 0;
gnss_rmc_t gnss_msg;

void dev_gnssModInit(void)
{
    // 初始化的同时开始接收
    // dev_gnssModEnableUartIdleInt();
    dev_gnssModSetOnlyOutRMC();
    dev_gnssModStartRx();
}

void dev_gnssModTxCmd(uint32_t cmd)
{
    switch (cmd)
    {
    case 1:
        /* code */
        break;
    
    default:
        break;
    }
}

void dev_gnssModSetOnlyOutRMC(void)
{
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)outputCtrl_cmd2, ARRAY_LENGTH(outputCtrl_cmd2));
}

void dev_gnssModStartRx(void)
{
    HAL_UART_Receive_DMA(&huart1, gnss_dmaRxBuf, GNSS_DMA_RX_BUF_SIZE);
}

void dev_gnssModReceiveAndParse(void)
{
    __HAL_DMA_DISABLE(huart1.hdmarx);                                       // 手动停止DMA
    gnssRxLen = GNSS_DMA_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    memcpy(gnss_lineBuf, gnss_dmaRxBuf, gnssRxLen);
    for (int i = 0; i < gnssRxLen; i++)
    {
        log_d("%c", gnss_dmaRxBuf[i]);
    }
    gnss_lineBuf[gnssRxLen] = '\0';

    char *line = strtok((char*)gnss_lineBuf, "\r\n");

    while (line != NULL)
    {
        if (strstr(line, "$GNGGA") || strstr(line, "$GPRMC"))           // 查找消息ID出现的第一个位置
        {
            char *tokens[20] = {0};
            uint8_t index = 0;
            char *token = strtok(line, ",");

            while (token && index < 20)
            {
                tokens[index++] = token;
                token = strtok(NULL, ",");
            }

            if (index >= 10 && tokens[2][0] == 'A')  // 有效定位
            {
                // char utc_time[11] = {0};
                strncpy(gnss_msg.utc_time, tokens[1], 10);              // hhmmss.sss
                // strncpy(gnss_msg.date, tokens[9], 7);                   

                float latitude = atof(tokens[2]);  // 纬度
                float longitude = atof(tokens[4]); // 经度

                char lat_dir = tokens[3][0];
                char lon_dir = tokens[5][0];

                // 可以根据方向修正符号
                if (lat_dir == 'S') latitude = -latitude;
                if (lon_dir == 'W') longitude = -longitude;

                log_d("GNSS: time=%s, date=%s, lat=%.5f %c, lon=%.5f %c\r\n",
                    gnss_msg.utc_time, gnss_msg.date, latitude, lat_dir, longitude, lon_dir);
            }
        }

        line = strtok(NULL, "\r\n");  // 继续解析下一行
    }
}

void HAL_UART_IdleCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 释放一个信号量
        extern osSemaphoreId_t gnssReceiveSem;
        osSemaphoreRelease(gnssReceiveSem);
    }
}
