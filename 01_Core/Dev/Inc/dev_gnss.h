#ifndef __DEV_GNSS_H__
#define __DEV_GNSS_H__

#include <stdint.h>
#include "main.h"

typedef struct 
{   
    char utc_time[11];                  // UTC时间 "hhmmss.sss"
    char locationStatus;                // A 有效， V 无效
    double latitude;                    // 纬度（十进制度）
    char lat_dir;                       // 纬度方向 'N' or 'S'
    double longitude;                   // 经度（十进制度）
    char lon_dir;                       // 经度方向 'E' or 'W'
    float speed;                        // 地面速度（单位：节）
    float course;                       // 航向角（度）
    char date[7];                       // 日期（ddmmyy）
} gnss_rmc_t;

void dev_gnssModInit(void);
void dev_gnssModTxCmd(uint32_t cmd);
void dev_gnssModSetOnlyOutRMC(void);
void dev_gnssModStartRx(void);
void dev_gnssModReceiveAndParse(void);
void HAL_UART_IdleCallback(UART_HandleTypeDef *huart);
#endif
