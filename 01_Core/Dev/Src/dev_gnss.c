#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "dev_gnss.h"
#include "dev_rx8130ce.h"
#include "board_gnss.h"
#include "usart.h"
#include "cmsis_os.h"
#include "elog.h"

/***********************************GNSS模块控制命令***********************************/

#define GNSS_DMA_RX_BUF_SIZE    1024
uint8_t gnss_dmaRxBuf[GNSS_DMA_RX_BUF_SIZE];
uint8_t gnss_lineBuf[GNSS_DMA_RX_BUF_SIZE];
uint16_t gnssRxLen = 0;

rx8130ce_time_t sync_time;

static void PrintTimeUTCtoCST(int year, int month, int day, int hour, int minute, int second);
static uint8_t get_week(int year, int month, int day);

void dev_gnssModInit(void)
{
    board_gnssModInit();
    board_gnssModReset();
}

void dev_gnssModTxCmd(uint32_t cmd) {
    switch (cmd) {
    case 1:
        /* code */
        break;
    
    default:
        break;
    }
}

//void dev_gnssModSetOnlyOutRMC(void)
//{
//}

void dev_gnssModStartRx(void) {
    HAL_UART_Receive_DMA(&huart4, gnss_dmaRxBuf, GNSS_DMA_RX_BUF_SIZE);
}

void dev_gnssModReceiveAndParse(void) {
    // __HAL_DMA_DISABLE(huart4.hdmarx);               // 使用这个宏，会出现再次使能DMA时无法使能的情况，显示是DMA还处于lock状态中                       
    HAL_DMA_Abort(huart4.hdmarx);                      // 手动停止DMA
    gnssRxLen = GNSS_DMA_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart4.hdmarx);
    memcpy(gnss_lineBuf, gnss_dmaRxBuf, gnssRxLen);
    ParseNMEA_Buffer_Limited(gnss_lineBuf, gnssRxLen);
}

#define MAX_NMEA_LINES 13

// 打印 UTC -> CST 时间
void PrintTimeUTCtoCST(int year,int month,int day,int hour,int minute,int second)
{
    int cst_year = year, cst_month = month, cst_day = day;
    int cst_hour = hour + 8, cst_minute = minute, cst_second = second;

    int mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if ((cst_year % 4 == 0 && cst_year % 100 != 0) || (cst_year % 400 == 0)) {
        mdays[1] = 29;
    }
    
    if (cst_hour >= 24) {
        cst_hour -= 24;
        cst_day++;
        if (cst_day > mdays[cst_month-1]) {
            cst_day = 1;
            cst_month++;
            if (cst_month > 12) {
                cst_month = 1;
                cst_year++;
            }
        }
    }
    sync_time.day     = cst_day;
    sync_time.month   = cst_month;
    sync_time.year    = cst_year - 2000;
    sync_time.hours   = cst_hour;
    sync_time.minutes = cst_minute;
    sync_time.seconds = cst_second;
    sync_time.week    = get_week(cst_year, cst_month, cst_day);
    dev_rx8130ceSetDateTime(&sync_time);

    // log_d("[UTC] %04d-%02d-%02d %02d:%02d:%02d -> [CST] %04d-%02d-%02d %02d:%02d:%02d\r\n",
    // year, month, day, hour, minute, second, cst_year, cst_month, cst_day, cst_hour, cst_minute, cst_second);
}

// 解析单条 NMEA 报文
void ParseSingleNMEA(char *line)
{
    // if (strncmp(line, "$GNZDA", 6) == 0 || strncmp(line, "$GPZDA", 6) == 0) {
    // if (strncmp(line, "$GNZDA", 6) == 0) {
    //     char *token, *saveptr;
    //     int field = 0;
    //     in  t hh = 0, mm = 0 , ss = 0, day = 0, month = 0,year = 0;
    //     token = strtok_r(line, ",", &saveptr);
    //     while (token) {
    //         field++;
    //         if (field == 2 && strlen(token) >= 6) {
    //             hh = (token[0] - '0') * 10 + (token[1] - '0');
    //             mm = (token[2] - '0') * 10 + (token[3] - '0');
    //             ss = (token[4] - '0') * 10 + (token[5] - '0');
    //         }
    //         else if(field == 3) day = atoi(token);
    //         else if(field == 4) month = atoi(token);
    //         else if(field == 5) year = atoi(token);
    //         token = strtok_r(NULL, ", ", &saveptr);
    //     }
    //     PrintTimeUTCtoCST(year, month, day, hh, mm, ss);
    // } else if (strncmp(line, "$GNRMC", 6) == 0 || strncmp(line, "$GPRMC", 6) == 0) {
    if (strncmp(line, "$GNRMC", 6) == 0 || strncmp(line, "$GPRMC", 6) == 0) {
        char *token, *saveptr;
        int field = 0;
        int hh = 0, mm = 0, ss = 0, day = 0, month = 0, year = 0;
        char status = 'V';
        token = strtok_r(line, ",", &saveptr);
        while (token) {
            field++;
            if (field == 2 && strlen(token) >= 6) {
                hh = (token[0] - '0') * 10 + (token[1] - '0');
                mm = (token[2] - '0') * 10 + (token[3] - '0');
                ss = (token[4] - '0') * 10 + (token[5] - '0');
            } else if(field == 3) {
                status = token[0]; 
            } else if(field == 10 && strlen(token) >= 6) {
                day   = (token[0] - '0') * 10 + (token[1] - '0');
                month = (token[2] - '0') * 10 + (token[3] - '0');
                year  = (token[4] - '0') * 10 + (token[5] - '0') + 2000;
            }
            token = strtok_r(NULL,",",&saveptr);
        }
        if (status == 'A') {
            PrintTimeUTCtoCST(year, month, day, hh, mm, ss);
            // log_d("time sync success.\r\n");
        } else {
            // log_d("[RMC] 定位无效\r\n");
        }
    } else {
        // 其它报文原样打印，可选
        // log_d("%s\r\n",line);
    }
}

// 解析缓冲区，只处理前 MAX_NMEA_LINES 条报文
void ParseNMEA_Buffer_Limited(uint8_t *buf,uint16_t len)
{
    uint16_t i = 0, j = 0;
    char line[128];
    int line_count = 0;

    while (i < len && line_count < MAX_NMEA_LINES)
    {
        if (buf[i] == '$')
        {
            j = 0;
            while (i < len && buf[i] != '\n' && j < sizeof(line) - 1)
            {
                if(buf[i]!='\r')
                { 
                    line[j++] = buf[i];
                }
                i++;
            }
            line[j] = '\0';
            ParseSingleNMEA(line);
            line_count++;
        }
        else i++;
    }
}

static uint8_t get_week(int year, int month, int day)
{
    static const uint8_t t[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
    if (month < 3)
    {
        year -= 1;
    }
    uint8_t w = (year + year/4 - year/100 + year/400 + t[month - 1] + day) % 7;
    // Sakamoto算法返回 0=周日, 1=周一, ... 6=周六
    if (w == 0)
    {
        return 7; // 周日改成 7
    }
    return w;     // 其他保持 1~6
}
