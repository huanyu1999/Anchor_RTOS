/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : app_sdCard.c
 * @brief          : 日志文件写入，具体实现：写入时间，写入内容，文件创建，文件管理
 ******************************************************************************
 * @attention
 ******************************************************************************
 */
#include <stdio.h>

#include "app_sdCard.h"
#include "dev_linkFatFs.h"
#include "dev_rx8130ce.h"
#include "dev_emmc.h"
#include "drv_emmc.h"

#include "cmsis_os2.h"
#include "elog.h"
#include "ff.h"
#include "ffconf.h"

#define LOG_SIZE        512 * 1024
#define LOG_MAX_NUM     5

#define LOG_DIR    "0:/log"
#define LOG_MIN_FREE_MB 10              // 最小保留空间10MB

FIL logFile;                            /* File object */
char SDPath[4];                         /* SD card logical drive path */
// static uint8_t workBuffer[FF_MAX_SS];   /* a work buffer for the f_mkfs() */

/*
    1. 创建一个逻辑驱动号，链接IO操作，然后f_mount
    2. 进行异常情况处理，SD卡插拔，系统突然断电，系统突然重启
    3. 基于easylogger创建日志，做好分类标记，TAG标记
    4. 日志写入，结合easylogger，文件轮转（暂定日期）
    5. 文件夹建立，比如按照一周或者一个月建立一个文件夹
    6. 读取后续考虑
*/

FATFS logSaveFatFs; /* File system object for SD card logical drive */

uint8_t app_sdFileSystemInit(void)
{
    // dev_emmcInitialize();
    if (dev_FATFS_LinkDriver(&emmc_driver, SDPath) == 1) // 链接EMMC驱动到fatfs
    {
        // Error_Handler();
        log_e("FATFS link driver failed.");
        return 0;
    }
    else
    {
        // 上电后检测逻辑驱动状态，操作创建驱动号，直接挂载
        if (f_mount(&logSaveFatFs, (TCHAR const *)SDPath, 0) != FR_OK)
        {
            // 挂载失败，输出报警信息（后续考虑添加一个指示灯，表示SD卡异常），继续处理其他任务
            log_e("SD Card mounted failed.");
        }
        else
        {
            log_d("SD Card mounted successfully.");
        }
        return 1;
    }
}

uint8_t app_emmcTest(void)
{
    dev_emmcInitialize(0);
    uint32_t tx_buf[64];
    uint32_t rx_buf[64];

    for (uint32_t i = 0; i < 64; i++) {
        tx_buf[i] = i & 0xFFFF;
    }
    
    /* 写一个扇区 */
    if (drv_emmcWriteBlocks(tx_buf, 1, 1) != EMMC_OK)
        Error_Handler();

    /* 读回来验证 */
    if (drv_emmcReadBlocks(rx_buf, 1, 1) != EMMC_OK)
        Error_Handler();
}   

void app_logWrite(const char* log_data)
{
    // 先获取当前日期，生成今天应该写入的log文件名称
    static rx8130ce_time_t now;
    static char* logName;
    FRESULT res;

    dev_rx8130ceGetDateTime(&now);
    logName = app_logFileNameMake(now.year, now.month, now.day);

    res = f_stat(logName, NULL);
    // 判断log是否已经存在

    if (res == FR_OK)
    {
        res =  f_open(&logFile, logName, FA_OPEN_APPEND | FA_WRITE);                // 文件已经存在，继续续写

    }
    else
    {
        res = f_open(&logFile, logName, FA_CREATE_NEW | FA_WRITE);                          // 文件不存在，新建该文件，写入
    }

    if (res == FR_OK)               // 文件创建或者打开成功,执行log写入
    {
        f_write(&logFile, log_data, strlen(log_data), 0);
    }
}

char* app_logFileNameMake(uint8_t year, uint8_t month, uint8_t day)
{
    static rx8130ce_time_t now;
    static char fileName[24];           // 定义为静态变量，函数返回该变量，存在线程不安全问题

    dev_rx8130ceGetDateTime(&now);

    // 先生成需要查找的文件名
    snprintf(fileName, 17, "%04d%02d%02d-log.txt", now.year, now.month, now.day);

    return &fileName[0];
}

uint8_t app_sdCapacityCheck(void)
{
    DWORD free_clusters, free_sectors;
    FATFS *fs_ptr;
    fs_ptr = &logSaveFatFs;
    FRESULT res = f_getfree("0:/", &free_clusters, &fs_ptr);
    if (res != FR_OK)
    {
        log_e("SD Card capacity check failed.");
        return 0;
    }
    else
    {
        free_sectors = free_clusters * fs_ptr->csize;
        DWORD free_mb = (free_sectors / 2) / 1024; // 因为每扇区512B
        return (free_mb >= LOG_MIN_FREE_MB) ? 1 : 2; // OK or No Space
    }
}

void app_logFlush(void *arg)
{
    for (;;)
    {
        f_sync(&logFile);
        osDelay(400);
    }
}


/**************************************************************FatFs test
 * **************************************************************/
#if 1
FATFS SDFatFsTest;                        /* File system object for SD card logical drive */
FIL MyTestFile;                           /* File object */
char SDTestPath[4];                       /* SD card logical drive path */
static uint8_t workTestBuffer[FF_MAX_SS]; /* a work buffer for the f_mkfs() */

void sdCard_readWriteDemo(void)
{
    FRESULT res;                                          /* FatFs function common result code */
    uint32_t byteswritten, bytesread;                     /* File write/read counts */
    uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
    uint8_t rtext[100];                                   /* File read buffer */
    /*##-1- Link the micro SD disk I/O driver ##################################*/
    if (dev_FATFS_LinkDriver(&emmc_driver, SDTestPath) == 0) {
        /*##-2- Register the file system object to the FatFs module ##############*/
        if (f_mount(&SDFatFsTest, (TCHAR const *)SDTestPath, 0) != FR_OK) {
            /* FatFs Initialization Error */
            log_e("f_mount failed.");
        } else {
            /*##-3- Create a FAT file system (format) on the logical drive #########*/
            /* WARNING: Formatting the uSD card will delete all content on the device */
            uint8_t res = f_mkfs((TCHAR const *)SDTestPath, FM_FAT32, 0, workTestBuffer, sizeof(workTestBuffer));
            if (res != FR_OK) {
                /* FatFs Format Error */
                log_e("f_mkfs failed.");
            } else {
                /*##-4- Create and Open a new text file object with write access #####*/
                if (f_open(&MyTestFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
                    /* 'STM32.TXT' file Open for write Error */
                    log_e("f_open failed.");
                } else {
                    /*##-5- Write data to the text file ################################*/
                    res = f_write(&MyTestFile, wtext, sizeof(wtext), (void *)&byteswritten);

                    if ((byteswritten == 0) || (res != FR_OK)) {
                        /* 'STM32.TXT' file Write or EOF Error */
                        log_e("'STM32.TXT' file Write or EOF Error.");
                    } else {
                        /*##-6- Close the open text file #################################*/
                        f_close(&MyTestFile);

                        /*##-7- Open the text file object with read access ###############*/
                        if (f_open(&MyTestFile, "STM32.TXT", FA_READ) != FR_OK) {
                            /* 'STM32.TXT' file Open for read Error */
                            log_e("'STM32.TXT' file Open for read Error.");
                        } else {
                            /*##-8- Read data from the text file ###########################*/
                            res = f_read(&MyTestFile, rtext, sizeof(rtext), (UINT *)&bytesread);

                            if ((bytesread == 0) || (res != FR_OK)) {
                                /* 'STM32.TXT' file Read or EOF Error */
                                log_e("'STM32.TXT' file Read or EOF Error.");
                            } else {
                                /*##-9- Close the open text file #############################*/
                                f_close(&MyTestFile);

                                /*##-10- Compare read data with the expected data ############*/
                                if ((bytesread != byteswritten)) {
                                    /* Read data is different from the expected data */
                                    log_e("Read data is different from the expected data.");
                                } else {
                                    /* Success of the demo: no error occurrence */
                                    log_d("Success of the sd card demo.");
                                }
                            }
                        }
                    }
                }
            }
        }
        log_d("Success of the emmc demo.");
    }

    /*##-11- Unlink the RAM disk I/O driver ####################################*/
    dev_FATFS_UnLinkDriver(SDPath);

//    for (;;)
//    {
//    }
}
#endif
