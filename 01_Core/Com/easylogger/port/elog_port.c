/*
 * This file is part of the EasyLogger Library.
 *
 * Copyright (c) 2015, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2015-04-28
 */

#include <stdio.h>
#include <elog.h>
#include "dev_rx8130ce.h"
#include "cmsis_os.h"

extern osSemaphoreId_t sema_elogLock;                                // elog_lock output Semaphore

#define LOG_STREAM_BUF_SIZE 1024
StreamBufferHandle_t log_streamBufferHandle;
static uint8_t log_streamBuffer[LOG_STREAM_BUF_SIZE];
StaticStreamBuffer_t streamBufferStructure;
const size_t triggerLevel = 16;          // set to 16 temporarily

/**
 * EasyLogger port initialize
 *
 * @return result
 */
ElogErrCode elog_port_init(void)
{
    ElogErrCode result = ELOG_NO_ERR;

    /* add your code here */
    // 此处创建一个stream buffer给log输出用
    log_streamBufferHandle = xStreamBufferCreateStatic(sizeof(log_streamBuffer), triggerLevel, log_streamBuffer, &streamBufferStructure);

    return result;
}

/**
 * EasyLogger port deinitialize
 *
 */
void elog_port_deinit(void)
{

    /* add your code here */
}

/**
 * output log port interface
 *
 * @param log output of log
 * @param size log size
 */
void elog_port_output(const char *log, size_t size)
{
    /* add your code here */
    size_t written = xStreamBufferSend(log_streamBufferHandle, log, size, 0);// 在这里朝streambuffer写入数据，传递给SWO端口

    if (written < size)
    {
        // 可在此统计丢弃的字节
    }
}

/**
 * output lock
 */
void elog_port_output_lock(void)
{
    /* add your code here */
    osSemaphoreAcquire(sema_elogLock, osWaitForever);
}

/**
 * output unlock
 */
void elog_port_output_unlock(void)
{
    /* add your code here */
    osSemaphoreRelease(sema_elogLock);
}

/**
 * get current time interface
 *
 * @return current time
 */
const char *elog_port_get_time(void)
{
    /* add your code here */
    // static char cur_system_time[16] = "";
    // snprintf(cur_system_time, 16, "%lu", osKernelGetTickCount());
    // return cur_system_time;

    static char cur_system_time[64] = "";
    static rx8130ce_time_t now;
    dev_rx8130ceGetDateTime(&now);         // I2C blocked read and write
    snprintf(cur_system_time, 64, "%d.%d.%d %02d:%02d:%02d", now.year + 2000, now.month, now.day, now.hours, now.minutes, now.seconds);
    return cur_system_time;
}

/**
 * get current process name interface
 *
 * @return current process name
 */
const char *elog_port_get_p_info(void)
{

    /* add your code here */
    return "";
}

/**
 * get current thread name interface
 *
 * @return current thread name
 */
const char *elog_port_get_t_info(void)
{

    /* add your code here */
    return "";
}

void elog_async_output_notice(void)
{
    // osSemaphoreRelease(elog_asyncSem);
}

void elog_componentInit(void)
{
    /* log componment init */
    elog_init();
    elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL & ~ELOG_FMT_P_INFO);
    elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL & ~(ELOG_FMT_FUNC | ELOG_FMT_P_INFO | ELOG_FMT_DIR));
    elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_ALL & ~(ELOG_FMT_FUNC | ELOG_FMT_P_INFO));
    elog_start();
}

#ifdef ELOG_BUF_OUTPUT_ENABLE
void elog_entry(void *para)
{
    size_t get_log_size = 0;
#ifdef ELOG_ASYNC_LINE_OUTPUT
    static char poll_get_buf[ELOG_LINE_BUF_SIZE - 4];
#else
    static char poll_get_buf[ELOG_ASYNC_OUTPUT_BUF_SIZE - 4];
#endif
    for (;;)
    {
        /* waiting log */
        // osSemaphoreAcquire(elog_asyncSem, osWaitForever); // 异步输出的循环buffer中有数据，接收到elog_async_output_notice函数释放的信号量
        /* polling gets and outputs the log */
        while (1)
        {
#ifdef ELOG_ASYNC_LINE_OUTPUT
            get_log_size = elog_async_get_line_log(poll_get_buf, sizeof(poll_get_buf));
#else
            get_log_size = elog_async_get_log(poll_get_buf, sizeof(poll_get_buf));
#endif
            if (get_log_size)
            {
                elog_port_output(poll_get_buf, get_log_size);
            }
            else
            {
                break;
            }
        }
    }
}
#endif
