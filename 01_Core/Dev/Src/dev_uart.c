#include "usart.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <stdarg.h>

// 使用双buffer的方法,单个buffer会导致连续打印数据时，数据重叠，丢失的异常现象
uint8_t debug_buff1[256];
uint8_t debug_buff2[256];
static uint8_t *current_buff = debug_buff1; 

void printf_use_dma(const char *format, ...)
{
    // uint8_t *send_buff = current_buff;
    // current_buff = (current_buff == debug_buff1) ? debug_buff2 : debug_buff1;
    // va_list args;
    
    // va_start(args, format);
    // uint32_t length = vsnprintf((char*) send_buff, sizeof(debug_buff1) + 1, (char*) format, args);
    // va_end(args);
    // extern osSemaphoreId_t uart_dmaLockSem;
    // osSemaphoreAcquire(uart_dmaLockSem, osWaitForever);
    // HAL_UART_Transmit_DMA(&huart1, send_buff, length);
}
