#include "app_gnss.h"
#include "dev_gnss.h"
#include "usart.h"
#include "cmsis_os.h"

void task_gnssModule(void* arg)
{
    MX_UART3_Init();
    dev_gnssModStartRx();
    for (;;)
    {
        extern osSemaphoreId_t gnssReceiveSem;
        osSemaphoreAcquire(gnssReceiveSem, osWaitForever); // 当串口空闲中断触发时中执行该任务
        dev_gnssModReceiveAndParse();
        dev_gnssModStartRx();
    }
}
