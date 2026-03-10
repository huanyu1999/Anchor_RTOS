#include "app_gnss.h"
#include "dev_gnss.h"
#include "usart.h"
#include "cmsis_os.h"

void task_gnssSyncTime(void* arg)
{
    UNUSED(arg);
    uint32_t tick;
    tick = osKernelGetTickCount();
    MX_UART3_Init();
    dev_gnssModStartRx();
    dev_gnssModReceiveAndParse();
    for (;;)
    {
        extern osSemaphoreId_t sema_gnssReceive;
        osSemaphoreAcquire(sema_gnssReceive, osWaitForever); // 当串口空闲中断触发时中执行该任务
        // 检测GNSS 定位有效引脚电平，
        dev_gnssModReceiveAndParse();
        dev_gnssModStartRx();
        tick += 10000;                                  // 固定10s钟进行一次同步
        osDelayUntil(tick);
    }
}
