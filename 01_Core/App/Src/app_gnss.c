#include "app.h"
#include "dev.h"

#include "cmsis_os2.h"

void task_gnssModule(void* arg)
{
    for (;;)
    {
        extern osSemaphoreId_t gnssReceiveSem;
        osSemaphoreAcquire(gnssReceiveSem, osWaitForever);
        dev_gnssModReceiveAndParse();
        dev_gnssModStartRx();
    }
}

