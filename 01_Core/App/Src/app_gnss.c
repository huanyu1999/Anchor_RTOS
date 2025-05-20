#include "app_gnss.h"
#include "dev_gnss.h"

#include "cmsis_os.h"

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
