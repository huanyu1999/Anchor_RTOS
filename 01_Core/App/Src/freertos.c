/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "ff.h"
#include "ffconf.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "instance.h"
#include "usart.h"
#include "usart_voice.h"
#include "timer.h"
#include "board_dw1000.h"
#include "dev.h"
#include "drv_sdio.h"
#include "elog.h"

#include "com_multiButton.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//由于 CMSIS_OS2 没有封装静态创建task 或者queue需要用到的类型，自己定义，方便代码命名风格统一
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/**************************************************************CAN Communication**************************************************************/
uint32_t anchorCanExtId = 0xAAA0;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

/**************************************************************FatFs support**************************************************************/
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD card logical drive path */
static uint8_t workBuffer[FF_MAX_SS]; /* a work buffer for the f_mkfs() */


/**************************************************************Voice Output**************************************************************/
static int32_t voiceOutputDis = 0;

/**************************************************************Semaphore**************************************************************/
osSemaphoreId_t binSem;                                         // 用于dw1000中断同步
StaticSemaphore_t binSemCB;
const osSemaphoreAttr_t binSem_attr = {
    .name = "binSem",
    .cb_mem = &binSemCB,
    .cb_size = sizeof(StaticSemaphore_t)
};

osSemaphoreId_t elog_lockSem;                                // 用于elog_lock
StaticSemaphore_t elog_lockSemCB;
const osSemaphoreAttr_t elog_lockSem_attr = {
    .name = "elog_lock",
    .cb_mem = &elog_lockSemCB,
    .cb_size = sizeof(StaticSemaphore_t)
};

osSemaphoreId_t elog_asyncSem;                               // 用于elog_async
StaticSemaphore_t elog_asyncSemCB;
const osSemaphoreAttr_t elog_asyncSem_attr = {
    .name = "elog_async",
    .cb_mem = &elog_asyncSem,
    .cb_size = sizeof(StaticSemaphore_t)
};

osSemaphoreId_t elog_dmaLockSem;
StaticSemaphore_t elog_dmaLockSemCB;
const osSemaphoreAttr_t elog_dmaLockSem_attr = {
    .name = "elog_dmaLock",
    .cb_mem = &elog_dmaLockSem,
    .cb_size = sizeof(StaticSemaphore_t)
};

/**************************************************************QueueMsg**************************************************************/
osMessageQueueId_t minDisQueue;                                 /* Definitions for minDisQueue */
outDistance_t     minDisQueueBuffer[6 * sizeof(outDistance_t)];
osStaticMessageQDef_t minDisQueueCB;
const osMessageQueueAttr_t minDisQueue_attr = {
    .name    = "minDisQueue",
    .cb_mem  = &minDisQueueCB,
    .cb_size = sizeof(minDisQueueCB),
    .mq_mem  = &minDisQueueBuffer,
    .mq_size = sizeof(minDisQueueBuffer)
};

osMessageQueueId_t rxDisQueue;                                 /* Definitions for rxDisQueue */
outDistance_t     rxDisQueueBuffer[6 * sizeof(outDistance_t)];
osStaticMessageQDef_t rxDisQueueCB;
const osMessageQueueAttr_t rxDisQueue_attr = {
    .name    = "rxDisQueue",
    .cb_mem  = &rxDisQueueCB,
    .cb_size = sizeof(rxDisQueueCB),
    .mq_mem  = &rxDisQueueBuffer,
    .mq_size = sizeof(rxDisQueueBuffer)
};

/**************************************************************Thread**************************************************************/
osThreadId_t task0_uwb_Handle;                                /* Definitions for dw1000Task_0 */
const osThreadAttr_t task0_uwb_attr = {
    .name = "task0_uwb", 
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityRealtime7,
};

osThreadId_t task1_anchorDisHandling_Handle;                 /* Definitions for calDistanceTask */
const osThreadAttr_t task1_anchorDisHandling_attr = {
    .name = "task1_anchorDisHandling",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityRealtime6,
};

osThreadId_t task2_voiceOut_Handle;                             /* Definitions for canRxTask_2 */
const osThreadAttr_t task2_voiceOut_attr = {
    .name = "task2_voiceOut",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityRealtime4,
};

osThreadId_t task3_canSend_Handle;
uint32_t canSendTask_buffer[512];
osStaticThreadDef_t canSendTaskCB;
const osThreadAttr_t task3_canSend_attr = {
    .name = " task3_canSend",
    .stack_mem = &canSendTask_buffer[0],
    .stack_size = sizeof(canSendTask_buffer),
    .cb_mem = &canSendTaskCB,
    .cb_size = sizeof(canSendTaskCB),
    .priority = (osPriority_t) osPriorityRealtime5,
};

osThreadId_t task4_Handle;
const osThreadAttr_t task4_attr = {
    .name = "task4",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityRealtime5,
};

osThreadId_t task5_findMinDis_Handle;
const osThreadAttr_t task5_findMinDis_attr = {
    .name = "task5_findMinDis",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityRealtime6,
};

osThreadId_t task6_logManage_Handle;
uint32_t logManageTask_buffer[256];
osStaticThreadDef_t logManageTaskCB;
const osThreadAttr_t task6_logManage_attr = {
    .name = "logManageTask",
    .stack_mem = &logManageTask_buffer[0],
    .stack_size = sizeof(logManageTask_buffer),
    .cb_mem = &logManageTaskCB,
    .cb_size = sizeof(logManageTaskCB),
    .priority = (osPriority_t) osPriorityRealtime5
};

osThreadId_t task7_sdCard_Handle;
uint32_t sdCardTask_buffer[256];
osStaticThreadDef_t sdCardTaskCB;
const osThreadAttr_t task7_sdCard_attr = {
    .name = " sdCardTask",
    .stack_mem = & sdCardTask_buffer[0],
    .stack_size = sizeof(sdCardTask_buffer),
    .cb_mem = & sdCardTaskCB,
    .cb_size = sizeof(sdCardTaskCB),
    .priority = (osPriority_t) osPriorityRealtime5
};

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void task0_uwb(void *argument);
void task1_anchorDisHandling(void *argument);
void task2_voiceOut(void *argument);
void task3_canSend(void *argument);
void task4_timerYield(void *argument);
void task5_findMinDis(void *argument);
void task7_sdCard(void *argument);

static void split32to8(uint32_t value, uint8_t *bytes);
static uint32_t combine8to32(const uint8_t *bytes);

/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) 
{
    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    minDisQueue = osMessageQueueNew(3, sizeof(outDistance_t), &minDisQueue_attr);
    rxDisQueue  = osMessageQueueNew(3, sizeof(outDistance_t), &rxDisQueue_attr);
    binSem          = osSemaphoreNew(1, 0, &binSem_attr);
    elog_lockSem    = osSemaphoreNew(1, 1, &elog_lockSem_attr);                 // 该二值信号量初始值必须设置为1
    // elog_asyncSem   = osSemaphoreNew(1, 0, &elog_asyncSem_attr);             // 这两个信号量创建暂时会出问题，后续解决
    // elog_dmaLockSem = osSemaphoreNew(1, 0, &elog_dmaLockSem_attr);   
    /* USER CODE END RTOS_QUEUES */

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    task0_uwb_Handle               = osThreadNew(task0_uwb, NULL, &task0_uwb_attr);
    task1_anchorDisHandling_Handle = osThreadNew(task1_anchorDisHandling, NULL, &task1_anchorDisHandling_attr);
    task2_voiceOut_Handle          = osThreadNew(task2_voiceOut, NULL, &task2_voiceOut_attr);
    task3_canSend_Handle           = osThreadNew(task3_canSend, NULL, &task3_canSend_attr);
    task4_Handle                   = osThreadNew(task4_timerYield, NULL, &task4_attr);
    task5_findMinDis_Handle = osThreadNew(task5_findMinDis, NULL, &task5_findMinDis_attr);
    task6_logManage_Handle = osThreadNew(elog_entry, NULL, &task6_logManage_attr);
    task7_sdCard_Handle = osThreadNew(task7_sdCard, NULL, &task7_sdCard_attr);

    /* USER CODE END RTOS_THREADS */
}

uint32_t flag;
uint8_t voice_clear_left[] = "clear voice L";
uint32_t anchorCanExtId1 = 0xAAA1;

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


int32_t anchorSelfDis  = 2000000;
int32_t anchorRxDis    = 2000000;
int32_t anchorFinalDis = 2000000;
/**
  * @brief  距离输出处理，此任务需要跟堆更新任务进行同步
  * @param  none 
  * @retval none
  */
void task1_anchorDisHandling(void *argument) 
{
    osStatus status;
    bool status1;
    
    for (;;)
    {
        dwDistance_t* distance =  get_the_local_structure_of_dis();
        status1 = heap_peek_root(&distance->dis_min_heap, &distance->min_dis, &distance->dis_idx);     // 堆更新完毕后，采集一次堆顶，也就是最小值

        if (status1 == true)
        {
            anchorSelfDis = distance->min_dis;      // 获取最小距离
        }
        else                                        // 堆为空的时候，说明没有有效距离，将该距离设置为无效值
        {
            anchorSelfDis = 2000000;
        }
        
        /* 存储最小距离，通过队列发送出去 */ 
        distance->disMsg[0].dis_class = ANCHOR_SELF_DIS;
        distance->disMsg[0].dis_value = anchorSelfDis;
        status = osMessageQueuePut(minDisQueue, &distance->disMsg[0], 0, 100);
        
        /* 从CAN接收中断中获取对侧基站最小距离 */
        status = osMessageQueueGet(rxDisQueue, &distance->disMsg[1], 0, 100);
        if ((status == osOK) && (distance->disMsg[1].dis_class == OTHER_ANCHOR_DIS))
        {
            anchorRxDis = distance->disMsg[1].dis_value; // 获取接收的距离
        }
        
        if (anchorSelfDis > anchorRxDis)
        {
            anchorFinalDis = anchorRxDis;                   /* 最近的标签位于对侧基站*/
            dev_ledOn(across_led);
            dev_ledOff(onside_led);
        }
        else
        {
            anchorFinalDis = anchorSelfDis;                 /* 最近的标签位于本侧基站*/
            dev_ledOff(across_led);
            dev_ledOn(onside_led);
        }
        
        voiceOutputDis = anchorFinalDis;

        log_d("SelfDis %.2f, RxDis %.2f, FinalDis %.2f \n", (float)(anchorSelfDis) / 1000, (float)(anchorRxDis) / 1000, (float)(anchorFinalDis) / 1000);
        osDelay(400);       /* 定时处理距离数据 */
    }
}

/**
  * @brief  
  * @param  none 
  * @retval none
  */
void task2_voiceOut(void *argument)
{
    uint32_t tick;

    tick = osKernelGetTickCount();
    for (;;)
    {
        if (voiceOutputDis < 100000)
        {
            dev_buzzerOpen(buzzer);
        }
        else 
        {
            dev_buzzerClose(buzzer);
        }
        
        Report_Dis((float)voiceOutputDis / 1000.0);
        tick += 1100;
        osDelayUntil(tick);
    }
}

void task3_canSend(void *argument)
{   
    outDistance_t sendDis;
    osStatus_t status;
    static uint8_t canSendBuf[4];
    for (;;)
    {
        status = osMessageQueueGet(minDisQueue, &sendDis, 0, portMAX_DELAY);
        if (status  == osOK)
        {
            split32to8(sendDis.dis_value, canSendBuf);
            dev_canSendMsg(anchorCanExtId, canSendBuf, 4);                                 // CAN 发送基站本测最小距离值(这句有问题)
        }
    }
}

void task4_timerYield(void *argument)
{
    for (;;)
    {
        multiTimerYield();
    }
}

void task5_findMinDis(void *argument)
{
    for (;;)
    {
        if (osThreadFlagsWait(0x00000001U, osFlagsWaitAll, osWaitForever))  // 等待定时任务发送的任务通知
        {
            tag_distance_handler();
        }
    }
}

void task7_sdCard(void *argument)
{
    FRESULT res;                                          /* FatFs function common result code */
    uint32_t byteswritten, bytesread;                     /* File write/read counts */
    uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
    uint8_t rtext[100];                                   /* File read buffer */

    /*##-1- Link the micro SD disk I/O driver ##################################*/
    if (dev_FATFS_LinkDriver(&SDCard_driver, SDPath) == 0)
    {
        /*##-2- Register the file system object to the FatFs module ##############*/
        if (f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
        {
            /* FatFs Initialization Error */
            log_e("f_mount failed.");
        }
        else
        {
            /*##-3- Create a FAT file system (format) on the logical drive #########*/
            /* WARNING: Formatting the uSD card will delete all content on the device */
            if(f_mkfs((TCHAR const*)SDPath, FM_FAT32, 0, workBuffer, sizeof(workBuffer)) != FR_OK)
            {
                /* FatFs Format Error */
                // Error_Handler();
                log_e("f_mkfs failed.");
            }
            else
            {
                /*##-4- Create and Open a new text file object with write access #####*/
                if(f_open(&MyFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
                {       
                    /* 'STM32.TXT' file Open for write Error */
                    // Error_Handler();
                    log_e("f_open failed.");
                }
                else
                {
                    /*##-5- Write data to the text file ################################*/
                    res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);

                    if((byteswritten == 0) || (res != FR_OK))
                    {
                        /* 'STM32.TXT' file Write or EOF Error */
                        // Error_Handler();
                        log_e("'STM32.TXT' file Write or EOF Error.");
                    }
                    else
                    {
                        /*##-6- Close the open text file #################################*/
                        f_close(&MyFile);
            
                        /*##-7- Open the text file object with read access ###############*/
                        if(f_open(&MyFile, "STM32.TXT", FA_READ) != FR_OK)
                        {
                            /* 'STM32.TXT' file Open for read Error */
                            log_e("'STM32.TXT' file Open for read Error.");
                        }
                        else
                        {
                            /*##-8- Read data from the text file ###########################*/
                            res = f_read(&MyFile, rtext, sizeof(rtext), (UINT*)&bytesread);
                            
                            if((bytesread == 0) || (res != FR_OK))
                            {
                                /* 'STM32.TXT' file Read or EOF Error */
                                log_e("'STM32.TXT' file Read or EOF Error.");Error_Handler();
                            }
                            else
                            {
                                /*##-9- Close the open text file #############################*/
                                f_close(&MyFile);
                                
                                /*##-10- Compare read data with the expected data ############*/
                                if((bytesread != byteswritten))
                                {                
                                    /* Read data is different from the expected data */
                                    log_e("Read data is different from the expected data.");
                                }
                                else
                                {
                                    /* Success of the demo: no error occurrence */
                                    // BSP_LED_On(LED1);
                                    log_d("Success of the sd card demo.");
                                }
                            }       
                        }
                    }
                }
            }
        }
    }

    /*##-11- Unlink the RAM disk I/O driver ####################################*/
    dev_FATFS_UnLinkDriver(SDPath);

    for (;;)
    {
    }
}



/*************************************************some callback function*************************************************/

/**
  * @brief  Rx Fifo 0 message pending callback
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    static int32_t anchorReceiveDis = 2000000;  /* 用于储存基站接收到的距离 */
    outDistance_t rxMsg;

    memset(&RxHeader, 0, sizeof(RxHeader));
    memset(&RxData, 0, sizeof(RxData));

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)      /* Get RX message */
    {
        if ((RxHeader.ExtId == 0xAAA0) && (RxHeader.IDE == CAN_ID_EXT))
        {
            anchorReceiveDis = combine8to32(RxData);             // CAN正确接收，填充距离
            rxMsg.dis_class = OTHER_ANCHOR_DIS;
            rxMsg.dis_value = anchorReceiveDis;
            osMessageQueuePut(rxDisQueue, &rxMsg, 0, 0);
            dev_ledBlink(can_rx_led);
        } 
        else if ((RxHeader.ExtId == 0xAAA1) && (RxHeader.IDE == CAN_ID_EXT))
        {
            HAL_GPIO_DeInit(BUZZER_GPIO_Port, BUZZER_Pin);
            HAL_UART_DeInit(&huart4);
        }
    } 
    else
    {
        Error_Handler();                    /* Reception Error */
    }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        button_ticks();
    }
    else if (htim->Instance == TIM6)
    {
        HAL_IncTick();  // HAL延时所使用的时基
    }
}

/* @fn		HAL_GPIO_EXTI_Callback
 * @brief	IRQ HAL call-back for all EXTI configured lines
 * 			i.e. DW_RESET_Pin and DW_IRQn_Pin
 * */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == Dw1000_RSTn_Pin)
    {
        // port_set_signalReset();
        board_dw1000SetSignalReset();
    }
    else if (GPIO_Pin == Dw1000_IRQ_Pin)
    {
        osSemaphoreRelease(binSem);    // 在这里释放信号量
    }
}

/*************************************************some static function*************************************************/
// 将32位整数分解为4个8位整数
static void split32to8(uint32_t value, uint8_t *bytes) {
    bytes[0] = (value >> 24) & 0xFF; // 高8位
    bytes[1] = (value >> 16) & 0xFF; // 次高8位
    bytes[2] = (value >> 8) & 0xFF;  // 次低8位
    bytes[3] = value & 0xFF;         // 低8位
}

// 将4个8位整数组合成1个32位整数
static uint32_t combine8to32(const uint8_t *bytes) {
    return ((uint32_t)bytes[0] << 24) | 
           ((uint32_t)bytes[1] << 16) | 
           ((uint32_t)bytes[2] << 8)  | 
           (uint32_t)bytes[3];
}

/* USER CODE END Application */
