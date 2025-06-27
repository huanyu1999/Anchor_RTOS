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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "instance.h"
#include "app_sdCard.h"
#include "app_gnss.h"
#include "app_ethernet.h"
#include "dev.h"
#include "usart_voice.h"
#include "board_dw1000.h"
#include "drv_sdio.h"
#include "timer.h"

#include "com_multiButton.h"
#include "elog.h"
#include "dhcp.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 由于 CMSIS_OS2 没有封装静态创建task 或者queue需要用到的类型，自己定义，方便代码命名风格统一
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
#if TASK_INFO
TIM_HandleTypeDef timer50usHandle;
volatile uint32_t ulHighFrequencyTimerTicks = 0UL;
#endif

/**************************************************************CAN Communication**************************************************************/
uint32_t anchorCanExtId = 0xAAA0;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[5];

/**************************************************************Voice Output**************************************************************/
static int32_t voiceOutputDis = -1;

/**************************************************************Semaphore**************************************************************/
osSemaphoreId_t binSem;                                      // 用于dw1000中断同步
StaticSemaphore_t binSemCB;
const osSemaphoreAttr_t binSem_attr = { .name = "binSem", .cb_mem = &binSemCB, .cb_size = sizeof(StaticSemaphore_t) };

osSemaphoreId_t elog_lockSem;                                // 用于elog_lock
StaticSemaphore_t elog_lockSemCB;
const osSemaphoreAttr_t elog_lockSem_attr = { .name = "elog_lock", .cb_mem = &elog_lockSemCB, .cb_size = sizeof(StaticSemaphore_t) };

osSemaphoreId_t elog_asyncSem;                               // 用于elog_async
StaticSemaphore_t elog_asyncSemCB;
const osSemaphoreAttr_t elog_asyncSem_attr = { .name = "elog_async", .cb_mem = &elog_asyncSemCB, .cb_size = sizeof(StaticSemaphore_t) };

osSemaphoreId_t uart_dmaLockSem;                             // 用于elog串口dma发送同步，串口发送完成中断中释放，串口DMA发送前获取
StaticSemaphore_t uart_dmaLockSemCB;
const osSemaphoreAttr_t uart_dmaLockSem_attr = { .name = "uart_dmaLock", .cb_mem = &uart_dmaLockSemCB, .cb_size = sizeof(StaticSemaphore_t) };

osSemaphoreId_t gnssReceiveSem;
StaticSemaphore_t gnssReceiveSemCB;
const osSemaphoreAttr_t gnssReceiveSem_attr = { .name = "gnssReceive", .cb_mem = &gnssReceiveSemCB, .cb_size = sizeof(StaticSemaphore_t) };

/**************************************************************QueueMsg**************************************************************/
osMessageQueueId_t minDisQueue;                                 /* Definitions for minDisQueue */
outDistance_t     minDisQueueBuffer[3 * sizeof(outDistance_t)];
osStaticMessageQDef_t minDisQueueCB;
const osMessageQueueAttr_t minDisQueue_attr = {
    .name    = "minDisQueue",
    .cb_mem  = &minDisQueueCB, .cb_size = sizeof(minDisQueueCB),
    .mq_mem  = &minDisQueueBuffer, .mq_size = sizeof(minDisQueueBuffer)
};

osMessageQueueId_t canRxDisQueue;                                 /* Definitions for rxDisQueue */
outDistance_t      canRxDisQueueBuffer[4 * sizeof(outDistance_t)];
osStaticMessageQDef_t canRxDisQueueCB;
const osMessageQueueAttr_t canRxDisQueue_attr = {
    .name    = "canRxDisQueue",
    .cb_mem  = &canRxDisQueueCB, .cb_size = sizeof(canRxDisQueueCB),
    .mq_mem  = &canRxDisQueueBuffer, .mq_size = sizeof(canRxDisQueueBuffer)
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

osThreadId_t task2_voiceOut_Handle;                          /* Definitions for canRxTask_2 */
const osThreadAttr_t task2_voiceOut_attr = {
    .name = "task2_voiceOut",
    .stack_size = 128 * 2,
    .priority = (osPriority_t) osPriorityRealtime6,
};

osThreadId_t task3_canSend_Handle;
uint32_t canSendTask_buffer[512];
osStaticThreadDef_t canSendTaskCB;
const osThreadAttr_t task3_canSend_attr = {
    .name = " task3_canSend",
    .stack_mem = &canSendTask_buffer[0], .stack_size = sizeof(canSendTask_buffer),
    .cb_mem = &canSendTaskCB, .cb_size = sizeof(canSendTaskCB),
    .priority = (osPriority_t) osPriorityRealtime5,
};

osThreadId_t task4_Handle;
uint32_t task4_buffer[512];
osStaticThreadDef_t task4CB;
const osThreadAttr_t task4_attr = {
    .name = "task4",
    .stack_mem = &task4_buffer[0], .stack_size = sizeof(task4_buffer),
    .cb_mem = &task4CB, .cb_size = sizeof(task4CB),
    .priority = (osPriority_t) osPriorityRealtime6,
};

osThreadId_t task5_Handle;
const osThreadAttr_t task5_attr = {
    .name = "task5",
    .stack_size = 256 * 2,
    .priority = (osPriority_t) osPriorityRealtime6,
};

osThreadId_t task6_logManage_Handle;
uint32_t logManageTask_buffer[256];
osStaticThreadDef_t logManageTaskCB;
const osThreadAttr_t task6_logManage_attr = {
    .name = "logManageTask",
    .stack_mem = &logManageTask_buffer[0], .stack_size = sizeof(logManageTask_buffer),
    .cb_mem = &logManageTaskCB, .cb_size = sizeof(logManageTaskCB),
    .priority = (osPriority_t) osPriorityRealtime5
};

osThreadId_t task7_Handle;
uint32_t  task7_buffer[256];
osStaticThreadDef_t task7CB;
const osThreadAttr_t task7_attr = {
    .name = " task7",
    .stack_mem = &task7_buffer[0], .stack_size = sizeof(task7_buffer),
    .cb_mem = &task7CB, .cb_size = sizeof(task7CB),
    .priority = (osPriority_t) osPriorityRealtime5
};

osThreadId_t task8_Handle;
uint32_t  task8_buffer[256];
osStaticThreadDef_t task8CB;
const osThreadAttr_t task8_attr = {
    .name = " task8",
    .stack_mem = &task8_buffer[0], .stack_size = sizeof(task8_buffer),
    .cb_mem = &task8CB, .cb_size = sizeof(task8CB),
    .priority = (osPriority_t) osPriorityRealtime5
};

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void task1_anchorDisHandling(void *arg);
void task2_voiceOut(void *arg);
void task3_canSend(void *arg);
void task_Test(void *arg); 
void task_canReceiveHandle(void *arg);

static void split32to8(int32_t value, uint8_t *bytes); 
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
    minDisQueue     = osMessageQueueNew(3, sizeof(outDistance_t), &minDisQueue_attr);
    canRxDisQueue   = osMessageQueueNew(3, sizeof(outDistance_t), &canRxDisQueue_attr);
    binSem          = osSemaphoreNew(1, 0, &binSem_attr);
    elog_lockSem    = osSemaphoreNew(1, 1, &elog_lockSem_attr);                 // 该二值信号量初始值必须设置为1
    elog_asyncSem   = osSemaphoreNew(1, 1, &elog_asyncSem_attr); 
    uart_dmaLockSem = osSemaphoreNew(1, 0, &uart_dmaLockSem_attr);  
    gnssReceiveSem  = osSemaphoreNew(1, 0, &gnssReceiveSem_attr);
    /* USER CODE END RTOS_QUEUES */

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
//    task0_uwb_Handle               = osThreadNew(task0_uwb, NULL, &task0_uwb_attr);
//    task1_anchorDisHandling_Handle = osThreadNew(task1_anchorDisHandling, NULL, &task1_anchorDisHandling_attr);
//    task2_voiceOut_Handle          = osThreadNew(task2_voiceOut, NULL, &task2_voiceOut_attr);
//    task3_canSend_Handle           = osThreadNew(task3_canSend, NULL, &task3_canSend_attr);
//    task4_Handle                   = osThreadNew(task_tagDistInsertAndUpdate, NULL, &task4_attr);
//    task5_Handle                   = osThreadNew(task_tagDistClearInvalid, NULL, &task5_attr);
    task6_logManage_Handle         = osThreadNew(elog_entry, NULL, &task6_logManage_attr);
    task7_Handle                   = osThreadNew(task_eth, NULL, &task7_attr);               // 定为GNSS 模组同步时间用，暂用为测试任务
//    task8_Handle                   = osThreadNew(task_canReceiveHandle, NULL, &task8_attr);

#if TASK_INFO
    drv_setTimerForInt(&timer50usHandle, TIM4, 20000, 6);
#endif

    /* USER CODE END RTOS_THREADS */
}

uint32_t flag;
uint8_t voice_clear_left[] = "clear voice L";

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  距离输出处理，此任务需要跟堆更新任务进行同步
  * @param  none 
  * @retval none
  */
void task1_anchorDisHandling(void *arg) 
{
    osStatus status;
    int32_t anchorSelfDis  = 2000000;
    uint8_t anchorSelfIdx = 0xFF;
    int32_t anchorRxDis    = 2000000;
    uint8_t anchorRxIdx = 0xFF;
    int32_t anchorFinalDis = 2000000;
    uint8_t anchorFinalIdx = 0xFF;
    static uint8_t canSendBuf[5];           // CAN 发送缓存设置为5位，前面4位存储距离，最后1位存储ID
    for (;;)
    {
        dwDistance_t *distance = get_the_local_structure_of_dis();
        /* 存储最小距离，通过队列发送出去 */ 
        status = osMessageQueueGet(minDisQueue, &distance->disMsg[0], 0, 100);
        if ((status == osOK) && (distance->disMsg[0].dis_class == ANCHOR_SELF_DIS))
        {
            anchorSelfDis = distance->disMsg[0].dis_value;
            anchorSelfIdx = distance->disMsg[0].dis_index;
            split32to8(anchorSelfDis, canSendBuf);
            canSendBuf[4] = anchorSelfIdx;
            dev_canSendMsg(anchorCanExtId, canSendBuf, ARRAY_LENGTH(canSendBuf));
        }

        /* 从CAN接收中断中获取对侧基站最小距离 */
        status = osMessageQueueGet(canRxDisQueue, &distance->disMsg[1], 0, 100);
        if ((status == osOK) && (distance->disMsg[1].dis_class == ANCHOR_OTHER_DIS))
        {
            anchorRxDis = distance->disMsg[1].dis_value; // 获取接收的距离
            anchorRxIdx = distance->disMsg[1].dis_index;
        }
        
        if (anchorSelfDis > anchorRxDis)
        {
            dev_ledOn(across_led);
            dev_ledOff(onside_led);
            anchorFinalDis = anchorRxDis;                   /* 最近的标签位于对侧基站*/
            anchorFinalIdx = anchorRxIdx; 
        }
        else
        {
            dev_ledOff(across_led);
            dev_ledOn(onside_led);
            anchorFinalDis = anchorSelfDis;                 /* 最近的标签位于本侧基站*/
            anchorFinalIdx = anchorSelfIdx;
        }
        
        voiceOutputDis = anchorFinalDis;

        log_d("SelfDis ID %d %.2f, RxDis ID %d %.2f, FinalDis ID %d %.2f \n", 
                anchorSelfIdx, (float)(anchorSelfDis) / 1000,
                anchorRxIdx, (float)(anchorRxDis) / 1000, 
                anchorFinalIdx, (float)(anchorFinalDis) / 1000);
        osDelay(450);       /* 定时处理距离数据 */
    }
}

/**
  * @brief  
  * @param  none 
  * @retval none
  */
void task2_voiceOut(void *arg)
{
    uint32_t tick;
    tick = osKernelGetTickCount();

    for (;;)
    {
        if (voiceOutputDis < 100000)
        {
            // dev_buzzerOpen(buzzer);
            // 这里也应该采用开关外设的方法来控制
        }
        else 
        {
            dev_buzzerClose(buzzer);
        }
        
        Report_Dis((float)voiceOutputDis / 1000.0); // 在此处直接输出最终的距离，后续调试看情况是否需要加上互斥量保护

        tick += 1100;
        osDelayUntil(tick);
    }
}

void task3_canSend(void *arg)
{   
    uint32_t CanExtId4Button = 0xAAA1;
    for (;;)
    {
        osThreadFlagsWait(0x03, osFlagsWaitAny, osWaitForever);
        uint8_t buttonVal = dev_buttonRead(BUTTON_ID_SWITCH);
        dev_canSendMsg(CanExtId4Button, &buttonVal, 1);
        log_d("switch button value sent.");
    }
}

void task_Test(void *arg)
{
    // char taskListBuffer[512];
    // vTaskList(taskListBuffer);
    // log_i("Task list:\n%s", taskListBuffer);
    // sdCard_readWriteDemo();
    for(;;) 
    {
        // 闪烁两个LED
        dev_ledBlink(uwb_ok_led);
        // dev_ledBlink(onside_led);
        // dev_ledOn(onside_led);
        // dev_ledOn(across_led);
        osDelay(1000);
    }
}

void task_canReceiveHandle(void *arg)
{
    outDistance_t rxMsg;
    static int32_t anchorReceiveDis = 2000000;  /* 用于储存基站接收到的距离 */
    for (;;)
    {
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

        if ((RxHeader.ExtId == 0xAAA0) && (RxHeader.IDE == CAN_ID_EXT))
        {
            anchorReceiveDis = combine8to32(RxData);             // CAN正确接收，填充距离
            rxMsg.dis_class = ANCHOR_OTHER_DIS;
            rxMsg.dis_value = anchorReceiveDis;
            rxMsg.dis_index = RxData[4];                        // 获取存储的标签ID
            osMessageQueuePut(canRxDisQueue, &rxMsg, 0, 0);
            dev_ledBlink(can_rx_led);
        } 
        else if ((RxHeader.ExtId == 0xAAA1) && (RxHeader.IDE == CAN_ID_EXT))        
        {
            // 接收到的为旋钮控制信息，校验两侧旋钮键值，播报还是静音
            uint8_t rxbutton_val = RxData[0];
            uint8_t selfButton_val =  dev_buttonRead(BUTTON_ID_SWITCH);
            log_d("receive button_val : %d", rxbutton_val);

            if (rxbutton_val == selfButton_val)
            {
                // 两侧键值相同，打开播报
                log_d("receive button_val same, open voice");
                dev_buzzerInit(buzzer);
            }
            else
            {
                // 两侧键值不相同，关闭播报
                log_d("receive button_val not same, close voice");
                dev_buzzerClose(buzzer);
            }
        }
    }
}

/*************************************************some interrupt callback function*************************************************/

/**
  * @brief  Rx Fifo 0 message pending callback
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)      /* Get RX message */
    {
        osThreadFlagsSet(task8_Handle, 0x01);
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
    else if (htim->Instance == TIM2)
    {
        extern osSemaphoreId_t tagDistClearSem;
        if (osSemaphoreRelease(tagDistClearSem) == osOK)
        {
            // log_d("tdmaCycleTimer_CallBack");
        }
    }

    else if (htim->Instance == TIM4)
    {
#if TASK_INFO
        ulHighFrequencyTimerTicks++;
#endif
        // log_d("dhcp 1 second test.");
        DHCP_time_handler();
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
        board_dw1000SetSignalReset();
    }
    else if (GPIO_Pin == Dw1000_IRQ_Pin)
    {
        osSemaphoreRelease(binSem);    // 在这里释放信号量
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) 
{
    if (huart->Instance == USART1)                     
    {
        osSemaphoreRelease(uart_dmaLockSem);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) 
    {
        osSemaphoreRelease(uart_dmaLockSem);
    }
}

/*************************************************some static function*************************************************/
// 将32位整数分解为4个8位整数
static void split32to8(int32_t value, uint8_t *bytes) 
{
    bytes[0] = (value >> 24) & 0xFF; // 高8位
    bytes[1] = (value >> 16) & 0xFF; // 次高8位
    bytes[2] = (value >> 8) & 0xFF;  // 次低8位
    bytes[3] = value & 0xFF;         // 低8位
}

// 将4个8位整数组合成1个32位整数
static uint32_t combine8to32(const uint8_t *bytes) 
{
    return ((uint32_t)bytes[0] << 24) | 
           ((uint32_t)bytes[1] << 16) | 
           ((uint32_t)bytes[2] << 8)  | 
           (uint32_t)bytes[3];
}

/* USER CODE END Application */
