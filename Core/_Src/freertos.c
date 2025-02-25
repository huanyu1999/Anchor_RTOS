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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "instance.h"
#include "usart.h"
#include "usart_voice.h"
#include "can.h"
#include "timer.h"
#include "gpio.h"
#include "com_multiButton.h"

extern void dw_main(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
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

/*******************************CAN Communication*******************************/
uint32_t anchorCanExtId = 0xAAA0;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
uint8_t canSendBuf[8];
static int32_t voiceOutputDis = 0;

osMessageQueueId_t minDisQueue;                                 /* Definitions for minDisQueue */
distanceData_t     minDisQueueBuffer[6 * sizeof(distanceData_t)];
osStaticMessageQDef_t minDisQueueCB;
const osMessageQueueAttr_t minDisQueue_attr = {
    .name    = "minDisQueue",
    .cb_mem  = &minDisQueueCB,
    .cb_size = sizeof(minDisQueueCB),
    .mq_mem  = &minDisQueueBuffer,
    .mq_size = sizeof(minDisQueueBuffer)
};

osMessageQueueId_t rxDisQueue;                                 /* Definitions for rxDisQueue */
distanceData_t     rxDisQueueBuffer[6 * sizeof(distanceData_t)];
osStaticMessageQDef_t rxDisQueueCB;
const osMessageQueueAttr_t rxDisQueue_attr = {
    .name    = "rxDisQueue",
    .cb_mem  = &rxDisQueueCB,
    .cb_size = sizeof(rxDisQueueCB),
    .mq_mem  = &rxDisQueueBuffer,
    .mq_size = sizeof(rxDisQueueBuffer)
};

osThreadId_t task0_uwb_Handle;                                /* Definitions for dw1000Task_0 */
const osThreadAttr_t task0_uwb_attr = {
    .name = "task0_uwb", 
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityRealtime7,
};

osThreadId_t task1_anchorDisHandling_Handle;                 /* Definitions for calDistanceTask */
const osThreadAttr_t task1_anchorDisHandling_attr = {
    .name = "task1_anchorDisHandling",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityRealtime7,
};

osThreadId_t task2_canRx_Handle;                             /* Definitions for canRxTask_2 */
const osThreadAttr_t task2_canRx_attr = {
    .name = "task2_canRx",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityRealtime7,
};

osThreadId_t task3_canSend_Handle;
const osThreadAttr_t task3_canSend_attr = {
    .name = " task3_canSend",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityRealtime7,
};

osThreadId_t task4_Handle;
const osThreadAttr_t task4_attr = {
    .name = "task4",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityRealtime7,
};

/* USER CODE END Variables */

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityRealtime7,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartDefaultTask(void *argument);
void task0_uwb(void *argument);
void task1_anchorDisHandling(void *argument);
void task2_canRx(void *argument);
void task3_canSend(void *argument);
void task4_timerYield(void *argument);

void split32to8(uint32_t value, uint8_t *bytes);
uint32_t combine8to32(const uint8_t *bytes);

void clearReceiveDis(void);

/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) 
{
    /* USER CODE BEGIN Init */
    
    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    minDisQueue = osMessageQueueNew(3, sizeof(distanceData_t), &minDisQueue_attr);
    rxDisQueue = osMessageQueueNew(3, sizeof(distanceData_t), &rxDisQueue_attr);
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    // defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    task0_uwb_Handle               = osThreadNew(task0_uwb, NULL, &task0_uwb_attr);
    task1_anchorDisHandling_Handle = osThreadNew(task1_anchorDisHandling, NULL, &task1_anchorDisHandling_attr);
    task2_canRx_Handle             = osThreadNew(task2_canRx, NULL, &task2_canRx_attr);
    task3_canSend_Handle           = osThreadNew(task3_canSend, NULL, &task3_canSend_attr);
    task4_Handle                   = osThreadNew(task4_timerYield, NULL, &task4_attr);
    if(task0_uwb_Handle == NULL) { Error_Handler(); }
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

    uint32_t flag;
    uint8_t voice_clear_left[] = "clear voice L";
    uint32_t anchorCanExtId1 = 0xAAA1;
/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */

    /* Infinite loop */
    for(;;)
    {
        flag = osThreadFlagsWait(0x0000003U, osFlagsWaitAny, osWaitForever);
        if(flag & (1 << 0))
        {
            canSendMsg(anchorCanExtId1, voice_clear_left, ARRAY_LENGTH(voice_clear_left));
            printf_use_dma("send sound clear message.\r\n");
        }
        else if(flag & (1 << 1))
        {
            // CAN接收到消音信息，消音
            printf_use_dma("task excute clear message.\r\n");
            HAL_GPIO_DeInit(BUZZER_GPIO_Port, BUZZER_Pin);
            HAL_UART_DeInit(&huart4);
        }
    }
    /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief 
  * @param  none 
  * @retval none
  */
void task0_uwb(void *argument) 
{
    dw_main();
}

int32_t anchorSelfDis  = 2000000;
int32_t anchorRxDis    = 2000000;
int32_t anchorFinalDis = 2000000;
/**
  * @brief  
  * @param  none 
  * @retval none
  */
void task1_anchorDisHandling(void *argument) 
{
    distanceData_t selfDisMsg;              // 用于消息队列发送
    distanceData_t rxDisMsg;                // 用于消息队列接收
    osStatus status;
    // uint32_t time = portTICK_RATE_MS(100);
    
    for(;;)
    {
        anchorSelfDis = findMin(sort_distance, MAX_TAG_LIST_SIZE);              // 获取最小距离
        
        /* 存储最小距离，通过队列发送出去 */ 
        selfDisMsg.dis_class = ANCHOR_SELF_DIS;
        selfDisMsg.dis_value = anchorSelfDis;
        status = osMessageQueuePut(minDisQueue, &selfDisMsg, 0, 100);
        
        /* 从CAN接收中断中获取对侧基站最小距离 */
        status = osMessageQueueGet(rxDisQueue, &rxDisMsg, 0, 100);
        if((status == osOK) && (rxDisMsg.dis_class == OTHER_ANCHOR_DIS))
        {
            anchorRxDis = rxDisMsg.dis_value;
        }
        
        if(anchorSelfDis > anchorRxDis)
        {
            anchorFinalDis = anchorRxDis;                   /* 最近的标签位于对侧基站*/
            HAL_GPIO_WritePin(Onside_LED_GPIO_Port, Onside_LED_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Across_LED_GPIO_Port, Across_LED_Pin, GPIO_PIN_SET);
        }
        else
        {
            anchorFinalDis = anchorSelfDis;                 /* 最近的标签位于本侧基站*/
            HAL_GPIO_WritePin(Onside_LED_GPIO_Port, Onside_LED_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Across_LED_GPIO_Port, Across_LED_Pin, GPIO_PIN_RESET);
        }
        
        voiceOutputDis = anchorFinalDis;

        printf_use_dma("SelfDis %.2f, RxDis %.2f, FinalDis %.2f \n", (float)(anchorSelfDis) / 1000, (float)(anchorRxDis) / 1000, (float)(anchorFinalDis) / 1000);
        osDelay(300);       /* 定时处理距离数据 */
    }
}

/**
  * @brief  
  * @param  none 
  * @retval none
  */
void task2_canRx(void *argument)
{
    uint32_t tick;
    distanceData_t disMsg;
    osStatus status;
    
    tick = osKernelGetTickCount();
    for(;;)
    {
        if(voiceOutputDis < 100000)
        {

            HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);    // 打开蜂鸣器
        }
        else 
        {
            HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);  // 关闭蜂鸣器 
        }
        
        Report_Dis((float)voiceOutputDis / 1000.0);
        tick += 1100;
        osDelayUntil(tick);
    }
}

void task3_canSend(void *argument)
{   
    distanceData_t sendDis;
    osStatus_t status;
    for(;;)
    {
        status = osMessageQueueGet(minDisQueue, &sendDis, 0, portMAX_DELAY);
        if(status  == osOK)
        {
            split32to8(sendDis.dis_value, canSendBuf);
            canSendMsg(anchorCanExtId, canSendBuf, 8);                                 // CAN 发送基站本测最小距离值
        }
    }
}

void task4_timerYield(void *argument)
{
    for(;;)
    {
        multiTimerYield();
    }
}

void task5_voiceManage(void *argument)
{
    for(;;)
    {
        
    }
}

/*************************************************Key function*************************************************/
void pause_key_handler1(void * buttonPause) 
{
    // 按键短按，关闭蜂鸣器，关闭扬声器
    HAL_GPIO_DeInit(BUZZER_GPIO_Port, BUZZER_Pin);
    HAL_UART_DeInit(&huart4);
}

void pause_key_handler2(void * buttonPause)
{
    // 按键长按，恢复蜂鸣器以及串口功能
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    GPIO_InitStruct.Pin = BUZZER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);  
    
    MX_UART4_Init();
}


void switch_key_left_handler(void * buttonPause) 
{
    // 旋钮切换，发送任务通知
    //    osThreadFlagsSet(defaultTaskHandle, 0x01U);         // 标志位第0位置1，使用该方法同步任务，会出现任务初始能够触发，运行一段时间后无法触发的现象，后续排查
//    printf_use_dma("switch_key_left_handler\r\n");
    canSendMsg(anchorCanExtId1, voice_clear_left, ARRAY_LENGTH(voice_clear_left));
    printf_use_dma("send sound clear message.\r\n");
}

void switch_key_right_handler(void * buttonPause)
{
    osThreadFlagsSet(defaultTaskHandle, 0x01U);         // 标志位第0位置1
    printf_use_dma("switch_key_right_handler\r\n");
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
    distanceData_t rxMsg;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)      /* Get RX message */
    {
        
        if ((RxHeader.ExtId == 0xAAA0) && (RxHeader.IDE == CAN_ID_EXT) && (RxHeader.DLC == 8))
        {
            anchorReceiveDis = combine8to32(RxData);             // CAN正确接收，填充距离
            rxMsg.dis_class = OTHER_ANCHOR_DIS;
            rxMsg.dis_value = anchorReceiveDis;
            osMessageQueuePut(rxDisQueue, &rxMsg, 0, 0);
            led_toggle(can_rx_led);
        } 
        else if ((RxHeader.ExtId == 0xAAA1) && (RxHeader.IDE == CAN_ID_EXT))
        {
//            osThreadFlagsSet(defaultTaskHandle, 0x02U);         // CAN接收到消音消息，标志位第1位置1
            printf_use_dma("receive sound clear message.\n");
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
    if (htim->Instance == TIM2) 
    {
        // clear_sortDistance();
        // clearReceiveDis();
    }
    else if (htim->Instance == TIM3)
    {
        button_ticks();
    }
    else if (htim->Instance == TIM6)
    {
        HAL_IncTick();  // HAL延时所使用的时基
    }
}

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

void clearReceiveDis(void)
{
    anchorSelfDis  = 2000000;
    anchorRxDis = 2000000;
    anchorFinalDis = 2000000;
}
/* USER CODE END Application */
