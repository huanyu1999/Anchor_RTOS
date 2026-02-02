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
#include "dw_sort.h"
#include "dw_instance.h"
#include "app_gnss.h"
#include "app_network.h"
#include "app_emmc.h"
#include "app_usb_msc.h"
#include "board_dw1000.h"
#include "board_w5500.h"
#include "dev_button.h"
#include "dev_can.h"
#include "dev_dw1000.h"
#include "dev_emmc.h"
#include "dev_gnss.h"
#include "dev_led_buzzer_dip.h"
#include "dev_linkFatFs.h"
#include "dev_rx8130ce.h"
#include "dev_uart.h"
#include "dev_w5500.h"
#include "dev_jq8400.h"
#include "drv_sdio.h"
#include "drv_timer.h"
#include "usart.h"

#include "com_multiButton.h"
#include "elog.h"

#include "dhcp.h"
#include "socket.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 由于 CMSIS_OS2 没有封装静态创建task 或者queue需要用到的类型，自己定义，方便代码命名风格统一
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
#if TASK_DEBUG_INFO
TIM_HandleTypeDef timer50usHandle;
volatile uint32_t ulHighFrequencyTimerTicks = 0UL;
#endif

/*********************************************************Tag Distance Management**********************************************************/
#define TAG_DIS_CHANGE_THRESHOLD    6000        // 标签距离减小阈值，单位mm，一旦越过该值，就恢复声音警报
#define TAG_DIS_ALARM_THRESHOLD     100000      // 标签距离最小有效值，单位mm，小于该值的距离不予考虑

/**************************************************************CAN Communication**********************************************************/
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[5];

/**************************************************************Voice Output**************************************************************/
static int32_t voiceOutputDis = -1;
static uint8_t voiceOutputIndex = 0xFF;
typedef enum {
    VOICE_OFF = 0, VOICE_PLAYING, VOICE_MUTED
} voiceState_t;
static voiceState_t voice_state = VOICE_OFF;

/**************************************************************Semaphore****************************************************************/
osSemaphoreId_t w5500IntSem;
StaticSemaphore_t w5500IntSemCB;
const osSemaphoreAttr_t w5500IntSem_attr = {
    .name = "w5500IntSem", .cb_mem = &w5500IntSemCB, .cb_size = sizeof(StaticSemaphore_t)
};

osSemaphoreId_t elog_lockSem;                                // 用于elog_lock
StaticSemaphore_t elog_lockSemCB;
const osSemaphoreAttr_t elog_lockSem_attr = {
    .name = "elog_lock", .cb_mem = &elog_lockSemCB, .cb_size = sizeof(StaticSemaphore_t)
};

osSemaphoreId_t elog_asyncSem;                               // 用于elog_async
StaticSemaphore_t elog_asyncSemCB;
const osSemaphoreAttr_t elog_asyncSem_attr = {
    .name = "elog_async", .cb_mem = &elog_asyncSemCB, .cb_size = sizeof(StaticSemaphore_t)
};

osSemaphoreId_t gnssReceiveSem;
StaticSemaphore_t gnssReceiveSemCB;
const osSemaphoreAttr_t gnssReceiveSem_attr = {
    .name = "gnssReceive", .cb_mem = &gnssReceiveSemCB, .cb_size = sizeof(StaticSemaphore_t)
};

osSemaphoreId_t sema_canReceive;
StaticSemaphore_t sema_canReceive_cb;
const osSemaphoreAttr_t sema_caReceive_attr = {
    .name = "sema_canReceive", .cb_mem = &sema_canReceive_cb, .cb_size = sizeof(sema_canReceive_cb)
};

// osSemaphoreId_t uart_dmaLockSem;                             // 用于elog串口dma发送同步，串口发送完成中断中释放，串口DMA发送前获取
// StaticSemaphore_t uart_dmaLockSemCB;
// const osSemaphoreAttr_t uart_dmaLockSem_attr = { .name = "uart_dmaLock", .cb_mem = &uart_dmaLockSemCB, .cb_size = sizeof(StaticSemaphore_t) };

/**************************************************************QueueMsg**************************************************************/
osMessageQueueId_t    queue_minimalDis;                                 /* Definitions for minDisQueue */
outDistance_t         queue_minimalDis_buf[16];
StaticQueue_t queue_minimalDis_cb;
const osMessageQueueAttr_t queue_minimalDis_attr = {
    .name    = "queue_minimalDis",
    .cb_mem  = &queue_minimalDis_cb, .cb_size = sizeof(queue_minimalDis_cb),
    .mq_mem  = &queue_minimalDis_buf, .mq_size = sizeof(queue_minimalDis_buf)
};

osMessageQueueId_t    canRxDisQueue;                                 /* Definitions for rxDisQueue */
outDistance_t         canRxDisQueueBuffer[8];
StaticQueue_t canRxDisQueueCB;
const osMessageQueueAttr_t canRxDisQueue_attr = {
    .name    = "canRxDisQueue",
    .cb_mem  = &canRxDisQueueCB, .cb_size = sizeof(canRxDisQueueCB),
    .mq_mem  = &canRxDisQueueBuffer, .mq_size = sizeof(canRxDisQueueBuffer)
};

/**************************************************************Thread**************************************************************/
osThreadId_t task0_uwb_handle;                           
const osThreadAttr_t task0_uwb_attr = {
    .name = "task_uwb", 
    .stack_size = 1024, 
    .priority = (osPriority_t) osPriorityRealtime7,
};

osThreadId_t task_twrRun_handle;
static uint8_t task_twrRun_buf[1024];
StaticTask_t task_twrRun_cb;
const osThreadAttr_t task_twrRun_attr = {
    .name = "task_twrRun", 
    .stack_mem = task_twrRun_buf, .stack_size = sizeof(task_twrRun_buf), 
    .cb_mem = &task_twrRun_cb, .cb_size = sizeof(task_twrRun_cb),
    .priority = (osPriority_t) osPriorityISR
};

osThreadId_t task1_anchorDisHandling_handle;                
const osThreadAttr_t task1_anchorDisHandling_attr = {
    .name = "task_anchorDisHandling", 
    .stack_size = 1024, 
    .priority = (osPriority_t) osPriorityRealtime6
};

osThreadId_t task2_voiceOutControl_handle;                  
const osThreadAttr_t task2_voiceOut_attr = {
    .name = "task_voiceOutControl", 
    .stack_size = 512, 
    .priority = (osPriority_t) osPriorityRealtime6
};

osThreadId_t task_canSend_handle;
static uint8_t  task_canSend_buf[512];
StaticTask_t task_canSend_cb;
const osThreadAttr_t task_canSend_attr = {
    .name = "task_canSend",
    .stack_mem = &task_canSend_buf[0], .stack_size = sizeof(task_canSend_buf),
    .cb_mem = &task_canSend_cb, .cb_size = sizeof(task_canSend_cb),
    .priority = (osPriority_t) osPriorityRealtime6
};

osThreadId_t task4_minHeapManage_handle;
const osThreadAttr_t task4_minHeapManage_attr = {
    .name = "task_minHeapManage",
    .stack_size = 1024,
    .priority = (osPriority_t) osPriorityRealtime7
};

osThreadId_t task5_tagDisMonitor_handle;
static uint8_t task_tagDisMonitor_buffer[1024];
StaticTask_t task_tagDistanceMonitorCB;
const osThreadAttr_t task_tagDisMonitor_attr = {
    .name       = "task_tagDisMonitor", 
    .stack_mem  = &task_tagDisMonitor_buffer[0], .stack_size = sizeof(task_tagDisMonitor_buffer),
    .cb_mem     = &task_tagDistanceMonitorCB,    .cb_size    = sizeof(task_tagDistanceMonitorCB),
    .priority   = (osPriority_t) osPriorityRealtime6
};

osThreadId_t task6_logManage_Handle;
static uint8_t logManageTask_buffer[1280];
StaticTask_t logManageTaskCB;
const osThreadAttr_t task6_logManage_attr = {
    .name       = "elog_entry",
    .stack_mem  = &logManageTask_buffer[0], .stack_size = sizeof(logManageTask_buffer),
    .cb_mem     = &logManageTaskCB, .cb_size = sizeof(logManageTaskCB),
    .priority   = (osPriority_t) osPriorityRealtime5
};

osThreadId_t task7_Handle;
static uint8_t task7_buffer[256];
StaticTask_t task7CB;
const osThreadAttr_t task7_attr = {
    .name       = "task_gnssSyncTime",
    .stack_mem  = &task7_buffer[0], .stack_size = sizeof(task7_buffer),
    .cb_mem     = &task7CB, .cb_size = sizeof(task7CB),
    .priority   = (osPriority_t) osPriorityRealtime5
};

osThreadId_t task_canReceive_handle;
static uint8_t task_canReceive_buf[512];
StaticTask_t task_canReceive_cb;
const osThreadAttr_t task_canReceive_attr = {
    .name = "task_canReceive",
    .stack_mem = &task_canReceive_buf[0], .stack_size = sizeof(task_canReceive_buf),
    .cb_mem = &task_canReceive_cb, .cb_size = sizeof(task_canReceive_cb),
    .priority = (osPriority_t) osPriorityRealtime6
};

osThreadId_t task9_Handle;
static uint8_t task9_buffer[512];
StaticTask_t task9CB;
const osThreadAttr_t task9_attr = {
    .name = "task_eth",
    .stack_mem = &task9_buffer[0], .stack_size = sizeof(task9_buffer),
    .cb_mem = &task9CB, .cb_size = sizeof(task9CB),
    .priority = (osPriority_t) osPriorityRealtime5
};

osThreadId_t task10_emmcHandle;
static uint8_t task10_buffer[64];
StaticTask_t task10CB;
const osThreadAttr_t task10_attr = {
    .name = "task10",
    .stack_mem = &task10_buffer[0], .stack_size = sizeof(task10_buffer),
    .cb_mem = &task10CB, .cb_size = sizeof(task10CB),
    .priority = (osPriority_t) osPriorityRealtime4,
};

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void task_anchorDisHandling(void *arg);
void task_voiceOutContorl(void *arg);
void task_canTransmit(void *arg);
void task_canReceive(void *arg);
void task_tagDisMonitor(void* arg);
void task_emmcTest(void *arg);
void task10_Test(void *arg); 

static void outputVoiceBuildAndPlay(uint8_t closetTagId, int32_t closetTagDis, char buffer[10]);

/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); 

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    queue_minimalDis  = osMessageQueueNew(16, sizeof(outDistance_t), &queue_minimalDis_attr);
    canRxDisQueue   = osMessageQueueNew(3, sizeof(outDistance_t), &canRxDisQueue_attr);
    w5500IntSem     = osSemaphoreNew(1, 0, &w5500IntSem_attr);
    elog_lockSem    = osSemaphoreNew(1, 1, &elog_lockSem_attr);                 // 该二值信号量初始值必须设置为1，用于log写入时上锁
    elog_asyncSem   = osSemaphoreNew(1, 1, &elog_asyncSem_attr); 
    gnssReceiveSem  = osSemaphoreNew(1, 0, &gnssReceiveSem_attr);
    sema_canReceive = osSemaphoreNew(1, 0, &sema_caReceive_attr);
    // uart_dmaLockSem = osSemaphoreNew(1, 0, &uart_dmaLockSem_attr);   
    dev_emmcSemaInit();
    /* USER CODE END RTOS_QUEUES */

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    task0_uwb_handle               = osThreadNew(task_uwb, NULL, &task0_uwb_attr);
    // task_twrRun_handle             = osThreadNew(task_twrRun, NULL, &task_twrRun_attr);
    // task1_anchorDisHandling_handle = osThreadNew(task_anchorDisHandling, NULL, &task1_anchorDisHandling_attr);
    // task2_voiceOutControl_handle   = osThreadNew(task_voiceOutContorl, NULL, &task2_voiceOut_attr);
    // task_canSend_handle            = osThreadNew(task_canTransmit, NULL, &task_canSend_attr);
    // task_canReceive_handle         = osThreadNew(task_canReceive, NULL, &task_canReceive_attr);
    task4_minHeapManage_handle     = osThreadNew(task_minHeapManage, NULL, &task4_minHeapManage_attr);
    // task5_tagDisMonitor_handle     = osThreadNew(task_tagDisMonitor, NULL, &task_tagDisMonitor_attr);
    // 上电不能自动运行，跟这个elog 任务貌似有关
    // task6_logManage_Handle         = osThreadNew(elog_entry, NULL, &task6_logManage_attr);
    // task7_Handle                   = osThreadNew(task_gnssSyncTime, NULL, &task7_attr);
    // task9_Handle                   = osThreadNew(task_eth, NULL, &task9_attr);
    task10_emmcHandle              = osThreadNew(task10_Test, NULL, &task10_attr);
    
#if TASK_DEBUG_INFO
    drv_setTimerForInt(&timer50usHandle, TIM4, 20000, 6);
#endif
    
    /* USER CODE END RTOS_THREADS */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
  * @brief  最小距离获取，收发处理，此任务需要跟堆清除无效距离处理任务配合同步，按照一个TDMA的周期算
  * @param  void *arg
  * @retval none
  */
void task_anchorDisHandling(void *arg)
{
    UNUSED(arg);
    int32_t anchorSelfDis  = 2000000, anchorRxDis = 2000000, anchorFinalDis = 2000000;
    uint8_t anchorSelfIdx  = 0xFF,    anchorRxIdx = 0xFF,    anchorFinalIdx = 0xFF;
    static outDistance_t tmp_outDis = {0};

    for (;;)
    {
        dwDistance_t *distance = get_the_local_structure_of_dis();

        // 从堆距离计算任务中获取本侧基站最小距离
        osStatus status = osMessageQueueGet(queue_minimalDis, &distance->disMsg[0], 0, 500);      
        if ((status == osOK) && (distance->disMsg[0].dis_class == ANCHOR_SELF_DIS))
        {   // 存储最小距离，通过CAN发送出去
            anchorSelfDis = distance->disMsg[0].dis_value;
            anchorSelfIdx = distance->disMsg[0].dis_index;
        }
        else
        {
            anchorSelfDis = 2000000;
            anchorSelfIdx = 0xFF;
        }

        status = osMessageQueueGet(canRxDisQueue, &distance->disMsg[1], 0, 500);
        if ((status == osOK) && (distance->disMsg[1].dis_class == ANCHOR_OTHER_DIS))
        {   // 从CAN接收中断中获取对侧基站最小距离
            anchorRxDis = distance->disMsg[1].dis_value;        // 获取接收的距离
            anchorRxIdx = distance->disMsg[1].dis_index;
        }
        else
        {
            anchorRxDis = 2000000;
            anchorRxIdx = 0xFF;
        }

        if ((anchorSelfDis > 0 && anchorSelfDis < 200000) || (anchorRxDis > 0 && anchorRxDis < 200000))
        {   // 获取到有效的标签距离，只要有一侧有效就视为有效
            if (anchorSelfDis > anchorRxDis)
            {
                dev_ledOff(ACROSS_LED);
                dev_ledOn(ONSIDE_LED);
                anchorFinalDis = anchorRxDis;                   /* 最近的标签位于对侧基站*/
                anchorFinalIdx = anchorRxIdx; 
            }
            else if (anchorSelfDis < anchorRxDis)
            {
                dev_ledOn(ACROSS_LED);
                dev_ledOff(ONSIDE_LED);
                anchorFinalDis = anchorSelfDis;                 /* 最近的标签位于本侧基站*/
                anchorFinalIdx = anchorSelfIdx;
            }

            voiceOutputDis = anchorFinalDis;
            voiceOutputIndex = anchorFinalIdx;
            osThreadFlagsSet(task2_voiceOutControl_handle, FLAG_DIS_ALARM);
        }
        else
        {   // 没有有效标签距离传入，关闭所有报警指示灯，最终距离为自身距离
            anchorFinalDis = anchorSelfDis;
            anchorFinalIdx = anchorSelfIdx;
            dev_ledOff(ACROSS_LED);
            dev_ledOff(ONSIDE_LED);
            osThreadFlagsSet(task2_voiceOutControl_handle, FLAG_DIS_INVAILD_ALARM);
        }

        log_d("SelfDis ID %d %.2f, RxDis ID %d %.2f, FinalDis ID %d %.2f \n",  anchorSelfIdx,  (float)(anchorSelfDis) / 1000.0f, anchorRxIdx, (float)(anchorRxDis) / 1000.0f, anchorFinalIdx, (float)(anchorFinalDis) / 1000.0f);
    }
}

// uint8_t voice_buf_kj[18]={ 0xAA, 0x08, 0x0E, 0x02, 0x2F, 0xBB, 0xF9, 0xD5, 0xBE, 0xBF, 0xAA, 0xBB, 0xFA, 0x2A, 0x3F, 0x3F, 0x3F, 0x3D };
/**
  * @brief  此任务用于基站语音输出当前实时最小距离
  * @param  void *arg
  * @retval none
  */
void task_voiceOutContorl(void *arg)
{
    UNUSED(arg);
    static uint32_t voicePlayStartTick = 0;
    char voiceBuffer[10];    // 用于存储语音播放数据
    // char buf1[] = "/jzkj";
    // dev_jq8400CommandData(SetVolume, 23);       // 音量控制也可以通过上位机来确定
    // dev_jq8400RandomPathPlay(JQ8X00_FLASH, buf1);
    
    for (;;)
    {
        uint32_t flags = osThreadFlagsWait( FLAG_DIS_ALARM | 
                                            FLAG_P_BUTTON_ALARM | 
                                            FLAG_S_BUTTON_ALARM | 
                                            FLAG_DIS_THRELOD_ALARM | 
                                            FLAG_DIS_INVAILD_ALARM |    
                                            FLAG_P_BUTTON_MUTE | 
                                            FLAG_S_BUTTON_MUTE, 
                                            osFlagsWaitAny, osWaitForever );

        switch (voice_state) 
        {
        case VOICE_OFF: // 空闲状态，收到有效距离后进入播报状态，根据输出距离是否进入报警阈值选择是否打开蜂鸣器
            if (flags & FLAG_DIS_ALARM)
            {
                voice_state = VOICE_PLAYING;
                if (voiceOutputDis < TAG_DIS_ALARM_THRESHOLD)
                {
                    dev_buzzerOpen(BUZZER);
                }
                dev_jq8400Init();           // Init 声音 UART
                voicePlayStartTick = osKernelGetTickCount();
                outputVoiceBuildAndPlay(voiceOutputIndex, voiceOutputDis, voiceBuffer);
            }
            break;

        case VOICE_PLAYING:  // 播报状态下，按键及旋钮静音才会进入静音状态
            if (flags & FLAG_P_BUTTON_MUTE || flags & FLAG_S_BUTTON_MUTE)
            {
                voice_state = VOICE_MUTED;
                dev_buzzerClose(BUZZER);
                dev_jq8400DeInit();                 // DeInit 声音 UART
                log_d("mute voice.");
            }
            else if (flags & FLAG_DIS_INVAILD_ALARM)   // 没有收到有效的距离，报警关闭
            {
                voice_state = VOICE_OFF;
                dev_buzzerClose(BUZZER);
                dev_jq8400DeInit();                 // DeInit 声音 UART
                // log_d("shutdown voice.");
            }
            else
            {   
                uint32_t voicePlayingTime = 0U;     //  针对三位数距离，播放时间 1790U ，针对三位数以下距离，播放时间
                if (voiceOutputDis <= 20000)        // 播报距离小于10m，1s播报完成
                {
                    voicePlayingTime = 1600U;
                }
                else if (voiceOutputDis < 100000 && voiceOutputDis > 20000)         // 播报距离小于10m，1s播报完成
                {
                    voicePlayingTime = 1600U;
                }
                else if (voiceOutputDis >= 100000)   // 播报距离大于10m小于100m，1.3s播报完成
                {
                    voicePlayingTime = 1900U;
                }

                // 播报状态下，延迟个1s的时间确保扬声器能够完整地报出距离
                if (osKernelGetTickCount() - voicePlayStartTick >= voicePlayingTime)
                {
                    dev_buzzerClose(BUZZER);
                    voice_state = VOICE_OFF;    
                }
            }
            break;

        case VOICE_MUTED:  // 被静音状态下，只有按键，旋钮，距离减小超过阈值才会重新开启声音
            if (flags & FLAG_P_BUTTON_ALARM || flags & FLAG_S_BUTTON_ALARM || flags & FLAG_DIS_THRELOD_ALARM)
            {
                log_d("re-open voice alarm.");
                voice_state = VOICE_PLAYING;
                if (voiceOutputDis < TAG_DIS_ALARM_THRESHOLD)
                {
                    dev_buzzerOpen(BUZZER);
                }
                dev_jq8400Init();           // Init 声音 UART
                voicePlayStartTick = osKernelGetTickCount();
                outputVoiceBuildAndPlay(voiceOutputIndex, voiceOutputDis, voiceBuffer);
            }
            break;
        }
    }
}

/**
  * @brief   
  * @param  void *arg
  * @retval none
  */
void task_tagDisMonitor(void* arg)
{
    UNUSED(arg);
    uint32_t tick = osKernelGetTickCount();
    int32_t currentVoiceOutputDis = 2000000;
    int32_t prevVoiceOutputDis = 0;
    for (;;)
    {
        if (voiceOutputDis > 0 && voiceOutputDis < 200000)
        {
            // 有效距离才进行监测
            currentVoiceOutputDis = voiceOutputDis;
            if (prevVoiceOutputDis - currentVoiceOutputDis > TAG_DIS_CHANGE_THRESHOLD)
            {
                osThreadFlagsSet(task2_voiceOutControl_handle, FLAG_DIS_THRELOD_ALARM);
                log_d("tag distance change up to threshold.");
            }
            prevVoiceOutputDis = currentVoiceOutputDis;
        }
        // 设置监控周期
        tick += 1500U;
        osDelayUntil(tick);
    }
}

/**
  * @brief  此任务用于本侧基站发送旋钮编码至对侧基站 
  * @param  void *arg
  * @retval none
  */
void task_canTransmit(void *arg)
{
    UNUSED(arg);
    static outDistance_t tmp_dis = {0};
    static uint8_t can_buf[5] = {0};
    for (;;)
    {
        // osStatus status = osMessageQueueGet(minDisQueue, &tmp_dis, 0, osWaitForever);
        // if ((status == osOK) &&(tmp_dis.dis_class == ANCHOR_SELF_DIS))
        // {   
        //     split32to8(tmp_dis.dis_value, can_buf);
        //     can_buf[4] = tmp_dis.dis_index;
        //     dev_canSendMsg(CAN_EXT_ID_DIS, &can_buf[0], ARRAY_LENGTH(can_buf));
        // }
        osThreadFlagsWait(FLAG_S_BUTTON_TRIGGER, osFlagsWaitAll, osWaitForever);
        uint8_t buttonVal = dev_buttonRead(BUTTON_ID_SWITCH);       // 读取键值，供can发送
        dev_canSendMsg(CAN_EXT_ID_BUTTON, &buttonVal, 1);
        log_d("anchor send switch key value : %d.", buttonVal);
    }
}

/**
  * @brief  此任务用于本侧基站接收对侧基站的旋钮编码，并做出对应的处理 
  * @param  void *arg
  * @retval none
  */
void task_canReceive(void *arg)                 // 尝试将can收发整合为一个任务
{
    UNUSED(arg);
    static outDistance_t rxMsg = {0};
    static int32_t anchorReceiveDis = 2000000;  // 用于储存基站接收到的距离
    for (;;)
    {
        // osThreadFlagsWait(0x01, osFlagsWaitAll, osWaitForever);
        osSemaphoreAcquire(sema_canReceive, osWaitForever);
        dev_ledBlink(CAN_RX_LED);
        dev_ledBlink(ACROSS_LED);
        log_d("can receive task.");
        if ((RxHeader.ExtId == CAN_EXT_ID_DIS) && (RxHeader.IDE == CAN_ID_EXT))
        {
            anchorReceiveDis = combine8to32(RxData);                // CAN正确接收，填充距离
            rxMsg.dis_class = ANCHOR_OTHER_DIS;
            rxMsg.dis_value = anchorReceiveDis;
            rxMsg.dis_index = RxData[4];                            // 获取存储的标签ID
            // osMessageQueuePut(canRxDisQueue, &rxMsg, 0, 0);
            // osMessageQueuePut(minDisQueue, &rxMsg, 0, 0);
            log_d("can receive dis, put in msg.");
            dev_ledBlink(CAN_RX_LED);
        }
        else if ((RxHeader.ExtId == CAN_EXT_ID_BUTTON) && (RxHeader.IDE == CAN_ID_EXT))
        {   // 接收键值存在问题
            // 接收到的为旋钮控制信息，校验两侧旋钮键值，播报还是静音
            uint8_t rxbutton_val = RxData[0];
            uint8_t selfButton_val =  dev_buttonRead(BUTTON_ID_SWITCH);
            // log_d("receive button_val : %d", rxbutton_val);

            if (rxbutton_val == selfButton_val)
            {   // 两侧键值相同，打开播报
                log_d("receive button_val same, open voice");
                osThreadFlagsSet(task2_voiceOutControl_handle, FLAG_S_BUTTON_ALARM);
            }
            else
            {   // 两侧键值不相同，关闭播报
                log_d("receive button_val not same, close voice");
                osThreadFlagsSet(task2_voiceOutControl_handle, FLAG_S_BUTTON_MUTE);
            }
        }
    }
}

void task_emmcTest(void *arg)
{
    UNUSED(arg);
    app_emmcReadWriteDemo();
    // app_usbMscInit();
    for (;;)
    {
        osDelay(2000);
    }
}

/**
  * @brief  此任务用于RTOS的系统运行信息显示
  * @param  void *arg
  * @retval none
  */
void task10_Test(void *arg)
{
    UNUSED(arg);
    // char taskListBuffer[512];
    for (;;)
    {
        // osThreadFlagsWait(0x01, osFlagsWaitAll, osWaitForever);
        // log_i("==========================================");
        // log_i("task_name    state priority stack_size num_of_task");
        // vTaskList(taskListBuffer);
        // log_i("%s", taskListBuffer);

        // log_i("task_name    run_time_counter   use_percentage");
        // vTaskGetRunTimeStats(taskListBuffer);
        // log_i("%s", taskListBuffer);
        // log_i("==========================================");     
        osDelay(500);
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
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        // osThreadFlagsSet(task_canReceive_handle, 0x01);
        osSemaphoreRelease(sema_canReceive);
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
        extern osSemaphoreId_t tagDistClear_sem;
        if (osSemaphoreRelease(tagDistClear_sem) == osOK)
        {
            // log_d("tdmaCycleTimer_CallBack");
        }
    }
    else if (htim->Instance == TIM4)
    {
#if TASK_INFO
        ulHighFrequencyTimerTicks++;
#endif
        DHCP_time_handler();
    }
}

extern osSemaphoreId_t uwbIntSem;
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
        osSemaphoreRelease(uwbIntSem);
        // osThreadFlagsSet(task0_uwb_handle, 0x00000001);
    }
    else if (GPIO_Pin == W5500_INT_PIN)
    {
        osSemaphoreRelease(w5500IntSem);
    }
}
/*************************************************static function*************************************************/
static void outputVoiceBuildAndPlay(uint8_t cloestTagId, int32_t cloestTagDis, char buffer[10])
{
    int cnt = 0;                                    // 记录buffer中语音片段数量
    int tens_digit = 0;
    int ones_digit = 0;
    char head_letter = '\0';

    int32_t cloestTagDis_meter = cloestTagDis / 1000;

    if (cloestTagDis_meter <= 100)            // 距离小于100m，按米播报 
    {
        tens_digit = cloestTagDis_meter / 10; // 获取10位数字
        ones_digit = cloestTagDis_meter % 10; // 获取个位数字
        head_letter = tens_digit + 'A';
        if (cloestTagId < 10)
        {
            cnt = sprintf(buffer, "0%d%c%d", cloestTagId, head_letter, ones_digit);
        }
        else
        {
            cnt = sprintf(buffer, "%d%c%d", cloestTagId, head_letter, ones_digit);
        }
    }
    else if (cloestTagDis_meter > 100)          // 距离大于100m，按5米播报
    {
        tens_digit = cloestTagDis_meter / 10;   // 获取10位数字
        ones_digit = cloestTagDis_meter % 10;   // 获取个位数字
        head_letter = tens_digit + 'A';
        if (ones_digit < 5)
        {
            cnt = sprintf(buffer, "%d%c0", cloestTagId, head_letter);
        }
        else if (ones_digit >=5)
        {
            cnt = sprintf(buffer, "%d%c5", cloestTagId, head_letter);
        }
    }
    dev_jq8400CombinePlay((char*)buffer, cnt);
}
/* USER CODE END Application */
