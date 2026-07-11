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
#include "app_config.h"
#include "dw_sort.h"
#include "dw_instance.h"
#include "os_event.h"
#include "app_gnss.h"
#include "app_network.h"
// #include "app_usb_msc.h"
#include "board_dw1000.h"
#include "board_w5500.h"
// #include "dev_button.h"
#include "dev_can.h"
#include "dev_dw1000.h"
#include "dev_emmc.h"
// #include "dev_gnss.h"
#include "dev_led_buzzer_dip.h"
// #include "dev_linkFatFs.h"
// #include "dev_rx8130ce.h"
// #include "dev_uart.h"
// #include "dev_w5500.h"
#include "dev_jq8400.h"
// #include "drv_sdio.h"
#include "drv_timer.h"
// #include "usart.h"

#include "com_multiButton.h"
#include "elog.h"
#include "SEGGER_RTT.h"

#include "dhcp.h"
// #include "socket.h"

#ifdef TASK_DEBUG_INFO
TIM_HandleTypeDef timer50usHandle;
volatile uint32_t ulHighFrequencyTimerTicks = 0UL;
#endif

/*********************************************************Tag Distance Management**********************************************************/
#define TAG_DIS_CHANGE_THRESHOLD    3000        // 标签距离减小阈值，单位mm，静音后距离减小超过该值恢复报警

/**************************************************************CAN Communication**********************************************************/
static CAN_RxHeaderTypeDef RxHeader;
static uint8_t RxData[5] = {0};
static const uint8_t can_mute[5]  = {'C', 'A', 'N', 'M', '0'};  // MUTE Cmd, transmitted by CAN when remote alarm cleared, used to mute remote side alarm
static const uint8_t can_alarm[5] = {'C', 'A', 'N', 'A', '0'};  // ALRM Cmd, transmitted by CAN when remote alarm triggered, used to trigger remote side alarm

/**************************************************************alarm Output**************************************************************/
static int32_t voiceOutputDis = -1;
static uint8_t voiceOutputIndex = 0xFF;
#if MODULE_ALARM_ENABLE
static alarm_state_t cur_alarmState = ALARM_LEVEL_0;
static bool voicePlaying = false;
static uint32_t voiceStartTick = 0;
static uint32_t voiceIntervalMs = VOICE_INTERVAL_L1;  // 当前等级对应的语音播报间隔
#endif

/**************************************************************Semaphore****************************************************************/
osSemaphoreId_t sema_w5500Int;
StaticSemaphore_t sema_w5500Int_cb;
const osSemaphoreAttr_t sema_w5500Int_attr = {
    .name = "sema_w5500Int", 
    .cb_mem = &sema_w5500Int_cb, .cb_size = sizeof(sema_w5500Int_cb)
};

osSemaphoreId_t sema_elogLock;                                // elog_lock output Semaphore
StaticSemaphore_t sema_elogLock_cb;
const osSemaphoreAttr_t sema_elogLock_attr = {
    .name = "sema_elogLock", 
    .cb_mem = &sema_elogLock_cb, .cb_size = sizeof(sema_elogLock_cb)
};

osSemaphoreId_t sema_gnssReceive;                             // 用于GNSS接收完成同步
StaticSemaphore_t sema_gnssReceive_cb;
const osSemaphoreAttr_t sema_gnssReceive_attr = {
    .name = "sema_gnssReceive", 
    .cb_mem = &sema_gnssReceive_cb, .cb_size = sizeof(sema_gnssReceive_cb)
};

// osSemaphoreId_t elog_asyncSem;                              // 用于elog_async
// StaticSemaphore_t elog_asyncSemCB;
// const osSemaphoreAttr_t elog_asyncSem_attr = {
//     .name = "elog_async", .cb_mem = &elog_asyncSemCB, .cb_size = sizeof(StaticSemaphore_t)
// };

// osSemaphoreId_t sema_canReceive;
// StaticSemaphore_t sema_canReceive_cb;
// const osSemaphoreAttr_t sema_caReceive_attr = {
//     .name = "sema_canReceive", .cb_mem = &sema_canReceive_cb, .cb_size = sizeof(sema_canReceive_cb)
// };

// osSemaphoreId_t uart_dmaLockSem;                             // 用于elog串口dma发送同步，串口发送完成中断中释放，串口DMA发送前获取
// StaticSemaphore_t uart_dmaLockSemCB;
// const osSemaphoreAttr_t uart_dmaLockSem_attr = { .name = "uart_dmaLock", .cb_mem = &uart_dmaLockSemCB, .cb_size = sizeof(StaticSemaphore_t) };

/**************************************************************QueueMsg**************************************************************/
osMessageQueueId_t    queue_minimalDis;                                 /* Definitions for minDisQueue */
outDistance_t         queue_minimalDis_buf[16];
StaticQueue_t queue_minimalDis_cb;
const osMessageQueueAttr_t queue_minimalDis_attr = {
    .name    = "queue_minimalDis",
    .mq_mem  = &queue_minimalDis_buf, .mq_size = sizeof(queue_minimalDis_buf), 
    .cb_mem  = &queue_minimalDis_cb, .cb_size = sizeof(queue_minimalDis_cb),
};

osMessageQueueId_t    queue_canRxDis;                                 /* Definitions for rxDisQueue */
outDistance_t         queue_canRxDis_buf[16];
StaticQueue_t queue_canRxDis_cb;
const osMessageQueueAttr_t canRxDisQueue_attr = {                    // 之前在这里貌似写错了，将改队列的分配内存赋值给了其他队列
    .name    = "queue_canRxDis",
    .mq_mem  = &queue_canRxDis_buf, .mq_size = sizeof(queue_canRxDis_buf), 
    .cb_mem  = &queue_canRxDis_cb, .cb_size = sizeof(queue_canRxDis_cb),
};

osMessageQueueId_t queue_alarm;
alarm_event_t      queue_alarm_buf[18];
StaticQueue_t      queue_alarm_cb;
const osMessageQueueAttr_t queue_alarm_attr = {
    .name    = "queue_alarm",
    .mq_mem  = &queue_alarm_buf, .mq_size = sizeof(queue_alarm_buf), .cb_mem  = &queue_alarm_cb, .cb_size = sizeof(queue_alarm_cb),
};

/**************************************************************Thread**************************************************************/
osThreadId_t task_uwb_handle;
static uint8_t task_uwb_buf[2048];              // 回调直驱 TWR 状态机（原 task_twrRun 职责并入，含 double 运算与 log），1280→2048；task_twrRun 的 2048 已删除，净 RAM 不增
StaticTask_t task_uwb_cb;
const osThreadAttr_t task_uwb_attr = {
    .name      = "task_uwb",
    .stack_mem = task_uwb_buf, .stack_size = sizeof(task_uwb_buf), .cb_mem = &task_uwb_cb, .cb_size = sizeof(task_uwb_cb),
    .priority  = (osPriority_t) osPriorityISR,   // TWR State machine core task
};

osThreadId_t task_anchorDisHandling_handle;
static uint8_t task_anchorDisHandling_buf[1280];
StaticTask_t task_anchorDisHandling_cb;               
const osThreadAttr_t task_anchorDisHandling_attr = {
    .name      = "task_anchorDisHandling",
    .stack_mem = task_anchorDisHandling_buf, .stack_size = sizeof(task_anchorDisHandling_buf), .cb_mem = &task_anchorDisHandling_cb, .cb_size = sizeof(task_anchorDisHandling_cb),
    .priority  = (osPriority_t) osPriorityRealtime4     // 距离融合 + LED + 投递 alarm 事件
};

osThreadId_t task_eventProcess_handle;
static uint8_t task_eventProcess_buf[1024];
StaticTask_t task_eventProcess_cb;
const osThreadAttr_t task_eventProcess_attr = {
    .name      = "task_eventProcess",
    .stack_mem = task_eventProcess_buf, .stack_size = sizeof(task_eventProcess_buf), .cb_mem = &task_eventProcess_cb, .cb_size = sizeof(task_eventProcess_cb),
    .priority  = (osPriority_t) osPriorityRealtime3     // 语音/蜂鸣器/CAN，人耳时间尺度，无需高优先级
};

osThreadId_t task_minHeapManage_handle;
static uint8_t task_minHeapManage_buf[1024];
StaticTask_t task_minHeapManage_cb;
const osThreadAttr_t task_minHeapManage_attr = {
    .name      = "task_minHeapManage",
    .stack_mem = task_minHeapManage_buf, .stack_size = sizeof(task_minHeapManage_buf), .cb_mem = &task_minHeapManage_cb, .cb_size = sizeof(task_minHeapManage_cb),
    .priority  = (osPriority_t) osPriorityRealtime5     // hash+heap 操作，TWR回调产生数据后异步处理
};

osThreadId_t task_getMinDis_handle;
static uint8_t task_getMinDis_buf[1024];
StaticTask_t task_getMinDis_cb;
const osThreadAttr_t task_getMinDis_attr = {
    .name      = "task_getMinDis",
    .stack_mem = task_getMinDis_buf, .stack_size = sizeof(task_getMinDis_buf), .cb_mem = &task_getMinDis_cb, .cb_size = sizeof(task_getMinDis_cb),
    .priority  = (osPriority_t) osPriorityRealtime4,    // 周期性扫描，不得与 TWR 抢占
};

osThreadId_t task_tagDisMonitor_handle;
static uint8_t task_tagDisMonitor_buf[1024];
StaticTask_t task_tagDisMonitor_cb;
const osThreadAttr_t task_tagDisMonitor_attr = {
    .name       = "task_tagDisMonitor",
    .stack_mem  = task_tagDisMonitor_buf, .stack_size = sizeof(task_tagDisMonitor_buf), .cb_mem = &task_tagDisMonitor_cb, .cb_size = sizeof(task_tagDisMonitor_cb),
    .priority   = (osPriority_t) osPriorityNormal       // 1500ms 周期，人感知时间尺度
};

osThreadId_t task_gnssSyncTime_handle;
static uint8_t task_gnssSyncTime_buf[768];
StaticTask_t task_gnssSyncTime_cb;
const osThreadAttr_t task_gnssSyncTime_attr = {
    .name       = "task_gnssSyncTime",
    .stack_mem  = task_gnssSyncTime_buf, .stack_size = sizeof(task_gnssSyncTime_buf), .cb_mem = &task_gnssSyncTime_cb, .cb_size = sizeof(task_gnssSyncTime_cb),
    .priority   = (osPriority_t) osPriorityNormal       // 秒级同步，不影响 TWR
};

osThreadId_t task_eth_handle;
static uint8_t task_eth_buf[1024];
StaticTask_t task_eth_cb;
const osThreadAttr_t task_eth_attr = {
    .name      = "task_eth",
    .stack_mem = task_eth_buf, .stack_size = sizeof(task_eth_buf), .cb_mem = &task_eth_cb, .cb_size = sizeof(task_eth_cb),
    .priority  = (osPriority_t) osPriorityNormal        // W5500 中断 sema 唤醒，网络收发不影响 TWR
};

osThreadId_t task_rtosMonitor_handle;
static uint8_t task_rtosMonitor_buf[1280];
StaticTask_t task_rtosMonitor_cb;
const osThreadAttr_t task_rtosMonitor_attr = {
    .name      = "task_rtosMonitor",
    .stack_mem = task_rtosMonitor_buf, .stack_size = sizeof(task_rtosMonitor_buf), .cb_mem = &task_rtosMonitor_cb, .cb_size = sizeof(task_rtosMonitor_cb),
    .priority  = (osPriority_t) osPriorityLow           // 10s 周期调试统计，最低优先级
};

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void task_anchorDisHandling(void *arg);
void task_tagDisMonitor(void* arg);
void task_eventHandler(void *arg);
void task_emmcTest(void *arg);
void task_Test(void *arg);
void task_rtosMonitor(void *arg);

#if MODULE_ALARM_ENABLE
static void outputVoiceBuildAndPlay(uint8_t closetTagId, int32_t closetTagDis, char buffer[10]);
static alarm_state_t alarm_levelFromDist(int32_t dist_mm);
static void alarm_applyLevel(alarm_state_t level, char *voice_buf);
#endif
/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN RTOS_QUEUES */
    queue_minimalDis = osMessageQueueNew(16, sizeof(outDistance_t), &queue_minimalDis_attr);
    queue_canRxDis   = osMessageQueueNew(16, sizeof(outDistance_t), &canRxDisQueue_attr);
    queue_alarm      = osMessageQueueNew(18, sizeof(alarm_event_t), &queue_alarm_attr);
    sema_w5500Int    = osSemaphoreNew(1, 0, &sema_w5500Int_attr);
    sema_elogLock    = osSemaphoreNew(1, 1, &sema_elogLock_attr);
    sema_gnssReceive = osSemaphoreNew(1, 0, &sema_gnssReceive_attr);
#if MODULE_EMMC_ENABLE
    dev_emmcSemaInit();
#endif
    /* USER CODE END RTOS_QUEUES */

    /* USER CODE BEGIN RTOS_THREADS */
#if MODULE_UWB_ENABLE
    task_uwb_handle               = osThreadNew(task_uwb, NULL, &task_uwb_attr);
    task_anchorDisHandling_handle = osThreadNew(task_anchorDisHandling, NULL, &task_anchorDisHandling_attr);
    task_minHeapManage_handle     = osThreadNew(task_minHeapManage, NULL, &task_minHeapManage_attr);
    task_getMinDis_handle         = osThreadNew(task_getMinDis, NULL, &task_getMinDis_attr);
#endif
#if MODULE_ALARM_ENABLE
    task_tagDisMonitor_handle     = osThreadNew(task_tagDisMonitor, NULL, &task_tagDisMonitor_attr);
#endif
#if (MODULE_ALARM_ENABLE || MODULE_CAN_ENABLE)
    task_eventProcess_handle      = osThreadNew(task_eventHandler, NULL, &task_eventProcess_attr);
#endif
#if MODULE_GNSS_ENABLE
    task_gnssSyncTime_handle      = osThreadNew(task_gnssSyncTime, NULL, &task_gnssSyncTime_attr);
#endif
#if MODULE_W5500_ENABLE
    task_eth_handle               = osThreadNew(task_eth, NULL, &task_eth_attr);
#endif
    task_rtosMonitor_handle       = osThreadNew(task_rtosMonitor, NULL, &task_rtosMonitor_attr);
    
#ifdef TASK_DEBUG_INFO
    drv_setTimerForInt(&timer50usHandle, TIM4, 20000, 6);
#endif
    
    /* USER CODE END RTOS_THREADS */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* 栈溢出/堆分配失败诊断现场，调试器暂停后可直接查看 */
volatile char    *g_overflow_task_name = NULL;   /* 溢出任务名 */
volatile uint32_t g_overflow_task      = 0;       /* 溢出任务句柄 */

/**
  * @brief  FreeRTOS 栈溢出回调（configCHECK_FOR_STACK_OVERFLOW=2 时启用）
  * @note   在任务切换上下文中被调用，pcTaskName 指向溢出任务名
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    g_overflow_task      = (uint32_t)xTask;
    g_overflow_task_name = pcTaskName;
    SEGGER_RTT_printf(0, "STACK OVERFLOW in task: %s", pcTaskName);
    taskDISABLE_INTERRUPTS();
    for (;;)
    {}
}

/**
  * @brief  FreeRTOS 堆分配失败回调（configUSE_MALLOC_FAILED_HOOK=1 时启用）
  * @note   heap_4 仅 4KB，pvPortMalloc 返回 NULL 时进入此处
  */
void vApplicationMallocFailedHook(void)
{
    size_t freeHeap = xPortGetFreeHeapSize();
    log_e("MALLOC FAILED, free heap = %u", (unsigned)freeHeap);
    taskDISABLE_INTERRUPTS();
    for (;;)
    {}
}

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
    uint32_t anchorRxLastTick = 0;
    static uint8_t can_buf[5] = {0};
    for (;;)
    {
        dwDistance_t *distance = get_the_local_structure_of_dis();

        // 阻塞等待本侧最小距离到来，无超时
        osStatus_t status = osMessageQueueGet(queue_minimalDis, &distance->disMsg[0], 0, osWaitForever);
        // drain：丢弃队列中积压的旧消息，只保留最新一条
        {
            outDistance_t newer;
            while (osMessageQueueGet(queue_minimalDis, &newer, 0, 0) == osOK)
            {
                distance->disMsg[0] = newer;
            }
        }
        if ((status == osOK) && (distance->disMsg[0].dis_class == ANCHOR_SELF_DIS))
        {   // 存储最小距离，通过CAN发送出去 store the min distance, send by can
            anchorSelfDis = distance->disMsg[0].dis_value;
            anchorSelfIdx = distance->disMsg[0].dis_index;

            split32to8(anchorSelfDis, can_buf);
            can_buf[4] = anchorSelfIdx;
            dev_canSendMsg(CAN_EXT_ID_DIS, can_buf, ARRAY_LENGTH(can_buf));             // send used can directly
        }
        else
        {
            anchorSelfDis = 2000000;
            anchorSelfIdx = 0xFF;
        }

        // 非阻塞取CAN距离：有新数据就更新（drain取最新），否则保留上次有效值，超2s才清零
        status = osMessageQueueGet(queue_canRxDis, &distance->disMsg[1], 0, 0);
        if (status == osOK)
        {
            outDistance_t newer_can;
            while (osMessageQueueGet(queue_canRxDis, &newer_can, 0, 0) == osOK)
            {
                distance->disMsg[1] = newer_can;
            }
        }
        if ((status == osOK) && (distance->disMsg[1].dis_class == ANCHOR_OTHER_DIS))
        {   // 从CAN接收中断中获取对侧基站最小距离
            anchorRxDis = distance->disMsg[1].dis_value;        // 获取接收的距离
            anchorRxIdx = distance->disMsg[1].dis_index;
            anchorRxLastTick = osKernelGetTickCount();
        }
        else if (osKernelGetTickCount() - anchorRxLastTick > 2000U)
        {   // 超过2s未收到CAN距离，视为对侧基站失联
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
            alarm_event_t event = final_minDistance_get;
            osMessageQueuePut(queue_alarm, &event, 0, 0);
        }
        else
        {   // 没有有效标签距离传入，关闭所有报警指示灯，最终距离为自身距离
            anchorFinalDis = anchorSelfDis;
            anchorFinalIdx = anchorSelfIdx;
            voiceOutputDis = -1;
            voiceOutputIndex = 0xFF;
            dev_ledOff(ACROSS_LED);
            dev_ledOff(ONSIDE_LED);
            alarm_event_t event = no_final_minDistance;
            osMessageQueuePut(queue_alarm, &event, 0, 0);
        }

        {
            static uint32_t lastFusionLogTick = 0;
            uint32_t now = osKernelGetTickCount();
            if ((now - lastFusionLogTick) >= 1000)
            {
                lastFusionLogTick = now;
                if (anchorRxIdx == 0xFF)
                {
                    log_d("self:tag-%u %.2fm | remote:N/A | final:tag-%u %.2fm",
                           anchorSelfIdx, (float)anchorSelfDis / 1000.0f,
                           anchorFinalIdx, (float)anchorFinalDis / 1000.0f);
                }
                else
                {
                    log_d("self:tag-%u %.2fm | remote:tag-%u %.2fm | final:tag-%u %.2fm",
                           anchorSelfIdx, (float)anchorSelfDis / 1000.0f,
                           anchorRxIdx, (float)anchorRxDis / 1000.0f,
                           anchorFinalIdx, (float)anchorFinalDis / 1000.0f);
                }
            }
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
                alarm_event_t evt = dis_changed_alarm;
                osMessageQueuePut(queue_alarm, &evt, 0, 0);
                log_d("tag distance change up to threshold.");
            }
            prevVoiceOutputDis = currentVoiceOutputDis;
        }
        // 设置监控周期
        tick += 1500U;
        osDelayUntil(tick);
    }
}

#if MODULE_ALARM_ENABLE
/**
 * @brief  根据距离值计算报警等级
 */
static alarm_state_t alarm_levelFromDist(int32_t dist_mm)
{
    if (dist_mm <= 0 || dist_mm >= ALARM_DIST_FAR)
    {
        return ALARM_LEVEL_0;
    }
    if (dist_mm < ALARM_DIST_DANGER)
    {
        return ALARM_LEVEL_3;
    }
    if (dist_mm < ALARM_DIST_NEAR)
    {
        return ALARM_LEVEL_2;
    }
    return ALARM_LEVEL_1;
}

/**
 * @brief  根据等级执行对应的报警输出（语音 + 蜂鸣器 + LED）
 */
static void alarm_applyLevel(alarm_state_t level, char *voice_buf)
{
    switch (level)
    {
    case ALARM_LEVEL_0:
        dev_buzzerClose(BUZZER);
        if (voicePlaying)
        {
            dev_jq8400Stop();
            voicePlaying = false;
        }
        break;

    case ALARM_LEVEL_1:
        dev_buzzerClose(BUZZER);
        voiceIntervalMs = VOICE_INTERVAL_L1;
        if (!voicePlaying)
        {
            outputVoiceBuildAndPlay(voiceOutputIndex, voiceOutputDis, voice_buf);
            voiceStartTick = osKernelGetTickCount();
            voicePlaying = true;
        }
        break;

    case ALARM_LEVEL_2:
        dev_buzzerClose(BUZZER);
        voiceIntervalMs = VOICE_INTERVAL_L2;
        if (!voicePlaying)
        {
            outputVoiceBuildAndPlay(voiceOutputIndex, voiceOutputDis, voice_buf);
            voiceStartTick = osKernelGetTickCount();
            voicePlaying = true;
        }
        break;

    case ALARM_LEVEL_3:
        dev_buzzerOpen(BUZZER);
        voiceIntervalMs = VOICE_INTERVAL_L3;
        if (!voicePlaying)
        {
            outputVoiceBuildAndPlay(voiceOutputIndex, voiceOutputDis, voice_buf);
            voiceStartTick = osKernelGetTickCount();
            voicePlaying = true;
        }
        break;

    case ALARM_MUTED:
        dev_buzzerClose(BUZZER);
        if (voicePlaying)
        {
            dev_jq8400Stop();
            voicePlaying = false;
        }
        break;
    }
}
#endif /* MODULE_ALARM_ENABLE */

/**
  * @brief  报警事件处理任务：接收事件队列，执行分级报警
  * @param  void *arg
  * @retval none
  */
void task_eventHandler(void *arg)
{
    UNUSED(arg);
#if MODULE_ALARM_ENABLE
    static char voice_buf[10];
#endif
#if MODULE_CAN_ENABLE
    static outDistance_t rxMsg = {0};
    static int32_t anchorReceiveDis = 2000000;
#endif

    for (;;)
    {
        alarm_event_t alarm_event;
        osMessageQueueGet(queue_alarm, &alarm_event, 0, osWaitForever);
        switch (alarm_event)
        {
#if MODULE_ALARM_ENABLE
        case final_minDistance_get:
            if (cur_alarmState != ALARM_MUTED)
            {
                alarm_state_t newLevel = alarm_levelFromDist(voiceOutputDis);
                if (newLevel != cur_alarmState)
                {
                    log_d("alarm level %d -> %d, dis=%d",
                           cur_alarmState, newLevel, voiceOutputDis);
                }
                cur_alarmState = newLevel;
                alarm_applyLevel(cur_alarmState, voice_buf);
            }
            break;

        case no_final_minDistance:
            cur_alarmState = ALARM_LEVEL_0;
            alarm_applyLevel(ALARM_LEVEL_0, voice_buf);
            dev_ledOff(ONSIDE_LED);
            dev_ledOff(ACROSS_LED);
            break;

        case pauseButton_longPress_alarm:
        case switchButton_alarm:
        case dis_changed_alarm:
            if (voiceOutputIndex == 0xFF)
            {
                /* 无有效标签，不处理 */
            }
            else
            {
                /* 从静音中恢复，根据当前距离重新判定等级 */
                alarm_state_t newLevel = alarm_levelFromDist(voiceOutputDis);
                if (newLevel != ALARM_LEVEL_0)
                {
                    cur_alarmState = newLevel;
                    alarm_applyLevel(cur_alarmState, voice_buf);
                }
            }
            break;

        case pauseButton_click_mute:
        case switchButton_mute:
            cur_alarmState = ALARM_MUTED;
            alarm_applyLevel(ALARM_MUTED, voice_buf);
            break;

        case switchButton_left_rotation: {
            uint8_t can_tmp1[5];
            memcpy(can_tmp1, can_alarm, 5);
            dev_canSendMsg(CAN_EXT_ID_BUTTON, can_tmp1, ARRAY_LENGTH(can_alarm));
            log_d("get msg switchButton_left_rotation");
            break;
        }

        case switchButton_right_rotation: {
            uint8_t can_tmp2[5];
            memcpy(can_tmp2, can_mute, 5);
            dev_canSendMsg(CAN_EXT_ID_BUTTON, can_tmp2, ARRAY_LENGTH(can_mute));
            log_d("get msg switchButton_right_rotation");
            break;
        }
#endif /* MODULE_ALARM_ENABLE */

#if MODULE_CAN_ENABLE
        case can_received:
            if ((RxHeader.ExtId == CAN_EXT_ID_DIS) && (RxHeader.IDE == CAN_ID_EXT))
            {
                anchorReceiveDis = combine8to32(RxData);
                rxMsg.dis_class = ANCHOR_OTHER_DIS;
                rxMsg.dis_value = anchorReceiveDis;
                rxMsg.dis_index = RxData[4];
                osMessageQueuePut(queue_canRxDis, &rxMsg, 0, 0);
                dev_ledBlink(CAN_RX_LED);
            }
            else if ((RxHeader.ExtId == CAN_EXT_ID_BUTTON) && (RxHeader.IDE == CAN_ID_EXT))
            {
#if MODULE_ALARM_ENABLE
                if (RxData[0] == 'C')
                {
                    if (RxData[3] == 'A')
                    {
                        alarm_event_t event = switchButton_alarm;
                        osMessageQueuePut(queue_alarm, &event, 0, 0);
                        log_d("receive can_alarm, open voice");
                    }
                    else if (RxData[3] == 'M')
                    {
                        alarm_event_t event = switchButton_mute;
                        osMessageQueuePut(queue_alarm, &event, 0, 0);
                        log_d("receive can_mute, close voice");
                    }
                }
#endif
            }
            break;
#endif /* MODULE_CAN_ENABLE */

        default:
            break;
        }

#if MODULE_ALARM_ENABLE
        /* 语音播报间隔控制：间隔到、且 BUSY 引脚显示已播完，才清标志让下次事件重新播报。
           叠加 BUSY 判断可避免上一条语音还没播完就被下一条打断/重叠；
           PD9 下拉，引脚悬空(未接/异常)时读到空闲，退化为原有纯间隔行为，不会卡住复播。 */
        if (voicePlaying)
        {
            if ((osKernelGetTickCount() - voiceStartTick) >= voiceIntervalMs
                && !dev_jq8400IsBusy())
            {
                voicePlaying = false;
            }
        }
#endif
    }
}

void task_emmcTest(void *arg)
{
    UNUSED(arg);
    // app_emmcReadWriteDemo();
    // app_usbMscInit();
    for (;;)
    {
        osDelay(2000);
    }
}

/**
  * @brief  此任务为RTOS最小单元测试，验证RTOS的基本功能是否正常
  * @param  void *arg
  * @retval none
  */
void task_test(void *arg)
{
    UNUSED(arg);
    for (;;)
    {
        osDelay(2000);
    }
}

/**
  * @brief  此任务用于RTOS的系统DEBUG监控，周期性输出各个任务的剩余堆栈空间和运行时间统计等信息，辅助开发调试和性能优化
  * @param  void *arg
  * @retval none
  */
void task_rtosMonitor(void *arg)
{
    UNUSED(arg);
    extern osThreadId_t task_swoOutput_handle;
    char taskListBuffer[512];               // 文档推荐每个任务信息大概占用40个bytes
    for (;;)
    {
        // osThreadFlagsWait(0x01, osFlagsWaitAll, osWaitForever);
        log_d("=================usage percentage=========================");
        // log_i("task_name    state priority stack_size num_of_task");
        // vTaskList(taskListBuffer);
        // log_i("%s", taskListBuffer);

        size_t freeHeap = xPortGetFreeHeapSize();
        size_t minHeap  = xPortGetMinimumEverFreeHeapSize();
        log_d("Heap free: %d, min ever: %d\r\n", freeHeap, minHeap);

        log_i("task_name    run_time_counter   use_percentage");
        vTaskGetRunTimeStats(taskListBuffer);
        log_i("%s", taskListBuffer);
        // log_i("==========================================");
        // dev_ledBlink(ONSIDE_LED);    

        log_d("===================stack monitor===================");
        log_d("task_uwb stack hwm: %d bytes", uxTaskGetStackHighWaterMark(task_uwb_handle));
        log_d("task_anchorDisHandling stack hwm: %d bytes", uxTaskGetStackHighWaterMark(task_anchorDisHandling_handle));
        log_d("task_minHeapManage stack hwm: %d bytes", uxTaskGetStackHighWaterMark(task_minHeapManage_handle));
        log_d("task_getMinDis stack hwm: %d bytes", uxTaskGetStackHighWaterMark(task_getMinDis_handle));
        log_d("task_tagDisMonitor stack hwm: %d bytes", uxTaskGetStackHighWaterMark(task_tagDisMonitor_handle));
        log_d("task_eventProcess stack hwm: %d bytes", uxTaskGetStackHighWaterMark(task_eventProcess_handle));
        log_d("task_gnssSyncTime stack hwm: %d bytes", uxTaskGetStackHighWaterMark(task_gnssSyncTime_handle));
        log_d("task_eth stack hwm: %d bytes", uxTaskGetStackHighWaterMark(task_eth_handle));
        log_d("task_rtosMonitor stack hwm: %d bytes", uxTaskGetStackHighWaterMark(task_rtosMonitor_handle));
        log_d("task_swoOutput stack hwm: %d bytes", uxTaskGetStackHighWaterMark(task_swoOutput_handle));
        osDelay(60000);
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
        alarm_event_t evt = can_received;
        osMessageQueuePut(queue_alarm, &evt, 0, 0);
    }
    else
    {
        Error_Handler();
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
        extern osSemaphoreId_t sema_tagDistClear;
        if (osSemaphoreRelease(sema_tagDistClear) == osOK)
        {
            // log_d("tdmaCycleTimer_CallBack");
        }
    }
    else if (htim->Instance == TIM4)
    {
#ifdef TASK_DEBUG_INFO
        ulHighFrequencyTimerTicks++;
#endif
        DHCP_time_handler();
    }
}

extern osSemaphoreId_t sema_uwbInt;
/* @fn		HAL_GPIO_EXTI_Callback
 * @brief	IRQ HAL call-back for all EXTI configured lines
 * 			i.e. DW_RESET_Pin and DW_IRQn_Pin
 * */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == W5500_INT_PIN)
    {
        osSemaphoreRelease(sema_w5500Int);
    }
#if defined(USE_DW3000)
    else if (GPIO_Pin == Dw3000_RSTn_Pin)
    {
        board_dw3000SetSignalReset();
    }
    else if (GPIO_Pin == Dw3000_IRQ_Pin)
    {
        osSemaphoreRelease(sema_uwbInt);
    }
#else
    else if (GPIO_Pin == Dw1000_RSTn_Pin)
    {
        board_dw1000SetSignalReset();
    }
    else if (GPIO_Pin == Dw1000_IRQ_Pin)
    {
        osSemaphoreRelease(sema_uwbInt);
    }
#endif
}
/*************************************************static function*************************************************/
/**
  * @brief  根据输入的标签ID和距离构建语音文件名，并调用组合播报函数进行播报，距离小于100m按米播报，距离大于100m按5米播报
  * @param  cloestTagId: 最近标签的ID
  * @param  cloestTagDis: 最近标签的距离，单位mm
  * @param  buffer: 用于存储构建的语音文件名的缓冲区，建议大小至少为10字节
  *         
  * @retval None
  */
#if MODULE_ALARM_ENABLE
static void outputVoiceBuildAndPlay(uint8_t cloestTagId, int32_t cloestTagDis, char buffer[10])
{
    int cnt = 0;                              // 记录buffer中语音片段数量
    int tens_digit = 0;
    int ones_digit = 0;
    char head_letter = '\0';

    int32_t cloestTagDis_meter = cloestTagDis / 1000;

    if (cloestTagDis_meter < 10)              // 距离小于10m，只报距离不报ID（紧急）
    {
        ones_digit = cloestTagDis_meter % 10;
        cnt = sprintf(buffer, "A%d", ones_digit);
    }
    else if (cloestTagDis_meter <= 100)       // 10~100m，按米播报（ID+距离）
    {
        tens_digit = cloestTagDis_meter / 10;
        ones_digit = cloestTagDis_meter % 10;
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
    /*
        调用组合播报函数进行播报，进行数据包构建，buffer中存储的格式为 "ID"+"10位数字对应的字母"+"个位数字"，例如 "03A5" 表示ID为3，距离为15米；"12C0" 表示ID为12，距离为20米；"07D5" 表示ID为7，距离为35米，以此类推
    */
    dev_jq8400CombinePlay((char*)buffer, cnt);
}
#endif /* MODULE_ALARM_ENABLE */
/* USER CODE END Application */
