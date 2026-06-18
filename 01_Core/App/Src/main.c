/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include <stdarg.h>

#include "main.h"
#include "app_config.h"
#include "app_usb_msc.h"
#include "app_log_manage.h"
#include "dwt_delay.h"
#include "dw_instance.h"
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
// #include "iwdg.h"
#include "spi.h"
#include "usart.h"
#include "drv_timer.h"

#include "cmsis_os.h"
#include "elog.h"
#include "SEGGER_RTT.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
osThreadId_t task_swoOutput_handle;
static uint8_t task_swoOutput_buffer[1024];
StaticTask_t task_swoOutput_cb;
const osThreadAttr_t task_swoOutput_attr = {
    .name      = "task_swoOutput",
    .stack_mem = task_swoOutput_buffer, .stack_size = sizeof(task_swoOutput_buffer), 
    .cb_mem = &task_swoOutput_cb, .cb_size = sizeof(task_swoOutput_cb),
    .priority  = (osPriority_t) osPriorityRealtime5,
};

osThreadId_t task_main_handle;
static uint8_t task_main_buffer[1024];  /* 启动任务：初始化调用链较深，跑完即 osThreadExit 释放 */
StaticTask_t task_main_cb;
const osThreadAttr_t task_main_attr = {
    .name      = "task_main",
    .stack_mem = task_main_buffer, .stack_size = sizeof(task_main_buffer), 
    .cb_mem = &task_main_cb, .cb_size = sizeof(task_main_cb),
    .priority  = (osPriority_t) osPriorityRealtime4,
};

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
void swo_init(void);
void task_swoOutPut(void *arg);
void task_mainProcess(void* arg);

/*
    最开始先初始化时钟，hal

    创建一个system main task，system main task 处理函数先进行一系列外设初始化，信号量，队列创建

    外部表现： 语音 报警灯光 蜂鸣器

    内部：uwb task，距离处理，日志，GNSS

    外部信号： 按键 ，旋钮
*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{   // 底层初始化 --> flash，时钟，DWT，外设接口
    HAL_Init();                 // Reset of all peripherals, Initializes the Flash interface and the Systick.
    DWT_Init();                 // 初始化 DWT，用于延时功能
    SystemClock_Config();       // Configure the system clock
    // swo_init();              // SWO LOG 输出端口初始化（改用 SEGGER RTT，保留函数体方便回滚）
    SEGGER_RTT_Init();          // 主动初始化 RTT 控制块（魔术字立即写入 BSS，便于 OpenOCD/Cortex-Debug 上电搜索）
    SEGGER_RTT_WriteString(0, "\r\nSystem and RTT boot.\r\n");
    MX_SPI1_Init();             // 初始化SPI1 配置，读写DMA通道，DMA中断，供uwb模组传输
    MX_UART4_Init();            // 初始化UART4 配置，读DMA通道，串口中断，供jq8400语音模组传输

    osKernelInitialize();       // Init scheduler
    task_main_handle = osThreadNew(task_mainProcess, NULL, &task_main_attr);
    osKernelStart();            // Start scheduler
    /* We should never get here as control is now taken by the scheduler */
    while (1)
    {}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;                         // 设置PLL48CK，为48MHz，这里设置为7，336除以7为48                
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 
                                | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    // log_e("111");      // 计划在此打印出问题的文件名以及行数
    while (1)
    {}
    /* USER CODE END Error_Handler_Debug */
}

void task_mainProcess(void* arg)
{
    UNUSED(arg);
    // the Initializes sequence can not be changed
#if MODULE_CAN_ENABLE
    dev_canInit();
#endif
    dev_ledAndRT9013Init();
#if MODULE_ALARM_ENABLE
    dev_buzzerInit(BUZZER);
#endif
    dev_jq8400Init();
    dev_jq8400RandomPathPlay(JQ8X00_FLASH, "jzkj");   // 开机语音
    dev_buttonInit(SWITCH_KEY);
    dev_buttonInit(PAUSE_KEY);
    dev_buttonMultiInit();
    dev_rx8130ceInit();
#if MODULE_CAN_ENABLE
    dev_canStartRx();
#endif
#if MODULE_GNSS_ENABLE
    dev_gnssModInit();
#endif
    dev_dipInit(ANCHOR_ID0);
    dev_dipInit(ANCHOR_ID1);
    dev_dipInit(ANCHOR_ID2);
    MX_FREERTOS_Init();

    elog_componentInit();
    task_swoOutput_handle = osThreadNew(&task_swoOutPut, NULL, &task_swoOutput_attr);
#if MODULE_USB_MSC_ENABLE
    app_usbMscInit();
#endif
#if MODULE_LOG_MANAGE_ENABLE
    log_mgr_init();
#endif
    osThreadExit();
}

/************************************************* swo log **************************************************/
void swo_init(void)
{
    // __HAL_RCC_GPIOB_CLK_ENABLE();
    // GPIO_InitTypeDef GPIO_InitStructure = {0};
    // GPIO_InitStructure.Pin = SWO_GPIO_PIN;
    // GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStructure.Pull = GPIO_NOPULL;
    // GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    // GPIO_InitStructure.Alternate = GPIO_AF0_TRACE;
    // HAL_GPIO_Init(SWO_GPIO_PORT, &GPIO_InitStructure);

    // // 打开 SWO 功能 LAR(Lock Acess reg) TCR(Trace control reg) TER(Trace Enable reg)
    // CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable trace
    // ITM->LAR  = 0xC5ACCE55;                          // Unlock ITM
    // ITM->TCR  = ITM_TCR_ITMENA_Msk | ITM_TCR_SYNCENA_Msk | ITM_TCR_TSENA_Msk;
    // ITM->TER  = 0x1;                                 // Enable stimulus port 0   
}

void task_swoOutPut(void *arg)
{
    UNUSED(arg);
    static uint8_t temp_buf[1024];
    size_t num_bytes;

    for (;;)
    {
        extern StreamBufferHandle_t log_streamBufferHandle;
        // 阻塞等待 streambuffer 数据，转发到 SEGGER RTT 通道 0
        num_bytes = xStreamBufferReceive(log_streamBufferHandle, temp_buf, sizeof(temp_buf), 
                    portMAX_DELAY);
        SEGGER_RTT_Write(0, temp_buf, num_bytes);

        // --- 原 SWO/ITM 输出路径（保留以便回滚）---
        // size_t idx = 0;
        // while (idx < num_bytes)
        // {
        //     if (ITM->TCR & ITM_TCR_ITMENA_Msk)
        //     {
        //         if (ITM->PORT[0].u32 & 1)
        //         {
        //             ITM->PORT[0].u8 = temp_buf[idx++];
        //         }
        //         else
        //         {
        //             taskYIELD();
        //         }
        //     }
        // }
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    printf("Assert failed at %s:%lu", file, line);
  /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */
