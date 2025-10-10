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
#include "app_sdCard.h"
#include "instance.h"
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
#include "iwdg.h"
#include "spi.h"
#include "usart.h"
#include "drv_timer.h"

#include "cmsis_os.h"
#include "elog.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
void swo_init(void);
static void swo_putc(char ch);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    swo_init();

    /* Initialize all configured peripherals */
    MX_SPI1_Init();
    
    /* device init */
    dev_canInit();
    dev_canStartRx();
    dev_rx8130ceInit();
    dev_gnssModInit();
    dev_ledAndRT9013Init();
    dev_buzzerInit(BUZZER);
    dev_buttonInit(SWITCH_KEY);
    dev_buttonInit(PAUSE_KEY);
    dev_buttonTaskInit();
    dev_dipInit(ANCHOR_ID0);
    dev_dipInit(ANCHOR_ID1);
    dev_dipInit(ANCHOR_ID2);
    dev_dipInit(ANCHOR_ID3);
    dev_w5500Initialize();
    dev_jq8400CommandData(SetVolume, 23);       // 音量控制也可以通过上位机来确定
    dev_gnssModStartRx();                       // 上电后先同步时间
    dev_gnssModReceiveAndParse();
    
    uwb_init();

    /* Init scheduler */
    osKernelInitialize();

    /* Call init function for freertos objects (in freertos.c) */
    MX_FREERTOS_Init();

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    while (1) {}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /* Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {}
    /* USER CODE END Error_Handler_Debug */
}

void swo_init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.Pin = SWO_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Alternate = GPIO_AF0_TRACE;
    HAL_GPIO_Init(SWO_GPIO_PORT, &GPIO_InitStructure);

    // 打开 SWO 功能
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable trace
    ITM->LAR  = 0xC5ACCE55;                          // Unlock ITM
    ITM->TCR  = ITM_TCR_ITMENA_Msk | ITM_TCR_SYNCENA_Msk | ITM_TCR_TSENA_Msk;
    ITM->TER  = 0x1;                                  // Enable stimulus port 0
}

static void swo_putc(char ch) {
    if (ITM->TCR & ITM_TCR_ITMENA_Msk) {
        while (!(ITM->PORT[0].u32 & 1));
        ITM->PORT[0].u8 = ch;
    }
}

// SWO 格式化输出（最大支持256字节缓冲）
void swo_printf(const char *fmt, ...) {
    char buffer[256];  // 根据需要修改大小
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    for (char *p = buffer; *p; p++) {
        swo_putc(*p);
    }
}

void swo_logOutput(const char *data, size_t dataSize) {
    for (int i = 0; i < dataSize; i++) {
        swo_putc(*data++);
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
