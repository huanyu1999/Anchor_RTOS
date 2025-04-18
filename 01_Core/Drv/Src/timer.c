#include "timer.h"
#include "usart.h"
#include "com_MultiTimer.h"
#include "instance.h"

TIM_HandleTypeDef htimer2;
TIM_HandleTypeDef htimer3;

void MX_TIM2_Init(uint32_t period_ms)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    uint32_t period_value = (period_ms * 10000 / 1000) - 1;

    /* Compute the prescaler value to have TIM2 counter clock equal to 1 KHz */
    uint32_t uwPrescalerValue = (uint32_t) ((SystemCoreClock / 2) / 10000) - 1;
    
    /* Set TIMx instance */
    htimer2.Instance = TIM2;
    
    htimer2.Init.Period = period_value;                 // 500ms 中断一次
    htimer2.Init.Prescaler = uwPrescalerValue;
    htimer2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htimer2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htimer2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if(HAL_TIM_Base_Init(&htimer2) != HAL_OK)
    {
        Error_Handler();
    }
    
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htimer2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htimer2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* Start Channel1 */
    if(HAL_TIM_Base_Start_IT(&htimer2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* TIM3 init function period 5ms, for button ticks */
void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    
    /* Compute the prescaler value to have TIM2 counter clock equal to 1 KHz */
    uint32_t uwPrescalerValue = (uint32_t) ((SystemCoreClock / 2) / 10000) - 1;
    
    /* Set TIMx instance */
    htimer3.Instance = TIM3;
    
    /* Initialize TIM3 peripheral as follow:
       + Period = 1000 - 1
       + Prescaler = ((SystemCoreClock/2)/1000) - 1
       + ClockDivision = 0
       + Counter direction = Up
    */
    htimer3.Init.Period = 100 - 1;                 // 5ms 中断一次
    htimer3.Init.Prescaler = uwPrescalerValue;
    htimer3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htimer3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htimer3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if(HAL_TIM_Base_Init(&htimer3) != HAL_OK)
    {
        Error_Handler();
    }
    
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htimer3, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htimer3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* Start Channel1 */
    if(HAL_TIM_Base_Start_IT(&htimer3) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief TIM MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* TIMx Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    /*##-2- Configure the NVIC for TIMx ########################################*/
    /* Set Interrupt Group Priority */ 
    HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
    HAL_NVIC_SetPriority(TIM3_IRQn, 6, 0);

    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/**********************************************multi timer init**************************************************** */
// MultiTimer timer_test;

// void multiTimer_init(void)
// {
//     multiTimerInstall(platform_Ticks_Get);
//     // multiTimerStart(&timer_test, 400, timer_test_callBack, NULL);
// }

// uint64_t platform_Ticks_Get(void)
// {
//     return (uint64_t)HAL_GetTick();
// }

// void timer_test_callBack(MultiTimer* timer, void* userData)
// {

//     printf_use_dma("timer_test_callBack\r\n");
//     multiTimerStart(&timer_test, 400, timer_test_callBack, NULL);
// }
