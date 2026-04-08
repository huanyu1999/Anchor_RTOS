#include "drv_timer.h"
// #include "com_MultiTimer.h"

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
    
    htimer2.Init.Period = period_value;        
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
    UNUSED(htim);
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* TIMx Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    /*##-2- Configure the NVIC for TIMx ########################################*/
    /* Set Interrupt Group Priority */ 
    HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
    HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);

    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}


void drv_enaleRCCofTimer(TIM_TypeDef* timer)
{
    if (timer == TIM1) __HAL_RCC_TIM1_CLK_ENABLE();
    else if (timer == TIM8) __HAL_RCC_TIM8_CLK_ENABLE();
    else if (timer == TIM9) __HAL_RCC_TIM9_CLK_ENABLE();
    else if (timer == TIM10) __HAL_RCC_TIM10_CLK_ENABLE();
    else if (timer == TIM11) __HAL_RCC_TIM11_CLK_ENABLE();
    else if (timer == TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
    else if (timer == TIM3) __HAL_RCC_TIM3_CLK_ENABLE();
    else if (timer == TIM4) __HAL_RCC_TIM4_CLK_ENABLE();
    else if (timer == TIM5) __HAL_RCC_TIM5_CLK_ENABLE();
    else if (timer == TIM6) __HAL_RCC_TIM6_CLK_ENABLE();
    else if (timer == TIM7)  __HAL_RCC_TIM7_CLK_ENABLE();
    else if (timer == TIM12)  __HAL_RCC_TIM12_CLK_ENABLE();
    else if (timer == TIM13)  __HAL_RCC_TIM13_CLK_ENABLE();
    else if (timer == TIM14)  __HAL_RCC_TIM14_CLK_ENABLE();
}

/**
  * @brief  配置TIMER以及NVIC，用于简单定时中断，开启定时器中断，使能中断
  * @param  TIM_HandleTypeDef* timHandler: 定时器配置句柄
  * @param  TIM_TypeDef* timer: 需要配置的定时器
  * @param  uint32_t freq: 定时器输出频率
  * @param  uint8_t interruptPriority: 中断优先级
  * @retval None
  */
void drv_setTimerForInt(TIM_HandleTypeDef* timHandler, TIM_TypeDef* timer, uint32_t freq, uint8_t interruptPriority)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    uint16_t period;
    uint16_t prescaler;
    uint32_t timerClk;

    //使能TIM时钟
    drv_enaleRCCofTimer(timer);

    if(freq == 0)
    {
        // 关闭定时器输出
    }

    /*-----------------------------------------------------------------------
		main.c 中的SystemClock_Config函数对时钟配置如下：

		HCLK = SYSCLK / 1     (AHB1Periph)
		PCLK2 = HCLK / 2      (APB2Periph)
		PCLK1 = HCLK / 4      (APB1Periph)

		因为APB1 prescaler != 1, 所以 timerClk = PCLK1 x 2 = SystemCoreClock / 2;
		因为APB1 prescaler != 1, 所以 timerClk = PCLK2 x 2 = SystemCoreClock;

		APB1 总线上的timer有 TIM2, TIM3 ,TIM4, TIM5, TIM6, TIM7, TIM12, TIM13,TIM14
		APB2 总线上的timer有 TIM1, TIM8 ,TIM9, TIM10, TIM11
	----------------------------------------------------------------------- */
	if ((timer == TIM1) || (timer == TIM8) || (timer == TIM9) || (timer == TIM10) || (timer == TIM11))
	{
		timerClk = SystemCoreClock;
	}
	else
	{
		timerClk = SystemCoreClock / 2;
	}

    if (freq < 100) 
    {
        prescaler = 10000 - 1;
        period = ((timerClk / 10000) / freq) - 1;
    } 
    else if (freq < 3000)
    {
        prescaler = 100 - 1;
        period = ((timerClk / 100) / freq) - 1;
    }
    else
    {
        prescaler = 0;
        period = (timerClk / freq) - 1;
    }
   
    timHandler->Instance = timer;
    timHandler->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    timHandler->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timHandler->Init.CounterMode = TIM_COUNTERMODE_UP;
    timHandler->Init.Period = period; 
    timHandler->Init.Prescaler = prescaler;
    timHandler->Init.RepetitionCounter = 0x0000;
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIM_Base_Init(timHandler) != HAL_OK) 
    {
        Error_Handler(); 
    }
    if (HAL_TIM_ConfigClockSource(timHandler, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIMEx_MasterConfigSynchronization(timHandler, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    {   // 配置定时器定时更新中断
        uint8_t irqNum = 0;
        if ((timer == TIM1) || (timer == TIM10))
            irqNum = TIM1_UP_TIM10_IRQn;
        else if (timer == TIM2)
            irqNum = TIM2_IRQn;
        else if (timer == TIM3)
            irqNum = TIM3_IRQn;
        else if (timer == TIM4)
            irqNum = TIM4_IRQn;
        else if (timer == TIM5)
            irqNum = TIM5_IRQn;
        else if (timer == TIM6)
            irqNum = TIM6_DAC_IRQn;
        else if (timer == TIM7)
            irqNum = TIM7_IRQn;
        else if (timer == TIM7)
            irqNum = TIM7_IRQn;
        else if (timer == TIM7)
            irqNum = TIM7_IRQn;
        else if ((timer == TIM8) || (timer == TIM13))
            irqNum = TIM8_UP_TIM13_IRQn;
        else if (timer == TIM9)
            irqNum = TIM1_BRK_TIM9_IRQn;
        else if (timer == TIM11)
            irqNum = TIM1_TRG_COM_TIM11_IRQn;
        else if (timer == TIM12)
            irqNum = TIM8_BRK_TIM12_IRQn;
        else if (timer == TIM14)
            irqNum = TIM8_TRG_COM_TIM14_IRQn;

        HAL_NVIC_SetPriority(irqNum, interruptPriority, 0xF);
        HAL_NVIC_EnableIRQ(irqNum);
    }
    
    if(HAL_TIM_Base_Start_IT(timHandler) != HAL_OK)
    {
        Error_Handler();
    }
}
