#ifndef __DRV_TIMER_H__
#define __DRV_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"
// #include "com_Multitimer.h"

void MX_TIM2_Init(uint32_t period_ms);
void MX_TIM3_Init(void);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);
void drv_enaleRCCofTimer(TIM_TypeDef* timer);
void drv_setTimerForInt(TIM_HandleTypeDef* timHandler, TIM_TypeDef* timer, uint32_t freq, uint8_t interruptPriority);

#ifdef __cplusplus
}
#endif

#endif      /* __TIMER_H__ */
