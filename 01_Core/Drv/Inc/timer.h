#ifndef __TIMER_H__
#define __TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"
#include "com_Multitimer.h"

void MX_TIM2_Init(uint32_t period_ms);
void MX_TIM3_Init(void);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);

void multiTimer_init(void);
uint64_t platform_Ticks_Get(void);
void timer_compareDistance_callBack(MultiTimer* timer, void* userData);
#ifdef __cplusplus
}
#endif


#endif      /* __TIMER_H__ */
