
#include "iwdg.h"

IWDG_HandleTypeDef hiwdg;


void IWDG_Init(void)
{
    /* Set counter reload value to obtain 500ms IWDG TimeOut.
     IWDG counter clock Frequency = LsiFreq / 32
     Counter Reload Value = 500ms / IWDG counter clock period
                          = 0.5s / (32/LsiFreq)
                          = LsiFreq / (32 * 2)
                          = LsiFreq / 64 */
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg.Init.Reload = 1000;

    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
}
